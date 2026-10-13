"""One run: handles, run mode, recording, marks, teardown.

C9's SIM rung is the controller's own simulation mode, ON HARDWARE, and the
controller simulates NEITHER the pole NOR the hands. So a SIM run must skip
those two devices while still commanding every arm motion — and it must skip
them without changing the SHAPE of the program, because `custody.stage_facts`
requires one call per stage and `schedule` turns each non-scene call into a
Step. Nothing is elided here; only the DISPOSITION of two devices changes.

Import-safe by construction: no SDK import, no socket and no thread until
`__enter__`, so the 84 tests that import this package with no emulator present
keep working.
"""

from __future__ import annotations

import signal
import threading
import time
from typing import Callable, Dict, List, Optional, Sequence

from alix_dispatch import connect, limits, runmode
from alix_dispatch.backends import (ExecutionError, RecordingBackend,
                                    SdkBackend, _issue)
from alix_dispatch.runmode import RunModeError


class RunSession:
    """Owns everything a run needs, and gives all of it back on the way out."""

    def __init__(self, sides: Sequence[str], *, task: str = "",
                 mode: Optional[int] = None,
                 ips: Optional[Dict[str, str]] = None,
                 port: Optional[int] = None,
                 thread_mode: int = connect.THREAD_MODE_TRIPLE,
                 recorder=None,
                 line_speed: Optional[float] = None,
                 line_acc: Optional[float] = None,
                 angular_speed: Optional[float] = None,
                 joint_speed: Optional[Dict[int, float]] = None,
                 open_arm: Optional[Callable[..., object]] = None,
                 log: Callable[[str], None] = print) -> None:
        self.sides = list(sides)
        self.task = task
        self.mode = mode
        self.ips = dict(ips or {})
        self.port = connect.DEFAULT_PORT if port is None else port
        self.thread_mode = thread_mode
        self.recorder = recorder
        # The TCP constraint a commanded v% is a percentage OF. Without this
        # the config's `cleaning_line_speed_mps` reaches nothing: v=100 % is
        # 100 % of whatever the arm already held.
        self.line_speed = line_speed
        # The task's OWN acceleration, when it carries one. Without this the
        # session wrote `line_acc_for(v)` — C2's minimum — while the turn
        # policy computed every corner speed from the config's number. The
        # 2026-08-21 SIM run wrote 2.41 against a config saying 3.2, so each
        # turn was priced sqrt(3.2/2.41) = 1.15x faster than the arm could
        # hold. a_lat and the written acceleration have to be one value.
        self.line_acc = line_acc
        self.angular_speed = angular_speed
        #: {joint: deg/s} caps. Lowered only, verified, and restored — see
        #: limits.apply_joint_speed for why a joint above 180 is refused.
        self.joint_speed = dict(joint_speed or {})
        self._limits_before: Dict[str, object] = {}
        self._joints_before: Dict[str, object] = {}
        self._open_arm = open_arm or connect.open_arm
        self._log = log

        self.handles: Dict[str, object] = {}
        self._backends: Dict[str, SdkBackend] = {}
        self._originals = None
        self._observed: Dict[str, Optional[int]] = {}
        self._as_found_tools: Dict[str, object] = {}
        self._pole_assumed: Dict[str, float] = {}
        self._marks: List[dict] = []
        self._warnings: List[str] = []
        self._prev_signals: Dict[int, object] = {}
        self._torn = False
        self._issuing = False
        # Assigned in __enter__ but INITIALISED HERE so it always exists. The
        # upstream equivalent is assigned mid-try and read in a finally, so any
        # exception between the two raises NameError, gets swallowed, and the
        # record silently loses `sim` — which four analysers gate on.
        self._sim_mode: Optional[bool] = None

    # ── properties the backend guard reads ────────────────────────────────
    @property
    def sim_mode(self) -> bool:
        return bool(self._sim_mode)

    @property
    def issuing(self) -> bool:
        return self._issuing

    # ── lifecycle ─────────────────────────────────────────────────────────
    def __enter__(self) -> "RunSession":
        try:
            for side in self.sides:
                handle = self._open_arm(side, self.ips.get(side), self.port,
                                        thread_mode=self.thread_mode)
                if handle is None:
                    raise RunModeError(
                        f"hardware not reachable at "
                        f"{self.ips.get(side) or connect.DEFAULT_IPS.get(side)} "
                        f"for the {side} arm — C9's SIM and REAL rungs both "
                        "require hardware; this is a SKIP, not a failure")
                self.handles[side] = handle

            robots = {s: h.robot for s, h in self.handles.items()}

            # As-found tool frame, captured BEFORE anything changes it, so
            # teardown can put back exactly what was there. The 2026-08-08 leak
            # persisted across a reboot because nothing captured this.
            for side, robot in robots.items():
                try:
                    self._as_found_tools[side] = robot.rm_get_current_tool_frame()
                except Exception:
                    self._as_found_tools[side] = None

            # LIMITS BEFORE MODE, and both before any motion. The pair must
            # satisfy acc >= 3 x speed or the controller refuses with a bare
            # ret=1 and keeps the old value, so `line_acc_for` carries the
            # margin that survives the 2-decimal wire format.
            if self.line_speed:
                want = {"line_speed": float(self.line_speed),
                        "line_acc": self._line_acc()}
                if self.angular_speed:
                    want["angular_speed"] = float(self.angular_speed)
                    want["angular_acc"] = limits.angular_acc_for(self.angular_speed)
                for side, robot in robots.items():
                    self._limits_before[side] = limits.apply(robot, **want)
                    self._log(f"  [INFO] {side}: {limits.describe(limits.read(robot), 100)}")

            if self.joint_speed:
                for side, robot in robots.items():
                    self._joints_before[side] = limits.apply_joint_speed(
                        robot, self.joint_speed)
                    self._log(f"  [INFO] {side}: joint caps "
                              + ", ".join(f"J{j}={v} deg/s"
                                          for j, v in sorted(self.joint_speed.items())))

            originals = runmode.apply_run_mode(self.mode, robots)
            if originals is None:
                raise RunModeError(
                    f"could not engage {runmode.mode_label(self.mode)} on every "
                    "arm — refusing to dispatch. A SIM request that silently "
                    "stayed REAL would move real metal")
            self._originals = originals
            runmode.report_run_modes(robots)

            self._observed = {s: runmode.read_run_mode(r)[1]
                              for s, r in robots.items()}
            self._sim_mode = runmode.resolve_sim(self.mode, self._observed)

            for side, handle in self.handles.items():
                backend = SdkBackend(handle.robot, side,
                                     thread_mode=self.thread_mode)
                backend._sim_guard = self
                self._backends[side] = backend

            self._install_signal_handlers()

            if self.recorder is not None:
                # The recorder needs handles, addresses and a UDP target that
                # only exist now. Filling them HERE keeps the caller from
                # having to construct a half-live recorder and start it itself
                # — which double-started it, and the process-global guard then
                # refused the second call.
                self.recorder.robots = {s: h.robot
                                        for s, h in self.handles.items()}
                self.recorder.arm_ips = {s: h.ip for s, h in self.handles.items()}
                self.recorder._by_ip = {h.ip: s for s, h in self.handles.items()}
                self.recorder.host_ips = {s: h.host_ip
                                          for s, h in self.handles.items()}
                self.recorder._rows = {s: [] for s in self.handles}
                self.recorder.emulated = any(
                    connect.sdk_provenance(h.robot) == "emulated"
                    for h in self.handles.values())
                ok, detail = self.recorder.start()
                self._log(f"  [{'INFO' if ok else 'FAIL'}] {detail}")
                if not ok:
                    raise RunModeError(f"recording could not start: {detail}")

            if self._sim_mode:
                self._log("  [INFO] SIMULATION: the arm executes; pole and hand "
                          "stages are ASSUMED successful (the controller models "
                          "neither)")
            return self
        except BaseException:
            self._teardown()
            raise

    def __exit__(self, exc_type, exc, tb) -> bool:
        self._teardown()
        return False        # never swallow: a run's failure is the result

    def _install_signal_handlers(self) -> None:
        """Best effort. `signal.signal` raises off the main thread, and
        `schedule.execute_dual` is threaded, so guard rather than assume."""
        if threading.current_thread() is not threading.main_thread():
            return
        for sig in (signal.SIGTERM, getattr(signal, "SIGHUP", None)):
            if sig is None:
                continue
            try:
                self._prev_signals[sig] = signal.getsignal(sig)
                signal.signal(sig, self._on_signal)
            except Exception:
                pass

    def _on_signal(self, signum, frame):
        self._log(f"  [WARN] signal {signum} — tearing down the session")
        self._teardown()
        raise KeyboardInterrupt(f"signal {signum}")

    def _teardown(self) -> None:
        """Idempotent, never raises. A raise here would replace the real error."""
        if self._torn:
            return
        self._torn = True
        robots = {s: h.robot for s, h in self.handles.items()}

        # TOOL FRAMES FIRST — the only ordering with a stated reason: the tool
        # frame is global controller state shared with the Web GUI and every
        # other program, and a leaked one persists across a reboot.
        for side, as_found in self._as_found_tools.items():
            name = None
            if isinstance(as_found, (tuple, list)) and len(as_found) > 1:
                frame = as_found[1]
                name = (frame.get("name") if isinstance(frame, dict)
                        else getattr(frame, "name", None))
            if not name:
                continue
            try:
                robots[side].rm_change_tool_frame(name)
            except Exception as exc:
                self._warnings.append(f"{side}: tool frame not restored ({exc!r})")

        if self.recorder is not None:
            try:
                self.recorder.stop(self.run_record())
            except Exception as exc:
                self._warnings.append(f"recording stop failed: {exc!r}")

        try:
            self._warnings.extend(
                runmode.restore_run_modes(self._originals, robots))
        except Exception as exc:
            self._warnings.append(f"run-mode restore failed: {exc!r}")

        # Speed limits are GLOBAL controller state shared with the Web GUI and
        # every other program; a run that raised them owns putting them back.
        for side, before in self._limits_before.items():
            if side in robots:
                self._warnings.extend(limits.restore(robots[side], before))
        for side, before in self._joints_before.items():
            if side in robots:
                self._warnings.extend(limits.restore_joints(robots[side], before))

        for sig, prev in self._prev_signals.items():
            try:
                signal.signal(sig, prev)
            except Exception:
                pass

        for handle in self.handles.values():
            connect.close_arm(handle)

    # ── dispatch ──────────────────────────────────────────────────────────
    def backend(self, side: str) -> SdkBackend:
        return self._backends[side]

    def backends(self) -> Dict[str, SdkBackend]:
        return dict(self._backends)

    def issue(self, call, backend):
        """The injectable issuer. SIM assumes pole and hand; nothing else."""
        if call.kind in ("lift", "hand") and call.device not in ("pole", "hand"):
            raise ExecutionError(
                f"{call.stage}: kind={call.kind!r} on device={call.device!r} — "
                "the SIM skip is defined per DEVICE and this pairing is not one "
                "it can classify; a skip is a verdict, never a default")
        t0 = time.perf_counter()
        ret = None
        detail = ""
        raised = None
        self._issuing = True
        try:
            if self._sim_mode and call.device in ("pole", "hand"):
                ret, detail = self._assume(call, backend)
            else:
                ret = _issue(call, backend)
            return ret
        except BaseException as exc:
            # A RAISE IS NOT A PASS. `ret` stays None when `_issue` throws, and
            # None is the same value a device that commands nothing returns —
            # so the mark scored a rejected dispatch as ok=True and the run
            # record said `execute_path ... ok: True` on a run that had just
            # failed with ret=1 (front_right, 2026-08-20).
            raised = exc
            raise
        finally:
            self._issuing = False
            # ALWAYS mark, even when _issue raised. Without this the run that
            # failed is exactly the run with no `execute_path` window, and C9's
            # mandatory check degrades to NOT-CHECKED precisely when it matters.
            self._mark(call, t0, time.perf_counter(), ret, detail, raised)

    def _assume(self, call, backend):
        """Record what a pole/hand stage WOULD have done. Skip only the command.

        The target is still resolved, so a hand state the plan cannot express
        still raises here — that is a config fault in every mode, not something
        SIM should hide.
        """
        d = call.detail
        side = call.arm or backend.side
        if call.kind == "lift":
            RecordingBackend.lift(backend, d["height_m"], d["speed_pct"])
            hw = backend.calls[-1].get("hw")
            # Pole units are harmonized 1:1 with MoveIt, so hw units ARE
            # millimetres now (backends.LIFT_UNITS_PER_METRE = 1000). The
            # neutral "hw units" spelling is kept so the audit trail does not
            # have to be reinterpreted if the scale ever moves again.
            detail = (f"SIM assumed: {hw} hw units at {d['speed_pct']}% "
                      "(controller does not simulate the pole)")
            self._pole_assumed[side] = d["height_m"]
        else:
            counts = backend.hand_counts(d["joints_rad"])
            RecordingBackend.hand(backend, d["joints_rad"], d["speed_pct"],
                                  state=d.get("state"))
            backend.calls[-1]["counts"] = counts
            backend.calls[-1]["dwell_s_assumed"] = backend.hand_dwell_s(counts)
            detail = (f"SIM assumed: {d.get('state')} {counts} "
                      "(controller does not simulate the hand)")
        backend.calls[-1]["sim_assumed"] = True
        return 0, detail

    def _line_acc(self) -> float:
        """The acceleration to WRITE: the task's own, floored by C2's minimum.

        Never below `line_acc_for(v)` — the controller refuses a pair on the
        exact `acc >= 3 x speed` boundary with a bare ret=1 and keeps the old
        value, and `%.2f` rounds any smaller nudge away.
        """
        floor = limits.line_acc_for(self.line_speed)
        if self.line_acc is None:
            return floor
        return max(float(self.line_acc), floor)

    def _mark(self, call, t0, t1, ret, detail, raised=None) -> None:
        entry = {"side": call.arm, "stage": call.stage, "device": call.device,
                 "kind": call.kind, "t_start": t0, "t_end": t1,
                 "ok": raised is None and ret in (None, 0),
                 "detail": detail if raised is None else repr(raised),
                 "raised": None if raised is None else repr(raised)}
        self._marks.append(entry)
        if self.recorder is not None:
            self.recorder.mark(call.arm, call.stage, call.device, t0, t1,
                               entry["ok"], detail)

    def execute(self, program):
        from alix_dispatch import backends as _b
        return _b.execute(program, self._single_backend(program),
                          issue=self.issue)

    def _single_backend(self, program):
        side = getattr(program, "side", None) or self.sides[0]
        return self._backends[side]

    def execute_schedule(self, plan):
        from alix_dispatch import schedule as _s
        return _s.execute_dual(plan, self.backends(), issue=self.issue)

    def execute_region_chains(self, plan):
        from alix_dispatch import schedule as _s
        return _s.execute_region_chains(plan, self.backends(), issue=self.issue)

    # ── record ────────────────────────────────────────────────────────────
    def marks(self) -> List[dict]:
        return list(self._marks)

    def run_record(self) -> dict:
        return {
            "task": self.task,
            "requested_mode": runmode.mode_label(self.mode),
            "observed_mode": {s: runmode.mode_label(m)
                              for s, m in self._observed.items()},
            "sim": self._sim_mode,
            "sdk": {s: connect.sdk_provenance(h.robot)
                    for s, h in self.handles.items()},
            "arm_ips": {s: h.ip for s, h in self.handles.items()},
            "pole_assumed_m": dict(self._pole_assumed),
            "line_speed_written_mps": self.line_speed,
            "line_acc_written_mps2": (self._line_acc()
                                      if self.line_speed else None),
            "limits_before": {s: v for s, v in self._limits_before.items()},
            "joint_caps_written": dict(self.joint_speed),
            "joint_limits_before": {s: v for s, v in self._joints_before.items()},
            "marks": self.marks(),
            "warnings": list(self._warnings),
            # The residual, stated rather than hidden: `session.marks` and
            # `schedule.Event` are two timelines of the same run. The
            # completion window is derived from marks ONLY.
            "window_source": "session.marks",
        }
