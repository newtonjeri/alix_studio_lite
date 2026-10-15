"""Lite's commands to the real arms, through ALIX's own robot code (the owner's decision of 14
October: Lite does every write and setup step that teaching and running need). One RealArm per
connected controller, on the link's connection (robot_link.ArmLink):

- the pole and the hand to a file's stage, ALIX's way (``staging.stage_arm``: the hand opened when
  the pole must move, the arm checked at rest, the pole, then the hand, each read back), at 50 %
  speed. Real only: the controller cannot simulate them, so in Hardware sim they are taken as
  arrived (robot_standin.py);
- the controller's run mode, SIM or REAL, verified by reading it back (``runmode``);
- drag teaching: the SDK's drag-teach start and stop, and the controller's recording saved by the
  SDK itself, as the control program does, so the files are byte for byte the control program's;
- upload to a program number, the program list and a stored program's run: the control program's
  own tool, ``vendor/control_app/rm_upload.py`` (the operators' upload-and-run script), its steps
  and its printed lines (into the hardware log); the read-back after an upload is tried again when
  the controller does not answer at once;
- a move to a joint pose at 30 % (``staging.park_arm``), checked against the pose.

Every command goes to the hardware log with its return code and what it means (logbook.py), and
ALIX's own lines with it. While a command runs, the link only watches the E-stop input, so two
callers never compete for one controller (ALIX measured reply timeouts when they do).
"""
from __future__ import annotations

import io
import sys
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

from .jobs import UserError
from .robot_link import return_text

POLE_SPEED_PCT = 50       # the pole in Real (the owner's choice, 14 Oct; ALIX itself ran it at 100 %)
HAND_SPEED_SET = 500      # the hand's SPEED_SET, of 1000: 50 %
MOVE_SPEED = 30           # % for a move to a start: what Lite's simulation models (staging.TRANSIT_FRACTION)
RUN_TRANSIT_SPEED = 20    # % of the controller's own move to a program's first point (rm_upload.py's default)
RESTORE_PCT = 50          # the global speed ratio left after a run or a move, as ALIX and rm_upload leave it
POINT_SPEED = 20          # % for a move to a global point: the control program's default
CHECK_TRIES = 4           # the read-back after an upload: tries, 1 s apart, before it is called failed


def _words(code: str | None) -> str:
    return (code or "no reason given").replace("_", " ")


def _settled(staged: dict) -> bool:
    """Whether stage_arm's hand step ended SETTLED: ANGLE_ACT stopped changing (ALIX's
    HandModbus.command), the way the control program's Hand.command takes a hand as done."""
    step = next((s for s in staged.get("steps") or [] if s.get("step") == "hand"), None)
    r = (step or {}).get("result") or {}
    return bool(r.get("ok") and (r.get("arrived") or r.get("settled")))


# ── the control program's upload-and-run tool, as a module, and its printed lines in Lite's log
_RMU = None
_RMU_LOCK = threading.Lock()


def _rmu():
    """vendor/control_app/rm_upload.py, loaded once. Its main() sets the SDK's rm_send_project_t;
    Lite is not its main, so it is set here."""
    global _RMU
    with _RMU_LOCK:
        if _RMU is None:
            import importlib.util

            from .. import vendor_paths
            from Robotic_Arm.rm_robot_interface import rm_send_project_t

            spec = importlib.util.spec_from_file_location("lite_rm_upload", vendor_paths.RM_UPLOAD)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            mod.rm_send_project_t = rm_send_project_t
            _RMU = mod
        return _RMU


# ── the control program's runner v14 (v53 support), as a module: the controller's global points
_RUNNER = None


def _runner():
    """vendor/control_app/rm_upload_fast_v14_safe_transit.py, loaded once: its global points are
    listed and moved to as the control program does (global_waypoints, move_global_waypoint)."""
    global _RUNNER
    with _RMU_LOCK:
        if _RUNNER is None:
            import importlib.util

            from .. import vendor_paths

            spec = importlib.util.spec_from_file_location("lite_runner_v14", vendor_paths.RUNNER_V14)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            _RUNNER = mod
        return _RUNNER


class _ThreadLines(io.TextIOBase):
    """sys.stdout, with the lines one thread prints sent to its sink (the hardware log of the arm
    whose command prints them); every other thread's output goes to the terminal as before."""

    def __init__(self, real):
        self.real, self.sinks, self.parts = real, {}, {}

    def write(self, text):
        tid = threading.get_ident()
        sink = self.sinks.get(tid)
        if sink is None:
            return self.real.write(text)
        *lines, self.parts[tid] = (self.parts.get(tid, "") + text).split("\n")
        for line in lines:
            if line.strip():
                sink(line.rstrip())
        return len(text)

    def flush(self):
        self.real.flush()


@contextmanager
def _printed_to(sink):
    if not isinstance(sys.stdout, _ThreadLines):
        sys.stdout = _ThreadLines(sys.stdout)
    out, tid = sys.stdout, threading.get_ident()
    out.sinks[tid] = sink
    try:
        yield
    finally:
        rest = out.parts.pop(tid, "")
        out.sinks.pop(tid, None)
        if rest.strip():
            sink(rest.rstrip())


class RealArm:
    def __init__(self, link, log=None):
        self.link, self.log, self.side = link, log, link.side
        self.SIDE = link.side.upper()

    # ── plumbing
    @property
    def handle(self):
        if self.link.handle is None:
            raise UserError(f"The {self.SIDE} controller is not connected.", "Connect it in Robot & mode.")
        return self.link.handle

    @property
    def robot(self):
        return self.handle.robot

    def _alix_log(self):
        return self.log.hw_logger(self.side) if self.log is not None else (lambda *_: None)

    def _hw(self, verb, args=None, ret=None, ok=None, elapsed_s=None, text=None) -> None:
        if self.log is not None:
            self.log.hardware(self.side, verb, args=args, ret=ret, ok=ok, elapsed_s=elapsed_s, text=text)

    def _call(self, verb: str, fn, *args):
        """One SDK call, logged with its return code and what it means."""
        t0 = time.monotonic()
        try:
            out = fn(*args)
        except Exception as exc:                                                    # noqa: BLE001
            self._hw(verb, list(args), None, False, time.monotonic() - t0, f"raised {exc!r}")
            raise UserError(f"The {self.SIDE} controller did not answer {verb} ({exc}).",
                            "Check the controller's power and cable.") from None
        ret = out[0] if isinstance(out, tuple) else out
        self._hw(verb, list(args), ret, ret == 0, time.monotonic() - t0, None if ret == 0 else return_text(ret, verb))
        return out

    # ── the pole and the hand
    def stage(self, stage) -> dict:
        """The pole and the hand to ``stage`` (staging.Stage), each read back. Raises when either
        did not arrive: then nothing else may run."""
        from alix_dispatch import staging as alix_staging

        log = self._alix_log()
        with self.link.busy("staging the pole and the hand"):
            pole, hand, opened = alix_staging.open_devices(self.handle, log=log, speed_set=HAND_SPEED_SET)
            if hand is None:
                self._hw("stage", text=f"refused: {_words(opened.get('refusal'))}", ok=False)
                raise UserError(f"The {self.SIDE} hand cannot be reached (Modbus), so nothing was staged.",
                                "Check the hand's cable and power, then try again.")
            task = {"side": self.side, "pre_stroke": {"pole_m": stage.pole_mm / 1000.0, "hand_counts": list(stage.hand)}}
            out = alix_staging.stage_arm(self.handle, task, pole=pole, hand=hand, log=log, pole_speed_pct=POLE_SPEED_PCT)
        p, h = out.get("pole") or {}, out.get("hand") or {}
        if not out.get("ok") and out.get("refusal") == "hand_unverified" and _settled(out):
            # the control program's rule (vendor/control_app/hand.py, Hand.command): a hand that has
            # ARRIVED or SETTLED is done, "fingers closing on a tool stop early on force"
            out = dict(out, ok=True, refusal=None, settled=True)
            self._hw("hand settled", args=dict(target=list(stage.hand)), ok=True,
                     text=f"stopped at {h.get('read')} (on the tool): done, as the control program takes it")
        found = p.get("found_m")
        self._hw("stage", args=dict(pole_mm=stage.pole_mm, hand=list(stage.hand), speed_pct=POLE_SPEED_PCT),
                 ok=bool(out.get("ok")), text=(f"pole {found * 1000:.0f} mm" if found is not None else "pole not read")
                 + f", hand {h.get('read')}" + ("" if out.get("ok") else f"; refused: {_words(out.get('refusal'))}"))
        if not out.get("ok"):
            raise UserError(f"The {self.SIDE} arm was not staged: {_words(out.get('refusal'))} "
                            f"(pole {'?' if found is None else f'{found * 1000:.0f}'} mm of {stage.pole_mm:.0f} mm). "
                            "Nothing else ran.", "Check the pole and the hand, then try again.")
        return out

    def pole_to(self, mm: float) -> dict:
        """The pole alone to ``mm`` (Advanced), at 50 %, arrival verified."""
        from alix_dispatch import peripherals

        with self.link.busy("moving the pole"):
            out = peripherals.Pole(self.robot, log=self._alix_log()).move(float(mm) / 1000.0, speed_pct=POLE_SPEED_PCT)
        self._hw("pole move", args=dict(mm=mm, speed_pct=POLE_SPEED_PCT), ret=out.get("ret"), ok=bool(out.get("ok")),
                 elapsed_s=out.get("t_s"), text=f"final {out.get('final_m')} m, verified by {out.get('verified_by')}")
        if not out.get("ok"):
            raise UserError(f"The {self.SIDE} pole did not reach {mm:.0f} mm ({out.get('error') or return_text(out.get('ret'))}).",
                            "Check the pole, then try again.")
        return out

    def hand_to(self, counts: list) -> dict:
        """The hand alone to six counts (Advanced), at 50 %, read back. Done as staging takes it
        (_settled: the control program's Hand.command): ARRIVED, or SETTLED short of the target,
        fingers stopped on force (on the lab's hand, 14 Oct, the thumb stopped 714 counts short)."""
        from alix_dispatch import peripherals

        with self.link.busy("moving the hand"):
            hand = peripherals.HandModbus(self.robot, self.side, log=self._alix_log())
            ret = hand.open(speed_set=HAND_SPEED_SET)
            if ret != 0:
                self._hw("hand open", ret=ret, ok=False, text=return_text(ret))
                raise UserError(f"The {self.SIDE} hand cannot be reached (Modbus).", "Check the hand's cable and power.")
            out = hand.command(list(counts), wait=True)
        done = bool(out.get("ok") and (out.get("arrived") or out.get("settled")))
        short = "" if out.get("arrived") or not done else " (settled short of the target: stopped on force)"
        self._hw("hand move", args=dict(counts=list(counts), speed_set=HAND_SPEED_SET), ret=out.get("ret"),
                 ok=done, elapsed_s=out.get("t_s"), text=f"read back {out.get('after')}{short}")
        if not done:
            raise UserError(f"The {self.SIDE} hand did not reach its pose (it reads {out.get('after')}).",
                            "Check the hand, then try again.")
        return out

    # ── the controller's run mode
    def set_run_mode(self, mode: str) -> dict:
        """SIM or REAL, read back. Returns what to hand to ``restore_run_mode``."""
        from alix_dispatch import runmode

        with self.link.busy(f"switching to {mode}"):
            originals = runmode.apply_run_mode(runmode.SIMULATION if mode == "SIM" else runmode.REAL,
                                               {self.side: self.robot})
        self._hw(f"run mode {mode}", ok=originals is not None, text=None if originals is not None else "not taken (read back)")
        if originals is None:
            raise UserError(f"The {self.SIDE} controller did not switch to its {mode} mode (read back), so nothing ran.",
                            "Check the controller's errors on its pendant.")
        return originals

    def restore_run_mode(self, originals: dict) -> None:
        from alix_dispatch import runmode

        with self.link.busy("restoring the run mode"):
            failed = runmode.restore_run_modes(originals, {self.side: self.robot})
        if failed and self.log is not None:
            self.log.alert("warn", f"The controller's run mode could not be set back ({', '.join(map(str, failed))}).",
                           hint="Check its mode on the pendant before the next run.", side=self.side, source="runmode")

    # ── drag teaching, as the control program does it
    def teach_start(self) -> None:
        with self.link.busy("starting drag teaching"):
            ret = self._call("rm_start_drag_teach", self.robot.rm_start_drag_teach, 1)
        if ret != 0:
            raise UserError(f"The {self.SIDE} controller refused drag teaching ({return_text(ret, 'rm_start_drag_teach')}).",
                            "Check the arm's errors on its pendant, then try again.")

    def teach_stop(self, path: Path) -> int | None:
        """Leave drag teaching and save the controller's recording at ``path`` (written by the
        SDK, as the control program's files are). Returns its number of points."""
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with self.link.busy("saving the recording"):
            ret = self._call("rm_stop_drag_teach", self.robot.rm_stop_drag_teach)
            out = self._call("rm_save_trajectory", self.robot.rm_save_trajectory, str(path))
        if ret != 0:
            raise UserError(f"The {self.SIDE} arm may still be in drag teaching ({return_text(ret, 'rm_stop_drag_teach')}).",
                            "Press STOP, or the red E-stop button, before anyone lets go of the arm.")
        r, n = out if isinstance(out, tuple) else (out, None)
        if r != 0 or not Path(path).is_file() or Path(path).stat().st_size == 0:
            raise UserError(f"The {self.SIDE} controller did not save the recording ({return_text(r, 'rm_save_trajectory')}).",
                            "Record the region again.")
        return n

    def teach_cancel(self) -> None:
        """Leave drag teaching without saving (STOP, an E-stop, a cancel, Lite closing). Never raises."""
        try:
            with self.link.busy("leaving drag teaching"):
                self._call("rm_stop_drag_teach", self.robot.rm_stop_drag_teach)
        except Exception:                                                           # noqa: BLE001
            pass

    # ── programs
    def _slots(self) -> dict:
        """{slot: (name, bytes, speed)}: rm_upload.slots, tried again when the controller does not
        answer (it calls sys.exit then)."""
        rmu = _rmu()
        why = None
        for i in range(CHECK_TRIES):
            try:
                return rmu.slots(self.robot)
            except SystemExit as exc:
                why = str(exc)
                self._hw("program list", ok=False, text=why)
                time.sleep(1.0)
        raise UserError(f"The {self.SIDE} controller's program list could not be read ({why}).",
                        "Check the controller, then try again.")

    def programs(self) -> list[dict]:
        with self.link.busy("reading the program list"), _printed_to(self._printed):
            held = self._slots()
        self._hw("program list", ok=True, text=f"{len(held)} programs")
        return [dict(id=k, name=v[0], size=v[1], speed=v[2]) for k, v in sorted(held.items())]

    def _printed(self, line: str) -> None:
        self._hw("rm_upload", text=line)

    def upload(self, path: Path, program_id: int, name: str) -> dict:
        """Store a track file as program ``program_id`` with rm_upload.upload (the project file, store
        only, then the stored size read back); the read-back is tried again if the controller does
        not answer at once."""
        import tempfile

        rmu = _rmu()
        pid = int(program_id)
        t0 = time.monotonic()
        with self.link.busy(f"sending program {pid}"), _printed_to(self._printed):
            try:
                lines = rmu.read_track(Path(path))
            except SystemExit as exc:
                raise UserError(f"The file cannot be sent: {exc}.") from None
            with tempfile.TemporaryDirectory() as tmp:
                sent = rmu.write_project(lines, Path(tmp) / f"{name}.proj")       # the bytes the controller gets
            a = SimpleNamespace(track=Path(tmp) / f"{name}.txt", slot=pid, speed=0, yes=True)
            log = io.StringIO()
            with _printed_to(lambda line: (log.write(line + "\n"), self._printed(line))):
                try:
                    rc = rmu.upload(self.robot, a, self.link.ip, lines)
                except SystemExit as exc:                 # its program list did not answer
                    rc = 1
                    self._printed(f"** {exc}")
            said = log.getvalue()
            if rc != 0 and "sent: ret=0 err_line=-1" in said:           # sent; only the check did not answer
                for i in range(CHECK_TRIES):
                    time.sleep(1.0)
                    try:
                        got = rmu.slots(self.robot).get(pid)
                    except SystemExit as exc:
                        self._printed(f"** read-back {i + 1}: {exc}")
                        continue
                    if got and abs(got[1] - sent) <= max(64, sent // 1000):
                        self._printed(f"  VERIFIED (read back again): slot {pid} holds {got[0]!r}, {got[1]} bytes (sent {sent})")
                        rc = 0
                    break
        elapsed = time.monotonic() - t0
        self._hw("upload", args=dict(program_id=pid, file=Path(path).name, bytes=sent), ret=rc, ok=rc == 0, elapsed_s=elapsed)
        if rc != 0:
            why = next((ln.strip(" *") for ln in reversed(said.splitlines()) if ln.strip().startswith("**")), "not confirmed")
            raise UserError(f"Program {pid} was not stored on the {self.SIDE} controller: {why}.", "Send it again.")
        held = self._slots_quiet().get(pid)
        return dict(program_id=pid, bytes=sent, points=len(lines), stored_name=(held or ("?",))[0], seconds=round(elapsed, 1))

    def _slots_quiet(self) -> dict:
        try:
            with _printed_to(self._printed):
                return self._slots()
        except UserError:
            return {}

    def run(self, program_id: int, first_row_millideg: list, points: int, on_line=None) -> dict:
        """rm_upload.run: refuses an empty slot, an arm error or a program already running; the
        controller moves the arm to the first point at 20 % and plays the points at their own
        timing; error codes are watched every half second; the global speed is put back."""
        rmu = _rmu()
        pid = int(program_id)
        timeout = max(60.0, (int(points) - 1) * 0.005 * 1.5 + 60.0)
        a = SimpleNamespace(yes=True, side=self.side, run_speed=RUN_TRANSIT_SPEED, restore_speed=RESTORE_PCT, timeout=timeout)
        log = io.StringIO()
        t0 = time.monotonic()
        def line_out(line):
            log.write(line + "\n")
            self._printed(line)
            if on_line is not None:
                on_line(line)

        with self.link.busy(f"running program {pid}"), _printed_to(line_out):
            try:
                rc = rmu.run(self.robot, a, self.link.ip, pid, list(first_row_millideg))
            except SystemExit as exc:                     # its program list did not answer
                rc = 1
                log.write(f"** {exc}\n")
        said = log.getvalue()
        why = next((ln.strip(" *") for ln in reversed(said.splitlines()) if ln.strip().startswith("**")), None)
        self._hw("run", args=dict(program_id=pid, points=points, transit_speed=RUN_TRANSIT_SPEED), ret=rc, ok=rc == 0,
                 elapsed_s=time.monotonic() - t0, text=why)
        return dict(ok=rc == 0, why=why, seconds=round(time.monotonic() - t0, 2), lines=said.splitlines())

    # ── the controller's global points (the control program's runner v14)
    def global_points(self) -> list[dict]:
        """The global points stored on this controller, as the control program lists them: each
        one's name, joints (deg), pose (m, rad), work and tool frame, and when it was saved."""
        rt = _runner()
        with self.link.busy("reading the global points"), _printed_to(self._runner_line):
            try:
                pts = rt.global_waypoints(self.robot)
            except Exception as exc:                                                # noqa: BLE001
                self._hw("global points", ok=False, text=str(exc))
                raise UserError(f"The {self.SIDE} controller's global points could not be read ({exc}).",
                                "Check the controller, then try again.") from None
        self._hw("global points", ok=True, text=f"{len(pts)} point(s)")
        return [dict(name=p["point_name"], joints=[round(float(v), 3) for v in p["joint"][:7]],
                     pose=[round(float(v), 5) for v in p["pose"][:6]], work_frame=p["work_frame"],
                     tool_frame=p["tool_frame"], time=p["time"]) for p in pts]

    def move_point(self, name: str, move: str = "movej", speed: int = POINT_SPEED, wait_s: float = 0.0) -> dict:
        """MoveJ (to its joints) or MoveL (to its pose, in its own frames, put back after) to a global
        point, with the runner's move_global_waypoint; a MoveJ is read back against the point's joints."""
        from alix_dispatch import staging as alix_staging
        from alix_dispatch import track

        rt = _runner()
        move = "movel" if str(move).lower() == "movel" else "movej"
        t0, gap, why = time.monotonic(), None, None
        with self.link.busy(f"{'MoveL' if move == 'movel' else 'MoveJ'} to {name}"), _printed_to(self._runner_line):
            try:
                rc = rt.move_global_waypoint(self.robot, str(name), move, int(speed), float(wait_s))
            except Exception as exc:                     # noqa: BLE001 - a controller without a call refuses too
                rc, why = 1, str(exc)
            if rc == 0 and move == "movej":
                point = rt.get_global_waypoint(self.robot, str(name))
                gap = track.start_gap_deg(self.robot, [round(float(v) * 1000) for v in point["joint"][:7]])
                if gap is None or gap > alix_staging.START_TOLERANCE_DEG:
                    rc, why = 1, f"the arm is {'?' if gap is None else round(gap, 1)} degrees from it"
        self._hw(f"{move} global point", args=dict(name=name, speed=int(speed), wait_s=float(wait_s)), ret=rc, ok=rc == 0,
                 elapsed_s=time.monotonic() - t0, text=why or (None if gap is None else f"gap {gap:.2f} deg"))
        if rc != 0:
            raise UserError(f"The {self.SIDE} arm did not reach global point {name!r} ({why or 'the move did not finish'}).",
                            "Check the arm and the point on the pendant, then try again.")
        return dict(name=name, move=move, gap_deg=gap, seconds=round(time.monotonic() - t0, 2))

    def save_global_point(self, name: str, expect_deg: list | None = None, replace: bool = False) -> dict:
        """The arm's pose now, saved on this controller as global point ``name`` (feedback v2, slide 14):
        its joints and its tool pose as the controller reads them, in its current work and tool frames.
        A point of that name is replaced only when asked. Neither the control program nor ALIX saves a
        point, so this is RealMan's own add / update call (the owner allowed it, 15 Oct); it is read back
        with the runner's get_global_waypoint. ``expect_deg``: the joints the operator saw, the arm
        must still be there (0.5 degrees)."""
        import importlib

        rt = _runner()
        waypoint = getattr(importlib.import_module("Robotic_Arm.rm_robot_interface"), "rm_waypoint_t", None)
        if waypoint is None:
            raise UserError(f"This SDK cannot save global points on the {self.SIDE} controller.")
        t0, rc, gap, exists = time.monotonic(), 1, None, False
        with self.link.busy(f"saving global point {name}"), _printed_to(self._runner_line):
            ret, st = self.robot.rm_get_current_arm_state()
            if ret != 0 or not st or len(st.get("joint") or []) < 7 or len(st.get("pose") or []) < 6:
                raise UserError(f"The {self.SIDE} controller did not answer with the arm's pose.", "Try again.")
            joints = [float(v) for v in st["joint"][:7]]
            pose = [float(v) for v in st["pose"][:6]]
            if expect_deg is not None:
                off = max(abs(float(a) - b) for a, b in zip(expect_deg, joints))
                if off > 0.5:
                    raise UserError(f"The {self.SIDE} arm is no longer at the pose shown ({off:.1f} degrees away).",
                                    "Fetch the current pose again, then save it.")
            work, tool = rt._capture_current_frames(self.robot)
            try:
                rt.get_global_waypoint(self.robot, name)
                exists = True
            except Exception:                                                   # noqa: BLE001 - not on this controller
                exists = False
            if exists and not replace:
                raise UserError(f"“{name}” is already a global point on the {self.SIDE} controller.",
                                "Confirm to replace it, or choose another name.", code="replace")
            wp = waypoint(point_name=name, joint=joints, pose=pose, work_frame=work, tool_frame=tool)
            rc = self.robot.rm_update_global_waypoint(wp) if exists else self.robot.rm_add_global_waypoint(wp)
            if rc == 0:
                back = rt.get_global_waypoint(self.robot, name)
                gap = max(abs(float(a) - b) for a, b in zip(back["joint"][:7], joints))
        ok = rc == 0 and gap is not None and gap <= 0.05
        self._hw("update global point" if exists else "add global point",
                 args=dict(name=name, joints_deg=[round(v, 3) for v in joints], pose=[round(v, 5) for v in pose],
                           work_frame=work, tool_frame=tool), ret=rc, ok=ok, elapsed_s=time.monotonic() - t0,
                 text=None if gap is None else f"read back: {gap:.3f} deg")
        if not ok:
            raise UserError(f"Global point “{name}” was not saved on the {self.SIDE} controller "
                            + (f"(ret {rc})." if rc != 0 else "(it reads back different)."), "Check the controller, then try again.")
        return dict(name=name, joints_deg=[round(v, 3) for v in joints], pose=[round(v, 6) for v in pose],
                    work_frame=work, tool_frame=tool, replaced=exists)

    def _runner_line(self, line: str) -> None:
        self._hw("runner v14", text=line)

    def move_to(self, target_millideg: list) -> dict:
        """A move to a joint pose at 30 %, checked against it (within 2 degrees)."""
        from alix_dispatch import staging as alix_staging
        from alix_dispatch import track

        deg = [v / 1000.0 for v in target_millideg]
        with self.link.busy("moving to the start"):
            out = alix_staging.park_arm(self.handle, {}, rest_deg=deg, rest_speed=MOVE_SPEED, restore_speed=RESTORE_PCT,
                                        log=self._alix_log())
            gap = track.start_gap_deg(self.robot, list(target_millideg))
        ok = bool(out.get("ok")) and gap is not None and gap <= alix_staging.START_TOLERANCE_DEG
        self._hw("move to pose", args=dict(deg=[round(d, 2) for d in deg], speed=MOVE_SPEED), ok=ok,
                 text=f"gap {gap} deg" + ("" if out.get("ok") else f"; refused: {_words(out.get('refusal'))}"))
        if not ok:
            raise UserError(f"The {self.SIDE} arm did not reach the start (" + (_words(out.get("refusal")) if not out.get("ok")
                            else f"{'?' if gap is None else round(gap, 1)} degrees away") + ").",
                            "Check the arm on its pendant, then try again.")
        return dict(gap_deg=gap)
