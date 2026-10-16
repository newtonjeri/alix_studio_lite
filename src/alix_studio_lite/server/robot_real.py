"""Lite's commands to the real arms, through ALIX's own robot code (the owner's decision of 14
October: Lite does every write and setup step that teaching and running need). One RealArm per
connected controller, on the link's connection (robot_link.ArmLink):

- the pole and the hand to a file's stage, ALIX's way (``staging.stage_arm``: the hand opened when
  the pole must move, the arm checked at rest, the pole, then the hand, each read back), at 50 %
  speed; Prepare's Apply beside the pole moves the pole alone, and the one beside the hand the hand
  alone (the owner, 15 Oct). Real only: the controller cannot simulate them, so in Hardware sim they
  are taken as arrived (robot_standin.py);
- the controller's run mode, SIM or REAL, verified by reading it back (``runmode``);
- drag teaching: the SDK's drag-teach start and stop, and the controller's recording saved by the
  SDK itself, as the control program does, so the files are byte for byte the control program's;
- upload to a program number, the program list and a stored program's run: the control program's
  own tool, ``vendor/control_app/rm_upload.py`` (the operators' upload-and-run script), its steps
  and its printed lines (into the hardware log); the read-back after an upload is tried again when
  the controller does not answer at once;
- a move to a joint pose at the speed slider's % (``staging.park_arm``), checked against the pose.

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
# MoveJ / MoveL (a move to a start, to a global point, the controller's own move to a program's first
# point) go at the top bar's speed slider, the same for both arms (the owner, 15 Oct), passed in by the
# caller: ALIX's way (staging.park_arm), the controller's global speed at 100 % while Lite moves, the
# move at the slider's %, then the global speed put back to the one found when the arm was connected.
RESTORE_PCT = 50          # the global speed put back when the controller did not say which it had
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


_DAC = None


def _dac():
    """The operator's error tools, butterfli-realman-alix's dual-arm concept code vendored as it is
    (``dual_arm_common``): every error the controller exposes for an arm (``error_state``), and the
    controller web GUI's Clear Error buttons (``clear_errors``: the system's, then each joint's)."""
    global _DAC
    if _DAC is None:
        from .. import vendor_paths

        src = str(vendor_paths.ALIX / "external/butterfli-realman-alix/Demo/RMDemo_Python/RMDemo_DualArmConcept/src")
        if src not in sys.path:
            sys.path.insert(0, src)
        import dual_arm_common

        _DAC = dual_arm_common
    return _DAC


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


def _interrupt(tid: int, exc=KeyboardInterrupt) -> None:
    """``exc`` raised in thread ``tid`` at its next Python step (None: forget one not yet raised)."""
    import ctypes

    ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_ulong(tid), None if exc is None else ctypes.py_object(exc))


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
        """The pole alone to ``mm`` (Prepare's Apply beside the pole, Advanced), at 50 %, arrival verified;
        nothing else moves."""
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

    # ── Arm Drag Mode, for recovery and never for recording, and the controller's errors (feedback v5,
    #    slide 8; the owner, 16 Oct: "Lite should be able to clear errors of any kind, similar to how the
    #    vendor web GUI does, we have scripts implemented to do that")
    def _sdk(self, name: str):
        fn = getattr(self.robot, name, None)
        if fn is None:
            raise UserError(f"The {self.SIDE} controller does not answer {name} (ALIX's emulator has no such call).")
        return fn

    def errors(self) -> dict:
        """Every error the controller exposes for this arm, as the operator's tools read them
        (``dual_arm_common.error_state``): system codes, joint error flags, the lift driver and the
        joints not enabled, each with its meaning (``error_codes``, from RealMan's own tables)."""
        dac = _dac()
        with self.link.busy("reading the errors"):
            st = dac.error_state(self)
        return self._errors_view(st)

    def _errors_view(self, st: dict) -> dict:
        from error_codes import describe_joint_err, describe_system_err

        dac = _dac()
        return dict(readable=bool(st["readable"]), clean=bool(st["readable"] and dac.error_state_clean(st)),
                    text=dac.describe_error_state(st), system=[describe_system_err(c) for c in st["sys"]],
                    joints=[dict(joint=j, text=describe_joint_err(f)) for j, f in st["joints"]],
                    lift=st.get("lift_err") or None, disabled=list(st.get("disabled") or []))

    def clear_errors(self) -> dict:
        """The controller web GUI's Clear Error, as the operator's tools do it (``dual_arm_common.clear_errors``):
        the system's errors, then every flagged joint's own clear (the system clear leaves those set), read
        back. Nothing moves."""
        dac = _dac()
        with self.link.busy("clearing the errors"), _printed_to(lambda line: self._hw("clear errors", text=line.strip())):
            ok = dac.clear_errors(self)
            st = dac.error_state(self)
        out = self._errors_view(st)
        self._hw("clear errors", ok=bool(ok), text=out["text"])
        return dict(out, ok=bool(ok))

    def read_sensitivity(self) -> int | None:
        """The drag sensitivity, 0-100 % (higher: the arm moves more easily by hand; the controller's own
        default is 100), or None when the controller does not answer it."""
        try:
            with self.link.busy("reading the drag sensitivity"):
                ret, grade = self._sdk("rm_get_drag_teach_sensitivity")()
        except Exception as exc:                                                    # noqa: BLE001
            self._hw("read drag sensitivity", ok=False, text=str(exc))
            return None
        ok = ret == 0 and isinstance(grade, int) and not isinstance(grade, bool) and 0 <= grade <= 100
        self._hw("read drag sensitivity", ret=ret, ok=ok, text=f"{grade} %" if ok else "not reported")
        return int(grade) if ok else None

    def set_sensitivity(self, pct: int) -> int:
        """Sets the drag sensitivity and reads it back. Raises when the controller refuses it or reads
        another value."""
        with self.link.busy("setting the drag sensitivity"):
            ret = self._call("rm_set_drag_teach_sensitivity", self._sdk("rm_set_drag_teach_sensitivity"), int(pct))
        if ret != 0:
            raise UserError(f"The {self.SIDE} controller refused the drag sensitivity {pct} % "
                            f"({return_text(ret, 'rm_set_drag_teach_sensitivity')}).", "Try again, or set it on its pendant.")
        back = self.read_sensitivity()
        if back != int(pct):
            raise UserError(f"The {self.SIDE} controller reads a drag sensitivity of {back} % after it was set to {pct} %.",
                            "Try again, or check it on its pendant.")
        return back

    def drag_ready(self) -> list[str]:
        """What stops Drag Mode on this arm: a joint not enabled (its brake releases and it drops: ALIX's
        F21a), or a program running or paused on the controller (a later continue would start from an arm
        moved by hand)."""
        with self.link.busy("checking the arm before drag mode"):
            es = _dac().error_state(self)
            ret, st = self.robot.rm_get_program_run_state()
        out = []
        if es.get("disabled"):
            js = ", ".join(f"J{j}" for j in es["disabled"])
            out.append(f"{js} {'is' if len(es['disabled']) == 1 else 'are'} not enabled (a brake would release and the joint drop)")
        if ret == 0 and (st or {}).get("run_state") not in (0, None):
            out.append("a program is running or paused on the controller")
        return out

    def drag_on(self) -> None:
        """Drag Mode: the controller's drag teaching with recording OFF (``rm_start_drag_teach(0)``; the
        control program calls it with 1 to record): the arm moves by hand and nothing is recorded."""
        with self.link.busy("drag mode"):
            ret = self._call("rm_start_drag_teach", self._sdk("rm_start_drag_teach"), 0)
        if ret != 0:
            raise UserError(f"The {self.SIDE} controller refused drag mode ({return_text(ret, 'rm_start_drag_teach')}).",
                            "It refuses while an error is latched: read and clear the errors, then try again.", code="errors")

    def drag_off(self) -> bool:
        """Leaves Drag Mode (``rm_stop_drag_teach``). Never raises: False when the controller did not confirm."""
        try:
            with self.link.busy("leaving drag mode"):
                return self._call("rm_stop_drag_teach", self._sdk("rm_stop_drag_teach")) == 0
        except Exception:                                                           # noqa: BLE001
            return False

    def safe_state(self) -> dict:
        """After Drag Mode (Normal Mode): the arm at rest, no program running or paused, no error latched
        and every joint enabled. ``problems``: what is not, in plain words."""
        from alix_dispatch import track

        problems = []
        with self.link.busy("checking the arm after drag mode"):
            rest = track.wait_until_at_rest(self.robot, timeout_s=3.0)
            ret, st = self.robot.rm_get_program_run_state()
            es = _dac().error_state(self)
        view = self._errors_view(es)
        if not rest.get("at_rest"):
            problems.append(f"the arm is still moving ({float(rest.get('peak_deg_s') or 0.0):.1f} °/s)")
        if ret != 0:
            problems.append("its program state cannot be read")
        elif (st or {}).get("run_state") not in (0, None):
            problems.append("a program is running or paused on the controller")
        if not view["readable"]:
            problems.append("its errors cannot be read")
        elif not view["clean"]:
            problems.append(f"the controller reports {view['text']}")
        return dict(ok=not problems, problems=problems, errors=view)

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

    # ── what the controller stores (Data Management; the owner, 16 Oct), read only
    def controller_data(self) -> dict:
        """Its programs as RealMan's program list gives them (slot, name: cut to 31 characters by its API,
        size in bytes, stored speed), the program its IO starts (``rm_get_default_run_program``, 0: none),
        its tracks list (``rm_get_trajectory_file_list``: RealMan's API answers it on 4th-generation
        controllers only) and its global points. Nothing is written."""
        with self.link.busy("reading the controller's data"), _printed_to(self._printed):
            held = self._slots()
            try:
                ret, default = self.robot.rm_get_default_run_program()
            except Exception as exc:                                                # noqa: BLE001
                ret, default = None, None
                self._hw("IO default program", ok=False, text=str(exc))
            tracks = self._tracks()
        self._hw("controller data", ok=True, text=f"{len(held)} programs; IO default {default if ret == 0 else 'not read'}; "
                 f"tracks {'read' if tracks['available'] else 'not available'}")
        return dict(programs=[dict(num=k, name=v[0], bytes=v[1], speed=v[2]) for k, v in sorted(held.items())],
                    default_program=int(default) if ret == 0 and default is not None else None,
                    default_ret=ret, tracks=tracks, points=self.global_points(), ip=self.link.ip)

    def _tracks(self) -> dict:
        """The drag-teaching tracks the controller keeps, a page of 10 at a time (a long page overflowed the
        SDK's buffer on the program list: alix_dispatch.track)."""
        out = []
        for page in range(1, 101):
            try:
                ret, d = self.robot.rm_get_trajectory_file_list(page, 10, "")
            except Exception as exc:                                                # noqa: BLE001
                return dict(available=False, list=[], why=f"RealMan's API could not read it here ({exc}).")
            if ret == -4:
                return dict(available=False, list=[], ret=ret, why="RealMan's API reads the tracks list on 4th-generation "
                            "controllers only; this one is 3rd generation (it answers -4). The arm's own web page lists them.")
            if ret != 0:
                return dict(available=False, list=out, ret=ret, why=f"The controller did not give its tracks list (ret {ret}).")
            batch = d.get("tra_list") or []
            out += [dict(name=t.get("name"), points=t.get("point_num"), created=t.get("create_time")) for t in batch]
            if len(batch) < 10 or len(out) >= int(d.get("total_size") or 0):
                break
        return dict(available=True, list=out)

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

    # ── the controller's global speed (the speed slider)
    def read_speed(self) -> int | None:
        """The controller's global speed ratio as it reports it, or None. The program run state carries
        it (``plan_speed``); ALIX found no getter for it (F98) and its hardware run records show the field
        empty whenever no program runs, so None is the expected answer on the real arms."""
        try:
            with self.link.busy("reading the global speed"):
                ret, st = self.robot.rm_get_program_run_state()
        except Exception as exc:                                                    # noqa: BLE001
            self._hw("read global speed", ok=False, text=f"raised {exc!r}")
            return None
        v = (st or {}).get("plan_speed") if ret == 0 and (st or {}).get("run_state") == 0 else None
        ok = isinstance(v, int) and not isinstance(v, bool) and 1 <= v <= 100
        self._hw("read global speed", ret=ret, ok=ok, text=f"{v} %" if ok else "not reported")
        return int(v) if ok else None

    def restore_speed(self, pct: int) -> bool:
        """The global speed put back (Lite disconnecting or closing). Never raises."""
        from alix_dispatch import track

        try:
            with self.link.busy("putting the global speed back"):
                ret = track.set_global_speed(self.robot, int(pct))
        except Exception as exc:                                                    # noqa: BLE001
            self._hw("restore global speed", args=dict(pct=int(pct)), ok=False, text=f"raised {exc!r}")
            return False
        self._hw("restore global speed", args=dict(pct=int(pct)), ret=ret, ok=ret == 0)
        return ret == 0

    def run(self, program_id: int, first_row_millideg: list, points: int, on_line=None, *, speed: int = RESTORE_PCT,
            restore_pct: int = RESTORE_PCT) -> dict:
        """rm_upload.run: refuses an empty slot, an arm error or a program already running; the
        controller moves the arm to the first point at ``speed`` % (the speed slider) and plays the
        points at their own timing; error codes are watched every half second; the global speed is
        put back to ``restore_pct`` however the run ends (its ``finally``)."""
        rmu = _rmu()
        pid = int(program_id)
        timeout = 900.0 if points is None else max(60.0, (int(points) - 1) * 0.005 * 1.5 + 60.0)   # unknown length: 15 min
        a = SimpleNamespace(yes=True, side=self.side, run_speed=int(speed), restore_speed=int(restore_pct), timeout=timeout)
        log = io.StringIO()
        t0 = time.monotonic()
        def line_out(line):
            log.write(line + "\n")
            self._printed(line)
            if on_line is not None:
                on_line(line)

        with self.link.busy(f"running program {pid}"), _printed_to(line_out):
            try:
                rc = rmu.run(self.robot, a, self.link.ip, pid, None if first_row_millideg is None else list(first_row_millideg))
            except SystemExit as exc:                     # its program list did not answer
                rc = 1
                log.write(f"** {exc}\n")
        said = log.getvalue()
        why = next((ln.strip(" *") for ln in reversed(said.splitlines()) if ln.strip().startswith("**")), None)
        self._hw("run", args=dict(program_id=pid, points=points, transit_speed=int(speed), restore_speed=int(restore_pct)),
                 ret=rc, ok=rc == 0,
                 elapsed_s=time.monotonic() - t0, text=why)
        return dict(ok=rc == 0, why=why, seconds=round(time.monotonic() - t0, 2), lines=said.splitlines())

    # ── Merge + Sequence S2: Page 02's steps through runner v14 (feedback v5, slide 9)
    def run_workflow(self, steps: list, overall_pct: int, restore_pct: int, timeout_s: float = 300.0, on_line=None,
                     stop_hook=None) -> dict:
        """An arm's part of a Page 02 stage through runner v14's own ``run_workflow_sequence``, as V53
        runs it in a process of its own: one connection; the slots read once (a missing one refuses it
        all), the arm's errors and run state checked, the overall speed set, each step (Program, Pole,
        Hand, Wait, Global Point) run and checked, the speed put back to ``restore_pct`` in its
        ``finally``. ``stop_hook(fn)`` registers ``fn`` for STOP, which interrupts the runner as V53's
        STOP does (Ctrl+C: its own slow stop of the arm, the pole stopped, the hand frozen)."""
        rt = _runner()
        norm = [rt.normalize_workflow_step(st) for st in steps]
        a = SimpleNamespace(yes=True, side=self.side, workflow_overall_speed=int(overall_pct), restore_speed=int(restore_pct),
                            timeout=float(timeout_s))
        log = io.StringIO()
        t0 = time.monotonic()

        def line_out(line):
            log.write(line + "\n")
            self._runner_line(line)
            if on_line is not None:
                on_line(line)

        tid, guard, state = threading.get_ident(), threading.Lock(), {"inside": False, "stopped": False}

        def stop():
            with guard:
                state["stopped"] = True
                if state["inside"]:
                    _interrupt(tid)

        if stop_hook is not None:
            stop_hook(stop)
        rc = 1
        with self.link.busy(f"sequence of {len(norm)} step(s)"), _printed_to(line_out):
            try:
                try:
                    with guard:
                        if state["stopped"]:
                            raise KeyboardInterrupt
                        state["inside"] = True
                    rc = rt.run_workflow_sequence(self.robot, a, self.link.ip, norm)
                finally:
                    with guard:
                        state["inside"] = False
                        stopped = state["stopped"]
                    if stopped:                       # a stop that came as it ended is not raised later (cleared
                        _interrupt(tid, None)         # only then: clearing leaves CPython's flag up, slowing the server)
            except KeyboardInterrupt:                 # outside the runner's own handling (its checks before the steps)
                rc = 1
                line_out("** WORKFLOW STOP REQUESTED")
            except SystemExit as exc:                 # the runner exits when the program list does not answer
                rc = 1
                line_out(f"** WORKFLOW FAILED: {exc}")
            except Exception as exc:                  # noqa: BLE001 - the runner's own lines say the rest
                rc = 1
                line_out(f"** WORKFLOW FAILED: {exc}")
        said = log.getvalue()
        why = next((ln.strip(" *") for ln in reversed(said.splitlines()) if ln.strip().startswith("**")), None)
        self._hw("workflow", args=dict(steps=len(norm), overall_speed=int(overall_pct), restore_speed=int(restore_pct)),
                 ret=rc, ok=rc == 0, elapsed_s=time.monotonic() - t0, text=why)
        return dict(ok=rc == 0, rc=rc, why=why, stopped=state["stopped"], seconds=round(time.monotonic() - t0, 2),
                    lines=said.splitlines())

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

    def move_point(self, name: str, move: str = "movej", speed: int = RESTORE_PCT, wait_s: float = 0.0,
                   restore_pct: int = RESTORE_PCT) -> dict:
        """MoveJ (to its joints) or MoveL (to its pose, in its own frames, put back after) to a global
        point at ``speed`` % (the speed slider), with the runner's move_global_waypoint inside ALIX's
        global speed guard (the global speed at 100 % for the move, ``restore_pct`` after it, however it
        ends); a MoveJ is read back against the point's joints."""
        from alix_dispatch import staging as alix_staging
        from alix_dispatch import track

        rt = _runner()
        move = "movel" if str(move).lower() == "movel" else "movej"
        t0, gap, why = time.monotonic(), None, None
        with self.link.busy(f"{'MoveL' if move == 'movel' else 'MoveJ'} to {name}"), _printed_to(self._runner_line):
            try:
                with track.global_speed_guard(self.robot, int(restore_pct), log=self._alix_log()):
                    track.set_global_speed(self.robot, 100)
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

    def move_to(self, target_millideg: list, speed: int = RESTORE_PCT, restore_pct: int = RESTORE_PCT) -> dict:
        """A MoveJ to a joint pose at ``speed`` % (the speed slider), ALIX's park move (the global speed at
        100 % for it, ``restore_pct`` after), checked against the pose (within 2 degrees)."""
        from alix_dispatch import staging as alix_staging
        from alix_dispatch import track

        deg = [v / 1000.0 for v in target_millideg]
        with self.link.busy("moving to the start"):
            out = alix_staging.park_arm(self.handle, {}, rest_deg=deg, rest_speed=int(speed), restore_speed=int(restore_pct),
                                        log=self._alix_log())
            gap = track.start_gap_deg(self.robot, list(target_millideg))
        ok = bool(out.get("ok")) and gap is not None and gap <= alix_staging.START_TOLERANCE_DEG
        self._hw("move to pose", args=dict(deg=[round(d, 2) for d in deg], speed=int(speed), restore_speed=int(restore_pct)), ok=ok,
                 text=f"gap {gap} deg" + ("" if out.get("ok") else f"; refused: {_words(out.get('refusal'))}"))
        if not ok:
            raise UserError(f"The {self.SIDE} arm did not reach the start (" + (_words(out.get("refusal")) if not out.get("ok")
                            else f"{'?' if gap is None else round(gap, 1)} degrees away") + ").",
                            "Check the arm on its pendant, then try again.")
        return dict(gap_deg=gap)
