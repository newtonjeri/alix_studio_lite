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
import math
import sys
import tempfile
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

from .jobs import Cancelled, UserError
from .robot_link import CLOSE_WAIT_S, return_text

POLE_SPEED_PCT = 50       # the pole in Real (the owner's choice, 14 Oct; ALIX itself ran it at 100 %)
HAND_SPEED_SET = 500      # the hand's SPEED_SET, of 1000: 50 %
# MoveJ / MoveL (a move to a start, to a global point, the controller's own move to a program's first
# point) go at the top bar's speed slider, the same for both arms (the owner, 15 Oct), passed in by the
# caller: ALIX's way (staging.park_arm), the controller's global speed at 100 % while Lite moves, the
# move at the slider's %, then the global speed put back to the one found when the arm was connected.
RESTORE_PCT = 50          # the global speed put back when the controller did not say which it had
CHECK_TRIES = 4           # the read-back after an upload: tries, 1 s apart, before it is called failed
CANFD_PERIOD_S = 0.005    # the lid and the seat's streams: one line every 5 ms (Lite's clock, D26; ALIX's CanFD default)
UDP_PORT = {"left": 8089, "right": 8090}    # each controller's 5 ms push to this computer, a port of its own


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


FORCE_CUT_N = 120.0       #: a stream stops when the wrist carries more than this (the owner, 26 Oct: a gate, not a note;
#: 23 Oct: the way back dragged the seat at 160 N to the E-stop; the lifts carry 72-80 N)
FORCE_CUT_FRAMES = 10     #: for this many 5 ms frames in a row (50 ms): a load, not one spike
END_TOL_DEG = 0.5         #: a stream has ended when the feed shows the arm within this of its last line (26 Oct: within 0.08 in
                          #: all 72 streams captured, by the time the last line was sent) ...
END_STILL_DEG_S = 1.6     #: ... and under this (ALIX's at-rest gate, wait_until_at_rest) ...
END_FRAMES = 3            #: ... for this many frames in a row (15 ms). ALIX's settle after a stream (six polls of the controller
                          #: 25 ms apart) took 0.24 s a stream and never had to wait (the owner, 26 Oct: redundant for a stream)
END_WAIT_S = 1.0          #: the feed not showing it within this: ALIX's polled wait takes over (the frames gone quiet, a lagging arm)


def wrist_load_n(frame: dict) -> float | None:
    """The six-axis sensor's load in N from one push frame: its force less its zero reference (both in mN)."""
    sensor = frame.get("six_force_sensor") or {}
    f, z = sensor.get("force"), sensor.get("zero_force")
    if not f or len(f) < 3:
        return None
    z = z if z and len(z) >= 3 else [0, 0, 0]
    return math.sqrt(sum(((a - b) / 1000.0) ** 2 for a, b in zip(f[:3], z[:3])))


class _FrameTap:
    """The kept feed's latest frame, read between streams (the wrist's load after a hand let go: RealArm.hand_load)."""

    def __init__(self):
        self.t_mono, self.frame = None, None

    def on_frame(self, t, frame):
        self.t_mono, self.frame = t, frame


class _StopWatch:
    """ALIX's CanfdWatch with Lite's STOP: the stream's stop check also trips when STOP was pressed, and when the wrist
    carries more than FORCE_CUT_N for FORCE_CUT_FRAMES frames (the owner, 26 Oct). It keeps the last line a check saw
    sent: a STOP raised inside run_verified leaves no result, and the lab read "0/397 lines" for a way back that had
    moved (23 Oct 15:08:34). ``tap`` also sees every frame (the arm's latest-frame reading)."""

    def __new__(cls, rows, period_s, stop, tap=None):
        from alix_dispatch import track

        class Watch(track.CanfdWatch):
            last_line = -1                    # checked every CANFD_CHECK_EVERY_S (10 lines): a count at most 9 short
            over = 0                          # frames in a row with the wrist over FORCE_CUT_N

            def on_frame(self, t, frame):
                super().on_frame(t, frame)
                if tap is not None:
                    tap(t, frame)
                load = wrist_load_n(frame)
                if load is None:
                    return
                self.over = self.over + 1 if load > FORCE_CUT_N else 0
                if self.over >= FORCE_CUT_FRAMES and self.tripped is None:
                    self.tripped = {"why": "force", "t_mono": t, "line": self.last_line,
                                    "detail": f"{load:.0f} N on the wrist for {FORCE_CUT_FRAMES * 5} ms "
                                              f"(a stream stops above {FORCE_CUT_N:.0f} N)"}

            def check(self, now, i_sent):
                self.last_line = max(self.last_line, int(i_sent))
                if stop.is_set() and self.tripped is None:
                    self.tripped = {"why": "stop", "t_mono": now, "line": i_sent, "detail": "STOP was pressed"}
                return self.tripped if self.tripped is not None else super().check(now, i_sent)

        return Watch(rows, period_s)


#: S2's recording of the controllers' feed (a diagnostic, asked per run): every 20 ms (the owner, 23 Oct: "let us use 20ms,
#: which is more than enough, that is 50Hz"). The 23 Oct pace test (tools/push_pace.py, the left arm, L-R01): programs play
#: 3.4 % slower with it on at 20 ms, 4.4 % at 10 ms (5 ms, 14-21 Oct: some 113 % of their lines' time).
S2_FEED_CYCLE = 4


def CycleRecorder(out_path, host_ip, port=None, log=print, cycle: int = 1):
    """ALIX's UdpRecorder pushing every ``cycle`` x 5 ms instead of every 5 ms (the owner, 23 Oct: "We can use 10ms or
    20ms"): with the push on at 5 ms, programs played some 13 % slower (21 Oct), past S2's 105 % stop. The config is
    ALIX's own, with its cycle set as it is written (``_apply_push``), and put back on stop exactly as ALIX's does.
    ``tools/push_pace.py`` measures what each cycle costs a program's pace."""
    from alix_dispatch import track

    class Recorder(track.UdpRecorder):
        def _apply_push(self, robot, cfg, want: dict, what: str):
            if what == "capture":
                cfg.cycle = int(cycle)
            return super()._apply_push(robot, cfg, want, what)

    def said(line):                              # ALIX's start line names its own 5 ms
        log(str(line).replace("cycle 5 ms", f"cycle {int(cycle) * 5} ms"))

    rec = Recorder(out_path, host_ip, port=port, log=said)
    rec.cycle = int(cycle)
    return rec


class _EmulatedCapture:
    """The 5 ms capture of an EMULATED arm only (``--robot emulated``): ALIX's emulator cannot push over UDP (it
    hands its frames to the SDK's callback), so its arm's own joints are read every 5 ms and fed to the stream's
    watch, as ALIX's own test of this lane does (alix_dispatch test_track_canfd_channel.py, ``_NoCapture``), and
    written to the capture file, marked emulated. A real arm's capture is always ALIX's UdpRecorder."""

    def __init__(self, out_path, host_ip, port=None, log=print, period_s: float = 0.005, **_):
        self.out_path, self.frames, self.monitor, self.period_s = Path(out_path), 0, None, float(period_s)
        self._stop, self._th, self._fh = threading.Event(), None, None

    def attach_monitor(self, monitor) -> None:
        self.monitor = monitor

    def start(self, robot) -> None:
        import json

        inner = getattr(robot, "_emulated_inner", robot)
        self._fh = open(self.out_path, "w", encoding="utf-8")

        def feed():
            while not self._stop.wait(self.period_s):
                try:
                    st = inner.rm_get_current_arm_state()[1]
                    frame = {"emulated": True, "err": [0], "arm_current_status": "run",
                             "joint_status": {"joint_position": [int(round(v * 1000)) for v in st["joint"]],
                                              "joint_err_code": [0] * 7}}
                except Exception:                                                   # noqa: BLE001
                    continue
                t = time.monotonic()
                self._fh.write(json.dumps({"t_mono": t, "frame": frame}, separators=(",", ":")) + "\n")
                self.frames += 1
                if self.monitor is not None:
                    self.monitor.on_frame(t, frame)

        self._th = threading.Thread(target=feed, name="lite-emulated-capture", daemon=True)
        self._th.start()

    def stop(self, robot) -> None:
        self._stop.set()
        if self._th is not None:
            self._th.join(timeout=1.0)
        if self._fh is not None:
            self._fh.close()


def _quick_pole(Pole):
    """The control program's Pole with Lite's arrival (the owner, 26 Oct): done when the controller reads the pole idle
    within its tolerance (LIFT_TOL_M, 5 mm) and the slew time plus POLE_RAMP_S has passed, instead of the model's 1.5 s
    creep allowance for the last millimetre (which held S2's pole steps 1.2 s after the pole stood: the 15:07 run,
    49 mm in 1.3 s, the step 2.54 s). The getter still leads a moving pole, so the slew floor stays."""
    from .staging import POLE_RAMP_S

    m = sys.modules.get(Pole.__module__)
    if not all(hasattr(m, k) for k in ("LIFT_TOL_M", "LIFT_STROKE_M", "LIFT_TIMEOUT_MARGIN", "LIFT_SLEW_UP_MPS", "LIFT_SLEW_DOWN_MPS", "lift_travel_s")):
        return Pole                                   # a stand-in Pole (the tests'): its own move

    class QuickPole(Pole):
        def move(self, height_m, speed_pct=100, tol_m=None, block=False):
            tol_m = m.LIFT_TOL_M if tol_m is None else tol_m
            start = self.height_m()
            dist = abs(height_m - start) if start is not None else m.LIFT_STROKE_M
            up = None if start is None else height_m > start
            slew = (m.LIFT_SLEW_DOWN_MPS if up is False else m.LIFT_SLEW_UP_MPS) * max(1, min(100, speed_pct)) / 100
            expected = POLE_RAMP_S + dist / slew
            timeout = m.LIFT_TIMEOUT_MARGIN * m.lift_travel_s(dist, up, speed_pct)
            if start is not None and dist <= tol_m:
                return True, start
            ret = self.robot.rm_set_lift_height(int(speed_pct), int(round(height_m * 1000)), 1 if block else 0)
            if ret != 0:
                raise RuntimeError(f"rm_set_lift_height refused, ret={ret}")
            t0, pos = time.monotonic(), start
            while True:
                elapsed = time.monotonic() - t0
                ret, st = self.robot.rm_get_lift_state()
                if ret == 0 and st:
                    pos = float(st["pos"]) / 1000.0
                    if int(st.get("err_flag", 0) or 0):
                        raise RuntimeError(f"pole err_flag {st['err_flag']}")
                    if int(st.get("mode", 0) or 0) == 0 and abs(pos - height_m) <= tol_m and elapsed >= expected:
                        return True, pos
                if elapsed > timeout:
                    self.stop()
                    return False, pos
                time.sleep(self.poll_s)

    return QuickPole


def _hand_arrived(hand, rule, target: list, sent: float) -> tuple:
    """The hand of S2's pole and hand (RealArm.pole_and_hand) by runner v14's own rule (the control program's
    Hand.command, its numbers from its hand.py ``rule``): arrived (each channel set within HAND_ARRIVE_COUNTS of its
    target) or settled (two reads within HAND_SETTLE_COUNTS once a whole stroke's time has passed), timed from the command
    ``sent`` before the pole moved: the hand moved meanwhile, so it is neither sent again nor waited for from the start
    (24 Oct: that took 0.6-1.0 s after every pole, 1.0-1.3 s of a step with no pole travel). (False, the last read) when
    neither in three strokes' time and a second, as Hand.command's own limit."""
    stroke = rule.HAND_LATENCY_S + rule.HAND_STROKE_K * 1000.0 / (hand.speed_set or 1) / 1000.0
    prev, cur = None, None
    while True:
        cur = hand.read()
        elapsed = time.monotonic() - sent
        if cur is not None:
            if all(t is None or abs(int(a) - int(t)) <= rule.HAND_ARRIVE_COUNTS for a, t in zip(cur, target)):
                return True, cur
            if (prev is not None and elapsed >= stroke
                    and all(abs(int(a) - int(b)) <= rule.HAND_SETTLE_COUNTS for a, b in zip(cur, prev))):
                return True, cur
            prev = cur
        if elapsed > 3.0 * stroke + 1.0:
            return False, cur
        time.sleep(hand.poll_s)


def _interrupt(tid: int, exc=KeyboardInterrupt) -> None:
    """``exc`` raised in thread ``tid`` at its next Python step (None: forget one not yet raised)."""
    import ctypes

    ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_ulong(tid), None if exc is None else ctypes.py_object(exc))


class RealArm:
    def __init__(self, link, log=None):
        self.link, self.log, self.side = link, log, link.side
        self.SIDE = link.side.upper()
        self._feed = None            # (recorder, capture): the 5 ms feed kept on across a lid or seat run (keep_feed)
        self._tap = _FrameTap()      # the kept feed's latest frame (hand_load)
        self._hand = None            # ALIX's HandModbus opened once for a lid or seat run (keep_hand)
        self._hand_speeds = None     # the SPEED_SET each channel of the kept hand holds
        self._pole_done = None       # when the last pole move ended (the controller's warm start)
        self._ratio = None           # the global ratio this run left on the controller (None: not known)
        self._s2_feed = None         # S2's recording of the feed, kept across a stage's runner calls (keep_s2_feed)

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
            if self._hand is not None:               # a lid or seat run's hand, opened once (keep_hand), at the staging speed
                from alix_dispatch import peripherals

                pole, hand, opened = peripherals.Pole(self.robot, log=log), self._hand, {"ok": True}
                self._hand_speeds_to(hand, [HAND_SPEED_SET] * 6)
            else:
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

    def pole_to(self, mm: float, duration_s: float | None = None) -> dict:
        """The pole alone to ``mm`` (Prepare's Apply beside the pole, Advanced, a lid or seat plan's pole entry), arrival
        verified; nothing else moves. At 50 %, or, for a plan's pole move (``duration_s``: its planned time), at the
        speed ALIX's executive sets from that time (device_speed.pole_speed_pct; the owner, 24 Oct). A pole standing
        idle within its 5 mm goal tolerance of ``mm`` is not commanded, as ALIX's stage_arm does: every controller code
        4117 (undocumented) came within a second of a pole command, three of them a command to where the pole stood
        (lid open's 150 -> 150 mm; the owner, 23 Oct: test it)."""
        from alix_dispatch import peripherals

        pct = POLE_SPEED_PCT
        with self.link.busy("moving the pole"):
            pole = peripherals.Pole(self.robot, log=self._alix_log())
            st = pole.state()
            here = None if st is None else float(st["pos"]) / peripherals.LIFT_UNITS_PER_METRE
            idle = st is not None and not int(st.get("mode", 0) or 0) and not int(st.get("err_flag", 0) or 0)
            if idle and abs(here - float(mm) / 1000.0) <= peripherals.LIFT_TOL_M:
                pole.log(f"   pole: already at {here * 1000:.0f} mm, the {float(mm):.0f} mm asked (within "
                         f"{peripherals.LIFT_TOL_M * 1000:.0f} mm, idle): nothing sent")
                out = dict(ok=True, ret=None, target_m=float(mm) / 1000.0, start_m=here, final_m=here, t_s=0.0,
                           verified_by="no_op", no_op=True, no_op_source="getter", sent=False)
            else:
                if duration_s is not None and here is not None:
                    from .device_speed import pole_speed_pct

                    warm = self._pole_done is not None and time.monotonic() - self._pole_done < 2.0
                    pct, how = pole_speed_pct(self.side, here, float(mm) / 1000.0, float(duration_s), warm=warm)
                    pole.log(f"   pole: {pct} % ({how})")
                out = pole.move(float(mm) / 1000.0, speed_pct=pct)
                self._pole_done = time.monotonic()
        self._hw("pole move", args=dict(mm=mm, speed_pct=pct), ret=out.get("ret"), ok=bool(out.get("ok")),
                 elapsed_s=out.get("t_s"), text=f"final {out.get('final_m')} m, verified by {out.get('verified_by')}"
                 + ("; nothing sent: already there" if out.get("sent") is False else ""))
        if not out.get("ok"):
            raise UserError(f"The {self.SIDE} pole did not reach {mm:.0f} mm ({out.get('error') or return_text(out.get('ret'))}).",
                            "Check the pole, then try again.")
        return out

    def pole_and_hand(self, mm: float, values: list, stop_hook=None) -> dict:
        """S2's Pole row with the Hand row straight after it, as one step (the owner, 22 Oct: "there is a delay when the
        sequence moves from the pole movement to the hand movement"), and a Program's own pole and hand (24 Oct): the
        hand is sent first and moves while the pole travels; the pole as runner v14 moves it (the control program's
        Pole.move at 100 %: done when idle at its height and its travel model has run out, as the position it reads leads
        the pole by up to 73 mm); then the hand confirmed by runner v14's own rule (Hand.command: arrived, or settled on
        force), timed from the command it moved by (``_hand_arrived``; sent again only when it has not arrived). A hand
        channel None holds where it is. The arm does not move. STOP (``stop_hook``) stops the pole and holds the hand
        where it is."""
        Hand, _counts, Pole = _runner().load_aux_classes()
        rule = next(m for c in Hand.__mro__ if hasattr(m := sys.modules.get(c.__module__), "HAND_ARRIVE_COUNTS"))
        # ^ the control program's hand.py: its arrival rule's own numbers
        target = [None if v is None else int(v) for v in values]
        tid, guard, state = threading.get_ident(), threading.Lock(), {"inside": False, "stopped": False}

        def on_stop():
            with guard:
                state["stopped"] = True
                if state["inside"]:
                    _interrupt(tid)

        if stop_hook is not None:
            stop_hook(on_stop)
        t0 = time.monotonic()
        hand = pole = None
        out = dict(pole_ok=False, hand_ok=False)
        why = None
        with self.link.busy("moving the pole and the hand", expect_s=60):
            try:
                try:
                    with guard:
                        if state["stopped"]:
                            raise KeyboardInterrupt
                        state["inside"] = True
                    hand, pole = Hand(self.robot), _quick_pole(Pole)(self.robot)
                    start = pole.height_m()
                    hand.open()
                    sent = time.monotonic()
                    hand.command(target, wait=False)                 # the hand moves while the pole travels
                    ok_p, final_m = pole.move(float(mm) / 1000.0, speed_pct=100, block=False)
                    out.update(pole_ok=bool(ok_p), pole_s=round(time.monotonic() - t0, 3), start_mm=None if start is None else round(start * 1000.0, 1),
                               pole_mm=None if final_m is None else round(final_m * 1000.0, 1))
                    if ok_p:
                        ok_h, final_h = _hand_arrived(hand, rule, target, sent)
                        if not ok_h:                                 # not there in a whole stroke's time: as runner v14 does
                            ok_h, final_h = hand.command(target, wait=True)
                            out["hand_sent_again"] = True
                        out.update(hand_ok=bool(ok_h), hand=final_h)
                finally:
                    with guard:
                        state["inside"] = False
                        stopped = state["stopped"]
                    if stopped:                       # a stop that came as it ended is not raised later
                        _interrupt(tid, None)
            except KeyboardInterrupt:
                for what, fn in (("pole stop", lambda: pole.stop() if pole is not None else None),
                                 ("hand freeze", lambda: hand.freeze() if hand is not None else None)):
                    try:
                        self._hw(what, ret=fn())
                    except Exception as exc:                                    # noqa: BLE001
                        self._hw(what, ok=False, text=str(exc))
                why = "stop"
            except Exception as exc:                                            # noqa: BLE001 - the pole or the hand refused
                why = str(exc)
            finally:
                try:
                    if hand is not None:
                        hand.close()
                except Exception:                                               # noqa: BLE001
                    pass
        out["t_s"] = round(time.monotonic() - t0, 3)
        ok = why is None and out["pole_ok"] and out["hand_ok"]
        self._hw("pole and hand", args=dict(mm=float(mm), hand=target), ok=ok, elapsed_s=out["t_s"],
                 text=(f"pole {out.get('start_mm')} -> {out.get('pole_mm')} mm in {out.get('pole_s')} s, the hand moving meanwhile; "
                       f"hand {out.get('hand')}") if why is None else f"stopped: {why}")
        if why == "stop":
            raise Cancelled()
        if not ok:
            raise UserError(f"The {self.SIDE} pole and hand did not reach {float(mm):g} mm and "
                            f"{','.join('hold' if v is None else str(v) for v in target)} "
                            f"({why or ('the pole did not arrive' if not out['pole_ok'] else 'the hand did not arrive')}).",
                            "Check the pole and the hand, then run it again.")
        return out

    def keep_hand(self) -> bool:
        """The hand's Modbus mode and its speed set once for a lid or seat run: its hand moves and stagings then command
        it at once (24 Oct: opening it again before each of a run's 13 hand moves took about 0.25 s each). False when
        it cannot be opened: each hand move then opens it as before, and says why."""
        if self._hand is not None:
            return True
        from alix_dispatch import peripherals

        with self.link.busy("opening the hand"):
            hand = peripherals.HandModbus(self.robot, self.side, log=self._alix_log())
            ret = hand.open(speed_set=HAND_SPEED_SET)
        if ret != 0:
            self._hw("hand open", ret=ret, ok=False, text=return_text(ret))
            return False
        self._hand, self._hand_speeds = hand, [HAND_SPEED_SET] * 6
        return True

    def drop_hand(self) -> None:
        """The run's hand let go (its Modbus mode stays on, as every hand move leaves it): the next command opens it."""
        self._hand = self._hand_speeds = None

    def _hand_speeds_to(self, hand, sets: list, why: str = "") -> None:
        """The hand's channels at these SPEED_SETs: one Modbus write, none when the kept hand holds them already. A
        write that fails is said; the move then goes at the speeds the hand holds (its arrival is read back anyway)."""
        from alix_dispatch import peripherals

        sets = [int(v) for v in sets]
        if hand is self._hand and self._hand_speeds == sets:
            return
        if len(set(sets)) == 1:
            ok = hand.set_speed(sets[0]) == 0                 # "SPEED_SET n on all six channels"
        else:
            rep = hand.write_report(peripherals.HAND_REG_SPEED_SET, sets)
            ok = bool(rep.get("ok"))
            if ok:                                    # the stroke law's wait: the slowest channel's (longer, never shorter)
                hand.speed_set = min(sets)
            hand.log(f"   hand: SPEED_SET {sets}" + (f" ({why})" if why else "")
                     + ("" if ok else f" — write FAILED ret={rep.get('ret')}"))
        if hand is self._hand:
            self._hand_speeds = sets if ok else None

    def hand_to(self, counts: list, duration_s: float | None = None) -> dict:
        """The hand alone to six counts, read back: at SPEED_SET 500 (Advanced), or, for a plan's hand move
        (``duration_s``: its planned time), at the speeds ALIX's executive sets from that time, channel by channel
        (device_speed.hand_speed_sets; the owner, 24 Oct). Done as staging takes it (_settled: the control program's
        Hand.command): ARRIVED, or SETTLED short of the target, fingers stopped on force (on the lab's hand, 14 Oct, the
        thumb stopped 714 counts short)."""
        from alix_dispatch import peripherals

        with self.link.busy("moving the hand"):
            hand = self._hand
            if hand is None:
                hand = peripherals.HandModbus(self.robot, self.side, log=self._alix_log())
                ret = hand.open(speed_set=HAND_SPEED_SET)
                if ret != 0:
                    self._hw("hand open", ret=ret, ok=False, text=return_text(ret))
                    raise UserError(f"The {self.SIDE} hand cannot be reached (Modbus).", "Check the hand's cable and power.")
            from .staging import HAND_SAME_COUNTS

            now = getattr(hand, "read", lambda: None)()
            if now is not None and all(c is None or abs(int(c) - int(a)) <= HAND_SAME_COUNTS for c, a in zip(counts, now)):
                self._hw("hand move", args=dict(counts=list(counts), speed_set=None), ok=True, elapsed_s=0.0,
                         text=f"already at {now} (within {HAND_SAME_COUNTS} counts), nothing sent")
                return dict(ok=True, arrived=True, after=now, skipped=True)
            used = HAND_SPEED_SET
            if duration_s is not None:
                from .device_speed import hand_speed_sets

                used, how = hand_speed_sets(self.side, now, list(counts), float(duration_s))
                self._hand_speeds_to(hand, used, how)
            elif hand is self._hand:
                self._hand_speeds_to(hand, [HAND_SPEED_SET] * 6)
            out = hand.command(list(counts), wait=True)
        done = bool(out.get("ok") and (out.get("arrived") or out.get("settled")))
        short = "" if out.get("arrived") or not done else " (settled short of the target: stopped on force)"
        self._hw("hand move", args=dict(counts=list(counts), speed_set=used), ret=out.get("ret"),
                 ok=done, elapsed_s=out.get("t_s"), text=f"read back {out.get('after')}{short}")
        if not done:
            raise UserError(f"The {self.SIDE} hand did not reach its pose (it reads {out.get('after')}).",
                            "Check the hand, then try again.")
        return out

    # ── the controller's run mode
    def set_run_mode(self, mode: str) -> dict:
        """SIM or REAL, read back. Returns what to hand to ``restore_run_mode``. A controller that does not
        take it is asked once more a second later (the lab, 17 Oct: the RIGHT controller refused REAL right
        after a busy moment, its global point list unanswered, and took it seconds later); the read back
        stays the gate."""
        from alix_dispatch import runmode

        target = runmode.SIMULATION if mode == "SIM" else runmode.REAL
        with self.link.busy(f"switching to {mode}"):
            originals = runmode.apply_run_mode(target, {self.side: self.robot})
            if originals is None:
                time.sleep(1.0)
                originals = runmode.apply_run_mode(target, {self.side: self.robot})
        self._hw(f"run mode {mode}", ok=originals is not None, text=None if originals is not None else "not taken (read back)")
        if originals is None:
            raise UserError(f"The {self.SIDE} controller did not switch to its {mode} mode (read back), so nothing ran.",
                            "Check the controller's errors on its pendant.")
        return originals

    def restore_run_mode(self, originals: dict) -> None:
        """The run mode the controller had, put back (a job's ``finally``): it waits for a command or a STOP
        still holding the controller (at most CLOSE_WAIT_S) and never raises, so the job's own ending is
        what is said."""
        from alix_dispatch import runmode

        try:
            with self.link.busy("restoring the run mode", wait_s=CLOSE_WAIT_S):
                failed = runmode.restore_run_modes(originals, {self.side: self.robot})
        except Exception as exc:                                                    # noqa: BLE001
            failed = [f"{self.side}: not restored ({exc})"]
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
        SDK, as the control program's files are). Returns its number of points. Leaving it is never
        refused: a command or a STOP holding the controller is waited for (at most CLOSE_WAIT_S), and
        then drag teaching is left at once, as STOP leaves it, without the recording. An arm that may
        still be in drag teaching raises ``code="drag"``."""
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        try:
            with self.link.busy("saving the recording", wait_s=CLOSE_WAIT_S, expect_s=120):
                ret = self._call("rm_stop_drag_teach", self.robot.rm_stop_drag_teach)
                out = self._call("rm_save_trajectory", self.robot.rm_save_trajectory, str(path))
        except UserError as exc:
            if exc.code != "busy":
                raise
            ret = self._call("rm_stop_drag_teach", self.link.leave_drag)
            if ret != 0:
                raise UserError(f"The {self.SIDE} arm may still be in drag teaching ({return_text(ret, 'rm_stop_drag_teach')}).",
                                "Press STOP, or the red E-stop button, before anyone lets go of the arm.", code="drag") from None
            raise UserError(f"The {self.SIDE} arm left drag teaching, but its recording could not be saved. {exc}",
                            "Record the region again.") from None
        if ret != 0:
            raise UserError(f"The {self.SIDE} arm may still be in drag teaching ({return_text(ret, 'rm_stop_drag_teach')}).",
                            "Press STOP, or the red E-stop button, before anyone lets go of the arm.", code="drag")
        r, n = out if isinstance(out, tuple) else (out, None)
        if r != 0 or not Path(path).is_file() or Path(path).stat().st_size == 0:
            raise UserError(f"The {self.SIDE} controller did not save the recording ({return_text(r, 'rm_save_trajectory')}).",
                            "Record the region again.")
        return n

    def teach_cancel(self, at_once: bool = False) -> bool:
        """Leave drag teaching without saving (an E-stop, a cancel, Lite closing). Never refused: a
        command holding the controller is waited for (at most CLOSE_WAIT_S), then it is sent as STOP
        sends its calls; ``at_once`` (an E-stop): so at once. Never raises; True when the controller
        took it."""
        try:
            if not at_once:
                try:
                    with self.link.busy("leaving drag teaching", wait_s=CLOSE_WAIT_S):
                        out = self._call("rm_stop_drag_teach", self.robot.rm_stop_drag_teach)
                    return (out[0] if isinstance(out, tuple) else out) == 0
                except UserError as exc:
                    if exc.code != "busy":
                        raise
            return self._call("rm_stop_drag_teach", self.link.leave_drag) == 0
        except Exception:                                                           # noqa: BLE001
            return False

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

    def clear_warning(self, warn_only=frozenset({"4105"})) -> dict:
        """Before a move that waits for its arrival (S2: a Global Point, a lid or seat task): the arm's errors read
        (``dual_arm_common.error_state``) and a warning alone (``warn_only``: 4105, a joint over its speed limit, which a
        Program raises now and then and runner v14 lets stand) cleared as the controller web GUI's Clear Error does, then
        read back; the SDK's blocking move returns 1 at once while any code is latched (workflow.WARN_ONLY). Anything
        else latched is not cleared. Nothing moves. Returns {readable, cleared: [codes], other: what is latched that is
        not a warning, or None}."""
        dac = _dac()
        with self.link.busy("checking the errors before the move"), _printed_to(lambda line: self._hw("clear warning", text=line.strip())):
            st = dac.error_state(self)
            if not st["readable"] or dac.error_state_clean(st):      # said every time: the logs show the check ran
                self._hw("clear warning", ok=bool(st["readable"]), text="clean" if st["readable"] else "the errors could not be read")
                return dict(readable=bool(st["readable"]), cleared=[], other=None)
            if set(st["sys"]) - set(warn_only) or st["joints"] or st.get("lift_err") or st.get("disabled"):
                self._hw("clear warning", ok=False, text=f"not cleared, not a warning: {dac.describe_error_state(st)}")
                return dict(readable=True, cleared=[], other=dac.describe_error_state(st))
            ok = dac.clear_errors(self)
            back = dac.error_state(self)
        clean = bool(ok) and dac.error_state_clean(back)
        self._hw("clear warning", args=dict(codes=st["sys"]), ok=clean, text="cleared" if clean else dac.describe_error_state(back))
        return dict(readable=True, cleared=list(st["sys"]) if clean else [], other=None if clean else dac.describe_error_state(back))

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
        with self.link.busy(f"sending program {pid}", expect_s=300), _printed_to(self._printed):
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
            with self.link.busy("putting the global speed back", wait_s=CLOSE_WAIT_S):
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

        with self.link.busy(f"running program {pid}", expect_s=timeout), _printed_to(line_out):
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
                     stop_hook=None, record: Path | None = None) -> dict:
        """An arm's part of a Page 02 stage through runner v14's own ``run_workflow_sequence``, as V53
        runs it in a process of its own: one connection; the slots read once (a missing one refuses it
        all), the arm's errors and run state checked, the overall speed set, each step (Program, Pole,
        Hand, Wait, Global Point) run and checked, the speed put back to ``restore_pct`` in its
        ``finally``. ``stop_hook(fn)`` registers ``fn`` for STOP, which interrupts the runner as V53's
        STOP does (Ctrl+C: its own slow stop of the arm, the pole stopped, the hand frozen). ``record``: the
        controller's feed recorded to that file around the call (S2_FEED_CYCLE; the result's ``feed``), unless a
        recording kept for the stage (``keep_s2_feed``) is on."""
        self.drop_feed()                             # a lid or seat run's kept feed never stays on under a program
        rt = _runner()
        norm = [rt.normalize_workflow_step(st) for st in steps]
        a = SimpleNamespace(yes=True, side=self.side, workflow_overall_speed=int(overall_pct), restore_speed=int(restore_pct),
                            timeout=float(timeout_s))
        log = io.StringIO()
        t0 = time.monotonic()

        tid, guard = threading.get_ident(), threading.Lock()
        state = {"inside": False, "stopped": False, "talking": False, "held": False, "raised": False}

        def line_out(line):
            log.write(line + "\n")
            self._runner_line(line)
            if on_line is None:
                return
            with guard:
                state["talking"] = True
            try:
                on_line(line)
            finally:
                with guard:
                    state["talking"] = False
                    held, state["held"] = state["held"], False
            if held:                                  # the STOP that came while the server acted on this line
                raise KeyboardInterrupt

        def stop():
            # Raised once: a second one would cut the runner's own handling of the first (its slow stop). Never in the
            # middle of a line's handshake with the server (robot_process: the line sent, its ack read back): one raised
            # there left the ack unread, the arm's worker read it as its next command and ended (the lab, 21 Oct 23:04,
            # 23:14 and 23:18: Lite's early-end check stopped the RIGHT arm from its own line, three times); it is held
            # and raised when the line is done.
            with guard:
                state["stopped"] = True
                if not state["inside"] or state["raised"]:
                    return
                state["raised"] = True
                if state["talking"]:
                    state["held"] = True
                else:
                    _interrupt(tid)

        if stop_hook is not None:
            stop_hook(stop)
        rc, feed, fed = 1, None, None
        with self.link.busy(f"sequence of {len(norm)} step(s)", expect_s=float(timeout_s) * max(1, len(norm))), _printed_to(line_out):
            if record is not None and self._s2_feed is None:
                feed = self._feed_start(Path(record), line_out)
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
            finally:
                fed = self._feed_stop(feed)
        said = log.getvalue()
        why = next((ln.strip(" *") for ln in reversed(said.splitlines()) if ln.strip().startswith("**")), None)
        self._hw("workflow", args=dict(steps=len(norm), overall_speed=int(overall_pct), restore_speed=int(restore_pct)),
                 ret=rc, ok=rc == 0, elapsed_s=time.monotonic() - t0, text=why)
        return dict(ok=rc == 0, rc=rc, why=why, stopped=state["stopped"], seconds=round(time.monotonic() - t0, 2),
                    lines=said.splitlines(), feed=fed)

    def _feed_start(self, cap: Path, say):
        """S2's recording of this controller's feed, every S2_FEED_CYCLE x 5 ms (ALIX's UdpRecorder at that cycle; on an
        emulated arm, its joints read as the stream's capture does). One that cannot start is said ("!!", the job's log)
        and the run goes on without it: the recording is a diagnostic."""
        cap.parent.mkdir(parents=True, exist_ok=True)
        h = self.handle
        try:
            if self.link.emulated:
                rec = _EmulatedCapture(cap, h.host_ip, port=UDP_PORT[self.side], log=self._alix_log(),
                                       period_s=S2_FEED_CYCLE * 0.005)
            else:
                rec = CycleRecorder(cap, h.host_ip, port=UDP_PORT[self.side], log=self._alix_log(), cycle=S2_FEED_CYCLE)
            rec.start(self.robot)
        except Exception as exc:                                                    # noqa: BLE001
            self._hw("s2 feed", ok=False, text=f"not recorded: {exc}")
            say(f"!! {self.SIDE}: its feed is not recorded for this part ({exc})")
            return None
        self._hw("s2 feed", args=dict(cycle_ms=S2_FEED_CYCLE * 5, file=cap.name), ok=True, text="recording")
        return rec

    def _feed_stop(self, rec) -> dict | None:
        """The recording stopped (the controller's push put back as it was); what it kept."""
        if rec is None:
            return None
        try:
            rec.stop(self.robot)
        except Exception as exc:                                                    # noqa: BLE001
            self._hw("s2 feed", ok=False, text=f"stopping it: {exc}")
        n = int(getattr(rec, "frames", 0) or 0)
        self._hw("s2 feed", args=dict(file=Path(rec.out_path).name), ok=n > 0, text=f"{n} frames")
        return dict(file=str(rec.out_path), frames=n, cycle_ms=S2_FEED_CYCLE * 5)

    def keep_s2_feed(self, cap: Path) -> str | None:
        """S2's recording of this controller's feed (``_feed_start``) on for a whole stage, its runner calls and Lite's own
        steps between them, until ``drop_s2_feed`` (24 Oct: turned on and off around each runner call, it took 0.46 s to
        start and as long to stop at every call). Returns its file's name; None when it could not start (said in the
        hardware log; the run goes on, the recording is a diagnostic)."""
        if self._s2_feed is None:
            self.drop_feed()                         # a lid or seat run's 5 ms feed never stays on under a stage
            with self.link.busy("turning on the controller's feed"):
                self._s2_feed = self._feed_start(Path(cap), lambda _text: None)
        return None if self._s2_feed is None else Path(self._s2_feed.out_path).name

    def drop_s2_feed(self) -> dict | None:
        """The stage's recording stopped (the controller's push put back as it was): what it kept. Never raises."""
        rec, self._s2_feed = self._s2_feed, None
        if rec is None:
            return None
        try:
            with self.link.busy("turning off the controller's feed", wait_s=CLOSE_WAIT_S):
                return self._feed_stop(rec)
        except Exception as exc:                                                    # noqa: BLE001
            self._hw("s2 feed", ok=False, text=f"not stopped ({exc})")
            return None

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
        with self.link.busy(f"{'MoveL' if move == 'movel' else 'MoveJ'} to {name}", expect_s=120), _printed_to(self._runner_line):
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

    # ── deleting what the controller stores (the owner, 21 Oct: "Add a delete feature for anything stored on the
    #    controller: uploaded projects, global points, any other stored content"). Neither the control program nor
    #    ALIX deletes, so these are RealMan's own delete calls, each read back; the page asks first, every time.
    def delete_program(self, program_id: int) -> dict:
        """Program ``program_id`` deleted (``rm_delete_program_trajectory``), then the program list read again: gone."""
        pid = int(program_id)
        t0, after = time.monotonic(), None
        with self.link.busy(f"deleting program {pid}"), _printed_to(self._printed):
            held = self._slots()
            if pid not in held:
                raise UserError(f"The {self.SIDE} controller holds no program {pid}.", "Read the controller again.")
            name = held[pid][0]
            rc = self._sdk("rm_delete_program_trajectory")(pid)
            if rc == 0:
                try:
                    after = self._slots()
                except UserError:
                    after = None
        ok = rc == 0 and after is not None and pid not in after
        self._hw("delete program", args=dict(program_id=pid, name=name), ret=rc, ok=ok, elapsed_s=time.monotonic() - t0,
                 text=None if ok else return_text(rc, "delete program") if rc != 0 else
                 "the list could not be read back" if after is None else "still listed")
        if rc != 0:
            raise UserError(f"Program {pid} ({name}) was not deleted from the {self.SIDE} controller ({return_text(rc, 'delete program')}).",
                            "Check the controller, then try again.")
        if after is not None and pid in after:
            raise UserError(f"The {self.SIDE} controller still lists program {pid} ({name}) after deleting it.", "Read the controller again.")
        return dict(program_id=pid, name=name, read_back=after is not None)

    def delete_global_point(self, name: str) -> dict:
        """Global point ``name`` deleted (``rm_delete_global_waypoint``), then asked for again: gone."""
        rt = _runner()
        t0, gone = time.monotonic(), False
        with self.link.busy(f"deleting global point {name}"), _printed_to(self._runner_line):
            try:
                rt.get_global_waypoint(self.robot, name)
            except Exception:                                                   # noqa: BLE001 - not on this controller
                raise UserError(f"“{name}” is not a global point on the {self.SIDE} controller.", "Read the controller again.") from None
            rc = self._sdk("rm_delete_global_waypoint")(name)
            if rc == 0:
                try:
                    rt.get_global_waypoint(self.robot, name)
                except Exception:                                               # noqa: BLE001 - gone, as asked
                    gone = True
        ok = rc == 0 and gone
        self._hw("delete global point", args=dict(name=name), ret=rc, ok=ok, elapsed_s=time.monotonic() - t0,
                 text=None if ok else return_text(rc, "delete global point") if rc != 0 else "still read back")
        if not ok:
            raise UserError(f"Global point “{name}” was not deleted from the {self.SIDE} controller"
                            + (f" ({return_text(rc, 'delete global point')})." if rc != 0 else ": it still reads back."),
                            "Check the controller, then try again.")
        return dict(name=name)

    def delete_track(self, name: str) -> dict:
        """A drag-teaching track deleted (``rm_delete_trajectory_file``: 4th-generation controllers only), then the
        tracks list read again: gone."""
        t0, listed = time.monotonic(), None
        with self.link.busy(f"deleting track {name}"):
            rc = self._sdk("rm_delete_trajectory_file")(name)
            if rc == 0:
                listed = self._tracks()
        gone = rc == 0 and listed is not None and listed.get("available") and not any(t.get("name") == name for t in listed["list"])
        self._hw("delete track", args=dict(name=name), ret=rc, ok=bool(gone), elapsed_s=time.monotonic() - t0,
                 text=None if gone else return_text(rc, "delete track") if rc != 0 else "still listed")
        if rc == -4:
            raise UserError("RealMan's API deletes tracks on 4th-generation controllers only; this one is 3rd generation.",
                            "Delete it on the arm's own web page.")
        if not gone:
            raise UserError(f"Track “{name}” was not deleted from the {self.SIDE} controller"
                            + (f" ({return_text(rc, 'delete track')})." if rc != 0 else ": it is still listed."),
                            "Check the controller, then try again.")
        return dict(name=name)

    # ── ALIX's CanFD lane: a lid or seat plan's arm move, streamed from this computer, one line every 5 ms
    def _capture(self, label: str, records: Path | None):
        """ALIX's capture of this controller's 5 ms push (``track.UdpRecorder``, a port per arm; an emulated arm: its
        joints read every 5 ms), started: (recorder, path). Runs inside the link's busy(). Raises UserError when it
        cannot be set up: nothing was streamed."""
        from alix_dispatch import track

        folder = Path(records) if records else Path(tempfile.gettempdir()) / "lite-canfd"
        folder.mkdir(parents=True, exist_ok=True)
        slug = "".join(ch if ch.isalnum() else "_" for ch in label)[:60]
        cap = folder / f"{time.strftime('%Y%m%d_%H%M%S')}_{slug}.frames.jsonl"
        alog = self._alix_log()

        def log(*parts):
            alog(" ".join(str(p) for p in parts))

        capture = _EmulatedCapture if self.link.emulated else track.UdpRecorder
        rec = capture(cap, self.handle.host_ip, port=UDP_PORT[self.side], log=log)
        try:
            rec.start(self.robot)
        except Exception as exc:                                                # noqa: BLE001
            self._hw("canfd capture", ok=False, text=f"refused: {exc}")
            from .udp_push import capture_refused_hint       # who holds the port: this connection, or another program

            hint = capture_refused_hint(UDP_PORT[self.side]) if "already bound" in str(exc) else None
            raise UserError(f"The {self.SIDE} controller's 5 ms feed to this computer could not be set up ({exc}), so "
                            "nothing was streamed.", hint or "Check that nothing else (ALIX's driver) takes the controller's push.") from None
        return rec, cap

    def keep_feed(self, label: str, records: Path | None = None) -> str:
        """The controller's 5 ms push turned on and kept on, recorded on this computer, until ``drop_feed``: a lid or seat
        run's moves attach their watches to it one after another, as ALIX's recorder is made to be used ("opened once
        per RUN", a monitor per stroke), instead of turning it on and off around each move: four controller calls,
        about 1.1 s a move on 24 Oct (12 moves, 13 s of a 79 s seat run). Returns the capture's name."""
        if self._feed is None:
            self.drop_s2_feed()                      # an S2 stage's recording never stays on under a lid or seat run
            self._ratio = None
            with self.link.busy(f"turning on the 5 ms feed ({label})"):
                self._feed = self._capture(label, records)
            self._feed[0].attach_monitor(self._tap)
            self._hw("canfd feed", args=dict(label=label, capture=self._feed[1].name), ok=True, text="on, kept for the run's moves")
        return self._feed[1].name

    def hand_load(self) -> dict | None:
        """The wrist's load now (N) from the kept feed's latest frame, and that frame's age, or None without a feed or
        a six-axis reading (an emulated arm)."""
        t, frame = self._tap.t_mono, self._tap.frame
        load = None if frame is None else wrist_load_n(frame)
        if load is None:
            return None
        return dict(load_n=round(load, 1), age_s=round(time.monotonic() - t, 2))

    def _stream_end(self, last_row_millideg: list, t_after: float | None) -> dict:
        """The stream's end, read from the feed's frames instead of ALIX's six polls of the controller (the owner, 26 Oct):
        at rest when END_FRAMES frames in a row, taken after the last line went (``t_after``), show every joint within
        END_TOL_DEG of the last line and under END_STILL_DEG_S (the frame's own speed, and the step between frames).
        When the feed does not show it within END_WAIT_S, ALIX's ``wait_until_at_rest`` takes over, as before. Returns
        {'waited_s', 'at_rest', 'gap_deg', 'peak_deg_s', 'by': 'feed' | 'polled'}; recorded, never raised on."""
        from alix_dispatch import track

        t0 = time.monotonic()
        last = [float(v) / 1000.0 for v in last_row_millideg]
        seen, prev, quiet, peak, gap = None, None, 0, 0.0, None
        while time.monotonic() - t0 < END_WAIT_S:
            t, frame = self._tap.t_mono, self._tap.frame
            js = (frame or {}).get("joint_status") or {}
            pos = js.get("joint_position")
            if t is None or t == seen or (t_after is not None and t < t_after) or not pos:
                time.sleep(0.002)
                continue
            seen = t
            pos = [float(v) / 1000.0 for v in pos]
            gap = max(abs(a - b) for a, b in zip(pos, last))
            speed = max((abs(float(v)) / 1000.0 for v in (js.get("joint_speed") or [])), default=0.0)
            if prev is not None:
                speed = max(speed, max(abs(a - b) for a, b in zip(pos, prev[1])) / max(t - prev[0], 1e-3))
            prev, peak = (t, pos), max(peak, speed)
            quiet = quiet + 1 if gap <= END_TOL_DEG and speed < END_STILL_DEG_S else 0
            if quiet >= END_FRAMES:
                return dict(waited_s=round(time.monotonic() - t0, 3), at_rest=True, gap_deg=round(gap, 3), peak_deg_s=round(peak, 2), by="feed")
            time.sleep(0.002)
        rest = track.wait_until_at_rest(self.robot)
        return dict(waited_s=round(float(rest.get("waited_s") or 0.0) + (time.monotonic() - t0), 3), at_rest=bool(rest.get("at_rest")),
                    gap_deg=None if gap is None else round(gap, 3), peak_deg_s=round(float(rest.get("peak_deg_s") or 0.0), 2), by="polled")

    def drop_feed(self) -> dict | None:
        """The kept feed turned off: the controller's push put back as it was found (ALIX's recorder). Never raises; one
        that cannot be turned off now is kept, for the next drop_feed (and the next connection clears it)."""
        kept, self._feed, self._ratio = self._feed, None, None
        if kept is None:
            return None
        rec, cap = kept
        rec.attach_monitor(None)
        try:
            with self.link.busy("turning off the 5 ms feed", wait_s=CLOSE_WAIT_S):
                rec.stop(self.robot)
        except Exception as exc:                                                    # noqa: BLE001
            self._feed = kept
            self._hw("canfd feed", args=dict(capture=cap.name), ok=False, text=f"not turned off ({exc})")
            return None
        restored = getattr(rec, "push_restored", None) or {}
        self._hw("canfd feed", args=dict(capture=cap.name), ok=restored.get("ok") is not False,
                 text=f"off, {rec.frames} frames kept")
        return dict(capture=cap.name, frames=int(rec.frames))

    def stream(self, rows_millideg: list, label: str, *, speed: int = 50, restore_pct: int = RESTORE_PCT,
               bridge_speed: int | None = None, stop_hook=None, records: Path | None = None) -> dict:
        """One arm move of ALIX's lid and seat plans (mechanism.py), as ALIX's ROS-free lane plays a track
        (cleaning_tracks.run_one): the controller's 5 ms push captured on this computer (``track.UdpRecorder``, a
        port per arm: the run's kept feed when ``keep_feed`` turned it on, else this move's own) and watched
        (``CanfdWatch``: a stop code, or an arm that does not follow, stops the stream); refused when no frame comes
        or a stop code is live; then ``track.run_verified(channel="canfd")``: the lines checked against the
        pass-through's limits (the per-joint gate), the arm at the first line (within one frame's step) or, with
        ``bridge_speed``, brought there by ALIX's explicit MoveJ at that speed, then one ``rm_movej_canfd`` per line on
        absolute 5 ms deadlines; then its end read from the feed's frames (``_stream_end``: the arm within END_TOL_DEG of
        the last line and still, in place of ALIX's polled settle). STOP (``stop_hook``) stops the sending at once (the watch,
        and the thread interrupted); the arm holds the last line it took, and Lite's STOP stops it on its controller.
        The capture is kept in ``records``; the record names the stretch of it this move took (``t_mono``)."""
        from alix_dispatch import track

        rows = [[int(v) for v in r] for r in rows_millideg]
        n = len(rows)
        if n == 0:
            raise UserError(f"{label}: no lines to stream.")
        h = self.handle
        stop = threading.Event()
        watch = _StopWatch(rows, CANFD_PERIOD_S, stop, tap=self._tap.on_frame)
        tid, guard, state = threading.get_ident(), threading.Lock(), {"inside": False}

        def on_stop():
            stop.set()
            with guard:
                if state["inside"]:
                    _interrupt(tid)

        if stop_hook is not None:
            stop_hook(on_stop)
        alog = self._alix_log()

        def log(*parts):
            alog(" ".join(str(p) for p in parts))

        def first_frame() -> bool:
            t_end = time.monotonic() + 1.0
            while watch.n_frames == 0 and time.monotonic() < t_end:
                time.sleep(0.02)
            return watch.n_frames > 0

        t0 = time.monotonic()
        out: dict = {}
        held = False
        with self.link.busy(f"streaming {label}", expect_s=n * CANFD_PERIOD_S + 60):
            kept = self._feed is not None
            rec, cap = self._feed if kept else self._capture(label, records)
            rec.attach_monitor(watch)
            t_mono0 = time.monotonic()
            try:
                if not first_frame() and kept:        # the kept feed went quiet: this move's own, kept from here on
                    rec.attach_monitor(None)
                    self._hw("canfd feed", args=dict(capture=cap.name), ok=False, text="no frame for 1 s: turned on again")
                    try:
                        rec.stop(self.robot)
                    except Exception:                                                   # noqa: BLE001
                        pass
                    self._feed = None
                    rec, cap = self._feed = self._capture(label, records)
                    rec.attach_monitor(watch)
                    first_frame()
                if watch.n_frames == 0:
                    raise UserError(f"No 5 ms frame came from the {self.SIDE} controller to {h.host_ip}:{UDP_PORT[self.side]}, so "
                                    "nothing was streamed (the stream's stop check would be blind).",
                                    "Check that this computer is on the robot's network, then try again.")
                if watch.stop_codes:
                    code = next(iter(watch.stop_codes))
                    raise UserError(f"The {self.SIDE} controller reports {code}, so nothing was streamed.",
                                    "Clear the error (Robot & mode), then run it again.")
                held = self._feed is not None and self._ratio == int(speed) == int(restore_pct)
                try:
                    with guard:
                        if stop.is_set():
                            raise Cancelled()
                        state["inside"] = True
                    out = track.run_verified(
                        self.robot, save_id=0, band_s=track.expected_band_s(n), first_row_millideg=rows[0], points=n,
                        speed=int(speed), restore_pct=int(restore_pct), start_tolerance_deg=2.0,
                        require_at_start=bridge_speed is None, movej_speed=bridge_speed, log=log, settle_after=False,
                        channel="canfd", track_rows_millideg=rows, canfd_watch=watch,
                        canfd={"period_s": CANFD_PERIOD_S, "rate_gate": "per_joint", "follow": True,
                               "trajectory_mode": 0, "radio": 0}, ratio_held=held)
                    if out.get("ok"):                 # its end: the arm at the last line and still, from the feed (26 Oct)
                        cs0 = out.get("canfd") or {}
                        out["settle"] = self._stream_end(rows[-1], cs0.get("t_last_line") or cs0.get("t_end"))
                    if not held:                      # put back to restore_pct (or not known, when that failed)
                        self._ratio = int(restore_pct) if (out.get("restored") or {}).get("global_speed", {}).get("restored") else None
                finally:
                    with guard:
                        state["inside"] = False
                    if stop.is_set():                 # a stop that came as it ended is not raised later
                        _interrupt(tid, None)
            except KeyboardInterrupt:
                out = dict(out, interrupted=True)
            finally:
                if self._feed is not None and self._feed[0] is rec:
                    rec.attach_monitor(self._tap)     # the run's feed stays on for its next move, its frames still read
                else:
                    rec.stop(self.robot)
            t_mono1 = time.monotonic()
        cs = out.get("canfd") or {}
        sent, ok = int(cs.get("lines_sent") or 0), bool(out.get("ok")) and not out.get("restore_failed")
        if not cs and out.get("interrupted"):         # STOP inside run_verified: no result, the stop checks saw the lines
            sent = watch.last_line + 1
        bridged = out.get("gap_deg") if out.get("movej_ret") == 0 else None
        why = out.get("cut_reason") or cs.get("aborted") or ("STOP" if stop.is_set() else None)
        settle = out.get("settle") or {}
        end_words = ("" if not settle else f", end {settle['waited_s'] * 1000:.0f} ms" if settle.get("by") == "feed"
                     else f", at rest by the polled wait after {settle['waited_s']:.2f} s" if settle.get("at_rest")
                     else f", NOT at rest after {settle['waited_s']:.1f} s")
        self._hw("canfd stream", args=dict(label=label, lines=n, period_ms=CANFD_PERIOD_S * 1000, rate_gate="per_joint",
                                           bridge_speed=bridge_speed, capture=cap.name, ratio_held=held,
                                           t_mono=[round(t_mono0, 3), round(t_mono1, 3)], settle=settle or None),
                 ret=out.get("verdict"), ok=ok, elapsed_s=time.monotonic() - t0,
                 text=f"{sent}/{n} lines" + end_words + (f", bridged {bridged:.2f} deg" if bridged else "") + (f"; {why}" if why and not ok else ""))
        if stop.is_set():
            raise Cancelled()
        if not ok:
            raise UserError(f"{label}: the stream did not complete ({why or out.get('verdict') or 'no verdict'}; {sent} of {n} lines sent).",
                            "The arm holds where it stopped: check it, then press STOP or run the task again.")
        return dict(lines=n, lines_sent=sent, seconds=round(float(cs.get("replay_s") or 0.0), 3), bridged_deg=bridged,
                    capture=str(cap), verdict=out.get("verdict"), settle=settle or None)

    def move_to(self, target_millideg: list, speed: int = RESTORE_PCT, restore_pct: int = RESTORE_PCT) -> dict:
        """A MoveJ to a joint pose at ``speed`` % (the speed slider), ALIX's park move (the global speed at
        100 % for it, ``restore_pct`` after), checked against the pose (within 2 degrees)."""
        from alix_dispatch import staging as alix_staging
        from alix_dispatch import track

        from .staging import MOVEJ_SKIP_DEG

        deg = [v / 1000.0 for v in target_millideg]
        with self.link.busy("moving to the start", expect_s=120):
            gap0 = track.start_gap_deg(self.robot, list(target_millideg))
            if gap0 is not None and gap0 <= MOVEJ_SKIP_DEG:      # already there (the owner, 26 Oct): the MoveJ's second is not spent
                self._hw("move to pose", args=dict(deg=[round(d, 2) for d in deg], speed=int(speed), restore_speed=int(restore_pct)),
                         ok=True, text=f"already there: {gap0:.2f} deg away (within {MOVEJ_SKIP_DEG:g}), nothing sent")
                return dict(gap_deg=gap0, skipped=True)
            out = alix_staging.park_arm(self.handle, {}, rest_deg=deg, rest_speed=int(speed), restore_speed=int(restore_pct),
                                        log=self._alix_log())
            self._ratio = int(restore_pct) if out.get("ok") else None     # what park_arm put back
            gap = track.start_gap_deg(self.robot, list(target_millideg))
        ok = bool(out.get("ok")) and gap is not None and gap <= alix_staging.START_TOLERANCE_DEG
        self._hw("move to pose", args=dict(deg=[round(d, 2) for d in deg], speed=int(speed), restore_speed=int(restore_pct)), ok=ok,
                 text=f"gap {gap} deg" + ("" if out.get("ok") else f"; refused: {_words(out.get('refusal'))}"))
        if not ok:
            raise UserError(f"The {self.SIDE} arm did not reach the start (" + (_words(out.get("refusal")) if not out.get("ok")
                            else f"{'?' if gap is None else round(gap, 1)} degrees away") + ").",
                            "Check the arm on its pendant, then try again.")
        return dict(gap_deg=gap)
