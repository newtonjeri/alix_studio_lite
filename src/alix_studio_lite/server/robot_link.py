"""A live link to the real arms, through ALIX's own robot I/O code (``alix_dispatch``).

It connects each controller (``connect.open_arm``) and reads, about five times a second, the
feedback of the three devices ALIX drives on each side (alix_hw: one transport per side):

- the RM75 arm: its seven joints and error codes, and the controller's run mode (SIM or REAL);
- the lift pole: its height (``peripherals.Pole``);
- the Inspire hand: where its six channels actually are (``peripherals.HandModbus.read``, ANGLE_ACT);

and the E-stop input, IO3, which must be in the controller's E-stop mode (5), wired as a
normally-closed button to ground: it reads 0 when pressed.

The link reads; Lite's commands (robot_real.py: staging, drag teaching, upload, runs, moves) use
its connection, and while one runs the link reads only the E-stop input, so two callers never
compete for a controller. STOP (``ArmLink.stop``) is ALIX's own stop of a controller
(``staging.stop_devices``: the arm, the pole, and the hand frozen where it is), then ALIX's check
that the arm is still (``track.stop_program``); STOP BOTH stops the E-stop's way
(``rm_set_arm_stop``: the fastest stop, the trajectory dropped).

``run_lite --robot realman`` uses it with the real controllers; ``--robot emulated`` with ALIX's
controller emulator, which has no IO, so its E-stop reads as clear.
"""
from __future__ import annotations

import threading
import time
from contextlib import contextmanager

from .jobs import UserError

ESTOP_IO = 3                       # the E-stop input on both controllers (decided 13 Oct)
ESTOP_MODE = 5                     # the controller's "input emergency stop" mode
# Every robot is wired as RealMan's circuit-type IO E-stop: a normally-closed button between IO3
# and ground. Released, the circuit is closed and IO3 reads 1; pressed (or a cut wire), it opens
# and reads 0. So 0 is the E-stop (decided 14 Oct, after reading both controllers).
ESTOP_PRESSED_LEVEL = 0
STALE_S = 1.0                      # no fresh reading for this long: the E-stop is UNKNOWN


def estop_state(read, active_level: int = ESTOP_PRESSED_LEVEL) -> tuple[str, dict]:
    """The E-stop chip from one ``rm_get_io_state(3)`` reading: CLEAR, ENGAGED, NOT_CONFIGURED
    (IO3 not in E-stop mode, so it would not stop the arm) or UNKNOWN (unreadable). Fails closed."""
    try:
        ret, st = read()
        mode = int((st.get("io_config") or {}).get("io_mode", -1)) if ret == 0 else None
        level = int(st.get("io_state")) if ret == 0 else None
    except Exception:                                                    # noqa: BLE001
        return "UNKNOWN", {}
    if ret != 0 or level is None:
        return "UNKNOWN", {"ret": ret}
    if mode != ESTOP_MODE:
        return "NOT_CONFIGURED", {"mode": mode, "level": level}
    pressed = level == active_level
    return ("ENGAGED" if pressed else "CLEAR"), {"mode": mode, "level": level, "pressed": pressed}


def error_text(code: int) -> str:
    """A controller error code in words, from ALIX's RealMan tables (alix_core.diagnostics):
    e.g. "4109 (0x100D Robotic arm collision)"."""
    try:
        from alix_core.diagnostics import error_codes

        return error_codes.describe_system_err(int(code))
    except Exception:                                                    # noqa: BLE001
        return f"{code} (0x{int(code):04X})"


def return_text(ret, func: str | None = None) -> str:
    """An SDK call's return code in words (ALIX's tables; a call with its own codes says so)."""
    try:
        from alix_core.diagnostics import error_codes

        text = error_codes.describe_api2_return(int(ret), func)
    except Exception:                                                    # noqa: BLE001
        return f"return code {ret}"
    return text.split("  FIX:")[0].rstrip() + (")" if "  FIX:" in text else "")      # the description only


def error_codes(err) -> list[int]:
    """The arm's error codes, as the runner reads them: decimal text, "0" when all is well."""
    raw = (err.get("err") or []) if isinstance(err, dict) else (err or [])
    out = []
    for c in raw:
        s = str(c).strip()
        try:
            v = int(s, 16) if s.lower().startswith("0x") else int(s)
        except ValueError:
            continue
        if v:
            out.append(v)
    return out


class ArmLink:
    """One controller: connected once, read by one thread, which also owns the SDK handle (it
    closes it when it stops, so nothing uses a deleted handle). ``on_read`` is called after every
    reading, so an E-stop press reaches the robot's jobs without waiting for a page to poll."""

    def __init__(self, side: str, ip: str, port: int = 8080, *, period_s: float = 0.2,
                 estop_active: int = ESTOP_PRESSED_LEVEL,
                 on_read=None):
        self.side, self.ip, self.port = side, ip, int(port)
        self.period_s, self.estop_active = period_s, estop_active
        self.on_read = on_read
        self.handle = None
        self.emulated = False
        self.feedback: dict = {}
        self._hand = None
        self._modbus_set = False
        self._stopping = threading.Event()          # a STOP is being sent: no new reading starts
        self._busy: list = []                       # commands running (robot_real.py): only IO3 is read
        self._read_lock = threading.Lock()          # one reading at a time
        self._light_t = 0.0
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = None

    def open(self) -> "ArmLink":
        from alix_dispatch import connect

        try:                                     # open_arm reports a missing SDK as an unreachable arm
            import Robotic_Arm.rm_robot_interface  # noqa: F401
        except Exception:                                                # noqa: BLE001
            raise UserError("The RealMan SDK does not load on this computer, so Lite cannot connect to the arms.",
                            "The robot link needs Linux (x86-64 or ARM), like the control program.") from None
        h = connect.open_arm(self.side, ip=self.ip, port=self.port)
        if h is None:
            raise UserError(f"Lite cannot reach the {self.side.upper()} arm's controller at {self.ip}:{self.port}.",
                            "Check that the controller is on, the cable, and the address in Robot & mode.")
        self.handle = h
        self.emulated = connect.sdk_provenance(h.robot) == "emulated"
        try:
            self.read_once()
        except Exception:
            connect.close_arm(h)
            self.handle = None
            raise
        self._thread = threading.Thread(target=self._loop, name=f"lite-link-{self.side}", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        """Stop reading. The thread closes the controller connection itself once its last read is
        done, so a slow controller cannot have its handle deleted under a running call."""
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        from alix_dispatch import connect

        try:
            while not self._stop.wait(self.period_s):
                if self._stopping.is_set():
                    continue
                try:
                    if self._busy:
                        if time.time() - self._light_t >= 0.5:
                            self._light_t = time.time()
                            self.read_estop()
                    else:
                        self.read_once()
                except Exception:                                        # noqa: BLE001 - keep reading
                    pass
        finally:
            connect.close_arm(self.handle)
            self.handle = None

    @contextmanager
    def busy(self, what: str):
        """A command runs on this controller (robot_real.py): meanwhile the reading loop only
        watches the E-stop input, twice a second, so two callers never compete for it (ALIX
        measured reply timeouts when they do). The panel shows what is running."""
        with self._lock:
            self._busy.append(what)
        try:
            yield
        finally:
            with self._lock:
                self._busy.remove(what)
                idle = not self._busy
            if idle and self.handle is not None:    # what the command left, read at once (not 0.2 s later)
                try:
                    self.read_once()
                except Exception:                                                # noqa: BLE001
                    pass

    def read_estop(self) -> None:
        """The light reading while a command runs: IO3 only; the rest stays as last read."""
        with self._read_lock:
            self._read_estop()

    def _read_estop(self) -> None:
        robot = self.handle.robot
        with self._lock:
            fb = dict(self.feedback)
            fb["busy"] = self._busy[-1] if self._busy else None
        if hasattr(robot, "rm_get_io_state"):
            fb["estop"], fb["io3"] = estop_state(lambda: robot.rm_get_io_state(ESTOP_IO), self.estop_active)
        elif self.emulated:
            fb["estop"], fb["io3"] = "CLEAR", {"emulated": True}
        else:
            fb["estop"], fb["io3"] = "UNKNOWN", {}
        fb["t"] = time.time()
        with self._lock:
            self.feedback = fb
        if self.on_read is not None:
            self.on_read()

    def stop(self, hard: bool = True) -> dict:
        """STOP this controller as ALIX stops one (``staging.stop_devices``): the arm, the E-stop's
        way when ``hard`` (``rm_set_arm_stop``), else the slow stop on its path (escalating when it
        is refused); the pole (``rm_set_lift_speed(0)``); the hand frozen where it is. Then ALIX's
        check (``track.stop_program``) that the arm is still, a program running on the controller
        included, escalating until it is. ``ok``: the arm and the pole took the stop and the arm
        is proven still. Never raises."""
        from alix_dispatch import peripherals, staging, track

        side, h = self.side.upper(), self.handle
        if h is None:
            return dict(ok=False, text=f"{side}: not connected, so STOP could not be sent")
        quiet = lambda *_: None                                                     # noqa: E731
        self._stopping.set()
        try:
            hand = self._hand if self.feedback.get("hand") else None
            if hand is not None:
                hand.opened = True              # its port is in Modbus mode: the last reading came through it
            st = staging.stop_devices(h, pole=peripherals.Pole(h.robot, log=quiet), hand=hand, log=quiet,
                                      hard=hard).get("stop") or {}
            proof = track.stop_program(h.robot, log=quiet, **({"rungs": ("stop",)} if hard else {}))
        except Exception as exc:                                                    # noqa: BLE001
            return dict(ok=False, text=f"{side}: STOP could not be sent ({exc})")
        finally:
            self._stopping.clear()
        arm, pole, hand_out = st.get("arm") or {}, st.get("pole") or {}, st.get("hand") or {}
        ok = bool(arm.get("ok") and pole.get("ok") and proof.get("stopped"))
        if ok:
            text = (f"{side}: stopped on its controller ({arm.get('escalated') or arm.get('verb')}), the pole "
                    f"stopped, the hand {'frozen' if hand_out.get('ok') else 'not frozen (not readable)'}; the arm is still")
        else:
            bad = ([f"the arm's stop returned {arm.get('escalated_ret', arm.get('ret'))}"] if not arm.get("ok") else []) \
                + ([f"the pole: {pole.get('error') or pole.get('ret')}"] if not pole.get("ok") else []) \
                + ([f"the arm is not proven still ({proof.get('refusal') or 'no proof'})"] if not proof.get("stopped") else [])
            text = f"{side}: STOP not confirmed: " + "; ".join(bad)
        return dict(ok=ok, text=text, arm=arm, pole=pole, hand=hand_out, still=proof.get("stopped"))

    def _read_hand(self, robot):
        from alix_dispatch import peripherals

        if self._hand is None:
            self._hand = peripherals.HandModbus(robot, self.side, log=lambda *_: None)
        counts = self._hand.read()
        if counts is None and not self._modbus_set:                  # the end port is not in Modbus mode yet
            self._modbus_set = True
            try:
                robot.rm_set_modbus_mode(peripherals.HAND_MODBUS_PORT, peripherals.HAND_MODBUS_BAUD, 2)
            except Exception:                                            # noqa: BLE001
                return None
            counts = self._hand.read()
        return counts

    def read_once(self) -> dict:
        with self._read_lock:
            return self._read_all()

    def _read_all(self) -> dict:
        from alix_dispatch import peripherals, runmode

        robot = self.handle.robot
        fb: dict = {"t": time.time()}
        try:
            ret, st = robot.rm_get_current_arm_state()
        except Exception:                                                # noqa: BLE001
            ret, st = -1, None
        ok = ret == 0 and bool(st)
        fb["joints_deg"] = [round(float(v), 3) for v in (st.get("joint") or [])] if ok else None
        fb["errors"] = error_codes(st.get("err")) if ok else None
        r, mode = runmode.read_run_mode(robot)
        fb["run_mode"] = {0: "SIM", 1: "REAL"}.get(mode) if r == 0 else None
        height = peripherals.Pole(robot, log=lambda *_: None).height_m()
        fb["pole_mm"] = None if height is None else round(height * 1000.0, 1)
        fb["hand"] = self._read_hand(robot)
        if hasattr(robot, "rm_get_io_state"):
            fb["estop"], fb["io3"] = estop_state(lambda: robot.rm_get_io_state(ESTOP_IO), self.estop_active)
        elif self.emulated:                                              # ALIX's emulator models no IO
            fb["estop"], fb["io3"] = "CLEAR", {"emulated": True}
        else:
            fb["estop"], fb["io3"] = "UNKNOWN", {}
        fb["busy"] = None
        with self._lock:
            self.feedback = fb
        if self.on_read is not None:
            self.on_read()
        return fb

    def view(self) -> dict:
        with self._lock:
            fb = dict(self.feedback)
        age = time.time() - fb["t"] if fb.get("t") else None
        if age is None or age > STALE_S:
            fb["estop"] = "UNKNOWN"                                      # no fresh reading: fail closed
        fb["age_s"] = None if age is None else round(age, 2)
        fb.pop("t", None)
        return dict(ip=self.ip, port=self.port, emulated=self.emulated, feedback=fb)
