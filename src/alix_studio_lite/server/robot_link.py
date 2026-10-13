"""A read-only link to the real arms, through ALIX's own robot I/O code (``alix_dispatch``).

It connects each controller (``connect.open_arm``) and reads, about five times a second, the
feedback of the three devices ALIX drives on each side (alix_hw: one transport per side):

- the RM75 arm: its seven joints and error codes, and the controller's run mode (SIM or REAL);
- the lift pole: its height (``peripherals.Pole``);
- the Inspire hand: where its six channels actually are (``peripherals.HandModbus.read``, ANGLE_ACT);

and the E-stop input, IO3, which must be in the controller's E-stop mode (5).

READ ONLY. Lite commands nothing through this link: robot actions stay with the stand-in until
Lite's robot commands exist, and the screens say so. The one setting it may write is the end
port's Modbus mode, which reading the hand needs (the control program and ALIX set it the same
way); it never writes the hand, the pole or the arm.

``run_lite --robot realman`` uses it with the real controllers; ``--robot emulated`` with ALIX's
controller emulator, which has no IO, so its E-stop reads as clear.
"""
from __future__ import annotations

import threading
import time

from .jobs import UserError

ESTOP_IO = 3                       # the E-stop input on both controllers (decided 13 Oct)
ESTOP_MODE = 5                     # the controller's "input emergency stop" mode
STALE_S = 1.0                      # no fresh reading for this long: the E-stop is UNKNOWN


def estop_state(read, active_level: int = 1) -> tuple[str, dict]:
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
    return ("ENGAGED" if level == active_level else "CLEAR"), {"mode": mode, "level": level}


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

    def __init__(self, side: str, ip: str, port: int = 8080, *, period_s: float = 0.2, estop_active: int = 1,
                 on_read=None):
        self.side, self.ip, self.port = side, ip, int(port)
        self.period_s, self.estop_active = period_s, estop_active
        self.on_read = on_read
        self.handle = None
        self.emulated = False
        self.feedback: dict = {}
        self._hand = None
        self._modbus_set = False
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
                try:
                    self.read_once()
                except Exception:                                        # noqa: BLE001 - keep reading
                    pass
        finally:
            connect.close_arm(self.handle)
            self.handle = None

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
