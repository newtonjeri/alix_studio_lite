"""A live link to the real arms, through ALIX's own robot I/O code (``alix_dispatch``).

It connects each controller (``connect.open_arm``, twice: the commands' connection and the readings' own) and reads, about five times a second, the
feedback of the three devices ALIX drives on each side (alix_hw: one transport per side):

- the RM75 arm: its seven joints and error codes, and the controller's run mode (SIM or REAL);
- the lift pole: its height (``peripherals.Pole``);
- the Inspire hand: where its six channels actually are (``peripherals.HandModbus.read``, ANGLE_ACT);

and the E-stop input, IO3, which must be in the controller's E-stop mode (5), wired as a
normally-closed button to ground: it reads 0 when pressed.

The link reads; Lite's commands (robot_real.py: staging, drag teaching, upload, runs, moves) use
its connection, and while one runs the link does not read at all: a second caller on one
controller makes its replies time out (ALIX measured it; the hardware showed it on 14 Oct, a
program list after an upload answering -2). The controller acts on its own E-stop meanwhile; Lite
reads everything again the moment the command ends. STOP (``ArmLink.stop``) is ALIX's own stop of a controller
(``staging.stop_devices``: the arm, the pole, and the hand frozen where it is), then ALIX's check
that the arm is still (``track.stop_program``); STOP BOTH stops the E-stop's way
(``rm_set_arm_stop``: the fastest stop, the trajectory dropped).

``run_lite --robot realman`` uses it with the real controllers; ``--robot emulated`` with ALIX's
controller emulator, which has no IO, so its E-stop reads as clear.
"""
from __future__ import annotations

import collections
import functools
import os
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
CMD_WAIT_S = 5.0                   # a command waits this long for the one before it on its controller, then is refused
CLOSE_WAIT_S = 30.0                # a closed link's handle is deleted once the calls still running on it have ended
READ_WAIT_S = 0.3                  # a STOP waits this long at most for a reading under way (their replies never meet)
# The arm indicators (the owner, 17 Oct: green a good connection, yellow a weak one, red none): a reading
# normally lands every 0.2-0.45 s and takes about 0.1 s (the lab's logs, 16-17 Oct)
LATE_S = 0.8                       # no reading for this long: weak
SLOW_S = 0.5                       # a reading that took this long, among the last ten: weak
LOST_S = 3.0                       # no reading answered for this long: none
HELD_S = 60.0                      # a command that has not ended in this long (unless it says it takes longer): weak
HAND_LULL_S = 0.5                  # the hand (Modbus on the shared end port) is read on the readings' connection only this long after
                                   # the last command ended: a run's commands come closer than that, and none waits for a hand read
HAND_REOPEN_S = 2.0                # a hand read that fails has the end port put back in Modbus mode, at most this often
# A call into the SDK that does not come back (the lab, 19 Oct: three times a call made right after "run mode REAL"
# never returned; nothing on that controller answered, STOP included, until Lite was killed). The SDK answers a
# request within its reply timeout (1.5 s, connect.ACK_TIMEOUT_MS), so a call that has not returned in CALL_STUCK_S
# is stuck inside the library, and Python cannot interrupt it: the connection is given up and made again
# (robot_standin). The calls that last as long as their work (LONG_CALLS: the arm moving, a file sent, a recording
# saved) are judged by the command's own expectation (busy(expect_s)), never by CALL_STUCK_S.
CALL_SLOW_S = float(os.environ.get("LITE_CALL_SLOW_S", 5.0))      # a call not answered in this long: weak, and named
CALL_STUCK_S = float(os.environ.get("LITE_CALL_STUCK_S", 20.0))   # ... in this long: stuck (the env: the tests' arm workers)
LONG_CALLS = frozenset({"rm_movej", "rm_movel", "rm_movec", "rm_movej_p", "rm_moves", "rm_save_trajectory",
                        "rm_send_project", "rm_set_program_id_run", "rm_run_drag_trajectory"})


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


def _noted(fn, name: str, link: "ArmLink"):
    @functools.wraps(fn)
    def call(self, *a, **kw):
        link._enter(name)
        try:
            return fn(self, *a, **kw)
        finally:
            link._leave()
    return call


def _swap_class(obj, link: "ArmLink") -> bool:
    cls = type(obj)
    ns = {}
    for name in dir(cls):
        if name.startswith("rm_"):
            fn = getattr(cls, name, None)
            if callable(fn):
                ns[name] = _noted(fn, name, link)
    if not ns:
        return False
    try:
        sub = type(cls.__name__, (cls,), ns)
        sub.__module__, sub.__qualname__ = cls.__module__, cls.__qualname__
        object.__setattr__(obj, "__class__", sub)      # not through a __setattr__ that forwards to a wrapped object
    except TypeError:
        return False
    return True


def watch_calls(robot, link: "ArmLink") -> bool:
    """Every ``rm_*`` method of the SDK object noted on its link as it is called (what, since when), through a
    subclass of the object's own class put in its place (``__class__``): the SDK is untouched, the object is the
    same one (its callbacks stay registered), and ALIX's checks of its type and module still hold
    (``type(robot).__module__`` names the emulator or the SDK). ALIX's emulated handle wraps the emulator and
    forwards to it (``_emulated_inner``): both are watched. False when nothing could be swapped: then nothing is
    noted and a stuck call is only ever seen as a command that does not end."""
    done = _swap_class(robot, link)
    inner = getattr(robot, "_emulated_inner", None)
    if inner is not None and inner is not robot:
        done = _swap_class(inner, link) or done
    return done


JOINT_WARN_C = 60.0       #: a joint at or above this is said (an alert) in the status snapshots (app.py)


def _joint_floats(robot, name: str, scale: float, digits: int) -> list | None:
    """One of the SDK's per-joint reads (rm_get_current_joint_temperature, _current), scaled and rounded, or None when
    the controller does not answer it (ALIX's emulator has no such call)."""
    fn = getattr(robot, name, None)
    if fn is None:
        return None
    try:
        ret, values = fn()
    except Exception:                                                    # noqa: BLE001
        return None
    if ret != 0 or not values:
        return None
    return [round(float(v) * scale, digits) for v in values]


def _controller_state(robot) -> dict | None:
    """The controller's own voltage (V), current (A) and temperature (C), as the vendor's GUI shows them under Controller
    Info (rm_get_controller_state), or None when the controller does not answer it (ALIX's emulator)."""
    fn = getattr(robot, "rm_get_controller_state", None)
    if fn is None:
        return None
    try:
        st = fn()
    except Exception:                                                    # noqa: BLE001
        return None
    if not isinstance(st, dict) or st.get("return_code") != 0:
        return None
    return dict(voltage_v=round(float(st.get("voltage") or 0.0), 1), current_a=round(float(st.get("current") or 0.0), 2),
                temp_c=round(float(st.get("temperature") or 0.0), 1), sys_err=int(st.get("system_error") or 0))


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
        self.reader = None                          # a second connection to the same controller, for the readings only (26 Oct)
        self.emulated = False
        self.feedback: dict = {}
        self._hand = None
        self._modbus_reopened = 0.0                 # when the end port was last put back in Modbus mode for a reading
        self._stopping = threading.Event()          # a STOP is being sent: no new reading starts
        self._busy: list = []                       # commands running (robot_real.py): no reading meanwhile
        self._busy_since, self._busy_expect = 0.0, HELD_S   # when the command took the link, and how long it may take
        self._recent: collections.deque = collections.deque(maxlen=10)   # (when, answered, seconds) of the last readings
        self._freed = 0.0                           # when the last command ended (its reading follows at once)
        self._ended = 0.0                           # ... on the monotonic clock (the hand's lull, HAND_LULL_S)
        self._fresh_before = False                  # the reading was fresh as the commands began (their grace is due)
        self._read_lock = threading.Lock()          # one reading at a time
        self._hand_lock = threading.Lock()          # the hand's Modbus read under way (the end port is shared with the commands)
        self._wake = threading.Event()              # a reading asked for now (what a command left, on the readings' connection)
        self._cmd = threading.RLock()               # one command at a time (busy)
        self._stop_lock = threading.Lock()          # one STOP at a time
        self._stop_hard = False                     # the STOP being sent is the E-stop's way
        self._stop_last: dict | None = None         # its answer, for a press that came while it was sent
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._calls: dict = {}                      # thread id -> (SDK call, since): what is in flight on the handle
        self.abandoned = False                      # given up: a call on its handle never came back (robot_standin)

    def open(self) -> "ArmLink":
        from alix_dispatch import connect

        try:                                     # open_arm reports a missing SDK as an unreachable arm
            import Robotic_Arm.rm_robot_interface  # noqa: F401
        except Exception:                                                # noqa: BLE001
            raise UserError("The RealMan SDK does not load on this computer, so Lite cannot connect to the arms.",
                            "The robot link needs Linux (x86-64 or ARM), like the control program.") from None
        # The readings' own connection, opened first (ALIX's emulator stamps its arrival events with the handle made last,
        # which must be the commands'). The vendor's GUI reads the same way, on a connection of its own, while a program
        # plays (the owner, 26 Oct: "Lite is our company's version of the vendor's GUI"); one that cannot be opened leaves
        # the readings on the commands' connection, between commands, as before. LITE_LINK_READER=0 keeps it that way.
        reader = None
        if os.environ.get("LITE_LINK_READER", "1") != "0":
            try:
                reader = connect.open_arm(self.side, ip=self.ip, port=self.port)
            except Exception:                                            # noqa: BLE001
                reader = None
        h = connect.open_arm(self.side, ip=self.ip, port=self.port)
        if h is None:
            connect.close_arm(reader)
            raise UserError(f"Lite cannot reach the {self.side.upper()} arm's controller at {self.ip}:{self.port}.",
                            "Check that the controller is on, the cable, and the address in Robot & mode.")
        self.handle, self.reader = h, reader
        self.emulated = connect.sdk_provenance(h.robot) == "emulated"
        watch_calls(h.robot, self)
        if reader is not None:
            watch_calls(reader.robot, self)
        try:
            self.read_once()
        except Exception:
            connect.close_arm(h)
            connect.close_arm(reader)
            self.handle = self.reader = None
            raise
        self._thread = threading.Thread(target=self._loop, name=f"lite-link-{self.side}", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        """Stop reading. The thread closes the controller connection itself once its last read is
        done, so a slow controller cannot have its handle deleted under a running call."""
        self._stop.set()
        self._wake.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    # ── the calls into the SDK, as they are made (watch_calls)
    def _enter(self, name: str) -> None:
        with self._lock:
            self._calls[threading.get_ident()] = (name, time.monotonic())

    def _leave(self) -> None:
        with self._lock:
            self._calls.pop(threading.get_ident(), None)

    def in_flight(self) -> tuple[str, float] | None:
        """The SDK call that has waited longest for its answer (name, seconds), or None."""
        now = time.monotonic()
        with self._lock:
            calls = list(self._calls.values())
        if not calls:
            return None
        name, t0 = min(calls, key=lambda c: c[1])
        return name, now - t0

    def stuck(self) -> tuple[str, float] | None:
        """A call that has not come back within its bound (name, seconds): CALL_STUCK_S, or for the calls that last
        as long as their work (LONG_CALLS) the command's own expectation."""
        now = time.monotonic()
        with self._lock:
            calls = list(self._calls.values())
            expect = self._busy_expect if self._busy else HELD_S
        worst = None
        for name, t0 in calls:
            age = now - t0
            bound = max(CALL_STUCK_S, expect) if name in LONG_CALLS else CALL_STUCK_S
            if age > bound and (worst is None or age > worst[1]):
                worst = (name, age)
        return worst

    def abandon(self) -> None:
        """Given up: a call on its handle never came back. The thread stops reading; the handle is never deleted
        (that call still runs on it: the lab, 17 Oct) and nothing waits for the thread. The arm is connected again
        on a new link (robot_standin)."""
        self.abandoned = True
        self._stop.set()
        self._wake.set()

    def _loop(self) -> None:
        from alix_dispatch import connect

        try:
            while not self._stop.is_set():
                self._wake.wait(self.period_s)       # every period, or at once when a command has just ended (busy)
                self._wake.clear()
                if self._stop.is_set():
                    break
                if self._stopping.is_set():
                    continue
                try:
                    self._read_if_idle()
                except Exception:                                        # noqa: BLE001 - keep reading
                    pass
        finally:
            # The handle is deleted once no call runs on it, a command's or a STOP's on another thread; one
            # still running after CLOSE_WAIT_S keeps it (never deleted under a call: the lab, 17 Oct, closed
            # both links while their STOPs went unanswered, and no Connect answered after)
            cmd = self._cmd.acquire(timeout=CLOSE_WAIT_S)
            stop = self._stop_lock.acquire(timeout=CLOSE_WAIT_S if cmd else 0)
            try:
                h, self.handle = self.handle, None      # no caller takes it from here on
                r, self.reader = self.reader, None      # the readings' connection: this thread's alone
                if cmd and stop:
                    connect.close_arm(h)
                if r is not None:
                    connect.close_arm(r)
            finally:
                if stop:
                    self._stop_lock.release()
                if cmd:
                    self._cmd.release()

    def _take(self, wait_s: float | None = None) -> None:
        """This controller for one command: a STOP being sent ends first, then the command before it
        (together at most ``wait_s``, CMD_WAIT_S by default), else it is refused, saying what holds it."""
        t_end = time.monotonic() + (CMD_WAIT_S if wait_s is None else wait_s)
        while self._stopping.is_set() and time.monotonic() < t_end:     # never alongside a STOP's own calls
            time.sleep(0.05)
        if self._stopping.is_set() or not self._cmd.acquire(timeout=max(0.0, t_end - time.monotonic())):
            with self._lock:
                now = "sending STOP" if self._stopping.is_set() else self._busy[-1] if self._busy else "another command"
            raise UserError(f"The {self.side.upper()} controller is busy: {now}.", "Try again once it has finished.",
                            code="busy")

    @contextmanager
    def busy(self, what: str, wait_s: float | None = None, expect_s: float | None = None):
        """A command runs on this controller (robot_real.py): no other command starts meanwhile: it waits for this one
        (at most CMD_WAIT_S, or ``wait_s``), then is refused, saying what holds the controller (the lab, 17 Oct: three
        MoveJ's switched the RIGHT controller's run mode while it saved a 24 s recording, each read back "not taken",
        and a global point list read in those seconds answered -2). A STOP being sent is waited for the same way; STOP
        never waits (stop()). The panel shows what is running, with the last reading.

        With the readings' own connection (``reader``, 26 Oct) the loop reads through the command; the command waits
        only for a hand read under way (the end port's Modbus is shared: the lab's first run, a reading overlapping the
        hand's staging), and as it ends the loop is woken to read what it left, on that connection, while the caller
        goes on (27 Oct: the reading made here on the command's connection, 0.1 to 0.35 s with the hand, held every
        command's caller for it, some 60 times a run of S2; and a command arriving during a reading waited for it to
        end, up to 0.2 s more). Without it, as before: nothing is read while a command runs, a reading under way ends
        first, and everything is read again here as the command ends."""
        self._take(wait_s)
        try:
            with self._lock:
                if not self._busy:                  # how fresh the reading is as a command takes the connection
                    t, now = self.feedback.get("t"), time.time()
                    self._fresh_before = bool(t) and min(now - t, now - self._freed) <= STALE_S
                    self._busy_since, self._busy_expect = now, max(HELD_S, float(expect_s or 0))
                self._busy.append(what)
                self.feedback = dict(self.feedback, busy=what)
            with (self._hand_lock if self.reader is not None else self._read_lock):   # ... under way ends before the command starts
                pass
            try:
                yield
            finally:
                with self._lock:
                    self._busy.remove(what)
                    idle = not self._busy
                    if idle:                        # the grace for the reading that follows: only for a link that was fresh
                        self._freed = time.time() if self._fresh_before else 0.0
                        self._ended = time.monotonic()
                if idle and self.handle is not None and not self._stopping.is_set():
                    if self.reader is not None:     # what the command left, read at once (not 0.2 s later) by the loop, on
                        self._wake.set()            # the readings' connection: the caller does not wait for it
                    else:
                        try:
                            self.read_once()
                        except Exception:                                            # noqa: BLE001
                            pass
        finally:
            self._cmd.release()

    def read_fresh(self) -> dict:
        """One reading now, as a command takes the controller: never alongside one (Capture)."""
        self._take()
        try:
            return self.read_once()
        finally:
            self._cmd.release()

    @contextmanager
    def reserved(self, wait_s: float | None = None):
        """This controller held for the caller's next commands (busy() then takes it at once): S2's TOGETHER
        stage takes both arms' before either starts, so neither waits for its controller after the other
        has started."""
        self._take(wait_s)
        try:
            yield
        finally:
            self._cmd.release()

    def stop_in_progress(self) -> bool:
        return self._stop_lock.locked()

    def _quiet(self) -> None:
        """No reading starts (``_stopping``, cleared by the caller), and one under way ends first (at most
        READ_WAIT_S), so a STOP's replies never meet a reading's."""
        self._stopping.set()
        if self._read_lock.acquire(timeout=READ_WAIT_S):
            self._read_lock.release()

    def leave_drag(self) -> int:
        """``rm_stop_drag_teach`` sent as STOP sends its calls (an E-stop; a command still holding the
        controller): never waits for a command, never alongside a reading or a STOP. Its return code
        (-1: not connected, or it raised)."""
        with self._stop_lock:
            h = self.handle
            if h is None:
                return -1
            self._quiet()
            try:
                return int(h.robot.rm_stop_drag_teach())
            except Exception:                                                    # noqa: BLE001
                return -1
            finally:
                self._stopping.clear()

    def stop(self, hard: bool = True, drag: bool = False) -> dict:
        """STOP this controller as ALIX stops one (``staging.stop_devices``): the arm, the E-stop's
        way when ``hard`` (``rm_set_arm_stop``), else the slow stop on its path (escalating when it
        is refused); the pole (``rm_set_lift_speed(0)``); the hand frozen where it is. Then ALIX's
        check (``track.stop_program``) that the arm is still, a program running on the controller
        included, escalating until it is. ``ok``: the arm and the pole took the stop and the arm
        is proven still. Never raises; never waits for a command.

        One STOP at a time: a press while one is being sent waits for its answer instead of sending
        a second alongside it (the lab, 17 Oct: three STOP BOTH presses in 40 s, two controllers each
        sent stop after stop at once, none confirmed, and the links did not recover); a STOP BOTH that
        comes during one arm's STOP still stops that arm the E-stop's way at once (one call). ``drag``: the
        arm is in drag teaching, which it leaves first (``rm_stop_drag_teach``, its return code in ``drag``).
        No reading starts meanwhile, and one under way ends first (at most READ_WAIT_S)."""
        stuck = self.stuck()
        if stuck:                                   # nothing passes through this handle: STOP goes through a new one (robot_standin)
            return dict(ok=False, stuck=True, text=f"{self.side.upper()}: its connection is stuck ({stuck[0]} not answered "
                        f"for {stuck[1]:.0f} s), so STOP cannot pass through it")
        if not self._stop_lock.acquire(blocking=False):
            h = self.handle
            if hard and not self._stop_hard and h is not None:
                try:
                    h.robot.rm_set_arm_stop()
                except Exception:                                                # noqa: BLE001
                    pass
            with self._stop_lock:
                return dict(self._stop_last or dict(ok=False, text=f"{self.side.upper()}: STOP not confirmed"), joined=True)
        try:
            self._stop_hard = hard
            self._stop_last = self._stop_now(hard, drag)
            return dict(self._stop_last)
        finally:
            self._stop_hard = False
            self._stop_lock.release()

    def _stop_now(self, hard: bool, drag: bool = False) -> dict:
        from alix_dispatch import peripherals, staging, track

        side, h = self.side.upper(), self.handle
        if h is None:
            return dict(ok=False, text=f"{side}: not connected, so STOP could not be sent")
        quiet = lambda *_: None                                                     # noqa: E731
        self._quiet()
        dragged = None
        try:
            if drag:                                # drag teaching ends first, then the stop
                try:
                    dragged = int(h.robot.rm_stop_drag_teach())
                except Exception:                                                # noqa: BLE001
                    dragged = -1
            hand = self._hand if self.feedback.get("hand") else None
            if hand is not None:
                hand.opened = True              # its port is in Modbus mode: the last reading came through it
            st = staging.stop_devices(h, pole=peripherals.Pole(h.robot, log=quiet), hand=hand, log=quiet,
                                      hard=hard).get("stop") or {}
            proof = track.stop_program(h.robot, log=quiet, **({"rungs": ("stop",)} if hard else {}))
        except Exception as exc:                                                    # noqa: BLE001
            return dict(ok=False, text=f"{side}: STOP could not be sent ({exc})", drag=dragged)
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
        return dict(ok=ok, text=text, arm=arm, pole=pole, hand=hand_out, still=proof.get("stopped"), drag=dragged)

    def _read_hand(self, robot):
        """The hand's six counts over the end port's Modbus, None when the read fails. The end port is in Modbus mode only
        while someone has set it (rm_set_modbus_mode) and no one has closed it: runner v14's Hand steps and, until 27 Oct,
        Lite's own pole and hand step closed it after each use (rm_close_modbus_mode), and every reading after that
        failed until Lite's next hand command set it again (27 Oct lab: the panel's hand empty for 21 to 56 % of the
        readings, for minutes at a time). A failed read now puts the mode back, at most every HAND_REOPEN_S, and reads
        again. The vendor's web GUI, running on the controller, never closes the mode, which is why it never loses the hand."""
        from alix_dispatch import peripherals

        if self._hand is None or self._hand.robot is not robot:          # the hand as read on this connection
            self._hand = peripherals.HandModbus(robot, self.side, log=lambda *_: None)
        counts = self._hand.read()
        now = time.monotonic()
        if counts is None and now - self._modbus_reopened >= HAND_REOPEN_S:   # the end port not in Modbus mode (closed since)
            self._modbus_reopened = now
            try:
                robot.rm_set_modbus_mode(peripherals.HAND_MODBUS_PORT, peripherals.HAND_MODBUS_BAUD, 2)
            except Exception:                                            # noqa: BLE001
                return None
            counts = self._hand.read()
        return counts

    def estop_now(self) -> str:
        """IO3 read once, by the caller that holds the connection (a command's own thread, between two of
        its steps: never alongside it): ENGAGED, CLEAR, NOT_CONFIGURED or UNKNOWN, failing closed."""
        robot = self.handle.robot if self.handle is not None else None
        if robot is None:
            return "UNKNOWN"
        if hasattr(robot, "rm_get_io_state"):
            return estop_state(lambda: robot.rm_get_io_state(ESTOP_IO), self.estop_active)[0]
        return "CLEAR" if self.emulated else "UNKNOWN"          # ALIX's emulator models no IO

    def read_once(self) -> dict:
        with self._read_lock:
            return self._read_all()

    def hand_seen(self, counts) -> None:
        """The hand as a command of Lite's read it back (robot_real: a staging, a hand move, the pole and hand step): the
        panel shows it at once, as the loop reads the hand only in a lull between commands (HAND_LULL_S)."""
        if counts is None:
            return
        with self._lock:
            self.feedback = dict(self.feedback, hand=[int(v) for v in counts])

    def _read_if_idle(self) -> None:
        """The loop's reading. With the readings' own connection (``reader``) the arm, the pole, the controller and IO3
        are read whether a command runs or not (a reading on a connection of its own costs the commands nothing and
        the controller's clock nothing: the owner, 26 Oct); the hand, Modbus on the end port, only between commands,
        in a lull (``_read_all``: HAND_LULL_S after the last; busy() waits for a hand read under way, so the two never
        overlap: on the lab's first run a reading overlapping the hand's staging made the hand's command time out).
        Without it, nothing is read while a command runs."""
        with self._read_lock:
            if self.reader is not None:
                self._read_all(self.reader.robot, hand=not self._busy)
            elif not self._busy:                    # a command owns the connection: it alone talks
                self._read_all()

    def _read_all(self, robot=None, hand: bool = True) -> dict:
        from alix_dispatch import peripherals, runmode

        if robot is None:
            robot = (self.reader or self.handle).robot
        fb: dict = {"t": time.time()}
        t0 = time.monotonic()
        try:
            ret, st = robot.rm_get_current_arm_state()
        except Exception:                                                # noqa: BLE001
            ret, st = -1, None
        ok = ret == 0 and bool(st)
        fb["joints_deg"] = [round(float(v), 3) for v in (st.get("joint") or [])] if ok else None
        fb["pose"] = [round(float(v), 6) for v in (st.get("pose") or [])[:6]] if ok and st.get("pose") else None   # m, rad
        fb["errors"] = error_codes(st.get("err")) if ok else None
        # the joints' temperatures (C), currents (A) and voltages (V), and the controller's own, with every reading, asked
        # over the arm's connection as the vendor's GUI asks them (no realtime push; the owner, 26 Oct: the right arm's joint
        # 3 ran to 80 C at a high current and stopped following; only the 5 ms captures had shown temperatures, and only
        # during streams; "Lite is our company's version of the vendor's GUI")
        fb["temps_c"] = _joint_floats(robot, "rm_get_current_joint_temperature", 1.0, 1) if ok else None
        fb["currents_a"] = _joint_floats(robot, "rm_get_current_joint_current", 0.001, 2) if ok else None
        fb["voltages_v"] = _joint_floats(robot, "rm_get_current_joint_voltage", 1.0, 1) if ok else None
        fb["controller"] = _controller_state(robot) if ok else None
        r, mode = runmode.read_run_mode(robot)
        fb["run_mode"] = {0: "SIM", 1: "REAL"}.get(mode) if r == 0 else None
        height = peripherals.Pole(robot, log=lambda *_: None).height_m()
        fb["pole_mm"] = None if height is None else round(height * 1000.0, 1)
        if hand and self.reader is not None:       # the end port is the commands' too: not under one, nor between two that
            with self._hand_lock:                   # come close (HAND_LULL_S); busy() waits for a read under way here
                hand = not self._busy and time.monotonic() - self._ended >= HAND_LULL_S
                fb["hand"] = self._read_hand(robot) if hand else self.feedback.get("hand")
        else:
            fb["hand"] = self._read_hand(robot) if hand else self.feedback.get("hand")   # a command runs: the last read
        if hasattr(robot, "rm_get_io_state"):
            fb["estop"], fb["io3"] = estop_state(lambda: robot.rm_get_io_state(ESTOP_IO), self.estop_active)
        elif self.emulated:                                              # ALIX's emulator models no IO
            fb["estop"], fb["io3"] = "CLEAR", {"emulated": True}
        else:
            fb["estop"], fb["io3"] = "UNKNOWN", {}
        fb["busy"], fb["ok"] = None, ok
        with self._lock:
            self.feedback = fb
            self._recent.append((fb["t"], ok, time.monotonic() - t0))
        if self.on_read is not None:
            self.on_read()
        return fb

    def view(self) -> dict:
        with self._lock:
            fb = dict(self.feedback)
            fb["busy"] = self._busy[-1] if self._busy else None
            freed = self._freed
        now = time.time()
        age = now - fb["t"] if fb.get("t") else None
        # Stale: no reading for STALE_S, counted from the end of the last command too: while a command owns
        # the connection nothing is read, and the reading it is followed by takes a moment to land (on the lab's
        # arms, 16 Oct, every long run, upload or saved take raised "No fresh reading" in that moment).
        since = None if age is None else min(age, now - freed)
        fb["stale"] = (since is None or since > STALE_S) and not fb["busy"]
        if fb["stale"]:
            fb["estop"] = "UNKNOWN"                                      # no fresh reading: fail closed
        fb["age_s"] = None if age is None else round(age, 2)
        fb.pop("t", None)
        call = self.in_flight()
        fb["call"] = None if call is None or call[1] < CALL_SLOW_S else dict(name=call[0], age_s=round(call[1], 1))
        return dict(ip=self.ip, port=self.port, emulated=self.emulated, abandoned=self.abandoned, reader=self.reader is not None,
                    feedback=fb, quality=self.quality())

    def quality(self) -> dict:
        """The connection as the arm's indicator shows it (the owner, 17 Oct: "green a good connection,
        yellow a weak connection, red no connection"): ``good``, readings answered and fresh; ``weak``,
        readings late, slow or failing now and then, or a command that has held the controller longer than
        it takes; ``none``, no reading answered for LOST_S (a cable out, the controller off or not
        answering). ``text`` says why."""
        now = time.time()
        with self._lock:
            busy, since, expect = list(self._busy), self._busy_since, self._busy_expect
            recent, t = list(self._recent), self.feedback.get("t")
        doing = f" ({busy[-1]})" if busy else ""
        stuck = self.stuck()
        if stuck:                                   # a call into the SDK that does not come back
            return dict(level="none", text=f"stuck: {stuck[0]} has not answered for {stuck[1]:.0f} s{doing}")
        call = self.in_flight()
        if call is not None and call[1] > CALL_SLOW_S:
            return dict(level="weak", text=f"{call[0]} has not answered for {call[1]:.0f} s{doing}")
        if busy:                                    # a command holds it: nothing is read meanwhile, by design
            held = now - since
            if held > expect:
                return dict(level="weak", text=f"no answer for {held:.0f} s from the controller ({busy[-1]})")
            return dict(level="good", text=f"connected; busy: {busy[-1]}")
        answered = [w for w, ok, _ in recent if ok]
        if not answered or now - answered[-1] > LOST_S:
            gone = now - answered[-1] if answered else None
            return dict(level="none", text="no answer from the controller" + (f" for {gone:.0f} s" if gone else ""))
        failed = sum(1 for _, ok, _ in recent if not ok)
        slow = max(dt for _, _, dt in recent)
        late = now - (t or 0)
        if failed or slow > SLOW_S or late > LATE_S:
            why = ([f"{failed} of the last {len(recent)} readings unanswered"] if failed else []) \
                + ([f"a reading took {slow:.1f} s"] if slow > SLOW_S else []) + ([f"no reading for {late:.1f} s"] if late > LATE_S else [])
            return dict(level="weak", text=", ".join(why))
        return dict(level="good", text=f"readings every {self.period_s:g} s, answered in {slow * 1000:.0f} ms at most")
