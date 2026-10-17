"""The two arms behind Lite's API. Without hardware it is a stand-in, so every robot screen works
(``run_lite``'s default). With ``--robot realman`` (or ``emulated``) each connected arm has a live
link (robot_link.py: its feedback, its E-stop input driving the interlock, STOP) and its commands
go to its controller through ALIX's own code (robot_real.py): in Real the pole and the hand are
staged at 50 % and read back, drag teaching records on the controller, programs are uploaded and
run; in Hardware sim the controller runs them in its SIM mode and the pole and the hand, which it
cannot simulate, are taken as arrived.

It follows the real rules the backend must keep:

- connect reads the E-stop input (IO3, mode 5) on each arm;
- while either arm's E-stop is not CLEAR, motion is refused on BOTH arms; uploads are allowed;
- STOP is never refused and never queued. It acts like the E-stop: it reaches a linked arm's
  controller (ALIX's stop: STOP BOTH the E-stop's way, one arm's STOP on its path), ends every
  job and drag teaching, and holds each connected arm it stopped until Recover;
- an E-stop press is LATCHED: released, the arm reads RECOVERING until Recover, whatever happens in
  between (a reconnect, an unreadable input); while drag teaching runs on either arm, no other
  motion starts;
- a run of N lines takes (N - 1) x 5 ms; an E-stop during a run ends it as INTERRUPTED, never complete;
- program IDs 1-100; replacing a stored program needs the caller's confirmation;
- every run, playback and move to a start stages the pole and then the hand first (pole within
  5 mm, every set hand channel within 30 counts); when either does not arrive, the job ends there
  and the arm never moves (``staging.py``). Times follow ALIX's measured pole and hand models.
  An engineer can make the pole or the hand fail on purpose to test this;
- HARDWARE SIM MOVES NOTHING (D30, R4 (b)): the controllers run programs in their own SIM mode, so
  the arms stay still, and the pole and the hand are EMULATED: their state and their times are
  kept, nothing is sent (``emulated_stage``). Only Real moves them and reads them back. So drag
  teaching, which needs the real pole and hand, needs Real;
- every motion keeps the mode it was asked (and confirmed) in: when the mode is no longer that one,
  it does not start, or it fails before staging and before its program, so a Real run never goes
  on under Hardware sim and a Hardware sim request never moves the real pole or hand.
"""
from __future__ import annotations

import json
import math
import os
import re
import threading
import time
from pathlib import Path

from . import staging
from .jobs import Jobs, UserError

ARMS = ("left", "right")
DEFAULT_IP = {"left": "192.168.1.10", "right": "192.168.1.103"}
UPLOAD_BYTES_S = 45_000            # the controller's measured transfer rate
REAL_UPLOAD_BYTES_S = 20_000       # sent and read back, as on the lab's robot (127,846 bytes in 6.4 s, 14 Oct)
POLE_MM_S = 25.0
# The stand-in's own clock against the wall clock, as ALIX's emulator has RM_EMU_TIME_SCALE: 1 in use;
# the tests set it higher, so a 15 s program or a pole move plays in a tenth of the time, every step
# still taken. What a job reports stays in the robot's own seconds.
TIME_SCALE = max(1e-6, float(os.environ.get("LITE_STANDIN_TIME_SCALE", "1") or 1))


def _s(seconds: float) -> float:
    """``seconds`` of the robot's time on the wall clock."""
    return float(seconds) / TIME_SCALE


MOTION_KINDS = ("run", "prepare", "teach", "move", "playback", "pole", "hand", "stage")   # one at a time per arm
#: jobs that hold an arm's controller connection: none of these starts while another runs on that arm (two
#: commands would share the connection: an upload's rm_send_project beside a run, or two uploads at once)
CONTROLLER_KINDS = MOTION_KINDS + ("upload",)
ESTOP_STOP = ("INTERRUPTED — E-STOP: it did not finish.",
              "Release the E-stop and follow Recover. Nothing restarts by itself.")
MODE_NAME = {"visual": "Visual sim", "hwsim": "Hardware sim", "real": "Real robot"}
ESTOP_TEXT = {
    "NOT_CONFIGURED": "IO3 not in E-stop mode: motion blocked",
    "CLEAR": "E-stop released",
    "ENGAGED": "E-STOP PRESSED",
    "RECOVERING": "Recovering",
    "UNKNOWN": "E-stop unknown",
    "NOT_CONNECTED": "Not connected",
}


class StandInArm:
    def __init__(self, side: str, store: Path):
        self.side = side
        self.ip = DEFAULT_IP[side]
        self.connected = False
        self.estop = "NOT_CONNECTED"
        self.controller_mode = "SIM"
        self.pole_mm = 300.0                    # the robot model's standing pose, where every 3D replay starts
        self.hand = [1000] * 6
        self.teaching = False
        self.teach_started = None
        self.teach_stage = None
        self.staged_hand = None                 # {target, read}: where the last real staging left the hand
        self.recovery = None                    # Arm Drag Mode: "drag" (by hand), "held" (left it, not yet safe)
        self.recovery_note = None               # why it is held, in plain words
        self.estop_latched = False              # pressed (or STOPped) since the last Recover
        self.stop_latched = False               # held by STOP rather than by the E-stop button
        self.button_pressed = False             # the stand-in's own button (an engineer's test)
        self._store = store
        self.programs: dict[int, dict] = {}
        try:
            raw = json.loads(store.read_text(encoding="utf-8"))
            self.programs = {int(k): v for k, v in raw.get("programs", {}).items()}
        except (OSError, ValueError):
            pass

    def save(self) -> None:
        self._store.parent.mkdir(parents=True, exist_ok=True)
        self._store.write_text(json.dumps({"programs": self.programs}, indent=1), encoding="utf-8")

    def view(self) -> dict:
        by_stop = self.estop == "RECOVERING" and self.stop_latched
        return dict(side=self.side, ip=self.ip, connected=self.connected, estop=self.estop, stopped=by_stop,
                    estop_text="Stopped by STOP: press Recover" if by_stop else ESTOP_TEXT[self.estop],
                    estop_io="IO3", controller_mode=self.controller_mode,
                    pole_mm=round(self.pole_mm), hand=list(self.hand), teaching=self.teaching,
                    teach_s=round(time.time() - self.teach_started, 1) if self.teaching else None,
                    recovery=self.recovery, recovery_note=self.recovery_note)


_REPORTS_RE = re.compile(r"^!! the arm reports (.+) at ([\d.]+) s$")
STOP_WAIT_S = 5.0                     # how long STOP's answer waits for each controller's confirmation
CONNECT_WAIT_S = 20.0                 # a controller that has not answered a Connect by then is said not to


def arm_report(line: str) -> dict | None:
    """What the control program's run says the arm reported, as it comes ("!! the arm reports 4105 (0x1009:
    joint speed over its limit (a warning; the run goes on)) at 2.3 s"): {codes, at_s, warn}, or None."""
    m = _REPORTS_RE.match(str(line).strip())
    if not m:
        return None
    return dict(codes=m.group(1), at_s=float(m.group(2)), warn="a warning" in m.group(1))


def ended_early(seconds, lines, scale: float = 1.0, share: float = 0.9) -> str | None:
    """Whether a program's run took less time than its own points need ((lines - 1) x 5 ms, as the
    controller never plays them faster; ``scale``: an emulator's time scale): then it was stopped, an
    E-stop raising no error code. Says how short, or None."""
    try:
        need = max(int(lines) - 1, 0) * 0.005 / max(float(scale), 1e-6)
    except (TypeError, ValueError):
        return None
    if need < 1.0 or float(seconds) >= share * need:
        return None
    return f"after {float(seconds):.1f} s of the {need:.1f} s its points take"


def emulated_stage(job, side: str, a, stage: "staging.Stage", faults=(), *, emulated: bool = True,
                   step: int = 1, steps: int = 1) -> None:
    """The pole to the stage height, then the hand to the stage pose, on ALIX's measured times,
    on arm state ``a`` (pole_mm, hand, estop). ``emulated``: Hardware sim, which sends nothing
    (D30, R4 (b)); otherwise the stand-in acting as the real arm. Raises, and nothing else runs,
    when either does not arrive (``faults``: an engineer's test) or the E-stop is pressed."""
    how = "emulated: Hardware sim moves nothing" if emulated else "read back"
    start, goal = a.pole_mm, float(stage.pole_mm)
    dur = _s(staging.pole_travel_s(start, goal))
    end = start + (0.6 if "pole" in faults else 1.0) * (goal - start)
    t0 = time.time()
    while dur and (el := time.time() - t0) < dur:
        if a.estop != "CLEAR":
            raise UserError("INTERRUPTED — E-STOP while the pole was moving. The arm did not move.",
                            "Release the E-stop and follow Recover.")
        a.pole_mm = start + min(1.0, el / dur) * (end - start)
        job.progress(f"Pole {start:.0f} → {goal:g} mm" + (" (emulated)" if emulated else ""), step, steps)
        job.sleep(min(0.1, dur))
    a.pole_mm = end if dur else a.pole_mm
    if abs(a.pole_mm - goal) > staging.POLE_TOL_MM:
        raise UserError(f"The {side.upper()} pole did not reach {goal:g} mm ({how.split(':')[0]} {a.pole_mm:.0f} mm). "
                        "Nothing else ran.", "Check the pole and try again.")
    job.log(f"Pole at {a.pole_mm:.0f} mm ({how}).")
    target = [h if t is None else int(t) for h, t in zip(a.hand, stage.hand)]
    dur = _s(staging.hand_move_s(a.hand, target))
    if "hand" in faults:
        target = [h + (t - h) // 2 for h, t in zip(a.hand, target)]
    if dur:
        job.progress(f"Hand to {staging.hand_words(stage.hand)}" + (" (emulated)" if emulated else ""), step, steps)
        job.sleep(dur)
        if a.estop != "CLEAR":
            raise UserError("INTERRUPTED — E-STOP while the hand was moving. The arm did not move.",
                            "Release the E-stop and follow Recover.")
    a.hand = target
    if not staging.same_hand(a.hand, stage.hand):
        raise UserError(f"The {side.upper()} hand did not reach its pose ({how.split(':')[0]} "
                        f"{', '.join(map(str, a.hand))}). Nothing else ran.", "Check the hand and try again.")
    job.log(f"Hand in pose ({how}): " + ", ".join(map(str, a.hand)))


class StandInRobot:
    """The stand-in, optionally with a live link to the arms (``link``: "realman" for the real
    controllers, "emulated" for ALIX's emulator; robot_link.py). Linked, Connect reaches the
    controller and the panel shows its feedback (arm, pole, hand, IO3); the real E-stop input
    drives the motion interlock; and the robot's actions go to the controller through ALIX's
    functions (robot_real.py): staging, drag teaching, upload, Real and Hardware sim runs, STOP."""

    def __init__(self, data_dir: Path, jobs: Jobs, link: str | None = None, log=None):
        self.jobs = jobs
        self.log = log                                  # logbook.Logbook: the hardware log and its alerts
        self._seen: dict = {}                           # side -> what was last reported (errors, link, E-stop)
        self.mode = "visual"                            # visual | hwsim | real, set by LiteApp.set_mode
        self.switching: str | None = None               # the mode the controllers are being switched to: no motion meanwhile
        self.link_kind = link
        self.links: dict = {}                           # side -> robot_link.ArmLink while connected
        self.test_estop: dict = {}                      # an engineer's E-stop test on emulated arms
        self.lock = threading.RLock()
        self._send_lock = threading.Lock()              # an upload's check and its start together (a double click sends once)
        self.arms = {s: StandInArm(s, Path(data_dir) / "standin" / f"{s}_programs.json") for s in ARMS}
        self.faults = {s: set() for s in ARMS}         # engineer test: {"pole", "hand"} fail to arrive
        # the speed slider (the owner, 15 Oct): every MoveJ / MoveL Lite sends, both arms; and each arm's
        # controller speed found when it was connected, put back after every move and when Lite lets go
        self.speed_pct = staging.SPEED_FOUND_PCT
        staging.set_move_pct(self.speed_pct)
        self.speed_found = {s: None for s in ARMS}      # side -> the global speed found (None: not reported)
        self.sens_found = {s: None for s in ARMS}       # side -> the drag sensitivity found (None: not reported)
        self.sens_now = {s: None for s in ARMS}         # side -> the drag sensitivity Lite set

    @property
    def read_only(self) -> bool:
        """Lite commands the linked arms (robot_real.py): Real robot is available with them."""
        return False

    @property
    def backend(self) -> str:
        """Where robot actions go: "stand-in", or the linked controllers ("realman", "emulated")."""
        return self.link_kind or "stand-in"

    @property
    def label(self) -> str:
        if self.link_kind == "realman":
            return "Live link to the real arms: Lite reads and commands them"
        if self.link_kind == "emulated":
            return "Live link to ALIX's emulated arms (no hardware)"
        return "Stand-in robot (no hardware)"

    def _real(self, side: str):
        """A linked arm's controller commands (robot_real.RealArm), or None for the stand-in."""
        link = self.links.get(side)
        if link is None:
            return None
        from .robot_real import RealArm

        return RealArm(link, self.log)

    def _measured(self, side: str) -> tuple:
        """The pole (mm) and the hand as they are now: read from a linked controller, else the stand-in's."""
        link = self.links.get(side)
        if link is None:
            a = self.arm(side)
            return a.pole_mm, list(a.hand)
        fb = link.view()["feedback"]
        return fb.get("pole_mm"), fb.get("hand")

    def _leave_drag(self, side: str, at_once: bool = False) -> None:
        """A linked arm in drag teaching leaves it (STOP, an E-stop, a cancel, Lite closing); ``at_once``
        (STOP, an E-stop): without waiting for a command that holds its controller."""
        real = self._real(side)
        if real is not None:
            real.teach_cancel(at_once)

    def _engage(self, side: str) -> None:
        """An E-stop press: latched until Recover; every motion job on BOTH arms ends as
        INTERRUPTED, and drag teaching stops."""
        a = self.arms[side]
        newly = a.estop != "ENGAGED"
        a.estop, a.estop_latched, a.stop_latched = "ENGAGED", True, False
        if newly:
            self._alert("warn", side, "E-stop pressed: robot motion is blocked on both arms.",
                        "Make the cell safe, release the button, then press Recover.", "estop")
            for arm in self.arms.values():
                if (arm.teaching or arm.recovery == "drag") and arm.side in self.links:
                    threading.Thread(target=self._leave_drag, args=(arm.side, True), daemon=True).start()
                arm.teaching = False
                if arm.recovery == "drag":
                    arm.recovery, arm.recovery_note = "held", "the E-stop was pressed in Arm Drag Mode"
            raised = None
            for j in self.jobs.running():
                if j.kind in MOTION_KINDS:
                    j.log(f"E-stop pressed on the {side.upper()} arm.", "error")
                    try:
                        j.cancel(*ESTOP_STOP)
                    except BaseException as exc:                # noqa: BLE001 - every job is stopped first
                        raised = raised or exc
            if raised is not None:
                raise raised

    def estop_check(self, side: str, on_press=None) -> bool:
        """IO3 read once by a command that holds a linked arm, between two of its steps, in its own
        thread (S2's runner: while a command runs the link does not read). A press is the E-stop's:
        latched, and every motion job on both arms stopped (their stop hooks interrupt the runners).
        True when it was pressed (``on_press`` is called first). Only a press stops: an unreadable
        input is logged, as the link's reading does not stop a running job either.
        The press is engaged from a thread of its own, the caller waiting for it: a stop hook interrupts
        the runner's thread (robot_real._interrupt), which is the caller's, and run there it would cut
        the stops short (the other arm's hook, the other jobs); the caller's interrupt comes once every
        stop has been sent."""
        link = self.links.get(side)
        if link is None:
            return False
        live = "ENGAGED" if link.emulated and self.test_estop.get(side) else link.estop_now()
        if live == "ENGAGED":
            if on_press is not None:
                on_press()
            done = threading.Event()

            def engage():
                try:
                    with self.lock:
                        self._engage(side)
                finally:
                    done.set()

            threading.Thread(target=engage, name=f"lite-estop-{side}", daemon=True).start()
            done.wait(15)
            return True
        if live != "CLEAR" and self.log is not None:
            self.log.hardware(side, "estop check", ok=False, text=f"IO3 reads {live} between two steps")
        return False

    def emulator_scale(self, side: str) -> float:
        """How much faster than real time a linked emulated arm plays (ALIX's emulator time scale); 1 on hardware."""
        link = self.links.get(side)
        if link is None or not link.emulated:
            return 1.0
        try:
            from alix_emulator import rm_emulator

            return max(1e-6, float(rm_emulator._time_scale))
        except Exception:                                                       # noqa: BLE001
            return 1.0

    def _sync(self, side: str) -> None:
        """The arm's E-stop chip from the live IO3 reading, failing closed (unreadable, stale, not in
        mode 5, or linked but without its link: blocked). After a press it reads RECOVERING until
        Recover, whatever came in between. Called by every request and by the link after each read."""
        with self.lock:
            a = self.arms[side]
            link = self.links.get(side)
            if link is None:
                if self.link_kind and a.connected:
                    a.estop = "UNKNOWN"
                return
            fb = link.view()["feedback"]
            self._report(side, fb)
            live = fb.get("estop", "UNKNOWN")
            if link.emulated and self.test_estop.get(side):
                live = "ENGAGED"
            if live == "ENGAGED":
                self._engage(side)
            elif live in ("UNKNOWN", "NOT_CONFIGURED"):
                a.estop = live
            else:
                a.estop = "RECOVERING" if a.estop_latched else "CLEAR"

    # ── hardware alerts (logbook.py; the page shows them at the bottom right until closed)
    def _alert(self, level: str, side: str | None, text: str, hint: str | None = None, source: str | None = None) -> None:
        if self.log is not None:
            self.log.alert(level, text, hint=hint, side=side, source=source)

    def _once(self, side: str, what: str, now, level: str, text: str, hint: str | None = None) -> None:
        """An alert when ``what`` changes to a reportable value ``now`` (None: back to normal)."""
        key = (side, what)
        if self._seen.get(key) == now:
            return
        self._seen[key] = now
        if now:
            self._alert(level, side, text, hint, what)

    def _report(self, side: str, fb: dict) -> None:
        """What a linked controller's feedback says that the operator must see: its error codes, a
        link that stopped reading, IO3 not in E-stop mode."""
        from .robot_link import STALE_S, error_text

        errs = tuple(fb.get("errors") or ())
        self._once(side, "errors", errs or None, "error",
                   "The controller reports " + "; ".join(error_text(c) for c in errs) + ".",
                   "Check the arm and its pendant; clear the error there before running.")
        age = fb.get("age_s")
        stale = fb["stale"] if "stale" in fb else (age is None or age > STALE_S) and not fb.get("busy")   # robot_link.view
        self._once(side, "link", "stale" if stale else None, "warn",
                   "No fresh reading from the controller: robot motion is blocked.",
                   "Check the controller's power and cable; Lite reconnects by itself when it answers.")
        self._once(side, "io3", fb.get("estop") if fb.get("estop") == "NOT_CONFIGURED" else None, "error",
                   "IO3 is not in the controller's E-stop mode (5): its E-stop button would not stop the arm.",
                   "An engineer sets IO3 to mode 5 at the pendant. Motion stays blocked until then.")

    # ── state
    def view(self) -> dict:
        out = {}
        for s, a in self.arms.items():
            self._sync(s)
            out[s] = a.view() | {"faults": sorted(self.faults[s])}
            if s in self.links:
                out[s]["link"] = self.links[s].view()
        return out

    def arm(self, side: str) -> StandInArm:
        if side not in self.arms:
            raise UserError(f"Unknown arm {side!r}.")
        return self.arms[side]

    def motion_block(self) -> str | None:
        """Why motion is refused right now, in plain words (None = allowed)."""
        for s in self.links:
            self._sync(s)
        for s, a in self.arms.items():
            if a.connected and a.estop == "RECOVERING" and a.stop_latched:
                return (f"The {s.upper()} arm was stopped by STOP. Robot motion is blocked on both arms "
                        "until you press Recover.")
            if a.connected and a.estop != "CLEAR":
                return (f"The {s.upper()} arm's E-stop is {ESTOP_TEXT[a.estop].lower()}. "
                        "Robot motion is blocked on both arms until it is released and recovered.")
        return None

    def busy(self) -> str | None:
        """What is moving (or being taught) right now, on either arm; None when nothing is."""
        for a in self.arms.values():
            if a.teaching:
                return f"drag teaching on the {a.side.upper()} arm"
            if a.recovery:
                return f"Arm Drag Mode on the {a.side.upper()} arm" if a.recovery == "drag" else \
                    f"the {a.side.upper()} arm held after Arm Drag Mode"
        jobs = [j for j in self.jobs.running() if j.kind in MOTION_KINDS]
        return jobs[0].title if jobs else None

    def require_motion(self, side: str, alone: bool = False) -> None:
        """Every motion's interlocks. ``alone``: the other arm must be still too, and stays so while
        this runs (V32's single-arm transition: "the opposite arm must remain stopped")."""
        sides = ARMS if side == "both" else (side,)
        for s in sides:
            if not self.arm(s).connected:
                raise UserError(f"The {s.upper()} arm is not connected.", "Connect it from the bar at the top.")
        why = self.motion_block()
        if why:
            raise UserError(why, "Release the E-stop button, then press Recover.")
        if self.switching:
            raise UserError(f"The controllers are switching to {self.switching}.", "Try again in a moment.", code="busy")
        teaching = [a.side for a in self.arms.values() if a.teaching]
        if teaching:
            raise UserError(f"Drag teaching is running on the {teaching[0].upper()} arm.",
                            "Stop the teaching first: nothing else moves while an arm is being guided by hand.")
        held = [a for a in self.arms.values() if a.recovery]
        if held:
            raise UserError(f"The {held[0].side.upper()} arm is in Arm Drag Mode." if held[0].recovery == "drag"
                            else f"The {held[0].side.upper()} arm is held after Arm Drag Mode: {held[0].recovery_note}.",
                            "Press Normal Mode: nothing else moves until the arm is safe.")
        busy = [j for s in sides for j in self.jobs.running(arm=s) if j.kind in CONTROLLER_KINDS]
        if busy:
            raise UserError(f"The {busy[0].arm.upper()} arm is busy: {busy[0].title}.", "Wait, or press STOP.")
        lone = [j for j in self.jobs.running() if getattr(j, "alone", False) and j.arm not in sides]
        if lone:
            raise UserError(f"{lone[0].title} runs on the {lone[0].arm.upper()} arm alone: the {sides[0].upper()} arm "
                            "stays stopped until it ends.", "Wait, or press STOP.")
        if alone:
            other = [j for j in self.jobs.running() if j.kind in MOTION_KINDS and j.arm not in sides]
            if other:
                raise UserError(f"The {other[0].arm.upper()} arm is moving ({other[0].title}); this runs on the "
                                f"{sides[0].upper()} arm only while the other arm is stopped.", "Wait, or press STOP.")

    def _same_mode(self, mode: str, done: str = "nothing was staged or moved") -> None:
        """Refuses when the mode is no longer ``mode``, the one the motion was asked (and
        confirmed) in."""
        if self.mode != mode:
            raise UserError(f"The mode changed to {MODE_NAME.get(self.mode, self.mode)} after this was asked for in "
                            f"{MODE_NAME.get(mode, mode)}, so {done}.", "Check the mode at the top, then ask again.",
                            code="mode")

    def _start(self, kind: str, title: str, work, side: str, mode: str, alone: bool = False):
        """The interlocks and the job's start in one step, so two requests cannot both pass, and
        only while the mode is still the one the request was made in."""
        with self.lock:
            self._same_mode(mode)
            self.require_motion(side, alone)
            job = self.jobs.start(kind, title, work, arm=side)
            job.alone = alone
            return job

    def close(self) -> None:
        """Let go of every live link (the server is stopping, after STOP); an arm in drag teaching
        leaves it first, and every controller gets the global speed it had when it was connected, once
        the jobs STOP ended have ended (their own ``finally`` runs on a link still open)."""
        for side, a in self.arms.items():
            if a.teaching or a.recovery == "drag":
                a.teaching, a.recovery = False, None
                self._leave_drag(side)
        for side in list(self.links):
            self._settle(side)
            try:
                self._give_back_speed(side)
                self._give_back_sensitivity(side)
            except UserError:                           # its controller still busy: every link is let go all the same
                pass
        with self.lock:
            links, self.links = list(self.links.values()), {}
        for link in links:
            link.close()

    # ── the speed slider: MoveJ / MoveL for both arms, and the controller's own speed put back
    def found_speed(self, side: str) -> int:
        """The global speed this arm's controller had when it was connected, or the 50 % ALIX and the
        control program's runner leave (the controller does not report it: ALIX's F98)."""
        f = self.speed_found.get(side)
        return int(f) if f else staging.SPEED_FOUND_PCT

    def speed_view(self) -> dict:
        return dict(pct=int(self.speed_pct), assumed=staging.SPEED_FOUND_PCT,
                    found={s: dict(pct=self.found_speed(s), read=self.speed_found[s] is not None,
                                   connected=self.arms[s].connected) for s in ARMS})

    def set_speed(self, pct) -> dict:
        try:
            v = int(round(float(pct)))
        except (TypeError, ValueError):
            raise UserError("The speed is 1 to 100 %.") from None
        if not 1 <= v <= 100:
            raise UserError("The speed is 1 to 100 % (the controller takes no 0).")
        self.speed_pct = v
        staging.set_move_pct(v)                          # the 3D's moves follow it
        return self.speed_view()

    def _read_speed(self, side: str) -> None:
        """On connecting: the controller's global speed, when it says (the slider starts there when it
        is the first arm connected); else the 50 % that ALIX and the runner leave."""
        real = self._real(side)
        got = real.read_speed() if real is not None else None
        first = not any(self.arms[o].connected for o in ARMS if o != side)
        with self.lock:
            self.speed_found[side] = got
            if first:
                self.set_speed(self.found_speed(side))
        if self.log is not None:
            self.log.activity(f"{side.upper()} global speed " + (f"read: {got} %" if got is not None else
                              f"not reported by the controller: taken as {staging.SPEED_FOUND_PCT} %")
                              + f"; the speed slider is at {self.speed_pct} %", detail=dict(side=side, found=got))

    def _give_back_speed(self, side: str) -> None:
        real = self._real(side)
        if real is not None:
            real.restore_speed(self.found_speed(side))

    def _refuse_if_busy(self, side: str, what: str) -> None:
        if self.arm(side).teaching:                      # its controller would be left in drag teaching
            raise UserError(f"The {side.upper()} arm is in drag teaching. It stays connected.",
                            f"Press STOP DRAG, or STOP, then {what}.", code="busy")
        if self.arm(side).recovery:                      # or in drag mode, or not yet safe after it
            raise UserError(f"The {side.upper()} arm is in Arm Drag Mode. It stays connected.",
                            f"Press Normal Mode, then {what}.", code="busy")
        busy = [j for j in self.jobs.running(arm=side) if not j.cancelled]
        if busy:
            raise UserError(f"The {side.upper()} arm is busy: {busy[0].title}. It stays connected.",
                            f"Press STOP, or let it finish, then {what}.", code="busy")

    def _settle(self, side: str, timeout_s: float = 5.0) -> None:
        """Wait (at most ``timeout_s``) for the arm's stopped jobs to end."""
        t_end = time.monotonic() + timeout_s
        while self.jobs.running(arm=side) and time.monotonic() < t_end:
            time.sleep(0.05)

    # ── the drag sensitivity (feedback v5, slide 2): read on connecting, set from Drag & Teach, and the
    #    controller's own given back when Lite lets go, as the speed
    def _read_sensitivity(self, side: str) -> None:
        real = self._real(side)
        self.sens_found[side] = self.sens_now[side] = real.read_sensitivity() if real is not None else None

    def _give_back_sensitivity(self, side: str) -> None:
        real, found = self._real(side), self.sens_found.get(side)
        if real is not None and found is not None and self.sens_now.get(side) not in (None, found):
            try:
                real.set_sensitivity(found)
                self.sens_now[side] = found
            except UserError:
                pass

    def sensitivity_view(self) -> dict:
        return {s: dict(pct=self.sens_now[s], found=self.sens_found[s], linked=s in self.links) for s in ARMS}

    def set_sensitivity(self, side: str, pct) -> dict:
        """The arm's drag sensitivity, 0-100 % (higher: easier to move by hand), set on its controller and
        read back. Not while the arm is being guided by hand."""
        try:
            v = int(round(float(pct)))
        except (TypeError, ValueError):
            raise UserError("The drag sensitivity is 0 to 100 %.") from None
        if not 0 <= v <= 100:
            raise UserError("The drag sensitivity is 0 to 100 %.")
        a, real = self.arm(side), self._real(side)
        if real is None:
            raise UserError(f"The drag sensitivity is the {side.upper()} controller's: connect the arm with its link.",
                            "Start Lite with --robot realman and connect the arm.", code="connect")
        if a.teaching or a.recovery or self.jobs.running(arm=side, kind="teach"):
            raise UserError(f"The {side.upper()} arm is being guided by hand: its drag sensitivity changes after.", code="busy")
        self.sens_now[side] = real.set_sensitivity(v)
        return self.sensitivity_view()[side]

    # ── Arm Drag Mode and Normal Mode (feedback v5, slide 8): one arm free to be moved by hand, for
    #    recovery after a collision or an unexpected stop, never for recording
    def drag_mode(self, side: str, on: bool, mode: str | None = None) -> dict:
        """Drag Mode needs Real, the arm linked, its E-stop button released, nothing running on it, no
        program running or paused on its controller and every joint enabled (a disabled joint drops when
        its brake releases); it is allowed while STOP holds the arm, as recovering from a stop is what it
        is for. While an arm is in it, nothing else moves on either arm. Normal Mode leaves it and checks
        the arm is safe: at rest, no program, no error latched, every joint enabled; until then the arm
        stays held (``recovery`` "held", with why) and nothing moves."""
        a, real = self.arm(side), self._real(side)
        mode = mode or self.mode
        if on:
            if a.recovery == "drag":
                return a.view()
            if mode != "real":
                raise UserError("Arm Drag Mode frees the real arm, so it needs Real robot.",
                                "Choose Real robot in the status pill.", code="mode")
            if real is None:
                raise UserError(f"The {side.upper()} arm is not connected to its controller.",
                                "Start Lite with --robot realman and connect the arm.", code="connect")
            self._sync(side)
            if a.estop == "ENGAGED" or a.button_pressed:
                raise UserError(f"The {side.upper()} arm's E-stop button is pressed.", "Release it, then try again.")
            busy = [j for j in self.jobs.running(arm=side) if j.kind in MOTION_KINDS]
            if a.teaching or busy:
                raise UserError(f"The {side.upper()} arm is busy: {'drag teaching' if a.teaching else busy[0].title}.",
                                "Press STOP first.", code="busy")
            why = real.drag_ready()
            if why:
                raise UserError(f"Arm Drag Mode is refused on the {side.upper()} arm: {'; '.join(why)}.",
                                "Check the arm on its pendant.")
            real.drag_on()
            a.recovery, a.recovery_note = "drag", None
            if self.log is not None:
                self.log.activity(f"{side.upper()} arm in Arm Drag Mode (recovery, nothing recorded)", detail=dict(side=side))
            return a.view()
        if not a.recovery:
            return dict(a.view(), check=None)
        if real is not None and a.recovery == "drag" and not real.drag_off():
            a.recovery, a.recovery_note = "drag", "the controller did not confirm it left drag mode"
            raise UserError(f"The {side.upper()} arm may still be in drag mode.",
                            "Press Normal Mode again, or STOP, before anyone lets go of the arm.")
        a.recovery = "held"
        check = real.safe_state() if real is not None else dict(ok=True, problems=[], errors=None)
        a.recovery, a.recovery_note = (None, None) if check["ok"] else ("held", "; ".join(check["problems"]))
        if self.log is not None:
            self.log.activity(f"{side.upper()} arm: Normal Mode" + ("" if check["ok"] else f", held: {a.recovery_note}"),
                              detail=dict(side=side, check=check))
        return dict(a.view(), check=check)

    def errors(self, side: str) -> dict:
        real = self._real(side)
        if real is None:
            raise UserError(f"The {side.upper()} arm's errors are its controller's: connect it with its link.", code="connect")
        return real.errors()

    def clear_errors(self, side: str) -> dict:
        """The controller's errors cleared, as its web GUI's Clear Error buttons do (nothing moves). An arm
        held after Arm Drag Mode is checked again."""
        real = self._real(side)
        if real is None:
            raise UserError(f"The {side.upper()} arm's errors are its controller's: connect it with its link.", code="connect")
        out = real.clear_errors()
        a = self.arm(side)
        if a.recovery == "held":
            check = real.safe_state()
            a.recovery, a.recovery_note = (None, None) if check["ok"] else ("held", "; ".join(check["problems"]))
            out["check"] = check
        if self.log is not None:
            self.log.activity(f"{side.upper()} controller: errors cleared" + ("" if out["clean"] else f", still {out['text']}"),
                              detail=dict(side=side, errors=out))
        return out

    # ── connection and safety
    def _open_link(self, link):
        """``link`` opened on a thread of its own, so a controller that does not answer is said after
        CONNECT_WAIT_S instead of holding the request with no answer at all (the lab, 17 Oct: after STOPs
        that went unanswered, four Connects got no answer, each press one more connection). A link that
        opens after that is closed at once."""
        box, lock, done = {}, threading.Lock(), threading.Event()

        def work():
            try:
                got = link.open()
            except BaseException as exc:                                                 # noqa: BLE001
                with lock:
                    box["error"] = exc
                done.set()
                return
            with lock:
                box["link"], late = got, box.get("late")
            done.set()
            if late:
                got.close()

        threading.Thread(target=work, name=f"lite-connect-{link.side}", daemon=True).start()
        if not done.wait(CONNECT_WAIT_S):
            with lock:
                late = "link" not in box and "error" not in box
                box["late"] = late
            if late:
                raise UserError(f"The {link.side.upper()} controller did not answer within {CONNECT_WAIT_S:g} s, so the "
                                "arm is not connected.", "Check its power and cable. If it still does not answer, restart "
                                "its controller, then Lite.")
        if "error" in box:
            raise box["error"]
        return box["link"]

    def connect(self, side: str, ip: str | None = None) -> dict:
        a = self.arm(side)
        target = (ip or a.ip).strip()
        if self.link_kind:
            from .robot_link import ArmLink

            if side in self.links:                       # its old link is closed below: not under a running job
                self._refuse_if_busy(side, "connect again")
            # the new link first: when it cannot open, the arm keeps its old link (or stays
            # disconnected), never "connected" without a reading behind its E-stop chip
            new = self._open_link(ArmLink(side, target, on_read=lambda: self._sync(side)))
            with self.lock:
                old = self.links.get(side)
                self.links[side] = new
                a.ip, a.connected = target, True
            if old is not None:
                old.close()
            self._sync(side)
            self._read_speed(side)
            self._read_sensitivity(side)
            if self.mode in ("hwsim", "real"):          # connected in Hardware sim or Real: its controller follows
                try:
                    self.set_controller_modes(self.mode, [side])
                except UserError as exc:
                    self._alert("warn", side, f"{side.upper()} connected, but {exc}", "Check its mode on the pendant.", "runmode")
            return self.view()[side]
        time.sleep(_s(0.4))
        self._read_speed(side)
        with self.lock:
            a.ip, a.connected = target, True
            a.estop = "ENGAGED" if a.button_pressed else "RECOVERING" if a.estop_latched else "CLEAR"
        return a.view()

    def set_controller_modes(self, mode: str, sides=None) -> dict:
        """Hardware sim puts each linked controller in its SIM mode and Real in its REAL mode, read back (the
        owner, 17 Oct: "When Hardware Sim is selected, the controller moves to sim mode"; the lab's controllers
        had stayed in REAL, one put in SIM by hand on its pendant). Visual sim leaves them as they are. All or
        none: a controller that does not take it puts back the ones switched, and the error says which.
        Returns {side: "SIM" | "REAL"} for the controllers switched."""
        if mode not in ("hwsim", "real"):
            return {}
        want = "SIM" if mode == "hwsim" else "REAL"
        done, out = [], {}
        for side in (sides or ARMS):
            real = self._real(side)
            if real is None or not self.arm(side).connected:
                continue
            try:
                originals = real.set_run_mode(want)
            except UserError:
                for s, orig in done:
                    try:
                        self._real(s).restore_run_mode(orig)
                    except Exception:                                           # noqa: BLE001 - said by its own alert
                        pass
                raise UserError(f"the {side.upper()} controller did not switch to its {want} mode (read back)",
                                "Check its errors on the pendant, then choose the mode again.", code="runmode") from None
            done.append((side, originals))
            self.arm(side).controller_mode = want
            out[side] = want
        return out

    def disconnect(self, side: str) -> dict:
        """A linked arm lets go only once nothing runs on it: its controller's own speed is put back,
        then the link closes (a move or run under way would lose its link mid-command, and with it its
        own ``finally`` that puts the speed back)."""
        a = self.arm(side)
        if side in self.links:
            self._refuse_if_busy(side, "disconnect")
            self._settle(side)                           # a job STOP ended finishes on the open link
            self._give_back_speed(side)                  # the controller's own speed, as found on connecting
            self._give_back_sensitivity(side)            # and its drag sensitivity
        with self.lock:
            self.jobs.cancel_arm(side)
            link = self.links.pop(side, None)
            a.connected, a.estop, a.teaching = False, "NOT_CONNECTED", False   # the latch is kept
        if link is not None:
            link.close()
        return a.view()

    def set_controller_mode(self, side: str, mode: str) -> dict:
        if mode not in ("SIM", "REAL"):
            raise UserError(f"Unknown controller mode {mode!r}.")
        a = self.arm(side)
        if not a.connected:
            raise UserError(f"The {side.upper()} arm is not connected.")
        real = self._real(side)
        if real is not None:
            real.set_run_mode(mode)                     # read back; an engineer's setting, not restored
        a.controller_mode = mode
        return a.view()

    def press_estop(self, side: str, pressed: bool) -> dict:
        """Engineer test only: act as if the IO3 button were pressed or released."""
        a = self.arm(side)
        if not a.connected:
            raise UserError(f"The {side.upper()} arm is not connected.")
        link = self.links.get(side)
        if link is not None and not link.emulated:
            raise UserError("These are the real arms: test the E-stop with the red button itself.",
                            "Lite reads IO3 and shows the press within a fraction of a second.")
        if link is not None:                            # emulated arms: ALIX's emulator has no IO
            self.test_estop[side] = bool(pressed)
            self._sync(side)
            return a.view()
        with self.lock:
            a.button_pressed = bool(pressed)
            if pressed:
                self._engage(side)
            elif a.estop == "ENGAGED":
                a.estop = "RECOVERING"
        return a.view()

    def recover(self, side: str) -> dict:
        a = self.arm(side)
        self._sync(side)
        with self.lock:
            if a.estop in ("UNKNOWN", "NOT_CONFIGURED"):
                raise UserError(f"The {side.upper()} arm's E-stop input reads {a.estop.replace('_', ' ').lower()}.",
                                "Lite needs IO3 readable and in the controller's E-stop mode (5) before motion.")
            if a.estop == "ENGAGED" or a.button_pressed:
                raise UserError("The E-stop button is still pressed.", "Make the cell safe, then release the button.")
            a.estop_latched = a.stop_latched = False
            if a.connected:
                a.estop = "CLEAR"
        return a.view()

    def stop(self, side: str) -> dict:
        """STOP: never refused, and it acts like the E-stop. "all" and "both" mean every arm.

        A linked arm's controller gets ALIX's stop first (robot_link.ArmLink.stop), both at once:
        STOP BOTH the E-stop's way, one arm's STOP on its path. Then Lite's jobs and drag teaching
        end, and every connected arm stopped is held until Recover, as after an E-stop.
        ``controllers``: what each controller confirmed; ``ok`` False when one did not."""
        every = side in ("all", "both")
        sides = [s for s in (ARMS if every else (side,)) if s in self.arms]
        snap = dict(self.links)
        links = {s: snap[s] for s in sides if s in snap}
        got: dict = {}                                  # what each controller confirmed, written by its thread
        teaching = {s for s in links if self.arms[s].teaching or self.arms[s].recovery == "drag"}
        t_sent = time.time()

        def one(s, link):
            if s in teaching:
                self._leave_drag(s, at_once=True)       # drag teaching ends first, then the stop
            got[s] = link.stop(hard=every)

        threads = [threading.Thread(target=one, args=(s, l), name=f"lite-stop-{s}", daemon=True) for s, l in links.items()]
        for t in threads:
            t.start()
        with self.lock:
            n = self.jobs.cancel_arm("all" if every else side)
            held = []
            for s in sides:
                a = self.arms[s]
                a.teaching = False
                if a.recovery == "drag":
                    a.recovery, a.recovery_note = "held", "STOP was pressed in Arm Drag Mode"
                if a.connected:
                    held_by_button = a.estop == "ENGAGED" or (a.estop == "RECOVERING" and not a.stop_latched)
                    a.estop_latched = True
                    a.stop_latched = a.stop_latched or not held_by_button   # an E-stop press keeps its own words
                    if a.estop == "CLEAR":
                        a.estop = "RECOVERING"
                    held.append(s)
        for t in threads:
            t.join(timeout=STOP_WAIT_S)
        sent = dict(got)
        # ALIX's whole stop (the arm, the pole, the hand frozen through Modbus, then the proof that the arm is
        # still) can take more than 5 s while the controller answers (the lab, 17 Oct: "no answer" from both, which
        # took Recover and Clear errors in those same seconds): said as not confirmed yet, and its own result is
        # logged and shown when it comes
        late = [s for s in links if s not in sent]
        for s in late:
            sent[s] = dict(ok=False, text=f"{s.upper()}: STOP sent, not confirmed by its controller within {STOP_WAIT_S:g} s")
        if late:
            threading.Thread(target=self._stop_confirmed_late, args=(late, threads, got, t_sent), name="lite-stop-late",
                             daemon=True).start()
        ok = all(r["ok"] for r in sent.values())
        for s, r in sent.items():
            if not r["ok"]:
                self._alert("error", s, r["text"] + ".", "Press the red E-stop button on the robot if it is still moving.", "stop")
        names = " and ".join(x.upper() for x in held)
        text = " ".join(sent[s]["text"] + "." for s in ARMS if s in sent) + (
            f" Nothing moves until you press Recover on the {names} arm{'s' if len(held) > 1 else ''} (Robot & mode)."
            if held else "")
        return dict(stopped=n, side="all" if every else side, controllers=sent, ok=ok, held=held, text=text.strip())

    def _stop_confirmed_late(self, late: list, threads: list, got: dict, t_sent: float) -> None:
        """What a controller confirmed of STOP after the 5 s the request waits for: logged, and shown."""
        for t in threads:
            t.join(timeout=60.0)
        for s in late:
            r = got.get(s)
            took = time.time() - t_sent
            if r is None:
                self._alert("error", s, f"{s.upper()}: its controller has not confirmed STOP after {took:.0f} s.",
                            "Press the red E-stop button on the robot if it is still moving.", "stop")
            else:
                if self.log is not None:
                    self.log.hardware(s, "stop confirmed late", ok=bool(r.get("ok")), elapsed_s=took, text=r.get("text"))
                self._alert("warn" if r.get("ok") else "error", s, f"{r.get('text')} (confirmed after {took:.1f} s).",
                            None if r.get("ok") else "Press the red E-stop button on the robot if it is still moving.", "stop")

    # ── staging: pole, then hand, each read back; nothing moves the arm unless both arrived
    def set_fault(self, side: str, pole: bool | None = None, hand: bool | None = None) -> dict:
        f = self.faults[self.arm(side).side]
        for name, on in (("pole", pole), ("hand", hand)):
            if on is True:
                f.add(name)
            elif on is False:
                f.discard(name)
        return dict(side=side, faults=sorted(f))

    def _stage(self, job, side: str, stage: "staging.Stage", mode: str, step: int = 1, steps: int = 1) -> None:
        """The pole, then the hand: moved and read back in Real, emulated otherwise (the stand-in
        only takes the time either way; the real backend sends the commands in Real). Nothing is
        staged when the mode is no longer ``mode``, the one the job was asked in."""
        self._same_mode(mode)
        real = self._real(side)
        if real is not None and mode == "real":          # the real pole and hand, at 50 %, read back
            job.progress(f"Staging the {side.upper()} pole, then the hand (50 % speed)", step, steps)
            out = real.stage(stage)
            a = self.arm(side)
            found = (out.get("pole") or {}).get("found_m")
            a.pole_mm = stage.pole_mm if found is None else found * 1000.0
            a.hand = list((out.get("hand") or {}).get("read") or a.hand)
            a.staged_hand = dict(target=list(stage.hand), read=list(a.hand))      # where this pose left it
            job.log(f"Pole at {a.pole_mm:.0f} mm, then hand in pose: " + ", ".join(map(str, a.hand))
                    + " (read back on the controller).")
            job.check()
            return
        emulated_stage(job, side, self.arm(side), stage, self.faults[side], emulated=mode != "real",
                       step=step, steps=steps)

    @staticmethod
    def _then(work, after):
        """``work``, then ``after(its result)`` inside the job: the job is done only once that ran."""
        if after is None:
            return work

        def run(job):
            out = work(job)
            after(out)
            return out
        return run

    def stage(self, side: str, stage: "staging.Stage", title: str, mode: str | None = None, on_done=None):
        """Stage the pole and the hand only (Prepare's Apply). ``on_done(result)`` runs before the job ends."""
        mode = mode or self.mode

        def work(job):
            self._stage(job, side, stage, mode)
            return dict(pole_mm=self.arm(side).pole_mm, hand=self.arm(side).hand)

        return self._start("stage", title, self._then(work, on_done), side, mode)

    def stage_part(self, side: str, stage: "staging.Stage", part: str, title: str, mode: str | None = None,
                   on_done=None):
        """Prepare's Apply beside the pole or beside the hand: that one alone (the owner, 15 Oct: "If I
        click apply for pole, hand apply automatically triggered"; and again on 15 Oct: the pole's Apply
        moves the pole only, the hand's the hand only). A channel the stage holds keeps the value it
        reads now; what the hand reads after is what this pose left (as staging keeps it, for a hand
        that settled short on the tool)."""
        if part == "pole":
            return self.move_pole(side, stage.pole_mm, title, mode=mode, on_done=on_done)
        if part != "hand":
            raise UserError("Apply the pole or the hand.")
        commanded = side in self.links and (mode or self.mode) == "real"
        now = self._measured(side)[1] or ([None] * 6 if commanded else self.arm(side).hand)   # a hand not read holds nothing
        if any(t is None and h is None for h, t in zip(now, stage.hand)):
            raise UserError(f"The {side.upper()} hand's reading is not known yet, so a channel left empty cannot be held.",
                            "Try again in a moment, or set all six channels.")
        counts = [int(h if t is None else t) for h, t in zip(now, stage.hand)]

        def after(out):
            a = self.arm(side)
            a.staged_hand = dict(target=list(stage.hand), read=list(a.hand))
            if on_done is not None:
                on_done(out)

        return self.set_hand(side, counts, title, mode=mode, on_done=after)

    def in_stage(self, side: str, stage: "staging.Stage") -> bool:
        pole_mm, hand = self._measured(side)
        if pole_mm is None or hand is None or abs(pole_mm - stage.pole_mm) > staging.POLE_TOL_MM:
            return False
        if staging.same_hand(hand, stage.hand):
            return True
        # a hand that settled short of the pose (on the tool, as the control program allows) is in it
        # while it still reads what staging that same pose left
        last = self.arm(side).staged_hand
        return bool(last and last["target"] == list(stage.hand) and staging.same_hand(hand, last["read"]))

    # ── pole, hand, teach
    def move_pole(self, side: str, pole_mm: float, title: str, mode: str | None = None, on_done=None):
        """The pole alone to ``pole_mm`` (Prepare's Apply beside the pole, Advanced): nothing else moves."""
        a = self.arm(side)
        mode = mode or self.mode
        if not 0 <= float(pole_mm) <= staging.POLE_MAX_MM:
            raise UserError(f"The pole height must be 0 to {staging.POLE_MAX_MM} mm.")

        def work(job):
            self._same_mode(mode)
            real = self._real(side)
            if real is not None and mode == "real":
                job.progress(f"Moving the pole to {float(pole_mm):.0f} mm (50 % speed)", 1, 1)
                out = real.pole_to(float(pole_mm))
                a.pole_mm = float(out.get("final_m") or float(pole_mm) / 1000.0) * 1000.0
                job.log(f"Pole at {a.pole_mm:.0f} mm (read back on the controller).")
                return dict(pole_mm=a.pole_mm)
            start, goal = a.pole_mm, float(pole_mm)
            dur = _s(max(0.5, abs(goal - start) / POLE_MM_S))
            t0 = time.time()
            while (f := min(1.0, (time.time() - t0) / dur)) < 1.0:
                a.pole_mm = start + f * (goal - start)
                job.progress(f"Moving the pole to {goal:.0f} mm", 1, 1)
                job.sleep(min(0.1, dur))
            a.pole_mm = goal
            job.log(f"Pole at {goal:.0f} mm ({self._how(mode)}).")
            return dict(pole_mm=goal)

        return self._start("pole", title, self._then(work, on_done), side, mode)

    def set_hand(self, side: str, counts: list, title: str, mode: str | None = None, on_done=None):
        a = self.arm(side)
        mode = mode or self.mode
        vals = [int(v) for v in counts]
        if len(vals) != 6 or not all(0 <= v <= 1000 for v in vals):
            raise UserError("The hand needs six values from 0 to 1000.")

        def work(job):
            self._same_mode(mode)
            real = self._real(side)
            if real is not None and mode == "real":
                job.progress("Setting the hand (50 % speed)", 1, 1)
                out = real.hand_to(vals)
                a.hand = list(out.get("after") or vals)
                job.log(("Hand in pose" if out.get("arrived") else "Hand settled short of the pose, stopped on force")
                        + " (read back on the controller): " + ", ".join(map(str, a.hand)))
                return dict(hand=a.hand, arrived=bool(out.get("arrived")))
            job.progress("Setting the hand", 1, 1)
            job.sleep(_s(0.8))
            a.hand = vals
            job.log(f"Hand set ({self._how(mode)}): " + ", ".join(map(str, vals)))
            return dict(hand=vals)

        return self._start("hand", title, self._then(work, on_done), side, mode)

    def _how(self, mode: str | None = None) -> str:
        return "read back" if (mode or self.mode) == "real" else "emulated: Hardware sim moves nothing"

    def teach_start(self, side: str, setup: "staging.Stage | None", mode: str | None = None) -> dict:
        """Drag teaching starts only in Real (``mode``: the one the request was made in, still the
        mode now), with the arm in the region's setup (read back), so the recording's stage is
        what the robot really held. In Hardware sim the pole and the hand are only emulated, so a
        recording's stage could not be known."""
        mode = self._teach_mode(mode, setup)
        a = self.arm(side)
        with self.lock:
            self._same_mode(mode)
            self.require_motion(side)
            self._teach_ready(side, setup)
            a.teaching, a.teach_started = True, time.time()
            a.teach_stage = setup
        self._teach_on(side)
        return a.view()

    def teach_countdown(self, side: str, setup: "staging.Stage | None", delay_s: float, mode: str | None = None):
        """Drag teaching after a countdown (the owner, 16 Oct: after START, time to walk to the arm).
        Every check is made at once, so a refusal comes before the countdown; the countdown is a job
        that STOP, the E-stop or a cancel ends, and then nothing is recorded; at its end the checks are
        made again and the arm goes into drag teaching, as ``teach_start``. Returns the job."""
        mode = self._teach_mode(mode, setup)
        a = self.arm(side)
        with self.lock:
            self._same_mode(mode)
            self._teach_ready(side, setup)

        def work(job):
            t_end = time.monotonic() + float(delay_s)
            while (left := t_end - time.monotonic()) > 0:
                job.progress(f"Drag teaching starts in {math.ceil(left)} s: go to the {side.upper()} arm", 0, 1)
                job.sleep(min(0.1, left))
            with self.lock:
                self._same_mode(mode, "drag teaching did not start")
                why = self.motion_block()
                if why:
                    raise UserError(why, "Release the E-stop button, then press Recover.")
                self._teach_ready(side, setup)
                a.teaching, a.teach_started = True, time.time()
                a.teach_stage = setup
            self._teach_on(side)
            job.progress("Drag teaching", 1, 1)
            job.log(f"Drag teaching on the {side.upper()} arm: guide it along the region, then press STOP DRAG.")
            return dict(teaching=True, started=a.teach_started)

        return self._start("teach", f"Drag teaching on the {side.upper()} arm in {float(delay_s):g} s", work, side, mode)

    @staticmethod
    def _teach_mode(mode, setup) -> str:
        if mode != "real":
            raise UserError("Drag teaching uses the real arm, pole and hand, so it needs Real robot.",
                            "In Hardware sim nothing moves: the pole and the hand are only emulated, so the "
                            "recording would not know its true stage. Choose Real robot in the status pill.",
                            code="mode")
        if setup is None:
            raise UserError("This region has no pole height and hand pose to teach with.",
                            "Set them in Prepare first.", code="stage")
        return mode

    def _teach_ready(self, side: str, setup: "staging.Stage") -> None:
        """The arm in the region's setup (read back), so the recording's stage is what the robot held."""
        if not self.in_stage(side, setup):
            pole_mm, hand = self._measured(side)
            raise UserError(f"The {side.upper()} arm is not in this region's setup ({staging.describe(setup)}): "
                            f"it reads pole {'?' if pole_mm is None else f'{pole_mm:.0f}'} mm, hand "
                            f"{'?' if hand is None else ', '.join(map(str, hand))}.",
                            "Press Apply in Prepare for the pole and the hand, then start again.", code="stage")

    def _teach_on(self, side: str) -> None:
        real = self._real(side)
        if real is not None:
            try:
                real.teach_start()                      # the controller records, as the control program has it do
            except Exception:
                self.arm(side).teaching = False
                raise

    def teach_stop(self, side: str, save_to: Path | None = None) -> dict:
        """End drag teaching. A linked arm's controller saves its recording (``file``: written by
        the SDK, as the control program's are); the stand-in has none (``file`` None)."""
        a = self.arm(side)
        if not a.teaching:
            if self.jobs.cancel_arm(side, kinds=("teach",)):    # STOP DRAG during the countdown: it never started
                raise UserError("Drag teaching had not started yet: nothing was recorded.")
            raise UserError("Teaching is not running on this arm.")
        a.teaching = False
        secs = time.time() - (a.teach_started or time.time())
        real = self._real(side)
        if real is None:
            return dict(seconds=secs, file=None, points=None, stage=a.teach_stage)
        import tempfile

        path = Path(save_to) if save_to else Path(tempfile.mkdtemp(prefix="lite-teach-")) / \
            f"{side}_drag_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        points = real.teach_stop(path)
        return dict(seconds=secs, file=path, points=points, stage=a.teach_stage)

    def teach_cancel(self, side: str) -> dict:
        """Drop the recording being taught: nothing is saved; a linked arm leaves drag teaching."""
        a = self.arm(side)
        self.jobs.cancel_arm(side, kinds=("teach",))     # a countdown ends: drag teaching never starts
        was, a.teaching = a.teaching, False
        if was:
            self._leave_drag(side)
        return a.view()

    def move_to(self, side: str, what: str, title: str, seconds: float = 2.5, stage: "staging.Stage | None" = None,
                mode: str | None = None, target_millideg: list | None = None):
        """Move the arm to a pose (a start pose, a file's first line), staging the pole and hand
        first when the pose belongs to a trajectory: a linked arm by ALIX's park move, read back;
        the stand-in only takes the time."""
        a = self.arm(side)
        mode = mode or self.mode
        if side in self.links and target_millideg is None:
            raise UserError("Start poses by name are not stored on the controller yet, so Lite cannot move the arm to one.",
                            "Use Move to Recorded Start, or move the arm with its pendant.")

        def work(job):
            if stage is not None:
                self._stage(job, side, stage, mode)
            self._same_mode(mode, "the arm did not move")
            real = self._real(side)
            if real is not None:                         # the controller moves it (REAL) or simulates it (SIM)
                originals = real.set_run_mode("REAL" if mode == "real" else "SIM")
                try:
                    job.progress(f"Moving to {what} ({self.speed_pct} % speed)", 1, 1)
                    out = real.move_to(target_millideg, speed=self.speed_pct, restore_pct=self.found_speed(side))
                finally:
                    real.restore_run_mode(originals)
                job.log(f"At {what}: {out['gap_deg']:.2f} degrees away at most (read back"
                        + ("" if mode == "real" else "; the controller's SIM mode, the arm did not move") + ").")
                return dict(at=what, gap_deg=out["gap_deg"])
            t0, wall = time.time(), _s(seconds)
            while (el := time.time() - t0) < wall:
                if a.estop != "CLEAR":
                    raise UserError("INTERRUPTED — E-STOP. The arm did not reach the pose.",
                                    "Release the E-stop and follow Recover.")
                job.progress(f"Moving to {what}", int(100 * el / wall), 100)
                job.sleep(min(0.1, wall))
            job.log(f"At {what} (joints read back)." if mode == "real"
                    else f"At {what} (dry run: the controller's SIM mode, the arm did not move).")
            return dict(at=what)

        return self._start("move", title, work, side, mode)

    # ── the arm's pose now (V32's Capture Current Pose: a linked arm only)
    def capture(self, side: str) -> dict:
        """What a linked arm's controller reads now: J1-J7 (deg), the tool pose (X Y Z in m, RX RY
        RZ in rad, the controller's own frames), the pole, the hand, the time and the controller.
        Read afresh while no command runs; the stand-in has no controller to read."""
        a = self.arm(side)
        if not a.connected:
            raise UserError(f"The {side.upper()} arm is not connected.", "Connect it, then capture its pose.", code="connect")
        link = self.links.get(side)
        if link is None:
            raise UserError(f"The stand-in robot has no {side.upper()} controller to read the pose from.",
                            "Start Lite with --robot realman and connect the arm.", code="connect")
        fb = link.read_fresh()                          # never alongside a command (busy: said, with what holds it)
        if not fb.get("joints_deg") or len(fb["joints_deg"]) < 7:
            raise UserError(f"The {side.upper()} controller did not answer with the arm's joints.", "Try again.")
        return dict(arm=side, joints_deg=fb["joints_deg"][:7], pose=fb.get("pose"), pole_mm=fb.get("pole_mm"),
                    hand=fb.get("hand"), errors=fb.get("errors"), run_mode=fb.get("run_mode"),
                    at=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                    controller=dict(side=side, ip=link.ip, port=link.port, emulated=bool(link.emulated)))

    # ── the controller's global points (a linked arm only: they are stored on its controller)
    def global_points(self, side: str) -> list[dict] | None:
        """A linked arm's global points; None for the stand-in, which has no controller."""
        real = self._real(side)
        return None if real is None else real.global_points()

    def save_global_point(self, side: str, name: str, expect_deg: list | None = None, replace: bool = False) -> dict:
        """The arm's pose now as a global point on its controller (a linked arm, standing still)."""
        real = self._real(side)
        if real is None:
            raise UserError(f"Global points are stored on the {side.upper()} arm's controller: the stand-in has none.",
                            "Start Lite with --robot realman and connect the arm.", code="connect")
        moving = [j for j in self.jobs.running(arm=side) if j.kind in CONTROLLER_KINDS]
        if moving or self.arm(side).teaching:
            raise UserError(f"The {side.upper()} arm is busy ({moving[0].title if moving else 'drag teaching'}).",
                            "Save the point once the arm has stopped.")
        from . import content

        try:
            return real.save_global_point(name, expect_deg, replace)
        finally:
            content.forget(side)                        # the controller's lists are read again at their next use

    def move_point(self, side: str, name: str, move: str, speed: int, wait_s: float, title: str, mode: str | None = None):
        """MoveJ or MoveL to a global point of a linked arm: in Real the arm moves, in Hardware sim
        the controller's SIM mode simulates it (read back either way)."""
        mode = mode or self.mode
        if self._real(side) is None:
            raise UserError(f"Global points are stored on the {side.upper()} arm's controller: the stand-in has none.",
                            "Start Lite with --robot realman and connect the arm.", code="connect")
        label = "MoveL" if str(move).lower() == "movel" else "MoveJ"

        def work(job):
            self._same_mode(mode, "the arm did not move")
            real = self._real(side)
            originals = real.set_run_mode("REAL" if mode == "real" else "SIM")
            try:
                job.progress(f"{label} to {name} ({int(speed)} % speed)", 1, 1)
                out = real.move_point(name, move, speed, wait_s, restore_pct=self.found_speed(side))
            finally:
                real.restore_run_mode(originals)
            job.log(f"At global point {name} ({label}" + (f", {out['gap_deg']:.2f} degrees away at most" if out["gap_deg"] is not None else "")
                    + ("; read back)." if mode == "real" else "; the controller's SIM mode, the arm did not move)."))
            return out

        return self._start("move", title, work, side, mode)

    def playback(self, side: str, lines: int, *, dry_run: bool, title: str, stage: "staging.Stage",
                 mode: str | None = None):
        """Stage the pole and hand, move to a recording's start, then replay it: (lines - 1) x 5 ms."""
        a = self.arm(side)
        want = "SIM" if dry_run else "REAL"
        mode = mode or ("hwsim" if dry_run else "real")

        def work(job):
            self._stage(job, side, stage, mode)
            self._same_mode(mode, "the pole and the hand were staged, but the playback did not start")
            a.controller_mode = want
            job.progress("Moving to the recorded start", 0, lines)
            job.sleep(_s(1.5))
            dur = max(int(lines) - 1, 0) * 0.005
            t0, wall = time.time(), _s(dur)
            while (el := time.time() - t0) < wall:
                if a.estop != "CLEAR":
                    raise UserError("INTERRUPTED — E-STOP. The playback did not finish.",
                                    "Release the E-stop and follow Recover.")
                job.progress("Playing back the recording" + (" (dry run: arms stay still)" if dry_run else ""),
                             int(lines * el / wall), lines)
                job.sleep(min(0.1, wall))
            job.log("Playback finished.")
            return dict(seconds=round(dur, 1), dry_run=dry_run)

        return self._start("playback", title, work, side, mode)

    def _run_real(self, job, real, side: str, pid: int, prog: dict, dry_run: bool, on_done) -> dict:
        """A stored program on a linked controller: it must hold the program Lite sent; then its
        SIM or REAL mode (read back), then the control program's own run (rm_upload.run: the move to
        the first point, the program, its error codes); the mode it had is put back after."""
        held = next((x for x in real.programs() if int(x["id"]) == pid), None)
        if held is None or not held.get("name"):        # deleted on the controller (the lab, 17 Oct: program 80)
            raise UserError(f"The {side.upper()} controller no longer holds program {pid}: it was deleted there.",
                            "Upload it again.")
        if not self._same_program(prog, held):
            raise UserError(f"Program {pid} on the {side.upper()} controller is not the one Lite sent "
                            f"(it holds “{held.get('name')}”).", "Upload it again.")
        if not prog.get("first_row"):
            raise UserError(f"Lite has no first line recorded for program {pid}.", "Upload it again.")
        want = "SIM" if dry_run else "REAL"
        how = "Hardware sim" if dry_run else "Real"
        reported = []                                   # what the arm reported during the run, said as it comes

        def line(ln):
            s = ln.strip()
            if s.startswith(("running", "run slot", "RUN", "!!", "**")):
                job.log(s)
            if s.startswith("running"):                 # the controller has started it (no count: it says no more)
                job.progress(f"Program {pid} running on the {side.upper()} controller"
                             + (" (its SIM mode: the arm stays still)" if dry_run else ""))
            rep = arm_report(s)
            if rep:                                     # a pop-up at once (the owner, 17 Oct), not only after the run
                reported.append(rep)
                job.set(reported=list(reported))
                self._alert("warn" if rep["warn"] else "error", side,
                            f"{side.upper()} {how} run of program {pid}: the arm reports {rep['codes']} at {rep['at_s']:.1f} s.",
                            "In Hardware sim the run goes on to its end; its report is the controller's own." if dry_run
                            else "Check the arm; a code that stops the program ends the run.", "run")

        originals = real.set_run_mode(want)
        try:
            self.arm(side).controller_mode = want
            job.log(f"Controller mode {want} (read back).")
            job.progress(f"Moving to the start ({self.speed_pct} %), then running program {pid}"
                         + (" (the controller's SIM mode: the arm stays still)" if dry_run else ""))
            out = real.run(pid, prog["first_row"], prog["lines"], on_line=line,       # its own lines, as they come
                           speed=self.speed_pct, restore_pct=self.found_speed(side))
        finally:
            real.restore_run_mode(originals)
        job.check()                                     # STOP or an E-stop ended it
        lines = [str(x).strip() for x in out.get("lines") or []]
        codes = "; ".join(f"{r['codes']} at {r['at_s']:.1f} s" for r in reported)
        said = f" The arm reported {codes}." if reported else ""
        short = ended_early(out["seconds"], prog.get("lines"), self.emulator_scale(side))
        if dry_run:
            # The Hardware sim test (the owner, 17 Oct): it passes once the program has run to its end in the
            # controller's SIM mode, whatever the arm reported on the way (said here and in the pop-ups). Its
            # length is the judge: the control program says "RUN COMPLETE" of a program the controller cut short
            # (the lab, 17 Oct: 4105 counted a warning, program 50 over after 3.8 s of its 10 s).
            started = any(x.startswith("running") for x in lines)
            cut = next((x.strip(" *") for x in lines if "did not start within" in x or "still running after" in x), None)
            if not started or cut or short:
                why = cut or (f"it ended {short}" if short else "it did not start")
                raise UserError(f"The Hardware sim run of program {pid} did not run to its end: {why}.{said}",
                                "Look at what the arm reported; a file faster than the arm allows is tuned slower in Tune.")
        else:
            if not out["ok"]:
                raise UserError(f"The run did not complete: {out['why'] or 'see the hardware log'}.{said}",
                                "Check the arm and the controller's errors, then run it again.")
            if short:                                   # the controller stopped it
                cause = (f"the controller stopped it after the arm reported {codes}" if reported
                         else "it was stopped on the controller (an E-stop, or its pendant)")
                raise UserError(f"Program {pid} ended {short}: {cause}, not completed.",
                                "A file faster than the arm allows is tuned slower in Tune (its L2 and L3 limits say so)."
                                if reported else "Check the arm and its E-stop, then run it again.")
        job.log(f"Finished: the controller completed program {pid} ({out['seconds']:.1f} s with the move to the start)."
                + (f" The arm reported {codes} on the way." if reported else ""))
        res = dict(program_id=pid, dry_run=dry_run, seconds=out["seconds"], reported=reported)
        if on_done:
            on_done(res)
        return res

    # ── programs
    def programs(self, side: str) -> list[dict]:
        """Programs 1-100: a linked arm's as its controller lists them (with Lite's record of the
        ones it sent), else the stand-in's."""
        a = self.arm(side)
        real = self._real(side)
        if real is None:
            return [dict(id=i, **a.programs[i]) if i in a.programs else dict(id=i, name=None) for i in range(1, 101)]
        held = {int(x["id"]): x for x in real.programs()}
        out = []
        for i in range(1, 101):
            x, mine = held.get(i), a.programs.get(i)
            ours = bool(x and mine and self._same_program(mine, x))
            out.append(dict(id=i, name=(x or {}).get("name"), bytes=(x or {}).get("size"),
                            **({k: mine[k] for k in ("sha256", "stage", "lines", "uploaded")} if ours else {})))
        return out

    def controller_data(self, side: str, files: dict | None = None) -> dict:
        """Data Management (the owner, 16 Oct: the controller's stored data as RealMan's own web page lists
        it, per arm, display only): a linked arm's programs (Graphical Program), global points and tracks,
        with Lite's record of each program it sent (when; which project file, by its bytes, from ``files``:
        {sha256: the project's file}); the stand-in's programs, as Lite sent them. Read only, and never
        while the arm moves or teaches (the link reads nothing then)."""
        from .content import file_for

        a = self.arm(side)
        real = self._real(side)
        out = dict(side=side, connected=a.connected, linked=real is not None, read=time.strftime("%Y-%m-%d %H:%M:%S"), ip=None)

        def lite(mine):
            f = file_for(files, mine.get("sha256"), mine.get("name"))
            return dict(sent=mine.get("uploaded"), file=mine.get("name"), lines=mine.get("lines"),
                        project_file=f.get("name"), region=f.get("region"))

        if real is None:
            progs = [dict(num=i, name=r.get("stored_name") or r.get("name"), bytes=r.get("bytes"), speed=None, lite=lite(r))
                     for i, r in sorted(a.programs.items())]
            return dict(out, programs=progs, default_program=None, points=None,
                        tracks=dict(available=False, list=[], why="The stand-in robot has no controller."),
                        note="The stand-in robot has no controller: the programs Lite sent to it are listed. Start Lite with "
                             "--robot realman and connect the arm to read its controller.")
        if not a.connected:
            raise UserError(f"Connect the {side.upper()} arm to read its controller's data.", code="connect")
        moving = [j for j in self.jobs.running(arm=side) if j.kind in CONTROLLER_KINDS]
        if moving or a.teaching:
            raise UserError(f"The {side.upper()} arm is busy ({moving[0].title if moving else 'drag teaching'}).",
                            "Its controller's data is read once it has stopped.", code="busy")
        d = real.controller_data()
        progs = []
        for x in d["programs"]:
            mine = a.programs.get(int(x["num"]))
            ours = bool(mine) and self._same_program(mine, dict(size=x["bytes"]))
            progs.append(dict(x, lite=lite(mine) if ours else None))
        return dict(out, ip=d["ip"], programs=progs, default_program=d["default_program"], points=d["points"], tracks=d["tracks"])

    @staticmethod
    def _same_program(mine: dict, held: dict) -> bool:
        """Whether the controller's program is the one Lite sent (its stored size, as read back at upload)."""
        sent = mine.get("controller_bytes")
        size = int(held.get("size") or 0)
        return sent is not None and abs(size - int(sent)) <= max(64, int(sent) // 1000)

    def upload(self, side: str, program_id: int, path: Path, name: str, sha256: str, replace: bool, title: str,
               stage: dict | None = None):
        """Store a file as program ``program_id``, with the stage it must run with (so a run by
        number can stage it too). Never while the arm moves or another upload runs on it: a run (S2's
        included) would go on with a program it was not checked with, and two commands would share the
        arm's connection. One upload is checked and started at a time (a double click sends once)."""
        with self._send_lock:
            return self._upload(side, program_id, path, name, sha256, replace, title, stage)

    def _upload(self, side, program_id, path, name, sha256, replace, title, stage):
        a = self.arm(side)
        if not a.connected:
            raise UserError(f"The {side.upper()} arm is not connected.", "Connect it from the bar at the top.")
        moving = [j for j in self.jobs.running(arm=side) if j.kind in CONTROLLER_KINDS]
        if moving:
            raise UserError(f"The {side.upper()} arm is busy: {moving[0].title}.", "Send the file once it has ended.",
                            code="busy")
        pid = int(program_id)
        if not 1 <= pid <= 100:
            raise UserError("Program numbers are 1 to 100.")
        cur = a.programs.get(pid)
        real = self._real(side)
        held = None
        if real is not None:                            # what the controller really holds in that slot
            held = next((x for x in real.programs() if int(x["id"]) == pid), None)
        ours = cur is not None and (real is None or (held is not None and self._same_program(cur, held)))
        if ours and cur.get("sha256") == sha256:
            if stage is not None and cur.get("stage") != stage:   # the same bytes, kept again with another stage
                cur["stage"] = stage
                a.save()
            return None                                       # unchanged: nothing to send
        if real is not None:                            # the controller's slot, whoever filled it
            holder = held.get("name") if held else None
        else:
            holder = cur.get("name") if cur else None
        if holder and not replace:
            raise UserError(f"Program {pid} on the {side.upper()} controller already holds “{holder}”.",
                            "Confirm to replace it, or choose another Program ID.", code="replace")
        size = Path(path).stat().st_size
        lines = sum(1 for _ in open(path, "rb"))
        with open(path, "rb") as f:
            first = json.loads(f.readline())["point"]

        if real is not None:
            def work(job):
                # The SDK sends the file in one call and says nothing until it is done: the bar
                # moves by the expected time (said to be estimated) and fills at the read-back.
                what = f"Sending {size / 1e3:.0f} kB to program {pid} (estimated)"
                expect, t0, done = max(1.0, size / REAL_UPLOAD_BYTES_S), time.time(), threading.Event()

                def tick():
                    while not done.wait(0.25):
                        try:
                            job.progress(what, min(95, int(100 * (time.time() - t0) / expect)), 100)
                        except Exception:                                           # noqa: BLE001 - stopped
                            return

                job.progress(what, 0, 100)
                threading.Thread(target=tick, name="lite-upload-progress", daemon=True).start()
                try:
                    out = real.upload(Path(path), pid, name)
                finally:
                    done.set()
                job.progress("Stored: the name and the size read back", 100, 100)
                a.programs[pid] = dict(name=name, lines=lines, bytes=size, sha256=sha256, stage=stage, first_row=first,
                                       controller_bytes=out["bytes"], stored_name=out["stored_name"],
                                       uploaded=time.strftime("%Y-%m-%d %H:%M"))
                a.save()
                from . import content

                content.forget(side)                    # the controller's lists are read again at their next use
                job.log(f"Stored as program {pid} on the {side.upper()} controller as “{out['stored_name']}”; "
                        f"{out['bytes']} bytes read back.")
                return dict(program_id=pid, lines=lines, bytes=size, seconds=out["seconds"])

            return self.jobs.start("upload", title, work, arm=side)

        def work(job):
            dur = max(1.0, size / UPLOAD_BYTES_S)
            t0, wall = time.time(), _s(dur)
            while (el := time.time() - t0) < wall:
                job.progress(f"Sending {size / 1e3:.0f} kB to program {pid}", int(100 * el / wall), 100)
                job.sleep(min(0.2, wall))
            a.programs[pid] = dict(name=name, lines=lines, bytes=size, sha256=sha256, stage=stage, first_row=first,
                                   uploaded=time.strftime("%Y-%m-%d %H:%M"))
            a.save()
            job.log(f"Stored as program {pid}; name and size read back.")
            return dict(program_id=pid, lines=lines, bytes=size, seconds=round(dur, 1))

        return self.jobs.start("upload", title, work, arm=side)

    def run(self, side: str, program_id: int, *, dry_run: bool, title: str, on_done=None,
            stage: "staging.Stage | None" = None, mode: str | None = None, alone: bool = False):
        """Run a stored program: the pole and the hand are staged first (read back in Real,
        emulated in Hardware sim), and the program never starts when either fails, nor without a
        stage at all, nor once the mode is no longer ``mode`` (by default the one ``dry_run`` was
        decided in: Hardware sim for a dry run, else Real)."""
        a = self.arm(side)
        prog = a.programs.get(int(program_id))
        if not prog:
            raise UserError(f"Program {program_id} is empty on the {side.upper()} arm.", "Send the file to the robot first.")
        if stage is None:
            raise UserError(f"Program {program_id} has no recorded pole height and hand pose, so it cannot run.",
                            "Send it again from its region: Lite records the stage with the upload.", code="stage")
        want = "SIM" if dry_run else "REAL"
        mode = mode or ("hwsim" if dry_run else "real")

        real = self._real(side)

        def work(job):
            self._stage(job, side, stage, mode)
            self._same_mode(mode, "the pole and the hand were staged, but the program did not start")
            if real is not None:
                return self._run_real(job, real, side, int(program_id), prog, dry_run, on_done)
            a.controller_mode = want
            job.log(f"Controller mode {want} (read back).")
            job.progress("Moving to the start pose", 0, prog["lines"])
            job.sleep(_s(1.5))
            dur = max(prog["lines"] - 1, 0) * 0.005
            t0, wall = time.time(), _s(dur)
            while (el := time.time() - t0) < wall:
                if a.estop != "CLEAR":
                    raise UserError("INTERRUPTED — E-STOP. The run did not finish.",
                                    "Release the E-stop and follow Recover. Nothing restarts by itself.")
                job.progress(f"Running program {program_id}" + (" (dry run: arms stay still)" if dry_run else ""),
                             int(prog["lines"] * el / wall), prog["lines"])
                job.sleep(min(0.1, wall))
            job.log("Finished: the controller reports the program complete.")
            out = dict(program_id=int(program_id), dry_run=dry_run, seconds=round(dur, 1))
            if on_done:
                on_done(out)
            return out

        return self._start("run", title, work, side, mode, alone)
