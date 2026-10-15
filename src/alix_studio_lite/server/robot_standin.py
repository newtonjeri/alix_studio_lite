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
MOTION_KINDS = ("run", "prepare", "teach", "move", "playback", "pole", "hand", "stage")   # one at a time per arm
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
                    teach_s=round(time.time() - self.teach_started, 1) if self.teaching else None)


def emulated_stage(job, side: str, a, stage: "staging.Stage", faults=(), *, emulated: bool = True,
                   step: int = 1, steps: int = 1) -> None:
    """The pole to the stage height, then the hand to the stage pose, on ALIX's measured times,
    on arm state ``a`` (pole_mm, hand, estop). ``emulated``: Hardware sim, which sends nothing
    (D30, R4 (b)); otherwise the stand-in acting as the real arm. Raises, and nothing else runs,
    when either does not arrive (``faults``: an engineer's test) or the E-stop is pressed."""
    how = "emulated: Hardware sim moves nothing" if emulated else "read back"
    start, goal = a.pole_mm, float(stage.pole_mm)
    dur = staging.pole_travel_s(start, goal)
    end = start + (0.6 if "pole" in faults else 1.0) * (goal - start)
    t0 = time.time()
    while dur and (el := time.time() - t0) < dur:
        if a.estop != "CLEAR":
            raise UserError("INTERRUPTED — E-STOP while the pole was moving. The arm did not move.",
                            "Release the E-stop and follow Recover.")
        a.pole_mm = start + min(1.0, el / dur) * (end - start)
        job.progress(f"Pole {start:.0f} → {goal:g} mm" + (" (emulated)" if emulated else ""), step, steps)
        job.sleep(0.1)
    a.pole_mm = end if dur else a.pole_mm
    if abs(a.pole_mm - goal) > staging.POLE_TOL_MM:
        raise UserError(f"The {side.upper()} pole did not reach {goal:g} mm ({how.split(':')[0]} {a.pole_mm:.0f} mm). "
                        "Nothing else ran.", "Check the pole and try again.")
    job.log(f"Pole at {a.pole_mm:.0f} mm ({how}).")
    target = [h if t is None else int(t) for h, t in zip(a.hand, stage.hand)]
    dur = staging.hand_move_s(a.hand, target)
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
        self.link_kind = link
        self.links: dict = {}                           # side -> robot_link.ArmLink while connected
        self.test_estop: dict = {}                      # an engineer's E-stop test on emulated arms
        self.lock = threading.RLock()
        self.arms = {s: StandInArm(s, Path(data_dir) / "standin" / f"{s}_programs.json") for s in ARMS}
        self.faults = {s: set() for s in ARMS}         # engineer test: {"pole", "hand"} fail to arrive

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

    def _leave_drag(self, side: str) -> None:
        """A linked arm in drag teaching leaves it (STOP, an E-stop, a cancel, Lite closing)."""
        real = self._real(side)
        if real is not None:
            real.teach_cancel()

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
                if arm.teaching and arm.side in self.links:
                    threading.Thread(target=self._leave_drag, args=(arm.side,), daemon=True).start()
                arm.teaching = False
            for j in self.jobs.running():
                if j.kind in MOTION_KINDS:
                    j.log(f"E-stop pressed on the {side.upper()} arm.", "error")
                    j.cancel(*ESTOP_STOP)

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
        stale = (age is None or age > STALE_S) and not fb.get("busy")      # a command owns the connection
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
        teaching = [a.side for a in self.arms.values() if a.teaching]
        if teaching:
            raise UserError(f"Drag teaching is running on the {teaching[0].upper()} arm.",
                            "Stop the teaching first: nothing else moves while an arm is being guided by hand.")
        busy = [j for s in sides for j in self.jobs.running(arm=s) if j.kind in MOTION_KINDS]
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
        """Let go of every live link (the server is stopping); an arm in drag teaching leaves it first."""
        for side, a in self.arms.items():
            if a.teaching:
                a.teaching = False
                self._leave_drag(side)
        with self.lock:
            links, self.links = list(self.links.values()), {}
        for link in links:
            link.close()

    # ── connection and safety
    def connect(self, side: str, ip: str | None = None) -> dict:
        a = self.arm(side)
        target = (ip or a.ip).strip()
        if self.link_kind:
            from .robot_link import ArmLink

            # the new link first: when it cannot open, the arm keeps its old link (or stays
            # disconnected), never "connected" without a reading behind its E-stop chip
            new = ArmLink(side, target, on_read=lambda: self._sync(side)).open()
            with self.lock:
                old = self.links.get(side)
                self.links[side] = new
                a.ip, a.connected = target, True
            if old is not None:
                old.close()
            self._sync(side)
            return self.view()[side]
        time.sleep(0.4)
        with self.lock:
            a.ip, a.connected = target, True
            a.estop = "ENGAGED" if a.button_pressed else "RECOVERING" if a.estop_latched else "CLEAR"
        return a.view()

    def disconnect(self, side: str) -> dict:
        a = self.arm(side)
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
        sent: dict = {}
        teaching = {s for s in links if self.arms[s].teaching}

        def one(s, link):
            if s in teaching:
                self._leave_drag(s)                     # drag teaching ends first, then the stop
            sent[s] = link.stop(hard=every)

        threads = [threading.Thread(target=one, args=(s, l), name=f"lite-stop-{s}", daemon=True) for s, l in links.items()]
        for t in threads:
            t.start()
        with self.lock:
            n = self.jobs.cancel_arm("all" if every else side)
            held = []
            for s in sides:
                a = self.arms[s]
                a.teaching = False
                if a.connected:
                    held_by_button = a.estop == "ENGAGED" or (a.estop == "RECOVERING" and not a.stop_latched)
                    a.estop_latched = True
                    a.stop_latched = a.stop_latched or not held_by_button   # an E-stop press keeps its own words
                    if a.estop == "CLEAR":
                        a.estop = "RECOVERING"
                    held.append(s)
        for t in threads:
            t.join(timeout=5.0)
        for s in links:
            sent.setdefault(s, dict(ok=False, text=f"{s.upper()}: no answer from its controller within 5 s"))
        ok = all(r["ok"] for r in sent.values())
        for s, r in sent.items():
            if not r["ok"]:
                self._alert("error", s, r["text"] + ".", "Press the red E-stop button on the robot if it is still moving.", "stop")
        names = " and ".join(x.upper() for x in held)
        text = " ".join(sent[s]["text"] + "." for s in ARMS if s in sent) + (
            f" Nothing moves until you press Recover on the {names} arm{'s' if len(held) > 1 else ''} (Robot & mode)."
            if held else "")
        return dict(stopped=n, side="all" if every else side, controllers=sent, ok=ok, held=held, text=text.strip())

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

    def stage(self, side: str, stage: "staging.Stage", title: str, mode: str | None = None):
        """Stage the pole and the hand only (Prepare's Apply)."""
        mode = mode or self.mode

        def work(job):
            self._stage(job, side, stage, mode)
            return dict(pole_mm=self.arm(side).pole_mm, hand=self.arm(side).hand)

        return self._start("stage", title, work, side, mode)

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
    def move_pole(self, side: str, pole_mm: float, title: str, mode: str | None = None):
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
            dur = max(0.5, abs(goal - start) / POLE_MM_S)
            t0 = time.time()
            while (f := min(1.0, (time.time() - t0) / dur)) < 1.0:
                a.pole_mm = start + f * (goal - start)
                job.progress(f"Moving the pole to {goal:.0f} mm", 1, 1)
                job.sleep(0.1)
            a.pole_mm = goal
            job.log(f"Pole at {goal:.0f} mm ({self._how(mode)}).")
            return dict(pole_mm=goal)

        return self._start("pole", title, work, side, mode)

    def set_hand(self, side: str, counts: list, title: str, mode: str | None = None):
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
            job.sleep(0.8)
            a.hand = vals
            job.log(f"Hand set ({self._how(mode)}): " + ", ".join(map(str, vals)))
            return dict(hand=vals)

        return self._start("hand", title, work, side, mode)

    def _how(self, mode: str | None = None) -> str:
        return "read back" if (mode or self.mode) == "real" else "emulated: Hardware sim moves nothing"

    def teach_start(self, side: str, setup: "staging.Stage | None", mode: str | None = None) -> dict:
        """Drag teaching starts only in Real (``mode``: the one the request was made in, still the
        mode now), with the arm in the region's setup (read back), so the recording's stage is
        what the robot really held. In Hardware sim the pole and the hand are only emulated, so a
        recording's stage could not be known."""
        mode = mode or self.mode
        if mode != "real":
            raise UserError("Drag teaching uses the real arm, pole and hand, so it needs Real robot.",
                            "In Hardware sim nothing moves: the pole and the hand are only emulated, so the "
                            "recording would not know its true stage. Choose Real robot in the status pill.",
                            code="mode")
        a = self.arm(side)
        if setup is None:
            raise UserError("This region has no pole height and hand pose to teach with.",
                            "Set them in Prepare first.", code="stage")
        with self.lock:
            self._same_mode(mode)
            self.require_motion(side)
            if not self.in_stage(side, setup):
                pole_mm, hand = self._measured(side)
                raise UserError(f"The {side.upper()} arm is not in this region's setup ({staging.describe(setup)}): "
                                f"it reads pole {'?' if pole_mm is None else f'{pole_mm:.0f}'} mm, hand "
                                f"{'?' if hand is None else ', '.join(map(str, hand))}.",
                                "Press Apply in Prepare for the pole and the hand, then start again.", code="stage")
            a.teaching, a.teach_started = True, time.time()
            a.teach_stage = setup
        real = self._real(side)
        if real is not None:
            try:
                real.teach_start()                      # the controller records, as the control program has it do
            except UserError:
                a.teaching = False
                raise
        return a.view()

    def teach_stop(self, side: str, save_to: Path | None = None) -> dict:
        """End drag teaching. A linked arm's controller saves its recording (``file``: written by
        the SDK, as the control program's are); the stand-in has none (``file`` None)."""
        a = self.arm(side)
        if not a.teaching:
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
        was, a.teaching = a.teaching, False
        if was:
            self._leave_drag(side)
        return a.view()

    def move_to(self, side: str, what: str, title: str, seconds: float = 2.5, stage: "staging.Stage | None" = None,
                mode: str | None = None, target_millideg: list | None = None):
        """Move the arm to a pose (a start pose, a file's first line), staging the pole and hand
        first when the pose belongs to a trajectory. The real backend sweeps the joint-space line
        for collisions first (R4); the stand-in only takes the time."""
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
                    job.progress(f"Moving to {what} (30 % speed)", 1, 1)
                    out = real.move_to(target_millideg)
                finally:
                    real.restore_run_mode(originals)
                job.log(f"At {what}: {out['gap_deg']:.2f} degrees away at most (read back"
                        + ("" if mode == "real" else "; the controller's SIM mode, the arm did not move") + ").")
                return dict(at=what, gap_deg=out["gap_deg"])
            t0 = time.time()
            while (el := time.time() - t0) < seconds:
                if a.estop != "CLEAR":
                    raise UserError("INTERRUPTED — E-STOP. The arm did not reach the pose.",
                                    "Release the E-stop and follow Recover.")
                job.progress(f"Moving to {what}", int(100 * el / seconds), 100)
                job.sleep(0.1)
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
        if link.view()["feedback"].get("busy"):
            raise UserError(f"The {side.upper()} controller is busy ({link.view()['feedback']['busy']}).",
                            "Capture the pose when the arm has stopped.")
        fb = link.read_once()
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
        moving = [j for j in self.jobs.running(arm=side) if j.kind in MOTION_KINDS]
        if moving or self.arm(side).teaching:
            raise UserError(f"The {side.upper()} arm is busy ({moving[0].title if moving else 'drag teaching'}).",
                            "Save the point once the arm has stopped.")
        return real.save_global_point(name, expect_deg, replace)

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
                out = real.move_point(name, move, speed, wait_s)
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
            job.sleep(1.5)
            dur = max(int(lines) - 1, 0) * 0.005
            t0 = time.time()
            while (el := time.time() - t0) < dur:
                if a.estop != "CLEAR":
                    raise UserError("INTERRUPTED — E-STOP. The playback did not finish.",
                                    "Release the E-stop and follow Recover.")
                job.progress("Playing back the recording" + (" (dry run: arms stay still)" if dry_run else ""),
                             int(lines * el / dur), lines)
                job.sleep(0.1)
            job.log("Playback finished.")
            return dict(seconds=round(dur, 1), dry_run=dry_run)

        return self._start("playback", title, work, side, mode)

    def _run_real(self, job, real, side: str, pid: int, prog: dict, dry_run: bool, on_done) -> dict:
        """A stored program on a linked controller: it must hold the program Lite sent; then its
        SIM or REAL mode (read back), then the control program's own run (rm_upload.run: the move to
        the first point, the program, its error codes); the mode it had is put back after."""
        held = next((x for x in real.programs() if int(x["id"]) == pid), None)
        if held is None or not self._same_program(prog, held):
            raise UserError(f"Program {pid} on the {side.upper()} controller is not the one Lite sent "
                            f"(it holds “{(held or {}).get('name')}”).", "Upload it again.")
        if not prog.get("first_row"):
            raise UserError(f"Lite has no first line recorded for program {pid}.", "Upload it again.")
        want = "SIM" if dry_run else "REAL"
        originals = real.set_run_mode(want)
        try:
            self.arm(side).controller_mode = want
            job.log(f"Controller mode {want} (read back).")
            job.progress(f"Moving to the start (20 %), then running program {pid}"
                         + (" (the controller's SIM mode: the arm stays still)" if dry_run else ""), 0, prog["lines"])
            out = real.run(pid, prog["first_row"], prog["lines"],      # its own lines, as they come
                           on_line=lambda ln: job.log(ln.strip()) if ln.strip().startswith(("running", "RUN", "!!", "**")) else None)
        finally:
            real.restore_run_mode(originals)
        job.check()                                     # STOP or an E-stop ended it
        if not out["ok"]:
            raise UserError(f"The run did not complete: {out['why'] or 'see the hardware log'}.",
                            "Check the arm and the controller's errors, then run it again.")
        job.log(f"Finished: the controller completed program {pid} ({out['seconds']:.1f} s with the move to the start).")
        res = dict(program_id=pid, dry_run=dry_run, seconds=out["seconds"])
        if on_done:
            on_done(res)
        return res

    def run_foreign(self, side: str, program_id: int, *, stage: "staging.Stage", mode: str, title: str):
        """A program on a linked controller that Lite did not send (feedback v2, slide 11): its pole and
        hand as the operator states them, staged first; then the control program's own run, which
        lets the controller move the arm to the program's first point in its own joint move. On this
        arm alone: the other arm stays stopped."""
        real = self._real(side)
        if real is None:
            raise UserError("The stand-in holds only the programs Lite sent.", code="connect")
        pid, dry = int(program_id), mode != "real"

        def work(job):
            self._stage(job, side, stage, mode)
            self._same_mode(mode, "the pole and the hand were staged, but the program did not start")
            want = "SIM" if dry else "REAL"
            originals = real.set_run_mode(want)
            try:
                self.arm(side).controller_mode = want
                job.log(f"Controller mode {want} (read back).")
                job.progress(f"Running program {pid}: the controller first moves the arm to its first point"
                             + (" (the controller's SIM mode: the arm stays still)" if dry else ""), 0, 1)
                out = real.run(pid, None, None,
                               on_line=lambda ln: job.log(ln.strip()) if ln.strip().startswith(("running", "RUN", "!!", "**")) else None)
            finally:
                real.restore_run_mode(originals)
            job.check()
            if not out["ok"]:
                raise UserError(f"Program {pid} did not complete: {out['why'] or 'see the hardware log'}.",
                                "Check the arm and the controller's errors.")
            job.log(f"Finished: the controller completed program {pid} ({out['seconds']:.1f} s).")
            return dict(program_id=pid, dry_run=dry, seconds=out["seconds"])

        return self._start("run", title, work, side, mode, alone=True)

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

    @staticmethod
    def _same_program(mine: dict, held: dict) -> bool:
        """Whether the controller's program is the one Lite sent (its stored size, as read back at upload)."""
        sent = mine.get("controller_bytes")
        size = int(held.get("size") or 0)
        return sent is not None and abs(size - int(sent)) <= max(64, int(sent) // 1000)

    def upload(self, side: str, program_id: int, path: Path, name: str, sha256: str, replace: bool, title: str,
               stage: dict | None = None):
        """Store a file as program ``program_id``, with the stage it must run with (so a run by
        number can stage it too)."""
        a = self.arm(side)
        if not a.connected:
            raise UserError(f"The {side.upper()} arm is not connected.", "Connect it from the bar at the top.")
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
                job.log(f"Stored as program {pid} on the {side.upper()} controller as “{out['stored_name']}”; "
                        f"{out['bytes']} bytes read back.")
                return dict(program_id=pid, lines=lines, bytes=size, seconds=out["seconds"])

            return self.jobs.start("upload", title, work, arm=side)

        def work(job):
            dur = max(1.0, size / UPLOAD_BYTES_S)
            t0 = time.time()
            while (el := time.time() - t0) < dur:
                job.progress(f"Sending {size / 1e3:.0f} kB to program {pid}", int(100 * el / dur), 100)
                job.sleep(0.2)
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
            job.sleep(1.5)
            dur = max(prog["lines"] - 1, 0) * 0.005
            t0 = time.time()
            while (el := time.time() - t0) < dur:
                if a.estop != "CLEAR":
                    raise UserError("INTERRUPTED — E-STOP. The run did not finish.",
                                    "Release the E-stop and follow Recover. Nothing restarts by itself.")
                job.progress(f"Running program {program_id}" + (" (dry run: arms stay still)" if dry_run else ""),
                             int(prog["lines"] * el / dur), prog["lines"])
                job.sleep(0.1)
            job.log("Finished: the controller reports the program complete.")
            out = dict(program_id=int(program_id), dry_run=dry_run, seconds=round(dur, 1))
            if on_done:
                on_done(out)
            return out

        return self._start("run", title, work, side, mode, alone)
