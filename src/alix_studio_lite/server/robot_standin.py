"""A stand-in for the two arms, so every robot screen works before the robot I/O backend exists
(decision D27: UI/UX first). Nothing here talks to hardware, and every reply says so.

It follows the real rules the backend must keep:

- connect reads the E-stop input (IO3, mode 5) on each arm;
- while either arm's E-stop is not CLEAR, motion is refused on BOTH arms; uploads are allowed;
- STOP is never refused and never queued;
- a run of N lines takes (N - 1) x 5 ms; an E-stop during a run ends it as INTERRUPTED, never complete;
- program IDs 1-100; replacing a stored program needs the caller's confirmation;
- every run, playback and move to a start stages the pole and then the hand first, each read back
  (pole within 5 mm, every set hand channel within 30 counts); when either does not arrive, the job
  ends there and the arm never moves (``staging.py``). Times follow ALIX's measured pole and hand
  models. An engineer can make the pole or the hand fail on purpose to test this.
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
POLE_MM_S = 25.0
MOTION_KINDS = ("run", "prepare", "teach", "move", "playback", "pole", "hand", "stage")   # one at a time per arm
ESTOP_TEXT = {
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
        return dict(side=self.side, ip=self.ip, connected=self.connected, estop=self.estop,
                    estop_text=ESTOP_TEXT[self.estop], estop_io="IO3", controller_mode=self.controller_mode,
                    pole_mm=round(self.pole_mm), hand=list(self.hand), teaching=self.teaching,
                    teach_s=round(time.time() - self.teach_started, 1) if self.teaching else None)


class StandInRobot:
    backend = "stand-in"
    label = "Stand-in robot (no hardware)"

    def __init__(self, data_dir: Path, jobs: Jobs):
        self.jobs = jobs
        self.lock = threading.RLock()
        self.arms = {s: StandInArm(s, Path(data_dir) / "standin" / f"{s}_programs.json") for s in ARMS}
        self.faults = {s: set() for s in ARMS}         # engineer test: {"pole", "hand"} fail to arrive

    # ── state
    def view(self) -> dict:
        return {s: a.view() | {"faults": sorted(self.faults[s])} for s, a in self.arms.items()}

    def arm(self, side: str) -> StandInArm:
        if side not in self.arms:
            raise UserError(f"Unknown arm {side!r}.")
        return self.arms[side]

    def motion_block(self) -> str | None:
        """Why motion is refused right now, in plain words (None = allowed)."""
        for s, a in self.arms.items():
            if a.connected and a.estop != "CLEAR":
                return (f"The {s.upper()} arm's E-stop is {ESTOP_TEXT[a.estop].lower()}. "
                        "Robot motion is blocked on both arms until it is released and recovered.")
        return None

    def require_motion(self, side: str) -> None:
        sides = ARMS if side == "both" else (side,)
        for s in sides:
            if not self.arm(s).connected:
                raise UserError(f"The {s.upper()} arm is not connected.", "Connect it from the bar at the top.")
        why = self.motion_block()
        if why:
            raise UserError(why, "Release the E-stop button, then press Recover.")
        busy = [j for s in sides for j in self.jobs.running(arm=s) if j.kind in MOTION_KINDS]
        if busy:
            raise UserError(f"The {busy[0].arm.upper()} arm is busy: {busy[0].title}.", "Wait, or press STOP.")

    # ── connection and safety
    def connect(self, side: str, ip: str | None = None) -> dict:
        a = self.arm(side)
        if ip:
            a.ip = ip
        time.sleep(0.4)
        a.connected, a.estop = True, "CLEAR"
        return a.view()

    def disconnect(self, side: str) -> dict:
        self.jobs.cancel_arm(side)
        a = self.arm(side)
        a.connected, a.estop, a.teaching = False, "NOT_CONNECTED", False
        return a.view()

    def set_controller_mode(self, side: str, mode: str) -> dict:
        if mode not in ("SIM", "REAL"):
            raise UserError(f"Unknown controller mode {mode!r}.")
        a = self.arm(side)
        if not a.connected:
            raise UserError(f"The {side.upper()} arm is not connected.")
        a.controller_mode = mode
        return a.view()

    def press_estop(self, side: str, pressed: bool) -> dict:
        """Engineer test only: act as if the IO3 button were pressed or released."""
        a = self.arm(side)
        if not a.connected:
            raise UserError(f"The {side.upper()} arm is not connected.")
        if pressed:
            a.estop = "ENGAGED"
            a.teaching = False
            for j in self.jobs.running():
                if j.kind in MOTION_KINDS:
                    j.log(f"E-stop pressed on the {side.upper()} arm.", "error")
                    j.cancel()
        elif a.estop == "ENGAGED":
            a.estop = "RECOVERING"
        return a.view()

    def recover(self, side: str) -> dict:
        a = self.arm(side)
        if a.estop == "ENGAGED":
            raise UserError("The E-stop button is still pressed.", "Make the cell safe, then release the button.")
        a.estop = "CLEAR"
        return a.view()

    def stop(self, side: str) -> dict:
        n = self.jobs.cancel_arm(side)
        for s in (ARMS if side == "all" else (side,)):
            self.arm(s).teaching = False
        return dict(stopped=n, side=side)

    # ── staging: pole, then hand, each read back; nothing moves the arm unless both arrived
    def set_fault(self, side: str, pole: bool | None = None, hand: bool | None = None) -> dict:
        f = self.faults[self.arm(side).side]
        for name, on in (("pole", pole), ("hand", hand)):
            if on is True:
                f.add(name)
            elif on is False:
                f.discard(name)
        return dict(side=side, faults=sorted(f))

    def _stage(self, job, side: str, stage: "staging.Stage", step: int = 1, steps: int = 1) -> None:
        a = self.arm(side)
        start, goal = a.pole_mm, float(stage.pole_mm)
        dur = staging.pole_travel_s(start, goal)
        end = start + (0.6 if "pole" in self.faults[side] else 1.0) * (goal - start)
        t0 = time.time()
        while dur and (el := time.time() - t0) < dur:
            if a.estop != "CLEAR":
                raise UserError("INTERRUPTED — E-STOP while the pole was moving. The arm did not move.",
                                "Release the E-stop and follow Recover.")
            a.pole_mm = start + min(1.0, el / dur) * (end - start)
            job.progress(f"Pole {start:.0f} → {goal:g} mm", step, steps)
            job.sleep(0.1)
        a.pole_mm = end if dur else a.pole_mm
        if abs(a.pole_mm - goal) > staging.POLE_TOL_MM:
            raise UserError(f"The {side.upper()} pole did not reach {goal:g} mm (read back {a.pole_mm:.0f} mm). "
                            "Nothing else ran.", "Check the pole and try again.")
        job.log(f"Pole at {a.pole_mm:.0f} mm (read back, within 5 mm of {goal:g}).")
        target = [h if t is None else int(t) for h, t in zip(a.hand, stage.hand)]
        dur = staging.hand_move_s(a.hand, target)
        if "hand" in self.faults[side]:
            target = [h + (t - h) // 2 for h, t in zip(a.hand, target)]
        if dur:
            job.progress(f"Hand to {staging.hand_words(stage.hand)}", step, steps)
            job.sleep(dur)
            if a.estop != "CLEAR":
                raise UserError("INTERRUPTED — E-STOP while the hand was moving. The arm did not move.",
                                "Release the E-stop and follow Recover.")
        a.hand = target
        if not staging.same_hand(a.hand, stage.hand):
            raise UserError(f"The {side.upper()} hand did not reach its pose (read back "
                            f"{', '.join(map(str, a.hand))}). Nothing else ran.", "Check the hand and try again.")
        job.log("Hand in pose (read back): " + ", ".join(map(str, a.hand)))

    def stage(self, side: str, stage: "staging.Stage", title: str):
        """Stage the pole and the hand only (Prepare's Apply)."""
        self.require_motion(side)

        def work(job):
            self._stage(job, side, stage)
            return dict(pole_mm=self.arm(side).pole_mm, hand=self.arm(side).hand)

        return self.jobs.start("stage", title, work, arm=side)

    def in_stage(self, side: str, stage: "staging.Stage") -> bool:
        a = self.arm(side)
        return abs(a.pole_mm - stage.pole_mm) <= staging.POLE_TOL_MM and staging.same_hand(a.hand, stage.hand)

    # ── pole, hand, teach
    def prepare(self, side: str, pole_mm: float, hand: list[int], title: str):
        self.require_motion(side)
        a = self.arm(side)

        def work(job):
            start, goal = a.pole_mm, float(pole_mm)
            dur = max(0.5, abs(goal - start) / POLE_MM_S)
            t0 = time.time()
            while True:
                f = min(1.0, (time.time() - t0) / dur)
                a.pole_mm = start + f * (goal - start)
                job.progress(f"Moving the pole to {goal:.0f} mm", 1, 2)
                if f >= 1.0:
                    break
                job.sleep(0.1)
            job.log(f"Pole at {goal:.0f} mm (read back).")
            job.progress("Setting the hand", 2, 2)
            job.sleep(1.0)
            a.hand = [int(v) for v in hand]
            job.log("Hand set (read back).")
            return dict(pole_mm=goal, hand=a.hand)

        return self.jobs.start("prepare", title, work, arm=side)

    def move_pole(self, side: str, pole_mm: float, title: str):
        self.require_motion(side)
        a = self.arm(side)
        if not 0 <= float(pole_mm) <= 400:
            raise UserError("The pole height must be 0 to 400 mm.")

        def work(job):
            start, goal = a.pole_mm, float(pole_mm)
            dur = max(0.5, abs(goal - start) / POLE_MM_S)
            t0 = time.time()
            while (f := min(1.0, (time.time() - t0) / dur)) < 1.0:
                a.pole_mm = start + f * (goal - start)
                job.progress(f"Moving the pole to {goal:.0f} mm", 1, 1)
                job.sleep(0.1)
            a.pole_mm = goal
            job.log(f"Pole at {goal:.0f} mm (read back).")
            return dict(pole_mm=goal)

        return self.jobs.start("pole", title, work, arm=side)

    def set_hand(self, side: str, counts: list, title: str):
        self.require_motion(side)
        a = self.arm(side)
        vals = [int(v) for v in counts]
        if len(vals) != 6 or not all(0 <= v <= 1000 for v in vals):
            raise UserError("The hand needs six values from 0 to 1000.")

        def work(job):
            job.progress("Setting the hand", 1, 1)
            job.sleep(0.8)
            a.hand = vals
            job.log("Hand set (read back): " + ", ".join(map(str, vals)))
            return dict(hand=vals)

        return self.jobs.start("hand", title, work, arm=side)

    def teach_start(self, side: str, setup: "staging.Stage | None") -> dict:
        """Drag teaching starts only with the arm in the region's setup (read back), so the
        recording's stage is what the robot really held."""
        self.require_motion(side)
        a = self.arm(side)
        if setup is None:
            raise UserError("This region has no pole height and hand pose to teach with.",
                            "Set them in Prepare first.", code="stage")
        if not self.in_stage(side, setup):
            raise UserError(f"The {side.upper()} arm is not in this region's setup ({staging.describe(setup)}): "
                            f"it reads pole {a.pole_mm:.0f} mm, hand {', '.join(map(str, a.hand))}.",
                            "Press Apply in Prepare for the pole and the hand, then start again.", code="stage")
        a.teaching, a.teach_started = True, time.time()
        a.teach_stage = setup
        return a.view()

    def teach_stop(self, side: str) -> float:
        a = self.arm(side)
        if not a.teaching:
            raise UserError("Teaching is not running on this arm.")
        a.teaching = False
        return time.time() - (a.teach_started or time.time())

    def move_to(self, side: str, what: str, title: str, seconds: float = 2.5, stage: "staging.Stage | None" = None):
        """Move the arm to a pose (a start pose, a file's first line), staging the pole and hand
        first when the pose belongs to a trajectory. The real backend sweeps the joint-space line
        for collisions first (R4); the stand-in only takes the time."""
        self.require_motion(side)
        a = self.arm(side)

        def work(job):
            if stage is not None:
                self._stage(job, side, stage)
            t0 = time.time()
            while (el := time.time() - t0) < seconds:
                if a.estop != "CLEAR":
                    raise UserError("INTERRUPTED — E-STOP. The arm did not reach the pose.",
                                    "Release the E-stop and follow Recover.")
                job.progress(f"Moving to {what}", int(100 * el / seconds), 100)
                job.sleep(0.1)
            job.log(f"At {what} (joints read back).")
            return dict(at=what)

        return self.jobs.start("move", title, work, arm=side)

    def playback(self, side: str, lines: int, *, dry_run: bool, title: str, stage: "staging.Stage"):
        """Stage the pole and hand, move to a recording's start, then replay it: (lines - 1) x 5 ms."""
        self.require_motion(side)
        a = self.arm(side)
        want = "SIM" if dry_run else "REAL"

        def work(job):
            self._stage(job, side, stage)
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

        return self.jobs.start("playback", title, work, arm=side)

    # ── programs
    def programs(self, side: str) -> list[dict]:
        a = self.arm(side)
        return [dict(id=i, **a.programs[i]) if i in a.programs else dict(id=i, name=None)
                for i in range(1, 101)]

    def upload(self, side: str, program_id: int, path: Path, name: str, sha256: str, replace: bool, title: str):
        a = self.arm(side)
        if not a.connected:
            raise UserError(f"The {side.upper()} arm is not connected.", "Connect it from the bar at the top.")
        pid = int(program_id)
        if not 1 <= pid <= 100:
            raise UserError("Program numbers are 1 to 100.")
        cur = a.programs.get(pid)
        if cur and cur.get("sha256") == sha256:
            return None                                       # unchanged: nothing to send
        if cur and not replace:
            raise UserError(f"Program {pid} already holds “{cur['name']}”.", "Confirm the replacement to overwrite it.")
        size = Path(path).stat().st_size
        lines = sum(1 for _ in open(path, "rb"))

        def work(job):
            dur = max(1.0, size / UPLOAD_BYTES_S)
            t0 = time.time()
            while (el := time.time() - t0) < dur:
                job.progress(f"Sending {size / 1e3:.0f} kB to program {pid}", int(100 * el / dur), 100)
                job.sleep(0.2)
            a.programs[pid] = dict(name=name, lines=lines, bytes=size, sha256=sha256,
                                   uploaded=time.strftime("%Y-%m-%d %H:%M"))
            a.save()
            job.log(f"Stored as program {pid}; name and size read back.")
            return dict(program_id=pid, lines=lines, bytes=size, seconds=round(dur, 1))

        return self.jobs.start("upload", title, work, arm=side)

    def run(self, side: str, program_id: int, *, dry_run: bool, title: str, on_done=None,
            stage: "staging.Stage | None" = None):
        """Run a stored program. With ``stage`` (always, for a region's trajectory) the pole and
        hand are staged and read back first, and the program never starts when either fails."""
        self.require_motion(side)
        a = self.arm(side)
        prog = a.programs.get(int(program_id))
        if not prog:
            raise UserError(f"Program {program_id} is empty on the {side.upper()} arm.", "Send the file to the robot first.")
        want = "SIM" if dry_run else "REAL"

        def work(job):
            if stage is not None:
                self._stage(job, side, stage)
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

        return self.jobs.start("run", title, work, arm=side)
