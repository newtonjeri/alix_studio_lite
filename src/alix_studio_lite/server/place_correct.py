"""Where a lid or seat task's plan works today, from its two sources (the owner, 31 Oct: "there are 2 sources of the
offset, the UGV reported value and the camera depth image positioning; the UGV value is the first and is compensated by
the depth image values, a secondary check for the mechanism tasks").

THE UGV, FIRST. The plans were made with the UGV at mechanism.STANDOFF_MM from the commode and no move (ALIX's place).
The placement the UGV reports for the run (placement.py: its standoff, its moves front, rear, left and right; no turn)
moves the commode the other way against the robot (placement.scene). That move is applied before anything of the plan
runs: each arm's working stretch moved with the commode (plan_shift.shift, its gates and its collision check).

THE DEPTH, SECOND. At the task's first look (the camera arm at the plan's inspection pose, before its first check; in a
sequence, where that check is not asked, a frame is taken there all the same: the owner, 31 Oct, agreed) the commode's
place is read on the look's frame (GapCheck.place) and compared with the REFERENCE look: the same arm pose and pole,
the lid and the seat in the same state. Its absolute place is off by the camera's mount (CAD, never calibrated:
inspection poses 4 and 1 read the same commode 11 and 15 mm and 3.2 deg apart), so only that change counts. The change
is where the commode stands now against where the plan worked:
- it agrees with the UGV's change since the reference (within UGV_TOL_MM and the look's own scatter): the depth's is
  run, the UGV's compensated by it (re-shifted only when the two put the hands DEAD_MM apart or more; a plan that cannot
  follow it, refused by plan_shift's gates, runs the UGV's as it would without the camera, said: Close the lid's plan of
  29 Oct, its pull at the wrist's singularity, follows a height only);
- it does not: the run stops for a person before any of the working moves (the UGV's report, the camera or the commode
  itself is not where the other says);
- a look the camera reads poorly (WEAK_LOOKS) only checks the UGV's against its own wider scatter: the UGV's runs;
- no reference for the look yet, no reading (a hand in the view, a part not at rest, the pose not a named one) or no
  camera (Hardware sim, the stand-in, the test harness): the UGV's alone, said.
A move that puts the hands' working path less than plan_shift.DEAD_MM from where the plan works runs the plan as it is
(the owner's +/-5 mm, measured where the hands work: 1 deg about the commode's middle moves them 4.3 to 4.8 mm).

THE REFERENCE, taught by itself (the owner, 31 Oct, agreed): the first look of the last run of the task that passed with
nothing moved (no move from either source, no re-lift) becomes the reference of its look; a run moved by either never
changes it. Kept per robot (its camera's mount) in the logs folder (REFERENCE_FILE), each with the digest of the plan it
was taught on: a re-planned task runs on the UGV's alone until a run of the new plan teaches it again.
"""
from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import plan_shift
from .jobs import UserError

UGV_TOL_MM = 10.0          #: the owner, 16 Oct: "+-10mm in all measurements" (the standoff to 5): the UGV's moves measure to it
GOOD_SCATTER_MM = 2.0      #: a good look's place: 0.4 mm within a check, a day's change agreeing across states to 1-2 mm
WEAK_LOOKS = {             #: (pose, state) -> (scatter mm, why): the depth checks the UGV's there, it does not compensate it
    ("inspection_pose_4", "lid shut"): (8.0, "the lid's front edge is behind the left glove: from this look the commode's "
                                             "place scatters 5 to 8 mm along x (the lab's checks, 26-31 Oct)"),
}
POLE_STEP_MM = 5.0         #: a look's pole, to this (the reference's pole and the look's: the same height)
REFERENCE_FILE = "place_reference.json"


def plans_placement() -> dict:
    """The UGV placement ALIX's lid and seat plans were made at."""
    from . import mechanism

    return {"standoff_mm": float(mechanism.STANDOFF_MM), "front_mm": 0.0, "rear_mm": 0.0, "left_mm": 0.0, "right_mm": 0.0}


def ugv_move(place: dict | None, fid: str) -> plan_shift.Move | None:
    """The commode's move against the robot from the UGV's reported ``place`` (placement.py's standoff and moves), from
    where the plans were made (plans_placement); None when no placement was reported (then the plans' own is assumed)."""
    if not place or place.get("standoff_mm") is None:
        return None
    from . import placement

    now = placement.scene(fid, None, place)["offset_m"]
    at = placement.scene(fid, None, plans_placement())["offset_m"]
    return plan_shift.Move(round(now[0] - at[0], 4), round(now[1] - at[1], 4))


def reference_path(robot) -> Path | None:
    """REFERENCE_FILE in the robot's logs folder (the camera's mount is the robot's), None without one."""
    log = getattr(robot, "log", None)
    return Path(log.root) / REFERENCE_FILE if log is not None and getattr(log, "root", None) else None


def run_placement(programs: dict, steps: dict) -> dict | None:
    """The UGV placement of an S2 run: the first of its Programs' that has one (workflow.programs_known), as its
    simulation shows the room (the Programs taught at others are said there)."""
    for side, rows in (steps or {}).items():
        for st in rows or []:
            if st.get("type") == "program":
                got = ((programs or {}).get(side) or {}).get(int(st["id"])) or {}
                if got.get("placement"):
                    return got["placement"]
    return None


def look_key(fid: str, pose: str, pole_mm: float, state: str) -> str:
    return f"{fid}|{pose}|{int(round(float(pole_mm) / POLE_STEP_MM) * POLE_STEP_MM)}|{state}"


class References:
    """The reference looks (REFERENCE_FILE beside the logs): {key: {task, plan, reading, about, ugv, at, record}}."""

    def __init__(self, path: Path | None):
        self.path = Path(path) if path is not None else None
        self._lock = threading.Lock()

    def _load(self) -> dict:
        if self.path is None:
            return {}
        try:
            return dict(json.loads(self.path.read_text(encoding="utf-8")).get("looks") or {})
        except (OSError, ValueError):
            return {}

    def get(self, key: str) -> dict | None:
        with self._lock:
            return self._load().get(key)

    def start_for(self, fid: str, ugv: plan_shift.Move | None):
        """(pose, pole_mm) -> (where a place search starts: dx, dy, dz m, dyaw deg; whether a reference gave it): the
        latest reference read at that pose and pole (whatever the lid and the seat), moved by the UGV's change since,
        else the UGV's move (searched 40 mm about it: the camera's mount puts an absolute place 11 to 15 mm off)."""
        looks = self._load()
        u = ugv or plan_shift.Move()

        def start(pose, pole_mm):
            head = look_key(fid, pose, pole_mm, "")
            refs = sorted((r for k, r in looks.items() if k.startswith(head)), key=lambda r: str(r.get("at")))
            if not refs:
                return (u.dx_m, u.dy_m, 0.0, 0.0), False
            r, u0 = refs[-1]["reading"], refs[-1].get("ugv") or {}
            return (float(r["dx_m"]) + u.dx_m - float(u0.get("dx_m") or 0.0), float(r["dy_m"]) + u.dy_m - float(u0.get("dy_m") or 0.0),
                    float(r["dz_m"]), float(r["dyaw_deg"])), True

        return start

    def teach(self, key: str, ref: dict) -> None:
        if self.path is None:
            return
        with self._lock:
            looks = self._load()
            looks[key] = ref
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix(".tmp")
            tmp.write_text(json.dumps({"looks": looks}, indent=1), encoding="utf-8")
            tmp.replace(self.path)


def _placement(d: dict):
    from alix_perception_core import fixture_place as fp

    return fp.Placement(ok=True, dx_m=float(d["dx_m"]), dy_m=float(d["dy_m"]), dz_m=float(d["dz_m"]),
                        dyaw_deg=float(d["dyaw_deg"]))


def depth_move(reading: dict, ref: dict, pose0) -> plan_shift.Move:
    """The commode's move from where the reference look read it to where ``reading`` reads it (fixture_place.change), about
    its middle as the reference reads it."""
    from alix_perception_core import fixture_place as fp

    a, b = _placement(ref["reading"]), _placement(reading)
    c = fp.change(a, b, np.asarray(pose0, dtype=float))
    P = a.pose(np.asarray(pose0, dtype=float))
    middle = P[:3, :3] @ np.asarray(fp.DEFAULT_PLACE.centre_body) + P[:3, 3]
    return plan_shift.Move.from_change(c, (float(middle[0]), float(middle[1])))


@dataclass
class Decision:
    """What a task runs with: ``move`` (None: as the plan is), from ``source`` ("ugv", "depth", "none"), said in ``words``."""
    move: plan_shift.Move | None
    source: str
    words: str
    level: str = "info"
    depth: plan_shift.Move | None = None
    refuse: str | None = None


def _mm(m: plan_shift.Move | None) -> str:
    return "none" if m is None else m.words()


def decide(ugv: plan_shift.Move | None, look: dict | None, ref: dict | None, hands=None) -> Decision:
    """The UGV's move compensated by the depth's (see the module). ``look``: the first look's reading (RealExec.look_place:
    ok, reading, pose, pole_mm, state, pose0), None without a camera; ``ref``: its reference look (References), None when
    none is taught; ``hands(a, b)``: how far two moves put the hands apart (mm; plan_shift.hands_apart_mm on the plan)."""
    hands = hands or (lambda a, b: 0.0)
    if look is None:
        return Decision(ugv, "ugv" if ugv is not None else "none",
                        f"no depth reading here (no camera): the UGV's placement alone, the commode {_mm(ugv)}")
    if not look.get("ok"):
        return Decision(ugv, "ugv" if ugv is not None else "none",
                        f"the camera could not read the commode's place ({look.get('note') or 'no reading'}): the UGV's "
                        f"placement alone, the commode {_mm(ugv)}", "warn")
    if ref is None:
        return Decision(ugv, "ugv" if ugv is not None else "none",
                        f"no reference look yet for {look['pose']} at pole {look['pole_mm']:.0f} mm, {look['state']} (taught by "
                        f"the next run of this task that passes with nothing moved): the UGV's placement alone, the commode "
                        f"{_mm(ugv)}")
    dm = depth_move(look["reading"], ref, look["pose0"])
    u = ugv or plan_shift.Move()
    u0 = ref.get("ugv") or {}
    ugv_change = plan_shift.Move(u.dx_m - float(u0.get("dx_m") or 0.0), u.dy_m - float(u0.get("dy_m") or 0.0))
    apart = math.hypot(dm.dx_m - ugv_change.dx_m, dm.dy_m - ugv_change.dy_m) * 1000.0
    scatter, why = WEAK_LOOKS.get((look["pose"], look["state"]), (GOOD_SCATTER_MM, None))
    tol = UGV_TOL_MM + scatter
    said = (f"the camera reads the commode {dm.words()} from where the reference look ({ref.get('at')}) saw it; the UGV's "
            f"placement says {ugv_change.words()} since then: {apart:.1f} mm apart")
    if apart > tol:
        return Decision(None, "none", said, "error", dm,
                        refuse=f"the UGV's placement and the camera disagree on where the commode stands: {said} (they agree "
                               f"to {tol:.0f} mm: the UGV's {UGV_TOL_MM:.0f}, this look's {scatter:.0f})")
    if why is not None:
        return Decision(ugv, "ugv" if ugv is not None else "none",
                        f"{said}, within this look's {tol:.0f} mm; {why}: the UGV's placement runs", "info", dm)
    if hands(dm, ugv) < plan_shift.DEAD_MM:
        return Decision(ugv, "ugv" if ugv is not None else "none",
                        f"{said}: the hands within {plan_shift.DEAD_MM:.0f} mm of the UGV's move, which runs", "info", dm)
    return Decision(dm, "depth", f"{said}: the camera's runs (the UGV's compensated)", "warn", dm)


@dataclass
class Correction:
    """A run's lid and seat tasks placed by the UGV and the depth (Runner(place=...)). ``ugv``: the UGV's move (ugv_move),
    None when it reported none; ``refs``: the reference looks; ``scene``, ``fid``, ``cubicle_mm``: the collision gate's
    world at the plans' placement (the moved run's moves the commode and the room with it). ``collide``: False skips the
    collision gate (a test's)."""
    ugv: plan_shift.Move | None = None
    refs: References = field(default_factory=lambda: References(None))
    fid: str = "commode_c"
    scene: dict | None = None
    cubicle_mm: int = 1400
    collide: bool = True
    log: list = field(default_factory=list)       # one row a task: its look, the sources, what it ran with

    def gated(self, plan0, move: plan_shift.Move | None, say) -> object:
        """``plan0`` moved by ``move`` through plan_shift's gates and the collision check, or ``plan0`` for a move within
        the dead band. UserError (code "shift") when refused: nothing of the plan is sent."""
        if move is None or plan_shift.hands_apart_mm(plan0, move) < plan_shift.DEAD_MM:
            return plan0
        moved = plan_shift.shift(plan0, move)
        if self.collide:
            got = plan_shift.collisions(plan0, moved, self.scene, self.fid, self.cubicle_mm)
            if not got["ok"]:
                ev = got["new"][0]
                raise plan_shift.refuse(f"{plan0.title}: moved {move.words()}, the arms would meet "
                                        f"{', '.join(ev.get('what') or ['something'])} at {ev['first_t']:.1f} s, which the "
                                        "plan's own run does not.")
        say(f"{plan0.title}: each arm's working stretch moved with the commode ({move.words()}): the hands within "
            f"{max((r.get('hand_off_mm') or 0.0) for r in moved.shift_report):.1f} mm of the plan's, moved", "warn")
        return moved


class TaskPlace:
    """One task's run of a Correction: the UGV's move applied as it starts (``plan``), the first look decided
    (``look``), the reference taught as it ends (``end``)."""

    def __init__(self, corr: Correction, plan0, say):
        from . import mechanism

        self.corr, self.plan0, self.say = corr, plan0, say
        self.applied = None
        self.decision: Decision | None = None
        self.look_at = mechanism.first_check_at(plan0.entries)
        roles = plan_shift.stretches(plan0.entries)
        first = sorted(set(roles) | set(plan_shift._raise_at(plan0.entries, roles).values()))
        self.changes_from = first[0] if first else len(plan0.entries)     # nothing before this differs, whatever the move
        self.stretch_from = min(roles) if roles else len(plan0.entries)    # the first working move (a pole raise may go before)
        self.raise_at = plan_shift._raise_at(plan0.entries, roles)         # {side: its pole raise for a height}
        self.waited = False
        self.points = plan_shift.hand_points(plan0)
        self.row = dict(task=plan0.name, ugv=None if corr.ugv is None else corr.ugv.words())
        self._thread = None
        self._got = None
        self.plan = plan0
        if corr.ugv is not None:
            self.plan = corr.gated(plan0, corr.ugv, say)
            self.applied = corr.ugv if self.plan is not plan0 else None
            if self.plan is plan0:
                say(f"{plan0.title}: the UGV's placement puts the commode {corr.ugv.words()} from where the plan was made: the "
                    f"hands within {plan_shift.DEAD_MM:.0f} mm, the plan runs as it is")
        corr.log.append(self.row)

    @property
    def started(self) -> bool:
        return self._thread is not None or self._got is not None

    def hands(self, a, b) -> float:
        return plan_shift.hands_apart_mm(self.plan0, a, b, self.points)

    def start_look(self, ex, take: bool) -> None:
        """At the first look: its frame (here, the arms still: RealExec.look_frame), then the place read and decided in a
        thread of its own while the moves before the working stretch run (``wait`` collects it before the first of them
        a move could change)."""
        frame_of, read = getattr(ex, "look_frame", None), getattr(ex, "read_look", None)
        if frame_of is None or read is None:
            self._got = ("done", decide(self.corr.ugv, None, None, self.hands), None)
            return
        if self._thread is not None:
            return
        shot = frame_of(take=take, task=self.plan0.name)
        if shot is None:
            self._got = ("done", decide(self.corr.ugv, dict(ok=False, note="no frame at the look"), None, self.hands), None, None)
            return
        start_for = self.corr.refs.start_for(self.corr.fid, self.corr.ugv)   # the search from where this robot's looks read it

        def work():
            try:
                got = read(shot, start_for)
                ref = None
                if got and got.get("ok"):
                    key = look_key(self.corr.fid, got["pose"], got["pole_mm"], got["state"])
                    got["key"] = key
                    ref = self.corr.refs.get(key)
                    if ref is not None and ref.get("plan") != _digest(self.plan0):
                        self.say(f"{self.plan0.title}: the reference look of {key} was taught on another version of the plan "
                                 f"({ref.get('at')}): not used until a run of this one teaches it again", "warn")
                        ref = None
                d = decide(self.corr.ugv, got, ref, self.hands)
                plan = None
                if d.refuse is None and d.source == "depth":
                    try:
                        plan = self.corr.gated(self.plan0, d.move, self.say)
                    except UserError as exc:            # the plan cannot follow the camera's: the UGV's runs, as before
                        if exc.code != "shift":
                            raise
                        d = Decision(self.corr.ugv, "ugv" if self.corr.ugv is not None else "none",
                                     f"{d.words}; but the plan cannot follow it ({str(exc).split(': ', 1)[-1]}): the UGV's "
                                     "placement runs, as it would without the camera", "warn", d.depth)
                self._got = ("done", d, plan, got)
            except BaseException as exc:                                            # noqa: BLE001 - raised in wait
                self._got = ("failed", exc)

        self._thread = threading.Thread(target=work, name="lite-place-look", daemon=True)
        self._thread.start()

    def wait(self, check_stop) -> tuple:
        """(the decision, the plan to run from here or None to keep the one running); raises a refusal."""
        self.waited = True
        while self._thread is not None and self._thread.is_alive():
            check_stop()
            self._thread.join(0.05)
        got = self._got
        self._got = ("taken",)
        if got is None or got[0] == "taken":
            return None, None
        if got[0] == "failed":
            raise got[1]
        _, d, plan = got[:3]
        self.look = got[3] if len(got) > 3 else None
        self.decision = d
        self.row.update(look=None if not self.look else {k: self.look.get(k) for k in ("ok", "pose", "pole_mm", "state", "key",
                                                                                          "note", "record")},
                        depth=None if d.depth is None else d.depth.words(), source=d.source, words=d.words,
                        move=None if d.move is None else d.move.words())
        self.say(f"PLACE {self.plan0.title}: {d.words}", d.level)
        if d.refuse is not None:
            self.row["verdict"] = "refused"
            raise UserError(f"{self.plan0.title} STOPPED before its working moves: {d.refuse}.",
                            "A person looks at where the UGV and the commode stand, then the UGV's placement is entered "
                            "again (or the commode put back).", code="place")
        if d.source == "depth" and plan is not None:
            self.applied = d.move
            return d, plan
        return d, None

    def end(self, outcome: str, relifts: int) -> None:
        """Teach the reference: this run passed (``outcome`` done, no re-lift) with nothing moved, and its first look read
        the commode's place."""
        look = getattr(self, "look", None)
        self.row.update(outcome=outcome, relifts=relifts)
        if outcome != "done" or relifts or self.applied is not None or not look or not look.get("ok") or not look.get("key"):
            return
        if self.decision is not None and self.decision.source == "depth":
            return
        self.corr.refs.teach(look["key"], dict(task=self.plan0.name, plan=_digest(self.plan0),
                                               reading={k: look["reading"][k] for k in ("dx_m", "dy_m", "dz_m", "dyaw_deg",
                                                                                         "lid_deg", "seat_deg")},
                                               ugv=None if self.corr.ugv is None else dict(dx_m=self.corr.ugv.dx_m,
                                                                                           dy_m=self.corr.ugv.dy_m),
                                               at=time.strftime("%Y-%m-%d %H:%M:%S"), record=look.get("record")))
        self.row["taught"] = look["key"]
        self.say(f"PLACE {self.plan0.title}: this run passed with nothing moved: its first look is now the reference of "
                 f"{look['key']}")


def _digest(plan) -> str:
    from . import mechanism

    return mechanism._sha256(plan.path)
