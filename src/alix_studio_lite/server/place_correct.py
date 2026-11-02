"""Where a lid or seat task's plan works today, from its two sources (the owner, 31 Oct: "there are 2 sources of the
offset, the UGV reported value and the camera depth image positioning; the UGV value is the first and is compensated by
the depth image values, a secondary check for the mechanism tasks"; 2 Nov: "if we read a value within the tolerance,
it means there's no movement needed, we reuse the plan; if the read value from the camera is beyond the tolerance, we
need to apply the value, the correction, and generate a new plan from the offset").

AN OFFSET MOVES THE PRE-START AND THE START, AND THE PATH WITH THEM (the owner, 2 Nov: "all the points and thus the path
is defined relative to the start pose, so offsetting the start pose applies to the path"): each arm's working stretch,
its pre-start, start, carry and retracts, moved whole (plan_shift.shift: IK every 20 ms on the plan's own timing, its
gates and its collision check); everything else is the plan's (the inspection poses, rest, the hands, the checks).

THE UGV, FIRST. The plans were made with the UGV at mechanism.STANDOFF_MM from the commode and no move (ALIX's place).
The placement the UGV reports for the run (placement.py: its standoff, its moves front, rear, left and right; no turn)
moves the commode the other way against the robot (placement.scene). That move is applied before anything of the plan
runs. Until the UGV is integrated it reports none: every task of the lab's project is at 250 mm, no move, and the plans
run as they were made.

THE CAMERA, SECOND. At the task's first look (the camera arm at the plan's inspection pose, before its first check; in a
sequence, where that check is not asked, a frame is taken there all the same: the owner, 31 Oct, agreed) the commode's
place is read on the look's frame (GapCheck.place) and compared with the REFERENCE look: the same arm pose and pole,
the lid and the seat in the same state. Its absolute place is off by the camera's mount (the URDF's, never calibrated:
inspection poses 4 and 1 read the same commode 11 and 15 mm and 3.2 deg apart), so only that change counts: where the
commode stands now against where the plan worked. Against the move the run has (the UGV's):
- within the owner's tolerance (plan_shift.DEAD_MM where the hands work: 1 deg about the commode's middle moves them 4.3
  to 4.8 mm): no correction, the plan is reused;
- beyond it: the camera's value is applied and the plan made again from it; a plan that cannot follow it (plan_shift's
  gates: IK, branch, limits, the wrist's singularity) runs as it would without the camera, said (Close the lid's plan of
  29 Oct, its pull at the wrist's singularity, follows a height only); beyond plan_shift's bounds (30 mm, 3 deg, 15 mm
  in height: the owner, 31 Oct, "a person's to see") the run stops before any working move;
- a look that reads an axis poorly (WEAK_LOOKS: Open the lid's, the lid's front edge behind the left glove) leaves that
  axis at the UGV's;
- no reference for the look yet, no reading (a hand in the view, a part not at rest, the pose not a named one) or no
  camera (Hardware sim, the stand-in, the test harness): the UGV's alone, said.

THE RETRIES, by the camera too (the owner, 2 Nov: "all the retries should be decided by the camera ... the hard-coded
retries we have in the configuration files should be populated by the values from the camera"; "we can do away with
the rungs"). A check that fails with on_fail retry_lift (Open the lid's lifted check, 3 retries; Open the seat's, 1):
the commode's place is read again on that check's own frames (the camera arm where the first look was: TaskPlace.retry)
and decided by the same rule against the move the run has: within the tolerance the same lift again; beyond, the lift of
the plan made again from the camera's value; no frame, no reading or no reference: the same lift again. The config's
retry_offsets and ALIX's pre-planned rungs are not used. The arm goes back along the lift it made while that plan is
made (mechanism.Runner._camera_relift).

THE REFERENCE, taught by itself (the owner, 31 Oct, agreed): the first look of the last run of the task that passed with
nothing moved (no move from either source, no re-lift) becomes the reference of its look; a run moved by either never
changes it. Kept per robot (its camera's mount) in the logs folder (REFERENCE_FILE), each with the digest of the plan it
was taught on: a re-planned task runs on the UGV's alone until a run of the new plan teaches it again.

GENERATED AHEAD, while the robot runs (the owner, 1 Nov: "generate the first task, as it executes be generating the
other tasks"; "proceed to second task planning as the first executes, and continue generation in parallel with
execution"). A moved plan takes 1.2 to 2.8 s to make (plan_shift.shift 0.1 s, its collision check the rest; the plan's
own run checked once, then half that), and each task waited for its own before it started. Now:
- the run's start: its tasks (S2: its lid and seat stages, in order; a task on its own: itself) generated for the UGV's
  placement one after the other in a process of their own (LANES[1]), the first first: the next while one runs; a task
  takes its plan from there, and waits only for one not made yet (the run's first, when nothing runs before it);
- a task's first look: the camera's move made in another process (LANES[0]: the task running's, its own run checked
  there as the task starts), as the moves before its working stretch run (the Runner waits before the first move it
  changes, as before);
- the camera's move run on one task: the run's next tasks generated for it too (LANES[1]), the commode standing where it
  stands for the run: a later task whose own look reads it within DEAD_MM of it (at the hands) runs that plan at once.
Lite's own process makes none of them in a run (Correction.submit: in_workers): FCL, the collision check's, is not safe
across threads (collision_check), and in a thread of Lite's process the check holds Python's lock up to 80 ms at a time
against Lite's other threads (the page, STOP, the camera; measured 1 Nov: a 5 ms loop beside it ran 9.6 ms late on
average). The arms' 5 ms streams run in each arm's own process (robot_process): they do not wait on it either way.
"""
from __future__ import annotations

import dataclasses
import json
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import plan_shift
from .jobs import UserError

WEAK_LOOKS = {             #: (pose, state) -> (the axis it reads poorly, why): the camera's value left at the UGV's there
    ("inspection_pose_4", "lid shut"): ("x", "the lid's front edge is behind the left glove: from this look the commode's "
                                             "place scatters 5 to 8 mm along x (the lab's checks, 26-31 Oct)"),
}
POLE_STEP_MM = 5.0         #: a look's pole, to this (the reference's pole and the look's: the same height)
REFERENCE_FILE = "place_reference.json"
LANES = ("plans_now", "plans_ahead")   #: Lite's processes (workers.pool) for the task running's moved plans, the next tasks'


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

    def has_task(self, task: str, digest: str) -> bool:
        """A reference look taught on this plan of ``task`` (its digest): its look can run the camera's move."""
        return any(r.get("task") == task and r.get("plan") == digest for r in self._load().values())

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
    depth: plan_shift.Move | None = None         # what the camera read (``move``: what runs from it)
    refuse: str | None = None


def _mm(m: plan_shift.Move | None) -> str:
    return "none" if m is None else m.words()


def decide(ugv: plan_shift.Move | None, look: dict | None, ref: dict | None, hands=None,
           running: str = "the UGV's placement") -> Decision:
    """The UGV's move (``ugv``: the move the run has; None: none reported, the plans' own) and the camera's (see the
    module). ``look``: a look's reading (RealExec.read_look: ok, reading, pose, pole_mm, state, pose0), None without a
    camera; ``ref``: its reference look (References), None when none is taught; ``hands(a, b)``: how far two moves put the
    hands apart (mm; plan_shift.hands_apart_mm on the plan); ``running``: what ``ugv`` is, in the words."""
    hands = hands or (lambda a, b: 0.0)
    keep = "ugv" if ugv is not None else "none"
    if look is None:
        return Decision(ugv, keep, f"no depth reading here (no camera): {running} alone, the commode {_mm(ugv)}")
    if not look.get("ok"):
        return Decision(ugv, keep, f"the camera could not read the commode's place ({look.get('note') or 'no reading'}): "
                                   f"{running} alone, the commode {_mm(ugv)}", "warn")
    if ref is None:
        return Decision(ugv, keep, f"no reference look yet for {look['pose']} at pole {look['pole_mm']:.0f} mm, "
                                   f"{look['state']} (taught by the next run of this task that passes with nothing moved): "
                                   f"{running} alone, the commode {_mm(ugv)}")
    read = dm = depth_move(look["reading"], ref, look["pose0"])
    said = f"the camera reads the commode {dm.words()} from where the reference look ({ref.get('at')}) saw it"
    weak = WEAK_LOOKS.get((look["pose"], look["state"]))
    if weak is not None:                         # that axis left at the UGV's
        u = ugv or plan_shift.Move()
        dm = dataclasses.replace(dm, **({"dx_m": u.dx_m} if weak[0] == "x" else {"dy_m": u.dy_m}))
        said += f"; {weak[1]}: its {weak[0]} is left at {running}'s ({dm.words()})"
    apart = hands(dm, ugv)
    if apart < plan_shift.DEAD_MM:
        return Decision(ugv, keep, f"{said}: the hands within {apart:.1f} mm of where {running} puts them, inside the "
                                   f"{plan_shift.DEAD_MM:.0f} mm: no correction, the plan is reused", "info", read)
    try:
        plan_shift.bounds(dm)
    except UserError as exc:
        return Decision(None, "none", said, "error", read, refuse=f"{said}: {str(exc).split(': ', 1)[-1]}")
    return Decision(dm, "depth", f"{said}: the hands {apart:.1f} mm from where {running} puts them, beyond the "
                                 f"{plan_shift.DEAD_MM:.0f} mm: the camera's value is applied, the plan made again from it",
                    "warn", read)


def in_workers(lane: str, fn, *args):
    """``fn(*args)`` in Lite's process ``lane`` (workers.pool): Correction.submit in Real and the emulated runs."""
    from . import workers

    return workers.pool(lane).submit(fn, *args)


def _move_args(m: plan_shift.Move) -> tuple:
    return (m.dx_m, m.dy_m, m.dz_m, m.dyaw_deg, (float(m.about[0]), float(m.about[1])))


def _key(name: str, m: plan_shift.Move) -> tuple:
    a = _move_args(m)
    return (name,) + tuple(round(float(v), 5) for v in (*a[:4], *a[4]))


def _world(digest, scene, fid, cubicle_mm) -> tuple:
    return (digest, json.dumps(scene or {}, sort_keys=True, default=str), fid, int(cubicle_mm))


_OWN: dict = {}            # in each process: _world -> the plan's own collision check (plan_shift.own_run), kept


def prepare(name: str, path: str, digest: str, scene: dict | None, fid: str, cubicle_mm: int) -> float:
    """``name``'s own collision check (plan_shift.own_run) made and kept in this process, ahead of a move asked of it
    (generate then checks the moved run alone). -> seconds."""
    t0 = time.monotonic()
    from .. import vendor_paths

    vendor_paths.activate()
    from . import mechanism

    world = _world(digest, scene, fid, cubicle_mm)
    if world not in _OWN:
        _OWN[world] = plan_shift.own_run(mechanism.Plan(name, Path(path)), scene, fid, cubicle_mm)
    return round(time.monotonic() - t0, 2)


def generate(name: str, path: str, digest: str | None, move: tuple, scene: dict | None, fid: str, cubicle_mm: int,
             collide: bool = True) -> dict:
    """``name``'s plan (its file ``path``, as ``digest``) moved by ``move`` (_move_args) through plan_shift's gates and,
    ``collide``, the collision check against the plan's own run (kept in this process: prepare). In one of Lite's
    processes (LANES, Correction.submit), else here. -> {"entries", "report", "state", "seconds"}, what
    plan_shift.assemble takes; plan_shift's UserError (code "shift") when refused."""
    t0 = time.monotonic()
    from .. import vendor_paths

    vendor_paths.activate()
    from . import mechanism

    plan0 = mechanism.Plan(name, Path(path))
    if digest and _digest(plan0) != digest:
        raise plan_shift.refuse(f"{plan0.title}: its plan's file changed while the run was on.")
    m = plan_shift.Move(*move[:4], about=tuple(move[4]))
    moved = plan_shift.shift(plan0, m)
    if collide:
        world = _world(digest, scene, fid, cubicle_mm)
        got = plan_shift.collisions(plan0, moved, scene, fid, cubicle_mm, own=_OWN.get(world))
        _OWN[world] = got["own"]
        if not got["ok"]:
            ev = got["new"][0]
            raise plan_shift.refuse(f"{plan0.title}: moved {m.words()}, the arms would meet "
                                    f"{', '.join(ev.get('what') or ['something'])} at {ev['first_t']:.1f} s, which the "
                                    "plan's own run does not.")
    return dict(entries=moved.entries, report=moved.shift_report, state=getattr(moved, "shift_state", None),
                seconds=round(time.monotonic() - t0, 2))


def _wait(fut, check_stop=None) -> dict:
    """A lane's result (its refusal raised), ``check_stop`` asked every 50 ms meanwhile."""
    from concurrent.futures import TimeoutError as Pending

    while True:
        try:
            return fut.result(timeout=0.05)
        except Pending:
            if check_stop is not None:
                check_stop()


@dataclass
class Correction:
    """A run's lid and seat tasks placed by the UGV and the depth (Runner(place=...)). ``ugv``: the UGV's move (ugv_move),
    None when it reported none; ``refs``: the reference looks; ``scene``, ``fid``, ``cubicle_mm``: the collision gate's
    world at the plans' placement (the moved run's moves the commode and the room with it). ``collide``: False skips the
    collision gate (a test's). ``submit(lane, fn, *args) -> Future``: where the moved plans are made, ahead of the tasks
    (in_workers: Lite's processes LANES); None: here, each as it is needed (the tests')."""
    ugv: plan_shift.Move | None = None
    refs: References = field(default_factory=lambda: References(None))
    fid: str = "commode_c"
    scene: dict | None = None
    cubicle_mm: int = 1400
    collide: bool = True
    submit: object = None
    log: list = field(default_factory=list)       # one row a task: its look, the sources, what it ran with
    order: list = field(default_factory=list)     # the run's tasks in its order (ahead), ``at`` the one running
    at: int = -1
    _jobs: dict = field(default_factory=dict, repr=False)      # _key -> (its move, its Future)
    _others: list = field(default_factory=list, repr=False)    # the own runs' Futures (prepare)
    _lock: object = field(default_factory=threading.Lock, repr=False)

    def ahead(self, names: list, move: plan_shift.Move | None = None) -> int:
        """``names`` (the first call: the run's tasks in its order), each one's plan moved by ``move`` (None: the UGV's)
        generated in LANES[1] one after the other, the first first, the next while one runs (the owner, 1 Nov). A plan
        the move leaves within DEAD_MM (at the hands) runs as it is: nothing to make. Without ``submit``: nothing.
        -> how many plans are being made."""
        if self.submit is None:
            return 0
        from . import mechanism

        if not self.order:
            self.order = list(names)
        move = self.ugv if move is None else move
        if move is None:
            return 0
        made = 0
        for name in names:
            plan0 = mechanism.Plan(name)
            if plan_shift.hands_apart_mm(plan0, move) >= plan_shift.DEAD_MM:
                self._job(plan0, move, LANES[1])
                made += 1
        return made

    def _job(self, plan0, move: plan_shift.Move, lane: str) -> tuple:
        key = _key(plan0.name, move)
        with self._lock:
            got = self._jobs.get(key)
            if got is None:
                got = self._jobs[key] = (move, self.submit(lane, generate, plan0.name, str(plan0.path), _digest(plan0),
                                                           _move_args(move), self.scene, self.fid, self.cubicle_mm,
                                                           self.collide))
            return got

    def started(self, plan0) -> None:
        """A task of the run starts (TaskPlace): where the run is in its order; the plan's own collision run checked in
        LANES[0] by its first look, when a reference look of it can bring the camera's move there."""
        if plan0.name in self.order[self.at + 1:]:
            self.at = self.order.index(plan0.name, self.at + 1)
        if self.submit is not None and self.collide and self.refs.has_task(plan0.name, _digest(plan0)):
            with self._lock:
                self._others.append(self.submit(LANES[0], prepare, plan0.name, str(plan0.path), _digest(plan0), self.scene,
                                                self.fid, self.cubicle_mm))

    def take(self, plan0, move: plan_shift.Move | None, say, check_stop=None):
        """``plan0`` moved by ``move`` through plan_shift's gates and the collision check, or ``plan0`` for a move within
        the dead band: generated ahead (that one, waited for when it is not made yet), else made now (LANES[0], or here
        without ``submit``). UserError (code "shift") when refused: nothing of the plan is sent."""
        if move is None or plan_shift.hands_apart_mm(plan0, move) < plan_shift.DEAD_MM:
            return plan0
        if self.submit is None:
            return self._moved(plan0, move, generate(plan0.name, str(plan0.path), _digest(plan0), _move_args(move),
                                                     self.scene, self.fid, self.cubicle_mm, self.collide), say)
        t0 = time.monotonic()
        got = self._jobs.get(_key(plan0.name, move))
        fut = (got or self._job(plan0, move, LANES[0]))[1]
        ready = fut.done()
        parts = _wait(fut, check_stop)
        waited = time.monotonic() - t0
        return self._moved(plan0, move, parts, say, ", made while the run went on" if got is not None and ready else
                           f", made while the run went on, {waited:.1f} s waited for it" if got is not None else
                           f", made now in {waited:.1f} s")

    def _moved(self, plan0, move, parts: dict, say, how: str = ""):
        moved = plan_shift.assemble(plan0, move, parts["entries"], parts["report"], parts["state"])
        say(f"{plan0.title}: each arm's working stretch moved with the commode ({move.words()}){how}: the hands within "
            f"{max((r.get('hand_off_mm') or 0.0 for r in moved.shift_report), default=0.0):.1f} mm of the plan's, moved",
            "warn")
        return moved

    def near(self, plan0, move: plan_shift.Move, points=None) -> tuple | None:
        """A plan of this task made ahead (after) for a move that puts the hands within DEAD_MM of where ``move`` puts
        them: (its parts, its move, how far apart mm), the nearest; None."""
        best = None
        for key, (m, fut) in list(self._jobs.items()):
            if key[0] != plan0.name or not fut.done() or fut.cancelled() or fut.exception() is not None:
                continue
            apart = plan_shift.hands_apart_mm(plan0, move, m, points)
            if apart < plan_shift.DEAD_MM and (best is None or apart < best[2]):
                best = (fut.result(), m, apart)
        return best

    def after(self, plan0, move: plan_shift.Move) -> None:
        """The camera read the commode moved by ``move`` at ``plan0``'s look: the run's tasks after it generated for that
        move too (their own looks read the same commode)."""
        if self.submit is not None and self.order and 0 <= self.at < len(self.order):
            try:
                self.ahead(self.order[self.at + 1:], move)
            except Exception:                                               # noqa: BLE001 - then made as they are needed
                pass

    def close(self) -> None:
        """The run ended: what is still queued is dropped (a job already being made ends in its process, unused)."""
        with self._lock:
            for fut in [f for _m, f in self._jobs.values()] + self._others:
                fut.cancel()
            self._jobs.clear()
            self._others.clear()


class TaskPlace:
    """One task's run of a Correction: the UGV's move applied as it starts (``plan``), the first look decided
    (``look``), the reference taught as it ends (``end``)."""

    def __init__(self, corr: Correction, plan0, say, check_stop=None):
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
        self._retry, self._retry_got, self._retry_n = None, None, 0     # a re-lift's look (retry)
        self.plan = plan0
        corr.started(plan0)
        if corr.ugv is not None:
            self.plan = corr.take(plan0, corr.ugv, say, check_stop)      # made ahead while the run went on (1 Nov)
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
        self._thread = self._in_thread(lambda: self._decided(read, shot, self.corr.ugv, "the UGV's placement"), "_got")

    def retry(self, ex, n: int) -> None:
        """Re-lift ``n`` after a check that failed (mechanism.Runner._camera_relift): the commode's place read again on
        that check's own frames (the camera arm where the first look was) and decided against the move the run has (the
        module's rule): within the tolerance the plan running again, beyond it the plan made again from the camera's
        value, in a thread of its own while the arm goes back along its lift (``wait_retry`` collects it before the lift
        again). No camera, frame, reading or reference: the plan running again."""
        self._retry, self._retry_got, self._retry_n = None, None, n
        frame_of, read = getattr(ex, "look_frame", None), getattr(ex, "read_look", None)
        shot = frame_of(take=False, task=self.plan0.name) if frame_of is not None and read is not None else None
        if shot is None:
            self._retry_got = ("done", decide(self.applied, None if frame_of is None else dict(ok=False, note="no frame of the "
                                              "check"), None, self.hands, "the plan running"), None, None)
            return
        self._retry = self._in_thread(lambda: self._decided(read, shot, self.applied, "the plan running"), "_retry_got")

    def _in_thread(self, fn, slot: str) -> threading.Thread:
        def work():
            try:
                setattr(self, slot, fn())
            except BaseException as exc:                                            # noqa: BLE001 - raised in the wait
                setattr(self, slot, ("failed", exc))

        t = threading.Thread(target=work, name="lite-place-look", daemon=True)
        t.start()
        return t

    def _decided(self, read, shot, baseline, running: str) -> tuple:
        """The commode's place read on ``shot`` (``read``: RealExec.read_look) and decided against ``baseline``, the move
        the run has (``running``, its words): ("done", the decision, the plan the camera's value runs or None, the
        reading)."""
        got = read(shot, self.corr.refs.start_for(self.corr.fid, self.corr.ugv))   # searched where this robot's looks read it
        ref = None
        if got and got.get("ok"):
            key = look_key(self.corr.fid, got["pose"], got["pole_mm"], got["state"])
            got["key"] = key
            ref = self.corr.refs.get(key)
            if ref is not None and ref.get("plan") != _digest(self.plan0):
                self.say(f"{self.plan0.title}: the reference look of {key} was taught on another version of the plan "
                         f"({ref.get('at')}): not used until a run of this one teaches it again", "warn")
                ref = None
        d = decide(baseline, got, ref, self.hands, running)
        plan = None
        if d.refuse is None and d.source == "depth":
            seen = d.move
            try:
                plan, d = self._camera_plan(d, baseline, running)
            finally:                                    # the run's next tasks made for it too (the owner, 1 Nov)
                self.corr.after(self.plan0, seen)
        return ("done", d, plan, got)

    def _camera_plan(self, d: Decision, baseline, running: str) -> tuple:
        """(the plan the camera's value runs, the decision): one made ahead for a move within DEAD_MM of it at the hands
        (an earlier task's look read the commode there: Correction.after), else made now (Correction.take); a plan that
        cannot follow it, or that could not be made, runs ``baseline`` as it would without the camera, said."""
        def without(why: str) -> tuple:
            return None, Decision(baseline, "ugv" if baseline is not None else "none",
                                  f"{d.words}; but {why}: {running} runs, as it would without the camera", "warn", d.depth)

        try:
            hit = self.corr.near(self.plan0, d.move, self.points)
            if hit is not None:
                parts, m, apart = hit
                plan = self.corr._moved(self.plan0, m, parts, self.say, f", made ahead for an earlier task's look (the hands "
                                        f"{apart:.1f} mm from the camera's value)")
                return plan, dataclasses.replace(d, move=m)
            return self.corr.take(self.plan0, d.move, self.say), d
        except UserError as exc:                        # the plan cannot follow the camera's: the one running runs, as before
            if exc.code != "shift":
                raise
            return without(f"the plan cannot follow it ({str(exc).split(': ', 1)[-1]})")
        except Exception as exc:                        # noqa: BLE001 - its process lost: as without the camera
            return without(f"its plan could not be made ({type(exc).__name__}: {exc})")

    def _look_row(self, look) -> dict | None:
        return None if not look else {k: look.get(k) for k in ("ok", "pose", "pole_mm", "state", "key", "note", "record")}

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
        self.row.update(look=self._look_row(self.look), depth=None if d.depth is None else d.depth.words(), source=d.source,
                        words=d.words, move=None if d.move is None else d.move.words())
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

    def wait_retry(self, check_stop) -> tuple:
        """(the decision, the plan to lift again from, or None for the plan running) of ``retry``; raises a refusal."""
        while self._retry is not None and self._retry.is_alive():
            check_stop()
            self._retry.join(0.05)
        got, self._retry, self._retry_got = self._retry_got, None, None
        if got is None:
            return None, None
        if got[0] == "failed":
            raise got[1]
        _, d, plan = got[:3]
        self.row.setdefault("retries", []).append(dict(          # source: depth (made again) or kept (the plan running)
            relift=self._retry_n, look=self._look_row(got[3] if len(got) > 3 else None),
            source="depth" if d.source == "depth" and plan is not None else "kept", words=d.words,
            depth=None if d.depth is None else d.depth.words(), move=None if d.move is None else d.move.words()))
        self.say(f"PLACE {self.plan0.title}, re-lift {self._retry_n}: {d.words}", d.level)
        if d.refuse is not None:
            raise UserError(f"{self.plan0.title} STOPPED before re-lift {self._retry_n}: {d.refuse}.",
                            "A person looks at where the robot and the commode stand.", code="place")
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
