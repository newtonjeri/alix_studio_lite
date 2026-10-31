"""A plan's working stretch moved with the commode (the owner, 31 Oct: the depth camera to place the arms where a lid
or seat task starts, and the UGV's offset the same way: "if there is an offset, we only apply it to the carrying path of
the involved arm", the stages as the task configs group them).

WHAT MOVES: each arm's own block of its task config, the path that works the lid or the seat: from its move to the
pre-start pose, through the start and the carry (execute_path: the config's cleaning points from the start pose), to its
retracts (move_to_retract, move_to_retract_2, or back to the pre-start pose), moved whole by the commode's move: every
20 ms of each by IK from the plan's own joints (each seeded with the plan's line and the change the line before took, the
pole where the plan has it), the lines between interpolated in the joints, the plan's own timing; a move that starts on
the line the one before ends on still does. The move into the stretch (from rest or the inspection pose) ends on the
moved pre-start, and the move out of it (to rest or the inspection pose) starts where the stretch ends: each blended in
the joints along its way, as Lite's preview of a config edit blends (contact_preview._blend). Everything else is the
plan's: the inspection poses (the camera's place readings compare only with looks from the same pose), rest, the hands,
the checks. The height goes to the pole the arm's stretch runs at (its last pole move before the stretch): the arm's
joints then follow the move sideways and its turn alone. The pre-planned re-lifts (rungs) move with the plan.

REFUSED, nothing moved (UserError code "shift"), each said:
- a move beyond the owner's bounds (31 Oct: 30 mm, 3 deg; the height 15 mm);
- a pose the IK cannot reach (0.5 mm, 0.5 deg: the solver's own tolerance);
- a joint within LIMIT_MARGIN_DEG of its limit, or BRANCH_DEG from the plan's own line (another branch of the arm);
- a line ALIX's own stream check refuses (track.canfd_preflight at the per_joint gate: the check every stream passes);
- the wrist (J6) within WRIST_ZONE_DEG of its singularity and nearer it than the plan's own move comes, by more than
  WRIST_MARGIN_DEG;
- the pole out of its travel.
VERIFIED: every 5 ms line of a moved stretch, its hand against the plan's hand moved by the move (and the ends of the
moves into and out of it): within VERIFY_MM and VERIFY_DEG (the owner, 31 Oct: "verify whether by applying the offset
we can translate the path by the same margin").
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from .jobs import UserError

MAX_MM = 30.0              #: the owner, 31 Oct: the commode farther than this from where the plan works is a person's to see
DEAD_MM, DEAD_DEG = 5.0, 1.0   #: the owner, 31 Oct: "we can be able to open with a tolerance of plus or minus 5 mm": a
                               #: commode nearer than this to where the plan works (1 deg: 5 mm at its ends) runs the plan
MAX_YAW_DEG = 3.0
MAX_DZ_MM = 15.0
IK_EVERY = 4               #: every 4th line (20 ms) by IK, the lines between interpolated (contact_preview's)
VERIFY_MM, VERIFY_DEG = 1.0, 0.5
BRANCH_DEG = 20.0          #: a joint this far from the plan's own line: another branch of the arm
LIMIT_MARGIN_DEG = 2.0
WRIST_ZONE_DEG = 15.0      #: the wrist near its singularity (J6 = 0: J5 and J7 in line): within this of it, a moved path
WRIST_MARGIN_DEG = 1.0     #: comes no nearer than the plan's own move does, by more than this
INTO = "move_to_pre_start"
WORK = ("move_to_pre_start", "move_to_start", "execute_path", "move_to_retract", "move_to_retract_2")
CENTRE_XY = (0.735, 0.0)   #: commode_c's middle at its scene place, base_link (body x 0.27 m, the body at x 1.005 turned 180 deg)


@dataclass(frozen=True)
class Move:
    """The commode's rigid move in base_link: (dx, dy, dz) m of its middle at ``about`` (base_link x, y) and a turn of
    ``dyaw_deg`` about the vertical through it."""
    dx_m: float = 0.0
    dy_m: float = 0.0
    dz_m: float = 0.0
    dyaw_deg: float = 0.0
    about: tuple = CENTRE_XY

    def matrix(self, height: bool = True) -> np.ndarray:
        t = math.radians(self.dyaw_deg)
        R = np.array([[math.cos(t), -math.sin(t), 0.0], [math.sin(t), math.cos(t), 0.0], [0.0, 0.0, 1.0]])
        c = np.array([float(self.about[0]), float(self.about[1]), 0.0])
        D = np.eye(4)
        D[:3, :3] = R
        D[:3, 3] = c - R @ c + np.array([self.dx_m, self.dy_m, self.dz_m if height else 0.0])
        return D

    @classmethod
    def from_change(cls, change: dict, about=CENTRE_XY) -> "Move":
        """A place change (alix_perception_core.fixture_place.change: dx, dy, dz m of the commode's middle, dyaw deg) as
        the move of the commode about its middle ``about`` (base_link x, y: where the reference reads it)."""
        return cls(float(change["dx_m"]), float(change["dy_m"]), float(change["dz_m"]), float(change["dyaw_deg"]),
                   (float(about[0]), float(about[1])))

    @property
    def small(self) -> bool:
        """Within the owner's tolerance (DEAD_MM, DEAD_DEG): the plan runs as it is."""
        return (math.hypot(self.dx_m, self.dy_m) * 1000.0 < DEAD_MM and abs(self.dz_m) * 1000.0 < DEAD_MM
                and abs(self.dyaw_deg) < DEAD_DEG)

    @property
    def sideways(self) -> bool:
        return bool(self.dx_m or self.dy_m or self.dyaw_deg)

    def words(self) -> str:
        return (f"x {self.dx_m * 1000:+.1f}, y {self.dy_m * 1000:+.1f}, z {self.dz_m * 1000:+.1f} mm, "
                f"turned {self.dyaw_deg:+.2f} deg")


def refuse(text: str, hint: str = "") -> UserError:
    return UserError(f"Not moved with the commode: {text}", hint or "Nothing was sent: a person looks at where the robot "
                     "and the commode stand.", code="shift")


def bounds(move: Move) -> None:
    far = math.hypot(move.dx_m, move.dy_m) * 1000.0
    if far > MAX_MM + 1e-9 or abs(move.dyaw_deg) > MAX_YAW_DEG + 1e-9 or abs(move.dz_m) * 1000.0 > MAX_DZ_MM + 1e-9:
        raise refuse(f"the commode stands {far:.1f} mm, {move.dyaw_deg:+.2f} deg and {move.dz_m * 1000:+.1f} mm in height "
                     f"from where the plan works, beyond {MAX_MM:.0f} mm, {MAX_YAW_DEG:.0f} deg and {MAX_DZ_MM:.0f} mm.",
                     "Move the UGV nearer its place, or have the plan made for this place.")


def stretches(entries: list) -> dict:
    """{entry position: "into" | "moved" | "out"}: each arm's working stretch (WORK, from its first move to the
    pre-start), the move that brings the arm into it and the one that takes it out."""
    roles, where = {}, {}
    for pos, e in enumerate(entries):
        if e.kind != "arm":
            continue
        at = where.get(e.side, "before")
        if at == "before" and e.stage == INTO:
            roles[pos], where[e.side] = "into", "in"
        elif at == "in" and e.stage in WORK:
            roles[pos] = "moved"
        elif at == "in":
            roles[pos], where[e.side] = "out", "after"
    return roles


def ik_frames(task: str) -> dict:
    """{side: the frame its block's poses are given in} (the task config's ik_frame), as the planner solves them."""
    import yaml

    from .. import vendor_paths
    from . import alix_sync, mechanism

    base = mechanism.TASKS.get(task, {}).get("variant_of", task)
    cfg = yaml.safe_load((vendor_paths.ALIX / alix_sync.CONFIGS_REL / f"{base}.yaml").read_text(encoding="utf-8"))
    out = {}
    for side in ("left", "right"):
        frame = ((cfg.get(side) or {}).get("task_parameters") or {}).get("ik_frame")
        if frame:
            out[side] = frame
    return out


def start_poles(data: dict) -> dict:
    """{side: pole m} before any pole move of the plan: its strokes' (track_groups pre_stroke), as contact_preview."""
    out = {}
    for g in data.get("track_groups") or []:
        pm = (g.get("pre_stroke") or {}).get("pole_m")
        if pm is not None:
            out.setdefault("left", float(pm))
            out.setdefault("right", float(pm))
    return out


@lru_cache(maxsize=None)
def _limits(side: str) -> tuple:
    """(lower, upper) of the arm's seven joints, degrees (the URDF's, as the IK solver clamps to them)."""
    from alix_workspace.ik import IKSolver

    from . import robot_fk

    m = robot_fk.model()
    pfx = "L" if side == "left" else "R"
    solver = IKSolver(m, m.root_link(), f"{pfx}_tip_frame", locked={f"{pfx}_sliding_plate_joint": 0.15})
    order = [int(j.name.split("joint")[-1]) - 1 for j in solver.movable]
    lo, hi = np.zeros(7), np.zeros(7)
    lo[order], hi[order] = np.degrees(solver.lower), np.degrees(solver.upper)
    return tuple(lo), tuple(hi)


@lru_cache(maxsize=None)
def _ref() -> np.ndarray:
    from . import robot_fk

    m = robot_fk.model()
    return m.fk(m.chain(m.root_link(), "alix_ref_frame"), [])


def _rot_deg(R: np.ndarray) -> float:
    return float(math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0)))))


def _off(A: np.ndarray, B: np.ndarray) -> tuple:
    """The largest position (mm) and orientation (deg) difference between two runs of hand poses."""
    mm = float(np.max(np.linalg.norm(A[:, :3, 3] - B[:, :3, 3], axis=1))) * 1000.0 if len(A) else 0.0
    deg = max((_rot_deg(a[:3, :3] @ b[:3, :3].T) for a, b in zip(A, B)), default=0.0)
    return mm, deg


def _blend(rows: np.ndarray, d0: np.ndarray, d1: np.ndarray) -> np.ndarray:
    from .contact_preview import _blend as blend

    return blend(rows, d0, d1)


class Shift:
    """One plan's (or one rung block's) entries moved by ``move`` (see the module)."""

    def __init__(self, move: Move, frames: dict, poles: dict, label: str):
        self.move, self.frames, self.label = move, dict(frames), label
        self.pole = dict(poles)              # the plan's pole per side as its entries go
        self.raised: dict = {}               # side -> the pole raise it runs its stretch at (m)
        T_ref = _ref()
        self.D_ref = np.linalg.inv(T_ref) @ move.matrix(height=False) @ T_ref     # sideways and turn, alix_ref_frame
        self.D_full = np.linalg.inv(T_ref) @ move.matrix(height=True) @ T_ref
        self.rows: list = []                 # the report: one row per entry that changed
        self.solvers: dict = {}

    def _frame(self, side: str) -> str:
        if side not in self.frames:
            raise refuse(f"{self.label}: the task config names no IK frame for the {side} arm.")
        return self.frames[side]

    def _solver(self, side: str, pole_m: float):
        """(fk: lines in degrees -> hand poses in alix_ref_frame, solve: contact_preview's IK) at this pole."""
        from .contact_preview import _fk_ref, _ik

        key = (side, round(pole_m, 5))
        if key not in self.solvers:
            frame = self._frame(side)
            self.solvers[key] = (_fk_ref(side, frame, pole_m), _ik(side, frame, pole_m))
        return self.solvers[key]

    def entries(self, entries: list, roles: dict, raise_at: dict) -> list:
        from . import mechanism

        out, carry, last = [], {}, {}
        for pos, e in enumerate(entries):
            if e.kind == "pole":
                self.pole[e.side] = float(e.last()[0])
                if raise_at.get(e.side) == pos and self.move.dz_m:
                    new = self.pole[e.side] + self.move.dz_m
                    from .staging import POLE_MAX_MM
                    if not 0.0 <= new * 1000.0 <= POLE_MAX_MM:
                        raise refuse(f"{self.label}: the {e.side} pole would stand at {new * 1000:.0f} mm, outside its "
                                     f"0 to {POLE_MAX_MM:.0f} mm.")
                    self.raised[e.side] = self.move.dz_m
                    self.rows.append(dict(side=e.side, stage=e.stage, role="pole", pole_mm=[round(self.pole[e.side] * 1000, 1),
                                                                                          round(new * 1000, 1)]))
                    e = dataclasses.replace(e, q=np.asarray(e.q, dtype=float) + self.move.dz_m)
                out.append(e)
                continue
            role = roles.get(pos)
            if e.kind != "arm" or role is None or e.rows is None:
                out.append(e)
                continue
            rows = np.asarray(e.rows, dtype=float) / 1000.0
            d0 = carry.get(e.side, np.zeros(7))
            joined = e.side in last and np.array_equal(last[e.side][0], rows[0])
            if role == "into":
                new = _blend(rows, d0, self._solve(e, [rows[-1]], rows[-1:])[0] - rows[-1]) if self.move.sideways else rows + 0.0
            elif role == "moved":
                new = self._moved(e, rows, last[e.side][1] if joined else None)
            else:
                new = _blend(rows, d0, np.zeros(7))
                if joined:
                    new[0] = last[e.side][1]
            self._check(e, role, rows, new)
            carry[e.side] = new[-1] - rows[-1]
            last[e.side] = (rows[-1], new[-1])
            mdeg = np.round(new * 1000.0).astype(np.int64)
            out.append(dataclasses.replace(e, rows=[[int(v) for v in r] for r in mdeg],
                                           t=np.arange(len(mdeg), dtype=float) * mechanism.CLOCK_S,
                                           q=np.radians(mdeg / 1000.0)))
        return out

    def _solve(self, e, keys_rows, seeds) -> np.ndarray:
        """IK of each line's hand moved by the move (its pole the plan's), seeded with ``seeds``."""
        fk, solve = self._solver(e.side, self.pole.get(e.side, 0.15))
        T = fk(np.asarray(keys_rows, dtype=float))
        out = []
        for Tk, seed in zip(T, seeds):
            q, ok, mm, deg = solve(self.D_ref @ Tk, seed)
            if not ok:
                raise refuse(f"{self.label}: the {e.side} arm's {e.stage} cannot reach its moved pose (IK off by "
                             f"{mm:.1f} mm, {deg:.2f} deg).", "The commode stands where this arm cannot follow it: move the "
                             "UGV nearer its place.")
            out.append(q)
        return np.asarray(out)

    def _moved(self, e, rows: np.ndarray, first) -> np.ndarray:
        if not self.move.sideways:
            return rows + 0.0
        n = len(rows)
        keys = list(range(0, n, IK_EVERY)) + ([n - 1] if (n - 1) % IK_EVERY else [])
        fk, solve = self._solver(e.side, self.pole.get(e.side, 0.15))
        T = fk(rows[keys])
        sol, delta = [], (np.zeros(7) if first is None else first - rows[0])
        for j, k in enumerate(keys):
            if j == 0 and first is not None:          # on the line the move before ends on: it stays joined
                sol.append(np.asarray(first, dtype=float))
                continue
            q, ok, mm, deg = solve(self.D_ref @ T[j], rows[k] + delta)
            if not ok:
                raise refuse(f"{self.label}: the {e.side} arm's {e.stage} cannot follow the commode at {k * 5} ms (IK off "
                             f"by {mm:.1f} mm, {deg:.2f} deg).", "Move the UGV nearer its place, or have the plan made for "
                             "this place.")
            sol.append(q)
            delta = q - rows[k]
        sol = np.asarray(sol)
        return np.stack([np.interp(np.arange(n), keys, sol[:, c]) for c in range(7)], axis=1)

    def _check(self, e, role: str, rows: np.ndarray, new: np.ndarray) -> None:
        from alix_dispatch import track

        from . import mechanism

        what = f"{self.label}: the {e.side} arm's {e.stage}"
        change = float(np.max(np.abs(new - rows))) if len(rows) else 0.0
        if change > BRANCH_DEG:
            raise refuse(f"{what} would turn a joint {change:.1f} deg from the plan's own: another branch of the arm.")
        lo, hi = (np.asarray(v) for v in _limits(e.side))
        near = np.minimum(new - lo, hi - new).min(axis=0)
        bad = [j + 1 for j in range(7) if near[j] < LIMIT_MARGIN_DEG and near[j] < np.minimum(rows - lo, hi - rows).min(axis=0)[j]]
        if bad:
            raise refuse(f"{what} would bring J{bad[0]} within {LIMIT_MARGIN_DEG:.0f} deg of its limit.")
        wrist_plan, wrist_new = float(np.abs(rows[:, 5]).min()), float(np.abs(new[:, 5]).min())
        if wrist_new < min(wrist_plan, WRIST_ZONE_DEG) - WRIST_MARGIN_DEG:
            raise refuse(f"{what} would take the wrist (J6) to {wrist_new:.1f} deg of its singularity, nearer than the "
                         f"plan's own {wrist_plan:.1f} deg.")
        mdeg = np.round(new * 1000.0).astype(np.int64).tolist()
        pre = track.canfd_preflight(mdeg, mechanism.CLOCK_S, mechanism.RATE_GATE) if len(mdeg) > 1 else {"ok": True}
        if not pre.get("ok"):
            raise refuse(f"{what}: ALIX's stream check refuses its lines ({'; '.join(pre.get('reasons') or [])}).")
        mm = deg = 0.0
        if self.move.sideways or e.side in self.raised:
            fk_new, _ = self._solver(e.side, self.pole.get(e.side, 0.15) + self.raised.get(e.side, 0.0))
            fk_old, _ = self._solver(e.side, self.pole.get(e.side, 0.15))
            pick = slice(None) if role == "moved" else (slice(-1, None) if role == "into" else slice(0, 1))
            got = fk_new(new[pick])
            want = self.D_full @ fk_old(rows[pick]) if role != "out" else fk_old(rows[pick])
            if role == "out":                          # its first line is where the stretch ended: moved
                want = self.D_full @ want
            mm, deg = _off(got, want)
            if mm > VERIFY_MM or deg > VERIFY_DEG:
                raise refuse(f"{what}: its hand would be {mm:.1f} mm and {deg:.2f} deg from the plan's hand moved with the "
                             f"commode (within {VERIFY_MM:.1f} mm and {VERIFY_DEG:.1f} deg).")
        self.rows.append(dict(side=e.side, stage=e.stage, role=role, lines=len(rows), joint_change_deg=round(change, 2),
                              hand_off_mm=round(mm, 2), hand_off_deg=round(deg, 3),
                              wrist_min_deg=[round(wrist_plan, 1), round(wrist_new, 1)],
                              rate_util=round(max(pre.get("peak_rate_util") or [0.0]), 3)))


def _raise_at(entries: list, roles: dict) -> dict:
    """{side: the position of its last pole move before its stretch}: the height goes there."""
    first = {}
    for pos in sorted(roles):
        first.setdefault(entries[pos].side, pos)
    out = {}
    for side, at in first.items():
        poles = [p for p, e in enumerate(entries[:at]) if e.kind == "pole" and e.side == side]
        if poles:
            out[side] = poles[-1]
    return out


def _rungs(plan, sh: Shift):
    """``plan``'s pre-planned re-lift blocks, moved with the plan."""
    load = plan.rung_block

    def rung_block(k: int, lifter: str) -> list | None:
        block = load(k, lifter)
        if block is None or not sh.move.sideways:
            return block
        again = Shift(sh.move, sh.frames, sh.pole, f"{plan.title}, re-lift {k}")
        again.raised = dict(sh.raised)
        return again.entries(block, {i: "moved" for i, e in enumerate(block) if e.kind == "arm"}, {})

    return rung_block


def shift(plan, move: Move):
    """``plan`` (a mechanism.Plan) with each arm's working stretch moved by ``move``: a copy of it, its own methods on the
    moved entries, its pre-planned re-lifts moved the same way, ``move`` and ``shift_report`` (one row per entry
    changed) on it. UserError (code "shift") when a gate refuses: nothing is sent."""
    import copy

    bounds(move)
    out = copy.copy(plan)
    out.move, out.shift_report = move, []
    if not move.sideways and not move.dz_m:
        return out
    sh = Shift(move, ik_frames(plan.name), start_poles(plan.data), plan.title)
    roles = stretches(plan.entries)
    out.entries = sh.entries(plan.entries, roles, _raise_at(plan.entries, roles))
    out.shift_report = sh.rows
    out.rung_block = _rungs(plan, sh)
    return out


MATCH_S = 0.5      #: an event of the moved run is the plan's own when it meets the same links within this (a raised pole's
                   #: travel takes a little longer, and the run's clock with it)


def collisions(plan, moved, scene: dict | None = None, fid: str = "commode_c", cubicle_mm: int = 1400, dt: float = 0.04,
               progress=None) -> dict:
    """The collision check over ``plan``'s run and over ``moved``'s (shift's), the commode and the room moved with it
    (collision_check.check_states: the walls, the commode, the bin, the two arms; each entry after the other, as the
    contact model lays a plan out). A lid or a seat held by a hand has no state the check has meshes for: those moments
    take the task's end state in both runs alike. -> {"ok", "new": the moved run's events the plan's run has not,
    "plan": its events, "moved": the moved run's events, "seconds"}."""
    import time

    from . import collision_check, contact_sim, mechanism

    t0 = time.monotonic()
    scene = dict(scene or {})
    home = contact_sim.default_home()

    def run(p, sc):
        times, arms, ends = contact_sim.plan_samples(p.name, p, dt)
        phases, k = [], 0
        for t in times:
            while k < len(ends) - 1 and t > ends[k] + 1e-9:
                k += 1
            phases.append(mechanism.fixture_phase_at(p.entries, k, p.name) or mechanism.PHASE_AFTER.get(p.name, "lid_closed"))
        return collision_check.check_states(times, arms, home, sc, fid, phases, cubicle_mm, progress=progress)

    before = run(plan, scene)
    after = run(moved, dict(scene, move=moved.move.matrix().tolist()))

    def links(ev):
        return {(p["side"], p["link"], p["object"]) for p in ev.get("pairs") or []}

    new = [ev for ev in after.get("events") or []
           if not any(links(ev) & links(old) and ev["first_t"] <= old["last_t"] + MATCH_S and old["first_t"] <= ev["last_t"] + MATCH_S
                      for old in before.get("events") or [])]
    return dict(ok=not new, new=new, plan=before.get("events") or [], moved=after.get("events") or [],
                seconds=round(time.monotonic() - t0, 1))
