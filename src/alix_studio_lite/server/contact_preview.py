"""An edited lid or seat config's contact forces before it is re-planned (the owner, 29 Oct: "we make edits to the paths, we
already know the areas we need to touch"): the plan's own moves, each moved by the change of its config's poses and
points, through contact_sim.

How a move follows its config's edit (an approximation of the planner, for edits of a few centimetres and degrees):
- a move to a cartesian pose (pre_start_pose, start_pose, retract_pose ...): its end solved again by IK (ALIX's own
  alix_workspace.ik, the pole locked, seeded at the plan's end, so on the plan's arm configuration), the change of its
  start and of its end blended over the move in the joints;
- the stroke (execute_path): each line's hand pose moved by the change of the points of the segment it is on (the
  planner's corners rounded as it rounded them), then solved by IK from the line before;
- a move to a named state (inspection, rest, half length): unchanged but for the change of its start, blended out.
The plan's timing is kept; the planner's own (TOPP-RA on the new path) differs a little. A config whose moves are not the
plan's (a stage added or removed) is refused: re-plan it in ALIX first.
"""
from __future__ import annotations

import dataclasses
import json
from pathlib import Path

import numpy as np

IK_EVERY = 4            #: the stroke's lines solved by IK every 4th (20 ms), the ones between interpolated in the joints


class EditedPlan:
    """A stand-in for mechanism.Plan with edited entries (``entries``, ``path``, ``name``, ``title``)."""

    def __init__(self, plan, entries, cfg_path: Path):
        self.name, self.title, self.path = plan.name, plan.title, plan.path
        self.entries = entries
        self.cfg_path = cfg_path


def _ik(side: str, frame: str, pole_m: float):
    from alix_workspace.ik import IKSolver

    from . import robot_fk

    m = robot_fk.model()
    pfx = "L" if side == "left" else "R"
    solver = IKSolver(m, m.root_link(), frame, locked={f"{pfx}_sliding_plate_joint": float(pole_m)})
    T_ref = m.fk(m.chain(m.root_link(), "alix_ref_frame"), [])
    order = [int(j.name.split("joint")[-1]) - 1 for j in solver.movable]

    def solve(T_in_ref: np.ndarray, seed_deg) -> tuple:
        seed = np.radians(np.asarray(seed_deg, dtype=float))[order]
        res = solver.solve(T_ref @ T_in_ref, q_init=seed, restarts=1, pos_tol=5e-4, rot_tol_deg=0.5, max_iters=200)
        q = np.zeros(7)
        q[order] = res.q
        return np.degrees(q), res.success, res.pos_err_m * 1000.0, res.rot_err_deg

    return solve


def _fk_ref(side: str, frame: str, pole_m: float):
    from . import alix_sync

    fk = alix_sync._fk()
    return lambda q_deg: fk(side, frame, np.radians(np.atleast_2d(q_deg)), pole_m)


def _blend(rows: np.ndarray, d0: np.ndarray, d1: np.ndarray) -> np.ndarray:
    """``rows`` (n x 7 deg) moved by ``d0`` at its start and ``d1`` at its end, blended by its progress in the joints."""
    if len(rows) < 2:
        return rows + d1
    step = np.linalg.norm(np.diff(rows, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(step)])
    lam = s / s[-1] if s[-1] > 1e-9 else np.linspace(0.0, 1.0, len(rows))
    return rows + d0[None, :] * (1.0 - lam)[:, None] + d1[None, :] * lam[:, None]


def _seg_delta(T_old: np.ndarray, old_pts: list, new_pts: list) -> np.ndarray:
    """The pose change at hand pose ``T_old`` on the old polyline: its segment's two ends' changes, by where it is along it."""
    from scipy.spatial.transform import Rotation as Rot, Slerp

    p = T_old[:3, 3]
    best, k, f = None, 0, 0.0
    for i in range(len(old_pts) - 1):
        a, b = old_pts[i][:3, 3], old_pts[i + 1][:3, 3]
        ab = b - a
        L2 = float(ab @ ab)
        t = 0.0 if L2 < 1e-12 else float(np.clip((p - a) @ ab / L2, 0.0, 1.0))
        d = float(np.linalg.norm(p - (a + t * ab)))
        if best is None or d < best - 1e-9:
            best, k, f = d, i, t
    dp = (1 - f) * (new_pts[k][:3, 3] - old_pts[k][:3, 3]) + f * (new_pts[k + 1][:3, 3] - old_pts[k + 1][:3, 3])
    r0 = Rot.from_matrix(new_pts[k][:3, :3] @ old_pts[k][:3, :3].T)
    r1 = Rot.from_matrix(new_pts[k + 1][:3, :3] @ old_pts[k + 1][:3, :3].T)
    dr = Slerp([0.0, 1.0], Rot.concatenate([r0, r1]))([f])[0]
    T = np.eye(4)
    T[:3, :3] = dr.as_matrix() @ T_old[:3, :3]
    T[:3, 3] = p + dp
    return T


def _stroke_poses(block: dict) -> list:
    """The stroke's polyline as the planner takes it: the start pose, then each point of the cleaning sequence in order."""
    from . import alix_sync

    poses, pts = block["cartesian_poses"], block["cleaning_points"]
    order = []
    for pair in block.get("cleaning_sequence") or []:
        for p in pair:
            if not order or order[-1] != p:
                order.append(p)
    return [alix_sync._pose_T(poses["start_pose"])] + [alix_sync._rel_T(poses["start_pose"], pts[p]) for p in order]


def edited(task: str, cfg_path: Path, plan=None) -> tuple:
    """(EditedPlan, notes): ``task``'s plan with its moves following the config at ``cfg_path``."""
    import yaml

    from .. import vendor_paths
    from . import alix_sync, mechanism

    plan = plan or mechanism.cached(task)
    raw = json.loads(plan.path.read_text(encoding="utf-8"))
    root = vendor_paths.ALIX
    old_cfg = yaml.safe_load((root / alix_sync.CONFIGS_REL / f"{task}.yaml").read_text())
    new_cfg = yaml.safe_load(Path(cfg_path).read_text())
    flags = {k: v for k, v in (raw.get("task_flags") or {}).items() if isinstance(v, bool)}
    defaults = alix_sync._defaults(root)
    want_old, want_new = alix_sync.config_stages(old_cfg, flags, defaults), alix_sync.config_stages(new_cfg, flags, defaults)
    names = lambda seq: [(k, x if isinstance(x, str) else str(x.get("stage"))) for k, x in seq]   # noqa: E731
    if names(want_old) != names(want_new):
        raise ValueError(f"{Path(cfg_path).name} has other stages than the plan's config: re-plan it in ALIX, then check the plan")
    moves = [st for k, st in want_new if k == "move"]
    if len(moves) != len(plan.entries):
        raise ValueError(f"the plan has {len(plan.entries)} entries, its config {len(moves)} moves: re-plan it in ALIX")
    pole = {}
    for g in raw.get("track_groups") or []:
        pm = (g.get("pre_stroke") or {}).get("pole_m")
        if pm is not None:
            pole.setdefault("left", float(pm))
            pole.setdefault("right", float(pm))
    out, notes, carry = [], [], {"left": np.zeros(7), "right": np.zeros(7)}
    for e, st in zip(plan.entries, moves):
        if e.kind == "pole":
            pole[e.side] = float(e.last()[0])
        if e.kind != "arm":
            out.append(e)
            continue
        side, stage = e.side, str(st.get("stage"))
        blk_old, blk_new = old_cfg.get(side) or {}, new_cfg.get(side) or {}
        frame = (blk_new.get("task_parameters") or {}).get("ik_frame")
        poses_old, poses_new = blk_old.get("cartesian_poses") or {}, blk_new.get("cartesian_poses") or {}
        rows = np.asarray(e.rows, dtype=float) / 1000.0
        pm = pole.get(side, 0.15)
        d0 = carry[side]
        name = st.get("pose_name")
        if stage == "execute_path" and frame:
            old_pts, new_pts = _stroke_poses(blk_old), _stroke_poses(blk_new)
            if all(np.allclose(a, b) for a, b in zip(old_pts, new_pts)) and len(old_pts) == len(new_pts):
                new_rows = _blend(rows, d0, d0 * 0.0) if np.any(d0) else rows
            else:
                if len(old_pts) != len(new_pts):
                    raise ValueError(f"the {side} stroke has another number of points: re-plan it in ALIX")
                fk, solve = _fk_ref(side, frame, pm), _ik(side, frame, pm)
                keys = list(range(0, len(rows), IK_EVERY)) + ([len(rows) - 1] if (len(rows) - 1) % IK_EVERY else [])
                Told = fk(rows[keys])
                sol, seed, worst = [], rows[0] + d0, 0.0
                for j, k in enumerate(keys):
                    q, ok, mm, deg = solve(_seg_delta(Told[j], old_pts, new_pts), seed)
                    worst = max(worst, mm)
                    sol.append(q)
                    seed = q
                sol = np.asarray(sol)
                new_rows = np.stack([np.interp(np.arange(len(rows)), keys, sol[:, c]) for c in range(7)], axis=1)
                if worst > 2.0:
                    notes.append(f"{side.upper()} {stage}: IK off by up to {worst:.1f} mm (out of reach on this arm configuration?)")
                notes.append(f"{side.upper()} {stage}: its points moved (preview by IK)")
            carry[side] = new_rows[-1] - rows[-1]
        elif name in poses_new and frame:
            p_old, p_new = poses_old.get(name), poses_new[name]
            if p_old == p_new and blk_old.get("cartesian_poses", {}).get("start_pose") == poses_new.get("start_pose"):
                d1 = np.zeros(7)                             # the same pose: the same end
            else:
                Tt = alix_sync._pose_T(p_new) if isinstance(p_new, list) else alix_sync._rel_T(poses_new["start_pose"], p_new)
                q, ok, mm, deg = _ik(side, frame, pm)(Tt, rows[-1])
                if not ok:
                    notes.append(f"{side.upper()} {stage}: IK to the new {name} misses by {mm:.1f} mm, {deg:.2f} deg")
                d1 = q - rows[-1]
                notes.append(f"{side.upper()} {stage}: its {name} moved (preview by IK)")
            new_rows = _blend(rows, d0, d1)
            carry[side] = d1
        else:
            new_rows = _blend(rows, d0, np.zeros(7))
            carry[side] = np.zeros(7)
        out.append(dataclasses.replace(e, rows=np.round(new_rows * 1000.0).astype(int).tolist()))
    return EditedPlan(plan, out, Path(cfg_path)), notes


def check(task: str, cfg_path: Path, back_mm: float | None = None, shift_mm=(0.0, 0.0, 0.0), progress=None) -> dict:
    """The contact forces of ``task`` with its moves following ``cfg_path`` (contact_sim.check_plan's, on the edited moves)."""
    from . import contact_sim

    plan, notes = edited(task, cfg_path)
    times, arms, ends = contact_sim.plan_samples(task, plan)
    updates = [(ends[i], deg) for i, deg in contact_sim.fixture_updates(task, plan, cfg_path) if i < len(ends)]
    out = contact_sim.simulate(task, times, arms, None, back_mm=back_mm, updates=updates, shift_mm=shift_mm, progress=progress)
    out["stages"] = [dict(i=e.i, side=e.side, stage=e.stage, t1=round(t, 3)) for e, t in zip(plan.entries, ends)]
    out["notes"] = notes
    out["config"] = str(cfg_path)
    return out
