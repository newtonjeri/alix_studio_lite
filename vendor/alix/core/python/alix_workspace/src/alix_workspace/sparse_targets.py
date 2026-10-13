"""C16.2 — how few joint targets can carry a free-space stage?

Contract C16.2 (mode 2): MoveIt plans the free space and collision-checks it;
the controller then executes that motion as **3-5 joint-space targets** rather
than a dense stream, while cleaning and other linear motion goes as ``movel``.
The question this module answers is the one that decides whether that is safe:

    between two commanded joint targets the controller moves joint-linearly,
    NOT along MoveIt's path — so how far does the shortcut deviate?

That deviation is the whole risk. A sparse chain is safe exactly when the
deviation stays inside the clearance the dense path had, so the number
computed here is what a clearance check is later compared against. It is a
CONSERVATIVE proxy and needs no fixture, no scene assembly and no hardware.

WHY THIS IS NOT THE APPROACH THE CONTRACT REJECTS. RM_API2's task_config
docstring calls subsampling a saved trajectory "backwards" — correctly, for
CLEANING: movel takes Cartesian poses and the Cartesian path is already in the
config. C16.2 is the complement: joint-space targets for FREE-SPACE motion,
where there is no authored Cartesian path and MoveIt's plan is the only
description of the motion. C16.2 records that distinction as normative.

Re-homed from RM_API2's ``run_hinge_verify.py`` (DENSE vs MODE A vs SPARSE-N)
onto alix's own ``UrdfModel``: that prototype resolves geometry against the
butterfli URDF, which has no ``alix_ref_frame`` and is not this robot's model.

Usage:
    python3 -m alix_workspace.sparse_targets PLAN.json [--targets 2,3,4,5,6]
"""

from __future__ import annotations

import argparse
import json
import math
from typing import Dict, List, Optional, Sequence

import numpy as np

from alix_workspace.urdf_kinematics import UrdfModel

# The stage that is the cleaning stroke, not free-space transit. C16.2 applies
# to everything EXCEPT this one; the stroke stays movel from the config.
STROKE_STAGE = "execute_path"


def load_stages(plan_path: str) -> List[dict]:
    """Arm sub-trajectories of a saved plan, in order.

    Only stages that actually command arm joints are returned — a plan also
    carries pole, hand and scene stages, and those are not this question.
    """
    with open(plan_path) as fh:
        plan = json.load(fh)
    out = []
    for sub in plan.get("sub_trajectories", []):
        names = sub.get("joint_names") or []
        arm = [n for n in names if "_joint" in n and n.split("_joint")[-1].isdigit()]
        if len(arm) < 7:
            continue
        wps = sub.get("waypoints") or []
        if len(wps) < 2:
            continue
        out.append({
            "stage": sub.get("stage_name") or f"#{sub.get('index')}",
            "joint_names": names,
            "positions": [w["positions"] for w in wps],
            "times": [w.get("time_from_start", 0.0) for w in wps],
        })
    return out


def subsample_indices(n_dense: int, n_targets: int) -> List[int]:
    """``n_targets`` evenly spaced indices; first and last always included.

    Same rule as RM_API2's ``subsample`` so the two studies are comparable.
    """
    if n_targets >= n_dense:
        return list(range(n_dense))
    step = (n_dense - 1) / (n_targets - 1)
    return [round(i * step) for i in range(n_targets)]


def _arm_side(joint_names: Sequence[str]) -> Optional[str]:
    for n in joint_names:
        if n.startswith("L_joint"):
            return "L"
        if n.startswith("R_joint"):
            return "R"
    return None


def stage_deviation(model: UrdfModel, stage: dict, n_targets: int,
                    base_link: str = "base_link") -> Optional[dict]:
    """Max deviation of the SPARSE-N shortcut from the dense MoveIt path.

    The sparse chain visits dense waypoints ``i0 < i1 < …`` and moves
    joint-linearly between them. Each dense waypoint between two targets is
    compared against the sparse path at the SAME normalised fraction of that
    leg, which is where a joint-linear move actually is when the dense path is
    at that waypoint. Reported in mm at the tip and degrees per joint.
    """
    side = _arm_side(stage["joint_names"])
    if side is None:
        return None
    tip = f"{side}_ConnectorLink"
    arm_names = [f"{side}_joint{i}" for i in range(1, 8)]
    try:
        idx = [stage["joint_names"].index(n) for n in arm_names]
    except ValueError:
        return None
    chain = model.chain(base_link, tip)
    movable = model.movable_joints(chain)
    mov_names = [jt.name for jt in movable]
    # Where each arm joint sits in the chain's movable-joint vector; joints of
    # the chain the plan does not command (e.g. the pole) hold at zero.
    slot = {n: mov_names.index(n) for n in arm_names if n in mov_names}
    if len(slot) != 7:
        return None

    dense = [[wp[i] for i in idx] for wp in stage["positions"]]
    n_dense = len(dense)
    targets = subsample_indices(n_dense, n_targets)

    def tip_xyz(q7: Sequence[float]) -> np.ndarray:
        q = [0.0] * len(movable)
        for name, val in zip(arm_names, q7):
            q[slot[name]] = val
        return model.fk(chain, q)[:3, 3]

    worst_mm = 0.0
    worst_deg = 0.0
    worst_at = None
    for a, b in zip(targets, targets[1:]):
        if b <= a:
            continue
        qa, qb = np.array(dense[a]), np.array(dense[b])
        for k in range(a, b + 1):
            frac = (k - a) / (b - a)
            q_sparse = qa + frac * (qb - qa)      # joint-linear, as movej moves
            q_dense = np.array(dense[k])
            d_mm = float(np.linalg.norm(tip_xyz(q_dense) - tip_xyz(q_sparse))) * 1e3
            d_deg = float(np.max(np.abs(q_dense - q_sparse))) * 180.0 / math.pi
            if d_mm > worst_mm:
                worst_mm, worst_at = d_mm, k
            worst_deg = max(worst_deg, d_deg)
    return {
        "stage": stage["stage"],
        "side": side,
        "dense_waypoints": n_dense,
        "targets": n_targets,
        "max_tip_dev_mm": worst_mm,
        "max_joint_dev_deg": worst_deg,
        "worst_waypoint": worst_at,
        "duration_s": stage["times"][-1] if stage["times"] else None,
    }


def analyse(plan_path: str, urdf_path: str,
            target_counts: Sequence[int] = (2, 3, 4, 5, 6)) -> List[dict]:
    """Every free-space arm stage of a plan, at each target count."""
    model = UrdfModel.from_file(urdf_path)
    rows = []
    for stage in load_stages(plan_path):
        if stage["stage"] == STROKE_STAGE:
            continue                      # movel from the config, not mode 2
        for n in target_counts:
            if n >= len(stage["positions"]):
                continue
            row = stage_deviation(model, stage, n)
            if row:
                rows.append(row)
    return rows




# ── deviation for an arbitrary target set ────────────────────────────────────

def deviation_for_indices(model: UrdfModel, stage: dict, indices: Sequence[int],
                          base_link: str = "base_link") -> Optional[dict]:
    """Worst tip deviation of the joint-linear chain through `indices`.

    Also reports WHERE the worst point is, which is what curvature-aware
    placement needs: the next target belongs at the deviation, not at the
    midpoint of an index range.
    """
    side = _arm_side(stage["joint_names"])
    if side is None:
        return None
    tip = f"{side}_ConnectorLink"
    arm_names = [f"{side}_joint{i}" for i in range(1, 8)]
    try:
        col = [stage["joint_names"].index(n) for n in arm_names]
    except ValueError:
        return None
    chain = model.chain(base_link, tip)
    movable = model.movable_joints(chain)
    mov_names = [jt.name for jt in movable]
    slot = {n: mov_names.index(n) for n in arm_names if n in mov_names}
    if len(slot) != 7:
        return None
    dense = [[wp[i] for i in col] for wp in stage["positions"]]

    def tip_xyz(q7):
        q = [0.0] * len(movable)
        for name, val in zip(arm_names, q7):
            q[slot[name]] = val
        return model.fk(chain, q)[:3, 3]

    worst_mm, worst_at, worst_leg = 0.0, None, None
    for leg, (a, b) in enumerate(zip(indices, list(indices)[1:])):
        if b <= a:
            continue
        qa, qb = np.array(dense[a]), np.array(dense[b])
        for k in range(a + 1, b):
            frac = (k - a) / (b - a)
            d_mm = float(np.linalg.norm(
                tip_xyz(np.array(dense[k])) - tip_xyz(qa + frac * (qb - qa)))) * 1e3
            if d_mm > worst_mm:
                worst_mm, worst_at, worst_leg = d_mm, k, leg
    return {"max_tip_dev_mm": worst_mm, "worst_at": worst_at, "worst_leg": worst_leg}


def greedy_indices(model: UrdfModel, stage: dict, budget_mm: float = 10.0,
                   max_targets: int = 12, base_link: str = "base_link") -> Optional[dict]:
    """Place targets WHERE THE PATH BENDS, not at even index intervals.

    Start with the two endpoints and repeatedly insert a target at the
    waypoint of worst deviation, which is the split that buys the most. The
    uniform rule spends targets wherever the indices fall — measured (F82),
    N=3 was barely better than N=2 because the index midpoint does not land on
    the bend, and MoveIt's waypoints bunch where the motion is slow, so even
    index spacing is not even spatial spacing either.
    """
    n_dense = len(stage["positions"])
    if n_dense < 2:
        return None
    idx = [0, n_dense - 1]
    dev = deviation_for_indices(model, stage, idx, base_link)
    if dev is None:
        return None
    while dev["max_tip_dev_mm"] > budget_mm and len(idx) < max_targets:
        at = dev["worst_at"]
        if at is None or at in idx:
            break
        idx = sorted(idx + [at])
        dev = deviation_for_indices(model, stage, idx, base_link)
    return {"indices": idx, "targets": len(idx),
            "max_tip_dev_mm": dev["max_tip_dev_mm"],
            "meets_budget": dev["max_tip_dev_mm"] <= budget_mm}


# ── C16.2 target selection ───────────────────────────────────────────────────

def select_targets(model: UrdfModel, stage: dict, budget_mm: float = 10.0,
                   max_targets: int = 12, base_link: str = "base_link") -> Optional[dict]:
    """The FEWEST targets whose shortcut stays inside `budget_mm`.

    C16.2 says "3 to 5". Measured (F82) that is right at the top of its range
    and wrong at the bottom, and the answer is per stage — so this searches
    rather than assuming, and REPORTS FAILURE rather than returning a number
    that does not meet the budget. A stage that cannot meet it inside
    `max_targets` is one where the dense chain (Mode B) is worth its cost.
    """
    n_dense = len(stage["positions"])
    for n in range(2, min(max_targets, n_dense) + 1):
        row = stage_deviation(model, stage, n, base_link=base_link)
        if row is None:
            return None
        if row["max_tip_dev_mm"] <= budget_mm:
            idx = subsample_indices(n_dense, n)
            return {
                "stage": stage["stage"],
                "targets": n,
                "indices": idx,
                "max_tip_dev_mm": row["max_tip_dev_mm"],
                "max_joint_dev_deg": row["max_joint_dev_deg"],
                "budget_mm": budget_mm,
                "meets_budget": True,
                # Chained: connect=1 queues each target and the closing
                # connect=0 executes the chain as ONE motion with a single
                # arrival event. N unchained targets buy N-1 full stops
                # (ProfiledPositionController's stop->replan->go), which is
                # exactly what C4's chained geometry exists to avoid.
                "connect": [1] * (n - 1) + [0],
                "joint_targets": [stage["positions"][i] for i in idx],
                "joint_names": stage["joint_names"],
            }
    row = stage_deviation(model, stage, min(max_targets, n_dense), base_link=base_link)
    return {
        "stage": stage["stage"], "targets": None, "indices": [],
        "max_tip_dev_mm": row["max_tip_dev_mm"] if row else None,
        "budget_mm": budget_mm, "meets_budget": False, "connect": [],
        "joint_targets": [], "joint_names": stage["joint_names"],
    }


def reduce_plan(plan_path: str, urdf_path: str, budget_mm: float = 10.0,
                max_targets: int = 12) -> List[dict]:
    """Every free-space arm stage of a plan, reduced to its own target count."""
    model = UrdfModel.from_file(urdf_path)
    out = []
    for stage in load_stages(plan_path):
        if stage["stage"] == STROKE_STAGE:
            continue
        sel = select_targets(model, stage, budget_mm, max_targets)
        if sel:
            out.append(sel)
    return out


def _default_urdf() -> str:
    from pathlib import Path
    return str(Path(__file__).resolve().parent / "urdf" / "alix.urdf")


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("plan")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--targets", default="2,3,4,5,6",
                    help="comma-separated target counts to evaluate")
    args = ap.parse_args(argv)
    counts = [int(x) for x in args.targets.split(",") if x.strip()]
    rows = analyse(args.plan, args.urdf or _default_urdf(), counts)
    if not rows:
        print("no free-space arm stages in this plan")
        return 1
    print(f"{'stage':<28}{'side':<6}{'wps':>5}{'N':>4}"
          f"{'max tip dev (mm)':>18}{'max joint dev (deg)':>21}")
    for r in rows:
        print(f"{r['stage']:<28}{r['side']:<6}{r['dense_waypoints']:>5}"
              f"{r['targets']:>4}{r['max_tip_dev_mm']:>18.1f}"
              f"{r['max_joint_dev_deg']:>21.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
