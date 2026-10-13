"""The APPROACH TRANSIT gate: the joint sweeps rest -> pre_start -> start.

G43, measured live. The certified `seat_ring_bottom_right` config died at

    move_to_pre_start/joint — Waypoint is in collision!
    R_Link5 <-> washroom_structure

The straight joint-space sweep from rest to the approach solution crosses the
washroom wall on a 157 deg J1 backhand windup. The gate had certified the
ENDPOINTS only, so it never looked at the class at all, and OMPL could not
rescue it at its 5 s default either. The C++ approach stages are `MoveTo` with a
`joint -> pipeline` Fallbacks chain: the joint leg is exactly this lerp.

WHY THIS MODULE EXISTS RATHER THAN THE FUNCTIONS STAYING IN `sdk_reach`.
Those primitives were reachable only from the LEGACY reach oracle
(`sdk_reach.sdk_reach_check`, called by `auto_generate` and the legacy CLI), and
the C-space route called none of them -- MEASURED 2026-09-16 by grep: no C-space
caller for `_transit_blocked`, `_transit_corridor_witness` or
`_stage_pipeline_budget` anywhere in the tree. So the rule that exists because it
killed the first certified config on the live stack was about to be deleted with
the generator it happened to live next to. It is route-agnostic arithmetic over
(seed, solutions, pose_filter); it belongs where both routes can reach it.

UNITS. Everything here is DEGREES, which is what `BodyCollisionWorld.make_pose_filter`
takes (the Algo convention) and what `TRANSIT_STEP_DEG` is a step of. Callers
holding radians -- `stroke_gate._rest_state`, `predict_start`'s solutions, the
`entry.pre_start.q7` on a generated path -- convert at the boundary. Getting this
wrong does not raise: it samples the wrong number of points along the right line
and reports a clean sweep.
"""

from __future__ import annotations

import math
from typing import Any, Callable, Dict, List, Optional, Sequence


#: Joint-space sampling step for the approach TRANSIT check (deg). The C++
#: 'joint' fallback interpolates straight in joint space and collision-checks
#: its waypoints; 2 deg per sample resolves every sweep the live stack has
#: named (Link3/Link5 arcs are tens of degrees wide).
TRANSIT_STEP_DEG = 2.0

#: Minimum pipeline budget (s) for a witnessed corridor to count as plannable --
#: OMPL failed the live 157 deg windup at the 5 s default.
TRANSIT_PIPELINE_BUDGET_S = 15.0

#: The approach transit legs, in order: (target pose, leg id, from-label,
#: runtime stage). The LEG -- not a substring of the printable reason -- is what
#: attributes a blockage to a stage. Prose-matching could never select
#: `move_to_start`, because the second leg's message ALSO contains
#: "pre_start_pose", so every descent-into-contact blockage was priced against
#: move_to_pre_start's budget and certified rescued (Newton, 2026-08-20).
TRANSIT_LEGS = (
    ("pre_start_pose", "rest->pre_start", "rest", "move_to_pre_start"),
    ("start_pose", "pre_start->start", "pre_start_pose", "move_to_start"),
)


def _steps(a: Sequence[float], b: Sequence[float]) -> int:
    """Interior samples for the lerp a -> b, at least one.

    CHANGED, not moved (2026-09-16). The rule as inherited was
    ``max(1, int(span / TRANSIT_STEP_DEG))`` over ``range(1, steps)``, which
    gives a leg of span < 2 x the step ZERO interior samples -- `int` truncates
    to 1 and `range(1, 1)` is empty. A 3.9 deg sweep was therefore never looked
    at. The endpoints are checked elsewhere (`predict_start` solves both under
    the world), so the risk was small, but "small" is not the same as "checked"
    and this gate exists because an unchecked sweep reached the live stack.
    `ceil` with a floor of 2 guarantees at least the midpoint on every leg.
    """
    span = max(abs(x - y) for x, y in zip(a, b))
    return max(2, int(math.ceil(span / TRANSIT_STEP_DEG)))


class TransitBlockage(str):
    """The printable reason PLUS the leg identity that produced it.

    A `str` subclass so every consumer keeps working unchanged -- the truthiness
    test, the `transitBlocked` string in a reach dict, the gate's sentence --
    while stage attribution reads `.stage` / `.leg` instead of re-parsing prose
    (Newton, 2026-08-20).
    """

    leg: str
    stage: str

    def __new__(cls, text: str, leg: str, stage: str) -> "TransitBlockage":
        obj = super().__new__(cls, text)
        obj.leg = leg
        obj.stage = stage
        return obj


def transit_blocked(seed: Sequence[float], solutions: Dict[str, Sequence[float]],
                    pose_filter: Callable[[List[float]], Any]) -> Optional[TransitBlockage]:
    """First blocked sample on the rest->pre_start->start JOINT transits, or None.

    *seed*, *solutions* and whatever *pose_filter* takes are all DEGREES.
    """
    legs, prev = [], list(seed)
    for name, leg_id, frm, stage in TRANSIT_LEGS:
        sol = solutions.get(name)
        if sol is None:
            return None       # the approach itself failed; the transit is moot
        legs.append((name, leg_id, frm, stage, prev, list(sol)))
        prev = list(sol)
    for name, leg_id, frm, stage, a, b in legs:
        steps = _steps(a, b)
        for k in range(1, steps):
            s = k / steps
            q = [x + s * (y - x) for x, y in zip(a, b)]
            pairs = pose_filter(q)
            if pairs:
                return TransitBlockage(
                    f"approach transit {frm} -> {name} blocked at joint "
                    f"sample {k}/{steps} — {', '.join(str(p) for p in pairs[:3])}",
                    leg_id, stage)
    return None


def corridor_witness(seed: Sequence[float], solutions: Dict[str, Sequence[float]],
                     pose_filter: Callable[[List[float]], Any]) -> Optional[Dict[str, float]]:
    """A collision-free joint corridor rest -> pre_start, or None (G43).

    When the straight joint lerp is blocked, the runtime's pipeline leg (OMPL)
    must ROUTE around -- nondeterministic, so the gate does not trust it blindly:
    it certifies that a corridor EXISTS by exhibiting one from the tuck family
    (fold the elbow in tighter, wind J1 up at the fold's small sweep radius,
    extend). Measured on seat_ring_bottom_right's 157 deg backhand windup: rest
    fold blocked, J2 -43 deg / J4 +32 deg fold ALL CLEAN.
    """
    pre = solutions.get("pre_start_pose")
    if pre is None:
        return None
    pre = list(pre)
    seed = list(seed)

    def clean_lerp(a, b):
        steps = _steps(a, b)
        for k in range(1, steps):
            s = k / steps
            if pose_filter([x + s * (y - x) for x, y in zip(a, b)]):
                return False
        return True

    for dj2 in (0.0, -13.0, -28.0, -43.0):
        for dj4 in (0.0, 17.0, 32.0):
            fold0 = [seed[0], seed[1] + dj2, seed[2], seed[3] + dj4,
                     seed[4], seed[5], seed[6]]
            fold1 = [pre[0]] + fold0[1:]
            if pose_filter(fold0) or pose_filter(fold1):
                continue
            if (clean_lerp(seed, fold0) and clean_lerp(fold0, fold1)
                    and clean_lerp(fold1, pre)):
                return {"fold_dj2": dj2, "fold_dj4": dj4}
    return None


def stage_pipeline_budget(config: Dict[str, Any], stage_name: str) -> float:
    """The pipeline-leg timeout (s) the config grants *stage_name*, or 0."""
    for st in config.get("stage_sequence") or []:
        if str(st.get("stage")) != stage_name:
            continue
        chain = st.get("planner") or []
        if isinstance(chain, str):
            chain = [chain]
        timeouts = st.get("planner_timeouts") or []
        for i, token in enumerate(chain):
            if str(token).split(":")[0] in ("pipeline", "ompl"):
                try:
                    return float(timeouts[i])
                except (IndexError, TypeError, ValueError):
                    return 5.0          # C++ kPipelineSolverTimeout default
    return 0.0


def transit_verdict(config: Dict[str, Any], seed_deg: Sequence[float],
                    solutions_deg: Dict[str, Sequence[float]],
                    pose_filter: Optional[Callable[[List[float]], Any]]
                    ) -> Dict[str, Any]:
    """``{blocked, rescued, leg, stage, ok}`` for this config's approach.

    G43's RULING 2 (Newton, 2026-08-20), applied unchanged: only
    ``rest->pre_start`` can be rescued, for two independent and individually
    binding reasons.

    1. A declared ``planner:`` chain is an OPT-OUT of the Cartesian line
       (dual_arm_cleaning_task.cpp). The pipeline rescue and the straight line
       are therefore mutually exclusive on a leg: ``move_to_start`` leaves
       ``planner:`` unset precisely because that descent is linearity-critical,
       so it gets the line and has NO pipeline leg to route around a blockage.
       A blockage there must fail honestly.
    2. `corridor_witness` only certifies a corridor for rest -> pre_start; it
       proves nothing about pre_start -> start, so it must not be consulted for
       that leg.

    With no *pose_filter* there is no world, so there is no verdict: ``ok`` is
    None rather than True. "The world was not loaded" is not evidence that a
    sweep is clear -- that is the same mistake `auto_generate._reach_gate`'s
    "reach NOT MEASURED" rung exists to refuse.
    """
    if pose_filter is None:
        return {"ok": None, "blocked": None, "rescued": None,
                "why": "no collision world: the approach transit was not checked"}
    blocked = transit_blocked(seed_deg, solutions_deg, pose_filter)
    if not blocked:
        return {"ok": True, "blocked": None, "rescued": None}
    stage = blocked.stage
    budget = stage_pipeline_budget(config, stage)
    witness = None
    if blocked.leg == "rest->pre_start":
        witness = corridor_witness(seed_deg, solutions_deg, pose_filter)
    if witness is not None and budget >= TRANSIT_PIPELINE_BUDGET_S:
        return {"ok": True, "blocked": str(blocked), "leg": blocked.leg, "stage": stage,
                "rescued": {"witness": witness, "pipelineBudgetS": budget, "stage": stage}}
    why = str(blocked)
    if blocked.leg == "rest->pre_start":
        why += (f" — and no corridor witness + pipeline budget rescue (witness "
                f"{'found' if witness else 'NOT found'}, budget {budget:g} s, needs "
                f"{TRANSIT_PIPELINE_BUDGET_S:g} s)")
    else:
        why += (" — this leg is linearity-critical: it leaves `planner:` unset and gets the "
                "Cartesian line, so it has no pipeline leg to route around a blockage")
    return {"ok": False, "blocked": str(blocked), "leg": blocked.leg, "stage": stage,
            "rescued": None, "pipelineBudgetS": budget, "why": why}
