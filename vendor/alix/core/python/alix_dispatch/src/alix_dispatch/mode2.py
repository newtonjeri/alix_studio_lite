"""C16.2 — MoveIt plans the free space; the controller executes it as a few
joint targets. ROS-free.

Mode 2 differs from mode 1 in exactly one respect, and it is worth stating
narrowly: **how a FREE-SPACE arm move is expressed.** Everything else — tool
frame, pole, hand, the cleaning stroke, the order — is mode 1's, unchanged and
built by mode 1's own builder, because a second opinion about any of it would
be a second authority.

    mode 1   move_to_rest  ->  one `movej` to the named state
    mode 2   move_to_rest  ->  N joint targets lifted from MoveIt's plan for
                               that stage, issued as ONE CHAIN

WHY N TARGETS AND NOT THE DENSE PATH. Between two commanded joint targets the
controller moves joint-linearly, NOT along MoveIt's path, so the reduction has
a deviation and `sparse_targets` measures it. N is chosen per stage as the
FEWEST targets whose deviation fits the budget — measured (F82) at 5 for 59 of
80 stages and 3 for only 17, so a constant N would be wrong at both ends.

WHY A CHAIN AND NOT N GOALS. `connect=1` queues each target and the closing
`connect=0` executes the lot as one motion with a single arrival event. N
unchained targets buy N-1 full stops (stop -> replan -> go), which is exactly
what C4's chained geometry exists to avoid.

WHAT IT WILL NOT TOUCH, and this is the part that bites:

* **`movel` stages stay `movel`.** `move_to_start` is issued linear on purpose
  (plan/DECISIONS: the linearity-critical legs get no planner — they are a
  Cartesian line), and joint targets through it would bow the approach. The
  rule is mechanical rather than a name list: mode 2 reduces exactly the calls
  mode 1 issues as `movej`/`movej_p`, and leaves every Cartesian one alone.
* **the cleaning stroke stays `movel_chain`.** C16.2 is about free space; the
  Cartesian path for cleaning is already authored in the config, and RM_API2's
  own note calls subsampling that "backwards".
* **a stage with no plan sub-trajectory is NOT silently reduced or silently
  skipped** — it falls back to mode 1's call and says so in `warnings` (F81:
  a skip is a verdict, never a default).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

from alix_dispatch import program as progmod
from alix_dispatch.program import Call, DispatchError, Program

# Mode 1 kinds that describe a FREE-SPACE move — the only ones mode 2 rewrites.
REDUCIBLE_KINDS = ("movej", "movej_p")


class Mode2Error(DispatchError):
    """The plan cannot carry this program's free space."""


@dataclass
class StageReduction:
    stage: str
    targets: int
    indices: List[int]
    max_tip_dev_mm: float
    budget_mm: float
    placement: str                 # "greedy" | "even"
    joint_targets: List[List[float]]
    joint_names: List[str]
    dense: int

    @property
    def saving(self) -> float:
        return 1.0 - (self.targets / float(self.dense))


def _plan_stages(plan_path: str) -> Dict[str, dict]:
    """Free-space arm sub-trajectories of a saved plan, keyed by stage name.

    A plan can carry the same stage name twice (`move_to_pre_start` runs on the
    way out and on the way back). Keeping only one would silently reduce the
    second leg with the first leg's targets, so repeats are refused rather than
    collapsed — mode 1 falls back for them and says so.
    """
    from alix_workspace.sparse_targets import load_stages
    out: Dict[str, dict] = {}
    duplicated = set()
    for stage in load_stages(plan_path):
        name = stage["stage"]
        if name in out:
            duplicated.add(name)
            continue
        out[name] = stage
    for name in duplicated:
        out.pop(name, None)
    out["__duplicated__"] = sorted(duplicated)          # type: ignore[assignment]
    return out


def reduce_stage(model, stage: dict, *, budget_mm: float, max_targets: int,
                 placement: str = "greedy") -> Optional[StageReduction]:
    """The fewest joint targets that carry this stage inside `budget_mm`.

    `placement="greedy"` puts targets WHERE THE PATH BENDS rather than at even
    index intervals — measured 22 % fewer targets for the same budget (F87).
    Even spacing is kept selectable because it is what `select_targets` has
    always used and what F82's numbers were taken with.
    """
    from alix_workspace import sparse_targets as st

    dense = len(stage["positions"])
    if placement == "greedy":
        # Both helpers return a DICT, not a bare value — `greedy_indices` gives
        # {indices, targets, max_tip_dev_mm, meets_budget}.
        g = st.greedy_indices(model, stage, budget_mm=budget_mm,
                              max_targets=max_targets)
        if g and g.get("meets_budget") and g.get("indices"):
            idx = list(g["indices"])
            return StageReduction(
                stage["stage"], len(idx), idx, float(g["max_tip_dev_mm"]),
                budget_mm, "greedy", [stage["positions"][i] for i in idx],
                list(stage["joint_names"]), dense)
    sel = st.select_targets(model, stage, budget_mm, max_targets)
    if not sel or not sel.get("meets_budget"):
        return None
    return StageReduction(
        sel["stage"], int(sel["targets"]), list(sel["indices"]),
        float(sel["max_tip_dev_mm"]), budget_mm, "even",
        [list(p) for p in sel["joint_targets"]], list(sel["joint_names"]), dense)


def _arm_joint_columns(joint_names: Sequence[str], side: str) -> List[int]:
    """Indices of that arm's 7 joints, in joint order, within a plan row.

    A plan row can carry pole and hand columns too. Commanding those to the arm
    would be a 14-value `movej`, which the SDK takes as a length error at best.
    """
    prefix = "L" if side == "left" else "R"
    want = [f"{prefix}_joint{i}" for i in range(1, 8)]
    cols = []
    for name in want:
        if name not in joint_names:
            raise Mode2Error(
                f"plan stage does not carry {name}; its joint names are "
                f"{list(joint_names)}")
        cols.append(list(joint_names).index(name))
    return cols


def build_program(cfg, states, model, plan_path: str, *,
                  budget_mm: float = 10.0, max_targets: int = 12,
                  placement: str = "greedy",
                  blend_pct: Optional[int] = None,
                  pole_start_m: Optional[float] = None) -> Program:
    """A mode-1 program with its free-space arm moves replaced by joint chains."""
    import math

    base = progmod.build_program(cfg, states, model, blend_pct=blend_pct,
                                 pole_start_m=pole_start_m)
    stages = _plan_stages(plan_path)
    duplicated = stages.pop("__duplicated__", [])       # type: ignore[arg-type]

    calls: List[Call] = []
    warnings = list(base.warnings)
    reductions: List[StageReduction] = []
    reduced_stages = set()

    for call in base.calls:
        if call.device != "arm" or call.kind not in REDUCIBLE_KINDS:
            calls.append(call)
            continue

        side = call.arm or base.side
        plan_stage = stages.get(call.stage)
        if plan_stage is None:
            why = ("appears more than once in the plan" if call.stage in duplicated
                   else "has no arm sub-trajectory in the plan")
            warnings.append(
                f"mode2: {call.stage!r} {why} — issued as mode 1 ({call.kind}). "
                "NOT reduced and NOT skipped.")
            calls.append(call)
            continue

        red = reduce_stage(model, plan_stage, budget_mm=budget_mm,
                           max_targets=max_targets, placement=placement)
        if red is None:
            warnings.append(
                f"mode2: {call.stage!r} cannot meet the {budget_mm} mm budget "
                f"within {max_targets} targets — issued as mode 1 ({call.kind}). "
                "This is a stage where the dense chain (Mode B) earns its cost.")
            calls.append(call)
            continue

        cols = _arm_joint_columns(red.joint_names, side)
        speed = call.detail.get("speed_pct")
        moves = []
        for k, row in enumerate(red.joint_targets):
            moves.append({
                "joints_deg": [math.degrees(row[c]) for c in cols],
                "v_pct": speed,
                "r": call.detail.get("r"),
                # connect=1 queues; the closing 0 executes the chain as ONE
                # motion with a single arrival event.
                "connect": 0 if k == len(red.joint_targets) - 1 else 1,
                "index": red.indices[k],
            })
        calls.append(Call(call.stage, "arm", "movej_chain", {
            "moves": moves,
            "targets": red.targets,
            "dense": red.dense,
            "max_tip_dev_mm": red.max_tip_dev_mm,
            "budget_mm": red.budget_mm,
            "placement": red.placement,
            "speed_pct": speed,
            "replaces": call.kind,
        }, arm=call.arm))
        reductions.append(red)
        reduced_stages.add(call.stage)

    if not reductions:
        warnings.append(
            "mode2: NOTHING was reduced — every free-space stage fell back to "
            "mode 1. This program is mode 1 with extra steps; check the plan "
            "matches the config's stage names.")

    prog = Program(base.task, base.side, calls, warnings)
    prog.mode = 2                                        # type: ignore[attr-defined]
    prog.reductions = reductions                         # type: ignore[attr-defined]
    prog.plan_path = plan_path                           # type: ignore[attr-defined]
    return prog


def describe(prog) -> str:
    """One line per reduced stage — what was replaced and at what deviation."""
    reds = getattr(prog, "reductions", [])
    if not reds:
        return "mode 2: no stage reduced"
    out = [f"mode 2: {len(reds)} stage(s) reduced from MoveIt's plan"]
    for r in reds:
        out.append(f"  {r.stage:<24} {r.dense:>5} waypoints -> {r.targets} targets "
                   f"({r.placement}, {100*r.saving:.1f}% fewer)  "
                   f"deviation {r.max_tip_dev_mm:.2f} mm / {r.budget_mm:.0f} mm budget")
    return "\n".join(out)
