"""Dual-arm execution modes for mode 1 — locked, chained, free. ROS-free.

WHY THIS EXISTS. Until now `MultiArmBackend` ran every two-arm program strictly
sequentially, because we held no dual-arm evidence of our own. We do hold it —
it was collected on hardware and then not carried across. The record, from
`RM_API2/.../RMDemo_DualArmConcept` (read 2026-08-20):

  C1  `test_dual_connect`  9 PASS / 0 FAIL (x3). Two RM_75 gen-3 controllers,
      distinct handles (left id=1, right id=2), ONE process-global arrival
      callback demuxed by handle_id.
  C2  `test_dual_locked`   7 PASS / 0 FAIL, REAL, 2026-08-08 19:00. Full 7-step
      sequence on BOTH arms, barrier per step. Max dispatch skew 27.45 ms;
      arrival event for every device.
  C3  `test_dual_chained`  ordering invariant PASS, both chains completed,
      2026-08-08 22:13. Event-gated follower latency 0.2-10.8 ms. (Its one FAIL
      is our own polling on HAND steps — device 2 emits no arrival event, F4 —
      not arm behaviour; PHASE_PLAN closes C3 on this data.)
  C4  `test_dual_free`     4 PASS / 0 FAIL, REAL. Both arms 4/4 steps in
      11.71 s / 11.70 s, window overlap 100 % of the shorter run. Run
      `--no-hands --no-pole`, so free mode is proven ARM-ONLY.

So concurrency ACROSS the two controllers is proven and unrestricted. What is
NOT licensed by C2 is same-arm arm+pole overlap: those sync steps ran the arm
half on CANFD, and `RM_SYNC_BACKEND=planned` is hard-locked behind
`RM_UNLOCK_PLANNED_SYNC=1` (F9, vendor-confirmed gen-3, no fix ETA). Reading
"C2 locked passed with sync steps" as "F9 is dead" is the trap; F9 is alive and
`program.enforce_device_serialization` still runs per arm inside every mode
here. PHASE_PLAN 5.4 states it directly: *"If arm and pole must move together,
the arm must be in the CANFD stream. That is the only combination the
controller supports."* — which is mode 3, not this one.

WHAT THIS MODULE WILL NOT DO. It schedules two INDEPENDENT single-arm programs.
It does not parallelise a COMPOUND config (one `stage_sequence` with per-stage
`arm:`), and that refusal is deliberate, not unfinished work — see
`plan_compound_refusal` for the counterexample that decides it.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from alix_dispatch.program import Program, enforce_device_serialization

MODES = ("sequential", "locked", "chained", "free")

# The orchestrator already HAS an execution grammar, and it is the authority:
# `alix_orchestrator/config/schedule_commode.csv`, parsed by
# `core/schedule_loader.py` into `type = single | paired | serial | chained |
# compound`. Newton, 2026-08-20: *"we do not only pair the symmetric tasks."*
# He is right and the CSV proves it — row 6 is `10R,3L` (bottomlid RIGHT with
# seat-ring LEFT) and row 7 is the swap `3R,10L`. A paired row is TWO DIFFERENT
# REGIONS, so there is nothing to align stage-for-stage and nothing that should
# try: `paired` means run both to completion, concurrently.
#
#   CSV type   meaning                                      mode here
#   single     one arm                                      (build one program)
#   paired     `11R,11L` / `10R,3L` — concurrent            free
#   serial     `13R,>13L` — col1 finishes first             sequential + leader
#   chained    `0R>1R,1L>0L` — per-arm REGION chains, run   plan_region_chains
#              concurrently and counter-phased
#   compound   `2C` — one config drives both arms           custody.py
#
# `locked` (barrier per rendezvous) has no CSV row today. It is kept because it
# is what RM_API2 C2 actually proved, and it is the mode to reach for if a
# future row needs the two arms stepping together.
CSV_TYPES = {"paired": "free", "serial": "sequential", "single": "sequential"}


class ScheduleError(RuntimeError):
    """The requested dual-arm schedule cannot be issued safely."""


@dataclass
class Step:
    """One config stage for one arm — the unit a cross-arm gate acts on.

    A gate can only open on a whole stage: `connect=1` returns IMMEDIATELY even
    in blocking mode, so the only point at which an arm is known to be idle is
    after a stage whose chain closed with `connect=0`.
    """
    arm: str
    stage: str
    calls: List = field(default_factory=list)


@dataclass
class Phase:
    """What one arm does between two rendezvous points, inclusive of the second."""
    index: int
    steps: List[Step] = field(default_factory=list)

    @property
    def stages(self) -> List[str]:
        return [s.stage for s in self.steps]


@dataclass
class DualPlan:
    mode: str
    arms: Tuple[str, str]              # (leader, follower) — config/caller order
    phases: Dict[str, List[Phase]]     # per arm, same length for every arm
    rendezvous: List[str]              # stage name each phase ends on ("" = tail)
    warnings: List[str] = field(default_factory=list)

    @property
    def n_phases(self) -> int:
        return len(self.rendezvous)

    def summary(self) -> str:
        lead, follow = self.arms
        rows = []
        for i in range(self.n_phases):
            rows.append(f"  {i}: {lead}={self.phases[lead][i].stages} "
                        f"{follow}={self.phases[follow][i].stages} "
                        f"@{self.rendezvous[i] or '<tail>'}")
        return f"{self.mode} over {self.arms}\n" + "\n".join(rows)


# ── cross-arm safety ─────────────────────────────────────────────────────────

@dataclass(frozen=True)
class CrossArmClearance:
    """Evidence that the two arms do not meet while running concurrently.

    Deliberately NOT defaulted and NOT computed here. R6 (`PHASE_PLAN` risk
    register) is open: "Dual-arm concurrent collision validity — locked/free
    modes could collide; Phase 2 scoped to one arm." Every single-arm guarantee
    we hold — the segment verifier, the rehearsal loop, the FCL scene — models
    ONE arm against the fixture. None of them has ever asked whether the left
    arm's swept volume meets the right arm's.

    The margin threshold is the caller's to state (C16.4: no execution
    parameter from a code default), and `source` must say what produced it so a
    green run is attributable.

    **`drift_window_s` is not optional bookkeeping — it is half the verdict.**
    A separation figure is a statement about a PHASING: these two arms, at these
    two points in their plans. Measured 2026-08-20 with every arm link, the
    unbounded figure (any phasing at all) is 0.00 mm for every concurrent pair
    on this commode — both arms reach over one small fixture and `L_Link3` /
    `R_Link4` sweep through each other at some offset. So a number without the
    phasing it assumes is not a verdict, and

        drift_window_s = None  ->  the figure holds for ANY relative phasing
        drift_window_s = X     ->  it holds only while the two arms stay within
                                   X seconds of their nominal phasing

    and only a mode that GATES can hold a bound: `locked` re-synchronises at
    every rendezvous, `chained` at every phase, `free` never does. A bounded
    verdict handed to free mode is therefore refused.
    """
    min_separation_mm: float
    threshold_mm: float
    source: str
    drift_window_s: Optional[float] = None

    @property
    def clear(self) -> bool:
        return self.min_separation_mm >= self.threshold_mm

    @property
    def holds_for_any_phasing(self) -> bool:
        return self.drift_window_s is None


def _require_clearance(mode: str, clearance: Optional[CrossArmClearance]) -> None:
    if mode == "sequential":
        return
    if clearance is None:
        raise ScheduleError(
            f"mode {mode!r} runs both arms at once and no CrossArmClearance was "
            "supplied. Cross-arm collision validity is R6 — OPEN. Every check we "
            "have (segment verifier, rehearsal loop, FCL scene) models one arm "
            "against the fixture, never arm against arm, and alix.srdf leaves 398 "
            "cross-side pairs deliberately enabled precisely so this question is "
            "not answered by default. Supply the verdict and own it, or use "
            "mode='sequential'.")
    if not clearance.clear:
        raise ScheduleError(
            f"mode {mode!r} refused: cross-arm separation "
            f"{clearance.min_separation_mm:.1f} mm < threshold "
            f"{clearance.threshold_mm:.1f} mm (source: {clearance.source})")
    if mode == "free" and not clearance.holds_for_any_phasing:
        raise ScheduleError(
            f"mode 'free' refused: the clearance verdict holds only within "
            f"{clearance.drift_window_s} s of nominal phasing, and free mode has "
            "no gate that can hold it — the two arms advance on their own "
            "arrivals and drift without bound. Use 'locked' (re-synchronises at "
            "every rendezvous) or 'chained' (at every phase), or supply a "
            "verdict measured with no drift window.")


# ── planning ─────────────────────────────────────────────────────────────────

def _steps(program: Program) -> List[Step]:
    """Group a single-arm program's calls into per-stage steps, in order.

    `config` calls (tool-frame selection) ride with the stage that follows them
    — they are setup for it, and splitting them across a gate would select a
    tool frame on one side of a barrier and use it on the other. `scene` calls
    command no device and are dropped: they are planner/collision bookkeeping,
    not controller traffic.
    """
    steps: List[Step] = []
    pending: List = []
    for call in program.calls:
        if call.device == "scene":
            continue
        if call.device == "config":
            pending.append(call)
            continue
        steps.append(Step(program.side, call.stage, pending + [call]))
        pending = []
    if pending:
        raise ScheduleError(
            f"{program.task}: trailing configuration call(s) "
            f"{[c.stage for c in pending]} with no motion stage to attach to")
    return steps


def _lcs(a: Sequence[str], b: Sequence[str]) -> List[Tuple[int, int]]:
    """Index pairs of a longest common subsequence of two stage-name lists.

    The two arms' configs are NOT symmetric — `toplid_right` puts
    `allow_collision` before `move_to_pre_start` where `toplid_left` puts it
    after, and carries a second `move_to_pre_start` after the stroke — so an
    index-for-index barrier (which is what RM_API2's locked mode assumes, since
    both arms there run ONE shared sequence) would pair unrelated stages. The
    common subsequence gives real rendezvous points and leaves the unmatched
    stages to run inside a phase, un-gated.
    """
    n, m = len(a), len(b)
    table = [[0] * (m + 1) for _ in range(n + 1)]
    for i in range(n - 1, -1, -1):
        for j in range(m - 1, -1, -1):
            table[i][j] = (table[i + 1][j + 1] + 1 if a[i] == b[j]
                           else max(table[i + 1][j], table[i][j + 1]))
    out: List[Tuple[int, int]] = []
    i = j = 0
    while i < n and j < m:
        if a[i] == b[j]:
            out.append((i, j))
            i += 1
            j += 1
        elif table[i + 1][j] >= table[i][j + 1]:
            i += 1
        else:
            j += 1
    return out


def _phases(steps: List[Step], cuts: List[int], n_phases: int) -> List[Phase]:
    """Slice one arm's steps at `cuts` (inclusive ends), tail into the last phase."""
    phases: List[Phase] = []
    start = 0
    for idx, cut in enumerate(cuts):
        phases.append(Phase(idx, steps[start:cut + 1]))
        start = cut + 1
    while len(phases) < n_phases - 1:
        phases.append(Phase(len(phases), []))
    phases.append(Phase(len(phases), steps[start:]))
    return phases


def plan_dual(programs: Dict[str, Program], mode: str = "sequential", *,
              clearance: Optional[CrossArmClearance] = None,
              leader: Optional[str] = None) -> DualPlan:
    """Schedule two INDEPENDENT single-arm programs across the two controllers.

    `programs` maps side -> Program, e.g. {"left": toplid_left, "right":
    toplid_right}. Each program keeps its own order exactly; this only decides
    what the two arms may do at the same time.
    """
    if mode not in MODES:
        raise ScheduleError(f"unknown mode {mode!r}; expected one of {MODES}")
    if len(programs) != 2:
        raise ScheduleError(
            f"a dual plan needs exactly two arms, got {sorted(programs)}")
    for side, prog in programs.items():
        if prog.side == "dual":
            raise ScheduleError(
                f"{prog.task} is a COMPOUND task (one stage_sequence, per-stage "
                "`arm:`) — it already sequences both arms and must not be fed to "
                "plan_dual. See plan_compound_refusal().")
        if prog.side != side:
            raise ScheduleError(
                f"program {prog.task!r} is for the {prog.side} arm but was keyed "
                f"as {side!r}")

    warnings: List[str] = []
    # F9 is per controller and stays per controller: assert each arm's own
    # stream is serialised BEFORE allowing the two streams to overlap.
    for side, prog in programs.items():
        ok, _, problems = enforce_device_serialization(prog)
        if not ok:
            raise ScheduleError(
                f"{side} arm violates F9 device serialization on its own; "
                f"running it concurrently with the other arm cannot fix that: "
                f"{problems}")

    _require_clearance(mode, clearance)

    sides = list(programs)
    if leader is not None:
        if leader not in programs:
            raise ScheduleError(f"leader {leader!r} is not one of {sides}")
        sides = [leader] + [s for s in sides if s != leader]
    lead, follow = sides[0], sides[1]

    steps = {s: _steps(programs[s]) for s in sides}
    pairs = _lcs([st.stage for st in steps[lead]], [st.stage for st in steps[follow]])
    rendezvous = [steps[lead][i].stage for i, _ in pairs] + [""]
    phases = {
        lead: _phases(steps[lead], [i for i, _ in pairs], len(rendezvous)),
        follow: _phases(steps[follow], [j for _, j in pairs], len(rendezvous)),
    }

    unmatched = {s: sum(len(p.steps) for p in phases[s]) - len(pairs) for s in sides}
    # Only the GATED modes care about alignment. A paired row is routinely two
    # different regions — schedule_commode.csv row 6 is `10R,3L`, bottomlid RIGHT
    # with seat-ring LEFT — so asymmetry there is the normal case, not a defect,
    # and warning about it would train the reader to ignore the warning.
    if any(unmatched.values()) and mode in ("locked", "chained"):
        warnings.append(
            f"the two arms' stage sequences are not symmetric: {len(pairs)} shared "
            f"rendezvous, unmatched steps {unmatched}. {mode} gates on rendezvous, "
            "so unmatched steps run inside a phase with no cross-arm gate.")
    if mode == "free":
        warnings.append(
            "free mode is hardware-proven ARM-ONLY (C4 ran --no-hands --no-pole); "
            "pole and hand traffic in free mode is UNTESTED on hardware.")
    return DualPlan(mode, (lead, follow), phases, rendezvous, warnings)


def plan_compound_refusal(program: Program) -> str:
    """Why a COMPOUND config cannot be parallelised from what it states today.

    A compound task is one `stage_sequence` with `arm:` per stage. It is
    tempting to treat the `scene` stages as barriers and let each arm's runs
    between them overlap — the structure invites it. It is WRONG, and
    `lid_open` says so plainly. Between `forbid_collision(right)` and the
    preceding scene stage the config states, in order:

        move_to_start(left)      <- left goes to grasp the lid
        hand_release(right)      <- right lets go of the lid
        move_to_retract(right)

    Overlap those and the right hand may open before the left has arrived: the
    lid falls. That ordering is a real hand-over dependency and the config
    expresses it ONLY as adjacency — there is no field that distinguishes "this
    follows that because it must" from "this follows that because a list has an
    order". No derivation can recover the difference, so the dispatcher keeps
    the stated order and this returns the reason rather than a schedule.
    """
    return (
        f"{program.task}: compound tasks are dispatched in the order the config "
        "states. Cross-arm concurrency cannot be derived from a compound "
        "stage_sequence because a hand-over dependency and an incidental "
        "adjacency are written identically. Parallelising it needs the config to "
        "SAY which stages may overlap.")


# ── execution ────────────────────────────────────────────────────────────────

@dataclass
class Event:
    arm: str
    stage: str
    phase: int
    t_dispatch: float
    t_done: float
    ret: object = None


def execute_dual(plan: DualPlan, by_side: Dict[str, object], *,
                 issue: Optional[Callable] = None,
                 clock: Callable[[], float] = time.perf_counter,
                 step_timeout_s: float = 600.0) -> List[Event]:
    """Run a `DualPlan`, returning the timeline in completion order.

    Threading follows `dual_arm_common.run_locked/run_chained/run_free`, which
    is the shape that ran on hardware: one thread per arm, each holding its OWN
    controller handle. Two things that shape depends on, kept true here:

    * each arm's backend wraps a DISTINCT handle (C1: left id=1, right id=2);
      one handle driven from two threads is not what was tested;
    * nothing here touches `rm_algo_*`, which is one C-library-global context
      (model / work frame / tool frame, last writer wins). Mode 1 sends poses to
      the controller and lets the controller solve, so two arms in one process
      are safe. Anything that adds LOCAL IK must re-set the full algo context
      per use or confine algo to one side.
    """
    from alix_dispatch.backends import ExecutionError, _issue
    issue = issue or _issue

    lead, follow = plan.arms
    events: List[Event] = []
    lock = threading.Lock()
    failed: List[BaseException] = []
    stop = threading.Event()

    def run_phase(side: str, phase: Phase) -> None:
        backend = by_side[side]
        for step in phase.steps:
            if stop.is_set():
                return
            t0 = clock()
            ret = None
            for call in step.calls:
                ret = issue(call, backend)
                if ret not in (None, 0):
                    raise ExecutionError(f"{call.stage}: {call.kind} returned {ret}")
            with lock:
                events.append(Event(side, step.stage, phase.index, t0, clock(), ret))

    def guard(fn) -> None:
        try:
            fn()
        except BaseException as exc:          # noqa: BLE001 - re-raised below
            with lock:
                failed.append(exc)
            stop.set()

    if plan.mode == "sequential":
        for side in (lead, follow):
            for phase in plan.phases[side]:
                guard(lambda p=phase, s=side: run_phase(s, p))
    elif plan.mode == "locked":
        for i in range(plan.n_phases):
            threads = [threading.Thread(
                target=guard, args=(lambda s=s, i=i: run_phase(s, plan.phases[s][i]),),
                daemon=True) for s in (lead, follow)]
            for t in threads:
                t.start()
            for t in threads:
                t.join(step_timeout_s)
                if t.is_alive():
                    stop.set()
                    raise ScheduleError(f"phase {i} did not complete within "
                                        f"{step_timeout_s}s")
            if failed:
                break
    elif plan.mode == "chained":
        gates = [threading.Event() for _ in range(plan.n_phases)]

        def leader_work() -> None:
            for i, phase in enumerate(plan.phases[lead]):
                if stop.is_set():
                    break
                run_phase(lead, phase)
                gates[i].set()

        def follower_work() -> None:
            for i, phase in enumerate(plan.phases[follow]):
                if not gates[i].wait(step_timeout_s) or stop.is_set():
                    return
                run_phase(follow, phase)

        threads = [threading.Thread(target=guard, args=(leader_work,), daemon=True),
                   threading.Thread(target=guard, args=(follower_work,), daemon=True)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(step_timeout_s * plan.n_phases)
        for g in gates:                        # release a follower stuck on a
            g.set()                            # gate the leader never opened
    elif plan.mode == "free":
        def all_phases(side: str) -> None:
            for phase in plan.phases[side]:
                if stop.is_set():
                    break
                run_phase(side, phase)

        threads = [threading.Thread(target=guard, args=(lambda s=s: all_phases(s),),
                                    daemon=True) for s in (lead, follow)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(step_timeout_s * plan.n_phases)
    else:                                       # pragma: no cover - MODES gate
        raise ScheduleError(f"unknown mode {plan.mode!r}")

    if failed:
        raise failed[0]
    events.sort(key=lambda e: e.t_done)
    return events


# ── verdicts on a timeline ───────────────────────────────────────────────────

def overlap_s(events: List[Event], a: str, b: str) -> float:
    """Total wall-clock during which BOTH arms were inside a step."""
    spans = {a: [(e.t_dispatch, e.t_done) for e in events if e.arm == a],
             b: [(e.t_dispatch, e.t_done) for e in events if e.arm == b]}
    total = 0.0
    for s0, e0 in spans[a]:
        for s1, e1 in spans[b]:
            total += max(0.0, min(e0, e1) - max(s0, s1))
    return total


def check_ordering(plan: DualPlan, events: List[Event]) -> Tuple[bool, List[str]]:
    """The invariant each mode promises, checked against what actually ran.

    C3's own verdict line is `ordering invariant (follower step k always after
    leader step k done)`; this is that check, generalised to phases.
    """
    problems: List[str] = []
    lead, follow = plan.arms
    per_arm: Dict[str, Dict[int, List[Event]]] = {lead: {}, follow: {}}
    for e in events:
        per_arm[e.arm].setdefault(e.phase, []).append(e)

    for arm in (lead, follow):
        seen = sorted(per_arm[arm])
        for i, j in zip(seen, seen[1:]):
            if max(e.t_done for e in per_arm[arm][i]) > min(
                    e.t_dispatch for e in per_arm[arm][j]) + 1e-9:
                problems.append(f"{arm}: phase {j} started before phase {i} finished")

    if plan.mode in ("chained", "locked"):
        for i in sorted(per_arm[follow]):
            if i not in per_arm[lead]:
                continue
            lead_done = max(e.t_done for e in per_arm[lead][i])
            follow_start = min(e.t_dispatch for e in per_arm[follow][i])
            if plan.mode == "chained" and follow_start < lead_done - 1e-9:
                problems.append(
                    f"phase {i}: follower started {1000*(lead_done-follow_start):.1f} ms "
                    "before the leader finished — the chain gate did not hold")
    if plan.mode == "locked":
        for i in sorted(per_arm[lead]):
            if i not in per_arm[follow] or not per_arm[follow][i]:
                continue
            for j in sorted(per_arm[lead]):
                if j <= i:
                    continue
                if not per_arm[follow].get(i):
                    continue
                if min(e.t_dispatch for e in per_arm[lead][j]) < max(
                        e.t_done for e in per_arm[follow][i]) - 1e-9:
                    problems.append(
                        f"locked barrier broken: {lead} entered phase {j} before "
                        f"{follow} finished phase {i}")
    if plan.mode == "sequential":
        if overlap_s(events, lead, follow) > 0.0:
            problems.append("sequential mode produced overlapping arm motion")
    return (not problems), problems


# ── CSV `chained`: per-arm region chains, counter-phased ─────────────────────

@dataclass
class RegionChain:
    """One arm's ordered run of regions, as a chained schedule cell states it.

    `0R>1R` is `RegionChain("right", [(0, toplid_right), (1, side_right)])`.
    """
    arm: str
    regions: List[Tuple[int, Program]] = field(default_factory=list)

    @property
    def region_ids(self) -> List[int]:
        return [rid for rid, _ in self.regions]


@dataclass
class RegionChainPlan:
    chains: List[RegionChain]
    warnings: List[str] = field(default_factory=list)

    def summary(self) -> str:
        return "chained: " + "  ".join(
            f"{c.arm}={'>'.join(str(r) for r in c.region_ids)}" for c in self.chains)


def plan_region_chains(chains: Sequence[RegionChain], *,
                       clearance: Optional[CrossArmClearance] = None
                       ) -> RegionChainPlan:
    """Schedule a `0R>1R,1L>0L` row: both arms run, counter-phased by region.

    The safety property of a chained row is the COUNTER-PHASING — right takes
    toplid then side while left takes side then toplid — so the two arms are
    never working the same region at the same time. That is enforced here as a
    per-region mutex rather than trusted to the two chains happening to take
    similar time, because "they finish together" (Newton's own note on C3) is an
    observation about equal work, not a guarantee.

    **No deadlock, and it is worth saying why**, because two arms taking two
    resources in opposite orders is the textbook deadlock. Each arm RELEASES
    region k before it REQUESTS region k+1, so the hold-and-wait condition never
    holds; the worst case is that one arm waits, not that both do. That is
    asserted below rather than argued.
    """
    if len(chains) != 2:
        raise ScheduleError(f"a chained row has two cells, got {len(chains)}")
    if {c.arm for c in chains} != {"left", "right"}:
        raise ScheduleError(
            f"a chained row must name opposite arms, got {[c.arm for c in chains]}")
    warnings: List[str] = []
    for chain in chains:
        if not chain.regions:
            raise ScheduleError(f"{chain.arm}: chained cell names no region")
        for rid, prog in chain.regions:
            if prog.side != chain.arm:
                raise ScheduleError(
                    f"region {rid} in the {chain.arm} chain carries a "
                    f"{prog.side} program")
            ok, _, problems = enforce_device_serialization(prog)
            if not ok:
                raise ScheduleError(f"{chain.arm} region {rid} violates F9: {problems}")

    a, b = chains
    if a.region_ids == b.region_ids:
        warnings.append(
            f"both arms take regions in the SAME order {a.region_ids} — a chained "
            "row is normally counter-phased so the arms never want the same "
            "region at once; this one serialises on every region instead.")
    shared = set(a.region_ids) & set(b.region_ids)
    if shared:
        _require_clearance("chained", clearance)
    return RegionChainPlan(list(chains), warnings)


def execute_region_chains(plan: RegionChainPlan, by_side: Dict[str, object], *,
                          issue: Optional[Callable] = None,
                          clock: Callable[[], float] = time.perf_counter,
                          region_timeout_s: float = 1800.0) -> List[Event]:
    """Run both chains concurrently under a per-region mutex."""
    from alix_dispatch.backends import ExecutionError, _issue
    issue = issue or _issue

    events: List[Event] = []
    lock = threading.Lock()
    failed: List[BaseException] = []
    stop = threading.Event()
    region_locks: Dict[int, threading.Lock] = {
        rid: threading.Lock() for c in plan.chains for rid in c.region_ids}
    # Proof obligation for the no-deadlock claim: an arm may hold at most one
    # region at a time. Violate it and the counter-phased pair CAN deadlock.
    held: Dict[str, Optional[int]] = {c.arm: None for c in plan.chains}

    def work(chain: RegionChain) -> None:
        backend = by_side[chain.arm]
        for phase_i, (rid, prog) in enumerate(chain.regions):
            if stop.is_set():
                return
            if held[chain.arm] is not None:
                raise ScheduleError(
                    f"{chain.arm} still holds region {held[chain.arm]} while "
                    f"requesting {rid} — hold-and-wait, this can deadlock")
            if not region_locks[rid].acquire(timeout=region_timeout_s):
                raise ScheduleError(f"{chain.arm}: region {rid} never freed")
            held[chain.arm] = rid
            try:
                for step in _steps(prog):
                    if stop.is_set():
                        return
                    t0 = clock()
                    ret = None
                    for call in step.calls:
                        ret = issue(call, backend)
                        if ret not in (None, 0):
                            raise ExecutionError(
                                f"{call.stage}: {call.kind} returned {ret}")
                    with lock:
                        events.append(Event(chain.arm, step.stage, phase_i,
                                            t0, clock(), ret))
            finally:
                held[chain.arm] = None
                region_locks[rid].release()

    def guard(chain: RegionChain) -> None:
        try:
            work(chain)
        except BaseException as exc:          # noqa: BLE001 - re-raised below
            with lock:
                failed.append(exc)
            stop.set()

    threads = [threading.Thread(target=guard, args=(c,), daemon=True)
               for c in plan.chains]
    for t in threads:
        t.start()
    for t in threads:
        t.join(region_timeout_s)
    if failed:
        raise failed[0]
    events.sort(key=lambda e: e.t_done)
    return events


def region_exclusion_holds(plan: RegionChainPlan, events: List[Event]) -> Tuple[bool, List[str]]:
    """No region was worked by both arms at the same time.

    This is the property a chained row exists for; measured on the timeline
    rather than assumed from the mutex, because a mutex the executor forgot to
    take also produces a green run.
    """
    windows: Dict[Tuple[str, int], Tuple[float, float]] = {}
    for e in events:
        key = (e.arm, e.phase)
        lo, hi = windows.get(key, (e.t_dispatch, e.t_done))
        windows[key] = (min(lo, e.t_dispatch), max(hi, e.t_done))
    by_region: Dict[int, List[Tuple[str, float, float]]] = {}
    for c in plan.chains:
        for i, rid in enumerate(c.region_ids):
            if (c.arm, i) in windows:
                lo, hi = windows[(c.arm, i)]
                by_region.setdefault(rid, []).append((c.arm, lo, hi))
    problems = []
    for rid, spans in by_region.items():
        for (a1, s1, e1), (a2, s2, e2) in zip(spans, spans[1:]):
            if min(e1, e2) - max(s1, s2) > 0:
                problems.append(
                    f"region {rid}: {a1} and {a2} overlapped by "
                    f"{1000*(min(e1,e2)-max(s1,s2)):.1f} ms")
    return (not problems), problems
