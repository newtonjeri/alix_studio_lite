"""Who is holding the fixture — derived from a compound config, not assumed.

Newton, 2026-08-20: *"i do not understand your argument on compound sequential
tasks, like lid and seat opening."* Fair. The earlier claim — that a compound
task cannot be parallelised because a hand-over dependency and an incidental
adjacency are written identically — was **half right and stated far too
broadly**, and this module is the correction.

The half that was right: in `lid_open` the order

    14 left  allow_collision(seat_and_lid)
    15 left  move_to_start          <- left comes into contact with the lid
    16 right hand_release           <- right lets go of it

IS load-bearing. Reverse or overlap 15 and 16 and the right hand may open before
the left arm has arrived; the lid is at -30 deg, off its stable state, held by
nothing, and it falls.

The half that was wrong: I said the config does not distinguish that from an
incidental ordering. **It does.** The config states, per arm, exactly when the
hand closes and when the contact licence opens — and an arm that has a CLOSED
hand inside an ACTIVE contact licence on the fixture is, by construction, the
arm holding it. That is custody, and it is computable:

    lid_open   custody(right) = stages [6, 16)     custody(left) = [14, 24)
               continuous cover [6, 24), ONE transfer window, stages 14-16.

So the rule is not "compound tasks are sequential". It is:

    **Never re-order or overlap across a change of custody.**
    Everywhere else, the two arms may overlap subject to clearance.

Which leaves real concurrency on the table in every compound task we have —
see `concurrency_report`.

WHAT THIS DOES NOT MODEL. Custody says who is holding the object; it says
nothing about whether the two arms fit. That is the cross-arm clearance question
(R6), answered separately by
`alix_workspace.dual_arm_plan_collision.cross_arm_separation`. Both must pass
before anything overlaps.
"""

from __future__ import annotations

import statistics
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# Stage names that open / close a contact licence on a fixture surface.
LICENCE_OPEN = "allow_collision"
LICENCE_CLOSE = "forbid_collision"


@dataclass
class StageFacts:
    """One stage of a compound sequence, with what it does to custody."""
    index: int
    stage: str
    arm: Optional[str]
    device: str
    hand_closure: Optional[float] = None   # mean commanded joint, 0 rad = open
    licence: Optional[str] = None          # surface named by allow/forbid


@dataclass
class Custody:
    arm: str
    start: int          # stage index at which this arm takes custody
    end: int            # stage index at which it gives it up (exclusive)
    surface: str

    def covers(self, i: int) -> bool:
        return self.start <= i < self.end


@dataclass
class Transfer:
    """A stage range across which the set of holding arms changes."""
    at: int
    before: Tuple[str, ...]
    after: Tuple[str, ...]

    @property
    def is_release(self) -> bool:
        return len(self.after) < len(self.before)


def stage_facts(cfg, program) -> List[StageFacts]:
    """Line up the config's stage_sequence with the resolved program.

    Hand closure comes from the PROGRAM, not the config: the config names a
    state (`half_grasp`), and only the resolved program knows what that state
    means in joint terms. 0 rad is fully open (`conversions.hpp`), so a larger
    mean is a more closed hand.
    """
    by_stage: Dict[int, StageFacts] = {}
    seq = cfg.stage_sequence
    # program calls follow stage order; `config` calls (tool frame) carry the
    # name "select_tool_frame" and are not stages.
    calls = [c for c in program.calls if c.device != "config"]
    if len(calls) != len(seq):
        raise ValueError(
            f"{cfg.task}: {len(calls)} program calls against {len(seq)} stages — "
            "the 1:1 stage-to-call mapping this analysis relies on does not hold")
    for i, (stage, call) in enumerate(zip(seq, calls)):
        f = StageFacts(i, str(stage.get("stage")), stage.get("arm") or call.arm,
                       call.device)
        if call.device == "hand":
            f.hand_closure = statistics.fmean(call.detail["joints_rad"].values())
        if f.stage in (LICENCE_OPEN, LICENCE_CLOSE):
            f.licence = stage.get("contact_surface")
        by_stage[i] = f
    return [by_stage[i] for i in sorted(by_stage)]


def custody_intervals(facts: List[StageFacts]) -> List[Custody]:
    """Stage ranges over which each arm is HOLDING the fixture.

    custody = hand CLOSED and contact licence ACTIVE.

    "Closed" is relative to the arm's own most-open commanded state in this
    task, not to a constant: the configs command `release`, `half_grasp` and
    `grasp`, and which of those counts as gripping is a property of the task,
    not a number this module gets to invent (C16.4).
    """
    arms = sorted({f.arm for f in facts if f.arm})
    out: List[Custody] = []
    for arm in arms:
        mine = [f for f in facts if f.arm == arm]
        closures = [f.hand_closure for f in mine if f.hand_closure is not None]
        if not closures:
            continue
        most_open = min(closures)

        closed_from: Optional[int] = None
        licence_from: Optional[int] = None
        surface = ""
        holding_from: Optional[int] = None

        def close_interval(end: int) -> None:
            nonlocal holding_from
            if holding_from is not None:
                out.append(Custody(arm, holding_from, end, surface))
                holding_from = None

        for f in facts:
            if f.arm != arm:
                continue
            if f.hand_closure is not None:
                closed_from = f.index if f.hand_closure > most_open else None
                if closed_from is None:
                    close_interval(f.index)
            if f.stage == LICENCE_OPEN:
                licence_from, surface = f.index, f.licence or ""
            elif f.stage == LICENCE_CLOSE:
                licence_from = None
                close_interval(f.index)
            if (closed_from is not None and licence_from is not None
                    and holding_from is None):
                holding_from = max(closed_from, licence_from)
        close_interval(facts[-1].index + 1)
    return sorted(out, key=lambda c: (c.start, c.arm))


def holders_at(intervals: List[Custody], i: int) -> Tuple[str, ...]:
    return tuple(sorted(c.arm for c in intervals if c.covers(i)))


def transfers(facts: List[StageFacts], intervals: List[Custody]) -> List[Transfer]:
    """Stages at which the set of holding arms changes."""
    out: List[Transfer] = []
    prev = holders_at(intervals, facts[0].index)
    for f in facts[1:]:
        cur = holders_at(intervals, f.index)
        if cur != prev:
            out.append(Transfer(f.index, prev, cur))
        prev = cur
    return out


def frozen_stages(facts: List[StageFacts], intervals: List[Custody]) -> Dict[int, str]:
    """Stages whose position must not move, with the reason.

    A stage is frozen when it is the one that CHANGES custody: re-ordering or
    overlapping it changes whether custody ever reaches zero, which is the
    difference between a held lid and a dropped one. Every other stage is free
    to overlap as far as this analysis is concerned.
    """
    frozen: Dict[int, str] = {}
    for t in transfers(facts, intervals):
        stage = facts[t.at].stage if t.at < len(facts) else "?"
        arm = facts[t.at].arm if t.at < len(facts) else "?"
        frozen[t.at] = (
            f"custody {list(t.before) or 'none'} -> {list(t.after) or 'none'} at "
            f"{stage}({arm})")
        # The stage that hands custody over is only safe because the OTHER arm
        # took it first: freeze the acquiring stage too, or the pair can slide.
        for c in intervals:
            if c.start == t.at or c.end == t.at:
                frozen.setdefault(c.start, f"{c.arm} takes custody of {c.surface}")
                frozen.setdefault(min(c.end, len(facts) - 1),
                                  f"{c.arm} gives up custody of {c.surface}")
    return dict(sorted(frozen.items()))


def concurrency_report(cfg, program) -> dict:
    """What a compound task may and may not overlap, with the reason for each."""
    facts = stage_facts(cfg, program)
    intervals = custody_intervals(facts)
    frozen = frozen_stages(facts, intervals)
    motion = [f for f in facts if f.device not in ("scene", "config")]
    free = [f for f in motion if f.index not in frozen]

    # A run of consecutive free stages owned by ONE arm can overlap a run owned
    # by the other, provided no frozen stage separates them.
    blocks: List[dict] = []
    cur: List[StageFacts] = []
    for f in motion:
        if f.index in frozen:
            if cur:
                blocks.append({"arm": cur[0].arm,
                               "stages": [x.stage for x in cur],
                               "indices": [x.index for x in cur]})
                cur = []
            continue
        if cur and cur[-1].arm != f.arm:
            blocks.append({"arm": cur[0].arm, "stages": [x.stage for x in cur],
                           "indices": [x.index for x in cur]})
            cur = []
        cur.append(f)
    if cur:
        blocks.append({"arm": cur[0].arm, "stages": [x.stage for x in cur],
                       "indices": [x.index for x in cur]})

    return {
        "task": cfg.task,
        "stages": len(facts),
        "motion_stages": len(motion),
        "custody": [(c.arm, c.start, c.end, c.surface) for c in intervals],
        "custody_continuous": _continuous(intervals),
        "transfers": [(t.at, list(t.before), list(t.after))
                      for t in transfers(facts, intervals)],
        "frozen": frozen,
        "free_stages": len(free),
        "blocks": blocks,
    }


def _continuous(intervals: List[Custody]) -> Optional[Tuple[int, int]]:
    """The stage range over which SOMEONE is always holding, or None if it breaks."""
    if not intervals:
        return None
    ordered = sorted(intervals, key=lambda c: c.start)
    lo, hi = ordered[0].start, ordered[0].end
    for c in ordered[1:]:
        if c.start > hi:
            return None                    # a gap: nobody is holding it
        hi = max(hi, c.end)
    return (lo, hi)
