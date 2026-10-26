"""The lift check's policy: what PROVES that a lid or a seat is lifted.

Pulled out of `arm_trajectory_sender.py` on 2026-10-06 so it imports no ROS and
can be tested, and so both senders share one rule. The rule (Newton, 2026-10-06,
item 6 of the real-frames review): **the state classifier alone never proves a
lift.** On the 111 real D435 gate frames the best model confuses lid_open and
lid_seat_open — ring up or ring down — on 14 of 47 fully visible views, and that
is exactly the question a seat-lift check asks. The depth-gap mode
(`perception_node` `gap_check`, plan/P20 F2) measures the part's height instead,
and it is fail-closed. So a `check_fixture_state` action whose expectation is a
lift is proven by `check: fixture_gap` only; `check: fixture_state` (the
classifier) on a lift expectation is refused before the service is even asked,
unless the action says `classifier_may_prove: true` — an explicit, visible
opt-in for a rig without depth.

The P20 configs (`mechanism_tasks/commode_c/lid_open.yaml`, `seat_open.yaml`)
already say `check: "fixture_gap"`; this makes a config that forgets fail
closed rather than fall back to the classifier.
"""

from __future__ import annotations

import os as _os
#: The perception service every check asks. ALIX_CHECK_SERVICE overrides it for a
#: TEST HARNESS ONLY (a scripted answerer beside a live perception node, 2026-10-09:
#: the ladder's re-lifts were exercised in the simulator without killing the
#: bringup's node); never set it on the robot.
CHECK_SERVICE = _os.environ.get('ALIX_CHECK_SERVICE') or '/perception/analyze_scene'
CHECK_TIMEOUT_S = 60.0
CHECK_MIN_CONFIDENCE = 0.5

#: `check:` value in the config -> AnalyzeScene mode. fixture_state is the
#: commode status classifier (closed / lid_open / lid_seat_open); fixture_gap
#: is the depth-gap mode (plan/P20 F2) — a node without it refuses the mode,
#: which reads as NOT lifted.
CHECK_MODES = {'fixture_state': 'status_check',
               'fixture_gap': 'gap_check'}
DEFAULT_CHECK = 'fixture_state'

#: Which answers prove each expectation. The classifier names the fixture
#: state (closed / lid_open / lid_seat_open); the gap mode names what it
#: measured: lid_closed / lid_lifted / lid_open (the lid absent from its
#: closed place), seat_closed / seat_lifted (the ring absent from its place).
#: Newton 2026-10-07 (flow): a mechanism plan also asks for the fixture STATE
#: it needs before it moves (seat_open needs lid_open; a plan whose outcome
#: already holds skips itself), so the table carries state expectations too.
LIFTED_STATES = {'lid_lifted': ('lid_open', 'lid_seat_open', 'lid_lifted'),
                 'seat_lifted': ('lid_seat_open', 'seat_lifted'),
                 'lid_open': ('lid_open', 'lid_seat_open', 'lid_lifted'),
                 'lid_closed': ('closed', 'lid_closed'),
                 'seat_open': ('lid_seat_open', 'seat_lifted', 'seat_open'),
                 'seat_closed': ('closed', 'lid_open', 'seat_closed')}

#: Which answers prove the OPPOSITE of each expectation: the states its
#: counterpart accepts (lid_lifted / lid_open <-> lid_closed, seat_lifted /
#: seat_open <-> seat_closed). An answer in neither table — refused, uncertain,
#: lid_between, not_lifted — proves nothing either way.
OPPOSITE_STATES = {'lid_lifted': LIFTED_STATES['lid_closed'],
                   'seat_lifted': LIFTED_STATES['seat_closed'],
                   'lid_open': LIFTED_STATES['lid_closed'],
                   'lid_closed': LIFTED_STATES['lid_open'],
                   'seat_open': LIFTED_STATES['seat_closed'],
                   'seat_closed': LIFTED_STATES['seat_open']}

# ── The vote over frames (Newton 2026-10-23, shared by ALIX and Studio Lite) ──
# One frame is one answer, and one answer is not a verdict: on 22-23 Oct the
# left hand that lifts the seat filled the seat ROI and a single refused frame
# counted as NOT lifted, which started a false re-lift. A check therefore asks
# CHECK_FRAMES_FIRST frames CHECK_FRAME_SPACING_S apart, each votes yes / no /
# nothing (frame_vote), and `decide` rules; when the three do not settle it, two
# more are taken. A check that is still UNDECIDED after CHECK_FRAMES_MAX frames
# cannot judge, and a check that cannot judge stops for a person: the senders
# escalate it whatever the plan's on_fail says (never a re-lift, a skip or a
# prerequisite).
CHECK_FRAMES_FIRST = 2      # 26 Oct: two frames that agree decide (was 3; a third and more only when they disagree)
CHECK_FRAMES_MAX = 5
CHECK_FRAME_SPACING_S = 0.15   # 26 Oct: the depth noise is independent frame to frame (was 0.3)

#: `decide` verdicts
PASS, FAIL, UNDECIDED = 'pass', 'fail', 'undecided'


#: Expectations that are LIFTS (or ring states the classifier confuses), and
#: the only check kinds that may prove one.
LIFT_EXPECTATIONS = ('lid_lifted', 'seat_lifted', 'seat_open', 'seat_closed')

#: on_fail policies a check may carry (config_loader validates the same set):
#:   abort            stop the plan (the default)
#:   retry_lift       the lifting arm goes back to the lift block's start and the
#:                    lift legs replay, up to max_retries times
#:   run_prerequisite the plan named by `prerequisite` runs first (it establishes
#:                    the state this check asked for), then this plan restarts,
#:                    up to max_retries times (default once)
#:   skip             the plan's own outcome already holds; it ends here as done
ON_FAIL_POLICIES = ('abort', 'retry_lift', 'run_prerequisite', 'skip')
LIFT_PROOF_CHECKS = ('fixture_gap',)


def check_kind(action) -> str:
    return str(action.get('check') or DEFAULT_CHECK)


def mode_for(action) -> tuple[str | None, str]:
    """-> (AnalyzeScene mode to ask, or None, why not).

    None means: do not ask — the answer could not prove the expectation.
    """
    kind = check_kind(action)
    mode = CHECK_MODES.get(kind)
    if mode is None:
        return None, f'unknown check {kind!r}; one of {sorted(CHECK_MODES)}'
    expect = action.get('expect')
    if expect in LIFT_EXPECTATIONS and kind not in LIFT_PROOF_CHECKS \
            and not bool(action.get('classifier_may_prove', False)):
        return None, (f'check: {kind} cannot prove {expect}: the state classifier alone never '
                      f'proves a lift (Newton, 2026-10-06); use check: fixture_gap, or say '
                      f'classifier_may_prove: true to opt in on a rig without depth')
    return mode, ''


def evaluate_fixture_check(action, response) -> bool:
    """True iff *response* (an AnalyzeScene response, or None) PROVES the
    expectation of *action*. Every doubt is False."""
    mode, _why = mode_for(action)
    if mode is None:
        return False
    accepted = LIFTED_STATES.get(action.get('expect'))
    if not accepted or response is None or not getattr(response, 'success', False):
        return False
    state = getattr(response, 'fixture_state', '')
    confidence = float(getattr(response, 'state_confidence', 0.0) or 0.0)
    return state in accepted and confidence >= CHECK_MIN_CONFIDENCE


def frame_vote(action, answer) -> str | None:
    """One frame's vote on *action*'s expectation: 'yes' when the answer is a
    state that proves it (LIFTED_STATES) at confidence >= CHECK_MIN_CONFIDENCE,
    'no' when it is a state that proves the opposite (OPPOSITE_STATES) at that
    confidence, otherwise None — a refusal, 'uncertain', 'lid_between', no frame
    or a low confidence proves nothing either way. A check whose kind could not
    prove the expectation (mode_for) never votes."""
    mode, _why = mode_for(action)
    expect = action.get('expect')
    if mode is None or answer is None or not getattr(answer, 'success', False):
        return None
    state = getattr(answer, 'fixture_state', '')
    confidence = float(getattr(answer, 'state_confidence', 0.0) or 0.0)
    if confidence < CHECK_MIN_CONFIDENCE:
        return None
    if state in LIFTED_STATES.get(expect, ()):
        return 'yes'
    if state in OPPOSITE_STATES.get(expect, ()):
        return 'no'
    return None


def decide(action, answers) -> str:
    """PASS, FAIL or UNDECIDED from the frames asked so far.

    From CHECK_FRAMES_FIRST (2) frames on: >= 2 yes and 0 no -> PASS; >= 2 no and
    0 yes -> FAIL; anything else -> UNDECIDED (take one more, up to five).
    At CHECK_FRAMES_MAX (5) frames: (>= 2 yes and 0 no) or (>= 3 yes and
    <= 1 no) -> PASS; the same for no -> FAIL; else UNDECIDED.
    Fewer than two frames is always UNDECIDED; a sender consults this at every
    frame from the second on (26 Oct; before: at three and at five)."""
    votes = [frame_vote(action, a) for a in answers or []]
    yes, no = votes.count('yes'), votes.count('no')
    if len(votes) < CHECK_FRAMES_FIRST:
        return UNDECIDED
    if len(votes) < CHECK_FRAMES_MAX:
        if yes >= 2 and no == 0:
            return PASS
        if no >= 2 and yes == 0:
            return FAIL
        return UNDECIDED
    if (yes >= 2 and no == 0) or (yes >= 3 and no <= 1):
        return PASS
    if (no >= 2 and yes == 0) or (no >= 3 and yes <= 1):
        return FAIL
    return UNDECIDED


def second_answer(response, region: str):
    """The answer to a FURTHER region asked in the same request — perception_node
    measures it on the same depth frame and returns it in the roi arrays
    (roi_regions[i] = the region, roi_recommendations[i] = the class,
    roi_confidences[i] = its confidence) — as an answer frame_vote can read, or
    None when the response carries no answer for it (an older node, no frame)."""
    if response is None:
        return None
    regions = [str(r) for r in (getattr(response, 'roi_regions', None) or [])]
    if region not in regions:
        return None
    i = regions.index(region)
    classes = list(getattr(response, 'roi_recommendations', None) or [])
    confidences = list(getattr(response, 'roi_confidences', None) or [])
    state = str(classes[i]) if i < len(classes) else ''
    confidence = float(confidences[i]) if i < len(confidences) else 0.0
    return _Answer(success=bool(state) and state != 'refused', fixture_state=state, state_confidence=confidence)


def observe(action, answers) -> tuple[str, int]:
    """A check's SECOND question (`also_observe`, Newton 2026-10-05: "the same
    frame also says whether the seat came up with the lid") decided over the
    SAME frames the check itself took, by the same vote. -> (PASS = observed,
    FAIL = not, UNDECIDED = unknown; how many of the frames answered it)."""
    what = str(action.get('also_observe') or '')
    question = {'check': action.get('check'), 'expect': what,
                'classifier_may_prove': action.get('classifier_may_prove', False)}
    seconds = [second_answer(a, what) for a in answers or []]
    return decide(question, seconds), sum(1 for s in seconds if s is not None)


class _Answer:
    """The three fields of an AnalyzeScene response the vote reads."""
    __slots__ = ('success', 'fixture_state', 'state_confidence')

    def __init__(self, success: bool, fixture_state: str, state_confidence: float):
        self.success, self.fixture_state, self.state_confidence = success, fixture_state, state_confidence


# ── The retry ladder (Newton 2026-10-09) ─────────────────────────────────────
# A retry_lift check may carry `retry_offsets`: one [x, y, z] per retry (metres,
# the reference frame's axes), a translation of the LIFTING arm's TAUGHT start
# pose. The miss on the lid is the hook fingers not getting far enough between
# lid and seat along -y, so the ladder is y -5 mm, y -10 mm, then y -5 + z -5 mm.
# Re-lift k (1-based) is PLANNED LIVE with rung k-1: the lift block is
# regenerated by the task node with `pose_offset:=<arm>:x,y,z` (pre_start, start
# and the arc points all follow the start) and spliced in place of the recorded
# block; the same ladder applies whether the plan is run live or replayed.
# Without a ladder the recorded block replays unchanged (the pre-10-09 re-lift).

#: Maximum magnitude of one correction, metres — the loader refuses larger.
RETRY_OFFSET_MAX_M = 0.05

#: Where a live re-plan is saved (Resource/plans/<fixture>/<this>/), never the
#: production folder.
RETRY_PLANS_SUBDIR = 'retry_live'


def retry_offsets(action) -> list:
    """The ladder carried by a check action: a list of (x, y, z) tuples in metres,
    or [] when the action carries none or carries garbage (then the recorded
    block replays unchanged, as before the ladder existed)."""
    raw = action.get('retry_offsets') if isinstance(action, dict) else None
    if not isinstance(raw, (list, tuple)):
        return []
    out = []
    for rung in raw:
        if isinstance(rung, dict):
            vals = [rung.get(k, 0.0) for k in ('x', 'y', 'z')]
        elif isinstance(rung, (list, tuple)) and len(rung) == 3:
            vals = list(rung)
        else:
            return []
        try:
            xyz = tuple(float(v) for v in vals)
        except (TypeError, ValueError):
            return []
        if any(abs(v) > RETRY_OFFSET_MAX_M for v in xyz):
            return []
        out.append(xyz)
    return out


def retry_offset(action, attempt: int):
    """The rung for re-lift number *attempt* (1-based): (x, y, z) metres, or None
    when the action carries no ladder or the ladder has no such rung."""
    ladder = retry_offsets(action)
    if attempt < 1 or attempt > len(ladder):
        return None
    return ladder[attempt - 1]


def pose_offset_arg(arm: str, xyz) -> str:
    """The task node's `pose_offset` value for *arm*: 'left:0.0000,-0.0050,0.0000'."""
    x, y, z = (float(v) for v in xyz)
    return f'{arm}:{x:.4f},{y:.4f},{z:.4f}'


def describe_offset(xyz) -> str:
    """'y -5 mm, z -5 mm' — the non-zero axes in millimetres, or 'no offset'."""
    parts = [f'{k} {v * 1000:+.0f} mm' for k, v in zip('xyz', xyz) if abs(float(v)) > 1e-9]
    return ', '.join(parts) if parts else 'no offset'
