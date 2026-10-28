"""The two arms at once inside a lid or seat plan (the user, 27 Oct night: "the left arm and right arm can move together
for some canfd tasks, for example moving to the start poses"): where the plan allows it (mechanism.two_arm_leg), with
the two arms' clearance over the stretch checked first (leg_clearance), run on two threads (RealExec.move_together)."""
import threading
import time
from types import SimpleNamespace

import pytest

from alix_studio_lite import vendor_paths
from alix_studio_lite.server import mechanism as m

vendor_paths.activate()


def _entries(task, hand_set=("right",)):
    plan = m.Plan(m.sequence_plan(task))
    entries, _ = m.without_start_checks(list(plan.entries))
    unsent = m.trailing_releases(entries, set(hand_set))
    bridged = {id(next((e for e in entries if e.kind == "arm" and e.side == s), None)) for s in plan.sides()}
    return plan, entries, unsent, bridged


def test_the_plans_stretches_the_two_arms_run_at_once(monkeypatch):
    """Open the lid (the owner's order since 29 Oct: the right's look first, then its pole and grasp): the right's look,
    pole and grasp with the left's pole and half grasp (set at the approach: no time); the left's approach and lift stay
    out, its lift's lines would begin 1.2 s in, before the right's grasp ends (1.7 s: its fingers closed before the lid
    rises); the left's retreat to rest with the right's grasp and look; nothing while the lid is lifted and held (no
    meshes for that state), nothing across a check. Open the seat: the first stretch alone (after the left's release
    every move is a hand-off the plan orders). With the right's look alone before the lift (the order before 29 Oct),
    the lift joins the stretch: the right's move ends LEG_MARGIN_S before its lines begin."""
    plan, entries, unsent, bridged = _entries("lid_open")
    legs = m.plan_legs(plan, entries, unsent, bridged)
    assert [(i, leg.used, leg.phase) for i, leg in legs] == [(0, 5, "lid_closed"), (14, 4, "lid_open")]
    first, second = legs[0][1], legs[1][1]
    assert first.label() == ("LEFT move_to_half_pole_length + hand_grasp || "
                             "RIGHT move_to_inspect + move_to_inspect_pole_length + hand_grasp")
    assert first.seconds("left") == 0.0 and first.seconds("right") == pytest.approx(1.73, abs=0.02)
    carry = m.joined_moves(entries, first.used, bridged)[0]
    assert carry.side == "left" and m.carries_fixture(carry) and 1.0 < m.fixture_offset_s(carry) < 1.5
    assert first.seconds("right") + m.LEG_MARGIN_S > m.fixture_offset_s(carry)             # so the lift is not in it
    looked_first = [e for k, e in enumerate(entries) if not (k in (1, 2) and e.side == "right")]   # the right's pole and grasp
    with monkeypatch.context() as mp:                                   # out: its look alone before the lift, a margin it keeps
        mp.setattr(m, "LEG_MARGIN_S", 0.2)
        leg = m.two_arm_leg(looked_first, 0, unsent, bridged, plan.name)
    carry = next(e for e, _, _ in leg.order if m.carries_fixture(e))
    assert carry.side == "left" and carry.stage == "move_to_pre_start + move_to_start + execute_path"
    assert leg.seconds("right") == pytest.approx(0.84, abs=0.02) and leg.seconds("right") + 0.2 <= m.fixture_offset_s(carry)
    assert m.leg_window_s(leg) == pytest.approx(m.fixture_offset_s(carry) + m.LEG_MARGIN_S, abs=1e-6)
    assert [leg.took(e) for e, _, _ in leg.order if e.side == "left"][:2] == [0.0, 0.0]    # the left's pole and first hand: set already
    assert [(e.side, e.kind) for e, _, _ in second.order] == [("left", "arm"), ("right", "hand"), ("right", "arm")]
    assert second.label() == "LEFT move_to_rest || RIGHT hand_grasp + move_to_inspect"
    release = next((e, n) for e, _, n in second.order if e.kind == "hand")
    assert release[1] == "move_to_inspect"                                                  # the hand entry knows the arm's next move
    assert m.two_arm_leg(entries, 8, unsent, bridged, plan.name) is None                    # the lid lifted and held: one at a time
    assert m.fixture_phase_at(entries, 8, plan.name) is None and m.fixture_phase_at(entries, 14, plan.name) == "lid_open"
    plan, entries, unsent, bridged = _entries("seat_open")
    legs = m.plan_legs(plan, entries, unsent, bridged)
    assert [(i, leg.used, leg.phase) for i, leg in legs] == [(0, 5, "lid_open")]
    assert m.two_arm_leg(entries, 13, unsent, bridged, plan.name) is None                   # after the left's release: the plan's order
    assert (m.LEG_MARGIN_S, m.LEG_CLEARANCE_MM, m.LEG_GRID_S, m.LEG_SHIFTS_S) == (0.3, 50.0, 0.02, (-0.3, 0.0, 0.3, 0.6))


def test_a_carry_too_soon_for_the_other_arms_move_ends_the_stretch_before_it(monkeypatch):
    """The other arm's moves must end LEG_MARGIN_S before a carry's lines: with a margin the right's move cannot keep,
    the left's lift stays out of the stretch (which is then the two arms' set-up only, nothing to gain, so none)."""
    plan, entries, unsent, bridged = _entries("lid_open")
    monkeypatch.setattr(m, "LEG_MARGIN_S", 1.0)
    leg = m.two_arm_leg(entries, 0, unsent, bridged, plan.name)
    assert leg is not None and not any(e.kind == "arm" and e.side == "left" for e, _, _ in leg.order)
    assert leg.used == 5


def test_the_two_arms_clearance_over_each_stretch_is_checked_with_the_real_meshes():
    """The three stretches of the two plans: the arms never within 50 mm, with one arm up to 0.6 s late or 0.3 s early
    and the hands as before or as each hand move sets them (collision_check.clearance_states on the real meshes)."""
    for task, n in (("lid_open", 2), ("seat_open", 1)):
        plan, entries, unsent, bridged = _entries(task)
        legs = m.plan_legs(plan, entries, unsent, bridged)
        assert len(legs) == n
        for i, leg in legs:
            out = m.leg_clearance(plan, entries, i, leg)
            assert out["ok"], (task, leg.label(), out)
            assert "never within 50 mm" in out["why"] and out["closest_mm"] is None
            assert m.leg_clearance(plan, entries, i, leg) is out                       # kept: a pure function of the plan


def _exec(arms, verdicts=None):
    ex = m.RealExec.__new__(m.RealExec)
    said = []
    ex.job = SimpleNamespace(progress=lambda *a, **k: None, sleep=lambda s: None, log=lambda *a, **k: None, on_cancel=None,
                             check=lambda: None)
    ex.robot = SimpleNamespace(speed_pct=60, found_speed=lambda side: 50, links={})
    ex.kit = SimpleNamespace(records_dir=lambda what: None)
    ex._log, ex._frames, ex._settled_s, ex._unsettled, ex._legs = None, None, 0.0, {}, dict(verdicts or {})
    ex._feeds, ex._hands, ex.feeds_after, ex.live = {"left", "right"}, {"left", "right"}, True, lambda *a, **k: None
    ex.say = lambda text, level="info": said.append((level, text))
    ex._arm = lambda side: arms[side]
    ex._keep_feed = lambda side: None
    ex._keep_hand = lambda side: None
    ex.__dict__["_hand_results"] = {}
    return ex, said


class _Arm:
    def __init__(self, side, log, fail=None):
        self.side, self.log, self.fail = side, log, fail

    def stream(self, rows, label, speed, restore_pct, bridge_speed, stop_hook, records, settle=True):
        t0 = time.monotonic()
        time.sleep(0.15)
        self.log.append((self.side, "stream", label, t0, time.monotonic(), threading.current_thread().name))
        if self.fail == label:
            raise m.UserError(f"{label}: refused at its first line")
        return dict(lines=len(rows), lines_sent=len(rows), seconds=0.15, bridged_deg=None, settle=None)

    def settle_end(self, last_row, label=""):
        self.log.append((self.side, "settle", label))
        return dict(waited_s=0.01, at_rest=True, by="feed")

    def hand_to(self, counts, duration_s=None):
        time.sleep(0.05)
        self.log.append((self.side, "hand", list(counts), time.monotonic()))
        return dict(ok=True, arrived=True, after=list(counts))

    def pole_to(self, mm, duration_s=None):
        self.log.append((self.side, "pole", round(mm)))
        return dict(ok=True)

    def joints_now(self):
        return dict(joints_deg=[0.0] * 7, by="feed")

    def hand_load(self):
        return dict(load_n=3.0, age_s=0.0)


def test_a_clear_stretch_runs_on_two_threads_and_the_plans_order_otherwise(monkeypatch):
    """Clear: each arm's moves in their order on a thread of its own, the two started together (their streams overlap in
    time); the release gate runs after a hand release as in the plan's order. Not clear, or not checked in time: one
    move after the other in the plan's order, the reason said. A move that fails is raised once the other arm's has ended."""
    plan, entries, unsent, bridged = _entries("lid_open")
    legs = m.plan_legs(plan, entries, unsent, bridged)
    leg = legs[1][1]                                                           # LEFT retreat || RIGHT grasp + inspect
    log = []
    ex, said = _exec({s: _Arm(s, log) for s in ("left", "right")}, {leg.key: dict(ok=True, why="never within 50 mm")})
    ex.move_together(leg)
    streams = [x for x in log if x[1] == "stream"]
    assert [x[0] for x in streams] == ["left", "right"] or [x[0] for x in streams] == ["right", "left"]
    left, right = next(x for x in streams if x[0] == "left"), next(x for x in streams if x[0] == "right")
    assert left[3] < right[4] and right[3] < left[4] and left[5] != right[5]       # at once, on two threads
    assert [x[1] for x in log if x[0] == "right"] == ["hand", "stream"]              # the right's order kept: grasp, then inspect
    assert any("both arms at once" in t for _, t in said)
    # not clear: the plan's order, said
    log.clear(); said.clear()
    ex, said = _exec({s: _Arm(s, log) for s in ("left", "right")}, {leg.key: dict(ok=False, why="the arms would come 31 mm apart")})
    ex.move_together(leg)
    assert [(x[0], x[1]) for x in log if x[1] in ("stream", "hand")] == [("left", "stream"), ("right", "hand"), ("right", "stream")]
    assert any("one arm after the other: the arms would come 31 mm apart" in t for _, t in said)
    # no verdict in time: the plan's order
    monkeypatch.setattr(m, "LEG_WAIT_S", 0.2)
    log.clear()
    ex, said = _exec({s: _Arm(s, log) for s in ("left", "right")}, {})
    t0 = time.monotonic()
    ex.move_together(leg)
    assert 0.15 < time.monotonic() - t0 < 1.5 and [x[0] for x in log if x[1] == "stream"] == ["left", "right"]
    assert any("not checked within" in t for _, t in said)
    # a failing move: raised after the other arm's move has ended
    log.clear()
    arms = {"left": _Arm("left", log, fail="LEFT arm move_to_pre_start + move_to_rest"), "right": _Arm("right", log)}
    ex, said = _exec(arms, {leg.key: dict(ok=True, why="clear")})
    with pytest.raises(m.UserError, match="refused at its first line"):
        ex.move_together(leg)
    assert [x[0] for x in log if x[1] == "stream"].count("right") == 1             # the right's move went to its end


def test_the_release_gate_runs_inside_a_stretch():
    """A hand release inside a stretch is gated as in the plan's order: the hand must have let go before that arm's next
    move (RealExec.released, with the arm's next stage)."""
    plan, entries, unsent, bridged = _entries("seat_open")
    # a made-up stretch: the left's release then retract, the right's inspect alongside (the plan itself orders these)
    left_release = next(e for e in entries if e.side == "left" and e.kind == "hand" and e.stage.startswith("hand_release"))
    left_retract = next(e for e in entries if e.side == "left" and e.stage == "move_to_retract")
    right_inspect = next(e for e in entries if e.side == "right" and e.stage == "move_to_inspect")
    leg = m.Leg(order=[(left_release, False, "move_to_retract"), (left_retract, False, None), (right_inspect, True, None)], used=3, phase="lid_open")
    log, gated = [], []
    ex, said = _exec({s: _Arm(s, log) for s in ("left", "right")}, {leg.key: dict(ok=True, why="clear")})
    ex.released = lambda e, nxt: gated.append((e.side, e.stage, nxt))
    ex.move_together(leg)
    assert gated == [("left", "hand_release", "move_to_retract")]
    assert [x[1] for x in log if x[0] == "left"] == ["hand", "stream"]
