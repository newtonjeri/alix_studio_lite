"""A plan's working stretch moved with the commode (the owner, 31 Oct: "if there is an offset, we only apply it to the
carrying path of the involved arm", and "verify whether by applying the offset we can translate the path by the same
margin"; plan_shift). ALIX's four vendored plans: the seat plans move within the owner's bounds; the lid plans, whose
carries start at the wrist's singularity (J6 near 0), are refused any move sideways and take a height through the pole."""
import numpy as np
import pytest

from alix_studio_lite.server import mechanism, plan_shift
from alix_studio_lite.server.jobs import UserError

SEAT_CLOSE_MOVE = plan_shift.Move(-0.02, 0.02, 0.0, -2.0)


def hands(side: str, frame: str, rows, pole_m: float) -> np.ndarray:
    """The hand's poses in base_link at each line (degrees), Lite's own FK (alix_sync), not plan_shift's."""
    from alix_studio_lite.server import alix_sync, robot_fk

    m = robot_fk.model()
    T_ref = m.fk(m.chain(m.root_link(), "alix_ref_frame"), [])
    return T_ref @ alix_sync._fk()(side, frame, np.radians(np.asarray(rows, dtype=float)), pole_m)


@pytest.fixture(scope="module")
def seat_close():
    plan = mechanism.Plan("seat_close")
    return plan, plan_shift.shift(plan, SEAT_CLOSE_MOVE)


def test_no_move_leaves_the_plan_as_it_is():
    plan = mechanism.Plan("seat_open")
    assert plan_shift.shift(plan, plan_shift.Move()).entries == plan.entries


def test_only_each_arms_working_stretch_moves(seat_close):
    plan, moved = seat_close
    roles = plan_shift.stretches(plan.entries)
    named = {(plan.entries[i].side, plan.entries[i].stage, r) for i, r in roles.items()}
    assert ("left", "move_to_pre_start", "into") in named and ("left", "execute_path", "moved") in named
    assert ("left", "move_to_retract_2", "moved") in named and ("left", "move_to_rest", "out") in named
    assert ("right", "execute_path", "moved") in named and ("right", "move_to_inspect", "out") in named
    for i, (a, b) in enumerate(zip(plan.entries, moved.entries)):
        if i not in roles:                       # the look, rest after the stretch, the poles and hands: the plan's own
            assert a.rows == b.rows and np.array_equal(np.asarray(a.q), np.asarray(b.q)), (a.side, a.stage)
        elif roles[i] == "into":                 # from where the plan has the arm, to the moved pre-start
            assert b.rows[0] == a.rows[0] and b.rows[-1] != a.rows[-1]
        elif roles[i] == "out":                  # from where the stretch ends, to where the plan goes
            assert b.rows[-1] == a.rows[-1] and b.rows[0] != a.rows[0]
        assert len(b.rows or []) == len(a.rows or []) and b.seconds == pytest.approx(a.seconds, abs=0.006)   # its own timing


def test_the_path_moves_by_the_commodes_move(seat_close):
    """Every 5 ms line of each moved move: its hand where the plan's hand is, moved by the commode's move (to 1 mm and
    half a degree; 31 Oct: 0.5 to 0.7 mm, the IK's 0.5 mm and the interpolation between its 20 ms keys)."""
    from alix_studio_lite.server.plan_shift import ik_frames, start_poles

    plan, moved = seat_close
    D = SEAT_CLOSE_MOVE.matrix()
    frames, pole = ik_frames("seat_close"), start_poles(plan.data)
    roles = plan_shift.stretches(plan.entries)
    for i, (a, b) in enumerate(zip(plan.entries, moved.entries)):
        if a.kind == "pole":
            pole[a.side] = float(a.last()[0])
        if roles.get(i) != "moved":
            continue
        want = D @ hands(a.side, frames[a.side], np.asarray(a.rows) / 1000.0, pole[a.side])
        got = hands(b.side, frames[b.side], np.asarray(b.rows) / 1000.0, pole[b.side])
        mm = np.linalg.norm(got[:, :3, 3] - want[:, :3, 3], axis=1).max() * 1000.0
        cos = np.clip((np.einsum("nij,nij->n", got[:, :3, :3], want[:, :3, :3]) - 1.0) / 2.0, -1.0, 1.0)
        assert mm <= plan_shift.VERIFY_MM and np.degrees(np.arccos(cos)).max() <= plan_shift.VERIFY_DEG, (a.side, a.stage, mm)


def test_moved_streams_join_as_the_plans_do_and_pass_alixs_stream_check(seat_close):
    from alix_dispatch import track

    plan, moved = seat_close
    for side in ("left", "right"):
        a = [e for e in plan.entries if e.kind == "arm" and e.side == side]
        b = [e for e in moved.entries if e.kind == "arm" and e.side == side]
        for (a0, a1), (b0, b1) in zip(zip(a, a[1:]), zip(b, b[1:])):
            assert (a0.rows[-1] == a1.rows[0]) == (b0.rows[-1] == b1.rows[0]), (side, a1.stage)
        for e in b:
            assert track.canfd_preflight(e.rows, mechanism.CLOCK_S, mechanism.RATE_GATE)["ok"], (side, e.stage)
    assert {r["stage"] for r in moved.shift_report} >= {"move_to_pre_start", "execute_path", "move_to_rest", "move_to_inspect"}


def test_height_goes_through_the_pole():
    """12 mm up: the pole each arm's stretch runs at raised by 12 mm, the arms' joints the plan's, the hands 12 mm up."""
    plan = mechanism.Plan("seat_open")
    moved = plan_shift.shift(plan, plan_shift.Move(dz_m=0.012))
    raised = [(a.side, a.stage, round((b.last()[0] - a.last()[0]) * 1000.0, 3))
              for a, b in zip(plan.entries, moved.entries) if a.kind == "pole" and a.last() != b.last()]
    assert raised == [("left", "move_to_half_pole_length", 12.0), ("right", "move_to_half_pole_length", 12.0)]
    assert all(a.rows == b.rows for a, b in zip(plan.entries, moved.entries) if a.kind == "arm")
    lift = next(e for e in plan.entries if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
    pole = next(e for e in plan.entries if e.kind == "pole" and e.side == "left").last()[0]
    up = hands("left", "L_glove_frame_3", np.asarray(lift.rows) / 1000.0, pole + 0.012)
    here = hands("left", "L_glove_frame_3", np.asarray(lift.rows) / 1000.0, pole)
    assert np.allclose(up[:, :3, 3] - here[:, :3, 3], [0.0, 0.0, 0.012], atol=1e-6)     # the pole stands upright


@pytest.mark.parametrize("move", [plan_shift.Move(0.031), plan_shift.Move(0.0, 0.0, 0.0, 3.1), plan_shift.Move(dz_m=-0.016),
                                  plan_shift.Move(0.025, 0.02)])
def test_a_move_beyond_the_owners_bounds_is_refused(move):
    with pytest.raises(UserError) as err:
        plan_shift.shift(mechanism.Plan("seat_close"), move)
    assert err.value.code == "shift" and "beyond 30 mm, 3 deg and 15 mm" in str(err.value)


def test_a_carry_through_the_wrists_singularity_takes_no_move_sideways():
    """Open the lid's lift starts at J6 -1.7 deg (its move to the start crosses 0): moved 5 mm, the arm's IK carries on
    from the other side of the singularity, J6 ending near +77 deg where the plan has -68: another branch, refused. The
    height alone goes through the pole."""
    with pytest.raises(UserError) as err:
        plan_shift.shift(mechanism.Plan("lid_open"), plan_shift.Move(0.005))
    assert err.value.code == "shift" and "another branch of the arm" in str(err.value)
    assert plan_shift.shift(mechanism.Plan("lid_open"), plan_shift.Move(dz_m=0.008)).shift_report


def test_a_move_toward_the_wrists_singularity_is_refused():
    """Open the seat's right arm reaches its pre-start at J6 16.7 deg: the commode 30 mm to the left takes it to 9 deg."""
    with pytest.raises(UserError) as err:
        plan_shift.shift(mechanism.Plan("seat_open"), plan_shift.Move(0.0, 0.03))
    assert "singularity" in str(err.value)


def test_the_collision_gate_finds_nothing_new_for_a_move_it_carries_with_the_room(seat_close):
    plan, moved = seat_close
    out = plan_shift.collisions(plan, moved)
    assert out["ok"] and not out["new"], out["new"][:2]


def test_the_collision_gate_names_what_only_the_moved_run_meets(seat_close, monkeypatch):
    from alix_studio_lite.server import collision_check

    plan, moved = seat_close
    ev = dict(first_t=3.0, last_t=3.2, what=["wall"], pairs=[dict(side="left", link="L_link4", object="cubicle wall", depth_mm=4.0)])
    monkeypatch.setattr(collision_check, "check_states",
                        lambda times, arms, home, scene, *a, **k: dict(events=[ev] if scene.get("move") else []))
    out = plan_shift.collisions(plan, moved)
    assert not out["ok"] and out["new"] == [ev]


def test_the_runner_sends_the_moved_stretch():
    """The runner moves the plan before any of it is sent, says so, and its re-lift (going back along the lift) is the
    moved lift's."""
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).parent))
    from test_mechanism import FakeExec

    class Rows(FakeExec):
        def move(self, e, bridge=False):
            super().move(e, bridge)
            if e.kind == "arm":
                self.calls.append(("rows", e.side, e.stage, e.rows))

    move = plan_shift.Move(0.01, -0.01, 0.0, 1.0)
    said = []
    ex = Rows(["lid_open", "seat_closed", "seat_lifted", "seat_open"])
    mechanism.Runner(ex, say=lambda text, *a: said.append(text), move=move).run("seat_open")
    sent = [c for c in ex.calls if c[0] == "rows"]
    want = plan_shift.shift(mechanism.Plan("seat_open"), move)
    lift = next(e for e in want.entries if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
    assert any(c[2].startswith("move_to_pre_start") and c[1] == "left" for c in sent)
    assert any(lift.rows[-1] == c[3][-1] for c in sent if c[1] == "left")       # the moved lift's end was sent
    assert not any(c[3][-1] == next(e for e in mechanism.Plan("seat_open").entries if e.kind == "arm" and e.side == "left"
                                     and e.stage == "execute_path").rows[-1] for c in sent if c[1] == "left")
    assert any("moved with the commode" in t for t in said)
