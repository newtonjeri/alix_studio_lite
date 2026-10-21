"""The lid and the seat (server/mechanism.py): ALIX's four plans read and checked, the camera found by forward
kinematics, and ALIX's order and policies (arm_trajectory_sender.py) played through with the answers the robot gave."""
import json
from pathlib import Path

import numpy as np
import pytest

from alix_studio_lite.server import mechanism, robot_fk
from alix_studio_lite.server.gap_check import Answer
from alix_studio_lite.server.jobs import UserError

DATA = Path(__file__).parent / "data" / "gap_checks"


def test_the_four_plans_load_their_tracks_check_and_every_move_streams_at_5_ms():
    from alix_dispatch import track

    for name in mechanism.TASKS:
        plan = mechanism.Plan(name)
        assert plan.entries and plan.sides()
        arms = [e for e in plan.entries if e.kind == "arm"]
        assert arms and all(e.rows for e in arms)
        strokes = [e for e in arms if e.group is not None]
        assert strokes, name                                      # the lifts and pushes come from their own tracks
        for e in arms:
            pf = track.canfd_preflight(e.rows, mechanism.CLOCK_S, mechanism.RATE_GATE)
            assert pf["ok"], (name, e.stage, pf["reasons"])
        for i in range(1, len(plan.entries)):                     # one move ends where the next of that device starts
            e = plan.entries[i]
            before = [p for p in plan.entries[:i] if p.side == e.side and p.kind == e.kind]
            if before:
                assert np.allclose(before[-1].last(), e.first(), atol=1e-6), (name, e.stage)
    lid = mechanism.Plan("lid_open")
    assert lid.sides() == ["right", "left"]                      # the camera arm first, as ALIX approaches
    assert [c["expect"] for c in lid.checks()] == ["lid_closed", "lid_lifted"]
    assert lid.view()["rungs"] == [1, 2, 3]


def test_alix_s_5_ms_law_rebuilds_each_stroke_s_track():
    for name in mechanism.TASKS:
        plan = mechanism.Plan(name)
        for e in plan.entries:
            if e.group is not None:
                assert mechanism.emit_rows(e.t, e.q) == e.rows, (name, e.stage)


def test_a_track_that_is_not_the_planned_one_is_refused(tmp_path):
    src = mechanism.PLANS
    for p in src.glob("seat_close_right_toppra*"):
        (tmp_path / p.name).write_bytes(p.read_bytes())
    g = tmp_path / "seat_close_right_toppra.g0.txt"
    g.write_text(g.read_text().replace("]}", ", 0]}", 1))
    with pytest.raises(UserError, match="sha256 differs"):
        mechanism.Plan("seat_close_right", path=tmp_path / "seat_close_right_toppra.json")


def test_the_wrist_camera_is_where_the_robot_s_tf_put_it():
    """12 Oct, 09:39: the right arm at inspection_pose_4, the pole at 150 mm (half)."""
    pose4 = [0.5585, -1.6755, 0.1745, 1.9548, -0.4363, 1.0472, -0.2443]
    readings = {"right": {"q_deg": [float(np.degrees(v)) for v in pose4], "pole_mm": 150.0, "hand": [516] * 4 + [133, 944]}}
    M = robot_fk.camera_to_base(readings)
    rec = json.loads((DATA / "20261012_093917_987_gap_check_lid_closed_lid_closed.json").read_text())
    R = np.array(rec["camera_to_world"])
    assert np.linalg.norm(M[:3, 3] - R[:3, 3]) < 0.001
    cos = (np.trace(M[:3, :3].T @ R[:3, :3]) - 1) / 2
    assert np.degrees(np.arccos(np.clip(cos, -1, 1))) < 0.2
    assert robot_fk.camera_to_base({"right": {"q_deg": None, "pole_mm": 150.0}}) is None     # not read: no transform
    hands = robot_fk.hand_positions(readings, ["IRH_plam", "IRH_right_thumb_tip", "ILH_plam"])
    assert set(hands) == {"IRH_plam", "IRH_right_thumb_tip"}      # the left arm was not read: its hand is missing


class FakeExec:
    """Records what the runner asks for; the camera answers from a script."""

    def __init__(self, answers):
        self.answers, self.calls = list(answers), []

    def approach(self, plan, targets, at, words=None):
        self.calls.append(("approach", plan.name, at, tuple(targets)))

    def move(self, e, bridge=False):
        self.calls.append(("move", e.stage, e.side, e.kind, bridge, e.note))

    def hold(self, seconds, label):
        self.calls.append(("hold", label, seconds))

    def ask(self, action, expect, observe=False):
        state = self.answers.pop(0)
        self.calls.append(("ask", action.get("stage"), expect, state))
        return Answer(success=state is not None, fixture_state=state or "not_lifted", state_confidence=1.0 if state else 0.0)

    def check_stop(self):
        pass


def moves(ex):
    return [c for c in ex.calls if c[0] == "move"]


def test_lid_open_as_on_the_robot_on_12_october():
    """Closed; three misses (the lift and re-lifts 1 and 2), re-lift 3 lifts it; the seat came up with it, so the
    seat is closed after; seat_close_right's own questions."""
    ex = FakeExec(["lid_closed", "lid_closed", "lid_closed", "lid_closed", "lid_lifted", "seat_lifted",
                   "lid_open", "seat_lifted", "seat_closed"])                 # the last: seat_close_right's final check
    out = mechanism.Runner(ex).run("lid_open")
    assert [(p["plan"], p["outcome"], p["relifts"]) for p in out["plans"]] == [("lid_open", "done", 3), ("seat_close_right", "done", 0)]
    notes = [m[5] for m in moves(ex) if m[5]]
    assert notes[:6] == ["re-lift 1, pre-planned"] * 6 and "re-lift 3, pre-planned" in notes
    retract = [m for m in moves(ex) if m[1] == "move_to_retract"]
    assert retract == [("move", "move_to_retract", "left", "arm", True, "")]     # 24 degrees from rung 3's end: bridged
    first = {m[2]: m for m in moves(ex) if m[3] == "arm"}
    assert [m for m in moves(ex) if m[3] == "arm" and m[4]][0][1] == "move_to_inspect"   # the first move after the approach
    # lid_open's own final check is seat_close_right's first look (a follow-up looks first); seat_close_right's end is
    # checked by the camera from Open the seat's way in, then the right arm goes back to where it ended
    assert [c[1:3] for c in ex.calls if c[0] == "approach"] == [("lid_open", "start"), ("seat_close_right", "start"),
                                                                ("seat_open", "start"), ("seat_close_right", "end")]
    assert [c[1:] for c in ex.calls if c[0] == "ask"][-1] == ("verify_seat_closed_after", "seat_closed", "seat_closed")
    assert [p["check"] for p in out["checks"] if p.get("final")] == ["verify_seat_closed_after"]
    assert first


def test_a_lid_already_open_skips_lid_open_and_parks_where_it_would_end():
    ex = FakeExec(["lid_open"])
    out = mechanism.Runner(ex).run("lid_open")
    assert out["plans"] == [dict(plan="lid_open", title="Open the lid", outcome="skipped", relifts=0)]
    assert [c[2] for c in ex.calls if c[0] == "approach"] == ["start", "end"]
    assert [m[1] for m in moves(ex)] == ["move_to_inspect_pole_length", "hand_grasp", "move_to_inspect"]


def test_seat_open_with_the_lid_closed_runs_lid_open_first_then_itself_again():
    ex = FakeExec(["lid_closed",                         # seat_open: verify_lid_open -> NO: its prerequisite
                   "lid_closed", "lid_lifted", "seat_closed",     # lid_open: closed, lifted, the seat stayed down
                   "lid_open",                                    # lid_open's final check: the lid open
                   "lid_open", "seat_closed", "seat_lifted",      # seat_open again: lid open, seat down, seat lifted
                   "seat_open"])                                  # seat_open's final check: the seat open
    out = mechanism.Runner(ex).run("seat_open")
    assert [p["plan"] for p in out["plans"]] == ["lid_open", "seat_open"]
    assert [c[1:3] for c in ex.calls if c[0] == "approach"] == [
        ("seat_open", "start"), ("lid_open", "start"),
        ("seat_open", "start"), ("lid_open", "end"),             # lid_open's final check: Open the seat's way in, back
        ("seat_open", "start"),
        ("seat_close_right", "start"), ("seat_open", "end")]     # seat_open's: Close the seat's way in, back


def test_seat_open_re_lifts_back_along_its_lift_without_a_rung():
    ex = FakeExec(["lid_open", "seat_closed", "seat_closed", "seat_lifted", "seat_open"])
    out = mechanism.Runner(ex).run("seat_open")
    assert out["plans"][0]["relifts"] == 1
    back = [m for m in moves(ex) if m[5] == "re-lift restore, reversed"]
    assert [m[1] for m in back] == ["back along execute_path", "back along move_to_start", "back along move_to_pre_start"]
    assert all(m[2] == "left" for m in back)


def test_a_lift_never_proven_escalates_to_a_person_with_nothing_moved_after():
    ex = FakeExec(["lid_closed"] + ["lid_closed"] * 4)
    with pytest.raises(UserError, match="still NOT lid_lifted after 3 re-lift") as err:
        mechanism.Runner(ex).run("lid_open")
    assert err.value.code == "escalate"
    assert moves(ex)[-1][1] == "execute_path" and moves(ex)[-1][5] == "re-lift 3, pre-planned"


def test_an_unanswered_question_is_not_lifted():
    ex = FakeExec(["lid_closed", None, None, None, None])
    with pytest.raises(UserError, match="escalate|re-lift"):
        mechanism.Runner(ex).run("lid_open")


def test_a_plan_whose_end_state_the_camera_does_not_see_stops_the_run():
    """The lab, 21 Oct: the left arm lifted the lid, the right one did not take it, and the plan ran to its end all the
    same. Now the right arm goes back to its inspection pose after the plan and the camera checks the end state: a NOT
    stops the run there, for a person to look, with nothing moved after the look."""
    ex = FakeExec(["lid_closed", "lid_lifted", "seat_closed",        # closed, lifted, the seat stayed down
                   "lid_closed"])                                    # the final check: the lid fell back
    with pytest.raises(UserError, match="Open the lid ran to its end, but the camera does not see lid open") as err:
        mechanism.Runner(ex).run("lid_open")
    assert err.value.code == "escalate" and "A person must look" in err.value.hint
    assert [m[1:3] for m in moves(ex)][-3:] == [("move_to_inspect_pole_length", "right"), ("hand_grasp", "right"),
                                                ("move_to_inspect", "right")]       # Open the seat's way in, then nothing
    assert ex.calls[-1] == ("ask", "verify_lid_open_after", "lid_open", "lid_closed")


def test_each_final_check_looks_the_way_alix_planned_it_in_the_state_the_plan_leaves():
    """The right arm's way in of the plan that starts where this one ends: its pole, its hand and its move_to_inspect,
    which ends at inspection pose 4, where the camera's depth checks are made."""
    pose4 = next(p for p in mechanism.poses() if p["name"] == "inspection_pose_4")["joints_deg"]
    for name, (expect, by) in mechanism.FINAL_CHECKS.items():
        fc = mechanism.final_check(name)
        assert fc["expect"] == expect and fc["by"].name == by and fc["action"]["on_fail"] == "abort"
        assert [(e.kind, e.side) for e in fc["look"]] == [("pole", "right"), ("hand", "right"), ("arm", "right")]
        assert np.allclose(np.degrees(np.asarray(fc["look"][-1].last(), dtype=float)), pose4, atol=0.5), name
        # the way in is planned for the state this plan leaves (the lid closed covers the seat: "lid closed")
        assert mechanism.TASKS[by]["start"] == {"lid and seat closed": "lid closed"}.get(mechanism.TASKS[name]["end"],
                                                                                         mechanism.TASKS[name]["end"]), name


def test_the_catalogue_says_what_each_task_does_and_the_rows_it_runs_for():
    cat = {c["name"]: c for c in mechanism.catalogue()}
    assert set(cat) == set(mechanism.TASKS)
    assert cat["lid_open"]["standoff_mm"] == 250 and cat["lid_open"]["seconds"] > 5
    assert set(mechanism.ROWS.values()) == set(mechanism.TASKS)
    # ALIX's inspection poses of the right arm, from its group states; pose 4 is where lid_open's move_to_inspect ends
    poses = {p["name"]: p for p in mechanism.poses()}
    assert list(poses) == ["inspection_pose_1", "inspection_pose_4"] and all(p["arm"] == "right" for p in poses.values())
    assert np.allclose(poses["inspection_pose_4"]["joints_deg"], np.degrees([0.5585, -1.6755, 0.1745, 1.9548, -0.4363, 1.0472, -0.2443]), atol=0.01)
    inspect = next(e for e in mechanism.Plan("lid_open").entries if e.stage == "move_to_inspect")
    assert np.allclose(np.degrees(np.asarray(inspect.last(), dtype=float)), poses["inspection_pose_4"]["joints_deg"], atol=0.01)


def test_s2_s_plan_with_lid_and_seat_tasks_is_v53_s_plan_cut_at_them():
    """The owner, 18 Oct: pre-recorded tasks injected at a point in the sequence. Without one, the plan is V53's as it
    is; with them (on both arms, the same order), V53 plans each part and each task is a stage between them."""
    from alix_studio_lite.server import workflow

    def prog(pid, rule="TOGETHER"):
        return {"type": "program", "id": pid, "name": "", "speed": 100, "transit_speed": 30, "dual_rule": rule}

    lid = {"type": "mechanism", "task": "lid_open"}
    seat = {"type": "mechanism", "task": "seat_open"}
    left, right = [prog(71), prog(72)], [prog(71), prog(72)]
    assert workflow.build_full(left, right) == workflow.build_plan(left, right)
    plan = workflow.build_full([prog(71), lid, prog(72), seat], [prog(71), lid, prog(72), seat, {"type": "wait", "seconds": 1.0}])
    assert [s["kind"] for s in plan] == ["parallel", "mechanism", "parallel", "mechanism", "tail"]
    assert [s.get("task") for s in plan if s["kind"] == "mechanism"] == ["lid_open", "seat_open"]
    assert workflow.counts(plan)["mechanisms"] == 2
    with pytest.raises(ValueError, match="on both arms' lists, the same tasks in the same order"):
        workflow.build_full([prog(71), lid], [prog(71)])
    with pytest.raises(ValueError, match="unmatched Program section"):         # V53's own rule, in each part
        workflow.build_full([prog(71), lid, prog(72)], [prog(71), lid])
    checked = workflow.check_steps([lid, {"type": "mechanism", "task": "lid_open", "note": "after the pre-lid part"}], "left")
    assert checked[0] == {"type": "mechanism", "task": "lid_open"} and checked[1]["note"]
    with pytest.raises(UserError, match="is not a lid or seat task"):
        workflow.check_steps([{"type": "mechanism", "task": "flush"}], "left")
