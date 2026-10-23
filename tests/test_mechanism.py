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


def test_a_one_point_move_gets_alix_s_half_second_from_where_its_arm_was():
    """ALIX's sender gives a one-point move at t = 0 half a second to get there (arm_trajectory_sender's
    SINGLE_POINT_MIN_S): the revised lid and seat open (21 Oct) have the right arm's start at its pre-start, one point.
    A 5 ms stream needs two lines, so it runs from where that arm's previous move ended, within every gate."""
    from pathlib import Path

    from alix_dispatch import track

    joints = [f"R_joint{k}" for k in range(1, 8)]
    a, b = [0.1] * 7, [0.12] * 7
    data = {"sub_trajectories": [
        {"stage_name": "move_to_pre_start", "joint_names": joints,
         "waypoints": [{"time_from_start": 0.0, "positions": [0.0] * 7}, {"time_from_start": 1.0, "positions": a}]},
        {"stage_name": "move_to_start", "joint_names": joints, "waypoints": [{"time_from_start": 0.0, "positions": b}]}]}
    pre, one = mechanism._entries(data, Path("."), "a test plan")
    assert one.note == "one point: 0.5 s to it" and one.seconds == 0.5 and len(one.rows) == 101
    assert np.allclose(one.first(), pre.last()) and np.allclose(one.last(), b)
    assert track.canfd_preflight(one.rows, mechanism.CLOCK_S, mechanism.RATE_GATE)["ok"]
    assert not track.canfd_preflight(one.rows[:1], mechanism.CLOCK_S, mechanism.RATE_GATE)["ok"]     # what one line gets


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


def _plans_with_rung1_at(tmp_path, monkeypatch, off_deg: float) -> float:
    """ALIX's plans copied, lid_open's first re-lift made to start ``off_deg`` from where its lift ends (the rungs chain
    from there), and Lite reading them: the ladder's runs whatever re-lifts are vendored (21 Oct: the revised Open the lid
    ends its lift 3.2 degrees from where its re-lifts of 10 Oct begin). The lift's end, joint 1 (rad)."""
    import shutil

    dst = tmp_path / "hardware"
    shutil.copytree(mechanism.PLANS, dst)
    plan = json.loads((dst / "lid_open_toppra.json").read_text())
    lift = next(st for st in plan["sub_trajectories"] if st["stage_name"] == "execute_path" and st["joint_names"][0].startswith("L_"))
    end = list(lift["waypoints"][-1]["positions"])
    rung = dst / "rungs" / "lid_open_toppra.rung1.json"
    r = json.loads(rung.read_text())
    arm = next(st for st in r["sub_trajectories"] if st.get("waypoints") and st["joint_names"][0].startswith("L_")
               and "sliding_plate" not in st["joint_names"][0])
    arm["waypoints"][0]["positions"] = [end[0] + np.radians(off_deg)] + end[1:]
    rung.write_text(json.dumps(r))
    monkeypatch.setattr(mechanism, "PLANS", dst)
    mechanism.cached.cache_clear()
    return end[0]


@pytest.fixture
def fitted(tmp_path, monkeypatch):
    """lid_open's re-lifts starting where its lift ends: the ladder's policies, played through."""
    _plans_with_rung1_at(tmp_path, monkeypatch, 0.0)
    yield
    mechanism.cached.cache_clear()


def moves(ex):
    return [c for c in ex.calls if c[0] == "move"]


def test_lid_open_as_on_the_robot_on_12_october(fitted):
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


def test_a_lift_never_proven_escalates_to_a_person_with_nothing_moved_after(fitted):
    ex = FakeExec(["lid_closed"] + ["lid_closed"] * 4)
    with pytest.raises(UserError, match="still NOT lid_lifted after 3 re-lift") as err:
        mechanism.Runner(ex).run("lid_open")
    assert err.value.code == "escalate"
    assert moves(ex)[-1][1] == "execute_path" and moves(ex)[-1][5] == "re-lift 3, pre-planned"


def test_an_unanswered_question_is_not_lifted():
    ex = FakeExec(["lid_closed", None, None, None, None])
    with pytest.raises(UserError, match="escalate|re-lift"):
        mechanism.Runner(ex).run("lid_open")


def test_no_camera_refuses_a_lid_or_seat_task(monkeypatch):
    """The owner, 23 Oct: with no camera every check read NOT, and Open the lid's "is it closed?" NOT ended it as done
    with the lid closed (the lab, 14:38). Without the wrist camera a lid or seat task does not run; one already under
    way stops at the stage that needs it."""
    from types import SimpleNamespace

    monkeypatch.delenv("LITE_CHECK_SCRIPT", raising=False)
    plugged = {"ok": False, "devices": [], "why": "no D435 is plugged into this computer"}
    monkeypatch.setattr(mechanism, "kit", lambda robot: SimpleNamespace(camera=SimpleNamespace(available=lambda: plugged)))
    with pytest.raises(UserError, match=r"^Not run: the wrist camera cannot be read \(no D435 .* Nothing moved\.$") as err:
        mechanism.camera_required(object())
    assert err.value.code == "camera"
    with pytest.raises(UserError, match=r"^Open the lid STOPPED: the wrist camera cannot be read") as err:
        mechanism.camera_required(object(), "Open the lid")
    assert err.value.code == "camera"
    plugged.update(ok=True, devices=[{"name": "Intel RealSense D435"}], why=None)
    assert mechanism.camera_required(object())["ok"]


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


def test_s2_s_plan_has_each_lid_and_seat_task_as_a_point_both_arms_meet():
    """The owner, 18 Oct: pre-recorded tasks injected at a point in the sequence. A task uses both arms: it is on both
    arms' lists, the same tasks in the same order (with the Meets: at each, both arms meet), and it is a stage of its
    own between the arms' parts; each arm's rows after its last Program are its own (22 Oct: no tail waiting)."""
    from alix_studio_lite.server import flow, workflow

    def prog(pid):
        return {"type": "program", "id": pid, "name": "", "speed": 100, "transit_speed": 30}

    lid = {"type": "mechanism", "task": "lid_open"}
    seat = {"type": "mechanism", "task": "seat_open"}
    plan = flow.build([prog(71), lid, prog(72), seat], [prog(71), lid, prog(72), seat, {"type": "wait", "seconds": 1.0}])
    assert [s["kind"] for s in plan] == ["parallel", "mechanism", "parallel", "mechanism", "parallel"]
    assert plan[-1]["left"] is None and plan[-1]["right"]["steps"] == [{"type": "wait", "seconds": 1.0}]
    assert [s.get("task") for s in plan if s["kind"] == "mechanism"] == ["lid_open", "seat_open"]
    assert flow.counts(plan)["mechanisms"] == 2
    with pytest.raises(ValueError, match="must come in the same order on both arms"):
        flow.build([prog(71), lid], [prog(71)])
    assert [s["kind"] for s in flow.build([prog(71), lid, prog(72)], [prog(71), lid])] == ["parallel", "mechanism", "parallel"]
    checked = workflow.check_steps([lid, {"type": "mechanism", "task": "lid_open", "note": "after the pre-lid part"}], "left")
    assert {k: v for k, v in checked[0].items() if k != "uid"} == {"type": "mechanism", "task": "lid_open"} and checked[1]["note"]
    with pytest.raises(UserError, match="is not a lid or seat task"):
        workflow.check_steps([{"type": "mechanism", "task": "flush"}], "left")


def test_a_simulation_made_with_a_plan_revised_since_is_never_played_and_says_so():
    """21 Oct: ALIX's Open the lid was revised and vendored again; the lab project's last simulation names that plan's
    moves by their place (mechanism:lid_open:13 was the right arm's move to its start, now a hand step), and its 3D
    failed on the server ("Something went wrong"). Such a simulation is refused by name, and Real's gate says what
    changed instead of failing."""
    from alix_studio_lite.server import workflow

    now = mechanism.digest("seat_open")
    sim = {"identity": {"used": {"mechanisms": {"lid_open": "0" * 64, "seat_open": now}}}}
    assert workflow.plans_changed(sim) == ["Open the lid"]
    with pytest.raises(UserError) as err:
        workflow._require_plans(sim)
    assert err.value.code == "plans" and "Open the lid is not the one this simulation was made with" in str(err.value)
    assert workflow.plans_changed({"identity": {"used": {"mechanisms": {"seat_open": now}}}}) == []
    sides = {"left": {"programs": {}, "points": {}}, "right": {"programs": {}, "points": {}}}
    ident = dict(mode="both", pick=None, steps={"left": [], "right": []}, overall=100, used=dict(sides, mechanisms={"lid_open": "new"}))
    old = {"identity": dict(ident, used=dict(sides, mechanisms={"lid_open": "old"}))}
    assert workflow._changed(old, ident) == "changed since it was simulated: ALIX's plan of Open the lid (revised since)"


def test_re_lifts_planned_for_another_version_of_the_plan_stop_the_run_before_anything_moves(tmp_path, monkeypatch):
    """21 Oct: Open the lid revised (its lift ends elsewhere), its pre-planned re-lifts of 10 Oct not: they chain from where
    the lift ends, so the first one's stream would be refused at its first line ("the arm is not at the track's first
    point"). Lite says it at the plan (the LID / SEAT tab's plan, S2's simulation) and, at a missed lift, stops before
    the re-lift moves anything: a person must look."""
    _plans_with_rung1_at(tmp_path, monkeypatch, 3.0)
    try:
        note = mechanism.Plan("lid_open").rungs_note()
        assert note and "start 3.0 degrees from where its lift ends" in note and "Plan the re-lifts again in ALIX" in note
        assert mechanism.Plan("lid_open").view()["rungs_note"] == note
        ex = FakeExec(["lid_closed", "lid_closed"])                    # closed: the lift runs; NO: the lift missed
        with pytest.raises(UserError) as err:
            mechanism.Runner(ex).run("lid_open")
        assert err.value.code == "escalate" and "re-lift 1's pre-planned rung starts 3.0 degrees" in str(err.value)
        assert moves(ex)[-1][1] == "execute_path" and not any(m[5] for m in moves(ex))      # nothing of the re-lift moved
        assert mechanism.Plan("seat_open").rungs_note() is None                          # no rungs: back along its lift
    finally:
        mechanism.cached.cache_clear()


def test_a_re_lift_restore_goes_back_from_where_the_arm_stands():
    """The lab, 22 Oct 11:58, 11:59 and 12:00: the seat lift ended with the left hand on the seat, J6 2.4 to 4.8 degrees
    short of the lift's end and the other joints at it; the restore (back along the lift) began at the lift's end, so its
    stream refused to start (one 5 ms line closes at most 0.72 degrees) and the task failed there. The restore now begins
    where the arm stands: J6 held until the way back comes past it, never further than it got, and the restore still
    ends at the lift's start, every line inside the stream's limits."""
    from alix_dispatch import track

    plan = mechanism.cached("seat_open")
    lift = [e for e in plan.entries if e.side == "left" and e.kind == "arm" and e.stage == "execute_path"]
    back = mechanism.reverse_block(lift)[0]
    end = np.array(lift[0].rows[-1]) / 1000.0
    stands = end.copy()
    stands[5] -= 4.80                                   # the lab's arm at 11:58:05 (its capture): J6 short, the rest at the end
    rows, held = mechanism.hold_short(back.rows, stands)
    assert held == [(6, 4.8)]
    first = np.array(rows[0]) / 1000.0
    assert np.abs(first - stands).max() <= 0.06 < mechanism.start_tol_deg()          # the stream starts where the arm is
    assert max(r[5] for r in rows) <= round(stands[5] * 1000)                       # J6 never further into the seat
    assert rows[-1] == back.rows[-1] == lift[0].rows[0]                              # back at the lift's start
    assert track.canfd_preflight(rows, mechanism.CLOCK_S, mechanism.RATE_GATE)["ok"]
    same, none = mechanism.hold_short(back.rows, end)   # an arm at the lift's end: the restore as planned
    assert same == back.rows and none == []
