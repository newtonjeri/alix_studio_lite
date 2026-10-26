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
    for p in src.glob("seat_open_toppra*"):
        (tmp_path / p.name).write_bytes(p.read_bytes())
    g = tmp_path / "seat_open_toppra.g0.txt"
    g.write_text(g.read_text().replace("]}", ", 0]}", 1))
    with pytest.raises(UserError, match="sha256 differs"):
        mechanism.Plan("seat_open", path=tmp_path / "seat_open_toppra.json")


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
    """Closed; three misses (the lift and re-lifts 1 and 2), re-lift 3 lifts it; the right opens it; the end check (lid
    open, seat down) passes. The seat under the lifted lid is not looked at (the owner, 26 Oct): one question per check,
    the seat judged once, at the end."""
    ex = FakeExec(["lid_closed", "lid_closed", "lid_closed", "lid_closed", "lid_lifted", "lid_open"])
    out = mechanism.Runner(ex).run("lid_open")
    assert [(p["plan"], p["outcome"], p["relifts"]) for p in out["plans"]] == [("lid_open", "done", 3)]
    notes = [m[5] for m in moves(ex) if m[5]]
    assert notes[:6] == ["re-lift 1, pre-planned"] * 6 and "re-lift 3, pre-planned" in notes
    # the left's next move (since 24 Oct 21:39 its way out, after the right's push) starts where the ORIGINAL lift ended,
    # 149.5 degrees from rung 3's end: bridged
    left = [m for m in moves(ex) if m[2] == "left" and m[3] == "arm"]
    after = left[max(i for i, m in enumerate(left) if m[5].startswith("re-lift")) + 1]
    assert after[1].startswith("move_to_pre_start") and after[4] and not after[5], after
    first = {m[2]: m for m in moves(ex) if m[3] == "arm"}
    assert [m for m in moves(ex) if m[3] == "arm" and m[4]][0][1] == "move_to_inspect"   # the first move after the approach
    # the end check: Open the seat's way in (pose 4), the camera asked once, back along it, the right arm where the plan ended
    assert [c[1:3] for c in ex.calls if c[0] == "approach"] == [("lid_open", "start"), ("seat_open", "start"), ("lid_open", "end")]
    asks = [c[1:] for c in ex.calls if c[0] == "ask"]
    assert asks[-1] == ("verify_lid_open_after", "lid_open", "lid_open") and len(asks) == 6
    assert [p["check"] for p in out["checks"] if p.get("final")] == ["verify_lid_open_after"]
    assert first


def test_a_lid_already_open_skips_lid_open_and_parks_where_it_would_end():
    ex = FakeExec(["lid_open"])
    out = mechanism.Runner(ex).run("lid_open")
    assert out["plans"] == [dict(plan="lid_open", title="Open the lid", outcome="skipped", relifts=0)]
    assert [c[2] for c in ex.calls if c[0] == "approach"] == ["start", "end"]
    assert [m[1] for m in moves(ex)] == ["move_to_inspect_pole_length", "hand_grasp", "move_to_inspect"]


def test_seat_open_with_the_lid_closed_runs_lid_open_first_then_itself_again():
    ex = FakeExec(["lid_closed",                         # seat_open: verify_lid_open -> NO: its prerequisite
                   "lid_closed", "lid_lifted",                    # lid_open: closed, lifted (the seat under it: not asked)
                   "lid_open",                                    # lid_open's final check: the lid open
                   "lid_open", "seat_closed", "seat_lifted",      # seat_open again: lid open, seat down, seat lifted
                   "seat_open"])                                  # seat_open's final check: the seat open
    out = mechanism.Runner(ex).run("seat_open")
    assert [p["plan"] for p in out["plans"]] == ["lid_open", "seat_open"]
    assert [c[1:3] for c in ex.calls if c[0] == "approach"] == [
        ("seat_open", "start"), ("lid_open", "start"),
        ("seat_open", "start"), ("lid_open", "end"),             # lid_open's final check: Open the seat's way in, back
        ("seat_open", "start"),
        ("seat_open", "start"), ("seat_open", "end")]            # seat_open's: its own way in (pose 4), back


def test_an_arms_back_to_back_moves_go_as_one_stream_and_the_final_check_comes_back_along_its_way_in():
    """24 Oct: every stream has its own start, stop and settle (1.6 s each), and the final check went back by a MoveJ
    at the speed slider (3.2 s). An arm's moves straight after one another, none asking the camera or holding before
    it, are one stream (they meet on the same line in all four plans); the final check comes back along its way in."""
    ex = FakeExec(["lid_open", "seat_closed", "seat_lifted", "seat_open"])
    out = mechanism.Runner(ex).run("seat_open")
    assert [(p["plan"], p["outcome"]) for p in out["plans"]] == [("seat_open", "done")]
    arm = [(m[1], m[2]) for m in moves(ex) if m[3] == "arm"]
    assert arm == [("move_to_inspect", "right"), ("move_to_pre_start + move_to_start + execute_path", "left"),
                   ("move_to_rest", "right"), ("move_to_pre_start + move_to_start", "right"), ("move_to_retract", "left"),
                   ("move_to_retract_2 + move_to_rest", "left"), ("execute_path + move_to_rest", "right"),   # 24 Oct 15:07: retract_pose_2
                   ("move_to_inspect", "right"), ("back along move_to_inspect", "right")]   # the final check's way in, back
    assert [m[5] for m in moves(ex) if m[1] == "back along move_to_inspect"] == ["the final check's way in, reversed"]
    plan = mechanism.Plan("seat_open")
    joined, used = mechanism.joined_moves(plan.entries, [e.stage for e in plan.entries].index("move_to_pre_start"))
    parts = plan.entries[[e.stage for e in plan.entries].index("move_to_pre_start"):][:used]
    assert used == 3 and joined.rows == [r for e in parts for r in e.rows]
    assert abs(joined.seconds - sum(e.seconds for e in parts)) < 1e-9 and joined.side == "left"
    assert [c[1:3] for c in ex.calls if c[0] == "approach"][-1] == ("seat_open", "end")      # pole and hand, as it ended


def test_arms_whose_next_tasks_start_follows_go_to_no_plans_end():
    """The owner, 24 Oct: after any lid or seat task each arm goes to the start of its next task (S2: the task's
    ``then``, workflow.next_after_tasks). The run's last plan takes those arms (``leave``) to no end of its own: no
    skipped plan's end, no way back to where the plan ended after its final check; an earlier plan's still are, as the
    next plan starts from there."""
    ex = FakeExec(["lid_open", "seat_closed", "seat_lifted", "seat_open"])
    mechanism.Runner(ex, leave={"left", "right"}).run("seat_open")
    assert [c[1:3] for c in ex.calls if c[0] == "approach"] == [("seat_open", "start"), ("seat_open", "start")]
    ex = FakeExec(["lid_open"])                                   # the lid already open: Open the lid skipped
    mechanism.Runner(ex, leave={"right"}).run("lid_open")
    assert [(c[2], c[3]) for c in ex.calls if c[0] == "approach"] == [("start", ("right", "left")), ("end", ("left",))]
    ex = FakeExec(["lid_closed", "lid_closed", "lid_lifted", "lid_open", "lid_open", "seat_closed",
                   "seat_lifted", "seat_open"])                   # Open the seat runs Open the lid first, then itself again
    mechanism.Runner(ex, leave={"left", "right"}).run("seat_open")
    assert [c[1:3] for c in ex.calls if c[0] == "approach"] == [
        ("seat_open", "start"), ("lid_open", "start"), ("seat_open", "start"), ("lid_open", "end"),   # not its last plan
        ("seat_open", "start"), ("seat_open", "start")]


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


def test_a_question_the_camera_cannot_judge_stops_for_a_person_and_never_re_lifts():
    """The owner, 23 Oct: a check the camera cannot judge (its frames undecided) is a person's to look at, not a NOT:
    the seat checks of 22-23 Oct read a lifted seat as NOT through the glove, and the re-lift dragged the hand back."""
    ex = FakeExec(["lid_closed", None])
    with pytest.raises(UserError, match="verify_lid_lifted: the camera cannot judge whether it is lid lifted") as err:
        mechanism.Runner(ex).run("lid_open")
    assert err.value.code == "escalate" and "A person must look" in err.value.hint
    assert not [m for m in moves(ex) if m[5]]                    # no re-lift: the lift and nothing after it
    ex = FakeExec(["lid_open", "seat_closed", None])             # the lab's seat_open, 23 Oct 15:08
    with pytest.raises(UserError, match="verify_seat_lifted: the camera cannot judge") as err:
        mechanism.Runner(ex).run("seat_open")
    assert not [m for m in moves(ex) if m[5] == "re-lift restore, reversed"]
    ex = FakeExec(["lid_closed", "lid_lifted", None])                      # the final check undecided
    with pytest.raises(UserError, match="^Open the lid ran to its end, but verify_lid_open_after: the camera cannot judge"):
        mechanism.Runner(ex).run("lid_open")


def _frames_exec(frames):
    """A RealExec whose camera answers ``frames`` one per frame; the pauses between them kept."""
    from types import SimpleNamespace

    slept = []
    job = SimpleNamespace(progress=lambda *a, **k: None, set=lambda **k: None, sleep=slept.append, check=lambda: None)
    ex = mechanism.RealExec(job, SimpleNamespace(), {}, SimpleNamespace(scripted=lambda expect: None, records_dir=lambda what: None))
    answers = iter(frames)
    ex._frame = lambda expect, check=None: next(answers)
    return ex, slept


def _seen(state, conf=0.9):
    return Answer(success=True, fixture_state=state, state_confidence=conf, message=f"{state} seen")


REFUSED = Answer(success=False, message="only 0 % of the seat ROI lies in the seat band after masking the hands")
#: the checks as the plans carry them (check: fixture_gap: the gap check may prove a lift, lift_check.mode_for)
SEAT_LIFTED = {"stage": "verify_seat_lifted", "check": "fixture_gap", "expect": "seat_lifted"}
LID_LIFTED = {"stage": "verify_lid_lifted", "check": "fixture_gap", "expect": "lid_lifted"}


def test_each_check_writes_its_frames_and_what_they_decided(tmp_path):
    """Beside the frames' records, one file per check: each frame's record and answer, and the decision (a dataset
    tells a check's frames from its decision; the owner, 23 Oct: every frame's record kept)."""
    ex, _ = _frames_exec([_seen("seat_lifted"), _seen("seat_lifted"), REFUSED])
    ex.kit.records_dir = lambda what: tmp_path / what
    ex.ask(SEAT_LIFTED, "seat_lifted")
    rows = list((tmp_path / "checks").glob("*_verify_seat_lifted_frames.json"))
    assert len(rows) == 1, rows
    s = json.loads(rows[0].read_text())
    assert s["decided"] and s["state"] == "seat_lifted" and s["expect"] == "seat_lifted"
    assert [(f["success"], f["state"]) for f in s["frames"]] == [(True, "seat_lifted"), (True, "seat_lifted"), (False, "not_lifted")]


def test_a_checks_frames_are_named_beside_it_so_pulled_logs_open_them(tmp_path):
    """The owner, 24 Oct: the lab's ``_frames.json`` named its frames by the lab computer's absolute paths
    (/home/alai-labs/...), which do not open once the logs are pulled. Each record is named relative to the file."""
    from dataclasses import replace

    checks = tmp_path / "checks"
    ex, _ = _frames_exec([replace(_seen("lid_lifted"), record_path=str(checks / f"20261024_1141{k}0_000_gap_check_x.json"))
                          for k in range(2)] + [replace(REFUSED, record_path=None)])
    ex.kit.records_dir = lambda what: tmp_path / what
    ex.ask(LID_LIFTED, "lid_lifted")
    s = json.loads(next(checks.glob("*_frames.json")).read_text())
    assert [f["record"] for f in s["frames"]] == ["20261024_114100_000_gap_check_x.json", "20261024_114110_000_gap_check_x.json", None]


def test_each_check_is_decided_on_three_frames_or_five():
    """The owner, 23 Oct: "for every check we should be using 3 frames ... for a failed classification in 3, we add 2
    more frames, a total of 5", ruled by ALIX's own lift_check.decide (24 Oct: the same vote as ALIX's senders): each
    frame votes yes / no / nothing on the expectation (a refused frame, or below 0.5, nothing); after 3, 2 yes and no
    no pass, 2 no and no yes fail; after 5, also 3 yes with at most 1 no. 0.3 s between frames."""
    ex, slept = _frames_exec([_seen("seat_lifted", 0.8), _seen("seat_lifted", 1.0), REFUSED, _seen("seat_closed")])
    a = ex.ask(SEAT_LIFTED, "seat_lifted")
    assert a.success and a.fixture_state == "seat_lifted" and a.state_confidence == pytest.approx(0.8)   # the first yes
    assert a.message.startswith("3 frames (seat_lifted, seat_lifted, refused; 2 yes, 0 no, 1 no vote)") and slept == [0.3, 0.3]
    ex, slept = _frames_exec([_seen("seat_lifted"), _seen("seat_closed"), REFUSED, _seen("seat_lifted"), _seen("seat_lifted")])
    a = ex.ask(SEAT_LIFTED, "seat_lifted")
    assert a.success and a.fixture_state == "seat_lifted" and a.message.startswith("5 frames") and len(slept) == 4
    ex, _ = _frames_exec([_seen("seat_closed")] * 3)               # a clear NOT: the plan's own policy follows
    a = ex.ask(SEAT_LIFTED, "seat_lifted")
    assert a.success and a.fixture_state == "seat_closed"
    ex, _ = _frames_exec([REFUSED] * 5)                            # the lab's glove over the window, five times
    a = ex.ask(SEAT_LIFTED, "seat_lifted")
    assert not a.success and a.fixture_state == mechanism.CANNOT_JUDGE
    assert a.message.startswith("no decision in 5 frames (refused, refused, refused, refused, refused; 0 yes, 0 no, 5 no vote)")
    ex, _ = _frames_exec([_seen("seat_lifted"), _seen("seat_closed"), REFUSED, _seen("seat_closed"), REFUSED])
    assert not ex.ask(SEAT_LIFTED, "seat_lifted").success          # 1 yes against 2 no of 5: no decision
    ex, _ = _frames_exec([_seen("seat_lifted"), _seen("seat_lifted"), _seen("seat_closed"), REFUSED, REFUSED])
    assert not ex.ask(SEAT_LIFTED, "seat_lifted").success          # 2 yes but 1 no: neither 3 nor 5 frames settle it


def test_the_seat_is_not_looked_for_under_the_lifted_lid():
    """The owner, 26 Oct: the lift check's second question (ALIX's also_observe seat_lifted / if_observed
    seat_close_right of 5 Oct, still in the vendored plan) is not asked: under the lifted lid the seat is hidden by the
    lid and the hand (26 Oct: 0 of 3 frames answered it), the look costs time, and the seat is judged once, by the end
    check. No closing runs after Open the lid; none is offered."""
    ex = FakeExec(["lid_closed", "lid_lifted", "lid_open"])
    out = mechanism.Runner(ex).run("lid_open")
    assert [c[1:3] for c in ex.calls if c[0] == "ask"] == [("verify_lid_closed", "lid_closed"), ("verify_lid_lifted", "lid_lifted"),
                                                           ("verify_lid_open_after", "lid_open")]
    assert [p["plan"] for p in out["plans"]] == ["lid_open"]
    assert not {"seat_close", "lid_close", "seat_close_right"} & set(mechanism.TASKS) and set(mechanism.ROWS.values()) == set(mechanism.TASKS)
    assert not any("also_observe" in c or "if_observed" in c for c in mechanism.Plan("lid_open").checks())
    ex, said = FakeExec(["lid_closed", "lid_lifted", "lid_seat_open"]), []  # the seat came up with the lid: the end check says so
    out = mechanism.Runner(ex, lambda text, *a: said.append(text)).run("lid_open")
    assert [(p["plan"], p["outcome"]) for p in out["plans"]] == [("lid_open", "done")] and out["checks"][-1]["verdict"] == "PASS"
    assert said[-1].endswith("the camera reads lid_seat_open, which proves lid open; the seat came up with the lid: "
                             "Open the seat finds it up and skips itself")
    ex = FakeExec(["lid_closed", "lid_lifted", "seat_lifted"])             # half up under the opened lid: not lid open, a person looks
    with pytest.raises(UserError, match="does not see lid open after it") as e:
        mechanism.Runner(ex).run("lid_open")
    assert e.value.code == "escalate"


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
    ex = FakeExec(["lid_closed", "lid_lifted",                       # closed, lifted
                   "lid_closed"])                                    # the final check: the lid fell back
    with pytest.raises(UserError, match="Open the lid ran to its end, but the camera does not see lid open") as err:
        mechanism.Runner(ex).run("lid_open")
    assert err.value.code == "escalate" and "A person must look" in err.value.hint
    assert [m[1:3] for m in moves(ex)][-3:] == [("move_to_inspect_pole_length", "right"), ("hand_grasp", "right"),
                                                ("move_to_inspect", "right")]       # Open the seat's way in, then nothing
    assert ex.calls[-1] == ("ask", "verify_lid_open_after", "lid_open", "lid_closed")


def test_each_final_check_looks_the_way_alix_planned_it_in_the_state_the_plan_leaves():
    """The right arm's way in, up to its move_to_inspect, which ends at one of ALIX's inspection poses: Open the seat's
    (its pole, its hand, pose 4, where the depth checks are made) after Open the lid, the plan that starts where it
    ends; Open the seat's own after Open the seat (since 26 Oct no plan starts in its end state: the closings are
    withdrawn; until then Close the seat's, pose 1)."""
    poses = {p["name"]: p["joints_deg"] for p in mechanism.poses()}
    looks = {"lid_open": ([("pole", "right"), ("hand", "right"), ("arm", "right")], "inspection_pose_4"),
             "seat_open": ([("pole", "right"), ("hand", "right"), ("arm", "right")], "inspection_pose_4")}
    assert set(mechanism.FINAL_CHECKS) == set(looks) == set(mechanism.TASKS)
    for name, (expect, by) in mechanism.FINAL_CHECKS.items():
        fc = mechanism.final_check(name)
        assert fc["expect"] == expect and fc["by"].name == by and fc["action"]["on_fail"] == "abort"
        kinds, pose = looks[name]
        assert [(e.kind, e.side) for e in fc["look"]] == kinds and fc["look"][-1].stage == "move_to_inspect"
        assert np.allclose(np.degrees(np.asarray(fc["look"][-1].last(), dtype=float)), poses[pose], atol=0.5), name
        # the way in is planned for the state this plan leaves, when a plan starts there
        assert by == name or mechanism.TASKS[by]["start"] == mechanism.TASKS[name]["end"], name


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
        assert moves(ex)[-1][1].endswith("execute_path") and not any(m[5] for m in moves(ex))   # nothing of the re-lift moved
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


