"""A lid or seat task placed by its two sources (the owner, 31 Oct): the UGV's reported placement first, compensated by
the wrist camera's reading at the task's first look against the reference look (server/place_correct.py)."""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from alix_studio_lite.server import mechanism, place_correct, plan_shift
from alix_studio_lite.server.jobs import UserError

sys.path.insert(0, str(Path(__file__).parent))
from test_mechanism import FakeExec  # noqa: E402

POSE0 = np.array([[-1.0, 0.0, 0.0, 1.005], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.42], [0.0, 0.0, 0.0, 1.0]])
LOOKS = {"seat_close": ("inspection_pose_1", "lid open, seat open"), "lid_close": ("inspection_pose_1", "lid open, seat down"),
         "seat_open": ("inspection_pose_4", "lid open, seat down"), "lid_open": ("inspection_pose_4", "lid shut")}
REF = dict(dx_m=0.012, dy_m=-0.015, dz_m=0.002, dyaw_deg=-3.1, lid_deg=110.0, seat_deg=110.0)    # the camera's mount in it


def look(task: str, dx=0.0, dy=0.0, dyaw=0.0, ok=True) -> dict:
    """A first look's reading the commode moved (dx, dy m, dyaw deg) from the reference's."""
    pose, state = LOOKS[task]
    r = dict(REF, dx_m=REF["dx_m"] + dx, dy_m=REF["dy_m"] + dy, dyaw_deg=REF["dyaw_deg"] + dyaw, ok=ok, state=state)
    return dict(ok=ok, reading=r, state=state, pose=pose, pole_mm=150.0, pose0=POSE0, note="" if ok else "a hand in the view",
                record=None)


def reference(task: str, refs: place_correct.References, ugv=None) -> str:
    pose, state = LOOKS[task]
    key = place_correct.look_key("commode_c", pose, 150.0, state)
    refs.teach(key, dict(task=task, plan=place_correct._digest(mechanism.Plan(task)), reading=dict(REF), ugv=ugv,
                         at="2026-10-31 09:00:00", record=None))
    return key


def test_the_ugvs_placement_moves_the_commode_the_other_way():
    """placement.py's: the UGV 15 mm further back (standoff 265 mm) and 10 mm to its left: the commode 15 mm further
    from the robot and 10 mm to its right; the plans' own placement is no move; none reported is None."""
    m = place_correct.ugv_move(dict(standoff_mm=265.0, left_mm=10.0), "commode_c")
    assert (round(m.dx_m, 4), round(m.dy_m, 4), m.dz_m, m.dyaw_deg) == (0.015, -0.010, 0.0, 0.0)
    assert place_correct.ugv_move(place_correct.plans_placement(), "commode_c") == plan_shift.Move()
    assert place_correct.ugv_move(None, "commode_c") is None


def test_a_turn_counts_by_the_millimetres_it_moves_the_hands():
    """The owner's +/-5 mm where the hands work: 1 deg about the commode's middle moves Close the seat's 4 to 5 mm."""
    plan = mechanism.Plan("seat_close")
    assert 4.0 < plan_shift.hands_apart_mm(plan, plan_shift.Move(dyaw_deg=1.0)) < 5.0
    assert plan_shift.hands_apart_mm(plan, plan_shift.Move(0.004, 0.003)) == pytest.approx(5.0, abs=0.01)
    assert plan_shift.hands_apart_mm(plan, plan_shift.Move(0.01), plan_shift.Move(0.01)) == pytest.approx(0.0, abs=1e-9)


def test_the_camera_decides_against_the_ugvs_move(tmp_path):
    """No camera, no reading, no reference: the UGV's alone. The camera's value within the tolerance of the UGV's (5 mm
    where the hands work): the plan is reused; beyond it the camera's value is applied, however far from the UGV's (the
    owner, 2 Nov: no stop for a disagreement), up to plan_shift's bounds (30 mm: a person's to see). Open the lid's look
    (the lid's front behind the glove: x scatters 5 to 8 mm) leaves its x at the UGV's."""
    refs = place_correct.References(tmp_path / "refs.json")
    plan = mechanism.Plan("seat_close")
    hands = lambda a, b: plan_shift.hands_apart_mm(plan, a, b)          # noqa: E731
    ugv = plan_shift.Move(0.012, 0.0)
    assert place_correct.decide(ugv, None, None, hands).move == ugv
    assert place_correct.decide(ugv, look("seat_close", ok=False), None, hands).level == "warn"
    assert "no reference look yet" in place_correct.decide(ugv, look("seat_close", 0.012), None, hands).words
    ref = refs.get(reference("seat_close", refs))
    d = place_correct.decide(ugv, look("seat_close", 0.014, 0.001), ref, hands)
    assert d.source == "ugv" and d.move == ugv and "the plan is reused" in d.words
    d = place_correct.decide(ugv, look("seat_close", 0.012, 0.008), ref, hands)
    assert d.source == "depth" and abs(d.move.dy_m - 0.008) < 1e-3 and abs(d.move.dx_m - 0.012) < 1e-3
    d = place_correct.decide(ugv, look("seat_close", 0.012, 0.015), ref, hands)         # 15 mm from the UGV's: applied
    assert d.source == "depth" and d.refuse is None and abs(d.move.dy_m - 0.015) < 1e-3
    d = place_correct.decide(ugv, look("seat_close", 0.0, 0.032), ref, hands)
    assert d.refuse and "beyond 30 mm" in d.refuse and d.move is None
    lid = mechanism.Plan("lid_open")
    lid_hands = lambda a, b: plan_shift.hands_apart_mm(lid, a, b)       # noqa: E731
    ref = refs.get(reference("lid_open", refs))
    d = place_correct.decide(ugv, look("lid_open", 0.020, 0.010), ref, lid_hands)
    assert d.source == "depth" and abs(d.move.dx_m - 0.012) < 1e-9 and abs(d.move.dy_m - 0.010) < 1e-3
    assert abs(d.depth.dx_m - 0.020) < 1e-3 and "its x is left at" in d.words          # what the camera read, kept
    assert place_correct.decide(ugv, look("lid_open", 0.030, 0.001), ref, lid_hands).source == "ugv"   # only x off


def test_the_cameras_value_is_against_where_the_plan_worked(tmp_path):
    """The camera's value is where the commode stands against the reference look (where the plan worked, whatever the
    UGV reported then), compared with the UGV's move today: the UGV reporting 15 mm, the camera 12 mm: within the 5 mm,
    the UGV's runs."""
    refs = place_correct.References(tmp_path / "refs.json")
    ref = refs.get(reference("lid_close", refs, ugv=dict(dx_m=0.003, dy_m=0.0)))
    plan = mechanism.Plan("lid_close")
    d = place_correct.decide(plan_shift.Move(0.015), look("lid_close", 0.012), ref,
                             lambda a, b: plan_shift.hands_apart_mm(plan, a, b))
    assert d.source == "ugv" and d.move == plan_shift.Move(0.015) and "the plan is reused" in d.words


class LookExec(FakeExec):
    """FakeExec with a camera that reads the commode's place at the first look (RealExec.look_frame / read_look) and
    the hook a sequence's look waits on; ``reading``: one for every look, or a list read in turn (the last again)."""

    def __init__(self, answers, reading):
        super().__init__(answers)
        self.readings = list(reading) if isinstance(reading, list) else [reading]
        self.look_after, self.looks = None, []

    def move(self, e, bridge=False):
        super().move(e, bridge)
        if e.kind == "arm":
            self.calls.append(("rows", e.side, e.stage, e.rows))
        hook = self.look_after
        if hook is not None and e.side == hook[0] and str(e.stage).split(" + ")[-1] == hook[1]:
            self.look_after = None
            hook[2]()

    def look_frame(self, take, task):
        self.looks.append(("take" if take else "kept", task, len(self.calls)))
        return ("frame", [0.0] * 7, None)

    def read_look(self, shot, start_for=None):
        return dict(self.readings.pop(0) if len(self.readings) > 1 else self.readings[0])


def sent_rows(ex, side, stage):
    """The streams of ``side`` that carry ``stage`` (alone or joined with the moves around it)."""
    return [c[3] for c in ex.calls if c[0] == "rows" and c[1] == side and stage in c[2].split(" + ")]


def carries(streams, rows) -> bool:
    """One of ``streams`` carries the middle line of ``rows`` (a joined stream's junctions may be blended)."""
    mid = list(rows[len(rows) // 2])
    return any(mid in [list(r) for r in s] for s in streams)


def run(task, ex, corr, **kw):
    said = []
    out = mechanism.Runner(ex, say=lambda text, *a: said.append(text), place=corr, **kw).run(task)
    return out, said


def test_the_ugvs_move_runs_from_the_start_and_the_camera_agreeing_keeps_it(tmp_path):
    """Close the seat with the UGV 12 mm further off: its working moves are the plan's moved 12 mm before anything is
    sent; the camera reads the same at the first look (within the dead band): the UGV's runs, nothing re-planned."""
    refs = place_correct.References(tmp_path / "refs.json")
    reference("seat_close", refs)
    ugv = plan_shift.Move(0.012)
    corr = place_correct.Correction(ugv=ugv, refs=refs, collide=False)
    plan = mechanism.Plan("seat_close")
    ex = LookExec([c["expect"] for c in plan.checks()], look("seat_close", 0.0125, 0.0005))
    out, said = run("seat_close", ex, corr)
    want = plan_shift.shift(plan, ugv)
    carry = next(e for e in want.entries if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
    assert carries(sent_rows(ex, "left", "execute_path"), carry.rows)
    assert not carries(sent_rows(ex, "left", "execute_path"),
                       next(e for e in plan.entries if e.kind == "arm" and e.side == "left" and e.stage == "execute_path").rows)
    assert ex.looks == [("kept", "seat_close", ex.looks[0][2])]
    row = out["places"][0]
    assert row["source"] == "ugv" and row["move"] == ugv.words() and "taught" not in row
    assert any("moved with the commode" in t for t in said)


def test_the_camera_corrects_and_stops_beyond_the_bounds(tmp_path):
    """No placement from the UGV (the plans' own): the camera reads the commode 8 mm to the left at Open the seat's
    first look: the plan's working moves from there on are the plan moved 8 mm, and nothing before them is; 15 mm to the
    right: the same (no stop for a disagreement since 2 Nov); 35 mm, beyond plan_shift's bounds: the run stops before any
    working move, said."""
    refs = place_correct.References(tmp_path / "refs.json")
    key = reference("seat_open", refs)
    plan = mechanism.Plan("seat_open")
    answers = [c["expect"] for c in plan.checks()]
    ex = LookExec(answers, look("seat_open", 0.0, 0.008))
    out, said = run("seat_open", ex, place_correct.Correction(refs=refs, collide=False))
    want = plan_shift.shift(plan, place_correct.depth_move(look("seat_open", 0.0, 0.008)["reading"], refs.get(key), POSE0))
    lift = next(e for e in want.entries if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
    assert carries(sent_rows(ex, "left", "execute_path"), lift.rows)
    assert sent_rows(ex, "right", "move_to_inspect")[0] == plan.entries[0].rows       # the look's own move, the plan's
    assert out["places"][0]["source"] == "depth"
    ex = LookExec(answers, look("seat_open", 0.0, -0.015))
    out, said = run("seat_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert out["places"][0]["source"] == "depth" and out["plans"][0]["outcome"] == "done"
    ex = LookExec(answers, look("seat_open", 0.0, 0.035))
    with pytest.raises(UserError) as err:
        run("seat_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert err.value.code == "place" and "before its working moves" in str(err.value) and "beyond 30 mm" in str(err.value)
    assert not sent_rows(ex, "left", "move_to_pre_start")


def test_a_run_that_passed_with_nothing_moved_teaches_the_reference(tmp_path):
    """Open the seat with no reference: the UGV's alone (none), the run passes with nothing moved: its first look is the
    reference now. Moved by the camera, a run never teaches."""
    refs = place_correct.References(tmp_path / "refs.json")
    plan = mechanism.Plan("seat_open")
    reading = look("seat_open", 0.001)
    ex = LookExec([c["expect"] for c in plan.checks()], reading)
    out, said = run("seat_open", ex, place_correct.Correction(refs=refs, collide=False))
    key = place_correct.look_key("commode_c", "inspection_pose_4", 150.0, "lid open, seat down")
    assert out["places"][0]["taught"] == key and refs.get(key)["reading"]["dx_m"] == reading["reading"]["dx_m"]
    assert refs.get(key)["plan"] == place_correct._digest(plan)
    ex = LookExec([c["expect"] for c in plan.checks()], look("seat_open", 0.009))
    out, said = run("seat_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert out["places"][0]["source"] == "depth" and "taught" not in out["places"][0]
    assert refs.get(key)["reading"]["dx_m"] == reading["reading"]["dx_m"]


def test_a_sequence_takes_its_first_look_without_its_check(tmp_path):
    """In a sequence the start checks are not asked (the owner, 27 Oct); the look is taken all the same, as the camera
    arm reaches its inspection pose (the owner, 31 Oct), and decides as a check's frame would."""
    refs = place_correct.References(tmp_path / "refs.json")
    reference("seat_close", refs)
    plan = mechanism.Plan("seat_close")
    dropped = {a.get("stage") for a in plan.entries[mechanism.first_check_at(plan.entries)].pre_actions}
    ex = LookExec([c["expect"] for c in plan.checks() if c["stage"] not in dropped], look("seat_close", 0.0, -0.007))
    out, said = run("seat_close", ex, place_correct.Correction(refs=refs, collide=False), start_checks=False)
    assert ex.looks and ex.looks[0][0] == "take"
    first_move = next(k for k, c in enumerate(ex.calls) if c[0] == "rows" and c[2].split(" + ")[-1] == "move_to_inspect")
    assert ex.looks[0][2] > first_move                         # after the camera arm reached its pose
    assert out["places"][0]["source"] == "depth"


def test_a_reference_belongs_to_its_look_and_is_kept(tmp_path):
    """A reference look is the offline plans' commode as this camera reads it (the owner, 2 Nov): one taught on an
    earlier version of the plan is used (every plan is made for the same commode place), and a run that passes with
    nothing moved never moves it."""
    refs = place_correct.References(tmp_path / "refs.json")
    key = reference("seat_open", refs)
    refs.teach(key, dict(refs.get(key), plan="another"))
    plan = mechanism.Plan("seat_open")
    out, said = run("seat_open", LookExec([c["expect"] for c in plan.checks()], look("seat_open", 0.0, 0.008)),
                    place_correct.Correction(refs=refs, collide=False))
    assert out["places"][0]["source"] == "depth"
    out, said = run("seat_open", LookExec([c["expect"] for c in plan.checks()], look("seat_open", 0.002)),
                    place_correct.Correction(refs=refs, collide=False))
    assert out["places"][0]["source"] == "none" and out["plans"][0]["outcome"] == "done" and "taught" not in out["places"][0]
    assert refs.get(key)["reading"]["dx_m"] == REF["dx_m"]


def test_the_real_look_reads_the_place_the_pose_and_the_pole():
    """RealExec.read_look on a lab frame (inspection pose 1, 27 Oct, the lid open, the seat down): the right arm's named
    pose from its joints, the pole from the frame's reading, the place searched 20 mm about where a reference look read
    it, alike the full search."""
    import json
    from types import SimpleNamespace

    import test_fixture_place as tf
    from alix_studio_lite.server.gap_check import GapCheck, load_params

    gc = GapCheck(params={**load_params(), "gap_cross_check": False}, records=None)
    stem = tf.SEAT_DOWN[1]
    rec = json.loads((tf.DATA / f"{stem}.json").read_text())
    frame = tf.frame(stem)
    frame = type(frame)(**{**frame.__dict__, "context": dict(arms=dict(right=dict(q_deg=rec["right_q_deg"], pole_mm=200.0)))})
    ex = SimpleNamespace(kit=SimpleNamespace(gap=gc))
    full = mechanism.RealExec.read_look(ex, (frame, rec["right_q_deg"], "rec.json"))
    assert full["ok"] and full["pose"] == "inspection_pose_1" and full["pole_mm"] == 200.0
    assert full["state"] == "lid open, seat down" and full["record"] == "rec.json"
    near = mechanism.RealExec.read_look(ex, (frame, rec["right_q_deg"], None), lambda pose, pole: (
        (full["reading"]["dx_m"] + 0.004, full["reading"]["dy_m"] - 0.003, full["reading"]["dz_m"], full["reading"]["dyaw_deg"]), True))
    assert near["ok"] and abs(near["reading"]["dx_m"] - full["reading"]["dx_m"]) < 0.0015
    assert abs(near["reading"]["dy_m"] - full["reading"]["dy_m"]) < 0.0015


def test_a_plan_that_cannot_follow_the_camera_runs_as_without_it(tmp_path):
    """Close the lid's plan of 29 Oct (its pull at the wrist's singularity) follows a height only: the camera reading the
    commode 8 mm to the left, its move is refused by plan_shift's gates, and the plan runs as it would without the camera
    (the UGV's: none), said; it does not stop (the lab's Close the lid runs of 27 Oct passed at such places)."""
    refs = place_correct.References(tmp_path / "refs.json")
    reference("lid_close", refs)
    plan = mechanism.Plan("lid_close")
    ex = LookExec([c["expect"] for c in plan.checks()], look("lid_close", 0.0, 0.008))
    out, said = run("lid_close", ex, place_correct.Correction(refs=refs, collide=False))
    pull = next(e for e in plan.entries if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
    assert carries(sent_rows(ex, "left", "execute_path"), pull.rows)
    row = out["places"][0]
    assert row["source"] == "none" and "cannot follow" in row["words"] and out["plans"][0]["outcome"] == "done"


def lifts_of(ex, side="left"):
    return sent_rows(ex, side, "execute_path")


def test_a_missed_lift_is_lifted_again_where_the_camera_reads_the_commode(tmp_path):
    """Open the seat (the owner, 2 Nov: "all the retries should be decided by the camera"): its first look within the
    tolerance (the plan as it was made), its lift missed; the commode's place read again on the failed check's frames,
    9 mm to the right of where the plan worked: the left arm goes back along the lift it made, then lifts from the plan
    made from the camera's value; the check passes, and the rest of the plan is the moved plan's."""
    refs = place_correct.References(tmp_path / "refs.json")
    key = reference("seat_open", refs)
    plan = mechanism.Plan("seat_open")
    ex = LookExec(["lid_open", "seat_closed", "seat_closed", "seat_lifted", "seat_open"],
                  [look("seat_open", 0.001), look("seat_open", 0.0, -0.009)])
    out, said = run("seat_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert out["plans"][0]["outcome"] == "done" and out["plans"][0]["relifts"] == 1
    row = out["places"][0]
    assert row["source"] == "none" and [r["source"] for r in row["retries"]] == ["depth"]
    moved = plan_shift.shift(plan, place_correct.depth_move(look("seat_open", 0.0, -0.009)["reading"], refs.get(key), POSE0))
    lift0, lift1 = (next(e for e in p.entries if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
                    for p in (plan, moved))
    lifts = lifts_of(ex)
    assert len(lifts) == 2 and carries(lifts[:1], lift0.rows) and carries(lifts[1:], lift1.rows)
    assert not carries(lifts[1:], lift0.rows)
    back = [c for c in ex.calls if c[0] == "move" and c[5] == "re-lift restore, reversed"]
    assert [c[1] for c in back] == ["back along execute_path", "back along move_to_start", "back along move_to_pre_start"]
    push = next(e for e in moved.entries if e.kind == "arm" and e.side == "right" and e.stage == "execute_path")
    assert carries(sent_rows(ex, "right", "execute_path"), push.rows)               # the rest: the moved plan's
    assert any("the config's retry offsets are not used" in t for t in said)
    assert "taught" not in row                                                      # a re-lift never teaches


def test_in_a_sequence_the_re_lift_takes_the_cameras_value_too(tmp_path):
    """The same in a sequence (the checks before the lift not asked: the run's entries are the plan's without them)."""
    refs = place_correct.References(tmp_path / "refs.json")
    key = reference("seat_open", refs)
    plan = mechanism.Plan("seat_open")
    ex = LookExec(["seat_closed", "seat_lifted", "seat_open"], [look("seat_open", 0.001), look("seat_open", 0.0, -0.009)])
    out, said = run("seat_open", ex, place_correct.Correction(refs=refs, collide=False), start_checks=False)
    assert out["plans"][0]["relifts"] == 1 and out["places"][0]["retries"][0]["source"] == "depth"
    moved = plan_shift.shift(plan, place_correct.depth_move(look("seat_open", 0.0, -0.009)["reading"], refs.get(key), POSE0))
    lift1 = next(e for e in moved.entries if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
    assert carries(lifts_of(ex)[1:], lift1.rows)


def test_open_the_lids_retries_take_the_cameras_value_not_the_ladder(tmp_path):
    """Open the lid's lift missed twice: re-lift 1 where the camera reads the commode (10 mm to the right, and 20 mm along
    x, left at the plan's: the lid shut from pose 4 reads x poorly), re-lift 2 that lift again (the commode read there
    again: nothing new to correct); no pre-planned rung, no retry offset of the config's."""
    refs = place_correct.References(tmp_path / "refs.json")
    key = reference("lid_open", refs)
    plan = mechanism.Plan("lid_open")
    reading = look("lid_open", 0.020, -0.010)
    ex = LookExec(["lid_closed", "lid_closed", "lid_closed", "lid_lifted", "lid_open"], [look("lid_open", 0.001), reading])
    out, said = run("lid_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert out["plans"][0]["outcome"] == "done" and out["plans"][0]["relifts"] == 2
    d = place_correct.decide(None, reading, refs.get(key), lambda a, b: plan_shift.hands_apart_mm(plan, a, b))
    assert d.source == "depth" and d.move.dx_m == 0.0 and abs(d.move.dy_m + 0.010) < 1e-3
    lift = next(e for e in plan_shift.shift(plan, d.move).entries
                if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
    lifts = lifts_of(ex)
    assert len(lifts) == 3 and carries(lifts[1:2], lift.rows) and carries(lifts[2:], lift.rows)
    notes = [c[5] for c in ex.calls if c[0] == "move" and c[5]]
    assert not any("pre-planned" in n for n in notes) and "re-lift 1, moved by the camera" in notes and "re-lift 2" in notes
    assert [r["source"] for r in out["places"][0]["retries"]] == ["depth", "kept"]


def test_without_a_reference_a_missed_lift_is_lifted_again_as_it_was(tmp_path):
    """Until a run of the task has passed with nothing moved (no reference look: each task's first runs at the lab), the
    camera has no value to give: a missed lift is lifted again as it was made, no rung."""
    plan = mechanism.Plan("lid_open")
    ex = LookExec(["lid_closed", "lid_closed", "lid_lifted", "lid_open"], look("lid_open", 0.0, -0.010))
    out, said = run("lid_open", ex, place_correct.Correction(refs=place_correct.References(tmp_path / "refs.json"),
                                                             collide=False))
    assert out["plans"][0]["relifts"] == 1 and out["plans"][0]["outcome"] == "done"
    lift = next(e for e in plan.entries if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
    assert len(lifts_of(ex)) == 2 and all(carries([x], lift.rows) for x in lifts_of(ex))
    retry = out["places"][0]["retries"][0]
    assert retry["source"] == "kept" and "no reference look yet" in retry["words"]


def test_re_lifts_stop_for_a_person_after_the_configs_count_or_beyond_the_bounds(tmp_path):
    """Open the lid misses four times: after its config's 3 re-lifts the run stops for a person. A retry whose reading
    is beyond plan_shift's bounds stops before that re-lift, the arm back along its lift."""
    refs = place_correct.References(tmp_path / "refs.json")
    ex = LookExec(["lid_closed"] * 5, look("lid_open", 0.0))
    with pytest.raises(UserError, match="still NOT lid_lifted after 3 re-lift") as err:
        run("lid_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert err.value.code == "escalate" and len(lifts_of(ex)) == 4
    reference("seat_open", refs)
    ex = LookExec(["lid_open", "seat_closed", "seat_closed"], [look("seat_open", 0.001), look("seat_open", 0.0, -0.035)])
    with pytest.raises(UserError) as err:
        run("seat_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert err.value.code == "place" and "before re-lift 1" in str(err.value) and "beyond 30 mm" in str(err.value)
    assert len(lifts_of(ex)) == 1 and [c[1] for c in ex.calls if c[0] == "move"][-1] == "back along move_to_pre_start"


def test_every_task_fails_three_times_before_a_person_is_called(tmp_path):
    """The owner, 2 Nov: "by default number of retries after failure is three; you should never abort on the first
    failure; this applies to all mechanism tasks". Open the seat's lift (its config: 1 re-lift) is lifted again up to 3
    times; Close the seat's end check (its config: abort) runs the task again from its start, its start check asked
    (here in a sequence too), up to 3 times, then a person."""
    refs = place_correct.References(tmp_path / "refs.json")
    ex = LookExec(["lid_open", "seat_closed"] + ["seat_closed"] * 3 + ["seat_lifted", "seat_open"], look("seat_open", 0.0))
    out, said = run("seat_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert out["plans"][0]["relifts"] == 3 and out["plans"][0]["outcome"] == "done"
    # Close the seat in a sequence: its end check fails once, it runs again (its start check asked: the seat still open)
    ex = LookExec(["seat_open", "seat_open", "seat_closed"], look("seat_close", 0.0))
    out, said = run("seat_close", ex, place_correct.Correction(refs=refs, collide=False), start_checks=False)
    assert [(p["plan"], p["outcome"]) for p in out["plans"]] == [("seat_close", "again"), ("seat_close", "done")]
    asked = [c[1] for c in ex.calls if c[0] == "ask"]
    assert asked == ["verify_seat_closed", "verify_seat_open", "verify_seat_closed"]
    assert [c[1:3] for c in ex.calls if c[0] == "approach"] == [("seat_close", "start"), ("seat_close", "start")]
    assert any("runs again from its start (1/3)" in t for t in said)
    # the seat never closes: three runs again, then a person
    ex = LookExec(["seat_open"] + ["seat_open", "seat_open"] * 3, look("seat_close", 0.0))
    with pytest.raises(UserError, match="still NOT seat_closed after running Close the seat again 3 time") as err:
        run("seat_close", ex, place_correct.Correction(refs=refs, collide=False), start_checks=False)
    assert err.value.code == "escalate"


def test_a_task_run_again_reads_where_the_part_is_first(tmp_path):
    """Run again after its end check failed, a task's start check decides: the part where the plan ends after all, the
    outcome holds (done, nothing moved); neither, a person. The camera's lifted part counts as open to the start check,
    so the depth's parts reading decides it too: a part between closed and open stops the run again before any working
    move (its plan starts from a part at rest)."""
    refs = place_correct.References(tmp_path / "refs.json")
    ex = LookExec(["lid_open", "lid_lifted", "lid_closed"], look("lid_close", 0.0))
    out, said = run("lid_close", ex, place_correct.Correction(refs=refs, collide=False))
    assert [(p["plan"], p["outcome"]) for p in out["plans"]] == [("lid_close", "again"), ("lid_close", "skipped")]
    assert any("lid_closed after all" in t for t in said)
    ex = LookExec(["lid_open", "lid_lifted", "seat_lifted"], look("lid_close", 0.0))
    with pytest.raises(UserError, match="between where Close the lid starts and where it ends") as err:
        run("lid_close", ex, place_correct.Correction(refs=refs, collide=False))
    assert err.value.code == "escalate"
    held = dict(look("lid_close", ok=False), note="the lid at 45 deg, the seat at 0: a part between closed and open "
                                                   "moves with what holds it, no place")
    ex = LookExec(["lid_open", "lid_lifted", "lid_lifted"], [look("lid_close", 0.0), held])
    with pytest.raises(UserError, match="the lid at 45 deg, the seat at 0, a part between closed and open") as err:
        run("lid_close", ex, place_correct.Correction(refs=refs, collide=False))
    assert err.value.code == "escalate" and len(lifts_of(ex)) == 1          # the run again moved nothing of its work


def test_the_task_run_again_takes_the_cameras_value_at_its_own_look(tmp_path):
    """Close the seat's end check fails; run again, its first look reads the commode 8 mm to the right of where the
    offline plan works: the run again is the plan made from the camera's value."""
    refs = place_correct.References(tmp_path / "refs.json")
    key = reference("seat_close", refs)
    plan = mechanism.Plan("seat_close")
    ex = LookExec(["seat_open", "seat_open", "seat_open", "seat_closed"], [look("seat_close", 0.001), look("seat_close", 0.0, -0.008)])
    out, said = run("seat_close", ex, place_correct.Correction(refs=refs, collide=False))
    assert [(p["plan"], p["outcome"]) for p in out["plans"]] == [("seat_close", "again"), ("seat_close", "done")]
    assert [r["source"] for r in out["places"]] == ["none", "depth"]
    moved = plan_shift.shift(plan, place_correct.depth_move(look("seat_close", 0.0, -0.008)["reading"], refs.get(key), POSE0))
    pull0, pull1 = (next(e for e in p.entries if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
                    for p in (plan, moved))
    pulls = lifts_of(ex)
    assert len(pulls) == 2 and carries(pulls[:1], pull0.rows) and carries(pulls[1:], pull1.rows)


def test_the_references_are_seeded_from_the_labs_looks_of_the_offline_plans(tmp_path):
    """tools/placement_replay.py --seed: each task's first look's reference, the median of the lab's looks of that pose,
    pole and state (the commode where the offline plans work); those of other looks are left out."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
    import placement_replay

    def row(pose, state, dy, day="2026-10-30"):
        return dict(ok=True, pose=pose, state=state, pole_mm=149.1, day=day, dx_m=-0.007, dy_m=dy, dz_m=0.0, dyaw_deg=6.0,
                    lid_deg=110.0, seat_deg=0.0)

    res = [row("inspection_pose_1", "lid open, seat open", dy) for dy in (0.033, 0.035, 0.036)]
    res += [row("inspection_pose_1", "lid shut", 0.030), dict(row("inspection_pose_4", "lid shut", 0.017), ok=False)]
    placement_replay.seed(res, tmp_path / "refs.json")
    refs = place_correct.References(tmp_path / "refs.json")
    got = refs.get(place_correct.look_key("commode_c", "inspection_pose_1", 150.0, "lid open, seat open"))
    assert got["task"] == "seat_close" and got["reading"]["dy_m"] == 0.035 and got["frames"] == 3
    assert len(json.loads((tmp_path / "refs.json").read_text())["looks"]) == 1
    assert refs.has_task("seat_close") and not refs.has_task("lid_open")


def test_the_labs_references_are_there_from_the_first_run():
    """Resources/Logs/place_reference.json: the four tasks' first looks (pole 150 mm), as the lab's looks of 24-31 Oct
    read the commode where the offline plans work."""
    refs = place_correct.References(Path(__file__).resolve().parents[1] / "Resources" / "Logs" / place_correct.REFERENCE_FILE)
    for task, (pose, state) in LOOKS.items():
        got = refs.get(place_correct.look_key("commode_c", pose, 150.0, state))
        assert got is not None and got["task"] == task and got["frames"] >= 100, task


class Lanes:
    """Correction.submit in threads, one a lane as Lite's processes are (in_workers), each job recorded: (lane, what,
    task, the move's dx and dy mm)."""

    def __init__(self):
        from concurrent.futures import ThreadPoolExecutor

        self.pools = {lane: ThreadPoolExecutor(1) for lane in place_correct.LANES}
        self.jobs = []

    def __call__(self, lane, fn, *args):
        move = args[3] if fn is place_correct.generate else None
        self.jobs.append((lane, fn.__name__, args[0], None if move is None else (round(move[0] * 1000, 1), round(move[1] * 1000, 1))))
        return self.pools[lane].submit(fn, *args)

    def drain(self, lane):
        """Every job of ``lane`` made (one thread: those before a last one)."""
        self.pools[lane].submit(lambda: None).result()

    def made(self, lane):
        return [(task, mm) for at, what, task, mm in self.jobs if what == "generate" and at == lane]


def test_the_runs_next_tasks_are_made_while_one_runs(tmp_path):
    """The UGV 12 mm further off: as the run starts each of its tasks is queued for its moved plan, in the run's order,
    in a lane of their own (the owner, 1 Nov: "generate the first task, as it executes be generating the other tasks");
    each task takes its own from there, the second made while the first ran: nothing is made as a task starts."""
    lanes = Lanes()
    ugv = plan_shift.Move(0.012)
    corr = place_correct.Correction(ugv=ugv, refs=place_correct.References(tmp_path / "refs.json"), collide=False,
                                    submit=lanes)
    corr.ahead(["seat_open", "seat_close"])
    assert lanes.made("plans_ahead") == [("seat_open", (12.0, 0.0)), ("seat_close", (12.0, 0.0))]
    said = {}
    for task in ("seat_open", "seat_close"):
        plan = mechanism.Plan(task)
        ex = LookExec([c["expect"] for c in plan.checks()], look(task, ok=False))
        out, said[task] = run(task, ex, corr)
        carry = next(e for e in plan_shift.shift(plan, ugv).entries
                     if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
        assert carries(sent_rows(ex, "left", "execute_path"), carry.rows)
        lanes.drain("plans_ahead")                     # a task runs 14 s, a plan is made in 1 to 3: the next made meanwhile
    assert lanes.made("plans_now") == [] and len(lanes.made("plans_ahead")) == 2
    assert any("made while the run went on: the hands" in t for t in said["seat_close"])
    assert [r["task"] for r in corr.log] == ["seat_open", "seat_close"] and corr.at == 1


def test_the_cameras_move_on_one_task_is_made_for_the_next(tmp_path):
    """No placement from the UGV: the camera reads the commode 8 mm to the left at Open the seat's look; that task's plan
    moved there is made at once (the running task's lane), and the run's next task, Close the seat, for the same move
    meanwhile (the other lane). Its own look reading the commode 8.5 mm to the left (the hands within 5 mm of the move
    made), Close the seat runs that plan: nothing is made as it runs."""
    refs = place_correct.References(tmp_path / "refs.json")
    key = reference("seat_open", refs)
    reference("seat_close", refs)
    lanes = Lanes()
    corr = place_correct.Correction(refs=refs, collide=False, submit=lanes)
    corr.ahead(["seat_open", "seat_close"])
    assert lanes.jobs == []                            # the plans' own placement: nothing to make before a look
    plan = mechanism.Plan("seat_open")
    out, said = run("seat_open", LookExec([c["expect"] for c in plan.checks()], look("seat_open", 0.0, 0.008)), corr)
    assert out["places"][0]["source"] == "depth"
    lanes.drain("plans_ahead")
    assert [t for t, _ in lanes.made("plans_now")] == ["seat_open"]
    assert [t for t, _ in lanes.made("plans_ahead")] == ["seat_close"]
    plan = mechanism.Plan("seat_close")
    ex = LookExec([c["expect"] for c in plan.checks()], look("seat_close", 0.0, 0.0085))
    out, said = run("seat_close", ex, corr)
    assert [t for t, _ in lanes.made("plans_now")] == ["seat_open"]           # none for Close the seat's look
    row = out["places"][-1]
    assert row["task"] == "seat_close" and row["source"] == "depth" and row["move"] != row["depth"]
    assert any("made ahead for an earlier task's look" in t for t in said)
    first = place_correct.depth_move(look("seat_open", 0.0, 0.008)["reading"], refs.get(key), POSE0)
    carry = next(e for e in plan_shift.shift(plan, first).entries
                 if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
    assert carries(sent_rows(ex, "left", "execute_path"), carry.rows)
    assert row["move"] == first.words()


def test_a_plan_made_in_a_process_of_its_own_is_the_one_made_here():
    """In a run the moved plans are made in Lite's own processes (in_workers, workers.pool): the entries shift() makes
    here, the re-lifts moved here when asked for; a refusal comes back as it was raised, its code with it."""
    from alix_studio_lite.server import workers

    plan = mechanism.Plan("lid_open")
    move = plan_shift.Move(0.012, -0.004)
    try:
        parts = place_correct.in_workers(place_correct.LANES[1], place_correct.generate, plan.name, str(plan.path),
                                         place_correct._digest(plan), place_correct._move_args(move), None, "commode_c",
                                         1400, False).result(timeout=120)
        made, here = plan_shift.assemble(plan, move, parts["entries"], parts["report"], parts["state"]), plan_shift.shift(plan, move)
        assert [e.rows for e in made.entries] == [e.rows for e in here.entries] and made.shift_report == here.shift_report
        assert [e.rows for e in made.rung_block(2, "left")] == [e.rows for e in here.rung_block(2, "left")]
        with pytest.raises(UserError) as err:
            place_correct.in_workers(place_correct.LANES[1], place_correct.generate, plan.name, str(plan.path),
                                     place_correct._digest(plan), place_correct._move_args(plan_shift.Move(0.04)), None,
                                     "commode_c", 1400, False).result(timeout=120)
        assert err.value.code == "shift" and "beyond" in str(err.value)
    finally:
        workers.shutdown()
