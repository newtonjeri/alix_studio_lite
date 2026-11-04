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

pytestmark = pytest.mark.usefixtures("windows_rules")   # the rules before 4 Nov: gap_decider "windows"

POSE0 = np.array([[-1.0, 0.0, 0.0, 1.005], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.42], [0.0, 0.0, 0.0, 1.0]])
LOOKS = {"seat_close": ("inspection_pose_1", "lid open, seat open"), "lid_close": ("inspection_pose_1", "lid open, seat down"),
         "seat_open": ("inspection_pose_4", "lid open, seat down"), "lid_open": ("inspection_pose_4", "lid shut")}
REF = dict(dx_m=0.012, dy_m=-0.015, dz_m=0.002, dyaw_deg=-3.1, lid_deg=110.0, seat_deg=110.0)    # the camera's mount in it


@pytest.fixture(autouse=True)
def camera_place_on(monkeypatch):
    """The camera's place check (place_correct.CAMERA_PLACE), off since 5 Nov: its code kept and tested on here; the
    tests of it off ask for ``camera_place_off``."""
    monkeypatch.setattr(place_correct, "CAMERA_PLACE", True)


@pytest.fixture
def camera_place_off(monkeypatch):
    monkeypatch.setattr(place_correct, "CAMERA_PLACE", False)


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
    assert abs(d.depth.dx_m - 0.020) < 1e-3 and "its x and yaw left at" in d.words      # what the camera read, kept
    assert place_correct.decide(ugv, look("lid_open", 0.030, 0.001), ref, lid_hands).source == "ugv"   # only x off
    d = place_correct.decide(ugv, look("lid_open", 0.012, 0.001, 4.5), ref, lid_hands)   # 3 Nov: +4.5 deg, a bad turn
    assert d.source == "ugv" and d.refuse is None and abs(d.depth.dyaw_deg - 4.5) < 0.1


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


@pytest.fixture
def camera_relifts(monkeypatch):
    """The camera's re-lifts (the owner, 2 Nov), off since 5 Nov (mechanism.CAMERA_RELIFTS): their code kept, tested on."""
    monkeypatch.setattr(mechanism, "CAMERA_RELIFTS", True)


def test_a_missed_lift_re_lifts_by_the_task_configurations_rungs(tmp_path):
    """The user, 5 Nov: "for now ... just reuse the rungs in the task configuration" (02:40, the camera's re-lift made its
    first move 87 degrees from where the arm stood after going back: refused). Open the lid, its first look within the
    tolerance, its lift missed twice: re-lifts 1 and 2 are the pre-planned rungs of its config's retry_offsets, each
    from where the lift before it ended; the camera reads nothing again, nothing goes back along the lift."""
    refs = place_correct.References(tmp_path / "refs.json")
    reference("lid_open", refs)
    plan = mechanism.Plan("lid_open")
    ex = LookExec(["lid_closed", "lid_closed", "lid_closed", "lid_lifted", "lid_open"], look("lid_open", 0.001))
    out, said = run("lid_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert out["plans"][0]["outcome"] == "done" and out["plans"][0]["relifts"] == 2
    notes = [c[5] for c in ex.calls if c[0] == "move" and c[5]]
    assert "re-lift 1, pre-planned" in notes and "re-lift 2, pre-planned" in notes
    assert not any("moved by the camera" in n or n == "re-lift restore, reversed" for n in notes)
    rung = [next(e for e in plan.rung_block(k, "left") if e.stage == "execute_path") for k in (1, 2)]
    lifts = lifts_of(ex)
    assert len(lifts) == 3 and carries(lifts[1:2], rung[0].rows) and carries(lifts[2:], rung[1].rows)
    assert any("the task configuration's retry_offsets" in t for t in said)
    assert not out["places"][0].get("retries")                                    # the camera asked nothing again


def test_a_lift_the_camera_moved_re_lifts_by_the_rung_bridged_from_where_it_ended(tmp_path):
    """The camera moved the plan (the commode 10 mm off along y, beyond the 5 mm): its rungs are moved with it (plan_shift),
    so re-lift 1 is the rung moved the same way, starting where the moved lift ended (a gap up to 3 degrees bridged as a
    crept start is); the rest of the plan stays the moved one."""
    refs = place_correct.References(tmp_path / "refs.json")
    key = reference("lid_open", refs)
    plan = mechanism.Plan("lid_open")
    reading = look("lid_open", 0.0, -0.010)
    ex = LookExec(["lid_closed", "lid_closed", "lid_lifted", "lid_open"], reading)
    out, said = run("lid_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert out["plans"][0]["outcome"] == "done" and out["plans"][0]["relifts"] == 1
    d = place_correct.decide(None, reading, refs.get(key), lambda a, b: plan_shift.hands_apart_mm(plan, a, b))
    moved = plan_shift.shift(plan, d.move)
    lift = next(e for e in moved.entries if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
    rung = next(e for e in moved.rung_block(1, "left") if e.stage == "execute_path")
    lifts = lifts_of(ex)
    assert len(lifts) == 2 and carries(lifts[:1], lift.rows) and carries(lifts[1:], rung.rows)
    assert not carries(lifts[1:], next(e for e in plan.rung_block(1, "left") if e.stage == "execute_path").rows)
    push = next(e for e in moved.entries if e.kind == "arm" and e.side == "right" and e.stage == "execute_path")
    assert carries(sent_rows(ex, "right", "execute_path"), push.rows)               # the rest: the moved plan's
    gap = mechanism._gap_deg(next(e for e in moved.rung_block(1, "left") if e.kind == "arm").first(),
                             next(e for e in reversed(moved.entries[:moved.entries.index(lift) + 1])
                                  if e.kind == "arm" and e.side == "left").last())
    assert gap <= mechanism.BRIDGE_MAX_DEG
    assert (gap > mechanism.start_tol_deg()) == any("bridged at 40 %" in t for t in said)


def test_a_task_with_no_retry_offsets_lifts_again_the_same(tmp_path):
    """Open the seat (its config: retry_lift, no retry_offsets): a missed lift goes back along the lift it made and lifts
    again the same, the camera's second reading (9 mm off) not used."""
    refs = place_correct.References(tmp_path / "refs.json")
    reference("seat_open", refs)
    plan = mechanism.Plan("seat_open")
    ex = LookExec(["lid_open", "seat_closed", "seat_closed", "seat_lifted", "seat_open"],
                  [look("seat_open", 0.001), look("seat_open", 0.0, -0.009)])
    out, said = run("seat_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert out["plans"][0]["outcome"] == "done" and out["plans"][0]["relifts"] == 1
    lift = next(e for e in plan.entries if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
    assert len(lifts_of(ex)) == 2 and all(carries([x], lift.rows) for x in lifts_of(ex))
    back = [c[1] for c in ex.calls if c[0] == "move" and c[5] == "re-lift restore, reversed"]
    assert back == ["back along execute_path", "back along move_to_start", "back along move_to_pre_start"]
    assert not any("moved by the camera" in (c[5] or "") for c in ex.calls if c[0] == "move")


def lifts_of(ex, side="left"):
    return sent_rows(ex, side, "execute_path")


@pytest.mark.usefixtures("camera_relifts")
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


@pytest.mark.usefixtures("camera_relifts")
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


@pytest.mark.usefixtures("camera_relifts")
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


@pytest.mark.usefixtures("camera_relifts")
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


@pytest.mark.usefixtures("camera_relifts")
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
    times; Close the lid's end check (its config: abort) runs the task again from its start, its start check asked
    (here in a sequence too), up to 3 times, then a person."""
    refs = place_correct.References(tmp_path / "refs.json")
    ex = LookExec(["lid_open", "seat_closed"] + ["seat_closed"] * 3 + ["seat_lifted", "seat_open"], look("seat_open", 0.0))
    out, said = run("seat_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert out["plans"][0]["relifts"] == 3 and out["plans"][0]["outcome"] == "done"
    # Close the lid in a sequence (its end check: abort; Close the seat's is retry_close since 5 Nov): its end check fails
    # once, it runs again (its start check asked: the lid still open)
    ex = LookExec(["lid_open", "lid_open", "lid_closed"], look("lid_close", 0.0))
    out, said = run("lid_close", ex, place_correct.Correction(refs=refs, collide=False), start_checks=False)
    assert [(p["plan"], p["outcome"]) for p in out["plans"]] == [("lid_close", "again"), ("lid_close", "done")]
    asked = [c[1] for c in ex.calls if c[0] == "ask"]
    assert asked == ["verify_lid_closed", "verify_lid_open", "verify_lid_closed"]
    assert [c[1:3] for c in ex.calls if c[0] == "approach"] == [("lid_close", "start"), ("lid_close", "start")]
    assert any("runs again from its start (1/3)" in t for t in said)
    # the lid never closes: three runs again, then a person
    ex = LookExec(["lid_open"] + ["lid_open", "lid_open"] * 3, look("lid_close", 0.0))
    with pytest.raises(UserError, match="still NOT lid_closed after running Close the lid again 3 time") as err:
        run("lid_close", ex, place_correct.Correction(refs=refs, collide=False), start_checks=False)
    assert err.value.code == "escalate"


def test_a_task_run_again_reads_where_the_part_is_first(tmp_path):
    """Run again after its end check failed, a task's start check decides by the camera's classes: proving the
    opposite of where the task starts (where it ends) its outcome holds after all, nothing moved; a class that proves
    neither stops it for a person; one the camera cannot judge stops it too, as every check (23 Oct)."""
    refs = place_correct.References(tmp_path / "refs.json")
    ex = LookExec(["lid_open", "lid_lifted", "lid_closed"], look("lid_close", 0.0))
    out, said = run("lid_close", ex, place_correct.Correction(refs=refs, collide=False))
    assert [(p["plan"], p["outcome"]) for p in out["plans"]] == [("lid_close", "again"), ("lid_close", "skipped")]
    assert any("lid_closed after all" in t for t in said) and len(lifts_of(ex)) == 1
    ex = LookExec(["lid_open", "lid_lifted", "seat_lifted"], look("lid_close", 0.0))
    with pytest.raises(UserError, match="between where Close the lid starts and where it ends") as err:
        run("lid_close", ex, place_correct.Correction(refs=refs, collide=False))
    assert err.value.code == "escalate"
    ex = LookExec(["lid_open", "lid_lifted", None], look("lid_close", 0.0))
    with pytest.raises(UserError) as err:
        run("lid_close", ex, place_correct.Correction(refs=refs, collide=False))
    assert err.value.code == "escalate" and len(lifts_of(ex)) == 1


def test_the_task_run_again_takes_the_cameras_value_at_its_own_look(tmp_path):
    """Close the lid's end check fails; run again, its first look reads the commode 8 mm to the right of where the
    offline plan works: the run again is the plan made from the camera's value."""
    refs = place_correct.References(tmp_path / "refs.json")
    key = reference("lid_close", refs)
    plan = mechanism.Plan("lid_close")
    ex = LookExec(["lid_open", "lid_open", "lid_open", "lid_closed"], [look("lid_close", 0.001), look("lid_close", 0.0, -0.008)])
    out, said = run("lid_close", ex, place_correct.Correction(refs=refs, collide=False))
    assert [(p["plan"], p["outcome"]) for p in out["plans"]] == [("lid_close", "again"), ("lid_close", "done")]
    assert [r["source"] for r in out["places"]] == ["none", "depth"], [r["words"] for r in out["places"]]
    moved = plan_shift.shift(plan, place_correct.depth_move(look("lid_close", 0.0, -0.008)["reading"], refs.get(key), POSE0))
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


def test_the_labs_references_read():
    """Resources/Logs/place_reference.json: the lab's reference looks. Those of 24-31 Oct were cleared on 4 Nov night
    (the camera's mount was refixed at ~15:32 and every look since read the commode 12 to 39 mm off them); the next run
    of each task that passes with nothing moved teaches its own. Whatever is there names its task and its look."""
    refs = place_correct.References(Path(__file__).resolve().parents[1] / "Resources" / "Logs" / place_correct.REFERENCE_FILE)
    tasks = set(LOOKS)
    for key, got in refs._load().items():
        assert got["task"] in tasks and key.startswith("commode_c|") and "reading" in got, key


class LegExec(LookExec):
    """LookExec running the plan's two-arm stretches (Runner's move_together) one move after the other."""

    def move_together(self, leg):
        for e, bridge, _nxt in leg.order:
            self.move(e, bridge=bridge)


def test_a_closings_look_in_a_sequence_comes_before_its_working_moves(tmp_path):
    """2 Nov (Isaac, the lid and seat tasks of S2's sequence): Close the lid's first two-arm stretch runs the camera arm
    to its look and the left arm to its start; in a sequence (no check to take the look at) the look was taken before
    that stretch, the camera arm still at rest, and no correction could be read. No stretch now reaches past the first
    move a correction changes until the look is taken: the camera arm at its inspection pose, nothing of the work yet."""
    refs = place_correct.References(tmp_path / "refs.json")
    reference("lid_close", refs)
    ex = LegExec(["lid_closed"], look("lid_close", -0.010))
    out, said = run("lid_close", ex, place_correct.Correction(refs=refs, collide=False), start_checks=False)
    assert ex.looks and ex.looks[0][0] == "take"
    before = ex.calls[:ex.looks[0][2]]
    assert any(c[0] == "move" and c[2] == "right" and "move_to_inspect" in c[1] for c in before)
    assert not any(c[0] == "move" and c[2] == "left" and c[3] == "arm" for c in before)
    assert out["places"][0]["source"] == "depth" and out["plans"][0]["outcome"] == "done"


SEAT_3_NOV = Path(__file__).parent / "data" / "gap_checks" / "seat_at_place" / "20261103_192737_724_gap_check_seat_lifted_refused"
PLACE_3_NOV = dict(dx_m=0.0118, dy_m=0.0244, dz_m=0.0076, dyaw_deg=3.53)   #: Open the seat's first look that day, absolute


def frame_3_nov():
    from alix_studio_lite.server.gap_check import DepthFrame, Intrinsics

    rec = json.loads(SEAT_3_NOV.with_suffix(".json").read_text())
    k = rec["intrinsics"]
    return rec, DepthFrame(depth=np.load(str(SEAT_3_NOV) + "_depth_mm.npz")["depth_mm"],
                           intrinsics=Intrinsics(k["fx"], k["fy"], k["cx"], k["cy"], k["width"], k["height"]),
                           cam_to_base=np.asarray(rec["camera_to_world"]), hands={n: tuple(v) for n, v in rec["hands"].items()})


def test_the_place_is_read_on_the_cameras_millimetres():
    """The D435 gives z16 millimetres: read as metres the commode was found nowhere (the lab, 3 Nov, both runs' looks: "only
    0 depth pixels where the lid's or the seat's angle shows"). Converted as the checks convert, the place reads as from
    metres; the checks judging at a read place (at_place) do not move it."""
    import test_fixture_place as tf
    from alix_studio_lite.server.gap_check import GapCheck, load_params

    gc = GapCheck(params={**load_params(), "gap_cross_check": False}, records=None)
    frame = tf.frame(tf.SEAT_DOWN[1])
    metres = gc.place(frame)
    mm_frame = type(frame)(**{**frame.__dict__, "depth": np.clip(np.round(np.nan_to_num(np.asarray(frame.depth)) * 1000.0),
                                                                 0, 65535).astype(np.uint16)})
    gc.at_place(PLACE_3_NOV)
    mm = gc.place(mm_frame)
    assert metres["ok"] and mm["ok"], (metres["note"], mm["note"])
    assert all(abs(mm[a] - metres[a]) < 1.5e-3 for a in ("dx_m", "dy_m", "dz_m")) and abs(mm["dyaw_deg"] - metres["dyaw_deg"]) < 0.2


def test_the_seats_lifted_check_judges_where_the_first_look_read_the_commode():
    """The lab, 3 Nov, twice: Open the seat stopped at its lifted check, no decision in 5 frames (the hand over 77 % of
    the seat's window, its fit "seat_lifted and seat_closed explain the frame alike", margins +0.13 to +0.23 with the
    fixture at its scene place: the commode 10 mm from where the plans worked and the camera's mount on top). With the
    fixture where the task's first look read it, the same frame reads seat_lifted (all 10 of that day: margins +0.37 to
    +0.41)."""
    from alix_studio_lite.server.gap_check import GapCheck, load_params

    gc = GapCheck(params={**load_params(), "gap_lift_by_distance": False}, records=None)   # the rules of 3 Nov
    rec, frame = frame_3_nov()
    q = rec["context"]["arms"]["right"]["q_deg"]
    assert not gc.ask("seat_lifted", frame, q_deg=q).success                         # as at the lab
    gc.at_place(PLACE_3_NOV)
    a = gc.ask("seat_lifted", frame, q_deg=q)
    assert a.success and a.fixture_state == "seat_lifted" and a.state_confidence > 0.9, a.message
    assert gc.fixture_record()["placed"]["dy_m"] == PLACE_3_NOV["dy_m"]
    gc.at_place(None)
    assert not gc.ask("seat_lifted", frame, q_deg=q).success
    # since 4 Nov (the user) the lift question is a distance against the scene's closed top: lifted wherever the
    # commode was read, no fit and no place needed
    now = GapCheck(params=load_params(), records=None)
    a = now.ask("seat_lifted", frame, q_deg=q)
    assert a.success and a.fixture_state == "seat_lifted" and "by distance" in a.message, a.message


class AtExec(LookExec):
    """LookExec recording where the checks are told to judge the commode (RealExec.fixture_at)."""

    def fixture_at(self, reading):
        self.calls.append(("at", None if reading is None else dict(reading)))


def test_a_tasks_checks_judge_the_commode_where_its_first_look_read_it(tmp_path):
    """After its first look reads the commode's place, a task's checks judge the commode there (the lifted check
    included), and the next run starts at its scene place again; Open the lid's look (x and turn poorly read) leaves
    its checks at the scene place."""
    refs = place_correct.References(tmp_path / "refs.json")
    reference("seat_open", refs)
    plan = mechanism.Plan("seat_open")
    reading = look("seat_open", 0.002)
    ex = AtExec([c["expect"] for c in plan.checks()], reading)
    run("seat_open", ex, place_correct.Correction(refs=refs, collide=False))
    ats = [(k, c[1]) for k, c in enumerate(ex.calls) if c[0] == "at"]
    lifted = next(k for k, c in enumerate(ex.calls) if c[0] == "ask" and c[1] == "verify_seat_lifted")
    assert ats[0][1]["dx_m"] == reading["reading"]["dx_m"] and ats[0][0] < lifted and ats[-1][1] is None
    plan = mechanism.Plan("lid_open")
    ex = AtExec([c["expect"] for c in plan.checks()], look("lid_open", 0.002))
    run("lid_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert [c[1] for c in ex.calls if c[0] == "at"] == [None]


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


@pytest.fixture
def seat_close_rungs(tmp_path, monkeypatch):
    """Lite's plans with Close the seat's end check on retry_close (2 retries, the config's offsets) and its two
    whole-task rungs beside it (here copies of the plan), as ALIX's generation of 5 Nov makes them."""
    import shutil

    src = mechanism.PLANS
    plans = tmp_path / "plans"
    shutil.copytree(src, plans)
    (plans / "rungs").mkdir(exist_ok=True)

    def retry_close(path, stamp):
        data = json.loads(path.read_text(encoding="utf-8"))
        for s in data["sub_trajectories"]:
            for a in s.get("pre_actions") or []:
                if a.get("stage") == "verify_seat_closed":
                    a.update(on_fail="retry_close", max_retries=2, retry_offsets=[[0.005, 0.005, 0.0], [0.005, 0.01, -0.015]])
        data["timestamp_human"] = stamp
        return data

    main = plans / "seat_close_toppra.json"
    for k in (1, 2):
        (plans / "rungs" / f"seat_close_toppra.rung{k}.json").write_text(
            json.dumps(retry_close(main, f"2026-11-05 22:3{k}:00")), encoding="utf-8")
        for side in main.parent.glob("seat_close_toppra.g*"):
            shutil.copy(side, plans / "rungs" / side.name)
    main.write_text(json.dumps(retry_close(main, "2026-11-05 22:30:00")), encoding="utf-8")
    monkeypatch.setattr(mechanism, "PLANS", plans)
    return plans


def test_close_the_seat_runs_again_from_its_start_as_its_rung(tmp_path, seat_close_rungs):
    """The user, 5 Nov: Close the seat's end check on_fail retry_close, the left's pull again with the config's offsets:
    the seat not closed, Close the seat runs again from its start as its rung 1 (the whole task re-planned in ALIX with
    the left start pose moved 5 / 5 mm), its start check asked; closed then, done."""
    refs = place_correct.References(tmp_path / "refs.json")
    ex = LookExec(["seat_open", "seat_open", "seat_closed"], look("seat_close", 0.0))
    out, said = run("seat_close", ex, place_correct.Correction(refs=refs, collide=False), start_checks=False)
    assert [(p["plan"], p["outcome"], p.get("retry")) for p in out["plans"]] == [("seat_close", "again", None),
                                                                                 ("seat_close", "done", 1)]
    assert out["plans"][1]["title"] == "Close the seat, retry 1"
    assert [c[1] for c in ex.calls if c[0] == "ask"] == ["verify_seat_closed", "verify_seat_open", "verify_seat_closed"]
    assert any("runs again from its start with the left start pose moved" in t and "(its rung 1" in t and "(1/2)" in t
               for t in said), said
    assert any("seat_close_toppra.rung1.json" in t for t in said)


def test_close_the_seat_stops_for_a_person_after_its_rungs(tmp_path, seat_close_rungs):
    """The seat never closes: rung 1, rung 2 (the config's max_retries 2), then a person; a rung not in Lite stops the
    run before anything of it moves."""
    refs = place_correct.References(tmp_path / "refs.json")
    ex = LookExec(["seat_open"] + ["seat_open", "seat_open"] * 2, look("seat_close", 0.0))
    with pytest.raises(UserError, match="still NOT seat_closed after running Close the seat again 2 time") as err:
        run("seat_close", ex, place_correct.Correction(refs=refs, collide=False), start_checks=False)
    assert err.value.code == "escalate"
    (seat_close_rungs / "rungs" / "seat_close_toppra.rung1.json").unlink()
    ex = LookExec(["seat_open"], look("seat_close", 0.0))
    with pytest.raises(UserError, match="retry 1 .* is not in Lite") as err:
        run("seat_close", ex, place_correct.Correction(refs=refs, collide=False), start_checks=False)
    assert [c[1] for c in ex.calls if c[0] == "approach"].count("seat_close") == 1        # nothing of the retry moved


def test_with_the_cameras_place_check_off_a_far_reading_neither_stops_nor_moves_the_plan(tmp_path, camera_place_off):
    """The user, 5 Nov: "Please remove these camera position checks, they are causing the program to stop ... yet they
    are not that accurate" (Open the seat STOPPED before its working moves: the commode read 23.4 mm, -3.13 deg and
    -7.4 mm off). Off: the look reads nothing (no frame), the plan runs as made with the UGV's placement alone (none
    here), its checks judged at the scene place, no reference taught."""
    refs = place_correct.References(tmp_path / "refs.json")
    reference("seat_open", refs)
    plan = mechanism.Plan("seat_open")
    ex = LookExec(["lid_open", "seat_closed", "seat_lifted", "seat_open"], look("seat_open", -0.010, 0.0212, -3.13))
    out, said = run("seat_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert out["plans"][0]["outcome"] == "done"
    lift = next(e for e in plan.entries if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
    assert carries(lifts_of(ex), lift.rows)                                          # the plan as made
    assert ex.looks == []                                                            # no frame taken for the place
    assert any("the camera's place check is off" in t for t in said)
    assert not any("STOPPED" in t or "judge the commode where its first look" in t for t in said)
    assert out["places"][0]["source"] == "none" and "taught" not in out["places"][0]


def test_with_the_cameras_place_check_off_a_sequence_runs_its_tasks_as_made(tmp_path, camera_place_off):
    """In a sequence (the first check not asked) the same: no look, the closing's moves as made."""
    refs = place_correct.References(tmp_path / "refs.json")
    reference("seat_close", refs)
    plan = mechanism.Plan("seat_close")
    ex = LookExec(["seat_closed"], look("seat_close", 0.020, 0.0, 2.0))
    out, said = run("seat_close", ex, place_correct.Correction(refs=refs, collide=False), start_checks=False)
    assert out["plans"][0]["outcome"] == "done"
    pull = next(e for e in plan.entries if e.kind == "arm" and e.side == "left" and e.stage == "execute_path")
    assert carries(sent_rows(ex, "left", "execute_path"), pull.rows)
    assert ex.looks == []
