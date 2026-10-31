"""A lid or seat task placed by its two sources (the owner, 31 Oct): the UGV's reported placement first, compensated by
the wrist camera's reading at the task's first look against the reference look (server/place_correct.py)."""
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


def test_the_depth_decides_against_the_ugv(tmp_path):
    """No camera, no reading, no reference: the UGV's alone. The two agreeing to the dead band: the UGV's runs; apart by
    more (within the UGV's 10 mm and the look's 2): the camera's; further: stopped. A look the camera reads poorly
    (Open the lid's: the lid's front behind the glove) only checks the UGV's, to its wider scatter."""
    refs = place_correct.References(tmp_path / "refs.json")
    plan = mechanism.Plan("seat_close")
    hands = lambda a, b: plan_shift.hands_apart_mm(plan, a, b)          # noqa: E731
    ugv = plan_shift.Move(0.012, 0.0)
    assert place_correct.decide(ugv, None, None, hands).move == ugv
    assert place_correct.decide(ugv, look("seat_close", ok=False), None, hands).level == "warn"
    assert "no reference look yet" in place_correct.decide(ugv, look("seat_close", 0.012), None, hands).words
    ref = refs.get(reference("seat_close", refs))
    d = place_correct.decide(ugv, look("seat_close", 0.014, 0.001), ref, hands)
    assert d.source == "ugv" and d.move == ugv
    d = place_correct.decide(ugv, look("seat_close", 0.012, 0.008), ref, hands)
    assert d.source == "depth" and abs(d.move.dy_m - 0.008) < 1e-3 and abs(d.move.dx_m - 0.012) < 1e-3
    d = place_correct.decide(ugv, look("seat_close", 0.012, 0.015), ref, hands)
    assert d.refuse and "disagree" in d.refuse and d.move is None
    ref = refs.get(reference("lid_open", refs))
    d = place_correct.decide(ugv, look("lid_open", 0.012, 0.010), ref, hands)
    assert d.source == "ugv" and d.move == ugv and "the lid's front edge" in d.words
    assert place_correct.decide(ugv, look("lid_open", 0.012, 0.019), ref, hands).refuse


def test_the_ugvs_change_since_the_reference_is_what_the_camera_must_agree_with(tmp_path):
    """A reference taught with the UGV 3 mm off its place (within the dead band, nothing moved): today's placement is
    compared with the camera by its change since."""
    refs = place_correct.References(tmp_path / "refs.json")
    ref = refs.get(reference("lid_close", refs, ugv=dict(dx_m=0.003, dy_m=0.0)))
    plan = mechanism.Plan("lid_close")
    d = place_correct.decide(plan_shift.Move(0.015), look("lid_close", 0.012), ref,
                             lambda a, b: plan_shift.hands_apart_mm(plan, a, b))
    assert "0.0 mm apart" in d.words


class LookExec(FakeExec):
    """FakeExec with a camera that reads the commode's place at the first look (RealExec.look_frame / read_look) and
    the hook a sequence's look waits on."""

    def __init__(self, answers, reading):
        super().__init__(answers)
        self.reading, self.look_after, self.looks = reading, None, []

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
        return dict(self.reading)


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


def test_the_camera_compensates_the_ugv_and_stops_on_a_disagreement(tmp_path):
    """No placement from the UGV (the plans' own assumed): the camera reads the commode 8 mm to the left at Open the
    seat's first look: the plan's working moves from there on are the plan moved 8 mm, and nothing before them is. 15 mm
    off: the run stops before any working move, said."""
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
    ex = LookExec(answers, look("seat_open", 0.0, 0.015))
    with pytest.raises(UserError) as err:
        run("seat_open", ex, place_correct.Correction(refs=refs, collide=False))
    assert err.value.code == "place" and "before its working moves" in str(err.value)
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


def test_a_reference_of_another_plan_is_not_used(tmp_path):
    """Re-planned (its file's digest another), a task runs on the UGV's alone until a run of the new plan teaches it."""
    refs = place_correct.References(tmp_path / "refs.json")
    key = reference("lid_close", refs)
    refs.teach(key, dict(refs.get(key), plan="another"))
    plan = mechanism.Plan("lid_close")
    ex = LookExec([c["expect"] for c in plan.checks()], look("lid_close", 0.0, 0.009))
    out, said = run("lid_close", ex, place_correct.Correction(refs=refs, collide=False))
    assert out["places"][0]["source"] == "none" and any("another version of the plan" in t for t in said)


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
