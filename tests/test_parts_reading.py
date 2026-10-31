"""The parts reading (the owner, 31 Oct: depth only, each part measured against the commode in the same frame;
GapCheckSession.read_parts, perception.yaml gap_decider): the whole-commode fit with the commode at its scene place and
nothing learned decides from the parts' angles. Lab frames from tests/data/gap_checks (depth in mm), each with its
state as its photo shows it, asked the question its check asked, as the runner asks it: the vote it gives
(lift_check.frame_vote) must be the photo's. tools/parts_replay.py asks every recorded frame (31 Oct: 1155 frames of
21-31 Oct, the lab's vote on 1081, never the opposite way, the other 74 as the photos show or undecided by rule)."""
import json
from pathlib import Path

import numpy as np
import pytest

from alix_studio_lite.server import mechanism
from alix_studio_lite.server.gap_check import DepthFrame, GapCheck, Intrinsics, load_params

DATA = Path(__file__).parent / "data" / "gap_checks"

#: (frame, the question, a closing's end check, the photo's vote): yes / no / none (undecided: no vote)
CASES = [
    ("20261031_112831_141_gap_check_lid_lifted_uncertain", "lid_lifted", False, "yes"),      # 11:28: the lab could not judge
    ("20261031_113639_217_gap_check_seat_lifted_refused", "seat_lifted", False, "yes"),      # 11:36: the glove holds it up
    ("20261024_114110_625_gap_check_lid_lifted_lid_closed", "lid_lifted", False, "no"),       # a missed lift: the lid flat
    ("20261031_110500_760_gap_check_lid_open_lid_open", "lid_open", False, "yes"),
    ("20261026_105236_063_gap_check_seat_open_seat_lifted", "seat_open", False, "yes"),       # standing against the lid
    ("20261024_163304_657_gap_check_seat_open_seat_closed", "seat_open", False, "no"),       # Open the seat left it down
    ("20261030_153742_229_gap_check_seat_closed_seat_lifted", "seat_closed", True, "no"),    # Close the seat left it open
    ("20261030_153635_137_gap_check_seat_closed_uncertain", "seat_closed", True, "yes"),     # 15:36: down, read 15 deg
    ("20261031_111355_303_gap_check_seat_closed_seat_closed", "seat_closed", True, "yes"),
    ("20261031_111629_932_gap_check_lid_closed_lid_closed", "lid_closed", True, "yes"),
    ("20261028_145829_087_gap_check_lid_closed_uncertain", "lid_closed", True, "yes"),       # closed, read exactly 25 deg
    ("20261029_191333_508_gap_check_lid_closed_uncertain", "lid_closed", True, "yes"),
    ("20261024_162704_298_gap_check_seat_lifted_seat_lifted", "seat_lifted", False, "none"),  # the seat under the lifted lid
]


def frame(stem: str) -> DepthFrame:
    base = next(d / stem for d in (DATA / "parts", DATA / "closing") if (d / f"{stem}.json").is_file())
    rec = json.loads(Path(f"{base}.json").read_text())
    depth = np.load(f"{base}.npz")["depth_mm"].astype(np.float32) / 1000.0
    k = rec["intrinsics"]
    return DepthFrame(depth=depth, intrinsics=Intrinsics(k["fx"], k["fy"], k["cx"], k["cy"], depth.shape[1], depth.shape[0]),
                      cam_to_base=np.asarray(rec["camera_to_world"], dtype=float), hands=rec.get("hands") or {}, source=rec["source"])


def session(**params) -> GapCheck:
    return GapCheck(params={**load_params(), "gap_cross_check": False, **params}, records=None)


def vote(asked: str, answer) -> str:
    return mechanism.lift_check().frame_vote({"check": "fixture_gap", "expect": asked}, answer) or "none"


@pytest.mark.parametrize("stem, asked, closing, want", CASES)
def test_each_lab_frame_reads_as_its_photo_shows(stem, asked, closing, want):
    a = session(gap_decider="parts").ask(asked, frame(stem), closing=closing)
    assert vote(asked, a) == want, a.message
    assert a.record["parts"]["decides"] and a.message.startswith("parts (depth only)")


def test_the_parts_reading_learns_nothing_and_asks_no_colour_model():
    gc = session(gap_decider="parts", gap_cross_check=True)
    asked = []
    for stem, question, closing, _ in CASES:
        f = frame(stem)
        gc.session.check(question, f.depth, f.intrinsics, f.cam_to_base, f.hands, models=lambda: asked.append(1),
                         closed_within_deg=25.0 if closing else 0.0)
    assert not gc.session._registration and not gc.session._seat_refs and not asked


def test_a_part_lifted_but_not_open_answers_no_open_question():
    """From inspection pose 1, where the right hand hides the seat's front, a closed seat reads 15-20 deg and a closed lid
    10-15: a part in the lifted band never passes as open (lift_check counts lifted as open for the windows)."""
    gc = session(gap_decider="parts")
    a = gc.ask("lid_open", frame("20261031_112831_141_gap_check_lid_lifted_uncertain"))
    assert a.fixture_state == "lid_between" and vote("lid_open", a) == "none"
    b = gc.ask("seat_open", frame("20261030_153635_137_gap_check_seat_closed_uncertain"))
    assert vote("seat_open", b) in ("none", "no")


def test_the_closing_allowance_reaches_its_angle():
    """A closed lid of 28 Oct read exactly 25 deg: within the closing allowance (inclusive), in both readings."""
    stem = "20261028_145829_087_gap_check_lid_closed_uncertain"
    assert session(gap_decider="parts").ask("lid_closed", frame(stem), closing=True).fixture_state == "lid_closed"
    assert session(gap_decider="parts").ask("lid_closed", frame(stem)).fixture_state == "lid_lifted"   # the plain 10 deg rule


def test_the_windows_decide_by_default_with_the_parts_reading_beside():
    gc = session()
    a = gc.ask("lid_lifted", frame("20261031_112831_141_gap_check_lid_lifted_uncertain"))
    parts = a.record["parts"]
    assert not parts["decides"] and parts["answer"] == "lid_lifted" and parts["lid_deg"] == 35.0
    assert not a.message.startswith("parts")
