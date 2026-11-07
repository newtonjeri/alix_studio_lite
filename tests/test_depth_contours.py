"""The lid read from the depth's iso-distance contours (the user, 6 Nov: "contours should be applied in the depth image
... lines joining points with the same distances ... a 1 centimeter tolerance"): ALIX's alix_perception_core.depth_contours,
vendored, decides every lid question in Lite's GapCheck before the fixture fit. The lab's recorded frames of every lid
state (vendor/alix/core/python/alix_perception_core/test/data/contours), asked the question their check asked, vote as
their photos show (lift_check.frame_vote); the new robot's frame of 6 Nov, which the fit read lifted 55 deg with the
lid closed (its camera pose 20 mm and 3.4 deg off), reads closed."""
import json

import numpy as np
import pytest

pytest.importorskip("cv2")                                  # the contours and the fit both read the frame with OpenCV

from alix_studio_lite import vendor_paths
from alix_studio_lite.server import mechanism
from alix_studio_lite.server.gap_check import DepthFrame, GapCheck, Intrinsics, load_params

DATA = vendor_paths.ALIX / "core" / "python" / "alix_perception_core" / "test" / "data" / "contours"

#: (frame, the question its check asked, a closing's end check, the photo's vote)
CASES = [
    ("20261106_215826_599_gap_check_lid_closed_uncertain", "lid_closed", False, "yes"),   # the new robot, 6 Nov 21:58
    ("20261105_230601_739_gap_check_lid_closed_lid_closed", "lid_closed", False, "yes"),
    ("20261105_230104_379_gap_check_lid_closed_lid_closed", "lid_closed", True, "yes"),   # resting ~10 deg on the seat
    ("20261105_230718_285_gap_check_lid_closed_lid_open", "lid_closed", False, "no"),     # INSPECT found the lid open
    ("20261105_034936_683_gap_check_lid_open_lid_open", "lid_open", False, "yes"),
    ("20261105_033823_273_gap_check_lid_lifted_lid_lifted", "lid_lifted", False, "yes"),  # held by the hand
    ("20261105_033823_273_gap_check_lid_lifted_lid_lifted", "lid_open", False, "none"),   # lifted, not open: no vote
    ("20261105_005754_226_gap_check_seat_open_seat_lifted", "lid_closed", False, "no"),   # the seat standing against the open lid
]


def frame(stem: str) -> tuple:
    rec = json.loads((DATA / f"{stem}.json").read_text(encoding="utf-8"))
    depth = np.load(DATA / f"{stem}_depth.npz")["depth_mm"].astype(np.float32) / 1000.0
    k = rec["intrinsics"]
    return rec, DepthFrame(depth=depth, intrinsics=Intrinsics(k["fx"], k["fy"], k["cx"], k["cy"], depth.shape[1], depth.shape[0]),
                           cam_to_base=np.asarray(rec["camera_to_world"], dtype=float), hands=rec.get("hands") or {},
                           source="native depth (test)")


def vote(asked: str, answer) -> str:
    return mechanism.lift_check().frame_vote({"check": "fixture_gap", "expect": asked}, answer) or "none"


@pytest.fixture(scope="module")
def gc(tmp_path_factory):
    return GapCheck(params={**load_params(), "gap_cross_check": False}, records=tmp_path_factory.mktemp("checks"))


@pytest.mark.parametrize("stem,asked,closing,want", CASES)
def test_each_lab_frame_votes_as_its_photo_shows(gc, stem, asked, closing, want):
    rec, f = frame(stem)
    a = gc.ask(asked, f, closing=closing)
    assert a.success, a.message
    assert a.message.startswith("contours (depth): "), a.message
    assert vote(asked, a) == want, (stem, a.fixture_state, a.message)
    parts = a.record["parts"]
    assert parts["reading"] == "contours" and parts["decides"] and not parts["searched"]


@pytest.mark.parametrize("stem,asked,want", [
    ("isaac_closed_pose4", "lid_closed", "yes"),
    ("isaac_open_pose4", "lid_closed", "no"),
    ("isaac_lifted_18deg", "lid_lifted", "yes"),
    ("isaac_lifted_34deg", "lid_closed", "no"),
])
def test_isaac_sims_renders_vote_as_rendered(gc, stem, asked, want):
    """Isaac Sim's camera (1920 x 1080, ideal depth, a synthetic washroom, 6 Nov): the contours decide the same way."""
    rec, f = frame(stem)
    a = gc.ask(asked, f)
    assert a.success and a.record["parts"]["reading"] == "contours", a.message
    assert vote(asked, a) == want, (stem, a.fixture_state, a.message)


def test_the_new_robots_frame_reads_closed_without_the_fit_or_the_colour_model(gc):
    """6 Nov 21:58: the fit read lifted 55 deg (the commode 20 mm below its model) and the colour model closed, so the
    check was uncertain five times and INSPECT stopped. The contours read the lid 2.3 deg to the floor, no bowl: closed."""
    rec, f = frame("20261106_215826_599_gap_check_lid_closed_uncertain")
    a = gc.ask("lid_closed", f)
    assert a.fixture_state == "lid_closed" and a.state_confidence >= 0.9
    parts = a.record["parts"]
    assert parts["lid_floor_deg"] < 3.0 and parts["cavity_m"] < 0.2
    assert "lid_deg" in parts and parts["lid_deg"] == parts["lid_floor_deg"]
    assert a.record["colour"]["used"] is False                     # no colour frame given: the depth alone


def test_a_frame_without_the_commode_falls_back_to_the_fit_which_refuses(gc):
    """Floor and a wall only (6 Nov 17:55, pole 10): the contours cannot judge, the fit searches and refuses, as at the lab."""
    rec, f = frame("20261106_175505_250_gap_check_lid_closed_refused")
    a = gc.ask("lid_closed", f)
    assert not a.success
    assert a.message.startswith("cannot judge: contours (depth): the lid cannot be judged (") and "the fit:" in a.message
    assert a.record["parts"]["contours"]["reading"] == "contours"


def test_the_seat_is_still_read_by_the_fit(gc):
    rec, f = frame("20261105_005754_226_gap_check_seat_open_seat_lifted")
    a = gc.ask("seat_open", f)
    assert a.success and a.message.startswith("parts (depth): "), a.message
    assert a.record["parts"].get("reading") != "contours"
    assert vote("seat_open", a) == "yes"


def test_the_contours_can_be_turned_off(tmp_path):
    off = GapCheck(params={**load_params(), "gap_cross_check": False, "gap_lid_contours": False}, records=tmp_path)
    rec, f = frame("20261105_230601_739_gap_check_lid_closed_lid_closed")
    a = off.ask("lid_closed", f)
    assert a.success and a.message.startswith("parts (depth): ")
