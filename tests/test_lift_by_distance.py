"""The lift questions decided by distance (the user, 4 Nov: "for the lid and seat lifted state, this is very obvious. It's
distance measurement ... the returned distance expressed in the alix frame"): ALIX's shared check
(alix_perception_core.gap_check, gap_lift_by_distance), lid_lifted and seat_lifted only, on the lab's own frames. The
part's height in base_link against its closed top in the scene, the gloves left out; no fixture fit, nothing learned.
The distance decides "lifted" only; anything else, the part's own rules answer as before (closed, open: "already
working", and they tell a seat standing open from a closed one)."""
import json
from pathlib import Path

import numpy as np
import pytest

from alix_studio_lite.server.gap_check import DepthFrame, GapCheck, Intrinsics, load_params

pytestmark = pytest.mark.usefixtures("windows_rules")   # the rules before 4 Nov: gap_decider "windows"

CHECKS = Path(__file__).resolve().parents[1] / "Resources" / "Logs"
LID_LIFTED = CHECKS / "2026-11-04/checks/20261104_202716_965_gap_check_lid_lifted_uncertain"   # held high; the run stopped
LID_MISSED = CHECKS / "2026-10-21/checks/20261021_214655_523_gap_check_lid_lifted_lid_closed"  # a missed lift: lid flat
SEAT_LIFTED = CHECKS / "2026-11-04/checks/20261104_133317_866_gap_check_seat_lifted_uncertain"  # held up by the glove
SEAT_UNDER_LID = CHECKS / "2026-10-24/checks/20261024_155335_115_gap_check_seat_lifted_uncertain"  # asked during the lid's lift
ON_THE_FLOOR = CHECKS / "2026-10-27/checks/20261027_125702_232_gap_check_lid_lifted_refused"   # the camera saw the floor


def _ask(stem: Path, asked: str, **params):
    if not Path(str(stem) + ".json").exists():
        pytest.skip(f"the lab's record {stem.name}")
    rec = json.loads(Path(str(stem) + ".json").read_text())
    depth = np.load(str(stem) + "_depth.npy")
    k = rec["intrinsics"]
    gc = GapCheck(params={**load_params(), "gap_cross_check": False, "gap_parts_shadow": False, **params}, records=None)
    if rec.get("registration_m"):
        gc.session._registration[gc.session._registration_key(rec.get("registration_pose") or "")] = float(rec["registration_m"])
    frame = DepthFrame(depth=depth, intrinsics=Intrinsics(k["fx"], k["fy"], k["cx"], k["cy"], depth.shape[1], depth.shape[0]),
                       cam_to_base=np.asarray(rec["camera_to_world"], float), hands=rec.get("hands") or {}, source="native depth")
    q = (((rec.get("context") or {}).get("arms") or {}).get("right") or {}).get("q_deg")
    return gc.ask(asked, frame, q_deg=q), rec


def test_the_parameters_are_alix_s():
    p = load_params()
    assert p["gap_lift_by_distance"] is True and p["gap_lift_m"] == 0.05
    assert p["gap_lift_roi"] == [0.30, 0.70, 0.30, 0.70] and p["gap_seat_lift_roi"] == [0.40, 1.00, 0.05, 0.95]
    assert p["gap_lift_hand_mask_m"] == 0.10 and p["gap_fit_height_search_m"] == 0.010   # the fit's search: not used here


def test_a_lifted_lid_reads_lifted_by_its_distance_without_the_fit():
    a, rec = _ask(LID_LIFTED, "lid_lifted")
    assert rec["class"] == "uncertain"                                   # the lab: the fit called it closed, the run stopped
    assert a.success and a.fixture_state == "lid_lifted", a.message
    r = a.record
    assert "by distance" in a.message and "lid_fit" not in r and r["registration_m"] == 0.0
    assert r["lift_reading"]["risen_frac"] > 0.9 and 0.09 < r["rise_m"] < 0.2 and r["closed_top_m"] == 0.45
    off, _ = _ask(LID_LIFTED, "lid_lifted", gap_lift_by_distance=False)
    pytest.importorskip("cv2")                                           # the rules before ask the fit, which needs OpenCV
    assert off.fixture_state == "uncertain"


def test_a_missed_lift_is_left_to_the_lids_own_rules_which_read_it_closed():
    a, rec = _ask(LID_MISSED, "lid_lifted")
    assert rec["class"] == "lid_closed" and a.fixture_state == "lid_closed", a.message
    assert "by distance" not in a.message                                # the closed lid: the rules that worked
    assert a.record["lift_reading"]["risen_frac"] < 0.1 and a.record["lift_reading"]["decides"] is False


def test_a_seat_held_up_by_the_glove_reads_lifted():
    a, rec = _ask(SEAT_LIFTED, "seat_lifted")
    assert rec["class"] == "uncertain"                                   # the lab could not judge it (the band saw the rim)
    assert a.success and a.fixture_state == "seat_lifted", a.message
    lr = a.record["lift_reading"]
    assert lr["risen_frac"] > 0.5 and lr["bowl_frac"] >= 0.78 and lr["closed_top_m"] == 0.44


def test_a_seat_under_the_lid_is_not_read_lifted_from_the_lid():
    a, _ = _ask(SEAT_UNDER_LID, "seat_lifted")                           # the window sees the lifted lid, not the seat
    assert a.record["lift_reading"]["decides"] is False and a.record["lift_reading"]["bowl_frac"] < 0.78
    assert "by distance" not in a.message and a.fixture_state != "seat_lifted", a.message   # the seat's own rules


def test_nothing_in_view_is_never_lifted():
    a, _ = _ask(ON_THE_FLOOR, "lid_lifted")                               # the camera on the floor: the lid's rules refuse
    assert not a.success and a.record["lift_reading"]["decides"] is False, a.message


def test_the_other_questions_keep_their_rules():
    a, _ = _ask(LID_MISSED, "lid_closed")                                 # not a lift question: the windows and the fit
    assert "by distance" not in a.message and "lift_reading" not in a.record
