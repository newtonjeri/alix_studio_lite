"""The lid's and the seat's state from the parts against the commode found in each frame, the colour image beside it for
open and closed (the user, 4 Nov: "regardless of the viewpoint, and the distance ... the distance between the lid and the
seat, and the lid and the commode, and the seat and the commode"; "for seat and lid opened and closed we can supplement
them with the coloured image ... a place holder for now"). ALIX's shared check as configured (gap_decider "parts",
gap_parts_find_place, gap_parts_colour), on the lab's own frames."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from alix_studio_lite.server.gap_check import DepthFrame, GapCheck, Intrinsics, load_params

LOGS = Path(__file__).resolve().parents[1] / "Resources" / "Logs"
LID_LIFTED = LOGS / "2026-11-04/checks/20261104_202716_965_gap_check_lid_lifted_uncertain"      # held high; the run stopped
LID_MISSED = LOGS / "2026-10-21/checks/20261021_214655_523_gap_check_lid_lifted_lid_closed"     # a missed lift: lid flat
SEAT_LIFTED = LOGS / "2026-11-04/checks/20261104_133317_866_gap_check_seat_lifted_uncertain"     # held up by the glove
SEAT_UNDER_LID = LOGS / "2026-10-24/checks/20261024_155335_115_gap_check_seat_lifted_uncertain"  # asked during the lid's lift
LID_OPEN = LOGS / "2026-11-04/checks/20261104_225638_274_gap_check_lid_open_lid_open"            # the lid standing
SEAT_UP = LOGS / "2026-11-04/checks/20261104_161418_473_gap_check_seat_closed_seat_lifted"        # Close the seat: still up


def _frame(stem: Path, shift=None):
    if not Path(str(stem) + ".json").exists():
        pytest.skip(f"the lab's record {stem.name}")
    rec = json.loads(Path(str(stem) + ".json").read_text())
    depth = np.load(str(stem) + "_depth.npy")
    k = rec["intrinsics"]
    cam, hands = np.asarray(rec["camera_to_world"], float), dict(rec.get("hands") or {})
    if shift is not None:                    # the camera's pose moved = the commode moved the other way
        M = np.eye(4)
        M[:3, 3] = shift
        cam = M @ cam
        hands = {n: (np.asarray(v, float)[:3] + np.asarray(shift)).tolist() for n, v in hands.items()}
    return rec, DepthFrame(depth=depth, intrinsics=Intrinsics(k["fx"], k["fy"], k["cx"], k["cy"], depth.shape[1], depth.shape[0]),
                           cam_to_base=cam, hands=hands, source="native depth")


def _gc(**params):
    gc = GapCheck(params={**load_params(), **params}, records=None)
    gc.scene_pose()
    return gc


def _colour(state: str, conf: float = 0.9):
    return lambda: SimpleNamespace(fixture_state=state, state_source="colour", state_confidence=conf,
                                   diagnostics={"colour_state": state, "colour_confidence": conf})


def _check(gc, asked, frame, models=None, closing_deg=0.0):
    d = np.asarray(frame.depth)
    d = d.astype(np.float64) * (0.001 if d.dtype == np.uint16 else 1.0)
    return gc.session.check(asked, d, frame.intrinsics, frame.cam_to_base, frame.hands, models=models,
                            closed_within_deg=closing_deg)


def test_the_parts_decide_with_the_commode_found_and_the_colour_for_open_and_closed():
    p = load_params()
    assert p["gap_decider"] == "parts" and p["gap_parts_find_place"] is True and p["gap_parts_colour"] is True
    assert p["gap_parts_place_reach_m"] == 0.06 and p["gap_parts_place_height_m"] == 0.015


@pytest.mark.parametrize("stem, asked, want", [(LID_LIFTED, "lid_lifted", "lid_lifted"),    # the lab: uncertain, stopped
                                               (LID_MISSED, "lid_lifted", "lid_closed"),
                                               (SEAT_LIFTED, "seat_lifted", "seat_lifted")])  # the lab: uncertain
def test_a_lift_is_read_from_the_parts_where_the_commode_stands(stem, asked, want):
    pytest.importorskip("cv2")                                # the fit renders the commode with OpenCV
    _, f = _frame(stem)
    d = _check(_gc(), asked, f, models=_colour("closed"))     # a lift question never asks the colour model
    assert d.success and d.state == want, d.message
    assert "where this frame reads it" in d.message and "colour" not in d.record
    assert d.record["parts"]["place"] and d.record["parts"]["decides"]


def test_a_seat_under_the_lifted_lid_is_not_judged():
    pytest.importorskip("cv2")
    _, f = _frame(SEAT_UNDER_LID)
    d = _check(_gc(), "seat_lifted", f, models=_colour("lid_seat_open"))
    assert not d.success and "under the lifted lid" in d.message, d.message


def test_the_colour_agrees_decides_or_makes_it_uncertain():
    pytest.importorskip("cv2")
    _, f = _frame(LID_OPEN)
    agree = _check(_gc(), "lid_open", f, models=_colour("lid_open"))
    assert agree.state == "lid_open" and "the colour agrees" in agree.message, agree.message
    disagree = _check(_gc(), "lid_open", f, models=_colour("closed"))
    assert disagree.state == "uncertain" and "they disagree" in disagree.message, disagree.message
    weak = _check(_gc(), "lid_open", f, models=_colour("closed", 0.4))           # under confidence_threshold
    assert weak.state == "lid_open" and "the depth alone" in weak.message
    blind = DepthFrame(np.zeros_like(np.asarray(f.depth)), f.intrinsics, f.cam_to_base, hands=f.hands)
    decides = _check(_gc(), "lid_open", blind, models=_colour("lid_open"))      # the depth cannot judge: the colour
    assert decides.success and decides.state == "lid_open" and "the colour decides" in decides.message, decides.message
    nothing = _check(_gc(), "lid_open", blind)                                   # neither: cannot judge, never a guess
    assert not nothing.success and nothing.message.startswith("cannot judge")


def test_a_seat_left_standing_by_a_closing_reads_not_closed():
    pytest.importorskip("cv2")
    _, f = _frame(SEAT_UP)
    d = _check(_gc(), "seat_closed", f, models=_colour("lid_seat_open"), closing_deg=25.0)
    assert d.success and d.state == "seat_lifted" and "the colour agrees" in d.message, d.message


@pytest.mark.parametrize("shift", [(0.03, -0.03, 0.0), (-0.03, 0.03, 0.015), (0.04, 0.0, -0.015)])
def test_the_lifted_lid_reads_lifted_with_the_commode_moved(shift):
    pytest.importorskip("cv2")
    _, base = _frame(LID_LIFTED)
    _, f = _frame(LID_LIFTED, shift)
    d0, d = _check(_gc(), "lid_lifted", base), _check(_gc(), "lid_lifted", f)
    assert d.success and d.state == "lid_lifted", d.message
    p0, p = d0.record["parts"]["place"], d.record["parts"]["place"]
    assert abs((p["dx_m"] - p0["dx_m"]) - shift[0]) < 0.012 and abs((p["dy_m"] - p0["dy_m"]) - shift[1]) < 0.012   # found there
