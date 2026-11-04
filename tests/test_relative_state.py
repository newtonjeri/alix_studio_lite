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
    assert p["gap_decider"] == "parts" and p["gap_parts_place_search"] == "uncertain" and p["gap_parts_colour"] is True
    assert p["gap_parts_place_reach_m"] == 0.06 and p["gap_parts_place_height_m"] == 0.015


@pytest.mark.parametrize("stem, asked, want", [(LID_LIFTED, "lid_lifted", "lid_lifted"),    # the lab: uncertain, stopped
                                               (LID_MISSED, "lid_lifted", "lid_closed"),
                                               (SEAT_LIFTED, "seat_lifted", "seat_lifted")])  # the lab: uncertain
def test_a_lift_is_read_from_the_parts_where_the_commode_stands(stem, asked, want):
    """As configured (5 Nov): a lift by its distance (no search); a missed lift, not lifted by distance, read by the
    parts at the plans' commode place. With the commode searched for every frame (4 Nov) the same answers."""
    pytest.importorskip("cv2")                                # the fit renders the commode with OpenCV
    _, f = _frame(stem)
    d = _check(_gc(), asked, f, models=_colour("closed"))     # a lift question never asks the colour model
    assert d.success and d.state == want, d.message
    assert "colour" not in d.record
    if want.endswith("lifted"):
        assert "by distance" in d.message and d.record["lift_reading"]["decides"]
    else:
        assert d.record["parts"]["decides"] and not d.record["parts"].get("first")      # definitive at the plans' place
    a = _check(_gc(gap_parts_place_search="always"), asked, f, models=_colour("closed"))
    assert a.success and a.state == want and "where this frame reads it" in a.message
    assert a.record["parts"]["place"] and a.record["parts"]["decides"]


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
    assert _check(_gc(), "lid_lifted", f).state == "lid_lifted"                        # by distance, as configured
    d0, d = (_check(_gc(gap_parts_place_search="always"), "lid_lifted", x) for x in (base, f))
    assert d.success and d.state == "lid_lifted", d.message
    p0, p = d0.record["parts"]["place"], d.record["parts"]["place"]
    assert abs((p["dx_m"] - p0["dx_m"]) - shift[0]) < 0.012 and abs((p["dy_m"] - p0["dy_m"]) - shift[1]) < 0.012   # found there


LAB_5NOV = LOGS / "2026-11-05"
LID_OPEN_5NOV = LAB_5NOV / "checks/20261105_005651_234_gap_check_lid_open_lid_open"        # Open the lid's end check, frame 2
SEAT_OPEN_LOOK = LAB_5NOV / "place/20261105_005735_109_place_look_seat_open_look"          # Open the seat's first look
SEAT_OPEN_F1 = LAB_5NOV / "checks/20261105_005754_226_gap_check_seat_open_seat_lifted"     # its open check: read open
SEAT_OPEN_F2 = LAB_5NOV / "checks/20261105_005755_931_gap_check_seat_open_uncertain"       # the lab: read closed, 16 mm low


def test_a_place_kept_against_another_fixture_pose_is_never_a_start():
    """The lab, 5 Nov 00:57: Open the seat's checks judged at its first look's place; its open check's second frame took
    the place Open the lid's check had kept for that camera pose (read at the scene place) and so started 42 mm off: the
    seat standing at 105 deg read flat, the commode 16 mm low, four frames, the run stopped. A kept place is a start only
    against the fixture pose it was read against, and only from a camera pose within 5 mm and 1 deg."""
    pytest.importorskip("cv2")
    from alix_studio_lite.server.gap_check import GapCheck as LiteGapCheck

    gc = LiteGapCheck(params={**load_params(), "gap_parts_place_search": "always"}, records=None)   # the search each frame
    gc.scene_pose()
    _, lid = _frame(LID_OPEN_5NOV)
    assert _check(gc, "lid_open", lid, models=_colour("lid_open")).state == "lid_open"   # kept: at the scene place
    _, look = _frame(SEAT_OPEN_LOOK)
    reading = gc.place(look)
    assert reading["ok"]
    gc.at_place(reading)                                      # the task's checks where its first look read the commode
    assert gc.session.parts_first_reach_m == pytest.approx(0.02)
    places = []
    for stem in (SEAT_OPEN_F1, SEAT_OPEN_F2):
        _, f = _frame(stem)
        d = _check(gc, "seat_open", f, models=_colour("lid_seat_open"))
        assert d.success and d.state == "seat_lifted" and "the seat open at" in d.message, d.message
        places.append(d.record["parts"]["place"])
    assert abs(places[1]["dz_m"] - places[0]["dz_m"]) < 0.005    # the second frame started where the first found it
    gc.new_places()                                           # a new run: nothing kept
    assert gc.session._parts_places == []


def test_the_render_is_the_same_image_as_before_5_nov():
    """region3d.render_hypothesis (5 Nov: one mask, no copy, 1.7x faster) gives the very image the arithmetic before it
    gave, the raised parts' mask too, cells behind the camera dropped."""
    pytest.importorskip("cv2")
    import cv2

    from alix_perception_core import region3d

    def before(points_body, body_from_cam, fx, fy, cx, cy, width, height, stride, raised=None):
        cfb = np.linalg.inv(body_from_cam)
        cam = points_body @ cfb[:3, :3].T + cfb[:3, 3]
        front = cam[:, 2] > 0.05
        cam = cam[front]
        flags = raised[front] if raised is not None else None
        u = ((fx * cam[:, 0] / cam[:, 2] + cx) / stride).astype(int)
        v = ((fy * cam[:, 1] / cam[:, 2] + cy) / stride).astype(int)
        w, h = width // stride, height // stride
        ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        u, v, z = u[ok], v[ok], cam[ok, 2].astype(np.float32)
        img = np.full(h * w, region3d._EMPTY, np.float32)
        pix = v * w + u
        np.minimum.at(img, pix, z)
        img = cv2.erode(img.reshape(h, w), np.ones((5, 5), np.uint8))
        img[img >= region3d._EMPTY * 0.5] = 0.0
        if flags is None:
            return img
        rz = np.full(h * w, region3d._EMPTY, np.float32)
        fr = flags[ok]
        np.minimum.at(rz, pix[fr], z[fr])
        rz = cv2.erode(rz.reshape(h, w), np.ones((5, 5), np.uint8))
        return img, (img > 0) & (rz < region3d._EMPTY * 0.5) & (rz <= img + 0.002)

    rng = np.random.default_rng(5)
    pts = rng.uniform([-0.3, -0.3, -0.4], [0.3, 0.3, 0.9], size=(20000, 3))       # some behind the camera
    raised = rng.random(len(pts)) < 0.3
    for k in range(6):
        M = np.eye(4)
        M[:3, 3] = rng.uniform(-0.05, 0.05, 3)
        for stride in (2, 3, 6):
            a = before(pts, M, 600.0, 600.0, 424.0, 240.0, 848, 480, stride, raised)
            b = region3d.render_hypothesis(pts, M, 600.0, 600.0, 424.0, 240.0, 848, 480, stride, raised)
            assert np.array_equal(a[0], b[0]) and np.array_equal(a[1], b[1])
            assert np.array_equal(before(pts, M, 600.0, 600.0, 424.0, 240.0, 848, 480, stride),
                                  region3d.render_hypothesis(pts, M, 600.0, 600.0, 424.0, 240.0, 848, 480, stride))


def test_the_commode_is_searched_for_only_when_the_answer_is_not_definitive():
    """The user, 5 Nov: the search "in an instance we cannot be able to judge or where the check is not able to give us
    a definitive state". Open the seat's lifted check of 5 Nov 00:57 (the seat held at 30 deg): not lifted by distance,
    not definitive at the plans' place, lifted once the commode is searched for. A standing seat (24 Oct) read where the
    plans put the commode: definitive at once; the commode 42 mm off: searched for, the seat open, the colour agreeing."""
    pytest.importorskip("cv2")
    _, held = _frame(LAB_5NOV / "checks/20261105_005742_392_gap_check_seat_lifted_seat_lifted")
    d = _check(_gc(), "seat_lifted", held)
    assert d.success and d.state == "seat_lifted" and d.record["parts"]["searched"], d.message
    assert d.record["parts"]["first"]["place"] == "the plans'" and "the commode searched for" in d.message
    stem = LOGS / "2026-10-24/checks/20261024_224223_714_gap_check_seat_open_seat_lifted"
    _, here = _frame(stem)
    d = _check(_gc(), "seat_open", here, models=_colour("lid_seat_open"))
    assert d.state == "seat_lifted" and not d.record["parts"]["searched"], d.message
    _, far = _frame(stem, (0.03, -0.03, 0.0))
    d = _check(_gc(), "seat_open", far, models=_colour("lid_seat_open"))
    assert d.state == "seat_lifted" and d.record["parts"]["searched"] and "the colour agrees" in d.message, d.message
    never = _check(_gc(gap_parts_place_search="never"), "seat_open", far, models=_colour("lid_seat_open"))
    assert not never.record["parts"]["searched"] and never.state != "seat_lifted"
