"""ALIX's gap check, ported without ROS (server/gap_check.py): the robot's own checks replayed through it.

tests/data/gap_checks holds checks the robot made with the wrist D435 on 10 and 12 October 2026 (ALIX's
Resource/captures): the depth each one used, where the camera was (``camera_to_world``) and what it
decided. Lite's port must decide the same from the same depth. The records carry no intrinsics, so the
D435's nominal ones are used, as ALIX's own replay tools do, and no hand positions, so nothing is masked:
heights within 3 mm, classes and verdicts the same.
"""
import json
from pathlib import Path

import numpy as np

from alix_studio_lite.server.gap_check import DepthFrame, GapCheck, Intrinsics, load_params

DATA = Path(__file__).parent / "data" / "gap_checks"

#: the robot's lid_open run of 12 October, in order: the closed lid (it learns the placement), the left arm's lift
#: missed three times (re-lifts 1 and 2 too), the third re-lift lifted it, the seat came up with it, the lid open,
#: then seat_close_right's question (the seat read closed: it skipped itself)
RUN_12_OCT = ["20261012_093917_987_gap_check_lid_closed_lid_closed", "20261012_093924_332_gap_check_lid_lifted_lid_closed",
              "20261012_093930_236_gap_check_lid_lifted_lid_closed", "20261012_093936_247_gap_check_lid_lifted_lid_closed",
              "20261012_093942_104_gap_check_lid_lifted_lid_lifted", "20261012_093942_368_gap_check_seat_lifted_seat_lifted",
              "20261012_094001_199_gap_check_lid_open_lid_open", "20261012_094001_358_gap_check_seat_open_seat_closed"]


def frame_of(stem: str):
    rec = json.loads((DATA / f"{stem}.json").read_text())
    mm = np.load(DATA / f"{stem}_depth.npz")["mm"]                # the camera's z16 millimetres, as the node got them
    return rec, DepthFrame(depth=mm, intrinsics=Intrinsics.from_fov(mm.shape[1], mm.shape[0]),
                           cam_to_base=np.array(rec["camera_to_world"], dtype=float))


def same_answer(a, rec, stem):
    assert a.success, (stem, a.message)
    assert a.fixture_state == rec["class"], (stem, a.message, rec["message"])
    assert abs(a.record["height_m"] - rec["height_m"]) < 0.003, (stem, a.record["height_m"], rec["height_m"])
    assert abs(a.record["closed_top_m"] - rec["closed_top_m"]) < 0.003, (stem, a.record["closed_top_m"], rec["closed_top_m"])
    assert a.record["passes_expectation"] == rec["passes_expectation"], stem


def test_the_robots_lid_open_run_of_12_october_replays_to_its_own_answers(tmp_path):
    said = []
    gc = GapCheck(records=tmp_path / "checks", log=said.append)
    for stem in RUN_12_OCT:
        rec, frame = frame_of(stem)
        same_answer(gc.ask(rec["expect"], frame), rec, stem)
        if stem == RUN_12_OCT[0]:
            # the closed lid teaches the placement: +12.2 mm on the robot
            assert abs(gc.registration_for("") * 1000 - rec["registration_m"] * 1000) < 2.0
            assert any("gap registration" in s for s in said)
    # the seat learned its own placement at the last question, as on the robot (+3.8 mm)
    assert abs(gc.registration_view()["seat_mm"] - 3.8) < 1.5


def test_the_robots_seat_checks_of_10_october_with_the_registration_they_had(tmp_path):
    gc = GapCheck(records=tmp_path / "checks")
    lid = gc.registration_key("")
    gc._registration[lid] = 0.0076146446132198675                 # the lid's, learned at 19:06:38 that day
    gc._registration[lid + ("seat",)] = -0.0041094321152266855    # the seat's, from 19:07:20
    for stem in ("20261010_190855_049_gap_check_seat_closed_seat_closed", "20261010_190903_863_gap_check_seat_lifted_seat_closed"):
        rec, frame = frame_of(stem)
        same_answer(gc.ask(rec["expect"], frame), rec, stem)
    # re-learned at 19:08:55 (the ring moved 7 mm): +3.14 on the robot, about 1 mm more here, from the nominal intrinsics
    assert abs(gc.registration_view()["seat_mm"] - 3.14) < 1.5


def test_a_seat_hidden_by_the_lid_is_refused_as_on_the_robot(tmp_path):
    rec, frame = frame_of("20261010_190702_419_gap_check_seat_lifted_refused")
    gc = GapCheck(records=tmp_path / "checks")
    gc._registration[gc.registration_key("")] = 0.0076146446132198675
    a = gc.ask(rec["expect"], frame)
    assert not a.success and a.fixture_state == "not_lifted"
    assert "seat band after masking the hands -> NOT lifted" in a.message


def test_every_doubt_is_not_lifted_and_every_check_is_recorded(tmp_path):
    rec, frame = frame_of(RUN_12_OCT[4])
    gc = GapCheck(records=tmp_path / "checks")
    assert "no depth frame" in gc.ask("lid_lifted", None).message
    assert "no TF camera_depth_optical_frame -> base_link" in gc.ask("lid_lifted", DepthFrame(frame.depth, frame.intrinsics, None)).message
    assert "no camera_info" in gc.ask("lid_lifted", DepthFrame(frame.depth, None, frame.cam_to_base)).message
    blank = DepthFrame(np.zeros_like(frame.depth), frame.intrinsics, frame.cam_to_base)
    a = gc.ask("lid_lifted", blank)
    assert not a.success and "only 0 % of the ROI carries depth" in a.message
    a = gc.ask("lid_lifted", frame)
    assert a.success and a.fixture_state == "lid_lifted" and a.record_path
    files = {p.name for p in (tmp_path / "checks").iterdir()}
    stem = Path(a.record_path).stem
    assert {f"{stem}.json", f"{stem}_depth.npy", f"{stem}_depth.png"} <= files
    saved = json.loads(Path(a.record_path).read_text())
    assert saved["class"] == "lid_lifted" and saved["camera_to_world"] and saved["params"]["gap_lift_m"] == 0.05
    assert np.load(tmp_path / "checks" / f"{stem}_depth.npy").shape == (480, 848)


def test_the_hands_are_masked_out_of_the_lids_cues(tmp_path):
    """The robot, 9 Oct 20:23: the left hand in the top rows was taken for a standing lid. A hand frame given
    where the depth sees something near the camera takes those pixels out of the cues."""
    rec, frame = frame_of(RUN_12_OCT[4])
    depth = frame.depth.astype(np.float64) / 1000.0
    h, w = depth.shape
    near = depth.copy()
    near[: int(0.3 * h), :] = 0.25                                  # something 25 cm from the lens, all over the top rows
    M = frame.cam_to_base
    i = DepthFrame(near, frame.intrinsics, M).intrinsics
    u, v, d = w / 2.0, 0.15 * h, 0.25
    centre = (M @ np.array([(u - i.cx) * d / i.fx, (v - i.cy) * d / i.fy, d, 1.0]))[:3]
    gc = GapCheck(params=dict(load_params(), gap_hand_mask_m=2.0))  # a mask that covers that whole area
    masked = gc.ask("lid_closed", DepthFrame(near, frame.intrinsics, M, hands={"ILH_plam": centre}))
    unmasked = GapCheck().ask("lid_closed", DepthFrame(near, frame.intrinsics, M))
    assert "standing-lid pixels 0 %" in masked.message and "standing-lid pixels 0 %" not in unmasked.message
