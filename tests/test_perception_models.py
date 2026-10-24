"""The wrist camera's colour models (the owner, 18 Oct: "the perception visual models"), as ALIX's perception node
asks them: its core vendored (alix_perception_core, the robot bringup's models_synthetic_real_v10.yaml), its
status_check and the gap check's cross-check ported (server/gap_check.py).

Where ultralytics and ALIX's recorded frames are on the computer, Lite's status_check must answer as ALIX's own core
answered on the same frames (tests/data/perception/expected_status.json, made by running ALIX's core in place)."""
import importlib.util
import json
import os
from pathlib import Path

import numpy as np
import pytest

from alix_studio_lite.server.gap_check import DepthFrame, GapCheck, Intrinsics

DATA = Path(__file__).parent / "data"
FRAMES = Path.home() / "alix_ws/src/alix/Resource/Data/d435_frames_2026-10-08"
MODELS = importlib.util.find_spec("ultralytics") is not None and importlib.util.find_spec("cv2") is not None
needs_models = pytest.mark.skipif(not (MODELS and FRAMES.is_dir()), reason="ultralytics, OpenCV or ALIX's recorded frames not here")


def recorded(name):
    import cv2
    import yaml

    f = next(x for x in yaml.safe_load((FRAMES / "labels.yaml").read_text())["frames"] if x["file"] == name)
    img = cv2.imread(str(FRAMES / f["path"]), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(FRAMES / "depth" / (os.path.splitext(name)[0] + ".png")), cv2.IMREAD_UNCHANGED)
    i = f["intrinsics"]
    base_cam, ref_cam = np.array(f["camera_to_base_link"]), np.array(f["camera_to_alix_ref_frame"])
    return dict(colour=img, aligned=depth, colour_intrinsics=Intrinsics(i["fx"], i["fy"], i["cx"], i["cy"], i["width"], i["height"]),
                colour_to_base=base_cam, ref_to_base=base_cam @ np.linalg.inv(ref_cam))


def test_the_core_is_alix_s_own_built_from_lite_s_files():
    said = []
    gc = GapCheck(log=said.append)
    core = gc.core()
    assert core is not None and core.backend.name == "local"
    import alix_perception_core

    assert "vendor/alix/core/python/alix_perception_core" in alix_perception_core.__file__
    assert gc._fixture_in_world[:3, 3].round(3).tolist() == [1.005, 0.0, 0.42]
    assert gc._fixture_in_world[0, 0] == pytest.approx(-1.0)      # the venue's 180 deg yaw: the commode faces the robot (24 Oct)
    assert any("models_synthetic_real_v10.yaml" in s and "21 regions" in s for s in said)


@pytest.mark.skipif(MODELS, reason="the models are installed here")
def test_without_the_models_runtime_the_depth_answers_alone():
    """No ultralytics: the colour model cannot be asked (said), and the gap check answers from the depth, as the node."""
    rec = json.loads((DATA / "gap_checks/20261012_093917_987_gap_check_lid_closed_lid_closed.json").read_text())
    mm = np.load(DATA / "gap_checks/20261012_093917_987_gap_check_lid_closed_lid_closed_depth.npz")["mm"]
    colour = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame = DepthFrame(depth=mm, intrinsics=Intrinsics.from_fov(848, 480), cam_to_base=np.array(rec["camera_to_world"]),
                       colour=colour, aligned=np.zeros((720, 1280), dtype=np.uint16), colour_intrinsics=Intrinsics.from_fov(1280, 720, 69.4, 42.5),
                       colour_to_base=np.array(rec["camera_to_world"]), ref_to_base=np.eye(4))
    said = []
    gc = GapCheck(log=said.append)
    a = gc.ask("lid_closed", frame)
    assert a.success and a.fixture_state == "lid_closed" and a.record["models"] == {}
    assert any("the models could not be asked" in s for s in said)
    s = gc.status(frame)                                          # said, not raised
    assert not s.success and ("ultralytics" in s.message or "cv2" in s.message)


@needs_models
def test_status_check_answers_as_alix_s_core_on_the_robot_s_frames():
    """The state and what decided it are ALIX's own on every frame. The confidence is the model's own number: ALIX's
    answers were recorded on a CPU, and the lab computer runs the model on its GPU, whose arithmetic differs in the
    third decimal (19 Oct, pose 4 lid and seat open: 0.870 against 0.871), so it is matched within 0.01."""
    expected = json.loads((DATA / "perception/expected_status.json").read_text())["frames"]
    # one checker for all 15, in order, as one running node (each answer uses the registration learned so far); ALIX's
    # answers made by Resource/reports/2026-10-24_fixture_state/expected_status_from_core.py with the node's placement
    gc = GapCheck()
    for row in expected:
        a = gc.status(DepthFrame(depth=None, intrinsics=None, cam_to_base=None, **recorded(row["file"])))
        assert (a.fixture_state, a.record["state_source"]) == (row["state"], row["source"]), row["file"]
        assert abs(a.state_confidence - row["conf"]) <= 0.01, (row["file"], a.state_confidence, row["conf"])
        assert a.fixture_state == row["declared"]                    # and as the operator declared it


@needs_models
def test_a_closed_lid_the_colour_model_sees_open_is_uncertain():
    """The gap check's cross-check (perception_node.py 1126-1135): the geometry reads the lid closed, the colour model
    confidently reads it open: "uncertain", which proves nothing."""
    rec = json.loads((DATA / "gap_checks/20261012_093917_987_gap_check_lid_closed_lid_closed.json").read_text())
    mm = np.load(DATA / "gap_checks/20261012_093917_987_gap_check_lid_closed_lid_closed_depth.npz")["mm"]
    seen_open = recorded("inspect_inspection_pose_4_full_length_lid_open_20261008_174502_00.jpg")
    frame = DepthFrame(depth=mm, intrinsics=Intrinsics.from_fov(848, 480), cam_to_base=np.array(rec["camera_to_world"]), **seen_open)
    a = GapCheck().ask("lid_closed", frame)
    assert a.success and a.fixture_state == "uncertain" and a.record["models"]["contradiction"]
    assert a.record["models"]["colour_state"] == "lid_open" and "CONTRADICTS the geometry" in a.message
