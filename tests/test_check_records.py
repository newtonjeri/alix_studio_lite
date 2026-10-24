"""Every camera check's record is a training row (the owner, 23 Oct: the models are to be revised, and "keep every
frame's record"): beside ALIX's own fields, the colour camera, the named pose and its registration, the fixture as
placed, the run's context (both arms, which check and frame, the camera), what made it, and the depth aligned to the
colour frame."""
import json
from pathlib import Path

import numpy as np

from alix_studio_lite.server.gap_check import DepthFrame, GapCheck, Intrinsics, write_check_record

DATA = Path(__file__).parent / "data" / "gap_checks"
STEM = "20261012_093917_987_gap_check_lid_closed_lid_closed"           # the robot's closed lid, 12 Oct


def test_a_checks_record_holds_its_training_row(tmp_path):
    rec = json.loads((DATA / f"{STEM}.json").read_text())
    mm = np.load(DATA / f"{STEM}_depth.npz")["mm"]
    aligned = np.full((720, 1280), 431, dtype=np.uint16)
    arms = {"left": {"q_deg": [0.0] * 7, "pole_mm": 150.0, "hand": [1000] * 6},
            "right": {"q_deg": [32.0, -96.0, 10.0, 112.0, -25.0, 60.0, -14.0], "pole_mm": 150.0, "hand": [546] * 6}}
    context = dict(arms=arms, check=dict(check_id="c1", stage="verify_lid_closed", expect="lid_closed", frame=2),
                   camera={"name": "Intel RealSense D435", "serial": "425122070189"})
    colour_k = Intrinsics.from_fov(1280, 720)
    frame = DepthFrame(depth=mm, intrinsics=Intrinsics.from_fov(mm.shape[1], mm.shape[0]),
                       cam_to_base=np.array(rec["camera_to_world"], dtype=float), aligned=aligned,
                       colour_intrinsics=colour_k, colour_to_base=np.eye(4), ref_to_base=np.eye(4), context=context)
    gc = GapCheck(records=tmp_path / "checks")
    a = gc.ask(rec["expect"], frame)
    assert a.success and a.fixture_state == rec["class"]
    assert gc.flush()                                    # the records are written by their own thread
    row = json.loads(Path(a.record_path).read_text())
    assert row["context"] == context and row["split"] == ""
    assert row["colour_intrinsics"]["width"] == 1280 and row["colour_to_world"] == np.eye(4).tolist()
    assert row["ref_to_world"] == np.eye(4).tolist() and "named_pose" in row
    assert set(row["registration"]) == {"lid_m", "seat_m"} and row["registration"]["lid_m"] != 0.0      # learnt here
    assert row["fixture"]["model"] == "commode_c" and row["fixture"]["scene_sha256"]
    prov = row["provenance"]
    assert prov["models"] and all(len(v) == 64 for v in prov["models"].values())
    assert prov["alix_vendored"] and prov["alix_vendored"]["commit"]
    npz = Path(a.record_path).with_name(row["files"]["aligned_depth_npz"])
    back = np.load(npz)["depth_mm"]
    assert back.dtype == np.uint16 and back.shape == (720, 1280) and int(back[0, 0]) == 431


def test_an_aligned_depth_in_metres_is_kept_in_millimetres(tmp_path):
    path = write_check_record(tmp_path, "gap_check", "seat_lifted", {"class": "refused"},
                              aligned=np.full((4, 6), 0.4325, dtype=np.float32))
    row = json.loads(path.read_text())
    back = np.load(tmp_path / row["files"]["aligned_depth_npz"])["depth_mm"]
    assert back.dtype == np.uint16 and int(back[0, 0]) == 432
