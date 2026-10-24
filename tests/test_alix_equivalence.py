"""ALIX's gap check and Lite's give the same answer for the same frame (the owner, 24 Oct: "what ALIX ... does should be
the same as what happens in Lite ... the same result for the same input in both").

Both run ALIX's alix_perception_core/gap_check.py (vendored). ALIX's perception node, the ROS service's own code, answered
the recorded test frames (vendor/alix/core/python/alix_perception_core/test/data/seat_rise), each day one session in stamp
order, into vendor/alix/Resource/reports/2026-10-24_fixture_state/golden_node_frames.json; Lite's GapCheck, given the
same frames as its D435 and FK would give them, must answer every one the same: class, confidence, message, heights,
registration, and each reading's own answer (the fixture fit, the seat's comparison with its closed frame)."""
import calendar
import json
import time

import numpy as np
import pytest

from alix_studio_lite import vendor_paths
from alix_studio_lite.server.gap_check import DepthFrame, GapCheck, Intrinsics

GOLDEN = vendor_paths.ALIX / "Resource" / "reports" / "2026-10-24_fixture_state" / "golden_node_frames.json"
DATA = vendor_paths.ALIX / "core" / "python" / "alix_perception_core" / "test" / "data" / "seat_rise"


def decision_time(rec, stamp: str) -> float:
    """golden_node_frames.py's rule: the record's decision time, else its stamp read as UTC."""
    if rec.get("decision_wall_s") is not None:
        return float(rec["decision_wall_s"])
    return calendar.timegm(time.strptime(stamp[:15], "%Y%m%d_%H%M%S")) + int(stamp[16:19]) / 1000.0


def without_colour_note(message: str) -> str:
    """The message without its last clause "; colour: ...": each side's note on ITS colour stream when there is no colour
    frame (the node names its ROS topic, Lite its wrist camera; the golden run asked the node with no note). The
    decision, every measurement and every reading's text before it must be identical."""
    return message.split("; colour: ")[0]


def test_lite_answers_every_recorded_frame_as_alix_s_node_does(tmp_path):
    golden = json.loads(GOLDEN.read_text(encoding="utf-8"))
    gc = GapCheck(params=dict(__import__("alix_studio_lite.server.gap_check", fromlist=["load_params"]).load_params(),
                              gap_cross_check=False), records=tmp_path / "checks")
    day = None
    for row in golden["rows"]:
        stamp = row["stamp"]
        if stamp[:8] != day:                                     # a new day: a new session, as the node's
            day = stamp[:8]
            gc.forget()
        rec = json.loads((DATA / f"{stamp}.json").read_text(encoding="utf-8"))
        depth = np.load(DATA / f"{stamp}.npz")["depth_mm"].astype(np.float32) / 1000.0
        k = rec["intrinsics"]
        frame = DepthFrame(depth=depth, intrinsics=Intrinsics(k["fx"], k["fy"], k["cx"], k["cy"], depth.shape[1], depth.shape[0]),
                           cam_to_base=np.asarray(rec["camera_to_world"], dtype=float), hands=rec.get("hands") or {},
                           source=golden["source"])
        gc.session.now = (lambda t: (lambda: t))(decision_time(rec, stamp))
        a = gc.ask(row["expect"], frame)
        r = a.record
        got = dict(success=a.success, state=a.fixture_state, confidence=float(a.state_confidence), recommendation=a.recommendation,
                   message=a.message, record_class=r.get("class"), height_m=r.get("height_m"), closed_top_m=r.get("closed_top_m"),
                   registration_m=r.get("registration_m"),
                   lid_fit=(r.get("lid_fit") or {}).get("lid", {}).get("state") if r.get("lid_fit") else None,
                   seat_fit=(r.get("seat_fit") or {}).get("seat", {}).get("state") if r.get("seat_fit") else None,
                   seat_rise=(r.get("seat_rise") or {}).get("state") if r.get("seat_rise") else None)
        for key, want in row.items():
            if key in ("stamp", "expect"):
                continue
            if key == "message":
                assert without_colour_note(got[key]) == want, (stamp, got[key], want)
            elif isinstance(want, float):
                assert got[key] == pytest.approx(want, abs=1e-9), (stamp, key, got[key], want)
            else:
                assert got[key] == want, (stamp, key, got[key], want)
    assert len(golden["rows"]) >= 26
