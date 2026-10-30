"""The closings' outcome checks count a part nearly closed as closed (the owner, 30 Oct: the camera may look while the
seat is still settling). On 30 Oct at 15:36 Close the seat ended with the seat down, but its check read it 20, 15, 10,
10, 10 deg from closed while the band reading said closed: five frames uncertain, the run stopped for a person; Close the
lid's checks of 29 Oct began the same way (15 and 10 deg) before they read closed. ALIX's shared gap check takes a closing
allowance (GapCheckSession.check ``closed_within_deg``, perception.yaml gap_closing_max_deg 25 deg) and Lite's runner
passes it for Close the seat's seat_closed and Close the lid's lid_closed only. A seat or lid really up stays NOT
closed: the seat standing open against the lid (30 Oct 15:37-15:38, three runs), the lid lifted 35 deg (30 Oct 11:44).
tests/data/gap_checks/closing holds those frames from the lab's records (depth in mm)."""
import json
from pathlib import Path

import numpy as np
import pytest

from alix_studio_lite.server import mechanism
from alix_studio_lite.server.gap_check import DepthFrame, GapCheck, Intrinsics, load_params

from test_mechanism import FakeExec

DATA = Path(__file__).parent / "data" / "gap_checks" / "closing"
SETTLING_SEAT = "20261030_153635_137_gap_check_seat_closed_uncertain"     # the seat 15 deg from closed, coming down
OPEN_SEAT = "20261030_153742_229_gap_check_seat_closed_seat_lifted"       # the seat standing open against the lid
SETTLING_LID = "20261029_191333_508_gap_check_lid_closed_uncertain"       # the lid 15 deg from closed, coming down
LIFTED_LID = "20261030_114406_282_gap_check_lid_lifted_lid_lifted"        # the lid lifted 35 deg by the left hand


def ask(stem: str, expect: str, closing: bool):
    rec = json.loads((DATA / f"{stem}.json").read_text())
    depth = np.load(DATA / f"{stem}.npz")["depth_mm"].astype(np.float32) / 1000.0
    k = rec["intrinsics"]
    frame = DepthFrame(depth=depth, intrinsics=Intrinsics(k["fx"], k["fy"], k["cx"], k["cy"], depth.shape[1], depth.shape[0]),
                       cam_to_base=np.asarray(rec["camera_to_world"], dtype=float), hands=rec.get("hands") or {},
                       source=rec["source"])
    gc = GapCheck(params=dict(load_params(), gap_cross_check=False), records=None)
    return rec, gc.ask(expect, frame, closing=closing)


@pytest.mark.parametrize("stem, expect, plain, allowed", [
    (SETTLING_SEAT, "seat_closed", "uncertain", "seat_closed"),
    (OPEN_SEAT, "seat_closed", "seat_lifted", "seat_lifted"),
    (SETTLING_LID, "lid_closed", "uncertain", "lid_closed"),
    (LIFTED_LID, "lid_closed", "lid_lifted", "lid_lifted"),
])
def test_a_part_nearly_closed_counts_as_closed_in_a_closings_outcome_check(stem, expect, plain, allowed):
    rec, a = ask(stem, expect, closing=False)
    assert a.success and a.fixture_state == plain == rec["class"], a.message        # the plain rule: as the lab read it
    rec, b = ask(stem, expect, closing=True)
    assert b.success and b.fixture_state == allowed, b.message
    assert b.record["closed_within_deg"] == load_params()["gap_closing_max_deg"] == 25.0
    if allowed != plain:
        assert "nearly closed" in b.message and "inside the closing allowance of 25 deg" in b.message
    assert b.record["passes_expectation"] == (allowed == expect)


def test_only_the_closings_outcome_checks_ask_with_the_allowance():
    """Close the seat's seat_closed and Close the lid's lid_closed: with it; Open the seat's own "is the seat down?" before
    it lifts, and every other check: without it (a seat 20 deg up is not closed before an opening)."""

    class Ex(FakeExec):
        def ask(self, action, expect, observe=False):
            self.calls.append(("closing", action.get("stage"), bool(action.get("closing_outcome"))))
            return super().ask(action, expect, observe)

    asked = {}
    for task, answers in (("seat_close", ["seat_lifted", "seat_closed"]), ("lid_close", ["lid_open", "lid_closed"]),
                          ("seat_open", ["lid_open", "seat_closed", "seat_lifted", "seat_open"])):
        ex = Ex(answers)
        mechanism.Runner(ex).run(task)
        asked[task] = [c[1:] for c in ex.calls if c[0] == "closing"]
    assert asked["seat_close"] == [("verify_seat_open", False), ("verify_seat_closed", True)]
    assert asked["lid_close"] == [("verify_lid_open", False), ("verify_lid_closed", True)]
    assert asked["seat_open"] == [("verify_lid_open", False), ("verify_seat_closed", False), ("verify_seat_lifted", False),
                                  ("verify_seat_open", False)]
