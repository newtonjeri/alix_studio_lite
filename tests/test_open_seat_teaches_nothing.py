"""Only a question that expects the seat closed teaches the gap check the seat's height (the lab, 3 Nov). With the seat
open or lifted, the band at the closed ring's height sees the bowl's rim, 12 mm below the ring: at 20:51 Open the seat's
end check (seat_open) took the rim as the closed ring (-11.8 mm), its next four frames read the open seat at its closed
height (uncertain: the run stopped), and as nothing teaches the seat after Open the lid (31 Oct) the wrong height stayed:
the next runs' lifted checks (21:06, 21:13), the fixture fit held 12 mm low, refused every frame. Replayed through one
session (the frames from the lab's records, depth in mm; each judged with the fixture where its run placed it), the end
check passes on every frame, nothing is learned, and the next lifted check reads lifted. tests/data/gap_checks/open_seat."""
import pytest
import json
from pathlib import Path

import numpy as np

from alix_studio_lite.server.gap_check import DepthFrame, GapCheck, Intrinsics, load_params

pytestmark = pytest.mark.usefixtures("windows_rules")   # the rules before 4 Nov: gap_decider "windows"

DATA = Path(__file__).parent / "data" / "gap_checks" / "open_seat"
END = ["20261103_205133_383_gap_check_seat_open_seat_lifted",            # Open the seat's end check, 20:51:33
       "20261103_205134_194_gap_check_seat_open_uncertain"]              # its second frame (uncertain at the lab)
LIFT = "20261103_210600_102_gap_check_seat_lifted_refused"               # the next run's lifted check, 21:06 (refused)
CLOSED = "20261024_122902_343_gap_check_seat_closed_seat_closed"         # Open the seat's start check, 24 Oct


def frame(stem: str) -> tuple:
    rec = json.loads((DATA / f"{stem}.json").read_text())
    depth = np.load(DATA / f"{stem}.npz")["depth_mm"]
    k = rec["intrinsics"]
    return rec, DepthFrame(depth=depth, intrinsics=Intrinsics(k["fx"], k["fy"], k["cx"], k["cy"], depth.shape[1], depth.shape[0]),
                           cam_to_base=np.asarray(rec["camera_to_world"], dtype=float), hands=rec.get("hands") or {},
                           source=rec["source"])


def ask(gc, stem: str, expect: str):
    rec, fr = frame(stem)
    gc.at_place(rec.get("placed"))
    return gc.ask(expect, fr, q_deg=rec.get("q_deg"))


def seat_learned(gc) -> bool:
    return any("seat" in str(k) for k in gc.session._registration)


def test_the_open_seats_end_check_teaches_nothing_and_the_next_lift_reads_lifted():
    gc = GapCheck(params={**load_params(), "gap_cross_check": False}, records=None)
    for stem in END:
        a = ask(gc, stem, "seat_open")
        assert a.success and a.fixture_state == "seat_lifted", a.message      # the seat open: lifted proves it (lift_check)
        assert "registered" not in a.message and not seat_learned(gc)
    lift = ask(gc, LIFT, "seat_lifted")
    assert lift.success and lift.fixture_state == "seat_lifted", lift.message


def test_a_closed_seat_question_still_teaches_the_seats_height():
    """24 Oct, Open the seat's start check: the ring read +2 mm with the lid's +10 mm learned before (the lab's session
    then): the seat's own height is taught, as before."""
    gc = GapCheck(params={**load_params(), "gap_cross_check": False}, records=None)
    gc.core()
    gc.session._registration[gc.session._registration_key("")] = 0.010
    a = ask(gc, CLOSED, "seat_closed")
    assert a.success and a.fixture_state == "seat_closed", a.message
    assert seat_learned(gc)
