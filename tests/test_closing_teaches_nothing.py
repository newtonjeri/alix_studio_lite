"""A closing's outcome check judges, but teaches the gap check's session nothing (the owner, 31 Oct: the sequence no longer
finished once the closings ran in it). On 31 Oct Close the lid's end check (inspection pose 1) read the closed lid 22 to 28
mm above its nominal top, the fixture fit putting the lid 5 to 10 deg up and the fixture at the model's own height, and
taught that as the session's registration; the colour models' closed decision then made it +32 mm. The next sequence's
Open the lid lift check, its fit held 10 mm short of that, read the lifted lid closed (11:28, five frames uncertain) and
Open the seat's lift check found no seat in its band (11:36, five frames refused). The same frames through one session,
the closings asked as the runner asks them (``closing``): the closings pass as they did, nothing is learned, and both
lift checks read lifted. tests/data/gap_checks/closing holds the frames from the lab's records (depth in mm)."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from alix_studio_lite.server.gap_check import DepthFrame, GapCheck, Intrinsics, load_params

DATA = Path(__file__).parent / "data" / "gap_checks" / "closing"
SEAT_CLOSED = "20261031_111355_303_gap_check_seat_closed_seat_closed"        # Close the seat's end (11:13)
LID_CLOSED = ["20261031_111629_425_gap_check_lid_closed_lid_closed",         # Close the lid's end (11:16)
              "20261031_111629_932_gap_check_lid_closed_lid_closed"]
LID_LIFT = "20261031_112831_141_gap_check_lid_lifted_uncertain"              # the next Open the lid's lift (11:28)
SEAT_LIFT = "20261031_113639_217_gap_check_seat_lifted_refused"              # the next Open the seat's lift (11:36)


def frame(stem: str) -> DepthFrame:
    rec = json.loads((DATA / f"{stem}.json").read_text())
    depth = np.load(DATA / f"{stem}.npz")["depth_mm"].astype(np.float32) / 1000.0
    k = rec["intrinsics"]
    return DepthFrame(depth=depth, intrinsics=Intrinsics(k["fx"], k["fy"], k["cx"], k["cy"], depth.shape[1], depth.shape[0]),
                      cam_to_base=np.asarray(rec["camera_to_world"], dtype=float), hands=rec.get("hands") or {},
                      source=rec["source"])


def session(**params) -> GapCheck:
    return GapCheck(params={**load_params(), "gap_cross_check": False, **params}, records=None)


def test_the_closings_end_checks_leave_the_next_lift_checks_as_they_were():
    gc = session()
    key = gc.session._registration_key("")
    assert gc.ask("seat_closed", frame(SEAT_CLOSED), closing=True).fixture_state == "seat_closed"
    for stem in LID_CLOSED:
        a = gc.ask("lid_closed", frame(stem), closing=True)
        assert a.fixture_state == "lid_closed" and "this frame only" in a.message      # judged on its own height
    assert key not in gc.session._registration and key + ("seat",) not in gc.session._registration   # nothing learned
    assert gc.ask("lid_lifted", frame(LID_LIFT)).fixture_state == "lid_lifted"
    seat = gc.ask("seat_lifted", frame(SEAT_LIFT))
    assert seat.success and seat.fixture_state == "seat_lifted", seat.message


def test_as_it_was_the_closed_lid_taught_the_height_and_the_lift_checks_could_not_judge():
    """The same end frames asked as a plain question, which teaches as every check did until now (and every check but
    a closing's still does): the lid 5 to 10 deg up teaches 22 to 31 mm before its own fit is consulted; with the +32 mm
    the models' closed decision made of it (11:16:29, not replayed here), the lift checks fail as at the lab."""
    gc = session()
    key = gc.session._registration_key("")
    for stem in LID_CLOSED:
        gc.ask("lid_closed", frame(stem))
    assert 0.020 <= gc.session._registration[key] <= 0.032
    gc.session._registration[key] = 0.032
    assert gc.ask("lid_lifted", frame(LID_LIFT)).fixture_state == "uncertain"
    assert not gc.ask("seat_lifted", frame(SEAT_LIFT)).success


def test_the_models_closed_decision_teaches_nothing_in_a_closing():
    gc = session(gap_cross_check=True)
    key = gc.session._registration_key("")
    fr = frame(LID_CLOSED[1])
    closed = SimpleNamespace(fixture_state="closed", state_source="depth+colour", state_confidence=0.99,
                             diagnostics={"depth_offset_mm": 32.0, "fusion": "", "colour_state": "", "depth_state": ""})

    def check(closed_within_deg):
        return gc.session.check("lid_closed", fr.depth, fr.intrinsics, fr.cam_to_base, fr.hands, models=lambda: closed,
                                closed_within_deg=closed_within_deg)

    assert check(25.0).state == "lid_closed" and key not in gc.session._registration
    assert check(0.0).state == "lid_closed" and gc.session._registration[key] > 0.02     # a plain question: taught
