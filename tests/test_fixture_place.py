"""The commode's place from the depth (the owner, 31 Oct: the depth camera to place the arms where a lid or seat task
starts, and the UGV's offset the same way; ALIX's GapCheckSession.read_place, vendored: alix_perception_core.fixture_place).

Its absolute place is off by the wrist camera's mount (a CAD value: inspection pose 4 and pose 1 read the same commode
11 and 15 mm and 3.2 deg apart), so what it gives is a CHANGE of place, between frames from the same arm pose with the
lid and the seat in the same state. Lab frames from tests/data/gap_checks/place (depth in mm): inspection pose 1, 26 and
27 Oct, the lid open with the seat down and with the seat open, each the frame nearest its day's median;
tools/placement_replay.py reads every recorded frame."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from alix_studio_lite.server.gap_check import DepthFrame, GapCheck, Intrinsics, load_params

DATA = Path(__file__).parent / "data" / "gap_checks" / "place"
SEAT_DOWN = ("20261026_192459_777_gap_check_lid_open_lid_open", "20261027_142603_133_gap_check_lid_open_lid_open")
SEAT_OPEN = ("20261026_185528_477_gap_check_seat_open_seat_lifted", "20261027_185016_134_gap_check_seat_open_seat_lifted")
HELD_LID = "20261031_110453_887_gap_check_lid_lifted_lid_lifted"      # inspection pose 4, the lid in the left hand


@pytest.fixture(scope="module")
def gc():
    return GapCheck(params={**load_params(), "gap_cross_check": False}, records=None)


def frame(stem: str, move=None) -> DepthFrame:
    rec = json.loads((DATA / f"{stem}.json").read_text())
    depth = np.load(DATA / f"{stem}.npz")["depth_mm"].astype(np.float32) / 1000.0
    k = rec["intrinsics"]
    cam, hands = np.asarray(rec["camera_to_world"], dtype=float), rec.get("hands") or {}
    if move is not None:                   # the camera moved by `move` (base_link): every point it measures moves by it
        cam = move @ cam
        hands = {n: list((move @ np.r_[np.asarray(p, dtype=float)[:3], 1.0])[:3]) for n, p in hands.items()}
    return DepthFrame(depth=depth, intrinsics=Intrinsics(k["fx"], k["fy"], k["cx"], k["cy"], depth.shape[1], depth.shape[0]),
                      cam_to_base=cam, hands=hands, source="native depth")


def change(a: dict, b: dict) -> np.ndarray:
    """b's place against a's, as the rigid move of the commode about its middle (fixture_place.change)."""
    from alix_perception_core import fixture_place as fp

    def pl(p):
        return fp.Placement(ok=True, dx_m=p["dx_m"], dy_m=p["dy_m"], dz_m=p["dz_m"], dyaw_deg=p["dyaw_deg"])

    c = fp.change(pl(a), pl(b), GC_POSE[0])
    return np.array([c["dx_m"], c["dy_m"], c["dz_m"], c["dyaw_deg"]])


GC_POSE: list = []
_READ: dict = {}


def read(gc, stem: str) -> dict:
    """A recorded frame's place, read once for the module (a frame as recorded always reads alike)."""
    if stem not in _READ:
        _READ[stem] = gc.place(frame(stem))
    return _READ[stem]


@pytest.fixture(scope="module", autouse=True)
def _pose(gc):
    read(gc, SEAT_DOWN[0])                 # loads the region meshes and the fixture's scene pose
    GC_POSE[:] = [gc._fixture_in_world]


def test_a_change_of_place_reads_alike_in_two_states(gc):
    """26 to 27 Oct the commode stood a few mm otherwise against the robot: read with the seat down and with it open,
    from inspection pose 1, the change agrees to a millimetre and half a degree."""
    down = [read(gc, s) for s in SEAT_DOWN]
    up = [read(gc, s) for s in SEAT_OPEN]
    assert all(p["ok"] for p in down + up), [p["note"] for p in down + up]
    assert {p["state"] for p in down} == {"lid open, seat down"} and {p["state"] for p in up} == {"lid open, seat open"}
    a, b = change(*down), change(*up)
    assert np.all(np.abs(a[:3] - b[:3]) <= 0.0015) and abs(a[3] - b[3]) <= 0.5, (a, b)
    assert abs(a[1]) >= 0.002                         # there was a change to read (y about -3.5 mm)


@pytest.mark.parametrize("move, yaw_deg", [((0.0, -0.03, 0.0), 0.0), ((0.03, 0.02, -0.005), -3.0)])
def test_a_known_move_is_found_from_where_it_was_last_read(gc, move, yaw_deg):
    """The camera moved by a known move moves every point it measures by it: read from where the commode was last
    read, the place moves by it to a millimetre and 0.3 deg (a docking's miss: 30 mm, 3 deg)."""
    base = read(gc, SEAT_OPEN[0])
    from alix_perception_core import fixture_place as fp

    A = fp.moved(GC_POSE[0], base["dx_m"], base["dy_m"], base["dz_m"], base["dyaw_deg"])
    o = A[:3, :3] @ np.asarray(fp.DEFAULT_PLACE.centre_body) + A[:3, 3]
    t = math.radians(yaw_deg)
    D = np.eye(4)
    D[:3, :3] = [[math.cos(t), -math.sin(t), 0.0], [math.sin(t), math.cos(t), 0.0], [0.0, 0.0, 1.0]]
    D[:3, 3] = o - D[:3, :3] @ o + np.asarray(move)
    again = gc.place(frame(SEAT_OPEN[0], D), start=(base["dx_m"], base["dy_m"], base["dz_m"], base["dyaw_deg"]))
    assert again["ok"], again["note"]
    got = change(base, again)
    assert np.allclose(got[:3], move, atol=0.001) and abs(got[3] - yaw_deg) <= 0.3, got


def test_a_lid_in_a_hand_gives_no_place(gc):
    """31 Oct 11:04: the lid lifted by the left hand moves with the hand: no place, and the note says why."""
    p = read(gc, HELD_LID)
    assert not p["ok"] and p["state"] == "" and "between closed and open" in p["note"], p["note"]
