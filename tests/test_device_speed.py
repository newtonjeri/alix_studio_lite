"""A lid or seat plan's pole and hand moves at the speeds ALIX's robot executive sets (the owner, 24 Oct): its
alix_hw ProfiledPositionController matches each move's planned time, its hardware layer turns that into the pole's
speed % and each hand channel's SPEED_SET. device_speed.py ports it; these hold it to ALIX's own files (vendored) and
to the worked values on ALIX's current plans."""
import re

import pytest

from alix_studio_lite import vendor_paths
from alix_studio_lite.server import device_speed as ds

HPP = vendor_paths.ALIX / "ros/hw/alix_hw/include/alix_hw/conversions.hpp"
XACRO = vendor_paths.ALIX / "ros/hw/alix_hw/urdf/rm_side.ros2_control.xacro"


def test_the_constants_are_alix_hws_own():
    hpp = HPP.read_text()
    assert float(re.search(r"kLiftCruiseMpsPerPct = ([\d.]+);", hpp).group(1)) == ds.LIFT_CRUISE_MPS_PER_PCT
    assert float(re.search(r"kHandStrokeK = ([\d.]+);", hpp).group(1)) == ds.HAND_STROKE_K
    assert int(re.search(r"kHandSpeedSetMin = (\d+);", hpp).group(1)) == ds.HAND_SPEED_SET_MIN
    assert int(re.search(r"kHandSpeedSetMax = (\d+);", hpp).group(1)) == ds.HAND_SPEED_SET_MAX
    block = re.search(r"kHandMaxRad = \{(.*?)\};", hpp, re.S).group(1)
    assert tuple(float(v) for v in re.findall(r"([\d.]+),\s*//", block)) == ds.HAND_MAX_RAD
    assert int(re.search(r"lift_min_speed_pct:=(\d+)", XACRO.read_text()).group(1)) == ds.LIFT_MIN_SPEED_PCT


def test_the_controllers_parameters_are_read_from_alix():
    pole, hand = ds.params("left", "pole"), ds.params("right", "hand")
    assert (pole["accel"], pole["start_latency_down"], pole["start_latency_up"], pole["warm_latency_extra"]) == (0.166, 0.23, 0.33, 0.12)
    assert pole["min_match_speed"] == 0.0074 and pole["max_match_speed"] == pytest.approx(0.185)   # unset: kLiftMaxVelocity
    assert (hand["accel"], hand["start_latency"], hand["min_match_speed"], hand["max_match_speed"]) == (0.0, 0.115, 0.3603, 1.4038)
    assert pole["match_trajectory_duration"] and hand["match_trajectory_duration"] and hand["default_speed"] == 1.7


def test_the_pole_takes_the_speed_whose_profile_ends_on_the_plans_time():
    # seat_open's quarter pole: 75 mm down in 1.344 s cannot be met after its 0.23 s start: as fast as it goes
    assert ds.pole_speed_pct("left", 0.150, 0.075, 1.3444)[0] == 100
    # 75 mm down in 4 s: the trapezoid's smaller root, 0.0206 m/s, rounded UP to 12 %
    pct, how = ds.pole_speed_pct("left", 0.150, 0.075, 4.0)
    assert pct == 12 and "0.0206 m/s" in how and "ALIX's executive" in how
    # no planned time to match: the controller's default 0.09 m/s, 49 %; a crawl: never under the 4 % floor
    assert ds.pole_speed_pct("left", 0.150, 0.075, 0.0)[0] == 49
    assert ds.pole_speed_pct("left", 0.150, 0.149, 60.0)[0] == 4
    # going up starts later (0.33 s), a pole just stopped later still (+0.12 s): a little faster to end on time
    up = ds.pole_speed_pct("right", 0.075, 0.150, 2.5)[0]
    assert up > ds.pole_speed_pct("right", 0.150, 0.075, 2.5)[0] and ds.pole_speed_pct("right", 0.075, 0.150, 2.5, warm=True)[0] >= up


def test_the_hand_takes_one_rad_s_for_every_channel_each_in_its_own_speed_set():
    open_, grasp = [1000] * 6, [33, 33, 33, 33, 133, 944]          # the plans' grasp, as the lab's log wrote it
    sets, how = ds.hand_speed_sets("right", open_, grasp, 0.89)    # 1.3 rad in 0.89 s: capped at 1.4038 rad/s
    assert sets == [390, 390, 390, 390, 1000, 420] and "1.404 rad/s" in how
    assert ds.hand_speed_sets("right", open_, grasp, 0.0)[0] == [472, 472, 472, 472, 1000, 509]      # the default 1.7 rad/s
    assert ds.hand_speed_sets("right", open_, grasp, 5.0)[0] == [100, 100, 100, 100, 257, 108]       # floored: 0.3603 rad/s
    assert ds.hand_speed_sets("left", None, grasp, 0.89)[0] == [472, 472, 472, 472, 1000, 509]       # nothing read: no travel


def test_alix_s_plans_pole_and_hand_moves():
    from alix_studio_lite.server import mechanism

    vendor_paths.activate()
    # Open the seat keeps both poles at half length since 24 Oct 21:42: the single-arm Close the seat's pole move
    # (150 -> 75 mm; ALIX's plan of 7 Oct, vendored still, by its path: Lite offers the compound closing since 25 Oct)
    pole = next(e for e in mechanism.Plan("seat_close_right", mechanism.PLANS / "seat_close_right_toppra.json").entries
                if e.kind == "pole" and e.seconds > 0)
    assert ds.pole_speed_pct(pole.side, pole.first()[0], pole.last()[0], pole.seconds)[0] == 100
    plan = mechanism.Plan("seat_open")
    hand = next(e for e in plan.entries if e.kind == "hand" and e.side == "right" and e.seconds > 0)
    assert ds.hand_speed_sets("right", hand.hand_counts("first"), hand.hand_counts(), hand.seconds)[0] == [390, 390, 390, 390, 1000, 420]
