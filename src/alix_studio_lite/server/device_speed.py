"""How fast a lid or seat plan's pole and hand moves go: as ALIX's robot executive sets them (the owner, 24 Oct: "like
ALIX's robot executive").

ALIX's task configs set the pole and hand speeds (a stage's ``speed_class``, the pole and hand scalings), MoveIt times
each move with them, and the saved plan keeps only that timing. On the robot ALIX's alix_hw ProfiledPositionController
takes a move's planned duration back to a speed (``handle_accepted``: the speed whose measured profile, after the
device's start latency, ends when the plan says, clamped to the calibrated range: early is benign, the device waits at
its target) and the hardware layer turns it into the SDK's units: the pole's speed % (lift_channel.cpp:
``lift_speed_percent``, at least ``lift_min_speed_pct``) and each hand channel's SPEED_SET (rm_side_system.cpp:
``hand_speed_set_from_rad_s``, the same rad/s on every channel). Ported here as written. The controllers' parameters
are read from ALIX's controllers_real.yaml (vendored); the hardware constants are conversions.hpp's and the floor is
rm_side.ros2_control.xacro's (tests/test_device_speed.py holds them to the vendored files).

A stage's own pole and hand moves (staging, Advanced) keep Lite's speeds (robot_real.POLE_SPEED_PCT, HAND_SPEED_SET).
"""
from __future__ import annotations

import math
from functools import lru_cache

from .. import vendor_paths

CONTROLLERS = vendor_paths.ALIX / "ros/mobility/alix_control/config/controllers_real.yaml"

LIFT_CRUISE_MPS_PER_PCT = 0.00185                   # kLiftCruiseMpsPerPct: the pole's cruise speed per speed %
LIFT_MAX_VELOCITY = LIFT_CRUISE_MPS_PER_PCT * 100.0  # kLiftMaxVelocity: 0.185 m/s at 100 %
LIFT_MIN_SPEED_PCT = 4                              # lift_min_speed_pct: LiftChannel's floor (rm_side.ros2_control.xacro)
#: kHandMaxRad: each SDK channel's travel (0 little, 1 ring, 2 middle, 3 index, 4 thumb flex, 5 thumb rotation)
HAND_MAX_RAD = (1.344, 1.344, 1.344, 1.344, 0.5236, 1.246165)
HAND_STROKE_K = 373.0                               # kHandStrokeK: SPEED_SET = k x rad/s / max_rad
HAND_SPEED_SET_MIN, HAND_SPEED_SET_MAX = 100, 1000  # kHandSpeedSetMin, kHandSpeedSetMax

#: ProfiledPositionController's parameter defaults (auto_declare), where controllers_real.yaml sets none
DEFAULTS = dict(default_speed=0.05, accel=0.0, start_latency=0.0, match_trajectory_duration=False,
                use_velocity_interface=False, start_latency_down=0.0, start_latency_up=0.0, warm_latency_extra=0.0,
                warm_window=2.0, min_match_speed=0.0, max_match_speed=0.0)


def _lround(x: float) -> int:
    """C++ std::lround: halves away from zero."""
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


def lift_speed_percent(velocity_m_s: float) -> int:
    """conversions.hpp: the speed % whose measured cruise is this speed, rounded UP (late is the harmful direction)."""
    return int(min(100, max(1, math.ceil(abs(velocity_m_s) / LIFT_CRUISE_MPS_PER_PCT - 1e-9))))


def profile_travel_time(distance: float, cruise: float, accel: float) -> float:
    """conversions.hpp: a symmetric trapezoid's time (triangular when it never cruises; d/v without an acceleration)."""
    d, v = abs(distance), abs(cruise)
    if v < 1e-9:
        return 0.0
    if accel <= 1e-9:
        return d / v
    if d >= v * v / accel:
        return d / v + v / accel
    return 2.0 * math.sqrt(d / accel)


def profile_speed_for_duration(distance: float, duration_s: float, accel: float,
                               max_speed: float = LIFT_MAX_VELOCITY) -> float:
    """conversions.hpp: the cruise speed whose profile covers ``distance`` in ``duration_s`` (the trapezoid's smaller
    root); ``max_speed`` when that time cannot be met, d/T without an acceleration."""
    d = abs(distance)
    if d < 1e-9 or duration_s <= 1e-9:
        return max_speed
    if accel <= 1e-9:
        return min(d / duration_s, max_speed)
    at = accel * duration_s
    disc = at * at - 4.0 * accel * d
    if disc <= 0.0:
        return max_speed
    return min(max((at - math.sqrt(disc)) / 2.0, 0.0), max_speed)


def hand_speed_set_from_rad_s(rad_per_s: float, channel: int) -> int:
    """conversions.hpp: a channel's SPEED_SET for this rad/s, within the hardware's 100-1000."""
    max_rad = HAND_MAX_RAD[channel]
    if max_rad <= 0.0:
        return HAND_SPEED_SET_MAX
    return int(min(HAND_SPEED_SET_MAX, max(HAND_SPEED_SET_MIN, _lround(HAND_STROKE_K * abs(rad_per_s) / max_rad))))


def hand_counts_to_rad(counts: int, channel: int) -> float:
    """conversions.hpp hand_hw_to_rad: 1000 (open) is 0 rad, 0 is the channel's whole travel."""
    return (1.0 - min(1000, max(0, int(counts))) / 1000.0) * HAND_MAX_RAD[channel]


@lru_cache(maxsize=1)
def _controllers() -> dict:
    import yaml

    return yaml.safe_load(CONTROLLERS.read_text(encoding="utf-8")) or {}


def params(side: str, device: str) -> dict:
    """``<side>_<device>_controller``'s parameters (device: pole | hand), the controller's defaults where none is set."""
    node = (_controllers().get(f"{side}_{device}_controller") or {}).get("ros__parameters") or {}
    out = {k: type(v)(node.get(k, v)) for k, v in DEFAULTS.items()}
    if out["max_match_speed"] <= 0.0:                   # the controller's own fallback
        out["max_match_speed"] = LIFT_MAX_VELOCITY
    return out


def matched_speed(p: dict, distance: float, duration_s: float, moving_up: bool = False, warm: bool = False) -> tuple:
    """ProfiledPositionController.handle_accepted: (speed, latency s, matched). The latency: by direction where the
    controller has one, plus its warm-start penalty when the previous goal ended within its window. No duration or no
    distance: the default speed (ALIX's sender forwards no velocities)."""
    latency = float(p["start_latency"])
    if moving_up and p["start_latency_up"] > 0.0:
        latency = float(p["start_latency_up"])
    elif not moving_up and p["start_latency_down"] > 0.0:
        latency = float(p["start_latency_down"])
    if p["warm_latency_extra"] > 0.0 and warm:
        latency += float(p["warm_latency_extra"])
    if p["match_trajectory_duration"] and p["use_velocity_interface"] and duration_s > 1e-3 and distance > 1e-6:
        speed = profile_speed_for_duration(distance, max(0.05, duration_s - latency), float(p["accel"]),
                                           float(p["max_match_speed"]))
        if p["min_match_speed"] > 0.0:
            speed = max(speed, float(p["min_match_speed"]))
        return speed, latency, True
    return float(p["default_speed"]), latency, False


def pole_speed_pct(side: str, from_m: float, to_m: float, duration_s: float, warm: bool = False) -> tuple[int, str]:
    """A plan's pole move: (the SDK's speed %, how it was found)."""
    p = params(side, "pole")
    v, latency, matched = matched_speed(p, abs(to_m - from_m), float(duration_s), to_m > from_m, warm)
    pct = min(100, max(LIFT_MIN_SPEED_PCT, lift_speed_percent(v))) if abs(v) > 1e-6 else LIFT_MIN_SPEED_PCT
    how = (f"{v:.4f} m/s for the plan's {duration_s:.2f} s after a {latency:.2f} s start" if matched
           else f"the controller's default {v:.4f} m/s (no planned time to match)")
    return pct, how + ", as ALIX's executive sets it"


def hand_speed_sets(side: str, from_counts, to_counts, duration_s: float) -> tuple[list, str]:
    """A plan's hand move: (each channel's SPEED_SET, how it was found). The distance is the largest channel's travel
    in radians, as the controller measures it; a channel with no reading takes its target (it does not move)."""
    p = params(side, "hand")
    d = 0.0
    for ch, target in enumerate(to_counts):
        if target is None:
            continue
        here = from_counts[ch] if from_counts is not None and from_counts[ch] is not None else target
        d = max(d, abs(hand_counts_to_rad(target, ch) - hand_counts_to_rad(here, ch)))
    v, latency, matched = matched_speed(p, d, float(duration_s))
    sets = [hand_speed_set_from_rad_s(v, ch) for ch in range(len(HAND_MAX_RAD))]
    how = (f"{v:.3f} rad/s for the plan's {duration_s:.2f} s after a {latency:.3f} s start" if matched
           else f"the controller's default {v:.2f} rad/s (no planned time to match)")
    return sets, how + ", as ALIX's executive sets it"
