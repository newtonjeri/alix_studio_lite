"""Where the wrist camera and the hands are, from the arms' own readings: ALIX's URDF and its forward
kinematics (alix_workspace.urdf_kinematics), no ROS.

On the robot the gap check had these from TF, which robot_state_publisher computes on the same URDF from
the joint states. Here they come from what Lite reads of each arm: its seven joints, its pole and its hand.
The camera is ``camera_depth_optical_frame`` on the right wrist; with the right arm at ALIX's
inspection_pose_4 and the pole at 150 mm it is where the robot's TF put it on 12 Oct within 0.3 mm and
0.08 degrees (tests/test_mechanism.py).
"""
from __future__ import annotations

import importlib.util
from functools import lru_cache

import numpy as np

from .. import vendor_paths

URDF = vendor_paths.ALIX / "core" / "python" / "alix_workspace" / "src" / "alix_workspace" / "urdf" / "alix.urdf"
HAND_UNITS = vendor_paths.ALIX / "ros" / "hw" / "alix_hw" / "scripts" / "hand_units.py"
CAMERA = "camera_depth_optical_frame"
COLOUR = "camera_color_optical_frame"          # the colour models' camera (status_check, the gap check's cross-check)
REF = "alix_ref_frame"                         # the node's target frame: the fixture's pose and the regions are in it
PREFIX = {"right": "R", "left": "L"}


@lru_cache(maxsize=1)
def model():
    vendor_paths.activate()
    from alix_workspace.urdf_kinematics import UrdfModel

    return UrdfModel.from_file(str(URDF))


@lru_cache(maxsize=1)
def hand_units():
    """ALIX's hand unit converter (0-1000 counts <-> the URDF's finger radians), a script, loaded by its path."""
    spec = importlib.util.spec_from_file_location("alix_hand_units", HAND_UNITS)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def joint_values(readings: dict) -> tuple[dict, dict]:
    """({URDF joint: value}, {side: what is missing}) from Lite's readings of each arm,
    ``{"right": {"q_deg": [7 joints], "pole_mm": mm, "hand": [6 counts]}, "left": ...}``. A reading that is
    missing is left out and named: forward kinematics would put it at zero, so its frames are not used."""
    hu = hand_units()
    out: dict = {}
    missing: dict = {}
    for side, p in PREFIX.items():
        r = readings.get(side) or {}
        gone = []
        q = r.get("q_deg")
        if q is not None and len(q) >= 7:
            out.update({f"{p}_joint{i + 1}": float(np.radians(float(q[i]))) for i in range(7)})
        else:
            gone.append("arm")
        if r.get("pole_mm") is not None:
            out[f"{p}_sliding_plate_joint"] = float(r["pole_mm"]) / 1000.0
        else:
            gone.append("pole")
        hand = r.get("hand")
        if hand is not None and len(hand) >= 6:
            out.update({hu.joint_name(side, ch): hu.hw_to_rad(float(hand[ch]), ch) for ch in range(6)})
        else:
            gone.append("hand")
        if gone:
            missing[side] = gone
    return out, missing


def camera_to_base(readings: dict) -> np.ndarray | None:
    """base_link <- the right wrist camera's depth optical frame, or None when the right arm or its pole was
    not read (the check then refuses, as the node did without TF)."""
    values, missing = joint_values(readings)
    if {"arm", "pole"} & set(missing.get("right", [])):
        return None
    return model().link_world_transforms(values)[CAMERA]


def camera_frames(readings: dict) -> dict | None:
    """The transforms the perception node had from TF, all in base_link: ``depth`` (base_link <- the depth optical
    frame), ``colour`` (base_link <- the colour optical frame) and ``ref`` (base_link <- alix_ref_frame); None when the
    right arm or its pole was not read."""
    values, missing = joint_values(readings)
    if {"arm", "pole"} & set(missing.get("right", [])):
        return None
    links = model().link_world_transforms(values)
    return dict(depth=links[CAMERA], colour=links[COLOUR], ref=links[REF])


def hand_positions(readings: dict, names) -> dict:
    """{hand frame: (x, y, z) in base_link} for every frame in ``names`` whose arm, pole and hand were all read;
    the others are left out (the gap check names them as missing, as it named the frames TF could not give)."""
    values, missing = joint_values(readings)
    links = model().link_world_transforms(values)
    out = {}
    for name in names:
        side = "left" if name.startswith("ILH_") else "right" if name.startswith("IRH_") else None
        if side is None or missing.get(side) or name not in links:
            continue
        out[name] = tuple(float(v) for v in links[name][:3, 3])
    return out
