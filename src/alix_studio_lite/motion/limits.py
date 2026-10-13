"""The arm's limits, as one object both optimisers and every check use.

Sources:

- joint position limits and speed ratings: the robot URDF (``L_joint1..7`` / ``R_joint1..7``);
  the ratings are 180, 180, 225, 225, 225, 225, 225 deg/s;
- acceleration: 600 deg/s^2 on every joint, the arm's hardware limit. Hard, no override
  (decision D23);
- step: at most 10 deg between two lines (controller error 0x4000);
- the execution model ALIX fitted on the robot (``Resource/tracks/taught/EXECUTION_MODEL.json``):
  how far a joint overshoots a commanded turn, used for the joint-limit margin (G1) and the
  predicted path departure (G5).
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

from .. import vendor_paths

SIDES = ("left", "right")


@dataclass(frozen=True)
class LimitSet:
    side: str
    frame: str                                  # the contact frame paths are measured at
    rating_deg_s: tuple                         # (7,) joint speed ratings
    pos_deg: tuple                              # (7,) of (lower, upper)
    acc_deg_s2: float = 600.0                   # hardware limit (D23)
    rate_bar: float = 0.97                      # L2: 20 Hz band-limited speed <= 0.97 x rating
    rate_design: float = 0.95                   # TOPP-RA designs at 0.95 x rating
    acc_tol: float = 0.02                       # L3: 50 ms windowed mean <= 600 x 1.02
    acc_window_s: float = 0.050
    step_deg: float = 10.0                      # L4
    max_dev_mm: float = 10.0                    # L5: path change from the recording
    ends_tol_deg: float = 0.05                  # L6: first and last line vs the recording
    margin_warn_deg: float = 5.0                # note when a joint comes this close to a limit
    jerk_ref_rad_s3: float = 1500.0             # C2 reference (reported only)
    dt_gate: float = 0.005
    model: dict = field(default_factory=dict, compare=False, hash=False)

    @property
    def acc_bar_deg_s2(self) -> float:
        return self.acc_deg_s2 * (1.0 + self.acc_tol)

    def with_max_dev(self, max_dev_mm) -> "LimitSet":
        """The same limits with a region's MAX DEV applied (it can only tighten L5)."""
        if max_dev_mm is None:
            return self
        from dataclasses import replace

        return replace(self, max_dev_mm=min(float(self.max_dev_mm), float(max_dev_mm)))


def side_prefix(side: str) -> str:
    if side not in SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    return "L" if side == "left" else "R"


def contact_frame(side: str) -> str:
    """The glove contact frame ALIX measures taught paths at."""
    return f"{side_prefix(side)}_glove_frame_2"


def urdf_joint_limits(side: str, urdf_path=None) -> tuple[tuple, tuple]:
    """((lower, upper) x 7 in degrees, ratings x 7 in deg/s) for one arm, from the URDF."""
    pre = side_prefix(side)
    root = ET.parse(str(urdf_path or vendor_paths.URDF)).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    pos, rate = [], []
    for k in range(1, 8):
        j = joints.get(f"{pre}_joint{k}")
        lim = None if j is None else j.find("limit")
        if lim is None:
            raise ValueError(f"URDF has no limits for {pre}_joint{k}")
        pos.append((round(math.degrees(float(lim.get("lower"))), 6),
                    round(math.degrees(float(lim.get("upper"))), 6)))
        rate.append(round(math.degrees(float(lim.get("velocity"))), 6))
    return tuple(pos), tuple(rate)


def load_limits(side: str, *, max_dev_mm=None, urdf_path=None, model_path=None) -> LimitSet:
    vendor_paths.activate()
    import taught_tracks as tt

    pos, rate = urdf_joint_limits(side, urdf_path)
    model = tt.execution_model(model_path or vendor_paths.EXECUTION_MODEL)
    lim = LimitSet(side=side, frame=contact_frame(side), rating_deg_s=rate, pos_deg=pos, model=model)
    return lim.with_max_dev(max_dev_mm)
