"""Is the arm at one of its named poses? (ROS-free; the node feeds it joint states.)

Newton, 2026-10-08: "Only pose 4 is used for depth based state inspection, and
maybe stain detection and position estimation; the other poses are only used
for coloured inspection." The perception service cannot tell from a request
which pose the robot is at — the request carries a mode and a fixture hint —
but the joint states say. The MoveIt group states
(`alix_moveit_config/config/group_states/arms_group_states.yaml`) are the
source of truth for the named poses; this reads them and matches.

    poses = load_group_states(default_group_states_file(), "rm75_arm_right")
    name = match_named_pose({"R_joint1": 0.56, ...}, poses)      # "inspection_pose_4" or ""
"""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from . import paths

#: Every joint of the named pose must be within this of the measured one. The
#: inspection poses differ from each other by radians; the planner reaches a
#: named state to millimetres (MEASURED 2026-10-07: 0.4 deg).
DEFAULT_TOLERANCE_RAD = 0.1

GROUP_STATES_RELATIVE = Path("ros") / "manipulation" / "alix_moveit_config" / "config" / "group_states" / "arms_group_states.yaml"


def default_group_states_file(share_dir: str | Path | None = None) -> Path | None:
    """The arms' group-states file: `<share_dir>/config/group_states/...` when the
    MoveIt package's config is installed and `share_dir` is given, else the
    source tree beside this core (the repository root is found by marker)."""
    if share_dir is not None:
        candidate = Path(share_dir) / "config" / "group_states" / "arms_group_states.yaml"
        if candidate.is_file():
            return candidate
    root = paths.repo_root()
    if root is not None:
        candidate = Path(root) / GROUP_STATES_RELATIVE
        if candidate.is_file():
            return candidate
    return None


def load_group_states(path: str | Path, group: str = "rm75_arm_right") -> dict[str, dict[str, float]]:
    """{pose name: {joint: radians}} of one group, from the fragment file."""
    with open(path) as handle:
        doc = yaml.safe_load(handle) or {}
    states = (doc.get("group_states") or {}).get(group) or {}
    return {str(name): {str(j): float(v) for j, v in (joints or {}).items()} for name, joints in states.items()}


def match_named_pose(joint_positions: dict[str, float], poses: dict[str, dict[str, float]],
                     tolerance_rad: float = DEFAULT_TOLERANCE_RAD) -> str:
    """The name of the pose every joint of which is within `tolerance_rad` of
    the measured positions, or "". A pose with a joint the measurement lacks
    cannot match. Ties go to the closest."""
    best, best_worst = "", math.inf
    for name, joints in poses.items():
        if not joints or any(j not in joint_positions for j in joints):
            continue
        worst = max(abs(float(joint_positions[j]) - v) for j, v in joints.items())
        if worst <= tolerance_rad and worst < best_worst:
            best, best_worst = name, worst
    return best
