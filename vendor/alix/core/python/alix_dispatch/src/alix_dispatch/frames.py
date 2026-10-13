"""Reference frame -> the controller's own base frame. ROS-free.

THE TRAP THIS MODULE EXISTS FOR. The controller knows nothing of the UGV body,
the pole, the hand or `alix_ref_frame` — its world starts at its own base, and
that base is MOUNTED: the controller's frame is `Ry(+90 deg)` applied to the
URDF's `*_base_link` (same origin, axes rotated). Three independent sources
agree and all are exact — the URDF's `*_base_link <- *_Sliding_Plate`
rpy="0 1.5707963 0", the controller's own `rm_get_install_pose() -> (0, 90, 0)`,
and a Kabsch fit over 16 hardware captures (90.00 deg about +Y, translation
<= 0.038 mm). Sending URDF-base poses straight to the SDK put every cleaning
target 90 deg out, and the chain was ACCEPTED and then failed in execution.

THE SECOND TRAP: the pole is prismatic and CARRIES THE ARM BASE, so this
transform is a function of pole height. Resolving a path at the wrong height
displaces every waypoint by the difference — 225 mm between the SRDF home
0.30 and a task's commanded 0.075 (contract A/C16.1).
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Sequence

import numpy as np

MOUNT_RY_DEG = 90.0


def _Ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def mount_matrix(ry_deg: float = MOUNT_RY_DEG) -> np.ndarray:
    """controller <- URDF `*_base_link`: a pure rotation, zero translation."""
    T = np.eye(4)
    T[:3, :3] = _Ry(math.radians(ry_deg))
    return T


def quat_to_R(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_euler_xyz(R: np.ndarray) -> tuple:
    """Extrinsic-xyz Euler (R = Rz*Ry*Rx) — the controller's convention.

    Contract A.6: `rx, ry, rz` in a controller pose are Euler RPY, NOT a
    rotation vector; reading them as one gave a 35 % error in the wrong
    direction on the reference path.
    """
    sy = min(1.0, max(-1.0, -R[2, 0]))
    return (math.atan2(R[2, 1], R[2, 2]), math.asin(sy), math.atan2(R[1, 0], R[0, 0]))


# ⚠ THIS MODULE CARRIES TWO EULER CONVENTIONS, ON PURPOSE, AND THEY ARE NOT
# INVERSES OF EACH OTHER:
#   * `R_to_euler_xyz`  EXTRINSIC xyz (R = Rz*Ry*Rx) — controller poses going OUT
#     (contract A.6).
#   * `rpy_deg_to_R`    INTRINSIC XYZ (R = Rx*Ry*Rz) — config rotation DELTAS
#     coming IN (delta_converter.py:8-13).
# Round-tripping a delta out through the first and back through the second
# silently corrupts any rotation with more than one non-zero component — which
# is 10 of the 11 points on toplid_right, worst case 6.4 deg. Verified
# numerically 2026-08-20: on a [12, -7, 33] deg delta the intrinsic form matches
# to 2.2e-16 and the extrinsic one is 0.135 out.


def slerp_R(Ra: np.ndarray, Rb: np.ndarray, u: float = 0.5) -> np.ndarray:
    """Shortest-arc interpolation between two rotations.

    `rm_movec` needs a full 6-DOF pose for its VIA, but a turnaround via is
    authored as a POSITION only (`arc_moves[].via_translation`). Giving it
    either endpoint's orientation would make the tool snap through the turn;
    the half-way slerp is what the runtime does between waypoints anyway (F75).
    """
    qa, qb = R_to_quat(Ra), R_to_quat(Rb)
    if float(np.dot(qa, qb)) < 0.0:
        qb = -qb                                   # short way round
    d = float(np.clip(np.dot(qa, qb), -1.0, 1.0))
    if d > 0.9995:                                 # numerically parallel
        q = qa + u * (qb - qa)
    else:
        th = math.acos(d)
        q = (math.sin((1 - u) * th) * qa + math.sin(u * th) * qb) / math.sin(th)
    q = q / (float(np.linalg.norm(q)) or 1.0)
    return quat_to_R(q[0], q[1], q[2], q[3])


def R_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 -> `[qx, qy, qz, qw]`, sign-normalised to `qw >= 0`."""
    tr = float(np.trace(R))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                      (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q = np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                      (R[2, 1] - R[1, 2]) / s])
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q = np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                      (R[0, 2] - R[2, 0]) / s])
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q = np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
                      (R[1, 0] - R[0, 1]) / s])
    return -q if q[3] < 0 else q


def pose_from_xyz_quat(vals: Sequence[float]) -> np.ndarray:
    """A task-config pose -> 4x4, in EITHER encoding.

    7 values are the ROS 2 form `[x, y, z, qx, qy, qz, qw]`, which every
    global-frame config uses. 6 are `[x, y, z, rx, ry, rz]` in RADIANS with
    `R = Rz.Ry.Rx` — `rm_pose_t`, the controller's own form, which is how a
    task authored in `arm_base` or `arm_world` writes its poses. The arm frames
    ARE controller frames, so they carry the controller's encoding.

    A 6-value pose is unambiguous (a quaternion needs 7), so accepting both
    costs nothing and lets a hand-edited file load.
    """
    vals = [float(v) for v in vals]
    T = np.eye(4)
    T[:3, 3] = vals[0:3]
    if len(vals) == 7:
        T[:3, :3] = quat_to_R(vals[3], vals[4], vals[5], vals[6])
    elif len(vals) == 6:
        rx, ry, rz = vals[3], vals[4], vals[5]
        T[:3, :3] = _Rz(rz) @ _Ry(ry) @ _Rx(rx)
    else:
        raise ValueError(
            f"a pose is 7 values [x y z qx qy qz qw] or 6 values "
            f"[x y z rx ry rz] in radians; got {len(vals)}")
    return T


def _Rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _Rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_deg_to_R(rpy_deg: Sequence[float]) -> np.ndarray:
    """`R = Rx*Ry*Rz` of the config's rotation DELTA, in degrees.

    Matches the runtime rule the configs are authored against:
    `R_final = R_delta @ R_start_pose` (left-multiplied).
    """
    rx, ry, rz = (math.radians(a) for a in rpy_deg)
    Rx = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
    Ry_ = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
    Rz = np.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
    return Rx @ Ry_ @ Rz


class FrameResolver:
    """`reference_frame` -> controller base, at a commanded pole height."""

    def __init__(self, model, side: str, reference_frame: str = "alix_ref_frame",
                 ry_deg: float = MOUNT_RY_DEG, tool_shift=(0.0, 0.0, 0.0)):
        self.model = model
        self.side = side
        self.prefix = "L" if side == "left" else "R"
        self.reference_frame = reference_frame
        self.base_link = f"{self.prefix}_base_link"
        self.pole_joint = f"{self.prefix}_sliding_plate_joint"
        self._mount = mount_matrix(ry_deg)
        self._cache: Dict[float, np.ndarray] = {}
        # RIGHT-multiplied, because it is a TOOL change: it moves which point of
        # the hand the pose describes, leaving the arm where it is. The base
        # transform above is left-multiplied for the same reason in reverse.
        self._tool = np.eye(4)
        self._tool[:3, 3] = np.asarray(tool_shift, dtype=float)

    #: Frames a task may declare that are NOT URDF link names. They are
    #: resolved against the task's OWN arm, which is what makes them local:
    #: `arm_base` means this side's `*_base_link`, and `arm_world` is the
    #: controller frame itself — already where `rm_movel` wants its poses, so
    #: it takes no transform at all and must NOT be mounted a second time.
    LOCAL_FRAMES = ("arm_base", "arm_world")

    def _resolved_frame(self) -> str:
        """The URDF link this task's declared frame corresponds to."""
        if self.reference_frame == "arm_base":
            return self.base_link
        return self.reference_frame

    def ref_to_controller(self, pole_m: float) -> np.ndarray:
        """T(controller base <- reference frame) with the pole at `pole_m`."""
        if self.reference_frame == "arm_world":
            return np.eye(4)          # the poses are already controller-frame
        key = round(float(pole_m), 6)
        if key not in self._cache:
            tw = self.model.link_world_transforms({self.pole_joint: key})
            frame = self._resolved_frame()
            for link in (self.base_link, frame):
                if link not in tw:
                    raise KeyError(
                        f"{link!r} is not a link of this URDF — the model and the "
                        f"config disagree about frames. Task frames are "
                        f"`alix_ref_frame`, `base_link`, or the arm-local "
                        f"`arm_base` / `arm_world`.")
            self._cache[key] = self._mount @ np.linalg.inv(tw[self.base_link]) @ tw[frame]
        return self._cache[key]

    def to_controller(self, T_ref: np.ndarray, pole_m: float) -> np.ndarray:
        """A pose expressed in the reference frame -> the controller's frame.

        `@ self._tool` retargets the pose when the controller drives a
        different tool frame from the one the task was authored against — see
        `program.CONTROLLER_TOOL_ALIAS`. It is the identity for every task that
        names a frame the controller has.
        """
        return self.ref_to_controller(pole_m) @ T_ref @ self._tool


def to_sdk_pose(T: np.ndarray) -> Dict[str, float]:
    """4x4 in the controller frame -> the SDK's pose fields (m, rad)."""
    rx, ry, rz = R_to_euler_xyz(T[:3, :3])
    return {"x": float(T[0, 3]), "y": float(T[1, 3]), "z": float(T[2, 3]),
            "rx": float(rx), "ry": float(ry), "rz": float(rz)}
