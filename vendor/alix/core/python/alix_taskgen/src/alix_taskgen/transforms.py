"""Frame transform helpers for cleaning path generation.

The generator keeps the stages explicit:

* STL-local coordinates are placed in `base_link` with the scene/manual pose.
* Arm filtering is evaluated in `base_link` around y=0.
* Kept points are converted to `alix_ref_frame` for start-pose deltas.
"""

from dataclasses import dataclass

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation


# Derived from the fixed URDF chain (butterfli_description/urdf/butterfli.urdf.xacro
# + ugv.xacro + pole.xacro), all joints identity rotation:
#   base_link -> UGV_body        (0,     0,      0.195)
#   UGV_body  -> R_Pole          (-0.01, -0.124, 0.296)
#   R_Pole    -> alix_ref_frame (0, 0,      0.035)
# Must match the TF2 fallback in cleaning_tasks/src/common/task_base.cpp.
BASE_TO_BUTTERFLI_REF_TRANSLATION = np.array([-0.01, -0.124, 0.526], dtype=float)
BASE_TO_BUTTERFLI_REF_ROTATION = Rotation.identity()


@dataclass(frozen=True)
class Transform:
    translation: np.ndarray
    rotation: Rotation

    def apply_points(self, points):
        points = np.asarray(points, dtype=float)
        return self.rotation.apply(points) + self.translation

    def apply_vectors(self, vectors):
        return self.rotation.apply(np.asarray(vectors, dtype=float))

    def inverse(self):
        inv_rotation = self.rotation.inv()
        return Transform(inv_rotation.apply(-self.translation), inv_rotation)

    def compose(self, other):
        return Transform(
            self.apply_points(other.translation),
            self.rotation * other.rotation,
        )


def pose_rpy_to_transform(values):
    """Scene-placement pose (x y z roll pitch yaw, degrees) → Transform.

    CONVENTION CONTRACT — do not change: scene/fixture placement RPY uses the
    EXTRINSIC fixed-axis convention (scipy lowercase "xyz"), which is exactly
    tf2::Quaternion::setRPY — the call the C++ ArticulatedSceneBuilder feeds
    every scene.yaml / washroom_scene.yaml ``orientation_rpy_deg`` through.
    Cleaning-POINT rotations use a different contract: intrinsic "XYZ"
    (Eigen AngleAxis(r,X)*AngleAxis(p,Y)*AngleAxis(y,Z) in task_base.cpp) —
    see delta_converter/pose_generator. Both are pinned by
    test_pipeline.py::test_euler_convention_contracts.
    """
    if len(values) != 6:
        raise ValueError("pose RPY values must be x y z roll pitch yaw")
    translation = np.asarray(values[:3], dtype=float)
    rotation = Rotation.from_euler("xyz", values[3:], degrees=True)
    return Transform(translation, rotation)


def pose_quat_to_transform(values):
    if len(values) != 7:
        raise ValueError("pose quaternion values must be x y z qx qy qz qw")
    translation = np.asarray(values[:3], dtype=float)
    rotation = Rotation.from_quat(values[3:])
    return Transform(translation, rotation)


def identity_transform():
    return Transform(np.zeros(3, dtype=float), Rotation.identity())


def base_to_alix_ref_frame_pose():
    """Pose of `alix_ref_frame` origin expressed in `base_link`."""
    return Transform(BASE_TO_BUTTERFLI_REF_TRANSLATION.copy(), BASE_TO_BUTTERFLI_REF_ROTATION)


def base_to_ref_transform():
    """Coordinate transform that maps `base_link` points into `alix_ref_frame`."""
    return base_to_alix_ref_frame_pose().inverse()


def ref_to_base_transform():
    """Coordinate transform that maps `alix_ref_frame` points into `base_link`."""
    return base_to_alix_ref_frame_pose()


def stl_to_ref_transform(stl_to_base):
    return base_to_ref_transform().compose(stl_to_base)


def transform_waypoints(waypoints, transform):
    transformed = []
    for waypoint in waypoints:
        # Frame-independent metadata (e.g. the ring planner's ring/theta
        # tags, G50) rides along untouched; only the geometric keys are
        # re-expressed.
        extra = {k: v for k, v in waypoint.items()
                 if k not in ("position", "normal", "tangent")}
        transformed.append({
            **extra,
            "position": transform.apply_points([waypoint["position"]])[0].tolist(),
            "normal": _unit(transform.apply_vectors([waypoint["normal"]])[0]).tolist(),
            "tangent": _unit(transform.apply_vectors([waypoint["tangent"]])[0]).tolist(),
        })
    return transformed


def transform_mesh(mesh, transform):
    matrix = np.eye(4)
    matrix[:3, :3] = transform.rotation.as_matrix()
    matrix[:3, 3] = transform.translation
    copy = mesh.copy()
    copy.apply_transform(matrix)
    return copy


def _unit(vector):
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)
    if norm < 1e-12:
        return vector
    return vector / norm