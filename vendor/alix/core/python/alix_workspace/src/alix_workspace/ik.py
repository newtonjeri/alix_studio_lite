"""Numerical inverse kinematics for any URDF serial chain (ROS/MoveIt-free).

A damped least-squares (Levenberg-Marquardt) solver built on the forward
kinematics in :mod:`alix_workspace.urdf_kinematics`. It answers the two
questions the position-only reach *envelope* cannot:

  * can the end-effector reach a target **at a given orientation**, and
  * with what **joint configuration** (needed for collision checking).

No ROS, no MoveIt, no KDL/TRAC-IK binaries — just numpy/scipy. The geometric
Jacobian is assembled analytically from each joint's world axis + origin, and
a few random restarts within the URDF joint limits avoid local minima.

Typical use::

    model = UrdfModel.from_file("alix.urdf")
    solver = IKSolver(model, "base_link", "R_glove_frame_2")
    res = solver.solve(target_T)          # 4x4 base_link -> tip target
    if res.success:
        q_map = res.joint_map             # {joint_name: value} for FK/collision
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np
from scipy.spatial.transform import Rotation

from alix_workspace.urdf_kinematics import Joint, UrdfModel

def _cross3(a, b, out):
    """3-vector cross product, written out, into *out*.

    `np.cross` is general — it accepts stacked inputs and routes every call
    through `moveaxis` and `normalize_axis_tuple` — and the Jacobian calls it
    once per joint per iteration: 614,726 calls and 16.5 s of an 84 s coverable
    stage, for nine multiplications each. Writing into *out* also avoids the
    temporary. Bit-identical to `np.cross`, asserted in the test.
    """
    a0, a1, a2 = a[0], a[1], a[2]
    b0, b1, b2 = b[0], b[1], b[2]
    out[0] = a1 * b2 - a2 * b1
    out[1] = a2 * b0 - a0 * b2
    out[2] = a0 * b1 - a1 * b0
    return out


def _rotvec_from_matrix(m):
    """Rotation vector of a 3x3 rotation matrix, in closed form.

    This is `Rotation.from_matrix(m).as_rotvec()` written out for a single
    matrix. The solver calls it once per Newton iteration -- 2.0 M times in one
    toplid coverable map -- and scipy charges a great deal for the generality
    we do not use: a determinant, a Gram matrix against the identity through
    `np.isclose` (2.0 M calls, 45.7 s on its own), the array-API promotion
    layer, and a `Rotation` object built and thrown away.

    None of that guards anything here. The argument is always `R_tgt @ R_cur.T`
    -- a product of two rotation matrices -- so it is orthogonal to machine
    precision by construction, and scipy's re-orthogonalisation branch is dead
    code on every one of those calls.

    The algorithm below is scipy's own, followed branch for branch so the
    result is not merely equivalent but numerically the same: Markley's
    quaternion-from-matrix (`_rotation_xp._from_matrix_orthogonal`), the
    canonical sign convention (`_quat_canonical`), then the atan2 rotation
    vector with scipy's small-angle series below 1e-3 rad
    (`_rotation_xp.as_rotvec`). `argmax` takes the first maximum on a tie,
    which the comparison chain reproduces. Agreement with scipy is asserted
    over structured and random matrices in the test.
    """
    m00 = m[0, 0]; m01 = m[0, 1]; m02 = m[0, 2]
    m10 = m[1, 0]; m11 = m[1, 1]; m12 = m[1, 2]
    m20 = m[2, 0]; m21 = m[2, 1]; m22 = m[2, 2]
    trace = m00 + m11 + m22

    # argmax over (m00, m11, m22, trace), first maximum winning.
    choice = 0
    best = m00
    if m11 > best:
        choice = 1; best = m11
    if m22 > best:
        choice = 2; best = m22
    if trace > best:
        choice = 3

    if choice == 3:
        x = m21 - m12
        y = m02 - m20
        z = m10 - m01
        w = 1.0 + trace
    elif choice == 0:
        x = 1.0 - trace + 2.0 * m00
        y = m10 + m01
        z = m20 + m02
        w = m21 - m12
    elif choice == 1:
        x = m10 + m01
        y = 1.0 - trace + 2.0 * m11
        z = m21 + m12
        w = m02 - m20
    else:
        x = m20 + m02
        y = m21 + m12
        z = 1.0 - trace + 2.0 * m22
        w = m10 - m01

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x /= norm; y /= norm; z /= norm; w /= norm

    # Canonical quaternion: w >= 0, ties broken on x, then y, then z.
    if w < 0.0 or (w == 0.0 and (x < 0.0
                                 or (x == 0.0 and (y < 0.0
                                                   or (y == 0.0 and z < 0.0))))):
        x = -x; y = -y; z = -z; w = -w

    ax_norm = math.sqrt(x * x + y * y + z * z)
    angle = 2.0 * math.atan2(ax_norm, w)
    if angle <= 1e-3:
        angle2 = angle * angle
        scale = 2.0 + angle2 / 12.0 + 7.0 * angle2 * angle2 / 2880.0
    else:
        scale = angle / math.sin(angle / 2.0)
    return np.array([scale * x, scale * y, scale * z])


logger = logging.getLogger("alix_workspace.ik")


@dataclass
class IKResult:
    success: bool                       # position AND orientation within tol
    position_ok: bool                   # position within tol (orientation maybe not)
    orientation_ok: bool
    pos_err_m: float
    rot_err_deg: float
    q: np.ndarray = field(default_factory=lambda: np.zeros(0))   # movable-joint values, chain order
    joint_names: List[str] = field(default_factory=list)
    iterations: int = 0

    @property
    def joint_map(self) -> Dict[str, float]:
        return {n: float(v) for n, v in zip(self.joint_names, self.q)}


class IKSolver:
    def __init__(self, model: UrdfModel, base_link: str, tip_link: str,
                 base_transform: Optional[np.ndarray] = None, seed: int = 0,
                 locked: Optional[Dict[str, float]] = None):
        self.model = model
        self.base_T = np.eye(4) if base_transform is None else np.asarray(base_transform, dtype=float)
        self.chain: List[Joint] = model.chain(base_link, tip_link)
        # ``locked`` joints are pinned at a fixed value and EXCLUDED from the
        # solve (e.g. the pole/sliding-plate joint once a length is chosen): FK
        # still applies them, but they are not DOFs in the Jacobian.
        self.locked: Dict[str, float] = dict(locked or {})
        movable_all = model.movable_joints(self.chain)
        self.movable: List[Joint] = [jt for jt in movable_all if jt.name not in self.locked]
        self.names = [jt.name for jt in self.movable]
        # Columns of revolute (dexterous) joints — manipulability/condition use
        # ONLY these. Mixing the prismatic pole (column units m/m) with revolute
        # columns (m/rad) makes Yoshikawa's index unit-inconsistent and biases it
        # toward the pole's axis; the pole is a gross-positioning DOF, not a
        # dexterity DOF, so it is excluded from the dexterity metrics.
        self._rev_cols = [i for i, jt in enumerate(self.movable) if jt.type != "prismatic"]
        self._rng = np.random.default_rng(seed)
        # Per-joint sampling/clamping limits (continuous joints -> +/- pi).
        self.lower = np.array([self._lo(jt) for jt in self.movable]) if self.movable else np.zeros(0)
        self.upper = np.array([self._hi(jt) for jt in self.movable]) if self.movable else np.zeros(0)

    @staticmethod
    def _lo(jt: Joint) -> float:
        if jt.type == "continuous":
            return -np.pi
        if jt.lower == 0.0 and jt.upper == 0.0:
            # lower==upper==0 means a locked joint or an omitted <limit> —
            # sweeping it over the full circle inflated the workspace and
            # admitted infeasible IK seeds. Treat as FIXED at 0 (warned once
            # in reachability's sampler); declare type="continuous" for a
            # genuinely unlimited joint.
            return 0.0
        return jt.lower

    @staticmethod
    def _hi(jt: Joint) -> float:
        if jt.type == "continuous":
            return np.pi
        if jt.lower == 0.0 and jt.upper == 0.0:
            return 0.0
        return jt.upper

    # ------------------------------------------------------------------
    def _fk_frames(self, q: np.ndarray):
        """FK over the chain using solved ``q`` for free joints and the pinned
        value for locked joints. Returns (T_tip, [(jt, T_pre) for solved joints])."""
        T = self.base_T.copy()
        pre = []
        si = 0
        for jt in self.chain:
            T = T @ jt.origin_matrix()
            if jt.movable:
                if jt.name in self.locked:
                    val = self.locked[jt.name]
                else:
                    val = float(q[si]); pre.append((jt, T.copy())); si += 1
                T = T @ jt.motion_matrix(val)
        return T, pre

    def _jacobian(self, q: np.ndarray):
        """6xN geometric Jacobian (world frame) and the tip transform at ``q``."""
        T_tip, pre = self._fk_frames(q)
        p_e = T_tip[:3, 3]
        J = np.zeros((6, len(self.movable)))
        for i, (jt, T_pre) in enumerate(pre):
            axis_world = T_pre[:3, :3] @ jt.axis
            origin = T_pre[:3, 3]
            if jt.type == "prismatic":
                J[:3, i] = axis_world
                J[3:, i] = 0.0
            else:  # revolute / continuous
                _cross3(axis_world, p_e - origin, J[:3, i])
                J[3:, i] = axis_world
        return J, T_tip

    def jacobian(self, q) -> np.ndarray:
        """Public 6xN geometric Jacobian at joint values ``q`` (world frame)."""
        return self._jacobian(np.asarray(q, dtype=float))[0]

    def dexterity_jacobian(self, q) -> np.ndarray:
        """The geometric Jacobian restricted to the revolute (dexterous) columns
        — the unit-consistent basis for manipulability/condition number."""
        J = self.jacobian(q)
        return J[:, self._rev_cols] if (self._rev_cols and len(self._rev_cols) < J.shape[1]) else J

    def manipulability(self, q, kind: str = "translation") -> float:
        """Yoshikawa manipulability index ``w = sqrt(det(J J^T))`` at ``q``, over
        the revolute (dexterous) joints only.

        ``kind``: "translation" (3 linear rows — position dexterity, unit-clean;
        the default and what the cleaning task cares about), "rotation" (3 angular
        rows), or "full" (all 6 rows). 0 at a singularity, larger = more dexterous.
        """
        J = self.dexterity_jacobian(q)
        Js = {"translation": J[:3], "rotation": J[3:], "full": J}.get(kind, J[:3])
        det = float(np.linalg.det(Js @ Js.T))
        return float(np.sqrt(max(det, 0.0)))

    def condition_number(self, q, kind: str = "translation") -> float:
        """Jacobian condition number (sigma_max / sigma_min) at ``q``: 1 = isotropic,
        large = near-singular/ill-conditioned. Over the revolute joints only.
        ``kind`` as in :meth:`manipulability`."""
        J = self.dexterity_jacobian(q)
        Js = {"translation": J[:3], "rotation": J[3:], "full": J}.get(kind, J[:3])
        s = np.linalg.svd(Js, compute_uv=False)
        return float(s[0] / s[-1]) if s[-1] > 1e-12 else float("inf")

    @staticmethod
    def _pose_error(T_cur: np.ndarray, T_tgt: np.ndarray) -> np.ndarray:
        """6-vector twist [dx, dy, dz, rx, ry, rz] taking current -> target."""
        e = np.zeros(6)
        e[:3] = T_tgt[:3, 3] - T_cur[:3, 3]
        R_err = T_tgt[:3, :3] @ T_cur[:3, :3].T
        e[3:] = _rotvec_from_matrix(R_err)
        return e

    def solve(self, target_T: np.ndarray, q_init: Optional[np.ndarray] = None,
              pos_tol: float = 2e-3, rot_tol_deg: float = 3.0,
              max_iters: int = 120, restarts: int = 6,
              damping: float = 0.05) -> IKResult:
        """Solve IK for a 4x4 target (base_link frame). Returns the best result
        found across restarts (success only if BOTH tolerances are met)."""
        target_T = np.asarray(target_T, dtype=float)
        rot_tol = np.deg2rad(rot_tol_deg)
        n = len(self.movable)
        mid = 0.5 * (self.lower + self.upper)
        best: Optional[IKResult] = None

        for r in range(max(1, restarts)):
            if r == 0 and q_init is not None:
                q = np.clip(np.asarray(q_init, dtype=float), self.lower, self.upper)
            elif r == 0:
                q = mid.copy()
            else:
                q = self._rng.uniform(self.lower, self.upper)

            it = 0
            for it in range(1, max_iters + 1):
                J, T_cur = self._jacobian(q)
                e = self._pose_error(T_cur, target_T)
                pos_err = float(np.linalg.norm(e[:3]))
                rot_err = float(np.linalg.norm(e[3:]))
                if pos_err < pos_tol and rot_err < rot_tol:
                    break
                # damped least squares: dq = J^T (J J^T + λ²I)^-1 e
                JT = J.T
                lam2 = damping * damping
                dq = JT @ np.linalg.solve(J @ JT + lam2 * np.eye(6), e)
                # step-limit to keep the linearisation valid, then clamp to limits
                step = float(np.linalg.norm(dq))
                if step > 0.3:
                    dq *= 0.3 / step
                q = np.clip(q + dq, self.lower, self.upper)

            J, T_cur = self._jacobian(q)
            e = self._pose_error(T_cur, target_T)
            pos_err = float(np.linalg.norm(e[:3]))
            rot_err = float(np.linalg.norm(e[3:]))
            res = IKResult(
                success=(pos_err < pos_tol and rot_err < rot_tol),
                position_ok=(pos_err < pos_tol),
                orientation_ok=(rot_err < rot_tol),
                pos_err_m=pos_err, rot_err_deg=float(np.rad2deg(rot_err)),
                q=q.copy(), joint_names=list(self.names), iterations=it,
            )
            if best is None or _better(res, best):
                best = res
            if res.success:
                break
        return best  # type: ignore[return-value]


def _better(a: IKResult, b: IKResult) -> bool:
    """Prefer full success, then smaller position error, then orientation."""
    if a.success != b.success:
        return a.success
    if a.position_ok != b.position_ok:
        return a.position_ok
    if abs(a.pos_err_m - b.pos_err_m) > 1e-6:
        return a.pos_err_m < b.pos_err_m
    return a.rot_err_deg < b.rot_err_deg
