"""RM75 kinematics for the analysis — ONE FK, borrowed from production.

`alix_workspace.urdf_kinematics.UrdfModel` is the repo's canonical ROS-free FK
(test_kinematic_gate's witness; it reproduces the controller's reported TCP to
0.007 mm median on 25 hardware runs when fed the SDK joint array as
joint1..7 with URDF signs). This module only adapts it: picks the arm, adds
the `Arm_Tip` offset the URDF does not carry, and exposes limits and speed
ceilings from the xacro at full precision — the packaged alix.urdf rounds
them (3.14/3.92 rad/s, +/-3.1 rad).

Frames: everything is expressed in `{S}_base_link` (the arm's own base). The
controller's World frame is that base rotated Ry(+90 deg); the comparison
tool applies that when it needs the controller's numbers.
"""
from __future__ import annotations

import pathlib
import re
from typing import Optional, Sequence, Tuple

import numpy as np

try:
    from alix_workspace.urdf_kinematics import UrdfModel      # an editable install, if there is one
except ModuleNotFoundError:                                    # else straight from this repository, as
    import sys                                                 # tools/cut_dossier.py and plan_min_time do:
    _WS = pathlib.Path(__file__).resolve().parents[4] / "core/python/alix_workspace/src"
    if str(_WS) not in sys.path:                                # the taught CANFD lane installs nothing
        sys.path.insert(0, str(_WS))
    from alix_workspace.urdf_kinematics import UrdfModel

#: Joint speed ceilings, deg/s — rm_75_macro.xacro:82,113,144,175,206,237,294
#: and RMDemo_DualArmConcept/ROBOT_MODEL.md section 2.3 ("agree across SDK,
#: vendor page and the live controller"). NOT the packaged alix.urdf, which
#: rounds to 3.14/3.92 rad/s. NOT ROBOT_MODEL.md:114 ("uniform 3.14"), which
#: is stale.
QDOT_MAX_DEG_S: Tuple[float, ...] = (180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0)

#: Controller joint acceleration ceiling (joint_drive_max_acc), deg/s^2 —
#: ROBOT_MODEL.md section 2.3; alix_moveit_config joint_limits.yaml 10.472 rad/s^2.
QDDOT_MAX_DEG_S2 = 600.0

#: Position limits, deg — rm_75_macro.xacro (3.10668607 rad = 178.0 deg etc.).
JOINT_LIMITS_DEG: Tuple[Tuple[float, float], ...] = (
    (-178.0, 178.0), (-130.0, 130.0), (-178.0, 178.0), (-135.0, 135.0),
    (-178.0, 178.0), (-128.0, 128.0), (-360.0, 360.0))

#: Arm_Tip (the controller's tool zero) sits 15.3 mm back along +Z from
#: {S}_ConnectorLink — docs/kinematics/FRAME_MAP.md:31-35 (C14 offline, RM_MODEL_RM_ISF_E);
#: joint6 -> Arm_Tip then equals the vendor d7 = 0.1612 m exactly.
ARM_TIP_FROM_CONNECTOR_M = (0.0, 0.0, -0.0153)

#: Controller World = arm base rotated Ry(+90 deg) (FRAME_MAP.md:34).
MOUNT_RY_DEG = 90.0

#: Vendor pass-through limits (develop.realman-robotics.com en/robot/json/motionConfig):
#: "maximum angle between frames for each joint must not exceed 10 deg, and the
#: joint planning speed must not exceed 180 deg/s. Otherwise, the joint will not
#: respond." Error 0x4000 latches at > 10 deg target-vs-motor.
CANFD_MAX_STEP_DEG = 10.0
CANFD_MAX_RATE_DEG_S = 180.0

#: Curve-fitting mode (trajectory_mode 1) buys smoothness with LAG. Vendor
#: (develop.realman-robotics.com, JSON motionConfig / movePlan): "the larger the
#: smoothing coefficient, the smoother the trajectory; however, the lag effect
#: will be more noticeable, with a maximum lag of about 15 cycles."  In wall
#: clock the lag therefore scales with the stream period: 30 ms at 2 ms, 75 ms
#: at 5 ms, 150 ms at 10 ms.  Two consequences: a stream must keep sending its
#: last point for longer than this or the tail never executes, and a DTW-aligned
#: comparison CANNOT see this at all (it removes timing by construction) — lag
#: has to be measured against the command log.
CANFD_MAX_LAG_CYCLES = 15


def alix_urdf_path() -> pathlib.Path:
    import alix_workspace
    return pathlib.Path(alix_workspace.__file__).resolve().parent / "urdf" / "alix.urdf"


def side_prefix(side: str) -> str:
    s = str(side).upper()[0]
    if s not in ("L", "R"):
        raise ValueError(f"side must be L or R, got {side!r}")
    return s


def ry(deg: float) -> np.ndarray:
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


#: Tool / IK frames the URDF carries as fixed links on `*_ConnectorLink`
#: (ros/description/alix_description/urdf/ik_frames.xacro; zero rotation).
#: Resolved through the URDF chain when the link exists there, else from this
#: table — the same numbers, kept so an older packaged URDF still works.
TOOL_FRAMES_FROM_CONNECTOR_M = {
    "glove_frame_1":   (0.05,   0.0,    0.145),   # contact axis +x (R) / -x (L): the palm patch
    "glove_frame_2":   (0.0135, 0.0,    0.165),   # contact axis +z
    "glove_frame_3":   (0.075,  0.007,  0.17),
    "glove_frame_4":   (0.055,  0.007,  0.205),
    "tip_frame":       (0.015,  0.005,  0.23),
    "index_tip_frame": (0.0242, 0.0288, 0.225),
}


def canonical_frame(frame: str, side: str) -> str:
    """'glove_frame_1' | 'glove_1' | 'R_glove_frame_1' -> 'R_glove_frame_1';
    Arm_Tip / ConnectorLink pass through."""
    f = str(frame).strip()
    if f in ("Arm_Tip", "ConnectorLink"):
        return f
    if f == f"{side}_ConnectorLink":
        return "ConnectorLink"
    m = re.fullmatch(r"(?:[LR]_)?(glove|index_tip|tip)(?:_frame)?(?:_(\d))?", f)
    if m:
        kind, n = m.group(1), m.group(2)
        key = f"{kind}_frame_{n}" if kind == "glove" else f"{kind}_frame"
        if kind == "glove" and n is None:
            raise ValueError(f"glove frame needs a number: {frame!r}")
        return f"{side}_{key}"
    return f


_URDF_CACHE: dict = {}


def _parsed_urdf(path: pathlib.Path):
    """One parse per URDF file (and per change of it): an ArmKinematics is built per call in the
    lanes' measures — 446 parses in one certify, 2 s — and the model is read-only once built."""
    path = pathlib.Path(path)
    key = (str(path.resolve()), path.stat().st_mtime_ns)
    if key not in _URDF_CACHE:
        _URDF_CACHE.clear()
        _URDF_CACHE[key] = UrdfModel.from_file(str(path))
    return _URDF_CACHE[key]


class ArmKinematics:
    """FK / Jacobian for one RM75 arm, joint1..7 in URDF order, radians in.

    `frame` is the point whose path is reported: `Arm_Tip` (what the
    controller calls the tool zero, comparable with its recordings),
    `ConnectorLink` (the URDF flange), or a tool/IK frame — `glove_frame_1`,
    `glove_1`, `R_glove_frame_1`, `tip_frame`, … — resolved as a link of the
    URDF when it has one (alix.urdf does) and from
    TOOL_FRAMES_FROM_CONNECTOR_M otherwise. `frame_offset_m` adds a further
    translation expressed in the ConnectorLink frame.
    """

    def __init__(self, side: str = "L", frame: str = "Arm_Tip",
                 urdf: Optional[pathlib.Path] = None,
                 frame_offset_m: Optional[Sequence[float]] = None) -> None:
        self.side = side_prefix(side)
        self.frame = canonical_frame(frame, self.side)
        self.urdf = pathlib.Path(urdf) if urdf else alix_urdf_path()
        self.model = _parsed_urdf(self.urdf)
        self.base_link = f"{self.side}_base_link"
        self.tip_link = f"{self.side}_ConnectorLink"
        self.frame_source = "urdf"
        tail = np.eye(4)
        if self.frame == "Arm_Tip":
            tail[:3, 3] = ARM_TIP_FROM_CONNECTOR_M
            self.frame_source = "ConnectorLink + ARM_TIP_FROM_CONNECTOR_M"
        elif self.frame == "ConnectorLink":
            pass
        else:
            key = self.frame[2:] if self.frame.startswith(f"{self.side}_") else self.frame
            try:
                self.model.chain(self.base_link, self.frame)
                self.tip_link = self.frame                      # a real URDF link: use it
                self.frame_source = f"URDF link {self.frame} ({self.urdf.name})"
            except Exception:                                   # noqa: BLE001
                if key not in TOOL_FRAMES_FROM_CONNECTOR_M:
                    raise ValueError(f"unknown frame {frame!r}: Arm_Tip, ConnectorLink, or one of "
                                     f"{sorted(TOOL_FRAMES_FROM_CONNECTOR_M)} (optionally prefixed L_/R_)")
                off = np.asarray(TOOL_FRAMES_FROM_CONNECTOR_M[key], dtype=float)
                if self.side == "L":
                    off = off * np.array([-1.0, 1.0, 1.0])        # mirrored x on the left (ik_frames.xacro)
                tail[:3, 3] = off
                self.frame_source = "ConnectorLink + TOOL_FRAMES_FROM_CONNECTOR_M (link not in this URDF)"
        self.chain = self.model.chain(self.base_link, self.tip_link)
        self.joints = self.model.movable_joints(self.chain)
        self.joint_names = [j.name for j in self.joints]
        expected = [f"{self.side}_joint{i}" for i in range(1, 8)]
        if self.joint_names != expected:
            raise RuntimeError(f"chain movable joints {self.joint_names} != {expected}")
        if frame_offset_m is not None:
            tail[:3, 3] = tail[:3, 3] + np.asarray(frame_offset_m, dtype=float)
        self._tail = tail
        self._solver = None          # IKSolver built lazily (Jacobian only)

    # limits from the URDF actually loaded (rounded) and the xacro (exact)
    @property
    def urdf_limits_deg(self) -> list:
        return [(float(np.degrees(j.lower)), float(np.degrees(j.upper))) for j in self.joints]

    @property
    def urdf_qdot_max_deg_s(self) -> list:
        return [float(np.degrees(j.velocity)) for j in self.joints]

    def fk(self, q_rad: Sequence[float]) -> np.ndarray:
        return self.model.fk(self.chain, q_rad) @ self._tail

    def fk_batch(self, q_rad: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """(positions (N,3) m, rotations (N,3,3)) for N joint vectors — batched
        (`UrdfModel.fk_batch`, bit-identical to `fk` row by row, ~17x faster)."""
        q = np.asarray(q_rad, dtype=float)
        if q.shape[0] == 0:
            return np.empty((0, 3)), np.empty((0, 3, 3))
        T = self.model.fk_batch(self.chain, q) @ self._tail
        return T[:, :3, 3].copy(), T[:, :3, :3].copy()

    def jacobian(self, q_rad: Sequence[float]) -> np.ndarray:
        """6x7 geometric Jacobian at the ConnectorLink (base frame). The
        reported frame is a fixed offset of it; joint-rate mapping is unaffected
        for the linear part up to that lever arm, which is what utilization
        screens need."""
        if self._solver is None:
            from alix_workspace.ik import IKSolver
            self._solver = IKSolver(self.model, self.base_link, self.tip_link)
        return self._solver.jacobian(q_rad)

    def describe(self) -> str:
        return (f"{self.side} arm, frame={self.frame} [{self.frame_source}] (+{self._tail[:3, 3].round(4).tolist()} m "
                f"from {self.tip_link}), URDF={self.urdf.name}, joints={self.joint_names}")


def check_limits_deg(q_deg: np.ndarray, limits=JOINT_LIMITS_DEG) -> list:
    """Per-joint count of samples outside the xacro limits; never clamps."""
    q = np.asarray(q_deg, dtype=float)
    out = []
    for j in range(q.shape[1]):
        lo, hi = limits[j]
        u = int((q[:, j] < lo).sum()); o = int((q[:, j] > hi).sum())
        if u or o:
            out.append(f"joint{j+1}: {u} under / {o} over [{lo}, {hi}] deg")
    return out


def dh_fk_arm_tip(q_rad: Sequence[float]) -> np.ndarray:
    """The vendor standard-DH chain (what the CONTROLLER solves with), base ->
    Arm_Tip, from the frozen instrument in this experiment. Agrees with the
    URDF walk to 2.96 um / 0.57 millideg over 300 random configurations
    (reconstruct_tcp_path.py --self-test). Kept as a cross-check only."""
    import importlib
    import sys
    from teachlab import SRC_DIR
    if str(SRC_DIR) not in sys.path:
        sys.path.insert(0, str(SRC_DIR))
    mod = importlib.import_module("reconstruct_tcp_path")
    return mod.dh_fk(np.asarray(q_rad, dtype=float))
