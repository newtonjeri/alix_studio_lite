"""C16.2 — does the sparse chain keep the clearance the dense path had?

F82 measured DEVIATION: how far joint-linear motion between targets strays from
MoveIt's path. That is a conservative proxy, not a verdict — 6 mm of shortcut in
open space is free, 6 mm beside the commode rim is not. This module answers the
actual question by posing the robot against the fixture and measuring distance.

THE RULE, from RM_API2's `run_hinge_verify`, which asked it first: the sparse
chain is SAFE where its minimum clearance stays above the margin, **or stays
within tolerance of the dense reference** — because a cleaning stroke
deliberately touches the fixture, so an absolute margin is meaningless there.
The dense path is the reference precisely because MoveIt collision-checked it.

ROS-free: trimesh + the alix URDF, no MoveIt, no planning scene.
"""

from __future__ import annotations

import math
import pathlib
from typing import Dict, List, Optional, Sequence

import numpy as np
import yaml

from alix_workspace.collision import RobotCollisionModel, build_fixture_manager
from alix_workspace.urdf_kinematics import UrdfModel


def _rpy(rpy_deg: Sequence[float]) -> np.ndarray:
    r, p, y = (math.radians(a) for a in rpy_deg)
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _axis_angle(axis: Sequence[float], angle_deg: float) -> np.ndarray:
    a = np.array(axis, dtype=float)
    n = np.linalg.norm(a)
    if n == 0.0:
        return np.eye(3)
    a = a / n
    th = math.radians(angle_deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def load_fixture(scene_yaml: str, articulation: Optional[Dict[str, str]] = None):
    """The fixture as ONE mesh, posed in base_link, at a named articulation.

    `articulation` maps part -> state name from the scene's own
    `angles_deg` table (e.g. {"lid": "closed", "seat": "closed"}); parts with
    no entry use their first declared angle. The states are the scene's, not
    invented here — a cleaning task states which one it assumes.
    """
    import trimesh
    scene_path = pathlib.Path(scene_yaml)
    fx = (yaml.safe_load(scene_path.read_text()) or {})["fixture"]
    pose = fx.get("pose", {})
    T_fix = np.eye(4)
    T_fix[:3, 3] = pose.get("position", [0.0, 0.0, 0.0])
    T_fix[:3, :3] = _rpy(pose.get("orientation_rpy_deg", [0, 0, 0]))

    articulation = articulation or {}
    meshes = []
    for part in ("body", "lid", "seat"):
        spec = fx.get(part)
        if not spec or "mesh" not in spec:
            continue
        rel = spec["mesh"].split("commode_c/")[-1]
        path = scene_path.parent / rel
        if not path.is_file():
            continue
        m = trimesh.load_mesh(str(path))
        scale = spec.get("scale")
        if scale and tuple(scale) != (1.0, 1.0, 1.0):
            m.apply_scale(scale)
        T = np.eye(4)
        joint = spec.get("joint")
        if joint:
            angles = joint.get("angles_deg", {})
            state = articulation.get(part) or next(iter(angles), None)
            ang = float(angles.get(state, 0.0)) if state else 0.0
            origin = np.array(joint.get("origin", [0, 0, 0]), dtype=float)
            R = _axis_angle(joint.get("axis", [0, 1, 0]), ang)
            # Rotate about the hinge: translate to the hinge, rotate, translate back.
            T[:3, :3] = R
            T[:3, 3] = origin - R @ origin
        m.apply_transform(T_fix @ T)
        meshes.append(m)
    if not meshes:
        raise FileNotFoundError(f"no fixture meshes found beside {scene_path}")
    return trimesh.util.concatenate(meshes)


class ClearanceProbe:
    """Minimum robot-to-fixture distance at a joint configuration."""

    def __init__(self, urdf_path: str, scene_yaml: str, side: str,
                 articulation: Optional[Dict[str, str]] = None,
                 include_hand: bool = True):
        self.model = UrdfModel.from_file(urdf_path)
        self.rcm = RobotCollisionModel.from_file(urdf_path)
        self.fixture = load_fixture(scene_yaml, articulation)
        self.mgr = build_fixture_manager(self.fixture)
        self.prefix = "L" if side == "left" else "R"
        hand = "ILH_left" if side == "left" else "IRH_right"
        self.link_filter = lambda name: (
            name.startswith(f"{self.prefix}_") or (include_hand and name.startswith(hand)))

    def at(self, joint_values: Dict[str, float]) -> float:
        """Minimum distance in METRES; 0.0 when something interpenetrates."""
        meshes = self.rcm.posed_meshes(self.model, joint_values, link_filter=self.link_filter)
        best = math.inf
        for m in meshes:
            d = self.mgr.min_distance_single(m)
            if d < best:
                best = d
            if best <= 0.0:
                return 0.0
        return 0.0 if best is math.inf else float(best)


def chain_clearance(probe: ClearanceProbe, stage: dict, indices: Sequence[int],
                    samples_per_leg: int = 8,
                    extra_joints: Optional[Dict[str, float]] = None) -> dict:
    """Worst clearance along the JOINT-LINEAR chain through `indices`.

    Samples between targets, because the risk of a shortcut is between the
    targets — at the targets themselves the chain sits on MoveIt's own path.
    """
    names = [n for n in stage["joint_names"] if n.startswith(f"{probe.prefix}_joint")]
    col = [stage["joint_names"].index(n) for n in names]
    dense = [[wp[i] for i in col] for wp in stage["positions"]]
    base = dict(extra_joints or {})

    worst, worst_at = math.inf, None
    for a, b in zip(indices, list(indices)[1:]):
        qa, qb = np.array(dense[a]), np.array(dense[b])
        for s in range(samples_per_leg + 1):
            q = qa + (s / samples_per_leg) * (qb - qa)
            d = probe.at({**base, **dict(zip(names, q))})
            if d < worst:
                worst, worst_at = d, (a, b, s)
    return {"min_clearance_mm": worst * 1000.0, "worst_leg": worst_at}


def dense_clearance(probe: ClearanceProbe, stage: dict,
                    extra_joints: Optional[Dict[str, float]] = None) -> dict:
    """Worst clearance along MoveIt's own path — the reference."""
    names = [n for n in stage["joint_names"] if n.startswith(f"{probe.prefix}_joint")]
    col = [stage["joint_names"].index(n) for n in names]
    base = dict(extra_joints or {})
    worst, worst_at = math.inf, None
    for k, wp in enumerate(stage["positions"]):
        q = [wp[i] for i in col]
        d = probe.at({**base, **dict(zip(names, q))})
        if d < worst:
            worst, worst_at = d, k
    return {"min_clearance_mm": worst * 1000.0, "worst_waypoint": worst_at}


def verdict(dense_mm: float, sparse_mm: float, margin_mm: float = 10.0,
            tolerance_mm: float = 2.0) -> dict:
    """SAFE when the shortcut clears the margin, or matches the dense path.

    The second clause is not a loophole: a cleaning stroke deliberately
    contacts the fixture, so an absolute margin is meaningless there, and the
    honest question becomes whether the shortcut is WORSE than what MoveIt
    already accepted.
    """
    lost = dense_mm - sparse_mm
    ok = sparse_mm >= margin_mm or lost <= tolerance_mm
    return {"safe": bool(ok), "dense_mm": dense_mm, "sparse_mm": sparse_mm,
            "clearance_lost_mm": lost, "margin_mm": margin_mm,
            "reason": ("clears the margin" if sparse_mm >= margin_mm else
                       "within tolerance of the dense reference" if lost <= tolerance_mm else
                       "gives up more clearance than the dense path had")}
