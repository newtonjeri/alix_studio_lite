"""Minimal, dependency-free URDF kinematics for workspace analysis.

Parses a plain ``.urdf`` file with the standard library only
(``xml.etree.ElementTree``) and computes forward kinematics for any serial
chain with ``numpy`` + ``scipy`` rotations. No ROS, no ``yourdfpy``/``urdfpy``.

Supported joint types: ``revolute``, ``continuous``, ``prismatic`` (movable) and
``fixed``. ``<mimic>`` joints are honoured: their value is derived as
``multiplier * value(source) + offset`` (chained mimics are resolved), which
matters for e.g. the inspire hands where the distal finger joints mimic the
proximal ones. Mimic joints are not independent DOFs (excluded from IK).

Typical use::

    model = UrdfModel.from_file("rm_75.urdf")
    chain = model.chain("base_link", "Link7")          # ordered Joint list
    T = model.fk(chain, q)                              # 4x4, base_link -> tip

Xacro files must be expanded to a ``.urdf`` first (e.g. ``xacro robot.xacro >
robot.urdf``); that step is intentionally left to the user so this module pulls
in no ROS tooling.
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import math

import numpy as np
from scipy.spatial.transform import Rotation

logger = logging.getLogger("alix_workspace.urdf_kinematics")

_MOVABLE = {"revolute", "continuous", "prismatic"}


#: copied rather than rebuilt — see `Joint.motion_matrix`
_EYE4 = np.eye(4)
_EYE3 = np.eye(3)


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    lower: float = 0.0
    upper: float = 0.0
    velocity: float = 0.0
    effort: float = 0.0
    mimic: Optional[tuple] = None   # (source_joint_name, multiplier, offset) or None
    #: built once and reused — see `origin_matrix` / `motion_matrix`
    _origin_cache: Optional[np.ndarray] = field(default=None, repr=False, compare=False)
    _skew_cache: Optional[np.ndarray] = field(default=None, repr=False, compare=False)
    _skew_sq_cache: Optional[np.ndarray] = field(default=None, repr=False, compare=False)

    @property
    def movable(self) -> bool:
        return self.type in _MOVABLE

    @property
    def is_mimic(self) -> bool:
        return self.mimic is not None

    @property
    def independent(self) -> bool:
        """A real DOF: movable and not driven by a <mimic>."""
        return self.movable and self.mimic is None

    def origin_matrix(self) -> np.ndarray:
        """The joint's FIXED parent->child transform. CONSTANT, so built once.

        `rpy` and `xyz` do not change after parsing, yet this rebuilt the matrix
        through `Rotation.from_euler` on EVERY forward-kinematics evaluation —
        1.58 million calls in a single coverable stage, 38.5 s of a 160 s run,
        most of it scipy's array-API promotion (`xp_promote`, and 13 million
        `is_torch_array` checks) rather than the rotation itself. Cached, and
        returned READ-ONLY so an accidental write raises instead of silently
        corrupting the model for every later caller.
        """
        T = self._origin_cache
        if T is None:
            T = np.eye(4)
            T[:3, :3] = Rotation.from_euler("xyz", self.rpy).as_matrix()
            T[:3, 3] = self.xyz
            T.setflags(write=False)
            self._origin_cache = T
        return T

    def motion_matrix(self, value: float) -> np.ndarray:
        """The joint's motion at *value* — Rodrigues in closed form.

        The axis is fixed, so `R = I + sin(t) K + (1 - cos(t)) K^2` with K the
        axis' skew matrix precomputed once. Identical to
        `Rotation.from_rotvec(axis * value).as_matrix()` to floating point (the
        test asserts 1e-15) and free of the per-call scipy dispatch that made
        this the second-hottest function in the profile.
        """
        # `np.eye(4)` builds the identity from scratch 1.58 M times a stage —
        # 3.3 M calls and 4.0 s counting the one in the caller. Copying a
        # prebuilt one is the same array for a third of the cost.
        T = _EYE4.copy()
        if self.type == "prismatic":
            T[:3, 3] = self.axis * value
        elif self.type in ("revolute", "continuous"):
            K = self._skew_cache
            if K is None:
                a = np.asarray(self.axis, dtype=float)
                n = float(np.linalg.norm(a))
                a = a / n if n > 1e-12 else a
                K = np.array([[0.0, -a[2], a[1]],
                              [a[2], 0.0, -a[0]],
                              [-a[1], a[0], 0.0]])
                self._skew_cache = K
                self._skew_sq_cache = K @ K
            T[:3, :3] = (_EYE3 + math.sin(value) * K
                         + (1.0 - math.cos(value)) * self._skew_sq_cache)
        return T


@dataclass
class UrdfModel:
    """Parsed URDF: joints keyed by child link, plus the link adjacency."""

    joints_by_child: Dict[str, Joint] = field(default_factory=dict)
    name: str = ""

    # ------------------------------------------------------------------
    @classmethod
    def from_file(cls, urdf_path: str) -> "UrdfModel":
        root = ET.parse(urdf_path).getroot()
        return cls.from_element(root, name=root.get("name", ""))

    @classmethod
    def from_string(cls, urdf_xml: str) -> "UrdfModel":
        root = ET.fromstring(urdf_xml)
        return cls.from_element(root, name=root.get("name", ""))

    @classmethod
    def from_element(cls, root: ET.Element, name: str = "") -> "UrdfModel":
        joints_by_child: Dict[str, Joint] = {}
        for j in root.findall("joint"):
            origin = j.find("origin")
            xyz = _floats(origin.get("xyz") if origin is not None else None, [0, 0, 0])
            rpy = _floats(origin.get("rpy") if origin is not None else None, [0, 0, 0])
            axis_el = j.find("axis")
            axis = _floats(axis_el.get("xyz") if axis_el is not None else None, [0, 0, 1])
            n = np.linalg.norm(axis)
            axis = axis / n if n > 1e-12 else axis
            lim = j.find("limit")
            mim = j.find("mimic")
            mimic = None
            if mim is not None and mim.get("joint"):
                mimic = (mim.get("joint"),
                         float(mim.get("multiplier", 1.0)),
                         float(mim.get("offset", 0.0)))
            joint = Joint(
                name=j.get("name", ""),
                type=j.get("type", "fixed"),
                parent=j.find("parent").get("link"),
                child=j.find("child").get("link"),
                xyz=xyz,
                rpy=rpy,
                axis=axis,
                lower=float(lim.get("lower", 0.0)) if lim is not None else 0.0,
                upper=float(lim.get("upper", 0.0)) if lim is not None else 0.0,
                velocity=float(lim.get("velocity", 0.0)) if lim is not None else 0.0,
                effort=float(lim.get("effort", 0.0)) if lim is not None else 0.0,
                mimic=mimic,
            )
            joints_by_child[joint.child] = joint
        logger.info("Parsed URDF '%s': %d joints", name, len(joints_by_child))
        return cls(joints_by_child=joints_by_child, name=name)

    # ------------------------------------------------------------------
    def links(self) -> List[str]:
        ls = set()
        for jt in self.joints_by_child.values():
            ls.add(jt.parent)
            ls.add(jt.child)
        return sorted(ls)

    def tips(self) -> List[str]:
        parents = {jt.parent for jt in self.joints_by_child.values()}
        return [c for c in self.joints_by_child if c not in parents]

    def chain(self, base_link: str, tip_link: str) -> List[Joint]:
        """Ordered joints from ``base_link`` (exclusive) to ``tip_link``."""
        chain: List[Joint] = []
        cur = tip_link
        seen = set()
        while cur != base_link:
            if cur in seen:
                raise ValueError(f"cycle in URDF near '{cur}'")
            seen.add(cur)
            jt = self.joints_by_child.get(cur)
            if jt is None:
                raise ValueError(
                    f"no path from '{base_link}' to '{tip_link}' "
                    f"(stuck at link '{cur}')"
                )
            chain.append(jt)
            cur = jt.parent
        chain.reverse()
        return chain

    def movable_joints(self, chain: Sequence[Joint]) -> List[Joint]:
        return [jt for jt in chain if jt.movable]

    def fk(self, chain: Sequence[Joint], q: Sequence[float],
           base_transform: Optional[np.ndarray] = None) -> np.ndarray:
        """FK for ``chain``; ``q`` are the movable-joint values in chain order."""
        T = np.eye(4) if base_transform is None else np.array(base_transform, dtype=float)
        qi = 0
        q = np.asarray(q, dtype=float)
        for jt in chain:
            T = T @ jt.origin_matrix()
            if jt.movable:
                T = T @ jt.motion_matrix(float(q[qi]))
                qi += 1
        return T

    def fk_batch(self, chain: Sequence[Joint], Q: np.ndarray,
                 base_transform: Optional[np.ndarray] = None) -> np.ndarray:
        """:meth:`fk` for every ROW of ``Q`` at once -> (N, 4, 4).

        The same products in the same order as :meth:`fk` (origin, then motion, joint by joint),
        each one done for all N rows in a single batched matmul instead of N Python-level 4x4
        products: bit-identical results, ~17x faster on a 12k-line track. The per-point loop was
        the cost of every certify ladder and its tests (2026-09-22: 815k scalar fk calls in one
        test)."""
        Q = np.atleast_2d(np.asarray(Q, dtype=float))
        n = Q.shape[0]
        T = np.broadcast_to(np.eye(4) if base_transform is None
                            else np.asarray(base_transform, dtype=float), (n, 4, 4)).copy()
        qi = 0
        for jt in chain:
            T = T @ jt.origin_matrix()
            if jt.movable:
                t = Q[:, qi]
                M = np.broadcast_to(_EYE4, (n, 4, 4)).copy()
                if jt.type == "prismatic":
                    M[:, :3, 3] = np.asarray(jt.axis, dtype=float)[None, :] * t[:, None]
                elif jt.type in ("revolute", "continuous"):
                    if jt._skew_cache is None:
                        jt.motion_matrix(0.0)                    # fills the axis caches
                    M[:, :3, :3] = (_EYE3[None] + np.sin(t)[:, None, None] * jt._skew_cache[None]
                                    + (1.0 - np.cos(t))[:, None, None] * jt._skew_sq_cache[None])
                T = T @ M
                qi += 1
        return T

    def fk_frames(self, chain: Sequence[Joint], q: Sequence[float],
                  base_transform: Optional[np.ndarray] = None):
        """Like :meth:`fk` but also returns, for each movable joint, the world
        transform of the frame the joint acts in (i.e. after its ``origin`` but
        *before* its motion). Used to assemble the geometric Jacobian.

        Returns ``(T_tip, [(joint, T_pre), ...])`` for movable joints only.
        """
        T = np.eye(4) if base_transform is None else np.array(base_transform, dtype=float)
        qi = 0
        q = np.asarray(q, dtype=float)
        pre = []
        for jt in chain:
            T = T @ jt.origin_matrix()
            if jt.movable:
                pre.append((jt, T.copy()))
                T = T @ jt.motion_matrix(float(q[qi]))
                qi += 1
        return T, pre

    # ------------------------------------------------------------------
    # Whole-tree FK (every link's world transform), for collision checks.
    # ------------------------------------------------------------------
    def root_link(self) -> str:
        """The single link that is never a child (the kinematic tree root)."""
        children = set(self.joints_by_child)
        parents = {jt.parent for jt in self.joints_by_child.values()}
        roots = parents - children
        if len(roots) != 1:
            # Fall back to the most common ancestor if the URDF has stray links.
            return sorted(roots)[0] if roots else next(iter(parents))
        return next(iter(roots))

    def resolve_mimics(self, joint_values: Optional[Dict[str, float]] = None) -> Dict[str, float]:
        """Return ``joint_values`` with every ``<mimic>`` joint set to
        ``multiplier * value(source) + offset``. Chained mimics (a mimic whose
        source is itself a mimic, e.g. the inspire thumb) are resolved by
        iterating to a fixed point. Provided values for mimic joints are ignored
        (they are derived, not independent)."""
        vals = dict(joint_values or {})
        mimics = {jt.name: jt for jt in self.joints_by_child.values() if jt.is_mimic}
        for _ in range(len(mimics) + 1):
            changed = False
            for name, jt in mimics.items():
                src, mult, off = jt.mimic
                nv = mult * float(vals.get(src, 0.0)) + off
                if vals.get(name) != nv:
                    vals[name] = nv
                    changed = True
            if not changed:
                break
        return vals

    def link_world_transforms(self, joint_values: Optional[Dict[str, float]] = None,
                              base_transform: Optional[np.ndarray] = None) -> Dict[str, np.ndarray]:
        """World transform of *every* link given a (partial) joint map.

        ``joint_values`` maps joint name -> value; any movable joint not listed
        defaults to 0 (home). ``<mimic>`` joints are derived automatically from
        their source (so e.g. posing a finger's proximal joint also bends the
        distal ones). ``base_transform`` places the root link. The result maps
        link name -> 4x4. Walks the tree from :meth:`root_link` so branches
        (e.g. fingers off the palm) are all resolved.
        """
        joint_values = self.resolve_mimics(joint_values)
        # children adjacency: parent link -> list of joints
        by_parent: Dict[str, List[Joint]] = {}
        for jt in self.joints_by_child.values():
            by_parent.setdefault(jt.parent, []).append(jt)
        root = self.root_link()
        out: Dict[str, np.ndarray] = {
            root: np.eye(4) if base_transform is None else np.array(base_transform, dtype=float)
        }
        stack = [root]
        while stack:
            parent = stack.pop()
            for jt in by_parent.get(parent, []):
                T = out[parent] @ jt.origin_matrix()
                if jt.movable:
                    T = T @ jt.motion_matrix(float(joint_values.get(jt.name, 0.0)))
                out[jt.child] = T
                stack.append(jt.child)
        return out


def _floats(text: Optional[str], default) -> np.ndarray:
    if not text:
        return np.array(default, dtype=float)
    return np.array([float(x) for x in text.split()], dtype=float)
