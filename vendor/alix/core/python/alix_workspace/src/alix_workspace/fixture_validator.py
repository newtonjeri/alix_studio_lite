"""Fixture-specific workspace validation (optional example domain, ROS-free).

For every fixture (toilet, urinal, sink, mirror, floor, wall) this module turns
the declarative ``fixtures`` section of the config into:

* plain-dict collision primitives (box / cylinder / plane) — JSON / web /
  external-ROS-bridge friendly, no ROS message types,
* approach poses (position + quaternion dicts) from the approach vectors,
* a clearance-envelope check, and
* a reachability validation of cleaning waypoints, using
  :class:`~alix_workspace.reachability.ReachabilityAnalyzer`.

This is part of the example washroom domain; delete or replace it for another
application.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from alix_workspace.reachability import ReachabilityAnalyzer

logger = logging.getLogger("alix_workspace.fixture_validator")


@dataclass
class WaypointReport:
    """Result of validating one fixture's cleaning waypoints."""

    fixture: str
    arm: str
    total: int
    reachable: int
    unreachable_indices: List[int] = field(default_factory=list)

    @property
    def reachable_fraction(self) -> float:
        return self.reachable / self.total if self.total else 0.0

    @property
    def all_reachable(self) -> bool:
        return self.total > 0 and not self.unreachable_indices


def approach_vector_to_quaternion(approach: Sequence[float]) -> np.ndarray:
    """Map an approach *direction* to a tool orientation quaternion (x,y,z,w).

    Convention: the tool's local +Z axis points *along* the approach vector
    (i.e. into the surface). The remaining DoF is resolved by keeping the tool
    +X as horizontal as possible.
    """
    z_axis = np.array(approach, dtype=float)
    norm = np.linalg.norm(z_axis)
    if norm < 1e-9:
        raise ValueError("approach vector must be non-zero")
    z_axis /= norm

    # Pick a world up reference that is not parallel to z_axis.
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(up, z_axis)) > 0.95:
        up = np.array([1.0, 0.0, 0.0])

    x_axis = np.cross(up, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)

    rot = np.column_stack((x_axis, y_axis, z_axis))
    return Rotation.from_matrix(rot).as_quat()  # (x, y, z, w)


class FixtureWorkspaceValidator:
    """Validate cleaning workspaces against the declared fixtures."""

    def __init__(self, config: dict, analyzer: Optional[ReachabilityAnalyzer] = None):
        self._cfg = config
        self._fixtures: Dict[str, dict] = config.get("fixtures", {})
        self._analyzer = analyzer
        # NOTE: the reachability grid is built in base_link (chains root at
        # base_link, no root transform) — points are queried directly. A former
        # base_link_to_ugv_body_xyz offset knob was removed: it was zero in
        # every config and its docstring wrongly claimed a UGV_body grid frame.

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def fixture_names(self) -> List[str]:
        return list(self._fixtures.keys())

    def spec(self, name: str) -> dict:
        if name not in self._fixtures:
            raise KeyError(f"Unknown fixture '{name}'")
        return self._fixtures[name]

    def preferred_arm(self, name: str) -> str:
        return self.spec(name).get("preferred_arm", "dual")

    def approach_poses(self, name: str) -> List[dict]:
        """Approach poses as plain dicts: position + quaternion per approach vector."""
        spec = self.spec(name)
        pos = [float(x) for x in spec.get("position_base_link", [0.0, 0.0, 0.0])]
        poses = []
        for vec in spec.get("approach_vectors", []):
            q = approach_vector_to_quaternion(vec)
            poses.append({"position": pos, "orientation_xyzw": [float(x) for x in q]})
        return poses

    # ------------------------------------------------------------------
    # Collision primitives (plain dicts; JSON / web / external-bridge friendly)
    # ------------------------------------------------------------------

    def collision_primitive(self, name: str, pad_clearance: bool = False) -> dict:
        """Return a plain-dict collision primitive for the fixture.

        ``{"id", "type": box|cylinder|plane, "dimensions", "position", "frame"}``.
        Consumers (the web viewer or an external ROS bridge) build their own
        ``CollisionObject`` from this — keeping this package ROS-free.
        ``pad_clearance`` inflates dimensions by ``clearance_envelope_m``.
        """
        spec = self.spec(name)
        prim_type = spec.get("primitive", "box")
        dims = [float(d) for d in spec.get("dimensions", [])]
        pad = spec.get("clearance_envelope_m", 0.0) if pad_clearance else 0.0
        pos = [float(x) for x in spec.get("position_base_link", [0.0, 0.0, 0.0])]

        out = {"id": name, "type": prim_type, "position": pos, "frame": "base_link"}
        if prim_type == "plane":
            # Normal from the fixture's config entry (``normal_base_link``);
            # the old fallback guessed by NAME (== "wall" -> X) and gave any
            # renamed/added plane a silently wrong orientation. Vertical-wall
            # names still get the X default for back-compat, with a warning.
            declared = spec.get("normal_base_link")
            if declared is not None:
                out["normal"] = [float(v) for v in declared]
            elif name == "wall":
                out["normal"] = [1.0, 0.0, 0.0]
            else:
                logger.warning(
                    "plane fixture '%s' declares no normal_base_link — "
                    "assuming horizontal [0,0,1]; declare it in the config",
                    name)
                out["normal"] = [0.0, 0.0, 1.0]
            out["dimensions"] = dims
        elif prim_type == "cylinder":
            out["dimensions"] = [dims[0] + pad, dims[1] + 2 * pad]  # radius, height
        else:  # box
            out["dimensions"] = [d + 2 * pad for d in dims[:3]]
        return out

    def all_collision_primitives(self, pad_clearance: bool = False) -> List[dict]:
        return [self.collision_primitive(n, pad_clearance) for n in self.fixture_names()]

    # ------------------------------------------------------------------
    # Clearance / reachability validation
    # ------------------------------------------------------------------

    def within_clearance(self, name: str, point_base_link: Sequence[float]) -> bool:
        """True if *point* respects (is outside) the fixture clearance envelope.

        Uses an axis-aligned bounding shell around the primitive. Conservative
        for cylinders/planes (treats them as their bounding box).
        """
        spec = self.spec(name)
        pos = np.array(spec.get("position_base_link", [0.0, 0.0, 0.0]), dtype=float)
        pad = spec.get("clearance_envelope_m", 0.0)
        prim_type = spec.get("primitive", "box")
        dims = spec.get("dimensions", [])

        if prim_type == "box":
            half = np.array(dims[:3], dtype=float) / 2.0 + pad
        elif prim_type == "cylinder":
            r, h = float(dims[0]) + pad, float(dims[1]) / 2.0 + pad
            half = np.array([r, r, h])
        else:  # plane: only the normal direction matters
            half = np.array([pad, pad, pad])

        delta = np.abs(np.array(point_base_link, dtype=float) - pos)
        # Inside the inflated box => violates clearance.
        return bool(np.any(delta > half))

    def validate_waypoints(
        self,
        name: str,
        waypoints_base_link: Sequence[Sequence[float]],
        arm: Optional[str] = None,
    ) -> WaypointReport:
        """Check every cleaning waypoint of a fixture lies in reachable space.

        ``waypoints_base_link`` are Nx3 positions in ``base_link``. The arm
        defaults to the fixture's preferred arm (``dual`` -> reachable by either).
        """
        if self._analyzer is None:
            raise RuntimeError("validate_waypoints requires a ReachabilityAnalyzer")

        arm = arm or self.preferred_arm(name)
        pts = np.array(waypoints_base_link, dtype=float)

        unreachable: List[int] = []
        for i, p in enumerate(pts):
            if arm == "dual":
                ok = self._analyzer.is_reachable(p, "right") or self._analyzer.is_reachable(p, "left")
            else:
                ok = self._analyzer.is_reachable(p, arm)
            if not ok:
                unreachable.append(i)

        report = WaypointReport(
            fixture=name,
            arm=arm,
            total=len(pts),
            reachable=len(pts) - len(unreachable),
            unreachable_indices=unreachable,
        )
        if report.all_reachable:
            logger.info(
                "Fixture '%s' (%s arm): all %d waypoints reachable",
                name, arm, report.total,
            )
        else:
            logger.warning(
                "Fixture '%s' (%s arm): %d/%d waypoints UNREACHABLE (indices %s)",
                name, arm, len(unreachable), report.total, unreachable,
            )
        return report
