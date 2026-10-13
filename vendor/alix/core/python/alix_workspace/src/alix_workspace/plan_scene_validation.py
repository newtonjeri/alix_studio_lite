"""
plan_scene_validation.py
========================
Validate saved plan JSONs against STATIC scene geometry — the washroom walls
(``washroom_structure`` from the venue scene yaml). Any robot-link↔wall contact
is a genuine violation: wall pairs are never ACM-allowed by any task.

Why: OMPL validates motion at a discrete resolution (unconfigured here ⇒
0.5–1 % of the joint-space extent ⇒ the arm can sweep 8–20 cm between
consecutive collision checks), so sampling-based plans can tunnel through thin
wall panels non-deterministically. This sweep is the deterministic gate.

Used two ways:
  • the generate_plans.py IN-PIPELINE GATE — every freshly planned task is
    swept before it can be accepted (no separate validation run needed), and
  • the standalone triage CLI: alix_workspace/scripts/check_plan_wall_collisions.py.

No ROS required — pure URDF FK + trimesh/FCL, the same stack as the studio's
dual-arm collision checker.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

def _repo_root() -> Path:
    """Repo root by MARKER, never by counting `..` (decision D8).

    Both defaults below used to be built from `parents[2]`, which resolves to
    `core/python/alix_workspace` — the package directory, not the repo. So
    `DEFAULT_URDF` and `DEFAULT_SCENE` both pointed at paths that do not
    exist, and `DEFAULT_SCENE` additionally still named `cleaning_tasks`, a
    package renamed to `alix_tasks` by P13. This module is the in-pipeline
    wall-collision gate; a gate whose defaults dangle is not a gate.
    (ROADMAP §4 item 3; fixed 2026-08-17.)

    `ALIX_REPO_ROOT` overrides, matching `alix_tasks/src/common/task_base.cpp`.
    """
    import os

    env = os.environ.get("ALIX_REPO_ROOT")
    if env:
        return Path(env)
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "ARCHITECTURE.md").is_file() and (parent / "ros").is_dir():
            return parent
    raise RuntimeError(
        "repo root not found from %s — expected an ancestor holding both "
        "ARCHITECTURE.md and ros/, or ALIX_REPO_ROOT set." % here
    )


REPO_ROOT = _repo_root()

#: The URDF ships INSIDE this package, so it is module-relative — not
#: repo-relative. Keep it that way: it must resolve from an installed wheel.
DEFAULT_URDF = Path(__file__).resolve().parent / "urdf" / "alix.urdf"

#: The 800 mm cubicle is the standardised operating space (Newton,
#: 2026-08-17): walls 800 mm apart, and the scene the task configs load
#: through their `scene_config_file` parameter. Validate against this unless a
#: caller passes something else; the 1000/1400 scenes exist as alternatives.
DEFAULT_SCENE = (REPO_ROOT / "ros" / "orchestration" / "alix_tasks" /
                 "config" / "scene" / "washroom_800mm_wide_scene.yaml")


_PKG_DIRS: Optional[Dict[str, Path]] = None


def resolve_package_uri(uri: str) -> Path:
    """`package://<pkg>/<rel>` -> a real path, by FINDING the package.

    The previous form was `REPO_ROOT / uri.replace("package://", "")`, which
    assumes every package sits directly at the repo root. That was true of the
    old butterfli layout and is false here — packages live under
    `ros/description/`, `ros/orchestration/`, `ros/manipulation/` and so on —
    so the wall mesh resolved to a path that does not exist and this gate
    raised before checking a single plan.

    Discovery is by `package.xml`, reusing `tools/ament_index_shim.py` so the
    repo has one implementation of "where does package X live". No ROS and no
    install tree required, which is the property that lets this gate run
    in-pipeline (see the module docstring).
    """
    global _PKG_DIRS
    if not uri.startswith("package://"):
        return Path(uri)
    rest = uri[len("package://"):]
    pkg, _, rel = rest.partition("/")
    if _PKG_DIRS is None:
        import sys

        sys.path.insert(0, str(REPO_ROOT / "tools"))
        from ament_index_shim import discover_packages  # noqa: E402

        _PKG_DIRS = dict(discover_packages(REPO_ROOT))
        vendor = REPO_ROOT.parent / "vendor"
        if vendor.is_dir():
            _PKG_DIRS.update(discover_packages(vendor))
    root = _PKG_DIRS.get(pkg)
    if root is None:
        raise RuntimeError(
            "package %r not found under %s — a mesh URI names a package that "
            "does not exist in this tree (known cases: the P13 rename of "
            "`cleaning_tasks` to `alix_tasks`, and `butterfli_description`, "
            "which is not part of this workspace at all)." % (pkg, REPO_ROOT)
        )
    return Path(root) / rel


def load_wall_manager(scene_yaml: Path, mesh_override: Optional[Path] = None):
    """CollisionManager holding the washroom_structure mesh, posed per the
    venue scene yaml (position + orientation_rpy_deg in base_link).
    Returns ``(manager, mesh_path)``."""
    import trimesh
    import yaml

    sc = (yaml.safe_load(Path(scene_yaml).read_text()) or {}).get("washroom_scene") or {}
    ws = sc.get("washroom_structure") or {}
    if not ws.get("enabled", False):
        raise RuntimeError(f"washroom_structure disabled in {scene_yaml}")
    if mesh_override:
        mesh_path = Path(mesh_override)
    else:
        mesh_path = resolve_package_uri(str(ws.get("mesh", "")))
    if not mesh_path.is_file():
        raise RuntimeError(f"wall mesh not found: {mesh_path}")

    mesh = trimesh.load(str(mesh_path), force="mesh")
    scale = np.asarray(ws.get("scale", [1.0, 1.0, 1.0]), dtype=float)
    if not np.allclose(scale, 1.0):
        mesh.apply_scale(scale)

    pose = ws.get("pose") or {}
    pos = np.asarray(pose.get("position", [0, 0, 0]), dtype=float)
    rpy = np.radians(np.asarray(pose.get("orientation_rpy_deg", [0, 0, 0]), dtype=float))
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = pos

    mgr = trimesh.collision.CollisionManager()
    mgr.add_object("washroom_structure", mesh, transform=T)
    return mgr, mesh_path


def stage_bounds(plan_path: Path):
    """[(t0, t1, index, stage_name), ...] using the same concatenation the
    timeline loader applies (stages run back-to-back)."""
    data = json.loads(Path(plan_path).read_text())
    bounds, t0 = [], 0.0
    for st in data.get("sub_trajectories", []):
        if not (st.get("joint_names") and st.get("waypoints")):
            continue
        dur = max((float(w.get("time_from_start", 0.0)) for w in st["waypoints"]), default=0.0)
        bounds.append((t0, t0 + dur, st.get("index"), st.get("stage_name", "?")))
        t0 += dur
    return bounds


def stage_at(bounds, t):
    for t0, t1, idx, name in bounds:
        if t0 <= t <= t1 + 1e-9:
            return idx, name
    return None, "?"


class PlanWallChecker:
    """Sweeps one plan at a time against the walls. Heavy assets (URDF FK model,
    robot collision meshes, wall mesh) are loaded lazily on first use and
    reused across checks — cheap enough to run inline after every plan."""

    def __init__(self, urdf_path: Optional[Path] = None,
                 scene_yaml: Optional[Path] = None,
                 mesh_override: Optional[Path] = None):
        self.urdf_path = Path(urdf_path or DEFAULT_URDF)
        self.scene_yaml = Path(scene_yaml or DEFAULT_SCENE)
        self.mesh_override = mesh_override
        self._ready = False

    def _ensure(self):
        if self._ready:
            return
        from alix_workspace.collision import RobotCollisionModel
        from alix_workspace.urdf_kinematics import UrdfModel
        from alix_workspace.srdf_states import find_srdf
        from alix_workspace.dual_arm_plan_collision import _ArmCollision, _build_home
        self.model = UrdfModel.from_file(str(self.urdf_path))
        rcm = RobotCollisionModel.from_file(str(self.urdf_path), tag="collision")
        self.home = _build_home(rcm, find_srdf(str(self.urdf_path.parent)))
        self.robot = _ArmCollision(rcm, lambda link: True)   # ALL collision links
        self.wall_mgr, self.mesh_path = load_wall_manager(self.scene_yaml, self.mesh_override)
        self._ready = True

    def _check_state(self, times, rows, t):
        from alix_workspace.dual_arm_plan_collision import _sample_at
        q = dict(self.home)
        q.update(_sample_at(times, rows, t))
        self.robot.pose(self.model.link_world_transforms(q))
        hit, data = self.robot.mgr.in_collision_other(self.wall_mgr, return_data=True)
        if not hit:
            return None
        links = sorted({n.split("#")[0] for c in data for n in c.names if n != "washroom_structure"}
                       ) if data else []
        depth = max((float(c.depth) for c in data), default=0.0) if data else 0.0
        return {"t": t, "links": links, "depth": depth}

    def check(self, plan_path: Path, dt: float = 0.05, refine_dt: float = 0.01) -> List[dict]:
        """Sweep one plan. Returns summarized contact RUNS (empty list = clear):
        ``[{stage_idx, stage, links, t0, t1, depth}, ...]``. Coarse pass at
        ``dt``; each hit neighbourhood refined at ``refine_dt``."""
        from alix_workspace.dual_arm_plan_collision import load_plan_timeline
        self._ensure()
        times, rows = load_plan_timeline(str(plan_path))
        if not times:
            return []
        total = times[-1]
        bounds = stage_bounds(plan_path)

        coarse_hits = []
        n = int(total / dt) + 1
        for i in range(n + 1):
            r = self._check_state(times, rows, min(i * dt, total))
            if r:
                coarse_hits.append(r)

        windows: List[List[float]] = []
        for h in coarse_hits:
            a, b = max(0.0, h["t"] - dt), min(total, h["t"] + dt)
            if windows and a <= windows[-1][1]:
                windows[-1][1] = max(windows[-1][1], b)
            else:
                windows.append([a, b])

        contacts = []
        for a, b in windows:
            t = a
            while t <= b + 1e-9:
                r = self._check_state(times, rows, t)
                if r:
                    idx, name = stage_at(bounds, t)
                    r["stage_idx"], r["stage"] = idx, name
                    contacts.append(r)
                t += refine_dt
        return summarize_runs(contacts)


def summarize_runs(contacts: List[dict]) -> List[dict]:
    """Group refined contacts into contiguous runs per (stage, link-set)."""
    runs: List[dict] = []
    for c in contacts:
        key = (c["stage_idx"], tuple(c["links"]))
        if runs and runs[-1]["key"] == key and c["t"] - runs[-1]["t1"] < 0.1:
            runs[-1]["t1"] = c["t"]
            runs[-1]["depth"] = max(runs[-1]["depth"], c["depth"])
        else:
            runs.append({"key": key, "stage_idx": c["stage_idx"], "stage": c["stage"],
                         "links": c["links"], "t0": c["t"], "t1": c["t"], "depth": c["depth"]})
    return runs
