"""Cubicle wall models: 1400 mm (default, built by tools/make_cubicle.py) and 800 mm (ALIX copy)."""
import importlib.util
import struct
from pathlib import Path

import numpy as np
import pytest
import trimesh
import yaml

from alix_studio_lite import vendor_paths as vp

ALIX_MESHES = vp.ALIX / "ros/description/alix_description/models/meshes"


def _load_make_cubicle():
    spec = importlib.util.spec_from_file_location("make_cubicle", vp.LITE_ROOT / "tools/make_cubicle.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _walls(stl: Path):
    mesh = trimesh.load(str(stl), force="mesh")
    return sorted(mesh.split(only_watertight=False), key=lambda b: (round(b.bounds[0][1], 3), round(b.bounds[0][0], 3)))


def _vertex_set(stl: Path):
    b = stl.read_bytes()
    n = struct.unpack("<I", b[80:84])[0]
    pts = set()
    for i in range(n):
        off = 84 + 50 * i + 12
        for k in range(3):
            pts.add(tuple(round(c, 4) for c in struct.unpack("<3f", b[off + 12 * k: off + 12 * k + 12])))
    return pts


def test_rule_reproduces_alix_1000mm_cubicle(tmp_path):
    mc = _load_make_cubicle()
    stl, _ = mc.make(1000, tmp_path)
    alix = ALIX_MESHES / "washroom_1000mm_planning_scene.stl"
    assert _vertex_set(stl) == _vertex_set(alix)
    assert [round(w.volume, 5) for w in _walls(stl)] == [round(w.volume, 5) for w in _walls(alix)]


@pytest.mark.parametrize("width_mm", [1400, 800])
def test_cubicle_is_three_closed_walls(width_mm):
    walls = _walls(vp.CUBICLES / f"cubicle_{width_mm}mm.stl")
    assert len(walls) == 3
    assert all(w.is_watertight for w in walls)


def test_1400mm_geometry():
    walls = _walls(vp.CUBICLES / "cubicle_1400mm.stl")
    side_inner = sorted(min(abs(w.bounds[0][1]), abs(w.bounds[1][1])) for w in walls if w.extents[1] < 0.2)
    assert side_inner == pytest.approx([0.70, 0.70], abs=1e-6)
    back = [w for w in walls if w.extents[0] < 0.2][0]
    assert back.bounds[1][0] == pytest.approx(0.0, abs=1e-6)          # inner face of the back wall at x = 0
    assert all(w.bounds[1][2] == pytest.approx(2.0) for w in walls)


def test_800mm_is_alix_copy():
    assert (vp.CUBICLES / "cubicle_800mm.stl").read_bytes() == (ALIX_MESHES / "washroom_800mm_planning_scene.stl").read_bytes()


@pytest.mark.parametrize("width_mm", [1400, 800])
def test_venue_yaml_pose_and_mesh_resolve(width_mm):
    yml = vp.cubicle_yaml(width_mm)
    ws = yaml.safe_load(yml.read_text(encoding="utf-8"))["washroom_scene"]["washroom_structure"]
    assert ws["enabled"] is True
    assert ws["pose"]["position"] == [1.215, 0.0, 0.0]
    assert ws["pose"]["orientation_rpy_deg"] == [0.0, 0.0, 180.0]
    from alix_workspace.plan_scene_validation import resolve_package_uri
    assert Path(resolve_package_uri(ws["mesh"])).resolve() == (vp.CUBICLES / f"cubicle_{width_mm}mm.stl").resolve()


def test_default_cubicle_is_1400mm():
    assert vp.DEFAULT_CUBICLE_MM == 1400
    assert vp.cubicle_yaml().name == "cubicle_1400mm.yaml"


def test_1400mm_walls_in_robot_frame():
    """Loaded through ALIX's own wall loader: inner side faces at |y| = 0.70 m and the back wall's
    inner face 1.215 m in front of base_link."""
    from alix_workspace.plan_scene_validation import load_wall_manager
    mgr, mesh_path = load_wall_manager(vp.cubicle_yaml(1400))
    assert Path(mesh_path).resolve().is_relative_to(vp.LITE_ROOT)
    # FCL mesh-vs-mesh reports surface intersections, so each probe either straddles an inner
    # face (hit) or stays clear of it in the open cubicle (no hit).
    probe = trimesh.creation.box(extents=[0.02, 0.02, 0.02])

    def hits(x, y, z=1.0):
        T = np.eye(4)
        T[:3, 3] = [x, y, z]
        return mgr.in_collision_single(probe, transform=T)

    assert hits(0.5, 0.70) and not hits(0.5, 0.66)       # side wall inner face at y = +0.70
    assert hits(0.5, -0.70) and not hits(0.5, -0.66)     # side wall inner face at y = -0.70
    assert hits(1.215, 0.0) and not hits(1.19, 0.0)      # back wall inner face at x = 1.215
