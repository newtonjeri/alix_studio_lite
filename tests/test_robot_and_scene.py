"""The robot model, its meshes and the commode C collision world load from vendor/ alone."""
from pathlib import Path

import numpy as np
import pytest

from alix_studio_lite import vendor_paths as vp


def test_forward_kinematics_runs():
    from alix_workspace.urdf_kinematics import UrdfModel
    model = UrdfModel.from_file(str(vp.URDF))
    tf = model.link_world_transforms({})
    for link in ("L_Link7", "R_Link7", "base_link"):
        assert link in tf, link
        assert np.isfinite(np.asarray(tf[link])).all()


@pytest.mark.slow
def test_every_robot_mesh_resolves_inside_lite():
    from alix_workspace.collision import RobotCollisionModel
    for tag in ("collision", "visual"):
        rcm = RobotCollisionModel.from_file(str(vp.URDF), tag=tag)
        files = [g.filename for geoms in rcm.links.values() for g in geoms if g.filename]
        assert files, tag
        for uri in files:
            p = rcm.index.resolve_uri(uri) if uri.startswith("package://") else Path(uri)
            assert p is not None and Path(p).is_file(), f"{tag}: {uri}"
            assert Path(p).resolve().is_relative_to(vp.LITE_ROOT), f"{tag}: {uri} -> {p}"


@pytest.mark.slow
@pytest.mark.parametrize("width_mm", [1400, 800])
def test_commode_c_world_builds_with_cubicle(width_mm):
    from alix_taskgen.body_collision import BodyCollisionWorld
    world = BodyCollisionWorld(scene_yaml=vp.cubicle_yaml(width_mm))
    world._ensure()
    assert world.walls is not None
    assert world.initial, "initial joint positions must load (ALIX swallows this failure silently)"
    assert world.acm, "SRDF disabled-collision pairs must load"
    for state in ("closed", "lid_open", "lid_seat_open"):
        mgr = world.fixture_manager("commode_c", state)
        names = set(mgr._objs) if hasattr(mgr, "_objs") else set()
        assert {"commode_body", "commode_lid", "commode_seat"} <= names, (state, names)
