"""Lite runs from this repository alone: vendored ALIX code imports from vendor/, and the
vendored files are exactly the ones recorded in vendor/MANIFEST.json."""
import hashlib
import importlib
import json
import os
from pathlib import Path

import pytest

from alix_studio_lite import vendor_paths as vp

VENDORED_MODULES = [
    "alix_workspace.urdf_kinematics", "alix_workspace.collision", "alix_workspace.package_index",
    "alix_workspace.srdf_states", "alix_workspace.plan_scene_validation",
    "alix_workspace.dual_arm_plan_collision", "alix_workspace.ik", "alix_workspace.srs_ik",
    "alix_workspace.srs", "alix_workspace.redundancy",
    "alix_taskgen.body_collision", "alix_taskgen.scene_pose", "alix_taskgen.singularity",
    "alix_taskgen.glove_frames", "alix_taskgen.approach_transit",
    "alix_dispatch.track", "alix_dispatch.connect", "alix_dispatch.runmode", "alix_dispatch.ladder",
    "alix_dispatch.peripherals", "alix_dispatch.recording", "alix_dispatch.emulated_program",
    "alix_emulator.rm_emulator",
    "taught_tracks", "plan_min_time", "dual_arm_verdicts", "cleaning_tracks", "local_speed",
    "emit_toppra_rung", "predict_track_replay", "teachlab.kinematics", "teachlab.teachfile",
    "cut_dossier", "Robotic_Arm.rm_robot_interface",
]


@pytest.mark.parametrize("name", VENDORED_MODULES)
def test_vendored_module_imports(name):
    importlib.import_module(name)


def test_no_alix_module_comes_from_outside_lite():
    for name in VENDORED_MODULES:
        importlib.import_module(name)
    assert vp.assert_independent() == []


def test_environment_points_inside_lite():
    for var in ("ALIX_REPO_ROOT", "ALIX_SRDF", "ALIX_TASKS_PKG"):
        assert Path(os.environ[var]).resolve().is_relative_to(vp.LITE_ROOT), var
    assert vp.URDF.is_file() and vp.SRDF.is_file() and vp.COMMODE_C_SCENE.is_file()
    assert vp.RUNNER_V14.is_file() and vp.EXECUTION_MODEL.is_file()


def test_workspace_package_does_not_load_reachability():
    """Importing the package no longer pulls in the X2 Workspace-analysis module (vendor/PATCHES.md).
    Checked in a fresh interpreter, because other tests in this session may import it."""
    import subprocess
    import sys
    code = ("import sys; sys.path.insert(0, %r); from alix_studio_lite import vendor_paths as vp; vp.activate(); "
            "import alix_workspace.package_index, alix_workspace.urdf_kinematics; "
            "print('alix_workspace.reachability' in sys.modules)") % str(vp.LITE_ROOT / "src")
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip().splitlines()[-1] == "False"


def test_manifest_matches_files():
    manifest = json.loads((vp.VENDOR / "MANIFEST.json").read_text(encoding="utf-8"))
    assert manifest["missing"] == []
    assert manifest["alix_git"]["commit"]
    for entry in manifest["files"]:
        p = vp.LITE_ROOT / entry["dest"]
        assert p.is_file(), entry["dest"]
        digest = hashlib.sha256(p.read_bytes()).hexdigest()
        assert digest == entry.get("sha256_after_patch", entry["sha256"]), entry["dest"]
