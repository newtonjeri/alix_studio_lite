"""Make the vendored ALIX code run from inside ALIX Studio Lite, with no ALIX checkout.

``activate()`` must run before any vendored module is imported. It:

1. puts the vendored packages on ``sys.path`` (ALIX's own layout under ``vendor/alix``);
2. forces the environment variables ALIX code reads (``ALIX_REPO_ROOT``, ``ALIX_SRDF``,
   ``ALIX_TASKS_PKG``) to point at the vendored tree, so a machine-wide setting that points
   at a real ALIX checkout cannot leak in;
3. makes ALIX's package lookup find packages inside ``vendor/alix`` (ALIX looks for a
   ``.git`` folder, which the vendored copy does not have) and adds Lite's own asset
   package ``alix_studio_lite`` (``assets/``), used by the cubicle wall models.

Nothing in ``vendor/`` is modified at runtime. File-level patches are listed in
``vendor/PATCHES.md``.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

LITE_ROOT = Path(__file__).resolve().parents[2]
VENDOR = LITE_ROOT / "vendor"
ALIX = VENDOR / "alix"
ASSETS = LITE_ROOT / "assets"
CONTROL_APP = VENDOR / "control_app"

#: Import roots inside the vendored ALIX tree, in ALIX's own layout.
SYS_PATHS = [
    ALIX / "core/python/alix_workspace/src",
    ALIX / "core/python/alix_taskgen/src",
    ALIX / "core/python/alix_dispatch/src",
    ALIX / "core/python/alix_perception_core/src",               # the wrist camera's perception (lid and seat checks)
    ALIX / "core/python/src",                                   # alix_core, alix_emulator
    ALIX / "tools",                                             # taught_tracks, plan_min_time, ...
    ALIX / "experiments/exp01_teach_trajectory_reverse_engineering/src",  # local_speed, teachlab, ...
    ALIX / "external/butterfli-realman-alix/Python",            # RealMan SDK (Robotic_Arm)
]

URDF = ALIX / "core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf"
SRDF = ALIX / "ros/manipulation/alix_moveit_config/config/alix.srdf"
TASKS_PKG = ALIX / "ros/orchestration/alix_tasks"
COMMODE_C_SCENE = TASKS_PKG / "config/commode_cleaning/commode_c/scene.yaml"
CUBICLES = ASSETS / "scene/cubicles"
DEFAULT_CUBICLE_MM = 1400                                       # decision D21
EXECUTION_MODEL = ALIX / "Resource/tracks/taught/EXECUTION_MODEL.json"
RUNNER_V14 = CONTROL_APP / "rm_upload_fast_v14_safe_transit.py"
RM_UPLOAD = CONTROL_APP / "rm_upload.py"             # upload a track to a slot, check it, run it

_ACTIVE = False


def cubicle_yaml(width_mm: int = DEFAULT_CUBICLE_MM) -> Path:
    """Venue YAML of a single-cubicle wall model (1400 mm default, 800 mm selectable)."""
    p = CUBICLES / f"cubicle_{int(width_mm)}mm.yaml"
    if not p.is_file():
        raise FileNotFoundError(f"no cubicle model for {width_mm} mm: {p}")
    return p


def activate() -> None:
    """Idempotent. See the module docstring."""
    global _ACTIVE
    if _ACTIVE:
        return
    for p in reversed(SYS_PATHS):
        s = str(p)
        if s not in sys.path:
            sys.path.insert(0, s)
    os.environ["ALIX_REPO_ROOT"] = str(ALIX)
    os.environ["ALIX_SRDF"] = str(SRDF)
    os.environ["ALIX_TASKS_PKG"] = str(TASKS_PKG)

    import alix_workspace.package_index as package_index

    def _find_repo_root(start):          # ALIX looks for .git; Lite's copy has the marker layout instead
        return ALIX

    package_index.find_repo_root = _find_repo_root
    _original_discover = package_index.PackageIndex.discover.__func__

    def _discover(cls, start):
        idx = _original_discover(cls, start)
        idx.name_to_dir.setdefault("alix_studio_lite", ASSETS)
        return idx

    package_index.PackageIndex.discover = classmethod(_discover)

    import alix_workspace.plan_scene_validation as psv
    from ament_index_shim import discover_packages

    if psv._PKG_DIRS is None:
        psv._PKG_DIRS = dict(discover_packages(psv.REPO_ROOT))
    psv._PKG_DIRS.setdefault("alix_studio_lite", ASSETS)
    _ACTIVE = True


def assert_independent() -> list[str]:
    """Return any imported module whose file lives outside this repository (should be none
    for ALIX modules). Third-party libraries (numpy, trimesh, ...) are ignored."""
    alix_prefixes = ("alix_", "Robotic_Arm", "taught_tracks", "plan_min_time", "dual_arm_verdicts",
                     "cleaning_tracks", "local_speed", "emit_toppra_rung", "predict_track_replay",
                     "teachlab", "ament_index_shim", "tcp_gate", "track_report")
    bad = []
    for name, mod in list(sys.modules.items()):
        f = getattr(mod, "__file__", None)
        if not f or not name.startswith(alix_prefixes):
            continue
        if not Path(f).resolve().is_relative_to(LITE_ROOT):
            bad.append(f"{name}: {f}")
    return bad
