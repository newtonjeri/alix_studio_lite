#!/usr/bin/env python3
"""Copy everything ALIX Studio Lite needs into this folder, so Lite runs with no ALIX checkout.

ALIX Studio Lite is an independent workspace. It reuses ALIX code and data (kinematics and
collision, robot I/O, the taught-track tools with TOPP-RA, the robot model and meshes, the
commode C scene, the RealMan SDK) and the upload-channel control program. This script copies
exactly those files, in ALIX's own folder layout, into ``vendor/alix/`` so that ALIX's own
path lookups (repo root by marker, ``*_moveit_config`` SRDF search, ``package://`` meshes)
resolve inside Lite. It also copies the control program, reference documents, the regions
deck and test data.

Every copied file is listed in ``vendor/MANIFEST.json`` with its source, sha256 and the ALIX
commit it came from. Patches applied to copied files are listed in ``vendor/PATCHES.md``.

Usage:
    python3 tools/vendor_from_alix.py                    # default sources, see --help
    python3 tools/vendor_from_alix.py --alix /path/to/alix --dry-run
"""
from __future__ import annotations

import argparse
import fnmatch
import hashlib
import json
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

LITE = Path(__file__).resolve().parents[1]
DEFAULT_ALIX = Path.home() / "alix_ws" / "src" / "alix"
DEFAULT_WORKSPACE = LITE.parent

SKIP_DIRS = {"__pycache__", ".pytest_cache", ".mypy_cache"}

# ---------------------------------------------------------------------------------------------
# What to copy from ALIX (paths relative to the ALIX repo root -> same path under vendor/alix/)
# ---------------------------------------------------------------------------------------------
ALIX_TREES = [
    # (source dir, exclude globs relative to it)
    ("core/python/alix_workspace/src/alix_workspace", ["workspace_data/reach_*", "workspace_data/manip_*"]),
    ("core/python/alix_dispatch/src/alix_dispatch", []),
    ("core/python/src/alix_emulator", []),
    ("experiments/exp01_teach_trajectory_reverse_engineering/src/teachlab", []),
    ("external/butterfli-realman-alix/Python/Robotic_Arm", ["libs/win_*"]),
    ("core/python/alix_taskgen/src/alix_taskgen/web_editor", []),          # reference for the Lite UI
    ("ros/manipulation/alix_moveit_config/config/group_states", []),
]
ALIX_FILES = [
    # alix_taskgen: only the light modules Lite uses (no generator, no Studio server)
    *[f"core/python/alix_taskgen/src/alix_taskgen/{m}.py" for m in (
        "__init__", "approach_transit", "body_collision", "constants", "contract_motion",
        "fixture_parts", "glove_frames", "junction_limits", "paths", "scene_pose",
        "singularity", "transforms")],
    "core/python/alix_taskgen/src/alix_taskgen/config/glove_frames.yaml",
    "core/python/alix_taskgen/src/alix_taskgen/config/ik_frames.yaml",
    "core/python/alix_taskgen/src/alix_taskgen/config/ik_frame_pairs.yaml",
    # alix_core: only what alix_dispatch and the emulator import
    "core/python/src/alix_core/__init__.py",
    "core/python/src/alix_core/sdk/__init__.py",
    "core/python/src/alix_core/sdk/endpoints.py",
    # the RealMan error-code tables (controller codes, SDK returns), for Lite's hardware messages
    "core/python/src/alix_core/diagnostics/__init__.py",
    "core/python/src/alix_core/diagnostics/error_codes.py",
    # taught-track tools (TOPP-RA chain, gates, shim) — top-level modules in ALIX's tools/
    *[f"tools/{m}" for m in (
        "taught_tracks.py", "plan_min_time.py", "dual_arm_verdicts.py", "cleaning_tracks.py",
        "tcp_gate.py", "track_report.py", "track_report_pdf.py", "cut_dossier.py",
        "ament_index_shim.py", "requirements_taught_canfd.txt")],
    *[f"experiments/exp01_teach_trajectory_reverse_engineering/src/{m}" for m in (
        "local_speed.py", "emit_toppra_rung.py", "predict_track_replay.py")],
    "Resource/tracks/taught/EXECUTION_MODEL.json",
    # robot description data (meshes are added from the URDF below)
    "ros/manipulation/alix_moveit_config/package.xml",
    "ros/manipulation/alix_moveit_config/config/alix.srdf",
    "ros/manipulation/alix_moveit_config/config/joint_limits.yaml",
    "ros/description/rm_description/config/initial_positions.yaml",
    "ros/description/ugv_description/config/initial_positions.yaml",
    "ros/description/ugv_description/config/peripheral_joints.yaml",
    "ros/description/pole_description/config/initial_positions.yaml",
    "ros/description/inspire_left_hand_description/config/initial_positions.yaml",
    "ros/description/inspire_right_hand_description/config/initial_positions.yaml",
    "ros/hw/alix_hw/scripts/hand_units.py",
    # commode C scene and ALIX's 800 mm cubicle
    "ros/orchestration/alix_tasks/package.xml",
    "ros/orchestration/alix_tasks/config/contact_links.yaml",
    "ros/orchestration/alix_tasks/config/commode_cleaning/fixtures.yaml",
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml",
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/models/body.stl",
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/models/lid.stl",
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/models/seat.stl",
    # ALIX's own bake tool for the robot overlay (run by tools/bake_robot_overlay.py)
    "tools/meshes/bake_visual_overlay.py",
    # the fixture pool (13 Oct 2026): commodes A and B, urinal A, wash basin A
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_a/scene.yaml",
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_a/models/regions.yaml",
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_a/models/closed.stl",
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_a/models/lid_open.stl",
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_a/models/lid_seat_open.stl",
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_b/scene.yaml",
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_b/models/regions.yaml",
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_b/models/body.stl",
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_b/models/lid.stl",
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_b/models/seat.stl",
    "ros/orchestration/alix_tasks/config/urinal_cleaning/urinal_a/scene.yaml",
    "ros/orchestration/alix_tasks/config/urinal_cleaning/urinal_a/models/regions.yaml",
    "ros/orchestration/alix_tasks/config/urinal_cleaning/urinal_a/models/urinal.stl",
    "ros/orchestration/alix_tasks/config/washtub_cleaning/washtub_a/scene.yaml",
    "ros/orchestration/alix_tasks/config/washtub_cleaning/washtub_a/models/regions.yaml",
    "ros/orchestration/alix_tasks/config/washtub_cleaning/washtub_a/models/washbasin.stl",
    "ros/orchestration/alix_tasks/config/urinal_cleaning/fixtures.yaml",
    "ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/models/regions.yaml",
    "ros/orchestration/alix_tasks/config/scene/washroom_800mm_wide_scene.yaml",
    "ros/description/alix_description/models/meshes/washroom_800mm_planning_scene.stl",
    "ros/description/alix_description/models/meshes/washroom_1000mm_planning_scene.stl",
    # design tokens (ALIX Studio theme)
    "design/_tokens.css",
]
URDF_IN_TREE = "core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf"
EXTRA_PACKAGE_ROOTS = ["../vendor"]          # realsense2_description lives beside the ALIX repo

# ---------------------------------------------------------------------------------------------
# What to copy from this workspace (paths relative to the workspace root -> Lite paths)
# ---------------------------------------------------------------------------------------------
WORKSPACE_COPIES = [
    ("upload_channel/rm_upload_fast_v14_safe_transit.py", "vendor/control_app/rm_upload_fast_v14_safe_transit.py"),
    ("upload_channel/upload/rm_upload.py", "vendor/control_app/rm_upload.py"),     # upload, check, run a slot
    ("upload_channel/hand.py", "vendor/control_app/hand.py"),
    ("upload_channel/pole.py", "vendor/control_app/pole.py"),
    ("upload_channel/usage.py", "vendor/control_app/usage.py"),
    ("upload_channel/README_COMMANDS.md", "vendor/control_app/README_COMMANDS.md"),
    ("upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py", "vendor/control_app/reference/rm_trajectory_ui_v53_safe_lid_checkpoint.py"),
    ("RealMan Dual-Arm Operations UI.txt", "docs/reference/RealMan Dual-Arm Operations UI.txt"),
    ("ALIX Studio LITE Edition_prabhu_inputs.txt", "docs/reference/ALIX_Studio_Lite_requirement_inputs.txt"),
    ("Alix_Studio_Lite_Trajectory_V2.html", "docs/reference/Alix_Studio_Lite_Trajectory_V2.html"),
    ("Toilet_regions_V1.pptx", "assets/regions/Toilet_regions_V1.pptx"),
]
WORKSPACE_TREES = [
    ("Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES", "tests/data/legacy_v6"),
    ("upload_channel/zip", "tests/data/recordings/lid_top_variants"),
    ("upload_channel/plan_txt_files", "tests/data/recordings/lid_top_plans"),
]
ALIX_TEST_DATA = [
    ("experiments/exp01_teach_trajectory_reverse_engineering/data/teach_files/commode_c_taught_paths",
     "tests/data/recordings/commode_c_taught_paths"),
]

ARCH_MARKER = """# ALIX subset vendored into ALIX Studio Lite

This folder holds the parts of the ALIX repository that ALIX Studio Lite reuses, copied by
`tools/vendor_from_alix.py` in ALIX's own folder layout. This file is a marker: ALIX code finds
its repo root by looking for `ARCHITECTURE.md` next to `core/` and `ros/`, so keeping the layout
lets that code run unchanged inside Lite. See `../MANIFEST.json` for the source of every file
and `../PATCHES.md` for the few patches applied.
"""

WORKSPACE_INIT_PATCH = '''"""Butterfli ALIX — Dual-Arm Workspace Analysis Toolkit (vendored into ALIX Studio Lite).

Lite patch: the package used to import ``reachability`` (a Workspace-analysis feature that Lite
excludes, X2) every time any submodule was imported. It is now imported only on first use of
``ReachabilityAnalyzer``. See vendor/PATCHES.md.
"""

__all__ = ["ReachabilityAnalyzer"]


def __getattr__(name):
    if name == "ReachabilityAnalyzer":
        from .reachability import ReachabilityAnalyzer
        return ReachabilityAnalyzer
    raise AttributeError(name)
'''

PATCHES_MD = """# Patches applied to vendored files

| File | Change | Why |
|---|---|---|
| `alix/core/python/alix_workspace/src/alix_workspace/__init__.py` | Imports `reachability` lazily (module `__getattr__`) instead of at package import | Lite uses the kinematics and collision modules but not the Workspace-analysis features (X2); the eager import loaded them on every use of the package |

Runtime adjustments (no file changed) are made by `src/alix_studio_lite/vendor_paths.py`:
it puts the vendored packages on `sys.path`, points `ALIX_REPO_ROOT`, `ALIX_SRDF` and
`ALIX_TASKS_PKG` at `vendor/alix`, and makes `alix_workspace.package_index` find packages
inside `vendor/alix` (ALIX looks for a `.git` folder, which Lite's copy does not have).
"""


def sha256(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


class Copier:
    def __init__(self, dry_run: bool):
        self.dry_run = dry_run
        self.entries: list[dict] = []
        self.missing: list[str] = []

    def file(self, src: Path, dst: Path, origin: str) -> None:
        if not src.is_file():
            self.missing.append(str(src))
            return
        if not self.dry_run:
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
        self.entries.append({"dest": str(dst.relative_to(LITE)), "source": str(src), "origin": origin,
                             "sha256": sha256(src), "bytes": src.stat().st_size})

    def tree(self, src: Path, dst: Path, origin: str, exclude=()) -> None:
        if not src.is_dir():
            self.missing.append(str(src))
            return
        for p in sorted(src.rglob("*")):
            if not p.is_file() or any(part in SKIP_DIRS for part in p.parts):
                continue
            rel = p.relative_to(src).as_posix()
            if any(fnmatch.fnmatch(rel, g) for g in exclude):
                continue
            self.file(p, dst / rel, origin)


def package_dirs(roots: list[Path]) -> dict[str, Path]:
    """name -> directory for every package.xml under the given roots (first hit wins)."""
    out: dict[str, Path] = {}
    for root in roots:
        if not root.is_dir():
            continue
        for pkgxml in sorted(root.rglob("package.xml")):
            if any(part in {"build", "install", "log", ".git"} for part in pkgxml.parts):
                continue
            try:
                name = ET.parse(pkgxml).getroot().findtext("name") or pkgxml.parent.name
            except ET.ParseError:
                continue
            out.setdefault(name, pkgxml.parent)
    return out


def obj_material_files(mesh: Path) -> list[Path]:
    """The .mtl files an OBJ names in its `mtllib` lines, and the textures those name (map_*)."""
    if mesh.suffix.lower() != ".obj" or not mesh.is_file():
        return []
    out = []
    names = re.findall(rb"^mtllib\s+(.+?)\s*$", mesh.read_bytes(), flags=re.M) or [(mesh.stem + ".mtl").encode()]
    for n in names:
        mtl = mesh.parent / n.decode()
        if not mtl.is_file():
            continue
        out.append(mtl)
        for t in re.findall(rb"^\s*map_\w+\s+(?:-\S+\s+\S+\s+)*(\S+)\s*$", mtl.read_bytes(), flags=re.M):
            tex = mtl.parent / t.decode()
            if tex.is_file():
                out.append(tex)
    return out


def urdf_mesh_uris(urdf: Path) -> list[str]:
    root = ET.parse(urdf).getroot()
    uris = {m.get("filename") for m in root.iter("mesh") if (m.get("filename") or "").startswith("package://")}
    return sorted(uris)


def alix_commit(alix: Path) -> dict:
    def git(*args):
        try:
            return subprocess.run(["git", "-C", str(alix), *args], capture_output=True, text=True, timeout=20).stdout.strip()
        except Exception:
            return ""
    return {"commit": git("rev-parse", "HEAD"), "branch": git("rev-parse", "--abbrev-ref", "HEAD"),
            "dirty": bool(git("status", "--porcelain"))}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--alix", type=Path, default=DEFAULT_ALIX, help="ALIX repo root (default ~/alix_ws/src/alix)")
    ap.add_argument("--workspace", type=Path, default=DEFAULT_WORKSPACE, help="alix_studio_lite_ws root")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    alix, ws = args.alix.resolve(), args.workspace.resolve()
    va = LITE / "vendor" / "alix"
    c = Copier(args.dry_run)

    for rel, excl in ALIX_TREES:
        c.tree(alix / rel, va / rel, "alix", excl)
    for rel in ALIX_FILES:
        c.file(alix / rel, va / rel, "alix")

    # Robot meshes: exactly the files the URDF references, placed under their package in vendor/alix.
    urdf = alix / URDF_IN_TREE
    pkgs = package_dirs([alix / "ros"] + [(alix / r).resolve() for r in EXTRA_PACKAGE_ROOTS])
    for uri in urdf_mesh_uris(urdf):
        pkg, _, relpath = uri[len("package://"):].partition("/")
        pdir = pkgs.get(pkg)
        if pdir is None:
            c.missing.append(uri)
            continue
        try:
            dest_pkg = va / pdir.relative_to(alix)
        except ValueError:                                   # package outside the ALIX repo
            dest_pkg = va / "ros" / "description" / pkg
        c.file(pdir / "package.xml", dest_pkg / "package.xml", "alix")
        c.file(pdir / relpath, dest_pkg / relpath, "alix")
        # an OBJ's colours live in the .mtl files its `mtllib` lines name (and their textures);
        # without them the meshes load grey, in Lite's 3D view and in the overlay bake
        for extra in obj_material_files(pdir / relpath):
            c.file(extra, dest_pkg / Path(relpath).parent / extra.relative_to((pdir / relpath).parent), "alix")

    # Lite's 800 mm cubicle = ALIX's standalone 800 mm cubicle, with the mesh path made local.
    cub = LITE / "assets" / "scene" / "cubicles"
    c.file(alix / "ros/description/alix_description/models/meshes/washroom_800mm_planning_scene.stl",
           cub / "cubicle_800mm.stl", "alix")
    venue800 = (alix / "ros/orchestration/alix_tasks/config/scene/washroom_800mm_wide_scene.yaml").read_text(encoding="utf-8")
    venue800 = re.sub(r'mesh:\s*"package://[^"]+"', 'mesh: "package://alix_studio_lite/scene/cubicles/cubicle_800mm.stl"', venue800)
    header = ("# ALIX Studio Lite copy of ALIX's washroom_800mm_wide_scene.yaml (the standalone 800 mm cubicle).\n"
              "# Only the mesh URI changed: it points into Lite's asset package `alix_studio_lite`\n"
              "# (assets/package.xml). Selectable in Settings; the default is 1400 mm (decision D21).\n")
    if not args.dry_run:
        cub.mkdir(parents=True, exist_ok=True)
        (cub / "cubicle_800mm.yaml").write_text(header + venue800)

    for src_rel, dst_rel in WORKSPACE_COPIES:
        c.file(ws / src_rel, LITE / dst_rel, "workspace")
    for src_rel, dst_rel in WORKSPACE_TREES:
        c.tree(ws / src_rel, LITE / dst_rel, "workspace")
    for src_rel, dst_rel in ALIX_TEST_DATA:
        c.tree(alix / src_rel, LITE / dst_rel, "alix")

    if not args.dry_run:
        (va / "ARCHITECTURE.md").write_text(ARCH_MARKER)
        init = va / "core/python/alix_workspace/src/alix_workspace/__init__.py"
        init.write_text(WORKSPACE_INIT_PATCH)
        for e in c.entries:
            if e["dest"].endswith("alix_workspace/__init__.py"):
                e["patched"] = True
                e["sha256_after_patch"] = sha256(init)
        (LITE / "vendor" / "PATCHES.md").write_text(PATCHES_MD)
        manifest = {
            "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "alix_root": str(alix), "alix_git": alix_commit(alix), "workspace_root": str(ws),
            "files": sorted(c.entries, key=lambda e: e["dest"]), "missing": c.missing,
        }
        (LITE / "vendor" / "MANIFEST.json").write_text(json.dumps(manifest, indent=1), encoding="utf-8")

    total = sum(e["bytes"] for e in c.entries)
    print(f"{'would copy' if args.dry_run else 'copied'} {len(c.entries)} files, {total / 1e6:.1f} MB")
    by_top: dict[str, int] = {}
    for e in c.entries:
        top = "/".join(e["dest"].split("/")[:2])
        by_top[top] = by_top.get(top, 0) + e["bytes"]
    for k, v in sorted(by_top.items()):
        print(f"  {k:40s} {v / 1e6:7.2f} MB")
    if c.missing:
        print("MISSING:", *c.missing, sep="\n  ")


if __name__ == "__main__":
    main()
