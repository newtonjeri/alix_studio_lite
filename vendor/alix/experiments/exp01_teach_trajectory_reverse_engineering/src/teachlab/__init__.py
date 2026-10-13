"""teachlab — exp01's analysis package for RealMan drag-teach trajectories.

ROS-free. Depends on numpy, scipy, pyyaml, matplotlib (plots only) and the
pip-editable `alix_workspace` (URDF FK / Jacobian). Nothing under core/ ros/
tools/ imports this package (experiments/README.md §5.1).

Modules
    teachfile   parse {"point":[ints]} NDJSON teach files, unit inference, the
                1x/2x step-beat test that exposes the record/source rate ratio
    kinematics  RM75 FK/Jacobian adapter over alix_workspace.urdf_kinematics,
                joint limits and qdot_max from the xacro, Arm_Tip offset
    smoothing   zero-phase Butterworth / Savitzky-Golay + residual report
    jointspace  derivatives, spectra, reversal statistics, utilization
    cartesian   TCP path, speed, curvature, lateral/tangential acceleration,
                power-law fit, corner (radius, speed) pairs, angular rate
    profile     the constraint-profile YAML (schema exp01.constraint_profile/1)
    report      plots + markdown summary
    recordings  stream.csv / run.json / command.csv loaders (Phase C)
    benchmark   run-vs-run comparison metrics and the verdict table (Phase C)

Every time-based quantity carries the sample period it was computed with, and
the period's status (`assumed` until Phase C measures it — see README).
"""
from __future__ import annotations

import pathlib
import sys

__version__ = "0.1.0"

SRC_DIR = pathlib.Path(__file__).resolve().parent.parent      # exp01/src
EXP_DIR = SRC_DIR.parent                                      # exp01/


def repo_root() -> pathlib.Path:
    """The alix repo root, by marker — never by counting `..` (D8)."""
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "ARCHITECTURE.md").is_file() and (parent / "core").is_dir():
            return parent
    raise RuntimeError(f"repo root not found above {here}")


def add_production_paths() -> None:
    """Make alix_dispatch and the vendored Robotic_Arm SDK importable.

    Neither is pip-installed (tools/alix-dispatch:16-23 is the sanctioned
    recipe; this is the same four entries). Only the live tools need this —
    the offline analysis imports nothing but alix_workspace, which is an
    editable install and needs no path work.
    """
    root = repo_root()
    for rel in ("core/python/alix_dispatch/src", "core/python/src",
                "core/python/alix_workspace/src",
                "external/butterfli-realman-alix/Python"):
        p = str(root / rel)
        if p not in sys.path:
            sys.path.insert(0, p)
