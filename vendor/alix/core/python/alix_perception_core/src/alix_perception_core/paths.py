"""Where this package's inputs live, resolved without ROS.

Same problem `alix_orchestrator/core/_paths.py` solves, and deliberately the same
shape: anchor on the repo marker (`ARCHITECTURE.md` beside a `ros/` directory)
rather than counting `parents[N]`, because the count is what silently broke on
the butterfli -> alix move. A wrong count resolves to a directory that does not
exist, and every caller reads "not found" as "nothing configured".

`ALIX_TASKS_PKG` and `ALIX_MOVEIT_CONFIG` override, matching
`alix_taskgen.paths` and `alix_orchestrator.core._paths` so a redirected tree
redirects all three at once ([[exp02 mock stack + plan redirect]] is the reason
those overrides exist).

For `package://` URIs in a fixture's `scene.yaml`, prefer
`alix_workspace.package_index.PackageIndex` — it already indexes every
`package.xml` in the workspace including vendored siblings. This module only
answers the two questions that have a fixed home in the tree.
"""

from __future__ import annotations

import os
from pathlib import Path

#: Upstream's flat layout, still accepted so the core is testable in a
#: butterfli checkout without a second code path.
_LEGACY_SIBLING = "cleaning_tasks"


def repo_root() -> Path | None:
    """The alix source root, identified by its own marker files."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "ARCHITECTURE.md").is_file() and (parent / "ros").is_dir():
            return parent
    return None


def task_config_root() -> Path | None:
    """``alix_tasks/config``, or None if it cannot be found.

    None rather than a raise: a caller that cannot find the fixture model must
    be able to say so in a service response, not take the node down at import.
    """
    override = os.environ.get("ALIX_TASKS_PKG")
    if override:
        candidate = Path(override) / "config"
        return candidate if candidate.is_dir() else None

    root = repo_root()
    if root is not None:
        candidate = root / "ros" / "orchestration" / "alix_tasks" / "config"
        return candidate if candidate.is_dir() else None

    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / _LEGACY_SIBLING / "config"
        if candidate.is_dir():
            return candidate
    return None


def fixture_model_dir(fixture_type: str, model: str) -> Path | None:
    """``<config>/<fixture_type>_cleaning/<model>``.

    e.g. ``fixture_model_dir("commode", "commode_c")`` ->
    ``.../config/commode_cleaning/commode_c``.
    """
    root = task_config_root()
    if root is None:
        return None
    candidate = root / f"{fixture_type}_cleaning" / model
    return candidate if candidate.is_dir() else None


def models_manifest() -> Path | None:
    """The local-inference model manifest.

    ``ALIX_PERCEPTION_MODELS`` points at a specific ``models.yaml``. Otherwise
    the packaged default is used, which declares no models — see
    `backends.local` for why that is a refusal rather than a fallback.
    """
    override = os.environ.get("ALIX_PERCEPTION_MODELS")
    if override:
        candidate = Path(override)
        return candidate if candidate.is_file() else None
    packaged = Path(__file__).resolve().parent / "config" / "models.yaml"
    return packaged if packaged.is_file() else None
