"""Where the task-layer data lives, resolved once.

The generator reads fixture scenes, meshes and hand-authored task configs that
belong to the **task package**, not to itself. Upstream those sat at
``<repo>/cleaning_tasks/`` and the code spelled that out at a dozen call sites
as ``repo_root / "cleaning_tasks" / ...``.

In the alix tree the task package is ``ros/orchestration/alix_tasks``, and
every one of those literals resolved to a directory that does not exist. The
failure mode was not an error — it was empty results: no fixtures, no regions,
no meshes, reported as if the data were missing rather than the path wrong.
That cost ~30 confusing test failures to diagnose, so the mapping now lives in
exactly one place.

``ALIX_TASKS_PKG`` overrides, for tests and for reading an installed ROS share
directory (``$AMENT_PREFIX_PATH/share/alix_tasks``).
"""

from __future__ import annotations

import os
from pathlib import Path

#: Package name in this tree, and the name the URIs now use.
PKG_NAME = "alix_tasks"

#: What the scene YAML said before the rename.
#:
#: The original decision here was to leave ``package://cleaning_tasks/...`` in
#: the data and map it in code — 34 MB of task configs seemed a lot to touch to
#: change one string. **That was wrong, and only half the problem was visible
#: from here.** The same files carry ``model://cleaning_tasks/...`` URIs that
#: Gazebo resolves through ``GZ_SIM_RESOURCE_PATH`` with *no Python in the
#: loop*: Gazebo cannot consult this constant, so the six commode meshes the
#: fixture spawner loads would simply not have been found. The URIs are now
#: renamed in the data; this name is still accepted so the module keeps working
#: against an upstream butterfli checkout, which is what its own docstring
#: promises.
LEGACY_PKG_NAME = "cleaning_tasks"

#: Both spellings a ``package://`` URI may use, current first.
PKG_URI_NAMES = (PKG_NAME, LEGACY_PKG_NAME)

#: Its home in this tree.
TASKS_PKG_RELPATH = ("ros", "orchestration", PKG_NAME)


def repo_root(start: Path | None = None) -> Path:
    """The alix repo root, found by marker rather than by counting ``..``."""
    here = (start or Path(__file__)).resolve()
    for parent in [here, *here.parents]:
        if (parent / "ARCHITECTURE.md").is_file() and (parent / "core").is_dir():
            return parent
    return here.parents[3]


def tasks_pkg_root(hint: Path | str | None = None) -> Path:
    """Root of the task package — the old ``<repo>/cleaning_tasks``.

    ``hint`` is the caller's notion of the repo root, kept so existing call
    sites can pass what they already have. It is used only as a fallback: if a
    directory named ``cleaning_tasks`` really does sit under it (the upstream
    layout), that wins, so this module stays usable against a butterfli
    checkout.
    """
    override = os.environ.get("ALIX_TASKS_PKG")
    if override:
        return Path(override)

    if hint is not None:
        for name in PKG_URI_NAMES:
            beside = Path(hint) / name
            if beside.is_dir():
                return beside
        # A hint shaped like THIS repo (ros/orchestration/alix_tasks under it)
        # must resolve inside the hint. It used to fall through to the global
        # repo_root(), so a mirrored tree in tmp silently read the LIVE
        # tree's configs — the F74 leakage class (a "pristine" fixture test
        # inheriting live start poses, and once, writing into the real
        # scene.yaml). F75.1: honour the repo layout before falling back.
        repo_shaped = Path(hint).joinpath(*TASKS_PKG_RELPATH)
        if repo_shaped.is_dir():
            return repo_shaped

    return repo_root().joinpath(*TASKS_PKG_RELPATH)


def strip_package_uri(uri: str) -> str | None:
    """The path part of a task-package ``package://`` URI, or None.

    Accepts either spelling (``alix_tasks`` or the upstream ``cleaning_tasks``)
    so a scene file from a butterfli checkout still resolves. One function
    because there were two prefix tests — here and in ``scene_pose`` — and a
    rename that updated one of them left the other silently returning ``None``,
    which surfaces as an empty mesh manifest rather than an error.
    """
    for name in PKG_URI_NAMES:
        prefix = f"package://{name}/"
        if uri.startswith(prefix):
            return uri[len(prefix):]
    return None


def resolve_package_uri(uri: str, hint: Path | str | None = None) -> Path | None:
    """Map ``package://<task-package>/<rel>`` to a real path, or None."""
    rel = strip_package_uri(uri)
    return None if rel is None else tasks_pkg_root(hint) / rel


def plan_roots() -> list[Path]:
    """Where committed plan JSONs live, most-specific first.

    ONE definition, because there were three literals in `server.py` and **two
    of them pointed at nothing**:

    * ``core/python/alix_core/plans`` — correct until the src/ layout move put
      `alix_core` under `core/python/src/`. The literal was not updated.
    * ``Resource/plans`` — empty when this comment was first written; it has
      since been ported and is now the AUTHORITATIVE saved-plan source, laid
      out ``<fixture>/<plan_set>/{keyword}_{arm}_{mode}.json``.

    `alix_core/plans` still holds the four plan JSONs `segment_verifier`
    treats as THE contract (PORT_PLAN.md A.3).

    Derived from `repo_root()` rather than written out again, so the next
    layout change moves them together. ``ALIX_PLANS_DIR`` overrides.
    """
    override = os.environ.get("ALIX_PLANS_DIR")
    if override:
        return [Path(override)]
    root = repo_root()
    return [
        root / "Resource" / "plans",         # authoritative saved plans
        root / "core" / "python" / "src" / "alix_core" / "plans",
        tasks_pkg_root() / "plans",          # legacy: the task node saved here until
                                             # 2026-08-24; moved to Resource/plans. Kept
                                             # so an ALIX_TASKS_PKG override still resolves
    ]


def output_root() -> Path:
    """Where generated task configs are written.

    ONE definition, because there were two: `server.py` resolved the project
    directory while `scripts/generate_cleaning_config.py` resolved its own,
    and after the src/ layout move they pointed at different places — the CLI
    wrote to `src/auto-generated_config` while everything else read
    `<project>/auto-generated_config`. Nothing errored; the file simply was
    not where the reader looked.

    That is the same failure shape as F32 (three hard-coded homes for task
    data). Generated output is a project artifact, never package data, so it
    must not live under `src/`.

    ``ALIX_TASKGEN_OUTPUT_ROOT`` overrides.
    """
    override = os.environ.get("ALIX_TASKGEN_OUTPUT_ROOT")
    if override:
        return Path(override)
    # src/alix_taskgen/paths.py -> src/alix_taskgen -> src -> <project>
    return Path(__file__).resolve().parents[2]
