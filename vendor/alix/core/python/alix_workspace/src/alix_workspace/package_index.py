"""Portable ROS-package path resolution from the *source* tree (ROS-free).

Resolves ``package://<pkg>/<rel>`` URIs by indexing ``package.xml`` files under
the repository/workspace root, rather than relying on a built
``install/<pkg>/share/<pkg>`` layout. This makes mesh + config resolution work on
any machine that has the source checked out, with or without a colcon build.

    idx = PackageIndex.discover("/path/to/alix.urdf")
    path = idx.resolve_uri("package://rm_description/meshes/link1.STL")
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional

logger = logging.getLogger("alix_workspace.package_index")

# Directories never worth descending into when scanning for packages.
_SKIP = {"build", "install", "log", ".git", "__pycache__", "node_modules",
         ".venv", "venv", ".cache", "site-packages"}


def find_repo_root(start: str) -> Optional[Path]:
    """Best-effort source root: the nearest ancestor with a ``.git`` (the repo
    holding all the description packages), else a ``src`` workspace dir."""
    p = Path(start).resolve()
    for parent in p.parents:
        if (parent / ".git").exists():
            return parent
    for parent in p.parents:
        if parent.name == "src":
            return parent
    return p.parents[2] if len(p.parents) > 2 else p.parent


def _sibling_workspace_roots(root: Optional[Path]) -> list:
    """Package trees beside the repo, from the same colcon workspace.

    `find_repo_root` stops at the nearest `.git`, which for this project is
    `alix_ws/src/alix`. Everything declared in `alix.repos` is imported by
    `vcs import src <` into `alix_ws/src/vendor/…` — a **sibling**, so a
    repo-bounded scan never sees it.

    The symptom was a warning and a silently missing object:

        package 'realsense2_description' not found in source index

    and the D435 simply absent from the Studio viewport, because the URDF
    references it via `package://realsense2_description/meshes/…`. Nothing
    failed; the camera was just not there.

    So when the repo root sits inside a `src/`, its siblings are scanned too.
    The repo is scanned FIRST and `setdefault` keeps the first hit, so a
    package we own always wins over a vendored copy of the same name.
    """
    if root is None:
        return []
    parent = root.parent
    if parent.name != "src" or not parent.is_dir():
        return []
    try:
        return sorted(
            d for d in parent.iterdir()
            if d.is_dir() and not d.is_symlink() and d != root and d.name not in _SKIP
        )
    except OSError:
        return []


@dataclass
class PackageIndex:
    name_to_dir: Dict[str, Path] = field(default_factory=dict)
    root: Optional[Path] = None

    @classmethod
    def discover(cls, start: str) -> "PackageIndex":
        root = find_repo_root(start)
        idx: Dict[str, Path] = {}
        roots = []
        if root and root.is_dir():
            cls._scan(root, idx)
            roots.append(root)
        for extra in _sibling_workspace_roots(root):
            cls._scan(extra, idx)
            roots.append(extra)
        logger.info("Indexed %d source packages under %s",
                    len(idx), ", ".join(str(r) for r in roots))
        return cls(name_to_dir=idx, root=root)

    @staticmethod
    def _scan(root: Path, idx: Dict[str, Path]) -> None:
        stack = [root]
        while stack:
            d = stack.pop()
            pkgxml = d / "package.xml"
            if pkgxml.is_file():
                name = _pkg_name(pkgxml) or d.name
                idx.setdefault(name, d)        # first (shallowest) wins
                # keep descending: some repos nest packages inside a meta dir
            try:
                children = list(d.iterdir())
            except OSError:
                continue
            for e in children:
                if e.is_dir() and not e.is_symlink() and e.name not in _SKIP:
                    stack.append(e)

    def package_dir(self, name: str) -> Optional[Path]:
        return self.name_to_dir.get(name)

    def resolve_uri(self, uri: str) -> Optional[Path]:
        if uri.startswith("package://"):
            pkg, _, rel = uri[len("package://"):].partition("/")
            base = self.package_dir(pkg)
            if base is None:
                logger.warning("package '%s' not found in source index", pkg)
                return None
            return base / rel
        p = Path(uri)
        return p if p.is_file() else None


def _pkg_name(pkgxml: Path) -> Optional[str]:
    try:
        return ET.parse(pkgxml).getroot().findtext("name")
    except Exception:
        return None
