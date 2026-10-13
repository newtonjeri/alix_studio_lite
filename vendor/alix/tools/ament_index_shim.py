"""Synthesise a throwaway ament index so xacro can expand without colcon.

`$(find <pkg>)` in xacro resolves through `ament_index_python`, which looks for
an *installed* package: a marker file at
`<prefix>/share/ament_index/resource_index/packages/<pkg>` and the package
content at `<prefix>/share/<pkg>`. Source trees on `ROS_PACKAGE_PATH` are not
enough — that is a ROS 1 mechanism.

Building the workspace to satisfy that would make the kinematic gate depend on
colcon, a compiler and a populated install tree. A gate that expensive does not
run, and a gate that does not run proves nothing. This shim gives xacro exactly
what it needs — marker files and symlinks — in a temp directory, in
milliseconds.

Symlinks rather than copies on purpose: the gate must expand *the sources in
the tree*, not a stale copy of them.
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path


def build_index(package_dirs: dict[str, Path], root: Path | None = None) -> Path:
    """Create an ament index exposing `package_dirs`; return its prefix path."""
    prefix = Path(root or tempfile.mkdtemp(prefix="alix_ament_"))
    markers = prefix / "share" / "ament_index" / "resource_index" / "packages"
    markers.mkdir(parents=True, exist_ok=True)
    share = prefix / "share"
    for name, path in package_dirs.items():
        path = Path(path).resolve()
        if not path.is_dir():
            continue
        (markers / name).write_text("")
        link = share / name
        if link.is_symlink() or link.exists():
            link.unlink()
        link.symlink_to(path)
    return prefix


def discover_packages(repo_root: Path) -> dict[str, Path]:
    """Every package.xml-bearing directory in the tree, by package name."""
    out: dict[str, Path] = {}
    for manifest in sorted(Path(repo_root).rglob("package.xml")):
        if any(p in manifest.parts for p in ("build", "install", "log", ".git")):
            continue
        name = manifest.parent.name
        # <name> from the manifest wins over the directory name when they differ
        try:
            text = manifest.read_text(errors="replace")
            start = text.index("<name>") + 6
            name = text[start:text.index("</name>", start)].strip() or name
        except ValueError:
            pass
        out.setdefault(name, manifest.parent)
    return out


def env_with_index(prefix: Path, base_env: dict | None = None) -> dict:
    """A copy of the environment with `prefix` prepended to AMENT_PREFIX_PATH."""
    env = dict(base_env or os.environ)
    existing = env.get("AMENT_PREFIX_PATH", "")
    env["AMENT_PREFIX_PATH"] = f"{prefix}:{existing}" if existing else str(prefix)
    return env


if __name__ == "__main__":  # pragma: no cover - manual use
    import sys

    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    pkgs = discover_packages(root)
    prefix = build_index(pkgs)
    print(f"index at {prefix}")
    for n in sorted(pkgs):
        print(f"  {n:<40} {pkgs[n]}")
