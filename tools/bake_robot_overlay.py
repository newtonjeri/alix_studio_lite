#!/usr/bin/env python3
"""Re-bake the robot's 3D overlay with ALIX's own tool, from the vendored URDF.

    .venv/bin/python tools/bake_robot_overlay.py          (Windows: .venv\\Scripts\\python tools\\bake_robot_overlay.py)

This is how ALIX Studio makes the overlay it shows: ``tools/meshes/bake_visual_overlay.py``
(vendored unchanged) extracts every visual mesh with its material colours, reduces it within a
1.5 mm measured deviation and keeps the original wherever reduction would tear a surface.
Lite runs it against ``vendor/alix``'s URDF and writes ``assets/robot/visual_overlay_baked.npz``,
which the 3D view serves (links missing from it fall back to their full-resolution meshes, as in
Studio). Re-run it only when the robot model changes. Needs the optional ``meshoptimizer``
package (``tools/setup_lite.py`` installs it where it can be built).
"""
from __future__ import annotations

import runpy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "assets/robot/visual_overlay_baked.npz"


def main(argv=None) -> int:
    sys.path.insert(0, str(ROOT / "src"))
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()              # package:// resolution inside vendor/alix, without a .git
    tool = vendor_paths.ALIX / "tools/meshes/bake_visual_overlay.py"
    args = ["--urdf", str(vendor_paths.URDF), "--out", str(OUT), *(argv if argv is not None else sys.argv[1:])]
    sys.argv = [str(tool), *args]
    runpy.run_path(str(tool), run_name="__main__")
    print(f"\nBaked: {OUT.relative_to(ROOT)}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
