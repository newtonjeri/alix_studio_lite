#!/usr/bin/env python3
"""Launch ALIX Studio Lite, the same way ALIX Studio is launched, on Linux, Windows or macOS.

    python3 tools/run_lite.py                 # opens a browser tab at http://127.0.0.1:8766/
    python3 tools/run_lite.py --no-browser
    python3 tools/run_lite.py --port 8800
    python3 tools/run_lite.py --fixture commode_b      # commode_a | commode_b | commode_c | urinal_a | washtub_a

Install once with ``python3 tools/setup_lite.py`` (Windows: ``py tools\\setup_lite.py``). This
launcher then runs inside that environment (``.venv`` in this folder) by itself: no activation.

Lite uses port 8766, so it can run beside ALIX Studio (8765). Project data lives in
``~/ALIX-Lite`` (``--data``), outside this folder, one project per fixture (commode C by default).
Until the robot I/O backend is built (docs/STATUS.md), the robot is a stand-in: every robot screen
works and nothing moves.
"""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VENV = ROOT / ".venv"


def venv_python() -> Path | None:
    p = VENV / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    return p if p.is_file() else None


def in_venv() -> bool:
    try:
        return Path(sys.prefix).resolve() == VENV.resolve()
    except OSError:
        return False


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if any(a in ("-h", "--help") for a in argv):
        print(__doc__)
        return 0
    vpy = venv_python()
    if vpy and not in_venv() and not os.environ.get("ALIX_LITE_NO_VENV"):
        # run inside Lite's own environment, with PYTHONPATH cleared (a sourced ROS workspace
        # would otherwise put its own numpy ahead of the venv's)
        env = dict(os.environ)
        env.pop("PYTHONPATH", None)
        cmd = [str(vpy), str(Path(__file__).resolve()), *argv]
        if os.name == "nt":                      # Windows has no exec that keeps the console
            return subprocess.call(cmd, env=env)
        os.execve(str(vpy), cmd, env)
    if vpy is None:
        print("note: Lite's environment is not installed yet. Run once:  python3 tools/setup_lite.py\n"
              "      (starting with this Python for now)", flush=True)
    sys.path.insert(0, str(ROOT / "src"))
    os.chdir(tempfile.gettempdir())       # as run_studio.py does: never import from the source tree's cwd
    port = 8766
    for i, a in enumerate(argv):
        if a in ("-p", "--port") and i + 1 < len(argv):
            port = int(argv[i + 1])
    print(f"ALIX Studio Lite  ->  http://127.0.0.1:{port}/", flush=True)
    print("Robot: stand-in (no hardware) -- the robot screens work, nothing moves.", flush=True)
    if "--no-browser" not in argv:
        print("Opening a browser tab...", flush=True)
    from alix_studio_lite import server

    return server.main(argv) or 0


if __name__ == "__main__":
    sys.exit(main())
