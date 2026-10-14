#!/usr/bin/env python3
"""Launch ALIX Studio Lite, the same way ALIX Studio is launched, on Linux, Windows or macOS.

    python3 tools/run_lite.py                 # opens the Lite UI at http://127.0.0.1:8766/
    python3 tools/run_lite.py --ui legacy     # opens Legacy Lite at http://127.0.0.1:8766/legacy/
    python3 tools/run_lite.py --no-browser
    python3 tools/run_lite.py --robot realman    # connect to the real arms and show their feedback (read only)
    python3 tools/run_lite.py --port 8800
    python3 tools/run_lite.py --fixture commode_b      # Legacy Lite: commode_a | commode_b | commode_c | urinal_a | washtub_a
    python3 tools/run_lite.py --set-pin operator       # change the operator (or engineer) PIN, then exit

Install once with ``python3 tools/setup_lite.py`` (Windows: ``py tools\\setup_lite.py``). This
launcher then runs inside that environment (``.venv`` in this folder) by itself: no activation.

Lite uses port 8766, so it can run beside ALIX Studio (8765). Data lives in ``~/ALIX-Lite``
(``--data``), outside this folder: the Lite UI's projects in ``projects/``, Legacy Lite's in
``legacy/projects/``. Until the robot I/O backend is built (docs/STATUS.md), the robot is a
stand-in: every robot screen works and nothing moves.
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
    import argparse

    peek = argparse.ArgumentParser(add_help=False)
    peek.add_argument("-p", "--port", type=int, default=8766)
    peek.add_argument("--robot", default="standin")
    peek.add_argument("--data")
    peek.add_argument("--set-pin")
    known, _ = peek.parse_known_args(argv)
    if known.data:                        # a relative --data is relative to where Lite was started
        data = str(Path(known.data).expanduser().resolve())
        argv = [x for x in argv if x != known.data and not x.startswith("--data=")]
        argv = [x for x in argv if x != "--data"] + ["--data", data]
    os.chdir(tempfile.gettempdir())       # as run_studio.py does: never import from the source tree's cwd
    if not known.set_pin:
        print(f"ALIX Studio Lite  ->  http://127.0.0.1:{known.port}/", flush=True)
        robot = known.robot
        print({"realman": "Robot: live read-only link to the real arms -- Lite reads them, commands nothing.",
               "emulated": "Robot: live read-only link to ALIX's emulated arms -- no hardware."}.get(
            robot, "Robot: stand-in (no hardware) -- the robot screens work, nothing moves."), flush=True)
    from alix_studio_lite import server

    return server.main(argv) or 0


if __name__ == "__main__":
    sys.exit(main())
