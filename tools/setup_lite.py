#!/usr/bin/env python3
"""ONE COMMAND TO INSTALL ALIX STUDIO LITE, on Linux, Windows or macOS.

It builds a virtual environment inside this folder (``.venv``), installs the pinned packages of
``requirements.txt``, tries each optional package of ``requirements-optional.txt``, installs Lite
itself (editable, so code changes need no reinstall), and finishes with a check that proves it
works (the test suite).

    python3 tools/setup_lite.py                 # make .venv, install, check
    python3 tools/setup_lite.py --check         # only check what is there
    python3 tools/setup_lite.py --no-optional   # skip toppra and meshoptimizer
    python3 tools/setup_lite.py --no-tests      # skip the test suite at the end

On Windows, use ``py tools\\setup_lite.py``. Afterwards ``tools/run_lite.sh`` (Windows:
``py tools\\run_lite.py``) starts Lite with this environment by itself: no activation step.
An environment left unusable (an interrupted install, a Python Lite does not support) is built
again from scratch; a working one is updated in place.

NON-PYTHON REQUIREMENTS
    Python 3.12 to 3.14 with the venv module    (Ubuntu: sudo apt install python3-venv)
        (the vendored ALIX code needs 3.12; the pinned packages do not exist for 3.15 yet)
    internet access for the first install       (pip downloads the packages)
    a C/C++ compiler and git: only for the optional packages where no wheel exists
        (toppra off Linux x86-64; meshoptimizer everywhere). Lite works without them.

Like ALIX's own setup scripts, the environment is built with PYTHONPATH cleared: a sourced ROS
workspace puts its own site-packages on PYTHONPATH, ahead of the venv, and its numpy would
shadow the venv's. Lite needs no ROS.
"""
from __future__ import annotations

import argparse
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VENV = ROOT / ".venv"                                   # where tools/run_lite.py looks for it
REQ = ROOT / "requirements.txt"
OPT = ROOT / "requirements-optional.txt"
MIN_PY, MAX_PY = (3, 12), (3, 14)                       # vendored ALIX code needs 3.12; the pins have no 3.15 build
#: the toppra release Lite is tested with, as a git tag, for platforms without a wheel
TOPPRA_GIT = "toppra @ git+https://github.com/hungpham2511/toppra@0.6.10"


def say(text: str) -> None:
    print(f"\n== {text}", flush=True)


def venv_python() -> Path:
    return VENV / ("Scripts/python.exe" if os.name == "nt" else "bin/python")


def clean_env() -> dict:
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    env["PIP_DISABLE_PIP_VERSION_CHECK"] = "1"
    return env


def run(cmd: list, *, quiet: bool = False, check: bool = True, label: str | None = None) -> int:
    print("   $ " + (label or " ".join(str(c) for c in cmd)), flush=True)
    out = subprocess.DEVNULL if quiet else None
    rc = subprocess.call([str(c) for c in cmd], env=clean_env(), cwd=str(ROOT), stdout=out)
    if check and rc != 0:
        sys.exit(f"** that step failed (exit {rc}). Fix the message above and run setup again; it resumes.")
    return rc


def requirement_lines(path: Path) -> list[str]:
    return [ln.strip() for ln in path.read_text(encoding="utf-8").splitlines()
            if ln.strip() and not ln.strip().startswith("#")]


def usable(vpy: Path) -> bool:
    """The environment's Python runs, is a version Lite supports, and has pip. An install that was
    interrupted, or tried before python3-venv was installed, leaves one that is not."""
    version = f"import sys; sys.exit(not {MIN_PY} <= sys.version_info[:2] <= {MAX_PY})"
    quiet = {"env": clean_env(), "stdout": subprocess.DEVNULL, "stderr": subprocess.DEVNULL}
    try:
        return vpy.is_file() and all(subprocess.call([str(vpy), *args], **quiet) == 0
                                     for args in (["-c", version], ["-m", "pip", "--version"]))
    except OSError:
        return False


def make_venv() -> Path:
    say(f"Python and the virtual environment ({VENV})")
    print(f"   Python {platform.python_version()} at {sys.executable} ({platform.system()} {platform.machine()})")
    vpy = venv_python()
    if usable(vpy):
        print("   the environment exists; it is updated in place")
        return vpy
    if not MIN_PY <= sys.version_info[:2] <= MAX_PY:
        want = f"{MAX_PY[0]}.{MAX_PY[1]}"
        cmd = f"py -{want} tools\\setup_lite.py" if os.name == "nt" else f"python{want} tools/setup_lite.py"
        sys.exit(f"** Python {platform.python_version()} cannot build Lite's environment: Lite needs Python "
                 f"{MIN_PY[0]}.{MIN_PY[1]} to {want}.\n   Run setup with one of those, for example:  {cmd}")
    if VENV.exists():
        print("   the environment there does not work (an interrupted install, or an unsupported Python): "
              "it is built again")
    try:
        import venv as venv_mod

        # symlinks, as `python3 -m venv` makes them off Windows: a copied interpreter looks for its
        # standard library next to the Python it was made from, and finds none when that Python
        # was reached through a link in another folder (uv puts python3.14 in ~/.local/bin)
        venv_mod.EnvBuilder(with_pip=True, clear=True, symlinks=os.name != "nt", upgrade_deps=False).create(str(VENV))
    except Exception as exc:                                              # noqa: BLE001
        on_linux, v = platform.system() == "Linux", f"{sys.version_info[0]}.{sys.version_info[1]}"
        hint = f" On Ubuntu: sudo apt install python{v}-venv, then run setup again." if on_linux else ""
        sys.exit(f"** could not create the virtual environment: {exc}.{hint}")
    return vpy


def install(vpy: Path, optional: bool) -> dict:
    say("Packages (requirements.txt)")
    run([vpy, "-m", "pip", "install", "--upgrade", "pip"], quiet=True)
    run([vpy, "-m", "pip", "install", "-r", REQ])
    results = {}
    if optional:
        say("Optional packages (requirements-optional.txt)")
        for req in requirement_lines(OPT):
            name = req.split("==")[0].split(" ")[0]
            ok = run([vpy, "-m", "pip", "install", req], check=False) == 0
            if not ok and name == "toppra":
                print("   no toppra wheel for this computer: building it from its git tag (needs git and a C compiler)")
                ok = run([vpy, "-m", "pip", "install", "cython", TOPPRA_GIT], check=False) == 0
            results[name] = ok
            print(f"   {name}: {'installed' if ok else 'NOT installed (Lite works without it)'}")
    say("Lite itself (editable)")
    run([vpy, "-m", "pip", "install", "--no-deps", "-e", ROOT], quiet=True)
    return results


def check(vpy: Path, tests: bool) -> bool:
    say("Check")
    probe = ("import importlib.util as u, sys\n"
             "from alix_studio_lite import vendor_paths, __version__\n"
             "vendor_paths.activate()\n"
             "import numpy, scipy, trimesh, fcl, yaml, taught_tracks\n"
             "bad = vendor_paths.assert_independent()\n"
             "print(f'   Lite {__version__}; numpy {numpy.__version__}, scipy {scipy.__version__}, trimesh {trimesh.__version__}')\n"
             "print('   vendored ALIX code imports from this folder only' if not bad else f'** outside: {bad}')\n"
             "for m, what in (('toppra', 'Time-optimal optimiser'), ('meshoptimizer', 're-baking the robot overlay'), ('pyrealsense2', 'the wrist camera (lid and seat checks)'), ('ultralytics', 'the wrist camera colour models')):\n"
             "    print(f\"   {what}: {'available' if u.find_spec(m) else 'not installed'}\")\n"
             "sys.exit(1 if bad else 0)\n")
    ok = run([vpy, "-c", probe], check=False, label=f"{vpy.name} -c <import check>") == 0
    if ok and tests:
        t0 = time.time()
        ok = run([vpy, "-m", "pytest", "-q", "-p", "no:cacheprovider"], check=False) == 0
        print(f"   tests {'passed' if ok else 'FAILED'} in {time.time() - t0:.0f} s")
    return ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Install ALIX Studio Lite into a virtual environment.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--check", action="store_true", help="only check the existing environment")
    ap.add_argument("--no-optional", action="store_true", help="skip toppra and meshoptimizer")
    ap.add_argument("--no-tests", action="store_true", help="skip the test suite at the end")
    a = ap.parse_args(argv)
    if a.check:
        vpy = venv_python()
        if not usable(vpy):
            sys.exit(f"** Lite's environment ({VENV}) is missing or broken: run  python3 tools/setup_lite.py")
    else:
        vpy = make_venv()
        install(vpy, optional=not a.no_optional)
    ok = check(vpy, tests=not a.no_tests)
    say("Done" if ok else "Installed, but the check failed (see above)")
    start = "py tools\\run_lite.py" if os.name == "nt" else "tools/run_lite.sh"
    print(f"   Start Lite:  {start}      (it uses {VENV} by itself)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
