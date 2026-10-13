#!/usr/bin/env python3
"""GET ALIX STUDIO LITE: clone (or update) the code and install it, in one command.

    python3 install_lite.py                  # clone into ~/alix_studio_lite, install, check, show how to start
    python3 install_lite.py --dir ~/work/lite    # somewhere else (on Windows: py install_lite.py --dir D:\\lite)
    python3 install_lite.py --no-tests       # skip the test suite at the end (a minute faster)
    python3 install_lite.py --shortcut       # also add an "ALIX Studio Lite" launcher to the desktop / menu
    python3 install_lite.py --start          # start Lite when it is installed

Run it from anywhere, with Python 3.11 or newer; it uses only Python's standard library. Inside a
copy of the code it installs that copy instead of cloning. Run it again at any time to update:
it pulls the latest code (never over changes you made) and updates the installation in place.

WHAT IT DOES
    1. checks Python (3.11+, with venv) and git, and says how to get what is missing
    2. clones the repository (GitHub CLI if signed in, else git over HTTPS, else SSH), or updates
       an existing copy with a fast-forward pull
    3. runs tools/setup_lite.py: a virtual environment in the copy (.venv), the pinned packages,
       Lite itself, then a check (the test suite)
    4. optionally adds a launcher, and prints how to start Lite

THE REPOSITORY IS PRIVATE
    Sign in once, by any one of: `gh auth login` (GitHub CLI, the easiest), an SSH key added to
    your GitHub account, or a personal access token typed as the password when git asks.
    Without a clone, copy this one file to the new computer and run it there.
"""
from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

REPO = "newtonjeri/alix_studio_lite"
BRANCH = "lite-ui"
MIN_PY = (3, 11)
SYSTEM = platform.system()                          # Linux, Windows, Darwin


def say(text: str) -> None:
    print(f"\n== {text}", flush=True)


def fail(text: str) -> None:
    sys.exit(f"\n** {text}")


def run(cmd: list, *, cwd: Path | None = None, check: bool = True, quiet: bool = False) -> int:
    print("   $ " + " ".join(str(c) for c in cmd), flush=True)
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)                     # a sourced ROS workspace must not leak in
    rc = subprocess.call([str(c) for c in cmd], cwd=str(cwd) if cwd else None, env=env,
                         stdout=subprocess.DEVNULL if quiet else None, stderr=subprocess.DEVNULL if quiet else None)
    if check and rc != 0:
        fail(f"that step failed (exit {rc}). Fix the message above and run this script again; it resumes.")
    return rc


def output(cmd: list, cwd: Path | None = None) -> str:
    try:
        return subprocess.run([str(c) for c in cmd], cwd=str(cwd) if cwd else None, capture_output=True,
                              text=True, timeout=60).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


# ── 1. what this computer needs

def check_python() -> None:
    say(f"Python {platform.python_version()} ({sys.executable}, {SYSTEM} {platform.machine()})")
    if sys.version_info < MIN_PY:
        hint = {"Linux": "Ubuntu 22.04: sudo apt install python3.11 python3.11-venv, then run this with python3.11",
                "Windows": "install Python 3.12 from https://www.python.org/downloads/ (tick \"Add python.exe to PATH\")",
                "Darwin": "brew install python@3.12, or https://www.python.org/downloads/"}.get(SYSTEM, "")
        fail(f"Lite needs Python {MIN_PY[0]}.{MIN_PY[1]} or newer. {hint}")
    try:
        import ensurepip  # noqa: F401
        import venv  # noqa: F401
    except ImportError:
        v = f"{sys.version_info[0]}.{sys.version_info[1]}"
        fail(f"this Python has no venv module. On Ubuntu/Debian: sudo apt install python{v}-venv")
    print("   ok: Python with venv")


def check_git() -> None:
    if shutil.which("git"):
        print(f"   ok: {output(['git', '--version'])}")
        return
    hint = {"Linux": "sudo apt install git", "Windows": "winget install --id Git.Git -e   (then open a new terminal)",
            "Darwin": "xcode-select --install"}.get(SYSTEM, "install git")
    fail(f"git is not installed: {hint}")


# ── 2. the code

def own_checkout() -> Path | None:
    """The copy this script sits in, when it is run from inside one."""
    try:
        here = Path(__file__).resolve().parent
    except NameError:                              # piped in: python3 - < install_lite.py
        return None
    return here if (here / "tools/setup_lite.py").is_file() else None


def clone(target: Path, branch: str, repo_url: str | None) -> None:
    say(f"Cloning {repo_url or REPO} ({branch}) into {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    tries = []
    if repo_url:
        tries.append(["git", "clone", "--branch", branch, repo_url, target])
    else:
        if shutil.which("gh") and subprocess.call(["gh", "auth", "status"], stdout=subprocess.DEVNULL,
                                                  stderr=subprocess.DEVNULL) == 0:
            tries.append(["gh", "repo", "clone", REPO, target, "--", "--branch", branch])
        tries.append(["git", "clone", "--branch", branch, f"https://github.com/{REPO}.git", target])
        tries.append(["git", "clone", "--branch", branch, f"git@github.com:{REPO}.git", target])
    for cmd in tries:
        if run(cmd, check=False) == 0:
            return
        if target.exists() and not any(target.iterdir()):
            target.rmdir()                          # an empty folder from the failed attempt
    fail("the repository could not be cloned. It is private: sign in first, by one of\n"
         "     gh auth login                      (GitHub CLI: https://cli.github.com)\n"
         "     an SSH key on your GitHub account  (https://docs.github.com/authentication/connecting-to-github-with-ssh)\n"
         "     a personal access token as the password when git asks\n"
         "   then run this script again.")


def update(copy: Path, branch: str) -> None:
    say(f"Updating the copy in {copy}")
    if output(["git", "status", "--porcelain"], cwd=copy):
        print("   it has changes of its own, so it is installed as it is (no pull). To update: commit or stash, then run again.")
        return
    run(["git", "fetch", "--prune", "origin"], cwd=copy, check=False)
    current = output(["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=copy)
    if current != branch:
        run(["git", "checkout", branch], cwd=copy)
    if run(["git", "pull", "--ff-only", "origin", branch], cwd=copy, check=False) != 0:
        print("   ** it could not fast-forward (the copy has commits of its own); it is installed as it is.")


# ── 3. install, 4. launch

def install(copy: Path, tests: bool, optional: bool) -> None:
    say("Installing (tools/setup_lite.py)")
    cmd = [sys.executable, copy / "tools/setup_lite.py"]
    if not tests:
        cmd.append("--no-tests")
    if not optional:
        cmd.append("--no-optional")
    run(cmd, cwd=copy)


def shortcut(copy: Path) -> None:
    say("Launcher")
    icon = copy / "assets/brand/alix-mark.svg"
    if SYSTEM == "Linux":
        entry = ("[Desktop Entry]\nType=Application\nName=ALIX Studio Lite\n"
                 "Comment=Teach, tune and run drag-taught cleaning trajectories\n"
                 f"Exec=\"{copy / 'tools/run_lite.sh'}\"\nIcon={icon}\nTerminal=true\nCategories=Development;\n")
        apps = Path.home() / ".local/share/applications"
        apps.mkdir(parents=True, exist_ok=True)
        made = [apps / "alix-studio-lite.desktop"]
        desktop = Path(output(["xdg-user-dir", "DESKTOP"]) or Path.home() / "Desktop")
        if desktop.is_dir():
            made.append(desktop / "alix-studio-lite.desktop")
        for f in made:
            f.write_text(entry, encoding="utf-8")
            f.chmod(0o755)
            run(["gio", "set", f, "metadata::trusted", "true"], check=False, quiet=True)
        print("   " + "\n   ".join(str(f) for f in made))
    elif SYSTEM == "Windows":
        desktop = Path(os.environ.get("USERPROFILE", Path.home())) / "Desktop"
        f = desktop / "ALIX Studio Lite.bat"
        f.write_text(f'@echo off\r\ncd /d "{copy}"\r\ncall "{copy / "tools/run_lite.bat"}"\r\n', encoding="utf-8")
        print(f"   {f}")
    elif SYSTEM == "Darwin":
        f = Path.home() / "Desktop/ALIX Studio Lite.command"
        f.write_text(f'#!/bin/bash\ncd "{copy}" && exec "{copy / "tools/run_lite.sh"}"\n', encoding="utf-8")
        f.chmod(0o755)
        print(f"   {f}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Clone (or update) ALIX Studio Lite and install it.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--dir", type=Path, help="where the code goes (default: this copy, else ~/alix_studio_lite)")
    ap.add_argument("--branch", default=BRANCH, help=f"the branch to use (default {BRANCH})")
    ap.add_argument("--repo", help="clone from this URL or path instead of GitHub")
    ap.add_argument("--no-tests", action="store_true", help="skip the test suite at the end")
    ap.add_argument("--no-optional", action="store_true", help="skip the optional packages (toppra, meshoptimizer)")
    ap.add_argument("--shortcut", action="store_true", help="add a launcher to the desktop / applications menu")
    ap.add_argument("--start", action="store_true", help="start Lite when installed")
    a = ap.parse_args(argv)

    check_python()
    check_git()
    copy = a.dir.expanduser().resolve() if a.dir else (own_checkout() or Path.home() / "alix_studio_lite")
    if (copy / ".git").is_dir():
        update(copy, a.branch)
    elif (copy / "tools/setup_lite.py").is_file():
        say(f"Using the copy in {copy} (not a git clone, so it is not updated)")
    elif copy.exists() and any(copy.iterdir()):
        fail(f"{copy} exists and is not a copy of Lite. Choose another place with --dir.")
    else:
        clone(copy, a.branch, a.repo)
    install(copy, tests=not a.no_tests, optional=not a.no_optional)
    if a.shortcut:
        shortcut(copy)

    start = f"py {copy / 'tools/run_lite.py'}" if SYSTEM == "Windows" else str(copy / "tools/run_lite.sh")
    say("ALIX Studio Lite is installed")
    print(f"   Start it:      {start}")
    print("   It opens:      http://127.0.0.1:8766/   (log in with the operator PIN, 1234 until changed)")
    print(f"   Change a PIN:  {start} --set-pin operator      (or engineer)")
    print(f"   Update later:  {sys.executable} {copy / 'install_lite.py'}")
    print(f"   Read first:    {copy / 'README.md'}")
    if a.start:
        say("Starting Lite (Ctrl+C to stop)")
        return subprocess.call([sys.executable, str(copy / "tools/run_lite.py")])
    return 0


if __name__ == "__main__":
    sys.exit(main())
