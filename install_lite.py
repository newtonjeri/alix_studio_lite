#!/usr/bin/env python3
"""GET ALIX STUDIO LITE: clone (or update) the code and install it, in one command.

    python3 install_lite.py                  # clone into ~/alix_studio_lite, install, check, show how to start
    python3 install_lite.py --dir ~/work/lite    # somewhere else (on Windows: py install_lite.py --dir D:\\lite)
    python3 install_lite.py --no-tests       # skip the test suite at the end (a minute faster)
    python3 install_lite.py --shortcut       # also add an "ALIX Studio Lite" launcher to the desktop / menu
    python3 install_lite.py --start          # start Lite when it is installed

Run it from anywhere; it uses only Python's standard library. Lite needs Python 3.12 to 3.14 (the
ALIX code it reuses needs 3.12; its packages do not exist for 3.15 yet): run with any other Python,
it looks for one of those on this computer and runs itself again with it. Inside a copy of the
code it installs that copy instead of cloning. Run it again at any time to update: it brings the
branch the copy is on up to GitHub's (never over changes or commits of yours: it says plainly when
it did not update) and updates the installation in place.

WHAT IT DOES
    1. checks Python (3.12 to 3.14, with venv) and git, and says how to get what is missing
    2. clones the repository (GitHub CLI if signed in, else git over HTTPS, else SSH), or updates
       an existing copy: a fast-forward of the branch it is on (--branch moves it to another)
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
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

REPO = "newtonjeri/alix_studio_lite"
BRANCH = "lite-ui"                                  # a new clone's branch; an existing copy keeps its own
MIN_PY, MAX_PY = (3, 12), (3, 14)                   # vendored ALIX code needs 3.12; the pins have no 3.15 build
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
                              text=True, errors="replace", timeout=60).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def q(path) -> str:
    """A path to paste into a terminal: quoted when it needs it (always, on Windows)."""
    return f'"{path}"' if SYSTEM == "Windows" else shlex.quote(str(path))


# ── 1. what this computer needs

def supported_python() -> list | None:
    """The command that starts another Python Lite supports, the newest first: on Windows the py
    launcher (py -3.14 down to py -3.12), elsewhere python3.14 down to python3.12 on PATH."""
    for minor in range(MAX_PY[1], MIN_PY[1] - 1, -1):
        cmd = ["py", f"-3.{minor}"] if SYSTEM == "Windows" else [f"python3.{minor}"]
        probe = f"import sys; sys.exit(sys.version_info[:2] != (3, {minor}))"
        try:                                        # the exit status only: no output to decode
            if subprocess.call(cmd + ["-c", probe], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, timeout=60) == 0:
                return cmd
        except (OSError, subprocess.SubprocessError):
            pass
    return None


def this_script() -> Path | None:
    """This file, unless it was piped in (python3 - < install_lite.py)."""
    try:
        f = Path(__file__)
    except NameError:
        return None
    return f.resolve() if f.is_file() else None


def check_python(argv: list) -> None:
    say(f"Python {platform.python_version()} ({sys.executable}, {SYSTEM} {platform.machine()})")
    if not MIN_PY <= sys.version_info[:2] <= MAX_PY:
        need = f"Lite needs Python {MIN_PY[0]}.{MIN_PY[1]} to {MAX_PY[0]}.{MAX_PY[1]}"
        if sys.version_info[:2] > MAX_PY:
            need += f" (its packages do not exist for Python {sys.version_info[0]}.{sys.version_info[1]} yet)"
        other, script = supported_python(), this_script()
        if other and script:
            print(f"   {need}. Found {' '.join(other)} on this computer: the installer runs again with it.", flush=True)
            sys.exit(run(other + [script, *argv], check=False))
        if other:
            fail(f"{need}. Run this script with {' '.join(other)} instead.")
        v = f"{MAX_PY[0]}.{MAX_PY[1]}"
        hint = {"Linux": "Install one, for example on Ubuntu 22.04: sudo add-apt-repository ppa:deadsnakes/ppa, "
                         "then sudo apt install python3.12 python3.12-venv",
                "Windows": f"Install Python {v}: type  py install {v}  or use its installer from "
                           "https://www.python.org/downloads/windows/",
                "Darwin": f"Install one: brew install python@{v}, or https://www.python.org/downloads/macos/",
                }.get(SYSTEM, "Install one from https://www.python.org/downloads/")
        fail(f"{need}, and this computer has none of those.\n   {hint}\n   Then run this script again.")
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


def update(copy: Path, branch: str | None) -> str:
    """Fast-forwards the copy to GitHub: the branch it is on, or `branch` when one is given.
    Returns why it was NOT updated, or "" when it was. One fetch, so git asks for a password once."""
    current = output(["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=copy)
    branch = branch or current
    say(f"Updating the copy in {copy} ({branch})")
    if not current:
        return not_updated("git cannot read it", f"To see why, run  git status  in {q(copy)}.")
    if output(["git", "status", "--porcelain"], cwd=copy):
        return not_updated("it has changes of its own", "To update it, commit or stash them, then run this again.")
    if branch == "HEAD":
        return not_updated("it is not on a branch", f"To update it, run this again with --branch {BRANCH}.")
    if run(["git", "fetch", "--prune", "origin"], cwd=copy, check=False) != 0:
        return not_updated("could not reach GitHub (sign-in or network)",
                           "Check the internet connection and the GitHub sign-in (README.md), then run this again.")
    if not output(["git", "rev-parse", "--verify", "--quiet", f"origin/{branch}"], cwd=copy):
        return not_updated(f"GitHub has no branch {branch}",
                           f"For Lite's own branch, run this again with --branch {BRANCH}.")
    if branch != current:
        run(["git", "checkout", branch], cwd=copy)
    if run(["git", "merge", "--ff-only", f"origin/{branch}"], cwd=copy, check=False) == 0:
        return ""
    if output(["git", "rev-list", f"origin/{branch}..HEAD"], cwd=copy):
        return not_updated("it has commits of its own that GitHub does not have",
                           "A developer can rebase or push them.")
    return not_updated("git could not update it", "Fix the message above, then run this again.")


def not_updated(why: str, todo: str) -> str:
    print(f"   ** {why}: NOT updated, it is installed as it is. {todo}", flush=True)
    return why


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
            if shutil.which("gio"):
                run(["gio", "set", f, "metadata::trusted", "true"], check=False, quiet=True)
        print("   " + "\n   ".join(str(f) for f in made))
    elif SYSTEM == "Windows":
        desktop = windows_desktop()
        if desktop is None:
            print("   ** no Desktop folder was found, so no launcher was made. Start Lite with the command below.")
            return
        f = desktop / "ALIX Studio Lite.bat"
        # UTF-8, which chcp 65001 makes cmd.exe read (so a folder name with accents works), and the
        # environment's own Python, so no other Python on the computer is involved
        bat = ("@echo off\r\nchcp 65001 >nul\r\n"
               f'"{copy / ".venv/Scripts/python.exe"}" "{copy / "tools/run_lite.py"}"\r\n'
               "if errorlevel 1 pause\r\n")
        f.write_bytes(bat.replace("%", "%%").encode("utf-8"))
        print(f"   {f}")
    elif SYSTEM == "Darwin":
        f = Path.home() / "Desktop/ALIX Studio Lite.command"
        f.write_text(f'#!/bin/bash\ncd "{copy}" && exec "{copy / "tools/run_lite.sh"}"\n', encoding="utf-8")
        f.chmod(0o755)
        print(f"   {f}")


def windows_desktop() -> Path | None:
    """The Desktop folder Windows really uses: OneDrive often moves it out of %USERPROFILE%."""
    places = []
    try:
        import winreg

        with winreg.OpenKey(winreg.HKEY_CURRENT_USER,
                            r"Software\Microsoft\Windows\CurrentVersion\Explorer\User Shell Folders") as key:
            places.append(Path(os.path.expandvars(winreg.QueryValueEx(key, "Desktop")[0])))
    except (ImportError, OSError):
        pass
    places.append(Path.home() / "Desktop")
    return next((p for p in places if p.is_dir()), None)


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    ap = argparse.ArgumentParser(description="Clone (or update) ALIX Studio Lite and install it.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--dir", type=Path, help="where the code goes (default: this copy, else ~/alix_studio_lite)")
    ap.add_argument("--branch", help=f"the branch to use (default: {BRANCH} for a new clone; "
                                     "an existing copy stays on its own)")
    ap.add_argument("--repo", help="clone from this URL or path instead of GitHub")
    ap.add_argument("--no-tests", action="store_true", help="skip the test suite at the end")
    ap.add_argument("--no-optional", action="store_true", help="skip the optional packages (toppra, meshoptimizer)")
    ap.add_argument("--shortcut", action="store_true", help="add a launcher to the desktop / applications menu")
    ap.add_argument("--start", action="store_true", help="start Lite when installed")
    a = ap.parse_args(argv)

    check_python(argv)
    check_git()
    copy = a.dir.expanduser().resolve() if a.dir else (own_checkout() or Path.home() / "alix_studio_lite")
    stale = ""                                      # why the copy was NOT updated
    if (copy / ".git").is_dir():
        stale = update(copy, a.branch)
    elif (copy / "tools/setup_lite.py").is_file():
        say(f"Using the copy in {copy} (not a git clone, so it is not updated)")
        stale = "it is not a git clone"
    elif copy.exists() and any(copy.iterdir()):
        fail(f"{copy} exists and is not a copy of Lite. Choose another place with --dir.")
    else:
        clone(copy, a.branch or BRANCH, a.repo)
    install(copy, tests=not a.no_tests, optional=not a.no_optional)
    if a.shortcut:
        try:
            shortcut(copy)
        except Exception as exc:                    # noqa: BLE001 - never worth failing the install for
            print(f"   ** no launcher was made ({exc}). Start Lite with the command below.", flush=True)

    if SYSTEM == "Windows":                         # PowerShell cannot start a command with a quoted path
        py = "py" if shutil.which("py") else "python"
        start = f"{py} {q(copy / 'tools/run_lite.py')}"
    else:
        py, start = q(sys.executable), q(copy / "tools/run_lite.sh")
    again = f"{py} {q(copy / 'install_lite.py')}" + (f" --branch {a.branch}" if a.branch else "")
    say("ALIX Studio Lite is installed" + (f", but NOT updated: {stale}" if stale else ""))
    print(f"   Start it:      {start}")
    print("   It opens:      http://127.0.0.1:8766/   (log in with the operator PIN, 1234 until changed)")
    print(f"   Change a PIN:  {start} --set-pin operator      (or engineer)")
    print(f"   Update later:  {again}")
    print(f"   Read first:    {copy / 'README.md'}")
    if a.start:
        say("Starting Lite (Ctrl+C to stop)")
        return subprocess.call([sys.executable, str(copy / "tools/run_lite.py")])
    return 0


if __name__ == "__main__":
    sys.exit(main())
