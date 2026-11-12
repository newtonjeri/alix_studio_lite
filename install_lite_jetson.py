#!/usr/bin/env python3
"""GET ALIX STUDIO LITE ON THE ROBOT'S JETSON: prepare the Jetson, clone (or update) the code, install it, check it.

    python3 install_lite_jetson.py                  # everything, into ~/alix_studio_lite (or the copy this file is in)
    python3 install_lite_jetson.py --dir /data/alix_studio_lite     # somewhere else (a bigger disk)
    python3 install_lite_jetson.py --check          # change nothing: say what this Jetson has and what is missing
    python3 install_lite_jetson.py --no-sudo        # change nothing system-wide: print the commands to run instead
    python3 install_lite_jetson.py --no-tests       # skip the test suite at the end
    python3 install_lite_jetson.py --cpu-torch      # PyTorch's processor-only build (see below)
    python3 install_lite_jetson.py --shortcut       # also add an "ALIX Studio Lite" launcher to the desktop / menu
    python3 install_lite_jetson.py --start          # start Lite when it is installed

install_lite.py's twin for the Jetson (NVIDIA Jetson AGX Orin, ARM64, Ubuntu 24.04 / JetPack 7; on JetPack 6's Ubuntu
22.04 Lite installs without the wrist camera, whose package needs 24.04's libraries).
Run it from anywhere with the Jetson's own python3; it uses only Python's standard library. Copied alone to a new
Jetson (a USB stick) it clones the code; inside a copy of the code it installs that copy. Run it again at any time to
update: the code is updated as install_lite.py updates it (never over changes or commits of yours) and the
installation in place.

WHAT IT DOES
    1. says which computer this is (the Jetson's model, its JetPack, its Ubuntu) and refuses any that is not ARM64
    2. finds Python 3.12 (Ubuntu 24.04's own; on JetPack 6, Ubuntu 22.04, it is installed from the deadsnakes
       archive; JetPack 5's Ubuntu 20.04 has none: there one is given with --python)
    3. installs the system packages Lite needs and this Jetson lacks, with sudo, each command shown first:
       git, Python's venv module and headers, a compiler (the Time-optimal optimiser is built here: it has no ARM64
       build), the libraries the camera and OpenCV load
    4. checks the disk, then clones the repository or updates the copy, and installs Lite into the copy's own
       environment (.venv) by running the copy's install_lite.py with that Python
    5. installs the wrist camera's access rules (udev: the camera maker's own file, checked against its checksum)
    6. checks what only this computer can say: the robot's ARM64 library loads, the camera library and a camera that
       is plugged in, the colour model (it is loaded and asked: how long a frame takes, on the GPU or the processor),
       the power mode, the network toward the arms, a screen and a browser
    7. runs the test suite (a test that fails in the full run is run again alone: the full run loads every core, and
       a 5 ms stream that is fed late ends itself), and prints how to start Lite here and from the operator PC

WHAT IS DIFFERENT ON THE JETSON
    Python 3.12 exactly (install_lite.py takes 3.12 to 3.14): the camera package (pyrealsense2) has an ARM64 build for
    Python 3.12 only. The colour model runs where PyTorch can: PyPI's PyTorch for ARM64 carries CUDA libraries made
    for ARM servers (3 GB to download, 6 GB installed), and whether they drive this Jetson's GPU depends on its
    JetPack; the report says which it is, the GPU or the processor, and what a frame takes (the model is small). On a
    Jetson where it is the processor, --cpu-torch builds the environment again with PyTorch's processor-only build
    (the environment is then 1.5 GB instead of 7). On the robot's Jetson (L4T R39.2, 10 Nov 2026) it is the GPU: the
    model loads in 4.9 s and answers a frame in 15 ms. Lite listens on the Jetson itself only (127.0.0.1): from the
    operator PC it is opened through an SSH tunnel, the log-in the connection needs.

THE REPOSITORY IS PRIVATE
    Sign in once, by any one of: `gh auth login` (GitHub CLI), an SSH key added to your GitHub account, or a personal
    access token typed as the password when git asks. Or copy the whole folder to the Jetson and run this file in it.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import shlex
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = "newtonjeri/alix_studio_lite"
BRANCH = "lite-ui"                                  # a new clone's branch; an existing copy keeps its own
#: Python 3.12 exactly: pyrealsense2 2.57.7.10387 (requirements-optional.txt, the version ALIX's camera node was built on)
#: has ARM64 wheels for Python 3.9, 3.10 and 3.12 only, and the vendored ALIX code needs 3.12.
PY = (3, 12)
PY_NAME = f"python{PY[0]}.{PY[1]}"
#: The camera maker's udev rules, of the release the camera package is (its access to the camera without root).
RULES_URL = "https://raw.githubusercontent.com/realsenseai/librealsense/v2.57.7/config/99-realsense-libusb.rules"
RULES_SHA256 = "c610c3379d360006261b0fc26614316323551f5b11ffe81e9ebf48c4c0a43ce8"
RULES_FILE = Path("/etc/udev/rules.d/99-realsense-libusb.rules")
RULES_DIRS = ("/etc/udev/rules.d", "/lib/udev/rules.d", "/usr/lib/udev/rules.d", "/usr/local/lib/udev/rules.d")
#: Disk, in GB (measured 10 Nov 2026 on an emulated ARM64 Ubuntu): a new clone (the checkout is 10 GB and its history
#: 2.7, nearly all of it the lab's recorded runs in Resources/Logs, which grow), and the environment with pip's
#: downloads beside it: 7 + 3 with PyPI's PyTorch and its CUDA libraries, 1.5 + 1 with the processor-only build
#: (--cpu-torch). Each with a little to spare.
CLONE_GB, ENV_GB, ENV_CPU_TORCH_GB = 14, 12, 4
#: PyTorch's own index of processor-only builds (--cpu-torch): torch and torchvision without the CUDA libraries.
TORCH_CPU_INDEX = "https://download.pytorch.org/whl/cpu"
#: L4T's release (the first number of /etc/nv_tegra_release) and the JetPack it belongs to.
JETPACKS = {32: "4", 34: "5", 35: "5", 36: "6", 38: "7"}
PORT = 8766


def say(text: str) -> None:
    print(f"\n== {text}", flush=True)


def note(text: str) -> None:
    print(f"   {text}", flush=True)


def fail(text: str) -> None:
    sys.exit(f"\n** {text}")


def clean_env() -> dict:
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)                     # a sourced ROS workspace must not leak in
    return env


def run(cmd: list, *, cwd=None, check: bool = True, quiet: bool = False) -> int:
    print("   $ " + " ".join(str(c) for c in cmd), flush=True)
    rc = subprocess.call([str(c) for c in cmd], cwd=str(cwd) if cwd else None, env=clean_env(),
                         stdout=subprocess.DEVNULL if quiet else None, stderr=subprocess.DEVNULL if quiet else None)
    if check and rc != 0:
        fail(f"that step failed (exit {rc}). Fix the message above and run this installer again; it resumes.")
    return rc


def output(cmd: list, cwd=None, timeout: float = 60) -> str:
    try:
        return subprocess.run([str(c) for c in cmd], cwd=str(cwd) if cwd else None, capture_output=True, text=True,
                              errors="replace", timeout=timeout, env=clean_env()).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def works(cmd: list, timeout: float = 60) -> bool:
    try:
        return subprocess.call([str(c) for c in cmd], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, timeout=timeout, env=clean_env()) == 0
    except (OSError, subprocess.SubprocessError):
        return False


def q(path) -> str:
    return shlex.quote(str(path))


# ── 1. which computer this is

def read_text(path: Path) -> str:
    try:
        return path.read_bytes().replace(b"\x00", b"").decode("utf-8", "replace").strip()
    except OSError:
        return ""


def this_computer(root: Path = Path("/"), machine: str | None = None) -> dict:
    """What this computer is, from the files that say it (``root``: the tests' own tree): the Jetson's model
    (the device tree), L4T's release and the JetPack it belongs to (/etc/nv_tegra_release), Ubuntu (/etc/os-release)."""
    info = dict(machine=machine or platform.machine(), model=read_text(root / "proc/device-tree/model"),
                l4t=None, jetpack=None, os_id=None, os_version=None, os_name=None)
    m = re.search(r"R(\d+)\s*\(release\),\s*REVISION:\s*([\d.]+)", read_text(root / "etc/nv_tegra_release"))
    if m:
        info["l4t"] = f"R{m.group(1)}.{m.group(2)}"
        info["jetpack"] = JETPACKS.get(int(m.group(1)))
    for line in read_text(root / "etc/os-release").splitlines():
        key, _, value = line.partition("=")
        value = value.strip().strip('"')
        if key == "ID":
            info["os_id"] = value
        elif key == "VERSION_ID":
            info["os_version"] = value
        elif key == "PRETTY_NAME":
            info["os_name"] = value
    info["jetson"] = bool(info["l4t"]) or "jetson" in info["model"].lower()
    return info


def describe(info: dict) -> str:
    parts = [info["model"] or ("a Jetson" if info["jetson"] else "not a Jetson")]
    if info["l4t"]:
        parts.append(f"L4T {info['l4t']}" + (f" (JetPack {info['jetpack']})" if info["jetpack"] else ""))
    parts.append(info["os_name"] or "an unnamed Linux")
    parts.append(info["machine"])
    return ", ".join(parts)


def memory_gb() -> float | None:
    m = re.search(r"MemTotal:\s*(\d+)\s*kB", read_text(Path("/proc/meminfo")))
    return int(m.group(1)) / 1024 ** 2 if m else None


def ubuntu_at_least(info: dict, major: int, minor: int = 4) -> bool:
    try:
        a, b = (int(x) for x in str(info["os_version"]).split(".")[:2])
    except (TypeError, ValueError):
        return False
    return info["os_id"] == "ubuntu" and (a, b) >= (major, minor)


def check_computer(info: dict) -> None:
    say("This computer")
    ram, cores = memory_gb(), os.cpu_count()
    note(describe(info) + (f", {cores} cores" if cores else "") + (f", {ram:.0f} GB memory" if ram else ""))
    if info["machine"] not in ("aarch64", "arm64"):
        fail(f"this installer is for the robot's Jetson (ARM64), and this computer is {info['machine']}.\n"
             "   On this computer use install_lite.py.")
    if not info["jetson"]:
        note("not a Jetson (no /etc/nv_tegra_release): installed as on any ARM64 Linux, the Jetson's own checks left out")
    if info["os_id"] == "ubuntu" and not ubuntu_at_least(info, 24):
        note(f"** {info['os_name']}: Lite installs, without the wrist camera. The camera package's ARM64 build needs "
             "Ubuntu 24.04's libraries (glibc 2.38, found 10 Nov 2026 on an emulated 22.04), so the lid and seat tasks "
             "run in Simulation and Hardware sim only here.")


# ── 2. Python 3.12

def is_wanted_python(cmd) -> bool:
    return works([cmd, "-c", f"import sys; sys.exit(sys.version_info[:2] != {PY})"])


def find_python(explicit=None):
    """A Python 3.12 on this computer: the one given (--python), the one running this, python3.12 on PATH."""
    if explicit:
        path = shutil.which(str(explicit)) or str(Path(explicit).expanduser())
        if not is_wanted_python(path):
            fail(f"{explicit} is not Python {PY[0]}.{PY[1]} (or does not run). Lite needs {PY[0]}.{PY[1]} on the Jetson: "
                 "the camera package has an ARM64 build for that version only.")
        return path
    for cand in (sys.executable if sys.version_info[:2] == PY else None, shutil.which(PY_NAME)):
        if cand and is_wanted_python(cand):
            return cand
    return None


def python_has(py, what: str) -> bool:
    """Whether that Python can build an environment (``venv``: with pip) or C extensions (``headers``: Python.h)."""
    probe = {"venv": "import ensurepip, venv",
             "headers": "import os, sys, sysconfig; "
                        "sys.exit(not os.path.isfile(os.path.join(sysconfig.get_paths()['include'], 'Python.h')))"}[what]
    return works([py, "-c", probe])


def distro_python(py) -> bool:
    """Whether that Python is the distribution's own (apt's python3.12-venv and -dev belong to it)."""
    try:
        return str(Path(py).resolve()).startswith("/usr/bin/")
    except OSError:
        return False


# ── 3. system packages

def as_root(cmd: list):
    """The command as root: as it is when this is root, through sudo otherwise; None where there is neither."""
    if os.geteuid() == 0:
        return list(cmd)
    return ["sudo"] + list(cmd) if shutil.which("sudo") else None


def has_library(name: str) -> bool:
    """Whether the dynamic linker finds that shared library (asked in a process of its own: nothing is loaded here)."""
    return works([sys.executable, "-c", "import ctypes, sys; ctypes.CDLL(sys.argv[1])", name])


def system_needs(py, optional: bool, have=None) -> list:
    """What Lite needs from the system and whether this Jetson has it: [(what, apt package, have)]. Each is asked of
    the thing itself (the program on PATH, the library the linker finds, the module that Python imports), not of the
    package's name, which differs between Ubuntu releases (libglib2.0-0 is libglib2.0-0t64 on 24.04).
    ``have``: the tests' own answers, by apt package."""
    v = f"{PY[0]}.{PY[1]}"
    needs = [("git, to clone and update the code", "git", bool(shutil.which("git")))]
    if py is None or distro_python(py):
        needs.append((f"Python {v}'s venv module, for Lite's own environment", f"python{v}-venv", bool(py) and python_has(py, "venv")))
    if optional:
        if py is None or distro_python(py):
            needs.append((f"Python {v}'s headers, to build the Time-optimal optimiser (no ARM64 build exists)",
                          f"python{v}-dev", bool(py) and python_has(py, "headers")))
        needs += [("a C and C++ compiler, for the same build", "build-essential",
                   all(shutil.which(x) for x in ("gcc", "g++", "make"))),
                  ("libusb, which the camera library loads", "libusb-1.0-0", has_library("libusb-1.0.so.0")),
                  ("libudev, which the camera library loads", "libudev1", has_library("libudev.so.1")),
                  ("libGL, which OpenCV loads (the lid's depth contours, the colour models)", "libgl1", has_library("libGL.so.1")),
                  ("GLib, which OpenCV loads", "libglib2.0-0", has_library("libgthread-2.0.so.0"))]
    if have is not None:
        needs = [(what, apt, have.get(apt, ok)) for what, apt, ok in needs]
    return needs


def apt_install(packages: list, sudo: bool, why: str) -> None:
    """Installs the packages with apt, each command shown first; with --no-sudo, or where this user cannot become
    root, prints the commands and stops: nothing else can go on without them."""
    cmds = [["apt-get", "update"], ["apt-get", "install", "-y", "--no-install-recommends", *packages]]
    rooted = [as_root(["env", "DEBIAN_FRONTEND=noninteractive", *c]) for c in cmds]
    if not sudo or rooted[0] is None:
        lines = "\n".join("       sudo " + " ".join(c) for c in cmds)
        fail(f"{why}\n   Run these, then run this installer again:\n{lines}")
    run(rooted[0], check=False)                     # one source that does not answer must not stop the install
    run(rooted[1])


def install_python(info: dict, sudo: bool):
    """Python 3.12 where this computer has none: Ubuntu 24.04's own package (JetPack 7); on Ubuntu 22.04 (JetPack 6)
    from the deadsnakes archive, as install_lite.py advises there. Ubuntu 20.04 (JetPack 5) has none and the archive
    carries none for it any more (10 Nov 2026): there one is given with --python."""
    v = f"{PY[0]}.{PY[1]}"
    packages = [f"python{v}", f"python{v}-venv", f"python{v}-dev"]
    why = (f"This computer has no Python {v}, which Lite needs on the Jetson (the camera package has an ARM64 build "
           f"for {v} only).")
    if ubuntu_at_least(info, 24):
        apt_install(packages, sudo, why)
    elif ubuntu_at_least(info, 22):
        ppa = ["add-apt-repository", "-y", "ppa:deadsnakes/ppa"]
        manual = (f"{why}\n   {info['os_name']} has none of its own: it comes from the deadsnakes archive. Run these, "
                  "then run this installer again:\n"
                  + ("" if shutil.which("add-apt-repository") else "       sudo apt-get install -y software-properties-common\n")
                  + f"       sudo {' '.join(ppa)}\n       sudo apt-get install -y {' '.join(packages)}\n"
                  f"   Or give any Python {v} with  --python /path/to/python{v}")
        if not sudo or as_root(ppa) is None:
            fail(manual)
        note(f"{info['os_name']} has no Python {v} of its own: it is installed from the deadsnakes archive")
        # add-apt-repository, and the gpg agent it imports the archive's key with (a desktop Jetson has both)
        tools = [apt for apt, program in (("software-properties-common", "add-apt-repository"), ("gpg-agent", "gpg-agent"))
                 if not shutil.which(program)]
        if tools:
            apt_install(tools, sudo, why)
        if run(as_root(ppa), check=False) != 0:
            fail(manual)
        apt_install(packages, sudo, why)
    else:
        fail(f"{why}\n   {info['os_name'] or 'This system'} has none of its own, and the deadsnakes archive carries none "
             f"for it.\n   Install one (for example with uv, https://docs.astral.sh/uv/ :  uv python install {v}), then run "
             f"this installer again with  --python /path/to/python{v}")
    py = find_python()
    if not py:
        fail(f"Python {v} was installed and is still not found as {PY_NAME}. Run this installer again with "
             f"--python /path/to/python{v}")
    return py


def prepare_system(info: dict, a) -> str:
    """Steps 2 and 3: Python 3.12 and the system packages, installed where they are missing. Returns the Python."""
    say(f"Python {PY[0]}.{PY[1]}")
    py = find_python(a.python)
    if py is None:
        if a.check:
            note(f"** no Python {PY[0]}.{PY[1]} on this computer: this installer installs it")
        else:
            py = install_python(info, a.sudo)
    if py:
        note(f"{py} ({output([py, '-c', 'import platform; print(platform.python_version())'])})")
    say("System packages")
    needs = system_needs(py, optional=not a.no_optional)
    for what, apt, ok in needs:
        note(f"{'ok' if ok else '**'}: {what}" + ("" if ok else f"  (missing: {apt})"))
    missing = [apt for _, apt, ok in needs if not ok]
    if missing and a.check:
        a.missing += missing
    elif missing:
        apt_install(missing, a.sudo, "This Jetson lacks: " + ", ".join(missing) + ".")
        still = [apt for _, apt, ok in system_needs(py, optional=not a.no_optional) if not ok]
        if still:
            fail("still missing after the install: " + ", ".join(still) + ". See apt's messages above.")
    if py and not a.check and not python_has(py, "venv"):
        fail(f"{py} has no venv module with pip, so it cannot build Lite's environment. "
             f"Give another Python {PY[0]}.{PY[1]} with --python.")
    return py


# ── 4. the disk, the code, the installation

def existing_parent(path: Path) -> Path:
    path = Path(path)
    while not path.exists() and path != path.parent:
        path = path.parent
    return path


def check_disk(copy: Path, new: bool, cpu_torch: bool = False) -> None:
    need = (CLONE_GB if new else 0) + (ENV_CPU_TORCH_GB if cpu_torch else ENV_GB)
    try:
        free = shutil.disk_usage(str(existing_parent(copy))).free / 1e9
    except OSError:
        return
    say("Disk")
    what = ("a new copy: about 13 GB of code and history, nearly all of it the lab's recorded runs in Resources/Logs, "
            "and its environment" if new else "the copy's environment, with pip's downloads beside it,")
    note(f"{free:.0f} GB free where {copy} goes; {what} needs about {need} GB")
    if free < need:
        fail(f"not enough room in {existing_parent(copy)}: {free:.0f} GB free, about {need} GB needed.\n"
             "   Free some, or install on a bigger disk with  --dir /path/on/that/disk/alix_studio_lite"
             + ("" if cpu_torch else f"\n   (--cpu-torch needs {ENV_GB - ENV_CPU_TORCH_GB} GB less: PyTorch without its CUDA libraries)"))


def own_checkout():
    """The copy this script sits in, when it is run from inside one."""
    try:
        here = Path(__file__).resolve().parent
    except NameError:                              # piped in: python3 - < install_lite_jetson.py
        return None
    return here if (here / "tools/setup_lite.py").is_file() and (here / "install_lite.py").is_file() else None


def clone(target: Path, branch: str, repo_url) -> None:
    """A new copy, as install_lite.py clones it: the GitHub CLI when signed in, else git over HTTPS, else SSH."""
    say(f"Cloning {repo_url or REPO} ({branch}) into {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    tries = []
    if repo_url:
        tries.append(["git", "clone", "--branch", branch, repo_url, target])
    else:
        if shutil.which("gh") and works(["gh", "auth", "status"]):
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
         "   then run this installer again. Or copy the whole folder to this Jetson and run this file inside it.")


def venv_python(copy: Path) -> Path:
    return copy / ".venv/bin/python"


def drop_other_environment(copy: Path) -> None:
    """An environment of the copy built with another Python (3.13 or 3.14, which install_lite.py accepts) is removed:
    setup would update it in place, and the camera package has no ARM64 build for those."""
    vpy = venv_python(copy)
    if not vpy.exists():
        return
    version = output([vpy, "-c", "import sys; print('%d.%d' % sys.version_info[:2])"])
    if version and version != f"{PY[0]}.{PY[1]}":
        note(f"the copy's environment was built with Python {version}: it is built again with {PY[0]}.{PY[1]} "
             "(the camera package has no ARM64 build for another)")
        shutil.rmtree(str(copy / ".venv"), ignore_errors=True)


def torch_version(copy: Path) -> str:
    """The PyTorch in the copy's environment ("2.15.0", "2.15.0+cpu"), "" where there is none."""
    vpy = venv_python(copy)
    if not vpy.is_file():
        return ""
    return output([vpy, "-c", "import importlib.metadata as m; print(m.version('torch'))"])


def cpu_torch_first(copy: Path, py: str) -> None:
    """--cpu-torch: PyTorch's processor-only build put into the copy's environment before setup runs, so the colour
    models' package (ultralytics) finds PyTorch there and pip does not bring PyPI's, with its CUDA libraries. An
    environment that holds PyPI's is built again: its CUDA libraries would stay behind."""
    have = torch_version(copy)
    if have and "+cpu" not in have:
        note(f"the copy's environment holds PyTorch {have} with its CUDA libraries: it is built again without them")
        shutil.rmtree(str(copy / ".venv"), ignore_errors=True)
    vpy = venv_python(copy)
    if not works([vpy, "-m", "pip", "--version"]):
        run([py, "-m", "venv", copy / ".venv"])
    if "+cpu" in torch_version(copy):
        note("PyTorch's processor-only build is in the copy's environment")
        return
    run([vpy, "-m", "pip", "install", "torch", "torchvision", "--index-url", TORCH_CPU_INDEX])


def install(copy: Path, py: str, a) -> None:
    """The copy's own install_lite.py, with Python 3.12: it updates the copy (never over changes or commits of its
    own) and runs tools/setup_lite.py (the environment, the pinned packages, the optional ones, Lite itself). The
    test suite runs at this installer's end, after the Jetson's own steps."""
    say("Installing (the copy's install_lite.py)")
    drop_other_environment(copy)
    if a.cpu_torch and not a.no_optional:
        cpu_torch_first(copy, py)
    cmd = [py, copy / "install_lite.py", "--dir", copy, "--no-tests"]
    if a.branch:
        cmd += ["--branch", a.branch]
    if a.no_optional:
        cmd.append("--no-optional")
    if a.shortcut:
        cmd.append("--shortcut")
    run(cmd)


# ── 5. the wrist camera's access rules

#: What a rule that gives the wrist camera to this computer's users says: the camera on USB, by Intel's vendor id and
#: the D435's product id (0b07; the D435i is 0b3a), as librealsense's own file has one line for each model.
D435_RULE = re.compile(r'idVendor\}==?"8086".*idProduct\}==?"[^"]*\b(?:0b07|0b3a)\b', re.I)


def camera_rules() -> list:
    """The udev rules files on this computer that give the wrist camera (a D435 on USB) to its users: librealsense's
    99-realsense-libusb.rules, its packages' 60-librealsense2-udev-rules.rules, any other with such a rule. Asked of
    what a file says, not of its name: a Jetson has 60-librealsense2-mipi-udev-rules.rules, for cameras on MIPI, and
    the first install there (10 Nov 2026) took it for these by its name."""
    found = []
    for d in RULES_DIRS:
        try:
            files = sorted(Path(d).glob("*.rules"))
        except OSError:
            continue
        for f in files:
            try:
                if any(D435_RULE.search(line) for line in f.read_text(errors="replace").splitlines()
                       if not line.lstrip().startswith("#")):
                    found.append(str(f))
            except OSError:
                pass
    return found


def fetch(url: str, sha256: str, timeout: float = 60) -> bytes:
    """The file at ``url``, refused unless it is the one expected (its SHA-256)."""
    import urllib.request

    with urllib.request.urlopen(urllib.request.Request(url, headers={"User-Agent": "alix-studio-lite-installer"}),
                                timeout=timeout) as r:
        data = r.read()
    got = hashlib.sha256(data).hexdigest()
    if got != sha256:
        raise ValueError(f"the file is not the one expected (sha256 {got[:12]}..., expected {sha256[:12]}...)")
    return data


def install_camera_rules(sudo: bool) -> str:
    """The camera maker's udev rules, so the camera opens without root. Returns what was done, in words."""
    found = camera_rules()
    if found:
        return f"already installed ({found[0]})"
    if not shutil.which("udevadm"):
        return "not installed: this system has no udev"
    manual = (f"the file {RULES_URL} goes to {RULES_FILE} as root, then: sudo udevadm control --reload-rules && "
              "sudo udevadm trigger")
    if not sudo or as_root(["true"]) is None:
        return ("NOT installed (--no-sudo): without them the camera opens only for a user in the video group. This "
                f"installer installs them when it may use sudo; by hand, {manual}")
    try:
        data = fetch(RULES_URL, RULES_SHA256)
    except Exception as exc:                        # noqa: BLE001 - no network, a changed file: said, not fatal
        return f"NOT installed: {exc}. Run this installer again with the network up; by hand, {manual}"
    with tempfile.NamedTemporaryFile(prefix="realsense-", suffix=".rules", delete=False) as f:
        f.write(data)
    try:
        if run(as_root(["install", "-m", "0644", f.name, RULES_FILE]), check=False) != 0:
            return f"NOT installed (the copy into {RULES_FILE.parent} failed)"
        run(as_root(["udevadm", "control", "--reload-rules"]), check=False, quiet=True)
        run(as_root(["udevadm", "trigger"]), check=False, quiet=True)
    finally:
        os.unlink(f.name)
    return f"installed ({RULES_FILE}); a camera already plugged in is seen with them now"


# ── 6. what only this computer can say

#: Run with the environment's Python: Lite, the vendored ALIX code, each package, the robot's library for this
#: processor loaded as Lite loads it, the camera asked as Lite asks it, the colour model loaded and asked. One JSON line
#: for each answer, said as it comes: an answer that ends the process (a GPU library that does not fit this Jetson)
#: or never comes (the time limit) leaves the ones before it.
PROBE = r'''
import importlib, json, platform, sys
def tell(name, value):
    print("PROBE " + json.dumps({"name": name, "value": value}), flush=True)
tell("python", platform.python_version())
tell("machine", platform.machine())
def tried(fn):
    try:
        return dict(ok=True, **(fn() or {}))
    except BaseException as exc:
        return dict(ok=False, why=("%s: %s" % (type(exc).__name__, exc)).strip().splitlines()[-1][:300])
def package(name):
    def ask():
        version = str(getattr(importlib.import_module(name), "__version__", "") or "")
        if not version:                                    # a module that does not say: its distribution does
            from importlib import metadata
            for dist in ({"fcl": "python-fcl", "cv2": "opencv-python"}.get(name, name), name):
                try:
                    version = metadata.version(dist)
                    break
                except metadata.PackageNotFoundError:
                    pass
        if name == "cv2":                                  # OpenCV 5.0 has none: the camera calibration's solve refuses
            return {"version": version, "hand_eye": hasattr(importlib.import_module(name), "calibrateHandEye")}
        return {"version": version}
    return ask
def lite():
    from alix_studio_lite import vendor_paths, __version__
    vendor_paths.activate()
    return {"version": __version__, "outside": vendor_paths.assert_independent()}
def robot_library():
    import Robotic_Arm.rm_robot_interface                 # loads libapi_c.so for this processor, as robot_link does
    import Robotic_Arm.rm_ctypes_wrap as wrap
    return {"folder": str(getattr(wrap, "dll_path", ""))}
def arms():
    from alix_studio_lite.server import connection
    return {"ips": {s: connection.DEFAULTS[s]["ip"] for s in connection.ARMS}}
def sender():
    # ALIX's own 5 ms sender loop (alix_dispatch.track.stream_track: what Lite streams a lid or seat move with), on a
    # stand-in arm that takes every line at once, for 5 s: how late this computer wakes for a line when it has nothing
    # else to do. The streamer ends a stream at CANFD_LATE_ABORT_PERIODS periods late.
    from alix_studio_lite import vendor_paths
    vendor_paths.activate()
    from alix_dispatch import track
    class Arm:
        def rm_movej_canfd(self, *a):
            return 0
    period = track.CANFD_DEFAULT_PERIOD_S
    r = track.stream_track(Arm(), [[0] * 7] * 1000, period_s=period, hold_s=0.0, log=lambda *a: None)
    return {"lines": r["lines_sent"], "period_ms": period * 1000.0, "mean_ms": r["late_ms_mean"], "worst_ms": r["late_ms_max"],
            "late_lines": r["frames_late_over_period"], "ended": r["aborted"],
            "limit_ms": track.CANFD_LATE_ABORT_PERIODS * period * 1000.0}
def camera():
    from alix_studio_lite.server import camera_d435
    cam = camera_d435.D435()
    got = cam.available()
    res = {"found": bool(got["ok"]), "devices": got["devices"], "said": got["why"]}
    if got["ok"]:                                          # plugged in: the streams Lite opens, one moment of them
        try:
            s = cam.shot(wait_s=10.0)
            res["depth"] = list(s.depth.shape[:2])
            res["colour"] = None if s.colour is None else list(s.colour.shape[:2])
        except BaseException as exc:
            res["stream_error"] = ("%s" % exc)[:300]
        finally:
            cam.close()
    return res
def torch_device():
    import torch
    return {"version": torch.__version__, "cuda": bool(torch.cuda.is_available())}
def colour_model():
    # Lite's own warm-up (gap_check.GapCheck.warm: the core built, the colour model loaded and run once on a blank
    # picture), then the model asked three more times: what a frame takes on this computer
    import time
    import numpy as np
    from alix_studio_lite.server.gap_check import GapCheck
    gc = GapCheck(records=None)
    t0 = time.monotonic()
    said = gc.warm()
    if said != "ok":
        raise RuntimeError(said)
    load_s = time.monotonic() - t0
    blank, took = np.zeros((720, 1280, 3), dtype=np.uint8), []
    for _ in range(3):
        t = time.monotonic()
        gc.core().backend.check_status(blank, "commode")
        took.append(time.monotonic() - t)
    return {"load_s": round(load_s, 1), "frame_ms": int(round(min(took) * 1000))}
tell("lite", tried(lite))
for name in ("numpy", "scipy", "trimesh", "fcl", "toppra", "meshoptimizer", "cv2", "pyrealsense2", "ultralytics"):
    tell(name, tried(package(name)))
tell("robot_library", tried(robot_library))
tell("arms", tried(arms))
tell("sender", tried(sender))
tell("camera", tried(camera))
tell("torch", tried(torch_device))                          # the first touch of the GPU's libraries: after the rest
tell("colour_model", tried(colour_model))
'''


#: The answers the probe gives, in its order; and how long it may take in all (the camera's reset and first frames,
#: PyTorch's first start and the model's load are its long ones).
PROBE_ITEMS = ("lite", "numpy", "scipy", "trimesh", "fcl", "toppra", "meshoptimizer", "cv2", "pyrealsense2", "ultralytics",
               "robot_library", "arms", "sender", "camera", "torch", "colour_model")
PROBE_LIMIT_S = 300.0


def probe(copy: Path, limit_s: float = PROBE_LIMIT_S) -> dict | None:
    """What the copy's environment answers (PROBE), or None when its Python does not even start. An answer the
    process ended before, or did not reach within ``limit_s``, is marked as not given: the ones before it stand."""
    import threading

    vpy = venv_python(copy)
    if not vpy.is_file():
        return None
    try:
        p = subprocess.Popen([str(vpy), "-c", PROBE], cwd=tempfile.gettempdir(), env=clean_env(), stdin=subprocess.DEVNULL,
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, universal_newlines=True, errors="replace")
    except OSError:
        return None
    got, late = {}, []
    timer = threading.Timer(limit_s, lambda: (late.append(True), p.kill()))
    timer.start()
    try:
        for line in p.stdout:
            if line.startswith("PROBE "):
                try:
                    item = json.loads(line[6:])
                    got[item["name"]] = item["value"]
                except (ValueError, KeyError, TypeError):
                    pass
    finally:
        timer.cancel()
        p.wait()
    if "python" not in got:
        return None
    why = (f"no answer within {limit_s:.0f} s: the check was ended" if late
           else f"the check ended before this answered (exit {p.returncode})")
    first = True
    for name in PROBE_ITEMS:
        if name not in got:                         # the first one missing is the one that ended it
            got[name] = dict(ok=False, why=why if first else "not asked: the check had ended")
            first = False
    return got


def addresses() -> list:
    """This computer's IPv4 addresses, [(interface, address, prefix)], without the loopback (iproute2's own listing)."""
    found = []
    for line in output(["ip", "-4", "-o", "addr", "show"]).splitlines():
        m = re.match(r"\d+:\s+(\S+)\s+inet\s+([\d.]+)/(\d+)", line)
        if m and m.group(1) != "lo" and not m.group(1).startswith(("docker", "veth", "br-", "virbr")):   # containers' own
            found.append((m.group(1), m.group(2), int(m.group(3))))
    return found


def same_network(address: str, prefix: int, other: str) -> bool:
    try:
        a, b = (int.from_bytes(socket.inet_aton(x), "big") for x in (address, other))
    except OSError:
        return False
    mask = (0xFFFFFFFF << (32 - prefix)) & 0xFFFFFFFF
    return (a & mask) == (b & mask)


def saved_arm_ips() -> dict:
    """The arms' addresses saved on this computer's Connection page (~/ALIX-Lite/connection.json), if any."""
    try:
        raw = json.loads((Path.home() / "ALIX-Lite/connection.json").read_text(encoding="utf-8"))
        return {s: str(raw[s]["ip"]) for s in ("left", "right") if isinstance(raw.get(s), dict) and raw[s].get("ip")}
    except (OSError, ValueError, TypeError):
        return {}


def arm_ips_of(got) -> dict:
    """The arms' addresses: Lite's own (what its environment answered), then this computer's saved ones over them."""
    return dict((((got or {}).get("arms") or {}).get("ips") or {}), **saved_arm_ips())


def network_line(arm_ips: dict, found: list) -> tuple:
    """(text, ok): this computer's addresses and whether one of them is on the arms' network."""
    arms = ", ".join(f"{s.upper()} {ip}" for s, ip in arm_ips.items())
    if not found:
        return (f"** no network address on this computer; the arms are at {arms}", False)
    here = ", ".join(f"{name} {addr}/{prefix}" for name, addr, prefix in found)
    reach = [name for name, addr, prefix in found if any(same_network(addr, prefix, ip) for ip in arm_ips.values())]
    if reach:
        return (f"{here}; the arms ({arms}) are on {reach[0]}'s network", True)
    subnet = ".".join(next(iter(arm_ips.values())).split(".")[:3]) + ".x" if arm_ips else "the arms' network"
    return (f"{here}\n   ** none of them is on the arms' network ({arms}): give the port wired to the arms an address "
            f"in {subnet} (not an arm's own); Lite's Connection page then tests each device", False)


def power_mode():
    """The Jetson's power mode (nvpmodel), or None where it cannot be read."""
    m = re.search(r"NV Power Mode:\s*(\S+)", output(["nvpmodel", "-q"], timeout=20))
    return m.group(1) if m else None


def chrome():
    """Google Chrome or Chromium that Lite can give a window of its own (not a snap: server._chrome)."""
    for name in ("google-chrome", "google-chrome-stable", "chromium"):
        path = shutil.which(name)
        if path and "/snap/" not in path:
            return path
    return None


def ssh_server() -> bool:
    try:
        with socket.create_connection(("127.0.0.1", 22), timeout=0.5):
            return True
    except OSError:
        return False


def report(copy: Path, info: dict, got, rules: str | None, optional: bool) -> list:
    """What this Jetson has, line by line. Returns what needs looking at (each said with ** where it is)."""
    say("On this Jetson")
    issues = []

    def line(ok: bool, text: str, issue: str | None = None) -> None:
        note(("" if ok else "** ") + text)
        if not ok:
            issues.append(issue or text)

    if got is None:
        line(False, "Lite's environment could not be asked (see the installation above)", "the environment does not answer")
        return issues
    pk = lambda name: got.get(name) or {"ok": False, "why": "not asked"}           # noqa: E731
    v = lambda name: pk(name).get("version", "")                                  # noqa: E731
    lite = pk("lite")
    line(lite["ok"] and not lite.get("outside"),
         f"Lite {v('lite')} on Python {got['python']} ({got['machine']}); numpy {v('numpy')}, scipy {v('scipy')}"
         if lite["ok"] else f"Lite does not import: {lite.get('why')}", "Lite does not import")
    for name in ("numpy", "scipy", "trimesh", "fcl"):
        if not pk(name)["ok"]:
            line(False, f"{name} does not import: {pk(name).get('why')}")
    lib = pk("robot_library")
    line(lib["ok"], "The robot's library (RealMan, ARM64): loads" if lib["ok"]
         else f"The robot's library does not load: {lib.get('why')}", "the robot's library does not load")
    snd = pk("sender")
    if snd["ok"]:
        # half the streamer's limit is taken as little room: a line that late on an idle computer leaves a loaded one none
        roomy = not snd.get("ended") and float(snd.get("worst_ms") or 0.0) < 0.5 * float(snd.get("limit_ms") or 10.0)
        line(roomy, f"5 ms stream sender, this computer idle ({snd.get('lines')} lines to a stand-in arm): "
             f"{snd.get('mean_ms'):.2f} ms late on average, {snd.get('worst_ms'):.1f} ms at worst; a stream ends itself at "
             f"{snd.get('limit_ms'):.0f} ms" + ("" if roomy else ": little room, look for what else runs on this computer"),
             "the 5 ms stream sender has little room on this computer")
    else:
        line(False, f"5 ms stream sender: could not be measured ({snd.get('why')})", "the 5 ms stream sender was not measured")
    if not optional:
        note("The optional packages were left out (--no-optional): no Time-optimal optimiser, no wrist camera, no colour models")
        return issues
    line(pk("toppra")["ok"], f"Time-optimal optimiser ({('toppra ' + v('toppra')).strip()}): built" if pk("toppra")["ok"]
         else f"Time-optimal optimiser: NOT built ({pk('toppra').get('why')}); Lite offers the Default optimiser only",
         "the Time-optimal optimiser was not built")
    rs, cam = pk("pyrealsense2"), pk("camera")
    if rs["ok"]:
        text = f"Wrist camera library (pyrealsense2 {v('pyrealsense2')}): installed; its access rules: {rules or 'not looked at'}"
        line("NOT installed" not in (rules or ""), text, "the camera's access rules are not installed")
        if not cam["ok"]:
            line(False, f"The camera could not be asked: {cam.get('why')}")
        elif not cam.get("found"):
            note("no D435 is plugged into this Jetson now: plug it in and run this installer with --check to see it stream")
        elif cam.get("stream_error"):
            line(False, f"A D435 is plugged in ({cam['devices'][0]['name']}) and sent no frames: {cam['stream_error']}",
                 "the camera is plugged in and does not stream")
        else:
            d, c = cam.get("depth"), cam.get("colour")
            note(f"A D435 is plugged in ({cam['devices'][0]['name']}): its depth {d[1]} x {d[0]}"
                 + (f" and colour {c[1]} x {c[0]}" if c else "") + " arrive, as Lite opens them")
    else:
        absent = "No module named" in str(rs.get("why"))
        line(False, f"Wrist camera library: {'NOT installed' if absent else 'does NOT load'} ({rs.get('why')}). The lid and "
             "seat tasks run in Simulation and Hardware sim only; Real refuses their checks."
             + ("" if absent or ubuntu_at_least(info, 24) else " Its ARM64 build needs Ubuntu 24.04's libraries (glibc 2.38)."),
             "the camera library is not installed" if absent else "the camera library does not load")
    cv, ul, torch = pk("cv2"), pk("ultralytics"), pk("torch")
    if cv["ok"] and not cv.get("hand_eye", True):
        line(False, f"OpenCV {v('cv2')} has no calibrateHandEye: the camera calibration cannot be solved with it. Run this "
             "installer again: it installs the OpenCV Lite is tested with (requirements-optional.txt)",
             "this OpenCV cannot solve the camera calibration")
    else:
        line(cv["ok"], f"OpenCV {v('cv2')}: installed (the lid's depth contours, the camera calibration)" if cv["ok"]
             else f"OpenCV does not import ({cv.get('why')}): the lid is read by the fit alone", "OpenCV does not import")
    model = pk("colour_model")
    if ul["ok"] and torch["ok"] and model["ok"]:
        where = ("the GPU (CUDA)" if torch.get("cuda") else
                 "the processor (this PyTorch finds no GPU it can use on this Jetson; the model is small)")
        note(f"Colour model (ultralytics {v('ultralytics')}, PyTorch {v('torch')}): loads in {model.get('load_s')} s and "
             f"answers a frame in {model.get('frame_ms')} ms, on {where}")
        if not torch.get("cuda") and "+cpu" not in v("torch"):
            note("  its CUDA libraries (most of the environment's 7 GB) are not used here: this installer with --cpu-torch "
                 "builds the environment again without them (1.5 GB)")
    else:
        bad = ul if not ul["ok"] else torch if not torch["ok"] else model
        line(False, f"Colour model: does NOT answer ({bad.get('why')}); the depth answers alone", "the colour model does not answer")
    if info["jetson"]:
        mode = power_mode()
        if mode:
            note(f"Power mode: {mode}" + ("" if mode.upper().startswith("MAXN")
                                           else "  (the steadiest timing for Lite's 5 ms streams is MAXN: sudo nvpmodel -m 0)"))
    arm_ips = arm_ips_of(got)
    if arm_ips:
        text, ok = network_line(arm_ips, addresses())
        note("Network: " + text)
        if not ok:
            issues.append("no address on the arms' network")
    return issues


def tunnel_address(found: list, arm_ips: dict) -> str:
    """The address the operator PC reaches this Jetson at: the one on the arms' network (the PC is on it), else the
    first; a placeholder where this computer has none."""
    for _, addr, prefix in found:
        if any(same_network(addr, prefix, ip) for ip in arm_ips.values()):
            return addr
    return found[0][1] if found else "THE-JETSONS-ADDRESS"


def how_to_start(copy: Path, arm_ips: dict | None = None) -> None:
    say("Start Lite")
    start = q(copy / "tools/run_lite.sh")
    screen = os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    try:
        import getpass

        user = getpass.getuser()
    except Exception:                               # noqa: BLE001 - no name for this user: a placeholder
        user = "USER"
    host = tunnel_address(addresses(), arm_ips or {})
    note(f"With the real arms:   {start} --robot realman")
    note(f"Without (stand-in):   {start}")
    note(f"It opens:             http://127.0.0.1:{PORT}/   (log in with the operator PIN, 1234 until changed)")
    if screen:
        note("On this Jetson's screen it opens "
             + ("in a Chrome window of its own." if chrome() else
                "in the default browser. For a window of its own (the 3D with Chrome's GPU blocklist ignored), install "
                "Google Chrome for ARM64: https://www.google.com/chrome/"))
    else:
        note("This session has no screen: add --no-browser.")
    note("From the operator PC (Lite listens on the Jetson itself only, so the PC reaches it through an SSH tunnel):")
    note(f"    on the Jetson:  {start} --robot realman --no-browser")
    note(f"    on the PC:      ssh -N -L {PORT}:127.0.0.1:{PORT} {user}@{host}      then open http://127.0.0.1:{PORT}/ there")
    if not ssh_server():
        note("    ** no SSH server answers on this Jetson: sudo apt-get install -y openssh-server")
    note(f"Change a PIN:         {start} --set-pin operator      (or engineer)")
    note(f"Update later:         python3 {q(copy / 'install_lite_jetson.py')}")
    note(f"Read first:           {copy / 'README.md'}")


# ── 7. the test suite

def tee(cmd: list, cwd) -> tuple:
    """The command run with its output shown as it comes and kept: (exit code, the text)."""
    print("   $ " + " ".join(str(c) for c in cmd), flush=True)
    p = subprocess.Popen([str(c) for c in cmd], cwd=str(cwd), env=clean_env(), stdin=subprocess.DEVNULL,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    raw, kept = getattr(sys.stdout, "buffer", None), []
    while True:
        data = os.read(p.stdout.fileno(), 4096)
        if not data:
            break
        kept.append(data)
        if raw is not None:
            raw.write(data)
            raw.flush()
        else:
            sys.stdout.write(data.decode("utf-8", "replace"))
            sys.stdout.flush()
    return p.wait(), b"".join(kept).decode("utf-8", "replace")


def failed_tests(text: str) -> list:
    """The tests pytest's summary names as failed or in error (its FAILED / ERROR lines), in order, once each."""
    found = []
    for m in re.finditer(r"^(?:FAILED|ERROR) (\S.*?)(?: - .*)?$", text, re.M):
        name = m.group(1).strip()
        if "::" in name and name not in found:
            found.append(name)
    return found


def suite_workers(cores=None) -> int:
    """The suite's workers on this computer: one for every two cores, 8 at most (the suite's own number, pyproject.toml),
    2 at least. A test of the emulated arms runs a Lite server and two arm processes beside its own worker, and its 5 ms
    streams end themselves when fed late: on the Jetson's 12 cores 8 workers made one or two of them late in every full
    run (10 Nov 2026); on a PC with 16 cores or more the number is the suite's own."""
    cores = cores or os.cpu_count() or 16
    return max(2, min(8, int(cores) // 2))


def run_tests(copy: Path) -> dict:
    """The test suite as setup runs it (8 workers, a file each), after setup's own check of the environment. Tests
    that fail in the full run are run again alone, in one process: ALIX's streamer ends a 5 ms stream whose sender is
    more than two periods late (track.CANFD_LATE_ABORT_PERIODS: it stops feeding the arm sooner than feed it late),
    which 8 workers on every core can cause on the Jetson, and which is then the test run's load, not Lite's (the
    Jetson's first install, 10 Nov 2026: two emulated lid runs, "47 of 264" and "212 of 589 lines sent").
    Returns {ok: nothing fails alone, failed: in the full run, alone: those that pass alone, still: those that do not}."""
    say("Check (the test suite)")
    vpy = venv_python(copy)
    if run([vpy, copy / "tools/setup_lite.py", "--check", "--no-tests"], cwd=copy, check=False) != 0:
        return dict(ok=False, failed=[], alone=[], still=["the environment's own check (above)"])
    t0 = time.time()
    rc, text = tee([vpy, "-m", "pytest", "-q", "-n", str(suite_workers()), "-p", "no:cacheprovider"], copy)
    note(f"tests {'passed' if rc == 0 else 'did NOT all pass'} in {time.time() - t0:.0f} s")
    if rc == 0:
        return dict(ok=True, failed=[], alone=[], still=[])
    failed = failed_tests(text)
    if not failed:                                  # the run itself failed (nothing collected, interrupted)
        return dict(ok=False, failed=[], alone=[], still=["the test run itself (above)"])
    say(f"The {len(failed)} test{'' if len(failed) == 1 else 's'} that did not pass, run again alone")
    note("in one process, with the computer to themselves: a 5 ms stream ends itself when its sender is more than 10 ms "
         "late, which the full run's workers on every core can cause")
    rc, text = tee([vpy, "-m", "pytest", "-q", "-n", "0", "-p", "no:cacheprovider", *failed], copy)
    still = [] if rc == 0 else (failed_tests(text) or list(failed))
    alone = [name for name in failed if name not in still]
    if alone:
        note(f"{len(alone)} of them pass{'es' if len(alone) == 1 else ''} alone: the full run's load made "
             f"{'it' if len(alone) == 1 else 'them'} late, not Lite"
             + ("" if still else ". A Jetson that is not in the MAXN power mode is likelier to do this (sudo nvpmodel -q)"))
    for name in still:
        note(f"** fails alone too: {name}")
    return dict(ok=not still, failed=failed, alone=alone, still=still)


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    ap = argparse.ArgumentParser(description="Prepare the robot's Jetson, then clone (or update) ALIX Studio Lite and install it.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--dir", type=Path, help="where the code goes (default: this copy, else ~/alix_studio_lite)")
    ap.add_argument("--branch", help=f"the branch to use (default: {BRANCH} for a new clone; an existing copy stays on its own)")
    ap.add_argument("--repo", help="clone from this URL or path instead of GitHub")
    ap.add_argument("--python", help=f"the Python {PY[0]}.{PY[1]} to build Lite's environment with (default: the one found)")
    ap.add_argument("--check", action="store_true", help="change nothing: say what this Jetson has and what is missing")
    ap.add_argument("--no-sudo", dest="sudo", action="store_false",
                    help="change nothing system-wide: print the commands to run instead of running them with sudo")
    ap.add_argument("--no-tests", action="store_true", help="skip the test suite at the end")
    ap.add_argument("--no-optional", action="store_true",
                    help="skip the optional packages (the Time-optimal optimiser, the wrist camera, its colour models)")
    ap.add_argument("--cpu-torch", action="store_true",
                    help="PyTorch's processor-only build for the colour model (a 1.5 GB environment), instead of PyPI's "
                         "with CUDA libraries for ARM servers (7 GB): for a Jetson whose GPU those do not drive")
    ap.add_argument("--shortcut", action="store_true", help="add a launcher to the desktop / applications menu")
    ap.add_argument("--start", action="store_true", help="start Lite when installed")
    a = ap.parse_args(argv)
    a.missing = []

    info = this_computer()
    check_computer(info)
    py = prepare_system(info, a)
    copy = a.dir.expanduser().resolve() if a.dir else (own_checkout() or Path.home() / "alix_studio_lite")
    have_copy = (copy / "tools/setup_lite.py").is_file()

    if a.check:
        got = probe(copy) if have_copy else None
        rules = (camera_rules() or ["NOT installed"])[0] if not a.no_optional else None
        issues = report(copy, info, got, rules, optional=not a.no_optional) if have_copy and got else []
        if not have_copy:
            say(f"No copy of Lite in {copy} yet: run this installer without --check to install it")
        elif got is None:
            say(f"The copy in {copy} has no working environment yet: run this installer without --check")
        if a.missing:
            say("Missing system packages: " + ", ".join(a.missing) + "  (this installer installs them)")
        if have_copy and got:
            how_to_start(copy, arm_ips_of(got))
        return 1 if (a.missing or issues or not have_copy or got is None) else 0

    if not have_copy and copy.exists() and any(copy.iterdir()):
        fail(f"{copy} exists and is not a copy of Lite. Choose another place with --dir.")
    check_disk(copy, new=not have_copy, cpu_torch=a.cpu_torch)
    if not have_copy:
        clone(copy, a.branch or BRANCH, a.repo)
    install(copy, py, a)
    rules = None
    if not a.no_optional:
        say("The wrist camera's access rules (udev)")
        rules = install_camera_rules(a.sudo)
        note(rules)
    got = probe(copy)
    issues = report(copy, info, got, rules, optional=not a.no_optional)
    tests = dict(ok=True, failed=[], alone=[], still=[])
    if not a.no_tests:
        tests = run_tests(copy)
        if tests["still"]:
            issues.append("the test suite did not pass (" + ", ".join(t.split("::")[-1] for t in tests["still"]) + ")")
        elif tests["alone"]:
            n = len(tests["alone"])
            issues.append(f"{n} test{'' if n == 1 else 's'} passed only when run alone (timing under the full run's load)")
    how_to_start(copy, arm_ips_of(got))
    say("ALIX Studio Lite is installed on this Jetson"
        + ("" if not issues else ", with " + ("this" if len(issues) == 1 else "these") + " to look at: " + "; ".join(issues)))
    if a.start:
        say("Starting Lite (Ctrl+C to stop)")
        return subprocess.call([str(venv_python(copy)), str(copy / "tools/run_lite.py")], env=clean_env())
    return 0 if tests["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
