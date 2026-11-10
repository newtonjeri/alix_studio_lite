"""install_lite_jetson.py, the Jetson's twin of install_lite.py (the user, 10 Nov: "I want to set up this code in
jetson, can we have a similar setup file for that similar to install lite"; the robot's computer is a Jetson AGX Orin
Developer Kit, ARM64, Ubuntu 24.04). Its own decisions, on files and answers of the tests' own: which computer this is,
which Python, what the system lacks and how it is said, the camera's rules, the network toward the arms, the report.
The whole run was made on an emulated ARM64 Ubuntu (24.04 and 22.04); that is not a test of this suite."""
import ast
import hashlib
import importlib.util
import stat
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "install_lite_jetson.py"


@pytest.fixture(scope="module")
def jet():
    spec = importlib.util.spec_from_file_location("install_lite_jetson", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def tree(tmp_path, model=None, tegra=None, version="24.04", name="Ubuntu 24.04.3 LTS") -> Path:
    """A computer's own files: the device tree's model (NUL-ended, as the kernel gives it), L4T's release, os-release."""
    (tmp_path / "proc/device-tree").mkdir(parents=True)
    (tmp_path / "etc").mkdir()
    if model:
        (tmp_path / "proc/device-tree/model").write_bytes(model.encode() + b"\x00")
    if tegra:
        (tmp_path / "etc/nv_tegra_release").write_text(tegra + "\n")
    (tmp_path / "etc/os-release").write_text(f'PRETTY_NAME="{name}"\nNAME="Ubuntu"\nVERSION_ID="{version}"\nID=ubuntu\nID_LIKE=debian\n')
    return tmp_path


ORIN = "NVIDIA Jetson AGX Orin Developer Kit"
R38 = "# R38 (release), REVISION: 2.0, GCID: 41844464, BOARD: generic, EABI: aarch64, DATE: Fri Aug 22 00:55:42 UTC 2025"
R36 = "# R36 (release), REVISION: 4.3, GCID: 38968081, BOARD: generic, EABI: aarch64, DATE: Wed Jan  8 01:49:37 UTC 2025"
R35 = "# R35 (release), REVISION: 6.0, GCID: 37391689, BOARD: t186ref, EABI: aarch64, DATE: Wed Aug 28 19:52:46 UTC 2024"


def test_the_jetson_says_what_it_is_in_its_own_files(jet, tmp_path):
    info = jet.this_computer(tree(tmp_path, ORIN, R38), machine="aarch64")
    assert info["jetson"] and info["model"] == ORIN and info["l4t"] == "R38.2.0" and info["jetpack"] == "7"
    assert info["os_id"] == "ubuntu" and info["os_version"] == "24.04" and jet.ubuntu_at_least(info, 24)
    assert jet.describe(info) == "NVIDIA Jetson AGX Orin Developer Kit, L4T R38.2.0 (JetPack 7), Ubuntu 24.04.3 LTS, aarch64"


@pytest.mark.parametrize("tegra, version, jetpack, new", [(R36, "22.04", "6", False), (R35, "20.04", "5", False)])
def test_an_older_jetpack_is_named_and_has_no_python_of_its_own(jet, tmp_path, tegra, version, jetpack, new):
    info = jet.this_computer(tree(tmp_path, ORIN, tegra, version, f"Ubuntu {version} LTS"), machine="aarch64")
    assert info["jetpack"] == jetpack and jet.ubuntu_at_least(info, 24) is new


def test_another_arm64_computer_is_installed_as_one_and_an_x86_one_is_refused(jet, tmp_path, capsys):
    info = jet.this_computer(tree(tmp_path), machine="aarch64")
    assert not info["jetson"] and info["l4t"] is None
    jet.check_computer(info)                                           # goes on, saying so
    assert "not a Jetson" in capsys.readouterr().out
    with pytest.raises(SystemExit) as stop:
        jet.check_computer(jet.this_computer(tmp_path, machine="x86_64"))
    assert "this computer is x86_64" in str(stop.value) and "install_lite.py" in str(stop.value)


def test_what_the_system_lacks_is_asked_of_the_thing_itself(jet):
    """Each need is asked of the program, the library or the module, and named by the apt package that brings it; a
    Python that is not the distribution's own brings its own venv and headers (no apt package of them)."""
    everything = {"git": True, "python3.12-venv": True, "python3.12-dev": True, "build-essential": True, "libusb-1.0-0": True,
                  "libudev1": True, "libgl1": True, "libglib2.0-0": True}
    needs = jet.system_needs("/usr/bin/python3.12", optional=True, have=everything)
    assert [apt for _, apt, _ in needs] == list(everything) and all(ok for _, _, ok in needs)
    bare = jet.system_needs("/usr/bin/python3.12", optional=True, have=dict(everything, git=False, **{"python3.12-venv": False}))
    assert [apt for _, apt, ok in bare if not ok] == ["git", "python3.12-venv"]
    assert [apt for _, apt, _ in jet.system_needs("/usr/bin/python3.12", optional=False, have=everything)] == ["git", "python3.12-venv"]
    own = [apt for _, apt, _ in jet.system_needs("/opt/python/bin/python3.12", optional=True, have=everything)]
    assert "python3.12-venv" not in own and "python3.12-dev" not in own and "build-essential" in own
    none = dict((apt, ok) for _, apt, ok in jet.system_needs(None, optional=True))
    assert none["python3.12-venv"] is False and none["python3.12-dev"] is False       # no Python: both to install


def test_without_sudo_the_commands_are_printed_and_nothing_runs(jet, monkeypatch):
    ran = []
    monkeypatch.setattr(jet, "run", lambda cmd, **kw: ran.append(cmd) or 0)
    with pytest.raises(SystemExit) as stop:
        jet.apt_install(["git", "python3.12-venv"], sudo=False, why="This Jetson lacks: git, python3.12-venv.")
    said = str(stop.value)
    assert "This Jetson lacks: git, python3.12-venv." in said and "sudo apt-get update" in said
    assert "sudo apt-get install -y --no-install-recommends git python3.12-venv" in said and not ran
    # a user who cannot become root (no sudo on this computer): the same
    monkeypatch.setattr(jet.os, "geteuid", lambda: 1000)
    monkeypatch.setattr(jet.shutil, "which", lambda name: None)
    with pytest.raises(SystemExit):
        jet.apt_install(["git"], sudo=True, why="x")
    assert not ran


def test_with_sudo_each_command_is_run_as_root(jet, monkeypatch):
    ran = []
    monkeypatch.setattr(jet, "run", lambda cmd, **kw: ran.append([str(c) for c in cmd]) or 0)
    monkeypatch.setattr(jet.os, "geteuid", lambda: 1000)
    monkeypatch.setattr(jet.shutil, "which", lambda name: "/usr/bin/sudo" if name == "sudo" else None)
    jet.apt_install(["git"], sudo=True, why="x")
    assert ran == [["sudo", "env", "DEBIAN_FRONTEND=noninteractive", "apt-get", "update"],
                   ["sudo", "env", "DEBIAN_FRONTEND=noninteractive", "apt-get", "install", "-y", "--no-install-recommends", "git"]]
    ran.clear()
    monkeypatch.setattr(jet.os, "geteuid", lambda: 0)                 # root itself: no sudo
    jet.apt_install(["git"], sudo=True, why="x")
    assert ran[1][:4] == ["env", "DEBIAN_FRONTEND=noninteractive", "apt-get", "install"]


def test_python_312_comes_from_ubuntu_2404_itself_and_from_deadsnakes_before_it(jet, tmp_path, monkeypatch):
    """Ubuntu 24.04 (JetPack 7) has Python 3.12 of its own; 22.04 (JetPack 6) and 20.04 (JetPack 5) have not: there it
    is the deadsnakes archive's, as install_lite.py advises. Without sudo both are said, with --python as the other way."""
    noble = jet.this_computer(tree(tmp_path / "a", ORIN, R38), machine="aarch64")
    with pytest.raises(SystemExit) as stop:
        jet.install_python(noble, sudo=False)
    assert "sudo apt-get install -y --no-install-recommends python3.12 python3.12-venv python3.12-dev" in str(stop.value)
    assert "deadsnakes" not in str(stop.value)
    jammy = jet.this_computer(tree(tmp_path / "b", ORIN, R36, "22.04", "Ubuntu 22.04.5 LTS"), machine="aarch64")
    with pytest.raises(SystemExit) as stop:
        jet.install_python(jammy, sudo=False)
    said = str(stop.value)
    assert "Ubuntu 22.04.5 LTS has none of its own" in said and "sudo add-apt-repository -y ppa:deadsnakes/ppa" in said
    assert "sudo apt-get install -y python3.12 python3.12-venv python3.12-dev" in said and "--python /path/to/python3.12" in said
    ran = []
    monkeypatch.setattr(jet, "run", lambda cmd, **kw: ran.append([str(c) for c in cmd]) or 0)
    monkeypatch.setattr(jet.os, "geteuid", lambda: 0)
    monkeypatch.setattr(jet.shutil, "which", lambda name: f"/usr/bin/{name}")
    monkeypatch.setattr(jet, "find_python", lambda explicit=None: "/usr/bin/python3.12")
    assert jet.install_python(jammy, sudo=True) == "/usr/bin/python3.12"
    assert ran[0] == ["add-apt-repository", "-y", "ppa:deadsnakes/ppa"] and ran[-1][-3:] == ["python3.12", "python3.12-venv", "python3.12-dev"]
    # a Jetson without add-apt-repository or the gpg agent it imports the archive's key with (the emulated 22.04,
    # 10 Nov: "gpg: failed to start agent"): both are installed first
    ran.clear()
    monkeypatch.setattr(jet.shutil, "which", lambda name: None if name in ("add-apt-repository", "gpg-agent") else f"/usr/bin/{name}")
    jet.install_python(jammy, sudo=True)
    assert ran[1][-2:] == ["software-properties-common", "gpg-agent"] and ran[2] == ["add-apt-repository", "-y", "ppa:deadsnakes/ppa"]
    # Ubuntu 20.04 (JetPack 5): no Python 3.12 of its own and none in the archive any more: one is given
    focal = jet.this_computer(tree(tmp_path / "c", ORIN, R35, "20.04", "Ubuntu 20.04.6 LTS"), machine="aarch64")
    ran.clear()
    with pytest.raises(SystemExit) as stop:
        jet.install_python(focal, sudo=True)
    assert "Ubuntu 20.04.6 LTS has none of its own" in str(stop.value) and "--python /path/to/python3.12" in str(stop.value)
    assert "uv python install 3.12" in str(stop.value) and not ran


def test_a_given_python_must_be_312(jet):
    with pytest.raises(SystemExit) as stop:
        jet.find_python("/bin/true-is-not-python")
    assert "is not Python 3.12" in str(stop.value)
    if sys.version_info[:2] == (3, 12):
        assert jet.find_python(sys.executable) == sys.executable and jet.python_has(sys.executable, "venv")


def test_the_cameras_rules_are_taken_only_with_their_checksum(jet, monkeypatch):
    import urllib.request

    class Reply:
        def __init__(self, data):
            self.data = data

        def read(self):
            return self.data

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    rules = b'SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE:="0666", GROUP:="plugdev"\n'
    monkeypatch.setattr(urllib.request, "urlopen", lambda req, timeout=None: Reply(rules))
    assert jet.fetch("https://example.invalid/rules", hashlib.sha256(rules).hexdigest()) == rules
    with pytest.raises(ValueError, match="not the one expected"):
        jet.fetch("https://example.invalid/rules", "0" * 64)


def test_the_cameras_rules_are_left_alone_where_they_are_and_said_where_they_cannot_go(jet, tmp_path, monkeypatch):
    there = tmp_path / "rules.d"
    there.mkdir()
    usb = 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE:="0666", GROUP:="plugdev"\n'
    # a Jetson's rules for cameras on MIPI are named after RealSense and give the USB camera nothing: the first install
    # there (10 Nov) took them for the camera's rules by their name and installed none
    (there / "60-librealsense2-mipi-udev-rules.rules").write_text(
        '# D4XX MIPI\nSUBSYSTEM=="video4linux", ATTR{name}=="DS5 mux*", SYMLINK+="video-rs-depth-%n"\n'
        '# SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07"   (a comment is not a rule)\n')
    monkeypatch.setattr(jet, "RULES_DIRS", (str(there), str(tmp_path / "none")))
    assert jet.camera_rules() == []
    (there / "60-librealsense2-udev-rules.rules").write_text("# a package's\n" + usb)
    assert jet.camera_rules() == [str(there / "60-librealsense2-udev-rules.rules")]
    assert jet.install_camera_rules(sudo=True) == f"already installed ({there / '60-librealsense2-udev-rules.rules'})"
    (there / "60-librealsense2-udev-rules.rules").write_text(
        'KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad5|0afe|0b3a|0b3d", RUN+="/bin/true"\n')
    assert jet.camera_rules() == [str(there / "60-librealsense2-udev-rules.rules")]          # a list of models with the D435i
    monkeypatch.setattr(jet, "RULES_DIRS", (str(tmp_path / "none"),))
    monkeypatch.setattr(jet.shutil, "which", lambda name: None)                     # no udev at all
    assert jet.install_camera_rules(sudo=True) == "not installed: this system has no udev"
    monkeypatch.setattr(jet.shutil, "which", lambda name: f"/usr/bin/{name}")
    said = jet.install_camera_rules(sudo=False)
    assert said.startswith("NOT installed (--no-sudo)") and jet.RULES_URL in said and "udevadm control --reload-rules" in said
    monkeypatch.setattr(jet, "fetch", lambda url, sha: (_ for _ in ()).throw(OSError("no network")))
    assert jet.install_camera_rules(sudo=True).startswith("NOT installed: no network")


def test_the_rules_are_the_camera_packages_release(jet):
    """requirements-optional.txt pins the camera package; the rules are fetched from the same librealsense release."""
    pin = next(ln for ln in (ROOT / "requirements-optional.txt").read_text().splitlines() if ln.startswith("pyrealsense2=="))
    release = ".".join(pin.split("==")[1].split(".")[:3])
    assert f"/v{release}/" in jet.RULES_URL and len(jet.RULES_SHA256) == 64


def test_the_optimisers_source_build_names_its_python_folder():
    """Where toppra has no wheel (the Jetson: ARM64) setup builds it from its git tag. Its Python project is the
    repository's python/ folder: without it pip refused the build ("neither 'setup.py' nor 'pyproject.toml'") and
    the Jetson had no Time-optimal optimiser (the emulated ARM64 Ubuntu, 10 Nov). The tag is the pinned version."""
    spec = importlib.util.spec_from_file_location("setup_lite", ROOT / "tools/setup_lite.py")
    setup = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(setup)
    pin = next(ln for ln in (ROOT / "requirements-optional.txt").read_text().splitlines() if ln.startswith("toppra=="))
    assert setup.TOPPRA_GIT == f"toppra @ git+https://github.com/hungpham2511/toppra@{pin.split('==')[1]}#subdirectory=python"


def test_the_optimisers_expected_source_build_is_not_said_as_an_error(monkeypatch, capsys):
    """The Jetson, 10 Nov: PyPI has no ARM64 toppra, pip said so in two ERROR lines and the install read as failed,
    when building it from source is the normal way there. PyPI is asked without its answer being shown; what is said
    is which way it goes. pip's own words are shown only when it failed for another reason (no network)."""
    from types import SimpleNamespace

    spec = importlib.util.spec_from_file_location("setup_lite", ROOT / "tools/setup_lite.py")
    setup = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(setup)
    vpy, built, asked = Path("/home/dev/alix_studio_lite/.venv/bin/python"), [], []
    monkeypatch.setattr(setup, "run", lambda cmd, **kw: built.append([str(c) for c in cmd]) or 0)

    def pypi(returncode, stderr=""):
        def fake(cmd, **kw):
            asked.append([str(c) for c in cmd])
            return SimpleNamespace(returncode=returncode, stdout="", stderr=stderr)
        monkeypatch.setattr(setup.subprocess, "run", fake)

    pypi(0)                                                                        # Linux x86-64: PyPI's wheel
    assert setup.install_toppra(vpy, "toppra==0.6.10") and not built
    assert asked[0][-4:] == ["install", "--only-binary", ":all:", "toppra==0.6.10"]
    none = ("ERROR: Could not find a version that satisfies the requirement toppra==0.6.10 (from versions: 0.2.2a0, 0.6.3)\n"
            "ERROR: No matching distribution found for toppra==0.6.10\n")
    capsys.readouterr()
    pypi(1, none)                                                                  # the Jetson: none, so its source
    assert setup.install_toppra(vpy, "toppra==0.6.10")
    said = capsys.readouterr().out
    assert "PyPI has no ready-made toppra for this computer" in said and "the normal way on this computer" in said
    assert "ERROR" not in said and built == [[str(vpy), "-m", "pip", "install", "cython", setup.TOPPRA_GIT]]
    built.clear()
    pypi(1, "ERROR: Could not install packages due to an OSError: Network is unreachable\n")
    setup.install_toppra(vpy, "toppra==0.6.10")
    said = capsys.readouterr().out
    assert "PyPI's toppra could not be installed" in said and "Network is unreachable" in said and len(built) == 1


def test_the_network_toward_the_arms(jet):
    arms = {"left": "192.168.1.10", "right": "192.168.1.103"}
    assert jet.same_network("192.168.1.50", 24, "192.168.1.10") and not jet.same_network("10.0.0.7", 24, "192.168.1.10")
    text, ok = jet.network_line(arms, [("wlP1p1s0", "10.0.0.7", 24), ("end0", "192.168.1.50", 24)])
    assert ok and text.endswith("the arms (LEFT 192.168.1.10, RIGHT 192.168.1.103) are on end0's network")
    text, ok = jet.network_line(arms, [("wlP1p1s0", "10.0.0.7", 24)])
    assert not ok and "none of them is on the arms' network" in text and "192.168.1.x" in text
    assert not jet.network_line(arms, [])[1]
    # the operator PC's tunnel goes to the address on the arms' network, which the PC is on
    assert jet.tunnel_address([("wlP1p1s0", "10.0.0.7", 24), ("end0", "192.168.1.50", 24)], arms) == "192.168.1.50"
    assert jet.tunnel_address([("wlP1p1s0", "10.0.0.7", 24)], arms) == "10.0.0.7"
    assert jet.tunnel_address([], arms) == "THE-JETSONS-ADDRESS"


def answers(**changes) -> dict:
    """What the environment's own Python answers on a Jetson with everything installed and no camera plugged in."""
    ok = lambda v: {"ok": True, "version": v}                                    # noqa: E731
    got = {"python": "3.12.3", "machine": "aarch64", "lite": {"ok": True, "version": "0.1.0", "outside": []},
           "numpy": ok("2.4.6"), "scipy": ok("1.17.1"), "trimesh": ok("4.12.2"), "fcl": ok(""), "toppra": ok("0.6.10"),
           "meshoptimizer": ok("0.2.30a0"), "cv2": {"ok": True, "version": "4.13.0", "hand_eye": True},
           "pyrealsense2": ok("2.57.7.10387"), "ultralytics": ok("8.4.55"),
           "torch": {"ok": True, "version": "2.15.0", "cuda": False}, "colour_model": {"ok": True, "load_s": 4.2, "frame_ms": 85},
           "robot_library": {"ok": True, "folder": "libs/linux_arm"},
           "sender": {"ok": True, "lines": 1000, "period_ms": 5.0, "mean_ms": 0.11, "worst_ms": 1.24, "late_lines": 0,
                      "ended": None, "limit_ms": 10.0},
           "arms": {"ok": True, "ips": {"left": "192.168.1.10", "right": "192.168.1.103"}},
           "camera": {"ok": True, "found": False, "devices": [], "said": "no D435 is plugged into this computer"}}
    got.update(changes)
    return got


def test_the_report_says_each_thing_and_keeps_what_needs_looking_at(jet, tmp_path, monkeypatch, capsys):
    info = jet.this_computer(tree(tmp_path, ORIN, R38), machine="aarch64")
    monkeypatch.setattr(jet, "power_mode", lambda: "MAXN")
    monkeypatch.setattr(jet, "addresses", lambda: [("end0", "192.168.1.50", 24)])
    monkeypatch.setattr(jet, "saved_arm_ips", lambda: {})
    assert jet.report(tmp_path, info, answers(), "installed (/etc/udev/rules.d/99-realsense-libusb.rules)", optional=True) == []
    said = capsys.readouterr().out
    assert "The robot's library (RealMan, ARM64): loads" in said and "Time-optimal optimiser (toppra 0.6.10): built" in said
    assert "no D435 is plugged into this Jetson now" in said
    assert "loads in 4.2 s and answers a frame in 85 ms, on the processor" in said
    assert "Power mode: MAXN\n" in said and "are on end0's network" in said and "**" not in said
    assert "its CUDA libraries (most of the environment's 7 GB) are not used here" in said and "--cpu-torch" in said
    jet.report(tmp_path, info, answers(torch={"ok": True, "version": "2.15.0+cpu", "cuda": False}), "already installed (x)", optional=True)
    assert "CUDA libraries" not in capsys.readouterr().out                           # the processor-only build: nothing to say
    assert "OpenCV 4.13.0: installed (the lid's depth contours, the camera calibration)" in said
    # ALIX's own 5 ms sender, timed on this computer idle: the number behind a stream that "did not complete" (the
    # Jetson, 10 Nov: "the host was 11.4 ms late" under the full test run). Half the streamer's limit is little room.
    assert ("5 ms stream sender, this computer idle (1000 lines to a stand-in arm): 0.11 ms late on average, 1.2 ms at "
            "worst; a stream ends itself at 10 ms") in said
    late = answers(sender={"ok": True, "lines": 1000, "period_ms": 5.0, "mean_ms": 0.9, "worst_ms": 6.3, "late_lines": 2,
                           "ended": None, "limit_ms": 10.0})
    assert jet.report(tmp_path, info, late, "already installed (x)", optional=True) == ["the 5 ms stream sender has little room on this computer"]
    assert "6.3 ms at worst; a stream ends itself at 10 ms: little room" in capsys.readouterr().out
    # OpenCV 5.0, which a fresh install brought until it was pinned, cannot solve the camera calibration (the Jetson's
    # first install, 10 Nov: "this Python's OpenCV (5.0.0) has no calibrateHandEye", 2 tests of the suite)
    five = answers(cv2={"ok": True, "version": "5.0.0", "hand_eye": False})
    assert jet.report(tmp_path, info, five, "already installed (x)", optional=True) == ["this OpenCV cannot solve the camera calibration"]
    assert "** OpenCV 5.0.0 has no calibrateHandEye" in capsys.readouterr().out
    # PyTorch that can use the Jetson's GPU says so; a model that does not answer is kept (the depth answers alone)
    jet.report(tmp_path, info, answers(torch={"ok": True, "version": "2.15.0", "cuda": True}), "already installed (x)", optional=True)
    assert "answers a frame in 85 ms, on the GPU (CUDA)" in capsys.readouterr().out
    silent = answers(colour_model={"ok": False, "why": "RuntimeError: not warmed (no weights)"})
    assert jet.report(tmp_path, info, silent, "already installed (x)", optional=True) == ["the colour model does not answer"]
    assert "does NOT answer (RuntimeError: not warmed (no weights)); the depth answers alone" in capsys.readouterr().out
    # a camera plugged in that streams; another power mode; an address elsewhere
    monkeypatch.setattr(jet, "power_mode", lambda: "30W")
    monkeypatch.setattr(jet, "addresses", lambda: [("wlP1p1s0", "10.0.0.7", 24)])
    cam = {"ok": True, "found": True, "devices": [{"name": "Intel RealSense D435", "serial": "1"}], "said": None,
           "depth": [480, 848], "colour": [720, 1280]}
    issues = jet.report(tmp_path, info, answers(camera=cam), "already installed (x)", optional=True)
    said = capsys.readouterr().out
    assert "its depth 848 x 480 and colour 1280 x 720 arrive" in said and "sudo nvpmodel -m 0" in said
    assert issues == ["no address on the arms' network"]
    # what is missing is said and kept
    monkeypatch.setattr(jet, "addresses", lambda: [("end0", "192.168.1.50", 24)])
    bad = answers(toppra={"ok": False, "why": "ModuleNotFoundError: No module named 'toppra'"},
                  pyrealsense2={"ok": False, "why": "ModuleNotFoundError: No module named 'pyrealsense2'"},
                  robot_library={"ok": False, "why": "OSError: libapi_c.so: cannot open shared object file"})
    issues = jet.report(tmp_path, info, bad, None, optional=True)
    assert issues == ["the robot's library does not load", "the Time-optimal optimiser was not built", "the camera library is not installed"]
    assert "Real refuses their checks" in capsys.readouterr().out
    # installed and not loading is another thing than absent: on Ubuntu 22.04 (JetPack 6) the camera package's ARM64
    # build asks for a newer C library than the system has (the emulated 22.04, 10 Nov)
    old = answers(pyrealsense2={"ok": False, "why": "ImportError: /lib/aarch64-linux-gnu/libm.so.6: version `GLIBC_2.38' not found"})
    jammy = jet.this_computer(tree(tmp_path / "jammy", ORIN, R36, "22.04", "Ubuntu 22.04.5 LTS"), machine="aarch64")
    assert jet.report(tmp_path, jammy, old, None, optional=True) == ["the camera library does not load"]
    said = capsys.readouterr().out
    assert "Wrist camera library: does NOT load (ImportError" in said and "needs Ubuntu 24.04's libraries (glibc 2.38)" in said
    jet.report(tmp_path, info, old, None, optional=True)
    assert "needs Ubuntu 24.04's libraries" not in capsys.readouterr().out            # on 24.04 the reason is another
    jet.check_computer(jammy)
    assert "** Ubuntu 22.04.5 LTS: Lite installs, without the wrist camera" in capsys.readouterr().out
    jet.check_computer(info)
    assert "without the wrist camera" not in capsys.readouterr().out
    assert jet.report(tmp_path, info, None, None, optional=True) == ["the environment does not answer"]
    assert jet.report(tmp_path, info, answers(), None, optional=False) == []        # --no-optional: said, nothing to look at
    assert "left out (--no-optional)" in capsys.readouterr().out


def fake_environment(tmp_path, name: str, script: str) -> Path:
    """A copy whose environment's Python is a shell script: what it prints is what the environment answers."""
    copy = tmp_path / name
    (copy / ".venv/bin").mkdir(parents=True)
    py = copy / ".venv/bin/python"
    py.write_text("#!/bin/sh\n" + script)
    py.chmod(py.stat().st_mode | stat.S_IXUSR)
    return copy


def test_an_answer_that_ends_the_check_leaves_the_ones_before_it(jet, tmp_path):
    """The check asks the environment one thing after another and each answer is said as it comes. One that ends the
    process (a GPU library that does not fit this Jetson) or never comes (the time limit) is marked, with the ones
    after it; the ones before it stand, so the report still says the robot's library loads."""
    said = ('echo \'PROBE {"name": "python", "value": "3.12.3"}\'\necho \'PROBE {"name": "machine", "value": "aarch64"}\'\n'
            'echo \'PROBE {"name": "lite", "value": {"ok": true, "version": "0.1.0", "outside": []}}\'\necho "a library\'s own line"\n')
    got = jet.probe(fake_environment(tmp_path, "crash", said + "exit 139\n"))
    assert got["python"] == "3.12.3" and got["lite"]["ok"] and set(jet.PROBE_ITEMS) <= set(got)
    assert got["numpy"] == {"ok": False, "why": "the check ended before this answered (exit 139)"}
    assert got["colour_model"] == {"ok": False, "why": "not asked: the check had ended"}
    slow = jet.probe(fake_environment(tmp_path, "slow", said + "exec sleep 30\n"), limit_s=1.0)
    assert slow["lite"]["ok"] and slow["numpy"]["why"] == "no answer within 1 s: the check was ended"
    assert jet.probe(fake_environment(tmp_path, "mute", "exit 1\n")) is None      # its Python does not even start
    assert jet.probe(tmp_path / "none") is None                                   # no environment yet
    # the answers the report reads are the ones the check asks, and it asks the GPU's libraries last
    assert all(f'"{name}"' in jet.PROBE for name in ("lite", "robot_library", "arms", "sender", "camera", "torch", "colour_model"))
    assert jet.PROBE_ITEMS[-2:] == ("torch", "colour_model")


def test_cpu_torch_puts_the_processor_only_build_in_before_setup(jet, tmp_path, monkeypatch):
    """PyPI's PyTorch for ARM64 carries CUDA libraries for ARM servers (3 GB to download, 6 GB installed; the emulated
    ARM64 Ubuntu, 10 Nov), which a Jetson's GPU may not be driven by. --cpu-torch installs PyTorch's processor-only
    build into the copy's environment first, so setup's ultralytics finds PyTorch there; an environment that holds
    PyPI's is built again (its CUDA libraries would stay behind); one that holds the processor-only build is left."""
    says = lambda version: f'case "$*" in *metadata*) echo {version};; esac\nexit 0\n'      # noqa: E731
    ran = []
    monkeypatch.setattr(jet, "run", lambda cmd, **kw: ran.append([str(c) for c in cmd]) or 0)
    pypi = fake_environment(tmp_path, "pypi", says("2.15.0"))
    assert jet.torch_version(pypi) == "2.15.0"
    jet.cpu_torch_first(pypi, "/usr/bin/python3.12")
    assert not (pypi / ".venv").exists()
    assert ran == [["/usr/bin/python3.12", "-m", "venv", str(pypi / ".venv")],
                   [str(pypi / ".venv/bin/python"), "-m", "pip", "install", "torch", "torchvision", "--index-url", jet.TORCH_CPU_INDEX]]
    ran.clear()
    lean = fake_environment(tmp_path, "lean", says("2.15.0+cpu"))
    jet.cpu_torch_first(lean, "/usr/bin/python3.12")
    assert not ran and (lean / ".venv/bin/python").exists()
    jet.cpu_torch_first(tmp_path / "new", "/usr/bin/python3.12")                    # no environment yet: made, then filled
    assert [c[1:3] for c in ran] == [["-m", "venv"], ["-m", "pip"]]
    # the copy's install_lite.py runs after it; --no-optional has no colour model, so no PyTorch at all
    ran.clear()
    order = []
    monkeypatch.setattr(jet, "cpu_torch_first", lambda copy, py: order.append("torch"))
    monkeypatch.setattr(jet, "run", lambda cmd, **kw: order.append("install_lite") or 0)
    from types import SimpleNamespace

    a = SimpleNamespace(cpu_torch=True, no_optional=False, branch=None, shortcut=False)
    jet.install(tmp_path / "new", "/usr/bin/python3.12", a)
    assert order == ["torch", "install_lite"]
    order.clear()
    jet.install(tmp_path / "new", "/usr/bin/python3.12", SimpleNamespace(cpu_torch=True, no_optional=True, branch=None, shortcut=False))
    assert order == ["install_lite"]


def test_opencv_is_pinned_before_the_package_that_takes_any():
    """ultralytics takes any OpenCV from 4.6, so a fresh install brought the newest (5.0, without calibrateHandEye).
    The tested version is pinned, and before ultralytics in the file: setup installs the lines in order and pip keeps
    an OpenCV that is already there."""
    lines = [ln.split("==")[0] for ln in (ROOT / "requirements-optional.txt").read_text().splitlines()
             if ln.strip() and not ln.startswith("#")]
    assert "opencv-python" in lines and lines.index("opencv-python") < lines.index("ultralytics")
    pin = next(ln for ln in (ROOT / "requirements-optional.txt").read_text().splitlines() if ln.startswith("opencv-python=="))
    assert pin.split("==")[1].startswith("4.")
    cv2 = pytest.importorskip("cv2")
    assert hasattr(cv2, "calibrateHandEye"), f"this computer's OpenCV {cv2.__version__} cannot solve the camera calibration"


SUMMARY = ("....F.F\\n=========================== short test summary info ============================\\n"
           "SKIPPED [1] tests/test_alix_sync.py:38: ALIX is not here\\n"
           "FAILED tests/test_s2_lid_emulated.py::test_a_lid_task - AssertionError: 47 of 264 lines sent\\n"
           "FAILED tests/test_hand_eye_cal.py::test_solves[board-1] - UserError: no calibrateHandEye - twice\\n"
           "2 failed, 650 passed, 1 skipped in 278.64s\\n")


def test_a_test_that_fails_in_the_full_run_is_run_again_alone(jet, tmp_path, capsys):
    """The Jetson's first install, 10 Nov: 4 of 660 failed, two of them emulated lid runs whose 5 ms stream was fed
    late while 8 workers loaded every core (ALIX's streamer ends such a stream), two a real fault (OpenCV 5.0). The
    failed ones are run again in one process: what passes alone is the full run's load, what does not is real."""
    assert jet.failed_tests(SUMMARY.replace("\\n", "\n")) == ["tests/test_s2_lid_emulated.py::test_a_lid_task",
                                                              "tests/test_hand_eye_cal.py::test_solves[board-1]"]
    assert jet.failed_tests("654 passed, 6 skipped in 270s\n") == []
    full = f"printf '{SUMMARY}'; exit 1;;"
    # all of them pass alone: said, kept as something to look at, and not a failed install
    copy = fake_environment(tmp_path, "late", f'case "$*" in *"-n 0"*) printf "..\\n2 passed in 31s\\n"; exit 0;; *"-m pytest"*) {full} *) exit 0;; esac\n')
    got = jet.run_tests(copy)
    assert got["ok"] and len(got["failed"]) == 2 and got["alone"] == got["failed"] and got["still"] == []
    said = capsys.readouterr().out
    assert "The 2 tests that did not pass, run again alone" in said and "2 of them pass alone" in said and "nvpmodel" in said
    assert "-n 0 -p no:cacheprovider tests/test_s2_lid_emulated.py::test_a_lid_task tests/test_hand_eye_cal.py::test_solves[board-1]" in said
    # one of them fails alone too: that one is real
    again = ("printf 'F.\\nFAILED tests/test_hand_eye_cal.py::test_solves[board-1] - UserError: no calibrateHandEye\\n"
             "1 failed, 1 passed in 12s\\n'; exit 1;;")
    copy = fake_environment(tmp_path, "real", f'case "$*" in *"-n 0"*) {again} *"-m pytest"*) {full} *) exit 0;; esac\n')
    got = jet.run_tests(copy)
    assert not got["ok"] and got["still"] == ["tests/test_hand_eye_cal.py::test_solves[board-1]"]
    assert got["alone"] == ["tests/test_s2_lid_emulated.py::test_a_lid_task"]
    assert "** fails alone too: tests/test_hand_eye_cal.py::test_solves[board-1]" in capsys.readouterr().out
    # a clean run asks nothing again; an environment whose own check fails is not tested at all
    clean = fake_environment(tmp_path, "clean", 'case "$*" in *"-m pytest"*) printf "......\\n654 passed in 270s\\n"; exit 0;; *) exit 0;; esac\n')
    assert jet.run_tests(clean) == dict(ok=True, failed=[], alone=[], still=[])
    broken = fake_environment(tmp_path, "broken", "exit 1\n")
    assert not jet.run_tests(broken)["ok"]


def test_the_suite_runs_on_a_worker_for_every_two_cores(jet):
    """The suite's own 8 workers made one or two 5 ms streams late in every full run on the Jetson's 12 cores (each
    emulated-arm test runs a Lite server and two arm processes beside its worker). There it runs on 6; a PC with 16
    cores or more keeps the suite's own 8."""
    assert [jet.suite_workers(n) for n in (12, 16, 24, 8, 4, 2)] == [6, 8, 8, 4, 2, 2]


def test_an_environment_of_another_python_is_built_again(jet, tmp_path):
    """install_lite.py accepts Python 3.12 to 3.14 and updates an environment in place; on the Jetson one built with
    3.13 or 3.14 cannot hold the camera package, so it is removed before setup runs."""
    def copy_with(version: str) -> Path:
        copy = tmp_path / version
        (copy / ".venv/bin").mkdir(parents=True)
        py = copy / ".venv/bin/python"
        py.write_text(f"#!/bin/sh\necho {version}\n")
        py.chmod(py.stat().st_mode | stat.S_IXUSR)
        return copy

    other, same = copy_with("3.13"), copy_with("3.12")
    jet.drop_other_environment(other)
    jet.drop_other_environment(same)
    assert not (other / ".venv").exists() and (same / ".venv/bin/python").exists()


def test_the_disk_is_checked_before_a_clone(jet, tmp_path, monkeypatch):
    from types import SimpleNamespace

    usage = lambda free: (lambda path: SimpleNamespace(free=free * 1e9))             # noqa: E731
    monkeypatch.setattr(jet.shutil, "disk_usage", usage(8))
    with pytest.raises(SystemExit) as stop:
        jet.check_disk(tmp_path / "new/alix_studio_lite", new=True)
    assert "8 GB free, about 26 GB needed" in str(stop.value) and "--dir" in str(stop.value)
    with pytest.raises(SystemExit):                                                  # an existing copy: its environment alone
        jet.check_disk(tmp_path / "new/alix_studio_lite", new=False)
    assert "--cpu-torch needs 8 GB less" in str(stop.value)
    monkeypatch.setattr(jet.shutil, "disk_usage", usage(14))
    jet.check_disk(tmp_path / "new/alix_studio_lite", new=False)
    monkeypatch.setattr(jet.shutil, "disk_usage", usage(8))                           # the processor-only PyTorch: 8 GB less
    jet.check_disk(tmp_path / "new/alix_studio_lite", new=False, cpu_torch=True)
    with pytest.raises(SystemExit) as stop:
        jet.check_disk(tmp_path / "new/alix_studio_lite", new=True, cpu_torch=True)
    assert "8 GB free, about 18 GB needed" in str(stop.value)
    monkeypatch.setattr(jet.shutil, "disk_usage", usage(300))
    jet.check_disk(tmp_path / "new/alix_studio_lite", new=True)


def test_it_runs_with_any_python3_of_a_jetson_and_the_standard_library_alone(jet):
    """JetPack 5's python3 is 3.8 and JetPack 6's 3.10: the installer itself must start with them (it then finds or
    installs 3.12 for Lite), with nothing to install first."""
    source = SCRIPT.read_text(encoding="utf-8")
    module = ast.parse(source, feature_version=(3, 8))
    names = set()
    for node in ast.walk(module):
        if isinstance(node, ast.Import):
            names |= {a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            names.add(node.module.split(".")[0])
    assert names <= set(sys.stdlib_module_names) | {"__future__"}, names - set(sys.stdlib_module_names)
    assert jet.PY == (3, 12) and jet.own_checkout() == ROOT
    # the environment's own questions are asked with the environment's Python (3.12), so they may use its syntax
    ast.parse(jet.PROBE)
