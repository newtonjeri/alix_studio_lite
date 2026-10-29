"""Test setup: run Lite from this repository only (no ALIX checkout, no installed package)."""
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
os.environ.pop("ALIX_REPO_ROOT", None)          # vendor_paths forces its own value anyway
# The robot's simulated time runs 10 times faster than the wall clock in the tests: the stand-in's
# (robot_standin.TIME_SCALE) and ALIX's emulated arms' (RM_EMU_TIME_SCALE, also in the servers the
# tests start). Every step is still taken; a 15 s program plays in 1.5 s.
os.environ.setdefault("LITE_STANDIN_TIME_SCALE", "10")
os.environ.setdefault("RM_EMU_TIME_SCALE", "10")
# The servers the tests start do not link Lite's lid and seat files with ALIX's at their start (alix_sync.link): they
# would copy a change the owner is making in ALIX into this repository in the middle of the suite.
os.environ["LITE_START_LINK"] = "0"

from alix_studio_lite import vendor_paths  # noqa: E402

vendor_paths.activate()

DATA = ROOT / "tests" / "data"
RECORDINGS = DATA / "recordings"


@pytest.fixture(scope="session")
def lite_root() -> Path:
    return ROOT


@pytest.fixture(scope="session")
def commode_c_recordings() -> Path:
    return RECORDINGS / "commode_c_taught_paths"


@pytest.fixture(autouse=True, scope="session")
def _quick_pins():
    """The tests' PINs are hashed the same way with fewer PBKDF2 rounds (each test server logs in; at
    Lite's 200,000 rounds that was about 0.3 s a server). A stored record keeps its own count."""
    from alix_studio_lite.server import auth

    auth.ITERATIONS = 2_000
    yield
