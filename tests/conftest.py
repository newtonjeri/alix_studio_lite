"""Test setup: run Lite from this repository only (no ALIX checkout, no installed package)."""
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
os.environ.pop("ALIX_REPO_ROOT", None)          # vendor_paths forces its own value anyway

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
