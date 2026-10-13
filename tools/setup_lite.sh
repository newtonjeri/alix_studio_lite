#!/usr/bin/env bash
# Install ALIX Studio Lite (a shim: the installer is setup_lite.py, which also runs on Windows
# as  py tools\setup_lite.py ).
#
#   tools/setup_lite.sh              # make .venv, install, check
#   tools/setup_lite.sh --check      # only check
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${PYTHON:-python3}" "$ROOT/tools/setup_lite.py" "$@"
