#!/usr/bin/env bash
# Launch ALIX Studio Lite (as tools/run_studio.sh launches ALIX Studio).
#
#   tools/run_lite.sh                 # opens a browser tab at http://127.0.0.1:8766/
#   tools/run_lite.sh --no-browser
#   tools/run_lite.sh --port 8800
#
# A shim: the launcher is run_lite.py, so Lite also starts on any platform Python runs on.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${PYTHON:-python3}" "$ROOT/tools/run_lite.py" "$@"
