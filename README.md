# ALIX Studio Lite

A guided application for **non-technical operators** to teach, optimise, check and run drag-taught
cleaning trajectories on the dual-arm RealMan RM75 robot (commode C). It reuses ALIX's proven code
and drives the robot through the existing control-program runner.

It has two front ends on one backend:
- the **Lite UI**, at `/`: the designer's *ALIX Studio LITE Edition V19* page, connected to the backend (see [`docs/UI_V19_CONTRACT.md`](docs/UI_V19_CONTRACT.md));
- **Legacy Lite**, at `/legacy/`: the earlier ALIX-themed UI, kept working (git tag `legacy-lite-0.1`).

**This repository is self-contained.** Everything Lite needs is inside this folder; nothing is read
from an ALIX checkout or from the surrounding workspace at run time.

- Requirements: [`docs/ALIX_Studio_Lite_Requirements_Revised.md`](docs/ALIX_Studio_Lite_Requirements_Revised.md) (final, v1.6)
- Implementation plan: [`docs/ALIX_Studio_Lite_Implementation_Plan.md`](docs/ALIX_Studio_Lite_Implementation_Plan.md) (v1.6)
- Build status: [`docs/STATUS.md`](docs/STATUS.md)
- Lite UI contract (V19 page ↔ backend): [`docs/UI_V19_CONTRACT.md`](docs/UI_V19_CONTRACT.md); the designer's files are in `docs/reference/ui_v19/`
- Region catalogue for sign-off: [`docs/region_catalogue_commode_c.csv`](docs/region_catalogue_commode_c.csv)

## Install (one command, any platform)

```bash
cd alix_studio_lite
python3 tools/setup_lite.py          # Windows: py tools\setup_lite.py   (or tools/setup_lite.sh, tools\setup_lite.bat)
```

This takes a few minutes the first time. It:
- builds a virtual environment, `.venv`, inside this folder;
- installs the pinned packages of [`requirements.txt`](requirements.txt), which all have ready-made wheels for Linux x86-64 and aarch64 (Jetson), Windows and macOS, so nothing compiles;
- tries the optional packages of [`requirements-optional.txt`](requirements-optional.txt), and says if one cannot be installed;
- installs Lite itself, then checks that it works (imports, then the test suite).

You need Python 3.11 or newer with its `venv` module (on Ubuntu: `sudo apt install python3-venv`) and internet access for the first install. No ROS and no ALIX checkout. Re-running setup updates the environment in place, and `--check` only checks it.

The optional packages are:
- **toppra**, for the Time-optimal optimiser. PyPI has a wheel for Linux x86-64 only; elsewhere setup builds it from its git tag, which needs git and a C compiler. Without it Lite offers the Default optimiser only.
- **meshoptimizer** and **rtree**, only for re-baking the robot's 3D overlay (`tools/bake_robot_overlay.py`) when the robot model changes. The bake itself is kept in `assets/robot/`.

## Start Lite

Lite starts the same way as ALIX Studio:

```bash
cd alix_studio_lite/tools
./run_lite.sh                  # opens the Lite UI at http://127.0.0.1:8766/   (Ctrl+C to stop)
./run_lite.sh --ui legacy      # opens Legacy Lite at http://127.0.0.1:8766/legacy/ instead
./run_lite.sh --no-browser     # also: --port 8800, --data DIR, --fixture commode_b (Legacy Lite)
./run_lite.sh --set-pin operator   # change a PIN (operator or engineer); asks twice, then exits
```

On Windows: `py tools\run_lite.py`, or double-click `tools\run_lite.bat`.

- The launcher runs inside `.venv` by itself, so there is no activation step, and it clears `PYTHONPATH` so a sourced ROS workspace cannot interfere. Its first lines name the Python in use.
- Port 8766 lets Lite run beside ALIX Studio (8765).
- Data lives in `~/ALIX-Lite/`, outside this folder:
  - the Lite UI's projects in `projects/<id>/`, one folder per project in the layout the handoff recommends;
  - Legacy Lite's projects in `legacy/projects/<fixture>/` (moved there once from `projects/`, nothing deleted).
- Log in to the Lite UI with the operator PIN; Advanced Robot Controls ask for the engineer PIN. Both are `1234` until changed. They are stored hashed in `~/ALIX-Lite/auth.json`, and Legacy Lite's engineer role uses the same engineer PIN.
- Until the robot I/O backend is built, the robot is a **stand-in**: every robot screen works and nothing moves. The status pill in the top bar says so.
- STOP BOTH works without a log-in.

What works, and what comes next, is in [`docs/STATUS.md`](docs/STATUS.md).

## Layout

| Folder | What it holds |
|---|---|
| `src/alix_studio_lite/` | Lite's own code:<br>• `motion/` (tracks, the 5 ms clock, limits, checks);<br>• `optimise/` (the two optimisers);<br>• `server/` (the local web server, projects, log-in, the robot stand-in);<br>• `web/` (the Lite UI: the designer's page and `lite/`, its bridge to the backend);<br>• `web_legacy/` (Legacy Lite).<br>`vendor_paths.py` makes the vendored code run from this folder. |
| `vendor/alix/` | The parts of the ALIX repository Lite reuses, in ALIX's own folder layout. See the table below. |
| `vendor/control_app/` | The upload-channel control program: runner v14, `hand.py`, `pole.py`, `usage.py`. The v53 UI is under `reference/`. |
| `vendor/MANIFEST.json` | Every vendored file with its source path, sha256 and the ALIX commit it came from. |
| `vendor/PATCHES.md` | The patches applied to vendored files, and the runtime adjustments. |
| `assets/scene/cubicles/` | The cubicle wall models: **1400 mm (default)**, built by `tools/make_cubicle.py`, and 800 mm (ALIX's standalone cubicle). |
| `assets/regions/` | The regions deck `Toilet_regions_V1.pptx` (commode C), its pictures (`deck/`), and `commode_c_catalogue.json` (built from the sign-off CSV by `tools/build_catalogue.py`). |
| `assets/package.xml` | Names the asset folder `alix_studio_lite`, so `package://alix_studio_lite/...` URIs resolve. |
| `docs/` | Requirements, plan, the region catalogue, `reference/` inputs and `analysis/` evidence notes. |
| `tests/` | Tests, plus `tests/data/` (taught recordings, the legacy V6 package). |
| `tools/` | `setup_lite.py` (install, one command), `run_lite.sh` / `run_lite.py` (start Lite), `bake_robot_overlay.py` (ALIX's overlay bake), `vendor_from_alix.py` (refresh the vendored copy), `make_cubicle.py` (build a cubicle model) and `build_catalogue.py` (CSV to catalogue JSON). |

**What `vendor/alix/` contains:**

| Area | What was copied |
|---|---|
| Kinematics and collision | `alix_workspace` (the copy loads the excluded reachability module only on demand) and the light `alix_taskgen` modules (`body_collision`, `scene_pose`, …) |
| Robot I/O | `alix_dispatch`, the emulator and the RealMan SDK (`Robotic_Arm`, Linux x86-64 and aarch64 libraries) |
| Taught-track tools | `taught_tracks`, `plan_min_time`, `dual_arm_verdicts`, `cleaning_tracks`, and the exp01 TOPP-RA chain (`local_speed`, `emit_toppra_rung`) |
| Robot model | The URDF, every mesh it references, the SRDF and joint configs |
| Scene | The commode C scene with its body, lid and seat meshes |
| Studio web sources | `web_editor/`, kept as reference for the Lite UI |
| Design | The ALIX design tokens |

## Run the tests

```bash
cd alix_studio_lite
.venv/bin/python -m pytest -q        # Windows: .venv\Scripts\python -m pytest -q   (about 25 s; no robot, no network)
```

ROS 2's own pytest plugins are blocked in `pyproject.toml`, so the tests also run with a ROS workspace sourced.

The tests check that:
- the vendored code imports from `vendor/` only;
- the files match the manifest;
- the cubicles have the intended geometry;
- every robot mesh resolves inside this folder;
- the commode C collision world builds with each cubicle;
- ALIX's TOPP-RA chain reproduces the reference result (commode C lid top: 3839 lines, 19.19 s);
- the web API takes one region through every step against the robot stand-in, including the E-stop interlock and the confirmations;
- the Lite UI page is the designer's file plus one marked block, every page function the bridge replaces still exists, and nothing loads from the network;
- the Lite UI's API: PINs hashed and locked out after five wrong tries, projects as folders (duplicate, trash, ZIP round trip, unsafe ZIPs refused), STOP without a log-in;
- the Trajectory Viewer's library: arm from names and folders, uploads checked and kept once, the built-in sets, 3D frames, the limits as taught, and using a file for a region;
- the Lite UI's region loop over HTTP: the task queue as the deck has it, an existing recording used by default, tuning with every limit checked, keeping a version, upload with a replace confirmation, Hardware sim and Real runs, PASS only after a completed run, the E-stop interlock, and the optimiser's cuts and recorded-pace mode;
- Legacy Lite keeps the ALIX theme: ALIX's tokens unchanged, no colour literals outside them, nothing loaded from the network.

## Refresh the vendored copy

```bash
python3 tools/vendor_from_alix.py --alix ~/alix_ws/src/alix          # add --dry-run to preview
```

The script overwrites `vendor/`, rewrites `vendor/MANIFEST.json` with the new ALIX commit, and re-applies the patches in `vendor/PATCHES.md`. Run the tests afterwards.

## Cubicle models

```bash
python3 tools/make_cubicle.py 1400      # writes assets/scene/cubicles/cubicle_1400mm.{stl,yaml}
```

The rule matches ALIX's standalone planning-scene cubicles:
- a back wall and two side walls, each 0.1 m thick and 2 m high;
- side walls about 1.98 m deep;
- the back wall's inner face 1.215 m in front of `base_link` (yaw 180°).

The tests check that the rule reproduces ALIX's 1000 mm cubicle exactly.
