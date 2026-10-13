# Build status

Last updated: 13 October 2026.

**Order of work (decision D27):** UI/UX first. The backend is on hold at the checkpoint below.

## Install and start Lite

Install once, on any platform. This builds `.venv` in the Lite folder from `requirements.txt`, then runs a check (see the README):

```bash
cd alix_studio_lite
python3 tools/setup_lite.py         # Windows: py tools\setup_lite.py
```

Then Lite starts the same way as ALIX Studio. The launcher uses `.venv` by itself:

```bash
cd alix_studio_lite/tools
./run_lite.sh                  # opens a browser tab at http://127.0.0.1:8766/   (Ctrl+C to stop)
./run_lite.sh --no-browser     # or --port 8800, --data DIR, --fixture commode_b
```

- Port 8766 is used, so Lite can run beside ALIX Studio (8765).
- Project data lives in `~/ALIX-Lite/` (outside the repository).
- The first start creates the **Commode C (demo)** project. It holds the commode C test recordings, assigned to 8 left-arm regions.
- The robot is a **stand-in** until robot I/O is built: every robot screen works and nothing moves. A grey line under the top bar says so.
- The engineer PIN is `0000`; it is stored in `~/ALIX-Lite/settings.json`.

## UI/UX (in progress, second iteration working)

The front end is in `src/alix_studio_lite/web/` (no build step). The local server is in `src/alix_studio_lite/server/`. It is built on ALIX Studio's design tokens, brand mark, Plotly 2.35.2 and three.js r128, all vendored.

The second iteration answers the first review (13 Oct). It was checked with headless-Chrome screenshots at 1024, 1280, 1366 and 1920 px wide, in light and dark themes. No screen shows a console error, and nothing scrolls sideways at any of those widths.

| Area | State |
|---|---|
| Layout (D32) | **Tabs at the top**, as in ALIX Studio: Regions · Teach · Optimise · 3D view · Run; Robot and Settings for engineers; Help. The current region is chosen at the left of the tab bar. Step tabs show ✓ when done and 🔒 when not yet open.<br>The top bar and every screen reflow below 1440, 1280, 1100, 1024 and 900 px. Plots and the 3D view follow their container's size, can be dragged taller, and plots have **Full size**. |
| Fixture pool (D29) | Click the fixture name under the title to choose commode A, B or C, urinal A or wash basin A. Commode C is the default.<br>Each fixture has its own project. Its parts are placed with ALIX's own `scene_pose`, including the venue's yaw 180°: commode C's seat hinge sits at x = 0.89 m, as ALIX measured on the robot.<br>Fixtures other than commode C get a provisional region list from ALIX's `regions.yaml`. |
| Modes (D30) | **Visual sim**: the robot moves in the 3D view, with no hardware.<br>**Hardware sim**: the controller's SIM mode (`rm_set_arm_run_mode(0)`); the arms stay still.<br>**Real**: the arms move, after a confirmation. |
| Existing recordings (D31) | A new project uses the fixture's library recordings for their regions by default. For commode C that is 8 of the commode C taught set (`assets/recordings/commode_c.yaml`).<br>Teach shows where the recording came from. **Use another file…** picks one from the library (same arm only) or uploads a `.txt`; every line is checked. Teaching again stays available. |
| 3D view | The robot is built the way ALIX Studio builds its overlay. Its meshes come from ALIX's own bake tool, run on the current URDF (`tools/bake_robot_overlay.py` → `assets/robot/visual_overlay_baked.npz`: 1.40 M → 357 k triangles, every reduction within ALIX's 1.5 mm check, real CAD colours). Any link missing from the bake uses its full-resolution mesh. Every pose comes from ALIX's FK on the server (Studio's one-FK rule), and the browser places each link.<br>The scene also shows the fixture in the step's lid/seat state and the cubicle walls.<br>Every joint can be moved with the joint panel (Home, Zero arms).<br>A region's version or recording plays on the robot (Visual sim), with a joint plot and playhead.<br>The 3D check verdict is still a **stand-in** until the 3D check backend exists. |
| Optimise | Default or Time-optimal, a target time, undo and redo.<br>The closest safe time is shown to the hundredth, with **Use X s**. L1–L6 are shown with values. Joint and limit-use plots.<br>**Speed of parts of the motion**: drag across the recording to pick a part, then set its time % (Default) or speed cap (Time-optimal), as an undoable step. |
| Run | Visual sim: **Play in the 3D view**.<br>Hardware sim and Real: send to a program number 1–100 (with a replacement confirmation), run, progress, verdict. |
| Robot console, Settings, Help | As in the first iteration, with the new mode names. |

UI tests:
- `tests/test_server_api.py` takes one region through every step against the stand-in. It covers the E-stop interlock, the confirmations, the fixture switch, the library and upload imports, and refusal of a bad file.
- `tests/test_fixtures.py` pins fixture placement.
- `tests/test_robot_model.py` covers the robot model.
- `tests/test_ui_theme.py` keeps the ALIX theme rules.

Next for the UI:
1. Review this iteration with the team, then a pilot with operators (NFR-UX).
2. The Program screen for Release 2: Page 02 sequences with TOGETHER / RUN FIRST / WAIT OTHER.
3. Both arms playing together in the 3D view, for TOGETHER pairs.

## Done (backend)

| Area | State |
|---|---|
| Self-contained repository | Done. The ALIX subset is vendored in `vendor/` with a manifest. 52 tests pass in about 5 s, with no ALIX checkout. |
| Cubicle walls | Done. The 1400 mm cubicle (default) and the 800 mm cubicle are in `assets/scene/cubicles/`. |
| Motion basics (`src/alix_studio_lite/motion/`) | Implemented and smoke-tested. It covers track read/write, the 5 ms clock, the limit set from the URDF, and the hard limits L1–L6 plus checks C1–C4 on the integer bytes. |
| Optimiser engine (`src/alix_studio_lite/optimise/`) | Implemented and smoke-tested. **On hold.** |

The optimiser engine covers:

- the **default** optimiser: the prototype's algorithm with fixes F-a, F-b and F-c;
- the **time-optimal** optimiser: ALIX's TOPP-RA chain, copied, plus an end-pose pin;
- the shared target-time search with the binding-limit report;
- TOGETHER pairs;
- the `.lite.json` record;
- a command line: `python3 -m alix_studio_lite.optimise`.

### Smoke results

All times are true time: a file of N lines lasts (N − 1) × 5 ms. Every hard limit passed on these runs.

| Recording | Optimiser | Result |
|---|---|---|
| FILE_10.1 (lid seat open) | Time-optimal | Fastest safe time 4.275 s (856 lines). The 3.0 s target is refused. Limited mostly by joint 6 acceleration, 39 % of the time. The research measured 4.26 s and 38 %. |
| FILE_10.1 | Default | Fastest safe time 7.73 s. |
| FILE_01.1 (lid top) | Default | A 60 s target is met exactly (12,001 lines). The fastest safe time is 38.05 s; the research measured 38.30 s. |

## Not done yet (backend, on hold)

1. Golden tests for the optimiser (G-T1–G-T4, G-D1–G-D5, G-X1–G-X4 in `docs/analysis/research/research_optimisers.md` §6.3). None are written yet.
2. The 3D check with walls: an every-row sweep against the commode C scene and the cubicle.
3. Robot I/O:
   - runner v14 wrapper and upload to any ID 1–100;
   - the IO3 E-stop monitor (mode 5);
   - STOP, pole and hand.
4. The local server and API that the UI will call.

## Findings to carry forward

- **G5 cannot be a hard limit.** ALIX's predicted-departure model reads 33–109 mm on the raw commode C recordings themselves, which ran fine on the robot. Lite shows it as a warning (C1). A region's MAX DEV tightens the path-accuracy limit (L5) instead.
- **TOPP-RA moved the end pose.** ALIX's 30 mm arc-length smoothing moves the last pose by up to 0.06°. Lite pins both ends back to the recording over the last 30 mm, so L6 (0.05°) holds.
- **Controller speed envelope (for robot I/O).** The controller's TCP speed settings default to 0.25 m/s. ALIX raised them to 1.5 m/s for its replays and restored them after (`vendor/alix/.../alix_dispatch/limits.py`, `guard`). Runner v14 never touches them. TOPP-RA files move the hand at up to about 0.9 m/s. This must be checked on the robot before fast files are run.

## How to resume the backend

```bash
cd alix_studio_lite
export PYTHONPATH=src
python3 -m alix_studio_lite.optimise tests/data/recordings/commode_c_taught_paths/FILE_10.1.txt \
    --side left --optimiser toppra --target 3.0
python3 -m pytest -q
```

Start with the golden tests (item 1), then the 3D check (item 2).
