# Build status

Last updated: 13 October 2026.

**Order of work (decision D27):** UI/UX first. The backend is on hold at the checkpoint below.

**Front ends (13 October, later):** the designer's *ALIX Studio LITE Edition V19* page is now the primary UI (the **Lite UI**, at `/`), and the backend is built around it. The earlier UI is **Legacy Lite**: it is kept working at `/legacy/`, and git tag `legacy-lite-0.1` marks it. What the V19 page needs, what is connected, the defects and gaps found, and the open decisions are in [`UI_V19_CONTRACT.md`](UI_V19_CONTRACT.md).

## Install and start Lite

Install once, on any platform. This builds `.venv` in the Lite folder from `requirements.txt`, then runs a check (see the README):

```bash
cd alix_studio_lite
python3 tools/setup_lite.py         # Windows: py tools\setup_lite.py
```

Then Lite starts the same way as ALIX Studio. The launcher uses `.venv` by itself:

```bash
cd alix_studio_lite/tools
./run_lite.sh                  # the Lite UI at http://127.0.0.1:8766/   (Ctrl+C to stop)
./run_lite.sh --ui legacy      # Legacy Lite at http://127.0.0.1:8766/legacy/
./run_lite.sh --no-browser     # or --port 8800, --data DIR, --fixture commode_b (Legacy Lite)
./run_lite.sh --set-pin engineer   # change the operator or engineer PIN
```

- Port 8766 is used, so Lite can run beside ALIX Studio (8765).
- Data lives in `~/ALIX-Lite/` (outside the repository):
  - the Lite UI's projects in `projects/<id>/`;
  - Legacy Lite's projects in `legacy/projects/<fixture>/`. The first start of this version moves them there from `projects/`.
- The operator PIN (log-in) and the engineer PIN (Advanced Robot Controls, and Legacy Lite's engineer role) are both `1234` until changed. They are stored hashed in `~/ALIX-Lite/auth.json`.
- Legacy Lite's first start creates the **Commode C (demo)** project. It holds the commode C test recordings, assigned to 8 left-arm regions.
- The robot is a **stand-in** until robot I/O is built: every robot screen works and nothing moves. The Lite UI's status pill and Legacy Lite's grey line under the top bar say so.

## Lite UI (V19): shell and region loop working

The designer's page is served unchanged from `src/alix_studio_lite/web/index.html`. Lite's bridge is `web/lite/` (no build step). The API is `/api/*`:
- `server/api_v2.py`: log-in, projects, robots;
- `server/api_units.py`: tasks and arm steps;
- `server/lite_units.py`: each arm step's files in the project.

What is connected, screen by screen, is in [`UI_V19_CONTRACT.md`](UI_V19_CONTRACT.md).

**Working:**
- **Shell.**
  - Log-in and the engineer PIN are checked server-side: hashed, with a lock-out after five wrong tries.
  - Projects are folders, with duplicate, trash, and `.alixproj` export and import, and auto save to the server. A project has a fixture (D29).
  - STOP BOTH and SLOW STOP need no log-in.
  - The status pill opens Robot & mode (D30). It covers Visual sim, Hardware sim and Real, connecting each arm, and each arm's E-stop with Recover.
- **Task queue (D35).** V19's 22 tasks are tied to the region catalogue: codes, TASK 9 as the deck has it, each step's status. The 8 steps outside Release 1 are LATER and cannot be chosen.
- **Region loop**, against the robot stand-in:
  - Prepare: each region's setup (pole height and hand pose); Apply stages it on the robot, the pole and then the hand, each read back.
  - Drag & Teach: the existing recording is offered first (D31); drag teaching is in Hardware sim or Real, and starts only with the arm in the region's setup.
  - Save & Review: every recording is listed with its arm and stage, one is active, and a loaded file never replaces the new one. A loaded file needs its arm and its pole and hand; a file of the other arm is refused. Playback runs in 3D or on the robot, staging first.
  - Tune + Simulation:
    - V19's Trim, Mean Smooth (41 by default, R11) and Scale regions are the region's server-side settings, with undo.
    - Rows marked with Trim Selected are cut, and the gap is crossed by a short joint move that is then smoothed.
    - Optimiser: Default or Time-optimal (D24). Total time: the recorded pace, a target, or as fast as safe.
    - A result strip shows the gates (arm, stage, limits, robot) and L1–L6.
    - The real recording is on the plot. ▶ Start stages the pole and the hand in 3D, moves the arm to the start, then plays the trajectory on the 5 ms clock, with V19's playhead on the recording's rows.
  - Real Test:
    - The kept version uploads to a Program ID 1–100, checked by the server; replacing a program needs a confirmation.
    - It runs in Hardware sim or, after a confirmation, in Real, after the pole and the hand are staged and read back; if either fails, nothing else runs and the reason is shown.
    - PASS needs a completed run of the current version. After one arm passes, V19 opens the other arm.
  - Advanced Robot Controls: SIM, REAL and CHECK; hand; pole; the program list.
- **Merge + Full Sequence: the full simulation in 3D** (in place of the page's picture):
  - each included row of the flow table runs its regions' kept versions, each arm continuing from where its previous step left it: the pole, the hand, the move to the first line, then the trajectory on the 5 ms clock;
  - the row's rule (TOGETHER, LEFT/RIGHT RUN FIRST, ONE BY ONE, PROCEDURE) and checkpoint wait; steps outside Release 1 are skipped;
  - Start is refused until every included arm step has a kept version, each listed with what it needs; a preview with the recordings as taught plays what exists and never approves the sequence;
  - play, pause, speed and a time slider; the arm panels say what each arm is doing (staging, waiting, running).
- **ALIX's brand mark**, as ALIX Studio draws it, in place of the page's "AX" tiles, in both themes and as the tab's icon.
- **Trajectory Viewer** (in the Operator Journey, on Home, and in Save & Review):
  - Taught `.txt` files are uploaded one by one or as a folder. Each is checked line by line and kept in `~/ALIX-Lite/library/`, beside the built-in sets: the commode C taught set and the legacy V6 package.
  - The arm comes from the name or folder (`L_`, `LEFT/`, `R_`, `RIGHT/`), else the operator chooses it. The operator states the pole height and hand pose each upload was taught with; the built-in files carry their record's.
  - A LEFT and a RIGHT file replay as the robot runs them: each arm stages its pole and hand, then moves to its start, and both trajectories start together on the 5 ms clock. Play, pause, speed, a time slider and the toilet state; the joints on a plot with a playhead.
  - Each file is checked against L1–L4 as taught. For example, V6's `L_P01P02` reaches 1,543 °/s² on joint 6 (20 Hz band-limited), against the 600 °/s² limit.
  - "Use for <task> · <region>" makes a file the active recording of a region of its own arm, ready for Save & Review and tuning.
- **The gates on every execution** (contract, "The gates on every execution"), checked by the server in the 3D simulation, Hardware sim and Real:
  - a trajectory runs only on the arm it was taught on;
  - its pole height and hand pose are staged first and read back, and nothing runs if either fails;
  - a run needs a version that passes L1–L6, with L3 read as ALIX reads it (20 Hz band-limited);
  - the robot's own rules (mode, confirmation, E-stop on either arm).
- **Fixed in the bridge:** six defects of the page (contract, D1–D6). The most serious was STOP BOTH leaving the screen below 1,610 px wide; the widest was the dark theme, which covered only part of the page; and the full simulation was a picture with a clock that could approve the real full run.

**Checked:**
- Tests: `tests/test_lite_ui.py` (the page is the designer's file plus one block), `tests/test_api_v2.py`, `tests/test_lite_region_loop.py` (one arm step through the whole loop over HTTP, the gates, staging failures, the E-stop interlock, cuts and the recorded-pace mode), `tests/test_library.py` (the Trajectory Viewer's files), `tests/test_staging.py` (staging times and FK, L3 against ALIX's own reading), `tests/test_sequence.py` (the full sequence's schedule and chained steps).
- Headless Chrome:
  - the whole loop, in light and dark themes, with no console errors;
  - STOP BOTH visible from 900 to 1920 px wide.

**Not yet:**
- Merge + Full Sequence: the custom merge with joins, the single-arm merged simulation (Simulate LEFT / RIGHT) and the real full run are still the template.
- The rest of Advanced Robot Controls: uploading any file, stored and multi runs, Global Points, the arm's own drag teach.
- Stored start poses.
- The 3D collision check.
- Everything that needs the real robot I/O.

## Legacy Lite (UI iteration 2, kept at /legacy/)

The front end is in `src/alix_studio_lite/web_legacy/` (no build step). The local server is in `src/alix_studio_lite/server/`. It is built on ALIX Studio's design tokens, brand mark, Plotly 2.35.2 and three.js r128, all vendored.

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

Legacy Lite is no longer developed; its working parts (3D robot, plots, optimiser screens) are being carried into the Lite UI.

## Done (backend)

| Area | State |
|---|---|
| Self-contained repository | Done. The ALIX subset is vendored in `vendor/` with a manifest. 120 tests pass in about two minutes, with no ALIX checkout. |
| Cubicle walls | Done. The 1400 mm cubicle (default) and the 800 mm cubicle are in `assets/scene/cubicles/`. |
| Motion basics (`src/alix_studio_lite/motion/`) | Implemented and smoke-tested. It covers track read/write, the 5 ms clock, the limit set from the URDF, and the hard limits L1–L6 plus checks C1–C4 on the integer bytes. L3 reads acceleration as ALIX does (20 Hz band-limited, F99c; ALIX accepts at 1.00 of 600 °/s², Lite at 1.02). |
| Optimiser engine (`src/alix_studio_lite/optimise/`) | Implemented, smoke-tested and connected to the Lite UI's Tune + Simulation. Golden tests still to write. |

The optimiser engine covers:

- the **default** optimiser: the prototype's algorithm with fixes F-a, F-b and F-c;
- the **time-optimal** optimiser: ALIX's TOPP-RA chain, copied, plus an end-pose pin;
- the shared target-time search with the binding-limit report;
- TOGETHER pairs;
- the `.lite.json` record;
- a command line: `python3 -m alix_studio_lite.optimise`;
- for the Lite UI (13 Oct):
  - **cuts**: rows removed before tuning, with each gap crossed by a straight joint move at 20 °/s and then smoothed, so the slowdown stays local. The cut recording is the reference for L5 and L6.
  - **the recorded pace**: no target, the recording's own timing with the Scale regions. The engine's default is still "as fast as safe".
  - A FILE_08 example: recorded 41.72 s; the recorded pace with smoothing 41 gives 38.79 s; as fast as safe, 27.16 s. With smoothing off, the fastest safe is 276 s: the recorder's ripple (on joint 6, every third or fourth line is a double step) is above 20 Hz at the recorded pace, but a slowed file brings it into the band, where it is real commanded vibration.

### Smoke results

All times are true time: a file of N lines lasts (N − 1) × 5 ms. Every hard limit passed on these runs. Re-measured on 13 October with L3 read band-limited, as ALIX reads it; the earlier windowed reading let short peaks through, which made some times shorter.

| Recording | Optimiser | Result |
|---|---|---|
| FILE_10.1 (lid seat open) | Time-optimal | Fastest safe time 4.61 s (923 lines). The 3.0 s target is refused. Limited by joint 3 acceleration around 2.4 s. (Windowed reading: 4.275 s; the research measured 4.26 s.) |
| FILE_10.1 | Default | Fastest safe time 8.03 s, limited by joint 6 acceleration. |
| FILE_01.1 (lid top) | Default | A 60 s target is met exactly (12,001 lines). The fastest safe time is 37.90 s; the research measured 38.30 s. |

## Not done yet (backend, on hold)

1. Golden tests for the optimiser (G-T1–G-T4, G-D1–G-D5, G-X1–G-X4 in `docs/analysis/research/research_optimisers.md` §6.3). None are written yet.
2. The 3D check with walls: an every-row sweep against the commode C scene and the cubicle.
3. Robot I/O:
   - runner v14 wrapper and upload to any ID 1–100;
   - the IO3 E-stop monitor (mode 5);
   - STOP, pole and hand.

   The robot stand-in already follows these rules behind the Lite UI's API, so the real backend replaces it there.
4. ~~The local server and API that the UI will call.~~ Done: `/api/*` for the Lite UI, `/api/lite/*` for Legacy Lite.

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
