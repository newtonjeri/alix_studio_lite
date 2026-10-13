# Lite UI (V19) — front-end contract

Last updated: 13 October 2026.

The Lite UI is the designer's **ALIX Studio LITE Edition V19 Project Manager**: one HTML page and a developer handoff. Both are kept verbatim in `docs/reference/ui_v19/`. The UI built until now is **Legacy Lite**: it stays at `/legacy/` (`tools/run_lite.sh --ui legacy`) and is tagged `legacy-lite-0.1` in git.

This file states, screen by screen, what the page needs from the backend, what is connected today, and what is still the designer's template. It also lists what reviewing the page found: defects, data issues, gaps against the requirements, and the decisions that are open.

## How the page is connected

- **`web/index.html`** is the designer's file, byte for byte, plus one marked block at the end that loads `web/lite/`. `tests/test_lite_ui.py` enforces this. A new designer version therefore drops in as a file swap.
- **`web/lite/`** is Lite's bridge. It replaces the page's template functions, one group per file, with versions that call the backend. It keeps the page's own screen updates.
  - The same test fails if a function the bridge replaces is renamed or removed in a new version.
  - The bridge uses only the page's colour variables, so the light and dark themes both apply.
- **`/api/*`** is the Lite UI's API (`server/api_v2.py`). It is named after the handoff's suggested boundary (section 17).
  - Every route needs the session from the log-in, except the log-in itself and STOP.
  - Legacy Lite keeps `/api/lite/*`. Both share one backend, so one robot, one E-stop state and one STOP.

| Bridge file | Replaces | Backend |
|---|---|---|
| `safety.js` | `demoStop`, `rcStopBoth`, `rcSlowStop` | `POST /api/robots/stop-both`, `POST /api/robots/{left,right}/stop`. No log-in needed. STOP is sent before any page work. |
| `layout.js` + `lite.css` | — (top-bar labels) | Keeps STOP BOTH and the status on screen from 900 px wide (defect D1). |
| `fixes.js` | `rcSyncGuidedValues`, `demoApplyToRobotControl` | Page defect D2 |
| `projects.js` | `loadProjectDatabase`, `saveProjectDatabase`, `duplicateProject`, `deleteProject`, `exportProjectFile`, `importProjectFile` | `/api/projects…` (folders, ZIP project files, trash; a new project's fixture) |
| `auth.js` | `submitOperatorLogin`, `lockOperatorSession`, `submitRobotPin`, `requestRobotControlAccess`, `closeRobotControl`, `lockRobotControl` | `/api/auth/…` (hashed PINs, sessions, engineer level). Also fixes page defect D3. |
| `status.js` | the "FRONTEND READY" pill | `GET /api/robots/status` every 1.5 s and after each mode change |
| `robotpanel.js` | — (the pill opens it) | Robot & mode: Visual sim / Hardware sim / Real (D30), connect each arm, E-stop per arm with Recover (R4), engineer E-stop test |
| `tasks.js` | `loadTask`, `selectArm`, `decision`, `openProjectById`, `createProject`, `showNewProjectPanel`, `renderProjectManager` | `GET …/tasks`, `GET /api/fixtures/<id>/tasks`, `POST …/units/<KEY>/verdict` (D35, D29) |
| `prepare.js` | `demoApply`, `demoMove`, `moveRedoToStart` | `POST /api/robots/<arm>/pole`, `/hand`, `/move-pose` |
| `teach.js` | `startTeach`, `stopTeach`, `resetTimer`, `demoSave`, `handlePreviousTrajectory`, `demoPlayback`, `demoAction` | `…/units/<KEY>/teach/*`, `/recordings`, `/active`, `/name`, `/playback`, `/move-start` (D31) |
| `sim.js`, `three_scene.js`, `robot3d.js` | `simulationDurationSec`, `updateSimulationPlaybackFrame`, `resetSimulationPlayback`, `simulationControl` | `GET …/units/<KEY>/frames`, `GET /api/fixtures/<id>/scene`, Legacy's `/api/lite/robot-model` |
| `tune.js` | `setStage`, `rerunSimulationAndFocus`, `renderManualScaleRegions`, `trimSelectedPlotRegion`, `applySelectedPlotRegion`, `toggleTheme` | `…/units/<KEY>/settings`, `/undo`, `/redo`, `/optimise`, `/keep`, `/track` (D24, D36) |
| `realtest.js` | `simulateUpload`, `simulateRealRun`, `syncRealTestTask` | `…/units/<KEY>/upload`, `/run` |
| `advanced.js` | `rcSetMode`, `rcCheckMode`, `rcApplyHand`, `rcApplyPole`, `rcRefreshPrograms` | `POST /api/robots/<arm>/controller-mode`, `/hand`, `/pole`, `GET /api/robots/<arm>/programs` |

## Screen by screen

State legend:
- **Live**: works through the backend.
- **Stand-in**: works through the backend against the robot stand-in, so nothing moves.
- **Template**: still the designer's demo behaviour.
- **Page only**: browser-side by design.

| Screen and element | What it needs | State | Requirement |
|---|---|---|---|
| **Log-in** (keypad) | Operator PIN checked server-side | **Live** (PBKDF2, 5 tries then 30 s) | Handoff §19 |
| **Project Manager**: Continue, New, Open/Manage, rename, duplicate, archive, delete, Load/Save Project File, auto save | Projects owned by the backend, with their files | **Live**: `~/ALIX-Lite/projects/<id>/` in the handoff's layout. A project file is a ZIP. Delete moves the folder to `.trash/`. | Handoff §5, §20; NFR-DATA |
| Top bar: project chip, Save, Projects, Lock | as above | **Live** | — |
| Top bar: status pill | Mode, robot and E-stop | **Live** (stand-in robot) | R4, D30 |
| Top bar: **STOP BOTH** | Both arms stop, from any state | **Live** (no log-in needed; on screen from 900 px) | NFR-SAFETY; handoff §21.13–14 |
| Tour, theme | Browser preferences | Page only | — |
| **01 Setup & Safety**: 5 checks | Stored with the project | **Live** (in the project record) | R1 |
| Start-pose profiles, Edit/New (J1–J7), Move LEFT/RIGHT/BOTH to start | Stored poses per arm; `movej` with a transit check | **Stand-in** for the moves (Hardware sim and Real; confirmation in Real). The profiles themselves stay page-only for now. | R4 (transit check), R7 |
| **02 Region Teaching**: task queue, arm cards, zoom | Task list tied to the region catalogue | **Live**: the page's 22 tasks with the catalogue's codes and names (TASK 9 as the deck has it), each step's status, LATER steps that cannot be chosen; another fixture's project gets that fixture's list | R1, D35, D29 |
| Prepare: pole, hand preset and 6 channels, Apply, region start pose | `move_pole`, `set_hand`, per-region defaults | **Stand-in**: each step starts from its region's pole and hand values; Apply goes to the robot | R7 |
| Drag & Teach: target time, START/STOP, voice timer | Drag-teach start/stop, recording saved in the project | **Stand-in** (Hardware sim and Real). An existing recording is offered first (D31). | R1, D31 |
| Save & Review: Save TXT, Browse previous file, Move to recorded start, Playback | Store, validate, (N−1)×5 ms duration, playback in 3D or on the robot | **Live** for files (every recording listed, one active, a loaded file never replaces the new one); playback in **3D** in Visual sim, on the **stand-in** otherwise | R1, R3, NFR-DATA |
| Tune + Simulation: Trim, Mean Smooth (window), Scale regions, plot drag-select, Trim selected, Re-run, Start/Stop/Reset, Expand | Default optimiser (the prototype's algorithm), real plot, 3D playback, limits L1–L6 | **Live**: settings on the server with undo; the real recording on the plot; tuning with every limit checked; the robot in 3D on the 5 ms clock, with the playhead on the recording's rows | R2, R3, R6, R11, R12, D24 |
| Real Test: Program ID, Upload, Move to start, Run, PASS / Needs Tuning / Record Again | Program 1–100 checked server-side, upload, run, completion | **Stand-in**: the kept version uploads (replace needs a confirmation), runs in Hardware sim or Real (confirmed), the real outcome is shown; PASS needs a completed run | R4, R13 |
| **03 Merge + Full Sequence**: flow table, rules, waits, include, sequence JSON, custom merge per arm, step editor, visual confirm, full simulation, interlock test, real full run | Merge with joins, sequence scheduler, full simulation, dual-arm interlock | Template | R8, R10, NFR-SAFETY |
| **04 Project Summary**, task export | From the project record | **Live** (page state, saved on the server) | R1 |
| **Advanced Robot Controls** (engineer PIN) | Engineer level server-side | **Live** (PIN). SLOW STOP and Stop Both are **live**. | Handoff §14, §19 |
| Advanced: RUN (SIM/REAL/CHECK, upload, run, multi, stored programs) | Runner v14: modes, upload, run, program list | **Stand-in** for SIM/REAL/CHECK and the program list; uploading any file, stored and multi runs: template | R4, R13, D30 |
| Advanced: GLOBAL POINTS, DRAG TEACH, HAND/POLE, TELEMETRY | Runner v14 points, drag teach, `hand.py`/`pole.py`, telemetry stream | **Stand-in** for HAND/POLE; the rest template | R4, R7 |
| Advanced: runner and SDK paths, Validate | Lite's own pinned runner v14 and SDK (vendored); a self-check | Template | NFR-OFFLINE-DEPLOY |

## Defects found in V19 (fixed by the bridge, for the designer's next version)

- **D1 — STOP BOTH leaves the screen.** The top bar needs 1,610 px. At 1600, 1440, 1366, 1280 and 1024 px wide, STOP BOTH and the status pill are off the right edge, and the page scrolls sideways. This breaks the handoff's own rule 14.
  - Fix: labels collapse to their icons step by step (`layout.js`, `lite.css`).
  - Checked: STOP BOTH is fully visible at 1920, 1600, 1440, 1366, 1280, 1024 and 900 px, with no sideways scroll.
- **D2 — Opening Advanced Robot Controls throws `vals.forEach is not a function`.** `readHandValues()` returns an object, but `rcSyncGuidedValues` and `demoApplyToRobotControl` iterate it as an array. As a result, Prepare's **Apply Hand** never reaches the arm panel.
- **D3 — Resume loses the stage after Lock.** Logging in again opens the Project Manager, which saves the open project with the view the log-in has just set (Home). "Resume: TASK 5A · RIGHT ARM · Tune + Simulation" then becomes "Resume: Home". After a Lock, the bridge closes the project; Continue restores the saved stage.
- **D4 — Wording.** The delete confirmation says the project is "browser-saved"; the bridge's text says its folder moves to the trash.

## Task list: V19 against the region catalogue

V19's 22 tasks come from the same deck (`Toilet_regions_V1.pptx`, slides 5–25) as Lite's catalogue: 42 arm steps, of which 34 are in Release 1 (decision D3). They match task by task, with these issues:

- **TASK 9 has its arms swapped in V19.** The deck (slide 17) gives LEFT = SEAT UNDERSIDE LEFT HALF (`L-R08`) and RIGHT = FLOOR SPILLAGE PART 2 (`R-R08` part 2). V19 lists them the other way round. Its picture crop follows the slide, so the LEFT card shows the seat but is labelled "floor spillage".
- **R-T04 PICK DRAIN BRUSH and R-T10 HOLD BRUSH OUTSIDE** each appear in two tasks (10A–10B and 11A–11B). In the deck each is one action spanning two tasks.
- **D3 exclusions.** Eight arm steps are outside Release 1. V19 shows the glove steps as "procedure later" but makes the drain-brush and drain-hole steps teachable:
  - drain brush: R-T04, R-T10, R-T06;
  - drain hole: R-T05 parts 1 and 2;
  - glove: L-T06, L-T04, R-T07.
- **Names and codes.** V19 names are shorter, and many have no code (for example "FLOOR SPILLAGE PART 1" is `R-R08` part 1). The catalogue also carries per-region pole height, hand preset and pictures cropped per arm, which V19 lacks: V19 uses 150 mm and Grip for every region.

## Gaps against the requirements, and how the bridge closes them (D36)

The requirements hold these, V19's design does not; each is added in V19's own style:

| Gap | Addition |
|---|---|
| E-stop per arm (IO3) with guided recovery (R4 h) | Robot & mode panel: each arm's E-stop (Clear, pressed, released, unknown) and Recover; the status pill shows E-STOP whenever motion is blocked |
| Connecting the arms | Robot & mode panel: Connect and Disconnect per arm (addresses LEFT 192.168.1.10, RIGHT 192.168.1.103, as runner v14) |
| Three modes, D30 | Robot & mode panel, opened from the status pill. Lite starts in Visual sim; Real needs a confirmation. The Real Test stage says what Run will do in the current mode. |
| Fixture pool, D29 | New Project has a Fixture choice (commode C by default); the project's fixture sets its task list and its 3D scene. Project Library rows show it. |
| Existing recordings by default, D31 | Drag & Teach offers the region's existing recording first ("Use this recording → Save & Review"); a new project gets the fixture's taught set |
| Time-optimal (D24) and target time | Tune: Optimiser (Default or Time-optimal) and Total time (keep the recorded pace, a target, or as fast as safe), with "Use X s" when a target is out of reach |
| Limits L1–L6; no upload unless every limit passes | Tune: the result strip with every limit's value; "Simulation Looks Good → Real Test" keeps the version only when all pass |
| Confirmations before motion | Every Real move asks first, in the page's modal style with Cancel focused; replacing a program asks too |
| Undo and redo of tuning (R2) | Tune: Undo and Redo, one step per change |
| 3D check against the toilet, walls, bin and other arm (R3, R5) | **Still open**: the simulation shows the robot in the scene; the collision check comes with the backend's 3D check |

The handoff suggests PyBullet for simulation. Lite keeps ALIX's own approach, as built in Legacy Lite: the baked robot overlay, ALIX's FK on the server and its collision model; only the display moved into V19's simulation pane.

## While the robot is a stand-in

- Nothing moves; replies say "read back" as the real backend will.
- Drag teaching saves one of the fixture's existing recordings as the new take. The library holds LEFT-arm recordings only, so a RIGHT-arm "take" reuses a left-arm file: its 3D motion is not meaningful until real drag teaching exists.
- Start-pose profiles are names only; stored poses come with the robot I/O backend.

## Decisions (13 October 2026, D33–D36 in the requirements)

1. **Theme (D34):** the Lite UI keeps V19's look; the ALIX Studio theme stays in Legacy Lite.
2. **Task list (D35):** V19's 22-task queue, with each arm step tied to its catalogue unit (code, pictures, pole and hand defaults, existing recordings). TASK 9 is corrected from the deck, and the eight arm steps outside Release 1 show as LATER, the way V19 shows procedures.
3. **Settled decisions in V19 (D36):** each is added as a small addition in V19's own style:
   - D24 (Time-optimal);
   - D29 (fixture choice);
   - D30 (three modes);
   - D31 (existing recordings by default);
   - the target time with the closest safe time;
   - the limit results;
   - the E-stop state per arm;
   - the confirmations before motion.

## Next, in the handoff's order (section 23)

1. **Done:** application shell, project persistence and log-in (Phase 1); STOP; the region loop against the stand-in (Phases 4, 5 and 7, and Phase 6 for one arm): recordings in the project, tuning with the optimisers and every limit, real plots, the 3D robot in the simulation pane, upload, run and PASS.
2. Robot I/O (Phases 2, 3 and 9's core): the runner v14 wrapper, the IO3 E-stop monitor, Hardware sim on the controllers, real drag teaching, stored start poses, Global Points, telemetry; the rest of Advanced Robot Controls.
3. The real 3D check with walls (R3, R5), shown on the plot and in the simulation.
4. Merge + Full Sequence (Phase 8), then safety hardening (Phase 9).
