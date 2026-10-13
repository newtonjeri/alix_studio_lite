# Lite UI (V19) — front-end contract

Last updated: 13 October 2026.

The Lite UI is the designer's **ALIX Studio LITE Edition V19 Project Manager**: one HTML page and a developer handoff. Both are kept verbatim in `docs/reference/ui_v19/`. The UI built until now is **Legacy Lite**: it stays at `/legacy/` (`tools/run_lite.sh --ui legacy`) and is tagged `legacy-lite-0.1` in git.

This file states, screen by screen, what the page needs from the backend, what is connected today, and what is still the designer's template. It also lists what reviewing the page found: defects, data issues, gaps against the requirements, and the decisions that are open.

## How the page is connected

- **`web/index.html`** is the designer's file, byte for byte, plus one marked block at the end that loads `web/lite/`. `tests/test_lite_ui.py` enforces this. A new designer version therefore drops in as a file swap.
- **`web/lite/`** is Lite's bridge. It replaces the page's template functions, one group per file, with versions that call the backend. It keeps the page's own screen updates.
  - The same test fails if a function the bridge replaces is renamed or removed in a new version.
  - The bridge uses only the page's colour variables, so the light and dark themes both apply. The page's own dark theme is incomplete (defect D5); `lite/dark.css` completes it.
- **`/api/*`** is the Lite UI's API (`server/api_v2.py`). It is named after the handoff's suggested boundary (section 17).
  - Every route needs the session from the log-in, except the log-in itself, STOP, and `GET /api/lite/hello` (how a second start of Lite finds the one already running).
  - Legacy Lite keeps `/api/lite/*` and asks for the same operator PIN; its engineer level, like the Lite UI's, belongs to the session that entered the PIN. Both share one backend, so one robot, one E-stop state and one STOP.

| Bridge file | Replaces | Backend |
|---|---|---|
| `safety.js` | `demoStop`, `rcStopBoth`, `rcSlowStop` | `POST /api/robots/stop-both`, `POST /api/robots/{left,right}/stop`. No log-in needed. STOP is sent before any page work. While an overlay covers the top bar (the log-in screen, the Project Manager, any dialog, the Trajectory Viewer), a second STOP BOTH in the page's own style stays on top of everything. |
| `layout.js` + `lite.css` | — (top-bar labels) | Keeps STOP BOTH and the status on screen from 900 px wide (defect D1). |
| `fixes.js` | `rcSyncGuidedValues`, `demoApplyToRobotControl` | Page defect D2 |
| `projects.js` | `loadProjectDatabase`, `saveProjectDatabase`, `duplicateProject`, `deleteProject`, `exportProjectFile`, `importProjectFile` | `/api/projects…` (folders, ZIP project files, trash; a new project's fixture) |
| `auth.js` | `submitOperatorLogin`, `lockOperatorSession`, `submitRobotPin`, `requestRobotControlAccess`, `closeRobotControl`, `lockRobotControl` | `/api/auth/…` (hashed PINs, sessions, engineer level). Also fixes page defect D3. |
| `status.js` | the "FRONTEND READY" pill | `GET /api/robots/status` every 1.5 s and after each mode change |
| `robotpanel.js` | — (the pill opens it) | Robot & mode: Visual sim / Hardware sim / Real (D30), each arm's address and Connect, E-stop per arm with Recover (R4), each arm's pole and hand; with `--robot realman`, the live feedback of the arm, the pole, the hand and IO3 (`server/robot_link.py`, read only); engineer tests: E-stop, and a pole or hand that fails to arrive (`POST /api/robots/<arm>/fault`, stand-in only) |
| `tasks.js` | `loadTask`, `selectArm`, `decision`, `openProjectById`, `createProject`, `showNewProjectPanel`, `renderProjectManager` | `GET …/tasks`, `GET /api/fixtures/<id>/tasks`, `POST …/units/<KEY>/verdict` (D35, D29) |
| `prepare.js` | `demoApply`, `demoMove`, `moveRedoToStart` | `POST …/units/<KEY>/setup` (the region's pole and hand as shown: the preset, or with Custom the six channels, an empty one held; staged and read back in Real, emulated in Hardware sim), `POST /api/robots/<arm>/move-pose` (BOTH: one confirmation in Real, each arm's outcome shown) |
| `teach.js` | `startTeach`, `stopTeach`, `resetTimer`, `demoSave`, `handlePreviousTrajectory`, `demoPlayback`, `demoAction` | `…/units/<KEY>/teach/*`, `/recordings`, `/recordings/<FILE>/stage`, `/active`, `/name`, `/playback`, `/move-start` (D31) |
| `sim.js`, `three_scene.js`, `robot3d.js` | `simulationDurationSec`, `updateSimulationPlaybackFrame`, `resetSimulationPlayback`, `simulationControl` | `GET …/units/<KEY>/timeline` (the execution: pole, hand, move to the start, trajectory), `GET /api/fixtures/<id>/scene`, Legacy's `/api/lite/robot-model` |
| `tune.js` | `setStage`, `rerunSimulationAndFocus`, `renderManualScaleRegions`, `trimSelectedPlotRegion`, `applySelectedPlotRegion`, `toggleTheme` | `…/units/<KEY>/settings`, `/undo`, `/redo`, `/optimise`, `/keep`, `/track` (D24, D36) |
| `realtest.js` | `simulateUpload`, `simulateRealRun`, `syncRealTestTask` | `…/units/<KEY>/upload`, `/run` |
| `advanced.js` | `rcSetMode`, `rcCheckMode`, `rcApplyHand`, `rcApplyPole`, `rcRefreshPrograms` | `POST /api/robots/<arm>/controller-mode`, `/hand`, `/pole`, `GET /api/robots/<arm>/programs` |
| `viewer.js` | — (added: the Trajectory Viewer) | `/api/library…` (`server/api_library.py`, `server/library.py`) |
| `sequence.js` | `runFullSequenceSimulation`, `fullSequenceSimulationDuration`, `updateFullSequenceOverlayFrame`, `fullSequenceSimulationControl`, `updateFullSimArmPanels` | `POST /api/projects/<id>/sequence` (the schedule, `server/sequence.py`), `POST …/units/<KEY>/timeline` (one step's 3D frames from where the arm was left) |
| `brand.js` + `tokens.css` | — (the page's "AX" tiles) | ALIX's brand mark as ALIX Studio draws it, in ALIX's brand blues, in the top bar, on the log-in screen and as the tab's icon |

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
| Prepare: pole, hand preset and 6 channels, Apply, region start pose | `move_pole`, `set_hand`, per-region defaults | **Stand-in**: the region's setup (its existing recording's pole and hand, else the catalogue's) is shown and stored; Apply keeps the page's values as the setup and stages them: in Real on the robot, the pole then the hand, each read back; in Hardware sim emulated, as nothing moves there. A failure says so and leaves the setup unapplied. | R7 |
| Drag & Teach: target time, START/STOP, voice timer | Drag-teach start/stop, recording saved in the project | **Stand-in**, in Real only: in Hardware sim the pole and the hand are emulated, so a recording would not know its true stage. An existing recording is offered first (D31). Teaching starts only with the arm in the region's setup (read back), and the recording keeps the arm and the stage it was taught with. | R1, D31 |
| Save & Review: Save TXT, Browse previous file, Move to recorded start, Playback | Store, validate, (N−1)×5 ms duration, playback in 3D or on the robot | **Live** for files: every recording listed with its arm and stage, one active, a loaded file never replaces the new one. A loaded file needs its arm (from its name, or stated) and its pole and hand; a file of the other arm is refused. Playback in **3D** in Visual sim (staging first), on the **stand-in** otherwise | R1, R3, NFR-DATA |
| Tune + Simulation: Trim, Mean Smooth (window), Scale regions, plot drag-select, Trim selected, Re-run, Start/Stop/Reset, Expand | Default optimiser (the prototype's algorithm), real plot, 3D playback, limits L1–L6 | **Live**: settings on the server with undo; the real recording on the plot; tuning with every limit checked; the gates (arm, stage, limits, robot) above the results; ▶ Start stages the pole and the hand in 3D (badge STAGING), moves the arm to the start, then plays the trajectory on the 5 ms clock with the playhead on the recording's rows | R2, R3, R6, R11, R12, D24 |
| Real Test: Program ID, Upload, Move to start, Run, PASS / Needs Tuning / Record Again | Program 1–100 checked server-side, upload, run, completion | **Stand-in**: the gates; the kept version uploads (replace needs a confirmation), runs in Hardware sim or Real (confirmed) after the pole and the hand are staged and read back, and the real outcome is shown ("Stopped: The LEFT pole did not reach 200 mm (read back 160 mm). Nothing else ran."); PASS needs a completed run | R4, R13 |
| **03 Merge + Full Sequence**: flow table, rules, waits, include, sequence JSON, visual confirm, **full simulation** | Sequence scheduler, full simulation | **Live** for the full simulation, in 3D: each included row's arm steps run their regions' kept versions (each refused for the other arm or without its stage), each arm continuing from where its previous step left it: pole, hand, move to the first line, trajectory; the row's rule (TOGETHER, LEFT/RIGHT RUN FIRST, ONE BY ONE, PROCEDURE) and checkpoint wait; LATER steps skipped. Start is refused until every included arm step has a kept version, listed with the reason. A preview with the recordings as taught never approves the sequence. Play, pause, speed, a time slider. | R8, R10 |
| 03: custom merge per arm (files, joins), Simulate LEFT / RIGHT, step editor, interlock test, real full run | Merge with joins, dual-arm interlock, the real full run | Template | R8, R10, NFR-SAFETY |
| **04 Project Summary**, task export | From the project record | **Live** (page state, saved on the server) | R1 |
| **Trajectory Viewer** (added; Operator Journey, Home, Save & Review) | Upload taught .txt files (or a folder), replay them in 3D, check them | **Live**: files kept in `~/ALIX-Lite/library/` beside the built-in sets (commode C taught set, V6 package, each file with its record's pole and hand); the arm from the name or folder, else chosen; the pole and hand each upload was taught with are asked for, and a file without them cannot be viewed. A LEFT and a RIGHT file replay as the robot runs them: each arm stages its pole and hand, the arm ready first waits at its start, and both trajectories start on the same tick, then run on the 5 ms clock, with play, pause, speed, a time slider and the toilet state; the joints on a plot with a playhead; the gates and L1–L4 on each file as taught; "Use for <task> · <region>" only for a region of the file's own arm | R1, R3, R6 |
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
- **D6 — The full sequence simulation is a picture with a clock.** It shows a still reference image and moves a timer at 1.25 s per step, whatever the files; it can still approve the sequence for the real full run. The bridge replaces it with the robot in 3D running the kept versions (see Merge + Full Sequence above).
- **D7 — The hand preset does nothing.** Choosing Grip or All Open changes neither the six channels nor what Apply sends, and a held channel (left where it is, as the taught set's "fingers 50 % open, thumb held") cannot be shown. The bridge fills the channels from the preset, switches to Custom when a channel is edited, shows a held channel empty, and sends what is on screen.
- **D5 — The dark theme covers only part of the page.** V19's dark rules recolour its variables and 24 components; everything else stays light. In the dark theme, a headless audit of every screen and modal found 96 kinds of element still on a light background and 102 with text below 3:1 contrast. Examples: the task numbers (white on white), the top-bar buttons, the status bars, the stage pipeline, the PASS / Needs Tuning / Record Again buttons, the whole Merge + Full Sequence page, Advanced Robot Controls, form labels (2.3:1) and every coloured badge.
  - Fix: `web/lite/dark.css`, generated from the page's own stylesheet by `tools/make_dark_css.py`. Every light background, pastel border and dark text moves onto V19's dark palette by its hue; the values live in `lite/tokens.css`. The page's cascade is kept with `:where()`. Picture frames stay white, as V19 keeps them. A test checks the file is current; rerun the tool for a new designer version.
  - Checked: after the fix, the same audit finds only the white picture frames, the carousel dots, and V19's white-on-orange buttons (2.9:1 in both themes, the designer's choice).

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

## The gates on every execution

Checked by the server on every execution, in the 3D simulation, in Hardware sim and in Real; the page only shows them (`server/api_units.py`, `identity` and `gates`):

1. **Arm.** A trajectory runs only on the arm it was taught on. Every recording, tuned candidate and kept version carries `taught_arm`; a file of the other arm, or of an unknown arm, is refused (`code: "arm"`): it is never shown moving, tuned, uploaded or run there. A loaded file's arm comes from its name (`L_`, `R_`, `LEFT`, `RIGHT`) or is stated, and a name and a statement that disagree are refused. An older recording without a recorded arm gets one back only when its bytes are a built-in file's.
2. **Stage.** Every file carries the pole height and hand pose it was taught with (`stage`). Every execution sets them first: the pole (ALIX's measured travel time), then the hand (ALIX's hand units and speed), each read back (pole within 5 mm, every set hand channel within 30 counts), then the joint move to the first line, then the trajectory. If the pole or the hand does not arrive, the job ends there and the arm never moves. A file without its stage is refused (`code: "stage"`) until the operator states it; a recorded stage is never overwritten.
3. **Limits.** A run needs a kept version that passes L1–L6. L3 reads acceleration as ALIX does: the 20 Hz zero-phase band-limited second difference (`plan_min_time.band_limited_util`, finding F99c), never raw differences, which read the recorder's millidegree steps as thousands of °/s². FILE_04 as taught: 757 °/s² band-limited (ALIX's own function gives the same), 13,800 raw.
4. **Robot.** Hardware sim or Real, a Real motion confirmed, no motion on either arm while an E-stop is not clear; STOP always works. **Hardware sim moves nothing physically** (D30 as clarified on 13 Oct): programs run in the controllers' SIM mode and the pole and the hand are emulated, so drag teaching needs Real, and a recording as taught never plays back on the real arms.

The 3D simulation plays the same execution the robot does (`GET …/timeline`): ALIX's FK poses every link below the sliding plate at each instant, so the arm is drawn at its stage height, not at the standing pose's.

## The live link to the arms (read only)

`run_lite --robot realman` connects each controller through ALIX's own robot I/O code (`alix_dispatch.connect.open_arm`) and reads, about five times a second, the three devices of each side: the arm (seven joints, error codes, the controller's run mode), the pole (`peripherals.Pole`) and the hand (`peripherals.HandModbus.read`, ANGLE_ACT), and the E-stop input IO3 (`server/robot_link.py`). Robot & mode shows it, and the live E-stop drives the interlock (unreadable or not in mode 5 blocks motion, like a press). Lite commands nothing through it: actions stay the stand-in's, so Real robot is refused while linked, and the screens say that nothing reached a controller. The one setting it writes is the end port's Modbus mode, which reading the hand needs (the control program and ALIX set it the same way). **Lite's STOP BOTH does not reach the linked controllers**: it stops the stand-in's jobs; the red E-stop buttons stop the arms. The link needs Linux (the RealMan SDK ships for Linux only); elsewhere Connect says so. `--robot emulated` gives the same link to ALIX's emulated arms.

## While the robot is a stand-in

- Nothing moves. Prepare and the region steps say "read back" as the real backend will; Real Test and Advanced Robot Controls add that nothing reached a controller. The arms start at the robot model's standing pose (pole 300 mm, hand open), as the 3D view does.
- Drag teaching saves a real recording of the SAME arm as the new take, with the stage it was taught with: the region's own existing recording if it has one, else the first built-in file of that arm (the V6 package for the RIGHT arm). A left-arm file is never a right-arm take. Older takes that reused a left-arm file for a right-arm region are recognised by their bytes and refused.
- An engineer can make the pole or the hand fail to arrive (Robot & mode, engineer section), to see that nothing runs.
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
4. Merge + Full Sequence (Phase 8): the full simulation is done in 3D; the custom merge with joins, the single-arm merged simulation and the real full run remain. Then safety hardening (Phase 9).
