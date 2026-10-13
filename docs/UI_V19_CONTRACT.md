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
| `projects.js` | `loadProjectDatabase`, `saveProjectDatabase`, `duplicateProject`, `deleteProject`, `exportProjectFile`, `importProjectFile` | `/api/projects…` (folders, ZIP project files, trash) |
| `auth.js` | `submitOperatorLogin`, `lockOperatorSession`, `submitRobotPin`, `requestRobotControlAccess`, `closeRobotControl`, `lockRobotControl` | `/api/auth/…` (hashed PINs, sessions, engineer level). Also fixes page defect D3. |
| `status.js` | the "FRONTEND READY" pill | `GET /api/robots/status` every 1.5 s |

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
| Start-pose profiles, Edit/New (J1–J7), Move LEFT/RIGHT/BOTH to start | Stored poses per arm; `movej` with a transit check | Template | R4 (transit check), R7 |
| **02 Region Teaching**: task queue, arm cards, zoom | Task list tied to the region catalogue | Template data (22 tasks inside the page; see "Task list") | R1 |
| Prepare: pole, hand preset and 6 channels, Apply, region start pose | `move_pole`, `set_hand`, per-region defaults | Template (the stand-in supports both through `/api/lite`) | R7 |
| Drag & Teach: target time, START/STOP, voice timer | Drag-teach start/stop, recording saved in the project | Template (stand-in exists) | R1 |
| Save & Review: Save TXT, Browse previous file, Move to recorded start, Playback | Store, validate, (N−1)×5 ms duration, playback in 3D or on the robot | Template | R1, R3, NFR-DATA |
| Tune + Simulation: Trim, Mean Smooth (window), Scale regions, plot drag-select, Trim selected, Re-run, Start/Stop/Reset, Expand | Default optimiser (the prototype's algorithm), real plot, 3D playback, limits L1–L6 | Template (optimiser engine done, not connected) | R2, R3, R6, R11, R12 |
| Real Test: Program ID, Upload, Move to start, Run, PASS / Needs Tuning / Record Again | Program 1–100 checked server-side, upload, run, completion | Template (stand-in exists) | R4, R13 |
| **03 Merge + Full Sequence**: flow table, rules, waits, include, sequence JSON, custom merge per arm, step editor, visual confirm, full simulation, interlock test, real full run | Merge with joins, sequence scheduler, full simulation, dual-arm interlock | Template | R8, R10, NFR-SAFETY |
| **04 Project Summary**, task export | From the project record | **Live** (page state, saved on the server) | R1 |
| **Advanced Robot Controls** (engineer PIN) | Engineer level server-side | **Live** (PIN). SLOW STOP and Stop Both are **live**. | Handoff §14, §19 |
| Advanced: RUN (SIM/REAL/CHECK, upload, run, multi, stored programs) | Runner v14: modes, upload, run, program list | Template (stand-in has modes, upload, run, programs) | R4, R13, D30 |
| Advanced: GLOBAL POINTS, DRAG TEACH, HAND/POLE, TELEMETRY | Runner v14 points, drag teach, `hand.py`/`pole.py`, telemetry stream | Template | R4, R7 |
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

## Gaps against the requirements (v1.5) and settled decisions

These are in the requirements but not in V19's design:

- **E-stop per arm** (IO3) with guided recovery (R4 h). V19 has a dual-interlock concept but no E-stop state per arm. The status pill now shows E-STOP whenever the backend blocks motion.
- **Connecting the arms** (IP entry, connect or disconnect). V19 shows fixed addresses: LEFT 192.168.1.10 and RIGHT 192.168.1.103, which match runner v14.
- **Three modes, D30** (Visual sim, Hardware sim, Real). V19 has SIM and REAL per arm only inside Advanced, and none in the operator flow.
- **Fixture pool, D29.** V19 is commode only.
- **Existing recordings used by default, D31.** V19 teaches every region, and "Browse previous file" reads only from this computer.
- **Time-optimal (TOPP-RA, D24) and target time.** For R2, the target time means the closest feasible time and the limit that binds. V19's "target time" is a teaching timer, and tuning is Trim, Smooth and Scale only.
- **Limits L1–L6, and no upload unless every limit passes** (R2, R12). V19 has no limit results.
- **3D collision check** against the toilet, walls, bin and the other arm (R3, R5). V19's simulation is a picture with a timer and playhead.
- **Confirmations before motion** (NFR-SAFETY). V19 uses the browser's `confirm()` in Advanced and asks nothing before Real Test's Run. Replacing a program also needs a confirmation (R13).
- **Undo and redo of tuning** (R2).

The handoff suggests PyBullet for simulation. Lite keeps ALIX's own approach, already built in Legacy Lite: the baked robot overlay, ALIX's FK on the server, and its collision model. Only the display moves into V19's simulation pane.

## Decisions needed

1. **Theme.** V19 has its own look: blue and orange, light and dark. Requirements v1.5 (NFR-THEME, §3) say the UI is rebuilt "with the ALIX Studio theme". Keep V19's look (recommended), or re-skin it with ALIX's tokens?
2. **Task list.** Keep V19's 22-task queue, but tie each arm step to its catalogue unit: codes, pictures, pole and hand defaults, existing recordings. Then fix TASK 9 from the deck, and show the D3-excluded steps as "LATER", the way V19 shows procedures (recommended)?
3. **Settled decisions missing from V19** (D24, D29, D30, D31; target time; limits; E-stop; confirmations). Add each as a small addition in V19's own style (recommended), or keep V19 exactly as designed and park them?

## Next, in the handoff's order (section 23)

1. **Done in this step:** application shell, project persistence and log-in (handoff Phase 1). STOP is wired.
2. Teach → Save & Review → Tune + Simulation → Real Test against the stand-in (Phases 4, 5, 7, plus Phase 6 for one arm):
   - recordings in the project;
   - the Default optimiser;
   - real plots;
   - the 3D robot in the simulation pane;
   - program upload and run.
3. Advanced Robot Controls against the stand-in (Phases 2 and 3).
4. Robot I/O: runner v14 wrapper, the IO3 E-stop monitor, Hardware sim. Then the real 3D check with walls.
5. Merge + Full Sequence (Phase 8), then safety hardening (Phase 9).
