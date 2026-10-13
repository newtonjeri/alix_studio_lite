# ALIX Studio Lite — Requirements (Revised)

**Status: FINAL — version 1.5, 13 October 2026.** This is the requirements document of record for ALIX Studio Lite; it supersedes all earlier drafts.

**Revision of the requirement inputs, with findings, a borrow-from-ALIX-Studio verdict and a recommended approach for every requirement**

| | |
|---|---|
| Version | 1.5 (final) — revised with the decisions of 13 October 2026 (section 1) |
| Date | 13 October 2026 |
| Revises | [`reference/ALIX_Studio_Lite_requirement_inputs.txt`](reference/ALIX_Studio_Lite_requirement_inputs.txt) (14 requirements + scope notes) |
| Companion document | `ALIX_Studio_Lite_Implementation_Plan.md` in this folder (architecture, schedule, tests, safety) |
| Location | `alix_studio_lite/docs/`; the code is implemented in `alix_studio_lite/` |
| For review by | The requester and the ALIX team |

**Path conventions.** `alix/` = `~/alix_ws/src/alix` (ALIX monorepo, branch `feature/alix_studio`). `lite/` = `~/alix_studio_lite_ws` (this workspace). `studio/` = `alix/core/python/alix_taskgen/src/alix_taskgen` (ALIX Studio). Evidence is cited as `path:line`.

**How each requirement is written up.** Original text (verbatim) → revised requirement (testable) → findings → *can it be borrowed from ALIX Studio?* → recommendation and approach → acceptance criteria → priority, effort, dependencies → open questions.

**Borrow verdicts** (about ALIX Studio only; other sources are listed separately):

| Verdict | Meaning |
|---|---|
| **Borrow as-is** | The feature exists in ALIX Studio and can be lifted with no functional change. |
| **Borrow + adapt** | The feature exists but needs changes (inputs, layout, behaviour). |
| **Partial** | Some building blocks exist; the requirement still needs new work. |
| **Build new** | Nothing usable in ALIX Studio; another source or new code is needed. |

**Size** is a relative measure of how much work a requirement is: **S** small · **M** medium · **L** large · **XL** very large. Delivery time is planned in the implementation plan (agent-driven build plus robot lab days), not here.

---

## Contents

1. [Executive summary](#1-executive-summary)
2. [What ALIX Studio Lite is](#2-what-alix-studio-lite-is)
3. [Sources reviewed — what each one does](#3-sources-reviewed--what-each-one-does)
4. [Glossary](#4-glossary)
5. [The operator journey](#5-the-operator-journey)
6. [Summary of verdicts](#6-summary-of-verdicts)
7. [Functional requirements R1–R14](#7-functional-requirements-r1r14)
8. [Scope exclusions (X1, X2)](#8-scope-exclusions-x1-x2)
9. [Non-functional requirements](#9-non-functional-requirements)
10. [Integration requirements](#10-integration-requirements)
11. [Open questions for the requester and the team](#11-open-questions-for-the-requester-and-the-team)
- [Appendix A — Region catalogue for sign-off (commode C)](#appendix-a--region-catalogue-for-sign-off-commode-c)
- [Appendix B — Evidence base](#appendix-b--evidence-base)

---

## 1. Executive summary

**What was done.** Every requirement in the requirement inputs was checked against the code in four places:

- ALIX Studio;
- the trajectory-optimisation prototype;
- the upload-channel control application;
- related ALIX work (kinematics and collision, robot I/O, taught-track tools including TOPP-RA, the RealMan manuals).

An independent reviewer re-checked each finding against the code. This version (1.5) applies the decisions taken on 13 October 2026, listed below. Measurements quoted here were made with read-only scripts; nothing was run on the robot.

### Decisions taken on 13 October 2026

| Topic | Decision | Effect on the requirements |
|---|---|---|
| Target computer | The operator PC (x86-64 Linux) now; the robot's Jetson AGX Orin (aarch64) in production | Both builds are required (NFR-OFFLINE-DEPLOY). The Jetson build must compile `toppra` from source. |
| Optimisers | Two: the **default** (the trajectory prototype's algorithm) and a **time-optimal** one (TOPP-RA). The operator's time for a region is the **target**; if the limits do not allow it, Lite returns the closest feasible time and names the limit that binds. | R2, R10, R11, R12 and INT-TRAJ rewritten |
| Fixture | The regions deck describes commode C | The region catalogue (Appendix A) is for ALIX `commode_c` |
| Safety stop | The emergency-stop buttons wired to an IO input on each arm controller are the primary safety stop. They stop the robot without cutting power. The software STOP will still be tested. | R4 and NFR-SAFETY: E-stop state per arm, motion blocked while it is pressed, guided recovery |
| E-stop input | IO3 on each controller, in the controller's E-stop mode. The stop holds from the press until the button is released (the manual reset). | Lite reads IO3 on both arms, checks at connect that it is in E-stop mode (RealMan's demo maps IO3 to "continue program") and blocks motion otherwise (R4, NFR-SAFETY) |
| Acceleration limit | 600 °/s² is the arm's hardware limit | A hard limit for both optimisers, with no override (R2) |
| Optimiser choice | Operators and engineers can both switch a region between the Default and the Time-optimal optimiser | INT-TRAJ, NFR-UX |
| Clock | 5 ms per line is the only time base; durations and targets are (lines − 1) × 5 ms (13 Oct 2026, D26) | One line is one 5 ms controller cycle. Target times, achieved times, playback at 1× and timelines use (lines − 1) × 5 ms, and every limit is judged at 5 ms (R2, R3, R4, R8, R10, INT-TRAJ) |
| Scene | The URDF and the `commode_c` scene already hold the cell (commode pose and lid/seat articulation, bin, poles). Only wall collision is not enforced today. | No calibration project. Walls are enforced (R3, R5, R8, R14). |
| Walls | A single cubicle, not the whole washroom: 1400 mm wide by default, 800 mm selectable | ALIX has an 800 mm cubicle mesh, but its 1400 mm venue is the whole washroom (4.3 × 8.1 m). Lite's 1400 mm cubicle is now built the same way (`assets/scene/cubicles/`). |
| First-release regions | Codes in the deck's own format, with seven agreed changes. Glove and bin tasks, and the drain hole, are out for now. Lid and seat operations (opening and closing) are in, and they are not cleaning tasks. | 34 units: 26 cleaning regions and 8 lid/seat operations (Appendix A, R1) |
| Program IDs | Like the control program: upload to any ID 1–100, replacing an existing program after a confirmation | R13 and INT-CTRL |
| Location | Code in `alix_studio_lite/`, documents in `alix_studio_lite/docs/`. Everything Lite needs lives inside that folder. | The reused ALIX code and data are vendored into `alix_studio_lite/vendor/` with a manifest; no ALIX checkout at run time (X1/X2, deployment, plan) |
| Delivery | Implemented with Claude (agent-driven). Order: UI/UX first; the backend is on hold at its checkpoint (D27) | Requirements show relative size only; the plan carries the schedule and the order |

### Bottom line

Lite is feasible with heavy reuse, but almost nothing can be lifted from ALIX Studio unchanged. The recommended shape is a separate, offline application in `alix_studio_lite/` that:

1. uses ALIX's kinematics, collision and taught-track code, including its TOPP-RA chain;
2. keeps the prototype's algorithm as the default optimiser, with three small fixes;
3. copies ALIX Studio's theme and selected UI pieces;
4. drives the robot through the current control-app runner (v14), later turned into a library.

### Answers to the notes in the inputs

| Note in the inputs | Finding |
|---|---|
| Control app: is `rm_trajectory_ui_v48_handoff_continue_fix.py` the latest and most comprehensive? | **No.** UI **v53** + runner **v14** in `lite/upload_channel/` is the current pair and includes everything in v48 (section 3). |
| R3 Simulation playback — "already existing in ALIX Studio, borrow" | **Partly.** The 3D viewer, robot model and players exist. They only play ALIX plan files, with ALIX's default pole, hand and bin settings, and have no link back to an editor. Lite needs a `.txt` adapter, explicit pole/hand/bin values and its own playback screen. |
| R5 Collision graph highlighting — "borrow from ALIX Studio" | **Partly.** The Dual-Arm Collisions tab turns colliding links red and has a collision timeline. It never plots joint values and never stops at a collision. It ignores the toilet, the walls and the fingers, and it misses a real thumb-to-wrist contact on V6 data. |
| R7 Pole height and hand position — "borrow from ALIX Studio" | **Not in ALIX Studio.** The Studio never commands the robot. The pole and hand commands are in the control app; per-area pole and hand values are in ALIX's taught-track notes. |
| R8 Merging — "a mechanism exists … taking the commode into account" | **Partly.** ALIX has join functions and the commode collision model, but no join checks collisions. The V6 package was merged without any collision check. |
| R10 Dual-arm view — "borrow from ALIX Studio; correct URDF and meshes; bin toggle" | **Partly.** The URDF is correct. The Studio's baked meshes are stale (old, taller bin; no wrist camera). There is no split view, and the bin is fixed LOWERED, a position that collides with 4 of 5 taught low-pole V6 programs. |
| R13 Faster upload — "check the ALIX findings" | The controller fixes the transfer rate at about 45 kB/s. A faster upload comes from fewer points (optimised files are shorter), skipping unchanged programs, one connection per arm and both arms in parallel in the background. For the V6 package that means about 55 s in parallel against 1 min 46 s one arm after the other. |

### Other key findings

1. **Two optimisers, one rule.**
   - ALIX already has a hardware-proven TOPP-RA chain for taught paths: pause removal, zero-phase smoothing, a time-optimal timing within the joint ratings and 600 °/s², then 5 ms integer lines. It took commode C's lid top from 62.1 s to 19.2 s.
   - On the same recordings TOPP-RA reaches 1.6–2.0× shorter times than the default algorithm, with similar path change.
   - The default (prototype) algorithm has three measured defects (a jump at the ends, speed spikes at window boundaries, and a 101-point smoothing window that moves the pad 9–21 mm). Three small fixes keep it the default: mirrored edges, blended speed at boundaries, and a smaller window with a path-change check.
   - Both optimisers follow the same rule: if the target time is feasible, stretch the timing uniformly to reach it exactly. Otherwise return the shortest feasible time and say which limit stopped it.
   - Both obey the same hard limits: speed within 0.97 × rating, acceleration 600 °/s², at most 10° per line, a joint-limit margin, and path change of at most 10 mm.
2. **The IO E-stop is the safety stop, but Lite must watch it.**
   - The button is on IO3 of each controller, set to the controller's "input emergency stop" mode. It stops the arm with power kept and refuses new arm and pole commands until the button is released. RealMan's demo maps IO3 to "continue program", so Lite checks the mode at connect.
   - The hand stays powered and keeps working, and a moving pole may not stop, so Lite must freeze both itself.
   - No error code is raised. Runner v14 would most likely report an E-stopped run as "complete". In a sequence, the next program could start once the button is released.
   - Lite therefore reads the E-stop input on each controller, blocks motion while it is pressed, ends runs and sequences as "interrupted", and leads a recovery that checks the move back to the program's start.
   - A drill per arm and with both arms confirms the remaining unknowns: the input's active level, one chain or two, and whether the drives lose enable.
3. **Offsets (R9) need a new solver.** Existing IK solvers make the taught paths jump. Near straight-elbow passages the full offset is physically impossible, so the offset must taper smoothly and Lite must report what was achieved. A prototype solver gave zero jumps on four real cases.
4. **"SIM" in the control app is the controller's own dry run, not a 3D simulation.** It runs on the real controller and does not simulate the pole or the hand. Lite names three levels: *3D simulation → robot dry run → real robot*.
5. **Walls.**
   - The commode, bin and poles come from the URDF and the scene. The walls sit in separate venue files and are not checked today.
   - Lite checks against a single cubicle: 1400 mm wide by default, 800 mm selectable. ALIX has single-cubicle meshes for 800 and 1000 mm only. Its 1400 mm venue is the whole washroom, so Lite builds the 1400 mm cubicle the same way (back wall 1.215 m in front of the robot base, inner side faces at ±0.70 m).
   - The 800 mm walls produced false wall hits on a recording that ran fine in the wider cell, which supports the 1400 mm default. The cubicle is checked once on site against known-good recordings.
6. **Region data.** The regions deck defines 22 steps and 42 teachable units on commode C. 34 are in the first release; glove and bin tasks wait. 16 units have never been taught, and no per-region raw recording exists yet (Appendix A).
7. **Jetson.** PyPI has no aarch64 build of `toppra` 0.6.10. The production bundle must compile it, and Lite still works without it, offering the default optimiser only.

### Recommended changes to the requirement set

- **Add three requirements:**
  - **NFR-SAFETY** — the IO E-stop as the primary stop, monitored by Lite; one connection owner per arm; checks tied to exact file versions.
  - **NFR-OFFLINE-DEPLOY** — Lite runs without internet, ROS or the full ALIX repository, on both computers.
  - **NFR-DATA** — a project folder, raw recordings that are never edited, a provenance record per file, and checksums.
- **Keep X1 and X2, with one clarification:** exclude the features, but keep the kinematics and collision engine underneath them.
- **Re-word the four "borrow from ALIX Studio" notes** to point at what actually exists (the table above).

### Size and schedule

Each requirement carries a relative size (S to XL). The schedule is in the implementation plan. Since 13 October 2026 it starts with the UI/UX work, and the backend follows from its checkpoint (D27):

| Milestone | Agent build time | Robot / lab time |
|---|---|---|
| **UI/UX first:** the operator workflow screens on the ALIX Studio theme, working against the existing optimiser and stand-ins for the robot | within Release 1 | none |
| **Backend:** optimiser golden tests and integration, 3D check with walls, robot I/O with the IO3 E-stop monitor, the server and API | within Release 1 | within Release 1 |
| **Core loop:** teach, optimise (both optimisers, target time), 3D check with walls, upload and run | about 2 working days | a supervised dry run |
| **Release 1** | about 5 working days in total (4–6) | about 3.5 lab days |
| **Release 2:** merging, dual-arm, offsets, robot agents | about 3 more days (2–4) | about 1.5 lab days and an operator field trial |

Robot time, operator trials and sign-offs do not shrink with coding speed. A conventional single-developer sizing (about 280 person-days) is kept in the plan for reference only.

**Of the 23 items**, 17 can be partly borrowed from ALIX Studio, 1 can be borrowed with adaptation (the theme), 3 must be built new (R7, R12, R13) and 2 are exclusions. None can be borrowed as-is.

### Decisions still open (details in section 11)

1. **Region catalogue:** the codes and first-release scope are agreed. Still to fill in: a target time per unit, the pole and hand values, and the teaching order (Appendix A and `region_catalogue_commode_c.csv`).
2. **Cubicle check:** confirm once on site that the cubicle is 1400 mm wide and that its back wall sits where the model puts it (1.215 m in front of the robot base).
3. **E-stop drill and software STOP test:**
   - the active level of IO3;
   - whether one button trips both arms;
   - what the drives do;
   - whether the software slow stop ends a running stored program.
4. **Hand presets and contact rules** for the GRIP hand.
5. **Jetson screen:** does it have its own? Otherwise the UI is opened from the operator PC through an authenticated connection.

---

## 2. What ALIX Studio Lite is

**Revised scope statement.** ALIX Studio Lite is a light, guided application for **non-technical operators**. It takes drag-and-teach recordings of each cleaning region (joint-space `.txt` files), turns them into smooth, faster and safe robot programs, checks them in a 3D dual-arm simulation, tests them on the real robot, and uploads them to the two RealMan controllers. It keeps the look of ALIX Studio and reuses selected ALIX Studio components, the trajectory-optimisation prototype's processing flow, and the robot functions of the upload-channel control application.

The recordings are taught slowly and contain many stops, which is why every recording must be optimised (idle stops removed, smoothed, sped up) before it is used.

**Settled context** (decisions of 13 October 2026)

- **Fixture:** ALIX *commode C*. The regions deck describes commode C, and its pose, lid/seat articulation and the robot (bin, poles, arms, hands) come from the ALIX URDF and the `commode_c` scene.
- **Computers:** Lite runs on the operator PC (x86-64 Linux) now and on the robot's Jetson AGX Orin (aarch64) in production.
- **Code location:** `alix_studio_lite/` in this workspace (not inside the ALIX monorepo); these documents live in `alix_studio_lite/docs/`.

**In scope**

- Guided, region-by-region teaching with pole height and hand position set per region (R1, R7).
- Trajectory optimisation per region and sub-region with **two optimisers**: the *default* (the trajectory prototype's algorithm) and a *time-optimal* one (TOPP-RA). Each tries to reach the operator's **target time** and, if the limits do not allow it, gives the closest feasible time. Includes undo, spike analysis and adjustable plots (R2, R6, R11, R12).
- 3D dual-arm simulation with collision detection linked to the joint plots (R3, R5, R10).
- Offsets through forward and inverse kinematics, tested in simulation first (R9, R14).
- Merging trajectory files into one program, checked against the commode scene (R8).
- Connecting to the real arms, testing, uploading and running programs (R4, R13).

**Out of scope** (from the inputs)

- **X1** — geometric path auto-generation and editing (Lite starts in joint space from taught `.txt` plans).
- **X2** — the C-Space and Workspace features.

**Users.**

- **Operator** (non-technical): teaches regions, runs simulations and tests, reports what went wrong.
- **Supervisor / engineer**: sets region parameters (cleaning time, pole height, hand preset), approves real-arm runs, manages controller programs and advanced settings.

---

## 3. Sources reviewed — what each one does

A high-level summary of every source. Details sit in the per-requirement findings.

| Source | What it is and what it does | What it means for Lite |
|---|---|---|
| **ALIX Studio back end**<br>`studio/server.py` | Local Python web server (standard-library HTTP server, about 64 JSON API routes). It generates and validates cleaning-task configs, computes robot poses (forward kinematics) and collision sweeps through `alix_workspace`, and serves the single-page UI. Long jobs run as start / poll / cancel jobs. **It never commands the robot.** Start-up takes 30–60 s because it loads the whole path generator. | Copy its thin server pattern and job model. Do not run Lite inside it. |
| **ALIX Studio front end**<br>`studio/web_editor/` (`template.html`, `editor.js`, `editor.css`) | One offline web page: a 3D viewport (three.js) showing the robot and the fixture; tabs *Regions, Board, Workbench, Dual-Arm Collisions, Schedule & Run, Workspace, Gizmo*; *Author* and *Simulate* views. It has a tested design-token theme (brand blue `#2F96EE`, light and dark), resizable docks, draggable dialogs, undo/redo, a status line, toasts and confirmation dialogs. | Reuse the theme and selected components (3D viewport and robot posing, collision timeline, docks, board table, confirmation dialog). Leave out the authoring, C-Space and Workspace parts. |
| **ALIX kinematics and collision**<br>`alix/core/python/alix_workspace`, `studio/body_collision.py` | ROS-free forward/inverse kinematics on the robot's flat URDF, dual-arm collision sweeps, and a full collision world (both arms, hands, bin, poles, walls, commode in its three states). | Import as a library for simulation, collision checks and offsets. |
| **ALIX robot I/O and taught-track tools**<br>`alix/core/python/alix_dispatch`, `alix/tools/taught_tracks.py`, `alix/docs/motion/` | A library to upload, run and stop programs and switch run mode on the RealMan controllers; certification gates (G1–G5) for taught tracks; measured findings on upload speed and the controller's playback clock. | Basis for faster, safer uploads and for automatic validation. |
| **Trajectory-optimisation prototype**<br>`lite/Alix_Studio_Lite_Trajectory_V2.html` | A single web page that loads **one** taught `.txt`, removes idle stops, smooths it, speeds it up (globally or per time window), suggests speed changes, compares original and optimised, and downloads the new `.txt` plus a report. | Keep its guided six-step flow and comparison views; move the processing to the server and fix the algorithm issues found (R2, R11, R12). |
| **Control application — UI v53**<br>`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py` | Tkinter desktop app. **Page 01** has LEFT and RIGHT arm panels with tabs RUN, POINTS, DRAG TEACH, HAND/POLE and TELEMETRY. **Page 02** builds dual-arm sequences (program / pole / hand / wait / global-point steps, TOGETHER / RUN FIRST / WAIT OTHER rules, a manual lid checkpoint). Every action runs the runner below as a separate process. | Its robot functions become Lite's robot features; the UI itself is rebuilt in Lite's web UI with the ALIX Studio theme. |
| **Control application — runner v14**<br>`lite/upload_channel/rm_upload_fast_v14_safe_transit.py` | Command-line back end on the RealMan SDK. It uploads a `.txt` to a controller program slot and verifies it, runs programs by ID, runs multi-program sequences, records and replays drag-teach, moves the pole and hand, switches the controller between SIM and REAL run modes, and moves to stored global points. | Wrap it first (and add IP entry), then turn its core into a library. |
| **Helpers**<br>`pole.py`, `hand.py`, `usage.py`, `upload/rm_upload.py` | Pole-lift and Inspire-hand control helpers, a command-line wrapper for them, and the original 344-line uploader the runner grew from. | Reusable as-is. |
| **Control-app history**<br>`lite/upload_channel/py programs/` | Older UI versions v8–v48 and runner versions v4–v12. | Archive only. |
| **Operations notes**<br>`lite/RealMan Dual-Arm Operations UI.txt` | Feature description of the control app (it states UI v50 / runner v13). | Accurate for Page 01; out of date for the Page 02 dual-arm rules and the v51–v53 additions. |
| **Regions deck**<br>`lite/Toilet_regions_V1.pptx` | 25 slides: fixture states, two named region maps, and 22 dual-arm task steps with LEFT (blue) and RIGHT (orange) region overlays. | The region list for teaching and for the dashboard (Appendix A). |
| **V6 merged package**<br>`lite/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/` | Time-matched, merged LEFT and RIGHT programs (IDs 71–83) joined with joint-space bridges, plus workflow JSONs and reports. | Legacy reference only: its bridges are not collision-checked and several files exceed the arms' joint-speed ratings. |

> **Is `rm_trajectory_ui_v48_handoff_continue_fix.py` the most comprehensive version? — No.**
> Three newer UI versions sit in `lite/upload_channel/`: **v51** (safe start speed), **v52** (timeline rules) and **v53** (safe lid checkpoint). **UI v53 with runner v14 is the current pair**, and it contains every v48 feature. It is also the only version that can run the V6 production workflow; v48 rejects it. The two `v48` files (`upload_channel/` and `py programs/`) are byte-identical.
> **Use v53 + v14 as the baseline.** Issues that affect the plan, one line each:
> - *Multi Run* and Page 01 *Load Sequence* fail with a `TypeError` (methods missing `@staticmethod`).
> - There is no IP address entry; the IPs are fixed labels (the runner itself accepts `--ip` / `--port`).
> - The UI does not check that the runner is v14; an older runner would run approach moves at 100 % speed.
> - A few older features were dropped along the way: saving a drag recording to the controller as a `DT_` program (the runner still supports it), the dual-run REAL-mode warning, the simple %-open hand slider and the Page 02 run-mode selector.

---

## 4. Glossary

| Term | Meaning in this document |
|---|---|
| **Region** | One teachable unit from the regions deck: one arm, one area or action. Codes follow the deck's own format, for example `L-R05` SEAT TOP PART 1 (file key `L-R05_PART1`). Lid and seat operations (`L-T01`, `R-T03`, …) are units too, but they are not cleaning tasks (Appendix A). |
| **Sub-region** | A time window inside one region's trajectory, for example seconds 4–9 of `L-R05` SEAT TOP PART 1. The trajectory prototype calls these "scaling regions". |
| **Raw recording** | The `.txt` saved by drag-and-teach: one line per point, `{"point":[j1..j7]}`, seven joint angles in integer millidegrees. Never edited. |
| **Recipe** | The ordered list of edits (trim, smooth, scale, offset, …) applied to a raw recording. Undo removes steps from the recipe. |
| **Optimised trajectory** | The `.txt` produced by applying a recipe to a raw recording. |
| **Program / slot** | A trajectory stored on a RealMan controller under a program ID (1–100). Like the control program, Lite can upload to any ID and replace an existing program after a confirmation. |
| **Target time** | The duration the operator sets for a region's optimised trajectory. The optimiser reaches it when the limits allow; otherwise it returns the closest feasible time and names the limit that binds. |
| **Default optimiser** | The trajectory prototype's algorithm (remove idle stops → smooth → scale), kept as the default. |
| **Time-optimal optimiser** | TOPP-RA (time-optimal path parameterisation, the `toppra` library already used by ALIX tools): the fastest timing along the same path within the joint speed and acceleration limits. |
| **IO E-stop** | The emergency-stop buttons wired to an input on each arm controller (left and right). They stop the robot without cutting power and are the primary safety stop. |
| **Cubicle** | The wall model Lite checks against: a single cubicle, not the whole washroom. It is 1400 mm wide by default and can be switched to 800 mm. ALIX has an 800 mm cubicle file; Lite adds a 1400 mm one built the same way. |
| **3D simulation** | Lite's own playback of a trajectory on the robot model, with collision checking. No robot is involved. |
| **Controller SIM mode** | The RealMan controller's own dry-run mode. The controller runs the program without moving the motors. The pole and hand are not simulated. |
| **Emulator** | ALIX's software stand-in for a RealMan controller, used by engineers for testing without hardware. |
| **REAL mode** | The controller drives the real arm. |
| **Fixture state** | The commode state a region needs: *lid closed*, *lid open* (seat down) or *seat open*. |
| **Pole height** | Height of the arm's lift column, 0–315 mm. |
| **Hand preset** | A named set of six Inspire-hand finger values, 0 (closed) to 1000 (open), for example GRIP = `0,0,0,0,150,1000`. |
| **Transit speed** | The speed of the controller's own approach move to a program's first point. It does not change the speed of the program itself. |
| **Player clock** | The controller plays at most one point (one line) per 5 ms cycle. This is Lite's only time base: a file of N lines lasts (N − 1) × 5 ms, from its first line to its last, target times are in those seconds, and every limit is judged at 5 ms. |
| **Joint ratings** | The arm's joint speed limits: 180, 180, 225, 225, 225, 225, 225 °/s for joints 1–7. |
| **Gates G1–G5** | ALIX's automatic checks for taught tracks: joint-limit margin, joint speed, step size per line (≤ 10°), path deviation and predicted executed deviation. |

---

## 5. The operator journey

Lite follows one guided path. Each step can send the operator back to an earlier one when a problem is found.

```
  1 TEACH            2 OPTIMISE           3 SIMULATE            4 TEST ON ROBOT       5 UPLOAD & RUN
  region by region   target time; default both arms in 3D,      controller SIM mode,  any program ID,
  pole + hand set    or time-optimal,     commode + walls,      then REAL; IO E-stop  overwrite after
  timer, auto-next   undo, spikes, plots  collisions on plots,  and STOP always       confirmation,
  (R1, R7)           (R2, R6, R11, R12)   bin toggle            available (R4)        merged programs
                                          (R3, R5, R10, R14)                          (R8, R13)
      |                   ^      |              |     |                |
      |                   |      +--------------+     |                |
      |                   |       issue found in sim  |                |
      |                   +---------------------------+----------------+
      |                     issue found on the robot (collision, missed spot, ...)
      +<---- re-teach when the recording itself is unusable
```

Offsets (R9, R14) are an optional step inside *Optimise*, always checked in *Simulate* before the robot.

---

## 6. Summary of verdicts

The verdict column is about **ALIX Studio only**. Other sources (control app, trajectory prototype, ALIX libraries) are listed separately; each requirement's section gives the details. Size is relative (S, M, L, XL).

| ID | Requirement | ALIX Studio | Main other sources | Note in the inputs | Priority | Size |
|---|---|---|---|---|---|---|
| [R1](#r1--region-wise-data-collection) | Region-wise data collection | Partial | Control-app drag teach (adapt); ALIX `SOURCE.json` provenance (as-is) | No borrow claim | Must | XL (MVP: L) |
| [R2](#r2--trajectory-editing-per-region-and-sub-region-with-two-optimisers-a-target-time-and-undo) | Trajectory editing, two optimisers, target time, undo | Partial | Prototype algorithm = default optimiser (adapt, 3 fixes); ALIX TOPP-RA chain = time-optimal (import) | Points at the prototype: kept as the default | Must | L |
| [R3](#r3--simulation-playback) | Simulation playback | Partial | `alix_workspace` (adapt); `.txt`-to-plan adapter (adapt) | "Borrow": partly accurate | Must | L |
| [R4](#r4--real-arm-testing) | Real-arm testing | Partial | Runner v14 (adapt); `alix_dispatch` (adapt); IO E-stop monitoring (new) | IP entry missing in the UI; "sim" is the controller dry run | Must | XL (MVP: L) |
| [R5](#r5--collision-detection-and-graph-highlighting) | Collision detection and graph highlighting | Partial | `alix_workspace` (adapt); venue walls (adapt); `alix_dispatch` 5 ms capture (adapt) | "Borrow": partly accurate | Must | XL |
| [R6](#r6--adjustable-plots-and-editing-windows) | Adjustable plots and windows | Partial | Prototype Plotly set-up (adapt) | Points at the prototype: fixed-size plots | Should | M |
| [R7](#r7--pole-height-and-hand-position) | Pole height and hand position | **Build new** | Runner v14 + `pole.py`/`hand.py` (adapt); `usage.py` (as-is) | "Borrow": mostly inaccurate | Must | M |
| [R8](#r8--txt-file-merging) | TXT file merging | Partial | ALIX joins and gates (adapt); V6 package (test data) | "Mechanism exists": partly accurate | Must (parts Should) | XL |
| [R9](#r9--offset-adjustment-using-ik) | Offset via FK → offset → IK | Partial | `alix_workspace` FK/Jacobian (adapt); new solver | No borrow claim | Should | L |
| [R10](#r10--dual-arm-speed-optimisation-split-view-correct-model-bin-toggle) | Dual-arm speed view, URDF, bin toggle | Partial | ALIX URDF (as-is); mesh bake (re-run); target-time readout from both optimisers | "Borrow": partly accurate | Should (parts Must) | L |
| [R11](#r11--smoothness-adjustment-for-both-optimisers-with-an-immediate-tiered-simulation-check) | Smoothness with immediate validation | Partial | ALIX smoothing presets and checks (adapt) | No borrow claim | Should (parts Must) | L |
| [R12](#r12--spike-limit-and-sudden-motion-analysis-with-one-click-fixes) | Spike analysis and suggestions | **Build new** | ALIX G1/G2 detectors (port) | No borrow claim | Should (parts Must) | L |
| [R13](#r13--faster-trajectory-upload) | Faster upload | **Build new** | Runner v14 upload path: any ID 1–100, replace after confirmation (as-is); `alix_dispatch` (adapt) | "Check ALIX findings": accurate | Must (parts Should) | M |
| [R14](#r14--offset-calculation-and-testing) | Offset testing in simulation | Partial | ALIX `BodyCollisionWorld` (adapt) | No borrow claim | Should | L |
| [X1](#x1--exclude-geometric-path-generation-and-editing) | Exclude geometric path generation | Not applicable | — | Accurate | Must | S |
| [X2](#x2--exclude-c-space-authoring-and-workspace-analysis) | Exclude C-Space and Workspace | Not applicable | — | Accurate; keep the engine | Must | S |
| [NFR-UX](#nfr-ux--very-user-friendly-for-non-technical-operators) | Very user-friendly for non-technical users | Partial | Prototype stepper (adapt) | — | Must | L |
| [NFR-THEME](#nfr-theme--keep-the-alix-studio-theme-and-formatting) | Keep the ALIX Studio theme | **Borrow + adapt** | ALIX design tokens (as-is) | Accurate | Must | M |
| [NFR-SAFETY](#nfr-safety--safe-robot-operation) | Safe robot operation (new) | Partial | IO E-stop (monitored by Lite); control app, `alix_dispatch` (adapt) | New | Must | XL |
| [NFR-OFFLINE-DEPLOY](#nfr-offline-deploy--offline-self-contained-deployment) | Offline deployment on both computers (new) | Partial | Operator PC now, Jetson in production; `toppra` built for aarch64 | New | Must | L |
| [NFR-DATA](#nfr-data--project-data-provenance-and-integrity) | Project data and provenance (new) | Partial | ALIX taught-lane rules (adapt) | New | Must | L |
| [INT-TRAJ](#int-traj--trajectory-optimisation-tab-built-from-the-prototype) | Integrate the trajectory prototype | Partial | Prototype flow and default algorithm (adapt); ALIX TOPP-RA (import) | Right base; not production-ready as is | Must | L |
| [INT-CTRL](#int-ctrl--control-application-integration-ui-v53--runner-v14) | Integrate the control app | Partial | Runner v14 (as-is); v53 planner (port) | v48 is not the latest | Must (parts Should) | XL (MVP: L) |

**Totals (ALIX Studio column):** Borrow as-is 0 · Borrow + adapt 1 · Partial 17 · Build new 3 · Not applicable 2.

---

## 7. Functional requirements R1–R14

### R1 — Region-wise data collection

**Original (inputs):**
> Region-wise Data Collection: The UI should prompt the (non technical person) user to clean each region with a specified cleaning time. Once a region is completed, the system should automatically prompt the next region. (Regions Tab -> Regions PPT and the Dashboard to show the regions with and withput the taught raw txt)

**Revised requirement.** Lite shall guide a non-technical operator through drag-and-teach recording of every region in the cleaning plan, one region at a time, and shall show which regions already have a kept raw recording.

- (a) **Region catalogue.** Lite holds one versioned record per region, built from the regions deck: ID and deck aliases, arm, step (deck task and slide), type (cleaning region, lid/seat operation, tool, glove or idle), required lid/seat state, tool, picture, target cleaning time, and pole height and hand setting (R7). The target cleaning time is the operator-set **target duration** for the region's optimised program: the optimiser (INT-TRAJ) tries to reach it and, if the joint limits do not allow it, returns the closest time that obeys them and names the limit that binds. A record may also carry an optional teach time set by a supervisor (see (b)). Mechanism, tool and glove tasks list ordered sub-steps, each taught as its own recording, with the hand events between them. `Toilet_regions_V1` gives 42 regions: 22 LEFT and 20 RIGHT. The deck describes commode C, so the catalogue's fixture is ALIX `commode_c`, and lid/seat states and fixture pose come from its scene (`alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml:15-40`).
- (a2) **First-release scope (decided 13 Oct 2026).** Codes follow the deck's own format (`L-R04`, `R-T03`; parts written `PART 1`, file key `L-R05_PART1`). The first release holds 34 units: 26 cleaning regions and 8 lid and seat operations (open lid + seat, close seat only, open seat, close seat + lid; the left arm is MASTER and the right arm SUPPORT). Glove tasks and bin tasks (pick, hold and return the drain brush; drop and change the glove) are excluded for now and stay listed as *Not in this release*. Lid and seat operations are **not cleaning tasks**: their card says what to operate (for example "Open the seat — left arm leads, right arm supports"), there is no "clean for N seconds" prompt (the target time is the duration of the operation), they change the toilet state for the regions that follow, and the dashboard counts them apart from cleaning coverage (Appendix A).
- (b) **Guided recording.** The operator chooses Left, Right or Both (Both needs two people). Lite shows the next region without a kept recording as a card: the deck picture with the area highlighted, a plain instruction, the arm, "part n of m", the tool and the time. The operator presses **Prepare** (R7), then **START**. While recording, Lite shows a countdown as guidance and sounds at 10 s left and at 0 s. The countdown uses the region's target duration unless a supervisor has set a separate teach time for that region; the card labels which one it shows. Recording continues after 0 s until STOP, and a configurable hard limit stops it automatically after a 10 s warning. Opening a card or auto-advancing never moves the robot.
- (c) **Save and check.** On STOP, Lite saves the controller's recording under a temporary name and checks it: the line format, at least 2 points, and the controller's point count equal to the number of lines. It then renames the file to a new name made of the region ID and the time; an existing file is never replaced. A provenance sidecar records arm, region, deck version, lid/seat and bin state, pole and hand (target and measured), operator, date, controller IP, tool frame, target and elapsed time, point count, measured record period and checksum. Problems are reported in plain words.
- (d) **Review and auto-advance.** The operator chooses Keep or Redo. A redo keeps the earlier attempt, marked rejected. Skip needs a reason. After Keep, the next card opens without further input. Lite skips kept and blocked regions, inserts lid/seat change prompts, and lets a session pause and resume.
- (e) **Regions tab and dashboard.** The Regions tab shows the deck overview maps (PAGE 04/05) coloured by status, the step list grouped by lid/seat state, the region card and, on request, the original slide. The dashboard shows a region × arm table with Not taught, Awaiting review or Taught (attempts, date, duration), the later pipeline states (optimised, simulation OK, real-tested, uploaded) when their records exist, counters per arm and phase, a "missing only" filter and a "Teach next missing" action.
- (f) **Interlocks and recovery.** Lite shows the IO E-stop state of each arm and does not start drag teaching while an arm reports it; after an E-stop it guides recovery (release the button, confirm the arm state, then save or discard any recording). In Both mode one START puts both arms in drag mode and STOP saves two separate files. While any arm is in drag mode, Lite sends no other robot command, and STOP is always available. During a session no other client may drive the arms (the ALIX ROS stack, the v53 app or a second Lite), and only one browser session holds control. If Lite stops while an arm is in drag mode, on restart it takes the arm out of drag mode and offers to save the recording the controller still holds.
- (g) **Practice mode.** A simulated recorder for training and tests writes to a separate practice folder. Its files are never counted as taught.

**Findings**
- **ALIX Studio:** The Studio Regions tab is a mesh-painting editor for the excluded path generator (X1): "Mark regions once …, then Generate creates a config per region". It writes `regions.yaml`, cannot show the deck and should not be ported (`studio/web_editor/template.html:156-201`).
- **ALIX Studio:** The Board tab has the right look for the dashboard: summary chips, a `present/expected` badge on the tab, and a region × arm table of pills that pair a glyph with a word. Its roughly 150 lines of JavaScript and its backend are tied to the generator (point counts, routes, gate verdicts, "Generate missing", generated `*_cleaning_points.yaml` files), so only the CSS and patterns carry over (`studio/web_editor/editor.js:15208-15289`; `studio/server.py:5396-5486`).
- **ALIX Studio:** Studio has no wizard, timer or robot channel, by design: "Grouped by purpose, not numbered" and "the studio has no execution channel … nothing here commands hardware". R1 is a linear guided flow that drives the arms, so it is new work (`studio/web_editor/template.html:116-118`; `studio/web_editor/editor.js:13045-13047`).
- **ALIX Studio:** Two parts are reusable. One is the feedback toolkit: status line, toast, the `showStudioModal` confirm dialog, glyph-plus-colour chips, and a coverage bar with a target marker that can serve as the countdown bar. The other is the `schedule_guards` "reality-logical" checks, which confirm that mechanism steps operate only parts in the right state and that each cleaning region runs only in its declared lid/seat state (`studio/schedule_guards.py:8-12, 196-236`).
- **Control app v53/v14:** Drag teach exists end to end, per arm and for both arms: start, stop, save, playback, and a two-arm start behind `threading.Barrier(2)`. The runner prints `DRAG TEACH ACTIVE`, `DRAG TEACH STOPPED` and `DRAG SAVE COMPLETE: <path> // <N> points`, which Lite can parse. No test record was found; the V6 package and the UI history indicate use on the arms (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:912-989`; `lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:7841-7935`).
- **Control app v53/v14 (gaps that block the plan, one line each):**
  - The save is tried once, written straight to the final path and not checked, while ALIX saw the save lag and retries every second (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:957-989`).
  - v53 sets the save path once and never resets it, so a second save overwrites the first unless the operator edits the path.
  - The runner checks nothing before drag teach: error codes, a running program, all joints enabled.
  - Drag-session processes run in their own session, so after a Lite crash they keep the arm in drag mode; the controller still holds the recording (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:5745-5750`).
- **ALIX taught tracks and exp01 notes:** ALIX already has the provenance format Lite needs: a `SOURCE.json` beside each recording (`side, frame, pole_m, hand_state, hand_counts, hardware, taught_by, date, units, note`) and the rule that source files are never edited. The ALIX emulator implements no drag-teach or save call, so Lite needs its own fake recorder (`alix/Resource/tracks/taught/source/README.md:3-27`; `alix/experiments/exp01_teach_trajectory_reverse_engineering/notes/2026-08-25_phase_a.md:93-98`).
- **Fixture and cell (decided 13 Oct 2026):** The deck's toilet is commode C, which ALIX models as `commode_c` with `display_name: "Commode C"`, lid and seat articulation and a default pose of x 1.005 m relative to `base_link`; a venue scene file may override that pose (`alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml:1-40`). The URDF and this scene already hold the cell information, so R1 needs no cell measurement; selecting the venue file belongs to the simulation set-up (R5, R10).
- **RealMan E-stop:** Each arm controller has digital inputs that can be set to "Input Emergency Stop Function Multiplexing" (IO mode 5), and the SDK reads a port's mode and level with `rm_get_io_state(n)`; `rm_get_io_input()` returns -1 for a port that is not in general-input mode, so it may not show a mode-5 port (RealMan Gen3 API2 Python p.159-160; E-stop research §2). RealMan lists "The robotic arm is currently in an IO emergency stop state" as a reason drag teaching fails to start (RealMan Gen3 teach-pendant manual, Digital Input/Output; RealMan Gen3 JSON protocol, `set_IO_mode` and `start_multi_drag_teach`; `alix/external/butterfli-realman-alix/Python/Robotic_Arm/rm_robot_interface.py:2927-2980`). The robot's E-stop buttons are wired to such an input on each controller and stop the robot without cutting power (user decision, 13 Oct 2026); in the ALIX field test the hand kept working during an E-stop, so Lite freezes it (`alix/docs/TEST_PROCEDURE.md:453`). Which input number is used on each controller must be read from the controllers.
- **Regions deck, V6 package and ALIX recordings:** The deck (`lite/Toilet_regions_V1.pptx`, 25 slides) has no cleaning times, no pole or hand values and inconsistent codes (21 of 44 arm-step labels have no code, `FL-R09` breaks the pattern and `R-T08` is missing), so Lite needs its own IDs. No per-region raw recording exists: V6 holds only merged, time-stretched programs covering about half the regions, and 16 regions are not taught at all (`lite/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/README.txt:4-8`). ALIX's raw commode C left-arm recordings, taught on the same fixture model, give default target durations: whole regions 41.7-66.0 s (median 48.2 s), mechanisms 16.5 and 23.7 s, assuming 5 ms per point (`alix/experiments/exp01_teach_trajectory_reverse_engineering/data/teach_files/commode_c_taught_paths/description.txt:1-20`).

**Can it be borrowed from ALIX Studio?** **Partial** — Borrow the Board tab's look (CSS, summary chips, tab badge, glyph-and-word pills, cells that act as controls, the `age()` helper), the feedback components, the theme tokens and the fixture-state checks in `schedule_guards`. The Regions tab and the Board's JavaScript and backend are tied to the excluded generator, and the guided session, countdown, auto-advance and drag recording do not exist in Studio, so they are new work.
- *Other sources:*
  - RealMan SDK IO call `rm_get_io_state` (on the IO in mode 5) — Borrow as-is: per-arm IO E-stop state for the status command (R4 (h), NFR-SAFETY (c)).
  - Control app (runner v14 + UI v53) — Borrow + adapt: `--drag-session`, `--drag-save`, `--drag-playback`, `--get-arm-mode`, the status lines, the confirmation wording, the both-arm barrier start and the distinct-name check; `arm_state` and `_capture_current_frames` for a small status command.
  - ALIX taught lane (`SOURCE.json`, `alix/tools/taught_tracks.py`) — Borrow as-is (schema and rule): the field set and "raw is never edited", extended with a `lite` block.
  - Trajectory prototype — Borrow + adapt (pattern): the six-step stepper with lock rules (`lite/Alix_Studio_Lite_Trajectory_V2.html:330-340`), re-skinned with Studio tokens.
  - Deck renders, crops and the normalised ID scheme (Appendix A) — Borrow + adapt: turn this analysis's scripts into an offline `deck_import` tool owned by Lite.
  - ALIX commode C raw recordings — Reference: default target durations and optional reference imports (same fixture as Lite, `commode_c`).
  - V6 package — Reference: legacy coverage flags on the dashboard only.
- *Note on the inputs:* Partly accurate. The input makes no explicit borrow claim for R1; it names a "Regions Tab" and a "Dashboard". Studio has a Regions tab, but it is a mesh-painting editor for the excluded generator, so only the name carries over. The Board tab is a good model for the dashboard's look, but its code reads generated configs, not taught `.txt` files.

**Recommendation.** Build R1 as a new guided **Teach** flow in the Lite web shell, around a Lite-owned, versioned region catalogue that is generated offline from the deck and signed off by the requester. From Studio, reuse the Board tab's look and the feedback components, but write the dashboard code and backend new, and do not port the Studio Regions tab. Drive drag teaching through runner v14 behind a `TeachDevice` adapter (subprocesses for the MVP) and add what the runner lacks: preconditions, Lite-chosen file names that are never overwritten, a save retry with a point-count check, an ALIX-compatible sidecar with the measured record period, process supervision and crash recovery. Show the IO E-stop state of each arm and block drag teaching while it is active. Teach mechanism, tool and glove tasks as sub-steps. Seed target durations from the ALIX commode C recordings (same fixture) for a supervisor to own; the same number drives the teach countdown and the optimiser (INT-TRAJ), with an optional supervisor teach time per region. We recommend one number by default: the original asks the operator to clean "with a specified cleaning time", one value is simpler to agree and sign off, and the countdown is guidance only (recording continues past 0 s) while the optimiser, not the teaching pace, brings the program to the target. A separate teach time is worth setting only where operators rush delicate steps, for example lid and seat mechanisms. If time is short, ship an MVP: crops instead of drawn highlights, deck order only, a table-only dashboard, tool and glove regions BLOCKED, and resume limited to crash recovery.

**Approach**
1. **Agree the model with the requester (no code).** Terms, IDs, the sub-step model, target durations (and any teach times) and teaching order, and fixes for the deck issues (`FL-R09`, uncoded labels, `R-T04` repeated on 10B, the drain-hole parts, the missing `R-T08`).
2. **Deck import, offline (build; ports this analysis's scripts).** `deck_import.py` unzips the PPTX and reads the slide XML with the standard library: label text, fill colour (the arm), shape positions (highlight boxes) and media. It proposes IDs and writes a draft `regions_<deck version>.yaml` that a person reviews and commits; a re-import prints a diff instead of overwriting. Slides and per-arm crops are rendered offline (LibreOffice headless, `pdftoppm`) and shipped, so Lite does not need LibreOffice at run time.
3. **Catalogue, data layout and status engine (build; two Studio rules borrowed).** A validator refuses duplicate IDs, missing fields and unknown states. Raw files sit read-only in `units/<ID>/raw/` beside their `.SOURCE.json`; Keep, Redo and Skip decisions go to an append-only `events.jsonl`; practice data has its own root. Status is derived from files at each refresh ("only what is on disk"), and a later status counts only if its record names the current file's sha256 ("stale is never passing").
4. **Hardware adapter (borrow + adapt runner v14; build the rest).** A `TeachDevice` interface (`status`, `start_drag`, `stop_drag`, `save`, `apply_pole`, `apply_hand`, `read_pole_hand`). `RunnerTeachDevice` starts runner v14 as v53 does (own session, SIGINT to stop) but always passes `--ip` and `--sdk`. Add a small status command (error codes, run mode, program running, tool frame, IO E-stop input level from `rm_get_io_state` on the IO in mode 5, as in R4 (h)) built from the runner's `arm_state`, `get_arm_run_mode` and `_capture_current_frames`. Store child process IDs and add a stop-drag fallback: a fresh process calling `rm_stop_drag_teach` (verify on hardware that a second connection can do this). `FakeTeachDevice` replays an existing raw file at real-time pace and can simulate an E-stop.
5. **Teach session state machine (build).** It is persisted in `sessions/<id>.json`: select arms → card → fixture check → Prepare (R7) → ready → recording → stopping → saving → checking → review → keep, redo or skip → next, plus a recover state on start-up and an E-stop state entered whenever an arm reports the IO E-stop. The countdown starts on `DRAG TEACH ACTIVE`, not on the button press.
6. **Ordering and auto-advance (build; fixture-state logic borrowed from `schedule_guards`).** Default order is by arm within each lid/seat phase (fewest changes); deck order is an option. Kept and BLOCKED regions are skipped. A lid/seat prompt with the deck photo is inserted when the state must change. Mechanisms are taught as alternating MASTER/SUPPORT sub-steps: no Prepare for an arm holding the lid or seat, and a redo invalidates the later sub-steps. Tool and glove tasks are split at hand events. After Keep, "Next: <title>" shows for 3 s with Pause; Lite never prepares or starts by itself.
7. **Save and check (build).** Wait 1 s, save to `<final>.partial`, retry up to 3 times 1 s apart, apply the runner's file rules (every line `{"point":[7 integers]}`, at least 2 points) and compare the controller's point count with the lines. Rename only if the final name does not exist, set the file read-only, compute sha256 and write the sidecar: ALIX keys first, so `taught_tracks.py` can read it, then a `lite` block. Warnings that do not block Keep: record period more than 30 % from 5 ms, a joint within 5° of a limit, a step above 10° between lines, a recording shorter than 30 % of the target time.
8. **Front end (build on borrowed Studio tokens and components).** Regions tab (PAGE 04/05 label hotspots including Floor Spillage, step list by phase, region card, original slide); Teach view (big card, START/STOP on screen and on one dedicated key or foot switch handled globally, countdown, sounds, overtime, review with a small joint plot); dashboard in the Board look (chips, tab badge "n/42", "missing only", "Teach next missing"); supervisor settings (target duration, optional teach time, pole, hand, BLOCKED); a per-arm E-stop indicator; the control lock.
9. **Guard rails (build).** No other command while any arm is in drag mode; one teach per arm; the next START disabled until the recording is saved or discarded with confirmation; a REAL-mode check; no START while an arm reports the IO E-stop; Both mode only after a two-person confirmation; the operator confirms that the ALIX ROS stack and the v53 app are closed.
10. **Tests.** Unit tests (validator, status engine, ordering with sub-steps, state machine); end to end in practice mode with `FakeTeachDevice`; crash and recovery tests; hardware acceptance on both arms: single and Both, save retry, STOP during recording, the IO E-stop pressed during a recording (does the controller keep the recording, and can it be saved after release?), killing Lite mid-recording, the physical DRAG button, and one recording of about 5 min to probe the controller's limit.

**Acceptance criteria**
- [ ] **Catalogue.** Loading the `Toilet_regions_V1` catalogue gives all 42 deck units (22 LEFT, 20 RIGHT; 34 in the first release, the other 8 marked Not in this release) with unique keys, all required fields, the fixture `commode_c` and sub-steps in order. Each region's target duration is the value the optimiser receives for that region. A catalogue with a duplicate ID, a missing field or an unknown state is refused with a message naming the region.
- [ ] **Regions tab.** Every region opens with its picture highlighted, a plain title, the arm badge (blue LEFT, orange RIGHT) and, on request, the original slide. Clicking a PAGE 04/05 label, including Floor Spillage, filters the list to that family.
- [ ] **Dashboard from files.** With an empty data folder it shows "0/34 taught" and every region "Not taught". In practice mode, keeping L-R01 shows "Taught" with date, duration and attempts and 1/42 (LEFT 1/22) on the practice dashboard without a restart, while the real dashboard is unchanged; practice files live only in the practice folder and carry `hardware: analysis_only`. A kept raw file removed outside Lite shows "Not taught (file missing)" after the next refresh.
- [ ] **Guided flow.** With the fake recorder and 3 untaught LEFT regions, the operator finishes all three using only Prepare, START, STOP and Keep; the next card appears after each Keep and a summary after the third. The command log shows no robot command when a card opens or auto-advances; pole and hand move only after Prepare, and drag mode starts only after START.
- [ ] **Countdown.** It shows the region's target duration, or the supervisor's teach time when one is set, labelled as such. It starts when drag mode is reported active, sounds at 10 s left and at 0 s, and continues as overtime. At the hard limit, after a 10 s warning, recording stops automatically with an explanation.
- [ ] **Save.** Two recordings of one region give two files, and no file is ever replaced. Raw files are read-only, and each sidecar holds the ALIX keys and the Lite block, with a sha256 equal to its file's. In Both mode, STOP saves two differently named files with two sidecars. An empty or unparseable save, or a point-count mismatch, shows "Recording failed — please redo" and the session does not advance. A failed controller save is retried 3 times, 1 s apart, and the next START stays disabled until the recording is saved or discarded with confirmation.
- [ ] **Review and order.** Redo keeps the earlier file, logged as rejected. Skip requires a reason and shows "Skipped". Redoing S1 of a mechanism flags its later sub-steps "Needs re-teach". When the next region needs another lid/seat state, START stays disabled until the operator confirms it.
- [ ] **Interlocks and recovery.** Teaching does not start while the arm reports SIMULATION mode, error codes, a running program or an active IO E-stop, and Lite says why in one sentence. While any arm is in drag mode, pole, hand, run and upload commands are refused and STOP stays available; in single-arm mode the other arm never enters drag mode. The START/STOP key acts on recording whichever control has focus and triggers nothing else; a second browser session is read-only. Closing and reopening Lite offers "Resume" at the first region without a kept recording. If Lite is killed while an arm is recording, on restart it reports this, takes the arm out of drag mode and offers Save or Discard; Save produces a normal raw file and sidecar. With the fake device, an E-stop during recording shows the E-stop state on that arm, disables START and, after release and confirmation, offers Save or Discard.

**For operators:**
- One region per screen: a large picture with the area highlighted, a plain title ("LEFT arm — wipe the seat top, left half (part 1 of 2)") and progress ("Region 7 of 42 · LEFT arm · Lid open"). No slot numbers, millidegrees or runner terms; supervisor settings sit behind a separate switch.
- The operator's hands are on the arm, so START/STOP sits on one dedicated key or USB foot switch, with sounds at 10 s left and at time up and a timer readable from 2 m. Use Lite's START/STOP, not the arm's own DRAG button, which Lite cannot see.
- Keep or Redo after every recording; Redo never deletes anything. Statuses carry a glyph and a word (✓ Taught, … Awaiting review, · Not taught), never colour alone. Blue and orange mark the arms only, with matching marks on the robot's arms.
- The red E-stop button on the robot stops the arms at once. When it is pressed, Lite shows "E-stop pressed (LEFT arm)" and the steps to continue: make the area safe, release the button, confirm in Lite, then save or discard the recording.
- Errors are sentences with the next action: "The left arm did not answer. Check its power and cable, then press Try again."

**Safety:**
- Entering drag mode makes a loaded arm limp. The operator supports the arm before START until Lite shows "drag active"; at STOP the arm holds where it is (`alix/ros/hw/alix_hw/scripts/teach_mode.py:11-16`).
- One client per arm. The ALIX ROS stack holds the arm at 100 Hz and fights dragging, and the v53 app or a second Lite must not run either. While any arm is in drag mode, Lite blocks every other robot command.
- Always leave drag mode on STOP, window close or crash: Lite supervises the child processes, recovers on restart and has a stop-drag fallback. Run the hardware layer on Linux, because on Windows the stop signal does not reach the runner's stop path (inferred); both target computers, the operator PC and the Jetson, run Linux. Never start a new recording before the previous one is saved or deliberately discarded: the controller keeps only the latest.
- Teaching is a REAL-mode activity. Both-arm teaching needs two trained people. Teach lid and seat mechanisms slowly and confirm the fixture state before and after. The IO E-stop buttons, wired to an input on each arm controller, are the primary safety stop: they stop the robot without cutting power (F4). In the ALIX field test the hand kept working during an E-stop, so Lite freezes the hand itself (`alix/docs/TEST_PROCEDURE.md:453`). Lite's STOP is a convenience; its behaviour is still tested on the robot. Lite shows the E-stop state and guides recovery but never resets the E-stop itself.

| Priority | Size | Depends on |
|---|---|---|
| Must | XL (MVP: L) | R7, R4, INT-CTRL, NFR-UX, NFR-THEME |

**Open questions:**
- Who sets and signs off the target duration per region, and should any region get a separate teach time (recommended only for lid and seat mechanisms)? Are the proposed default targets acceptable: whole surface 60 s, half or part 30 s, floor part 30 s, drain-hole pass 20 s, mechanism 20 s (sub-step 10 s), tool or glove 30 s, idle hold 5 s? They assume 5 ms per point and double at 10 ms; Lite measures the period on every recording.
- At time-up, should Lite only warn (recommended, with a hard maximum) or stop recording?
- Confirm the IDs and deck fixes: is `FL-R09` = `L-R09`? Is `R-T04` on task 10B "pick brush" again, or "hold brush"? Are the two drain-hole parts two passes? What was `R-T08`?
- Teaching order: by arm within each lid/seat phase (fewest changes) or strict deck order?
- Are the lid and seat mechanisms taught for the robot, or done by hand as in V6? Are the brush and glove regions in v1 scope, and where are the brush holder and glove station?
- One operator or two? Is a USB foot switch acceptable for START/STOP?
- Should every recording start from a common drag start pose, as ALIX does?
- Is the ALIX ROS stack ever running on this robot while Lite is used, and who stops it? In production both run on the same Jetson, so Lite may need to detect it.
- Should the review step offer a robot replay? It moves the arm: first to the start at 20 %, then at taught speed.
- Should V6 segments or the ALIX commode C recordings appear on the dashboard as references?
- Lite keeps its data folder on the computer it runs on (operator PC now, Jetson later). How are kept recordings moved to the Jetson at the switch, and should Lite files also be copied into the ALIX taught lane?
- Which digital input is configured as the E-stop input (IO mode 5) on each arm controller, and does the controller keep a drag-teach recording that can be saved after an E-stop is released? Both must be checked on the robot.

---

### R2 — Trajectory editing per region and sub-region, with two optimisers, a target time and undo

**Original (inputs):**
> 2. Trajectory Editing: Each region and sub-region should have independent options to adjust smoothing, trimming, and scaling, with an undo option for up to five previous changes. - Trajectory Optimization Tab (/home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html)

**Revised requirement.** Lite shall optimise each taught region with one of two optimisers and a target time, let the operator adjust **Remove stops** (trimming), **Smoothness** and **Speed** (scaling) for the whole region and on any sub-region, and keep a labelled undo history. A region is one arm's raw recording for one unit in Appendix A (for example `L-R05 PART 1`); a sub-region is a time window inside it.

- (a) **Two optimisers behind one interface (decision of 13 Oct 2026):**
  - **Default:** the prototype's algorithm, trim idle → moving-mean smooth → per-sub-region time scale (`lite/Alix_Studio_Lite_Trajectory_V2.html:1438-1616`, `lite/Alix_Studio_Lite_Trajectory_V2.html:1676-1724`). It keeps the prototype's stages, controls and the meaning of "%", with only the minimal fixes for its measured defects (see Findings), plus limit enforcement.
  - **Time-optimal (TOPP-RA):** ALIX's hardware-proven chain for taught paths, reused, not rewritten (`alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:17-40`). It runs 5 Hz zero-phase smoothing, pause removal (R4), pad arc-length smoothing (30 mm), then a TOPP-RA solve at 0.95 × the joint ratings and 600 °/s², with zero speed at both ends.
- (b) **Target time.** The region's target time is the catalogue's cleaning time (R1), which the operator can override per region. It is in seconds at 5 ms per line (one line per controller cycle), so its line count is the target ÷ 5 ms. For both optimisers:
  - Lite finds **T_min**, the shortest time whose written bytes pass every hard limit in (g).
  - If the target is at or above T_min, Lite hits it to within one line by uniform time scaling (speed ÷ k, acceleration ÷ k²); the path does not change.
  - If the target is below T_min, Lite delivers T_min, says that the target was not met, and names what binds: joint, quantity (speed, acceleration, joint limit or path accuracy) and the time window.
- (c) **Sub-regions.** Any number of non-overlapping sub-regions may override the region's settings; a sub-region that is not overridden inherits them. In the Default optimiser a sub-region's speed is a **time %** (the prototype's meaning), with an optional pinned duration under Advanced. In TOPP-RA it is a **speed cap %** of the ratings, passed to the solver, so braking into the sub-region stays within the limits; "faster than optimal" cannot exist there. Smoothness and Remove stops can be overridden on any sub-region in both optimisers, and any sub-region can keep its stops or add a timed hold.
- (d) Edits are non-destructive. The raw recording is never changed, and every output is recomputed on the Lite server from the raw file plus an ordered recipe that names the optimiser.
- (e) Sub-region boundaries are blended: no position step, and the speed change spread over 0.25 s.
- (f) Each region keeps an undo/redo history of at least its last 5 changes. Each entry has a plain-language label, and the history is saved with the region. Changing the optimiser or the target time is an undoable change.
- (g) **Hard limits, the same for both optimisers,** judged on the integer millidegrees as written, at 5.0 ms per line (`alix/tools/taught_tracks.py:94-102`). A version can go to simulation, real-arm test or upload only if all pass:
  - L1 (G1): commanded positions inside the joint limits, with the ALIX model's predicted executed margin at least 0.5° (`alix/tools/taught_tracks.py:466-480`);
  - L2 (G2): 20 Hz band-limited joint speed at most 0.97 × the ratings 180/180/225×5 °/s (`alix/tools/taught_tracks.py:229`);
  - L3: acceleration at most 600 °/s² (× 1.02 tolerance), read as a 50 ms windowed mean. 600 °/s² is the arm's hardware limit (decided 13 Oct), so Lite makes it hard with no override, although ALIX treats acceleration as advisory;
  - L4 (G3): every step at most 10° per line (`alix/tools/taught_tracks.py:127-128`);
  - L5 (G4): pad path within 10 mm of the **raw** recording at the contact frame (5 mm preferred), or within the region's MAX DEV if that is smaller;
  - L6: the first and last lines within 0.05° of the recorded start and end poses.
- (h) **Checks shown but not blocking:** G5 (predicted executed departure, ALIX default budget 20 mm), a warning only (D11: ALIX's model reads 33–109 mm on the raw commode C recordings, which ran fine on the robot, so it cannot gate a file; a region's MAX DEV tightens L5 instead); jerk; pad speed; and single raw ticks over a rating.
- (i) **Save version** writes a new numbered file with its recipe, library versions, checksums and check results. Earlier versions are kept.
- (j) In a TOGETHER block of a dual-arm step, both arms get the same line count, N = max(target, left T_min, right T_min), and the report names the arm that binds.

**Findings**
- **Trajectory prototype:** Only speed can be set per sub-region. Trimming and smoothing are global settings, and there is no undo, redo or history (`lite/Alix_Studio_Lite_Trajectory_V2.html:729-737`). Sub-regions are defined by row numbers (typed or box-selected), cannot be moved after creation and are not drawn on the plot (`lite/Alix_Studio_Lite_Trajectory_V2.html:1337-1371`).
- **Trajectory prototype, no limits:** Nothing checks ratings, acceleration, steps or limits, and the scale field accepts anything from 1 % (`lite/Alix_Studio_Lite_Trajectory_V2.html:1676-1810`). It counts durations as N × 5 ms for N lines, one period too long (`lite/Alix_Studio_Lite_Trajectory_V2.html:1726-1729`); Lite counts (N − 1) × 5 ms.
- **Trajectory prototype, three measured defects** (measured 13 Oct 2026 on the commode C recordings in `alix/experiments/exp01_teach_trajectory_reverse_engineering/data/teach_files/commode_c_taught_paths/`):
  - **Endpoint snap.** Overwriting the end points gives first or last steps of 966, 1005 and 546 millidegrees on FILE_08, FILE_10.1 and FILE_01.1, larger than any interior step (`lite/Alix_Studio_Lite_Trajectory_V2.html:1513-1514`, `lite/Alix_Studio_Lite_Trajectory_V2.html:1721-1723`). With the snap present, the fastest limit-safe time is 6.7–13× slower than the recording (FILE_08: 473 s instead of 22 s).
  - **Boundary spikes.** Unblended boundaries with the prototype's suggested speeds on FILE_01.1 give one-tick acceleration jumps of 640–919 °/s² against an 80 °/s² median (`lite/Alix_Studio_Lite_Trajectory_V2.html:1568-1616`).
  - **Path bending.** The default 101-line moving average moves the cleaning pad 10.6–20.9 mm (maximum of both directions, at `L_glove_frame_2`); the earlier figures of 9.05 and 10.39 mm were one direction only (`lite/Alix_Studio_Lite_Trajectory_V2.html:1494-1517`). ALIX's G4 limit is 10 mm, 5 mm preferred (`alix/tools/taught_tracks.py:129-132`).
- **Minimal fixes, measured:** Odd-reflection edge padding (no mean correction, no forced end rows) brings the end steps to 56–99 millidegrees with the 101 window. Blending the per-line time step over 0.25 s and resampling on the chords brings the FILE_01.1 boundary peaks to 200–280 °/s² with the duration unchanged; the blend follows ALIX's `retime_lambda` form (`alix/experiments/exp01_teach_trajectory_reverse_engineering/src/local_speed.py:198-219`). A 41-line default window moves the pad 2.9–4.5 mm; 21 lines, 1.1–2.1 mm (measured 13 Oct 2026).
- **ALIX TOPP-RA chain:** `smooth_taught` 5 Hz (`alix/tools/taught_tracks.py:891-912`) → `strip_pauses` (`alix/tools/taught_tracks.py:915-951`) → `TaughtPath` and `arclen_smooth` 30 mm (`alix/experiments/exp01_teach_trajectory_reverse_engineering/src/local_speed.py:78-104`, `…/local_speed.py:139-169`) → `retime_toppra` (`…/local_speed.py:282-324`: `Interpolation` scheme, `seidel` solver, zero speed at both ends), driven at `alix/experiments/exp01_teach_trajectory_reverse_engineering/src/emit_toppra_rung.py:306-330`. On hardware it gave 3.24–3.49× on commode C slots 101-104 and 3.88–3.99× on bowl-out recordings, with the executed pad 2-3 mm p95 from the taught path (`alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:42-55`, `alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:588-591`). These runs were on the CANFD channel.
- **Re-measured 13 Oct (ALIX venv: numpy 2.5.3, scipy 1.18.1, toppra 0.6.10, this x86-64 PC):** FILE_01.1 gives 3839 lines = 19.19 s at 5 ms from 62.1 s raw, matching slot 101 exactly (`alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:588`); FILE_08 16.80 s; FILE_10.1 4.26 s; FILE_6 20.25 s. A solve takes 0.11–1.82 s; importing the ALIX modules takes 0.35 s and needs no ROS. At the optimum acceleration binds almost everywhere; on FILE_10.1 joint 6 acceleration binds 38 % of the time (longest 2.88–3.05 s).
- **Uniform scaling is enough to hit a target:** scaling the TOPP-RA profile by k matches a re-solve with limits scaled by 1/k and 1/k² to within 0.0001° at k = 1.3 and 0.0006° at k = 2, so no second solve is needed. ALIX's slow ladder re-solves with velocity and acceleration scaled by the same factor, a different profile that is not needed here (`alix/tools/taught_pipeline.py:117-121`, `alix/tools/taught_pipeline.py:140`).
- **Gap between the optimisers:** with the fixes, the Default optimiser's fastest legal time is 27.42 s (FILE_08), 8.14 s (FILE_10.1) and 38.30 s (FILE_01.1) at 5 ms per line, 1.6–2.0× slower than TOPP-RA at a similar path change. Uniform scaling slows the whole region to suit its worst corner.
- **Byte readings:** TOPP-RA bytes read 1.007 × 600 °/s² as a 50 ms windowed mean, but 1.18–1.22× on ALIX's 20 Hz Butterworth reading, which is the filter overshooting. The optional 6 Hz post low-pass raises the real reading to 1.17–1.25× (`alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:184-191`). G4 reads 0.0 mm on ALIX's TOPP-RA slots because it compares the derived file with itself (`alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:320-324`, `alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:547-550`), so Lite must measure G4 against the raw recording.
- **Importing ALIX:** `local_speed.RATE_DEG_S` and `local_speed.ACC_DEG_S2` (`alix/experiments/exp01_teach_trajectory_reverse_engineering/src/local_speed.py`) are module globals that the emitter changes (`alix/experiments/exp01_teach_trajectory_reverse_engineering/src/emit_toppra_rung.py:304-305`), so they are not safe across threads. The emitter is left-arm only (`…/emit_toppra_rung.py:71`), but `TaughtPath` and `retime_toppra` take the side as a parameter. Bytes vary by up to 1 millidegree across library versions (`…/emit_toppra_rung.py:417-427`). `retime_toppra` samples with `arange` and drops the end remainder; `plan_min_time.resample_clock` rounds the point count up and lands exactly on the end (`alix/tools/plan_min_time.py:375-394`).
- **Jetson:** toppra 0.6.10 on PyPI has x86_64 wheels only and no source package; the newest source package is 0.6.3 (checked on PyPI, 13 Oct 2026). The aarch64 build must compile it from the git tag.
- **ALIX taught-track tools (other functions):** `strip_pauses`, `smooth_taught`, `capped_millideg`, `insert_dwells` and `rate_fit_track` are stateless command-line functions with no undo (`alix/tools/taught_tracks.py:767-832`, `alix/tools/cleaning_tracks.py:1675-1714`). `strip_pauses` judges stillness against the first line of a run, because judging it step by step cut a slow apex by up to 9 mm at the pad. `capped_millideg` does not default to the 5 ms clock; it is not needed by either optimiser.
- **ALIX motion docs:** On the upload channel, staying under the joint ratings is not enough. Rating-capped uniform 2.0× and 2.2× runs measured 20–26 mm and 44.5 mm p95 executed path error, and ALIX's model allows a bowl-outside scrub only 0.70× taught speed inside a 20 mm executed budget (`alix/docs/motion/TRACK_CHANNEL_TIMING_REQUIREMENT.md:210-214`). This is why G5 is always shown. It stays a warning (D11): ALIX's model reads 33–109 mm on the raw commode C recordings, which ran fine on the robot, so it cannot gate a file. A region's MAX DEV tightens the path-accuracy limit L5 instead.
- **ALIX Studio:** Undo is one global snapshot stack of the geometric editor's state: capped at 80 entries, unlabelled, not saved and cleared on load (`studio/web_editor/editor.js:1896-1977`). The Studio has no taught `.txt` editing at all; `studio/server.py` has no `.txt` or millidegree handling.
- **V6 package:** No per-region raw recordings exist. The V6 files are merged, time-stretched outputs (`lite/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/`), so they can only be imported as already processed.
- **Control app v53/v14:** New raw recordings come from the drag-teach save and are named by side and time only (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:3840-3850`). The runner's `read_track` needs 7 integer millidegrees per line and at least 2 lines, and refuses the whole file on one bad line (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:76-89`).

**Can it be borrowed from ALIX Studio?** **Partial** — ALIX Studio has no trajectory editing. Only its undo mechanics (snapshot before a change, undo/redo stacks, Ctrl+Z/Ctrl+Y with a typing guard, one entry per gesture) and its stale-on-edit pattern carry over, and both are re-implemented because they capture geometric-editor (X1) state (`studio/web_editor/editor.js:5371-5430`). The status line, error toast and confirmation modal can be used as-is.
- *Other sources:*
  - **Trajectory prototype** — Borrow + adapt: its algorithm becomes the Default optimiser, ported to the Lite server with the three minimal fixes and the limit checks. Keep the sub-region model, the original-to-processed index mapping and box-select (`lite/Alix_Studio_Lite_Trajectory_V2.html:1438-1566`).
  - **ALIX TOPP-RA chain** (`alix/tools/taught_tracks.py`, `alix/experiments/exp01_teach_trajectory_reverse_engineering/src/local_speed.py`, `…/emit_toppra_rung.py`) — Borrow as-is through one adapter, imported from the vendored ALIX copy: `smooth_taught`, `strip_pauses`, `TaughtPath`, `arclen_smooth`, `arrival_hold`. Borrow + adapt: a Lite-owned `retime_toppra(path, vlim, alim, vcap)` of about 40 lines with explicit arguments instead of module globals, held to parity with ALIX by a golden test; sampling in the `plan_min_time.resample_clock` form.
  - **ALIX checks** — Borrow as-is: G1 and G5 from the execution model and the exact `deviation_mm` (`alix/tools/taught_tracks.py:466-480`, `alix/tools/taught_tracks.py:607-716`), with forward kinematics from `alix_workspace`.
  - **alix_dispatch** — Borrow as-is: `parse_track` (strict reader) (`alix/core/python/alix_dispatch/src/alix_dispatch/track.py:66-82`).
  - **Control app v53/v14** — Not applicable to editing: it supplies the raw recordings and the output file contract.
- *Note on the inputs:* Partly accurate. The note points at the HTML, not at ALIX Studio, and the HTML is the right starting point: it has per-sub-region speed, box-select, a staged pipeline and a report, and its algorithm stays the default. It does not meet R2 as it is: trimming and smoothing are global, there is no undo, it has three measured defects (end-point jumps, boundary acceleration spikes, smoothing 10.6–20.9 mm off the path), and it checks no ratings, acceleration, limits or path accuracy.

**Recommendation.** Run both optimisers in a server-side recipe processor in `alix_studio_lite/lite/optimise/`, behind one interface that returns the bytes, the target, the achieved time, T_min, whether the target was met, the binding limits, every check result and the recipe. Keep the prototype's algorithm as the Default optimiser with only the three minimal fixes, and use ALIX's validated pieces (pause removal, zero-phase smoothing, arc-length smoothing, TOPP-RA, the G1 and G5 checks) as the Time-optimal optimiser and as the shared checks. Treat the operator's time as a target: meet it when the limits allow, otherwise deliver the closest legal time and say why. Keep Auto-optimise as the one-click path and sub-region editing as fine-tuning. Re-scope the Studio's undo into one labelled, saved history per region that stores recipes, not point arrays. The browser only previews; only the server's output can be simulated, run or uploaded.

**Approach**
1. **Recipe schema v1 (build).** One JSON per region: raw file SHA-256 and clock (0.005 s); optimiser (`default` or `toppra`); target time (catalogue value or override); region operations; ordered sub-regions with overrides (time % or pinned seconds for Default; speed cap % for TOPP-RA), holds and Keep stops; blend settings (0.25 s); limits (L1-L6 and the MAX DEV, if any); execution-model version; history. Boundaries are stored in raw-recording lines, so they stay valid when the output length changes.
2. **ALIX adapter (borrow as-is).** One module sets the import paths once and imports the ALIX functions from the vendored copy. The bundle must carry what `taught_tracks.py` pulls in (`cleaning_tracks`, `alix_dispatch`, the exp01 `teachlab` modules) and scipy.
3. **Default optimiser (borrow + adapt from the prototype):** `trim_idle_proto` (window 11, 8 millidegrees, run 20) → `smooth_moving_mean` (window 41, odd-reflection padding) → G4 check against the raw recording → blended time warp (per-line time step blended over 0.25 s, points resampled on the chords). T_min by bisection on log k over the integer bytes (about 25 iterations, under 0.5 s on 12,000 lines). A pinned sub-region that cannot meet its time is raised to its own minimum, and the report names it.
4. **TOPP-RA optimiser (borrow + adapt from ALIX):** 5 Hz smoothing → pause removal → arc-length smoothing (30 mm; 15 mm when the region has a MAX DEV) → G4 check → solve with 0.95 × ratings, 600 °/s², sub-region speed caps as a varying velocity constraint and `arrival_hold` caps if G1 needs them → sample in the `resample_clock` form → byte check. If the bytes fail by a small margin, slow by the measured factor and re-check (at most 3 times). The 6 Hz post low-pass is off by default. The solver runs in a worker process with Cancel.
5. **Target fit (build, shared):** convert the target to a line count at 5 ms per line; scale uniformly if feasible, else deliver T_min. The binding report names the top 1-3 constraints by share of time for TOPP-RA, and the check that fails at 0.995 × k_min for Default, each with joint, quantity and time window.
6. **Checks after every recompute** (shared module, see R12): L1-L6 hard, G5, jerk, pad speed and raw-tick warnings, all on the written integers.
7. **TOGETHER blocks (build):** align both arms to a common N and mark the dual-arm clearance result out of date until it is re-run on the retimed pair (R10, R5).
8. **Per-region history (build, Studio pattern):** entries `{seq, label, recipe, recipe_sha, at}` saved with the region. Undo and redo move a pointer, a new change clears redo, labels come from the recipe diff, and outputs are recomputed on restore.
9. **Server API (build):** `GET/POST /api/units/{id}/recipe`, `…/undo`, `…/redo`, `…/versions`, `…/optimise` (returns the result with T_min and the binding report), `…/optimisers` (which are available on this computer).
10. **UI (build on the prototype's interaction):** Auto-optimise button; optimiser choice; target time; a timeline in seconds with labelled sub-region bands (box-select, "Split here", drag edges); region and sub-region panels with "Same as region / Custom" and "Reset to region"; Undo/Redo with labelled tooltips. Save version writes `<ID>__OPT_v<k>.txt` plus a sidecar JSON.
11. **Legacy files (V6, lid_top):** import as `legacy_processed`, with smoothing off and "fit to limits" (Default optimiser, uniform scaling only) as the offer, and L1-L6 and G5 findings shown.
12. **Platforms:** the operator PC (x86-64 Linux) now and the Jetson AGX Orin (aarch64, Ubuntu 24.04) in production. The aarch64 bundle builds toppra 0.6.10 from its git tag (NFR-OFFLINE-DEPLOY). If `import toppra` fails, Lite starts with TOPP-RA unavailable and the Default optimiser working.
13. **Tests (golden, run on both architectures):** TOPP-RA parity (FILE_01.1 gives 3839 lines, bytes within 1 millidegree of ALIX); TOPP-RA T_min on FILE_08, FILE_10.1 and FILE_6; uniform scaling at k = 1.3 and 2.0; an infeasible target on FILE_10.1 reporting joint 6 acceleration; Default end snap, boundaries and G4; Default T_min; a monotonicity property test; TOGETHER with equal N; Lite without toppra; writer format; identity recipe; locality; undo of 5 or more changes end to end (Playwright).

**Acceptance criteria**
- [ ] Default optimiser with every operation off and the target equal to the recorded time: the saved output points are identical to the raw file.
- [ ] **Target met:** when the target is at or above T_min, the achieved time (lines × 5 ms) is within one line of the target, for both optimisers. Example: FILE_01.1, Default, target 60 s.
- [ ] **Target not met:** with a target below T_min, Lite delivers T_min and names the binding limit in plain language. FILE_10.1 with TOPP-RA and a 3.0 s target reports joint 6 acceleration, about 38 % of the time, window 2.88–3.05 s.
- [ ] **TOPP-RA parity:** FILE_01.1 gives 3839 lines (19.19 s at 5 ms), bytes within 1 millidegree of ALIX's chain; FILE_08, FILE_10.1 and FILE_6 give 16.80 s, 4.26 s and 20.25 s, within one line.
- [ ] **Default fixes:** first and last steps ≤ 100 millidegrees on FILE_08 with the 101 window (was 966); on FILE_01.1 with sub-regions [75, 80, 95, 115, 75] %, the windowed acceleration at every boundary is ≤ 300 °/s² (was 640–919); at window 41 the pad moves ≤ 4.6 mm on FILE_08, FILE_10.1 and FILE_01.1, and window 101 fails G4 with a "reduce smoothing" hint. Default T_min is 27.42, 8.14 and 38.30 s on those files, within 1 %.
- [ ] **Monotone feasibility:** every hard limit passes for every scale at or above k_min (property test on 20 files).
- [ ] **Locality:** after a change to one sub-region, the path outside that sub-region plus its blend zone (≤ 0.25 s) is identical. In the Default optimiser the speed profile there differs only by the one region-wide factor that keeps the target.
- [ ] Five consecutive changes, including an optimiser switch and a target change, can each be undone and redone, restoring the previous output point for point. Each entry names the region or sub-region, the operation and the old → new value. The history survives a restart, and undo in one region never changes another.
- [ ] Every version saved as sendable passes L1-L6 on its integer bytes at 5.0 ms, with L5 at the region's MAX DEV where one is set. It has 7 integer millidegrees per line and at least 2 lines, and runner v14 `read_track` and `alix_dispatch.parse_track` accept it unchanged. The raw file's SHA-256 never changes.
- [ ] In a TOGETHER block both arms are saved with the same line count, the larger of the target and both T_min values, and the binding arm is named.
- [ ] The sidecar records the optimiser, every parameter, the numpy, scipy and toppra versions, the raw file's SHA-256 and every check value. Re-running the recipe gives the same bytes, within 1 millidegree across platforms.
- [ ] With `toppra` not importable, Lite starts, TOPP-RA is shown as unavailable with the reason, and the Default optimiser works. The golden tests pass on the operator PC (x86-64) and on the Jetson (aarch64, toppra built from source).
- [ ] On the operator PC, one change to a 12,000-line region recomputes and re-plots in under 1 s with the Default optimiser and in under 3 s with TOPP-RA. The Jetson time is measured and recorded.

**For operators:**
- One button, **Auto-optimise (recommended)**, does the safe default with the region's target time; sub-region fine-tuning is optional.
- The target time comes pre-filled from the catalogue. When it cannot be met, an amber note says so in plain words, for example "The robot cannot clean this region in 18 s without exceeding its joint limits. The fastest safe time is 19.2 s.", with a **Use 19.2 s** button.
- Plain names ("Remove stops", "Smoothness", "Speed", "Keep stops here"), seconds and degrees, never rows. The effect shows at once: "Takes 19.2 s (was 62.1 s)".
- Undo/Redo stay visible, with a "Last 5 changes" list.
- The optimiser choice and any MAX DEV are set per region by an engineer; the operator sees which is in use.

**Safety:**
- Any smoothing change moves the path, so simulation and collision results become out of date (R11, R5).
- Passing the joint limits does not make a fast file accurate. On the upload channel, rating-capped uniform 2.0–2.2× runs left the path by 20–45 mm p95, and TOPP-RA's hardware record so far is on the CANFD channel. G5 is itself a p95 prediction with a worst case about twice it (`alix/docs/motion/TRACK_CHANNEL_TIMING_REQUIREMENT.md:93-99`). Simulation and a slow first real run stay mandatory, and the first upload-channel runs of TOPP-RA files are measured against G5.
- ALIX's TOPP-RA chain has run on hardware for left-arm recordings only (`alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:556-557`); right-arm files get a supervised first run.
- Some stops are deliberate (press, soak), so every sub-region offers "Keep stops" or "Hold N s".
- In a dual-arm step any retiming shifts one arm against its partner, and ALIX ties a clearance verdict to both tracks' SHA-256 (`alix/tools/dual_arm_verdicts.py:18-22`), so the dual-arm clearance must be re-checked (R10, R5).

| Priority | Size | Depends on |
|---|---|---|
| Must | L | R1, INT-TRAJ, R12, R11, R3, R5, R7, R10, NFR-THEME, NFR-OFFLINE-DEPLOY (aarch64 toppra build) |

**Open questions:**
- Are sub-regions free time windows, or the deck's PART 1/2?
- Should trimming also crop the start and end of a recording?
- Is "five" a minimum, and must the history survive a restart? Recommended: at least 5, saved.
- Which regions keep their stops?
- Which regions carry a MAX DEV (5, 10 or 20 mm)? It tightens the path-accuracy limit L5 for them; G5 stays a warning (D11).
- Which execution-model file does Lite pin for G1 and G5: `Resource/tracks/taught/EXECUTION_MODEL.json` from the vendored copy (`vendor/alix/Resource/tracks/taught/EXECUTION_MODEL.json`), or the defaults at `alix/tools/taught_tracks.py:196-223`?
- Is a 41-line moving-average window acceptable as the Default optimiser's new default (the prototype used 101)?
- How long does one TOPP-RA solve take on the Jetson AGX Orin? To be measured early, together with the toppra source build.

---

### R3 — Simulation playback

**Original (inputs):**
> Simulation Playback: The edited trajectory should be played back in a simulation window to identify collisions, missed spots, or other issues. If any issues are detected, the user should be able to return to trajectory editing. -> Already exixting in ALix Studio - borrow

**Revised requirement.** Lite shall replay any edited region trajectory in a 3D simulation window, on the ALIX robot model and with both arms of the task step, before the trajectory can be sent to the real arm. Lite shall show collisions, missed spots and other issues on one row-indexed timeline, and shall take the operator back to the editor at the problem row.

- (a) **Inputs.** The edited per-arm `.txt` (7 integer millidegrees per line), the region's pole height (mm) and hand counts (6 × 0-1000), the partner arm's trajectory for the same task step, the toilet state (closed, lid open, lid and seat open), and the scene: the ALIX URDF plus the `commode_c` scene (toilet pose and articulation, bin, UGV, poles) and the selected venue file for the walls. An untaught partner is held at a declared park pose (default: its last pose in the workflow), and the result is labelled "partial". Lite refuses to simulate when the pole or the hand is unknown.
- (b) **Playback.** Play/pause, step one row, scrub, and speed 0.25×-4×. 1× plays one row per 5 ms, the controller's cycle. The 3D pose, every joint-plot cursor and the timeline always show the same row.
- (c) **Issues shown.** Collisions and close calls (R5), joint-limit and rate warnings (R12), and a **wiped-area view**: toilet surface that the contact links of the region's hand preset touched at least once is painted, untouched surface is not, and the contact path is drawn. Contact is never judged from a single pad frame.
- (d) **Return to editing.** From the playhead or any issue, one action opens the R2 editor on the same region, arm and row window. Any edit marks the result "out of date".
- (e) **Verdict.** Each region shows *Not simulated*, *No issues found* or *n issues*, and names what was not checked, for example "partner arm not taught", "walls not checked: no venue selected", "scene file missing" or "bin not checked". A missing scene or venue file gives NOT CHECKED for that part, never a pass.
- (f) **Flow playback.** Start lags, approach moves, pole moves, hand changes, waits and operator checkpoints appear with durations from laws measured on the robot.
- (g) **Lid and seat operations.** In these units the lid or seat moves with the hand, so the toilet state changes during the playback. The 3D check holds the starting state for everything except the part being operated, treats contact between the gripping hand and that part as intended, and checks every other contact as usual. Animating the moving lid or seat from the taught path is a later improvement.

**Findings**
- **ALIX Studio:** It has three 3D players (Dual-Arm Collisions tab, Schedule & Run, plan-file player). They accept only MoveIt-style `.json` plans under the repo's plan folders (`studio/server.py:4185-4196`). None has a speed control: two play at 2×, and the Dual-Arm Collisions player steps every max(20 ms, dt) (`studio/web_editor/editor.js:12504-12506, 14561-14581`).
- **alix_workspace:** Every frame starts from bin LOWERED, poles at 0.300 m and open hands, and hand states are accepted only as SRDF names (`alix/core/python/alix_workspace/src/alix_workspace/dual_arm_plan_collision.py:434-450`). Taught files therefore need an adapter that seeds pole, hand and bin explicitly, with hand counts passed in as joint values.
- **ALIX Studio:** There is no FK in the browser. `applyRobotPose` takes 4×4 transforms per link computed on the server (`studio/web_editor/editor.js:12423-12437`). The geometry endpoint builds its context through `_ik_context`, which imports the Workspace reachability module (X2) (`studio/robot_model.py:169-228`). Lite needs a thin loader, and browser FK for row-accurate scrubbing.
- **ALIX Studio / alix_workspace:** Toilet-face marking already works in joint space. `fixture_contact_faces` returns the touched faces and `paintCollisionFaces` paints them (`alix/core/python/alix_workspace/src/alix_workspace/collision.py:430-487`; `studio/web_editor/editor.js:12008-12032`). This is the base for a wiped-area view. The Studio's coverage score works on generator configs, not on taught tracks.
- **Measured on V6 data:** On `R2_LIDTOP` with the GRIP hand `[0,0,0,0,150,1000]`, the flexed finger links touch the closed lid on 36 of 188 sampled rows, while the `R_glove_frame_2` origin stays 19-35 mm away (only 1 row is within 20 mm). A "pad frame within 20 mm" rule would call this wipe "in the air". ALIX's contact list for `glove_frame_2` was derived for the `open_tenth` hand, not GRIP (`alix/ros/orchestration/alix_tasks/config/contact_links.yaml:81-84`).
- **Trajectory prototype:** It assumes 5 ms per row, which is Lite's time base. It numbers rows from 1, and its plot selection creates only scale regions (`lite/Alix_Studio_Lite_Trajectory_V2.html:1176-1187, 1392-1396`).
- **Control app v14 / ALIX run records:** The controller moves each arm to a program's first point by itself, and nothing checks that move; the runner only prints the gap (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:205-214`). V6 contains such moves of up to 64.6°. In 283 ALIX hardware run records at 50 % run speed, these moves took 5-10× as long as the emulator's law predicts (`alix/docs/EMULATOR.md:73`), so flow timing must come from measured laws.
- **Scene sources (decision of 13 Oct 2026):** The deck's commode C is ALIX's `commode_c`, the only toilet model ALIX treats as validated (`studio/body_collision.py:58`). Its pose `[1.005, 0, 0.42]` relative to `base_link`, the lid and seat hinge (open −110°) and the UGV standoff are in `alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml:32-68`; the bin, UGV and poles are in the URDF. No measured cell profile is needed. Walls are the one part not enforced today. ALIX's venue files in `alix/ros/orchestration/alix_tasks/config/scene/` hold the wall meshes, but only the 800 mm and 1000 mm ones are single cubicles; the 1400 mm venue is the whole washroom (4.3 × 8.1 m). Lite therefore uses a single 1400 mm cubicle by default, built like ALIX's 800 mm one, with the 800 mm cubicle as the alternative (decision D21), loaded by `load_wall_manager` (`alix/core/python/alix_workspace/src/alix_workspace/plan_scene_validation.py:117-155`).
- **Measured on V6 data:** With ALIX's default 800 mm venue, `R2_LIDTOP` (pole 150 mm, GRIP) touches a wall on 28 of 188 sampled rows; the 1000 mm and 1400 mm venues give 0. The wrong cubicle therefore gives false wall hits, which supports the 1400 mm default; the cubicle is verified once against known-good recordings. The fixture's washroom rotation comes from `washroom_scene.yaml` while the offline walls default to the 800 mm venue; both are yaw 180 today, so they agree, but Lite should take both from the one selected cubicle (`studio/scene_pose.py:93-120`; `studio/body_collision.py:135-136`).

**Can it be borrowed from ALIX Studio?** **Partial** — The viewport shell, the robot-geometry payload and baked overlay (re-baked, R10), `meshFromPayload`, `applyRobotPose`, the toilet and wall renderers, the transport controls, the live joint-feed endpoints and `paintCollisionFaces` can be lifted. The players cannot be used as they are: they take only plan JSON, at fixed speeds, with ALIX's default pole, hand and bin. The Studio also has no missed-spot view for taught tracks and no route back to a joint editor.
- *Other sources:*
  - **alix_workspace** (`UrdfModel`, `RobotCollisionModel`, `fixture_contact_faces`) — Borrow + adapt: feed in-memory joint maps with explicit pole, hand and bin; never rely on the library defaults.
  - **ALIX tools** (`alix/tools/dual_arm_verdicts.py:501-731`) — Borrow + adapt: copy the ~100-line `.txt`-to-timeline shim (`track_points`, `hand_rad`, `build_timeline`, `hold_timeline`, arm-side guard), decouple it from ALIX task manifests, add the 7-integer check and keep it in memory.
  - **`hand_units.py`** — Borrow as-is: 0-1000 counts to radians, with the mimic chain (`alix/ros/hw/alix_hw/scripts/hand_units.py:30-60`). Do not use `sim_bridge`'s travel table, which exceeds the URDF limits.
  - **alix_dispatch** — Reference: the emulator, for developer dry runs only.
  - **ALIX run records and start-lag law** — Borrow as data (seed only): approach-move and start-lag durations, re-fitted on Lite's own runs.
  - **Trajectory prototype** — Borrow + adapt: plot cursor shapes, animation-frame playback and plot selection; it needs a generic row selection.
  - **Control app v53/v14** — Reference: workflow meaning of pole, hand, program and wait steps, and the pole timing law (`lite/upload_channel/pole.py:3-14`).
- *Note on the inputs:* Partly accurate. ALIX Studio has 3D joint-motion playback and a live-feed view. But it plays only MoveIt-style plan JSON from its own plan folders, at fixed speeds, from a lowered bin, 0.300 m poles and open hands, with the toilet state taken from ALIX task configs. It has no missed-spot analysis for taught joint tracks and no route back to a joint editor, because it edits Cartesian configs.

**Recommendation.** Build simulation as a step of every region in Lite's own backend and UI, not as a Studio tab. Lift the Studio's viewer, mesh payload, scene renderers, transport and face painting, and feed them from an in-memory `.txt` adapter that always seeds pole, hand and bin. Do FK in the browser with a parity test against the Python FK, keep collision checking on the server (R5), and compute "missed spots" from the preset's contact links. Take the scene from the URDF and the `commode_c` scene, and enforce the walls from the selected venue file. Until the venue has been verified with known-good recordings, wall results carry "venue not verified"; a missing scene or venue file gives NOT CHECKED.

**Approach**
1. **Asset bundle (shared with the architecture work).** Package the URDF, the re-baked visual overlay (R10), collision meshes, SRDF, initial positions, the toilet model and scene, and the venue walls. Load them with an explicit `PackageIndex`. Set `ALIX_REPO_ROOT` or copy `load_wall_manager`, because `plan_scene_validation` raises at import without a repo root (`alix/core/python/alix_workspace/src/alix_workspace/plan_scene_validation.py:31-59`).
2. **Scene and cubicle (setup step, not a calibration project).** Load the toilet pose, articulation and parts from the `commode_c` scene, and the bin, UGV and poles from the URDF. Apply the scene pose as written, including any non-X component, because the Studio's `fixture_manager` shifts the toilet only along X. The walls are a single cubicle: 1400 mm wide by default, 800 mm selectable in Settings (decision D21). ALIX ships single-cubicle wall meshes for 800 mm and 1000 mm (`washroom_800mm_planning_scene.stl`, `washroom_1000mm_planning_scene.stl`: back wall 1.215 m in front of the robot base, 0.1 m walls, 1.98 m deep, 2 m high), but its 1400 mm venue uses the whole-washroom mesh (`washroom_1400mm_wide.stl`, 4.3 × 8.1 m). Lite therefore adds a 1400 mm single-cubicle mesh built the same way, with inner side faces at ±0.70 m. Lite records the cubicle used with every check. Verify the cubicle once by replaying 3-5 known-good taught recordings (for example the V6 70-series files): they must show no wall hits. Missing files give NOT CHECKED.
3. **Track adapter and timeline endpoint (adapt the ALIX shim).** Parse the `.txt`, refuse any line that is not 7 integers, convert to radians, and add the pole joint and the six hand joints from `hand_units.py`. Return per-arm arrays, pole, hand and bin per event, event durations with the law each used, and the row-to-seconds map. Unit-test on V6 files.
4. **Browser FK (build).** Port `UrdfModel.link_world_transforms` (`alix/core/python/alix_workspace/src/alix_workspace/urdf_kinematics.py:294-344`) to JavaScript, including mimic joints and "unlisted joints = 0". The server sends the kinematic tree and the home seed once. The parity test runs in CI.
5. **Viewer (borrow + adapt).** Lift the viewport shell, geometry payload, `meshFromPayload`, `applyRobotPose` and the toilet and wall renderers. Serve geometry through a thin loader instead of `_ik_context`, and the scene through a thin Lite manifest instead of `build_scene_manifest`, which is tied to ALIX's region and generator data.
6. **Player (build on the Studio transport markup).** One shared `currentRow` and an animation loop that advances by speed × elapsed time / clock. Add speed, step, scrub and keyboard control. The R2 plots follow through a cursor shape, as in the prototype (`lite/Alix_Studio_Lite_Trajectory_V2.html:1134-1141`).
7. **Wiped-area view (borrow face painting, build the job).** A server job tests the preset's contact links against each toilet part on every row (or every 2nd row), records touched faces and the closest contact point, and caches the result by content hash. The client paints touched faces and draws the contact path.
8. **Verdict and editor link (build).** Add the verdict chip with its "not checked" reasons, and "Edit here" / "Back to editor". A content hash marks results out of date, following the Studio's `markDirty` → `invalidateValidation` pattern (`studio/web_editor/editor.js:5371-5389`).
9. **Whole-flow playback (build).** Drive it from the workflow JSON: replay at (lines − 1) × 5 ms, and measured laws for start lag, approach move, settle, pole and hand. Seed the laws from ALIX records and re-fit them on Lite's first real runs (R4).
10. **Later.** Add a coverage % per region with the Studio's `compute_coverage` (`studio/coverage.py:100-140`), once regions have surfaces.

**Acceptance criteria**
- [ ] `lite/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/RIGHT/R2_LIDTOP.txt` (3,751 rows), with pole 150 mm and hand `[0,0,0,0,150,1000]`, plays with no plan JSON on disk. A file with a non-integer or 6-value line is refused, and the message names the line.
- [ ] Over 200 random V6 rows, including pole and hand joints, browser link transforms equal `UrdfModel.link_world_transforms` within 1e-6 m and 1e-6 rad. The arm base is drawn at z = 0.6485 m + pole (0.7985 m at 150 mm). GRIP gives fingers 1.344 rad, `thumb_2` 0.445 rad and `thumb_1` 0 rad.
- [ ] At 1× a 3,751-row file plays in 18.75 s ((3,751 − 1) × 5 ms) ± 1 %, and at 2× in half that. A step moves exactly one row. The plot cursor row always equals the 3D row, with one row-numbering convention on screen.
- [ ] A task step with both arms taught plays both arms on one clock from their modelled start offsets. With one arm untaught, the partner holds its park pose and the verdict reads "partial: partner arm not taught". In a multi-program flow, start lags, approach moves, pole moves and hand changes show durations labelled "modelled", with the source of each law.
- [ ] The toilet appears in the step's state (for `commode_c`, open is −110° about the hinge). The toilet pose comes from the `commode_c` scene and the walls from the selected venue file; the verdict names the venue. With no venue selected or the file missing, walls show NOT CHECKED. With the verified venue, the V6 known-good recordings give no wall hits.
- [ ] **Wiped-area regression.** With `commode_c` at ALIX's pose and `R2_LIDTOP` with GRIP, lid-top faces are painted only from rows where finger links intersect the lid (measured: 36 of 188 sampled rows). With the toilet moved 100 mm away, nothing is painted. No glove-frame-distance rule is used.
- [ ] "Edit here" at row r opens the same file in the editor with rows [r − 0.5 s, r + 0.5 s] selected. After any edit the region shows "Simulation out of date".
- [ ] A 10,983-row file plays at 30 fps or more on the dev box with the wiped-area layer on. The first frame appears within 5 s of opening a region once robot geometry is cached.

**For operators:**
- One large **Check in simulation** button per region. The result is a sentence, for example "1 problem: right arm touches the toilet lid at 0:12".
- Space plays and pauses; ←/→ step one row. Speed reads *Slow / Normal / Fast*. Deck region names are used, never program IDs.
- **Back to editor** sits next to every issue and on the playhead, and returns to exactly the same place.
- The wiped-area legend is in words ("Painted = the hand touched this surface; unpainted = not wiped, check it"), with a neutral paint and a hatch rather than red/green.

**Safety:**
- A clean simulation means "nothing found in the model", not "safe". Every verdict lists what was not modelled.
- Never accept ALIX's defaults (0.300 m poles, open hands, LOWERED bin). Refuse to simulate when the pole or the hand is unknown.
- The run speed % scales only the approach move (`alix/core/python/alix_dispatch/src/alix_dispatch/track.py:20-24`), so a reduced-speed first run means uploading a slowed (resampled) copy of the file.
- Two-arm playback is nominal: upload-channel arms can drift apart by seconds, and the controller does not check approach moves. R5 checks both.

| Priority | Size | Depends on |
|---|---|---|
| Must | L | R1, R2, R5, R7, R10 (b, c); R8 and R4 for flow timing; venue selection and verification; the requester's contact-link ruling |

**Open questions:**
- "Missed spots": surface never touched, or too little pressure? Pressure cannot be seen in joint data (`lite/Alix_Studio_Lite_Trajectory_V2.html:437`).
- Which hand links may touch the toilet with GRIP? The measurement shows the flexed finger links on the lid.
- Does the test cell match the `commode_c` scene pose and UGV standoff, or does it need its own venue/scene file?
- Is region-level playback enough for v1, or is whole-flow playback needed?
- Mechanism tasks: should the lid and seat be animated, or switched at the end of the step?
- Where should an untaught partner arm park?

---

### R4 — Real-arm testing

**Original (inputs):**
> Real-Arm Testing: The validated simulation trajectory should be executed on the actual robotic arms. If collisions, missed spots, or other issues occur, the user should be able to return to trajectory editing. - should be able to connect to the real robot, have the IP address entry options, also toggle between real and sim

**Revised requirement.** Lite shall let an operator run on the real arms a trajectory whose exact content has passed simulation. The operator shall be able to stop it at any moment, bring the arm back to a safe pose, record what happened and return to editing with the run's context. The hardware E-stop buttons, wired to an IO input on each arm controller, are the primary safety stop; Lite shall show their state per arm, block motion while either is active and guide recovery.

- (a) **Connection.** Lite runs on the operator PC (x86-64 Linux) now and on the robot's Jetson (AGX Orin, aarch64, Ubuntu 24.04) in production; connection, run and stop work the same on both. Each arm has an IP address and port, prefilled LEFT 192.168.1.10, RIGHT 192.168.1.103, port 8080. Only an engineer can change them, and they persist. **Test connection** reports Connected, Unreachable or Error within 5 s, with the controller's run mode as read back. Every robot command uses the configured IP and port, never environment defaults.
- (b) **Run target.** A clearly labelled selector offers **Simulation** (Lite 3D, no robot), **Robot dry run** (controller SIM mode: the arms stay still and pole and hand steps are always skipped) and **Real robot**. Lite always opens in Simulation. The controller mode is read back after every change and again inside the run process just before start; a mismatch refuses the run. After a dry run Lite restores the mode it found and reads it back.
- (c) **Gate.** A real run starts only when all of these hold: the exact content has a passed simulation record (a failed record cannot be overridden; a missing one only by an engineer with a written reason); it passes the R12 motion checks; the slot holds that content, checked inside the run process against Lite's registry and the stored size and name (R13); the arm is idle with no blocking error; both arms' E-stop chips read CLEAR (h); the transit check (d) passes; Lite has applied and read back the region's pole/hand preset (R7); the operator has ticked the human-only checklist (fixture state, area clear, pad off the surface, E-stop within reach). A robot dry run comes before the first real run of new content unless an engineer has switched this off.
- (d) **Transit check.** With the controller in REAL mode, Lite reads the arm's joints and sweeps the straight joint-space line to the first point in its collision world (the commode_c fixture in the region's state, the bin, the poles, the venue walls from the selected venue scene file, the other arm; R3). It shows the move and its size, runs it at reduced speed (default 20 %, an engineer setting) and refuses the run on a collision or when the move exceeds an engineer-set size limit.
- (e) **Run, stop and monitor.** A large, always-visible STOP per arm and a STOP ALL, labelled as software stops ("STOP (software)"); STOP ALL slow-stops the arm, stops the pole, holds the hand and cancels the sequence in one click, and the stop is sent once. The software STOP is a convenience, not the safety stop; whether it ends a running stored program is tested on the robot (G4) before operators use REAL mode, and Lite uses whichever stop verb that test proves. Live status shows the phase, elapsed against expected time, translated controller codes and a live 3D mirror of the arms. The global speed is restored after every run and the result is recorded. The arm is slow-stopped if the operator screen is lost for more than a set time (default 3 s) or the Lite backend dies. In dual-arm runs both arms start within a measured skew (target 0.2 s or less), and a fault or STOP on one arm stops the other within 1 s.
- (f) **Completion.** Lite shows **Finished** only when no blocking code was reported, the final pose read after settling is within 2° of the last point, playback lasted at least 85 % of the expected time, and (once commissioning has confirmed it on these controllers) the controller's program-finish event was received. Otherwise it shows "Stopped at about X %" or "Not confirmed: please check".
- (g) **Recovery, verdict and return.** After any run that did not finish (including one interrupted by the E-stop, after the recovery in (h)), before anything else moves, Lite reads both arms' joints and offers a checked way back to a safe pose, one arm at a time: a joint move to the safe pose if it is collision-free in simulation, otherwise reversing out along the executed path, otherwise hand guiding. "Run again" stays disabled until the arm is at a safe pose. After each real run the operator records Looks good / Hit something / Missed a spot / Other, with a note; Lite stores a run record; **Back to editing** opens the trajectory at the stop point or a point the operator marks. Any edit marks the simulation and real-test status as stale.
- (h) **IO E-stop (primary safety stop).** Lite shows one E-stop chip per arm: CLEAR, E-STOP ENGAGED, RECOVERING, UNKNOWN or NOT CONFIGURED. It reads the chip from **IO3**, the E-stop input on both controllers (decided 13 Oct), which must be in mode 5 ("input emergency stop"), cross-checked with the joint enable flags (and Modbus register 302 where enabled), never from `arm_current_status` alone, at 5–10 Hz through the one connection owner per arm. UNKNOWN and NOT CONFIGURED block motion exactly like ENGAGED. While either arm is not CLEAR, Lite refuses Run, Sequence, Test on robot, Move to start, Pole, Hand, Drag-teach start and the switch to REAL on both arms; uploads stay allowed. When an E-stop is seen during a run, Lite marks the run "Interrupted: E-stop" (never Finished), cancels the rest of any sequence, sends pole stop and hand freeze (the IO E-stop does not stop the hand) and logs the controller state with timestamps. After release, a guided recovery takes the operator through: confirm the cell is safe; release the button; read the state back in plain words; clear errors (at most two rounds, read back each time); re-enable joints with the wrist supported, or pendant or power-cycle instructions; restore the global speed; collision-check the move from the current pose to the program's first point (walls included) and show its size; re-home the pole. An interrupted run is never resumed automatically. Lite never writes IO modes; it blocks motion at start-up when IO3 is not in mode 5 on either controller (RealMan's demo maps IO3 to "continue program" (mode 4) and IO4 to the E-stop (`alix/external/butterfli-realman-alix/Demo/RMDemo_Python/RMDemo_IOControl/readme.md:68`), so a controller left in that mapping would resume a program instead of stopping it). The E-stop state holds from the press until the button is released, which is the manual reset; Lite's chip follows the same rule and then shows RECOVERING until the recovery steps are done.

**Findings**
- **ALIX Studio:** No execution channel. A test pins that "no run/stop/pause route exists" and that the run panel says "arms not connected" (`alix/core/python/alix_taskgen/test/test_studio_interaction_guards.py:2204-2216`). Its Simulate view does accept an external joint feed, which can be the live mirror of the real arms: post both arms in one map and request FK per arm with `source: "external"`, because `auto` prefers the emulator (`studio/server.py:5312-5393`).
- **Control app v53/v14:** The runner connects by `--ip/--port` (`--ip` wins over `--side`), but v53 never passes them and shows the IPs as fixed labels (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:2283-2286`; `lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:7506`). Each action is one runner process with one TCP connection, with no reconnect, retry or heartbeat.
- **Control app v53/v14 and alix_workspace:** v53's SIM/REAL/CHECK switches the controller's run mode (0/1) with read-back; it is a dry run on the controller, not a 3D simulation (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:1213-1262`). In SIM the controller plays the arm virtually, does not move the lift or hand and has no commode model; it did catch a 4109 collision error caused by a controller configuration defect (a wrong payload centroid) that also affected REAL (`alix/docs/PHASE_PLAN.md:115-116, 139, 144`).
- **alix_workspace (completion):** The runner's `RUN COMPLETE` is not proof. ALIX recorded a silent cut that raised only warning 4105 and stopped at point 2,319 of 2,971; the runner treats 4105 as a warning and would report it complete (`alix/plan/FINDINGS.md:9563-9572`; `lite/upload_channel/rm_upload_fast_v14_safe_transit.py:263-266`). Pose alone also misleads: V6 `M3_OPNST02.txt` comes within 1.07° of its last point before its last 10 % (derived).
- **alix_dispatch:** It holds the witnesses and rules Lite needs. The controller's `program_run_finish` event separated 90 complete runs from 4 cuts with no overlap, and played time against the expected time (bar 0.85) is a second witness (`alix/core/python/alix_dispatch/src/alix_dispatch/track.py:2383-2390, 1573-1588`). Mode is gated on read-back (an unreadable mode is refused), pole and hand are skipped in SIM, and a SIM pass licenses REAL only for identical content; a failed SIM blocks REAL with no override (`alix/core/python/alix_dispatch/src/alix_dispatch/ladder.py:101-145`).
- **alix_workspace (transit and speed):** The controller does not collision-check the move to the first point, but the move is predictable: `movej` was measured joint-linear to 0.004° (`alix/docs/PHASE_PLAN.md:186`), so Lite can sweep the straight joint-space line (inferred for the program's own transit; confirmed at commissioning). The run speed scales only this transit and becomes a controller-wide ratio that survives power cycles, so it must be restored after every run (`alix/plan/FINDINGS.md:5854-5881`).
- **Control app v53/v14 and alix_dispatch (STOP):** STOP is SIGINT to the runner, which then slow-stops the arm; on Windows the runner is probably killed with no slow stop (inferred; Windows is not a target, Lite runs on Linux). A second interrupt can skip the speed restore, a "did not start within 5 s" exit sends no stop (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:238-240`), and a program keeps playing if its client dies. Whether slow stop ends a running stored program has never been tested on hardware; `alix_dispatch` already has the probe for it, `probe_stop_policy` (its "gate G4", not the taught-track G4) (`alix/core/python/alix_dispatch/src/alix_dispatch/track.py:3273-3300`).
- **RealMan manuals (IO E-stop).** Each Gen-3 controller has four multiplexable inputs, IO1–IO4 on pins 5–8 of the 16-core IO cable, and any of them can be set to function 5, "Input emergency stop function multiplexing mode", on the pendant or through the API (HW p.39–40; TP p.112, p.131; JSON `set_IO_mode` p.118; `rm_set_io_mode`, SDK `rm_robot_interface.py:2882`). The inputs trigger on a HIGH level (DEMO p.82). The manuals do not say what the IO E-stop stops or give its stop category, safety rating or reaction time; RealMan's own guide says safety devices belong on safety interfaces, not normal IO (HW p.32). The closest documented behaviours are the pendant E-stop ("stops at maximum speed", TP p.11) and `set_arm_stop` (maximum deceleration, power kept, trajectory cannot be resumed; JSON p.61, PY p.84); API2 code -6 groups the IO, pendant and JSON E-stops as one external stop (CPP p.318). So the IO E-stop is very likely the controller's quick stop with power kept, which matches the field report (inferred; measured in the drill). The Chinese manual V1.2.0 also lists pin 14 = E_STOP (CN p.24), which the English Gen-3 page lists as reserved (HW p.39). *(Manual short names: `alix/docs/Manuals/Realman/` 02 = HW, 03 = TP, 04 = JSON, 05 = CPP, 06 = PY, 08 = DEMO, 09 = MB, 01 = REL; CN = Chinese user manual V1.2.0.)*
- **RealMan SDK and alix_workspace (E-stop detection).** The direct signal is `rm_get_io_state(n)` on the IO in mode 5 (SDK `rm_robot_interface.py:2927`; JSON p.120); `rm_get_io_input` returns -1 for a port not in input mode, so it may not show a mode-5 port (PY p.160; SDK :2961). Modbus input register 302 "Emergency Stop Status" clears on release (MB p.8; REL p.14, firmware V1.6.7 and later; Modbus-TCP must be enabled on the pendant, TP p.125). There is no E-stop entry in the system-error table or the joint-error bitmask, so `err` most likely stays 0. `arm_current_status` = 9 (RM_STOP_E) is sticky after any stop and is not cleared by `clear_system_err` (`alix/ros/hw/alix_hw/src/rm_side_system.cpp:1856-1860`), so it cannot be used alone. `rm_set_arm_emergency_stop` is Gen-4 only and returns -4 on Gen-3 (SDK :7226-7240). The SDK ships both x86-64 and aarch64 `libapi_c.so`, so the same status code runs on the operator PC and on the Jetson.
- **Control app v14 (E-stop blind spot).** The runner's run and sequence loops watch only `run_state` and error codes (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:122-138, 219-271, 641-700`). An E-stop that ends the program without an error code would very likely print "RUN COMPLETE … no error" and exit 0, and a sequence would carry on: a hand step still runs and, if the button has been released, the next program starts by itself (inferred; drill DR3, DR10). v53 shows only the text "Software Slow Stop is not the hardware E-stop" and no E-stop state (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:3177`).
- **alix_workspace (E-stop field evidence).** On 2026-08-06, with the E-stop pressed, both controllers rejected pole commands (ret 1), one E-stop chain covered both arms, and end-port power survived, so the hand kept working (`alix/docs/TEST_PROCEDURE.md:216-242, 453`). The software arm stops do not stop the pole, which kept moving for up to 2.6 s and needs `rm_set_lift_speed(0)` (`alix/core/python/alix_dispatch/src/alix_dispatch/peripherals.py:189-199`). ALIX's recovery ladder is: release the button, clear errors, read back, re-enable joints if needed, then pendant or power cycle (`alix/docs/TEST_PROCEDURE.md:240-242`; `rm_side_system.cpp:1919-1935`); `rm_clear_system_err` can return 0 without clearing joint code 16384 (`alix/core/python/alix_dispatch/src/alix_dispatch/program.py:236-240`). ALIX code assumes a physical E-stop drops joint power and enable flags (`alix/ros/hw/alix_hw/README.md:449`; `rm_side_system.cpp:1855`), which conflicts with the operators' report that it stops everything without cutting power; the drill must measure which flags drop. The RM75 wrist joints 5–7 have no brake inside (HW p.34).
- **alix_workspace and control app v53 (recovery, dual start):** A stopped arm may rest on the commode, and "run again" starts with an unchecked transit out of that pose; ALIX hit this (H40) and built a read-first recovery that parks one arm at a time (`alix/tools/dual_arm_flow.py:6180-6188`). v53 synchronises only the launch of the two runner processes, so the motion-start skew is unbounded; the runner's own start prompt, printed after all its gates, can serve as a READY/GO handshake (inferred) (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:5654-5709`).

**Can it be borrowed from ALIX Studio?** **Partial** — None of the robot-facing core (connect, mode, run, stop, recovery, verdict) exists in the Studio, and a test pins that it has no execution channel. Lite can borrow front-end parts: the Simulate live view with its external joint feed (the live mirror), `showStudioModal` for the REAL confirmation, `setStatus`/`showErrorToast`, and the chip and stale-on-edit pattern and CSS. The `gateState()` logic is not reusable, because it computes X1 geometric-config gates (`studio/web_editor/editor.js:15760-15800`).
- *Other sources:*
  - Runner v14 — Borrow + adapt: a pinned, Lite-owned copy run as a subprocess, always with `--ip/--port/--sdk` and explicit speeds, plus opt-in flags that are default off and proven unchanged by diff (`--telemetry`, `--require-mode`, `--expect-size/--expect-name`, `--track-ref`, `--set-global-speed`, a stdin GO/STOP channel, a finish-event print). One change is not opt-in: the E-stop fix (read the E-stop IO in `arm_state()` and the run and sequence loops, refuse to start while it is held, report "interrupted" instead of "RUN COMPLETE", never start the next sequence step after an E-stop, print parseable `ESTOP ENGAGED`/`ESTOP CLEAR` lines).
  - alix_hw `rm_side_system.cpp` and ALIX `TEST_PROCEDURE.md` — Reference: the clear, read-back and re-enable order for the E-stop recovery and the field facts above.
  - UI v53 — Reference only: REAL confirmation wording, one-click STOP that is disabled after the first click, barrier launch; do not port the Tk UI, Multi Run / Load Sequence (they raise TypeError) or completion decided from status text.
  - alix_dispatch — Borrow + adapt: `open_arm`, `require_real`, `restore_run_modes`, the SIM pole/hand skip, the `gate_real` ladder, `stop_program`, `start_gap_deg`, `played_fraction`, `phase_end` and the `ArmEventLog` pattern; also the target library for a later in-process session.
  - alix_workspace tools — Borrow: `probe_stop_policy` on the `cleaning_tracks dry` in-place track for commissioning; the `cmd_recover` read-first pattern for recovery.
  - Control app `usage.py` — Borrow the pattern: STOP ALL = arm slow stop + pole stop + hand freeze.
  - ALIX emulator — Engineering tests only: a fake controller for CI; it never sends program-finish events.
  - Trajectory prototype — Not applicable: no robot I/O.
- *Note on the inputs:* Partly accurate. The note is a feature list, not a borrow claim, and today it is only partly met. Connecting exists in the control app, as one runner connection per action. IP entry exists only in the runner (`--ip/--port`), not in the UI. The real/sim toggle exists (v53 SIM/REAL/CHECK), but it switches the controller's dry-run mode, not a 3D simulation. ALIX Studio provides none of the three. The note does not mention the IO E-stop; neither the control app nor ALIX Studio shows its state today.

**Recommendation.** Build R4 in Lite's backend around a pinned, Lite-owned copy of runner v14, run as a subprocess with explicit `--ip/--port/--sdk` and speeds, and add only opt-in flags and the E-stop fix to it. Take the rules ALIX learned on hardware (read-back mode with the dry-run pole/hand skip and mode restore, a simulation licence keyed by content, several completion witnesses, a checked transit, read-first recovery) and ALIX Studio's interface patterns (live mirror, confirmation modal, status line and toast, readiness chips). Offer three plainly named run targets, open in Simulation, and suggest a robot dry run before the first real run of new content. Treat the IO E-stop on both controllers as the primary safety stop: show its state per arm, fail closed when it cannot be read, block motion on both arms while either is active, report interrupted runs honestly and guide recovery; keep the software STOP as a labelled convenience and test it on the robot (G4). Build the same code for the operator PC (x86-64) and the Jetson (aarch64). Keep operator REAL runs disabled until commissioning (the E-stop drill items DR1, DR3, DR9, DR10 and DR11, the stop policy and the finish event in particular) is recorded; move to an in-process `alix_dispatch` session later.

**Approach**
1. **Settings and Test connection (build).** Per-site JSON with left/right IP and port, SDK path (x86-64 or aarch64 build of the RealMan SDK, chosen by platform), runner path and runner sha256, and per arm the E-stop IO number, its active level and the last-drill date; IPv4 and 1–65535 validation; engineer-only edits. The test is a raw TCP connect with a 2 s timeout (never during a job on that arm), then runner `--get-arm-mode` and `--list`; it warns if a controller holds the other side's Lite programs (possible IP swap). It also reads IO1–IO4 modes and warns when no IO is in mode 5 or the two arms differ.
2. **Lite runner copy (borrow + adapt runner v14).** Vendor the runner, `hand.py` and `pole.py`, pinned by sha256. Add the opt-in flags above, all default off, and the E-stop fix. Prove the default paths unchanged, apart from the E-stop fix, by diff and by fake-SDK tests in the style of `alix/core/python/alix_dispatch/test/test_rm_upload.py`.
3. **Run target and mode (rules borrowed from `alix_dispatch.runmode`).** Lite opens in Simulation. Dry run: `--set-arm-mode simulation` verified by read-back; record the mode found; skip pole and hand; restore and read back afterwards. Real: the `showStudioModal` confirmation (both IPs, mode read back, region, transit size, checklist chips), then `--set-arm-mode real`.
4. **Pre-run gate (build; ladder rules from `alix_dispatch.ladder`).** A simulation ledger keyed by content sha256 in an append-only JSONL file (rungs `LITE_SIM`, `DRY_RUN`, `REAL`), the R12 gates, the R13 registry plus `--expect-size/--expect-name`, mode, codes and idle. Lite applies the region's pole → hand preset with the arm still and the pad off the surface and reads it back (R7); the operator ticks only the human items, shown with pictures from the regions deck.
5. **Transit check (build, using `start_gap_deg`).** With the mode REAL, read the joints and sweep the joint-space line to the first point at 1° steps in R3's collision world (commode_c scene, bin, poles, the selected venue's wall meshes via `plan_scene_validation.load_wall_manager`; about 2 ms per pose). Show it in 3D with its size; refuse on collision or above the limit. For dual starts, check both transits together or start them one after the other.
6. **Start (build a run-job manager).** One job per arm under a per-arm lock shared with upload, drag-teach, pole and hand; each runner in its own process group. Single arm: `--run SLOT --require-mode real --expect-size … --expect-name … --track-ref FILE --run-speed 20 --restore-speed 50 --timeout T --telemetry`, with T = expected × 1.3 + 30 s. Region with a preset: a workflow file ordered pole → hand → program only, with explicit `--workflow-overall-speed` and `transit_speed`. Dual: launch both runners without `--yes`, wait for both start prompts, release both with GO and record the skew.
7. **E-stop monitor and recovery (build; ladder from ALIX `TEST_PROCEDURE.md`).** The per-arm connection owner polls the E-stop IO (plus enable flags, and Modbus 302 if enabled) at 5–10 Hz and publishes the chip state; the server refuses motion routes while either chip is not CLEAR. On ENGAGED during a job: mark it interrupted, cancel pending stages, send `rm_set_lift_speed(0)` and hand freeze, and log IO, enable, power, brake, `err` and `run_state` with timestamps. The recovery wizard runs the steps in (h); its transit check is step 5's sweep.
8. **Monitor and stop (build; Studio live view borrowed).** Telemetry feeds the live mirror and "Cleaning: 45 s of about 92 s". STOP is sent once through stdin; STOP ALL also stops the pole and holds the hand. A Web Worker sends a heartbeat every 1 s and the backend stops a REAL run after 3 s without one; if the backend dies, the runner's stdin closes and it slow-stops. After a "did not start" exit, read the run state and stop any running program. After every outcome, check the speed-restore result and offer Retry.
9. **Completion (borrow `played_fraction` and `phase_end`).** Expected time = transit estimate + 0.35 s dwell + (lines − 1) × 5 ms. Report Finished only when all witnesses agree; otherwise "Stopped at about X %" from `phase_end`, or "Not confirmed". The finish-event print is enabled once commissioning confirms the event.
10. **Recovery, verdict and return (build; pattern from `cmd_recover`).** Read both arms and stop all devices. Then, one arm at a time: joint move to the safe pose if the sweep is clear; else reverse out (the executed part played backward as a temporary program at slow transit); else hand guiding. Then the verdict modal, a JSONL run record (content sha, slot, arm, target, mode found and read back, start/end, exit code, codes, final joints, witnesses, stop index, restore result, verdict, note) and Back to editing at the stop index. Edits reset the readiness chips (Simulated, On robot, Practised, Real-tested).
11. **Commissioning, then migration (borrow `probe_stop_policy` and `cleaning_tracks dry`).** Engineer only, per controller and after firmware changes: the E-stop drill (DR1–DR14 in the E-stop research notes; DR1, DR3, DR9, DR10 and DR11 must pass), stop policy, finish event present/absent, `plan_speed` read-back, mode read-back, speed restore, transit linearity, dual skew. Operator REAL stays disabled until this is recorded. Later, an in-process `alix_dispatch` session (`open_arm`, `require_real`, `global_speed_guard`, `run_program`, `stop_program`) with Lite's own stop token, because `session.py` does not handle SIGINT.

**Acceptance criteria**
- [ ] **Settings and target.** IP and port per arm are prefilled 192.168.1.10 / 192.168.1.103 / 8080; an invalid IPv4 address or a port outside 1–65535 is rejected; values persist; only the engineer role can edit them. Every runner command Lite builds contains `--ip` and `--port` equal to the settings, and `RM_LEFT_IP`/`RM_RIGHT_IP` do not change the target (unit test). Test connection answers Connected, Unreachable or Error within 5 s per arm (tested against an unreachable address), shows the mode read back, and warns when a controller holds Lite programs tagged for the other side.
- [ ] **Mode.** Lite opens in Simulation every time. Switching to Real robot shows a confirmation with both IPs, then the mode read back; a mismatch disables Start with a plain reason. With `--require-mode real`, a fake controller reporting SIMULATION at start refuses the run. In a dry run, pole and hand steps are skipped (the screen says so), and the mode found is restored and read back.
- [ ] **Refusals.** A REAL run is refused with a named reason, one test each: no simulation record (engineer override with a reason possible); failed record (no override); motion gates fail; slot size or name mismatch; mode not REAL; blocking code present; program already running; transit check fails; pole/hand read-back mismatch; checklist incomplete.
- [ ] **Transit.** With a fake current pose whose joint-space line to the first point passes through the commode model, Start is refused and the clash is shown in 3D. The transit size in degrees is shown before every start.
- [ ] **STOP and watchdogs.** STOP per arm and STOP ALL are visible without scrolling on every screen during motion. One click leads to `rm_set_arm_slow_stop` within 0.5 s on a fake SDK, with no dialog; repeated clicks send no second interrupt; STOP during a pole or hand step stops that device. The stdin STOP works on x86-64 Linux and aarch64 Linux (CI on the operator PC build, and one run on the Jetson). Killing the Lite backend during a fake run makes the runner slow-stop and restore the speed. Missing heartbeats for more than 3 s during a REAL run slow-stop it; a tab hidden for 10 minutes during a fake run causes no false stop. After every outcome the speed restore is attempted, its return code is shown and stored, and a failure offers Retry (no motion).
- [ ] **Completion.** A fake run that ends at 40 % of the file reads "Stopped at about 40 %" (±3 %) although the runner exits 0. A fake run whose final pose matches but whose playback took under 85 % of the expected time reads "Not confirmed". With the finish-event witness enabled, a run with no event is never shown as Finished.
- [ ] **Recovery and return.** After a stop away from the start or end, any new run (including Run again) is disabled until a simulation-checked recovery move has put the arm at its safe pose, one arm at a time. A verdict is required before the same content runs on the real robot again. The run record holds every field listed in step 10. Back to editing opens at the stop index or the marked point, and after any edit the chips show "needs re-check".
- [ ] **IO E-stop.** With a fake SDK: the chip shows ENGAGED within 0.5 s of the mode-5 IO going active; a stale reading (over 1 s) shows UNKNOWN and a controller with no mode-5 IO shows NOT CONFIGURED, and both block motion; `arm_current_status` = 9 alone never changes the chip. While either chip is not CLEAR, every motion route on both arms (Run, Sequence, Test on robot, Move to start, Pole, Hand, Drag-teach start, switch to REAL) is refused server-side with the reason, and upload still works. An E-stop during a fake run whose runner exits 0 is recorded "Interrupted: E-stop", no later sequence step or hand step starts, pole stop and hand freeze are sent, and the log holds the state fields. Recovery cannot reach CLEAR without the read-back, the speed restore and a passed transit check that includes the venue walls. Lite never calls `rm_set_io_mode`.
- [ ] **Dual arms and commissioning.** Start skew between the two `rm_set_program_id_run` calls is 0.2 s or less on the fake controller and is measured on hardware; a fault or STOP on one arm slow-stops the other within 1 s (fault injection); a dual run is blocked when either arm is not REAL. Operator REAL stays disabled until each controller has a commissioning record: E-stop drill DR1, DR3, DR9, DR10 and DR11 passed; software STOP ends a running uploaded program with the arm still within 2 s; finish event present/absent; `plan_speed` read-back; mode read-back; speed restore; transit path.

**For operators:**
- Three targets with plain names: "Simulation (no robot)", "Practice on robot (arms stay still)", "Real robot (arms move)". A banner on every screen: neutral, amber "PRACTICE: arms will not move", or red "REAL ROBOT: arms will move", always with a word and a glyph.
- One large STOP per arm and a STOP ALL, labelled "STOP (software)" (at least 44 px, Space and Esc), always visible during motion: "Stops the arm gently. In an emergency press the red E-stop button on the robot."
- An E-stop chip per arm in the robot bar: "Left E-stop: clear", or a red strip "E-STOP PRESSED (left arm): all arm, pole and hand movement is blocked". After release: "E-stop released. Follow the steps to get the robot ready again." The run reads "Interrupted by the E-stop", never "Finished".
- Progress and codes in words: "Moving to start position…", "Cleaning: 45 s of about 92 s"; 4116 reads "A joint went past its software limit. Press and hold the green button at the end of the arm to recover."
- After a stop: "The arm stopped part-way. Lite will first move it back to a safe position." Then four verdict buttons and Back to editing. Slots, speeds and paths stay in an engineer panel.

**Safety:**
- The IO-wired E-stop buttons are the primary safety stop. Lite only shows their state and adds interlocks; it never writes IO modes. The manuals give no stop category or safety rating for an IO E-stop, recommend safety interfaces over normal IO (HW p.32), and a high-trigger input may not trip on a broken wire (drill DR13), so the site's wiring must be reviewed by whoever owns machine safety.
- An E-stop during a run must never be reported as success, and no sequence step may start after it. The IO E-stop does not stop the hand and the software stops do not stop the pole, so Lite stops both itself. The wrist joints have no brake; re-enable joints only with the wrist supported.
- Whether slow stop ends a running stored program is unproven; it is tested (G4) before operator REAL, and the software STOP stays a convenience.
- The controller does not check the transit. Lite sweeps it, shows it, runs it at 20–30 % and refuses large moves; recovery comes before any new motion.
- The run speed persists controller-wide: restore it on every exit path and never interrupt the runner twice. A dry run never sends pole or hand commands and is never a commode-clearance check.
- One command source per arm: the Tk control app, the ALIX ROS driver and pendant programs must not drive an arm during a Lite run, and Lite's lock cannot see them. An IP swap sends a program to the wrong arm, hence engineer-only IPs, side-tagged programs and the live mirror.

| Priority | Size | Depends on |
|---|---|---|
| Must | XL (MVP, single arm without dual or live mirror: L) | R3, R12, R13, R7, R2, R5, R10/INT-CTRL, R1, NFR-UX, NFR-THEME |

**Open questions:**
- By "toggle between real and sim", did the requester mean Lite's 3D simulation, the controller dry run, or both?
- Does slow stop end a running uploaded program on these controllers? Who runs commissioning (`probe_stop_policy`), and when?
- Does the `program_run_finish` event arrive on both controllers in the runner's process? Does `rm_get_program_run_state().plan_speed` read back the global ratio when idle?
- What must "validated in simulation" include (clearance threshold, commode state per region), and who may override a missing record?
- Should a robot dry run precede the first real run of new content by default? Is a slower rehearsal file required for that run?
- What is each arm's safe pose for recovery, and is "reverse out" acceptable on contact strokes?
- What heartbeat timeout (3 s proposed) and transit size limit (for example 45°) should apply?
- E-stop wiring: IO3 carries the button on both controllers (decided). To record in the drill: its active level, and whether one button trips both controllers. Which flags drop (enable, power, brakes), and does the program end (`run_state` 0) or pause (2)? To be recorded in the drill.
- Who reviews the E-stop circuit against RealMan's advice to use safety interfaces, and how often is the drill repeated (RealMan asks at least yearly, HW p.35)?
- On the Jetson, are the controllers reachable at the same IP addresses and ports as from the operator PC, and does the Jetson run the runner with the aarch64 SDK unchanged?
- Are the controllers shared with ALIX production and the Tk app, and who guarantees a single command source?
- Are photos or video wanted with verdicts?

---

### R5 — Collision detection and graph highlighting

**Original (inputs):**
> Collision Detection and Graph Highlighting: If a collision occurs during real-arm testing or simulation view , the graph should stop at the corresponding joint value and highlight the collision point for easy identification and tuning. - Borrow from ALix Studio

**Revised requirement.** Lite shall check every simulated playback for collisions against the whole cell, stop playback and the joint plots at the first colliding row, and mark that row and its joint values on the plots and in 3D. Lite shall do the same for a collision that the controller reports during a real-arm test.

- (a) **What is checked.** Left arm vs right arm (including hands, fingers and the right wrist camera); each arm vs itself; each arm vs the toilet (body, seat and lid in the step's state), the cubicle walls, the floor and, when fitted, the bin. The robot, bin, UGV and poles come from the ALIX URDF, the toilet from the `commode_c` scene, and the walls from the selected venue file; a missing scene or venue file makes that check NOT CHECKED. The **full check** covers every row and is the gate before any real test. The **quick check** covers every 4th row and is labelled "quick". Contact by the contact links of the region's hand preset is *pad contact (expected)*; any arm structural link touching anything is a *collision*.
- (b) **Two-arm timing.** Arm-vs-arm results are given twice: at the nominal alignment from the flow model, and over a drift window built from the measured start lag and run-to-run timing spread. A contact that can occur only inside the window shows as "possible if the arms drift".
- (c) **Stop and highlight.** With "Stop at problems" on (the default), playback stops at the first colliding row. Each affected arm's joint plot shows a vertical line at that row, a marker on each of the 7 joint curves labelled with its value in degrees, and a shaded band over the colliding rows. The 3D view shows the colliding parts (robot links, toilet or wall faces) in the danger colour and names the pair in plain words.
- (d) **Event list.** Each event shows start/end time, duration, first/last row per arm, the parts (plain and technical names) and the kind (arm-arm, self, toilet, wall, floor, bin). Previous/Next and click-to-jump work from the list.
- (e) **Moves around programs.** The controller's approach move to each program's first point (as a joint-linear path), pole moves and hand changes are checked too. Before every real run, the approach from the arm's live pose is checked, and a predicted collision blocks the run.
- (f) **Real arm.** When the controller reports collision 4109, Lite takes the joints at the stop from a 5 ms joint capture of the run (fallback: joints printed by the runner with the code). It maps them to a row of the uploaded file with an ordered matcher, or reports "during the approach move". It shows the same markers, and the actual pose against the planned one in 3D. An IO E-stop press (the E-stop buttons wired to an IO input on each arm controller) is reported separately from a collision, as "E-stop pressed", never as 4109; its stop pose is mapped to the stop row of the uploaded file in the same way and shown with the same markers.
- (g) **Fix in editor.** Every event offers "Fix in editor" (R3 d).

**Findings**
- **ALIX Studio:** The Dual-Arm Collisions tab has a 0/1 bar chart in hard-coded dark colours, click-to-scrub, red recolouring of robot links, a "COLLIDING" label, Previous/Next and a log (`studio/web_editor/editor.js:14478-14559`). It plots no joint values, and playback never stops at a collision (`studio/web_editor/editor.js:14561-14581`).
- **ALIX Studio / alix_workspace:** The tab's endpoint passes no hand state and no extra collision objects (`studio/server.py:4355-4357`). The sweep checks only arm links plus palms (13 links per arm, no fingers), and extra objects are tested only against that slim set, so fingers never meet the toilet (`alix/core/python/alix_workspace/src/alix_workspace/dual_arm_plan_collision.py:165-182, 645-646`). The toilet, walls and floor are not checked.
- **Measured on V6 data:** On the V6 pair `L_P03P05` (from row 3424, pole 50 mm) and `R2_LIDTOP` (pole 150 mm), GRIP on both hands, full link sets find the right thumb touching `L_CameraHolderLink` on 41 rows (`R2_LIDTOP` rows 2788-2828) and `L_Link7` on 10. The Studio's slim set finds nothing. With full hands and curated self pairs, every row costs about 2-3.5 ms, about 30 s for a 60 s pair. FCL managers are not thread-safe, so the checks run in one worker process (`studio/body_collision.py:150-171`).
- **ALIX Studio / alix_workspace:** The building blocks of the world exist. `fixture_manager` poses the toilet parts at a lid/seat articulation but shifts the toilet only along X (`studio/body_collision.py:401-464`). `load_wall_manager` takes a mesh override, and the wall check refines coarse hits and reports a depth (`alix/core/python/alix_workspace/src/alix_workspace/plan_scene_validation.py:117-155`). `fixture_contact_faces` and `paintCollisionFaces` mark touched toilet faces.
- **Measured on V6 data / ALIX contact lists:** With GRIP on `R2_LIDTOP`, the lid contact on 36 of 188 sampled rows is made by finger links, all in ALIX's `glove_frame_2` allowance. That allowance was derived for the `open_tenth` hand (`alix/ros/orchestration/alix_tasks/config/contact_links.yaml:81-110`). Thumb links touch on 4 rows, and `R_ConnectorLink`, a structural link, on 1 row: that one is a collision. Contact versus collision must be decided per hand preset.
- **ALIX tools (dual-arm timing):** Two upload-channel arms are not row-locked: each controller does its own approach move and start lag. ALIX models the drift with measured timing envelopes, but refuses to extrapolate beyond 4,281 rows, while V6 programs reach 10,983 (`alix/tools/dual_arm_verdicts.py:862-973`). ALIX moves each arm to its first point one at a time, with the partner static (`alix/tools/dual_arm_flow.py:3763-3800`).
- **Control app v14:** The runner watches the arm's error codes every 0.5 s and prints collision code 4109 once, but not the joints at that moment, and it never stops a program itself (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:122-141, 244-268`). Blocking for the fallback path: the runner must print the joints with each code.
- **RealMan documentation (IO E-stop):** A controller digital input can be set to "input emergency stop function" (`set_IO_mode` mode 5), and the 5 ms UDP state report carries `arm_current_status`, whose value `"stop"` means emergency-stop state (RealMan Gen-3 JSON protocol, `alix/docs/Manuals/Realman/RealMan_Gen3_04_JSON_Protocol.pdf`, p.131). This field alone cannot label the event: it is a sticky last-run state that stays at STOP (9) after any stop, software stops included, and `clear_system_err` does not reset it (`alix/ros/hw/alix_hw/src/rm_side_system.cpp:1856-1860`; E-stop research §2). An IO E-stop is therefore told apart from a 4109 collision by the E-stop IO state read with `rm_get_io_state` on the mode-5 IO (R4 (h)), logged with timestamps alongside the capture. Which fields and error codes this robot's controllers report on an IO E-stop press must be checked on the robot.
- **alix_dispatch:** Production capture tools exist. `UdpRecorder` records the controller's 5 ms push (joints and error codes) and refuses a push owned by another host; `push_error_codes` gives each code's first and last time; `verify_replay_from_frames` maps executed frames to file rows with an ordered matcher (`alix/core/python/alix_dispatch/src/alix_dispatch/track.py:974-1080, 2037-2075`). The controller's `plan_num` stays at 1 for a whole replay, so it cannot index points.

**Can it be borrowed from ALIX Studio?** **Partial** — Borrow the red-link highlight with colour restore (`dacHighlight`), the collision log with Previous/Next and click-to-jump, toilet-face painting, the toilet collision manager at an articulation, the curated self-collision pairs and the content-hash result cache. The Studio never plots joints and never stops at a collision. It checks no toilet, walls or fingers, assumes both arms start together with no drift, and has no real-arm input because it has no execution channel.
- *Other sources:*
  - **alix_workspace** — Borrow + adapt: re-implement the ~60-line sweep loop (`_ArmCollision`) in memory with full hand sets, named toilet parts, hand joints from counts and explicit pole and bin; use `cross_arm_separation` for the drift window, the wall manager with approximate depth, and `fixture_contact_faces` as-is. Fix `classify_pair`, which reads the bin's `lid_opening_cover1` as toilet (`alix/core/python/alix_workspace/src/alix_workspace/redundancy.py:90-105`). Pin the ALIX commit.
  - **ALIX tools** (`dual_arm_verdicts.py`, `dual_arm_flow.py`) — Borrow + adapt: copy the drift-window model and its 25 mm clearance floor, and re-fit its timing envelopes on Lite runs; follow `_bridge_to_start` as the reference for one-arm-at-a-time approach moves.
  - **ALIX contact-link lists** (`contact_links.yaml`, `glove_frames.yaml`) — Borrow as data: re-derive per hand preset with `alix/tools/regen_contact_links.py`; GRIP needs the requester's ruling.
  - **alix_dispatch** — Borrow + adapt (preferred real-arm path): `UdpRecorder`, `push_error_codes`, `verify_replay_from_frames` and `MotionMonitor`, run inside the process that owns the arm connection.
  - **Trajectory prototype** — Borrow + adapt: extend its plot cursor shapes with a band, 7 value markers and an annotation (`lite/Alix_Studio_Lite_Trajectory_V2.html:1134-1141`).
  - **Control app v14** — Borrow + adapt (fallback path): a small change so the runner prints the joints it already reads with each error code.
  - **RealMan SDK** — Operational: read the collision-detection level on both arms before tests (`alix/external/butterfli-realman-alix/Python/Robotic_Arm/rm_robot_interface.py:585-618`).
- *Note on the inputs:* Partly accurate. The Studio's Dual-Arm Collisions tab does detect arm-vs-arm, self and bin contacts, and shows them as red links with a timeline, a log and Previous/Next. It does not plot joint values or stop at a collision, does not check the toilet, walls or fingers, assumes both arms start together, and cannot take real-arm input.

**Recommendation.** Build Lite's own collision module on alix_workspace and run it in one worker process. Check every row with full hand geometry, curated self pairs and the whole cell (URDF + `commode_c` scene, plus walls from the selected and verified venue file), and classify contact by the hand preset's contact links. Report arm-vs-arm results both at the nominal alignment and over the drift window; when contact is possible inside the window, advise running the pair one arm after the other. Reuse the Studio's 3D highlight, log and face painting, build the joint markers and stop-at-first-collision on the R2 plots, and map real 4109 stops and IO E-stop stops to rows from a 5 ms capture, with the runner's printed joints as the fallback.

**Approach**
1. **World builder (build on alix_workspace and the Studio's `fixture_manager`).** Inputs: the ALIX URDF, the `commode_c` scene in the step's state, the selected venue file, region pole and hand, bin setting. Objects: left arm + hand; right arm + hand + `camera_link`; named toilet parts at the scene's full pose (`alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml:32-68`); walls from the venue's `washroom_structure` mesh via `load_wall_manager` (`alix/core/python/alix_workspace/src/alix_workspace/plan_scene_validation.py:117-155`); floor plane; the bin set when fitted. Use the curated self pairs (`studio/body_collision.py:354-399`) and the SRDF allowed-collision list with the two disabled thumb-vs-camera pairs re-enabled (`alix/ros/manipulation/alix_moveit_config/config/alix.srdf:1442, 1452`).
2. **Sweep worker (build).** One worker process. Full mode checks every row; quick mode checks every 4th row and re-checks every row around hits. Events are keyed by row. Results are cached by a hash of the files, pole, hand, bin, scene, venue file and mode, following the Studio's cache (`studio/server.py:4303-4316`).
3. **Contact classifier (build + data).** One contact-link list per hand preset. Start from `glove_frame_2` plus the GRIP evidence, and ask the requester to confirm. Warn when depth exceeds 20 mm, and treat depth as approximate.
4. **Drift-window check (copy and adapt the ALIX model).** Seed it with ALIX's timing envelopes and re-fit after about 10 Lite runs per program-length class. Run it in the background and show contacts inside the window in amber.
5. **Approach, pole and hand-move checks (build).** Offline: check a joint-linear approach from the previous program's last row; sweep pole moves with the arm fixed, which also covers the bin interlock; flag pole or hand changes made while a hand link is within 20 mm of the toilet. Live: before each real run, check the approach from the arm's current pose, which the runner already reads.
6. **Front end (borrow highlight, log and face painting; build the plot markers).** Event list with Previous/Next; stop-at-first-collision in the R3 player; line, band, 7 value markers and annotation on the R2 plots, styled from theme tokens; colliding parts and faces in the danger colour in 3D.
7. **Real-arm path (with R4; adapt alix_dispatch).** Run the 5 ms capture inside Lite's robot I/O process. Take the first 4109 frame, or the first frame at or after the time the E-stop IO went active (R4 (h); not `arm_current_status` alone), map it to a row with `verify_replay_from_frames`, and draw the planned pose as a ghost. Label the event "collision (4109)" or "E-stop pressed". Fallback: the runner prints joints with each code, and Lite searches for the nearest row within a time prior. When several rows match (repeated strokes), the ambiguity is shown.
8. **Regression suite.** A seeded collision confirmed by a brute-force every-row check; the V6 thumb-to-wrist case; `R2_LIDTOP` finger contact vs `R_ConnectorLink`; no wall hits on the V6 known-good recordings with the selected venue (the 800 mm default gives 28 of 188 sampled rows on `R2_LIDTOP`, so a wrong venue is caught); the ALIX plan pair that touches at about 27.3 s and 29.7 s, which reportedly ran 13/13 on hardware, so the model can disagree with the robot (`alix/core/python/alix_taskgen/GENERATION_FINDINGS.md:2782, 3107`).
9. **Operations.** Before tests, read both arms' collision-detection level, and block or warn loudly if they differ or one is 0. Changing the level is an engineer action with confirmation. Document how to clear a latched 4109 at the pendant.

**Acceptance criteria**
- [ ] The result header lists which checks ran: arm-arm nominal, arm-arm window, self, toilet [state], walls [venue], floor, bin [setting], approach moves. A missing check shows as "not checked": a missing scene or venue file gives NOT CHECKED for the toilet or walls, and a venue not yet verified with known-good recordings carries "venue not verified". "No problems found" never appears alone.
- [ ] **Seeded collision.** On a V6 file edited so that a brute-force every-row check finds a collision, the full check reports exactly the same first row. The quick check reports it within ±4 rows and is labelled "quick".
- [ ] With "Stop at problems" on, playback halts at that row. The 7 markers show the row's millidegrees / 1000 to 0.01°, the band covers exactly the colliding rows, and the colliding links and faces are in the danger colour. Each event shows kind, plain-language parts, technical pair, first/last row per arm and time, and Previous/Next visits the events in order.
- [ ] **Contact cases (measured).** With the ruled GRIP list on `R2_LIDTOP` against the closed lid, finger contacts are "pad contact (expected)", and the `R_ConnectorLink` contact (1 of 188 sampled rows at ALIX's `commode_c` pose) is a collision. On the designed-aligned V6 pair, the full check reports `IRH_right_thumb_*` against `L_CameraHolderLink`/`L_Link7` around `R2_LIDTOP` rows 2788-2828; the quick check reports it or says "fingers approximated". The Studio's slim set is never accepted as a pass.
- [ ] **Drift window.** For two tracks released together, the window equals ALIX's `clock_envelope_window_s` for the same timing envelopes. Contacts found only inside the window show "possible if the arms drift".
- [ ] **Approach moves.** A flow whose approach path hits the toilet is flagged before upload. Before a real run, a live start pose whose approach would collide blocks the run with a plain message.
- [ ] **Real-arm mapping.** A recorded 5 ms capture with a 4109 frame maps the stop to a row through the ordered matcher. Without a capture, the runner's "code + joints" line maps to the nearest row. Joints more than 2° from every row give "during the approach move". When two or more rows inside the time prior match within 1°, the ambiguity is shown. A recorded IO E-stop press during a run is shown as "E-stop pressed" (not as a collision), mapped to a row by the same matcher, with the same markers.
- [ ] **Performance.** The full every-row check of a 60 s pair (about 11,000 rows per arm) finishes in 60 s or less on the dev box, with progress. The quick check takes 10 s or less, unchanged inputs return from cache in under 1 s, and the drift-window check runs in the background.

**For operators:**
- Parts are named in plain words ("right wrist", "left hand thumb", "toilet lid", "cubicle wall", "bin cover"). The technical pair sits under "Details".
- Three states, each with colour, symbol and word: *Collision*, *Close call / possible if the arms drift*, *Clear*. Expected pad contact is shown in neutral grey.
- Every event offers **Show me**, **Fix in editor** and **Re-check**. "Stop at problems" is on by default, with a visible switch.
- After a real-arm stop: "The controller stopped the right arm (collision detected) at 0:12. Clear the alarm on the teach pendant, then press Re-check. This can be real contact or a motion that is too fast."
- After an E-stop press: "The E-stop was pressed at 0:12 (right arm, row shown). Release the E-stop when the area is safe, then follow the recovery steps." Recovery follows the E-stop guidance in NFR-SAFETY.

**Safety:**
- Never accept the Studio's palms-only check as a pass: it misses a thumb-to-wrist contact on V6 data. Re-enable the SRDF-disabled thumb-vs-camera pairs.
- An arm-vs-arm "clear" at the nominal alignment is not "clear" on the robot. Use the window result, and run uncertain pairs one arm after the other.
- On the last probe the right arm's collision detection was off (level 0) and the left arm's was at level 3 (`alix/ros/description/rm_description/docs/arm_version_info.md:27-33`). Align both before real tests.
- 4109 can fire without contact on aggressive motion (`alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:675-682`), and the runner never aborts a program. Suggest slowing the segment, keep the E-stop within reach, and run first tests from a slowed copy.
- The IO E-stop on each arm controller is the primary safety stop; it stops the robot without cutting power (F4). Lite only reports it and maps it to a row; it never treats an E-stop stop as a collision result or a collision result as an E-stop.

| Priority | Size | Depends on |
|---|---|---|
| Must | XL (without the real-arm path: L) | R3, R2, R7, R10 (b, c), R4 (real-test path, runner change); venue selection and verification; the requester's contact-link ruling |

**Open questions:**
- Which hand links may touch the toilet with GRIP?
- What near-miss distance should warn? ALIX uses a 25 mm floor (`alix/tools/dual_arm_verdicts.py:369`).
- May Lite take the controller's realtime push during tests (does anything else consume it on this robot), or should the runner print the joints?
- Which collision-detection level should both arms use (left 3, right 0 on 2026-08-13)?
- Should the full check and the window check be required before every real test, or only before upload?
- Should pairs marked "possible if the arms drift" run one arm after the other by default?
- What exactly does each controller report on an IO E-stop press (`arm_current_status` "stop", an error code, or both), and does the press reach the partner arm's controller? To be checked on the robot.

---

### R6 — Adjustable plots and editing windows

**Original (inputs):**
> 6. Adjustable plot : The UI should allow users to resize and adjust the graph, and editing windows based on their requirements. - check the /home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html

**Revised requirement.** Lite shall let the operator resize, zoom and rearrange the plots and editing panels of the Trajectory Optimisation screen, and shall remember the layout. "Graph" means the joint-position, speed-utilisation and comparison plots; changing data by dragging plot points is out of scope, because data changes go through R2.

- (a) The plot area can be resized against the 3D simulation and the editing panels, and each stacked plot's height can be changed, by drag or keyboard. Plots re-draw within 100 ms of a resize.
- (b) Any one plot can be maximised, and one click or Esc restores the exact previous layout.
- (c) Zoom and pan act on the time axis. Plots of the same file stay on the same time window. Before/after plots stay on the same part of the path, linked by raw-line position. One click resets the view.
- (d) Individual joints can be shown or hidden.
- (e) Editing panels collapse, expand and resize; floating dialogs move and resize.
- (f) The layout is remembered per screen, with a "Reset layout" action.
- (g) The mouse wheel scrolls the page; zoom uses Ctrl+wheel, pinch or the toolbar. There is no horizontal page scroll at 1366×768.
- (h) Where an optimised result is shown, the plots carry duration markers: a target-duration line and an achieved-duration line on the time axis, plus the closest feasible time when the target could not be met, with the binding-limit window shaded (R2, R10). Markers keep their positions under zoom, resize and maximise.

**Findings**
- **Trajectory prototype:** Plot heights are fixed in CSS (600 px and 520 px); only the width follows the window (`lite/Alix_Studio_Lite_Trajectory_V2.html:125-129`).
- **Trajectory prototype:** Zoom, pan, legend toggles and PNG export work. But the mouse wheel zooms the plot instead of scrolling the page, the plots are not linked, and the x axis is the row number (`lite/Alix_Studio_Lite_Trajectory_V2.html:847-867`).
- **Trajectory prototype:** The compare cursor links before and after by the same seconds. The optimised file is shorter, so the two plots then show different strokes (`lite/Alix_Studio_Lite_Trajectory_V2.html:1290-1304`).
- **ALIX Studio:** Dock separators resize by drag and keyboard (16 px, Shift 64 px, Home/End), clamp to 180–620 px, save on release and are exposed as aria separators (`studio/web_editor/editor.js:7564-7645`). Their storage calls are not wrapped in try/catch, unlike the dialogs' (`studio/web_editor/editor.js:7588`).
- **ALIX Studio:** Dialogs can be dragged and resized by pointer or keyboard. Their size and position are saved, and they are pulled back on-screen after a window resize (`studio/web_editor/editor.js:8389-8523`). "Maximize view" restores the exact previous layout, but only for the 3D viewport (`studio/web_editor/editor.js:7538-7559`).
- **ALIX Studio:** Its two plots have fixed heights (190 px and 64 px) and cannot be resized. Their Plotly layouts hard-code the text colour `#e6edf3`, which gives 1.18:1 contrast on the white panel (`studio/web_editor/editor.css:928`, `studio/web_editor/editor.js:14488`).
- **ALIX Studio:** The dark theme exists only in CSS; no script sets `data-theme` (`studio/web_editor/editor.css:131-134`). The dock tests are static source-text checks, not browser tests (`alix/core/python/alix_taskgen/test/test_studio_interaction_guards.py:2388-2525`).

**Can it be borrowed from ALIX Studio?** **Partial** — The resize machinery is in ALIX Studio (dock separators, dialog drag/resize/save/clamp, maximise with restore, and vendored Plotly 2.35.2 loaded by `ensurePlotly`), but it is applied to the 3D viewport and the docks, not to plots. Horizontal plot splitters, per-plot maximise, a resize bus, linked zoom, a token-based plot theme and per-screen layout memory must be built.
- *Other sources:*
  - **Trajectory prototype** — Borrow + adapt: keep its Plotly config (responsive, no logo, lasso removed) and its resize after a step is shown. Change the x axis to seconds and turn off wheel zoom.
- *Note on the inputs:* Partly accurate. The HTML gives zoom, pan, legend toggles and responsive width, but it has fixed plot heights, no resizable windows, no maximise and no linked zoom. The resize machinery to borrow is in ALIX Studio, which does not apply it to plots.

**Recommendation.** Lift the Studio's separator, dialog and maximise code into a small layout library, and apply it to a curated Trajectory screen: [3D simulation | plot column | editing panel], with stacked plot cards separated by horizontal splitters. Use fixed splitters, not free docking. Add linked zoom (by time within a file, by path position for before/after), labelled plot toolbars and per-screen layout memory.

**Approach**
1. **Splitter (borrow + adapt):** generalise the Studio's `setDockWidth`/`initDockResize` into `initSplitter(el, {axis, cssVar, min, max, storageKey})`. Keep the aria separator, the 16/64 px keyboard steps, the neighbour clamps and save-on-release. Wrap storage in try/catch under `alixLite.layout.*`.
2. **Screen grid (build)** from CSS variables. Minimum sizes at 1366×768: simulation ≥ 320 px, plot column ≥ 360 px, each plot ≥ 160 px. The editing panel can collapse.
3. **Plot cards (build):** a header with text-labelled buttons (Zoom, Move, Reset view, Maximise) and J1–J7 chips synced with the legend. Maximise stores the splitter sizes and restores them on Restore or Esc (the Studio's maximise pattern).
4. **Resize bus (build):** a ResizeObserver per plot body calls `Plotly.Plots.resize` inside `requestAnimationFrame`. Above 15,000 points use `scattergl`, as the prototype does, or decimate per pixel.
5. **Linked zoom (build):** plots join a link group and pass on `plotly_relayout` under a re-entrancy flag. Mode "time" links plots of one file; mode "path" links before/after through the raw-line index from the R2 processor.
6. **Defaults:** drag zooms (box-select only in sub-region mode); `scrollZoom` is off and Ctrl+wheel zooms; double-click resets.
7. **Dialogs and Plotly (borrow as-is):** the Studio's dialog drag/resize/save/clamp for floating panels (findings list, sub-region details), with renamed storage keys; `ensurePlotly` with vendored Plotly 2.35.2 served from Lite's `/vendor/`.
8. **Duration markers (build):** from R2's OptResult draw vertical shapes for the target and achieved durations (and T_min when the target is not met) and a shaded rectangle for each binding-constraint window, each with a text label. They are plot shapes, so linked zoom and resize carry them.
9. **Plot theme (build):** `plotTheme()` reads the text, grid and background tokens and `--joint-1..7` colour tokens. Do not copy the Studio's `#e6edf3` literals.
10. **Tests (Playwright):** splitter drag and keyboard, layout after reload, maximise and restore, both link modes, duration markers, no horizontal scroll at 1366×768, blocked storage. Re-write the Studio's static dock guards against Lite's code.

**Acceptance criteria**
- [ ] Dragging the simulation/plots separator resizes both sides continuously. Plots re-lay out within 100 ms of release, and minimum sizes hold.
- [ ] Each plot's height changes by drag (minimum 160 px) and by keyboard (Arrow ±16 px, Shift+Arrow ±64 px, Home/End).
- [ ] Maximise fills the plot area. Esc or Restore returns every pane to its prior size (±1 px).
- [ ] Zooming one plot of a file applies the same time window to that file's other plots in the same frame. Zooming the "before" plot shows the same raw-line span in the "after" plot. "Reset view" restores all of them.
- [ ] The mouse wheel over a plot scrolls the page unless Ctrl is held.
- [ ] Splitter sizes, collapsed panels and visible joints are restored after reload, and "Reset layout" restores the defaults. With browser storage blocked, the page still works.
- [ ] At 1366×768 the simulation, at least one plot and the editing panel are visible, with no horizontal scroll.
- [ ] For an optimised region the plots show labelled target and achieved duration markers; when the target is below the shortest feasible time, the closest feasible time and the binding window are also shown. The markers stay correct after zoom, resize and maximise.
- [ ] Plot text and grid colours come from tokens, and tick labels have at least 4.5:1 contrast.

**For operators:**
- 8 px grab handles show a visible bar on hover or focus, and an "Esc to restore" hint appears when a plot is maximised.
- Duration markers are labelled in words ("Target 25 s", "Achieved 25.0 s", "Closest feasible 19.2 s"), not by colour alone.
- Toolbar buttons have text labels, "Reset view" is always visible, and hiding a joint keeps the current zoom.

**Safety:** Maximising or resizing must never hide the region's check status, the robot-state indicator or the Stop control during a run.

| Priority | Size | Depends on |
|---|---|---|
| Should | M | INT-TRAJ, R2, R10, NFR-THEME |

**Open questions:**
- Is a pop-out plot window for a second monitor needed?
- Should the layout be stored per user or per machine? Lite runs on the operator PC now and on the Jetson in production (F1), so a layout saved on one does not follow to the other unless it is exported.
- Which plots are shown by default?

---

### R7 — Pole height and hand position

**Original (inputs):**
> Pole Height and Hand Position: Each region should have independent settings to adjust the pole height and robotic hand position before the person start drag and teach cleaning. - borrow from alix studio

**Revised requirement.** Lite shall store a pole height and a hand setting for each region and arm, apply them to that arm in a **Prepare robot** step before drag teaching, and record the values actually used with the recording.

- (a) **Settings per region.** A pole height of 0-315 mm and a hand setting: a named preset of six channel values 0-1000 (0 = closed, 1000 = open), in which a channel may be "hold". Defaults are seeded from the V6 workflows, with the ALIX commode C teach sheet shown as an alternative, and stay marked "proposed" until confirmed. Only a supervisor edits defaults. Thumb-held presets are the default unless a thumb pose is needed.
- (b) **Apply before teaching.** **Prepare robot** on the region card moves the pole and the hand in the region's order (pole then hand, or open fingers, then pole, then hand) and shows target and measured values. START stays disabled until the pole is within ±5 mm and every commanded hand channel is within ±15 counts or reported settled. Only a supervisor can override this, and the reason is logged.
- (c) **Interlocks.** Lite never moves the pole or hand of an arm that is in drag mode, moving faster than 1.6°/s, holding the fixture or a tool part-way through a task (except at a declared hand event), or driven by another client. Before moving, the operator confirms that the cloth or pad touches nothing, that hands are clear, and that the arm and hand are clear of the toilet and the other arm.
- (d) **Bin rule.** The bin geometry and its positions come from the URDF and its peripheral-joint states (STOWED, EXTENDED, LOWERED). The bin's runtime position cannot be read, so it stays a declared setting, chosen per session and shared with the R10 bin toggle (fitted yes/no and position); one setting drives both the simulation and this interlock. With STOWED, Lite refuses any pole move whose travel from the current height to the target overlaps the stowed-bin band, and any move while the current height is inside that band, and explains why. The band is computed from the URDF model at start-up (as in R10), not hard-coded.
- (e) **Provenance and fine-tune.** The measured pole height and hand values used for each recording go into its provenance sidecar (R1). The operator may fine-tune on the card, for example ±10 mm or another preset. The values used are always recorded, and they become the region default only with supervisor approval.
- (f) **Recorded values win downstream.** A joint-space recording replays correctly only at the pole height and hand pose it was taught with, so R3, R4, R8 and R13 use the kept recording's own values, not the region's current default. Changing a default later flags existing recordings "taught at other pole/hand".
- (g) **Hand events inside a task.** Tool and glove tasks that need a hand action part-way (for example gripping the brush) are taught as sub-steps (R1). The hand action is its own step: drag mode off, arm still, the operator confirms the hand is at the grasp point, then the hand moves and is read back. The event is stored in the catalogue and in the next sub-step's sidecar, and replayed as program → hand step → program.

**Findings**
- **ALIX Studio:** The region model has no pole or hand field. The region editor accepts only coverage, surface, spacing, lid/seat state and similar fields (`studio/server.py:4074-4077`).
- **ALIX Studio:** The concept exists only inside the excluded generator (X1). The Workbench has a "Pole (written to the task)" field with named heights, a custom height of 0-0.315 m and a stowed-bin warning, and it writes a pole stage into the generated task, which the ROS orchestrator runs (`studio/web_editor/template.html:235-238`; `studio/stage_sequence.py:181-207`). The viewer's pole selector only pins the pole for IK.
- **ALIX Studio:** The hand state follows the chosen glove pad: "It is derived, never chosen". Studio never commands hardware, so it cannot apply anything before a teach (`studio/web_editor/editor.js:9959-9963`). Per-region hand settings are still real practice: ALIX cleans the inner rim and hinge with finger pads at half or quarter grasp (`studio/region_mesh_loader.py:306-314`).
- **Control app v53/v14:** The apply machinery exists. The runner's `--manual-pole MM`, `--manual-hand-raw` (six values) and `--manual-hand PCT` (four fingers, thumbs held) print target and final values that Lite can use as read-back, and the HAND / POLE tab has GRIP `0 0 0 0 150 1000` and ALL OPEN presets (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:756-909`). Missing for R7: no per-region preset, pole/hand and drag teach on different tabs, no confirmation on APPLY, and neither runner pole path checks that the arm is still. The Page-02 workflow format also carries pole and hand steps and V6 uses it; Multi Run raises a `TypeError` in v53, so Lite must not rely on it.
- **Control app helpers:** `usage.py` holds the rule ("the pole moves only with the arm still; never move pole or hand with the pad on a surface"), the arm-still check (under 1.6°/s for 6 polls), the `prestroke` order (hand open → arm still → pole → hand), stop-all, and read-only `pole read` / `hand read` (`lite/upload_channel/usage.py:24-25, 44-58`). `pole.py` and `hand.py` are reusable as-is: the pole waits out its modelled travel, about 2-7 s per move, because "the getter leads the real pole by up to 73 mm" (`lite/upload_channel/pole.py:3-58`).
- **V6 package and ALIX commode C sheet:** Default values exist but disagree. V6 starts both arms at 200 mm with GRIP `[0,0,0,0,150,1000]` and then switches between 50 and 150 mm per program; the region mapping is inferred, and the left and right hinges differ (150 vs 50 mm) (`lite/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/LEFT_V6_70SERIES_FINAL.json`). The commode C sheet uses 140 or 200 mm with the four fingers 100 % or 50 % open and the thumb left as it is (`alix/experiments/exp01_teach_trajectory_reverse_engineering/data/teach_files/commode_c_taught_paths/description.txt:3-20`). Defaults are therefore proposals.
- **ALIX taught tracks:** ALIX records pole and hand with every recording (`pole_m`, `hand_state`, `hand_counts` in `SOURCE.json`), sets "pole, then hand" before the arm moves, and notes that "a joint-space track is pole-agnostic, so a quarter-height pole sweeps the whole … path 225 mm low" (`alix/experiments/exp01_teach_trajectory_reverse_engineering/data/teach_files/MANIFEST.yaml:136-139`). It switched to "fingers %, thumb held" after commanded thumb channels did not move (`alix/tools/taught_tracks.py:343-361`).
- **ALIX orchestrator (bin):** The stowed-bin interlock `pole_bin_conflict` (153-282 mm, measured on BinShort) checks only a target height, but the arm base rides the pole, so the whole travel must be checked (`alix/ros/orchestration/alix_orchestrator/alix_orchestrator/core/task_config.py:58-124`). The control app has no bin code, so Lite can neither read nor set the bin, and the band is provisional because "the PHYSICAL part has not been remade" (`alix/ros/description/ugv_description/urdf/bin.xacro:32-34`).
- **ALIX description (bin states):** The bin is a manual mechanism with no hardware feedback ("no ros2_control hardware; bin_state_publisher is the sole joint-state source"). Its states are defined on two joints: STOWED (rail 0, drop 0), EXTENDED (rail 0.200 m, drop 0) and LOWERED (rail 0.200 m, drop 0.450 m), with transitions STOWED ↔ EXTENDED ↔ LOWERED (`alix/ros/description/ugv_description/config/peripheral_joints.yaml:33-52`). Lite can therefore take positions and geometry from the model, but the position in use must be declared. The R10 model measurement puts the stowed-bin conflict at poles 0.070-0.275 m, which disagrees with all three documented bands (R10 Findings), so Lite computes the band from the model rather than copying a constant.

**Can it be borrowed from ALIX Studio?** **Build new** — What R7 asks for, per-region pole and hand settings applied to the robot before drag teaching, does not exist in Studio. Its region model has no such fields, its per-task pole field and pad-derived hand state live in the excluded generator and are executed by ROS at run time, and it has no channel to the robot. Only small pieces carry over: the stowed-bin rule and wording, the 0-315 mm range, the Workbench pole UI pattern (named heights, custom height, band warning) and the 3-D robot model for an optional preview.
- *Other sources:*
  - Runner v14 (`--manual-pole`, `--manual-hand-raw`, `--manual-hand`, `--workflow-sequence-file`) with `pole.py` and `hand.py` — Borrow + adapt: use the commands and device classes unchanged; add per-region values, Lite's own arm-still check, the checklist, read-back tolerances, the travel-band check and logging. Subprocess first, library later.
  - `usage.py` — Borrow as-is (logic and reads): `arm_at_rest`, the `prestroke` order, `stop_all`, `pole read`, `hand read`.
  - ALIX taught lane (`alix/tools/taught_pipeline.py`, `alix/tools/taught_tracks.py`, `SOURCE.json`) — Borrow + adapt: pole and hand per recording, the sidecar keys, the pole-then-hand rule and the "fingers %, thumb held" hand spec.
  - ALIX orchestrator `pole_bin_conflict` — Borrow + adapt: the refusal sentence and the rule, extended from a target check to a travel check, with the band computed from the URDF model (shared with R10) instead of the constant.
  - ALIX `peripheral_joints.yaml` — Borrow as-is: the bin joint values for STOWED, EXTENDED and LOWERED and the allowed transitions.
  - `alix_dispatch/peripherals.py` and `hand_units.py` — Borrow + adapt (library target): library-grade pole and hand classes, and the hardware hand scale for a 3-D preview.
  - V6 workflows and the ALIX commode C teach sheet — Reference: seed defaults, marked "proposed".
  - ALIX emulator — Borrow as-is, for library-level tests only: it runs in-process, so its state does not carry across runner subprocesses.
- *Note on the inputs:* Mostly inaccurate. "borrow from alix studio" names the right concept in the wrong place. Studio carries a per-task pole height and a pad-derived hand state, but only inside the excluded generator: the value goes into generated configs that the ROS orchestrator runs, Studio never applies it before a teach, and the user never chooses the hand. The working apply machinery is in the control app, `usage.py` and the ALIX taught lane.

**Recommendation.** Store pole and hand per region in the Lite catalogue, and apply them in a **Prepare robot** step of the R1 teach card using the control app's pole and hand commands (subprocesses for the MVP, `usage.py` for reads), in the region's order. Guard the step with Lite's own arm-still check, a short checklist and a stowed-bin check over the whole travel (bin position declared once and shared with R10, band computed from the URDF), and never prepare an arm that holds the fixture or a tool part-way through a task. Show target against measured values and write the measured values into every recording's sidecar, where they are authoritative downstream. Seed defaults from V6 with the ALIX commode C sheet beside them, prefer thumb-held presets, have the supervisor confirm them, and later move the sequence into the Lite hardware library (one connection per arm, a stop token).

**Approach**
1. **Catalogue fields (build, shared with R1).** `pole_mm`, `hand: {preset, counts[6]}` with `null` = hold, `prep_order: pole_then_hand | open_pole_hand`, `defaults_source`, `confirmed_by`, and hand events in `substeps`. The supervisor editor has range checks and "copy to the same family/phase", and follows the Studio Workbench pole pattern (borrowed): named heights, custom mm, band warning.
2. **Seed defaults (reference data).** Map each region to its V6 program and take that program's pole and GRIP; list the ALIX commode C value beside it; mark regions with no source TBD (brush, glove, floor-left, drain); propose thumb-held presets where V6's fist is not needed. The requester or the supervisor confirms.
3. **Preset library (borrow + adapt).** Plain names with a small hand picture each: Fingers open, thumb held `[1000 ×4, hold, hold]`; Fingers half open, thumb held `[500 ×4, hold, hold]`; Grip (fist, V6) `[0, 0, 0, 0, 150, 1000]`; Fully open `[1000 ×6]`; optionally ALIX `open_tenth` `[129, 129, 129, 129, 133, 944]`.
4. **Session settings (build, shared with R1 and R10).** The bin setting is the same one as the R10 bin toggle: fitted yes/no and position STOWED, EXTENDED or LOWERED (mapped to the joint values in `peripheral_joints.yaml`), asked with a photo per state. Changing it in either place changes it for both. At start-up Lite computes the stowed-bin pole band from the URDF. The bin setting goes into every sidecar.
5. **Prepare step per arm inside the R1 card (build around borrowed commands).** Preconditions (connected, REAL mode, not in drag mode, no error codes, no program running, no other client) → checklist → `arm_at_rest` → bin check (refuse when STOWED and [min(current, target), max(current, target)] overlaps the model-computed band, or the current height is inside it) → per `prep_order`, optional `--manual-hand 100`, then `--manual-pole <mm>`, then `--manual-hand-raw` or `--manual-hand <pct>` → read-back parsed from the `final=` values of the `MANUAL POLE COMPLETE`, `MANUAL HAND COMPLETE` and `MANUAL HAND RAW COMPLETE` lines → "Ready ✓". Option: run the same as one `realman_workflow_sequence_v1` file with `--workflow-sequence-file` (one process, one SIGINT to stop).
6. **Reads, failure and stop (borrow `usage.py`).** Show the current pole and hand on card entry with `usage.py pole read` / `hand read` (no `--sdk` flag, so the SDK must be on `PYTHONPATH`). On failure, show a plain message with Retry and Stop; Stop is pole stop (`rm_set_lift_speed(0)`) plus hand freeze. Show the IO E-stop state of the arm on the card; Prepare does not start while it is active, and a Prepare interrupted by the E-stop ends as "not ready" and must be run again after release.
7. **Hand-event step (build).** For tool and glove sub-steps: drag mode off, arm still, the operator confirms the grasp point, hand command, read-back. The event is stored in the catalogue sequence and in the next sub-step's sidecar.
8. **Record and fine-tune (build).** Write targets, measured values, the preset name and any override reason into the R1 sidecar at save time. The card has ±10 mm buttons and a preset menu; "Save as region default" needs supervisor confirmation and flags recordings taught at other values.
9. **Later and optional.** Move the sequence into the Lite hardware library (one connection per arm, a stop token), reusing `pole.py` / `hand.py` or `alix_dispatch.peripherals`. Optional 3-D preview of pole height and hand pose with the Studio robot model and the hardware hand scale in `alix/ros/hw/alix_hw/scripts/hand_units.py`, not the emulator's view scale.
10. **Tests.** Unit tests for the interlocks (travel band from the model, moving arm, drag mode, ranges, holding-fixture lock, active E-stop); Prepare against the fake device (subprocess path) and against the ALIX emulator in a single-process harness (library path); a hardware checklist per arm: 50 → 150 → 200 mm with the bin deployed (not STOWED), GRIP ↔ thumb-held open, Stop during a pole move, the IO E-stop pressed during a pole move and during a hand move (does the lift stop, and does Prepare recover cleanly?), read-back values and a gloved-thumb test.

**Acceptance criteria**
- [ ] **Ranges and defaults.** Every region record has `pole_mm` in 0-315 and a hand preset of six values in 0-1000 or "hold"; the supervisor editor refuses out-of-range values. Seeded defaults match V6 for covered regions (for example L-R01 50 mm GRIP, R-R04 150 mm GRIP) and are marked "proposed".
- [ ] **Prepare.** "Prepare robot" moves the pole before the hand (the log shows the order) and shows target and measured values. START stays disabled until the pole is within ±5 mm and every commanded hand channel is within ±15 counts or settled. An override is possible only in supervisor mode, and its reason is stored in the sidecar.
- [ ] **Interlocks.** A pole command is refused while the arm moves faster than 1.6°/s. Pole and hand commands are refused while the arm is in drag mode or marked as holding the fixture or a tool between sub-steps. The checklist must be ticked before the first move for each region.
- [ ] **Bin.** The band used by the interlock equals the one computed from the URDF at start-up and is shown in supervisor settings. With the bin STOWED, a target inside the band is refused with the foul-band explanation, and so are a move that crosses the band from below to above, and any move while the current height is inside the band. With EXTENDED, LOWERED or not fitted, all three proceed. Changing the bin setting in R7 changes the R10 toggle and the reverse.
- [ ] **Provenance.** Each recording's sidecar holds the measured `pole_m`, the measured `hand_counts` and the preset name, equal to the read-back shown before START. A fine-tuned value is stored in that recording's sidecar; the region default changes only after supervisor confirmation, and recordings taught at the old value are then flagged "taught at other pole/hand".
- [ ] **Failure and stop.** Against the fake device (subprocess path) and the ALIX emulator (library path) the Prepare step completes. A forced timeout shows a plain message with Retry and Stop, and Stop halts the pole and freezes the hand. With the fake device reporting an active IO E-stop, Prepare is refused with a message naming the arm.
- [ ] **Tools.** For a brush region with `prep_order: pole_then_hand`, the hand never opens before the pole move.
- [ ] **Hand events.** In a tool sub-step sequence, the hand event runs only with drag mode off and the arm still, after operator confirmation, and appears in the next sub-step's sidecar.

**For operators:**
- One **Prepare robot** button does everything in order, with a progress line: "Moving pole… Moving hand… Ready ✓".
- The pole shows as "Pole height: 150 mm" with a gauge, and the hand as a named picture ("Fist", "Open hand, thumb still", "Half open"); channel values appear only on the supervisor screen. A small "changed" note shows when values differ from the region default, and operators can fine-tune only for the current recording.
- The bin question comes once per session, with a photo of each position. Refusals are sentences with the fix, using the computed band: "The bin is stowed, so the pole cannot pass <low>-<high> mm. Deploy the bin, or keep the pole below <low> mm."
- If the red E-stop is pressed, the card shows "E-stop pressed" and Prepare must be run again after the button is released.

**Safety:**
- The pole moves the whole arm. Never move it with the pad on a surface, with the arm moving, in drag mode, or while the hand holds the lid, the seat or a tool part-way through a task (`lite/upload_channel/usage.py:24-25`). Neither runner pole path checks the arm and v53's APPLY has no confirmation, so Lite adds both.
- The stowed-bin band is computed from the URDF, which uses BinShort; the physical bin has not been remade and is about 40 mm taller (`alix/ros/description/ugv_description/urdf/bin.xacro:28-34`), so confirm which bin is installed before relying on the band. The documented bands (the orchestrator measures 158-277 mm and refuses 153-282 mm with a 5 mm margin; Studio uses 153-282 mm; bin.xacro states 160-285 mm) and the R10 model measurement (70-275 mm) disagree (`alix/ros/orchestration/alix_orchestrator/alix_orchestrator/core/task_config.py:85-94`; R10 Findings), so the computed band must be checked against the hardware before the interlock is relied on. Lite keeps the orchestrator's 5 mm margin around the computed band. STOWED is the ALIX start-up default. V6 starts at 200 mm, inside every band, so V6 runs assumed the bin was not stowed. A wrong bin declaration defeats the interlock.
- Closing fingers can pinch, so nobody touches the robot hand while it moves. With a tool in the hand, do not open it before the pole move. With a glove, commanded thumb channels may not move, so prefer thumb-held presets.
- Controller SIM mode does not move the lift or hand, so a SIM run proves nothing for R7. Prepare has a one-click Stop (pole stop and hand freeze), which is a convenience. The primary safety stop is the IO E-stop: buttons wired to an input on each arm controller (left and right) that stop the robot without cutting power (F4). Lite shows its state and guides recovery. In the ALIX field test the controllers refused lift commands during an E-stop but the hand kept working (`alix/docs/TEST_PROCEDURE.md:216-242, 453`), so Lite sends pole stop and hand freeze itself on every E-stop (NFR-SAFETY (c)); whether the E-stop halts a pole that is already moving is confirmed in the hardware checklist (E-stop drill DR6).

| Priority | Size | Depends on |
|---|---|---|
| Must | M (tool and glove hand events and the optional 3-D preview add to it) | R1, INT-CTRL; bin setting and model band shared with R10 |

**Open questions:**
- Which hand preset per region: V6 GRIP (fist) or ALIX "100 % / 50 % open, thumb held"? How is the glove or cloth worn during teaching?
- Which pole defaults: V6 (50/150/200 mm) or the ALIX commode C sheet (140/200 mm)?
- Which bin position is normally used during teaching and production (STOWED, EXTENDED, LOWERED or not fitted), so it can be the session default? Is the installed bin BinShort, as in the URDF, or the original, about 40 mm taller?
- Does the IO E-stop stop a pole (lift) that is already moving (drill DR6), and does the hand behave as in the ALIX field test, where it kept working (drill DR7)? How does each recover after release?
- Prep order: always "pole then hand", or "open fingers → pole → hand" where no tool is held?
- Are the read-back tolerances of ±5 mm and ±15 counts acceptable?
- Before a pole move, is the operator's confirmation enough, or should the arm first go to a stored safe pose? The runner supports a controller global point (`--global-point`), but that is itself a motion near the operator.
- May operators fine-tune at the card, and who approves a new default?
- Do the mechanism tasks need a hand change part-way, or is the grip held throughout, as in V6?

---

### R8 — TXT file merging

**Original (inputs):**
> 8. TXT File Merging: The software should provide an option to merge multiple trajectory TXT files into a single executable trajectory. - There exists a mechanism, provided takes into account the collision scene (commode) in alix workspace in the upload channel tests and experiments

**Revised requirement.** Lite shall turn an operator's ordered list of taught region `.txt` files for one arm (normally R2 outputs) into the fewest controller programs that are safe to run, one `.txt` per program, and shall build the per-arm workflow that runs them in order, with the pole, hand and checkpoint steps between them. One file for a whole arm's day is not possible: a stored program carries only the 7 arm joints, and a V6 arm day is 41,000–45,000 points, about 3.5 times the largest program known to have been uploaded.

- (a) **Grouping.** Two regions share a program only if they are for the same arm, need the same pole height, the same 6 hand values and the same toilet state (lid closed, lid open or seat open; a region marked "any" takes the program's state), and both are TOGETHER cleaning units. Mechanism, tool, glove and serial (RUN FIRST / WAIT OTHER) units always get their own program. At each split Lite inserts the pole, hand or checkpoint step and says why in plain words.
- (b) **Joins and program ends.** A gap under 0.001° drops the duplicate point. Contiguous rows of one recording join directly. Any other gap gets a rest-to-rest minimum-jerk link move on the straight joint line, sized on the written integers at the 5.0 ms tick: peak joint speed ≤ the link-move speed (default 30 °/s) and ≤ 0.97 × the joint rating, nominal peak acceleration ≤ 300 °/s². Every program starts and ends at rest (≤ 5 °/s over its first and last 10 ticks, proposed); otherwise Lite adds a short ease-in or ease-out, or flags the region end for trimming. A blocked link move or transit gets a detour through a checked safe pose. "Slower" is never offered as the fix for a collision.
- (c) **File gates**, judged on the written integers and shared with R12: format (7 integer millidegrees per line, ≥ 2 lines); step ≤ 10° per line; rating by ALIX's G2 (no tick over a joint rating at 5.0 ms on the 20 Hz band-limited rate; raw tick counts shown for information); joint limits (inside 5° of a URDF limit the band-limited rate stays ≤ 10 °/s; a margin under 2° is a hard block); size ≤ a configurable point cap; stored name `"<id>_" + [DT_] + stem + ".txt"` ≤ 32 characters; predicted run time < the runner's 300 s timeout. Only G2 and the 2–5° near-limit rule may be overridden, by an engineer, with the reason recorded in the report.
- (d) **Collision check**, run as a background job in one worker process. The scene comes from the URDF and ALIX's `commode_c` scene (`alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml`): the commode body, lid and seat in the program's toilet state at the scene's hinge angles (closed 0°, open −110°), the bin in its declared state, the UGV, poles and robot, and the partner arm at its known pose. The venue walls are always enforced: they come from the ALIX venue file selected for the test cell (`alix/ros/orchestration/alix_tasks/config/scene/washroom_*.yaml`), and a program with no venue selected is NOT CHECKED. Lite checks every link-move point, every controller transit (straight joint line at the new pole height, ≤ 2° per sample), every pole or hand change with the arm still, and every cleaning row. Untaught motion gets a strict check with no contact allowed, and its minimum clearance to toilet and walls is reported (under 20 mm, proposed: NEEDS ATTENTION). On cleaning rows, the active hand on the region's own surface is expected contact (deeper than 20 mm: NEEDS ATTENTION), and any arm-link contact is a collision. With the bin stowed, the 153–282 mm pole band is enforced as a rule. If the raw recording of a selected region shows arm-link contact, the scene is wrong and every verdict in it is NOT CHECKED. A collision makes the program BLOCKED; a check that did not run, or an incomplete scene, gives NOT CHECKED, never READY.
- (e) **Dual-arm clearance (Must for TOGETHER programs).** A TOGETHER program is not READY until an arm-vs-arm check of its whole block, including both arms' start transits and pole moves, shows ≥ 25 mm over the predicted timing spread. The check is shared with R10. If it fails, Lite proposes SERIAL.
- (f) **Outputs, staleness and preview.** Lite writes the merged `.txt` (LF, integer millidegrees); a provenance report (inputs with sha256 and row ranges, link moves, gate values, scene, verdict, versions); per-arm `realman_workflow_sequence_v1` JSON with the Program IDs the operator assigned (any ID 1–100, see (h)); and a slowed test file for each new link move. Any change to an input, its metadata, the scene (URDF, `commode_c` scene file, selected venue file) or the engine version marks the program out of date and blocks upload. The result plays in the Lite simulation view with link moves and transits marked, and a reported collision jumps the view to its sample.
- (g) **Should:** Lite matches the timing of TOGETHER blocks automatically, by holds or stretch, using the runner's own pole and hand step times and measured start and transit times, and shows the predicted skew band. Stretching uses the two R2 optimisers (the prototype's trim → smooth → scale by default, or time-optimal TOPP-RA) with the matched duration as the target time: where an arm cannot reach that time within its limits, Lite uses the closest time that obeys the limits, names the limit that binds, and pads the other arm with holds.
- (h) **Program IDs.** As in the control program, merged programs may be uploaded to any Program ID 1–100 on either arm. An occupied slot is overwritten only after a confirmation that shows the program currently in it. Lite does not reserve or refuse ID ranges; it warns when a slot holds a program from another workflow (for example the V6 70-series) or sits in a range ALIX uses (taught slots 40–59, reserved 70–100), and suggests a free slot.

**Findings**
- **ALIX Studio:** Nothing in the Studio merges files, generates link moves, allocates Program IDs or reads `.txt` tracks. It does hold the world needed to check a merge: `BodyCollisionWorld` poses the robot, bin, UGV, poles, walls and the commode body, lid and seat in three states, at about 2–3 ms per pose. It is validated only for `commode_c` (`studio/body_collision.py:1-40, 58`).
- **ALIX Studio (what must change):** That world cannot judge untaught motion as it stands. It has no strict mode: with no contact stage it allows hand contact with every commode part, so a link move that drags the glove through the seat would pass. It also takes the hand only as a named state, always poses the bin lowered, and silently drops any pair already touching at rest (`studio/body_collision.py:595-596, 629, 871`). It never tests the arm base against the stowed bin covers (both count as static), so the measured 153–282 mm pole band must be a rule (`studio/studio_generate.py:472-475`).
- **ALIX tools and experiments (joins):** `plan_min_time.concatenate` and the EXP01 `plan_to_track.concatenate` join stages of one MoveIt plan, refuse any gap over 0.5°, and are tested for continuity, not collisions. Their commode safety came from MoveIt planning the stages against the articulated scene (`alix/tools/plan_min_time.py:142-169`). The C++ planner supplies the split rule: "a hand or pole stage ends the group" (`alix/ros/orchestration/alix_tasks/include/cleaning_tasks/common/stroke/track_emit.hpp:145-148`).
- **V6 package:** Merged offline by a tool that is not on disk. Its README says the bridges are "NOT collision-checked against the toilet, cubicle walls, second robot, end-effector, or right-arm camera" (`lite/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/README.txt:48-56`). All 11 bridges reproduce at 0 mdeg as rest-to-rest minimum-jerk moves, so they make good golden test data. Between programs V6 leaves unchecked controller transits of up to 64.6° (J5), and its programs start at up to ~55 °/s and stop from up to ~67 °/s.
- **V6 package (gates):** On ALIX's G2 (20 Hz band-limited rate at 5.0 ms), 2 of 15 V6 files exceed a joint rating: R_P06P07 (102 ticks, peak 312 °/s on J6) and R_P10P12 (29 ticks). The raw per-tick count flags 10 of 15, because taught data ripples (`alix/tools/cleaning_tracks.py:1646-1672`). R10_HNGR has a 1.53° margin on J4, below ALIX's 2° floor, so it is a hard block (`alix/tools/taught_tracks.py:172-173`).
- **alix_dispatch / taught tracks:** The controller's player does not smooth ("the file's point SPACING is the velocity profile"), and the run speed scales only the transit to the first point (`alix/core/python/alix_dispatch/src/alix_dispatch/track.py:7-11, 21-24`). So program ends need ramps, and a slow first run needs a slowed copy of the file. ALIX itself runs each taught slot as its own program and labels its rest legs and bridges "NOT collision-checked" (`alix/tools/cleaning_tracks.py:148-158`).
- **Control app v53/v14:** The runner already validates and runs per-arm `realman_workflow_sequence_v1` workflows of program, pole, hand and wait steps, so R8 only has to emit them. Four constraints shape merged programs: a program must report "running" within 5.0 s, while ALIX's start delay grows by 0.41 ms per point (about 4.9–5.8 s for 10,983 points, extrapolated); error 4105 is warning-only; a run checks that a slot is not empty, not that it holds the checked file; TOGETHER arms are released together only at block start, with both start transits at once (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:56-57, 238, 1668-1763`; `alix/tools/dual_arm_verdicts.py:221-224`).
- **Dual-arm tools and shared controllers:** ALIX's `.txt` shim checks arm against arm only, needs explicit pole and hand seeds, and sweeps a timing window at 0.02 s against 25 mm, which takes minutes (`alix/tools/dual_arm_verdicts.py:356, 369`). Lite's controller IPs equal ALIX's defaults, and V6 used IDs 71–83 inside ALIX's reserved 70–100, so the controllers may be shared and overwriting a slot can remove another workflow's program (`alix/core/python/alix_dispatch/src/alix_dispatch/connect.py:31-32`). This is a reason for warnings and a replacement confirmation, not for blocking IDs.
- **Control app v53/v14 (Program IDs):** The control program accepts any slot 1–100 ("Choose controller program slot 1-100"), says "If the slot already contains a program, it will be replaced", shows existing programs before replacement and asks YES to replace, and supports batch upload with per-file Program IDs (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:9-11, 898, 936, 4279, 4472`; `lite/upload_channel/rm_upload_fast_v14_safe_transit.py:324`). Lite follows the same model.
- **Scene (URDF and `commode_c`):** The `commode_c` scene gives the fixture pose relative to `base_link` ([1.005, 0.0, 0.42] m at a 0.25 m UGV standoff), the lid and seat hinges (closed 0°, open −110°) and the meshes (`alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml:33-68`). Walls are the one thing not enforced today; ALIX loads them from a venue file with `plan_scene_validation.load_wall_manager` and `PlanWallChecker` (`alix/core/python/alix_workspace/src/alix_workspace/plan_scene_validation.py:117, 179`).

**Can it be borrowed from ALIX Studio?** **Partial** — The Studio has no merging, link-move generation, Program-ID allocation or `.txt` I/O, so the merge itself is new work. It does have what is needed to check a merge (`BodyCollisionWorld`, the "no world = not checked" convention, sentence-style sequence verdicts and the Dual-Arm Collisions playback), but the world must be vendored and changed (strict mode, hand counts, bin state, visible rest baseline) before it can judge untaught motion.
- *Other sources:*
  - V6 package — Reference and golden test data: take the method (holds and stretch, minimum-jerk link moves, split at pole moves) and compare its merged files row for row (they are CRLF); do not reuse its 1.38 s pole model, IDs 71–83 or unchecked link moves.
  - ALIX joins (`plan_min_time`, EXP01 `plan_to_track`) — Borrow + adapt: keep the join contract (drop a duplicate under 1 mdeg, record spans, holds as repeated points), port it to integer millidegrees, and hand gaps to the link-move generator instead of refusing them.
  - ALIX planner grouping (`track_emit`) — Borrow the rule: a pole or hand stage ends a program.
  - ALIX gates (`taught_tracks.py`, `cleaning_tracks.py`) — Borrow + adapt: port G2, the near-limit rule, the step check and `rate_fit_track` ("slow only where over the rating") to pure numpy.
  - Dual-arm tools (`dual_arm_verdicts.py`, alix_workspace `dual_arm_plan_collision.py`) — Borrow + adapt: the `.txt` shim with explicit seeds, the aligned sweep with commode and walls through `extra_managers`, and the windowed 25 mm check. Not usable: the Studio's Dual-Arm Collisions endpoint (no commode, walls, hands or pole) and alix_workspace `ClearanceProbe` (poses the open lid and seat wrongly).
  - Trajectory prototype — Borrow + adapt: `interpolateRegion` for stretch and `dataToTxt` as the LF writer; it has no merge feature and loads one file at a time.
  - `lite/upload_channel/pole.py` and `hand.py` — Borrow as-is: the runner's own pole and hand step times for the dual timeline.
  - Control app v53/v14 — Use as-is to run the emitted workflow; it has no merge function.
- *Note on the inputs:* Partly accurate. Join mechanisms with tests do exist in ALIX's upload-channel experiments and planner, and so does the commode collision scene. But no join checks collisions: the joins concatenate stages of one MoveIt plan, their commode safety came from MoveIt, they refuse the gaps that taught recordings have, and their tests check continuity only. The Lite workspace has no mechanism at all, and V6 was merged by a tool that is not on disk and is explicitly not collision-checked. The checked merge has to be assembled.

**Recommendation.** Build a Program Builder in Lite. It splits an operator's ordered regions wherever the pole, hand, toilet state or sync rule changes, joins the rest with sized minimum-jerk link moves (which reproduce the V6 bridges exactly), and makes every program start and end at rest. Each program must pass ALIX's hardware-proven gates and a strict collision check of every link move, transit, device step and cleaning row in an adapted `BodyCollisionWorld`; "not checked" is never READY, and TOGETHER programs also need the arm-vs-arm check. Automatic dual time matching follows as a Should.

**Approach**
1. **Segment model (build).** Parse each region with a port of `parse_track` (`alix/core/python/alix_dispatch/src/alix_dispatch/track.py:66-82`) into int32 (N, 7) rows. Attach the R1, R7 and R2 metadata, including R2's row map back to the raw recording.
2. **Planner (build; rule borrowed from `track_emit`).** Split on arm, pole, hand, toilet state, non-TOGETHER units, the point budget and manual breaks, with a reason sentence for each. Between programs put the pole step, then the hand step, then any checkpoint or wait. The operator assigns each program a Program ID 1–100 (Lite suggests a free slot from the controller's program list and warns on slots used by V6 or ALIX), and stems stay ≤ 21 characters.
3. **Link moves and ends (build; join contract ported from `plan_min_time.concatenate`).** Size each link move as `T = max(0.2 s, maxⱼ 1.875·gapⱼ/v_capⱼ, maxⱼ √(5.7735·gapⱼ/300))` with `v_capⱼ = min(link speed, 0.97·ratingⱼ)`, rounded up to 5 ms; re-measure on the integers and add ticks until the caps hold. Add "Ease in/out" ramps, the "Go via the safe pose" detour and, as an option, checked approach link moves that replace large controller transits (over 2° inside a TOGETHER block, over 10° elsewhere).
4. **Gates (borrow + adapt).** Port ALIX's G2, near-limit rule, step check and `rate_fit_track` fix to pure numpy, shared with R12. `scipy.signal` does not import under the system Python here, so carry the filter in numpy or pin a venv.
5. **Collision adapter (borrow + adapt).** Vendor `body_collision.py`, `scene_pose.py`, `fixture_parts.py`, their assets and `hand_units.py`. Add strict mode, hand joints from the 6 counts, bin state and a visible rest baseline. Load the walls from the selected venue file through `load_wall_manager` (enforced in every check). Then add the stowed-bin rule, the raw-recording plausibility check, the sampling (transits ≤ 2°, pole sweeps every 10 mm, every cleaning row), FCL clearance on untaught samples and Lite's own pair-name table. Run it in the shared R5 worker, cached by input sha256, metadata, scene and engine version.
6. **Dual check (Must; borrow + adapt `dual_arm_verdicts.py` and `dual_arm_plan_collision.py`).** Build each arm's block timeline from the program lengths at (lines − 1) × 5 ms, both start transits, pole and hand steps timed by `lite/upload_channel/pole.py` and `hand.py` (2.5–3.2 s per 100 mm, not V6's 1.38 s), the runner's settle and the start delay (ALIX's band until measured on Lite). Run the aligned sweep with commode and walls, then the windowed 25 mm check; propose SERIAL on failure. Time matching by holds or by stretch through the R2 optimisers (target time, closest feasible time, binding limit), and `DUAL_TIMELINE.json`, are the Should part.
7. **Outputs (build).** Write `<stem>.txt` (LF; the runner makes the CRLF `.proj`), `<stem>.merge.json` (`alix_lite_merge/1`), the per-arm workflow JSON, and a first-run kit: a slowed copy of each new link move (same path, ≤ 1/3 speed) plus a checklist. Watch the inputs for staleness.
8. **Program Builder UI (build, Studio theme).** The operator drags regions into order, presses "Check & build" and reads traffic-light verdicts with fixes as sentences; Lite then hands off to the R3 preview and R13 upload.
9. **Tests.** V6 golden row for row, split rules, link-move sizing, ends, G2 on V6, strict-mode and stowed-bin regressions, synthetic collisions, NOT CHECKED paths, staleness, performance and offline workflow validation; then a controller SIM-mode run before any REAL run.

**Acceptance criteria**
- [ ] **V6 golden.** Re-merging the V6 segments in "V6 mode" (rest-to-rest, given T) reproduces every parsed row of `L_P01P02`, `L_P03P05`, `L_P06P07`, `L_P10P12`, `R_P01P03`, `R_P06P07` and `R_P10P12`.
- [ ] **Splits, link moves, ends.** Poles 150 → 150 → 50 mm give two programs, a pole step and the sentence "Pole height changes 150 → 50 mm — new program"; hand, toilet-state, mechanism and serial changes always split, and toilet-state splits get a checkpoint step. At 30 °/s a 39.08° gap gives T ≥ 2.445 s, and every link move, measured on the written integers, keeps peak speed ≤ the selected speed and ≤ 0.97 × rating and nominal acceleration ≤ 300 °/s², with no duplicate rows. The R_P10P12 start and the M3_ASOPN01 end are flagged and, after "Ease in/out", start and end at rest and pass every gate.
- [ ] **Gates on V6.** Exactly R_P06P07 (102 band-limited ticks, first at tick 2851, peak 312 °/s on J6) and R_P10P12 (29 ticks) fail G2, with raw counts shown as information. Neither is READY without a recorded engineer override, and "slow only where needed" clears the ticks without changing the path. R10_HNGR is BLOCKED (J4 margin 1.53°) with no override offered.
- [ ] **Untaught motion.** A synthetic link move that drags the glove through the seat is BLOCKED with sample, pair and depth, and the test shows that the unmodified `make_pose_filter` would have allowed it. Glove-on-lid contact during lid-top rows is expected contact with depth; the same contact in the middle of a link move or transit is a collision. A pole step whose 10 mm sweep puts the hand into the toilet, or an end pose with hand–toilet contact before a device step, is an error.
- [ ] **Scene and staleness.** With the bin STOWED, any pole height in or travel across 153–282 mm is BLOCKED (test case: V6's 200 mm first step). Walls are enforced: a link move pushed into a wall of the selected venue is BLOCKED. With the default 800 mm venue, the raw R2_LIDTOP recording shows `R_Link5↔washroom_structure`, so the scene is NOT CHECKED until the venue that matches the test cell is selected and known-good recordings pass against it; a non-empty rest baseline is NEEDS ATTENTION. With the toilet state, venue or bin state unset, or after any change to an input, its metadata, the URDF, scene or venue file or the engine version, upload stays disabled until a rebuild and re-check.
- [ ] **Execution.** The workflow passes `normalize_workflow_step` offline, loads in UI v53 Page-02 and runs end to end in controller SIM mode with runner v14. The largest merged program on each controller prints its start-ack in under 4 s, or the point cap is lowered to the largest size that passed. Stored names are ≤ 32 characters for any ID 1–100. Any ID 1–100 can be assigned; uploading to an occupied slot requires a confirmation that shows the existing program, and a slot used by V6 or ALIX shows a warning but is not blocked. Every new link move has a slowed test file that the checklist requires before the full program.
- [ ] **Dual (Must for TOGETHER).** Each block reports per-arm durations (lines × 5 ms), both start transits, the predicted skew band and the windowed minimum arm-to-arm clearance. Below 25 mm it is proposed SERIAL, and TOGETHER programs are never READY without this check.
- [ ] **Time matching (Should).** With a target block time, each arm reaches it within one 5 ms tick, or Lite reports the closest feasible time and the binding limit and pads the faster arm with holds; the result still passes every gate in (c).
- [ ] **Performance.** A single-arm check of a 10,000-point program with 3 link moves, every row checked, finishes in ≤ 60 s on the operator PC with progress shown (and is re-measured on the Jetson production build).

**For operators:**
- One "Build program" screen: drag taught regions into order; Lite draws the program boxes with plain separators such as "Pole moves 150 → 50 mm" or "Open the lid now (manual check)".
- One "Check & build" button. Results read Ready, Needs attention, Blocked or Not checked, and problems come as sentences with fixes: Show me, Go via the safe pose, Back to editing, Re-teach this join. "Slower" is never offered for a hit.
- Link-move speed is Gentle (20 °/s) or Normal (30 °/s, default); Brisk (45 °/s) is engineer-only. Run times show as ranges. Each program shows its Program ID (1–100, editable, with a suggested free slot); replacing an occupied slot asks for confirmation and names the program being replaced. sha values stay in the engineer panel.

**Safety:**
- Link moves, transits and pole or hand moves were never taught. Check them strictly, never with the unmodified Studio filter, and treat "not checked" as not ready.
- Play the slowed test files before the first real run: the speed setting does not slow a stored program.
- Enforce G2 and the near-limit rule before upload. Runner v14 treats error 4105 as a warning, and ALIX saw 4105 end a program at 58 % (`alix/tools/cleaning_tracks.py:211-219`), so a cut-short program can read as complete and the next pole or hand step starts from the wrong pose.
- Never move the pole or hand with the glove on a surface. Overwrite a slot only after the confirmation that shows its current program, and read back each slot's name and size before overwrite and before run (R13), because the controllers may be shared with ALIX and V6 workflows.
- The IO emergency-stop buttons on each arm controller remain the primary stop during first runs of merged programs.

| Priority | Size | Depends on |
|---|---|---|
| Must (single-arm core and the dual check of TOGETHER programs); Should for automatic dual time matching | XL (Must: L) | R1, R2, R3, R5, R7, R10, R12, R13, INT-CTRL; URDF + `commode_c` scene + selected venue walls |

**Open questions:**
- Which known-good recordings verify the 1400 mm cubicle (or the 800 mm one where that is the cell)? Does the cell use the scene's default 0.25 m UGV standoff?
- Is the bin present during cleaning, and in which state?
- Which slots should trigger an overwrite warning (V6 71–83, ALIX taught slots 40–59, ALIX reserved 70–100), given that the controllers may be shared with ALIX?
- Should G2 and 2–5° near-limit failures block READY with an engineer override (as proposed), or only warn?
- Default link move: rest-to-rest minimum-jerk at 30 °/s (V6-compatible), or velocity-matched?
- Will regions be taught as a chain (each starts where the previous ended, as in V6) or each from one safe start pose (as in ALIX)? Where is the safe pose for detours?
- Dual blocks: keep V6's open-loop overlap of one arm's link move with the other arm's pole move, or re-sync at each pole move with a new sync step? Match time by hold (keeps wiping speed) or by stretch through the R2 optimisers (slower wiping), and which optimiser is the default for stretch?
- What start-ack times did the 10,000-plus-point V6 programs print under runner v14, and what is the largest program actually run on each controller?
- Should V6 programs be imported as legacy regions, split at the known bridge rows, so they can be re-merged and checked?

---

### R9 — Offset adjustment using IK

**Original (inputs):**
> Offset Adjustment Using IK: The software should allow users to apply position offsets using Forward Kinematics (FK) apply offset to each point then inverse kinematics (IK) making sure the path is traceable no adjuscent point jumps, recalculate the joint positions, and test the adjusted trajectory. - obeyin the joint limits

**Revised requirement.** Lite shall let the operator shift the taught pad path of a program, a sub-region or a whole flow by a small rigid offset, and recompute the joints with an IK that follows the taught motion. The result shall be a new file version with no jumps, no loss of joint-limit margin and an honest report of where the full offset was not possible, and it shall pass R14 before any real run or upload.

- (a) **Scope and frames.** Scope: one program (`.txt`), a sub-region (a row range chosen on the joint plot), or every program of both arms as one "toilet moved" offset. Frames: robot (`base_link`: towards/away ±X, left/right ±Y, up/down ±Z); pad (press harder/lighter along the pad's contact axis, or slide sideways); toilet (a shift plus a small turn about the vertical axis, about the `commode_c` fixture frame placed by the scene, `alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml:33-44`). The toilet-frame offset is for when the commode on site is not where the scene says; the scene itself needs no calibration project.
- (b) **Size (proposed, for the requester to confirm).** The limit applies to the largest pad displacement the request causes anywhere on the path, translation and turn together: resolution 0.5 mm, warn above 10 mm, refuse above 30 mm and advise re-teaching. Press-in along the pad axis ≤ 5 mm. Toilet turn ≤ ±3°.
- (c) **Continuity and accuracy.** Per point, Lite computes FK at the region's pad frame (pole from the workflow), applies the offset and solves the IK, keeping the point count and timing and writing integer millidegrees. Each joint's step between adjacent points is ≤ max(1.5 × the taught step, 0.5°) and never > 10°; no joint wraps by 360°; sign(q4) matches the taught sign wherever |q4 taught| ≥ 2°. Where the full offset is applied, the pad is within 1 mm and 0.5° of the requested pose.
- (d) **Joint limits.** The solver keeps every joint ≥ 5° inside the URDF limits (stricter than the controller drive limits), or no closer than the taught track already went. The output passes the same joint-margin gate as R2 (ALIX G1: predicted executed margin ≥ 0.5°, with overshoot = max(0.85°, 0.139 s × arrival rate); fallback without the model: commanded margin ≥ 5°, or 2-5° only where the 20 Hz band-limited rate ≤ 10°/s). No joint's predicted executed margin may be smaller than in the input.
- (e) **Honest reduction and smooth joins.** Where the full offset is impossible (straight elbow, reach, joint limit), it is reduced smoothly. Each reduced span is reported with its time, mm achieved, reason and effect along the pad's press axis: "may press harder by X mm", "may miss by X mm" or "sideways only". For a toilet-moved offset, a span more than 2 mm deeper than taught blocks Apply. Ramps and tapers are raised-cosine or Hann with no hard weight floor; sub-region ramps last ≥ max(10 % of the span, 60 points); the extra pad speed they cause is ≤ 25 mm/s (proposed; a 10 mm offset needs ≥ 0.63 s of ramp).
- (f) **Pole option.** A vertical-only robot-frame offset of a whole program, of 10 mm or more, may be done by changing that program's pole height (exact geometry, full coverage). The new pole is within 0-315 mm and, with the bin stowed (bin state known), outside 153-282 mm. Lite inserts its own pole step before the program and restores the previous height after it, tells the operator the lift lands within ±5 mm, and re-checks dual-arm timing.
- (g) **Versions.** The result is a new file version with a provenance record. The original is never overwritten, the change is one labelled undo step, and the region shows "needs simulation check" until R14 passes.

**Findings**
- **ALIX Studio:** There is no offset feature for taught joint tracks. `do_retarget` only re-expresses a Cartesian task config and refuses any change of motion; `do_animate_sequence` and `/api/path-ik` are X1/X2 previews with no jump check (`studio/server.py:2964-3128`). The Studio's "ease-in rule" (`PSI_OFFSET_EASE_*`, 10 % of the path, ≥ 0.05 m) eases in a swivel-angle offset for C-space authoring (X2); it is not a path-offset ramp (`studio/stroke_gate.py:296-298`).
- **ALIX Studio (reusable parts):** `stroke_gate`'s continuity test (a step is legal when |J·dq − pose error| ≤ 1 mm / 0.5°; failures named jump, limit or unreachable), the vendor singularity law (near 10°, gate 15°, floor 5°; q4 = 0 is the case that matters here) and the pad contact axes (`glove_frame_2` presses along +Z; 20 mm glove band) carry over (`studio/singularity.py:36, 66-67`; `studio/config/glove_frames.yaml:99-100`).
- **alix_workspace:** `UrdfModel.fk/fk_batch` reproduce the controller's TCP to 0.007 mm median, and the Jacobian is public with the pole lockable. `IKSolver.solve` restarts from random seeds and hops branches, so it must not drive track following (`alix/core/python/alix_workspace/src/alix_workspace/urdf_kinematics.py:218-259`; `alix/core/python/alix_workspace/src/alix_workspace/ik.py:282-291`).
- **alix_dispatch / taught tracks:** A flat 5° margin does not cover servo overshoot: J4 executed 5.6-5.9° past its command and reached 135.055 / 135.366° with no fault. ALIX's G1 judges the predicted executed margin, and G2 judges 20 Hz band-limited rates at the 5.0 ms clock (`alix/plan/FINDINGS.md:6721-6760`; `alix/tools/taught_tracks.py:466-487`).
- **V6 package (re-measured):** Straight-elbow passages are common. R2_LIDTOP crosses q4 = 0 four times and has 23.6 % of its points within 10° of it; L_P01P02 and L_P03P05 cross it twice. By G2 only 2 of the 15 V6 files exceed a joint rating (R_P06P07 1.38×, R_P10P12 1.08×); raw single-step rates overstate this (10 of 15) (`lite/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/`).
- **Solver trial (re-measured):** An independent taught-following continuation IK gave 0 jumps and ≤ 0.54 mm residual on four real cases (R2_LIDTOP +10 mm Z, L_P01P02 +10 mm Y, R10_HNGR +10 and +30 mm X). Without an elbow taper the arm flips branch (J4 176° from taught). Full-offset coverage is limited: R2_LIDTOP about 55-66 % at 10 mm, R10_HNGR 46 % at 30 mm. A 0.05 weight floor or a box-shaped taper causes one-tick jumps (105 mm/s per 10 mm of offset; 1,000 mm/s target steps).
- **Control app v53/v14 and the pole:** Neither program offers offsets, and joint programs ignore controller work and tool frames, so Lite must compute the offset. The pole is a pure +Z slide, so a whole-program vertical offset equals a pole change. But `pole.py` lands within ±5 mm and skips moves of 5 mm or less, and one V6 pole step serves several programs (LEFT step 7, 150 mm, covers four) (`lite/upload_channel/pole.py:3, 36-37`; `lite/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/LEFT_V6_70SERIES_FINAL.json`).
- **Trajectory prototype:** It has no kinematics. Its plot selection gives a row range usable as the offset scope, but `addRegion` refuses unless the Scale stage is on (`lite/Alix_Studio_Lite_Trajectory_V2.html:1392-1397`).

**Can it be borrowed from ALIX Studio?** **Partial** — ALIX Studio has no offset tool for taught joint tracks and no IK that follows a taught motion. Its FK and Jacobian (through `alix_workspace`), the `stroke_gate` continuity test, the singularity law and the pad contact axes can be borrowed. The solver, tapers, span report, pole option, provenance and offset panel must be built.
- *Other sources:*
  - `alix_workspace` — Borrow + adapt: `UrdfModel` FK as-is and the Jacobian inside the new solver; closed-form `srs_ik.solve_all` only as an optional cross-check where the elbow margin is ≥ 15°.
  - `alix_dispatch` / taught tracks — Port, shared with R2: G1/G2/G3 and the execution model as one Lite gate module; `parse_track` as-is for input checks.
  - `task_config.pole_bin_conflict` — Borrow, then re-measure: the stowed-bin band was measured on a model bin 40 mm shorter than the physical one, which has not been remade (`alix/ros/description/ugv_description/urdf/bin.xacro:25-34`).
  - Trajectory prototype — Borrow + adapt: range selection and cursor, without the Scale-stage dependency.
  - Vendor SDK `rm_algo_ik_remote` — Reference only: an optional cross-check; closed source, with a process-global tool frame.
  - Control app v53/v14 — Not available: no offset capability; the pole step is the only vertical lever.
- *Note on the inputs:* Partly accurate. The input makes no borrow claim, and its method (FK, offset each point, IK, no adjacent jumps, joint limits) is sound, but it cannot be met literally: at straight-elbow passages an exact offset does not exist, so a smooth, reported reduction must be allowed. "Obeying the joint limits" is necessary but not sufficient: limits must be judged on the predicted executed motion (G1), and continuity, accuracy, rates and collisions need gates too.

**Recommendation.** Build R9 as a Lite back-end service, not as a port of a Studio feature: a taught-following differential IK with a smooth straight-elbow taper, a smoothed adaptive taper with no hard floor, and scope ramps whose length grows with the offset so the extra pad speed stays ≤ 25 mm/s. Borrow the FK and Jacobian from `alix_workspace`, the continuity test, singularity law and pad axes from the Studio, and ALIX's G1/G2/G3 gates as one module shared with R2. Make "toilet moved" (one 4-DOF transform for both arms) the main guided case, classify every reduced span along the pad's press axis, and offer the pole option only for whole-program vertical moves of 10 mm or more. Place the offset between Smooth and Speed in the recipe (Trim → Smooth → Offset → Speed → Gates), re-apply it when anything upstream changes, and ship it only with R14's gate.

**Approach**
1. **Load** (borrow). Parse the `.txt` with `alix_dispatch`'s `parse_track` rules. Take the pole from the preceding workflow pole step and the pad frame from the region (default `{S}_glove_frame_2`). FK every point with `UrdfModel.fk_batch` on the bundled flat URDF, pole locked. Run in Lite's Python back end with its own venv (the host numpy and scipy do not load together), importing `alix_workspace` from the vendored ALIX copy; the venv must also build for aarch64 (Jetson production).
2. **Targets** (build). Robot frame: T′ = Trans(w·d)·T. Pad frame: T′ = T·Trans(w·d). Toilet frame: T′ = M(w)·T, with M a 4-DOF (x, y, z, yaw) transform, typed, from a real-test observation, or measured by the optional R14 touch-off.
3. **Weights** (build). w = scope · elbow · adapt, each smooth, with no hard floor. Scope ramps last ≥ max(10 % of span, 60 points, π|d| / (2 · 25 mm/s)). Elbow taper: clip((|q4| − 2°) / 6°, 0, 1), then a sliding minimum and Hann smoothing over ≥ ±60 points, wider for large offsets.
4. **Solve** (build, with the borrowed Jacobian). Seed each point with q′(k−1) + (q(k) − q(k−1)). Run 4 damped least-squares steps (λ 0.02) with a null-space pull towards the taught posture, then 3 polish steps (λ 0.005); cap |dq| at 0.05 rad; clip to the URDF limits minus 5°, widened to the taught extremes. A zero weight reproduces the taught track exactly.
5. **Check and repair** (build, with the borrowed continuity test). Per point: residual ≤ 1 mm / 0.5°, step ≤ max(1.5 × taught, 0.5°) and ≤ 10°. Around failing points, multiply the adaptive weight by a Hann-smoothed 0.5 window of ±0.3 s and re-solve only those windows, up to 10 passes (a whole-file pass takes about 6 s on an 11 k-point file). A remaining failure blocks Apply and is logged as a defect.
6. **File gates** (port, shared with R2). G1 executed margin never worse than the input; G2 band-limited rate before and after, with any joint newly over its rating flagged for the speed stage; G3 ≤ 10°; q4 sign pattern kept.
7. **Span report** (build). For each span with w < 1: time, mm achieved, reason and press-axis effect. For a toilet-moved offset, any span more than 2 mm deeper blocks Apply.
8. **Output** (build). Round to millidegrees and re-check the gates. Write a new version plus a provenance JSON (source and output SHA-256, side, pole, pad, scope, frame, offset, weight summary, gate results, tool version). Add one labelled undo entry to the R2 recipe.
9. **Pole option** (build; borrow the interlock after re-measuring). For a whole-program robot-Z offset of 10 mm or more, propose pole + dz. Check 0-315 mm and the stowed band with a known bin state, insert a pole step before the program and a restore step after it, show the ±5 mm note and re-check dual-arm timing.
10. **Panel and batch** (build; borrow the plot selection from the prototype). Arrows and a number field per frame, scope from the plot selection, Apply with progress, and a results card. Batch mode applies M to every program of both arms in worker processes and hands the set to R14 as one batch.

**Acceptance criteria**
- [ ] A zero offset gives output identical to the input (every joint within 1 millidegree).
- [ ] On all V6 and raw taught files, at ±10 mm along each robot axis: every step is ≤ max(1.5 × taught step, 0.5°) and ≤ 10°, there is no 360° wrap, and sign(q4) matches the taught sign wherever |q4 taught| ≥ 2°.
- [ ] Where the reported weight is 1, the output FK is within 1.0 mm and 0.5° of the request, and the reported "mm achieved" matches an independent FK within 0.2 mm. Sign tests pass on both arms: +10 mm in the robot frame moves the pad +10 mm along `base_link` X; "press harder 3 mm" moves it +3 mm along Z of `glove_frame_2`.
- [ ] The output passes G1 wherever the input did, and no joint's predicted executed margin is smaller than in the input. G2 is reported before and after; a joint newly over its rating blocks R14 until the speed stage fixes it.
- [ ] Sub-regions: rows outside the span and its ramps are unchanged; each ramp is ≥ max(60 points, 10 % of span); the extra pad speed from any ramp or taper is ≤ 25 mm/s; no single-tick weight jump exists.
- [ ] R2_LIDTOP +10 mm Z lists every reduced span with time, mm achieved, reason and press-axis effect. A toilet-moved offset with any span more than 2 mm deeper is blocked.
- [ ] The pole option is not offered for |dz| < 10 mm. It is refused outside 0-315 mm, or inside 153-282 mm when the bin is stowed or its state is unknown. It inserts its own pole step and a restore step, and shows the ±5 mm note.
- [ ] The output parses with `parse_track` and with the runner and keeps its point count; the original is untouched and one undo restores the previous version. A ±10 mm offset on any V6 file finishes in ≤ 30 s on the operator PC with progress shown, and a decimated preview is ready in ≤ 5 s; the same test is run and reported on the Jetson (aarch64) production build.

**For operators:**
- Plain words and arrows: "Move path: Towards toilet / Away / Left / Right / Up / Down" and "Press harder / lighter", with ±1 mm and ±5 mm buttons and a number field. A sub-region is chosen by dragging across the joint plot and is highlighted on the plot and in 3D.
- No solver words. A traffic-light card says, for example: "Applied fully on 81 % of this path. Reduced at 2 places where the arm is fully stretched (0:04-0:05: may press 3 mm harder — blocked)." Each place has a "Show me" button.
- For an up/down move of a whole program of 10 mm or more: "Changing the pole height does this exactly (the pole lands within ±5 mm). Use pole instead?"
- If less than about 80 % of a contact span takes the offset, or the request is above 30 mm, Lite suggests re-teaching the region.

**Safety:**
- Press-in is ≤ 5 mm, because the glove band is 20 mm and a hard surface can take large forces. A toilet-moved span more than 2 mm deeper than taught blocks Apply.
- No hard weight floor and no box-shaped tapers, because both cause one-tick target jumps; the extra pad speed from ramps is bounded. Joint limits are judged on predicted executed motion: R10_HNGR already commands J4 1.5° from 135°.
- An offset moves the program's start and end points. The controller's transit to the first point and the V6 bridges are not collision-checked, so R8/R14 must re-check them, including against the enforced venue walls.
- Pole option: the stowed-bin band was measured on the shorter model bin, and the V6 flows start at pole 200 mm, inside that band, so the bin state must be known. Never move the pole or the hand while the pad is on a surface.

| Priority | Size | Depends on |
|---|---|---|
| Should | L (excludes R14 and the shared 3D stack) | R14, R2 (recipe, undo, shared G1/G2/G3 module), R1, R7, R8, R3, R10 |

**Open questions:**
- What size and type of misplacement happen on site (mm, any turn)? This sets the proposed 10/30 mm and ±3° limits.
- Which pad frame does each region use?
- Should offsets apply to raw or smoothed files and then be re-sped (recommended), or also to already-optimised production files?
- Is a "toilet moved" offset always common to both arms?
- What coverage on contact spans is acceptable before Lite advises re-teaching? Proposal: re-teach if less than 80 % of a contact span gets at least 90 % of the offset.
- Is the bin stowed during Lite flows? The V6 flows start at pole 200 mm, inside the stowed band.
- How repeatable is the lift's landing height, and can its 5 mm tolerance be tightened?
- Lite imports ALIX libraries from the vendored copy; may that bundle include ALIX's experiment kinematics helpers (`teachlab`), or only `alix_workspace`?
- Does the 30 s budget for a ±10 mm offset hold on the Jetson, or does the production build need a lower point budget per pass?

---

### R10 — Dual-arm speed optimisation (split view, correct model, bin toggle)

**Original (inputs):**
> Dual-Arm Speed Optimization: The UI should display dual-arm simulation on the left screen and trajectory plot analysis on the right screen to optimize movement speed and cleaning time. - Borrow from ALix Studio -> should use the correct urdf and the robot meshes, also add toggle for bin inclusion and exclusion

**Revised requirement.** Lite shall provide one Dual-Arm view, with the 3D simulation in the left pane and timing and speed analysis in the right pane, where a supervisor can set per-region target times and per-segment speeds and see the effect on cleaning time, limit headroom and collisions. Both R2 optimisers are available here: the default (the trajectory prototype's trim idle → smooth → scale, with Lite's fixes) and time-optimal TOPP-RA. Lite shall draw this view from the committed ALIX URDF with a re-baked mesh overlay, take the bin geometry and positions from the URDF and SRDF, and let the user state whether the bin is fitted and where it is.

- (a) **Layout.** 3D on the left, plots on the right, split by a draggable, keyboard-operable splitter. Each pane can be maximised and restored. The view works at 1366 × 768.
- (b) **Correct model (Must).** The 3D robot uses `alix/core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf` and an overlay re-baked from it, so the visible bin and wrist camera match the collision model. The view's FK equals the collision checker's FK (parity test).
- (c) **Bin control (Must).** The bin geometry (`bin_link`, handles, lid covers) comes from the URDF; no separate bin measurement is needed (decision F5). "Bin fitted on robot: yes/no" shows or hides the bin and adds it to, or removes it from, every check; removing it needs a confirmation and prints "Bin not checked" on every result. "Bin position: stowed / out / lowered" moves it to the SRDF states `BIN_STOWED` / `BIN_EXTENDED` / `BIN_LOWERED`. Every verdict states the bin setting. A pole height that puts the arm base into the bin at the chosen position gives a warning, from a band computed from the model at start-up.
- (d) **Analysis pane.** For the selected task step or the whole flow: left and right joint plots on one time axis; a dual-lane timeline (programs, start lags, approach moves, pole moves, hand changes, waits, operator checkpoints); per-segment durations; the total cleaning time as an expected value (programs at (lines − 1) × 5 ms, the other steps modelled); and, per segment, speed headroom (the 20 Hz band-limited joint speed against 0.97 × the rating, limit L2) and acceleration headroom (the 50 ms windowed mean against 600 °/s², limit L3), with single raw ticks over a rating shown as a check only.
- (e) **Speed controls.** Per-segment settings are applied through the R2 optimisers: time % for the default optimiser (the prototype's meaning, with an optional pinned time) and speed cap % for TOPP-RA. Durations, totals and headroom update within 1 s. Factors below 1.0× are suggested when a segment is already over a limit. Simulation and collision results show "out of date" until re-checked.
- (f) **Target and achieved time per region (Must).** For each region and for the chosen optimiser, the view shows the target time (the region catalogue time, which the operator can override), the achieved time (lines × 5 ms), the recorded time and the shortest feasible time T_min. When the target is below T_min, Lite delivers T_min and shows "Closest feasible: X s — limited by joint N speed/acceleration in a–b s", with a "Use X s" action and the window shaded on the plots (decision F2).
- (g) **TOGETHER blocks (Must).** Each together-block shows both arms' target, achieved and shortest feasible times. Both arms get the same line count N = max(target, left T_min, right T_min); the view names the arm that binds, how long the other arm waits, and the drift uncertainty. The dual-arm collision timeline is re-run on the retimed pair before upload.

**Findings**
- **alix_workspace (URDF):** The committed flat URDF is correct. It matches a fresh expansion of the current xacro (2026-10-10): 128 joints and 129 links each, and 0 differences in joint types, origins, axes, limits, mimics or mesh references. ALIX Studio and alix_workspace already read this file (`alix/core/python/alix_workspace/src/alix_workspace/dual_arm_plan_collision.py:40`).
- **ALIX Studio / alix_workspace (meshes):** The baked visual overlay is stale (2026-08-15). Its `bin_link` reaches z 0.4382 m, the old `Bin.obj`, while the URDF has used `BinShort.obj` (top 0.3982 m) since 2026-08-29. `camera_link` is missing, so the server loads a 15.8 MB camera DAE from the vendored realsense tree at run time (`studio/server.py:1950-1980`). The physical bin has not been remade and is about 40 mm taller than the model (`alix/ros/description/ugv_description/urdf/bin.xacro:28-34`).
- **ALIX Studio / alix_workspace (bin state):** The bin is fixed LOWERED in at least seven places, and the Dual-Arm Collisions body set cannot be changed from the UI (`studio/server.py:1135, 1529, 1725, 1991`; `alix/core/python/alix_workspace/src/alix_workspace/dual_arm_plan_collision.py:434`).
- **Measured on V6 data (bin vs taught motion):** With the bin LOWERED, 4 of 5 taught low-pole V6 programs collide with it (sampled rows: `L_P03P05` 28/440, `R_P01P03` 111/418, `R_SPILLAGE` 160/227, `R10_HNGR` 11/142). With the bin STOWED only `R_SPILLAGE` collides (14/227). With the bin EXTENDED every sampled row collides.
- **Measured on the model (pole/bin interlock):** With the bin stowed, the arm bases enter `lid_opening_cover1` for poles from 0.070 to 0.275 m. All three documented bands disagree with this: orchestrator 0.158-0.277 m (`alix/ros/orchestration/alix_orchestrator/alix_orchestrator/core/task_config.py:62-96`), Studio 0.153-0.282 m, bin.xacro 0.16-0.285 m. V6 commands poles of 50, 150 and 200 mm.
- **ALIX Studio (layout):** There is no left/right split: the Dual-Arm Collisions tab is a top band with its log in the right dock. The dock splitter (drag, keyboard, persisted, clamped to 180-620 px, audited at 1366 × 768) can be reused (`studio/web_editor/editor.js:7562-7645`). Schedule & Run's proportional timeline has one lane only (`studio/web_editor/editor.js:13587-13612`).
- **alix_dispatch / measured rates:** Point spacing is the velocity profile, and the run speed % scales only the approach move, so speed changes must resample the file (`alix/core/python/alix_dispatch/src/alix_dispatch/track.py:9-24`). Measured as raw per-tick speeds at the 5.0 ms cycle, 10 of 15 V6 files already exceed a joint rating; `R_P06P07` peaks at 491 °/s on J6 against 225 °/s. ALIX's G2 gate counts ticks over a rating on a 20 Hz band-limited rate (`alix/tools/taught_tracks.py:2155`).
- **ALIX TOPP-RA chain (measured 2026-10-13):** ALIX already re-times taught paths with TOPP-RA (5 Hz smoothing, pause removal, 30 mm arc-length smoothing, then `retime_toppra` at 0.95 × the ratings and 600 °/s²; `alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:17-40`, `alix/tools/local_speed.py:282-324`). Re-run today it reproduces slot 101: FILE_01.1 gives 3839 lines = 19.19 s from 62.1 s raw. FILE_08 gives 16.80 s and FILE_10.1 4.26 s; one solve takes 0.11-1.82 s on the operator PC. Uniform time scaling of the profile matches a re-solve with scaled limits to within 0.0006°, so a feasible target is hit to the tick without a second solve.
- **Default optimiser (measured):** With Lite's fixes (odd-reflect edges, window 41, boundary blend) the default's fastest legal times are FILE_08 27.42 s, FILE_10.1 8.14 s and FILE_01.1 38.30 s, 1.6-2.0× slower than TOPP-RA at a similar path change (2.9-4.5 mm). On FILE_10.1 an infeasible target is limited by joint 6 acceleration, 38 % of the time, at 2.88-3.05 s.
- **Dual-arm verdicts:** a clearance verdict is tied to both tracks' sha256, so any retiming of either arm invalidates it (`alix/tools/dual_arm_verdicts.py:18-22`).
- **Control app v53 / V6 package:** v53's SAFE TIMELINE rules define the stages. TOGETHER sections may differ per arm, a WAIT before a TOGETHER program on both arms is an operator checkpoint, and RUN FIRST / WAIT OTHER run serially (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:9665-10017`). `DUAL_FLOW_TIMELINE_V6.json` is listed in the V6 checksums but absent (`lite/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/SHA256SUMS.txt:14`), so the dual timeline must be rebuilt from the per-arm JSONs and these rules.

**Can it be borrowed from ALIX Studio?** **Partial** — Borrow the robot-geometry payload and pose applier (serving a re-baked overlay through a thin loader), the viewport, the dock splitter, the playback and highlight pieces through R3 and R5, and the collision view that shows the bin's collision-only covers and handles. The Studio already reads the correct URDF, but its bake is stale and its bin is fixed LOWERED. It has no 3D-left / plots-right view, no dual-lane timeline and no speed analysis.
- *Other sources:*
  - **ALIX URDF** (`alix_workspace/urdf/alix.urdf`) — Borrow as-is: a frozen asset, regenerated only on a ROS machine.
  - **Bake script** (`alix/tools/meshes/bake_visual_overlay.py`) — Borrow as-is (re-run): re-bake into Lite's bundle, never over the ALIX copy; it needs meshoptimizer and the realsense tree.
  - **SRDF bin states** — Borrow as data: stowed (0, 0), out (0.2, 0) and lowered (0.2, 0.45) (`alix/ros/manipulation/alix_moveit_config/config/alix.srdf:286-297`).
  - **Orchestrator `pole_bin_conflict`** — Reference: keep the interlock idea, but compute the band from the model, because the constant is stale.
  - **Trajectory prototype (R2)** — Borrow + adapt: joint plots, per-region scaling and the automatic speed analysis; its trim → smooth → scale is the default optimiser, with Lite's fixes.
  - **ALIX TOPP-RA chain** (`local_speed.py`, `taught_tracks.py`, `emit_toppra_rung.py`) — Borrow + adapt through R2's adapter: the second optimiser and the source of T_min and the binding limit.
  - **Control app v53** — Reference (high level): the SAFE TIMELINE rules, re-implemented as a data model.
  - **ALIX tools** (`dual_arm_verdicts.py`, `taught_tracks.py`) — Borrow + adapt: start-lag and timing envelopes and the drift window, re-fitted on Lite runs (ALIX's envelope stops at 4,281 rows); the G2 rate gate for ticks over rating.
  - **ALIX run records** — Borrow as data (seed only): the approach-move law.
  - **V6 package** — Test data: the FINAL JSONs and the runtime CSV.
- *Note on the inputs:* Partly accurate. The Studio already reads the correct URDF and the right mesh files, but the overlay it serves is stale (old 406 mm bin, no wrist camera), and it has no 3D-left / plots-right view and no speed analysis. The note is right that a bin toggle is needed: the Studio fixes the bin LOWERED, and that state collides with 4 of 5 taught low-pole V6 programs.

**Recommendation.** Build the Dual-Arm view as a Lite screen that reuses R3's viewport and player and R5's checks, and fix the model first: re-bake the overlay into Lite's own bundle with hash, height and camera tests. Take the bin geometry from the URDF and its positions from the SRDF, and make "fitted" and "position" project settings, not a measured cell profile (F5). The taught V6 motions collide with a LOWERED bin, and a stowed bin would put the arm bases into its covers at 150 and 200 mm; this suggests the bin was not fitted when they were taught (inferred), so ask the requester before choosing a default. Build the right pane from R2's joint plots, the per-region target / achieved / closest-feasible readout of both optimisers, a dual-lane timeline that follows v53's rules with measured durations, and per-segment speed and acceleration headroom, and never present predicted durations or arm overlaps as exact.

**Approach**
1. **Re-bake (borrow the ALIX script).** Run `bake_visual_overlay.py` into Lite's bundle with the realsense tree present. Add tests: the served URDF hash matches, the bake's `bin_link` top equals the URDF's visual bin top, and `camera_link` is present.
2. **Bin defaults.** Ask the requester whether the bin is fitted and in which position, and record the answers as project defaults. The geometry itself comes from the URDF.
3. **Bin control (build).** Bin links: `bin_rail_link`, `bin_link`, `bin_handle_left/right`, `lid_opening_cover1/2`. Toggle visibility, build the bin collision set only when the bin is fitted, and take positions from the SRDF. At start-up, sweep the pole against the covers at the chosen position to get the base band; leave the J1-dependent `Link1` contacts to R5. Warn when a region's pole is in the band.
4. **Split layout (borrow the dock splitter).** A grid of [3D | splitter | plots], default 55/45, at least 320 px per pane, two presets and per-pane maximise. Resizing re-renders both the plots and the 3D view. Verify at 1366 × 768.
5. **Timeline model (build; v53 as reference).** Per-arm events from the workflow JSON: start lag, approach move (measured law), replay (lines × 5 ms), settle, pole, hand, wait or checkpoint, grouped into v53 stages. Per block, the critical path is the maximum over the two arms, and the start-lag spread comes from the ALIX envelopes. Unit-test against the V6 FINAL pair, with v53's own stage builder as the oracle.
6. **Headroom and suggestions (build).** Per segment, on the written bytes at 5.0 ms, compare the 20 Hz band-limited speed with 0.97 × the ratings (180, 180, 225, 225, 225, 225, 225 °/s; L2), and the 50 ms windowed acceleration with 600 °/s² (L3). A factor s scales velocity by s and acceleration by s². Suggested maximum factor = min over joints of min(0.97 · v_rating / v_peak, √(600 / a_peak)), capped by R2; it falls below 1.0 when a segment is over a limit. Show the raw ticks-over-rating count as a check.
7. **Target-time readout (build on R2).** Call R2's optimiser interface per region and show its OptResult: target, achieved time, recorded time, T_min, whether the target was met, and the top 1-3 binding constraints (joint, quantity, time window) for TOPP-RA or the failing check at 0.995 k_min for the default. "Use X s" sets the target to T_min. TOPP-RA is greyed out, with the reason, when the toppra import fails (for example on an aarch64 bundle built without it).
8. **TOGETHER timing (build).** For each together-block take N = max(target, left T_min, right T_min) in lines, retime both arms to N, name the binding arm, and re-run the dual-arm collision timeline (with commode, bin and venue walls, through R5) on the retimed pair before upload.
9. **Speed controls (build on R2).** Write settings as R2 segment settings, with live update. Mark R3/R5 results out of date and start the background quick re-check. Show the pace-setting arm per together-block, with the other arm's idle time and the drift band.
10. **Sync.** Hover and scrub on any plot move the 3D robot and all cursors through R3's shared `currentRow`.
11. **Re-fit (with R4).** After Lite's first real runs, fit Lite's own approach-move (against gap and transit %) and start-lag laws to replace the ALIX seeds.

**Acceptance criteria**
- [ ] The splitter resizes from 25 % to 75 % by drag and by arrow keys, and both panes re-render. At 1366 × 768 there is no horizontal scroll, each pane is at least 320 px, and each has Maximise/Restore.
- [ ] The served URDF's SHA-256 equals the committed `alix_workspace/urdf/alix.urdf`, and a scripted comparison with a fresh xacro expansion shows 0 differences in joints, limits, mimics and mesh references.
- [ ] The re-baked overlay's `bin_link` top equals the URDF's visual bin top (0.3982 m for BinShort, or the physical bin if that is chosen). `camera_link` is in the bake, and no DAE is loaded live at start-up.
- [ ] **Bin control.** "Bin fitted: no" hides all bin links, and no `bin_*` or `lid_opening_cover*` pair appears in any check. "Bin position" moves the bin to (0, 0), (0.2, 0) or (0.2, 0.45). With the bin stowed and the region pole inside the computed band (current model: 0.070-0.275 m), a warning shows. Every verdict states the bin setting.
- [ ] **Durations.** Each program's replay time is its line count × 5 ms. Start-lag, approach-move, pole and hand durations are labelled "modelled", with their source. A unit test reproduces the V6 LEFT and RIGHT flow totals from the FINAL JSONs using the same laws.
- [ ] For the V6 FINAL pair the timeline shows the stages that v53's builder produces (expected: TIMELINE-1, LID-PREP-1, LID-CHECK-1, TIMELINE-2, serial 78, serial 79, TIMELINE-3).
- [ ] **Speed changes.** Setting a segment to 1.5× updates its duration and the totals within 1 s, recomputes speed and acceleration headroom and the raw ticks-over-rating count, and marks the simulation "out of date". A segment already over a limit (for example `R_P06P07`, J6 at about 312 °/s band-limited, 491 °/s on raw ticks) shows a suggested factor below 1.0×.
- [ ] **Target time.** With TOPP-RA on FILE_01.1, a target of 25 s gives an achieved time of 25 s within one 5 ms tick and "target met"; a target of 10 s gives "Closest feasible: 19.19 s" (3839 lines) and names the binding limit. On FILE_10.1 a target below 4.26 s names joint 6 acceleration in about 2.88-3.05 s, and the window is shaded on the plots. The default optimiser shows its own T_min for the same files (FILE_01.1: 38.30 s).
- [ ] **TOGETHER.** For a together-block both arms are written with the same line count N = max(target, left T_min, right T_min), the binding arm is named, and the dual-arm collision verdict is shown "out of date" until re-run on the retimed pair's sha256.
- [ ] With toppra not importable, the view still works with the default optimiser and TOPP-RA is greyed out with a reason.
- [ ] In each together-block both arms' times are shown, the slower arm is marked, and the other arm's idle time is shown with the drift band. Hovering or scrubbing any plot moves the 3D robot and all other cursors to the same instant.

**For operators:**
- The headline is in words, for example "Cleaning time: 4 min 12 s", then "Left arm busy 3:48 — right arm busy 3:31 — right waits 41 s".
- Per region: "Target 25 s — achieved 25 s", or "Target 10 s — closest feasible 19.2 s (joint 6 accelerating hard at 2.9-3.1 s)" with a **Use 19.2 s** button. The optimiser is chosen by name: "Default" or "Time-optimal (TOPP-RA)", the labels of INT-TRAJ (b).
- A speed meter per segment, with suggestions such as "Can go up to 1.8× faster" or "Too fast now: slow to 0.6×". Nothing changes until the user presses **Apply**.
- Bin wording: "Bin fitted on robot: Yes / No" and "Bin position: Stowed / Out / Lowered". When the bin is off, a "Bin not checked" chip stays visible.
- Lanes use deck region names and arm colours (blue = left, orange = right), always with a text label. The 3D view and the plots are never hidden without a visible **Restore**.

**Safety:**
- Turning the bin off removes real collisions from the check: confirm once, and print "Bin not checked" on every result. If the bin is fitted, use the physical height or a 40 mm margin, because the model bin is 40 mm shorter.
- Compute the stowed-bin band from the model and re-measure it on the robot; the documented bands disagree with the model.
- Speed-ups raise joint rates by s and accelerations by s², which makes 4109 trips more likely (`alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:675-682`). The R2 hard limits L1-L6 on the written bytes (including L2 band-limited speed and L3 windowed acceleration) block upload; single raw ticks over a rating are a warning.
- A retimed TOGETHER pair has new sha256 values, so its old clearance verdict no longer applies; it must be re-checked before upload.
- Flow durations are predictions: programs count at (lines − 1) × 5 ms, and the start lag adds about 1 s of spread. Never use them to time the two arms against each other on the robot.

| Priority | Size | Depends on |
|---|---|---|
| Should; (b) model, (c) bin, (f) target readout and (g) TOGETHER timing are Must and ship with R3/R5/R2 | L | R3, R5, R2, R12, R8, R1, R7; R4 for measured timing; the requester's bin answers |

**Open questions:**
- Is the bin fitted on the Lite robot during cleaning, and in which position? The URDF uses the 366 mm BinShort, but `bin.xacro:28-34` says the physical bin has not been remade; confirm which bin is fitted, or keep the 40 mm margin.
- Should the view write speeds directly into R2, or only suggest them?
- What are the screen resolutions of the operator PC and of the display used with the Jetson in production? The 1366 × 768 floor is an assumption.
- Should the view cover a single task step, the whole flow, or both?
- Is the TOPP-RA solve time on the Jetson AGX Orin acceptable for the 1 s live-update goal? It is 0.11-1.82 s per solve on the operator PC; time it early on the Orin, and update the readout in the background if needed.

---

### R11 — Smoothness adjustment for both optimisers, with an immediate, tiered simulation check

**Original (inputs):**
> 11. Smoothness Adjustment: Users should be able to adjust trajectory smoothness and immediately validate the changes through simulation.

**Revised requirement.** Lite shall let the operator choose a smoothness level for each region and sub-region from named presets (Off, Light, Standard (recommended), with a numeric value under Advanced), with either optimiser (R2), and shall check the result in tiers so that the first answer appears within 1 s. Smoothing always happens before time parametrisation: in the Default optimiser it is the prototype's moving average; in the Time-optimal (TOPP-RA) optimiser it is ALIX's 5 Hz zero-phase filter followed by pad arc-length smoothing, and the solve then runs on the smoothed path. Any smoothness change marks earlier simulation and real-test results out of date, and a level that breaks G4 (or the region's MAX DEV) cannot be saved as sendable.

- (a) **Within 1 s:** the before/after joint curves, and how far the cleaning pad moves from the raw recording, in mm, measured exactly in both directions at the region's contact frame, against the G4 limit (10 mm, 5 mm preferred, or the region's MAX DEV). This figure does not depend on timing, so it is shown for both optimisers before any solve finishes. With the Default optimiser, the speed, acceleration, step and joint-limit checks (L1-L6) and the new shortest feasible time are also shown within 1 s.
- (b) **Within 3 s on the operator PC (TOPP-RA):** the new solve, the shortest feasible time, the achieved time against the target and the L1-L6 results. Less smoothing usually means a longer shortest time, because curvature then limits the speed more; Lite shows that change. The Jetson time is measured and stated.
- (c) **Within 5 s:** the predicted executed departure (G5) against the region's budget, and a collision re-check of the changed part against the commode (in the region's lid/seat state), the walls from the selected venue file, the bin and the UGV, with the pad's intended contact allowed.
- (d) **In the background:** the dual-arm clearance check against the partner arm.
- (e) **On demand:** 3D playback of the result at 0.25×–4×, with the original as an optional ghost, stopping at any problem and jumping to it from the list.
- (f) Collision verdicts report the clearance left, not only contact or no contact, because the real arm departs from the commanded path.

**Findings**
- **Trajectory prototype:** Smoothing is one global "Mean smoothing window" (101 lines), applied only when the output is generated. There is no preview, no deviation figure and no simulation, and the page says so (`lite/Alix_Studio_Lite_Trajectory_V2.html:382-383`, `lite/Alix_Studio_Lite_Trajectory_V2.html:436-439`).
- **Measured (prototype moving average, 13 Oct 2026):** Maximum pad deviation, both directions, at `L_glove_frame_2`, over FILE_01.1, FILE_08 and FILE_10.1: 101-line window 10.6–20.9 mm; 41 lines 2.9–4.5 mm; 21 lines 1.1–2.1 mm; ALIX 5 Hz zero-phase filter 1.0–2.1 mm. With odd-reflection edge padding (R2 fix), the 41-line window gives the Default optimiser's fastest legal times of 27.42 s (FILE_08), 8.14 s (FILE_10.1) and 38.30 s (FILE_01.1) at 5 ms per line, against 30.67, 10.07 and 44.04 s with 21 lines. In the Default optimiser, more smoothing buys real speed.
- **Measured (temporal filter ladder, one direction):** Maximum pad deviation on FILE_01.1, FILE_08 and FILE_10.1 (zero-phase, order 2, 5 ms clock, forward kinematics at `L_glove_frame_2`): 5 Hz 0.91–1.53 mm; 3 Hz 1.86–2.76 mm; 2 Hz 3.32–5.51 mm. On the rating-limited speed-up, smoothing adds only 3–6 % at 5 Hz and 7–10 % at 2 Hz (files in `alix/experiments/exp01_teach_trajectory_reverse_engineering/data/teach_files/commode_c_taught_paths/`).
- **ALIX TOPP-RA chain:** Smoothing comes before the solve. The 5 Hz filter (`alix/tools/taught_tracks.py:891-912`) and pause removal are followed by `arclen_smooth`, a 4th-order zero-phase filter on the path resampled every 0.5 mm of pad travel (`alix/experiments/exp01_teach_trajectory_reverse_engineering/src/local_speed.py:139-169`). ALIX calls it the main path change (`alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:111-138`). Every run slot used 30 mm; rows with a path budget use 15 mm (`alix/tools/taught_pipeline.py:141`). With 30 mm the file stays within p95 2.2–2.8 mm and max 3.2–4.7 mm of the recording (`alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:329-335`). Arc-length smoothing is deliberate, because curvature binds before the speed ratings on 99.9 % of the path (`alix/experiments/exp01_teach_trajectory_reverse_engineering/src/emit_toppra_rung.py:22-26`), so the smoothing level changes the shortest time.
- **Measured (TOPP-RA solve cost, 13 Oct 2026):** 0.11–1.82 s per solve on this x86-64 PC, plus 0.35 s to import the ALIX modules. Not yet measured on the Jetson.
- **ALIX results corpus:** On the upload channel smoothing does not improve executed accuracy (median 1.04×, 4 of 9 cells better); it helps on CANFD (0.36–0.41×) (`alix/Resource/reports/2026-09-20_geometric_intervention/RESULT_FROM_CORPUS.md:77-99`). Light smoothing's real job is removing recorder ripple.
- **ALIX taught-track tools:** ALIX measures path deviation with an exact search in both directions, about 0.2 s per 9,600 lines with a KD-tree. Windowed searches gave false readings, such as 320 mm on a re-timed track that is within 2 mm (`alix/tools/taught_tracks.py:607-716`, `alix/tools/taught_tracks.py:621-627`). ALIX's TOPP-RA slots read G4 = 0.0 mm because the check compares the derived file with itself (`alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:547-550`); Lite measures against the raw recording.
- **Measured (check costs):** The numeric checks fit in 1 s: on 12,429 lines the filter takes 1.4 ms, forward kinematics of both tracks 45 ms and the rate checks about 2 ms. The G5 ceiling takes about 6 s per 19,000 lines, so it belongs in a later tier (`alix/tools/taught_tracks.py:1828`).
- **ALIX Studio:** The 3D viewer, robot posing from server-computed link transforms, the collision timeline with click-to-scrub, Prev/Next and red-link highlight, and stale-on-edit can be reused (`studio/web_editor/editor.js:14478-14559`). Its dual-arm collision endpoint cannot: it samples at dt 0.05 s, has no commode or walls, plays at 1× only and takes 12–15 s per pair (`studio/server.py:4335-4366`).
- **ALIX dual-arm verdicts:** ALIX gates the minimum arm-to-arm separation over the start-lag window at ≥ 25 mm, sampled at dt 0.02 s with pole and hand seeded. At dt 0.05 s, links move up to 109 mm between samples (`alix/tools/dual_arm_verdicts.py:352-369`, `alix/tools/dual_arm_verdicts.py:24-61`).
- **ALIX Studio collision world:** `BodyCollisionWorld` covers the partner arm, bin, UGV, walls and commode by fixture state, at about 2 ms per pose. But it silently builds a world without the commode when the fixture scene file is missing, and it always poses the bin lowered (`studio/body_collision.py:433-434`, `studio/body_collision.py:629`).
- **Scene and walls (decision of 13 Oct 2026):** The URDF and the `commode_c` scene already hold the commode pose and articulation, the bin, the UGV and the poles (`alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml:32-68`). Walls are the one part not enforced today. They come from the venue files in `alix/ros/orchestration/alix_tasks/config/scene/`, loaded by `load_wall_manager` (`alix/core/python/alix_workspace/src/alix_workspace/plan_scene_validation.py:117-155`). With the 800 mm default venue, V6 `R2_LIDTOP` shows false wall hits on 28 of 188 sampled rows, so the venue file must match the test cell and be verified with known-good recordings (R3, R5).

**Can it be borrowed from ALIX Studio?** **Partial** — ALIX Studio covers only the simulation side: the 3D viewer and robot posing, the collision timeline with scrub, Prev/Next and red-link highlight (re-themed), the transport controls, stale-on-edit, and `BodyCollisionWorld` as a library once adapted (explicit asset bundle, synthetic config, bin pose, selected venue, fail loudly). It has no smoothing, no `.txt` input and no deviation measure, and its dual-arm endpoint defaults (dt 0.05 s, no commode or walls) must not be reused.
- *Other sources:*
  - **Trajectory prototype** — Borrow + adapt: its moving average, with odd-reflection padding, is the Default optimiser's smoothing; its compare view and cursor carry over.
  - **ALIX taught-track tools and TOPP-RA chain** — Borrow as-is through the ALIX adapter (R2): `smooth_taught`, `arclen_smooth`, the exact `deviation_mm`, and the G5 prediction (`pad_departure_joint_mm`, `departure_lambda_cap`).
  - **alix_workspace** — Borrow as-is for forward kinematics (`UrdfModel.fk_batch`, pole as q[0], `alix/core/python/alix_workspace/src/alix_workspace/urdf_kinematics.py:231-259`) and for the walls (`load_wall_manager`); Borrow + adapt for sweeps: `cross_arm_separation` for the dual-arm gate, and `compute_dual_arm_plan_collisions` with `extra_managers` and dt 0.02 for playback frames.
  - **ALIX dual-arm verdicts** (`alix/tools/dual_arm_verdicts.py`) — Borrow + adapt: the `.txt`-to-plan shim with pole/hand seeding and the swapped-side guard, and the windowed sweep.
- *Note on the inputs:* R11 has no borrow note. ALIX Studio covers only the simulation side; the smoothing comes from the prototype (Default) and ALIX's TOPP-RA chain (Time-optimal), and its measurement from ALIX's taught-track tools.

**Recommendation.** Give one Smoothness control with the same preset names for both optimisers, mapped to each optimiser's own smoothing stage, and always applied before time parametrisation. Default optimiser: Off, Light (21-line window, about 1–2 mm), Standard (41 lines, the default, about 3–4.5 mm) and an Advanced window size, with G4 blocking anything over the limit (the prototype's 101 lines fails it). TOPP-RA: the 5 Hz filter is always on; Light is 15 mm arc-length smoothing, Standard (the default) is ALIX's 30 mm (about 3–4.7 mm), with an Advanced value in mm; Off is not offered, because the solve needs a smooth path. Validate in tiers: the path change at once for both, the time result within 1 s (Default) or 3 s (TOPP-RA on the operator PC), G5 and a scene check of the changed part within 5 s, the dual-arm gate in the background, 3D playback on demand. "Simulation" here is Lite's kinematic 3D simulation, not the controller's SIM mode, which needs an upload and shows no motion or collisions. Tell operators plainly that smoothing removes hand shake and rounds corners; it does not make the arm more accurate on the upload channel.

**Approach**
1. **Presets (inside the R2 processor):** Default optimiser: `smooth_moving_mean(window)` with odd-reflection padding, at the 5 ms record clock, before the time warp. TOPP-RA: `smooth_taught` 5 Hz, then `arclen_smooth(λ)` per preset, before the solve. Sub-region overrides are cross-faded over 0.25 s at the boundaries.
2. **Tier 0, synchronous (build):** recompute the smoothed path; forward kinematics of raw and result at the region's contact frame with its pole height (borrowed from `alix_workspace`); exact deviation in both directions with a KD-tree (ALIX `deviation_mm`). For the Default optimiser, also the target fit and L1-L6 on the bytes. Return decimated curves, a deviation strip and a verdict.
3. **Tier 0b, TOPP-RA solve (worker process):** the solve, target fit and L1-L6 run in the optimiser worker; a newer change cancels an older solve. The previous result stays on screen, marked "updating".
4. **Tier 1, automatic, within 5 s (build + adapt):** G5 prediction. Collision check of the changed line ranges ±0.5 s, sampled at dt 0.02 s, in one pre-warmed `BodyCollisionWorld` worker, using a synthetic config (contact frame, fixture state), the region's pole, the bin pose from R10, the `commode_c` scene and the walls from the selected venue file (R3). Arm against arm waits for Tier 2.
5. **Fail loudly (adapt):** the collision worker refuses to run if the fixture scene, meshes or venue file are missing, and never reports "clear" in that case. Until the venue has been verified with known-good recordings, wall results carry "venue not verified".
6. **Tier 2, background (adapt ALIX dual-arm verdicts):** the `.txt`-to-plan shim with pole and hand seeded, then `cross_arm_separation` with a windowed minimum ≥ 25 mm at dt 0.02. Cache by the SHA of both files plus the scene and venue.
7. **Present results (borrow Studio components, re-themed):** red bands on the plots, red links in 3D, and verdict sentences that state the clearance left. Playback runs at 0.25×–4×, stops at the first problem and can show a ghost of the original.
8. **Stale-on-edit (Studio pattern):** any change marks the region's Simulated, Real-tested and Upload-ready states out of date.
9. **Tests (both architectures):** preset deviations on FILE_08, FILE_10.1 and FILE_01.1 for both optimisers; the deviation equals ALIX `deviation_mm`; tier timings; the windowed re-check equals a full sweep; a missing fixture or venue gives an error; pad contact is not flagged and thumb contact is; stale-state transitions.

**Acceptance criteria**
- [ ] Changing smoothness on a 12,000-line file updates the overlay and the path-change figure within 1 s with either optimiser. With the Default optimiser the L1-L6 verdict and the shortest feasible time also update within 1 s; with TOPP-RA they update within 3 s on the operator PC, and the Jetson time is measured and recorded.
- [ ] The displayed deviation equals ALIX `deviation_mm` (exact, both directions) within 0.1 mm on FILE_08 and FILE_10.1 for every preset of both optimisers.
- [ ] Default optimiser: Light (21 lines) moves the pad ≤ 2.2 mm and Standard (41 lines) ≤ 4.6 mm on FILE_01.1, FILE_08 and FILE_10.1; a 101-line window fails G4 on all three. TOPP-RA Standard (30 mm) stays within 4.8 mm of the recording on the same files.
- [ ] A level over the G4 limit or the region's MAX DEV shows red and cannot be saved as sendable. The message names the limit and the lighter preset that fits.
- [ ] When a smoothness change moves the shortest feasible time past the target, the target chip changes to "Closest feasible" and names the binding limit (R2).
- [ ] Within 5 s of a change that affects ≤ 10 s of motion, a scene collision verdict for the changed window is shown, with the minimum clearance; it names the venue file used.
- [ ] Contact between the commode and the contact frame's allowed links is not reported as a collision; contact by any other link is. If the fixture scene, meshes or venue file are missing, the check reports an error, never "clear".
- [ ] The smoothed region plays in 3D at 0.25×–4× with scrubbing. Clicking a collision mark poses the robot there with the colliding links in red.
- [ ] After a smoothness change, "Simulated" and "Real-tested" read "out of date" until re-run.
- [ ] Smoothing keeps the first and last points within 0.05° of the recording (L6).

**For operators:**
- One sentence per preset. Light: "removes hand shake; the path moves about 1–2 mm". Standard: "recommended; the path moves about 3–4 mm". Advanced values are for engineers.
- "Path moved 1.2 mm (limit 10 mm)" is shown large, with a Before/After toggle. A "Checking…" chip turns into the verdict, and editing is never blocked while checks run.
- When smoothing changes the time, Lite says so: "Less smoothing: the fastest safe time rises to <new time>."
- Lite never suggests that more smoothing is safer or more accurate.

**Safety:**
- Smoothing changes the path. It can lose contact (missed spots) or create collisions near commode edges, so the changed part is re-checked after every change.
- The collision world must include the commode, the walls of the selected cubicle (1400 mm by default, 800 mm selectable), verified once with known-good recordings, the bin and fingers, with pole and hand seeded. A missing asset stops the check, because `BodyCollisionWorld` otherwise drops the commode silently. A wrong venue file gives false wall hits (or misses), so the venue choice is shown with every verdict.
- A kinematic "clear" is not proof. The executed path departs p95 10–23 mm at 1× (`alix/plan/FINDINGS.md:6699-6716`), and G5 is a p95 prediction with a worst case about twice it. Keep the slow real test before upload.

| Priority | Size | Depends on |
|---|---|---|
| Should (the hard gates are Must, under INT-TRAJ) | L (M if the R3/R5 collision worker already offers a windowed scene check) | R2, R3, R5, R10, R7, R1; venue selection and verification (R3) |

**Open questions:**
- Is a single-arm check against the scene enough for "immediate", with the dual-arm check in the background?
- Is the contact frame always `{S}_glove_frame_2`?
- Is the thumb allowed to touch the lid?
- How far do the TOPP-RA Light (15 mm) and Advanced arc-length settings move the pad on the commode C recordings, and how much do they lengthen the shortest time? Not yet measured; to be measured before the preset values are fixed.
- Does the Jetson meet the 3 s TOPP-RA tier? To be measured with the first aarch64 build.

---

### R12 — Spike, limit and sudden-motion analysis with one-click fixes

**Original (inputs):**
> 12. Plot Spike Analysis: The software should identify high and low spikes in joint trajectory plots and provide suggestions to reduce sudden movements and improve trajectory smoothness.

**Revised requirement.** After every change, Lite shall scan the output file at the 5.0 ms tick, mark every sudden or unsafe place on the joint plots, and list it in plain language with where it is (time, joint, value), why it matters and one or two suggested fixes. Applying a fix takes one click, becomes a labelled, undoable change in the region's history (R2), and triggers a new scan.

- (a) **Too fast.** Must fix: a joint over 0.97 × its rating (180/180/225/225/225/225/225 °/s) after the 20 Hz band limit (ALIX G2; R2 limit L2). Warning: over a rating only on single raw ticks.
- (b) **Step fault.** A step over 10° between lines. It blocks upload.
- (c) **Isolated jumps.** "High" and "low" spikes: sharp upward and downward excursions of a joint curve or its speed, marked ▲ and ▼ (D10: "low" means a sudden downward jump, not a hesitation).
- (d) **Sudden speed changes.** Must fix: acceleration above 600 °/s² × 1.02 as a 50 ms windowed mean (R2 limit L3, hard in Lite). Advisory: the 20 Hz Butterworth acceleration reading, which overshoots on TOPP-RA output. Should fix: a one-tick speed step at a sub-region boundary or at either end of the file.
- (e) **Near a joint limit.** Must fix: ALIX G1 predicts the arm will come within 0.5° of a limit or cross it. Warning: the commanded margin is inside 5°.
- (f) **Hesitations.** Mid-stroke slow-downs that were not removed as stops (information).
- (g) **Suggestions include optimiser choices.** Besides local fixes, a suggestion may be "Switch to the time-optimal optimiser (TOPP-RA)" or "Relax the target time to X s", where X is R2's closest feasible time T_min, when the finding comes from a target that the chosen optimiser cannot meet within the limits.

**Findings**
- **Trajectory prototype:** Its analysis averages the overall step size over 5–10 equal chunks and ranks them with relative quartile rules. It has no per-joint speeds, no thresholds and no markers, so it cannot find a spike (`lite/Alix_Studio_Lite_Trajectory_V2.html:944-1050`).
- **ALIX Studio:** There are no joint plots and no taught-track analysis. `/api/analyze-plan` has no UI caller, checks only a plan's `velocities` field and never fills `acc_violations` (`studio/server.py:4272`, `alix/core/python/alix_workspace/src/alix_workspace/plan_analysis.py:126-136`).
- **ALIX taught-track tools:** The detectors already exist: a rate report (raw and 20 Hz band-limited, ticks over, longest run), step and position reports, gate reasons in plain text, `rate_fit_track`, an acceleration advisory, `slowness_census` and per-joint departure attribution (`alix/experiments/exp01_teach_trajectory_reverse_engineering/src/predict_track_replay.py:92-136`, `alix/tools/taught_tracks.py:420-604`).
- **V6 package and lid_top files (measured):** 14 of 19 shipped files exceed a rating on single raw 5 ms ticks, but only 6 fail ALIX's enforced band-limited gate G2 (`alix/tools/taught_tracks.py:594-599`). They are `R_P06P07` (102 ticks, J6 at 312 °/s = 1.39× its rating, longest 285 ms), `R_P10P12` and the four files in `lite/upload_channel/plan_txt_files/`. The other 8 V6 files are over only on raw ticks.
- **ALIX cleaning tracks:** Raw ticks still matter. The controller flags each one as warning 4105, latched for the run, and one ALIX program ended at 58 % of its track with the flag up (`alix/tools/cleaning_tracks.py:211-218`). Other runs carried on with 4105 (`alix/docs/motion/TRACK_CHANNEL_TIMING_REQUIREMENT.md:31-36`).
- **V6 package:** `R10_HNGR.txt` takes J4 to 133.47°, 1.53° from its 135° limit, arriving at about 21.5 °/s. ALIX's current G1 rule predicts about 3.0° of overshoot, so the arm would cross the limit; slowing the arrival to ≤ 7.4 °/s satisfies it (`alix/tools/taught_tracks.py:1352-1365`). J4 has run 5.6–5.9° past 135° on hardware, on a slow track (`alix/plan/FINDINGS.md:6721-6737`).
- **V6 package (measured):** `R_P06P07.txt` has an isolated J3 jump of 1.049° at lines 5058→5059, between steps of 0.271° and 0.276°.
- **Trajectory prototype:** Editing itself creates spikes: overwriting the end points gives first steps of 966 and 1005 millidegrees, and unblended sub-region boundaries give one-tick speed changes of 440–880 °/s² (`lite/Alix_Studio_Lite_Trajectory_V2.html:1721-1723`, `lite/Alix_Studio_Lite_Trajectory_V2.html:1568-1616`). The 13 Oct re-measurement found end snaps of 546–1005 millidegrees and boundary spikes of 640–919 °/s²; with R2's fixes (odd-reflect edges, 0.25 s boundary blend) the spikes drop to 200–280 °/s² (`lite/Alix_Studio_Lite_Trajectory_V2.html:1513-1514`).
- **ALIX TOPP-RA output (measured 2026-10-13):** At the optimum, acceleration binds almost everywhere. The bytes keep 600 °/s² to within 0.7 % as a 50 ms windowed mean, but read 1.18–1.22× on ALIX's 20 Hz Butterworth, which is the filter overshooting, not the robot. With the optional 6 Hz post low-pass the windowed reading rises to 1.17–1.25× (`alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:184-191`). The acceleration detector must therefore use the windowed mean for its hard limit.
- **Default vs TOPP-RA (measured):** The default optimiser's fastest legal time is 1.6–2.0× TOPP-RA's (FILE_01.1: 38.30 s against 19.19 s), so a too-fast or acceleration finding caused by a short target may be cleared by switching optimiser instead of slowing the region.

**Can it be borrowed from ALIX Studio?** **Build new** — ALIX Studio has no joint plots and no taught-track analysis; `/api/analyze-plan` is reference only. Only presentation parts can be reused: validation rows with glyph, sentence and location (`studio/web_editor/editor.css:1462-1484`), gate chips (re-implemented, because they are coupled to Studio gates), Prev/Next with click-to-scrub, and stale-on-edit.
- *Other sources:*
  - **ALIX taught-track tools** (`taught_tracks.py`, `cleaning_tracks.py`, exp01 `predict_track_replay.py`) — Borrow + adapt (port): `rate_report`, `step_report`, `position_report`, G1 through `model_predictions`/`model_caps` with the execution-model constants, `gate_report` wording, `rate_fit_track`/`capped_millideg` for fixes, the acceleration advisory, `slowness_census` and `_departure_attribution`.
  - **ALIX `taught_tracks.py screen`** — Borrow as-is, now: a batch triage of the legacy files that writes nothing, run in the ALIX environment (`alix/tools/taught_tracks.py:4149-4162`).
  - **R2 optimisers** (Lite) — Use: their OptResult (T_min, target met, binding constraints, L1–L6 results) feeds the "switch optimiser" and "relax target" suggestions.
  - **Trajectory prototype** — Reference only for analysis (`analyzeMotionRegions` measures nothing physical); Borrow + adapt for the plot cursor and marker shapes.
- *Note on the inputs:* R12 has no borrow note. Neither ALIX Studio nor the prototype meets it; the detectors and the fixes exist in ALIX's taught-track tools.

**Recommendation.** Build a server-side detector suite that runs on the output bytes at 5.0 ms. ALIX's band-limited G2 decides "too fast", raw ticks only warn, ALIX's G1 execution-model rule decides "near a limit", and steps and spikes use raw values. Pair it with a fixed rule table that turns each finding into one or two previewable recipe fixes; no AI is involved. Show ▲/▼ markers, shaded bands and a severity-grouped list linked to the 3D view; every fix is an R2 operation, so it is labelled and undoable.

**Approach**
1. **Port ALIX's instruments (borrow + adapt):** `rate_report` (raw and band-limited; merge gaps under 50 ms), `step_report`, `position_report`, the G1 `model_predictions` with the execution-model constants, the acceleration advisory, `slowness_census` and `_departure_attribution`.
2. **Detectors for (a)–(f) (build)** on the output bytes at 5.0 ms. Each returns findings `{id, kind, severity, joints, t_start, t_end, peak, threshold, line_range, suggestions[]}`. The isolated-jump thresholds are proposals to tune on the corpus: a step over 0.5° and over 3× both neighbours, or over 4× the local median (±50 lines) and over 0.3°.
3. **Suggestion rule table (build).** Too fast in a sped-up sub-region: "Lower speed from a× to b×" or "Let Lite slow only this stretch". Raw ticks only: "Light smoothing here". Step fault: "Re-teach this part", or undo the edit that made it. Isolated jump: "Smooth this spot". Boundary: "Blend boundary". Near a limit: "Slow the approach to ≤ X °/s here", with X = (margin − 0.5°) ÷ 0.139 s, or, when the commanded margin is under 1.35°, "Re-teach further from the limit". Hesitation: "Remove this stop". Target-driven (the finding is caused by a target shorter than the chosen optimiser's T_min): "Relax the target to X s" (X = T_min, with the binding limit named), and, when the default optimiser is selected and TOPP-RA's T_min is at or below the target, "Switch to the time-optimal optimiser". If TOPP-RA is unavailable on this machine, the switch suggestion is not offered.
4. **One-click fix (build):** applied as an R2 recipe operation with a label, after a dry-run preview (under 0.5 s). The file is re-scanned afterwards, and undo brings the finding back.
5. **Findings panel and overlay (build, with Studio presentation parts):** groups Must fix / Should fix / Advisory with a count badge; Show / Fix / Ignore-with-reason; markers and bands on the affected joint; an optional speed-utilisation plot with a 100 % line.
6. **Gates:** too fast after the band limit (L2), windowed acceleration (L3), step faults (L4) and G1 near-limit findings (L1) block real runs and uploads (the same module as INT-TRAJ's and R2's hard limits). Everything else warns. Under this rule, 7 of the 19 shipped files are blocked until fixed: the 6 that fail G2 and `R10_HNGR` (G1). This count uses ALIX's G2 at 1.0 × the ratings; it must be re-counted with L2 at 0.97 × and the L3 acceleration limit, which can only add files.
7. **Regression corpus:** the 15 V6 files, the 4 lid_top files and the raw commode recordings, with golden findings.
8. **Legacy triage now:** run ALIX `taught_tracks.py screen` on the legacy files to prepare the rollout report.

**Acceptance criteria**
- [ ] On `R_P06P07.txt` the scan reports a Must-fix too-fast stretch on J6: band-limited peak about 312 °/s (1.39×), 102 band-limited ticks, longest 285 ms, with the raw figures (491 °/s, 172 ticks) as detail. It also reports the isolated J3 jump at lines 5058–5059.
- [ ] On `R10_HNGR.txt` it reports J4 (1.53° from 135°, arriving at about 21.5 °/s) as Must fix and offers "slow the approach to ≤ 7.4 °/s" and "re-teach". After the slow-down fix, G1 passes.
- [ ] On `R2_LIDTOP.txt`, `L_P01P02.txt` and the other raw-only files, "too fast" is a warning, not Must fix, and Light smoothing removes it.
- [ ] Every finding is drawn on the affected joint's plot at the right time (±1 line). Its list entry gives the time (s), joint, value and limit in plain words, and clicking it moves the plot cursor and the 3D robot there.
- [ ] Applying the suggested fix for a too-fast finding removes it, adds no new Must-fix finding, and appears as one labelled undo entry. Undo brings the finding back.
- [ ] A file with any step over 10° cannot be sent, and the message says where.
- [ ] A full scan of a 12,000-line file takes under 0.5 s; G1 and G5 take under 5 s.
- [ ] With the default R2 recipe, FILE_08 and FILE_10.1 show no end-point or boundary findings; the prototype-processed versions do.
- [ ] With the default optimiser and a FILE_01.1 target below its T_min (38.30 s) but at or above TOPP-RA's (19.19 s), the finding offers "Relax the target to 38.3 s" and "Switch to the time-optimal optimiser"; applying the switch clears it, as one labelled undo entry. Without toppra installed, only the relax suggestion appears.
- [ ] TOPP-RA output at T_min raises no acceleration Must-fix (windowed mean within 600 °/s² × 1.02); the Butterworth reading is shown as advisory only.

**For operators:**
- Every severity has an icon and a word, never colour alone.
- Plain sentences, for example "Joint 6 moves too fast here (312 °/s; the arm's limit is 225 °/s) for 0.3 s", each with a short "Why does this matter?".
- One main Fix button with a preview, plus "Show me". Adjacent findings are merged. No blanket "Fix all" until the fixes are proven on hardware.

**Safety:**
- Must-fix findings block real runs and uploads, and fixes are never applied silently.
- Slow-down fixes keep the path but change timing, so a dual-arm step needs its clearance re-checked (R10). Smoothing fixes change the path, so collisions are re-checked (R11, R5).
- Near-limit thresholds come from a fitted model, and J4 has crossed 135° on hardware, so the 5° warning stays even when G1 passes.
- Acceleration is advisory on ALIX's upload channel (`alix/tools/cleaning_tracks.py:278-290`); Lite makes the 50 ms windowed mean a hard limit (R2 limit L3) and keeps the 20 Hz Butterworth reading advisory.
- "Switch optimiser" changes the path (different smoothing), so collisions and path change (L5) are re-checked after it, as for any smoothing fix.

| Priority | Size | Depends on |
|---|---|---|
| Should; its hard gates (too fast after the band limit, windowed acceleration, step faults, G1 near-limit) are Must under INT-TRAJ | L | R2, INT-TRAJ, R11, R5, R10 |

**Open questions:**
- Should operators see the Butterworth acceleration advisory, or only Engineer mode?
- Is a "Fix all safe issues" button wanted?

---

### R13 — Faster trajectory upload

**Original (inputs):**
> Faster Trajectory Upload: The software should support fast and efficient uploading of trajectory files to the robot controller for real-arm testing. - Check some of the findings in the alix ws related work

**Revised requirement.** Lite shall get trajectories onto the robot controllers quickly and safely, and make the waiting visible. The controller fixes the transfer rate (about 45 kB/s), so the gain comes from sending fewer points (R2), sending only what changed, one session per arm, both arms in parallel and uploading in the background.

- (a) **[Must] Exact, verified upload.** Upload exactly the content that passed simulation and the checks, in the controller's program format: every line ending in CRLF, header `{"file":7}`, then a Folder line whose `type` equals the point count. Store only; never auto-run. Verify each program by reading back its stored size, which must equal the bytes sent within the configured tolerance (default `max(64 B, 0.1 %)`, tightened to exact if commissioning shows exact matches). A failed verification blocks running that program.
- (b) **[Must] Any Program ID, overwrite after confirmation.** As in the control program, Lite uploads to any Program ID from 1 to 100 on either arm, in single uploads and in batch uploads with a Program ID per file, and runs a stored program by ID or by name. Just before writing (one list read per batch, taken under the arm's lock) Lite reads the controller's program list; if the target slot holds a program, Lite shows that program's current name and size and asks to replace it (Replace / Skip this file / Cancel the rest of the batch). Lite also shows soft warnings, which never block: the slot holds a program Lite's ledger did not write (for example a pendant program), the slot is in a range ALIX uses (70–100 production, 60 dry, 40–59 taught, 21–39 and 61–69 experiments), or the slot holds a program from the V6 workflow (71–83).
- (c) **[Should] Skip unchanged.** When Lite's registry hash, the stored size and the stored name (which carries a content-hash fragment) all match, show "Already on robot" and send nothing. Any mismatch or unreadable value means send again.
- (d) **[Should] One session per arm, both arms in parallel.** One connection and one program-list read before and one after each batch. Both arms upload at the same time.
- (e) **[Should] Background queue with visible cost.** Uploads run in the background with per-arm progress and a time estimate while the operator keeps working. Optionally a trajectory is queued as soon as it passes simulation, but a background upload never replaces an occupied slot without the confirmation in (b); it waits for it. The editor shows each trajectory's upload cost (points → seconds).
- (f) **[Must] Busy interlock.** Never upload to an arm that is running a program, drag-teaching or moving its pole or hand. An engaged E-stop does not block uploads, because an upload does not move the arm (R4 (h)); Lite shows a "robot in E-stop" note.
- (g) **[Could, future] CANFD streaming**, which needs no upload, as a separate engineering option with its own safety case.

**Findings**
- **ALIX Studio:** No upload, slot or controller code at all, and a test pins that it has no execution channel (`alix/core/python/alix_taskgen/test/test_studio_interaction_guards.py:2204-2216`).
- **Control app v53/v14 (upload path):** The runner's upload is ALIX's original `alix/tools/rm_upload.py` (identical by diff apart from docstrings). It parses the whole file and refuses any bad line, writes `{"file":7}` + Folder(type = N) + the points in CRLF, stores with `only_save=1, auto_start=0`, and verifies the stored size within `max(64, sent // 1000)` (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:76-175`). That tolerance lets about 10 points go missing on a 600 KB file; the one recorded hardware read-back matched exactly, 729,857 bytes sent and stored (`lite/upload_channel/README_COMMANDS.md:51`). The "fast" in the runner's name means fast switching between stored programs, not faster upload.
- **Control app v53 (batch and slots):** Batch upload starts 2N+1 runner processes, each with its own connection, for N files (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:4504-4692`). It already uploads LEFT and RIGHT in parallel. v53 accepts any Program ID 1–100, offers batch upload with a Program ID per file and a replacement confirmation, and runs a stored program by ID or by name (from the refreshed program list) (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:9-15, 925-936, 4781-4800`). The batch path reads the program list immediately before each file and, when the slot is in use, shows the existing program's name with YES = replace, NO = skip, CANCEL = stop the batch (`…v53_safe_lid_checkpoint.py:4458-4480, 4535-4545`). The single UPLOAD asks only "If the slot already contains a program, it will be replaced" and passes `--yes`, which skips the runner's prompt showing what the slot holds (`…v53_safe_lid_checkpoint.py:4275-4290`); Lite shows the slot's current program in both paths. v53 defaults to slot 99 for batch and 25 for single uploads.
- **alix_workspace (throughput):** The controller acknowledges every 2 KB chunk, and it answers late during and after a large upload (`alix/plan/FINDINGS.md:5701-5706, 5816-5822`). Five ALIX uploads of 180–325 KB took 4.04–7.15 s (44.3–46.5 kB/s); the fit is 0.24 s + 21.2 µs × bytes, about 1.2 ms per point (derived from `alix/runs/2026082{8,9}T*_upload*/run.json`). No faster transfer path exists in the vendor API or ALIX's evidence. Worked example (derived): V6 LEFT (7 files, 2.60 MB) about 55 s and V6 RIGHT about 51 s, so about 1 min 46 s one arm after the other or about 55 s in parallel; the largest file, `L_P03P05.txt` (632 KB), about 13.7 s. Files above 325 KB are extrapolated.
- **alix_workspace (fewer points):** Fewer points is the main lever. Operator holds are about 24 % of nominal time, and TOPP-RA re-timing gives 3.2–4.0× fewer lines (`alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:41-51, 582-591`). On the same toplid path, 5,047 → 3,142 points cut the upload from 6.43 s to 4.04 s.
- **alix_dispatch and ALIX tools:** `alix_dispatch.track` has the same verbs as a library: `build_project_file`, `send_project` (returns seconds), `program_list` (pages of 10) and `preflight_slots`, which notes that "a short slot would end its program early with no error" (`alix/core/python/alix_dispatch/src/alix_dispatch/track.py:66-152, 353-390`). `cleaning_tracks.py upload` already batches over one connection with one list read and sha256 pins, and refuses programs it did not write (`alix/tools/cleaning_tracks.py:4046-4160`); Lite keeps the ledger and pins but turns that refusal into a confirmation, as the control program does.
- **alix_workspace (slots and naming):** ALIX uses 70–100 (production), 60 (dry), 40–59 (taught; 101 and up are CANFD-only registry numbers), and 21–39 and 61–69 (experiments); slots below 40 are shared with pendant programs (`alix/tools/cleaning_tracks.py:2858-2870`), and slot 1 holds one. V6 uses 71–83. The controller stores a program as `<id>_<stem>.txt`, cut to 32 bytes and always `.txt`, as ALIX measured on hardware on 2026-09-10 (`alix/core/python/alix_dispatch/src/alix_dispatch/emulated_program.py:326-341`). With a 3-digit ID the stem must be 24 characters or fewer, and the temporary project path must stay under 300 bytes.
- **V6 package:** At 5 ms per point, 10 of the 15 files exceed a joint velocity rating; the peak is 2.18× on `R_P06P07.txt` J6 (491°/s, 2.45° per step) (derived from `lite/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/`). A strict R12 rate gate would block the current production package. Fast files also overshoot commanded positions by 4–20° (`alix/plan/FINDINGS.md:9550-9552`).

**Can it be borrowed from ALIX Studio?** **Build new** — The Studio has no upload, slot or controller code, and a test pins that it has no execution channel. Only generic UI parts can be reused for the queue: the status line and toast (`studio/web_editor/editor.js:5431-5462`) and a Board-style status table.
- *Other sources:*
  - Runner v14 upload path (= `alix/tools/rm_upload.py`) — Borrow as-is for the MVP, plus one opt-in addition: the exact `.proj` bytes and the read-back verify; add `--upload-batch MANIFEST.json` in Lite's runner copy.
  - `alix_dispatch.track` — Borrow + adapt (target): `build_project_file`, `send_project`, `program_list` and `preflight_slots` for an in-process store with one persistent session per arm.
  - ALIX `cleaning_tracks.py` — Borrow the pattern: one-connection batch, a single read-back, sha pins and the name treated as advisory while the naming model is unconfirmed; its refusal of programs it did not write becomes a replacement confirmation with a warning.
  - ALIX measurements — Borrow as data: the ETA model, the CRLF rule, paging by 10, the controller-busy behaviour.
  - `alix_dispatch.emulated_program` — Borrow for tests: an offline controller double that checks the format, the 32-byte name and the 300-byte path, and counts sends and list reads.
  - UI v53 — Reference for the capability Lite must match: Program IDs 1–100, batch upload with per-file IDs, the replacement confirmation that shows the slot's current program, run by ID or name, parallel L/R uploads. Do not copy the 2N+1 process pattern or the single upload's `--yes` without showing what the slot holds.
  - CANFD streaming (`alix_dispatch.track`) — Reference only (future): it avoids upload but needs host timing, a different rate gate and its own safety case; ALIX's taught CANFD pipeline is left-arm only.
  - Trajectory prototype — Not applicable: no controller I/O; its "Upload" means opening a file.
- *Note on the inputs:* Accurate as a pointer. ALIX holds the measured throughput, the exact format and CRLF rule, paged listing, sha and size pins, a one-session batch, its slot-use conventions and the naming model. The same findings show that the controller fixes the transfer rate, so "faster" must come from fewer points, skipping unchanged programs, one session per arm, both arms in parallel and background queuing, or from avoiding upload with CANFD, a different and riskier channel.

**Recommendation.** Treat upload as a background service built on a content-identity registry, not as a button: send only changed content, over one session per arm, with both arms in parallel, keep the proven byte format and size read-back, and show an honest estimate. The real speed gain comes from R2 (a trajectory sped up 2× has half the points and uploads in half the time), so the editor should show the upload cost. For the MVP keep the runner's upload code and add an opt-in batch mode in Lite's copy; move to an in-process `alix_dispatch.track` session later, and keep CANFD streaming as a separate, later decision. Match the control program's slot capability (any ID 1–100, overwrite after a confirmation that shows the current program, per-file IDs in a batch, run by ID or name), with the ledger and soft warnings protecting other people's programs. Agree how R12 treats the existing V6 files that exceed the joint ratings.

**Approach**
1. **Program IDs (build).** Any ID 1–100 per arm, chosen per file (a batch dialog with one ID per file, as v53). Before each write, read the list and, for an occupied slot, show the current name and size with Replace / Skip / Cancel. Soft warnings from the ledger and a configurable list of known ranges (ALIX, V6 71–83, pendant); none of them blocks. Run by ID or by name through the same list.
2. **Registry and naming (build).** One JSON file per arm maps each slot to `{txt_sha256, proj_sha256, proj_bytes, points, short_name, stored_name, uploaded_at, verified}`. Names such as `L1A_r3_3f9c2a` (side + region code + revision + 6 hex of the content sha, 24 characters or fewer); temporary path under 300 bytes. The side tag also lets Test connection (R4) detect swapped IPs.
3. **Runner copy with batch mode (borrow + adapt runner v14).** In Lite's pinned runner copy, add an opt-in `--upload-batch MANIFEST.json` that reuses `read_track`, `write_project`, `rm_send_project(only_save=1)` and the size verify over one connection, with one list read before and one after, and prints VERIFIED / NOT VERIFIED / SKIPPED with seconds per file. Prove by diff that the default paths are unchanged; test against `emulated_program`.
4. **Upload service (build).** One worker per arm, both in parallel. Per batch: take the per-arm lock and check run state 0; read the list; collect the replacement confirmations for occupied target slots (before the batch starts, so the worker never waits on a dialog mid-send); compute the skip set (sha, size and stored name all match); send the rest; read back once; update the registry; release the lock.
5. **Pre-upload gate (build a hook).** Run the R12 checks before queueing, with plain reasons. Record any engineer override with its reason; the existing V6 files will need this.
6. **Queue UI (build; Studio status line and toast borrowed).** Per-arm progress and an ETA from bytes (the SDK has no progress callback). States: Waiting, Sending, Checking, On robot, Already on robot, Not verified. Jobs can be cancelled before sending, and "needed next" goes first. An optional setting pre-stages a trajectory after its simulation passes; it waits for the replacement confirmation rather than overwrite an occupied slot.
7. **Upload-cost readout (build).** "Send to robot: about X s (N points)" next to the duration in the editor, using ETA = 0.24 s + 21.2 µs × bytes, recalibrated from Lite's own logged seconds.
8. **Hardware measurement.** One 600 KB file, the V6 package serial versus parallel, and the skip path. Calibrate the ETA, check whether stored sizes match exactly, and confirm the naming model on both controllers.
9. **Later.** Phase 2: an in-process store on `alix_dispatch.track` with `connect.open_arm` (persistent session, reconnect), keeping the same registry, tests and UI. CANFD streaming only by a separate decision.

**Acceptance criteria**
- [ ] **Bytes and store-only.** For three sample files (one LF, one CRLF), Lite's `.proj` bytes are byte-identical to runner v14 `write_project` output. Every upload uses `only_save=1`, `auto_start=0` and `project_type=0` (asserted on a fake SDK).
- [ ] **Verification.** Each program is verified by stored size within the configured tolerance. A mismatch shows "Not verified: do not run" and disables running that program.
- [ ] **One session.** A batch of N files to one arm opens exactly one connection and performs at most two program-list reads (fake counters).
- [ ] **Skip.** Unchanged content causes zero sends and shows "Already on robot". Changing one file in a batch of seven sends exactly one. A stored-name mismatch triggers a re-send, never a skip.
- [ ] **Parallel and timing.** With a fake controller at 21 µs per byte, LEFT and RIGHT together take at most 1.15 × the longer arm alone. On hardware, the V6 LEFT package (7 files, 2.60 MB) uploads and verifies in 70 s or less, and both arms in parallel (about 5.0 MB) in 75 s or less. The ETA is within ±25 % of the measured time for at least 5 files of 50 KB or more.
- [ ] **Program IDs and replacement.** Uploads to IDs 1, 50 and 100 succeed on a fake controller; 0 and 101 are rejected as invalid. Uploading to an occupied slot shows that slot's current name and size (read from the list taken just before the write) and writes only after Replace; Skip leaves it unchanged and Cancel stops the rest of the batch. A batch of three files with three different per-file IDs writes each to its own ID. A slot holding a program not in Lite's ledger, or in 70–100 or 71–83, shows a warning but can be replaced after confirmation. Background pre-staging never replaces without confirmation. A stored program can be run by ID and by name.
- [ ] **Names.** Stored names are 32 bytes or fewer, including `<id>_` and `.txt`, and carry the side, region and a content-hash fragment.
- [ ] **Busy interlock.** Upload is refused while that arm has an active run, drag-teach, pole or hand job, or reports run state 1 or 2.
- [ ] **Motion gates.** A file that fails the motion gates (for example, a 12° step) is never sent without a recorded engineer override.

**For operators:**
- "Send to robot" means the controller upload; the trajectory HTML's "Upload trajectory" becomes "Open file".
- A per-arm queue reads "Left arm: sending 3 of 7, about 40 s left", with chips On robot / Already on robot / Not verified. Sending never blocks editing or simulation, and a toast reports when a batch finishes or fails.
- The editor shows the cost next to the duration: "Send to robot about 9 s (7,353 points)"; after a 5× speed-up, "about 2 s (1,471 points)". Estimates say "about" because the SDK reports no transfer progress.
- The operator sees the robot program number (1–100) for each upload, pre-filled by Lite with the region's usual number or a free one, and can change it; the engineer area shows every slot with its current program. Before replacing a program Lite asks: "Program 25 on the left arm holds 'M3_OPNST02' (412 KB). Replace it?" A program Lite did not upload, or one from the V6 workflow, gets an extra note, not a block. A background job never asks; it waits.

**Safety:**
- An upload never moves the arm: keep `only_save=1` and `auto_start=0`; `only_save=0` starts the program immediately.
- Keep the size read-back: ret 0 means "accepted", not "stored completely", and a short slot ends early with no error. Refuse malformed files whole; if the registry or the read-back cannot be read, send again.
- Never upload to a busy arm, and never replace a stored program without a confirmation that shows what the slot holds now; a program Lite did not write gets a warning in that confirmation. The size and name read-back protect against running a slot that someone else changed after Lite's upload.
- Fewer points means faster motion, which is safe only behind R12's rate, step and joint-limit-margin gates.

| Priority | Size | Depends on |
|---|---|---|
| Must ((a), (b), (f) Must; (c)–(e) Should; (g) Could) | M (phase 2: S; CANFD streaming is a separate XL item, not included) | R12, INT-CTRL, R2, R3, R1; interfaces with R4 and R8 |

**Open questions:**
- Which ranges should the soft warnings name by default (ALIX 70–100, 60, 40–59, 21–39, 61–69; V6 71–83; pendant slots), and what default Program ID should a new upload propose?
- May Lite delete programs (`rm_delete_program_trajectory`), or only overwrite them as the control program does?
- What upload wait per region test is acceptable to operators?
- How should R12 treat the existing files above the joint ratings: re-time, warn or engineer override?
- Should the size check be exact (the one hardware read-back matched exactly) instead of `max(64 B, 0.1 %)`, which can miss about 10 points on a 600 KB file?
- Is automatic background upload after a simulation pass on or off by default?
- Is CANFD streaming wanted later (Linux host, different rate gate, left arm only so far)?

---

### R14 — Offset calculation and testing

**Original (inputs):**
> Offset Calculation and Testing: Users should be able to calculate, apply, and test positional offsets in simulation before executing the adjusted trajectory on the actual robotic arms. - simulate offsets related to 9

**Revised requirement.** Lite shall let the operator calculate an offset (typed, from a real-test observation, or optionally by touch-off), preview it against the taught path, and test it automatically in Lite's 3D simulation and then in controller SIM mode before any real run. Every real run and upload of an offset file shall be gated by the SHA-256 of its exact bytes.

- (a) **Calculate.** The robot cell (commode pose and articulation, bin, UGV, poles) comes from the URDF and ALIX's `commode_c` scene, so no cell calibration is needed (`alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml:33-68`). The offset comes from typed or nudged values, from a correction observed in a real test, or, as an option, from **touch-off**, used when the commode on site is not where the scene says. In touch-off the operator drag-guides the fingertip (`{S}_index_tip_frame`, hand in the SRDF "pointing" state, the same hand and glove state at both captures) to reference marks recorded at teach time, or retrofitted while the robot and toilet are still in the taught placement. Lite records the joints and the measured pole height at rest, applies FK, and fits a translation (1 mark), a translation plus a turn about the vertical axis (2 or more marks; the default), or a full rigid transform (only from 4 or more marks spread at least 300 mm apart). It shows the fit residual and refuses a residual above 3 mm.
- (b) **Preview.** The 3D view shows the original pad path as a grey ghost and the offset path in blue, with reduced spans amber and refused spans red, and draws the toilet moved for toilet-frame offsets. The robot plays the offset motion, with a toggle to show the original arm at the same row. The joint plot overlays original and offset curves on one cursor.
- (c) **World and engines.** Checks run automatically after every change, on the offset motion and on the taught baseline, in the same world: the `commode_c` toilet at the region's lid/seat state (moved by the same transform for a toilet-moved offset); the venue walls from the ALIX venue file selected for the test cell (`alix/ros/orchestration/alix_tasks/config/scene/washroom_*.yaml`, loaded through `plan_scene_validation.load_wall_manager`), always enforced and checked both moved and unmoved, because either the robot was parked differently or the toilet itself moved; the bin per R10; the active arm with its hand; the partner arm on its paired program; and the pole and hand from the workflow. Each arm with its hand is checked against the world per pose, with contact depth, at dt ≤ 0.02 s, and the two arms (hands included) are checked against each other over time.
- (d) **Verdicts.** Only contacts that are new or ≥ 1 mm deeper than the baseline are reported; allowed pad contacts are listed separately. A press check that needs no mesh compares the pad's displacement along its own press axis, relative to the toilet, with the taught track: it fails where the pad is more than 2 mm deeper than requested, and warns "may miss" where it is more than 5 mm shallower on a contact span. A pad-to-surface mesh profile is used only after the toilet model is registered. The R9 gates, the G2 rates after the speed stage, and the bridges and transits (R8) are also checked.
- (e) **Gate ladder.** Records are keyed by the SHA-256 of the exact bytes and follow SIM3D (this offline check) → CTRL-SIM (a controller SIM-mode run of the uploaded program, required by default as in ALIX contract C9) → REAL. A failed CTRL-SIM blocks REAL with no override; a missing one can be overridden only with a typed reason, which is logged. Any edit invalidates the records. The run mode is verified before every run and restored after a CTRL-SIM run, and the real-run confirmation lists the gate chips and the offset summary.
- (f) **First real run.** The first run uses a time-stretched copy (for example 50 %) with its own SHA record: geometry checks carry over because the joint path is unchanged, and the rate and limit gates are re-run. The verdict requires the arm to reach the program's last point. Codes 4105 and 4109 and the operator's verdict are stored, and "Back to offset" returns to editing. Joints are mirrored live once the runner can stream them (R4).

**Findings**
- **ALIX Studio (collision sweep):** The Dual-Arm Collisions sweep (`compute_dual_arm_plan_collisions`) checks the arm links plus the bare palm only, with no fingers, and returns no depth. It would miss hand-to-toilet contact, which is exactly what an offset changes (`alix/core/python/alix_workspace/src/alix_workspace/dual_arm_plan_collision.py:165-182`). The right arm-against-arm engine is `cross_arm_separation` through the `dual_arm_verdicts` shim: "full" link set with hands, dt 0.02 s, 25 mm floor and a swapped-arm guard (`alix/tools/dual_arm_verdicts.py:580-731`).
- **ALIX Studio (`body_collision.py`):** `BodyCollisionWorld` checks the active arm and hand against the partner, bin, covers, UGV, poles, walls and commode at a fixture state, filters out a rest baseline and returns depths in mm, at 3.0-3.4 ms per pose (re-measured). It is validated only for `commode_c`. The commode can only be set to its three states and shifted along +X; there is no general (x, y, z, yaw) placement (`studio/body_collision.py:499-520, 905-925`; `studio/scene_pose.py:123-145`).
- **V6 package (re-measured baseline):** In the model world the taught R2_LIDTOP already touches the closed lid with the right thumb (9 of 188 samples, up to 7.7 mm deep). The default 800 mm venue adds wall hits on R_Link5 (28 of 188, up to 17.2 mm) that vanish at 1400 mm. The regions deck describes commode C, so the scene is ALIX's `commode_c`, whose default pose is [1.005, 0.0, 0.42] m from `base_link` at a 0.25 m standoff; whether the toilet stood exactly there at teach time has not been checked (regions-deck FK review; `alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml:33-44`). Verdicts must therefore be differential, and the primary contact check must not need a toilet mesh (`lite/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/RIGHT/R2_LIDTOP.txt`).
- **alix_dispatch:** ALIX contract C9 keeps an append-only ladder keyed by the program fingerprint, where SIM is the controller's own SIM mode on hardware. A failed SIM blocks REAL with no override; a missing SIM may be overridden only with a recorded reason. A controller left in SIM makes every dispatch "succeed with nothing moving" (`alix/core/python/alix_dispatch/src/alix_dispatch/ladder.py:117-145`; `alix/core/python/alix_dispatch/src/alix_dispatch/runmode.py:248-253`).
- **ALIX findings log:** The run speed % scales only the transit to the first point, not the program, and completion and a limit stop look the same in the run state, so a test verdict must check how far the program got (`alix/plan/FINDINGS.md:5854-5864`). The slowest track overshot most, so a slow run does not reduce joint overshoot (`alix/plan/FINDINGS.md:6721-6737`).
- **Control app v53/v14:** The runner already records and saves drag-teach motion (usable for touch-off), reads the pole through `pole.py` and switches SIM/REAL. It reads joints only between steps and has no joint stream during runs, so a live mirror needs a new runner output. 4105 (speed over limit) is warning-only; 4109 (collision) stops (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:56-57, 912-990`). Plan-blocking: v53 Multi Run and Page-01 Load Sequence raise `TypeError` (missing `@staticmethod`) (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:4853, 4876`).
- **ALIX Studio (UI):** `editor.js` provides robot geometry and posing, the collision highlight with Prev/Next and log, `atlasTubeRibbon` (exact point-to-station mapping with per-point colour; it sits in X2 atlas code, so copy it with `atlasRotateAbout`), gate chips and `showStudioModal`. Only the coverage chip knows about edits, so invalidation is new work (`studio/web_editor/editor.js:11235-11299, 15760-15790`).
- **Touch-off parts:** `{S}_index_tip_frame` sits at the tip of the fully opened index finger, matching the SRDF "pointing" hand state, and a 6-line Kabsch fit exists (`alix/core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf:4537-4556`; `alix/tools/track_report.py:683-688`). A 4-DOF (x, y, z, yaw) fit suits a robot and toilet that both stand on the floor; a 6-DOF fit from 3 hand-placed marks would add false tilt (about 0.6°, or 5 mm at 0.5 m, with ±3 mm noise over 300 mm). `alix_perception_core` has no fixture pose estimate.

**Can it be borrowed from ALIX Studio?** **Partial** — The Studio supplies the 3D viewport, robot model, path tubes, collision highlight and log, gate chips and confirm dialog. Its collision sweep is the wrong engine for offsets (no fingers, no depth); the right engines, `BodyCollisionWorld` and `cross_arm_separation`, are in code the Studio already uses. Optional touch-off, the moved-toilet world with enforced venue walls, differential verdicts, the press check and the SHA-keyed ladder with a controller-SIM rung must be built.
- *Other sources:*
  - `BodyCollisionWorld` (`studio/body_collision.py`) — Borrow + adapt: a synthetic task config, `pole_override_m`, the partner state from the workflow, a new general transform on the fixture root and walls, the selected ALIX venue file with walls enforced, and one worker process.
  - `cross_arm_separation` + `alix/tools/dual_arm_verdicts.py` — Borrow as-is: the shim and the sweep, with pole and hand seeded in the timeline, dt 0.02 s and the 25 mm floor.
  - `compute_dual_arm_plan_collisions` — Reference only: no fingers and no depth; its frames can drive collision playback.
  - `alix_dispatch` (`ladder.py`, `runmode.py`) — Borrow + adapt: rungs SIM3D → CTRL-SIM → REAL keyed by the output SHA-256 plus its workflow context, the ledger in Lite's data directory, run-mode verify and restore.
  - Control app v53/v14 — Partial: real run and stop, the SIM/REAL switch, drag-teach capture and the pole read exist; a joint stream during runs and a one-click joint read are missing.
  - Trajectory prototype — Borrow + adapt: plot cursor and shapes for the original/offset overlay, linked to the 3D scrub.
  - Emulator — Not applicable as evidence: it has no collision model.
  - `alix_perception_core` — Not applicable: no fixture pose estimate.
- *Note on the inputs:* Accurate as a dependency. "Simulate offsets related to 9" correctly ties R14 to R9, and the input makes no borrow claim. "Simulation" needs a definition: Lite's offline 3D and collision check, followed by the controller-SIM rung that ALIX's contract C9 puts before REAL (it proves the controller accepts and completes the exact bytes, but it is not collision evidence). ALIX Studio cannot simulate a `.txt` today, and its collision sweep would miss hand contacts.

**Recommendation.** Build R14 as the mandatory test half of R9, on the shared Lite simulation stack (R3/R5/R10). Use `BodyCollisionWorld` for each arm and hand against the toilet, walls and bin (with depths) and `cross_arm_separation` for arm against arm (with hands), and keep the Studio's highlight UI, tubes, gate chips and confirm dialog. Make the mesh-free press check along the pad axis the primary contact gate. Take the cell from the URDF and the `commode_c` scene with the selected venue walls enforced; calculate a toilet-moved offset by typed values or a real-test observation, and offer touch-off (4-DOF fit, measured pole) as an option when the commode is not where the scene says. Gate every real run and upload by the output SHA-256 through SIM3D → CTRL-SIM → REAL, run a time-stretched copy first, and prove the run reached its last point.

**Approach**
1. **Touch-off, optional** (build; capture borrowed). Only when the commode is not where the scene says and the offset is not known. Capture each mark with the pointing hand and the same glove state: a one-click joint read (a new runner verb) or a short drag-teach recording (`--drag-session` / `--drag-save`, median of the still tail), plus a pole read at rest through `pole.py`. At a new placement, capture the same marks, FK both sets at `{S}_index_tip_frame`, and fit translation, 4-DOF or 6-DOF by mark count, extending the Kabsch helper. Show the result in plain words with the residual; refuse above 3 mm.
2. **Instant preview** (borrow + adapt). Before solving, draw the original path (grey ghost) and the requested path (blue) with a copied `atlasTubeRibbon`, and draw the toilet moved for toilet-frame offsets.
3. **Solve and play** (R9; prototype cursor borrowed). Run R9 in a worker, recolour the offset tube per point, and compute FK frames for both motions. The robot plays the offset; "show original arm" swaps in the taught frames at the same row; the joint plot overlays dashed original and solid offset curves on one cursor.
4. **World check** (borrow + adapt `BodyCollisionWorld`). Build baseline and offset timelines with `dual_arm_verdicts.build_timeline` (pole and hand seeded). For each arm, run the pose filter and depths on every 4th point (dt ≤ 0.02 s), with the fixture at the region state and posed by M, the selected venue walls moved and unmoved, the partner state from the workflow and the bin per R10. Diff against the baseline.
5. **Arm against arm** (borrow). Run `dual_arm_verdicts.sweep` on the offset pair and the baseline pair, and flag a new contact or a minimum below 25 mm. After a dual-arm "toilet moved" batch, repeat for every paired step.
6. **Press check** (build). Project the pad's displacement relative to the toilet (offset FK minus M · taught FK) on the taught pad press axis. Fail where it is more than 2 mm deeper than requested; warn where it is more than 5 mm shallower on a contact span. Add a mesh profile only after the optional toilet registration (fit the commode pose to the taught pad points, or touch it off).
7. **Gates and SIM3D record** (chips borrowed; invalidation built). Show limits, steps, accuracy (R9), G2 after the speed stage, collisions, press and bridges/transits (R8) as chips that go stale on any edit. On a pass, append {sha256, rung SIM3D, gates, user, time, tool version} to the ledger.
8. **CTRL-SIM** (borrow + adapt `ladder.py` and `runmode.py`). Upload the exact bytes, set SIMULATION, run, confirm the last point was reached, set REAL and verify, then record the result. A failure blocks REAL.
9. **Real test** (build). The confirmation dialog lists the chips and the offset summary. Run the stretched copy (R2 scale stage) under its own record, check completion, record 4105/4109 and the operator's verdict, then allow full speed. "Back to offset" returns to the offset panel.
10. **Performance** (build). Cache sweeps by SHA-256 and run collision work in one worker process, because the collision managers are not thread-safe.

**Acceptance criteria**
- [ ] The world check always includes the selected venue walls; a run with no venue selected is NOT CHECKED, and a synthetic offset that pushes R_Link5 into a wall is reported. Known-good recordings (for example R2_LIDTOP) show no wall contact with the venue chosen for the test cell.
- [ ] Touch-off (optional), from recorded joint sets: a known 10 mm shift is recovered within 0.5 mm; with 2 marks at least 300 mm apart, a known 2° turn is recovered within 0.3°; a residual above 3 mm is refused with a plain message; the fit uses the measured pole.
- [ ] Preview: both paths are drawn, with reduced spans amber and refused spans red; the moved toilet is shown; scrubbing the plot cursor moves the 3D robot to the same row.
- [ ] The world check runs at dt ≤ 0.02 s per arm with the hand included. It lists only contacts that are new or ≥ 1 mm deeper than the baseline (allowed pad contacts listed separately), and it detects a deliberately injected 20 mm press into the lid.
- [ ] The arm-against-arm sweep (hands included) is re-run on the offset pair and reports any new contact or a minimum below 25 mm. After a dual-arm "toilet moved" batch it is re-run for every paired step and summarised in one table.
- [ ] The press check flags every span deeper than requested by more than 2 mm, with a "Show me" link.
- [ ] "Test on real robot" and "Upload" stay disabled until the exact SHA-256 has a SIM3D pass and a CTRL-SIM pass (or a logged override for a missing CTRL-SIM). A failed CTRL-SIM cannot be overridden, any edit re-locks, and the run mode is checked before every run.
- [ ] The first real run uses a stretched copy with its own passing record. Its verdict requires reaching the last point, and 4105/4109 and the operator's verdict are stored.

**For operators:**
- One guided flow for the main case: "The toilet is not where it was when we taught" → Enter the shift (or, optionally, measure it by touching the marks) → Preview → Check → Test slowly → Upload. Touch-off shows a picture of each mark, one "Capture" button per mark and a live residual.
- Plain verdicts with "Show me" buttons, for example: "No new collisions. The pad keeps the same pressure on 96 % of the path. 2 places may press up to 3 mm harder."
- A greyed-out button always says why: "Run the simulation check first", "You changed the offset after the last check" or "Controller test (no motion) not done yet".
- A large SIM/REAL badge whenever a robot action is possible, and the same 3D colours everywhere: grey original, blue offset, amber reduced, red refused or colliding.

**Safety:**
- A pass licenses only the exact bytes; the slow copy has its own record. Lite verifies and restores the run mode around every robot action, because a controller left in SIM silently moves nothing.
- The run speed % scales only the transit, and a slow run does not reduce joint overshoot, so the verdict must confirm the last point was reached. Lite treats 4109 (which stops the run) and 4105 (warning-only in the runner) as a failed test.
- A press-check depth more than 2 mm beyond the request blocks; a collision-world contact ≥ 1 mm deeper than the baseline is a warning that needs explicit acceptance.
- Touch-off is done in drag mode at low speed; never move the pole or hand while the fingertip touches the toilet. The collision world must match the cell: use the cubicle that matches the cell (1400 mm by default, 800 mm selectable) and verify it with known-good recordings (the 800 mm walls gave false wall hits on R2_LIDTOP in the wider cell); the commode pose comes from the `commode_c` scene unless an offset says otherwise; and the bin model is 40 mm shorter than the physical one.
- The IO emergency-stop buttons on each arm controller are the primary stop during the first real run of an offset file; Lite shows their state before the run.

| Priority | Size | Depends on |
|---|---|---|
| Should (ships with R9) | L (on top of the shared R3/R5/R10 stack; optional touch-off and toilet registration: M) | R9, R3, R5, R10, R7, R8, R4, R13, R2, INT-CTRL; URDF + `commode_c` scene + selected venue walls |

**Open questions:**
- If touch-off is wanted: are there, or can there be, physical reference marks on each toilet? Who records them at teach time, and can marks be retrofitted for the existing V6 programs?
- Is touch-off done gloved or ungloved, and can the gloved hand take the "pointing" state?
- Was the toilet at the `commode_c` scene pose when V6 was taught?
- Should the controller SIM rung be mandatory, as ALIX C9 requires (recommended), and who may override a missing one?
- Are 2 mm (press check) and 1 mm (deeper contact) the right thresholds?
- Where is the gate ledger kept, and who reviews it?

---

## 8. Scope exclusions (X1, X2)

### X1 — Exclude geometric path generation and editing

**Original (inputs):**
> What is ALIX Studio Lite - It is a light weight version of the ALix Studio (/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen) that uses the drag and teach generated plans, that are optimized (smoothed and speed increased) since they are recorded at slow speed and with a number of stops, with th e following functionality not include,
> 1. the geometric path autogeneration and editing since we start in joint space (taught .txt plans)

**Revised requirement.** Lite shall edit taught joint-space recordings only. It shall not contain, ship, load or expose ALIX Studio's geometric (Cartesian) path generation or path editing.

- (a) **Screens.** No excluded tab, panel, control or overlay (listed below) appears in Lite.
- (b) **Routes.** None of the 35 excluded routes exists in Lite.
- (c) **Code.** No excluded module is imported at run time or included in the release artifact.
- (d) **Separate code base and write boundary.** Lite is its own code base in `alix_studio_lite` (`/home/newtonjeri/alix_studio_lite_ws/alix_studio_lite`), not a package inside the ALIX monorepo and not a trimmed `alix_taskgen`. It imports the kept ALIX libraries from the vendored ALIX copy in `alix_studio_lite/vendor/` (F7). Lite never writes into the ALIX checkout. All outputs go to the Lite project folder (NFR-DATA).
- (e) **Enforcement.** Automated tests enforce (a)-(d): an import guard, a route allow-list and a release-content check. The Keep/Adapt/Drop list of Studio parts is kept in the design document, signed off by the requester and reviewed at each milestone.
- (f) **Help text.** The user help says: "Lite does not create new cleaning paths. To change a path, teach the region again or (engineer) apply an offset."

**What is excluded**
- *Tabs and panels:*
  - Workbench, both halves: the Generate Task dock (Target, Start pose, Pad & rows, Stroke & turn, Spline, ψ / C-space, Output, Generate / Cancel / Reattach, knot edits) and the point editor (Editor Mode, Selected Point, Start / Pre-start Pose, Sequence, Analyze & Finish, Reference Frame) (`studio/web_editor/template.html:204-526`).
  - Regions-tab mesh painting: Paint / Erase brush, Import / Export OBJ, Combine Regions, part-state selects, region rename and delete (`studio/web_editor/template.html:156-201`).
  - Gizmo tab: Orientation Gizmo and IK Frame Calibration (`studio/web_editor/template.html:675-695`).
  - Schedule & Run tab and its back end. It previews and edits the ROS orchestrator's schedule CSV and imports `alix_orchestrator` from the ROS tree (`studio/server.py:4706-4730`).
- *Controls and overlays:* the Board's Generate missing, Raster sweep and coverage and validation metrics; the footer's Mirror Arm, Mirror + Reverse and Points/deltas; the overlays Points, Path, Spline, Frames, Normals, Gizmo, Start gizmo, start EE and task frame (`studio/web_editor/template.html:698-741`).
- *Routes (35):* `/api/generate`, `/api/auto-generate`, `/api/auto-path`, `/api/auto-path-reauthor`, `/api/auto-path-status`, `/api/auto-path-cancel`, `/api/start-pose-check`, `/api/path-compare`, `/api/bulk-generate`, `/api/bulk-generate-status`, `/api/bulk-generate-config`, `/api/bulk-generate-cancel`, `/api/bulk-generate-apply`, `/api/coverage`, `/api/mirror`, `/api/retarget`, `/api/validate`, `/api/verify-save`, `/api/save`, `/api/deploy-path`, `/api/generate-mechanism`, `/api/mechanism-ops`, `/api/mechanism-exemplar`, `/api/save-regions-obj`, `/api/update-region`, `/api/regions`, `/api/analyze-points`, `/api/analyze-config`, `/api/animate-sequence`, `/api/fixture-board`, `/api/board-cell`, `/api/save-ik-frames`, `/api/save-gizmo`, `/api/gizmos`, `/api/gizmo` (route table `studio/server.py:5961-6029`).
- *Modules:* `alix_taskgen.server` and the generator stack (`scripts.generate_cleaning_config`, `scripts.visualize_generated_config`, `studio_generate`, `studio_bulk`, `auto_generate`, `cspace_*`, `path_*`, `path_planners/*`, `stroke_gate`, `geometric_path`, `coverage`, `coverable_map`, `mirror`, `validator`, `yaml_assembler`, `mechanism_assembler`, `entry_planner`, `tilt_*`), plus `robot_model`, which pulls in `coverage` and `alix_workspace.reachability`. The stack is large (`cspace_author` alone is 10,999 lines), and importing the server loads it, which is why the Studio takes 30-60 s to start (`studio/server.py:34-44`).
- *Write targets:* the ALIX task-config, plan and schedule trees, `auto-generated_config`, `Resource/plans`, and the Studio's `.cache` folders inside the repo (`studio/server.py:4154, 5153`).

**What stays**
- Joint-space edits of taught recordings: trim, smooth, scale and merge (R2, R8, R11).
- Small Cartesian offsets on an existing taught track by FK → offset → IK on the kept `alix_workspace` engine (R9, R14). They change a taught path; they create no new path.
- Read-only display of the robot, the commode and a pad (TCP) trail computed by FK from the joint track. The trail is how Lite shows "missed spots" (R3), because ALIX's coverage tools are X1: `alix_workspace.coverage` has no coverage maths of its own and imports `alix_taskgen.coverage` (`alix/core/python/alix_workspace/src/alix_workspace/coverage.py:1-8`).
- Dual-Arm Collisions playback and the Simulate view (R3, R5, R10).
- From Schedule & Run: the proportional timeline, the transport controls and the "guards as sentences / stale is not pass" rules, fed by Lite's own workflow steps (program, pole, hand, wait) instead of the orchestrator CSV (R8, R10) (`studio/web_editor/editor.js:13601-13676`).
- From the rest of the Studio: the theme and shell, the robot overlay, the Board table UI (R1, with a new back end), the pole vocabulary and the pole/bin foul-band warning, which moves to Lite's pole controls (R7) (`studio/web_editor/editor.js:17144-17186`).
- Allowed ALIX helpers, taken from the vendored ALIX copy, with provenance (source path, ALIX commit, sha256) and with their repo lookups replaced by explicit Lite asset paths: `body_collision`, `scene_pose`, `fixture_parts`, `transforms`, `paths`, `constants`, `glove_frames` (with its YAML), `sim_bridge` (emulator view only), `schedule_guards` (atomic write, guard sentences) and `singularity`. They generate no paths.
- Reuse is by copied code or re-implemented behaviour, never by importing the Studio server. Even the "reusable" robot routes pull in X1/X2 code: the IK context imports `alix_workspace.reachability` (`studio/robot_model.py:170-181`), and `/api/manifest` is built in a script that imports X1 modules at load (`studio/scripts/visualize_generated_config.py:13-20`).
- The copied collision world takes generated task configs and fixes the bin at LOWERED (`studio/body_collision.py:499-530, 627-631`). Lite adds a taught-track entry point that takes the declared bin state (R3, NFR-SAFETY). The commode_c fixture, bin, UGV and poles come from the URDF and the commode_c scene (`alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml`); Lite adds the venue walls, which are not enforced today, through the kept `plan_scene_validation.load_wall_manager` / `PlanWallChecker` and the venue scene files in `alix/ros/orchestration/alix_tasks/config/scene/` (F3, F5; `alix/core/python/alix_workspace/src/alix_workspace/plan_scene_validation.py:117, 179`). Wall checking is collision checking on taught tracks, not path generation, so it is not excluded by X1.

**Can it be borrowed from ALIX Studio?** **Not applicable** — this is an exclusion. The parts Lite keeps are listed under *What stays*.
- *Note on the inputs:* Partly accurate. "A light weight version of the ALix Studio (alix_taskgen)" holds for the look, the vocabulary and selected components, but Lite should not be a trimmed `alix_taskgen`: importing its server loads the generator; Studio tests pin rules Lite must break (no numbered tabs; "the studio has no execution channel") (`alix/core/python/alix_taskgen/test/test_studio_interaction_guards.py:1207-1224, 2207-2215`); and its resources resolve relative to the repo. "We start in joint space (taught .txt plans)" is accurate (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:14-15`). The exclusion must not be read as excluding the R9/R14 offsets.

**Acceptance criteria**
- [ ] After every Lite route has been called in a test session, `sys.modules` contains no deny-listed module: `alix_taskgen.server`, `alix_taskgen.robot_model`, `scripts.*`, `studio_*`, `auto_generate`, `path_*`, `path_planners*`, `stroke_gate`, `cspace_*`, `coverage`, `coverable_map`, `geometric_path`, `mirror`, `validator`, `yaml_assembler`.
- [ ] Lite's route table contains none of the 35 excluded routes (allow-list test).
- [ ] The release artifact contains no `cspace_author.py`, `studio_generate.py`, `stroke_gate.py`, `path_emission.py`, `robot_model.py`, `scripts/*` or `path_planners/*`.
- [ ] An automated DOM scan of every Lite screen finds no excluded element id. The signed UI checklist shows no Paint/Erase, Import/Export OBJ, Generate, Mirror, knot or waypoint editing.
- [ ] Any Lite API write to a path outside the project folder fails and writes nothing; in particular nothing is written into `vendor/` (checked against `vendor/MANIFEST.json` after the test session).
- [ ] Lite's source tree under `alix_studio_lite` contains no copy of the excluded modules, and Lite runs from the vendored ALIX copy in `alix_studio_lite/vendor/` without modifying either.
- [ ] The R9/R14 offset integration tests and the R3 pad-trail display pass with the import guard active.

| Priority | Size | Depends on |
|---|---|---|
| Must | S (the scene manifest, IK context and collision entry point are sized under R3 and R9) | F7 (separate `alix_studio_lite` folder, vendored ALIX copy), NFR-OFFLINE-DEPLOY, NFR-DATA, R9, R14, R3, R5, R10, R8, R1 |

**Open questions:**
- Is any waypoint-level Cartesian editing wanted beyond the R9 offsets? It would conflict with X1.
- Should Lite show regions on the 3D commode (read-only labelled meshes) or only the deck pictures?
- How should missed spots (R3) be judged: by eye from the pad trail, or with a new simple contact-coverage estimate on the commode mesh?
- Please confirm that the Schedule & Run tab is excluded and that its timeline and transport are reused.

---

### X2 — Exclude C-space authoring and Workspace analysis

**Original (inputs):**
> What is ALIX Studio Lite - It is a light weight version of the ALix Studio (/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen) that uses the drag and teach generated plans, that are optimized (smoothed and speed increased) since they are recorded at slow speed and with a number of stops, with th e following functionality not include,
> 2. the C SPace and Workspace features

**Revised requirement.** Lite shall not contain, ship, load or expose ALIX Studio's C-space authoring and Workspace-analysis features. It shall keep the kinematics and collision engine underneath them, used only inside Lite operations.

- (a) **Screens.** No Workspace tab, atlas, reachability or manipulability view appears in Lite, and the 15 `--atlas-*` style tokens are dropped.
- (b) **Routes.** None of the 9 excluded routes exists. There is no `/api/ik` route and no per-point IK panel; IK runs only inside the R9/R14 offset operations.
- (c) **Code and data.** No X2 module is imported at run time or packaged, and no `reach_*.npz` or `manip_*.npz` cache ships.
- (d) **Engine packaging.** Lite lives in its own folder `alix_studio_lite`, not in the ALIX monorepo, and imports the kept engine modules from the vendored ALIX copy in `alix_studio_lite/vendor/` (F7). Decided 13 Oct: the vendored copy, a filtered subset in ALIX's own layout, so Lite needs no ALIX checkout at run time and does not load X2 code at start-up (`vendor/PATCHES.md`). It must work on the operator PC (x86-64 Linux) and on the production Jetson (AGX Orin, aarch64, Ubuntu 24.04) (F1).
- (e) **Answers, not maps.** Operators get yes/no answers from the engine ("This offset can be followed without jumps: Yes"), never voxel maps, heatmaps or atlas views.

**What is excluded**
- *Screens and controls:* the Workspace tab (Workspace Analysis, Kinematics Solver, Redundancy Atlas, Selected Point — Analysis) (`studio/web_editor/template.html:556-597`); Check Arm Reachability on the Regions tab and the ψ / C-space group of the Generate dock (`studio/web_editor/template.html:188, 327`); the overlays Reach voxels, Manip heatmap, Safety zones, Unreachable, Atlas band, Ring cage and Reach dome; the atlas "Pole m" input.
- *Routes (9):* `/api/reachability`, `/api/region-reachability`, `/api/reachable-mask`, `/api/manipulability`, `/api/redundancy-atlas`, `/api/atlas-pose`, `/api/atlas-sweep`, `/api/atlas-environment`, `/api/path-ik`. The last one is built on `stroke_gate` (X1) and atlas poses (X2), so R9 calls the kept IK engine directly (`studio/studio_generate.py:4243-4262`).
- *`alix_taskgen` code:* `cspace_author` (10,999 lines), `cspace_pair`, `sdk_reach` and the Studio's atlas handlers.
- *`alix_workspace` modules:* the X2 features `reachability`, `manipulability`, `redundancy`, `swivel_arcs`, `placement`, `contact` (atlas contact relaxation) and `safety` (safety-zone overlay); modules Lite does not need (`sensor_model`, `mtc_feasibility`, `fixture_validator`, `sparse_targets`, `cli`); and the X1-coupled `coverage`.
- *Data and styles:* the `reach_*.npz` and `manip_*.npz` caches, which share `workspace_data/` with the kept visual overlay (`alix/core/python/alix_workspace/pyproject.toml:44-45`), and the 15 `--atlas-*` tokens (`studio/web_editor/editor.css:79-93, 169-183`).

**What stays**
- *Kept engine modules (not X2):* `urdf_kinematics`, `ik`, `srs`, `srs_ik`, `collision`, `dual_arm_plan_collision`, `plan_scene_validation`, `plan_analysis`, `srdf_states`, `package_index` and `config_loader`, used by R3, R5, R8, R9, R10, R12 and R14. `plan_scene_validation` also carries the wall checker (`load_wall_manager`, `PlanWallChecker`) that Lite uses to enforce the venue walls (F5; `alix/core/python/alix_workspace/src/alix_workspace/plan_scene_validation.py:117, 179`). `config_loader` is on the list because `plan_analysis` needs it (`alix/core/python/alix_workspace/src/alix_workspace/plan_analysis.py:21-22`).
- *Optional helpers (kinematic warnings, not X2),* adopted if R3/R9/R12 need them: `clearance` (robot-to-fixture distance), `wipe_kinematics` and `studio/singularity.py` (RealMan singularity screens). The Studio server never imports `clearance` or `wipe_kinematics`; they are engine helpers, not Workspace-tab features.
- *All collision geometry:* bin, covers, poles and hand links from the URDF and the commode_c scene, plus the venue wall mesh (F3, F5). X2 removes views, not bodies. Joint-limit, continuity and collision checks stay in the kept engine and must still run before real motion (NFR-SAFETY).
- *Lite's own joint-space edits* (trim, smooth, scale). "C-space" in X2 means ALIX's C-space path-authoring pipeline, not editing joint values.
- *Why a filtered bundle.* The kept engine is not cleanly separable today:
  - The package init eagerly imports `ReachabilityAnalyzer`, so importing `urdf_kinematics` loads `reachability` (measured: 0.18-0.33 s) (`alix/core/python/alix_workspace/src/alix_workspace/__init__.py:8`).
  - `plan_scene_validation` finds the repo root at import and raises "repo root not found" outside the repo unless `ALIX_REPO_ROOT` is set (measured) (`alix/core/python/alix_workspace/src/alix_workspace/plan_scene_validation.py:31-59`).
  - The Studio's IK context imports `reachability._placement`, a 6-line transform that Lite inlines (`studio/robot_model.py:179`; `alix/core/python/alix_workspace/src/alix_workspace/reachability.py:47-52`).
  - With the bundle the import guard is strict. With the installed package it allows `alix_workspace.reachability` only as the package-init side effect. Two optional upstream patches can be proposed to the ALIX maintainers: a lazy package init and an import-safe repo root in `plan_scene_validation`.

**Can it be borrowed from ALIX Studio?** **Not applicable** — this is an exclusion. The engine underneath is borrowed with adaptation as a library (see *What stays*); building the bundle is sized under NFR-OFFLINE-DEPLOY.
- *Note on the inputs:* Accurate as an exclusion, with two clarifications. Exclude the features, not the FK/IK/collision engine: Lite needs it for the R9 offsets and the R3/R5/R10 simulation. The vendored copy in `alix_studio_lite/vendor/` provides that engine without the repo, and a one-line package patch keeps X2 code from loading at start-up.

**Acceptance criteria**
- [ ] Bundle option: after every route has been called, `sys.modules` contains none of `reachability`, `manipulability`, `redundancy`, `swivel_arcs`, `placement`, `contact`, `safety`, `cspace_*` or `sdk_reach`. Installed option: only `alix_workspace.reachability` is present, loaded by the package init, and a spy test shows none of its functions is called.
- [ ] None of the 9 routes exists, and there is no `/api/ik` route.
- [ ] The release contains no `reach_*.npz` or `manip_*.npz`.
- [ ] No DOM element exists for the Workspace tab, the atlas, or the reach and manipulability overlays.
- [ ] The R9 offset and R3/R5 simulation integration tests pass with the guard active.
- [ ] Bundle option: Lite's kinematics and collision modules, including the wall checker, import with no ALIX checkout and no `ALIX_*` variables, with no "repo root not found".
- [ ] The same import-guard and engine tests pass on x86-64 Linux and on aarch64 (Jetson AGX Orin, Ubuntu 24.04).

| Priority | Size | Depends on |
|---|---|---|
| Must | S (building the bundle is sized under NFR-OFFLINE-DEPLOY) | R9, R14, R3, R5, R10, NFR-OFFLINE-DEPLOY (bundle filter, aarch64 build), NFR-THEME (atlas tokens) |

**Open questions:**
- Who refreshes the vendored ALIX copy (`tools/vendor_from_alix.py`) when ALIX changes, and how often?
- The kept engine uses `trimesh`, whose collision manager needs `python-fcl`, and `collision.py` imports `fast_simplification` (`alix/core/python/alix_workspace/src/alix_workspace/collision.py:111, 354`; `plan_scene_validation.py:121`). Are aarch64 builds of these available for Ubuntu 24.04 on the Jetson? To be checked.
- Please confirm that no visual reachability maps are wanted. R9 keeps a per-point "can it reach" check inside the engine.

---

## 9. Non-functional requirements

NFR-UX and NFR-THEME come from the inputs. NFR-SAFETY, NFR-OFFLINE-DEPLOY and NFR-DATA are **recommended additions**: the analysis showed that Lite cannot meet the other requirements safely without them.

### NFR-UX — Very user-friendly for non-technical operators

**Original (inputs):**
> the lite version should be very user friendly (we are targetting non technical users)

This comes from the project brief, not the inputs file. The closest text in the inputs file is "(non technical person) user" in R1 (`docs/reference/ALIX_Studio_Lite_requirement_inputs.txt:8`).

**Revised requirement.** Lite shall let a non-technical operator complete the operator journey (Teach → Optimise → Simulate → Test on robot → Upload & Run) for a region without engineering help. Every screen follows the rules below; the per-feature screens are specified under R1-R14.

- (a) **Guided flow.** Each region is one five-step stepper with the states locked, to do, done and needs attention. Back is always available, and every step has a "What to do now" sentence. The Regions dashboard is the home screen and offers "Next region to teach". Operators can open any region from the dashboard. There is no forced tutorial.
- (b) **Plain language.** Operator screens show seconds (not row numbers), degrees (not millidegrees), "Left arm" / "Right arm" and the deck's region names. They never show SDK, runner, slot, rc, Folder, NDJSON, counts/sample, MoveJ/MoveL, TOGETHER / RUN FIRST / WAIT OTHER, ψ or C-space. The controller program store is shown as "Robot program number (1-100)", the control app's own range, not as "slot" or "program ID" (F6; `lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:9, 4235, 4279`). A glossary, a searchable "?" help and a tooltip on every control are one click away. Errors state what happened, what to do next (with a button where possible) and offer Details. Controller codes are translated, for example 4109 "Arm collision detected — the arm stopped" and 4105 "A joint moved faster than allowed (warning)", and the translations are checked against RealMan's error list and on the bench.
- (c) **Presets and roles.** Optimisation is one click: Auto-optimise to the region's target time (R2), with named Smoothness presets (Off / Light / Standard, R11). The default optimiser is the trajectory prototype's trim idle → smooth → scale; the time-optimal optimiser is the second choice, which operators and engineers can both select (decided 13 Oct), labelled "Time-optimal (TOPP-RA)" as in INT-TRAJ (b), with the plain explanation "fastest within the limits" (F2; `lite/Alix_Studio_Lite_Trajectory_V2.html`). Pole and hand use named per-region presets. Numeric parameters, program management, IP settings and raw logs sit behind "Advanced" in Engineer mode, with disclosure at most two levels deep. Operator mode is the default, and the current mode is always shown.
- (d) **Safe defaults, confirmations and undo.** Lite starts in Simulation, with no arm armed and the approach speed at the lowest preset; every optimisation preset stays within the joint ratings. Only robot motion (including pole and hand moves) and controller overwrites are confirmed. The confirmation names the arm(s), the mode (SIMULATION / REAL), the controller IP, the region or program and the checks passed; Cancel has default focus and Enter does not confirm. An always-visible STOP is shown whenever any arm is connected (NFR-SAFETY). The hardware E-stop is the primary safety stop (F4): each arm card and the app bar show the E-stop state of the left and right arm controllers as colour + glyph + word, using the chip states of NFR-SAFETY (c) ("E-stop: clear", "E-STOP PRESSED", "E-stop released: recovery needed", "E-stop: unknown" when the state cannot be read, "E-stop: not configured"), and while either chip is not clear every motion button is disabled with the reason and a step-by-step recovery guide. Edits are never confirmed: each region offers at least 5 labelled undo steps (Ctrl+Z / Ctrl+Y), and robot actions are clearly stated as not undoable.
- (e) **Visual rules.** Status is shown as colour + glyph + word, never colour alone. Each screen has exactly one primary (filled) action. Targets are at least 24 px; primary and frequent controls at least 44 px, and larger under coarse pointers. All text, chart text included, is at least 14 px. Screens are usable at 1366×768 without horizontal scrolling.
- (f) **Progress and saving.** Nothing is shown for waits under 1 s, a spinner with words for 1-10 s, and a determinate "x of n" with time remaining above 10 s. Long jobs can be cancelled. Work is saved automatically, and leaving warns only when something is unsaved.
- (g) **Target time in plain words.** The Optimise screen shows, per region, the operator's target time, the achieved time and, when the target cannot be met, the closest time that keeps within the limits and the limit that sets it, in one sentence, for example: "Target 15 s. Fastest safe time is 18 s. Limited by: right arm, joint 4 speed." It offers "Use 18 s" and "Change target". Meeting the target shows "Target 15 s reached (15.0 s)." (F2.)
- (h) **Usability gate.** A usability test with non-technical operators passes before release (see the acceptance criteria).

**Findings**
- **ALIX Studio:** There is no guided flow, by design. Tabs are "grouped by purpose, not numbered", a test forbids numbered tabs, and the screen is a dense seven-tab ribbon with expert tooltips, although ALIX's design system targets "robot operators, not CAD professionals" (`studio/web_editor/template.html:115-125`; `alix/core/python/alix_taskgen/test/test_studio_interaction_guards.py:1207-1224`).
- **ALIX Studio:** A reusable feedback toolkit exists: a status line, an error toast that stays 12 s, a busy state, global error capture (worded "(see console)"), the `{error, hint}` convention, tooltips with keyboard parity, a searchable "?" help built from them, and gate chips and Board cells that pair a glyph with a word (`studio/web_editor/editor.js:5431-5467, 7946-8040`).
- **ALIX Studio:** Three parts need adapting. The confirmation modal focuses the primary button on open, which is wrong for motion (`studio/web_editor/editor.js:13780-13832`). Undo is one global, unlabelled snapshot stack (cap 80) of Studio editor state, so the pattern is reusable but the code is not. The 24/44 px target tokens exist, but 44 px is used only for Generate actions and there is no coarse-pointer rule (`studio/web_editor/editor.css:127-128, 1828`).
- **ALIX tools:** `alix/tools/layout_audit.py` drives headless Chromium (Playwright) and already enforces target size, the type floor, no clipped controls and band height, reading its thresholds from the stylesheet (`alix/tools/layout_audit.py:1-40`).
- **ALIX UX research:** It gives rules Lite can adopt: an operator screen separate from the programmer view, simulation by default with a separately armed hardware mode, disclosure at most two levels deep, rare confirmations with undo preferred, plain-language errors, and wait-time thresholds (`alix/docs/studio/STUDIO_UIUX_RESEARCH.md:378-380, 793-796`).
- **Trajectory prototype:** It has a good six-step stepper with lock and done states and Back/Next. But it puts engineer parameters on the surface, shows row numbers and raw millidegrees, and saves through browser downloads (`lite/Alix_Studio_Lite_Trajectory_V2.html:329-341, 609-623`).
- **Control app v53/v14:** A Tkinter console with two arm consoles, a dual-arm sequence page, 87 buttons and technical labels; each action runs the runner as a subprocess. Most motion is confirmed with E-stop reminders, but manual hand and pole apply are not (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:4167, 4192-4203`).
- **alix_dispatch:** The controller's SIM mode runs on the real controller ("SIM and REAL both require hardware access"), so a SIM-mode usability task needs the bench controller or the ALIX emulator (`alix/core/python/alix_dispatch/src/alix_dispatch/runmode.py:1-8`).

**Can it be borrowed from ALIX Studio?** **Partial** — Lite can lift the Studio's feedback toolkit, confirmation and multi-choice modals, tooltips and "?" help, gate chips, pills, Board table and tab badge, the undo *pattern*, the UX research rules and the layout audit. Its screen design (a CAD-style seven-tab ribbon, expert wording, no guided flow) does not suit operators. The stepper, Operator/Engineer modes, presets, the plain-language message catalogue, the STOP bar, coarse-pointer sizing and the usability test are new.
- *Other sources:*
  - Trajectory prototype — Borrow + adapt: the stepper skeleton and the status/hint idioms, restyled with ALIX tokens and generalised to the five Lite steps.
  - Control app v53/v14 — Partial: reuse its functions and its confirmation wording with E-stop reminders, not its density or jargon.
  - Operator subset of the control-app functions (from the Operations notes review) — Borrow as-is: arm cards, guided teach, pole/hand presets, read-only mode badge, plain checkpoints, STOP, time left, named recipes.
- *Note on the inputs:* There is no borrow claim in this requirement. The implicit assumption that ALIX Studio's UI already suits non-technical users is partly accurate: its components and rules do; its screen design does not.

**Recommendation.** Build Lite around one guided operator journey with a dashboard home. Reuse ALIX Studio's feedback components, UX rules and layout audit. Keep every number behind presets and Engineer mode. Make release depend on a measured usability test.

**Approach**
1. **Task lists** (build). Write the operator and engineer task lists from the control-app operator subset and the operator journey.
2. **Wireframes** (build). Draw the dashboard and the five step screens at 1366×768, using the ALIX design-system cards (`alix/design/components/*.html`) as reference. Review them with the requester and one operator.
3. **Shell and stepper** (borrow + adapt). Build the app bar (brand, mode banner, role chip, per-arm E-stop indicator, STOP), the left step rail, the content area and the status line from lifted ALIX components and the prototype's stepper.
4. **Message catalogue** (build). Put every string in one file, tagged operator or engineer, with an error template, a controller-code table (4105, 4109, the upload return codes, connection errors), the target-time sentences of (g) with one wording per binding limit, the E-stop states and recovery steps of (d), and a blocked-terms lint test.
5. **Presets and Engineer mode** (build). Add the presets and a PIN-gated Engineer mode, and move every numeric parameter behind Advanced.
6. **Confirmations** (adapt). Wrap the Studio modal as `confirmMotion()`: Cancel focused, a plain question as title, chips for the arm, mode, IP and checks passed, and an E-stop reminder. Keep an inventory of which actions confirm.
7. **Undo and auto-save** (build on the Studio pattern). Wire per-region labelled undo with R2 and auto-save with NFR-DATA.
8. **Automated UI checks** (adapt). Point ALIX's layout audit at the Lite pages with the 14 px floor, and add the DOM checks to CI.
9. **Usability test** (build). Pilot with 2 people and fix. Run the formal test with at least 5 people, fix severity 3-4 issues, then re-test.

**Acceptance criteria**
- [ ] A usability test with ≥ 5 non-technical participants who have not used the ALIX tools or the control app covers six tasks: (1) find the untaught regions; (2) teach one region with the right pole/hand preset; (3) optimise with a preset and undo one change; (4) run the simulation and say whether and when a collision happens; (5) run a region in the controller's SIM mode (emulator, or bench controller with an engineer at the E-stop); (6) stop a running arm. Pass levels: each task completed unaided by ≥ 4 of 5 participants; zero critical errors (unintended motion, wrong arm, wrong program uploaded); STOP pressed within 3 s of the prompt in every trial; mean SUS ≥ 70.
- [ ] No operator-tagged string contains a term blocked in (b) (automated lint).
- [ ] The adapted layout audit passes on every screen at 1366×768 and 1920×1080: no clipped control, no text below 14 px, all interactive elements ≥ 24×24 px, primary and STOP ≥ 44 px, exactly one visible `button.primary`, a glyph and a text label on every status element, and no horizontal scroll.
- [ ] Every motion or overwrite action calls `confirmMotion()`, and no edit action opens a dialog (inventory test).
- [ ] Every catalogue error has a non-empty "what to do", and codes 4105 and 4109 map to plain sentences.
- [ ] Undo offers ≥ 5 labelled steps per region, and Ctrl+Z restores the previous recipe and plot within 1 s.
- [ ] Start-up lands on the dashboard, in Simulation, in Operator role.
- [ ] For a target shorter than the feasible minimum, the Optimise screen shows the target, the closest feasible time and the binding limit in one plain sentence, with "Use <time>" and "Change target"; for a feasible target it shows the achieved time within the stated tolerance. No term blocked in (b) appears in either message.
- [ ] With an arm controller's E-stop input active (bench or emulator), the E-stop state for that arm changes to "E-STOP PRESSED" within 1 s, all motion buttons are disabled with the reason, and the recovery guide is shown; after release the state changes to "E-stop released: recovery needed" and motion stays disabled until the recovery wizard of NFR-SAFETY (c) is completed. When the state cannot be read, the indicator says "unknown", never "clear".

**For operators:**
- Operator words throughout: "Teach", "Clean the seat top (left half) for 15 s", "Check in simulation", "Try on the robot", "Send to robot". Every region card shows the deck picture with the region highlighted.
- The one primary button is always in the same place: bottom right of the step panel. No number appears without its unit, and times show as m:ss.
- The Optimise screen speaks in times, not parameters: "Target 15 s — fastest safe time 18 s".
- The E-stop indicator for each arm is always on screen. When it shows pressed, the screen says what to do: "Make the area safe, release the red E-stop, then press Recover."
- Dialog titles are plain questions ("Run the LEFT arm on the real robot?"), facts go in a short list, and buttons are named after the action ("Run LEFT arm", "Cancel").
- Empty states teach: "No recording yet — press Start teaching".

**Safety:**
- Confirmations stay rare (motion and overwrites only) and specific (arm and mode named). The confirm button never has default focus.
- STOP never asks for confirmation and is never hidden behind a dialog or scrolled away. The on-screen STOP is a software stop; the red hardware E-stop is the primary emergency stop, and the screen says so.
- The E-stop indicator never shows "clear" from stale or missing data; an unreadable state is shown as unknown and blocks motion.
- Engineer mode is never the default and is always visibly shown, because it exposes speeds and overrides.

| Priority | Size | Depends on |
|---|---|---|
| Must | L (cross-cutting framework and one usability-test cycle; per-feature screens are sized under R1-R14) | NFR-SAFETY, NFR-THEME, NFR-DATA, R1, R2, R4, INT-TRAJ, INT-CTRL |

**Open questions:**
- Who are the operators (language, reading level, robot experience)? Is English the only language?
- Mouse and keyboard, or touchscreen? What is the screen size of the operator PC (x86-64 Linux) now, and how will operators reach Lite in production on the Jetson (AGX Orin): a display attached to the robot, or a browser on another device? The layout audit sizes must cover both.
- Which IO port on each arm controller carries the E-stop (mode 5, "input emergency stop function multiplexing") and with which level for "pressed"? Lite reads it with `rm_get_io_state` on the mode-5 IO (`rm_get_io_input` may return -1 for that port; E-stop research §2); the port, the level and whether the read works while the E-stop is held are confirmed in drill DR1-DR2 (NFR-SAFETY).
- Which of the recovery steps in NFR-SAFETY (c) (release, clear errors, re-enable joints) these controllers actually need after an E-stop is measured in drill DR11; the guide's wording follows that result.
- Who may switch to Engineer mode, and how (PIN, OS account)?
- Can 5 operators be scheduled for a usability test, and on what setup (emulator or bench)?

---

### NFR-THEME — Keep the ALIX Studio theme and formatting

**Original (inputs):**
> the lite version should also maintain the theme and formatting used in the Alix studio

This comes from the project brief, not the inputs file.

**Revised requirement.** Lite shall render every screen, including the ported trajectory editor and the robot-control functions, in the ALIX Studio visual language, from one shared token source. "Theme" means the colour, type, spacing and shape tokens plus component styling; "formatting" means the layout conventions (sentence-case labels, label-left fields, mono numbers, 4 px radius, tonal elevation, status vocabulary), not the CAD-style ribbon or command band.

- (a) **One token file.** `alix_tokens.css` is copied from ALIX `design/_tokens.css` with identical values (light default; dark via `[data-theme="dark"]`). It adds the `editor.css` tokens Lite needs (`--accent`, `--accent-soft`, `--brand-seam`, `--surface-bar`, `--surface-dock`, `--surface-ribbon`) and a documented "Lite additions" block. A parity test fails on drift. No colour, font, radius or shadow literal appears outside the token file.
- (b) **Type, brand and icons.** Inter for UI text and JetBrains Mono for numbers, IDs and paths, shipped as local woff2 files with their OFL licences. Type scale 14 / 16 / 17 / 19 / 22 px, hierarchy by weight, nothing below 14 px (charts included). The inline-SVG butterfli mark and the title "ALIX Studio Lite" in the app bar. ALIX's inline-SVG line icons (`currentColor`) and the state glyphs ✓ △ ✗ · ⓘ.
- (c) **Components.** ALIX base controls and shell components, lifted with the known defects fixed.
- (d) **Colour discipline (ISA-101).** About 90 % of the screen is neutral. Brand blue is used only for selection and the primary action. Green, amber and red mean state only, always with a glyph or word; STOP and a pressed hardware E-stop use the danger colour, always with words. The arm identity colours (deck: blue LEFT, orange RIGHT) are their own tokens, visibly different from brand blue `#2F96EE` and warning amber `#C77700`, and always shown with the words LEFT / RIGHT.
- (e) **Charts.** A `plotTheme()` helper builds every Plotly layout from tokens in both themes. A documented seven-colour joint palette avoids the state red, green and amber and the brand blue. Collision and limit markers use a state colour plus a text label.
- (f) **Other palettes retired.** The trajectory prototype's palette (accent `#175cd3`, dark header `#0b1220`, 8-12 px radii), the Tk app's neon palette (`#06111F`, `#00E5FF`, …) and the Studio's leftover JS literals (old cyan `#00d4ff`, GitHub-dark hues) are not used. Their meanings map to ALIX tokens. Any Tk tool kept during the transition takes generated constants from the same token source.
- (g) **Accessibility and switching.** WCAG AA contrast in both themes. Where a shared ALIX token fails, Lite adds an accessible variant token instead of silently changing the shared value. Light is the default, with a user toggle for dark that is remembered per browser; the 3D scene and the chart colours update on toggle.

**Findings**
- **ALIX Studio:** The token system is clean and test-pinned: light `:root` and dark `:root[data-theme="dark"]` tokens, a type scale, the authored source `design/_tokens.css`, parity tests, the brand `--brand #2F96EE` measured from butterfli.ai and an inline-SVG mark (`studio/web_editor/editor.css:1-129`; `alix/design/_tokens.css:8-55`).
- **ALIX Studio:** Three things are missing. Dark mode is not wired: there is no `data-theme` setter and no `prefers-color-scheme` handling, and `syncColorsFromCss()` runs once at start-up (`studio/web_editor/editor.js:185, 311`). Webfonts are not shipped (no `@font-face`) and Inter and JetBrains Mono are not installed on this PC. `--accent` and `--accent-soft` exist only in `editor.css`, although the primary-button and modal rules use them (`studio/web_editor/editor.css:39-40`).
- **ALIX Studio:** Most CSS defects sit in X1/X2 rules that Lite drops anyway (the Generate dock and atlas rules). The ones that matter for Lite: the `.info-row` block nests the rules that follow it, `.pill-ok` and `button.primary` use colour literals, and `html, body` restates the font stack (`studio/web_editor/editor.css:1016-1027, 847-852`).
- **ALIX Studio:** JS colour literals bypass the theme and pass ALIX's guard, which checks only `#00d4ff` in CSS and `0x00d4ff` in JS. The schedule timeline uses the old cyan and GitHub-dark hues; both Plotly charts hard-code the dark-theme text colour `#e6edf3` (1.18:1 on white, invisible in the default theme) at 10 px (`studio/web_editor/editor.js:13593-13594, 14483-14491`; `alix/core/python/alix_taskgen/test/test_studio_theme.py:86-89`).
- **Contrast (computed):** White on `--brand` is 3.13:1 (primary buttons) and white on the dark-theme `--brand-strong` hover is 2.52:1. `--ok` text is 3.41:1, `--warning` text 3.46:1, and `--muted` on `--surface-inset` 4.37:1. AA candidates: `#1A75C4` (4.79:1) or `#1872BF` (5.00:1) for the primary fill, `#9A5A00` (5.47:1) for warning text, `#1F7A45` (5.346:1) for ok text (`studio/web_editor/editor.css:22-44, 135-155`).
- **Regions deck:** The arm colours (blue LEFT, orange RIGHT) sit close to brand blue (reserved for selection) and warning amber (reserved for state). Used as-is, they would blur "selected" and "warning" (`lite/Toilet_regions_V1.pptx`; `alix/design/_tokens.css:10, 20`).
- **Trajectory prototype:** It has its own palette and a dark header, fixed plot heights of 600/520 px, category10 joint colours (J3 green, J4 red) and Plotly from a CDN. It reuses ALIX token *names* with different values, so its CSS must be namespaced (`lite/Alix_Studio_Lite_Trajectory_V2.html:7, 9-21`).
- **Control app v53/v14 and Operations notes:** The control app uses a neon Tk palette. The Operations notes ask for a "dark robotics / control-console theme" (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:56-71`; `lite/RealMan Dual-Arm Operations UI.txt:227-237`).

**Can it be borrowed from ALIX Studio?** **Borrow + adapt** — Lift the tokens (light and dark, plus `--accent` / `--accent-soft`), the brand mark, the icon set, the base and shell components and the theme guard tests. Filtering out the X1/X2 rules and atlas tokens removes most defects; Lite fixes the rest, replaces the JS colour literals, and adds local webfonts, `plotTheme()` with a joint palette, accessible and arm-identity tokens, and a working dark toggle that re-syncs the 3D and chart colours. Do not lift the Fusion-style command band.
- *Other sources:*
  - ALIX design system (`alix/design/_tokens.css` and the component cards) — Borrow as-is: the single source of the token values.
  - Trajectory prototype — Not applicable: its palette is retired; its screens are restyled under INT-TRAJ.
  - Control-app Tk palette — Not applicable: retired; only its colour meanings are kept.
- *Note on the inputs:* Accurate and feasible, with caveats. "The theme" is in practice the light theme: dark exists but cannot be switched on. Some components, especially the JS charts and the timeline, carry off-theme literals that ALIX's own guard misses. "Formatting" should mean the visual language, not the CAD ribbon. And the requirement conflicts with the Operations notes' dark console theme unless the ALIX dark variant is offered.

**Recommendation.** Copy the ALIX token file and components rather than re-creating them, and keep one token source for everything Lite renders. Fix the few defects that survive the X1/X2 filtering and replace every JS colour literal. Add what the Studio lacks: local fonts, a chart theme helper, distinct arm-identity colours and a working dark toggle.

**Approach**
1. **Tokens** (borrow). Copy `alix/design/_tokens.css` into `alix_studio_lite/web/theme/alix_tokens.css` (the Lite code folder, F7) without the `.ds*` rules, and append the needed `editor.css`-only tokens and the "Lite additions" block. The source file has mode 600, so the copy is made world-readable.
2. **Guard tests** (borrow + adapt). Port `test_design_tokens.py` to compare Lite's file with the vendored `vendor/alix/design/_tokens.css` (its sha256 is in `vendor/MANIFEST.json`). Add a literal scanner for Lite's CSS **and** JS (hex, `rgb()`, `rgba()`).
3. **Base and shell CSS** (adapt). Lift the base and shell rules, dropping the X1/X2 rules and the atlas tokens. Fix the `.info-row` nesting, `.pill-ok`, the body font and the `#fff` literal. Add `button.danger` and `.hint.error`.
4. **Fonts** (build). Vendor Inter and JetBrains Mono woff2 with `@font-face` and the OFL licence files.
5. **Charts** (build). Write `plotTheme()` (font ≥ 14 px, tick and grid colours from tokens) and the joint palette. Convert every chart (the ported trajectory plots, the collision timeline) and the schedule-timeline colours. Add contrast tests.
6. **Ported screens** (build). Restyle the trajectory and control screens: remove their own `:root` palettes, namespace their classes (`.traj-*`) and map their roles to ALIX tokens. Generate `alix_tokens.py` for any Python/Tk output.
7. **Theme toggle** (build). Add an app-bar toggle that stores `alixLite.theme` in browser storage (inside try/catch) and re-runs `syncColorsFromCss()` for the 3D scene and the charts.
8. **Review.** Check contrast and identity distinctness in both themes. Review with the requester, and with the ALIX design owner if a shared token should change.

**Acceptance criteria**
- [ ] The token parity test passes against `alix/design/_tokens.css` (or the recorded snapshot hash).
- [ ] A scan of Lite's CSS and JS finds no hex, `rgb()` or `rgba()` colour literal outside `alix_tokens.css` and `plotTheme()` (SVG path data allow-listed).
- [ ] With the network off and the fonts not installed, body text renders in the bundled Inter (`document.fonts.check` passes).
- [ ] In every Plotly chart, tick labels are ≥ 14 px and reach ≥ 4.5:1 contrast against the plot background, in light and dark.
- [ ] Switching theme updates the panels, the 3D scene and the charts without a reload.
- [ ] All text-on-fill pairs used by buttons (including hover) and status pills reach ≥ 4.5:1 in both themes (automated table test).
- [ ] The LEFT and RIGHT identity colours differ from brand blue and warning amber by a documented minimum (for example ΔE ≥ 20) and are always shown with their words.
- [ ] Side-by-side review: Lite and ALIX Studio share the app bar, brand mark, buttons, pills and fonts (signed off by the requester).

**For operators:**
- Light by default (ALIX research: dense text reads better on light); dark is offered for control-room use.
- The 3D viewport stays dark in both themes (ALIX rule), so the robot and the commode stand out.
- STOP is the only large red element on the screen; the E-stop indicator turns red only while an E-stop is pressed.

**Safety:**
- Red is reserved for problems and STOP, so a red mark always means "look here". Joint lines are never red.
- The REAL-mode banner does not rely on colour alone: it shows the words "REAL ROBOT" and an icon.
- An arm's identity colour must never be confused with "warning" or "selected"; the wrong-arm risk is reduced further by always naming the arm.

| Priority | Size | Depends on |
|---|---|---|
| Must | M (the full restyle of the trajectory editor is sized under INT-TRAJ) | NFR-OFFLINE-DEPLOY (local fonts and libraries), NFR-UX, INT-TRAJ, INT-CTRL, R5, R6, R8, R10, R12 |

**Open questions:**
- Default theme: ALIX light (research) or dark (Operations notes)? The same default applies on the operator PC and, in production, on the Jetson display.
- May Lite add accessible variant tokens, or should `design/_tokens.css` change for both products? Changing a shared value also changes ALIX Studio and its parity test.
- Arm identity colours: keep the deck's blue/orange (with words), or adopt distinct ALIX identity hues?
- Is the product name and badge text "ALIX Studio Lite"?

---

### NFR-SAFETY — Safe robot operation

**Original (inputs):**
> (new — recommended)

The inputs require real-robot runs (R4), uploads (R13) and pole and hand moves (R7), but they state no safety rules.

**Revised requirement.** Lite shall move a robot (arm, pole or hand) only when the operator clearly intends it and no E-stop is engaged, shall show the state of the hardware E-stop on each arm, shall always offer a software stop, and shall refuse programs that have not passed its checks. The **primary safety stop is the hardware E-stop**: the buttons wired to an IO input on each arm controller (LEFT and RIGHT), which stop the robot without cutting power (user decision F4). Lite's STOP is a secondary, controlled software stop and a convenience; it does not replace the E-stop or the site risk assessment.

- (a) **Safe start and arming.** Lite starts in Simulation with no arm armed. REAL mode is armed per arm and per session, after a connection check, a run-mode read, an E-stop state of CLEAR on both arms (see (c)) and an explicit confirmation. While an arm is armed, the app bar shows "REAL ROBOT — LEFT 192.168.1.10" as word, icon and colour. Arming lapses after a configurable idle time and on disconnect. On session end or crash recovery, Lite restores the run mode, tool frame and speed settings it changed.
- (b) **One command source, one gateway.** While an arm is armed, Lite's back end is its only command source, through one long-lived connection per arm; the operator confirms that the ROS driver, the Tk control app and the vendor web UI are not driving it. Every motion command (program run, sequence, move-to-start, drag-teach start, playback, pole move, hand move, global-point move) passes one server-side gateway. The gateway checks arming, the E-stop state, interlocks and gates, checks that the connection's arm matches the region's arm, requires a single-use confirmation token bound to that exact command, and logs the command. E-stop status polling uses the same connection owner, so it never opens a second competing connection.
- (c) **Hardware E-stop: state, blocking, interruption and recovery.**
  - *State.* Lite shows one E-stop chip per arm: CLEAR, E-STOP ENGAGED, RECOVERING (released, not yet recovered), UNKNOWN (no fresh reading for more than 1 s, or controller unreachable) or NOT CONFIGURED (IO3 is not in E-stop mode on that controller). The primary source is `rm_get_io_state(n)` on **IO3**, the E-stop input on both controllers (decided 13 Oct), which must be in mode 5, "emergency stop function multiplexing"; IO1-IO4 are also read at connect to report the full IO map; it is cross-checked with the joint enable flags and, if the site enables Modbus-TCP, input register 302 "Emergency Stop Status". The state is never derived from `arm_current_status == 9` alone, because that value stays set after any stop. Polling runs at 5-10 Hz per arm. The active level is a per-arm site setting (default HIGH, per the RealMan demo) confirmed by the drill. The E-stop state holds from the press until the button is released, which is the manual reset; Lite's chip follows the same rule and then shows RECOVERING until the recovery steps are done.
  - *Fail closed.* UNKNOWN and NOT CONFIGURED block motion exactly like ENGAGED (a supervisor override for NOT CONFIGURED is allowed in Simulation only).
  - *Blocking.* While either arm is not CLEAR, the server refuses, on both arms: Run, Sequence, Test-on-robot, Move-to-start, Pole, Hand, Drag-teach START and the switch to REAL. Both arms are blocked because one E-stop chain has been seen to cover both arms. Uploads to any slot stay allowed, with a "robot in E-stop" note, because an upload moves nothing.
  - *Interruption.* If an E-stop is seen during a run, sequence or teach, Lite marks the run "INTERRUPTED — E-STOP" (never "Complete"), cancels the rest of the sequence (no next program, no hand or pole step), sends pole stop (`rm_set_lift_speed(0)`) and hand freeze on both sides, because the E-stop has been seen not to stop the hand, and logs IO state, enable flags, power, brake state, error codes and run state with timestamps.
  - *Runner fix.* While runner v14 is kept as the robot back end, it is patched to read the E-stop IO, refuse to start while the E-stop is held, report "interrupted" instead of "RUN COMPLETE", never start the next sequence step after an E-stop, and print parseable `ESTOP ENGAGED <side> io=<n>` / `ESTOP CLEAR <side>` lines.
  - *Guided recovery.* A wizard, per arm or both: (1) confirm the cell is safe; (2) release the button and wait for the released level (chip goes to RECOVERING, never straight to CLEAR); (3) show the controller state in plain words; (4) clear errors with `rm_clear_system_err` and `rm_set_joint_clear_err`, at most two rounds, reading back each time; (5) re-enable joints with `rm_set_joint_en_state` with the wrist supported if any are disabled, otherwise show the pendant or power-cycle instructions; (6) restore the global speed; (7) collision-check the controller's move from the current pose to the program's first point against the full checked world of (e) including the walls, and show its size; (8) re-home the pole. An interrupted trajectory is never resumed automatically; the operator restarts the region or a chosen sequence step on purpose.
  - *Configuration.* Lite never writes IO modes. At connect it checks that IO3 is in mode 5 on both controllers and blocks motion if not: RealMan's demo maps IO3 to "continue program" (mode 4) and IO4 to the E-stop (`alix/external/butterfli-realman-alix/Demo/RMDemo_Python/RMDemo_IOControl/readme.md:68`), so a controller left in that mapping would resume a program instead of stopping it. The site config stores, per arm, the E-stop IO number, its active level and the date of the last passed drill; Lite shows "E-stop drill overdue" after a configurable number of days.
- (d) **Software stop (secondary).** STOP ALL (both arms, poles and hands: arm slow stop, lift speed 0, hand freeze, sequence cancel) and a per-arm STOP are always visible, take one click, need no confirmation and work during any job and when idle. They are labelled "STOP (software)" next to the text "The hardware E-stop is the safety stop". STOP goes through the connection that owns the arm and reaches the controller within 300 ms of the click. A stop path that does not depend on the browser exists on the server or runner side, with a stop-on-disconnect watchdog. Lite then reads the program run state and the joint motion to confirm the stop; if the stop is not confirmed within 1 s, it shows "Press the hardware E-STOP now". If the bench test shows that slow stop does not end a running stored program, STOP uses the verb that is proven to.
- (e) **Gates before real motion.** A trajectory runs on the real robot only if its exact bytes (sha256) passed (1) file checks: format, 7 integers per line, joint limits with margin, ≤ 10° per line and joint speeds within ratings at the controller's fastest 5.0 ms tick; and (2) a 3D simulation with collision checking against the checked world: the robot from the URDF (both arms, hands, poles, UGV), the commode_c scene in the region's fixture state, the **declared bin state**, the pole height, the **venue walls**, and the controller's implicit approach move from the arm's measured pose to the first point. Wall collision is not enforced in ALIX today; Lite enforces it, using the venue scene file selected for the site (see (h)). Any edit makes earlier passes stale. An engineer override needs a typed reason and is logged.
- (f) **Interlocks, speed and approach.** An arm reserved by a running job cannot be commanded from any other screen. Pole moves are refused inside the stowed-bin foul band (0.153-0.282 m with the bin STOWED; provisional until re-measured on the physical bin). The pole moves only between programs with the arm at rest, and its arrival is judged with a settle margin. Pole and hand move only with the pad off the surface (the operator confirms). No motion while a blocking controller error is active (4105 is a warning only). Only one arm is in drag-teach at a time unless dual teach is chosen. Transit (approach) speed defaults to ≤ 30 %. The approach-move size is shown before a run; above a site threshold the run is refused until the arm has been moved to a checked start pose. The controller's global speed is restored to the site default after every run and on every reconnect.
- (g) **Run modes and dual-arm runs.** The controller's SIM mode is described as "controller dry run — the arm does not move; pole and hand are not simulated". In SIM runs Lite skips pole and hand commands, because the real pole and hand would move, and says so. A SIM run never counts as a robot-test pass. An unreadable run mode, or arms in different modes, refuses the run. If one arm fails or stops in a dual-arm run, the other arm is stopped by default. A stop made outside Lite is reported as "Stopped outside Lite", and an E-stop as "INTERRUPTED — E-STOP", never as "Complete". The 3D verdict lists what was checked (arms, commode state, bin, walls with the venue file name, hands).
- (h) **Network, programs, venue and records.** The server listens on 127.0.0.1 by default. Every state-changing request needs the session token in a custom header and must pass Host and Origin checks, because binding to localhost alone does not stop another web page in the operator's browser from posting to Lite. Other interfaces (for example a browser on the operator PC reaching Lite on the Jetson, NFR-OFFLINE-DEPLOY) require configured authentication. Uploads may go to **any program ID 1-100**, as in the control app. Uploading to an occupied slot replaces the program only after a confirmation that lists what the slot holds (name, size, and the ledger entry if any); Lite may add warnings, for example "this slot holds a program from the V6 workflow", but does not block. A run by program ID requires the slot's listed name and size to match Lite's upload-ledger entry for a passed sha256. The venue wall file is chosen once per site in Engineer settings from the ALIX venue scenes and verified against known-good recordings before REAL use; the choice is recorded with every simulation pass. Every motion, confirmation, stop, E-stop event, recovery step, error code, override and run mode (SIM/REAL) goes into the project run log with time, role, arm, controller IP, program or region, and sha256.

The 300 ms, 30 %, the approach threshold, the polling rate, the drill interval and the idle time are proposed defaults, to be confirmed on the bench.

**Findings**
- **RealMan documentation (IO E-stop).** Short names: HW, TP, JSON, CPP, PY, DEMO, MB, REL, CN = the Gen-3 manuals in `alix/docs/Manuals/Realman/` (`RealMan_Gen3_02_RM75_Hardware_and_Getting_Started.pdf`, `..._03_Teach_Pendant_and_Configuration.pdf`, `..._04_JSON_Protocol.pdf`, `..._05_API2_C_Cpp.pdf`, `..._06_API2_Python.pdf`, `..._08_Demos_C_and_Python.pdf`, `..._09_Modbus_FAQ_Downloads.pdf`, `..._01_Overview_and_Release_Notes.pdf`) and the Chinese user manual V1.2.0; RRI = `alix/external/butterfli-realman-alix/Python/Robotic_Arm/rm_robot_interface.py`.
  - Each Gen-3 controller's 16-core IO cable has four multiplexable inputs IO1-IO4 (pins 5-8). Each can be set to mode 5, "input emergency stop function multiplexing", on the pendant or with `set_IO_mode` / `rm_set_io_mode` (HW p.39-40; TP p.112, 131; JSON p.118; RRI:2882). The inputs are NPN and trigger on a HIGH level (DEMO p.82). The Chinese manual lists pin 14 = E_STOP, while the English Gen-3 page lists it as reserved (CN p.24; HW p.39).
  - The docs do not say what the IO E-stop stops, and give no stop category, safety rating or reaction time. The pendant E-stop stops the arm "at maximum speed" (TP p.11); `set_arm_stop` is a maximum-deceleration stop that keeps power on and cannot be resumed (JSON p.61; PY p.84); API2 code -6 groups IO, pendant and JSON E-stops as one external stop, and applies only to blocking motion calls (CPP p.318; RRI:1568). The IO E-stop is therefore very likely the controller's quick stop with power kept, which matches F4. RealMan advises connecting safety devices to safety interfaces rather than normal IO (HW p.32).
  - Detection: `rm_get_io_state(n)` returns the mode and level of a port (JSON p.120; RRI:2927); `rm_get_io_input` returns -1 for a port not in input mode, so it may not show a mode-5 port (PY p.160; RRI:2961). Modbus input register 302 "Emergency Stop Status" exists since V1.6.7, clears on release, and needs Modbus-TCP enabled on the pendant (MB p.3, 8; REL p.14; TP p.125). There is no E-stop entry in the system-error table or the joint-error bitmask.
  - Software stops: `rm_set_arm_stop` (quick stop, power kept) and `rm_set_arm_slow_stop` (along the path) (JSON p.61-62; RRI:2069, 2084). `rm_set_arm_emergency_stop` is Gen-4 only and returns -4 on Gen-3 (RRI:7226-7240). `rm_clear_system_err`: "If not cleared manually, the error will remain until a new motion command is issued" (JSON p.93; RRI:2427). RM75 joints 1-4 have a hard brake and 5-7 a soft brake, with no brake inside J5-J7 (HW p.3, 34). Full notes: `research_estop.md` in the analysis folder.
- **ALIX field notes and code (E-stop).** During the 2026-08-06 E-stop both controllers rejected pole commands (ret 1), one E-stop chain covered both arms, and end-port power survived so the hand kept working; the documented recovery ladder is release → clear errors → lift panel → power cycle (`alix/docs/TEST_PROCEDURE.md:216-242, 453`). ALIX code assumes a physical E-stop drops joint power and the enable flags, and observed that `arm_current_status` 9 (RM_STOP_E) stays set after any stop and is not cleared by `clear_system_err` (`alix/ros/hw/alix_hw/README.md:449`; `alix/ros/hw/alix_hw/src/rm_side_system.cpp:1855-1860, 1919-1935`). The "drops joint power" assumption conflicts with F4 ("no power cut") and must be measured. `rm_clear_system_err` can return 0 without clearing joint code 16384 (`alix/core/python/alix_dispatch/src/alix_dispatch/program.py:236-240`). Software arm stops do not stop the pole, which kept moving for up to 2.6 s (`alix/core/python/alix_dispatch/src/alix_dispatch/peripherals.py:189-199`).
- **Runner v14 (E-stop).** It watches only `run_state` and error codes (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:122-138, 219-271, 641-700`). An E-stop that ends a program without an error code would very likely print "RUN COMPLETE … no error" and exit 0, and a sequence would start its next step, including hand steps (inferred; to be confirmed by drill DR3 and DR10).
- **ALIX Studio:** The Studio never commands hardware ("HONEST: the studio has no execution channel"), and a test pins that. Its server binds to 127.0.0.1 but has no Origin, Host or token check, which is fine for an authoring tool but not for motion endpoints. Its useful patterns are the confirmation modal, stale-on-edit, honest preview labels and the foul-band warning text (`studio/web_editor/editor.js:13045-13047`; `studio/server.py:6510-6544`).
- **Control app v53/v14:** SLOW STOP sends SIGINT to the runner's process group and the runner calls `rm_set_arm_slow_stop`; with no live runner the button does nothing to the robot. The runner refuses an empty slot, an arm error or a running program, names 4105 (warning) and 4109 (arm collision), and restores the global speed after runs. Manual hand and pole apply are not confirmed, and manual pole accepts 0-315 mm with no bin check (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:6028-6071, 4192-4203`; `lite/upload_channel/rm_upload_fast_v14_safe_transit.py:26-35`).
- **Control app v53/v14 (slots):** Uploads take any Program ID 1-100 (default 25); upload-only confirms that an occupied slot "will be replaced"; batch upload pre-fills per-file Program IDs, checks duplicates and the 1-100 range, lists the controller's programs and asks Yes/No/Cancel per file; stored programs run by ID or name (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:760-968, 2170-2262, 4265-4291, 4330-4692, 4776-4842`). The runner refuses IDs outside 1-100 (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:324-325`). Existing slot conventions (25 / 99 / 40-59 / 70-100 / 71-83) are habits, not controller rules (`alix/tools/taught_tracks.py:29-32`).
- **ALIX motion docs:** ALIX has ruled "one command source per arm" ("Two TCP clients = two command sources — not advisable. ACCEPTED AS AXIOM."). The SDK has no program-scoped stop or pause, only motion-scoped stop, slow stop (on-path) and pause (`alix/docs/motion/DRIVER_TRACK_CHANNEL_ARCHITECTURE.md:24-28, 220-233`).
- **alix_dispatch (run mode, session):** The controller's SIM mode runs on hardware and simulates neither the pole nor the hands, so a SIM run must skip those commands. An arm left in SIM is a "phantom" that still scores complete; unreadable or mixed modes are refused. Session teardown restores the tool frame, run mode and speed limits (`alix/core/python/alix_dispatch/src/alix_dispatch/session.py:1-8, 217-265`; `alix/core/python/alix_dispatch/src/alix_dispatch/runmode.py:178-260`).
- **alix_dispatch (track, peripherals):** Ready-made guards exist: `global_speed_guard`, `start_gap_deg` (size of the controller's implicit approach move), `assert_side` (wrong-arm refusal) and `slow_stop`. A lift command landing while a planned arm move is in flight aborts that move and latches joint errors, and the lift state leads the physical pole by up to 73 mm (`alix/core/python/alix_dispatch/src/alix_dispatch/track.py:161-200, 323-350`; `alix/core/python/alix_dispatch/src/alix_dispatch/peripherals.py:18-24`). The emulator supports both "aborts" and "ignores" stop policies, because whether a stop ends a stored program is unproven on hardware (`alix/core/python/alix_dispatch/src/alix_dispatch/emulated_program.py:51-56`).
- **Orchestrator, V6 package and teach table:** With the bin STOWED, the arm base fouls the bin lid covers for pole 0.158-0.277 m (0.153-0.282 m with margin), and "NO ARM MOTION AVOIDS THIS". Both V6 FINAL workflows start at pole 200 mm, and the commode-C teach table uses 140 mm and 200 mm. The band was measured on the shorter BinShort model; the physical bin has not been remade (`alix/ros/orchestration/alix_orchestrator/alix_orchestrator/core/task_config.py:58-124`; `alix/ros/description/ugv_description/urdf/bin.xacro:32-34`).
- **Collision code and scene:** The URDF and the commode_c scene already hold the cell (commode pose and articulation, bin geometry, UGV, poles) (`alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml`). Both ALIX checkers hard-code the bin as LOWERED. The dual-arm sweep checks no commode, and `BodyCollisionWorld` takes generated task configs. A "checked world" gate therefore needs a bin-state parameter and a taught-track entry point (`alix/core/python/alix_workspace/src/alix_workspace/dual_arm_plan_collision.py:596-611`; `studio/body_collision.py:499-530, 627-631`). Walls exist as venue meshes in `alix/ros/orchestration/alix_tasks/config/scene/` (single cubicles only for 800 mm and 1000 mm; the 1400 mm venue is the whole washroom); `load_wall_manager` loads them and `PlanWallChecker` sweeps a plan coarse-then-fine with depth, but only in the plan-generation gate and CLI, not on taught tracks. The default venue is 800 mm (`alix/core/python/alix_workspace/src/alix_workspace/plan_scene_validation.py:62-70, 117-155, 179-256`). With that default, a taught right-arm lid track touches a wall on 28 of 188 sampled rows, while the 1000 mm and 1400 mm venues give 0, so Lite uses a single 1400 mm cubicle by default, with 800 mm selectable (decision D21), and verifies it once with known-good recordings.
- **ALIX taught lane and runner v14:** ALIX binds certificates to the bytes by sha256 and re-checks them at upload and run. The runner verifies an upload by stored size only (± max(64 B, 0.1 %)), so only a local ledger can tie a slot to a sha256 (`alix/tools/taught_tracks.py:29-32`; `lite/upload_channel/rm_upload_fast_v14_safe_transit.py:147-175`).

**Can it be borrowed from ALIX Studio?** **Partial** — The Studio contributes UI and process patterns only: the confirmation modal, stale-on-edit, honest labels, the foul-band warning text and the loopback default. It never commands hardware, so arming, the gateway, the E-stop chip and recovery, STOP, interlocks, run-mode handling, speed restore and dual-arm handling come from the control app, `alix_dispatch`, `alix_hw` field knowledge and new work.
- *Other sources:*
  - Control app v53 + runner v14 — Borrow + adapt: SIGINT → slow stop, refusal checks, speed restore, REAL confirmation, E-stop wording, the 1-100 slot model with replace confirmation and batch upload; add the E-stop read and "interrupted" verdict to the runner; close the hand/pole confirmation and interlock gaps.
  - Helpers `usage.py` / `pole.py` / `hand.py` — Borrow as-is: `stop_all` and the rule "the pole moves only with the arm still; never move pole or hand with the pad on a surface".
  - `alix_dispatch` (`runmode`, `session`, `track`, `peripherals`, `verdict`, `program`) — Borrow + adapt: imported from the vendored ALIX copy (it has no `pyproject.toml`, so it is vendored into the bundle).
  - `alix_hw` E-stop recovery sequence (`alix/ros/hw/alix_hw/src/rm_side_system.cpp:1919-1935`) and the ALIX recovery ladder (`alix/docs/TEST_PROCEDURE.md:240-242`) — Borrow + adapt as the recovery wizard steps.
  - ALIX taught lane (`alix/tools/taught_tracks.py`) — Borrow + adapt: sha256-bound certificates and gates.
  - `plan_scene_validation` (`load_wall_manager`, `PlanWallChecker`) — Borrow + adapt: wall checking on taught tracks with the selected venue file.
  - Orchestrator foul-band function (`task_config.py`) — Borrow as-is by copying the 20-line function with provenance; never import it from the ROS tree (ALIX itself keeps copies).
  - ALIX emulator and `emulated_program.py` — Borrow + adapt: automated tests without hardware, including a simulated E-stop IO.
- *Note on the inputs:* No claim (new requirement). Two related input claims need care. R4's "toggle between real and sim" is the controller's dry run on hardware, not a 3D simulation, and the pole and hand still move in it unless Lite skips them. R5's "borrow from ALIX Studio" holds for the view, but both ALIX checkers fix the bin as lowered, the dual-arm sweep omits the commode and walls are checked only in plan generation, so their verdicts cannot gate real runs without adaptation.

**Recommendation.** Make safety a designed subsystem around the hardware E-stop: show its state per arm, fail closed when it is unknown, block motion on both arms while either is engaged, never report an interrupted run as complete, and guide recovery with a checked approach move. Around that: one connection owner per arm (ALIX's one-command-source rule), one motion gateway, a software STOP that does not depend on the UI and is labelled as secondary, sha256-bound gates checked against the URDF, the commode_c scene, the declared bin state and the enforced venue walls, explicit controller run-mode rules, the control app's open 1-100 slot model with replace confirmation, and a written risk assessment validated on the bench with the E-stop drill.

**Approach**
1. **Hazard review** (build). Cover unintended motion; wrong arm or program; collision with the commode, the partner arm, the bin or a wall; pole-bin foul; pole moved during arm motion; pole or hand moved with the pad on a surface; high persisted speed; an E-stop not seen by software; a sequence continuing after an E-stop; an ineffective software stop; dual-arm desync; a second command source; SIM-mode pole/hand motion; network command injection; a corrupted upload; overwriting a slot another workflow relies on. Assign a control and a test to each.
2. **Connection owner** (adapt the runner core, with INT-CTRL). One long-lived connection per arm in the back end, which also runs the E-stop and status polling. Do not rely on a second TCP client for STOP. If runner subprocesses are kept during a transition, STOP goes to the owning process.
3. **E-stop monitor** (build on the SDK calls above). IO3 mode check at connect (with an IO1-IO4 report), per-arm chip states, fail-closed rules, configuration warnings, site-config fields (IO number, active level, last drill date), the interruption handling and the event log.
4. **Runner v14 patch** (adapt). E-stop read in `arm_state()` and in the run and sequence loops, refusal while held, "interrupted" verdict, no next sequence step after an E-stop, parseable ESTOP lines.
5. **Recovery wizard** (build, borrowing the ALIX recovery sequence). The eight steps of (c), with read-back after each clear and the approach-move collision check.
6. **Session state machine** (borrow `alix_dispatch` `session` / `runmode`). DISCONNECTED → CONNECTED (SIM) → ARMED_REAL → BUSY, with E-STOP and RECOVERING as blocking states, the arming flow, the banner, idle disarm and teardown restore.
7. **Motion gateway** (build on borrowed guards). Single-use token (60 s, bound to a hash of the command), reservation lock, E-stop check, error-code check, run-mode check (SIM skips pole and hand), foul-band check (copied function), pole-between-programs check, approach check (`start_gap_deg` plus a 3D check of the joint-linear approach), gate check (sha256 → pass record + world descriptor) and logging.
8. **Software STOP service** (borrow the `stop_all` composition). STOP ALL and per-arm STOP, the fixed STOP UI with its "software" label, the server-side path and disconnect watchdog, and stop confirmation by run state. Measure latency on the emulator and the bench, add the escalation screen, and settle the stop verb after the bench test of slow stop on a stored program.
9. **Speed, approach and dual-arm rules** (borrow + build). `global_speed_guard` restore, approach preview and site threshold, the dual-arm supervisor (stop the partner on failure; completion judged from run state) and the honest completion states.
10. **Request protection** (build). Bind to 127.0.0.1; add the custom-header token and the Host/Origin checks; refuse non-loopback starts without authentication.
11. **Checked world, walls and ledger** (with R3, R5, R13, NFR-DATA). Build the world from the URDF and the commode_c scene with a bin-state parameter and a taught-track entry point; add the walls from the selected venue file with `load_wall_manager` / `PlanWallChecker`; verify the venue choice with known-good recordings; bind a world descriptor (including the venue file and its sha256) to each pass record; check the upload ledger before a run by ID; implement the 1-100 replace confirmation with warnings.
12. **Tests, drill and bench protocol** (build). Emulator tests (including a simulated E-stop IO), the E-stop drill DR1-DR14 per arm and with both arms (idle; running stored program; dual-arm run; moving pole; moving hand; drag teach; held E-stop with every command; quick release during a sequence; recovery; PC or network failure; open-circuit wiring check; software STOP test), and a bench protocol (stop latency, pole interlock, error-code handling, SIM skip) with signed records.

**Acceptance criteria**
- [ ] Emulator: with no arm armed, every motion endpoint is refused with a plain message; after arming LEFT only, RIGHT motion is still refused. A request without a valid single-use token is refused, a reused token fails, and tokens expire after 60 s.
- [ ] A foreign Origin or a wrong Host header on any state-changing endpoint is refused, and a non-loopback start without authentication configured exits with an explanation.
- [ ] Emulator: with the E-stop IO of either arm at its active level, every motion endpoint on both arms and the switch to REAL are refused with "E-stop engaged on <ARM>, IO<n>", while an upload succeeds and shows the "robot in E-stop" note. A stale reading (> 1 s) shows UNKNOWN and blocks motion; a controller with no IO in mode 5 shows NOT CONFIGURED and blocks REAL motion.
- [ ] Emulator: an E-stop raised during a stored-program run is recorded as "INTERRUPTED — E-STOP" (never "Complete"); in a sequence no later program, pole or hand step starts, and pole stop and hand freeze are sent. The patched runner prints the ESTOP lines and does not report "RUN COMPLETE".
- [ ] After an E-stop the chip goes ENGAGED → RECOVERING → CLEAR only through the recovery wizard, which reads back the controller state after each clear and shows the approach-move collision verdict (walls included) and size before a re-run is allowed. No run resumes automatically.
- [ ] Bench: drills DR1, DR3, DR9, DR10 and DR11 pass on each arm and are recorded in the site config before operators use Lite in REAL mode; the chip turns red within 0.5 s of a button press (DR1). The remaining drills are run and their results recorded.
- [ ] STOP ALL and per-arm STOP, labelled "STOP (software)", are visible on every screen while any arm is connected. A click issues the stop on the owning connection within ≤ 300 ms (emulator and bench, measured and recorded), with no confirmation; the run state then confirms the stop, or the escalation screen appears within 1 s. Closing the browser during a run triggers the server-side stop.
- [ ] Emulator: a run with the controller in SIM issues no pole or hand command, is recorded as "controller dry run" and never satisfies the "tested on robot" step. An unreadable mode, or LEFT and RIGHT in different modes, refuses the run.
- [ ] A pole command to 0.200 m with the bin STOWED is refused with the foul-band sentence and 0.120 m is allowed (unit test). A pole command while a program is running is refused. A run whose approach move exceeds the site threshold is refused, with the move size shown (emulator).
- [ ] A trajectory whose sha256 has no passing simulation record for the current world (fixture state, bin state, pole, venue file) is refused for REAL; after any edit the old pass shows as stale; an override requires a reason and is logged. A track that enters a wall of the selected venue fails the simulation gate, and the known-good recordings used to verify the venue choice pass it with no wall contact.
- [ ] Uploads to any program ID 1-100 are accepted; an upload to an occupied slot replaces it only after a confirmation that lists the existing program; Lite never refuses an upload because of the slot number. A run by program ID is refused when the slot's listed name or size does not match the ledger (emulator).
- [ ] Emulator: after a normal end, a STOP and a killed process, the next connection sets the global speed to the site default and logs it. In a dual-arm run, a forced failure on LEFT stops RIGHT within 1 s. An external stop is reported as "Stopped outside Lite", not "Complete".
- [ ] The run log has an entry for every motion, confirmation, stop, E-stop event, recovery step and override, with sha256, controller IP and run mode.

**For operators:**
- The REAL banner, the two E-stop chips and STOP are the things an operator must see at a glance; nothing else gets the same treatment.
- The E-stop chip says what to do: "E-STOP ENGAGED on LEFT — release the button, then press Recover". While it is red, motion buttons are disabled and say why.
- Confirmations name the arm, the mode and what will move: "The LEFT arm will move to the start of SEAT TOP (left half), then clean for 15 s". A slot replacement says what is replaced: "Program 71 'R05_seat_top' (12 KB) will be replaced".
- A refusal says why and what to do: "Simulation not passed for this version — run Check in simulation first". In SIM mode: "The arm will not move. The pole and hand stay where they are."

**Safety:**
- The hardware E-stop is the safety stop. Lite's STOP is a controlled software stop for ending a run cleanly; keep the physical E-stop within reach, as the control app already says (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:3177-3178`).
- The IO E-stop is wired to a normal multiplexed IO, not a dedicated safety interface, and RealMan documents no stop category or rating for it (HW p.32). A HIGH-trigger input may not trip on a broken wire; drill DR13 checks this, and if it does not trip, the site decides whether to rewire (normally closed contact and safety relay).
- The IO E-stop does not stop the hand (ALIX field note) and may not stop a moving pole; Lite sends pole stop and hand freeze when it sees an E-stop, but this depends on Lite running. The drill measures both.
- Lite never changes IO modes; a misconfigured E-stop input is reported, not fixed, by Lite.
- Collision verdicts gate real runs only once the commode, the declared bin state, the pole height, the hands and the selected venue walls are in the checked world. The bin state is declared, not sensed, so the foul-band check is only as good as the declaration. A wrong venue file gives false wall hits or missed wall hits; the choice is verified with known-good recordings.
- Re-measure the foul band on the physical bin; `task_config.py` notes only 7.2 mm of clearance at 0.150 m.
- Overwriting a slot can replace a program another workflow (for example V6 sequences) runs by ID. The replace confirmation lists the existing program, and the ledger check stops a replaced slot from running under an old pass.
- Never let two programs drive one arm: stop the ROS driver and the Tk control app before arming in Lite.

| Priority | Size | Depends on |
|---|---|---|
| Must | XL | R4, R3, R5, R10, R7, R8, R12, R13, INT-CTRL, NFR-DATA, NFR-UX, E-stop drill on the robot |

**Open questions:**
- Who owns and signs the risk assessment for Lite?
- What is the bin state during teaching and runs (stowed, extended, lowered, removed)? Is it sensed or declared? Has the physical bin been shortened?
- IO3 carries the E-stop on both controllers (decided). Still to confirm in the drill: its active level, and whether one button trips both controllers (DR1-DR2).
- Does the IO E-stop disable the joint drives or enable flags (ALIX code assumes it does), does it latch after release, and does a running program end (`run_state` 0) or pause (2)? (Drill DR1, DR3, DR11.)
- Does the IO E-stop stop a moving pole? (Drill DR6.)
- Does the E-stop input trip on an open circuit? If not, should it be rewired through a normally closed contact and safety relay? (Drill DR13, with the electrician.)
- Should the site enable Modbus-TCP so register 302 can cross-check the E-stop state?
- How often must the E-stop drill be repeated (RealMan asks for safety functions to be tested at least yearly, HW p.35; weekly or per site is proposed)?
- What is the site threshold for the approach move (degrees)?
- On a dual-arm failure, should Lite stop the partner arm or pause it?
- Should an engineer override of the simulation gate be allowed at all on production robots?

---

### NFR-OFFLINE-DEPLOY — Offline, self-contained deployment

**Original (inputs):**
> (new — recommended)

The inputs expect Lite to run beside the real robot, but they say nothing about installation, network access or dependencies.

**Revised requirement.** Lite shall install and run with no internet access and no ROS on two targets: the **operator PC** (x86-64 Linux) now, and the robot's **Jetson AGX Orin** (aarch64, Ubuntu 24.04) in production (user decision F1). Both release bundles are required. Lite's code lives in its own folder, `alix_studio_lite` (`/home/newtonjeri/alix_studio_lite_ws/alix_studio_lite`), not inside the ALIX monorepo; it uses ALIX libraries from the vendored ALIX copy in `alix_studio_lite/vendor/` (F7).

- (a) **Platforms.** Two supported targets, both built and tested for every release:
  - *Operator PC (now):* Ubuntu 24.04 (22.04 to be confirmed) x86-64 on the robot network (192.168.1.x), with a local browser.
  - *Jetson AGX Orin (production):* Ubuntu 24.04 aarch64 (the JetPack version ALIX's `release` profile requires), the computer ALIX itself targets.
  - Python 3.10-3.12 in Lite's own virtual environment, with a wheelhouse built per architecture (x86-64 and aarch64). A current Chromium, Chrome, Edge or Firefox with WebGL.
  - If the Jetson has a screen, Lite is used from a local browser on 127.0.0.1. If it has no screen, the UI is opened from a browser on the operator PC through an authenticated connection (the non-loopback mode of NFR-SAFETY (h)); no motion endpoint is reachable from the network without that authentication.
- (b) **No network dependencies.** Every script, stylesheet, font and image is served from local files: three.js r128 and its controls, Plotly 2.35.2, and the Inter and JetBrains Mono woff2 files. No page references an http(s) URL; the trajectory prototype's CDN tag is removed. In the remote-browser mode, the browser talks only to Lite on the Jetson.
- (c) **Self-contained Python and engine.** Dependencies install from a pinned lock file and an included wheelhouse per architecture. The launcher runs Python isolated: it ignores the user site-packages and `PYTHONPATH` (for example from a sourced ROS setup, which is normal on the Jetson). Lite's kinematics, collision and dispatch code does not depend on repo markers: it uses the vendored ALIX copy in `vendor/alix` (the kept subset, copied from one ALIX commit in ALIX's own layout; X2). Decided 13 Oct: everything Lite needs lives inside `alix_studio_lite`, so no ALIX checkout is used at run time; `vendor_paths.py` forces `ALIX_REPO_ROOT`, `ALIX_SRDF` and `ALIX_TASKS_PKG` to the vendored copy, and the tests fail if any ALIX module loads from outside the folder.
- (d) **Asset bundle.** Shipped with Lite with explicit paths: the robot URDF (`alix.urdf`) and the meshes of all six referenced packages (`rm_description`, `inspire_left_hand_description`, `inspire_right_hand_description`, `pole_description`, `ugv_description`, and `realsense2_description` from the sibling `vendor/realsense-ros` tree); the SRDF and `initial_positions` files; the commode_c scene (`alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml`) with its fixture-state meshes; the cubicle wall models: ALIX's 800 mm cubicle (`alix/ros/orchestration/alix_tasks/config/scene/washroom_800mm_wide_scene.yaml` with `washroom_800mm_planning_scene.stl`) and Lite's own 1400 mm cubicle built the same way (the default, decision D21); a freshly re-baked visual overlay. The asset bundle is architecture-independent and identical in both releases. A manifest lists each file's sha256, source and ALIX commit, plus the licences. Files are world-readable.
- (e) **Start-up self-check.** It checks Python and dependency versions, the FCL backend, the asset manifest, that the SRDF is found with a non-empty disabled-collision matrix, that every URDF mesh URI resolves, that initial positions load, that the commode_c scene and the selected venue wall file and meshes are present, that the RealMan SDK is present and matches the CPU architecture (`linux_x86` or `linux_arm`), that the ALIX bundle or checkout matches the pinned commit, and that the data folder is writable. A failure shows a plain page with the fix and blocks the simulation gate.
- (f) **RealMan SDK and robot settings.** The `Robotic_Arm` package with both `libapi_c.so` builds (`linux_x86` and `linux_arm`) is bundled if the licence allows; otherwise it is located once through Settings. There are no hard-coded home paths. Controller IPs and ports are editable in Settings (defaults LEFT 192.168.1.10, RIGHT 192.168.1.103, port 8080), with "Test connection", and are shown wherever an arm is named. The venue wall file is chosen in Engineer settings (NFR-SAFETY (h)).
- (g) **Start-up, lifecycle and paths.** One icon or command starts the local server (127.0.0.1, default port 8766, single instance) and opens the browser; on a headless Jetson the server is started as a service or from a terminal and the operator PC's browser connects to it. The UI is usable within 10 s; the first 3D simulation of a region is ready within 30 s, on each target. Install, upgrade and uninstall work offline from one release artifact per architecture; project data lives outside the install folder and survives upgrades (NFR-DATA). Lite works from paths that contain spaces. The controller project file is built in a short temporary path and refused if that path would exceed the controller's 300-byte limit.
- (h) **Coexistence on the Jetson.** On the Jetson, Lite runs beside the ALIX stack. Lite does not install into, import from or modify the ALIX ROS workspace, and the one-command-source rule of NFR-SAFETY (b) applies: the ALIX ROS driver must not be driving an arm while Lite has it armed.

**Findings**
- **ALIX Studio:** The front end already works offline: three.js r128, OrbitControls/TransformControls, Plotly 2.35.2 and js-yaml are vendored, a test forbids remote assets, and Plotly is served from a whitelist. Fonts are not shipped, and Inter and JetBrains Mono are not installed on this PC (`studio/web_editor/vendor/`; `alix/core/python/alix_taskgen/test/test_studio_theme.py:185-195`).
- **ALIX Studio and alix_workspace:** The back end is tied to the repo. It finds the repo root by marker files (or `ALIX_REPO_ROOT`), reads task configs under `ros/orchestration`, finds meshes by `.git` / `package.xml` scanning, and imports the generator at load (30-60 s start-up) (`studio/server.py:56-87`; `alix/core/python/alix_workspace/src/alix_workspace/package_index.py:20-37`).
- **alix_workspace (measured):** `plan_scene_validation` sets `REPO_ROOT` at import and raises "repo root not found" outside the repo unless `ALIX_REPO_ROOT` is set; it also loads `ament_index_shim` from `REPO_ROOT/tools`. Its default venue is `washroom_800mm_wide_scene.yaml`. `body_collision._repo_root()` raises on use (`alix/core/python/alix_workspace/src/alix_workspace/plan_scene_validation.py:31-59, 62-70`; `studio/body_collision.py:114-119`).
- **alix_workspace and body_collision:** Missing assets fail silently. No SRDF gives an empty disabled-collision matrix and no group states; an initial-positions failure gives `{}`; an unresolved `package://` URI gives a warning and `None` (`studio/body_collision.py:229-235`; `alix/core/python/alix_workspace/src/alix_workspace/package_index.py:115-121`).
- **URDF and packaging:** The mesh URIs reference six packages, and `realsense2_description` lives in the sibling `vendor/` tree, found by scanning beside the repo. `alix_workspace` packages the URDF, config and `*.npz` but not the meshes; its `visual_overlay_baked.npz` is stale (2.9 MB) and has mode 600 (`alix/core/python/alix_workspace/src/alix_workspace/package_index.py:43-59`; `alix/core/python/alix_workspace/pyproject.toml:44-45`).
- **This PC (measured):** Under `python3 -I`, `alix_workspace` is not importable at all, because it is installed editable in the user site. Separately, the system SciPy 1.11.4 fails against NumPy 2.4.6 from `/usr/local`. This is direct evidence for an isolated venv (`alix/SETUP.md:442-456`).
- **ALIX setup (two targets):** ALIX itself runs on a developer host (Ubuntu 24.04, x86_64) and the Jetson AGX Orin (Ubuntu 24.04 aarch64, JetPack 7.2+ only, `release` branch), with the same native build on both and the arm64 `.so` picked at import (`alix/SETUP.md:3-5, 74-93`). `python-fcl` ships wheels only (no sdist), and the dual-arm sweep has no non-FCL fallback; `python-fcl`, numpy and scipy publish aarch64 wheels (`alix/SETUP.md:469-474`; `alix/DEPENDENCIES.md:35-70`). Whether every pinned version in Lite's lock file has a matching aarch64 wheel for the Jetson's Python must be checked when the wheelhouse is built.
- **RealMan SDK:** The Python SDK v1.1.6 ships `libs/linux_x86` and `libs/linux_arm` builds of `libapi_c.so` (plus Windows builds), so the same robot code runs on the operator PC and on the Jetson; its licence is unknown (`alix/external/butterfli-realman-alix/Python/Robotic_Arm/libs/`; `alix/DEPENDENCIES.md:35-70`).
- **Trajectory prototype and control app v53/v14:** The prototype loads Plotly from `cdn.plot.ly`, so every plot fails offline (`lite/Alix_Studio_Lite_Trajectory_V2.html:7`). The control app's default SDK path is another machine's home folder, and its IPs are display labels while the runner takes `--ip` or `RM_*_IP` (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:48-51`).
- **Code location:** The folder `alix_studio_lite` does not exist yet in the workspace; it is created by this work (F7).

**Can it be borrowed from ALIX Studio?** **Partial** — Borrow the vendored JS libraries, the no-remote-assets tests, the page-assembly pattern with lazy Plotly (copy the pattern; its module imports X1 code) and the launcher preflight idea. Do not borrow the repo-relative resolution or the heavy imports. Packaging for two architectures, the asset bundle, the isolated venv, the self-check, fonts, settings, the remote-browser mode and repo independence are new.
- *Other sources:*
  - `alix_workspace` — Borrow + adapt: the kept subset, imported from the vendored ALIX copy in `alix_studio_lite/vendor/`, with explicit paths (excluding the X2 data).
  - `alix_dispatch` (`connect`, a `track` subset, `runmode`, `session`, `peripherals`, `emulated_program`) — Borrow + adapt: included in the ALIX bundle, since it has no `pyproject.toml` (`alix/LAYERS.md:25`).
  - ALIX setup profiles (`alix/SETUP.md`) — Borrow as reference: the Jetson target definition and the arm64 `.so` selection at import.
  - ALIX emulator — Borrow as-is (optional): runs on Linux x86_64/aarch64 and Windows only.
  - Trajectory prototype — Not applicable for deployment (its CDN tag is removed).
  - Control app v53/v14 — Borrow + adapt: remove the hard-coded SDK path; take IPs from Settings.
- *Note on the inputs:* No claim (new requirement). It fills a gap: the inputs expect Lite to run beside the real robot, while ALIX Studio assumes a full checkout and the trajectory prototype needs the internet.

**Recommendation.** Ship Lite from its own `alix_studio_lite` folder as a self-contained release for both x86-64 and aarch64: its own venv from a per-architecture wheelhouse, the ALIX libraries as the vendored copy in `vendor/alix` (one ALIX commit, recorded in `vendor/MANIFEST.json`), an architecture-independent asset bundle with checksums and licences (including the commode_c scene and the venue wall files), locally served web libraries and fonts, and a start-up self-check that fails loudly where ALIX code fails silently. Develop and test on the operator PC first, but build and test the aarch64 release on the Jetson from the first release, so production does not discover architecture problems late.

**Approach**
1. **Targets** (record). Record the operator PC (OS, Python, browser) and the Jetson (JetPack, Python, screen or headless, browser if any) in the release notes; decide whether Windows is needed (not required by F1).
2. **Package** (build). Create the `alix_studio_lite` package (`pyproject.toml`) in `/home/newtonjeri/alix_studio_lite_ws/alix_studio_lite` and a lock file, and build a wheelhouse per architecture (x86-64 on the PC, aarch64 on the Jetson or an aarch64 builder); install with `pip --no-index --find-links`.
3. **Vendoring** (built 13 Oct as `tools/vendor_from_alix.py`). It runs on a machine with an ALIX checkout, copies the kept Python subset and the files in ALIX's layout (the robot meshes are exactly those the URDF references), and writes `vendor/MANIFEST.json` (source, sha256, ALIX commit) and `vendor/PATCHES.md`. Still to add: the re-baked visual overlay (current bin, camera) and licence notes. Re-running the script moves Lite to a new ALIX commit; the tests then confirm nothing broke.
4. **Repo independence** (adapt). Explicit-path constructors for the kept engine subset (with X1/X2), so a release runs with no ALIX checkout; development mode checks the checkout's commit against the pin.
5. **Web assets** (borrow). Copy `studio/web_editor/vendor/*` to Lite's web vendor folder, add the fonts, remove the prototype's CDN tag, and extend the no-remote-assets test to CSS `url()` and `@import` as a CI scan.
6. **Launcher, self-check and Settings** (adapt `alix/tools/run_studio.py:92-205`). Isolated Python (`python -I`, cleared `PYTHONPATH`), a single-instance lock and browser open (local) or service mode (headless Jetson); a plain-language self-check page with the SRDF/ACM, mesh-URI, initial-position, scene, venue and SDK-architecture assertions; Settings for IPs, SDK path, venue file and data folder (Engineer mode).
7. **Remote access** (with NFR-SAFETY). The authenticated non-loopback mode for a headless Jetson: configured credentials, a bind address limited to the robot network, and the Host/Origin/token checks.
8. **Release** (build). Package as a tarball with an install script, or a `.deb`, per architecture, and document install, upgrade and uninstall on both targets.
9. **Clean-machine tests** (build). On a network-disabled x86-64 VM and on the Jetson (or an aarch64 VM, then the Jetson): install, self-check, open a sample project, run a 3D simulation with walls, and run an emulator upload/run if the emulator ships.

**Acceptance criteria**
- [ ] On a fresh Ubuntu 24.04 x86-64 VM and on a Jetson AGX Orin (Ubuntu 24.04 aarch64), each with networking to the internet disabled, no ROS environment active and no ALIX checkout, Lite installs from its release artifact and passes its self-check.
- [ ] Every Lite module imports with no ALIX checkout and no `ALIX_*` environment variables (release mode), and no "repo root not found" appears. In development mode, a checkout at a commit other than the pin produces a warning.
- [ ] The browser network log during a full operator journey (emulator) shows requests only to Lite (127.0.0.1 locally, or the Jetson's address in remote mode), and the CI scan finds no http(s) URL in served HTML, CSS or JS (licence comments excepted).
- [ ] On a headless Jetson, the UI opens from a browser on the operator PC only after authentication; without it, every state-changing endpoint is refused.
- [ ] The UI is usable ≤ 10 s after launch, and the first 3D simulation of a sample region is ready ≤ 30 s, measured and recorded on both the operator PC and the Jetson.
- [ ] Deleting one mesh from the bundle makes the self-check fail and name the file. Deleting the SRDF, the commode_c scene or the selected venue file makes it fail instead of running with an empty matrix or no walls. In each case the 3D check refuses to report "no collision".
- [ ] On each target the self-check loads the RealMan SDK build for that CPU (`linux_x86` or `linux_arm`) and fails with a clear message if only the other one is present.
- [ ] Lite runs correctly with a sourced ROS environment (`PYTHONPATH` set) and with conflicting user site-packages present, on both targets.
- [ ] Controller IPs changed in Settings are used by every robot command and shown in the UI.
- [ ] Fonts render as Inter / JetBrains Mono on a machine without them installed.
- [ ] An upgrade on either target keeps all project data and settings.

**For operators:**
- Operators start Lite from one icon on the operator PC, or open a bookmark to the Jetson and sign in. If something is missing, they see one plain page that says what is wrong and whom to call.
- Settings that operators never touch (IPs, SDK path, venue file, data folder) live in Engineer mode.

**Safety:**
- Silent asset loss weakens the collision checks. The self-check blocks the simulation gate when meshes, the SRDF, the commode_c scene or the venue walls are missing.
- A wrong IP means the wrong robot. The IP is shown on every motion confirmation and on the arm cards.
- A remote-browser deployment on the Jetson needs the NFR-SAFETY authentication before any motion endpoint is reachable, and the operator must still be able to reach the hardware E-stop from where the browser is.
- On the Jetson, Lite and the ALIX ROS driver must never command the same arm at the same time.

| Priority | Size | Depends on |
|---|---|---|
| Must | L | X1, X2, NFR-THEME (fonts), NFR-SAFETY (remote access), NFR-DATA (data location), R3, R5, R10 (simulation assets), R4, INT-CTRL (SDK, IP settings), RealMan SDK licence decision, access to the Jetson for aarch64 builds and tests |

**Open questions:**
- Does the production Jetson have a screen, or is Lite always used from the operator PC's browser? Which authentication method fits the site (local accounts, shared site password, client certificate)?
- Which JetPack and Python version run on the production Jetson, and do all pinned wheels (including `python-fcl`) exist for them?
- Is the 3D view fast enough in a browser on the Jetson itself, if it has a screen?
- Should Lite run on the Jetson as a system service that starts at boot, or be started by an engineer?
- How do Lite and the ALIX ROS stack share the arms on the Jetson in practice (stop the ALIX driver before arming, or a hand-over signal)?
- May the RealMan SDK be redistributed with Lite?
- Should the ALIX emulator ship with Lite for engineers and tests?
- Which ALIX commit is pinned for the first release, and who approves moving the pin?

---

### NFR-DATA — Project data, provenance and integrity

**Original (inputs):**
> (new — recommended)

The inputs require a taught raw `.txt` per region (R1), edits with undo (R2), merging (R8) and uploads (R13), but they define no data rules.

**Revised requirement.** Lite shall keep all work in one project folder, never change raw recordings, record where every file came from and how it was made, keep every output version, and verify integrity before any robot use. Lite's code lives in `alix_studio_lite` (F7); project data never lives in the code or install folder.

- (a) **Data location and project folder.** Projects live in a data root outside the install folder and outside any ALIX checkout, so upgrades and re-installs never touch them. The default is a per-user folder (proposed: `~/ALIX-Lite/projects/`, as in the implementation plan, decision D17), changeable in Engineer settings; Lite refuses a data root inside its install folder or inside a git checkout. The same layout is used on the operator PC and on the Jetson (NFR-OFFLINE-DEPLOY), so a project folder can be copied between them unchanged. A fixed layout per site and fixture, as in the implementation plan §4.6: `project.yaml` (fixture `commode_c`, scene and venue file with their sha256, arms, version pins); `scene/venue_selection.json`; `registry/` (the region catalogue from the deck: ID, deck label/task/slide, arm, type, fixture state, target time, pole and hand presets, paired region); `units/<ID>/{raw, edits, opt, sim, runs}/`; `programs/`; `controller/` (the upload ledger per arm); `ladder/`; `audit/`; `sessions/`; `legacy/`; `trash/`. Site settings (controller IPs, E-stop IO, active level and last drill date per arm) live in `settings.json` at the data root.
- (b) **Immutable raw recordings.** The controller SDK writes the drag-teach recording to a temporary name inside `raw/<ID>/` (`rm_save_trajectory` writes the file itself). Lite then validates it (7 integers per line, at least 2 lines), computes the sha256, renames it to `<ID>__RAW__<yyyymmdd-hhmmss>.txt` and marks it read-only. Lite never edits, renames or overwrites a raw file; teaching again creates a new file. Deletion is an Engineer action that moves the file to `trash/`.
- (c) **Provenance and versions.** Each raw file has a `<stem>.SOURCE.json` compatible with ALIX's taught-track schema (`side`, `frame`, `pole_m`, `hand_state`, `hand_counts`, `hardware`, `taught_by`, `date`, `units: "millideg"`, `note`), plus region ID, deck task, fixture state, bin state, venue file, target and actual cleaning time, controller IP, SDK and Lite versions, point count and sha256. Every optimised or offset output is a new version `<ID>__OPT_v<k>.txt` (`k` increases and is never reused) with a `<stem>.RECIPE.json`: source sha256, the optimiser used (the prototype's default trim → smooth → scale, or time-optimal TOPP-RA), ordered recipe steps with parameters, the target duration, the achieved duration and, if the target could not be met, the limit that binds, Lite version, output sha256 and metrics (duration, peak joint speed vs rating, max step, spike count, gate results). Re-applying the recipe to the raw file reproduces the output byte for byte.
- (d) **Checksums, ledger and packages.** sha256 is recorded for every raw, derived and package file. Simulation records, robot-test records and uploads refer to the sha256 of the exact bytes used; it is verified before simulation sign-off, before any real run and before upload, and Lite refuses on mismatch. An upload ledger records slot (program ID 1-100) → (track sha256, project-file sha256, name, size, time, controller IP, and the program it replaced, as listed by the controller), because the controller reports only name and size. Overwriting a slot is allowed after confirmation (NFR-SAFETY (h)); the ledger keeps the history, so a replaced program can be identified later. Every merged or uploaded set is written to `packages/<name>_v<k>/` with `MANIFEST.json` and a `SHA256SUMS` covering every file, including the sequence JSON.
- (e) **Safe writes and formats.** Writes are atomic (temporary file in the same folder, fsync, rename). An edit checks that the file has not changed since it was loaded. Registry and sequence files keep their last 20 backups. Trajectories are NDJSON `{"point":[7 integers]}` in millidegrees; Lite reads LF or CRLF and keeps raw bytes unchanged, and writes its own `.txt` outputs with CRLF and integer values only. The controller project file is always CRLF. The sha256 is always of the bytes on disk. Sequences use a versioned, schema-validated format (`realman_workflow_sequence` v2) with automatic migration from v1.
- (f) **Names.** Region codes follow the deck's format `<L|R>-<R|T><NN>`, with parts written as in the deck (`PART 1`); in file names the key is the code plus `_PART<n>`, for example `L-R05_PART1` (Appendix A). File names use only `A-Z a-z 0-9 _ -`. Controller program names are generated so that `<id>_<name>` fits the controller's 32-byte name field and stays unique.
- (g) **Logs, browser storage and import/export.** An append-only JSONL action log in `logs/` records time, role, action, region, file sha256, controller IP, run mode and result. Browser storage holds only UI preferences, under `alixLite.*`; no work exists only in the browser. A region or project can be exported as a zip with manifest and checksums. The V6 package can be imported read-only into `legacy/`, with its checksum status recorded. Files dropped into folders by hand are adopted only through an import step that creates their sidecars.

**Findings**
- **Control app v53/v14:** Today's drag-teach save goes to `~/Downloads/RealMan_DragTeach/<side>_drag_<YYYYmmdd_HHMMSS>.txt`, with no region, pole, hand or checksum. The SDK writes the recording itself (`rm_save_trajectory(path)`), so Lite cannot make that write atomic: it must save to a temporary name, validate, then rename (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:3840-3850`; `lite/upload_channel/rm_upload_fast_v14_safe_transit.py:957-990`).
- **Runner v14:** It refuses non-integer values and files with fewer than 2 points, reads LF or CRLF, and writes the controller project file with CRLF (the controller rejects LF-only). It verifies an upload by size only (± max(64 B, 0.1 %)), so only a local ledger can tie a slot to a sha256 (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:74-96, 147-175`).
- **Trajectory prototype:** Outputs are browser downloads named `<base>_<stages>_processed.txt`, written with LF, plus a processing report that is a good recipe seed but has no source checksum, version or region. Invalid lines are skipped silently (`lite/Alix_Studio_Lite_Trajectory_V2.html:1732-1754, 1914-1917`).
- **ALIX taught lane:** The source shelf rule is "Drop taught .txt files here unmodified … nothing here is ever edited", with a `SOURCE.json` schema. Certificates are bound to the bytes by sha256, and upload and run re-check them (`alix/Resource/tracks/taught/source/README.md:1-27`; `alix/tools/taught_tracks.py:29-32, 310-342`).
- **ALIX Studio:** The atomic write exists only as the helper `schedule_guards.write_schedule_csv`. The config save handler uses plain `write_text` (not atomic) with an mtime conflict guard, and schedule backups go into the repo's `.cache`. Copy the helper, not the handler (`studio/schedule_guards.py:59-104`; `studio/server.py:3350-3360`).
- **alix_dispatch:** No record in ALIX's 157-run ledger carried a run-mode field until M47, and an arm left in SIM produces "phantom" runs that score complete. The controller's name field is 32 bytes (`<id>_<basename>` cut to 32, observed on hardware), while the runner allows 56-character names (`alix/core/python/alix_dispatch/src/alix_dispatch/runmode.py:205-215`; `alix/core/python/alix_dispatch/src/alix_dispatch/emulated_program.py:81-86`).
- **V6 package (measured):** Its own `SHA256SUMS.txt` fails `sha256sum -c`: 28 files OK, `README.txt` FAILED and 5 listed JSONs missing. The four JSONs actually used to run are not covered (`lite/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/SHA256SUMS.txt`).
- **Regions deck and slot conventions:** Deck region IDs are inconsistent; a normalised scheme and an alias table exist (Appendix A). Five slot habits differ: 25 / 99 / 40-59 / 70-100 / 71-83 (`alix/core/python/alix_dispatch/src/alix_dispatch/track.py:13-16`). They are conventions, not controller rules: the control app uploads to any ID 1-100 and replaces an occupied slot after confirmation (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:760-968, 4265-4291`), and Lite follows it (F6), so the ledger, not a reserved range, is what tells Lite's programs apart.
- **Code and data location:** Lite's code is in `/home/newtonjeri/alix_studio_lite_ws/alix_studio_lite` (F7, folder still to be created); ALIX Studio's habit of writing backups into the repo's `.cache` is the pattern to avoid (`studio/server.py:3350-3360`).

**Can it be borrowed from ALIX Studio?** **Partial** — From the Studio: the atomic-write *helper* (not the save handler), the mtime conflict guard, backups, the Board's "show only what is on disk" idea and the browser-storage rule. Its data model (task-config YAML per region × arm inside the repo) does not apply. The project layout, sidecars, versioning, ledger and checksum gates are new or come from ALIX's taught-track lane.
- *Other sources:*
  - ALIX taught lane — Borrow + adapt: the immutable shelf, the `SOURCE.json` schema and sha256 certificates.
  - Trajectory prototype processing report — Borrow + adapt: it becomes `RECIPE.json`.
  - Control-app workflow JSON v1 — Borrow + adapt: the import format; v2 adds a schema and migration (v1 is checked only for its format name and a step list, `lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:6819-6852`).
  - Runner v14 `read_track` / `write_project` — Borrow as-is: the format rules.
  - `alix_dispatch.runmode` — Borrow + adapt: the run-mode record field, stored with every run and test record.
  - V6 package — Not applicable as a data model: imported read-only as legacy reference.
  - Normalised deck ID scheme (Appendix A) — Borrow as-is.
- *Note on the inputs:* No claim (new requirement). R1's "Dashboard to show the regions with and withput the taught raw txt" implies that raw files can be found per region. Today they cannot (timestamp-only names in `~/Downloads`), so this requirement is a precondition for R1.

**Recommendation.** Define the project folder, sidecars and ledger before building R1 and R2. Reuse ALIX's immutable-source and sha256-certificate rules. Make every later step (simulation pass, robot test, upload, run by ID) refer to the exact bytes by sha256, and to the run mode in which it happened.

**Approach**
1. **Data root, layout and schemas** (build). Data-root setting with the install-folder and git-checkout refusal. Write the project layout and JSON Schemas for `project.yaml`, `registry/regions_<deck>.yaml`, `SOURCE.json`, `RECIPE.json`, simulation and test records, and sequence v2, validated on load with row-level messages. Review with the requester (registry fields) and the R1/R2/R8/R13 owners.
2. **Storage module** (borrow the atomic-write helper, build the rest). Layout, atomic writes, sha256, read-only raw files, version counters, conflict guard, backups inside the project, append-only log. Test atomicity under kill -9.
3. **Drag-save** (with R1). Save to a temporary name in `raw/<ID>/`, validate, hash, rename, mark read-only and write `SOURCE.json`.
4. **Versions and recipes** (with R2). On Save, write `OPT_v<k>` and `RECIPE.json` (seeded from the prototype's report), and add the replay determinism test.
5. **Records and ledger** (with R3, R4, R13). Key simulation and test records (with run mode) and uploads by sha256, and keep the upload ledger.
6. **Packages** (with R8, R13). Write `MANIFEST.json` and `SHA256SUMS` covering every file, and a verifier.
7. **Program names and slots** (with R13). A generator that keeps `<id>_<name>` within 32 bytes and unique against the controller's list; the ledger entry for every upload, including the program it replaced; a warning (not a block) when the slot holds a program Lite did not upload or one from the V6 workflow.
8. **Import and export** (build). V6 legacy import (read-only, checksum status, v1 → v2 migration), loose-file import with sidecar creation, and zip export.
9. **Engineer help** (build). Document the folder layout.

**Acceptance criteria**
- [ ] A teach save creates `raw/<ID>/<ID>__RAW__<ts>.txt` and its `SOURCE.json` with every required field. The raw file is read-only and any API attempt to overwrite it fails. A failed or partial SDK save leaves no file under a final raw name.
- [ ] Two optimisation saves produce `_v1` and `_v2`. The `RECIPE.json` of v2 names the raw sha256 and the parameters, and replaying it reproduces v2 byte for byte.
- [ ] Every optimised file contains only integer values and CRLF line endings, and passes the runner's track check.
- [ ] Changing one byte of a file makes simulation sign-off, real runs and uploads that use it refuse, with a plain message naming the file. A run by program ID is refused when the controller's listed name or size for that slot does not match the ledger.
- [ ] Killing Lite during a save leaves either the previous or the new complete file (fault-injection test, 100 runs).
- [ ] Every generated package passes `sha256sum -c SHA256SUMS`, and the sums cover every file, including the sequence JSON.
- [ ] The V6 FINAL JSONs import and migrate to v2 with a migration note, and a JSON with a missing key is rejected with its row number. Every generated controller program name satisfies `len("<id>_<name>") ≤ 32` bytes and is unique in the target controller's list.
- [ ] The data root defaults to a folder outside the install folder; setting it inside the install folder or a git checkout is refused; uninstalling and reinstalling Lite leaves every project intact. A project copied from the operator PC to the Jetson opens and verifies unchanged.
- [ ] An upload that replaces an occupied slot writes a ledger entry naming the replaced program; an upload to any ID 1-100 is never refused because of the slot number.
- [ ] Clearing browser storage loses no work. The action log has one line per action, with sha256 wherever a file is involved and the run mode for every run.

**For operators:**
- Operators never see file paths. The dashboard shows "Taught 12 min ago · 52 s · version 3".
- "Teach again" keeps the old recording as an earlier version, so nothing taught is lost.
- Engineers get "Open folder" and "Export region" buttons.

**Safety:**
- sha256 binding is what makes "simulated and tested" mean "these exact bytes". Without it, an edited file could reach the robot on an old pass. The ledger check stops an old or foreign program in a slot from running under a new pass.
- The pole height in `SOURCE.json` matters: a joint-space track replayed at a different pole height runs the whole path at the wrong height (`alix/experiments/exp01_teach_trajectory_reverse_engineering/data/teach_files/MANIFEST.yaml:136-138`).
- A SIM-mode run recorded without its mode would look like a real test, so the mode is always recorded.

| Priority | Size | Depends on |
|---|---|---|
| Must | L (integration inside R1, R2 and R13 is sized there) | R1, R2, R3, R4, R8, R13, NFR-SAFETY, NFR-OFFLINE-DEPLOY, sign-off of the deck ID scheme |

**Open questions:**
- Is the proposed default data root acceptable, and how are project folders backed up (network share, USB, copy from the Jetson to the operator PC)?
- Should Lite's raw shelf also feed ALIX's taught-track lane (export in its `SOURCE.json` format) for certification?
- Who maintains `registry/regions_<deck>.yaml` (target times, pole and hand presets): the requester or the site engineer?
- How long must raw recordings and run logs be kept?

---

## 10. Integration requirements

The brief asks for two existing pieces of work to be integrated into Lite: the trajectory-optimisation prototype and the upload-channel control application (UI + runner).

### INT-TRAJ — Trajectory Optimisation tab built from the prototype

**Original (inputs):**
> "we have the trajectory optimization ... which should be integrated in the Lite" (/home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html)

*(From the review brief, not one of the 14 numbered items in the inputs. The inputs file points to the same page from R2 and R6 (`docs/reference/ALIX_Studio_Lite_requirement_inputs.txt:10`, `docs/reference/ALIX_Studio_Lite_requirement_inputs.txt:18`), and its scope statement says the taught plans are "optimized (smoothed and speed increased) since they are recorded at slow speed and with a number of stops" (`docs/reference/ALIX_Studio_Lite_requirement_inputs.txt:3`).)*

**Revised requirement.** Lite shall contain a Trajectory Optimisation tab, rebuilt from `Alix_Studio_Lite_Trajectory_V2.html` inside the Lite web shell (not as an iframe), in which the operator picks the optimiser and the target time, the Lite server produces every saved file with one of the two R2 optimisers and the shared limit checks, and the browser only previews.

- (a) The tab opens a taught region from the region dashboard (R1) and shows its unit ID, arm, region, fixture state, pole/hand and any MAX DEV, instead of a loose file. "Import file" stays as an advanced path that requires an ID and an arm.
- (b) **Optimiser choice (operators and engineers, decided 13 Oct):** **Default** (the prototype's trim → smooth → scale, with the R2 fixes and limit enforcement) or **Time-optimal (TOPP-RA)** (ALIX's chain). TOPP-RA is greyed out, with the reason, when it is not available on this computer.
- (c) **Target time:** prefilled from the region catalogue (R1), editable per region, and optionally per sub-region (pinned seconds, Default optimiser only). Next to it the tab shows the recorded time, the shortest feasible time and the achieved time (lines × 5 ms). If the target is met, a green "Target met" chip; if not, an amber "Closest feasible: X s — limited by joint N speed/acceleration in a–b s" chip with a **Use X s** button and the window shaded on the plots.
- (d) **Auto-optimise (recommended)** is the main path: it runs the region's optimiser with its target time. A guided fine-tune path follows: Choose region → Optimiser and target → Remove stops → Smooth → Speed (sub-regions) → Check → Compare & save.
- (e) Joints are shown in degrees against seconds. Durations are (lines − 1) × 5 ms, one line per controller cycle; all limit checks run at the same 5.0 ms tick.
- (f) An L1-L6 panel shows pass/fail with values for every hard limit, with G5, jerk, pad speed and raw-tick warnings as notes (R2, R12).
- (g) Both arms of a dual-arm step are shown on one time axis, with each arm's duration and the difference. In a TOGETHER block both arms share one line count and the tab names the binding arm.
- (h) Numbered versions are saved to the region store with recipe, optimiser, library versions, checksums and check results. Only the latest version with current, passing checks can go to simulation, real-arm test or upload.
- (i) The tab uses ALIX Studio's tokens, components and vendored Plotly 2.35.2, works offline, and runs on the operator PC (x86-64 Linux) now and on the Jetson AGX Orin (aarch64, Ubuntu 24.04) in production.

**Findings**
- **Trajectory prototype:** It is a single static page: Plotly from a CDN, no server, no storage, no region identity and one file at a time (`lite/Alix_Studio_Lite_Trajectory_V2.html:7`, `lite/Alix_Studio_Lite_Trajectory_V2.html:587-604`).
- **Trajectory prototype:** Worth keeping: the six-step stepper with lock/done states, the DOM-free processing functions (now the basis of the Default optimiser), the compare view with a synchronised cursor, the report JSON and an honest note on what the browser cannot prove (`lite/Alix_Studio_Lite_Trajectory_V2.html:609-704`, `lite/Alix_Studio_Lite_Trajectory_V2.html:1732-1755`).
- **Trajectory prototype, no limits:** Its pipeline checks no ratings, acceleration, steps or joint limits (`lite/Alix_Studio_Lite_Trajectory_V2.html:1676-1810`), and three defects were measured: end-point snaps of 546–1005 millidegrees, boundary acceleration spikes of 640–919 °/s², and 10.6–20.9 mm of pad path change with the 101-line window (`lite/Alix_Studio_Lite_Trajectory_V2.html:1513-1514`, `lite/Alix_Studio_Lite_Trajectory_V2.html:1568-1616`, `lite/Alix_Studio_Lite_Trajectory_V2.html:1494-1517`). The minimal fixes are in R2.
- **Two optimisers, measured 13 Oct 2026:** fastest legal time at 5 ms per line, Default (fixed, 41-line window) against TOPP-RA: FILE_08 27.42 s vs 16.80 s; FILE_10.1 8.14 s vs 4.26 s; FILE_01.1 38.30 s vs 19.19 s (3839 lines, the same as ALIX's hardware slot 101, `alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:588`). One TOPP-RA solve takes 0.11–1.82 s on this x86-64 PC, so it runs in a worker.
- **Trajectory prototype:** The prototype plots millidegree counts against row numbers and computes durations as N × 5 ms, so a 9,813-line output shows 49.1 s (`lite/Alix_Studio_Lite_Trajectory_V2.html:1176-1179`). Lite keeps its 5 ms time base but counts a file of N lines as (N − 1) × 5 ms, the true time from the first line to the last; the prototype's N × 5 ms is one period too long.
- **Trajectory prototype vs control app v14:** The prototype's parser accepts non-integer values and skips bad lines. Runner v14 `read_track` and `alix_dispatch` `parse_track` refuse the whole file on one bad line (`lite/Alix_Studio_Lite_Trajectory_V2.html:803-829`, `lite/upload_channel/rm_upload_fast_v14_safe_transit.py:76-89`). The prototype's integer output is accepted.
- **Trajectory prototype:** Its theme differs from ALIX: accent `#175cd3`, 8–12 px radius and a dark header, against ALIX `--brand #2F96EE` and `--radius 4px` (`lite/Alix_Studio_Lite_Trajectory_V2.html:9-21`). The evidence review found 8 UI-state bugs, for example a scale edit that skips `invalidateProcessed` (`lite/Alix_Studio_Lite_Trajectory_V2.html:1359-1371`).
- **ALIX Studio:** It has no trajectory processing. It vendors the same Plotly 2.35.2 and loads it with `ensurePlotly` (`studio/web_editor/editor.js:5322-5337`); its tokens, status line, toast and modal are reusable.
- **V6 package and lid_top files:** 6 of 19 shipped files fail ALIX's enforced band-limited rate gate (G2) and `R10_HNGR` fails G1, so hard gates are needed at integration. ALIX's `taught_tracks.py screen` already gives per-file verdicts with reasons and writes nothing (`alix/tools/taught_tracks.py:4149-4162`).
- **Control app v53/v14:** Raw recordings are saved as `<side>_drag_<stamp>.txt`, with no region ID, so R1 must attach the ID when saving (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:3840-3850`). The controller's own move to the first point is not collision-checked (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:26-29`). Upload accepts any program ID 1-100 and replaces an existing program after confirmation (INT-CTRL, R13); the tab's Upload hand-off follows that.
- **Jetson:** toppra 0.6.10 on PyPI has x86_64 wheels only and no source package (checked 13 Oct 2026), so the aarch64 bundle must build it from the git tag, and the tab must work when it is missing.

**Can it be borrowed from ALIX Studio?** **Partial** — ALIX Studio has no trajectory processing. Its tokens, vendored Plotly with `ensurePlotly`, status line, toast and modal can be borrowed as-is; gate chips, validation rows, docks, dialogs and the 3D viewer and collision timeline (for the Check step) need adaptation. Everything trajectory-specific comes from the prototype and from ALIX's taught-track tools and TOPP-RA chain.
- *Other sources:*
  - **Trajectory prototype** — Borrow + adapt: take the stepper, the pure functions (ported to the server as the Default optimiser, and kept in the browser for preview), the compare view, the report schema, box-select and the cursor. Replace the theme, units and analysis, apply the R2 fixes, add the limit checks and fix the 8 UI-state bugs.
  - **ALIX taught-track tools and TOPP-RA chain** — Borrow as-is through the ALIX adapter: the Time-optimal optimiser and the G1, G4 and G5 checks (see R2, R12).
  - **alix_dispatch** — Borrow as-is: `parse_track` (`alix/core/python/alix_dispatch/src/alix_dispatch/track.py:66-82`).
  - **alix_workspace** — Borrow as-is: forward kinematics (`UrdfModel.fk_batch`).
  - **Control app v53/v14** — Borrow as-is (file contract only): output must pass `read_track`, and input is its drag-teach recordings. No control-app issue blocks this tab.
- *Note on the inputs:* Accurate that the prototype is the right base for the tab's flow, pure functions and compare view, and its algorithm stays the default. Inaccurate if read as production-ready: its algorithm needs three fixes and limit checks, it has no target time and no time-optimal option, and it lacks region identity, storage, validation and simulation.

**Recommendation.** Port the prototype's guided flow into the Lite shell, re-skinned with ALIX tokens and served with vendored Plotly, and lead with Auto-optimise. Put the optimiser choice and the target time at the top of the tab, and always show the recorded, shortest feasible and achieved times, with the binding limit when the target cannot be met. Make the server's recipe processor (both optimisers) and the shared L1-L6 checks the only source of saved files. Bind the tab to region identity and to both arms, show durations as (lines − 1) × 5 ms, and gate every hand-off on a current, passing version. Keep the prototype's honest disclaimer, in plain words.

**Approach**
1. **Tab module (build)** at `alix_studio_lite/web/trajectory/` in the Lite code folder (`/home/newtonjeri/alix_studio_lite_ws/alix_studio_lite`). Lite is not a package inside the ALIX monorepo; it imports `alix_workspace`, `alix_dispatch` and the taught-track modules from the vendored ALIX copy in `alix_studio_lite/vendor/`. Port the prototype's stepper (borrow + adapt), map its colours to ALIX tokens (`--accent` → `--brand`, `--line` → `--border-strong`, state colours to ALIX's), vendor the Inter and JetBrains Mono fonts and drop the dark header.
2. **Region binding (build):** replace file upload with "Choose region" from the R1 dashboard, and keep "Import file" as an advanced path that requires an ID and an arm. Files use the Appendix A IDs: raw `<ID>__RAW__<ts>.txt`, optimised `<ID>__OPT_v<k>.txt`.
3. **Server routes (build):** `/api/units/{id}`, `…/recipe`, `…/optimise`, `…/optimisers`, `…/checks`, `…/preview`, `…/versions`.
4. **Clock and durations (borrow from the prototype):** one time base, 5 ms per line, for filters, limit checks, durations ((lines − 1) × 5 ms) and the target's line count (target ÷ 5 ms + 1), as the prototype's single `sampleMs` already does. Plot at 5 ms per line so the axis end matches the displayed duration. Show the taught duration ((N − 1) × 5 ms) and the controller's move to the first point separately; the control app's speed % changes only that move, not the track (`alix/plan/FINDINGS.md:5855-5857`).
5. **Optimiser and target panel (build):** optimiser choice (TOPP-RA greyed out when `/optimisers` reports it unavailable); target time input (s) prefilled from the catalogue; recorded, shortest feasible and achieved times; the Target met / Closest feasible chip with **Use X s**; the binding window shaded on the joint, speed-utilisation and acceleration-utilisation plots. A sub-region table shows time % for Default and speed cap % for TOPP-RA, with pinned seconds under Advanced. A TOPP-RA solve shows "Optimising…" with Cancel; the previous result stays visible.
6. **Limit panel (build):** L1-L6 pass/fail with values; G5, jerk, pad speed and raw ticks as notes; each with its plain-language label, never a bare code.
7. **Compare view (borrow + adapt from the prototype):** a path-linked axis by raw-line index, linked zoom, compare with an external file or with the other optimiser's result for the same target, and metrics (duration at (lines − 1) × 5 ms, time saved, G4, predicted G5, check summary).
8. **Dual-arm lane (build):** the partner unit's current version on the same axis with the duration difference. For TOGETHER blocks: both arms' shortest times side by side, the common line count and the binding arm, and "clearance re-check required" until R5 passes on the retimed pair (R10 owns synchronisation).
9. **Output and report (borrow + adapt):** integer millidegrees with CRLF, validated with the `parse_track` rules. A report v2 sidecar holds the unit ID, arm, raw and output SHA-256, optimiser and recipe, numpy, scipy and toppra versions, execution-model version, target, shortest feasible and achieved times, the binding report, L1-L6 and G5 results, and links to simulation and real-test records.
10. **Gated hand-offs and legacy files (build):** Simulate, Test on robot and Upload are enabled only for the latest saved version with current, passing checks. Upload goes to any program ID 1-100 chosen by the operator, with the replacement confirmation of INT-CTRL. V6 and lid_top files import as `legacy_processed`, with L1-L6 findings, a "fit to limits" offer and the ALIX `screen` output attached.
11. **Tests:** offline load with no CDN, the 8 prototype UI-state bugs, theme guard tests, Playwright flows (including an infeasible target and TOPP-RA unavailable), run on both the operator PC and the Jetson.

**Acceptance criteria**
- [ ] With the network disabled, the tab loads and plots, with no CDN requests, on the operator PC and on the Jetson.
- [ ] Opening a unit from the dashboard shows its ID, arm, region name, fixture state, pole/hand, any MAX DEV and the catalogue target time, with no file dialog.
- [ ] Axes are seconds and degrees. A 9,813-line output shows 49.1 s ((9,813 − 1) × 5 ms = 49.06 s).
- [ ] The optimiser choice offers Default and Time-optimal (TOPP-RA). With `toppra` not importable, TOPP-RA is disabled with the message "TOPP-RA unavailable on this computer — using the default optimiser", and the Default optimiser works.
- [ ] With a target at or above the shortest feasible time, the chip reads "Target met" and the achieved time is within one line of the target. With a shorter target (FILE_10.1, TOPP-RA, 3.0 s), the chip reads "Closest feasible: 4.26 s" (853 lines at 5 ms per line), names joint 6 acceleration with its window (2.88–3.05 s), and **Use X s** sets the target to that value.
- [ ] A version can be saved as sendable only if L1-L6 all pass on its bytes (with L5 at the region's MAX DEV where one is set; G5 is a warning, D11). Otherwise only "Save draft" is possible, and the reason is shown.
- [ ] Saved versions appear as `<ID>__OPT_v<k>.txt` plus a sidecar. Re-running the stored recipe on the raw file reproduces the output (within 1 millidegree across platforms), and the file passes runner v14 `read_track` and `alix_dispatch.parse_track` unchanged.
- [ ] Auto-optimise on FILE_08 produces a sendable version in one click with each optimiser, showing the target, the achieved time, the time saved and, when the target is not met, which limit binds.
- [ ] A dual-arm step shows both arms on one time axis, with each arm's duration and the difference; a TOGETHER block shows the common line count and the binding arm.
- [ ] The tab's CSS has no colour literals outside token definitions (guard test), plots are legible in light and dark, and regression tests cover the 8 prototype UI-state bugs.

**For operators:**
- "Auto-optimise (recommended)" comes first. Fine-tune steps use plain names, and "Next" is disabled with the reason shown.
- The target time is a plain number of seconds. When it cannot be met, the tab says why in one sentence and offers the fastest safe time.
- No jargon ("counts", "rows", "JSONL", "TOPP-RA" without the label "Time-optimal"). Drafts save automatically, and main buttons are 44 px.
- The honesty note stays: "These checks look at the arm's motion only. Run the simulation and a slow real test before uploading."

**Safety:**
- Hand-offs are gated: a version older than the latest checks is never uploaded or run.
- The controller's move to the first point is not collision-checked, so Lite must announce it before a real run.
- Legacy files that fail G2 or G1 (6 + 1 shipped files) are blocked from real runs until fixed; files over only on raw ticks warn.
- A time-optimal file runs close to the acceleration limit for most of its length, and TOPP-RA's hardware record is on the CANFD channel and the left arm (`alix/docs/motion/TAUGHT_CANFD_PIPELINE.md:556-557`). The first real run of any new version is slow, and its measured result is recorded against the prediction.
- The IO emergency-stop buttons on both arm controllers remain the primary stop during real tests (NFR-SAFETY).

| Priority | Size | Depends on |
|---|---|---|
| Must | L (excludes the R2, R11 and R12 internals) | R1, R2, R6, R11, R12, R3, R4, R5, R10, R13/INT-CTRL, NFR-THEME, NFR-OFFLINE-DEPLOY |

**Open questions:**
- Should the prototype stay usable on its own for engineers?
- Is the Appendix A ID scheme approved?
- Will Lite later stream over CANFD, where smoothing matters more and TOPP-RA has its hardware record?

---

### INT-CTRL — Control application integration (UI v53 + runner v14)

**Original (inputs):**
> "And the Control Application, latest of the ui and the server program, confirm whether this is the most comprehensive of all .../upload_channel/py programs/rm_trajectory_ui_v48_handoff_continue_fix.py"

*(From the review brief, not one of the 14 numbered items in the inputs. It goes with the operations notes `lite/RealMan Dual-Arm Operations UI.txt`, which give "Current version: UI v50 / Runner v13".)*

**Revised requirement.** Lite shall integrate the upload-channel control application, taking UI v53 and runner v14 (with `hand.py` and `pole.py`) as the functional baseline; v48 is not the baseline. Lite shall offer its robot functions in an operator flow and an engineer area, run first through a pinned copy of the runner and later through a library on `alix_dispatch`. It runs on the operator PC (x86-64 Linux) now and on the robot's Jetson (aarch64) in production.
- (a) **Executor.** In Phase 1, Lite moves the robot only through runner v14 + `hand.py` + `pole.py`, as a Lite-owned copy pinned by checksum whose only changes are opt-in flags (R4, R13) and the E-stop fix (R4). UI v53 is the reference for behaviour and for the dual-arm rules.
- (b) **Operator area.** Without seeing exit codes, runner terms or stage labels, an operator can: see each arm's connection, mode, activity and E-stop state; apply a region's pole and hand preset while that arm is idle; drag-teach and save a recording named by region; run a named recipe on one or both arms after one confirmation that names the arms, the mode and the recipe; answer plain-language prompts (lid checkpoint, handoff); see the robot program number (1–100) that Lite pre-fills for an upload and change it if needed, with a confirmation before an occupied number is replaced; stop one arm or both with one click at any time.
- (c) **Engineer area.** Connection settings (IP and port per arm, SDK path, a test-connection button that shows the mode and the stored programs); controller programs, with the control program's capability (upload and verify to any Program ID 1–100, batch upload with a Program ID per file, slot list with each slot's current program, a replacement confirmation that shows the program being replaced, run by ID or by name, single and dual test runs; R13); recipe builder (Program, Pole, Hand, Wait and Global steps, dual rules, Start % and overall speed, plan preview); global points; SIM/REAL mode; raw logs; version information.
- (d) **Compatibility.** Lite reads and writes `realman_workflow_sequence_v1` files that v53 loads and plans identically (v52 timeline and v53 checkpoint semantics). A golden test on the V6 70-series pair proves this. Lite refuses a file whose `side` does not match the arm.
- (e) **Architecture.** Phase 1 (Must): one runner subprocess per action, behind a localhost-only backend that passes `--ip/--port`, turns the output into structured events, owns the dual-arm orchestration and holds one lock per controller. Phase 2 (Should): a library on `alix_dispatch` with one persistent connection per arm; the screens and files do not change.
- (f) **Safety.** Keep every runner safety behaviour, and add: the IO E-stop as the primary safety stop, with its state shown per arm and all motion on both arms refused while either is not clear (R4 (h)); cross-arm abort; a pole/hand interlock (no move while that arm is running or settling); REAL-mode visibility and confirmation; a pinned runner version; one command source per controller; a slot check before every recipe; stop-all on backend shutdown; pole and hand steps skipped, and said so, in controller SIM mode.
- (g) **Hardware gate.** Before operators run recipes in REAL mode, the E-stop drill items DR1, DR3, DR9, DR10 and DR11 pass (R4) and the stop gate (ALIX "G4": does slow stop abort a running stored program?) is measured on both controllers. Its result decides whether "both arms together" blocks are offered to operators.

**What the programs do.** The control application is a front end, a back end and two helpers. The front end, UI v53, is a Tkinter desktop app ("RealMan // Dual-Arm Operations Console"). Page 01 has a LEFT and a RIGHT arm panel with the tabs RUN, POINTS, DRAG TEACH, HAND / POLE and TELEMETRY; Page 02 builds one recipe per arm and runs both with "RUN BOTH // SAFE TIMELINE". The UI never talks to the robot. Every button starts the back end, runner v14, as `python -u <runner> --sdk <SDK> --side left|right --yes <action>` (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:4255-4262`). The runner (the "server program") is a command-line program, not a network server: one process opens one connection to one controller, does one job and exits, and exit code 0 means done. Its jobs are: upload and verify a `.txt`, list the slots, run a slot or a sequence, run a Page-02 workflow, drag teach (record, save, play back), move the hand and the pole (through `hand.py` and `pole.py`), read or set SIM/REAL mode, and move to global points. The UI learns what happened only from the printed lines and the exit code. STOP sends SIGINT, which the runner turns into a slow stop; a dual start is two launches released together by a software barrier. The dual-arm rules live only in the UI's planner. It splits the two recipe files into stages (blocks where both arms run, serial handoffs, the lid checkpoint) and starts one runner per arm per stage, so the V6 recipe makes 12 runner processes.

**Findings**
- **Control app v53/v14:** UI v53 + runner v14 is the current pair. v53 declares it (`UI_PROGRAM_VERSION = "v53"`, `EXPECTED_RUNNER_VERSION = "v14"`) and keeps every v48 function, button and runner flag except one inner helper and two labels (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:45-46`). v48 pairs with the older runner v13.
- **V6 package:** The production recipe decides it. On the V6 FINAL pair, v48 and v51 reject the recipe ("Missing on RIGHT: 73 Missing on LEFT: 74, 75"), v52 plans 4 stages with no lid pause, and only v53 plans 7 stages with the manual lid checkpoint (`lite/upload_channel/rm_trajectory_ui_v48_handoff_continue_fix.py:9576`; `lite/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/README.txt:69-80`).
- **Operations notes:** Out of date. They name UI v50 / Runner v13 (no such files exist), describe the old rule that TOGETHER pairs matching Program IDs, and omit Start %, timeline blocks and the lid checkpoint. Otherwise all 24 sections exist in v53/v14 (`lite/RealMan Dual-Arm Operations UI.txt:115-123, 222-224`).
- **Control app v53/v14:** The runner reports only through printed lines and its exit code. It prints `WORKFLOW STEP/PROGRAM i/n` lines that v53 never parses. The IPs in the UI are display-only, `--ip` overrides `--side`, and nothing checks a file's `side` field (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:2347`).
- **ALIX Studio:** It cannot drive the robot. It has no connect, upload, run or stop route, and a test pins "The studio has no execution channel" (`alix/core/python/alix_taskgen/test/test_studio_interaction_guards.py:2204-2211`). Reusable: the stdlib `ThreadingHTTPServer` scaffold with its localhost default and its warning against unauthenticated binding (`studio/server.py:6539-6544`), the theme tokens, `setStatus`/`showErrorToast` and the `showStudioModal` confirmation card.
- **alix_dispatch:** ROS-free modules cover most of Phase 2: `connect.open_arm`, `send_project`, `global_speed_guard`, the `stop_program` escalation (slow stop, then pause, then stop), `wait_until_at_rest`, `preflight_slots`, `assert_side`, `runmode`, `peripherals` (of which `hand.py`/`pole.py` are trimmed copies) and the hardware-proven `staging` order. It has no drag-teach or global-point code and no `pyproject` (`alix/core/python/alix_dispatch/src/alix_dispatch/track.py:3109`).
- **alix_workspace (stop gate):** Nobody has measured whether slow stop aborts a running stored program. ALIX's rule is "Program replay is single-arm-only until G4(b) passes", and a ready probe exists (`alix/docs/motion/DRIVER_TRACK_CHANNEL_ARCHITECTURE.md:203`; `alix/core/python/alix_dispatch/src/alix_dispatch/track.py:3273`). Lite's STOP and its "both arms together" blocks depend on the answer.
- **Control app v53/v14 (program slots and E-stop):** v53 accepts any Program ID 1–100, uploads batches with a Program ID per file and a replacement confirmation that names the program in the slot, and runs a stored program by ID or by name (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:9-15, 4458-4480, 4781-4800`). Neither program reads the E-stop: v53 only shows "Software Slow Stop is not the hardware E-stop" (`…v53_safe_lid_checkpoint.py:3177`), and the runner would very likely report an E-stopped run as complete and continue a sequence (`lite/upload_channel/rm_upload_fast_v14_safe_transit.py:219-271, 641-700`; inferred, see R4).
- **alix_workspace (SIM mode):** The controller does not execute lift or hand commands in SIM (`alix/docs/PHASE_PLAN.md:116`). Runner v14 still sends them, so a SIM dry run of the V6 recipe will probably time out at its first pole step (inferred; verify on hardware).

**Can it be borrowed from ALIX Studio?** **Partial** — ALIX Studio has no execution channel, so connect, upload, run, stop, drag teach, pole, hand, SIM/REAL and dual sequencing must all come from elsewhere. Lite borrows only the shell: the localhost-only server scaffold, the theme tokens, the status, toast and confirmation-card components, and the Board table as a visual pattern (its actions call the excluded generator).
- *Other sources:*
  - Runner v14 + `hand.py` + `pole.py` — **Borrow + adapt** (Phase 1): run a pinned Lite copy as a subprocess with `--ip/--port` and an explicit `--sdk`, plus R4's opt-in flags and E-stop fix; it keeps store-only upload with size read-back, pre-run refusals, speed restore and mode read-back.
  - UI v53 — **Partial**: port the pure planner (`_page2_program_chunks`, `_page2_build_program_rule_plan`), the checkpoint stage and the exit-code handoff; rebuild the screens; do not port the Tk widgets or the dark palette.
  - `alix_dispatch` — **Borrow + adapt** (Phase 2, plus the G4 probe now): persistent sessions, upload, speed guard, stop ladder, at-rest wait, slot preflight, side check, run mode, pole and hand; drag teach and global points are ported from r14.
  - `alix/core/python/alix_dispatch/test/test_rm_upload.py` — **Borrow + adapt**: its fake-controller pattern lets the pinned runner copy run in CI through a fake `Robotic_Arm` package passed with `--sdk`.
  - `alix/tools/dual_arm_flow.py` — **Reference**: the model for gating "both arms together" on G4 (a concurrent step is demoted to serial when the gate is unmet).
  - `usage.py`, `upload/rm_upload.py`, `py programs/` — **Reference only**: the safe pole/hand order and at-rest check; history.
  - V6 FINAL JSONs — **Test data**: golden planner input (7 stages, one checkpoint).
  - Trajectory prototype — **Not applicable**: it only produces the `.txt` files (LF line ends; the runner rebuilds CRLF).
- *Note on the inputs:* **Inaccurate.** v48 is neither the latest nor the most comprehensive version. Three newer UIs (v51, v52, v53, all dated after v48) sit in the `upload_channel` root; v53 contains all of v48 and pairs with runner v14. The companion notes ("UI v50 / Runner v13") are also stale: no v49 or v50 file exists, v52 replaced the TOGETHER rule they describe, and their Multi Run and Load Sequence do not work in v48 or v53.

**Recommendation.** Take UI v53 + runner v14 as the baseline, and first spend time on the robot to run the E-stop drill, measure the software stop gate (G4) and see what pole and hand commands do in controller SIM. In Phase 1 (Must), wrap the pinned runner copy behind a localhost-only Lite backend that adds IP entry, the E-stop state and interlock, structured events, a dual-arm orchestrator with cross-arm abort and the safety checks, and rebuild the screens in the ALIX Studio shell as an operator flow and an engineer area. In Phase 2 (Should), replace the subprocess with a library on `alix_dispatch` without changing the screens or the files. Keep `realman_workflow_sequence_v1` and the v52/v53 dual-arm rules exactly; if v53 stays as an engineering fallback, it must never drive a controller while Lite does.

**Where each control-app feature goes in Lite** (the 24 sections of the operations notes, grouped):

| OPS sections | Control-app feature | Lite place | What changes from v53 |
|---|---|---|---|
| 1, 9, 19 | Two-arm status and mode; STOP LEFT / RIGHT / BOTH | Operator flow (Robot bar, always visible) | Mode is read-only here. An E-stop chip per arm is added. STOP ALL, labelled as a software stop, is one click, with no dialog, and stays above every dialog. Cross-arm abort is added |
| 13-15, 21, 24 | Automatic handoff, handoff confirmation, live status, runtime, JSON workflows | Operator flow (Run: pick a named recipe, press Start) | Handoff runs on the server. Operators see the region, "step k of n", any waiting reason and the elapsed time. v1 files are kept exactly |
| 5 | Drag & teach | Operator flow (Teach per region); Engineer area (playback, file paths) | Recordings named by region; the save is retried; no new recording until the last one is saved |
| 7, 8 | Hand; pole / lift | Operator flow (GRIP, OPEN, region pole preset); Engineer area (six sliders, free mm entry 0-315) | Only while that arm is idle; skipped in controller SIM |
| 1, 22 | Arm IP addresses; version display | Engineer area (Settings & Connection, version information) | IP and port editable per arm and passed as `--ip/--port`; test connection; runner checksum enforced, not only shown |
| 2-4 | Page 01 upload and slots, trajectory execution, dual stored-program run | Engineer area (Controller Programs) | Same capability as v53: any Program ID 1–100, per-file IDs in a batch, replacement confirmation that shows the slot's current program (in single uploads too), run by ID or name, single and dual test runs; one connection per arm per batch; no speeds for operators |
| 2 (part) | Multi Run, Page 01 Load Sequence | Not needed | Both raise `TypeError` in v48 and v53; recipes replace them |
| 10-12, 18 | Page 02 builder, per-program dual rules, mixed parallel and serial, overall speed | Engineer area (Recipe Builder with plan preview) | v52 timeline and v53 checkpoint rules, not the "matching Program ID" rule in the notes |
| 6 | Global points | Engineer area | Blocking moves; no operator "Park" button in Phase 1 |
| 9 | SIM / REAL switch | Engineer area (switch); operators see the mode | Called "Dry run on the controller"; confirm before REAL |
| 20 | Telemetry | Engineer area (Logs) | One log per arm per run, kept across stages and page reloads; operators see failures in plain words |
| 16, 17 | Completion detection, fast switching | Not needed as a screen (backend) | Runner logic kept; Phase 2 adds one persistent connection per arm |
| 23 | Tk interface (tabs, dark palette) | Not needed | Replaced by the ALIX Studio shell and theme; only the colour meaning (run, upload, stop) is kept |

**Approach**
1. **Phase 0, hardware gates (on the arm; borrow the probe).** Run the E-stop drill from R4 on each arm and with both arms. On each controller, run `track.probe_stop_policy` on a dry, in-place track and record whether slow stop aborts the program and how far the arm travels. In controller SIM, send one pole and one hand command and record the result. If G4 fails, change the runner's stop to escalate to pause and then stop, as `stop_program` does, before release.
2. **Pin the runner (borrow + adapt).** Copy r14, `hand.py` and `pole.py` into `alix_studio_lite/robot/runner/`, apply only R4's opt-in flags and E-stop fix (proven by diff and fake-SDK tests), record their sha256 in `runner_manifest.json`, and refuse any other runner.
3. **Settings (build).** `lite_settings.json` holds `{ip, port}` per arm, the E-stop IO and level per arm, and an explicit `sdk_dir` with the x86-64 or aarch64 SDK build (the v53 default points at another machine). Refuse identical LEFT and RIGHT IPs. "Test connection" runs `--list --get-arm-mode` with a 10 s time-out. Always pass `--ip/--port`.
4. **Arm channel and events (borrow the server scaffold, build the rest).** One channel and one lock per arm; each runner starts in its own process group with `--sdk`, `--side`, `--ip`, `--port` and `--yes`. Every line goes to `runs/<date>/<run-id>_<side>.log`, and a regex table turns the lines into typed events served at `GET /api/lite/robot/state` and `/events?after=N`. Bind to 127.0.0.1 and require a session token on motion routes.
5. **Stop (build).** `POST /api/lite/robot/stop` sends SIGINT once to each active runner, with no confirmation, and cancels pending stages. The stop counts as confirmed when the slow-stop line and the exit arrive (allow about 8 s); otherwise show "Stop not confirmed: press the E-stop button". On backend shutdown, stop all runners first.
6. **Planner (port from v53).** Move the two pure planner functions into `planner.py`, working on plain step lists, keep v53's validation messages for engineers, and add the V6 golden tests.
7. **Orchestrator (build).** A server-side state machine runs parallel, serial, checkpoint and tail stages. It adds what v53 lacks: a launching state, cross-arm abort, an E-stop interlock that cancels the recipe and freezes pole and hand on both arms, "Stopped" (not "Complete") between stages, a start time per arm, pole and hand steps stripped in controller SIM, and a slot check (`--list` against Lite's upload record) before every recipe. "Both arms together" is offered to operators only after G4 passes.
8. **Recipes and mode (build).** A recipe is a folder with LEFT.json and RIGHT.json (v1, imported and exported unchanged) plus `recipe.json` (name, regions, checkpoint text); `side` is checked on load. Read the mode on connect and after every switch, and confirm before REAL.
9. **Screens (borrow the shell, build the content).** Operator: Robot bar, Run, Teach hooks, Hand & Pole presets, with prompts in `showStudioModal`. Engineer: the screens in the table above. Lever: the first release can import recipes built in v53's Page 02 and defer Lite's own builder.
10. **Phase 2 (Should; borrow + adapt `alix_dispatch`).** Behind the same backend interface, replace the subprocess with one persistent session per arm, port the workflow, drag-teach and global-point code from r14, and prove equivalence with `emulated_program` plus one recorded hardware session.

**Acceptance criteria**
- [ ] The backend refuses a runner whose sha256 is not the pinned Lite runner value; pointing it at `rm_upload_fast_v13_overall_speed.py` gives a clear refusal.
- [ ] The pinned runner copy runs upload, run, workflow, drag start/stop/save, hand, pole, mode and list against a fake `Robotic_Arm` package; each action gives the expected events, every command line contains `--sdk`, `--side`, `--ip`, `--port` and `--yes`, and unknown lines go to the raw log.
- [ ] The V6 FINAL pair plans exactly `TIMELINE-1 L[71,73] R[71,74,75]`, `LID-PREP-1`, `LID-CHECK-1`, `TIMELINE-2 [76]`, serial 78 (LEFT first), serial 79 (LEFT first), `TIMELINE-3 [80,83]`; a WAIT on one arm only and an unmatched RUN FIRST raise the v53 errors; a load-and-save round trip is accepted by v53 and plans the same stages; a `side=left` file is refused on the RIGHT channel.
- [ ] In a fake two-arm run, one STOP ALL click signals every active runner within 200 ms with no dialog, no later stage starts, and a second click sends no second signal.
- [ ] In a fake "both arms together" stage where LEFT prints a 4109 line or exits with rc=1, RIGHT is signalled within 500 ms and the recipe ends "Stopped: left arm failed"; in a serial handoff the waiting arm never starts after a failure or a stop.
- [ ] Changing the LEFT IP changes `--ip` on every later LEFT command; identical IPs are refused; test connection reports Connected (mode and stored programs) or Not reachable within 10 s; a slot whose stored name or size differs from Lite's upload record blocks the recipe and names the slot (engineer) or the region (operator).
- [ ] In controller SIM, the stage files contain no pole or hand steps and the run screen says they were skipped; stopping the server with a runner active signals it and waits for its exit; the server listens on 127.0.0.1 and motion routes reject requests without the session token; operator screens show no slot numbers, Program IDs, `rc=` or `TIMELINE-`; each run keeps one log per arm across stages and page reloads.
- [ ] With a fake SDK, an E-stop on either arm during a two-arm recipe stops every later stage on both arms, marks the recipe "Interrupted: E-stop", and refuses new motion on both arms until recovery; uploads still work. A batch upload to Program IDs chosen per file shows the current program of each occupied slot before replacing it.
- [ ] On hardware, in order: E-stop drill DR1, DR3, DR9, DR10 and DR11; G4 probe on both controllers; SIM behaviour of pole and hand; upload and verify one `.txt` per arm to a chosen Program ID, replacing an existing program after the confirmation; the V6 recipe end to end in controller SIM with the lid prompt; one region's recipe in REAL; STOP ALL during REAL motion stops both arms, with the start skew recorded.

**For operators:**
- An always-visible Robot bar shows each arm in plain words ("Left arm: Connected · Real robot · E-stop clear · Cleaning 'Lid top' (step 3 of 9) · 01:12"), with STOP LEFT, STOP RIGHT and a red STOP ALL (software stops) that never ask for confirmation. When an E-stop button is pressed the bar turns red and says which arm.
- Pick a named recipe, confirm once (arms, mode, recipe) and press Start. Prompts appear only when a person must act, for example "Open the toilet lid fully, step out of the robot area, then press Continue".
- Plain words replace control-app terms: "Both arms together", "Left arm first, then right", "Dry run on the controller: the arm does not move; pole and hand steps are skipped", "Real robot: the arm moves".
- An arm's controls are disabled on every screen while it runs a recipe, and the screen says why.

**Safety:**
- The IO-wired E-stop buttons on both controllers are the safety stop; Lite shows their state, refuses motion while either is active and never writes IO modes. Lite's STOP is a convenience, not a safety function, and is tested (G4). If the backend crashes, stored programs keep playing; the E-stop still works without the PC (drill DR12).
- Measure G4 before operators use REAL mode. Until it passes, "both arms together" blocks run only with an engineer present, as ALIX's own dual executive demotes them (`alix/tools/dual_arm_flow.py:49-53`).
- In v53 one arm's failure lets the other finish its block (`lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py:10273-10300`); Lite adds cross-arm abort. Pole and hand never move during or just after that arm's run, or with the pad on a surface.
- Pin the runner (a v13 runner would run v51+ approach moves at 100 %), keep one command source per controller (ROS driver down, Tk app closed), and keep global-point moves and drag playback engineer-only at low speed, because STOP acts only after these blocking calls return (inferred).

| Priority | Size | Depends on |
|---|---|---|
| Must (Phase 0 and Phase 1); Should (Phase 2) | XL (MVP with the recipe builder deferred: L; Phase 0: S; Phase 2: L) | NFR-THEME, NFR-UX; both controllers and arm time for the E-stop drill and G4; the RealMan SDK for x86-64 (operator PC) and aarch64 (Jetson); agreement on controller ownership (one command source). Required by R1, R4, R5, R7, R8, R10, R13 |

**Open questions:**
- If G4 fails, should "both arms together" blocks run one arm at a time (as ALIX does), or should the runner gain a stronger stop first?
- What do a pole command and a hand command actually do while a controller is in SIM: no motion, a time-out, or real motion?
- Should the lid checkpoint stay "a positive WAIT on both arms" (the v53 contract), or become an explicit step type exported as that WAIT?
- Will v53 stay as the engineering fallback? If so, who fixes its Multi Run and Load Sequence `TypeError`?
- Do operators still need a feature dropped before v48: `DT_` save-to-controller, the LEFT-then-RIGHT or ALT modes, or the "% open" hand slider?
- How are the operator and engineer roles separated: a role switch or a PIN?
- Should REAL motion stop automatically when the browser stops polling, and after how many seconds?
- On the Jetson, does the runner run unchanged with the aarch64 SDK and the same controller IPs, and where does the operator view the Lite screens (a browser on the Jetson, or on a PC connected to it)?
- Phase 2: should `alix_dispatch` be vendored into Lite, or pinned to an ALIX revision?

---

## 11. Open questions for the requester and the team

The implementation plan (section 11) keeps the full decision log, D1–D27. Each requirement section also lists its own smaller questions.

**Decided on 13 October 2026**

| ID | Decision |
|---|---|
| D1 | Operator PC now; the Jetson AGX Orin in production. Both builds are required. |
| D2 | Any program ID 1–100, as in the control program. An occupied slot is replaced after a confirmation. |
| D3 (part) | Fixture: commode C. First release: 34 units with codes in the deck's own format. Glove and bin tasks are out for now, and so is the drain hole (it needs the brush). Lid and seat operations, opening and closing, are in, and they are not cleaning tasks. The seven changes from the deck are agreed (Appendix A). |
| D4 | The operator's time for a region is the optimiser's target. If it cannot be met within the limits, Lite returns the closest feasible time and names the binding limit. During teaching the same time drives a guidance countdown. |
| D7 | No calibration project. The URDF and the `commode_c` scene hold the cell. |
| D10 | Low spikes are sudden downward jumps of a joint curve or its speed (▼), the mirror of high spikes (▲); both get a "smooth this spot" suggestion. Hesitations are not spikes; they stay an information note (R12 (f)). |
| D11 | Gate strictness: G5 (predicted path departure) is a warning only, because ALIX's model reads 33–109 mm on the raw commode C recordings, which ran fine on the robot, so it cannot gate a file. A region's MAX DEV tightens the path-accuracy limit L5 instead. |
| D15 | Code in `alix_studio_lite/`; documents in `alix_studio_lite/docs/`. Everything Lite needs lives inside that folder: the reused ALIX code and data are vendored into `vendor/` with a manifest, so no ALIX checkout is used at run time. |
| D21 | Walls: a single cubicle, not the whole washroom. It is 1400 mm wide by default and can be switched to 800 mm. The 1400 mm cubicle is built (`assets/scene/cubicles/cubicle_1400mm.stl` and `.yaml`), the same way as ALIX's 800 mm one. |
| D23 | 600 °/s² is the arm's hardware acceleration limit: hard for both optimisers, with no override. |
| D24 | Operators and engineers can both switch a region between the Default and the Time-optimal optimiser; Default is preselected. |
| D25 | The E-stop is on IO3 of each controller, in E-stop mode. The stop holds until the button is released (the manual reset). Lite blocks motion if IO3 is not in E-stop mode. |
| D26 | Clock: 5 ms per line is the only time base; durations and targets are (lines − 1) × 5 ms. |
| D27 | Order: UI/UX first; backend on hold at its checkpoint. |
| D28 | Dual-arm timing margin: kept. Arms running together never finish in perfect step, so dual-arm clearance checks keep the run-to-run timing spread measured by ALIX. |
| D29 | Fixture pool: commodes A, B and C, urinal A and wash basin A, commode C by default. Commode C uses the deck catalogue; the others start with a provisional one from ALIX's regions.yaml. |
| D30 | Three modes: Visual sim (the 3D robot moves, no hardware), Hardware sim (the controller's SIM mode, arms still), Real (arms move, after a confirmation). |
| D31 | An existing taught recording for a region is used by default; re-teaching live is an option, and a recording can be picked from the library or uploaded. |
| D32 | Tabs at the top, as in ALIX Studio (Regions, Teach, Optimise, 3D view, Run; Robot and Settings for engineers; Help); every screen reflows with the window size. |
| — | Safety stop: the IO-wired E-stop buttons are primary, and the software STOP will be tested. |

**Still open: needed early**

1. **Region catalogue sign-off (D3).** The codes and the first-release scope are agreed. Still to fill in, in Appendix A and the sheet `region_catalogue_commode_c.csv`:
   - a target time for each of the 34 units;
   - confirmation of the proposed pole heights and the GRIP hand;
   - the teaching order: all units of one arm per toilet state, or strict deck order.
2. **Cubicle check (D21).** Once on site:
   - confirm the cubicle is 1400 mm wide and its back wall is 1.215 m in front of the robot base;
   - name the known-good recordings that verify it. They must show no wall contact.
3. **E-stop drill and software STOP test.** Book about one lab day for drills DR1–DR14 and the G4 probe. The drills confirm:
   - IO3 is in E-stop mode on both controllers, and its active level;
   - whether one button trips both arms;
   - whether the drives lose enable;
   - whether a moving pole stops;
   - whether a slow stop ends a running stored program.

   Operator real tests stay blocked until DR1, DR3, DR9, DR10 and DR11 of the drill pass.

**Still open: needed later**

4. **Hand presets and contact rules (D5, D8).** V6 GRIP (fist) or ALIX's "fingers open, thumb held" per region? With GRIP, which hand links may touch the toilet without counting as a collision?
5. **Pole defaults (D5).** V6 values (50 / 150 / 200 mm) or the ALIX commode C sheet (140 / 200 mm)?
6. **Bin (D6).** Which state is used during teaching and production? Has the stowed-bin pole band been confirmed on the installed bin?
7. **Collision-detection level (D9).** Align both arms (left 3, right 0 at the last check).
8. **Dry-run rule (D12).** Required for every changed file before a real run, or only before upload?
9. **Executor and seams (D13, D14).** Agree the small runner v14 patch for E-stop reporting, and the small upstream changes (S1–S7) with the ALIX owner.
10. **Jetson screen (D22).** Does the production Jetson have its own screen? If not, the UI is opened from the operator PC through an authenticated connection.
11. **Data, legacy and teaching (D17–D20).**
    - Where project data lives and how it is backed up.
    - Whether V6 segments appear as "legacy" on the dashboard.
    - One or two operators during teaching, and whether to use a foot switch.
    - Whether the RealMan SDK may be bundled.

---

## Appendix A — Region catalogue for sign-off (commode C)

The regions deck (`lite/Toilet_regions_V1.pptx`) describes **commode C** (ALIX `commode_c`). Every unit below has a code in the deck's own format, `<L|R>-<R|T><NN>`:

- `L` or `R` is the arm.
- `R` is a cleaning region and `T` a task.
- `NN` is the number. Where the deck prints a code it is kept; where it prints none, the code fills the gap in the deck's own numbering (marked *new code*).
- Parts are written as in the deck: `PART 1`, `PART 2`.
- In file names and in the software, the key is the code plus `_PART<n>` for units with parts (for example `L-R05_PART1`), and the code alone otherwise (`L-R04`).

**First release (decided 13 October 2026): 34 units — 26 cleaning regions and 8 lid and seat operations.**

- **Lid and seat operations are included (opening and closing, confirmed 13 Oct), and they are not cleaning tasks.** They open or close the lid and the seat, with the left arm as MASTER and the right arm as SUPPORT. They change the toilet state for the regions that follow; their target time is the duration of the operation, not a cleaning time.
- **Glove tasks and bin tasks are not included for now.** "Bin tasks" is read as the drain-brush tasks (pick, hold and return the brush, which lives in the robot's bin) and the glove drop. Cleaning the drain hole needs the brush, so `R-T05` is out too (confirmed 13 Oct).
- While the left arm cleans the inner rim, bowl inside and left floor spillage (steps 14–19), the right arm has no taught unit in this release and stays parked.

**Proposed settings** (still to confirm at sign-off):

- **Pole heights** come from the V6 workflows; values marked \* have no V6 program and copy the nearest similar unit.
- **Hand:** every unit uses the V6 GRIP preset `0, 0, 0, 0, 150, 1000`.
- **Target times:** the deck gives none, so they are left to set at sign-off. The CSV sheet has a column for them.

**V6 column:** what the merged V6 package already holds for each unit:

| Value | Meaning |
|---|---|
| covered | the same arm, inside a merged program |
| merged | two deck parts in one V6 file |
| partial | V6 splits the area differently |
| other arm | taught on the other arm |
| none | not taught |

No unit has a per-region raw recording yet.

### LEFT arm

| Step | Deck task | Code | Name | Type | Toilet state | First release | Pole (mm) | V6 | Deck note |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 1A | `L-R01` | BASE LEFT | Cleaning | Lid closed | **Yes** | 50 | covered | as printed |
| 2 | 2A | `L-R02` | SIDE EXTERIOR LEFT | Cleaning | Lid closed | **Yes** | 50 | covered | as printed |
| 3 | 1C | `L-R03` | PEDESTAL UPPER LEFT | Cleaning | Lid closed | **Yes** | 50 | covered | as printed |
| 4 | 2 | `L-R04` | LEFT HINGE | Cleaning | Lid closed | **Yes** | 150 | covered | new code (deck: no code) |
| 5 | 3 | `L-T01` | OPEN LID + SEAT (MASTER) | Lid/seat operation (not cleaning) | Lid closed → lid and seat up | **Yes** | 150* | none | new code (deck: no code) |
| 6 | 4 | `L-T02` | CLOSE SEAT ONLY (MASTER) | Lid/seat operation (not cleaning) | Lid and seat up → seat down (lid open) | **Yes** | 150* | none | new code (deck: no code) |
| 7 | 5A | `L-R05` | SEAT TOP PART 1 | Cleaning | Lid open | **Yes** | 150 | covered | new code (deck: no code) |
| 8 | 5B | `L-R05` | SEAT TOP PART 2 | Cleaning | Lid open | **Yes** | 150 | other arm | new code (deck: no code) |
| 9 | 6 | `L-R06` | LID UNDERSIDE LEFT HALF | Cleaning | Lid open | **Yes** | 150 | covered | new code (deck: no code) |
| 10 | 7 | `L-T03` | OPEN SEAT (MASTER) | Lid/seat operation (not cleaning) | Lid open → seat up (seat open) | **Yes** | 150 | covered | as printed |
| 11 | 8A | `L-R07` | RIM TOP PART 1 | Cleaning | Seat open | **Yes** | 150 | covered | new code (deck: no code) |
| 12 | 8B | `L-R07` | RIM TOP PART 2 | Cleaning | Seat open | **Yes** | 150 | other arm | new code (deck: no code) |
| 13 | 9 | `L-R08` | SEAT UNDERSIDE LEFT HALF | Cleaning | Seat open | **Yes** | 150 | covered | new code (deck: no code) |
| 14 | 10A | `L-R09` | INNER RIM PART 1 | Cleaning | Seat open | **Yes** | 150 | covered | deck prints FL-R09 |
| 15 | 10B | `L-R09` | INNER RIM PART 2 | Cleaning | Seat open | **Yes** | 150 | other arm | deck prints FL-R09 |
| 16 | 11A | `L-R10` | BOWL INSIDE PART 1 | Cleaning | Seat open | **Yes** | 50 | merged | as printed |
| 17 | 11B | `L-R10` | BOWL INSIDE PART 2 | Cleaning | Seat open | **Yes** | 50 | merged | as printed |
| 18 | 12A | `L-R11` | FLOOR SPILLAGE LEFT PART 1 | Cleaning | Seat open | **Yes** | 50* | none | as printed |
| 19 | 12B | `L-R11` | FLOOR SPILLAGE LEFT PART 2 | Cleaning | Seat open | **Yes** | 50* | none | as printed |
| 20 | 13 | `L-T06` | DROP GLOVE | Glove task | Any | No | — | none | new code (deck: no code) |
| 21 | 14 | `L-T04` | LEFT GLOVE CHANGE | Glove task | Any | No | — | none | as printed |
| 22 | 15 | `L-T05` | CLOSE SEAT + LID (MASTER) | Lid/seat operation (not cleaning) | Seat open → all closed | **Yes** | 150* | none | as printed |

### RIGHT arm

| Step | Deck task | Code | Name | Type | Toilet state | First release | Pole (mm) | V6 | Deck note |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 1A | `R-R01` | LID TOP PART 1 | Cleaning | Lid closed | **Yes** | 150 | partial | deck prints PART 01 |
| 2 | 2A | `R-R01` | LID TOP PART 2 | Cleaning | Lid closed | **Yes** | 150 | partial | deck prints PART 02 |
| 3 | 1C | `R-R02` | RIGHT HINGE | Cleaning | Lid closed | **Yes** | 50 | covered | as printed |
| 4 | 2 | `R-R03` | SIDE EXTERIOR RIGHT | Cleaning | Lid closed | **Yes** | 50 | covered | new code (deck: no code) |
| 5 | 3 | `R-T01` | OPEN LID + SEAT (SUPPORT) | Lid/seat operation (not cleaning) | Lid closed → lid and seat up | **Yes** | 150* | none | new code (deck: no code) |
| 6 | 4 | `R-T02` | CLOSE SEAT ONLY (SUPPORT) | Lid/seat operation (not cleaning) | Lid and seat up → seat down (lid open) | **Yes** | 150* | none | new code (deck: no code) |
| 7 | 5A | `R-R04` | LID UNDERSIDE RIGHT HALF | Cleaning | Lid open | **Yes** | 150 | covered | new code (deck: no code) |
| 8 | 5B | `R-R05` | PEDESTAL UPPER RIGHT | Cleaning | Lid open | **Yes** | 50 | covered | new code (deck: no code) |
| 9 | 6 | `R-R06` | BASE RIGHT | Cleaning | Lid open | **Yes** | 50 | covered | new code (deck: no code) |
| 10 | 7 | `R-T03` | OPEN SEAT (SUPPORT) | Lid/seat operation (not cleaning) | Lid open → seat up (seat open) | **Yes** | 150 | covered | as printed |
| 11 | 8A | `R-R07` | SEAT UNDERSIDE RIGHT HALF | Cleaning | Seat open | **Yes** | 150 | covered | new code (deck: no code) |
| 12 | 8B | `R-R08` | FLOOR SPILLAGE RIGHT PART 1 | Cleaning | Seat open | **Yes** | 50 | merged | new code; deck: FLOOR SPILLAGE PART 1 |
| 13 | 9 | `R-R08` | FLOOR SPILLAGE RIGHT PART 2 | Cleaning | Seat open | **Yes** | 50 | merged | new code; deck: FLOOR SPILLAGE PART 2 |
| 14–15 | 10A–10B | `R-T04` | PICK DRAIN BRUSH | Brush/bin task | Seat open | No | — | none | as printed (also printed on 10B) |
| 16–17 | 11A–11B | `R-T10` | HOLD BRUSH OUTSIDE | Brush/bin task | Seat open | No | — | none | new code (deck: no code; tasks 11A-11B) |
| 18 | 12A | `R-T05` | CLEAN DRAIN HOLE PART 1 | Cleaning with the drain brush | Seat open | No | — | none | as printed (PART1) |
| 19 | 12B | `R-T05` | CLEAN DRAIN HOLE PART 2 | Cleaning with the drain brush | Seat open | No | — | none | as printed; same area as part 1 |
| 20 | 13 | `R-T06` | RETURN DRAIN BRUSH AND DROP GLOVE | Brush/bin task | Any | No | — | none | as printed |
| 21 | 14 | `R-T07` | RIGHT GLOVE CHANGE | Glove task | Any | No | — | none | as printed |
| 22 | 15 | `R-T09` | CLOSE SEAT + LID (SUPPORT) | Lid/seat operation (not cleaning) | Seat open → all closed | **Yes** | 150* | none | as printed |

### Changes from the deck (agreed 13 Oct 2026)

1. `FL-R09` (inner rim) is written `L-R09`, the deck's own format for a left-arm cleaning region.
2. New codes fill the gaps in the deck's numbering:
   - left: `L-R04` to `L-R08`, `L-T01`, `L-T02`, `L-T06`;
   - right: `R-R03` to `R-R08`, `R-T01`, `R-T02`, `R-T10`.
3. `R-T08` does not appear in the deck, so the code is left unused.
4. Task 10B repeats *R-T04 PICK DRAIN BRUSH*; it probably means *hold brush outside* (`R-T10`). Both are out of this release.
5. The right arm's *FLOOR SPILLAGE PART 1/2* becomes *FLOOR SPILLAGE RIGHT PART 1/2*, to match the left arm's name.
6. The lid top's *PART 01/02* is written *PART 1/2*, like every other part.
7. The two drain-hole parts mark the same area, so they are probably two passes. They are out of this release.

The sign-off sheet `region_catalogue_commode_c.csv` (next to this document) has the same rows with empty columns for the target time and for who confirmed each row.

---

## Appendix B — Evidence base

The analysis behind this document read the sources listed in section 3 in full or by targeted search, and checked every borrow claim against the code. The main evidence files are:

| Area | Main files read |
|---|---|
| ALIX Studio (front and back end) | `studio/server.py`, `studio/web_editor/template.html`, `studio/web_editor/editor.js`, `studio/web_editor/editor.css`, `studio/body_collision.py`, `studio/sim_bridge.py`, `studio/robot_model.py`, `studio/paths.py`, `alix/design/_tokens.css`, `alix/docs/STUDIO_REDESIGN.md`, `alix/docs/studio/*`, Studio tests (`test_design_tokens.py`, `test_studio_theme.py`, `test_studio_interaction_guards.py`) |
| Kinematics and collision | `alix/core/python/alix_workspace/src/alix_workspace/` (`urdf_kinematics.py`, `ik.py`, `srs_ik.py`, `srs.py`, `collision.py`, `dual_arm_plan_collision.py`, `plan_scene_validation.py`, `urdf/alix.urdf`) |
| Robot I/O and taught tracks | `alix/core/python/alix_dispatch/src/alix_dispatch/` (`track.py`, `connect.py`, `runmode.py`, `peripherals.py`, `emulated_program.py`), `alix/tools/` (`taught_tracks.py`, `rm_upload.py`, `cleaning_tracks.py`, `dual_arm_flow.py`, `dual_arm_verdicts.py`, `plan_min_time.py`), `alix/docs/motion/*`, `alix/Resource/tracks/taught/README.md`, `alix/experiments/exp01_teach_trajectory_reverse_engineering/` |
| Robot description | `alix/ros/description/` (`alix_description`, `ugv_description` incl. the bin, `rm_description`, poles, Inspire hands), `alix/BIN_HEIGHT_HANDOFF.md`, `alix/docs/EMULATOR.md` |
| Trajectory prototype | `lite/Alix_Studio_Lite_Trajectory_V2.html` (read in full) |
| Control application | `lite/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py` (and v48, v51, v52), `lite/upload_channel/rm_upload_fast_v14_safe_transit.py` (and v13), `pole.py`, `hand.py`, `usage.py`, `README_COMMANDS.md`, `upload/rm_upload.py`, `py programs/` (version history), `lite/RealMan Dual-Arm Operations UI.txt` |
| Regions and data | `lite/Toilet_regions_V1.pptx` (all 25 slides, text and images), `lite/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/` (programs, bridges, JSON, reports, checksums), `lite/upload_channel/zip/` and `plan_txt_files/` |

The detailed, line-referenced analysis notes behind each section are in `analysis/` next to this document: 13 source maps, 8 verified requirement analyses, the E-stop and optimiser research, and the three architecture proposals.
