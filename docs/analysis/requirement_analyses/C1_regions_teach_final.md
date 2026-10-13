# C1 · Regions and teaching: R1 and R7 (verified final)

Cluster `C1_regions_teach`. The requirements are **R1**, region-wise data collection, and **R7**, pole height and hand position. Both come from
`docs/reference/ALIX_Studio_Lite_requirement_inputs.txt`, lines 8 and 20.

This document is the analyst draft after an adversarial verification pass on 2026-10-13. I re-opened every cited line and every claim
that something can be borrowed. Claims that did not hold are corrected. Items the draft missed are added. Section 4 is the change log.
All work was read-only. Nothing was run against a robot or a network.

## Path legend

| Alias | Absolute path |
|---|---|
| `TG` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen` (ALIX Studio) |
| `WE` | `TG/web_editor` (`template.html`, `editor.js`, `editor.css`) |
| `SRV` | `TG/server.py` |
| `UC` | `/home/newtonjeri/alix_studio_lite_ws/upload_channel` |
| `UI53` | `UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py` (control-app UI, current) |
| `RUN14` | `UC/rm_upload_fast_v14_safe_transit.py` (control-app runner, current) |
| `ALIX` | `/home/newtonjeri/alix_ws/src/alix` |
| `ORCH` | `ALIX/ros/orchestration/alix_orchestrator/alix_orchestrator` |
| `EXP01` | `ALIX/experiments/exp01_teach_trajectory_reverse_engineering` |
| `TT` | `ALIX/Resource/tracks/taught` |
| `SDKI` | `ALIX/external/butterfli-realman-alix/Python/Robotic_Arm/rm_robot_interface.py` |
| `EMU` | `ALIX/core/python/src/alix_emulator/rm_emulator.py` |
| `V6` | `/home/newtonjeri/alix_studio_lite_ws/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES` |
| `HTML` | `/home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html` |
| `SP` | `/tmp/claude-1000/-home-newtonjeri-alix-studio-lite-ws/747fb97d-6769-47bf-aaab-d34d82139984/scratchpad` |

---

## Summary

| | R1 · Region-wise data collection | R7 · Pole height and hand position |
|---|---|---|
| **Can it be borrowed from ALIX Studio?** | **Partly.** Borrow the look of the Board tab: its CSS, summary chips, progress badge on the tab, and status pills that pair a glyph with a word. Also borrow the feedback components, the theme tokens and the fixture-state check logic in `schedule_guards`. The Board's JavaScript and backend are tied to the excluded path generator, so write those new. Do not port the Studio Regions tab: it is a 3-D mesh-painting editor for the generator. The guided prompt, countdown, auto-advance and drag recording do not exist in Studio. | **No, for what R7 asks.** Studio never commands the robot and has no "apply before teaching" step. The concept does exist inside the excluded generator. Its Workbench writes a pole height into each generated task, and the hand state follows from the chosen pad. Lite can reuse that UI pattern (named heights, a custom height and a bin warning), the stowed-bin rule and the 3-D model. |
| **Where the working parts are** | Control-app runner v14: drag start, stop, save and playback, plus the two-arm start. The ALIX taught lane: the `SOURCE.json` provenance format and the rule that source files are never edited. The trajectory HTML's stepper as a pattern for the guided flow. The deck renders and the map-13 ID scheme. | Control-app pole and hand commands (`--manual-pole`, `--manual-hand-raw`, `--manual-hand`). `usage.py` supplies the rules, the arm-still check, the "prestroke" order and read-only `pole read` / `hand read`. `pole.py` and `hand.py` are reusable. The ALIX taught lane records pole and hand per recording. V6 and the ALIX commode C sheet give default values. |
| **Input note** | No explicit claim. The named "Regions Tab" exists in Studio, but its content cannot be reused. | "borrow from alix studio" is **mostly inaccurate**: the concept exists in Studio, the working machinery does not (§2.5). |
| **Priority · effort** | Must · XL (about 27 person-days; an MVP is about 15) | Must · M (about 7-8 person-days; +1 if tool and glove tasks are in v1) |

## Verification summary

**Confirmed.** The following draft claims hold:
- the Studio Regions tab is a painting editor (`WE/template.html:156-201`);
- the Board tab's markup, JavaScript and CSS are at the cited lines;
- the Board backend reads generated configs (`SRV:5396-5486`);
- Studio has no wizard and no execution channel (`WE/template.html:116-118`, `WE/editor.js:13045-13047`);
- the feedback toolkit is at the cited lines;
- the control app's drag teach, save, both-arm start, pole and hand code is at the cited lines;
- the TypeError methods are at `UI53:4853, 4876, 4885, 5000`;
- the SDK docstrings, the `SOURCE.json` schema and its reader and writer, the V6 pole values and the commode C teach sheet are as described;
- the recomputed commode C durations match: cleaning regions 41.7-66.0 s (median 48.2 s) and mechanisms 16.5 and 23.7 s at 5 ms per point;
- the deck has 25 slides and no slide text mentions time, pole or hand (checked by searching all 25 slide XMLs).

**Corrected.**
1. **R7 input note.** It is "mostly inaccurate", not simply inaccurate. The Studio Workbench has a per-task "Pole (written to the task)" field and a pad-derived hand state (`WE/template.html:229-238`).
2. **Per-program pole and hand records.** Multi Run is not the only one. The working Page-02 workflow format, which V6 uses, also carries pole and hand steps.
3. **Read-back.** A read-only pole and hand reading already exists in `usage.py`.
4. **Bin interlock.** It must check the whole pole travel, not only the target.
5. **Bin state names.** Use ALIX's names: STOWED, EXTENDED, LOWERED.
6. **Bin band.** It is provisional, because the physical bin has not been remade.
7. **Limp arm.** The risk is when the arm enters drag mode, not when it stops.
8. **Emulator.** It runs in-process only, so it cannot hold state across runner subprocesses.
9. **Unsupported adjectives.** "Hardware-proven" and "in daily use" are removed: no test record was found.
10. **Marker string.** The runner prints `DRAG SAVE COMPLETE: <path> // <N> points`.
11. **Practice mode.** Two acceptance criteria contradicted each other; practice recordings now go to a separate folder.
12. **Effort.** Recomputed from the item list.

**Added (missed by the draft).**
1. **Other clients.** No other client may drive the arms. The ALIX ROS stack holds the arm at 100 Hz, and the runner says to stop the ROS driver first.
2. **Drag preconditions.** The runner does not check them before drag teach: all joints enabled, no error codes, no program running.
3. **Orphaned drag sessions.** A drag session can outlive a Lite crash and keep the arm in drag mode. The controller still holds the recording, so it can be saved afterwards.
4. **Save via a temporary name.** Save to a temporary name, compare the controller's point count with the file, then rename.
5. **Sub-steps.** Mechanism regions, and tool and glove tasks that need a hand action mid-task, are taught as sub-steps.
6. **Downstream values.** Recorded pole and hand values are authoritative downstream.
7. **Thumb-held presets.** ALIX found that commanded thumb channels sometimes do not move, so thumb-held presets should be the default.
8. **Physical DRAG button.** The arm has its own DRAG button, which bypasses Lite.
9. **Record period.** There is evidence for 5 ms per point, but it is still marked open; Lite stores the measured period.
10. **Control lock.** Only one browser session may command the robot.
11. **Foot-switch key.** It must not act on whatever control has focus.

---

## 0. Background the two requirements share

### 0.1 Terms (proposed for Lite; please confirm)

- **Region.** The unit of teaching: one arm's task in one deck step part, taught as one drag-and-teach recording. IDs follow the map-13 scheme
  `<ARM>-<KIND><NN>[-P<n>|-S<n>]`, where KIND `R` is a surface wiped for a time and `T` is a task (mechanism, tool, glove, idle). Examples:
  `L-R01` "Base, left side" and `R-R01-P1` "Lid top, rear half". The printed deck code and label are kept as aliases (`FL-R09` becomes `L-R09`).
  Toilet_regions_V1 gives **42 regions: 22 LEFT and 20 RIGHT**. That is 28 cleaning regions, including the 2 drain-hole passes, and 14 task regions.
- **Sub-step** (`-S<n>`). An ordered part of one region, taught as its own recording. It is used in two cases:
  1. MASTER/SUPPORT mechanisms taught by alternating arms (V6 runs L8 → R8 → L9 → R9).
  2. Tool and glove tasks that need a hand action part-way. A drag-teach file holds arm joints only, and Lite forbids hand commands in drag mode,
     so the hand action happens between two sub-steps.

  A region counts as taught only when all its sub-steps are kept.
- **Region family.** A named surface on deck PAGE 04/05 (12 of them) plus Floor Spillage. Families group regions on the overview map.
- **Step.** One deck task card (1A … 15) that pairs a LEFT and a RIGHT region. Steps are numbered `ST01..ST22` in deck order, because the
  deck's own task numbers are irregular.
- **Segment.** The UI word for R2's "sub-region": a time window inside one recording. Deck "PART 1/2" items are separate regions, not segments.
  The trajectory HTML calls its time windows "scaling regions" (`HTML:392-400`); they must be renamed when the HTML is integrated.
- **Raw recording.** The file the controller writes for a drag-teach recording (`rm_save_trajectory`), unmodified, and kept by the operator.
  The file itself carries no units, timestamps, frame or arm (`EXP01/src/teachlab/teachfile.py:3-12`), so its provenance sidecar is the only
  record of how it was taught. V6 programs and edited files are not raw.
- **Cleaning time.** The target duration of a teach recording, shown as a countdown. This is an assumption to confirm (open question R1-1).

### 0.2 What the control-app programs do (high level only)

Current pair: **UI v53 + runner v14**. `UI53:45-46` declares UI v53 and expects runner v14. The two other v14 files,
`…_v14_safe_transit_v52support.py` and `…_v53support.py`, are byte-identical copies (same md5). `UC/py programs/` holds the history: UI v8 … v48,
runners v4 … v12 and the original `rm_upload.py`. `UC/upload/` holds the original uploader with UI v8 and the helpers. v48, named in earlier notes,
is three versions older than v53 and contained in it.

| File | What it does |
|---|---|
| `rm_trajectory_ui_v53_safe_lid_checkpoint.py` | Tkinter desktop console. Each arm has a panel with the tabs RUN, POINTS, DRAG TEACH, HAND / POLE and TELEMETRY (`UI53:2156-2166`). A second page holds per-arm workflow builders with program, global point, pole, hand and wait steps (`UI53:6604-6656`), and the dual rules TOGETHER, RUN FIRST and WAIT OTHER. Each button starts the runner as a separate process (`python -u runner --sdk …`, `UI53:4255-4262`) in its own session (`UI53:5745-5750`). The UI reads the runner's printed status lines and stops it with SIGINT; on Windows it sends CTRL_BREAK (`UI53:3897-3901`). |
| `rm_upload_fast_v14_safe_transit.py` | Command-line runner. Each process connects to one arm controller: `--side` selects 192.168.1.10 or 192.168.1.103, and `--ip` or `RM_LEFT_IP`/`RM_RIGHT_IP` override it (`RUN14:46-49`). It uploads and verifies programs and runs programs, sequences and workflows. It starts, stops, saves, replays and stores drag-teach recordings. It moves the pole and the hand, reads and sets the controller's SIM/REAL mode, and moves to global points. |
| `pole.py`, `hand.py` | Small device classes. Pole: 5 mm tolerance, waits out a modelled travel time, stops on timeout. Hand: Inspire hand over Modbus, 0-1000 per channel, 1000 = open, `freeze()` holds the current position. |
| `usage.py` | Standalone pole and hand tool: `pole read/move/stop`, `hand read/open/close/pct/set/freeze`, `prestroke`, `stop`. It states the safety rules (`UC/usage.py:5-25`). |

Plan-blocking items for this cluster, one line each:
1. Multi Run and Page-01 Load Sequence raise `TypeError` in v53 (`UI53:4853, 4876, 4885, 5000`), so Lite must not rely on Multi Run's per-program pole and hand fields.
2. v53 sets the drag save path once and never resets it (`UI53:3937-3938`), so a second save overwrites the first unless the operator edits the path. Lite names files itself.
3. The runner saves once, writes straight to the final path and does not check the file (`RUN14:957-989`), while ALIX saw the save lag and retries every second (`EXP01/notes/2026-08-25_phase_a.md:93-94`). Lite retries and checks.
4. The UI has no IP entry, and its default SDK path is in another machine's home directory (`UI53:48-51`). Lite passes `--ip` and `--sdk`.
5. Drag-session processes run in their own session (`UI53:5745-5750`), so they outlive a crashed parent and keep the arm in drag mode. On Windows the stop signal is not turned into the runner's stop path (inferred, map 09). Lite must supervise these processes and run the hardware layer on Linux.

---

## 1. R1 · Region-wise data collection

### 1.1 Original text

> Region-wise Data Collection: The UI should prompt the (non technical person) user to clean each region with a specified cleaning time. Once a
> region is completed, the system should automatically prompt the next region. (Regions Tab -> Regions PPT and the Dashboard to show the regions
> with and withput the taught raw txt)

### 1.2 Revised requirement

Lite guides a non-technical operator through drag-and-teach recording of every region in the cleaning plan, one region at a time. It also shows
which regions already have a kept raw recording.

- **(a) Region catalogue.** Lite holds one record per region, built from the regions deck. Each record has:
  - the ID and deck aliases, the arm, and the step with its deck task and slide;
  - the type: surface, mechanism, tool, glove or idle;
  - the required lid/seat state, the tool, the picture and the target cleaning time;
  - the pole height and hand setting (R7);
  - for mechanism, tool and glove tasks, the ordered sub-steps with the hand events between them.

  Toilet_regions_V1 gives 42 regions: 22 LEFT and 20 RIGHT.
- **(b) Guided session.** The operator chooses Left, Right or Both arms. Both needs two people. Lite shows the next region without a kept
  recording as a card: the deck picture with the area highlighted, a plain-language instruction, the arm, "part n of m", the tool and the time.
  The operator presses **Prepare** (R7) and then **START**. Opening a card or auto-advancing never moves the robot.
- **(c) Cleaning time.** While recording, Lite shows a countdown of the region's target time and sounds at 10 s left and at 0 s. Recording
  continues after 0 s until STOP. A configurable hard limit stops recording automatically and is announced 10 s ahead.
- **(d) Save.** On STOP, Lite saves the controller's recording under a temporary name and checks it: the line format, at least 2 points, and
  the controller's point count equal to the number of lines. It then renames the file to a new name made of the region ID and the time; an
  existing file is never replaced. It writes a provenance sidecar with:
  - arm, region, deck version, lid/seat and bin state;
  - pole and hand, target and measured;
  - operator, date, controller IP and the controller's tool frame;
  - target time, wall-clock time, point count, measured record period and checksum.

  Lite reports any problem in plain words.
- **(e) Review.** The operator chooses Keep or Redo. A redo keeps the earlier attempt, marked rejected. Skip needs a reason.
- **(f) Auto-advance.** After Keep, the next card opens without further input. Lite skips kept and blocked regions and inserts lid/seat change
  prompts until the plan is done. A session can be paused and resumed.
- **(g) Regions tab.** It shows the deck overview maps (PAGE 04/05) coloured by status, the step list grouped by lid/seat state, the region card,
  and the original slide on request.
- **(h) Dashboard.** A region × arm table shows Not taught, Awaiting review or Taught (with attempts, date and duration). It also shows the later
  pipeline states (optimised, simulation OK, real-tested, uploaded) when their records exist. It has counters per arm and per phase, a
  "missing only" filter and a "Teach next missing" action.
- **(i) Both arms.** One START puts both arms in drag mode. STOP saves two separate files.
- **(j) Interlock.** While any arm is in drag mode, Lite sends no other robot command. During a session no other client may drive the arms: not
  the ALIX ROS stack, the v53 app or a second Lite. Only one browser session holds control. STOP is always available.
- **(k) Recovery.** If Lite stops while an arm is in drag mode, on restart it takes the arm out of drag mode and offers to save the recording the
  controller still holds.
- **(l) Practice mode.** A simulated recorder for training and tests writes to a separate practice folder. Its files are never counted as taught.

### 1.3 Clarifications and assumptions

- "Region", "sub-step", "region family", "step", "segment", "raw" and "cleaning time" are used as defined in §0.1.
- "Completed" means the operator pressed **Keep** after a save that passed the file check. For a region with sub-steps, every sub-step must be kept.
- **Cleaning time** is set per region by a supervisor and stored in the catalogue. Operators can extend a countdown during a session (logged) but
  cannot change the default. The deck holds no times: none of the 25 slide XMLs mentions time, pole or hand.
- **Proposed defaults** come from the ALIX commode C raw left-arm recordings (`EXP01/data/teach_files/commode_c_taught_paths/FILE_*.txt`), at
  points × 5 ms. Whole cleaning regions took 41.7-66.0 s (median 48.2 s); mechanisms took 16.5 s and 23.7 s. Proposal, to be confirmed by the requester:

  | Region kind | Proposed default |
  |---|---|
  | Whole surface | 60 s |
  | Half or part | 30 s |
  | Floor part | 30 s |
  | Drain-hole pass | 20 s |
  | Mechanism | 20 s (a sub-step 10 s) |
  | Tool or glove | 30 s (no reference) |
  | Idle hold | 5 s |
- **Record period.** No vendor source states it.
  - Evidence for 5 ms: Newton's single-joint sweep files have names whose durations equal points × 5 ms exactly (`ALIX/docs/sources.md:5293`).
  - exp01 still marks it open (`EXP01/README.md:63-68, 275-282`); a vendor file is catalogued at an assumed 10 ms (`EXP01/data/teach_files/MANIFEST.yaml:25-26`).
  - If it is 10 ms, the default times above double.
  - Lite stores the wall-clock time between the "DRAG TEACH ACTIVE" and "DRAG TEACH STOPPED" markers for every recording. That measures the
    period and settles the open question.
- **Scope.** All 42 regions are in the catalogue. A region that needs a station Lite does not have yet (brush holder, glove station) can be
  marked BLOCKED with a reason.
- **Default operator model.** One person drags one arm at a time, using both hands. Both-arm mode needs two people. In Both mode a MASTER/SUPPORT
  mechanism may be taught as one simultaneous pair instead of alternating sub-steps. The catalogue records which way it was taught, because the
  replay (R8/R13) must match: TOGETHER or alternating.
- Dashboard statuses after Taught come from R2, R3, R4 and R13. R1 must deliver Not taught, Awaiting review and Taught.

### 1.4 Findings (verified)

**ALIX Studio (alix_taskgen)**

1. **The Studio Regions tab is a mesh-painting editor for the excluded generator (X1).** The panel "Region Labels — step 1" says "Mark regions
   once …, then Generate creates a config per region", with Paint and Erase brushes over the fixture meshes (`WE/template.html:156-201`; heading
   and tooltip at `:158`). It writes `regions.yaml` (`SRV:4030-4129`). It cannot show the PPT and should not be ported. Only one layout idea is
   useful: the active region's properties sit next to the region list (`WE/template.html:783-806`).
2. **The Board tab gives the right look for the dashboard, but its code is generator-bound.**
   - The markup says "Deliberately shows ONLY what is on disk" (`WE/template.html:529-553`).
   - `loadBoard` draws the summary chips and the `present/expected` badge on the tab (`WE/editor.js:14708-14757`). `renderBoardRows` draws the
     region × arm table with ✓, · and — pills and the relative age (`WE/editor.js:15208-15289`).
   - The cells show point counts, the generator route and gate verdicts. A click opens the Workbench (`WE/editor.js:15410-15460`), and
     "Generate missing" runs the C-space pipeline (`WE/template.html:545-547`). Those are the excluded features X1 and X2.
   - **Borrow:** the CSS (`WE/editor.css:830-838, 1507-1524, 1974-1999`), the "cell is a control" and "progress on the tab" patterns, and the
     small `age()` helper.
   - **Rewrite:** the roughly 150 lines of JavaScript.
3. **The Board backend is not reusable.** `do_fixture_board` reads `models/regions.yaml` and checks for generated
   `<region>_<arm>_cleaning_points.yaml` files under the repository's task package (`SRV:5396-5486`). Its coverage and validation metrics are
   generator features (`SRV:5562-5773`).
4. **Studio has no guided flow, wizard or timer, by design.** "Grouped by purpose, not numbered. The ordinals told a linear story … over a loop
   the operator runs 38 times" (`WE/template.html:116-118`; `WE/editor.css:830-831`). R1 *is* a linear story, so Lite needs a staged flow that
   Studio deliberately avoided.
5. **Studio cannot drive drag teaching.** "HONEST: the studio has no execution channel … nothing here commands hardware" (`WE/editor.js:13045-13047`;
   "idle — offline preview (arms not connected)" at `WE/template.html:889`). The emulator bridge only reads (`TG/sim_bridge.py:10-14`).
6. **The feedback toolkit is reusable.**
   - `setStatus` and `showErrorToast` (`WE/editor.js:5431-5462`).
   - `showStudioModal`, a styled confirm dialog with chips and sentences (`WE/editor.js:13780-13830`).
   - Gate chips that carry state as glyph plus colour (`WE/editor.js:15904-15925`; `WE/editor.css:1409-1433`).
   - The coverage bar with a target marker, which can serve as the countdown bar (`WE/editor.css:1435-1460`).
   - The rule "a verdict not computed for the current input is stale, never passing" (`TG/schedule_guards.py:13-17`).
7. **(New) Studio already validates a fixture-state timeline.** The "reality-logical" tier of `schedule_guards` checks three things:
   - mechanism rows operate only parts that are in the state they operate from;
   - `implies` physics hold (an open seat rests on the open lid);
   - each cleaning region runs only in its declared state.

   (`TG/schedule_guards.py:8-12, 196-236, 449-470`.) It is small, pure Python and bound to Studio's manifest objects. Reuse it as logic for R1's
   teaching order and lid/seat prompts, and later for R8 and R10.

**Control app (UI v53 + runner v14)**

8. **Per-arm drag teach exists end to end.**
   - START DRAG + RECORD asks for confirmation ("The arm will enter Drag Teach mode and become hand-guidable … keep the hardware E-stop
     accessible", `UI53:3921-3931`) and runs `--drag-session`. The same button stops it (`UI53:3877-3965`).
   - The runner calls `rm_start_drag_teach(1)`, idles until SIGINT, then calls `rm_stop_drag_teach()`. Its `finally` block stops drag mode if the
     function exits abnormally (`RUN14:912-954`).
   - Status lines: `DRAG TEACH ACTIVE: …`, `DRAG TEACH STOPPED: …` and `DRAG SAVE COMPLETE: <path> // <N> points` (`RUN14:929-930, 943, 985-987`).
     The UI matches them by prefix (`UI53:5894-5911`).
   - No test record was found. The V6 package and the UI history (`UC/py programs/…v25_drag_teach_test.py` … `…v37_dual_drag_save_popup.py`)
     indicate it has been used on the arms.
9. **Saving is a separate process with no naming, metadata, retry or check.**
   - `--drag-save FILE` calls `rm_save_trajectory(path)`, which returns `(ret, point_count)`. It is tried once and writes straight to the given
     path (`RUN14:957-989`).
   - The default name `~/Downloads/RealMan_DragTeach/<side>_drag_<YYYYmmdd_HHMMSS>.txt` carries no region, pole, hand or operator
     (`UI53:3840-3850`).
   - The path is set only when empty and never reset (`UI53:3937-3938`).
10. **Both-arm teaching exists.** START BOTH DRAG checks that both arms are idle, asks for confirmation and launches both sessions behind
    `threading.Barrier(2)` (`UI53:7841-7935`). The save popup proposes `left_drag_<stamp>.txt` and `right_drag_<stamp>.txt` (`UI53:8043-8056`) and
    refuses identical names (`UI53:8295-8300`).
11. **(New) The runner checks nothing before drag teach.**
    - `run()` refuses when the arm reports an error or a program is already running (`RUN14:26-30`; `arm_state` at `RUN14:122-141`).
      `drag_teach_session` calls `rm_start_drag_teach(1)` directly (`RUN14:912-929`).
    - ALIX notes that all joints must be enabled before any drag session (`EXP01/notes/2026-08-25_phase_a.md:98`).
    - There is no command-line flag that reports error codes or program state.

    Lite therefore needs a small status command: errors, run mode, program state and current tool frame. The building blocks are `arm_state`,
    `get_arm_run_mode` and `_capture_current_frames` (`RUN14:122-141, 1213-1227, 1430-1448`).
12. **(New) No other client may drive the arm during teaching.** When the ALIX ROS stack runs, it "ACTIVELY HOLDS the arm … streams it at 100 Hz, so a
    manually displaced arm creeps back toward the last commanded pose" (`ALIX/ros/hw/alix_hw/scripts/teach_mode.py:18-22`). The runner says:
    "nothing else may be driving it; stop the ROS driver first" (`RUN14:26`).
13. **(New) A drag session can outlive a crashed parent.**
    - Child processes start with `start_new_session=True` (`UI53:5745-5750`), so they get no signal when the parent dies.
    - The runner's `finally` runs only on a normal exit (`RUN14:945-954`). A SIGKILL, or the Windows CTRL_BREAK (`UI53:3897-3901`; inferred,
      map 09), leaves the arm in drag mode.
    - The controller keeps the latest recording until the next start (`EXP01/notes/2026-08-25_phase_a.md:95`), so a recording can still be saved
      after a crash.
14. **What the runner does not do.** It has no region concept, time limit, countdown, auto-advance or dashboard. Pole/hand and drag teach sit on
    different tabs (`UI53:2156-2166`). Playback replays only the controller's latest recording (`RUN14:1173-1210`), so a new recording replaces an
    unsaved one.
15. **(New) The arm has its own physical DRAG button** (`ALIX/ros/hw/alix_hw/scripts/teach_mode.py:3, 9, 29`). If an operator uses it, Lite does
    not know a teach happened. Whether the button also replaces the controller's latest recording is unknown and should be checked on hardware.

**RealMan SDK and ALIX emulator**

16. **SDK behaviour.**
    - No recording-length limit is documented (`SDKI:3604-3620, 3873-3891`).
    - A drag sensitivity setting (0-100 %, `SDKI:3687-3720`) exists and the control app does not use it.
    - The multi-mode drag teach lists "robot in simulation mode" and "IO E-stop" as failure reasons (`SDKI:3637-3661`). Teaching is therefore
      treated as REAL-mode only.
    - A robot replay first moves to the recording's start at 20 % speed (`SDKI:3722-3725`).
17. **No drag-teach emulation exists.** The ALIX emulator implements no drag-teach or save call (`EXP01/notes/2026-08-25_phase_a.md:97-98`; no
    `rm_start_drag_teach` or `rm_save_trajectory` in `EMU`). R1 needs its own fake recorder.
18. **(Corrected) The limp-arm risk is at the start, not the stop.** ALIX deferred `rm_start_drag_teach` because it "makes a loaded arm genuinely
    limp on release" (`ALIX/ros/hw/alix_hw/scripts/teach_mode.py:11-16`). "On release" means when the arm becomes compliant, so the operator must
    support the arm before START until drag mode is confirmed. At STOP the arm holds where it is.

**Regions deck and existing data**

19. **The deck has 25 slides.** Slides 1-2 show the three lid/seat states and the exterior views. Slides 3-4 are the hand-drawn region maps
    (PAGE 04/05). Slides 5-25 are the 22 dual-arm task steps (1A … 15), with LEFT in blue and RIGHT in orange (map 13 §1-§2; renders
    `SP/pptx_render/hi-05.png` and `hi-19.png` checked). The deck has no cleaning times and no pole or hand values. 21 of 44 arm-step labels carry no
    code, `FL-R09` breaks the pattern and R-T08 is missing (map 13 §5.1), so Lite needs its own IDs.
20. **No per-region raw recording exists for the deck's regions.** V6 holds only merged, time-stretched programs ("stretched only by adding/
    interpolating points", `V6/README.txt:4-8`) covering about half the regions; 16 regions have nothing taught (map 13 §6.3). V6 segments can be
    shown only as legacy references.
21. **Raw left-arm commode C recordings exist** with a per-file teach sheet (`EXP01/data/teach_files/commode_c_taught_paths/description.txt:1-20`).
    They cover whole regions, not the deck's parts, and were taught on 2026-09-22. They are useful as reference data and for default times (§1.3).

**ALIX taught-track lane (provenance)**

22. **ALIX already has the provenance format Lite needs.**
    - `SOURCE.json` sits beside each recording with `side, frame, pole_m, hand_state, hand_counts, hardware, taught_by, date, units, note`
      (`TT/source/README.md:8-27`).
    - Source recordings are kept "unmodified … nothing here is ever edited after it lands" (`TT/source/README.md:3-6`).
    - `tools/taught_tracks.py` looks for `<stem>.SOURCE.json` (`ALIX/tools/taught_tracks.py:310-342`) and writes it when it is missing (`:3624-3634`).
    - Its `screen` step marks files without arm, pole or hand as NEEDS-PROVENANCE and rejects joint margins inside 5° (`TT/README.md:103-107`).
    - The ALIX pipeline's first step writes the sidecar with the original path and sha256 (`ALIX/docs/motion/TAUGHT_PIPELINE_README.md:56`).

**Trajectory HTML and workstation**

23. **The trajectory HTML has a 6-node stepper** (Upload, Setup, Analyze, Review, Generate, Compare) with lock rules (`HTML:330-340, 609-623, 659`).
    It is the in-house pattern for a guided flow. It uses its own palette, so it must be re-skinned with the Studio tokens.
24. **Deck renders already exist.** The earlier analysis left 25 slide renders and 37 per-arm crops (`SP/pptx_render/`), with the scripts
    `SP/scripts/{pptx_detail,arm_crops,overlay_norm}.py`; none is in a repository. LibreOffice and `pdftoppm` are installed on this workstation;
    `python-pptx` is not. The slide XML is readable with the standard library. Deck import is an offline step, so Lite does not need LibreOffice
    at run time.

### 1.5 Borrow verdict

**ALIX Studio: Partial — borrow some, build rest.**
- Borrow the Board tab's look: CSS, chips, tab badge, pills and the cell-as-control pattern.
- Borrow the feedback components, the theme tokens and the fixture-state timeline logic from `schedule_guards`.
- Do not borrow the Regions tab (finding 1), the Board JavaScript or the Board backend (findings 2, 3).
- The guided session, countdown, auto-advance and drag recording are new work: Studio has no wizard by design and no robot channel (findings 4, 5).

**Input claim check: partly accurate.** The input makes no explicit borrow claim for R1; it names a "Regions Tab" and a "Dashboard". ALIX Studio
has a Regions tab, but it is a mesh-painting editor for the excluded generator, so only the name carries over. The Studio Board tab is a good
precedent for how the dashboard looks, but its code reads generated configs, not `.txt` files.

| Other source | Verdict | What to take |
|---|---|---|
| Control-app runner v14 + UI v53 | Borrow with adaptation | `--drag-session`, `--drag-save`, `--drag-playback`, `--get-arm-mode`, the status lines, the confirmation wording, the barrier start for both arms, the distinct-name check, and `arm_state` / `_capture_current_frames` for a status command. |
| ALIX taught lane (`TT/source/README.md`, `taught_tracks.py`) | Borrow as-is (schema and rule) | The `SOURCE.json` field set and the "raw is never edited" rule, extended with a `lite` block. |
| `TG/schedule_guards.py` tier 2 | Borrow with adaptation (logic) | Fixture-state timeline checks for the teaching order and lid/seat prompts. |
| Trajectory HTML | Borrow with adaptation (pattern) | Stepper header with lock rules, re-skinned; rename its "regions" to "segments". |
| Deck renders, crops, map-13 ID scheme and boxes | Borrow with adaptation | Turn the scratchpad scripts into an offline `deck_import` tool owned by Lite. |
| ALIX commode C raw recordings | Reference | Default cleaning times (5 ms assumption); optional reference imports. |
| V6 package | Reference | Legacy coverage flags on the dashboard only. |

### 1.6 What to borrow and what to build

**Borrow**

| Item | From | Adaptation |
|---|---|---|
| Dashboard look: table CSS, summary chips, tab badge, pills, cell-as-control, `age()` | `WE/editor.css:830-838, 1507-1524, 1974-1999`; `WE/editor.js:14708-14757, 15208-15289, 15410-15460` | Copy the CSS and patterns; write new JavaScript. Rows are steps ST01-ST22 (toggle: region families); columns are LEFT and RIGHT; each cell holds the region ID and a status pill with glyph and word. A click opens the card or starts teaching. No pair, coverage, route or generate columns. |
| Feedback toolkit | `WE/editor.js:5431-5462, 13780-13830, 15904-15925`; `WE/editor.css:1409-1460` | Plain-language status and toast; the confirm dialog before drag mode; chips as pipeline chips (Taught, Optimised, Sim OK, Real tested, Uploaded); the coverage bar with target marker as the countdown bar. |
| "Only what is on disk" and "stale is never passing" rules | `SRV:5396-5407`; `TG/schedule_guards.py:13-17` | Status is derived from files at each refresh. A later status counts only if its record names the current file's sha256. |
| Fixture-state timeline logic | `TG/schedule_guards.py:196-236, 449-600` | Validate the teaching order and generate the lid/seat change prompts from the catalogue's states and the mechanism steps. |
| Drag-teach commands and both-arm start | `RUN14:912-989, 1173-1227`; `UI53:3877-4059, 7841-8446` | Wrap behind `TeachDevice`. Lite chooses names and passes `--ip`/`--sdk`. Parse the status lines by prefix. |
| Status helpers | `RUN14:122-141, 1213-1227, 1430-1448` | A small Lite-owned status command: error codes, run mode, program running, tool and work frame. |
| Provenance sidecar | `TT/source/README.md:8-27`; `ALIX/tools/taught_tracks.py:310-342, 3624-3634` | Same keys plus a `lite` block. Practice files get `hardware: analysis_only` and live in the practice folder. |
| Guided-flow stepper | `HTML:330-340, 609-623, 659` | Session header "Region 7 of 42 · LEFT · Lid open" with back/next and locks, in Studio tokens. |
| Deck renders, crops, overlay boxes, ID scheme | `SP/pptx_render/`, `SP/scripts/{pptx_detail,arm_crops,overlay_norm}.py`; map 13 §5.2, Appendix A | An offline `deck_import` tool that regenerates the assets and a draft catalogue, and prints a diff on re-import. |
| Default cleaning times | `EXP01/data/teach_files/commode_c_taught_paths/FILE_*.txt` | Per-type defaults (§1.3), supervisor-editable. |

**Build**

| Item | Approach |
|---|---|
| Region catalogue + validator | One YAML file per deck version. Fields: `id`, deck aliases, `title`, `arm`, `type`, `family`, `part`, `step`, `paired_with`, `role`, `fixture_state`, `tool`, `teach_time_s`, `pole_mm`, `hand`, `prep_order`, `substeps` with `hand_events`, `image`, `blocked`, `legacy`. Produced by `deck_import` and reviewed by a person. The validator refuses duplicate IDs, missing fields and unknown states. |
| Regions tab | PAGE 04/05 overview with clickable label hotspots (map 13 Appendix A.2) and a Floor Spillage hotspot; step list grouped by lid/seat phase; region card; "View original slide". |
| Status engine + dashboard API | Scans `units/<ID>/raw`, `opt` and `events.jsonl`. Derives Not taught, Awaiting review, Taught, Optimised, Sim OK, Real tested and Uploaded, with checksum matching and stale flags. Endpoints: `GET /api/lite/regions`, `/api/lite/regions/<id>`, `/api/lite/dashboard`, `POST /api/lite/regions/<id>/settings` (supervisor). |
| Teach session (backend + UI) | A persisted state machine (§1.8.3). Teach view with a big card, START/STOP on screen and on one dedicated key or foot switch, the countdown, sounds, overtime display, hard-limit stop, and a review screen with a summary and a small joint plot (vendored Plotly). |
| Ordering and auto-advance | Rules in §1.8.4, including sub-steps and hand events. |
| Save, file check and provenance | §1.8.5: temporary name, check, rename without overwrite, read-only, sha256, sidecar. |
| Hardware adapter, status command, fake recorder | `TeachDevice` with a runner-subprocess implementation, child-process supervision and a stop-drag fallback. `FakeTeachDevice` serves practice and tests. |
| Interlocks and control lock | While any arm is in drag mode, no other command is sent. One teach per arm. Next START stays disabled until the recording is saved or discarded. REAL mode is required. A single browser control lock. Operator confirmation that no other client drives the arms. |
| Crash recovery | On start, check `sessions/*.json` and the recorded child process IDs. Stop a live drag session, or call `rm_stop_drag_teach` on a fresh connection. Offer Save or Discard for the controller's retained recording. |

### 1.7 Recommendation

Build R1 as a new guided **Teach** flow in the Lite web shell, around a Lite-owned, versioned region catalogue. Generate the catalogue offline
from the deck and have the requester sign it off.
- **From Studio:** reuse the Board tab's look and the feedback components, but write the dashboard code and backend new. Do not port the Studio
  Regions tab.
- **Robot control:** drive drag teaching through runner v14's commands behind a `TeachDevice` adapter, as subprocesses for the MVP. Add what the
  runner lacks:
  - preconditions: REAL mode, no error codes, all joints enabled, no program running, no other client such as the ALIX ROS stack;
  - Lite-chosen file names that are never overwritten, written through a temporary name;
  - a save retry and a point-count check;
  - an ALIX-compatible provenance sidecar that records the measured record period;
  - supervision of drag-session processes and crash recovery;
  - the countdown, and auto-advance that never moves the robot.
- **Catalogue model:** teach mechanism, tool and glove tasks as sub-steps.
- **Cleaning times:** seed defaults from the ALIX commode C raw recordings, noting the 5 ms assumption, and let a supervisor own them.

### 1.8 Approach

#### 1.8.1 Steps

1. **Agree terms, IDs and the sub-step model** with the requester. Also agree fixes for the deck's inconsistencies: FL-R09, the uncoded labels, R-T04
   repeated on 10B, the drain-hole parts and the missing R-T08.
2. **Deck import (offline).** `deck_import.py` unzips the PPTX and reads the slide XML with the standard library: label text, fill colour (which
   gives the arm), shape positions (which give the highlight boxes) and the base media. It proposes IDs and writes a draft
   `regions_<deck version>.yaml`. A person reviews and commits it. Re-running on a new deck version prints a diff instead of overwriting.
3. **Assets (offline).** Render with LibreOffice headless → PDF → `pdftoppm`, cut per-arm crops (port `SP/scripts/arm_crops.py`) and copy the
   base media. Ship the results with Lite. v1 cards use the crops. v2 draws highlights from the boxes, so shared pictures (tasks 5A, 8A, 14, 15;
   photos of tasks 3, 4, 7) highlight only the active arm.
4. **Data layout and status engine** (§1.8.2, §1.8.6), with a separate practice root.
5. **Hardware adapter.** `TeachDevice` methods: `status(side)`, `start_drag(side|both)`, `stop_drag`, `save(side, path)`, `apply_pole`,
   `apply_hand`, `read_pole_hand`.
   - **`RunnerTeachDevice`:** calls runner v14 exactly as v53 does — `sys.executable -u runner --sdk <dir> --ip <ip> … --yes`, its own session,
     SIGINT to stop. A small Lite-owned status script and `usage.py`'s reads complete it.
   - **Supervision:** child process IDs are stored in the session file.
   - **Stop-drag fallback:** a fresh process calling `rm_stop_drag_teach`. Verify on hardware that a second connection can do this.
   - **`FakeTeachDevice`:** replays an existing raw file at real-time pace, for practice and tests.
6. **Teach session state machine** in the backend (§1.8.3), persisted in `sessions/<id>.json`.
7. **Ordering and auto-advance** (§1.8.4).
8. **Save, check and provenance** (§1.8.5).
9. **Front end** in the Lite shell, built from Studio tokens and components:
   - Regions tab;
   - Teach view;
   - Dashboard (Board look, chips, tab badge "n/42", "missing only", "Teach next missing");
   - Supervisor settings (teach time, pole, hand, BLOCKED);
   - control lock and the dedicated START/STOP key.
10. **Guard rails:**
    - while any arm is in drag mode, no other command;
    - one teach per arm;
    - next START disabled until the recording is saved or discarded;
    - REAL-mode check;
    - Both mode only after a two-person confirmation;
    - the operator confirms that the ALIX ROS stack and the v53 app are closed.
11. **Tests:**
    - unit tests: catalogue validator, status engine, ordering with sub-steps, state machine;
    - end to end with `FakeTeachDevice` in practice mode;
    - crash and recovery tests;
    - hardware acceptance on both arms: single and Both, save retry, STOP during recording, killing Lite during a recording, the physical
      DRAG button, and one long recording (about 5 min) to probe the controller's limit.

#### 1.8.2 Data layout and catalogue record

```
<lite_data>/<fixture>/                      # real data
  registry/regions_Toilet_regions_V1.yaml   # the catalogue (source of truth), one per deck version
  assets/Toilet_regions_V1/{slides,crops,media}/
  units/<REGION_ID>/raw/<REGION_ID>[-S<n>]__RAW__<yyyymmdd-hhmmss>.txt          # read-only, as saved by the controller
  units/<REGION_ID>/raw/<REGION_ID>[-S<n>]__RAW__<yyyymmdd-hhmmss>.SOURCE.json  # provenance, written once
  units/<REGION_ID>/opt/...                 # R2 outputs
  units/<REGION_ID>/events.jsonl            # append-only: kept / rejected / skipped / sim / real / upload decisions
  sessions/<session_id>.json                # resume state, child process IDs
<lite_data>/practice/<fixture>/...          # same layout, fake recorder only, never counted as taught
```

Catalogue record (map 13 §7.3, extended):

```yaml
- id: L-R05-P1
  deck: {code: null, label: "SEAT TOP PART 1", task: "5A", slide: 11, code_source: inferred-gap-fill}
  title: "Seat top, left half"
  arm: left
  type: surface                           # surface | mechanism | tool | glove | idle
  family: "Seat Top"
  part: {n: 1, of: 2, where: "left half"}
  step: ST07
  paired_with: R-R04
  role: null                              # MASTER | SUPPORT for mechanisms
  fixture_state: LID_OPEN                 # CLOSED | LID_OPEN | SEAT_OPEN | ANY ; mechanisms: {before, after}
  tool: glove                             # glove | brush | none
  teach_time_s: 30                        # supervisor-owned; hard limit = max(3 x teach_time_s, 120 s)
  pole_mm: 150                            # R7
  hand: {preset: GRIP, counts: [0, 0, 0, 0, 150, 1000]}   # R7; null = hold that channel
  prep_order: pole_then_hand              # R7
  substeps: []                            # e.g. R-T04: [{id: S1, arm: right}, {hand_event: grip_brush}, {id: S2, arm: right}]
  image: {crop: crops/crop-s11-T5A-LR.png, media: image2.png, box_uv: [0.334, 0.420, 0.516, 0.954]}
  blocked: null
  legacy: {package: V6, file: LEFT/L_P06P07.txt, samples: [3865, 6744], confidence: medium-high}
```

Sidecar (`<file stem>.SOURCE.json`). The ALIX keys come first, so `taught_tracks.py` can read the file; the Lite keys sit in a block:

```json
{"side": "left", "frame": null, "pole_m": 0.1496, "hand_state": "GRIP",
 "hand_counts": [3, 2, 4, 1, 152, 998], "hardware": "ok", "taught_by": "<operator>",
 "date": "2026-10-13", "units": "millideg", "note": "Lite teach session <id>",
 "lite": {"region": "L-R05-P1", "substep": null, "deck_version": "Toilet_regions_V1", "deck_task": "5A",
          "fixture_state": "LID_OPEN", "bin_state": "EXTENDED", "placement_confirmed": true,
          "controller_ip": "192.168.1.10", "tool_frame": "<as read>", "run_mode": "REAL",
          "pole_target_mm": 150, "hand_target": [0, 0, 0, 0, 150, 1000], "prepare_override": null,
          "teach_time_target_s": 30, "recording_elapsed_s": 34.2, "point_count": 6123,
          "controller_point_count": 6123, "record_period_ms_measured": 5.59,
          "sha256": "...", "attempt": 2, "mode": "single", "lite_version": "...", "runner": "v14"}}
```

`frame` is the glove pad when known and `null` otherwise. Lite works in joint space, but R9/R14 offsets need the pad.

#### 1.8.3 Teach session states

`SELECT_ARMS → CARD → FIXTURE_CHECK → PREPARE (R7) → READY → RECORDING → STOPPING → SAVING → CHECKING → REVIEW → (KEEP → NEXT | REDO → PREPARE | SKIP → NEXT) → DONE`, plus `RECOVER` on start-up.

- **SELECT_ARMS.** Choose Left, Right or Both, then check:
  - the controller answers;
  - the arm is in REAL mode;
  - there are no error codes and all joints are enabled;
  - no program is running.

  The operator confirms that the ALIX ROS stack and the v53 app are closed. Lite asks for the bin state once (R7) and confirms the robot is
  parked at the marked position for this fixture.
- **CARD and FIXTURE_CHECK.** These never send a robot command.
- **PREPARE.** Runs only on an explicit press (R7).
- **READY.** Checklist: "I am holding the arm" (the arm may sag when it becomes compliant), then START.
- **RECORDING.** The countdown starts on `DRAG TEACH ACTIVE`, not on the button press. Sounds at 10 s left and at 0 s. Overtime is shown in amber.
  At the hard limit there is a 10 s warning, then an automatic stop.
- **STOPPING.** STOP comes from the on-screen button or the dedicated key/foot switch. Lite waits for `DRAG TEACH STOPPED`.
- **SAVING.**
  1. Wait 1 s.
  2. Save to `<final>.partial`.
  3. On failure, retry up to 3 times, 1 s apart.
  4. Compare the controller's point count with the lines in the file.
  5. On final failure, stay in SAVING with "Try again" and "Discard". Discard needs confirmation, because the controller holds only the latest recording.
- **RECOVER.** Runs on start-up if a session was in RECORDING, STOPPING or SAVING, or a recorded drag child is alive. Lite stops it (SIGINT, or the
  fallback stop) and offers "Save the recording the robot still holds" or "Discard".

#### 1.8.4 Ordering and auto-advance rules

1. **Default order is "by arm within each lid/seat phase":** all untaught LEFT regions of the phase, then all RIGHT regions, then the state change.
   This minimises lid/seat changes for one operator. The option "deck order" follows ST01..ST22.
2. **Skip regions already kept** (unless "re-teach" is chosen) **and BLOCKED regions** (reason shown).
3. **Insert a prompt when the next region needs another lid/seat state.** The prompt shows the deck state photo and is validated with the
   fixture-state timeline logic (finding 7). START stays disabled until the operator confirms the new state.
4. **Mechanism regions (MASTER/SUPPORT) are taught as alternating sub-steps:** S1 master, S1 support, S2 master, S2 support (as V6 ran them).
   - No Prepare runs for an arm that is holding the lid or seat between sub-steps; a pole or hand change would move or drop it.
   - A Redo of a sub-step invalidates the later sub-steps of the same mechanism, because their start state changed.
   - In Both mode the pair may instead be taught together.
5. **Tool and glove tasks are split at hand events** (R7 (h)): S1 drag to the grasp point, STOP, hand action with drag mode off, then S2.
6. **Idle regions** (R-T10 "Hold brush outside") record a short hold pose only. Their replay duration is the partner arm's.
7. **After Keep,** Lite shows "Next: <title>" for 3 s with a Pause button, then opens the next card. It never prepares or starts by itself.

#### 1.8.5 Save and file check (the raw file is never modified)

- **Naming.** The name is `units/<ID>/raw/<ID>[-S<n>]__RAW__<yyyymmdd-hhmmss>.txt`. The controller writes to `<name>.partial`. After the
  check, Lite renames it to the final name only if that name does not exist, then sets the file read-only and computes its sha256.
- **Blocking checks.** Lite applies the runner's reading rules (`RUN14:76-88`): every line is `{"point":[7 integers]}` and there are at least 2
  points. The controller's point count must equal the number of lines. If any check fails, Lite shows "Recording failed — please redo" and Keep
  is blocked.
- **Warnings** (plain words; they do not block Keep):
  - the measured record period differs from 5 ms by more than 30 %;
  - a joint is within 5° of a URDF limit (ALIX rejects these, `TT/README.md:103-107`; Lite recommends Redo);
  - a step above 10° between lines (a corruption sign; the controller rejects such lines);
  - the recording is shorter than 30 % of the target time.
- **Summary.** Duration, points, a small joint plot and the warnings, then Keep or Redo.

#### 1.8.6 Dashboard status rules (derived from files)

| Status | Rule |
|---|---|
| Not taught | No kept raw recording (or its file is missing). |
| Awaiting review | A raw file exists with no Keep/Redo decision in `events.jsonl` (for example after a crash). |
| Taught | A kept raw recording exists for every sub-step and its sha256 matches the sidecar. |
| Optimised | An R2 output whose report names the kept raw's sha256. |
| Simulation OK | An R3 verdict naming the current optimised file's sha256; otherwise "stale". |
| Real tested | An R4 operator pass naming the current file; otherwise "stale". |
| Uploaded | An R13 upload record (controller, slot, verified size) naming the current file; otherwise "stale". |
| Flags | Skipped (reason), Blocked (reason), Legacy V6 (reference only), Needs re-teach (split or arm changed; earlier sub-step redone), Taught at other pole/hand (R7 (g)). |

### 1.9 Acceptance criteria

1. Loading the Toilet_regions_V1 catalogue gives 42 regions (22 LEFT, 20 RIGHT) with unique IDs and all required fields. A catalogue with a
   duplicate ID, a missing field or an unknown state is refused with a message naming the region. Regions with sub-steps list them in order.
2. Every region opens from the Regions tab and shows its picture with the area highlighted, a plain-language title, the arm badge (blue LEFT,
   orange RIGHT) and, on request, the original slide. Clicking a PAGE 04/05 label (including Floor Spillage) filters the list to that family.
3. With an empty data folder the dashboard shows "0/42 taught" and every region "Not taught".
4. In practice mode (fake recorder, practice folder), after a recording for L-R01 is kept, the practice dashboard shows "Taught" with date,
   duration and attempt count, and the counters show 1/42 (LEFT 1/22) without restarting Lite. The real dashboard is unchanged.
5. If the kept raw file is removed outside Lite, the cell shows "Not taught (file missing)" after the next refresh.
6. With the fake recorder and 3 untaught LEFT regions, the operator finishes all three using only Prepare, START, STOP and Keep per region.
   The next card appears automatically after each Keep, and a summary after the third.
7. The command log shows that opening a card and auto-advancing send no robot command. Pole and hand moves happen only after Prepare is
   pressed; drag mode only after START.
8. The countdown starts when drag mode is reported active, sounds at 10 s left and at 0 s, and continues as overtime after 0 s. At the hard
   limit, after a 10 s warning, recording stops automatically with an explanation.
9. Two recordings of the same region produce two files. No file is ever replaced: the save goes to a temporary name and is renamed only after
   the check. Raw files are read-only, and each sidecar's sha256 equals its file's sha256.
10. Each sidecar holds the ALIX keys (side, frame, pole_m, hand_state, hand_counts, hardware, taught_by, date, units) and the Lite block:
    region, sub-step, deck version and task, fixture and bin state, controller IP, tool frame, targets, measured values, target time, elapsed
    time, both point counts, measured record period, sha256, attempt and versions.
11. An unparseable or empty save, or one whose point count differs from the controller's, is reported as "Recording failed — please redo", and
    the session does not advance. Period, margin, step and short-recording warnings are shown but do not block Keep.
12. Redo keeps the earlier file and logs it as rejected. Skip requires a reason and shows "Skipped" on the dashboard. Redoing S1 of a
    mechanism flags its later sub-steps "Needs re-teach".
13. When the next region needs another lid/seat state, START stays disabled until the operator confirms the new state on the prompt.
14. Closing Lite during a session and reopening it offers "Resume" at the first region without a kept recording.
15. If Lite is killed while an arm is recording (fake recorder; once on hardware), on restart Lite reports that the arm was left in drag mode,
    takes it out, and offers Save or Discard for the retained recording. Saving produces a normal raw file and sidecar.
16. In Both mode, one START puts both arms in drag mode and STOP saves two differently named files with two sidecars. In single-arm mode the
    other arm never enters drag mode.
17. While any arm is in drag mode, Lite refuses pole, hand, run and upload commands. STOP stays available.
18. A failed controller save is retried 3 times, 1 s apart. START for the next region stays disabled until the recording is saved or discarded
    with confirmation.
19. Teaching cannot start while the arm reports SIMULATION mode, reports error codes or has a program running. Lite explains why in one sentence.
20. Practice-mode files live only under the practice folder and carry `hardware: analysis_only`. The real dashboard never counts them.
21. The START/STOP key starts or stops recording whichever on-screen control has focus, and triggers no other control. A second browser
    session sees the session read-only.

### 1.10 UX notes for non-technical operators

- **One region per screen.** A large picture with the area highlighted and a plain title ("LEFT arm — wipe the seat top, left half (part 1 of
  2)"), with the codes in small type underneath. Blue and orange are used only for the arm badge, never for statuses.
- **Mark the arms.** Put a blue mark on the robot's LEFT arm and an orange mark on its RIGHT arm, so the screen colours match the hardware.
  Explain "robot's left" with a top-view picture.
- **Card instruction template:** "Hold the arm. Start with the cloth 3-5 cm above the area. Lower, wipe slowly, pausing is fine. Lift back to
  3-5 cm. Then press STOP." Starting and ending off the surface also makes merging (R8) and pole moves (R7) safer.
- **The operator's hands are on the arm.** START/STOP sits on one dedicated key (a USB foot switch mapped to a key no other control uses) and is
  handled globally; in a browser, Space activates whatever button has focus. There are sounds at 10 s left and at time up, and the timer is
  readable from 2 m. A tablet mounted near the robot can show the Teach view.
- **Use Lite's START/STOP only, not the arm's own DRAG button.** Lite does not see the button.
- **Show progress plainly:** "Region 7 of 42 · LEFT arm · Lid open".
- **Statuses carry a glyph and a word** (✓ Taught, … Awaiting review, · Not taught, — Not for this arm), never colour alone.
- **Fixture changes** are a photo plus one question ("Is the lid UP and the seat DOWN?" Yes / No).
- **Keep or Redo after every recording.** Redo is never destructive. "Teach next missing" is one click from the dashboard.
- **Errors are sentences with the next action** ("The left arm did not answer. Check its power and cable, then press Try again.").
- **No jargon on operator screens:** no slot numbers, millidegrees, SDK or runner terms. Supervisor settings sit behind a separate
  "Supervisor" switch.
- **Confirm only motion-relevant actions** (entering drag mode, Prepare) with a short checklist. Do not confirm edits.

### 1.11 Safety notes

- **Entering drag mode.** The arm becomes compliant and may sag. The operator holds and supports the arm *before* START until Lite shows "drag
  active" (`ALIX/ros/hw/alix_hw/scripts/teach_mode.py:11-16`). Check the controller's tool and payload setting before the first session and
  whenever the hand holds the brush. At STOP the arm holds where it is.
- **One client per arm.** While a Lite session runs, no other client may drive the arms: the ALIX ROS stack holds the arm at 100 Hz and fights
  dragging (`teach_mode.py:18-22`; `RUN14:26`), and the v53 app and a second Lite must not run either. Only one browser session holds control.
- **While any arm is in drag mode,** block every other robot command from Lite (pole, hand, programs, uploads, global moves).
- **Always leave drag mode** on STOP, window close or crash. The runner stops drag mode in `finally` (`RUN14:945-954`), but only on a normal
  exit. Lite stores child process IDs, recovers on restart, and has a stop-drag fallback. Run the hardware layer on Linux: on Windows the stop
  signal does not reach the runner's stop path (inferred).
- **Auto-advance and opening a card never move the robot.**
- **Never start a new recording** before the previous one is saved or deliberately discarded. The controller keeps only the latest.
- **Teaching is a REAL-mode activity.** Read the mode and error codes first.
- **Both-arm teaching** only with two trained people, one per arm.
- **Lid and seat mechanism regions** involve contact forces on the fixture. Teach them slowly, confirm the fixture state before and after, and
  never Prepare an arm that is holding the lid or seat.
- **The hardware E-stop is the safety stop.** Lite's STOP is a convenience.

### 1.12 Dependencies, risks and open questions

**Dependencies**
- The Lite shell and theme (Studio tokens and feedback components) and the Lite server scaffold, with a control lock.
- The control-app integration layer: a runner v14 wrapper with `--ip` and `--sdk`, a small status command and child-process supervision.
  Current pair: UI v53 + runner v14. R4's connection settings (IP entry).
- R7: the Prepare step inside the teach card, the pole and hand values in the catalogue and sidecar, and hand events for tool tasks.
- The region catalogue is shared with R2 (per-region editing), R8 (merge order, sub-steps and hand events), R10 (dual timeline) and R13
  (program names and workflow steps).
- Later dashboard statuses depend on R2, R3, R4 and R13 writing records that name file checksums.
- The requester's sign-off on IDs, the sub-step model, cleaning times and teaching order.

**Risks**
- The deck is a V1 draft with inconsistent codes and ambiguous parts. IDs may change, so the catalogue must be versioned and re-teach flags handled.
- The record period is undocumented. The 5 ms assumption is supported by file-name durations but not settled. Mitigation: measure it per recording.
- One person cannot hold the arm and use a mouse. Without a foot switch or a helper, START/STOP timing adds idle data, which R2 trims later.
- The controller keeps only the latest recording. A missed save loses the teach; recovery covers crashes but not a new START.
- It is not yet verified that a second connection can stop drag mode. If it cannot, the fallback must reuse the session's own connection
  (library implementation).
- The maximum recording length the controller can hold is undocumented.
- Operators may press the arm's own DRAG button out of habit.
- Joint-space recordings are valid only for the same robot-to-toilet placement. Teaching with a different placement silently invalidates them.
- No drag-teach emulation exists anywhere. The teach flow can only be fully proven on hardware.
- Plan-blocking control-app items are listed once in §0.2.

**Open questions**
1. Is "specified cleaning time" the teach countdown, the target run time after optimisation, or both? Who sets it, and are the proposed
   defaults (§1.3) acceptable?
2. Should time-up stop recording (hard) or only warn (soft, recommended) with a hard maximum?
3. Confirm the region IDs (map-13 scheme) and the deck fixes. Is FL-R09 = L-R09? Is R-T04 on task 10B really "pick brush" again, or "hold
   brush"? Are the two drain-hole parts two passes? What was R-T08?
4. Teaching order: by arm within each lid/seat phase (fewest changes) or strict deck order?
5. Are the mechanism regions (open lid + seat, close seat, open seat, close all) taught for the robot, or done by hand as in V6? Are the brush
   and glove regions in scope for v1, and where are the brush holder and glove station?
6. One operator or two? Is a USB foot switch acceptable for START/STOP?
7. Should V6 segments or the ALIX commode C raw recordings appear on the dashboard as references?
8. Is the Lite toilet the same fixture as ALIX "commode C"? How is the robot placement relative to the toilet fixed and recorded?
9. Should every recording start from a common drag start pose (ALIX practice, `ALIX/docs/motion/TAUGHT_PIPELINE_README.md:95-96`)?
10. Is the ALIX ROS stack ever running on this robot while Lite is used? If so, who stops it?
11. Should the review step offer a robot replay? It moves the arm: first to the start at 20 %, then at taught speed.
12. Where should the data folder live, and should Lite files also be copied into the ALIX taught lane?

### 1.13 Priority and effort

**Must · XL (about 27 person-days, one developer, with tests).**

| Item | Person-days |
|---|---|
| Catalogue, deck import and sub-step model | 3.5-4 |
| Assets | 1 |
| Regions tab | 3 |
| Dashboard and status engine | 3 |
| Teach session backend (incl. resume and ordering) | 4.5 |
| Teach UI | 3 |
| Hardware adapter, status command and crash recovery | 3 |
| Save, check and provenance | 1.5 |
| Fake recorder and tests | 2.5 |
| Hardware acceptance | 2 |

This excludes the shared Lite shell and server scaffold and R4's connection settings. An MVP is about 15 days (L, top of the band): crops
instead of drawn highlights, deck order only, a table-only dashboard, tool and glove regions BLOCKED, resume limited to crash recovery.

---

## 2. R7 · Pole height and hand position

### 2.1 Original text

> Pole Height and Hand Position: Each region should have independent settings to adjust the pole height and robotic hand position before the
> person start drag and teach cleaning. - borrow from alix studio

### 2.2 Revised requirement

Each region has its own pole height and hand setting. Lite applies them to that arm before teaching and records them with the recording.

- **(a) Settings per region.** A pole height (0-315 mm) and a hand setting: a named preset with six channel values 0-1000, where a channel may
  be "hold". Defaults are seeded from the V6 workflows, with the ALIX commode C teach sheet shown as an alternative. A supervisor can edit them.
  Thumb-held presets are the default unless a thumb pose is needed.
- **(b) Apply before teaching.** The card's **Prepare robot** button moves the pole and the hand to the region's values, in the region's order:
  pole then hand, or open fingers, then pole, then hand. Lite shows target and measured values. START stays disabled until the pole is within
  ±5 mm and every commanded hand channel is within ±15 counts or reported settled. Only a supervisor can override this, and the reason is logged.
- **(c) Interlocks.** Lite never moves the pole or hand of an arm that is:
  - in drag mode;
  - moving faster than 1.6°/s;
  - holding the fixture or a tool part-way through a task (except at a declared hand event);
  - being driven by another client.

  Before moving, the operator confirms that the cloth or pad touches nothing, that hands are clear, and that the arm and hand are clear of
  the toilet and the other arm.
- **(d) Bin rule.** The bin state is declared per session (STOWED, EXTENDED, LOWERED or NOT FITTED), because Lite cannot read it. With STOWED,
  Lite refuses any pole move whose travel from the current height to the target overlaps 153-282 mm, and any move when the current height is
  already inside that band. Lite explains why.
- **(e) Provenance.** The measured pole height and hand values used for each recording go into its provenance file. A joint-space recording
  replays correctly only at the same pole height and hand pose.
- **(f) Fine-tune.** The operator may adjust values on the card, for example ±10 mm or another preset. The values used are always recorded. They
  become the region's default only with supervisor approval.
- **(g) Recorded values win downstream.** Once a recording is kept, its own pole and hand (from the sidecar) are what R3, R4, R8 and R13 must
  use, not the region's current default. Changing a default later flags existing recordings "taught at other pole/hand".
- **(h) Hand events inside a task.** For tool and glove tasks the hand must act part-way, for example to grip the brush. Teach these as sub-steps
  (R1). The hand action is its own step: drag mode off, arm still, the operator confirms the hand is at the grasp point, then the hand moves and
  is read back. The event is stored in the catalogue sequence and the next sub-step's sidecar, and replayed as program → hand step → program.

### 2.3 Clarifications and assumptions

- **Pole height** is the lift position in mm as read by `rm_get_lift_state`; the runner and UI use 0-315 mm (`RUN14:865-909`; `UI53:4194`).
  It equals the URDF `<L|R>_sliding_plate_joint` in metres × 1000. Both poles have been 1:1 true-mm since 2026-08-07 (`TG/sim_bridge.py:30-34, 166-177`).
- **Hand position** is the Inspire hand's six channels in the order little, ring, middle, index, thumb bend, thumb rotate; 0 = closed and
  1000 = open (`UC/hand.py:3-17`). "Hold" leaves a channel where it is (`UC/hand.py:38-40, 88-96`).
- **"Before the person starts"** means the Prepare step of each region card, with the arm in normal (not drag) mode and at rest.
- **"Independent"** means per region and per arm, not one global setting.
- **Studio's generated pole stage and its viewer pole selector are not robot commands**, so they do not satisfy this requirement (§2.4).
- **Default order is "pole, then hand"**, as in V6 and ALIX's run order. Where the hand holds nothing, the ALIX/`usage.py` variant "open fingers
  (thumb held) → pole → hand to preset" may be used. Where the hand holds a tool (the brush), the hand must not open first. Hence a per-region
  `prep_order`.
- **Preset names** (plain language):

  | Preset | Counts |
  |---|---|
  | Fingers open, thumb held | [1000 ×4, hold, hold] |
  | Fingers half open, thumb held | [500 ×4, hold, hold] |
  | Grip (fist) (V6) | [0, 0, 0, 0, 150, 1000] |
  | Fully open | [1000 ×6] |
  | ALIX `open_tenth` (optional) | [129, 129, 129, 129, 133, 944] |
- **Bin state** uses the ALIX names STOWED, EXTENDED and LOWERED (`ORCH/core/task_config.py:42-55`), plus NOT FITTED. It is a session setting
  shared with R10's bin toggle. "Deployed" in ALIX notes means EXTENDED then LOWERED.

### 2.4 Findings (verified)

**ALIX Studio (alix_taskgen) — what it has and does not have**

1. **The region model has no pole or hand field.** The region editor's whitelist is `arm_coverage, description, surface_type, spacing_m,
   articulation_state, region_id, requires_lid, requires_seat, lid_state, seat_state, part_states, arm_overlap, ugv_standoff_m`
   (`SRV:4074-4077`). The only per-region hint is the pad `ik_frame` (`TG/region_mesh_loader.py:306-314`).
2. **(Corrected) The concept exists per task inside the excluded generator.** The Workbench has a "Pole (written to the task)" field:
   - it offers the named rungs or a custom height of 0-0.315 m, and warns about the stowed-bin band (`WE/template.html:235-238`;
     `WE/editor.js:17161-17200`);
   - it writes a pole stage into the generated task config (`TG/stage_sequence.py:181-207`), which the ROS orchestrator executes at run time.

   The viewer's pole selector is IK-only: "Pin the pole … IK then solves only the arm joints. Bin is always LOWERED" (`WE/template.html:747-754`;
   presets from the SRDF, `SRV:1763-1785`).
3. **The hand state follows the chosen pad and is never chosen directly.** "It is derived, never chosen … offering it as an editable control
   would be offering a value nothing downstream reads" (`WE/editor.js:9959-9963`; `WE/template.html:230`; `TG/config/glove_frames.yaml:97-109`).
   *(New)* ALIX cleans narrow areas, such as the inner rim and hinge, with finger pads at half or quarter grasp
   (`TG/region_mesh_loader.py:306-314`; `glove_frames.yaml:101-109`). Per-region hand settings are therefore real practice.
4. **Studio never commands hardware** (`WE/editor.js:13045-13047`; `TG/sim_bridge.py:10-14`). It cannot apply anything before a teach and has no
   drag-teach concept.
5. **Reusable from Studio.** The stowed-bin foul band and its wording (`WE/editor.js:17144-17149, 17170-17187`) are a copy of the orchestrator's
   measured interlock `pole_bin_conflict`: measured 0.158-0.277 m plus a 5 mm margin, worst −61.8 mm, BinShort at mount 0.294
   (`ORCH/core/task_config.py:58-124`). Also reusable: the Workbench pole UI pattern and the 3-D robot model with sliding-plate and finger
   joints, for an optional preview.

**Control app (UI v53 + runner v14)**

6. **The apply machinery exists.**
   - The HAND / POLE tab has six sliders, "APPLY HAND // ALL 6 CHANNELS", and the presets "GRIP PRESET // 0 0 0 0 150 1000" and "ALL OPEN // 1000 ×6"
     (`UI53:2978-2999`), which apply immediately (`UI53:4127-4174`). POLE runs 0-315 mm with APPLY POLE (`UI53:4177-4209`, clamped at `:4194`).
   - The runner commands are `--manual-hand-raw` (six values), `--manual-hand PCT` (four fingers, thumbs held) and `--manual-pole MM`
     (`RUN14:756-909`). They print the target and the measured result ("MANUAL POLE COMPLETE: target X mm, final=Y mm"; "MANUAL HAND RAW
     COMPLETE: target=[…], final=[…]"), which Lite can use as read-back.
   - No test record was found.
7. **Missing for R7:**
   - no per-region preset object;
   - pole/hand and drag teach sit on different tabs (`UI53:2156-2166`);
   - APPLY has no confirmation by design ("APPLY itself is the explicit motion command", `UI53:4167, 4204`);
   - neither the manual pole command (`RUN14:865-904`) nor a workflow pole step (`RUN14:2174-2178`) checks that the arm is still.
8. **(Corrected) Two per-program pole and hand records exist.**
   - Multi Run (`RUN14:274-363`) is broken in v53 by the TypeError.
   - The Page-02 workflow format `realman_workflow_sequence_v1` works. It supports program, pole, hand, wait and global-point steps
     (`UI53:6604-6656`; `RUN14:1668-1763, 1986-2215`), and V6 uses it.
   - Neither is per region or applied before teaching. The workflow format is how pole and hand reach production (R13).
9. **`usage.py` holds the rules, the order and read-only reads.**
   - The rule: "the pole moves only with the arm still; never move pole or hand with the pad on a surface" (`UC/usage.py:24-25`).
   - `arm_at_rest` (< 1.6°/s for 6 polls, `:44-58`) guards pole moves (`:66-72`).
   - `prestroke` runs hand open → arm still → pole → hand (`:128-138`).
   - `stop_all` runs arm slow stop + pole stop + hand freeze (`:141-150`).
   - *(New)* `pole read` and `hand read` already give read-only values (`:5, 10, 61-63, 95-96`). `usage.py` has no `--sdk` flag, so the SDK must
     be on `PYTHONPATH`, and `hand read` opens the Modbus session and writes the speed register, with no motion.
10. **`pole.py` and `hand.py` are reusable as-is.**
    - Pole: 5 mm tolerance; it waits out the modelled travel time because "the getter leads the real pole by up to 73 mm", and stops on timeout
      (`UC/pole.py:3-58`). Travel is 1.5 s plus distance at 0.059 m/s up or 0.097 m/s down: about 2-7 s per move (full stroke up ≈ 6.8 s).
    - Hand: ANGLE_SET 1486 and ANGLE_ACT 1546. A move succeeds when the channels are within 15 counts or have settled ("fingers closing on a tool
      stop early"); `freeze` re-commands the current angles (`UC/hand.py:3-17, 88-126`).

**Default values that exist**

11. **V6 workflows** (verified from `V6/LEFT_V6_70SERIES_FINAL.json` and `V6/RIGHT_V6_70SERIES_FINAL.json`). Both arms start at pole 200 mm with
    GRIP `[0,0,0,0,150,1000]`.

    | Arm | Before | Pole | Region (map 13 §6.2, inferred) |
    |---|---|---|---|
    | LEFT | program 71 | 150 mm | lid top left, left hinge |
    | LEFT | program 73 | 50 mm | base, pedestal, side |
    | LEFT | lid-open phase onwards (76-80) | 150 mm | |
    | LEFT | program 83 | 50 mm | bowl inside |
    | RIGHT | program 71 | 50 mm | base, pedestal, side |
    | RIGHT | program 74 | 150 mm | lid top |
    | RIGHT | program 75 | 50 mm | right hinge |
    | RIGHT | lid-open phase onwards | 150 mm | |
    | RIGHT | program 83 | 50 mm | floor spillage |

    Right and left hinge differ (50 vs 150 mm).
12. **ALIX commode C teach sheet** (left arm, 2026-09-22):
    - lid top, bowl outside, seat top and rim top at 140 mm, hand 100 % open;
    - bowl inside at 200 mm, 100 % open;
    - rim side, bowl inner back, hinge, lid-seat open and seat-only close at 200 mm, 50 % open.

    "Hand % open = the first four fingers; the thumb is left as it is" (`EXP01/data/teach_files/commode_c_taught_paths/description.txt:3-20`).
    The two sources disagree (for example bowl inside 50 vs 200 mm; fist vs open hand), so defaults are proposals only.
13. **(New) ALIX's flow sheets specify the hand as "fingers %, thumb held".** ALIX switched to that after commanded thumb channels failed to move:
    "both right-arm runs of 2026-09-15 logged 'channels [5, 6] were commanded to move and did NOT'" (`ALIX/tools/taught_tracks.py:343-361`).

**ALIX taught lane**

14. **ALIX treats pole and hand as part of every recording.**
    - The teach sheet carries POLE and HAND per file (`ALIX/tools/taught_pipeline.py:11-27`; strict parser `:183-221`).
    - `SOURCE.json` stores `pole_m`, `hand_state` and `hand_counts` (`TT/source/README.md:14-24`).
    - At run time "pole, then hand" is set before the arm moves (`ALIX/docs/motion/TAUGHT_PIPELINE_README.md:93, 100`), and the fingers open
      (thumb held) before a pole move (`:60`).
    - The reason: "The replay MUST stage the pole here first: a joint-space track is pole-agnostic, so a quarter-height pole sweeps the whole …
      path 225 mm low" (`EXP01/data/teach_files/MANIFEST.yaml:136-139`).
15. **Library equivalents.** `alix_dispatch/peripherals.py` has `Pole` / `Pole.move` and `HandModbus` / `HandModbus.command` (`:169, 227, 462, 601`),
    with a counts ↔ radians conversion that refuses to "invent a hand scale" without `hand_units.py` (`:751-760`).

**Bin**

16. **(New) Lite can neither read nor set the bin.** The control app has no bin code (searched UI53 and RUN14). The bin is driven by the ROS side.
    - "STOWED is the startup default".
    - The direction "pole into the band while the bin is already STOWED" is enforced nowhere in the repo (`ORCH/core/task_config.py:126-150`).
    - `pole_bin_conflict` checks only a target height. The arm base moves with the pole alone (`:62-75`), so the band blocks the whole travel:
      a 100 → 300 mm move crosses it.
17. **(New) The band is provisional.** The band was measured on BinShort (`task_config.py:64-66`), but "the PHYSICAL part has not been remade"
    (`ALIX/ros/description/ugv_description/urdf/bin.xacro:32-34`). V6 starts at 200 mm (inside the band), which implies the bin was not stowed
    or not fitted when V6 ran.

**Testing without hardware**

18. **(Corrected) The emulator helps library-level tests only.** It implements the lift and hand-register calls (`EMU:2051, 2215, 2220, 2369-2416`)
    and arm state and run mode (`EMU:1988, 1995`). But it is in-process: `install()` swaps the SDK modules in the current process (`EMU:34-35`;
    registry `EMU:549`). Its state does not carry across runner subprocesses, so it suits library-level or single-process tests, and the
    subprocess MVP is tested with the fake device. The controller's own SIM mode does not move the lift or hand (map 09 §7), so a SIM run proves
    nothing for R7.
19. **Two hand scales exist in ALIX.** The hardware scale is `ALIX/ros/hw/alix_hw/scripts/hand_units.py:33-60`; the emulator's view scale is
    `TG/sim_bridge.py:95-102`. A 3-D preview must use the hardware scale.

### 2.5 Borrow verdict

**ALIX Studio: Not available — build new.**
The requested capability does not exist in ALIX Studio: per-region pole and hand settings applied to the robot before drag teaching.
- The region model has no pole or hand fields (finding 1).
- The per-task pole field and the derived hand state live in the excluded generator and are executed by ROS at run time (findings 2, 3).
- Studio has no channel to the robot (finding 4).

Small pieces are reusable: the stowed-bin rule and wording, the 0-315 mm range, the Workbench pole UI pattern (named heights, custom height,
band warning), and the 3-D robot model for an optional preview (finding 5).

**Input claim check: mostly inaccurate.** "borrow from alix studio" names the right concept in the wrong place. Studio does carry a per-task
pole height and a pad-derived hand state, but only inside the excluded generator (X1). The value goes into generated configs that the ROS
orchestrator runs, Studio never applies it before a teach, and Studio never lets the user choose the hand. The working apply machinery is
elsewhere:
- the control app: APPLY POLE and APPLY HAND, the GRIP and ALL OPEN presets, and runner `--manual-pole` / `--manual-hand-raw`;
- `usage.py`: the rules, the order and the reads;
- the ALIX taught lane: pole and hand per recording, plus provenance.

| Other source | Verdict | What to take |
|---|---|---|
| Runner v14 (`--manual-pole`, `--manual-hand-raw`, `--manual-hand`, `--workflow-sequence-file`) + `pole.py` + `hand.py` | Borrow with adaptation | Use the commands and device classes unchanged. Add per-region values, Lite's own arm-still check, the checklist, read-back tolerance, the travel-band check and logging. Subprocess first, library later. |
| `usage.py` | Borrow as-is (logic and reads) | `arm_at_rest`, the `prestroke` order, `stop_all`, `pole read`, `hand read`. |
| ALIX taught lane (`taught_pipeline.py`, `taught_tracks.py`, `TT/source/README.md`, `TAUGHT_PIPELINE_README.md`) | Borrow with adaptation | Pole and hand per recording, the `SOURCE.json` keys, the pole-then-hand rule, the "fingers %, thumb held" hand spec. |
| `ORCH/core/task_config.py` `pole_bin_conflict` | Borrow with adaptation | The interlock sentence and band, extended from a target check to a travel-interval check; convert mm to metres. |
| Studio Workbench pole field (`WE/template.html:235-238`; `WE/editor.js:17161-17200`) | Borrow with adaptation (UI pattern only) | Named heights plus a custom height plus the band warning, as the supervisor's pole editor. |
| `alix_dispatch/peripherals.py`, `hand_units.py` | Borrow with adaptation (library target) | Library-grade pole and hand classes; the hardware hand scale for the 3-D preview. |
| V6 FINAL JSON, ALIX commode C teach sheet | Reference data | Seed defaults, marked "proposed". |
| ALIX emulator | Borrow as-is (library-level tests) | Hardware-free tests of the library implementation of the Prepare step. |

### 2.6 What to borrow and what to build

**Borrow**

| Item | From | Adaptation |
|---|---|---|
| Pole and hand commands with read-back | `RUN14:756-909`; `UC/pole.py`; `UC/hand.py` | Call per region from the Prepare step and parse the `final=` values. Option: one `--workflow-sequence-file` run with `[pole, hand]` steps (`RUN14:1986-2215`): one process and connection, one SIGINT to stop. |
| Read-only state | `UC/usage.py:5, 10, 61-63, 95-96` | `usage.py --ip <ip> pole read` and `hand read`, shown on card entry (MVP); later a library call. |
| Preset library | `UI53:2978-2999`; `UC/hand.py:38-40`; `ALIX/tools/taught_tracks.py:343-361`; commode C sheet; `MANIFEST.yaml:139` | Plain-named presets with pictures; thumb-held presets first (§2.3). |
| Arm-still check, prep order, stop-all | `UC/usage.py:44-58, 128-150` | Lite runs `arm_at_rest` itself before every pole move. Stop = pole stop + hand freeze. `prep_order` per region. |
| Stowed-bin interlock | `ORCH/core/task_config.py:58-124`; `WE/editor.js:17180-17185` | Refuse when STOWED and the interval [current, target] overlaps the band, or the current height is inside it. Plain-language refusal (§2.10). |
| Workbench pole UI pattern | `WE/template.html:235-238`; `WE/editor.js:17161-17200` | Supervisor editor: named heights, custom mm, band warning. |
| Provenance keys | `TT/source/README.md:14-24` | Measured `pole_m`, `hand_counts`, preset name, targets and override reason in each sidecar (R1). |
| Default values | `V6/*_FINAL.json`; `EXP01/data/teach_files/commode_c_taught_paths/description.txt:3-20` | Seed per region via the map-13 V6 mapping, marked "proposed", with the ALIX value alongside. Brush, glove, floor-left and drain are TBD. |
| Optional 3-D preview | Studio robot model; `ALIX/ros/hw/alix_hw/scripts/hand_units.py` | `<side>_sliding_plate_joint` = mm / 1000; finger joints from counts with the hardware scale. |
| Emulator for tests | `EMU:1988-2416` | Library-level or single-process tests of the Prepare sequence. |

**Build**

| Item | Approach |
|---|---|
| Per-region settings | Catalogue fields `pole_mm`, `hand {preset, counts[6]}` (`null` = hold), `prep_order`, `defaults_source`, `confirmed_by`. Supervisor editor with range checks and "copy to the same family/phase". |
| Prepare-robot step | Preconditions → checklist → `arm_at_rest` → bin travel check → `prep_order` (optional fingers open → pole → hand) → read-back (pole ±5 mm; hand ±15 counts or settled) → "Ready ✓". Retry and Stop. A supervisor-only override with a reason. |
| Hand-event step | For tool and glove sub-steps: drag off, arm still, operator confirms the grasp point, hand command, read-back. Stored in the catalogue sequence and in the next sub-step's sidecar. |
| Session bin state and placement | Asked at session start with a photo per state (STOWED, EXTENDED, LOWERED, NOT FITTED) and the robot parked at the marked position. Stored in sidecars; shared with R10. |
| Fine-tune, approval, downstream rule | ±10 mm buttons and a preset menu on the card. Values used are logged per recording. "Save as region default" needs supervisor confirmation and flags recordings taught at other values. |
| Tests | Unit tests for the interlocks (band interval, moving arm, drag mode, ranges, holding-fixture lock). Prepare against the fake device (subprocess path) and the emulator (library path). A hardware checklist per arm. |

### 2.7 Recommendation

Do not look for the apply machinery in ALIX Studio; only the concept and a UI pattern are there.
- Store pole and hand per region in the Lite catalogue.
- Apply them in a **Prepare robot** step of the R1 teach card, using the control app's pole and hand commands (subprocesses for the MVP;
  `usage.py` for reads). Follow the region's order: pole then hand, or open, pole, hand.
- Guard the step with Lite's own arm-still check and a short checklist. Check the stowed-bin band over the whole travel. Never Prepare an arm
  that holds the fixture or a tool part-way through a task.
- Show target against measured values, and write the measured values into every recording's provenance file. A recording's own values are
  authoritative downstream.
- Seed defaults from the V6 workflows, with the ALIX commode C sheet beside them, prefer thumb-held presets, and have the supervisor confirm them.
- Later, move the apply sequence into the Lite hardware library (one connection per arm, a stop token), reusing `pole.py` / `hand.py` or
  `alix_dispatch.peripherals`.

### 2.8 Approach

1. **Catalogue fields** (shared with R1): `pole_mm`, `hand: {preset, counts[6]}` with `null` = hold, `prep_order: pole_then_hand |
   open_pole_hand`, `defaults_source`, `confirmed_by`, and hand events in `substeps`.
2. **Seed defaults.**
   - Map each region to its V6 program (map 13 §6.2) and take that program's pole and GRIP.
   - List the ALIX commode C value beside it.
   - Mark regions without a source TBD (brush, glove, floor-left, drain).
   - Propose thumb-held presets where V6's fist is not needed.
   - The requester or the supervisor confirms.
3. **Preset library** with plain names and a small hand picture each (§2.3).
4. **Session settings.** Bin state (STOWED, EXTENDED, LOWERED, NOT FITTED) with photos, and robot-placement confirmation, shared with R1 and R10.
5. **Prepare step,** per arm, inside the R1 card:
   1. Preconditions: connected, REAL mode, not in drag mode, no error codes, no program running, no other client.
   2. Checklist: "The cloth/pad is not touching anything"; "Hands and body are clear of the robot"; "The arm will not hit the toilet or the
      other arm when it moves up or down".
   3. `arm_at_rest`; refuse if the arm is moving.
   4. Bin check: if STOWED, refuse when [min(current, target), max(current, target)] overlaps [153, 282] mm or the current height is inside the
      band. Give the sentence and the advice "deploy the bin, or keep the pole below 153 mm".
   5. Per `prep_order`: optional `--manual-hand 100` (fingers open, thumbs held), then `--manual-pole <mm>`, then `--manual-hand-raw v1..v6`
      (all six known) or `--manual-hand <pct>` (thumb-held presets). Or the same as one `realman_workflow_sequence_v1` file run with
      `--workflow-sequence-file`.
   6. Read-back: parse `MANUAL POLE COMPLETE: … final=Y mm` and `MANUAL HAND RAW COMPLETE: target=[…], final=[…]`. Show ✓ when the pole is
      within ±5 mm and each commanded hand channel within ±15 counts, or the hand reports settled.
   7. On failure: a plain message, Retry, and Stop (= `rm_set_lift_speed(0)` + hand freeze).
6. **Read-only state** on card entry: `usage.py pole read` / `hand read` (MVP), later `Pole.height_m` / `Hand.read` in the Lite library.
7. **Hand-event step** for tool and glove sub-steps (§2.2 (h)).
8. **Record** targets and measured values, preset name and any override reason into the R1 sidecar at save time.
9. **Fine-tune** on the card. Supervisor-confirmed default changes are logged and flag existing recordings (§2.2 (g)).
10. **Optional:** a 3-D preview of pole height and hand pose with the Studio robot model and `hand_units.py`.
11. **Library target:** one connection per arm runs the whole sequence with a stop token.
12. **Tests:**
    - unit tests for the interlocks;
    - Prepare against the fake device (subprocess path) and against the ALIX emulator in a single-process harness (library path);
    - a hardware checklist per arm: 50 → 150 → 200 mm with the bin deployed, GRIP ↔ thumb-held open, Stop during a pole move, read-back values,
      and a gloved-thumb test.

### 2.9 Acceptance criteria

1. Every region record has `pole_mm` in 0-315 and a hand preset with six values in 0-1000 or "hold". The supervisor editor refuses
   out-of-range values. Seeded defaults match the V6 values for covered regions (for example L-R01 50 mm GRIP, R-R04 150 mm GRIP) and are
   marked "proposed".
2. "Prepare robot" moves the pole before the hand (the order is visible in the log) and shows target and measured values. START stays disabled
   until the pole is within ±5 mm and every commanded hand channel within ±15 counts or settled. An override is possible only in supervisor
   mode, and its reason is stored in the sidecar.
3. A pole command is refused while the arm moves faster than 1.6°/s. Pole and hand commands are refused while the arm is in drag mode or is
   marked as holding the fixture or a tool between sub-steps.
4. The checklist items must be ticked before the first move for each region.
5. With the bin STOWED, a 200 mm target is refused with the foul-band explanation. So is a 100 → 300 mm move, which crosses the band, and any
   move while the current height is inside the band. With EXTENDED, LOWERED or NOT FITTED all three proceed.
6. Each recording's sidecar contains the measured `pole_m`, the measured `hand_counts` and the preset name, equal to the read-back shown
   before START.
7. A fine-tuned value used for one recording is stored in that recording's sidecar. The region default changes only after supervisor
   confirmation, and recordings taught at the old value are then flagged "taught at other pole/hand".
8. Against the fake device (subprocess path) and the ALIX emulator (library path) the Prepare step completes. A forced timeout shows a plain
   message with Retry and Stop, and Stop halts the pole and freezes the hand.
9. For a region whose `prep_order` is `pole_then_hand` and whose tool is a brush, the hand is never opened before the pole move.
10. For a tool sub-step sequence, the hand event runs only with drag mode off and the arm still, after operator confirmation, and appears in
    the next sub-step's sidecar.
11. (Optional) The 3-D preview shows the pole at the region's height and the fingers in the preset pose, using the hardware hand scale.

### 2.10 UX notes for non-technical operators

- **Show the pole simply.** "Pole height: 150 mm" with a vertical gauge. Show the hand as a picture with a name ("Fist", "Open hand, thumb
  still", "Half open"), not six numbers; channel values appear only on the supervisor screen.
- **One "Prepare robot" button** that does everything in order, with a progress line ("Moving pole… Moving hand… Ready ✓").
- **The checklist in plain words with a photo:** "The cloth is not touching the toilet." "Keep your hands away from the robot hand."
- **Mark changes.** If values differ from the region default, show a small "changed" note so the next operator notices.
- **Ask about the bin once per session,** with a photo of each position.
- **Lock supervisor editing.** Operators see values and can fine-tune only for the current recording.
- **Refusals are sentences with the fix:** "The bin is stowed, so the pole cannot pass 153-282 mm. Deploy the bin, or keep the pole below 153 mm."

### 2.11 Safety notes

- **The pole moves the whole arm** (the arm base rides the pole). Never move it with the pad on a surface, with the arm moving, during drag mode,
  or while the hand holds the lid, the seat or a tool part-way through a task (`UC/usage.py:24-25`). Neither runner pole path checks the arm, so
  Lite must.
- **Stowed-bin foul band, 153-282 mm.** Refuse any travel that enters the band when the bin is STOWED (`ORCH/core/task_config.py:58-124`); STOWED
  is the start-up default (`:126-150`).
  - V6's start height of 200 mm lies inside the band, so V6 runs assumed the bin was not stowed.
  - The band was measured on BinShort, while the physical bin has not been remade (`bin.xacro:32-34`). Treat the band as provisional and have it
    re-measured or confirmed for the installed bin.
  - Lite cannot read the bin state. A wrong declaration defeats the interlock, so ask with photos.
- **Closing fingers can pinch.** Nobody touches the robot hand while it moves.
- **Tools.** Where the hand holds a tool, do not open the hand before the pole move.
- **Thumbs with a glove.** Commanding the thumb channels may not move them (`ALIX/tools/taught_tracks.py:343-351`). Prefer thumb-held presets.
- **Stop.** A one-click Stop for the Prepare step: pole stop and hand freeze. The hardware E-stop remains the safety stop.
- **SIM mode.** The controller's SIM mode does not move the lift or hand; never treat a SIM run as proof of R7.
- **v53's behaviour.** Manual APPLY POLE/HAND in v53 has no confirmation and no arm-still check; Lite must not inherit that behaviour.

### 2.12 Dependencies, risks and open questions

**Dependencies**
- The R1 catalogue (including sub-steps), the teach card (the Prepare step lives there) and the sidecar.
- The control-app integration layer: pole and hand commands, the `usage.py` reads, Lite's arm-still check and the status command. Current pair:
  UI v53 + runner v14.
- The session bin state, shared with R10 (bin toggle), and the robot placement, shared with R1.
- For the optional preview: the Studio robot model and `hand_units.py` (shared with the R3/R10 viewers).
- Supervisor or requester confirmation of per-region defaults; a confirmed bin band for the installed bin.

**Risks**
- Conflicting defaults (V6 vs the ALIX sheet; fist vs open hand) can make recordings inconsistent across regions unless the supervisor settles them.
- Two hand count → radian scales exist in ALIX. A preview using the emulator scale would mislead.
- The pole read-back leads the real pole by up to 73 mm while it moves (`UC/pole.py:52`). Any custom read must also wait out the modelled travel time.
- Each pole move takes about 2-7 s, a hand move about 1 s, plus runner start-up per process (not measured). This is noticeable over 42
  regions; one workflow process per Prepare reduces it.
- If the foul band is wrong for the installed bin, the interlock either blocks valid heights or misses a collision. A wrong bin declaration
  bypasses it.
- Multi Run, the older per-program pole/hand record, is broken in v53 (TypeError). Use the per-step commands or the workflow format.

**Open questions**
1. Which hand preset per region: V6 GRIP (fist) or ALIX "100 % / 50 % open, thumb held"? How is the glove or cloth worn during teaching?
2. Which pole defaults: V6 (50/150/200 mm) or the ALIX commode C sheet (140/200 mm)?
3. What is the bin state during teaching and production: stowed, extended, lowered or not fitted? Has the physical bin been shortened to
   BinShort?
4. Prep order: plain "pole then hand", or "open fingers → pole → hand" where no tool is held?
5. Are the read-back tolerances of ±5 mm and ±15 counts acceptable?
6. Before a pole move, is the operator's confirmation enough, or should the arm first go to a stored safe pose? The runner supports a controller
   global point (`--global-point`), but that is a motion near the operator.
7. May operators fine-tune at the card, and who approves a new default?
8. Do the mechanism tasks need a hand change part-way, or is the grip held throughout, as in V6?

### 2.13 Priority and effort

**Must · M (about 7-8 person-days, one developer, with tests).**

| Item | Person-days |
|---|---|
| Catalogue fields and supervisor editing | 1.5 |
| Prepare step with interlocks (travel band, preconditions) | 2.5 |
| Reads via `usage.py` and adapter | 0.5 |
| Sidecar integration | 0.5 |
| Fake-device and emulator tests | 1.5 |
| Hardware acceptance | 1 |

The hand-event step adds about 1 day if tool and glove regions are in v1 scope. The optional 3-D preview adds 1-2 days.

---

## 3. Cross-cutting notes

1. **The region catalogue is the backbone** for R1, R2 (per-region editing), R7, R8 (merge order; merges must split where pole or hand change
   and at hand events), R10 (dual timeline) and R13 (program names and workflow steps). Define it once, version it with the deck, and keep the
   deck aliases, sub-steps, hand events and V6 legacy links in it.
2. **The provenance sidecar** (ALIX `SOURCE.json` keys plus a Lite block) carries pole, hand, lid/seat state, bin state, placement, tool frame
   and the measured record period. A recording's own values are authoritative for R3, R4, R8 and R13; the catalogue default is not. Keeping the
   ALIX keys keeps files admissible to the ALIX taught lane.
3. **Hardware seam.**
   - A `TeachDevice` interface, with a runner-v14 subprocess implementation for the MVP: child-process supervision, a stop-drag fallback and a
     small status command.
   - A library implementation later (one connection per arm, stop token).
   - A fake recorder for practice and tests; no drag emulation exists anywhere.
   - The in-process ALIX emulator for library-level pole/hand tests.
4. **One control client per arm.** The ALIX ROS stack (which holds the arm at 100 Hz), the v53 app and a second Lite must not drive the arms
   during a Lite session. The Lite server allows one controlling browser session.
5. **Terminology.** "Region" is the deck teach unit, "sub-step" a part of a region, and "segment" a time window inside a recording (R2's
   "sub-region"). The trajectory HTML's "scaling regions" must be renamed when integrated.
6. **The deck is a V1 draft,** without cleaning times or pole/hand values and with inconsistent codes. The requester should sign off the IDs and the
   sub-step model and supply or approve times and settings.
7. **Session settings shared across requirements:** controller IPs (R4); bin state, with the ALIX names STOWED, EXTENDED and LOWERED plus NOT
   FITTED (R7 foul band, R10 bin toggle); robot placement relative to the toilet, on which all joint-space data depends.
8. **Timing.** The drag-file record period is not documented. 5 ms is supported by file-name durations but marked open in exp01, and Lite
   measures it per recording. The controller plays about 5.55 ms per point.
9. **Control app.** The current pair is UI v53 + runner v14. The runner exists as three byte-identical copies; v48 is older and contained. The
   plan-blocking items are listed once in §0.2.
10. **Colour discipline.** Blue and orange identify arms only, as in the deck. Statuses use the Studio semantic tokens with a glyph and a word.

---

## 4. Change log (verifier pass, 2026-10-13)

| Req | Change | Reason | Evidence |
|---|---|---|---|
| R1 | "Hardware-proven" replaced by "exists end to end; no test record; V6 and the UI history indicate use on the arms". | No test record or log supports "proven". | Maps 08/09/12 contain no hardware-use record; `UC/py programs/` (v25_drag_teach_test … v37_dual_drag_save_popup); `V6/README.txt` ("previously hardware-tested smooth bridge") |
| R1 | Limp-arm safety note corrected: support the arm before START (entering compliance); at STOP the arm holds. | "Limp on release" refers to the arm becoming compliant, not to the stop. | `ALIX/ros/hw/alix_hw/scripts/teach_mode.py:11-16` |
| R1 | Added: no other client may drive the arms (ALIX ROS stack, v53, second Lite); browser control lock. | The ROS stack holds the arm at 100 Hz and a dragged arm creeps back; the runner says to stop the ROS driver first. | `teach_mode.py:18-22`; `RUN14:26` |
| R1 | Added preconditions: no error codes, all joints enabled, no program running; a small status command. | The runner checks these before a run but not before drag teach; ALIX requires all joints enabled. | `RUN14:26-30, 122-141, 912-929`; `EXP01/notes/2026-08-25_phase_a.md:98` |
| R1 | Added crash recovery (orphaned drag session; save the retained recording). | Children run in their own session; `finally` does not run on SIGKILL or the Windows break; the controller keeps the latest recording. | `UI53:5745-5750, 3897-3901`; `RUN14:945-954`; `EXP01/notes/2026-08-25_phase_a.md:95`; map 09 (Windows, inferred) |
| R1 | Save now goes to a temporary name, then a point-count check, then a rename without overwrite; wait 1 s after stop. | The runner writes straight to the final path and returns a point count; ALIX saw the save lag. | `RUN14:957-989`; `EXP01/notes/2026-08-25_phase_a.md:93-94` |
| R1 | Status-line format corrected (`DRAG SAVE COMPLETE: <path> // <N> points`; colon forms). | The draft's parse string did not match the runner output. | `RUN14:929-930, 943, 985-987`; `UI53:5894-5911` |
| R1 | Record-period evidence added; measured period stored per recording; default times flagged as doubling at 10 ms. | 5 ms is supported by file names but still open in exp01; a vendor file is catalogued at an assumed 10 ms. | `ALIX/docs/sources.md:5293`; `EXP01/README.md:63-68, 275-282`; `EXP01/data/teach_files/MANIFEST.yaml:25-26` |
| R1 | Added borrow: Studio `schedule_guards` tier-2 fixture-state timeline logic. | Missed existing code that validates mechanism steps, `implies` physics and required states. | `TG/schedule_guards.py:8-12, 196-236, 449-470` |
| R1 | Board borrow narrowed to CSS and patterns; JavaScript rewritten. | Board JavaScript is bound to the generator (route, gate, points, Workbench) and the C-space pipeline. | `WE/template.html:545-547`; `WE/editor.js:15208-15240, 15410-15460` |
| R1 | Added the sub-step model for mechanisms (no Prepare while an arm holds the fixture; a redo invalidates later sub-steps; counting rule) and for tool/glove hand events. | Drag files hold arm joints only and Lite forbids hand commands in drag mode; V6 alternates mechanism sub-steps. | `EXP01/src/teachlab/teachfile.py:3-12`; `RUN14:912-954, 1702-1717`; `V6/*_FINAL.json` (RUN FIRST / WAIT OTHER steps) |
| R1 | Practice mode with a separate folder; acceptance criteria 4/20 reconciled. | The draft's acceptance criteria (fake files not counted vs a cell showing Taught after Keep) contradicted each other. | Draft acceptance criteria 4 and 18 |
| R1 | Added: the dedicated START/STOP key must not act on the focused control; the foot switch maps only to START/STOP. | In browsers, Space activates the focused button. | Browser behaviour; draft UX note |
| R1 | Added: opening a card or auto-advancing never moves the robot (acceptance criterion 7). | Makes the motion boundary explicit and testable. | Draft (f), §1.8.4 |
| R1 | Added: the physical DRAG button bypasses Lite (UX and risk). | Missed hardware fact. | `teach_mode.py:3, 9, 29` |
| R1 | Added "Awaiting review" status; sidecar gains controller point count, measured period and tool frame. | Crash between save and Keep; tool frame affects drag gravity compensation. | `RUN14:957-989, 1430-1448` |
| R1 | Deck import marked offline; LibreOffice not needed at run time. | Deployment simplicity. | `which libreoffice pdftoppm` on this workstation |
| R1 | Effort recomputed to about 27 days (XL); MVP about 15 (L). | The draft's item sum (about 25.5) exceeded its "20-25" range; new items were added. | §1.13 |
| R7 | Input claim check changed from "inaccurate" to "mostly inaccurate"; Studio Workbench per-task pole field and pad-derived hand state documented; verdict unchanged. | The concept exists in Studio (X1), the apply-before-teach capability does not. | `WE/template.html:229-238`; `WE/editor.js:17161-17200`; `TG/stage_sequence.py:181-207`; `TG/config/glove_frames.yaml:97-109` |
| R7 | Finding corrected: the Page-02 workflow format also carries per-program pole/hand (works, used by V6); Multi Run is not the only record. | The draft said Multi Run was the only one. | `UI53:6604-6656`; `RUN14:1668-1763, 1986-2215`; `V6/LEFT_V6_70SERIES_FINAL.json` |
| R7 | Read-only state: borrow `usage.py pole read` / `hand read` instead of building a new runner flag. | Missed existing reads. | `UC/usage.py:5, 10, 61-63, 95-96` |
| R7 | Bin rule extended from target to the whole travel interval, plus the current-inside-band case. | The base moves with the pole alone; the orchestrator checks only a target and does not enforce the pole-into-band direction; STOWED is the start-up default. | `ORCH/core/task_config.py:62-75, 97-124, 126-150` |
| R7 | Band marked provisional; bin state is operator-declared because Lite cannot read it. | The physical bin has not been remade; the control app has no bin code. | `ALIX/ros/description/ugv_description/urdf/bin.xacro:32-34`; search of UI53/RUN14 |
| R7 | Bin state names aligned with ALIX (STOWED, EXTENDED, LOWERED, plus NOT FITTED). | "DEPLOYED" is not an ALIX state. | `ORCH/core/task_config.py:42-55` |
| R7 | Emulator use limited to library-level or single-process tests; subprocess MVP tested with the fake device. | The emulator is in-process; state does not carry across runner subprocesses. | `EMU:34-35, 549` |
| R7 | Added (g): a recording's own pole/hand are authoritative downstream; default changes flag existing recordings. | A joint-space track replays correctly only at its taught pole height. | `EXP01/data/teach_files/MANIFEST.yaml:136-139` |
| R7 | Added (h): hand events inside tool/glove tasks; no Prepare while holding the fixture or a tool. | Drag files hold no hand actions; a pole or hand change while gripping moves or drops the object. | `RUN14:912-954, 1702-1717` |
| R7 | Thumb-held presets made the default; GRIP kept as the V6 option. | ALIX saw commanded thumb channels not move and switched to "fingers %, thumb held". | `ALIX/tools/taught_tracks.py:343-361` |
| R7 | Added evidence of per-region hand practice (finger pads for inner rim and hinge). | Supports per-region hand settings. | `TG/region_mesh_loader.py:306-314`; `TG/config/glove_frames.yaml:101-109` |
| R7 | Override of the read-back gate limited to supervisor mode. | Non-technical operators should not bypass the data-quality gate. | §2.2 (b) |
| R7 | Option added: Prepare as one `--workflow-sequence-file` run; Lite runs the arm-still check itself. | Fewer processes and one stop signal; neither runner pole path checks the arm. | `RUN14:865-904, 1986-2215, 2174-2178` |
| R7 | Pole move time corrected to about 2-7 s. | Computed from the `pole.py` constants (1.5 s + distance / 0.059 m/s up). | `UC/pole.py:3-14` |
| R7 | "In daily use" removed. | No evidence. | Maps 08, 09, 12 |
