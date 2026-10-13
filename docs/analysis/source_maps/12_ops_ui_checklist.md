# 12 - "RealMan Dual-Arm Operations UI" document vs code (UI v53 / runner v14)

**Document checked:** `/home/newtonjeri/alix_studio_lite_ws/RealMan Dual-Arm Operations UI.txt` (24 sections, says "UI v50 / Runner v13").

**Code checked:** static reading only. Nothing was run against a robot or the network.

| Tag | File (all in `/home/newtonjeri/alix_studio_lite_ws/upload_channel/`) |
|---|---|
| `U` | `rm_trajectory_ui_v53_safe_lid_checkpoint.py`: 11,581 lines, `UI_PROGRAM_VERSION="v53"`, `EXPECTED_RUNNER_VERSION="v14"` (U:45-46) |
| `R` | `rm_upload_fast_v14_safe_transit.py`: 2,702 lines. The `_v52support` and `_v53support` copies are byte-identical (md5 a0b285e6...) |
| `U48` | `rm_trajectory_ui_v48_handoff_continue_fix.py`: the baseline. Its default runner is v13 (U48:6854) |
| also diffed | `rm_trajectory_ui_v51_safe_start_speed.py`, `rm_trajectory_ui_v52_timeline_rules.py`, `rm_upload_fast_v13_overall_speed.py`, and the history in `py programs/` (v11/v12 runners, v17/v18/v21 UIs) |

**Limits and extra sources:**
- **v49/v50 are not in the workspace.** The document describes v50. For features that are missing in v48 but present in v51, the version that added them (v49, v50 or v51) cannot be pinned down.
- **SDK semantics.** I checked them against the SDK docstrings in `/home/newtonjeri/realman_API/butterfli-realman-alix/Python/Robotic_Arm/` (`rm_robot_interface.py`, `rm_ctypes_wrap.py`), read only.
- **Production workflow trace.** The real workflow is `upload_channel/LEFT_V6_70SERIES_FINAL.json` + `RIGHT_V6_70SERIES_FINAL.json`, the same files as in `Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/`. I took the two planning methods (`_page2_program_chunks`, `_page2_build_program_rule_plan`) out of v51, v52 and v53 via `ast` and ran them on these JSONs. This is pure logic: no Tk, no I/O, no robot.
- **"BROKEN" findings are static.** They come from the AST plus normal Python method-binding rules. They have not been reproduced at runtime.

**Status legend:**
- **PRESENT**: matches the document.
- **DIFFERENT**: present, but behaves differently or only partly as described.
- **ABSENT**: not in the code.
- **BROKEN**: the UI control exists, but its code path raises.

---

## 0. Bottom line

- **Almost everything in the document exists in v53/v14.** No documented feature is ABSENT. The checklist has 158 rows (each bullet, sub-bullet and example state):
  - 136 are PRESENT. Some carry caveats in the notes.
  - 20 are DIFFERENT, one of which (Multi Run save/load) is partly broken.
  - 1 is BROKEN (MULTI RUN).
  - 1 is outright WRONG (the version string).
- **Most of the document is still accurate for Page 01.** Page 01's per-arm features are practically unchanged since v48: same code, line numbers shifted by +3 (or +112 after the version-bar insert).
- **The document is outdated where it matters most: the Page 02 dual rules.**
  - It describes the v48/v50 rule "TOGETHER = matching Program IDs run together".
  - Since v52, consecutive TOGETHER rows run as one independent **timeline block per arm**, and Program IDs may differ.
  - Since v53, a positive WAIT before a TOGETHER Program on both arms becomes a **manual lid-cover checkpoint**.
  - The document mentions neither, nor the v51 "Start %" transit speed.
  - With the document's rule (v51 planner), the real production workflow is **rejected**: "Missing on RIGHT: 73 / Missing on LEFT: 74, 75".
- **"Fast switching" (2a) is real but local to one runner process.** There is no long-lived connection. In v53, RUN BOTH starts **one runner process (one SDK connection) per arm per stage**. The production V6 workflow therefore makes **7 stages and 12 connections**, each with a full program-list read and a speed set/restore.
- **"Run-state monitoring" (2b) works as follows.** The runner polls `rm_get_program_run_state` every 50 ms on Page 02 (100 ms on Page 01). "Complete" means it saw `run_state` 1/2 and then 0, with no blocking error codes. The UI only sees the runner's exit code (0 or non-zero), polled every 120 ms.
  - A stop made from the pendant that leaves no error code looks like **successful completion**.
- **Two Page 01 features in the document do not work.**
  - **MULTI RUN never starts.** `_step_preview` is missing `@staticmethod`, so it raises TypeError at U:5159.
  - **Page 01 "LOAD SEQUENCE" (load and run) always errors**, for the same reason, at U:4932.
  - Both have been broken since v21 (`@staticmethod` was present in v17/v18).

---

## 1. Checklist: every section and bullet (v53 UI + v14 runner)

"v48?" says whether v48 already had the bullet. Unless noted otherwise, v48 has the same code at roughly the line minus 3 (before about line 6900) or minus 112 (after it).

### §1 Dual-Robot Control
| Bullet | Status | Evidence (v53 / v14) | v48? | Notes |
|---|---|---|---|---|
| Independent control LEFT RM75 / RIGHT RM75 | PRESENT | Two `ArmControlPanel` instances U:7584-7607 (class U:1922); each command runs its own subprocess U:5712-5752 | yes (U48:7476) | "RM75" is never named in the code. The runner only enforces 7-joint integer points, R:76-89 |
| LEFT `192.168.1.10` | PRESENT | R:47, U:7588, header text U:7506 | yes | Runner IP can be overridden with env `RM_LEFT_IP` (R:47). UI labels are hard-coded |
| RIGHT `192.168.1.103` | PRESENT | R:48, U:7600 | yes | env `RM_RIGHT_IP` |
| UI 50 % / 50 % LEFT/RIGHT workspace | PRESENT | Page 01 U:7571-7572 (`uniform="arms"`); Page 02 U:9181-9182 | yes (U48:7459) | The top control row is 30/70 (system / dual run), U:7612-7617 |
| Independent or coordinated operation | PRESENT | Per-arm commands, plus DUAL RUN U:11246, RUN BOTH .TXT U:8703, START BOTH DRAG U:7841, Page 02 RUN BOTH U:10986 | yes | No global arm lock: Page 01 commands still work on an idle arm during Page 02 RUN BOTH (see §4) |

### §2 Page 01 - Manual + Setup
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Select trajectory `.txt` separately for LEFT and RIGHT | PRESENT | `left/right_track_var` U:6931-6932; per-arm entry + Browse U:2181-2214; custom browser `TrajectoryFileDialog` U:74 | yes | |
| Select Program ID / upload slot | PRESENT | Spinbox 1-100 U:2221; default 25 U:6933-6934; validated U:4234-4239 | yes | |
| Upload trajectory files to controller | PRESENT | UPLOAD ONLY U:2275/4265; UPLOAD + RUN U:2283/4294. Runner `upload()` R:147-175: `rm_send_project` R:159, then a size check against the program list R:165-175 | yes | |
| Multi-file upload | PRESENT | MULTI UPLOAD U:2299 → U:4330; `BatchUploadDialog` with a Program ID per file U:760 | yes (U48:2296) | |
| Select multiple `.txt` files | PRESENT | `TrajectoryFileDialog(multiple=True)` U:4343; SELECT ALL FILES U:202; SELECT FOLDER FILES U:209 | yes | |
| Upload multiple programs sequentially | PRESENT | `_batch_upload_worker` U:4504: one `--list` process plus one upload process per file (U:4537-4541, 4612-4620), then a final list U:4642-4657 | yes | Two process spawns per file |
| Existing-slot replacement confirmation | DIFFERENT | `_ask_batch_confirmation` U:4453-4502 (Yes = replace, No = skip, Cancel = stop batch) | yes (same bug) | The message is built with `\\n`, so the dialog shows literal "\n" (U:4467-4485). Single UPLOAD ONLY shows only a generic "will be replaced" warning (U:4276-4282) and passes `--yes`, which skips the runner's own occupied-slot prompt (R:150-157) |
| Controller program list refresh | PRESENT | REFRESH LIST U:2393 → `_list_slots` U:5486; ↻ SYNC (both arms) U:7762/11200; Page 02 ↻ PROGRAMS U:9067/9216; runner `slots()` reads 10 per page R:102-119 | yes | |
| Run stored controller programs alone | PRESENT | RUN STORED, by ID or by exact/unique-partial name, U:2369/5442; resolver U:4776-4842; runner `run()` R:189-271 | yes | |
| Run one program individually | PRESENT | As above, plus RUN ONLY (runs the current upload slot) U:2291/4694 | yes | |
| Multi Run stored programs | **BROKEN** | MULTI RUN U:2377 opens the dialog. After OK, `_multi_run_setup` calls `self._step_preview(...)` at U:5159 (also U:5083, U:5212). `_step_preview` (U:5000) has no `self` and no `@staticmethod`, so it raises TypeError: nothing runs, and the traceback goes only to stderr. The runner side (`--sequence-file` → `run_sequence` R:393-753) is fine | yes, same bug (U48:4997/5156) | The decorator was lost in v21; v17/v18 have `@staticmethod` |
| Save and load Multi Run sequences | DIFFERENT (partly BROKEN) | SAVE and LOAD inside the Multi Run dialog work (U:1277/1284, U:1684-1895, format `realman_multi_run_sequence` v3), but anything loaded can't be run (MULTI RUN bug). Page 01 "📂▶ LOAD SEQUENCE" (U:2385 → U:5019) always fails: `_read_saved_sequence_main` calls `self._normalize_hand_for_main(...)` at U:4932, and that method lacks `self`/staticmethod (U:4853, 4876, 4885). The TypeError is caught and shown as "Invalid Hand / Pole / Wait value" | yes, same bug | |

### §3 Trajectory Execution
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Run uploaded/stored trajectory by Program ID | PRESENT | `rm_set_program_id_run`: Page 01 R:223, Multi Run R:629, Page 02 R:1873 | yes | |
| Program speed control ("global points speed") | DIFFERENT | **Page 01:** runs expose no speed. The UI never passes `--run-speed`, so the runner default of 20 % always applies (R:2337). **Page 02:** each Program row has **"Start %"** (default 30) = transit/approach speed only (U:6088-6093, 6176, 6510-6547; R:1680-1699, R:1873), since v51/v14. The v48 per-Program "Speed %" (default 20, U48:6170) survives only as a legacy field, always 100. **Global Points** have their own MoveJ/MoveL Speed % (default 20) U:2506, R:1467-1523 | v48 had a per-Program "Speed %" | The document's wording does not match any single code concept |
| Overall controller speed control | PRESENT | Page 02 OVERALL SPEED % U:9114-9140 → `--workflow-overall-speed` U:10091/9317 → `rm_set_plan_speed`, once at workflow start (R:2115-2132) and again before each Program (R:1855-1866) | yes | "respective to global points speed" is not a code concept. Global-point steps in a workflow simply run under the same plan speed |
| Page 02 overall speed default 100 % | PRESENT | U:6977, R:2335 | yes | Values outside 1-100 or non-numeric are silently clamped (or set to 100), U:9292-9300 |
| Fast switching between stored programs ("need to check") | DIFFERENT | Answered in §2(a) | runner logic yes (v12+) | Applies inside one runner process only |
| Reduced delay between trajectories | PRESENT | Quick settle capped at 0.6/1.0 s (R:1766-1804, 1947-1956), against the legacy `settle()` of up to 5 s (R:178-186) | yes | |
| Controller run-state monitoring ("need to check") | PRESENT | Answered in §2(b). `rm_get_program_run_state` polled at R:1890 (every 50 ms), R:230 and R:643 (every 100 ms) | yes | |
| Detects still running / completed | PRESENT | R:1897-1907 | yes | Caveats in §2(b) |

### §4 Dual Stored Program Run (Page 01 top row)
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Select one LEFT stored program | PRESENT | Combo "L" U:7710-7733, bound to the left panel's `stored_program_var` | yes (U48:7598) | |
| Select one RIGHT stored program | PRESENT | Combo "R" U:7738-7759 | yes | |
| Start both approximately together | PRESENT | `_run_both_programs` U:11246-11380 launches two `--run SLOT` processes | yes | |
| Synchronized launch using a barrier | PRESENT | `threading.Barrier(2)` U:11343; `start_barrier.wait(timeout=8.0)` U:5693, right before `Popen` at U:5750 | yes | Only the process launch is synchronized. Each runner then loads the SDK, connects, reads the full program list and checks state (R:189-204) before `rm_set_program_id_run`. The skew in motion start is not bounded. Defaults apply: transit 20 %, restore 50 % |
| Common STOP BOTH | PRESENT | U:7790 → `_stop_both` U:11437 (no confirmation) | yes | |

### §5 Drag & Teach
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Individual LEFT Drag Teach | PRESENT | DRAG TEACH tab U:2700-2810 | yes (U48:2723) | |
| Individual RIGHT Drag Teach | PRESENT | Same panel class | yes | |
| START DRAG + RECORD | PRESENT | U:2726 → `_drag_teach_toggle` U:3877 → `--drag-session` U:3941 → `rm_start_drag_teach(1)` R:923 | yes | |
| STOP DRAG | PRESENT | The same button toggles to "■ STOP DRAG" (U:3950, 5596). It sends SIGINT, and the runner calls `rm_stop_drag_teach` (R:938) | yes | |
| Save recorded trajectory locally as `.txt` | PRESENT | SAVE LOCAL U:2805/3983 → `rm_save_trajectory` R:968. Default path `~/Downloads/RealMan_DragTeach/<side>_drag_<stamp>.txt` (U:3840-3850) | yes | |
| Move robot back to recorded start pose | PRESENT | MOVE TO START + PLAYBACK U:2741/4019 → `rm_drag_trajectory_origin(1)` R:1182 | yes | |
| Playback recorded trajectory | PRESENT | `rm_run_drag_trajectory(1)` R:1190 (blocking). On Ctrl+C: `rm_stop_drag_trajectory` R:1201 plus slow stop | yes | |
| Common START BOTH DRAG | PRESENT | U:7532 → U:7841 (barrier launch) | yes (U48:7420) | |
| Common STOP BOTH DRAG | PRESENT | U:7540 → U:8023 | yes | |
| Save both LEFT and RIGHT recordings | PRESENT | Popup after STOP BOTH DRAG U:8088-8275; `--drag-save` per arm U:8277-8400; refuses identical file names U:8298 | yes | |
| Run both saved Drag Teach `.txt` files | PRESENT | RUN BOTH .TXT U:7549 → popup U:8472. It uploads both files to Program 99 on each controller by default, overwriting it (U:6966-6967, U:8703-8840), then runs them with a barrier `--run` (U:8896-8952) | yes (U48:7437) | Program 99 silently gets overwritten |

### §6 Global Points (POINTS tab)
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Read Global Points from each controller | PRESENT | `--global-list` U:3212 → `rm_get_global_waypoints_list`, 20 per page (R:1320-1369); output parsed at U:5768-5790 | yes (U48:2458) | |
| Refresh list | PRESENT | U:2461; Page 02 ↻ GLOBAL POINTS U:9074 | yes | |
| Select multiple points | PRESENT | Listbox `selectmode="extended"` U:2472, plus a search filter U:2436 | yes | |
| MoveJ | PRESENT | U:2552 → `rm_movej(joint, speed, 0, 0, 1)` R:1498 | yes | |
| MoveL | PRESENT | U:2560. The runner switches to the point's work/tool frame, calls `rm_movel` (R:1500-1523), then restores the frames (R:1557-1559) | yes | |
| Adjustable speed % | PRESENT | U:2506; validated 1-100 U:3280-3317 | yes | |
| Optional wait after reaching a point | PRESENT | U:2529; R:1530-1538 | yes | |
| Build ordered sequence | PRESENT | ADD ↓ U:2576/3385; table U:2595-2610; cells editable U:3521-3668 | yes | |
| Move Up / Down | PRESENT | U:3479-3504 | yes | |
| Remove / Clear | PRESENT | U:3463, U:3506 | yes | |
| Run Global Point sequence | PRESENT | U:2568 → `_run_global_point_sequence` U:3670 → `--global-sequence-file` U:3769 → R:1615-1665. Every point is validated before any motion (R:1619-1626) | yes | |
| Uses saved joint/pose information | PRESENT | `rm_get_given_global_waypoint` R:1395-1417 (checks at least 7 joints and 6 pose values) | yes | |

### §7 Hand Control (HAND / POLE tab)
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Independent LEFT/RIGHT hand | PRESENT | One per panel, U:2825-3000 | yes (U48:2949) | |
| Six channels (Little, Ring, Middle, Index, Thumb Bend, Thumb Rotate) | PRESENT | U:2004-2011; same order as `hand.py` | yes | |
| Range 0-1000 | PRESENT | U:2900-2948; runner validates at R:818-823 | yes | |
| Slider control | PRESENT | U:2912 | yes | |
| Direct numeric entry | PRESENT | U:2948 | yes | |
| Apply all six channels together | PRESENT | APPLY HAND U:2952/4142 → `--manual-hand-raw` U:4160 → R:804-862 (opens Modbus, writes 6 registers, waits for arrive/settle, closes) | yes | No confirmation popup, by design (U:4167) |
| GRIP preset `0,0,0,0,150,1000` | PRESENT | U:2980; applies immediately (U:4127-4140) | yes | |
| ALL OPEN preset `1000 x6` | PRESENT | U:2995 | yes | |

### §8 Pole / Lift
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Independent LEFT/RIGHT pole | PRESENT | U:3021-3110 | yes (U48:3018) | |
| Requested height in mm | PRESENT | Slider 0-315 plus entry U:3031-3075 | yes | Rounded to whole mm (U:4193) |
| Manual Apply | PRESENT | APPLY POLE U:3102/4176 → `--manual-pole` U:4197 → R:865-909 (`Pole.move`) | yes | No confirmation popup |
| Pole steps inside automated sequences | PRESENT | Page 02 `pole` step U:6604, R:1700-1704, R:2177-2181; Multi Run `pole_mm` R:546-575 | yes | |

### §9 Simulation / Real Mode
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Independent mode per arm | PRESENT | U:2080-2110 | yes (U48:2079-2095) | |
| SIMULATION | PRESENT | SIM U:2082 → `--set-arm-mode simulation` U:3814 → `rm_set_arm_run_mode(0)` R:1238 | yes | No confirmation |
| REAL | PRESENT | REAL U:2090 (mode 1) | yes | |
| CHECK MODE | PRESENT | CHECK U:2098/3785 → `rm_get_arm_run_mode` R:1216 | yes | |
| Controller mode read-back | PRESENT | Setting a mode verifies it by read-back (R:1244-1262); badge U:2071-2075; parse U:5874-5887 | yes | |
| REAL requires confirmation | PRESENT | U:3802-3809 | yes | |

### §10 Page 02 - Dual Sequence Builder
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Separate LEFT workflow builder | PRESENT | `WorkflowSequencePanel` U:6074, instantiated U:9194 | yes (U48:6071) | |
| Separate RIGHT workflow builder | PRESENT | U:9202 | yes | |
| Build sequences without modifying Python | PRESENT | | yes | |
| Step types Program / Global Point / Pole / Hand / Wait | PRESENT | U:6550, 6555, 6604, 6640, 6620; runner `normalize_workflow_step` R:1668-1748 | yes | Builder sub-tabs PROGRAM / GLOBAL POINT / HAND / POLE / WAIT (U:6153-6155) are not documented |
| Move steps Up / Down | PRESENT | U:6731-6743 | yes | |
| Duplicate steps | PRESENT | U:6745 | yes | |
| Remove steps | PRESENT | U:6753 | yes | |
| Clear sequence | PRESENT | U:6762-6772 (asks for confirmation) | yes | |
| Save sequence as JSON | PRESENT | U:6794-6817 (default `~/Downloads/<side>_workflow_sequence.json`) | yes | |
| Load sequence JSON | DIFFERENT | U:6819-6861 | yes | Only `format` and the steps list are checked. `side` is not checked, so a LEFT file loads into RIGHT silently. Since v51, `transit_speed=30` is back-filled (U:6836-6851) |

### §11 Per-Program Dual Rules
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Every Program row has its own dual rule | PRESENT | Combo U:6192-6205; stored as `dual_rule` U:6547 | yes | The runner ignores the field (R:1693-1699) |
| TOGETHER: "matching LEFT and RIGHT Program IDs run together" | **DIFFERENT** | Since v52, consecutive TOGETHER rows of each arm are concatenated into **one timeline block per arm** (`combine` U:9733-9749; U:9917-9949). IDs and row counts may differ. The two blocks are launched together (barrier U:10209-10233), but programs inside a block are **not paired or re-synchronized** | v48: paired by matching ID, in LEFT order (U48:9591-9604) | |
| RUN FIRST: this arm finishes first | PRESENT | U:9976-9982 → serial stage U:10323-10453 | yes | |
| WAIT OTHER: waits until the opposite arm finishes | PRESENT | U:9983-9989; second arm starts at U:10455-10502 | yes | |
| L/R paired by matching Program ID | **DIFFERENT** | Only RUN FIRST/WAIT OTHER pairs must match. They are paired **by position** in the program order and then checked for equal IDs (U:9952-9972) | v48: all rules paired by ID | |

### §12 Mixed Parallel + Sequential Operation
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Different parts of one sequence can behave differently | PRESENT | Planner U:9666-10017 | yes | |
| Example: 1-7 together, L8 → R8 → L9 → R9, 10+ together | DIFFERENT | Accepted by v53. The production V6 workflow is exactly this shape: 71-76 TOGETHER, 78/79 L→R, 80/83 TOGETHER (§2a trace) | yes (as per-ID pairs) | In v53, "1-7 together" means two independent back-to-back blocks. A positive WAIT in that region (on both arms) becomes a manual lid checkpoint. The document's per-ID rule (v48/v51) **rejects** the production workflow |
| No separate execution MODE needed | PRESENT | `_page2_execution_mode()` returns "PROGRAM_RULES" (U:9426-9428); the legacy one-arm and alternate modes are unreachable dead code | yes | |

### §13 Automatic Program Handoff
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| No fixed timer | PRESENT | Monitors poll every 120 ms (U:10376-10453) | yes | |
| Monitors actual program execution | PRESENT (indirect) | The runner polls `rm_get_program_run_state` (R:1886-1938); the UI only sees the exit code (U:5968-5972) | yes | |
| Waiting arm starts only after successful completion | PRESENT | `last_command_rc == 0` (U:10715-10717) gates U:10403-10453 | yes | |
| If the first program fails or is stopped, the waiting arm does not start | DIFFERENT | U:10403-10416 sets the waiting arm to "NOT STARTED" | yes | True for UI stops and runner-detected failures. A stop made **outside the UI** (pendant/another client) with no error code ends with `run_state` 0, which counts as COMPLETE (R:1905-1907). The handoff then proceeds, automatically if the checkbox is unchecked |

### §14 Optional Handoff Confirmation
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| CONFIRM BEFORE WAITING ARM checkbox | PRESENT | U:9104-9112; **checked by default** (U:6989) | yes (U48:8994) | Disabled while RUN BOTH runs (U:11077) |
| Checked → popup → YES → waiting arm starts | PRESENT | U:10433-10451 | yes | The popup text has literal "\n\n" (U:10436-10437) |
| Unchecked → automatic | PRESENT | U:10429-10431, U:10453 | yes | Applies only to RUN FIRST/WAIT OTHER handoffs. The v53 lid checkpoint always asks |

### §15 Live Program Status
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Dedicated LEFT live-status display | PRESENT | "LIVE ARM STATUS" U:9155-9178; vars U:6979-6980 | **no** (added after v48) | |
| Dedicated RIGHT live-status display | PRESENT | Same | **no** | |
| Shows current Program ID and state | DIFFERENT | `_page2_program_live` U:10026-10027 | no | For TOGETHER stages the "Program" shown is the **stage label** `TIMELINE-n` / `LID-PREP-n` (U:9855, 9878, 9906, 9940), not the program actually running inside the block. Serial stages show real IDs |
| `RUNNING` | PRESENT | U:10215, U:10256 | no | |
| `RUNNING // RUN FIRST` | PRESENT | U:10350 | no | |
| `WAITING FOR LEFT` / `WAITING FOR RIGHT` | PRESENT | U:10352 | no | |
| `WAITING // CONFIRMATION REQUIRED` | PRESENT | U:10426 | no | |
| `WAITING // AUTO HANDOFF` | PRESENT | U:10430 | no | |
| `RUNNING // SECOND` | PRESENT | U:10481 | no | |
| `COMPLETE ✓` | PRESENT | U:10258, 10304, 10540 | no | |
| `FAILED rc=...` | PRESENT | U:10261, 10407, 10530 | no | Every failure mode collapses to rc=1 |
| `STOP REQUESTED` | DIFFERENT | U:11177-11178 | no | Only transient: the next 120 ms monitor tick overwrites it with "RUNNING", then "FAILED rc=1". Undocumented states also exist: PREPARING, READY, NOT STARTED, HANDOFF CANCELLED, FINAL AUX // RUNNING, SEQUENCE COMPLETE ✓, PAUSED // MANUAL LID CHECK, READY // LID CONFIRMED, IDLE // CHECKPOINT CANCELLED |

### §16 Program Completion Detection
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Tracks the actual subprocess/controller result | PRESENT | `last_command_rc = rc` is set right after `wait()` and before any Tk callback (U:5968-5972) | yes (U48:5969) | |
| Does not depend only on UI status text | DIFFERENT | Page 02 rules use the rc (U:10715) | yes | Page 01 DUAL RUN and RUN BOTH .TXT still judge by status-text prefix (U:11418-11419, U:8422), for display only |
| Prevents timing/race problems | DIFFERENT | | yes | Fixes the status-text race. Two narrow windows remain: stage start before `Popen`, and process exit before the rc is stored (see §2b). One of them is not fail-safe |
| Continues automatically to the next rule stage | PRESENT | U:10316-10321, 10565-10568 → `_start_next_page2_rule_stage` U:10098 (waits until both arms are idle, U:10117-10127) | yes | |

### §17 Fast Program Switching (details in §2a)
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Persistent controller connection during Page 02 workflows | **DIFFERENT** | One connection per **runner process** (R:2608-2614, closed R:2698). A single-arm Page 02 run = 1 process. RUN BOTH = 1 process per arm **per stage** | v48: 1 process per arm **per Program** in RUN BOTH | |
| Avoids reading the full program list between trajectories | DIFFERENT | Read once per process (R:2019-2020), i.e. once per stage in RUN BOTH | yes | |
| Avoids per-program speed restoration | PRESENT (within a process) | Restore only in `finally` (R:2262-2277), but the overall speed is **re-applied** before every Program (R:1857) | yes | Each process still restores to 50 % at its end, so every RUN BOTH stage does |
| Short stability check | PRESENT | `workflow_quick_settle` R:1766-1804: 4 deg/s, 50 ms samples, 2 calm samples, cap 0.6 s / 1.0 s. **Advisory only**: the workflow continues on timeout | yes | |
| Designed to reduce transition delay | PRESENT | Runner v12 and later | yes | |

### §18 Overall Speed Control
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Page 02 OVERALL SPEED % | PRESENT | U:9114-9140 | yes (U48:9004) | |
| Default 100 % | PRESENT | U:6977, R:2335 | yes | |
| Applied before workflow execution | PRESENT | R:2115-2132 | yes | |
| Reapplied before stored Program launches | PRESENT | R:1855-1866 | yes | See the speed-semantics caveat in §2a (`rm_set_program_id_run` speed) |
| Prevents slow travel to the next start pose (rationale) | DIFFERENT | | n/a | A design claim the code cannot verify. Undocumented: the speed is set back to **50 %** at the end of every runner process (default `--restore-speed`, R:2340; the UI never passes it) |

### §19 Safety / Stop Controls
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Immediate SLOW STOP | PRESENT | `_slow_stop` U:6028-6073 sends SIGINT to the runner's process group (U:6061). The runner catches KeyboardInterrupt and calls `rm_set_arm_slow_stop` (R:257, 731, 1551, 1661, 2254) | yes | Software stop only; the hardware E-stop is still referenced in dialogs |
| No confirmation required | PRESENT | U:6046 | yes | |
| STOP LEFT | PRESENT | Per-arm ■ SLOW STOP: header U:2104 and RUN tab U:2305; Page 02 ■ STOP LEFT U:6460 → U:9417-9424 | yes | |
| STOP RIGHT | PRESENT | Same | yes | |
| STOP BOTH | PRESENT | Page 01 U:11437; Page 02 U:11171-11187 | yes | |
| Stopping a RUN FIRST program prevents WAIT OTHER from starting | PRESENT | U:10403-10416 | yes | |
| Errors stop the coordinated workflow rather than continuing blindly | DIFFERENT | | yes | Stops only at stage level. In a TOGETHER stage, if one arm fails the **other arm is not stopped**: it runs its whole block before the workflow stops (U:10273-10279). The runner never aborts a running program on an error code. Codes are sampled every 0.5 s and judged only at program end (R:1925-1936, 1958-1967) |

### §20 Telemetry
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Separate LEFT/RIGHT telemetry | PRESENT | Per-arm TELEMETRY tab U:3130-3180 | yes | Only visible on Page 01; Page 02 has no log view |
| Live command output | PRESENT | Unbuffered `-u` runner (U:4255-4262); stdout streamed line by line (U:5750-5752) | yes | |
| Robot/API errors | PRESENT | Runner prints `!! arm reports ...` and `ret=` codes (R:1929-1936) | yes | |
| Program execution messages | PRESENT | WORKFLOW / SEQUENCE / RUN lines | yes | |
| Runtime information | PRESENT | start ack, program seconds, FAST SWITCH seconds (R:1897-1903, R:1971-1981) | yes | |
| Warning information | PRESENT | | yes | |
| 4105 treated as warning-only where applicable | PRESENT | `WARN_ONLY={4105}` R:57; honoured in Multi Run (R:435-451) and Page 02 (R:1835-1850, R:2044-2056) | yes | Not honoured in the Page 01 single-run pre-check: **any** code, including 4105, refuses the start (R:198-200) |
| Auto-scroll does not force scrolling while reading | PRESENT | `_append_log` U:5630-5645 | yes (U48:5627) | Each new command/stage **clears** the arm's log (U:5668, U:5719), so output from earlier RUN BOTH stages is lost |

### §21 Runtime Display
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Individual arm runtime | DIFFERENT | Per-arm timer U:5521-5554 starts only on recognized runner lines ("running (", SEQUENCE RUNNING, GLOBAL..., DRAG PLAYBACK; U:5949-5951 etc.) | yes | The UI never parses `WORKFLOW ...` lines (no "WORKFLOW" string anywhere in U), so per-arm runtime **stays at 0.0 s during Page 02 runs** |
| Dual sequence runtime | PRESENT | Page 02 `page2_runtime_var` updated each tick (U:10724-10731); Page 01 dual run U:11386-11390 | yes | Single-arm Page 02 runs show no runtime (U:9401-9415) |
| Live elapsed time | PRESENT | 100-120 ms refresh | yes | |

### §22 Software Version Display
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Permanent bar above Page 01 and Page 02 | PRESENT | U:7426-7454 | **no** (added after v48) | |
| UI program version | PRESENT | U:45, U:7417 | no | |
| Current UI filename | PRESENT | U:7414 | no | |
| Runner version | DIFFERENT | Taken from the runner **filename** with regex `_v(\d+)` (U:7391-7404) | no | The runner has no version constant |
| Current runner filename | PRESENT | U:7404 | no | |
| Python version | PRESENT | U:7408-7412 | no | |
| Dual-arm indicator | PRESENT | "● DUAL ARM" U:7451 (also "● DUAL SYSTEM" U:7520) | no | |
| "Current version: UI v50 / Runner v13" | **WRONG** | It is **v53 / v14** (U:45-46; runner preference list U:6891) | - | |
| Runner version updates when another runner is selected | PRESENT | `trace_add` U:6923-6928 → U:7422-7424 | no | `EXPECTED_RUNNER_VERSION` is never enforced (only shown when no runner is selected, U:7394), so there is no mismatch warning |

### §23 User Interface
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Dark robotics theme | PRESENT | Palette U:57-71; `_apply_robot_theme` U:7002-7309 | yes | |
| High-contrast buttons/status | PRESENT | | yes | |
| White blinking cursor in editable fields | PRESENT | `insertbackground="#FFFFFF"`, `insertwidth=3` on 19 widgets; blinking is the Tk default | yes | |
| Tabs RUN / POINTS / DRAG TEACH / HAND / POLE / TELEMETRY | PRESENT | U:2162-2166 (Page 01, per arm) | yes (U48:2159-2163) | |
| Color-coded Run / Upload / Stop / List | PRESENT | U:7191-7195 (Upload purple, Run green, List blue, Stop red, Orange) | yes | |

### §24 JSON Workflow Support
| Bullet | Status | Evidence | v48? | Notes |
|---|---|---|---|---|
| Format `realman_workflow_sequence_v1` | PRESENT | U:6776, U:10078; R:1751-1763 | yes | |
| Program Dual Rules saved inside the JSON | PRESENT | `dual_rule` per program step (U:6547) | yes | |
| Program IDs, waits, hand values, pole heights and speeds preserved | DIFFERENT | IDs, waits, hand, pole and global speeds are kept | yes | Since v51/v14 the per-Program `speed` is **no longer used**. Rows without `transit_speed` get 30 % on load (U:6836-6851); new rows write `"speed": 100` + `transit_speed` (U:6540-6546). Under a v13 runner, `"speed": 100` would become the transit speed (§4) |
| Load JSON and run directly from Page 02 | PRESENT | U:6819, then RUN LEFT/RIGHT SEQUENCE or RUN BOTH | yes | |

---

## 2. The two items the document marks "need to check"

### 2(a) Fast switching between stored trajectory programs (§3, §17)

**Verdict:** implemented, but only inside a single runner process.
- **Where it lives.** It is the runner's Page 02 path (`run_workflow_sequence` + `run_workflow_program_fast` + `workflow_quick_settle`), introduced in runner v12. v13 added the overall speed; v14 only renamed `speed` to `transit_speed`.
- **No long-lived connection.** There is no long-lived runner or connection anywhere. Every UI action spawns `python -u rm_upload_fast_v14_safe_transit.py ...` (U:4255-4262, U:5712-5752). That process opens one SDK connection and closes it when it exits.

**Exact mechanism (one Page 02 runner process):**

| Step | Code | SDK calls / timing |
|---|---|---|
| UI writes the JSON and spawns the runner with `--workflow-sequence-file F --workflow-overall-speed N --side S --yes` | Single arm: U:9302-9322. RUN BOTH stage: U:10055-10096 (temp files `/tmp/realman_dual_sequence_ui/<side>_rule_<idx>_<pid>.json`) | - |
| Connect | R:2608-2614 | `RoboticArm(RM_TRIPLE_MODE_E)`; `rm_create_robot_arm(ip, 8080, 3)`; `rm_set_timeout(1500)` (1.5 s reply timeout) |
| Read program list **once** (only if the file has Program steps) | R:2019-2033 | `rm_get_program_trajectory_list`, 10 per page (R:102-119). With about 70 programs on a controller that is about 8 round trips |
| Workflow pre-checks | R:2035-2064 | `rm_get_current_arm_state` (blocking codes refuse, 4105 tolerated); `rm_get_program_run_state` must be idle |
| Set plan speed once | R:2115-2132 | `rm_set_plan_speed(overall)` |
| Per Program: pre-checks | R:1823-1854 | `rm_get_current_arm_state`; `rm_get_program_run_state` (refuse if run_state is 1 or 2) |
| Per Program: re-apply overall speed | R:1855-1866 | `rm_set_plan_speed(overall)` |
| Per Program: start | R:1873 | `rm_set_program_id_run(slot, transit_speed, 0)` (non-blocking) |
| Per Program: monitor | R:1886-1938 | `rm_get_program_run_state` every **50 ms** (R:1938); `rm_get_current_arm_state` every **0.5 s** for error codes (R:1925-1936); 5 s start window; `--timeout` 300 s, then slow stop |
| Per Program: transition guard | R:1947-1956 → R:1766-1804 | `workflow_quick_settle`: samples `rm_get_current_arm_state` every 50 ms plus RTT. Calm when max joint delta / 0.05 s < **4.0 deg/s** for **2 consecutive samples**. Cap **0.60 s** if the next step is a Program, else **1.00 s** |
| Per Program: final check, log | R:1958-1983 | Final `rm_get_current_arm_state`; any code other than 4105 fails. Prints `WORKFLOW FAST SWITCH i/n: X s` |
| End (success, failure or Ctrl+C) | R:2262-2277 | `rm_set_plan_speed(restore)`, restore = 50 (default R:2340) |
| Disconnect | R:2698 | `rm_delete_robot_arm()` |

Between two programs in the same process:
- **Skipped:** list re-read, full `settle()` (2 deg/s, 3 samples at 100 ms, up to 5 s; R:178-186), and the per-program speed restore.
- **Remaining work:** about 3 small state reads, 1 plan-speed write, the start acknowledgement, plus the quick settle (about 0.1-0.6 s).

**Process/connection model per UI path:**

| UI path | Runner processes = controller connections |
|---|---|
| Page 02 "▶ RUN LEFT/RIGHT SEQUENCE" (single arm) | **1 per whole workflow**, a true persistent connection (U:9324-9399) |
| Page 02 "RUN BOTH // SAFE TIMELINE" (v53) | **1 per arm per stage.** Stages are: each TOGETHER timeline block, each LID-PREP, each RUN FIRST program, each WAIT OTHER program, and the tail (U:10098-10712) |
| Page 02 RUN BOTH in v48-v51 (the document's era) | 1 per arm **per Program**: every TOGETHER pair and every serial program was its own stage (U48:9591-9604) |
| Page 01 RUN STORED / RUN ONLY / UPLOAD+RUN / DUAL RUN | 1 per program. Full checks, legacy settle, restore 50 % (R:189-271) |
| Page 01 MULTI RUN (`run_sequence`) | Would be 1 per sequence with a full settle after each program (R:393-753), but the UI path is BROKEN (§1 §2) |

**Production example: the V6 70-series LEFT/RIGHT JSONs, planner executed as described above.**

v53 produces **7 stages and 12 runner processes**:

1. **TIMELINE-1**
   - LEFT: pole 200, hand, pole 150, **71**, pole 50, **73**
   - RIGHT: pole 200, hand, pole 50, **71**, pole 150, **74**, pole 50, **75**
2. **LID-PREP-1**: pole 150 + wait 2 s on each arm.
3. **LID-CHECK-1**: manual dialog.
4. **TIMELINE-2**: **76** on each arm.
5. **Serial 78**: LEFT, then RIGHT.
6. **Serial 79**: LEFT, then RIGHT.
7. **TIMELINE-3**: **80**, pole 50, **83** on each arm.

Each stage process repeats interpreter start, SDK import, connect, paged list read, pre-checks, speed set, and speed restore.

The other versions on the same JSONs:
- **v52:** 4 stages, 8 processes. The 2 s "MANUAL LID COVER OPEN" wait was just a 2 s wait.
- **v51 (= the document's semantics):** refuses the workflow.

The package's own README (`Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/README.txt`) records that v51 could not run this timeline, and that the manual checkpoint "is not automatically paused" by the runner. v52 and v53 address those two points.

**Measured timings in comments:**
- **None in the UI or runner.** The runner prints measurements into telemetry at run time: `start ack`, `WORKFLOW PROGRAM COMPLETE ... s`, `WORKFLOW FAST SWITCH ... s`. Nothing measured is recorded in the repo.
- **Measured constants elsewhere:**
  - `pole.py`: slew 0.059 m/s up and 0.097 m/s down at 100 %, plus a 1.5 s creep, and `move()` also waits out the modelled travel time.
  - `hand.py`: bulk register write about 63 ms; stroke model 115 ms + 373 x span / speed.
  - The V6 README: 5 ms/point cadence; pole 10 → 300 mm in 4.0 s; **1.380 s assumed per 50 ↔ 150 mm pole move** for time-matching.

**Timing implication (estimate from those constants; verify on hardware):**
- **Per pole move.** The runner's pole step lasts at least 1.5 s + distance / slew: about **3.19 s up / 2.53 s down for 100 mm**, against the 1.38 s the time-matched V6 plan assumes.
- **Over TIMELINE-1.** RIGHT has one more pole move and one more program switch than LEFT, so RIGHT finishes roughly **1.8-2.9 s later** than LEFT. The offline plan has them equal (91.3-92.7 s each).
- **Why it matters.** Blocks are never re-synchronized internally. The README's intent ("LEFT bridge intervals ... run while the RIGHT pole moves") will drift by about that much. The switching itself is fast; the aux steps dominate inter-program time.

**Safety risks:**
1. **The stability check is advisory.** If the arm is not calm within 0.6 or 1.0 s, the next program is still sent; only the log suffix changes (R:1980). The threshold is looser than legacy: 4 deg/s with 2 samples, against 2 deg/s with 3 samples and up to 5 s.
2. **The speed restore is deferred to process end.** If the runner is killed, crashes, or the PC loses power, the controller stays at the overall speed (100 % by default). The runner's author notes this setting "survives power cycles" (R:32-33). A normal SLOW STOP (SIGINT) still restores 50 %.
3. **Speed semantics are unverified.**
   - The SDK docstring for `rm_set_program_id_run` says `speed` = "speed to run the trajectory, 1-100; 0 = use the stored speed".
   - v51+/v14 comments say Start % affects only the approach move (U:6088-6090, R:1680-1682).
   - The original runner header says the run speed also sets the persistent global speed (R:32-33). If so, `rm_set_plan_speed(overall)` at R:1857 is immediately overridden by `rm_set_program_id_run(slot, 30)` at R:1873. Global-point steps later in the same workflow would then run under the last transit speed, because the overall speed is re-applied only before Programs.
   - `rm_program_run_state_t` includes `plan_speed` (SDK `rm_ctypes_wrap.py`, "global planning speed ratio"). The read-back that R:33 says is impossible looks available, but is not used.
4. **Runner version is not enforced.** UI v51+ writes `"speed": 100` for the legacy field. A v13 runner reads `speed` as the transit speed (v13 `normalize_workflow_step`), so every Page 02 approach move would run at 100 %. The UI still lists v13 as a fallback runner (U:6892) and gives no mismatch warning.
5. **No cross-arm synchronization inside TOGETHER blocks.** This relies on offline time-matching, which the pole timing above already breaks by a couple of seconds. The approach moves (the controller's joint move to the next program's first point) and the bridges are not collision-checked; the V6 README says so for its bridges.
6. **Error codes do not abort a running program.** They are sampled every 0.5 s and judged only at program end. Collision handling is left to the controller.
7. **The 300 s per-program timeout is fixed** and cannot be configured from the UI.
8. **The program list is read once per process.** Uploads made during a workflow are not detected (low risk).

### 2(b) Controller run-state monitoring / "detects still running or completed" (§3, §16)

**Verdict:** done entirely in the runner by polling `rm_get_program_run_state()`.
- **Detection rule.** A program counts as started when `run_state` 1 (running) or 2 (paused) is first seen. It has ended when `run_state` goes back to 0 after that.
- **Error codes.** Arm error codes are polled separately via `rm_get_current_arm_state()`.
- **What the UI sees.** Only the runner process's exit code, which it uses to drive the handoff.

**SDK facts** (from `rm_ctypes_wrap.rm_program_run_state_t`; `rm_robot_interface.rm_set_program_id_run`):
- **`run_state` values:** 0 = not started/idle, 1 = running, 2 = paused.
- **Other fields:** `id` (running program), `plan_num` (line being executed), `plan_speed` (global speed ratio), loop info.
- **Blocking mode.** `rm_set_program_id_run(..., timeout=1)` can return **-4 = "run state stopped but no run-success received - stopped externally?"**. The runner uses non-blocking mode (`timeout=0`), so it loses this signal.

**Runner logic and poll rates:**

| Path | Poll | Code |
|---|---|---|
| Page 02 program (`run_workflow_program_fast`) | run state every **50 ms**; error codes every **0.5 s** | R:1886-1938 |
| Page 01 single run (`run`) | run state every **100 ms**; error codes every 0.5 s | R:226-252 |
| Page 01 Multi Run (`run_sequence`) | every **100 ms**; error codes every 0.5 s | R:637-676 |

**How outcomes are distinguished:**

| Outcome | How the runner decides | Exit code | UI live label |
|---|---|---|---|
| Completed | Saw 1/2, then 0 (R:1897-1907); no non-4105 code during the run or after the quick settle (R:1958-1967) | 0, if every step OK | `COMPLETE ✓` |
| Did not start | `rm_set_program_id_run` returns non-zero (R:1874-1879), or no 1/2 seen within 5 s (R:1909-1913) | 1 | `FAILED rc=1` |
| Failed (error code) | Blocking code seen during the run or present at the end | 1 | `FAILED rc=1` |
| Timeout | Still running after `--timeout` (300 s), then slow stop (R:1915-1923) | 1 | `FAILED rc=1` |
| Stopped from this UI | SIGINT → KeyboardInterrupt → `rm_set_arm_slow_stop` (R:2248-2259) | 1 | `STOP REQUESTED` (transient), then `FAILED rc=1` |
| Paused at pendant | `run_state` 2 counts as "running" | 1, after 300 s | `RUNNING`, then `FAILED` |
| **Stopped at pendant / other client, no error code** | `run_state` 0 = "ended" | **0** | **`COMPLETE ✓`, and the handoff proceeds** |
| Runner exception / UI exception | Python traceback / `last_command_rc=-999` (U:6005) | 1 / -999 | `FAILED rc=...` |

Only the runner's text distinguishes these cases ("did not start", "still running after", "STOP REQUESTED", codes). The UI shows rc only, and its telemetry is cleared on the next stage.

**How the UI consumes it for handoff:**
1. **Exit code capture.** A worker thread streams stdout. At EOF it runs `rc = process.wait()` and stores `last_command_rc = rc` (U:5968-5972) before any Tk callback.
2. **Monitors.** Tk `after()` loops poll `has_active_command()`, i.e. `process.poll() is None` (U:3184-3187), every **120 ms**:
   - parallel U:10245-10321
   - serial-first U:10376-10453
   - serial-second U:10504-10568
   - tail U:10668-10712
3. **Success test.** `_page2_command_succeeded` means `rc == 0` (U:10715-10717).
4. **Serial handoff.**
   - If rc == 0: the label becomes `WAITING // CONFIRMATION REQUIRED` or `AUTO HANDOFF`, an optional modal popup appears (U:10433-10451), then a **new runner process** starts for the second arm (U:10455-10502).
   - If rc != 0: `FAILED rc=..`, the other arm shows `NOT STARTED`, and the workflow finishes (U:10403-10416).
5. **Next stage.** Scheduled after 20, 40 or 100 ms. `_start_next_page2_rule_stage` waits until both arms are idle, retrying every 100 ms (U:10117-10127).
6. **Handoff latency (not measured).** Up to 120 ms of UI polling, plus the operator if confirmation is on, plus process start, SDK import, connect, list read and pre-checks.

**Edge cases and weaknesses:**
1. **External stop looks like completion.** No check of `id`, `plan_num` (last line reached?) or the blocking-mode -4 code.
2. **Pause counts as running.** It then fails only at the 300 s timeout.
3. **Very short programs** (under one poll interval: 50 ms Page 02, 100 ms Page 01) would be reported as "did not start". The shortest production programs are about 1.0 s, so this is fine today.
4. **Transient errors are missed or ignored.** Transient `ret != 0` reads are ignored. Error codes that clear within 0.5 s can be missed. Codes never abort the program early.
5. **Program identity is not verified** (`state['id']` is unused).
6. **Stage-start race, not fail-safe.**
   - `self.process` is assigned in the worker thread only after `Popen` (synced launches first wait on the barrier, U:5693).
   - If a monitor tick (120 ms) arrives first, `has_active_command()` is False and `last_command_rc` is None. The stage is treated as FAILED and the dual sequence is finished, while the runner then starts and moves the arm.
   - The Page 01 SLOW STOP for that arm still works. The probability is low.
7. **Exit race, fail-safe.** `poll()` on the Tk thread can see the exit before the worker thread stores the rc. The result is a spurious `FAILED rc=None` that stops the workflow.
8. **One arm's failure does not stop the other arm** in a parallel stage (U:10273-10279).
9. **STOP BOTH between stages reports success.** It clears the plan (U:11182). The already-scheduled `_start_next_page2_rule_stage` then reports "PROGRAM RULES COMPLETE" and "SEQUENCE COMPLETE ✓" (U:10101-10115). This is misleading, though no motion results.
10. **Page 01 single run and 4105.** The start pre-check rejects a lingering 4105 warning (R:198-200), while Page 02 tolerates it.

---

## 3. Features in v51 / v52 / v53 / runner v14 that the document does not describe

### 3.1 New in v51 (vs v48; v49/v50 not available)
- **Per-Program "Start %" (default 30)**, the transit/approach speed. Help text: "Stored trajectory timing is unchanged". Code: U:6088-6093, 6176, 6510-6547; table text "Start N% // Stored timing // RULE" U:6670-6675.
- **JSON back-compatibility.** Old Program rows without `transit_speed` get 30 on load (U:6836-6851). The legacy `"speed": 100` is always written (U:6540-6543).
- **Version constants, version banner, LIVE ARM STATUS row** (U:45-46, 7391-7454, 9155-9178). The document describes the banner and live row (§15, §22), but v48 lacks them, so they arrived in v49-v51.
- **Runner preference list starts with v14** (U:6891).
- **Page 01 hint text** "RUN BOTH follows TOGETHER / RUN FIRST / WAIT OTHER rules ... Start % ..." (U:7558-7567). It sits on Page 01, but those rules apply only to Page 02's RUN BOTH.
- **Page 02 kicker** "SAFE START + LIVE STATUS" (U:9043).

### 3.2 New in v52 ("timeline rules")
- **TOGETHER timeline blocks.** Consecutive TOGETHER rows become one block per arm; Program IDs and row counts may differ (`combine` U:9733-9749; v52 docstring example "LEFT 71 → 73 → 76 / RIGHT 71 → 74 → 75 → 76").
- **Persistent connection across consecutive TOGETHER programs.** As a side effect, they share one runner process per arm.
- **New validation errors:**
  - TOGETHER section on only one arm (U:9930-9933)
  - unmatched serial section (U:9952-9959)
  - serial pair must use the same Program ID at the same position (U:9966-9972)
  - invalid rule pair (U:9990-9997)
- **RUN BOTH confirmation** shows block and serial counts (U:11036-11070). Button label "RUN BOTH // TIMELINE RULES" (renamed in v53).

### 3.3 New in v53 ("safe lid checkpoint")
- **SAFE TIMELINE manual checkpoint.** A **positive WAIT before a TOGETHER Program on both arms** becomes a hard cross-arm checkpoint (docstring U:9670-9688; `positive_wait_index` U:9717-9729; `split_wait_chunk` U:9751-9790; planner U:9810-9915). The run order is:
  1. Finish all earlier timeline programs on both arms.
  2. **LID-PREP-n**: run the steps up to and including the WAIT (e.g. pole 150 + wait 2 s) on both arms.
  3. **LID-CHECK-n**: a modal OK/Cancel "MANUAL LID COVER CHECKPOINT" with a lid / hands / path checklist (U:9886-9901; handler U:10141-10188).
  4. The post-checkpoint TIMELINE.
- **Checkpoint validation:**
  - a WAIT on only one arm is a "checkpoint mismatch" (U:9827-9833)
  - ambiguous pre-checkpoint alignment (U:9839-9847)
  - the post-checkpoint Program must be TOGETHER on both arms (U:9867-9871)
  - exactly one Program after the WAIT in its chunk (U:9772-9778)
- **New live states:** PAUSED // MANUAL LID CHECK, READY // LID CONFIRMED, IDLE // CHECKPOINT CANCELLED; status texts "SAFE TIMELINE // ..." (U:10152-10183).
- **Button and confirmation text:** button "▶▶ RUN BOTH // SAFE TIMELINE" (U:9089); the confirmation shows "Manual safety checkpoints: N" (U:11044-11060).
- **Behaviour change.** In RUN BOTH, a positive WAIT in a TOGETHER region now pauses for the operator. A positive WAIT on one arm only blocks the run. Single-arm runs and serial sections keep plain waits. The checkpoint message is hard-coded for the toilet-lid use case.

### 3.4 New in runner v14 (vs v13; the only change)
- **`transit_speed`.** The Program step field `transit_speed` (alias `start_speed`, default 30) is what `rm_set_program_id_run` receives (R:1680-1699, R:1873). The legacy `speed` is validated (default 100) but unused (R:1689-1692).
- **Log text:** "START/TRANSIT x% // stored trajectory timing // overall y%" (R:1866-1870) and the "WORKFLOW START MOTION" banner (R:2005-2009).

### 3.5 Older features still present in v53/v14 that the document also omits
- **Page 01 run/upload extras:**
  - UPLOAD + RUN (upload, verify, run) U:2283/4294; RUN ONLY (current upload slot) U:2291/4694.
  - Program lookup by exact or unique partial name (U:4776-4842).
  - `[DRAG TEACH]` tagging of `DT_` programs, with a count (U:4737-4769).
- **File browser** (U:74-757): search, sort by name/size/date/time, Back/Up/Home/Downloads, history, SELECT ALL, SELECT FOLDER FILES. **Batch dialog** with a Program ID per file, starting at the current slot (U:760-967).
- **Multi Run table** (unreachable because of the bug): per-row Hand/Pole/Wait cell editing, quick add by IDs/names, POLE → HAND → WAIT → TRAJECTORY order, renamed-program detection on load (U:970-1919, 4948-4997).
- **Global Points extras:**
  - search filter (U:2436)
  - editable sequence cells (U:3521-3668)
  - MoveL switches to the point's frames and restores them (R:1430-1465, 1500-1559)
  - all points validated before motion (R:1619-1626)
- **Dependency checks:** `hand.py` / `pole.py` presence is checked before Hand/Pole commands (U:4148-4155, 4182-4189, 5139-5155).
- **Page 02 extras:**
  - single-arm RUN/STOP per side (U:6450-6466, 9324-9424)
  - ↻ PROGRAMS / ↻ GLOBAL POINTS for both arms (U:9062-9074, 9216-9280)
  - GRIP/OPEN quick-fill in the builder (U:6360-6378)
  - RUN BOTH requires **both** sequences (U:10999-11008) and refuses while other dual operations run (U:11010-11022)
- **Window and setup:**
  - close-window guard while a command is active (U:11509-11530)
  - runner/SDK path pickers (U:7620-7680)
  - env `ALIX_RM_SDK` (U:48-51) and `RM_LEFT_IP` / `RM_RIGHT_IP` (R:47-48)
- **Runner safety and limits:**
  - refuses an empty slot, an arm error, or an already-running program (R:189-204, R:2022-2064)
  - upload verified by stored size (R:165-175)
  - list paging of 10 because of a controller JSON-buffer bug (R:100-102)
  - SDK timeout 1.5 s (R:2614); 5 s start window; 300 s timeout
  - global speed **restored to 50 %** after every run/sequence/workflow (R:259-262, 745-751, 2262-2277)
- **Runner CLI features the UI never uses:**
  - `--sequence ID...`
  - `--manual-hand PCT`
  - `--drag-store`, `--drag-run-file`, `--drag-slot`, `--drag-name` (DT_ naming plus controller rename, R:992-1150)
  - `--run-speed`, `--restore-speed`, `--timeout`, `--speed`, `--ip`, `--port`
- **Dead code in v53** (no callers):
  - legacy ONE-ARM serial mode U:9426-9617
  - alternating-program mode U:10733-10985
  - `_start_page2_program_rules` U:10029-10053
  - `_monitor_page2_dual_sequence` U:11094-11143
  - `_page2_command_failed` U:10719

---

## 4. Statements in the document that are inaccurate for v53/v14

1. **Wrong version (§22).** "Current version: UI v50 / Runner v13" is v53 / v14. The runner "version" is only parsed from the filename (U:7391-7404) and `EXPECTED_RUNNER_VERSION` is not enforced.
2. **TOGETHER semantics changed (§11).** "TOGETHER - Matching LEFT and RIGHT Program IDs run together" and "programs are paired using their matching Program ID" no longer hold:
   - Since v52, TOGETHER rows form independent per-arm timeline blocks with no per-program pairing or sync (U:9733-9749, 9917-9949).
   - Only RUN FIRST/WAIT OTHER must match, and they are paired by position (U:9952-9972).
3. **The §12 example is now a timeline (§12).** It works, but:
   - "1-7 together" is a timeline, not seven synchronized pairs.
   - Any positive WAIT in that region (on both arms) now triggers a manual lid checkpoint; a WAIT on one arm only blocks the run.
   - The document's v50 rule would reject the real production workflow.
4. **The persistent connection is per stage, not per workflow (§17).** "Keeps a persistent controller connection during Page 02 workflows" is true only per runner process.
   - RUN BOTH reconnects at every stage: 12 connections for the V6 workflow (§2a).
   - The program list is re-read per stage.
   - The speed is restored to 50 % and re-set per stage.
5. **Undocumented speed reset (§18).** The section omits that every runner process ends by setting the global speed to 50 %. It also omits that the per-Program "speed" is no longer a playback/speed control but a transit "Start %" (v51+).
6. **"Program speed control (global points speed)" is unclear and outdated (§3).**
   - Page 01 has no speed control at all (fixed 20 % transit).
   - Page 02's per-Program control is approach-only "Start %" (30 %).
   - Global Points have a separate speed.
7. **Live status shows stage labels and loses stop states (§15).** For TOGETHER sections, "shows current Program ID" shows the stage label (`TIMELINE-n` / `LID-PREP-n`). `STOP REQUESTED` is overwritten within about 120 ms. Several newer states are missing from the list.
8. **"Stopped" excludes external stops (§13 / §19).** "If first program fails or is stopped, waiting arm is not started" ignores stops made outside the UI with no error code: they read as completion (R:1905-1907).
9. **Failures are handled per stage only (§19).** "Error states stop the coordinated workflow rather than continuing blindly" holds only at stage boundaries. In a TOGETHER stage the other arm keeps running its whole block after one arm fails (U:10273-10279).
10. **Residual races (§16).** "Prevents timing/race-condition problems" fixed the status-text race, but two narrow races remain (§2b items 6-7).
11. **Multi Run is broken (§2).** "Multi Run stored programs" and "Save and load Multi Run sequences" are not usable:
    - MULTI RUN raises TypeError before starting (U:5000/5159).
    - Page 01 LOAD SEQUENCE always errors (U:4853/4932).
    - Only save/load inside the dialog work.
12. **Replacement confirmation flaws (§2).** The existing-slot dialog shows literal "\n" text (U:4467-4485), and single uploads do not check the slot's current contents (U:4276-4282 + `--yes`).
13. **Telemetry gaps (§20).**
    - 4105 is not warning-only for Page 01 single/dual runs (start refused, R:198-200).
    - The telemetry log is cleared at every command/stage.
    - Telemetry is not visible on Page 02.
14. **Runtime gaps (§21).** "Individual arm runtime" does not tick during Page 02 runs, and single-arm Page 02 runs have no runtime display.
15. **Old JSON speeds are not preserved (§24).** For files from v50 or earlier, the Program `speed` values are not carried forward: rows get `transit_speed=30` (U:6836-6851).
16. **Handoff popup formatting (§14).** The popup text contains literal "\n\n" (U:10436-10437). Cosmetic.

**Other defects found while verifying (not documentation claims, but relevant for planning):**
- **No interlock during RUN BOTH.** `ArmControlPanel` does not know about `page2_dual_active`, so Page 01 commands on an idle arm (e.g. the WAIT OTHER arm, or between stages) are not blocked. Hand/Pole APPLY on that arm moves hardware with no popup.
- **Literal `\\n` strings elsewhere:** Select Folder Files dialogs (U:608-633), batch telemetry lines (U:4525-4680), and the dead alternate-mode popup (U:10884-10885).
- **Page 02 JSON load has no schema validation.** A row missing keys can raise inside `_row` during table refresh (U:6662-6697).

---

## 5. Notes for a web re-implementation for non-technical users

### 5.1 Which of the 24 sections an operator actually needs

**Operator-facing, rebuilt as guided flows:**

| § | Operator needs | Engineer-only parts |
|---|---|---|
| 1 | Two arm cards with "connected / mode" status | IPs, ports, SDK path (settings page) |
| 5 | Drag & Teach per region: guided "clean region X for N s" prompt, start/stop recording, auto-save named by region (matches requirement input #1/#7) | Playback-to-origin and file paths |
| 7-8 | Hand GRIP/OPEN presets and per-region pole presets | Six-channel sliders and free mm entry |
| 9 | Read-only SIM/REAL badge | Switching to REAL (gated, with the existing confirmation) |
| 13-14 | Handoff and lid checkpoint as plain-language prompts that appear only when a human action is needed ("Open the lid, step back, press Continue") | |
| 15 | Per-arm card: step k of n / region name / waiting reason / time left | Raw rc values |
| 19 | Always-visible STOP ALL plus per-arm stop; Pause/Continue could be added (SDK has `rm_set_arm_pause` / `rm_set_arm_continue`) | |
| 21 | Elapsed / remaining time | |
| 24 | Pick a named "cleaning recipe" and press Start | JSON editing |

**Engineer / commissioning only:**
- §2: upload, slots, multi-upload, list refresh, Multi Run. Multi Run is superseded by workflows and broken, so drop it.
- §3: speeds.
- §4: dual stored run test.
- §6: Global Points. An operator might get one "Park / Home" button.
- §10-12: recipe authoring and dual rules.
- §16-18: backend internals, with engineer-locked parameters.
- §20: raw telemetry. Operators get a translated event feed (e.g. 4109 = "collision detected").
- §22: version information, in an About panel.
- §23: theme. Re-design per audience.

### 5.2 Architecture lessons from this code
1. **Keep the controller connections in the backend.**
   - Replace the process-per-command runner with a long-lived backend service that holds **one SDK connection per arm**. That gives the persistent connection the document claims, with no per-stage reconnect or list re-read.
   - Cache the program list and invalidate it on upload.
2. **Move dual-arm orchestration server-side.** Today it lives in Tk `after()` loops in the UI.
   - Use an explicit state machine whose outcome per step is complete / failed / operator-stopped / timeout / paused / external-stop. Do not use process exit codes.
   - Decide what happens when the browser disconnects.
   - Add a global interlock so an arm reserved by a workflow cannot be commanded from another screen.
3. **Make the checkpoint an explicit step type** with a custom message, instead of inferring it from "positive WAIT before TOGETHER".
   - Version the JSON (v2) with schema validation (`side`, IDs, transit speed).
   - Migrate v1 files (`speed` → `transit_speed`).
4. **Use the richer SDK state.**
   - `rm_program_run_state_t.id`, `plan_num` (progress, and detecting early stops) and `plan_speed` (read back the speed).
   - Optionally use blocking-mode `-4` in a worker thread.
   - Use realtime push (`rm_set_realtime_push` / `rm_realtime_arm_state_call_back`) instead of 50 ms / 0.5 s polling.
   - Make the per-program timeout configurable from known runtimes (`FINAL_FILE_RUNTIME_REPORT.csv`).
5. **Settle speed semantics on hardware before exposing any speed control.**
   - What does `rm_set_program_id_run` speed actually change, and does it override `rm_set_plan_speed`?
   - Define the restore policy for crash/kill too (e.g. a watchdog that sets a safe global speed).
6. **Decide on cross-arm timing.** If time-matched TOGETHER blocks are a safety assumption, add per-program sync points or monitor relative progress (`plan_num`). Also align the pole wait model (`pole.py`) with the offline 1.38 s assumption.
7. **Logging.** Persist per-run structured logs instead of clearing per stage; translate error codes for operators; keep the raw console for engineers.
8. **Do not port the inherited bugs.** These are the Multi Run / Load Sequence TypeErrors, the literal `\n` dialogs, the races, the misleading "COMPLETE" after a stop between stages, and the misplaced Page 01 hint.
