# C4: Robot execution and upload (R4 Real-arm testing, R13 Faster upload): verified final

Cluster `C4_robot_execution_upload`. This is the adversarial verification of the analyst draft. Every claim was opened at its cited `path:line` and confirmed, corrected or refuted. I also searched for things that exist but were missed. The work was read-only: no robot, no network, no test suites.

- "(derived)" marks a number I computed from the cited raw data.
- "(inferred)" marks a conclusion read from code without running it.
- "NEW" marks a finding or item that the draft did not have.

| Alias | Absolute path |
|---|---|
| `UC` | `/home/newtonjeri/alix_studio_lite_ws/upload_channel` |
| `RUN` | `UC/rm_upload_fast_v14_safe_transit.py` (runner v14, 2,702 lines) |
| `UI` | `UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py` (UI v53, 11,581 lines) |
| `ALIX` | `/home/newtonjeri/alix_ws/src/alix` |
| `DP` | `ALIX/core/python/alix_dispatch/src/alix_dispatch` |
| `TG` | `ALIX/core/python/alix_taskgen/src/alix_taskgen` (ALIX Studio: `server.py`, `web_editor/editor.js`) |
| `SDK` | `ALIX/external/butterfli-realman-alix/Python/Robotic_Arm` |
| `V6` | `/home/newtonjeri/alix_studio_lite_ws/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES` |
| `HTML` | `/home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html` |

---

## 0. Verification result in one screen

Almost every cited line in the draft checks out. The upload facts, the runner behaviour, the ALIX library equivalents and the ALIX Studio "no execution channel" verdict are all correct. The material corrections are these:

1. **Completion detection.** "Final pose within 2° of the last point" is not enough on its own.
   - ALIX recorded a silent cut that raised only warning 4105 and stopped at point 2,319 of 2,971 (`ALIX/plan/FINDINGS.md:9563-9572`). The runner would print `RUN COMPLETE` for it.
   - One V6 file passes within 1.07° of its own end before its last 10 % (derived).
   - ALIX already has two stronger witnesses: the controller's `program_run_finish` event, and the played duration measured against the player law. Lite should use them.
2. **Transit (the move to the first point).** This move can be **checked**, not just announced. ALIX measured `movej` to be joint-linear to 0.004° (`ALIX/docs/PHASE_PLAN.md:186`), so Lite can sweep the straight joint-space line in its collision world before start.
3. **Recovery after a stop.** This was missing. A stopped arm may sit on the commode. "Run again" begins with an unchecked transit out of that pose. ALIX hit exactly this (H40) and built a read-first, park-one-arm-at-a-time recovery (`ALIX/tools/dual_arm_flow.py:6180-6188`).
4. **STOP.** Four corrections:
   - Send the stop signal once. A second interrupt can skip the speed restore in `run()`.
   - STOP ALL should also stop the pole and hold the hand.
   - A small stdin control channel in Lite's runner copy works on Linux and Windows. It also stops the arm if the Lite backend dies.
   - A failed run start leaves no stop behind.
5. **Dual start.** v53 synchronises only the process launch, so the skew in motion start is unbounded. The runner's own pre-start prompt can be used as a READY/GO handshake.
6. **Commissioning.** ALIX already has the stop-policy probe (`DP/track.py:3273`) and a zero-exposure dry track (`ALIX/tools/cleaning_tracks.py:22`). Lite should reuse them, not write a new script.
7. **Robot dry run.** Pole and hand steps are **always** skipped; the draft left this as an open question, and it is now decided. The dry run is recommended by default for the first real run of new content. Lite records the mode it finds and restores it afterwards.
8. **ALIX Studio gate chips and stale-on-edit.** Only the pattern and CSS are reusable. The logic is wired to the geometric-config gates (coverage, reach, checks and surface), which belong to X1.
9. **R13 slot ranges.** ALIX also uses slot 60 (dry) and numbers 101 and up (CANFD-only). Slots below 40 are shared with pendant programs, and slot 1 holds one. The stored-name model has been measured on hardware several times, not "once".
10. **R13 and R12.** A strict joint-rate gate at 5 ms would block 10 of the 15 current V6 production files (derived). Fast files also overshoot their commanded positions by 4–20°.
11. **Effort.** R4 rises from L to **XL** (about 18 person-days); a single-arm MVP is L. R13 stays M.

The full list is in the change log (§6).

---

## 1. The upload-channel programs at a glance (high level, as the user asked)

| File | What it does | Current? |
|---|---|---|
| **UI v53** `rm_trajectory_ui_v53_safe_lid_checkpoint.py` | Tkinter desktop app, "RealMan // Dual-Arm Operations Console" (`UI:6880`). See the breakdown below this table. | **Current UI** |
| **Runner v14** `rm_upload_fast_v14_safe_transit.py` | Command-line back end. One process opens one TCP connection to one controller and closes it at exit (`RUN:2606-2614, 2697-2698`). See the verb list below this table. | **Current runner.** The `_v52support` and `_v53support` copies are byte-identical (md5 `a0b285e6…`). |
| `hand.py` | Inspire RH56 hand over the arm's end-port **Modbus RTU**. Six channels, 0–1000, 1000 = open (`UC/hand.py:1-17`). | helper |
| `pole.py` | Pole (lift) through the **SDK lift API**, not Modbus: `rm_set_lift_height`, `rm_get_lift_state`, stop = `rm_set_lift_speed(0)`. It uses a measured travel model (`UC/pole.py:1-58`). | helper |
| `usage.py` | Small CLI with `--ip`/`--port`. Commands: pole read/move/stop, hand open/close/pct/set/freeze, `prestroke`, and `stop`, which slow-stops the arm, stops the pole and freezes the hand (`UC/usage.py:1-25, 141-148`). | helper |
| v48, v51, v52 UIs; v13 runner | Older versions. v48 is three versions behind and everything in it is also in v53. v48 and v51 reject the V6 production workflow; v53 runs it, adding a manual lid checkpoint. | superseded |

UI v53 in more detail:

- **Page 01** has a LEFT and a RIGHT arm panel. Each has the tabs RUN, POINTS, DRAG TEACH, HAND / POLE and TELEMETRY (`UI:2162-2166`). The TELEMETRY tab is only the runner's text log (`UI:3128-3180`).
- **Page 02** is the dual sequence builder. Its steps are program, pole, hand, wait and global point. Its arm rules are TOGETHER, RUN FIRST and WAIT OTHER, plus a manual lid checkpoint.
- **It never talks to the robot itself.** Every button starts one runner process: `python -u RUN --sdk … --side … --yes` (`UI:4255-4262`). The UI reads the runner's printed lines and stops it with SIGINT (`UI:6026-6070`). For dual starts it waits on a `threading.Barrier` just before launching both processes (`UI:5654-5709`).

Runner v14 verbs:

- upload a `.txt` and verify it (`RUN:147-175`);
- run a slot and watch it to the end (`RUN:189-269`);
- list the slots;
- multi-run (`RUN:393-753`);
- Page-02 workflows with fast program switching (`RUN:1668-2277`);
- drag-teach record, save and playback;
- global waypoints;
- manual hand and pole;
- get and set the controller's SIM/REAL mode with read-back (`RUN:1213-1262`).

Its upload and run core is ALIX `tools/rm_upload.py`; by diff they are identical apart from docstring lines.

**Plan-blocking issues (one line each, no bug audit):**
1. The UI never passes `--ip`/`--port`, and its IPs are fixed labels (`UI:7506, 7588, 7600`). Lite must pass the configured IP itself (`RUN:2283-2286, 2347-2349`).
2. Multi Run and Page-01 Load Sequence raise TypeError: the methods have no `self` and no `@staticmethod` (`UI:4853, 4876, 4885, 5000`). Do not port them; use the workflow path.
3. The runner version is shown but not enforced (`UI:46, 7394`). If v14 is missing, the UI silently falls back to v13, which runs Page-02 transits at 100 % (`UI:6889-6909, 6543`).
4. STOP is SIGINT only. On Windows, `CTRL_BREAK_EVENT` (`UI:6059`) most likely kills the runner with no slow stop and no speed restore (inferred; the runner has no signal handler).
5. The default SDK path is another machine's (`UI:48-51`), and the runner's repository fallback does not exist here (`RUN:62`). Lite must always pass `--sdk`.
6. UPLOAD passes `--yes` after only a generic "will be replaced" warning (`UI:4275-4290`). It therefore skips the runner's prompt showing what the slot holds (`RUN:148-157`). Lite needs its own check for programs it did not write.

---

## 2. Shared vocabulary

| Term | Meaning |
|---|---|
| **Simulation** | Lite's own 3D preview with collision checks (R3/R5). No robot. |
| **Robot dry run** ("Practice on robot") | The controller's own SIMULATION run mode (mode 0). The controller plays the program internally and reports joint states; **the arm does not move**. It does not simulate the pole or hand and has **no commode model** (`ALIX/docs/PHASE_PLAN.md:115-116, 139`). Lite skips pole and hand steps in this mode. |
| **Real robot** | Controller REAL mode (mode 1). The arms move. |
| **Transit** | The controller's own joint move (movej) from wherever the arm is to the program's first point. The controller does **not** collision-check it. Its path is a straight line in joint space (`ALIX/docs/PHASE_PLAN.md:186`), so Lite can check it. |
| **Global speed** | The run speed becomes a controller-wide ratio that survives power cycles, so it must be restored after every run (`DP/track.py:19-24`; `ALIX/plan/FINDINGS.md:5866-5881`). |
| **Send to robot** (upload) | Store a `.txt` in one of the controller's program slots 1–100. Nothing moves. The trajectory HTML's "Upload" step means opening a file (`HTML:335, 345`). |
| **Recovery** | Bringing an arm that stopped mid-path back to a safe pose before anything else moves. |

---

## 3. R4: Real-arm testing

### 3.1 Original text
"Real-Arm Testing: The validated simulation trajectory should be executed on the actual robotic arms. If collisions, missed spots, or other issues occur, the user should be able to return to trajectory editing. - should be able to connect to the real robot, have the IP address entry options, also toggle between real and sim"

### 3.2 Revised requirement
Lite shall let an operator run on the real arms a trajectory whose exact content has passed simulation. The operator shall be able to stop it at any moment, bring the arm back to a safe pose, record what happened, and return to editing with the run's context.

- **(a) Connection.**
  - Each arm has an editable IP address and port. The defaults are LEFT 192.168.1.10, RIGHT 192.168.1.103 and port 8080. Only an engineer can change them.
  - **Test connection** reports Connected, Unreachable or Error within 5 s, together with the controller's run mode as read back from the controller.
  - Settings persist.
  - Every robot command uses the configured IP and port, never environment defaults.
- **(b) Run target.** A clearly labelled selector offers **Simulation** (Lite 3D, no robot), **Robot dry run** (controller SIMULATION mode; the arms stay still and pole and hand steps are always skipped) and **Real robot**.
  - Lite always opens in Simulation.
  - The controller mode is read back after every change and again inside the run process immediately before start. A mismatch refuses the run.
  - Lite records the mode it found and restores it, verified by read-back, after a dry run.
- **(c) Gate.** A real run is allowed only when all of these hold:
  - the exact content has a passed Lite-simulation record (a failed record cannot be overridden; a missing one only by an engineer with a written reason);
  - the content passes the motion checks (R12);
  - the slot holds that content, checked inside the run process against Lite's registry and the controller's stored size and name;
  - the arm is idle and reports no blocking error;
  - the transit check passes (d);
  - Lite has applied the region's pole/hand preset itself and read it back (R7);
  - the operator has ticked the human-only checklist: fixture state, area clear, pad off the surface, E-stop within reach.

  For the first real run of new content, a robot dry run comes first unless an engineer has switched this off.
- **(d) Transit check.**
  - Before start, with the controller in REAL mode, Lite reads the arm's joints and sweeps the straight joint-space line to the first point in its collision world (commode in the region's state, walls, other arm).
  - Lite shows the size of the move and runs it at reduced speed (default 20 %, an engineer setting).
  - Lite refuses the run if the line collides or exceeds an engineer-set size limit.
- **(e) Run, stop, monitor.**
  - There is a large, always-visible STOP per arm and a STOP ALL. STOP ALL slow-stops the arm, stops the pole and holds the hand in one click, and the stop is sent once.
  - Live status shows the phase, elapsed versus expected time and translated controller codes, with a live 3D mirror of the arms.
  - The global speed is restored after every run, and its result is recorded.
  - If the operator screen is lost for longer than a set time (default 3 s), or the Lite backend dies, the arm is slow-stopped.
- **(f) Completion.** Lite shows **Finished** only when all of these hold:
  - no blocking code was reported;
  - the final pose, read after the arm settles, is within 2° of the last point;
  - playback lasted at least 85 % of the expected time;
  - the controller's program-finish event was received (once commissioning has confirmed this event on these controllers).

  Otherwise Lite reports "Stopped at about X %" or "Not confirmed: please check".
- **(g) Recovery.** After any run that did not finish, before anything else moves, Lite reads both arms' joints and offers a checked way back to a safe pose, one arm at a time. The options, in order:
  1. a joint move to the arm's safe pose, if it is collision-free in simulation;
  2. otherwise, reverse out along the executed path;
  3. otherwise, hand guiding.

  "Run again" stays disabled until the arm is at a safe pose.
- **(h) Verdict and return.**
  - After each real run the operator records Looks good / Hit something / Missed a spot / Other, plus a note.
  - Lite stores a run record.
  - **Back to editing** opens the trajectory at the stop point or at a point the operator marks.
  - Any edit marks the simulation and real-test status as stale.
- **(i) Dual-arm runs.** Both arms start within a measured skew (target 0.2 s or less). A fault or STOP on one arm stops the other within 1 s.

### 3.3 Clarifications
- **"Validated simulation trajectory"** means the same content identity (sha256; see §5) passed two checks with no blocking findings:
  - Lite's 3D collision sweep against the commode, the walls and the other arm (R3/R5);
  - the motion gates: joint limits with a margin, at most 10° per line, and rate at or below rating at the 5 ms cycle (R12).
- **"Toggle between real and sim"** is read as the three-way run-target selector.
  - The control app's SIM button is the controller dry run, not a 3D simulation.
  - The dry run catches controller-side refusals before any metal moves. Examples: the controller's own collision model (4109 raised in SIMULATION by a controller configuration defect, `ALIX/docs/PHASE_PLAN.md:139, 144`), limits, and a program that does not play to the end.
  - It does **not** check commode clearance, real-arm overshoot, the pole or the hand.
- **"Missed spots"** cannot be detected automatically, because Lite has no contact data (`HTML:437`). The operator reports them and marks the place.
- **The run "speed"** only scales the transit and becomes the controller-wide ratio (`ALIX/plan/FINDINGS.md:5854-5881`). It does not slow the stored trajectory. A slower first run therefore needs a slower file, re-timed in R2.
- **The transit is predictable.** The program's transit is a move_J (`ALIX/plan/FINDINGS.md:5716-5720`), and the controller's movej was measured joint-linear to 0.004° (`ALIX/docs/PHASE_PLAN.md:186`, C11, in a SIM-mode capture). The joint-space straight line is therefore the path to check (inferred for the implicit transit; confirm at commissioning).
- **Engineer role.** An "engineer" role (or settings lock) is assumed for IP/port, slot range, transit speed and limit, dry-run policy and gate overrides. Operators see these values read-only.
- **Out of scope for R4** (covered elsewhere): the 3D simulation itself (R3), plot highlighting (R5), TOGETHER / RUN FIRST / lid-checkpoint scheduling (R10/INT-CTRL), pole/hand presets (R7), upload mechanics (R13).

### 3.4 Findings (verified; NEW = not in the draft)

| # | Source | Finding | Evidence |
|---|---|---|---|
| 1 | ALIX Studio | **No execution channel.** A test pins it ("no run/stop/pause route exists"; the run panel must say "arms not connected"). No `RoboticArm`, `rm_send_project` or `rm_set_program_id_run` exists in `alix_taskgen`; `alix_dispatch` appears only in comments. | `TG/../../test/test_studio_interaction_guards.py:2204-2216`; grep of `TG` (comments at `server.py:1802,1835`, `yaml_assembler.py:170`, `reference_frames.py:118`, `contract_motion.py:362,368`) |
| 2 | ALIX Studio | **Live-mirror path exists.** `do_sim_joints` accepts a joint map ("future: the real arms") and **replaces the whole map**, so both arms go in one body. **NEW:** `do_sim_pose` computes FK for **one arm per call** (`arm` key) and in `auto` **prefers the emulator**, so Lite must ask for `source: "external"`. The Simulate view polls at 2/5/10 Hz and shows a status dot. | `TG/server.py:5312-5393`; `TG/web_editor/editor.js:19218-19296`; `TG/web_editor/template.html:87-99` |
| 3 | ALIX Studio | **Feedback parts:** `showStudioModal`, `setStatus`/`showErrorToast`, `markDirty`/`invalidateValidation`, gate chips, and the dual-arm-collision timeline. **NEW:** `gateState()` computes coverage, reach (IK), checks and surface for geometric configs, and `markDirty` is wired to the config editor's state. That logic is X1-coupled; only the pattern and CSS are reusable. | `editor.js:13786-13832, 5431-5462, 5371-5429, 15760-15800, 15904-15950, 14478-14548` |
| 4 | Runner v14 | **`--ip`/`--port` accepted; `--ip` wins over `--side`.** Default IPs .10/.103 can be overridden by `RM_LEFT_IP`/`RM_RIGHT_IP`. Port 8080. No IP validation. | `RUN:46-49, 2283-2286, 2347-2349` |
| 5 | UI v53 | **No IP entry.** Labels are hard-coded. A flag census finds 0 uses of `--ip`, `--port`, `--run-speed`, `--restore-speed` and `--timeout` (32 × `--side`). | `UI:7506, 7588, 7600, 11314-11315, 4255-4262` |
| 6 | Runner v14 | One TCP connection per process: triple thread mode, SDK log level 3, `rm_set_timeout(1500)`. No reconnect, retry, heartbeat or connect timeout. | `RUN:2606-2614, 2697-2698` |
| 7 | Runner + UI | **SIM/REAL/CHECK switches the controller run mode (0/1)**, verified by read-back. REAL asks for confirmation; SIM does not. The badge reads `MODE // UNKNOWN` until CHECK is pressed. | `RUN:1213-1262`; `UI:2079-2101, 3785-3816, 1989` |
| 8 | ALIX docs | **Controller SIM mode:** the arm is executed virtually and is observable; it does not execute lift or hand; it has no commode model. Its own collision model raised 4109 in SIMULATION. **NEW:** the cause was a controller configuration defect (payload centroid) that also affected REAL. | `ALIX/docs/PHASE_PLAN.md:115-116, 139, 144`; `ALIX/docs/TEST_PROCEDURE.md:335-338` |
| 9 | Runner v14 | **`--run` sequence:** <br>• gates: slot listed, state readable, any error code (even 4105) refuses, not running; <br>• `rm_set_program_id_run(slot, 20, 0)`; <br>• polls every 0.1 s and reads codes every 0.5 s; <br>• 5 s start window; `--timeout` 300 s, then slow stop; <br>• `finally`: slow stop if stopped, settle, restore 50 %. <br>Complete = ended, not stopped, no non-4105 code. | `RUN:189-269, 2337-2342` |
| 10 | Runner v14 (NEW) | **The workflow path differs from `--run`.** Lite would use it for pole → hand → program. <br>• It continues on 4105. <br>• It defaults to overall speed 100 and transit 30. <br>• It polls at 50 ms. <br>• It stops only the arm, or the pole during a pole step. <br>• It does not enforce its quick-settle result before the next step. <br>Lite must pass explicit values and order steps pole → hand → program. | `RUN:1668-1700, 1807-1983, 1949-1980, 2034-2063, 2248-2277` |
| 11 | ALIX FINDINGS | **The run speed scales only the transit** (track 29.16 / 29.54 / 29.56 s at 20 / 20 / 100) and persists controller-wide: a movej ran about 6× slower after a speed-10 program. `run_state` 0 cannot tell completion from a limit stop. | `ALIX/plan/FINDINGS.md:5854-5881`; `DP/track.py:19-24` |
| 12 | ALIX FINDINGS (NEW) | **Silent cuts exist.** A class-B cut raised only 4105 and stopped at point 2,319 of 2,971. The runner treats 4105 as warning-only, so it would print `RUN COMPLETE`. Class-A soft-limit cuts raise 4116, which the runner does catch. | `ALIX/plan/FINDINGS.md:9563-9572, 9533-9536`; `RUN:263-266` |
| 13 | alix_dispatch (NEW) | **ALIX's completion witnesses:** <br>• the controller's `program_run_finish` event: 90 complete / 4 cut, perfectly separated; <br>• the blocking-start probe returning −4: 118 / 7; <br>• the played fraction against the player law, with a bar of 0.85. <br>Event capture (`ArmEventLog`) needs a process-wide SDK callback and cannot be exercised offline (the emulator never sends a finish event). | `DP/track.py:2383-2390, 1795-1830, 1573-1588, 2020-2036`; `ALIX/plan/FINDINGS.md:9475-9497, 10086-10091` |
| 14 | V6 files (NEW, derived) | **Pose-only completion can mislead.** `M3_OPNST02.txt` comes within 1.07° of its last point before its last 10 %, and `M3_OPNST01.txt` within 2.38°. | python over `V6/LEFT/*.txt` |
| 15 | UI + runner + ALIX | **Stop** is SIGINT to the process group (`CTRL_BREAK_EVENT` on Windows); KeyboardInterrupt → `rm_set_arm_slow_stop`. There is no signal handler, so on Windows the runner is probably killed (inferred). Whether slow stop ends a running stored **program** is untested on hardware (G4). The program keeps playing if the client dies. **NEW:** <br>• v53 disables STOP after one click "to avoid repeated stop signals"; <br>• in `run()` a second interrupt during `finally` could skip settle and the speed restore (inferred; `run_sequence` guards against this); <br>• `--run` stops only the arm, while `run_sequence` and `usage.py stop` also stop the pole and freeze the hand. | `UI:6026-6070, 6053-6055`; `RUN:252-262, 714-737`; `UC/usage.py:141-148`; `DP/emulated_program.py:50-62` |
| 16 | Runner v14 (NEW) | **"Did not start within 5 s" exits without any stop command.** A program that starts late would play unmonitored (inferred). | `RUN:238-240, 1909-1913` |
| 17 | UI v53 | **Dual runs:** a barrier precedes the two `Popen` calls. In a parallel stage one arm's failure does not stop the other. The v32 dual "MODE WARNING" was dropped in v35 (0 hits in v35–v53). **NEW:** start skew is unbounded, because each runner then loads the SDK, connects, reads the list and checks state before starting. The runner's typed start prompt is printed after all its gates, so it can serve as a READY/GO handshake (inferred). | `UI:5654-5709, 5693, 5750, 10255-10279`; `UC/py programs/rm_trajectory_ui_v32_dual_files_fixed.py:7100-7125`; `RUN:189-223, 215-218, 2104-2113` |
| 18 | Runner + ALIX (NEW) | **Transit.** <br>• The runner prints the transit size only when it uploaded the file in the same process; no runner verb prints the current joints. <br>• ALIX rule: announce the gap and proceed by default, with a strict mode that refuses (`require_at_start`, tolerance 2°). <br>• movej is joint-linear on the controller, and the program's transit is a move_J. | `RUN:2692, 207-214`; `DP/track.py:323-338, 407-410`; `ALIX/docs/PHASE_PLAN.md:186`; `ALIX/plan/FINDINGS.md:5716-5720` |
| 19 | ALIX tools (NEW) | **Recovery** (H40): "the only recovery from a cut used to be `run` again, which begins with two simultaneous collision-unchecked bridges out of a state nobody read". `cmd_recover` reads both arms first, stops, then parks one arm at a time. `recover_arm` stops every device, then parks. | `ALIX/tools/dual_arm_flow.py:6180-6188`; `DP/staging.py:921-960` |
| 20 | alix_dispatch | **Library equivalents:** <br>• `open_arm` (1500 ms acknowledgement timeout); <br>• `apply_run_mode` / `require_real`: read-back is the gate and an unreadable mode is refused; <br>• **NEW:** `restore_run_modes`; <br>• **NEW:** in SIM, skip pole and hand ("a SIM arm that dropped the pole for real would be worse than a phantom"); <br>• `gate_real`: a SIM pass licenses REAL only for the identical fingerprint; **NEW:** a *failed* SIM blocks REAL with no override; a *missing* one may be overridden with a reason; <br>• `stop_program` (verified ladder); <br>• `start_gap_deg`; <br>• `expected_replay_s` = 0.2322 s + 5.6737 ms × (N−1). | `DP/connect.py:37-66, 141-182`; `DP/runmode.py:70-90, 134, 205-268`; `DP/ladder.py:101-145`; `DP/track.py:3069-3140, 323-338, 1558-1570` |
| 21 | alix_dispatch (NEW) | **The G4 probe already exists.** `track.probe_stop_policy` asks "does an arm-level stop abort a running PROGRAM?", using a zero-exposure in-place dry track (`cleaning_tracks.py dry`). It has never been run on hardware. | `DP/track.py:3273-3300, 3060-3068`; `ALIX/tools/cleaning_tracks.py:22, 3729`; `ALIX/Resource/reports/2026-10-01_remaining/T4_verdicts/show_final.txt:7` |
| 22 | alix_dispatch | **Stop position:** `phase_end` = the commanded index nearest the arm's last position, with a ±3 % margin check. An ordered matcher avoids "racing ahead" on rasters. | `DP/track.py:2037-2075` |
| 23 | RealMan SDK | **Run state** carries `run_state`, `id`, `plan_num` and **`plan_speed` ("global planning speed ratio 1-100")**. ALIX treats the global ratio as having no getter; whether `plan_speed` reads it back when idle is untested (NEW open item). Pause, continue and emergency stop exist but are unused. A blocking run returns −4 when the finish message is missing. | `SDK/rm_ctypes_wrap.py:2437-2500`; `SDK/rm_robot_interface.py:2069-2129, 4857-4921, 7226`; `ALIX/plan/FINDINGS.md:5866-5875` |
| 24 | ALIX emulator | **In-process stand-in** that swaps itself in for `Robotic_Arm` in `sys.modules`. It answers the IPs in `RM_LEFT_IP`/`RM_RIGHT_IP` (default .10/.103; corrected from "only the two default IPs"). It has no drag-teach. Program verbs come only through `emulated_program.attach` in `open_arm`, and it never fires program-finish events. Use it for engineering tests only. | `ALIX/core/python/src/alix_emulator/__init__.py:1-30`; `…/rm_emulator.py:474-477, 1936-1950`; `DP/connect.py:175-181`; `DP/track.py:1823-1825` |
| 25 | Error codes (NEW) | 4105 speed warning, 4109 collision, 4114 self-collision, 4115 fence, and **4116 joint out of soft limit** (recover by pressing and holding the green button at the arm end). | `ALIX/docs/hardware-sdk/ERROR_CODES.md:46-57`; `RUN:56-57` |

### 3.5 Borrow verdict
**ALIX Studio: Partial. Borrow some, build the rest.**

None of the robot-facing core comes from the Studio: connect, mode, run, stop, recovery and verdict (`test_studio_interaction_guards.py:2204-2216`). Lite can borrow these front-end patterns:
1. **The Simulate live view and the external joint feed**, to mirror the real arms. Post both arms in one map, request `source: "external"` and ask for FK once per arm (`server.py:5312-5393`; `editor.js:19218-19296`).
2. **`showStudioModal`**, for the REAL-mode and REAL-run confirmation with gate chips (`editor.js:13786-13832`).
3. **`setStatus` and `showErrorToast`** (`editor.js:5431-5462`).
4. **The chip and stale-on-edit pattern and its CSS.** The logic must not be reused: `gateState()` computes geometric-config gates (X1, `editor.js:15760-15800`).
5. **The dual-arm-collision timeline (optional, R5)**, to show where a run stopped (`editor.js:14478-14548`).

| Other source | Verdict | What to use |
|---|---|---|
| Runner v14 | **Borrow with adaptation** (subprocess, Lite-owned copy pinned by sha256, opt-in additions only) | `--ip/--port`, mode get/set with read-back, run gates, watchdog, slow stop, speed restore, stdout tokens. Opt-in additions (default off, proven unchanged by diff): <br>• `--telemetry` and `FINAL q=`; <br>• `--require-mode`; <br>• `--expect-size/--expect-name`; <br>• `--track-ref` (transit gap); <br>• `--set-global-speed`; <br>• a stdin control channel (GO / STOP / stop on end-of-input); <br>• a finish-event print. |
| UI v53 | Reference only (behaviour) | REAL confirmation wording (`UI:3802-3810`), one-click stop without a dialog and disabled after the first click (`UI:6026-6055`), barrier launch, the dropped v32 dual REAL warning. Do not port the Tk UI, its dark theme, Multi Run / Load Sequence or completion decided from status text. |
| `alix_dispatch` (connect, runmode, ladder, track) | Borrow with adaptation (target library) | `open_arm`, `require_real`, `restore_run_modes`, the SIM pole/hand skip, the `gate_real` rules, `stop_program`, `start_gap_deg` with strict mode, `expected_replay_s`, `played_fraction`, `PLAYED_SHORT_FRAC`, `phase_end`, the `ArmEventLog` pattern. These are stdlib at import; numpy is imported lazily. |
| ALIX tools: `probe_stop_policy`, `cleaning_tracks dry`, `dual_arm_flow cmd_recover` (NEW) | Borrow (commissioning) / pattern (recovery) | The G4 probe on a zero-exposure in-place track. The read-first, park-one-arm-at-a-time recovery. |
| `UC/usage.py` `stop_all` (NEW) | Borrow the pattern | STOP ALL = arm slow stop + pole stop + hand freeze. |
| ALIX emulator + `emulated_program` | Engineering tests only | Fake controller for CI. A small shim is needed to run the runner subprocess against it. It cannot test finish events. |
| Trajectory HTML | Not applicable | No robot I/O (`HTML:437`). |

**Check of the input note** ("connect to the real robot, IP entry options, toggle real and sim"). This is a feature list, not a borrow claim, and today it is only partly met:
- Connecting exists in the control app, as one runner connection per action.
- IP entry does not exist in the UI (`UI:7506, 7588, 7600`), although the runner accepts `--ip/--port` (`RUN:2283-2286`).
- The real/sim toggle exists (v53 SIM/REAL/CHECK), but it switches the controller's dry-run mode, not a 3D simulation.
- ALIX Studio provides none of the three.

### 3.6 Borrow items
| Item | From | Adaptation |
|---|---|---|
| Connect by IP/port; mode get/set with read-back; run gates, start window, timeout, slow stop, speed restore | `RUN:46-49, 189-269, 1213-1262, 2283-2286, 2606-2614` | Call it as a subprocess with explicit `--ip --port --sdk --run-speed --restore-speed --timeout`. For workflows, also pass `--workflow-overall-speed` and per-step `transit_speed`. Pin the runner by sha256. |
| Live 3D mirror | `TG/server.py:5312-5393`; `editor.js:19218-19296` | Fed from runner telemetry. Degrees become radians on `L_joint1..7` / `R_joint1..7` (`TG/sim_bridge.py:26`). Both arms go in one map; source `external`. Label it "Live: real robot". |
| REAL confirmation | `editor.js:13786-13832`; `UI:3802-3810` | Shows both IPs, the mode read back, region, transit size and checklist chips. Primary button: "Run on real robot". |
| Readiness chips + stale-on-edit | `editor.js:15904-15950, 5371-5429` (pattern and CSS only) | New logic. Chips: Simulated, On robot, Practised, Real-tested. Any edit resets them. |
| Simulation licence keyed by content | `DP/ladder.py:101-145` | Rungs `LITE_SIM`, `DRY_RUN`, `REAL`, keyed by content sha in an append-only JSONL file. A failed simulation cannot be overridden. A missing one can be overridden by an engineer with a written reason. |
| Mode policy | `DP/runmode.py:70-90, 134, 205-268` | Refuse an unreadable mode. Restore the mode found after a dry run. Skip pole and hand in a dry run. |
| Completion witnesses | `DP/track.py:1573-1588, 2020-2036, 2037-2075, 1795-1830` | `played_fraction` ≥ 0.85 plus `phase_end` (port the numpy code). Finish event via an opt-in callback print in Lite's runner copy, enabled once commissioning confirms it. |
| Expected time | `DP/track.py:1558-1570` | Expected = transit estimate + 0.35 s dwell + 0.2322 s + 5.6737 ms × (N−1). Progress counts from departure from the first point. |
| Transit gap and strict mode | `DP/track.py:323-338, 407-410` | Used together with the new collision sweep of the joint-space line. |
| Recovery pattern | `ALIX/tools/dual_arm_flow.py:6180-6188`; `DP/staging.py:921-960` | Read joints first; stop all devices; move one arm at a time to a simulation-checked safe pose. |
| G4 commissioning probe | `DP/track.py:3273-3300`; `ALIX/tools/cleaning_tracks.py:22, 3729` | Run once per controller (and after firmware changes) on an in-place dry track. Record the answer. |
| STOP ALL devices | `UC/usage.py:141-148`; `RUN:714-737` | STOP = arm slow stop + pole stop + hand freeze for that side. |
| Dual REAL-mode warning | `UC/py programs/rm_trajectory_ui_v32_dual_files_fixed.py:7100-7125` | Warn and block when either arm is not REAL. |
| Verified stop ladder (library phase) | `DP/track.py:3069-3140` | Slow stop → pause → stop, each proven by run state and joint stillness. |

### 3.7 Build items
| Item | Approach |
|---|---|
| Connection settings and Test connection | Per-site JSON: `{left:{ip,port}, right:{ip,port}, sdk_dir, runner_path, runner_sha256}`, with IPv4 and 1–65535 validation. The test does three things: a raw TCP connect with a 2 s timeout (never during a job on that arm), runner `--get-arm-mode`, and runner `--list`. The list counts Lite programs and **warns if a controller holds the other side's Lite programs** (possible IP swap). |
| Run-target selector and mode synchronisation | Simulation, Robot dry run or Real robot. `--set-arm-mode` is verified by read-back. The opt-in `--require-mode` re-checks the mode inside the run process right before start. Lite records the mode it found and restores it after a dry run. Lite starts in Simulation every time. |
| Lite runner copy (opt-in additions) | <br>• `--telemetry` (joint line every 0.2 s) and `FINAL q=` after settle; <br>• `--require-mode real\|simulation`; <br>• `--expect-size N --expect-name S` (refuse if the slot does not match); <br>• `--track-ref FILE`, so the existing gap print (`RUN:207-214`) works for `--run SLOT`; <br>• `--set-global-speed N` (no motion), to retry a failed restore; <br>• a stdin control channel: `GO`, `STOP`, and stop on end-of-input via `_thread.interrupt_main()`, reusing the existing KeyboardInterrupt → slow-stop path on Linux and Windows; <br>• an optional `PROGRAM FINISH EVENT` print from the SDK event callback. <br>All are default off. A diff test proves the existing paths are unchanged. |
| Run-job manager | One job per arm under a per-arm lock shared with upload, drag-teach, pole and hand. Spawn the runner in its own process group. Parse stdout tokens into events. Status is polled at 2–4 Hz. STOP is sent **once** per job; repeated clicks do nothing. `--timeout` = expected × 1.3 + 30 s. After a "did not start" exit, read the run state and stop if a program is running. |
| Transit check | Read the joints after the mode is REAL (`--track-ref` print, or a joints read). Sweep the joint-space line to the first point at 1° steps in R3's collision world, about 2 ms per pose. Refuse on collision, or when the size exceeds the engineer limit. Show it in 3D. For dual starts, check both transits together, or start them one after the other. |
| Heartbeat and backend-death stop | The browser sends a heartbeat from a Web Worker every 1 s, so a hidden tab is not throttled into a false stop. The backend stops a REAL run after 3 s without one. If the backend process dies, the runner's stdin closes and it slow-stops. |
| Completion and stop index | Use all witnesses together (§3.2 f). Otherwise show the `phase_end` index as "Stopped at about X %", or "Not confirmed". |
| Recovery flow | After an unfinished run: read both arms; stop all devices; propose recovery in this order: (1) joint move to the arm's safe pose if the sweep is clear; (2) reverse out (the executed part played backward, uploaded as a temporary program, run at a slow transit); (3) hand guiding. One arm at a time. "Run again" is disabled until the arm is at a safe pose. |
| Gate check and checklist | Gates: simulation ledger, R12 gates, R13 registry (+ `--expect-size/--expect-name`), mode, codes, idle, transit check, and pole/hand read back from R7. The human checklist shows pictures from the regions deck. |
| Verdict, run log, Back to editing | Verdict modal. JSONL run record with: sha, slot, arm, target, mode found and read back, start/end, runner exit code, codes, final joints, witnesses, stop index, restore result, verdict and note. Deep link into the R2 editor. |
| Dual-arm start and stop-both | Launch both runners **without `--yes` for the start prompt** (or with the stdin `GO`). Wait until both print their prompt, which comes after all gates, then release both together and record the skew. A fault, non-zero exit or STOP on one arm stops the other. |
| Commissioning procedure | Engineer only, per controller, and again after firmware changes. Covers: G4 via `probe_stop_policy`; finish event present on a complete run and absent on a stopped one; whether `plan_speed` reads back the global ratio; mode read-back; speed restore; transit path joint-linear (one captured transit); dual start skew. REAL stays disabled for operators until this is recorded. |

### 3.8 Recommendation
**Run path.** Build R4 in Lite's backend around a pinned, Lite-owned copy of runner v14, run as a subprocess.
- Its proven upload and run code stays unchanged.
- Add only the opt-in additions listed above.
- Always pass `--ip/--port/--sdk` and explicit speeds.

**Safety rules from ALIX.** Take the rules ALIX learned on hardware, mostly as rules rather than code:
- read-back mode, with the dry-run pole/hand skip and mode restore;
- a simulation licence keyed by content, where a failed simulation cannot be overridden;
- several witnesses for completion;
- a transit you can predict, so Lite checks it;
- read-first recovery before any new motion;
- the stop ladder and the G4 probe for commissioning.

**Interface.** Take ALIX Studio's interface patterns: the live mirror, the confirmation modal, the status line and toast, and the chip and stale-on-edit pattern.

**Run targets.** Present three run targets with plain names. Lite opens in Simulation. A practice run on the robot comes before the first real run of new content, unless an engineer switches this off.

**Before operators.** Make commissioning (G4 and the finish event in particular) a precondition for operator REAL runs.

**Later.** Move to an in-process `alix_dispatch` session with a persistent connection and a verified stop ladder. Note that `session.py` handles SIGTERM/SIGHUP but not SIGINT (`DP/session.py:198-215`), so the stop path still needs Lite's own token.

### 3.9 Approach
1. **Settings and Test connection** (§3.7). Arm cards show "Left arm · Connected · Real mode · Ready".
2. **Run target.**
   - Lite opens in Simulation.
   - Dry run: `--set-arm-mode simulation` (verified). Lite records the mode found, skips pole and hand, and afterwards restores the mode and reads it back.
   - Real: confirmation modal, then `--set-arm-mode real`.
3. **Lite runner copy.** Vendor `RUN`, `hand.py` and `pole.py`. Add the opt-in flags. Prove the default paths unchanged by diff and by `ALIX/core/python/alix_dispatch/test/test_rm_upload.py`-style fake-SDK tests.
4. **Pre-run gate** (§3.2 c).
   - Lite applies the region's pole → hand preset with the arm still and the pad off the surface (R7), then reads back the pole height and hand counts.
   - The operator ticks only the human items.
5. **Transit check.**
   - With the mode REAL, read the joints and compute the gap.
   - Sweep the joint-space line in R3's collision world.
   - Show the move in 3D with its size.
   - Refuse on collision or when the gap exceeds the limit.
6. **Start.**
   - Single arm: `--run SLOT` with `--require-mode real --expect-size … --expect-name … --track-ref FILE --run-speed 20 --restore-speed 50 --timeout T --telemetry`.
   - Region with a preset: a `--workflow-sequence-file` (pole → hand → program only) with explicit `--workflow-overall-speed` and `transit_speed`.
   - Dual: release both prompts together.
7. **Monitor and stop.**
   - Telemetry feeds the live mirror and "Cleaning: 45 s of about 92 s".
   - STOP is sent once through stdin (or SIGINT on Linux). STOP ALL also stops the pole and holds the hand.
   - The heartbeat watchdog runs throughout. After every outcome, check the speed-restore result and offer Retry.
8. **Completion.** Combine the witnesses (§3.2 f) and report Finished, "Stopped at about X %" or "Not confirmed".
9. **Recovery.** If the run did not finish, run the recovery flow before anything else.
10. **Verdict and return.** Verdict modal, run record, then Back to editing at the stop index or a marked point. Edits reset the chips.
11. **Commissioning** (§3.7) before operator use. Repeat it after firmware changes.
12. **Later.** Library path: `open_arm`, `require_real`, `global_speed_guard`, `run_program`, `stop_program`, plus Lite's stop token, keeping the same tests.

### 3.10 Acceptance criteria
1. **Settings.** IP and port per arm, prefilled 192.168.1.10 / 192.168.1.103 / 8080.
   - An invalid IPv4 address or a port outside 1–65535 is rejected with a message.
   - Values persist after restart.
   - Only the engineer role can edit them.
2. **Test connection** returns Connected, Unreachable or Error within 5 s per arm (tested against an unreachable address). When connected it shows the mode read back. It warns when a controller holds Lite programs tagged for the other side.
3. **Target.** Every runner command Lite builds contains `--ip` and `--port` equal to the settings (unit test). `RM_LEFT_IP`/`RM_RIGHT_IP` in the environment do not change the target.
4. **Mode.** Lite opens in Simulation every time.
   - Switching to Real robot shows a confirmation with both IPs, then the mode read back.
   - A mismatch disables Start with a plain reason.
   - With `--require-mode real`, a fake controller that reports SIMULATION at start time refuses the run.
5. **Refusals.** A REAL run is refused with a named reason in each of these cases (one test each):
   - no simulation record (an engineer override with a reason is possible);
   - a failed simulation record (no override);
   - motion gates fail;
   - slot size or name does not match (`--expect-*`);
   - mode not REAL;
   - a blocking code is present;
   - a program is already running;
   - the transit check fails;
   - the pole/hand read-back does not match;
   - the checklist is incomplete.
6. **Transit check.** With a fake current pose whose joint-space line to the first point passes through the commode model, Start is refused and the clash is shown in 3D. The transit size in degrees is shown before every start.
7. **STOP.**
   - STOP per arm and STOP ALL are visible without scrolling on every screen during motion.
   - One click leads to `rm_set_arm_slow_stop` within 0.5 s (fake-SDK timestamp), with no dialog.
   - Repeated clicks do not send a second interrupt.
   - STOP during a pole or hand step stops that device.
   - Hardware latency is measured at commissioning.
8. **Stop on both platforms.** The stdin STOP works on Linux and, if it is a target, Windows (CI on both). Killing the Lite backend during a fake run makes the runner slow-stop and restore the speed.
9. **Speed restore.** After every outcome (complete, failed, stopped, timeout) the restore is attempted, its return code is shown and stored, and a failed restore offers "Retry" (no motion).
10. **Heartbeat.** If the client stops sending heartbeats for more than 3 s during a REAL run, the run is slow-stopped. A browser tab hidden for 10 minutes during a fake run causes no false stop.
11. **Completion.**
    - A fake run that ends at 40 % of the file is reported as "Stopped at about 40 %" (±3 %) although the runner exits 0.
    - A fake run whose final pose matches the end but whose playback took less than 85 % of the expected time reads "Not confirmed".
    - With the finish-event witness enabled, a run with no event is never shown as "Finished".
12. **Recovery.** After a stop away from the start or end, any new run (including "Run again") is disabled until a recovery move, checked in simulation, has put the arm at its safe pose. Recovery moves one arm at a time.
13. **Verdict and record.** A verdict is required before the same content can run on the real robot again. The run record holds every field listed in §3.7.
14. **Back to editing** opens the trajectory at the stop index or the marked point. After any edit, the chips show "needs re-check".
15. **Dual arms.**
    - Start skew between the two `rm_set_program_id_run` calls is 0.2 s or less on the fake controller and is measured on hardware.
    - A fault or STOP on one arm slow-stops the other within 1 s (fault injection).
    - The dual run is blocked when either arm is not REAL.
16. **Dry run.** Pole and hand steps are skipped, and the screen says so. The mode found before the dry run is restored and read back afterwards.
17. **Commissioning record per controller.** Covers:
    - G4: Lite's STOP ends a running uploaded program, with the arm still within 2 s;
    - the finish event (present / absent);
    - `plan_speed` read-back (yes / no);
    - mode read-back;
    - speed restore;
    - the transit path.

    Operator REAL stays disabled until this record exists.

### 3.11 UX notes (non-technical operators)
- **Target names.** "Simulation (no robot)", "Practice on robot (arms stay still)" and "Real robot (arms move)". The practice run before the first real run of new content is suggested automatically.
- **Mode banner** on every screen:
  - neutral for Simulation;
  - amber with "PRACTICE: arms will not move";
  - a red border with "REAL ROBOT: arms will move".

  Colour always comes with a word and a glyph (ALIX Studio's ISA-101 rule).
- **Arm cards** read "Left arm · Connected · Real mode · Ready". IP and port are read-only there.
- **Checklist** pictures come from the regions deck: lid closed, lid open, seat open. The boxes are: area clear, pad off the surface, E-stop within reach. Pole and hand are set by Lite and shown as done ✓.
- **Transit preview.** "The arm will first move 34° to its start position (shown in 3D). Make sure this path is clear."
- **STOP buttons.** One large red STOP per arm and a STOP ALL, at least 44 px, with Space and Esc, always visible during motion. Help text: "Stops the arm gently. In an emergency press the red E-stop on the robot."
- **Progress in words.** "Moving to start position…", "Cleaning: 45 s of about 92 s", "Finished: did it go well?".
- **Translated controller codes.**
  - 4109: "The robot detected a collision and stopped."
  - 4105: "A joint moved faster than its rated speed (warning)."
  - 4114: "Self-collision."
  - 4115: "Outside the safety fence."
  - 4116: "A joint went past its software limit. Press and hold the green button at the end of the arm to recover."
- **After a stop:** "The arm stopped part-way. Lite will first move it back to a safe position. Check the path shown, then press Move back."
- **Verdict.** Four big buttons: Looks good / Hit something / Missed a spot / Other, plus a note. Then Back to editing (primary) or Run again (enabled only after recovery).
- **Engineer-only details.** Never show slots, speeds, runner or SDK paths to operators; they go in an engineer panel. Reuse ALIX Studio tokens (light theme, brand `#2F96EE`); do not reuse the Tk app's dark palette.

### 3.12 Safety notes
- **Transit.** The controller does not collision-check the transit (`RUN:207-214`; `DP/track.py:323-338`). Check it in simulation, run it at 20–30 %, show it, and refuse large moves.
- **Global speed.** The run speed becomes the controller-wide ratio and survives power cycles. Restore it on every exit path, retry if the restore fails, and never interrupt the runner twice, because a second interrupt can skip the restore in `run()` (inferred, `RUN:252-262`).
- **Mode.** Read it back inside the run process immediately before start. A SIM request that silently stayed REAL moves real metal, and an arm left in SIM produces phantom "complete" runs (`DP/runmode.py:70-90, 205-220`).
- **Dry run.** Never send pole or hand commands in a dry run (`DP/runmode.py:229-236`). A dry run is never a commode clearance check.
- **G4 is unproven.** Whether slow stop ends a running stored program has never been tested on hardware (`DP/track.py:3060-3068`). The hardware E-stop is the primary emergency means, and REAL for operators waits for commissioning.
- **The program outlives its client.** It keeps playing if the client dies (`DP/emulated_program.py:58-62`). Hence the heartbeat, the stop on backend death (stdin end-of-input), and the stop after a "did not start" exit (`RUN:238-240`).
- **"RUN COMPLETE" is not proof of a complete run.** Silent cuts exist (`ALIX/plan/FINDINGS.md:9563-9572`). Show "Finished" only on all witnesses.
- **Recovery before any new motion.** A stopped arm may rest on the commode, and "Run again" starts with an unchecked transit out of that pose (H40, `ALIX/tools/dual_arm_flow.py:6180-6188`).
- **STOP and blocking calls.** STOP cannot interrupt blocking SDK calls (drag playback, global MoveJ/MoveL). Real tests use only non-blocking program runs.
- **Workflow step order.** Order workflow steps pole → hand → program only. The runner may start a step after a program before the arm is confirmed still (`RUN:1949-1980`).
- **One command source per arm.** No ALIX ROS driver, pendant program or **concurrently running Tk control app** may drive an arm during a Lite run (`RUN:26`). Lite's lock cannot see the Tk app.
- **Pole and hand rules.** Never move the pole or hand with the pad on a surface, and move the pole only with the arm still (`UC/usage.py:25`). With the bin stowed, avoid pole heights of 0.153–0.282 m (`TG/studio_generate.py:475`).
- **Wrong arm.** An IP swap sends one arm's program to the other arm. IP changes are engineer-only; programs are side-tagged; Test connection warns; the live mirror lets the engineer confirm that each arm on screen matches the physical arm.
- **Overshoot.** Fast files overshoot commanded joint positions by 4–20°, and soft-limit cuts follow (`ALIX/plan/FINDINGS.md:10318-10322, 9533-9536`). R12 must keep a joint-limit margin (ALIX uses 5°).

### 3.13 Dependencies, risks, open questions
**Dependencies:**
- **R3:** the 3D simulation, the simulation-pass record and the collision world, also used for the transit and recovery checks.
- **R5:** stop and collision highlighting.
- **R2:** the editor to return to, with stale-on-edit.
- **R12:** the motion gates, including the joint-limit margin.
- **R13:** verified upload, the slot registry and the expected size/name.
- **R7:** pole/hand presets applied by Lite with read-back, and the arm-still interlock.
- **R10/INT-CTRL:** dual scheduling (port of the v53 planner).
- **R1:** region identity and checklist pictures.
- **NFR-UX / NFR-THEME:** the Studio shell components.

**Risks:**
- G4: STOP may not end a running stored program.
- The program-finish event may behave differently on these controllers (it is hardware-observed by ALIX only).
- Silent cuts could still read as complete if the witnesses are not implemented.
- The transit model is inferred from movej; it is confirmed at commissioning.
- Recovery when no collision-free retreat exists falls back to hand guiding.
- A wrong controller mode, or an IP swap.
- The controllers may be shared with ALIX (ROS driver, slots), and the Tk app may run at the same time.
- The dual start skew has never been measured, and the pole timing model makes time-matched TOGETHER blocks drift by about 2–3 s.
- The Windows stop path, if the stdin channel is not adopted.
- Runner/UI version drift (the v13 fallback runs transits at 100 %).
- A browser hidden in the background could cause false stops without the Web Worker heartbeat.

**Open questions:**
1. By "toggle between real and sim", did the requester mean Lite's 3D simulation, the controller dry run, or both?
2. Does slow stop end a running uploaded program on these controllers (G4)? Who runs commissioning, and when?
3. Does the `program_run_finish` event arrive on both controllers in the runner's process?
4. Does `rm_get_program_run_state().plan_speed` read back the global ratio when idle?
5. What heartbeat timeout should apply (proposed 3 s)?
6. What exactly must "validated in simulation" include (clearance threshold, commode state per region), and who may override a missing record?
7. Is the dry run before the first real run of new content wanted by default?
8. What is each arm's safe pose for recovery, and is "reverse out" acceptable on contact strokes?
9. What transit size limit should apply (e.g. 45°)?
10. Which OS will the operator laptop run?
11. Are the controllers shared with ALIX production and the Tk app? Who guarantees a single command source?
12. Is a slower rehearsal file required for the first real run?
13. Are photos or video wanted with verdicts?

### 3.14 Priority and effort
**Priority:** Must.

**Effort:** **XL**, about 18 person-days including tests:

| Item | Days |
|---|---|
| Settings and connection | 2 |
| Target, mode sync and restore | 1.5 |
| Run-job manager, stop, heartbeat, backend-death stop | 3 |
| Runner opt-in additions with diff and fake-SDK tests | 2 |
| Gate check and checklist | 1.5 |
| Transit check | 1 |
| Completion witnesses, stop index, verdict, run log, back-to-editing | 2 |
| Recovery flow | 1.5 |
| Live mirror and status | 1.5 |
| Dual GO handshake and stop-both | 1 |
| Commissioning procedure (reusing `probe_stop_policy`) | 1 |

- A **single-arm MVP** (no dual, no live mirror) is **L**, about 14–15 days.
- Excluded: R3's 3D simulation, R5's plot highlighting, the v53 dual-timeline port (INT-CTRL), and the later library migration (about M).

---

## 4. R13: Faster trajectory upload

### 4.1 Original text
"Faster Trajectory Upload: The software should support fast and efficient uploading of trajectory files to the robot controller for real-arm testing. - Check some of the findings in the alix ws related work"

### 4.2 Revised requirement
Lite shall get trajectories onto the robot controllers quickly and safely, and make the waiting visible.

- **(a) [Must] Exact, verified upload.**
  - Upload exactly the content that passed simulation and the checks, in the controller's program format: every line ending in CRLF, header `{"file":7}`, then a Folder line whose `type` equals the point count.
  - Store only; never auto-run.
  - Verify each program by reading back its stored size, which must equal the bytes sent within the configured tolerance (default `max(64 B, 0.1 %)`, tightened to exact if commissioning shows exact matches).
  - A failed verification blocks running that program.
- **(b) [Must] Program-ID ownership.**
  - Write only inside Lite's agreed ID range on each controller.
  - Never write ALIX's IDs (70–100 production, 60 dry, 40–59 taught, 21–39 and 61–69 experiments) unless an engineer explicitly allows it.
  - Never overwrite any program Lite did not write (pendant programs may sit in low slots) without an engineer's confirmation.
- **(c) [Should] Skip unchanged.** When Lite's registry hash, the stored size and the stored name (which carries a content-hash fragment) all match, show "Already on robot" and send nothing. Any mismatch or unreadable value means send again.
- **(d) [Should] One session per arm, both arms in parallel.** One connection and one program-list read before and one after each batch. Both arms upload at the same time.
- **(e) [Should] Background queue.**
  - Uploads run in the background with per-arm progress and a time estimate; the operator keeps working.
  - Optionally, a trajectory is queued as soon as it passes simulation.
  - Background uploads never overwrite a slot that needs confirmation.
- **(f) [Should] Show the cost.** The editor shows each trajectory's upload cost (points → seconds).
- **(g) [Must] Busy interlock.** Never upload to an arm that is running a program, drag-teaching or moving its pole or hand.
- **(h) [Could, future]** CANFD streaming, which needs no upload, as a separate engineering option with its own safety case.

### 4.3 Clarifications
- **"Faster" cannot mean a faster transfer.**
  - The controller acknowledges every 2 KB chunk. Five successful ALIX uploads of 180–325 KB took 4.04–7.15 s, which is 44.3–46.5 kB/s.
  - A fit over the six fully transferred uploads gives **0.24 s + 21.2 µs × bytes** (derived). The six include the LF-only upload the controller rejected only after receiving all of it (6.62 s). That is about 1.2 ms per point at about 58 B per point.
  - No faster transfer path exists in the vendor API or ALIX's evidence.
  - The measured range is 180–325 KB. Larger files are extrapolated; a 712.8 KB upload is known to work, but it was not timed (`UC/README_COMMANDS.md:45-51`).
- **The control app's "fast"** (runner v4 / v12) means switching quickly between stored programs, not faster upload. Its upload code is ALIX's `tools/rm_upload.py`.
- **Worked example** (derived, using the fit and the actual `.proj` sizes):

  | Package | Files | Bytes | Points | Estimated upload |
  |---|---|---|---|---|
  | V6 LEFT | 7 | 2,598,732 | 45,401 | about 55 s |
  | V6 RIGHT | 8 | 2,370,352 | 41,053 | about 51 s |
  | Largest single file `L_P03P05.txt` | 1 | 631,725 | 10,983 | about 13.7 s |

  That is about 1 min 46 s one arm after the other, or about 55 s with both arms in parallel.
- **A Program ID** is a controller slot: 1–100 per controller, persistent across power cycles, never reclaimed automatically.
  - The controller stores the program as `<id>_<file stem>.txt`, cut to 32 bytes; it forces the `.txt` extension.
  - The stem must therefore be 24 characters or fewer with a 3-digit ID.
  - The temporary project path must stay under 300 bytes.
- **"Upload" in the trajectory HTML** means opening a file. In Lite, "Send to robot" means the controller upload.
- **Fewer points** (pause removal, re-timing) is implemented in R2; R13 measures it and shows it.

### 4.4 Findings (verified; NEW = not in the draft)

| # | Source | Finding | Evidence |
|---|---|---|---|
| 1 | ALIX Studio | No upload, slot or controller code at all, and a test pins it as having no execution channel. | grep of `TG`; `test_studio_interaction_guards.py:2204-2216` |
| 2 | Runner v14 | **Upload path = the original ALIX uploader.** <br>• Parses the whole file; refuses on any bad line or fewer than 2 points. <br>• Writes `{"file":7}` + Folder(type = N) + the points, all CRLF. <br>• Reads the list in pages of 10. <br>• `rm_send_project(path, 0, only_save=1, slot, step_flag=0, auto_start=0, project_type=0)`; success = ret 0 and err_line −1. <br>• Verifies the stored size within `max(64, sent // 1000)`. <br>• No motion checks before upload. | `RUN:76-175`; `SDK/rm_ctypes_wrap.py:2269-2292`; diff against `ALIX/tools/rm_upload.py` |
| 3 | ALIX runs | **Throughput:** 293,247 B in 6.31 s; 325,446 B in 7.15 s; 290,066 B in 6.43 s; 214,009 B in 4.84 s; 180,153 B in 4.04 s. The LF-only file (293,245 B) was rejected after 6.62 s. The controller acknowledges every 2 KB. "Upload already costs 6.4 s for 290 KB." | `ALIX/runs/2026082{8,9}T*_upload*/run.json` (`stages[upload]`); `ALIX/plan/FINDINGS.md:5701-5706, 5772-5775`; `ALIX/docs/motion/MOTION_TRANSPORT_PLAN.md:210` |
| 4 | UI v53 | Batch upload starts **2N+1 runner processes** (each its own connection) for N files. | `UI:4504-4692` |
| 5 | UI v53 | RUN BOTH .TXT already uploads LEFT and RIGHT at the same time. It writes slot 99 on both arms (ALIX production range), and single uploads default to slot 25 (ALIX experiment range). **NEW:** UPLOAD passes `--yes`, which skips the runner's prompt showing what the slot holds. The control app has no protection for other people's programs. | `UI:8800-8840, 6966-6967, 6933-6934, 4275-4290`; `RUN:148-157` |
| 6 | Runner v14 | "Fast" = switching between stored programs: the list is read once, 50 ms polling and a quick settle. It does not mean faster upload. | `RUN:1766-1804, 1986-2033, 178-186` |
| 7 | ALIX `cleaning_tracks.py` | **One-session batch:** one connection, every row sent, then one list read comparing stored sizes. Rows that no longer match their sha256 pins are refused. **NEW:** <br>• the name check is downgraded to advisory when the naming model is unconfirmed; <br>• programs it did not write are refused only in experiment slots, "shared ground with the pendant's own programs". | `ALIX/tools/cleaning_tracks.py:4046-4160, 29-43, 4071-4087, 4107-4133` |
| 8 | ALIX policy | **Slot use.** <br>• 70–100 production; <br>• **NEW:** 60 dry; <br>• 40–59 taught, and **NEW:** 101 and up as CANFD-only registry numbers; <br>• 21–39 and 61–69 experiments; <br>• **NEW:** "slots below 40 are the band the pendant may also use", and slot 1 holds a pendant program (`1_Hand_test_new.txt`). <br>The V6 package uses 71–83 (IDs shifted +70); the README example used 85. | `DP/track.py:13-16`; `ALIX/tools/cleaning_tracks.py:1-12, 2755-2782, 2858-2870`; `DP/emulated_program.py:333-335`; `ALIX/Resource/tracks/taught/README.md:164-169`; `V6/README.txt:101-104`; `UC/README_COMMANDS.md:45-51` |
| 9 | Naming (corrected) | `trajectory_name` = `<id>_<stem>.txt`, cut to 32 bytes. This was **measured on hardware by ALIX on 2026-09-10** (`90_project.txt`, `70_70_commode_c_toplid_right_exe` exactly 32, slot 60 `…_020.txt`) and is also seen in the control app's session (`85_FILE_01.1.txt`). The project path field is 300 bytes. `only_save=0` starts the program immediately. | `DP/emulated_program.py:20-30, 76-87, 326-341`; `UC/README_COMMANDS.md:51` |
| 10 | ALIX FINDINGS | **The controller is busy during and after a large upload.** The push restore after a 293 KB upload got no reply (ret −2); push arrival jitter averaged 7.55 ms, with a maximum of 400.9 ms. | `ALIX/plan/FINDINGS.md:5816-5822`; `ALIX/runs/20260828T213858_exp01_G1_upload_only/run.json` |
| 11 | ALIX taught lane | **Fewer points is the main lever.** <br>• Operator holds are about 24 % of nominal time. <br>• TOPP-RA re-timing gives 3.2–4.0× fewer lines. <br>• Same toplid path: 5,047 → 3,142 points cut the upload from 6.43 s to 4.04 s. | `ALIX/docs/motion/TAUGHT_CANFD_PIPELINE.md:41-51, 582-591`; `ALIX/runs/20260828T221443_exp01_G4_upload_only`, `20260829T191701_exp01_H1_upload_fast` |
| 12 | alix_dispatch | **The same verbs exist as a library:** `parse_track`, `build_project_file`, `send_project` (returns seconds), `program_list` (pages of 10), `preflight_slots` (size check). "A short slot would end its program early with no error." | `DP/track.py:66-152, 353-390, 367-374` |
| 13 | alix_dispatch (corrected citation) | **CANFD streaming** needs no upload: one line per 5.0 ms after a whole-file step and rate preflight, with in-flight abort rules. ALIX's taught CANFD pipeline is **scoped to the left arm** (the emitter hard-codes it). | `DP/track.py:3706-3866, 3989`; `ALIX/docs/motion/TAUGHT_CANFD_PIPELINE.md:10` |
| 14 | V6 vs R12 (NEW, derived) | **At 5 ms per point, 10 of 15 V6 files exceed a joint velocity rating.** The peak is 2.18× on `R_P06P07.txt` J6 (491°/s, 2.45° per step). A strict R12 rate gate would block the current production package. Fast files also overshoot commanded positions by 4–20°. | python over `V6/{LEFT,RIGHT}/*.txt`; `ALIX/plan/FINDINGS.md:10318-10322` |
| 15 | Size tolerance (NEW) | `max(64 B, 0.1 %)` lets about 600 B (about 10 points) go missing on a 600 KB file. The one recorded hardware read-back matched exactly (729,857 sent and stored). | `RUN:171`; `UC/README_COMMANDS.md:51` |
| 16 | Trajectory HTML | "Upload" means opening a file; output reaches the robot only as a downloaded `.txt`. | `HTML:325-345, 526-531` |

### 4.5 Borrow verdict
**ALIX Studio: Not available. Build new.** The Studio has no upload, slot or controller code (`test_studio_interaction_guards.py:2204-2216`). Only generic UI parts can be reused for the queue: the status line and toast (`editor.js:5431-5462`) and a Board-style status table.

| Other source | Verdict | What to use |
|---|---|---|
| Runner v14 upload path (= ALIX `tools/rm_upload.py`) | Borrow as-is for the MVP, with one opt-in addition | The exact `.proj` bytes and the read-back verify (`RUN:76-175`). Add `--upload-batch MANIFEST.json` in Lite's runner copy. |
| `alix_dispatch.track` | Borrow with adaptation (target) | `build_project_file`, `send_project`, `program_list`, `preflight_slots` for an in-process store with one persistent session per arm (`DP/track.py:66-152, 353-390`). |
| ALIX `tools/cleaning_tracks.py` | Borrow the pattern | One-connection batch, a single read-back, sha pins, refusal of programs it did not write, the name treated as advisory when the naming model is unconfirmed (`cleaning_tracks.py:4046-4160, 29-43`). |
| ALIX measurements | Borrow as data | The ETA model, the CRLF rule, paging by 10, the controller-busy behaviour. |
| `alix_dispatch.emulated_program` | Borrow for tests | An offline controller double that checks the format, the 32-byte name and the 300-byte path, and counts sends and list reads. |
| UI v53 | Reference only | Parallel L/R uploads (`UI:8800-8840`) and a per-file ID dialog. Do not copy the 2N+1 process pattern or its `--yes` overwrite behaviour. |
| CANFD streaming | Reference only (future) | Avoids upload. Needs host timing, a different rate gate and its own safety case; the taught pipeline is left-arm only. |
| Trajectory HTML | Not applicable | No controller I/O. |

**Check of the input note** ("Check some of the findings in the alix ws related work"): accurate as a pointer.
- ALIX holds the measured throughput, the exact format and CRLF rule, paged listing, sha and size pins, a one-session batch, slot ownership rules and the naming model.
- Those same findings show the transfer rate is fixed by the controller. "Faster" must come from fewer points, skipping unchanged programs, one session per arm, both arms in parallel and background queuing, or from avoiding upload with CANFD, which is a different and riskier channel.

### 4.6 Borrow items
| Item | From | Adaptation |
|---|---|---|
| Project-file writer, whole-file parser, store-only upload, size read-back | `RUN:76-175` (= `ALIX/tools/rm_upload.py`) | Same bytes. Batched over one connection. Refusal reasons returned as data. |
| Timed upload and paged list | `DP/track.py:66-152, 353-390` | Vendored. The returned seconds calibrate the ETA. |
| One-connection batch, single read-back, pins, foreign refusal, advisory name | `ALIX/tools/cleaning_tracks.py:4046-4160, 29-43` | Lite registry pins: txt/proj sha256, bytes, points. The check for programs Lite did not write covers Lite's whole range. |
| Slot ranges and name/path limits | `DP/track.py:13-25`; `cleaning_tracks.py:2755-2782, 2858-2870`; `DP/emulated_program.py:76-87, 326-341` | A configurable Lite range per arm. Stem of 24 characters or fewer. Temporary path under 300 bytes. |
| Throughput model | ALIX `runs/*upload*/run.json`; `FINDINGS.md:5701-5706, 5772-5775` | ETA = 0.24 s + 21.2 µs × bytes, recalibrated from Lite's own logged seconds. |
| Offline controller double | `DP/emulated_program.py` | Counts connections, sends and list reads. |
| Parallel L/R | `UI:8800-8840` | Two per-arm workers in the backend. |

### 4.7 Build items
| Item | Approach |
|---|---|
| Program registry and ID policy | One JSON file per arm mapping each slot to `{txt_sha256, proj_sha256, proj_bytes, points, short_name, stored_name, uploaded_at, verified}`. Allocate only inside Lite's range. Read the controller's list before each batch and refuse any slot holding a program Lite did not write (engineer confirmation only). Reuse a slot that already holds identical verified content. |
| Naming | `L1A_r3_3f9c2a` = side + region code + revision + 6 hex of the content sha, at most 24 characters. The side tag also lets Test connection detect swapped IPs. |
| Batch upload in one session | MVP: opt-in `--upload-batch MANIFEST.json` in Lite's runner copy. It reuses `read_track` / `write_project` / `rm_send_project(only_save=1)` / size verify on one connection, with one list read before and one after, and prints VERIFIED / NOT VERIFIED / SKIPPED with seconds per file. Target: an in-process store on `alix_dispatch.track` + `connect.open_arm`. |
| Queue with per-arm workers | LEFT and RIGHT in parallel, first in first out with "needed next" priority. States: Waiting, Sending, Checking, On robot, Already on robot, Not verified. Jobs can be cancelled before sending. Background pre-staging waits instead of overwriting a slot that needs confirmation. |
| Skip logic | Skip only when sha, size and stored name all match. Anything unreadable, or a name mismatch, means send again. |
| ETA and progress | Estimated per file from bytes (the SDK has no progress callback). Shown per arm and in total. |
| Busy interlock | The per-arm lock is shared with runs, drag-teach, pole and hand. Before sending, check `rm_get_program_run_state` = 0. |
| Pre-upload gate hook | R12 checks before queueing, with plain reasons. Any engineer override is recorded with a reason (needed for the existing V6 files; see risks). |
| Upload-cost readout | "Send to robot: about X s (N points)" next to the duration. |
| Measurement | Log bytes and seconds per file. Hardware acceptance with the V6 package, serial versus parallel, plus the skip path. Check at commissioning whether stored sizes match exactly. |

### 4.8 Recommendation
Treat upload as a background service built on a content-identity registry, not as a button.
- Send only changed content, over one session per arm, with both arms in parallel.
- Keep the proven byte format and the size read-back, and show an honest estimate.
- **The real speed gain comes from R2.** A trajectory sped up 2× has half the points and uploads in half the time, so the editor should show the cost.
- **MVP:** keep the runner's proven upload code and add an opt-in batch mode in Lite's copy. **Later:** move to an in-process `alix_dispatch.track` session.
- **Before writing to shared controllers:** agree Lite's Program-ID range with the ALIX owners, and remember that low slots are pendant territory.
- **Before the first hardware tests:** agree how R12 treats the existing V6 files that exceed the joint ratings.
- Keep CANFD streaming as a later, separately justified engineering option.

### 4.9 Approach
1. **Program-ID range.** Agree Lite's range per controller with the ALIX team. The proposal is 1–20 per arm, configurable, with programs Lite did not write left alone. Refuse any ID outside the range in code.
2. **Registry and naming.** Implement the registry and short side-tagged naming (24-character stem or less). Keep the temporary path under 300 bytes.
3. **Runner copy with batch mode.** Vendor runner v14 with `hand.py` and `pole.py`, pinned. Add `--upload-batch`. Prove by diff that the default paths are unchanged, and test against a fake controller.
4. **Upload service.** One worker per arm, in parallel. Per batch:
   1. take the per-arm lock and check run state 0;
   2. read the list;
   3. refuse programs Lite did not write in target slots;
   4. compute the skip set;
   5. send the rest;
   6. read back once;
   7. update the registry and release the lock.
5. **Queue UI.** Per-arm progress, ETA, chips. An optional setting pre-stages a trajectory after its simulation passes (it never overwrites without confirmation).
6. **R12 gates** before queueing, with plain reasons and recorded engineer overrides.
7. **Upload-cost readout** in the editor.
8. **Hardware measurement.** One 600 KB file, the V6 package serial versus parallel, and the skip path. Calibrate the ETA. Check whether stored sizes match exactly.
9. **Phase 2.** In-process store on `alix_dispatch` (persistent session, reconnect), with the same registry, tests and UI.
10. **Future, by separate decision:** evaluate CANFD streaming for test runs.

### 4.10 Acceptance criteria
1. **Bytes.** For three sample files (one LF, one CRLF), Lite's `.proj` bytes are byte-identical to runner v14 `write_project` output.
2. **Store only.** Every upload uses `only_save=1`, `auto_start=0` and `project_type=0` (asserted on a fake SDK).
3. **Verification.** Each program is verified by stored size within the configured tolerance. A mismatch shows "Not verified: do not run" and disables running that program.
4. **One session.** A batch of N files to one arm opens exactly one connection and performs at most two program-list reads (fake counters).
5. **Skip.**
   - Unchanged content causes zero sends and shows "Already on robot".
   - Changing one file in a batch of seven sends exactly one.
   - A stored-name mismatch triggers a re-send, never a skip.
6. **Parallel.** LEFT and RIGHT run concurrently. With a fake controller at 21 µs per byte, total wall time is at most 1.15 × the longer arm alone.
7. **Ownership.**
   - Lite refuses any ID outside its range.
   - It refuses to overwrite a program it did not write unless an engineer confirms.
   - Background pre-staging never overwrites without confirmation.
8. **Busy interlock.** Upload is refused while that arm has an active run, drag-teach, pole or hand job, or reports run state 1/2.
9. **ETA accuracy.** On hardware the ETA is within ±25 % of the measured time for at least 5 files of 50 KB or more.
10. **Hardware timing.** The V6 LEFT package (7 files, 2.60 MB) uploads and verifies in 70 s or less, and both arms in parallel (about 5.0 MB) in 75 s or less (the fit predicts about 55 s and 51 s).
11. **Motion gates.** A file that fails the motion gates (for example, a 12° step) is never sent without a recorded engineer override.
12. **Names.** Stored names are 32 bytes or fewer, including `<id>_` and `.txt`, and carry the side, region and a content-hash fragment.

### 4.11 UX notes
- Say **"Send to robot"** for the controller upload, and rename the HTML's "Upload trajectory" to **"Open file"**.
- The per-arm queue reads like "Left arm: sending 3 of 7, about 40 s left", with chips On robot / Already on robot / Not verified.
- Sending never blocks editing or simulation. A toast appears when a batch finishes or fails.
- Show the cost next to the duration, e.g. "Send to robot about 9 s (7,353 points)"; after a 5× speed-up, "about 2 s (1,471 points)".
- Hide Program IDs from operators. Engineers see "Robot memory: 12 of 20 Lite programs used". Ask only when an old Lite program must be replaced, and never inside a background job.
- Plain messages: "The left robot did not confirm it stored the whole file. Please send it again." and "The right arm is moving; the file will be sent when it stops."
- Estimates are labelled "about", because the SDK reports no transfer progress.

### 4.12 Safety notes
- **An upload never moves the arm.** Keep `only_save=1`, `auto_start=0`; `only_save=0` starts the program immediately (`RUN:159`; `DP/emulated_program.py:20-30`).
- **Do not upload to a busy controller.** Do not upload to a controller that is running a program or being monitored; the controller answers late during and after a large upload (`ALIX/plan/FINDINGS.md:5816-5822`).
- **Keep the size read-back.** ret 0 / err_line −1 means "accepted", not "stored completely", and a short slot ends early with no error (`ALIX/tools/cleaning_tracks.py:4107-4111`; `DP/track.py:367-374`).
- **Never send half a file.** Refuse malformed files whole (`RUN:76-88`).
- **Leave other people's programs alone.** Never overwrite ALIX programs (70–100, 60, 40–59, 21–39, 61–69), pendant programs or anything Lite did not write. Slots persist and are never reclaimed (`DP/track.py:13-25`; `ALIX/tools/cleaning_tracks.py:2866-2869`). The control app's UPLOAD skips this check (`UI:4275-4290`).
- **Fewer points means faster motion.** That is safe only behind R12's rate, step and joint-limit-margin gates. 10 of 15 V6 files exceed ratings at 5 ms (derived), and fast files overshoot by 4–20° (`ALIX/plan/FINDINGS.md:10318-10322`).
- **When in doubt, re-send.** If the registry or the controller read-back cannot be read, send again rather than assume the program is on the robot.

### 4.13 Dependencies, risks, open questions
**Dependencies:**
- **R2:** fewer points, through pause removal and re-timing. This is the main lever.
- **R12:** pre-upload gates, plus an override policy for existing files.
- **R3:** a simulation pass triggers pre-staging, using the same content identity.
- **R4:** runs need a verified slot for the exact content, using `--expect-size/--expect-name`.
- **R8:** merged files are larger, so uploads take longer.
- **INT-CTRL:** runner packaging, settings, the per-arm lock.
- **R1:** region codes used in program names.

**Risks:**
- **Expectations.** The transfer rate is fixed; a 600 KB file still takes about 13 s.
- **ID collisions** with ALIX, pendant programs or the control app (slots 25, 85, 99, 71–83) overwrite their programs or leave stale content.
- **R12 strictness.** If R12 blocks on joint ratings, 10 of the 15 current V6 files cannot be sent until re-timed or overridden.
- **Registry drift.** The registry drifts when slots change from the pendant or another PC; it is mitigated by reading back size and name on every batch.
- **Naming model.** It is measured on ALIX controllers but not yet confirmed on both of Lite's target controllers. If it differs, skip falls back to sha + size, and the name is advisory (ALIX practice).
- **Unmeasured.** Parallel throughput and per-process overhead have not been measured. In-place overwrite of an occupied slot is ALIX practice but not separately verified.

**Open questions:**
1. Which Program-ID range does Lite own on each controller? Are the controllers shared with ALIX (70–100, 60, 40–59, 21–39, 61–69) and with the V6 workflow (71–83)?
2. May Lite delete its own old programs (`rm_delete_program_trajectory`), or only overwrite them?
3. What upload wait per region test is acceptable to operators?
4. How should R12 treat existing files above the joint ratings: re-time, warn, or engineer override?
5. Should the size check be exact (one hardware example matched exactly) instead of `max(64 B, 0.1 %)`?
6. Is automatic background upload after a simulation pass on or off by default?
7. Is CANFD streaming wanted later (Linux host, different rate gate, left arm only so far)?

### 4.14 Priority and effort
**Priority:** Must.
- (a) exact verified upload, (b) ID ownership and (g) the busy interlock are **Must**.
- (c)–(f) are **Should**.
- (h) CANFD is **Could**.

**Effort:** **M**, about 8 person-days:

| Item | Days |
|---|---|
| Registry, ID policy and foreign-program check | 1.5 |
| Naming | 0.5 |
| Batch mode in the runner copy, with diff tests | 1.5 |
| Per-arm workers, queue and ETA UI | 2 |
| Skip, dedupe and interlocks | 1 |
| Fake-controller tests and hardware measurement | 1.5 |

- Phase 2 (in-process session) adds about 3–4 days.
- CANFD streaming is a separate XL engineering lane, not included.

---

## 5. Cross-cutting notes
- **Upload-channel programs (high level).** See §1.
  - UI v53 = Tk front end. Each button starts one runner process; the UI parses its printed lines and stops it with SIGINT.
  - Runner v14 = CLI back end. One process holds one connection to one controller.
  - `hand.py` = Inspire hand over end-port Modbus. `pole.py` = lift through the SDK lift API. `usage.py` = pole/hand/stop CLI.
  - The current pair is UI v53 + runner v14. v48 is older and contained in v53.
- **Plan-blocking issues (one line each).** See §1: no `--ip/--port`; TypeError in Multi Run / Load Sequence; runner version not enforced (v13 fallback = 100 % transits); SIGINT-only stop (Windows); foreign SDK path; UPLOAD `--yes` skips the overwrite prompt.
- **One vocabulary for the whole product.** Simulation = Lite 3D. Robot dry run = controller SIM mode (arms still; pole and hand always skipped). Real robot = arms move. Use the same words in R3, R4, R5, R10, R11 and R14.
- **Content identity.** sha256 of the canonical point lines (what `write_project` sends minus the header, so LF/CRLF re-saves do not change it) links editing (R2), the simulation pass (R3/R5), upload (R13), real-test verdicts (R4) and the dashboard (R1). Any edit creates a new identity and makes earlier passes stale.
- **One owner per controller.** Lite keeps one lock per arm, shared by upload, run, drag-teach, pole and hand. The Tk control app, the ALIX ROS driver and pendant programs must not drive an arm while Lite does; Lite cannot lock them out, so procedure and a run-state check before every action are needed.
- **Integration path.** The MVP wraps a pinned, Lite-owned copy of runner v14, always passing `--ip/--port/--sdk` and explicit speeds. The opt-in additions are all default off and proven unchanged by diff:
  - `--telemetry` / `FINAL q=`;
  - `--require-mode`;
  - `--expect-size/--expect-name`;
  - `--track-ref`;
  - `--set-global-speed`;
  - the stdin control channel (GO / STOP / end-of-input);
  - the finish-event print;
  - `--upload-batch`.

  The target is an in-process `alix_dispatch` session with a persistent connection, a verified stop ladder and Lite's stop token. Do not port the Tk UI, its dark theme, Multi Run / Load Sequence, or completion decided from status text.
- **Timing.** Expected durations use the measured player law, 0.2322 s + 5.6737 ms × (N−1), ±6 % (`DP/track.py:1558-1570`), plus the transit and a 0.35 s dwell. Do not assume 5 ms per point; the V6 package's 5 ms assumption understates runtimes by about 10 %.
- **Program-ID ownership must be agreed first.** ALIX reserves 70–100, 60, 40–59, 21–39 and 61–69; pendant programs use low slots; V6 uses 71–83; the control app defaults to 25 and 99.
- **Commissioning on the real controllers before operator use.**
  - G4 via `probe_stop_policy` on an in-place dry track;
  - finish event present on complete runs and absent on stopped ones;
  - `plan_speed` read-back;
  - mode read-back and restore;
  - global-speed restore;
  - joint-linear transit;
  - dual start skew;
  - naming model and exact stored size;
  - parallel upload throughput.
- **Effort.** R4 is XL (single-arm MVP L); R13 is M.

---

## 6. Change log (material changes to the analyst draft)

| # | Req | Change | Reason | Evidence |
|---|---|---|---|---|
| 1 | R4 | Completion now needs several witnesses: no blocking code + final pose ≤ 2° + played ≥ 85 % of expected + program-finish event (once commissioned). It is no longer final pose alone. | Silent cuts are documented, and pose-only can mislead on short files. ALIX has hardware-separated witnesses. | `ALIX/plan/FINDINGS.md:9563-9572, 9475-9497, 10086-10091`; `DP/track.py:2383-2390, 1573-1588, 2036`; derived from `V6/LEFT/M3_OPNST02.txt` |
| 2 | R4 | Added a transit collision check: sweep the joint-space line from the current joints to the first point in Lite's collision world; refuse on clash or oversize. | The transit is unchecked by the controller but predictable (movej joint-linear to 0.004°). The runner prints the gap only after an upload in the same process. | `ALIX/docs/PHASE_PLAN.md:186`; `ALIX/plan/FINDINGS.md:5716-5720`; `RUN:207-214, 2692`; `DP/track.py:323-338, 407-410` |
| 3 | R4 | Added the recovery requirement (g): read the joints first, stop all devices, make a checked move to a safe pose one arm at a time; "Run again" is disabled until done. | A stopped arm can sit on the commode, and the next run's transit is unchecked. ALIX hit this as H40 and built a read-first recovery. | `ALIX/tools/dual_arm_flow.py:6180-6188`; `DP/staging.py:921-960` |
| 4 | R4 | STOP redesign: sent once per job; STOP ALL also stops the pole and holds the hand; a stdin control channel (cross-platform, also stops on backend death) replaces the SIGBREAK-only idea; a stop after a "did not start" exit. | A second interrupt can skip the restore in `run()`; v53 already de-bounces. `--run` stops only the arm. The program outlives its client. The 5 s start-failure path issues no stop. | `RUN:238-240, 252-262, 714-737`; `UI:6053-6055`; `UC/usage.py:141-148`; `DP/emulated_program.py:58-62` |
| 5 | R4 | Dual start: a READY/GO handshake on the runner's own pre-start prompt; skew target ≤ 0.2 s is now an acceptance criterion. | v53 synchronises only the process launch; each runner then connects, lists and checks, so the skew is unbounded. | `UI:5693, 5750`; `RUN:189-223, 215-218, 2104-2113`; `maps/12_ops_ui_checklist.md:99` |
| 6 | R4 | Gate tightened: <br>• a failed simulation record cannot be overridden; <br>• mode and slot content are re-checked inside the run process (`--require-mode`, `--expect-size/--expect-name`); <br>• the pole/hand preset is applied by Lite and read back instead of being an operator tick box. | ALIX ladder rule; closes time-of-check gaps; non-technical operators should not attest to machine state. | `DP/ladder.py:120-145`; `ALIX/tools/cleaning_tracks.py:29-43` (B15) |
| 7 | R4 | Dry run: pole and hand are always skipped (open question closed); the mode found is restored; Lite opens in Simulation; a dry run is recommended by default before the first real run of new content. | ALIX: "a SIM arm that dropped the pole for real would be worse than a phantom". The controller-side 4109 from a configuration defect showed up in SIM. The C9 ladder requires a SIM pass or a recorded override. | `DP/runmode.py:134, 225-236`; `DP/session.py:1-10`; `ALIX/docs/PHASE_PLAN.md:139, 144`; `DP/ladder.py:1-10` |
| 8 | R4 | Commissioning reuses ALIX `probe_stop_policy` and the `cleaning_tracks dry` in-place track instead of a new script. Added checks: finish event, `plan_speed` read-back, transit linearity, dual skew. | These already exist in ALIX and have never been run on hardware. The SDK run state has a `plan_speed` field that ALIX never tested as a read-back. | `DP/track.py:3273-3300, 3060-3068`; `ALIX/tools/cleaning_tracks.py:22, 3729`; `SDK/rm_ctypes_wrap.py:2437-2500` |
| 9 | R4 | ALIX Studio borrow narrowed: gate chips and `markDirty` are pattern/CSS only; `do_sim_pose` is per-arm and needs `source: "external"`. | `gateState()` computes coverage, reach, checks and surface for geometric configs (X1). In `auto`, `do_sim_pose` prefers the emulator and computes one arm per call. | `editor.js:15760-15800, 15904-15950, 5371-5429`; `TG/server.py:5344-5393` |
| 10 | R4 | Corrected: the emulator answers the IPs in `RM_LEFT_IP`/`RM_RIGHT_IP`, not only the defaults. It never fires program-finish events. | Code read. | `…/alix_emulator/rm_emulator.py:474-477, 1936-1950`; `DP/track.py:1823-1825` |
| 11 | R4 | Added findings and notes: <br>• the workflow path differs from `--run` (continues on 4105, overall speed 100, transit 30, quick settle not enforced), so pass explicit values and order steps pole → hand → program; <br>• the 4116 translation; <br>• risks from IP swap and the concurrent Tk app. | Code and docs read. | `RUN:1668-1700, 1807-1983, 1949-1980, 2034-2063`; `ALIX/docs/hardware-sdk/ERROR_CODES.md:57` |
| 12 | R4 | Effort L (12–15 d) → **XL** (about 18 d); single-arm MVP is L (about 14–15 d). | Added safety items: transit check, recovery, completion witnesses, runner opt-ins, stdin channel, commissioning. | §3.14 |
| 13 | R13 | Slot policy extended: ALIX 60 (dry) and 101+ (CANFD-only numbers); slots below 40 are pendant territory and slot 1 holds a pendant program; programs Lite did not write are refused in Lite's whole range. | Missed ALIX ranges; the proposed 1–20 range is not guaranteed free. | `ALIX/tools/cleaning_tracks.py:2755-2782, 2858-2870`; `DP/emulated_program.py:333-335` |
| 14 | R13 | Corrected the naming risk: the model was measured on hardware by ALIX (2026-09-10, several programs) and seen in the control-app session, not "observed once". A name mismatch now means re-send, never skip. Names carry a content-hash fragment, because size alone cannot see same-length edits. | Code and comment read. ALIX treats the name as advisory when the model is unconfirmed. | `DP/emulated_program.py:326-341`; `UC/README_COMMANDS.md:51`; `ALIX/tools/cleaning_tracks.py:4116-4133` |
| 15 | R13 | Added the R12 interplay: 10 of 15 V6 files exceed joint ratings at 5 ms (peak 2.18×); fast files overshoot 4–20°. Engineer overrides must be recorded, and the gates need a limit margin. | Derived from the files; a strict gate would block the current package. | python over `V6/*/*.txt`; `ALIX/plan/FINDINGS.md:10318-10322` |
| 16 | R13 | Throughput model scoped: the fit (0.24 s + 21.2 µs/B) includes the LF-rejected but fully transferred upload; measured range is 180–325 KB; larger files are extrapolated. | Recomputed from the run records. | `ALIX/runs/2026082{8,9}T*_upload*/run.json` |
| 17 | R13 | Added an open item on the size tolerance: `max(64 B, 0.1 %)` versus exact. | About 10 points could go missing unnoticed on a 600 KB file; the one hardware example matched exactly. | `RUN:171`; `UC/README_COMMANDS.md:51`; `DP/track.py:367-374` |
| 18 | R13 | CANFD citation corrected: the taught CANFD pipeline is scoped to the left arm. | The original claim cited a map, not a source line. | `ALIX/docs/motion/TAUGHT_CANFD_PIPELINE.md:10` |
| 19 | R13 | Added that v53 UPLOAD passes `--yes`, skipping the runner's slot-content prompt, so Lite must build its own check for programs it did not write. Removed the dialog-text bug mention. | Plan-relevant; the user asked for no bug audit. | `UI:4275-4290`; `RUN:148-157` |
| 20 | Cross | Corrected: `pole.py` drives the lift through the SDK lift API, not Modbus. Only the hand is on Modbus. | Code read. | `UC/pole.py:22-58`; `UC/hand.py:1-17` |
