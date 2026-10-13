# R8 · TXT file merging — verified requirement analysis (cluster C5_merging)

ALIX Studio Lite · requirement R8 · adversarial verification of the C5 analyst draft · 2026-10-13.
Read-only review: no robot, no network, no test suites, no servers. Nothing in the repositories was changed.

**Conventions.** Evidence is cited as `path:line`. **[M]** marks numbers measured here, read-only, with `python3 -I` fed on stdin (no files written). **(inferred)** marks a conclusion that the code or data do not show directly.

| alias | absolute path |
|---|---|
| `WS/` | /home/newtonjeri/alix_studio_lite_ws/ |
| `UC/` | WS/upload_channel/ |
| `V6/` | WS/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/ |
| `HTML` | WS/Alix_Studio_Lite_Trajectory_V2.html |
| `UI53` | UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py (current control UI) |
| `RUN14` | UC/rm_upload_fast_v14_safe_transit.py (current runner) |
| `ALIX/` | /home/newtonjeri/alix_ws/src/alix/ |
| `TG/` | ALIX/core/python/alix_taskgen/src/alix_taskgen/ (**ALIX Studio**) |
| `WSP/` | ALIX/core/python/alix_workspace/src/alix_workspace/ |
| `DP/` | ALIX/core/python/alix_dispatch/src/alix_dispatch/ |
| `EXP01/` | ALIX/experiments/exp01_teach_trajectory_reverse_engineering/ |
| `TE.hpp` / `TE.cpp` | ALIX/ros/orchestration/alix_tasks/include/cleaning_tasks/common/stroke/track_emit.hpp / .../src/common/stroke/track_emit.cpp |
| `TT` / `CT` / `DAV` / `DAF` | ALIX/tools/taught_tracks.py / cleaning_tracks.py / dual_arm_verdicts.py / dual_arm_flow.py |

---

## 1. Bottom line

- **Can R8 be borrowed from ALIX Studio? Partial — borrow some, build the rest.**
  - ALIX Studio has no TXT merger, no bridge (link-move) generator, no Program-ID handling and no `.txt` track I/O. Nothing anywhere in ALIX merges taught recordings into one program.
  - What the Studio does have is what you need to **check** a merge: the full offline collision world with the commode in its three states (`BodyCollisionWorld`), the "no world = not checked" convention, sentence-style sequence verdicts, and the Dual-Arm Collisions playback.
  - That collision world needs **real changes** before it can judge untaught motion. It has no strict (no-contact-allowed) mode. It takes the hand only as a named state, always poses the bin lowered, and silently ignores anything that already touches at the rest pose. It also cannot see the arm base hitting the stowed bin.
- **The input note ("there exists a mechanism … that takes into account the collision scene (commode)") is partly accurate.**
  - Join mechanisms do exist, with tests: `EXP01` `plan_to_track.concatenate`, `ALIX/tools/plan_min_time.concatenate`, and the C++ planner's track grouping.
  - The commode collision scene also exists.
  - But no join checks collisions. Their commode safety came from MoveIt planning the joined stages. They refuse the gaps that taught recordings have.
  - The V6 package was merged offline by a tool that is not on disk, and its own README says it is not collision-checked.
- **Recommendation.** Build a **Program Builder** in Lite. It takes an operator's ordered regions and builds the fewest safe controller programs:
  - It splits wherever the pole, hand, toilet state or sync rule changes.
  - It inserts sized minimum-jerk link moves, which reproduce the V6 bridges exactly.
  - It makes every program start and end at rest.
  - It gates each program with ALIX's hardware-proven rules.
  - It collision-checks every link move, controller transit and pole/hand change, plus every cleaning row, in an adapted `BodyCollisionWorld`.
  - It writes the merged `.txt`, a provenance report, slowed test files for the first hardware run, and the per-arm workflow JSON that the current control app (UI v53 + runner v14) already runs.
- **What this verification changed** (details in §13):
  1. **Rating gate.** The rating gate must use ALIX's 20 Hz band-limited rate (G2). On that measure **2 of 15** V6 files fail, not 10 of 15. The raw tick ripple is reported as information.
  2. **R10_HNGR.** Its J4 margin is 1.53°, below ALIX's 2° hard floor, so it is a **hard block**, not an override case.
  3. **Strict mode.** `BodyCollisionWorld` has no strict mode. Untaught motion needs a small change to a vendored copy.
  4. **Stowed-bin foul band.** This band (153–282 mm) cannot be detected by the collision world, so it must be a rule.
  5. **First runs at reduced speed.** Turning the speed setting down does not slow a stored program, so Lite must produce slowed test copies.
  6. **Program boundaries.** V6 programs start at up to ~55 °/s and stop from up to ~67 °/s, so every program must start and end at rest.
  7. **Blocked link moves.** A blocked link move is not fixed by "slower" or "separate programs": both follow the same straight joint line. Lite must offer a detour through a checked safe pose.
  8. **Arm-vs-arm check.** Programs that run TOGETHER need the arm-vs-arm check before they are READY (Must).
  9. **Control-app constraints.** Three things shape merged programs: the 5 s start window against a start delay that grows with program size, error 4105 being treated as warning-only, and no stored-size check before a run.
- **Priority: Must**, for the single-arm core plus the dual-arm clearance check of TOGETHER programs. Automatic dual time matching is **Should**.
- **Effort: XL, about 23 person-days.** That is about 20 pd for the Must part and about 3 pd for the Should part. Add about 5 pd if the shared collision worker and 3-D preview (R3/R5) are not built first.

---

## 2. The requirement

**Original text** (`WS/alix_studio_lite/docs/reference/ALIX_Studio_Lite_requirement_inputs.txt:22`):
> 8. TXT File Merging: The software should provide an option to merge multiple trajectory TXT files into a single executable trajectory. - There exists a mechanism, provided takes into account the collision scene (commode) in alix workspace in the upload channel tests and experiments

**Revised requirement (testable).** Lite shall let an operator put an ordered list of taught region `.txt` files for one arm into the fewest controller programs that are safe to run. The inputs are normally R2-optimised. Each program is a single `.txt` file. Lite also builds the per-arm workflow that runs those programs in order.

- **(a) Grouping.** Two regions share a program only if all of these hold:
  - they are for the same arm;
  - they need the same pole height;
  - they need the same 6 hand values;
  - they need the same toilet state (lid closed / lid open / seat open);
  - both are cleaning units with the TOGETHER sync rule.

  A region whose toilet state is "any" takes the state of the program it joins. Mechanism, tool, glove and serial (RUN FIRST / WAIT OTHER) units always get their own program.

  At every change Lite starts a new program and puts the pole, hand or checkpoint step between the programs. It says why in plain words.
- **(b) Joins.**
  - A gap under 0.001° drops the duplicate point.
  - Contiguous rows of one source recording join directly.
  - Any other gap gets a generated **link move** (bridge). By default this is a rest-to-rest minimum-jerk move along the straight joint line. It is sized so that, on the written integers at the 5.0 ms tick:
    - its peak joint speed is ≤ the link-move speed (default 30 °/s);
    - its peak joint speed is ≤ 0.97 × each joint's rating;
    - its nominal peak acceleration is ≤ 300 °/s².

  When a link move or controller transit is blocked, Lite offers a detour through a checked safe pose. It never offers "slower" as a fix for a geometric collision.
- **(c) Program ends.** Every program starts and ends at rest: ≤ 5 °/s over its first and last 10 ticks (a proposed threshold). Otherwise Lite adds a short ease-in or ease-out, or flags the region end for trimming.
- **(d) File gates.** These are judged on the written integers, with code shared with R12:
  - **Format:** 7 integer millidegrees per line, at least 2 lines.
  - **Step:** ≤ 10° per line.
  - **Rating (ALIX G2):** no tick over a joint rating at 5.0 ms, judged on the 20 Hz band-limited rate. Raw tick counts are reported for information.
  - **Joint limits (ALIX rule):** inside 5° of a URDF limit, the band-limited rate must stay ≤ 10 °/s. A margin below 2° is a hard block.
  - **Size:** at or under the point cap (configurable; see criterion 15).
  - **Name:** the controller's stored name `"<id>_" + [DT_] + stem + ".txt"` is ≤ 32 characters.
  - **Run time:** predicted run time < the runner timeout (300 s).

  Only G2 and the 2–5° near-limit rule may be overridden. The override must be made by an engineer, with a reason recorded in the report.
- **(e) Collision check.** This runs as a background job in one worker process. The scene is:
  - the commode body, lid and seat posed in the program's toilet state, at calibrated lid and seat angles;
  - the walls of the calibrated cell, or none;
  - the bin in its declared state;
  - the UGV and poles;
  - the robot itself;
  - the partner arm at its known pose.

  Lite checks:
  - every link-move point;
  - every controller transit: the straight joint line from the previous program's last point, at the new pole height, sampled at ≤ 2°;
  - every pole or hand change, with the arm held still;
  - every cleaning row.

  **Untaught motion** (link moves, transits, device moves) uses a strict check with no contact allowance. Lite reports its minimum clearance to the toilet and walls; below a margin (proposed 20 mm) it is NEEDS ATTENTION.

  **Cleaning rows:**
  - The active hand touching the region's own surface is **expected contact**. The depth is reported, and more than 20 mm is NEEDS ATTENTION.
  - Arm-link contact is a **collision**.

  **Stowed bin.** With the bin stowed, the 153–282 mm pole band is enforced as a rule.

  **Scene plausibility.** Before any verdict counts, the **raw** recordings of the regions are run through the same scene. If a raw recording shows arm-link contact, the scene is wrong, and every verdict in it is NOT CHECKED.

  **Verdicts.** A program with a collision is BLOCKED. A program whose check did not run, or whose scene is incomplete, is NOT CHECKED, never READY.
- **(f) Dual-arm clearance (Must for TOGETHER programs).** A program that runs TOGETHER is not READY until an arm-vs-arm check of its whole block has passed. The block includes both arms' start transits and pole moves. The check requires ≥ 25 mm over the predicted timing spread, and is shared with R10. If it fails, Lite proposes SERIAL.
- **(g) Outputs.**
  - the merged `.txt` (LF NDJSON, integer millidegrees);
  - a report with provenance: inputs with sha256 and row ranges, link moves, gate values, scene, verdict and versions;
  - per-arm workflow JSON (`realman_workflow_sequence_v1`) with Program IDs from Lite's declared range;
  - for every new link move, a slowed test file for the first hardware run.

  Any change to an input file, its metadata, the scene calibration or the engine version marks the program **out of date** and blocks upload.
- **(h) Preview.** The result plays in the Lite simulation view with link moves and transits marked. A reported collision jumps the view to its sample.
- **(i) Should.** Lite matches the timing of TOGETHER blocks automatically, by holds or stretch. It uses the runner's own pole and hand step times and measured start and transit times, and shows the predicted skew band.

**Clarifications and assumptions**
1. **"Single executable trajectory"** means one controller program (one `.txt` in one slot) per run of compatible regions, plus the workflow that sequences the programs. One file for a whole arm's day does not work:
   - **Joints only.** A stored program carries only the 7 arm joints (`RUN14:14-15`, `{"file":7}` at `RUN14:91-97`). Pole, hand and lid changes cannot happen inside it. ALIX's planner has the same rule: "a hand or pole stage ends the group" (`TE.hpp:145-148`).
   - **Size.** A whole V6 arm day is 45,401 (L) / 41,053 (R) points, about 2.6 / 2.4 MB [M]. That is 3.5× the largest program known to have been uploaded (12,864 points, §4.5).
   - **Not the 300 s timeout.** The draft gave the timeout as a reason, but it is not one: the L day would play for about 250–265 s by ALIX's clock band, extrapolated [M].
2. **Merge vs sequence.** *Merging* concatenates points into one file, with generated link moves. *Sequencing* is the workflow JSON (program / pole / hand / wait steps), which the control app already runs. R8 produces both.
3. **What merging buys (inferred).**
   - **Saving per join:** the runner's 0.6 s program-to-program settle (`RUN14:1947-1955`), the ~0.35 s fixed part of the controller's start delay (`DAV:221`) and the transit, minus the link-move time.
   - **No saving:** the per-point part of the start delay (0.41 ms/pt) does not shrink.
   - **Real benefits:** fewer steps, and the ability to keep one arm busy, by a hold, while the other arm's pole moves. That second one is V6's use (`V6/README.txt:37-39`).
   - **Costs:** a bigger re-upload whenever one region changes (~1.25 ms per point, map 06 §1.4), and coarser restart points after a stop.
4. **"Collision scene (commode)"** means:
   - the commode body, lid and seat at the right articulation;
   - the cubicle walls;
   - the robot's own bin, UGV and poles;
   - the partner arm.

   These are the objects `TG/body_collision.py:1-40` assembles.
5. **"Executable"** means four things: the file passes the runner's format check (`RUN14:76-89`), the gates in (c)–(d) and the checks in (e)–(f), and it is uploaded by the existing runner (R13).
6. **Clock policy.** Rates are judged at the 5.0 ms tick (the fastest the player advances), using ALIX's 20 Hz band-limited rate (`CT:1646-1672`, `TT:420-456`). Durations are shown as the player band `0.057 + 5.515 ms × (N−1)` … `0.750 + 5.810 ms × (N−1)` s (`DAV:172-176`).
7. **Inputs are R2 outputs.** R2 must also hand over a **row map back to the raw recording**, which the scene-plausibility check uses. Two parts that are contiguous rows of the same recording join without a link move.
8. **"Region"** means one taught unit, e.g. `L-R05-P1` (map 13 §5.2). The HTML's equal-index "regions" are R2 sub-regions and play no part in merging.
9. **UI terms:**
   - **program** = one controller slot;
   - **link move** = generated motion inside a program;
   - **transit** = the controller's own joint move to a program's first point, outside the file, at the workflow's Start %.
10. **Control app scope.** The upload-channel programs (UI v53 + runner v14) are described only at a high level (§3), as the user asked. Only constraints that block this plan are named.

---

## 3. The upload-channel programs at a glance (high level, as asked)

- **What is current.** UI v53 (`UI53`) with runner v14 (`RUN14`) is the current pair.
  - v53 looks for the v14 runner first (`UI53:6891`).
  - The three v14 copies in `UC/` are byte-identical [M md5].
- **UI v53** is a Tkinter desktop app for operators. It covers:
  - single programs, uploads and drag-teach;
  - Page-02 workflows, made of program / pole / hand / wait / global-point steps, for one arm or both.

  It never talks to the robot directly. For every action it starts one runner subprocess per arm and follows its printed lines.

  For both arms, the "SAFE TIMELINE" rules apply (`UI53:9666-9690`):
  - consecutive TOGETHER rows of an arm become one runner document;
  - the two arms are released together only at block start (`threading.Barrier(2)` just before the subprocess launch, `UI53:5654-5660, 10209`);
  - a positive WAIT on both arms becomes a manual lid checkpoint dialog;
  - RUN FIRST / WAIT OTHER is a serial hand-off that needs the same Program ID on both arms (`UI53:9966-9972`);
  - IDs must be unique per arm (`UI53:9700-9710`).
- **Runner v14** is a command-line tool that drives one arm's controller through the RealMan SDK. It can:
  - upload a `.txt` as a stored program: a CRLF `.proj`, verified by the stored size (`RUN14:91-97, 147-175`);
  - run stored programs;
  - run whole workflows over one connection (`RUN14:1986-2277`);
  - move the pole and hand through `UC/pole.py` and `UC/hand.py`;
  - record drag-teach;
  - set the SIM/REAL run mode.

  It restores the controller's global speed at the end.
- **How R8's output runs:** Lite emits the per-arm `realman_workflow_sequence_v1` JSON, and the runner validates it with `normalize_workflow_step` (`RUN14:1668-1763`).
- **Constraints that block or shape this plan** (one line each):
  1. TOGETHER blocks are synchronised only at launch (`UI53:10209`). A mid-block re-sync needs a new step type.
  2. A program must report "running" within **5.0 s**, or the runner gives up on it (`RUN14:238, 658, 1909`). ALIX's start delay grows 0.41 ms per point (`DAV:221-224`), which extrapolates to ~4.9–5.8 s for a 10,983-point program. Measure this before allowing large merged programs.
  3. Error 4105 ("joint speed over its limit") is **warning-only** (`RUN14:56-57`), and "run_state 0" counts as complete. ALIX saw 4105 latch and end a program at 58 % (`CT:211-219`). So a cut-short program can read as complete, and the next pole/hand step then runs from the wrong pose. Lite must enforce G2 before upload.
  4. A workflow runs a slot by ID. It only refuses empty slots (`RUN14:2017-2031`) and never checks that the slot still holds the checked bytes. ALIX compares stored size and name before a run (`CT:39-43`).
  5. The run and "overall" speed scale only the transit, not playback. Point spacing is the speed (`DP/track.py:7-11, 21-24`). A slow first run needs a slowed file.
  6. The lid checkpoint is implicit: a positive WAIT on both arms (`UI53:9676-9690`). Lite must emit it deliberately, or an explicit checkpoint type must be added.
  7. The 300 s timeout (`RUN14:2342`) does **not** bind programs below roughly 45,000 points (inferred from the clock band). Keep the run-time check anyway.

---

## 4. Verified findings

### 4.1 Lite workspace and the V6 package
- **No merge code in Lite.** Nothing under `WS/` merges, bridges or concatenates files.
  - Grep hits are only "emergency" strings in old UIs (v8, v13, v17, v18) and a v52 docstring. v53 has none [M].
  - The trajectory HTML loads one file at a time (`HTML:349-352`).
- **The V6 method is documented but has no generator.**
  - Partner segments were stretched "by adding/interpolating points along the same recorded joint path" (`V6/README.txt:4-8`).
  - A pole move forces a split (`V6/README.txt:33-39`).
  - IDs were shifted +70 (`V6/README.txt:101-104`).
  - The README states: "NOT collision-checked against the toilet, cubicle walls, second robot, end-effector, or right-arm camera" (`V6/README.txt:48-56`).
- **V6 bridges are exactly rest-to-rest minimum-jerk moves [M].**
  - All 11 `BRIDGES_TEST_ONLY` files reproduce with **0 mdeg error** as `round(q0 + s(k/(n−1))·(q1−q0))`, with `s = 10τ³−15τ⁴+6τ⁵` and `n = T/5 ms + 2`. Rounding mode makes no difference.
  - Each bridge is embedded verbatim in its merged file, and its end points are the neighbouring segments' own rows, so there are no duplicate rows.
  - Peak speed 1.875·gap/T matches `V6/BRIDGE_REPORT.csv:2-12`.
  - **Correction to the draft:** the 11 bridges fall into three groups:
    - **4 long timing holds:** R1→R2 0.44°/1.25 s; R2→R3 0.24°/1.38 s; L3→L4 0.86°/1.38 s; L6→L7 0.006°/1.6 s.
    - **4 short joins:** L10→L11 1.07°/0.25 s; R10→R11 0.09°/0.25 s; L11→L12 0.67°/0.2 s; R11→R12 0.08°/0.2 s.
    - **3 real moves:** L4→L5 3.9°; L1→L2 16.7° at 25.0 °/s; R6→R7 39.1° at 45.8 °/s.
- **The README does not say V6's 45.8 °/s bridge was hardware-tested.** It says its speed is "approximately the same as the previously hardware-tested smooth bridge", and asks for the bridge files to be tested first (`V6/README.txt:43-46, 53-56`).
- **Join smoothness [M].** Bridges start and end at rest while the segment ends are still moving.
  - The one-tick speed step at a join is up to **37.8 °/s** (R11→R12), and 31.2 °/s at L1→L2.
  - Inside the taught segments the same measure is already p99 66.8–172.6 and p99.9 83.3–250.9 °/s.
  - So a join gate for taught data must be relative: a warning, not ALIX's planned-track 10 °/s-per-tick rule (`CT:227-231`).
- **New: V6 programs start and stop while moving [M].**
  - **Starts at speed:** R_P10P12's first ticks command 35–72 °/s, right after the controller's transit stops at rest. R_P01P03 starts at ~40 °/s.
  - **Ends at speed:** M3_ASOPN01 ends at 31–67 °/s, R_SPILLAGE at 21–44 °/s, R10_HNGR at 21–44 °/s and R_P06P07 at 18–38 °/s.
  - The player does not smooth: "the file's point SPACING is the velocity profile" (`DP/track.py:7-11`). These are step starts and stops at program boundaries.
- **Unchecked controller transits between V6 programs [M]:**
  - LEFT M3_OPNST02 → L_P10P12: **64.60° (J5)**.
  - LEFT L_P10P12 → R15_BUPFUL: 17.25° (J5).
  - RIGHT R2_LIDTOP → R10_HNGR: **45.70° (J1)**, done after a 150 → 50 mm pole step.
  - RIGHT M3_ASOPN02 → R_P10P12: 13.65° (J6).
  - All other joins are ≤ 0.35°.
- **Pole timing.**
  - V6 assumes 1.38 s per 100 mm (`V6/README.txt:10-15`).
  - The runner's pole step (`manual_pole_control`, `RUN14:865-909`) calls `Pole.move` at 100 %. That returns only after `elapsed ≥ 1.5 s + d/slew` (`UC/pole.py:11-14, 53`), i.e. **3.19 s up / 2.53 s down per 100 mm**, whatever the physical pole does.
  - So TIMELINE-1 starts about 1 s skewed: L 200→150 takes 2.02 s and R 200→50 takes 3.05 s [M].
  - Map 10 §3.4 models +2.69 s of skew by P5.
- **Gates on V6 (corrected) [M].**
  - **Raw per-tick rating, 10 of 15 files over.** The worst is R_P06P07: 172 ticks, first at tick 288, peak 2.454°/tick = 491 °/s on J6.
  - **ALIX's G2, 2 of 15 files over.** G2 judges the 20 Hz band-limited rate (butter(4), filtfilt) at 5.0 ms, i.e. the gate ALIX enforces (`CT:1665-1672`, `TT:420-456`). The two failing files:
    - **R_P06P07:** 102 ticks, first at tick 2851, peak 312 °/s on J6.
    - **R_P10P12:** 29 ticks, first at tick 2941, peak 243 °/s on J4.
  - ALIX records the raw count beside G2 because "a drag-teach file's ripple spikes are over a rating tick by tick even when the band-limited rate is not" (`CT:1657-1661`).
- **Joint margin [M].**
  - R10_HNGR reaches J4 133.47°, a **1.53°** margin. It sits inside 5° of the limit for 492 rows at up to 24.3 °/s band-limited.
  - ALIX's rule: inside the 5° allowance the rate must be ≤ 10 °/s, and below 2° it refuses (`TT:126, 172-173`). R10_HNGR fails both.
  - ALIX has seen J4 overshoot its command by 4.5–5.9° (map 06 §2.2), so this file is likely to trip on hardware (inferred).
  - Every other V6 file keeps ≥ 7.1°.
- **Missing pieces and IDs.**
  - `DUAL_FLOW_TIMELINE_V6.json` is listed (`V6/SHA256SUMS.txt:14`) but absent.
  - `manual_checkpoint` exists only in prose (`V6/README.txt:64-67`).
  - V6 IDs 71–83 lie in ALIX's reserved production range 70–100 (`ALIX/Resource/tracks/taught/README.md:162-168`).
  - The controller stores names as `(f"{id}_" + basename)[:32]` (`ALIX/Resource/tracks/taught/README.md:150-160`).
  - **New:** Lite's controller IPs (192.168.1.10 / .103) equal ALIX's defaults (`DP/connect.py:31-32`). Lite's own README uploaded to **slot 85** using the ALIX SDK path (`UC/README_COMMANDS.md:44-51`). Treat the controllers as shared until told otherwise.
- **V6 files are CRLF** on every line [M]. A "byte-for-byte" golden test against Lite's LF output would fail; compare parsed rows instead.

### 4.2 ALIX joining primitives (they work on planned motion only)
- **`plan_min_time.concatenate(parts, names, max_gap_deg=0.5)`** (`ALIX/tools/plan_min_time.py:142-169`):
  - refuses a boundary gap over 0.5° with `SystemExit`;
  - drops a duplicate boundary point under 1 mdeg;
  - records each stage's span;
  - parameterises each stage with zero boundary velocity;
  - works in radians.
- **`EXP01` `plan_to_track.concatenate(stages, clock_s, max_gap_deg=0.5)`** (`EXP01/src/plan_to_track.py:100-136`) has the same contract:
  - it drops only an exact duplicate (< 1e-9°);
  - it turns `hold_before_s` into repeated points;
  - its CLI joins `move_to_start,execute_path,move_to_rest`, i.e. "the plan's own collision-planned approach and retreat" (`:143-151`);
  - its tests cover continuity, refusal, dwell and reordering (`EXP01/tests/test_plan_to_track.py:120-160`), **not collisions**.
- **The C++ planner grouping** (`TE.hpp:145-170`; `TE.cpp:378-453`; tests `test_track_emit.cpp:129, 262`):
  - contiguous track-channel arm stages form one track;
  - any hand or pole stage ends the group;
  - `pre_stroke` records the last pole and hand values;
  - joins under 1e-3° drop the duplicate point;
  - it is advisory only: "Nothing here refuses" (`TE.hpp:29-33`);
  - it writes the `.txt` as LF and only the `.proj` as CRLF (`TE.hpp:20-26`).
- **Where the commode awareness comes from.** These stages are MoveIt plans built against an articulated commode scene (`ALIX/ros/orchestration/alix_tasks/src/common/scene/articulated_scene_builder.cpp:176-199, 612-625`).

### 4.3 ALIX taught lane and dual executive (sequencing, never merging)
- **Each taught slot is its own program.** The order is: pole, then hand, then rest, then the move to the stroke start, then the stroke, then rest (`ALIX/docs/motion/TAUGHT_PIPELINE_README.md:90-97`).
- **The commode C recordings all begin at one shared start pose.** "the rest pose IS the stroke's start, the drag start pose every recording began at" (`:95`). So in ALIX there is almost nothing to bridge. The V6 sources were instead taught as a chain: each recording starts where the previous one ended (≤ 0.35° between most V6 programs [M]).
- **Rest legs and bridges are not checked.** They are joint movejs, "NOT collision-checked" (`CT:148-158`; `ALIX/docs/motion/TAUGHT_CANFD_PIPELINE.md:563-565`).
- **Device rules.** ALIX opens the hand before a pole move and retreats "so no device moves with the pad on the surface" (`CT:145-148`).
- **The dual executive** (`DAF:19-47`):
  - has four step classes: dual, serial, single and mechanism;
  - allows concurrency only where the windowed clearance is ≥ 25 mm;
  - carries each arm to its start "one arm at a time with the partner read and static", which is "collision-UNCHECKED" (`DAF:3763-3800`, start tolerance 2°).
- **v53 is different.** It starts both arms' transits at the same moment (`UI53:10209`).

### 4.4 Collision-scene building blocks: what works and what must change
- **`BodyCollisionWorld` (ALIX Studio) builds the full offline world:**
  - the URDF robot (both arms, hands, poles, UGV, bin and covers);
  - the venue walls;
  - the commode body, seat and lid at `closed` / `lid_open` / `lid_seat_open` (`TG/body_collision.py:1-40, 104-108, 401-464`).

  `make_pose_filter(config, fixture, partner_state, pole_override_m)` returns blocking pairs plus `.depths()` (`:499-952`).
  - **Cost:** about 2–3 ms per pose; map 05 measured a 1.96 ms median and a 2.6 s build.
  - **Validated** only for `commode_c` (`:58`).
  - **Locking:** one RLock serialises access (`:172`).
  - **Weight:** the package `__init__` is empty and the module imports only light scene helpers, so borrowing it drags in **no** X1/X2 generator code [M]. It does need the repo layout: root marker, venue and commode YAMLs, `contact_links.yaml`, meshes (map 05 §7).
- **Correction to the draft: there is no strict mode.**
  - With no `allow_collision` stage, `contact_surface` defaults to **all** commode parts (`:595-596`).
  - A frame with no contact-link set allows **every** hand link (`:496, 866`).
  - So hand-vs-toilet contact during a link move would be silently allowed. Lite needs a small change in a vendored copy, e.g. `strict=True` skips the allowance block.
- **Hand state is a named SRDF state only** (`:582-591, 665-667`). Lite's 6 hand counts must be converted (`ros/hw/alix_hw/scripts/hand_units.py`, map 05 §1.4). The converted joints are then injected, either through `partner_state["joints"]`, which is a plain dict merge into the home pose (`:698-701`), or through a vendored parameter.
- **Bin.** The bin is always posed BIN_LOWERED (`:629`).
- **Rest baseline.** Every pair present at the SRDF rest pose is subtracted from all later queries (`baseline = set(_pairs(None)[0])`, `:871`). Lite must show a non-empty baseline as NEEDS ATTENTION.
- **New: the stowed-bin pole foul is invisible to this world.**
  - The arm's `base_link` "moves with the pole alone" (`TG/studio_generate.py:472-475`).
  - The world splits links into "moving" (depends on the active arm's joints) and "static". `base_link` and the bin covers are both static (`:251-255, 264-277`), and static-vs-static pairs are never queried.
  - Lite must enforce the measured band (0.153–0.282 m, `TG/web_editor/editor.js:17144-17149`; orchestrator source `_STOWED_POLE_FOUL_M` ± 5 mm) as a **rule**.
- **No floor object** is loaded (map 05 §4.3). Floor-spillage regions cannot be checked against the floor.
- **"No walls" needs a venue file.** `load_wall_manager` raises when the walls are disabled (`WSP/plan_scene_validation.py:125-127`).
- **The Studio's DAC endpoint is not usable for R8.**
  - It calls `compute_dual_arm_plan_collisions(left, right, dt, window, return_frames=True)` with **no** commode, walls, hand states or pole (`TG/server.py:4335-4366`).
  - Its default dt is 0.05 s, while ALIX treats 0.02 s as the correctness setting (`DAV:356`; map 05 §4.5).
- **The library underneath is usable.**
  - It supports `extra_managers` (`WSP/dual_arm_plan_collision.py:596, 699-703`).
  - Its slim link set excludes fingers (`:164-182`).
  - Unseeded joints default to pole 0.300 m and open hands, which puts an arm 150–225 mm off (`:385-452`).
  - `compute_plan_frames(tracks=[{plans, start}])` can preview per-arm **chains** (`:487-583`). Its `_concat_timelines` jumps across program boundaries rather than sweeping transits (`:468-484`).
- **`approach_transit`.**
  - Its `_steps` rule samples a straight joint line at ≤ 2° with at least the midpoint (`TG/approach_transit.py:60-73`).
  - `transit_verdict` returns `ok=None` with no world (`:178-200`).
  - `transit_blocked` itself is tied to the rest → pre_start → start legs (`:54-57, 95-119`). So Lite borrows the **rule**, not the function.
  - `corridor_witness`, a tuck-family detour search (`:122-158`), is a useful idea for blocked link moves.
  - A rest-to-rest minimum-jerk move and the controller's synchronised movej both follow the straight joint line (inferred), so sampling that line covers both, at the sampling resolution.
- **Sequence guards** speak in sentences. A pair with no verdict is "stale", never passing (`TG/schedule_guards.py:1-30`). The editor refuses a pair whose plans need different fixture states (`TG/web_editor/editor.js:14332-14353`).
- **The `.txt` → plan shim and arm-vs-arm gate** (`DAV`):
  - millideg × π/180000 at the 5.5545 ms clock;
  - **mandatory** pole and hand seeding (`:22-33`);
  - a swapped-arm guard;
  - a windowed `cross_arm_separation` at dt 0.02 against 25 mm (`:356, 369`).

  It checks arm vs arm only, and a windowed sweep takes "minutes to tens of minutes" (`:67-72`).
- **`classify_pair` mislabels bin covers.** It tests fixture tokens first (`WSP/redundancy.py:90-114`), so a bin `lid_opening_cover` pair is named as a toilet part ("lid"). Lite needs its own plain-name table.
- **`WSP/clearance.ClearanceProbe`** gives a minimum distance to the commode. But `load_fixture` omits the venue rotation (`WSP/clearance.py:49-95`), so open lid and seat states are posed wrong (the G37 note at `TG/body_collision.py:439-447`). Use FCL distance on `BodyCollisionWorld`'s fixture manager instead.

### 4.5 Timing and size evidence
- **The player clock varies with length.** Replay lies within `0.057 + 5.515 ms·(N−1)` … `0.750 + 5.810 ms·(N−1)` s, measured only for 588–4,281-point tracks. The code refuses to extrapolate (`DAV:172-176`).
- **Start delay.** `0.351 s + 0.41 ms/pt·N`, residual −1.07…+0.99 s (`DAV:221-224`). This was measured with ALIX's launcher. Lite's runner adds its own subprocess start, SDK load, connection and list read after the barrier (`UI53:5654-5660`), which is unmeasured.
- **Most V6 TOGETHER programs are beyond the measured range [M].** They are 3,533–10,983 points, and 9 of 11 exceed 4,281.
- **Clock spread alone.** For a 10,000-point program it is about ±1.8 s [M]. Equal point counts do not keep the arms in step.
- **Largest known uploads.**
  - 12,429 points / 713 KB on the Lite left controller, upload verified; no run output is shown (`UC/README_COMMANDS.md:45-51`).
  - 12,864 points / 750,251 B in ALIX's registry, row status EMITTED; played is inferred (map 06 §1.5).
  - No documented maximum exists.
  - Upload time is about 0.24 s + 21.3 µs/byte, so a 12,000-point program takes ≈ 15 s.

---

## 5. Input-claim check

**Claim:** "There exists a mechanism, provided takes into account the collision scene (commode) in alix workspace in the upload channel tests and experiments."

**Verdict: partly accurate.**
- **Accurate:**
  - Joining mechanisms with tests exist in ALIX's upload-channel work: `EXP01/src/plan_to_track.py:100-151` (tests `EXP01/tests/test_plan_to_track.py:120-160`), `ALIX/tools/plan_min_time.py:142-169`, and the C++ grouping `TE.cpp:378-453` (tests `test_track_emit.cpp:129, 262`).
  - ALIX also holds the commode collision scene: `BodyCollisionWorld` (`TG/body_collision.py`), `extra_managers` in `WSP/dual_arm_plan_collision.py:596`, `PlanWallChecker`, and `WSP/clearance.py`. The last one poses open states wrongly.
- **Not accurate as a ready merge for taught `.txt` files:**
  1. The joins concatenate stages of **one** MoveIt plan that already meet within 0.5°. Their commode safety comes from MoveIt planning against the articulated scene, not from the join, and they refuse taught-recording gaps. Their tests check continuity, never collisions.
  2. No ALIX tool generates link moves between taught recordings, or collision-checks link moves, controller transits or pole moves. ALIX's own flows label those moves "NOT collision-checked" (`CT:148-158`; `DAF:3763-3800`).
  3. ALIX's `.txt` collision shim (`DAV`) measures arm vs arm only. The Studio's sweep endpoint passes no commode.
  4. The Lite workspace has no mechanism at all. V6 was made offline by a tool that is not on disk and is explicitly not collision-checked.

So the mechanism must be **assembled**: the join contract and grouping rule from ALIX's planner, a new link-move generator, an **adapted** collision world (strict mode, hand counts, bin state, baseline, foul rule), and ALIX's gates.

---

## 6. Borrow verdicts

### 6.1 ALIX Studio (alix_taskgen): **Partial — borrow some, build rest**

| Studio piece | Use in R8 | Verdict |
|---|---|---|
| `BodyCollisionWorld`, `fixture_manager`, `make_pose_filter` (`TG/body_collision.py:130-990`) | per-pose check of link moves, transits, device moves and cleaning rows against the commode (in its state), walls, bin, UGV and poles, and the partner | **Borrow with adaptation (vendor + change).** Needed changes:<ul><li>strict mode (`:595-596, 866`);</li><li>active-hand joints from counts (`:582-591, 665`);</li><li>bin state (`:629`);</li><li>surface the rest baseline (`:871`);</li><li>calibrated venue, standoff and lid/seat angles;</li><li>a no-walls venue;</li><li>one worker process (`:172`).</li></ul> |
| `approach_transit._steps` rule and the "no world = not checked" verdict (`TG/approach_transit.py:60-73, 178-200`) | sampling transits and link moves | **Borrow the rule and the convention.** `transit_blocked` is leg-specific (`:54-57, 95-119`). |
| `corridor_witness` tuck family (`TG/approach_transit.py:122-158`) | automatic detour when a straight link move is blocked | **Reference (idea)** |
| `schedule_guards` sentences, fixture tier, "stale is not pass" (`TG/schedule_guards.py:1-30`); `dacReconcileFixture` (`TG/web_editor/editor.js:14332-14353`) | plain-sentence validation of the program list; refuse mixed toilet states | **Borrow the pattern** (re-implement; it reads ALIX schedule CSVs) |
| DAC playback, red link highlight, Prev/Next, log (`TG/web_editor/editor.js:14272-14598`; `TG/web_editor/template.html:598-640`) | preview of a merged program; jump to a collision | **Borrow with adaptation** via R3/R5 |
| `do_dual_arm_collisions` endpoint (`TG/server.py:4335-4366`) | — | **Not usable:** no commode, walls, hands or pole; plan JSON only; dt 0.05 |
| `coordination.py`, slowdown-only dual re-timing (`TG/coordination.py:1-26`) | future automatic dual timing | **Reference only.** It is plan-JSON based and part of the C-space integration plan, and open-loop drift of seconds limits its value here (inferred). |
| TXT merging, link-move generation, Program-ID allocation, `.txt` I/O | — | **Not available — build** |

### 6.2 Other sources

| Source | Verdict | What to take |
|---|---|---|
| V6 package (`V6/README.txt`, `BRIDGE_REPORT.csv`, merged files) | reference + golden regression data | <ul><li>Take the method: holds and stretch, minimum-jerk link moves, split on pole moves.</li><li>The link moves reproduce at 0 mdeg [M]; compare parsed rows (the files are CRLF).</li><li>Do **not** reuse the 1.38 s pole model, IDs 71–83, or unchecked link moves.</li></ul> |
| `plan_min_time.concatenate` (`ALIX/tools/plan_min_time.py:142-169`); `EXP01` `concatenate` (`EXP01/src/plan_to_track.py:100-136`) | borrow with adaptation | <ul><li>Join contract: duplicate drop under 1 mdeg, spans, holds as repeated points.</li><li>Port to integer millidegrees, raising exceptions.</li><li>Pass gaps to the link-move generator instead of refusing them.</li></ul> |
| C++ grouping (`TE.hpp:145-170`; `TE.cpp:378-453`) | borrow the rule | A pole or hand stage ends a program; keep a `pre_stroke` record. |
| `DAV:501-733` (shim, seeding, side guard, windowed separation, 25 mm floor) | borrow with adaptation | `.txt` → plan at the player clock; arm vs arm over a window computed from Lite's measured timing. |
| `WSP/dual_arm_plan_collision.py:586-760` (`extra_managers`); `compute_plan_frames` chains (`:487-583`); `WSP/plan_scene_validation.py:117-256` | borrow with adaptation | Aligned dual sweep including the commode (`fixture_manager`) and walls, with explicit pole and hand seeds; chained previews. |
| ALIX gates: G2 band-limited (`CT:1646-1672`, `TT:420-456`), near-limit rule (`TT:122-176`), step 10° (`TT:128`), `rate_fit_track` (`CT:1675-1700`) | borrow (port to pure numpy) | <ul><li>Gates on the written integers.</li><li>A one-click "slow only where over the rating" fix that leaves the path unchanged.</li><li>`scipy.signal` does not import under system python here [M], so carry the 4th-order Butterworth coefficients and filtfilt in numpy, or pin a venv.</li></ul> |
| `UC/pole.py:3-58` (= `DP/peripherals.py:95-140`); `UC/hand.py:13-14` | borrow as-is | The runner's own pole and hand step durations for the dual timeline. |
| `DAF` | reference | Step classes; serialised start transits with the partner static; the 25 mm windowed rule. |
| `HTML` `interpolateRegion` (`:1568-1590`), `dataToTxt` (`:1618-1620`) | borrow with adaptation / as-is | Stretch along the path for time matching; LF writer. No merge feature (`:349-352`). |
| UI v53 + runner v14 | use as-is for execution (no merge function) | Run the emitted workflow; respect the constraints in §3. |
| `WSP/clearance.ClearanceProbe` | do not use for open states | Mis-posed fixture (`WSP/clearance.py:49-95`). |
| `WSP/redundancy.classify_pair` | adapt | Class codes are fine, but bin covers read as "lid" (`:90-114`). |

---

## 7. Approach

### 7.1 Terms (UI and code)
- **Region (unit):** one taught recording for one arm, after R2 (map 13 IDs).
- **Program:** one controller slot holding one merged `.txt`.
- **Link move:** generated motion inside a program.
- **Transit:** the controller's own move to a program's first point.
- **Block:** v53's TOGETHER stage.

### 7.2 When a new program starts
1. The arm changes.
2. The pole height changes (exact mm).
3. Any of the 6 hand values changes.
4. The toilet state changes ("any" inherits). Lite puts a checkpoint or mechanism program between.
5. The unit is not a TOGETHER cleaning unit: mechanism, tool, glove, or RUN FIRST / WAIT OTHER.
6. The point budget would be exceeded.
7. The operator has placed a manual break.

Between programs the workflow gets, in order: the pole step, then the hand step, then any checkpoint or wait, with the arm still. A "hand release before pole moves" setting (ALIX practice, `CT:145-147`) is off by default, as in V6.

### 7.3 Joins, link moves, program ends
- **Gap rules.**
  - A gap < 0.001° drops the duplicate.
  - Contiguous rows of one recording join directly.
  - Otherwise Lite inserts a link move.
- **Default link move: rest-to-rest minimum-jerk**, identical to V6:
  - Duration: `T = max(0.2 s, maxⱼ 1.875·gapⱼ/v_capⱼ, maxⱼ √(5.7735·gapⱼ/300))`, rounded up to 5 ms.
  - `v_capⱼ = min(link speed, 0.97·ratingⱼ)`. The ratings are 180/180/225×5 °/s.
  - Interior samples are `round(qA + s(k/(n−1))·(qB−qA))` with `n = T/5 ms + 2`.
  - After rounding, re-measure on the integers and add ticks until the caps hold.
- **Option: velocity-matched quintic.** It matches position and velocity at the ends. Because its path is not a straight line, every point is checked.
- **Program ends.** If the first or last 10 ticks exceed 5 °/s, offer "Ease in/out". This is a minimum-jerk ramp from or to rest, about 0.2–0.5 s, sized by the same caps. Alternatively, flag the region end for R2 trimming.
- **Blocked link move or transit.**
  - The first remedy is **"Go via the safe pose"**: two checked minimum-jerk legs through a stored safe pose per arm (the teach start pose, or a workflow `global` waypoint whose joints are known).
  - If that is also blocked, the remedy is to re-teach the join.
  - A tuck-family search (as in `corridor_witness`) can propose a via pose automatically, as an option.
  - "Slower" and "separate programs" are **not** offered: both follow the same straight joint line.
- **Large transits.** Optionally replace a transit over 2° inside a TOGETHER block, or over 10° anywhere, with a checked approach link move at the start of the next program. This leaves the controller a transit near 0° (ALIX practice, `EXP01/src/plan_to_track.py:143-151`).

  Caveat: that program then assumes it starts where the previous one ended. Running it alone makes the controller transit to that pose instead.

### 7.4 Gates (written integers; shared with R12)

| gate | rule | result |
|---|---|---|
| format | 7 integer millidegrees per line, ≥ 2 lines (`RUN14:76-89`) | hard block |
| step | ≤ 10° per line (0x4000, `TT:128`) | hard block |
| rating (G2) | 0 ticks over the rating at 5.0 ms on the 20 Hz band-limited rate. Raw count shown. | block; engineer override with reason; one-click fix "slow only where needed" (port of `CT:1675-1700`) |
| joint limits | URDF ±178/130/178/135/178/128/360°. Inside 5°: band-limited rate ≤ 10 °/s. Margin < 2°: refused (`TT:126, 172-173`). | < 2° hard block; 2–5° block with override |
| program ends | ≤ 5 °/s over the first and last 10 ticks (proposed) | warning with "Ease in/out" |
| join smoothness | one-tick speed step at each join ≤ max(40 °/s, the neighbours' p99.9) | warning |
| size | ≤ point cap (configurable; default 12,000 only after the start-time test in criterion 15) | split at a region boundary |
| name | `len(f"{id}_") + len(prefix) + len(stem) + 4 ≤ 32`. Stem ≤ 21 characters is safe for every ID 1–100 and both upload paths (`RUN14:992-1015` adds `DT_`). | block |
| run time | duration band < runner timeout 300 s | info / block |
| upload time | ≈ 0.24 s + 21.3 µs/byte | info |

### 7.5 Collision validation (one worker process; background; cached)
1. **World.**
   - Vendored `BodyCollisionWorld(scene_yaml=<calibrated venue>)`.
   - Fixture: `fixture_manager(fixture, program toilet state, ugv_standoff_m=<cell value>)`, with calibrated lid and seat open angles (`scene_pose` takes any angle, map 05 §4.3).
   - Bin in its declared state.
   - Active pole = `pole_mm/1000` (1:1 since 2026-08-30, map 11).
   - Active hand joints from `hand_units.py`.
   - Partner: at the end pose of its previous program for serial steps; covered by (f) for TOGETHER blocks.
2. **Rule checks before geometry:**
   - **Stowed bin:** any pole height in, or travel across, 153–282 mm is BLOCKED.
   - **Scene plausibility:** run the **raw** recordings of the selected regions through the world. Any arm-link contact, or hand penetration > 20 mm (`PAD_COMPLIANCE_MM`, `TG/body_collision.py:65`), makes the scene suspect, so every verdict in it is NOT CHECKED.
   - **Baseline:** a non-empty rest baseline is NEEDS ATTENTION.
3. **Samples:**
   - every link-move point;
   - **every cleaning row** (≈ 25 s per 10,000 rows at ~2.5 ms);
   - transits as straight joint lines at ≤ 2° per sample, at the new pole height;
   - pole steps swept every 10 mm with the arm held at the previous end pose;
   - hand changes at 3 intermediate states;
   - the end pose before any device step, which must have no hand–toilet contact.
4. **Classifying:**
   - **Untaught motion** (strict mode): any contact is a collision. The one exception is that the hand may keep, for the first or last 10 % of a link move, the contact it has at the adjacent region's end or start. Lite also reports the minimum clearance to the toilet and walls, using FCL distance on link-move and transit samples only (24–89 ms each, map 05 §4.5); below 20 mm it is NEEDS ATTENTION.
   - **Cleaning rows:** hand vs the region's own surface is expected contact (depth; > 20 mm is NEEDS ATTENTION). Arm structural links (`*_Link1-7`, Connector, CameraHolder, camera) against anything are a collision. Walls, bin, UGV, poles and partner are a collision.
   - **Names:** use Lite's own table, not `classify_pair`.
5. **Verdicts:**
   - **READY** — all gates pass and there are no collisions; TOGETHER programs also need (f).
   - **NEEDS ATTENTION** — warnings only.
   - **BLOCKED** — a collision or a hard gate failed.
   - **NOT CHECKED** — the world is incomplete or suspect, or the inputs are stale.

   Cache by (input sha256s, metadata, scene parameters, engine version).
6. **Mechanism programs** (the arm moves the lid or seat). The moving part is not modelled. Lite checks walls, bin, robot, partner and the commode body, and reports "lid/seat motion not modelled". At best these are NEEDS ATTENTION, until a hardware dry run is recorded.

### 7.6 Dual arm
- **Must, for TOGETHER blocks.** Build each arm's block timeline:
  - programs on the player clock band;
  - **both arms' start transits**: max Δq / (180 °/s × transit %), from the emulated movej law (map 11 §4.2, inferred for the controller);
  - pole steps from `UC/pole.py`;
  - hand steps from `UC/hand.py`;
  - switch settle of 0.6 / 1.0 s (`RUN14:1947-1955`);
  - start delay as measured on the Lite runner. Until measured, use ALIX's band and say so.

  Run the aligned `compute_dual_arm_plan_collisions` with `extra_managers = {commode, walls}` and seeded pole and hand joints. Then run the windowed `cross_arm_separation` over the predicted skew band through the `DAV` shim, against 25 mm. If either fails, propose SERIAL. Cache by sha; windowed sweeps take minutes.
- **Should:**
  - time matching by holds (repeated points) or stretch (`HTML:1568-1590`);
  - the skew band at each pole move and at block end;
  - recommend re-syncing at pole moves when skew exceeds 1 s, which needs a new sync step;
  - emit `DUAL_TIMELINE.json` (the file V6 promised).

### 7.7 Outputs, IDs, first hardware run
- **`<stem>.txt`:** LF NDJSON integers. The runner makes the CRLF `.proj` (`RUN14:91-97`).
- **`<stem>.merge.json` (`alix_lite_merge/1`):**
  - inputs: unit ID, file, sha256, output rows, raw-row map;
  - link moves: T, gap, peaks, mode;
  - gates: value and threshold for each;
  - scene: fixture, state, angles, venue, standoff, bin, pole, hand, partner, baseline;
  - verdict, with pairs and sample indices;
  - engine versions.

  The models are ALIX `SOURCE.json` (`ALIX/Resource/tracks/taught/source/README.md:8-27`) and the HTML report (`HTML:1732-1756`).
- **Per-arm workflow** `realman_workflow_sequence_v1`:
  - IDs from a per-arm registry over a declared range;
  - the same ID on both arms for serial steps;
  - unique per arm;
  - never in a reserved range; ALIX's 70–100 counts as reserved while the controllers may be shared.
- **First-run kit:** for each new link move, a slowed test file on the same straight path at ≤ 1/3 speed (minimum-jerk with 3T), plus a checklist. A slow first run cannot be made with the speed setting (`DP/track.py:21-24`).

### 7.8 Preview
Play the merged program, or both arms for a block, in the R3 view:
- link moves are shaded on the joint plot and transits are dashed;
- the collision sample is shown in red, with the link highlighted;
- "Back to editing" opens the owning region.

### 7.9 Steps
1. **Segment model.** Parse with a port of `DP/track.parse_track` (`DP/track.py:66-82`) into int32 (N, 7). Attach R1/R7/R2 metadata, including the raw-row map.
2. **Planner** (§7.2), with reason codes and sentences, an ID registry and stem names ≤ 21 characters.
3. **Link moves and program ends** (§7.3), assembled with spans as in `plan_min_time.concatenate`.
4. **Gates** (§7.4), in pure numpy, shared with R12, including the rate-fit fix.
5. **Collision adapter**: vendor `body_collision.py`, `scene_pose.py`, `fixture_parts.py`, assets and `hand_units.py`, and add strict mode, hand joints, bin state and the baseline flag. Then add the rules, plausibility check, sampling, classification and clearance margins (§7.5).
6. **Dual check** (Must for TOGETHER) and **time matching** (Should) (§7.6).
7. **Outputs**, staleness watch and first-run kit (§7.7).
8. **Program Builder UI** (§9), handing off to the R3 preview and R13 upload.
9. **Tests:**
   - V6 golden, row for row;
   - split rules;
   - link-move sizing;
   - ends;
   - G2 on V6;
   - strict-mode and stowed-bin regressions;
   - synthetic collisions;
   - NOT CHECKED paths;
   - staleness;
   - performance;
   - offline workflow validation;
   - optionally, the runner's `main()` driven against a fake controller, as ALIX does (`ALIX/core/python/alix_dispatch/test/test_rm_upload.py`, map 09 §9.2);
   - then a controller SIM-mode run before any REAL run.

---

## 8. Acceptance criteria
1. **V6 golden.** Re-merging the V6 segments, split at the known bridge rows, in "V6 mode" (rest-to-rest, given T) reproduces every **parsed row** of `L_P01P02`, `L_P03P05`, `L_P06P07`, `L_P10P12`, `R_P01P03`, `R_P06P07` and `R_P10P12`. The V6 files are CRLF, so the comparison is row for row.
2. **Splits.**
   - Poles 150 → 150 → 50 mm give two programs, a pole step between them, and the sentence "Pole height changes 150 → 50 mm — new program".
   - Different hand values, toilet states, and mechanism or serial units always split.
   - Toilet-state splits get a checkpoint step.
   - A region marked "any" takes the program's state.
3. **Link-move sizing.** At 30 °/s a 39.08° gap gives `T ≥ 2.445 s`. On the written integers at 5 ms, every link move has:
   - peak speed ≤ the selected speed and ≤ 0.97 × rating;
   - nominal acceleration ≤ 300 °/s²;
   - no duplicate boundary rows.
4. **Program ends.** Programs whose first or last 10 ticks exceed 5 °/s are flagged; the V6 test cases are the R_P10P12 start and the M3_ASOPN01 end. After "Ease in/out" they start and end at rest and still pass every gate.
5. **G2 on V6.** Exactly R_P06P07 (102 band-limited ticks, first at tick 2851, peak 312 °/s J6) and R_P10P12 (29 ticks) fail. Raw counts are shown as information (10 files; R_P06P07 has 172 raw ticks, first at 288). Neither failing file is READY without a recorded engineer override. "Slow only where needed" removes the band-limited ticks while leaving the path unchanged.
6. **Near limit.** R10_HNGR is BLOCKED: its J4 margin is 1.53° (< 2°), and it sits inside 5° for 492 rows at up to 24 °/s. No override is offered.
7. **Strict mode.** A synthetic link move that drags the glove through the seat is reported as a collision, with sample, pair and depth, and the program is BLOCKED. The regression test also shows that the unmodified `make_pose_filter` would have allowed that pair.
8. **Expected contact.**
   - Glove on the lid during lid-top rows is expected contact, with depth.
   - The same contact in the middle of a link move or transit is a collision.
   - Contact in the first or last 10 % of a link move that continues the adjacent region's own contact is reported but allowed.
9. **Device steps.** These are errors at their boundary:
   - a pole step whose 10 mm sweep, with the arm held at the previous end pose, puts the hand into the toilet;
   - an end pose with hand–toilet contact before a pole or hand step.
10. **Stowed bin.** With the bin declared STOWED, any pole height in, or travel across, 153–282 mm is BLOCKED with a sentence about the bin covers. V6's first step (200 mm) is the test case.
11. **Scene plausibility.**
    - With the default 800 mm venue, the raw R2_LIDTOP recording shows `R_Link5↔washroom_structure` (map 05 §4.6). Every verdict in that scene is then NOT CHECKED until it is recalibrated.
    - A non-empty rest baseline is shown as NEEDS ATTENTION.
12. **Missing scene data.** With the toilet state, venue, standoff or bin state unset, the verdict is NOT CHECKED, never READY, and upload is disabled.
13. **Staleness.** Changing an input file, its pole, hand or toilet-state metadata, the scene calibration or the engine version marks the program out of date. Upload stays disabled until it is rebuilt and re-checked.
14. **Names, IDs, size.**
    - The stored name `f"{id}_"+[DT_]+stem+".txt"` is ≤ 32 characters for any ID 1–100.
    - No ID comes from a reserved range.
    - Merges above the cap split at a region boundary.
    - Predicted run time is < 300 s.
15. **Execution.**
    - The workflow passes `normalize_workflow_step` offline (`RUN14:1668-1763`), loads in UI v53 Page-02, and runs end-to-end in controller SIM mode with runner v14.
    - The largest merged program on each controller prints a start-ack below 4 s (`RUN14:1897-1901`). Otherwise the cap is lowered to the largest size that passed.
16. **First hardware run.** Every new link move has a slowed test file (same path, ≤ 1/3 speed). The checklist requires that file before the full program.
17. **Performance.** A single-arm check of a 10,000-point program with 3 link moves, checking every row, finishes in ≤ 60 s on the Lite PC, with progress shown.
18. **Report contents.** For every program the report holds:
    - inputs with sha256, output rows and the raw-row map;
    - each link move's T, gap, peak speed and acceleration;
    - every gate value with its threshold;
    - the full scene and baseline;
    - engine versions;
    - the verdict.
19. **Dual check (Must for TOGETHER).** Each block reports:
    - per-arm duration bands;
    - both start transits;
    - the predicted skew band;
    - the windowed minimum arm-to-arm clearance.

    Below 25 mm the block is proposed SERIAL. TOGETHER programs are never READY without this check.
20. **Time matching (Should).** Holds or stretch make the nominal block lengths equal to within 1 tick, and the skew band is recomputed and shown.

---

## 9. UX notes (non-technical operators)
- **One screen, "Build program."**
  - Left: the arm's taught regions as cards (thumbnail, name, status).
  - Right: an ordered list the operator fills by dragging.
  - Lite draws the program boxes, with separators that say why ("Pole moves 150 → 50 mm", "Open the lid now (manual check)").
- **One main button, "Check & build."** Results show as traffic lights with icon and text: Ready / Needs attention / Blocked / Not checked, e.g. "Toilet position not measured yet".
- **Link moves** appear as small grey chips ("link move 1.3 s"). Numbers sit behind "Details".
- **Problems are sentences with real fixes.**
  - Example: "The link move from Lid Top to Right Hinge would hit the toilet seat (right forearm)."
  - Fixes: **Show me** (3-D jumps there), **Go via the safe pose**, **Back to editing**, **Re-teach this join**.
  - Never offer "slower" for a hit: slowing does not move the path.
- **Speed choices.** Operators choose link-move speed as Gentle (20 °/s) or Normal (30 °/s, the default). "Brisk" (45 °/s, about the earlier hardware-tested bridge, `V6/README.txt:43-46`) is engineer-only.
- **Run times** are ranges ("about 55–59 s"), never one exact number.
- **Engineer panel.** P-numbers, Program IDs, slots and sha values live there. Operators see region names.
- **Banners:**
  - "Lid Top was edited after this program was built — rebuild before upload."
  - "First run: play the slow test files first."
- **Theme.** Keep the ALIX Studio theme tokens, status chips, toast and confirm modal. Share R2's undo ring for builder edits.

---

## 10. Safety notes
- **Untaught motion.** Link moves, transits and pole/hand moves are motions nobody taught. Check them strictly. For the first real run, use the **slowed test files**: the speed setting does not slow a stored program (`DP/track.py:7-11, 21-24`).
- **Scene quality.**
  - A verdict is only as good as the scene. Map 05 found false wall hits with the default 800 mm venue, and the world is validated only for `commode_c` (`TG/body_collision.py:58`).
  - Calibrate the venue (or no walls), the standoff, the commode identity and the lid/seat open angles.
  - Use the raw-recording plausibility gate.
- **Allowances.**
  - Without strict mode, `make_pose_filter` allows hand-vs-toilet contact everywhere (`TG/body_collision.py:595-596, 866`).
  - Never judge untaught motion with the unmodified filter.
  - Show the rest baseline (`:871`); a pair hidden there is hidden for every sample.
- **Stowed bin.** The world always poses the bin lowered and cannot see the arm base hitting the covers (`TG/studio_generate.py:472-475`; `TG/body_collision.py:251-255, 629`). Enforce the 153–282 mm band whenever the bin may be stowed. V6 starts at 200 mm.
- **"Not checked" is never "pass"** (`TG/approach_transit.py:178-200`; `TG/schedule_guards.py:13-17`).
- **Device moves.**
  - Never merge across pole, hand or toilet-state changes.
  - Never move the pole or hand with the glove on a surface (`CT:145-148`).
  - Check end poses before device steps.
- **Program ends.** Programs must start and end at rest. A step start at ~55 °/s is commanded with no smoothing by the player.
- **Ratings.** Enforce G2 before upload. The runner treats 4105 as warning-only (`RUN14:56-57`), while ALIX saw it end a program at 58 % (`CT:211-219`), so a cut-short program can read as complete.
- **Joint margins.** Use ALIX's near-limit rule (≤ 10 °/s inside 5°; refuse below 2°). ALIX saw J4 overshoot by up to 5.9° (map 06 §2.2).
- **Dual arm.**
  - Equal point counts do not keep the arms in step: drift is seconds, and unmeasured beyond 4,281 points.
  - Pole steps take 2.5–3.2 s per 100 mm in the runner.
  - v53 starts both arms' transits together, unlike ALIX, which serialises them with the partner still.
  - Require ≥ 25 mm windowed clearance including the transits, or run SERIAL. Keep transits inside TOGETHER blocks under 2° by using checked approach link moves.
- **Large programs.** Measure the start delay (5 s runner window) before allowing them.
- **Slots and speed.**
  - Read back each slot's stored name and size before overwriting it, and before running it (R13).
  - Never write into 70–100 while the controllers may be shared with ALIX: same IPs, `DP/connect.py:31-32`.
  - The runner restores the global speed after runs; keep that.

---

## 11. Dependencies, risks, open questions

**Dependencies**
- **R1:** the region catalogue, canonical IDs and toilet state per region (including "any"); a teaching protocol that chains recordings or starts each from one safe pose.
- **R2:** optimised files with sha256, edit recipes and a **row map back to the raw recording**.
- **R7:** pole and hand values per region; counts converted to joints with `hand_units.py` (vendored).
- **R3:** the simulation preview.
- **R5:** the shared single-process collision worker (vendored and adapted `BodyCollisionWorld` + `DAV` shim) and the collision highlight/jump.
- **R10:** dual view, bin state declaration, drift display, and the shared dual check.
- **R12:** shared gates: G2, near-limit, step, ends, rate-fit fix.
- **R13:** the Program-ID and slot registry; upload; stored name/size readback before overwrite **and before run**.
- **INT-CTRL:** the workflow JSON for v53/v14, plus:
  - explicit checkpoint and sync steps;
  - handling of the 5 s start window;
  - 4105 handled as a stop condition, or programs gated before upload;
  - a slot-content check before a run.
- **Cell calibration:** venue or no walls, UGV-to-commode standoff, commode model, lid/seat open angles, bin state.
- **Python environment:** numpy, scipy, trimesh, python-fcl, pyyaml in a pinned venv. System scipy 1.11.4 fails to import `scipy.signal` against numpy 2.4.6 here [M]. Use vendored or pinned ALIX modules, because private names change.

**Risks**
- Scene mismatch gives false BLOCKED or, worse, false READY. The plausibility gate reduces this but cannot remove it.
- A strict mode added to a vendored copy can drift from ALIX upstream. Pin it, and keep a parity test.
- Large merged programs may exceed the runner's 5 s start window (extrapolated), and their drift is unmeasured.
- Regions that end in contact make the following device steps unsafe. They need trimming, ease-out or a taught lift-off.
- Over-rating ripple (raw) is common in taught data. If operators see raw counts as failures, they will over-override. Show G2 as the gate and raw as information.
- Windowed dual sweeps take minutes. Run them in the background, cached by sha.
- Pole and hand timing models are conservative upper bounds of the runner's waits. The physical motion may be faster, which matters for collision timing.
- Program-ID collisions with ALIX on shared controllers.
- Mechanism programs (moving lid or seat) cannot be fully checked offline.

**Open questions (for the requester)**
1. Is the Lite toilet ALIX's `commode_c`, with the same meshes and hinges? What are the cubicle (or no-wall) layout, the UGV-to-commode standoff and the manual lid/seat open angles in the Lite cell?
2. Are the two controllers shared with ALIX? They use the same IPs. Which Program-ID range does Lite own per arm (V6 uses 71–83; ALIX reserves 70–100 and uses 40–59 for taught slots)?
3. Should G2 and the 2–5° near-limit failures block READY with an engineer override, as proposed, or only warn?
4. Default link move: rest-to-rest minimum-jerk (V6-compatible) or velocity-matched? Is 30 °/s acceptable?
5. Teaching protocol: will regions be recorded as a chain (each starts where the previous ended, as V6) or from one safe start pose (ALIX)? Where is the safe pose for detours?
6. Dual blocks: keep V6's overlap of one arm's link move with the other arm's pole move (open loop), or re-sync at each pole move with a new sync step?
7. Time matching: hold (dwell, keeps wiping speed) or stretch (slower wiping)?
8. What start-ack times did the 10,000+-point V6 programs print under runner v14? What is the largest program actually run on each controller?
9. Is the bin present during cleaning, and in which state?
10. Should V6 programs be imported as legacy regions, split at the known bridge rows, so they can be re-merged and checked?

---

## 12. Priority and effort
- **Priority: Must.** The team already merges by hand without any collision check (V6), so a checked merge closes a real safety gap. The dual clearance check of TOGETHER programs is part of Must. Automatic time matching is **Should**.
- **Effort: XL, ≈ 23 person-days**, one experienced developer, including tests.

| part | estimate |
|---|---|
| planner, splitting, IDs and names | 3 pd |
| link moves, ends, detours, plus V6 golden tests | 3 pd |
| gates (numpy G2, near-limit, ends, sizes; rate-fit port; shared with R12) | 2 pd |
| collision adapter (vendor + strict mode, hand joints, bin, baseline, foul rule, clearance margins, classifier, plausibility gate) | 4.5 pd |
| outputs, provenance, staleness, first-run kit | 2 pd |
| Program Builder UI | 3 pd |
| dual clearance check for TOGETHER blocks (Must) | 3 pd |
| **Must subtotal** | **≈ 20.5 pd** |
| dual time matching, skew band, `DUAL_TIMELINE.json` (Should) | 2.5–3 pd |

This assumes the shared collision worker and 3-D preview from R3/R5 exist. If R8 lands first, add about 5 pd.

---

## 13. Verification log (material changes from the analyst draft)
1. **Rating gate.** Raw per-tick → ALIX G2 (20 Hz band-limited, 5.0 ms). V6 failures go from 10/15 to **2/15**. R_P06P07's figures are 102 ticks, first 2851, 312 °/s (raw 172/288/491 kept as info) [M]; `CT:1657-1672`, `TT:420-456`.
2. **R10_HNGR.** "Override possible" → **hard block**. Its 1.53° margin is below ALIX's 2° floor, and the draft contradicted its own "< 2° hard block" rule; `TT:172-173` [M].
3. **Margin rule.** Static "≥ 5° or override" → ALIX's near-limit rule: ≤ 10 °/s inside 5°, refuse < 2° (`TT:126, 172-176`).
4. **Strict filter.** "Strict no-allowance filter" was claimed to be available → it is **not** in `BodyCollisionWorld`: the allowance defaults to all commode parts (`TG/body_collision.py:595-596, 866`). Lite needs a vendored change.
5. **Other world adaptations added:** named hand state only (`:582-591, 665`); bin always lowered (`:629`); rest-baseline subtraction (`:871`); no floor; no-walls venue raises (`WSP/plan_scene_validation.py:125-127`).
6. **Stowed-bin foul band.** The pole sweep "via the world" → **rule**. `base_link` and the bin covers are both static and never queried against each other (`TG/body_collision.py:251-277`; `TG/studio_generate.py:472-475`).
7. **Reduced speed first run.** "Reduced overall speed" → **slowed test files**: speed only scales the transit (`DP/track.py:7-11, 21-24`; map 06 TL;DR 3).
8. **Program ends.** A new gate: programs start and end at rest. V6 starts at ~55 °/s and ends at ~67 °/s [M]; the player does not smooth (`DP/track.py:7-11`).
9. **Remedies for a blocked link move.** "Slower" and "Keep as separate programs" were removed: both keep the same straight joint line. Added: detour via a checked safe pose, an optional tuck-family search (`TG/approach_transit.py:122-158`), and re-teach.
10. **Cleaning-row sampling.** Every 4th row → **every row** (~25 s per 10k rows at ~2.5 ms; map 05 §4.5). Untaught motion also gets a clearance margin via FCL distance.
11. **Scene plausibility gate added.** Raw recordings run through the scene; arm-link contact means NOT CHECKED (map 05 §4.6 evidence).
12. **Dual-arm.** The clearance check for TOGETHER programs moves to **Must**; time matching stays Should. Both arms' simultaneous start transits are added to the timeline (`UI53:10209` vs `DAF:3763-3800`).
13. **Name rule.** "≤ 27 characters" → the stored-name formula, with stem ≤ 21 for every ID and both upload paths (`ALIX/Resource/tracks/taught/README.md:150-160`; `RUN14:992-1015`).
14. **Control-app constraints added** (one line each, §3): the 5 s start window vs a start delay that grows with size (`RUN14:238, 658, 1909`; `DAV:221-224`); 4105 warning-only (`RUN14:56-57`; `CT:211-219`); no slot-content check before a run (`RUN14:2017-2031`).
15. **300 s timeout.** No longer given as a reason against one-file days: the L day plays about 250–265 s [M]. The size and start-window reasons stand.
16. **V6 bridge description corrected.** 4 holds, 4 short joins and 3 moves [M]. "Brisk 45 °/s as tested in V6" → not hardware-tested per the README (`V6/README.txt:43-46, 53-56`), and engineer-only.
17. **`approach_transit`.** "Borrow as-is" → borrow the sampling rule and convention; `transit_blocked` is leg-specific (`TG/approach_transit.py:54-57, 95-119`).
18. **New caveats:** `classify_pair` mislabels bin covers (`WSP/redundancy.py:90-114`); `ClearanceProbe` mis-poses open states (`WSP/clearance.py:49-95`).
19. **V6 TOGETHER sizes.** "6,744–10,983 points" → 3,533–10,983, with 9 of 11 outside the measured envelope [M]. The start-delay model is ALIX's launcher, not Lite's.
20. **Largest upload.** "Proven upload 12,864" → Lite's 12,429 is upload-verified; ALIX's 12,864 row is EMITTED and played only by inference (`UC/README_COMMANDS.md:45-51`; map 06 §1.5).
21. **Shared-controller evidence added:** Lite IPs = ALIX defaults (`DP/connect.py:31-32`); Lite README used slot 85 (`UC/README_COMMANDS.md:44-51`).
22. **Input-claim check extended.** The tests check continuity, not collisions; `WSP/clearance.py` exists but mis-poses open states.
23. **New borrows:** `CT` `rate_fit_track` as the one-click G2 fix (`CT:1675-1700`); `compute_plan_frames` chains for preview (`WSP/dual_arm_plan_collision.py:487-583`).
24. **Golden test.** "Byte-for-byte" → row-for-row: the V6 files are CRLF [M]; Lite writes LF (`TE.hpp:20-26`).
25. **Merging benefit clarified** (inferred). It saves ~1–2 s per join and costs bigger re-uploads.
26. **Mechanism programs.** Marked as not fully checkable offline (moving lid or seat not modelled).
27. **Effort.** 17–19 pd → about 23 pd (XL). Must ≈ 20.5 pd.
