# 10 — Data formats, merged V6 package, region catalogue (ALIX Studio Lite)

Scope: R1, R8, R10, INT-CTRL (plus data facts feeding R2, R5, R7, R9, R12, R13, R14).
Everything below was read-only. Statistics came from small scripts in the session scratchpad
(`scratchpad/scripts/{txtstats,recon,findbridges,seqgaps,margins,spikes,fk_segments,fk_side_probe,pptxpos}.py`,
run with `python3 -I`). Re-run them to reproduce any number. FK used a numpy-only re-implementation of
`alix_workspace.urdf_kinematics`, because importing `alix_workspace` fails in this environment:
system scipy 1.11.4 is ABI-incompatible with numpy 2.4.6 (`ValueError: numpy.dtype size changed`).

Path aliases used in citations:

| alias | absolute path |
|---|---|
| `WS/` | /home/newtonjeri/alix_studio_lite_ws/ |
| `UC/` | /home/newtonjeri/alix_studio_lite_ws/upload_channel/ |
| `V6/` | /home/newtonjeri/alix_studio_lite_ws/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/ |
| `HTML` | WS/Alix_Studio_Lite_Trajectory_V2.html |
| `UI53` | UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py |
| `RUN14` | UC/rm_upload_fast_v14_safe_transit.py (byte-identical to `_v52support`/`_v53support`, `cmp` clean) |
| `OPS` | "WS/RealMan Dual-Arm Operations UI.txt" |
| `REQ` | "WS/alix_studio_lite/docs/reference/ALIX_Studio_Lite_requirement_inputs.txt" |
| `ALIX/` | /home/newtonjeri/alix_ws/src/alix/ |
| `CC/` | ALIX/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/ |
| `TT/` | ALIX/Resource/tracks/taught/ |
| `CT/` | ALIX/Resource/tracks/commode_c/ |
| `EXP01/` | ALIX/experiments/exp01_teach_trajectory_reverse_engineering/ |
| `PPTX` | scratchpad/pptx/ppt/slides/slideN.xml (the extracted Toilet_regions_V1.pptx) |

**Control-app version (INT-CTRL).** v48 is **not** the latest. `UC/` holds v48 (Oct 5 05:48), v51 (09:42),
v52 (16:48) and **v53 (Oct 5 17:09)**. `UI53:45-46` declares `UI_PROGRAM_VERSION = "v53"` and
`EXPECTED_RUNNER_VERSION = "v14"`. `UC/py programs/` stops at v48, which is byte-identical to `UC/rm_trajectory_ui_v48_handoff_continue_fix.py`.
`OPS:222-224` still says "UI v50 / Runner v13", so that document is stale. The runner search order in
`UI53:6890-6905` prefers `rm_upload_fast_v14_safe_transit.py`.

---

## 1. The trajectory `.txt` format

### 1.1 Fields, units and joints
- Each line is NDJSON: `{"point":[j1,…,j7]}`. Every file I checked has exactly one key, `point`.
  It holds 7 **integer millidegrees** of a RealMan RM75 arm and carries **no timestamp, side, region or speed**.
  The runner's docstring says so: "one joint point per line … in millidegrees" (`RUN14:14-15`).
  The validator `read_track` refuses the whole file unless every line has 7 ints, and needs at least 2 points (`RUN14:76-88`).
  The HTML parser also requires 7 finite numbers but silently *skips* bad lines (`HTML:803-829`).
  ALIX uses the same native format: "NDJSON integer millidegrees (the channel's native format)" (`TT/README.md:18`).
- Line endings are inconsistent. All V6 files are **CRLF**. `UC/zip/*` and `UC/plan_txt_files/*` are **LF**.
  The HTML writes LF (`dataToTxt`, `HTML:1618-1620`). The runner normalises `\r` (`RUN14:78`), then writes the controller project with CRLF.
  ALIX's taught lane demands that source bytes and CRLF stay untouched (`TT/source/README.md:3-6`).
- **Controller project file** (what `rm_send_project` sends): `{"file":7}` CRLF, then
  `{"name":"Folder","num":1,"type":<N points>,"enabled":true,"parent_number":0}` CRLF, then the point lines, all CRLF
  (`RUN14:18-24, 91-97`). The upload is verified by comparing the stored size in the program list
  (`RUN14:147-175`). The controller stores the name as `f"{id}_" + basename` truncated to **32 chars**
  (`TT/README.md:150-160`). The UI's drag-teach names (`DT_` prefix) can be up to 56 chars (`RUN14:992-1015`), so they truncate.
- **Raw recordings** come from the controller itself. `rm_start_drag_teach(1)` records and `rm_save_trajectory(path)` returns `(ret, point_count)`
  (`RUN14:912-989`). The UI's default name is `~/Downloads/RealMan_DragTeach/{left|right}_drag_YYYYmmdd_HHMMSS.txt`
  (`UI53:3840-3850, 8043-8056`). The name carries **no region and no metadata**.
- RM75 limits (deg) from `ALIX/core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf` (L_joint1 at :1185):
  J1 ±178, J2 ±130, J3 ±178, J4 ±135, J5 ±178, J6 ±128, J7 ±360. Velocity ratings are 180°/s (J1-J2) and 225°/s (J3-J7).
  The channel limits are rate ≤ rating, step ≤ 10° per line, and position inside the limits (`ALIX/docs/motion/TRACK_CHANNEL_TIMING_REQUIREMENT.md:24-31`).

### 1.2 Sample cadence: why 5 ms, and the 5.5545 ms caveat
- The files contain no timestamps. Cadence is an **assumption** documented elsewhere:
  - `V6/README.txt:4-8`: "Trajectory cadence used: 5 ms per point".
  - `HTML:367` defaults the sample interval to `5` ms (`getParams` `HTML:729-745`). Every time in the HTML is `rows × sampleMs`.
  - ALIX `EXP01/data/teach_files/MANIFEST.yaml:296-330` (and its J1..J7 entries): the vendor sweep files are named with
    durations that equal **points × 5 ms exactly** (`period_status: from_name`, e.g. "7042 x 5 = 35210 ms"). I recomputed
    all 14 J*-FAST/SLOW files and every one gives 5.000 ms/pt.
  - The record period was never measured. `EXP01/README.md:63-68, 151-155, 277-283` say "record period = replay period" is still open,
    10 ms is refuted, and the timed replay ran at 5.50–5.62 ms per index.
- **Playback is slower than 5 ms.** The upload player ticks at **5.5545 ms nominal (5.4–6.2 measured)**
  (`ALIX/docs/motion/TRACK_CHANNEL_TIMING_REQUIREMENT.md:13-16`). ALIX's manifest gives the executed window as
  `(points−1) × [5.5, 5.6] ms` (`CT/MANIFEST.json:4261` `clock_note`). `tools/dual_arm_verdicts.py:111` uses `TRACK_CLOCK_S = 0.0055545`.
  ⇒ Every "runtime @5 ms" number in the V6 package and the HTML is about **11 % short** on the real arm.

### 1.3 Statistics (computed; durations at 5 ms and at the 5.5545 ms player clock)
Workspace lid-top files (LF, one key, 7 ints, no bad lines):

| file | pts | s @5 ms | s @5.55 ms | max step ° (joint) | ticks over rating @5 / @5.55 ms | min margin to limit ° (J) |
|---|---|---|---|---|---|---|
| zip/01 lid_top_slow_01_idle_trimmed | 7353 | 36.77 | 40.84 | 0.906 (J7) | 0 / 0 | 13.43 (J6) |
| zip/02 …_mean_smoothed_30pct | 2206 | 11.03 | 12.25 | 1.818 (J7) | 116 / 104 | 13.64 (J6) |
| zip/03 …_28pct | 2059 | 10.30 | 11.43 | 1.949 (J7) | 137 / 105 | 13.64 |
| zip/04 …_26pct | 1912 | 9.56 | 10.61 | 2.096 (J7) | 167 / 111 | 13.64 |
| zip/05 …_24pct | 1765 | 8.83 | 9.80 | 2.272 (J7) | 210 / 134 | 13.64 |
| zip/06 …_22pct | 1618 | 8.09 | 8.98 | 2.481 (J7) | 242 / 183 | 13.64 |
| zip/07 …_20pct | 1471 | 7.36 | 8.17 | 2.725 (J7) | 283 / 215 | 13.64 |
| plan_txt lid_top_trimmed_mean_smoothed_28pct | 2252 | 11.26 | 12.50 | 2.002 (J6) | 200 / 147 | 6.94 (J6) |
| …_30pct | 2413 | 12.07 | 13.40 | 1.868 (J6) | 175 / 116 | 6.94 |
| …_35pct | 2815 | 14.08 | 15.63 | 1.602 (J6) | 116 / 77 | 6.94 |
| …_40pct | 3218 | 16.09 | 17.87 | 1.402 (J6) | 69 / 42 | 6.94 |

- Ranges for zip/01 (deg): J1 −163.27..4.12, J2 −105.82..−42.35, J3 −0.09..60.70, J4 6.32..89.53, J5 −28.40..39.46,
  J6 26.23..114.57, J7 −3.94..345.14. Ranges for plan_txt: J1 −36.46..41.42, J2 −98.62..115.71, J3 −64.25..51.85, J4 −75.58..109.67,
  J5 −58.97..16.88, J6 −68.39..121.06, J7 −53.31..−17.51.
- The plan_txt files are a **different recording** from zip (different first point), and their source is absent (≈8,043 pts implied).
  They still contain idle duplicates (21–37 identical pairs, runs of up to 13), so they were trimmed less strictly.
- `UC/upload/` is an older copy of the same zip and plan_txt files plus `rm_trajectory_ui_v8.py`/`rm_upload.py`. `UC/upload.zip` is the same set.
- Bytes are about 57 per point (LF) and 56.8 per point (CRLF). Sizes run from 11.7 KB (205 pts) to 632 KB (10,983 pts).
  `UC/README_COMMANDS.md:45-51` shows a 12,429-point (712.8 KB) upload working.
- Raw (untrimmed) teach lengths for reference, from ALIX `EXP01/data/teach_files/commode_c_taught_paths/FILE_*.txt`:
  41.7–66.0 s per cleaning region and 16.5–23.7 s for the lid/seat mechanisms. These are useful defaults for R1 "cleaning time".

V6 package files (all CRLF, one key, 7 ints). Point counts and @5 ms runtimes match `V6/FINAL_FILE_RUNTIME_REPORT.csv:2-16`, and so does
its `max_per_point_joint_change_deg`:

| file (arm, P) | pts | s @5 ms | s @5.55 ms | max step ° (J) | ticks over rating @5 / @5.55 | isolated spikes | min limit margin ° (J) |
|---|---|---|---|---|---|---|---|
| L_P01P02 (L P1+P2) | 7005 | 35.03 | 38.90 | 1.413 (J6) | 23 / 10 | 1 | 11.84 (J2) |
| L_P03P05 (L P3-P5) | 10983 | 54.92 | 61.00 | 0.922 (J6) | 0 / 0 | 0 | 16.98 (J6) |
| L_P06P07 (L P6+P7) | 6744 | 33.72 | 37.45 | 1.123 (J4) | 6 / 0 | 0 | 13.60 (J2) |
| M3_OPNST01 (L P8) | 455 | 2.28 | 2.52 | 0.862 (J6) | 0 / 0 | 0 | 41.77 (J4) |
| M3_OPNST02 (L P9) | 259 | 1.30 | 1.43 | 0.978 (J3) | 0 / 0 | 0 | 47.46 (J6) |
| L_P10P12 (L P10-12) | 10214 | 51.07 | 56.73 | 1.219 (J6) | 1 / 0 | 3 | 20.18 (J2) |
| R15_BUPFUL (L P13) | 9741 | 48.71 | 54.10 | 1.314 (J6) | 19 / 6 | 6 | 7.12 (J4) |
| R_P01P03 (R P1-P3) | 10428 | 52.14 | 57.92 | 1.335 (J4) | 26 / 3 | 0 | 8.17 (J4) |
| R2_LIDTOP (R P4) | 3751 | 18.76 | 20.83 | 1.594 (J6) | 60 / 37 | 1 | 8.50 (J6) |
| R10_HNGR (R P5) | 3533 | 17.67 | 19.62 | 1.013 (J7) | 0 / 0 | 0 | **1.53 (J4)** |
| R_P06P07 (R P6+P7) | 6744 | 33.72 | 37.45 | **2.454 (J6)** | **172 / 117** | 4 | 11.87 (J4) |
| M3_ASOPN01 (R P8) | 205 | 1.03 | 1.13 | 0.481 (J4) | 0 / 0 | 0 | 25.55 (J4) |
| M3_ASOPN02 (R P9) | 521 | 2.61 | 2.89 | 1.208 (J7) | 6 / 0 | 0 | 22.69 (J4) |
| R_P10P12 (R P10-12) | 10214 | 51.07 | 56.73 | 1.776 (J7) | 58 / 42 | 1 | 13.60 (J6) |
| R_SPILLAGE (R P13) | 5657 | 28.29 | 31.42 | 1.167 (J6) | 6 / 0 | 1 | 21.25 (J4) |

"Isolated spike" means one tick's step is >0.5° and more than 3× both neighbours (`spikes.py`).
The largest step, 2.454°, is 442°/s at 5.55 ms against 225 rated. The bridge test files peak at 0.229°/tick (R6→R7).

### 1.4 Naming conventions and what "30pct" means
- `…_idle_trimmed_mean_smoothed_NNpct.txt` (older pipeline; the tool itself is not in the workspace).
  **NNpct = output point count is NN % of the idle-trimmed input**, so it plays in NN % of the time (30pct ≈ 3.33× faster).
  I reproduced `zip/02` from `zip/01` to within **≤1 mdeg max error**. The steps were the HTML's mean smoothing
  (101-pt centred moving mean, edge-padded, per-joint mean re-centred, endpoints restored; `HTML:1417-1436, 1494-1516`),
  then whole-file linear resampling to `round(N×0.30)` points. Windows of 51 or 151 points give errors near 3°.
  The HTML's own scale uses `round((N−1)×pct)+1` points (`HTML:1568-1590`). That is one point more than the old files, so the two implementations differ slightly.
- The HTML writes `<base>_<trim|mean|scale joined by _ | unchanged>_processed.txt` (`HTML:756-762, 1917`).
  It also writes `<base>_<suffix>_processing_report.json` (`HTML:1732-1756, 1941`) and `…_processed_J1_to_J7.png` (`HTML:1965`).
  The report holds `sourceFile, generatedAt, selectedStages, parameters{sampleMs, idleWindow, idleThreshold, idleMinRun, smoothWindow},
  idleRegions, scalingRegions, rowsAfter*, *TimeSeconds, endpointPreserved`. It records **no** arm, region, pole or hand.
- Default parameters are `idleWindow 11`, `idleThreshold 8 counts/sample`, `idleMinRun 20`, `smoothWindow 101`, and the order Trim → Mean Smooth → Scale
  (`HTML:367-405`). Idle trim keeps the first and last row of each idle block (`HTML:1438-1492`).
- The HTML's "regions" are **equal index chunks** (5–10 of them, `regionCount = clamp(round(n/3000),5,10)`, `HTML:944-1050`).
  They are not commode regions or V6 segment boundaries. "Region" is overloaded, so Lite should call index ranges "segments".
- V6 names: `L_Pxx[Pyy].txt` / `R_Pxx[Pyy].txt` (merged P-segments), `R2_LIDTOP`, `R10_HNGR`, `R15_BUPFUL`, `R_SPILLAGE`,
  `M3_OPNST01/02` (LEFT open seat), `M3_ASOPN01/02` (RIGHT assist open). The prefixes `R2`/`R10`/`R15`/`M3` are **undocumented**.
  `R15_BUPFUL` is a **LEFT**-arm file even though it starts with "R".
- ALIX names: `<slot>_taught_<slug>_<side>` (`TT/README.md:150-160`) and `<slot>_commode_c_<region_key>_<side>_execute_path`
  (`CT/README.md:10`).

---

## 2. Workflow JSON (`realman_workflow_sequence_v1`) and sibling formats

### 2.1 Document
```
{ "format": "realman_workflow_sequence_v1",   // required, exact match (RUN14:1751-1763, UI53:6831-6833)
  "side": "left" | "right",                  // written by UI (UI53:6774-6779); runner ignores it, uses --side
  "workflow_note": "<free text>",             // only in *_FINAL.json; ignored by UI and runner
  "steps": [ <step>, … ] }                   // non-empty list
```
The UI writes a temp copy per stage to `$TMP/realman_dual_sequence_ui/{side}_rule_NN_<pid>.json` and runs
`RUN14 --workflow-sequence-file … --workflow-overall-speed N --side S --yes` (`UI53:10055-10093`).

### 2.2 Step types, with keys and validation (runner `normalize_workflow_step`, `RUN14:1668-1748`; UI builders `UI53:6489-6656`)
| type | keys | validation | runtime behaviour |
|---|---|---|---|
| `program` | `id` (int), `name` (str, informational), `speed` (legacy), `transit_speed` (alias `start_speed`), `dual_rule` (UI only) | id 1–100; transit 1–100 (default 30); legacy speed 1–100 (default 100); dual_rule ∈ {TOGETHER, RUN FIRST, WAIT OTHER} (UI default TOGETHER, `UI53:6532-6548`) | refuses empty slots before starting (`RUN14:2011-2033`); `rm_set_plan_speed(overall)` then `rm_set_program_id_run(slot, transit_speed, 0)` (`RUN14:1857-1873`); **`speed` is not used for playback**; the controller makes its own unchecked movej to the first point at `transit_speed` |
| `pole` | `mm` (float) | 0–315 | `pole.Pole.move` at 100 %, waits until arrival **and** the modelled travel time (`RUN14:865-903`, `UC/pole.py:29-58`) |
| `hand` | `values` (6 ints, or a "a,b,…" string) | each 0–1000; order little, ring, middle, index, thumb_bend, thumb_rot | Modbus ANGLE_SET on all six channels (`RUN14:804`, `UC/hand.py:3-17`); 1000 = open |
| `wait` | `seconds` (alias `wait_s`), `note` (FINAL only) | 0–3600 | sleep; in v53, a **positive WAIT before a TOGETHER program on both arms = hard manual checkpoint** (§2.4) |
| `global` | `name`, `move` (movej\|movel), `speed` (default 20), `wait_s` | speed 1–100; wait 0–3600 | moves to a controller global waypoint (`RUN14:1467`) |
The runner **drops** `dual_rule`, `note`, `workflow_note`, `side` and `name`. Pairing and checkpoints live only in the UI.
On load, the UI back-fills `transit_speed: 30` into old program rows (`UI53:6837-6852`).

### 2.3 Sibling formats in the same app (INT-CTRL)
- `realman_multi_run_sequence` v3 (Page-01 Multi Run): `{"format", "version":3, "programs":[{id, name, hand, pole_mm, wait_s}]}`
  (`UI53:1683-1700`; runner `RUN14:318-363`). `hand` can be `null`, `0..100`/`"50%"` (four fingers at that %, thumbs held), or `"v1,…,v6"`
  (`RUN14:274-316`). `pole_mm` is 0–315. **This is the only existing per-program pre-stroke pole/hand record (R7)**.
- `realman_global_point_sequence` v1: `{"steps":[{name|point_name, move, speed, wait_s}]}` (`UI53:3741`, `RUN14:1561-1612`).
- CLI defaults: `--workflow-overall-speed 100`, `--run-speed 20`, `--restore-speed 50` (`RUN14:2284-2340`).
  UI slot defaults are 25 (`UI53:6933-6934`) and 99 for dual-file runs (`UI53:6966`).

### 2.4 The V6 workflow files, `DUAL_FLOW_TIMELINE_V6.json` and `manual_checkpoint`
- `V6/JSON/` holds **only** `LEFT_V6_70SERIES.json` and `RIGHT_V6_70SERIES.json`.
- `V6/LEFT_V6_70SERIES_FINAL.json` and `V6/RIGHT_V6_70SERIES_FINAL.json` are byte-identical to the copies in `UC/` (`diff` clean).
  FINAL differs from `JSON/` in four ways:
  (a) top-level `workflow_note` "The 2-second WAIT is intentionally retained for manual lid-cover opening." (`…FINAL.json:4`);
  (b) the wait gets `"note":"MANUAL LID COVER OPEN"` (LEFT `:50-53`, RIGHT `:62-65`);
  (c) `"dual_rule":"TOGETHER"` is written explicitly on every non-serial program;
  (d) RIGHT `R_SPILLAGE` speed is 100 in FINAL (`RIGHT…FINAL.json:102-108`) but 50 in `JSON/RIGHT_V6_70SERIES.json:95-100`. This has no effect because the field is legacy.
- FINAL step lists (pole mm, hand values, program id/name/rule):
  - LEFT: pole 200 → hand [0,0,0,0,150,1000] (GRIP preset, `OPS:81`) → pole 150 → **71** L_P01P02 T → pole 50 → **73** L_P03P05 T →
    pole 150 → wait 2 s (checkpoint) → **76** L_P06P07 T → **78** M3_OPNST01 RUN FIRST → **79** M3_OPNST02 RUN FIRST →
    **80** L_P10P12 T → pole 50 → **83** R15_BUPFUL T.
  - RIGHT: pole 200 → hand GRIP → pole 50 → **71** R_P01P03 T → pole 150 → **74** R2_LIDTOP T → pole 50 → **75** R10_HNGR T →
    pole 150 → wait 2 s → **76** R_P06P07 T → **78** M3_ASOPN01 WAIT OTHER → **79** M3_ASOPN02 WAIT OTHER → **80** R_P10P12 T →
    pole 50 → **83** R_SPILLAGE T.
- **`DUAL_FLOW_TIMELINE_V6.json` does NOT exist anywhere on disk.** `find /` finds nothing. It is listed only in `V6/SHA256SUMS.txt:14`
  and named as "the reference for the next dual-scheduler update" (`V6/README.txt:78`).
  `SHA256SUMS.txt:15-18` also lists `LEFT/RIGHT_V6_PART_A_FLOWS_1_5.json` and `…PART_B_FLOWS_6_15/16.json`, which are missing too.
  `README.txt:60-62` says `JSON/` holds `LEFT_V6_FINAL.json`/`RIGHT_V6_FINAL.json`, a third naming.
  `sha256sum -c` shows `README.txt` itself **FAILS** its checksum: it was edited after the sums were written (07:42 vs 07:32).
- **`manual_checkpoint` exists only as prose.** `V6/README.txt:64-67` describes "a top-level manual_checkpoint field". No JSON contains it
  (grep finds only README). v53 implements the checkpoint *implicitly*: a positive WAIT inside the chunk before a TOGETHER program on
  **both** arms becomes `LID-PREP-n` (pole+wait on both) → `LID-CHECK-n` (blocking OK/Cancel dialog "MANUAL LID COVER CHECKPOINT") →
  `TIMELINE-n` (`UI53:9666-9905`; dialog `UI53:10141-10188`). A WAIT on only one arm raises "SAFE TIMELINE checkpoint mismatch".
  Side effect (inferred): any positive WAIT placed at that position becomes an operator gate.
- How v53 plans the FINAL pair (traced through `_page2_program_chunks` `UI53:9619-9664` and `_page2_build_program_rule_plan`):
  TIMELINE-1 {L: 71,73 | R: 71,74,75} → LID-PREP-1 → LID-CHECK-1 → TIMELINE-2 {76 | 76} → serial 78 (L first) → serial 79 (L first) →
  TIMELINE-3 {80,83 | 80,83}. That is 7 stages. Program IDs must be unique per arm. TOGETHER blocks may differ in IDs and counts.
  RUN FIRST/WAIT OTHER needs the **same** ID on both arms (`UI53:9960-9993`). The two per-arm subprocesses are released together by a
  `threading.Barrier(2)` at **launch only** (`UI53:5654-5700, 10209-10229`). Inside a block the arms run **open-loop**.

---

## 3. The V6 merged package: how it was made and what that means

### 3.1 What it contains
- 7 LEFT and 8 RIGHT `.txt` files, 11 bridge test files, 2 CSV reports, README, SHA256SUMS, 2+2 workflow JSONs
  (listing above; runtimes `V6/README.txt:82-98`, `V6/FINAL_FILE_RUNTIME_REPORT.csv:1-16`).
- **No generator code** exists in the workspace (no grep hits for min-jerk/bridge/merge outside README/SHA). The method is documented, not executable.

### 3.2 Merge method (verified against the data)
1. **Timing match by interpolation.** Partner segments were stretched by adding interpolated points so that LEFT and RIGHT P(k) have equal
   length. Nothing was sped up (`V6/README.txt:4-8`). Lengths I located (bridge files found inside merged files at identical rows; `findbridges.py`):

| P | LEFT rows (pts) | RIGHT rows (pts) | note |
|---|---|---|---|
| P1 | L_P01P02 0–3274 (3275) | R_P01P03 0–3274 (3275) | equal |
| P2 | L_P01P02 3527–7004 (3478) | R_P01P03 3527–7003 (3477) | Δ1 |
| P3 | L_P03P05 0–3145 (3146) | R_P01P03 7282–10427 (3146) | equal |
| P4 | L_P03P05 3424–7172 (3749) | R2_LIDTOP (3751) | Δ2 |
| P5 | L_P03P05 7451–10982 (3532) | R10_HNGR (3533) | Δ1 |
| P6 / P7 | L_P06P07 0–3542 / 3865–6743 | R_P06P07 same rows | equal |
| P8, P9 | M3_OPNST01 455, M3_OPNST02 259 | M3_ASOPN01 205, M3_ASOPN02 521 | serial, not matched |
| P10/P11/P12 | L_P10P12 0–3224 / 3277–6694 / 6737–10213 | R_P10P12 same rows | equal |
| P13 | R15_BUPFUL 9741 (48.7 s) | R_SPILLAGE 5657 (28.3 s) | "start together, no finish matching" (`README.txt:23-25`) |

2. **Bridges are minimum-jerk joint-space quintics.** Each file fits `s=10t³−15t⁴+6t⁵` from end(Pk) to start(Pk+1) within 0.5 mdeg.
   A file has `duration/5 ms + 2` points (`V6/BRIDGE_REPORT.csv:1-12`; inserted_points = duration/0.005).
   Reported peak speed = 1.875 × gap / T (L1→L2: 16.671° in 1.25 s → 25.0°/s; R6→R7: 39.08° in 1.6 s → 45.8°/s).
   Durations: 1.25 s (P1→P2), **1.38 s = modelled pole move** (R2→R3, L3→L4, L4→L5), 1.6 s (P6→P7), 0.25 s and 0.2 s (P10→P11→P12).
   Most bridges are near-stationary **holds** (gap 0.006–1.07°; the exceptions are L4→L5 3.92°, L1→L2 16.67° and R6→R7 39.08°). They keep one arm busy while the other's pole moves.
   They start and end at rest while the trimmed segment ends are moving (end steps up to 0.156°/tick ≈ 31°/s).
   That gives a one-tick velocity discontinuity at each join (inferred from data; relevant to R12).
3. **Safety: NOT collision-checked.** "JOINT-SPACE bridges only … NOT collision-checked against the toilet, cubicle walls, second
   robot, end-effector, or right-arm camera" (`V6/README.txt:48-56`).
   The **controller's own transits** between programs are also unchecked and unmodelled (`seqgaps.py`):
   LEFT M3_OPNST02→L_P10P12 **64.6° (J5)**, L_P10P12→R15_BUPFUL 17.25° (J5).
   RIGHT R2_LIDTOP→R10_HNGR **45.7° (J1)**, M3_ASOPN02→R_P10P12 13.65° (J6). All other joins are ≤0.35°.
4. **Splits:** a pole move forces a program boundary. "RIGHT P4 and P5 are NOT merged because pole movement is required between them"
   (`V6/README.txt:33-39`). LEFT P3–P5 stays continuous, and its two 1.38 s bridges are meant to overlap the RIGHT pole moves.
5. **Program-ID +70 remap:** "All Program IDs … shifted by +70" (`V6/README.txt:101-104`). The rule is ID = 70 + the first P in the file
   (71, 73, 74, 75, 76, 78, 79, 80, 83). IDs 72/77/81/82 are unused on both arms, and 73 is LEFT-only while 74/75 are RIGHT-only.
6. **Pole timing model:** 10→300 mm in 4.0 s → 72.5 mm/s → 100 mm ≈ 1.379 s → 1.380 s is used (`V6/README.txt:10-15`).
   **Two models contradict it.** The runner's `pole.py` (slew up 0.059 m/s, down 0.097 m/s, plus a 1.5 s creep; `UC/pole.py:3-14`)
   only returns once `elapsed >= expected` (`UC/pole.py:53`). So a 100 mm move costs at least **3.19 s up or 2.53 s down**.
   ALIX measured UP 75 mm in 2.56–2.66 s and DOWN 75 mm in 1.71–1.73 s (`ALIX/core/python/alix_dispatch/src/alix_dispatch/peripherals.py:100-126`).
7. Manual operation: "After Flow 5: LID OPEN MANUALLY. Then both poles move to 150 mm before Flow 6" (`README.txt:27-31`).
   PPT Task 3 (open lid + seat) and Task 4 (close seat only) are therefore replaced by a manual step.
   Flow numbering: flows 1–7 = P1–P7, 8–11 = L P8, R P8, L P9, R P9, 12–14 = P10–P12, 15/16 = L/R P13 (`README.txt:17-25`).

### 3.3 Where each V6 segment sits (FK; inferred region)
I computed the glove frame (`L/R_glove_frame_2`, ALIX URDF) in `base_link` at the workflow's pole height before each program.
The y axis is positive to the left. The commode is about x 0.475 (front) to 1.005 (back), top ≈0.42–0.43 m, floor ≈0
(`CC/scene.yaml:33-42`; mesh bbox z 0.022–0.433). The commode pose during teaching is **unknown**, so treat these as indicative only.

| arm/P | file rows | pole | glove x / y / z range (m) | inferred region |
|---|---|---|---|---|
| L P1 | L_P01P02 0–3274 | 150 | 0.51–0.96 / −0.07–0.19 / 0.47–1.26 | top/back left; ends (0.92,0.16,0.59) → LEFT HINGE? (weak) |
| L P2 | 3527–7004 | 150 | 0.42–0.95 / −0.03–0.36 / 0.38–0.55 | upper-left rim/side (unassigned) |
| L P3 | L_P03P05 0–3145 | 50 | 0.48–0.82 / −0.02–0.29 / 0.03–0.28 | low front → BASE LEFT (L-R01) |
| L P4 | 3424–7172 | 50 | 0.42–0.77 / −0.05–0.33 / 0.07–0.37 | mid → PEDESTAL UPPER LEFT (L-R03) |
| L P5 | 7451–10982 | 50 | 0.43–0.97 / 0.21–0.31 / 0.09–0.42 | left flank → SIDE EXTERIOR LEFT (L-R02) |
| L P6/P7 | L_P06P07 | 150 | 0.50–1.01 / −0.07–0.45 / 0.30–1.15 | lid open → SEAT TOP 1/2 (or LID UNDERSIDE L) |
| L P8/P9 | M3_OPNST01/02 | 150 | 0.58–0.71 / −0.02–0.28 / 0.36–0.61 | L-T03 OPEN SEAT (MASTER), strong (name + RUN FIRST) |
| L P10–P12 | L_P10P12 | 150 | 0.51–1.03 / −0.07–0.34 / 0.33–1.14 | seat open → RIM TOP 1/2 + SEAT UNDERSIDE LEFT |
| L P13 | R15_BUPFUL | 50 | 0.55–0.82 / −0.08–0.35 / 0.25–0.57 | inside bowl → L-R10 BOWL INSIDE |
| R P1 | R_P01P03 0–3274 | 50 | 0.48–0.80 / −0.29–−0.01 / 0.04–0.62 | low front → BASE RIGHT |
| R P2 | 3527–7003 | 50 | 0.44–0.73 / −0.29–−0.02 / 0.03–0.38 | → PEDESTAL UPPER RIGHT |
| R P3 | 7282–10427 | 50 | 0.53–0.94 / −0.29–−0.16 / 0.09–0.45 | right flank → SIDE EXTERIOR RIGHT |
| R P4 | R2_LIDTOP | 150 | 0.48–0.91 / −0.37–0.14 / 0.47–0.83 | over the lid → R-R01 LID TOP, strong (name + FK) |
| R P5 | R10_HNGR | 50 | 0.44–0.98 / −0.31–0.08 / 0.37–0.49 | to back at top → R-R02 RIGHT HINGE, strong |
| R P6/P7 | R_P06P07 | 150 | 0.48–1.03 / −0.36–0.10 / 0.45–1.15 | lid open → LID UNDERSIDE RIGHT HALF |
| R P8/P9 | M3_ASOPN01/02 | 150 | 0.55–0.94 / −0.26–−0.08 / 0.48–0.70 | R-T03 OPEN SEAT (SUPPORT), strong |
| R P10–P12 | R_P10P12 | 150 | 0.49–1.02 / −0.35–0.05 / 0.36–1.04 | seat open → SEAT UNDERSIDE RIGHT HALF (+?) |
| R P13 | R_SPILLAGE | 50 | 0.29–0.89 / −0.42–0.23 / **0.026**–0.64 | floor → FLOOR SPILLAGE, strong (name + FK) |
Glove z peaks of 1.1–1.26 m inside P1/P6/P10/P12 and R P6/P11 are lift-overs or approaches, not wiping.
They are candidates for R2 trimming and R12 review.
The zip lid_top recording looks LEFT-arm by FK (its y span matches ALIX's left `commode_c_lid_top`) (inferred). The PPT assigns lid top to the RIGHT arm.

### 3.4 Dual timeline (R10): modelled skew in TIMELINE-1
Assumptions: both processes launch together, transits and hand moves are ignored, and both poles start at 200 mm.

| model | P1 start L / R | P3 | P4 | P5 (R−L) |
|---|---|---|---|---|
| V6 design (5 ms, 72.5 mm/s) | 0.69 / 2.07 (+1.38) | +1.38 | +1.37 | +1.37 |
| runner (5.5545 ms, pole.py waits) | 2.02 / 3.05 (+1.03) | +0.04 | +1.69 | **+2.69** s, plus the 45.7° R transit |
Even the design model has a built-in 1.38 s offset, because the first pole moves differ (L 200→150, R 200→50).
After the checkpoint, TIMELINE-2 (P6/P7, equal 6,744 pts, transits ≤0.3°) is well aligned.
TIMELINE-3 starts with an unequal transit (L 64.6° vs R 13.65°).
Each arm's whole program time is 227.0 s @5 ms (LEFT) and 205.3 s (RIGHT), or **252.1 s / 228.0 s @5.5545 ms**, before poles, waits, transits and serial stages.

### 3.5 Implications
- **R8 (merge with a collision scene):**
  (a) concatenate only segments that share pole height and hand state, since pole and hand moves split programs;
  (b) check join continuity (ALIX `tools/plan_min_time.py:142-170` `concatenate`: refuses gaps >0.5° and drops duplicate boundary samples);
  (c) size bridges by peak rate at the **5.5545 ms** clock against the ratings, and match velocity at the boundary instead of starting from rest;
  (d) **collision-check every bridge AND every controller transit** against the commode in the right lid/seat state for that segment
  (`CC/scene.yaml` lid/seat joints, open = −110°), the other arm, the bin and the walls.
  The tools for (d) are `alix_workspace.dual_arm_plan_collision.compute_dual_arm_plan_collisions(..., extra_managers=fixture meshes, left/right_hand_state)`
  (`ALIX/core/python/alix_workspace/src/alix_workspace/dual_arm_plan_collision.py:586-620`) and `cross_arm_separation(..., offset_window_s)` (`:1089-1130`).
  The track→plan shim already exists: `tools/dual_arm_verdicts.py:501` (millideg × π/180000), `:591 build_timeline`, `:634 timeline_to_plan`, `:709 sweep`.
  Its docs say pole and hand seeds are **mandatory** (`:22-33`).
  (e) emit a segment manifest (rows ↔ region/sub-region, pole, hand, lid/seat) plus per-arm workflow JSON **plus the dual timeline file that V6 never produced**;
  (f) allocate program IDs from a declared policy (§5).
- **R10:** the dual view has to plot both arms on one time axis using the player clock, the pole.py durations and estimated transit times.
  It must show TOGETHER/RUN FIRST/WAIT OTHER and the checkpoint stages exactly as v53 builds them.
  It should flag skew above a drift window (ALIX `CT/PAIR_VERDICTS.json` `window_model`: 0.046 s release skew + phase error + 2×172 ppm × length).
  ALIX's rule is that a start-gap movej is done one arm at a time with the partner static (`ALIX/tools/dual_arm_flow.py:3763-3800`).

---

## 4. Region catalogue (Toilet_regions_V1.pptx) mapped to ALIX regions and existing trajectories

**Deck facts.** Slides 1–4 (PAGE 02–05) are overview views: lid closed / lid open / seat open top views, bowl side/front views,
and two labelled photos (slide 3: Lid Top, Lid Underside, Seat Underside, Seat Top, Rim Top, Inner Rim, Bowl Inside, Drain Hole, Hinge;
slide 4: Side Exterior, Pedestal Upper, Base ×2; label PNGs `media/image7-19.png`).
Slides 5–25 are 21 slides holding 22 task steps (slide 24 has Tasks 13 and 14). Each region is a semi-transparent rectangle over a base image, and **colour gives the arm**:
blue `4472C4` = LEFT, orange `ED7D31`/`accent2` = RIGHT, purple `7A73FF` = FL-R09, which sits on the LEFT (PPTX slide5–24 `<a:solidFill>`).
I confirmed each assignment by x-position next to the "LEFT ARM" (x≈5–9 cm) / "RIGHT ARM" (x≈22–25 cm) labels (`pptxpos.py`).
Base images are 1448×1086 PNGs, and the deck is 7.46 MB with 26 media files. The fixture state comes from the base image used
(image1 lid closed, image2 lid open, image3 seat open).

ALIX region keys and ids: `CC/models/regions.yaml` (toplid :523, side :130, front :91, top :211, hinge_area :294, bowl_inside_front :334,
bowl_inside_back :360, bowl_inside_ring :387, bowl_inside_rim :427, deep :479, bottomlid :563, lid_side :634, seat_ring :738,
seat_ring_bottom :772, lid_seat_hinge_area :969, seat_body_hinge_area :1015) and `ALIX/ros/orchestration/alix_orchestrator/config/regions_commode.csv:37-70`.

Status legend: **YES** = a V6 or workspace file matches by name and FK; **INF** = inferred by FK only; **ALIX** = a raw taught .txt exists only in the ALIX repo
(left arm, `TT/` and `EXP01/…/commode_c_taught_paths`); **NO** = nothing taught.

| Task (slide) | state | LEFT (code) | RIGHT (code) | ALIX region / mechanism (id) | Lite/V6 data | ALIX taught .txt | R1 L / R |
|---|---|---|---|---|---|---|---|
| 1A (5) | closed | L-R01 BASE LEFT | R-R01 LID TOP PART 01 | L: — (no base region; nearest `front` 11) · R: `toplid` 0 | L: V6 L P3 · R: R2_LIDTOP (part split unknown); zip/plan lid_top (side undeclared) | R: `40_taught_lid_top_01_right` | INF / YES |
| 2A (6) | closed | L-R02 SIDE EXTERIOR LEFT | R-R01 LID TOP PART 02 | L: `side` 1 · R: `toplid` 0 | L: V6 L P5 · R: R2_LIDTOP | L: `102_…bowl_outside_left`? | INF / YES(part?) |
| 1C (7) | closed | L-R03 PEDESTAL UPPER LEFT | R-R02 RIGHT HINGE | L: — (nearest side/front) · R: `hinge_area` 13 | L: V6 L P4 · R: R10_HNGR | R: `41_taught_lid_hin_01_right` | INF / YES |
| 2 (8) | closed | LEFT HINGE (none) | SIDE EXTERIOR RIGHT (none) | `hinge_area` 13 · `side` 1 | L: V6 L P1 (weak) · R: V6 R P3 | L: `108_…hinge_area_left` | INF / INF |
| 3 (9) | →lid+seat open | OPEN LID + SEAT (MASTER) | OPEN LID + SEAT (SUPPORT) | `lid_open` 2 (RL_compound; `mechanism_tasks/commode_c/lid_open.yaml`) | none; V6 uses a manual lid-open checkpoint | L: `109_…lid_seat_open_left` | ALIX / NO |
| 4 (10) | →seat closed | CLOSE SEAT ONLY (MASTER) | CLOSE SEAT ONLY SUPPORT | `seat_close_left/right.yaml` | none (manual) | L: `110_…seat_only_close_left` | ALIX / NO |
| 5A (11) | lid open | SEAT TOP PART 1 | LID UNDERSIDE RIGHT HALF | `seat_ring` 3 · `bottomlid` 10 | L: V6 L P6 · R: V6 R P6/P7 | L: `103_…seat_top_left` | INF / INF |
| 5B (12) | lid open | SEAT TOP PART 2 | PEDESTAL UPPER RIGHT | `seat_ring` 3 · — | L: V6 L P7 · R: V6 R P2 (pre-lid in V6) | L: 103 | INF / INF |
| 6 (13) | lid open | LID UNDERSIDE LEFT HALF | BASE RIGHT | `bottomlid` 10 · — | L: none (maybe in L P7) · R: V6 R P1 (pre-lid in V6) | — | NO / INF |
| 7 (14) | →seat open | L-T03 OPEN SEAT (MASTER) | R-T03 OPEN SEAT (SUPPORT) | `seat_open` 5 (`seat_open.yaml`) | M3_OPNST01/02 · M3_ASOPN01/02 | (`TT/source/commode_c_seat_open.txt`, not admitted) | YES / YES |
| 8A (15) | seat open | RIM TOP PART 1 | SEAT UNDERSIDE RIGHT HALF | `top` 6 · `seat_ring_bottom` 4 | L: V6 L P10 · R: V6 R P10 | L: `104_…rim_top_left`, `46_taught_rim_top_left` | INF / INF |
| 8B (16) | seat open | RIM TOP PART 2 | FLOOR SPILLAGE PART 1 | `top` 6 · — (floor not modelled) | L: V6 L P11 · R: R_SPILLAGE (one file, parts not split) | L: 104 | INF / YES |
| 9 (17) | seat open | SEAT UNDERSIDE LEFT HALF | FLOOR SPILLAGE PART 2 | `seat_ring_bottom` 4 · — | L: V6 L P12 · R: R_SPILLAGE | — | INF / YES |
| 10A/10B (18-19) | seat open | FL-R09 INNER RIM PART 1/2 | R-T04 PICK DRAIN BRUSH | `bowl_inside_rim` 15 · tool (none) | none | L: `106_…rim_side_left`? | ALIX? / NO |
| 11A/11B (20-21) | seat open | L-R10 BOWL INSIDE PART 1/2 | HOLD BRUSH OUTSIDE | `bowl_inside_ring` 9 / `bowl_inside_back` 8 / `deep` 20 · — | L: R15_BUPFUL (V6 P13) | L: `105_…bowl_inside_left`, `107_…bowl_inner_back_left`, `48_taught_bowl_in_left` | INF / NO |
| 12A/12B (22-23) | seat open | L-R11 FLOOR SPILLAGE LEFT PART 1/2 | R-T05 CLEAN DRAIN HOLE PART 1/2 | — · — (drain hole not modelled; nearest `deep` 20) | none | — | NO / NO |
| 13 (24) | — | DROP GLOVE | R-T06 RETURN DRAIN BRUSH AND DROP GLOVE | ALIX session phases `glove_remove_left/right` | none | — | NO / NO |
| 14 (24) | — | L-T04 LEFT GLOVE CHANGE | R-T07 RIGHT GLOVE CHANGE | `glove_wear_dual` | none | — | NO / NO |
| 15 (25) | →closed | L-T05 CLOSE SEAT + LID (MASTER) | R-T09 CLOSE SEAT + LID (SUPPORT) | `lid_close` 12 (`lid_close_left/right.yaml`; session phase) | none | EXP01 `F3_LIDCL01` (RIGHT, `hardware: analysis_only`, other mount) | NO / NO |

The session phases are listed in `ALIX/ros/orchestration/alix_orchestrator/alix_orchestrator/core/session_phases.py:5-16`.

Notes:
- **Dashboard count (R1):** 22 task steps × 2 arms = 44 arm-actions, counting 10A/10B, 11A/11B and 12A/12B separately.
  **YES 7**: R lid top ×2 parts (one file, R2_LIDTOP), R hinge, L+R open seat, R floor spillage ×2 parts (one file).
  **INF 16**: L 1A, 2A, 1C, 2, 5A, 5B, 8A, 8B, 9, 11A, 11B; R 2, 5A, 5B, 6, 8A.
  **ALIX-only 4** (left arm, a different session; 10A/10B uncertain): L 3, 4, 10A, 10B.
  **NO 17**: L 6, 12A, 12B, 13, 14, 15; R 3, 4, 10A, 10B, 11A, 11B, 12A, 12B, 13, 14, 15.
- V6 ordering and pairing **differ from the PPT**. V6 runs BASE RIGHT and PEDESTAL UPPER RIGHT before the lid opens (R P1/P2),
  and pairs L bowl inside with R spillage (P13). The PPT pairs L bowl inside with R "hold brush". The PPT is newer (Oct 12 vs Oct 5), so treat it as the target plan.
- **Sub-regions (R2):** parts 01/02, 1/2 and left/right halves. Both lid-top parts share the code `R-R01`.
- **Code gaps (inferred numbering):** the uncoded LEFT cleaning regions would be L-R04..L-R08 (left hinge, seat top, lid underside L, rim top,
  seat underside L), then FL-R09 = L-R09. RIGHT uncoded would be R-R03..R-R08. Tasks L-T01/L-T02 and R-T01/R-T02 (open lid+seat, close seat only)
  and R-T08 (hold brush) are missing.
- **Fixture state per region agrees with ALIX** `commode_fixture_type`: closed for toplid/side/hinge, `lid_open` for seat_ring/bottomlid,
  `lid_seat_open` for top/seat_ring_bottom/bowl_* (from `CC/*_cleaning_points.yaml` `task_parameters`, e.g. `toplid_right…:1002-1006`).
- **R7 presets that exist:**
  - ALIX config stages per region×arm (`CC/*_cleaning_points.yaml` `stage_sequence`, e.g. `toplid_right…:1749-1791`).
    Pole is quarter 0.075 m for most regions, half 0.15 m for bottomlid and seat_ring_bottom, full 0.30 for bowl_inside_rim_left.
    Hand is `open_tenth` [129,129,129,129,133,944] for most, `quarter_grasp` [750,…,133,1000] for hinge, `half_grasp` [516,…,133,944] for rim.
    Pole names map to metres in `ALIX/ros/manipulation/alix_moveit_config/config/group_states/poles_group_states.yaml:11-30`
    (0.01/0.075/0.15/0.225/0.30), hand states are in `alix.srdf:659-714`, and frame→hand is in `ALIX/core/python/alix_taskgen/src/alix_taskgen/config/glove_frames.yaml:97-109`.
  - The **taught** sheets (left arm, real teaching) are in `EXP01/…/commode_c_taught_paths/description.txt:3-16`.
    Lid top, bowl outside, seat top and rim top use 140 mm with the hand 100 % open. Bowl inside uses 200 mm with 100 % open.
    Rim side, bowl inner back, hinge, lid-seat open and seat-only close use 200 mm with 50 % open. "Hand %" means the first four fingers, thumb held.
  - V6 uses 50, 150 and 200 mm with GRIP [0,0,0,0,150,1000] throughout.

---

## 5. Gaps and inconsistencies
1. **Identifier soup.** The same work has seven numbering schemes: PPT task ids (1A, 2A, 1C, 2 …, with no 1B and 2A placed before 1C), PPT codes (partial),
   V6 P1–P13, V6 flows 1–16, program IDs (P+70), ALIX region ids/keys, and ALIX slots. Lite needs one canonical region/sub-region id that maps to each of them.
2. **Program-ID collisions.** V6 uses 71–83 per arm, which overlaps ALIX `commode_c` slots 70–81, 101 and 114 (`CT/README.md:1-24`, `CT/MANIFEST.json:4259`)
   and the ALIX "production range 70–100" (`TT/README.md:164-168`). Taught slots are 40–59 (`UC/README_COMMANDS.md:58`), yet the ALIX taught rows use 101–113.
   UI defaults are 25 and 99. The controllers are the same machines (`RUN14` ARM_IPS 192.168.1.10/.103).
3. **No per-file metadata.** Nothing records side, region, sub-region, pole, hand, lid/seat state, taught date/operator, source sha, clock or edit history.
   V6 pole/hand are implicit in the workflow order. ALIX's `SOURCE.json` (side, frame, pole_m, hand_state, hand_counts, hardware, taught_by, date, units, note;
   `TT/source/README.md:8-27`) and `pre_stroke` (`CT/MANIFEST.json:43-69`) are the models to copy.
4. **Missing or broken V6 artifacts.** `DUAL_FLOW_TIMELINE_V6.json` and the PART_A/B JSONs are absent, README names the wrong JSON files,
   README fails its checksum, `manual_checkpoint` does not exist, and JSON/ vs FINAL differ (§2.4).
5. **The V6 sources are gone.** Only merged and stretched outputs remain, so per-region re-editing (R2) would work on data that is already optimized and time-stretched.
   No raw drag file exists in the workspace. zip/01 is already idle-trimmed.
6. **Timing models conflict:** 5 ms vs 5.5545 ms; pole 1.38 s vs ≥2.53/3.19 s; transits unmodelled (§3.4).
7. **Rate and limit issues in shipped files.** Ticks exceed the ratings at the player clock in R_P06P07 (117 ticks, peak 2.454°/tick J6), R2_LIDTOP (37), R_P10P12 (42), L_P01P02 (10).
   Isolated single-tick spikes appear in R_P06P07 (row 5058 J3 1.049°), L_P10P12 (row 9815 J6 1.219°), R_P10P12 (row 1049 J4 1.161°) and R15_BUPFUL (6).
   **R10_HNGR comes within 1.53° of the J4 limit** (133.47° vs 135°); ALIX refuses margins under 5° (`TT/README.md:106-107` "REJECT").
   The speed-ups in zip/02–07 and plan_txt exceed ratings for 69–283 ticks.
8. **Vocabularies differ.** Pole heights: V6 50/150/200 mm, ALIX named 10/75/150/225/300, ALIX taught 20/140/177/200/300.
   Hand: the upload app writes all six channels, while ALIX uses `fingers_pct` with thumbs HOLD (`TT/README.md:117-126`).
9. **PPT gaps.** There are no cleaning times. Region codes are missing for most cells. "Base" and "Pedestal Upper" have no ALIX mesh region.
   Floor spillage, drain hole and the drain-brush tool tasks are outside the ALIX commode model, so the R3/R10 sim needs floor and brush props.
   ALIX `regions_commode.csv` repeats id 28 (`:63` toplid_zone, `:65` lid_top).
10. **Environment risk:** `alix_workspace` cannot be imported with system Python here (scipy/numpy ABI clash). Lite must pin its environment.

---

## Requirement relevance
- **R1**: the PPT gives 22 tasks / 44 arm-actions with colour-coded arms and fixture states (§4). The dashboard needs a catalogue file plus a per-recording
  sidecar. Raw files are named by side and timestamp only (`UI53:3840-3850`). Default cleaning times can come from raw teach lengths (40–66 s/region, §1.3).
  The Regions tab can redraw slides from base images plus rectangle overlays.
- **R2**: the HTML algorithms are verified (30pct reproduced). Sub-regions should be true segment rows (§3.2 table), not the HTML's equal chunks.
  The processing report can serve as the edit-provenance record.
- **R3/R14**: the sim scene must take lid/seat state per region (ALIX `commode_fixture_type`) and pole height per program. Floor and drain props are missing.
- **R4**: IPs are 192.168.1.10 / 192.168.1.103 (`OPS:5-6`, `RUN14` `ARM_IPS`, env `RM_LEFT_IP`/`RM_RIGHT_IP`).
- **R5**: map a collision sample to a plot index with `row = t / 0.0055545` (player) or `t/0.005` (design). Report both.
- **R7**: there is no per-region preset store. Use ALIX `pre_stroke`/`stage_sequence` presets, the taught sheet, and the multi-run `pole_mm`/`hand` fields (§2.3, §4).
- **R8**: §3.5. Merging must split on pole/hand changes, bridge with velocity-matched rate-limited quintics, and collision-check bridges and transits with alix_workspace.
- **R9**: watch joint-limit margins (R10_HNGR at 1.53°), wrap J7 at ±360 (zip J7 reaches 345°), and use the glove_frame_2 FK frame.
- **R10**: §2.4 and §3.4. Use the v53 SAFE TIMELINE semantics, the player clock, and pole.py durations.
- **R11/R12**: over-rating tick and isolated-spike tables (§1.3, §5.7). Bridge joins have velocity discontinuities.
- **R13**: file sizes are 11.7–632 KB. The upload is `rm_send_project` with size readback. Names truncate at 32 chars.
- **X1/X2**: all data is joint-space, and FK is used here only for diagnosis. That is consistent with the exclusions.
- **NFR-UX**: operators must not see P-numbers or program IDs. Show region names, PPT thumbnails and plain "lid open?" checkpoints instead.
- **INT-CTRL**: v53 + v14 is the baseline, not v48. Workflow, multi-run and global-point schemas are in §2.
- **INT-TRAJ**: HTML I/O is LF NDJSON with 7-int rows. It assumes 5 ms and has no side/region awareness.

## Reuse verdicts
| feature | verdict | what to lift | evidence |
|---|---|---|---|
| NDJSON track reader / project writer | reuse-as-is | `read_track`, `write_project` (normalise EOL, CRLF project) | `RUN14:76-97` |
| Trim / mean-smooth / scale algorithms | reuse-as-is | `trimIdle`, `meanSmooth`, `interpolateRegion`, `applyRegionScales`; agree one point-count rule | `HTML:1438-1616`; recon ≤1 mdeg |
| Processing report sidecar | reuse-with-adaptation | add side, region, sub-region, pole, hand, clock, undo stack | `HTML:1732-1756` |
| `realman_workflow_sequence_v1` | reuse-with-adaptation | keep step types; add explicit `checkpoint`, region refs, timeline id | `RUN14:1668-1763`, `UI53:6489-6779` |
| v53 SAFE TIMELINE planner | reuse-with-adaptation | `_page2_program_chunks`, `_page2_build_program_rule_plan`, checkpoint stage | `UI53:9619-10017, 10141-10188` |
| Multi-run pole/hand per program | reuse-with-adaptation | `pole_mm`/`hand` row fields as the R7 runtime carrier | `RUN14:274-363` |
| V6 merge method (timing match, min-jerk, +70) | reference-only | method only; reimplement with collision checks and velocity-matched joins | `V6/README.txt`, `BRIDGE_REPORT.csv` |
| V6 trajectories and FINAL JSONs | reuse-with-adaptation | seed library and regression data; re-label regions and slots | `V6/LEFT`, `V6/RIGHT`, `*_FINAL.json` |
| DUAL_FLOW_TIMELINE_V6.json / manual_checkpoint | not-available | — | `SHA256SUMS.txt:14`, `README.txt:64-78` |
| Pole timing model | reuse-as-is | `lift_travel_s` (both copies agree) | `UC/pole.py:11-14`, `ALIX/core/python/alix_dispatch/src/alix_dispatch/peripherals.py:121-140` |
| ALIX region registry | reuse-with-adaptation | keys, ids and lid/seat states; add base, pedestal, floor, drain, tool tasks | `CC/models/regions.yaml`, `regions_commode.csv` |
| ALIX pre_stroke and stage presets | reuse-with-adaptation | per-region pole/hand defaults | `CT/MANIFEST.json:43-69`, `CC/*_cleaning_points.yaml` |
| ALIX SOURCE.json provenance | reuse-with-adaptation | per-recording sidecar schema | `TT/source/README.md:8-27` |
| ALIX taught .txt (left, commode_c) | reuse-with-adaptation | prefill the R1 dashboard and test data (different session) | `TT/TAUGHT_TRACKS.json`, `EXP01/…/description.txt` |
| Dual-arm collision and drift gate | reuse-with-adaptation | `compute_dual_arm_plan_collisions`, `cross_arm_separation`, `dual_arm_verdicts` shim | `dual_arm_plan_collision.py:586,1089`; `dual_arm_verdicts.py:501-709` |
| Join continuity contract | reuse-as-is | `concatenate` (≤0.5° gap, drop duplicates) | `ALIX/tools/plan_min_time.py:142-170` |
| PPT deck | reuse-with-adaptation | base images, rectangles, colour to arm, fixture state | PPTX slides 1–25, media |
| HTML equal-chunk "regions" | reference-only | replace with real segment rows | `HTML:944-1050` |
| `schedule_commode.csv` format | reference-only | pairing syntax ideas (paired / serial / chained) | `ALIX/…/schedule_commode.csv:17-25` |
