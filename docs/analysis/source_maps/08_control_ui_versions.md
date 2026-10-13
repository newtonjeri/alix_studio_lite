# 08 — Upload-channel control UI: version lineage, verdict and inventory

Scope: INT-CTRL, R4, R7, R13 (plus touches on R1, R5, R8, R10, NFR-UX, NFR-THEME).
Method: md5/line counts, `diff` of consecutive versions, targeted `sed -n` reads, `grep` for
functions/labels/flags, and a read-only simulation of the Page-02 dual planner (the two pure
methods `_page2_program_chunks` + `_page2_build_program_rule_plan` were pulled out with `ast` and
fed the V6 FINAL JSONs and synthetic step lists; no robot/network/Tk involved).

Path legend (used in every citation below)

| Key | Absolute path |
|---|---|
| UC | `/home/newtonjeri/alix_studio_lite_ws/upload_channel` |
| PP | `"/home/newtonjeri/alix_studio_lite_ws/upload_channel/py programs"` (note the space) |
| v48 | `UC/rm_trajectory_ui_v48_handoff_continue_fix.py` (also `PP/...v48...`, byte-identical) |
| v51 | `UC/rm_trajectory_ui_v51_safe_start_speed.py` |
| v52 | `UC/rm_trajectory_ui_v52_timeline_rules.py` |
| v53 | `UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py` |
| r12 | `PP/rm_upload_fast_v12_fast_workflow.py` |
| r13 | `UC/rm_upload_fast_v13_overall_speed.py` |
| r14 | `UC/rm_upload_fast_v14_safe_transit.py` (= `_v52support.py` = `_v53support.py`, byte-identical) |
| V6 | `/home/newtonjeri/alix_studio_lite_ws/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES` |
| OPS | `"/home/newtonjeri/alix_studio_lite_ws/RealMan Dual-Arm Operations UI.txt"` |
| ALIX | `/home/newtonjeri/alix_ws/src/alix` |

---

## 0. Verdict (read this first)

* **v48 is NOT the latest nor the most comprehensive.** Three newer UIs exist in UC: v51 (Oct 5 09:42),
  v52 (16:48), v53 (17:09); v48 is Oct 5 05:48 (`ls -la UC`). The v48 in UC and PP are the same file
  (md5 `3196209593964a476eb9f91566be09c0` both).
* **Most comprehensive pair = UI v53 `rm_trajectory_ui_v53_safe_lid_checkpoint.py` + runner v14
  `rm_upload_fast_v14_safe_transit.py`.** v53 declares `UI_PROGRAM_VERSION = "v53"`,
  `EXPECTED_RUNNER_VERSION = "v14"` (v53:45-46) and auto-selects the v14 file first (v53:6890-6910).
  The three v14 files share md5 `a0b285e6336be2feac880e5dbe381db6`.
* v53 is a strict superset of v48 by functions, button labels and runner flags (programmatic
  `comm` of `def`/`text=`/`"--flag"` sets: the only v48-only symbol is the inner helper `unique_map`,
  replaced by the timeline planner). Nothing user-visible was removed v48→v53.
* **Decisive practical evidence:** the production merged package (V6/LEFT_V6_70SERIES_FINAL.json +
  RIGHT_V6_70SERIES_FINAL.json, also copied in UC) is **rejected by v48 and v51** in RUN BOTH
  ("PROGRAM RULES matches LEFT and RIGHT by Program ID … Missing on RIGHT: 73 | Missing on LEFT: 74,
  75"), accepted by v52 (no lid gate) and accepted by v53 with the manual lid checkpoint
  (plan: TIMELINE-1 L[71,73] R[71,74,75] → LID-PREP-1 → LID-CHECK-1 → TIMELINE-2 [76] → serial 78
  (LEFT first) → serial 79 (LEFT first) → TIMELINE-3 [80,83]). See §1.8.
* Behavioural (not feature) changes v48→v53 that a planner must know: per-Program "Speed %" became
  "Start %" (transit only, default 30, old saved speeds discarded); TOGETHER no longer re-syncs the
  arms at every matching Program ID (v52+); a positive WAIT is reinterpreted by v53 as a manual
  "lid cover" checkpoint and a WAIT on only one arm inside a TOGETHER section is rejected. See §1.6.
* Features that existed in **older** versions and are missing from v53 (and v48): per-arm Drag
  "SAVE TO CONTROLLER" (DT_ program) and "RUN DRAG .TXT" (v26–v32), the dual-run REAL-mode warning
  (v32), the single "% open" hand slider (v18–v38), and the Page-02 MODE selector PARALLEL /
  LEFT→RIGHT / RIGHT→LEFT / ALT PROGRAMS (v44–v46; ~520 lines of dead code still in v53). See §2.

---

## 1. Diffs v48 → v51 → v52 → v53 (and runner v12 → v13 → v14)

### 1.1 File identity

| File | md5 | lines | bytes | mtime |
|---|---|---|---|---|
| v48 (UC and PP) | 3196209593964a476eb9f91566be09c0 | 11068 | 354151 | Oct 5 05:48 |
| v51 | 6a74041b968ffdb4c7ba1bb2740f1da3 | 11295 | 362990 | Oct 5 09:42 |
| v52 | bbe8c94f49d4daac53df02f2f4b70f56 | 11362 | 365719 | Oct 5 16:48 |
| v53 | adab5030163ee0a62ee505682bcfd7a5 | 11581 | 375502 | Oct 5 17:09 |
| r12 (PP) | 19046f24b6099a33c6394aca9cf20015 | 2646 | 88987 | Oct 1 15:08 |
| r13 | 9f86ef9aaf0e824fb6e318d2fc756ff1 | 2683 | 90177 | Oct 5 05:48 |
| r14 ×3 | a0b285e6336be2feac880e5dbe381db6 | 2702 | 91105 | 09:42 / 16:48 / 17:13 |

Diff sizes: v48→v51 330 diff lines; v51→v52 268; v52→v53 415 (`diff | wc -l`).

### 1.2 v48 → v51 ("safe start speed" + version bar + live status) — line refs in v51

Added
* Docstring title "RealMan Trajectory UI v51" (v51:3) and constants `UI_PROGRAM_VERSION="v51"`,
  `EXPECTED_RUNNER_VERSION="v14"` (v51:45-47).
* Persistent **SOFTWARE VERSION bar** above both pages: `_runner_version_from_path`
  (regex `_v(\d+)` on the runner filename), `_refresh_version_banner` (UI version + UI file + runner
  version + runner file + Python version), `_on_runner_path_changed` trace (v51:6922-6929,
  7391-7424, bar widgets 7426-7454).
* **LIVE ARM STATUS** frame on Page 02 with `page2_left_live_var/page2_right_live_var`
  (v51:6979-6981, 9155-9178), setters `_page2_set_live` / `_page2_program_live` (v51:9792-9801) and
  state strings pushed from every rule stage: READY, RUNNING, RUNNING // RUN FIRST, WAITING FOR
  LEFT/RIGHT, WAITING // CONFIRMATION REQUIRED, WAITING // AUTO HANDOFF, HANDOFF CANCELLED,
  RUNNING // SECOND, COMPLETE ✓, FAILED rc=…, NOT STARTED, FINAL AUX // RUNNING, PREPARING,
  STOP REQUESTED, SEQUENCE COMPLETE ✓ (v51:9822-10368, 10787-10788, 10891-10892).
* Runner candidate `rm_upload_fast_v14_safe_transit.py` put first (v51:6891).

Changed
* Page-02 Program row: label "Speed %" → **"Start %"**, var `program_speed_var` (default "20") →
  `program_transit_speed_var` (default "30") (v51:6090-6093, 6176-6185); validation message
  "Start / Transit speed must be 1-100%" (v51:6510-6519); help text (v51:6221-6223, 7562-7564).
* Program step JSON now writes `"speed": 100` (legacy, ignored) + `"transit_speed": N`
  (v51:6540-6546); table text "Start N% // Stored timing // RULE" (v51:6670-6675).
* JSON load back-compat: Program rows lacking `transit_speed`/`start_speed` get `transit_speed=30`
  regardless of their old `speed` (v51:6836-6851).
* Page-02 kicker "… // SAFE START + LIVE STATUS" (v51:9043).

Removed: nothing user-visible.

### 1.3 v51 → v52 ("TIMELINE RULES") — line refs in v52

Changed (only the planner and its texts)
* `_page2_build_program_rule_plan` rewritten (v52:9666-9855): per-arm uniqueness `validate_unique`
  (9693-9705); `rule()` (9707-9708); `combine()` merges consecutive TOGETHER chunks of one arm into
  one runner document with `program_ids`, `program_name` "A + B" (9710-9728); loop builds
  `{"kind":"parallel","program_id":"TIMELINE-n","left_program_ids":[…],"right_program_ids":[…]}`
  stages where **LEFT and RIGHT may have different Program IDs and counts** (9730-9778); error if a
  TOGETHER section exists on one arm only (9755-9764); unmatched serial section error (9780-9796);
  serial RUN FIRST/WAIT OTHER still needs the **same** Program ID on both arms (9797-9833).
* Button "▶▶ RUN BOTH // TIMELINE RULES" (v52:9089); docstring (v52:10775-10777); confirmation text
  "DUAL TIMELINE-RULE EXECUTION … Program IDs may differ inside those blocks" (v52:10834-10841).
Removed: the v48/v51 rule that LEFT/RIGHT TOGETHER Programs are paired **by matching Program ID**
(v51 "Missing on RIGHT/LEFT" check, v51:9705-9723) — intended, see V6/README.txt:69-80.

### 1.4 v52 → v53 ("SAFE TIMELINE" + manual lid checkpoint) — line refs in v53

Added
* `positive_wait_index` (v53:9717-9729), `combine(…, label)` (9730-9749), `split_wait_chunk`
  (9751-9790): a positive WAIT before a TOGETHER Program on BOTH arms becomes a **hard cross-arm
  checkpoint**: finish all prior timeline Programs on both arms → run the setup steps up to the WAIT
  on both arms (`LID-PREP-n`) → `{"kind":"checkpoint","program_id":"LID-CHECK-n","title":"MANUAL LID
  COVER CHECKPOINT","message":…}` → the Program after the WAIT on both arms (v53:9798-9916).
* Errors: "SAFE TIMELINE checkpoint mismatch" when the WAIT exists on one arm only (9827-9833);
  "checkpoint alignment is ambiguous" (9839-9847); Program after checkpoint must be TOGETHER on both
  (9868-9872); checkpoint chunk must lead to exactly one Program (9772-9778).
* Stage dispatcher `elif kind == "checkpoint"` (v53:10136-10137) and
  `_start_page2_manual_checkpoint` (v53:10141-10188): robots idle, live status "PAUSED // MANUAL
  LID CHECK", `messagebox.askokcancel(… icon="warning")` with hard-coded text "Open the toilet lid
  cover manually … Both hands are completely outside the robot workspace … Both robot paths are
  clear" (v53:9886-9904); Cancel ends the sequence (v53:10168-10175).
* Confirmation dialog shows "Manual safety checkpoints: N" (v53:11044-11060); button "▶▶ RUN BOTH //
  SAFE TIMELINE" (v53:9089).
Removed: nothing.

### 1.5 Runner r12 → r13 → r14

* r13 adds `--workflow-overall-speed` (default 100, r13:2316-2317, validated 2398-2400);
  `rm_set_plan_speed(overall)` once at workflow start (r13:2096-2114) and again before every stored
  Program (r13:1842-1852). Program transit = step `speed` (default 20) passed to
  `rm_set_program_id_run(slot, int(speed), 0)` (r13:1679, 1860).
* r14 ("safe transit"): Program steps read `transit_speed` (fallback `start_speed`, default 30) and
  keep legacy `speed` only for JSON compatibility (r14:1679-1699); call becomes
  `rm_set_program_id_run(slot, int(transit_speed), 0)` (r14:1873); log text "START/TRANSIT N% //
  stored trajectory timing" (r14:1868-1869, 2005-2009, 2079-2080, 2168). Nothing removed.
* Runner lineage is strictly additive (function/arg `comm` over rm_upload.py → v4 → v5 → v6 → v7 →
  v9 → v10 → v11 → v12 → v13 → v14): v4 `--sequence/--sequence-file`; v5 `--manual-hand/--manual-pole`;
  v6 `--drag-session/--drag-save/--drag-playback`; v7 `--drag-store/--drag-run-file/--drag-slot/
  --drag-name`; v9 global points + `--get/--set-arm-mode`; v10 `--manual-hand-raw`; v11
  `--workflow-sequence-file`; v12 `run_workflow_program_fast`, `workflow_quick_settle`; v13
  `--workflow-overall-speed`; v14 transit semantics. No v8 runner file exists although UIs list
  `rm_upload_fast_v8_run_mode.py` as a candidate (v53:6897).
* The base `PP/rm_upload.py` is the monorepo `ALIX/tools/rm_upload.py` (diff = 10 lines, usage text
  only), so r14 is an extended fork of the monorepo tool.

### 1.6 Did v51–v53 remove or regress any v48 feature?

No feature removal. Semantic changes/regressions (all verified by code + planner simulation):
1. **Old per-Program speeds are discarded.** v48 saved `speed` (default 20) and r13 used it as the
   transit speed (r13:1679,1860). v51+ load assigns `transit_speed=30` when missing (v51:6836-6851)
   and r14 ignores `speed` (r14:1679-1699). Example: `V6/JSON/RIGHT_V6_70SERIES.json` had
   `"speed": 50` for Program 83; under v14 it transits at 30%.
2. **Version-mismatch hazard (new in v51):** v51+ writes `"speed": 100` (v53:6540-6546 area,
   `_parse_program` v53:6489-6548). If the operator browses to r13 (allowed; the version is only
   displayed, `EXPECTED_RUNNER_VERSION` is used only as a banner fallback v53:7394), r13 reads
   `speed` → transit at **100%** (r13:1679,1860). No handshake/enforcement exists.
3. **TOGETHER re-sync granularity (v52+):** v48/v51 made one parallel stage per matching Program ID
   (arms re-synchronised at every Program boundary). v52/v53 merge consecutive TOGETHER rows into one
   runner document per arm and synchronise only at block start/end. Simulation "C" (L=[1,2,3],
   R=[1,2,3]): v48 → 3 parallel stages; v52/v53 → 1 stage `TIMELINE-1`.
4. **WAIT semantics (v53):** a WAIT on one arm inside a TOGETHER section is now rejected
   (simulation "A": v48/v52 accept, v53 "SAFE TIMELINE checkpoint mismatch"); a WAIT on both arms
   always triggers the hard-coded *lid cover* dialog (simulation "B": LID-PREP-1, LID-CHECK-1, …),
   i.e. WAIT can no longer be used as a plain settle/pause on both arms.
5. Within TOGETHER blocks the live status shows "Program TIMELINE-n // RUNNING" instead of a Program
   number (v53:10193 `pid = stage["program_id"]`, used at 10215-10216).

### 1.7 v49 / v50

No v49/v50 files exist (`ls UC`, `ls PP`). v51–v53 never mention v49/v50 (grep of `v49|v50`: only
"v51", "v53", "v47", "v41", "v29", "v20" comments — v53:6545, 6837, 9427, 4429, 11461, 6889).
OPS describes "UI v50 / Runner v13" (OPS:222-224) including the version bar (OPS:213-225) and live
status (OPS:153-167) — both first appear in the v51 file, together with Start % (not in OPS).
**Inference:** v51 = the unsaved v50 (version bar + live status) + Start %/runner v14; v51 does not
claim this explicitly. OPS is therefore stale vs v53 (no Start %, timeline or lid checkpoint; OPS:115-124
still says Programs are paired by matching ID).

### 1.8 Planner simulation (read-only; methods extracted with `ast`)

| Input | v48 | v51 | v52 | v53 |
|---|---|---|---|---|
| V6 FINAL L/R JSONs | REJECTED (Missing on RIGHT: 73; on LEFT: 74, 75) | REJECTED (same) | TIMELINE-1 L[71,73,76] R[71,74,75,76]; serial 78; serial 79; TIMELINE-2 [80,83] — **no lid gate** | TIMELINE-1 L[71,73] R[71,74,75]; LID-PREP-1; LID-CHECK-1; TIMELINE-2 [76]; serial 78; serial 79; TIMELINE-3 [80,83] |
| A: L=[WAIT1,P1] R=[P1] | parallel 1 | – | TIMELINE-1 | REJECTED (checkpoint mismatch) |
| B: both [POLE100,WAIT1,P1,P2] | parallel 1, parallel 2 | – | TIMELINE-1 | LID-PREP-1, LID-CHECK-1, TIMELINE-1, TIMELINE-2 |
| C: both [P1,P2,P3] | 3× parallel | – | TIMELINE-1 | TIMELINE-1 |
| D: [P1, P8 RUN FIRST/WAIT OTHER, P10] | parallel 1, serial 8, parallel 10 | – | TIMELINE-1, serial 8, TIMELINE-2 | same as v52 |
| E: L=[P1,P3] R=[P1,P4,P5] | REJECTED | – | TIMELINE-1 | TIMELINE-1 |

V6/README.txt:69-80 documents exactly this: "The existing v51 'RUN BOTH // PROGRAM RULES' … cannot
execute the complete optimized overlapping timeline … Do NOT force these JSONs through current RUN
BOTH Program Rules". In v52 the LEFT arm could enter P76 (post-lid) while RIGHT is still in 74/75;
v53 closes that gap (v53:9672-9688 docstring).

---

## 2. Lineage in PP — features added and features dropped

Sizes: v8 1755 → v13 3341 → v17 4130 → v18 4354 → v21 4793 → v22 5249 → v23 5277 → v24 5396 → v25 5844 →
v26 6188 → v28 7287 → v29 7341 → v31 7368 → v32 7709 → **v35 7259 (−673/+223)** → v36 7262 → v37 7689 →
v38 8319 → v39 8372 → v40 8433 → v41 9692 → v42 9716 → v43 9763 → v44 10043 → v45 10388 → v46 11112 →
**v47 11010 (−148/+46)** → v48 11068.

First appearance of key functions (`grep -l "def X"` across PP):

| Feature | First version | Function |
|---|---|---|
| Upload / upload+run / run slot / run by ID-or-name / list | v8 | PP/rm_trajectory_ui_v8.py:1-22 docstring |
| Batch upload + Multi Run + save/load sequences | v13 | `_batch_upload_worker`, `_multi_run_worker` |
| Multi Run editable Hand/Pole/Wait table | v17 | `_begin_cell_edit` |
| Manual hand (% open, first 4 fingers) + pole | v18 | `_manual_hand_slider_changed` ("HAND // First 4 fingers", "APPLY HAND ONLY") |
| Two side-by-side arm panels | v21 | `_build_panel_ui`, `has_active_command` |
| Synchronized dual run (Barrier) | v22 | `_start_process_synced`, `_run_both_programs` |
| Separate LEFT/RIGHT files | v23 | `_browse_left_track` (later `_browse_arm_track`, v24) |
| Drag & Teach record/save/playback | v25 | `_drag_teach_toggle` |
| Drag → controller program (DT_), run drag .txt | v26 | `_drag_save_to_controller`, `_drag_run_txt` |
| Global points + SIM/REAL/CHECK mode | v28 | `_run_global_point_sequence`, `_set_arm_mode_ui` |
| Tabbed "user friendly" layout | v29 | `compact_button_style` (sections 01–07 became tabs) |
| Dual RUN FILES (upload both then run) + REAL-mode warning | v32 | `_run_both_selected_files`, `_dual_real_mode_warning` |
| Common START/STOP BOTH DRAG | v35 | `_start_both_drag` |
| Save-both-recordings popup | v37 | `_show_dual_drag_save_popup` |
| RUN BOTH .TXT popup | v38 | `_run_both_txt_files` |
| 6-channel hand | v39 | `_manual_hand_channel_slider_changed` |
| GRIP / ALL OPEN presets | v40 | `_manual_hand_apply_preset` |
| Page 02 dual sequence builder | v41 | `_build_sequence_page` |
| Overall speed % | v43 | `_page2_overall_speed` |
| One arm at a time (LEFT→RIGHT / RIGHT→LEFT) | v44 | `_start_page2_serial_first` |
| Alternate programs (ALT L→R / R→L), CONFIRM EACH | v45 | `_start_page2_alt_programs` |
| Per-program Dual Rules TOGETHER/RUN FIRST/WAIT OTHER | v46 | `_page2_build_program_rule_plan` |
| rc-based completion (`last_command_rc`) | v48 | `_page2_command_succeeded` |

**Dropped features (flag for Lite):**
1. **v35 dropped per-arm "⬆ SAVE TO CONTROLLER" and "▶ RUN DRAG .TXT"** with Controller ID / Name
   fields (`_drag_save_to_controller`, `_drag_run_txt`, `_drag_program_details`,
   `_ensure_drag_file_for_run`; e.g. PP/rm_trajectory_ui_v32_dual_files_fixed.py
   `_drag_save_to_controller` building `--drag-store PATH --drag-slot N --drag-name NAME`). Never
   restored (count of these defs in v35/v38/v48/v53 = 0). r14 still implements `--drag-store`,
   `--drag-run-file`, DT_ naming (r14:992-1172, 2309-2316); v53 still labels DT_ programs as
   "[DRAG TEACH]" (v53:4738-4769) but has no button to create them. Workaround in v53: SAVE LOCAL then
   UPLOAD ONLY / UPLOAD + RUN (no DT_ prefix).
2. **v35 dropped `_dual_real_mode_warning`** (v32): dual-run confirmation warned "MODE WARNING …
   If a controller is in SIMULATION mode, the physical arm will not move" when either badge was not
   "MODE // REAL". Never restored. Relevant to R4 (real/sim visibility).
3. v35 dropped "DUAL RUN // STORED OR SELECTED FILES / ▶▶ RUN FILES" — **restored** in v38 as the
   "▶ RUN BOTH .TXT" popup (v53:8448-9025). Not lost.
4. **v39 replaced the single "HAND // 0% CLOSED → 100% OPEN" slider** (one value, thumbs held) with
   6 raw channels; v40 added GRIP/ALL OPEN presets. The simpler % control is gone from the UI,
   although r14 still supports `--manual-hand PCT` (r14:2296-2297, 756-801) and Multi Run still
   accepts "50%" hand values (r14:274-316). Relevant to NFR-UX.
5. **v47 dropped the Page-02 MODE combobox** (`PROGRAM RULES`, `PARALLEL`, `LEFT → RIGHT`,
   `RIGHT → LEFT`, `ALT PROGRAMS L→R`, `ALT PROGRAMS R→L`; diff v46→v47 at v46:8982-9007,
   9307-9319, 10504-10624). `_page2_execution_mode` now returns "PROGRAM_RULES" (v53:9426-9428).
   Dead code left in v53 (never called): `_start_page2_serial_first`…`_monitor_page2_serial_second`
   (v53:9430-9618), `_start_page2_program_rules` (10029-10054), `_page2_build_alt_queue`…
   `_monitor_page2_alt_chunk` (10733-10985), `_monitor_page2_dual_sequence` (11094-11144); also unused
   `_page2_command_failed`, `_dual_mousewheel`, `_is_descendant`, `_path_row`. ≈520 dead lines.
   Partial equivalents: PARALLEL ≈ v52/v53 all-TOGETHER timeline; LEFT→RIGHT whole-sequence and
   ALT programs are only expressible as paired RUN FIRST/WAIT OTHER rows with matching IDs; per-arm
   "▶ RUN LEFT/RIGHT SEQUENCE" still exists (v53:6454-6466).
6. v18→v21: the single-arm "Robot arm" side selector was replaced by fixed LEFT/RIGHT panels (no loss).
   v28→v29 removed the label "SIMULATION = mode 0 • REAL = mode 1 • This applies only to this arm
   controller." (explanatory text only).

---

## 3. Runner pairing

| UI | Declared expectation | Auto-selected runner (first existing in UI folder) |
|---|---|---|
| v48 | none (no constants) | `rm_upload_fast_v13_overall_speed.py` first (v48:6854-6867) |
| v51/v52/v53 | `EXPECTED_RUNNER_VERSION = "v14"` (v53:46) | `rm_upload_fast_v14_safe_transit.py` first (v53:6890-6910) |

* `EXPECTED_RUNNER_VERSION` is only a banner fallback when no runner is selected (v53:7391-7404);
  there is no compatibility check (see §1.6 item 2 for the r13 + v51+ transit-100% hazard).
* The banner version is parsed from the filename (`re.search(r"_v(\d+)")`, v53:7397-7402), so the
  `_v53support` copy shows as "v14" too. Only the exact name `rm_upload_fast_v14_safe_transit.py`
  is auto-discovered; the support copies must be browsed manually.
* `hand.py` and `pole.py` must sit beside the runner (r14:365-390 `load_aux_classes`; UI checks
  v53:4148-4155, 4182-4189; also before Multi Run / Load Sequence v53:5065-5080, 5141-5156). They
  exist in UC (UC/hand.py, UC/pole.py).
* **Consistent pair: v53 + r14** (also v51/v52 + r14). v48 pairs with r13 (r14 also works with v48
  JSON: missing `transit_speed` → 30%).

---

## 4. Feature inventory of v53 + r14

### 4.1 Window, pages, tabs
* `DualArmTrajectoryUI(tk.Tk)` "RealMan // Dual-Arm Operations Console", 1880×1000, min 1320×760
  (v53:6876-6882). Top: SOFTWARE VERSION bar + "● DUAL ARM" (v53:7430-7453). Notebook pages
  "PAGE 01 // MANUAL + SETUP" and "PAGE 02 // DUAL SEQUENCE" (v53:7455-7478).
* Page 01 header: "REALMAN // DUAL ROBOTICS CONTROL", hard-coded "LEFT 192.168.1.10 • RIGHT
  192.168.1.103" (v53:7493-7509); dual tools ✋ START BOTH DRAG / ■ STOP BOTH DRAG / ▶ RUN BOTH .TXT
  (v53:7524-7553). Top row 30/70: "SYSTEM // RUNNER + SDK" (Runner path + SDK path + browse,
  v53:7620-7693) and "DUAL RUN // SYNCHRONIZED STORED PROGRAMS" (L/R combos, ↻ SYNC, ▶▶ RUN BOTH,
  ■ STOP BOTH, status, runtime; v53:7695-7834). Below: two `ArmControlPanel`s 50/50 (v53:7585-7607).
* `ArmControlPanel(ttk.Frame)` (v53:1922-6072): always-visible header "LEFT ARM // <ip>", mode badge,
  SIM / REAL / CHECK / ■ SLOW STOP (v53:2049-2113); STATUS + RUN time row (2116-2144); tabs RUN,
  POINTS, DRAG TEACH, HAND / POLE, TELEMETRY (v53:2149-2166).
* Page 02 `_build_sequence_page` (v53:9027-9208): header + ↻ PROGRAMS / ↻ GLOBAL POINTS; "DUAL
  SEQUENCE CONTROL" (▶▶ RUN BOTH // SAFE TIMELINE, ■ STOP BOTH SEQUENCES, ☐ CONFIRM BEFORE WAITING ARM
  (default checked, v53:6989), OVERALL SPEED % (default 100, v53:6977), status, runtime;
  v53:9080-9152); LIVE ARM STATUS (9155-9178); two `WorkflowSequencePanel`s (9180-9208).
* Dialogs: `TrajectoryFileDialog` searchable/sortable .txt browser with history, multi-select,
  Select All, Select Folder Files (v53:74-758); `BatchUploadDialog` per-file Program IDs prefilled
  sequentially, duplicate/1-100 checks (v53:760-968); `MultiRunDialog` "MULTI RUN // SEQUENCE TABLE"
  columns Program ID / Name / Hand / Pole (mm) / Wait (s), quick add by IDs or names, save/load,
  order "POLE → HAND → WAIT → TRAJECTORY" (v53:970-1920, 1126-1131, 1218).

### 4.2 Control groups per arm (Page 01)
| Tab | Group (label) | Controls | Evidence |
|---|---|---|---|
| RUN | TRAJECTORY FILE + UPLOAD SLOT | file + Browse, Upload slot spinbox 1-100 (default 25), "<SIDE> only // <ip>" | v53:2170-2262, 6933-6934 |
| RUN | TRAJECTORY COMMANDS | ⬆ UPLOAD ONLY, ▶ UPLOAD + RUN, ▷ RUN ONLY, ⬆⬆ MULTI UPLOAD, ■ SLOW STOP ("acts immediately — no confirmation") | v53:2265-2333 |
| RUN | CONTROLLER PROGRAMS / SEQUENCES | Program ID or name combo (Enter runs), ▶ RUN STORED, ▶▶ MULTI RUN, 📂▶ LOAD SEQUENCE, ↻ REFRESH LIST, count incl. Drag Teach count | v53:2338-2413, 4741-4769 |
| POINTS | GLOBAL POINTS // MOVEJ + MOVEL + SEQUENCE | search, ↻ REFRESH GLOBAL POINTS, list, Speed % (20), Wait (s), ▶ MOVE J, ▶ MOVE L, ▶▶ RUN SEQUENCE, ADD ↓, table #/Point/Motion/Speed/Wait with ↑ ↓ REMOVE CLEAR | v53:2419-2698, 1997 |
| DRAG TEACH | DRAG & TEACH // RECORD + PLAYBACK | ✋ START DRAG + RECORD (toggles to ■ STOP DRAG), ▶ MOVE TO START + PLAYBACK, Local Drag .txt + Choose, 💾 SAVE LOCAL | v53:2700-2822, 3877-4059 |
| HAND / POLE | MANUAL HAND / POLE CONTROL | 6 sliders+entries 0-1000 (Little, Ring, Middle, Index, Thumb Bend, Thumb Rotate), APPLY HAND // ALL 6 CHANNELS, GRIP PRESET 0 0 0 0 150 1000, ALL OPEN 1000×6, POLE 0 → 315 mm slider + mm entry, APPLY POLE | v53:2825-3130, 2004-2011 |
| TELEMETRY | CONTROLLER TELEMETRY | read-only log "<SIDE> controller // <ip>", auto-follow only when at bottom | v53:3132-3182, 5630-5646 |

### 4.3 Workflows (UI method → runner command; all commands = `sys.executable -u RUNNER --sdk SDK … --side left|right --yes`, `_base_command` v53:4255-4262)
| Workflow | UI entry | Runner args | Confirmation | Evidence |
|---|---|---|---|---|
| Upload only | `_upload_only` | `TRACK SLOT` | yes ("will be replaced") | v53:4265-4291 |
| Upload + verify + run | `_upload_and_run` | `TRACK SLOT --run` | yes (warning) | v53:4294-4327 |
| Run upload slot | `_run_upload_slot_only` | `--run SLOT` | yes | v53:4694-4736 |
| Run by ID or name | `_run_existing` → `_resolve_stored_program` | `--run SLOT` | yes | v53:5442-5484, 4776-4842 |
| List programs | `_list_slots` | `--list` (parsed by regex `^slot\s+(\d+)\s+[0-9.]+\s+KB\s+speed\s+\S+\s*(.*)$`) | no | v53:5486-5508, 5756-5767 |
| Batch upload | `_multi_upload_setup` → `_batch_upload_worker` | per file: `--list`, then `TRACK SLOT`; final `--list` | per file Yes/No/Cancel (replace or empty) | v53:4330-4692, 4453-4501 |
| Multi run | `_multi_run_setup` → `_multi_run_worker` | `--sequence-file tmp.json` (one connection; Pole→Hand→Wait→Trajectory) | one confirmation | v53:5111-5440 |
| Load sequence & run | `_load_sequence_and_run` | `--sequence-file` | yes | v53:5019-5109 |
| Synchronized dual run | `_run_both_programs` | `--run SLOT` ×2 via `threading.Barrier(2)` before Popen | yes ("BOTH ROBOT ARMS WILL MOVE AT THE SAME TIME") | v53:11246-11380, 5654-5710 |
| Run both .txt | `_show_dual_file_run_popup` → `_run_both_txt_files` | concurrent uploads (default IDs 99/99), then barrier `--run` ×2 | yes | v53:8472-8952, 6966-6967 |
| Drag & teach per arm | `_drag_teach_toggle` | `--drag-session` (SIGINT stops → `rm_stop_drag_teach`) | yes (start) | v53:3877-3964; r14:912-955 |
| Drag save | `_drag_save` | `--drag-save FILE` (`rm_save_trajectory`) default `~/Downloads/RealMan_DragTeach/<side>_drag_<stamp>.txt` | no | v53:3983-4017, 3840-3850; r14:957-989 |
| Drag playback | `_drag_playback` | `--drag-playback` (`rm_drag_trajectory_origin(1)`, `rm_run_drag_trajectory(1)`) | yes | v53:4019-4059; r14:1173-1210 |
| Both-arm drag | `_start_both_drag` / `_stop_both_drag` + save popup "💾 SAVE BOTH .TXT" | `--drag-session` ×2 (barrier), `--drag-save` ×2 | yes (start), none (stop) | v53:7841-8446 |
| Global point move | `_run_selected_global_point` | `--global-point NAME --global-move movej|movel --global-speed N --global-wait S` | yes | v53:3319-3383 |
| Global sequence | `_run_global_point_sequence` | `--global-sequence-file` (`realman_global_point_sequence` v1) | yes | v53:3670-3783 |
| Hand 6-channel / presets | `_manual_hand_apply` / `_manual_hand_apply_preset` | `--manual-hand-raw L R M I TB TR` | **no** ("APPLY itself is the explicit motion command") | v53:4127-4174 |
| Pole | `_manual_pole_apply` | `--manual-pole MM` (0-315) | **no** | v53:4176-4209 |
| SIM / REAL / CHECK per arm | `_set_arm_mode_ui` / `_check_arm_mode_ui` | `--set-arm-mode simulation|real` (0/1, read-back verified) / `--get-arm-mode` | REAL only | v53:3785-3815; r14:1213-1258 |
| Page-02 single arm | `_run_single_page2_sequence` | `--workflow-sequence-file tmp --workflow-overall-speed N` | yes | v53:9302-9400 |
| Page-02 RUN BOTH (SAFE TIMELINE) | `_run_both_page2_sequences` → `_start_next_page2_rule_stage` | one runner per arm per stage; stage docs `/tmp/realman_dual_sequence_ui/<side>_rule_<NN>_<pid|tail>.json` | yes + optional handoff + lid checkpoint | v53:10986-11092, 10055-10139 |
| Handoff confirmation | `_monitor_page2_rule_serial_first` | – | askyesno if CONFIRM BEFORE WAITING ARM | v53:10376-10453 |
| Stop controls | `_slow_stop` (SIGINT to process group), `_stop_both`, `_stop_both_drag`, `_stop_both_page2_sequences`, `_stop_single_page2_sequence` | runner KeyboardInterrupt → `rm_set_arm_slow_stop`, `rm_stop_drag_teach`, pole `rm_set_lift_speed(0)`, hand freeze | none by design | v53:6028-6071, 11437-11449, 8023-8040, 11171-11186, 9417-9424 |

Page-02 step types (`WorkflowSequencePanel`, v53:6074-6874; runner validation r14:1668-1748):
`program {id 1-100, name, speed(legacy 100), transit_speed 1-100 (default 30), dual_rule}`,
`pole {mm 0-315}`, `hand {values[6] 0-1000}`, `wait {seconds 0-3600}`,
`global {name, move movej|movel, speed 1-100 (20), wait_s}`. Builder tools ↑ UP, ↓ DOWN, DUPLICATE,
REMOVE, CLEAR (confirm), LOAD, SAVE, ▶ RUN <SIDE> SEQUENCE, ■ STOP <SIDE> (v53:6419-6466).
Chunking: a chunk ends at each Program, so Pole/Hand/Wait/Global before a Program travel with it;
trailing steps form a "FINAL AUX STEPS" tail (v53:9619-9664); tail stage runs both arms with a
barrier (v53:10570-10713).

Dual rules (v53): TOGETHER rows = synchronized timeline blocks (IDs may differ); RUN FIRST + WAIT
OTHER (same ID on both arms) = serial handoff; first arm must succeed (`last_command_rc == 0`,
v53:10715-10717) or the waiting arm is "NOT STARTED" (v53:10398-10410); positive WAIT on both arms
= manual checkpoint (§1.4).

Speeds actually in force (UI never passes `--speed`, `--run-speed`, `--restore-speed`, `--timeout`,
`--ip`, `--port`; grep of v53 flags): upload plan_speed 0 (r14:2287-2289); Page-01 runs/Multi Run
transit 20% (`--run-speed` default, r14:2337-2339, 223, 629); Page-02 Start % 30 default; Page-02
overall `rm_set_plan_speed` 100% default (r14:2115-2133 at workflow start, r14:1855-1863 before each
Program); after every run/sequence
the controller global speed is restored to 50% (r14:260, 748, 2263-2277); per-run timeout 300 s
(r14:2342).

### 4.4 JSON / file formats
| Format | Shape | Written by | Read by |
|---|---|---|---|
| Trajectory .txt | NDJSON `{"point":[j1..j7]}` int millideg, ≥2 lines | drag save (`rm_save_trajectory`), optimizer | r14 `read_track` (r14:76-88) |
| Controller project | `{"file":7}` CRLF, `{"name":"Folder","num":1,"type":<count>,"enabled":true,"parent_number":0}`, points, CRLF | r14 `write_project` (r14:91-102) | controller `rm_send_project` |
| `realman_workflow_sequence_v1` | `{format, side, steps[]}` (+ optional `workflow_note`, step `note`, ignored) | `sequence_document`/`write_temp`/`_save` (v53:6774-6817), stage chunks (v53:10055-10096) | `_load` (v53:6819-6861; only format + list checked), r14 `load_workflow_sequence_file` (r14:1751-1763) |
| `realman_multi_run_sequence` v3 | `{format, version:3, programs:[{id,name,hand,pole_mm,wait_s}]}`; hand = "v1,…,v6" or "N%" | `MultiRunDialog._sequence_data` (v53:1684-1701), `_multi_run_worker` (v53:5237-5242) | r14 `load_sequence_file` (r14:353-362) |
| `realman_global_point_sequence` v1 | `{format, version:1, steps:[{name,move,speed,wait_s}]}` | v53:3739-3757 (temp, deleted after run v53:6010-6016) | r14 `load_global_sequence_file` (r14:1561-1613) |
| "timeline" format | **none** — v52/v53 derive the timeline at run time from the two v1 docs | – | – |
| `manual_checkpoint` | mentioned in V6/README.txt:64-67 as a top-level field, but **absent** from every V6 JSON and never read by v53/r14 (grep) — v53 infers the checkpoint from a positive WAIT instead; FINAL JSONs carry `workflow_note` + `note: "MANUAL LID COVER OPEN"` (UC/LEFT_V6_70SERIES_FINAL.json) | – | – |
| `DUAL_FLOW_TIMELINE_V6.json` | referenced at V6/README.txt:78, **not present** anywhere in the workspace (`find -iname '*TIMELINE*'`) | – | – |

Folders: `~/Downloads/RealMan_DragTeach/`, `~/Downloads/RealMan_Sequences/realman_sequence.json`,
`/tmp/realman_dual_sequence_ui/` (v53:3843, 4845, 997, 6785, 10068).

### 4.5 Hard-coded addresses, paths, defaults
* UI: IPs are **display-only** strings: header (v53:7506), panel constructor `ip_address=`
  (v53:7588, 7600), dual-run confirmation (v53:11314-11315), close warning (v53:11513-11516).
  The panel never passes the IP to the runner; every command uses `--side` only.
* **No IP entry anywhere in the UI.** The runner supports `--ip`, `--port` (default 8080) and env
  `RM_LEFT_IP`/`RM_RIGHT_IP` (r14:46-49, 2285-2286, 2347); an env override would silently change the
  real target while the UI labels still show .10/.103.
* SDK default `/home/alai-labs/alix_ws/src/alix/external/butterfli-realman-alix/Python` via
  `ALIX_RM_SDK` (v53:48-51) — another machine's home.
* Slot conventions conflict: UI default 25 (v53:6933), RUN BOTH .TXT default 99 (v53:6966),
  README_COMMANDS says taught tracks 40–59 (UC/README_COMMANDS.md:58), monorepo policy reserves
  70–100 for cleaning tracks (ALIX/core/python/alix_dispatch/src/alix_dispatch/track.py:13-16), V6
  uses 71–83 (V6/README.txt:101-104).

---

## 5. Architecture and coupling points for a web re-implementation

* Desktop Tkinter app (ttk "clam"), no server: every action spawns one runner subprocess per arm.
  Classes: `TrajectoryFileDialog`, `BatchUploadDialog`, `MultiRunDialog` (tk.Toplevel),
  `ArmControlPanel`, `WorkflowSequencePanel` (ttk.Frame), `DualArmTrajectoryUI` (tk.Tk); entry
  `if __name__ == "__main__": DualArmTrajectoryUI().mainloop()` (v53:11579-11581).
* Spawning: `subprocess.Popen(cmd, stdout=PIPE, stderr=STDOUT, text=True, bufsize=1,
  start_new_session=True)` (Windows: `CREATE_NEW_PROCESS_GROUP`) in `_process_worker`
  (v53:5733-5751), `_run_streamed_subprocess` (4395-4421), `_multi_run_worker` (5281).
* Threading: one `self.worker` thread per arm; all Tk updates marshalled with `self.after(0, …)`;
  dual launches use `threading.Barrier(2)` immediately before Popen (v53:5654-5710; barriers at
  7934, 8926, 10209, 10586, 11343); orchestration is Tk `after()` polling every 100–200 ms of
  `has_active_command()` (= `multi_run_active or batch_active or process.poll() is None`,
  v53:3184-3187) and `last_command_rc` (set from `proc.wait()` before any Tk callback,
  v53:5969-5972; -999 on UI exception v53:6005).
* Output parsing (`_process_worker` v53:5753-5961): prefix markers `GLOBAL POINT:` (JSON),
  `GLOBAL POINT LIST COMPLETE:`, `GLOBAL SEQUENCE START/STEP/COMPLETE:`, `GLOBAL MOVE START/COMPLETE:`,
  `GLOBAL WAIT:`, `** GLOBAL`, `ARM RUN MODE:`, `ARM MODE SET COMPLETE:`, `** ARM MODE`,
  `DRAG TEACH ACTIVE:/STOPPED:` (STOPPED sets `drag_teach_has_recording`), `DRAG SAVE COMPLETE:`,
  `DRAG ORIGIN COMPLETE:`, `DRAG PLAYBACK START/COMPLETE:`, `** DRAG`, `MANUAL HAND/POLE COMPLETE:`,
  `** MANUAL HAND/POLE FAILED`, substrings `VERIFIED:`, `running (`, `RUN COMPLETE: X s`,
  `UPLOAD FAILED`/`NOT VERIFIED`, `RUN NOT COMPLETE`/`NOT STARTED`; Multi Run regexes
  `SEQUENCE PROGRAM i/n: slot S (name)`, `SEQUENCE PROGRAM COMPLETE i/n: X s`, `SEQUENCE SETTLE`,
  `SEQUENCE COMPLETE: i/n programs, X s`, `warning-only arm code` (v53:5285-5370). Page-02
  `WORKFLOW …` lines are **not parsed** (grep) — Page-02 progress is per stage only.
* Exit codes: 0 ok, 1 refused/failed, 2 argparse error, 130 hand-raw interrupt (r14:855).
* Status-text coupling still used (fragile): Page-01 dual run, RUN BOTH .TXT and dual drag save decide
  success by `status_var.startswith("COMPLETE //" | "VERIFIED //" | "DRAG SAVE COMPLETE")`
  (v53:11417-11420, 8873-8880, 8419-8424); Page 02 uses rc (v48 fix).
* Runner internals to replicate or wrap: one TCP connection per invocation
  (`rm_create_robot_arm(ip, port, 3)`, `rm_set_timeout(1500)`, r14:2607-2614; closed with
  `rm_delete_robot_arm()` r14:2698); program list paged 10
  at a time (`PAGE = 10`, r14:102-119, reason: SDK buffer truncation); upload verify by list size
  ±max(64 B, 0.1%) (r14:147-175); run refuses on blocking error codes / busy controller
  (r14:189-205); error polling every 0.5 s, `KNOWN_ERR = {4105 warning, 4109 ARM COLLISION}`
  (r14:56-57, 238-256); workflow fast switch (list read once, `workflow_quick_settle` 0.6/1.0 s,
  one global-speed restore) (r14:1766-1985, 1986-2277).
* Coupling points a web/backend port must keep: (1) runner CLI contract or a Python service with the
  same verbs; (2) a structured event stream instead of the marker strings; (3) SIGINT-equivalent stop
  that reaches the SDK slow-stop inside the same connection; (4) barrier-synchronised dual launch
  (software sync only — each runner then connects, reads the program list and state before
  `rm_set_program_id_run`, so the real start skew is unmeasured, inferred); (5) rc-based completion;
  (6) the pure planner functions `_page2_program_chunks` + `_page2_build_program_rule_plan`
  (portable as-is); (7) per-arm mutual exclusion and global "active" flags (`dual_run_active`,
  `dual_drag_active`, `dual_file_run_active`, `page2_dual_active`, v53:6936-6996);
  (8) hand.py/pole.py Modbus/lift classes (UC/hand.py:1-126, UC/pole.py:1-58).
* Race (inferred, unverified on hardware): `_run_both_programs` and `_start_uploaded_txt_together`
  call their monitors immediately (v53:11380, 8952); the monitors use `has_active_command()`, which is
  False until Popen assigns `process`. The same race was explicitly found and fixed for dual drag
  (comment v53:7972-7978). A port should track an explicit "launching" state.
* Safety gaps: in parallel stages a failure on one arm does **not** stop the other arm — monitors
  wait for both (v53:10245-10321, 11382-11435); no automatic mode/connection check at start (mode
  badge "MODE // UNKNOWN" until CHECK, v53:1989, 2096-2103; CHECK/list/global refresh are only
  triggered by buttons, grep of call sites v53:2099, 9232-9233, 9265-9266, 11219-11220); drag
  playback uses blocking SDK calls
  (`rm_run_drag_trajectory(1)`), so SIGINT may only act after the call returns (inferred, r14:1182-1209).

---

## 6. Usability for non-technical users (NFR-UX)

* Density: ≈87 `ttk.Button`s, 14 LabelFrames, 17 entries, 7 comboboxes, 4 Treeviews, 3 Notebooks
  (grep counts in v53); Page 01 shows two full arm consoles side by side, each with 5 tabs; min window
  1320×760.
* Jargon on screen: Runner, SDK, Upload slot, Program ID, VERIFIED, MoveJ/MoveL, Global Points,
  Start %, OVERALL SPEED %, TOGETHER / RUN FIRST / WAIT OTHER, SAFE TIMELINE, TIMELINE-n, LID-PREP,
  rc=…, telemetry, SIM/REAL/CHECK, 0→1000 channel counts, "Stored timing". Help texts reference
  internals ("Start % = speed used only to reach this Program's first pose", v53:6219-6224). The v53
  module docstring is stale (identical to v48's list, v53:1-29).
* Confirmations: 24 `askyesno` + 1 `askokcancel` (grep); present for upload/replace, all runs, REAL
  mode, drag start/playback, global moves, dual runs, Page-02 runs, handoff, lid checkpoint. **Absent**
  for manual HAND/POLE apply (v53:4166-4174, 4203-4209) — conflicts with the usage rule "the pole
  moves only with the arm still; never move pole or hand with the pad on a surface" (UC/usage.py:24-25)
  that the UI does not enforce. SIM switch has no confirmation (v53:3802-3815).
* Stop affordances: always-visible per-arm ■ SLOW STOP (enabled only for motion commands,
  v53:5621-5627), STOP BOTH, STOP BOTH DRAG, STOP BOTH SEQUENCES, STOP LEFT/RIGHT; one click, no
  confirmation (v53:6046, 8025, 11172, 11438); on signal failure the UI tells the user to use the
  hardware E-stop (v53:6062-6071).
* Defects visible to operators: 36 strings use `\\n` and render a literal "\n" — including the batch
  "Replace Program ID?" dialog (v53:4467-4485), the folder-select dialog (608-633) and the Program
  Handoff dialog (v53:10436-10438; same since v46).
* Page-01 hint above the arm panels talks about Page-02 rules (v53:7556-7564) — confusing.
* Theme (NFR-THEME): dark neon palette `BG #06111F`, `PANEL #0B1B2B`, `CYAN #00E5FF`, `GREEN
  #00D084`, `RED #FF3B5C`, DejaVu Sans (v53:56-71, 7002-7310). ALIX Studio uses a light theme
  (`--brand #2F96EE`, `--bg #FFFFFF`, `--ok #2E9E5B`, `--danger #D93A34`;
  ALIX/core/python/alix_taskgen/src/alix_taskgen/web_editor/editor.css:22-44). Do not port the Tk
  palette; keep only the colour semantics (run=ok, upload=brand, stop=danger, warning=amber).

---

## 7. Verdict detail

* v48 is not the most comprehensive. **v53 UI + r14 runner is**, and it is the only version that can
  run the current V6 70-series dual package safely (lid checkpoint).
* Nothing from v48 is missing in v53; four semantic changes must be carried consciously (§1.6).
* From earlier versions, missing in both v48 and v53: drag→controller DT_ program + run drag .txt
  (v26–v32), dual-run REAL-mode warning (v32), single % hand slider (v18–v38), Page-02 MODE selector
  incl. LEFT→RIGHT whole-sequence and ALT programs (v44–v46).
* Recommended base for INT-CTRL: port the r14 runner verbs (or wrap r14 unchanged as the "server
  program") and the v53 planner; re-implement the UI in the Lite web front end.

---

## Requirement relevance

* **INT-CTRL** — source of truth = v53 + r14 (§0, §3). Reusable now: runner CLI (r14:2280-2702),
  planner (v53:9619-9997), stop semantics, JSON formats (§4.4). Must add: version handshake,
  structured events, IP passing, explicit launch state.
* **R4 Real-arm testing / IP / real-sim toggle** — per-arm SIM/REAL/CHECK uses the controller's own
  run mode (`rm_set_arm_run_mode` 0/1 with read-back, r14:1213-1258; UI v53:2079-2113, 3785-3815).
  No IP entry (IPs display-only, §4.5) but r14 accepts `--ip/--port` and `RM_LEFT_IP/RM_RIGHT_IP`
  — the same env names the monorepo emulator uses (ALIX/docs/EMULATOR.md:169-178), and
  `ALIX/core/python/alix_dispatch/src/alix_dispatch/emulated_program.py:1-25` fakes the
  `rm_send_project`/list/run verbs, which makes a hardware-free "sim" backend for the same runner
  flows possible (inferred). Restore the dropped v32 REAL-mode warning. "Return to editing" needs
  run results mapped back to the trajectory (not present).
* **R5 Collision highlight (real)** — the only collision signal is controller error 4109 "ARM
  COLLISION" polled every 0.5 s and printed as `!! the arm reports … at X s` (r14:56-57, 246-251,
  1925-1934); the UI only shows it in telemetry/fault text. Mapping X s → point index ≈ X / 5.5545 ms
  minus the transit phase (ALIX/core/python/alix_dispatch/src/alix_dispatch/track.py:4-11;
  exp01 upload_track.py:33-35 says transit takes 3.6–4.0 s) is possible (inferred). exp01 also notes
  runs that stop early with NO error code (ALIX/experiments/exp01_teach_trajectory_reverse_engineering/src/live/upload_track.py:38-39).
* **R7 Pole height / hand position before teaching** — hardware path exists: `--manual-pole MM`
  (0–315, r14:865-909, pole.py lift slew 0.059/0.097 m/s UC/pole.py:3-8), `--manual-hand-raw` 6×0–1000
  (r14:804-862), presets GRIP 0,0,0,0,150,1000 / ALL OPEN (v53:2980-3000), Multi Run order
  Pole→Hand→Wait→Trajectory (v53:1218; r14:393-406). Per-region values already exist in the V6
  workflows (e.g. LEFT pole 150 before P71, 50 before P73; RIGHT 50 before P71, 150 before P74;
  hand GRIP at start — UC/LEFT_V6_70SERIES_FINAL.json, UC/RIGHT_V6_70SERIES_FINAL.json). Missing: a
  per-region preset object and an "apply preset → start drag" flow (pole/hand and drag are on
  different tabs).
* **R13 Faster upload** — r14 speeds up *switching*, not *upload*: persistent connection for
  sequences/workflows, single list read, 0.6/1.0 s settle, overall speed (r14:1766-2277). Upload is
  one file per process; batch upload spawns 2 processes and ≥3 paged list reads per file
  (v53:4504-4692; r14:105-119, 147-175). RUN BOTH .TXT uploads both arms concurrently (v53:8818-8830).
  Monorepo facts: transport is 2 KB acked chunks over TCP 8080 and the player clock is fixed at
  5.5545 ms/pt (ALIX/docs/sources.md:4885, 4892); speed params touch only the transit and persist
  across power cycles (track.py:21-24). Gains available: one connection for a batch, skip pre-list
  when the slot map is cached, parallel L/R uploads, `only_save` + verify, no re-upload when the
  slot already holds identical bytes (inferred).
* **R1 Region-wise collection** — drag record/save per arm and both arms exists (§4.3) but files are
  timestamp-named with no region, timer, auto-advance or dashboard (v53:3840-3850, 8043-8056).
* **R8 TXT merging** — not in the control UI; V6 merge was done offline (V6/README.txt:33-56, bridges
  "NOT collision-checked").
* **R10 Dual-arm speed/timing** — V6 timings assume 5 ms/pt (V6/README.txt:6; V6/FINAL_FILE_RUNTIME_REPORT.csv
  `runtime_s_at_5ms`) vs the measured 5.5545 ms/pt player clock (ALIX/docs/sources.md:4885) → real
  runtimes ≈11% longer and pole-bridge timing (1.380 s) is off (inferred). v52/v53 timeline blocks
  reduce stage boundaries (fewer process launches).
* **NFR-UX / NFR-THEME** — see §6.
* **X1/X2** — not applicable to the control UI.

---

## Reuse verdicts

| Feature | Verdict | What to lift | Evidence |
|---|---|---|---|
| Runner r14 (upload/verify/run/list/sequence/workflow/drag/hand/pole/mode/global) | reuse-with-adaptation | Call as a subprocess from the Lite backend first; add `--ip`, JSON event output, version handshake; later fold into a service | r14:2280-2702 |
| Upload + verify logic | reuse-as-is | `read_track`, `write_project` (CRLF), size-verify | r14:76-175 |
| Batch upload | reuse-with-adaptation | per-file ID mapping + replace prompt; make single-connection | v53:760-968, 4504-4692 |
| Synchronized dual run (Barrier) | reuse-with-adaptation | barrier launch + explicit launching state + cross-arm abort | v53:5654-5710, 11246-11435 |
| Page-02 sequence model + JSON v1 | reuse-as-is | step schema, chunking, save/load | v53:6074-6874, r14:1668-1763 |
| SAFE TIMELINE planner | reuse-with-adaptation | pure functions; make the checkpoint an explicit step type instead of "positive WAIT", parametrise the lid text | v53:9619-9997, 10141-10188 |
| Handoff with optional confirmation | reuse-as-is | rc-based gating, CONFIRM BEFORE WAITING ARM | v53:10376-10453 |
| Live arm status strings | reuse-as-is | state vocabulary | v53:10019-10027, OPS:153-167 |
| Version banner | reuse-with-adaptation | show UI + runner + Python; enforce, do not just display | v53:7391-7453 |
| SIM/REAL/CHECK | reuse-with-adaptation | controller run mode verbs + REAL confirm + restore v32 warning | v53:3785-3815, r14:1213-1258 |
| IP handling | not-available (UI) / reuse (runner) | add IP fields, pass `--ip`; align with `RM_LEFT_IP/RM_RIGHT_IP` | v53:7588-7600, r14:46-49 |
| Drag & teach (per arm, both, save, playback) | reuse-with-adaptation | runner verbs; add region naming, timer, auto-advance | v53:3877-4059, 7841-8446 |
| Drag → controller DT_ program | reference-only (UI dropped) | runner `--drag-store/--drag-run-file` if wanted | r14:992-1172 |
| Hand 6-channel + presets | reuse-with-adaptation | presets as named per-region settings; keep simple % option | v53:2825-3130, r14:804-862 |
| Pole control | reuse-with-adaptation | 0–315 mm with arm-still interlock | v53:3021-3110, UC/pole.py |
| Multi Run (Pole→Hand→Wait→Program) | reference-only | pattern for R7 preset-then-run | v53:970-1920 |
| Global points (MoveJ/MoveL) | reference-only | not needed for joint-space Lite flows | v53:2419-2698 |
| Stop controls | reuse-as-is (semantics) | one-click slow stop, stop both | v53:6028-6071 |
| Output marker parsing | reference-only | replace with structured events | v53:5753-5961 |
| Tk theme / layout | not-applicable | use ALIX Studio tokens | v53:56-71 vs editor.css:22-44 |
| Page-02 MODE selector (v44–v46 dead code) | reference-only | only if LEFT→RIGHT whole-sequence or ALT is re-required | v53:9430-9618, 10733-10985 |

---

## Open questions for the planner / user

1. Is the v53 "positive WAIT on both arms = manual lid checkpoint" rule the intended long-term
   contract, or should Lite introduce an explicit `checkpoint` step type (and keep WAIT as a plain
   pause)? The V6 README's `manual_checkpoint` field (V6/README.txt:64-67) was never implemented.
2. Should TOGETHER keep v52/v53 block semantics (sync only at block edges) or restore v48 per-Program
   re-sync? Both exist in code history; the V6 package needs block semantics.
3. Which slot range does Lite own (UI 25 / 99 defaults, README 40–59, monorepo policy 70–100, V6 71–83)?
4. Does Lite keep pole/hand on the upload-channel runner (as v53/r14 do via Modbus/lift calls) or
   follow the monorepo policy that the track channel carries cleaning stages only and hand/pole stay
   on ROS (ALIX/core/python/alix_dispatch/src/alix_dispatch/track.py:13-16)?
5. Are the dropped v44–v46 modes (LEFT→RIGHT whole sequence, ALT programs) or the v26 DT_ "save to
   controller" still wanted by operators?
6. Should the runner be wrapped unchanged (fastest integration, subprocess + stdout markers) or ported
   into the Lite backend as a library with structured events (needed for R5 graph sync)?
7. Real start skew of the barrier-synchronised dual launch has never been measured (no timing log of
   both `rm_set_program_id_run` calls); is it acceptable for TOGETHER blocks?
