# C2 — Trajectory optimisation (R2, R6, R11, R12, INT-TRAJ): verified final

ALIX Studio Lite requirements, cluster C2. Final version after adversarial verification, 2026-10-13. Written for the engineers who will build Lite and for the requester.

**How this was checked.** Every borrow verdict and every number in the analyst's draft (`reqs/C2_trajectory_optimization_analysis.md`) was traced to the cited code or document. The key measurements were then re-run independently, read-only:
- Butterworth/`filtfilt` and URDF forward kinematics were re-implemented in numpy, because scipy cannot be imported under `python3 -I` here (system scipy 1.11.4 against numpy 2.4.6).
- Most of the analyst's figures reproduced exactly.
- Some claims did not survive, and several safety-relevant items were missing. They are corrected below and listed in the change log at the end.
- No repository or workspace file was modified.

## Path aliases

| alias | absolute path |
|---|---|
| `REQ` | "/home/newtonjeri/alix_studio_lite_ws/alix_studio_lite/docs/reference/ALIX_Studio_Lite_requirement_inputs.txt" |
| `H` | /home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html (prototype, 2096 lines) |
| `JS` / `CSS` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/web_editor/editor.js / editor.css |
| `SRV` | …/alix_taskgen/src/alix_taskgen/server.py |
| `BC` | …/alix_taskgen/src/alix_taskgen/body_collision.py |
| `TIG` | …/alix_taskgen/test/test_studio_interaction_guards.py |
| `TT` / `CT` / `DAV` | /home/newtonjeri/alix_ws/src/alix/tools/taught_tracks.py / cleaning_tracks.py / dual_arm_verdicts.py |
| `PTR` / `K` | /home/newtonjeri/alix_ws/src/alix/experiments/exp01_teach_trajectory_reverse_engineering/src/predict_track_replay.py / src/teachlab/kinematics.py |
| `TRK` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_dispatch/src/alix_dispatch/track.py |
| `UK` / `DAPC` / `PA` | …/alix_workspace/src/alix_workspace/urdf_kinematics.py / dual_arm_plan_collision.py / plan_analysis.py |
| `EM` | /home/newtonjeri/alix_ws/src/alix/Resource/tracks/taught/EXECUTION_MODEL.json (ALIX execution model v4, fitted 2026-09-16) |
| `FIND` | /home/newtonjeri/alix_ws/src/alix/plan/FINDINGS.md |
| `TCTR` | /home/newtonjeri/alix_ws/src/alix/docs/motion/TRACK_CHANNEL_TIMING_REQUIREMENT.md |
| `GIC` | /home/newtonjeri/alix_ws/src/alix/Resource/reports/2026-09-20_geometric_intervention/RESULT_FROM_CORPUS.md |
| `CC` | …/exp01…/data/teach_files/commode_c_taught_paths/ (raw left-arm commode recordings + description.txt) |
| `RUN14` / `UI53` | /home/newtonjeri/alix_studio_lite_ws/upload_channel/rm_upload_fast_v14_safe_transit.py / rm_trajectory_ui_v53_safe_lid_checkpoint.py |
| `V6` / `PLAN` | /home/newtonjeri/alix_studio_lite_ws/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/ / …/upload_channel/plan_txt_files/ |
| `M01`…`M13` | evidence maps in …/scratchpad/maps/ |

**Gate names used throughout.** These are ALIX's own names (`TT:420-604`; map 06 §2.5):

| gate | what it checks |
|---|---|
| **G1** | Predicted executed joint margin at every joint extreme. |
| **G2** | Band-limited ticks over a joint rating. 20 Hz zero-phase filter, judged at the 5.0 ms tick. |
| **G3** | Step ≤ 10° between lines. |
| **G4** | The commanded pad path against the raw recording at the contact frame: ≤ 10 mm, 5 mm preferred. |
| **G5** | Predicted *executed* pad departure within an executed budget. ALIX default 20 mm. |

## What the upload-channel programs do (high level only, as asked)

- **`RUN14` (runner v14)** is a standalone command-line tool for one arm per call (`RUN14:1-35`). It covers three jobs:
  - **Upload:** it reads a `.txt` track, builds the controller's project file with CRLF line endings, writes it to a program slot, and checks the stored size.
  - **Run:** it runs stored programs or sequences, slow-stopping on Ctrl+C.
  - **Drag-teach and devices:** it records, saves and plays back drag-teach motions, and moves the pole and hand.
- **`UI53` (UI v53)** is the Tkinter operator window. It wraps the runner, starting one subprocess per action (`UI53:1-31`). It adds:
  - batch upload, and multi-run tables with hand, pole and wait steps;
  - synchronised dual-arm runs;
  - drag-teach save and playback;
  - pole and hand sliders.
- **Where they touch C2:**
  - **Input.** Raw recordings come from the controller's drag-teach save (`RUN14:957-989`). The UI names them by side and time only: `~/Downloads/RealMan_DragTeach/<side>_drag_<stamp>.txt` (`UI53:3840-3850`). Region identity must therefore be added by R1.
  - **Output.** C2 must hand on files that the runner's `read_track` accepts: 7 integer millidegrees per line, at least 2 lines, and one bad line refuses the whole file (`RUN14:76-89`).
  - No control-app issue blocks C2. The known v53 issues belong to INT-CTRL.

## Summary

| Req | What it asks | ALIX Studio verdict | Main other sources | Priority | Effort |
|---|---|---|---|---|---|
| **R2** | Trim / smooth / scale per region and sub-region, undo ×5 | Partial: undo and stale-on-edit patterns, feedback components; build the editor and processor | Prototype (sub-region UX); ALIX `TT`/`CT`: `strip_pauses`, `smooth_taught`, `capped_millideg`, `departure_lambda_cap`, `model_caps`, `rate_fit_track`, `insert_dwells` | Must | L (≈14–15 d) |
| **R6** | Resizable, adjustable plots and editing windows | Partial: dock separator, dialog grip, maximise-with-restore, vendored Plotly | Prototype Plotly config | Should | M (≈5–6 d) |
| **R11** | Smoothness control with immediate simulation check | Partial: 3D viewer, DAC scrub/highlight, stale-on-edit; `BodyCollisionWorld` as a library with adaptation | ALIX `smooth_taught`, exact `deviation_mm`, FK, ALIX dual-arm separation gate | Should | L (≈8–10 d; M if C3's collision worker provides the windowed check) |
| **R12** | Find high/low spikes and suggest fixes | Not available — build; presentation parts only | ALIX gate instruments (G1–G5), `rate_fit_track`, `slowness_census`, `_departure_attribution`, acceleration advisory | Should (hard gates Must via INT-TRAJ) | L (≈10–12 d) |
| **INT-TRAJ** | Integrate the prototype as Lite's tab, in the ALIX theme | Partial: tokens, components, vendored Plotly | Prototype (stepper, compare, report); `TRK` player law; runner file contract | Must | L (≈12–15 d) |

## Headline facts (verified)

1. **The prototype's default smoothing moves the cleaning pad 9–10 mm.**
   - Verified by FK at `L_glove_frame_2`. FILE_08 moves 9.05 mm. FILE_10.1 moves 10.39 mm, or 10.60 mm measured from the taught path to the new one.
   - ALIX's 5 Hz zero-phase filter moves the same files 1.53 and 0.91 mm (`TT:894-912`).
   - The prototype figure already exceeds or uses up ALIX's 10 mm G4 limit (`TT:129-132`). It also exceeds FILE_10.1's 5 mm budget (`CC/description.txt:13`) before any execution error is added. That 5 mm is an executed-path budget (`CC/description.txt:19`).
2. **The prototype's smoothing creates a step at the ends of the file.**
   - The first step is 966 millidegrees on J3 for FILE_08 and 1005 on J6 for FILE_10.1. The last step is 546 on J2 for FILE_01.1 (`H:1513-1514`, `H:1721-1723`).
   - Each is larger than any interior step of that file: 429, 392 and 311 respectively. This is not universal: on FILE_6 the end step is 254 against an interior maximum of 636.
   - A smooth_taught-style filter returned the exact first and last points after integer rounding on all five commode files, with first and last steps of 0–1 millidegree.
3. **Sub-region boundaries cause speed jumps.**
   - Applying the prototype's default suggestions `[75, 80, 95, 115, 75]` to FILE_01.1 gives one-tick speed changes of 640, 440, 880 and 840 °/s² at the four boundaries. The median elsewhere is 80 °/s² (`H:1568-1616`). Verified.
4. **Corrected: most shipped files fail only on single-tick spikes, not on ALIX's enforced speed gate.** Measured at the 5 ms tick:

   | measure | files over | notes |
   |---|---|---|
   | raw per-tick rate | 14 of 19 | the analyst's figure is right for this measure |
   | ALIX's enforced gate G2 (20 Hz band-limited, `TT:594-599`, `PTR:92-114`) | 6 of 19 | see breakdown below |

   The six files that fail G2:

   | file | band-limited ticks over a rating | detail |
   |---|---|---|
   | V6 `R_P06P07` | 102 | J6 peaks at 312 °/s (1.39×); longest over-run 285 ms |
   | V6 `R_P10P12` | 29 | J4 243 °/s, J7 229 °/s |
   | four lid_top files | 200 / 175 / 117 / 70 | — |

   - The other 8 V6 files exceed a rating only on single raw ticks. Their band-limited peak is at most 0.91 of the rating.
   - Single raw ticks are not harmless. The controller latches warning 4105 on them, and one ALIX program ended at 58 % with the flag up (`CT:211-218`).
5. **ALIX's speed policy is broader than "stay under the ratings".**
   - The joint ratings are hard limits. But "the only check is the deviation the hardware will produce … at a 20 mm executed budget" (`TCTR:3-9`).
   - ALIX's default re-timing applies three ceilings: the ratings, a per-line path-accuracy ceiling, and joint-limit arrival caps (`TT:1377-1437`, `TT:1817-1852`, `TT:1855-1985`, default `--timing model` at `TT:4371`).
   - Hardware runs show why the ratings alone are not enough (`TCTR:210-214`):

     | run | executed path error (p95) |
     |---|---|
     | uniform 2.0× on a rim-top wipe | 20–26 mm |
     | uniform 2.2× on a lid-side wipe | 44.5 mm, with warning 4105 |
     | bowl-outside scrub | must run at 0.70× taught to stay inside 20 mm |

   The analyst's draft capped speed by the ratings only. This is the main safety correction.
6. **Near-limit case R10_HNGR.**
   - J4 reaches 133.47°, a margin of 1.53° to its 135° limit. The J4 rate on the way into that peak is about 21.5 °/s.
   - Under ALIX's current G1 rule (`EM`: floor 0.85°, τ_limit 0.139 s, eps 0.5°), the arm is predicted to overshoot by about 3.0° and cross the limit. That is a must-fix. Slowing the arrival to ≤ 7.4 °/s would satisfy the model.
   - The pre-2026-09-15 rule (2° hard floor, `TT:162-173`) is superseded (`TT:195`, `TT:850-853`).
   - Hardware has run past J4's limit: a slow production track overshot by +5.6–5.9° on a cold arm (`FIND:6721-6737`).
7. **Smoothing buys little speed and no accuracy on the upload channel.**
   - The safe uniform speed-up allowed by the ratings is 1.35–2.76× raw. Smoothing at 5 Hz adds 3–6 %, and at 2 Hz 7–10 %. Verified.
   - ALIX's corpus shows no executed-accuracy gain on the upload channel (median 1.04×, `GIC:77-99`).
   - Light smoothing's real job is removing recorder ripple: FILE_6's per-tick utilisation drops from 1.08 to 0.69 (verified).

---

## R2 — Trajectory editing per region and sub-region, with labelled undo

**Original (`REQ:10`):** "2. Trajectory Editing: Each region and sub-region should have independent options to adjust smoothing, trimming, and scaling, with an undo option for up to five previous changes. - Trajectory Optimization Tab (/home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html)"

### Revised requirement

For every taught region, Lite shall let the operator optimise the trajectory with three operations: **Remove stops** (trimming), **Smoothness** and **Speed** (time scaling). A region is one arm's recording for one deck region or task, with an ID as proposed in map 13, for example `R-R01-P1`.

- (a) Each operation can be set for the whole region. A one-click **"Auto-optimise (recommended)"** applies the safe default recipe.
- (b) Each operation can be overridden on any number of non-overlapping sub-regions (time windows). A sub-region that is not overridden inherits the region setting. A sub-region can keep its stops or add a timed dwell.
- (c) Edits are non-destructive. The raw recording is never changed, and the result is recomputed from the raw file plus an ordered recipe.
- (d) Sub-region boundaries are blended. There is no position step, and speed changes only along a bounded ramp.
- (e) Each region keeps an undo/redo history of at least its last 5 changes. Each change has a plain-language label, and the history is saved with the region.
- (f) "Save version" writes a new numbered optimised file with its recipe, checksums and check results. Earlier versions are kept.
- (g) The speed used on every line is the lowest of four ceilings:
  - the operator's setting for that sub-region;
  - the joint-rating ceiling at the 5 ms tick;
  - the path-accuracy ceiling from ALIX's execution model (G5);
  - the joint-limit arrival ceiling (G1).
- (h) A version can go to simulation, real-arm test or upload only if it passes all of these:
  - G2: 0 band-limited ticks over a joint rating at 5.0 ms;
  - G3: every step ≤ 10°;
  - commanded positions inside the joint limits, and G1 predicted executed margin ≥ 0.5° at every joint extreme;
  - G4: the commanded pad path within 10 mm of the raw recording at the contact frame, in both directions (5 mm preferred);
  - G5: predicted executed departure within the region's executed budget (ALIX default 20 mm, tighter where set).

### Clarifications

- **Region and sub-region.** A region is one taught unit stored as one raw `.txt`. A sub-region is a time window inside it. The prototype's "scaling region" is a Lite sub-region.
- **Trimming** means removing the stops the operator made while teaching. Lite follows ALIX `strip_pauses`:
  - a run of ≥ 3 lines that all stay within 0.05° of the run's **first** line collapses to that first line;
  - the file's last line is always kept (`TT:915-953`, `TT:240-241`).

  Cropping the start or end is an open question.
- **Scaling** means time scaling. The operator sees "Speed 1.5× — takes 12.3 s instead of 18.5 s". Internally this is λ, the speed factor, where 1.0 means as taught. The prototype's "Scale %" is a duration percentage, so 75 % ≈ 1.33×.
- **Undo "up to five"** is read as at least 5 levels. The history is linear per region, one gesture makes one entry, and a new change clears redo.
- **Processing order** is ALIX's: Smooth (at the 5 ms record clock) → Remove stops → Speed ceilings → dwells → rating fit. This is exactly what ALIX `shape_search` does (`TT:2588`, `TT:2608-2611`, `TT:1855-1985`). The prototype uses Trim → Smooth → Scale (`H:1691-1719`).
- **Boundaries are stored in raw-recording lines** (time at 5 ms), so they stay valid when stop removal or speed changes the output length.
- **Two different deviation budgets — do not mix them up:**
  - **G4** compares the commanded bytes with the raw recording. ALIX allows 10 mm and prefers 5 mm (`TT:129-132`).
  - **The executed budget (G5)** covers what the arm will really do. ALIX's default is 20 mm (`TT:133-138`, `TCTR:3-9`, `TCTR:218-222`).
  - The "MAX DEV" column of `CC/description.txt` (FILE_10.1 and FILE_11: 5 mm; FILE_06 and FILE_08: 10 mm) is an **executed** TCP budget (`CC/description.txt:19`).

### Findings

| # | Finding | Evidence |
|---|---|---|
| 1 | Only scale can be set per sub-region in the prototype. Trim and smooth parameters are global. A region is `{start, end, scalePct}`. | `H:729-737`, `H:1392-1415`, `H:1568-1616` |
| 2 | The prototype has no undo, redo or history. | grep over `H` |
| 3 | Prototype sub-regions:<br>• entered as original row numbers;<br>• start/end cannot be edited after creation; only the scale % can;<br>• never drawn on the plot; only suggestion rectangles and the cursor are;<br>• overlap refused;<br>• box-select creates them. | `H:1337-1371`, `H:1392-1415`, `H:1134-1142`, `H:872-892` |
| 4 | Prototype idle rule:<br>• the rolling mean of \|Δq\| ≤ 8 millidegrees per line on all joints (≈1.6 °/s) for ≥ 20 transitions counts as idle;<br>• each idle block collapses to its first and last rows;<br>• measured collapse jumps are 0.11–0.41° on commode files (M07 §9 R-A2).<br>ALIX moved to an anchor rule after its own step-by-step judgement "cut[] the apex by up to 9 mm at the pad". | `H:1438-1492`; `TT:915-953` (comment `TT:926-928`) |
| 5 | Prototype default smoothing is a 101-point centred mean (≈0.88 Hz), plus a mean correction and an endpoint overwrite. Pad deviation by FK at `L_glove_frame_2` (verified):<br>• FILE_08: 9.05 mm max (p95 5.32);<br>• FILE_10.1: 10.39 mm (p95 4.67), or 10.60 mm from taught path to new path.<br>ALIX's 5 Hz filter gives 1.53 and 0.91 mm. | `H:1494-1517`; `TT:894-912`; Appendix V |
| 6 | Endpoint snap (verified):<br>• first step 966 (J3, FILE_08) and 1005 (J6, FILE_10.1);<br>• last step 546 (J2, FILE_01.1);<br>• each above the file's largest interior step (429 / 392 / 311);<br>• not universal: FILE_6 is 254 against 636. | `H:1513-1514`, `H:1721-1723` |
| 7 | No blending at boundaries. FILE_01.1 with default suggestions gives 640 / 440 / 880 / 840 °/s² one-tick speed changes against an 80 °/s² median. One count per tick² is 40 °/s², so the median is quantisation noise (verified). | `H:1568-1616` |
| 8 | No rating, step or limit check. Manual scale % accepts down to 1 % with no maximum. | `H:1352`, `H:1362` |
| 9 | ALIX algorithms R2 needs. All are stateless CLI functions; none has undo.<br>• `strip_pauses`.<br>• `smooth_taught`: order-2 zero-phase Butterworth at the record clock; `padlen` = min(450, n−1).<br>• `capped_millideg`: keeps points on the taught chords; per-line λ ceiling `lam_cap_line`; backward and forward ramp passes. Its **default `clock_s` is the 5.5545 ms nominal clock**, so callers must pass 0.005.<br>• `GUARD_RAMP` 0.01 and `MODEL_RAMP` 0.05.<br>• `--section A:B:F` with F ∈ (0, 1] relative to the fastest speed: sections can only slow down.<br>• `RATING_FILTER_ALLOWANCE` 0.97.<br>• `insert_dwells` for deliberate stops.<br>• `rate_fit_track` only slows. It tightens the allowance **globally** by 2 % per pass, for at most 6 passes, and raises an error if the file is still over. | `TT:915-953`, `TT:894-912`, `TT:767-832`, `TT:176`, `TT:226`, `TT:229`, `TT:1439-1451`, `TT:1873-1876`, `TT:1454-1483`; `CT:1675-1714` |
| 10 | ALIX's default re-timing is the execution-model rung `model_millideg`. It applies:<br>• the ratings × 0.97 on every line;<br>• `departure_lambda_cap` = budget ÷ predicted departure at λ = 1, per source line, in closed form;<br>• `model_caps`: arrival-rate caps at joint limits;<br>• sections, then dwells.<br>Map 06 §2.8 item 4 already said to add the departure ceiling once Lite has FK, and Lite will have FK. | `TT:1855-1985`, `TT:1817-1852`, `TT:1377-1437`, `TT:4371`; `EM` |
| 11 | Hardware evidence that rating-only speed-ups are not "safe" on this channel:<br>• uniform 2.0× (rim top): measured p95 path error 20–26 mm;<br>• uniform 2.2× (lid side): 44.5 mm, with warning 4105;<br>• a bowl-outside scrub needs 0.70× taught to stay inside 20 mm;<br>• the median off-curve distance goes 1.34 → 2.28 → 3.00 mm at 1×, 1.35× and 1.61×. | `TCTR:210-214`; `FIND:6706-6716` |
| 12 | The ALIX execution model is a p95 statement: "the worst case runs about twice it". It was fitted on the same controllers Lite uses (`RUN14` ARM_IPS; M10 §5 item 2). | `TCTR:93-99`; `RUN14:47-48` |
| 13 | ALIX Studio's undo is a global memento stack:<br>• deep clones of config, points, sequence, generator knots, picked poses and paint labels;<br>• capped at 80, unlabelled, not persisted, cleared on load;<br>• Ctrl+Z, Ctrl+Y and Ctrl+Shift+Z, with a typing guard;<br>• one entry per gizmo drag.<br>The mechanism is ~40 lines. The captured state belongs to the X1 geometric editor. | `JS:1839-1977`, `JS:5697-5698`, `JS:7108-7114`, `JS:276-279`, `JS:349-383` |
| 14 | No taught `.txt` editing anywhere in the Studio. `server.py` has no `.txt`, taught or millidegree handling, and only two Plotly charts exist. | `SRV` grep; `JS:5368`, `JS:14483` |
| 15 | The raw recordings behind V6 are not in the workspace. V6 files are merged, time-stretched outputs. | M10 §5 item 5 |
| 16 | The safe uniform speed-up from the ratings alone is 1.35–2.76× (verified): FILE_6 1.35, FILE_08 2.16, FILE_02 2.36, FILE_10.1 2.38, FILE_01.1 2.76. This is an **upper bound**: the G5 ceiling can be far lower (finding 11). | Appendix V |
| 17 | Endpoint behaviour of ALIX's filter (verified):<br>• 5 Hz, order 2, `padlen` 450 returned the exact first and last points after integer rounding on FILE_01.1, 02, 08, 10.1 and 6, with first and last steps of 0–1 millidegree;<br>• F3_LDTPS04 starts in motion: 32 / 37 millidegrees, against the boxcar's 715 / 468. | Appendix V |

### Borrow verdict

**ALIX Studio: Partial — borrow some, build the rest.**
- Borrow as patterns:
  - the undo mechanics: push before mutation, undo/redo stacks, footer buttons, shortcuts, typing guard, one entry per gesture (`JS:1896-1977`, `JS:7108-7114`, `JS:276-279`, `JS:349-383`);
  - stale-on-edit (`markDirty` → `invalidateValidation`, `JS:5371-5430`). Its code references X1 state (points, IK status, coverage, Deploy), so it is re-implemented, not copied.
- Borrow as-is: the status line, error toast and `showStudioModal`.
- Build everything trajectory-specific, plus a per-region history that is labelled, persisted, at least 5 deep and stores recipes.

**Other sources:**

| source | verdict | what to take or check |
|---|---|---|
| Trajectory HTML (`Alix_Studio_Lite_Trajectory_V2.html`) | Borrow with adaptation | **Take:** the sub-region model, the index mapping (`keptOriginalIndices`, `mapRegionsToTrimmed`, `lowerBound`/`upperBound`, `H:1438-1566`), box-select (`H:872-892`), the pure functions as preview reference, and the report schema (`H:1732-1755`).<br>**Replace:** the trim and smooth algorithms.<br>**Add:** boundary ramps and all gates. |
| ALIX `TT` / `CT` / `PTR` | Borrow with adaptation (port, not import) | **Port:** `strip_pauses`, `smooth_taught`, `capped_millideg` (with section ceilings and ramps), `departure_lambda_cap` with `pad_departure_joint_mm`, `model_caps` with the `EM` constants, `insert_dwells`, a locally tightened `rate_fit_track`, the exact `deviation_mm`, and the rate/step/position reports.<br>**Why port:** importing `TT` pulls in `cleaning_tracks`, exp01 and scipy (`TT:84-91`), and the module is a CLI with a registry. |
| `alix_dispatch/track.py` | Borrow as-is | `parse_track`: strict, 7 integers per line, ≥ 2 lines, refuses the whole file (`TRK:66-82`).<br>`expected_replay_s`: the player-law duration (`TRK:1558-1570`). |
| Upload-channel control app (runner v14) | Not applicable to editing | Output must pass `read_track` (`RUN14:76-89`). Raw input comes from its drag-teach save (`RUN14:957-989`). |

**Check of the input note** ("Trajectory Optimization Tab (HTML)"): partly accurate.
- The note points at the HTML, not at ALIX Studio. It makes no Studio-borrow claim.
- The HTML is the right starting point for the tab: it has per-sub-region scaling, box-select, a staged pipeline and a report.
- It does not meet R2:
  - trim and smooth are global;
  - there is no undo;
  - three algorithms have measured defects: the endpoint snap, smoothing 9–10 mm off the path, and boundary speed jumps;
  - there is no rating, limit or path-accuracy check.

### Borrow items

| item | from | adaptation |
|---|---|---|
| Sub-region model and original→processed index mapping | `H:1438-1566` | Store boundaries in raw lines. Combine the stop-removal keep-mask with the re-timing parameter u (`capped_millideg(..., return_u=True)`, `TT:767-832`), so a selection on either plot maps back to raw lines. |
| Box-select to create a sub-region | `H:872-892`, `H:1392-1415` | x axis in seconds; labelled shaded bands; drag edges; "Split here"; keep the overlap rule. |
| Pause removal (anchor rule) | `TT:915-953`, `TT:240-241` | Pure numpy, applied only inside spans where it is enabled. Report removed seconds at 5 ms. |
| Dwell insertion for deliberate stops | `TT:1454-1483` (`parse_dwells`, `insert_dwells`) | A per-sub-region "Keep stops" or "Hold N s here". Note that ALIX repeats lines at the 5.5545 ms clock. |
| De-jitter smoothing | `TT:880-912` (`smooth_taught`) | Presets Off / Light 5 Hz (default) / Medium 3 Hz / Strong 2 Hz, with 1–8 Hz under Advanced. Cross-fade differently filtered signals over 0.25 s between sub-regions. Never overwrite endpoints. Pin scipy in the Lite venv. |
| Per-line speed ceiling with ramps | `TT:767-832`, `TT:176`, `TT:226`, `TT:1439-1451`, `TT:1873-1876` | `lam_cap_line` = region λ × sub-region λ. Pass `clock_s=0.005` explicitly. Ramp 0.01 per source line. |
| Path-accuracy ceiling (G5) and limit-arrival ceiling (G1) | `TT:1817-1852` (`departure_lambda_cap`), `TT:1107-1165` (`pad_departure_joint_mm`), `TT:1377-1437` (`model_caps`), `EM` | Recompute when smoothing or stop removal changes the source (≈4–6 s per 12–19k lines in ALIX, `TT:1828`, so run it in the background). Cache by source SHA, smoothing setting and budget. Version the `EM` constants with the app. |
| Rating fit | `CT:1675-1714` (`rate_fit_track`) | Tighten **only** the stretches still over a rating (±ramp), not the global allowance, so a sub-region edit stays local. Cap the passes. If no fit is found, the version is a draft only. |
| Undo/redo mechanics | `JS:1896-1977`, `JS:7108-7114`, `JS:276-279`, `JS:349-383` | One stack per region. Store recipe snapshots, not arrays. Label every entry. Persist with the region. Recompute on restore, with outputs cached by recipe hash. |
| Stale-on-edit | `JS:5371-5430` (pattern) | Any recipe change marks checks, simulation, real-test and upload readiness as out of date. |
| Strict parser and integer writer | `TRK:66-82`; `H:1618-1620` | Refuse malformed or non-integer files (the prototype silently skips them, `H:803-829`). Write 7 integers per line. |

### Build items

| item | approach |
|---|---|
| Recipe model and server-side processor (Python) | A JSON recipe holds: the raw SHA-256; region operations; ordered sub-regions with optional overrides and dwells; blend settings; budgets (G4 10/5 mm, executed budget). A deterministic pipeline runs: smooth with per-sub-region cross-fade → remove stops per span → map boundaries → per-line λ ceiling = min(operator, G5 ceiling, G1 caps), with ratings × 0.97 as caps → `capped_millideg` at 0.005 s with a 0.01 ramp → local rating fit → dwells → integers. Outputs are cached by recipe hash. |
| Speed-ceiling composer and explanation | For each line, record which ceiling binds: ratings, path accuracy, joint limit or operator. This drives "Limited by path accuracy at the stroke ends" messages and the safe maximum per sub-region. |
| Auto-optimise (recommended) | The default recipe: Light smoothing, stops removed, speed = min(fastest allowed, 2.0×) on every sub-region. All gates checked. Shown with its duration and limits before saving. |
| Per-region undo history | Entries `{seq, label, recipe, recipe_sha, at}` stored in the region's recipe file with a pointer. Undo and redo move the pointer; a new change truncates redo. Labels come from the diff. The UI lists the last 5 with "go back to here". |
| Sub-region editor UI | Labelled bands on the timeline plot. Create by box-select or "Split here"; drag edges; rename; delete. A per-sub-region panel holds the three controls plus Keep stops / Dwell, a "Same as region / Custom" chip and "Reset to region". |
| Boundary blending | A 0.25 s raised-cosine cross-fade for smoothing, and a λ ramp of ≤ 0.01 per source line. A property test checks that the boundary acceleration stays within the AC5 bound. |
| Versioned save | Write `<ID>__OPT_v<k>.txt` and a sidecar JSON: recipe; raw and output SHA-256; check results G1–G5; `EM` version; tool version. The raw file is never modified. |
| Optional JS live preview | Not needed in release 1: server compute is small, apart from the background G5 ceiling. If built, it needs golden parity tests (±1 millidegree) and never produces saved bytes. |

### Recommendation

Keep the prototype's sub-region interaction. Move canonical processing to a server-side Python recipe processor that ports ALIX's taught-track lane:
- `strip_pauses`;
- `smooth_taught`;
- `capped_millideg`, with the rating, path-accuracy (G5) and limit-arrival (G1) ceilings;
- `insert_dwells`;
- a locally tightened rating fit.

Re-scope the Studio's undo pattern to one persisted, labelled history per region that stores recipes.

Make **Auto-optimise** the default path for operators. Make sub-region editing the "fine-tune" path.

Do not present the rating-only ceiling as "safe". ALIX's hardware evidence shows rating-capped 2× speed-ups leaving the path by 20–45 mm. The browser previews only; the server's output is the only file that can be simulated, run or uploaded.

### Approach steps

1. **Recipe schema v1**, one per region. Fields:
   - `unit_id`, `arm`;
   - `source{file, sha256, lines, clock_s=0.005, provenance raw|legacy_processed}`;
   - `region{smooth, stops{remove, tol_deg=0.05, min_lines=3}, speed{lambda}}`;
   - `sub_regions[{id, label, start_line, end_line, smooth|null, stops|null, speed|null, dwell_s|null}]`;
   - `blend{smooth_crossfade_s=0.25, speed_ramp_per_line=0.01}`;
   - `limits{rate_design=0.97, gate_clock_s=0.005, g4_mm=10, g4_prefer_mm=5, exec_budget_mm, em_version}`;
   - `history[...]`.
2. **Port into `alix_studio_lite/traj/` as pure numpy, with scipy pinned:**
   - parse;
   - `smooth_taught`;
   - `strip_pauses` restricted to spans;
   - `capped_millideg`;
   - `departure_lambda_cap` and `pad_departure_joint_mm`, using FK from `alix_workspace` (`UK:231-259`);
   - `model_caps`;
   - `insert_dwells`;
   - the exact `deviation_mm` (`TT:607-716`).

   Write golden tests against ALIX's functions, run in the ALIX venv.
3. **Processor.** Smooth → stops → map boundaries → λ ceiling = min(operator, G5, G1) → `capped_millideg(clock_s=0.005, caps=0.97×rating, ramp=0.01, lam_cap_line)` → local rating fit until 0 band-limited ticks → dwells → integer CRLF.
4. **Checks after every recompute** (the shared module, see R12):
   - G2;
   - G3;
   - commanded limits;
   - G1 (execution-model margin);
   - G4 (exact, both directions);
   - G5 (predicted executed departure);
   - endpoint and boundary checks.
5. **Server API:**
   - `GET/POST /api/units/{id}/recipe`;
   - `POST …/undo`, `…/redo`;
   - `POST …/versions`;
   - `GET …/ceilings` (per-line binding ceiling, computed in the background).
6. **History:** append on every committed gesture; move the pointer on undo/redo; truncate redo on a new change; persist in the region store.
7. **UI:**
   - region header with chips;
   - timeline with sub-region bands;
   - an Auto-optimise button;
   - region and sub-region panels;
   - Undo/Redo with labelled tooltips.
8. **Legacy files** (V6, lid_top):
   - import as `legacy_processed`;
   - default to "fit to ratings" only;
   - smoothing off by default;
   - show G1/G5 findings.
9. **Tests:**
   - an identity recipe gives identical points;
   - property tests for AC2, AC5 and AC6;
   - undo ≥ 5 end-to-end (Playwright);
   - ceiling composition;
   - golden tests against ALIX.

### Acceptance criteria

1. With all operations off, the saved output points are identical to the raw file.
2. **Locality.** Remove stops, Smoothness and Speed can each be set on the region and overridden per sub-region. After a change to one sub-region, outside that sub-region plus its blend and ramp zones:
   - the smoothed source path is identical;
   - the speed profile λ(u) along the source is identical.

   The ramp zone is ≤ |Δλ| ÷ 0.01 source lines; the blend zone is ≤ 0.25 s. Points re-sampled downstream may sit at different positions on the same chords.
3. Five consecutive changes can each be undone and redone, restoring the previous output point for point. The history survives an app restart. Undo in one region never changes another region.
4. Every history entry is labelled with the region or sub-region, the operation, and the old → new value.
5. Within ±0.25 s of every sub-region boundary, each joint's 20 Hz band-limited acceleration is ≤ max(600 °/s², that joint's 99th percentile elsewhere).
6. **Endpoints:**
   - the first and last output points equal the raw first and last points;
   - the first and last 20 steps are each ≤ max(2 × the median step of the next 100 lines, 0.05°);
   - this holds on FILE_08 and FILE_10.1, where the prototype fails.
7. Output is 7 integer millidegrees per line, ≥ 2 lines, and runner v14 `read_track` and `alix_dispatch.parse_track` accept it unchanged.
8. With "Keep within arm limits" on (the default, and required to send), every saved version passes:
   - G2 (0 band-limited ticks over a rating at 5.0 ms);
   - G3;
   - commanded limits;
   - G1 ≥ 0.5°;
   - G4 ≤ 10 mm;
   - G5 ≤ the region's executed budget.
9. **Light smoothing alone** moves the pad by at most 1.0 mm on FILE_10.1 (measured 0.91 mm) and at most 1.6 mm on FILE_08 (measured 1.53 mm).
10. **Speed ceilings.** For a sub-region where the operator asks for more speed than allowed:
    - the output speed equals the lowest ceiling on each line;
    - the UI names the binding ceiling;
    - on a bowl-outside scrub (FILE_02) with a 20 mm executed budget, the G5 ceiling binds below the ratings ceiling.
11. The raw file's SHA-256 is unchanged after any sequence of edits and saves.
12. One change on a 12,000-line file recomputes and refreshes the plot in < 1 s. A G5 ceiling refresh after a smoothing change finishes in < 10 s in the background.

### UX notes (non-technical operators)

- Make "Auto-optimise (recommended)" the primary action. Sub-region fine-tuning is secondary.
- Use plain names: "Remove stops", "Smoothness", "Speed", "Keep stops here". Show seconds and degrees, never rows or counts. Put parameters under Advanced.
- Show sub-regions as coloured, labelled bands. Click to select, drag edges to adjust.
- Show the effect at once: "Takes 41.2 s (was 62.1 s) — saves 20.9 s".
- The speed slider shows the allowed maximum and why, for example "Limited by path accuracy at the stroke ends" or "Limited by joint 6 speed".
- Keep Undo/Redo always visible with labelled tooltips and a "Last 5 changes" list. No confirm dialogs for edits.
- The executed budget (path accuracy on the robot) is set per region by an engineer at setup, not by the operator.

### Safety notes

- Smoothing changes the path. Any smoothing change marks simulation stale and needs a collision re-check (R11, R5).
- The ratings ceiling alone is not "safe". On this channel, rating-capped uniform 2.0–2.2× runs measured 20–45 mm p95 executed path error (`TCTR:210-214`). G5 is also a p95 prediction; the worst case is about 2× it (`TCTR:93-99`). Simulation plus a slow first real run stay mandatory.
- G1 matters because a joint overshooting its limit can stop a run. J4 has run past 135° on hardware (`FIND:6721-6737`; `TCTR:222`).
- Some stops are deliberate (press, soak). "Keep stops" / "Hold N s" must exist per sub-region.
- In a dual-arm step, any speed change shifts one arm against its partner, so the dual-arm clearance must be re-checked (R10, ALIX windowed separation gate).

### Dependencies, risks, open questions

**Depends on:**
- R1: region store, unit IDs, raw recordings, per-region contact frame and executed budget;
- INT-TRAJ: tab shell and plots;
- R12: shared checks, with one-click fixes that write into the same history;
- R11, R3, R5: simulation and collision re-check after edits;
- R7: pole height per region, for FK;
- NFR-THEME;
- the `EM` constants;
- a pinned venv with numpy and scipy.

**Risks:**
- Raw recordings are missing for V6 regions.
- The execution model is p95 only and is fitted to these controllers; refit after firmware or hardware changes.
- G5 can make scrubs slower than taught, which operators may not expect.
- A JS preview could drift from the Python processor.
- "Trim" may be read as "cut the ends".
- A linear history cannot undo one sub-region selectively.

**Open questions:**
- Is a region one taught file per arm? Are sub-regions free windows or the deck's PART 1/2?
- Should trimming also crop the start and end?
- Is "five" exact or a minimum, and must the history persist? Recommended: ≥ 5, persisted.
- Which regions keep stops?
- What executed budget applies per region: ALIX default 20 mm, or 5/10 mm as in `CC/description.txt`?
- Does G5 gate in release 1 or start advisory? Recommended: gate.

**Priority: Must. Effort: L (≈14–15 person-days).**
- Processor, ports and golden tests: 7 days, including ≈2 days for the G5/G1 ceilings.
- History and versioning: 2 days.
- Sub-region editor UI: 4–5 days.
- Assumes the INT-TRAJ shell exists. Add ≈3 days if the optional JS preview is built.

---

## R6 — Adjustable plots and editing windows

**Original (`REQ:18`):** "6. Adjustable plot : The UI should allow users to resize and adjust the graph, and editing windows based on their requirements. - check the /home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html"

### Revised requirement

On the Trajectory Optimisation screen the operator shall be able to:
- (a) resize the plot area against the 3D simulation and the editing panels, by dragging or with the keyboard;
- (b) change the height of each stacked plot;
- (c) maximise any one plot and restore the exact previous layout with one click or Esc;
- (d) zoom and pan the time axis:
  - plots of the same file stay on the same time window;
  - before/after comparison plots stay on the same part of the path, mapped through the raw-line index;
  - one click resets the view;
- (e) show or hide individual joints;
- (f) collapse, expand and resize the editing panels, and move and resize floating dialogs;
- (g) have the layout remembered per screen, with "Reset layout".

Plots re-draw within 100 ms of a resize. There is no horizontal page scroll at 1366×768.

### Clarifications

- "Graph" means the joint-position plots, the speed-utilisation plot and the comparison plots. "Editing windows" means the region and sub-region panels and floating dialogs.
- Editing data by dragging plot points is out of scope. Data changes go through R2.
- The mouse wheel scrolls the page. Zoom uses Ctrl+wheel, pinch or the toolbar. This reverses the prototype's `scrollZoom:true` (`H:864`).
- Use a curated layout with splitters, not free docking, as ALIX's UX research recommends (M01 §7 item 17).
- Before/after plots have different durations. Linking them by the same seconds would show different strokes, which is what the prototype's compare cursor does (`H:1290-1304`). Linking by raw-line position, through the re-timing map u, keeps the same stroke in view.

### Findings

| # | Finding | Evidence |
|---|---|---|
| 1 | Prototype plot heights are fixed in CSS: `.plot` 600 px and `.plot.small` 520 px. `.plot` drops to 480 px under 850 px, but `.plot.small` does not. Width is responsive, and plots are resized after a step change. | `H:125-129`, `H:311-313`, `H:862-867`, `H:909-915` |
| 2 | Prototype zoom/pan, legend toggles and PNG export work. But:<br>• the wheel zooms and hijacks page scroll;<br>• there is no linked zoom (no `plotly_relayout` handler);<br>• the x axis is the row number. | `H:847-867` |
| 3 | Studio docks:<br>• drag and keyboard resize (←/→ 16 px, Shift 64 px, Home/End);<br>• clamped to 180–620 px with a 320 px viewport floor;<br>• persisted once on pointer-up;<br>• aria separators. | `JS:7564-7645`; `CSS:331-341` |
| 4 | Studio dialogs: draggable; a corner grip resizes them by pointer and keyboard (min 420×320); their box is persisted inside try/catch and clamped back on-screen on window resize. | `JS:8389-8523` |
| 5 | "Maximize view" collapses the docks and restores exactly the previous arrangement (`preMaximize`). It works for the 3D viewport only. | `JS:7538-7559` |
| 6 | Studio plots cannot be resized. `#analysisPlot` is 190 px and `#dacPlot` 64 px. `onResize` resizes only `analysisPlot`; `dacPlot` is resized on tab entry. | `CSS:928`, `CSS:1108`, `CSS:2023`; `JS:632-645`, `JS:7410` |
| 7 | Plotly layouts hard-code the text colour `#e6edf3`, which gives 1.18:1 contrast on the white panel (verified by the WCAG formula). There is no plot-theme helper. | `JS:5368`, `JS:14488`; M01 §5 |
| 8 | Dock width storage reads and writes are not in try/catch, unlike dialog writes. | `JS:7588`, `JS:7595` vs `JS:8447-8453` |
| 9 | The Studio's dark theme exists only as CSS. No JS sets `data-theme`, so Lite must add a toggle if it wants one. | `CSS:131-134`; M01 §1.3 |
| 10 | The Studio's dock tests are static source-text guards, not browser tests. | `TIG:2388-2525` |

### Borrow verdict

**ALIX Studio: Partial — borrow some, build the rest.**
- Borrow:
  - the dock separator (drag, keyboard, clamp, persist on release, aria) (`JS:7564-7645`, `CSS:331-341`);
  - dialog drag, resize, persist and clamp (`JS:8389-8523`);
  - maximise-with-restore (`JS:7538-7559`);
  - `ensurePlotly` with vendored Plotly 2.35.2 (`JS:5322-5337`).
- Build:
  - horizontal height splitters;
  - per-plot maximise;
  - a ResizeObserver resize bus;
  - linked zoom: by time within a file, by raw-line position for before/after;
  - a token-based `plotTheme()`;
  - per-screen layout persistence.

**Other sources:**
- **Trajectory HTML: borrow with adaptation.** Keep the Plotly config (responsive, no logo, lasso removed) and resize-after-show. Change the x axis to seconds and turn off scroll-zoom.

**Check of the input note:** partly accurate.
- The HTML gives zoom, pan, legend toggles and responsive width.
- It has fixed heights, no resizable windows, no maximise and no linked zoom.
- The resize machinery is in ALIX Studio, which does not apply it to plots.

### Borrow items

| item | from | adaptation |
|---|---|---|
| Dock separator | `JS:7564-7645`, `CSS:331-341` | Generalise to `initSplitter(el, {axis, cssVar, min, max, storageKey})`. Wrap storage in try/catch under `alixLite.layout.*`. |
| Dialog drag/resize/persist/clamp | `JS:8389-8523` | Use for floating panels: the findings list and sub-region details. Rename the storage keys. |
| Maximise with exact restore | `JS:7538-7559` | Apply per plot card. Store the splitter sizes; restore on Restore or Esc. |
| Offline Plotly loading | `JS:5322-5337` + `vendor/plotly.min.js` 2.35.2 | Serve from Lite's `/vendor/`. It is the same version the prototype loads from the CDN (`H:7`). |
| Plotly config and resize-after-show | `H:862-867`, `H:894-920` | x axis in seconds; `scrollZoom` false; resize via the resize bus. |

### Build items

| item | approach |
|---|---|
| Resize bus | A ResizeObserver per plot body calls `Plotly.Plots.resize` inside `requestAnimationFrame`. |
| Linked axes | Plots register in a link group. On `plotly_relayout`, propagate under a re-entrancy flag. Mode "time" for plots of the same file; mode "path" for before/after, mapped through the raw-line index u from the R2 processor. "Reset view" resets everything. |
| Plot card toolbar | Labelled buttons Zoom, Move, Reset view, ⤢ Maximise, and J1–J7 chips synced with the legend. |
| Layout persistence | Per-screen JSON `{splitters, collapsed, joints}`, written on pointer-up inside try/catch. "Reset layout". |
| Plot theme | `plotTheme()` reads the tokens for text, muted, grid and transparent background, and re-applies them if Lite adds a theme toggle. Define `--joint-1..7` tokens. |
| Responsive rules | At ≤ 1366×768: sim ≥ 320 px, plots ≥ 360 px, each plot ≥ 160 px, editing panel collapsible. |

### Recommendation

Lift the Studio's separator, dialog and maximise code into a small layout library. Apply it to a curated Trajectory screen: [3D sim | plots column | editing panel], with stacked plot cards separated by horizontal splitters. Add linked zoom (time within a file, path position for before/after), labelled toolbars and per-screen persistence.

### Approach steps

1. Build the screen grid from CSS variables. Lift `setDockWidth`/`initDockResize` into `initSplitter` with an `axis` parameter. Keep the aria separator, the 16/64 px keyboard steps, the neighbour clamps and persistence on pointer-up.
2. Plot card: header plus toolbar. ⤢ stores the splitter sizes and restores them on Restore or Esc.
3. A ResizeObserver per plot body, with `Plotly.Plots.resize` in rAF.
4. Linked zoom in two modes: time within a file, and raw-line position for before/after, using the R2 u-mapping.
5. Defaults: drag zooms (select only in sub-region mode); `scrollZoom` false with a custom Ctrl+wheel; double-click resets; joint chips remembered.
6. `plotTheme()` from the tokens.
7. Playwright tests:
   - splitter drag and keyboard;
   - persistence after reload;
   - maximise and restore;
   - both link modes;
   - no horizontal scroll at 1366×768;
   - blocked storage.

   Re-express the Studio's static dock guards (`TIG:2388-2525`) against Lite's code.

### Acceptance criteria

1. Dragging the sim/plots separator resizes both sides continuously. Plots re-lay out within 100 ms of release, and minimum sizes hold.
2. Each plot's height can be changed by drag (min 160 px) and by keyboard (Arrow ±16 px, Shift+Arrow ±64 px, Home/End).
3. Maximise fills the plot area. Esc or Restore returns every pane to its prior size (±1 px).
4. **Linked zoom:**
   - zooming one plot of a file applies the same time window to that file's other plots in the same frame;
   - zooming the "before" plot shows the same raw-line span in the "after" plot;
   - "Reset view" restores everything.
5. The mouse wheel over a plot scrolls the page unless Ctrl is held.
6. Splitter sizes, collapsed panels and visible joints are restored after reload. "Reset layout" restores the defaults. With storage blocked the page still works.
7. At 1366×768 the sim, at least one plot and the editing panel are visible, with no horizontal scroll.
8. Plot text and grid colours come from tokens, and tick labels have ≥ 4.5:1 contrast.

### UX notes

- 8 px grab handles with a visible bar on hover or focus.
- Text-labelled toolbar buttons; "Reset view" always visible.
- An "Esc to restore" hint when maximised.
- Joint toggles keep the current window.

### Safety notes

- Maximising or resizing must never hide the region's check status, the robot-state indicator or the Stop control during a run.

### Dependencies, risks, open questions

**Depends on:**
- INT-TRAJ (plot cards);
- R2 (u-mapping for path-linked zoom);
- R10 (sim-left / plots-right layout);
- NFR-THEME.

**Risks:**
- Plotly speed with 7–14 traces × 10k points. Use `scattergl` above 15k points (`H:835`) or per-pixel decimation.
- Linked-zoom feedback loops.
- Small field laptops.

**Open questions:**
- Is a pop-out plot window for a second monitor needed?
- Should the layout be stored per user or per machine?
- Which plots are shown by default?

**Priority: Should. Effort: M (≈5–6 person-days)**, including Playwright tests. Assumes the plot cards from INT-TRAJ.

---

## R11 — Smoothness adjustment with immediate (tiered) simulation validation

**Original (`REQ:28`):** "11. Smoothness Adjustment: Users should be able to adjust trajectory smoothness and immediately validate the changes through simulation."

### Revised requirement

For each region and sub-region the operator chooses a smoothness level from named presets: Off, Light (recommended), Medium, Strong. A numeric cutoff is available under Advanced.

**Within 1 s of a change, Lite shows:**
- (a) the before/after joint curves;
- (b) how far the cleaning pad moves from the taught path, in mm, measured exactly in both directions at the region's contact frame, against the G4 limit (10 mm, 5 mm preferred);
- (c) the speed, step and joint-limit checks.

**Within a few seconds, Lite shows:**
- (d) the predicted executed departure (G5) against the region's executed budget;
- (e) a collision re-check of the changed part against the commode (in the region's lid/seat state), walls, bin and UGV, with the cleaning pad's intended contact allowed.

**In the background, Lite runs** the dual-arm clearance check against the partner.

The operator can play the result in 3D, with the original as an optional ghost, and jump to any problem. Any smoothness change marks earlier simulation and real-test results out of date. A level that breaks G4, or the executed budget, cannot be saved as sendable.

### Clarifications

- **Smoothness** means low-pass filtering of the joint trajectory, to remove hand tremor and recorder ripple. It is separate from speed.
- **"Immediately validate through simulation"** is delivered in tiers:
  - Tier 0: numeric checks, ≤ 1 s;
  - Tier 1: G5 and the single-arm scene collision check for the changed window, ≤ 5 s;
  - Tier 2: 3D playback on demand, and the dual-arm clearance check in the background.
- **"Simulation"** here is Lite's kinematic 3D simulation, not the controller's SIM run mode. That mode needs an upload and shows no motion or collisions (F3).
- **The simulation shows the commanded path.** The real arm departs from it: p50 ≈ 1.3 mm but p95 ≈ 10–23 mm at 1× on this channel (`FIND:6699-6716`). Collision verdicts must therefore report the clearance left, not only contact or no contact.
- **Honest expectation:** on the upload channel, smoothing does not improve executed accuracy (median 1.04×, `GIC:77-99`). It removes ripple and adds a few percent of safe speed. Heavy smoothing cuts corners.

### Findings

| # | Finding | Evidence |
|---|---|---|
| 1 | The prototype's smoothing is one global "Mean smoothing window" (101), applied only after "Confirm & Generate". It has no preview, no deviation metric and no simulation, and it says so. | `H:382-383`, `H:1703-1709`, `H:436-439` |
| 2 | Pad deviation by preset (zero-phase order-2 filter at the 5 ms clock, FK at `L_glove_frame_2`). Verified on FILE_08 and FILE_10.1:<br>• 5 Hz: 0.91–1.53 mm (p95 0.33–0.42)<br>• 3 Hz: 1.86–2.76 mm<br>• 2 Hz: 3.32–5.51 mm<br>• 1.5 Hz: 5.00–8.92 mm<br>• prototype boxcar: 9.05–10.39 mm (10.60 mm taught→new) | Appendix V; `CC` files |
| 3 | Smoothing barely raises the ratings-limited speed-up: +3–6 % at 5 Hz, +7–10 % at 2 Hz. Light smoothing removes per-tick ripple (FILE_6: 1.08 → 0.69). Verified. | Appendix V |
| 4 | ALIX evidence: upload median 1.04× (4 of 9 cells improve); CANFD 0.36–0.41×. The 5 Hz filter moves the pad 0.30 mm p95 / 1.11 mm max. | `GIC:77-99`; `TT:880-890` |
| 5 | **ALIX's deviation metric is an exact both-direction search, about 0.2 s per 9.6k lines with a KD-tree.** ALIX documents that windowed searches gave false readings on re-timed tracks (320 mm on a rung that is within 2 mm). A windowed search is valid only when the line count is unchanged. | `TT:607-667`, `TT:670-716`, `TT:621-627` |
| 6 | Reusable Studio simulation pieces:<br>• `applyRobotPose` (server-computed link transforms);<br>• the robot-geometry endpoint;<br>• the DAC timeline with click-to-scrub, Prev/Next and red link highlight;<br>• the shared transport block;<br>• stale-on-edit. | `JS:12423-12437`; `SRV:1918-1998`; `JS:14478-14559`; `JS:8749-8767`; `JS:5389-5430` |
| 7 | The Studio's DAC endpoint is not reusable as-is:<br>• `dt` defaults to 0.05;<br>• it passes no `extra_managers` (commode, walls) and no hand states;<br>• it reads repo-relative plan JSONs;<br>• playback is 1× only and never pauses at a collision;<br>• each sweep takes 12–15 s per pair and rebuilds models (M02 §1.6). | `SRV:4335-4366` (`SRV:4343`); `JS:14561-14581` |
| 8 | **ALIX's dual-arm gate** is the minimum arm-to-arm separation over the start-lag/clock window (`cross_arm_separation`):<br>• ≥ 25 mm;<br>• at dt 0.02;<br>• with pole and hand seeded.<br>It is not an aligned 0/1 sweep. At 0.05 s links move up to 109 mm between samples. | `DAV:24-61`, `DAV:352-369`, `DAV:580-660`, `DAV:709-731` |
| 9 | `BodyCollisionWorld` covers the full scene: partner arm, bin and covers, UGV, walls, commode by fixture state. It takes ~2 ms per pose (p95 4.5 ms). Shared FCL managers crashed (SIGSEGV), so they are serialised by an RLock. | `BC:104-108`, `BC:146-172`, `BC:401-464`; M05 §4.5 |
| 10 | **`BodyCollisionWorld` coupling:**<br>• needs the repo-root marker, venue YAMLs under `ros/…/config/scene/`, the fixture `scene.yaml` and STLs, `contact_links.yaml` and `alix_taskgen.scene_pose`/`fixture_parts`;<br>• takes a task-config-shaped dict;<br>• always uses the bin LOWERED (no flag);<br>• **if the fixture scene is missing it silently builds a world with no commode.** | `BC:114-119`, `BC:177-204`, `BC:206-249`, `BC:417-437` (`BC:433-434`), `BC:499-560`, `BC:629`; M05 §4.7, §7 item 5 |
| 11 | **Contact allowance:** commode contact is excused only for the contact frame's link set (`contact_links.yaml`). On a real track, the thumb against the lid was flagged (9/188 samples). With the default 800 mm venue, false wall hits appeared. | `BC:595-622`; M05 §4.6 |
| 12 | Collision sampling must use dt 0.02 s: it is a correctness parameter. | `DAV:352-356` |
| 13 | Tier-0 costs on 12,429 lines (analyst-measured with scipy): filter 1.4 ms; FK of two tracks 45 ms; rate gates ~2 ms. The exact deviation is ~0.2 s per ALIX. The G5 ceiling takes ~6 s per 19k lines in ALIX, so it belongs in Tier 1. | analyst Appendix A; `TT:682`, `TT:1828` |

### Borrow verdict

**ALIX Studio: Partial — borrow some, build the rest.**
- Borrow the simulation half:
  - the 3D viewer and robot geometry;
  - `applyRobotPose`;
  - DAC scrub, Prev/Next and link highlight, re-themed;
  - the transport block;
  - stale-on-edit;
  - gate chips;
  - `BodyCollisionWorld` as a library, with adaptation (explicit asset bundle, synthetic config, bin pose, fail-loud fixture).
- Build:
  - the smoothness control;
  - `.txt` input;
  - the exact deviation display;
  - the G5 display;
  - the tier orchestrator;
  - the windowed scene re-check;
  - stop-at-problem;
  - playback speed;
  - scene completeness (commode, walls, fingers);
  - the dual-arm separation gate.

**Other sources:**

| source | verdict | what to take |
|---|---|---|
| ALIX `TT` (`smooth_taught`, `deviation_mm`, `pad_departure_joint_mm`, `departure_lambda_cap`) | Borrow with adaptation (port) | Presets engine; exact deviation (both directions); G5 prediction and ceiling. |
| `alix_workspace` (`urdf_kinematics`, `dual_arm_plan_collision`) | Borrow as-is (FK) / with adaptation (sweeps) | `UrdfModel.fk_batch` with the pole as q[0] (`UK:231-259`; needs scipy).<br>`cross_arm_separation` for the dual-arm gate.<br>`compute_dual_arm_plan_collisions(extra_managers=…, dt=0.02)` for playback frames (`DAPC:586-764`). |
| ALIX `tools/dual_arm_verdicts.py` | Borrow with adaptation | The `.txt` → timeline → plan shim with pole/hand seeding and the swapped-side guard (`DAV:580-692`), and the windowed sweep (`DAV:709-731`). |
| Trajectory HTML | Reference only (smoothing); borrow with adaptation (compare view, cursor) | Before/after compare (`H:1260-1322`) and cursor shapes (`H:1134-1142`, `H:1286-1288`). |

**Check of the input note:** R11 has no borrow note. ALIX Studio covers only the simulation side. Smoothing comes from ALIX's taught-track tools.

### Borrow items

| item | from | adaptation |
|---|---|---|
| Zero-phase de-jitter filter | `TT:894-912` | Presets Off / Light 5 Hz / Medium 3 Hz / Strong 2 Hz; Advanced 1–8 Hz. Applied at the record clock before stops and speed. |
| Exact pad deviation, both directions | `TT:607-667`, `TT:670-716` | KD-tree search at the region's contact frame, with the region's pole height. Report the max of both directions. |
| G5 prediction | `TT:1107-1165`, `TT:1817-1852`, `EM` | Predicted executed departure per line; the binding line, and which joint owns it (`TT:1083-1104`). |
| FK for pad positions | `UK:231-259` | Chain `base_link` → `{S}_glove_frame_2`, with the pole as q[0]. |
| 3D viewer and pose application | `JS:12423-12437`, `SRV:1918-1998` | 0.25×–4× playback with scrubbing, and an optional ghost of the original pad trace. Client-side FK avoids ~16 KB/frame (M02 §2.3). |
| Collision navigation and highlight | `JS:14501-14559` | Driven from Lite's collision list. Token colours. Stop at the first problem (R5). |
| Per-pose scene collision | `BC:401-464`, `BC:499+` | One pre-warmed worker process (build 2.6 s + fixture 2–4 s). Synthetic config declaring the contact surface and frame. Bin pose via `partner_state` joints or a new exclude predicate (M05 §4.7). **Refuse to run if the fixture scene or meshes are missing.** |
| Dual-arm gate | `DAV:24-61`, `DAV:580-731`; `DAPC` `cross_arm_separation` | dt 0.02, seeded pole and hand, windowed minimum ≥ 25 mm. Cache by the SHA of both files plus the scene. |
| Stale-on-edit | `JS:5371-5430` (pattern) | A smoothness change marks Simulated, Real-tested and Upload-ready out of date. |

### Build items

| item | approach |
|---|---|
| Smoothness control | Preset selector with one-line explanations. Inheritance as in R2. Shows the measured deviation next to the G4 limit. |
| Tier orchestrator | Tier 0 synchronous (≤ 1 s). Tier 1 automatic (≤ 5 s): G5 plus the scene check of the changed window. Tier 2 in the background with progress. Cached by content hash; one verdict panel. |
| Windowed scene re-check | Diff the old and new outputs → changed line ranges, padded by 0.5 s → sample at 0.02 s → per-pose filter for this arm against fixture, walls, bin and UGV, with the partner at its idle pose. Arm-against-arm is left to Tier 2. |
| Before/after overlay and strips | Curves overlaid; a deviation strip and a G5 strip against time with their limits; collision bands on the same axis. |
| Verdict panel | Sentences with Show buttons, for example "Path moved 1.2 mm (limit 10 mm) ✓", "Predicted on-robot departure 14 mm (limit 20 mm) ✓", "Changed part clear of commode and walls ✓", "Dual-arm check running… 40 %". Show the minimum clearance left, not only pass/fail. |

### Recommendation

Offer four presets based on the measured ladder:

| preset | cutoff | pad deviation |
|---|---|---|
| Light (default) | 5 Hz | ≈1–1.5 mm |
| Medium | 3 Hz | ≈2–3 mm |
| Strong | 2 Hz | ≈3–6 mm |
| Off | — | — |

All are zero-phase at the record clock, applied before stops and speed. Show the file's exact deviation and its predicted on-robot departure against their limits. Validate in tiers:
- numeric checks in ≤ 1 s;
- G5 and a scene check of the changed part in ≤ 5 s;
- the dual-arm gate in the background;
- 3D on demand.

Use ALIX's dual-arm gate, not an aligned 0/1 sweep. The collision world must be complete, seeded and calibrated, and it must fail loudly when an asset is missing. Tell operators plainly that smoothing removes shake rather than improving accuracy.

### Approach steps

1. Presets inside the R2 processor (`smooth_taught` at 5 ms).
2. **Tier 0 (synchronous):**
   - recompute;
   - FK the raw and the result;
   - exact deviation in both directions (KD-tree);
   - G2, G3 and limit checks;
   - return decimated curves, the deviation strip and a verdict.
3. **Tier 1 (≤ 5 s, automatic):**
   - G5 prediction and ceiling refresh;
   - changed ranges ±0.5 s at dt 0.02 through the pre-warmed `BodyCollisionWorld` worker, using:
     - a synthetic config with `ik_frame` = the region's contact frame and `commode_fixture_type` = the region's state;
     - the region's pole;
     - the bin pose from R10;
     - the venue from Lite calibration.
4. **Tier 2 (background):**
   - the DAV shim with pole/hand seeding → `cross_arm_separation` windowed minimum ≥ 25 mm at dt 0.02;
   - optionally `compute_dual_arm_plan_collisions(dt=0.02, extra_managers={commode, walls})` for playback frames.
5. Present results as red bands on the plots, red links in 3D and verdict sentences. 3D playback runs at 0.25×–4×, stops at the first problem and can show a ghost of the original.
6. On any change, mark the region stale.
7. **Tests:**
   - preset deviations on FILE_08 and FILE_10.1;
   - the deviation equals ALIX `deviation_mm`;
   - tier timings;
   - windowed re-check = full sweep;
   - missing fixture → error;
   - pad contact not flagged, thumb contact flagged;
   - stale-state transitions.

### Acceptance criteria

1. Changing smoothness on a 12,000-line file updates the overlay, the deviation figure and the speed/step/limit verdict within 1 s.
2. The displayed deviation equals ALIX `deviation_mm` (exact, both directions) within 0.1 mm on FILE_08 and FILE_10.1 for every preset.
3. Light moves the pad ≤ 1.6 mm on FILE_01.1, FILE_08 and FILE_10.1 (measured 0.91–1.53 mm).
4. A level over the G4 limit or the executed budget shows red and cannot be saved as sendable. The message names the limit and the lighter preset that fits.
5. Within 5 s of a change affecting ≤ 10 s of motion, a scene collision verdict for the changed window is shown, with the minimum clearance.
6. Contact by the contact frame's allowed links with the commode is not reported as a collision. Contact by any other link is.
7. If the fixture scene, meshes or venue are missing, the check reports an error. It never reports "clear".
8. The smoothed region plays in 3D at 0.25×–4× with scrubbing. Clicking a collision mark poses the robot there with the colliding links in red.
9. After a smoothness change, "Simulated" and "Real-tested" read "out of date" until re-run.
10. Smoothing never changes the first or last point or the line count.

### UX notes

- One sentence per preset. Light: "removes hand shake; the path moves about 1 mm". Strong: "rounds corners; may miss edges".
- Show "Path moved 1.2 mm (limit 10 mm) ✓" large, with a Before/After toggle.
- Never suggest that more smoothing is safer or more accurate.
- Visual result first. A "Checking…" chip then turns into the verdict, and editing is never blocked while checks run.

### Safety notes

- Smoothing changes the path. It can lose contact (missed spots) or create collisions near commode edges, so collisions are re-checked after every change.
- The collision world must include the commode, walls, bin and fingers, and must seed the pole and hand. A missing asset must stop the check, because `BodyCollisionWorld` otherwise drops the commode silently (`BC:433-434`).
- A kinematic "clear" is not proof:
  - the executed path departs p95 10–23 mm at 1× (`FIND:6699-6716`);
  - the G5 prediction is p95 with a worst case ≈ 2× (`TCTR:93-99`).

  Report the clearance left, and keep the slow real test before upload.

### Dependencies, risks, open questions

**Depends on:**
- R2 (processor, G5 port);
- R3 and R5 (viewer, collision worker; C3);
- R10 (dual-arm view, bin toggle);
- R7 (pole and hand per region);
- R1 (contact frame, fixture state and executed budget per region);
- FK service with a pinned venv;
- Lite cell calibration (venue width, UGV standoff, fixture state).

**Risks:**
- The collision scene is not calibrated for the Lite cell.
- FCL is single-threaded, so one worker is a bottleneck.
- Dual-arm sweeps take tens of seconds, so "immediate" must be honest about the tiers.
- Operators may expect smoothing to fix accuracy.

**Open questions:**
- Who sets each region's executed budget (5/10/20 mm)?
- Is a single-arm-against-scene check enough for "immediate", with dual-arm in the background?
- What are the venue width and UGV standoff of the Lite cell?
- Is the contact frame always `{S}_glove_frame_2`?
- Is the thumb allowed to touch the lid?

**Priority: Should. Effort: L (≈8–10 person-days)** of R11-specific work: presets UI, tier orchestrator, windowed re-check client, overlays and verdict panel. Becomes M (≈6–7 days) if C3's collision worker already exposes a windowed scene-check API. The hard gates are Must under INT-TRAJ.

---

## R12 — Spike, limit and sudden-motion analysis with one-click, undoable fixes

**Original (`REQ:30`):** "12. Plot Spike Analysis: The software should identify high and low spikes in joint trajectory plots and provide suggestions to reduce sudden movements and improve trajectory smoothness."

### Revised requirement

After every change, Lite scans the output bytes at the 5.0 ms tick. It marks on the joint plots, and lists in plain language, every sudden or unsafe place:

- **(a) Too fast.**
  - Must-fix: a joint over its rating (180/180/225/225/225/225/225 °/s) after the 20 Hz band limit — ALIX G2.
  - Warning: over a rating only on single raw ticks.
- **(b) Step fault.** A step > 10°. This blocks upload.
- **(c) Isolated jumps** — "high" or "low" spikes, marked ▲/▼.
- **(d) Sudden speed changes.**
  - Advisory: band-limited acceleration above 900 °/s², or above 600 °/s² sustained.
  - Should-fix: any one-tick speed step at a sub-region boundary or at the file ends.
- **(e) Near a joint limit.**
  - Must-fix: ALIX G1 predicts the arm will come within 0.5° of, or cross, a limit.
  - Warning: the commanded margin is inside 5°.
- **(f) Hesitations.** Mid-stroke slow-downs not removed as stops.

Each finding shows where it is (time, joint, value), why it matters and the suggested fixes. Applying a fix is one click; it is a labelled, undoable change in the region's history (R2), and the checks run again.

### Clarifications

- **"High and low spikes"** means sharp upward and downward excursions of a joint curve or its speed, marked ▲/▼. "Low spikes" could mean hesitations, which are covered by (f). The requester to confirm.
- **Thresholds come from the arm:**
  - ratings (`K:37`);
  - step 0x4000 at 10° (`TT:127-128`);
  - limits (`K:44-46`);
  - G1 with the live `EM` constants: floor 0.85°, τ_limit 0.139 s, eps 0.5° (`TT:466-487`, `TT:1352-1365`).
  - The legacy rule (5° allowance, 10 °/s cap, 2° floor; `TT:162-173`) was superseded on 2026-09-15 (`TT:195`, `TT:850-853`). Both sets are fitted or provisional in ALIX.
  - Acceleration is advisory on this channel (`CT:278-290`; `TCTR:24-31`).
- **Instruments:**
  - rates and accelerations use the 20 Hz zero-phase band limit, ALIX's house rule (`PTR:76-85`, `PTR:92-114`);
  - steps and isolated jumps use raw per-tick values;
  - ALIX's G2 refuses on band-limited ticks only; raw ticks are recorded (`TT:594-599`).
- **Suggestions are deterministic rules.** Each maps to a recipe operation. No AI is involved.

### Findings

| # | Finding | Evidence |
|---|---|---|
| 1 | Prototype analysis:<br>• per-sample step norm and \|Δstep\|, averaged over 5–10 equal chunks with relative quartile rules;<br>• the first and last chunks are always 75 %;<br>• no per-joint rates, no thresholds, no markers. | `H:944-1050` |
| 2 | ALIX Studio has no joint plots and no taught-track analysis. `/api/analyze-plan` has no UI caller, checks only a plan's `velocities` field, and `acc_violations` is never filled. | `JS:5368`, `JS:14483`; `SRV:4272`, `SRV:6013`; `PA:126-136` |
| 3 | ALIX instruments exist:<br>• `rate_report` (raw and band-limited, ticks over, longest run);<br>• `position_report`, `step_report`;<br>• `gate_report` with plain-text reasons;<br>• `rate_fit_track`;<br>• `tick_step_check`;<br>• the acceleration advisory;<br>• `slowness_census` (time spent slow, and still runs that remain);<br>• `_departure_attribution` (which joint causes a departure);<br>• `taught_tracks.py screen` (batch triage, writes nothing). | `PTR:92-136`; `TT:420-604`; `CT:1675-1714`, `CT:1743-1750`, `CT:278-336`; `TT:958-991`, `TT:1083-1104`, `TT:4149-4162` |
| 4 | **Corrected statistics** (19 files, 5 ms tick, verified).<br>Raw ticks over a rating: 14 files.<br>Band-limited (G2), 6 files:<br>• `R_P06P07`: 102 ticks, J6 312 °/s (1.39×), longest 285 ms;<br>• `R_P10P12`: 29 ticks, J4 243 / J7 229 °/s;<br>• lid_top 28/30/35/40 %: 200 / 175 / 117 / 70.<br>Raw-only V6 files, raw peak (ticks): `R2_LIDTOP` J6 319 °/s (60); `R_P01P03` J4 267 (26); `L_P01P02` J6 283 (23); `R15_BUPFUL` J6 263 (19); `L_P06P07`, `M3_ASOPN02`, `R_SPILLAGE` (6 each); `L_P10P12` (1). | Appendix V |
| 5 | Raw-only ticks still matter: the controller flags each one as 4105, latched for the run, and one ALIX program ended at 58 % with it latched. On the other side, TCTR records runs that went on with 4105. | `CT:211-218`; `TCTR:31-36` |
| 6 | Isolated jump in `R_P06P07` at lines 5058→5059: J3 1.049° between 0.271° and 0.276°; J5 1.22° between 0.82° and 0.87° (not isolated). M10 lists others: `L_P10P12` J6 1.219°, `R_P10P12` J4 1.161°. | Appendix V; M10 §5 item 7 |
| 7 | **Near-limit case `R10_HNGR`** (verified):<br>• J4 maximum 133.466° at line 952, margin 1.53°;<br>• band-limited J4 rate on the way into the peak ≈ 21.5 °/s;<br>• G1 predicts an overshoot ≈ 3.0° > 1.53° margin → must fix;<br>• slowing the arrival to ≤ 7.4 °/s would satisfy G1; under the legacy 2° floor it would be "re-teach";<br>• J4 has run past 135° by 5.6–5.9° on a cold arm, even on a slow track. | `TT:1352-1365`; `EM`; `FIND:6721-6737` |
| 8 | Editing creates spikes:<br>• prototype end snaps of 966 and 1005 millidegrees;<br>• boundary jumps of 440–880 °/s² (verified);<br>• V6 bridges start and end at rest while segment ends move (joins with one-tick velocity steps). | `H:1513-1514`, `H:1568-1616`; M10 §3 |
| 9 | ALIX remedies:<br>• over-rating → slow only that stretch;<br>• ripple → 5 Hz de-jitter;<br>• step > 10° → refuse;<br>• boundary → ramp λ;<br>• fast arrival at a limit → slow the arrival (G1);<br>• geometry → re-teach.<br>ALIX warns not to promise accuracy from slowing down. | `CT:1675-1714`; `TT:880-912`; `TT:1377-1437`; M06 §4 |

### Borrow verdict

**ALIX Studio: Not available — build new.** Only presentation parts can be borrowed:
- validation rows with glyph, sentence and "where" (`CSS:1462-1484`);
- gate chips (`CSS:1409-1433`; `renderGates` `JS:15904`, coupled to Studio gates, so re-implemented);
- Prev/Next and click-to-scrub (`JS:14501-14559`);
- stale-on-edit (`JS:5389-5430`).

`/api/analyze-plan` is reference only.

**Other sources:**

| source | verdict | what to take |
|---|---|---|
| ALIX `PTR` / `TT` / `CT` | Borrow with adaptation (port) | `rate_report`, `step_report`, `position_report`; G1 via `model_predictions`/`model_caps` with `EM`; `gate_report` reason wording; `rate_fit_track`/`capped_millideg`; `tick_step_check`; the acceleration advisory; `slowness_census`; `_departure_attribution`. Run `taught_tracks.py screen` now, in the ALIX venv, for a no-write triage of legacy files. |
| Trajectory HTML | Reference only (analysis); borrow with adaptation (markers) | `analyzeMotionRegions` measures nothing physical. Its cursor and shapes (`H:1134-1142`, `H:1286-1288`) are reused for markers. |

**Check of the input note:** R12 has no borrow note. Neither the Studio nor the prototype meets it. ALIX's taught-track tools hold the detectors and fixes.

### Borrow items

| item | from | adaptation |
|---|---|---|
| Rate instrument | `PTR:92-114` | Return stretches with times and joints; merge gaps < 50 ms; band-limited count = G2, raw count = warning. |
| Step and position instruments | `PTR:117-136`; `TT:122-128` | Per-line output for markers. |
| G1 limit-arrival prediction | `TT:1258-1376` (`model_predictions`), `TT:1377-1437`, `EM` | Predicted executed margin per extreme; fix = slow the arrival to the cap; geometry → re-teach. |
| Acceleration advisory | `CT:278-336` | Flag above 900 °/s² and above 600 °/s² sustained. Advisory only. |
| One-tick speed-step check | `CT:1743-1750` | Localise to boundaries and the first/last 0.25 s. |
| "Slow only this stretch" fix | `CT:1675-1714`, `TT:767-832` | A local λ ceiling as an R2 recipe operation with a labelled undo entry. |
| Hesitation census | `TT:958-991` (`slowness_census`) | Time below 20/50 mm/s at the pad, plus still runs left after stop removal → D6 findings. |
| "Which joint" attribution | `TT:1083-1104` | Names the joint to blame in G5 and too-fast findings. |
| Reason wording | `TT:584-604` | Rewritten for operators. |
| Presentation | `CSS:1462-1484`, `CSS:1409-1433`, `JS:14501-14559`; `H:1134-1142` | Severity groups, icon + word, Show / Fix / Ignore-with-reason, tokens. |

### Build items

| item | approach |
|---|---|
| Detector suite (server, numpy) | Findings `{id, kind, severity, joints, t_start, t_end, peak, measure, threshold, line_range, suggestions[]}` from D1–D6 on the output bytes at 5.0 ms. |
| Suggestion rule table | Map kind × context to ordered, previewable recipe operations. |
| One-click fix | Applied as an R2 operation with a label. Re-scan afterwards. Undo restores the finding. |
| Findings panel and overlay | Groups Must fix / Should fix / Advisory with a count badge; Show / Fix / Ignore-with-reason; an optional speed-utilisation plot with a 100 % line. |
| Regression corpus | The 15 V6 files, 4 lid_top files and the raw commode files, with golden findings (see the AC). |

### Detectors

| detector | rule | severity |
|---|---|---|
| D1 too fast | Band-limited rate > rating → stretches. | Must fix |
| D1 raw ticks | Raw-only ticks over a rating. | Warning (Should fix) |
| D2 step fault | Step > 10°. | Must fix; blocks upload |
| D3 isolated jump | Proposed thresholds, to tune on the corpus: step > 0.5° and > 3× both neighbours, or > 4× the local median (±50 lines) and > 0.3°. The sign gives ▲ or ▼. | Should fix |
| D4 sudden speed change | Band-limited acceleration > 900 °/s² or > 600 °/s² sustained > 50 ms. | Advisory |
| D4 at a boundary or file end | One-tick speed step > max(10× local median, 100 °/s). | Should fix |
| D5 near limit (G1) | Predicted executed margin < 0.5° (commanded margin − max(0.85°, 0.139 s × band-limited arrival rate over 0.3 s)). | Must fix. Fix: slow the arrival to (margin − 0.5°) ÷ 0.139 s. Geometry (commanded margin < 1.35°): re-teach. |
| D5 inside allowance | Commanded margin < 5°. | Warning |
| D6 hesitation | 0.1–1.5 s with joint-space speed < 20 % of the surrounding 2 s median and not removed as a stop, plus `slowness_census`. | Info |

### Suggestion rules

| finding | context | suggested fix |
|---|---|---|
| D1 | Inside a sped-up sub-region | "Lower speed from a× to b×", with b = a × 0.97 / utilisation, or "Let Lite slow only this stretch". |
| D1 | Recording itself too fast (λ ≤ 1) | "Slow this stretch" or "re-teach slower". |
| D1 raw-only | — | "Light smoothing here". |
| D2 | — | "Re-teach this part", or undo the edit that created it. |
| D3 | — | "Smooth this spot" (local Light ±0.1 s) or "Replace this point". |
| D4 | At a boundary | "Blend boundary". |
| D4 | At file ends | "Ease in/out". |
| D5 | G1 rate case | "Slow the approach to ≤ X °/s here". |
| D5 | Geometry case | "Re-teach further from the limit". |
| D6 | — | "Remove this stop" or "Speed up this slow part". |

Previews are dry-run recipes (< 0.5 s). Gate integration: D2, D1 band-limited and D5 G1 block real runs and uploads. Everything else warns.

### Recommendation

Build a server-side detector suite on the output bytes at 5.0 ms:
- ALIX's G2 (band-limited) decides "too fast";
- raw ticks warn;
- ALIX's G1 execution-model rule decides "near a limit";
- steps and spikes use raw values.

Pair it with a deterministic rule table that turns each finding into one or two previewable recipe fixes. Show ▲/▼ markers, shaded bands and a severity-grouped list linked to the 3D view. Every fix is an R2 operation, so it is labelled and undoable.

### Acceptance criteria

1. On `R_P06P07.txt` the scan reports a Must-fix too-fast stretch on J6: band-limited peak ≈ 312 °/s (1.39×), 102 band-limited ticks, longest 285 ms. The raw figures (491 °/s, 172 ticks) are shown as detail. It also reports an isolated J3 jump at lines 5058–5059.
2. On `R10_HNGR.txt` it reports J4 (1.53° from 135°, arriving at ≈ 21.5 °/s) as Must fix. It offers "slow the approach to ≤ 7.4 °/s" and "re-teach". After the slow-down fix, G1 passes.
3. On `R2_LIDTOP.txt`, `L_P01P02.txt` and the other raw-only files, the too-fast finding is a warning, not Must fix, and Light smoothing removes it.
4. Every finding is drawn on the affected joint's plot at the right time (±1 line). Its list entry gives time (s), joint, value and limit in plain words.
5. Clicking a finding moves the plot cursor and the 3D robot to that time.
6. Applying the suggested fix for a too-fast finding removes it, adds no new Must-fix, and appears as one labelled undo entry. Undo brings the finding back.
7. A file with any step > 10° cannot be sent, and the message says where.
8. A full scan of a 12,000-line file takes < 0.5 s; G1 and G5 (Tier 1) take < 5 s.
9. With the default R2 recipe, FILE_08 and FILE_10.1 show no endpoint or boundary findings. The prototype-processed versions do.

### UX notes

- Every severity has an icon plus a word, never colour alone (ISA-101; M01 §1.6).
- Plain sentences, for example "Joint 6 moves too fast here (312 °/s; the arm's limit is 225 °/s) for 0.3 s".
- One primary Fix button with a preview, plus "Show me". Merge adjacent findings.
- A short "Why does this matter?" per kind.
- Count badge on the tab. No blanket "Fix all" until the fixes are proven on hardware.

### Safety notes

- Must-fix findings block real runs and uploads. Fixes are never applied silently.
- Slow-down fixes keep the path but change timing, so in a dual-arm step the clearance must be re-checked.
- Smoothing fixes change the path, so collisions must be re-checked.
- Acceleration is advisory on upload (`CT:278-290`; `TCTR:31-36`). On CANFD, high commanded acceleration tripped 4109 (M06 §2.3).
- Near-limit thresholds come from a fitted model. J4 has crossed 135° on hardware, so keep the 5° warning even when G1 passes.

### Dependencies, risks, open questions

**Depends on:**
- R2 (recipe operations and history);
- INT-TRAJ (plots, markers);
- R11 (tiers);
- R5 (collision marks on the same time axis);
- R10 (dual-arm re-check after slow-downs);
- `EM` constants;
- pinned scipy.

**Risks:**
- Thresholds are fitted or provisional in ALIX itself.
- Legacy files will show many warnings (8 raw-only V6 files), risking alarm fatigue. Group and explain them.
- "Low spike" may be misread.
- Fixes may interact, so always re-scan.

**Open questions:**
- What does the requester mean by "low spikes"?
- Should raw-only over-rating ticks block? Recommended: no, warn.
- Should operators see the acceleration advisory?
- Is "Fix all safe issues" wanted?

**Priority: Should**; the hard gates (D1 band-limited, D2, D5 G1) are Must and are repeated in INT-TRAJ. **Effort: L (≈10–12 person-days):** detectors including the G1 port 4 days, rules and fixes 3 days, UI 2–3 days, corpus tests 1–2 days.

---

## INT-TRAJ — Integrate the trajectory-optimisation prototype as Lite's Trajectory Optimisation tab

**Original (derived requirement):**
- `REQ:3`: "It is a light weight version of the ALix Studio (/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen) that uses the drag and teach generated plans, that are optimized (smoothed and speed increased) since they are recorded at slow speed and with a number of stops, …"
- `REQ:10`: "- Trajectory Optimization Tab (/home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html)"
- `REQ:18`: "- check the /home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html"

### Revised requirement

Lite shall contain a Trajectory Optimisation tab, built from `Alix_Studio_Lite_Trajectory_V2.html`, that:
- (a) opens a taught region from the region dashboard, showing unit ID, arm, region, fixture state, pole/hand and executed budget, instead of a loose file;
- (b) offers **Auto-optimise (recommended)** as the main path, and a guided fine-tune path: Choose region → Remove stops → Smooth → Speed → Check → Compare & save;
- (c) runs the optimisation on the Lite server with ALIX's validated algorithms, with the browser previewing only;
- (d) shows joints in degrees against seconds, with durations from the robot's playback law (0.2322 s + 5.6737 ms × (N−1)), shown with the measured band, and with all speed checks at the 5 ms tick;
- (e) limits every speed-up by the joint ratings, the path-accuracy ceiling (G5) and the joint-limit arrival ceiling (G1);
- (f) shows both arms of a dual-arm step on one time axis;
- (g) saves numbered versions with recipe, checksums and check results into the region store, and hands them to simulation, real-arm test and upload;
- (h) uses ALIX Studio's tokens, components and vendored Plotly 2.35.2, so it works offline.

### Clarifications

- The prototype remains a reference. Lite re-implements it in the Lite web shell, not as an iframe.
- **Identity** follows map 13: `<ARM>-<KIND><NN>[-P<n>|-S<n>]`; raw `<ID>__RAW__<ts>.txt`; optimised `<ID>__OPT_v<k>.txt`.
- **Raw recordings** today come from the control app's drag-teach save, named by side and time only (`UI53:3840-3850`). R1 must attach the ID when saving.
- **Three clocks:**
  - record clock 5.0 ms, for filters;
  - gate clock 5.0 ms, for rate and step checks;
  - player law and band, for durations (`TRK:1558-1570`; `TT:116-121`, `TT:278-281`).
- **Use the law's slope (5.6737 ms/line) for the x axis**, so the axis end agrees with the displayed duration within the 0.23 s ramp.
- The control app's overall/transit speed % does not change track duration. It scales only the transit to the first point (`FIND:5855-5857`). Lite shows the transit separately.
- The prototype's automatic speed analysis is replaced by the per-sub-region allowed maximum (R2) and by R12 findings.

### Findings

| # | Finding | Evidence |
|---|---|---|
| 1 | The prototype is a single static page: Plotly from a CDN; no server, storage or identity; one file at a time. | `H:7`, `H:349-352`, `H:587-604` |
| 2 | Strengths to keep:<br>• a 6-step stepper with lock/done states and Back/Next notes;<br>• DOM-free pure functions;<br>• a compare view with a synchronised cursor;<br>• the report JSON;<br>• an honesty note. | `H:609-704` (CSS `H:220-299`); `H:1417-1620`; `H:1260-1322`; `H:1732-1755`; `H:436-439` |
| 3 | Values are shown as millidegree counts against row numbers, and durations as N × 5 ms. A 9,813-line output shows 49.1 s, while the law gives 55.9 s (band 54.2–57.8 s). Verified arithmetic. | `H:851-852`, `H:1176-1179`; `TRK:1558-1570`; `TT:116-121` |
| 4 | The analysis is positional and relative, with no gates. Map 07 Appendix B lists 8 UI-state bugs, for example a scale edit that bypasses `invalidateProcessed` (`H:1359-1371`, verified). | `H:994-1037`; M07 App. B |
| 5 | Theme mismatch: `--accent #175cd3`, 8–12 px radius and a dark header, against ALIX `--brand #2F96EE`, `--radius 4px` and `--danger #D93A34` (test-pinned). | `H:9-21` vs `CSS:22-58` |
| 6 | The prototype needs `cdn.plot.ly`. The Studio vendors the same Plotly 2.35.2 and lazy-loads it with `ensurePlotly`. | `H:7`; `JS:5322-5337` |
| 7 | File contract: the prototype writes integer NDJSON with LF, which the runner accepts. Its parser accepts floats and skips bad lines, while runner v14 and `alix_dispatch` refuse the whole file. | `H:1618-1620`, `H:803-829`; `RUN14:76-89`; `TRK:66-82` |
| 8 | Shipped files: 6 of 19 fail ALIX's enforced band-limited rate gate, and 14 of 19 have raw-tick over-ratings. `R10_HNGR` fails G1. Hard gates are needed at integration. | Appendix V |
| 9 | Raw sources for the V6 regions are not in the workspace. | M10 §5 item 5 |
| 10 | The Studio has no trajectory processing. Its theme, shell components, viewer and DAC are reusable. | `SRV` grep; M01, M02 |
| 11 | ALIX's batch triage `taught_tracks.py screen` already gives per-file verdicts with reasons and writes nothing. Run in the ALIX venv, it can produce the legacy-file rollout report now. | `TT:4149-4162` |

### Borrow verdict

**ALIX Studio: Partial — borrow some, build the rest.**
- Borrow as-is:
  - tokens (`CSS:1-129`, `CSS:134-189`, `CSS:1714-1724`), fixing the drift listed in M01 §1.7;
  - vendored Plotly with `ensurePlotly` (`JS:5322-5337`);
  - status, toast and modal (`JS:5431-5467`, `JS:13786-13832`).
- Borrow with adaptation: gate chips, validation rows, docks, dialogs, and the 3D viewer/DAC for the Check step.
- Everything trajectory-specific comes from the prototype and ALIX's taught-track tools.

**Other sources:**

| source | verdict | what to take |
|---|---|---|
| Trajectory HTML | Borrow with adaptation | **Take:** the stepper, pure functions (preview and parity reference), compare view, report schema, box-select, cursor.<br>**Replace:** theme, units, analysis, algorithms.<br>**Fix:** the 8 UI-state bugs. |
| ALIX tools (`TT`, `CT`, exp01) | Borrow with adaptation (port) | Algorithms, ceilings and gates (see R2, R12). |
| `alix_dispatch/track.py` | Borrow as-is | `parse_track` (`TRK:66-82`) and `expected_replay_s` (`TRK:1558-1570`). |
| `alix_workspace` | Borrow as-is | FK (`UK:231-259`). |
| Upload-channel control app (UI v53 + runner v14, high level) | Borrow as-is (file contract only) | Output = `read_track` contract. Input = its drag-teach recordings. No control-app issue blocks this cluster. |

**Check of the input note:**
- Accurate that the prototype is the right base for the tab's flow and pure functions.
- Inaccurate if read as production-ready: its algorithms and analysis need replacing (measured defects, no gates), and it lacks identity, persistence, validation and simulation.

### Borrow items

| item | from | adaptation |
|---|---|---|
| Guided stepper | `H:609-704`, `H:220-299` | Auto-optimise first. Fine-tune steps re-ordered. Plain notes. ALIX tokens. |
| Compare view with synchronised cursor | `H:1260-1322`, `H:1242-1258` | Path-linked axis (raw-line index), linked zoom, external-file compare allowed before generating, law/band durations. |
| Report schema | `H:1732-1755` | Report v2 sidecar:<br>• `unit_id`, arm;<br>• raw and output SHA-256;<br>• recipe;<br>• tool and `EM` versions;<br>• G1–G5 results;<br>• durations (law and band);<br>• links to simulation and real-test records. |
| Theme tokens and components | `CSS:1-129`, `CSS:134-189`, `CSS:1714-1724`; `JS:5431-5467`, `JS:13786-13832`; `CSS:1409-1505` | Map `--accent` → `--brand`, `--line` → `--border-strong`, and the state colours to ALIX. Vendor Inter and JetBrains Mono (not vendored in the Studio, M01 §1.7). |
| Vendored Plotly | `JS:5322-5337` | Served from `/vendor/`; no CDN. |
| Strict parser and player law | `TRK:66-82`, `TRK:1558-1570` | Used by import and by every duration display. |

### Build items

| item | approach |
|---|---|
| Tab module and server routes | `alix_studio_lite/web/trajectory/`; `/api/units/{id}`, `…/recipe`, `…/checks`, `…/ceilings`, `…/preview`, `…/versions`. |
| Region-store binding | Step 1 lists units with raw/optimised status. "Import file" (advanced) requires a unit ID and arm. |
| Units, clocks, durations | y in degrees; x in seconds at 5.6737 ms/line. Duration = `expected_replay_s` plus the band. Taught duration (N × 5 ms) and transit shown separately. |
| Speed help | Per sub-region, the allowed maximum = the lowest of the ratings ceiling (0.97 / band-limited utilisation), the G5 ceiling and the G1 caps. Default suggestion = min(allowed, 2.0×), with a note that faster means a larger on-robot departure (`FIND:6706-6716`; `TCTR:210-214`). |
| Dual-arm lane | Show the partner unit's current version on the same axis, with the duration difference. Warn when edits change the alignment (R10 owns synchronisation). |
| Gated hand-offs | Simulate / Test on robot / Upload are enabled only for the latest saved version, with current, passing checks. |
| Legacy triage | Import V6 and lid_top as `legacy_processed`. Show G1–G5 findings. Offer "fit to ratings". Optionally attach ALIX `screen` output. |
| Tests | The 8 UI-state bugs (M07 App. B); theme guard tests (subset of M01 §1.8); an offline load test; Playwright flows. |

### Recommendation

Port the prototype's guided flow into the Lite shell, re-skinned with ALIX tokens and served with vendored Plotly. Lead with Auto-optimise. Make the server's recipe processor and shared checks (G1–G5) the only source of saved files. Bind the tab to region identity and both arms. Show durations by the robot's playback law. Gate every hand-off on a current, passing version. Keep the prototype's honest disclaimer, in plain words.

### Approach steps

1. Create `alix_studio_lite/web/trajectory/`. Port the stepper and map the tokens. Vendor the fonts. Drop the dark header.
2. Replace file upload with "Choose region" from R1. Keep "Import file" as an advanced path that requires an ID and arm.
3. Implement the routes: units, recipe, checks, ceilings, preview, versions.
4. Axes and durations: degrees against seconds at 5.6737 ms/line; the `expected_replay_s` band; taught duration and transit separate.
5. Auto-optimise button plus speed help with the binding ceiling named.
6. Compare view: path-linked, external compare allowed, metrics (law/band duration, time saved, G4, predicted G5, check summary).
7. Dual-arm lane for paired steps.
8. Report v2 and output: integer millidegrees, CRLF, validated with the `parse_track` rules, saved to the region store.
9. Hand-off buttons gated on the latest version and current checks.
10. `ensurePlotly` from `/vendor/`, with an offline test.
11. Regression tests for the 8 bugs, theme guards and Playwright flows.

### Acceptance criteria

1. With the network disabled, the tab loads and plots, with no CDN requests.
2. Opening a unit from the dashboard shows its ID, arm, region name, fixture state, pole/hand and executed budget, with no file dialog.
3. Axes are seconds and degrees. A 9,813-line output shows about 55.9 s (range 54.2–57.8 s), not 49.1 s.
4. A version can be saved as sendable only if G2, G3, commanded limits, G1 ≥ 0.5°, G4 ≤ 10 mm and G5 ≤ the region's executed budget all pass. Otherwise only "Save draft" is possible, and the reason is shown.
5. Saved versions appear as `<ID>__OPT_v<k>.txt` plus a sidecar. Re-running the stored recipe on the raw file reproduces the output exactly.
6. Saved files pass runner v14 `read_track` and `alix_dispatch.parse_track` unchanged.
7. Auto-optimise on FILE_08 produces a sendable version in one click, showing duration, time saved and which ceiling binds.
8. The tab's CSS has no colour literals outside token definitions (guard test). Plots are legible in light and dark.
9. A dual-arm step shows both arms on one time axis, with each arm's duration and the difference.
10. Regression tests cover the 8 prototype UI-state bugs.

### UX notes

- Lead with "Auto-optimise (recommended)". Fine-tune steps use plain names, and "Next" is disabled with the reason shown.
- Keep the honesty note: "These checks look at the arm's motion only. Run the simulation and a slow real test before uploading."
- Drafts save automatically. Primary actions use 44 px buttons.
- No jargon: no "counts", "rows", "Section B1" or "JSONL".

### Safety notes

- Hand-offs are gated. A version older than the latest checks is never uploaded or run.
- The controller's transit to the first point is not collision-checked (`RUN14:26-29`). Lite must announce it before a real run.
- Legacy files that fail G2 or G1 (6 + 1 shipped files) are blocked from real runs until fixed. Raw-tick-only files warn.
- The first real run of any new version is slow, and its measured result is recorded against the prediction.

### Dependencies, risks, open questions

**Depends on:**
- the Lite architecture decision: a standalone `alix_studio_lite` package importing `alix_workspace` (M04);
- R1 (dashboard, IDs, raw files, per-region metadata);
- R2, R6, R11 and R12 (tab internals);
- R10, R3, R4 and R5 (simulation, real test, collisions);
- R13 / INT-CTRL (upload hand-off);
- NFR-THEME;
- `EM`.

**Risks:**
- Scope overlap with R2, R11 and R12; keep the internals in those requirements.
- Legacy files fail the new gates; communicate before rollout.
- The playback clock varies ±6 % from run to run (`TRK:45-50`), so show a band.
- Webfonts must be vendored.
- The Python environment must be pinned (scipy/numpy ABI).

**Open questions:**
- Should the prototype stay usable standalone for engineers?
- Is the map 13 ID scheme approved?
- Which checks are hard and which advisory (Newton or the requester)? Recommended: G1–G5, rates, steps and limits hard; acceleration advisory; raw ticks warn.
- Will Lite later stream over CANFD, where smoothing matters more?

**Priority: Must. Effort: L (≈12–15 person-days):** UI port and restyle 4 days, routes and store 3 days, units/clocks/report 2 days, dual-arm lane 2 days, tests 2–3 days. R2, R11 and R12 internals are excluded.

---

## Cross-cutting notes

1. **One canonical processor.** A server-side recipe processor plus the shared G1–G5 checks is the only source of sendable bytes. Merge (R8), offset (R9/R14) and upload (R13) reuse it. The browser only previews.
2. **Three clocks, never one.**
   - Filters run at the 5.0 ms record clock (`TT:109-115`).
   - Rate and step gates run at the 5.0 ms gate clock (`TT:94-102`).
   - Durations use the player law and band (`TRK:1558-1570`; `TT:116-121`).

   The prototype uses one `sampleMs` for all three (`H:731`). Note that `capped_millideg`'s default clock is 5.5545 ms (`TT:767-769`), so pass 0.005.
3. **Hard limits and "the one check".** ALIX's rulings (`TCTR:3-9`, `TCTR:24-31`, `TCTR:218-222`):
   - the ratings (no factor), step ≤ 10° and joint limits are hard;
   - G1 stays enforced because overshoot stops runs;
   - the path check is the predicted **executed** departure (G5, 20 mm default executed budget), alongside G4 (bytes against the recording, 10 mm);
   - acceleration is advisory.

   A rating-only "safe speed" is not safe on this channel (`TCTR:210-214`).
4. **Existing outputs, corrected.**
   - 6 of 19 shipped files fail ALIX's enforced band-limited rate gate: `R_P06P07`, `R_P10P12` and the four lid_top files.
   - 14 of 19 have raw-tick over-ratings, which warn (4105 latch risk, `CT:211-218`).
   - `R10_HNGR` fails G1.
   - Tell the team before rollout, and provide a batch "fit to ratings / slow the approach". ALIX's `screen` (`TT:4149`) can triage today.
5. **Smoothing policy.** Light (5 Hz zero-phase, ≤ 1.5 mm measured) by default. Heavier smoothing buys 3–10 % more speed and no executed accuracy on upload (`GIC:77-99`). Present it as "removes hand shake".
6. **Raw recordings matter.**
   - V6 holds only merged, time-stretched outputs (M10 §5).
   - The control app saves raw recordings by side and time only (`UI53:3840-3850`).
   - Plan re-teaching through R1 with IDs attached. Import legacy files as "already optimised".
7. **Shared history and staleness.** R2 edits and R12 fixes go into one labelled per-region history (≥ 5 levels, persisted). Every change invalidates Simulated, Real-tested and Upload-ready (Studio pattern, `JS:5371-5430`).
8. **Dual-arm coupling.** Speed edits shift timing, and smoothing moves the path. The dual-arm check uses ALIX's windowed minimum separation:
   - ≥ 25 mm;
   - dt 0.02;
   - pole and hand seeded (`DAV:24-61`, `DAV:352-369`).

   The Studio's DAC endpoint defaults (dt 0.05, no commode or walls) must not be reused (`SRV:4335-4366`).
9. **Collision world must fail loudly.** `BodyCollisionWorld` silently omits the commode when its scene file is missing (`BC:433-434`), and always poses the bin LOWERED (`BC:629`). Lite needs an explicit, checked asset bundle and a bin control.
10. **Environment.** System scipy 1.11.4 cannot import against numpy 2.4.6 under `python3 -I` (re-confirmed). `alix_workspace` FK imports scipy (`UK:34`). Ship a pinned venv (numpy, scipy, trimesh, python-fcl), and run collision in one worker process (`BC:146-172`).
11. **Theme.**
    - Use ALIX tokens and a token-driven `plotTheme()`.
    - Do not copy the Studio's `#e6edf3` Plotly literals (1.18:1 on light; `JS:5368`, `JS:14488`).
    - Replace category10 joint colours with `--joint-1..7` tokens; J3 green and J4 red clash with ISA-101.
    - The Studio's dark theme is not wired (M01 §1.3).
12. **Suggested build order.**
    1. INT-TRAJ shell, parser/writer and the G2/G3/limits/G4 checks module.
    2. R2 processor with the G1/G5 ceilings, plus history.
    3. R12 detectors and fixes.
    4. R11 tiers, after C3's collision worker.
    5. R6 layout polish.
13. **Control app (high level only, as the user asked).**
    - Runner v14 uploads, verifies and runs tracks for one arm, and records drag-teach.
    - UI v53 is the Tkinter front end that drives it.
    - C2 touches only the input recordings and the `read_track` output contract.
    - No control-app issue blocks C2.

---

## Verifier's change log (material changes against the analyst draft)

| Req | Change | Why | Evidence |
|---|---|---|---|
| R2, INT-TRAJ, X | The speed ceiling now includes ALIX's path-accuracy (G5) and joint-limit arrival (G1) ceilings, not only the ratings. Gates G1 and G5 were added to "sendable". | ALIX's ruling: the executed deviation is "the one check". Rating-capped uniform 2.0–2.2× runs measured 20–45 mm p95; a scrub needs 0.70×. ALIX's default re-time uses all three ceilings. Map 06 said to add them once Lite has FK. | `TCTR:3-9`, `TCTR:210-222`; `TT:1377-1437`, `TT:1817-1852`, `TT:1855-1985`, `TT:4371`; M06 §2.8 item 4 |
| R12, X | "14 of 19 files over rating" restated: 14 of 19 on raw ticks; **6 of 19** on ALIX's enforced band-limited gate (2 of 15 V6). Acceptance criteria use band-limited figures (`R_P06P07`: 102 ticks, 312 °/s, 1.39×). | ALIX's G2 refuses only on band-limited ticks; the draft's own D1 uses band-limited. Re-measured. | `TT:594-599`; `PTR:92-114`; Appendix V |
| R12 | Raw-only over-rating ticks: Info → Warning. | The controller latches 4105 on raw ticks; one ALIX program ended at 58 % with it. | `CT:211-218` |
| R12 | D5 near-limit rule changed from the legacy 5°/10 °/s/2° rule to ALIX's current G1 execution-model rule. `R10_HNGR` remains Must fix, but its fix is "slow the arrival to ≤ 7.4 °/s or re-teach". | The legacy rule was superseded on 2026-09-15. `R10_HNGR` arrives at 21.5 °/s, giving a predicted 3.0° overshoot against a 1.53° margin. J4 has crossed 135° on hardware. | `TT:195`, `TT:466-487`, `TT:850-853`, `TT:1352-1365`; `EM`; `FIND:6721-6737`; Appendix V |
| R2, R11 | Budget wording fixed. `CC/description.txt` "MAX DEV" is an **executed** TCP budget. G4 (bytes against recording) is 10 mm (5 mm preferred); the executed budget (G5) defaults to 20 mm. | The draft used 5/10 mm as G4 budgets. | `CC/description.txt:19`; `TT:129-138` |
| R11 | Deviation metric: use ALIX's exact both-direction KD-tree search (~0.2 s), not a windowed search. | ALIX documents windowed-search artefacts (320 mm). The reverse direction reads 10.60 mm for the boxcar on FILE_10.1. | `TT:607-716` (`TT:621-627`); Appendix V |
| R11 | Collision feasibility added:<br>• `BodyCollisionWorld` coupling (repo marker, venue/fixture YAML, `contact_links`, task-config dict);<br>• silent omission of the commode when its scene is missing;<br>• bin always LOWERED;<br>• contact allowance;<br>• worker pre-warm. | Hidden coupling and false "clear" risk. | `BC:114-119`, `BC:177-249`, `BC:417-437`, `BC:499-622`, `BC:629`; M05 §4.5–4.7, §7 |
| R11 | Tier 2 now uses ALIX's dual-arm gate (windowed minimum separation ≥ 25 mm, dt 0.02, seeded), not an aligned 0/1 sweep. The Studio DAC endpoint's defaults are flagged. | ALIX gates on the windowed minimum. The Studio endpoint uses dt 0.05 with no commode or walls. | `DAV:24-61`, `DAV:352-369`, `DAV:709-731`; `SRV:4335-4366` |
| R11 | Executed versus commanded path: verdicts report the clearance left. | The upload channel departs p95 10–23 mm at 1×. | `FIND:6699-6716` |
| R2 | Locality acceptance criterion restated: identical source path and λ(u) outside blend and ramp zones; the rating fit tightens locally. | Ramps extend \|Δλ\|/0.01 lines (> 0.25 s for large changes). `rate_fit_track` tightens globally. Re-sampling phase differs downstream. | `TT:767-832`; `CT:1695-1709` |
| R2 | `capped_millideg`'s default clock is 5.5545 ms (pass 0.005). Section factors F ≤ 1. Dwells borrowed (`insert_dwells`). | Porting traps and missed reuse. | `TT:767-769`, `TT:1439-1483` |
| R2 | Endpoint criterion: tolerance floor of 0.05°. Zero-phase endpoint behaviour verified exact after rounding on 5 files. | Avoids spurious failures on near-still starts. Evidence for the claim. | Appendix V; `TT:894-912` |
| R12 | Borrow items added: `slowness_census` (D6), `_departure_attribution`, `screen`. D3 thresholds marked as proposals (map 10 lists the spikes but no rule). | Missed reuse; attribution corrected. | `TT:958-991`, `TT:1083-1104`, `TT:4149-4162`; M10 §5 item 7 |
| R6 | Before/after zoom linked by raw-line position, not by seconds. The dark theme is unwired in the Studio. TIG dock tests are static guards. | Same-seconds linking shows different strokes. | `H:1290-1304`; `TT:767-832` (`return_u`); M01 §1.3; `TIG:2388-2525` |
| INT-TRAJ | Auto-optimise as the main operator path. x axis at the law's slope. Raw-file provenance from the control app. Legacy triage via ALIX `screen`. | Usability for non-technical operators; display consistency; missing input-provenance step. | `TRK:1558-1570`; `UI53:3840-3850`; `TT:4149` |
| R2 | Finding 4 softened: "ALIX abandoned this rule" became "ALIX moved away from step-by-step stillness judgement". | ALIX's old rule and the prototype's rule share the approach but not the thresholds. | `TT:926-928`; `H:1438-1492` |
| X | The v53 TypeError mention was dropped from C2. | The user asked for a high-level understanding of the control-app programs only, and it does not block C2 (it belongs to INT-CTRL). | user request |
| Effort | R2 ≈14–15 days (+2 for the G1/G5 port), still L. R11 ≈8–10 days (M if C3's worker exists). R12 ≈10–12 days (+1 for G1). | Scope added above. | — |

**Verified as stated** (no change needed):
- the prototype line references;
- the Studio undo, dock, dialog, maximise, plot, DAC, `ensurePlotly` and theme references;
- `TT` constants and functions;
- `TRK` parser and law;
- `RUN14` `read_track`;
- `FIND:5855-5857` and `FIND:6706-6716`;
- `GIC:77-99`;
- the `REQ` text;
- the map 13 naming;
- every re-measured number in Appendix V that matched.

---

## Appendix V — What the verifier re-measured, and how

Method: inline Python on stdin under `python3 -I`; no file was written.
- **Filters.** scipy cannot import under `-I` here, so `butter` was re-implemented: the analog prototype, then a bilinear transform with pre-warping. It reproduces scipy's `butter(2, 0.05)` coefficients exactly. `lfilter_zi`, `lfilter` and `filtfilt` (odd padding) were re-implemented too.
- **Band limit.** 4th order, 20 Hz, at 5 ms, with the default `padlen` (as `PTR:82-85`).
- **`smooth_taught`.** Order 2 with `padlen` = min(450, n−1) (as `TT:911-912`).
- **FK.** A numpy URDF chain parsed from `alix_workspace/urdf/alix.urdf`: `base_link` → `L_sliding_plate_joint` (pole 0.200 m) → `L_joint1..7` → `L_glove_frame_2`.
- **Deviation.** Exact point-to-polyline distance: global for FILE_10.1, ±200-line window for FILE_08 (same line count).

| item | result | matches draft? |
|---|---|---|
| Raw ticks over a rating at 5 ms (19 files) | 14 files. R_P06P07 172 (J6 490.8 °/s); lid_top 200/175/117/69 (J6 400.4/373.6/320.4/280.4); R2_LIDTOP 60 (J6 318.8); R_P10P12 58 (J7 355.2); R_P01P03 26; L_P01P02 23; R15_BUPFUL 19; L_P06P07, M3_ASOPN02, R_SPILLAGE 6; L_P10P12 1 | yes |
| **Band-limited (G2) ticks over a rating** | **6 files**. R_P06P07 102 (J6 311.7 °/s, 1.39×, longest 285 ms); R_P10P12 29 (J4 242.8, J7 229.3); lid_top 200/175/117/70. All other V6 files ≤ 0.91 | **new — corrects the draft** |
| R_P06P07 lines 5058→5059 | J3 1.049° (neighbours 0.271°, 0.276°); J5 1.220° (neighbours 0.820°, 0.871°) | yes |
| R10_HNGR J4 | max 133.466° (line 952), margin 1.534°; band-limited arrival ≈ 21.5 °/s; G1 cap 7.44 °/s (live `EM`) | margin yes; G1 analysis new |
| Prototype trim+smooth end steps | FILE_08 first 966 (J3); FILE_10.1 first 1005 (J6); FILE_01.1 last 546 (J2); FILE_02 last 696 (J7); FILE_6 first 254 (interior max 636) | yes (FILE_6 is the exception) |
| 5 Hz zero-phase endpoints | exact after rounding on all five commode files; first/last steps 0–1 millidegree; F3_LDTPS04 32/37 (boxcar 715/468) | yes |
| FILE_01.1 default suggestions and boundaries | `[75, 80, 95, 115, 75]`; 16/11/22/21 millidegrees per tick² = 640/440/880/840 °/s²; median 2 (80 °/s²); 9,812 output lines (9,813 in JS rounding) | yes |
| Rating-safe uniform speed-up (raw → 5 Hz → 2 Hz) | FILE_01.1 2.76/2.86/2.97; FILE_02 2.36/2.46/2.49; FILE_08 2.16/2.27/2.37; FILE_10.1 2.38/2.53/2.60; FILE_6 1.35/1.40/1.49; FILE_6 raw-tick utilisation 1.08 → 0.69 | yes |
| Pad deviation (max / p95, mm) | FILE_10.1: 5 Hz 0.91/0.33; 2 Hz 3.56/1.44; boxcar 10.39/4.67 (reverse 10.60/4.85). FILE_08: 5 Hz 1.53/0.42; 2 Hz 3.32/1.75; boxcar 9.05/5.32 | yes (reverse direction new) |
| Player law, 9,813 lines | 55.90 s; band 54.17–57.76 s; prototype 49.07 s | yes |
| scipy under `python3 -I` | numpy 2.4.6 (`/usr/local/lib/python3.12/dist-packages`) with scipy 1.11.4 → "numpy.core.multiarray failed to import" | yes |
