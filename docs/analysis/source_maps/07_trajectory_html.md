# 07 — Trajectory Optimisation HTML (`Alix_Studio_Lite_Trajectory_V2.html`): evidence map

Scope: a full technical inventory of the trajectory-optimisation prototype that is the basis for Lite's
**Trajectory Optimisation tab** (INT-TRAJ, R2, R6, R11, R12). I read all 2096 lines. I checked the cross-references
against the upload-channel runner, the V6 trajectory package, the ALIX monorepo motion lane and the ALIX Studio web editor.

## Path aliases used in citations

| alias | absolute path |
|---|---|
| `H` | /home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html |
| `REQ` | "/home/newtonjeri/alix_studio_lite_ws/alix_studio_lite/docs/reference/ALIX_Studio_Lite_requirement_inputs.txt" |
| `OPS` | "/home/newtonjeri/alix_studio_lite_ws/RealMan Dual-Arm Operations UI.txt" |
| `RUN` | /home/newtonjeri/alix_studio_lite_ws/upload_channel/rm_upload_fast_v14_safe_transit.py |
| `PLAN` | /home/newtonjeri/alix_studio_lite_ws/upload_channel/plan_txt_files/ |
| `V6` | /home/newtonjeri/alix_studio_lite_ws/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/ |
| `A` | /home/newtonjeri/alix_ws/src/alix |
| `TT` | A/tools/taught_tracks.py |
| `TRK` | A/core/python/alix_dispatch/src/alix_dispatch/track.py |
| `TPR` | A/docs/motion/TAUGHT_PIPELINE_README.md |
| `AUD` | A/docs/motion/MOTION_TRANSPORT_AUDIT_2026-08.md |
| `URDF` | A/ros/description/rm_description/urdf/rm_75_macro.xacro |
| `WE` | A/core/python/alix_taskgen/src/alix_taskgen/web_editor/ (`editor.js`, `editor.css`, `template.html`, `vendor/`) |
| `TEACH` | A/experiments/exp01_teach_trajectory_reverse_engineering/data/teach_files/ |

Labels: **(measured)** means I computed the value with an in-memory Python port of the HTML functions (`H:1417-1616`, `H:934-1050`),
run with `python3 -I -` on stdin. No file was written. **(computed)** means arithmetic on cited constants. **(inferred)** means not directly seen.

---

## 0. What the file is (architecture at a glance)

- It is a single self-contained static page with no server, no persistence and no network I/O. The only external dependency is Plotly, loaded from a CDN:
  `<script src="https://cdn.plot.ly/plotly-2.35.2.min.js">` (`H:7`). The only `fetch` turns a data-URL into a PNG blob (`H:1672`).
  No `localStorage`, XHR or WebSocket (grep: none).
- Layout: CSS `H:8-320` · markup `H:322-571` · JS `H:573-2093`, one IIFE with `'use strict'` (`H:574-575`, `H:2093`). There are no globals.
  All functions close over one mutable `state` object (`H:587-604`) and the `el = id => document.getElementById(id)` helper (`H:606`).
- Title: "Alix Studio Lite - Trajectory Processor" (`H:6`). H1: "Alix Studio Lite - 7-Joint Trajectory Processor". The subtitle is a
  developer note: "Guided pipeline: Upload → Setup → Analyze → Review → Generate & Save → Compare & Validate · Plot visibility fix" (`H:323-326`).
- There are **6 pipeline steps**, not 5. Section ids: `pipelineStep1` (step 1, `H:343`), `pipelineStep2` (step 2, `H:359`), `analysisSection`
  (step 3, `H:427`), `pipelineStep4` (step 4, `H:467`), `pipelineStep5` (step 5, `H:519`), `comparisonSection` (step 6, `H:537`).
- State model (`H:587-604`): `fileName, original (number[][]), validSourceLineNumbers, skippedLines, regions [{start,end,scalePct}],
  processed, processingReport, analysisSuggestions, analysisReport, externalOptimized, externalOptimizedName, pipelineStep,
  playback.{original,compare}.{playing,currentSec,startPerf,raf}`.

---

## 1. Inputs

| aspect | finding | evidence |
|---|---|---|
| Accepted file types | `.txt,.jsonl,text/plain`, one file (`fileInput`, no `multiple`) | `H:349-352` |
| Line format | NDJSON with one `{"point":[J1..J7]}` per line; the expected format is shown in the UI | `H:346`, `H:815-821` |
| Parser | `parseTrajectoryFile(file)`: `file.text()`, split on `/\r?\n/`, trim, skip blank lines; `JSON.parse`; accept only when `obj.point` is an array of **exactly 7** values that are all `Number.isFinite`; else record the 1-based line number in `skipped` | `H:803-829` |
| Minimum | throws "The file must contain at least two valid 7-joint rows." | `H:827` |
| Integer check | **none**. Floats are accepted; values are rounded only on output (`dataToTxt`). The runner *refuses* non-integers (`isinstance(v, int)`), so external floats would never reach the controller unchanged | `H:816`, `H:1618-1620`, `RUN:76-89` |
| Header lines | RealMan project headers (`{"file":7}`, Folder line, `RUN:91-97`) have no `point` key, so they are skipped as "malformed" with a warn status rather than rejected (inferred from the parser) | `H:816-818`, `H:1834-1837` |
| Units | none are declared. Values are treated as unitless "Joint Value" counts (`yaxis.title`), and the idle threshold is labelled "counts/sample". In reality they are integer **millidegrees** (`RUN:14`, `RUN:84-85`). Users see raw numbers such as 175713, not degrees | `H:852`, `H:374-375`, `H:843` |
| Sample period | `sampleMs` default **5 ms** (`min 0.001`), used for every duration: `trajectoryTotalSeconds = N × sampleMs/1000` (N, not N−1) | `H:366-367`, `H:731`, `H:1176-1179` |
| Cadence vs reality | the 5 ms recording clock is correct: ALIX measured drag-teach recordings at exactly 5 ms (`CLOCK_RECORD_S = 0.005`), and the V6 package also uses 5 ms/point. The **controller playback** clock is longer (see §9 R-T1) | `TT:109-115`, `V6/README.txt:4-6`, `V6/FINAL_FILE_RUNTIME_REPORT.csv:1` |
| Changing `sampleMs` | re-runs the automatic analysis, which **discards user edits to suggestions**; nulls `processed` without `invalidateProcessed()`, so the pipeline UI goes stale | `H:1871-1886`, `H:1146` |
| Multi-file | **single file only**; there is no arm (L/R), region or task identity | `H:351` |
| `optimizedFileInput` | optional **external optimised file used only for comparison** (step 6). Parsed with the same parser into `state.externalOptimized`; it is not processed, and the region table says metadata is unavailable. `useGeneratedBtn` switches back to the generated output. Generating again discards the external file | `H:543-548`, `H:2001-2028`, `H:1248`, `H:1759-1761` |
| Chicken-and-egg | `optimizedFileInput` lives inside step 6, and step 6 unlocks only when `processed \|\| externalOptimized`. So **two existing files cannot be compared without first generating** | `H:537-548`, `H:618-623` |
| On load | resets regions, processed output, suggestions and external file; sets `manualStart=1`, `manualEnd=N`; enables controls; **runs the automatic analysis immediately**; defers plotting until step 4 is visible | `H:1811-1869`, `H:1841-1844`, `H:1861-1863` |

---

## 2. Processing pipeline (exact detail)

### 2.1 Parameters (`getParams`, `H:729-737`)

| id | default | sanitising | UI label | disabled when |
|---|---|---|---|---|
| `sampleMs` | 5 | `max(0.001, v \|\| 5)` | "Sample interval (ms)" | never |
| `idleWindow` | 11 | `validateOdd` → odd ≥ 3 (`H:723-727`) | "Idle rolling window" | Trim off (`H:792`) |
| `idleThreshold` | 8 | `max(0, v \|\| 8)` | "Idle threshold (counts/sample)" | Trim off (`H:793`) |
| `idleMinRun` | 20 | `max(1, round(v) \|\| 20)` | "Minimum idle run" | Trim off (`H:794`) |
| `smoothWindow` | 101 | `validateOdd` → odd ≥ 3 | "Mean smoothing window" | Smooth off (`H:795`) |

Markup: `H:364-385`. The UI says "Defaults match the trajectory algorithm used previously" (`H:362`). The files in `PLAN`
(`lid_top_trimmed_mean_smoothed_{28,30,35,40}pct.txt`) are probable outputs of that earlier algorithm (inferred from the names).
`processTrajectory` writes the odd-corrected windows back into the inputs (`H:1687-1689`).

### 2.2 Stage toggles and order

- Checkboxes `stageTrim`, `stageSmooth` and `stageScale` are all checked by default (`H:389-402`). `getStageSelection()` returns
  `{trim, meanSmooth, scale}` (`H:740-746`). The hint reads "always applied in this order: **Trim → Mean Smooth → Scale**. Unchecked stages are skipped completely" (`H:403-406`).
- If no stage is selected, generation throws (`H:1680-1682`). A stage change calls `invalidateProcessed(...)` and `updateStageUI()` (`H:1892-1900`).
  If Scale is off, saved regions are kept but ignored, with a warn status (`H:798-800`, `H:1382-1384`).
- The orchestrator is `processTrajectory()` (`H:1676-1809`): Trim (`H:1691-1701`), then Smooth (`H:1703-1709`), then Scale (`H:1711-1719`).
  It then **pins the global first and last rows to the original's** (`H:1721-1723`), builds the report (`H:1732-1755`) and refreshes the comparison (`H:1762`).

### 2.3 Idle detection and trimming (`trimIdle`, `H:1438-1492`). The maths:

```
for each joint j, i = 0..n-2:   d_j[i] = |q[i+1][j] - q[i][j]|                          (H:1446-1447)
s_j = centeredRollingMean(d_j, W)   (replicate-edge padding, divide by W)            (H:1417-1436, H:1448)
idle[i] = AND over j of ( s_j[i] <= T )                                              (H:1451-1458)
maximal run of idle transitions a..b; if (b-a+1) >= M:
    remove rows a+1..b (0-based), keep rows a and b+1                                (H:1463-1481)
    region = {originalStartRow:a+1, originalEndRow:b+2, transitions:b-a+1, removedPoints:b-a}
```

- Defaults in physical units (computed, at 5 ms): W = 11 transitions ≈ 55 ms; T = 8 counts/sample = 0.008°/5 ms = **1.6 °/s mean
  |speed| on every joint**; M = 20 transitions = 100 ms.
- The first and last rows are never removed: removal is `a+1..b` with `b ≤ n-2` (`H:1442`, `H:1472`).
- Returns `{data, keptOriginalIndices, regions}`. `keptOriginalIndices` is what lets sub-regions defined on ORIGINAL rows be mapped later (`H:1483-1491`).
- Semantics: a stop is collapsed to **two points**, its entry and exit. The arm passes through the stop without dwelling, and any
  drift inside the block becomes one step (§9 R-A2). The help text matches: "keep the first and last point of each idle block and remove the interior points" (`H:411-415`).
- (measured, defaults, raw commode-C recordings in `TEACH/commode_c_taught_paths/`):

| file | rows (s @5 ms) | idle blocks | rows removed | largest collapse jump |
|---|---|---|---|---|
| FILE_01.1 | 12429 (62.15) | 17 | 1431 (11.5 %) | 0.145° |
| FILE_02 | 13150 (65.75) | 12 | 1976 (15.0 %) | 0.114° |
| FILE_08 | 8344 (41.72) | 7 | 586 (7.0 %) | 0.273° |
| FILE_10.1 | 3293 (16.46) | 5 | 425 (12.9 %) | 0.405° (created FILE_10.1's largest J7 step) |
| F3_LDTPS04 | 4412 (22.06) | 0 | 0 | — |

### 2.4 Smoothing (`meanSmooth`, `H:1494-1517`)

- Algorithm: a per-joint **centred moving mean** of `W = smoothWindow` (default 101) samples (`H:1506`). At the edges the
  **first/last value is replicated** (`centeredRollingMean`, `H:1424-1433`); it is O(n) with prefix sums.
- Mean preservation: `correction = mean(original) − mean(smoothed)` is added to every sample (`H:1503-1510`). Values are rounded to integers (`H:1512`).
- Endpoints: `out[0]` and `out[n-1]` are **overwritten with the exact input endpoints** (`H:1513-1514`). This is described as
  "exact first and last joint positions are restored" (`H:416-419`). The start and end points are preserved, **but this creates a step** (§9 R-A1).
- Frequency response (computed): 101 × 5 ms = 505 ms boxcar, giving a **≈0.88 Hz −3 dB** low-pass with its first null near 1.98 Hz. ALIX's
  hardware-validated recipe is a **5 Hz 2nd-order zero-phase Butterworth at the 5 ms record clock** (`smooth_taught`, `TT:894-913`;
  rationale `TT:884-890`, which moves the pad 0.30 mm p95 / 1.11 mm max). The HTML filter is about 5.7× more aggressive.
- (measured) max |smoothed − trimmed| per joint: FILE_01.1 1.16–2.84°, FILE_02 1.83–3.37°, FILE_08 0.97–2.88°, FILE_10.1 0.72–1.88°,
  **F3_LDTPS04 2.6–8.9° (J6 8.875°)**. These are joint-space corner cuts. At roughly a 0.6 m lever, 1° ≈ 10 mm at the pad (inferred).
  ALIX's G4 gate is 10 mm against the recording (`TT:129-132`).
- Smoothing does not change the point count (`rowsAfterMeanSmooth == rowsAfterTrim`, `H:1742-1743`).

### 2.5 Scaling (time scaling by resampling)

- `interpolateRegion(region, scalePct)` (`H:1568-1590`): `newIntervals = max(1, round((L-1) × pct/100))`, `newLen = newIntervals+1`.
  It samples uniformly in normalised time, **linearly interpolates all 7 joints synchronously**, rounds to integers, and **pins the region's first and last rows** (`H:1587-1588`).
- The point count changes. Time per point stays `sampleMs`, so duration ∝ pct: 100 % = unchanged, 150 % = 50 % longer, 30 % = 30 % of the duration (`H:420-423`).
- `applyRegionScales(data, mappedRegions)` (`H:1592-1616`) copies the unscaled spans verbatim and splices each scaled region. There are no duplicated
  boundary rows: the region includes `trimmedStart..trimmedEnd`, and the cursor resumes at `trimmedEnd+1`. It returns
  `{data, regionReport:[{originalStartRow, originalEndRow, trimmedStartRow, trimmedEndRow, scalePct, inputRows, outputRows}]}`.
- `mapRegionsToTrimmed(regions, keptOriginalIndices)` (`H:1537-1566`) maps original-row regions to trimmed indices with `lowerBound`/`upperBound`
  (`H:1519-1535`). A region that falls fully inside a removed idle block is marked `skipped` (warned at `H:1793-1794`). An overlap after mapping throws (`H:1559-1564`).
  This is defensive: monotone mapping of non-overlapping regions cannot overlap (inferred).
- There is **no boundary blending, no velocity/acceleration limit, and no anti-alias filter** before down-sampling. When Smooth is off, down-sampling a noisy recording keeps the noise (inferred).
- Bounds on the factor: suggestions are clamped 1–300 % (`H:1071`, `H:1087`). Manual regions are clamped to ≥1 % with **no maximum** (`H:1352`, `H:1362`).

### 2.6 Outputs

| output | format | evidence |
|---|---|---|
| Processed TXT (`downloadTxtBtn`) | `JSON.stringify({point: row.map(Math.round)})` per line, LF, trailing newline. Name: `<base>_<trim_mean_scale\|unchanged>_processed.txt`. Byte-compatible with the runner's `read_track` (7 ints), which converts to CRLF itself | `H:1618-1620`, `H:756-762`, `H:1914-1936`, `RUN:76-97` |
| Report JSON (`downloadReportBtn`) | `<base>_<suffix>_processing_report.json`. Keys: `sourceFile, generatedAt, selectedStages{trim,meanSmooth,scale}, selectedStageLabel, parameters{sampleMs,idleWindow,idleThreshold,idleMinRun,smoothWindow}, sourceValidRows, skippedSourceLines[], idleRegions[{originalStartRow,originalEndRow,transitions,removedPoints}], idlePointsRemoved, rowsAfterTrim, rowsAfterMeanSmooth, scalingRegions[...]` (as §2.5) `, automaticOptimizationAnalysis{generatedAt,method,contactDataAvailable:false,suggestions[]}, finalRows, originalTimeSeconds, afterTrimTimeSeconds, afterMeanSmoothTimeSeconds, finalTimeSeconds, endpointPreserved{first,last}` | `H:1732-1755`, `H:1938-1960`, `H:1147-1152` |
| Report gaps | no input/output SHA-256, no arm/side, region/task id, operator or tool version, and no limit/velocity verdict. `automaticOptimizationAnalysis` is a snapshot from analysis time, so later edits to suggestions are not reflected | `H:1147-1152` |
| Plot PNG (`downloadPlotBtn`) | `Plotly.toImage('processedPlot', png 1600×900)`, saved as `<base>_<suffix>_processed_J1_to_J7.png` | `H:1665-1674`, `H:1962-1983` |
| Save mechanism | `showSaveFilePicker` (Chromium/Edge: native Save As), else an anchor download. Cancellation is reported | `H:1622-1663`, `H:530-532` |

---

## 3. Automatic speed-optimisation analysis (step 3, "Section B1/B2")

- `analyzeMotionRegions(data)` (`H:944-1050`) runs on the **original, untrimmed** rows, automatically on load (`H:1862`), on a `sampleMs` change (`H:1874`) and on "Analyze Again" (`H:1985`).
  - Per-sample signals (`H:948-959`): `step[i] = ‖q[i] − q[i−1]‖₂` over the 7 joints (counts/sample; this mixes joints, applies no dt and no per-joint rating), and
    `accel[i] = |step[i] − step[i−1]|`, a change in step norm rather than a true acceleration vector.
  - **Segmentation is equal-length chunks, not motion-based**: `regionCount = min(10, max(5, round(n/3000)))` (`H:961-962`).
    So **any file under 16,500 rows (82.5 s) gets exactly 5 chunks** (computed). Per chunk: `avgStep, maxStep, avgAccel, originalSec = count·dt` (`H:964-982`).
  - Thresholds are **relative quartiles across the chunks themselves** (`H:984-992`): `q25/q50` of avgStep, `q75` of maxStep, `q25/q50/q75` of avgAccel.
  - Classification is first-match (`H:994-1037`):

| rule (order) | motionType | action (badge text) | peakGuidance | scalePct |
|---|---|---|---|---|
| default | General continuous motion | Slightly shorter | Keep moderate | 95 |
| chunk 0 (**positional**) | Approach candidate | Shorter / faster | Can increase carefully | **75** |
| last chunk (**positional**) | Exit candidate | Shorter / faster | Can increase carefully | **75** |
| avgAccel ≥ q75Accel & > 0 | Direction-change / high-dynamic region | Longer / slower | Limit peak speed | 115 |
| maxStep ≥ q75Max & avgAccel ≥ q50Accel | High-peak motion | Keep or slightly slower | Limit peak speed | 105 |
| avgStep ≤ q25Step | Slow / possibly redundant | Shorter / review for trim | Can increase after review | 80 |
| avgAccel ≤ q25Accel & avgStep ≥ q50Step | Smooth / possible straight cleaning pass | Moderately shorter | Can increase moderately | 85 |

  - Every suggestion is **pre-selected**: `selected: scalePct !== 100`, and no rule yields 100 (`H:1045`). `estimatedSec = originalSec × pct/100` (`H:1046`).
- Badges: `recommendationBadge` uses class `fast` (<100), `slow` (>100) or `keep` (=100) (`H:1052-1055`; CSS `H:179-191`).
- `suggestionsTable` (`H:447-463`, rendered by `renderSuggestionsTable` at `H:1057-1099`) has columns Use ☐ · # · Rows · Original time · Motion type ·
  Peak guidance · Recommendation · Scale (%) (editable 1–300) · Estimated time · Reason. Editing the scale updates the badge text
  ("Shorter / faster" / "Longer / slower" / "Unchanged"), the time and the metrics (`H:1084-1097`).
- Metrics `analysisMetrics` (`H:1101-1119`): detected regions, faster count, slower count, "Motion-only estimated time", "Estimated change".
  This does **not** include trim savings, so it never matches the step-5 result (inferred from the formulas).
- Plot shading `analysisShapes()` (`H:1121-1132`) draws green, orange or blue rects for **every** suggestion, selected or not.
- `applySuggestionsBtn` → `applySelectedSuggestions()` (`H:1159-1174`): warns if none are selected; `window.confirm` before **replacing all manual regions** (`H:1166`);
  forces `stageScale` on; `state.regions = chosen.map({start,end,scalePct})` (this bypasses `addRegion`'s overlap and size checks); invalidates the processed output.
- Honesty note in the UI: "joint positions only … cannot prove physical cleaning contact or collision safety … **possible cleaning/straight-pass candidates** for operator review" (`H:436-439`).
- (measured) on real recordings: FILE_01.1 gave `[75,80,95,115,75]` and an estimated 12.0 % saving; FILE_08 `[75,80,80,95,75]` 19.0 %; F3_LDTPS04 `[75,85,80,115,75]` 14.0 %;
  bwl_out_v3 (18243 rows, 6 chunks) `[75,115,115,80,95,75]` 7.5 %. The first and last fifth of a cleaning file are always sped up by 33 %, whether or not
  they contain cleaning strokes (FILE_01.1 chunk 1 = rows 1–2485, 12.4 s).

---

## 4. Manual scaling regions (step 4) — these are Lite's "sub-regions"

- They are defined on **ORIGINAL row numbers (1-based)**, "Regions may not overlap", and each has an independent scale % (`H:472-474`).
- There are two ways to add one:
  1. Inputs `manualStart`/`manualEnd` (**row indices, not time**) plus `addManualBtn` (`H:490-501`, `H:1902-1904`). The defaults after load are 1 and N, so a click adds a whole-file region (`H:1841-1844`).
  2. **Box-select on the original plot**: `dragmode:'select'`, `selectdirection:'h'` (`H:855-856`). `plotly_selected` calls `addRegion(ceil(x0), floor(x1))`, then the selection is cleared (`attachOriginalPlotSelectionHandler`, `H:872-892`).
- `addRegion(start,end)` (`H:1392-1415`): refused if Scale is off (`H:1394-1397`); clamped to [1,N]; needs `e > s`; **overlap is rejected** with an err status (`H:1405-1409`);
  pushes `{start,end,scalePct:100}`, sorts, invalidates.
- `regionsTable` (`H:503-515`, `refreshRegionsTable` at `H:1337-1390`): # · start · end · editable Scale (%) (disabled when Scale is off) · Meaning ("x % longer/shorter") · Delete.
  There is no start/end editing after creation (delete and re-add instead).
- `clearRegionsBtn` empties all regions and invalidates (`H:1906-1910`). Controls are enabled only when Scale is on and a file is loaded (`updateStageUI`, `H:784-801`).
- Interaction with suggestions: applying suggestions **replaces** manual regions after confirmation (`H:1166-1169`). Suggestions and manual regions are never merged.
- **Manual regions are never drawn on the plot.** Only suggestion rects and the cursor are drawn (`updateOriginalPlotShapes`, `H:1134-1142`; there is no `state.regions` use in shapes, per grep `H:799-2048`).
- Bug: the scale input handler sets `state.processed=null` directly (`H:1359-1371`) instead of calling `invalidateProcessed()`. As a result, the pipeline UI, the comparison section and
  `processStatus` go stale, and step 6 stays visually unlocked but silently refuses navigation (`H:659-661`).

---

## 5. Playback and comparison

- Original playback (step 4): `playOriginal()` (`H:1217-1232`) runs a `requestAnimationFrame` loop at **real time (1×) only**, with `pauseOriginalBtn`, `restartOriginalBtn`
  and the `originalTimeline` range slider (`H:475-487`, `H:1988-1999`). `updateOriginalPlaybackUI()` (`H:1196-1208`) updates the time readout, the
  "Current region: n – motionType" label (`currentRegionText`, `H:1189-1194`) and **a red vertical cursor line (#b42318)** relayouted on `originalPlot` every frame (`H:1134-1142`).
- Time→row mapping: `rowForTime = 1 + (sec/total)·(N−1)` with `total = N·dt` (`H:1181-1187`), a slight N vs N−1 inconsistency.
- Comparison (step 6): `playCompare()` (`H:1306-1322`) drives a synchronized time cursor. Each plot maps the same seconds to its own row (`H:1290-1304`, `compareCursorShapes` at `H:1286-1288`).
  The timeline max is max(duration_orig, duration_opt) (`H:1280-1283`).
- What is animated: **only a cursor line on 2-D joint plots**. There is no robot, no 3-D view, no FK, no collision check, no joint-value readout at the cursor (only hover) and no click-to-seek
  (no `plotly_click` handler; grep).
- Metrics:
  - `originalMetrics` (`H:1324-1335`): Valid rows · Skipped/malformed lines · Sample interval · Original file time.
  - `processedMetrics` (`H:1765-1778`): Selected stages · Original rows · Idle points removed · Rows after Trim · Rows after Mean Smooth · Scaling regions applied ·
    Final rows · Original time · Final time · Time saved · Reduction %. The status text summarises each stage ON/OFF (`H:1780-1797`).
  - `comparisonMetrics` (`H:1271-1276`): Original duration · Optimized duration · Time saved · Percentage reduction · Modified scale regions.
  - `comparisonRegionsTable` (`H:563-569`, `H:1242-1258`): # · Original rows · Scale % · Input rows · Output rows · Effect.
- All durations are N × 5 ms. They are **not** the controller's real replay time (§9 R-T1).

---

## 6. Plots (Plotly 2.35.2)

| aspect | finding | evidence |
|---|---|---|
| Plot divs | `originalPlot` (`.plot`, 600 px) in step 4; `originalComparePlot` and `processedPlot` (`.plot.small`, 520 px) in step 6 | `H:488`, `H:561-562`, `H:125-129` |
| Heights | **fixed in CSS**. At ≤850 px `.plot` becomes 480 px (`.plot.small` stays 520 px because of specificity) | `H:125-129`, `H:311-313` |
| Width | `config.responsive:true`, plus `Plotly.Plots.resize` after a step change (double rAF to avoid hidden-div rendering) | `H:862-867`, `H:659-672`, `H:894-920` |
| Traces | 7 line traces `J1..J7`, colours `JOINT_COLORS` = category10 (`#1f77b4,#ff7f0e,#2ca02c,#d62728,#9467bd,#8c564b,#e377c2`), width 1.5. `scatter` (SVG), or `scattergl` when N > 15000 | `H:577-585`, `H:831-845` |
| Axes | x = **row number 1..N** ("Sample / Row Number"), not seconds; y = raw counts ("Joint Value"). Original and processed have **different x lengths**, so they are not visually time-aligned | `H:832`, `H:851-852` |
| Y range | `yRangeFor(a,b)` shared min/max + 5 % (≥1) pad; the comparison plots share one y-range | `H:922-932`, `H:1268-1270`, `H:904` |
| Interaction | `hovermode:'x unified'`; horizontal legend (click hides a joint, Plotly default); `scrollZoom:true` (the **mouse wheel zooms instead of scrolling the page**); modebar without the logo and `lasso2d`; original plot drag = box-select, others = zoom | `H:853-856`, `H:862-867` |
| Linked zoom | **none**. The two comparison plots zoom independently (no `plotly_relayout` handler; grep) | — |
| Shapes | suggestion rects (`layer:'below'`) and the cursor line | `H:1121-1142`, `H:1286-1288` |
| Export | modebar PNG (default) plus `plotToPngBlob` 1600×900 | `H:1665-1674` |
| Re-render | `Plotly.react` each time step 4 or step 6 becomes visible; `Plotly.purge` on reset | `H:847-870`, `H:894-920`, `H:2070-2072` |
| Version | the CDN version **2.35.2 is identical to ALIX Studio's vendored copy** (`WE/vendor/plotly.min.js` header "plotly.js v2.35.2"; 4,558,696 bytes, lazy-loaded by `ensurePlotly()` at `WE/editor.js:5322-5337`, offline note at `WE/template.html:7`) | `H:7` |

---

## 7. UX (stepper, hints, statuses, jargon) — suitability for non-technical users

- Stepper: a sticky (`position:sticky; top:0`) `pipeline-shell` with 6 `pipeline-node` buttons and states `active` / `done` (✓) / `locked` (`H:220-299`, `H:329-342`).
  `pipelineStepUnlocked`: step 1 is always unlocked; steps 2–5 need a loaded file; step 6 needs a processed or external file (`H:618-623`). `updatePipelineUI` sets `aria-current` and Next labels
  ("Next: Generate & save →", "Generate first to unlock Compare") (`H:625-657`). `installPipelineNavigation` injects Back/Next bars and per-step notes from `PIPELINE_META` (`H:609-616`, `H:680-704`).
- Feedback: `.status` boxes in neutral, ok, warn and err (`H:99-110`, `setStatus` at `H:718-721`); `.hint` (`H:90-98`); `.analysis-note` disclaimer (`H:170-178`); `<details>` "Algorithm used" (`H:409-424`);
  `window.confirm` before replacing regions (`H:1166`); buttons disabled until valid (`H:1846-1856`, `H:2086`).
- Jargon a non-technical operator meets: "TXT / JSONL", "Idle rolling window", "Idle threshold (counts/sample)", "Minimum idle run", "Mean smoothing window",
  "centered moving mean", "selected-region resampling", "synchronized linear interpolation", "Section B1/B2", "Section B4", "Direction-change / high-dynamic region",
  "Peak guidance", "Sample / Row Number", "Joint Value", "Original start row", "Apply Selected Suggestions to Scale Regions", "processing report (JSON)", and the dev note
  "Plot visibility fix" (`H:325`, `H:350`, `H:366-384`, `H:392-400`, `H:430`, `H:540`, `H:1014`).
- Friction for operators: row numbers instead of seconds; raw millidegree values; five numeric parameters exposed up front; no presets ("gentle/standard/fast"); no "safe to run" verdict;
  the result lives in browser downloads, not the region's slot; reset does not restore the parameter defaults (`H:2043-2089`); wheel-zoom hijacks page scroll; regions are invisible on the plot;
  real-time-only playback (a 60 s file takes 60 s to review).
- Verdict: a **good guided-flow skeleton for an engineer**, but **not yet operator-grade**. Keep the stepper pattern, status and hint idioms; hide the parameters behind "Advanced"; add presets and plain-language verdicts.

---

## 8. Gaps against the requirements

| req | requirement text (REQ) | present in H? | gap |
|---|---|---|---|
| R2 | "Each region and sub-region should have independent options to adjust smoothing, trimming, and scaling, with an undo option for up to five previous changes" (`REQ:10`) | **Partial** | per sub-region: **scale only** (`H:1392-1415`). Trim and smooth parameters are **global** (`H:729-737`). Per region (file): only by processing each file separately, with no stored recipe. **No undo/redo/history at all** (grep: none). |
| R6 | "resize and adjust the graph, and editing windows" (`REQ:18`) | **Partial** | Plotly zoom, pan, scroll-zoom, legend toggle and PNG are present. Plot heights are fixed (`H:125-129`); no splitter or resizable panels; no linked zoom; no persisted layout. |
| R11 | "adjust trajectory smoothness and immediately validate the changes through simulation" (`REQ:28`) | **No** | smoothing is a numeric window followed by Generate (`H:1888-1890`, `H:525`). There is no live preview, no 3-D or simulation, and no deviation metric. |
| R12 | "identify high and low spikes … provide suggestions to reduce sudden movements" (`REQ:30`) | **No (building blocks only)** | `step[]`/`accel[]` exist per sample but are only aggregated per chunk (`H:948-982`). There are no spike markers, no per-joint velocity or acceleration, no thresholds tied to ratings, and no fix suggestions beyond the "Limit peak speed" text (`H:1016`, `H:1022`). |
| Limits | joint position, velocity and acceleration checks | **None** (grep "limit/velocity/deg" only hits guidance text) | URDF: J1/J3/J5 ±178°, J2 ±130°, J4 ±135°, J6 ±128°, J7 ±360°; velocity 180/180/225×5 °/s; acceleration 10.471975 rad/s² = 600 °/s² (`URDF:201-430`). ALIX gate: `canfd_preflight()` (`TRK:3792-3864`), `CANFD_RATED_DEG_S` (`TRK:3728`), `STEP_FAULT_DEG=10` (`TT:127-128`). |
| Dual-arm | dual-arm review (R10) | **No** | one file at a time; no L/R lanes, shared timeline or duration matching. (V6 matched arms by stretching: `V6/README.txt:7-8`.) |
| R3/R5 | simulation playback, collision stop and highlight | **No** | 2-D cursor only (`H:1134-1142`). The UI itself warns it cannot prove collision safety (`H:436-439`). |
| R1/R7/R8/R9/R13/R14 | region flow, pole/hand, merge, offset-IK, upload, offset sim | **No** | out of the HTML's scope. The output TXT is upload-compatible (`RUN:76-89`). |
| NFR-THEME | keep the ALIX Studio theme | **Mismatch** | HTML tokens: `--bg #f4f6f8, --card #fff, --text #17202a, --muted #667085, --line #d0d5dd, --accent #175cd3, --accent2 #0e9384, --danger #b42318, --warning #b54708, --ok #027a48`; dark header `#0b1220`; radius 8–12 px (`H:9-21`, `H:29-44`, `H:54-75`). ALIX: `--brand #2F96EE, --brand-strong #1B7FD4, --bg #FFFFFF, --border #EEEEEE, --border-strong #DCDFE3, --text #14181D, --muted #6B7480, --ok #2E9E5B, --warning #C77700, --danger #D93A34, --radius 4px, --font Inter…, --mono JetBrains Mono`, spacing `--s1..--s7`, targets `--target-min 24px / --target-primary 44px` (`WE/editor.css:22-58`, `:109-128`); dark set at `WE/editor.css:134+` (no setter found in editor.js/template.html, so not wired (inferred)). |
| Offline | robot laptops may be offline | **Risk** | Plotly comes only from `cdn.plot.ly` (`H:7`). If offline, `Plotly` is undefined and every plot call throws. Fix: serve `WE/vendor/plotly.min.js` (same 2.35.2). The `Inter` font is not loaded anywhere (system fallback). |

---

## 9. Algorithm correctness risks (with measured evidence)

**R-A1. Endpoint snap from smoothing on a file that is moving at its ends (high).** Trim collapses the leading and trailing dwell, so motion starts at row 1. A recording can
also start or end in motion by itself: F3_LDTPS04 has no idle block. The 101-pt mean near the edges then averages moving samples, and `meanSmooth` overwrites `out[0]` and
`out[n-1]` with the exact input (`H:1513-1514`). The result is **a single-sample jump at the first and last step**.
(measured) trim+smooth start/end snaps reached **966 counts (FILE_08 J3, 0.966°/5 ms = 193 °/s from rest)** and **1005 counts (FILE_10.1 J6, 201 °/s)**,
with 546 (FILE_01.1 J2), 696 (FILE_02 J7) and 715 (F3_LDTPS04 J1, no trim involved). Smooth alone on recordings that start and end at rest gives only 1–17 counts
(FILE_01.1 4/1, FILE_08 17/2, FILE_10.1 7/2, start/end). In **all 5 measured files** the snap is **the largest single-joint step in the trim+smooth output**: the smoothing stage creates the
sharpest movement in the file. The global endpoint pin (`H:1721-1723`) keeps it. On the robot this is a velocity
step right after the controller's transit to the first point (`RUN:26-29`). Fix: keep ≥ W/2 dwell samples at each end, ramp the edge with a raised cosine, or use
zero-phase filtering with proper padding (ALIX `smooth_taught`, `TT:894-913`).

**R-A2. Idle trimming judged step by step (medium).** A block is "idle" if the *rolling mean of per-step |Δq|* ≤ 8 counts on all joints (`H:1445-1458`). A slow but
real motion (an apex or a slow wipe at < 1.6 °/s per joint) is collapsed to two points, and its drift becomes one step. ALIX documents this failure and fixed it: "STILL MEANS WITHIN tol OF
THE RUN'S FIRST LINE, not of the previous line: a slow apex … is path, not a pause (judged step by step it was collapsed, cutting the apex by up to 9 mm at the pad)"
(`strip_pauses`, `TT:915-953`, comment `TT:926-928`; `PAUSE_TOL_DEG=0.05`, `PAUSE_MIN_LINES=3` at `TT:237-241`). (measured) collapse jumps on commode files were
0.11–0.41°. FILE_10.1's 0.405° J7 jump became that joint's largest step after trim. With Smooth unchecked these jumps go straight to the controller.

**R-A3. Over-smoothing and path deviation (high for coverage and collision).** A ≈0.88 Hz boxcar versus ALIX's validated 5 Hz Butterworth (§2.4). (measured) Joint deviation from the
trimmed path had per-file maxima of 1.9–3.4° on commode files and **8.9° (J6) on F3_LDTPS04**. Corners and stroke reversals are rounded, which risks missed spots (R3) and changes clearance to the commode (R5).
There is no TCP or FK deviation metric. ALIX holds the bytes within 10 mm of the recording (`TT:129-132`, `deviation_mm` at `TT:607`).

**R-A4. Exceeding joint velocity ratings after scaling (high).** There is no check (§8). Rating at the 5 ms gate clock: J1/J2 0.9°/step, J3–J7 1.125°/step (computed from
`TRK:3728`, `TT:102`). (measured, whole-file scale after trim+smooth) utilisation >1 at **30 %** for FILE_02 (1.17), FILE_08 (1.27), FILE_10.1 (1.16), F3 (2.27); at
**50 %** for F3 (1.36); at 10 % for every file 2.75–6.79×. The UI accepts down to 1 %. **Production evidence:** `PLAN/lid_top_trimmed_mean_smoothed_28pct.txt` peaks at
J6 2.002°/5 ms = **400 °/s (1.78× rating), with 200 lines over**; 30 % 374 °/s (175 lines), 35 % 320 °/s (116), 40 % 280 °/s (69). **10 of the 15 V6 final files** have lines
over rating. The worst is `V6/RIGHT/R_P06P07.txt` J6 2.454°/step = 491 °/s (2.18×, 172 lines), which matches `V6/FINAL_FILE_RUNTIME_REPORT.csv:14` (max 2.454°). The runner
knows error 4105 "joint speed over its limit (a warning; the run goes on)" (`RUN:56-57`), and ALIX notes the arm then clamps at the rating with about 1.7× the path departure
(`TT:95-101`). No file reached the 10° step fault (max 2.454°).

**R-A5. Velocity discontinuities at sub-region boundaries (medium).** Each region is resampled independently with no blending (`H:1592-1616`). Speed jumps by
`v·|1/f₂ − 1/f₁|` in one sample. (measured, default suggestions applied after trim+smooth) one-step Δv at boundaries was 14/6/22/21 counts (FILE_01.1) and 8/8/8/25 counts (FILE_08),
which is **≈240–1000 °/s²**. That is a single-sample estimate on integer data, where 1 count = 40 °/s². **3 of the 8 boundaries (840, 880, 1000 °/s²) exceed the 600 °/s²
URDF/ALIX acceleration limit** (`URDF:201`, `TPR:57`), against a 120–320 °/s² neighbourhood. Even an 80 %→80 % boundary shows an 8-count blip from the resampling phase. With larger factor
gaps (for example 30 %→115 %) the jumps scale accordingly (inferred).

**R-T1. Cadence and duration assumption (medium, UX trust).** One `sampleMs` is used for both the recording clock and the playback clock. The recording clock is exactly 5 ms (`TT:109-115`).
The controller's online-program player averages **5.5545 ms nominal (5.4–6.2 measured)** and advances at most one point per 5 ms cycle (`TT:95-108`;
`AUD:357` gives 5.629 ± 0.065 ms; `A/docs/motion/TRACK_CHANNEL_TIMING_REQUIREMENT.md:15`). ALIX's duration envelope is `replay_s = ramp + rate×(N−1)`, with ramp 0.057–0.750 s and
rate 5.515–5.810 ms (`TT:116-122`). (computed) for FILE_01.1 processed to 9813 rows, the HTML says 49.07 s while the envelope says 54.2–57.8 s, so the HTML under-reports by 9–15 %.
The figures also assume 100 % overall speed (the control app has OVERALL SPEED %, `OPS:26-28`, `OPS:182-185`; `rm_set_plan_speed`, `RUN:1855-1869`).
Velocity gating should use 5.0 ms (worst case), and displayed durations should use the envelope.

**R-A6. Heuristic suggestions are positional and relative (medium).** First and last fifth are always 75 %. Quartile rules guarantee that some chunk is "slower" and some "faster" regardless of
absolute speed (`H:984-1037`). Chunks ignore motion boundaries (`H:962-966`). The suggestions measure nothing physical (no dt, no per-joint rating).

**R-A7. Integer rounding (low).** Values are rounded at smoothing (`H:1512`), interpolation (`H:1584`) and output (`H:1619`). Error ≤ 0.5 count (0.0005°). This adds ±1-count quantisation
jitter, which is negligible against the controller's resolution (`AUD:204` notes the 0.001° tick). The mean-correction constant shifts a whole joint by up to 4.6 counts (measured, FILE_10.1 J6).

**R-A8. Joint position limits (low for these stages).** Moving mean and linear interpolation are convex combinations, so outputs stay within the input range (plus the ≤ ~5-count correction).
Trim, smooth and scale therefore cannot create position-limit violations (inferred). Position limits matter for R9/R14 offset-IK, not here.

**R-A9. Edge cases (low).** For N < ~10 the equal-chunk analysis yields overlapping suggestions, and applying them throws "overlap after idle trimming" (`H:965-966`, `H:1168`, `H:1559-1564`).
The suggestion scale input shows unclamped text while the state is clamped (`H:1087`). Changing `sampleMs` wipes suggestion edits (`H:1874`).

---

## 10. Porting assessment

**Pure, DOM-free functions** that can move into a Lite JS module (`traj_core.js`) as-is or with one-line changes:

| function | lines | change needed |
|---|---|---|
| `centeredRollingMean` | `H:1417-1436` | none |
| `trimIdle` | `H:1438-1492` | none (later: anchored-tolerance rule, R-A2) |
| `meanSmooth` | `H:1494-1517` | fix the endpoint snap (R-A1) |
| `lowerBound` / `upperBound` | `H:1519-1535` | none |
| `mapRegionsToTrimmed` | `H:1537-1566` | none |
| `interpolateRegion` | `H:1568-1590` | add boundary blending / time-warp (R-A5) |
| `applyRegionScales` | `H:1592-1616` | none |
| `dataToTxt` | `H:1618-1620` | none (format already matches `RUN:76-89`) |
| `percentile`, `yRangeFor`, `formatSeconds`, `baseName`, `validateOdd` | `H:934-942`, `H:922-932`, `H:706-712`, `H:714-716`, `H:723-727` | none |
| `analyzeMotionRegions` | `H:944-1050` | take `dt` as an argument instead of `getParams()` (`H:946`) |
| parse logic of `parseTrajectoryFile` | `H:810-828` | split the text parser from the `File` API |

**View code, reusable with an ALIX restyle:** `buildTraces`/`plotTrajectory` (`H:831-870`), `attachOriginalPlotSelectionHandler` (`H:872-892`), the shape helpers (`H:1121-1142`,
`H:1286-1288`), the playback loops (`H:1176-1232`, `H:1290-1322`), the stepper (`H:609-704` + CSS `H:220-319`), and the tables and metric grids (`H:1057-1119`, `H:1242-1258`, `H:1324-1390`).
Split `processTrajectory` (`H:1676-1809`) into a pure `runPipeline(original, regions, stages, params) → {data, report}` plus UI glue.

**Move server-side (Python), shared with validation, merge and upload:**
- The **canonical processor** should be NumPy ports of trim/smooth/scale driven by a stored **recipe** (`{region_id, arm, source_sha256, stages, params, sub_regions[...]}`).
  The server re-computes from the recipe, stores the output and recipe with hashes, and serves R3 sim, R4 real test, R8 merge and R13 upload. The browser keeps the JS port only for instant preview,
  guarded by golden-file parity tests (inferred recommendation).
- **Validation** reuses ALIX: `canfd_preflight(rows_millideg, period_s, rate_gate)` (`TRK:3792-3864`, per-joint peak rate, utilisation, step > 10°, first offending line, minimum
  period); URDF limits (`URDF:201-430`); `deviation_mm` (`TT:607`) for TCP departure versus the raw recording; the duration envelope (`TT:116-122`).
- **Better algorithms already proven in ALIX** (offer as the default or "recommended"): `smooth_taught()` 5 Hz zero-phase Butterworth (`TT:894-913`) and `strip_pauses()`
  anchored pauses (`TT:915-953`). ALIX's full taught lane (filter, pause removal, arc-length smoothing, TOPP-RA at 0.95× ratings and 600 °/s², 6 Hz LP) is in `TPR:57`.
  Environment note: `scipy.signal` works in the user's normal env (scipy 1.17.1 in ~/.local with numpy 2.4.6) but **fails under `python3 -I`** (system scipy 1.11.4 vs numpy 2.4.6:
  "numpy.core.multiarray failed to import"). Pin a venv for the Lite server (measured).
- Browser-only pieces to drop in Lite: `saveBlobAs`/`saveContentAs`/`plotToPngBlob` (`H:1622-1674`). Results should save to the region slot through the Lite server, not to Downloads.

---

## Requirement relevance

- **INT-TRAJ**: this file *is* the integration source. Its compute core (`H:1417-1620`) and analysis (`H:934-1050`) are portable, and its UI patterns (stepper, tables, Plotly) can be reused after an ALIX restyle.
  Missing: arm/region identity, persistence, validation and simulation.
- **R2**: per-sub-region scale is present (`H:1392-1415`, `H:1568-1616`). Per-sub-region trim and smooth are **absent** (global params `H:729-737`). **Undo ×5 is absent**. Recommend extending sub-region objects to
  `{start,end,scalePct,smoothWindow?,trim?}` with blended boundaries, and an undo ring of the last 5 *recipe* snapshots (not data arrays). Lite "region" = one taught file (arm, task); HTML "region" = Lite sub-region.
- **R6**: Plotly interactions are present (`H:853-867`). Plot heights are fixed (`H:125-129`); no resizable windows and no linked zoom. Reuse the ALIX accessible separators `.dock-resize`
  (`WE/editor.css:333-341`, `initDockResize`/`setDockWidth` at `WE/editor.js:7580-7645`, persisted in localStorage) and add a plot-height handle.
- **R11**: there is a smoothing control (`smoothWindow`, `H:383`) but no immediate preview and no simulation. Pair it with the ALIX robot viewer (`applyRobotPose`, `WE/editor.js:12427`) and the shared transport
  (`renderPlaybackBlock`, `WE/editor.js:8749`). Show the deviation and the rating utilisation live.
- **R12**: absent. The building blocks are `step[]`/`accel[]` (`H:948-959`). Implement per-joint v and a against ratings and 600 °/s², flag step > 10°, and **specifically detect R-A1 endpoint snaps and R-A5 boundary jumps**.
  Each flag should carry a one-click fix (local smoothing, slow the sub-region by util/0.9, blend the boundary), and each fix should be undoable.
- **R3 / R5 / R10**: no simulation, collision or dual-arm support in the HTML. The 2-D cursor (`H:1134-1142`) maps naturally onto ALIX's Dual-Arm Collisions timeline (`dacRenderPlot`,
  `WE/editor.js:14478-14497`: red hit bars and click-to-scrub). The HTML disclaims collision safety (`H:436-439`).
- **R13**: indirect. The output is runner-compatible (`RUN:76-97`), and fewer rows mean a smaller project upload (FILE_01.1: 12429 → 9813 rows, −21 %, measured) (inferred: upload size ∝ bytes).
- **R8**: not in the HTML. Its `interpolateRegion` with pct > 100 is the same "stretch along the recorded path" V6 used for time matching (`V6/README.txt:7-8`) (inferred equivalence).
- **R1 / R4 / R7 / R9 / R14**: not covered by this file.
- **X1 / X2**: consistent. The HTML is joint-space only, with no geometric path, C-space or workspace features.
- **NFR-UX**: partially met (guided stepper `H:220-342`, statuses `H:99-110`). There is too much jargon (§7). Use presets, time-based selection in seconds, degrees, a traffic-light verdict and auto-save.
- **NFR-THEME**: mismatch (§8 table). Map `--accent → var(--brand)`, `--line → var(--border-strong)`, radii → `var(--radius)` 4 px, ok/warn/danger → ALIX values, and the dark header → ALIX top bar.
  Style charts like ALIX's (transparent background, grid `rgba(139,148,158,0.15)`, `WE/editor.js:5368`, `WE/editor.js:14483-14490`).
- **INT-CTRL**: file-level compatibility only (`RUN:76-89`). The duration shown should reflect the control app's overall speed % (`OPS:182-185`).

## Reuse verdicts

| feature | verdict | what to lift | evidence |
|---|---|---|---|
| NDJSON parser | reuse-with-adaptation | text parser; add integer and limit checks; keep skipped-line reporting | `H:803-829`; `RUN:76-89` |
| Idle trim | reuse-with-adaptation | `trimIdle` + `keptOriginalIndices` mapping; switch to or offer the anchored-pause rule | `H:1438-1492`; `TT:915-953` |
| Mean smoothing | reuse-with-adaptation | `centeredRollingMean`/`meanSmooth`; fix the endpoint snap; offer the ALIX Butterworth as the recommended option | `H:1417-1436`, `H:1494-1517`; `TT:894-913` |
| Per-sub-region scaling | reuse-with-adaptation | `mapRegionsToTrimmed`/`interpolateRegion`/`applyRegionScales`; add boundary blending and rating-aware clamping | `H:1537-1616` |
| Output TXT writer | reuse-as-is | `dataToTxt` | `H:1618-1620`; `RUN:76-97` |
| Processing report | reuse-with-adaptation | report schema; add hashes, arm, region, recipe and validation verdicts | `H:1732-1755` |
| Auto speed suggestions | reference-only | the idea and table UX; replace the positional and relative heuristics with rating-based analysis | `H:944-1050`, `H:1057-1174` |
| Guided stepper UX | reuse-with-adaptation | 6-step stepper, lock/done states, Back/Next notes; restyle with ALIX tokens | `H:220-342`, `H:609-704` |
| Status, hint and disclaimer idioms | reuse-with-adaptation | `setStatus` ok/warn/err, hints; plain-language copy | `H:90-110`, `H:718-721` |
| Plotly joint plots | reuse-with-adaptation | `buildTraces`/`plotTrajectory`; x in seconds, y in degrees; vendored Plotly; linked zoom | `H:831-870`; `WE/vendor/plotly.min.js`; `WE/editor.js:5322-5337` |
| Box-select to create sub-region | reuse-with-adaptation | `plotly_selected` → `addRegion`; draw the regions on the plot | `H:872-892`, `H:1392-1415` |
| 2-D playback cursor | reuse-with-adaptation | rAF loop and cursor shapes; drive the 3-D robot via `applyRobotPose`; ALIX transport block | `H:1176-1232`, `H:1286-1322`; `WE/editor.js:8749`, `WE/editor.js:12427` |
| Original vs optimised comparison | reuse-with-adaptation | metrics and region table; time-aligned x; real-duration envelope | `H:1260-1304`; `TT:116-122` |
| External optimised-file compare | reuse-with-adaptation | allow it without generating first | `H:2001-2028`, `H:618-623` |
| Save As (File System Access) | not-applicable | Lite saves to the region store on the server | `H:1622-1663` |
| Undo ×5 | not-available | build: recipe-snapshot ring buffer | grep: none |
| Per-sub-region trim/smooth | not-available | build: extend the region model | `H:729-737` |
| Spike analysis (R12) | not-available | build: reuse `step[]/accel[]` idea + `canfd_preflight` gate | `H:948-959`; `TRK:3792-3864` |
| Joint limit, velocity and acceleration validation | not-available | lift from ALIX (`canfd_preflight`, URDF limits, `deviation_mm`) | `TRK:3728-3864`; `URDF:201-430`; `TT:607` |
| Simulation / collision / dual-arm view | not-available | from ALIX Studio (DAC tab, robot viewer) | `WE/editor.js:12439-12450`, `WE/editor.js:14478-14497`; `WE/template.html:600-640` |
| Resizable panels (R6) | not-available | from ALIX `.dock-resize` | `WE/editor.css:333-341`; `WE/editor.js:7580-7645` |
| Theme tokens | reference-only | replace with ALIX `:root` tokens | `H:9-21` vs `WE/editor.css:22-128` |
| Plotly CDN load | reuse-with-adaptation | swap to vendored 2.35.2 + `ensurePlotly()` | `H:7`; `WE/template.html:7` |

## Appendix A — Event wiring (for porting)

`fileInput` change `H:1811` · `sampleMs` input `H:1871` · idle/smooth params input → `invalidateProcessed` `H:1888-1890` · stage checkboxes `H:1892-1900` ·
`addManualBtn` `H:1902` · `clearRegionsBtn` `H:1906` · `processBtn` `H:1912` · `downloadTxtBtn` `H:1914` · `downloadReportBtn` `H:1938` · `downloadPlotBtn` `H:1962` ·
`analyzeBtn` `H:1985` · `applySuggestionsBtn` `H:1986` · original transport `H:1988-1999` · `optimizedFileInput` `H:2001` · `useGeneratedBtn` `H:2019` ·
compare transport `H:2030-2041` · `resetBtn` `H:2043-2089` · init `installPipelineNavigation(); goToStep(1,false)` `H:2091-2092`.

## Appendix B — UI-state bugs found (for the porting backlog)

1. A region scale edit bypasses `invalidateProcessed()`, leaving the pipeline and comparison stale (`H:1359-1371`).
2. A `sampleMs` edit re-runs the analysis (losing suggestion edits) and nulls `processed` without a UI update or a `processingReport` reset (`H:1871-1886`).
3. Manual regions are not visualised on the plot (`H:1134-1142`).
4. External comparison is impossible before the first Generate (`H:618-623`, `H:537-548`).
5. Reset keeps custom parameter values (`H:2043-2089`).
6. `.plot.small` ignores the narrow-screen height (`H:129`, `H:312`).
7. `applySelectedSuggestions` bypasses the overlap and size validation (`H:1168`).
8. Durations use N×dt rather than (N−1)×dt (`H:1178`), and the time-to-row mapping mixes the two (`H:1181-1187`).
