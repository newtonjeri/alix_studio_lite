# Map 03 — ALIX Studio Regions / Board / Workbench / Schedule & Run, and how pole height + hand position are modelled

Scope: what ALIX Studio Lite can borrow for **R1** (region-wise data collection, Regions tab with the PPT, dashboard of regions with/without a taught raw .txt) and **R7** (per-region pole height and hand position before teaching), and where the **X1** (geometric path generation/editing) and **X2** (C-Space, Workspace) exclusions cut. Everything here was read, not run: no server was started, no robot was contacted.

## Abbreviations used in citations

| tag | absolute path |
|---|---|
| `TG` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen` |
| `WE` | `TG/web_editor` (`template.html` 953 lines, `editor.js` 19 325 lines, `editor.css` 2 166 lines) |
| `SRV` | `TG/server.py` (6 566 lines) |
| `ALIX` | `/home/newtonjeri/alix_ws/src/alix` |
| `CFG` | `ALIX/ros/orchestration/alix_tasks/config` |
| `ORCH` | `ALIX/ros/orchestration/alix_orchestrator` |
| `C_C` | `CFG/commode_cleaning/commode_c` |
| `LITE` | `/home/newtonjeri/alix_studio_lite_ws` |
| `UC` | `LITE/upload_channel` |
| `V53` | `UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py` (v48 has the same pole/hand/drag flags: `UC/rm_trajectory_ui_v48_handoff_continue_fix.py:4157,4194`) |
| `RUN14` | `UC/rm_upload_fast_v14_safe_transit.py` |
| `V6` | `LITE/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES` |
| `PPTX` | `/tmp/claude-1000/-home-newtonjeri-alix-studio-lite-ws/747fb97d-6769-47bf-aaab-d34d82139984/scratchpad/pptx/ppt` (unzipped `LITE/Toilet_regions_V1.pptx`) |

## 0. Headline answers

1. **Regions in Studio are a 3-D mesh-painting editor.** Regions are defined in `C_C/models/regions.yaml`, which the file itself calls the "SINGLE SOURCE OF TRUTH" (`C_C/models/regions.yaml:1-7`). The Regions tab paints labeled OBJ face groups onto the fixture meshes (`WE/template.html:156-201`). That painting feeds the geometric generator, so it belongs to X1. Lite's Regions tab should be a PPT and task registry, not a port of this tab.
2. **The Board tab is the closest Studio asset to the R1 dashboard.** It is a region × arm table with present/missing/not-applicable cells, summary chips, a tab badge and click-to-act cells (`SRV:5396-5486`, `WE/editor.js:14708-14757, 15208-15289, 15410-15467`). The UI pattern and CSS can be reused almost unchanged. The backend cannot: it reads task-config YAML, and its metrics (coverage and validation) depend on X1.
3. **Studio applies no pole height or hand position to the robot, before a teach or at any other time.** Studio has no execution channel (`WE/editor.js:13045-13047`), and it only reads the emulator, never writes to it (`TG/sim_bridge.py:10-21`; `SRV:5305-5306`). Here is what does exist in Studio:
   - Pole is chosen per task (region × arm) at generation time and written into the task config as a `move_pole_to_*` stage (`TG/stage_sequence.py:194-207, 713-719`).
   - A pole selector exists for the viewer and IK only (`WE/template.html:747-755`).
   - Hand state is *derived* from the glove `ik_frame` and is never chosen (`TG/config/glove_frames.yaml:55-60, 97-109`; `WE/editor.js:9959-9968`).
   - `regions.yaml` has **no pole key** and no hand key. It has only an optional `ik_frame` pad, which implies a hand state (`TG/region_mesh_loader.py:306-317`).
   
   The working R7 capability ("set pole and hand, then act") lives in two places: the **control app** (v48/v53 manual APPLY POLE / APPLY HAND and workflow `pole`/`hand` steps), and **ALIX tools** (`tools/taught_pipeline.py` `description.txt` POLE/HAND per recording, `tools/taught_tracks.py` `.SOURCE.json` provenance, `alix_dispatch/peripherals.py` `Pole.move` / `HandModbus.command`). See §3.8.
4. **Schedule & Run is an offline, whole-fixture playback and editing tool for the orchestrator's `schedule_<family>.csv`.** It plays MoveIt plan JSONs through FK. It never runs the robot ("offline preview (arms not connected)", `WE/template.html:889`). Its proportional timeline, transport and guard ideas are reusable for R10, R8 and R1 ordering. Its inputs (plan JSON plus CSV grammar) need an adapter for `.txt` tracks.
5. **Studio has no guided, wizard or timer flow.** The tabs are deliberately "not numbered" (`WE/template.html:116-118`). The only stepper in the Lite inputs is the trajectory HTML's 6-step pipeline (`LITE/Alix_Studio_Lite_Trajectory_V2.html:335-340, 609-616, 659`). R1's prompt → timer → auto-advance flow is new work.
6. **X1/X2 are large and separable.** Drop these:
   - Workbench (Generate dock plus point editing)
   - Board "Generate missing" and "Raster sweep"
   - Regions painting and "Check Arm Reachability"
   - Workspace tab and Gizmo tab
   - about 30 generator modules (§6)
   
   Keep the Dual-Arm Collisions tab, the Schedule & Run tab, the robot geometry overlay, `/api/plan-frames`, `/api/sim/*`, `/api/joint-presets` and `/api/hand-gizmo`.

---

## 1. Regions model

### 1.1 Where regions are defined (sources of truth)

- **Manifest per fixture:** `C_C/models/regions.yaml` (1 118 lines). Its header documents every key:
  - `groups`, `component`, `arm_coverage` R | L | RL_paired | RL_compound
  - `region_id`, `articulation_state`, `lid_state` / `seat_state`, `requires_lid` / `requires_seat`
  - `ugv_standoff_m`, `surface_type`, `spacing_m`, `generate`, `path_topology`, `extra_collisions`

  (`C_C/models/regions.yaml:16-82`). Labeled meshes: `labeled_meshes: {body: body_regions.obj, lid: lid_regions.obj, seat: seat_regions.obj}` (`:84-87`).
- **Corrected copy:** `C_C/models/corrected/regions.yaml` (1 180 lines) adds `coverage_target`, `singular_margin_deg`, `clearance_floor_mm`, `seam_intersection_m` and **`ik_frame`** (pad) on two regions: `hinge_area: glove_frame_4` (`:349`) and `bowl_inside_rim: glove_frame_3` (`:515`). The loader reads the corrected copy when it is fresh (`TG/region_mesh_loader.py:356-371` `manifest_source`; `:216-255` stale refusal).
- **Spec dataclass:** `RegionMeshSpec` (`TG/region_mesh_loader.py:265-419`). It holds every manifest key, including `ik_frame` (`:306-317`, "THE PAD this region is cleaned with … no rung second-guesses it"), `part_states`, `ugv_standoff_m`, `pre_start_retract_m`, `coverage_pass_pct`, `generate` and `path_topology`. The loader is `load_region_manifest` (`:702`).
- **Derived registry CSV:** `ORCH/config/regions_commode.csv` (`region_id, region_key, arm_coverage, requires_lid, requires_seat, description`, `:37-60`). It contributes the stable `region_id` that schedules use and holds the mechanism pseudo-regions `lid_open` (2), `seat_open` (5) and `lid_close` (12) (`TG/regions.py:1-8, 158-188`). The server re-syncs it on every region save (`SRV:4116-4125`: write manifest, `refresh_region_corrections`, `sync_regions_csv_from_manifest`).
- **Metadata resolver:** `get_region_info` is manifest-first; the CSV adds `region_id` only (`TG/regions.py:97-151`).
- **Physics rules:** `reality_required_states` (bowl_inside → lid+seat open; seat_ring → lid open, seat closed; bottomlid → lid open; toplid → lid closed) (`TG/regions.py:26-57`). Effective fixture state comes from `fixture_states_for_region` / `fixture_type_for_states` → `closed | lid_open | lid_seat_open` (`TG/region_component_resolver.py:133-168`).
- **HTTP:**
  - `GET /api/regions?fixture=` → `list_regions` (`SRV:259-308`, route `SRV:5970`). Payload per region: `key, provenance (labeled|family|mechanism|needs-labeling|unlabeled-here|other-fixture), offered, component(s), regionId, armCoverage, description, labeled, groups, groupedComponents, articulationState, surfaceType, states, lidState, seatState, fixtureType, explicitStates, ugvStandoffM` (`SRV:279-307`). The offering rule is `_region_offered` (`SRV:210-256`).
  - `POST /api/update-region` → `update_region` (`SRV:4030-4129`). Its editable whitelist is `arm_coverage, description, surface_type, spacing_m, articulation_state, region_id, requires_lid, requires_seat, lid_state, seat_state, part_states, arm_overlap, ugv_standoff_m` (`SRV:4071-4079`). **There is no pole, hand or `ik_frame` field**, so the pad can only be declared by editing the manifest by hand.
  - `GET /api/manifest` → fixture meshes and paint sources (`SRV:4132-4139`).

### 1.2 commode_c region inventory

Extracted from `C_C/models/regions.yaml` (line = region key line). There are 21 entries, 15 generated plus 6 with `generate: false`.

| key | id | comp | arm_coverage | generate | lid/seat | standoff m | pad (corrected) | line |
|---|---|---|---|---|---|---|---|---|
| front | 11 | body | RL_paired | yes | – | 0.38 | – | 91 |
| side | 1 | body | RL_paired | yes | – | – | – | 130 |
| top (bowl rim top) | 6 | body | RL_paired | yes | open/open | – | – | 211 |
| hinge_area | 13 | body (+lid strip) | RL_paired | yes | – | – | glove_frame_4 | 294 |
| bowl_inside_front | 7 | body | R | yes | open/open | – | – | 334 |
| bowl_inside_back | 8 | body | L | yes | open/open | – | – | 360 |
| bowl_inside_ring | 9 | body | RL_paired | yes | open/open | – | – | 387 |
| bowl_inside_rim | 15 | body | RL_paired | yes | open/open | – | glove_frame_3 | 427 |
| deep | 20 | body | RL_paired | **false** | open/open | – | – | 479 |
| toplid | 0 | lid | RL_paired | yes | – | – | – | 523 |
| bottomlid | 10 | lid | RL_paired | yes | open/closed | – | – | 563 |
| lid_side | 21 | lid (+seat) | RL_paired | yes | – | – | – | 634 |
| lidsides_back | 22 | lid | RL_paired | **false** | – | – | – | 704 |
| lidsides_front | 23 | lid | RL_paired | **false** | – | 0.38 | – | 717 |
| seat_ring | 3 | seat | RL_paired | yes | open/closed | – | – | 738 |
| seat_ring_bottom | 4 | seat | RL_paired | yes | open/open | – | – | 772 |
| seat_ring_sides | 18 | seat | RL_paired | **false** | open/closed | – | – | 804 |
| seat_ring_sides_front | 19 | seat | RL_paired | **false** | open/closed | 0.38 | – | 872 |
| lid_seat_front | 34 | lid (+seat) | R | yes | closed/closed | 0.38 | – | 901 |
| lid_seat_hinge_area | 24 | lid (+seat) | RL_paired | yes | open/closed | – | – | 969 |
| seat_body_hinge_area | 25 | body (+lid+seat) | RL_paired | yes | open/open | – | – | 1015 |

The Board's rules give 29 expected configs for commode_c (13 paired × 2 + 3 single-arm), and all 29 `*_cleaning_points.yaml` exist in `C_C/` (re-computed offline with the same rule as `SRV:5440-5476`; no server was run).

### 1.3 How the Regions tab displays regions

- **Ribbon panel** "Region Labels — step 1" (`WE/template.html:156-201`, h2 at `:158`):
  - fixture select and per-part state selects (`#paintStateControls`, `:160-170`)
  - Paint / Erase tools and brush radius (`:173-182`)
  - Import OBJ / Export OBJ (`:186-187`)
  - **Check Arm Reachability** (`#regionReachButton`, `:188`, workspace analysis = X2)
  - **Combine Regions…** (`:189`, popover `:194-199`)
- **Region list in the Browser dock** (`WE/template.html:783-806`):
  - `#paintRegionList`, an "add region" input
  - "Paintable parts" (`#paintActiveParts`, `:797-799`)
  - per-region UGV "Standoff (m)" (`#paintUgvStandoff`, `:800-803`)
- **List rendering** (`WE/editor.js:2778-2860`, `updatePaintPanel`). Each item shows:
  - a colour swatch (`regionColor` = golden-angle HSL, `:2243-2245`)
  - the name, plus tags for component span (`body+seat`) and open parts ("lid open")
  - a face-count badge (`.paint-region-count`)
  - rename ✎ and delete ✕ buttons
  - a tooltip "N faces (per component) — cleaned with lid open, seat closed" (`:2811-2830`)
  
  The stats line reads "N faces across k component(s), m unassigned — unsaved paint edits" (`:2846-2853`).
- **3-D view:** the paint meshes are posed in the *active region's* cleaning state (lid drawn open, and so on) by `applyPaintArticulation` (`:2863-2881`). State controls are `paintRegionStates` / `updatePaintRegionStateControls` / `onPaintRegionStateChange` (`:2891-3001`).
- **Status and badges:** there is no "taught / not taught" or "config present" status on the Regions tab; that lives on the Board. The legend reads "unpainted faces — not in any region"; "painted faces take the region's list colour"; arm reachability per region (`WE/editor.js:7212-7216`).
- **Tab entry** forces Paint mode, or shows "Region marking needs component meshes…" (`WE/editor.js:7419-7422`).
- **Generate options:** each option shows `key — armCoverage • comps • lid open • labeled • guessed` (`WE/editor.js:9368-9385`).
- **Sub-regions:** in Studio a "region" may hold several labeled `groups`, "planned per group, chained into one task". Combine creates such a region through `/api/update-region` with `groups=[{component,name}…]` (`WE/editor.js:3213-3256`; manifest doc `C_C/models/regions.yaml:17-22`).

### 1.4 The Lite region taxonomy (PPT) vs Studio's commode_c keys

**PPT structure** (from slide XML text positions; `PPTX/slides/slide*.xml`):
- **S1–S2 (PAGE 02/03):** fixture *states*. Lid cover closed, lid cover open, seat cover open (top views); bowl outer side and front. Base photos are `media/image1-5.png` (1448×1086).
- **S3 (PAGE 04):** hand-hatched top-view photo `media/image6.jpeg` with 9 label images: Lid Top, Lid Underside, Seat Underside, Seat Top, Rim Top, Inner Rim, Bowl Inside, Drain Hole, Hinge (`media/image7-15.png`).
- **S4 (PAGE 05):** side/front photo `media/image16.jpeg`, labels Side Exterior, Pedestal Upper, Base (`media/image17-19.png`). So the deck names **12 surface regions plus Floor Spillage**.
- **S5–S25:** 22 task cards (1A … 15), each with a LEFT ARM label at x≈2–4 in and a RIGHT ARM label at x≈8–10 in. The task text is placed beside the arm it belongs to. The cards reuse the same 5 base photos with a few rectangle or arrow overlays, so **regions are indicated by label position, not by per-task masks**.

**Arm assignment** (x position; S11 and S15 by vertical proximity, inferred):

| slide | task | LEFT arm | RIGHT arm | kind |
|---|---|---|---|---|
| 5 | 1A | L-R01 BASE LEFT | R-R01 LID TOP PART 01 | clean / clean |
| 6 | 2A | L-R02 SIDE EXTERIOR LEFT | R-R01 LID TOP PART 02 | clean / clean |
| 7 | 1C | L-R03 PEDESTAL UPPER LEFT | R-R02 RIGHT HINGE | clean / clean |
| 8 | 2 | LEFT HINGE | SIDE EXTERIOR RIGHT | clean / clean |
| 9 | 3 | OPEN LID + SEAT (MASTER) | OPEN LID + SEAT (SUPPORT) | mechanism |
| 10 | 4 | CLOSE SEAT ONLY (MASTER) | CLOSE SEAT ONLY SUPPORT | mechanism |
| 11 | 5A | SEAT TOP PART 1 (inferred) | LID UNDERSIDE RIGHT HALF (inferred) | clean / clean |
| 12 | 5B | SEAT TOP PART 2 | PEDESTAL UPPER RIGHT | clean / clean |
| 13 | 6 | LID UNDERSIDE LEFT HALF | BASE RIGHT | clean / clean |
| 14 | 7 | L-T03 OPEN SEAT (MASTER) | R-T03 OPEN SEAT (SUPPORT) | mechanism |
| 15 | 8A | RIM TOP PART 1 (inferred) | SEAT UNDERSIDE RIGHT HALF (inferred) | clean / clean |
| 16 | 8B | RIM TOP PART 2 | FLOOR SPILLAGE PART 1 | clean / clean |
| 17 | 9 | SEAT UNDERSIDE LEFT HALF | FLOOR SPILLAGE PART 2 | clean / clean |
| 18 | 10A | FL-R09 INNER RIM PART 1 | R-T04 PICK DRAIN BRUSH | clean / tool |
| 19 | 10B | FL-R09 INNER RIM PART 2 | R-T04 PICK DRAIN BRUSH (repeated) | clean / tool |
| 20 | 11A | L-R10 BOWL INSIDE PART 1 | HOLD BRUSH OUTSIDE | clean / tool |
| 21 | 11B | L-R10 BOWL INSIDE PART 2 | HOLD BRUSH OUTSIDE | clean / tool |
| 22 | 12A | L-R11 FLOOR SPILLAGE LEFT PART 1 | R-T05 CLEAN DRAIN HOLE PART1 | clean / clean (brush) |
| 23 | 12B | L-R11 FLOOR SPILLAGE LEFT PART 2 | R-T05 CLEAN DRAIN HOLE PART 2 | clean / clean (brush) |
| 24 | 13 | DROP GLOVE | R-T06 RETURN DRAIN BRUSH AND DROP GLOVE | tool |
| 24 | 14 | L-T04 LEFT GLOVE CHANGE | R-T07 RIGHT GLOVE CHANGE | tool |
| 25 | 15 | L-T05 CLOSE SEAT + LID (MASTER) | R-T09 CLOSE SEAT + LID (SUPPORT) | mechanism |

Totals: LEFT has 16 cleaning recordings, 4 mechanism masters and 2 tool tasks. RIGHT has 12 cleaning recordings (including drain hole), 4 mechanism supports and 4–6 tool tasks. **IDs are inconsistent:** many labels carry no code (`LEFT HINGE`, `SEAT TOP PART 1` …), `R-R01` labels both LID TOP PART 01 and PART 02, and the prefix `FL-` (FL-R09) is unexplained. Lite needs a canonical ID scheme, for example `<task>-<arm>-<region>-<part>`.

**PPT → Studio key mapping** (best effort; the names differ, so this is inferred):

| PPT region | Studio commode_c key(s) |
|---|---|
| Lid Top (PART 01/02) | toplid (0) |
| Lid Underside (R/L half) | bottomlid (10). PPT halves = Studio RL_paired split |
| Seat Top (P1/P2) | seat_ring (3) |
| Seat Underside (R/L half) | seat_ring_bottom (4) |
| Rim Top (P1/P2) | top (6) |
| Inner Rim (P1/P2) | bowl_inside_rim (15) / bowl_inside_ring (9) |
| Bowl Inside (P1/P2) | bowl_inside_front (7), bowl_inside_back (8), bowl_inside_ring (9) |
| Hinge (L/R) | hinge_area (13), lid_seat_hinge_area (24), seat_body_hinge_area (25) |
| Side Exterior (L/R) | side (1) |
| Pedestal Upper, Base, Drain Hole, Floor Spillage | **no Studio region** |
| Open lid+seat / open seat / close seat / close seat+lid | `CFG/mechanism_tasks/commode_c/{lid_open, seat_open, seat_close_left/right, lid_close_left/right}.yaml`; CSV ids 2, 5, 12 |
| Glove drop / change | `CFG/glove_tasks/*` plus session phases `glove_remove_*` / `glove_wear_*` (`ORCH/alix_orchestrator/core/session_phases.py:87-206`) |
| Drain brush pick / hold / return | **none** |
| (Studio only) | front (11), lid_side (21) and lid_seat_front (34) have no PPT card |

**PPT fixture states ↔ Studio fixture types:** lid closed / lid open / seat open ↔ `closed | lid_open | lid_seat_open` (`TG/region_component_resolver.py:159-168`). The lid and seat joints of commode_c are `closed 0°, open −110°` (`C_C/scene.yaml:48-67`).

**Evidence that the Lite toilet is commode C:** `ALIX/experiments/exp01_teach_trajectory_reverse_engineering/data/teach_files/commode_c_taught_paths/description.txt:1` reads "LEFT ARM (192.168.1.10) - commode C". Its tasks (LID TOP, BOWL OUTSIDE, SEAT TOP, RIM TOP, BOWL INSIDE, RIM SIDE, BOWL INNER BACK, HINGE AREA, LID SEAT OPEN, SEAT ONLY CLOSE) use PPT-style names (`:3-16`). The same IP is the Lite LEFT controller.

**Terminology collision (R1/R2):** the trajectory HTML uses "region" for a **time window of one trajectory** ("Scale *selected-region* resampling", manual scaling regions table `LITE/Alix_Studio_Lite_Trajectory_V2.html:392-400, 469-516`). The requirements use "region" for a **cleaning area** of the toilet. Lite should use "Area / Sub-area" for cleaning regions and "Segment" for time windows, or the reverse, but consistently.

---

## 2. Board tab ("Fixture Board")

### 2.1 What it shows

- **Markup:** `WE/template.html:529-553`. The comment explains the purpose: "The operator's unit of work is one region x one arm -> one config file … Deliberately shows ONLY what is on disk" (`:529-536`). Buttons:
  - `#boardRefresh`
  - `#boardGenerateMissing` (C-space bulk job, **X1**)
  - `#boardBulkCancel`
  - `#boardRasterSweep` (legacy raster, **X1**)
  
  Containers are `#boardSummary`, `#boardTable` and `#boardBatch` (`:543-552`).
- **Backend:** `GET /api/fixture-board?fixture=` → `do_fixture_board` (`SRV:5396-5486`). It reads `models/regions.yaml` and, for each region × arm, checks `config/<family>_cleaning/<fixture>/<region>_<arm>_cleaning_points.yaml` (`TG/regions.py:154-155` `output_filename`).
  - Arm applicability comes from `arm_coverage`. `generate:false` and `RL_compound` mark a cell not applicable, with a `reason` (`SRV:5438-5455`).
  - Cell payload: `{applicable, exists, points, path, mtimeNs, producedBy:{route: cspace-pipeline|raster-legacy|hand-authored|unknown, strokeGateOk}}` (`SRV:5455-5476`, `_produced_by` `SRV:5489-5512`).
  - Summary: `{regions, expected, present, missing}` (`SRV:5482-5486`).
  - While a generation holds the lock it refuses with `generationBusy` instead of reporting blanks (`SRV:5414-5422`).
- **Slow metrics:** `GET /api/board-cell?fixture&region&arm=right|left|pair` → `do_board_cell` (`SRV:5562-5636`) returns `validationPass/checksFailed` (`validate_config_only`), `coveragePct`, `passesPerPoint` and the `coverageTarget` block. The paired union is `_board_pair_cell` (`SRV:5648-5773`). Results are mtime-cached, with the key including region-input signatures (`_BOARD_CELL_CACHE` `SRV:5537`, `_region_inputs_signature` `SRV:5540-5560`, key `SRV:5593`).
- **Front end:**
  - `loadBoard` (`WE/editor.js:14708-14757`) renders summary **chips** "fixture, regions, configs present/expected, missing | complete ✓, paired, split, single-arm" (`:14733-14750`) and the **tab badge** `present/expected` (`:14754-14755`).
  - `renderBoardRows` (`:15208-15289`) builds the columns `region | component | right | left | pair | updated | description`. Cell states:
    - `data-state="present"`: pill "✓ N pts", route pill "C-space · gate ✓", async metrics slot
    - `data-state="missing"`: pill "· generate"
    - `data-state="na"`: "—", tooltip with the reason
  - "updated" is relative age ("just now / N min ago / N h ago").
  - `fillBoardMetrics` fills cells through 2 parallel lanes and stops on busy (`:15291-15326`). `renderCellMetrics` / `renderPairMetrics` draw pills for coverage %, passes ×, validation ✓/⚠ and union ∪ (`:15328-15408`).
  - `initBoard`: **clicking a cell opens the Workbench** bound to that region/arm (`:15410-15467`, click handler `:15454-15466`; `openWorkbench` `:14804-14809`).
- **CSS:** `editor.css:1507-1524` (table, sticky header, cell state colours `--ok`, `--warning`, `--muted`), `:1572-1574` (batch), `:1731-1750` (the Board takes the whole main area; the viewport is hidden), `:1974-1999` (chips, zebra rows, `.pill`, `.pill-ok`, `.pill-missing`, `.pill-na`, `.pill-warn`, `.pill-route`, `.board-age`). The tab badge is `.tab-badge` at `:832-838`.

### 2.2 Could it be the R1 dashboard ("regions with and without a taught raw .txt")?

**Yes, as UI and pattern. No, as backend.** The table, chips, tab badge, three cell states and click-to-act already express "which units of work are done". Changes Lite needs:

| Board today | Lite R1 dashboard |
|---|---|
| Rows = `models/regions.yaml` regions | Rows = Lite task registry built from the PPT (task 1A…15 × sub-area), in schedule order |
| Unit = region × arm → config YAML on disk | Unit = (task, arm, sub-area) → raw `.txt` on disk (plus optional optimized `.txt` and upload state) |
| present: "✓ N pts" + producer + gate | taught: "✓ N pts · 52.1 s @5 ms · taught 12 min ago". Optional pipeline chips: Optimized ✓ / Simulated ✓ / Real-tested ✓ / Uploaded #71 |
| missing: "· generate" → Workbench | not taught: "● Teach now" → opens the guided teach prompt (R1 flow) |
| na: arm not declared | na: the other arm has no task on this card (e.g. tool-only card) |
| metrics: coverage %, validation (X1) | metrics: points, duration, idle %, max per-point Δ° (as in `V6/FINAL_FILE_RUNTIME_REPORT.csv` columns `points, runtime_s_at_5ms, max_per_point_joint_change_deg`), pole/hand provenance present ✓ |
| `Generate missing` (bulk C-space job) | "Teach all missing" = guided sequence over the missing cells (R1) |
| mtime-keyed cache | the same idea; scanning a folder is cheap, so no cache is needed |

**Naming gap:** the control app saves drag recordings as `~/Downloads/RealMan_DragTeach/<side>_drag_<YYYYmmdd_HHMMSS>.txt` (`V53:3840-3850`), with no task or region token. The runner calls only `arm.rm_save_trajectory(path)` (`RUN14:957-988`). Lite must choose the filename, e.g. `<raw_dir>/<task>_<arm>_<area>[_<part>].txt`, and write a sidecar (§3.10) so the dashboard can find files.

---

## 3. Pole height and hand position — what Studio actually models

### 3.1 Units, ranges, names

- **SRDF pole rungs** (metres, per arm and `dual_poles`): `full_length 0.30, three_quarter_length 0.225, half_length 0.15, quarter_length 0.075, minimum_length 0.01` (`ALIX/ros/manipulation/alix_moveit_config/config/group_states/poles_group_states.yaml:11-63`). Joint: `<L|R>_sliding_plate_joint`, prismatic `lower=0.0 upper=0.315 velocity=0.16` (`ALIX/ros/description/pole_description/urdf/pole.xacro:56-63`).
- **Stage names:** `POLE_STAGE_BY_POSE = {minimum_length: move_pole_to_minimum, quarter_length: move_pole_to_quarter_height, half_length: move_pole_to_half_height, full_length: move_pole_to_max_height, custom_length: move_pole_to_custom}`. There is no three-quarter stage (`TG/stage_sequence.py:194-207`).
- **Custom height:** `CUSTOM_POLE_STATE = "custom_length"`, with the metres written to `joint_space_poses.pole_<arm>.custom_length: [m]`, and `POLE_RANGE_M = (0.0, 0.315)` (`TG/stage_sequence.py:181-192`). The C++ `TaskBase::addPoleStage` reads `joint_space_poses` **before** the SRDF. A missing row is fail-open (the pole stays where it was) and is logged at ERROR (`ALIX/core/python/alix_taskgen/test_generate/test_custom_pole_height.py:1-17, 66-76`).
- **Hand:** SRDF group `inspire_hand_<arm>` states (`open_tenth, half_grasp, quarter_grasp, grasp, release` …). The glove frame chooses the state: frames 1 and 2 → `open_tenth`, frame 3 → `half_grasp`, frame 4 → `quarter_grasp` (`TG/config/glove_frames.yaml:55-60, 97-109`; resolver `TG/glove_frames.py:142-168`).

### 3.2 How a task config carries pole and hand

There is no pole key in `task_parameters` (keys of `C_C/toplid_right_cleaning_points.yaml`: `commode_fixture_type, enable_washroom_structure, ik_frame, max_*_scaling, primary_planning_group, reference_frame, stroke_swivel_mode, task_type, ugv_standoff_m, use_prestart_stage`). Both pole and hand are **stages** in `stage_sequence`:

| config | hand stage | pole stage |
|---|---|---|
| `C_C/toplid_right_cleaning_points.yaml` | `{stage: hand_grasp, pose_name: open_tenth, planning_group: inspire_hand_right}` (open and close) | `{stage: move_pole_to_quarter_height, pose_name: quarter_length, planning_group: pole_right}` (around `:1754-1759`) |
| `C_C/bowl_inside_rim_left_cleaning_points.yaml` | `half_grasp` (ik_frame `L_glove_frame_3`) | `move_pole_to_max_height / full_length` |
| `C_C/front_right_cleaning_points.yaml` | `open_tenth` | quarter |

`build_stage_sequence(arm, pole_pose="quarter_length", …, restore_pole_pose=None, end_hand_stage="hand_grasp", open_with_hand_release=False, rest_before_approach=False, ik_frames=…)` (`TG/stage_sequence.py:477-849`):
- The hand pose comes from the first ik_frame's `hand_state` (`:538-542`) and is emitted **before** the pole move (`:675-681`).
- The pole stage is at `:713-719`. An optional teardown pole restore is at `:824-829`, and the end hand stage at `:830-849`.
- The docstring records corpus statistics: 26/32 quarter, 6/32 half, 4/32 max, and a `hand_release` lead in 12/32 (`:9-21`).

### 3.3 How the pole is chosen when generating (Workbench, X1)

- **UI:** the "Pole (written to the task)" select `#genPoleState` offers `(auto) | quarter_length | half_length | three_quarter_length | full_length | custom height (m)…`. `#genPoleM` is a number field 0–0.315 m, and `#genPoleBandWarn` is the band warning (`WE/template.html:235-238`). `genPoleIsCustom` / `genRenderPoleFields` show the metres field and warn about the stowed-bin foul band (`WE/editor.js:17161-17187`). `genPolicyFromPanel` sends **either** `poleM` **or** `poleState` (`:17189-17200`).
- **Server policy:** `poleState → pole_state`, `poleM → pole_m` (`TG/studio_generate.py:176, 232, 270-271, 288`). Naming both is refused ("named twice", `_check_pole_policy` `TG/studio_generate.py:503`; test `test_custom_pole_height.py:121-138`).
- **Defaults when not given:**
  - archive seed pole (`TG/scripts/generate_cleaning_config.py:553-572`)
  - else the height rule F74: highest point at base z > 0.85 → full, > 0.70 → half, else quarter (`:1474-1488`; CLI `--pole-state` `:1811-1814`)
  - the C-space lane records who chose it as `g1_pole_source ∈ {height rule, operator: named state, operator: explicit metres}` (`ALIX/core/python/alix_taskgen/test_generate/test_pipeline_pole_and_idle_hand.py:1-20, 38-79`; `TG/coverable_map.py:674, 987`)
  - unknown rungs or out-of-range metres raise `PoleStateError` (`TG/cspace_author.py:8829`)

### 3.4 Pole in the viewer and IK only

- The Browser dock panel "Tool Frame & Pole" (tabs regions/workbench/workspace/collisions/schedule) holds:
  - `#reachFrameSelect` (IK frame)
  - `#reachHandState` with the mismatch flag `#handStateMismatch`
  - `#poleStateSelect`, titled "Pin the pole/sliding-plate to this SRDF length; IK then solves only the arm joints. Bin is always LOWERED. Shared across all tabs."
  
  (`WE/template.html:747-755`)
- `populatePoleStates` fills it from `GET /api/joint-presets` (`WE/editor.js:12200-12215`). `do_joint_presets` returns `poleStates [{name,value}]` (longest first), `default`, and `armBaseFrames` (a per-pole-state table of arm Base/World frames, because "the arm base rides the pole") (`SRV:1763-1861`).
- `applyToolFramePanelFromConfig` sets the selector from a loaded config's first pole stage (`WE/editor.js:5766-5780`). `poleStateValue()` feeds IK, analysis and overlay calls (`WE/editor.js:12191`; uses at `:6827, 12061, 14656, 15962`).
- The Redundancy Atlas has its own "Pole m" input (`WE/template.html:574`, X2).

### 3.5 Hand in Studio: derived, displayed, never commanded

- "It is derived, never chosen: … offering it as an editable control would be offering a value nothing downstream reads" (`WE/editor.js:9959-9968`). `renderHandStates` writes the read-only `#genHandState` / `#reachHandState` and flags viewer-vs-task disagreement (`:9970-9995`). `syncViewerToTask` copies the task frame and pole into the viewer (`:9916-9950`).
- **Hand gizmo:** `GET /api/hand-gizmo?frame=` returns every hand link pose in the ik_frame, posed at the frame's SRDF hand_state, plus the pad (`SRV:3376-3450`). It is drawn by `handGizmoFor` (`WE/editor.js:1413`). `#startGizmoMode` = "Hand + pad" / "Pad only" (`WE/template.html:713`).
- **Idle partner hand in the collision world:** `IDLE_PARTNER_HAND_STATE = "grasp"`, with envelopes for `("grasp","release")` grown by 5 mm (`TG/body_collision.py:71-83`; tests `test_pipeline_pole_and_idle_hand.py:84-130`).
- **Region-level pad:** the manifest `ik_frame` (glove_frame_N) selects the pad, and therefore the hand state, for that region (`TG/region_mesh_loader.py:306-317, 442-460`). Only 2 commode_c regions declare it (§1.1).

### 3.6 Safety constraint Lite must keep (R7, R10)

**Stowed-bin pole foul band.** With the bin STOWED, the arm base fouls the bin lid covers anywhere in 0.158–0.277 m. With a 5 mm margin the band is **0.153–0.282 m** (worst −61.8 mm). "NO ARM MOTION AVOIDS THIS".
- Source: `ORCH/alix_orchestrator/core/task_config.py:58-95`
- `pole_bin_conflict(bin_state, pole_joint)`: `:97-124`
- Copies: `TG/studio_generate.py:475-500` and `WE/editor.js:17149`
- Panel warning text: `WE/editor.js:17178-17186`

The Lite V6 workflow starts both arms with `{"type":"pole","mm":200.0}` (`V6/LEFT_V6_70SERIES_FINAL.json` steps[0], `V6/RIGHT_V6_70SERIES_FINAL.json` steps[0]). 200 mm is **inside** that band, and the exp01 taught table uses 200 mm for six commode C recordings (`…/commode_c_taught_paths/description.txt:9-14`). That is only safe with the bin deployed (EXTENDED/LOWERED). Lite's R7 pole control should carry this warning and the bin-state question (inferred: the Lite runs may be done with the bin deployed or removed; confirm).

### 3.7 Verdict: is there a per-region setting that Studio applies BEFORE a teach?

**No.**
1. Studio has **no robot command path**. Schedule & Run's Execution Status says "HONEST: the studio has no execution channel — the orchestrator runs sessions ROS-side and nothing here commands hardware" (`WE/editor.js:13045-13047`). The emulator bridge "reads, never writes" (decision D4, `TG/sim_bridge.py:10-21`; `SRV:5305-5306`). `POST /api/sim/joints` only *accepts* an external joint map for display (`SRV:5326-5341`).
2. The **region** model (`regions.yaml`) has no pole or hand field. The keys of all 21 commode_c entries are listed in §1.2. The `update_region` whitelist has none (`SRV:4071-4079`).
3. Pole and hand are **task-config stages** (per region × arm) that the **ROS runtime** executes at run time, not at teach time (§3.2).
4. Studio has no drag-teach concept at all.

So R7's "borrow from ALIX Studio" can borrow these, but not an "apply-to-robot" feature:
- the vocabulary (SRDF rungs, custom metres, 0–0.315 m, the hand state per pad)
- the visualisation (robot overlay with the pole joint and finger joints; hand gizmo)
- the foul-band warning

### 3.8 Where "set pole and hand, then act" really exists (cross-repo evidence)

- **Control app (INT-CTRL):**
  - Per-arm manual **"POLE // 0 mm → 315 mm"** slider plus **APPLY POLE** → runner `--manual-pole <mm> --side <arm> --yes` (`V53:3021, 3102, 4176-4205`).
  - **"APPLY HAND // ALL 6 CHANNELS"** → `--manual-hand-raw v1..v6 --side --yes` (`V53:2952, 4142-4174`), with presets **GRIP [0,0,0,0,150,1000]** and **ALL OPEN [1000×6]** (`V53:2982, 2997, 4127-4140`). Channel order is "Little, Ring, Middle, Index, Thumb Bend, Thumb Rotate" (`V53:4129-4131`; labels `L,R,M,I,TB,TR` `V53:6348`).
  - Workflow-builder steps `{"type":"pole","mm"}` (0–315 mm, `V53:6604-6618`), `{"type":"hand","values":[6×0-1000]}` (`V53:6640-6656`) and `wait`. The program table has editable `hand`, `pole` and `wait` columns (`V53:982, 1118-1137`).
  - The same flags exist in v48 (`UC/rm_trajectory_ui_v48_handoff_continue_fix.py:4157, 4194`).
- **V6 package:** the per-arm sequence JSON (`format: realman_workflow_sequence_v1`) interleaves `pole` and `hand` steps before programs. LEFT: pole 200 → hand GRIP → pole 150 → P71 → pole 50 → P73 → pole 150 → wait 2 s "MANUAL LID COVER OPEN" → P76 … (`V6/LEFT_V6_70SERIES_FINAL.json`). The README gives a pole timing model of "10 mm → 300 mm takes 4.0 s … 72.5 mm/s … 1.380 s" (`V6/README.txt:10-15`) and "LID OPEN MANUALLY. Then both poles move to 150 mm before Flow 6" (`:28-31`).
- **ALIX taught-track lane:**
  - `tools/taught_pipeline.py` reads a folder `description.txt` with rows `FILE  TASK  <n>mm  <n>% open  [MAX DEV]` (`ALIX/tools/taught_pipeline.py:11-27`; strict parser `ROW_RE`/`parse_description` `:183-221`).
  - It copies each recording with a `.SOURCE.json` (arm, pad frame, pole, hand `fingers_<pct>pct`, `hardware: ok`, sha256) (`:36-45`).
  - At run time it sets **"pole, then hand" before the arm moves** (`ALIX/docs/motion/TAUGHT_PIPELINE_README.md:93-100, 114`) through `alix_dispatch/peripherals.py` `Pole.move` (`ALIX/core/python/alix_dispatch/src/alix_dispatch/peripherals.py:169, 227`) and `HandModbus.command` (`:462, 601`), with SRDF radians → counts by `counts_from_rad` (`:751-770`).
  - `tools/taught_tracks.py` refuses to certify or replay without provenance for `side, hardware, pole, hand`, asks once and stores it (`ALIX/tools/taught_tracks.py:309-340, 3531-3560, 3624-3634`). Flags: `--taught-here --pole-mm N --hand-state NAME` (`:4433-4441, 4537-4541`).
- **Real per-region values (commode C, LEFT arm):** `…/teach_files/commode_c_taught_paths/description.txt:3-16`:
  - LID TOP / BOWL OUTSIDE / SEAT TOP / RIM TOP: **140 mm, 100% open**
  - BOWL INSIDE: **200 mm, 100% open**
  - RIM SIDE / BOWL INNER BACK / HINGE AREA / LID SEAT OPEN / SEAT ONLY CLOSE: **200 mm, 50% open**
  - plus MAX DEV 5–20 mm on some rows
  
  The exp01 manifest explains why pole provenance matters: "The replay MUST stage the pole here first: a joint-space track is pole-agnostic, so a quarter-height pole sweeps the whole … path 225 mm low" (`…/teach_files/MANIFEST.yaml:136-139`).

### 3.9 Unit conversions Lite will need (and a conflict)

- **Pole:** the control app uses mm (0–315); the URDF uses metres (`*_sliding_plate_joint`). The emulator bridge divides by 1000 (`TG/sim_bridge.py:165-177`).
- **Hand counts ↔ URDF radians.** Two **different** scales exist in ALIX:
  - `ALIX/ros/hw/alix_hw/scripts/hand_units.py:33-37` uses `MAX_RAD = (1.344, 1.344, 1.344, 1.344, 0.5236, 1.246165)` for `little, ring, middle, index, thumb_bend, thumb_rot` (`hw_to_rad`/`rad_to_hw` `:50-60`). It is the hardware authority, and `alix_dispatch` refuses to "invent a hand scale" without it (`peripherals.py:757-759`).
  - `TG/sim_bridge.py:95-105` (`HAND_CHANNELS`) uses 1.60×4, 0.70, 1.10 for the emulator view.
  
  In both, 1000 counts = open = 0 rad. Lite must pick `hand_units.py` (risk §Gaps).
- **Pole travel time model** (useful for R1/R10 timeline estimates): `LIFT_SLEW_UP_MPS 0.059`, `DOWN 0.097`, `LIFT_CREEP_S 1.5` (`peripherals.py:121-124`, copied in `UC/pole.py:3-14`).

### 3.10 Recommendation for Lite R7 (approach)

Give each Lite area record `pole_mm` and `hand_counts[6]` (plus a friendly preset label: GRIP, OPEN, "fingers 50% open"), pre-filled from `commode_c_taught_paths/description.txt` and the V6 JSON.

Before **Start teaching**, the guided flow:
1. shows "Pole will move to 140 mm, hand will open fully"
2. runs the control app's `--manual-pole` then `--manual-hand-raw` (the same order as ALIX: pole, then hand; `TAUGHT_PIPELINE_README.md:93`)
3. checks the pole/bin foul band
4. starts `--drag-session`
5. after `--drag-save`, writes `<file>.SOURCE.json` with `side, pole_m, hand_counts, hand_state label, hardware: ok, taught_by, date, task/area id`, using the `taught_tracks` provenance schema (`ALIX/tools/taught_tracks.py:3628-3634`)

The 3-D preview uses the Studio robot overlay with `*_sliding_plate_joint` and finger joints set from these values (`/api/robot-geometry` plus FK, or `/api/sim/joints` external feed). Studio's `hand-gizmo` is optional.

---

## 4. Schedule & Run

### 4.1 What it does

- **Markup:**
  - ribbon `WE/template.html:642-672`: fixture, plan set `hardware|simulation`, mode, Load Schedule, Check Collisions, Save Schedule; Process Timeline `#schRuler`, `#schTrack`, playback `#schPlayback`
  - Browser dock: "Plan File" free player for any plan JSON (`:759-768`) and "Schedule Steps" `#schStepList` + "+ Add step" (`:815-821`)
  - Properties dock: Execution Status, Current Step, Guards (`:887-894`)
- **Inputs:**
  - `ORCH/config/schedule_commode.csv`, grammar `7R` (single), `0R,1L` (paired, concurrent), `2L,>2R` (serial), `0R>1R,1L>0L` (chained), `2C` (compound) (`:17-24`); the current 17 rows (`:26-45, 78-95`)
  - `regions_commode.csv`
  - plan JSON files resolved per (region, arm, mode, plan set) through the orchestrator's own loader (`SRV:4708-4790`)
  - session phases pre/post (bin_drop, glove_wear…, glove_remove, lid_close, flush_press, bin_raise) emitted as locked steps (`SRV:4793-4832`; `ORCH/alix_orchestrator/core/session_phases.py:87-255`)
  - glove/inspection checkpoints (`SRV:4912-4922`)
  - hinge geometry to animate lid/seat (`SRV:4924-4960`)
  - `carryJoints` carried between steps (`SRV:4890-4909`)
  - `taskDelaySec` from `orchestrator_params.yaml` (`ORCH/config/orchestrator_params.yaml:12` → 1.0 s)
- **Per-step duration:** `_plan_duration_sec`, the sum of sub-trajectory durations plus `hold_before_s` (`SRV:4677-4692`). Paired steps use max(left, right), serial the sum, chained the longer chain (`SRV:4854-4866`).
- **Guards (`TG/schedule_guards.py:1-37`):**
  - tier 1 logical (arm coverage, duplicates, plan present)
  - tier 2 reality (fixture articulation timeline, part `implies`)
  - tier 3 inter-arm FCL from the cached `/api/dual-arm-collisions` verdicts, where "stale" is never treated as passing
  - optional coordination verdict for `~name` cells
  
  Verdicts are "full sentences naming the row" (`:33-37`), a good UX rule for Lite. Payload builder: `_schedule_guards_for` (`SRV:5003-5040`).
- **Playback:** `POST /api/plan-frames` FK-renders a step's plans, given as `plans` (concurrent) or `tracks` (chained/staggered), with `initial` carry and an optional hinge angle derivation. It is disk-cached on content hash (`SRV:4590-4674`). `alix_workspace.dual_arm_plan_collision.load_plan_timeline` expects `sub_trajectories[].joint_names` + `waypoints[].positions` (radians) + `time_from_start` (`ALIX/core/python/alix_workspace/src/alix_workspace/dual_arm_plan_collision.py:46-91`).

### 4.2 Timeline UI

- `schTimelineLayout` makes bars proportional to `durationSec` (minimum 0.4 s) plus the inter-row `task_delay` (`WE/editor.js:13601-13611`).
- `schRenderTimeline` (`:13613-13676`):
  - one `<button class="sch-bar">` per step with flex-basis = share of total
  - type code chip PA/SE/SI/CH/CO and colour/texture (`SCH_TYPE_COLOR`/`SCH_TYPE_TEXTURE`/`SCH_TYPE_CODE`, `:13593-13596`)
  - locked session phases "PH"
  - verdict classes `is-bad/is-warn/is-stale`
  - ruler markers 🧤 glove / 🔍 inspection
  - total "N s · R rows · unsaved"
- Click a bar to jump; drag a bar to reorder rows (`schWireBarDrag` `:13678-13718`).
- Transport, shared with the Dual-Arm Collisions tab: `renderPlaybackBlock` "▶ Play all / ▶ Play step / ■ Stop / ⟸ Step / Step ⟹ / slider" (`:8749-8766, 8813-8824`).
- Step list, detail and edit: `schRenderSteps` (`:12885`), `schRenderStepDetail` (`:12967`), `schEditStep` (`:13840`).

### 4.3 Does it run on the emulator or the hardware?

**No.** "idle — offline preview (arms not connected)" (`WE/template.html:889`; `WE/editor.js:13048-13066`). Playback moves only the 3-D robot overlay. The separate "Simulate" view switch polls the emulator or an external feed (`WE/template.html:56-112`; `setView`/`simTick` `WE/editor.js:19204-19300`; `SRV:5312-5393`).

### 4.4 Output format

`POST /api/schedule-save` (`SRV:5080-5175`):
- tier-1/2 errors block the save; any recorded inter-arm hit blocks it
- rows never swept or carrying warnings need `ackStale`/`ackWarnings`
- an mtime conflict check is applied
- a backup is written to `.cache/schedule_backups/`
- the CSV is written **atomically** with a generated header (`schedule_csv_text`, `write_schedule_csv`, `TG/schedule_guards.py:59, 619`)

`POST /api/schedule-preview` resolves unsaved rows through the same loader (`SRV:5043-5054`).

### 4.5 Related dual-arm modules (not exposed in the UI)

- `TG/coordination.py:1-24`: "Stage 6 — the coordination diagram for one concurrent pair" (Wittmann et al.). Slowdown-only re-timing of two plans so they do not collide; it writes `coordination_<pair>.json`. Driven by CLI `ALIX/tools/motion_analysis/coordinate_pair.py:1-21`. The schedule cells reference it as `3R~row7` (`ORCH/alix_orchestrator/core/schedule_loader.py:47, 559-612`). Relevant to R10 and the V6 "TOGETHER" timing.
- `TG/schedule_chain.py:1-52`: partner state per schedule position (`PRESTART = {arm_state: ready, pole: full_length, hand_state: grasp}`).
- `TG/task_schema.py:84-90`: `COORDINATION_MODES = independent | synchronized | chained | dual_group`. The Lite control app has its own `dual_rule` values `TOGETHER | RUN FIRST | WAIT OTHER` (`V6/*_FINAL.json`). The two vocabularies should be mapped when R8/R10 are designed.

### 4.6 Fit for Lite

- **R10** (dual-arm view and timing analysis): reuse the timeline component and `/api/plan-frames`-style FK playback.
- **R8** (merging): a Lite "sequence" of rows (`program`/`pole`/`hand`/`wait`) is analogous to schedule rows plus session phases. Reuse the "guards as sentences" and "stale ≠ pass" rules.
- **R1:** the order of teaching prompts can follow the same ordered list as the run sequence (PPT task order).
- **Adapter needed:** `.txt` (NDJSON millidegrees, 5 ms/pt) → plan JSON. `joint_names = [L|R]_joint1..7`, positions = mdeg/1000 → rad, `time_from_start = i·0.005` s (inferred from the V6 README cadence "5 ms per point", `V6/README.txt:4-8`). Pole steps become an extra sub-trajectory on `<L|R>_sliding_plate_joint` (metres) timed with the pole model (§3.9).

---

## 5. Guided / wizard / step-by-step flows

- **Studio has none.**
  - The ribbon comment: "Grouped by purpose, not numbered. The ordinals told a linear story ('step 1 of 5') over a loop the operator runs 38 times" (`WE/template.html:116-118`). The only vestige is the heading "Region Labels — step 1" (`:158`).
  - The Studio "session" is EDIT vs GENERATE exclusivity on the Workbench (`WE/editor.js:14811-14835`), not a guided flow.
  - `GET /api/session` reports server busy state only (`SRV:6255-6290`).
  - Bulk "Generate missing" iterates over cells with per-cell progress and Cancel (`WE/editor.js:15494-15655`; `TG/studio_bulk.py`). That is the closest "loop over every region" behaviour, but it is X1 and unattended.
- Feedback primitives Lite can keep:
  - status line `#status` (`WE/template.html:71`)
  - error toasts (`showErrorToast`, 12 s, `WE/editor.js:5445-5462`)
  - `data-tip` tooltips and `? Help` (`WE/template.html:946`)
  - playback step buttons (`WE/editor.js:8813-8824`)
- **Trajectory HTML has a guided stepper.**
  - 6 nodes Upload → Setup → Analyze → Review → Generate → Compare (`LITE/Alix_Studio_Lite_Trajectory_V2.html:335-340`)
  - `PIPELINE_META` titles and notes (`:609-616`)
  - lock/unlock rules `pipelineStepUnlocked` (`:618-…`)
  - `goToStep` (`:659`), prev/next buttons (`:696-701`)
  
  This is the natural pattern for R1.
- **Control app:** Drag & Teach has a confirmation dialog ("The arm will enter Drag Teach mode… keep the hardware E-stop accessible… START DRAG + RECORD?") and a toggle that serves as Start/Stop (`V53:3877-3965`). It has no per-region prompts and no cleaning-time timer; `elapsed` appears only as run timers (`V53:5533-5551`).
- **R1 approach** (new work, composed from the pieces above). A "Teach Session" stepper per area:
  1. show the PPT card (slide image plus area name plus which arm plus fixture state)
  2. prepare the robot (pole, then hand; §3.10)
  3. start drag + record, with a **countdown = the area's cleaning time** and a large visible timer
  4. stop at timeout or by the user, then auto-save to the canonical filename plus `.SOURCE.json`
  5. offer quick checks (points, duration, idle %) with **Redo** or **Accept**
  6. auto-advance to the next untaught area; the dashboard updates live (`V53` runner flags `--drag-session`, `--drag-save`; `V53:3941, 4005-4010`)

  Teach both arms of a card sequentially by default, with an optional "both arms" mode (inferred: the control app has dual-drag history `UC/py programs/rm_trajectory_ui_v35_common_dual_drag.py`).

---

## 6. Excluded features (X1 / X2): inventory and coupling

### 6.1 Tabs and panels

| UI | where | class |
|---|---|---|
| Regions tab: paint/erase brush, OBJ import/export, paintable parts, Combine, state selects | `WE/template.html:156-201, 783-806`; `WE/editor.js:2162-3260, 8853` | X1 input (mesh labeling for the generator) |
| Regions tab: **Check Arm Reachability** | `WE/template.html:188`; `WE/editor.js:2742`; `SRV:850-863` | X2 |
| Workbench: Generate dock (Target, Start pose, Pad & rows, Stroke & turn, Spline, ψ/C-space, Output, mechanism Space path, Generate/Cancel/Reattach, Apply knot edits) | `WE/template.html:204-419, 861-865` | X1 (ψ group = X2) |
| Workbench: Editor Mode, Selected Point, Start/Pre-start, Sequence, Analyze & Finish (coverage, Validate, Deploy), Reference Frame | `WE/template.html:421-526` | X1 (Cartesian point editing) |
| Board: Generate missing, Raster sweep, coverage/validation metrics | `WE/template.html:544-548`; `SRV:5562-5773` | X1 |
| Workspace tab: Workspace Analysis, Kinematics Solver, Redundancy Atlas, Selected Point — Analysis | `WE/template.html:556-597` | X2 (Kinematics Solver = IK engine, needed by R9 under the hood) |
| Gizmo tab: Orientation Gizmo, IK Frame Calibration | `WE/template.html:675-695` | X1-adjacent (tool-frame authoring) |
| Visibility bar: Path (Points/Path/Spline/Frames/Normals), Reach voxels, Manip heatmap, Unreachable, Atlas band, Ring cage, Reach dome | `WE/template.html:698-741` | X1/X2 toggles (keep Robot/Visual/Collision/Walls/Grid) |
| Footer: Mirror Arm, Mirror + Reverse, Points: deltas | `WE/template.html:929-930, 943` | X1 |

STUDIO_GENERATE_TAB_PLAN planned to fold the Workspace tab into the Workbench (`ALIX/docs/studio/STUDIO_GENERATE_TAB_PLAN.md:1552-1575, 1659-1700`). The template still ships a separate Workspace tab (`WE/template.html:124, 556`), so the fold has not landed in this snapshot.

### 6.2 Endpoints (`SRV:5961-6029`)

- **X1:**
  - `/api/generate`, `/api/auto-generate`
  - `/api/auto-path`, `-reauthor`, `-status`, `-cancel`
  - `/api/start-pose-check`, `/api/path-compare`, `/api/path-ik`
  - `/api/bulk-generate`, `-status`, `-config`, `-cancel`, `-apply`
  - `/api/coverage`, `/api/mirror`, `/api/retarget`
  - `/api/validate`, `/api/verify-save`, `/api/save`, `/api/deploy-path`
  - `/api/generate-mechanism`, `/api/mechanism-ops`, `/api/mechanism-exemplar`
  - `/api/save-regions-obj`, `/api/update-region`
  - `/api/analyze-points`, `/api/analyze-config`, `/api/animate-sequence`
  - `/api/fixture-board`, `/api/board-cell` (config-based)
  - `/api/save-ik-frames`, `/api/save-gizmo`, `/api/ik-frames`, `/api/gizmos`, `/api/gizmo`
- **X2:**
  - `/api/reachability`, `/api/region-reachability`, `/api/reachable-mask`, `/api/manipulability`
  - `/api/redundancy-atlas`, `/api/atlas-pose`, `/api/atlas-sweep`, `/api/atlas-environment`
  - `/api/ik` (point IK panel; its *engine* is reused for R9/R14)
- **Keep / borrow:**
  - `/api/fixtures`, `/api/manifest` (meshes, read-only), `/api/regions` (read-only, if a 3-D area view is wanted)
  - `/api/robot-geometry`, `/api/joint-presets`, `/api/glove-frames`, `/api/hand-gizmo`
  - `/api/plans`, `/api/plan-frames`, `/api/dual-arm-collisions`, `/api/analyze-plan`
  - `/api/schedule`, `/api/schedule-pairs`, `/api/schedule-preview`, `/api/schedule-save`
  - `/api/sim/status`, `/api/sim/joints`, `/api/sim/pose`, `/api/session`

### 6.3 Modules (`TG/*.py`, line counts)

- **X1 generator stack:**
  - large: `path_emission` 8023, `cspace_author` 10999 (also X2), `studio_generate` 5846, `stroke_gate` 5140, `path_layout` 4324, `path_pipeline` 3803, `tilt_profile` 3392, `entry_planner` 2687, `path_generator` 2382, `geometric_path` 1571, `region_corrections` 1256, `auto_generate` 1167, `coverable_map` 1058, `body_collision` 989
  - also: `cleaning_actions` 846, `junction_limits` 760, `stl_analyzer` 744, `coverage` 734, `yaml_assembler` 704, `mirror` 679, `studio_bulk` 622, `frame_bands` 468, `derived_passes` 449, `reflection_guard` 430, `contract_motion` 423, `pair_coverage` 399, `arc_geometry` 351, `tilt_control` 328, `pose_generator` 325, `section_set` 318, `formatted_yaml_writer` 313, `arm_filter` 317, `archive_seeds` 272, `approach_transit` 226, `pad_transition` 217, `point_budget` 173, `pre_start` 161, `sequence_generator` 141, `surface_type_detector` 105, `delta_converter` 105, `surface_extractor` 83, and `path_planners/*` (boustrophedon, contour_parallel, racetrack, rag_wipe, ring_serpentine)
- **X2:** `cspace_author` (C-space authoring), `cspace_pair` 785, the atlas parts of `SRV` (`do_redundancy_atlas` `SRV:2568-2923`, `do_atlas_sweep` `SRV:2182-2322`), `singularity` 221 (gate), `sdk_reach` 405 (reach oracle; may still help R9 IK parity).
- **Neutral / reusable:**
  - `regions` 255, `region_component_resolver` 188, `region_mesh_loader` 1090 (read-only use), `fixture_parts` 383, `scene_pose` 330
  - `glove_frames` 169, `stage_sequence` 850 (vocabulary only), `schedule_guards` 689, `schedule_chain` 386, `coordination` 1318
  - `sim_bridge` 208, `robot_model` 405, `constants` 185, `paths` 161, `cache_locks` 41

### 6.4 Coupling notes

- **Regions tab ↔ X1:**
  - the tab is a mesh-paint editor that needs component meshes (`canUseMode('paint')`, `WE/editor.js:7419-7422`)
  - its saves rewrite `regions.yaml`, the CSV and the corrected copy (`SRV:4030-4129, 116-134`)
  - switching to the Workbench carries the painted region into Generate (`WE/editor.js:7376-7391`)
  
  For Lite R1 this coupling is unnecessary: Lite's "regions" are PPT cards. A read-only 3-D "where is this area" view could reuse `/api/manifest` paint meshes (labeled OBJ groups) *if* the Lite toilet is commode_c (§1.4), without any editing endpoint.
- **Board ↔ X1:** coverage per cell calls `do_coverage` (surface rebuild, `SRV:5622-5636`), and validation checks Cartesian configs (`validate_config_only`). Neither applies to `.txt`.
- **Schedule ↔ X1:** plan JSONs come from MoveIt planning of generated configs (plan roots `TG/paths.py`). The tab itself needs only plan files and the CSV, so it is X1-free.
- **Guards ↔ manifest:** the reality tier uses manifest `part_states` (`SRV:5003-5040`). Lite can reuse the idea with PPT fixture states (lid closed / open / seat open).
- **IK engine:** R9/R14 (FK → offset → IK) needs `alix_workspace` IK/FK. That is not the X2 *feature* (Workspace tab, atlas), but it is the same package; keep the engine and drop the panels.

---

## 7. Reuse assessment: concrete extraction targets

1. **Dashboard (R1):** lift `renderBoardRows` / `loadBoard` / `fillBoardMetrics` / `initBoard` (`WE/editor.js:14706-14757, 15208-15467`) and the Board CSS (`editor.css:1507-1524, 1572-1574, 1731-1750, 1974-1999`, `.tab-badge` `:832-838`). Re-write the backend as `GET /api/lite/areas` that scans `raw/`, `optimized/` and `uploaded/` folders against the Lite registry; drop `/api/board-cell`.
2. **Area registry (R1/R7):** a YAML in the spirit of `regions.yaml` (documented keys, comments, `region_id`) with fields `task, slide, arm, area_code, name, part, kind(clean|mechanism|tool), fixture_state, cleaning_time_s, pole_mm, hand_counts, hand_label, file_stem, order`. Seed it from the PPT XML parse (§1.4) and `commode_c_taught_paths/description.txt`. Re-use the strict-parse philosophy of `parse_description` (`ALIX/tools/taught_pipeline.py:187-221`).
3. **PPT display:** render slides to PNG once, at build time or on demand (inferred: LibreOffice headless or python-pptx plus media; not verified available). The fallback uses extracted `ppt/media/image*.png` plus parsed label text and positions.
4. **Pole/hand vocabulary (R7):**
   - SRDF rungs from `/api/joint-presets` (`SRV:1763-1785`)
   - the 0–315 mm range (`TG/stage_sequence.py:189-192`)
   - the foul band constant and warning text (`WE/editor.js:17149, 17178-17186`; `ORCH/.../task_config.py:97-124`)
   - hand presets from `V53:2982, 2997` (GRIP/OPEN) and `% open` from `taught_pipeline` (`fingers_<pct>pct`)
   - counts ↔ rad from `hand_units.py` for the 3-D preview
5. **Apply-to-robot (R7, INT-CTRL):** call the control-app runner with `--manual-pole <mm> --side <arm> --yes` and `--manual-hand-raw v×6 --side <arm> --yes` (`V53:4159-4165, 4190-4196`). Use the pole-then-hand order (`ALIX/docs/motion/TAUGHT_PIPELINE_README.md:93`). For a non-Tk Lite backend, re-implement on the SDK through `UC/pole.py` and `UC/hand.py`, or `alix_dispatch/peripherals.py`.
6. **Provenance sidecar (R1/R7):** the `.SOURCE.json` schema `{side, frame, pole_m, hand_state, hand_counts, hardware, taught_by, date, units:"millideg", note}` (`ALIX/tools/taught_tracks.py:314-317, 3628-3634`) plus `task/area` ids. This keeps Lite files compatible with `taught_tracks.py certify` (R13).
7. **Timeline and transport (R10/R8):** `schTimelineLayout` / `schRenderTimeline` / `renderPlaybackBlock` (`WE/editor.js:13593-13676, 8749-8766`) and the `.sch-*` CSS (`editor.css:1130-1230`). Feed them from Lite workflow steps (program durations = points × 5 ms; pole steps from the pole model; waits).
8. **3-D robot playback (R3/R10):** `/api/plan-frames` + `load_plan_timeline`, with a `.txt` → plan-JSON adapter (§4.6). `/api/sim/joints` + `/api/sim/pose` provide the "external feed" live view (R4 toggle precedent: Author/Simulate switch with Source auto/emulator/external, `WE/template.html:56-112`).
9. **Undo (R2):** Studio's snapshot stack (`pushHistory`, cap 80, `WE/editor.js:1896-1902`) shows the pattern. Lite needs a cap of 5 per area.

---

## Requirement relevance

- **R1:** No Studio feature covers prompting, timing or auto-advance (§5). Borrow the Board UI for the dashboard (§2.2) and the PPT-derived registry (§1.4). The drag-teach primitives exist in the control app (`V53:2698-2790, 3877-4017`; `RUN14:957-988`). Filename and provenance conventions must be added (§2.2, §3.10). The trajectory HTML stepper is the guided-flow pattern (§5).
- **R2:** Studio's "sub-region" = ordered `groups` in one region (`C_C/models/regions.yaml:17-22`). The PPT's sub-areas are PART 1/2 and halves (§1.4). The word "region" in the trajectory HTML means a time window (terminology fix needed). The undo pattern is in §7.9.
- **R3:** Schedule & Run and Plan File playback are offline FK playback of plan JSON (`SRV:4590-4674`). They need a `.txt` adapter (§4.6).
- **R4:** Studio has no hardware execution (§3.7). The Author/Simulate switch with source "External feed" (`WE/template.html:56-112`; `SRV:5326-5341`) is the UI precedent for a real/sim toggle. The real-arm connection belongs to the control app.
- **R5:** The Schedule timeline badges verdicts per step, and the DAC tab's prev/next "colliding sample" transport is shared (`WE/editor.js:8813-8817`). These are inputs for the highlight-and-stop behaviour (detailed in other maps).
- **R7:** §3 in full. Studio models pole and hand per task (stages), shows them in the viewer, and warns about the foul band. It **never applies** them. The apply path is control app v48/v53 manual pole/hand. Per-region values exist in `commode_c_taught_paths/description.txt` and in the V6 JSON pole/hand steps.
- **R8:** Schedule rows, session phases, guards ("stale ≠ pass", sentences) and the V6 workflow JSON (pole/hand/wait/program steps) are the model for a merged executable sequence. `coordination.py` handles concurrent re-timing.
- **R9 / R14:** The IK/FK engine under the X2 Workspace panels (`/api/ik`, `alix_workspace`, `sdk_reach`) is needed. Keep the engine, drop the panels (§6.4).
- **R10:** Schedule timeline plus 3-D playback plus the pole-joint animation. Correct pole height is part of the "correct URDF" (sliding plate joints). The bin state interacts with the pole foul band (§3.6).
- **R13:** The `.SOURCE.json` provenance keeps Lite recordings eligible for the `taught_tracks` certify/upload lane (§3.8).
- **X1:** Inventory in §6.1–6.3. The Regions-tab painting and the Board's generate and metrics are X1-coupled and must not be ported.
- **X2:** The Workspace tab, "Check Arm Reachability", atlas/reach/manipulability overlays and endpoints, and the ψ group in Generate.
- **NFR-UX:** The Board's "chips, not a sentence" scan pattern (`WE/editor.js:14733-14736`) and guard verdicts as full sentences (`TG/schedule_guards.py:33-37`) are good for non-technical users. Studio's tooltip prose is expert-level (e.g. `WE/template.html:158, 235-237`) and must be rewritten. The control app uses all-caps technical labels (e.g. `V53:2952` "APPLY HAND // ALL 6 CHANNELS"), which also need friendlier wording.
- **NFR-THEME:** Board pills and chips, tab badges and timeline classes all use the `:root` tokens (`editor.css:1-80`, brand `#2F96EE`, `--ok #2E9E5B`, `--warning #C77700`, `--danger #D93A34`, dark variant `:root[data-theme="dark"]` `editor.css:133-134`). Lifting them keeps the Studio look.
- **INT-CTRL:** The control app supplies drag-teach record/save/playback and manual pole/hand apply, and its workflow JSON carries pole/hand/wait/program steps. These are the actuators for R1/R7.
- **INT-TRAJ:** The trajectory HTML supplies the R1 stepper pattern. Its "regions" (time windows) must be renamed to avoid clashing with cleaning areas.

---

## Reuse verdicts

| feature | verdict | what to lift | evidence |
|---|---|---|---|
| Regions tab mesh painting / OBJ labeling / Combine | not for Lite (X1) | nothing. Optionally a read-only 3-D area view from `/api/manifest` | `WE/template.html:156-201`; `SRV:4030-4139` |
| Region manifest schema (regions.yaml) | reference-only | documented-keys style; `generate:false`, `groups` (sub-areas), part states ↔ PPT fixture states | `C_C/models/regions.yaml:16-82`; `TG/region_mesh_loader.py:265-419` |
| Board UI (table, chips, tab badge, cell states, click-to-act) | reuse-with-adaptation | `renderBoardRows`, `loadBoard`, `initBoard`, Board CSS | `WE/editor.js:14708-14757, 15208-15467`; `editor.css:1507-1524, 1974-1999` |
| Board backend (`/api/fixture-board`, `/api/board-cell`) | reference-only | the "only what is on disk" and mtime-cache ideas; rewrite for `.txt` | `SRV:5396-5486, 5562-5773` |
| Per-region pole + hand applied before teach | not-available in Studio | take from control app (`--manual-pole`, `--manual-hand-raw`) and ALIX taught lane | `WE/editor.js:13045-13047`; `TG/sim_bridge.py:10-21`; `V53:4142-4205` |
| SRDF pole rungs / custom height / 0–315 mm | reuse-as-is (vocabulary) | `/api/joint-presets`, `POLE_RANGE_M`, `CUSTOM_POLE_STATE` | `SRV:1763-1785`; `TG/stage_sequence.py:181-207`; `poles_group_states.yaml:11-63` |
| Pole/bin foul-band warning | reuse-as-is | band 0.153–0.282 m and its warning text | `ORCH/alix_orchestrator/core/task_config.py:58-124`; `WE/editor.js:17149-17186` |
| Hand state per pad + hand gizmo | reuse-with-adaptation | `glove_frames.yaml` states, `/api/hand-gizmo`; drive fingers from counts with `hand_units.py` | `TG/config/glove_frames.yaml:97-109`; `SRV:3376-3450`; `ros/hw/alix_hw/scripts/hand_units.py:33-60` |
| Per-recording pole/hand provenance | reuse-as-is (schema) | `.SOURCE.json` fields; `description.txt` table | `ALIX/tools/taught_tracks.py:309-340, 3628-3634`; `ALIX/tools/taught_pipeline.py:11-27, 183-221` |
| Real per-area pole/hand values (commode C) | reuse-as-is (data) | 140/200 mm, 100/50% open per task | `…/commode_c_taught_paths/description.txt:3-16` |
| Schedule & Run timeline + transport | reuse-with-adaptation | `schTimelineLayout`, `schRenderTimeline`, `renderPlaybackBlock`, `.sch-*` CSS | `WE/editor.js:8749-8766, 13593-13676` |
| Schedule CSV grammar + guards | reference-only | sentence verdicts, stale≠pass, atomic save + backup | `TG/schedule_guards.py:1-37, 59, 619`; `SRV:5080-5175` |
| Plan-frames FK playback | reuse-with-adaptation | `/api/plan-frames` + `load_plan_timeline` with a `.txt` → plan JSON adapter | `SRV:4590-4674`; `alix_workspace/.../dual_arm_plan_collision.py:46-91` |
| Session phases / checkpoints | reference-only | glove/inspection markers on the timeline; PPT tool tasks ≈ phases | `SRV:4793-4832, 4912-4922` |
| Coordination (concurrent re-timing) | reference-only (R10 later) | `coordinate_pair` CLI and artefact | `TG/coordination.py:1-24`; `ALIX/tools/motion_analysis/coordinate_pair.py:1-21` |
| Guided wizard / timer / auto-advance | not-available in Studio | build new; stepper pattern from the trajectory HTML | `WE/template.html:116-118`; `LITE/Alix_Studio_Lite_Trajectory_V2.html:335-340, 609-616, 659` |
| Workbench (Generate + point editing) | not-applicable (X1) | none | `WE/template.html:204-526` |
| Workspace tab / atlas / reachability | not-applicable (X2) | keep only the IK/FK engine for R9 | `WE/template.html:556-597`; `SRV:800-863, 2060-2923` |
| Simulate view (emulator / external feed) | reuse-with-adaptation | Author/Simulate switch, `/api/sim/joints` external feed for the live real arm | `WE/template.html:56-112`; `SRV:5312-5393` |
| Undo stack | reuse-with-adaptation | snapshot push/pop, cap 5 per area | `WE/editor.js:1896-1902` |

## Gaps and risks (summary)

- Two hand count→radian scales disagree: `hand_units.py` (1.344/0.5236/1.246) vs `sim_bridge.py` (1.60/0.70/1.10). The 3-D hand preview could mislead unless the hardware scale is used.
- The V6 workflow's 200 mm pole step, and the 200 mm commode C recordings, sit inside the stowed-bin foul band (0.153–0.282 m). This is safe only with the bin deployed.
- Drag-teach saves carry no task/area/pole/hand metadata (`V53:3840-3850`; `RUN14:957-988`), so the dashboard cannot tell which area a recording belongs to without a new naming and sidecar convention.
- PPT labels lack consistent IDs (R-R01 labels two parts; `FL-` prefix unexplained; many labels have no code), and the PPT has areas Studio never modelled (pedestal, base, drain hole, floor spillage). A Lite-owned registry is required.
- "Region" means three different things across Studio (mesh area), the trajectory HTML (time window) and the PPT (cleaning area).
- `V6/README.txt:60-78` names `LEFT_V6_FINAL.json` / `RIGHT_V6_FINAL.json`, a top-level `manual_checkpoint` field (`:64-65`) and `DUAL_FLOW_TIMELINE_V6.json`. The folder ships `*_V6_70SERIES*.json` whose top-level keys are only `format, side, workflow_note, steps` (the checkpoint is a `wait` step with a note), and the timeline file is absent.
- Schedule & Run depends on `alix_orchestrator` being importable and on plan JSON files (`SRV:4721-4730`). Porting it means the adapter and a Lite-native sequence model.
