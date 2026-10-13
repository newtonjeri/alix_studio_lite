# C7 — Scope, usability, theme, safety, deployment and data (verified)

Cluster **C7_scope_ux_theme**: X1, X2, NFR-UX, NFR-THEME, NFR-SAFETY, NFR-OFFLINE-DEPLOY, NFR-DATA.

This is the analyst draft after adversarial verification. I opened the cited code for every claim about what exists, how it works or what can be borrowed. Where a claim held, it is kept and marked **Verified**. Where it was wrong or incomplete, it is marked **Corrected**. Things the draft missed are marked **New**. "Measured" means I ran a read-only command. "Inferred" means a conclusion I did not observe directly. The change log at the end lists every material change and its evidence.

Readers: the engineers who will build Lite, and the requester, who wrote the inputs.

## Path abbreviations

| Abbrev | Absolute path |
|---|---|
| `A` | `/home/newtonjeri/alix_ws/src/alix` |
| `TG` | `A/core/python/alix_taskgen/src/alix_taskgen` (ALIX Studio package) |
| `WE` | `TG/web_editor` (`template.html` 953 lines, `editor.js` 19,325, `editor.css` 2,166) |
| `SRV` | `TG/server.py` (6,566 lines) |
| `TST` | `A/core/python/alix_taskgen/test` |
| `WS` | `A/core/python/alix_workspace/src/alix_workspace` |
| `DP` | `A/core/python/alix_dispatch/src/alix_dispatch` |
| `ORCH` | `A/ros/orchestration/alix_orchestrator/alix_orchestrator` |
| `RES` | `A/docs/studio/STUDIO_UIUX_RESEARCH.md` |
| `DTC` | `A/docs/motion/DRIVER_TRACK_CHANNEL_ARCHITECTURE.md` |
| `LITE` | `/home/newtonjeri/alix_studio_lite_ws` |
| `IN` | `LITE/alix_studio_lite/docs/reference/ALIX_Studio_Lite_requirement_inputs.txt` |
| `OPS` | `LITE/RealMan Dual-Arm Operations UI.txt` |
| `H` | `LITE/Alix_Studio_Lite_Trajectory_V2.html` (trajectory prototype, 2,096 lines) |
| `UC` | `LITE/upload_channel` |
| `V53` | `UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py` (control-app UI, current) |
| `R14` | `UC/rm_upload_fast_v14_safe_transit.py` (control-app runner, current) |
| `V6` | `LITE/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES` |

## Summary

| ID | Requirement | Borrow from ALIX Studio? | Priority | Effort | Main changes from the draft |
|---|---|---|---|---|---|
| X1 | Exclude geometric path generation and editing | Not applicable (exclusion) | Must | S (2-3 d) | The reusable Studio routes depend on X1/X2 code, so their behaviour must be re-implemented rather than imported. The "neutral" helpers are tied to the repo layout. Missed spots (R3) must be shown with a read-only pad trail, because ALIX's coverage tools are X1. |
| X2 | Exclude C-space and Workspace features | Not applicable (exclusion) | Must | S (1-2 d) | Added `config_loader` to the kept engine. Reclassified `clearance` and `wipe_kinematics` as optional helpers. The recommendation is now a vendored, pinned subset, because `plan_scene_validation` raises at import outside the repo (measured). |
| NFR-UX | Very user-friendly for non-technical operators | Partial: borrow some, build the rest | Must | L (12-15 d) | Added ALIX's layout audit tool for automated UI checks. Dropped the control-app bug detail (the user asked for no bug audit). The SIM-mode test task needs the emulator or hardware. |
| NFR-THEME | Keep the ALIX Studio theme and formatting | Borrow with adaptation | Must | M (5-7 d) | Most of the listed CSS defects sit in X1/X2 rules that Lite drops anyway. Off-theme JS colour literals get past ALIX's guard test. The deck's arm colours clash with the brand and warning colours. |
| NFR-SAFETY (new) | Safe robot operation | Partial: patterns only; the safety logic comes from the control app and `alix_dispatch` | Must | XL (16-22 d) | Follows ALIX's one-command-source rule. Adds the controller SIM/REAL rules (pole and hand are not simulated). Adds the approach-move check, cross-origin protection and an upload ledger. Both collision checkers hard-code the bin as lowered. Corrected the commode-C pole heights. |
| NFR-OFFLINE-DEPLOY (new) | Offline, self-contained install | Partial: borrow some, build the rest | Must | L (11-15 d) | The Jetson AGX Orin (aarch64) is a candidate target. Repo independence needs patches or a vendored subset (measured). The self-check now covers silent fallbacks. Corrected the environment variables. |
| NFR-DATA (new) | Project folder, immutable raw files, provenance, checksums | Partial: borrow some, build the rest | Must | L (10-14 d) | The SDK writes the raw file itself, so Lite saves to a temporary name, validates, then renames. Added an upload ledger, because the controller verifies by size only. Run mode is recorded with every run. The Studio's save handler is not atomic. |

Effort scale (one experienced developer, tests included): S ≤ 3 days, M 4-8, L 9-15, XL > 15.

---

## X1 — Exclude geometric path auto-generation and editing

### Original text (IN:3-4, verbatim)

> What is ALIX Studio Lite - It is a light weight version of the ALix Studio (/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen) that uses the drag and teach generated plans, that are optimized (smoothed and speed increased) since they are recorded at slow speed and with a number of stops, with th e following functionality not include,
> 1. the geometric path autogeneration and editing since we start in joint space (taught .txt plans)

### Revised requirement

Lite edits taught joint-space recordings only. It shall not contain, ship, load or expose ALIX Studio's geometric (Cartesian) path generation or path editing.

- **(a) Excluded screens and controls.**
  - The Workbench tab, both halves:
    - the Generate Task dock: Target, Start pose, Pad & rows, Stroke & turn, Spline, ψ / C-space, Output, Generate / Cancel / Reattach, knot edits;
    - the point editor: Editor Mode, Selected Point, Start / Pre-start Pose, Sequence, Analyze & Finish (coverage / Validate / Deploy), Reference Frame.
  - The Regions-tab mesh painting: Paint / Erase brush, Import / Export OBJ, Combine Regions, part-state selects, region rename and delete.
  - On the Board: Generate missing, Raster sweep, and the coverage and validation metrics.
  - The Gizmo tab: Orientation Gizmo and IK Frame Calibration.
  - In the footer: Mirror Arm, Mirror + Reverse, Points/deltas.
  - The path-authoring overlays: Points, Path, Spline, Frames, Normals, Gizmo, Start gizmo, start EE, task frame.
- **(b) Excluded routes (35).** `/api/generate`, `/api/auto-generate`, `/api/auto-path`, `/api/auto-path-reauthor`, `/api/auto-path-status`, `/api/auto-path-cancel`, `/api/start-pose-check`, `/api/path-compare`, `/api/bulk-generate` and its `-status`, `-config`, `-cancel`, `-apply`, `/api/coverage`, `/api/mirror`, `/api/retarget`, `/api/validate`, `/api/verify-save`, `/api/save`, `/api/deploy-path`, `/api/generate-mechanism`, `/api/mechanism-ops`, `/api/mechanism-exemplar`, `/api/save-regions-obj`, `/api/update-region`, `/api/regions`, `/api/analyze-points`, `/api/analyze-config`, `/api/animate-sequence`, `/api/fixture-board`, `/api/board-cell`, `/api/save-ik-frames`, `/api/save-gizmo`, `/api/gizmos`, `/api/gizmo`.
- **(c) Excluded code.**
  - `alix_taskgen.server` and the generator stack: `scripts.generate_cleaning_config`, `scripts.visualize_generated_config`, `studio_generate`, `studio_bulk`, `auto_generate`, `cspace_*`, the `path_*` modules, `path_planners/*`, `stroke_gate`, `geometric_path`, `coverage`, `coverable_map`, `mirror`, `validator`, `yaml_assembler`, `mechanism_assembler`, `entry_planner`, `tilt_*`.
  - `robot_model`, which pulls in `coverage` and `alix_workspace.reachability`.
  - The rest of the generator.
  - None of this is imported at run time, and none of it is in the release artifact.
- **(d) Write boundary.** Lite never writes into the ALIX repository. That covers the task-config, plan and schedule trees, `auto-generated_config`, `Resource/plans`, and the Studio's `REPO_ROOT/.cache` folders. All outputs go to the Lite project folder (NFR-DATA).
- **(e) Not excluded.**
  - Joint-space editing of taught recordings: trim, smooth, scale, merge (R2, R8, R11).
  - Small Cartesian offsets on an existing taught track by FK → offset → IK, using the `alix_workspace` engine (R9, R14).
  - Read-only display of the robot, the commode, and a **pad (TCP) trail** computed by FK from the joint track. The trail is how Lite shows "missed spots" for R3, because ALIX's coverage tools are X1.
  - Dual-Arm Collisions playback and the Simulate view (R3, R5, R10).
- **(f) Schedule & Run.**
  - The tab and its back end are excluded. They preview and edit the ROS orchestrator's schedule CSV over MoveIt plan JSON, and they import `alix_orchestrator` from the ROS tree.
  - Its proportional timeline, transport controls and its "guards as sentences / stale is not pass" rules are reused inside Lite's own sequence step (R8, R10).
- **(g) Allowed ALIX helpers.** These are copied with provenance (source path, ALIX commit, sha256):
  - `body_collision`, `scene_pose`, `fixture_parts`, `transforms`, `paths`, `constants`;
  - `glove_frames` with its YAML;
  - `sim_bridge`, for the emulator view only;
  - `schedule_guards`, for the atomic-write helper and the guard-sentence pattern;
  - `singularity`.

  Their repo-relative lookups are replaced by explicit Lite asset paths.

### Clarifications

- "Geometric path auto-generation" means ALIX's generators that build Cartesian cleaning waypoints from painted mesh regions (`TG/cspace_author.py:1`: "Author a cleaning task from the marked region and the arm's own C-space").
- "Editing" means Cartesian waypoint, knot and pose editing in the Workbench. Lite's edits are time and shape edits on joint arrays.
- "Joint space" means one line per point with 7 integer millidegree values (`R14:14-15`). The runner refuses non-integers (`R14:82-83`).
- Lite's regions come from the regions deck (glossary: Region = deck unit, Sub-region = time window), not from painted meshes.
- "Excluded" means no UI, no route, no import, not packaged. Copying the allow-listed helpers in (g) is allowed; they generate no paths.
- The R9/R14 offsets do Cartesian maths on taught tracks but create no new paths, so they are in scope.
- **New.** "Reusing" a Studio route means re-implementing its behaviour on the allowed helpers, not importing `server.py`:
  - The robot routes sit on `robot_model._ik_context`, which imports `alix_workspace.reachability._placement` (`TG/robot_model.py:170-181`).
  - `/api/manifest` is built by `build_scene_manifest`, which lives in a script that imports X1 modules at load (`TG/scripts/visualize_generated_config.py:13-20, 219`).
- **New.** "Missed spots" (R3, `IN:12`) cannot use ALIX coverage. That coverage is X1: `alix_taskgen.coverage` measures over painted regions, and `alix_workspace.coverage` imports it (`WS/coverage.py:1-8, 51-62`).

### Findings

| # | Status | Source | Finding | Evidence |
|---|---|---|---|---|
| 1 | Verified | Studio front end | There are seven tabs: Regions, Board, Workbench, Dual-Arm Collisions, Schedule & Run, Workspace, Gizmo. | `WE/template.html:115-125` |
| 2 | Verified | Studio front end | The Regions tab is a 3-D mesh-painting editor that feeds the generator. Its tip reads "Mark regions once…, then Generate creates a config per region". | `WE/template.html:156-201` (`:158`, `:176-189`) |
| 3 | Verified | Studio front end | The Workbench is the Generate Task dock plus a Cartesian point editor. | `WE/template.html:204-419, 421-526` |
| 4 | Verified | Studio Board | The Board reads generated `<region>_<arm>_cleaning_points.yaml` files and offers Generate missing and Raster sweep. Its UI suits R1; its back end is X1. | `WE/template.html:537-554`; `SRV:5396-5486, 5562-5636` |
| 5 | Verified | Studio front end | The Gizmo tab authors tool frames. The visibility bar carries the path overlays. | `WE/template.html:675-695, 698-741` |
| 6 | Verified | Studio server | 24 GET + 40 POST routes: 35 are X1, 9 are X2, 4 belong to Schedule & Run, and 16 are reuse candidates (behaviour, see #9-#10). | `SRV:5961-6029` |
| 7 | Verified | Studio server | Importing the server loads the generator. Start-up takes 30-60 s. | `SRV:34-44`; `A/docs/studio/STUDIO_GENERATE_TAB_PLAN.md:2586-2590` |
| 8 | Verified | `alix_taskgen` (measured) | The X1 stack is large: `cspace_author` 10,999 lines, `path_emission` 8,023, `studio_generate` 5,846, `stroke_gate` 5,140, plus five path planners. | `wc -l`; `ls TG/path_planners` |
| 9 | Corrected | `alix_taskgen` helpers | The helpers can be separated by imports but are **tied to the repo layout**. `body_collision._repo_root()` raises "repo root not found" outside the repo, and it looks up venues and the `alix_workspace` source under the repo. `scene_pose` also needs `paths.py` and `constants.py`. `sim_bridge` reads `alix_emulator` and `alix_core`. | `TG/body_collision.py:42-49, 114-119, 135-136, 216-224`; `TG/scene_pose.py:3-24`; `TG/sim_bridge.py:51-54, 76-79, 129` |
| 10 | New | `robot_model` / scripts | The Studio's IK context imports `alix_workspace.reachability._placement` (X2). `config_absolute_poses` imports `alix_taskgen.coverage` (X1). `/api/manifest` uses `build_scene_manifest`, whose module imports `region_component_resolver`, `region_mesh_loader`, `regions` and `yaml_assembler` at load. | `TG/robot_model.py:170-181, 230-244`; `TG/scripts/visualize_generated_config.py:13-20, 219-249`; `SRV:43, 4132-4139` |
| 11 | New | `body_collision` | `BodyCollisionWorld.make_pose_filter(config, fixture)` takes a generated task config (it reads `task_parameters.ik_frame` and the `move_pole` stages). It also fixes the bin at `BIN_LOWERED`. Lite needs a separate entry point for taught tracks. | `TG/body_collision.py:499-530, 627-631` |
| 12 | New | Studio server | The generator imports the server lazily (`cspace_author`, `coverable_map`, `auto_generate`). The import guard must deny both directions. | `SRV:5800-5804`; `TG/auto_generate.py:279` |
| 13 | New | Studio server | The Studio writes caches and backups into the repo: `.cache/dual_arm_collisions` and `.cache/schedule_backups`. Copied code must point these elsewhere. | `SRV:4154, 5153` |
| 14 | New | `alix_workspace` | `alix_workspace.coverage` contains no coverage maths of its own. It imports `alix_taskgen.coverage`, so it is X1-coupled. Missed spots must use a pad trail or new code. | `WS/coverage.py:1-8, 51-62` |
| 15 | Verified | Schedule & Run | It is an offline preview and editor of the orchestrator schedule. It imports `alix_orchestrator` from the ROS tree and writes the orchestrator CSV. | `SRV:4706-4730, 5080-5175`; `WE/editor.js:13045-13066` |
| 16 | Verified | Studio tests | Tests pin rules that contradict Lite's flow: numbered tabs are forbidden, and "the studio has no execution channel". | `TST/test_studio_interaction_guards.py:1207-1224, 2207-2215` |
| 17 | Verified | Runner v14 | Lite's input is joint space: "one joint point per line … in millidegrees — a drag-teach recording". | `R14:14-15` |

### Borrow verdict

- **ALIX Studio: Not applicable.**
  - X1 is an exclusion. ALIX Studio remains the source for theme and shell, the robot overlay, the Simulate and DAC playback, the Board UI, the timeline and transport, and the pole vocabulary and foul-band text.
  - **Corrected:** these are reused as copied code or re-implemented behaviour. The Studio's route handlers cannot be imported: they pull in `robot_model` (X1/X2 imports) and an X1-importing script.
- **Other sources.**
  - Trajectory prototype: not applicable. It is already joint space (`H:803-829`).
  - Control app: not applicable. It is joint space only (`R14:14-15`).
  - `alix_workspace` FK/IK/collision: borrow with adaptation as a library (see X2(e)).
- **Check of the input claim: partly accurate.**
  - "A light weight version of the ALIX Studio (alix_taskgen)" holds for the look, the vocabulary and selected components.
  - Lite should not be a trimmed `alix_taskgen`, for four reasons:
    - importing `alix_taskgen.server` loads the generator (`SRV:34-44`);
    - the rules Lite must break are pinned by tests (`TST/test_studio_interaction_guards.py:1207-1224, 2207-2215`);
    - resources resolve relative to the repo (`SRV:56-87`; `TG/body_collision.py:114-119`);
    - even the reusable robot routes import X1/X2 modules (`TG/robot_model.py:170-181`).
  - "We start in joint space (taught .txt plans)" is accurate.
  - The exclusion must not be read as excluding the R9/R14 offsets.

### What to borrow

| Item | From | Adaptation |
|---|---|---|
| Keep/Adapt/Drop list of Studio parts | `WE/template.html`, `editor.js`, `editor.css` | Lift only the theme/shell, robot overlay, Simulate view, DAC timeline, Board table UI and timeline/transport. Filter out the X1 CSS rules (for example `editor.css:901-921`). |
| Allowed helpers | `TG/body_collision.py`, `scene_pose.py`, `fixture_parts.py`, `transforms.py`, `paths.py`, `constants.py`, `glove_frames.py` (+ `config/glove_frames.yaml`, `ik_frames.yaml`), `sim_bridge.py`, `singularity.py` | Copy into `alix_studio_lite/vendor_alix/` with a provenance header. Replace `_repo_root()` and the venue/URDF lookups with explicit Lite asset paths. Add a taught-track entry point beside `make_pose_filter`. |
| Timeline, transport, guard sentences | `WE/editor.js:13601-13676` (`schTimelineLayout` / `schRenderTimeline`), `8749-8766` (`renderPlaybackBlock`); `TG/schedule_guards.py:1-37` | Feed from Lite's workflow model (program / pole / hand / wait steps), not the orchestrator CSV. Restyle the colour literals (NFR-THEME). |

### What to build

| Item | Approach |
|---|---|
| Import guard test | Start Lite in-process and call every route once. Assert that no deny-listed module is in `sys.modules`: `alix_taskgen.server`, `alix_taskgen.robot_model`, `scripts.*`, `studio_*`, `auto_generate`, `path_*`, `path_planners*`, `stroke_gate`, `cspace_*`, `coverage`, `coverable_map`, `geometric_path`, `mirror`, `validator`, `yaml_assembler`. |
| Route allow-list test | Compare Lite's route table to an explicit allow-list. Any route from (b) fails the test. |
| Release-content test | Build the artifact and assert it holds no deny-listed module file and no generator data. |
| Write boundary | All writes go through the Lite storage module, which refuses paths outside the project and settings folders. |
| Slim scene manifest (with R3) | Rebuild the commode and fixture-part manifest from `fixture_parts` + `scene_pose`, with no region or paint data. Do not import `scripts.visualize_generated_config`. |
| IK context (with R9) | Build Lite's own IK context from the `alix_workspace` kept modules. Inline the 6-line placement transform. Do not copy `robot_model` wholesale. |
| UI scope checklist | Keep the Keep/Adapt/Drop table in the design document and review it at each milestone. |

### Recommendation

Treat X1 as a hard boundary enforced by tests. Build Lite as its own package. Copy the few allowed ALIX helpers with provenance, and re-implement the behaviour of the reusable routes on top of them. Never import `alix_taskgen.server`, `robot_model` or the `scripts` package. State in the requirements that:

- the R9/R14 offsets and all joint-space edits are in scope;
- missed spots are shown with a read-only pad trail;
- the Schedule & Run tab is out, and its timeline and transport components are in.

### Approach

1. Put the Keep/Adapt/Drop table ((a)-(g)) in the Lite design document and get the requester's sign-off.
2. Define the module deny-list and the excluded-route list as constants in the Lite test suite.
3. Lift front-end code from map 01 §8.3 and map 03 §7. Delete the DOM ids and CSS of excluded panels (`#paintToolPaint`, `#genPoleState`, `#regionReachButton`, `.gen-*`, `.rg-*`, the atlas tokens).
4. Copy the allowed helpers into `alix_studio_lite/vendor_alix/` with a provenance header. Replace their repo lookups with explicit paths.
5. With R3/R9, build the slim scene manifest, the taught-track collision entry point and Lite's own IK context.
6. Implement the storage write boundary (NFR-DATA).
7. Add the import, route and release-content tests to CI.
8. Put this in the user help: "Lite does not create new cleaning paths. To change a path, teach the region again or (engineer) apply an offset."

### Acceptance criteria

- After every Lite route has been called in a test session, `sys.modules` contains no deny-listed module.
- Lite's route table contains none of the routes in (b).
- The release artifact contains no `cspace_author.py`, `studio_generate.py`, `stroke_gate.py`, `path_emission.py`, `robot_model.py`, `scripts/*` or `path_planners/*`.
- An automated DOM scan of every Lite screen finds no excluded element id. The signed UI checklist shows no Paint/Erase, Import/Export OBJ, Generate, Mirror, knot or waypoint editing.
- Any Lite API write to a path outside the project folder fails and writes nothing.
- The R9/R14 offset integration tests and the R3 pad-trail display pass with the import guard active.

### UX notes

- Operators never meet "Generate", "spline", "knot", "waypoint" or "coverage". The only ways to change a path are "Teach again" and, for engineers, "Apply offset".
- Region pictures come from the regions deck, which operators recognise, not from painted 3-D meshes.
- Missed spots are shown as the pad's trail drawn over the commode in the 3-D view.

### Safety notes

- Every robot path in Lite starts as a human-taught recording and then passes Lite's checks. There is no untested generated motion.
- Dropping the Generate dock must not drop the Studio's pole/bin foul-band warning (`WE/editor.js:17144-17186`). It moves to Lite's pole controls (R7).
- **New.** The copied collision world fixes the bin at LOWERED (`TG/body_collision.py:627-631`). The taught-track entry point must take the declared bin state, or the check tests the wrong world (NFR-SAFETY).

### Dependencies

- Architecture decision: standalone `alix_studio_lite` package.
- NFR-DATA: write boundary.
- R9/R14: kinematics engine.
- R3/R5/R10: slim scene manifest, taught-track collision entry point, pad trail.
- R8/R10: timeline components.
- R1: Board UI with a new back end.

### Risks

- Scope creep: "just move one waypoint" brings back Cartesian editing. Route such requests to R9 offsets or to re-teaching.
- A convenience import of `server`, `robot_model` or `scripts.visualize_generated_config` silently loads X1/X2 code. The import guard catches it.
- Copied helpers drift from ALIX fixes. Pin the ALIX commit and record the provenance.

### Open questions

- Is any waypoint-level Cartesian editing wanted beyond the R9 offsets? It would conflict with X1.
- Should Lite show regions on the 3-D commode (read-only labelled `commode_c` meshes) or only the deck pictures?
- How should missed spots (R3) be judged: by eye from the pad trail only, or with a new simple contact-coverage estimate on the commode mesh?
- Please confirm that the Schedule & Run tab is excluded and that its timeline and transport are reused.

**Priority:** Must. **Effort:** S, about 2-3 days (deny-lists, three guard tests, write boundary, Keep/Adapt/Drop document). The scene-manifest, IK-context and collision entry-point work is counted under R3/R9.

---

## X2 — Exclude C-space authoring and Workspace-analysis features

### Original text (IN:3, 5, verbatim)

> What is ALIX Studio Lite - It is a light weight version of the ALix Studio (/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen) that uses the drag and teach generated plans, that are optimized (smoothed and speed increased) since they are recorded at slow speed and with a number of stops, with th e following functionality not include,
> 2. the C SPace and Workspace features

### Revised requirement

Lite shall not contain, ship, load or expose ALIX Studio's C-space authoring and Workspace-analysis features.

- **(a) Excluded screens.**
  - The Workspace tab: Workspace Analysis, Kinematics Solver, Redundancy Atlas, Selected Point — Analysis.
  - Check Arm Reachability on the Regions tab.
  - The ψ / C-space group of the Generate dock.
  - The overlays Reach voxels, Manip heatmap, Safety zones, Unreachable, Atlas band, Ring cage and Reach dome.
  - The atlas "Pole m" input.
- **(b) Excluded routes (9).** `/api/reachability`, `/api/region-reachability`, `/api/reachable-mask`, `/api/manipulability`, `/api/redundancy-atlas`, `/api/atlas-pose`, `/api/atlas-sweep`, `/api/atlas-environment`, `/api/path-ik`. `/api/ik` is not exposed as a route or panel either; its engine runs only inside R9/R14 operations.
- **(c) Excluded code and data.**
  - X2 features in `alix_taskgen`: `cspace_author`, `cspace_pair`, `sdk_reach` and the Studio's atlas handlers.
  - X2 features in `alix_workspace`: `reachability`, `manipulability`, `redundancy`, `swivel_arcs`, `placement`, `contact` (atlas contact relaxation), `safety` (the safety-zone overlay).
  - Modules Lite does not need: `sensor_model`, `mtc_feasibility`, `fixture_validator`, `sparse_targets`, `cli`.
  - X1-coupled: `alix_workspace.coverage`.
  - Data: the `reach_*.npz` and `manip_*.npz` caches and the 15 `--atlas-*` tokens.
- **(d) Kept (not X2).**
  - Engine modules: `urdf_kinematics`, `ik`, `srs`, `srs_ik`, `collision`, `dual_arm_plan_collision`, `plan_scene_validation`, `plan_analysis`, `srdf_states`, `package_index`, `config_loader`. They are used by R3, R5, R8, R9, R10, R12 and R14.
  - Lite's own joint-space editing.
  - Optional helpers (not X2), adopted if R3/R9/R12 need them: `clearance` (robot-to-fixture distance), `wipe_kinematics` and `TG/singularity.py` (RM75 singularity screens).
  - "C-space" in X2 means ALIX's C-space path-authoring tools, not editing joint values.
- **(e) Packaging.**
  - Recommended: Lite uses the kept modules as a **vendored, pinned subset with explicit asset paths**.
  - Fallback: the installed package, with `ALIX_REPO_ROOT` pointing at a repo-shaped asset tree.
  - Lite's IK context inlines the 6-line placement transform instead of importing `alix_workspace.reachability`.
  - With the vendored subset, the import guard is strict. With the installed package, it allows `alix_workspace.reachability` only as the package-init side effect.

### Clarifications

- C-space = configuration (joint) space. In ALIX the term names the authoring pipeline that builds new cleaning paths from the arm's configuration space (`TG/cspace_author.py:1`, `TG/cspace_pair.py:1`), plus the ψ (elbow swivel) corridor tools.
- "Workspace features" means the Studio's Workspace tab: reachability voxels, manipulability heatmap, redundancy atlas. It does not mean the whole `alix_workspace` Python package, and it does not mean Lite's project folder.
- X2 removes analysis *features*, not the *engine*. Lite still needs FK, IK and collision for R9/R14 offsets and R3/R5/R10 simulation.
- `TG/singularity.py` and `WS/wipe_kinematics.py` (RealMan's four singularity types) are kinematic warnings, not C-space features. R9/R12 may reuse them.
- Per-point IK stays as an engine behind the R9 offsets. There is no user-facing solver panel.

### Findings

| # | Status | Source | Finding | Evidence |
|---|---|---|---|---|
| 1 | Verified | Studio front end | The Workspace tab panels are Workspace Analysis, Kinematics Solver, Redundancy Atlas and Selected Point — Analysis. | `WE/template.html:556-597` |
| 2 | Verified | Studio | Check Arm Reachability calls `/api/region-reachability`. The ψ / C-space group sits inside Generate. | `WE/template.html:188, 327`; `SRV:850` |
| 3 | Verified | Studio server | The X2 handlers are `do_reachability`, `do_region_reachability`, `do_reachable_mask`, `do_manipulability`, `do_atlas_sweep`, `do_atlas_pose`, `do_atlas_environment` and `do_redundancy_atlas`. | `SRV:800, 850, 866, 2060, 2182, 2412, 2509, 2568` |
| 4 | Verified | `studio_generate` | `/api/path-ik` is built on `stroke_gate` (X1) and atlas poses (X2), so R9 needs its own solver. | `TG/studio_generate.py:4243-4262`; `SRV:5955-5958` |
| 5 | Verified | `alix_taskgen` | `cspace_author` (10,999 lines), `cspace_pair` (785) and `sdk_reach` (405) are authoring and reach oracles. | `TG/cspace_author.py:1`; `TG/cspace_pair.py:1`; `TG/sdk_reach.py:1` |
| 6 | Verified | `alix_workspace` | Analysis module sizes: `reachability` 419 lines, `manipulability` 267, `redundancy` 1,247, `swivel_arcs` 669, `placement` 253, `sparse_targets` 340. | `wc -l WS/*.py` |
| 7 | Verified | Studio front end/CSS | There are 7 X2 overlay toggles and 15 `--atlas-*` tokens. | `WE/template.html:698-741`; `WE/editor.css:79-93, 169-183` |
| 8 | Verified (measured) | `alix_workspace` | The package init eagerly imports `ReachabilityAnalyzer`. Importing `alix_workspace.urdf_kinematics` loaded `alix_workspace.reachability` in 0.18-0.33 s. | `WS/__init__.py:8` |
| 9 | Corrected | `alix_workspace` | The kept modules import no X2 module, but there are two catches. `plan_analysis` needs `config_loader`. `plan_scene_validation` finds the repo root **at import** and raises outside the repo unless `ALIX_REPO_ROOT` is set; it also uses `REPO_ROOT/tools/ament_index_shim.py` for `package://` URIs. Measured: executing it with an out-of-repo `__file__` raised `RuntimeError: repo root not found`. | `WS/plan_analysis.py:21-22, 38-39, 52`; `WS/plan_scene_validation.py:31-59, 86-100` |
| 10 | New | `robot_model` | The Studio's IK context imports `alix_workspace.reachability._placement`, a 6-line pure function that is easy to inline. | `TG/robot_model.py:179`; `WS/reachability.py:47-52` |
| 11 | New | Studio server (grep) | The Studio imports from `alix_workspace`: `config_loader` 20×, `ik` 4×, `collision` 4×, `dual_arm_plan_collision` 3×, `reachability` 2×, `manipulability` 2×, `safety` 1× (inside `do_reachability`), `plan_analysis` 1×, `contact` 1× (atlas environment). It never imports `clearance` or `wipe_kinematics`; those are engine helpers, not Workspace-tab features. | `SRV:810, 2554, 4281`; `WS/clearance.py:1-8`; `WS/wipe_kinematics.py:1-10, 43` |
| 12 | New | `alix_workspace` | `alix_workspace.coverage` contains no coverage maths and imports `alix_taskgen.coverage`, so it is X1-coupled. | `WS/coverage.py:1-8, 51-62` |
| 13 | Verified | `alix_workspace` packaging | `workspace_data/*.npz` mixes the X2 caches with the kept `visual_overlay_baked.npz` (mode 600). | `A/core/python/alix_workspace/pyproject.toml:44-45`; `ls -la WS/workspace_data` |

### Borrow verdict

- **ALIX Studio: Not applicable.**
  - X2 is an exclusion. The Workspace tab, the atlas and C-space authoring are dropped.
  - The engine underneath is borrowed with adaptation, as a library.
- **Other sources.**
  - `alix_workspace` kept modules: borrow with adaptation. Use a vendored subset with explicit paths, or the installed package with a repo-shaped asset tree. Add `config_loader` to the kept list.
  - `TG/singularity.py`, `WS/wipe_kinematics.py`, `WS/clearance.py`: borrow as-is, optional.
  - Trajectory prototype and control app: not applicable.
- **Check of the input claim: accurate as an exclusion, with two clarifications.**
  - Lite still needs the FK/IK/collision engine that the Workspace panels sit on (R9 offsets, R3/R5/R10 simulation). Exclude the features, not the engine.
  - The engine is not cleanly separable today. The Studio's IK context calls `reachability._placement` (`TG/robot_model.py:179`), and `plan_scene_validation` resolves the repo at import (`WS/plan_scene_validation.py:59`). Excluding X2 cleanly therefore needs a vendored subset or a patch.
  - Lite's trim, smooth and scale edits are joint-space operations, not the excluded "C-space" feature.

### What to borrow

| Item | From | Adaptation |
|---|---|---|
| Kinematics and collision engine | The `alix_workspace` modules in (d) | Vendored, pinned subset with explicit asset paths and an explicit `PackageIndex(name_to_dir=…)`. Fallback: the installed package plus `ALIX_REPO_ROOT` / `ALIX_SRDF` / `BUTTERFLI_WORKSPACE_CONFIG` and a repo-shaped asset tree. |
| Placement transform | `WS/reachability.py:47-52` | Inline it in Lite's IK context. |
| Singularity screens (optional) | `TG/singularity.py`; `WS/wipe_kinematics.py` | Use as warnings in R9/R12. |
| Fixture clearance (optional) | `WS/clearance.py` | Near-miss distances for R3/R5. |

### What to build

| Item | Approach |
|---|---|
| X2 import guard | Extend the X1 guard with the X2 deny-list. Strict when vendored. |
| Vendored subset or environment configuration | Choose one (effort counted under NFR-OFFLINE-DEPLOY). |
| Optional upstream patches | A lazy `alix_workspace/__init__.py` (PEP 562) and an import-safe `REPO_ROOT` in `plan_scene_validation`. |
| Bundle filter | Exclude `reach_*.npz` and `manip_*.npz`. |
| Token clean-up | Drop the `--atlas-*` tokens. |

### Recommendation

Exclude the Workspace tab, the atlas and C-space authoring outright. Keep the kinematics and collision modules as a vendored, pinned subset with explicit asset paths, so Lite neither loads X2 code nor depends on an ALIX checkout. Give operators yes/no answers from the engine instead of maps.

### Approach

1. Record the kept, optional and excluded module lists in the design document.
2. Decide between the vendored subset and the installed package (recommended: vendored). Pin the ALIX commit, add a sha256 provenance header, and add a drift check that compares against ALIX when the repo is present.
3. Build Lite's IK context with the placement transform inlined. Do not copy `robot_model`.
4. Extend the import guard with the X2 deny-list (strict when vendored).
5. Build the asset bundle without the X2 caches.
6. Remove the atlas tokens and X2 overlay toggles from the lifted CSS and HTML.
7. State in the design that IK runs only inside R9/R14 operations, with no per-point IK panel.
8. Optionally, propose the two upstream patches to the ALIX maintainers.

### Acceptance criteria

- **Vendored option:** after every route has been called, `sys.modules` contains none of `reachability`, `manipulability`, `redundancy`, `swivel_arcs`, `placement`, `contact`, `safety`, `cspace_*` or `sdk_reach`.
- **Installed option:** only `alix_workspace.reachability` is present, loaded by the package init, and a spy test shows none of its functions is called.
- None of the routes in (b) exists, and there is no `/api/ik` route.
- The release contains no `reach_*.npz` or `manip_*.npz`.
- No DOM element exists for the Workspace tab, the atlas, or the reach and manipulability overlays.
- The R9 offset and R3/R5 simulation integration tests pass with the guard active.
- Vendored option: Lite's kinematics and collision modules import with no ALIX checkout and no `ALIX_*` variables, with no "repo root not found".

### UX notes

- Operators get yes/no answers from the engine ("This offset can be followed without jumps: Yes"), never voxel maps, heatmaps or atlas views.

### Safety notes

- Removing the reachability and atlas views removes no safety function. Joint-limit, continuity and collision checks live in the kept engine and must still run before real motion (NFR-SAFETY).
- Filtering modules must not drop collision geometry (bin, covers, poles, hand links). X2 removes views, not bodies.

### Dependencies

- R9/R14: offset IK uses the kept engine.
- R3/R5/R10: collision engine.
- NFR-OFFLINE-DEPLOY: vendored subset, bundle filter.
- NFR-THEME: atlas tokens dropped.

### Risks

- A vendored subset drifts from ALIX fixes to FK or collision. Pin the commit, add a diff check, and write a re-sync procedure.
- With the installed-package option, the import-time `RuntimeError` outside the repo and the eager init (0.2-0.3 s) remain.
- When an offset fails, users may ask for reach heatmaps. Give plain-language IK failure reasons instead.

### Open questions

- Vendored subset (Lite owns a copy) or the shared installed package (ALIX owns it)?
- Please confirm that no visual reachability maps are wanted. R9 keeps a per-point "can it reach" check inside the engine.

**Priority:** Must. **Effort:** S, about 1-2 days (guards, bundle filter, token clean-up). Vendoring the subset is counted under NFR-OFFLINE-DEPLOY.

---

## NFR-UX — Very user-friendly for non-technical operators

### Original text

> the lite version should be very user friendly (we are targetting non technical users)

This comes from the project brief and the user's request, not the inputs file. The closest text in the inputs file is "(non technical person) user" in R1 (`IN:8`).

### Revised requirement

A non-technical operator shall be able to complete the operator journey (Teach → Optimise → Simulate → Test on robot → Upload & Run) for a region without engineering help. Lite shall:

- **(a) Guide.**
  - Each region is one guided flow of five steps, shown as a stepper with the states locked, to do, done and needs attention.
  - Back is always available, and every step has a "What to do now" sentence.
  - The Regions dashboard is the home screen and offers "Next region to teach".
- **(b) Use plain language.**
  - Operator screens show seconds, not row numbers; degrees, not millidegrees; "Left arm" / "Right arm"; and region names from the deck.
  - They never show SDK, runner, slot, program ID, rc, Folder, NDJSON, counts/sample, MoveJ/MoveL, TOGETHER / RUN FIRST / WAIT OTHER, ψ or C-space.
  - A glossary is one click away.
- **(c) Prefer presets.**
  - Optimisation offers Gentle / Standard / Fast.
  - Pole and hand use named per-region presets.
  - Numeric parameters sit behind "Advanced" in Engineer mode. Disclosure is at most two levels deep.
- **(d) Show status as colour + glyph + word**, never colour alone.
- **(e) Show exactly one primary (filled) action per screen.**
- **(f) Use safe defaults.**
  - Lite starts in Simulation, with no arm armed.
  - The approach speed starts at the lowest preset.
  - Every optimisation preset stays within the joint ratings.
- **(g) Confirm only robot motion and controller overwrites.**
  - Edits are never confirmed, because they can be undone.
  - A motion confirmation names the arm(s), the mode (SIMULATION / REAL), the controller IP, the region or program, and the checks passed.
  - Cancel has default focus, and Enter does not confirm.
- **(h) Show an always-visible STOP** whenever any arm is connected (NFR-SAFETY).
- **(i) Offer undo for every edit:**
  - at least 5 labelled steps per region, with Ctrl+Z / Ctrl+Y;
  - robot actions are clearly stated as not undoable.
- **(j) Write errors as** what happened + what to do next (with a button where possible) + Details.
  - Controller codes are translated, for example 4109 "Arm collision detected — the arm stopped" and 4105 "A joint moved faster than allowed (warning)".
  - Translations are checked against RealMan's error list and on the bench.
- **(k) Separate Operator mode (the default) from Engineer mode.**
  - Engineer mode holds the advanced parameters, program management, IP settings and raw logs.
  - The current mode is always shown.
- **(l) Provide help.**
  - A searchable "?" panel and a tooltip on every control.
  - Dismissible first-use hints, and the deck picture on each region card.
  - No forced tutorial.
- **(m) Size targets.**
  - Targets are at least 24 px; primary and frequent controls are at least 44 px, and larger under coarse pointers.
  - All text, including chart text, is at least 14 px.
  - Screens are usable at 1366×768 without horizontal scrolling.
- **(n) Show honest progress.**
  - Nothing for under 1 s.
  - A spinner with words for 1-10 s.
  - A determinate "x of n" with time remaining above 10 s.
  - Long jobs can be cancelled.
- **(o) Save automatically.** Warn on leaving only when something is unsaved.
- **(p) Pass a usability test before release.**

### Clarifications

- Operator = the non-technical person who teaches regions and runs simulations and tests. Engineer / supervisor = sets region parameters, approves real runs and manages controller programs (requirements draft §2).
- "Guided" does not mean rigid. Operators can open any region from the dashboard; the stepper orders the steps within one region.
- The per-feature screens are specified under R1-R14. This NFR sets the rules every screen follows.
- "Robot motion" includes pole and hand moves and controller program overwrites.

### Findings

| # | Status | Source | Finding | Evidence |
|---|---|---|---|---|
| 1 | Verified | Studio front end/tests/research | There is no guided flow, by design: tabs are "Grouped by purpose, not numbered", a test forbids numbered tabs, and the research rejects forced tutorials. | `WE/template.html:116-118`; `TST/test_studio_interaction_guards.py:1207-1224`; `RES:797-801` |
| 2 | Verified | Design system | The Studio targets "robot operators, not CAD professionals", yet its screen is a dense seven-tab ribbon with expert tooltips. | `A/design/README.md:15-17`; `WE/template.html:115-125, 158` |
| 3 | Verified | Studio JS | A reusable feedback toolkit exists: `setStatus`, an error toast that stays 12 s, `setLoading`, global error capture (worded "(see console)"), and the `{error, hint}` convention. | `WE/editor.js:5431-5467, 19150-19166, 9088-9104` |
| 4 | Verified | Studio JS | The styled confirmation modal focuses the primary button on open, which is wrong for motion confirmations. A multi-choice variant exists. | `WE/editor.js:13780-13832` (`:13831`), `14849` |
| 5 | Verified | Studio JS | Tooltips (400 ms, keyboard parity) and a searchable "?" help built from them. | `WE/editor.js:7833-7845, 7946-8040` |
| 6 | Verified | Studio JS/CSS | Status carries glyph + word (gate chips, Board cells). The Board has a present/expected tab badge. | `WE/editor.js:15904-15912, 14751-14755`; `WE/editor.css:1409-1412, 1507-1509` |
| 7 | Verified | Studio CSS | The `.primary` "one action a panel exists for" idiom exists. The target tokens are 24/44 px, but 44 px is used only for Generate actions, and there is no coarse-pointer rule. | `WE/editor.css:841-852, 127-128, 1828` |
| 8 | Corrected | Studio JS | Undo is one global snapshot stack (cap 80, no labels) that captures the Studio's editor state. The pattern is reusable; the code is not. | `WE/editor.js:1896-1902, 7112-7113` |
| 9 | New | ALIX tools | `tools/layout_audit.py` runs headless Chromium (Playwright). It enforces target size ≥ `--target-min`, the type floor, no clipped controls and band height, reading its thresholds from the stylesheet. | `A/tools/layout_audit.py:1-40, 61, 177` |
| 10 | Verified | UX research | Usable rules: an operator screen separate from the programmer view; simulation by default with a separately armed hardware mode; disclosure ≤ 2 levels; rare confirmations and undo preferred; plain-language errors; async thresholds; 24/44 px targets enlarged for coarse pointers. | `RES:354-356, 378-380, 478-481, 728-731, 793-796, 883-886, 1011-1019` |
| 11 | Verified | Trajectory prototype | A good six-step stepper with lock/done states and Back/Next, but engineer parameters on the surface, a developer note in the header, row numbers and raw millidegrees, and saves through browser downloads. | `H:325, 329-341, 364-386, 526-531, 609-623` |
| 12 | Corrected | Control app (high level) | A Tkinter console that runs the runner as one subprocess per action. It has two arm consoles and a dual-arm sequence page, 87 buttons and technical labels. Most motion is confirmed with E-stop reminders; manual hand and pole apply are not. (The draft's "literal \\n" dialog detail is dropped: it is a cosmetic bug, and the user asked for no bug audit.) | `V53:2952, 3357, 3806, 3927-3929, 4167, 4203, 9089, 9116` |
| 13 | New | `alix_dispatch` | The controller's SIM mode runs on the real controller ("SIM and REAL both require hardware access"). A usability task in SIM mode therefore needs the bench controller or the ALIX emulator. | `DP/runmode.py:1-8` |
| 14 | Verified | Map 12 §5.1 | The operator subset of the control functions is already identified: arm cards, guided teach, pole/hand presets, read-only mode badge, plain checkpoints, STOP, time left, named recipes. | Map 12 §5.1 |

### Borrow verdict

- **ALIX Studio: Partial — borrow some, build the rest.**
  - Borrow: the feedback toolkit (status line, error toast, global error capture, the `{error, hint}` convention, confirmation and multi-choice modals, tooltips, "?" help, gate chips, pills, Board table and tab badge, legend, view switch, dock splitters), the undo *pattern*, the UX research rules, and the layout audit.
  - Build: the guided stepper, Operator/Engineer modes, presets, the plain-language message catalogue with controller-code translation, the glossary, the always-visible STOP bar, coarse-pointer sizing and the usability test.
  - Adapt: the confirmation modal (Cancel focus, motion facts) and undo (per region, labelled, ≥ 5 steps).
- **Other sources.**
  - Trajectory prototype: borrow with adaptation (the stepper skeleton `H:329-341, 609-623`; the status/hint idioms `H:90-110`).
  - Control app: partial. Reuse its functions and its confirmation wording with E-stop reminders, not its density or jargon.
  - Ops-notes operator subset (map 12 §5.1): borrow as-is.
- **Check of the input claim.** There is no borrow claim in the NFR text. The implicit assumption that ALIX Studio's UI already suits non-technical users is *partly accurate*: its components and rules do; its screen design (CAD-style seven-tab ribbon, expert wording, no guided flow) does not.

### What to borrow

| Item | From | Adaptation |
|---|---|---|
| Status line, error toast, busy, global error capture | `WE/editor.js:5431-5467, 19150-19166` | Replace "(see console)" with a plain sentence and a Details expander. |
| Confirmation and multi-choice modals | `WE/editor.js:13780-13832, 14849-14887` | `confirmMotion()` wrapper with Cancel focused, a plain question as title, chips for the checks passed, IP and mode, and an E-stop reminder. |
| Tooltips and "?" help | `WE/editor.js:7833-8040` | Plain-language texts plus a glossary section. |
| Gate chips, pills, Board table, tab badge | `WE/editor.js:14708-14757, 15208-15289, 15904-15950`; `WE/editor.css:1409-1433, 1507-1528, 1976-1998` | Per-region step status and the dashboard. |
| Undo pattern | `WE/editor.js:1835-1977` | Per region, ≥ 5 steps, labelled, one entry per slider gesture (new code; the snapshot is Studio-state specific). |
| Stepper | `H:220-342, 609-704` | Restyle with ALIX tokens; generalise to the five Lite steps. |
| Layout audit | `A/tools/layout_audit.py` | Point at the Lite pages. Check targets, the 14 px floor, no clipping and no horizontal scroll at 1366×768. |

### What to build

| Item | Approach |
|---|---|
| Lite shell | App bar (brand, mode banner, role chip, STOP), left step rail, content area, status line. |
| Stepper component | Step states + a "What to do now" panel. |
| Operator / Engineer mode | PIN-gated switch; the mode chip is always visible. |
| Message catalogue | All strings in one file, tagged operator or engineer; an error template; a controller-code table (4105, 4109, the upload return codes `R14:50-55`, connection errors); a blocked-terms lint test. |
| Presets | Optimise, pole, hand, approach speed. |
| Glossary panel | Built from the requirements glossary. |
| UI checks | One primary per screen, text on every status, target sizes, coarse-pointer CSS (with the adapted layout audit). |
| Usability-test kit | Tasks, script, consent, metrics sheet. Run on the emulator, or on the bench with an engineer at the E-stop. |

### Recommendation

Build Lite around one guided operator journey with a dashboard home. Reuse ALIX Studio's feedback components, rules and layout audit. Keep every number behind presets and Engineer mode. Make release depend on a measured usability test.

### Approach

1. Write the operator and engineer task lists (map 12 §5.1 and the draft journey).
2. Wireframe the dashboard and the five step screens at 1366×768, using the ALIX design-system cards (`A/design/components/*.html`) as reference. Review with the requester and one operator.
3. Build the shell, stepper, STOP bar and mode banner from lifted ALIX components.
4. Create the message catalogue, the blocked-terms lint and the controller-code translations.
5. Implement presets and Engineer mode, and move every numeric parameter behind Advanced.
6. Implement `confirmMotion()` and an inventory of which actions confirm.
7. Wire per-region undo (with R2) and auto-save (with NFR-DATA).
8. Adapt the layout audit and add the DOM checks to CI.
9. Pilot with 2 people and fix. Run the formal test with ≥ 5 people, fix severity 3-4 issues, then re-test.

### Acceptance criteria

- **Usability test** with ≥ 5 non-technical participants who have not used the ALIX tools or the control app. Six tasks:
  1. find the untaught regions;
  2. teach one region with the right pole/hand preset;
  3. optimise with a preset and undo one change;
  4. run the simulation and say whether and when a collision happens;
  5. run a region in the controller's SIM mode (emulator, or bench controller with an engineer at the E-stop);
  6. stop a running arm.

  Pass levels: each task is completed unaided by ≥ 4 of 5 participants; zero critical errors (unintended motion, wrong arm, wrong program uploaded); STOP is pressed within 3 s of the prompt in every trial; mean SUS ≥ 70.
- No operator-tagged string contains a term blocked in (b) (automated lint).
- The adapted layout audit passes on every screen at 1366×768 and 1920×1080. No clipped control, no text below 14 px, all interactive elements ≥ 24×24 px, primary and STOP ≥ 44 px, exactly one visible `button.primary`, every status element has a glyph and a text label, and no horizontal scroll.
- Every motion or overwrite action calls `confirmMotion()`, and no edit action opens a dialog (inventory test).
- Every catalogue error has a non-empty "what to do". Codes 4105 and 4109 map to plain sentences.
- Undo offers ≥ 5 labelled steps per region. Ctrl+Z restores the previous recipe and plot within 1 s.
- Start-up lands on the dashboard, in Simulation, in Operator role.

### UX notes

- Use operator words: "Teach", "Clean the seat top (left half) for 15 s", "Check in simulation", "Try on the robot", "Send to robot".
- Show the deck picture with the region highlighted on every region card. Arm identity is shown by the words LEFT/RIGHT plus an identity colour, never by colour alone (NFR-THEME (f)).
- Put the one primary button in the same place on every step: bottom right of the step panel.
- Never show a number without its unit. Show times as m:ss.
- Dialog titles are plain questions ("Run the LEFT arm on the real robot?"). Facts go in a short list. Buttons are named after the action ("Run LEFT arm", "Cancel").
- Empty states teach: "No recording yet — press Start teaching".

### Safety notes

- Keep motion confirmations rare (motion and overwrites only) and specific (arm and mode named). Never focus the confirm button by default.
- STOP never asks for confirmation and is never hidden behind a dialog or scrolled away.
- Engineer mode is never the default and is always visibly shown, because it exposes speeds and overrides.

### Dependencies

- NFR-SAFETY: STOP, arming, confirmation facts.
- NFR-THEME: components and tokens.
- R1: dashboard, guided teach, deck pictures.
- R2: per-region undo.
- R4: mode banner, IP settings in Engineer mode.
- NFR-DATA: auto-save.
- INT-TRAJ and INT-CTRL: screens rebuilt from the prototype and the control app.
- The requirements glossary.

### Risks

- A rigid wizard frustrates repeat users. Keep the dashboard jumps and Back (`RES:478-485`).
- Hiding too much in Engineer mode can strand operators. Every error needs an operator-level next step ("Call the engineer" is acceptable).
- Controller-error translations can be wrong. Keep the raw code in Details.
- The usability test needs real operators and a safe setup (emulator, or bench with an engineer at the E-stop). Booking them takes calendar time.

### Open questions

- Who are the operators (language, reading level, robot experience)? English only?
- Mouse and keyboard, or touchscreen? What is the operator PC's screen size?
- Who may switch to Engineer mode, and how (PIN, OS account)?
- Can 5 operators be scheduled for a usability test, and on what setup?

**Priority:** Must. **Effort:** L, about 12-15 days for the cross-cutting UX framework and one usability-test cycle with fixes. This is at the top of L; per-feature screens are estimated under R1-R14.

---

## NFR-THEME — Keep the ALIX Studio theme and formatting

### Original text

> the lite version should also maintain the theme and formatting used in the Alix studio

This comes from the project brief and the user's request, not the inputs file.

### Revised requirement

Every Lite screen, including the ported trajectory editor and the robot-control functions, shall use the ALIX Studio visual language:

- **(a) One shared token file.**
  - `alix_tokens.css` is copied from ALIX `design/_tokens.css` with identical values (light default; dark via `[data-theme="dark"]`).
  - It adds the `editor.css` tokens Lite needs (`--accent`, `--accent-soft`, `--brand-seam`, `--surface-bar`, `--surface-dock`, `--surface-ribbon`) and a documented "Lite additions" block.
  - A parity test against `design/_tokens.css` (or, without the ALIX repo, against the vendored snapshot's sha256) fails on drift.
  - No colour, font, radius or shadow literal appears outside the token file.
- **(b) Typography.**
  - Inter for UI text and JetBrains Mono for numbers, IDs and paths, shipped as local woff2 files with their OFL licences.
  - Type scale 14 / 16 / 17 / 19 / 22 px, with hierarchy carried by weight.
  - Nothing below 14 px, chart text included.
- **(c) Brand.** The inline-SVG butterfli mark and the title "ALIX Studio Lite" in the app bar.
- **(d) Icons.** ALIX's inline-SVG line icons (`currentColor`) and the state glyphs ✓ △ ✗ · ⓘ.
- **(e) Components.** ALIX base controls and shell components, lifted with the known defects fixed.
- **(f) Colour discipline (ISA-101).**
  - About 90% of the screen is neutral.
  - Brand blue is used only for selection and the primary action.
  - Green, amber and red mean state only, always with a glyph or word. STOP uses the danger colour.
  - **Arm identity colours** (deck: blue LEFT, orange RIGHT) are identity tokens. They must be visibly different from brand blue `#2F96EE` and warning amber `#C77700`, and they always come with the words LEFT/RIGHT.
- **(g) Charts.**
  - A `plotTheme()` helper builds every Plotly layout from tokens in both themes.
  - A documented seven-colour joint palette that avoids the state red, green and amber and the brand blue.
  - Collision and limit markers use a state colour plus a text label.
- **(h) Retire other palettes.**
  - Not used: the trajectory prototype's palette (accent `#175cd3`, dark header `#0b1220`, 8-12 px radii), the Tk app's neon palette (`#06111F`, `#00E5FF` …), and the Studio's own leftover JS literals (old cyan `#00d4ff`, GitHub-dark hues).
  - Their meanings map to ALIX tokens. Any Tk tool kept during the transition takes generated constants from the same token source.
- **(i) Accessibility.**
  - WCAG AA contrast in both themes.
  - Where a shared ALIX token fails (white on `--brand` for primary buttons, `--ok` and `--warning` as text, `--muted` on `--surface-inset`, white on dark-theme `--brand-strong`), Lite adds an accessible variant token rather than silently changing the shared value.
- **(j) Theme switching.**
  - Light by default, with a user toggle for dark that is remembered per browser.
  - The 3-D scene and chart colours update on toggle.

### Clarifications

- "Theme" means the colour, typography, spacing and shape tokens plus component styling. "Formatting" means the layout conventions: sentence-case labels, label-left fields, mono numbers, 4 px radius, tonal elevation, status vocabulary. It does not mean the CAD-style ribbon or command band.
- The Ops notes ask for a "Dark robotics / control-console theme" (`OPS:227-237`). ALIX's own dark variant meets that intent; the neon Tk palette does not. The default theme is an open question.
- The Studio has a dark theme, but there is no way to switch it on in the Studio today.

### Findings

| # | Status | Source | Finding | Evidence |
|---|---|---|---|---|
| 1 | Verified | Studio CSS / design system / tests | Light `:root` and dark `:root[data-theme="dark"]` tokens, a type scale, the authored source `design/_tokens.css`, and parity tests. | `WE/editor.css:1-129, 134-189, 1714-1724`; `A/design/_tokens.css:8-55`; `TST/test_design_tokens.py:80-141` |
| 2 | Corrected | Tests | The parity test is one-directional (every design token must exist in the app). `design/_tokens.css` has **no** atlas tokens, so a Lite parity test against it needs no atlas exclusion. `--accent` and `--accent-soft` exist only in `editor.css`, yet the primary-button and modal rules use them. | `TST/test_design_tokens.py:1-25, 80-105`; `A/design/_tokens.css:8-55`; `WE/editor.css:39-40, 150-151, 741, 854` |
| 3 | Verified | Brand | `--brand #2F96EE` measured from butterfli.ai; an inline-SVG mark; title and h1 pinned by tests. | `WE/editor.css:1-22`; `WE/template.html:38-52`; `TST/test_studio_theme.py:38-43, 179-182` |
| 4 | Verified | Design rules | Brand is for selection and the primary action only; ~90% neutral; every state pairs colour with a glyph; light default. | `A/design/_tokens.css:1-10`; `A/design/README.md:19-32` |
| 5 | Verified | Studio JS | Dark mode is not wired: no `data-theme` setter and no `prefers-color-scheme` handling. `syncColorsFromCss()` runs once at start-up. | grep = 0; `WE/editor.css:131-134`; `WE/editor.js:185, 311` |
| 6 | Verified (measured) | Studio CSS; this PC | Webfonts are not shipped (no `@font-face`), and Inter and JetBrains Mono are not installed here (`fc-list` = 0). | `WE/editor.css:109-110, 191` |
| 7 | Corrected | Studio CSS | **Most listed defects sit in X1/X2 rules Lite drops anyway:** `--radius-lg` in `.gen-dock` (`:386-398`), `--line` / `--text-muted` in the atlas rules (`:2096-2116`), and `--text-dim` / `--surface-2` / `--warn` plus the literal `"\\26A0\\FE0E"` in the Generate dock's hand-state rules (`:2126-2154`). The defects that matter for Lite: the `.info-row` block opened at `:1016` nests rules `:1017-1027`; `.pill-ok` uses a literal colour (`:1986`); `html, body` restates the font stack (`:191`); `button.primary` uses a literal `#fff` (`:847-852`); `.select-inline` is 11 px with dark fallbacks (`:2158-2166`), if the visibility bar is lifted. | `WE/editor.css` lines as given |
| 8 | New | Studio JS | Colour literals in JS bypass the theme. The schedule timeline uses the old cyan `#00d4ff` plus GitHub-dark hues (`SCH_TYPE_COLOR`). The DAC plot uses `#ff4d4d` and `rgba(0,212,255,0.25)`. `editor.js` holds 18 hex literals. ALIX's guard only checks `#00d4ff` in CSS and `0x00d4ff` in JS, so all of these pass it. | `WE/editor.js:13593-13594, 14485`; `TST/test_studio_theme.py:86-89` |
| 9 | Verified + New | Studio JS | Both Plotly charts hard-code the dark-theme text colour `#e6edf3` (1.18:1 on white, invisible in the default theme) at **font size 10 px**, below the 14 px floor. | `WE/editor.js:5368, 14483-14491` |
| 10 | Verified + New (computed) | Contrast | White on `--brand` 3.13:1 (primary buttons). White on `--brand-strong` 4.17:1 (light) and **2.52:1 (dark hover)**. `--ok` text 3.41:1; `--warning` text 3.46:1; `--danger` 4.56:1; `--muted` 4.74:1 on white but **4.37:1 on `--surface-inset`**. AA candidates: `#1A75C4` (4.79:1) or `#1872BF` (5.00:1) primary fill; `#9A5A00` (5.47:1) warning text; `#1F7A45` (5.35:1) ok text. | `WE/editor.css:22-44, 135-155, 847-853` |
| 11 | New | Deck vs tokens | The deck's arm colours (blue LEFT, orange RIGHT) sit close to brand blue (reserved for selection) and warning amber (state). Used as-is, they would blur "selected" and "warning". | Map 13 §0; `A/design/_tokens.css:9-10, 20` |
| 12 | Verified | Studio JS | Icons are inline SVG with a 16 px viewBox, 1.6 stroke and `currentColor`. | `WE/editor.js:7656-7664, 7705, 7736, 7791` |
| 13 | Verified | Trajectory prototype | It has its own palette and a dark header, fixed plot heights of 600/520 px, category10 joint colours (J3 green, J4 red) and Plotly from a CDN. It reuses ALIX token *names* with different values, so its CSS must be namespaced. | `H:7, 9-21, 29-31, 125-129, 577-585` |
| 14 | Verified | Control app; Ops notes | Neon Tk palette. The Ops notes ask for a dark console theme. | `V53:56-71`; `OPS:227-237` |
| 15 | Verified | Tests | Portable guards: brand value, tokens on `:root`, dark redefines the surfaces, viewport not white, inline-SVG brand, no remote `<link>`/`<script>`/`<img>`. | `TST/test_studio_theme.py:38-91, 185-195` |
| 16 | Verified | File system | `A/design/_tokens.css` has mode 600. | `ls -la A/design` |

### Borrow verdict

- **ALIX Studio: Borrow with adaptation.**
  - Lift: the tokens (light and dark, plus `--accent` / `--accent-soft`), the brand mark, the icon set, the base and shell components, and the theme guard tests.
  - Adapt: filter out the X1/X2 rules and atlas tokens (this also removes most of the defects); fix the remaining defects; replace the JS colour literals; add local webfonts, `plotTheme()` and a joint palette, accessible variant tokens, distinct arm-identity tokens, and a working dark toggle that re-syncs the 3-D and chart colours.
  - Do not lift the Fusion-style command band.
- **Other sources.**
  - ALIX design system (`A/design/_tokens.css`, the cards): borrow as-is.
  - Trajectory prototype: not applicable (palette retired; restyled under INT-TRAJ).
  - Control-app Tk palette: not applicable (retired; keep only its colour meanings).
- **Check of the input claim: accurate and feasible.** ALIX Studio has a clean, test-pinned token system. Caveats:
  - "The theme" is effectively the light theme; dark exists but cannot be switched on.
  - Some components, and especially the JS charts and the timeline, carry off-theme literals that ALIX's own guard misses.
  - "Formatting" should mean the visual language, not the CAD ribbon.
  - It conflicts with the Ops notes' dark console theme unless the ALIX dark variant is offered.

### What to borrow

| Item | From | Adaptation |
|---|---|---|
| Tokens | `A/design/_tokens.css:8-55`; `WE/editor.css:28, 39-40, 65-68` (light), `:137, 150-151, 160-163` (dark) | Verbatim values. Drop atlas, `--sidebar-width`, `--gen-w`, `--ribbon-h`. Add the "Lite additions" block. |
| Brand mark and app bar | `WE/template.html:38-52`; `WE/editor.css:196-202` | Title "ALIX Studio Lite"; the badge shows "Simulation" or "Real robot · <IP>". |
| Icons | `WE/editor.js:7656-7741` | As-is; add teach, stop and upload icons in the same style. |
| Base and shell CSS | Ranges in map 01 §8.3 | Fix the `.info-row` nesting, `.pill-ok`, the body font and the `#fff` literal; add `button.danger` and `.hint.error`. |
| Theme tests | `TST/test_design_tokens.py`, `TST/test_studio_theme.py` (subset) | Retarget to Lite. The literal scanner covers JS hex **and** `rgb()/rgba()`. |

### What to build

| Item | Approach |
|---|---|
| One token source | `alix_tokens.css` plus a generated `alix_tokens.py` for any Python/Tk output. |
| Local fonts | Inter and JetBrains Mono woff2 with `@font-face` and the OFL licence files. |
| Chart theme | `plotTheme()` (font ≥ 14 px, tick and grid colours from tokens), the joint palette, the collision-marker style; re-applied on theme change. |
| Theme toggle | App-bar menu; `localStorage` key `alixLite.theme` inside try/catch; re-run `syncColorsFromCss()`. |
| Accessible and identity tokens | `--primary-fill`, `--ok-text`, `--warning-text`, `--arm-left`, `--arm-right`, with contrast and distinctness tests. |
| Palette mapping table | For the ported trajectory and control screens and the lifted timeline. |

### Recommendation

Copy the ALIX token file and components rather than re-creating them. Keep one token source for everything Lite renders. Fix the few defects that survive the X1/X2 filtering. Replace every JS colour literal. Add what the Studio lacks: local fonts, a chart theme helper, distinct arm-identity colours and a working dark toggle.

### Approach

1. Copy `design/_tokens.css` into `lite/web/theme/alix_tokens.css` (without the `.ds*` rules). Append the needed `editor.css`-only tokens and the "Lite additions" block.
2. Port `test_design_tokens.py` to compare Lite's file with `design/_tokens.css`, or with the vendored snapshot's hash. Add a literal scanner for Lite's CSS **and JS**.
3. Lift the base and shell CSS by the map 01 §8.3 ranges, dropping X1/X2 rules. Fix the surviving defects and add `button.danger` and `.hint.error`.
4. Vendor the two fonts with `@font-face` and licences.
5. Write `plotTheme()` and the joint palette. Convert every chart (the ported trajectory plots and the collision timeline) and the schedule timeline colours. Add contrast tests.
6. Restyle the trajectory and control screens: remove their own `:root` palettes, namespace their classes (`.traj-*`), and map their roles to ALIX tokens.
7. Add the theme toggle and the 3-D/chart re-sync.
8. Check contrast and identity distinctness in both themes. Review with the requester, and with the ALIX design owner if a shared token should change.

### Acceptance criteria

- The token parity test passes against `design/_tokens.css` (or the recorded snapshot hash).
- A scan of Lite's CSS and JS finds no hex, `rgb()` or `rgba()` colour literal outside `alix_tokens.css` and `plotTheme()` (allow-list for SVG path data).
- With the network off and the fonts not installed, body text renders in the bundled Inter (`document.fonts.check` passes).
- In every Plotly chart, tick labels reach ≥ 4.5:1 contrast against the plot background in light and dark, at font size ≥ 14 px.
- Switching theme updates the panels, the 3-D scene and the charts without a reload.
- All text-on-fill pairs used by buttons (including hover) and status pills reach ≥ 4.5:1 in both themes (automated table test).
- The LEFT and RIGHT identity colours differ from brand blue and warning amber by a documented minimum (for example ΔE ≥ 20) and are always shown with their words.
- Side-by-side review: Lite and ALIX Studio share the app bar, brand mark, buttons, pills and fonts (sign-off by the requester).

### UX notes

- Light by default (ALIX research: dense text reads better on light). Dark is offered for control-room use.
- The 3-D viewport stays dark in both themes (ALIX rule), so the robot and commode stand out.
- STOP is the only large red element on screen.

### Safety notes

- Red is reserved for problems and STOP, so a red mark always means "look here". Joint lines are never red.
- The REAL-mode banner does not rely on colour alone: it shows the words "REAL ROBOT" and an icon.
- An arm's identity colour must never be confused with "warning" or "selected"; the wrong-arm risk is reduced by always naming the arm.

### Dependencies

- NFR-OFFLINE-DEPLOY: fonts and libraries shipped locally.
- INT-TRAJ: restyle of the trajectory editor.
- INT-CTRL: retiring the Tk palette.
- R5/R6/R10/R12: chart theme, joint palette, collision markers.
- R8/R10: timeline colours.
- NFR-UX.

### Risks

- Fixing contrast by changing shared token values would also change ALIX Studio and break its parity test. Use Lite-only variant tokens, or agree a joint change.
- The ported trajectory CSS uses the same custom-property names with different values. Without namespacing it overrides the ALIX tokens.
- The Studio reads 3-D colours once at start-up. Without a re-sync, colours go stale after a theme switch.
- The font licences (OFL) must ship with the fonts.

### Open questions

- Default theme on the operator PC: ALIX light (research) or dark (Ops notes)?
- May Lite add accessible variant tokens, or should `design/_tokens.css` change for both products?
- Arm identity colours: keep the deck's blue/orange (with words), or adopt distinct ALIX identity hues?
- Is the product name and badge text "ALIX Studio Lite"?

**Priority:** Must. **Effort:** M, about 5-7 days. This is lower than the draft's 6-8 because most CSS defects disappear with the X1/X2 filtering. The full restyle of the trajectory editor is counted under INT-TRAJ.

---

## NFR-SAFETY — Safe robot operation (new — recommended)

### Original text

(new — recommended). The inputs require real-robot runs (R4), uploads (R13) and pole/hand moves (R7), but they state no safety rules.

### Revised requirement

Lite shall move a robot (arm, pole or hand) only when the operator clearly intends it. It shall always offer an immediate stop, and it shall refuse programs that have not passed its checks.

- **(a) Safe start.**
  - Lite starts in Simulation with no arm armed.
  - Real-robot mode is armed per arm and per session, after a connection check, a run-mode read and an explicit confirmation.
  - While an arm is armed, the app bar shows "REAL ROBOT — LEFT 192.168.1.10" as word + icon + colour.
  - Arming lapses after a configurable idle time and on disconnect.
- **(b) One motion gateway.**
  - Every motion command passes through one server-side gateway: program run, sequence, move-to-start, drag-teach start, playback, pole move, hand move, global-point move.
  - The gateway checks arming, interlocks and gates, and requires a single-use confirmation token bound to that exact command. It logs every command.
- **(c) Stop.**
  - An always-visible STOP ALL (both arms, poles and hands: arm slow stop, lift speed 0, hand freeze) and a per-arm STOP. One click, no confirmation, working during any job and when idle.
  - STOP goes through the connection that owns the arm.
  - The stop command reaches the controller within 300 ms of the click (bench-measured).
  - After a stop, Lite reads the program run state and the joint motion to confirm it. A slow stop decelerates along the path, and the SDK has no program-scoped stop.
  - If the stop is not confirmed within 1 s, Lite shows "Press the hardware E-STOP now".
  - Lite labels its STOP as a software stop, not an emergency stop.
- **(d) Gates before real motion.** A trajectory may run on the real robot only if its exact bytes (sha256) passed two checks:
  1. **File checks:** format, 7 integers per line, joint limits with margin, ≤ 10° per line, and joint speeds within ratings at the controller's fastest 5.0 ms tick.
  2. **A 3-D simulation with collision checking** against the matching world: both arms, the commode in the region's fixture state, the **declared bin state**, pole height, walls, and the controller's implicit approach move from the arm's measured pose to the first point.

  Any edit makes earlier passes stale. An engineer override needs a typed reason and is logged.
- **(e) Interlocks.**
  - An arm reserved by a running job cannot be commanded from any other screen.
  - Pole moves are refused inside the stowed-bin foul band (0.153-0.282 m with the bin STOWED). This band is provisional until re-measured on the physical bin.
  - The pole moves only between programs, with the arm at rest. Its arrival is judged with a settle margin.
  - Pole and hand move only with the pad off the surface (the operator confirms).
  - No motion while a blocking controller error is active (4105 is a warning only).
  - Only one arm is in drag-teach at a time, unless dual teach is chosen.
- **(f) Speed and approach.**
  - Transit (approach) speed defaults to ≤ 30%.
  - The size of the approach move is shown before a run. Above a site threshold, the run is refused until the arm has been moved to a checked start pose.
  - The controller's global speed is restored to the site default after every run and on every reconnect.
- **(g) Dual-arm runs.**
  - If one arm fails or stops, the other arm is stopped by default.
  - A stop made outside Lite is reported as "Stopped outside Lite", never as "Complete".
- **(h) Honest labels and SIM-mode rules.**
  - The 3-D verdict lists what was checked (arms, commode state, bin, walls, hands).
  - The controller's SIM mode is described as "controller dry run — the arm does not move; pole and hand are not simulated".
  - In SIM runs, Lite **skips pole and hand commands**, because the real pole and hand would move, and it says so.
  - A SIM run never counts as a robot-test pass.
- **(i) Network.**
  - Lite's server listens on 127.0.0.1 by default.
  - Every state-changing request needs the session token in a custom header and must pass Host and Origin checks. Binding to localhost alone does not stop another web page in the operator's browser from posting to Lite.
  - Listening on other interfaces requires configured authentication.
- **(j) Records.** Every motion, confirmation, stop, error code, override and run mode (SIM/REAL) goes into the project run log, with time, role, arm, controller IP, program or region and sha256.
- **(k) Controller programs.**
  - Uploads go only to the slot range configured for the site and arm.
  - An occupied slot is never overwritten without a confirmation that shows what it holds.
  - Before a run by program ID, the slot's listed name and size must match Lite's upload ledger entry for a passed sha256.
- **(l) One command source per arm.**
  - While an arm is armed, Lite's back end is its only command source, through one long-lived connection per arm.
  - The operator confirms that the ROS driver, the Tk control app and the vendor web UI are not driving the arm.
  - Every command checks that the connection's arm matches the region's arm.
- **(m) Session teardown.** On session end or crash recovery, Lite restores the run mode, tool frame and speed settings it changed.

### Clarifications

- This is a new requirement recommended by this analysis.
- The software STOP is not an emergency stop. The physical E-stop and the site risk assessment remain the primary protection (the control app already says so, `V53:3177-3178`).
- "Motion" includes the pole (lift) and the hand (fingers), not only the arm.
- "Simulation pass" means Lite's 3-D check (R3/R5). The controller's SIM mode is a separate dry run on the real controller.
- 300 ms, 30%, the approach threshold and the idle time are proposed defaults to confirm on the bench.

### Findings

| # | Status | Source | Finding | Evidence |
|---|---|---|---|---|
| 1 | Verified | Studio | The Studio never commands hardware ("HONEST: the studio has no execution channel"), and a test pins that. There is nothing to borrow for arming, stop or interlocks. | `WE/editor.js:13045-13047`; `TST/test_studio_interaction_guards.py:2207-2215` |
| 2 | Verified + New | Studio server | It binds to 127.0.0.1 by default and warns about other bindings. It has **no Origin, Host or token check** (grep), which is fine for an authoring tool but not for motion endpoints. | `SRV:6510-6544` |
| 3 | Verified | Studio JS | Helpful patterns: a confirmation modal with destination and gate chips; stale-on-edit (`markDirty` → `invalidateValidation`); honest preview labels; the foul-band warning text. | `WE/editor.js:13780-13832, 5371-5389, 17144-17186`; `WE/template.html:889` |
| 4 | Verified | Control app (high level) | SLOW STOP sends SIGINT to the runner's process group, and the runner calls `rm_set_arm_slow_stop`. There is no confirmation by design, and the UI tells the user to use the hardware E-stop if signalling fails. With no live runner the button does nothing to the robot. | `V53:6028-6071` (`:6036-6044`); `R14:245-260` |
| 5 | Verified | Helpers | `stop_all` = arm slow stop + pole stop (`rm_set_lift_speed(0)`) + hand freeze. Rule: "the pole moves only with the arm still; never move pole or hand with the pad on a surface". | `UC/usage.py:25, 141-150`; `UC/pole.py:26-27`; `UC/hand.py:123-126` |
| 6 | Verified | Control app | Most motion is confirmed with E-stop reminders, but manual hand and pole apply are not ("APPLY itself is the explicit motion command"). Manual pole accepts 0-315 mm with no bin check. | `V53:3357, 3806, 3927-3929, 4167, 4192-4203` |
| 7 | Verified | Runner v14; `alix_dispatch` | The runner refuses an empty slot, an arm error or a running program. It names 4105 (warning) and 4109 (ARM COLLISION), and restores the global speed after runs. ALIX documents the same speed hazard. | `R14:26-35, 56-57, 189-205, 258-260`; `DP/track.py:20-24` |
| 8 | New | ALIX motion docs; runner | **One command source per arm:** "Two TCP clients = two command sources — not advisable. ACCEPTED AS AXIOM." The runner's own header says "nothing else may be driving it; stop the ROS driver first". | `DTC:220-233`; `R14:26` |
| 9 | New | ALIX motion docs | There is no program-scoped stop or pause API, only motion-scoped `set_arm_stop` / `set_arm_slow_stop` (on-path) / pause. A `rm_set_program_id_run` return of -4 means something else stopped it. | `DTC:24-28` |
| 10 | New | `alix_dispatch.runmode` / `session` | The controller's SIM mode runs on hardware and simulates **neither the pole nor the hands**, so a SIM run must skip those commands. An arm left in SIM is a "phantom" that still scores complete. An unreadable mode is refused, and arms in different modes are refused. | `DP/session.py:1-8`; `DP/runmode.py:178-260` (`:205-215`, `:225-235`) |
| 11 | New | `alix_dispatch.session` | Session teardown restores the tool frame, the run mode and the speed limits ("global controller state shared with the Web GUI … a leaked one persists across a reboot"). | `DP/session.py:217-265` |
| 12 | New | `alix_dispatch.track` | Ready-made guards: `global_speed_guard` (restores on any exit), `start_gap_deg` (size of the controller's implicit approach move), `assert_side` (wrong-arm refusal), `slow_stop`. ALIX accepts the implicit approach move only with the gap announced, and offers a strict refuse mode. | `DP/track.py:161-200, 323-350, 2274-2285` |
| 13 | New | `alix_dispatch.peripherals` | A lift command landing while a planned arm move is in flight **aborts that move and latches joint errors**. `rm_get_lift_state` leads the physical pole by up to 73 mm. So the pole is commanded only with the arm at rest, between programs. | `DP/peripherals.py:18-24` |
| 14 | Verified | Orchestrator; V6 | With the bin STOWED the arm base fouls the bin lid covers for pole 0.158-0.277 m (0.153-0.282 m with margin), and "NO ARM MOTION AVOIDS THIS". Both V6 FINAL workflows start at pole 200 mm (then 150/50 mm). | `ORCH/core/task_config.py:58-124`; `V6/LEFT_V6_70SERIES_FINAL.json`, `RIGHT_V6_70SERIES_FINAL.json` steps[0] |
| 15 | Corrected | exp01 teach table | The commode-C teach table uses **140 mm** (lid top, bowl outside, seat top, rim top) **and 200 mm** (bowl inside, rim side, inner back, hinge, lid-seat open, seat only). The 200 mm rows lie inside the band. | `A/experiments/exp01_teach_trajectory_reverse_engineering/data/teach_files/commode_c_taught_paths/description.txt:3-15` |
| 16 | New | Robot description | The foul band was measured on the BinShort model. The xacro notes "the PHYSICAL part has not been remade", so the band is provisional for the real bin. | `ORCH/core/task_config.py:63-67`; `A/ros/description/ugv_description/urdf/bin.xacro:32-34` |
| 17 | Corrected | Collision code | **Both** checkers hard-code the bin LOWERED: the DAC sweep (with no commode, because there is no `extra_managers` caller) and BodyCollisionWorld (whose entry point takes generated task configs). The "matching world" gate needs a bin-state parameter and a taught-track entry point. | `WS/dual_arm_plan_collision.py:596-611, 699`; `SRV:1135, 1529, 1725, 1991`; `TG/body_collision.py:499-530, 627-631` |
| 18 | Verified | ALIX taught lane | Certificates are bound to the bytes by sha256; upload and run re-check them. The enforced gates include joint margin, zero ticks over a joint rating at the fast clock, ≤ 10° per line and deviation ≤ 10 mm. | `A/tools/taught_tracks.py:29-32, 42-47` |
| 19 | New | Runner v14 | Upload verification compares only the stored **size** (± max(64 B, 0.1%)) with the bytes sent. The controller cannot return a hash. | `R14:147-175` |
| 20 | Verified | Runner v14 | Run mode SIM (0) / REAL (1) is set and read back: a dry run, not a 3-D simulation. | `R14:1213-1240` |
| 21 | Verified | Slot policies | The conventions conflict: ALIX reserves 70-100; V6 uses 71-83; the runner README says 40-59; v53 defaults to 25 and 99. | `DP/track.py:13-16`; `V6/README.txt:101-104`; `UC/README_COMMANDS.md:58`; map 08 §4.5 |
| 22 | Verified | Maps 08/12 (design gaps) | Lite must not inherit three control-app gaps: no interlock between Page 02 RUN BOTH and Page 01; a failure on one arm does not stop the other in a parallel stage; a pendant stop with no error code reads as completion. | Map 08 §5; map 12 §4 |

### Borrow verdict

- **ALIX Studio: Partial — borrow some, build the rest.**
  - The Studio contributes UI and process patterns only: the confirmation modal, stale-on-edit, honest labels, the foul-band warning text and the loopback default.
  - It never commands hardware. Arming, the gateway, STOP, interlocks, run-mode handling, speed restore and dual-arm handling come from the control app, **`alix_dispatch`** and new work.
- **Other sources.**
  - Control app v53 + runner v14: borrow with adaptation (SIGINT → slow stop, refusal checks, speed restore, REAL confirmation, E-stop wording; close the hand/pole confirmation and interlock gaps).
  - `usage.py` / `pole.py` / `hand.py`: borrow as-is (`stop_all`, the pole/hand rule).
  - **`alix_dispatch` (`runmode`, `session`, `track`, `peripherals`, `verdict`): borrow with adaptation. Vendor the subset, since it has no `pyproject.toml`.**
  - ALIX taught lane: borrow with adaptation (sha256-bound certificates, gates).
  - `ORCH/core/task_config.py`: **copy** the 20-line foul-band function with provenance; do not import from the ROS tree. ALIX itself keeps copies (`WE/editor.js:17144-17150`).
  - ALIX emulator + `DP/emulated_program.py`: borrow with adaptation, for automated tests without hardware.
- **Check of the input claim.** There is no claim (new requirement). Related claims:
  - R4's "toggle between real and sim" refers to the controller's dry-run mode on hardware. It is not a 3-D simulation, and pole and hand still move in it unless they are skipped.
  - R5's "borrow from ALIX Studio": both of ALIX's checkers fix the bin as lowered and the DAC sweep omits the commode, so their verdicts cannot gate real runs without adaptation.

### What to borrow

| Item | From | Adaptation |
|---|---|---|
| Confirmation modal | `WE/editor.js:13780-13832` | `confirmMotion()`: Cancel focused; arm, mode, IP, checks passed. |
| Stale-on-edit | `WE/editor.js:5371-5389` | Pass records keyed by sha256. |
| Stop composition | `UC/usage.py:141-150`; `UC/pole.py:26-27`; `UC/hand.py:123-126`; `DP/track.py:2274-2285` | Per arm, issued through the connection owner; confirmed by reading the run state. |
| Run-mode rules | `DP/runmode.py:178-260`; `DP/session.py:1-8, 182-188` | Read before each run; refuse unknown or mixed modes; skip pole and hand in SIM; record the mode. |
| Teardown restore | `DP/session.py:217-265` | Restore run mode, tool frame and speed on session end and recovery. |
| Speed guard, approach gap, wrong-arm check | `DP/track.py:161-200, 323-350` | Inside the gateway; approach threshold per site. |
| Pole rules | `DP/peripherals.py:18-24, 131-148`; `UC/pole.py` | Pole only between programs with the arm at rest; arrival with margin. |
| Refusals, error codes | `R14:26-35, 56-57, 189-205` | Inside the gateway. |
| Foul band | `ORCH/core/task_config.py:58-124` | Copied with provenance; called before every pole move with the declared bin state. |
| Certificates and gates | `A/tools/taught_tracks.py:29-47` | Lite pass records; site slot policy. |

### What to build

| Item | Approach |
|---|---|
| Risk assessment | A hazard → control → test table with the requester and site safety. |
| Connection owner | One long-lived connection per arm in Lite's back end; command queue; STOP path; interlock against other command sources. |
| Session state machine | DISCONNECTED → CONNECTED (SIM) → ARMED_REAL → BUSY, with idle auto-disarm and the banner. |
| Motion gateway | Single-use tokens (60 s, bound to a hash of the command), arm reservation, error-code check, run-mode check, interlocks, gate check, logging. |
| STOP service | Latency measured; stop confirmation by run state; failure escalation screen. |
| Approach check | `start_gap_deg` plus a 3-D check of the joint-linear approach from the measured pose; site threshold. |
| Dual-arm supervisor | Stop the partner on failure; completion judged from run state and `plan_num`. |
| Sim-world descriptor (with R3) | Fixture state, bin state, pole, hands, walls bound to the sha256 pass record. Needs a bin-state parameter in the collision world. |
| Upload ledger check (with NFR-DATA) | Slot → sha256, name, size, checked before a run by ID. |
| Request protection | Default 127.0.0.1; custom-header token; Host and Origin checks; refuse non-loopback without auth. |
| Bench protocol | Stop latency, pole interlock, error-code handling, SIM skip; signed records. |

### Recommendation

Make safety a designed subsystem with these parts:

- one connection owner per arm (ALIX's one-command-source rule);
- one motion gateway;
- one stop path that does not depend on the UI;
- sha256-bound gates checked against the declared bin and fixture state;
- explicit controller run-mode rules;
- a written risk assessment, validated on the bench.

Reuse the control app's stop and refusal logic and `alix_dispatch`'s run-mode, teardown, speed and pole rules. Do not inherit the unconfirmed pole/hand moves or the missing interlocks.

### Approach

1. Hold a one-day hazard workshop (engineer + the requester). Cover unintended motion; wrong arm or program; collision with the commode, the partner arm or the bin; pole-bin foul; pole moved during arm motion; pole or hand moved with the pad on a surface; high persisted speed; an ineffective stop; dual-arm desync; a second command source; SIM-mode pole/hand motion; network command injection; a corrupted upload. Assign a control and a test to each.
2. With INT-CTRL, implement the connection owner: one long-lived connection per arm in the back end. Do not rely on a second TCP client for STOP. If runner subprocesses are kept for a transition, STOP must go to the owning process.
3. Implement the per-arm session state machine and the arming flow (connection test, run-mode read, confirmation, banner, idle timeout, teardown restore).
4. Implement the motion gateway: token, reservation lock, error-code check, run-mode check (SIM skips pole and hand), foul-band check, pole-between-programs check, approach check, gate check (sha256 → pass record + world descriptor), logging.
5. Implement STOP ALL and per-arm STOP, the fixed STOP UI and stop confirmation. Measure latency on the emulator and the bench, and add the escalation screen.
6. Implement speed restore, approach preview and the threshold.
7. Implement the dual-arm supervisor rule and the honest completion states.
8. Bind to 127.0.0.1; add the custom-header token and the Host/Origin checks; refuse non-loopback without auth.
9. Write emulator tests and a bench protocol with signed records.

### Acceptance criteria

- Emulator test: with no arm armed, every motion endpoint is refused with a plain message. After arming LEFT only, RIGHT motion is still refused.
- Every motion endpoint refuses a request without a valid single-use token. A reused token fails, and tokens expire after 60 s.
- A cross-origin request (foreign Origin) or a wrong Host header to any state-changing endpoint is refused.
- STOP ALL and per-arm STOP are visible on every screen while any arm is connected. A click issues the slow stop on the owning connection within ≤ 300 ms (emulator and bench, measured and recorded), with no confirmation. The run state then confirms the stop, or the escalation screen appears within 1 s.
- Run-mode handling:
  - A run with the controller in SIM issues no pole or hand command.
  - Its record says "controller dry run", and it never satisfies the "tested on robot" step.
  - An unreadable mode, or LEFT and RIGHT in different modes, refuses the run (emulator).
- A pole command to 0.200 m with the bin STOWED is refused with the foul-band sentence, and 0.120 m is allowed (unit test). A pole command while a program is running is refused.
- A run whose approach move exceeds the site threshold is refused, with the move size shown (emulator).
- Gate and override rules:
  - A trajectory whose sha256 has no passing simulation record for the current world (fixture state, bin state, pole) is refused for REAL.
  - After any edit the old pass shows as stale.
  - An override requires a reason and is logged.
- A run by program ID is refused when the slot's listed name or size does not match the ledger entry (emulator).
- After a normal end, a STOP and a killed process, the next connection sets the global speed to the site default and logs it (emulator).
- In a dual-arm run, a forced failure on LEFT stops RIGHT within 1 s (emulator).
- An external stop is reported as "Stopped outside Lite", not "Complete" (emulator fault injection).
- A non-loopback start without auth configuration exits with an explanation.
- The run log has an entry for every motion, confirmation, stop and override, with sha256, controller IP and run mode.

### UX notes

- The REAL banner and STOP are the two things an operator must see at a glance; nothing else gets the same treatment.
- Confirmation text names the arm, the mode and what will move: "The LEFT arm will move to the start of SEAT TOP (left half), then clean for 15 s".
- When Lite refuses, it says why and what to do: "Simulation not passed for this version — run Check in simulation first".
- In SIM mode, say plainly: "The arm will not move. The pole and hand stay where they are."

### Safety notes

- The physical E-stop and the site risk assessment remain the primary protection. Lite's STOP is a controlled slow stop along the path.
- Collision verdicts gate real runs only once the commode, the declared bin state, the pole height and the hands are in the checked world.
- Keep confirmations rare and specific, to avoid click-through habits.
- Re-measure the foul band on the physical bin (BinShort is model-only). `task_config.py` notes only 7.2 mm clearance at 0.150 m.
- Never let two programs drive one arm: stop the ROS driver and the Tk control app before arming in Lite.

### Dependencies

- R4: connection, real/sim.
- R3/R5/R10: collision world for the gate (bin-state parameter, taught-track entry point).
- R7: pole/hand moves.
- R8: merged sequences, bridges.
- R12: rate and limit checks.
- R13: uploads, slots, ledger.
- INT-CTRL: connection owner.
- NFR-DATA: sha256 records, ledger, run log.
- NFR-UX: STOP and confirmation UI.

### Risks

- While runner subprocesses are still used, STOP depends on a live process. A crash or hang can delay or lose the stop.
- Blocking SDK calls in the owning thread can delay a stop (inferred for drag playback, map 08 §5). Keep owner calls short and non-blocking, and bench-measure.
- If the ROS driver or the Tk app also holds the arm, Lite's STOP cannot reach that session. Interlock (l) and training mitigate this.
- A gate based on an incomplete world (bin lowered, no commode) gives false confidence.
- Too many confirmations lead to click-through.
- The bin state is declared, not sensed, so the foul-band check is only as good as the declaration.

### Open questions

- Who owns and signs the risk assessment for Lite?
- What is the bin state during teaching and runs (stowed, extended, lowered, removed)? Is it sensed or declared? Has the physical bin been shortened?
- What are the allowed controller slot ranges per arm?
- What is the site threshold for the approach move (degrees)?
- On a dual-arm failure: stop the partner, or pause it?
- Should an engineer override of the simulation gate be allowed at all on production robots?

**Priority:** Must. **Effort:** XL, about 16-22 days (hazard workshop, connection owner and session model, gateway, STOP service with latency measurement, run-mode rules, interlocks, approach check, speed restore, dual-arm supervisor, request protection, emulator tests, bench validation). Borrowing `alix_dispatch` offsets the added items. Parts overlap R4 and INT-CTRL; count them once.

---

## NFR-OFFLINE-DEPLOY — Offline, self-contained deployment (new — recommended)

### Original text

(new — recommended)

### Revised requirement

Lite shall install and run on the operator/robot PC with no internet access, no ROS and no ALIX repository checkout.

- **(a) Platform.**
  - Confirm the target before the build. The candidates are:
    - Ubuntu 24.04 (or 22.04) x86_64 on an operator PC on the robot network;
    - Ubuntu 24.04 aarch64 on the robot's Jetson AGX Orin, the computer ALIX itself targets.
  - Python 3.10-3.12 in Lite's own virtual environment, with a wheelhouse built per architecture.
  - A current Chromium, Chrome, Edge or Firefox with WebGL.
  - If Lite runs on a headless Orin and is used from another PC's browser, the authenticated non-loopback mode of NFR-SAFETY (i) is required.
- **(b) No network dependencies.**
  - Every script, stylesheet, font and image is served from local files: three.js r128 and its controls, Plotly 2.35.2, and the Inter and JetBrains Mono woff2 files.
  - No page references an http(s) URL. The trajectory prototype's CDN tag is removed.
- **(c) Self-contained Python.**
  - Dependencies install from a pinned lock file and an included wheelhouse.
  - The launcher runs Python isolated: it ignores the user site-packages and `PYTHONPATH`, for example from a sourced ROS setup.
- **(d) Asset bundle.** These ship inside Lite with explicit paths:
  - the robot URDF (`alix.urdf`) and the meshes of all six referenced packages: `rm_description`, `inspire_left_hand_description`, `inspire_right_hand_description`, `pole_description`, `ugv_description`, and `realsense2_description` from the sibling `vendor/realsense-ros` tree;
  - the SRDF and the `initial_positions` files;
  - the commode scene and fixture-state meshes, and the washroom walls;
  - a freshly re-baked visual overlay.

  A manifest lists each file's sha256, source and ALIX commit, plus the licences. Files are world-readable.
- **(e) No repo coupling at run time.**
  - Lite's kinematics and collision code must not depend on repo markers.
  - Today three kept modules do: `plan_scene_validation` raises at import; `body_collision` raises on use; `package_index` and `srdf_states` scan for `.git`, `ARCHITECTURE.md` and `*_moveit_config`.
  - Use the vendored subset with explicit paths (X2(e)). Alternatively, set `ALIX_REPO_ROOT`, `ALIX_SRDF` and `BUTTERFLI_WORKSPACE_CONFIG` to a repo-shaped asset tree that also holds `tools/ament_index_shim.py` and the `package.xml` files.
- **(f) Start-up self-check.**
  - It checks: Python and dependency versions; the FCL backend; the asset manifest; the SRDF found with a non-empty disabled-collision matrix; every URDF mesh URI resolved; initial positions loaded; the commode and wall meshes present; the RealMan SDK present and matching the architecture; the data folder writable.
  - A failure shows a plain page with the fix and blocks the simulation gate.
- **(g) RealMan SDK.** The `Robotic_Arm` package and `libapi_c.so` are bundled if the licence allows; otherwise they are located once through Settings. No hard-coded home paths.
- **(h) Robot network settings.** Controller IPs and ports are editable in Settings (defaults LEFT 192.168.1.10, RIGHT 192.168.1.103, port 8080), with "Test connection", and are shown wherever an arm is named.
- **(i) Start-up.**
  - One icon or command starts the local server (127.0.0.1, default port 8766, single instance) and opens the browser.
  - The UI is usable within 10 s; the first 3-D simulation of a region is ready within 30 s.
- **(j) Install, upgrade, uninstall.** All work offline from one release artifact. Projects live outside the install folder and survive upgrades.
- **(k) Paths.**
  - Lite works from paths that contain spaces.
  - The controller project file is built in a short temporary path. It is refused if that path would exceed the controller's 300-byte limit.

### Clarifications

- This is a new requirement recommended by this analysis.
- "Operator/robot PC" means the PC on the robot network (192.168.1.x) used by operators. Its OS and CPU are to be confirmed.
- "No ALIX checkout" means Lite runs from its own pinned copies. If the host is the Orin, which already holds an ALIX release checkout, the bundle may be built from that checkout at install time, but Lite still runs from its own copy.
- The ALIX emulator (for engineers) is optional. It runs on Linux x86_64/aarch64 and Windows only.

### Findings

| # | Status | Source | Finding | Evidence |
|---|---|---|---|---|
| 1 | Verified | Studio | The front end works offline: three.js r128, Orbit/TransformControls, Plotly 2.35.2 and js-yaml are vendored; tests forbid remote assets; Plotly is served from a whitelist. | `ls WE/vendor`; `TST/test_studio_theme.py:185-195`; `SRV:6320-6338` |
| 2 | Verified (measured) | Studio CSS; this PC | Fonts are not shipped and not installed here. | `WE/editor.css:109-110, 191`; `fc-list` |
| 3 | Verified | Trajectory prototype | It loads Plotly from `cdn.plot.ly`; offline, every plot fails. | `H:7` |
| 4 | Verified | Studio; `alix_workspace` | The back end is tied to the repo: it finds the repo root by marker files (or `ALIX_REPO_ROOT`), reads task configs under `ros/orchestration`, and finds meshes by `.git`/`package.xml` scanning. | `SRV:56-87`; `WS/package_index.py:20-37` |
| 5 | New (measured) | `alix_workspace` | `plan_scene_validation` sets `REPO_ROOT = _repo_root()` at import and raises "repo root not found" outside the repo unless `ALIX_REPO_ROOT` is set. It also loads `ament_index_shim` from `REPO_ROOT/tools`. | `WS/plan_scene_validation.py:31-59, 86-100` |
| 6 | New | `body_collision` | `_repo_root()` raises outside the repo. Venues, the alix_workspace source and fixture configs are looked up under the repo. | `TG/body_collision.py:114-119, 135-136, 216-224, 427, 472` |
| 7 | New | `body_collision` / `package_index` | There are **silent fallbacks**: no SRDF gives an empty disabled-collision matrix and no group states; an initial-positions failure gives `{}`; an unresolved `package://` URI gives a warning and `None`. | `TG/body_collision.py:229-235`; `WS/package_index.py:115-121` |
| 8 | New | URDF | Mesh URIs reference six packages. `realsense2_description` lives in the sibling `vendor/` tree, found by scanning beside the repo. | grep of `WS/urdf/alix.urdf`; `WS/package_index.py:43-59` |
| 9 | New | `alix_workspace` | `ALIX_WORKSPACE_CACHE` is read only by `reachability` (X2), so Lite does not need it. `ALIX_SRDF` and `BUTTERFLI_WORKSPACE_CONFIG` are the real overrides. | `WS/reachability.py:81-84`; `WS/srdf_states.py:120-133`; `WS/config_loader.py:8, 38` |
| 10 | Verified | Studio | The server imports the generator at load; start-up takes 30-60 s. | `SRV:34-44`; `A/docs/studio/STUDIO_GENERATE_TAB_PLAN.md:2586-2590` |
| 11 | Corrected | ALIX setup | `python-fcl` ships wheels only, with no sdist. The dual-arm sweep has no non-FCL fallback. The 3.10-3.13 bound comes from ALIX's setup script; `pyproject` says only ≥ 3.10. | `A/SETUP.md:469-474`; `A/core/python/alix_workspace/pyproject.toml:9, 12-22` |
| 12 | Corrected (measured) | This PC | Under `python3 -I`, `alix_workspace` is not importable at all, because it is installed editable in the user site. Separately, the system SciPy 1.11.4 fails against NumPy 2.4.6 from `/usr/local` ("numpy.dtype size changed"). Normal mode uses `~/.local` NumPy 2.4.6 + SciPy 1.17.1. ALIX deliberately installs editable into the user site and notes a stray `sudo pip` in `/usr/local` on the dev host. | `python3 -I -B -c …`; `A/SETUP.md:442-456` |
| 13 | New | ALIX setup | ALIX's robot computer is a **Jetson AGX Orin** (aarch64, Ubuntu 24.04, JetPack 7.2+, "arms reachable: yes"). `python-fcl`, numpy and scipy publish aarch64 wheels, and the RealMan SDK ships `linux_arm`. | `A/SETUP.md:3-5, 74-85, 460-468`; `A/DEPENDENCIES.md:35-50` |
| 14 | Verified | `alix_workspace` packaging | It packages the URDF, config and `*.npz`, but not the meshes. `workspace_data` holds the X2 caches and a stale 2.9 MB `visual_overlay_baked.npz` with mode 600. | `pyproject.toml:44-45`; `ls -la WS/workspace_data` |
| 15 | Verified | ALIX layers | `alix_dispatch` has no `pyproject.toml`; vendor the subset. | `A/LAYERS.md:25` |
| 16 | Verified | ALIX dependencies | RealMan SDK v1.1.6; licence unknown ("a question to put to RealMan"). | `A/DEPENDENCIES.md:35-70` |
| 17 | Verified | `run_studio` | Emulator platforms: Linux x86_64/aarch64 and Windows. | `A/tools/run_studio.py:27-50` |
| 18 | Verified | Control app | The default SDK path is another machine's home. The UI's IPs are labels, while the runner takes `--ip` or `RM_*_IP`. | `V53:48-51, 7506, 7588, 7600`; `R14:46-49` |
| 19 | Corrected | Runner / `alix_dispatch` | `project_path` is 300 bytes. The runner builds the `.proj` in a `TemporaryDirectory`, so the limit concerns that temporary path, not the project folder. | `DP/emulated_program.py:84-86`; `R14:147-150` |

### Borrow verdict

- **ALIX Studio: Partial — borrow some, build the rest.**
  - Borrow: the vendored libraries, the no-remote-assets tests, page assembly with lazy Plotly (copy the pattern; its module imports X1 code), and the launcher preflight idea.
  - Do not borrow: the repo-relative resolution or the heavy imports.
  - New: packaging, the asset bundle, the isolated venv, the self-check, fonts, settings and repo independence.
- **Other sources.**
  - `alix_workspace`: borrow with adaptation (vendored subset with explicit paths, or env + repo-shaped tree; exclude X2 data).
  - `alix_dispatch` (`connect`, `track` subset, `runmode`, `session`, `peripherals`, `emulated_program`): borrow with adaptation (vendor).
  - `alix_emulator`: borrow as-is (optional).
  - Trajectory prototype: not applicable for deployment.
  - Control app: borrow with adaptation (remove the hard-coded SDK path; IPs from settings).
- **Check of the input claim.** There is no claim (new requirement). It fills a gap: the inputs expect Lite to run beside the real robot, while ALIX Studio assumes a full checkout and the trajectory prototype needs the internet.

### What to borrow

| Item | From | Adaptation |
|---|---|---|
| Vendored JS libraries | `WE/vendor/*` | Copy to `lite/web/vendor/`. |
| Page assembly, lazy Plotly | `TG/scripts/visualize_generated_config.py:530-568` (pattern only); `WE/editor.js:5320-5337`; `SRV:6297-6338` | Lite page server with its own template. |
| No-remote-assets tests | `TST/test_studio_theme.py:185-195` | Extended to CSS `url()` and `@import`. |
| Launcher preflight | `A/tools/run_studio.py:92-205` | Asset/SDK/SRDF checks, isolated Python. |
| Asset list | Map 04 §7.4; the URDF package grep | Input to the bundle builder (six mesh packages + licences). |

### What to build

| Item | Approach |
|---|---|
| Asset-bundle builder | Runs on a machine with the ALIX repo. It copies the files, rewrites `package://` URIs or writes an explicit package map, re-bakes the overlay, writes a manifest with sha256 + commit + licences, and sets mode 0644. |
| Repo independence | A vendored `alix_workspace` subset and helpers with explicit-path constructors (with X2/X1). Alternatively, a repo-shaped asset tree plus `ALIX_REPO_ROOT` / `ALIX_SRDF` / `BUTTERFLI_WORKSPACE_CONFIG`. |
| Lock file + wheelhouse | Per target architecture; install with `pip --no-index --find-links`. |
| Isolated launcher | `python -I`, cleared `PYTHONPATH`, single-instance lock, browser open. |
| Self-check page | Plain-language failures, including SRDF/ACM, mesh-URI and initial-position assertions. |
| Settings | Controller IPs/ports, SDK path, slot ranges, data folder (Engineer mode). |
| Webfonts | `@font-face` + licences. |
| CI checks | "No external URL" scanner; clean-VM install test with no ALIX checkout. |

### Recommendation

Ship Lite as a self-contained release: its own venv from a per-architecture wheelhouse, a vendored and pinned subset of the ALIX engine with explicit asset paths, an asset bundle with checksums and licences, locally served web libraries and fonts, and a start-up self-check that fails loudly where ALIX code would fail silently. Confirm early whether the target is an operator PC or the robot's Orin.

### Approach

1. Confirm the target PC (OS, CPU x86_64 or aarch64, Python, browser, screen, headless or not) and whether Windows is needed.
2. Create the `alix_studio_lite` package (`pyproject.toml`) and a lock file, and build a wheelhouse per target architecture.
3. Write the asset-bundle builder and produce `assets/` with `MANIFEST.json`. Re-bake the visual overlay (current bin, camera).
4. Make the kept engine repo-independent: a vendored subset with explicit-path constructors, or the env + repo-shaped tree.
5. Copy the vendored JS, add the fonts, remove the prototype's CDN tag, and add the URL scanner to CI.
6. Write the launcher and the self-check, and add Settings for IPs, SDK path and data folder.
7. Package (a tarball with an install script, or a `.deb`) and document install, upgrade and uninstall.
8. Test on a clean, network-disabled VM per architecture: install, self-check, open a sample project, run a 3-D simulation, run an emulator upload/run if shipped.

### Acceptance criteria

- On a fresh Ubuntu VM (each confirmed architecture) with networking disabled, no ROS and no ALIX checkout, Lite installs from the release artifact and passes its self-check.
- Every Lite module imports with no ALIX checkout and no `ALIX_*` environment variables (vendored option); no "repo root not found" appears.
- The browser network log during a full operator journey (emulator) shows requests only to 127.0.0.1.
- The CI scan finds no http(s) URL in served HTML/CSS/JS (licence comments excepted).
- The UI is usable ≤ 10 s after launch; the first 3-D simulation of a sample region is ready ≤ 30 s on the target PC.
- Silent fallbacks become loud:
  - Deleting one mesh from the bundle makes the self-check fail and name the file.
  - Deleting the SRDF makes the self-check fail; it does not silently run with an empty matrix.
  - In both cases the 3-D check refuses to report "no collision".
- Lite runs correctly with a sourced ROS environment (`PYTHONPATH` set) and with conflicting user site-packages present.
- Controller IPs changed in Settings are used by every robot command and shown in the UI.
- Fonts render as Inter / JetBrains Mono on a PC without them installed.

### UX notes

- Operators start Lite from one icon. If something is missing, they see one plain page saying what is wrong and whom to call.
- Settings operators never touch (IPs, SDK path) live in Engineer mode.

### Safety notes

- Silent asset loss weakens the collision checks. The self-check must block the simulation gate when meshes, the SRDF or the commode scene are missing.
- Wrong IP = wrong robot. Show the IP on every motion confirmation and on the arm cards.
- A non-loopback (remote browser) deployment needs NFR-SAFETY (i) authentication before any motion endpoint is reachable.

### Dependencies

- Architecture decision: standalone package with vendored subset.
- X1/X2: what is not shipped, the vendored subset.
- NFR-THEME: fonts.
- R3/R5/R10: simulation assets.
- R4/INT-CTRL: SDK, IP settings.
- NFR-SAFETY (i): remote access.
- RealMan SDK licence decision.

### Risks

- SDK redistribution may not be allowed (there is no licence file).
- `python-fcl` wheels may lag new Python versions.
- Mistakes in mesh-URI rewriting give an empty or wrong robot model. The self-check catches this.
- Robot PCs that source ROS can shadow packages unless the launcher isolates Python.
- Files with mode 600 in the ALIX repo (`visual_overlay_baked.npz`, `design/_tokens.css`) fail for other users unless the builder normalises permissions.
- The vendored engine drifts from ALIX. Pin the commit and keep a re-sync procedure.

### Open questions

- Target PC: OS version, CPU (x86_64 or Jetson aarch64), screen, browser? Headless?
- Is Windows support required?
- May the RealMan SDK be redistributed with Lite?
- Should the ALIX emulator ship with Lite for engineers and tests?
- Where should project data live (local disk, network share, USB backup)?

**Priority:** Must. **Effort:** L, about 11-15 days (package and lock files per architecture, asset-bundle builder and re-bake, repo independence, launcher and self-check, fonts and URL scan, packaging, clean-VM tests).

---

## NFR-DATA — Project data, provenance and integrity (new — recommended)

### Original text

(new — recommended). The inputs require a taught raw `.txt` per region (R1), edits with undo (R2), merging (R8) and uploads (R13), but they define no data rules.

### Revised requirement

Lite shall keep all work in one project folder, never change raw recordings, record where every file came from and how it was made, keep every output version, and verify integrity before any robot use.

- **(a) Project folder** with a fixed layout per site/fixture:
  - `project.yaml`: site, fixture, controller IPs, slot ranges, Lite version.
  - `regions.yaml`: the region registry from the deck — ID, deck label/task/slide, arm, type, fixture state, cleaning time, pole and hand presets, paired region.
  - Folders: `raw/`, `optimised/`, `sim/`, `tests/`, `sequences/`, `packages/`, `legacy/`, `logs/`, `backups/`, `trash/`.
- **(b) Raw recordings are immutable.**
  - The drag-teach save has the controller SDK write the recording to a temporary name inside `raw/<ID>/` (`rm_save_trajectory` writes the file itself).
  - Lite then validates it (7 integers per line, at least 2 lines), computes the sha256, renames it to `<ID>__RAW__<yyyymmdd-hhmmss>.txt` and marks it read-only.
  - Lite never edits, renames or overwrites a raw file. Teaching again creates a new file.
  - Deletion is an Engineer action that moves the file to `trash/`.
- **(c) Provenance sidecar.**
  - Each raw file has a `<stem>.SOURCE.json` compatible with ALIX's taught-track schema: `side`, `frame`, `pole_m`, `hand_state`, `hand_counts`, `hardware`, `taught_by`, `date`, `units: "millideg"`, `note`.
  - Lite adds: region ID, deck task, fixture state, bin state, target and actual cleaning time, controller IP, SDK and Lite versions, point count, sha256.
- **(d) Versioned outputs.**
  - Every optimised or offset output is a new version `<ID>__OPT_v<k>.txt`; `k` increases and is never reused.
  - Each has a `<stem>.RECIPE.json`: source sha256, ordered recipe steps with parameters, Lite version, output sha256, and metrics (duration, peak joint speed vs rating, max step, spike count, gate results).
  - Re-applying the recipe to the raw file reproduces the output byte for byte.
- **(e) Checksums and ledger.**
  - sha256 is recorded for every raw, derived and package file.
  - Simulation records, robot-test records and uploads refer to the sha256 of the exact bytes used. The sha256 is verified before simulation sign-off, before any real run and before upload, and Lite refuses on mismatch.
  - An **upload ledger** records slot → (track sha256, project-file sha256, name, size, time, controller IP). The controller reports only name and size.
- **(f) Packages.** Every merged or uploaded set is written to `packages/<name>_v<k>/` with `MANIFEST.json` and a `SHA256SUMS` covering every file, including the sequence JSON.
- **(g) Safe writes.**
  - Writes are atomic (temporary file in the same folder, fsync, rename).
  - An edit checks that the file has not changed since it was loaded.
  - Registry and sequence files keep their last 20 backups.
- **(h) Formats.**
  - Trajectories are NDJSON `{"point":[7 integers]}` in millidegrees. Lite reads LF or CRLF and keeps raw bytes unchanged.
  - Lite writes its own `.txt` outputs with CRLF and **integer values only**. The controller project file is always CRLF.
  - The sha256 is always of the bytes on disk.
  - Sequences use a versioned, schema-validated format (`realman_workflow_sequence` v2), with automatic migration from v1.
- **(i) Names.**
  - Region IDs follow the normalised deck scheme `<ARM>-<KIND><NN>[-P<n>|-S<n>]`.
  - File names use only `A-Z a-z 0-9 _ -`.
  - Controller program names are generated so that `<id>_<name>` fits the controller's 32-byte name field and stays unique.
- **(j) Browser storage** holds only UI preferences, under `alixLite.*`. No work exists only in the browser.
- **(k) Logs.** An append-only JSONL action log in `logs/`: time, role, action, region, file sha256, controller IP, run mode, result.
- **(l) Import / export.**
  - A region or project can be exported as a zip with manifest and checksums.
  - The V6 package can be imported read-only into `legacy/`, with its checksum status recorded.
  - Files dropped into folders by hand are adopted only through an import step that creates their sidecars.

### Clarifications

- This is a new requirement recommended by this analysis.
- Region = deck unit; Sub-region = a time window inside a region's trajectory. Recipe steps can target sub-regions.
- Undo (R2) works on the recipe in memory. Saving creates a version, and undo never deletes files.
- "Immutable" is enforced by Lite. An OS user could still change files; the sha256 checks detect that.

### Findings

| # | Status | Source | Finding | Evidence |
|---|---|---|---|---|
| 1 | Verified | Control app | Today's drag-teach save goes to `~/Downloads/RealMan_DragTeach/<side>_drag_<YYYYmmdd_HHMMSS>.txt`, with no region, pole, hand or checksum. | `V53:3840-3850` |
| 2 | New | Runner v14 | The SDK writes the recording itself (`rm_save_trajectory(path)`), so Lite cannot make that write atomic. It must save to a temporary name, validate, then rename. | `R14:957-990` |
| 3 | Verified | Trajectory prototype | Outputs are browser downloads named `<base>_<stages>_processed.txt`, plus a processing report (a good recipe seed: no source checksum, version or region). Outputs are written with LF. Invalid lines are skipped silently. | `H:803-829, 1617-1619, 1732-1754, 1914-1917` |
| 4 | Verified | ALIX taught lane | The source shelf rule: "Drop taught .txt files here unmodified … nothing here is ever edited". There is a `SOURCE.json` schema, and `hardware: analysis_only` is refused. | `A/Resource/tracks/taught/source/README.md:1-27` |
| 5 | Verified | ALIX taught lane | `source_provenance` fields; the record is written at certify and never overwrites a different recording; `taught_pipeline` records the original path + sha256. | `A/tools/taught_tracks.py:310-342, 3620-3636`; `A/tools/taught_pipeline.py:36-43` |
| 6 | Verified | ALIX taught lane | Certificates are bound to the bytes by sha256, and upload and run re-check them. | `A/tools/taught_tracks.py:29-32, 291-292` |
| 7 | Corrected | Studio | The **atomic** write exists only as the helper `schedule_guards.write_schedule_csv`. The config save handler uses plain `write_text` (not atomic), with an mtime conflict guard. Schedule saves are backed up into the repo's `.cache`. Copy the helper, not the handler. | `TG/schedule_guards.py:59-104`; `SRV:3350-3360, 5141-5156` |
| 8 | Verified (measured) | V6 package | Its own `SHA256SUMS.txt` fails: 28 files OK; `README.txt` FAILED; 5 listed JSONs missing. The four JSONs actually used to run are not covered. | `sha256sum -c V6/SHA256SUMS.txt` |
| 9 | Verified | Sources | Line endings differ: V6 files CRLF; `UC/zip` and `plan_txt_files` LF; the prototype writes LF. The runner reads either and writes the project with CRLF; the controller rejects LF-only. | Map 10 §1.1; `R14:20, 74-96`; `DP/track.py:19-20` |
| 10 | New | Runner v14 | The runner refuses non-integer values and files with fewer than 2 points. | `R14:74-86` |
| 11 | New | Runner v14 | Upload verification is by size only (± max(64 B, 0.1%)), so only a local ledger can tie a slot to a sha256. | `R14:147-175` |
| 12 | Verified | Control app | Workflow JSON v1 is checked only for its format name and a step list. Old `speed` values are not carried forward (transit defaults to 30). | `V53:6819-6852` |
| 13 | Verified | `alix_dispatch`; runner | The controller name field is 32 bytes (`<id>_<basename>` cut to 32, observed on hardware), while the runner allows 56-character names. `project_path` is 300 bytes. | `DP/emulated_program.py:25, 81-86`; `R14:1008-1014` |
| 14 | New | `alix_dispatch.runmode` | No record in ALIX's 157-run ledger carried a run-mode field until M47. An arm left in SIM produces "phantom" runs that score complete. | `DP/runmode.py:205-215` |
| 15 | Verified | Map 13 | Deck region IDs are inconsistent; a normalised scheme and an alias table exist. | Map 13 §5.1-5.2 |
| 16 | Verified | Slot conventions | Five conventions conflict (25 / 99 / 40-59 / 70-100 / 71-83). | Map 08 §4.5; `DP/track.py:13-16` |
| 17 | Verified | Control app | Temporary stage files go to `/tmp/realman_dual_sequence_ui/`. | Map 08 §4.4 |
| 18 | Verified | UX research | Browser storage is eviction-prone; keep only UI preferences and crash buffers there. | `RES:1003-1009` |

### Borrow verdict

- **ALIX Studio: Partial — borrow some, build the rest.**
  - From the Studio: the atomic-write *helper* (not the save handler), the mtime conflict guard, backups, the Board's "show only what is on disk" idea, and the browser-storage rule.
  - Its data model (task-config YAML per region × arm inside the repo) does not apply.
  - The project layout, sidecars, versioning, ledger and checksum gates are new or come from the ALIX taught-track lane.
- **Other sources.**
  - ALIX taught lane: borrow with adaptation (immutable shelf, `SOURCE.json`, sha256 certificates).
  - Prototype processing report: borrow with adaptation (becomes `RECIPE.json`).
  - Control-app workflow JSON v1: borrow with adaptation (import format; v2 with schema and migration).
  - Runner `read_track` / `write_project`: borrow as-is (format rules).
  - `alix_dispatch.runmode`: borrow the run-mode record field.
  - V6 package: not applicable as a data model (import read-only as legacy).
  - Map 13 ID scheme: borrow as-is.
- **Check of the input claim.** There is no claim (new requirement). R1's "dashboard to show the regions with and without the taught raw txt" implies raw files can be found per region. Today they cannot (timestamp-only names in `~/Downloads`), so this requirement is a precondition for R1.

### What to borrow

| Item | From | Adaptation |
|---|---|---|
| Atomic write helper, conflict guard, backups | `TG/schedule_guards.py:59-104`; `SRV:3350-3357, 5141-5156` | Storage module for all Lite writes; backups inside the project. |
| `SOURCE.json` schema | `A/tools/taught_tracks.py:310-342`; `A/Resource/tracks/taught/source/README.md:8-27` | Add the Lite fields. |
| sha256 certificate binding | `A/tools/taught_tracks.py:29-32, 291-292` | Lite pass and test records, upload checks. |
| Processing report | `H:1732-1754` | Becomes `RECIPE.json`. |
| ID scheme and region record | Map 13 §5.2, §7.3 | `regions.yaml` schema. |
| Track reader / project writer | `R14:74-96` | Format rules: integers only, ≥ 2 points, CRLF for controller files. |
| Run-mode record field | `DP/runmode.py:205-223` | Recorded with every run and test record. |

### What to build

| Item | Approach |
|---|---|
| Storage module | Layout, atomic writes, sha256, read-only raw, version counters, conflict guard, backups, append-only log. |
| JSON Schemas | `project.yaml`, `regions.yaml`, `SOURCE.json`, `RECIPE.json`, sim/test records, sequence v2; validated on load with row-level messages. |
| Drag-save integration (with R1) | Temporary name → validate → sha256 → rename → read-only → sidecar. |
| Recipe replay test | Determinism check. |
| Package writer + verifier | `MANIFEST.json` + `SHA256SUMS` covering every file. |
| Upload ledger (with R13/NFR-SAFETY) | Slot → sha256s, name, size; checked against the controller list before a run by ID. |
| Program-name generator | ≤ 32 bytes including `<id>_`; collision check against the controller list. |
| Import / export | V6 legacy (read-only, checksum status), loose files with sidecar creation, zip export. |

### Recommendation

Define the project folder, sidecars and ledger before building R1 and R2. Reuse ALIX's immutable-source and sha256-certificate rules. Make every later step (simulation pass, robot test, upload, run by ID) refer to the exact bytes by sha256, and to the run mode in which it happened.

### Approach

1. Write the project layout and JSON Schemas. Review with the requester (registry fields) and the R1/R2/R8/R13 owners.
2. Implement the storage module and its tests: atomicity under kill -9, read-only raw files, version numbering, conflict guard, backups.
3. With R1: drag-save to a temporary name in `raw/<ID>/`, validate, rename, write `SOURCE.json`.
4. With R2: on Save, write `OPT_v<k>` + `RECIPE.json` and add the determinism test.
5. With R3/R4/R13: key pass and test records (with run mode) and uploads by sha256, and keep the upload ledger.
6. Implement package writing (R8/R13) with `MANIFEST.json` + `SHA256SUMS`, and a verifier.
7. Implement the program-name generator and the slot-range check with R13.
8. Implement V6 legacy import and project export/import.
9. Document the folder layout in the engineer help.

### Acceptance criteria

- A teach save creates `raw/<ID>/<ID>__RAW__<ts>.txt` and its `SOURCE.json` with every required field. The raw file is read-only, and any API attempt to overwrite it fails.
- A failed or partial SDK save leaves no file under a final raw name.
- Two optimisation saves produce `_v1` and `_v2`. The `RECIPE.json` of v2 names the raw sha256 and the parameters, and replaying it reproduces v2 byte for byte.
- Every optimised file contains only integer values and CRLF line endings, and passes the runner's track check.
- Changing one byte of a file makes simulation sign-off, real runs and uploads that use it refuse, with a plain message naming the file.
- A run by program ID is refused when the controller's listed name or size for that slot does not match the ledger.
- Killing Lite during a save leaves either the previous or the new complete file (fault-injection test, 100 runs).
- Every generated package passes `sha256sum -c SHA256SUMS`, and the sums cover every file including the sequence JSON.
- The V6 FINAL JSONs import and migrate to v2 with a migration note. A JSON with a missing key is rejected with its row number.
- Every generated controller program name satisfies `len("<id>_<name>") ≤ 32` bytes and is unique in the target controller's list.
- Clearing browser storage loses no work.
- The action log has one line per action, with sha256 wherever a file is involved and the run mode for every run.

### UX notes

- Operators never see file paths. The dashboard shows "Taught 12 min ago · 52 s · version 3".
- "Teach again" keeps the old recording (shown as an earlier version), so nothing taught is lost.
- Engineers get "Open folder" and "Export region" buttons.

### Safety notes

- sha256 binding is what makes "simulated and tested" mean "these exact bytes". Without it, an edited file could reach the robot on an old pass.
- The ledger check stops an old or foreign program in a slot from running under a new pass.
- The pole height in `SOURCE.json` matters: a joint-space track replayed at a different pole height runs the whole path at the wrong height (`A/experiments/exp01_teach_trajectory_reverse_engineering/data/teach_files/MANIFEST.yaml:136-138`).
- A SIM-mode run recorded without its mode would look like a real test. Record the mode.

### Dependencies

- R1: drag-save naming and sidecar.
- R2: recipes, undo, versions.
- R3/R4: pass and test records.
- R8/R13: packages, upload ledger, slot ranges.
- NFR-SAFETY: gates use sha256, the ledger and run mode.
- NFR-OFFLINE-DEPLOY: data folder location.
- Sign-off of the map 13 ID scheme.

### Risks

- Hashes must be of the exact bytes on disk. Hashing normalised text would break the binding.
- Versions and logs grow on disk. Show sizes and offer an engineer clean-up tool; never delete automatically.
- Copying files into folders by hand bypasses the sidecars. The import step must be the only way in.
- A wrong clock on the robot PC makes timestamp names misleading. Record timezone-aware times in the sidecars.

### Open questions

- Where do project folders live, and how are they backed up?
- Should Lite's raw shelf also feed ALIX's taught-track lane (export in its `SOURCE.json` format) for certification?
- Who maintains `regions.yaml` (cleaning times, pole/hand presets): the requester or the site engineer?
- How long must raw recordings and run logs be kept?

**Priority:** Must. **Effort:** L, about 10-14 days (schemas, storage module, sidecars, versioning, sha256 gates, ledger, package writer/verifier, program-name generator, V6 import and export, tests). Integration inside R1/R2/R13 is counted there.

---

## Cross-cutting notes

1. **Architecture (corrected).**
   - These requirements assume a standalone `alix_studio_lite` package (map 04's recommendation).
   - "Imports `alix_workspace`" does not work unchanged outside the repo: `plan_scene_validation` raises at import (measured), and `body_collision`, `package_index` and `srdf_states` resolve resources through repo markers.
   - Recommended: a vendored, pinned subset with explicit asset paths. Fallback: a repo-shaped asset tree with `ALIX_REPO_ROOT` / `ALIX_SRDF` / `BUTTERFLI_WORKSPACE_CONFIG`.
2. **Studio routes are reused as behaviour, not code.**
   - The robot routes sit on `robot_model._ik_context`, which imports `reachability._placement` (X2); `config_absolute_poses` imports `alix_taskgen.coverage` (X1).
   - `/api/manifest` is built in a script that imports `yaml_assembler` and `regions` (X1).
   - Lite re-implements these on the allowed helpers; the import guard enforces it.
3. **Terminology.** "Region" means a painted mesh area (Studio), a time window (prototype "scaling regions") and a cleaning area (deck). Use the glossary: Region = deck unit, Sub-region = time window.
4. **One token source** for every renderer: `alix_tokens.css` for the web UI and generated constants for any Python/Tk output. Colour literals in lifted JS (timeline, DAC plot) must be replaced too.
5. **sha256 records and the upload ledger** connect NFR-DATA, NFR-SAFETY and R3/R4/R13. A pass applies only to the exact bytes it checked, and a slot is trusted only if its listing matches the ledger, because the controller verifies by size only.
6. **Operator vs Engineer roles** apply to every screen. Numeric parameters, slot IDs, IPs, raw logs and overrides live in Engineer mode.
7. **One command source per arm** (ALIX ruling, `DTC:220-233`). Lite's back end owns one connection per arm while it is armed. The ROS driver and the Tk control app must not drive an armed arm.
8. **The controller's SIM mode is a dry run on hardware.**
   - The arm does not move, but pole and hand commands would move the real devices, so Lite skips them in SIM runs.
   - A SIM run never counts as a robot test (`DP/session.py:1-8`; `DP/runmode.py:205-235`).
9. **Bin state.**
   - Both ALIX collision checkers hard-code BIN_LOWERED.
   - The foul band was measured on the BinShort model, and the physical bin has not been remade.
   - Both V6 workflows start at pole 200 mm, inside the stowed-bin band.
   - The bin state must be declared, checked and recorded before R7/R10 defaults are set.
10. **Missed spots (R3) versus X1.** ALIX's coverage tools are X1 (painted regions). Lite shows missed spots with a read-only FK pad trail over the commode, or builds a new contact-coverage estimate (open question).
11. **Platform.** ALIX's robot computer is a Jetson AGX Orin (aarch64). If Lite runs there, it needs aarch64 wheels and, if headless, the authenticated remote-browser mode.
12. **File modes.** `WS/workspace_data/visual_overlay_baked.npz` and `A/design/_tokens.css` are mode 600; the asset builder normalises permissions.
13. **Python environment on this PC (measured).** It mixes three site-package locations. Under `python3 -I`, `alix_workspace` is not importable and SciPy/NumPy are ABI-incompatible. This is direct evidence for the isolated venv.
14. **Control app, high level as requested (not a bug audit).**
    - Current pair: UI v53 (`rm_trajectory_ui_v53_safe_lid_checkpoint.py`) + runner v14 (`rm_upload_fast_v14_safe_transit.py`).
    - The three v14 runner files are byte-identical (md5 `a0b285e6…`). v53 picks v14 first and silently falls back to older runners if it is missing.
    - What it does: a Tkinter console that runs the runner as one subprocess per action — upload and verify, run by program ID, sequences, drag-teach record/save/playback, pole and hand moves, and the controller's SIM/REAL mode. Helpers: `pole.py`, `hand.py`, `usage.py`.
    - Plan-relevant items only, one line each:
      - Multi Run and Page-01 Load Sequence raise `TypeError` (`V53:4853, 4876, 4885, 5000` lack `@staticmethod`), so do not port those paths as-is.
      - The IPs are display labels with no entry field, so Lite needs IP settings.
      - Manual hand/pole apply has no confirmation and no bin check, so Lite's gateway must add both.
15. **Theme default conflict.** The Ops notes ask for a dark console theme (`OPS:227-237`). ALIX's dark variant meets the intent; the default needs the requester's decision.
16. **Controller naming and slots.** The 32-byte name field and five conflicting slot conventions (25 / 99 / 40-59 / 70-100 / 71-83) need one site policy before R13 is built.
17. **Legacy package integrity (measured).** `V6/SHA256SUMS.txt` fails `sha256sum -c` and does not cover the four JSONs that are actually run. Treat V6 as legacy reference only.
18. **Schedule & Run decision** (X1(f)): the tab and its back end are out; the timeline, transport and guard-sentence components are in, fed by Lite's workflow model and restyled to tokens.

---

## Change log (material changes to the analyst draft)

| # | Req | Change | Reason | Evidence |
|---|---|---|---|---|
| 1 | X1 | Added that the "reuse candidate" routes must be re-implemented, not imported | Their handlers pull in X1/X2 code | `TG/robot_model.py:170-181, 230-244`; `TG/scripts/visualize_generated_config.py:13-20, 219`; `SRV:43` |
| 2 | X1 | Corrected the "neutral modules" claim: separable by import, but repo-coupled; `paths.py`, `constants.py` and `glove_frames.py` added to the allow-list | `body_collision._repo_root()` raises outside the repo; `scene_pose` needs `paths` and `constants`; R9 needs tool frames | `TG/body_collision.py:114-119, 135-136, 216-224`; `TG/scene_pose.py:3-24`; `TG/glove_frames.py:1-25` |
| 3 | X1 | Added a read-only pad trail as the way to show missed spots, plus an open question | ALIX coverage is X1, and `alix_workspace.coverage` imports it | `WS/coverage.py:1-8, 51-62`; `IN:12` |
| 4 | X1 | Write boundary extended to the Studio's repo cache folders; deny-list extended to `robot_model` and `scripts.*` | Copied code would write into the repo; guard completeness | `SRV:4154, 5153`; `SRV:5800-5804`; `TG/auto_generate.py:279` |
| 5 | X1 | Added the taught-track entry point and bin-state note to the safety notes | BodyCollisionWorld takes task configs and fixes BIN_LOWERED | `TG/body_collision.py:499-530, 627-631` |
| 6 | X2 | Added `config_loader` to the kept list; reclassified `clearance` and `wipe_kinematics` as optional helpers and `coverage` as X1-coupled; restructured the exclusion list | Real imports and usage | `WS/plan_analysis.py:21-22, 52`; `SRV` grep; `WS/clearance.py:1-8`; `WS/wipe_kinematics.py:1-10, 43`; `WS/coverage.py:51-62` |
| 7 | X2 | Recommendation changed from "upstream lazy init" to "vendored pinned subset with explicit paths" (lazy init now optional) | `plan_scene_validation` raises at import outside the repo (measured) | `WS/plan_scene_validation.py:31-59, 86-100` |
| 8 | X2 | Added inlining of `_placement`; `/api/ik` not exposed as a route; kept-module verdict changed from "as-is" to "with adaptation"; acceptance gains a strict guard and a no-checkout import test | The IK context depends on X2; repo coupling | `TG/robot_model.py:179`; `WS/reachability.py:47-52` |
| 9 | NFR-UX | Removed the control-app literal-"\n" dialog detail | The user asked for no bug audit of the control app | User guidance in the task |
| 10 | NFR-UX | Added ALIX's Playwright layout audit as a borrow item and in the acceptance criteria | It already checks targets, type floor and clipping | `A/tools/layout_audit.py:1-40, 61, 177` |
| 11 | NFR-UX | Usability task 5 now specifies the emulator or the bench controller; undo is reused as a pattern only; text floor ≥ 14 px includes charts | Controller SIM requires hardware; the snapshot is Studio-specific; charts use 10 px | `DP/runmode.py:1-8`; `WE/editor.js:1896-1902, 5368` |
| 12 | NFR-THEME | Rescoped the CSS defect list (most are in X1/X2 rules Lite drops); added JS colour literals that ALIX's guard misses; added chart 10 px text; added `--accent` / `--accent-soft`; removed the unneeded atlas exclusion from the parity test; added contrast pairs (dark hover 2.52:1, muted on inset 4.37:1) | Code shows the defect locations, the guard scope and the token usage | `WE/editor.css:386-398, 1016-1029, 1986, 2096-2166, 39-40`; `WE/editor.js:13593, 14485-14488, 5368`; `TST/test_studio_theme.py:86-89`; `A/design/_tokens.css:8-55` |
| 13 | NFR-THEME | Added an arm-identity colour rule (distinct from brand blue and warning amber, always with words) | Deck colours collide with reserved meanings | Map 13; `A/design/_tokens.css:9-10, 20` |
| 14 | NFR-THEME | Effort 6-8 → 5-7 days | Fewer defects survive the X1/X2 filter | As #12 |
| 15 | NFR-SAFETY | STOP and connection model now follow ALIX's one-command-source rule (no second TCP client); added stop confirmation, on-path slow stop and no program-scoped stop; added interlock (l) | ALIX ruling and SDK facts | `DTC:24-28, 220-233`; `R14:26` |
| 16 | NFR-SAFETY | Added controller run-mode rules: SIM skips pole and hand, SIM ≠ test pass, unknown or mixed modes refused, teardown restore; added `alix_dispatch` borrow items | Pole and hand are not simulated in SIM; phantom runs; teardown pattern | `DP/session.py:1-8, 217-265`; `DP/runmode.py:178-260` |
| 17 | NFR-SAFETY | Added the approach-move size check and threshold, the wrong-arm check and the speed guard from `alix_dispatch.track` | The controller's implicit movej is not collision-checked | `DP/track.py:161-200, 323-350, 2274-2285` |
| 18 | NFR-SAFETY | Added the pole-between-programs rule and arrival margin | A lift command aborts a planned arm move and latches errors; lift state leads by up to 73 mm | `DP/peripherals.py:18-24` |
| 19 | NFR-SAFETY | Matching-world gate: both checkers hard-code BIN_LOWERED (BodyCollisionWorld too); foul band marked provisional (BinShort vs physical bin) | Code and description notes | `TG/body_collision.py:627-631`; `ORCH/core/task_config.py:63-67`; `A/ros/description/ugv_description/urdf/bin.xacro:32-34` |
| 20 | NFR-SAFETY | Corrected the commode-C teach pole claim: 140 mm and 200 mm, not only 200 mm | The table shows both | `…/commode_c_taught_paths/description.txt:3-15` |
| 21 | NFR-SAFETY | Added cross-origin and Host protection for state-changing requests | The Studio has no Origin/Host/token checks; localhost binding alone does not stop browser cross-site posts | `SRV:6510-6544` (grep) |
| 22 | NFR-SAFETY / NFR-DATA | Added the upload ledger and the slot check before a run by ID | The controller verification is size-only | `R14:147-175` |
| 23 | NFR-SAFETY | Foul-band function is copied, not imported, from the ROS tree | Standalone package; ALIX itself keeps copies | `WE/editor.js:17144-17150`; `ORCH/core/task_config.py:58-124` |
| 24 | NFR-OFFLINE-DEPLOY | Platform: Jetson AGX Orin (aarch64) added as a candidate; headless use needs the authenticated remote mode | ALIX's robot computer and wheel availability | `A/SETUP.md:3-5, 74-85, 460-468`; `A/DEPENDENCIES.md:35-50` |
| 25 | NFR-OFFLINE-DEPLOY | Repo-independence sub-requirement (e); corrected environment variables (`ALIX_REPO_ROOT`, `ALIX_SRDF`, `BUTTERFLI_WORKSPACE_CONFIG` essential; `ALIX_WORKSPACE_CACHE` only for reachability) | Import-time RuntimeError measured; env usage | `WS/plan_scene_validation.py:31-59`; `WS/reachability.py:81-84`; `WS/srdf_states.py:120-133`; `WS/config_loader.py:8, 38` |
| 26 | NFR-OFFLINE-DEPLOY | Self-check now asserts the SRDF/ACM, mesh-URI resolution and initial positions | ALIX code falls back silently | `TG/body_collision.py:229-235`; `WS/package_index.py:115-121` |
| 27 | NFR-OFFLINE-DEPLOY | Asset bundle lists the six mesh packages, including the vendor `realsense2_description`, with licences | URDF package URIs | grep of `WS/urdf/alix.urdf`; `WS/package_index.py:43-59` |
| 28 | NFR-OFFLINE-DEPLOY | Evidence corrections: Python bound source; isolated-mode details; the 300-byte limit applies to the temporary `.proj` path; effort 10-13 → 11-15 days | Setup doc, pyproject, runner code | `A/SETUP.md:442-456, 469-474`; `pyproject.toml:9`; `R14:147-150` |
| 29 | NFR-DATA | Drag-save via temporary name → validate → rename; integers-only CRLF outputs; `trash/` added; run mode in records | The SDK writes the file; runner format rules; layout consistency; phantom SIM runs | `R14:74-96, 957-990`; `DP/runmode.py:205-215` |
| 30 | NFR-DATA | Corrected the Studio safe-write claim: only the helper is atomic; the save handler is not | Code | `SRV:3359`; `TG/schedule_guards.py:59-104` |
| 31 | Cross-cutting | Control-app summary kept high level; added that the three r14 files are byte-identical and that v53 falls back to older runners | User asked for a high-level understanding and the current version | md5 of the `UC/rm_upload_fast_v14*.py` files; `V53:6890-6904` |
