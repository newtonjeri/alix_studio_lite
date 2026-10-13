# 02 — ALIX Studio: 3D viewport, playback and collision detection (evidence map for ALIX Studio Lite)

Scope: how ALIX Studio loads and draws the robot and scene, every motion player, every collision engine and how
collisions are shown, the Simulate (live) view, the bin, and the existing layout. The goal is to decide what Lite
can borrow for R3, R5, R10 and R11, with side notes on R4, R7, R8, R9, R12 and R14. Everything was read-only.
Line numbers are as of the files on disk on 2026-10-13. "(inferred)" marks a claim not directly seen.

## Path legend (absolute)

| Tag | Path |
|---|---|
| `SRV` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/server.py (6566 lines) |
| `JS` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/web_editor/editor.js (19325 lines) |
| `TPL` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/web_editor/template.html (953 lines) |
| `CSS` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/web_editor/editor.css (2166 lines) |
| `RM` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/robot_model.py |
| `SB` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/sim_bridge.py |
| `BC` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/body_collision.py |
| `DAPC` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_workspace/src/alix_workspace/dual_arm_plan_collision.py (1317 lines) |
| `COL` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_workspace/src/alix_workspace/collision.py |
| `UK` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_workspace/src/alix_workspace/urdf_kinematics.py |
| `PSV` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_workspace/src/alix_workspace/plan_scene_validation.py |
| `PA` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_workspace/src/alix_workspace/plan_analysis.py |
| `URDF` | /home/newtonjeri/alix_ws/src/alix/core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf |
| `SRDF` | /home/newtonjeri/alix_ws/src/alix/ros/manipulation/alix_moveit_config/config/alix.srdf |
| `EXP01` | /home/newtonjeri/alix_ws/src/alix/experiments/exp01_teach_trajectory_reverse_engineering/src |
| `TRAJ` | /home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html |
| `RUN14` | /home/newtonjeri/alix_studio_lite_ws/upload_channel/rm_upload_fast_v14_safe_transit.py |
| `PKG70` | /home/newtonjeri/alix_studio_lite_ws/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES |

## 0. Headline verdicts

1. **There is no client-side forward kinematics (FK).** Every pose the Studio draws goes through one function,
   `applyRobotPose(poseMap)` (JS:12427-12437). It takes per-link 4x4 world transforms that the server computes
   with `UrdfModel.link_world_transforms` (UK). Every player and the live view are "producers" for that function
   (JS:19186-19193).
2. **The Dual-Arm Collisions (DAC) tab is the closest thing to R3, R5 and R10.** It loads two joint-space plan
   JSONs, runs an FCL sweep on the server (`compute_dual_arm_plan_collisions`, DAPC:586-764), plays the motion on
   the robot, turns colliding links red, and draws a Plotly 0/1 collision bar chart over time (JS:14408-14598).
3. **Nothing in the Studio plots joint values.** It has only two Plotly charts: segment lengths (JS:5368) and the
   DAC collision bars (JS:14483). Playback never stops at a collision (JS:14561-14581). Nothing matches "graph
   stops at the joint value and highlights the collision point". It must be built by joining TRAJ's joint plots
   and cursor (TRAJ:1134-1141) to the DAC collision list. The library already computes the per-sample
   `joint_timeline` (DAPC:668, 755), but the server discards it (SRV:4357).
4. **A taught `.txt` can be fed to the existing machinery with a thin adapter.** The adapter converts
   millidegrees to radians, names the joints `L_joint1..7` or `R_joint1..7`, applies the player clock, and wraps
   the result as a one-stage plan JSON. This exact wrapper already exists as `teach_as_plan.json` in
   EXP01/predict_track_replay.py:182-188. The adapter **must inject the pole height and the hand state**:
   otherwise the pole defaults to 0.300 m and the hands to fully open (DAPC:385-452).
5. **The Studio collision world is incomplete for Lite.**
   - The DAC sweep checks arm vs arm, self-collision, and the bin plus glove lever only. It does **not** check the
     commode or the walls: the `extra_managers` hook exists (DAPC:699-703) but no caller uses it.
   - The fingers are excluded from the check; only the palms are included (DAPC:171-177).
   - The building blocks for the commode and walls exist: `BodyCollisionWorld.fixture_manager` (BC:401-464),
     `load_wall_manager` (PSV:117-155) and `PlanWallChecker` (PSV:179-256, which also reports depth).
6. **There is no simulation-left / plots-right split.** DAC is a top band (controls plus a small timeline plot)
   above the viewport, with a log in the right dock. **There is no bin include/exclude toggle.** The bin is always
   LOWERED and is always part of the DAC body set.
7. **The Simulate view can show the real robot today.** POST /api/sim/joints (SRV:5326-5341) was designed for
   "future: the real arms". The upload runner already reads joint degrees with `arm_state()` (RUN14:122-139).

---

## 1. 3D viewport (Q1)

### 1.1 Which URDF, and how it is parsed
- **URDF**: `URDF`, which was generated by xacro from
  `/home/newtonjeri/alix_ws/src/alix/ros/description/alix_description/urdf/alix.urdf.xacro` (URDF:1-5 header).
  The xacro file (mtime Oct 7) is newer than the expanded URDF (Sep 6), so the two may have drifted (inferred).
  This should be verified for R10 ("correct URDF").
- **Chains**: `/home/newtonjeri/alix_ws/src/alix/core/python/alix_workspace/src/alix_workspace/config/alix_workspace_config.yaml:41-50`.
  `left` and `right` both use `../urdf/alix.urdf`, run from `base_link` to `L_glove_frame_2` / `R_glove_frame_2`,
  and share one full-robot model.
- **Default config**: `default_config_path()` in `.../alix_workspace/config_loader.py:28-53`.
- **Server context**: `_ik_context(config_path, tip)` (RM:169-228) builds:
  - `UrdfModel` per chain;
  - `RobotCollisionModel(tag="collision")` (`rcm`) and `RobotCollisionModel(tag="visual")` (`rcm_visual`);
  - `SrdfStates` from `find_srdf` (= `SRDF`);
  - `initial` (the `initial_positions.yaml` home pose).
  
  It is cached in a bounded LRU (`IK_CACHE_MAX=8`, RM:86-87).
  - Minor bug: `_config_signature` uses `yaml` without importing it (RM:117; no `import yaml` in the file). The
    NameError is swallowed, so the cache signature covers only the config file. URDF/SRDF edits are not picked
    up until the server restarts (inferred from the code).
- **Model content** (counted from URDF): 133 links and 183 joints. Non-fixed joints:
  - `L_joint1..7`, `R_joint1..7` (revolute);
  - `L_/R_sliding_plate_joint` (prismatic pole, 0..0.315 m at 0.16 m/s);
  - `bin_rail_joint` and `bin_drop_joint` (prismatic);
  - `glove_wearing_unit_arm_joint`;
  - 8 UGV wheel/suspension joints (continuous);
  - 12 Inspire-hand joints per side, some driven by `<mimic>` (URDF:1754, 1812). FK resolves the mimics
    automatically (UK:294-310, 311-349).
- **RM75 limits from URDF** (useful for R9/R12):
  - J1 ±178°, J2 ±130°, J3 ±178°, J4 ±135°, J5 ±178°, J6 ±128°, J7 ±360°;
  - velocity 180°/s for J1–J2 and 225°/s for J3–J7.
- **Meshes**: `package://` URIs into rm_description (44), inspire_left/right_hand_description (62 each),
  ugv_description (36), pole_description (8), and the d435 camera. File types: 161 STL (141 `.STL` + 20 `.stl`),
  6 `.dae`, 6 `.obj`. They are resolved from the **source tree** through `PackageIndex` (COL:55-62).

### 1.2 Mesh conversion and the wire format
- `RobotCollisionModel._parse_links` (COL:65-105) reads `<collision>` or `<visual>` meshes **and primitives**
  (box/cylinder/sphere). `_load_mesh` uses `trimesh.load(force="mesh")` (COL:110-135).
  `_load_visual_submeshes` splits DAE/OBJ files per material using pycollada (COL:137-181).
- `local_meshes(..., split_materials=True)` (COL:183-246) puts each mesh in its **link frame**, welds vertices
  (`merge_vertices`), and picks colours in the order URDF `<material>`, then file material, then client default.
- **Endpoint**: `GET /api/robot-geometry?set=visual|collision` → `do_robot_geometry` (SRV:1918-1998).
  - Returns JSON `{meshes:[{link, vertices:[[x,y,z]...] (5 dp, link frame), faces:[[i,j,k]...], color:[r,g,b]?}],
    homePose:{link: 16 floats row-major}, count, set}`.
  - Built by `_mesh_payload` (SRV:1908-1915) and cached in `_ROBOT_GEOM_CACHE` (SRV:1864).
  - The visual set is first served from the **pre-baked** `.../alix_workspace/workspace_data/visual_overlay_baked.npz`
    (2.9 MB; `_baked_visual_path` / `_load_baked_visual`, SRV:1875-1905).
  - The bake tool is `/home/newtonjeri/alix_ws/src/alix/tools/meshes/bake_visual_overlay.py:1-30`. It reduces
    1,183,924 triangles to 277,400 with at most 1.48 mm deviation.
  - Measured contents: 200 submeshes, 100 links, 220,622 vertices, 268,178 triangles. Links missing from the bake
    (e.g. `camera_link`) fall through to the live path (SRV:1959-1980).
  - `_OVERLAY_MAX_FACES = 0`: no live decimation (SRV:1866-1872).
- **Home pose** = initial positions + SRDF `bin_rail/BIN_LOWERED` + `dual_poles/full_length` (SRV:1988-1996).
- **Client**: `loadRobotGeometry(set)` (JS:12352-12395) calls `meshFromPayload(m, color, opacity, indexed=true)`
  (JS:667-703).
  - It builds an indexed `BufferGeometry` with `MeshStandardMaterial` (sRGB→linear colour).
  - The visual set gets a matte "indoor" clamp (albedo ≤ 0.62, roughness 0.9) and stores `userData.baseColor`
    so the red collision highlight can be undone.
  - The collision set is a lazily loaded orange proxy at 50% opacity (`setRobotSet`, JS:12405-12420).
  - Meshes are grouped in `robotMeshes[link]` (JS:222) and added to `robotGroup` (JS:480-481).

### 1.3 What is drawn

| Object | Source | Code |
|---|---|---|
| Both RM75 arms, both Inspire hands (all finger links), poles + sliding plates, UGV body/wheels/suspension/lower cover, bin + rail, glove-wearing unit, wrist camera | URDF visual set (100 baked links + live `camera_link`) | SRV:1918-1998, JS:12352-12395 |
| `bin_handle_left/right`, `lid_opening_cover1/2` | collision-only links; drawn **only** with the "Collision" toggle | SRV:1981-1986, TPL:728 |
| Commode fixture (body static; lid/seat in `closed`/`open` states) | `GET /api/manifest?fixture=` → `scene_manifest_cached` → `build_scene_manifest` (`.../alix_taskgen/scripts/visualize_generated_config.py:219+`); mesh keys `<part>_<state>` | SRV:914-928, 4132-4139; JS:743-752, 799-815 (`fixtureMeshSpecs`/`partMeshKey`), 1046-1059 (`renderFixtureCompanions`), 9329-9343 (`loadFixtureForRegions`) |
| Lid/seat articulation per task "reality logic" | `fixtureStatesFromType('closed'|'lid_open'|'lid_seat_open')` | JS:1167-1172 |
| Washroom walls (venue `washroom_structure`) | `viewerManifest.washroom` → `buildWashroomWalls` (depthWrite off); hidden by default; toggle and opacity slider | JS:474-477, 9853-9874; TPL:736-737 |
| Floor grid (10 cm minor, 1 m major), base_link / ref frames, start EE, task frame | static helpers | JS:493-517; TPL:719-724 |
| Workspace/C-space overlays: reach voxels, manipulability heatmap, safety zones, unreachable, atlas band, ring cage, reach dome | Workspace tab (X2 exclusions) | TPL:729-735 |

Visibility toggles: Robot / Visual / Collision (TPL:726-728), one toggle per fixture part (TPL:703), Walls plus
opacity (TPL:736-737). In Lite, keep these and drop the X2 overlays.

### 1.4 The single client-side pose function (quoted)
JS:12423-12437:
```js
// poseMap: link -> 16-float row-major world transform (base_link). Decomposed
// into position/quaternion/scale so the meshes use the normal (auto-update)
// transform path, matching the fixture mesh.
const _rpM = new THREE.Matrix4(), _rpP = new THREE.Vector3(), _rpQ = new THREE.Quaternion(), _rpS = new THREE.Vector3();
function applyRobotPose(poseMap) {
    if (!poseMap) return;
    for (const [link, arr] of Object.entries(poseMap)) {
        const meshes = robotMeshes[link];
        if (!meshes || arr.length !== 16) continue;
        _rpM.set(...arr);                 // .set() takes row-major args
        _rpM.decompose(_rpP, _rpQ, _rpS);
        meshes.forEach(mesh => { mesh.position.copy(_rpP); mesh.quaternion.copy(_rpQ); mesh.scale.copy(_rpS); });
    }
    renderScene();
}
```
- It takes a **link-transform** map, not a joint map. A pose is a partial map: links that are not named keep
  their current transform.
- Call sites ("producers"):
  - atlas pose (JS:11917);
  - home pose (JS:11927, 12386);
  - IK preview (JS:12119);
  - Schedule (JS:13290, 13309);
  - plan file (JS:13496, 13521);
  - DAC (JS:14521);
  - sequence animation (JS:14660, 14681, 14683);
  - Simulate (JS:19281).
- Server FK is `UrdfModel.link_world_transforms(joint_map, base_transform)` (UK:311-349). It is a pure tree walk:
  joint origin (rpy) × Rodrigues motion (UK:78-128), with mimic resolution (UK:294-310). It is about 150 lines
  and would be straightforward to port to JS (inferred).

### 1.5 Camera and controls
- `initThree()` (JS:309-570):
  - `PerspectiveCamera(48°, near 0.001, far 100)`, Z-up;
  - WebGL creation falls back from `high-performance` to default to bare options, and shows a `.webgl-fail`
    notice if all fail (JS:318-344);
  - sRGB output with `NoToneMapping` (JS:325-330);
  - `OrbitControls` with damping 0.08 (JS:333-336);
  - hemisphere light plus key, fill and back directional lights (JS:428-438);
  - `TransformControls` gizmos (authoring only).
- Home View is a three-quarter view (45° azimuth, 30° pitch, JS:203-218). `fitCameraToScene()` includes the robot
  group (JS:1182-1197).
- Viewport islands: Orbit / Pan / Zoom / Fit / Overlays (TPL:842-848). `applyNavMode` remaps the left mouse button
  for trackpads (JS:4500-4512).
- Other controls: ViewCube canvas (TPL:828, `initViewCube` JS:7002), Home View button (JS:8919), "Maximize view"
  (TPL:947).
- Docks can be resized by drag **and keyboard**, 180–620 px, persisted in localStorage (`setDockWidth` /
  `initDockResize`, JS:7580-7642).

### 1.6 Performance facts
- **Rendering**: on demand only (`requestRender`/`animate`, JS:575-611). `renderScene()` just marks a frame dirty
  (JS:628-630). A `ResizeObserver` refits the canvas (JS:568).
- **Assets**: three.js **r128** UMD (`vendor/three.min.js` contains `const e="128"`) and Plotly **2.35.2** are
  vendored offline (`vendor/plotly.min.js` header). TRAJ uses the **same Plotly 2.35.2**, but loads it from the
  CDN (TRAJ:7). Plotly is lazy-loaded by `ensurePlotly()` (JS:5322-5337).
- **Server**: gzips responses over 8 KB (SRV:6369-6372), caps POST bodies at 64 MB (SRV:5178), and runs on a
  stdlib ThreadingHTTPServer with polling and no websockets (JS:19194-19199).
- **Frame payload, measured**: the cached sweep `/home/newtonjeri/alix_ws/src/alix/.cache/dual_arm_collisions/142c85b546d2617a.json`
  has 770 samples at dt 0.05 (38.5 s), 105 links per frame, and is **12.32 MB, about 16 KB per frame**. The cache
  directory holds 611 MB in 129 files.
- **Sweep cost**: 12.0–15.1 s per pair for the slim set
  (`/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/GENERATION_FINDINGS.md:3109`). The sweep builds its
  model and managers on every call (`docs/studio/STUDIO_GENERATE_TAB_PLAN.md:1931-1932`).

---

## 2. Playback (Q2)

### 2.1 Every existing player

| # | Player | Client | Server | Input | Joint units | Timing | Transport / UI | Speed |
|---|---|---|---|---|---|---|---|---|
| 1 | Sequence animation (Workbench) | `animateSequence`/`playAnimation`/`stopAnimation` JS:14600-14695 | `POST /api/animate-sequence` → `do_animate_sequence` SRV:1645-1760 | **Cartesian** poses `[{position, orientation}]` in base_link; IK per waypoint seeded from the previous solution; linear joint interpolation; frames for moving links only + `staticPose` | rad (internal) | fps 30, `speed_rad_s` 1.2, `max_frames` 900 (SRV:1730-1745); rAF in real time | Animate / Stop, loop toggle; **no scrubber** | fixed |
| 2 | DAC playback | `dacRunCheck`, `dacGoto`, `dacPlay`, `dacJumpHit` JS:14408-14598 | `POST /api/dual-arm-collisions` SRV:4335-4366 → DAPC:586-764 | two plan JSON paths (left, right) under the plan roots | rad in plan | uniform grid `dt` (UI default 0.05, range 0.01–0.5, TPL:624); `setInterval` at dt×1000 ms (≥20 ms) = **1× real time** (JS:14571) | ▶ Play / ⏸ toggle, ■ Stop, ⟸ Prev / Next ⟹ **collision**, slider, click the plot to scrub, click a log row to jump (`renderPlaybackBlock` JS:8749-8767, wired JS:8813-8817) | fixed 1× |
| 3 | Schedule & Run | `scheduleLoad`, `schPlay`, `schPlayOneStep`, `schPlaySegment` JS:12568-13735 | `GET /api/schedule`, `POST /api/schedule-preview` SRV:4695-5054; `POST /api/plan-frames` SRV:4590-4674 → `compute_plan_frames` DAPC:487-583 | orchestrator schedule CSV rows → resolved plans; `tracks` with start offsets; carried-in joints | rad | `SCH_DT 0.1`, `SCH_SPEED 2` = **2× real time** (JS:12505-12506, 13305) | Play all / Play step / Stop / Step ⟸⟹ / slider; DOM timeline bars, drag to reorder (JS:13601-13735) | fixed 2× |
| 4 | Plan-file player (Schedule tab, Browser dock) | `schPlayPlanFile`/`schPlayFileFrames` JS:13445-13560; TPL:759-768 | `/api/plan-frames {plans:[path]}` | any plan JSON under the plan roots | rad | dt 0.1, 2× | ▶ Play plan file / pause-resume | fixed 2× |
| 5 | Atlas pose (single pose) | JS:11895-11921 | `POST /api/atlas-pose` SRV:2412-2455 | `jointsDeg` (7 values, **degrees**) or `joints {name: rad}`, `arm`, `tipFrame` (gives the hand state), `poleM` | deg or rad | one pose | none | — |
| 6 | IK preview | JS:12100-12125 | `POST /api/ik` SRV:1095-1250 | Cartesian point → IK | — | one pose | none | — |
| 7 | Simulate (live) | JS:19202-19316 | `POST /api/sim/pose` SRV:5344-5393 | emulator registry or external joint push | rad (converted) | poll at 0 / 2 / 5 / 10 Hz | Go live / Refresh once | — |

Notes:
- No player has a user speed control (×0.5 / ×1 / ×2). The timing constants are hard-coded.
- `/api/analyze-plan` (SRV:4272-4293 → `PA.analyze_plan`) exists but has **no UI caller** in JS.
  - Per waypoint it reports FK tip position, manipulability, condition number, joint-limit margin, and URDF
    velocity-limit violations (PA:112-150).
  - The velocity check reads the plan's `velocities`, which a taught `.txt` does not have.
  - `acc_violations` is declared but never filled (PA:130).

### 2.2 The plan JSON format (the format every player and sweep consumes)
- Shape: `{task_name, sub_trajectories:[{index, stage_name, joint_names:[...], waypoints:[{time_from_start (s),
  positions (rad / m), velocities, accelerations}]}]}`.
- Example: `/home/newtonjeri/alix_ws/src/alix/Resource/plans/commode_c/moveit2_seeded_free/seat_ring_bottom_right_ruckig_on_s.json`
  has 11 stages: hand_release (IRH_*), move_pole_to_half_height (`R_sliding_plate_joint` 0.075 m),
  hand_grasp, move_to_pre_start, move_to_start, open_tenth, **execute_path** (4,341 waypoints at 0.01 s, 43.4 s),
  and so on.
- `load_plan_timeline` (DAPC:46-87) concatenates stages back to back and carries every joint's last value forward.
  `_sample_at` interpolates linearly (DAPC:90-107). `_concat_timelines` chains plans (DAPC:468-484).
- Plans are only accepted as `.json` files under the plan roots (`_resolve_plan`, SRV:4187-4197).
  - `plan_roots()` = `Resource/plans`, `core/python/src/alix_core/plans`, and the legacy root; `ALIX_PLANS_DIR`
    overrides them (`.../alix_taskgen/paths.py:110-135`).
  - `list_plans` tags each plan left/right/dual/neutral from its filename (SRV:4157-4184). Taught names such as
    `L_P01P02` contain neither "left" nor "right", so they would be tagged neutral (inferred).

### 2.3 Can a taught `.txt` be played? Yes, with an adapter (nothing accepts `.txt` today)
Taught format: NDJSON `{"point":[j1..j7]}` in integer millidegrees, e.g. `PKG70/LEFT/L_P01P02.txt` line 1 is
`[-653,-88886,986,91194,-2373,82010,175713]`. Files are 205–10,983 points; the 70-series totals 86,454 points.

Adapter specification:
1. **Parse.** Reuse the TRAJ parser `parseTrajectoryFile` (TRAJ:803-829) or EXP01 `teachlab/teachfile.py:252+`
   (`load_teach_file`, which checks units and limits).
2. **Units and names.** rad = mdeg × π / 180000. Joint names are `L_joint1..7` or `R_joint1..7` with a 1:1
   mapping. The same convention is used by `sim_bridge.joint_map` (SB:162-164) and `/api/atlas-pose`
   (SRV:2439-2446).
3. **Clock.**
   - TRAJ assumes 5 ms per point (`sampleMs`, TRAJ:367, 731). PKG70/README.txt says "5 ms per point".
   - The controller's **measured** player clock is **5.5545 ms** (`CLOCK_S`, EXP01/predict_track_replay.py:75).
   - `tools/taught_tracks.py` documents the clock as a band of 5.4–6.3 ms.
   - Use one constant for the simulation and the plot, show durations as a band, and convert collision time to a
     plot row with `row = round(t / clock)`.
4. **Wrap** as a one-stage plan. This is exactly EXP01/predict_track_replay.py:182-188:
   `{"sub_trajectories":[{"stage_name":"execute_path","joint_names":[...],"waypoints":[{"positions":rad,"time_from_start":i*CLOCK_S}]}]}`.
5. **Inject the joints that are not taught** (R7). Prepend 1-waypoint stages for the following; otherwise
   `_build_home` uses `dual_poles/full_length` = 0.300 m and fully open hands (DAPC:385-452), which puts the arm
   225 mm or 150 mm off its real position (DAPC:402-420).
   - `L_/R_sliding_plate_joint` = pole_mm / 1000;
   - hand joints from the workflow hand counts, using `SB.HAND_CHANNELS` (SB:95-110) with
     `closed = 1 − counts/1000` times each channel's travel (SB:176-186), or from an SRDF `inspire_hand_{side}`
     state;
   - the bin state (BIN_LOWERED by default).
6. **Workflow JSON → tracks.** `PKG70/LEFT_V6_70SERIES_FINAL.json` (`format: realman_workflow_sequence_v1`) has
   steps `pole{mm}`, `hand{values[6]}`, `program{id, name, speed, transit_speed, dual_rule}`, and `wait`. Map them to:
   - sliding-plate stages, with duration from 72.5 mm/s (PKG70/README.txt);
   - hand stages;
   - `execute_path` stages;
   - holds for waits.
   
   `dual_rule: TOGETHER` means both arms start at t=0, which is the DAC assumption. Alternating rows map to
   `compute_plan_frames(tracks=[{plans, start}])` (DAPC:487-520).
7. **Delivery.** Either write the JSON under `ALIX_PLANS_DIR` so `_resolve_plan` accepts it, or have a Lite server
   call `compute_plan_frames` / `compute_dual_arm_plan_collisions` directly. Both take **file paths**. For
   in-memory edits (R11), write temp files or add a wrapper that accepts `(times, rows)`; the internals already
   work on that form (DAPC:651-653).
8. **Size.** L_P03P05 is 10,983 points (about 61 s), which is about 1,220 frames at dt 0.05, or about 19.5 MB
   (16 KB per frame measured). Do not send a whole multi-program flow as one payload. Chunk per program, send only
   the moving links (the arm and hand prefixes), or **move FK to the client** (§1.4) and send joint arrays only.

---

## 3. Collision detection (Q3)

### 3.1 Engines found

| Engine | Code | Robot links checked | World objects | Granularity | Output | Used by Studio UI? |
|---|---|---|---|---|---|---|
| **A. DAC sweep** `compute_dual_arm_plan_collisions` | DAPC:586-764; called by `do_dual_arm_collisions` SRV:4335-4366 with `dt`, `window`, `return_frames=True` only | slim whitelist per arm: `{L,R}_Link1..7`, `*_CameraHolderLink`, `*_ConnectorLink`, `ILH_plam` / `IRH_plam`, plus `camera_link` on the right (DAPC:171-182). **No fingers.** | left vs right (DAPC:688-689); self per arm (DAPC:690-694); **body set = `bin_link` + `glove_wearing_unit_arm` only** (`kBodyCheckLinks`, DAPC:637-648); optional `extra_managers` (DAPC:699-703) — **never passed by any caller** (grep finds it only in DAPC); SRDF `disable_collisions` filtered (DAPC:112-123, 680-686; SRDF has 1,824 entries) | uniform `t = i·dt` with linear interpolation; discrete, no continuous collision detection | `{num_samples, dt, sample_times[], collisions:[{index, time, pairs:[[a,b],...]}], windows[], frames{str(i):{link:16}}, mesh_links (105), checks[], joint_timeline}` (DAPC:737-764). **No depth or distance**; `padding_m` only adds an unnamed `left_arm`/`right_arm` pair (DAPC:704-707) | Yes: DAC tab and Schedule "Check Collisions" (JS:13736-13785) |
| **B. Per-pose robot vs fixture** (IK preview) | `do_ik` SRV:1219-1250; `PosedRobotCollision` COL:366-410; `_fixture_collision_for` SRV:930-955; `fixture_contact_faces` COL:430+ | active-arm links (`_arm_link_filter`, RM:163-167) | commode meshes per manifest mesh key (UGV standoff shift supported) | one pose | `collisionFaces {meshKey: [face idx]}`; client paints fixture faces red (`paintCollisionFaces`, JS:12008, 12114) | Yes, Workbench. Joint-space capable: `set_pose(joint_map)` |
| **C. Plan vs walls** `PlanWallChecker` | PSV:179-256 (`load_wall_manager` PSV:117-155) | **all** collision links | venue `washroom_structure` mesh (default `washroom_800mm_wide_scene.yaml`) | coarse dt 0.05, then refined at 0.01 around hits | runs `{stage_idx, stage, links, t0, t1, depth}` (PSV:259-270) — **has depth** | No (generate_plans gate and CLI only) |
| **D. Full runtime world** `BodyCollisionWorld` | BC:130+; `fixture_manager` BC:401-464; `make_pose_filter` BC:499+ | moving = active arm (detected by FK); static = partner arm + UGV + bin + covers + poles; partner hand as envelope boxes | walls + commode `body/seat/lid` per `commode_fixture_type` (`FIXTURE_TYPE_STATES`, BC:104-108) | per pose, joints in **degrees** | blocking pair strings `"A<->B"` | Indirect (generator reach gate). `make_pose_filter` **needs a task config**; validated only for `commode_c` (BC:58) |
| **E. Cross-arm separation** | `cross_arm_separation` DAPC:1089-1283 | `full` / `hands` / `boxed` / `distal` link sets (DAPC:879-891), fingers included | arm vs arm | aligned and offset-invariant | `min_mm`, `min_pair`, `closest_samples[{t, mm, pair}]`, contacts | No. `_separation_context` is a shared `lru_cache` of FCL managers and is not thread-safe (DAPC:1016-1029; `docs/studio/STUDIO_GENERATE_TAB_PLAN.md:2774`) |

### 3.2 How the UI shows collisions (DAC) — quoted
- **Chart** `dacRenderPlot` (JS:14478-14497): a Plotly bar chart. Each sample is y=1 if it collides, else 0.
  Colliding bars are `#ff4d4d`, clear bars `rgba(0,212,255,0.25)`. The x axis is time in seconds, and
  `plotly_click` calls `dacGotoTime(t)`.
  ```js
  const y = x.map((_, i) => (dac.hitByIndex[i] ? 1 : 0));
  Plotly.react('dacPlot', [{ type: 'bar', x, y, width: (dac.res.dt || 0.05) * 0.9,
      marker: { color: y.map(v => (v ? '#ff4d4d' : 'rgba(0,212,255,0.25)')) }, ...
  ```
- **Pose and highlight** `dacGoto` (JS:14515-14529) calls `applyRobotPose(frame)`, then `dacHighlight(pairs)`,
  and writes the frame label `COLLIDING: L_Link3↔R_Link4` in `var(--danger)`. `dacHighlight` (JS:14531-14546)
  sets every mesh of each colliding link to `DAC_HIT_LINEAR` (red, JS:12446-12449) and restores
  `userData.baseColor` afterwards.
- **Play** `dacPlay` (JS:14561-14581) plays the **whole motion**. At the end it returns to t=0. It **never pauses
  at a collision**. The tooltip says "The full motion is always playable, collisions or not" (TPL:601).
- **Navigation**: `dacJumpHit` steps to the previous/next colliding sample (JS:14548-14559). `dacLogEvents`
  (JS:14272-14299) fills the right-hand "Collision Log" (TPL:895-899) with one row per colliding sample
  (`t=…s` + link pairs); clicking a row jumps there.
- **Fixture reality logic**: `dacReconcileFixture` (JS:14332-14354) refuses a pair whose regions need different
  lid/seat states. The states come from task configs via `viewerManifest.regions[*].commodeFixtureType`, which
  makes this part config-coupled.
- **Theme defect**: the chart font is hard-coded `#e6edf3` (the old dark palette, JS:14488; also JS:5368). The
  default chrome is light (`--surface-ribbon-2: #F4F6F8`, CSS:67; dark theme is opt-in only, CSS:130-134), so
  tick labels are close to invisible (inferred from the colours). Lite should read the CSS tokens instead.

### 3.3 Is there anything matching "the graph stops at the corresponding joint value and highlights the collision point"?
**No.** Nothing plots joint values (only the charts at JS:5368 and JS:14483 exist; a grep for joint plots finds
nothing). Playback does not stop at a collision (a grep for stop/pause-on-collision finds nothing). No joint value
is highlighted.

Partial building blocks that can be reused:
- a time marker for each colliding sample (red bar);
- 3D link highlighting (`dacHighlight`);
- Prev/Next collision;
- a clickable log.

Data that already exists: `joint_timeline` (DAPC:668, 755) holds the joint map at every sample, but the server
drops it (SRV:4357). The collision `index` and `time` are in the result.

Suggested Lite design, combining TRAJ and DAC:
1. Keep `joint_timeline`, or compute joints client-side from the edited rows.
2. Use **one clock** to convert a collision time to a TRAJ row.
3. TRAJ already draws a red playback cursor line as a Plotly shape (`updateOriginalPlotShapes`, TRAJ:1134-1141)
   and has rAF playback (`playOriginal`, TRAJ:1217-1231; `rowForTime`, TRAJ:1181). Extend it:
   - on the first collision row, call `stopPlayback()` (TRAJ:1210);
   - pin the cursor to that row;
   - add a translucent red `rect` over the collision run, red marker points at (row, q_j) for the 7 joints, and an
     annotation `L_Link3 ↔ R_Link4`;
   - call the 3D `dacHighlight(pairs)` at the same moment.
4. "Back to editing" pre-selects a region [row − w, row + w] with TRAJ's `addRegion` (TRAJ:1392) or the selection
   handler (TRAJ:872-889).

### 3.4 Correctness caveats Lite must carry or fix
- The commode and the walls are **not** in the DAC world (`extra_managers` is unused). To add them, pass
  `{"commode": BodyCollisionWorld.fixture_manager(fixture, fixture_type, standoff), "walls": load_wall_manager(scene)[0]}`
  as `extra_managers` (BC:401-464, PSV:117-155). Pairs come back as `<link>↔<key>` (DAPC:699-703).
- **Fingers are excluded** (palms only). `GENERATION_FINDINGS.md:3109` recommends widening the set (2–3 min per
  pair instead of 12–15 s).
- **SRDF suppresses two real contacts**: `alix.srdf:1527, 1537` disable left-thumb vs right-camera pairs
  (`GENERATION_FINDINGS.md:3095-3108`).
- The server passes no hand state, so the hands are fully open unless the plan drives them (DAPC:390-401). The
  pole defaults to 0.300 m (DAPC:402-420).
- Sampling is discrete at dt. At 225°/s and dt 0.05 a joint can move up to about 11° between checks (inferred).
  `PlanWallChecker` handles this with refinement at 0.01 (PSV:219-256).
- It is "a visualization aid — results need not exactly match MoveIt" (DAPC:1-22; TPL:601).
- PKG70/README.txt says the merged and bridged files are "**NOT collision-checked** against the toilet, cubicle
  walls, second robot, end-effector, or right-arm camera". This is exactly the gap that engine A plus
  `extra_managers` (and widened hand links) closes for R8.

---

## 4. Is there a split "sim on one side, plots on the other" layout? (Q4)
**No left/right split exists.**
- **DAC tab** (TPL:599-639):
  - A top **ribbon band** capped at `--cmdbar-h: 210px` (CSS:1722; `.ribbon-body` CSS:211).
  - Inside it: `.dac-main`, fixed at 430 px wide (CSS:1102), with schedule pair, path mode, left/right pickers
    with search, `dt`, and Check; and `.dac-tl` (flex, min 320 px, CSS:1107) with a transport head row
    (`.tl-head`, CSS:1121-1128) and `#dacPlot` (min-height **64 px**, CSS:1108).
  - The 3D viewport sits **below** the band in `.workspace-row` (TPL:742), between the Browser dock (left) and
    the Properties dock (right; "Collision Log", TPL:895-899).
- **Schedule & Run** (TPL:642-672) follows the same pattern, with a DOM timeline instead of Plotly
  (`sch-main` / `sch-tl`, CSS:1131-1195).
- **Simulate** hides the whole ribbon (CSS:1379-1381) and shows `.sim-panel` above the viewport (CSS:1383-1406).
- Reusable pieces for an R10 layout:
  - the dock-resize splitter pattern (JS:7580-7642);
  - `renderPlaybackBlock` (JS:8749-8767);
  - the on-demand renderer with ResizeObserver.
  
  Note that `onResize` only calls `Plotly.Plots.resize('analysisPlot')` (JS:632-645). New plots must use
  `responsive: true` or be resized explicitly.
- **Verdict:** the R10 layout (3D left, plots right, resizable) must be **new** CSS grid work, assembled from the
  Studio's viewport, docks and transport plus TRAJ's plots.

## 5. Bin (Q5)
- **Drawn**: yes. `bin_link` and `bin_rail_link` are in the visual bake. `bin_handle_left/right` and
  `lid_opening_cover1/2` are collision-only (SRV:1981-1986).
- **Pose**: always `BIN_LOWERED` (SRDF `bin_rail` states: `BIN_STOWED` rail 0 / drop 0; `BIN_EXTENDED` 0.2 / 0;
  `BIN_LOWERED` 0.2 / 0.45). This is hard-coded in:
  - geometry home (SRV:1991);
  - IK (SRV:1135);
  - analyze-points (SRV:1529);
  - animate (SRV:1725);
  - DAC and frames home (DAPC:440-441).
  
  The UI states it: "Bin is always LOWERED" (TPL:753; `do_joint_presets` docstring SRV:1763-1768).
- **Collision world**:
  - DAC body set = `bin_link` + `glove_wearing_unit_arm`. They can only be switched together, via the library
    parameter `include_body` (DAPC:595, 646-648), which the server does not expose.
  - In `BodyCollisionWorld` the bin and covers are robot links (BC:13-18 docstring).
- **Toggle**: **none**. `grep -nw bin` in JS/TPL finds only comments and labels.
- **Related rule to reuse**: `STOWED_POLE_FOUL_BAND_M = [0.153, 0.282]` m (JS:17144-17149, with the warning text at
  JS:17181-17185). With the bin **stowed**, a pole height in that band makes the arm base hit the bin lid covers
  (worst −61.8 mm). Lite's bin-exclude/stowed option must warn about this.
- **Lite implementation sketch**:
  - visual: set `robotMeshes[l].visible=false` for the bin links;
  - pose: SRDF `group_state("bin_rail", BIN_STOWED|EXTENDED|LOWERED)` in the home and frames joint map;
  - collision: a Lite-side body manager with `{bin_link, bin_handle_*, lid_opening_cover*}` and the glove lever as
    separate `extra_managers` keys (needs `include_body=False` plus custom managers), or a small library change
    that makes `kBodyCheckLinks` a parameter.

## 6. Simulate view (Q6)
- **UI**: view switch Author | Simulate (TPL:61-64). Sim panel (TPL:79-110): status dot; Source = Auto /
  Emulator / External feed; Poll = Paused / 2 / 5 / 10 Hz; Go live; Refresh once; readout; caveat ("Command-level,
  not dynamics").
- **Client** (JS:19202-19316): `setView`, `simRefreshStatus` (GET `/api/sim/status`), `simTick` (POST
  `/api/sim/pose {source}`, then `applyRobotPose(data.pose)`), `simStartLive` / `simStopLive` (setInterval).
- **Server** (SRV:5296-5393):
  - `do_sim_status` reports emulator arms plus the external feed's presence and age.
  - `do_sim_joints` **accepts `{joints:{name: value}, source}` from any process**; the docstring says "future: the
    real arms".
  - `do_sim_pose` reads the emulator first, then the external feed. It layers the joints over `ctx["initial"]` and
    returns the FK pose for all links.
- **Emulator bridge** (SB):
  - It reads `alix_emulator.rm_emulator._by_ip` **in-process only** (SB:112-149), i.e. `tools/run_studio.py
    --emulator` (lines 5, 22-24, 142-160).
  - Supported on Linux/Windows only (`docs/EMULATOR.md:14-41`).
  - Unit conversion: deg→rad, lift hw→m, hand counts→rad (SB:152-186). It reads, never writes (D4).
- **Can it show the REAL robot?** Yes, with no Studio change, through the external feed:
  1. Poll each controller with `rm_get_current_arm_state()`. The runner already wraps this as
     `arm_state(arm) -> (joint_deg[7], err_codes)` (RUN14:122-139). Also read lift and hand state.
  2. Convert to `{L_joint1..7, R_joint1..7 (rad), *_sliding_plate_joint (m), hand joints}`.
  3. POST **both arms in one body**. `do_sim_joints` **replaces** the stored map (SRV:5338), so per-arm posts would
     wipe the other arm.
  4. The client polls at 2–10 Hz.
- **"Simulation vs real" on the controller** (R4): the runner's `set_arm_run_mode` / `get_arm_run_mode`
  (RUN14:1213-1258) switch the RealMan run mode (0 = SIMULATION, 1 = REAL). In controller simulation mode the
  program runs virtually (inferred from vendor semantics). Feeding its joint state into the same view gives a
  controller-timed simulation in the Studio viewport.
- **Gap**: the in-repo emulator implements 75 SDK methods (`/home/newtonjeri/alix_ws/src/alix/core/python/src/alix_emulator/rm_emulator.py`).
  It does **not** emulate the program/project/drag-teach calls the runner uses (`rm_send_project`,
  `rm_set_program_id_run`, `rm_get_program_run_state`, `rm_save_trajectory`, `rm_start_drag_teach`, …). The upload
  channel therefore cannot run against the emulator unchanged.
- **No live collision check** exists in the Simulate view today. Lite could run an engine-A-style check per polled
  sample, which takes milliseconds with persistent managers (COL:366-410 pattern), and reuse `dacHighlight`.

## 7. Coupling to excluded features (Q7)

| Feature | Needs geometric path / cleaning config / C-space? | Joint-space only? |
|---|---|---|
| Robot geometry and `applyRobotPose` | No (URDF + workspace config only) | n/a |
| Fixture and walls rendering (`/api/manifest`) | No configs. Needs fixture `scene.yaml` + meshes (`.../ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml`) and venue YAML | n/a |
| DAC sweep and playback (`/api/dual-arm-collisions`) | No. Needs plan JSONs under the plan roots. Optional coupling: `/api/schedule-pairs` (orchestrator CSV) and `dacReconcileFixture` (config-derived `commodeFixtureType`) | **Yes** |
| `/api/plan-frames` | No (the optional `mech` body uses mechanism configs, SRV:4607-4617) | **Yes** |
| Schedule & Run | Orchestrator `schedule_<family>.csv` + `regions_<family>.csv` + plan naming (SRV:4708-4760) — not C-space, but not Lite's workflow JSON | joint-space plans |
| Sequence animation `/api/animate-sequence` | **Yes** — Cartesian poses / spline (X1) | no |
| IK preview collision `/api/ik` | **Yes** — Cartesian point (X1). The face-marking engine underneath is joint-capable | partially |
| `/api/atlas-pose` | Endpoint is generic joint FK; its UI is the redundancy atlas (X2) | **Yes** |
| `/api/path-compare`, `/api/path-ik` | **Yes** — geometric path / C-space oracle (SRV:5936-5958; `studio_generate.py:4046, 4243`) | no |
| `BodyCollisionWorld.make_pose_filter` | **Yes** — task config `stage_sequence`, `ik_frame` | no (but `fixture_manager` and walls are not coupled) |
| Simulate view (`/api/sim/*`) | No | **Yes** |
| `/api/analyze-plan` | No | **Yes** |

## 8. Reuse assessment and what to extract (Q8)
1. **Viewport core: adapt.** Lift `initThree` (JS:309-570, without paint/gizmo/TransformControls),
   `requestRender`/`animate`/`renderScene`/`onResize` (JS:575-645), `meshFromPayload` (JS:667-703),
   `loadRobotGeometry`/`setRobotSet`/`robotSetVisible` (JS:12352-12420), `applyRobotPose` (JS:12423-12437),
   `fitCameraToScene` (JS:1182-1197), nav islands and ViewCube (JS:4500-4530, 7002-7060), and dock resize
   (JS:7580-7642). Use the vendored `three.min.js` r128 + OrbitControls.
2. **Robot geometry endpoint: reuse as-is** (`/api/robot-geometry`, SRV:1864-1998 + `_ik_context` RM:169-228),
   including the baked npz.
3. **Fixture and walls rendering: reuse with adaptation.**
   - Server: `get_manifest` / `scene_manifest_cached` (SRV:914-928, 4132-4139).
   - Client: `manifestParts`, `fixtureMeshSpecs`, `partMeshKey`, `fixtureStatesFromType`,
     `renderFixtureCompanions`, `buildWashroomWalls`, `updateWallsVisibility` (JS:743-830, 1046-1059, 1167-1180,
     9853-9874).
   - The state comes from Lite's per-region/step setting instead of configs.
4. **Plan frames: reuse with an adapter.** `compute_plan_frames` (DAPC:487-583) via `/api/plan-frames`
   (SRV:4590-4674), fed by the `.txt` → plan adapter (§2.3).
5. **DAC sweep: adapt.**
   - Call `compute_dual_arm_plan_collisions` with `extra_managers` for the commode and walls, explicit
     `left/right_hand_state`, pole stages injected, and optionally a widened whitelist.
   - Keep `joint_timeline` (do not pop it as SRV:4357 does).
   - Cache by content hash as `_dac_cache_file` does (SRV:4303-4316), but in a Lite cache directory, not
     `REPO/.cache`.
6. **DAC client: adapt.** `dacRunCheck`, `dacGoto`, `dacGotoTime`, `dacHighlight`, `dacJumpHit`, `dacPlay`,
   `dacLogEvents`, and `renderPlaybackBlock` (JS:14272-14598, 8749-8767). Add: stop at the first collision, a
   speed selector, joint-plot linkage, and token-based Plotly colours.
7. **Collision bar chart: reference only.** Replace it with joint plots plus collision bands (§3.3).
8. **Walls check with depth: reuse.** `PlanWallChecker` (PSV:179-270) for per-arm wall contact depth.
9. **Fixture world: reuse.** `BodyCollisionWorld.fixture_manager` (BC:401-464) and `FIXTURE_TYPE_STATES`
   (BC:104-108). `make_pose_filter` is reference only (needs configs).
10. **Separation distance: reference / adapt.** `cross_arm_separation` (DAPC:1089-1283) for a "closest approach
    (mm)" readout, with per-request managers (not the shared lru_cache).
11. **Single-pose joint FK: reuse as-is.** `/api/atlas-pose` (SRV:2412-2455) for R7 pre-teach previews (pole plus
    hand) and for R9/R14 offset previews of one point.
12. **Simulate view: reuse with adaptation.** `/api/sim/status|joints|pose` (SRV:5296-5393), `sim_bridge` units
    (SB:95-186), and the client (JS:19202-19316). Add a real-robot poller that pushes both arms in one body, and a
    Real/Sim source badge.
13. **Schedule & Run timeline: reference only.** The proportional DOM timeline and pause/resume model
    (JS:13227-13735) could inspire a workflow-JSON timeline, but its data source is the orchestrator CSV.
14. **Sequence animation, path-compare/path-ik, IK preview: not applicable** (X1/X2-coupled). Keep only the FK loop
    pattern from SRV:1747-1757.
15. **Plan analysis: reference / adapt** for R12. `PA.analyze_plan` gives joint-limit margin and manipulability per
    sample. Velocities must be computed by finite differences for taught data.

---

## Requirement relevance
- **R3 Simulation playback.** Borrow the viewport (§1), the DAC/plan-frames players (§2.1 rows 2–4) and the
  `.txt` → plan adapter (§2.3). The existing transports have no speed control and fixed dt. Plan for client FK to
  get smooth 5 ms scrubbing.
- **R4 Real-arm testing / real-sim toggle.** The real robot can be shown in the same viewport via
  `/api/sim/joints` (SRV:5326-5341), fed from `arm_state()` (RUN14:122-139). The controller run mode 0/1 lives in
  RUN14:1213-1258. The emulator cannot run the upload channel's program APIs.
- **R5 Collision detection and graph highlighting.** The detection engine exists (DAPC:586-764). The 3D link
  highlight and log exist (JS:14531-14546, 14272-14299). Stop-at-collision and the joint-value highlight do
  **not** exist and must be built from TRAJ's cursor shapes plus `joint_timeline` (§3.3). The world must be
  extended with the commode and walls (§3.4).
- **R6 Adjustable plots.** The Studio offers resizable docks (JS:7580-7642) and Plotly `responsive`. The DAC plot
  is cramped (64 px minimum in a 210 px band). TRAJ is the better basis for plot UX.
- **R7 Pole height and hand position.** SRDF pole states (0.01 / 0.075 / 0.15 / 0.225 / 0.3 m), SRDF hand states
  (`release`, `grasp`, `half_grasp`, `quarter_grasp`, `tenth_grasp`, `open_tenth`, `thumb_grasp`, `pointing`,
  `lid_open` on the left), the `poleStateSelect` UI (TPL:748-755) and `/api/atlas-pose {poleM, tipFrame}` can
  preview the pre-teach configuration. The same values must be injected into the simulated timelines (§2.3 step 5).
- **R8 TXT merging with the collision scene.** The PKG70 bridges are not collision-checked (README). Engine A with
  `extra_managers` (commode + walls) plus `PlanWallChecker` is how to check merged files.
  `compute_plan_frames(tracks=…)` can model alternating or offset rows.
- **R9 / R14 Offset via FK → IK.** FK is `UK.link_world_transforms`. Continuity-seeded IK is shown in
  `do_animate_sequence` (seed from the previous solution, SRV:1702-1716) using `alix_workspace.ik.IKSolver`. Joint
  limits come from the URDF (§1.1). Simulate the result through the same DAC pipeline before running on hardware.
  `path_ik` is X1-coupled (reference only).
- **R10 Dual-arm view.** Use the URDF and meshes (§1.1–1.3), with the drift check (xacro newer than the URDF). The
  left/right split layout is new. The bin toggle is new (§5); respect the stowed-bin foul band.
- **R11 Smoothness with immediate validation.** The cost is a 12–15 s sweep per pair plus model build per call.
  "Immediate" needs cached FCL managers per worker thread, a windowed re-check of only the edited rows, and
  client-side FK. Content-hash caching already exists (SRV:4303-4316).
- **R12 Spike analysis.** The Studio has none for joints. `PA.analyze_plan` (limits, manipulability) and the URDF
  velocity limits give thresholds. TRAJ and `taught_tracks` own spike/step gates (another map).
- **X1 / X2.** Drop: animate-sequence, path-compare / path-ik, IK preview, atlas UI, the reach/manip/safety/atlas
  overlays (TPL:729-735), and the Workspace tab. Keep the joint-space engines (DAPC, PSV, COL fixture marking).
- **NFR-UX.** The DAC tab is expert-oriented (seven controls, dt field, schedule/plan-mode pickers). Lite should
  hide dt and mode, auto-pair from the region/workflow, and show a plain-language verdict plus "Go to problem" and
  "Back to editing" buttons.
- **NFR-THEME.** Tokens live on `:root` (CSS:1-129): brand `#2F96EE`, light chrome, dark canvas `--viewport-bg`
  `#1B2733`, `--danger #D93A34`, `--ok #2E9E5B`. The DAC/analysis plots ignore these tokens (`#e6edf3`) — fix that
  in Lite.
- **INT-CTRL.** The control app's joint-state reader (RUN14:122-139) and run-mode switch (RUN14:1213-1258) are the
  hooks for the live view and the real/sim toggle.
- **INT-TRAJ.** TRAJ has no 3D view and explicitly says it "cannot prove … collision safety without … simulation
  data" (TRAJ:437, 1155). Its cursor, playback and region model (TRAJ:1134-1141, 1181-1231, 1392) are the
  integration points for §3.3.

## Reuse verdicts

| Feature | Verdict | What to lift | Evidence |
|---|---|---|---|
| Robot model loading (URDF, visual/collision meshes, bake) | reuse-as-is | `/api/robot-geometry`, `_ik_context`, `visual_overlay_baked.npz` | SRV:1864-1998; RM:169-228; COL:44-246 |
| Single pose applier | reuse-as-is | `applyRobotPose` + `robotMeshes` + `meshFromPayload` | JS:12423-12437, 667-703, 12352-12420 |
| Viewport shell (camera, lights, nav, on-demand render) | reuse-with-adaptation | `initThree` (trimmed), `requestRender`/`animate`, `fitCameraToScene`, `applyNavMode`, ViewCube | JS:309-645, 1182-1197, 4500-4530, 7002-7060 |
| Fixture (commode) + walls rendering | reuse-with-adaptation | manifest endpoint, `fixtureMeshSpecs`, `renderFixtureCompanions`, `buildWashroomWalls` | SRV:914-928, 4132-4139; JS:743-830, 1046-1059, 9853-9874 |
| Plan JSON timeline + FK frames | reuse-with-adaptation | `load_plan_timeline`, `_sample_at`, `compute_plan_frames` + `.txt` adapter | DAPC:46-107, 468-583; SRV:4590-4674; EXP01/predict_track_replay.py:182-188 |
| DAC FCL sweep | reuse-with-adaptation | `compute_dual_arm_plan_collisions` with `extra_managers` (commode + walls), hand states, pole injection, keep `joint_timeline`; consider a widened whitelist | DAPC:586-764, 171-177, 699-703; SRV:4335-4366 |
| DAC playback / highlight / log UI | reuse-with-adaptation | `dacGoto`, `dacHighlight`, `dacJumpHit`, `dacPlay`, `dacLogEvents`, `renderPlaybackBlock`; add stop-on-collision and speed | JS:14272-14598, 8749-8767 |
| Collision bar chart | reference-only | concept of time-indexed collision marks; replace with joint-plot bands | JS:14478-14497 |
| "Graph stops at joint value + highlight" | not-available | build from TRAJ cursor shapes + `joint_timeline` | JS:14561-14581 (no stop); DAPC:755; TRAJ:1134-1141 |
| Robot vs commode per pose with face painting | reuse-with-adaptation | `PosedRobotCollision`, `_fixture_collision_for`, `fixture_contact_faces`, `paintCollisionFaces` | COL:366-430; SRV:930-955, 1219-1250; JS:12008 |
| Walls sweep with depth | reuse-as-is | `PlanWallChecker`, `load_wall_manager`, `summarize_runs` | PSV:117-270 |
| Commode collision world by articulation | reuse-with-adaptation | `BodyCollisionWorld.fixture_manager`, `FIXTURE_TYPE_STATES` (not `make_pose_filter`) | BC:104-108, 401-464, 499 |
| Arm-to-arm minimum distance | reference-only | `cross_arm_separation` (thread-safety caveat) | DAPC:1089-1283, 1016-1029 |
| Joint-space single-pose FK endpoint | reuse-as-is | `/api/atlas-pose` (jointsDeg, poleM, tipFrame) | SRV:2412-2455 |
| Live view (emulator / external) | reuse-with-adaptation | `/api/sim/*`, `sim_bridge` unit maps, client poller; add a real-robot pusher (both arms per POST) | SRV:5296-5393; SB:95-186; JS:19202-19316; RUN14:122-139 |
| Split layout (3D left, plots right) | not-available | build new; reuse dock-resize splitter + transport block | TPL:599-639; CSS:1097-1128, 1722; JS:7580-7642 |
| Bin include/exclude toggle | not-available | build new: hide bin meshes, SRDF bin states, separate body managers; stowed-pole foul-band warning | SRV:1991; DAPC:644-648; JS:17144-17185 |
| Pole / hand presets for pre-teach | reuse-with-adaptation | `/api/joint-presets` (SRDF pole states), SRDF hand states, `_hand_joint_values` | SRV:1763-1790, 1080-1092; TPL:748-755 |
| Schedule & Run timeline | reference-only | proportional DOM timeline, pause/resume model | JS:13227-13735 |
| Sequence animation / path-compare / path-ik / IK preview | not-applicable | X1/X2-coupled | SRV:1645-1760, 5936-5958, 1095-1250 |
| Plan analysis (limits, manipulability) | reference-only | `analyze_plan` per-sample metrics for R12/R9 | PA:74-188; SRV:4272-4293 |

## Gaps and risks (summary)
- No joint plot, no stop-at-collision, and the server drops `joint_timeline`.
- The DAC world has no commode, no walls and no fingers. SRDF disables two real thumb-vs-camera pairs. The default
  pole is 0.300 m and the default hands fully open.
- Frame payloads are about 16 KB per frame. Multi-program flows reach hundreds of MB unless chunked or FK moves to
  the client.
- A sweep takes 12–15 s per pair and rebuilds its models on every call. `cross_arm_separation` shares FCL managers
  across threads (crash risk).
- The DAC/analysis Plotly colours (`#e6edf3`) do not match the light theme.
- The URDF snapshot is older than the xacro. `_config_signature` misses URDF/SRDF edits (missing `import yaml`).
- The emulator cannot exercise the program/upload APIs. A real-robot feed must post both arms at once.
- `_resolve_plan` only reads `.json` under the plan roots. A Lite server must use `ALIX_PLANS_DIR` or call the
  library directly.

## Open questions
- Which clock should Lite simulate with: 5 ms (TRAJ, README) or the measured 5.5545 ms band? It affects collision
  time → row mapping and dual-arm phasing.
- Should the Lite collision gate include fingers (full link set, about 2–3 min per pair) or keep the slim set plus
  hand envelope boxes (`hand_envelope_box`, DAPC:832-844)?
- What is the commode state (closed / lid_open / lid_seat_open) per region/flow? The PKG70 README says "LID OPEN
  MANUALLY" after Flow 5. What are the UGV standoff and venue (800 / 1000 / 1400 mm scene) for the Lite site?
- Can a second TCP client poll `rm_get_current_arm_state` while the runner is uploading or running programs, or
  must the runner itself push joint state (realtime UDP push, `rm_set_realtime_push`)?
- Is the expanded `alix.urdf` still identical to the current `alix.urdf.xacro` (Oct 7)? R10 requires the correct
  URDF.
- Does "bin exclusion" mean "bin removed from the collision world", "bin stowed" (pose change), or both?
