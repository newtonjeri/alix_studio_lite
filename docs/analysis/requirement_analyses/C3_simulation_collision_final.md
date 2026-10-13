# C3: Simulation playback, collision detection and the dual-arm speed view (R3, R5, R10) — verified and corrected

This is the reviewed version of the C3 analysis for **ALIX Studio Lite**. It is written for the engineers who will build
Lite and for the requester, who wrote the inputs. Each requirement has a clear verdict on what can be borrowed from ALIX Studio
and an approach for the rest.

**How this version was checked.**
- Every code claim in the analyst draft was opened at its cited `path:line`. The draft is mostly accurate. Where it was
  wrong, imprecise or missing something, this version corrects it and records the change in the change log (§6).
- **[M]** marks a measurement from the analyst draft.
- **[V]** marks a measurement re-run by this verifier: in-memory Python (`python3 -I -B`, with the user-site
  `alix_workspace`/`alix_taskgen` on `sys.path`) on the real V6 files and the ALIX libraries. No files were written and
  no robot or network was used.
- "(inferred)" marks reasoning, not observation.
- For the upload-channel programs (control-app UI and runner) this document stays at a high level. It names only the
  current version and the issues that block the plan, one line each.

## Path legend

| Alias | Absolute path |
|---|---|
| `TG/` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/` (ALIX Studio) |
| `BC` / `RM` / `VGC` | `TG/body_collision.py` / `TG/robot_model.py` / `TG/scripts/visualize_generated_config.py` |
| `WS/` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_workspace/src/alix_workspace/` |
| `DAPC` / `PSV` / `COL` / `UK` / `RED` | `WS/dual_arm_plan_collision.py` / `WS/plan_scene_validation.py` / `WS/collision.py` / `WS/urdf_kinematics.py` / `WS/redundancy.py` |
| `DAV` / `DAF` | `/home/newtonjeri/alix_ws/src/alix/tools/dual_arm_verdicts.py` / `/home/newtonjeri/alix_ws/src/alix/tools/dual_arm_flow.py` |
| `TRK` / `EMU` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_dispatch/src/alix_dispatch/track.py` / `.../alix_dispatch/emulated_program.py` |
| `RUNS/` | `/home/newtonjeri/alix_ws/src/alix/Resource/runs/tracks/` (ALIX hardware run records) |
| `SRDF` | `/home/newtonjeri/alix_ws/src/alix/ros/manipulation/alix_moveit_config/config/alix.srdf` |
| `CL` / `GF` | `/home/newtonjeri/alix_ws/src/alix/ros/orchestration/alix_tasks/config/contact_links.yaml` / `TG/config/glove_frames.yaml` |
| `HU` | `/home/newtonjeri/alix_ws/src/alix/ros/hw/alix_hw/scripts/hand_units.py` |
| `CCS` | `/home/newtonjeri/alix_ws/src/alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml` |
| `BINX` / `TCFG` | `/home/newtonjeri/alix_ws/src/alix/ros/description/ugv_description/urdf/bin.xacro` / `/home/newtonjeri/alix_ws/src/alix/ros/orchestration/alix_orchestrator/alix_orchestrator/core/task_config.py` |
| `EMUDOC` / `CANFD` / `GENF` | `/home/newtonjeri/alix_ws/src/alix/docs/EMULATOR.md` / `.../docs/motion/TAUGHT_CANFD_PIPELINE.md` / `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/GENERATION_FINDINGS.md` |
| `AVI` | `/home/newtonjeri/alix_ws/src/alix/ros/description/rm_description/docs/arm_version_info.md` |
| `SDK/` | `/home/newtonjeri/alix_ws/src/alix/external/butterfli-realman-alix/Python/Robotic_Arm/` |
| `TRAJ` | `/home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html` |
| `RUN14` / `UI53` / `POLE` | `/home/newtonjeri/alix_studio_lite_ws/upload_channel/rm_upload_fast_v14_safe_transit.py` / `.../rm_trajectory_ui_v53_safe_lid_checkpoint.py` / `.../pole.py` |
| `V6/` | `/home/newtonjeri/alix_studio_lite_ws/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/` |

---

## 1. Summary

| Req | Borrow from ALIX Studio? | What must be built | Priority | Effort |
|---|---|---|---|---|
| **R3** Simulation playback | **Partial.** The 3D viewer, robot mesh payload, pose applier, toilet and wall renderers, transport markup and toilet-face painting can be lifted. The Studio's players only take MoveIt-style plan JSON from the repo's plan folders, at fixed speeds, with a 0.300 m pole, open hands and a lowered bin. | An in-memory `.txt` adapter that seeds pole, hand and bin. A player with a speed control kept in sync with the joint plots. Browser FK with a parity test. A **wiped-area** view for "missed spots", computed from the hand's contact links. The jump back to the editor. A per-region verdict. Flow timing from laws measured on the robot. | Must | L (13-15 days) |
| **R5** Collision detection and graph highlighting | **Partial.** Borrow the 3D red-link highlight, the collision log with Previous/Next, the toilet collision manager and toilet-face painting. The Studio never plots joints, never stops at a collision, checks no toilet, walls or fingers, assumes both arms start together, and has no real-arm input. | A complete collision world checked at every row. Contact versus collision decided by the hand preset. A drift window for two-arm timing. Checks of approach moves, pole moves and hand changes. Joint markers and stop-at-collision on the plots. The real-arm path from a 5 ms joint capture. | Must | XL (19-21 days) |
| **R10** Dual-arm speed view, correct model, bin toggle | **Partial.** The URDF is correct ([M], confirmed [V]). The baked meshes are stale (old tall bin, no camera). There is no split view and no speed analysis. The bin is fixed LOWERED in at least seven places. | Split layout. Re-baked overlay with tests. The bin "fitted / position" control. A dual-lane timeline with v53 rules and measured timing laws. Velocity and acceleration headroom. Speed controls linked to C2. | Should; (b) model and (c) bin are Must and ship with R3/R5 | L (13-15 days) |

All three input notes say "borrow from ALIX Studio". That is **partly accurate** each time. The Studio gives a 3D view
and a collision engine, but each requirement still needs a substantial new part.

---

## 2. Shared foundations (corrected)

1. **One row index, one clock.**
   - A taught `.txt` has no timestamps. Index everything by **row per arm**: the 3D pose, the plot cursor, collision
     events and real-arm stops. Convert to seconds only for display.
   - 1× playback and all durations use the controller's fitted player law: `0.2322 s + 5.6737 ms × (N−1)`, with a
     run-to-run band of 5.35-6.10 ms per row (`TRK:1558-1570`).
   - Rate checks use the 5.0 ms player cycle, the worst case.
   - The trajectory HTML assumes 5 ms (`TRAJ:1176-1179`), so its times read about 10 % short. It also numbers rows
     from 1 (`TRAJ:1181-1187`), while ALIX sample indices start at 0. Pick one convention on screen.
2. **Everything around a program has its own time, and ALIX has measured most of it.**
   - **Start lag**, from go to replay start: `0.351 s + 0.41 ms × N`, residual band −1.07…+0.99 s (`DAV:176-224`).
   - **Approach move** (the controller's own movej to the first point): **[V]** 283 ALIX hardware run records at run
     speed 50 % took about 0.65 s for gaps up to 10°, about 5 s for 45-90° and about 5.9 s for about 166°. That is 5-10×
     the emulator's law of max Δq / (180 °/s × v%) (`EMUDOC:73`). Examples: `RUNS/2026-09-02/run_slot76_210943.json`
     (93.6° in 6.95 s) and `RUNS/2026-09-01/run_slot70_190758.json` (59.7° at 20 % in 8.94 s).
   - **Settle**: 0.6-1.0 s (`RUN14:1947-1949`).
   - **Pole**: `lift_travel_s` (`POLE:3-14`).
   - **Hand**: about 115 ms + 373 × span / speed ms (`EMUDOC:84`).
   - Re-fit all of these on Lite's own runs. Lite uses 30 % transit speed and its own runner.
3. **Two arms on the upload channel are not row-locked.**
   - Each controller does its own approach move and start lag, then plays on its own clock.
   - **[V]** ALIX's measured envelope (`DAV:912-973`) gives a relative-phase window of **±3.56 s** for two
     3,751-row tracks released together.
   - ALIX refuses to extrapolate beyond 4,281 rows (`DAV:862-880`). V6 programs reach 10,983 rows.
   - ALIX moves each arm to its first point **one at a time with the partner static**, then releases both replays
     together (`DAF:3763-3800`).
   - Arm-vs-arm verdicts therefore need a nominal result **and** a window result (R5).
4. **One FK.**
   - The Studio has no FK in the browser. Every pose comes from server-side `UrdfModel.link_world_transforms`
     (`UK:314-344`; `TG/web_editor/editor.js:12423-12437`).
   - Port the tree walk to JavaScript for playback and keep FCL on the server.
   - Add a parity test. The Studio itself warns that a second FK is how two views of the robot start to disagree
     (`TG/server.py:5296-5303`).
5. **A calibrated cell profile is a prerequisite.** It holds the toilet model and pose, walls, floor, whether the bin is
   fitted, and the bin position.
   - **[V]** With the default 800 mm venue, R2_LIDTOP (right arm, pole 150 mm, GRIP) touches a wall on 28 of 188
     sampled rows (R_Link5 28, R_Link6 6). The 1000 mm and 1400 mm venues give 0.
   - The deck photos show a back-to-wall "RALTO" bowl in a mock-up corner. ALIX's validated model is `commode_c`
     (`BC:58`).
6. **Seed pole, hand and bin explicitly.**
   - The ALIX defaults are bin LOWERED, poles 0.300 m and open hands (`DAPC:385-452`, lines 434-435).
   - `compute_plan_frames` passes no hand state (`DAPC:537`).
   - The sweep accepts only **SRDF-named** hand states and raises on unknown names (`DAPC:439-450`), so 0-1000 counts
     must enter as joint values. Use the `HU:30-60` law.
7. **Contact versus collision depends on the hand preset.**
   - **[V]** With the V6 GRIP fist `[0,0,0,0,150,1000]` on R2_LIDTOP, the **flexed finger links** touch the closed lid
     on 36 of 188 rows. Thumb links touch on 4 and `R_ConnectorLink` on 1.
   - On those rows the `R_glove_frame_2` origin is 19-35 mm from the lid (median 29 mm). A "pad frame within 20 mm"
     rule would call this wipe "in the air".
   - ALIX's contact list for `glove_frame_2` was derived for the `open_tenth` hand (`CL:81-84`; `GF:99-100`), not for
     GRIP.
8. **Collision engineering ([V] costs per row on the dev box).**

   | Step | Cost |
   |---|---|
   | FK | 0.29-0.32 ms |
   | Posing both full arms | 0.22 ms |
   | Full arm-vs-arm | 0.003 ms when far, 0.52 ms when close |
   | Full-mesh self check (`in_collision_internal`) | 87 ms per arm |
   | Slim self check | 4.5 ms per arm |
   | Curated lo/hi/distal self pairs (`BC:354-399`) | 0.71 ms per arm |

   - A full check of **every row** costs about 2-3.5 ms per row, so about 30 s for a 60 s pair.
   - FCL managers must not be shared across threads (`BC:150-171`), so run one worker process.

---

## 3. R3 — Simulation Playback

### 3.1 Original text
> "Simulation Playback: The edited trajectory should be played back in a simulation window to identify collisions, missed spots, or other issues. If any issues are detected, the user should be able to return to trajectory editing. -> Already exixting in ALix Studio - borrow"

### 3.2 Revised requirement (testable)
Lite shall replay any edited region trajectory in a 3D simulation window, on the ALIX robot model, before it can be sent
to the real arm.

- **(a) Inputs.**
  - The edited per-arm `.txt`: 7 integer millidegrees per line.
  - The region's pole height (mm) and hand counts (6 × 0-1000).
  - The partner arm's trajectory for the same task step. If the partner is not taught, it is held at a declared park
    pose (default: its last pose in the workflow) and the result is labelled partial.
  - The toilet state: closed, lid open, or lid and seat open.
  - The cell profile.
  - Lite refuses to simulate when the pole or the hand is unknown.
- **(b) Playback.**
  - Play/pause, step one row, scrub, and speed 0.25×-4×.
  - 1× follows the fitted player law: 0.2322 s + 5.6737 ms per row.
  - The 3D pose, every joint-plot cursor and the timeline always show the same row.
- **(c) Issues shown.**
  - Collisions and close calls (R5).
  - Joint-limit and rate warnings (R12).
  - A **wiped-area view**: toilet surface that the hand's contact links touched at least once is painted, untouched
    surface is not, and the contact point's path is drawn. Touching is judged from the contact links of the region's
    hand preset, never from a single pad frame.
- **(d) Return to editing.**
  - From the playhead or any issue, one action opens the C2 editor on the same region, arm and row window.
  - Any edit marks the result "out of date".
- **(e) Verdict.**
  - Each region shows Not simulated / No issues found / n issues.
  - It names what was not checked, for example "partner arm not taught", "toilet position not calibrated" or "bin not
    checked".
- **(f) Flow playback.**
  - Start lags, approach moves, pole moves, hand changes, waits and operator checkpoints appear with modelled durations.
  - These durations come from laws measured on the robot.

### 3.3 Clarifications
- **"Simulation window"** means an offline 3D replay on the ALIX robot model. It runs on Linux and Windows with no robot.
  It is neither the controller's SIM run mode (R4) nor the SDK emulator, which is for developer dry runs only.
- **"Edited trajectory"** means one region's per-arm `.txt` after C2 editing.
  - Both arms of the deck's task step are played together, because each step pairs a LEFT and a RIGHT unit.
  - A partner that is not taught is held at its park pose and the check is labelled partial.
- **"Missed spots"** means toilet surface the hand never touched.
  - Joint data cannot show pressure (`TRAJ:437`).
  - Version 1 paints the touched surface. A coverage percentage comes later, after the toilet pose is calibrated and the
    deck's regions have surfaces.
- **Touching is measured from the hand's contact links for the preset.** **[V]** With GRIP on R2_LIDTOP the finger links
  touch the lid while `R_glove_frame_2` is 19-35 mm away, so a pad-frame rule under-reports contact.
- **"Other issues"** means R12's joint-limit, rate and step findings, on the same timeline.
- **"Return to editing"** needs a generic row selection in C2.
  - The trajectory HTML's `addRegion` only creates a scale region, and only when the Scale stage is on
    (`TRAJ:1392-1396`).
- **1×** is the fitted law (`TRK:1558-1570`), not the nominal 5.5545 ms and not the 5 ms recording clock.
- **Scope of the borrow verdict.** It covers ALIX Studio code only (`TG/`). alix_workspace, alix_dispatch, tools/, the
  trajectory HTML and the control app are listed under other sources.

### 3.4 Findings

| # | Source | Finding | Evidence |
|---|---|---|---|
| 1 | ALIX Studio web_editor | Every pose goes through `applyRobotPose`, which takes server-computed 4×4 transforms per link. There is no browser FK. | `TG/web_editor/editor.js:12423-12437` |
| 2 | ALIX Studio server | `/api/robot-geometry` serves link-frame meshes from the baked overlay. The overlay has 200 sub-meshes, 100 links and 220,622 vertices **[V]**. Missing links such as `camera_link` fall through to the live path. The home pose is the initial positions plus BIN_LOWERED plus `full_length`. | `TG/server.py:1864-1998` (home 1986-1993) |
| 3 | ALIX Studio robot_model | The endpoint builds its context with `_ik_context`. That imports `alix_workspace.reachability` (a Workspace/C-space module, X2) for base placement and builds per-chain models. Lite needs a thin loader. | `RM:169-228` (import at 179) |
| 4 | ALIX Studio web_editor | There are three players and none has a speed control. DAC uses `setInterval` at max(20 ms, dt), which is 1× only when dt ≥ 0.02 s. Schedule & Run and the plan-file player run at 2× (`SCH_DT 0.1`, `SCH_SPEED 2`). The shared transport markup is used by DAC and Schedule only. | `TG/web_editor/editor.js:14561-14581` (14572), `12504-12506`, `8746-8767` |
| 5 | ALIX Studio server | Players accept only `.json` plans under the repo plan roots, as MoveIt-style `sub_trajectories` in radians. | `TG/server.py:4185-4196`; `DAPC:46-107` |
| 6 | alix_workspace | Every frame starts from bin LOWERED, poles 0.300 m and open hands. `compute_plan_frames` passes no hand state. Hand states are accepted only as SRDF names. | `DAPC:385-452` (434-435, 439-450), `537` |
| 7 | tools/dual_arm_verdicts.py | A production `.txt`-to-plan shim exists. It converts millidegrees × π/180000, uses the 0.0055545 s clock, says "seeding is MANDATORY" for pole and hand, and guards against swapped arms. It is coupled to ALIX MANIFEST task dicts, `sweep()` writes temporary plan files, and `track_points` does not check for 7 values per line. | `DAV:28`, `501-731` (build_timeline 591-618, sweep 709-733) |
| 8 | ALIX Studio web_editor/server | The toilet is drawn per lid/seat state, and the state comes from the task config's `commode_fixture_type`. The manifest builder enumerates ALIX region keys, paint meshes and generator constants, so it is coupled to the Regions tab that feeds X1. | `TG/web_editor/editor.js:1163-1179`; `VGC:219-330`; `TG/server.py:914-928, 4132-4139` |
| 9 | ALIX Studio / alix_workspace | Walls come from the venue YAML and are hidden by default. The default venue is 800 mm. `fixture_manager` can only shift the toilet along X by a standoff, not to a free (x, y, yaw). | `TG/web_editor/editor.js:9853-9874`; `BC:136, 401-460`; `PSV:62-67` |
| 10 | ALIX Studio server | `/api/sim/joints` replaces the whole stored map on each post, so both arms must be posted in one body. `/api/sim/pose` layers joints over initial positions only, not over the bin or pole state. The view polls at 2-10 Hz. | `TG/server.py:5296-5393` (5338, 5383-5393); `TG/web_editor/editor.js:19259-19296` |
| 11 | ALIX Studio coverage; trajectory HTML | Coverage scores **generator** cleaning points against painted region meshes, with a 35×80 mm footprint and a 20 mm band. The algorithm needs only a region mesh plus a pose sequence. The trajectory HTML says it cannot prove contact. | `TG/coverage.py:100-140`; `GF:80-100`; `TRAJ:437` |
| 12 | alix_workspace + Studio | Toilet-face marking exists and works in joint space. `fixture_contact_faces` returns fixture faces in contact (5 mm tolerance) and the client paints them. | `COL:366-487`; `TG/web_editor/editor.js:12008-12032`; `TG/server.py:928-955` |
| 13 | Trajectory HTML | Hooks exist for a plot cursor shape, rAF playback on a 5 ms clock and select-to-region (scale region only). Rows are numbered from 1. | `TRAJ:1134-1141, 1176-1231, 872-889, 1392-1396` |
| 14 | Control app runner v14; emulator; ALIX run records | The controller moves to each program's first point by itself and nothing checks that move. The runner only prints the gap. V6 has such moves of up to 64.6°. The emulator law (max Δq / (180 °/s × v%)) underestimates real moves 5-10× **[V]** (see §2.2). | `RUN14:205-214`; map 10 §3.2; `EMUDOC:73`; `RUNS/2026-09-02/run_slot76_210943.json` |
| 15 | tools/dual_arm_verdicts.py | The upload start lag is `0.351 s + 0.41 ms × N`, residual −1.07…+0.99 s, measured over 588-4,281 rows. | `DAV:176-224` |
| 16 | Measurement **[V]** | Venue: R2_LIDTOP, right arm with all 38 collision links, pole 150 mm, GRIP via HU, every 20th row (188 rows). The 800 mm venue gives **28** rows in contact (R_Link5 28, R_Link6 6). The 1000 mm and 1400 mm venues give 0. The draft said 35; the R_Link5 count matches. | in-memory, `PSV:117-155` |
| 17 | Measurement **[V]** | Toilet: same setup against `commode_c` closed at ALIX's pose. 36 of 188 rows touch the lid, all with flexed finger links (middle_2 24, ring_2 24, little_2 23, …). Thumb links touch on 4 rows and `R_ConnectorLink` on 1. An **open** hand gives thumb contact on 21 rows, which matches the draft's "thumb on 20", so the draft most likely left the hand open. | in-memory, `BC:401-464` |
| 18 | Measurement **[V]** | On those 36 rows the `R_glove_frame_2` origin is 19-35 mm from the lid (median 29 mm). Only 1 of 188 rows is within 20 mm. | in-memory, trimesh proximity |
| 19 | Measurement **[V]** | The arm base is at z = 0.6485 m at pole 0 and 0.7985 m at 150 mm. | in-memory FK |
| 20 | alix_dispatch | The player law and its band are known. The player does not smooth and does not rate-limit. | `TRK:9-11, 45-51, 1558-1570` |

### 3.5 Can it be borrowed from ALIX Studio?
**Verdict: Partial. Borrow some, build the rest.**

**Borrow:**
- the viewport shell;
- the robot-geometry payload and baked overlay, re-baked (R10) and served through a thin loader instead of `_ik_context`;
- `meshFromPayload` and `applyRobotPose`;
- the toilet and wall renderers, fed by a thin Lite manifest rather than `build_scene_manifest`;
- the transport markup;
- the Simulate live-feed endpoints;
- `paintCollisionFaces`, for the wiped-area view.

**Build:**
- the `.txt` input path;
- pole, hand and bin seeding;
- flow timing from measured laws;
- the speed control;
- browser FK with a parity test;
- the contact-link wiped-area view;
- the jump back to the editor;
- the per-region verdict;
- the cell profile.

**Input-claim check: partly accurate.**
- What exists: ALIX Studio has 3D joint-motion playback (DAC tab, Schedule & Run, plan-file player) and a live-feed view.
- Its limits:
  - It plays only MoveIt-style plan JSON under the repo plan roots (`TG/server.py:4185-4196`).
  - It runs at fixed speeds (1×/2×).
  - It starts from a lowered bin, 0.300 m poles and open hands (`DAPC:434-435`).
  - It takes the toilet state from task configs.
  - It has no missed-spot analysis for taught joint tracks (coverage.py scores generator configs).
  - It has no route back to a joint editor, because the Studio edits Cartesian configs.

| Other source | Verdict | Detail |
|---|---|---|
| `alix_workspace` (`UrdfModel`, `RobotCollisionModel` with an explicit `PackageIndex`, `fixture_contact_faces`) | Borrow with adaptation | Feed in-memory joint maps with explicit pole, hand and bin. Never rely on `_build_home` defaults (`DAPC:385-452`). `fixture_contact_faces` gives the touched toilet faces (`COL:430-487`). |
| `tools/dual_arm_verdicts.py` shim | Borrow (copy about 100 lines, adapt) | `track_points`, `hand_rad`, `build_timeline` (decoupled from MANIFEST tasks), `hold_timeline`, `timeline_side`/`assert_arm_sides` (`DAV:501-692`). Add the 7-integer check (`RUN14:76-88`). Stay in memory. |
| `hand_units.py` | Borrow as-is | 0-1000 counts to radians plus the mimic chain (`HU:30-60`). Do not use `sim_bridge`'s travel table (1.60/0.70/1.10 rad, above the URDF limits 1.344/0.5236/1.246; `TG/sim_bridge.py:95-102`). |
| Trajectory HTML | Borrow with adaptation | Cursor shapes, rAF playback and `plotly_selected` (`TRAJ:1134-1231, 872-889`). Needs a generic selection. |
| `alix_dispatch` | Borrow formula; emulator optional | `expected_replay_s` and the band (`TRK:1558-1570`). The emulated program channel (`EMU`) is for developer dry runs. |
| ALIX hardware records and DAV timing laws | Borrow as data (seed only) | Approach-move durations (`RUNS/*/run_slot*.json`: `gap_deg`, `speed`, `movej_s`) and start lag (`DAV:176-224`). Re-fit on Lite runs. |
| Control app UI v53 + runner v14 | Reference (high level) | Workflow JSON semantics (pole, hand, program, wait). The pole law (`POLE:11-14`). `transit_speed` affects only the approach move (`RUN14:1681-1684`). |
| `alix_taskgen/coverage.py` | Borrow with adaptation (phase 2) | `compute_coverage` needs only a region mesh and a pose sequence (`TG/coverage.py:100-140`), so FK poses of the contact patch can feed it once Lite has region surfaces. |

### 3.6 Borrow and build lists

| Borrow | From | Adaptation |
|---|---|---|
| Viewport shell (camera, lights, OrbitControls, on-demand render, ViewCube, nav islands, fit camera) | `TG/web_editor/editor.js:309-645, 1182-1197, 4500-4530, 7002-7060` | Remove painting, gizmos, TransformControls and Workspace overlays. Use Lite theme tokens. Vendor three.js r128 and OrbitControls. |
| Robot-geometry payload and baked overlay | `TG/server.py:1864-1998`; `WS/workspace_data/visual_overlay_baked.npz` | Replace `_ik_context` (`RM:169-228`) with a thin loader (UrdfModel plus `RobotCollisionModel(tag="visual", index=PackageIndex(bundle))`). Serve a re-baked overlay from Lite's bundle. Build the home pose from the region pole and the cell's bin setting. |
| Pose applier and mesh builder | `TG/web_editor/editor.js:667-703, 12352-12437` | Drive from browser FK. Keep `userData.baseColor` for highlight restore. |
| Toilet and wall renderers | `TG/web_editor/editor.js:743-830, 1046-1059, 1163-1172, 9853-9874` | Feed from a thin Lite manifest: per-part meshes per state through `scene_pose` (`load_component_mesh`, `resolve_component_pose`) plus the venue walls, posed by the cell profile. Do not reuse `build_scene_manifest` (`VGC:219-330`). |
| Transport markup | `TG/web_editor/editor.js:8746-8767` | Add a speed selector, step ±1 row and "Stop at problems". Label in plain words. |
| Live joint feed (R4/R5 tests) | `TG/server.py:5296-5393` | Post both arms in one body (the store is replaced, line 5338). Layer joints over the cell's pole and bin. |
| Toilet-face painting | `TG/web_editor/editor.js:12008-12032`; `COL:430-487` | Accumulate touched faces over a run for the wiped-area view, in neutral colours. R5 reuses it in the danger colour. |
| `.txt`-to-timeline shim | `DAV:501-692` | Copy into Lite, keep it in memory, and take the clock, pole and hand from the region settings. |

| Build | Approach |
|---|---|
| Track adapter | Parse NDJSON and enforce 7 integers per line (`RUN14:76-88`). Convert millidegrees to radians on `{L,R}_joint1..7`. Add constant columns for `{S}_sliding_plate_joint` = pole_mm/1000 and the six hand joints from HU. Keep per-arm arrays keyed by row. |
| Flow model | Turn the workflow JSON into per-arm events, each with a duration from its own law: start lag (`DAV:176-224` law, re-fitted); approach move (joint-linear path, duration from a law fitted on Lite runs and seeded from ALIX records, never the emulator law); replay (TRK law plus band); settle (`RUN14:1947-1949`); pole (`lift_travel_s`); hand; waits and checkpoints, grouped as v53 stages. |
| Browser FK | The server sends the kinematic tree (joints, origins, axes, mimics) and the home seed (initial positions, bin, pole, hand) once. A JS `linkWorldTransforms` mirrors `UK:294-344`, including "unlisted joints = 0" and the mimic fixed point. Add a parity test. |
| Player | One `currentRow` store and an rAF loop that advances by speed × elapsed / clock. The 3D view, plot cursors, timeline and readouts subscribe to it. Under Advanced, a partner time offset (± s) replays the drift cases R5 reports. |
| Wiped-area view | A server job tests the preset's contact links against each toilet part per row (or every 2nd row). It records touched faces (`fixture_contact_faces` pattern) and the closest contact point, and caches the result by content hash. The client paints touched faces and draws the contact path, solid while touching and dotted in the air. |
| Back to editing and staleness | "Edit here" opens C2 with rows [r − 0.5 s, r + 0.5 s] selected; C2 must offer a generic selection. A content hash invalidates the verdict (Studio pattern `markDirty` → `invalidateValidation`, `TG/web_editor/editor.js:5371-5389`). |
| Verdict chip | Not simulated / No issues found / n issues, plus "not checked" reasons. |
| Cell profile | A JSON file holding the toilet model, the toilet pose (x, y, z, yaw) relative to `base_link`, the venue walls (mesh and pose), the floor height, whether the bin is fitted, and its position. Lite applies the full pose itself, because `fixture_manager` only shifts along X. Validate by replaying known-good taught files: no structural-link contacts, and contact on the known wiping segments. |
| Park pose | An untaught partner holds its last pose in the workflow, or a declared park pose. The verdict says "partial". |

### 3.7 Recommendation
Add a **Simulate step** to every region in Lite's own backend and UI, not as a Studio tab.
- Lift the Studio's viewer, mesh payload, scene renderers, transport and face painting.
- Feed them from an in-memory `.txt` adapter that always seeds pole, hand and bin.
- Do FK in the browser for row-accurate scrubbing, with a parity test against Python FK. Keep FCL on the server (R5).
- Show "missed spots" as a painted **wiped area**, computed from the hand's contact links for the region's preset. Do not
  use the `glove_frame_2` origin: **[V]** it sits 19-35 mm off the lid while the GRIP fist is touching it.
- Make the cell profile a prerequisite. Label every verdict "not calibrated" until the profile is validated.
- Time whole-flow playback with laws measured on the robot. Do not use the emulator's movej law: **[V]** it
  underestimates ALIX's real approach moves 5-10×.

### 3.8 Approach
1. **Asset bundle.** Include the URDF, the re-baked visual `.npz`, collision STLs, SRDF, `initial_positions.yaml` files,
   the toilet model and scene, and the venue YAML and STL.
   - Load them with an explicit `PackageIndex` and set `ALIX_SRDF`.
   - Set `ALIX_REPO_ROOT`, or copy `load_wall_manager`, because `plan_scene_validation` raises at import without a repo
     root (`PSV:31-59`).
2. **Cell-profile schema and on-site procedure.**
   - Touch the pad to 3-4 known toilet landmarks, read FK, and fit the pose.
   - Validate by replaying 3-5 known-good taught files.
3. **`/api/lite/sim/timeline`.**
   - Input: region id(s).
   - Output: per-arm joint arrays in radians; pole, hand and bin per event; events with durations and the law each used;
     the row-to-seconds mapping.
   - Unit-test on V6 files.
4. **Browser FK.** Port `linkWorldTransforms` to JS. Serve the tree and the home seed. Add a parity test over 200 random
   V6 rows at 1e-6.
5. **Viewer.** Lift the viewport shell, the geometry payload (thin loader), the pose applier and the toilet and wall
   renderers (thin manifest).
6. **Player.** Implement play, speeds, step, scrub and keyboard control. Subscribe the C2 plots through a cursor shape
   (`TRAJ:1134-1141` pattern).
7. **Wiped-area view.** Build the job, painting and legend. Cache results and run them in the background.
8. **Verdict and editor link.** Add the verdict chip and "Edit here" / "Back to editor". Wire file-hash staleness and the
   automatic quick re-check (R5).
9. **Whole-flow playback.** Drive it from the workflow JSON using the measured laws. Fit the approach-move law and the
   start-lag law from Lite's first real runs.
10. **Later.** Add coverage % per region with `coverage.compute_coverage` fed by contact-patch poses
    (`TG/coverage.py:100-140`).

### 3.9 Acceptance criteria
1. `V6/RIGHT/R2_LIDTOP.txt` (3,751 rows), with pole 150 mm and hand `[0,0,0,0,150,1000]`, plays with no plan JSON on disk.
   A file with a non-integer or 6-value line is refused, and the message names the line.
2. Over 200 random V6 rows, including pole and hand joints, browser link transforms equal `UrdfModel.link_world_transforms`
   within 1e-6 m and 1e-6 rad.
3. The arm base is drawn at z = 0.6485 m + pole (0.7985 m at 150 mm) **[V]**. Finger joints equal `hw_to_rad` of the
   counts (GRIP: fingers 1.344 rad, thumb_2 0.445 rad, thumb_1 0 rad).
4. At 1× a 3,751-row file plays in **21.5 s ± 1 %** wall time (0.2322 + 5.6737 ms × 3,750); at 2× it takes half that.
   - A step moves exactly one row.
   - The plot cursor row equals the 3D row at all times, using one numbering convention on screen.
5. A task step with both arms taught plays both arms on one clock from their modelled start offsets. With one arm
   untaught, the partner holds its park pose and the verdict reads "partial: partner arm not taught".
6. The toilet appears in the step's state; for commode_c, open is −110° about the hinge (`CCS:48-68`). Walls and floor come
   from the cell profile, never from ALIX task configs.
7. **Wiped-area regression.**
   - Setup: commode_c at ALIX's pose, R2_LIDTOP with GRIP.
   - Lid-top faces are painted, but only from rows where finger links intersect the lid (**[V]** 36 of 188 sampled rows).
   - With the toilet moved 100 mm away, nothing is painted.
   - The glove-frame-distance rule is not used.
8. A 10,983-row file plays at 30 fps or more on the dev box with the wiped-area layer on.
9. "Edit here" at row r opens the same file in the editor with rows [r − 0.5 s, r + 0.5 s] selected. After any edit the
   region shows "Simulation out of date".
10. In a multi-program flow, start lags, approach moves, pole moves and hand changes are drawn with their durations.
    - Each is labelled "modelled", with the source of its law (measured on this robot, or ALIX records).
11. The first frame appears within 5 s of opening a region once robot geometry is cached.

### 3.10 UX notes (non-technical operators)
- Give each region one large **"Check in simulation"** button. Report the result as a sentence, for example:
  - "No problems found (toilet position not set up — not checked)";
  - "1 problem: right arm touches the toilet lid at 0:12".
- Space plays and pauses; ←/→ step one row. Label the speed "Slow / Normal / Fast", with the × value shown small.
- Keep the camera still. Offer "Home view" and "Follow left/right hand"; never auto-orbit.
- Hide dt, link sets and frame names behind "Advanced". Use deck region names, never P-numbers or program IDs.
- Put "Back to editor" next to every issue and on the playhead, and return to exactly the same place.
- Wiped-area legend, in words: "Painted = the hand touched this surface; unpainted = not wiped, check it."
  - Use a neutral paint plus a hatch for unwiped areas, not red/green.

### 3.11 Safety notes
- A clean simulation means "nothing found in the model", not "safe". The verdict lists what was not modelled.
- The first real run of an edited file happens at reduced speed with the stop ready (`V6/README.txt:48-56`).
  - The run speed % scales only the approach move (`TRK:20-24`; `RUN14:1681-1684`).
  - So "reduced speed" means uploading a **slowed (resampled) copy** of the file.
- Never accept the defaults of 0.300 m poles, open hands and a LOWERED bin (`DAPC:385-452`). Refuse to simulate when pole
  or hand is unknown.
- Approach moves (up to 64.6° in V6) are not checked by the controller.
  - Show them and check them (R5).
  - Before a real run, check the approach from the arm's live pose.
- Without a validated cell profile, toilet and wall verdicts can be wrong in both directions (**[V]** the 800 mm venue
  gives 28 false wall rows).
- Two-arm playback is nominal. Upload-channel arms can drift by seconds (§2.3), so a clean two-arm playback does not prove
  the arms stay apart.

### 3.12 Dependencies, risks, open questions
**Dependencies**
- Lite server scaffold and asset bundle (architecture).
- R1: task-step pairing, lid/seat state, park pose.
- R2/C2: editor, plots, generic row selection.
- R7: pole and hand per region; the hand preset decides the contact links.
- R5: collision engine.
- R10: re-baked model and bin setting.
- R8: flow composition.
- C4/R4: real-run data for the approach-move and start-lag laws.
- On-site cell calibration.
- The requester's ruling on which hand links may touch the toilet.

**Risks**
- An uncalibrated toilet pose or venue gives false verdicts.
- The deck's toilet is a back-to-wall "RALTO" bowl in a mock-up corner, while ALIX's validated model is commode_c (`BC:58`).
- The two FK implementations drift apart unless the parity test runs in CI.
- Server frames would cost about 16 KB each (about 19.5 MB for a 61 s program at 0.05 s) if browser FK is dropped.
- The physical bin is about 40 mm taller than the model (`BINX:32-34`).
- Lid/seat motion in mechanism tasks is not modelled. The Studio's hinge-angle method is config-coupled
  (`TG/server.py:4518-4530`), so it is reference only.
- A wrong contact-link set or toilet pose creates false "missed spots".
- The approach-move law at Lite's 30 % transit speed is unmeasured.
- Private alix_workspace helpers may change; pin the ALIX commit.

**Open questions**
1. "Missed spots": surface never visited, or too little pressure? Pressure cannot be seen in joint data.
2. Which hand links may touch the toilet with GRIP? **[V]** shows the flexed finger links on the lid.
3. Is the toilet the commode_c model, and where does it stand relative to the robot base?
4. Is region-level playback enough for v1, or is whole-flow playback needed?
5. Mechanism tasks: should the lid/seat be animated, or switched at the end?
6. Which venue (cubicle width) and floor height apply?
7. Where should an untaught partner arm park?

**Priority: Must. Effort: L, about 13-15 person-days.**

| Item | Days |
|---|---|
| Viewport and scene port, with thin loader and manifest | 3.5 |
| Adapter and flow model with measured laws | 2.5 |
| Browser FK and parity test | 2 |
| Player and plot sync | 2 |
| Wiped-area job and painting | 2 |
| Editor link and staleness | 1 |
| Verdict and tests | 1.5 |

This assumes the scaffold and asset bundle already exist. On-site calibration is extra.

---

## 4. R5 — Collision Detection and Graph Highlighting

### 4.1 Original text
> "Collision Detection and Graph Highlighting: If a collision occurs during real-arm testing or simulation view , the graph should stop at the corresponding joint value and highlight the collision point for easy identification and tuning. - Borrow from ALix Studio"

### 4.2 Revised requirement (testable)
- **(a) Simulation check.** Every simulated playback is checked for contact between:
  - the left and right arms, including hands, fingers and the right wrist camera;
  - each arm and itself;
  - each arm and the toilet (body, seat and lid, in the step's state);
  - the cubicle walls;
  - the floor;
  - the bin, when fitted.

  Two check levels:
  - The **full check** examines every row. It is the gate before any real test.
  - The **quick check** examines every 4th row and is labelled "quick".

  Classification:
  - The region's contact links for its hand preset touching the toilet is **pad contact (expected)**.
  - Any arm structural link touching anything is a **collision**.
- **(b) Two-arm timing.** Arm-vs-arm results are given twice:
  - at the nominal alignment from the flow model;
  - over a drift window built from the measured start-lag and player-clock spread.

  A contact that can occur only inside the window shows as "possible if the arms drift".
- **(c) Stop and highlight.** With "Stop at problems" on (the default), playback stops at the first colliding row.
  - Each affected arm's joint plot shows a vertical marker at that row.
  - Each of the 7 joint curves gets a marker labelled with its value in degrees.
  - A shaded band covers the colliding rows.
  - The 3D view shows the colliding parts in the danger colour (robot links, and toilet or wall faces) and names the
    pair in plain words.
- **(d) List.** A list of all events shows, for each:
  - start/end time and duration;
  - first/last row per arm;
  - the parts, in plain and technical names;
  - the kind: arm-arm, self, toilet, wall, floor or bin.

  Previous/Next and click-to-jump work from the list.
- **(e) Moves around programs.** These are checked too:
  - the controller's approach move to each program's first point (joint-linear path);
  - pole moves;
  - hand changes.

  Before every real run, the approach from the arm's live pose is checked, and a predicted collision blocks the run.
- **(f) Real arm.** When the controller reports collision 4109, Lite:
  - takes the joints at the stop from a 5 ms joint capture of the run (fallback: joints printed by the runner with the
    code);
  - maps them to a row of the uploaded file with an ordered matcher, or reports "during the approach move";
  - shows the same markers;
  - shows a 3D comparison of the actual pose against the planned one.
- **(g) Fix in editor.** Every event offers "Fix in editor" (R3 d).

### 4.3 Clarifications
- **"The graph"** means Lite's per-arm joint-value plots (C2/TRAJ), not the Studio's 0/1 bar chart.
- **"Stop at the corresponding joint value"** means the cursor halts at the first colliding row, and each joint curve
  gets a marker with its value there.
- **"Collision" excludes intended contact by the preset's contact links.**
  - ALIX's runtime allows only the active hand's listed links against the contact surface (`BC:31-39`).
  - The list for `glove_frame_2` was derived for the `open_tenth` hand (`CL:81-84`; `GF:99-100`), so Lite needs a list
    per hand preset.
- **Real-arm collisions** are what the controller reports: error 4109 "ARM COLLISION", from its dynamics-based detection.
  - It can fire without contact.
  - On the last probe it was off on the right arm.
- **Rows are the shared key.** Seconds are for display only.
- **Two upload-channel arms are not row-locked** (§2.3), so arm-vs-arm results need a window.
- **Scope of the borrow verdict.** It covers ALIX Studio code only.

### 4.4 Findings

| # | Source | Finding | Evidence |
|---|---|---|---|
| 1 | alix_workspace | `compute_dual_arm_plan_collisions` checks L vs R, self per arm, arms vs `bin_link` + glove lever, plus any `extra_managers`. It returns collisions `{index, time, pairs}`, `joint_timeline` and frames. It gives names only (no depth) and samples discretely. | `DAPC:586-764` (644-646, 699-703, 755) |
| 2 | alix_workspace | `extra_managers` pairs keep only the dict key, and they are tested only against the **slim** arm sets. So fingers never meet the toilet, even when a toilet manager is passed. | `DAPC:645-646, 699-703` |
| 3 | alix_workspace | Hand states are accepted only as SRDF names; unknown names raise. Counts must go in as joint values. | `DAPC:595-598, 439-450` |
| 4 | ALIX Studio server | The DAC endpoint passes no `extra_managers`, no hand state and the default body set, and pops `joint_timeline`. | `TG/server.py:4335-4366` (4355-4357) |
| 5 | alix_workspace | The Studio link set is the arm links plus the palms (13 per arm). There are no fingers. | `DAPC:165-182` |
| 6 | ALIX Studio web_editor | The DAC UI has a 0/1 bar chart in hard-coded dark colours, click-to-scrub, red recolouring of robot links only, a "COLLIDING" label, Previous/Next and a log. | `TG/web_editor/editor.js:14478-14498, 14501-14546, 14548-14559, 14272-14298` |
| 7 | ALIX Studio web_editor/template | Playback never stops at a collision, and no joint values are plotted anywhere. | `TG/web_editor/editor.js:14561-14581`; `TG/web_editor/template.html:601` |
| 8 | ALIX Studio body_collision; plan_scene_validation | The building blocks for the toilet and walls exist. **`fixture_manager`** needs the world built first (**[V]** `AttributeError '_fixtures'` otherwise), uses `scene_pose` with the washroom rotation, holds one manager with `commode_body/seat/lid`, and only shifts along X. **`load_wall_manager`** supports a mesh override. **`PlanWallChecker`** samples coarsely at 0.05 s, refines at 0.01 s and gives a depth. | `BC:401-464`; `PSV:117-155, 179-270` |
| 9 | alix_workspace + Studio | Toilet-face marking and painting exist. | `COL:430-487`; `TG/web_editor/editor.js:12008-12032` |
| 10 | tools/dual_arm_verdicts.py | dt is a correctness parameter: at dt 0.05 a link moves 109 mm against clearances of 35-80 mm. ALIX sweeps at 0.02 s with a 25 mm floor. | `DAV:352-369` |
| 11 | SRDF | Two cross-arm pairs are disabled: left thumb vs right camera. | `SRDF:1442, 1452` |
| 12 | Measurement **[V]** | **Thumb vs left wrist.** Pairing: L_P03P05 row 3424+i at pole 50 mm with R2_LIDTOP row i at 150 mm, GRIP on both hands. Full link sets find 41 rows (R2_LIDTOP rows 2788-2828) where `IRH_right_thumb_4/tip` ↔ `L_CameraHolderLink` (41 rows) and `thumb_3` ↔ `L_Link7` (10 rows). The Studio slim set finds 0. The draft's "thumb vs L_Link7, rows 2800-2824" is substantially confirmed; the main pair is `L_CameraHolderLink`. | in-memory, `_ArmCollision` (`DAPC:126-160`) |
| 13 | Measurement **[V]** | Costs per row are as in §2.8. Curated self pairs cost 0.71 ms per arm, full-mesh self 87 ms per arm. | in-memory; `BC:354-399` |
| 14 | alix_tasks config | The pad allowance for `glove_frame_2` is the palm, palm rubber and 25 finger links, derived for `open_tenth`. The pad is 35×80×20 mm. | `CL:81-110`; `GF:82-100` |
| 15 | Measurement **[V]** | With GRIP on R2_LIDTOP against the closed lid, finger links make the contact on 36/188 rows, and all of them are in the `glove_frame_2` list. Thumb links touch on 4 rows. `R_ConnectorLink` (structural) touches on 1 row, which is a collision. | in-memory |
| 16 | alix_workspace redundancy | `classify_pair` checks fixture tokens first, so `lid_opening_cover1` (a bin part) reads as toilet ("lid"). | `RED:90-105` |
| 17 | Control app runner v14 | The runner knows 4109 and 4105. It polls every 0.5 s and prints each code once with seconds since start, discarding the joints from the same read. After the program ends it reads the codes once more and reports them **without a time**. It never aborts a program. | `RUN14:56-57, 122-141, 244-252, 263-268, 1925-1936, 1961-1967` |
| 18 | Hardware probe doc | On 2026-08-13 the left arm's collision level was 3 and the right arm's 0 (off). Self-collision and e-fence work only in simulation mode. | `AVI:3, 27-33` |
| 19 | ALIX motion docs | 4109 can fire without contact. In a 3× acceleration CANFD run it fired with the arm lagging 8-16°, no impact on the wrist force sensor, and the stop latched. | `CANFD:675-682` |
| 20 | ALIX hardware records **[V]** | **`plan_num` does not index points.** A replay of 2,309 points reports `run_state 1, plan_num 1` for its whole length, then idle. The emulator copies this shape. | `RUNS/2026-09-01/run_slot70_190758.json` (transitions); `EMU:46-49` |
| 21 | alix_dispatch | Real-arm capture tools exist. **`UdpRecorder`** records the 5 ms push (joints, error codes, status) during an off-driver run, refuses to take a push owned by another host, and restores the previous configuration. **`push_error_codes`** gives each code's first and last time. **`verify_replay_from_frames`** maps executed frames to commanded rows with an ordered matcher that avoids the "nearby stripe" trap. **`MotionMonitor`** detects cuts in about 15-20 ms. ALIX tools use these in production. | `TRK:974-1080, 1653-1688, 2037-2075, 2355-2380`; `DAF`; `/home/newtonjeri/alix_ws/src/alix/tools/cleaning_tracks.py` |
| 22 | tools/dual_arm_verdicts.py **[V]** | Upload timing envelopes: start lag `0.351 + 0.41 ms × N` (−1.07…+0.99 s); clock envelope [0.057 + 5.515 ms(N−1), 0.750 + 5.810 ms(N−1)], measured over 588-4,281 rows. The exact window for two 3,751-row tracks is ±3.56 s. ALIX refuses longer tracks. | `DAV:150-224, 862-880, 912-973` |
| 23 | tools/dual_arm_flow.py | ALIX bridges each arm to its first point one at a time with the partner static, then releases both together. | `DAF:3763-3800` |
| 24 | ALIX generation findings | Regression reference: an ALIX plan pair touches at about 27.3 s and 29.7 s (`L_Link3↔R_Link4`). The same plans reportedly ran 13/13 on hardware, so model verdicts can disagree with hardware. | `GENF:3050-3051, 3107, 2782` |
| 25 | RealMan SDK | `rm_get_collision_stage` / `rm_set_collision_state` (levels 0-8). | `SDK/rm_robot_interface.py:585-618` |

### 4.5 Can it be borrowed from ALIX Studio?
**Verdict: Partial. Borrow some, build the rest.**

**Borrow:**
- the red-link highlight with colour restore (`dacHighlight`);
- the collision log, Previous/Next and click-to-jump;
- click-to-scrub;
- toilet-face painting (`paintCollisionFaces`);
- the toilet collision manager at an articulation (`fixture_manager` + `scene_pose`, copied and extended with a full cell
  pose);
- the content-hash result cache.

**Build:**
- joint plots with markers and bands;
- stop at the first collision;
- the full world: toilet per part, walls, floor, fingers, configurable bin;
- preset-aware contact classification;
- every-row checking;
- the drift window;
- approach, pole and hand-move checks;
- the whole real-arm path.

**Input-claim check: partly accurate.**
- What exists: the Studio's Dual-Arm Collisions tab detects arm-vs-arm, self and bin contacts and shows them as red links
  with a 0/1 timeline, a log and Previous/Next (`TG/web_editor/editor.js:14272-14581`).
- Its limits:
  - It does not plot joint values.
  - It does not stop at a collision (`TG/web_editor/editor.js:14561-14581`; `TG/web_editor/template.html:601`).
  - It does not check the toilet, walls or fingers (`TG/server.py:4355`; `DAPC:165-182`).
  - It assumes both arms start together at t = 0 with no drift.
  - It has no real-arm input, because the Studio has no execution channel.

| Other source | Verdict | Detail |
|---|---|---|
| `alix_workspace` `_ArmCollision` / sweep loop | Borrow with adaptation | Re-implement the ~60-line loop in memory (`DAPC:651-735`) with full hand sets, named toilet parts, hand joints from counts and explicit pole and bin. Keep the joint timeline. Pin the ALIX commit. |
| `cross_arm_separation` + `DAV.clock_envelope_window_s` | Borrow with adaptation | Minimum arm-arm distance over an offset window (`DAPC:1089-1283`), using per-request managers (the cache is single-thread only, `DAPC:1015-1025`). Take the window from `DAV:912-973` (copy it with `_start_lag_band`/`_clock_band`/`_phase_intervals`). Re-fit on Lite runs. 25 mm floor (`DAV:369`). |
| `plan_scene_validation` | Borrow with adaptation | `load_wall_manager` (mesh override) and coarse-then-refine with depth (`PSV:117-155, 219-256`). Report depth as approximate. |
| `collision.fixture_contact_faces` | Borrow as-is | Touched toilet faces for the highlight (`COL:430-487`). |
| `redundancy.classify_pair` | Reference (copy about 20 lines and fix) | Check bin tokens before fixture tokens, or use exact names (`RED:90-105`). |
| `body_collision._self_managers` | Borrow with adaptation | Curated lo/hi/distal self pairs, **[V]** 0.71 ms per arm (`BC:354-399`). |
| `contact_links.yaml`, `glove_frames.yaml` | Borrow as data (re-derive per preset) | `/home/newtonjeri/alix_ws/src/alix/tools/regen_contact_links.py` derives lists from the pad plane. GRIP needs a ruling or a re-derivation. |
| `alix_dispatch.track` capture and matcher | Borrow with adaptation (preferred real-arm path) | `UdpRecorder`, `push_error_codes`, `verify_replay_from_frames` and `MotionMonitor` (`TRK:974-1080, 1653-1688, 2037-2075, 2355-2380`). They must run in the process that owns the arm connection. |
| `tools/dual_arm_flow.py` `_bridge_to_start` | Reference (C4) | Serialised approach moves with the partner static (`DAF:3763-3800`). |
| Trajectory HTML | Borrow with adaptation | Cursor shapes extended with a band, 7 markers and an annotation (`TRAJ:1134-1141`). |
| Control app runner v14 | Borrow with adaptation (small change, fallback path) | Print the joints it already reads (`RUN14:122-141`) with each code in `run()`, in the workflow monitor and in the post-run check (`RUN14:244-268, 1925-1967`). |
| RealMan SDK | Operational | Read the collision stage on both arms before tests (`SDK/rm_robot_interface.py:585-618`). |

### 4.6 Borrow and build lists

| Borrow | From | Adaptation |
|---|---|---|
| Colliding-link highlight with colour restore | `TG/web_editor/editor.js:14531-14546` | Take the colour from `--danger`. Also paint toilet and wall faces. |
| Collision log, Previous/Next, click-to-jump | `TG/web_editor/editor.js:14272-14298, 14548-14559` | Plain-language rows, kind chips, per-arm rows. |
| Toilet-face painting | `TG/web_editor/editor.js:12008-12032`; `COL:430-487` | Danger colour for collisions. |
| Toilet manager at an articulation | `BC:401-464` with `TG/scene_pose.py` | Copy it. Keep part names (one manager per part, or Lite's own loop). Apply the cell profile's full pose. |
| Walls with coarse-to-refine and depth | `PSV:117-155, 219-256` | Venue from the cell profile. Depth is approximate. |
| Curated self pairs | `BC:354-399` | lo/hi/distal sub-managers per arm. |
| Content-hash cache | `TG/server.py:4303-4316` | Lite cache folder. Key on file hashes plus pole, hand, bin, cell profile and mode. |
| Hand envelope boxes (quick mode) | `DAPC:201, 275-327, 832-844` | Fit the boxes over the region's actual hand pose(s) with `hand_envelope_over`. |
| Drift-window model | `DAV:862-973` | Copy it. Re-fit the bands on Lite runs. |
| Real-arm capture and matching | `TRK:974-1080, 1653-1688, 2037-2075` | Run it inside Lite's robot I/O process (or the runner). Keep the refusal to steal a push. |

| Build | Approach |
|---|---|
| Collision world | Managers: left arm + hand; right arm + hand + `camera_link`; named toilet parts; walls; floor plane; bin set when fitted. Curated self pairs. SRDF ACM, with `SRDF:1442, 1452` re-enabled. |
| Sweep worker | One worker process. Full mode checks every row. Quick mode checks every 4th row and refines every row around hits. Cache results. Events are keyed by row. |
| Two-arm timing | Nominal offsets from the flow model. A drift window from the DAV model. Minimum distance over the window with the sphere prefilter. An amber state for contacts inside the window. Runs as a background job. |
| Event model | `{kind, parts (plain + technical), first/last row per arm, t, approx. depth, isPadContact, nominal / window}` |
| Contact classifier | A per-preset contact-link list decides pad contact vs collision. Warn when depth exceeds `PAD_COMPLIANCE_MM` = 20 mm (`BC:65`). |
| Graph behaviour | Per-arm Plotly plot. Stop at the first event, then add the line and band shapes, 7 markers and an annotation, styled by `plotTheme()` from tokens. |
| Approach, pole and hand checks | Approach: a joint-linear path from the previous program's last row (offline) and from the live pose before each real run (the runner already reads it, `RUN14:205-214`). Pole moves: swept with the arm fixed, which covers the bin interlock. Flag pole or hand changes made while a hand link is within 20 mm of the toilet. |
| Real-arm path | Preferred: a 5 ms capture of every real test. Take the first 4109 frame and its joints, map them with the ordered matcher, and draw a planned-pose ghost. Fallback: the runner prints joints with each code, and Lite searches for the nearest row within a time prior. When several rows match (repeated strokes), show the ambiguity. |
| Pre-test check | Read both arms' collision stage. Block, or warn loudly, if the levels differ or one is 0. Changing the level is an engineer action with confirmation. |

### 4.7 Recommendation
Build Lite's own collision module on alix_workspace, run in one worker process.
- **Hands and self checks.** Use full hand geometry for arm-vs-arm and arm-vs-environment checks, and curated self pairs.
  **[V]** The Studio's palms-only set misses a real right-thumb-to-left-wrist contact on V6 data, and a full check of
  every row costs only about 2-3.5 ms per row.
- **World.** Add the toilet per part, the walls, the floor and the bin from a calibrated cell profile. Classify contact
  by the hand preset's contact links.
- **Two-arm timing.** Two upload-channel arms can drift by seconds (**[V]** ±3.56 s for two 3,751-row tracks), so:
  - report arm-vs-arm results both at the nominal alignment and over the drift window;
  - when contact is possible in the window, advise running the pair one after the other;
  - adopt ALIX's serialised bridge-to-start.
- **Graph.** Reuse the Studio's 3D highlight, log and face painting. Build the joint markers and stop-at-first-collision
  on the C2 plots, keyed by row.
- **Real tests.** Record the controller's 5 ms push (alix_dispatch `UdpRecorder`) and map a 4109 stop to a row with
  ALIX's ordered matcher. The runner printing joints with each code is the minimal fallback.
- **Before any real test,** align both arms' collision-detection levels.

### 4.8 Approach
1. **`lite/collision` world builder.** Inputs: cell profile, region pole and hand, bin setting. Add curated self pairs, an
   ACM loader with `SRDF:1442/1452` re-enabled, and per-part toilet managers.
2. **Sweep worker.** Full mode (every row) and quick mode, with caching and row-keyed events.
3. **Contact classifier.** One list per preset. Start from `CL` `glove_frame_2` plus the **[V]** GRIP evidence and ask
   the requester to confirm. Treat depth as approximate.
4. **Drift-window check.** Copy the DAV model and seed it with ALIX's bands. Re-fit after about 10 Lite runs per
   program-length class. Run it in the background and show the amber state.
5. **Approach, pole and hand-move checks.** Offline, plus a live check before each run.
6. **Regression suite.**
   - A seeded collision confirmed by a brute-force every-row check.
   - The V6 P4 thumb-to-wrist case.
   - R2_LIDTOP: finger contact is pad contact, `R_ConnectorLink` is a collision.
   - No false wall hits on the calibrated profile.
   - The ALIX plan pair at about 27.3 s and 29.7 s (`GENF:3107`).
7. **Front end.** Event list, Previous/Next, stop-at-first-collision in the R3 player, markers, band and annotation on the
   C2 plots, red parts and faces in 3D.
8. **Real-arm path, with C4.** Run the 5 ms capture inside Lite's robot I/O. Map stops with `verify_replay_from_frames`.
   Draw the ghost pose and the operator guidance text. Fallback: the one-line runner change.
9. **Operations.** Add the pre-test collision-stage check. Document how to clear a latched 4109 at the pendant.

### 4.9 Acceptance criteria
1. The result header lists which checks ran: arm-arm nominal, arm-arm window, self, toilet [state], walls [venue], floor,
   bin [setting], approach moves. Any missing check is shown as "not checked".
2. **Seeded collision.** Take a copy of a V6 file, edited so that a brute-force every-row check finds a collision.
   - The full check reports exactly the same first row.
   - The quick check reports it within ±4 rows and is labelled "quick".
3. With "Stop at problems" on, playback halts at that row.
   - The 7 markers show millidegrees/1000 of that row, to 0.01°.
   - The band covers exactly the colliding rows.
   - The colliding links and toilet or wall faces are painted in the danger colour.
4. Every event shows its kind, plain-language parts, technical pair, first/last row per arm and time. Previous/Next
   visits them in order.
5. With the GRIP list (once ruled) on R2_LIDTOP against the closed lid:
   - finger-link contacts are "pad contact (expected)";
   - the `R_ConnectorLink` contact (**[V]** 1 of 188 sampled rows at ALIX's commode_c pose) is a collision.
6. **Regression [V].** On the designed-aligned V6 P4 pair, the full check reports `IRH_right_thumb_*` ↔
   `L_CameraHolderLink`/`L_Link7` around R2_LIDTOP rows 2788-2828.
   - The quick mode with hand boxes reports it, or says "fingers approximated".
   - The Studio slim set is never used as a pass.
7. **Drift window.** For two tracks released together, the window equals `clock_envelope_window_s` for the same bands
   (±3.56 s for 3,751 + 3,751 rows with ALIX's bands). Contacts found only inside the window show "possible if the arms
   drift".
8. **Approach moves.**
   - A flow whose approach path (joint-linear from the previous program's last row) hits the toilet is flagged before
     upload.
   - Before a real run, a live start pose whose approach would collide blocks the run with a plain message.
9. **Real-arm mapping.**
   - A recorded 5 ms capture with a 4109 frame maps the stop to a row through the ordered matcher.
   - Without a capture, the runner's "code + joints" line maps to the nearest row.
   - Joints more than 2° from every row give "during the approach move".
   - When two or more rows inside the time prior match within 1°, the ambiguity is shown.
10. **Performance.**
    - The full every-row check of a 60 s pair (about 11,000 rows per arm) completes in 60 s or less on the dev box, with
      progress.
    - The quick check takes 10 s or less.
    - Unchanged inputs return from cache in under 1 s.
    - The drift-window check runs in the background.
11. Without a calibrated cell profile every verdict carries "toilet/walls not calibrated", and "No problems found" never
    appears alone.
12. Plot text and grid use theme tokens (contrast at least 4.5:1 in the light theme). Collisions use `--danger` with a
    glyph and the word "Collision".

### 4.10 UX notes (non-technical operators)
- **Name parts in plain words:** "right wrist (R_Link6)", "left hand thumb", "toilet lid", "cubicle wall", "bin cover".
  Keep the technical pair under "Details".
- **Use three states, each with colour, glyph and word:**
  - Collision;
  - Close call / possible if the arms drift;
  - Clear.

  Show expected pad contact in neutral grey.
- **Drift wording:** "These two parts can touch if one arm runs a little ahead of the other. Run them one after the
  other, or change one of them."
- **Actions on every event:** "Show me", "Fix in editor" and "Re-check".
- **After a real-arm stop:** "The controller stopped the right arm (collision detected) at 0:12. Clear the alarm on the
  teach pendant, then press Re-check." Add: "This can be real contact or a motion that is too fast."
- **"Stop at problems"** is on by default, with a visible switch.
- **Hover on a marker** shows, for example: "J3 = 41.2° at 0:12.84 (row 2,312)".

### 4.11 Safety notes
- Never treat the Studio's palms-only check as a pass. **[V]** It misses a thumb-to-wrist contact on V6 data.
- Re-enable the SRDF-disabled pairs (`SRDF:1442, 1452`) in Lite's own collision list.
- A simulated "clear" depends on the cell profile. **[V]** The 800 mm venue gives 28 false wall rows.
- An arm-vs-arm "clear" at the nominal alignment is **not** "clear" on the robot, because upload-channel arms can drift by
  seconds. Use the window result, and run uncertain pairs serially.
- The right arm's collision detection was off (level 0) on the last probe (`AVI:27-33`).
  - Align both arms before real tests.
  - Changing the level is a controller-wide engineer action.
- 4109 can fire without contact on aggressive motion (`CANFD:675-682`). Do not tell the operator to "move away from an
  obstacle" when the simulation shows none; suggest slowing that part.
- The runner samples error codes every 0.5 s and does not abort a program; the controller's own reaction is the only stop.
  - Keep the E-stop within reach.
  - Run first tests from a slowed copy.
- Approach moves (up to 64.6° in V6) are unchecked by the controller. Check them, and in dual runs move one arm at a time
  with the partner static (`DAF:3763-3800`).
- The realtime push has one destination per controller. Keep `UdpRecorder`'s refusal to take a push owned by another host
  (`TRK:1028-1063`).
- Model verdicts can disagree with hardware in both directions (`GENF:2782`). Confirm on the robot at reduced speed.

### 4.12 Dependencies, risks, open questions
**Dependencies**
- R3: player, adapter, cell profile.
- C2: plots, editor.
- R7: pole, hand, preset contact lists.
- R10: bin setting, re-baked model.
- R4/C4: real-test path, with the capture in the process that owns the arm connection; runner change.
- Architecture: worker process, cache.
- The requester's contact-link ruling.

**Risks**
- An uncalibrated cell gives false or missed collisions.
- A wrong GRIP contact list mislabels pad contact (**[V]** the flexed fingers touch the lid; the thumb only on 4 rows).
- Private DAPC helpers may change; pin the commit.
- A second TCP client may be refused while the runner owns the arm, so the capture must live in the owning process.
- Drift windows of seconds may flag many TOGETHER pairs. That means more serial running and longer cleaning time.
- ALIX's envelope covers only 588-4,281 rows; longer tracks need Lite's own measurements.
- Quick mode can miss thin contacts; full mode checks every row.
- FCL mesh-mesh depth is approximate; do not gate on it.
- The `classify_pair` token order mislabels bin covers as toilet unless it is fixed.

**Open questions**
1. Which hand links may touch the toilet with GRIP?
2. What near-miss distance should warn? ALIX uses a 25 mm floor (`DAV:369`).
3. May Lite take the realtime push during tests (is nothing else consuming it on this robot), or should the runner print
   joints?
4. Which collision level should both arms use (left 3 / right 0 on 2026-08-13)?
5. Should the full check and the window check be required before every real test, or only before upload?
6. Should pairs marked "possible if the arms drift" run serially by default?
7. Settled: `plan_num` cannot index points (**[V]** `RUNS/2026-09-01/run_slot70_190758.json`).

**Priority: Must. Effort: XL, about 19-21 person-days.**

| Item | Days |
|---|---|
| World assembly | 3.5 |
| Sweep worker (every-row, quick, cache) | 3 |
| Contact classifier | 1 |
| Drift window | 2 |
| Approach, pole and hand checks | 1.5 |
| Plot markers, stop and list | 3 |
| Real-arm capture, matching and ghost (shared with C4) | 3 |
| Calibration validation harness | 1 |
| Tests | 1.5 |

Without the real-arm path this is L (about 16 days).

---

## 5. R10 — Dual-Arm Speed Optimization (split view, correct model, bin toggle)

### 5.1 Original text
> "Dual-Arm Speed Optimization: The UI should display dual-arm simulation on the left screen and trajectory plot analysis on the right screen to optimize movement speed and cleaning time. - Borrow from ALix Studio -> should use the correct urdf and the robot meshes, also add toggle for bin inclusion and exclusion"

### 5.2 Revised requirement (testable)
- **(a) Layout.** One Dual-Arm view: 3D simulation on the left, analysis plots on the right, split by a draggable and
  keyboard-operable splitter. Each pane can be maximised and restored. It works at 1366 × 768.
- **(b) Correct model.**
  - The 3D robot uses the committed ALIX flat URDF (`alix_workspace/urdf/alix.urdf`).
  - It uses a visual overlay re-baked from that URDF, so the visible bin and wrist camera match the collision model.
  - The view's FK equals the collision checker's FK (parity test).
- **(c) Bin control.**
  - "Bin fitted on robot: yes/no" shows or hides the bin and adds it to, or removes it from, every check. Removing it
    needs a confirmation and prints "Bin not checked" on every result.
  - "Bin position: stowed / out / lowered" moves it (SRDF BIN_STOWED / BIN_EXTENDED / BIN_LOWERED).
  - Every verdict states the bin setting.
  - A pole height that puts the arm base into the bin at the chosen position triggers a warning. The band is computed
    from the model at start-up.
- **(d) Analysis pane.** For the selected task step or whole flow, the right pane shows:
  - left and right joint plots on one time axis;
  - a dual-lane timeline (programs, start lags, approach moves, pole moves, hand changes, waits, operator checkpoints);
  - per-segment durations;
  - the total cleaning time as an expected value with a band;
  - per segment, velocity headroom (peak per-tick joint speed at the 5 ms cycle against the rating) and acceleration
    headroom.
- **(e) Speed controls.**
  - Per-segment speed factors are applied through C2's scale engine.
  - Durations, totals and headroom update within 1 s.
  - Factors below 1.0× are suggested when a segment is already over its rating.
  - The simulation and collision results show "out of date" until re-checked.
  - The view marks which arm sets the pace in each together-block, and how long the other arm waits, with the drift
    uncertainty.

### 5.3 Clarifications
- **"Left/right screen"** means the left and right panes of one window, not two monitors.
- **"Trajectory plot analysis"** means per-arm joint plots plus timing and speed headroom, built from C2.
- **"Optimize speed and cleaning time"** means choosing per-segment speed-ups, or necessary slow-downs, within joint
  ratings, acceleration advisories and collision limits.
  - Changes resample the file, because point spacing is the velocity profile (`TRK:9-11`).
  - The run speed % is never used for this; it scales only the approach move (`TRK:20-24`).
- **"Correct URDF"** means `alix_workspace/urdf/alix.urdf`.
  - **[M, confirmed V]** It is identical to the 2026-10-10 expansion of the current xacro.
- **"Correct meshes"** means a re-baked overlay. The current bake shows the old 406 mm bin and has no camera.
- **"Bin inclusion/exclusion"** is two settings:
  - whether the bin is fitted;
  - if fitted, its position.
- **Durations are predictions:** replay law with band, start lag ±1 s, approach-move law, pole law. The two arms drift.
- **Scope of the borrow verdict.** It covers ALIX Studio code only.

### 5.4 Findings

| # | Source | Finding | Evidence |
|---|---|---|---|
| 1 | Measurement **[M, confirmed V]** | **The URDF is correct.** Compared with `ros/simulation/isaacsim/generated/staging/alix_expanded.urdf` (2026-10-10 13:25, after the xacro's last edit on 2026-10-07 12:17): 128 joints and 129 links each, and 0 differences in type, parent, child, origin, axis, limits, mimic, mesh files and scale, primitives, or visual/collision origins. Map 02's "133 links / 183 joints" is wrong. The Studio and alix_workspace read this file. | in-memory XML diff; `DAPC:40`; `BC:226`; `WS/config/alix_workspace_config.yaml:41-50` |
| 2 | Measurement **[M, confirmed V]** | **The bake is stale.** `visual_overlay_baked.npz` dates from 2026-08-15 10:30: 200 sub-meshes, 100 links, 220,622 vertices, 268,178 triangles. Its `bin_link` reaches z 0.4382 m, the old `Bin.obj` top; `BinShort.obj` tops out at 0.3982. `camera_link` is missing, so the server loads the 15.8 MB `d435.dae` from the vendored realsense-ros tree outside the repo. The URDF switched to BinShort on 2026-08-29. | `WS/workspace_data/visual_overlay_baked.npz`; `WS/urdf/alix.urdf:248-310`; `/home/newtonjeri/alix_ws/src/vendor/realsense-ros/realsense2_description/meshes/d435.dae`; `TG/server.py:1950-1980` |
| 3 | ugv_description bin.xacro | The physical bin has not been remade; it is about 40 mm taller than the model. | `BINX:32-34` |
| 4 | ALIX Studio, alix_workspace | **The bin is fixed LOWERED in at least seven places.** The UI text says so, and the DAC body set is fixed (`include_body` is not exposed). | `TG/server.py:1135, 1529, 1725, 1991`; `DAPC:434`; `BC:629`; `TG/schedule_chain.py:360`; `TG/web_editor/template.html:753`; `TG/server.py:1763-1768`; `DAPC:644-648` |
| 5 | Measurement **[M, confirmed V]** | **Bin vs taught data.** Setup: pole 50 mm, GRIP, SRDF ACM, every 25th row, full arm link set vs bin, handles, covers and rail. **STOWED:** 0/440 L_P03P05, 0/418 R_P01P03, 0/390 R15_BUPFUL, 14/227 R_SPILLAGE, 0/142 R10_HNGR. **EXTENDED:** every row. **LOWERED:** 28/440, 111/418, 0/390, 160/227, 11/142 (the draft had 27 and 115; same picture). 4 of 5 taught low-pole programs collide with the Studio's fixed LOWERED bin. | in-memory |
| 6 | Measurement **[M, confirmed V]** | **Interlock band in the current model** (arms at the initial pose, 5 mm steps). **STOWED:** `L/R_base_link` ↔ `lid_opening_cover1` from 0.070 to 0.275 m, `cover2` from 0.160 m, and `L/R_Link1` ↔ `cover2` from 0.190 m. **EXTENDED:** `L/R_Link1` ↔ `cover1` from 0.180 to 0.260 m (depends on J1). **LOWERED:** clear. The documented bands all disagree with this: orchestrator 0.158-0.277 m plus a 5 mm margin, with "half_length 0.150 legal by 7.2 mm"; Studio copy 0.153-0.282 m; bin.xacro "0.16 to 0.285". V6 commands poles of 50, 150 and 200 mm. | in-memory; `TCFG:62-96`; `TG/web_editor/editor.js:17148`; `BINX:80-88` |
| 7 | ALIX Studio web_editor/css | There is no left/right split. DAC is a top band (430 px controls plus a plot of at least 64 px, band capped at 210 px) with the log in the right dock. Docks resize by drag and keyboard, within 180-620 px with a 320 px viewport floor, and are audited at 1366 × 768. Storage writes are not guarded. | `TG/web_editor/editor.css:1097-1112, 1722`; `TG/web_editor/editor.js:7562-7645` |
| 8 | ALIX Studio web_editor | Schedule & Run uses one proportional DOM track fed from the orchestrator CSV. It is a pattern only, not dual-lane. | `TG/web_editor/editor.js:13587-13612` |
| 9 | alix_dispatch, DAV, control app, emulator docs, run records | Timing laws: replay `0.2322 + 5.6737 ms × (N−1)` with a band of 5.35-6.10 ms per row; ALIX clock envelope; start lag `0.351 + 0.41 ms × N`; pole `lift_travel_s`; settle 0.6-1.0 s; hand stroke law. Approach moves are 5-10× the emulator law **[V]** (§2.2). | `TRK:1558-1570`; `DAV:172-224`; `POLE:3-14`; `RUN14:1947-1949`; `EMUDOC:73, 84`; `RUNS/` |
| 10 | Control app UI v53 (high level) | SAFE TIMELINE: TOGETHER sections may differ per arm; a WAIT before a TOGETHER program on both arms is a hard checkpoint with operator confirmation; RUN FIRST / WAIT OTHER run serially; the barrier is used only at launch. The V6 stage list (TIMELINE-1, LID-PREP-1, LID-CHECK-1, TIMELINE-2, serial 78, serial 79, TIMELINE-3) comes from maps 08/10 and was not re-run here. | `UI53:9665-10017, 10209` |
| 11 | alix_dispatch | The speed knob is not a playback speed, and point spacing is the velocity profile. | `TRK:9-11, 20-24` |
| 12 | Measurement **[V]** | **Rates at the 5.0 ms cycle.** R_P06P07 peaks at 2.454° per row (J6, row 3194): 491 °/s, or 442 °/s at 5.5545 ms, against a 225 °/s rating, with 193 joint-ticks over rating. 10 of 15 V6 files have ticks over rating (R2_LIDTOP 60, R_P10P12 70, R_P01P03 33, …). | in-memory per-row deltas |
| 13 | V6 package | `DUAL_FLOW_TIMELINE_V6.json` is listed in SHA256SUMS and named in the README but is absent. | `V6/SHA256SUMS.txt:14`; `V6/README.txt:78`; `V6/JSON/` |
| 14 | Measurement **[V]** | The upload-channel drift window for two 3,751-row tracks released together is ±3.56 s. | `DAV:912-973` |

### 5.5 Can it be borrowed from ALIX Studio?
**Verdict: Partial. Borrow some, build the rest.**

**Borrow:**
- the robot-geometry payload and pose applier, serving a re-baked overlay through a thin loader;
- the viewport;
- the dock splitter (drag, keyboard, persist, clamp);
- the playback and highlight pieces, through R3/R5;
- the collision-set view that shows the collision-only covers and handles.

**Build:**
- the 3D-left / plots-right layout with per-pane maximise;
- the dual-lane timeline with v53 rules and measured laws;
- durations with bands;
- velocity and acceleration headroom;
- speed controls linked to C2;
- the bin control: visibility, collision set, position, interlock warning.

The Studio already reads the correct URDF, but its bake is stale and its bin is fixed LOWERED in at least seven places.

**Input-claim check: partly accurate.**
- The Studio already reads the correct URDF (**[M, confirmed V]** 0 differences against a fresh expansion of the current
  xacro).
- Its meshes are the right files, but the overlay it serves is stale: old 406 mm bin, no wrist camera.
- There is no 3D-left / plots-right view and no speed analysis.
- The note is right that a bin toggle is needed. The bin is fixed LOWERED, and **[M, confirmed V]** that state collides
  with 4 of 5 taught low-pole V6 programs.

| Other source | Verdict | Detail |
|---|---|---|
| `alix_workspace/urdf/alix.urdf` | Borrow as-is | Frozen asset. Regenerate only on a ROS machine. |
| `tools/meshes/bake_visual_overlay.py` | Borrow as-is (re-run) | Re-bake into Lite's bundle, never over the ALIX copy. Needs meshoptimizer and the realsense tree (`/home/newtonjeri/alix_ws/src/alix/tools/meshes/bake_visual_overlay.py:1-45`). |
| SRDF `BIN_*`, bin.xacro, peripheral_joints.yaml | Borrow as data | (0,0) / (0.2,0) / (0.2,0.45) (`SRDF:286-297`). |
| orchestrator `pole_bin_conflict` | Reference | Keep the idea. The constant is stale; compute the band from the model. |
| Trajectory HTML (C2) | Borrow with adaptation | Joint plots, per-region scale and auto speed analysis. |
| Control app UI v53 | Reference (high level) | SAFE TIMELINE rules (`UI53:9665-10017`), re-implemented as a data model. |
| `alix_dispatch.track` | Borrow as-is | `expected_replay_s`, `PLAYER_RAMP_S`, `PLAYER_MS_PER_PT`, `CLOCK_RATE_ENVELOPE_MS` (`TRK:1558-1570`). |
| `tools/dual_arm_verdicts.py` timing envelopes | Borrow with adaptation | Start lag, clock envelope and window (`DAV:150-224, 862-973`). Re-fit on Lite; ALIX stops at 4,281 rows. |
| ALIX hardware run records | Borrow as data (seed only) | Approach-move law (`RUNS/*/run_slot*.json`). |
| `tools/taught_tracks.py` G2 rate gate | Borrow with adaptation | Ticks over rating at 5.0 ms (map 06 §2.5). |
| V6 package | Test data | FINAL JSONs and the runtime CSV. |

### 5.6 Borrow and build lists

| Borrow | From | Adaptation |
|---|---|---|
| Dock splitter | `TG/web_editor/editor.js:7562-7645`; `TG/web_editor/editor.css:331-341` | Vertical splitter. Keys `alixLite.*`. Storage in try/catch. |
| Maximise/restore pattern | `TG/web_editor/editor.js:7538-7559` | The Studio's button collapses both docks; Lite maximises per pane. |
| Robot geometry payload with baked overlay | `TG/server.py:1864-1998` | Thin loader. Re-baked `.npz`. Home pose from the bin setting and region pole. |
| Collision-only links in the "Collision" view | `TG/server.py:1949-1986` | Keep, so covers and handles show when collision meshes are on. |
| Playback, pose applier, highlight | via R3/R5 | Shared. |

| Build | Approach |
|---|---|
| Split layout | CSS grid [3D \| splitter \| plots], default 55/45, at least 320 px per pane, with two presets. A ResizeObserver triggers `Plotly.Plots.resize` and a renderer refit. |
| Re-bake and model tests | Re-run the bake into Lite's bundle. Tests: the bake's `bin_link` top equals the URDF visual bin top; `camera_link` is present; the served URDF hash matches. |
| Bin control | Link set: `bin_rail_link`, `bin_link`, `bin_handle_left/right`, `lid_opening_cover1/2` (`bin_carriage_link` has no geometry). Visibility toggles. The collision set is built only when the bin is fitted. Position comes from SRDF. At start-up, sweep the pole against the covers at the chosen position to get the pose-independent base band. Link1 contacts depend on J1 and are left to R5. Warn when the region pole is in the band. |
| Timeline model | Per-arm events from the workflow JSON: start lag, approach (measured law), replay (law plus band), settle, pole, hand, wait or checkpoint. Group them into v53 stages. Per block, the critical path is the max over arms. Uncertainty bands come from the DAV envelopes. |
| Headroom and suggestions | Per segment, peak per-tick speed at 5.0 ms against 180/180/225×5 °/s, and peak acceleration against the 600 °/s² advisory. A factor s scales velocity by s and acceleration by s². Suggested maximum factor = min over joints of min(0.95·v_r/v_peak, √(0.95·a_adv/a_peak)), capped by C2. It falls below 1.0 when a segment is over rating. Show the ticks-over-rating count. |
| Speed controls | Factors are written as C2 scale regions, with live update. Mark R5 out of date and run the background quick re-check. |
| Pace-setter display | Per together-block, mark the slower arm and show the other arm's idle time, with the drift band. |

### 5.7 Recommendation
Build the Dual-Arm view as a Lite screen that reuses R3's viewport and player and R5's checks.
- **Fix the model first.** Re-bake the overlay into Lite's own bundle and add hash, height and camera tests.
- **Make the bin a cell-profile setting** (fitted yes/no, position). **[M, confirmed V]** The taught V6 motions collide
  with the Studio's LOWERED bin in 4 of 5 low-pole programs. A stowed bin would put the arm bases into its covers at 150
  and 200 mm. This suggests the bin was not fitted (inferred), so ask the requester before choosing a default.
- **Build the right pane from:**
  - C2's per-arm joint plots;
  - a dual-lane timeline that follows v53 rules, with durations from measured laws (player law with band, start lag,
    measured approach moves, pole law);
  - per-segment velocity and acceleration headroom at the 5 ms cycle;
  - speed controls that write C2 scale regions and invalidate the collision result.
- **Show overlaps between arms with their drift uncertainty.** Never present predicted durations as exact.

### 5.8 Approach
1. Re-bake with `bake_visual_overlay.py` into Lite's bundle (realsense tree present). Add the tests.
2. Ask the requester the bin questions and record the answers as cell-profile defaults.
3. Add the bin control to the scene and the R5 world builder. Compute the interlock band at start-up and show warnings.
4. Build the split layout and verify it at 1366 × 768.
5. Build the timeline model with v53 stage rules and the measured laws. Unit-test it against the V6 FINAL pair, running
   v53's stage builder in a test as the reference.
6. After the first Lite runs, fit Lite's own laws (approach vs gap and transit %, start lag vs N, replay band) to replace
   the ALIX seeds.
7. Render the right pane: joint plots and the dual-lane timeline on one x-axis, durations with bands, block and total
   times, and the pace-setter.
8. Add headroom and suggestions. Wire the speed controls to C2 with staleness and the background quick re-check.
9. Sync hover and scrub across views through R3's `currentRow`.

### 5.9 Acceptance criteria
1. The splitter resizes from 25 % to 75 % by drag and by arrow keys, and both panes re-render. At 1366 × 768:
   - there is no horizontal scroll;
   - each pane is at least 320 px;
   - each pane has Maximise/Restore.
2. The served URDF's SHA-256 equals `WS/urdf/alix.urdf`. A scripted comparison against a fresh expansion shows 0
   differences in joints, limits, mimics and mesh references.
3. The re-baked overlay's `bin_link` top equals the URDF's visual bin top (0.3982 m for BinShort, or the physical bin if
   chosen). `camera_link` is in the bake, and no DAE is loaded live at start-up.
4. **Bin control.**
   - "Bin fitted: no" hides all bin links, and no `bin_*` or `lid_opening_cover*` pair appears in any check.
   - "Bin position" moves the bin to (0,0), (0.2,0) or (0.2,0.45).
   - With the bin stowed and the region pole inside the computed band (current model **[V]**: 0.070-0.275 m), a warning
     shows.
   - Every verdict states the bin setting.
5. **Durations.**
   - Each program's expected replay time equals 0.2322 + 0.0056737 × (N − 1) s and is shown with its band.
   - Start lag, approach-move, pole and hand durations are shown and labelled "modelled", with their source.
   - A unit test reproduces the V6 LEFT and RIGHT flow totals from the FINAL JSONs using the same laws.
6. For the V6 FINAL pair the timeline shows the stages v53's builder produces. Expected: TIMELINE-1, LID-PREP-1,
   LID-CHECK-1, TIMELINE-2, serial 78, serial 79, TIMELINE-3.
7. **Speed changes.**
   - Setting a segment to 1.5× updates its duration and the totals within 1 s.
   - Velocity and acceleration headroom and the ticks-over-rating count are recomputed.
   - The simulation is marked "out of date".
   - A segment already over rating shows a suggested factor below 1.0× (e.g. R_P06P07, 491 °/s at 5 ms).
8. In each together-block the slower arm is marked and the other arm's idle time is shown with the drift band.
9. Hovering or scrubbing any plot moves the 3D robot and all other cursors to the same instant.

### 5.10 UX notes (non-technical operators)
- Headline: "Cleaning time: 4 min 12 s (about 4:05-4:30)". Below it: "Left arm busy 3:48 — right arm busy 3:31 — right
  waits 41 s".
- Show a speed meter per segment.
  - Suggestions read "Can go up to 1.8× faster" or "Too fast now: slow to 0.6×".
  - The operator presses Apply; nothing changes silently.
- Bin wording: "Bin fitted on robot: Yes / No" and "Bin position: Stowed / Out / Lowered". When the bin is off, a
  persistent "Bin not checked" chip shows.
- Use a fixed layout with two presets. Never hide the 3D view or the plots without a visible "Restore".
- Timeline lanes use deck region names and arm colours (blue = left, orange = right), always with a text label too.

### 5.11 Safety notes
- Turning the bin off removes real collisions from the check. Ask for confirmation once and print "Bin not checked" on
  every result.
- **[M, confirmed V]** The documented stowed-bin bands disagree with the current model (0.070-0.275 m). Compute the band
  from the model and re-measure on the robot.
- The model bin is 40 mm shorter than the physical bin (`BINX:32-34`). If the bin is fitted, use the physical height or a
  40 mm margin.
- Speed-ups raise joint rates by s and accelerations by s². Both make 4109 dynamics trips more likely (`CANFD:675-682`).
  Keep "no ticks over rating at the 5 ms cycle" as a hard limit and the acceleration advisory as a warning.
- Durations are predictions with a ±6 % run-to-run spread plus a start-lag spread of about 1 s. Never use them to time
  the two arms against each other on the robot.

### 5.12 Dependencies, risks, open questions
**Dependencies**
- R3: viewport, player, browser FK.
- R5: checks, bin set, quick re-check.
- C2/R2: scale engine and plots.
- R12: rates.
- R8: flow composition.
- R1/R7: regions, pole and hand.
- v53 timeline rules, as a reference data model.
- Lite's own timing measurements (C4).
- The requester's answers on the bin.

**Risks**
- `DUAL_FLOW_TIMELINE_V6.json` is missing, so the dual timeline must be derived from the per-arm JSONs and v53 rules.
- Arm drift (seconds) and differing pole timing models make predicted overlaps approximate.
- The approach-move law at 30 % transit speed is unmeasured.
- The bin default is undecided.
- Re-baking needs meshoptimizer and the realsense tree.
- Plot performance with two 11,000-row, 7-joint traces: decimate for display and keep exact rows for markers.

**Open questions**
1. Is the bin fitted on the Lite robot? If so, in which position during cleaning, and which height (406 or 366 mm)?
2. Should the view write speeds directly into C2, or only suggest them?
3. Is there a target cleaning time per region or task step?
4. What is the operator laptop's resolution?
5. Should the view cover a single task step, the whole flow, or both?

**Priority: Should.** Sub-items (b) and (c) are Must and ship with R3/R5.

**Effort: L, about 13-15 person-days.**

| Item | Days |
|---|---|
| Layout | 2 |
| Re-bake and tests | 1 |
| Bin control and interlock | 2 |
| Timeline model with measured laws | 4.5 |
| Headroom and speed controls | 3 |
| Tests | 1.5 |

This assumes R3 and C2 exist.

---

## 6. Cross-cutting notes
1. **Measurements.** **[V]** re-ran the analyst's key [M] numbers. Confirmed:
   - URDF parity;
   - the stale bake;
   - bin vs taught data (within ±4 rows);
   - the stowed-bin band;
   - the arm base height;
   - the thumb-to-wrist contact (main pair `L_CameraHolderLink`);
   - the cost picture.

   Corrected:
   - the wall count (28 rows, not 35);
   - the toilet contact (fingers, not thumb, with GRIP).

   Added:
   - the pad-frame proxy failure;
   - approach-move timing;
   - the drift window;
   - `plan_num` behaviour.
2. **Rows and clocks.** Use one row index for C2, R3, R5 and R4. Durations and 1× use the fitted law (`TRK:1558-1570`);
   rate checks use 5.0 ms. Watch the 0- vs 1-based numbering (`TRAJ:1181-1187`).
3. **Timing around programs.**
   - Start lag (`DAV:176-224`), approach move (**[V]** measured, 5-10× the emulator law), settle, pole and hand each have
     their own law.
   - Seed them from ALIX and re-fit on Lite's runs.
4. **Two-arm timing on the upload channel is uncertain by seconds.**
   - **[V]** ±3.56 s for 3,751-row tracks (`DAV:912-973`).
   - Adopt ALIX's serialised bridge-to-start (`DAF:3763-3800`).
   - Show nominal and window results; run uncertain pairs serially.
5. **One FK.** Port `UrdfModel.link_world_transforms` (`UK:294-344`) to the browser with a parity test in CI, as the
   Studio warns (`TG/server.py:5296-5303`). Keep FCL on the server.
6. **The cell profile is the biggest single risk.** It covers the toilet model and pose, walls, floor and bin.
   - **[V]** The 800 mm venue gives 28 false wall rows on a taught track.
   - The Studio's LOWERED bin collides with 4 of 5 low-pole programs.
   - Plan on-site calibration and validate by replaying known-good files.
7. **Never rely on ALIX defaults** (bin LOWERED, poles 0.300 m, open hands, `DAPC:385-452`). Seed pole, hand and bin from
   region settings with the `HU:30-60` law. The library accepts hand states only by SRDF name (`DAPC:439-450`), so inject
   counts as joint values.
8. **Collision engineering.**
   - One worker process (FCL managers are not thread-safe, `BC:150-171`).
   - Full hands for arm-arm and arm-environment checks; curated self pairs (0.71 ms per arm).
   - Every row is affordable (about 2-3.5 ms).
   - Re-enable `SRDF:1442, 1452`.
   - Fix the `classify_pair` token order (`RED:90-105`).
9. **Contact vs collision is decided per hand preset.**
   - **[V]** With GRIP, the flexed finger links make the lid contact, and the `glove_frame_2` origin sits 19-35 mm off the
     surface.
   - ALIX's lists were derived for `open_tenth` (`CL:81-84`).
   - The requester needs to rule on which links may touch.
10. **Real-arm data.**
    - Prefer a 5 ms capture of every real test with alix_dispatch's `UdpRecorder` plus `push_error_codes` and the ordered
      matcher (`TRK:974-1080, 1653-1688, 2037-2075`).
    - The runner printing joints with each code is the fallback.
    - `plan_num` does not index points (**[V]** `RUNS/2026-09-01/run_slot70_190758.json`).
11. **Architecture.**
    - Build a standalone Lite backend and front end that lift Studio parts.
    - Replace `_ik_context` (it imports the X2 reachability module, `RM:179`) with a thin loader.
    - Replace `build_scene_manifest` (coupled to the ALIX Regions and generator, `VGC:219-330`) with a thin manifest.
12. **Theme.** Build one token-based `plotTheme()`. The Studio's DAC plot hard-codes dark-theme colours
    (`TG/web_editor/editor.js:14485-14488`).
13. **Control app (high level, plan-blocking items only).**
    - UI v53 plus runner v14 is the current pair.
    - The runner prints error codes without joints; a one-line fallback change is needed for the R5 real-arm highlight.
    - Codes are polled every 0.5 s, each printed once, and never abort a program.
    - Multi Run and Page-01 Load Sequence raise `TypeError` (missing `@staticmethod`). This does not block R3/R5/R10, but
      fix it before porting.
14. **Operations prerequisite for any real test.**
    - Align both arms' collision-detection level (left 3 / right 0 on 2026-08-13, `AVI:27-33`).
    - A 4109 latches and must be cleared at the pendant.
    - It can fire without contact on aggressive motion.

---

## 7. Change log (material changes from the analyst draft)

| # | Req | Change | Reason | Evidence |
|---|---|---|---|---|
| 1 | R3 | 1× and acceptance criterion 4 now use the fitted law: 21.5 s ± 1 % for 3,751 rows (was 20.8 s ± 3 %). | 20.8 s was the nominal 5.5545 ms × N, which contradicted the draft's own fitted-law clock and the R10 durations. | `TRK:1558-1570`; 0.2322 + 5.6737 ms × 3,750 = 21.51 s |
| 2 | R3 | The pad-path rule "glove_frame_2 within 20 mm" was replaced by a contact-link wiped-area view. | **[V]** While the GRIP fist touches the lid, the glove frame is 19-35 mm away; only 1 of 188 rows is within 20 mm. The contact list was derived for `open_tenth`. | in-memory; `CL:81-84`; `GF:99-100` |
| 3 | R3/R5 | Toilet-contact finding corrected: fingers on 36/188 rows, thumb on 4, connector on 1. | The draft's "thumb on 20" matches an open hand (**[V]** 21), not GRIP. | in-memory, `BC:401-464`, `HU:30-60` |
| 4 | R3 | Venue finding corrected to 28/188 rows (R_Link5 28, R_Link6 6); the 1000 mm venue is also clear. | **[V]** re-measurement; the draft said 35. | `PSV:117-155` |
| 5 | R3/R10 | The approach-move duration law was replaced by a measured law. | **[V]** ALIX hardware records show 5-10× the emulator law. | `EMUDOC:73`; `RUNS/2026-09-02/run_slot76_210943.json`; `RUNS/2026-09-01/run_slot70_190758.json` |
| 6 | R3/R10 | Start lag, settle and the hand stroke were added to flow durations. | They were missing from the timeline model. | `DAV:176-224`; `RUN14:1947-1949`; `EMUDOC:84` |
| 7 | R3 | Coupling to X1/X2 recorded; a thin loader and thin manifest required. | `_ik_context` imports reachability (X2). `build_scene_manifest` enumerates ALIX regions, paint meshes and generator constants. `fixture_manager` shifts along X only. | `RM:179`; `VGC:219-330`; `BC:401-460` |
| 8 | R3/R5 | Borrow added: toilet-face marking and painting. | It exists and works in joint space; the draft missed it. | `COL:430-487`; `TG/web_editor/editor.js:12008-12032` |
| 9 | R3 | Safety: a reduced-speed first run needs a slowed copy of the file. | The run speed scales only the approach move. | `TRK:20-24`; `RUN14:1681-1684` |
| 10 | R3 | Dependency on a C2 generic selection; row numbering noted. | TRAJ `addRegion` creates only scale regions and needs the Scale stage; rows are 1-based. | `TRAJ:1181-1187, 1392-1396` |
| 11 | R3 | Shim coupling recorded. | `build_timeline` reads MANIFEST task dicts, `sweep` writes temp files, there is no 7-value check. | `DAV:580-618, 709-733` |
| 12 | R3 | Park pose and partner time offset added; effort raised to 13-15 days. | Undefined behaviour for an untaught partner; extra work for the thin manifest and the wiped area. | `BC:66-72` (ALIX idle-partner convention) |
| 13 | R3 | Players corrected. | The transport is shared by DAC and Schedule only; DAC's interval is max(20 ms, dt). | `TG/web_editor/editor.js:8746-8748, 14572` |
| 14 | R5 | Thumb contact corrected. | **[V]** The main pair is `L_CameraHolderLink` (41 rows) plus `L_Link7` (10), on rows 2788-2828. | in-memory |
| 15 | R5 | Costs restated per arm and per row; curated self pairs measured. | **[V]** 0.71 ms per arm; 87 ms full-mesh self per arm. | in-memory; `BC:354-399` |
| 16 | R5 | The full check now covers every row; quick mode is labelled. | Every row is affordable, and coarse passes can miss thin contacts. | **[V]**; `DAV:352-356` |
| 17 | R5 | The drift window changed from optional to required for arm-vs-arm checks. | **[V]** ±3.56 s for 3,751-row tracks; ALIX's envelope is limited to 4,281 rows. | `DAV:150-224, 862-973` |
| 18 | R5 | Real-arm path: UDP capture plus the ordered matcher is preferred; the runner joints change is the fallback. Post-run codes without a time are handled. | alix_dispatch already has production capture tools. The runner prints each code once and reports post-run codes without a time. | `TRK:974-1080, 1653-1688, 2037-2075`; `RUN14:244-268, 1925-1967` |
| 19 | R5 | The `plan_num` open question is resolved. | **[V]** Hardware records show `plan_num` fixed at 1 for the whole replay. | `RUNS/2026-09-01/run_slot70_190758.json`; `EMU:46-49` |
| 20 | R5 | Library limits recorded. | `extra_managers` meets only the slim sets; hand states are accepted by SRDF name only. | `DAPC:439-450, 645-646, 699-703` |
| 21 | R5 | The `classify_pair` token-order defect is recorded. | `lid_opening_cover1` reads as toilet. | `RED:90-105` |
| 22 | R5 | Approach, pole and hand-move checks added, including a live pre-run approach check. | Unchecked controller moves and the pole/bin interlock. | `RUN14:205-214`; `DAF:3763-3800` |
| 23 | R5 | Pad-contact acceptance criterion rewritten for GRIP; `R_ConnectorLink` is a collision. | **[V]** contact evidence. | `CL:81-110` |
| 24 | R5 | The seeded test is now oracle-based; the regression citation is fixed to `GENF:3050-3051, 3107`, with the hardware-disagreement note. | "+30° on J1" does not guarantee a collision; the draft's citation path was imprecise. | `GENF:2782, 3050-3051, 3107` |
| 25 | R5 | Effort raised to 19-21 days. | Every-row sweeping, drift window, move checks, capture integration. | — |
| 26 | R10 | Bin hard-coding raised from 5 to at least 7 places. | Also `BC:629` and `TG/schedule_chain.py:360`. | as cited |
| 27 | R10 | Bin-vs-taught numbers re-measured. | **[V]** LOWERED 28/440, 111/418, 0/390, 160/227, 11/142. | in-memory |
| 28 | R10 | Interlock band details added; all three documented bands disagree. | **[V]** `cover2`, `Link1` and the EXTENDED band; bin.xacro's 0.16-0.285 m. | `BINX:80-88`; `TCFG:62-96` |
| 29 | R10 | R_P06P07 rate corrected to 491 °/s at the 5.0 ms cycle. | The draft used the 5.5545 ms clock (442 °/s) while stating the 5.0 ms rule. | **[V]** |
| 30 | R10 | Acceleration headroom (∝ s²) added; suggestions below 1.0× allowed. | Most V6 files are already over rating; accelerations drive 4109 trips. | **[V]**; `CANFD:675-682` |
| 31 | R10 | Re-bake into Lite's bundle; the camera DAE location recorded. | Avoid writing to ALIX; the DAE sits in the vendor tree. | **[V]**; `/home/newtonjeri/alix_ws/src/vendor/realsense-ros/realsense2_description/meshes/d435.dae` |
| 32 | R10 | URDF parity confirmed; map 02's counts flagged as wrong. | **[V]** XML diff. | in-memory |
| 33 | R10 | The v53 7-stage claim marked as not re-verified; the acceptance criterion now uses v53's builder as the oracle. | It was not re-run here. | `UI53:9665-10017` |
| 34 | All | Cross-cutting notes rewritten for timing, drift, capture and the thin loader/manifest. | Material new evidence. | as above |
