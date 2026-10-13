# Map 11: Robot description, collision assets and simulation back-ends (for ALIX Studio Lite)

Scope: robot model assets (URDF/xacro, meshes, bin, commode, walls), how ALIX Studio serves them without ROS,
the RealMan SDK emulator, Isaac Sim, and what Lite can reuse for its 3D view (R10), simulation (R3), sim/real
toggle (R4) and collision highlighting (R5).

Abbreviations used in the evidence:
- `REPO` = `/home/newtonjeri/alix_ws/src/alix`
- `DESC` = `REPO/ros/description`
- `WS` = `REPO/core/python/alix_workspace/src/alix_workspace`
- `TG` = `REPO/core/python/alix_taskgen/src/alix_taskgen`
- `UC` = `/home/newtonjeri/alix_studio_lite_ws/upload_channel`

Every claim cites `path:line`. Anything marked "(inferred)" is my reasoning, not something stated in the source.
I ran no xacro, ROS, test, server or robot command. FK numbers come from a numpy-only FK I wrote over the
committed flat URDF (read-only).

---

## 1. Robot composition

### 1.1 Top-level assembly (alix.urdf.xacro)
- The top-level file is `DESC/alix_description/urdf/alix.urdf.xacro` (264 lines). It includes these macros:
  - `ugv_description/urdf/ugv.xacro` (`:144`)
  - `pole_description/urdf/pole.xacro` (`:147`)
  - `rm_description/urdf/rm_75_macro.xacro` (`:150`)
  - the Inspire RH56DFX left and right hands (`:153-154`)
  - `intel_realsense_d435_description/urdf/realsense_d435.urdf.xacro` (`:157`)
- xacro args:
  - Hardware: `use_mock_hardware`, `use_real_hardware`, `use_isaac_hardware` (`:3-8`).
  - Real-arm IPs: left `192.168.1.10`, right `192.168.1.103`; UDP target `192.168.1.235`, ports 8089/8090 (`:26-31`). The file marks these as hand-mirrored defaults, with `alix_core/sdk/endpoints.py` as the authority (`:10-25`).
  - Many real-driver args, `connection sdk|double` and the CANFD args (`:33-101`). These only reach `rm_side_ros2_control` blocks when `real_hw and arm_unit` (`:172-216`).
  - `include_arm_unit` (default true; `:116`) keeps or removes everything from the sliding plate up (README `DESC/alix_description/README.md:40-66`).
  - `exclude_thumb` (default false; `:135`). The committed offline URDF stays on the FULL model (`:133-134`).
- **No xacro argument includes or excludes the bin.** `ugv_macro` always instantiates `bin_macro` (`DESC/ugv_description/urdf/ugv.xacro:27-29`). The only structural switches are `include_arm_unit` and `exclude_thumb`.

### 1.2 Link/joint tree
Parsed from the committed flat URDF `WS/urdf/alix.urdf`: **129 links, 128 joints** (77 fixed, 39 revolute, 8 continuous, 4 prismatic). The README says "128 links with it" (`DESC/alix_description/README.md:52`); that one-off difference is cosmetic.
```
base_link
└─ UGV_body            [UGV_body_joint fixed z=0.195]               ugv.xacro:18-23
   ├─ bin_rail_link    [bin_mount_joint fixed (0,0,0.294)]           ugv.xacro:27-29, bin.xacro:125-129
   │  └─ bin_carriage_link [bin_rail_joint PRISMATIC +X 0..0.200 m, origin x+0.06]   bin.xacro:163-172
   │     └─ bin_link       [bin_drop_joint PRISMATIC -Z 0..0.450 m]  bin.xacro:186-195
   │        ├─ bin_handle_left / bin_handle_right   [fixed, z -0.04]      bin.xacro:285-343
   │        └─ lid_opening_cover1 / lid_opening_cover2 [fixed, z -0.04]   bin.xacro:352-402
   ├─ glove_wearing_unit_base [fixed (-0.20,0,0.982) pitch 0.1309]  ugv.xacro:33-35
   │  └─ glove_wearing_unit_arm [revolute 0..1.57 rad]               peripheral_joints.yaml:85
   ├─ UGV_lower_cover, 4x *_suspension (continuous) -> 4x *_wheel (continuous)  ugv.xacro:103-276
   ├─ L_Pole [L_pole_joint fixed (-0.01, 0.124, 0.296)]              alix.urdf.xacro:223
   │  └─ L_Sliding_Plate [L_sliding_plate_joint PRISMATIC +Z 0..0.315 m, v 0.16, origin z 0.1675]  pole.xacro:56-68
   │     └─ L_base_link [L_base_joint fixed (0.055,0,-0.01) rpy(0, pi/2, 0)]    alix.urdf.xacro:227
   │        └─ L_Link1..L_Link6 [L_joint1..L_joint6 revolute]
   │           └─ L_CameraHolderLink [L_joint7 revolute]  -> L_Link7 [fixed +0.05] -> L_ConnectorLink [fixed +0.0125]
   │              ├─ ILH_base_link [ILH_root_joint fixed z 0.015, yaw pi] -> left Inspire hand  alix.urdf.xacro:230
   │              └─ L_glove_frame_1..4, L_tip_frame, L_index_tip_frame (fixed IK frames)   ik_frames.xacro
   └─ R_Pole [R_pole_joint fixed (-0.01, -0.124, 0.296)]             alix.urdf.xacro:235
      ├─ R_Sliding_Plate -> R_base_link -> R_Link1..R_Link6 -> R_CameraHolderLink [R_joint7]
      │     ├─ R_Link7 -> R_ConnectorLink -> IRH_base_link (right hand) + R_glove_frame_*   alix.urdf.xacro:239-242
      │     └─ camera_bottom_screw_frame -> camera_link -> depth/infra1/infra2/color (+optical) frames  alix.urdf.xacro:245-247
      └─ alix_ref_frame [R_reference_frame_joint fixed z +0.035]    alix.urdf.xacro:250-258
```
- Arm chain:
  - Joint origins: j1 `(0,0,0.2405)` (`rm_75_macro.xacro:197`), j3 `0.256` (`:259`), j5 `0.21` (`:321`), j7 `0.114` (`:426`).
  - The camera holder sits between Link6 and Link7. `joint7` drives `CameraHolderLink`, and `Link7` is fixed to it at +0.05 (`rm_75_macro.xacro:425-432, 482-486`).
- The arms are mounted pitched 90°. The controller's frame is `Ry(+90°)` applied to URDF `*_base_link`, and the install pose reads `(0, 90, 0)` (`REPO/core/python/alix_dispatch/src/alix_dispatch/frames.py:3-16`; `TG/server.py:1794-1806`).
- Joint-angle convention: controller joint degrees map 1:1 to URDF radians.
  - `sim_bridge.joint_map` does `out[f"{side}_joint{i}"] = math.radians(deg)` (`TG/sim_bridge.py:163-164`).
  - At q=0 the link origins coincide to 1.7 µm and every axis agrees (`rm_75_macro.xacro:62-64`).
  - The pendant "Base" frame equals URDF `*_base_link` to 0.003 mm (`TG/server.py:1794-1799`).
- Arm-base height (FK, numpy, flat URDF) is `z = 0.6485 + pole` m in base_link. For pole 0.0, 0.15, 0.30 and 0.315 that gives 0.6485, 0.7985, 0.9485 and 0.9635 m. The bases sit at x 0.045, y ±0.124. `alix_ref_frame` is fixed at (-0.01, -0.124, 0.526) whatever the pole height.
- Pole units: the URDF metre value equals the lift mm/1000 (1:1 since 2026-08-30) (`DESC/ugv_description/urdf/bin.xacro:44-45`).
  - `UC/pole.py:22-24` reads `rm_get_lift_state()["pos"]/1000`.
  - `sim_bridge` multiplies `lift_hw` by `lift_hw_to_phys` and then divides by 1000 (`TG/sim_bridge.py:166-177`).
  - SRDF pole presets: `minimum_length` 0.01, `quarter_length` 0.075, `half_length` 0.15, `three_quarter_length` 0.225, `full_length` 0.30 (`REPO/ros/manipulation/alix_moveit_config/config/alix.srdf:271-315, 1101-1116`). Initial pose is 0.3 (`DESC/pole_description/config/initial_positions.yaml:6,10`).

### 1.3 Arm joint limits (URDF vs controller)
| joint | URDF limit (`rm_75_macro.xacro`) | URDF velocity | controller limit read from arms 2026-08-11 (`REPO/core/python/src/alix_emulator/rm_emulator.py:418-432`) |
|---|---|---|---|
| j1 | ±178° (`:201`) | π rad/s (180°/s) | ±183 |
| j2 | ±130° (`:232`) | π rad/s | ±135 |
| j3 | ±178° (`:263`) | 225°/s | ±183 |
| j4 | ±135° (`:294`) | 225°/s | ±169 left / ±140 right |
| j5 | ±178° (`:325`) | 225°/s | ±360 |
| j6 | ±128° (`:356`) | 225°/s | ±133 |
| j7 | ±360° (±2π) (`:430`) | 225°/s | ±365 |
- The flat URDF holds the same values: L/R_joint1..7 parsed at ±178/130/178/135/178/128/360.
- The URDF is uniformly the stricter set, so it is the safe choice for the R9 "obey joint limits" check (inferred).
- `expand_urdf.sh` documents an earlier drift in which joint1 was rounded to ±3.1 rad = 177.62° (`WS/../../scripts/expand_urdf.sh:24-28`). `docs/EMULATOR.md:87` still quotes the stale ±177.6/±130/... list, which the code corrected on 2026-08-12 (`rm_emulator.py:418-428`).
- The CANFD rate gate uses 180/180/225×5 °/s per joint (`alix.urdf.xacro:94-97`), matching the URDF velocities.
- Known model caveats:
  - The flange is 2.8 mm long along the tool axis (joint7 0.114 + holder 0.05 = 0.164 vs a measured 0.1612). This was left on purpose to keep FK parity with the hardware-proven URDF (`rm_75_macro.xacro:411-424`).
  - The emulator measures tool frames from Arm_Tip = ConnectorLink − 15.3 mm (`rm_emulator.py:143-149`).

### 1.4 Hands and camera
- Inspire RH56DFX L/R. The flat-URDF parse finds 12 revolute joints per hand:
  - 6 actuated: `thumb_1` (0..71.4°), `thumb_2` (0..30°), and `index_1`, `middle_1`, `ring_1`, `little_1` (0..77.006°).
  - 6 mimic: `thumb_3`, `thumb_4` and the `*_2` joints, e.g. `index_2` mimics `index_1` ×1.1169 −0.1865 (`DESC/inspire_left_hand_description/urdf/RH56DFX_L_8_19.xacro:414-483`; limits `:188,217,419`).
- SRDF hand presets per side: `release`, `grasp`, `half_grasp`, `quarter_grasp`, `tenth_grasp`, `thumb_grasp`, `open_tenth`, `pointing`. The left also has `lid_open` (`alix.srdf:659-787`).
- The emulator/SDK hand is 6 channels of 0..1000 (1000 = open). `sim_bridge.HAND_CHANNELS` maps them to radians with travels of 1.60 (fingers), 0.70 (`thumb_2`) and 1.10 (`thumb_1`) (`TG/sim_bridge.py:93-102, 179-184`). That exceeds the URDF limits (1.344 / 0.5236 / 1.246), and `UrdfModel.link_world_transforms` does not clamp (`WS/urdf_kinematics.py:314-344`). The result is cosmetic over-closing.
- The D435 hangs only on the RIGHT holder (`alix.urdf.xacro:244-247`); the left holder uses `connector_link2.stl` (`rm_75_macro.xacro:390-393`). Its visual is `package://realsense2_description/meshes/d435.dae` (flat `alix.urdf:4354`). That file lives OUTSIDE the monorepo at `/home/newtonjeri/alix_ws/src/vendor/realsense-ros/realsense2_description/meshes/d435.dae` (15.8 MB). It is resolved by scanning sibling `src/` dirs (`WS/package_index.py:40-71`).

### 1.5 The bin
- Source: `DESC/ugv_description/urdf/bin.xacro` (467 lines). Joint limits come from `ugv_description/config/peripheral_joints.yaml` via `xacro.load_yaml` (`bin.xacro:117-120`).
- Links: `bin_rail_link` (visual `BinRail.obj`, collision `BinRail.stl`), `bin_carriage_link` (no geometry), `bin_link`, `bin_handle_left/right`, `lid_opening_cover1/2`.
  - Handles and covers are **collision-only** links (`bin.xacro:280-402`). They exist to toggle per-link collision during grasps.
- Joints: `bin_mount_joint` (fixed), `bin_rail_joint` (prismatic +X 0..0.200), `bin_drop_joint` (prismatic −Z 0..0.450), 4 fixed attachments.
- States (`peripheral_joints.yaml:43-50`): STOWED (0,0), EXTENDED (0.2,0), LOWERED (0.2,0.45). Transitions are STOWED↔EXTENDED↔LOWERED. The SRDF mirrors these as `BIN_STOWED/BIN_EXTENDED/BIN_LOWERED` (`alix.srdf:286-297`).
- Bin geometry in base_link (my FK on the flat URDF plus the BinShort.stl extents):

| state | bin_link origin | body x range | floor z | rim z |
|---|---|---|---|---|
| STOWED | (0.06, 0, 0.489) | 0.043..0.232 | 0.505 | 0.871 |
| EXTENDED | (0.26, 0, 0.489) | 0.243..0.432 | 0.505 | 0.871 |
| LOWERED | (0.26, 0, 0.039) | 0.243..0.432 | 0.055 | 0.421 |

  LOWERED puts the bin between the UGV front and the commode (inferred from these numbers).
- **BinShort change (2026-08-29):**
  - The active `bin_link` uses `BinShort.obj` / `BinShort.stl`. That is 40 mm shorter (collision body 406 → 366 mm, z 0.016..0.382), and the logo-preserving band compression came from `tools/meshes/shorten_bin_mesh.py` (`bin.xacro:243-277`).
  - The original block is kept commented out (`bin.xacro:197-241`).
  - Handles and covers carry `xyz="0 0 -0.04"`, which MUST equal `REDUCTION_M` in the script (`bin.xacro:288-293, 355-360`; `REPO/BIN_HEIGHT_HANDOFF.md:152-169`).
  - Verified by my parse: `Bin.obj` y-max 0.4382 vs `BinShort.obj` 0.3982; `Bin.stl` z-max 0.422 vs `BinShort.stl` 0.382 (both 60 triangles).
  - **STILL OPEN: "the PHYSICAL part has not been remade"** (`bin.xacro:32-34`). I found no later evidence that it was remade.
- Collision policy:
  - Arm links must never be allowed to touch bin links; all 20 SRDF workaround rows were removed (`bin.xacro:60-67`). F78 is closed (`bin.xacro:69-75`).
  - Remaining interference: with the bin STOWED, `L/R_base_link` hits `lid_opening_cover1/2` at about 50–62 mm for pole 0.16–0.285 (`bin.xacro:77-99`). This is encoded as an orchestrator interlock: `_STOWED_POLE_FOUL_M = (0.158, 0.277)` plus a 0.005 margin, and `pole_bin_conflict()` (`REPO/ros/orchestration/alix_orchestrator/alix_orchestrator/core/task_config.py:85-125`).
  - Example (inferred): `UC/LEFT_V6_70SERIES_FINAL.json` commands `pole 200 mm`, which is inside that band if the bin is STOWED.
- Studio/offline tools always pose the bin LOWERED:
  - `_build_home` (`WS/dual_arm_plan_collision.py:385-435`)
  - `do_robot_geometry` home pose (`TG/server.py:1986-1992`)
  - `do_joint_presets` says "The bin is always LOWERED (not user-selectable)" (`TG/server.py:1763-1768`)
  - `body_collision` home (`TG/body_collision.py:629`)

### 1.6 Mesh formats and sizes
All unique meshes referenced by the flat URDF exist on disk; none are missing.

| set | format | files | MB |
|---|---|---|---|
| visual | STL (binary) | 74 | 48.7 |
| visual | DAE | 6 | 22.6 (`body.dae` 4.0, `wheel.dae` 2.3, `lower_cover.dae` 0.4, `camera_holder.dae`, `connector_link.dae`, `d435.dae` 15.8) |
| visual | OBJ+MTL | 5 | 9.8 (`ugv_cover` 3.5, `glove_wearing_unit` 3.55, `BinShort` 2.15 (14 materials, no textures — `BIN_HEIGHT_HANDOFF.md:33-35`), `glove_wearing_lever_arm` 0.43, `BinRail` 0.14) |
| collision | STL | 80 | 6.4 |

By subsystem:

| subsystem | visual | collision |
|---|---|---|
| arms+poles | 4.4 MB | 1.0 MB |
| hands | 46.4 MB | 2.7 MB |
| UGV | 17.5 MB | 5.0 MB |
| camera | 15.8 MB | — |
| glove unit | 4.0 MB | 0.4 MB |
| bin | 2.3 MB | 0.35 MB |

- RM75 per-link meshes: visual `rm_description/meshes/rm_75_arm/*.STL` at 0.2–0.43 MB each; collision `meshes/collision/rm_75_arm/*.STL` at 40–70 KB each.
- The pole uses `Pole.STL` (20 KB). The sliding plate is primitives: box `0.005×0.150×0.125` plus a cylinder of r 0.055 (`pole.xacro:72-104`).
- The commode and wall meshes are not part of the URDF (see §3).

---

## 2. Which URDF is current, and how a non-ROS app gets a flat URDF

### 2.1 The committed flat URDF
- `WS/urdf/alix.urdf` (196,666 B, 2026-09-06 17:30; git commit 82271103 "Fix inertial properties…") is the only committed expanded robot URDF in the repo.
  - A `find` turned up only it, the two Inspire-hand vendor URDFs, a probe URDF in `Resource/reports`, and the git-ignored Isaac staging copies.
- Its header says "autogenerated by xacro from …/alix_description/urdf/alix.urdf.xacro" (`WS/urdf/alix.urdf:1-5`).
- It is the "kinematic ground truth for every offline gate" (`REPO/core/python/alix_workspace/scripts/expand_urdf.sh:2-8`). Readers:
  - `alix_workspace_config.yaml` chains `left`/`right` → `urdf: ../urdf/alix.urdf`, `base_link` → `L_/R_glove_frame_2` (`WS/config/alix_workspace_config.yaml:41-50`)
  - `dual_arm_plan_collision._DEFAULT_URDF` (`WS/dual_arm_plan_collision.py:40`)
  - `BodyCollisionWorld._ensure` (`TG/body_collision.py:226-228`)
  - the Studio's `_ik_context` (`TG/robot_model.py:183-214`)
- **Is it current? Yes, kinematically and geometrically (verified).**
  - Since commit 82271103 the description changed only in:
    - real-hardware xacro args, which only affect `rm_side_ros2_control` when `use_real_hardware`;
    - `exclude_thumb` (default false keeps the thumbs);
    - a gazebo thumbless-overlay guard;
    - D435 Gazebo sensor resolution/FOV.

    Source: `git diff --stat 82271103 HEAD -- ros/description` and the diffs of `alix.urdf.xacro`, `gazebo_plugins.xacro`, `RH56DFX_L_8_19.xacro` and `realsense_d435_gazebo_plugins.xacro`.
  - The Isaac pipeline's fresh expansion, `REPO/ros/simulation/isaacsim/generated/staging/alix_expanded.urdf` (2026-10-10), has **0 differing joints** (type, parent, child, origin, limits) and **0 differing link mesh references** against the committed file. I compared them with a Python parse.
  - The kinematic gate test is cited as `test_kinematic_gate.py` (README `:53-55`; `rm_75_macro.xacro:63-66`).

### 2.2 Regenerating it (`expand_urdf.sh`)
- `REPO/core/python/alix_workspace/scripts/expand_urdf.sh`:
  - xacro source: `ros/description/alix_description/urdf/alix.urdf.xacro` (`:42`)
  - output: `core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf` (`:32`)
- **It needs ROS 2 Jazzy plus the workspace install.** It sources `/opt/ros/jazzy/setup.bash` and `$ROOT/../../install/setup.bash` (`:58-64`). `--check` diffs a fresh expansion against the committed file and exits 1 if stale (`:71-79`). Earlier path bugs once made it write a file nobody read (`:13-28`).
- Downstream caches are content-addressed by a hash of this file, so re-expanding invalidates the committed reach/manipulability `.npz` files (`:84-85`; `TG/server.py:750-762`).
- Lite consequence (inferred): Lite should consume the committed flat URDF as a frozen asset. It never needs xacro at runtime, and only needs `expand_urdf.sh` on a ROS machine when the description changes.

### 2.3 Resolving `package://` without ROS
- `WS/package_index.py` builds a `PackageIndex` by scanning `package.xml` under the repo root (nearest `.git`) and sibling dirs of `src/`. Scanning the siblings is how `realsense2_description` in `alix_ws/src/vendor/` resolves (`:27-120`).
- `RobotCollisionModel.from_file(urdf, tag="collision"|"visual")` loads meshes with trimesh. pycollada handles DAE and OBJ loads natively (`TG/robot_model.py:207-214`; deps in `REPO/core/python/alix_workspace/pyproject.toml`: numpy, scipy, pyyaml, trimesh, python-fcl, pycollada, networkx).
- Home pose: `load_initial_positions` merges each `*_description/config/initial_positions.yaml` (`WS/srdf_states.py:61-84`). For example, the RM arms sit at j2 −1.885, j4 1.798, j6 1.379, and L_j7 3.1416 (`DESC/rm_description/config/initial_positions.yaml:24-39`).
- The SRDF (`alix.srdf`, 3099 lines) is found by `find_srdf` in a sibling `*_moveit_config` (`WS/srdf_states.py:120-150`). It supplies `disable_collisions` and the group states.
- Environment note: importing `alix_workspace` with `/usr/bin/python3 -I` failed on this machine with `ValueError: numpy.dtype size changed` from scipy (observed). Lite needs its own venv.

### 2.4 How the Studio serves the robot to the browser
- **Geometry: `GET /api/robot-geometry?set=visual|collision`** (`TG/server.py:1918-1998`; route `:5977`).
  - Returns `{meshes:[{link, vertices, faces, color?}], homePose:{link: 16 floats row-major}, count, set}`. Vertices are in the LINK frame.
  - The visual set prefers a pre-baked `WS/workspace_data/visual_overlay_baked.npz` (2.9 MB; 200 submeshes, 100 links, 268,178 faces) (`:1875-1905, 1960-1970`). The bake is produced by `REPO/tools/meshes/bake_visual_overlay.py` (`:1-29`).
  - Links missing from the bake fall through to the live trimesh path (`:1971-1979`).
  - Collision-only links (handles, covers) are shown only in `set=collision` (`:1980-1984`).
  - No decimation is applied (`_OVERLAY_MAX_FACES = 0`), and the gzipped live payload is about 12 MB (`:1867-1872`).
- **Posing:** the client applies per-link 4×4 transforms that the server computed with `UrdfModel.link_world_transforms` (`WS/urdf_kinematics.py:314-344`). Sources for those transforms:
  - `/api/ik`
  - `/api/plan-frames` (`TG/server.py:4590-4674`, FK frames for plan playback, disk-cached)
  - `/api/dual-arm-collisions` (`:4335-4366`)
  - `/api/sim/pose` (`:5344-5393`)
- **Front end** (`TG/web_editor/editor.js`):
  - `loadRobotGeometry(set)` builds three.js meshes per link (`:12352-12395`). Visual colours are clamped to a matte look (`:12365-12378`).
  - `robotSetVisible` / `setRobotSet` handle the Visual/Collision toggles (`:12398-12421`; `template.html:727-728`).
  - `applyRobotPose(poseMap)` decomposes the row-major matrices onto meshes (`:12427-12437`).
- Libraries are vendored offline: three.js (2021 build, `REVISION "128"` string present), OrbitControls, TransformControls, js-yaml, and Plotly 2.35.2 (`TG/web_editor/vendor/`; `template.html:7`). The trajectory HTML loads the same Plotly 2.35.2 from a CDN (`/home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html:7`) and has **no 3D or robot view at all** (grep found no three/urdf/mesh).
- **Defect: the bake is stale for the bin.** The baked `bin_link` submeshes reach z 0.4382 (the old `Bin.obj` height; `BinShort` tops out at 0.3982). The bake file is dated 2026-08-15, which is before the BinShort change of 2026-08-29.
  - `bin_link` is present in the bake, so the server serves the OLD tall bin visual while collision uses `BinShort.stl` (`TG/server.py:1966-1970`).
  - The bake also lacks `camera_link`, which is filled from the live path (my parse: "visual links missing from bake: ['camera_link']").
  - Fix (inferred): re-run `bake_visual_overlay.py --urdf …/alix.urdf --out …/visual_overlay_baked.npz` (`REPO/tools/meshes/bake_visual_overlay.py:7-9`).

---

## 3. Commode fixture models, states, and washroom/wall meshes

### 3.1 Fixture registry
- `REPO/ros/orchestration/alix_tasks/config/commode_cleaning/fixtures.yaml:13-24`: the default is `commode_c`. `commode_a` is "Karcher SG" (articulated: false), `commode_b` is "Amsterdam" (articulated), `commode_c` is "Commode C" (articulated).
- `BodyCollisionWorld` treats only `commode_c` as validated against the live stack: `WORLD_VALIDATED_FIXTURES = frozenset({"commode_c"})` (`TG/body_collision.py:51-58`).

### 3.2 commode_c (the default; articulated)
- `…/commode_c/scene.yaml`:
  - Pose: `[1.005, 0.0, 0.42]`, rpy `[0,0,0]` relative to `base_link`, `default_ugv_standoff_m: 0.25` (`:32-42`).
  - Parts:
    - `body.stl` (`:44-46`)
    - `lid.stl` with hinge origin `[0.115,0,0.02]`, axis `[0,1,0]`, closed 0° / open −110° (`:48-57`)
    - `seat.stl` with the same hinge (`:59-68`)
  - `commode_classification: has_lid_and_seat` (`:27`).
- Meshes are in `…/commode_c/models/`. My STL parse gives:

| mesh | triangles | size | extents (m) |
|---|---|---|---|
| `body.stl` | 40,184 | 2.0 MB | x −0.004..0.535, y ±0.184, z −0.398..0.005 |
| `lid.stl` | 8,012 | 0.4 MB | origin at hinge |
| `seat.stl` | 8,000 | 0.4 MB | origin at hinge |

  Painted-region OBJs (`*_regions.obj`, `regions.yaml`, `corrected/`) sit beside them; they belong to the Studio region painter.
- States: `FIXTURE_TYPE_STATES = {"closed": lid closed/seat closed, "lid_open": lid open/seat closed, "lid_seat_open": both open}` (`TG/body_collision.py:103-108`). Configs carry `commode_fixture_type`, and the Studio maps it per part with `fixtureStatesFromType` (`TG/web_editor/editor.js:1167`; `:14321-14330`).
- Pose math lives in `TG/scene_pose.py`:
  - `_fixture_root_transform` = scene pose + standoff delta along +X, composed with the venue washroom rotation (`:123-143`).
  - `load_washroom_rotation` reads `config/scene/washroom_scene.yaml` (yaw 180) (`:93-120`).
  - `resolve_component_pose` rotates about the hinge axis by the state angle (`:161-214`).
  - The yaw-180 composition is what makes open lids lean TOWARD the robot. The live seat hinge was measured at x = 0.890 (`TG/body_collision.py:440-448`).
- Gazebo model: `DESC/alix_description/models/commode_c/model.sdf`. It holds revolute `lid_joint` / `seat_joint` about Y with limits −1.955..0.55 rad, partial −30° and open −110° (`:1-23, 159-196`), and is driven by `/commode_{lid,seat}_joint/cmd_pos` (`:203-230`). It is reference-only for Lite.

### 3.3 commode_a and commode_b
- **commode_a** (`…/commode_a/scene.yaml`):
  - `articulated: false` (`:18`), with three pre-baked whole-commode meshes, one per state: `closed.stl`, `lid_open.stl`, `lid_seat_open.stl` (`:49-51`).
  - **They are in millimetres**: `scale: [0.001,0.001,0.001]` (`:53`). My parse confirms x up to 710.
  - Pose: `[0.4764, 0, 0.4171]` (`:42`).
- **commode_b** (Amsterdam, `…/commode_b/scene.yaml`): body/lid/seat in metres, hinge origin around `[-0.43, ±0.003, 0.03..0.035]`, open −107° (`:46-87`). Gazebo SDF: `DESC/alix_description/models/commode_b/model.sdf:46-189`.
- Duplicates in `DESC/alix_description/models/meshes/`:
  - These are byte-identical to the `alix_tasks` twins: `closed_closet_sg_karcher.stl`, `lid_open_closet_sg_karcher.stl`, `lid_seat_open_closet_sg_karcher.stl` (commode_a, mm), `commode_Amsterdam.stl` (commode_b body), `urinal_sg.stl` (`REPO/core/python/src/alix_core/tests/test_description_assets.py:236-252`).
  - `lid_Amsterdam.stl` and `seat_Amsterdam.stl` are in metres. `washbasin.stl` is also present.

### 3.4 Washroom / wall meshes
- Venue scenes live in `REPO/ros/orchestration/alix_tasks/config/scene/` (`TG/body_collision.py:135-136, 177-204`). The default venue for the offline world is `washroom_800mm_wide_scene.yaml`. It places `washroom_800mm_planning_scene.stl` at `[1.215,0,0]` with yaw 180 (`…/scene/washroom_800mm_wide_scene.yaml:29-35`).
- Wall meshes in `DESC/alix_description/models/meshes/` (my STL parse):

| mesh | triangles | extents (m) | use |
|---|---|---|---|
| `washroom_800mm_planning_scene.stl` | 36 | x −0.103..1.884, y ±0.49, z 0..2 | 800 mm room |
| `washroom_1000mm_planning_scene.stl` | 36 | y ±0.6 | 1000 mm room |
| `washroom_800mm_wide.stl` | 144 | 4.26 × 8.1 m area | larger room |
| `washroom_1400mm_wide.stl` | 132 | — | used by `washroom_scene.yaml` at `[1.1,0.5,0]`, yaw 180 (`…/scene/washroom_scene.yaml:35-41`) |
| `washroom_2.stl` / `washroom_2_wider.stl` | 1500 | — | `washroom_2_wider` is listed as historically dangling (`test_description_assets.py:94-104`) |

- Walls load with `load_wall_manager(scene_yaml)`, which gives a trimesh `CollisionManager` with the structure posed in base_link (`WS/plan_scene_validation.py:117-155`).
- Two sources of truth to note: the fixture's washroom ROTATION comes from `washroom_scene.yaml` (1400 mm room), while the offline WALLS come from the 800 mm venue. Both are yaw 180, so they agree today (inferred risk).
- The fixture `flush_button` is captured as data only and has no collision object (`…/commode_c/scene.yaml:70-90`).

---

## 4. The emulator

### 4.1 What it is
- An **in-process Python stand-in for RealMan's RM_API2 Python SDK**, not a network server.
- Code: `REPO/core/python/src/alix_emulator/rm_emulator.py` (2616 lines). It is a verbatim port of Newton's upstream file and is pinned by `alix_core/tests/test_emulator_verbatim.py` (`REPO/core/python/src/alix_emulator/__init__.py:1-30`). The upstream copy is `REPO/external/butterfli-realman-alix/Demo/RMDemo_Python/RMDemo_DualArmConcept/src/rm_emulator.py`.
- `install()` puts fake `Robotic_Arm`, `Robotic_Arm.rm_robot_interface` and `Robotic_Arm.rm_ctypes_wrap` modules into `sys.modules`. It must run BEFORE the code under test imports the SDK (`rm_emulator.py:2541-2616`; `docs/EMULATOR.md:54-66`).
- It models two RM75-6FB / Gen-3 / V1.7.1 controllers at `RM_LEFT_IP` / `RM_RIGHT_IP`, defaulting to 192.168.1.10 and 192.168.1.103 (`rm_emulator.py:476-484`).
  - `rm_create_robot_arm` answers only those two IPs and ignores the port. Any other IP prints "socket connect err! (emulated)" and returns handle −1 (`rm_emulator.py:1936-1950`).
  - **So it cannot be selected "by IP/port". It is selected by which Python process called `install()`.**
- It imports the REAL vendored SDK first, for the offline `Algo` solver and ctypes structs, from `REPO/external/butterfli-realman-alix/Python` (`rm_emulator.py:82-112`). Vendored libs exist for `linux_x86`, `linux_arm`, `win_32` and `win_64` only.

### 4.2 What it supports (fidelity table `docs/EMULATOR.md:68-112`; API surface `:194-203`)
| capability | emulated? | evidence |
|---|---|---|
| connect/identity, arm state, joint degrees | yes | `rm_emulator.py:1936-2050` |
| `rm_movej` (synchronised joints at v% of 180 °/s, limits checked) | yes | `:2064`; `EMULATOR.md:73,87` |
| `rm_movel` / `rm_movec` (measured-law Cartesian) | yes | `:2080-2148`; `rm_emulator.py:37-55` |
| `rm_movej_canfd` (passthrough) | yes | `:2339` |
| pole lift `rm_set_lift_height` / `rm_set_lift_speed` / `rm_get_lift_state` (queued, measured speeds) | yes | `:2051, 2215-2232`; `EMULATOR.md:75` |
| Inspire hand `rm_set_hand_angle` / speed / force, Modbus registers | yes | `:2363-2443` |
| tool frames | yes | `:2149-2214` |
| run mode SIM/REAL | **stores the flag only; motion identical** | `:1988-1994`, `:815`; `EMULATOR.md:97-98` |
| fault injection (reject, fail at 60 %, drop event, latency, lift lock, power off) | yes | `EMULATOR.md:138-167`; `:555-563` |
| program upload / list / run by ID / run state / plan speed | **only through `alix_dispatch.emulated_program.ProgramSurface`** — the wrapper `alix_dispatch.connect.open_arm` attaches to emulated handles | `REPO/core/python/alix_dispatch/src/alix_dispatch/emulated_program.py:1-63, 313-428`; `connect.py:141-180` |
| drag teach / drag playback / save trajectory | **no** | no `rm_start_drag_teach`, `rm_save_trajectory`, `rm_run_drag_trajectory` in either layer (grep) |
| work frames, global waypoints, `rm_update_program_trajectory`, `rm_set_timeout`, `rm_set_arm_slow_stop` | **no** (`rm_set_arm_slow_stop` only via ProgramSurface, which delegates to stop) | grep; `emulated_program.py:514-518` |
| collision detection, force data, e-fence, joint-speed limits, joint-rate prediction | **no** | `rm_emulator.py:56-70, 987`; `TG/sim_bridge.py:40-48` |

- The fake program channel (`emulated_program.py`):
  - `parse_project_file` enforces the controller format: CRLF line ends, then `{"file":7}`, then a `Folder` line whose `type` is the point count, then `{"point":[7 ints]}` lines (`:128-173`).
  - Program names are stored as `f"{id}_{stem}.txt"`, cut to 32 bytes (`:328-345`).
  - `rm_set_program_id_run` performs a blocking transit `movej` at the run speed, dwells `DWELL_S=0.35`, then streams each point at `REPLAY_CLOCK_S=0.0055545` s (`:75-80, 396-485`). The run speed becomes a persistent global ratio (`:409-410`).
  - The same clock is documented in `alix_dispatch/track.py:44-51` (measured 5.51–6.02 ms/pt on hardware).

### 4.3 How to start it, and OS constraints
- Inside a Python process: `from alix_emulator import rm_emulator; rm_emulator.install()` (`__init__.py:26-46`).
  - Optional `set_time_scale(N)` or the `RM_EMU_TIME_SCALE` env var (`rm_emulator.py:31-32, 518`).
  - Initial state: left arm J7=180°, right arm at zero, lifts at 100 mm (`rm_emulator.py:1899-1903`; `EMULATOR.md:183-192`).
- With the Studio: `python3 tools/run_studio.py --emulator` installs the emulator, connects both arms in the same process, and enables the **Simulate** view (`REPO/tools/run_studio.py:1-25, 142-190`).
  - The view polls `/api/sim/status` and `/api/sim/pose` at 2/5/10 Hz with "Go live" (`TG/web_editor/editor.js:19200-19325`; `template.html:63, 87-101`).
  - `sim_bridge` reads the emulator's `_by_ip` registry read-only and converts units: deg→rad, lift mm→m, hand counts→rad (`TG/sim_bridge.py:1-49, 112-186`).
  - `/api/sim/joints` also accepts an external joint map ("future: the real arms") (`TG/server.py:5326-5341`).
- **OS: Linux (x86_64/aarch64) and Windows only. Not macOS.** The vendored SDK ships no `.dylib`, and `rm_ctypes_wrap` raises a `NameError` on darwin (`docs/EMULATOR.md:14-44`; `tools/run_studio.py:27-50, 92-120`).
- The repo-level validation ladder (contract C9) is **emulator → SIM (the controller's own simulation run mode, needs hardware) → REAL** (`REPO/core/python/alix_dispatch/src/alix_dispatch/runmode.py:1-7`).
  - The control application already implements the SIM/REAL switch per arm: the v53 UI SIM/REAL/CHECK buttons (`UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py:2080-2101, 3785-3815`) call the runner's `--get-arm-mode` / `--set-arm-mode simulation|real` (`UC/rm_upload_fast_v14_safe_transit.py:1213-1260, 2318-2320, 2621-2625`).
  - In controller SIM mode the arms report self-collision, payload self-collision and electronic fence as "simulation-mode only" capabilities (`DESC/rm_description/docs/arm_version_info.md:29-35`).

### 4.4 Could Lite's SIM toggle send the same runner commands to the emulator?
**Not by IP/port.** It is possible with a small shim, but the coverage is partial (inferred design, based on evidence):
1. The runner loads the SDK through `load_sdk(--sdk DIR)`. It tries `--sdk`, then an installed copy, then the repo copy (`UC/rm_upload_fast_v14_safe_transit.py:60-73`), and `README_COMMANDS.md` sets `ALIX_RM_SDK` to the monorepo's vendored SDK. A shim directory passed as `--sdk` could install the emulator. The shim must also:
   - (a) export `rm_send_project_t`, which the runner imports at load time (`:67`). `install()` does not provide it (`rm_emulator.py:2581-2611`); it is only patched in by `emulated_program.attach` (`emulated_program.py:537-546`).
   - (b) wrap each `RoboticArm` with `ProgramSurface`, because the runner calls `rm_create_robot_arm` directly (`:2607-2611`) instead of going through `alix_dispatch.connect.open_arm`, which is what normally attaches it (`connect.py:175-180`).
2. Runner features that would work under emulation:
   - upload, verify, list (`slots`, `:105-120`), run and sequence (program verbs plus `rm_get_current_arm_state`, `:122-140`)
   - workflow pole/hand steps (`UC/pole.py`, `UC/hand.py` use lift and Modbus verbs, which are emulated)
   - `rm_set_timeout`, which is `hasattr`-guarded (`:2613-2614`)
3. Runner features that would fail with `AttributeError`:
   - drag-teach and drag playback (`:923-968, 1182-1201`)
   - program rename `rm_update_program_trajectory` (`:1113`)
   - global waypoints (`:1325, 1395`)
   - work frames (`:1435-1502`)
4. Even with the shim, the emulator answers "does the program/upload/sequence logic run", not "does the arm collide". It has no collision model and no joint-rate prediction (`rm_emulator.py:56-70`).
   - Its arm state lives in the runner's subprocess, so a Lite 3D view cannot see it unless the shim POSTs joint maps to a `/api/sim/joints`-style endpoint (`TG/server.py:5326-5341`; `run_studio.py:22-25` explains why same-process matters).
5. Recommendation (inferred): for non-technical users, "SIM" should mean the **offline 3D preview with an FCL collision sweep** (no SDK, any OS). Offer the controller's own SIM run mode as "robot connected, arm stays still", and make the emulator an optional developer dry-run.

---

## 5. Isaac Sim — what exists, and relevance to Lite
- `REPO/ros/simulation/isaacsim/` loads the full robot into **Isaac Sim 6.0** with the GUI. Options include `--camera` (RTX cameras), `--fixture` / `--fixture-physics` (commode as PhysX bodies on hinges) and `--soil` / `--sdg` (synthetic datasets) (`README.md:1-72`; `run.sh:1-32`).
- It needs `$ISAACSIM_ROOT/python.sh` (default `~/isaacsim`) (`run.sh:24-32`) and an NVIDIA RTX GPU; measurements were taken on an RTX 4070 Laptop (`README.md:557-575`). The full stack runs via `ros2 launch alix_bringup alix_isaac_sim_hardware.launch.py` (`README.md:22-27`).
- `generated/` is git-ignored (`.gitignore`) and takes 4.0 GB (3.9 GB of it SDG datasets). The pipeline's `stage_assets.py` writes `generated/staging/alix_expanded.urdf`, which proves the committed flat URDF is current (§2.1).
- Peripherals (bin and glove arm) are driven only by ROS topics (`README.md:604-622`).
- **Not relevant to Lite** beyond that cross-check. It is heavyweight, GPU-bound, ROS-coupled and not suited to non-technical operators. Gazebo is likewise ROS-coupled (`DESC/alix_description/launch/alix_gazebo.launch.py`; README `:19-29`).

---

## 6. Reuse assessment: Lite's 3D view, collision world, and bin toggle

### 6.1 3D view (R10 left pane; R3/R11/R14 playback)
- **Model:** reuse the committed flat URDF (§2.1) as a frozen asset. No xacro or ROS is needed.
- **Geometry delivery (recommended):** lift the Studio's server-side pipeline: `do_robot_geometry` plus the URDF/visual/collision part of `_ik_context` (`TG/server.py:1918-1998`; `TG/robot_model.py:170-228`).
  - Raw visual meshes total about 81 MB, and the D435 alone is 15.8 MB. Loading them client-side with three.js STL/DAE/OBJ loaders is impractical (inferred). The bake brings them down to about 268k faces / 2.9 MB.
  - **Re-bake first** (stale bin, missing camera; §2.4).
  - Drop the reachability, manipulability, SRS and atlas parts of `_ik_context`. Those are C-space/workspace features excluded by X2 (`TG/robot_model.py:89-110, 282-306`).
- **Client:** lift `loadRobotGeometry`, `meshFromPayload`, `applyRobotPose`, the Visual/Collision toggles and the matte-material clamp (`editor.js:12346-12437`), plus vendored three.js r128 and OrbitControls (`TG/web_editor/vendor/`).
- **FK source:** server-side `UrdfModel.link_world_transforms` (`WS/urdf_kinematics.py:314-344`), as in `/api/plan-frames` and `/api/sim/pose`, so the 3D view and the collision checker share one FK. The Studio comment says the same: "A second FK implementation is how two views of 'the robot' start disagreeing" (`TG/server.py:5300-5303`).
  - Lite input mapping (inferred):
    - `.txt` millidegrees /1000 → degrees → radians onto `L_jointN` / `R_jointN`
    - pole mm /1000 → `*_sliding_plate_joint` (from the workflow JSON `{"type":"pole","mm":…}`, `UC/LEFT_V6_70SERIES_FINAL.json`)
    - hand 0..1000 → `HAND_CHANNELS` (`TG/sim_bridge.py:93-102`; clamp to URDF limits)
    - time base: point index × `REPLAY_CLOCK_S` (`alix_dispatch/track.py:44-51`)
- **Live mirroring (R4):** copy the Studio Simulate pattern, `/api/sim/joints` + `/api/sim/pose` + the polling view (`TG/server.py:5309-5393`; `editor.js:19259-19296`). During a REAL or controller-SIM test, the runner's `arm_state()` (joint degrees plus error codes, `UC/rm_upload_fast_v14_safe_transit.py:122-140`) can feed it (inferred).

### 6.2 Collision world (R3/R5/R10)
- **Engine:** `alix_workspace.dual_arm_plan_collision.compute_dual_arm_plan_collisions` (`WS/dual_arm_plan_collision.py:586-760`). Per sample it checks:
  - left vs right arm
  - self-collision per arm
  - arms vs a slim body set `("bin_link","glove_wearing_unit_arm")` (`:644-648`)
  - arms vs any `extra_managers` (`:699-703`)

  It returns `collisions[{index,time,pairs}]`, `windows` and per-sample `frames`. SRDF `disable_collisions` pairs are filtered (`:112-123, 678-684`). These outputs are exactly what R5 "stop the graph at the collision and highlight" needs. The Studio already renders them in its Dual-Arm Collisions tab: a Plotly bar timeline coloured red on hits, click-to-scrub, red link highlight and jump-to-next-hit (`editor.js:14478-14560`).
- **Adaptations needed (inferred):**
  1. Input is plan JSON `sub_trajectories[].waypoints[].{time_from_start, positions}` (`:46-87`), and the Studio endpoint only accepts `.json` under its plan roots (`TG/server.py:4187-4197`). Lite needs a `.txt`→timeline adapter or an in-memory variant.
  2. Pass commode and walls as `extra_managers`. Build them from `BodyCollisionWorld.fixture_manager(fixture, fixture_type)` (`TG/body_collision.py:401-464`) and `load_wall_manager` (`WS/plan_scene_validation.py:117-155`). The Studio's DAC endpoint does not pass them today (`TG/server.py:4354-4356`).
  3. Make the body-link set and the bin state parameters. Today they are hard-coded to `bin_link` + glove lever (`:644`) and `BIN_LOWERED` (`:434`).
  4. Set the pole per region, not the default `full_length` 0.30. `_build_home` documents the 150–225 mm vertical errors that come from forgetting this (`:410-445`).
  5. Pass the hand state, because the open-hand default changes verdicts (`:398-412`).
- Dependencies: trimesh, **python-fcl (required, no fallback)**, pycollada and networkx (`REPO/core/python/alix_workspace/pyproject.toml`). These are pure pip, so it works on any OS (inferred).
- Alternative or richer world: `TG/body_collision.BodyCollisionWorld` (`:130-464`). It holds the full robot (bin, covers, UGV and poles as statics), walls and the articulated fixture, with an FK-derived moving/static split (`:251-312`). It is single-arm-pose oriented (`make_pose_filter`, `:499-533`) and thread-locked (`:149-172`). Treat it as reference or a helper source for the fixture manager.

### 6.3 Bin toggle (R10): URDF variants vs runtime
- **URDF variants: not available without new xacro work.** There is no bin arg (`ugv.xacro:27-29`; `alix.urdf.xacro:116,135`). Adding `include_bin` would need a xacro edit, ROS to re-expand (`expand_urdf.sh:58-64`), and a second committed flat URDF. It would also break the "one kinematic ground truth" rule (`expand_urdf.sh:4-8`) and the content-hash caches (`:84-85`).
- **Runtime approach (recommended, inferred):**
  1. Treat the "bin set" as `{bin_rail_link, bin_carriage_link, bin_link, bin_handle_left, bin_handle_right, lid_opening_cover1, lid_opening_cover2}`.
  2. **Visibility:** hide or show `robotMeshes[link]` for those links in three.js (the pattern is `setRobotSet`, `editor.js:12404-12421`).
  3. **Collision exclusion:** remove those links from the body set or statics. Pass `include_body=False` if the glove lever can go too, or better add a `body_links` parameter (`WS/dual_arm_plan_collision.py:595, 644-648`). For `BodyCollisionWorld` statics, add a bin filter to the `build(lambda l: …)` predicate (`TG/body_collision.py:296-298`) (inferred).
  4. **Optional state selector (STOWED/EXTENDED/LOWERED):** set `bin_rail_joint` / `bin_drop_joint` in the home map from `peripheral_joints.yaml:43-46` or the SRDF `BIN_*` states, instead of the hard-coded LOWERED.
  5. Warn about the STOWED pole interlock with `pole_bin_conflict` (`task_config.py:97-125`).
- Fidelity caveat: the model bin is BinShort (366 mm) while the physical bin may still be 406 mm (`bin.xacro:32-34`). An "include bin" sim could under-report contact by up to 40 mm at the rim.

---

## Requirement relevance
- **R3 Simulation playback:**
  - Reuse the Studio overlay and server FK (§6.1) and the dual-arm FCL sweep with commode and walls (§6.2).
  - The trajectory HTML has no 3D view (§2.4), so the 3D view must be borrowed from the Studio, as the requirement itself says.
- **R4 Real/Sim toggle and IP entry:**
  - Controller SIM/REAL already exists (v53 UI buttons → runner `--set-arm-mode`; §4.3).
  - The emulator is in-process and not IP-addressable (§4.1, §4.4).
  - Arm IP defaults live in `alix_core/sdk/endpoints.py` (`LEFT_IP` 192.168.1.10, `RIGHT_IP` 192.168.1.103, `HOST_IP` 192.168.1.239, `ROBOT_PORT` 8080, `UDP_PORT` 8089; `REPO/core/python/src/alix_core/sdk/endpoints.py` lines 101–130). The xacro mirrors them with UDP target .235 (`alix.urdf.xacro:26-31`). This host-IP mismatch matters only if UDP push is used.
  - Recommended 3-way toggle: 3D Preview / Controller SIM / REAL (inferred).
- **R5 Collision highlight:**
  - Sim: `collisions[{index,time,pairs}]` from §6.2 plus DAC `dacRenderPlot` / `dacGoto` / `dacHighlight` / `dacJumpHit` (`editor.js:14478-14560`).
  - Real: the runner flags error 4109 "ARM COLLISION" (`UC/rm_upload_fast_v14_safe_transit.py:56-57`) along with live joint degrees (`:122-140`). Nearest track point gives the graph index (inferred). The left arm's dynamics collision detection is level 3; the right arm's is 0/off (`arm_version_info.md:29`).
- **R7 Pole height / hand position:**
  - Pole travel is 0..0.315 m (`pole.xacro:63`), with SRDF presets (§1.2) and lift mm = URDF metres ×1000.
  - Hand presets come from the SRDF (§1.4) and the 0..1000 channel mapping (`sim_bridge.py:93-102`).
  - Check the bin/pole interlock band 0.153–0.282 m when STOWED (`task_config.py:85-125`). `do_joint_presets` is the Studio's SRDF-driven pole dropdown (`TG/server.py:1763-1786`).
- **R8 TXT merging with the commode scene:** commode_c meshes, states and pose math (§3.2) and walls (§3.4) form the collision scene the merge/bridge check should use. The default validated venue is 800 mm (`TG/body_collision.py:135-136`).
- **R9 / R14 Offset via FK→IK:**
  - Use ONE model for FK and IK: the URDF (`UrdfModel`) or the vendor `Algo` used by the emulator (`rm_emulator.py:82-112`).
  - Mind the 2.8 mm flange note (`rm_75_macro.xacro:411-424`), the ConnectorLink→Arm_Tip 15.3 mm (`rm_emulator.py:143-149`), and that the controller frame is `Ry(90°)` of the URDF base and depends on pole height (`alix_dispatch/frames.py:3-38`).
  - Check URDF joint limits (§1.3). Simulate the offset trajectory in the §6.1/§6.2 pipeline before running it on the real arm.
- **R10 Dual-arm view, correct URDF and meshes, bin toggle:**
  - The correct URDF is `WS/urdf/alix.urdf` (verified current, §2.1). The meshes are as in §1.6.
  - Re-bake the overlay (stale bin).
  - Bin toggle via the runtime approach (§6.3).
  - The DAC tab's left-3D / right-Plotly-timeline layout is the closest existing UI.
- **R11 Immediate validation of smoothing:** server FK plus a cached sweep. The Studio caches by content hash (`TG/server.py:4303-4316, 4648-4673`), so re-running after each edit is cheap for unchanged pairs (inferred). Sweep cost scales with samples, controlled by `dt` (0.05 default).
- **R13 Faster upload:**
  - The emulator's program double documents the controller project format (CRLF, `{"file":7}`, Folder `type` = point count), the 32-byte name limit and the 300-byte path limit (`emulated_program.py:81-87, 128-173`).
  - It can unit-test an upload flow offline (Linux/Windows).
  - The REPLAY clock is 5.5545 ms/pt (`track.py:44-51`).
- **X1/X2:** exclude the reachability, manipulability, redundancy-atlas and coverage parts of `_ik_context` / `server.py` when lifting (`TG/robot_model.py:89-110, 282-306`; `TG/server.py:750-801, 2001-2080`).
- **NFR-UX:**
  - Default "SIM" should be the offline 3D preview (no hardware, any OS).
  - Hide collision-only links unless the Collision toggle is on (`TG/server.py:1980-1984`).
  - Plain-language status like the Studio Simulate view's dot/hint (`editor.js:19218-19247`).
  - The REAL-mode confirmation dialog already exists in v53 (`UC/…v53…py:3803-3810`).
- **NFR-THEME:** the Studio's three.js look (matte clamp, `0x9fb3c8` robot, `0xff4d4d` hit, `0xff8c42` collision proxy; `editor.js:12364-12378, 12446-12449`) and the dark Plotly layout (`editor.js:14483-14495`) can be lifted as-is.
- **INT-CTRL:** runner `--sdk` hook for a possible emulator shim (§4.4); existing `--get-arm-mode` / `--set-arm-mode` (§4.3); `arm_state()` for a live mirror.
- **INT-TRAJ:** the trajectory HTML uses the same Plotly 2.35.2 as the Studio (`Alix_Studio_Lite_Trajectory_V2.html:7`; vendored `plotly.min.js` header), so plots and the 3D view can share one page. Vendor Plotly locally for offline use, as the Studio does (`template.html:7`) (inferred).

## Reuse verdicts
| feature | verdict | what to lift | evidence |
|---|---|---|---|
| Flat URDF `alix.urdf` | reuse-as-is | copy or reference `WS/urdf/alix.urdf` (196,666 B) as a frozen asset | `expand_urdf.sh:2-8,32`; identical to Isaac fresh expansion (§2.1) |
| xacro sources + `expand_urdf.sh` | reference-only | only when the description changes; needs ROS Jazzy | `expand_urdf.sh:58-79` |
| Robot meshes (5 description pkgs + vendor `d435.dae`) | reuse-as-is (bundle) | visual 81 MB / collision 6.4 MB; `PackageIndex` to resolve | §1.6; `package_index.py:27-120` |
| Baked visual overlay `.npz` | reuse-with-adaptation | re-run `bake_visual_overlay.py` (bin stale, camera missing) | `server.py:1875-1970`; my npz parse |
| `/api/robot-geometry` + `_ik_context` subset | reuse-with-adaptation | `do_robot_geometry`, `_mesh_payload`, `_load_baked_visual`; drop X2 parts | `server.py:1864-1998`; `robot_model.py:170-228` |
| editor.js robot overlay | reuse-with-adaptation | `loadRobotGeometry`, `meshFromPayload`, `setRobotSet`, `applyRobotPose` | `editor.js:12346-12437` |
| three.js r128 + OrbitControls + Plotly 2.35.2 (vendored) | reuse-as-is | `TG/web_editor/vendor/*` | `template.html:7` |
| `UrdfModel` FK (mimic-aware) | reuse-as-is | `WS/urdf_kinematics.py` | `:1-40, 294-344` |
| `RobotCollisionModel` (trimesh/FCL) | reuse-as-is | `WS/collision.py` | `robot_model.py:207-214`; `pyproject.toml` deps |
| Dual-arm FCL sweep | reuse-with-adaptation | `compute_dual_arm_plan_collisions` / `compute_plan_frames`; add txt adapter, body-link and bin-state params, commode and walls managers | `dual_arm_plan_collision.py:46-87,385-435,586-760` |
| DAC timeline + highlight UI | reuse-with-adaptation | `dacRenderPlot`, `dacGoto`, `dacHighlight`, `dacJumpHit`, `dacPlay` | `editor.js:14478-14600` |
| Commode_c scene + meshes + pose math | reuse-as-is | `scene.yaml`, `models/{body,lid,seat}.stl`, `scene_pose.resolve_component_pose`, `FIXTURE_TYPE_STATES` | `commode_c/scene.yaml:32-68`; `scene_pose.py:93-214`; `body_collision.py:103-108` |
| Fixture collision manager | reuse-with-adaptation | `BodyCollisionWorld.fixture_manager` (or extract) | `body_collision.py:401-464` |
| Walls (800 mm venue) | reuse-as-is | `washroom_800mm_wide_scene.yaml` + `load_wall_manager` | `…/scene/washroom_800mm_wide_scene.yaml:29-35`; `plan_scene_validation.py:117-155` |
| Commode Gazebo SDFs | reference-only | hinge limits and state angles only | `models/commode_c/model.sdf:159-196` |
| Bin include/exclude via URDF variant | not-available | no xacro arg; would need new arg + ROS re-expansion | `ugv.xacro:27-29`; `alix.urdf.xacro:116,135` |
| Bin toggle at runtime (visibility + collision exclusion + state) | reuse-with-adaptation | link set from `bin.xacro`; states from `peripheral_joints.yaml` / SRDF | `bin.xacro:122-402`; `peripheral_joints.yaml:40-50`; `alix.srdf:286-297` |
| Pole/bin interlock | reuse-as-is | `pole_bin_conflict` for R7 warnings | `task_config.py:85-125` |
| SRDF (disable pairs, group states) | reuse-as-is | `alix.srdf` via `find_srdf` / `load_disabled_pairs` / `SrdfStates` | `srdf_states.py:120-150`; `dual_arm_plan_collision.py:112-123` |
| `initial_positions.yaml` home pose | reuse-as-is | `load_initial_positions` | `srdf_states.py:61-84` |
| Studio Simulate view + `/api/sim/*` + `sim_bridge` | reuse-with-adaptation | live mirror for emulator or real arms (`POST /api/sim/joints`) | `server.py:5309-5393`; `sim_bridge.py:1-208`; `editor.js:19200-19325` |
| RM emulator | reuse-with-adaptation (optional dev dry-run) | `alix_emulator.install()` + an SDK shim for the runner; Linux/Windows only | `rm_emulator.py:2541-2616`; `EMULATOR.md:14-44` |
| Emulated program channel | reuse-with-adaptation | `ProgramSurface` / `attach` / `parse_project_file` | `emulated_program.py:128-547` |
| Controller SIM run mode | reuse-as-is | runner `--get-arm-mode` / `--set-arm-mode`; v53 SIM/REAL/CHECK buttons; `alix_dispatch.runmode` | `rm_upload_fast_v14_safe_transit.py:1213-1260`; `v53:2080-2101`; `runmode.py:1-53` |
| Isaac Sim | not-applicable | — | `isaacsim/README.md:1-27,557-575`; `run.sh:24-32` |
| Gazebo / RViz launches | not-applicable | — | `alix_description/README.md:19-36` |

## Gaps and risks (with evidence)
1. **Stale baked overlay.** The baked `bin_link` is the 406 mm `Bin.obj` (z-max 0.438), but BinShort tops out at 0.398. The bake (2026-08-15) predates BinShort (2026-08-29) and lacks `camera_link`. The visual set would show a tall bin while collision uses the short one (`server.py:1966-1970`; my npz parse).
2. **Model ahead of hardware.** `bin.xacro:32-34` says the physical bin has not been remade. The sim bin may be 40 mm shorter than the real one.
3. **Hard-coded assumptions** in the reusable code: bin LOWERED, poles `full_length`, body set = `bin_link` + glove lever, open hands by default (`dual_arm_plan_collision.py:398-445,644`; `server.py:1763-1768,1986-1992`).
4. **The Studio sweep ignores commode and walls** (no `extra_managers` at `server.py:4354-4356`), and it only accepts plan JSON under plan roots (`server.py:4187-4197`).
5. **The emulator is not a network endpoint.** The runner shim needs `rm_send_project_t` and the `ProgramSurface` wrap. Drag-teach, waypoint, work-frame and rename verbs are missing. macOS is unsupported. There is no collision model and no joint-rate prediction (§4).
6. **Stale emulator doc limits.** `EMULATOR.md:87` vs `rm_emulator.py:418-432`. URDF and controller limits differ, and the URDF is stricter (§1.3).
7. **Small FK offsets.** The 2.8 mm flange note and the 15.3 mm ConnectorLink→Arm_Tip offset mean mixed FK/IK sources can introduce offsets of several mm (`rm_75_macro.xacro:411-424`; `rm_emulator.py:143-149`).
8. **Heavy raw meshes.** 81 MB visual; d435 is 15.8 MB and comes from outside the repo (`vendor/realsense-ros`). A standalone Lite bundle must include it or drop the camera.
9. **Python environment.** `python-fcl`, `trimesh` and `pycollada` are required. The system Python here has a numpy/scipy ABI mismatch (observed), so use a venv.
10. **Hand display mapping** exceeds URDF finger limits, and FK does not clamp (`sim_bridge.py:95-102` vs `RH56DFX_L_8_19.xacro:188,217,419`).
11. **Two washroom sources.** The fixture rotation comes from `washroom_scene.yaml` (1400 mm room) while walls come from the 800 mm venue. Both are yaw 180 today (`scene_pose.py:104`; `body_collision.py:136`).
12. **Pole/bin interlock.** The 70-series workflow pole of 200 mm is inside the STOWED foul band. The sim verdict depends on the bin state Lite assumes (`task_config.py:85`; `UC/LEFT_V6_70SERIES_FINAL.json`).

## Open questions
1. Has the physical bin been shortened 40 mm? Should the sim use BinShort or the original `Bin.stl`/`Bin.obj` (both still on disk)?
2. During real cleaning, which bin state applies (STOWED/EXTENDED/LOWERED)? Should the R10 toggle be include/exclude only, or a 3-state selector?
3. Which fixture, commode state per region (closed/lid_open/lid_seat_open) and venue (800 mm?) should Lite assume? The 70-series workflow includes a manual lid-cover opening wait.
4. What should "SIM" mean for Lite users: the offline 3D preview, the controller SIM run mode, the emulator dry-run, or all three?
5. Which OS will Lite run on? The emulator excludes macOS; the offline 3D preview does not.
6. Should Lite depend on the monorepo (`pip install -e alix_workspace`, plus `alix_taskgen` helpers), or freeze a copy of URDF + meshes + modules into the Lite workspace?
7. Should simulation time use `REPLAY_CLOCK_S` (5.5545 ms/pt) per point, and does the controller's program speed % change replay timing? This is not established in my sources.
8. Should hands be animated from the workflow JSON hand steps, or held at a fixed SRDF preset (e.g. `grasp`) for collision?
