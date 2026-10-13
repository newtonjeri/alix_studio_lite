# 05 — Workspace kinematics & collision (alix_workspace + friends) — evidence map

Scope: how ALIX Studio Lite can implement R9/R14 (FK -> Cartesian offset -> IK with continuity),
R8 (collision-aware .txt merging), R5 (collision detection on joint trajectories) and R10 (bin toggle)
using the existing ALIX kinematics/collision code.

Path abbreviations used in every citation:

| Tag | Absolute path |
|---|---|
| `WS/` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_workspace/src/alix_workspace/` |
| `WSPKG/` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_workspace/` (README, docs, scripts) |
| `TG/` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/` |
| `REPO/` | `/home/newtonjeri/alix_ws/src/alix/` |
| `EMU` | `REPO/core/python/src/alix_emulator/rm_emulator.py` |
| `SDK/` | `REPO/external/butterfli-realman-alix/Python/Robotic_Arm/` (RealMan RM API2 + offline `Algo`) |
| `TL/` | `REPO/experiments/exp01_teach_trajectory_reverse_engineering/src/teachlab/` |
| `URDF` | `WS/urdf/alix.urdf` |
| `SRDF` | `REPO/ros/manipulation/alix_moveit_config/config/alix.srdf` |
| `LITE/` | `/home/newtonjeri/alix_studio_lite_ws/` |
| `V6/` | `LITE/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/` |

`[M]` = measured by this agent on 2026-10-13 with inline `python3 -B -c` (no files written, no
robot/network), using the user-site installs (`~/.local/lib/python3.12/site-packages`: trimesh,
python_fcl 0.7.0.11, pycollada, editable alix_workspace/alix_taskgen) and real Lite tracks from `V6/`.
`(inferred)` = my conclusion, not stated in the source.

---

## 0. Headline answers (one screen)

1. **FK is solid and fast, and matches the controller.** `UrdfModel.fk/fk_batch` (WS/urdf_kinematics.py:218-259)
   over `URDF` reproduces controller TCP to 0.007 mm median when fed SDK joints 1:1 (TL/kinematics.py:3-6).
   [M] vendor `Algo.rm_algo_forward_kinematics` vs URDF Arm_Tip: max 0.0001 mm over R2_LIDTOP samples.
   [M] `fk_batch` 3751 points in 9.4 ms (2.5 us/pt); scalar `fk` 32 us/pt.
2. **Three IK engines exist; none gives jump-free offsets on drag-taught tracks out of the box.**
   (a) numerical DLS `IKSolver` (WS/ik.py:150-323), (b) closed-form S-R-S `srs_ik.solve/solve_all`
   parametrised by swivel (arm) angle (WS/srs_ik.py:349-391), (c) vendor `Algo` seeded / arm-angle IK
   (SDK/rm_robot_interface.py:6396-6504; EMU:364-410). [M] On real tracks with a 10 mm offset every
   engine produced some >10 deg adjacent steps or failures, because the taught tracks pass through
   vendor singular sets (q4~0, q2~0, q6~0: R2_LIDTOP min singular margin 0.00 deg, 23.6 % of samples
   within 10 deg). A continuity-engineered algorithm is required (section 8).
3. **Joint limits:** source of truth `REPO/ros/description/rm_description/urdf/rm_75_macro.xacro:201-430`
   (+-178/130/178/135/178/128/360 deg), expanded into `URDF`; vendor Algo defaults are identical [M];
   real-arm DRIVE ranges are wider and per-side (EMU:415-436). Use URDF limits minus a 5 deg allowance
   (REPO/tools/taught_tracks.py:122-126; REPO/tools/verify_task_motion.py:94-97).
4. **Collision = trimesh `CollisionManager` over python-fcl (required, no fallback)**
   (WSPKG/pyproject.toml:12-20; WS/collision.py:512-526). The richest world is
   `alix_taskgen.body_collision.BodyCollisionWorld` (robot self + partner arm + hands/envelopes + bin &
   covers + UGV/poles + venue walls + commode body/lid/seat at an articulation) with a per-pose filter
   `filter_fn(q_deg7) -> ["A<->B", ...]` and `.depths()` (TG/body_collision.py:499-952).
   [M] world+filter build 2.6 s, per-pose query median 1.96 ms / p95 4.46 ms.
5. **Whole-trajectory sweeps exist but take plan JSON, not .txt:** `compute_dual_arm_plan_collisions`
   (WS/dual_arm_plan_collision.py:586-760), `cross_arm_separation` (1089-1283), `PlanWallChecker.check`
   (WS/plan_scene_validation.py:219-256). The production .txt->plan shim already exists in
   `REPO/tools/dual_arm_verdicts.py:501-731` (millideg x pi/180000, clock 0.0055545 s, pole+hand seeding).
6. **Bin:** URDF links `bin_link` (+ `bin_handle_left/right`, `lid_opening_cover1/2`) on prismatic
   `bin_rail_joint` (0-0.2 m) and `bin_drop_joint` (0-0.45 m) (URDF:222-247; [M] link tree). Every
   world poses it `BIN_LOWERED` (SRDF:294-297; WS/dual_arm_plan_collision.py:434;
   TG/body_collision.py:629). **No bin toggle exists in Studio UI** ("Bin is always LOWERED",
   TG/web_editor/template.html:753). Backend: `include_body=False` exists only in
   `compute_dual_arm_plan_collisions` (WS/dual_arm_plan_collision.py:595,647-648) and is not exposed by
   the endpoint (TG/server.py:4335-4364). R10 toggle = small new code.
7. **Pole** is the prismatic `{L,R}_sliding_plate_joint` (0-0.315 m, +Z) upstream of each arm base;
   arm base z = 0.6485 m + pole [M]. SDK lift mm / 1000 = joint metres (REPO/ros/hw/alix_hw/doc/TECHNICAL_INTERFACE.md:74).
   Defaults are traps: `_build_home` seeds 0.300 m (WS/dual_arm_plan_collision.py:409-424,435).
8. **Standalone use is feasible** (pure Python, no ROS) with a checkout of the ALIX repo (URDF, 80
   collision STLs = 6.4 MB [M], SRDF, venue/fixture YAML + STLs, repo-root marker files). FCL managers
   are not thread-safe (TG/body_collision.py:148-171) -> one collision worker.

---

## 1. Forward kinematics

### 1.1 Engine and API
- `UrdfModel` — stdlib XML parse, numpy FK; joint types revolute/continuous/prismatic/fixed + `<mimic>`;
  xacro must be pre-expanded (WS/urdf_kinematics.py:1-22, 137-181).
  - `UrdfModel.from_file(urdf_path)` / `from_string(xml)` (WS/urdf_kinematics.py:137-145).
  - `chain(base_link, tip_link) -> List[Joint]` (195-213); `movable_joints(chain)` (215-216).
  - `fk(chain, q, base_transform=None) -> 4x4`; q = movable-joint values IN CHAIN ORDER (218-229).
    For `base_link -> R_glove_frame_2` the movable list is
    `[R_sliding_plate_joint, R_joint1..R_joint7]` [M] — i.e. the pole is q[0].
  - `fk_batch(chain, Q (N,k)) -> (N,4,4)`, bit-identical to `fk`, "~17x faster on a 12k-line track" (231-259).
  - `fk_frames(chain, q)` -> tip + per-joint pre-motion frames for the Jacobian (261-279).
  - `link_world_transforms(joint_values: Dict[str,float], base_transform=None) -> {link: 4x4}`
    whole tree, unspecified joints = 0, mimics resolved (294-343). This is what collision/overlay use.
  - Performance engineering: cached read-only origin matrices (78-96), closed-form Rodrigues motion (98-126).
- Thin wrappers worth knowing:
  - `SrsArm.chain_frames(q7, pole)` / `tool_pose(q7, pole)` — chain-only FK with the pole as an
    argument (WS/srs.py:522-542).
  - `teachlab.kinematics.ArmKinematics(side, frame, urdf=None, frame_offset_m=None)` with `fk`, `fk_batch`
    (positions, rotations), `jacobian`; frames `Arm_Tip`, `ConnectorLink`, `glove_frame_1..4`, `tip_frame`,
    `index_tip_frame` (TL/kinematics.py:139-224). Lives under `experiments/` (production code must not
    import it per REPO/tools/kinematics/check_kinematics_sources.py:28-34), but `taught_tracks.py`
    already imports it (REPO/tools/taught_tracks.py:649).
  - Vendor: `Algo.rm_algo_forward_kinematics(joint_deg, flag=1) -> [x,y,z,rx,ry,rz]` (m, rad, Euler)
    (SDK/rm_robot_interface.py:6506-6550); offline construction `Algo(RM_MODEL_RM_75_E, RM_MODEL_RM_ISF_E)`
    (EMU:86-107). Its tool frame is PROCESS-GLOBAL and must be re-asserted before every call (EMU:115-146).

### 1.2 Kinematic parameters (URDF chain, both arms mirror in Y)
| Element | Value | Evidence |
|---|---|---|
| `UGV_body_joint` fixed | z +0.195 | URDF:181-187 |
| `{L,R}_pole_joint` fixed | xyz (-0.01, +-0.124, 0.296) | URDF:892-896, 2615-2619 |
| `{L,R}_sliding_plate_joint` prismatic | axis +Z, origin z 0.1675, 0..0.315 m, v 0.16 m/s | URDF:933-946; [M] |
| `{L,R}_base_joint` fixed | rpy (0, pi/2, 0), xyz (0.055, 0, -0.01) -> arm base points along base_link +X | URDF:1000-1004, 2723-2727 |
| Arm base origin | (0.045, +-0.124, 0.6485 + pole) | [M] |
| Shoulder centre | (0.2855, +-0.124, 0.6485 + pole) | [M] via `srs_ik.shoulder_point` |
| joint1..7 origins | d1 0.2405, a(upper arm) 0.256, b(forearm) 0.21, j7 0.114 | [M] URDF parse; `SrsArm.geometry()` a=0.256 b=0.21 |
| Flange chain | joint7 -> CameraHolderLink -> (fixed z 0.05) Link7 -> (z 0.0125) ConnectorLink | URDF:1433-1437, 1489-1493 |
| S-R-S check | shoulder/wrist axes intersect, residual 7.9e-17 m on base_link->R_glove_frame_2 | [M]; WS/srs.py:349-452 |
| `alix_ref_frame` | base_link + (-0.010, -0.124, 0.526) | [M]; TG/transforms.py:23; REPO/tools/probe_task_ik.py:31 |

Source-of-truth chain: `rm_75_macro.xacro` -> `alix.urdf.xacro` -> `URDF` via
`WSPKG/scripts/expand_urdf.sh` (needs ROS `xacro`; `--check` diff mode, WSPKG/scripts/expand_urdf.sh:42-79);
copies policed by `REPO/tools/kinematics/check_kinematics_sources.py:8-41`.

### 1.3 TCP / tool-frame options (all fixed links on `{S}_ConnectorLink`, zero rotation)
| URDF link | xyz from ConnectorLink (right; left mirrors x) | Evidence |
|---|---|---|
| `{S}_glove_frame_1` | (+-0.05, 0, 0.145) | URDF:4446-4460 |
| `{S}_glove_frame_2` (default contact frame, Z press) | (+-0.0135, 0, 0.165) | URDF:4466-4480; WSPKG/WORKSPACE_ANALYSIS.md:82-84 |
| `{S}_glove_frame_3` | (+-0.075, 0.007, 0.17) | URDF:4486-4505 |
| `{S}_glove_frame_4` | (+-0.055, 0.007, 0.205) | URDF:4524-4537 |
| `{S}_tip_frame` | (+-0.015, 0.005, 0.23) | URDF:4507-4522 |
| `{S}_index_tip_frame` | (+-0.0242, 0.0288, 0.225) | URDF:4546-4556 |
| `Arm_Tip` (controller tool zero) | ConnectorLink - 15.3 mm on Z | TL/kinematics.py:47-51; EMU:149; REPO/docs/kinematics/FRAME_MAP.md:27-35 |
- Controller tool-frame names = URDF link minus `_frame` (11-char limit) (REPO/docs/kinematics/FRAME_MAP.md:16-24).
- RISK: FRAME_MAP's LEFT table is stale (L_glove_2 x -20.0 mm vs URDF -13.5 mm) — noted also at EMU:167-169.
- Contact patches/press axes per frame: `TG/config/glove_frames.yaml` via `TG/glove_frames.py:81-149`.
- No glove mesh exists: glove frames are geometry-less links (URDF:4440-4556) (inferred: the cloth glove
  is not in the collision model; the hand meshes stand in for it).

### 1.4 Units & conversions Lite needs
- `.txt` NDJSON `{"point":[7 ints]}` = millidegrees -> rad = v * pi/180000 (REPO/tools/dual_arm_verdicts.py:501, 580-589).
- SDK joint array maps 1:1 to URDF `{S}_joint1..7` (TG/sim_bridge.py:160-164; TL/kinematics.py:3-6).
- Pole: SDK lift mm -> `{S}_sliding_plate_joint` m = mm/1000 (TG/sim_bridge.py:166-177; TECHNICAL_INTERFACE.md:74).
- Hand: SDK counts 0..1000 (1000 = open) -> rad via `REPO/ros/hw/alix_hw/scripts/hand_units.py:33-60`
  (MAX_RAD 1.344 x4, thumb_bend 0.5236, thumb_rot 1.246). RISK: `TG/sim_bridge.py:95-102,179-185` uses a
  DIFFERENT travel table (1.60 / 0.70 / 1.10). Use hand_units.py (the one `dual_arm_verdicts.hand_rad` uses, 514-543).
- Controller World frame = arm base rotated Ry(+90 deg) (TL/kinematics.py:53-54; EMU:245-256, 339-361).
  Four authoring frames and their conventions: TG/reference_frames.py:1-45.
- Recording clock 5 ms/pt (drag-teach) vs playback ~5.5545 ms/pt (REPO/tools/taught_tracks.py:102-116;
  `V6/README.txt:6` uses 5 ms).

---

## 2. Inverse kinematics

### 2.1 Numerical DLS — `alix_workspace.ik.IKSolver`
- Signature: `IKSolver(model, base_link, tip_link, base_transform=None, seed=0, locked=None)`;
  `solve(target_T, q_init=None, pos_tol=2e-3, rot_tol_deg=3.0, max_iters=120, restarts=6, damping=0.05) -> IKResult`
  (WS/ik.py:150-153, 270-273).
- `locked={joint: value}` pins joints (the pole) out of the solve while FK still applies them (WS/ik.py:157-162).
- Algorithm: DLS `dq = J^T (J J^T + lambda^2 I)^-1 e`, step capped 0.3 rad, `np.clip` to limits each
  iteration (WS/ik.py:298-306). Analytic geometric Jacobian (213-231), verified vs finite differences
  (WSPKG/ANALYSIS_VERIFICATION.md:60-77).
- Seeding: ONLY restart 0 uses `q_init`; restarts 1..N use uniform random seeds within limits
  (WS/ik.py:282-288) -> a failed seeded solve can return a different branch. For continuity pass `restarts=1`.
- Limits: continuous -> +-pi; `lower==upper==0` treated as fixed (WS/ik.py:175-194).
- Failure reporting: `IKResult{success, position_ok, orientation_ok, pos_err_m, rot_err_deg, q, joint_names, iterations}`
  + `.joint_map` (WS/ik.py:134-147); best-of-restarts ranking `_better` (326-334).
- Extras: `jacobian(q)`, `dexterity_jacobian`, `manipulability(q, kind)`, `condition_number(q, kind)`
  (revolute columns only after the unit bug fix) (WS/ik.py:229-259; WSPKG/ANALYSIS_VERIFICATION.md:134-178).
- Speed [M]: seeded solve of a +10 mm target: median 0.20 ms, max 2.64 ms (default tolerances).
  Default `pos_tol=2 mm` is too loose for offsets ([M] residuals up to 1.987 mm) -> use `pos_tol=1e-4`.

### 2.2 Analytic structure — `alix_workspace.srs.SrsArm` (arm-angle / swivel parametrisation)
- `SrsArm.from_model(model, base_link, tip_link, swivel_ref_axis=None)` discovers 7 revolute DOF + at most one
  prismatic (the pole), verifies S-R-S (axis residual <= 0.5 mm), refuses otherwise (WS/srs.py:349-452, 86).
- The redundancy: elbow orbits the shoulder-wrist line; q4 depends only on shoulder-wrist distance
  (`elbow_from_distance`, WS/srs.py:473-482); both triples are ZYZ with two Euler branches each ->
  up to 4 configurations per swivel per elbow sign, mutually disjoint (WS/srs.py:11-29, 279-292).
- Swivel angle: `swivel_angle(q, pole, ref)` / `swivel_from_points` (WS/srs.py:725-758); reference axis
  must be chosen ONCE per stroke: `swivel_ref_for(directions)` (model Z, X, Y; refuses if all > 0.95)
  (553-606) or `stereographic_swivel_for(directions)` (608-670). `ref=None` is deprecated legacy (239-259).
  [M] model Z is ill-conditioned on L_P01P02 (|ref.a| = 0.9502 raised `IllConditionedSwivelError`);
  `swivel_ref_for` picked Y for both R2_LIDTOP and L_P01P02; stereographic default fine for both.
- Fibre: `fibre(T_tip, seed_q, pole, ref) -> SelfMotionFibre`; `SelfMotionFibre.at(psi[]) -> (n,4,7)`, NaN where
  a branch violates limits (WS/srs.py:772-785, 936-1091). Branch slot index is stable across psi (944-947).
- `branches_at(T, psi, seed_q, ...)` returns up to 4 configs SORTED BY LIMIT MARGIN, not by proximity to a
  reference (WS/srs.py:787-817) -> caller must pick nearest itself.
- Limit helpers: `wrap_into_limits` (+-2pi folding, needed for J7 +-360) (820-838), `within_limits` (842-844),
  `limit_margin` (846-853); `_polish` DLS steps (856-876); `jacobian(q, pole)` 6x7 (878-888).

### 2.3 Seedless closed form — `alix_workspace.srs_ik`
- `solve(arm, T_tip, psi, pole=0.0, ref=None, elbow_sign=1) -> (4,7)` (NaN row = limit violation; all-NaN =
  wrist outside the [|a-b|, a+b] shell or straight elbow); `solve_all(...) -> (8,7)` both elbow families;
  `fibre(arm, T_tip, pole, ref, elbow_sign)` seedless `SelfMotionFibre` (WS/srs_ik.py:349-442).
- Verified to 1.9e-10 rad vs `at()`, FK round trip 4.4e-13 m; 0.36 ms/pose vs 2.52 ms seeded numeric
  (WS/srs_ik.py:76-113). The referenced `verify.py` is NOT in the tree (missing file, `ls` failed).
- [M] On R2_LIDTOP: median 0.231 ms/solve; at the taught swivel the nearest branch reproduces the taught q
  to 1.7e-10 deg — i.e. closed form + taught arm angle exactly recovers the recording.
- `swivel_arcs.feasible_arcs(fib, arm, branch)` -> exact feasible swivel intervals under joint limits
  (no sampling), gimbal guard 1e-5 rad (WS/swivel_arcs.py:156, 513-597); `arcs_contain`, `arc_distance`
  (634-656) — use to move psi to the nearest feasible arc when the taught psi becomes illegal after an offset.

### 2.4 Continuity machinery already in the tree
- `redundancy.compute_layer(arm, poses, psi_deg, pole_m, seeds, seed_solver, collision=...)` builds a
  swivel x waypoint x branch grid with block codes (limit / singular / self / wall / fixture / partner)
  (WS/redundancy.py:56-86, 231-357); `corridor(layer, max_swivel_step_deg=15, max_joint_step_deg=10,
  allow_branch_change=False, ...)` = exact bottleneck DP for a continuous swivel choice that never
  teleports between branches (WS/redundancy.py:905-1196). This is the C-space atlas (X2 excludes its UI;
  the math can run server-side) (inferred).
- Vendor min-motion IK: `_ik_min_motion(seed_deg, pose6, span_deg=60, step_deg=5)` searches the arm angle
  around the seed's (`rm_algo_calculate_arm_angle_from_config_rm75`) and keeps the smallest max joint step;
  motivated by free-redundancy IK walking J5 1127 deg vs 113 deg planned (EMU:364-399).
- `tools/verify_task_motion.verify(...)`: vendor seeded IK along a path, candidates [seed, previous,
  SEED_HOME, zeros], FK-verify 2.0 mm / 0.6 deg, `pose_filter` collision with J3 +-45/90 branch search,
  `JOINT_JUMP_DEG = 45` (sparse waypoints), limit margin 5 deg warning, singularity warnings
  (REPO/tools/verify_task_motion.py:91-150, 800-900).
- Studio `do_animate_sequence`: IK per waypoint seeded from the previous solution, no jump check
  (TG/server.py:1645-1739; WSPKG/ANALYSIS_VERIFICATION.md:286-290).

### 2.5 Measured behaviour of offsets on real Lite tracks [M]
Pole 0.15 m (the program's own pole, `V6/JSON/RIGHT_V6_70SERIES.json`), tool `{S}_glove_frame_2`,
constant offset in base_link, orientation held. Taught max adjacent step: R2_LIDTOP 1.59 deg, L_P01P02 1.41 deg.

| Strategy | Track / offset | fails | adj step max | n>10 deg | other |
|---|---|---|---|---|---|
| A DLS, seed = taught q_k, tol 0.1 mm/0.05 deg, restarts=1 | R2_LIDTOP +10 mm z | 137/3751 | 38.3 | 1 | 4.8 s total |
| B DLS, seed = prev + taught increment | R2_LIDTOP +10 mm z | 127 | 80.3 | 1 | 2.4 s |
| C `srs_ik.solve_all` at taught psi, nearest to taught q | R2_LIDTOP +10 mm z | 47 | 170.1 | 16 | 2.0 s |
| Vendor arm-angle IK at taught arm angle | R2_LIDTOP +10 mm z | 1034 | 174.5 | 26 | 0.05 s |
| Vendor seeded IK, traversal mode, seed = taught | R2_LIDTOP +10 mm z | 0 | 41.1 | 15 | 0.12 s |
| Vendor seeded IK, single-step, seed = prev | L_P01P02 +10 mm y | 88 | 358.6 | 335 | 0.10 s |
| D prototype: differential IK (taught increment feed-forward + DLS + null-space pull to taught q, 4 iters) | L_P01P02 +10 mm y | 0 | **1.41** | **0** | 95/3000 pts >1 mm residual (max 4.3 mm at singular margin 5.5 deg) |
| D prototype | R2_LIDTOP +10 mm z | 0 | 15.6 | 2 | residual max 17 mm near q4~-0.2 deg, q2~1.2 deg (straight elbow) |
| D prototype | R2_LIDTOP +20 mm x | 0 | 40.1 | 3 | 1702 pts >1 mm (forward reach exhausted) |

Taught tracks cross zero on q2, q4 (elbow family flips: R2_LIDTOP q4 -86.7..91.5 deg) and q6
(R2_LIDTOP q6 -119.5..78.5) [M]; vendor singular margin (TG/singularity.py:6-11; WS/redundancy.py:120-140):
R2_LIDTOP min 0.00 deg, 23.6 % < 10 deg; L_P01P02 min 0.06 deg, 11.9 % < 10 deg [M].
Conclusion: at singular passages an exact offset is physically unavailable; Lite must detect, taper or
report — never silently accept the jumps.

---

## 3. Joint limits

| Joint | URDF / xacro (deg) | Vel (deg/s) | Vendor Algo default [M] | Real-arm DRIVE limits (EMU:429-433) |
|---|---|---|---|---|
| J1 | +-178 | 180 | +-178 | +-183 |
| J2 | +-130 | 180 | +-130 | +-135 |
| J3 | +-178 | 225 | +-178 | +-183 |
| J4 | +-135 | 225 | +-135 | L +-169 / R +-140 |
| J5 | +-178 | 225 | +-178 | +-360 |
| J6 | +-128 | 225 | +-128 | +-133 |
| J7 | +-360 | 225 | +-360 | +-365 |

Evidence: REPO/ros/description/rm_description/urdf/rm_75_macro.xacro:201,232,263,294,325,356,430; URDF joints
([M] parse; `R_joint4` 1.5707963 rpy etc.); acceleration 10.472 rad/s^2 (600 deg/s^2), jerk 1500 rad/s^3
(REPO/ros/manipulation/alix_moveit_config/config/joint_limits.yaml:1-36,161-167); TL/kinematics.py:37-48.
- Precedence: "where the two differ the document wins and the emulator is the thing to correct"
  (REPO/tools/verify_task_motion.py:270-274); drive vs motion limits are different things
  (REPO/tools/kinematics/check_kinematics_sources.py:36-41).
- Live readback available: `rm_get_joint_min_pos/max_pos` (software) and `rm_get_joint_drive_min_pos/max_pos`
  (SDK/rm_robot_interface.py:306-438). The Lite upload channel reads neither ([M] grep found nothing).
- Margins to apply: 5 deg position allowance for replay overshoot (REPO/tools/taught_tracks.py:122-126),
  5 deg cleaning / 2 deg mechanism margin (REPO/tools/verify_task_motion.py:94-112); 0x4000 step fault
  at 10 deg/point (REPO/tools/taught_tracks.py:127-128; TL/kinematics.py:56-61).
- Pole 0..0.315 m v 0.16 m/s (URDF:933-946); SRDF pole states 0.3/0.225/0.15/0.075/0.01 m (SRDF:271-317).
- Bin rail 0..0.2 m, drop 0..0.45 m (URDF:222-247).

---

## 4. Collision

### 4.1 Library & robot model
- trimesh `CollisionManager` backed by python-fcl; pyproject marks python-fcl REQUIRED for the dual-arm sweep
  (WSPKG/pyproject.toml:12-20); `_fcl_in_collision` raises if fcl missing (WS/collision.py:512-526).
- `RobotCollisionModel.from_file(urdf_path, tag="collision"|"visual", index=None)`: per-link meshes +
  box/cylinder/sphere primitives, `package://` resolved by `PackageIndex.discover` over the source tree and
  `src/` siblings (WS/collision.py:43-135; WS/package_index.py:27-120).
  - [M] 80 collision STL files = 6.4 MB; 85 visual files = 81.1 MB (DAE/OBJ/STL); packages:
    rm_description, inspire_left/right_hand_description, ugv_description, pole_description,
    realsense2_description (vendor/realsense-ros) — all resolved, none missing.
  - `local_meshes(link_filter, max_faces=800, split_materials=False)` for viewers (WS/collision.py:183-246);
    Studio also serves a baked overlay `WS/workspace_data/visual_overlay_baked.npz` (TG/server.py:1875-1883, 1918-1990).
  - `posed_meshes(model, joint_values, base_transform, link_filter)` (WS/collision.py:248-269).

### 4.2 API catalogue (signatures quoted)
| API | Signature | Checks | Output |
|---|---|---|---|
| `PosedRobotCollision` | `(rcm, model, link_filter=None)`; `.set_pose(joint_values, base_transform=None)`; `.in_collision(fixture_mgr)`; `.contact_count(fixture_mgr)` | robot links vs ONE fixture manager | bool / int (WS/collision.py:366-410) |
| `build_fixture_manager` | `(fixture_mesh)` | — | one-object manager (WS/collision.py:413-419) |
| `fixture_contact_faces` | `(robot_meshes, fixture_mesh, contact_tol_m=0.005, ring=1, require_fcl_collision=True)` | face-level contact | face indices (+rings) for painting (WS/collision.py:430-509) |
| `_ArmCollision` (private) | `(rcm, link_filter, boxes=None)`; `.pose(transforms)`; `.mgr` | any link set (+envelope boxes) | raw manager (WS/dual_arm_plan_collision.py:129-162) |
| `compute_dual_arm_plan_collisions` | `(left_plan, right_plan, urdf_path=None, srdf_path=None, dt=0.1, window=100, return_frames=False, include_self=True, include_body=True, extra_managers=None, padding_m=0.0, left_hand_state=None, right_hand_state=None)` | L-vs-R, self (each arm), arms-vs-{bin_link, glove_wearing_unit_arm}, arms-vs-extra managers, optional padding distance | dict: `sample_times`, `joint_timeline`, `collisions[{index,time,pairs}]`, `windows[{center,start,end}]`, `frames{i:{link:16 floats}}`, `mesh_links`, `checks`, counts (WS/dual_arm_plan_collision.py:586-760) |
| `cross_arm_separation` | `(left_plan, right_plan, urdf_path=None, srdf_path=None, dt=0.05, offset_invariant=True, offset_window_s=None, link_set="full", report_pairs=8, progress=None, left_hand_state=None, right_hand_state=None)` | min L-R distance, aligned and over relative phase offsets; link sets slim/hands/full/arms/boxed/distal | `collides`, `min_mm`, `min_pair`, `at`, `aligned_*`, `structural_floor_mm`, `closest_samples`... (1089-1283; LINK_SETS 855-890) |
| `compute_plan_frames` | `(plans=None, tracks=None, urdf_path=None, srdf_path=None, dt=0.1, initial=None, extra_links=None)` | none (FK only) | per-sample link transforms for playback (487-583) |
| `PlanWallChecker` | `(urdf_path=None, scene_yaml=None, mesh_override=None)`; `.check(plan_path, dt=0.05, refine_dt=0.01)` | ALL robot links vs `washroom_structure` | contact runs `{stage_idx, stage, links, t0, t1, depth}` (WS/plan_scene_validation.py:179-270) |
| `ClearanceProbe` | `(urdf_path, scene_yaml, side, articulation=None, include_hand=True)`; `.at(joint_values) -> m` | one arm vs commode (min distance) | metres; `chain_clearance`, `dense_clearance`, `verdict` (WS/clearance.py:97-179) |
| `BodyCollisionWorld` | `(repo_root=None, scene_yaml=None)`; `.fixture_manager(fixture, fixture_type, ugv_standoff_m=None)`; `.make_pose_filter(config, fixture, partner_state=None, pole_override_m=None) -> (filter_fn, info)` | active arm+hand vs {partner arm, partner-hand envelopes, bin & covers, UGV, poles, glove unit} + walls + commode body/seat/lid; curated self pairs; ACM + contact allowance + rest baseline | `filter_fn(q_deg7) -> ["A<->B"]`; `.depths(q, scope) -> {pair: mm}`; `.distal/.proximal/.self_only` (TG/body_collision.py:130-952) |
| `shared_world` | `(scene_yaml=None)` | lru-cached world per venue | (TG/body_collision.py:955-990) |
| `transit_blocked` / `corridor_witness` | `(seed, solutions, pose_filter)` | joint lerp at 2 deg steps; tuck-family detours | blockage / fold (TG/approach_transit.py:42, 95-158) |

### 4.3 World objects — what can be loaded
| Object | How represented | Toggle / state | Evidence |
|---|---|---|---|
| Self-collision | arm/hand links; SRDF ACM (1824 disabled pairs [M]) | `include_self`; curated lo/hi/distal sub-managers in BodyCollisionWorld | WS/dual_arm_plan_collision.py:112-123,689-693; TG/body_collision.py:354-399 |
| Partner arm | other side's links; cross-side pairs never ACM-disabled | posed via `partner_state{arm_state,pole_m,hand_state,joints}` | WS/dual_arm_plan_collision.py:1127-1131; TG/body_collision.py:499-519,666-708 |
| Hands | 31 links/side meshes, or envelope boxes (+5 mm) per SRDF hand state | `left/right_hand_state`; idle partner = union of grasp+release envelopes | WS/dual_arm_plan_collision.py:186-379; TG/body_collision.py:71-91 |
| Bin | `bin_link` (BinShort.stl hollow shell) + handles + 2 lid_opening_covers, prismatic rail/drop | posed BIN_LOWERED everywhere; dual-arm sweep `include_body` | URDF:222-440; SRDF:286-297; WS/dual_arm_plan_collision.py:644-648 |
| Glove-wearing unit lever | `glove_wearing_unit_arm` (revolute 0..1.57) | in slim body set | WS/dual_arm_plan_collision.py:644 |
| UGV body / poles / sliding plates | robot links | static in BodyCollisionWorld; `_MOUNT` excluded from separation | TG/body_collision.py:251-312; WS/dual_arm_plan_collision.py:796 |
| Walls | `washroom_structure` STL per venue (800/1000/1400 mm, amsterdam) at x=1.215, yaw 180 | venue choice only; disabled venue raises | WS/plan_scene_validation.py:117-155; TG/body_collision.py:177-204; `REPO/ros/orchestration/alix_tasks/config/scene/washroom_800mm_wide_scene.yaml:29-34` |
| Commode | body/lid/seat STLs (commode_c), hinge origin (0.115,0,0.02), closed 0 / open -110 deg; pose (1.005,0,0.42), standoff 0.25 | `fixture_type` closed / lid_open / lid_seat_open; any angle via `scene_pose.resolve_component_pose(..., joint_angle_deg=)` | TG/body_collision.py:104-108,401-464; `.../commode_cleaning/commode_c/scene.yaml`; TG/scene_pose.py:161-216 |
| Fixture faces for painting | Studio `_fixture_collision_for` + `fixture_contact_faces` | per mesh key | TG/server.py:931-952,1219-1255 |
- RISK: `clearance.load_fixture` (WS/clearance.py:49-94) omits the venue washroom rotation that
  `scene_pose` composes (TG/scene_pose.py:93-145; G37 note TG/body_collision.py:439-447) — prefer
  `BodyCollisionWorld.fixture_manager`.
- Floor/ceiling: no floor object is loaded by BodyCollisionWorld (inferred from `_pairs` objects list,
  TG/body_collision.py:807-813).

### 4.4 Inputs & units per API
- Dual-arm sweeps: plan JSON files (`sub_trajectories[].joint_names/waypoints[].positions/time_from_start`,
  radians) (WS/dual_arm_plan_collision.py:46-87); joint maps in radians; pole/hand joints only if present in
  the plan, else `_build_home` defaults (bin LOWERED, poles 0.300, hands open unless state given) (385-452).
- BodyCollisionWorld: active-arm joints in DEGREES (Algo convention) (TG/body_collision.py:24-27,757-762);
  active pole from config `move_pole*` stage or `pole_override_m`; partner joints in RADIANS via
  `partner_state["joints"]` (698-708); fixture type from `config.task_parameters.commode_fixture_type`.
- Output depth: FCL contact depth per pair (`contact.depth`) (TG/body_collision.py:774-784;
  WS/plan_scene_validation.py:211-217); distances via `min_distance_other` (clamped at 0 in separation,
  WS/dual_arm_plan_collision.py:1191-1204).

### 4.5 Cost per sample
| Operation | Cost | Evidence |
|---|---|---|
| BodyCollisionWorld build (URDF + meshes + walls + fixture) | 2.6 s | [M] |
| BodyCollisionWorld per-pose `filter_fn` | median 1.96 ms, p95 4.46 ms ("~3 ms") | [M]; TG/body_collision.py:798-799 |
| curated self-collision vs `in_collision_internal` | ~1 ms vs 95 ms | TG/body_collision.py:359-362,816-818 |
| Whole-tree FK `link_world_transforms` | 0.34 ms | [M] |
| Slim dual-arm per sample (L-vs-R + 2x self + 2x body) | median 8.9 ms, p95 13.4 ms | [M] (same calls as WS/dual_arm_plan_collision.py:686-698) |
| `min_distance_other` slim (10/11 links) / full (37/38 links) | 24.5 ms / 88.9 ms | [M] |
| Slim manager build / full manager build (meshes cached) | 0.9 s / 0.4 s; "~10 s" incl. mesh load | [M]; WS/dual_arm_plan_collision.py:1021-1022 |
| Fixture BVH build | ~2-4 s | TG/server.py:931-935 |
| Face marking for painting (8803 candidate faces) | 3.7-4.2 s, 0.41 GB at batch 500 | WS/collision.py:427,464-473 |
- dt is a CORRECTNESS parameter: max relative link displacement 109 mm at dt 0.05 vs 44-47 mm at 0.02
  -> sweep at dt 0.02 s (REPO/tools/dual_arm_verdicts.py:350-356). At 5.5 ms/pt that is every ~4th point
  (inferred). A 3751-pt track (~20.8 s) ~= 1040 samples ~= 9 s slim, ~2 s BodyCollisionWorld (inferred from [M]).

### 4.6 Whole-trajectory APIs and the .txt seam
- Per-sample sweep with clustering windows: `compute_dual_arm_plan_collisions` (WS/dual_arm_plan_collision.py:660-722).
- Coarse-then-refine contact runs: `PlanWallChecker.check` (WS/plan_scene_validation.py:219-256).
- Between-waypoint sampling: `clearance.chain_clearance(samples_per_leg=8)` (WS/clearance.py:125-146);
  `approach_transit.transit_blocked` at 2 deg (TG/approach_transit.py:60-120).
- **The .txt shim (reuse this)**: `REPO/tools/dual_arm_verdicts.py`
  - `track_points(path)` (580-589), `build_timeline(task, points, offsets_rad)` (591-619) with constant pole and
    hand seeding ("Seeding is MANDATORY ... reports 8.52 mm where the commanded hand state gives 42.03", 24-33),
  - `timeline_to_plan(timeline, name)` one-stage plan JSON (634-660),
  - `assert_arm_sides` swapped-argument guard (662-692; swapping silently returns 33.23 mm),
  - `sweep(left_tl, right_tl, dt=0.02, window_s=None, ...)` writes temp plans and calls `cross_arm_separation` (709-731),
  - gate: windowed minimum vs `CLEARANCE_FLOOR_MM = 25.0` (369; 34-61).
- [M] Real-track check (R2_LIDTOP right, pole 0.15, closed commode, partner L_P01P02 first point, hand grasp):
  800 mm venue -> `R_Link5<->washroom_structure` on 28/188 sampled points + hand/wall pairs; 1400 mm venue ->
  wall pairs vanish; remaining `IRH_right_thumb_2<->commode_lid` (9/188, depth ~2.2 mm) and
  `R_ConnectorLink<->commode_lid` (1). The arm did not physically hit a wall when taught, so the default
  venue does not match the Lite cell (inferred) and thumb is not in the `R_glove_frame_2` allowed contact set
  (`REPO/ros/orchestration/alix_tasks/config/contact_links.yaml:84-98`).
- [M] Aligned slim L-vs-R on R2_LIDTOP vs L_P01P02 started together: 28/209 samples hit (illustrative only —
  these two programs are not paired in V6).

### 4.7 Bin representation & toggling
- Links: `bin_rail_link` (fixed), `bin_link` (BinShort.stl, 366 mm hollow shell), `bin_handle_left/right`,
  `lid_opening_cover1/2` (fixed children of bin_link) [M]; URDF:188-440.
- States: BIN_STOWED (0,0), BIN_EXTENDED (0.2,0), BIN_LOWERED (0.2,0.45) (SRDF:286-297).
- Current behaviour: always BIN_LOWERED in `_build_home` (WS/dual_arm_plan_collision.py:434),
  BodyCollisionWorld (TG/body_collision.py:627-631), Studio IK (TG/server.py:1133-1135) and UI text
  (TG/web_editor/template.html:753).
- Toggle options for Lite (R10):
  1. Exclusion in dual-arm sweep: `include_body=False` (drops `bin_link` + glove lever) (WS/dual_arm_plan_collision.py:595,647-648).
  2. Re-pose: set `bin_rail_joint`/`bin_drop_joint` to BIN_STOWED values in the home/joint map before
     `link_world_transforms` (any API that takes joint maps).
  3. BodyCollisionWorld: no flag; workaround `partner_state={"joints": {"bin_rail_joint":0.0,"bin_drop_joint":0.0, ...}}`
     works because the dict is merged into `home` before the static manager is posed (TG/body_collision.py:698-708, 711-745)
     (inferred hack) — or post-filter pairs containing `bin_`/`lid_opening_cover` (cf. `classify_pair` tokens,
     WS/redundancy.py:90-117). Cleaner: add an `exclude_links` predicate to `_managers` (new code).
  4. Viewer must hide/show bin meshes consistently (Studio has no such control; grep found no bin toggle in editor.js).

---

## 5. `dual_arm_plan_collision` and the Studio Dual-Arm Collisions tab

- Inputs/outputs: see 4.2. Timeline per plan: stages concatenated, joints carried forward
  (WS/dual_arm_plan_collision.py:46-87); common grid at `dt` from t=0 for both (650-669); `_sample_at` linear
  interpolation clamped at ends (90-106) — a single-sample plan is a "hold" (REPO/tools/dual_arm_verdicts.py:621-632).
- Link sets: slim whitelist L/R Link1..7 + CameraHolder + Connector (+camera_link right) + palm, ~500 pairs/sample
  (WS/dual_arm_plan_collision.py:165-182); [M] 10/11 links + 2 body links.
- Endpoint: `POST /api/dual-arm-collisions {leftPlan, rightPlan, dt=0.05, window=100}` ->
  `do_dual_arm_collisions` calls `compute_dual_arm_plan_collisions(..., return_frames=True)`, drops
  `joint_timeline`, caches on the plans' content SHA-1 + `_DAC_FK_VERSION` (TG/server.py:4300-4364, 6015).
  It passes no `extra_managers` (no commode/walls), no hand states, `include_body` default (TG/server.py:4354-4356).
- UI (TG/web_editor/template.html:598-640, 897-898; editor.js):
  - pickers (schedule pair / plan mode / left / right), `dt`, "Check Collisions" (template.html:602-626);
  - `dacRunCheck` blocks pairs whose lid/seat states conflict (`dacReconcileFixture`) then calls the API
    (editor.js:14332-14476);
  - `dacRenderPlot` — Plotly bar timeline, red `#ff4d4d` at colliding samples, cyan elsewhere,
    click-to-scrub (editor.js:14478-14498);
  - `dacGoto` applies `res.frames[sample]` via `applyRobotPose`, `dacHighlight` recolours colliding links
    `DAC_HIT_COLOR 0xff4d4d` (robot base 0x9fb3c8), info line "COLLIDING: A<->B" (editor.js:12427-12450, 14515-14546);
  - Prev/Next jump between colliding samples, click-to-jump log (editor.js:14548-14559, 14272-14298).
  This is the closest existing pattern for R5's "graph stops at the joint value and highlights the
  collision point" — but it plots a collision timeline, not the joint curves (inferred gap).
- Related endpoints: `/api/plan-frames` (FK frames only, `compute_plan_frames`, cached) (TG/server.py:4590-4680),
  `/api/robot-geometry` (meshes, TG/server.py:1918-1990), `/api/ik` (per-point IK + fixture face painting,
  TG/server.py:1095-1255), `/api/analyze-plan` (`plan_analysis.analyze_plan`, TG/server.py:4272-4290).

---

## 6. Pole height in FK and collision

- Kinematic role: prismatic `{S}_sliding_plate_joint` between `{S}_Pole` and the arm base; it only
  translates (srs_ik asserts, WS/srs_ik.py:210-215); arm base z = 0.6485 + pole m [M].
- FK: include it as q[0] of the `base_link -> tip` chain (WS/urdf_kinematics.py:218-229, [M]); or
  `SrsArm.tool_pose(q, pole)` (WS/srs.py:522-542); or set it in the joint map for `link_world_transforms`.
- IK: `IKSolver(..., locked={"R_sliding_plate_joint": pole})` (WS/ik.py:157-162; Studio TG/server.py:1137-1160);
  SrsArm/srs_ik take `pole=` (WS/srs_ik.py:234-246).
- Collision: joint map entry; dual-arm sweep uses plan values else `_build_home` 0.300 m (WRONG for most
  programs: 150-225 mm too high — WS/dual_arm_plan_collision.py:409-424); BodyCollisionWorld active pole from
  `move_pole*` stage / `pole_override_m`, partner default `half_length` 0.15 (TG/body_collision.py:647-662).
- Units: lift mm -> m (TECHNICAL_INTERFACE.md:74; TG/sim_bridge.py:166-177; REPO/tools/taught_tracks.py:3532).
- Lite data: per-program pole mm already in `V6/JSON/*_V6_70SERIES.json` steps `{"type":"pole","mm":...}`
  ([M] R2_LIDTOP 150 mm, R_P01P03 50 mm, etc.); pole speed 72.5 mm/s measured (V6/README.txt:9-15).
- Named presets for R7: SRDF pole states (SRDF:271-317) and hand states (SRDF:659-795); Studio exposes
  `poleStateSelect` (template.html:753) and `/api/joint-presets` (TG/server.py:1763).

---

## 7. Standalone use from a Lite backend

Answer: YES — a Lite Python backend can run FK/IK and both collision engines with only
(left joint trajectory, right joint trajectory, pole heights, hand states, fixture articulation, venue).
What it needs:
1. Python deps: numpy, scipy, pyyaml, trimesh, python-fcl, pycollada, networkx (WSPKG/pyproject.toml);
   installed here in the user site ([M] `ls ~/.local/lib/python3.12/site-packages`). Note system scipy 1.11.4
   warns about numpy 2.4.6 ([M] import warning) — pin versions in a Lite venv (inferred).
2. Kinematics only (FK/IK/SRS): just `WS/urdf/alix.urdf` (package data, WSPKG/pyproject.toml package-data). No meshes.
3. Collision: mesh resolution via `PackageIndex.discover(urdf)` — needs the ALIX source tree (repo root by
   nearest `.git`, plus `src/` siblings for `realsense2_description`) (WS/package_index.py:27-120). Options:
   keep a read-only checkout of `alix_ws/src` or vendor the 80 collision STLs (6.4 MB) with stub package.xml files
   and pass a custom `PackageIndex` (`RobotCollisionModel.from_file(..., index=...)`, WS/collision.py:54-62) (inferred).
4. SRDF: `find_srdf` walks up to a project ceiling; override with `ALIX_SRDF` (WS/srdf_states.py:96-154).
   Initial positions from `*_description/config/initial_positions.yaml` (WS/srdf_states.py:61-82).
5. BodyCollisionWorld additionally needs: repo root marker (`ARCHITECTURE.md` + `core/`) (TG/body_collision.py:114-119),
   venue YAMLs under `ros/orchestration/alix_tasks/config/scene/`, `contact_links.yaml`, the commode
   `scene.yaml` + STLs, and the wall STL resolved through `tools/ament_index_shim.py`
   (WS/plan_scene_validation.py:31-114 — that module raises at IMPORT if no repo root and no `ALIX_REPO_ROOT`, 53-59).
   It also takes a task-config-shaped dict; a minimal synthetic
   `{"task_parameters":{"ik_frame":"R_glove_frame_2","commode_fixture_type":"closed"},"stage_sequence":[{"stage":"hand_grasp","planning_group":"inspire_hand_right","pose_name":"grasp"}]}`
   + `pole_override_m` + `partner_state` worked [M].
6. The plan-file APIs need files: either reuse the `dual_arm_verdicts.timeline_to_plan` temp-file shim or
   re-implement the 60-line loop of `compute_dual_arm_plan_collisions` in memory with `_ArmCollision`,
   `_build_home`, `load_disabled_pairs` (private names — pin the ALIX commit) (inferred).
7. Threading: FCL managers shared across threads segfault (TG/body_collision.py:148-171;
   WS/dual_arm_plan_collision.py:1023-1025) -> single collision worker/process with a queue.
8. Venue/fixture calibration for the Lite cell is a prerequisite for meaningful verdicts ([M] 4.6).

---

## 8. Gaps for R9 / R14 and the recommended algorithm

### 8.1 What is missing
- G1 No .txt-native offset tool anywhere: no Cartesian-offset/retarget function for taught tracks
  (grep of tools/ and exp01 found none; `build_timeline(offsets_rad=)` offsets JOINTS, not Cartesian,
  REPO/tools/dual_arm_verdicts.py:591-619).
- G2 No IK in the tree guarantees continuity along a dense path: DLS random restarts (WS/ik.py:282-288),
  `branches_at` sorts by margin (WS/srs.py:787-817), vendor IK resolves redundancy its own way (EMU:364-372);
  [M] all produce >10 deg steps on real tracks (2.5).
- G3 Singular passages: taught tracks cross q4=0/q2=0/q6=0 [M]; nothing tapers or limits an edit there.
- G4 Swivel reference conditioning must be chosen per stroke (`swivel_ref_for`) or SRS calls raise [M].
- G5 Boundary blending of a sub-region edit into the unedited neighbours (no function exists) (inferred).
- G6 Validation exists piecemeal (limits, steps, deviation, singularity, collision) but not as one gate on
  .txt tracks: `taught_tracks.deviation_mm` (REPO/tools/taught_tracks.py:607-668), `STEP_FAULT_DEG`,
  `wipe_kinematics.screen_path` (WS/wipe_kinematics.py:210-233), BodyCollisionWorld.
- G7 Frames for a non-technical user: offset must be expressed as "towards toilet / sideways / up" (base_link
  axes) or "press deeper" (glove contact axis, TG/glove_frames.py:134-141) — no UI exists.

### 8.2 Recommended algorithm ("Offset & Verify", server-side, ~1-5 s per 3-4 k-point track [M])
1. Inputs: side, point range (region/sub-region), offset d (mm) + frame (base_link or tool), pole mm, hand
   counts, tool frame per region (default `{S}_glove_frame_2`), fixture state, venue, bin on/off.
2. FK all points once: `T_k = fk_batch(chain(base_link, tip), [pole, q_k])` (2.5 us/pt [M]).
3. Offset profile with smooth ramps: `d_k = w_k * d`, w = raised-cosine 0->1 over ~0.3-0.5 s at the edited
   range ends (new) so the edit never introduces a step at the boundary.
4. Track with continuity by construction (prototype D, [M] best result): for k = i0..i1
   `q <- q'_{k-1} + (q_k - q_{k-1})` (taught increment as feed-forward), then 2-4 iterations of
   `dq = J^+_lambda e + (I - J^+_lambda J) * k0 * (q_k - q)` with adaptive damping lambda (larger when
   `manipulability` small) and `np.clip` to limits shrunk by 5 deg. Use `IKSolver._jacobian/_pose_error`
   (WS/ik.py:213-268) or `SrsArm.jacobian` (WS/srs.py:878-888).
5. Optional exact polish per point where singular margin >= 10 deg: `IKSolver.solve(T'_k, q_init=q_D,
   pos_tol=1e-4, rot_tol_deg=0.05, restarts=1)`; accept only if the step to neighbours stays within gate.
   Alternative/cross-check engine: closed-form `srs_ik.solve_all` at the taught swivel (with `swivel_ref_for`)
   nearest to q_D, and `swivel_arcs.feasible_arcs` to move psi minimally when out of limits; or vendor
   `_ik_min_motion`-style arm-angle search (EMU:364-399) for controller parity.
6. Gate every point (all exist, need assembling): (a) limits with 5 deg allowance; (b) adjacent step
   <= max(1.5 x taught step, 0.5 deg) and hard <= 10 deg (0x4000); (c) joint speed at replay clock <=
   180/225 deg/s (TL/kinematics.py:37) — re-timing by the R2 optimizer comes after; (d) Cartesian residual
   <= 1 mm and `deviation_mm`-style polyline deviation vs the intended shifted path <= 5 mm (10 mm hard)
   (REPO/tools/taught_tracks.py:127-132, 607-668); (e) singular margin < 10 deg -> "near singular" warning
   (TG/singularity.py:36,136); (f) collision via BodyCollisionWorld filter per sample at dt 0.02 s (or slim
   dual-arm sweep when both arms move), classifying pairs (fixture / wall / partner / self / bin) with
   `redundancy.classify_pair` (WS/redundancy.py:95-117).
7. Repair, never silently: for failing spans, bisect a local scale on d_k (taper) and report
   "region X can move at most Y mm here"; if a span cannot be fixed, refuse the edit for that span and keep
   the taught points (with ramped transition).
8. R14: regenerate frames (`compute_plan_frames`-equivalent) for before/after overlay playback, show the
   flagged indices on the joint plots, and only then enable "Test on real arm" (R4) and upload (INT-CTRL).
9. Output stays a millidegree NDJSON .txt (round to int) so the upload channel is unchanged.

---

## Requirement relevance

- **R3** Simulation playback: FK frames API (`compute_plan_frames`, WS/dual_arm_plan_collision.py:487-583) +
  robot meshes (`/api/robot-geometry`, baked overlay) + `applyRobotPose` viewer pattern (editor.js:12427-12437) are
  reusable; inputs must come from .txt via the shim.
- **R4** Real/sim toggle: sim state bridge uses the same units table (TG/sim_bridge.py:23-36); limits differ between
  URDF and real drive limits (section 3). (Emulator details belong to other maps.)
- **R5** Collision detection on joint trajectories: BodyCollisionWorld filter (~2 ms/pose) for commode/walls/bin/partner,
  slim dual-arm sweep (~9 ms/sample) for arm-vs-arm; DAC tab gives the timeline+highlight UX pattern (section 5);
  venue/hand/pole must match the cell ([M] false wall hits with default venue).
- **R7** Pole & hand presets: SRDF pole/hand group states; pole mm -> m; hand counts -> rad via hand_units.py;
  both must feed FK and collision (section 6).
- **R8** Merge with collision: `plan_min_time.concatenate` continuity rule (gap <= 0.5 deg, drop duplicate)
  (REPO/tools/plan_min_time.py:142-171); V6 bridges are min-jerk and NOT collision-checked (V6/README.txt:44,50-52);
  check bridges with `transit_blocked`-style 2 deg lerp + pose filter at the right fixture articulation
  (lid opened manually after Flow 5 -> `lid_open`), detour with `corridor_witness` tuck family (TG/approach_transit.py:122-158).
- **R9 / R14** Offset: section 8 (FK/IK engines, measured failure modes, recommended algorithm and gates).
- **R10** Dual-arm sim + bin toggle: correct URDF/meshes available (section 4.1); bin toggle missing in Studio,
  implementable via include_body / re-pose / filter (4.7).
- **R11** Smoothness validated in sim: reuse adjacent-step, joint-rate and singular-margin screens (3, 8.2-6) and
  collision re-check after smoothing (smoothing changes the path between points).
- **R12** Spike analysis: spikes cluster at singular passages ([M] 23.6 % of R2_LIDTOP within 10 deg);
  `wipe_kinematics.screen_path` names worst points (WS/wipe_kinematics.py:210-233); `plan_analysis.analyze_plan`
  per-waypoint manipulability/limit margin/velocity violations (WS/plan_analysis.py:74-187) needs the .txt shim.
- **X1** Path auto-generation excluded: do not port `path_generator`/`auto_generate`; FK/IK libs are independent of it.
- **X2** C-space/workspace excluded: `redundancy` atlas, `reachability`, `manipulability` maps and Studio
  Workspace/C-space tabs stay out of the UI; `srs.py`/`srs_ik.py`/`swivel_arcs.py` math may be used internally.
- **NFR-UX**: translate pair names to plain language via `classify_pair` classes; separate expected "contact"
  (allowed hand links on the region surface, contact_links.yaml) from real "collision"; report offsets as
  "max achievable Y mm here" rather than solver errors.
- **NFR-THEME**: reuse DAC colours (hit `#ff4d4d`, robot `0x9fb3c8`, bars `rgba(0,212,255,0.25)`) (editor.js:12446,14478-14491).
- **INT-CTRL**: keep millidegree NDJSON in/out; poles/hands from the workflow JSON drive FK/collision context.
- **INT-TRAJ**: the trajectory HTML has no kinematics of its own (inferred, other map); FK/IK/collision must be
  backend services the HTML calls.

---

## Reuse verdicts

| Feature | Verdict | What to lift | Evidence |
|---|---|---|---|
| URDF FK (single/batch/whole tree) | reuse-as-is | `UrdfModel.fk`, `fk_batch`, `link_world_transforms` + packaged `alix.urdf` | WS/urdf_kinematics.py:137-343; [M] 2.5 us/pt |
| Tool-frame FK incl. Arm_Tip | reuse-with-adaptation | `teachlab.kinematics.ArmKinematics` logic (copy the 40-line frame resolution; it lives in experiments) | TL/kinematics.py:97-224 |
| Numerical IK | reuse-with-adaptation | `IKSolver` with `locked` pole, `restarts=1`, tight tolerances; add continuity gate | WS/ik.py:150-323 |
| Closed-form SRS IK / swivel | reuse-with-adaptation | `SrsArm`, `srs_ik.solve_all`, `swivel_ref_for`, `swivel_arcs.feasible_arcs` as cross-check/repair | WS/srs.py:349-1102; WS/srs_ik.py:349-442; WS/swivel_arcs.py:513-669 |
| Vendor Algo FK/IK | reuse-with-adaptation | `Algo` FK (controller parity), `_ik_min_motion` pattern; assert tool frame per call | SDK/rm_robot_interface.py:6396-6550; EMU:115-146,364-410 |
| Continuity-preserving offset solver | not-available | build "Offset & Verify" (section 8.2); prototype D measured | [M] section 2.5 |
| Joint limits & gates | reuse-as-is | URDF limits, 5 deg allowance, 10 deg step fault, 10/5 mm deviation, joint speed ceilings | rm_75_macro.xacro:201-430; REPO/tools/taught_tracks.py:121-132 |
| Path deviation metric | reuse-with-adaptation | `deviation_mm` + `_point_to_polyline_mm` | REPO/tools/taught_tracks.py:607-700 |
| Singularity screen | reuse-as-is | `singularity.margin_deg/screen`, `wipe_kinematics.screen_path`, vectorised `singular_margin_deg` | TG/singularity.py:94-205; WS/redundancy.py:120-140 |
| Full-world per-pose collision | reuse-with-adaptation | `BodyCollisionWorld.make_pose_filter` with synthetic config, `pole_override_m`, `partner_state`; add bin/wall toggles; single worker | TG/body_collision.py:130-990; [M] 1.96 ms |
| Dual-arm sweep over time | reuse-with-adaptation | `compute_dual_arm_plan_collisions` / `cross_arm_separation` via the .txt shim; set poles+hands explicitly | WS/dual_arm_plan_collision.py:586-1283; REPO/tools/dual_arm_verdicts.py:501-731 |
| .txt -> plan shim | reuse-as-is | `track_points`, `build_timeline`, `timeline_to_plan`, `assert_arm_sides` | REPO/tools/dual_arm_verdicts.py:580-692 |
| Wall check over a plan | reuse-with-adaptation | `PlanWallChecker` coarse/refine pattern; venue must match Lite cell | WS/plan_scene_validation.py:179-270 |
| Commode at articulation | reuse-as-is | `BodyCollisionWorld.fixture_manager` / `scene_pose.resolve_component_pose(joint_angle_deg=)` | TG/body_collision.py:401-464; TG/scene_pose.py:161-216 |
| Clearance (min distance) to commode | reference-only | `ClearanceProbe` lacks washroom rotation; use pattern only | WS/clearance.py:49-179 |
| Bin toggle | not-available | implement via include_body / re-pose / exclude predicate + viewer hide | WS/dual_arm_plan_collision.py:595,647; template.html:753 |
| Collision timeline + link highlight UI | reuse-with-adaptation | DAC tab: Plotly timeline, scrub, Prev/Next, red link recolour; extend to joint plots | editor.js:14408-14595; template.html:598-640 |
| Bridge/merge collision check | reuse-with-adaptation | `transit_blocked` 2 deg lerp + `corridor_witness`; `plan_min_time.concatenate` gap rule | TG/approach_transit.py:42-158; REPO/tools/plan_min_time.py:142-171 |
| Robot meshes for sim view | reuse-as-is | `/api/robot-geometry` payload or baked overlay npz | TG/server.py:1875-1990; WS/workspace_data/visual_overlay_baked.npz |
| Redundancy atlas / corridor DP | reference-only (X2) | algorithmic idea for continuity (max joint step 10 deg, no branch change) | WS/redundancy.py:905-1196 |
| Human safety zones (`safety.py`) | not-applicable | — (human intrusion tiers, not robot collision) | WS/safety.py:1-153 |
| Reachability/manipulability voxel maps | not-applicable (X2) | — | WSPKG/README.md:1-40 |
| MoveIt `/compute_ik` probe | not-applicable | ROS-only (`rclpy`) | REPO/tools/probe_task_ik.py:22-30 |
