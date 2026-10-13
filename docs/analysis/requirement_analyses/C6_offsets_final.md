# C6 Offsets: R9 and R14, verified and corrected

Adversarial verification of the analyst draft for cluster C6_offsets, written 2026-10-13. This is a read-only review: no repository or workspace file was changed, and this file is the only one written. I opened every cited line. Every number marked **[V]** is my own re-measurement, made with read-only Python fed on stdin (`python3 -I -B -`, no files written) against the real V6 tracks and the ALIX code.

For the control app (UI v53 + runner v14) this file stays high-level, as the user asked: what it does and what Lite can use. Each plan-blocking issue gets one line.

## Path roots

| Tag | Absolute path |
|---|---|
| `TG/` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/` (ALIX Studio) |
| `WS/` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_workspace/src/alix_workspace/` |
| `REPO/` | `/home/newtonjeri/alix_ws/src/alix/` (holds `tools/`, `plan/`, `ros/`, `docs/`, `external/`, `experiments/`) |
| `DISP/` | `REPO/core/python/alix_dispatch/src/alix_dispatch/` |
| `UC/` | `/home/newtonjeri/alix_studio_lite_ws/upload_channel/` (UI v53, runner v14, `pole.py`, `hand.py`) |
| `TRAJ` | `/home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html` |
| `V6/` | `/home/newtonjeri/alix_studio_lite_ws/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/` |
| `URDF` | `WS/urdf/alix.urdf` |
| `FIND` | `REPO/plan/FINDINGS.md` |

---

## 1. Verdict summary

| | R9 Offset Adjustment Using IK | R14 Offset Calculation and Testing |
|---|---|---|
| Borrow from ALIX Studio? | **Partial: borrow some, build the rest** (verdict unchanged). There is no offset tool for joint tracks. The FK, the Jacobian, the singularity law, the pad axes and the continuity test can be borrowed. The "ease-in rule for offsets" cannot: it is a swivel-angle ramp from C-space authoring (X2). | **Partial: borrow some, build the rest** (verdict unchanged). The borrowable parts are the viewport, the robot model, the tube drawing, the gate chips and the confirm dialog. The collision engine needs correcting: the Studio's DAC sweep has **no fingers and no depth**, so the hand-and-toilet check must use `BodyCollisionWorld`, and the arm-against-arm check must use `cross_arm_separation`. |
| Main corrections | The joint-limit rule must budget for servo overshoot; ALIX measured J4 at 5.6-5.9° past its command. Rates must be judged band-limited. The 0.05 weight floor must go, the adaptive taper must be smoothed, and the ramps need an added-pad-speed bound. The pole option holds only to ±5 mm (lift tolerance), skips moves of 5 mm or less, and a pole step serves several programs. The magnitude limits are tightened. | Touch-off fits 4 DOF by default and uses the measured pole. The press check needs no mesh and is the main contact gate. The walls move with the toilet. The controller-SIM rung is required by default, as in ALIX contract C9. The slow first-run copy has its own SHA record, and the run must be proven complete. Live mirroring needs a joint stream from the runner, which does not exist today. |
| Priority | Should | Should. Ship it with R9: its gate is mandatory for any offset file. |
| Effort | L (12-15 person-days) | L (12-15 person-days on top of the shared R3/R5/R10 stack) |

---

## 2. Verification results

### 2.1 Claims that did not hold

| # | Analyst claim | What the code or measurement shows | Evidence |
|---|---|---|---|
| 1 | "The Studio has an existing ease-in rule for offsets (10 % of the path, ≥ 0.05 m), reusable as the sub-region ramp" | It is a **swivel-angle (elbow arm-angle) offset** in degrees, eased in linearly over the first 10 % of a path only. It belongs to the C-space swivel-plan authoring (X2). It is not a Cartesian path offset and not a two-sided ramp. Only the idea carries over. | `TG/stroke_gate.py:296-298, 1827-1850, 1939-1957` |
| 2 | `do_animate_sequence` solves "at the IKSolver defaults of 2 mm / 3°" | 2 mm is the solver default, but the endpoint passes **8°** rotation tolerance by default | `TG/server.py:1674`; `WS/ik.py:270-273` |
| 3 | "A 5° allowance covers measured servo overshoot (J4 executed up to 5.9° past its command)" | It does not, because 5.9 > 5. ALIX itself says the commanded-margin gate "does not budget for overshoot — and never did". The production track crossed ±135° on J4 (135.055 / 135.366°) with no fault. ALIX's answer is the execution-model gate G1. | `FIND:6721-6737, 6754-6760`; `REPO/tools/taught_tracks.py:126, 172-173, 466-487`; `REPO/Resource/tracks/taught/EXECUTION_MODEL.json` |
| 4 | "Four of six production files already exceed a joint rating at the 5.0 ms gate clock (1.17-1.42×)" | Those are **raw single-step** rates. ALIX's gate G2 judges **20 Hz band-limited** rates at 5.0 ms. By G2 none of the six exceed a rating (R2_LIDTOP 0.91×, L_P01P02 0.83×). Across all 15 V6 files only R_P06P07 (1.38×) and R_P10P12 (1.08×) do. **[V]** | `REPO/tools/taught_tracks.py:102, 440-460`; §2.3 |
| 5 | R2_LIDTOP "2 q4 sign changes"; L_P03P05 "0 sign changes" | R2_LIDTOP has **4** true sign changes, with 428 points within 2° of a straight elbow in 5 runs. L_P03P05 (q4 −0.4…114.7°) has **2**, so it also passes through a straight elbow. **[V]** | §2.3 |
| 6 | Repair step "flooring weights below 0.05 to 0" | This creates a one-tick jump of 5 % of the offset. Per 10 mm it adds **105 mm/s** of pad speed, against 33 mm/s without the floor. On R2_LIDTOP it also slowed convergence: after 10 passes 1 point still failed, against 0 failing after 3 passes without it. **[V]** | §2.3 |
| 7 | Repair step "halve adapt_k over ±0.3 s" (no smoothing stated) | A box-shaped halving left 10 jumps, raised the raw rate use to 1.84×, and produced target steps of **1,000 mm/s**. The adaptive weight must be smoothed (Hann). **[V]** | §2.3 |
| 8 | Pole option is "an exact pole-height change" | The geometry is exact, but the lift is not. `pole.py` accepts arrival within **±5 mm** and **skips moves of 5 mm or less** ("already there"). Workflow pole steps go through this code. | `UC/pole.py:3, 29-34`; `UC/rm_upload_fast_v14_safe_transit.py:546-556` |
| 9 | Pole option "edits the workflow pole step and its wait time" | A pole step has no wait field (`{"type":"pole","mm":…}`). One pole step also serves **several programs**: on the LEFT, pole 150 covers L_P06P07, M3_OPNST01/02 and L_P10P12. The shortcut must insert its own pole step and restore the height afterwards. | `V6/LEFT_V6_70SERIES_FINAL.json` (steps 7-12); `V6/RIGHT_V6_70SERIES_FINAL.json` |
| 10 | "The pole matters … for the pivot of a toilet turn" | A turn about a vertical axis commutes with the vertical pole translation, so the IK result is pole-independent for robot-frame translations and vertical-axis turns. The pole matters for collisions, for touch-off FK and for display. | `URDF:2656-2663` (prismatic +Z, zero rpy) |
| 11 | R14: "Run `compute_dual_arm_plan_collisions` with `extra_managers` = {commode, walls} … use `BodyCollisionWorld.depths` for depth" | `compute_dual_arm_plan_collisions` checks **arm links + bare palm only, with no fingers** and returns **no depth**. Hand-to-toilet contact, which is exactly what offsets change, would be missed. `BodyCollisionWorld` (active arm and hand, walls, commode, bin, with depths) and `cross_arm_separation` (hands included) are the right engines. | `WS/dual_arm_plan_collision.py:165-182, 600-648, 855-891`; `TG/body_collision.py:499-520, 912-925` |
| 12 | R14: "IRH_right_thumb_2↔commode_lid … about 2.2 mm deep" | Re-run: 9 of 188 samples, **up to 7.7 mm** deep. The 800 mm venue adds R_Link5↔wall on 28 of 188 samples, up to 17.2 mm; these vanish at 1400 mm. **[V]** | §2.3 |
| 13 | R14: the controller SIM-mode run is "optional" | ALIX contract C9 makes the **controller SIM rung required before REAL**: a failed SIM blocks REAL with no override, and a missing SIM may be overridden only with a recorded reason. SIM is not collision evidence, but it is the controller's own acceptance of the exact bytes. | `DISP/ladder.py:1-12, 112-145`; `DISP/runmode.py:1-16` |
| 14 | R14: "Gate chips … stale on edit" | Chips render pass/warn/fail/unrun/info and can disable a button. Only the coverage verdict knows about edits (`state.dirty`), so invalidation is new work. | `TG/web_editor/editor.js:15760-15790, 15904-15924` |
| 15 | R14: "the emulator and controller SIM have no … joint-rate prediction" | The emulator docstring itself points to "controller SIM for joint loads". The controller SIM is a real check of controller-side acceptance. The emulator is not evidence. | `REPO/core/python/src/alix_emulator/rm_emulator.py:56-70` |

### 2.2 What the analyst missed

- **Overshoot-aware limit gate already exists** (G1). It requires a predicted executed margin ≥ 0.5°, with overshoot = max(0.85°, 0.139 s × arrival rate). The legacy fallback is: 5° commanded margin, or 2-5° only where the 20 Hz band-limited rate ≤ 10°/s. Sources: `REPO/tools/taught_tracks.py:126, 172-173, 420-487`; `EXECUTION_MODEL.json`. R9 and R2 should share it.
- **The lift tolerance (±5 mm)** bounds the vertical repeatability of every program, not only the pole option (`UC/pole.py:3`). Vertical offsets smaller than this are not meaningful unless the lift is positioned more precisely.
- **Stowed-bin interlock caveats.** The band was measured on the BinShort model, but "the PHYSICAL part has not been remade" (`REPO/ros/description/ugv_description/urdf/bin.xacro:32-34`). The V6 flows also start at pole 200 mm, which is inside the stowed refusal band (`V6/*_FINAL.json` step 1). So the bin state during Lite flows must be known before the interlock means anything.
- **The teach-time toilet pose is unknown.** "The real base placement during teaching is unknown, so positions are only relative" (map 13 §6.2(d)). A mesh-based pad-to-surface check therefore needs the toilet model registered first. A press check along the pad's own axis needs no mesh.
- **Touch-off joint capture already exists in the runner**: `--drag-session` and `--drag-save` record and save a drag-teach trajectory (`UC/rm_upload_fast_v14_safe_transit.py:912-990, 2303-2306`). The pole is read with `rm_get_lift_state` through `pole.py` (`UC/pole.py:22-24`). A one-click read verb is still nicer for operators.
- **No live joint stream exists** in UI v53 or runner v14. A grep for realtime or UDP finds nothing, and the runner reads `arm_state()` only between steps (`:516, :612`). Live mirroring needs a new runner output.
- **"Completion and limit-stop are indistinguishable from the run state."** A real-test verdict must check that the program reached its end (`FIND:5859-5864`).
- **Controller SIM left on is a hazard**: "every dispatch would succeed with nothing moving" (`DISP/runmode.py:248-253`).
- **4105 (speed over limit) is warning-only in the runner. 4109 (arm collision) stops.** See `UC/rm_upload_fast_v14_safe_transit.py:56-57`.
- **`atlasTubeRibbon` lives in the X2 atlas code, and the ghost material in X1 path-gen code.** Copy the function and its helper `atlasRotateAbout`. Do not import those modules (`TG/web_editor/editor.js:11235-11299, 18505-18517`).
- **alix_perception_core is no help for "calculate".** It does region segmentation with self-reported poor quality and has no fixture pose estimate (`REPO/core/python/alix_perception_core/README.md:1-16`).

### 2.3 Re-measurements [V]

Single-threaded Python, user-site numpy 2.4.6 / scipy 1.17.1. The pad frame is `{S}_glove_frame_2` at the program's own pole. I implemented the solver from the analyst's text.

| Check | Analyst | Verifier | Result |
|---|---|---|---|
| R2_LIDTOP q4 range / sign changes / min margin / share < 10° | −86.7…91.5 / 2 / 0.00° / 23.6 % | −86.7…91.5 / **4** / 0.00° / 23.6 % | corrected |
| L_P01P02 | −82.6…91.2 / 2 / 0.06° / 11.9 % | same | confirmed |
| L_P03P05 | −0.4…114.7 / 0 / 0.00° | −0.4…114.7 / **2** / 0.00° | corrected |
| R10_HNGR min joint-limit margin | "1.5° from J4 limit" | 1.53° (J4 133.47° vs 135°) | confirmed |
| Rate use, R2_LIDTOP | 1.42× | raw 1.42×; **20 Hz band-limited 0.91×** | method corrected |
| V6 files over a rating (G2 method) | "4 of 6" | **2 of 15** (R_P06P07 1.38×, R_P10P12 1.08×). Raw: 10 of 15. | corrected |
| R2_LIDTOP +10 mm Z | 1 pass, 0 jumps, 0.08 mm, 65.7 % full | 3 passes, **0 jumps**, 0.08 mm, 59.5 % full, band-limited rate 0.91→0.93× | approach confirmed; coverage depends on implementation |
| same, with the 0.05 floor | — | 10 passes, 1 point still failing, 54.5 % full | defect |
| same, box-shaped adaptive halving | — | 10 jumps, raw rate 1.42→1.84×, 1,000 mm/s target step | defect |
| same, no taper (control) | 19.96° step, 42.7 mm, 174° excursion | 442 jumps, 10.55° step, 40.0 mm, J4 176° from taught, band-limited rate 5.41× | confirmed |
| L_P01P02 +10 mm Y | 0 jumps, 0.44 mm, 80.5 % | 0 jumps, 0.44 mm, 80.5 %, rate 0.83→0.83× | confirmed (identical) |
| R10_HNGR +10 mm X | 100 % full | 0 jumps, 0.00 mm, 93.5 % (run with the floor) | confirmed |
| R10_HNGR +30 mm X | — | 0 jumps, 0.54 mm, **45.8 % full** | new: large offsets lose coverage |
| Sub-region R10_HNGR rows 1000-2000, +10 mm X | outside rows unchanged | outside rows unchanged (0.0); 109 mm/s at the ramp start, caused by the floor | confirmed, plus defect |
| Added pad speed from the elbow taper (±60-point Hann) | — | 33 mm/s per 10 mm of offset (≈3.3 mm/s per mm) | new: ramps must scale with the offset |
| Largest joint change from taught (+10 mm) | — | 9.8-9.9° (J4); q4 sign kept wherever \|q4\| ≥ 2° | new testable criterion |
| Solver time | 2-17 s per file | ≈2.2 s per pass per 3.75 k points, so ≈6 s per pass on an 11 k-point file | 30 s needs a local re-solve |
| BodyCollisionWorld baseline, R2_LIDTOP (closed lid, pole 0.150, sampled every 20th point) | thumb–lid 9/188 "~2.2 mm"; wall hits at 800 mm | thumb–lid 9/188 **up to 7.7 mm**; R_Link5–wall 28/188 up to 17.2 mm at 800 mm, none at 1400 mm; 3.0-3.4 ms per pose | confirmed, depth corrected |
| scipy under isolated python3 | fails | fails (numpy 2.4.6 in /usr/local against system scipy) | confirmed |

---

## 3. R9 — Offset Adjustment Using IK (corrected)

**Original text.** "Offset Adjustment Using IK: The software should allow users to apply position offsets using Forward Kinematics (FK) apply offset to each point then inverse kinematics (IK) making sure the path is traceable no adjuscent point jumps, recalculate the joint positions, and test the adjusted trajectory. - obeyin the joint limits"

### 3.1 Revised requirement
- **(a) Scope.** The operator can move the cleaning path of:
  1. a whole program (one `.txt`);
  2. a sub-region, meaning a row range chosen on the joint plot;
  3. every program of both arms in the flow, as a "toilet moved" fixture offset.
- **(b) Frames.**
  - Robot frame (`base_link`): towards/away from the toilet ±X, left/right ±Y, up/down ±Z.
  - Pad frame: press harder/lighter along the pad's contact axis, or slide sideways.
  - Toilet frame: a shift plus a small turn about the vertical axis.
- **(c) Magnitude.** These are proposals for the requester to confirm.
  - The limit applies to the **largest pad displacement the request causes anywhere on the path**, covering translation and turn together. Resolution 0.5 mm. Warn above 10 mm; refuse above 30 mm and advise re-teaching.
  - Press-in along the pad axis: 5 mm or less.
  - Toilet turn: ±3° or less.
  - Reason: measured full-offset coverage falls to 46 % at +30 mm on R10_HNGR and 40 % at +20 mm on R2_LIDTOP.
- **(d) Computation.** For each point the software computes FK at the region's pad frame (pole from the workflow), applies the offset, and solves an IK that follows the taught motion. It keeps the point count and the timing, and writes integer-millidegree NDJSON.
- **(e) No jumps.** For every adjacent pair, each joint's step is at most max(1.5 × the taught step, 0.5°) and never more than 10° (fault 0x4000). No joint wraps by 360°. The elbow sign is kept: sign(q4) equals the taught sign wherever |q4 taught| ≥ 2°.
- **(f) Joint limits.**
  - The solver keeps every joint ≥ 5° inside the URDF limits (±178/130/178/135/178/128/360°), or no closer than the taught track already went.
  - The output must also pass the **same joint-margin gate as R2** (ALIX G1): predicted executed margin ≥ 0.5°, with overshoot = max(0.85°, 0.139 s × arrival rate). Without the model, the legacy rule applies: commanded margin ≥ 5°, or 2-5° only where the 20 Hz band-limited rate ≤ 10°/s.
  - The offset must never reduce any joint's predicted executed margin compared with the input file.
- **(g) Accuracy.** Where the full offset is applied, the pad is within 1 mm and 0.5° of the requested pose.
- **(h) Honest reduction.**
  - Where the full offset is impossible (straight elbow, reach, joint limit), it is reduced smoothly.
  - Each reduced span is reported with its time, the mm achieved, the reason, and its **effect along the pad's press axis relative to the toilet**: "may press harder by X mm", "may miss by X mm" or "sideways only".
  - For a toilet-moved offset, a span where the pad would sit more than 2 mm deeper than taught **blocks Apply**.
- **(i) Smooth joins.**
  - Every ramp and taper is smooth (raised cosine or Hann), with **no hard weight floor**.
  - Sub-region ramps last at least max(10 % of the span, 60 points).
  - Every ramp and taper is long enough that the extra pad speed it causes stays ≤ 25 mm/s (proposed). For example, a 10 mm offset needs at least 0.63 s.
- **(j) Pole option.** A vertical-only robot-frame offset of a whole program, **of at least 10 mm**, may be done by changing that program's pole height. This keeps the geometry exact and gives 100 % coverage even at straight-elbow passages. Conditions:
  - the new pole is between 0 and 315 mm;
  - with the bin stowed (the bin state must be known), the new pole is outside 153-282 mm;
  - Lite inserts its own pole step before the program and restores the previous height after it;
  - the operator is told the lift lands within ±5 mm;
  - dual-arm timing is re-checked, because the pole travel time changes.
- **(k) Versions.** The result is a new file version with a provenance record. The original is never overwritten, and the change is one labelled undo step.
- **(l) Testing.** The adjusted file must pass R14 before any real run or upload.

### 3.2 Clarifications
- **What a "position offset" is.** It is a rigid displacement of the taught pad path: a translation in the robot, pad or toilet frame, plus an optional small turn about the vertical axis. The pad's orientation relative to the surface is kept.
- **"Traceable, no adjacent point jumps" made testable.** Per-joint step ≤ max(1.5 × taught step, 0.5°) and ≤ 10°, no 360° wrap, and the q4 sign pattern preserved.
- **An exact offset at every point is not always possible.** At straight-elbow passages an exact offset does not exist physically, so "apply offset to each point" cannot be met literally there. Measured: R2_LIDTOP takes the full 10 mm on only about 55-66 % of its points. Smooth, reported reduction is the honest form.
- **Which pad frame matters.** For robot-frame and toilet-frame offsets the TCP choice does not change the result, because all flange frames move rigidly together. The pad frame matters only for "press harder/lighter" and for reporting.
- **When the pole matters.** For translations and vertical-axis turns the IK result does not depend on the pole, because the pole is a pure +Z translation upstream of the arm. The pole still matters for collisions, touch-off and display, so it is always taken from the workflow.
- **Which joint limits.** The limits are the URDF limits, which are stricter than the per-side controller drive limits. A flat 5° allowance **does not** cover the measured overshoot: J4 went 5.6-5.9° past its command and crossed ±135° with no fault (`FIND:6721-6760`). So limits are judged on the predicted executed motion (G1).
- **How rates are judged.** Rates use ALIX G2: 20 Hz band-limited at the 5.0 ms gate clock. Raw single-step rates overstate them (R2_LIDTOP raw 1.42× against band-limited 0.91×).
- **Why Lite must compute the offset.** Controllers ignore work and tool frames for joint programs; the runner changes frames only around global-point `movel`. The offset must therefore be computed in Lite.
- **Pipeline order.** Trim → Smooth → **Offset** → Speed → Gates. The offset is stored as a parameter and re-applied whenever anything upstream changes.
- **"Test the adjusted trajectory"** is R14.

### 3.3 Findings

| Source | Finding | Evidence |
|---|---|---|
| ALIX Studio | No offset feature for taught joint tracks. The route table has no offset endpoint. `do_retarget` only re-expresses a Cartesian task YAML, and refuses if the flange moves more than 1e-6 m or the delivered file moves more than 0.5 mm. | `TG/server.py:5961-6028, 2964-3128, 3068-3071, 3113-3116` |
| ALIX Studio | The Studio's IK uses belong to X1/X2. `do_animate_sequence` seeds each Cartesian waypoint's IK from the previous solution, at 2 mm and 8° by default, with random restarts and no jump check. `/api/path-ik` is a closed-form continuation along a geometric runtime path. | `TG/server.py:1645-1760, 1674`; `WS/ik.py:270-291`; `TG/server.py:5946-5958` |
| `TG/stroke_gate.py` | `track` is a runtime-calibrated continuation. A step is legal when \|J(q_from)·dq − pose error\| ≤ 1 mm / 0.5°; it is bisected up to 6 times, then raised as `StrokeFailure('jump'\|'limit'\|'unreachable')`. It matches the runtime to within 0.16°. It is tied to the X1 RuntimePath, so borrow the test, the constants and the failure wording, not the function. | `:57-62, 112, 129-130, 939-993, 1005-1158` |
| `TG/stroke_gate.py` (correction) | `PSI_OFFSET_EASE_*` eases a **swivel-angle** offset in over the first 10 % of a path. It is C-space swivel-plan authoring (X2), not a path-offset rule. Reference only. | `:296-298, 1827-1850, 1939-1957` |
| `TG/singularity.py` | Four vendor types; NEAR 10°, gate default 15°, floor 5°. The controller does not singularity-check streamed joint points. "q4 = 0 … the one that bites this corpus." | `:1-80` (`:36, 66-67, 76-78`) |
| `WS` (used by the Studio) | `UrdfModel.fk/fk_batch` are bit-identical and match the controller TCP to 0.007 mm median. `IKSolver.jacobian` is public, and `locked` pins the pole. `solve()` restarts from random seeds, so it hops branches. `SrsArm.branches_at` sorts by limit margin. | `WS/urdf_kinematics.py:218-259`; `REPO/experiments/exp01_teach_trajectory_reverse_engineering/src/teachlab/kinematics.py:3-9`; `WS/ik.py:150-169, 229-231, 282-291`; `WS/srs.py:772-817, 878-888` |
| V6 tracks **[V]** | Straight-elbow passages: R2_LIDTOP 4 q4 sign changes, minimum margin 0.00°, 23.6 % of points under 10°. L_P01P02 2 changes, 0.06°. L_P03P05 2 changes, 0.00°. R10_HNGR (17.8°) and R15_BUPFUL (22.8°) are far from singular. By G2 (band-limited) only R_P06P07 and R_P10P12 of the 15 V6 files exceed a rating. | §2.3 |
| Measured solver **[V]** | Re-implemented independently from the description: feed-forward, DLS with null-space pull, polish, straight-elbow pre-taper and smoothed adaptive taper. It gave **0 jumps** on R2_LIDTOP +10 Z, L_P01P02 +10 Y and R10_HNGR +10/+30 X, with a maximum residual of 0.54 mm. Without a taper the arm hops branch (J4 176°, 40 mm residual). Coverage is implementation-dependent (R2_LIDTOP 59.5 % here, 65.7 % for the analyst). | §2.3 |
| Measured defects **[V]** | The 0.05 weight floor adds a one-tick jump of 5 % of the offset (105 mm/s per 10 mm) and hurts convergence. Box-shaped adaptive halving leaves jumps (raw rate 1.84×). Even smooth tapers add about 3.3 mm/s of pad speed per mm of offset. | §2.3 |
| Sub-region **[V]** | Raised-cosine ramps leave rows outside the span unchanged (0.0) with a residual of 0.00 mm (R10_HNGR rows 1000-2000, +10 mm X). | §2.3 |
| URDF / V6 / orchestrator / UC | The pole is prismatic +Z with zero-rotation parents, so a whole-program vertical offset equals a pole change. V6 runs at 50/150 mm and starts at 200 mm. The stowed-bin band of 0.158-0.277 m is widened to 0.153-0.282 m, and the pole at 0.150 m keeps 7.2 mm of real clearance. The band was measured on BinShort, and the physical bin has not been remade. The lift accepts ±5 mm and skips moves of 5 mm or less. A pole step serves several programs. | `URDF:2656-2663`; `V6/*_FINAL.json`; `REPO/ros/orchestration/alix_orchestrator/alix_orchestrator/core/task_config.py:62-124`; `REPO/ros/description/ugv_description/urdf/bin.xacro:32-34`; `UC/pole.py:3, 29-34`; `UC/rm_upload_fast_v14_safe_transit.py:546-556` |
| `TG/config/glove_frames.yaml` + URDF | `glove_frame_2` presses +Z on both arms; frames 1, 3 and 4 press ±X, mirrored. The pad band (thickness) is 20 mm. `glove_frame_2` sits at (±0.0135, 0, 0.165) on ConnectorLink. | `:14-27, 52-54, 81-82, 97-109`; `URDF:4466-4480` |
| `REPO/tools/taught_tracks.py` | Gate clock 5.0 ms, 5° allowance, 10° step fault, 10 mm deviation (5 mm preferred), and the near-limit rule (≤ 10°/s inside the allowance, 2° floor). G1 uses the execution model (overshoot max(0.85°, 0.139 s × arrival rate), eps 0.5°). G2 uses 20 Hz band-limited rates. | `:102, 126-132, 172-173, 420-487`; `REPO/Resource/tracks/taught/EXECUTION_MODEL.json` |
| `FIND` | J4 executed +5.6/+5.9° past its command and reached 135.055/135.366° with no fault. "Treat 135.0 as a number the servo can cross." "The position-margin gate does not budget for overshoot." | `FIND:6721-6737, 6754-6760` |
| Vendor SDK | `rm_algo_ik_remote` offers joint-velocity weights, a q3 tracker and a limit hold. It is exported by the x86 and arm `libapi_c.so`. ALIX never calls it. Its documentation forbids q4 = 0. | `REPO/external/butterfli-realman-alix/Python/Robotic_Arm/rm_robot_interface.py:6932-7090`; `nm -D`; `REPO/docs/sources.md:5241` |
| Control app (high level) | No offset feature. Frames change only around global-point `movel`. The pole step is the vertical lever, carried out by `pole.py` within ±5 mm. | `UC/rm_upload_fast_v14_safe_transit.py:1488-1520, 546-556`; `UC/pole.py:3` |
| TRAJ | No kinematics (no match for "offset" or "kinemat"). `plotly_selected` → `addRegion` gives a row range, but `addRegion` refuses unless the Scale stage is on, so it needs adapting. There is also a cursor shape. | `TRAJ:872-889, 1392-1397, 1134-1141` |
| `DISP/frames.py` | The controller frame is Ry(+90°) of the arm base, and the pole carries the base. Resolving at the wrong pole height displaces waypoints (225 mm in the cited case). | `:1-17` |

### 3.4 Borrow verdict
**ALIX Studio: Partial — borrow some, build the rest.**

The Studio has no offset tool for taught joint tracks and no IK that follows taught motion. `do_retarget` refuses any change of motion; `do_animate_sequence` and `/api/path-ik` are X1/X2 previews.

Borrow, from the Studio and the `alix_workspace` code it uses:
- the FK (`UrdfModel.fk/fk_batch`) on the flat URDF;
- the Jacobian (`IKSolver.jacobian` with the pole locked, or `SrsArm.jacobian`). Do not use `IKSolver.solve`.
- `stroke_gate`'s linearisation continuity test and its `StrokeFailure` wording;
- `singularity.py` with its 10/15/5° thresholds;
- the `glove_frames` contact axes and the 20 mm band.

The PSI ease-in constants are a swivel-angle ramp (X2) and are reference only. Do not port `stroke_gate.track`.

Build new:
- the taught-following continuation solver;
- the smooth straight-elbow and adaptive tapers, with the added-speed bound;
- the span report with press-axis classes;
- the toilet-frame and touch-off transforms;
- the pole option, with step insertion;
- provenance and versioning;
- the offset panel and the batch mode.

**Other sources**

| Source | Verdict | Detail |
|---|---|---|
| `alix_workspace` (FK/IK/SRS/redundancy) | Borrow with adaptation | `UrdfModel` FK as-is, and the Jacobian inside the new solver. Closed-form `srs_ik.solve_all` is an optional cross-check where the margin is ≥ 15°. The redundancy corridor DP is reference only (X2). |
| `REPO/tools/taught_tracks.py` (+ `EXECUTION_MODEL.json`) | Port, shared with R2 | G1 (executed joint margin), G2 (20 Hz band-limited rate at 5.0 ms), G3 (≤ 10° step), the constants, and `deviation_mm` as a reference. For offsets, the primary accuracy metric is the per-index residual against the intended target. |
| `DISP/track.py` | Borrow as-is | `parse_track` (7 ints per line, ≥ 2 lines, refuses malformed lines). |
| `task_config.pole_bin_conflict` | Borrow, then re-measure | The band was measured on BinShort and the physical bin has not been remade. Confirm the band and the bin state on the real robot before relying on it. |
| Vendor SDK (`rm_algo_ik_remote`, Algo FK) | Reference only | An optional cross-check spike. It is closed source, and its tool frame is process-global. |
| TRAJ | Borrow UI pieces with adaptation | Range selection and cursor. `addRegion` is tied to the Scale stage. |
| Control app (UI v53 + runner v14) | Not available | No offset capability. The pole step is the vertical lever, with ±5 mm landing and moves of 5 mm or less skipped. |
| Analyst prototype | Build (approach reproduced) | Reproduced independently with 0 jumps on 4 real cases. Remove the 0.05 floor and smooth the adaptive weight. |

**Check of the input note.** The input makes no borrow claim. Its method (FK → offset at each point → IK, no adjacent jumps, joint limits) is sound but cannot be met literally. At straight-elbow passages an exact offset does not exist (R2_LIDTOP takes the full 10 mm on only about 55-66 % of its points), so the requirement must allow a smooth, reported reduction. "Obeying the joint limits" is necessary but not sufficient. ALIX measured J4 executing 5.6-5.9° past its command and crossing ±135° with no fault, so limits must be judged on the predicted executed motion (G1). Continuity, accuracy, rates and collisions must be gated as well. The limit set must also be named: URDF limits, which are stricter than the per-side drive limits.

### 3.5 Borrow items

| Item | From | Adaptation |
|---|---|---|
| URDF FK and the flat URDF | `WS/urdf_kinematics.py:218-259`; `URDF` | Bundle the URDF as a frozen asset. Use the chain `base_link → {S}_glove_frame_N` with the pole locked. |
| Geometric Jacobian, pole locked | `WS/ik.py:213-231` or `WS/srs.py:878-888` | Use inside the new solver only. Never call `IKSolver.solve` (random restarts). |
| Continuity test and failure wording | `TG/stroke_gate.py:112, 129-130, 1118-1156` | An independent jump detector on Lite's output, plus the report wording. `_unwrap_toward` (`:864-875`) is needed only if the closed-form cross-check is used. |
| Joint-margin, rate and step gates (G1/G2/G3) and the execution model | `REPO/tools/taught_tracks.py:102, 126-132, 172-173, 420-487`; `EXECUTION_MODEL.json` | One shared Lite gate module for R2 and R9 (and R14). |
| Vendor singularity law | `TG/singularity.py:36, 66-67, 94-205`; `WS/redundancy.py:120-140` | For warnings and the report. The taper uses only the q4 term. |
| Pad contact axes and band | `TG/config/glove_frames.yaml:81-109`; `TG/glove_frames.py` | The press direction, press-in limit and press-axis report. |
| Track parser | `DISP/track.py:66-82` | Input validation. |
| Pole/bin interlock | `task_config.py:62-124` | Applied to the pole option when the bin is stowed, after re-measuring on the real bin. |
| Row-range selection and cursor | `TRAJ:872-889, 1392-1397, 1134-1141` | Remove the Scale-stage dependency and use the selection as the offset scope. |

### 3.6 Build items

| Item | Approach |
|---|---|
| Taught-following continuation solver | Start each point at q′₍k−1₎ + (q_k − q₍k−1₎). Then 4 steps of dq = J⁺e + (I − J⁺J)·0.5·(q_k − q) with λ = 0.02, followed by 3 polish steps with λ = 0.005. Cap \|dq\| at 0.05 rad. Clip to URDF limits ∓5°, widened to the taught extremes. Weight 0 reproduces the taught track exactly. |
| Smooth tapers, repair and speed | Elbow pre-taper: clip((\|q4\| − 2°)/6°, 0, 1), then a sliding min, then Hann smoothing. Adaptive weight: multiply by (1 − 0.5 · a Hann-smoothed window mask of ±0.3 s) around failing points, never a box. **No hard floor.** Widen the windows until the extra pad speed is ≤ 25 mm/s. Re-solve only the affected windows, up to 10 passes. |
| Offset frames and transforms | Robot frame: Trans(w·d)·T. Pad frame: T·Trans(w·d). Toilet frame: M(w)·T, with M a 4-DOF (x, y, z, yaw) transform from touch-off, or a turn about a calibrated vertical pivot. Sign tests on both arms. |
| Span report with press-axis classes | For each span with w < 1: time window (at the 5 ms record clock, plus the playback band), mm achieved, reason class, and the shortfall projected on the taught pad's press axis ("may press harder / may miss / sideways"). Totals, singular-margin warnings, and G1/G2 before and after. |
| Pole option | Offer only for a whole-program, robot-Z-only change of 10 mm or more. Insert a dedicated pole step before the program and restore the previous height after it. Check 0-315 mm and the stowed band (bin state required). Show the ±5 mm landing note. Re-check dual-arm timing. |
| Output, provenance, versioning, undo | Round to millidegrees and re-check the gates. Write a new version plus a sidecar: source/output SHA-256, side, pole, pad, scope, frame, d/θ, weight summary, gate results, tool version. One labelled undo entry in the R2 history. |
| Offset panel and batch mode | Arrows and a number field per frame, the scope from the plot selection, Apply with progress, and a results card. Batch mode applies M to every program of both arms in worker processes. |
| Regression corpus | All V6 and raw taught files at ±10 mm on each axis. Checks: zero-offset identity, continuity, q4 sign, accuracy, G1 not worse, outside-span invariance, ramp speed, no single-tick weight jump, runtime. |

### 3.7 Recommendation
Build R9 as a Lite backend service, not as a port of any Studio feature. Use a taught-following differential continuation IK with:
- a straight-elbow pre-taper;
- a smoothed adaptive taper, with no hard floor;
- raised-cosine scope ramps whose length grows with the offset size, keeping the extra pad speed ≤ 25 mm/s.

The approach is independently reproduced: 0 jumps and ≤ 0.54 mm residual on four real cases. It reports honestly where the full offset is physically impossible.

Borrow:
- the FK and Jacobian from `alix_workspace`;
- `stroke_gate`'s continuity test, the singularity law and the glove contact axes from the Studio;
- ALIX's G1/G2/G3 gates, shared with R2.

The press-axis classification of reduced spans is the key safety addition: for a toilet-moved offset, a reduced span can push the pad into the moved toilet.

Offer the pole option only for vertical offsets of 10 mm or more, with its own pole step. Treat "toilet moved" (one 4-DOF transform for both arms) as the main guided case. Put the offset between Smooth and Speed, and ship it only with R14's gate.

### 3.8 Approach steps
1. **Parse.** Parse the `.txt` with the `DISP/track.py` rules. Take the pole from the preceding workflow pole step and the pad frame from the region (default `{S}_glove_frame_2`).
2. **FK.** FK every point with `fk_batch` over `base_link → pad`, pole locked: T_k.
3. **Targets.**
   - Robot frame: T′_k = Trans(w_k·d)·T_k.
   - Pad frame: T′_k = T_k·Trans(w_k·d).
   - Toilet frame: T′_k = M(w_k)·T_k.
4. **Weights.** w_k = scope_k · elbow_k · adapt_k, each smooth.
   - Scope ramps last at least max(10 % of span, 60 points, π|d|/(2·25 mm/s)).
   - elbow_k = clip((|q4| − 2°)/6°, 0, 1), then a sliding min, then Hann smoothing over at least ±60 points, widened for large |d|.
   - No hard floor.
5. **Continuation.** Per point as in §3.6. The first point gets 30 iterations.
6. **Per-point gates.** Residual ≤ 1 mm and ≤ 0.5°. Step ≤ max(1.5 × taught step, 0.5°) and ≤ 10°. Optionally, the `stroke_gate` linearisation test.
7. **Repair.** Around failing points, multiply adapt_k by a Hann-smoothed factor of 0.5 over ±0.3 s. Re-solve only the affected windows, up to 10 passes. If anything still fails, block Apply and log a defect; w = 0 reproduces the taught track exactly, so a remaining failure is a defect.
8. **File gates.** G1 executed margin (never worse than the input), G2 band-limited rate before and after (any joint newly over its rating is flagged for the speed stage), G3 ≤ 10°, q4 sign pattern kept.
9. **Report.** Each span with w < 1: time, mm achieved, reason, and press-axis effect. For a toilet-moved offset, more than 2 mm deeper blocks Apply. Add totals and warnings.
10. **Output.** Round, re-check, write a new version and the provenance JSON, never overwrite the source, and add one labelled undo entry.
11. **Pole option.** For a whole-program robot-Z offset with |dz| ≥ 10 mm, propose pole + dz. Check 0-315 mm and the stowed band with a known bin state. Insert a pole step before the program and a restore step after it, and show the ±5 mm note.
12. **Batch mode.** Apply M to every program of both arms in a process pool, then hand the set to R14 as one batch. Mark the region version "offset — not yet simulated" until R14 passes.

### 3.9 Acceptance criteria
1. A zero offset gives output identical to the input (every joint within 1 millidegree).
2. On all V6 and raw taught files, at ±10 mm along each robot axis:
   - every adjacent step is ≤ max(1.5 × taught step, 0.5°) and ≤ 10°;
   - there is no 360° wrap;
   - sign(q4) matches the taught sign wherever |q4 taught| ≥ 2°.
3. Where the reported weight is 1, the output FK is within 1.0 mm and 0.5° of the requested pose. The reported "mm achieved" matches an independent FK within 0.2 mm.
4. The output passes the shared G1 joint-margin gate wherever the input did, and no joint's predicted executed margin is smaller than in the input.
5. The G2 band-limited rate at 5.0 ms is reported before and after. A joint newly above its rating is flagged and blocks R14 until the speed stage fixes it.
6. Sub-region joins and ramps:
   - rows outside the span and its ramps are unchanged;
   - each ramp is ≥ max(60 points, 10 % of span);
   - the extra pad speed from any ramp or taper is ≤ 25 mm/s;
   - no single-tick weight jump exists.
7. R2_LIDTOP +10 mm Z lists every reduced span with its time, mm achieved, reason and press-axis effect. A toilet-moved offset with any span more than 2 mm deeper is blocked.
8. Pole option:
   - it is not offered for |dz| < 10 mm;
   - it is refused outside 0-315 mm, or inside 153-282 mm when the bin is stowed or its state is unknown;
   - it inserts its own pole step and a restore step, and shows the ±5 mm note.
9. The output parses with `DISP/track.py:parse_track` and with the runner, and keeps its point count. The original is untouched, and one undo restores the previous version.
10. Sign tests:
    - +10 mm in the robot frame moves the pad +10 mm along `base_link` X on both arms;
    - "press harder 3 mm" moves the pad +3 mm along Z of `glove_frame_2` on both arms.
11. A ±10 mm offset on any V6 file finishes in ≤ 30 s on the target PC, with progress shown. A decimated preview is ready in ≤ 5 s.

### 3.10 UX notes (non-technical operators)
- Use plain words and arrows: "Move path: Towards toilet / Away / Left / Right / Up / Down" and "Press harder / lighter". Provide ±1 mm and ±5 mm buttons and a number field.
- Choose the scope by dragging across the joint plot. Highlight the chosen span on the plot and in 3D.
- Never show solver words. Show a traffic-light card, for example: "Applied fully on 81 % of this path. Reduced at 2 places where the arm is fully stretched (0:04-0:05: may press 3 mm harder — blocked)." Each place gets a "Show me" button.
- For an up/down move of a whole program of 10 mm or more, offer: "Changing the pole height does this exactly (the pole lands within ±5 mm). Use pole instead?"
- Label undo entries, for example "Offset 10 mm towards toilet". Any offset leaves the region marked "needs simulation check".
- If less than about 80 % of a contact span can take the offset, or the request is above 30 mm, suggest re-teaching the region.

### 3.11 Safety notes
- Press-in along the pad axis is 5 mm or less, because the glove band is 20 mm and a hard surface can take large forces. For a toilet-moved offset, a reduced span that puts the pad more than 2 mm deeper than taught is blocking.
- The extra pad speed from tapers and ramps is bounded (25 mm/s proposed). The 0.05 weight floor and box-shaped tapers are forbidden, because they cause one-tick target jumps.
- Joint limits are judged on predicted executed motion (G1). R10_HNGR already commands J4 1.5° from 135°, and ALIX saw 5.6-5.9° of overshoot.
- Never move the pole or the hand while the pad is on a surface.
- The pole option has these limits:
  - the stowed-bin band was measured on a model bin that is 40 mm shorter than the physical one;
  - the V6 flows already start at 200 mm, so the bin state must be known;
  - the lift lands within ±5 mm and skips moves of 5 mm or less.
- An offset moves program start and end points. The controller's transit to the first point and the V6 bridges are not collision-checked, so they must be re-checked (R8/R14).
- Rates must not get worse. Measured band-limited R2_LIDTOP went 0.91 → 0.93×. A file newly over a rating must not be uploaded.
- An offset file never reaches the robot without passing R14.

### 3.12 Dependencies
- R14 (simulation, collision, gating).
- R2: pipeline order, labelled undo, and the **shared G1/G2/G3 gate module**.
- R7: pole and hand per program.
- R8: bridges and transits when start/end points move.
- R3/R10: 3D view and model.
- R1: region and sub-region IDs, and the pad frame per region.
- A Lite Python backend with its own venv (numpy/scipy, with the URDF bundled).
- Workflow JSON editing to insert pole steps.

### 3.13 Risks
- Exact offsets are impossible near straight-elbow passages: R2_LIDTOP gets the full offset on only about 55-66 % of its points. Large offsets lose coverage fast (R10_HNGR +30 mm: 46 %).
- Coverage depends on the implementation details (taper widths, gates). Acceptance is on gates, not percentages.
- The lift tolerance of ±5 mm bounds vertical repeatability for every program. Vertical offsets below this are not meaningful unless the lift is positioned more precisely.
- A toilet turn needs a calibrated pivot or a good touch-off fit.
- Performance: about 2.2 s per pass per 3.75 k points in Python, so whole-file repair passes exceed 30 s on 11 k-point files. A local re-solve and worker processes are needed.
- The host numpy/scipy ABI mismatch (verified) means Lite needs its own venv.
- Mixing FK sources gives mm-level differences. Keep one FK (the URDF).

### 3.14 Open questions
1. What are the typical size and type of misplacement on site (mm, any turn)? This sets the proposed 10/30 mm and ±3° limits.
2. Which pad frame does each region use?
3. Should offsets apply to raw/smoothed files and then be re-sped (recommended), or also to already-optimised production files?
4. Is a fixture offset always common to both arms?
5. What reduced-offset coverage on contact spans is acceptable before Lite advises re-teaching? Proposal: re-teach if less than 80 % of a contact span gets at least 90 % of the offset.
6. Is the bin stowed during Lite flows? (V6 starts at pole 200 mm, inside the stowed band.)
7. How repeatable is the lift's landing height in practice, and can its 5 mm tolerance be tightened?
8. May Lite copy ALIX's experiments-folder kinematics helpers (teachlab), or must it depend only on `alix_workspace`?

**Priority: Should. Effort: L (12-15 person-days)** for one experienced developer, including tests:

| Part | Person-days |
|---|---|
| Solver, smooth tapers, local re-solve and gates (G1 port shared with R2; 1 day less if R2 ports it first) | 5-6 |
| Span report with press-axis classes | 1 |
| Pole option with step insertion and interlock | 1-2 |
| I/O, provenance, undo | 1-2 |
| Panel and batch mode | 3 |
| Regression corpus | 2 |

This excludes the shared 3D/collision stack and R14.

---

## 4. R14 — Offset Calculation and Testing (corrected)

**Original text.** "Offset Calculation and Testing: Users should be able to calculate, apply, and test positional offsets in simulation before executing the adjusted trajectory on the actual robotic arms. - simulate offsets related to 9"

### 4.1 Revised requirement
- **(a) Calculate.** The offset can come from three sources:
  1. typed or nudged values;
  2. **touch-off**, described below;
  3. a correction observed in a real test.

  **Touch-off.**
  - The operator drag-guides the fingertip (`{S}_index_tip_frame`, hand in the SRDF "pointing" state, the same hand and glove state at both captures) to 1-3 or more reference marks.
  - The marks were captured at teach time, or retrofitted while the robot and toilet are still in the taught placement.
  - Lite records the joints and the **measured** pole height at rest, applies FK, and fits:
    - a translation from 1 mark;
    - translation plus a turn about the vertical axis from 2 or more marks (the default model);
    - a full rigid fit only from 4 or more marks spread at least 300 mm apart.
  - Lite shows the fit residual and refuses the result when it exceeds 3 mm.
- **(b) Preview.**
  - The 3D view shows the original pad path as a grey ghost and the offset path in blue; reduced spans are amber and refused spans red.
  - For toilet-frame offsets the toilet is drawn moved.
  - The robot plays the offset motion, with a toggle to show the original arm at the same row.
  - The joint plot overlays original and offset curves on one cursor.
- **(c) Simulation checks.** These run automatically after every change, on the offset motion and on the taught baseline, in the same world.
  - **World.** The toilet is at the region's lid/seat state. For a toilet-moved offset it is moved by the same transform, and the walls are checked **both moved and unmoved**: usually the robot was parked differently, but sometimes the toilet itself moved. The world also contains the Lite-cell walls, the bin per R10, the active arm **with its hand**, the partner arm on its paired program, and the pole and hand from the workflow.
  - **Engines.**
    - Each arm against the world: a per-pose check including the hand, with contact depth, at dt ≤ 0.02 s.
    - Arm against arm over time, including the hands.
    - The Studio's DAC sweep is not enough: it has no fingers and no depth.
  - **Verdict.** Only contacts that are new or ≥ 1 mm deeper than the baseline are reported; allowed pad contacts are listed separately.
  - **Press check (no mesh needed).** The pad's displacement along its own press axis, relative to the toilet, is compared with the taught track. It fails if the pad is deeper than requested by more than 2 mm, and warns "may miss" where it is more than 5 mm shallower on a contact span.
  - **Optional mesh profile.** A pad-to-surface distance profile is used only after the toilet model is registered.
  - **Gates.** The R9 gates, G2 rates after the speed stage, and bridges/transits (R8).
- **(d) Gate ladder.** The ladder is keyed by the SHA-256 of the exact bytes:
  1. **SIM3D** (this offline check);
  2. **CTRL-SIM**: a controller simulation-mode run of the uploaded program. It is required by default, as in ALIX C9. A failed run blocks REAL with no override; a missing run can be overridden only with a typed reason, which is logged.
  3. **REAL**.

  Any edit invalidates the records. The run mode is verified before every run and restored after a CTRL-SIM run. The real-run confirmation lists the gate chips and the offset summary.
- **(e) First real run.**
  - The first run uses a time-stretched copy (for example 50 %). The copy has its own SHA record: the geometry checks carry over because the joint path is unchanged, and the rate and limit gates are re-run. This is needed because the run speed % scales only the transit.
  - The verdict requires the arm to **reach the program's last point**, because a limit stop and a completion look the same in run_state.
  - Codes 4105 and 4109 are recorded.
  - Joints are mirrored live when the runner can stream them.
  - The operator's verdict is stored, and "Back to offset" returns to editing.

### 4.2 Clarifications
- "Calculate" means deriving the offset from a measurement (touch-off) or an observation, not only typing numbers.
- **Touch-off fits 4 DOF (x, y, z, yaw) by default.** The robot and the toilet both stand on the floor, and a 6-DOF fit from 3 hand-placed marks would add spurious tilt. With ±3 mm positioning noise over 300 mm, that is about 0.6°, or 5 mm at 0.5 m.
- **The pole matters for touch-off.** The pole height enters the fingertip FK directly, so it must be read at rest, not assumed. The lift lands within ±5 mm.
- **Two kinds of "simulation".** "Test in simulation" means Lite's offline 3D and collision check (SIM3D). The controller's SIM mode is a separate, cheap rung. It proves the controller accepts and completes the exact bytes, but it is not collision evidence. The in-repo emulator is not evidence either.
- **Why verdicts are differential.** Taught tracks are contact tasks, and in the model the taught R2_LIDTOP itself touches the lid with the thumb, up to 7.7 mm deep. Verdicts must therefore be differential against the taught baseline.
- **The teach-time toilet pose is unknown** (map 13 §6.2(d)). The press check along the pad axis needs no toilet model and is the primary contact gate. Mesh-based distances need the toilet model registered first.
- **A slow run reduces less risk than it seems.** It reduces collision energy and gives the operator time to react, but it does not reduce joint overshoot: ALIX found the slowest track overshot most (`FIND:6721-6737`).
- **The gate is enforced by content hash**, not by a reminder.

### 4.3 Findings

| Source | Finding | Evidence |
|---|---|---|
| ALIX Studio DAC endpoint | It accepts plan JSON under its roots and passes no `extra_managers`. It drops `joint_timeline`, and its cache is keyed by the plans' content SHA-1. | `TG/server.py:4300-4366` (call `:4355-4356`, pop `:4357`) |
| `compute_dual_arm_plan_collisions` (correction) | The checked links are the arm links plus the **bare palm, with no fingers**. The body set is `bin_link` and the glove lever. It returns **no depth**. The `extra_managers` hook is never passed by any caller. | `WS/dual_arm_plan_collision.py:165-182, 600-648, 699-703` |
| `BodyCollisionWorld` | Checks the active arm **and hand** against the partner (static), bin, covers, UGV, poles, walls and commode at an articulation state. It is ACM-, allowance- and rest-baseline-filtered, and `filter_fn.depths` gives depth in mm. It is validated only for `commode_c`. **[V]** 3.0-3.4 ms per pose. | `TG/body_collision.py:58, 401-464, 499-520, 905-925` |
| `cross_arm_separation` via `REPO/tools/dual_arm_verdicts.py` | The arm-vs-arm sweep with the "full" link set, which includes the hands. dt 0.02 s is a correctness value, the floor is 25 mm, and there is a swapped-arm guard. | `WS/dual_arm_plan_collision.py:855-891, 1089-1283`; `dual_arm_verdicts.py:20-33, 352-369, 580-692, 709-731` |
| Re-measured baseline **[V]** | Taught R2_LIDTOP, closed lid, pole 0.150. Thumb_2↔lid on 9 of 188 samples, up to 7.7 mm deep. The 800 mm venue gives R_Link5↔wall on 28 of 188 samples, up to 17.2 mm; these vanish at 1400 mm. The model world does not match the cell, and absolute verdicts would flag the taught track itself. | §2.3 |
| `TG/body_collision.py`, `TG/scene_pose.py`, `TG/web_editor/template.html` | The commode can be posed at closed / lid_open / lid_seat_open and shifted along +X by a standoff delta, as the C++ does. There is no general (dx, dy, dz, yaw) placement. The standoff fields are in the X1 generator and Regions panels. | `TG/body_collision.py:103-108, 401-464`; `TG/scene_pose.py:123-145, 161-214`; `template.html:241-242, 800-802` |
| `TG/web_editor/editor.js` | Reusable 3D pieces: `loadRobotGeometry`, `applyRobotPose`, the DAC goto/highlight/jump/log functions, and hit colour 0xff4d4d. | `:12352, 12427-12437, 12446, 14272-14299, 14501-14560` |
| `TG/web_editor/editor.js` | `atlasTubeRibbon` gives an exact station mapping with per-station colour. It lives in the X2 atlas code, so copy it together with `atlasRotateAbout`. The ghost material lives in the X1 path-gen code. | `:11235-11299, 18342-18348, 18505-18517` |
| `TG/web_editor/editor.js` | Gate chips and `showStudioModal` (Promise, chips, lines) can be reused. The chips are Studio-specific gates, and only coverage invalidates on edit. | `:5431-5462, 13786-13832, 15760-15790, 15904-15950` |
| `DISP/ladder.py`, `DISP/runmode.py` | Fingerprint ledger EMU → SIM → REAL, where SIM is **the controller's SIM mode on hardware**. A failed SIM blocks REAL with no override; a missing SIM can be overridden with a reason. The emulator geometry is not evidence. A controller left in SIM makes every dispatch "succeed with nothing moving". | `ladder.py:1-12, 68-72, 112-145`; `runmode.py:1-16, 248-253` |
| `FIND` | `plan_speed` scales ONLY the transit, and the speed persists as a global ratio. Completion and limit stop are indistinguishable from the run state. | `FIND:5854-5881, 5859-5864` |
| Studio simulate endpoints and control app | `/api/sim/joints` accepts external joint maps. The runner does not stream joints during runs (`arm_state()` only between steps), and there is no realtime/UDP feed in v53 or v14. | `TG/server.py:5326-5393`; `UC/rm_upload_fast_v14_safe_transit.py:122-140, 516, 612` |
| Control app (high level) | It already offers drag-teach record/save (usable for touch-off capture), a pole read through `pole.py`, and SIM/REAL mode get/set. 4105 is warning-only; 4109 stops. | `UC/rm_upload_fast_v14_safe_transit.py:56-57, 912-990, 2303-2306, 2317-2320`; `UC/pole.py:22-24` |
| URDF / SRDF | `{S}_index_tip_frame` is defined at the tip of the fully opened index finger, which matches the SRDF "pointing" state (index_1 0.01). | `URDF:4537-4556`; `REPO/ros/manipulation/alix_moveit_config/config/alix.srdf:715-722, 787-794` |
| `REPO/tools/track_report.py` | A 6-line Kabsch helper exists. A 4-DOF fit is a small new function. | `:683-688` |
| map 13 | The teach-time base placement is unknown; positions agree with `commode_c` only relatively. | map 13 §6.2(d) |
| alix_perception_core | Region segmentation with self-reported poor quality; no fixture pose estimate. | `REPO/core/python/alix_perception_core/README.md:1-16` |
| Emulator | "JOINT RATES … use a saved plan or controller SIM"; it does not emulate fence/collision config. | `REPO/core/python/src/alix_emulator/rm_emulator.py:56-70` |

### 4.4 Borrow verdict
**ALIX Studio: Partial — borrow some, build the rest.**

Borrow from the Studio:
- the 3D viewport, robot geometry and `applyRobotPose`, shared with R3/R10;
- the DAC highlight, Prev/Next and log UI, plus its frames and timeline for playback;
- `atlasTubeRibbon` (copied) and a translucent ghost material for the two paths;
- the commode articulation states and the standoff placement math;
- gate chips, `showStudioModal`, the status line and toast;
- the `/api/sim/*` pattern for live mirroring.

The Studio's DAC **sweep** is not the right engine for offsets, because it has no fingers and no depth. Borrow `BodyCollisionWorld` (per pose, hand included, depth) and `cross_arm_separation` (arm against arm, hands included) from the code the Studio already uses.

Missing in the Studio, to be built:
- `.txt` input through the existing shim;
- a general fixture and room transform;
- the baseline-differential verdict;
- the press check without a mesh;
- optional toilet registration;
- touch-off;
- the original-against-offset joint overlay;
- the SHA-keyed ladder with the controller-SIM rung and edit invalidation.

**Other sources**

| Source | Verdict | Detail |
|---|---|---|
| `BodyCollisionWorld` (`TG/body_collision.py`) | Borrow with adaptation | Synthetic task config (ik_frame, fixture state, hand stage), `pole_override_m`, the partner state from the workflow, a general room transform on the fixture root and walls (new), the Lite venue, and one worker process. |
| `cross_arm_separation` + `REPO/tools/dual_arm_verdicts.py` | Borrow as-is (shim and sweep) | `track_points`, `build_timeline` (pole and hand seeded), `timeline_to_plan`, `assert_arm_sides`, `sweep` at dt 0.02 s, with the 25 mm floor. |
| `compute_dual_arm_plan_collisions` | Reference only for R14 | No fingers, no depth. Its frames and timeline can drive DAC-style playback. |
| `DISP/ladder.py` + `DISP/runmode.py` | Borrow with adaptation | Rungs SIM3D → CTRL-SIM → REAL, keyed by the output SHA-256 plus its workflow context. The ledger lives in Lite's data directory. Run-mode verify and restore. |
| Control app (UI v53 + runner v14) | Partial: borrow some, build the rest | Real run and stop, the SIM/REAL mode switch, drag-teach capture and the pole read all exist. Missing: a joint stream during runs (for the live mirror) and, ideally, a one-click joint-read verb. |
| TRAJ | Borrow with adaptation | Joint plot cursor and shapes for the original/offset overlay, linked to the 3D scrub. |
| Emulator | Not applicable as evidence | No collision model; its geometry is explicitly not evidence. |
| alix_perception_core | Not applicable | No fixture pose estimate. |

**Check of the input note.** "Simulate offsets related to 9" is accurate as a dependency, and the input makes no borrow claim. "Simulation" must be defined as Lite's offline 3D and collision check. ALIX's own contract adds a controller-SIM rung before REAL, which Lite should keep. ALIX Studio cannot simulate a `.txt` today, and its DAC sweep would miss hand contacts.

### 4.5 Borrow items

| Item | From | Adaptation |
|---|---|---|
| 3D viewport, robot geometry, pose application | `TG/web_editor/editor.js:12352-12437`; `TG/server.py:1918-1998` | Trim the X1/X2 overlays and feed FK frames from the `.txt`, both original and offset. |
| Path tubes | `editor.js:11235-11299` (copy with `atlasRotateAbout`) | Two tubes: original grey and translucent; offset blue with per-point amber/red. Decimate for display. |
| World check per pose, including the hand, with depth | `TG/body_collision.py:401-464, 499-520, 905-925` | Pose the fixture root (and walls) by M. Synthetic config. Sample every 4th point (dt ≤ 0.02 s). Run on baseline and offset and diff the depths. |
| Arm-vs-arm sweep with hands | `REPO/tools/dual_arm_verdicts.py:580-731`; `WS/dual_arm_plan_collision.py:1089-1283` | Offset pair against baseline pair; flag new pairs under the 25 mm floor or a smaller minimum. |
| DAC highlight/navigation/log | `editor.js:12446, 14272-14299, 14501-14560` | Link to joint-plot bands, not the 0/1 bar chart. |
| Commode articulation and placement math | `TG/scene_pose.py:123-214`; `TG/body_collision.py:103-108` | Add a general rigid transform on the fixture root. |
| Pair classes and allowed contacts | `WS/redundancy.py:95-117`; `REPO/ros/orchestration/alix_tasks/config/contact_links.yaml:84-98` | Plain-language classes; separate allowed pad contact from collision. |
| Gate chips, confirm modal, status and toast | `editor.js:5431-5462, 13786-13832, 15904-15950` | Lite gates: limits, steps, accuracy, rates, collisions, press, bridges, controller SIM. Invalidation is new. |
| Fingerprint ladder and run-mode checks | `DISP/ladder.py:1-145`; `DISP/runmode.py` | SIM3D → CTRL-SIM → REAL; override only for a missing rung. |
| Kabsch helper | `REPO/tools/track_report.py:683-688` | Base for the 4-DOF and 6-DOF touch-off fits. |
| Drag-teach capture and pole read | `UC/rm_upload_fast_v14_safe_transit.py:912-990, 2303-2306`; `UC/pole.py:22-24` | Touch-off capture fallback: take the median of the still tail. A one-click read verb is preferred. |

### 4.6 Build items

| Item | Approach |
|---|---|
| Touch-off | Mark capture at teach time, or a retrofit in the verified taught placement. Use the pointing hand state and the same glove state. A one-click joint read (or a short drag-teach recording) plus a pole read at rest. Store {fixture setup, side, mark, joints, pole, hand}. FK at `{S}_index_tip_frame`. Fit translation (1 mark), 4-DOF x/y/z/yaw (2 or more, default), or 6-DOF only with 4 or more spread marks. Show the residual; refuse above 3 mm. |
| Moved-room world | Compose M onto the fixture root for rendering and collision. Check the walls both moved and unmoved, and keep the more conservative verdict. |
| Differential world and arm-vs-arm verdicts | Same world for baseline and offset. Report pairs that are new or ≥ 1 mm deeper (`depths`). Re-run the arm-vs-arm sweep with hands. Use plain-language classes. |
| Press check (mesh-free) | From R9's FK: the pad displacement relative to the toilet (offset FK minus M·taught FK, or minus taught FK when the toilet did not move), projected on the taught pad press axis. Fail if deeper than requested by more than 2 mm; warn if more than 5 mm shallower on contact spans. |
| Optional toilet registration | Fit the commode model pose (x, y, yaw, z) to the taught contact-pad points, or measure it with touch-off. Only then show a pad-to-surface distance profile. |
| Original-vs-offset joint overlay | Dashed original and solid offset curves, shaded reduced spans, one cursor shared with the 3D scrub. |
| Gate ledger, controller SIM rung, invalidation | Append-only records {sha256, rung, gates, user, time, tool version}. CTRL-SIM: upload, set SIMULATION, run, confirm the last point was reached, set REAL and verify. Edits re-lock. Overrides need a typed reason and apply only to a missing rung. |
| Slow first real run | A stretched copy through the R2 scale stage, with its own SHA and gate record. Confirmation modal, completion check, record 4105/4109 and the operator verdict, then allow the full-speed run. Live mirror when the runner can stream joints (R4). |

### 4.7 Recommendation
Build R14 as the mandatory test half of R9, on the shared Lite simulation stack (R3/R5/R10).

Use these engines:
- `BodyCollisionWorld` for each arm and its hand against the toilet, walls and bin, with depths;
- `cross_arm_separation` through the `dual_arm_verdicts` shim for arm against arm, with hands.

The Studio's DAC sweep misses fingers and gives no depth. Keep its UI (highlight, Prev/Next, log), the tubes, gate chips and confirm modal.

Make the press check along the pad axis the primary contact gate. It needs no toilet model, and the teach-time toilet pose is unknown. Add mesh distances only after the toilet model is registered.

Use touch-off with a 4-DOF fit and a measured pole as the main way to calculate a toilet-moved offset. Gate every real run and upload by the output SHA-256 through SIM3D → CTRL-SIM → REAL, as in ALIX's C9 ladder. Run a time-stretched copy first, and prove the run reached its last point.

### 4.8 Approach steps
1. **Touch-off.**
   - Capture the reference marks: pointing hand, same glove, one-click joint read or a short drag recording, and the pole read at rest.
   - At a new placement, capture the same marks, FK both sets, and fit translation, 4-DOF or 6-DOF depending on the mark count.
   - Show the result in plain words with the residual; refuse above 3 mm.
2. **Instant preview.** Draw the original path (grey ghost) and the requested path (blue) before solving. Draw the toilet moved for toilet-frame offsets.
3. **Solve.** Run R9 in a worker, recolour the offset tube per point, and compute FK frames for both motions. The robot plays the offset; "show original arm" swaps in the taught frames at the same row.
4. **World check.**
   - Build the baseline and offset timelines with `dual_arm_verdicts.build_timeline` (pole and hand seeded).
   - For each arm, run the `BodyCollisionWorld` filter and depths at dt ≤ 0.02 s (every 4th point). The fixture is at the region state and posed by M; the walls are checked moved and unmoved; the partner state comes from the workflow; the bin follows R10.
   - Diff against the baseline.
5. **Arm against arm.** Run `dual_arm_verdicts.sweep` on the offset pair and the baseline pair. Flag a new contact or a minimum distance that drops below 25 mm.
6. **Press check.** Mesh-free, along the pad axis. Use the optional mesh profile only if the toilet is registered.
7. **Gates.** Limits, steps, accuracy (R9), G2 after the speed stage, collisions, press, bridges/transits (R8). Show them as chips that go stale on any edit.
8. **SIM3D record.** On a pass, append {sha256, rung SIM3D, gates, user, time}.
9. **CTRL-SIM.** Upload the exact bytes, set SIMULATION, run, check the last point was reached, set REAL and verify, then record the result. A failure blocks REAL.
10. **Real test.**
    - The confirmation modal lists the chips and the offset summary.
    - Run the stretched copy, which has its own record.
    - Check completion, record 4105/4109 and the operator verdict, then allow full speed.
    - "Back to offset" returns to the panel.
11. **Performance.** Cache sweeps by SHA-256 and run collision work in one worker process, because FCL managers are not thread-safe.

### 4.9 Acceptance criteria
1. Touch-off, from recorded joint sets:
   - a known 10 mm shift is recovered within 0.5 mm;
   - with 2 marks at least 300 mm apart, a known 2° turn is recovered within 0.3°;
   - a residual above 3 mm is refused with a plain message;
   - the fit uses the measured pole.
2. Preview:
   - both paths are drawn, reduced spans amber and refused spans red;
   - the moved toilet is shown;
   - scrubbing the plot cursor moves the 3D robot to the same row.
3. World check:
   - runs at dt ≤ 0.02 s per arm with the hand included;
   - lists only contacts that are new or ≥ 1 mm deeper than the baseline, with allowed pad contacts listed separately;
   - a deliberately injected 20 mm press into the lid is detected.
4. The arm-vs-arm sweep (hands included) is re-run on the offset pair and reports any new contact or a minimum below 25 mm.
5. The press check flags every span deeper than requested by more than 2 mm, with a "Show me" link.
6. Gate:
   - "Test on real robot" and "Upload" stay disabled until the exact SHA-256 has a SIM3D pass and a CTRL-SIM pass, or a logged override for a missing CTRL-SIM;
   - a failed CTRL-SIM cannot be overridden;
   - any edit re-locks;
   - the run mode is checked before every run.
7. The first real run uses a stretched copy with its own passing record. The verdict requires reaching the last point, and 4105/4109 and the operator verdict are stored.
8. After a dual-arm "toilet moved" batch, the arm-vs-arm sweep is re-run for every paired step and summarised in one table.

### 4.10 UX notes
- Use a guided flow for the main case: "The toilet is not where it was when we taught" → Measure (touch the marks) → Preview → Check → Test slowly → Upload.
- Give plain verdicts, for example: "No new collisions. The pad keeps the same pressure on 96 % of the path. 2 places may press up to 3 mm harder." Each comes with a "Show me" button.
- A greyed-out button always says why ("Run the simulation check first" / "You changed the offset after the last check" / "Controller test (no motion) not done yet").
- Keep the 3D colours consistent: grey original, blue offset, amber reduced, red refused or colliding (0xff4d4d).
- Touch-off uses pictures of the marks, one "Capture" button per mark, a beep, and a live residual readout.
- Show the controller run mode (SIM/REAL) as a large badge whenever a robot action is possible.

### 4.11 Safety notes
- The simulation pass licenses only the exact bytes (SHA-256). A changed file must be re-checked, and the slow copy has its own record.
- The controller SIM mode is not collision evidence, but a failed controller SIM blocks REAL. Always verify and restore the run mode: a controller left in SIM silently moves nothing.
- The first real run of an offset file is slowed with a stretched copy, because the speed % only scales the transit. A slow run does not reduce joint overshoot. The verdict must confirm the last point was reached.
- Contact deeper than the taught baseline means higher pad forces. More than 2 mm deeper (press check) is blocking; 1 mm or more in the collision world is a warning that needs explicit acceptance.
- In the real test, 4109 (collision) stops the run, and 4105 (speed over limit) is warning-only in the runner. Lite treats either as a failed test and returns to editing.
- Touch-off is done in drag mode at low speed. Never move the pole or hand while the fingertip touches the toilet.
- The collision world must match the cell: walls, commode pose, and the bin, since the model bin is 40 mm shorter than the physical one.

### 4.12 Dependencies
- R9 (offset engine and report).
- R3/R10 (3D view, URDF/meshes, bin toggle).
- R5 (collision engines and highlight).
- R7 (pole and hand per program).
- R8 (bridge and transit re-check).
- R4 (real test, IP entry, SIM/REAL switch, and a runner joint stream for the live mirror).
- R13 (upload gated by the same ledger).
- R2 (scale stage for the slow copy; G1/G2 gates).
- C8 control-app integration (drag-teach capture or a one-click read, pole read). Plan-blocking: v53 Multi Run and Page-01 Load Sequence raise TypeError (missing `@staticmethod`, `UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py:4853, 4876, 4885, 5000`).
- Lite-cell calibration (venue walls, commode pose; optional toilet registration).

### 4.13 Risks
- The collision world may not match the cell. Re-measured: the default 800 mm venue gives false wall hits, `commode_c` is the only validated fixture, the teach-time toilet pose is unknown, and the model bin differs from the physical one. The result can be false alarms or missed contacts. The mesh-free press check limits the damage.
- Touch-off accuracy depends on the fingertip frame, the glove, positioning by hand (likely ±2-3 mm) and the lift's ±5 mm landing. A yaw from 2 marks degrades when the marks are close together.
- Sweep cost: about 3 ms per pose per arm for the world check, plus the arm-vs-arm sweep (seconds to minutes with the full link set). Needs caching and one worker.
- Gate fatigue may lead to routine overrides. Overrides are allowed only for a missing rung and stay visible.
- The v53 Multi Run / Load Sequence TypeError blocks those dual-arm paths until fixed.

### 4.14 Open questions
1. Are there, or can there be, physical reference marks on each toilet? Who records them at teach time? Can marks be retrofitted for the existing V6 programs?
2. Is touch-off done gloved or ungloved, and can the gloved hand take the "pointing" state?
3. Which venue walls and commode pose calibration match the Lite cell? Is the site toilet a `commode_c`?
4. Should the controller SIM rung be mandatory, as ALIX C9 requires (recommended), and who may override a missing one?
5. Are 2 mm (press check) and 1 mm (deeper contact) the right thresholds?
6. Where is the ledger kept, and who reviews it?

**Priority: Should**, and it must ship with R9. **Effort: L (12-15 person-days)** on top of the shared R3/R5/R10 stack:

| Part | Person-days |
|---|---|
| Ghost and joint overlays | 2 |
| Moved-room world | 1-2 |
| Differential world check, arm-vs-arm check and press check | 3-4 |
| Ladder with CTRL-SIM, run-mode checks, invalidation and override log | 2 |
| Touch-off (capture, marks, 4-DOF fit, UI, hardware trial) | 3-4 |
| Slow-run flow | 1 |

Optional toilet registration adds M (3-5 days). The live mirror needs a runner joint stream (R4).

---

## 5. Cross-cutting notes
- **One kinematic model.** The flat URDF with `UrdfModel` FK drives the offset IK, the 3D view, the collision checks and the gates. The vendor Algo is a cross-check only (process-global tool frame; Arm_Tip 15.3 mm from ConnectorLink).
- **Shared gate module.** ALIX's G1 (executed joint margin with overshoot), G2 (20 Hz band-limited rate at 5.0 ms) and G3 (≤ 10° step) should be one Lite module used by R2, R9, R11 and R14. A flat 5° margin does not cover the measured 5.6-5.9° J4 overshoot, and raw single-step rates overstate rate use (R2_LIDTOP raw 1.42× against band-limited 0.91×).
- **Pipeline position.** Offsets are a non-destructive stage: Trim → Smooth → Offset → Speed → Gates. They are re-applied when upstream edits change, produce a new versioned file with provenance, and count as one labelled undo step.
- **Smoothness rule for every edit stage.** Weights and ramps must be C¹-smooth, with no hard floors. The extra pad speed they cause is bounded (25 mm/s proposed). The same applies to R2 scale regions and R11 smoothing.
- **Collision engines.** Use `BodyCollisionWorld` (per pose, hand included, depths) for the fixture, walls and bin, and `cross_arm_separation` (hands included) for arm against arm. The Studio DAC sweep (arm links + palm, no depth) suits only playback UI. Verdicts on contact tracks must be differential against the taught baseline in the same world. This applies to R5, R8, R11 and R14.
- **One ladder for all edits.** SIM3D → CTRL-SIM → REAL, keyed by the output SHA-256 (`DISP/ladder.py`, C9), should be the single Lite mechanism for all edits (R2, R9, R11) and uploads (R13). Run mode is verified before and after every robot action.
- **Lift tolerance.** `pole.py` accepts ±5 mm and skips moves of 5 mm or less. This bounds the vertical repeatability of every program and the precision of any pole-based offset.
- **Bin state.** The stowed-bin interlock was measured on the BinShort model, and the physical bin has not been remade. The V6 flows command pole 200 mm, which is inside the stowed band. The bin state during Lite flows must be confirmed.
- **Moved start and end points.** Offsets and merges move them. The controller's transit to the first point and the V6 bridges are not collision-checked, so they must be re-checked (R8).
- **Dedicated venv.** Running ALIX code needs one: under isolated python3, numpy 2.4.6 (/usr/local) fails against the system scipy (verified).
- **Control app, high level.** Neither v53 nor v14 has an offset or IK feature. Joint programs ignore controller frames. The pole step is the vertical lever, within ±5 mm. Drag-teach record/save can capture touch-off joints. No joint stream exists during runs. 4105 is warning-only and 4109 stops. Plan-blocking: v53 Multi Run and Page-01 Load Sequence raise TypeError (`UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py:4853, 4876, 4885, 5000`).
- **X1/X2 boundary.** The offset engine uses IK internally, which is allowed. Do not port `/api/path-ik`, auto-path, the redundancy atlas, the PSI swivel-offset machinery, or any C-space/workspace UI. Copy `atlasTubeRibbon` and its helper as standalone functions.

---

## 6. Change log

| Req | Change | Reason | Evidence |
|---|---|---|---|
| R9 | Reclassified the "ease-in rule for offsets" from a borrow item to reference only | It is a swivel-angle offset ramp in C-space authoring (X2), one-sided and linear, not a path offset | `TG/stroke_gate.py:296-298, 1827-1850, 1939-1957` |
| R9 | `do_animate_sequence` tolerance corrected to 2 mm / 8° | The endpoint default rot_tol is 8.0 | `TG/server.py:1674` |
| R9 | Joint-limit rule (f) now uses the G1 executed-margin gate and "never worse than input". The claim that 5° covers overshoot is removed. | The 5.9° J4 overshoot exceeds 5°; ALIX says the commanded gate never budgeted for overshoot | `FIND:6721-6737, 6754-6760`; `REPO/tools/taught_tracks.py:126, 172-173, 466-487` |
| R9 | Rate figures re-based on G2 (20 Hz band-limited at 5.0 ms) | Raw single-step rates overstate them: 2 of 15 V6 files are over by G2, not "4 of 6" | `REPO/tools/taught_tracks.py:102, 440-460`; [V] §2.3 |
| R9 | Corrected q4 sign-change counts (R2_LIDTOP 4; L_P03P05 2) | Re-measured | [V] §2.3 |
| R9 | Removed the 0.05 weight floor; required a smoothed adaptive taper and an added-pad-speed bound with offset-scaled ramps | The floor gave one-tick jumps (105 vs 33 mm/s per 10 mm) and hurt convergence; box halving left 10 jumps and 1,000 mm/s target steps | [V] §2.3 |
| R9 | Acceptance is on gates, not coverage percentages; added a testable q4-sign criterion | Coverage is implementation-dependent (59.5 % vs 65.7 %); the no-taper branch hop flips q4 | [V] §2.3 |
| R9 | Pole option restricted to \|dz\| ≥ 10 mm, with its own inserted pole step and restore, a ±5 mm note, a sync re-check and a known bin state | Lift tolerance 5 mm and skip of ≤ 5 mm; a pole step serves several programs; no wait field; BinShort model; V6 starts at 200 mm | `UC/pole.py:3, 29-34`; `UC/rm_upload_fast_v14_safe_transit.py:546-556`; `V6/*_FINAL.json`; `bin.xacro:32-34`; `task_config.py:62-95` |
| R9 | Magnitude limits proposed at 10/30 mm on the resulting pad displacement, turn ±3° (was 20/50 mm, ±5°) | Coverage falls to 46 % at 30 mm and 40 % at 20 mm; turn and translation must share one cap | [V] §2.3; analyst table §3.2 |
| R9 | Added press-axis classification of reduced spans and the 2 mm blocking rule for toilet-moved offsets | A reduced span of a fixture offset leaves the pad up to (1−w)·\|d\| inside the moved toilet | `TG/config/glove_frames.yaml:14-16, 99-100` (press axis) |
| R9 | Pole-independence clarified (it also holds for vertical-axis turns) | Translations and Z-rotations commute with the +Z pole translation | `URDF:2656-2663` |
| R9 | Performance criterion now requires local re-solve | Measured ≈2.2 s per pass per 3.75 k points; 11 k points × multiple passes exceeds 30 s | [V] §2.3 |
| R9 | Noted that TRAJ `addRegion` is tied to the Scale stage | It refuses unless Scale is enabled | `TRAJ:1392-1397` |
| R9 | Effort L 11-14 → L 12-15 | Adds the G1 port, smooth tapers with local re-solve, and pole-step insertion | — |
| R14 | Engines corrected: `BodyCollisionWorld` (hand, depth) and `cross_arm_separation` (hands) instead of the DAC sweep with `extra_managers` | The DAC sweep checks arm + palm only and returns no depth | `WS/dual_arm_plan_collision.py:165-182, 600-648, 855-891`; `TG/body_collision.py:499-520, 905-925` |
| R14 | Baseline numbers updated (thumb–lid up to 7.7 mm; wall hits 28/188 up to 17.2 mm at 800 mm, none at 1400 mm; 3.0-3.4 ms per pose) | Re-run | [V] §2.3 |
| R14 | Mesh-free press check made the primary contact gate; mesh profile only after toilet registration | The teach-time toilet pose is unknown | map 13 §6.2(d); `glove_frames.yaml:99-100` |
| R14 | Controller-SIM rung required by default (failure blocks REAL; override only for a missing record); run-mode verify and restore | ALIX contract C9 and the ladder; SIM-left-on hazard | `DISP/ladder.py:1-12, 112-145`; `DISP/runmode.py:1-16, 248-253` |
| R14 | Touch-off: 4-DOF default fit, measured pole, same glove state, retrofit only in the verified taught placement, drag-teach capture fallback | A 6-DOF fit from 3 hand-placed marks injects tilt; the pole enters the FK; capture verbs exist | `UC/rm_upload_fast_v14_safe_transit.py:912-990, 2303-2306`; `UC/pole.py:22-24`; `URDF:4537-4556` |
| R14 | Walls checked both moved and unmoved for toilet-moved offsets | The cause (robot parked differently or toilet moved) is unknown | reasoning; `TG/scene_pose.py:123-145` |
| R14 | Slow first run: its own SHA record, a completion check, and the note that it does not cut overshoot | Speed % scales transit only; completion and limit stop look the same; the slowest track overshot most | `FIND:5854-5881, 5859-5864, 6721-6737` |
| R14 | The live mirror depends on a new runner joint stream (R4) | None in v53/v14; `arm_state()` is read only between steps | `UC/rm_upload_fast_v14_safe_transit.py:516, 612` (grep: no realtime/UDP) |
| R14 | 4105 is warning-only in the runner; Lite treats it as a failed test | Runner behaviour | `UC/rm_upload_fast_v14_safe_transit.py:56-57` |
| R14 | Gate-chip invalidation listed as new work; `atlasTubeRibbon` and the ghost material are to be copied, not imported | Only coverage is edit-aware; the functions sit in X2/X1 code | `editor.js:15760-15790, 11235-11299, 18505-18517` |
| R14 | Added alix_perception_core as not applicable for "calculate" | No fixture pose estimate | `REPO/core/python/alix_perception_core/README.md:1-16` |
| R14 | Effort L 10-12 → L 12-15 | Adds the CTRL-SIM rung, the engine split, the press check and the touch-off fit | — |
