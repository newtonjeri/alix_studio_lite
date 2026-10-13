# ALIX Studio Lite — architecture proposal (lens: reuse-maximiser)

Author role: architect, reuse-maximiser lens. Audience: the engineers who will build Lite, and the author of the inputs.
Status: proposal for review. Nothing in any repository was changed to write it; every claim below was read in place or taken from the evidence maps and re-checked where a verdict depends on it.

## Path aliases used in citations

| Alias | Absolute path |
|---|---|
| `ALIX/` | `/home/newtonjeri/alix_ws/src/alix/` (git branch `feature/alix_studio`) |
| `TG/` | `ALIX/core/python/alix_taskgen/src/alix_taskgen/` (ALIX Studio: `server.py`, `web_editor/`) |
| `WS/` | `ALIX/core/python/alix_workspace/src/alix_workspace/` |
| `DP/` | `ALIX/core/python/alix_dispatch/src/alix_dispatch/` |
| `EMU/` | `ALIX/core/python/src/alix_emulator/` |
| `LITE/` | `/home/newtonjeri/alix_studio_lite_ws/` |
| `UC/` | `LITE/upload_channel/` |
| `V53` | `UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py` |
| `R14` | `UC/rm_upload_fast_v14_safe_transit.py` |
| `HTML` | `LITE/Alix_Studio_Lite_Trajectory_V2.html` |
| `V6/` | `LITE/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/` |
| `REQ` | `LITE/alix_studio_lite/docs/reference/ALIX_Studio_Lite_requirement_inputs.txt` |
| `MAP-nn` | evidence maps in `/tmp/claude-1000/-home-newtonjeri-alix-studio-lite-ws/747fb97d-6769-47bf-aaab-d34d82139984/scratchpad/maps/nn_*.md` |

"(inferred)" marks a conclusion that was not observed directly. "pd" = person-days for one experienced developer, tests included. Effort scale: S ≤ 3 pd, M 4–8, L 9–15, XL > 15.

---

## 0. Summary

- **What Lite is.** A guided, operator-friendly application that takes drag-taught joint-space recordings region by region, optimises them (trim, smooth, speed up), checks them in a dual-arm 3D simulation with the commode, tests them on the robot and uploads them. It excludes geometric path generation (X1) and the C-space and Workspace features (X2) (`REQ:3-5`).
- **Where it should live.** A new sibling package in the ALIX monorepo, `ALIX/core/python/alix_studio_lite/`. It **imports** the ROS-free ALIX libraries instead of copying them. It is **not** a mode of the ALIX Studio server: that option was weighed honestly (§2) and rejected for the product, but it is kept as a zero-code test harness in Phase 0.
- **What is reused, and how.**
  - **Imported:** `alix_workspace` (FK, IK, SRS, collision, dual-arm sweep, plan frames, SRDF states), `alix_dispatch` (upload/list/run/stop/run-mode/pole/hand, emulated program channel), the light `alix_taskgen` modules (`body_collision`, `scene_pose`, `approach_transit`, `singularity`), `alix_emulator`, and the hardware-validated taught-track functions once they are promoted out of `tools/`.
  - **Copied, with a provenance header and a drift test:** the Studio server scaffold, about 400 Python lines that cannot be imported without the generator stack, and about 1,900 lines of Studio front-end JS/CSS (viewport, robot overlay, Dual-Arm Collisions playback, feedback toolkit, docks and dialogs, Board).
  - **Referenced, not copied:** the design tokens and the vendored JS libraries.
  - **Vendored byte for byte, sha256-pinned:** the control app's runner v14 with `hand.py`/`pole.py`. This is the real-arm executor from day one.
  - **Ported with golden-file parity tests:** the trajectory HTML's pure algorithms and the v53 dual-timeline planner.
- **New code** is limited to what nothing in ALIX or the workspace provides:
  - the region registry and guided teach flow (R1)
  - per-segment recipes with a 5-deep undo (R2)
  - joint plots that stop and highlight at a collision (R5)
  - a 3-way Simulation / Dry run / Real switch with IP entry (R4)
  - a continuity-engineered offset solver (R9)
  - a collision-checked merger (R8)
  - spike suggestions (R12)
  - an upload ledger (R13)
- **Plan:** 10 phases, **about 100 pd** in total (§5). An operator-usable cut covering R1–R7 and R10–R13 lands after Phase 6 (about 70 pd). Merge (R8) and offset (R9/R14) follow. By rough estimate, reuse saves 50–70 pd against a from-scratch build (§8).

---

## 1. Background — what the existing programs do (high level)

This section answers the user's request for a high-level understanding of the Python programs and the related backend and front-end files. It is deliberately not a bug audit; only plan-blocking issues are listed, one line each.

### 1.1 The control application (`UC/`)

| File | What it does |
|---|---|
| `rm_trajectory_ui_v53_safe_lid_checkpoint.py` (11,581 lines, Tkinter) | **The operator console. It never talks to the robot itself:** every button spawns the runner (below) as a subprocess and reads the lines it prints (`V53:5733-5961`). **Page 01 "Manual + Setup"** has two side-by-side arm panels, LEFT 192.168.1.10 and RIGHT 192.168.1.103. Each panel has the tabs **RUN** (upload a `.txt` to a controller program slot, list slots, run stored programs), **POINTS** (controller global waypoints, MoveJ/MoveL), **DRAG TEACH** (record, save `.txt`, play back), **HAND/POLE** (six hand channels 0–1000 with GRIP/ALL OPEN presets; pole 0–315 mm) and **TELEMETRY**. Each panel also has SIM / REAL / CHECK buttons for the controller's run mode. Dual tools: start/stop both drags, run both `.txt` files, run both stored programs (synchronised by a thread barrier). **Page 02 "Dual Sequence"** has per-arm workflow builders (Program / Pole / Hand / Wait / Global steps), dual rules TOGETHER / RUN FIRST / WAIT OTHER, the "SAFE TIMELINE" planner with the manual lid-cover checkpoint, an overall speed and a live arm status (MAP-08 §4). |
| `rm_upload_fast_v14_safe_transit.py` (2,702 lines) | **The "runner": the command-line program that does all robot I/O through the RealMan SDK.** **Upload:** `.txt` → controller project file (CRLF, `{"file":7}` header) → `rm_send_project` → verify by stored size. It also lists slots, runs a slot while watching run state and error codes, and runs sequences/workflows over one connection. Its other verbs are drag-teach session/save/playback, manual pole/hand, run mode, and global points. "Fast" means fast switching between programs inside one process; **the upload itself is not faster** (MAP-09 §3.3). Lines 1–268 are the ALIX `tools/rm_upload.py` (MAP-09 §0). `_v52support.py` and `_v53support.py` are byte-identical copies. |
| `hand.py` | Inspire hand over the arm's end-port Modbus: 6 channels, 0–1000 where 1000 = open, arrive/settle logic, freeze (`UC/hand.py:3-17`). |
| `pole.py` | Pole (lift) read/move/stop with the measured slew model, 0.059 m/s up, 0.097 m/s down, plus a 1.5 s creep (`UC/pole.py:3-14`). |
| `usage.py` | A small CLI over `hand.py`/`pole.py`. It includes `prestroke` (open → pole → hand) and `stop` (arm + pole + hand), and states the rule "the pole moves only with the arm still; never move pole or hand with the pad on a surface" (`UC/usage.py:1-25`). |
| `README_COMMANDS.md` | How to upload/run with the plain uploader. |
| `py programs/`, `upload/`, `upload.zip` | History: UI v8…v48 and runner v4…v12, plus an older snapshot. |
| `zip/`, `plan_txt_files/` | Sample lid-top tracks: idle-trimmed raw files and versions sped up to 20–40 %. |
| `LEFT/RIGHT_V6_70SERIES_FINAL.json` | The production dual workflow (`realman_workflow_sequence_v1`). |

**Which version is current:** **UI v53 + runner v14** (`V53:45-46` declares `UI_PROGRAM_VERSION="v53"`, `EXPECTED_RUNNER_VERSION="v14"`). v48, named in the inputs, is three versions older and fully contained in v53. v48 and v51 reject the V6 production workflow; v53 runs it with the manual lid checkpoint (MAP-08 §0, §1.8). The "RealMan Dual-Arm Operations UI.txt" note describes "UI v50 / Runner v13" and is out of date on the Page-02 dual rules (MAP-12 §4).

**Plan-blocking issues (one line each; everything else is out of scope here):**
1. Multi Run and Page-01 Load Sequence raise `TypeError`: `_step_preview` and `_normalize_*_for_main` lack `@staticmethod` (`V53:4853, 4876, 4885, 5000`; called at `V53:4932, 5083, 5159, 5212`). Lite should not port Multi Run; workflows supersede it.
2. The UI has no IP entry (IPs are display-only labels). The runner accepts `--ip`/`--port` and `ip = a.ip or ARM_IPS.get(a.side)` (`R14:2285-2286, 2347`), so Lite can pass the IP the operator types.
3. The runner version is not enforced. If v14 is missing, the UI falls back to v13, which would run Program transits at 100 % (`V53:6890-6909`; MAP-09 §1.1). Lite must pin the runner by sha256.
4. "SIM/REAL/CHECK" is the **controller's** dry-run mode, not a 3D simulation, and the lift and hand are not simulated in it (MAP-09 §7). Lite needs its own 3D preview.
5. Stop works by SIGINT → `KeyboardInterrupt` → `rm_set_arm_slow_stop`, and the runner installs no signal handler (`grep signal R14` = 0 hits). On Windows `CTRL_BREAK_EVENT` would end the process without the slow stop (inferred, MAP-09 §8).
6. A stop made outside the UI (pendant) that leaves no error code is reported as a successful completion (MAP-12 §2b). Lite must label such completions "not verified".
7. Program-ID ranges collide: V6 uses 71–83, and ALIX reserves 70–100 for production on the same controllers (MAP-10 §5.2). Lite needs a declared slot range.

### 1.2 ALIX Studio (`TG/`) — backend and front-end files

| File | What it does |
|---|---|
| `server.py` (6,566 lines) | **Local web server** (stdlib `ThreadingHTTPServer`, `TG/server.py:30`) with 64 JSON routes (`ROUTES_GET` at `:5961`, `ROUTES_POST` at `:5988`), a per-route "needs" table (`:6064`), one global generation lock (`:389`) and a job/poll model for multi-minute generation. It serves robot meshes (`do_robot_geometry`, `:1918`), FK frames for plan playback (`do_plan_frames`, `:4590`), dual-arm collision sweeps (`do_dual_arm_collisions`, `:4335`), SRDF pole presets (`:1763`), a live view of the in-process emulator (`/api/sim/*`, `:5309-5393`), plus generation (X1) and workspace/C-space analysis (X2). It **never commands the robot**. |
| `web_editor/template.html` (953), `editor.js` (19,325), `editor.css` (2,166) | A **single-page app** with a three.js 3D viewport and seven tabs: Regions (mesh painting, X1), Board (region × arm table), Workbench (X1), Dual-Arm Collisions, Schedule & Run, Workspace (X2) and Gizmo. It also has a Simulate view, a test-pinned light/dark token theme (brand `#2F96EE`) and a rich feedback toolkit (status line, toast, confirm modal, gate chips, tooltips, help). |
| `web_editor/vendor/` (5.1 MB) | three.js r128, OrbitControls, TransformControls, **Plotly 2.35.2** (the same version the trajectory HTML loads from a CDN) and js-yaml. Everything is offline. |

### 1.3 The trajectory prototype (`HTML`, 2,096 lines)

A single static page. It loads **one** `.txt`, removes idle stretches (rolling-mean rule), smooths with a 101-point moving mean, speeds up user-chosen row windows by linear resampling, compares the result with the original and saves it. It also offers heuristic "speed suggestions" (equal-length chunks). It has no 3D view, no limit or rate checks, no undo and no dual-arm view, and it loads Plotly from a CDN (`HTML:7`; MAP-07 §0–§9).

### 1.4 ALIX libraries that Lite can lean on (all ROS-free, `ALIX/LAYERS.md` §1)

- `alix_workspace`:
  - FK: `UrdfModel.fk_batch`, 2.5 µs per point, matches the controller
  - IK: `IKSolver`, `SrsArm`, `srs_ik`, `swivel_arcs`
  - collision: `RobotCollisionModel`, `compute_dual_arm_plan_collisions`, `compute_plan_frames`, `PlanWallChecker`
  - SRDF states
  
  (MAP-05)
- `alix_dispatch`:
  - track I/O and upload/list/run with verified stop (`DP/track.py`)
  - `open_arm(side, ip)` (`DP/connect.py:141`)
  - controller run mode (`DP/runmode.py`)
  - pole and hand (`DP/peripherals.py`)
  - the emulated program channel (`DP/emulated_program.py`)
- `alix_emulator`: an in-process RealMan SDK double (Linux/Windows).
- `ALIX/tools/`:
  - the hardware-validated taught-track math: `strip_pauses`, `smooth_taught`, `capped_millideg`, `deviation_mm`, gates G1–G5 (`ALIX/tools/taught_tracks.py:420-958`)
  - the `.txt`→plan shim (`ALIX/tools/dual_arm_verdicts.py:531-731`)
  - the join-continuity primitive `concatenate` (`ALIX/tools/plan_min_time.py:142`)

---

## 2. Where Lite should live — the options, argued honestly

| Criterion | **A. Lite as a profile/mode of the ALIX Studio server** | **B. Sibling package in the ALIX monorepo** (recommended) | **C. Standalone package in `LITE/` with vendored copies** (MAP-04 §7.4) |
|---|---|---|---|
| Python reused by import | Highest: existing handlers plus libraries | High: every ROS-free library, plus the light `alix_taskgen` modules and the promoted `tools/` functions | Low–medium: `alix_workspace` as a wheel; copies of the rest |
| Code copied | About 0 Python. JS must still be copied, or `editor.js` refactored | About 400 lines of server scaffold + about 1,900 lines of JS/CSS, each with a provenance header and a drift test | As B, plus copies of `track`/shim/taught math (about 1,500+ Python lines) that then drift |
| Startup | 30–60 s: `server.py` imports the generator stack at module import (`TG/server.py:36-44`; MAP-04 §3.2) | Seconds: no generator import, lazy model build, baked overlay | Seconds |
| Studio regression risk | High: shared handlers, route tables and tests; DAC semantics must change for Lite (commode, `joint_timeline`) | Low: the upstream changes are small and keep their defaults (§3.10) | None |
| Operator laptop | Needs the full monorepo checkout, plus the `vendor/realsense-ros` sibling for meshes (MAP-04 §2.3) | A layout-preserving **sparse bundle** built from a pinned commit (§3.11) | Its own bundle |
| ALIX fixes reaching Lite | Automatic | Automatic for imports; flagged by the drift test for copied JS | Manual |
| Process-global hazards | Audit hook (`TG/server.py:6187`), interpreter-wide `open` patching during generation, one global lock | None of these | None |
| Coordination cost | Low: one owner (all 300 most recent monorepo commits are by `newtonjeri`) | Low: same owner; the seams need small ALIX commits | None |

**Honest notes.**
- The Studio's test that is usually quoted as "no execution channel" (`ALIX/core/python/alix_taskgen/test/test_studio_interaction_guards.py:2204-2215`) only pins the Schedule panel text "arms not connected". It does **not** forbid robot routes, so tests do not technically block option A.
- What rules A out is weight, coupling and the data model:
  - 30–60 s startup for an operator tool.
  - Repo-relative resolution that silently returns empty outside the repo (`TG/server.py:60-101`).
  - DAC/plan routes that accept only `.json` under plan roots (`TG/server.py:4187-4197`), ignore the commode (`:4354-4356`) and drop `joint_timeline` (`:4357`). Lite would need either changes to shared Studio handlers or parallel Lite handlers, and parallel handlers bring A's reuse down to the same library calls B makes anyway.
  - Release coupling to a branch with daily commits.

**Verdict:**
- **B for the product.** It gets C's deployment independence without C's copy debt: the release artefact is a bundle built from B.
- **A as a zero-code harness in Phase 0.** Convert taught `.txt` files to plan JSON with the existing shim, point `ALIX_PLANS_DIR` at a folder inside the repo, and inspect them in the unchanged Studio DAC tab. That folder must be inside the repo because `list_plans` uses `relative_to(REPO_ROOT)` (`TG/server.py:4182`); `.cache/` is git-ignored (`ALIX/.gitignore:44`).

---

## 3. Architecture

### 3.1 Process model

```
 Browser (Chromium/Edge/Firefox, offline)                     loopback only, port 8766
   lite.html  ── fetch /api/lite/* (JSON, gzip) ── poll jobs ──┐
                                                               │
 ┌─────────────────────────────── alix-studio-lite (one Python process) ─────────────────────────────┐
 │ HTTP layer: stdlib ThreadingHTTPServer, route tables + "needs" (copied Studio scaffold)           │
 │   needs: robot_io:<left|right> (one owner per arm) · collision_worker · project_write · job       │
 │ Job registry (start / status / cancel; copied Studio pattern) ── worker threads                   │
 │ Services (in-process, import ALIX libs): project store · optimise · gates · kin/offset · merge    │
 │ Robot layer: ControllerBackend  ┬─ RunnerSubprocessBackend (MVP) → runner v14 subprocess per action│
 │                                 ├─ DispatchBackend (target)       → robot-agent process per arm    │
 │                                 ├─ EmulatedBackend (dev/CI)       → robot-agent process + emulator │
 │                                 └─ (no backend) Preview mode = 3D simulation only                 │
 └──────────┬───────────────────────────────────────────────┬────────────────────────────────────────┘
            │ ProcessPool(max_workers=1, spawn)              │ subprocess / JSON lines
 ┌──────────▼──────────────────┐                ┌────────────▼─────────────────────────────┐
 │ Collision worker (warm)      │                │ runner v14 (verbatim) │ robot agent       │
 │ FCL managers: arms, self,    │                │ RealMan SDK, TCP 8080 per controller      │
 │ bin/body, commode @ state,   │                │ LEFT 192.168.1.10 / RIGHT 192.168.1.103  │
 │ walls; dual-arm sweep;       │                │ (IP entered by operator)                  │
 │ per-pose filter; windowed    │                └───────────────────────────────────────────┘
 │ re-check                     │
 └──────────────────────────────┘
```

Why this shape:
- **FCL managers are not thread-safe** (`TG/body_collision.py:148-171`), and a world takes 2.6–10 s to build (MAP-05 §4.5). One warm worker process makes repeat checks take seconds, which is how R11's "immediately" can be met. The pattern is the Studio's own: a 1-worker `spawn` pool (`TG/server.py:700-725`).
- **Robot I/O runs out of process.** An SDK crash or a hung blocking call cannot freeze the UI. `alix_emulator.install()` replaces `sys.modules["Robotic_Arm"]` (`EMU/rm_emulator.py:2541-2616`), so real and emulated backends cannot share one process anyway.
- **All long work is a job.** HTTP/1.0 has no request timeout (MAP-04 §1.1), so uploads, runs, sweeps, offset solves and merges are jobs polled at about 1–2 s, with honest progress (stage i of N, elapsed, no fake %).

### 3.2 Backend modules (package `alix_studio_lite`)

| Module | Responsibility | Reuse from (mode) |
|---|---|---|
| `server`, `routes/*` | HTTP scaffold, gzip, 64 MiB cap, vendor whitelist, `{error, hint}` convention, route-needs table, page assembly, `/api/lite/state` | **Copy** `TG/server.py:6228-6469` (dispatch, handler, gzip, error payload), `:6297-6317` (mtime-stamped page cache), `TG/scripts/visualize_generated_config.py:530-568` (placeholder assembly). About 400 lines, provenance header. |
| `jobs` | Job registry: start/status/cancel, one live job per kind, poll interval, partial results | **Copy pattern** `TG/studio_generate.py:537-600, 861-1060` |
| `project` | Project folder store, unit registry (`units.yaml`), canonical file names, `SOURCE.json` provenance, recipe and undo history, atomic writes, slot ledger | **Schemas reused:** `SOURCE.json` (`ALIX/tools/taught_tracks.py:3628-3634`), unit record (MAP-13 §7.3), `description.txt` parse (`ALIX/tools/taught_pipeline.py:187`). Atomic-write idea from `TG/schedule_guards.py:619`. |
| `seed` (build-time tool) | Builds the registry from the PPT (slide renders, overlay boxes, labels), the V6 legacy mapping and ALIX pole/hand defaults | **Promote** the map-13 scripts (`scratchpad/scripts/pptx_detail.py`, `overlay_norm.py`, `arm_crops.py`) plus `parse_description` |
| `tracks` | NDJSON read/write (CRLF-safe, integers only), units, clocks (record 5.0 ms; gate 5.0 ms; replay 5.5545 ms band), duration law | **Import** `DP/track.py:66 parse_track`, `:85 build_project_file`, `:1566 expected_replay_s` |
| `optimise` | Canonical per-segment pipeline: trim → smooth → speed, with boundary ramps. "Recommended" = ALIX methods; "Classic" = the HTML methods (for parity with old files) | **Import (after promotion, §3.10)** `strip_pauses` (`ALIX/tools/taught_tracks.py:915`), `smooth_taught` (`:894`), `uniform_millideg` (`:748`), `capped_millideg` (`:767`). **Port** the HTML pure functions (`HTML:1417-1616`) to numpy with golden tests. |
| `gates` | Limits, G2 band-limited rate at 5.0 ms, G3 step ≤ 10°, G4 deviation against the raw file, endpoint-snap and boundary-jump detectors, spike list with plain-language suggestions | **Import (after promotion)** `deviation_mm` (`ALIX/tools/taught_tracks.py:607`), exp01 `rate_report/position_report/step_report` (`…/exp01…/src/predict_track_replay.py:92-131`). **Import** `DP/track.py:3792 canfd_preflight`, `WS/plan_analysis.py` (optional). **Port** CT `rate_fit_track`/tick-step check (MAP-06 §7.2). |
| `kin` | FK/IK services, tool frames, URDF limits (minus 5° margin), singular margin | **Import** `WS/urdf_kinematics.py` (`UrdfModel.fk_batch`, `link_world_transforms`), `WS/ik.py` (`IKSolver`, `locked` pole), `WS/srs.py`, `WS/srs_ik.py`, `WS/swivel_arcs.py`, `TG/singularity.py` |
| `offset` | "Offset & Verify": differential IK with taught-increment feed-forward, null-space pull to the taught pose, adaptive damping, raised-cosine taper, gates, repair or refuse | **New algorithm** (MAP-05 §8.2, prototype D measured), built entirely on `kin` + `gates` + `collide` |
| `sim` | `.txt`→timeline adapter (pole/hand/bin seeding), FK frames for playback (windowed, moving links only), robot geometry payload (baked overlay), commode + walls scene | **Import (after promotion)** shim `track_points/build_timeline/timeline_to_plan/assert_arm_sides/hand_rad` (`ALIX/tools/dual_arm_verdicts.py:531-692`); **import** `WS/dual_arm_plan_collision.py:46 load_plan_timeline`, `:487 compute_plan_frames`, `TG/scene_pose.py` (`resolve_component_pose`), `WS/srdf_states.py`, `ALIX/ros/hw/alix_hw/scripts/hand_units.py` (hand scale). **Copy** `TG/server.py:1864-1998` (geometry payload + baked-npz loader, about 120 lines). |
| `collide` | The warm collision worker: dual-arm sweep (arm-vs-arm, self, bin/body, commode at the right lid/seat state, walls), per-pose full-world filter, windowed re-check, closest-approach readout, plain-language pair classes | **Import** `WS/dual_arm_plan_collision.py:586 compute_dual_arm_plan_collisions` (with the §3.10 seam), `TG/body_collision.py` (`BodyCollisionWorld.fixture_manager`, `:401-464`), `WS/plan_scene_validation.py` (`load_wall_manager`, `PlanWallChecker`), `WS/redundancy.py:95 classify_pair`, `cross_arm_separation` (`WS/dual_arm_plan_collision.py:1089`) |
| `merge` | Program builder: join check, velocity-matched rate-limited bridges, bridge and transit collision checks, per-arm workflow JSON (with an explicit checkpoint step), dual timeline file, SAFE TIMELINE planner | **Import** `ALIX/tools/plan_min_time.py:142 concatenate`, `TG/approach_transit.py` (2° lerp transit check). **Port** v53 `_page2_program_chunks` + `_page2_build_program_rule_plan` with an `ast` parity test (MAP-08 §1.8). The V6 method is reference only (`V6/README.txt`). |
| `robot` | `ControllerBackend` protocol and implementations (§3.9); per-arm lock; process interlock with the v53 console; upload ledger (skip unchanged); event stream; error-code translation; live joint feed | **Vendor verbatim** `R14`, `UC/hand.py`, `UC/pole.py` (sha256 pins). **Import** `DP/connect.py:141 open_arm`, `DP/track.py` (`send_project`, `program_list`, `preflight_slots`, `run_program`, `stop_program`, `global_speed_guard`), `DP/runmode.py`, `DP/peripherals.py`, `DP/recording.py`, `DP/emulated_program.py`, `EMU/rm_emulator.py` |
| `launcher` | Preflight (Python 3.10–3.13, wheels, SDK path, bundle layout, port), env defaults (`ALIX_REPO_ROOT`, `ALIX_SRDF`, `BUTTERFLI_WORKSPACE_CONFIG`, `ALIX_WORKSPACE_CACHE`), browser open | **Copy pattern** `ALIX/tools/run_studio.py:92-205`, `TG/server.py:6493-6507` |

### 3.3 Front end

**Assembly (reused mechanism).**
- One page, `lite.html`, assembled at request time by placeholder substitution: `__VENDOR_JS__`, `__LITE_CSS__`, `__LITE_JS__`, `__MANIFEST__`. This is the mechanism of `render_html` (`TG/scripts/visualize_generated_config.py:558`).
- Plotly is lazy-loaded from a whitelisted `/vendor/` route with `ensurePlotly()` (`TG/web_editor/editor.js:5322`).
- No bundler and no CDN, as in the Studio. Vendor files are served from the installed `alix_taskgen` package via `importlib.resources`; they are referenced, not copied. The release bundle copies them in.

**Files.**

| File | Origin |
|---|---|
| `css/alix_tokens.css` | Generated from `ALIX/design/_tokens.css` (62 lines, the declared single source) + Lite tokens (`--joint-1…7`, `--plot-*`, `--mode-sim/dry/real`); the Studio's `test_design_tokens.py` copied for parity |
| `css/alix_base.css`, `css/alix_shell.css` | **Copied, filtered** `TG/web_editor/editor.css` ranges listed in MAP-01 §8.3 (fix the `.info-row` nesting and 6 undefined tokens while lifting) |
| `js/alix_shell.js` (about 1,000 lines) | **Copied**: `setStatus/showErrorToast/setLoading` (`editor.js:5431-5467`), `api` (`:9088-9104`), `bind/need/runSteps` (`:8784-8811`), tooltips/help (`:7835`, `:7996`), `showStudioModal`/`askSessionChoice` (`:13786`, `:14849`), docks (`:7580-7645`), dialogs (`:8390-8523`), history pattern (`:1839-1977`), global error capture (`:19150-19166`) |
| `js/alix_viewport.js` (about 500 lines) | **Copied**: `initThree` trimmed (`:309-570`), render loop (`:575-645`), `meshFromPayload` (`:667`), `loadRobotGeometry/setRobotSet/robotSetVisible/applyRobotPose` (`:12352-12437`), `fitCameraToScene` (`:1182`), fixture/walls (`:743-830, 1046, 1167, 9853`) |
| `js/alix_dac.js` (about 350 lines) | **Copied + extended**: `dacGoto/dacGotoTime/dacHighlight/dacJumpHit/dacPlay/dacLogEvents` (`:14272-14598`), `renderPlaybackBlock` (`:8749`). Add stop-at-first-collision, speed ×0.5/1/2/4, and a token-based plot theme (the copied layout is dark-only, `:14487-14491`). |
| `js/alix_board.js` | **Copied**: `loadBoard/renderBoardRows/initBoard` (`:14708-14757, 15208-15467`) + Board CSS (`editor.css:1507-1528, 1976-1998`) |
| `js/traj_core.js` | **Ported verbatim** from `HTML:1417-1620` (trim, smooth, scale, region mapping, writer) + `:934-1050` (analysis) + `:810-828` (parser); two fixes behind flags (§3.8) |
| `js/lite_plots.js` | **New**: `plotTheme()` from tokens, joint plots in seconds and degrees, linked zoom, cursor, collision bands and markers, `ResizeObserver` resize, maximise |
| `js/lite_flow.js` | **New**: top stepper (pattern from `HTML:220-342, 609-704`, restyled), teach countdown, auto-advance, empty states |
| `js/screens/*.js` | **New**: one per screen below |

Each copied function carries a header such as `// lifted-from TG/web_editor/editor.js#applyRobotPose @ <commit>`. A drift test extracts the same function from the installed `editor.js` and fails with "Studio changed X since lift: review and re-lift or bump the pin". The idea reuses the Studio's own `_function_body` helper in `test_studio_interaction_guards.py`.

**Screens** (top stepper: Regions → Teach → Optimise → Simulate → Test on robot → Upload; the Dashboard is home):

| Screen | Purpose | Requirements |
|---|---|---|
| App shell | Brand bar "ALIX Studio Lite" (inline SVG mark, `TG/web_editor/template.html:15-51`); a loud mode chip (Simulation / Robot dry run / Real robot + IP); a connection dot per arm; status line, toast, help; STOP ALL always visible when connected | NFR-THEME, NFR-UX, R4 |
| Dashboard (home) | 22 steps × LEFT/RIGHT matrix (Board pattern). Statuses NOT TAUGHT / RAW TAUGHT / OPTIMISED / SIM OK / REAL TESTED / IN PROGRAM, plus flags LEGACY V6 / NEEDS RE-TEACH / FAILED; counters per arm and phase; "Teach next missing" | R1 |
| Regions | PAGE 04/05 region maps with clickable labels; step list grouped by fixture phase; "What to clean" card (plain title, deck picture with overlay, lid/seat state check, pole/hand preset, cleaning time) | R1, R7 |
| Teach (guided session) | Per unit: show card → prepare robot (pole, then hand; foul-band and arm-still checks) → START DRAG + countdown → STOP & SAVE (canonical name + `SOURCE.json`) → quick check → Accept / Redo → auto-advance | R1, R7, R4 (connect), INT-CTRL |
| Optimise | One unit at a time; segment table (box-select on the plot creates a segment); per-segment Trim / Smooth / Speed with presets (Gentle / Standard / Fast) and "Advanced"; 5-deep labelled undo per unit; resizable joint plots; findings panel (spikes, limits, rates) with one-click fixes; gate chips; compare before/after | R2, R6, R11, R12, INT-TRAJ |
| Simulate | **3D dual-arm on the left, plots on the right**, draggable splitter (R10); world controls (lid/seat state, bin include/exclude/state, venue profile); playback transport; collision sweep; stop at the first collision with joint-plot band + red links; "Back to editing"; pad trail over the commode for missed-spot review; dual timeline (both arms, one axis) | R3, R5, R10, R11, R14 |
| Offset | Pick unit + segment; direction in plain words (towards toilet / sideways / up / press deeper) and mm; compute (job) → before/after overlay in 3D; flagged points on plots; "max achievable here" messages; accept → new optimised version | R9, R14 |
| Program (merge) | Build each arm's program in plan order from units; join and bridge checks; transit checks; dual timeline (TOGETHER / RUN FIRST / WAIT OTHER / lid checkpoint, as v53 plans it); export merged `.txt` + workflow JSON + timeline | R8, R10 |
| Robot (test & upload) | Connection cards (IP, port, Test connection, mode read-back); mode switch with confirmations; upload queue (skip unchanged, both arms in parallel, honest progress); test run with live 3D mirror; collision event → plot marker → Back to editing | R4, R5, R13, INT-CTRL |
| Settings & About (engineer) | SDK path, runner pin, slot range, world profile (venue, standoff, commode pose), clocks, versions, open v53 console instructions | INT-CTRL, deployment |

**Not ported (X1/X2 and expert-only):**
- Regions mesh painting, Workbench/Generate dock, Board "Generate missing"/"Raster sweep", Workspace tab, atlas/reach/manipulability overlays, Gizmo tab (`TG/web_editor/template.html:156-201, 204-526, 544-548, 556-597, 675-695, 729-735`).
- The Fusion-style ribbon and command palette; they are too dense for non-technical users (MAP-01 §8.4).

### 3.4 API surface (`/api/lite/*`)

All responses are JSON. An expected refusal returns HTTP 200 with `{error, hint}`, as in the Studio (`TG/server.py:6162`). Every route has a needs row.

| Group | Routes |
|---|---|
| State | `GET state` (connections, locks, live jobs; pattern `TG/server.py:6255`), `GET project`, `GET jobs/<id>`, `POST jobs/<id>/cancel` |
| Registry | `GET units` (dashboard rows), `GET unit?id=`, `POST unit/preset` (pole, hand, cleaning time), `GET deck/<asset>` (whitelisted renders) |
| Tracks / optimise | `GET track?id=&ver=&decimate=`, `POST optimise/preview`, `POST optimise/accept`, `POST recipe`, `POST history/undo`, `POST history/redo`, `POST gates` |
| Simulation | `GET robot-geometry?set=visual\|collision`, `GET scene?state=closed\|lid_open\|lid_seat_open&venue=`, `POST sim/frames` (window), `POST sim/sweep` (job), `POST sim/recheck` (windowed job) |
| Offset / merge | `POST offset/compute` (job), `POST offset/accept`, `POST merge/build` (job), `GET merge/<program>` |
| Robot | `POST robot/connect {side, ip, port}`, `GET robot/status`, `POST robot/mode {side, mode, confirm}`, `POST robot/pole`, `POST robot/hand`, `POST robot/teach/start\|stop\|save`, `POST robot/upload` (job), `POST robot/run` (job), `POST robot/stop {side\|all}` (no confirmation), `GET robot/live` |

Robot-motion routes require a one-time confirmation token issued by the confirm modal. This stops a replayed request from moving the arm. The server binds to loopback only; the Studio warns why (`TG/server.py:6539-6544`).

### 3.5 Data model and project folder

```
~/AlixLite/projects/<project>/                 (outside every repo; one per site/fixture)
  project.yaml        fixture (commode_c), world profile {venue, ugv_standoff_m, commode pose}, bin state,
                      clocks {record 0.005, gate 0.005, replay 0.0055545}, slot range per arm, default IPs,
                      pins {alix_commit, runner_sha256, sdk_version}
  registry/units.yaml 42 teachable units (22 steps × 2 arms; MAP-13 §3), one record each (below)
  registry/deck/      slide renders hi-NN.png, crops, overlay boxes (from the PPT, built by `seed`)
  units/<UNIT_ID>/
    raw/<ID>__RAW__<yyyymmdd-hhmmss>.txt        drag recording bytes, never modified
    raw/<ID>__RAW__<ts>.SOURCE.json             provenance (taught_tracks schema + unit/task ids, sha256)
    recipe.json        source sha, segments [{t0_s, t1_s, trim{method,tol}, smooth{method,hz|window},
                       speed{pct|lambda, ramp}, offset{dir, mm, frame}?}], preset, notes
    history.json       undo ring (≤ 5 labelled recipe snapshots) + redo stack
    opt/<ID>__OPT_v<k>.txt  + .report.json      derived output + gates, hashes, recipe copy
    checks/sim_<k>.json  checks/real_<n>.json   verdicts: world params, collisions {index, t, pairs},
                                                stop row, operator notes
  programs/<name>/   LEFT_<name>.txt, RIGHT_<name>.txt, workflow_LEFT.json, workflow_RIGHT.json,
                     timeline.json (the dual timeline V6 never shipped), merge_report.json
  controller/slots.json   per arm: slot → {unit|program, proj_sha256, bytes, stored_name, uploaded_at, verified}
  logs/runs/<ts>_<side>.jsonl   structured run events (no clearing between stages)
```

**Unit record** (abridged; full schema MAP-13 §7.3):

```yaml
- id: L-R05-P1
  deck: {code: null, label: "SEAT TOP PART 1", task: "5A", slide: 11, code_source: inferred}
  arm: left
  type: surface            # surface | mechanism | tool | glove | idle
  part: {n: 1, of: 2, where: "left half"}
  step: 7                  # deck order
  paired_with: R-R04
  sync: TOGETHER           # TOGETHER | RUN_FIRST | WAIT_OTHER
  fixture_state: LID_OPEN  # CLOSED | LID_OPEN | SEAT_OPEN | ANY
  cleaning_time_s: 15
  pole_mm: 150
  hand: {preset: GRIP, counts: [0, 0, 0, 0, 150, 1000]}
  image: {media: image2.png, box_uv: [0.334, 0.420, 0.516, 0.954], mirror: false}
  legacy: {package: V6, file: LEFT/L_P06P07.txt, program_id: 76, samples: [3865, 6744], confidence: medium-high}
```

Conventions:
- IDs follow MAP-13 §5.2 (`<ARM>-<KIND><NN>[-P<n>|-S<n>]`), keeping deck aliases (FL-R09 → L-R09).
- The word "region" is overloaded today: a PPT area, an HTML time window and a Studio mesh area (MAP-03 §1.4). Lite uses **Area/Part** for cleaning regions and **Segment** for time windows.
- Operators never see P-numbers or program IDs.

### 3.6 Reuse strategy per source

| Source | Mode | What, and why that mode |
|---|---|---|
| `alix_workspace` | **Import** (editable in dev, wheel in bundle) | FK/IK/SRS/collision/sweep/frames/SRDF. ROS-free with a small dependency set (MAP-04 §7.2). Two default-preserving seams (§3.10). |
| `alix_taskgen` light modules | **Import** | `body_collision`, `scene_pose`, `fixture_parts`, `transforms`, `constants`, `paths`, `approach_transit`, `singularity`. Safe because `alix_taskgen/__init__.py` contains only `__version__`, and these modules import only numpy/scipy/trimesh/yaml and each other. **Never import `server`** or the generator modules. |
| Studio server scaffold | **Copy** (about 400 lines, provenance) | Cannot be imported: `server.py` pulls the generator stack at import (`TG/server.py:36-44`). |
| Studio front end | **Copy** (about 1,900 lines JS/CSS) with drift test; tokens and vendor **by reference** | `editor.js` is one 19k-line single-scope script with no module boundaries; extracting shared modules upstream would collide with about 140 structural guards on that file (MAP-04 §5). |
| `alix_dispatch` | **Import** (sys.path today; pyproject seam) | `open_arm`, upload/list/run/verified stop, run mode, pole/hand, recording, emulated program. Stdlib-only at import (MAP-06 §7.1). |
| `alix_emulator` | **Import** (dev/CI backend only) | Verbatim-pinned upstream (`ALIX/core/python/src/alix_core/tests/test_emulator_verbatim.py`); extensions only at seams. |
| `ALIX/tools/dual_arm_verdicts.py` shim | **Import after promotion** (seam) | `.txt`→timeline with mandatory pole/hand seeding (`:531-692`). Fallback: copy about 200 lines with a parity test. |
| `ALIX/tools/taught_tracks.py` math | **Import after promotion** (seam) | `strip_pauses`, `smooth_taught`, `uniform/capped_millideg`, `deviation_mm`. Importing the tool today pulls `cleaning_tracks` and exp01 at module load (`ALIX/tools/taught_tracks.py:78-92`), hence promotion. |
| `ALIX/tools/plan_min_time.py` | **Import** | `concatenate` (`:142`); module imports only numpy at top. |
| Runner v14 + `hand.py` + `pole.py` | **Vendor verbatim**, sha256-pinned, run as a subprocess | Field-proven safety behaviour (CRLF, `only_save=1`, size verify, speed restore, slow stop); no rewrite risk in the MVP. |
| UI v53 | **Port** the two pure planner functions (`ast` parity test); **re-express** the operator flows in the web shell; **keep** v53 unchanged as an engineer console | Tk app, dark theme (`V53:56-71`); not a reusable UI. |
| Trajectory HTML | **Port** the pure JS verbatim (preview) + a numpy port (canonical), with golden tests; reuse the stepper/table/plot patterns restyled | The single integration source for R2 (MAP-07 §10). |
| Design tokens | **Reference** `ALIX/design/_tokens.css`; parity test | Declared single source ("the single source every card inlines verbatim", `ALIX/design/_tokens.css:1`). |
| URDF, meshes, SRDF, commode, walls | **Reference in repo; sparse copy in the bundle** | `WS/urdf/alix.urdf` verified current (MAP-11 §2.1); re-bake the overlay (seam). |

### 3.7 Control-app integration (INT-CTRL)

1. **Layer 1, reuse as-is (MVP).**
   - `R14`, `hand.py` and `pole.py` are vendored byte for byte under `alix_studio_lite/vendor_runner/`, with a sha256 pin test copied from `test_emulator_verbatim.py`.
   - `RunnerSubprocessBackend` invokes them exactly as v53 does: `[python, -u, runner, --sdk DIR, --side S, --ip IP, --port P, --yes, …]`. The `--ip` comes from the connection card, and `a.ip` wins over `--side` (`R14:2347`).
   - The backend parses the documented stdout tokens (`VERIFIED:`, `RUN COMPLETE:`, `DRAG SAVE COMPLETE:`, `ARM RUN MODE:`, `!! … reports`, …; MAP-09 §1.4) into structured events.
   - Stop is SIGINT to the process group. On Windows a 20-line launcher (`lite_runner_launch.py`) installs a SIGBREAK→`KeyboardInterrupt` handler and then `runpy.run_path(runner, run_name="__main__")`, so the runner bytes stay untouched.
   - A per-arm lock enforces one command source per arm.
2. **Layer 2, port.**
   - The operator-facing v53 flows are re-expressed in Lite's screens: drag teach and save, pole/hand presets, mode CHECK/SIM/REAL with read-back, upload/verify/run, run both, the Page-02 dual workflow, and the lid checkpoint.
   - The SAFE TIMELINE planner (`V53:9619-9997`) is ported with a parity test on the V6 FINAL JSONs and the synthetic cases A–E (MAP-08 §1.8).
   - The lid checkpoint becomes an explicit `checkpoint` step type with a custom message, written as workflow v2 while still emitting v1 for the runner. It is no longer inferred from a "positive WAIT".
   - Multi Run is not ported; it is broken and superseded.
3. **Layer 3, keep.**
   - The v53 Tk console stays available unchanged as the engineer tool for low-frequency features: global points, multi-upload, hand-editing JSON.
   - Lite refuses robot commands while a v53 console or a runner it did not start is running (a `/proc` scan on Linux; process list on Windows). This preserves "one command source per arm" (MAP-06 §7.3 item 9).
4. **Layer 4, converge (Phase 6 onward).**
   - `DispatchBackend` runs in a robot-agent process per arm over `alix_dispatch`.
   - It gives persistent sessions, batch upload in one session, skip-unchanged by readback, `stop_program`'s verified stop ladder, `global_speed_guard`, and UDP capture (`DP/recording.py`). Drag teach is a direct port of `R14:912-989`, with save retry.
   - It is needed for exact real-arm collision mapping (R5) and for the fastest upload (R13).

### 3.8 Trajectory-HTML integration (INT-TRAJ)

- **Two runtimes, one algorithm.**
  - `traj_core.js` is a verbatim port of `HTML:1417-1620` plus the analysis at `:934-1050`, used for instant preview while a slider moves.
  - `optimise` in Python is canonical; it writes the files the robot will run.
  - A golden test fixes both at ≤ 1 mdeg on 10 reference files. MAP-10 reproduced `UC/zip/02…_30pct.txt` from `zip/01` to ≤ 1 mdeg with the HTML method.
- **Presets.**
  - **"Recommended"** uses the ALIX hardware-validated methods: anchored pause removal (`strip_pauses`, 0.05°, ≥ 3 lines), 5 Hz zero-phase Butterworth on the 5 ms record clock (`smooth_taught`, pad deviation ≤ 1.43 mm), and rating-capped re-time with ramps (`capped_millideg`, `--section` semantics).
  - **"Classic"** keeps the HTML methods for continuity with existing files. The fixes sit behind flags so "Classic (legacy)" still reproduces old outputs exactly:
    - R-A1, endpoint snap: keep W/2 dwell or a raised-cosine edge (MAP-07 §9).
    - R-A5, boundary jumps: ramp the speed factor by ≤ 0.05 per line (MAP-06 §2.6).
- **Model change.** The HTML's global trim/smooth parameters become per-segment recipes. The HTML's "regions" (row windows) become Lite "segments", defined in seconds rather than row numbers. The undo is a ring of 5 recipe snapshots, not of data arrays.
- **UI reuse.** The 6-step stepper becomes the Optimise sub-flow (Load → Setup → Analyse → Review → Generate → Compare). The suggestion and segment tables, box-select-to-segment, playback cursor and comparison metrics are kept, restyled to ALIX tokens.
- **Changes made while porting.**
  - Plotly comes from the vendored 2.35.2 copy, the same version, so the plot code needs no API changes.
  - Joint colours: the category10 set gives J3 green and J4 red, which clashes with ISA-101 state colours. Replace them with a palette that avoids state red/green, or document them as identity colours (MAP-01 §5).
  - Fix the 8 UI-state bugs listed in MAP-07 Appendix B rather than porting them.
- **Dropped.** `showSaveFilePicker` downloads; results are saved in the unit folder through the server.

### 3.9 The sim/real seam

**What "Sim" means.** R4's "toggle between real and sim" has three distinct meanings in the evidence (MAP-09 §7, MAP-11 §4.4). Lite makes the user switch 3-way and states plainly what each position does:

| Mode (chip text) | What runs | Hardware | What it proves |
|---|---|---|---|
| **Simulation (no robot)** | 3D FK playback + FCL sweep in Lite (`sim` + `collide`) | none, any OS | collisions with arm/self/bin/commode/walls; joint limits, rates, steps; pad trail |
| **Robot dry run** | Controller run mode SIMULATION (`--set-arm-mode simulation`, verified by read-back, `R14:1229-1262`) | connected; arms do not move | controller accepts the program; controller timing; controller's own collision model. Lift and hand are not simulated (MAP-09 §7). |
| **Real robot** | Controller run mode REAL after a confirmation modal showing destination + gates (`showStudioModal`, `editor.js:13786`) | connected; arms move | the actual run |

**`ControllerBackend` interface.** Every robot screen talks to this protocol; no screen knows which backend is behind it:

```
connect(side, ip, port) · status() · get_mode() · set_mode(sim|real)
list_programs() · upload(track, slot, name) -> verified · run(slot, transit_pct) -> event stream · stop(side|all)
pole(mm) · hand(counts6) · teach_start() · teach_stop() · teach_save(path) · joints()
```

| Implementation | Over | When |
|---|---|---|
| `RunnerSubprocessBackend` | runner v14 | MVP; real controllers, in either controller mode |
| `DispatchBackend` | `alix_dispatch` in a robot-agent process | Target, from Phase 6 |
| `EmulatedBackend` | `alix_emulator.install()` + `emulated_program.attach` via `open_arm` (`DP/connect.py:141-182`), in a robot-agent process | Developer/CI and demos without hardware. The program verbs and lift/hand are emulated; drag teach is **not** (MAP-04 §4.4), so Lite adds seam fakes that replay a stored recording. Linux/Windows only (`ALIX/docs/EMULATOR.md:14-44`). Hidden from operators. |

**Live mirror.**
- The joint feed (backend `joints()`, or the runner's `arm_state` equivalent) is converted to URDF joints (deg → rad; lift mm → m; hand counts → rad with `hand_units.py`, not the `sim_bridge` scale, MAP-03 §3.9).
- Lite's own FK service turns it into link poses, which `applyRobotPose` draws. The pattern is the Studio's `/api/sim/joints` + `/api/sim/pose` (`TG/server.py:5326-5393`), and **both arms are posted in one body**, because a per-arm post replaces the stored map (MAP-02 §6).

### 3.10 Upstream seams in ALIX (small, default-preserving)

| # | Change | Why | Effort |
|---|---|---|---|
| S1 | `WS/dual_arm_plan_collision.py`: kwargs `body_links=None` (default = today's `kBodyCheckLinks`, `:644`), `bin_state="BIN_LOWERED"` (today hard-coded, `:434`), `pole_m=None`; plus an in-memory entry point taking timelines instead of file paths (the loop already works on (times, rows), `:651-653`) | Bin toggle (R10); commode/walls via the existing `extra_managers`; no temp files for R11 | 1 pd |
| S2 | Promote `hand_rad/track_points/build_timeline/timeline_to_plan/assert_arm_sides` (`ALIX/tools/dual_arm_verdicts.py:531-692`) into `WS/taught_plan.py`; the tool re-exports | One `.txt`→timeline source with mandatory seeding | 0.5 pd |
| S3 | Promote `strip_pauses`, `smooth_taught`, `uniform_millideg`, `capped_millideg`, `deviation_mm`/`_point_to_polyline_mm` and exp01 `rate/position/step_report` into `DP/taught_math.py`, FK via `WS.urdf_kinematics`; the tools re-export | ALIX's own rule: "Imported, never copied — one source" (`ALIX/tools/taught_tracks.py:83-84`) | 1.5 pd |
| S4 | Re-bake `WS/workspace_data/visual_overlay_baked.npz` with `ALIX/tools/meshes/bake_visual_overlay.py` | The bake shows the old tall bin and lacks `camera_link` (MAP-11 §2.4) | 0.25 pd |
| S5 | Add `pyproject.toml` to `alix_dispatch` | Today it is reached by `sys.path` only (`ALIX/LAYERS.md`, "MEASURED 2026-09-06" row) | 0.25 pd |
| S6 (optional) | Move the robot-geometry payload builder (`TG/server.py:1864-1998`) into `WS/overlay.py`; the Studio handler wraps it | Removes Lite's largest Python copy | 0.5 pd |

Each seam ships with tests. The Studio's DAC payload for a cached pair must be byte-identical before and after S1.

### 3.11 Deployment and packaging

- **Development:**
  - Install as a console script `alix-studio-lite` from `ALIX/core/python/alix_studio_lite`, run from the checkout in a pinned venv.
  - The system Python here has a numpy/scipy ABI clash (MAP-05 §7; MAP-10 header). `python-fcl` ships wheels only for 3.10–3.13 (MAP-04 §3.1).
  - Port 8766 means Lite can run beside the Studio (8765).
- **Operator laptop (no ROS, no internet, no repo checkout).** A **Lite bundle** is built by `tools/build_lite_bundle.py` from a pinned ALIX commit. It contains:
  1. A wheelhouse: numpy, scipy, pyyaml, trimesh, python-fcl, pycollada, networkx, lxml, plus `alix-workspace`, `alix-core` (with the emulator), `alix-dispatch`, `alix-taskgen` (light modules only are used) and `alix-studio-lite`.
  2. A **layout-preserving sparse repo snapshot**, so every existing repo-relative resolver works unchanged via `ALIX_REPO_ROOT`. `BodyCollisionWorld(repo_root=…)` takes the root explicitly and `plan_scene_validation` reads `ALIX_REPO_ROOT` at import (`TG/body_collision.py:114-119, 138-140`; `WS/plan_scene_validation.py:31-59`). The snapshot holds:
     - `ARCHITECTURE.md` and `core/` markers
     - `package.xml` + collision meshes of the 5 description packages (6.4 MB)
     - the re-baked overlay (about 3 MB)
     - the `initial_positions.yaml` files
     - `alix.srdf`
     - `alix_tasks/config/scene/*` (44 KB)
     - `commode_c/{scene.yaml, models/body,lid,seat.stl}` (about 2.8 MB)
     - `contact_links.yaml`
     - the D435 `package.xml` + `d435.dae`, or the camera omitted
     
     About 15–20 MB in total, against 96 MB for `ros/description` alone.
  3. The RealMan SDK for the target OS. Redistribution is an open question: the SDK has no LICENSE file (MAP-04 §3.1).
  4. Vendored Inter and JetBrains Mono woff2 (SIL OFL). The Studio does not load its fonts (MAP-01 §1.7).
- **Launcher:** preflight, env defaults (`ALIX_REPO_ROOT`, `ALIX_SRDF`, `BUTTERFLI_WORKSPACE_CONFIG`, `ALIX_WORKSPACE_CACHE`), browser open. Target cold start under 10 s: no generator import, lazy model build, warm worker started in the background.
- **OS:**
  - Linux x86_64 is recommended: the runner stop semantics, the emulator and the SDK are all proven there.
  - Windows works for Simulation, and for robot I/O through the stop shim (to be verified).
  - macOS supports Simulation only; there is no SDK library (`ALIX/docs/EMULATOR.md:14-44`).

---

## 4. Per-requirement verdicts — can it be borrowed from ALIX Studio, and what is the approach?

| Req | Borrow from ALIX Studio? | Reused (source → mode) | New work | Phase |
|---|---|---|---|---|
| **R1** Region-wise collection, PPT Regions tab, dashboard | **Partly.** The Board is a direct UI template (`editor.js:14708-14757, 15208-15467`) for "regions with/without raw txt". The Studio Regions tab is mesh painting for the generator (X1) and cannot be ported. Studio has no wizard or timer (`TG/web_editor/template.html:116-118`). | Board JS/CSS → copy. PPT renders, overlay boxes and ID scheme from MAP-13 → seed tool. Drag teach → runner `--drag-session/--drag-save` (`R14:912-989`). `SOURCE.json` schema → `taught_tracks`. | Unit registry; Regions screen; teach wizard with countdown and auto-advance; canonical file names; sidecar provenance; fixture-state check | P1, P2, P5 |
| **R2** Per region/sub-region trim, smooth, scale + 5-deep undo | **No** for the algorithms. Only the undo pattern (`editor.js:1839-1977`, global cap 80) is reusable, rescoped. | HTML pure functions → port; ALIX `strip_pauses/smooth_taught/capped_millideg` → import (S3); undo pattern → copy and adapt (per unit, depth 5, labelled, one entry per slider gesture). | Segment model with per-segment recipes; presets; boundary ramps; recipe-snapshot undo persisted in `history.json` | P3 |
| **R3** Simulation playback, return to editing | **Yes, mostly.** Viewport, robot overlay, FK frames and DAC transport exist and are joint-space (MAP-02 §7). | `applyRobotPose`, `loadRobotGeometry`, `meshFromPayload` → copy; `compute_plan_frames` → import; fixture/walls rendering → copy; `.txt` adapter → import (S2). | Pole/hand/bin seeding in the adapter; windowed frames; "Back to editing" with the problem span pre-selected; pad trail for missed-spot review. The Studio's coverage analysis is X1/Cartesian-config based and does not apply. | P4 |
| **R4** Real-arm test, IP entry, real/sim toggle | **No.** The Studio has no execution path (`editor.js:13045-13047`). | Runner v14 → vendor; `DP/connect.open_arm(side, ip)` → import; run mode with read-back (`R14:1213-1262`, `DP/runmode.py`); confirm modal → copy; sim-dot/mode chip → copy. | Connection cards with IP/port entry; 3-way mode switch; per-arm locks; process interlock; live mirror | P5, P6 |
| **R5** Collision → graph stops at the joint value and highlights it | **Partly.** The DAC sweep, 0/1 timeline, red links, Prev/Next and log exist. **No joint plots and no stop-at-collision anywhere** (MAP-02 §3.3). The DAC world omits the commode, walls and fingers. | `compute_dual_arm_plan_collisions` + `extra_managers` (commode via `BodyCollisionWorld.fixture_manager`, walls via `load_wall_manager`) → import + S1; keep `joint_timeline` (the Studio drops it, `TG/server.py:4357`); `dacGoto/dacHighlight/dacJumpHit` → copy. | Stop at the first colliding sample; cursor + translucent band + 7 joint markers + pair annotation on the joint plots; real arm: 4109/4114 event → index via the player clock, later exact via UDP capture (`DP/track.py:2056-2069`) | P4 (sim), P6 (real) |
| **R6** Resizable/adjustable plots and windows | **Partly (parts only).** Studio plots are fixed-size (MAP-01 §2.6); its dock separator, dialog grip and maximise-restore are reusable. | `initDockResize/setDockWidth` (`editor.js:7580-7645`), `makeDialogResizable` (`:8470`), `preMaximize` → copy; HTML zoom/pan/box-select → port. | Plot-pane splitters (vertical and horizontal), maximise per plot, linked zoom, `ResizeObserver`, persisted layout | P3 |
| **R7** Per-region pole height + hand before teaching | **No** (the inputs' note is inaccurate). The Studio never commands hardware; it offers only vocabulary (SRDF rungs), visualisation and the stowed-bin foul-band warning (MAP-03 §3.7). | Apply path: runner `--manual-pole`, `--manual-hand-raw`, presets GRIP `[0,0,0,0,150,1000]` / ALL OPEN (`V53:2980-2998`) → vendor; ALIX order "pole first, then hand, arm still" (`UC/usage.py:24-25`, `prestroke`); foul band 0.153–0.282 m (copy the constant); defaults from V6 / commode-C `description.txt`. | Per-unit preset record; apply-before-teach step with read-back; arm-still check; 3D preview of pole + finger joints | P5 |
| **R8** Merge `.txt` files into one executable trajectory, collision-aware | **No** (Studio). No taught-file merger exists anywhere; V6 is output only (`V6/README.txt`). | `concatenate` (≤ 0.5° boundary) → import; dual-arm sweep + commode `extra_managers` → import; `approach_transit` lerp → import; v53 planner → port; V6 bridge method → reference. | Merger: split on pole/hand changes, velocity-matched rate-limited quintic bridges, check every bridge **and** every controller transit (V6 has unchecked transits up to 64.6°, MAP-10 §3.2) against commode-in-state + partner + bin; dual timeline file; slot allocation | P7 |
| **R9** Offset via FK → offset → IK, no adjacent jumps, joint limits | **No** for the feature. **Yes** for the library: `UrdfModel`, `IKSolver`, `SrsArm`, `srs_ik`, `swivel_arcs`. Existing IK engines produce >10° jumps on taught tracks with a 10 mm offset (MAP-05 §2.5). | `WS` kinematics → import; limits (URDF ±178/130/178/135/178/128/360) − 5° margin; G3/G4 gates → import (S3). | "Offset & Verify" solver: taught-increment feed-forward + damped least squares + null-space pull, taper, repair or refuse with "max achievable Y mm here" | P8 |
| **R10** Dual-arm sim left, plots right; correct URDF/meshes; bin toggle | **Partly.** URDF, meshes, viewport and DAC are reusable. The split layout and bin toggle do not exist; the bin is hard-coded LOWERED (`WS/dual_arm_plan_collision.py:434`). | `WS/urdf/alix.urdf` (verified current, MAP-11 §2.1) + re-baked overlay (S4); dock splitter → copy; Schedule timeline (`editor.js:13601-13676`) → copy for the dual timeline. | Split layout; bin include/exclude/state (visibility + `body_links` S1 + home joint map) with foul-band warning; dual timeline at the player clock | P4, P7 |
| **R11** Adjust smoothness, validate immediately in sim | **Partly.** Engines yes; "immediate" no: the Studio rebuilds models per call (`WS/dual_arm_plan_collision.py:549-552`). Stale-on-edit gate chips → copy (`editor.js:5371-5430`). | Warm collision worker; content-hash cache idea (`TG/server.py:4303-4316`). | Windowed re-check of only the edited span; live gate chips (rate, step, deviation) under 1 s; sweep in seconds | P3, P4 |
| **R12** Spike analysis with suggestions | **No** (Studio). **Yes** from the ALIX taught lane. | G2 (20 Hz band-limited rate at 5.0 ms vs 180/180/225×5 °/s), G3 step ≤ 10°, `canfd_preflight` (`DP/track.py:3792`), tick-step and accel bands → import/port; suggestion mapping from MAP-06 §4 | Per-joint markers on plots; plain-language findings ("J6 too fast at 12.4 s → slow this stretch to 85 %"), each with a one-click, undoable fix; detectors for the HTML's endpoint snap and boundary jumps | P3 |
| **R13** Faster upload | **No** (Studio). Throughput is fixed by the controller (≈45 kB/s, 2 KB acked chunks, ≈1.25 ms/point; MAP-06 §1.4). | Upload path → runner (MVP) / `DP/track.send_project` (target); readback verify; `preflight_slots` idea → import. | Ledger with skip-unchanged (sha + size + stored name), both arms in parallel, one session per arm per batch (DispatchBackend), background queue, fewer points through the optimiser (e.g. 632 KB ≈ 14 s → about 6 s at 40 % points) | P6 |
| **R14** Calculate, apply and test offsets in sim before real | **Partly.** The sim pipeline is reused (R3/R5); the solver is new (R9). | As R3/R5 | Before/after overlay, flagged indices on plots, "sim passed" gate required before "Test on robot" | P8 |
| **X1/X2** | Excluded | — | Not ported: painting, Generate/Workbench, Board generate, Workspace/atlas/reach/manipulability, Gizmo (§3.3). The IK engine is used internally for R9 only. | — |
| **NFR-UX** | **Rules yes, layout no.** The Studio's design rules target "operators, not CAD professionals": ≥24 px targets (44 px primary), colour + glyph, persistent dialogs (`ALIX/design/README.md:9-37`). | Feedback toolkit (status, toast, confirm modal with destination + gates, gate chips, honest progress, empty states, tooltips, help) → copy | Guided stepper; plain words instead of technical names (reverses Studio decision RED:268-270); presets on the surface, parameters under "Advanced"; translated error codes; confirm only motion and controller writes | all |
| **NFR-THEME** | **Yes.** Tokens are a contract-tested single source. | `ALIX/design/_tokens.css` → reference; parity test → copy; brand mark → copy | Lite tokens for joints, plots and modes; a `plotTheme()` read from tokens (the Studio's plots are dark-only and unreadable in light) | P1 |
| **INT-TRAJ** | — | §3.8 | §3.8 | P3 |
| **INT-CTRL** | — | §3.7 | §3.7 | P5, P6, P7 |

---

## 5. Phased delivery plan

**Order of work:**
- Phases 0–4 need no robot. They de-risk the largest reuse bets (sweep world, viewport, optimiser) on existing data: V6 package, `UC/zip`, ALIX commode-C recordings.
- Hardware work starts in Phase 5.
- Phases 0–6 (about 70 pd) give an operator-usable Lite for R1–R7 and R10–R13.

| Phase | Goal | Deliverables | Reqs | pd | Exit criteria |
|---|---|---|---|---|---|
| **P0 Spike & upstream seams** | De-risk the collision world and land the ALIX seams | (1) Studio-as-harness: shim-converted V6 L/R tracks in `.cache/lite_spike_plans`, inspected in the unchanged DAC tab. (2) World-calibration script: sweep V6 files with commode (closed/lid_open/lid_seat_open) + walls per venue + correct pole/hand seeding; classify pairs. (3) Seams S1–S5 (+S6 optional) with tests. | R3, R5, R8, R10 (de-risk) | 7 | Calibration note naming the venue, standoff and commode pose that remove false wall hits (MAP-05 §4.6 found them at 800 mm); ALIX test suites for `alix_workspace`, `alix_dispatch` and `alix_taskgen` green; Studio DAC payload byte-identical for a cached pair; re-baked overlay shows BinShort and the camera |
| **P1 Skeleton, shell, data model** | A running, themed, empty Lite with the project and registry model | Package + console script; copied server scaffold, job registry, page assembly; tokens/vendor by reference; copied shell CSS/JS; fonts; `plotTheme()`; project store; `units.yaml` seeded from MAP-13 (42 units, aliases, deck boxes) + V6 legacy mapping + pole/hand defaults; guard tests (token parity, quickjs execution of every Lite JS file, route needs, no remote assets, verbatim pins, JS drift test) | NFR-THEME, NFR-UX, R1 (registry) | 8 | Cold start to usable page < 5 s on the dev box; light and dark parity tests pass; registry lists 22 steps / 42 units with deck aliases; no X1/X2 control present |
| **P2 Regions & Dashboard** | The operator sees what must be cleaned and what is done | Dashboard (Board port) with statuses and flags computed from files; Regions screen (region maps with hotspots, steps by phase, unit cards with overlays, fixture-state question, presets view); legacy V6 import as LEGACY | R1 | 6 | Dropping a raw file into a unit flips its cell to RAW TAUGHT on refresh; every unit card shows a plain-language title, picture, state and presets; counters correct per arm and phase |
| **P3 Optimise** | Per-segment trim/smooth/speed with undo, spikes and gates | `traj_core.js` port + numpy canonical pipeline (Classic + Recommended); segments in seconds with box-select; per-segment recipes; 5-deep labelled undo/redo persisted; resizable, linked, maximisable plots; gates G2/G3/limits/G4 + endpoint-snap and boundary-jump detectors; findings with one-click fixes; compare view; accept → `OPT_vk` + report | R2, R6, R11 (gates), R12, INT-TRAJ | 14 | JS preview vs Python ≤ 1 mdeg on 10 reference files; Classic reproduces `UC/zip/02…_30pct.txt` from `zip/01` within 1 mdeg; undo depth 5 per unit survives reload; known issues flagged (R_P06P07 over-rating ticks, R10_HNGR J4 margin 1.53°, MAP-10 §1.3); Recommended preset on commode-C raw files gives 0 G2 ticks, G3 ≤ 10°, G4 ≤ 10 mm against raw |
| **P4 Simulate & collisions** | Dual-arm 3D with the commode, stop-at-collision on the joint plots | Copied viewport/overlay/fixture/walls; adapter (S2) with pole/hand/bin seeding; warm collision worker (S1 world: arms, self, bin per toggle, commode in state, walls); windowed frames; split layout (3D left, plots right); transport with speed; stop at first collision + band/markers/red links + log; "Back to editing"; bin include/exclude/state + foul-band warning; pad trail; dual timeline | R3, R5 (sim), R10, R11, R14 (pipeline) | 13 | A seeded collision case stops the transport at the first colliding sample and the plot cursor matches the sample within one row at the stated clock; bin toggle changes both view and verdict; re-check of a 5 s edited span returns in ≤ 5 s with a warm worker; whole 20 s pair sweep ≤ 20 s at dt 0.02 |
| **P5 Robot connect, pole/hand, Teach** | Operator teaches region by region on the robot | Vendored runner + `RunnerSubprocessBackend` (tokens → events, IP pass-through, SIGINT stop, Windows shim, per-arm lock, sha pin, process interlock with v53); connection cards (IP/port, test, mode read-back); presets apply (pole, then hand; foul band; arm still); teach wizard (card → prepare → drag + countdown → save with `SOURCE.json` → quick check → accept/redo → auto-advance; optional both-arms); `EmulatedBackend` for CI | R1, R4 (connect/mode), R7, INT-CTRL | 12 | On hardware: three consecutive units taught with countdown and auto-advance; pole and hand applied in order with read-back; files and sidecars in the right folders; dashboard live. In CI: emulated list/upload/pole/hand paths pass; the runner sha pin test passes. |
| **P6 Test on robot & upload** | Validated trajectories run on the arms; uploads are faster | Upload ledger (skip unchanged by sha + size + stored name), both arms in parallel, slot-range enforcement, honest progress; test run (dry run / real) with confirm modal; STOP ALL; live mirror (second connection if the controller allows it, else post-run replay); 4109/4114 → plot marker → Back to editing; `DispatchBackend` spike (persistent session batch upload) | R4, R5 (real), R13, INT-CTRL | 10 | Second upload of unchanged files sends 0 bytes and finishes in < 2 s; parallel two-arm upload measured against the v53 batch baseline; a dry-run is mirrored in 3D (or the fallback documented); an injected or real 4109 lands on the plot within ±0.5 s of the event; stop verified on the target OS; global speed restored after every run |
| **P7 Program (merge) & dual timing** | Collision-checked merged programs and dual timeline | Merger (split on pole/hand, `concatenate`, velocity-matched rate-limited bridges, bridge and transit sweeps against commode in state + partner + bin); ported SAFE TIMELINE planner + explicit checkpoint step; workflow JSON (v1 for the runner, v2 for Lite) + `timeline.json`; Program screen; pole timing from `pole.py` law | R8, R10 (timing), INT-CTRL | 10 | A V6-equivalent L/R program pair rebuilt from per-unit files; every join ≤ 0.5°; every bridge and transit swept; planner parity with v53 on the V6 FINAL JSONs and cases A–E; workflow runs unchanged under runner v14 in dry-run mode |
| **P8 Offset (FK → offset → IK)** | Safe positional offsets, tested in sim first | "Offset & Verify" solver + gates + taper/repair; plain-language directions; before/after 3D overlay; flagged points on plots; sim-pass gate before real test | R9, R14 | 12 | L_P01P02 +10 mm y: 0 failures, max adjacent step ≤ max(1.5 × taught, 0.5°), all joints inside URDF limits − 5°, residual ≤ 1 mm except flagged spans; R2_LIDTOP +10 mm z: singular spans detected and tapered or refused with a plain message; "Test on robot" stays locked until the offset version passes simulation |
| **P9 Packaging & usability hardening** | Install on an operator laptop; non-technical operators succeed | Bundle builder (wheelhouse + sparse snapshot + SDK + fonts); launcher preflight; layout audit (pattern `ALIX/tools/layout_audit.py`); two usability sessions with operators; copy review; docs | NFR-UX, deployment | 8 | Clean-laptop install from the bundle in < 30 min; offline cold start < 10 s; two operators complete Teach → Optimise → Simulate → Dry run without engineer help; layout passes at 1366×768 and 1920×1080 |

**Total: about 100 pd.** P0–P6 is about 70 pd (operator-usable cut); P7–P9 is about 30 pd. Every estimate includes tests. Effort is lowest exactly where reuse is highest: sweep, viewport and robot I/O.

---

## 6. Risks and mitigations

| # | Risk | Mitigation |
|---|---|---|
| 1 | **Monorepo churn breaks Lite.** The branch `feature/alix_studio` has daily commits. | Pin an ALIX commit per Lite release (recorded in `project.yaml` and the bundle). Lite tests run with the ALIX suites. The seams keep their defaults. The JS drift test makes Studio changes a deliberate review. |
| 2 | **The collision world does not match the real cell.** The default venue gives false wall hits (MAP-05 §4.6); the commode pose during teaching is unknown; BinShort vs the physical bin (MAP-11 §1.5). | P0 calibration with site measurements; a per-project world profile; an "unverified world" banner until calibrated; bin variant selectable until the hardware is confirmed. |
| 3 | **"Immediate" validation too slow.** Sweeps take 12–15 s per pair; models rebuild per call. | Warm worker process; windowed re-check of the edited span; content-hash cache; dt adaptive (0.02 s for verdicts, coarser for preview). |
| 4 | **Frame payloads too large for long tracks** (about 16 KB per frame; MAP-02 §2.3). | Moving links only; windowed frames around the cursor; client-side interpolation. Fallback: a JS port of `link_world_transforms` (about 150 lines) behind a bit-parity test, keeping one FK definition. |
| 5 | **Runner subprocess model limits.** It reconnects per action, re-reads the list and stops by signal. | Accepted for the MVP. The Windows stop shim keeps the runner bytes unchanged; `DispatchBackend` follows in P6. Prefer Linux operator laptops. |
| 6 | **Controller semantics that mislead.** An external stop reads as complete; the global speed ratio persists; the effect of transit speed is unverified (MAP-12 §2a–2b). | Always restore speed (runner does; `global_speed_guard` in Dispatch). Label completions "not verified" until `plan_num`/`id` checks or the blocking `-4` probe exist (`DP/track.py:447-490`). Measure the speed semantics in P6 before exposing any speed control. |
| 7 | **Offsets infeasible near singular passages.** The tracks cross q2/q4/q6 ≈ 0 (MAP-05 §2.5). | Detect (singular margin), taper, refuse per span, report "max achievable"; never silently accept jumps; the sim gate is required. |
| 8 | **Slot collisions** with ALIX production 70–100 and V6 71–83 on the same controllers. | Declared Lite slot range per arm, enforced by the ledger; readback of the stored name before overwrite; open decision #5. |
| 9 | **Two tools commanding one arm** (Lite plus the v53 console). | Process interlock; per-arm lock; a documented rule. |
| 10 | **Hand display scale mismatch** (`sim_bridge` 1.60/0.70/1.10 vs `hand_units.py` 1.344/0.5236/1.246). | Use `hand_units.py` everywhere; a unit test. |
| 11 | **Python environment fragility** (numpy/scipy ABI clash; `python-fcl` wheels only). | Pinned venv and wheelhouse; launcher preflight. |
| 12 | **Non-technical users get lost** among Studio-derived expert components. | Guided stepper; plain copy; presets first; usability sessions in P5 and P9; operators never see P-numbers or program IDs. |
| 13 | **Data gaps:** V6 is merged and time-stretched; about 16–17 units have nothing taught; no cleaning times exist (MAP-13 §6.3, §7.4). | Import V6 only as LEGACY reference; the dashboard shows the truth; cleaning times are a decision for the requester; defaults from V6 durations. |
| 14 | **SDK licence unknown** for redistribution. | Bundle without the SDK and point to a site install, if required. |
| 15 | **Dual-arm timing drift inside TOGETHER blocks** (pole model 1.38 s vs ≥ 2.5–3.2 s; no re-sync). | Merger uses the `pole.py` law; the dual timeline shows the skew; the ALIX windowed clearance ≥ 25 mm rule offered as a gate (`ALIX/tools/dual_arm_flow.py`; MAP-06 §5). |

---

## 7. Open decisions (for the requester and the ALIX owner)

1. **Home of Lite:** a sibling package in the ALIX monorepo (recommended), or a standalone repository?
2. **Upstream seams S1–S6:** may Lite land small, default-preserving changes in `alix_workspace`, `alix_dispatch` and `tools/`?
3. **Meaning of "sim" for operators:** the proposed 3-way switch (Simulation / Robot dry run / Real robot), with the emulator hidden as a developer backend?
4. **Operator laptop OS:** Linux (recommended), Windows (needs the stop shim to be verified), or macOS (simulation only)?
5. **Controller slot range** that Lite owns per arm. Does V6's 71–83 stay in ALIX's production range 70–100?
6. **Collision world for the Lite cell:** venue (800 / 1000 / 1400 mm), UGV standoff, commode pose; the bin state during cleaning (STOWED / EXTENDED / LOWERED); BinShort or the original bin.
7. **Joint-limit table for offsets and gates:** URDF (stricter, recommended) minus a 5° margin, or the per-side controller limits?
8. **Clock policy:** gates at 5.0 ms, durations shown as a 5.5545 ms band, recordings assumed at 5.0 ms. Confirm.
9. **Default optimisation methods:** ALIX "Recommended" (anchored pause removal, 5 Hz Butterworth, rating-capped re-time), with HTML "Classic" kept for continuity?
10. **Terminology and IDs:** "Area/Part" vs "Segment"; adopt the MAP-13 ID scheme (L-R04…, FL-R09 → L-R09, R-T08 reserved)?
11. **Cleaning time per unit** (none in the deck), and who sets it.
12. **R8 output form:** one merged `.txt` per arm (literal R8), a sequence of programs with a workflow JSON (V6 / control-app style), or both?
13. **Dual concurrency gate:** require windowed arm-vs-arm clearance ≥ 25 mm for TOGETHER blocks?
14. **Executor for v1:** runner v14 as a subprocess (recommended), or straight to `alix_dispatch` in-process?
15. **Lid checkpoint:** an explicit `checkpoint` step type (workflow v2) instead of v53's "positive WAIT" inference?
16. **Fingers in the collision world:** the full link set (2–3 min per pair) or slim + hand envelopes (seconds)?
17. **RealMan SDK redistribution:** acceptable given the missing licence?
18. **Engineer features kept in the v53 console** (global points, multi-upload) or ported later?

---

## 8. Reuse versus new code (rough estimate)

| Area | Reused (mode) | New code (approx.) | Saved vs from scratch (approx.) |
|---|---|---|---|
| FK/IK/collision/sweep/frames | `alix_workspace`, `body_collision`, `plan_scene_validation` (import) | adapter wiring, worker, offset solver (about 1.5k Python lines) | 25–30 pd |
| Robot I/O | runner v14 (vendor), `alix_dispatch`, emulator (import) | bridge, backends, ledger (about 1.2k lines) | 10–15 pd |
| Trajectory processing and gates | HTML (port), `taught_tracks` math (import) | segments, recipes, suggestions (about 1.5k Python + 1k JS lines) | 6–8 pd |
| UI shell, theme, viewport, DAC, Board | Studio JS/CSS (copy), tokens/vendor (reference) | screens, plots, wizard (about 5–6k JS lines) | 10–12 pd |
| Total | | about 10–11k new lines | **about 50–70 pd** (Lite ≈ 100 pd vs an estimated 150–170 pd from scratch) |

These figures are estimates for planning only. The phase exit criteria are the measurable commitments.
