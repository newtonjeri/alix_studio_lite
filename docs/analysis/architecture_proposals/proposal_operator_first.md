# ALIX Studio Lite: architecture proposal (operator-first lens)

Lens: **operator-first**. The proposal optimises for non-technical operators working on the robot's operator PC. That means a guided and simple app, fast startup, offline use, few dependencies, a robust installer and simple updates.

Date: 2026-10-13. The survey was read-only. No repository or workspace file was changed. This file is the only output.

## Path aliases used in citations

| Alias | Absolute path |
|---|---|
| `ALIX/` | `/home/newtonjeri/alix_ws/src/alix/` |
| `TG/` | `ALIX/core/python/alix_taskgen/src/alix_taskgen/` (Studio: `server.py`, `web_editor/`) |
| `WE/` | `TG/web_editor/` (`template.html` 953 lines, `editor.js` 19,325, `editor.css` 2,166) |
| `WS/` | `ALIX/core/python/alix_workspace/src/alix_workspace/` |
| `DP/` | `ALIX/core/python/alix_dispatch/src/alix_dispatch/` |
| `LITE/` | `/home/newtonjeri/alix_studio_lite_ws/` |
| `UC/` | `LITE/upload_channel/` |
| `V53` | `UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py` (current control UI) |
| `R14` | `UC/rm_upload_fast_v14_safe_transit.py` (current runner) |
| `HTML` | `LITE/Alix_Studio_Lite_Trajectory_V2.html` |
| `REQ` | `LITE/alix_studio_lite/docs/reference/ALIX_Studio_Lite_requirement_inputs.txt` |
| `V6/` | `LITE/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/` |
| `MAP nn` | earlier evidence maps in `scratchpad/maps/nn_*.md` |

---

## 0. Summary

**What Lite is.** ALIX Studio Lite is a standalone, offline application for one job. An operator teaches a region by hand, the app makes the slow recording fast and smooth, the operator checks it in 3D against the toilet and the other arm, tests it on the real arms, and adds it to the robot's program. Lite works only in joint space, on taught `.txt` files. It contains no geometric path generation (X1) and no C-Space or Workspace features (X2) (`REQ:3-5`).

**The five architecture decisions:**

1. **Lite is a standalone package, not a mode of ALIX Studio.** Studio loads its full generator stack at import and takes 30–60 s to start (`ALIX/docs/studio/STUDIO_GENERATE_TAB_PLAN.md:2586-2591`). It resolves every resource relative to the repo (`TG/server.py:56-79`, `WS/package_index.py:27-120`). It also has no execution channel, and a test pins that rule (`ALIX/core/python/alix_taskgen/test/test_studio_interaction_guards.py:2204-2211`). Lite copies a few small pieces from Studio and imports none of Studio itself.
2. **A web UI in a chromeless browser window, served by a small local Python server.** The server uses the stdlib `ThreadingHTTPServer` pattern Studio already uses (`TG/server.py:30`, `:6320-6469`). The ALIX theme is a CSS token system (`WE/editor.css:1-189`), the robot view is three.js, and the trajectory prototype is Plotly 2.35.2, the same version Studio vendors (`HTML:7`; `WE/vendor/plotly.min.js`). A web front end can therefore keep the theme and reuse both code bases as they are.
3. **One guided loop per region.** The steps are Prepare → Teach → Optimise → Check in 3D → Test on robot → Add to program, starting from a dashboard built from the PPT regions. Engineer tools (offset, robot console, advanced parameters) sit behind an "Engineer" drawer.
4. **Heavy work never runs on the UI path.** A single 3D-check worker process owns `alix_workspace` FK and the FCL collision world. FCL managers are not thread-safe (`TG/body_collision.py:148-171`), so there is only one owner. Robot actions run through the field-proven runner **v14**, wrapped unchanged as a subprocess with an explicit `--ip` in v1. A persistent in-process robot service is an optional later phase.
5. **A portable offline bundle.** It contains embedded CPython 3.12, prebuilt wheels, the robot and toilet assets, and vendored JS and fonts. You install it by unpacking and clicking. You update it from a file, and one click rolls it back. Project data lives outside the install folder.

**Effort:** about **98 person-days** for phases P0–P8, for one experienced developer including tests. The operator loop is complete per region after P5, at about 63 pd. An optional persistent robot service (P9) adds about 12 pd.

**Can each requirement be borrowed from ALIX Studio?**

| Req | Short name | Borrow from ALIX Studio? | Main source of the rest |
|---|---|---|---|
| R1 | Region-wise data collection | **Partly**: the Board table, pills and tab badge UI only | New PPT catalogue, timer and auto-advance; drag-teach from the control app |
| R2 | Trajectory editing + undo ×5 | **No**: only Studio's undo *pattern* | Port of the trajectory HTML + ALIX taught-lane algorithms |
| R3 | Simulation playback | **Yes, with adaptation**: robot overlay, FK playback, DAC player | New `.txt` adapter; commode, walls and bin in the world |
| R4 | Real-arm testing, IP entry, real/sim toggle | **No**: Studio cannot command hardware | Control-app runner v14 (+ `alix_dispatch` semantics) |
| R5 | Collision → graph stops and highlights | **Partly**: DAC detection, red links, Prev/Next | New joint plot with stop-at-collision; real-arm error-code mapping |
| R6 | Adjustable plots and windows | **Partly**: dock splitter, dialog grip, maximize patterns | Trajectory HTML plots + new resizable panes |
| R7 | Pole and hand per region before teaching | **No**: Studio never commands pole or hand | Control-app apply verbs + ALIX "pole, then hand" rule + new per-region presets |
| R8 | Merge `.txt` files | **No** | ALIX `concatenate` + dual-arm sweep with commode; new collision-checked bridges |
| R9 | Offset via FK → IK without jumps | **No**: no such tool anywhere | New continuity-engineered solver on `alix_workspace` FK/Jacobian |
| R10 | Dual-arm view left, plots right; correct URDF; bin toggle | **Partly**: URDF, meshes, overlay, DAC | New split layout; new runtime bin toggle; re-baked overlay |
| R11 | Smoothness with immediate validation | **Partly**: Studio's stale-on-edit pattern | Instant client gates + windowed background re-check |
| R12 | Spike analysis + suggestions | **No** | Port of ALIX gates G2/G3 and others + new detectors and fixes |
| R13 | Faster upload | **No** | Control-app upload path + ALIX findings: fewer points, skip unchanged, parallel arms |
| R14 | Offset test in simulation first | **No** (beyond R3 parts) | R9 solver + R3/R5 check pipeline |

---

## 1. Evidence base and what was re-checked here

This proposal builds on the 13 evidence maps (`MAP 01`–`MAP 13`). I re-checked every fact a verdict below depends on:

| Fact | Check (read-only) | Result |
|---|---|---|
| Current control-app pair | `sed -n 45,51p V53` | `UI_PROGRAM_VERSION = "v53"`, `EXPECTED_RUNNER_VERSION = "v14"`. The SDK default path is another machine's home directory (`V53:48-51`) |
| v53 Multi Run / Load Sequence defect | `grep` of the defs and their callers | `_normalize_hand_for_main` (`V53:4853`), `_normalize_pole_for_main` (`:4876`), `_normalize_wait_for_main` (`:4885`) and `_step_preview` (`:5000`) have no `self` and no `@staticmethod`, yet are called as `self._…(…)` (`V53:4932-4934, 5083, 5159, 5212`). Python raises `TypeError` |
| IP handling | `grep` R14 / V53 | The runner has `--ip`/`--port` and `ARM_IPS` from `RM_LEFT_IP`/`RM_RIGHT_IP` (`R14:46-48, 2285-2286, 2347`). The UI never passes `--ip` (0 occurrences) |
| Runner function map | `grep '^def ' R14` | `read_track` :76, `write_project` :91, `slots` :105, `arm_state` :122, `upload` :147, `run` :189, `manual_hand_raw_control` :804, `manual_pole_control` :865, `drag_teach_session` :912, `drag_save_trajectory` :957, run mode :1213/:1229, workflow :1668-2277, `main` :2280 |
| Studio assets | `ls WE/vendor`, `wc -l` | three.min.js (r128), OrbitControls.js, TransformControls.js, plotly.min.js (4,558,696 B), js-yaml. `editor.js` has 19,325 lines |
| Studio functions to lift | `grep` WE/editor.js | `meshFromPayload` :667, undo :1839/:1896, `ensurePlotly` :5322, `setStatus` :5431, `showErrorToast` :5447, `setDockWidth`/`initDockResize` :7580/:7593, `makeDialogResizable` :8470, `renderPlaybackBlock` :8749, `api` :9088, `applyRobotPose` :12427, `showStudioModal` :13786, DAC :14408-14561, Board :14708/:15208 |
| Theme source | `ls ALIX/design`, `sed WE/editor.css` | `ALIX/design/_tokens.css` exists. Brand `#2F96EE` at `WE/editor.css:22`. The dark variant `:root[data-theme="dark"]` starts at `WE/editor.css:134`. Token parity test: `ALIX/core/python/alix_taskgen/test/test_design_tokens.py:41` |
| Collision API | `grep WS/dual_arm_plan_collision.py` | `compute_dual_arm_plan_collisions` :586, `extra_managers` :596/:699, hard-coded `kBodyCheckLinks = ("bin_link","glove_wearing_unit_arm")` :644, `_build_home` defaults :385, `compute_plan_frames` :487 |
| Import trap | `sed WS/plan_scene_validation.py:50-59` | Raises at **import** when no ALIX repo root is found. Lite must copy `load_wall_manager` (:117), not import the module |
| ALIX taught-lane functions | `grep ALIX/tools/taught_tracks.py` | `deviation_mm` :607, `uniform_millideg` :748, `capped_millideg` :767, `smooth_taught` :894, `strip_pauses` :915, `CLOCK_GATE_S` :102, `CLOCK_RECORD_S` :115, `STEP_FAULT_DEG` :128, `PAUSE_TOL_DEG`/`PAUSE_MIN_LINES` :240-241 |
| `.txt`→plan shim | `grep ALIX/tools/dual_arm_verdicts.py` | `hand_rad` :531, `track_points` :580, `build_timeline` :591, `timeline_to_plan` :634, `assert_arm_sides` :685, `sweep` :709, `CLEARANCE_FLOOR_MM = 25.0` :369 |
| `alix_dispatch` pieces | `grep DP/*.py` | `REPLAY_CLOCK_S` `DP/track.py:51`; `parse_track` :66; `build_project_file` :85; `send_project` :101; `program_list` :121; `global_speed_guard` :162; `stop_program` :3109; `run_program` :3517; CANFD limits :3706-3707; `canfd_preflight` :3792; `open_arm` `DP/connect.py:141`; `apply_run_mode` `DP/runmode.py:70`; `ProgramSurface`/`attach` `DP/emulated_program.py:285/:527` |
| Trajectory HTML | `sed`/`grep HTML` | Plotly is loaded from the CDN (`HTML:7`). Pure functions sit at `HTML:1417-1620`, the orchestrator at `HTML:1676` |
| Dev box environment | `python3 --version`, `os-release` | Python 3.12.3, Ubuntu 24.04.5, x86_64. The robot PC runs Linux: the bash prompt is `alai-labs@alailabs` and the path is `/home/alai-labs/...` (`UC/README_COMMANDS.md:44`). That it is Ubuntu is inferred |

---

## 2. The upload-channel programs at a glance (high level only)

This section gives a high-level picture of what the programs do. It is not a bug audit.

| File | What it is | What it does |
|---|---|---|
| `V53` `rm_trajectory_ui_v53_safe_lid_checkpoint.py` | **The current operator UI** (Tkinter desktop window "RealMan // Dual-Arm Operations Console") | **Page 01, Manual + Setup:** two side-by-side arm panels, LEFT 192.168.1.10 and RIGHT 192.168.1.103. Each has tabs RUN (upload a `.txt` to a controller program slot, upload+run, run a stored program, batch upload, list), POINTS (controller-stored global waypoints), DRAG TEACH (record, stop, save, playback), HAND / POLE (6-channel hand with GRIP/OPEN presets, pole 0–315 mm) and TELEMETRY, plus SIM/REAL/CHECK and SLOW STOP per arm. Dual tools start or stop both drags and run both files or both stored programs together. **Page 02, Dual Sequence:** a per-arm workflow builder with program, pole, hand, wait and global steps, dual rules TOGETHER / RUN FIRST / WAIT OTHER, an overall speed, the "SAFE TIMELINE" planner with a manual lid-cover checkpoint, and live arm status. The UI itself never talks to the robot. Every button starts the runner as a subprocess and reads its printed status lines (`MAP 08 §4-5`) |
| `R14` `rm_upload_fast_v14_safe_transit.py` | **The current runner** (command-line tool) | Opens one SDK connection per call and does one action. **Upload:** `.txt` → controller program slot, written as a CRLF `.proj` and verified by stored size (`R14:91-175`). **Run:** runs a slot and watches the run state and error codes (`R14:189`). **List** slots. **Sequences/workflows** of pole → hand → wait → program over one connection (`R14:393`, `:1986`). **Drag-teach:** start, stop, save, playback (`R14:912-1210`). **Manual** pole and hand (`R14:804-909`). **Controller run mode** SIM/REAL (`R14:1213-1262`). **Global points.** Lines 1–268 are the original `rm_upload.py` from the ALIX repo. "Fast" means fast switching between programs, not faster upload (`MAP 09 §3.3`). The `_v52support`/`_v53support` copies are byte-identical backups |
| `UC/hand.py`, `UC/pole.py` | Hardware helpers the runner loads | `hand.py` drives the Inspire hand: six channels 0–1000 over end-port Modbus. `pole.py` drives the lift height and waits for arrival plus a modelled travel time |
| `UC/usage.py` | Example and helper script | States the safety rules ("the pole moves only with the arm still; never move pole or hand with the pad on a surface", `UC/usage.py:25`) and the pre-stroke order (`prestroke` :128, `stop_all` :141, `arm_at_rest` :44) |
| `UC/upload/rm_upload.py`, `UC/py programs/` | History | The original uploader, then UIs v8…v48 and runners v4…v12 |
| `UC/LEFT/RIGHT_V6_70SERIES_FINAL.json` | Production workflows | The V6 dual-arm program sequence: pole/hand/program/wait steps, program IDs 71–83 |

**Which version is current:** UI **v53** plus runner **v14** (`V53:45-46`). v48 is three versions older and v53 contains all of it. Only v53 runs the V6 package, because it adds the manual lid checkpoint (`MAP 08 §0`).

**Issues that affect this plan (one line each):**
1. Multi Run and Page-01 Load Sequence raise `TypeError` in v53 because `@staticmethod` is missing (`V53:4853, 4876, 4885, 5000`). Lite does not reuse those code paths.
2. The UI shows IPs as labels only and never passes `--ip`. Lite must pass `--ip` from its own connection settings (`R14:2285`).
3. The UI does not enforce the runner version. Lite pins the runner by SHA-256.
4. On a real-arm collision the runner reports only an error code and elapsed time, with no joint values (`R14:56-57`). R5 on the real arm needs a joint snapshot (decision D3).

---

## 3. Operator-first design rules (the lens applied)

1. **One loop, one screen at a time.** Every unit (one arm × one region) follows the same steps: Prepare → Teach → Optimise → Check → Test → Program. A unit card always shows the next step as one large primary button.
2. **Plain words, real units.** Use seconds and degrees, not rows and millidegrees. Name regions ("seat top, left half"), not program IDs or slots. Never show file paths. Codes such as `L-R05-P1` appear in small type for traceability. Studio already sets this bar: "no code, no frames, no quaternions visible" (`MAP 01 §7` citing `ALIX/docs/studio/STUDIO_UIUX_RESEARCH.md:438-440`).
3. **Safe by construction.** A real-motion run needs (a) gate-clean bytes, (b) a passed 3D check for this exact version, and (c) a confirmed fixture and bin state. A supervisor override must be recorded with a reason. Confirm only irreversible or physical actions. Everything else can be undone.
4. **Three explicit modes that are never confused.** The modes are **Simulation (no robot)**, **Robot: practice (arms stay still)** and **Robot: real motion**. Each pairs a colour with words, following the ISA-101 rule Studio already applies (`MAP 01 §1.6`). REAL shows a persistent red banner and an always-visible STOP ALL.
5. **Fast and offline.** The UI is usable in about 3 s. The 3D check warms up in the background with an honest "getting ready" chip. Nothing is loaded from the network.
6. **Honest progress and results.** Show real elapsed time and an ETA band, with no fake percentages. Durations use the controller player clock and are shown as a band (5.5545 ms/pt nominal, `DP/track.py:51`). Collision verdicts are full sentences ("Left forearm touches the toilet lid at 12.4 s").
7. **No data loss.** Raw recordings are immutable and hashed. Edits are recipes with a 5-step undo. Writes are atomic. Updates never touch project data.

---

## 4. Architecture

### 4.1 Process model

```
 Desktop icon "ALIX Studio Lite"
        │
        ▼
 launcher  (alix_lite.launcher; bundled python3)
   • single-instance lock (~/.local/share/alix-studio-lite/run.lock + port probe)
   • starts the server, then opens a chromeless window:
     chromium/chrome/edge --app=http://127.0.0.1:8766/?t=<token>
     with --user-data-dir=<lite profile>  (fallback: default browser)
        │
        ▼
 Lite server process  (alix_lite.server; stdlib ThreadingHTTPServer, 127.0.0.1 only)
   ├─ HTTP threads ── /, /vendor/*, /assets/*, /api/lite/*      (session token + Origin check)
   ├─ Project store (filesystem; atomic writes; sha256; schema versions)
   ├─ Job manager (job registry, event log with sequence numbers, cancel)
   │     • robot jobs: one live job per arm (per-arm lock), STOP never queued
   │     • sim jobs: forwarded to the sim worker queue
   ├─ Robot backends (the sim/real seam, §4.6)
   │     • RunnerBackend ──spawns──► python -u runner_v14.py --sdk … --ip … --side … --yes …
   │     │                          (one process = one SDK connection = one action; SIGINT = slow stop)
   │     • PreviewBackend (no hardware; timed playback via the sim worker)
   │     • EmulatorBackend (engineers/tests only; ALIX emulator behind an SDK shim)
   └─ Sim worker supervisor
        │  multiprocessing (spawn) Queue
        ▼
 Sim worker process  (alix_lite.sim.worker; the only owner of FCL managers)
   • warm models: UrdfModel, RobotCollisionModel(collision) via a prebuilt PackageIndex,
     SRDF disabled pairs, commode managers per state, wall managers per venue,
     body managers per bin state
   • jobs: sweep (unit / pair / program), offset (R9), bridge+transit checks (R8)
   • cooperative cancel between samples; results cached by content hash
```

- **Startup path.** Launcher → server import → page served. This path imports only the stdlib and Lite's light modules. numpy, scipy, trimesh and fcl load only in the worker, after first paint. Targets (not yet measured): UI interactive in ≤ 3 s, 3D check ready in ≤ 15 s in the background. Studio's prewarm thread is the pattern to copy (`TG/server.py:6472-6490`).
- **Browser closed mid-run.** Robot jobs belong to the server, not the page. When the window reopens, the UI re-attaches to live jobs through `/api/lite/state`. A stored program keeps playing even if the client dies (`MAP 06 §7.3 item 6`), so the server must keep supervising.
- **Quit.** The File menu "Quit Lite" asks whether robot jobs are active, stops them through the per-arm stop, then shuts the server down. An idle watchdog exits the server after 30 minutes with no UI heartbeat and no robot job.
- **Why polling, not websockets.** Studio's stack is HTTP/1.0 with no request timeout, so long work must be a job, never a held request (`TG/server.py:5814-5819`). Lite polls an event log, `GET /api/lite/jobs/<id>?since=<seq>`, at 4 Hz with a long-poll of at most 1 s. Live joints are polled at 5–10 Hz. This matches Studio's job contract (`TG/studio_generate.py:934-1060`).

### 4.2 Backend modules

| Module | Responsibility | Reuse from (and how) |
|---|---|---|
| `alix_lite.launcher` | Single instance; start the server; open the app window; graphics (WebGL) check on first run; clean shutdown | Pattern from `ALIX/tools/run_studio.py:92-205` (preflight, neutral cwd) and `TG/server.py:6493-6507` (browser open), **copied and adapted** |
| `alix_lite.server.http` | Loopback HTTP handler; gzip above 8 KB; 64 MB POST cap; vendor whitelist; `{error, hint}` convention; token and Origin check; disconnect handling | **Copy** about 200 lines of `TG/server.py:5178, 6320-6469` |
| `alix_lite.server.routes` | `ROUTES_GET`/`ROUTES_POST` plus a **needs table**: `robot_io:<arm>` (exclusive), `sim` (queued), `project_write` (serialized), `none` | Pattern from `TG/server.py:5961-6253` (tables, needs, busy contract), **copied** |
| `alix_lite.server.jobs` | Job registry: start/status/cancel, event log, at most one live robot job per arm | Pattern from `TG/studio_generate.py:537-600, 861-1060`, **copied** |
| `alix_lite.server.page` | Assemble `template.html` with inline CSS/JS; mtime-stamped cache; `no-store` | **Copy** `TG/scripts/visualize_generated_config.py:530-568`, `TG/server.py:6297-6317` |
| `alix_lite.project.store` | Project folders, atomic writes, sha256, schema version + migrations, backups | **New** |
| `alix_lite.project.catalogue` | The 44 arm-units: IDs, aliases, phase order, pairing, fixture state, defaults (pole/hand/time), auto-advance rules | **New** from `MAP 13 §5.2, §7.3`; defaults from `V6/*_FINAL.json` and `ALIX/experiments/exp01_teach_trajectory_reverse_engineering/data/teach_files/commode_c_taught_paths/description.txt:3-16` |
| `alix_lite.project.provenance` | Per-recording `.source.json` (side, pole, hand counts, taught_by, date, hardware ok, sha256, clock) | Schema from `ALIX/Resource/tracks/taught/source/README.md:8-27`, **copied** |
| `alix_lite.traj.io` | Strict NDJSON reader/writer (7 ints, ≥2 points); LF/CRLF handling | **Port** `R14:76-97` and `DP/track.py:66-98` (raise, do not `sys.exit`) |
| `alix_lite.traj.pipeline` | Recipe → output. Trim: ALIX anchor rule by default, HTML rule as "legacy". Smooth: 5 Hz zero-phase Butterworth by default, HTML moving mean with fixed edges as "legacy". Per-segment speed with ramps; rating clamp | **Port** `HTML:1417-1616` + `ALIX/tools/taught_tracks.py:748-951` (numpy/scipy) |
| `alix_lite.traj.gates` / `.spikes` / `.timing` | G2 (rating at 5.0 ms), G3 (≤ 10°), limits with 5° allowance, accel flag, one-tick Δv, endpoint snap, boundary jump, isolated spikes, stalls; duration band from the player law; suggestions | **Port** `ALIX/tools/taught_tracks.py:94-138, 607-716, 954-991`; `DP/track.py:1537-1570, 3706-3866`; detectors from `MAP 06 §4`, `MAP 07 §9` |
| `alix_lite.sim.assets` | Locate the asset bundle; prebuilt `PackageIndex`; `ALIX_SRDF`; home pose | **Import** `alix_workspace` (`WS/package_index.py:75-121`, `WS/srdf_states.py:61-154`) and pass `index=` to `RobotCollisionModel.from_file` (`WS/collision.py:55`) |
| `alix_lite.sim.worker` | Spawned process; warm models; job queue; cancel; cache | Pattern of Studio's spawn executor (`TG/server.py:700-725`); **new** |
| `alix_lite.sim.adapter` | `.txt` → timeline/plan: millideg → rad, clock, **pole/hand/bin seeding** | **Port** `ALIX/tools/dual_arm_verdicts.py:531-731`; hand scale from `ALIX/ros/hw/alix_hw/scripts/hand_units.py:33-60`, not the Studio `sim_bridge` table (`MAP 05 §1.4`) |
| `alix_lite.sim.world` | Collision world: arm vs arm, self, **body set parameterised by bin state**, **commode by fixture state**, **walls by venue** | **Import** `WS/dual_arm_plan_collision.py:586-760` (`extra_managers`, `include_body`). **Copy** about 60 lines of `TG/body_collision.py:401-464` (`fixture_manager`) and about 40 lines of `WS/plan_scene_validation.py:117-155` (`load_wall_manager`), because those modules need an ALIX repo root at import |
| `alix_lite.sim.sweep` | Coarse sweep (dt 0.05 s), refinement around hits (0.02/0.01 s), keeps `joint_timeline`, plain-language verdict sentences, cache by content hash | Library call above; pair classes from `WS/redundancy.py:90-117`; cache idea from `TG/server.py:4303-4316` |
| `alix_lite.sim.offset` | Offset & Verify for R9/R14: FK, ramps, differential IK with taught-increment feed-forward and null-space pull, gates, taper | **Import** `WS/urdf_kinematics.py:218-343`, `WS/ik.py:150-334` (Jacobian), `WS/srs.py`, `WS/srs_ik.py`. The algorithm is **new** (`MAP 05 §8.2`) |
| `alix_lite.sim.merge` | Join units into programs (≤ 0.5° gap); velocity-matched, rate-limited bridges; transit checks | **Port** `ALIX/tools/plan_min_time.py:142-171`; bridges and checks **new** |
| `alix_lite.robot.backend` | `RobotBackend` protocol + event types (§4.6) | **New** |
| `alix_lite.robot.runner_backend` | Wraps runner v14 + `hand.py` + `pole.py` **unchanged** (sha-pinned); builds the argv with `--ip`; parses stdout tokens into events; SIGINT stop | **Vendor** `R14`, `UC/hand.py`, `UC/pole.py`. Token list from `V53:5753-5961` (`MAP 09 §1.4`) |
| `alix_lite.robot.preview_backend` | "Simulation" runs with no hardware: timed playback at the player clock, pole/hand timing laws, collisions from the sweep, same events as real | **New**. Timing from `DP/track.py:51, 1537-1570` and `UC/pole.py:3-14` |
| `alix_lite.robot.emulator_backend` | Developer dry-run: runner v14 through an SDK shim that installs the ALIX emulator and wraps `ProgramSurface` | **Vendor** `ALIX/core/python/src/alix_emulator/` (verbatim) + `DP/emulated_program.py:285-547`. Linux/Windows only (`MAP 11 §4.3`) |
| `alix_lite.robot.ledger` | Per-controller slot ledger (slot → name, size, sha256, verified) to skip unchanged uploads; slot-range policy | Pattern from `ALIX/tools/cleaning_tracks.py:4046-4110`; **new** |
| `alix_lite.robot.planner` | Dual timeline: TOGETHER blocks, RUN FIRST/WAIT OTHER, **explicit checkpoint step** | **Port** the pure functions `_page2_program_chunks` / `_page2_build_program_rule_plan` (`V53:9619-9997`, `MAP 08 §4.3`) |

### 4.3 HTTP API (`/api/lite/*`, JSON; refusals are HTTP 200 `{error, hint}` like Studio)

| Method | Route | Needs | Purpose (req) |
|---|---|---|---|
| GET | `/api/lite/state` | none | Version, project, mode, arms (ip, connected, run mode, busy job), sim worker readiness, live jobs |
| GET | `/api/lite/catalogue` | none | Units + statuses for the dashboard (R1) |
| GET | `/api/lite/unit?id=` | none | Unit card: presets, takes, recipe, versions, results (R1, R7) |
| GET | `/api/lite/track?unit=&kind=raw\|opt&v=` | none | Joint rows (int32 arrays) + an overview for plots (R2, R6) |
| GET | `/api/lite/robot-geometry?set=visual\|collision` | none | Robot meshes + home pose (copied from `TG/server.py:1918-1998`) (R3, R10) |
| GET | `/api/lite/fixture?state=closed\|lid_open\|lid_seat_open&venue=` | none | Commode and wall meshes for the 3D view (R3, R10) |
| GET | `/api/lite/jobs/<id>?since=` | none | Job events (all long work) |
| GET | `/api/lite/live-joints` | none | Latest joints of both arms (preview or real) (R4, R5) |
| POST | `/api/lite/project/{new,open,import-raw,import-v6}` | project_write | Projects; guided import of existing files; V6 legacy split (R1) |
| POST | `/api/lite/unit/presets` | project_write | Pole mm, hand preset, cleaning time, fixture state (R1, R7) |
| POST | `/api/lite/teach/{prepare,start,stop,save}` | robot_io:arm | Pole → hand → drag session → save named file + provenance (R1, R7) |
| POST | `/api/lite/recipe/{apply,undo,redo,reset,save-version}` | project_write | Edits with undo ×5 (R2, R11) |
| POST | `/api/lite/analyze` | none | Gates, spikes, suggestions (R12) |
| POST | `/api/lite/sim/check` | sim | Sweep a unit, pair or program (bin state, fixture state, venue) (R3, R5, R10, R14) |
| POST | `/api/lite/robot/connect` | none | IPs, test connection, read the run mode (R4) |
| POST | `/api/lite/robot/mode` | robot_io:arm | practice (controller SIM) / real, verified by read-back (R4) |
| POST | `/api/lite/robot/upload` | robot_io:arm | Upload with ledger skip; both arms in parallel (R13) |
| POST | `/api/lite/robot/run` | robot_io:arm | Run with pre-run gates (R4, R5) |
| POST | `/api/lite/robot/stop` | (bypasses locks) | Stop one arm or ALL; never queued (safety) |
| POST | `/api/lite/robot/verdict` | project_write | Operator verdict after a test (R4) |
| POST | `/api/lite/program/{build,check,export,upload}` | sim / robot_io | Merge, bridges, dual timeline, workflow JSON export, batch upload (R8, R10, R13) |
| POST | `/api/lite/offset/{preview,apply}` | sim / project_write | Offset & Verify (R9, R14) |

### 4.4 Front end

**Shell (always visible).** Copied from Studio: app bar with the inline-SVG brand mark (`WE/template.html:15-51`), retitled "ALIX Studio Lite". Next to it sit:
- the project name;
- the **mode switch** (three large segments with words);
- **LEFT/RIGHT arm chips** (connected dot, IP, controller mode);
- a big **STOP ALL** whenever a robot connection exists;
- `?` help.

A left rail holds the six steps as large buttons with a progress badge, plus a collapsed **Engineer** drawer. Status line, toast and confirmation modal are copied (`WE/editor.js:5431-5467, 13786-13832`).

| Screen | Purpose | Reqs |
|---|---|---|
| **Dashboard** | Matrix of the 22 deck steps × LEFT/RIGHT (toggle: 13 region families). Each cell shows the unit, a thumbnail and a status chip with glyph and word: NOT TAUGHT / RAW / OPTIMISED / 3D OK / ROBOT OK / IN PROGRAM, plus flags LEGACY V6, NEEDS RE-TEACH, FAILED, BLOCKED. Counters per arm and phase. One-click **"Teach next missing"** | R1 |
| **Regions & Teach** | Left: clickable PPT region map (PAGE 04/05 renders + FLOOR SPILLAGE hotspot). Centre: unit card with the picture and overlay, a plain title, the fixture-state question ("Is the lid UP and the seat DOWN?") and a tool check. Right: **Before you start** (pole height and hand preset with Apply + readback) → big **START TEACHING** → countdown ring to the target cleaning time (extend / done early, beep) → **STOP & SAVE** → review mini-plot → Redo / Accept / Skip → auto-advance, with interstitials for fixture-state changes | R1, R7 |
| **Optimise** | Per unit: preset Gentle / Standard / Fast; segment strip (drag on the plot to create sub-regions; each overrides speed, smoothing and trimming); **Undo (5)** with labels; findings panel (spikes and limits with one-click fixes); stats (duration band, points, % saved, peak rating use). Original vs optimised plots in seconds and degrees; resizable panes, plot height handles, maximize, linked zoom. Advanced drawer holds the raw parameters | R2, R6, R11, R12 |
| **Check in 3D** | **Left:** dual-arm 3D (robot, commode in the unit's state, walls toggle, **bin: Lowered / Extended / Stowed / Not fitted**, glove-pad trail to help spot missed areas). **Right:** joint plots for LEFT and RIGHT on one time axis with collision bands. Transport ×0.5/×1/×2/×4; **stop at first problem** on by default; red links + red joint markers + sentence; Prev/Next; **Back to editing** (pre-selects ±1 s) | R3, R5, R10, R11, R14 |
| **Test on robot** | Connection card (IP fields, test); mode; pre-run gate chips (bytes OK, 3D OK for this version, fixture state, bin state, people clear); Upload (skip-unchanged, progress); Run (transit 30 % default); live 3D mirror + plot cursor; on stop or collision the plot stops at the row → **Back to editing**; verdict buttons Clean OK / Missed spot / Collision / Other | R4, R5, R13 |
| **Program & upload** | Per-arm ordered unit list (deck order by default, drag to reorder); automatic grouping into programs (split at pole/hand/fixture-state changes); bridges preview; dual timeline (TOGETHER / first-then-other / **checkpoint** "Open the lid now"); whole-program 3D check; export; **Upload all** | R8, R10, R13 |
| Engineer: **Adjust position** | Pick a unit and segment, a direction in words (towards / away from toilet, left / right, up / down, press deeper) and mm; before/after ghost; "max achievable here" when tapered; then check in 3D and test on the robot | R9, R14 |
| Engineer: **Robot console** | Manual pole/hand (6 channels), list slots, upload/run a file, run mode, telemetry log with translated error codes (replaces Page 01 for engineers) | R4, R7, INT-CTRL |
| **Settings & About** | Robot IPs/port, SDK folder, slot ranges per arm, data folder, venue and commode placement, theme light/dark, large text; versions (Lite, runner sha, ALIX commit, assets hash); install update from file / roll back; export support bundle | R4, deployment |

**Front-end files:**

| File | Contents |
|---|---|
| `lite_tokens.css` | `WE/editor.css:1-129, 134-189, 1714-1724` without the atlas tokens. Parity-tested against `ALIX/design/_tokens.css` |
| `lite_base.css`, `lite_shell.css` | Filtered ranges listed in `MAP 01 §8.3`. Fixes the `.info-row` nesting and the undefined tokens |
| `lite_shell.js` | Status, toast, modal, `api()`, splitters, dialogs, `ensurePlotly`, error capture |
| `viewer.js` | `initThree` (trimmed), `meshFromPayload`, `loadRobotGeometry`, `applyRobotPose` (`WE/editor.js:309-645, 667-703, 12352-12437`) + **client-side FK** |
| `dac.js` | `WE/editor.js:14408-14598`, adapted |
| `traj_core.js` | `HTML:1417-1620` |
| `plots.js` | New `plotTheme()` from tokens |
| Screens | One module per screen |

### 4.5 Data model and folder layout

**Core object: the Unit.** A unit is one arm × one teachable region or task from the deck, with IDs from `MAP 13 §5.2`, for example `L-R05-P1` "seat top, left half". A **segment** (sub-region) is a time range inside a unit's track. The deck's "part 1 / part 2" become separate units when taught separately, or segments when taught in one take.

**Status pipeline per unit:**

```
NOT_TAUGHT → RAW → OPTIMISED → CHECKED_3D (pass|fail) → ROBOT_TESTED (ok|missed|collision) → IN_PROGRAM → UPLOADED
```

Any upstream change marks the downstream states *stale* (amber, glyph + word). This is Studio's `markDirty → invalidateValidation` idea (`WE/editor.js:5371-5430`).

```
~/ALIX-Lite/                                   data root (settable; never inside the install folder)
  settings.json                                IPs, port, SDK folder, slot ranges, venue, operator list, UI prefs
  logs/2026-10-13.jsonl                        structured events: robot commands, results, verdicts, errors
  projects/<name>/                             e.g. commode_c_site01
    project.json                               schema, fixture (commode_c), venue, default bin state, catalogue version
    catalogue.yaml                             44 units: id, deck aliases, arm, type, step, paired_with, sync, fixture_state,
                                               cleaning_time_s, pole_mm, hand_preset(+6 values), tool, image box
    units/<UNIT-ID>/
      unit.json                                status + timestamps + chosen take + current version
      raw/<UNIT-ID>__RAW__<yyyymmdd-hhmmss>.txt        immutable (read-only after save)
      raw/<UNIT-ID>__RAW__<ts>.source.json             provenance + sha256
      edits/recipe.json                        current recipe
      edits/history.json                       undo ring: last 5 recipes with labels (persists across restarts)
      opt/<UNIT-ID>__OPT_v<k>.txt + .report.json      derived bytes; recipe hash; source sha; gate results
      checks/<content-hash>.json               3D verdicts (collisions, windows, pairs, settings used)
      tests/<ts>.json                          robot runs: mode, slot, rc, codes, elapsed, joint snapshot, verdict
    programs/<program-name>/
      program.json                             ordered units per arm, groups, bridges, dual blocks, checkpoints, slots
      LEFT_<id>.txt / RIGHT_<id>.txt           merged program files
      LEFT.workflow.json / RIGHT.workflow.json realman_workflow_sequence_v1 (v53-compatible export)
      timeline.json                            explicit dual timeline (the file V6 never produced, V6/README.txt:78)
      checks/…, upload_log.json
    ledger/<left|right>.json                   controller slot → name, size, sha256, uploaded_at, verified
  legacy/V6/                                   imported reference segments (not counted as raw)
```

**Recipe** (drives the derived bytes deterministically; undo restores recipes, not arrays):

```json
{ "schema": "alix_lite_recipe_v1", "unit": "L-R05-P1",
  "source": {"file": "raw/L-R05-P1__RAW__20261013-101500.txt", "sha256": "…"},
  "trim":   {"method": "anchor", "tol_deg": 0.05, "min_lines": 3},
  "smooth": {"method": "butterworth", "hz": 5.0},
  "speed":  {"base_pct": 100, "rating_cap": 0.97, "ramp_per_line": 0.05},
  "segments": [{"id": "s1", "start_s": 0.0, "end_s": 12.4, "speed_pct": 40, "smooth": null, "trim": null, "label": "approach"}],
  "offset": null, "edited_by": "…", "edited_at": "…" }
```

**Program** (R8/R10): `{"schema":"alix_lite_program_v1","fixture":"commode_c","arms":{"left":[{"pole":150},{"hand":"GRIP"},{"program":{"slot":41,"units":["L-R04","L-R01"],"file":"LEFT_41.txt"}},{"checkpoint":"Open the lid cover now, step back, press Continue"}],…},"dual":[{"block":"T1","rule":"TOGETHER","left":[41],"right":[41]},…],"slot_range":{"left":[40,59],"right":[40,59]}}`.

The export writes per-arm `realman_workflow_sequence_v1` files. A positive WAIT before a TOGETHER program on both arms encodes the checkpoint, so the v53 app can still run them (`MAP 10 §2.4`).

### 4.6 The sim/real seam

One interface; every screen uses only this interface and its events.

```python
class RobotBackend(Protocol):
    def connect(self, arm, ip, port=8080) -> ConnInfo                 # test + read run mode
    def run_mode(self, arm) -> Mode; def set_run_mode(self, arm, mode) -> Mode   # read-back verified
    def list_programs(self, arm) -> list[Slot]
    def upload(self, arm, txt_path, slot) -> Job                      # events: progress, verified(size,name) | failed
    def run(self, arm, slot, transit_pct=30) -> Job                   # events: started, joints, code(n,t), finished(rc) | stopped
    def stop(self, arm) -> None                                       # never queued
    def pole(self, arm, mm) -> Job; def hand(self, arm, values6) -> Job
    def drag_start(self, arm) -> Job; def drag_stop(self, arm); def drag_save(self, arm, path) -> Job
    def joints(self, arm) -> JointSample | None                       # live mirror
```

| Operator mode | Backend | What happens | Teach possible? |
|---|---|---|---|
| **Simulation (no robot)**, the default | `PreviewBackend` | Upload is a no-op. Run plays the bytes at 5.5545 ms/pt in 3D, pole and hand use the timing laws, and collisions come from the sweep. Events are the same as on the robot. Works on any OS | No ("Teaching needs the robot") |
| **Robot: practice (arms stay still)** | `RunnerBackend` + controller run mode SIM, verified by read-back (`R14:1229-1262`) | Real upload and real program run on the controller with no motion. Pole and hand are **not** simulated by the controller (`MAP 09 §7`), so practice skips them with a notice. A 4109 can still appear from the controller's own models | No (drag-teach is a REAL-mode activity, `MAP 09 §7`) |
| **Robot: real motion** | `RunnerBackend` + REAL | Confirmation modal with gates; red banner; STOP ALL | Yes |
| Emulator (Engineer drawer only) | `EmulatorBackend` | Runner v14 against the ALIX emulator via an SDK shim (`MAP 11 §4.4`). Upload, list, run, pole and hand work; drag, waypoints and rename do not | No |

Optional P9 adds `NativeBackend` (`lite_rm`): persistent per-arm sessions, stop token, joint push, single-session batch upload, Windows robot I/O. It plugs into the same seam.

### 4.7 Reuse strategy per source

| Source | Strategy | What exactly | Why |
|---|---|---|---|
| `alix_workspace` | **Import** as a pinned wheel built from a recorded ALIX commit | FK/IK/SRS, `RobotCollisionModel`, `PackageIndex`, `SrdfStates`, `compute_dual_arm_plan_collisions`, `compute_plan_frames`, `analyze_plan` | ROS-free with a small dependency set (`ALIX/core/python/alix_workspace/pyproject.toml:12-21`); FK matches the controller (`MAP 05 §0`). Private helpers (`_ArmCollision`, `_build_home`) are used only through a Lite wrapper with tests. Ask upstream for `body_links`/`bin_state` parameters and in-memory plan input (decision D11) |
| `alix_taskgen` (Studio) | **Copy** small pieces; **never import** | Server scaffold, page assembly, tokens, shell JS/CSS, robot overlay, DAC player, board/pill CSS, `fixture_manager` logic, `pole_bin_conflict` rule; a `VENDORED.md` lists path, commit and sha per piece | `import alix_taskgen.server` pulls in the generator stack (`TG/server.py:34-44`). The pieces are small and stable. Test guards are copied too (token parity, no remote assets, JS integrity) |
| Runner v14 + `hand.py` + `pole.py` | **Vendor unchanged** + wrap as a subprocess (v1) | sha-pinned copies under `alix_lite/robot/vendor/runner_v14/` | Field-proven hardware safety (CRLF, `only_save=1`, read-back verify, speed restore, slow stop; `MAP 09 §9.3`). Lowest risk to first real use |
| v53 UI | **Port** only the pure planner; **re-express** UI concepts in the web shell; drop Tk | `_page2_program_chunks`, `_page2_build_program_rule_plan`, the checkpoint stage, the live-status vocabulary | The planner is pure logic (`MAP 08 §5` item 6). The Tk palette and layout clash with the ALIX theme (`V53:56-71`) |
| `alix_dispatch` | **Vendor** a subset (unchanged files + seam wrappers) | `track.py` constants and preflight, `connect.py`, `runmode.py`, `emulated_program.py` (tests / emulator backend) | No pyproject; reached through `sys.path` (`ALIX/LAYERS.md:25`). ROS-free with stdlib imports (`MAP 06 §7.1`) |
| `alix_emulator` | **Vendor verbatim** (dev/test only) | `rm_emulator.py` + `install()` | Verbatim-pinned upstream (`MAP 04 §4.1`); needs the real SDK `Algo` |
| ALIX taught lane (`tools/taught_tracks.py`, `tools/dual_arm_verdicts.py`, `tools/plan_min_time.py`) | **Port** about 15 pure functions, with golden-file parity tests against ALIX outputs | `strip_pauses`, `smooth_taught`, `uniform/capped_millideg`, `deviation_mm`, rate/step reports, slowness census, `concatenate`, the `.txt` → timeline shim | Importing pulls in the whole lane and `experiments/` (`MAP 06 §7.2`) |
| Trajectory HTML | **Port** pure functions to `traj_core.js` (instant preview) **and** to Python `traj/pipeline.py` (canonical), with parity tests; **restyle** the views | `HTML:1417-1620` (+ analysis `HTML:944-1050` as reference) | Single file with no persistence or validation; defects documented in `MAP 07 §9` |
| ALIX robot/toilet assets | **Copy** into an asset bundle by a build script (`tools/build_assets.py`) | `WS/urdf/alix.urdf`, 80 collision STLs (6.4 MB), **re-baked** `visual_overlay_baked.npz`, `alix.srdf`, `initial_positions.yaml` ×5, commode_c `scene.yaml` + body/lid/seat STL, venue YAML + wall STL, PPT renders | Runtime must not need the ALIX checkout (`MAP 04 §2.3`). The current bake is stale: old tall bin, no camera (`MAP 11 §2.4`) |
| X1/X2 machinery | **Not ported** | Workbench/Generate, region painting, coverage, reachability, manipulability, redundancy atlas, path-ik/path-compare/auto-path, Workspace and Gizmo tabs (38 of 64 Studio routes, `MAP 04 App. A`) | Exclusions (`REQ:3-5`). SRS math may run internally for IK repair; there is no C-space UI |

### 4.8 Control-app integration (INT-CTRL)

1. **v1 (P2/P5): wrap, do not rewrite.** `RunnerBackend` calls the vendored R14 exactly as v53 does: `[python, -u, runner, --sdk, SDK, …, --side S, --ip IP, --yes]`, with a new session or process group (`V53:4255-4262, 5733-5751`). Lite always passes `--ip`, which v53 never did. Lite confirms in its own modal before passing `--yes`. Stop sends SIGINT to the process group; the runner turns it into `rm_set_arm_slow_stop`, a pole stop and a hand freeze (`V53:6051-6061`; `MAP 09 §8`).
2. **Events instead of UI state strings.** The runner's printed tokens (`VERIFIED:`, `RUN COMPLETE: X s`, `DRAG TEACH STOPPED:`, `ARM RUN MODE:` and so on; the full list is in `MAP 09 §1.4`) are parsed into typed events. Parser tests replay **recorded transcripts**. Completion is decided by exit code plus tokens, never by UI label text (a fragility in v53, `MAP 08 §5`).
3. **Startup handshake.** The sha256 of `runner_v14.py`, `hand.py` and `pole.py` must match the pinned values, otherwise robot features are disabled with a plain message. The SDK folder is taken from Settings. The default `--sdk` fallback is broken outside the ALIX repo layout (`MAP 09 §2`).
4. **Joint snapshot for R5 (decision D3).** Preferred: a small **additive** runner patch, "v14-lite", that prints `JOINTS: [..]` from the `arm_state()` call the runner already makes every 0.5 s and at every error code (`R14:122-140, 245-251`). No other behaviour changes; the control-app owner reviews it. If rejected, Lite estimates the row from elapsed time minus transit (±6 % band) and says so.
5. **What carries over from v53 conceptually.** Per-arm cards with mode; STOP ALL; dual barrier launch, moved into the server (`V53:5654-5710`); TOGETHER / RUN FIRST / WAIT OTHER and the lid checkpoint, made an **explicit step type**; live arm status words; the REAL confirmation; the v32 "controller is in SIMULATION, the arm will not move" warning that v35 dropped (`MAP 08 §2` item 2), restored.
6. **Transition.** The v53 Tk app stays installed for engineers while Lite matures. Lite exports v53-compatible workflow JSON, so programs built in Lite can still run from v53 (decision D12).
7. **Later (P9, optional).** `lite_rm` ports the SDK-calling core of R14 into a library. It keeps a persistent connection per arm (`rm_set_timeout(1500)` once), batch upload in one session, a stop token instead of signals (needed on Windows), and UDP push joints. Equivalence is proven against the emulator, ALIX's fake-controller tests (`ALIX/core/python/alix_dispatch/test/test_rm_upload.py:1-212`) and A/B runs against R14.

### 4.9 Trajectory HTML integration (INT-TRAJ)

1. **Becomes the Optimise screen.** Its 6-step stepper (`HTML:609-704`) is replaced by the unit loop. Its plots, box-select-to-create-segment (`HTML:872-892`) and cursor/playback (`HTML:1134-1142, 1217-1232`) are restyled with ALIX tokens and vendored Plotly.
2. **Two implementations, one truth.** `traj_core.js` (a port of `HTML:1417-1620`) gives instant previews while sliders move. Python `traj/pipeline.py` is canonical: it writes the bytes that get checked, uploaded and merged. A golden test asserts ≤ 1 millidegree difference. The earlier map reproduced `zip/02` from `zip/01` to ≤ 1 mdeg with the HTML method (`MAP 10 §1.4`).
3. **Algorithm fixes** (from `MAP 07 §9`):
   - Endpoint snap from the mean filter (R-A1): keep edge dwell / ramp edges.
   - Step-to-step idle rule cuts slow apexes (R-A2): the ALIX anchor rule becomes the default (`ALIX/tools/taught_tracks.py:915`).
   - Over-smoothing (R-A3): 5 Hz zero-phase Butterworth becomes the default (`:894`).
   - No rating check, which let 10 of 15 V6 files exceed joint ratings (R-A4): clamp at 0.97 × rating at 5.0 ms.
   - Speed steps at segment boundaries (R-A5): ramp λ ≤ 0.05 per line.
   - Wrong duration clock (R-T1): show the band at the player clock.
   - Positional heuristics (R-A6): replace them with rating-based suggestions.
4. **UI defects fixed in the port** (`MAP 07 App. B`): stale state after a scale edit, regions not drawn, compare locked before generate, reset keeping parameters, N vs N−1 time.
5. **Removed:** browser downloads and Save-As. Results save to the unit's folder on the server.

### 4.10 Deployment, packaging, updates

| Topic | Proposal |
|---|---|
| Target machines | **Primary:** the robot operator PC, Linux x86_64. Build for Ubuntu 24.04 (the robot PC runs Linux per `UC/README_COMMANDS.md:44`, Ubuntu inferred; dev box Ubuntu 24.04.5 / Python 3.12.3; confirm under D1). **Secondary (later):** office laptops on Ubuntu or Windows 11 for offline optimise and check only. macOS: offline features only; no SDK library for macOS (`MAP 04 §3.3`) |
| Bundle | `alix-studio-lite-<ver>-linux-x86_64.tar.gz`. Contents: relocatable CPython 3.12 (python-build-standalone); pre-installed numpy, scipy, pyyaml, trimesh, python-fcl 0.7.x, pycollada, lxml, networkx; `alix_workspace` wheel from the pinned commit; `alix_lite`; asset bundle; vendored JS + Inter / JetBrains Mono woff2; runner v14 + hand/pole; RealMan SDK Python + `libapi_c.so` if the licence allows (decision D2). Estimated 200–300 MB (inferred) |
| Why not pip/venv on site | python-fcl ships wheels only and no sdist (`ALIX/SETUP.md:469-475`); the system Python here has a numpy/scipy ABI clash (`MAP 05 §7`); operators cannot run pip. A frozen interpreter + wheels removes all three problems |
| Why not PyInstaller | The runner must keep running as a separate Python script (`sys.executable -u runner.py`). An embedded interpreter keeps that natural and debuggable |
| Install | `./install.sh` (no root). Unpacks to `~/.local/opt/alix-studio-lite/versions/<ver>/`, points the `current` link at it, writes a `.desktop` entry with icon, creates `~/ALIX-Lite/`, runs a self-test (imports, assets hash, WebGL page, runner sha). `uninstall.sh` removes the program, never the data |
| Updates | About → **Install update from file** (USB or download). Verifies the sha256 (+ optional signature), unpacks next to the current version, self-tests, switches `current` on the next start, keeps the previous version for **one-click rollback**. Data schema migrations make a backup first |
| Offline | All JS, fonts and assets are local. A test fails on any remote `<script>`/`<link>`/`<img>` (copied Studio guard, `MAP 04 §5`). Plotly is vendored, not the CDN line `HTML:7` |
| Startup budget (targets) | Launcher + server ≤ 1.5 s; first paint ≤ 1 s (Plotly lazy-loaded; robot geometry after first paint, from the re-baked npz); sim worker warm ≤ 15 s in the background with a status chip |
| Security | Binds 127.0.0.1 only. A random session token is injected into the page; every POST requires it plus a same-origin header. Robot endpoints also take the per-arm lock. Studio warns about non-loopback binds for a reason (`TG/server.py:6539-6544`) |
| Support | Export support bundle: logs, versions, settings and project metadata; raw files only if ticked |

---

## 5. Requirement-by-requirement findings, verdicts and approach

### R1: Region-wise data collection (`REQ:8`)
- **Finding.** Studio's Regions tab is a mesh-painting editor that feeds the geometric generator (X1), so it cannot be ported (`MAP 03 §0`). Studio has no guided, wizard or timer flow (`WE/template.html:116-118`). Drag-teach exists only in the control app, with timestamp-only file names (`V53:3840-3850`). The deck has 22 steps / 44 arm-units with inconsistent codes; `MAP 13 §5.2` proposes a normalised ID scheme. No raw per-region file exists in the V6 package (`MAP 13 §6.3`).
- **Borrow from ALIX Studio:** **partly.** Take the Board table, pills, chips and tab badge UI (`WE/editor.js:14708-14757, 15208-15289`; `WE/editor.css:1507-1528, 1976-1998`). Do not take its backend (it reads task-config YAML, `TG/server.py:5396-5486`).
- **Approach.**
  - Catalogue file plus PPT renders and overlay boxes, built at release time.
  - Dashboard over unit folders.
  - Guided teach state machine: fixture-state question → bin/pole foul-band check → pole → hand → START → countdown → STOP & SAVE (named file + provenance) → review → Redo/Accept/Skip → auto-advance.
  - Teach order: all left units of a fixture phase, then all right units, then the state change. One operator can drag only one arm at a time (`MAP 13 §7.2`).
  - Default cleaning times are set by the supervisor. Reference values: ALIX raw teach lengths are 41.7–66.0 s per region and 16.5–23.7 s per mechanism (`MAP 10 §1.3`).
- **Phases:** P1 (dashboard, regions) and P2 (teach flow).

### R2: Trajectory editing per region and sub-region, undo ×5 (`REQ:10`)
- **Finding.** The HTML has per-sub-region **scale only**. Trim and smooth are global, and it has **no undo** (`MAP 07 §8`). Studio's undo is one global snapshot stack of depth 80 (`WE/editor.js:1839-1977`).
- **Borrow:** **no.** Only the snapshot pattern carries over.
- **Approach.**
  - Region = unit; sub-region = time segment.
  - Each segment can override trim, smooth and speed.
  - Undo history per unit holds the last **5 recipe snapshots**, labelled (for example "Speed of 'approach' 60 % → 40 %"), with one entry per slider gesture. It persists across restarts. Reset to raw is always available.
  - Recipes, not arrays, are stored. Outputs are deterministic from raw + recipe.
- **Phase:** P3.

### R3: Simulation playback (`REQ:12`)
- **Finding.** Studio has the robot overlay (`TG/server.py:1918-1998`; `WE/editor.js:12352-12437`), FK playback (`WS/dual_arm_plan_collision.py:487`) and the DAC player (`WE/editor.js:14408-14598`). Gaps:
  - It accepts only plan JSON under repo plan roots (`TG/server.py:4187-4197`).
  - Its collision world omits the commode, walls and fingers.
  - The bin is always LOWERED, the pole defaults to 0.300 m and the hands to open (`MAP 02 §3.4`).
  - Frame payloads are about 16 KB/frame (`MAP 02 §1.6`).
- **Borrow:** **yes, with adaptation.**
- **Approach.**
  - `.txt` adapter with pole/hand/bin seeding (port of `ALIX/tools/dual_arm_verdicts.py:580-660`).
  - Commode by fixture state and walls by venue, passed as `extra_managers`.
  - **Client-side FK** for smooth playback (a port of `link_world_transforms`, `WS/urdf_kinematics.py:294-343`, about 150 lines). A parity test against the server must agree to ≤ 0.01 mm on 10k poses.
  - Glove-pad trail in 3D to help spot missed areas (FK only, not X1).
- **Phase:** P4.

### R4: Real-arm testing, IP entry, real/sim toggle (`REQ:14`)
- **Finding.** Studio has no execution channel (`MAP 04 §4.4`). The control app's SIM/REAL is the **controller's own** dry run, not a 3D simulation (`R14:1213-1262`; `MAP 09 §7`). v53 has no IP entry (`MAP 08 §4.5`).
- **Borrow:** **no.**
- **Approach.**
  - `RunnerBackend` with `--ip` from Settings.
  - A connection test (`--get-arm-mode`, `--list`).
  - The **three-mode switch** of §4.6.
  - REAL confirmation listing gates and destination (Studio's `showStudioModal` style).
  - "Back to editing" from every failed test.
- **Phases:** P2 (connect, modes) and P5 (runs).

### R5: Collision → graph stops at the joint value and highlights (`REQ:16`)
- **Finding.** Nothing in Studio plots joint values or stops at a collision (`MAP 02 §3.3`). The sweep computes `joint_timeline`, but Studio's server drops it (`TG/server.py:4357`). The real arm reports collisions only as code 4109 at elapsed time X (`R14:56-57`).
- **Borrow:** **partly.** Take detection (sweep), red links (`dacHighlight`, `WE/editor.js:14531`), Prev/Next (`:14548`) and click-to-scrub (`:14478-14513`).
- **Approach.**
  - Joint plots for both arms.
  - Playback stops at the first colliding sample.
  - Cursor pinned; red markers on the 7 joint traces; a translucent band over the colliding run; a sentence naming the parts.
  - On the real arm, map to the nearest track row by joint-space distance (needs D3), else by time minus transit, with an uncertainty band.
- **Phases:** P4 (sim) and P5 (real).

### R6: Adjustable plots and windows (`REQ:18`)
- **Finding.** HTML plot heights are fixed (`HTML` CSS `.plot { height: 600px }`, `MAP 07 §6`). Studio plots are not resizable either; only docks and dialogs are (`MAP 01 §2.6`).
- **Borrow:** **partly.** Take the dock separator (`WE/editor.js:7580-7645`), the dialog grip (`:8470-8503`) and the maximize/restore pattern.
- **Approach.**
  - Pane splitters, keyboard operable.
  - Plot height handles; maximize/restore any plot; **linked x-zoom** across stacked plots; double-click reset.
  - `ResizeObserver` → `Plotly.Plots.resize`.
  - Layout persisted per screen in `localStorage` (UI preferences only, inside try/catch).
- **Phase:** P3, reused in P4.

### R7: Pole height and hand position per region before teaching (`REQ:20`)
- **Finding.** Studio **never** commands pole or hand. It has per-task pole stages and a derived, read-only hand state (`MAP 03 §3.7`), so the requirement's note "borrow from alix studio" does not hold. The apply machinery is in the control app (`R14:804-909`; GRIP preset `0,0,0,0,150,1000`) and the ALIX taught lane ("pole, then hand"; `MAP 03 §3.8`).
- **Borrow:** **no.** Only the vocabulary, the 3D preview and the stowed-bin foul-band rule (`ALIX/ros/orchestration/alix_orchestrator/alix_orchestrator/core/task_config.py:85-125`; band 0.153–0.282 m) carry over.
- **Approach.**
  - Per-unit preset (pole mm, named hand preset + 6 values).
  - The **Prepare** step applies them in the safe order from `UC/usage.py:128-138`: hand open → arm still (`UC/usage.py:44`) → pole → hand preset.
  - A foul-band warning appears if the bin is stowed.
  - Defaults come from V6 and the ALIX taught sheet.
- **Phase:** P2.

### R8: TXT merging with the collision scene (`REQ:22`)
- **Finding.** No taught-file merger exists. V6 is output only: min-jerk bridges that are **not collision-checked** (`V6/README.txt:48-56`), controller transits of up to 64.6° that are unchecked, and program IDs 71–83 inside ALIX's reserved 70–100 (`MAP 10 §3`).
- **Borrow:** **no.**
- **Approach.**
  - Group units that share pole, hand and fixture state into one program.
  - Join with `concatenate` (≤ 0.5°, `ALIX/tools/plan_min_time.py:142`).
  - Velocity-matched, rate-limited bridges.
  - **Collision-check every bridge and every controller transit** (2° lerp) at the right lid/seat state against the commode, walls, the partner arm and the bin.
  - Pole/hand/state changes become workflow steps.
  - Export merged `.txt` + v53-compatible workflow JSON + `timeline.json`.
- **Phase:** P6.

### R9: Offset via FK → offset → IK, no jumps, joint limits (`REQ:24`)
- **Finding.** No such tool exists. All existing IKs (DLS, closed-form SRS, vendor) produced jumps above 10° or failures on real tracks with a 10 mm offset, because the tracks cross the q2/q4/q6 singular sets (`MAP 05 §2.5`). A prototype "differential IK with taught-increment feed-forward + null-space pull" gave 0 failures and a 1.41° maximum step on L_P01P02 (+10 mm), but not across a straight-elbow passage on R2_LIDTOP.
- **Borrow:** **no.**
- **Approach.**
  - Offset & Verify (`MAP 05 §8.2`): raised-cosine ramps at the edges, adaptive damping, limits minus 5°.
  - Gates: step ≤ max(1.5 × taught, 0.5°) and ≤ 10°; rate; residual ≤ 1 mm; deviation; singular-margin warning.
  - **Taper and report** "max achievable here" instead of accepting jumps.
  - Engineer-only at first.
- **Phase:** P7.

### R10: Dual-arm sim left, plots right; correct URDF and meshes; bin toggle (`REQ:26`)
- **Finding.** The committed flat URDF `WS/urdf/alix.urdf` is current (0 differing joints or meshes against a fresh expansion, `MAP 11 §2.1`). The baked visual overlay is **stale**: it has the old tall bin and no camera (`MAP 11 §2.4`). No URDF variant removes the bin (`MAP 11 §1.1`). No split layout exists (`MAP 02 §4`).
- **Borrow:** **partly.** Take the URDF, meshes, overlay code and DAC.
- **Approach.**
  - Re-bake the overlay at build time (`ALIX/tools/meshes/bake_visual_overlay.py`).
  - New split layout.
  - **Runtime bin selector**: Lowered / Extended / Stowed / Not fitted. It hides or shows the meshes, includes or excludes the collision body, re-poses through the SRDF `BIN_*` states and warns about the foul band.
  - Dual timeline with drift bands, using the player clock and `pole.py` durations (`MAP 10 §3.4`).
- **Phases:** P4 and P6.

### R11: Smoothness adjustment with immediate validation (`REQ:28`)
- **Finding.** A sweep takes 12–15 s per pair and Studio rebuilds models on every call (`MAP 02 §1.6`). Byte gates take about 0.25 s per file (`MAP 06 §8`).
- **Borrow:** **partly** (the stale-on-edit pattern).
- **Approach.** While a slider moves, the JS pipeline updates plots and gate chips instantly (150 ms debounce) and 3D replays the edited window. In the background, the warm worker re-checks **only the edited window** for collisions. The "3D OK" status goes stale and then returns to pass or fail.
- **Phases:** P3 and P4.

### R12: Plot spike analysis and suggestions (`REQ:30`)
- **Finding.** The HTML has only per-chunk aggregates (`HTML:944-1050`). ALIX has proven gates and remedies (`MAP 06 §4`). Shipped files have isolated spikes and over-rating ticks; R10_HNGR is 1.53° from the J4 limit (`MAP 10 §5`).
- **Borrow:** **no.**
- **Approach.**
  - **High spikes**: rate over rating at 5.0 ms (G2), step > 10° (G3, refuse), isolated single-tick spikes (> 0.5° and > 3× both neighbours), one-tick speed steps at boundaries, endpoint snaps, accel flag > 900°/s².
  - **Low spikes**: sudden stalls and hesitations inside motion (slowness census `ALIX/tools/taught_tracks.py:954-991`) and near-limit approaches.
  - Each finding carries a plain sentence, its location on the plot and a one-click undoable fix: slow this stretch, de-jitter 5 Hz, ramp the boundary, keep the edge dwell, slow the approach, or re-teach.
  - The meaning of "low spikes" is to be confirmed (decision D14).
- **Phase:** P3.

### R13: Faster trajectory upload (`REQ:32`)
- **Finding.** Throughput is fixed by the controller at about 45 kB/s (2 KB acked chunks), about 1.25 ms per point (`MAP 06 §1.4`). "Fast" in r14 means fast program switching, not upload (`MAP 09 §3.3`). The batch upload in v53 costs 2N+1 processes and 3N+1 list reads (`MAP 09 §3.3`).
- **Borrow:** **no.**
- **Approach.**
  - **Fewer points.** Optimised files are often 20–70 % shorter.
  - **Skip unchanged.** The ledger plus read-back of size and name decides "already there" without sending.
  - **Parallel LEFT/RIGHT** (two controllers).
  - **No pre-list processes**, and never upload to a moving controller (push gaps of 400–600 ms during uploads, `MAP 06 §1.5`).
  - Honest ETA.
  - P9 adds one session per arm. CANFD streaming (no upload) stays out of v1 (decision D15).
- **Phases:** P5 and P6.

### R14: Offset calculation and testing in simulation first (`REQ:34`)
- **Approach.** R9 output → ghost before/after → the same P4 check (sweep, stop-at-collision) → only then is "Test on robot" enabled for that version.
- **Borrow:** **no** beyond the R3 parts.
- **Phase:** P7.

### X1 / X2 exclusions
No Workbench/Generate, region painting, coverage, reachability, manipulability, redundancy atlas, path-ik, path-compare, auto-path, Workspace or Gizmo tabs. That is 38 of Studio's 64 routes (`MAP 04 App. A`). SRS and swivel math may be used *internally* by the offset repair; no C-space view is shown. The glove-pad trail is FK visualisation of taught joints, not path generation.

### NFR-UX
Apply §3. Concretely:
- 44 px primary targets (`--target-primary`, `WE/editor.css:127-128`); usable at 1366×768; large-text option.
- Plain-language errors with a fix hint (`api()` hint convention, `WE/editor.js:9088-9104`).
- Empty states that say what to do; one confirmation only for physical or irreversible actions.
- Undo for edits; audio cue at the end of the teach timer; no jargon on operator screens.
- Usability test with two non-technical operators at P1, P3 and P5.

### NFR-THEME
- Copy the tokens verbatim (`WE/editor.css:1-189`, `ALIX/design/_tokens.css`) with a parity test.
- Brand `#2F96EE`, light default and optional dark, Inter / JetBrains Mono (vendored; Studio does not load them, `MAP 01 §1.7`), 4 px radius, ISA-101 state colours.
- Replace the HTML palette (`--accent #175cd3`, dark header) and the Tk neon palette.
- Build a `plotTheme()` from tokens. Studio's Plotly text colour `#e6edf3` is unreadable on light panels, 1.18:1 (`MAP 01 §5`).
- Primary buttons: white on brand is 3.13:1, so use bold ≥ 18.7 px labels (AA large text) and keep the tokens byte-equal.

---

## 6. Phased delivery plan

Each phase ends with something operators or engineers can use. Estimates are for one experienced developer including tests (S ≤ 3, M 4–8, L 9–15, XL > 15).

| Phase | Goal | Reqs | pd | Size |
|---|---|---|---|---|
| P0 | Installable, offline, fast-starting shell with the ALIX look | NFR-THEME, NFR-UX, deployment | 7 | M |
| P1 | Projects, PPT region catalogue, Regions tab, Dashboard | R1, R7 (data), NFR-UX | 8 | M |
| P2 | Robot connection (IP, modes, STOP) + guided teaching with presets | R1, R4, R7, INT-CTRL | 11 | L |
| P3 | Optimise tab: ported HTML + ALIX algorithms, undo ×5, spikes, resizable plots | R2, R6, R11, R12, INT-TRAJ | 13 | L |
| P4 | 3D Check: sim worker, full collision world, dual-arm split view, stop-at-collision, bin selector | R3, R5, R10, R11, R14 | 15 | L |
| P5 | Robot test loop: upload/run, live mirror, real collision mapping, verdicts, engineer console | R4, R5, R13, INT-CTRL | 9 | L |
| P6 | Program builder: merge, checked bridges and transits, dual timeline, batch upload | R8, R10, R13, INT-CTRL | 13 | L |
| P7 | Adjust position: Offset & Verify + simulate-before-real | R9, R14 | 13 | L |
| P8 | Field trial, hardening, guides, update channel | all, NFR-UX | 9 | L |
| **Total** | | | **98** | |
| P9 (optional) | Native robot service `lite_rm`: persistent sessions, stop token, joint push, single-session batch, Windows robot I/O | R4, R5, R13 | 12 | L |

### P0 — Foundation: shell, theme, packaging skeleton (7 pd)
- **Deliverables:**
  - `alix_studio_lite` package skeleton.
  - Launcher: single instance, app window, fallback browser.
  - Copied server scaffold with needs table, job registry, token/Origin check, and page assembly.
  - `lite_tokens.css` / `lite_base.css` / `lite_shell.css` / `lite_shell.js` (lift list in `MAP 01 §8.3`, with the noted fixes).
  - Vendored three r128, OrbitControls, Plotly 2.35.2 and fonts.
  - Settings and About.
  - `tools/build_bundle.py` (python-build-standalone + wheelhouse), `install.sh` / `uninstall.sh`, desktop entry, update-from-file + rollback.
  - Copied guard tests: token parity, no remote assets, JS integrity.
- **Exit criteria:**
  - On a clean Ubuntu 24.04 VM with the network unplugged, install by unpacking and double-clicking, and get the UI interactive in ≤ 3 s.
  - No network request is made (test).
  - Token parity passes.
  - Update and rollback demonstrated.

### P1 — Projects, region catalogue, Regions tab, Dashboard (8 pd)
- **Deliverables:**
  - Project store with atomic writes and schema versions.
  - `catalogue.yaml` with 44 units, IDs and aliases (`MAP 13 §5.2`), phase order, pairing, fixture states, and default pole/hand/time (`MAP 13 §7.4`).
  - PPT renders + overlay/hotspot boxes produced by `tools/build_assets.py` (LibreOffice render on the dev box only).
  - Regions tab: overview map, step list, unit card.
  - Dashboard matrix with chips, counters and "Teach next missing".
  - Raw-file import wizard (side and unit assignment + provenance).
  - V6 legacy import split at the known segment rows (`MAP 13 §6.2`), shown as "legacy", not raw.
- **Exit criteria:**
  - A seeded project shows correct statuses.
  - V6 import marks the covered, merged, partial and other-arm units as in `MAP 13 §6.3`.
  - Two non-technical testers find and open the next missing unit without help.

### P2 — Robot connection and guided teaching (11 pd)
- **Deliverables:**
  - `RobotBackend` + `RunnerBackend` (vendored R14 + hand/pole, sha-pinned, `--ip`, `--sdk`, `--yes`) + a token → event parser tested with recorded transcripts.
  - Connection card (IP validation, test, persist); arm chips; controller mode with read-back; REAL confirmation; restored "controller in SIMULATION" warning.
  - STOP ALL / per-arm stop (SIGINT to the process group).
  - Guided teach: fixture-state check, foul-band check, Prepare (pole → hand in the safe order with an arm-still check), START drag, countdown/extend/beep, STOP & SAVE (`--drag-save` with retry), named file + `.source.json`, review, Redo/Accept/Skip, auto-advance with interstitials.
- **Exit criteria:**
  - On the robot, an operator teaches 3 consecutive units end-to-end without a terminal.
  - STOP ALL works during a drag session and during a pole move.
  - A changed IP is used by every command and shown everywhere.
  - A modified runner file disables robot features with a clear message.

### P3 — Optimise tab (13 pd)
- **Deliverables:**
  - `traj_core.js` + Python `traj/pipeline.py` with a parity test (≤ 1 mdeg).
  - ALIX anchor pause removal and 5 Hz Butterworth as "Recommended", HTML algorithms as "Legacy".
  - Endpoint-snap fix; per-segment trim/smooth/speed with λ ramps; rating clamp (0.97 × rating at 5.0 ms).
  - Recipe + persistent, labelled undo ring of 5; presets Gentle/Standard/Fast.
  - Seconds/degrees axes; `plotTheme()`; splitters, height handles, maximize, linked zoom.
  - Findings panel with one-click fixes; duration band; versioned OPT files with reports.
- **Exit criteria:**
  - `zip/02` is reproduced from `zip/01` (legacy mode, ≤ 1 mdeg).
  - The gates flag all 10 over-rating V6 files and the 1.53° J4 margin of R10_HNGR (`MAP 10 §1.3`).
  - Undo/redo round-trips 5 steps exactly, also after a restart.
  - A tester makes a gate-clean optimised file in < 3 min.

### P4 — 3D Check (15 pd)
- **Deliverables:**
  - Asset bundle loader (prebuilt `PackageIndex`, `ALIX_SRDF`, home pose); re-baked overlay.
  - Sim worker with warm models.
  - `.txt` adapter with pole/hand(`hand_units`)/bin seeding.
  - World: arm vs arm, self, parameterised body/bin, commode by state, walls by venue.
  - Sweep: coarse, then refined; `joint_timeline` kept; verdict sentences; hash cache.
  - Client FK + parity test.
  - Check screen: split view, transport speeds, stop-at-first-problem, red links/markers/band, Prev/Next, Back to editing, bin selector, glove-pad trail.
- **Exit criteria:**
  - Synthetic collision cases (arm vs lid, arm vs arm, arm vs bin when stowed) are detected, and the plot stops on the right sample (±1).
  - A V6 L_P01P02 + R_P01P03 pair checks in ≤ 30 s cold and ≤ 2 s cached.
  - FK parity ≤ 0.01 mm on 10k poses.
  - The visual bin matches the collision bin.

### P5 — Robot test loop (9 pd)
- **Deliverables:**
  - Upload with slot-range policy, ledger skip and parallel arms.
  - Run with default transit 30 % (the runner restores global speed itself, `R14:255-262`).
  - Pre-run gate chips; supervisor override with a recorded reason.
  - Live mirror: preview always; real if D3 is approved, else coarse.
  - Error codes in plain language (4105/4109/4114/4115).
  - Stop or collision → row → plot stop + Back to editing; operator verdicts.
  - `PreviewBackend` runs.
  - Engineer robot console.
- **Exit criteria:**
  - Practice-mode and real-mode runs complete with verdicts on the robot.
  - Re-uploading an unchanged file is skipped in ≤ 5 s.
  - An injected 4109 (emulator backend or controller SIM) stops the plot at the expected row.
  - Stop mid-run is verified.

### P6 — Program builder and merge (13 pd)
- **Deliverables:**
  - Program model; automatic grouping; `concatenate`; velocity-matched, rate-limited bridges.
  - Bridge and transit collision checks at the right fixture state.
  - Ported v53 planner with an explicit checkpoint; timeline view.
  - Export of v53-compatible workflow JSON + `timeline.json`.
  - Whole-program 3D check; Upload all (parallel, skip-unchanged, ETA).
- **Exit criteria:**
  - A V6-equivalent program is rebuilt from units, with every bridge and transit carrying a verdict (V6's 64.6° and 45.7° transits are flagged).
  - The exported JSON passes the ported planner and loads in v53.
  - A second "Upload all" with no changes finishes in seconds.

### P7 — Adjust position, offset IK (13 pd)
- **Deliverables:**
  - Offset & Verify (`MAP 05 §8.2`) with gates, taper and report.
  - UI with direction words; ghost before/after; plots; hand-off to P4 check and P5 test.
- **Exit criteria:**
  - L_P01P02 +10 mm y: 0 failures, maximum adjacent step ≤ taught maximum.
  - R2_LIDTOP +10 mm z: no step > 10°; singular spans tapered and reported.
  - Property test: output never leaves the URDF limits minus 5°.

### P8 — Field trial and hardening (9 pd)
- **Deliverables:**
  - Two-week operator trial and fixes.
  - Crash/restart recovery: re-attach to jobs, keep recipes.
  - Fault-injection tests: kill the server mid-save, unplug the network mid-upload.
  - Translated messages; support bundle; one-page operator guide + in-app help; engineer guide.
  - Release checklist; optional Windows offline build.
- **Exit criteria:**
  - Operators complete the full loop for ≥ 6 units in one session without engineer help.
  - No data loss under fault injection.
  - An operator performs update and rollback from a USB file.

---

## 7. Risks and mitigations

| Risk | Mitigation |
|---|---|
| Unchecked motion on real runs: controller transits to a program's first point, bridges, the persistent speed ratio (`MAP 06 §1.6`) | Gate real runs on a 3D check of the exact bytes, including transits from the expected start pose. Default transit 30 %; the runner restores the global speed. Practice mode first; STOP ALL always visible |
| 3D verdicts not matching the real cell: the commode pose during teaching is unknown; the default venue gave false wall hits (`MAP 05 §4.6`); BinShort vs the physical bin (`MAP 11 §1.5`); fingers excluded from the slim set | Settings for commode standoff and venue width, with a calibration check against one known run. Label results "check, not proof". Optional wider link set for final checks. Decisions D6/D7 |
| Offset IK cannot be exact through singular passages (`MAP 05 §2.5`) | Detect, taper and report instead of jumping. Engineer-only. Hard gates (≤ 10°/line, limits − 5°) |
| Stdout-token wrapping of the runner is brittle | Pinned runner sha, transcript-replay tests, events not labels, optional P9 native backend |
| Stop relies on SIGINT; Windows `CTRL_BREAK` would not reach `KeyboardInterrupt` (`MAP 09 §8`) | Robot I/O on Linux only in v1. Windows robot I/O only with P9 (stop token) |
| No joint values on real-arm collisions | Additive `JOINTS:` print patch (D3). Otherwise a time-based estimate with a shown uncertainty band |
| Offline packaging of python-fcl / trimesh / scipy; ABI clashes (`MAP 05 §7`) | Embedded interpreter + prebuilt wheels; self-test at install; CI smoke test on a clean VM per release |
| RealMan SDK licence unknown (`ALIX/DEPENDENCIES.md:63-70`) | D2. If redistribution is not allowed, the bundle asks for the SDK folder once (Settings) and validates it |
| Slot collisions with ALIX production programs 70–100 and V6 71–83 (`MAP 10 §5`) | A declared Lite slot range per arm (D4); the ledger refuses writes outside it; replace confirmation shows what is in the slot |
| ROS driver or another app commanding the same arm (`MAP 06 §7.3` item 9) | Connection test warns if the controller is busy or not idle; per-arm lock inside Lite; the operator guide says "close the old console" |
| Operators confuse SIM / practice / real | Three named modes with colour + word; REAL banner; practice is the default after connecting; REAL resets to practice after idle |
| 3D slowness on long tracks (10k points) or modest laptops | Client FK; scattergl and downsampled overview plots; windowed re-checks; hash cache; worker warm-up off the UI path |
| Theme drift from Studio | Tokens copied byte-equal with a parity test against `ALIX/design/_tokens.css` |
| Scope creep: tool, glove and brush tasks, mechanisms (`MAP 13 §6.3`) | v1 covers surface units + seat/lid mechanisms taught as tracks. Tool/glove units show "not in v1"; the catalogue keeps them for later |
| Data loss or corruption | Immutable raw files with sha256; atomic writes; backups before migration; undo ring; data outside the install folder |
| Dual-arm start skew and drift unmeasured (`MAP 08 §5`; `MAP 10 §3.4`) | Measure in P6 (log both start times); show drift bands; keep TOGETHER blocks short; use explicit checkpoints |

---

## 8. Open decisions (for the requester and the team)

- **D1. Operator PC and browser.** Is Ubuntu the only v1 target (recommended)? Is Chrome/Chromium/Edge available for app mode, or should a webview be bundled?
- **D2. SDK redistribution.** May the RealMan SDK (`Robotic_Arm` + `libapi_c.so`) be redistributed in the bundle? The licence is unknown.
- **D3. Runner patch.** May Lite ship a reviewed **additive** v14 patch that prints joint snapshots (needed for exact R5 on the real arm), or must v14 stay byte-for-byte?
- **D4. Slot ranges.** Which slot range does Lite own on each controller? Options seen: 40–59 per README, V6 71–83, ALIX reserved 70–100, UI defaults 25/99.
- **D5. Meaning of "SIM".** Confirm the three operator modes: Simulation (no robot), Robot practice (controller SIM, arms still), Robot real.
- **D6. Bin.** Which bin state applies during cleaning? Does "bin toggle" mean exclude, stowed, or a 3-state choice? Has the physical bin been shortened (BinShort)?
- **D7. Cell calibration.** What commode placement and venue (wall width) apply in the Lite cell? These are needed for meaningful collision verdicts.
- **D8. Duration display.** Show durations as a band at the player clock (5.5545 ms nominal) instead of the 5 ms recording clock?
- **D9. Default algorithms.** Make the ALIX anchored pause removal + 5 Hz Butterworth the defaults, with the HTML algorithms kept as "Legacy"?
- **D10. Hard gate.** Is "3D check passed" a hard gate for real runs, with a supervisor override recorded with a reason?
- **D11. Upstream change.** Upstream small `alix_workspace` parameters (`body_links`, `bin_state`, in-memory plan input), or keep a Lite-side wrapper over private helpers?
- **D12. Tk fallback.** Does the v53 Tk app stay installed as an engineer fallback, and must Lite keep exporting v53-compatible workflow JSON?
- **D13. Catalogue ownership.** Who owns the region catalogue (IDs, cleaning times, presets)? Adopt the normalised IDs in `MAP 13 §5.2`?
- **D14. "Low spikes".** What do "low spikes" in R12 mean: stalls/hesitations, or negative peaks only?
- **D15. CANFD.** Keep CANFD streaming (no upload) out of v1?
- **D16. Offset audience.** Engineer-only offset tool in v1 (recommended)?

---

## Appendix A. Lift list (copy/port with source ranges)

| Lite file | Source | Notes |
|---|---|---|
| `server/http.py` | `TG/server.py:5178, 6320-6469` | Drop the audit hook, generation lock and Studio routes |
| `server/routes.py`, `jobs.py` | `TG/server.py:5961-6253`; `TG/studio_generate.py:537-600, 861-1060` | Needs: `robot_io:<arm>`, `sim`, `project_write` |
| `server/page.py` | `TG/scripts/visualize_generated_config.py:530-568`; `TG/server.py:6297-6317` | Own template |
| `web/lite_tokens.css` | `WE/editor.css:1-71, 94-129, 134-166, 184-189, 1714-1724` | Drop atlas tokens; parity test against `ALIX/design/_tokens.css` |
| `web/lite_shell.js` | `WE/editor.js:5431-5467, 9088-9104, 13786-13832, 7580-7645, 8389-8523, 5322-5337, 8749-8766, 19150-19166` | Wrap `localStorage` in try/catch; namespace `alixLite.*` |
| `web/viewer.js` | `WE/editor.js:309-645, 667-703, 12352-12437` | Plus client FK port of `WS/urdf_kinematics.py:294-343` |
| `web/dac.js` | `WE/editor.js:14408-14598` | Retheme; stop-at-collision; joint markers |
| `web/board` | `WE/editor.js:14708-14757, 15208-15289`; `WE/editor.css:1507-1528, 1976-1998` | New data source |
| `web/traj_core.js` | `HTML:1417-1620` | Fix R-A1/R-A5 |
| `robot/vendor/runner_v14/` | `R14`, `UC/hand.py`, `UC/pole.py` | Unchanged, sha-pinned |
| `robot/planner.py` | `V53:9619-9997` (pure methods) | Explicit checkpoint step |
| `traj/pipeline.py` etc. | `ALIX/tools/taught_tracks.py:607-716, 748-832, 894-951, 954-991` | Golden tests |
| `sim/adapter.py` | `ALIX/tools/dual_arm_verdicts.py:531-731`; `ALIX/ros/hw/alix_hw/scripts/hand_units.py:33-60` | Seeding is mandatory |
| `sim/world.py` | `TG/body_collision.py:104-108, 401-464`; `WS/plan_scene_validation.py:117-155` | Copied: these modules need a repo root at import |
| `sim/merge.py` | `ALIX/tools/plan_min_time.py:142-171` | Bridges new |
| Assets | `WS/urdf/alix.urdf`; collision meshes; re-baked npz; `ALIX/ros/manipulation/alix_moveit_config/config/alix.srdf`; commode_c scene + models; venue YAML + wall STL | `tools/build_assets.py` with a manifest of hashes |

## Appendix B. Test strategy (built up per phase)

- **Unit tests:**
  - Parsers and writers (CRLF/LF, strict 7 ints).
  - Pipeline golden files (JS ↔ Python ↔ ALIX functions).
  - Gates against known V6 statistics (`MAP 10 §1.3`).
  - Planner on the V6 FINAL JSONs. Expected plan: TIMELINE-1 → LID-PREP-1 → LID-CHECK-1 → TIMELINE-2 → serial 78 → serial 79 → TIMELINE-3 (`MAP 08 §1.8`).
- **Integration tests:**
  - Runner-token parser on recorded transcripts.
  - `EmulatorBackend` end-to-end (upload / list / run / stop) on Linux.
  - Sim worker sweeps on synthetic collision cases.
  - Client/server FK parity.
- **UI guards** (copied from Studio): token parity, no remote assets, JS integrity / quickjs execution, every bound id exists.
- **Hardware acceptance** per phase exit, with a written checklist. Practice mode first, real mode second.

## Appendix C. Glossary for operators (used on screen)

- **Region / unit:** one arm cleaning one area (for example "LEFT arm, seat top, left half").
- **Teach:** move the arm by hand while it records.
- **Optimise:** make the recording faster and smoother.
- **3D check:** watch it on the screen with the toilet and the other arm.
- **Practice:** the robot runs the program with the arms standing still.
- **Real:** the arms move.
- **Program:** what the robot keeps and runs later.
