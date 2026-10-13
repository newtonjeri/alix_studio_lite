# ALIX Studio Lite: architecture proposal (safety- and maintainability-first lens)

Audience: the engineers who will build Lite, and the author of the requirements.
Status: proposal. Nothing in either repository was changed. Evidence comes from the 13 source maps in
`scratchpad/maps/` plus spot re-checks listed in §0.3.

## Path legend used in every citation

| Alias | Absolute path |
|---|---|
| `ALIX/` | `/home/newtonjeri/alix_ws/src/alix/` (monorepo, HEAD `842c0e59`, 2026-10-12) |
| `TG/` | `ALIX/core/python/alix_taskgen/src/alix_taskgen/` (ALIX Studio package) |
| `WE/` | `TG/web_editor/` (`template.html`, `editor.js`, `editor.css`, `vendor/`) |
| `WS/` | `ALIX/core/python/alix_workspace/src/alix_workspace/` |
| `DP/` | `ALIX/core/python/alix_dispatch/src/alix_dispatch/` |
| `EMU` | `ALIX/core/python/src/alix_emulator/rm_emulator.py` |
| `TT` | `ALIX/tools/taught_tracks.py` |
| `UC/` | `/home/newtonjeri/alix_studio_lite_ws/upload_channel/` |
| `v53` / `r14` | `UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py` / `UC/rm_upload_fast_v14_safe_transit.py` |
| `H` | `/home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html` |
| `V6/` | `/home/newtonjeri/alix_studio_lite_ws/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/` |
| `REQ` | `/home/newtonjeri/alix_studio_lite_ws/alix_studio_lite/docs/reference/ALIX_Studio_Lite_requirement_inputs.txt` |
| `MAP nn` | `scratchpad/maps/nn_*.md` (detailed evidence behind each claim) |

---

## 0. Summary

### 0.1 The proposal in one paragraph

Build Lite as a **standalone, loopback-only Python application** (`alix_studio_lite`) with four kinds of
process: a web/API process, one collision-and-kinematics **simulation worker**, and one long-lived **robot agent
per arm** that holds that arm's only controller connection. The single-page front end reuses ALIX Studio's
design tokens, shell components, 3-D robot viewport and collision-playback code, plus the trajectory HTML's plots
and stepper, all restyled to the Studio theme. **Everything that decides whether a robot may move is imported from one
source, or copied byte for byte with a hash check.** That covers joint limits, gates, the collision world, upload
and run semantics, and the validation ladder. Only presentation code is copied freely. Every motion goes up one
ladder: **OFFLINE 3-D check, then controller dry run, then REAL**. Each rung is keyed to the exact bytes and the
exact context (pole, hand, bin, fixture state, partner arm). Any edit makes the downstream rungs stale.

### 0.2 Headline decisions

1. **Not a mode of ALIX Studio.** Studio has no execution channel, and a test pins that
   (`ALIX/core/python/alix_taskgen/test/test_studio_interaction_guards.py:2205`). It also imports the generator stack
   at start-up (30–60 s), resolves everything relative to the repo, and has about 140 structural UI guards that
   contradict Lite's guided flow (MAP 04 §7.1). Lite takes the Studio's *parts*, not its shell.
2. **Import, never copy:** `alix_workspace` (FK, IK, collision, plan frames, SRDF states, wall checker), the commode
   world from `alix_taskgen.body_collision` / `scene_pose` (only those whitelisted modules), contact policy
   `contact_links.yaml`, `alix_dispatch` (upload, run, stop, run mode, ladder, emulated program channel), the
   emulator, and `hand_units.py`.
3. **One asset bundle, hash-verified.** This is a sparse, byte-identical extract of the ALIX repo at a pinned
   commit, laid out exactly like the repo, so the unmodified libraries resolve URDF, meshes, SRDF, commode, walls and
   `hand_units.py` the way they do in ALIX. A start-up **world self-test fails closed** when anything resolves
   to "missing". These libraries otherwise degrade *silently* (MAP 04 §2.3; `WS/package_index.py:115-121`;
   `TG/body_collision.py:432-435`).
4. **One canonical trajectory processor (Python).** It produces every byte that can reach a controller. The
   trajectory HTML's JavaScript survives as an instant *preview* that is parity-tested against the Python output.
5. **Controller semantics come from `alix_dispatch`, the most hardware-evidenced library.** Runner v14 stays,
   unchanged and hash-pinned, only as a transitional executor and as the parity oracle for the bytes it sends.
   Drag-teach verbs, which `alix_dispatch` lacks, are ported from v14.
6. **Fail closed everywhere:** unknown run mode, unverified arm identity, a missing collision object, a stale
   certificate, a version mismatch or an unrestored global speed all refuse the action, with a plain-language reason.

### 0.3 Facts re-checked for this proposal (beyond the maps)

- `compute_dual_arm_plan_collisions(..., include_body, extra_managers, left/right_hand_state)` and
  `compute_plan_frames(...)` signatures: `WS/dual_arm_plan_collision.py:586-600, 487-495`. The body set is hard-coded:
  `kBodyCheckLinks = ("bin_link", "glove_wearing_unit_arm")` (`:644`). The sweep builds `RobotCollisionModel.from_file`
  **without** an index on every call (`:629`), so mesh discovery depends on the file layout around the URDF.
- `BodyCollisionWorld.fixture_manager` returns an **empty** manager when the commode `scene.yaml` is not found
  ("no scene -> no fixture objects", `TG/body_collision.py:432-435`). A missing asset therefore becomes a false
  "clear". This is the main reason for the world self-test.
- `alix_dispatch` has no `pyproject.toml` (`ALIX/LAYERS.md:25`), and its modules import only the standard library at
  load time (`DP/track.py:31-43`, `DP/connect.py:18-23`, `DP/ladder.py:28-34`). It has **no drag-teach verbs** (grep).
- `alix_dispatch.ladder` already implements "a SIM pass licenses REAL only for the identical commanded motion",
  with an append-only ledger and recorded overrides (`DP/ladder.py:1-26, 63-145`).
- `contact_links.yaml` (`ALIX/ros/orchestration/alix_tasks/config/contact_links.yaml:1-40`) is the hardware-calibrated
  list of links that may touch porcelain for each glove frame. Lite needs it to tell *intended contact* from
  *collision*.
- v53 declares `UI_PROGRAM_VERSION="v53"` and `EXPECTED_RUNNER_VERSION="v14"` (`v53:45-46`). r14 supports `--ip`
  (`r14:2285`). `_normalize_*_for_main` and `_step_preview` have no `self` and no `@staticmethod`
  (`v53:4853, 4876, 4885, 5000`), called as `self._…` (`v53:4932, 5083, 5159`).

---

## 1. Design rules for this lens

| # | Rule | Consequence in Lite |
|---|---|---|
| S1 | **One source of truth for anything that gates motion.** | URDF, SRDF, meshes, commode and walls, joint limits, ratings, clocks, gates, contact policy and upload format are imported from ALIX or copied byte-identical with a hash manifest. No hand-retyped constants. |
| S2 | **Copies are allowed only for presentation, or behind a parity test.** | UI code is copied freely. Any ported algorithm (HTML processor, taught-lane gates, v53 planner, `.txt`→plan shim) carries a golden test against the pinned original. |
| S3 | **Silence is a refusal.** | Missing meshes, an empty fixture manager, an unknown SRDF or run mode, or an unreadable state: the action is refused with a reason. It never proceeds on defaults. |
| S4 | **Exact-bytes ladder.** | OFFLINE → (EMU, developer only) → CONTROLLER-SIM → REAL, keyed by `sha256(.proj bytes)` plus `sha256(context)`. Editing anything makes downstream rungs stale ("stale is not pass", from `TG/schedule_guards.py:1-37`). |
| S5 | **One command source per arm.** | Exactly one agent process holds each controller connection. Screens and jobs reserve an arm through a lock. STOP bypasses the lock. |
| S6 | **Provenance for every artefact.** | Raw recording → `SOURCE.json`. Derived file → recipe + certificate. Robot action → audit event. Run → run directory. All are append-only or immutable. |
| S7 | **Operators see verdicts, engineers see parameters.** | Operator mode shows presets, plain words, seconds and degrees. Engineer mode shows raw parameters, slot maps and overrides, and every override needs a written reason. |
| S8 | **Prefer upstream fixes to local forks.** | Each Lite port is listed as an upstream request (§5.3). When ALIX ships the importable version, Lite deletes its copy. |

---

## 2. What the upload-channel programs do (high level) and which version is current

The user asked only for an understanding of these programs, not a code audit. This section describes them; issues
are listed only where they block the plan, one line each.

| Program | What it does | Status |
|---|---|---|
| `v53` `rm_trajectory_ui_v53_safe_lid_checkpoint.py` (Tkinter, 11,581 lines) | Operator console with two pages. **Page 01** is a per-arm console: upload/run/list stored programs, drag-teach record/save/playback, hand (6 channels plus GRIP/ALL OPEN presets), pole 0–315 mm, controller SIM/REAL/CHECK, global points, telemetry. **Page 02** is a dual-sequence builder: per-arm lists of program/pole/hand/wait/global steps, dual rules TOGETHER / RUN FIRST / WAIT OTHER, the "SAFE TIMELINE" planner and a manual lid checkpoint. Every action spawns one runner subprocess per arm and reads its stdout markers. Stop sends SIGINT. Dual starts use a `threading.Barrier`. | **Current UI.** It runs the V6 production package (MAP 08 §0, §1.8). |
| `r14` `rm_upload_fast_v14_safe_transit.py` (2,702 lines) | Command-line tool over the RealMan SDK, one TCP connection per call. **Upload**: `.txt` → `.proj` (CRLF, store-only, verified by size read-back). **Run** a stored program: pre-run gates, polling, watchdog, slow stop, global-speed restore. **Sequences/workflows** over one connection ("fast" means fast program switching, not faster upload). Also drag teach, pole/hand, run mode with read-back, and global waypoints. Lines 1–268 are ALIX `tools/rm_upload.py`. | **Current runner.** v53 expects it (`v53:46`). |
| `UC/hand.py`, `UC/pole.py` | Inspire hand over end-port Modbus RTU (6 channels, 0–1000, 1000 = open). Lift moves with a measured travel-time model. | Used by r14. |
| `UC/usage.py` | Notes on the safe order (hand open → arm still → pole → hand) and `stop_all`. | Reference. |
| v48 / v51 / v52, runner v13, `py programs/` | History. v48 is fully contained in v53. v48 and v51 reject the V6 package. | Superseded. |
| `UC/*_V6_70SERIES_FINAL.json` | Production per-arm workflows (programs 71–83, pole/hand steps, lid wait). | Data. |

**Plan-blocking issues (one line each):**
1. The UI never passes `--ip`; the IPs it shows are labels only. Lite must send the operator-entered IP (`r14:2285`).
2. The runner version is not enforced. A v13 fallback would run transits at 100 % (MAP 09 §1.1), so Lite pins runner bytes by sha256.
3. Stop is an OS signal. On Windows `CTRL_BREAK` does not raise `KeyboardInterrupt`, so no slow stop happens (MAP 09 §8). This blocks the subprocess path on Windows.
4. Nobody has tested on hardware whether `rm_set_arm_slow_stop` aborts a *running stored program* (MAP 09 §4.4). Lite's STOP promise depends on it.
5. Multi Run and Page-01 Load Sequence raise `TypeError` (missing `@staticmethod`, `v53:4853, 5000`). Lite does not port the Tk UI, so this matters only if someone keeps using v53 in the meantime.
6. Program IDs 71–83 sit inside ALIX's reserved 70–100 range on the same controllers. A slot policy decision is needed.
7. Controller SIM mode does not simulate the pole or hand (MAP 09 §7), so a "dry run" must treat pole and hand steps explicitly.
8. The dual rules run in the Tk UI, not in the runner. The planner has to move into Lite's backend.

---

## 3. Requirement verdicts: can it be borrowed from ALIX Studio, and how Lite gets it

"Studio" means ALIX Studio (`TG/`). The "Phase" column refers to §10.

| Req | Borrow from Studio? | What is borrowed (evidence) | What is built, or taken from elsewhere | Lite home | Phase |
|---|---|---|---|---|---|
| **R1** Region-wise collection | **Partly: UI pattern only.** Studio's Regions tab is a mesh-painting editor for the X1 generator (`WE/template.html:156-201`). It has no wizard, timer or auto-advance (MAP 03 §5). | Board table, chips, tab badge and click-to-act cells (`WE/editor.js:14708-14757, 15208-15467`; `editor.css:1507-1528, 1976-1998`); status/toast/modal toolkit. | Unit registry from the PPT (MAP 13 ID scheme `<ARM>-<KIND><NN>[-P<n>/-S<n>]`). Dashboard backend that scans the project store. Guided **Teach Session** (prepare → countdown → save → check → auto-advance). Drag-teach verbs ported from `r14:912-989`. `SOURCE.json` provenance (ALIX schema, `TT:309-340`). "Import existing recording" so v53 recordings can be assigned now. | Dashboard, Regions, Teach | 1, 4 |
| **R2** Per region/sub-region trim, smooth, scale + undo ×5 | **No.** Only Studio's undo *pattern* (`WE/editor.js:1835-1977`). | Snapshot-undo idea, one entry per gesture (`WE/editor.js:351-383`). | Port the HTML pipeline (`H:1417-1620`) into a **canonical Python processor**, with per-segment trim/smooth/scale. Add ALIX's validated options: the anchor pause rule (`TT:915-951`) and 5 Hz zero-phase Butterworth (`TT:894-912`). Fix the HTML defects: endpoint snap, unramped boundaries, no rating clamp (MAP 07 §9). Undo/redo ring of 5 labelled **recipe** states per unit, persisted. | Optimise | 2 |
| **R3** Simulation playback, back to editing | **Partly: parts yes, assembly no.** Studio's DAC tab takes only plan JSON under repo plan roots, leaves out the commode and walls, defaults the pole to 0.300 m and the hands to open, always shows the bin LOWERED, and has no joint plot (MAP 02 §0). | Viewport and robot overlay (`WE/editor.js:309-645, 667-703, 12352-12437`), `/api/robot-geometry` logic (`TG/server.py:1864-2001`), `compute_plan_frames` (`WS/dual_arm_plan_collision.py:487-583`), DAC playback/highlight (`WE/editor.js:14272-14600`), transport (`:8749-8767`). | `.txt` → plan adapter that seeds pole, hand and bin (logic of `ALIX/tools/dual_arm_verdicts.py:580-660`). World composition: robot + commode at the unit's state + walls + bin toggle. **Contact policy** from `contact_links.yaml`, so wiping is not reported as a collision. Gating sweep at dt 0.02 s. "Back to editing" pre-selects the segment. | Simulate | 3 |
| **R4** Real-arm test, IP entry, real/sim toggle | **No.** Studio has no execution channel (test-pinned, `test_studio_interaction_guards.py:2205`). | Simulate-view idea for a live mirror: `POST /api/sim/joints` was written for "the real arms" (`TG/server.py:5326-5341`). | Per-arm **robot agent** on `DP/connect.open_arm(side, ip)` (`DP/connect.py:141-182`), `DP/runmode` (read-back), `DP/track` upload/run/stop, and `DP/ladder` gating. IP entry plus an arm-identity check. Operator verdict ("OK / missed spot / collision / other") feeding back to editing. | Robot Test, Connection | 4, 5 |
| **R5** Collision → graph stops at the joint value, highlighted | **Partly.** Studio highlights colliding links, lists hits, has Prev/Next and click-to-scrub (`WE/editor.js:14478-14560`). It never stops, and it has **no joint plot anywhere** (MAP 02 §3.3). | DAC highlight/log/scrub; `joint_timeline` from the sweep (`WS/dual_arm_plan_collision.py:668, 755`), which Studio throws away (`TG/server.py:4357`). | Joint-value plots that stop at the first collision row with marker, band and pair label, kept in sync with the 3-D view. For the real arm: error 4109 plus a joint snapshot or UDP frames mapped to the nearest track row (`DP/track.py:2037-2072`). | Simulate, Robot Test | 3, 5 |
| **R6** Resizable plots and editing windows | **Partly.** Studio has resizable docks and dialogs but fixed plot heights (MAP 01 §2.6). The HTML plots are fixed too (`H:125-129`). | Dock separator with keyboard support and persistence (`WE/editor.js:7562-7645`; `editor.css:331-341`), dialog grip (`:8470-8503`), maximise/restore (`:7538-7559`). | Split panes (3-D beside plots; plot above plot), plot height handles, per-plot maximise, pop-out windows, time-linked zoom, `ResizeObserver` → `Plotly.Plots.resize`. | All plot screens | 2, 3 |
| **R7** Per-region pole height and hand before teaching | **No for actuation.** The requirement note "borrow from ALIX Studio" is inaccurate: Studio never commands the pole or hand (`WE/editor.js:13045-13047`; `TG/sim_bridge.py:10-21`). | SRDF pole/hand vocabulary (`/api/joint-presets`, `TG/server.py:1763-1785`), overlay preview, stowed-bin foul-band warning 0.153–0.282 m (`ALIX/ros/orchestration/alix_orchestrator/alix_orchestrator/core/task_config.py:58-124`). | Actuation through `DP/peripherals.Pole` / `HandModbus` (`DP/peripherals.py:169, 462`), with the field-used `UC/pole.py`/`UC/hand.py` as fallback. Order "hand open → arm still → pole → hand" (`UC/usage.py:128-138`). Interlocks: arm at rest, pad off the surface, foul band. Defaults per unit from V6 and the ALIX taught sheet. | Regions card, Teach | 4 |
| **R8** Merge `.txt` files into one executable program, with the commode scene | **No.** No merger exists in ALIX or the workspace, and V6 ships no generator code (MAP 10 §3.1). | Primitives: `plan_min_time.concatenate` (≤ 0.5° boundary, `ALIX/tools/plan_min_time.py:142-171`); sweep with `extra_managers`; dual clearance ≥ 25 mm (`ALIX/tools/dual_arm_flow.py:19-47`). | Merge engine: group by pole/hand/fixture state, check continuity, add velocity-matched bridges limited to the ratings, **collision-check bridges and the controller's implicit transits**, write a segment manifest. Dual workflow v2 with an explicit checkpoint step, plus the dual timeline file V6 never produced. | Combine | 7 |
| **R9** Offset via FK → offset → IK, no jumps, within limits | **No.** Studio's IK UI is X2, and the existing IK engines jump or fail on taught tracks (MAP 05 §2.5). | Library-level: `UrdfModel.fk_batch`, Jacobians, `IKSolver`, `SrsArm`/`srs_ik`, `swivel_arcs` (`WS/urdf_kinematics.py:218-259`; `WS/ik.py:150-334`; `WS/srs.py:296-900`). | New **Offset & Verify** solver: differential IK with taught-increment feed-forward and a null-space pull. Prototype D measured 0 failures and a 1.41° maximum step on L_P01P02 at +10 mm (MAP 05 §2.5). Ramped ends, per-point gates, taper or refuse at singular passages. | Offset | 6 |
| **R10** Dual-arm sim left, plots right; correct URDF and meshes; bin toggle | **Partly.** | URDF `WS/urdf/alix.urdf` is current: 0 differing joints and 0 differing mesh references against a fresh expansion (MAP 11 §2.1). Meshes and the baked overlay come through the same endpoint. | **Re-bake** the overlay first: the bake shows the old tall bin and has no camera (MAP 11 §2.4). New split layout. Runtime bin include/exclude plus STOWED/EXTENDED/LOWERED (no xacro switch exists, MAP 11 §6.3). Dual timeline on the player clock and pole model. | Simulate | 3, 7 |
| **R11** Adjust smoothness, validate immediately in sim | **Partly.** | Stale-on-edit pattern (`WE/editor.js:5371-5430`); content-hash caches (`TG/server.py:4303-4316`). | A warm sim worker. A windowed "preview check" of the edited segment, labelled *preview*. The gate sweep stays authoritative. | Optimise (mini-sim), Simulate | 3 |
| **R12** Spike analysis and suggestions | **No.** | — | Port the ALIX detectors: band-limited rate at 5.0 ms (G2), step ≤ 10° (G3), one-tick speed step, acceleration advisory (MAP 06 §4). Add detectors for the HTML-specific endpoint snap and boundary jumps. Plain-language suggestions with one-click, undoable fixes. | Optimise | 2 |
| **R13** Faster upload | **No.** | — | Throughput is fixed by the controller at about 45 kB/s (MAP 06 §1.4). So: fewer points; skip unchanged slots (name + size + sha pin); one connection per arm; both arms in parallel (to be measured); show predicted upload time; never upload while that controller is moving. | Robot Test | 5 |
| **R14** Calculate, apply and test offsets in sim first | **No.** | — | R9 output is a new derived file that must pass OFFLINE before CONTROLLER-SIM or REAL; before/after overlay playback. | Offset, Simulate | 6 |
| **X1 / X2** exclusions | — | — | Lite never imports `alix_taskgen.server` or the generator, never exposes reachability, atlas or C-space, and never edits Cartesian paths. An **import-boundary test** enforces this. IK/FK and swivel maths are used internally only (R9). | — | 0 |
| **NFR-UX** | Partly: feedback toolkit and Studio design rules (`ALIX/design/README.md:9-37`). | Status + toast, `showStudioModal` confirm with gate chips (`WE/editor.js:13786-13832`), gate chips, honest progress, legend. | Guided flows with back navigation, presets, operator/engineer modes, plain words, 44 px primary targets. Confirm only motion, upload-replace and REAL. | All | 1+ |
| **NFR-THEME** | **Yes.** | `ALIX/design/_tokens.css:8-62`, `WE/editor.css:1-189, 1714-1724`, brand mark `WE/template.html:15-52`, token-parity test `test_design_tokens.py:80-141`. | `plotTheme()` read from tokens; vendored fonts; fix the drift list (MAP 01 §1.7). | All | 1 |
| **INT-TRAJ** | — | — | §6.2 | Optimise | 2 |
| **INT-CTRL** | — | — | §6.1 | Robot screens | 4, 5, 7 |

---

## 4. Architecture

### 4.1 Process model

```
 Browser (one page, served from 127.0.0.1 only)
   │  JSON over HTTP (+ per-launch token header)   long-poll /api/lite/events
   ▼
 lite-web  (Python, stdlib ThreadingHTTPServer scaffold copied from ALIX Studio)
   │  project store · unit registry · canonical processor · gates · certificates
   │  ladder ledger · job registry · dual coordinator · audit log
   ├── pipe ──► lite-sim      1 process, 1 worker thread. alix_workspace + asset bundle loaded once:
   │                           UrdfModel, collision models, SRDF pairs, commode managers per state,
   │                           wall manager, contact policy. FK frames, preview checks, gate sweeps.
   ├── pipe ──► lite-arm-left  1 process = 1 SDK connection (real controller or emulator)
   └── pipe ──► lite-arm-right 1 process = 1 SDK connection
```

Why separate processes. Each reason is a safety reason.

- **FCL managers are not thread-safe**, and sharing them has segfaulted (`TG/body_collision.py:148-171`;
  `WS/dual_arm_plan_collision.py:1023-1025`). A crash in the sim worker must not take the robot channel with it.
- **SDK state is process-global**: `rm_init`, `rm_set_timeout` (`r14:2613-2614`), and the offline `Algo` tool frame
  (`EMU:115-146`). The emulator must be installed *before* the SDK is imported (`EMU:2541-2616`). One process per arm
  makes "is this handle real or emulated?" a property of the process, checked with `DP/connect.sdk_provenance`
  (`DP/connect.py:121`). A REAL agent refuses an emulated handle, and the reverse.
- **STOP must not wait for web threads, sweeps or uploads on the other arm.** Each agent runs commands as
  non-blocking state machines in slices of 50 ms or less and checks a stop flag between slices.
- **One command source per arm** (MAP 06 §7.3 item 9). The web process holds a per-arm robot-I/O lock with a named
  owner (screen or job). STOP ignores the lock.

Liveness rules, all fail-safe:

| Event | Agent behaviour |
|---|---|
| Web process heartbeat missing for 2 s during motion | Agent runs the stop ladder (`DP/track.py:3109` `stop_program`: slow stop → pause → stop, each step verified), stops the pole, freezes the hand, restores the global speed, and records the event. |
| Browser client heartbeat missing during a REAL run | Configurable. Default proposal: stop after 5 s (open decision D8). |
| Agent crash | Web marks the arm UNKNOWN and refuses further motion until reconnect. On restart the agent reads run state and error codes and re-asserts the global speed ratio. |
| SDK reply `-2` (late) | One retry for reads only. Never retry motion commands. |

### 4.2 Backend modules (package `alix_studio_lite`)

| Module | Responsibility | Reuse source and mode |
|---|---|---|
| `launcher` | Preflight: Python 3.10–3.12, venv lock, bundle manifest, SDK present, port free. Spawns processes, opens the browser. | **Copy/adapt** `ALIX/tools/run_studio.py:92-205` (preflight, neutral cwd), `TG/server.py:6493-6507`. |
| `server.http` | Request handler, gzip, 64 MiB cap, disconnect handling, vendor whitelist, error payload `{error, hint}`. **Loopback only**, and every POST must carry the per-launch token. | **Copy/trim** `TG/server.py:5178, 6320-6469` (about 200 lines). |
| `server.routes` | Route tables plus a **needs table** (`robot_io:left/right`, `sim`, `writes_project`, `long_running`). A test checks every route declares its needs. | **Pattern** `TG/server.py:5961-6253`; test from `TGP/test_generate/test_route_needs.py`. |
| `server.jobs` | Start/status/cancel jobs, honest progress, at most one live job per resource. | **Pattern** `TG/studio_generate.py:537-600, 861-1060`. |
| `server.page` | Assemble one HTML page (placeholders), mtime-stamped cache, `no-store`. | **Copy** `TG/scripts/visualize_generated_config.py:530-568`; `TG/server.py:6297-6317`. |
| `assets.bundle` | Load and verify the ALIX runtime bundle (§8.2). Set `ALIX_REPO_ROOT`, `ALIX_SRDF`, `BUTTERFLI_WORKSPACE_CONFIG`, `ALIX_WORKSPACE_CACHE`. Run the **world self-test**. | New. Mirrors the repo layout so ALIX code runs unmodified. |
| `limits` | One table: URDF position and velocity limits (via `UrdfModel`), 5° allowance (`TT:126`), step fault 10° (`TT:128`), 600°/s² advisory, rating allowance 0.97 (`TT:229`). Cross-checks controller read-back at connect. | **Import** `WS/urdf_kinematics.py`; constants from the pinned TT port (§5). |
| `timing` | Clocks: record 5.0 ms, gate 5.0 ms, nominal 5.5545 ms. Duration band. Upload-time estimate (0.24 s + 21.3 µs/byte). | **Import** `DP/track.py` constants and `expected_band_s` (`:61`); constants from `TT:102-121`. |
| `project.store` | Project folders, atomic writes (temp + rename), immutable raw and derived files, sha256, per-unit file locks. | New. |
| `project.registry` | Unit catalogue (MAP 13 §7.3 schema), PPT media and overlay boxes, status derived from files. | Data from MAP 13; strict-parse style of `ALIX/tools/taught_pipeline.py:183-221`. |
| `project.provenance` | `SOURCE.json`, recipe, certificate, audit `events.jsonl`, run records. | **Schema** from `TT:309-340, 3628-3634`. |
| `project.legacy_v6` | Import the V6 package read-only and split it into units by sample index (MAP 13 §6.2). Flagged LEGACY, never counted as raw. | New. Uses `V6/SHA256SUMS.txt` for the 28 entries that pass. |
| `motion.io` | Strict NDJSON reader (7 ints per line, refuse the whole file on any bad line); `.proj` builder. | **Import** `DP/track.parse_track` / `build_project_file` (`DP/track.py:66, 85`). |
| `motion.pipeline` | Recipe → bytes: trim (anchor rule, or HTML rolling-mean "classic"), smooth (5 Hz Butterworth, or HTML mean "classic" with the endpoint fix), per-segment scale with ramps and rating clamp, segment boundary cross-fade. | **Port** `TT:748-832, 894-951` and `H:1417-1616` (parity-tested). |
| `motion.gates` | G2 band-limited rate at 5.0 ms, G3 step, position margin, G4 pad deviation against the raw recording (FK at `{L,R}_glove_frame_2`), acceleration advisory, duration band. | **Port** `TT:420-604, 607-716`; FK **import** `WS`. |
| `motion.spikes` | R12 detectors, the remedy table, one-click fixes as recipe edits. | **Port** MAP 06 §4 detectors (`ALIX/experiments/.../predict_track_replay.py:82-136`; `ALIX/tools/cleaning_tracks.py:1743-1750`). |
| `motion.offset` | R9 Offset & Verify (§7.4). | **Import** `WS` FK/IK/SRS; algorithm new. |
| `motion.merge` | R8 merge engine (§7.5). | **Port** `plan_min_time.concatenate`; bridges new. |
| `sim.worker` | Warm models; FK frames per window; preview checks; gate sweeps; one thread, queue, restart on crash. | **Import** `WS/dual_arm_plan_collision.py`, `WS/collision.py`, `WS/plan_scene_validation.py:117-270`, `TG/body_collision.py:401-464` (fixture manager only). |
| `sim.adapter` | `.txt` + pole + hand + bin + fixture state → one-stage plan JSON (time = i × 5.5545 ms), pole/hand seed stages, hand counts → radians with `hand_units.py`. | **Port** `ALIX/tools/dual_arm_verdicts.py:506-660` (parity test). |
| `sim.world` | World composition and **world manifest**: robot links, bin state and inclusion, commode parts at state, walls/venue, contact policy. Refuses if any expected object is missing. | **Import** `contact_links.yaml` (data), `WS/redundancy.py:95-114` `classify_pair`. |
| `robot.agent` | Per-arm process: connect, command loop, events, heartbeat, stop flag, fail-safe. | New; semantics from `DP/`. |
| `robot.verbs` | `connect` (`DP/connect.open_arm`, ack timeout 1500 ms), `state` (joints + error codes), `upload` (`send_project` + `program_list` page 10 + size/name verify + skip-unchanged), `run` (`run_program` + `global_speed_guard`), `stop` (`stop_program` ladder), `mode` (`runmode`, read-back), `pole`/`hand` (`peripherals`), `drag_*` (ported from `r14:912-989`, with save retry). | **Vendor/import** `DP/track.py:85-198, 3075-3130, 3517`; `DP/runmode.py`; `DP/peripherals.py`. **Port** drag teach from r14. |
| `robot.backend_emu` | Developer/CI backend: `alix_emulator.install()` + `open_arm` (auto-wrapped by `emulated_program`) + a seam fake for drag-teach verbs. | **Import** `EMU`, `DP/emulated_program.py`. |
| `robot.legacy_runner` | Transitional wrapper around r14 as a subprocess (hash-pinned, `--ip`, `--sdk`, `--yes`), parsing stdout tokens (MAP 09 §1.4). Also the parity oracle for `.proj` bytes. | **Reference/wrap** r14 unchanged. |
| `robot.dual` | Dual coordinator: SAFE TIMELINE planner with an explicit `checkpoint` step; two-phase start (both ready → GO); cross-arm abort; handoff confirmations. | **Port** pure functions `v53:9619-9997` (`_page2_program_chunks`, `_page2_build_program_rule_plan`), parity-tested on the V6 FINAL JSONs. |
| `robot.slots` | Slot policy: the ID range Lite owns per controller; slot names carry a short hash; refuses writes outside the range. | New. |
| `ladder` | OFFLINE / EMU / CONTROLLER-SIM / REAL records and gates per fingerprint. | **Import** `DP/ladder.py:63-145` (record, latest, gate_real) with `root=<project>/ladder`. |

### 4.3 HTTP API (`/api/lite/*`, all JSON; expected refusals are `200 {error, hint}`)

| Area | Endpoint | Needs |
|---|---|---|
| State | `GET state` (versions, bundle id, sim worker, agents, live jobs, locks) · `GET events?since=N` | — |
| Registry | `GET units` · `GET unit?id=` · `POST unit/preset` (pole, hand, cleaning time; supervisor) · `POST import-recording` (assign a v53 file to a unit and answer the provenance questions) | writes_project |
| Optimise | `POST optimise/preview` (recipe → decimated rows + gates + spikes) · `POST optimise/commit` (→ derived file + certificate) · `POST optimise/undo` · `POST optimise/redo` | writes_project |
| Sim | `GET robot-geometry?set=` · `GET fixture-geometry?state=&venue=` · `GET sim/world` · `POST sim/frames` (window) · `POST sim/preview-check` · `POST sim/check` (gate job) | sim (+ long_running) |
| Offset | `POST offset/solve` (job) · `POST offset/accept` | sim, writes_project |
| Combine | `POST merge/plan` · `POST merge/build` (job) · `POST workflow/plan` (staged plan + dual timeline) | sim, writes_project |
| Robot | `POST robot/connect {side, ip, port}` · `POST robot/identify` · `POST robot/mode {side, SIM\|REAL}` · `POST robot/pole` · `POST robot/hand` · `POST robot/drag/{start,stop,save}` · `POST robot/upload` (job) · `POST robot/run` (job, ladder-gated) · `POST robot/verdict` · `GET robot/live?since=` | robot_io:side |
| Robot | `POST robot/stop {side\|all}` | **none: always accepted** |
| Jobs | `GET job?id=` · `POST job/cancel` | — |

Security: Studio has no authentication (`TG/server.py:6539-6544`). That is acceptable there because Studio cannot
move anything. Lite can. Lite binds to `127.0.0.1` only. A random token is injected into the page, and every POST
must carry it as an `X-Lite-Token` header. With a JSON content type, that forces a CORS preflight, so another
browser tab cannot send motion commands.

### 4.4 Front-end screens

The shell is copied from Studio: app bar with brand mark, title "ALIX Studio Lite", the **mode chip** (OFFLINE SIM /
DRY RUN on robot / REAL) and per-arm connection dots with the IP actually in use; status line and toast; an
always-visible **STOP ALL**. The Fusion-style ribbon is not lifted (MAP 01 §8.4). Lite uses a left navigation in
flow order. Numbered steps are fine here because Lite is a linear flow.

| Screen | Purpose | Requirements |
|---|---|---|
| **Dashboard** | Board-style matrix: 22 deck steps × LEFT/RIGHT. Chips (glyph + word): NOT TAUGHT, RAW, OPTIMISED, SIM OK, DRY-RUN OK, REAL TESTED, IN PROGRAM, plus flags LEGACY V6, NEEDS RE-TEACH, FAILED. Per-arm counters. "Teach next missing". | R1, NFR-UX |
| **Regions** | PPT overview (PAGE 04/05 with hotspots), step list grouped by fixture state, unit card (overlay picture, arm colour, fixture-state question), pole/hand preset with Apply + read-back, cleaning time. | R1, R7 |
| **Teach session** | Wizard per unit: (1) card and fixture-state confirmation, (2) prepare: hand clear of the toilet → arm still → pole → hand, with interlocks, (3) START DRAG + countdown of the cleaning time, (4) STOP & SAVE to the canonical name + `SOURCE.json`, (5) offline 3-D replay and stats, REDO / ACCEPT / SKIP with reason, (6) auto-advance. | R1, R7 |
| **Optimise** | Per unit. Segment table (sub-regions) with trim/smooth/scale per segment. Presets (Gentle / Standard / Fast). Advanced drawer. Undo/redo ×5 with labels. Joint plots in degrees against seconds. Spike panel with suggestions. Live gate chips. Mini 3-D preview with a preview check. Raw vs result comparison. | R2, R6, R11, R12, INT-TRAJ |
| **Simulate** | Dual-arm 3-D on the left, plots on the right, with a draggable splitter. Transport (play, pause, ×0.5/×1/×2, step). Collision bands on the joint plots, **stop at first collision**, link highlight, "Go to problem", "Back to editing". Bin include/exclude + state, fixture state, venue. A "What was checked" panel shows the world manifest. | R3, R5, R10, R11, R14 |
| **Offset** | Plain-language offset ("towards the toilet / sideways / up / press deeper", in mm) on a unit or segment. Solve. Report ("max achievable here: N mm", flagged spans). Before/after overlay. Accept → new derived file. | R9, R14 |
| **Combine** | Pick units to merge (only those with the same pole, hand and fixture state). Boundary report, bridges, collision check of bridges and transits. Build the program. Dual workflow builder (TOGETHER / MASTER FIRST / SUPPORT WAIT / CHECKPOINT) and dual timeline. | R8, R10, INT-CTRL |
| **Robot test** | Connection (IP per arm, Test, Identify), mode read-back, upload with progress/verify/skip, dry run then real run. Confirmation modal names the destination, slot, gates and ladder status. Live mirror (3-D + plot cursor), error → plot row, operator verdict → back to editing. | R4, R5, R13, INT-CTRL |
| **Settings (engineer)** | IP defaults, slot range, SDK path, bundle and versions, logs export, ladder overrides (reason required), gate strictness. | R4, R13 |

Shared front-end modules, all copied and restyled:
- `lite_shell.js`: status/toast/busy/modal/tooltips/help/docks/dialogs (MAP 01 §8.2 ranges).
- `lite_viewport.js`: `initThree` trimmed, `meshFromPayload`, `loadRobotGeometry`, `applyRobotPose`, fixture and walls rendering, bin visibility (`WE/editor.js:309-645, 667-703, 743-830, 9853-9874, 12352-12437`).
- `lite_playback.js`: DAC goto/highlight/jump/log and the transport (`WE/editor.js:8749-8767, 14272-14600`).
- `lite_plot.js`: `plotTheme()` read from tokens; joint palette that keeps red and green for state only; cursor and band shapes from the HTML (`H:1134-1142`); linked zoom.
- `lite_traj_preview.js`: the parity-tested JS port of the processor.

### 4.5 Data model and project folder layout

```
<LiteProjects>/<project>/                         e.g. commode_c_siteA
  project.yaml        project id; fixture commode_c; venue; bin policy; arms {left:{ip,port,identity}, right:{…}};
                      slot ranges; Lite version; ALIX commit; bundle sha256
  registry/units.yaml unit catalogue (schema below), supervisor-editable, validated on load
  registry/ppt/       slide renders hi-NN.png, media, overlay boxes (MAP 13 App. A)
  units/<UNIT_ID>/
    raw/<UNIT_ID>__RAW__<yyyymmdd-hhmmss>.txt           immutable, bytes exactly as saved by the controller
    raw/<…>.SOURCE.json                                 provenance (below)
    edits/recipe.json                                   current recipe + labelled undo/redo ring (5)
    derived/<UNIT_ID>__OPT_v<k>.txt                     canonical processor output, immutable
    derived/<…>.cert.json                               certificate (below)
    sim/<fingerprint>/{plan_L.json, plan_R.json, sweep.json, world.json, verdict.json}
    runs/<run_id>/{run.json, events.jsonl, capture/, operator_verdict.json, legacy_stdout.log}
  programs/<name>/{<name>.txt, manifest.json, cert.json, sim/}                 merged programs (R8)
  workflows/<name>.workflow.json + <name>.timeline.json                         dual workflow v2 + timeline
  ladder/ladder.jsonl                       append-only; rows in DP/ladder.py:63-87 format plus rung "OFFLINE"
  controller/slots_<side>.json              last-read slot map + sha of the bytes Lite sent (skip-unchanged)
  audit/events.jsonl                        every operator action, robot command, verdict and override (with reason)
  legacy/V6/…                               imported read-only; segment → unit mapping with confidence
```

Key records (fields abbreviated):

- **Unit** (MAP 13 §7.3): `id, deck{code,label,task,slide,code_source}, arm, type(surface|mechanism|tool|glove|idle),
  map_region, part, step, paired_with, sync(TOGETHER|MASTER_FIRST|SUPPORT_WAIT), role, fixture_state(CLOSED|LID_OPEN|SEAT_OPEN|ANY),
  cleaning_time_s, pole_mm, hand{preset, counts[6]}, pad_frame (default glove_frame_2), tool, image{media,box_uv,mirror},
  legacy{…}`.
- **SOURCE.json** (ALIX taught-lane schema plus Lite fields): `unit, side, controller_ip, arm_identity_check, pole_mm,
  hand_counts, hand_preset, fixture_state, pad_frame, tool, taught_by, recorded_at, points, sha256, units:"millideg",
  clock_record_s:0.005, hardware:"ok", lite_version`.
- **Recipe**: `format: alix_lite_recipe_v1, source{file,sha256}, clocks, trim{method,…}, smooth{method,…,edge}, segments[{id,
  start_s, end_s (raw time), trim?, smooth?, scale_pct}], speed{ramp_per_line:0.05, rating_cap:0.97}, history[≤5 undo + redo, labelled]`.
- **Certificate**: `bytes_sha256, proj_sha256, parent_sha256, recipe_sha256, recipe (copy), gates{name: value, bar, pass},
  advisories, duration_band_s, predicted_upload_s, tool_versions{lite, alix_commit, bundle_sha}, created_at`.
- **Fingerprint** for the ladder: `sha256(.proj bytes) + ":" + sha256(context)`. The context holds pole, hand, bin
  state/inclusion, fixture state, venue, partner fingerprint (dual), transit %, and bundle sha. A pass licenses the
  next rung **only for the identical fingerprint**.
- **Workflow v2**: the steps of `realman_workflow_sequence_v1` (`r14:1668-1748`) plus an explicit `checkpoint{message}`
  step (instead of v53's "a positive WAIT before TOGETHER"), `unit` references, and `sync`. v1 files migrate with
  `speed` → `transit_speed`.

### 4.6 The validation ladder

| Rung | What runs | What it proves | What it cannot prove |
|---|---|---|---|
| **GATES** (on every commit) | Byte gates G2/G3/margin/G4, acceleration advisory | Bytes are legal for the controller and close to the recording | Collisions, missed spots |
| **OFFLINE** | 3-D FCL sweep: arms, self, partner, bin as configured, commode at the unit's state, walls; contact policy; dt 0.02 s | No unexpected contact in the *modelled* world | Model gaps: fingers on the slim set, physical bin height, venue calibration (§11) |
| **EMU** (developer/CI only) | Emulator + emulated program channel | Program logic: upload, list, run, stop, speed restore | Geometry ("emulator geometry is not evidence", `DP/ladder.py:68-72`) |
| **CONTROLLER-SIM** ("dry run on robot") | Real controller in run mode 0, read back; arms stay still | The controller accepts and plays the program without faults (limits, speed, its own self-collision) | Pole and hand (not simulated); the commode (the controller has no model of it) |
| **REAL** | Supervised real motion | Real outcome; the operator records a verdict | — |

Gating follows `DP/ladder.py:101-145`: a failed CONTROLLER-SIM blocks REAL with no override. A *missing*
CONTROLLER-SIM record may be overridden only in engineer mode, with a written reason that goes to the audit log.
Lite adds one rule: **REAL also needs an OFFLINE pass for the same fingerprint, and this cannot be overridden.**

### 4.7 The sim/real seam

One agent interface, three backends, plus the offline simulator, which is not a controller backend:

```
AgentAPI: connect(ip,port) · identify() · state() · mode(get/set) · upload(bytes,slot) · list() · run(slot,transit%)
          · stop() · pole(mm) · hand(counts) · drag_start/stop/save(path) · events()
  ├─ RealBackend      DP/connect.open_arm(side, ip) on the vendored RealMan SDK          → controller run mode 0 or 1
  ├─ EmuBackend       alix_emulator.install() + open_arm (emulated_program attach) + drag seam fake   [dev/CI]
  └─ LegacyRunner     r14 subprocess per action (transition only; Linux only)
Offline simulator (lite-sim): never a controller; the default meaning of "Simulation" for operators.
```

- Operator wording for R4's toggle: **"Simulation (on this computer)"**, **"Dry run on the robot (arms stay still)"**
  and **"Real motion"**. The mode chip always shows which one is active, with the controller's read-back value.
- Live mirror: during a dry run or a real run the agent polls joints at about 10 Hz (never while uploading), or
  records UDP frames with `DP/recording.RunRecorder`. The web process computes FK in the sim worker, and the
  viewport and plot cursor show the result. **One FK for both views** (Studio's own rule, `TG/server.py:5300-5303`).
- Arm identity: `DP/track.assert_side` (`DP/track.py:341-352`) only compares labels, so a swapped IP would get
  through. Lite adds `identify()`. It reads the drive limits at connect (`rm_get_joint_drive_min/max_pos`); J4 differs
  per arm, L ±169° vs R ±140° (MAP 05 §3, `EMU:429-433`). The operator then confirms with a tiny hand-finger wiggle,
  only with the arm still and the pad off surfaces. The IP-to-side binding is stored in `project.yaml` and checked again
  whenever the IP changes.

---

## 5. Reuse strategy per source

### 5.1 Decision table

| Source | Decision | Why (lens) | Guard |
|---|---|---|---|
| `alix_workspace` (FK, IK, SRS, collision, plan frames, SRDF states, wall checker, `classify_pair`) | **IMPORT, mandatory**, pinned to the ALIX commit | FK matches the controller to 0.007 mm (MAP 05 §0). It is the kinematic ground truth of every ALIX gate. A second FK "is how two views of the robot start disagreeing" (`TG/server.py:5300-5303`). | Import-boundary test; pinned commit in the certificate. |
| URDF, SRDF, collision and visual meshes, commode_c scene + STLs, venue walls, `initial_positions.yaml`, `contact_links.yaml`, `hand_units.py`, `design/_tokens.css` | **VENDOR as the ALIX runtime bundle**: byte-identical extract at the pinned commit, repo layout kept | The libraries find these by repo-relative markers. Copying them with their layout and a hash manifest keeps one source of truth. | `MANIFEST.sha256` checked at start-up; world self-test (§8.2). |
| `visual_overlay_baked.npz` | **RE-BAKE upstream** (`ALIX/tools/meshes/bake_visual_overlay.py`), then bundle | The current bake shows the old 406 mm bin and has no camera (MAP 11 §2.4). | Bake hash in the bundle; bin z-max check in the self-test. |
| `alix_taskgen.body_collision`, `scene_pose`, `fixture_parts`, `transforms`, `paths`, `constants` | **IMPORT these modules only**, from the pinned `alix_taskgen` | The commode pose maths carries hardware fixes G37 (washroom rotation) and G38 (standoff) (`TG/body_collision.py:401-464`). Re-deriving it is the exact failure those fixes closed. The modules import only numpy, scipy, yaml and trimesh. | Import-boundary test: no `alix_taskgen.server`, `studio_*`, `path_*`, `cspace_*`. |
| `alix_taskgen.server`, generator, `editor.js`, `template.html` | **DO NOT IMPORT.** Copy UI pieces (§4.4) | X1/X2, 30–60 s start-up, test-pinned UI. | Same test. |
| `alix_dispatch` (`track`, `connect`, `runmode`, `ladder`, `emulated_program`, `peripherals`, `backends`, `limits`, `recording`) | **VENDOR verbatim inside the bundle** (`core/python/alix_dispatch/src` on `sys.path`, as `ALIX/tools/alix-dispatch:16-23` does). Ask upstream for a `pyproject.toml`, then IMPORT. | Most hardware-evidenced controller semantics: CRLF `.proj`, page-10 listing, global-speed guard, verified stop ladder, run-mode read-back, ladder (MAP 06 §7.3). | Byte-identical hash check; no local edits; extensions only at seams (ALIX's own rule D4). |
| `alix_emulator` + vendored RealMan SDK | **IMPORT** from the bundle (developer/CI rung) | Verbatim-pinned upstream (`EMU`; test_emulator_verbatim). | — |
| `ALIX/tools/taught_tracks.py` gates and filters, `plan_min_time.concatenate`, `dual_arm_verdicts` shim | **PORT** into `motion.*` / `sim.adapter` | They live in CLI scripts with `sys.path` hacks (`TT:79-92`), so they are not importable. | **Golden parity tests**: outputs from the pinned ALIX functions stored with hashes; ≤ 1 mdeg / ≤ 0.01 mm. |
| Trajectory HTML compute core (`H:1417-1620`, `H:944-1050`) | **PORT twice**: canonical Python + JS preview | Uploadable bytes come from one implementation only. | JS ↔ Python parity in quickjs. "Classic" mode reproduces `UC/zip/02…30pct` within 1 mdeg (MAP 10 §1.4). |
| Trajectory HTML UI (stepper, tables, plots, box-select, cursor) | **COPY + restyle** | Presentation only. | Theme tests. |
| Studio theme, shell CSS/JS, viewport, DAC playback, Board, Simulate polling | **COPY** (with the drift fixes from MAP 01 §1.7) | Presentation; `editor.js` is not importable. | Token parity (`test_design_tokens.py` copy), no remote assets, JS integrity and quickjs execution tests. |
| Studio server scaffold, jobs, page assembly | **COPY + trim** | About 200 lines of proven plumbing. | Route-needs test. |
| Runner r14 + `UC/pole.py`, `UC/hand.py` | **REFERENCE**; transitional **WRAP** (hash-pinned); **PORT** drag teach | Field-proven, but duplicates `alix_dispatch` (lines 1–268 = `tools/rm_upload.py`). | `.proj` from Lite is byte-identical to r14's `write_project` for the same input. |
| UI v53 | **REFERENCE ONLY**; port the pure planner functions | The Tk UI is replaced. The planner logic runs the V6 package. | Planner parity on V6 FINAL JSONs (7 stages, MAP 08 §1.8). |
| V6 package | **IMPORT AS LEGACY DATA** | Merged, time-stretched, untested; 10 of 15 files exceed joint ratings (MAP 07 R-A4). | Not uploadable until re-certified through Lite's gates. |

### 5.2 Where copying is acceptable and where importing is mandatory

- **Importing is mandatory** for kinematics, the collision world (robot, commode, walls, contact policy), joint
  limits, controller I/O semantics and the ladder. A divergence there changes whether metal moves.
- **Copying is acceptable** for presentation code, HTTP plumbing and job patterns.
- **Porting is acceptable** only for algorithms that ALIX keeps in non-importable scripts, and only with a golden
  parity test. Each port has an upstream request (§5.3) and is deleted when that lands.

### 5.3 Upstream requests to ALIX (Lite works without them)

| ID | Request | Lite workaround meanwhile |
|---|---|---|
| U1 | Add `pyproject.toml` to `alix_dispatch` | Vendored, hash-checked bundle copy |
| U2 | Move the taught-lane pure functions (`strip_pauses`, `smooth_taught`, gate maths, `deviation_mm`, re-timers, `concatenate`) into an importable module | Ports with golden parity |
| U3 | Parameterise `compute_dual_arm_plan_collisions`: `body_links`, bin state, arm link set; accept in-memory timelines; add a warm `DualArmSweeper` (build once, sweep many) | `include_body=False` plus a Lite-built body manager posed at the chosen bin state, passed in `extra_managers`. Warm preview sweep parity-tested against the library call. |
| U4 | Move the fixture-world composition (`fixture_manager`, `scene_pose`) into `alix_workspace` | Whitelisted import from `alix_taskgen` |
| U5 | Re-bake the visual overlay after the BinShort change | Lite bundles a re-bake from the upstream tool |

---

## 6. Integrations

### 6.1 Control application (INT-CTRL)

- **What survives:** the runner's *semantics* (store-only upload with read-back verification, pre-run gates,
  transit-only speed, global-speed restore, slow stop on interrupt, pole → hand order, Modbus hand, lift model,
  workflow step types) and v53's *dual logic* (TOGETHER blocks, RUN FIRST / WAIT OTHER handoff, lid checkpoint, live
  arm-status vocabulary). The Tk UI and the stdout protocol do not survive.
- **How, step by step:**
  1. Phase 4/5: the agents implement the verbs on `alix_dispatch`. Drag teach is ported from `r14:912-989` (adding the
     save retry ALIX found necessary, MAP 09 §5).
  2. During transition, `robot.legacy_runner` can run r14 unchanged (Linux only) for any verb not yet ported. It always
     passes `--ip`, `--sdk` and `--yes` after Lite's own confirmation, and maps the stdout tokens (MAP 09 §1.4) to events.
  3. A parity test proves Lite's `.proj` bytes equal r14 `write_project` output for the same `.txt`.
  4. The v53 planner functions are ported into `robot.dual`, with the checkpoint as an explicit step. On the V6 FINAL
     JSONs the planner must give v53's 7-stage plan.
  5. Retire the wrapper after Phase 5 hardware acceptance. Keep r14 as a test oracle only.
- **What Lite adds over v53:** IP entry and arm identity; enforced versions (no silent runner fallback); controller
  connections held by the backend (no per-stage reconnect or list re-read); cross-arm abort in TOGETHER blocks; a
  restored REAL-mode warning (v32 had it, MAP 08 §2); structured events and run records; slot policy; the ladder.
- **What Lite drops for operators:** global points (kept as a "Park" button at most), Multi Run (superseded and
  broken), raw 6-channel sliders and free mm entry (moved to engineer mode).

### 6.2 Trajectory optimisation HTML (INT-TRAJ)

- **Algorithms:** `trimIdle`, `meanSmooth`, `mapRegionsToTrimmed`, `interpolateRegion` and `applyRegionScales`
  (`H:1438-1616`) become `motion.pipeline` "classic" methods in Python, so earlier outputs can be reproduced. They
  include these fixes:
  - (R-A1) no endpoint snap: edge taper or padding, not overwriting rows 0 and n−1;
  - (R-A5) ramped scale boundaries (≤ 0.05 per line, `TT:224-226`);
  - (R-A4) rating clamp at 0.97 × rating at 5.0 ms.
  
  ALIX's anchor pause rule and 5 Hz Butterworth are offered as **"Recommended"**, and the classic HTML methods as
  **"Classic"**.
- **Suggestions:** the HTML's equal-chunk heuristics (`H:944-1050`) are replaced by rating-based analysis (§7.2). Its
  table UX is kept.
- **UI:** the 6-step stepper (`H:609-704`) becomes the Optimise screen's flow. Plots use seconds and degrees.
  Box-select creates segments, and segments are drawn on the plot (the HTML never draws them). Comparison keeps its
  metrics, adds a time-aligned x axis and the real duration band (`TT:116-121`). Plotly is the vendored 2.35.2,
  the same version as the CDN tag in `H:7`.
- **Naming:** the HTML's "regions" are time windows. Lite calls them **segments**, so they are not confused with
  cleaning regions (MAP 10 §1.4).

---

## 7. Key algorithms (brief)

### 7.1 Canonical processor and gates
- Input: raw bytes plus a recipe. Output: derived bytes plus a certificate. Deterministic: the same recipe hash
  gives the same bytes.
- Order: trim → smooth → per-segment scale. Each segment's parameters apply with a cross-fade of about 0.25 s at
  segment boundaries.
- Hard gates by default:
  - inside URDF limits minus a 5° allowance (`TT:126`);
  - G2: 0 ticks over a rating at 5.0 ms, 20 Hz band-limited;
  - G3: ≤ 10° per line;
  - G4: pad deviation against the **raw** recording ≤ 10 mm. 5–10 mm shows a warning chip.
  
  ALIX notes that G4 against a derived file reads 0.0 (MAP 06 §2.5), so Lite always compares with the raw file.
- Advisory: share of lines over 600°/s², one-tick speed step, slowness census.
- Durations shown as a band (`replay_s ≈ 0.2322 + 5.6737 ms × (N−1)`, ±6 %; MAP 06 §2.1), never "N × 5 ms".

### 7.2 Spike analysis (R12)
Detectors and remedies come from MAP 06 §4.

| Spike | Remedy |
|---|---|
| Band-limited rate over a rating | "Slow only this stretch" (cap at 0.97 × rating), or subdivide |
| Raw ripple | 5 Hz de-jitter |
| Corner forces a slowdown | Round the corner |
| Shake at speed | 6 Hz post low-pass |
| Fast arrival near a limit | Slow the approach |
| Step > 10° | Refuse |
| Boundary speed jump | Ramp |
| Endpoint snap | Taper |

Every remedy is a recipe edit, so it is undoable. Lite never promises "slower = more accurate" in general (MAP 06 §4).

### 7.3 Collision world, contact policy and verdicts (R3, R5, R10)
- The world is built per verdict and stored in `world.json`:
  - robot links (`alix.urdf`);
  - bin state and inclusion: `include_body=False` plus a Lite body manager posed at the chosen state, with the
    glove lever always included;
  - the commode from `BodyCollisionWorld.fixture_manager(commode_c, fixture_state, standoff)`;
  - walls from `load_wall_manager(venue)`;
  - pole and hand joints seeded from the unit (avoids the 150–225 mm pole error, `WS/dual_arm_plan_collision.py:402-420`).
- The **world manifest** asserts all of these:
  - expected object counts: 80 collision meshes resolved, SRDF disabled pairs loaded (1,824), commode parts {body,
    lid, seat} present, walls present;
  - the fixture state applied;
  - the bin state applied.
  
  If any assertion fails, there is no verdict.
- **Contact policy:** a pair is *intended contact* if one side is in `contact_links.yaml[<pad_frame>]` and the other
  is the fixture part being cleaned in that unit. Everything else is a **collision**, classified with `classify_pair`
  (fixture / environment / partner / self). This keeps every wiping stroke from being reported as a collision.
  It still catches the kind of finding ALIX made, where `IRH_right_thumb_2↔commode_lid` was not in the
  `glove_frame_2` set (MAP 05 §4.6).
- **Gate sweep:** dt 0.02 s, because dt is a correctness parameter (`ALIX/tools/dual_arm_verdicts.py:350-356`).
  For TOGETHER pairs it adds a windowed arm-vs-arm clearance of at least 25 mm with `cross_arm_separation` (full link
  set) as a background job. **Preview check (R11):** a warm windowed re-check of the edited segment ± 1 s, labelled
  "preview".
- Mapping to the plots: sample time t maps to track row `round(t / 5.5545 ms)`, using the adapter's own clock, so the
  mapping is exact.

### 7.4 Offset & Verify (R9, R14)
1. FK of all points (`fk_batch`, 2.5 µs/pt).
2. Offset profile `d_k = w_k·d`, with raised-cosine ramps of 0.3–0.5 s at the ends.
3. Differential IK per point:
   - seed = previous solution + taught increment;
   - 2–4 damped least-squares iterations with a null-space pull towards the taught q;
   - adaptive damping near singular sets;
   - clamp to URDF limits minus 5°.
4. Optional exact polish (`IKSolver`, `pos_tol 0.1 mm`, `restarts=1`) where the singular margin is ≥ 10°.
5. Per-point gates:
   - limits;
   - adjacent step ≤ max(1.5 × taught step, 0.5°) and never over 10°;
   - rate at 5.0 ms;
   - Cartesian residual ≤ 1 mm;
   - deviation ≤ 5 mm (10 mm hard);
   - singular-margin warning.
6. Repair, never silently. Taper `d` locally and report "max achievable N mm here", or refuse that span. Refused spans
   keep the taught points with ramped transitions.
7. The output is a new derived file, which must pass OFFLINE before any robot rung.

The frames offered to operators are base_link axes in plain words, plus "press deeper" along the pad axis
(`TG/glove_frames.py:134-141`).

### 7.5 Merge (R8)
1. Only units with the same pole height, hand state and fixture state are merged into one program (a pole move forces a
   program boundary, `V6/README.txt:33-39`). Different states become workflow steps.
2. Joins: gaps ≤ 0.5° are concatenated (duplicate dropped). Larger gaps get a velocity-matched quintic bridge whose peak
   rate stays within the ratings at 5.0 ms.
3. **Every bridge, and the controller's implicit transit to the first point** (up to 64.6° unchecked in V6, MAP 10
   §3.2), goes through the gate sweep with the commode at the right state, the partner arm, the bin and the walls.
4. Output: program bytes, `manifest.json` (rows ↔ units/segments, pole, hand, fixture state, bridges), certificate,
   sweep, and for dual programs the workflow v2 plus the timeline.

---

## 8. Deployment and packaging

### 8.1 Target and environment
- **OS:** Linux x86_64 (Ubuntu 22.04/24.04) first. Windows 64 only after the agent's in-process stop has been checked
  on Windows; the legacy subprocess path is never used there. macOS can run offline only (the SDK has no `.dylib`,
  MAP 11 §4.3).
- **Python** 3.10–3.12 in a dedicated venv from an offline wheelhouse with a lock file:
  - numpy, scipy, trimesh;
  - python-fcl (wheels only; a missing wheel cannot be built, `ALIX/SETUP.md:469-475`);
  - pycollada, lxml, networkx, pyyaml;
  - plus `alix_workspace` and `alix_taskgen` wheels built from the pinned commit.
  
  The system Python on the dev machine has a numpy/scipy ABI clash (observed in MAP 05/07/10), so a venv is not optional.
- **Server:** Lite listens on 127.0.0.1:8766, so it can run beside Studio on 8765.

### 8.2 The ALIX runtime bundle
- Built with `git archive <commit> -- <path list>`, so it is byte-identical and carries the commit id. It is about
  20 MB (MAP 04 §7.4).
- Contents:
  - `ARCHITECTURE.md` (the marker file);
  - `core/python/alix_dispatch/src`, `core/python/src/alix_emulator`;
  - `external/butterfli-realman-alix/Python` (the SDK; licence open);
  - `core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf`;
  - `ros/description/*` meshes + `package.xml` + `initial_positions.yaml`;
  - `ros/manipulation/alix_moveit_config/config/alix.srdf`;
  - `ros/orchestration/alix_tasks/config/{commode_cleaning/commode_c, scene, contact_links.yaml}`;
  - `ros/orchestration/alix_orchestrator/alix_orchestrator/core/task_config.py` (foul-band constants; read as data or ported with parity);
  - `ros/hw/alix_hw/scripts/hand_units.py`;
  - `design/_tokens.css`;
  - the re-baked overlay;
  - optionally `vendor/realsense-ros/realsense2_description` for the D435 mesh, at 15.8 MB (MAP 11 §1.4).
- A `.git` **marker file** at the bundle repo root makes `PackageIndex.find_repo_root` stop there
  (`WS/package_index.py:27-37`). Without it, the nearest `src/` ancestor would hide the description packages and the
  meshes would silently disappear.
- **Self-test at every start** (fail closed). It checks that:
  - the manifest hashes match;
  - all `package://` URIs resolve (0 missing);
  - the collision model has the expected link and mesh count;
  - the SRDF loads with 1,824 disabled pairs;
  - the commode manager holds body, lid and seat for all three states;
  - the walls are present;
  - `hand_units.py` loads;
  - FK at three fixed joint vectors matches the stored values bit for bit;
  - the bin bake z-max is 0.398 m (BinShort).

### 8.3 Launcher, versions and logs
- **Launcher steps:**
  1. Run the preflight.
  2. Start lite-web and show the UI within 10 s.
  3. Warm the sim worker in the background (10–15 s) with an honest "Simulation warming up" status.
  4. Start the agents only when the operator connects.
- **Versions:** the Lite version, ALIX commit, bundle sha and runner sha (if used) are shown in About and stamped into
  every certificate, run record and audit event.
- **Logs:** per-project `audit/events.jsonl` and `runs/`. "Export support bundle" zips logs, certificates and the
  world manifests. It never includes raw data unless the user asks.

---

## 9. Testing strategy (emulator → controller dry run → real)

| Layer | Tests |
|---|---|
| Unit / parity | ALIX golden parity for every port. JS preview equals Python. `.proj` bytes equal r14. Planner equals v53 on the V6 FINAL JSONs. Lite's warm sweep equals `compute_dual_arm_plan_collisions` on golden pairs. |
| World safety | Self-test negative cases: remove a mesh, the SRDF or the commode scene → no verdict. Synthetic collision (arm driven into the lid) → flagged. Wiping stroke on the right surface → contact, not collision. Thumb on the lid with `glove_frame_2` → collision. |
| UI | Copied Studio guards (token parity, no remote assets, JS integrity, quickjs execution); route-needs; a layout audit in a real browser (`ALIX/tools/layout_audit.py` pattern). |
| Emulator integration (CI, Linux) | Upload → list → verify; skip-unchanged; run → complete; stop mid-run → verified stop + speed restored; kill the agent mid-run → web shows UNKNOWN; run-mode read-back; ladder refusals; dual two-phase start; cross-arm abort. Based on `DP/emulated_program.py` and `ALIX/core/python/alix_dispatch/test/test_rm_upload.py`. |
| Hardware acceptance (supervised, every release) | (a) STOP on a running stored program: measured time and verified state. (b) Global speed restored after normal end, stop, agent kill and web kill. (c) Run-mode read-back and refusal when unknown. (d) Upload throughput, skip-unchanged, both arms in parallel. (e) Dual start skew with two-phase start. (f) Pole/hand interlocks. (g) Arm identity check with swapped IPs is refused. (h) Controller dry run: what the pole and hand do. |

---

## 10. Phased delivery plan

Estimates are for one experienced developer including tests (S ≤ 3, M 4–8, L 9–15, XL > 15 person-days).
Hardware sessions need a supervisor and lab time; they are counted inside each phase.

| # | Phase | Goal | Requirements | Person-days |
|---|---|---|---|---|
| 0 | Foundations and hardware characterisation | Pinned environment and bundle, single sources, safety unknowns measured before design locks | Base for all; X1/X2 boundary | 7 (M) |
| 1 | Shell, project store, registry, dashboard, regions | Operators can see what is taught; v53 recordings can be imported with provenance | R1 (dashboard/regions), NFR-THEME, NFR-UX | 11 (L) |
| 2 | Canonical processor and Optimise screen | Certified, gated, undoable edits per unit and segment; spike advice | R2, R6, R12, INT-TRAJ, R11 (gates part) | 15 (L) |
| 3 | Offline simulation and collisions | Trustworthy 3-D check with commode, walls, bin, contact policy; stop-at-collision plots | R3, R5 (sim), R6, R10 (single pair), R11 | 15 (L) |
| 4 | Robot agents, connection, pole/hand, Teach Session | Safe connect with IP and identity; guided teach with presets and interlocks | R1 (teach flow), R4 (connect/mode), R7, INT-CTRL | 12.5 (L) |
| 5 | Upload, dry run, real run, live mirror | Ladder-gated real testing with verified stop and faster upload | R4, R5 (real), R13, INT-CTRL | 11 (L) |
| 6 | Offset & Verify | Offsets that are continuous, within limits, and simulated first | R9, R14 | 11 (L; research risk) |
| 7 | Combine and dual workflow | Collision-checked merges and dual timelines that execute safely | R8, R10 (dual), INT-CTRL (dual) | 14 (L) |
| 8 | Hardening and release | Installable, documented, operator-tested product | NFR-UX, deployment | 6.5 (M) (+2 Windows, optional) |

**Total ≈ 103 person-days** (about 5 months for one developer), plus about 15 % contingency for lab availability and
the R9 research risk.

**Minimum safe release = Phases 0–5 (≈ 71.5 person-days).** It covers R1–R7, R11, R12, R13 and the control-app
replacement for single-arm programs. Dual programs keep running in v53 until Phase 7.

### Phase details

**Phase 0: Foundations and hardware characterisation (7 pd)**
- Deliverables:
  - venv lock + offline wheelhouse;
  - bundle builder + manifest + world self-test;
  - re-baked overlay;
  - `limits`/`timing` modules;
  - vendored `alix_dispatch` with a hash check;
  - import-boundary test (no X1/X2);
  - decisions log (§12);
  - hardware characterisation report, using existing ALIX tools and r14 under supervision: slow stop on a running stored
    program, global speed persistence and restore, controller-SIM behaviour of pole/hand, upload throughput and parallel
    arms, J4 drive limits per arm (identity), dual-start skew.
- Exit criteria:
  - The bundle passes the self-test on a machine **without** the ALIX checkout.
  - Every negative self-test refuses.
  - FK parity is bit-identical with the repo.
  - The characterisation report is signed off, and the stop and speed-restore results are recorded as design inputs.
  - Slot range and gate strictness decided.

**Phase 1: Shell, store, registry, dashboard, regions (11 pd)**
- Deliverables:
  - copied and trimmed server scaffold, route needs, jobs, page assembly, token-protected POSTs;
  - `alix_tokens.css` / `alix_base.css` / `alix_shell.css` / `lite_shell.js` extracted with the drift fixes;
  - vendored three r128, Plotly 2.35.2 and fonts;
  - project store and provenance primitives;
  - unit registry seeded from the deck (44 arm units, MAP 13 §5.2);
  - Dashboard (Board UI) and Regions (overview, cards, presets view-only);
  - "Import recording" flow (assign a v53 drag file to a unit and answer the provenance questions);
  - V6 legacy import.
- Exit criteria:
  - Token parity, no remote assets and JS integrity tests pass.
  - The dashboard reflects the files on disk after every change.
  - The registry validator rejects duplicate or unknown IDs.
  - One non-technical operator completes "find the next untaught unit and import a recording" unaided.

**Phase 2: Canonical processor and Optimise (15 pd)**
- Deliverables:
  - `motion.pipeline` (classic and recommended methods, per-segment, ramps, rating clamp);
  - `motion.gates` (G2/G3/margin/G4 against raw), duration band, certificates;
  - `motion.spikes` with suggestions and one-click fixes;
  - recipe history (5 undo + redo, labelled, persisted);
  - JS preview port;
  - Optimise screen: segments, presets, advanced drawer, resizable panes, linked plots, comparison.
- Exit criteria:
  - Golden parity with ALIX ports within 1 mdeg.
  - Classic mode reproduces `UC/zip/02…30pct` within 1 mdeg.
  - JS preview equals Python on 20 golden cases.
  - Every derived file has a certificate.
  - Every edit makes downstream ladder rungs stale.
  - The R-A1 endpoint snap and R-A5 boundary spikes are absent on the 5 measured files from MAP 07.

**Phase 3: Offline simulation (15 pd)**
- Deliverables:
  - sim worker (warm, isolated, restartable);
  - adapter (pole/hand/bin seeding) with shim parity;
  - world composition + manifest + contact policy;
  - gate sweep (dt 0.02) and preview check;
  - viewport (robot, commode by state, walls, bin toggle and state);
  - split Simulate screen with stop-at-first-collision on the joint plots, highlight, go-to-problem and back-to-editing;
  - mini-sim in Optimise.
- Exit criteria:
  - Sweep parity with the library call.
  - All world negative tests refuse.
  - Synthetic collisions are found; intended contact is classified as contact.
  - The preview check of a 15–20 s segment returns in ≤ 3 s on the target laptop (labelled preview).
  - Results always list "what was checked".

**Phase 4: Robot agents, connection, pole/hand, Teach (12.5 pd)**
- Deliverables:
  - agent process (one connection, command loop, events, heartbeat, fail-safe stop);
  - real and emulator backends;
  - Connection screen (IP, Test, Identify, mode read-back), STOP ALL;
  - pole/hand verbs with interlocks;
  - drag-teach verbs (ported, save retry);
  - Teach Session wizard (prepare, countdown, save + `SOURCE.json`, offline replay, redo/accept, auto-advance).
- Exit criteria:
  - The emulator suite passes.
  - Hardware: 3 units taught end to end, `SOURCE.json` complete, dashboard updated.
  - The pole is refused while the arm moves.
  - A swapped IP is refused by the identity check.
  - Killing the web process during a pole move stops it.

**Phase 5: Upload, dry run, real run, live mirror (11 pd)**
- Deliverables:
  - upload verbs (one session per arm, verify, skip-unchanged with sha names, parallel arms, predicted time);
  - run verbs with `global_speed_guard`, transit %, start-gap announcement;
  - ladder gating (OFFLINE → CONTROLLER-SIM → REAL);
  - live mirror;
  - mapping of error codes and UDP frames to plot rows;
  - operator verdict → back to editing;
  - legacy runner wrapper + `.proj` parity.
- Exit criteria:
  - The hardware acceptance list items (a)–(d) and (g) pass on both arms.
  - REAL is refused without OFFLINE + CONTROLLER-SIM for the same fingerprint (an override is possible only for a missing
    CONTROLLER-SIM, with a reason).
  - Re-uploading an unchanged file is skipped.

**Phase 6: Offset & Verify (11 pd)**
- Deliverables:
  - solver with ramps, gates and taper/refuse;
  - plain-language offset UI;
  - before/after sim overlay;
  - integration with the ladder.
- Exit criteria:
  - On L_P01P02 +10 mm y: 0 failures, max adjacent step ≤ taught max + 0.5°.
  - On R2_LIDTOP +10 mm z: singular spans are detected and reported, never accepted silently.
  - Every output passes gates or is refused per span.
  - A hardware test of one accepted offset runs in dry run, then real.

**Phase 7: Combine and dual workflow (14 pd)**
- Deliverables:
  - merge engine (grouping, joins, bridges, manifest);
  - collision check of bridges and transits;
  - workflow v2 with an explicit checkpoint and v1 migration;
  - ported SAFE TIMELINE planner;
  - dual timeline (player clock, `pole.py` durations, transits, skew flags);
  - dual execution (two-phase start, handoff, checkpoint prompt, cross-arm abort).
- Exit criteria:
  - Planner parity with v53 on the V6 FINAL JSONs (7 stages).
  - No unchecked bridge or transit in any built program.
  - Hardware: one TOGETHER block and one RUN FIRST/WAIT OTHER pair run with measured skew inside the agreed window.
  - Cross-arm abort works.

**Phase 8: Hardening and release (6.5 pd, +2 for Windows)**
- Deliverables:
  - installer (wheelhouse + bundle + launcher);
  - About and versions; support-bundle export;
  - operator quick guide, engineer guide, safety notes;
  - two operator usability sessions with copy fixes;
  - full hardware acceptance run.
- Exit criteria:
  - Clean install on a fresh laptop with no ALIX checkout.
  - All acceptance items pass.
  - Operators complete teach → optimise → simulate → dry run → real for one unit without help.

---

## 11. Risks and mitigations

| # | Risk | Mitigation |
|---|---|---|
| 1 | A missing mesh, SRDF or commode scene silently empties the collision world, giving a false "clear" (`WS/package_index.py:115-121`; `TG/body_collision.py:432-435`). | Bundle with a hash manifest, a `.git` marker and a start-up self-test. Every verdict stores and asserts its world manifest. No manifest, no verdict. |
| 2 | Model gaps make an OFFLINE pass optimistic: slim link set without fingers; SRDF disables two thumb-vs-camera pairs; model bin is BinShort while the physical bin may be 40 mm taller; venue walls do not match the Lite cell; commode pose relative to the robot during teaching is unknown. | OFFLINE is never enough for REAL; CONTROLLER-SIM and a supervised REAL follow. Full-hand checks against the fixture and walls through BodyCollisionWorld. Calibrate venue and standoff in Phase 0/3. Measure the physical bin. Verdict text says "model-based". |
| 3 | STOP does not abort a running stored program (untested), or the stop signal is lost on Windows. | Phase 0 measures it. Lite uses the verified stop ladder (`DP/track.py:3109`) in-process, not signals. The hardware E-stop is always within reach and stated in the UI. Windows comes only after verification. |
| 4 | The global speed ratio persists across power cycles and has no getter. | `global_speed_guard` on every path; the agent re-asserts it after restart; acceptance item (b). |
| 5 | Controller dry run does not simulate the pole or hand, so they might move physically, or not at all, while the operator expects "nothing moves". | Phase 0 characterises it. Until then Lite skips pole/hand steps in dry run (logged) and requires them in REAL. |
| 6 | Program-slot collision with ALIX production (70–100) on shared controllers. | Declared Lite range per controller; writes outside it refused; slot names carry a hash; read-back check of name and size before every run. |
| 7 | Ported algorithms drift from ALIX's hardware-validated versions. | Golden parity tests pinned to the ALIX commit; upstream requests U1–U5; delete ports when upstream lands. |
| 8 | FCL thread-safety crashes. | Single-threaded sim worker process with automatic restart; no FCL in web or agent processes. |
| 9 | Offset IK cannot be exact at singular passages (q2, q4, q6 near 0 on taught tracks). | Detect, taper, report "max achievable", refuse spans. Never silent. R9 is planned with a research margin. |
| 10 | Dual-arm drift: open-loop TOGETHER blocks, the 1.38 s pole model vs `pole.py` waits of 2.5–3.2 s, unmeasured start skew. | Two-phase start, measured skew, windowed clearance ≥ 25 mm gate, transits serialised with the partner static (`ALIX/tools/dual_arm_flow.py:3763-3800`), cross-arm abort, timeline uses `pole.py` durations. |
| 11 | Wrong arm: a typed IP is swapped. | Identity check (J4 drive limits + hand wiggle), IP binding stored, labels always show the IP in use. |
| 12 | Unauthenticated localhost endpoints that command motion. | Loopback only, per-launch token header, REAL confirmation modal, optional supervisor PIN (D11). |
| 13 | Operators over-trust "Simulation". | Three-word mode chip, "what was checked" panel, ladder enforcement, plain-language limits. |
| 14 | V6 legacy files (rates over ratings, R10_HNGR 1.53° from the J4 limit) are uploaded as-is. | Imported read-only as LEGACY; not uploadable without re-certification through Lite gates. |
| 15 | Python environment breakage (numpy/scipy ABI, python-fcl wheels). | Dedicated venv, lock file, offline wheelhouse, preflight version checks. |
| 16 | RealMan SDK licence blocks redistribution. | Open decision. Fallback: the installer installs the SDK from the internal ALIX checkout on the target machine. |
| 17 | Data loss or a corrupt artefact. | Atomic writes, immutable raw and derived files, append-only logs, export and backup. |
| 18 | Scope creep and UI density. | Operator/engineer split, usability sessions in Phases 1 and 8, engineer features hidden by default. |

---

## 12. Open decisions (for the requester and the team)

| # | Decision | Recommendation |
|---|---|---|
| D1 | Joint-limit authority | URDF limits minus 5°. Refuse if the controller ever reports stricter limits. |
| D2 | Program-slot ranges Lite owns on each controller; is the controller shared with ALIX (70–100)? | — |
| D3 | Meaning of "bin exclusion": removed from the collision world, STOWED pose, or physically absent? | Gate verdicts use the real bin state; "exclude" is a what-if only. |
| D4 | Is the physical bin the 366 mm BinShort or the original 406 mm? | Simulate the taller one if unknown. |
| D5 | Venue and standoff of the Lite cell (wall mesh, commode pose relative to the robot) | Must be calibrated before OFFLINE verdicts count. |
| D6 | Which gates are hard vs advisory | Hard: limits, G2, G3, collision. Advisory: acceleration. G4 > 10 mm hard, 5–10 mm warn. |
| D7 | Is the controller dry run mandatory before REAL for every changed program? | Yes, with an engineer-only, reasoned override when the dry-run record is missing. |
| D8 | Policy when the browser disconnects during a REAL run | Slow stop after 5 s. |
| D9 | Target OS for v1 | Linux only; Windows later. |
| D10 | Project storage location, backup, and whether several laptops share projects | — |
| D11 | Who may switch to REAL | Supervisor PIN, or operator? |
| D12 | Hand counts → radians scale | Confirm `hand_units.py` (hardware authority) over the `sim_bridge` table. |
| D13 | Simulation clock and display bands | Confirm 5.5545 ms nominal for simulation and ±6 % bands for display. |
| D14 | Upstream requests U1–U5 | Accept them, or Lite keeps ports with parity tests long-term. |
| D15 | Merge semantics | Merge only same pole/hand/fixture-state segments; everything else via workflow. |
| D16 | Runner v14 after Phase 5 | Retire as an executor; keep as a test oracle. |
| D17 | CANFD streaming (no upload) | Out of scope for v1; engineering lane only. |
| D18 | Cleaning time per unit (the deck gives none) | Process owner sets it; V6 durations are defaults. |
| D19 | Teaching order for one operator | All LEFT units of a fixture phase, then RIGHT, then the state change (MAP 13 §7.2), or deck order? |
| D20 | Units allocated differently in V6 (other-arm and extra segments) | Re-teach to the deck allocation, or adopt V6's split? |
