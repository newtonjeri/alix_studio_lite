# ALIX Studio Lite — Implementation Plan and Approach

**Status: Version 1.6, 13 October 2026 (current).**

> **Update (D33–D36, 13 October 2026):** the designer's V19 page is now the primary UI (the Lite UI) and the backend is built around it, in the order of its handoff (`UI_CONTRACT.md`, last section). The earlier UI is kept as Legacy Lite. Where this plan describes screens, read them as the matching V19 screens.

| | |
|---|---|
| Version | 1.6 — revised to the team's decisions of 13 October 2026 (F1–F8, D3, D15, D21, D23–D27, D33–D36) |
| Date | 13 October 2026 |
| Readers | The engineers who will build Lite; the author of the requirement inputs; the ALIX owner |
| Location | Final version in `LITE/alix_studio_lite/docs/`, beside the code in `LITE/alix_studio_lite/` (F7) |
| Companion document | `ALIX_Studio_Lite_Requirements_Revised.md` (same folder): the revised requirements, with findings and a borrow verdict for each requirement |
| Built from | The verified requirement analyses behind the companion document (C1–C8 below), 13 source evidence maps, three architecture proposals (reuse-first, operator-first, safety-first) scored by two independent judges, read-only spot checks of the code, and two follow-up research notes of 13 October 2026 (optimisers; E-stop) |

**Path aliases used in citations**

| Alias | Absolute path |
|---|---|
| `ALIX/` | `/home/newtonjeri/alix_ws/src/alix/` (branch `feature/alix_studio`, HEAD `842c0e59`, 2026-10-12) |
| `TG/`, `WE/` | `ALIX/core/python/alix_taskgen/src/alix_taskgen/` (ALIX Studio); `TG/web_editor/` |
| `WS/` | `ALIX/core/python/alix_workspace/src/alix_workspace/` |
| `DP/` | `ALIX/core/python/alix_dispatch/src/alix_dispatch/` |
| `TT`, `CT`, `DAV` | `ALIX/tools/taught_tracks.py`, `ALIX/tools/cleaning_tracks.py`, `ALIX/tools/dual_arm_verdicts.py` |
| `SCENE/` | `ALIX/ros/orchestration/alix_tasks/config/`: `commode_cleaning/commode_c/scene.yaml` (the commode C fixture) and `scene/washroom_*.yaml` (the venue wall files) |
| `EXP/` | `ALIX/experiments/exp01_teach_trajectory_reverse_engineering/src/` (the TOPP-RA chain: `emit_toppra_rung.py`, `local_speed.py`) |
| `LITE/` | `/home/newtonjeri/alix_studio_lite_ws/` |
| `LITE/alix_studio_lite/` | Lite's code and, in `LITE/alix_studio_lite/docs/`, its documents (F7). Written out in full, because `LS` stands for `local_speed.py` in §4, §5 and §7 |
| `UC/` | `LITE/upload_channel/` |
| `V53`, `R14` | `UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py` (UI v53); `UC/rm_upload_fast_v14_safe_transit.py` (runner v14) |
| `HTML` | `LITE/Alix_Studio_Lite_Trajectory_V2.html` |
| `V6/` | `LITE/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/` |
| `C1`…`C8` | The verified requirement analyses summarised in the companion document: C1 = R1, R7 · C2 = R2, R6, R11, R12, INT-TRAJ · C3 = R3, R5, R10 · C4 = R4, R13 · C5 = R8 · C6 = R9, R14 · C7 = X1, X2 and the NFRs · C8 = INT-CTRL. References such as "C1 §1.8.2" and "source map 04" point to the analysis notes behind both documents, kept in `analysis/`. |
| `F1`…`F8` | The team's decisions of 13 October 2026: F1 target computer, F2 optimisers and target time, F3 commode C, F4 IO E-stop, F5 scene and walls, F6 upload slots, F7 code and docs location, F8 effort |
| "optimiser research", "E-stop research" | The two research notes of 13 October 2026 behind F2 and F4, kept in `analysis/` with the C1–C8 notes. The RealMan manual references they use (HW, JSON, PY, CPP, MB, REL, TP, DEMO, CN, RRI) are quoted here as written there. |

**Effort scale.** Lite is to be built by an AI coding agent (Claude Opus 5.5) working with an engineer who reviews the code, runs the lab sessions and signs off (F8). The schedule is therefore given in **agent working days** for the build and **lab days** for supervised time on the robot (§7). Work items carry a relative **size**: S (small), M, L or XL. A conventional single-developer sizing in person-days is kept only as a reference in §7.

---

## Summary

- **What Lite is.** A guided, offline application for non-technical operators. It takes slow, stop-filled drag-and-teach recordings region by region (joint-space `.txt`). It optimises them, checks them in a dual-arm 3D simulation of the commode C cell, tests them on the two RealMan RM75 arms and uploads them. It has no geometric path generation (X1) and no C-space or Workspace features (X2).
- **Architecture decision.**
  - **Location (F7):** Lite is a **standalone application in its own folder, `LITE/alix_studio_lite/`**, with its documents in `LITE/alix_studio_lite/docs/`. It is not a mode of ALIX Studio and not a package inside the ALIX monorepo. It imports the ALIX libraries from its **vendored copy** in `vendor/alix` (decided 13 Oct: everything Lite needs lives inside `alix_studio_lite`; `tools/vendor_from_alix.py` records the ALIX commit and the sha256 of every file), and its parity tests run against the pinned commit.
  - **Target computers (F1):** the **operator PC (x86-64 Linux)** now; the **robot's Jetson AGX Orin (aarch64, Ubuntu 24.04)** in production. Both are supported from one code base, and an aarch64 production build is required (§4.8).
  - **Base:** Lite follows the **operator-first** proposal: a local web app in a chromeless browser window, served by a small loopback Python server, with one warm simulation worker, one robot-executor seam and a no-root installer with update and rollback.
  - **Grafted from the safety-first proposal:** a fail-closed collision world (asset bundle in repo layout, world manifest, start-up self-test), an intended-contact policy, an exact-bytes validation ladder built on `DP/ladder.py`, the verified stop ladder as a fallback, and an arm-identity check.
  - **Grafted from the reuse-max proposal:** ALIX code is imported, not copied (`alix_workspace`, the whitelisted light `alix_taskgen` modules, `alix_dispatch`), through small upstream seams. Also a zero-code Phase-0 harness on the unchanged Studio, drift tests on copied UI code, one-time confirmation tokens and a process interlock.
- **Optimisers (F2).** Two optimisers behind one interface:
  - **Default:** the trajectory prototype's algorithm (trim idle → smooth → scale, `HTML:1438-1616, 1676-1724`). It stays the default, with three small fixes for its measured defects (end-point snap, speed spikes at segment boundaries, pad path bending) and with hard limit checks added;
  - **Time-optimal:** TOPP-RA (`toppra` 0.6.10), through the chain ALIX has already proven on hardware (`ALIX/docs/motion/TAUGHT_CANFD_PIPELINE.md:17-40`).
  - The time the operator sets for a region is the **target**. The optimiser meets it to the tick when the limits allow. Otherwise it returns the **closest (minimum) time that obeys the limits** and names the limit that binds (joint, speed or acceleration, and time window).
- **Scene and walls (F5, F3).** The cell comes from the URDF and the ALIX commode C scene (`SCENE/commode_cleaning/commode_c/scene.yaml`): commode pose and articulation, bin, UGV and poles. No cell-calibration project is needed. The one gap is **wall collision**, which today's checks do not enforce. Lite enforces it against a single cubicle, 1400 mm wide by default with 800 mm selectable (decision D21), loaded by `WS/plan_scene_validation.py:117, 179`. ALIX has an 800 mm cubicle mesh but no 1400 mm one (its 1400 mm venue is the whole washroom, 4.3 × 8.1 m), so Lite adds a 1400 mm cubicle built like ALIX's 800 and 1000 mm ones. Verifying it once with known-good recordings is a small setup step.
- **Safety stop (F4).** The **IO-wired emergency-stop buttons** on the left and right arm controllers are the primary safety stop. They stop the robot without cutting power; in the ALIX field test the hand kept working during an E-stop, so Lite freezes it (E-stop research). Lite shows each arm's E-stop state, blocks motion on both arms while either stop is engaged or its state is unknown, marks interrupted runs as INTERRUPTED (never COMPLETE) and guides recovery. Lite's own STOP is labelled a software convenience.
- **Uploads (F6).** As in the control program (UI v53, runner v14), Lite uploads to **any program number 1–100** and overwrites an existing program after a confirmation that shows what is in the slot (`V53:9-11, 4279, 4469-4488`; `R14:324`). Lite may warn, for example "this slot holds a program from the V6 workflow", but never blocks a slot.
- **Executor.**
  - **Release 1:** runner v14 executes all robot actions, sha256-pinned, started through a small Lite launch wrapper. The one change is a small E-stop patch (F4: the runner reports an E-stop as an interruption, not as "RUN COMPLETE"), pinned by its own sha256 (§3.3).
  - **Release 2:** per-arm agents on `alix_dispatch` take over, but only after passing a hardware acceptance list. Runner v14 then remains as the fallback and as a test oracle.
  - **Dual-arm flows** keep running from v53 until Lite's dual coordinator passes its hardware tests.
- **Releases.**
  - **Release 1 (P0–P7):** the single-arm operator loop, end to end. Covers R1–R7, R10 (split view and bin control), R11, R12, R13 and INT-TRAJ, plus INT-CTRL for single arms.
  - **Release 2 (P8–P11):** robot agents, merging and dual-arm workflows (R8, R10 dual timeline), and offsets (R9, R14).
- **Schedule (F8, §7).** Much of the work already exists in ALIX Studio, the ALIX libraries and this workspace, and Lite is built by an AI coding agent with an engineer reviewing. Since 13 Oct 2026 the build starts from the GUI (D27). The schedule is:
  - **UI/UX first:** the operator workflow screens on the ALIX Studio theme, working against the existing optimiser and stand-ins for the robot;
  - **then the backend:** optimiser golden tests and integration, 3D check with walls, robot I/O with the IO3 E-stop monitor, and the server and API. The backend is on hold at its checkpoint (§7.1, [`docs/STATUS.md`](STATUS.md));
  - **a runnable core loop** (import or teach a recording, optimise with either optimiser to a target time, 3D check with walls, upload to any slot 1–100 with the overwrite confirmation and run it in SIM mode) in about **2 agent working days** of build, once the screens and the backend are in place;
  - **Release 1:** about **5 agent days** (range 4–6), plus about **3.5 robot lab days**;
  - **Release 2:** about **3 agent days** (range 2–4), plus about **1.5 lab days** and an operator field trial.
  - Lab time, the E-stop drill and the operators' sign-off, not coding, set the calendar. A conventional single-developer sizing (about 280 person-days) is kept only as a reference in §7.
- **First steps.** The target computer, the slot policy, the code location and the safety stop are now decided (F1, F4, F6, F7), and so are the time base (D26) and the delivery order (D27). Still to do first:
  - build the operator workflow screens (UI/UX first, D27);
  - sign off the region IDs and the per-region target times for commode C (D3, D4), and agree the upstream seams (D14);
  - on a supervised lab day: run the E-stop drill cases that gate REAL use, test whether a software stop ends a running stored program (G4), check SIM-mode pole and hand behaviour and measure timing;
  - select the venue wall file for the test cell and verify it by replaying known-good recordings;
  - build `toppra` 0.6.10 for aarch64 and time one solve on the Orin;
  - prove the asset bundle and its fail-closed self-test on a machine with no ALIX checkout;
  - keep collecting the untaught regions with v53 and a teach sheet, for import into Lite later (§12).

---

## 1. Purpose and inputs

### 1.1 Purpose

This plan turns the revised requirements into a buildable design and schedule. It records:
- the architecture decision and why it was taken;
- what is reused from ALIX Studio, the ALIX libraries, the trajectory prototype and the control application, and how (import, copy, port or vendor);
- the operator and engineer workflows;
- a phased plan whose effort is consistent with the verified requirement analyses;
- the verification and safety approach;
- the risks;
- the decisions the requester and the team still need to take.

### 1.2 Inputs

| Input | What it contributed |
|---|---|
| `LITE/alix_studio_lite/docs/reference/ALIX_Studio_Lite_requirement_inputs.txt` | R1–R14 and the exclusions X1 (geometric path generation and editing) and X2 (C-space and Workspace features) |
| `LITE/RealMan Dual-Arm Operations UI.txt` | Feature notes for the control app. They state UI v50 / runner v13 and are out of date (C8 §1) |
| `LITE/Toilet_regions_V1.pptx` | 25 slides, 22 dual-arm task steps, **42 teachable units** (22 LEFT, 20 RIGHT; requirements, Appendix A). The deck describes **commode C**, so Lite's fixture is ALIX's `commode_c` (F3) |
| `V6/` | The merged dual-arm production package (programs 71–83), used here as legacy reference data |
| `HTML` | The trajectory-optimisation prototype, to be integrated (INT-TRAJ) |
| `UC/` | The control application (Tkinter UI, runner and helpers), to be integrated (INT-CTRL) |
| `ALIX/` | ALIX Studio (`TG/`), the ROS-free libraries (`WS/`, `DP/`, the emulator) and the hardware-validated taught-track tools |
| C1–C8 | The verified requirement analyses. C1 covers R1 and R7; C2 R2, R6, R11, R12 and INT-TRAJ; C3 R3, R5 and R10; C4 R4 and R13; C5 R8; C6 R9 and R14; C7 X1, X2 and the NFRs (UX, theme, safety, deployment, data); C8 INT-CTRL |
| Three architecture proposals (reuse-first, operator-first, safety-first) | Alternative architectures, scored in §3.2 |
| Judges' verdicts | Scores, grafts and concerns for the three proposals (§3.2, §3.5) |
| Team decisions F1–F8 (13 October 2026) | Target computers, the two optimisers and the target-time rule, commode C, the IO E-stop, the scene and walls, the upload-slot policy, the code and docs location, and the agent-driven schedule |
| Optimiser and E-stop research (13 October 2026) | The measured behaviour of the prototype and of ALIX's TOPP-RA chain (F2); how the IO E-stop works, how to detect it and how to recover (F4) |

### 1.3 What the existing programs and files do (high level)

This section answers the user's request for a high-level understanding of the Python programs and the related back-end and front-end files. It is **not** a bug audit. Issues appear only where they change the plan, one line each.

#### 1.3.1 The control application (`UC/`)

| File | What it does | Status |
|---|---|---|
| `rm_trajectory_ui_v53_safe_lid_checkpoint.py` (11,581 lines, Tkinter) | **The operator console**, titled "RealMan // Dual-Arm Operations Console".<br>**Page 01** has two arm panels, LEFT 192.168.1.10 and RIGHT 192.168.1.103. Each panel has these tabs (`V53:2149-2166`):<br>• **RUN:** upload a `.txt` into any controller program slot 1–100, replacing an existing program after a confirmation, or batch-upload several files with one Program ID each; list the slots; run stored programs by ID or name (`V53:9-11, 761-950, 4279, 4469-4488`);<br>• **POINTS:** controller global waypoints;<br>• **DRAG TEACH:** record, save and play back;<br>• **HAND / POLE:** six hand channels 0–1000 with GRIP and ALL OPEN presets, and the pole at 0–315 mm;<br>• **TELEMETRY:** the runner's text log.<br>Each panel also has SIM / REAL / CHECK buttons for the controller's run mode.<br>**Page 02** builds dual sequences: per-arm lists of program, pole, hand, wait and global-point steps; the dual rules TOGETHER, RUN FIRST and WAIT OTHER; the "SAFE TIMELINE" planner; and a manual lid checkpoint.<br>**It never talks to the robot itself.** Every button starts the runner as a separate process (`V53:4255-4262`) and reads the lines it prints. | **Current UI** |
| `rm_upload_fast_v14_safe_transit.py` (2,702 lines) | **The runner.** A command-line program, not a network server: one process opens one TCP connection to one controller, does one job and exits.<br>Its jobs:<br>• upload a `.txt` as a CRLF `.proj`, store only, and verify it by reading back the stored size (`R14:76-175`);<br>• list the slots;<br>• run a slot or a sequence, watching run state and error codes (`R14:189-269`);<br>• run a Page-02 workflow over one connection (`R14:1668-2277`);<br>• drag teach: record, save and play back (`R14:912-1210`);<br>• move the pole and the hand by hand (`R14:804-909`);<br>• read and set the controller's SIM/REAL run mode (`R14:1213-1262`);<br>• move to global points.<br>Lines 1–268 are ALIX's `tools/rm_upload.py`. "Fast" means fast switching between stored programs, not a faster upload. | **Current runner.** The `_v52support` and `_v53support` files are byte-identical copies (md5 `a0b285e6…`) |
| `hand.py`, `pole.py` | `hand.py` drives the Inspire hand over end-port Modbus RTU: six channels, 1000 = open.<br>`pole.py` drives the lift through the SDK lift API, with a measured travel-time model and a 5 mm tolerance.<br>Both are trimmed copies of `DP/peripherals.py`. | Helpers the runner loads |
| `usage.py` | A small command-line tool for the pole and hand: read, move, stop, `prestroke` and `stop`. It states the rule "the pole moves only with the arm still; never move pole or hand with the pad on a surface" (`UC/usage.py:25`). | Reference |
| `rm_upload_fast_v13_overall_speed.py`; UIs v48, v51, v52 | Older versions. v48 is three versions older than v53, and v53 contains all of it. | Superseded |
| `py programs/`, `upload/`, `upload.zip` | History: 28 UIs (v8–v48), runners v4–v12 and the original uploader. | Archive |
| `LEFT/RIGHT_V6_70SERIES_FINAL.json`; `zip/`, `plan_txt_files/`; `README_COMMANDS.md` | The production dual workflow (`realman_workflow_sequence_v1`, programs 71–83); sample lid-top tracks; usage notes for the plain uploader. The robot PC runs Linux, user `alai-labs`. | Data and reference |

**Which version is current: UI v53 with runner v14.**
- `V53:45-46` declares `UI_PROGRAM_VERSION = "v53"` and `EXPECTED_RUNNER_VERSION = "v14"`.
- On the V6 production recipe, v48 and v51 reject it, and v52 plans it without the lid pause. Only v53 plans it, in 7 stages with the manual lid checkpoint. C8 §1 re-ran all four planners to establish this.

**How the pieces work at run time** (C8 §2):

```
Operator ─► UI v53 (Tk) ── for every action: python -u <runner> --sdk <SDK> --side left|right --yes <verb …>
                 │            (one process = one TCP connection = one job; it imports hand.py / pole.py)
                 ▼
           runner v14 ──RealMan SDK──► LEFT 192.168.1.10:8080  |  RIGHT 192.168.1.103:8080
UI ◄── printed progress lines + exit code (the UI's only status channel)
STOP       = SIGINT to the runner's process group → KeyboardInterrupt → rm_set_arm_slow_stop()
Dual start = two launches released by threading.Barrier(2) just before Popen (software sync only)
```

**Issues that change the plan** (one line each):
1. **No IP entry.** The UI shows the IPs as labels and never passes `--ip`. The runner accepts `--ip`/`--port` (`R14:2285-2286, 2347`), so Lite passes them.
2. **The runner version is not enforced.** If v14 is missing, the UI silently falls back to v13, which runs approach moves at 100 % (`V53:6889-6909`). Lite pins the runner by sha256.
3. **STOP is SIGINT only, and the IO E-stop is not seen.** The runner installs no signal handler (`grep signal R14` finds 0 hits). Whether a slow stop ends a *running stored program* has never been tested ("G4"); Lite tests it in P0. The safety stop is the **IO-wired E-stop button** on each arm controller (F4), but the runner watches only run state and error codes (`R14:122-138, 219-271, 641-700`). An E-stop during a run would therefore very likely be reported as "RUN COMPLETE, no error" with exit code 0, and a sequence would go on to its next step (E-stop research; inferred, to be confirmed in the drill). Lite adds E-stop detection and a small runner patch (§3.3).
4. **SIM mode and the pole and hand.** The controller's SIM mode does not execute pole or hand commands, but the runner still sends them (C8 §4). Lite strips those steps in dry runs.
5. **Slot policy.** The control program uploads to any Program ID 1–100 and replaces an existing program after a confirmation that lists it (`V53:898, 4279, 4469-4488`; the runner checks 1–100, `R14:324`). Lite keeps this policy (F6): any slot 1–100, overwrite after confirmation, and a warning, never a block, when the slot holds a program from another workflow (for example V6's 71–83, inside ALIX's 70–100 production range).
6. **The default SDK path is another machine's home directory** (`V53:48-51`). Lite always passes `--sdk`.
7. **In a TOGETHER stage, one arm's failure does not stop the other** (`V53:10273-10300`). Lite adds cross-arm abort.
8. **Multi Run and Page-01 Load Sequence raise `TypeError`.** Four helpers lack `@staticmethod` (`V53:4853, 4876, 4885, 5000`). Lite does not port these paths, so this matters only while v53 stays in service as the fallback.

#### 1.3.2 ALIX Studio: back end and front end (`TG/`)

| File | What it does |
|---|---|
| `TG/server.py` (6,566 lines) | **A local web server** built on the standard library (`ThreadingHTTPServer`). It has 64 JSON routes (24 GET, 40 POST), a per-route "needs" table, one global generation lock and a start / poll / cancel job model.<br>**It serves:** robot meshes and FK frames; dual-arm collision sweeps of two MoveIt plan JSONs; SRDF pole presets; and a read-only view of the in-process emulator. It also runs the geometric generator (X1) and the Workspace and C-space analyses (X2).<br>**It never commands the robot.** Start-up takes 30–60 s because it imports the generator stack. It binds to 127.0.0.1 and warns that any other binding is unauthenticated (`TG/server.py:6539-6544`). |
| `WE/template.html` (953 lines), `WE/editor.js` (19,325), `WE/editor.css` (2,166) | **One offline single-page app** with a three.js 3D viewport and seven tabs:<br>• Regions (mesh painting for the generator, X1);<br>• Board (region × arm table);<br>• Workbench (X1);<br>• Dual-Arm Collisions (sweep timeline, red links, Previous/Next, log);<br>• Schedule & Run (an offline preview of the orchestrator schedule);<br>• Workspace (X2);<br>• Gizmo.<br>It also has a Simulate view, and these parts:<br>• a test-pinned design-token theme (brand `#2F96EE`, light by default; dark tokens exist but nothing switches them on);<br>• resizable docks and draggable dialogs;<br>• one global undo stack (80 entries, unlabelled);<br>• a feedback toolkit: status line, toast, confirm modal with chips, gate chips, tooltips, searchable help.<br>It has no guided flow and no joint-value plots, and its plots cannot be resized. |
| `WE/vendor/` | three.js r128, OrbitControls, TransformControls, Plotly 2.35.2 (the version the trajectory HTML loads from a CDN), js-yaml |
| `ALIX/design/_tokens.css` | The single-source design tokens that the Studio's parity test checks |

#### 1.3.3 The trajectory prototype (`HTML`, 2,096 lines)

**What it does.** It is a single static page:
- it loads **one** taught `.txt`;
- it removes idle stretches with a rolling-mean rule;
- it smooths with a 101-point moving mean;
- it speeds up chosen row windows by linear resampling;
- it suggests speed changes from equal-length chunks;
- it compares the result with the original and downloads it with a report.

**What it lacks.** Undo, rate and limit checks, a 3D view and a dual-arm view. It loads Plotly from a CDN (`HTML:7`).

**Measured issues** (C2; re-measured for F2 in the optimiser research, which measures the pad change in both directions at `L_glove_frame_2`):
- the default 101-point smoothing moves the cleaning pad 9–10 mm (C2), or 10.6–20.9 mm when measured both ways;
- the smoothing snaps the end points: the first or last line jumps 546–1005 millidegrees in one tick (`HTML:1513-1514, 1721-1723`);
- segment boundaries cause acceleration spikes of 440–880 °/s² (C2) or 640–919 °/s² (`HTML:1568-1616`).

**Its role in Lite (F2).** It becomes Lite's **default optimiser**, with three small fixes (odd-reflection padding at the window edges, a 0.25 s speed blend at each segment boundary, a default window of 41 points with a hard path-change check) and the same hard limit checks as TOPP-RA. With the fixes, the pad stays 2.9–4.5 mm from the recording and the boundary spikes drop to 200–280 °/s² (optimiser research).

#### 1.3.4 ALIX libraries Lite builds on (all ROS-free)

| Library | What Lite uses it for |
|---|---|
| `alix_workspace` (`WS/`) | FK (`UrdfModel.fk_batch`, 2.5 µs per point; matches the controller); Jacobian and IK; the collision model; the dual-arm sweep and plan frames; the wall checker that loads a venue scene file (`load_wall_manager`, `PlanWallChecker`, `WS/plan_scene_validation.py:117, 179`), which Lite uses to enforce the walls (F5); SRDF states; the package index |
| `alix_taskgen` light modules | `body_collision` is the full world: commode body, lid and seat in each lid/seat state, walls, bin and UGV. The commode C fixture comes from `SCENE/commode_cleaning/commode_c/scene.yaml` (F3, F5). Also used: `scene_pose`, `fixture_parts`, `transforms`, `paths`, `constants`, `singularity`, `glove_frames`, `approach_transit`, `schedule_guards`. `alix_taskgen/__init__.py` holds only `__version__`, and these modules import only numpy, scipy, yaml, trimesh and each other (checked for this plan) |
| `alix_dispatch` (`DP/`) | Controller I/O with semantics proven on ALIX hardware:<br>• connecting and moving programs: `open_arm`, `send_project`, `program_list`, `preflight_slots`, `run_program`;<br>• stopping: the verified stop ladder `stop_program` (`DP/track.py:3109`) and the stop-policy probe `probe_stop_policy` (`:3273`);<br>• safety state: `global_speed_guard`, run-mode read-back, and `ladder` (a controller SIM pass licenses REAL only for the identical program, `DP/ladder.py:63-145`);<br>• devices and capture: pole and hand, UDP capture and the ordered matcher;<br>• testing: the emulated program channel.<br>It has no drag-teach verbs and no `pyproject.toml` |
| `alix_emulator` | An in-process RealMan SDK stand-in for CI. It has no drag teach and fires no program-finish events |
| `ALIX/tools/`, `EXP/` | The hardware-validated taught-track maths and gates (`TT`, `CT`); the TOPP-RA chain for taught paths, proven on hardware (`TT:891-951`, `EXP/local_speed.py:139-169, 282-324`, `EXP/emit_toppra_rung.py:306-330`; `ALIX/docs/motion/TAUGHT_CANFD_PIPELINE.md:17-40`). It lives under `experiments/`, so Lite calls it through one adapter module and keeps its own small `retime_toppra` with a parity test (optimiser research); the `.txt`→plan shim with pole and hand seeding (`DAV:501-731`); `concatenate` (`ALIX/tools/plan_min_time.py:142`); the layout audit (`ALIX/tools/layout_audit.py`); the overlay bake (`ALIX/tools/meshes/bake_visual_overlay.py`) |

#### 1.3.5 Other inputs

- **Regions deck.** It describes commode C (F3). No cleaning times and no pole or hand values. About half the labels have no code, so every unit gets a code in the deck's own format, `<L|R>-<R|T><NN>`, with parts written as in the deck (requirements, Appendix A). About 16–17 units have nothing taught (requirements, Appendix A).
- **V6 package.**
  - Its programs are merged and time-stretched.
  - Its minimum-jerk bridges and the controller transits between programs (up to 64.6°) are not collision-checked.
  - Two of its 15 files fail ALIX's band-limited rate gate, and `R10_HNGR` comes within 1.53° of the J4 limit.
  - `DUAL_FLOW_TIMELINE_V6.json` is listed but missing.
  - Lite uses it as legacy reference data only.
- **Operations notes.** Accurate for Page 01; out of date for the Page-02 dual rules and the v51–v53 additions.

### 1.4 Terms used in this plan

| Term | Meaning |
|---|---|
| **Region** (unit) | One arm's teachable unit from the deck, for example `L-R05` SEAT TOP PART 1 (file key `L-R05_PART1`). The deck has 42; 34 are in the first release. |
| **Segment** | A time window inside one region's recording: the "sub-region" of R2. The prototype calls these "scaling regions". |
| **Program** | One controller slot holding one `.txt` |
| **Link move** | Motion that Lite generates inside a merged program (R8) |
| **Transit** | The controller's own joint move to a program's first point. The controller does not collision-check it. |
| **Simulation** | Lite's own 3D check. No robot is involved. |
| **Dry run on the robot** | The controller's SIM run mode. The arms do not move, and Lite skips pole and hand steps. |
| **Real robot** | The controller's REAL mode. The arms move. |
| **Ladder** | The rungs GATES → 3D CHECK → DRY RUN → REAL, each recorded against one fingerprint |
| **Fingerprint** | `sha256(.proj bytes) : sha256(context)`. The context is pole, hand, bin state, fixture state, scene (URDF, commode_c scene and venue file), partner arm, transit % and bundle. |
| **Scene** | The collision world Lite checks against: the URDF (arms, hands, poles, UGV), the bin geometry (URDF, `ALIX/core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf:188-359`; bin states from the SRDF `BIN_*` group states), the ALIX commode C scene (commode pose and articulation; `SCENE/commode_cleaning/commode_c/scene.yaml`) and the **selected venue file** for the walls (`SCENE/scene/washroom_*.yaml`). All come from the pinned ALIX commit; selecting and verifying the venue file is a setup step, not a calibration project (F5). |
| **Target time** | The duration the operator sets for a region (seeded from the region catalogue). The optimiser meets it when the limits allow; otherwise it delivers the closest time that obeys the limits and names the binding limit (F2). |
| **Default optimiser** | The prototype's trim idle → smooth → scale algorithm (`HTML:1438-1616, 1676-1724`) with three fixes and Lite's hard limit checks (F2) |
| **Time-optimal optimiser (TOPP-RA)** | ALIX's hardware-proven chain: smoothing, pause removal, arc-length smoothing, then a TOPP-RA solve (`toppra` 0.6.10) at the joint speed and acceleration limits (F2). Offered only where `toppra` imports. |
| **IO E-stop** | The emergency-stop buttons wired to an IO input on each arm controller (an IO set to mode 5, "input emergency stop function multiplexing"; E-stop research). They stop the robot without cutting power and are the primary safety stop (F4); the hand is not stopped by them in the ALIX field test, so Lite freezes it (`ALIX/docs/TEST_PROCEDURE.md:453`). |
| **World manifest** | The record, kept with every verdict, of exactly what the collision world contained |
| **Contact policy** | The rule that separates intended pad contact from collision |
| **Stage A / Stage B executor** | Runner v14 through the launch wrapper (Release 1) / per-arm agents on `alix_dispatch` (Release 2) |
| **Gates G1–G5** | ALIX's taught-track checks: G1 predicted joint-limit margin, G2 band-limited speed against the joint ratings, G3 step ≤ 10° per line, G4 pad deviation from the raw recording, G5 predicted executed path departure |
| **Spike detectors SD1–SD6** | R12 checks on the output file at the 5.0 ms tick: SD1 too fast, SD2 step fault, SD3 isolated jump, SD4 sudden speed change, SD5 near a joint limit (G1 rule), SD6 hesitation |
| **Seams S1–S7** | Small, default-preserving changes proposed upstream in ALIX (§5) |
| **Decisions D1–D27** | Open decisions for the requester and the team (§11) |

---

## 2. Guiding principles

1. **Operators first.**
   - Each region follows one guided loop: Prepare → Teach → Optimise → Check in 3D → Test on robot → Send to robot.
   - Screens use plain words, presets and one primary button per screen.
   - Engineer parameters sit behind an Engineer switch.
2. **Only checked bytes reach a robot.**
   - Every motion goes up a ladder keyed to the exact bytes and their context.
   - Any edit makes the later rungs stale ("stale is not pass", `TG/schedule_guards.py:13-17`).
3. **Fail closed.**
   - A missing mesh, SRDF, commode scene, venue wall file, run mode, E-stop reading, pin or record refuses the action and says why.
   - "Not checked" is never shown as "pass".
4. **One source of truth for anything that decides whether metal moves.**
   - Import, from one pinned ALIX commit: the kinematics, the collision world, the controller semantics and the ladder.
   - Copy only presentation code, with a provenance header and a drift test.
   - Port only with golden parity tests and an upstream request, and delete the port when upstream lands.
5. **One command source per arm.** This is ALIX's ruling: "Two TCP clients = two command sources — not advisable. ACCEPTED AS AXIOM." (`ALIX/docs/motion/DRIVER_TRACK_CHANNEL_ARCHITECTURE.md:220-233`). Lite never holds two concurrent connections to one controller in normal operation, and it refuses to command an arm that another tool may be driving.
6. **Keep what works until its replacement is proven on hardware.**
   - Runner v14 runs as the first executor, unchanged except for a small E-stop patch that is pinned by its own sha256 (F4, §3.3).
   - v53 stays installed for dual flows.
   - Lite reads and writes `realman_workflow_sequence_v1`.
7. **Measure before relying.** These facts are measured in P0 and recorded as design inputs: software-stop behaviour, the IO E-stop's behaviour and state signals (drill cases), SIM-mode pole and hand behaviour, upload timing and the dual-start skew. The cell geometry is not measured: it comes from the URDF and the commode C scene, and the venue wall file chosen for the test cell is verified by replaying known-good recordings (F5).
8. **Raw is immutable; edits are recipes; everything has provenance.** Raw recordings are never changed. Outputs are versioned. Every file, pass and run names the sha256 of what it used.
9. **Honest results.**
   - Every verdict lists what was checked and what was not.
   - Program durations are (lines − 1) × 5 ms (D26); clearances are shown as bands.
   - A simulation pass means "nothing found in the model", not "safe".
10. **The IO-wired E-stop buttons are the safety stop.** The buttons wired to an IO input on the left and right arm controllers stop the robot without cutting power (F4); because the hand kept working during an E-stop in the ALIX field test, Lite freezes it (`ALIX/docs/TEST_PROCEDURE.md:453`). Lite shows their state per arm, treats an unknown state as pressed, and never resumes automatically after one. Lite's STOP is labelled "STOP (software)": a convenience, not a safety function.
11. **Scope is enforced by tests.** X1/X2 code is never imported, routed or shipped.
12. **Offline and self-contained.** The operator PC (x86-64) now, and the Jetson AGX Orin (aarch64) in production, need no internet, no ROS and no ALIX checkout at run time (F1). Lite's code lives in `LITE/alix_studio_lite/`; each release vendors one ALIX commit into `alix_studio_lite/vendor/` (F7).

---

## 3. Architecture options considered and decision

### 3.1 Options

| Option | Shape | Strengths | Why it was not taken as is |
|---|---|---|---|
| **A. Lite as a mode of ALIX Studio** | Add Lite routes and tabs to `TG/server.py` and `WE/editor.js` | Maximum code sharing | Studio takes 30–60 s to start because it imports the generator (source map 04), and it resolves files relative to the repo. Its collision handler ignores the commode and drops `joint_timeline` (`TG/server.py:4335-4366`). It has test-pinned UI rules (no numbered tabs) and no execution channel. All three proposals reject this option. It survives only as the **zero-code Phase-0 harness**. |
| **B. Reuse-max** | A sibling package in the ALIX monorepo that imports the ROS-free libraries, including the light `alix_taskgen` modules. It copies about 400 lines of server code and about 1,900 lines of JS/CSS, with drift tests. Runner v14 is vendored, a `DispatchBackend` follows later, and upstream seams S1–S6 are added. Assets ship as a sparse repo snapshot. | Strongest reuse and maintainability. The Phase-0 harness. One-time tokens and a process interlock. | It assumed `ALIX_REPO_ROOT` fixes asset resolution, but `PackageIndex` finds the repo by walking up to a `.git` directory and never reads it (`WS/package_index.py:27-37`). It has no contact policy and no fail-closed world. Stop stays SIGINT-only until late. |
| **C. Operator-first** | A standalone app in a chromeless window, with one guided loop per region and three named modes. Runner v14 is wrapped unchanged. A relocatable CPython bundle installs without root and supports update and rollback. UI start-up target is 3 s. Usability tests at three points. | Best operator experience and deployability. Teaching on hardware starts early. Lowest delivery risk (judge 1). | It copies the hardware-fixed commode-pose code (G37/G38). It adds a second FK in JS. It has an optional runner patch that breaks the sha pin. It has no contact policy and no world manifest, and its Phase 0 is optimistic. |
| **D. Safety-first** | Four processes: web, simulation, and one agent per arm on `alix_dispatch` from Phase 4. An exact-bytes ladder, a fail-closed bundle and self-test, a contact policy, an arm-identity check and the verified stop ladder. | Best safety and validation fidelity | The most new code on the robot path, early, replacing the field-proven runner. Its OFFLINE gate cannot be overridden and could block all real testing until the cell is calibrated. It has the most operator friction. |

### 3.2 Judges' scores

| Lens | Judge | Coverage | Reuse / effort | Operator UX | Safety | Maintainability | Deployability | Delivery risk | **Total** |
|---|---|---|---|---|---|---|---|---|---|
| reuse_max | 1 | 8 | 9 | 7 | 6 | 8 | 6 | 7 | **51** |
| reuse_max | 2 | 9 | 9 | 8 | 5 | 9 | 6 | 7 | **53** |
| operator_first | 1 | 8 | 7 | 9 | 7 | 6 | 9 | 8 | **54** |
| operator_first | 2 | 9 | 7 | 9 | 7 | 6 | 9 | 6 | **53** |
| safety_first | 1 | 9 | 7 | 6 | 9 | 8 | 7 | 5 | **51** |
| safety_first | 2 | 8 | 8 | 7 | 9 | 8 | 7 | 6 | **53** |

- **Combined totals:** operator_first 107, reuse_max 104, safety_first 104.
- **Judge 1** chose operator_first.
- **Judge 2** scored all three at 53 and chose safety_first.
- **Both judges** asked for grafts across the proposals, and their grafts largely agree.

### 3.3 Decision

**Lite is a standalone application in its own folder, `LITE/alix_studio_lite/`, built on the operator-first architecture.** The grafts below close the gaps the judges found. Items 5 and 7–10 record the team's decisions of 13 October 2026 (F1–F7).

1. **Product shape (operator-first).**
   - A loopback web app in a chromeless window, served by a small Python server.
   - One guided loop per region.
   - Three named modes: Simulation / Dry run on the robot / Real robot.
   - An Engineer switch.
   - A relocatable CPython bundle with a no-root installer, update from file and one-click rollback.
   - The UI must be usable within about 3 s, while the 3D worker warms up in the background.
   - Usability tests run at three points.
2. **World integrity (safety-first).**
   - The ALIX assets ship as a **git-archive snapshot of the pinned commit, in repo layout, with a `.git` marker file**. The bundle's URDF path is passed explicitly, so the unmodified libraries resolve every mesh.
   - A **start-up self-test fails closed**, and every verdict stores its **world manifest**.
   - The **scene** comes from the pinned ALIX commit: the URDF (arms, hands, poles, UGV, bin geometry, `ALIX/core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf:188-359`) and the commode C scene (`SCENE/commode_cleaning/commode_c/scene.yaml`: commode pose and articulation). No cell-calibration project is needed (F5).
   - **Walls are enforced.** Today's checks do not enforce wall collision (F5). Lite loads a single cubicle, 1400 mm wide by default or 800 mm (`SCENE/scene/washroom_800mm_wide_scene.yaml`), selected in Settings (decision D21), through `load_wall_manager` / `PlanWallChecker` (`WS/plan_scene_validation.py:117, 179`) and checks every motion against it, transits included. The selected file is part of the fingerprint and the world manifest. An earlier analysis saw false wall hits with the 800 mm walls in the wider cell, so the cubicle is verified once per cell by replaying known-good recordings.
   - An **intended-contact policy** comes from `contact_links.yaml`.
   - Verdicts use a **ladder keyed by fingerprint**, built on `DP/ladder.py`.
   - Arm identity is checked when connecting.
   - The **verified stop ladder** (`DP/track.py:3109`) is the fallback stop.
3. **Reuse discipline (reuse-max).**
   - Lite **imports** `alix_workspace`, `alix_dispatch` and the whitelisted light `alix_taskgen` modules (`body_collision`, `scene_pose`, `fixture_parts`, `transforms`, `paths`, `constants`, plus `singularity`, `glove_frames`, `approach_transit`). An import-boundary test guards this.
   - Small, default-preserving **upstream seams** (S1–S5) replace ports where the ALIX owner agrees.
   - The **unchanged Studio** serves as a zero-code harness in Phase 0.
   - Copied UI code carries provenance headers and **drift tests**.
   - Every motion route needs a **one-time confirmation token**.
   - A **process interlock** refuses robot commands while v53 or a runner that Lite did not start is running.
4. **Executor policy.**
   - **Release 1:** runner v14, **sha256-pinned**, run through a Lite-owned launch wrapper (§4.4).
   - **One runner patch, for the E-stop only (F4).** Runner v14 watches only run state and error codes (`R14:122-138, 219-271, 641-700`), so an IO E-stop during a run would very likely be reported as "RUN COMPLETE". The patch reads the E-stop IO (the IO in mode 5, via `rm_get_io_state`) at start and in the run and sequence loops; refuses to start while the E-stop is held; reports the run as interrupted; never starts the next sequence step after an E-stop; and prints parseable `ESTOP ENGAGED` / `ESTOP CLEAR` lines (E-stop research). The patched runner is pinned by its own sha256, and its upload bytes are checked against v14's `write_project` by the `.proj` parity test. How Lite reads the E-stop state *between* runs without a second connection to the controller (principle 5) is settled in P0/P3.
   - **Release 2:** per-arm agents on `alix_dispatch`, after a hardware acceptance list. The agents poll the E-stop IO through their own connection. The runner then stays as the fallback and test oracle.
   - No other runner patch. Real-arm collision mapping uses a post-run joint read in Release 1 and UDP capture inside the agent in Release 2.
5. **Location and release coupling (decided, F7).**
   - Lite's code lives in `LITE/alix_studio_lite/` and its documents in `LITE/alix_studio_lite/docs/`. It is **not** a package inside the ALIX monorepo.
   - Lite imports the ALIX libraries from its vendored copy in `vendor/alix`; nothing is read from an ALIX checkout at run time. The commit is recorded in every certificate.
   - Lite's own CI runs the parity tests (FK, collision world, `.proj` bytes, TOPP-RA) against the pinned commit, and a nightly canary runs them against ALIX HEAD. ALIX CI does not run Lite's tests, so moving the pin is a deliberate step that needs the parity suite green.
   - Lite ships as an offline bundle that pins one ALIX commit, the runner sha256 and the SDK version.
6. **FK.**
   - FK runs on the server, in the simulation worker. It sends windowed frames for the moving links only, following the Studio's one-FK rule (`TG/server.py:5299-5302`).
   - A JS FK is added only if measured payloads are too heavy, and then for display only, with a bit-parity test.
7. **Target computers (decided, F1).**
   - The operator PC (x86-64 Linux) now; the robot's Jetson AGX Orin (aarch64, Ubuntu 24.04, `ALIX/SETUP.md:76-82`) in production. Both are supported, and an aarch64 production build is required.
   - The bundle is built per architecture. The RealMan SDK ships both `linux_x86` and `linux_arm` `libapi_c.so` (E-stop research). `toppra` 0.6.10 has x86-64 wheels only on PyPI and no source package, so the aarch64 bundle compiles it from the git tag (optimiser research). One solve is timed on the Orin early. Lite works without `toppra`: the TOPP-RA option is then greyed out and the default optimiser remains.
8. **Optimisers and target time (decided, F2).**
   - Two optimisers behind one interface in `LITE/alix_studio_lite/lite/optimise/`: the **default** (the prototype's trim → smooth → scale, with three fixes) and **TOPP-RA** (ALIX's chain through one adapter, `EXP/emit_toppra_rung.py:306-330`, with a Lite-owned `retime_toppra` that must reproduce ALIX's slot-101 file, 3839 lines from `FILE_01.1`, within 1 millidegree).
   - The operator's time is the **target**. Each optimiser finds T_min, the shortest time whose written bytes pass the hard limits. If the target is at or above T_min, uniform time scaling meets it to the tick. If not, Lite delivers T_min and names the binding limit, for example "joint 6 acceleration, 2.88–3.05 s" on `FILE_10.1` (optimiser research).
   - The hard limits are the same for both and are judged on the integer bytes at 5.0 ms per line: G1 joint-limit margin, band-limited speed within 0.97 × the ratings, acceleration within 600 °/s² as a 50 ms windowed mean, at most 10° per line, pad path change within 10 mm (or the region's maximum deviation), and the recorded start and end poses kept.
9. **Safety stop (decided, F4).**
   - The IO-wired E-stop buttons on the left and right controllers are the primary safety stop. Lite never writes IO modes; it finds the IO in mode 5 at connect and warns if there is none or the two arms differ.
   - Each arm has an E-stop chip: CLEAR, E-STOP ENGAGED, RECOVERING, UNKNOWN or NOT CONFIGURED. It is read from `rm_get_io_state` on the mode-5 IO and cross-checked with the joint enable flags, never from `arm_current_status == 9` alone, because that flag stays set after any stop (`ALIX/ros/hw/alix_hw/src/rm_side_system.cpp:1856-1860`). While either arm is not CLEAR, motion commands are refused on both arms; uploads stay allowed.
   - A run interrupted by an E-stop is marked INTERRUPTED, the rest of the sequence is cancelled, and Lite sends a pole stop and freezes the hand, because end-port power survived an E-stop in the ALIX field test and the hand kept working (`ALIX/docs/TEST_PROCEDURE.md:216-242, 453`). Recovery is a guided wizard that ends with a collision check, walls included, of the move to the program's first point. Lite never resumes automatically.
   - The IO E-stop's stop category and reaction time are not documented by RealMan, and ALIX's code assumes an E-stop drops joint power, which conflicts with "no power cut" (E-stop research). The drill measures what actually loses power and how quickly the arms stop, before operators use REAL mode.
10. **Upload slots (decided, F6).**
    - Lite uploads to any program number 1–100, as UI v53 and runner v14 do (`V53:9-11, 4279`; `R14:324`), and overwrites an existing program after a confirmation that shows the program in the slot (`V53:4469-4488`). Batch upload with one Program ID per file is supported.
    - Lite does not reserve a range and blocks no slot. It warns when a slot holds a program it did not write (for example one from the V6 workflow), and it reads back the stored name and size before every run.

### 3.4 Rationale

- **Operators are the product's users.** Operator-first had the best combined score, the best UX and deployability marks from both judges, and the lowest delivery risk from judge 1. Its weaknesses are exactly the areas the grafts cover: world integrity, contact policy, copy discipline, and the stop and capture design.
- **Importing beats copying for the commode world.** `fixture_manager` carries the hardware fixes for the washroom rotation and the UGV standoff (`TG/body_collision.py:401-464`). Copying it would recreate the failure those fixes closed. The modules are light, and the import is checked for this plan.
- **The runner stays for first hardware use, changed only for the E-stop.** It is the field-proven path: CRLF `.proj`, store-only upload, size read-back, speed restore and slow stop. The E-stop patch (§3.3 item 4) is the one change, because without it an E-stop during a run is very likely reported as a completed run. Safety-first's agents become the Release-2 executor. They bring persistent sessions, the verified stop ladder, heartbeats, UDP capture and faster uploads, and they will have earned their place through hardware acceptance.
- **The world fails closed but does not block testing forever.** The cell geometry comes from the URDF and the commode C scene, so there is no calibration to wait for (F5). Until the selected venue wall file has been verified with known-good recordings, wall results are shown as "walls not verified". A supervisor can accept that, with a reason, and the first real run then uses a slowed copy. A *collision* found in the verified world, walls included, can never be overridden; only per-pair waivers reviewed by an engineer are allowed (§9.2). This resolves the judges' concern that a non-overridable OFFLINE gate would stall testing.

### 3.5 How the judges' concerns are resolved

| Concern | Resolution in this plan |
|---|---|
| The target hardware is unknown: an x86_64 operator PC or the robot's Jetson AGX Orin (aarch64, Ubuntu 24.04, `ALIX/SETUP.md:76-82`). Snap Chromium may block the app window. | Decided (F1): the operator PC now, the Orin in production; both are supported. The bundle is built per architecture; aarch64 wheels exist for python-fcl and the SDK ships `linux_arm`. `toppra` 0.6.10 is compiled from its git tag for aarch64, and Lite runs without it (§3.3 item 7). Golden tests run on both architectures. The browser falls back to the default if `--app` fails. A headless Orin needs the authenticated remote mode (§9.7). |
| STOP is unproven: SIGINT only, no signal handler, a program keeps playing if its client dies, Windows CTRL_BREAK skips the slow stop | The IO-wired E-stop buttons are the primary safety stop (F4); Lite shows their state, blocks motion while either is engaged or unknown, and reports interrupted runs (§3.3 item 9). The software STOP is labelled a convenience. P0 characterises it on hardware (G4 probe, killed runner, a fresh-connection stop after the client dies). STOP is a single stdin/SIGINT signal through the launch wrapper. When the backend dies, stdin closes and the runner slow-stops. If a stop is not confirmed, Lite kills the runner and runs the verified stop ladder on a fresh connection. Software arm stops do not stop the pole, so STOP also sends a pole stop (`DP/peripherals.py:189-199`). Robot I/O is Linux-only in v1. REAL is not available to operators until commissioning is recorded and the E-stop drill cases DR1, DR3, DR9, DR10 and DR11 (E-stop research) have passed (§9.3). |
| Collision verdicts need a calibrated cell | No calibration project (F5): the URDF and the commode C scene already hold the commode pose and articulation, bin, UGV and poles. The one missing check, wall collision, is enforced against a single cubicle, 1400 mm wide by default with 800 mm selectable (D21, §3.3 item 2). Verifying the cubicle by replaying known-good recordings is a P0 exit criterion; an earlier analysis saw false wall hits with the 800 mm walls in the wider cell, which is why it is verified. Until then wall results show "walls not verified", a supervisor may accept that with a recorded reason, and a slowed first run follows. Per-pair waivers need an engineer and an audit record. There is never a bypass. |
| R11's "immediate" validation needs a warm sweeper | One warm worker process. Seam S1 adds `body_links`, `bin_state`, `pole_m`, in-memory timelines and an explicit index. If S1 is declined, Lite wraps the private helpers, with a parity test against the library call. The choice is made in P0. |
| Copying `alix_taskgen` creates drift in safety code | Lite imports the whitelisted modules from the pinned bundle. An import-boundary test denies `server`, `studio_*`, `path_*`, `cspace_*` and `robot_model`. |
| The JOINTS runner patch contradicts the sha256 pin | No JOINTS patch. Release 1 reads the joints after the runner exits (sequential, so still one command source). Release 2 uses UDP capture inside the agent. The only runner change is the E-stop patch (F4), which is reviewed, pinned by its own sha256 and covered by the `.proj` parity test (§3.3 item 4). |
| The world degrades silently (empty fixture manager, unresolved meshes, missing SRDF) | The asset bundle keeps the repo layout and a `.git` marker, and the URDF path is passed explicitly. A self-test runs at every start and in CI on a clean machine. Every verdict has a world manifest (§4.8, §9.2). Lite does not rely on `ALIX_REPO_ROOT` alone. |
| The executor transition must be explicit, with the runner's non-negotiables kept | Stage A and Stage B have written switch criteria and a list of non-negotiables. A `.proj` parity test checks Lite's bytes against R14's `write_project` (§4.4). |
| Gate friction: a dry run per changed fingerprint, an identity "finger wiggle" | The `DP/ladder` rule applies: a failed dry run blocks REAL; a missing one can be overridden by an engineer with a reason. The identity check reads J4 drive limits passively; the wiggle is optional and engineer-only. |
| Never copy the G37/G38 world code; JS FK only for display | Both adopted (§3.3 items 3 and 6). |
| "Practice (arms stay still)" may over-promise | The wording is "Dry run on the robot: the arms do not move; pole and hand steps are skipped". It is finalised after P0 measures what the pole and hand do in SIM. |
| The unit catalogue says 42 in one place and 44 in another | 42 units (requirements, Appendix A; C1). Codes now follow the deck's own format; the requester signs off the catalogue (Appendix A and its CSV) before P2 (D3). |
| Program-slot policy | Decided (F6), following the control program: any program number 1–100, overwrite after a confirmation that shows the existing program, batch upload with one Program ID per file (`V53:9-11, 4279, 4469-4488`). No reserved range and no blocked slots; Lite warns when a slot holds a program it did not write (for example V6's 71–83), keeps an upload ledger, and reads back the stored name and size before every run. |
| Dual-arm programs arrive late | v53 stays the dual executor, and Lite exports v1 workflows. The TOGETHER drift is modelled (pole law, timing envelopes), and start skew is measured in P0 and P8. |
| Effort and contingency | Answered by the agent-driven schedule (F8, §7), which starts with the UI/UX work (D27): a runnable core loop in about 2 agent working days of build; Release 1 in about 5 agent days (range 4–6) plus about 3.5 robot lab days; Release 2 in about 3 agent days (range 2–4) plus about 1.5 lab days and an operator field trial. Most of the work reuses ALIX Studio, the ALIX libraries and this workspace. Lab time and sign-offs set the calendar. The minimum safe release (P0–P7) stays separate from Release 2, and the R9 offset tool is engineer-only and comes late. The bottom-up single-developer sizing (about 280 person-days) is kept in §7 as a reference only. |
| Monorepo churn: 73 commits in 14 days on `feature/alix_studio`, all by one author (checked) | Lite sits outside the monorepo (F7) and imports ALIX from the vendored copy in `vendor/alix`. One ALIX commit is pinned per Lite release and stamped into every certificate. Lite's CI runs the parity tests at that commit, and a nightly canary runs them against ALIX HEAD. |
| The meaning of "low spikes" in R12 | Decided (D10): sudden downward jumps (▼), the mirror of high spikes. Hesitations (SD6) stay an information note, not a spike. |
| Drag teach has no emulator | A fake recorder replays stored recordings in CI and practice mode, and P4 budgets supervised hardware sessions. |
| The write-up must stay high-level for the control app | §1.3.1 says what each program does and which version is current. Issues appear one line each, only where they affect the plan. |
| The data-collection gap: 16–17 units untaught, V6 merged | Teaching starts now in v53, with a teach sheet. P2 ships "Import recording" with provenance. V6 is imported as LEGACY and cannot be uploaded until it is re-certified. |
| The `TypeError` in v53 | It matters only while v53 is the fallback. Its owner applies the one-line fix if v53 stays in service (D16). |

---

## 4. Target architecture

### 4.1 Process and component diagram

```
 Computer that runs Lite (decision F1)                                            Robot network 192.168.1.x
   now: operator PC, x86-64 Linux · production: the robot's Jetson AGX Orin, aarch64, Ubuntu 24.04
   headless Jetson: the page opens in a browser on the operator PC through an authenticated tunnel (§9.7)
┌──────────────────────────────────────────────────────────────────────────────┐
│ Desktop icon ─► launcher: single instance · isolated Python · preflight ·      │
│                 world self-test · opens a chromeless window                    │
│                 --app=http://127.0.0.1:8766/?t=<token>   (fallback: default browser)
│                                                                                │
│ Browser page: one single-page app on ALIX Studio tokens and components        │
│   Dashboard · Regions · Teach · Optimise · 3D Check · Test on robot            │
│   [Release 2] Program · [Engineer] Adjust position · Robot console · Settings  │
│   E-STOP chip per arm (IO input state) · STOP (software) LEFT / RIGHT / ALL    │
│        │  JSON over HTTP · per-launch token header · Host/Origin check · polling│
│        ▼                                                                       │
│ lite-web  (Python, stdlib ThreadingHTTPServer, bound to 127.0.0.1 only)         │
│   routes + needs table · job registry · project store · catalogue + status     │
│   recipe processor · gates + certificates · ladder (DP/ladder)                 │
│   motion gateway (arming, one-time tokens, interlocks, E-stop interlock,       │
│   process interlock) · dual coordinator (Release 2) · audit log · executor     │
│   supervisor (per-arm lock; STOP bypasses every lock and queue)                │
│        │ spawn queue (1 worker each)                │ one channel per arm       │
│        ▼                                            ▼                           │
│ lite-opt  (1 process)                     Robot executor seam (RobotBackend)   │
│   default optimiser (prototype, fixed)     ├ Stage A, Release 1:                │
│   TOPP-RA optimiser (toppra, if present)   │   lite_runner_launch + runner v14 ─┼─TCP 8080─► LEFT  192.168.1.10
│   target-time solver · L1–L6 byte checks   │   + small E-stop patch (sha-pinned)┼─TCP 8080─► RIGHT 192.168.1.103
│                                            │   one subprocess per action        │
│ lite-sim  (1 process, 1 thread)            │   + lite_status (read-only; identity;
│   alix_workspace FK + FCL world            │     E-stop IO poll while idle;     │
│   commode_c scene at lid/seat state        │     post-run joints; fallback stop ladder)
│   walls (selected venue file), bin state,  ├ Stage B, Release 2:                │
│   partner arm, contact policy, manifest    │   lite-arm-left / lite-arm-right   │
│   jobs: sweeps (every row), windowed       │   agents on alix_dispatch: one     │
│   re-checks, FK frames, transit/link-move  │   persistent connection each, stop │
│   checks, offset solves; cache by hash     │   ladder, heartbeat, UDP capture,  │
│                                            │   E-stop IO poll 5–10 Hz           │
│                                            ├ Preview (Simulation; no robot)     │
│                                            └ Emulator (engineers and CI only)   │
│                                                                                │
│ Vendored ALIX copy (read-only): vendor/alix, one ALIX commit, in repo layout   │
│   + .git marker + MANIFEST.sha256: alix_workspace / alix_taskgen / alix_dispatch│
│   / alix_emulator sources, taught-track tools and exp01 TOPP-RA modules, URDF,  │
│   meshes, SRDF, initial positions, commode_c scene, 800/1400 mm cubicles and    │
│   their wall meshes, contact_links.yaml, hand_units.py, design tokens, overlay  │
│ Project data (outside the install folder): ~/ALIX-Lite/…                        │
└──────────────────────────────────────────────────────────────────────────────┘
 Code: /home/newtonjeri/alix_studio_lite_ws/alix_studio_lite (F7) · Docs: alix_studio_lite/docs
```

**Why this shape:**
- **Two computers, one design (F1).** Lite runs on the operator PC (x86-64 Linux) now and on the robot's Jetson AGX Orin (aarch64, Ubuntu 24.04, `ALIX/SETUP.md:76-82`) in production. The code is the same on both; only the bundle differs (§4.8). When the Jetson runs without a screen, the server stays bound to 127.0.0.1 and the operator opens the page on the operator PC through an authenticated tunnel (§9.7); the server is never opened to the robot network directly.
- **One simulation worker.** FCL collision managers are not thread-safe, and sharing them has crashed (`TG/body_collision.py:148-171`). So exactly one worker process owns the collision world. It stays warm, which makes repeat checks take seconds; a world takes 2.6–10 s to build (source map 05).
- **The optimiser runs in its own worker.** A TOPP-RA solve takes 0.1–1.8 s on the operator PC (measured 13 Oct, optimiser research) and is expected to be slower on the Jetson (not yet measured). A separate process keeps the UI responsive and Cancel available, and keeps scipy and toppra off the UI path.
- **Heavy imports stay off the UI path.** numpy, scipy, toppra, trimesh and fcl load only in the workers, so the page comes up in about 3 s (the Studio takes 30–60 s).
- **Robot I/O runs outside the web process.** A crashed SDK or a hung blocking call cannot freeze the UI. The emulator replaces `sys.modules["Robotic_Arm"]`, so real and emulated backends cannot share a process anyway.
- **All long work is a job.** HTTP/1.0 has no request timeout (source map 04). Uploads, runs, sweeps, solves, optimisations and merges are therefore jobs with honest progress: stage *i* of *n* and elapsed time, never an invented percentage.
- **One command source per arm.** Each arm has one per-arm lock and at most one live executor job. STOP never waits in a queue. E-stop polling goes through the same connection owner, so it never opens a second competing connection.

### 4.2 Backend modules (package `alix_studio_lite`)

The code lives in `/home/newtonjeri/alix_studio_lite_ws/alix_studio_lite/` (F7). Lite is a standalone project, not a package inside the ALIX monorepo; it imports ALIX libraries from the vendored ALIX copy in `alix_studio_lite/vendor/` (§4.8).

Aliases added for this section and §5: `EXP01/` = `ALIX/experiments/exp01_teach_trajectory_reverse_engineering/src/`; `LS` = `EXP01/local_speed.py`; `ETR` = `EXP01/emit_toppra_rung.py`; `KIN` = `EXP01/teachlab/kinematics.py`; `PMT` = `ALIX/tools/plan_min_time.py`; `TRK` = `DP/track.py`; `PIPE` = `ALIX/docs/motion/TAUGHT_CANFD_PIPELINE.md`; `VENUE/` = `ALIX/ros/orchestration/alix_tasks/config/scene/`; `CCS` = `ALIX/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml`; `RRI` = the RealMan SDK `rm_robot_interface.py` (`ALIX/external/butterfli-realman-alix/Python/Robotic_Arm/`). "Optimiser research" and "E-stop research" are the 13 Oct research notes behind decisions F2 and F4.

| Module | Responsibility | Built from (mode) | Phase |
|---|---|---|---|
| `launcher` | Single instance; `python -I` with `PYTHONPATH` cleared; preflight; world self-test; opens the app window; clean shutdown that stops robot jobs first | Pattern of `ALIX/tools/run_studio.py:92-205` (copy and adapt) | P1 |
| `server.http`, `server.routes`, `server.jobs`, `server.page` | Loopback handler, gzip, request cap, vendor whitelist, the `{error, hint}` refusal convention, a needs table per route (`robot_io:<arm>`, `sim`, `opt`, `project_write`, `long_running`), the job registry, and page assembly | Copy about 400 lines of `TG/server.py:5178, 5961-6253, 6297-6469` and `TG/scripts/visualize_generated_config.py:530-568`, plus the job pattern `TG/studio_generate.py:537-600, 861-1060`. Each copy carries a provenance header | P1 |
| `assets` | Locates and verifies the vendored ALIX copy (`vendor/alix`, against `vendor/MANIFEST.json`); puts the `alix_*` sources, `ALIX/tools` and `EXP01/` on `sys.path` once; sets `ALIX_REPO_ROOT`, `ALIX_SRDF` and `BUTTERFLI_WORKSPACE_CONFIG`; runs the world self-test; builds the world manifest; reports the machine architecture and whether `toppra` imports | New | P0, P1 |
| `project.store`, `project.schemas`, `project.audit` | Project folders; atomic writes (temp file, fsync, rename); read-only raw files; sha256; version counters; conflict guard; backups; the append-only audit log; JSON Schemas | Copy the atomic-write helper `TG/schedule_guards.py:59-104` | P1 |
| `project.catalogue` | The 42-unit catalogue for commode C (YAML per deck version, `Toilet_regions_V1.pptx`, decision F3) and its validator; the region target time and optional MAX DEV per unit; deck assets; the status engine that derives each unit's status from the files on disk | New. Schema from the region analysis (requirements, Appendix A) and C1. Offline `deck_import` tool promoted from the deck-analysis scripts written during this analysis | P2 |
| `project.importers` | Imports v53 drag recordings (assigned to a unit, with provenance questions); V6 legacy import split at known sample indices and flagged LEGACY; export and import of zip packages | New | P2, P11 |
| `teach` | The teach session state machine (C1 §1.8.3), the ordering rules (by arm within each lid/seat phase), save → check → rename → sidecar, crash recovery, and the fake recorder | New; pole/hand rules from `UC/usage.py:44-150` | P4 |
| `motion.io`, `motion.timing`, `motion.limits` | Strict parser (7 integers per line, at least 2 lines, the whole file refused on any bad line); CRLF integer writer; `.proj` writer identical to R14 `write_project`; the clock (one time base of 5.0 ms per line, D26: the record clock, limits judged at 5.0 ms per line, `TT:94-102`, and durations of (lines − 1) × 5 ms); the limit set: speed ratings 180/180/225×5 °/s (`TRK:3728`), acceleration 600 °/s² (`KIN:39-41`), position limits from the URDF (`KIN:43-46`), step ≤ 10° per line (`TT:127-128`), the 0.97 allowance on the 20 Hz reading (`TT:229`); execution-model constants | Import `TRK:66 parse_track, 85 build_project_file`; URDF through `WS` | P1, P5 |
| `motion.pipeline` | The recipe processor and the single entry point for optimisation: raw track + the unit's region settings + the operator's **target time** + the optimiser choice → `OptResult` (bytes, target, achieved time, T_min, met yes/no, binding constraints, L1–L6 results, recipe with library versions and the origin sha256) → dwells → integers → certificate. Two optimisers sit behind one interface (decision F2). | New; the interface and types from the optimiser research §6.1 | P5 |
| `motion.optimise.default` | **Default optimiser**: the prototype's algorithm, trim idle → moving-mean smooth → per-segment linear time scaling, ported to Python, with three minimal fixes: (a) odd-reflection edge padding, which removes the 546–1005 millidegree first/last-line snap (`HTML:1513-1514, 1721-1723`); (b) the speed change at each segment boundary blended over 0.25 s and resampled on the chords, which cuts boundary spikes from 640–919 °/s² to 200–280 °/s²; (c) default window 41 instead of 101 plus a hard path-change check (G4) against the raw recording, which cuts pad movement from 10.6–20.9 mm to 2.9–4.5 mm. Segment setting: time % (the prototype's meaning) with an optional pinned time. A JS preview mirrors it for the live plot. | Port of `HTML:1438-1616, 1676-1724` with golden parity tests (Python canonical; JS preview within 1 millidegree) | P5 |
| `motion.optimise.toppra` | **Time-optimal optimiser (TOPP-RA)**: ALIX's hardware-proven chain for taught paths: `smooth_taught` at 5 Hz → `strip_pauses` (R4) → `TaughtPath` + `arclen_smooth` 30 mm (15 mm when the region has a MAX DEV) → retime with 0.95 × the speed ratings, 600 °/s², the Interpolation scheme, the seidel solver and zero speed at both ends → sample on the 5.0 ms clock → integer millidegrees. Segment setting: speed cap %, passed to the solver as a varying velocity constraint so braking stays legal. The optional 6 Hz post low-pass (`ETR:200-215`) is off by default. Offered only when `import toppra` succeeds; otherwise the option is greyed out with the reason and Lite works with the default optimiser. | Import `TT:891-912 smooth_taught, 915-951 strip_pauses`, `LS:78-104 TaughtPath, 139-169 arclen_smooth` through one adapter module; the recipe of `ETR:306-330`. A Lite-owned retime function of about 40 lines with explicit arguments (`retime_toppra(path, vlim, alim, vcap)`), because `LS.RATE_DEG_S` and `LS.ACC_DEG_S2` are module globals that the emitter changes and are not thread-safe (`ETR:304-305`); parity test against `LS:282-324` (FILE_01.1 must give 3839 lines within 1 millidegree, `PIPE:588`). Sampling in the `PMT:375-394` form. Library: toppra 0.6.10 | P5 |
| `motion.optimise.target` | **Target-time solver**, the same rule for both optimisers. The target is in seconds at 5 ms per line: the region's catalogue time, which the operator can override, turned into a line count (target ÷ 5 ms, D26). T_min is the shortest time whose written bytes pass L1–L6: TOPP-RA gets it from one solve plus a byte check; the default gets it by bisection on log k (about 25 iterations, under 0.5 s). If the target is at or above T_min, it is hit to the tick by uniform time scaling (speed ÷ k, acceleration ÷ k²); for TOPP-RA this matches a re-solve with scaled limits within 0.0006° (measured 13 Oct). If the target is below T_min, Lite delivers T_min and names what binds as joint, quantity and time window (top 1–3 by share of time for TOPP-RA; the check that fails at 0.995 × k_min for the default). Example: FILE_10.1 is limited by joint 6 acceleration, 38 % of the time, 2.88–3.05 s. TOGETHER blocks: both arms get the same line count N = max(target, left T_min, right T_min), the report names the binding arm, and the dual-arm collision timeline is re-run on the retimed pair. | New (`target`, `binding`, `together` of the optimiser research §6.1); verdicts tied to both tracks' sha256 (`DAV:18-22`) | P5 (TOGETHER: P9) |
| `motion.gates`, `motion.spikes` | **Hard limits L1–L6**, judged on the written integer bytes at 5.0 ms per line, for both optimisers: L1 the G1 predicted executed joint margin ≥ 0.5°; L2 20 Hz band-limited speed ≤ 0.97 × the ratings (G2); L3 acceleration ≤ 600 °/s² × 1.02 as a 50 ms windowed mean (hard in Lite, advisory in ALIX; the 20 Hz Butterworth reading is shown as advisory because the filter overshoots); L4 step ≤ 10° per line (G3); L5 path change from the raw recording (G4) ≤ 10 mm, or the region's MAX DEV if smaller; L6 the file ends at the recorded start and end poses. **Checks only:** G5 predicted executed departure (a warning only, D11), jerk, pad speed, single raw ticks over a rating. Spike detectors SD1–SD6 with the rule table and one-click fixes | Import/port `TT:420-716, 958-991, 1083-1104`; `CT:278-336, 1743-1750`; `EXP01/predict_track_replay.py:76-136`; `ALIX/Resource/tracks/taught/EXECUTION_MODEL.json` | P5 |
| `sim.worker` | The warm world; job queue; cooperative cancel; cache by content hash; restart on crash | New; pattern of the Studio's spawn pool `TG/server.py:700-725` | P6 |
| `sim.adapter`, `sim.frames` | `.txt` → timeline with **mandatory** pole, hand and bin seeding (hand counts → radians via `hand_units.py`); flow events (programs at (lines − 1) × 5 ms, measured laws for the other steps); windowed FK frames for moving links | Import after seam S2, else port `DAV:501-731`; `WS/urdf_kinematics.py:294-344`; `WS/dual_arm_plan_collision.py:487-583` | P6 |
| `sim.world`, `sim.sweep`, `sim.scene` | World composition (arms with full hands, curated self pairs, partner, commode by state, walls of the selected venue, bin state), contact policy, world manifest; every-row gating sweep, quick sweep, windowed re-check, transit / link-move / device-move checks (including the recovery move from the current pose to a program's first point), drift-window arm-vs-arm. **`sim.scene`** builds the scene from what ALIX already holds (decision F5): the URDF (arms, UGV, poles); the `commode_c` scene (body, lid and seat meshes, articulation, and the authoritative pose relative to `base_link`, `CCS:29-37`); the bin geometry from the URDF (`ALIX/core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf:188-359`) with its states through the SRDF `BIN_*` group states; and the walls of the venue file selected in Settings, loaded with `load_wall_manager` (which takes only the `washroom_structure` mesh and pose from the venue file, `WS/plan_scene_validation.py:117-156`) and checked as in `PlanWallChecker` (`:179-257`). Walls are the one item ALIX does not enforce today; Lite enforces them. Choosing the cubicle is a setup step, not a calibration project: the 1400 mm cubicle is the default and the 800 mm one can be selected (D21); replay known-good recordings once and expect no wall contact. The 800 mm default produced false wall hits on R2_LIDTOP that vanish at 1400 mm (C3 §4.4 #12), which is why the selection must be verified. | Import `TG/body_collision.py:401-464` (`fixture_manager`), `WS/plan_scene_validation.py:117-270`, `WS/dual_arm_plan_collision.py:586-764, 1089-1283` (with seam S1), `WS/redundancy.py:90-117`; data `contact_links.yaml`, `CCS`, `VENUE/*.yaml`; envelopes `DAV:150-224, 862-973` | P0 (venue check), P6, P9 |
| `robot.backend` | The `RobotBackend` protocol and its typed events, including E-stop events | New | P3 |
| `robot.runner_backend`, `lite_runner_launch.py` | Stage A executor (§4.4): builds the argument list with `--sdk --side --ip --port --yes`; parses the stdout tokens, including the patched runner's `ESTOP ENGAGED` / `ESTOP CLEAR` lines, into events; STOP once; stop on backend death; sha pins | Vendor R14 with the small E-stop patch (§4.4), `UC/hand.py` and `UC/pole.py` byte-identical. Token list from source map 09 | P3 |
| `robot.status` | Read-only helper: joints, error codes, run state, run mode, lift height, hand angles, tool frame, J4 drive limits (identity); post-run joint read; fallback stop ladder on a fresh connection, used only after the runner process has ended. **E-stop state per arm** (decision F4): at connect, scan IO1–IO4 for the input whose `io_mode` is 5 ("input emergency stop function multiplexing"); then poll `rm_get_io_state` on it at 5–10 Hz while no runner holds the arm, cross-checked with the joint enable flags and, if the site enables Modbus-TCP, input register 302. Never derived from `arm_current_status == 9` alone, which stays set after any stop (`ALIX/ros/hw/alix_hw/src/rm_side_system.cpp:1856-1860`). States: CLEAR / E-STOP ENGAGED / RECOVERING / UNKNOWN / NOT CONFIGURED; the last two block motion like ENGAGED. Lite never writes IO modes; it warns when no IO is in mode 5 or the two arms use different IOs. Guided recovery: clear errors (`rm_clear_system_err`, `rm_set_joint_clear_err`, at most two rounds with read-back), re-enable joints if needed, restore the global speed, check the move to the program's first point in `sim`, re-home the pole; never auto-resume | Import `DP/connect.open_arm`, `DP/track.stop_program`, `DP/runmode`; IO calls `RRI:2882, 2927` (E-stop research §5) | P3, P7 |
| `robot.gateway` | Arming per arm per session; one-time confirmation tokens (60 s, bound to the command hash); interlocks; **E-stop interlock**: while either arm is not CLEAR, the server refuses Run, Sequence, Test on robot, Move to start, Pole, Hand, Drag-teach START and the switch to REAL mode on both arms, because one E-stop chain covered both arms in the 2026-08-06 test (`ALIX/docs/TEST_PROCEDURE.md:453`); uploads stay allowed; on an E-stop during a run it marks the run INTERRUPTED — E-STOP, cancels the rest of the sequence and sends pole stop (`rm_set_lift_speed(0)`, `DP/peripherals.py:189-199`) and hand freeze; process interlock (`/proc` scan for v53 or runners Lite did not start); logging | New | P3 |
| `robot.ladder`, `robot.slots` | Ladder records and gates (`record_rung`, `latest`, `gate_real` with Lite's OFFLINE rung). **Program IDs** as in the control program (decision F6): any ID 1–100 on either arm (`R14:324-325`); batch upload with a program ID per file (`V53:11`); overwriting an occupied slot needs a confirmation that shows the program now in it (stored name and size from the controller, plus the ledger's unit, sha256 and date when known), as v53 does (`V53:4279, 4472-4476, 8771`); run by ID or by name. Soft warnings only, never a refusal by range: for example "this slot holds a program from the V6 workflow", "this slot is not in Lite's ledger", "this file is already in slot n". Upload ledger; side-tagged program names of 32 bytes or fewer; name and size read-back after every upload and before every run | Import `DP/ladder.py:63-145`; patterns from `CT:4046-4160` | P7 |
| `robot.agent` | Stage B executor: one process per arm, one persistent connection, non-blocking verbs, heartbeat thread, verified stop ladder, `global_speed_guard`, run-mode read-back, one-session batch upload, UDP capture and ordered matcher, E-stop IO poll on the same connection, drag teach ported from R14 with save retry | Import `DP/connect, track, runmode, peripherals, recording, staging`; port `R14:912-1210, 1265-1665` | P8 |
| `robot.dual` | The SAFE TIMELINE planner, with an explicit `checkpoint` step; two-phase start; cross-arm abort (also on an E-stop of either arm); handoff prompts | Port `V53:9619-10017` (`_page2_program_chunks`, `_page2_build_program_rule_plan`), parity-tested on the V6 FINAL pair | P9 |
| `merge` | The Program Builder (R8): grouping, joins, rest-to-rest minimum-jerk link moves, program ends at rest, detours through a checked safe pose, outputs and provenance, slowed first-run kit | Import `PMT:142 concatenate`, `TG/approach_transit.py:60-73` (sampling rule) | P9 |
| `offset` | R9/R14: taught-following differential IK (feed-forward, null-space pull, smooth tapers), span report with press-axis classes, pole option, touch-off fit | Import FK/Jacobian `WS/urdf_kinematics.py:218-259`, `WS/ik.py:213-231`; `TG/singularity.py`; `TG/glove_frames.py`; the Kabsch idea from `ALIX/tools/track_report.py:683-688` | P10 |

### 4.3 Front-end screens mapped to requirements

| Screen | What the user does there | Requirements | Phase |
|---|---|---|---|
| **App shell** (always visible) | • Brand bar "ALIX Studio Lite".<br>• Mode chip: Simulation / Dry run on robot / Real robot + IP. REAL adds a persistent red banner with the word "REAL ROBOT".<br>• Arm chips: connected, mode, busy.<br>• **E-stop chip per arm**: CLEAR / E-STOP ENGAGED / RECOVERING / UNKNOWN / NOT CONFIGURED, with the IO number in the tooltip. Any state other than CLEAR disables motion buttons on both arms and offers **Recover**, a guided wizard (make the cell safe → release the button → state in plain words → clear errors → re-enable joints or pendant instructions → restore speed → 3D check of the move to the first point → re-home the pole).<br>• **STOP (software)** LEFT / RIGHT / ALL: never behind a dialog, never queued; the text "The hardware E-stop is the safety stop" stays beside it.<br>• Status line, toast, help, glossary.<br>• Operator/Engineer switch. | NFR-UX, NFR-THEME, NFR-SAFETY, R4 | P1, P3 |
| **Dashboard** (home) | • Matrix of 22 steps × LEFT/RIGHT (toggle: region families).<br>• Pipeline chips with glyph and word: Not taught, Awaiting review, Taught, Optimised, 3D OK, Dry run OK, Real tested, Uploaded.<br>• Flags: Legacy V6, Needs re-teach, Blocked, Stale.<br>• Counters per arm and lid/seat phase; "Teach next missing". | R1 | P2 |
| **Regions** | • PAGE 04/05 maps of commode C with clickable labels, including Floor Spillage.<br>• Steps grouped by lid/seat state.<br>• Unit card: picture with the area highlighted, plain title, arm badge, fixture state, tool, cleaning (target) time, pole/hand preset.<br>• "View original slide". | R1, R7 | P2 |
| **Teach** | Select arms → card → fixture check → **Prepare robot** (pole then hand, with read-back) → "I am holding the arm" → START (on-screen or foot switch) → countdown → STOP → save and check → review (mini plot, warnings) → Keep / Redo / Skip with a reason → auto-advance, which never moves the robot. Recovery runs at start-up. START is refused while an E-stop chip is not CLEAR. | R1, R7 | P3, P4 |
| **Optimise** | • **Optimiser**: Default (the prototype's algorithm, with fixes) or Time-optimal (TOPP-RA); TOPP-RA is greyed out with the reason when toppra is not available on this computer.<br>• **Target time** in seconds, pre-filled from the region catalogue and editable.<br>• Results: recorded time; **achieved time** (lines × 5 ms); **shortest feasible time** (T_min). When the target cannot be met, an amber chip "Closest feasible: X s — limited by joint N speed/acceleration in a–b s" with a **Use X s** button, and the binding window shaded on the plots.<br>• **Auto-optimise (recommended)**.<br>• Fine-tune per segment: Remove stops, Smoothness, time % (Default) or speed cap % (TOPP-RA), Keep stops / Hold N s.<br>• An L1–L6 pass/fail panel; other findings grouped Must fix / Should fix / Advisory, each with Show / Fix / Ignore-with-reason.<br>• At least 5 labelled undo steps, persisted.<br>• Plots in seconds and degrees with splitters, per-plot maximise and linked zoom.<br>• Compare with the raw file; Save version. | R2, R6, R11, R12, INT-TRAJ | P5 |
| **3D Check** | • 3D on the left, plots on the right, with a splitter.<br>• Both arms of the deck step; commode C in the step's state; the walls of the selected venue; bin stowed/out/lowered.<br>• Transport 0.25×–4×; **stop at first problem**.<br>• Red links and faces; 7 joint markers and a band on the plots; event list with Previous/Next.<br>• A **"What was checked"** panel that names the venue file; pad trail; "Back to editing" at the same rows. | R3, R5, R10, R11, R14 | P6 |
| **Test on robot** | • **Send to robot**: background queue, skips unchanged files, ETA.<br>• **Slot picker**: any program ID 1–100, with what each slot holds now (from the controller list and the ledger). Choosing an occupied slot opens a replacement confirmation that shows the current program; batch upload sets a program ID per file. Soft warnings only.<br>• **Dry run on robot.**<br>• **Real run**: the confirmation names arm, mode, IP, region, program ID and transit size, and shows a checklist with deck pictures.<br>• Progress in words; STOP (software).<br>• Result: Finished / Stopped at about X % / Interrupted — E-stop / Not confirmed.<br>• Verdict buttons (Looks good / Hit something / Missed a spot / Other).<br>• Recovery; "Back to editing". | R4, R5, R13, INT-CTRL | P7 |
| **Program** (Release 2) | • Order each arm's units; automatic grouping into programs, each split explained.<br>• Link moves, transit checks, "Go via the safe pose".<br>• Dual blocks: Both arms together (equal line count, binding arm named) / Left first, then right / Checkpoint.<br>• Windowed clearance; timeline with bands and pace-setter.<br>• Export of v53-compatible workflows; slowed first-run kit; Upload all to the chosen program IDs; run. | R8, R10, INT-CTRL | P9 |
| **Adjust position** (Engineer, Release 2) | • Offset in plain words, or by touch-off.<br>• Ghost preview; solve; span report with press-axis effects; then the ladder. | R9, R14 | P10 |
| **Robot console** (Engineer) | Manual pole and hand (6 channels), slot list 1–100, upload and run of one file, run mode, E-stop IO details per arm (IO number, level, enable flags, register 302 if enabled), logs with translated codes, global points (blocking calls, engineer only) | R4, R7, INT-CTRL | P3, P7 |
| **Settings & About** (Engineer) | IPs and ports, SDK folder, venue file selection and its verification record, E-stop IO number, active level and last-drill date per arm (with an overdue warning), default optimiser, executor (Stage A/B), gate strictness, override log, versions (Lite, ALIX commit, bundle, architecture, runner sha, SDK, toppra), update / rollback, support bundle | R4, R13, NFR-OFFLINE-DEPLOY, NFR-SAFETY | P1, P3 |

Shared front-end code:
- **Copied from the Studio**, with provenance headers and drift tests:
  - shell components (`WE/editor.js:5431-5467, 9088-9104, 13786-13832, 7580-7645, 8389-8523, 7538-7559, 7833-8040, 19150-19166`);
  - viewport and robot overlay (`:309-645, 667-703, 743-830, 12352-12437`);
  - Dual-Arm Collisions playback (`:14272-14600, 8749-8767`);
  - Board CSS and patterns (`WE/editor.css:1507-1528, 1976-1998`).
- **New:** `plotTheme()` driven by tokens, a joint palette that avoids the state red, green and amber and the brand blue, the E-stop chip and recovery wizard, and the screen modules.

### 4.4 The robot executor seam: Stage A and Stage B

Every screen talks to one interface. No screen knows which backend is behind it:

```
RobotBackend: connect(arm, ip, port) · identify() · status() · estop_state() → events · run_mode() / set_run_mode()
              list_programs() · upload(track, slot 1–100, name, replace_confirmed) → verified · run(slot, transit_pct) → events
              stop(arm | all)  [never queued] · pole(mm) · pole_stop() · hand(counts6) · hand_freeze()
              drag_start() · drag_stop() · drag_save(path) · joints()
```

**Non-negotiables for every backend.** These come from runner v14, ALIX and decision F4; a test checks each one.
- `.proj` bytes are CRLF with header `{"file":7}` and a Folder line whose `type` equals the point count. They must be byte-identical to R14 `write_project` for the same input.
- Uploads are store-only: `only_save=1`, `auto_start=0`. Any program ID 1–100 is accepted; an occupied slot is replaced only with a confirmation token bound to that slot and the program read from it.
- After upload, the stored size and name are read back and checked.
- A file with any bad line is refused whole.
- Before a run, the program refuses an empty slot, an arm error, a running program, or an E-stop that is not CLEAR.
- An E-stop seen at any time during a run makes the result INTERRUPTED, never COMPLETE, and no further sequence step starts.
- The global speed is restored on every exit path, including a killed process.
- The run mode is verified by read-back.
- The transit (approach) runs at 30 % by default.
- There is one command source per arm.

**Stage A (Release 1): runner v14 through `lite_runner_launch.py`**
- R14 is vendored with **one small E-stop patch** and sha256-pinned; `hand.py` and `pole.py` are vendored **byte-identical**. Lite refuses to start robot features if a sha256 differs from its pin, so v13 or another modified copy is refused. The unpatched R14 stays in the test suite as the oracle for everything outside the patch.
- **Why the patch.** Runner v14 watches only run state and error codes (`R14:122-138, 219-271, 641-700`), and an IO E-stop most likely leaves the error code at 0. A run cut by an E-stop would therefore very likely print "RUN COMPLETE … no error", exit 0, and let a sequence start its next step (E-stop research §2.1). During a Stage A run the runner holds the arm's only connection, so only the runner can watch the E-stop input. The patch: read the E-stop IO in `arm_state()` and in the run and sequence loops; refuse to start with a clear reason while it is held; report "interrupted" instead of "RUN COMPLETE"; never start the next sequence step after an E-stop; print parseable `ESTOP ENGAGED <side> io=<n>` and `ESTOP CLEAR <side>` lines. The diff is reviewed, kept to these items, and tested with the fake SDK.
- The wrapper (about 50 lines, Lite-owned, tested) does five things:
  1. It checks the pins.
  2. It starts a stdin reader. A `STOP` line, or end-of-input when the Lite backend dies, calls `_thread.interrupt_main()` exactly once. That raises the same `KeyboardInterrupt` that SIGINT does, so the runner's existing slow stop, settle and speed restore run.
  3. It runs the runner with `runpy.run_path(…, run_name="__main__")`, so the pinned runner bytes stay untouched.
  4. After the runner exits, and only then, it may read the final joints, run state, error codes and E-stop input through `alix_dispatch` on a fresh connection. That read is sequential, so the arm still has one command source.
  5. It prints one `LITE POSTRUN {…}` line.
- **Passed every time:** `--sdk`, `--side`, `--ip`, `--port`, `--yes` (the last only after Lite's own confirmation), plus explicit speeds and timeouts.
- **Status reads, E-stop polling and the identity check** run through `robot.status`, and only when no runner is active on that arm.
- **E-stop drill before REAL mode.** E-stop drill items DR1, DR3, DR9, DR10 and DR11 (of DR1–DR14, numbered in the E-stop research §5.6) must pass on the site's arms before operators use Lite in REAL mode. The results and the drill date are stored per arm in the site settings.
- **Fallback stop.** If the runner has not printed its slow-stop line and exited within about 8 s, Lite kills it and then runs `stop_program` (slow stop → pause → stop, each verified) on a fresh connection, plus pole stop and hand freeze. If that fails too, the screen says "Press the hardware E-stop now". P0 checks whether the controller accepts the fresh connection.
- **Known limits.**
  - STOP (software) cannot interrupt blocking SDK calls (drag playback, global MoveJ/MoveL), so those stay engineer-only. The hardware IO E-stop is the safety stop in every case.
  - Whether a software slow stop or stop ends a running stored program is unproven (hardware gate G4 not yet run, `DP/emulated_program.py:51-56`); it is tested in P0.
  - There is no live joint stream while a program runs. The 3D view mirrors the *planned* motion against elapsed time, labelled "planned", and shows the actual stop pose afterwards.
  - Linux only, on both x86-64 and aarch64; the SDK ships `linux_x86` and `linux_arm` libraries (E-stop research §5.1).

**Stage B (Release 2): one agent process per arm on `alix_dispatch`**
- The agent holds one persistent connection and runs non-blocking verbs in slices of 50 ms or less, checking a stop flag between slices. It polls the E-stop input at 5–10 Hz on the same connection.
- A heartbeat thread runs the stop ladder if the web process is silent for 2 s while the arm is moving.
- Its verbs come from `DP/track.py`: verified stop through `stop_program`; `global_speed_guard`; run-mode read-back; one-session batch upload with skip-unchanged; UDP capture (`DP/recording.RunRecorder`) with `verify_replay_from_frames` for exact real-arm collision rows; a live mirror. Drag teach and global points are ported from R14.
- **Switch criteria** (all required; signed by the responsible engineer). When met, Stage B becomes the default executor; Stage A stays selectable for one release and remains the test oracle.
  1. The same fake-SDK and emulator suite passes on both backends.
  2. `.proj` parity holds.
  3. On hardware, on both arms: G4 behaves as characterised in P0, and the arm is still within 2 s of a stop; the speed is restored after an agent kill; mode read-back works; one drag-teach session matches R14; a batch upload of 7 files uses one connection and at most two list reads; a 10-minute run with a hidden browser tab causes no false heartbeat stop; E-stop drill cases 1, 3, 9, 10 and 11 pass again with the agent.

### 4.5 The 3D check: world, contact policy and verdicts

- **World** (decision F5: built from the URDF and the existing scene files; no cell-calibration project).
  - **Robot:** both arms with **full hand link sets**, the UGV and the poles from the URDF. The Studio's palms-only set misses a real right-thumb-to-left-wrist contact on V6 data (C3 §4.4 #12). Curated self pairs (0.71 ms per arm), plus the right wrist camera. The SRDF-disabled thumb-vs-camera pairs are re-enabled.
  - **Commode:** commode C (decision F3) from the `commode_c` scene: body, lid and seat posed at the unit's lid/seat state through `fixture_manager`, at the scene's authoritative pose relative to `base_link` (`CCS:29-37`). The commode meshes inside the venue files are not used.
  - **Room:** a single cubicle selected in Settings (decision D21): 1400 mm wide by default (Lite's own mesh, built like ALIX's 800 and 1000 mm planning-scene meshes with inner side faces at ±0.70 m) or 800 mm (`VENUE/washroom_800mm_wide_scene.yaml`), loaded through `load_wall_manager` (`WS/plan_scene_validation.py:117-156`). Wall collision is the one item ALIX does not enforce today; in Lite it gates like any other collision. The selection is verified once by replaying known-good recordings, which must show no wall contact; the record (venue file, sha256, recordings, result, who, date) is stored in the project.
  - **Bin:** stowed/out/lowered from the scene, posed through SRDF `BIN_*`. Never the hard-coded LOWERED.
  - **Seeding:** pole and hand are always seeded from the recording's own sidecar values.
- **Contact policy.** A pair counts as **intended pad contact** only when two things hold:
  - one link is in `contact_links.yaml[<pad frame>]` for the region's hand preset;
  - the other object is the fixture part the region cleans.

  Every other pair is a **collision**, classified with Lite's own plain-name table, because `classify_pair` reads bin covers as "lid". Contact deeper than 20 mm shows as Needs attention.
  - The ALIX list was derived for the `open_tenth` hand. With the V6 GRIP fist, the flexed finger links make the lid contact (C3 §2.7). The requester rules on the GRIP list (D8).
  - Untaught motion (transits, link moves, pole and hand moves, and the recovery move after an E-stop) uses a **strict** check with no contact allowance (C5 §7.5).
- **Sampling.** Gating sweeps check **every row** (about 2–3.5 ms per row, so about 30 s for a 60 s pair). dt 0.02 s is a correctness parameter, not a tuning knob. Quick and windowed checks are labelled "quick" or "preview".
- **Two-arm timing.** Arm-vs-arm results are reported twice: at the nominal alignment, and over a drift window built from the measured start lag and run-to-run timing spread (`DAV:912-973`). A contact that can occur only inside the window reads "possible if the arms drift". A retimed pair (new target time or TOGETHER alignment) is always re-checked, because a verdict is tied to both tracks' sha256 (`DAV:18-22`).
- **Verdict states.**
  - **No problems found**, always with a "What was checked" list (including the venue file) and the minimum clearance.
  - **n problems.**
  - **NOT CHECKED**, when the world is incomplete, no venue is selected or the selection is not verified, or the input is stale. It is never shown as a pass.

### 4.6 Data model and project folder layout

```
~/ALIX-Lite/                                   data root (settable; never inside the install folder)
  settings.json                                engineer and site settings: IP/port per arm, SDK folder, executor,
                                               default optimiser, E-stop IO {io, active_level, last_drill} per arm, UI prefs
  logs/YYYY-MM-DD.jsonl                        application log
  projects/<project>/                          one per site and fixture, e.g. commode_c_siteA
    project.yaml                               fixture (commode_c); scene {commode_c scene sha256, venue file + sha256,
                                               venue verification ref}; bin policy; arms {ip, port, identity};
                                               pins {lite, alix_commit, bundle_sha256, arch, runner_sha256,
                                               sdk_version, toppra_version, em_version}
    scene/venue_selection.json                 venue file, sha256, verification record (known-good recordings
                                               replayed, wall contacts found, who, date)
    registry/regions_<deck>.yaml               the unit catalogue (42 deck units, 34 in release 1), supervisor-owned, versioned with the deck
    registry/deck/<deck>/                      slide renders, per-arm crops, media, overlay boxes
    units/<UNIT_ID>/
      raw/<KEY>__RAW__<yyyymmdd-hhmmss>.txt     read-only; bytes exactly as saved by the controller
      raw/<…>.SOURCE.json                              ALIX provenance keys + "lite" block
      edits/recipe.json                                current recipe + labelled history (≥ 5 undo, redo)
      opt/<ID>__OPT_v<k>.txt                           derived; immutable; CRLF integers
      opt/<…>.cert.json                                certificate: hashes, recipe, optimiser, target/achieved/T_min,
                                                       binding limit, L1–L6 and G5, durations, versions
      sim/<fingerprint>/{world.json, verdict.json, events.json}    3D verdicts with world manifest
      runs/<run_id>/{run.json, events.jsonl, stdout.log, capture/, verdict.json}
      events.jsonl                                     append-only: kept / redo / skip / sim / dry run / real / upload
    programs/<name>/                                   Release 2
      LEFT_<name>.txt, RIGHT_<name>.txt, manifest.json, cert.json,
      workflow.v2.json, workflow_LEFT.v1.json, workflow_RIGHT.v1.json, timeline.json, sim/
    controller/ledger_<left|right>.json                slot (1–100) → {track_sha, proj_sha, bytes, points, stored_name,
                                                       uploaded_at, verified, replaced {name, bytes}, confirmed_by}
    ladder/ladder.jsonl                                DP/ladder rows + Lite OFFLINE rung + overrides with reasons
    audit/events.jsonl                                 every motion, confirmation, replacement, stop, E-stop event,
                                                       override, run mode, with sha + IP
    sessions/<session_id>.json                         teach-session resume state and child process IDs
    legacy/V6/…                                        read-only import; segment → unit mapping with confidence
    trash/                                             engineer deletions (never automatic)
  practice/<project>/…                                 same layout; fake recorder only; never counted as taught
```

**Unit record** (abridged; full schema in C1 §1.8.2):
- `id`, deck aliases, `title`, `arm`, `type` (surface / mechanism / tool / glove / idle), `family`, `part`, `step`, `paired_with`, `sync`, `role`;
- `fixture_state`, and **`fixture_part`**, the part being cleaned, which drives the contact policy;
- `tool`, `pad_frame`, `teach_time_s`, **`target_time_s`** (the region's cleaning time from the catalogue, the optimiser's default target), `max_dev_mm` (optional), `pole_mm`, `hand {preset, counts[6]}`, `prep_order`;
- `exec_budget_mm`, `substeps` with `hand_events`, `image`, `blocked`, `legacy`;
- the latest optimisation result per version: `optimiser`, `target_time_s`, `achieved_time_s`, `t_min_s`, `binding_limit`.

**Identities:**
- `file_sha`: the sha256 of the bytes on disk. Used for provenance.
- `track_sha`: the sha256 of the canonical point lines, ignoring line endings. Used for de-duplication.
- `proj_sha`: the sha256 of the `.proj` built by the R14-identical writer. Used by the upload ledger.
- **Fingerprint**: `proj_sha : context_sha`. Used by the ladder. The context includes the scene (commode_c and venue file sha256).

### 4.7 File formats

| File | Format | Notes |
|---|---|---|
| Raw recording `.txt` | NDJSON, one `{"point":[j1..j7]}` per line, integer millidegrees | Written by the controller SDK (`rm_save_trajectory`) to a temporary name. Lite checks it, renames it, makes it read-only and never edits it. LF or CRLF are read. |
| Derived `.txt` (`OPT_v<k>`, programs) | Same NDJSON, integers only, CRLF | Must pass R14 `read_track` and `DP/track.parse_track` unchanged; written as `{"point":[7 ints]}` as ALIX does (`TT:737-745`) |
| `.proj` (upload only) | `{"file":7}` + Folder line (`type` = N) + points, CRLF | Built only by the R14-identical writer, in a short temporary path (under 300 bytes) |
| `<stem>.SOURCE.json` | ALIX keys `side, frame, pole_m, hand_state, hand_counts, hardware, taught_by, date, units, note`, plus a `lite` block | The `lite` block holds region, sub-step, deck version, fixture and bin state, IP, tool frame, targets and measured values, target and elapsed time, both point counts, the measured record period, sha256, attempt and versions. Readable by `TT`. |
| `regions_<deck>.yaml` | Unit catalogue (§4.6) | Validator refuses duplicate IDs, missing fields and unknown states |
| `recipe.json` | `alix_lite_recipe_v1` | Holds: source {file, sha256, lines, clock_s 0.005}; region settings; **optimiser** {`default` or `toppra`, parameters, library versions (numpy, scipy, toppra)}; **`target_time_s`** and its origin (catalogue or operator); segments {start/end in raw lines, time % or speed cap %, pinned time, overrides}; blend; limits {L1–L6 values: G1 margin 0.5°, rate 0.97, acceleration 600 °/s² × 1.02 over 50 ms, step 10°, G4 10 mm or MAX DEV, end poses; executed budget; EM version}; result {**`achieved_time_s`**, `t_min_s`, met, **`binding_limit`** {arm, joint, quantity, time window, share}}; history |
| `*.cert.json` | Certificate | Holds: file/track/proj sha; parent sha; recipe sha and a copy of the recipe; optimiser, `target_time_s`, `achieved_time_s`, `t_min_s`, `binding_limit`; L1–L6 values with thresholds and pass flags; advisories (G5, jerk, pad speed, Butterworth acceleration reading); predicted upload time; versions and architecture |
| `world.json`, `verdict.json` | World manifest and result | Objects with counts, fixture state, bin, commode_c scene sha256, venue file and sha256, venue verification status, contact policy and checks run; events {kind, parts (plain and technical), first/last row per arm, time, depth, nominal or window} |
| `run.json` | Run record | Holds: fingerprint, slot, arm, IP, target (dry run or real), mode found and read back, start/end, exit code, codes, final joints, completion witnesses, stop index, E-stop events (IO state, enable flags, power, brakes, err, run state, with timestamps), speed-restore result, result (Finished / Stopped / Interrupted — E-stop / Not confirmed), verdict, note |
| `workflow_*.v1.json` | `realman_workflow_sequence_v1` (`R14:1668-1763`) | Exported for runner v14 and for v53 Page 02. A checkpoint is encoded as a positive WAIT on both arms, which keeps v53 compatibility. |
| `workflow.v2.json` | Lite internal: v1 steps + explicit `checkpoint{message}` + unit references + `sync` | Migrates v1 (`speed` → `transit_speed`) |
| `timeline.json` | Dual timeline | Per-arm events with durations (programs at (lines − 1) × 5 ms, other steps from measured laws); blocks; skew band (the file V6 never shipped) |
| `ledger_<side>.json` | Upload ledger | The controller verifies only name and size, so the ledger ties each slot (1–100) to a sha and records what each upload replaced |
| `ladder.jsonl` | `DP/ladder.py:63-83` rows + Lite `OFFLINE` rung | Append-only |
| `venue_selection.json` | Venue choice and its verification | Venue file name and sha256, known-good recordings replayed, wall contacts found, who, date. Every verdict names it. |

### 4.8 Deployment and packaging

- **Target** (decision F1).
  - **Now:** the operator PC on the robot network (x86-64, Linux). **Production:** the robot's Jetson AGX Orin (aarch64, Ubuntu 24.04, `ALIX/SETUP.md:76-82`). **Release 1 ships both bundles**; the aarch64 bundle is required for production, and every golden test runs on both architectures (bytes may differ by up to 1 millidegree across library versions and architectures, `ETR:417-427`).
  - On a headless Jetson, the page is opened from the operator PC through the authenticated remote mode (§9.7).
  - Robot I/O runs on Linux only in v1. The RealMan SDK ships `linux_x86` and `linux_arm` `libapi_c.so`, so the same robot code runs on both.
  - Simulation and optimisation could also run on Windows later; this is not planned.
- **Bundle contents** (`alix-studio-lite-<ver>-linux-<x86_64|aarch64>.tar.gz`):
  - relocatable CPython 3.12 (python-build-standalone);
  - a wheelhouse: numpy, scipy, pyyaml, trimesh, python-fcl, pycollada, lxml, networkx, toppra 0.6.10;
  - `alix_studio_lite`;
  - the **vendored ALIX copy** (`vendor/alix`): one ALIX commit in repo layout, listed with sha256 in `vendor/MANIFEST.json`, permissions normalised to 0644. It holds:
    - the `alix_workspace`, `alix_taskgen`, `alix_dispatch` and `alix_emulator` sources;
    - the taught-track tools (`TT`, `CT`, `PMT`, `DAV`) and the exp01 modules the TOPP-RA chain imports (`LS`, `ETR`, `teachlab`), plus `ALIX/Resource/tracks/taught/EXECUTION_MODEL.json`;
    - the URDF, and the meshes of all six referenced packages, including `realsense2_description`;
    - the SRDF and `initial_positions.yaml` files;
    - the `commode_c` scene and STLs; the 800 mm cubicle venue (`VENUE/washroom_800mm_wide_scene.yaml`, `washroom_800mm_planning_scene.stl`) and Lite's 1400 mm cubicle files;
    - `contact_links.yaml`, `hand_units.py`, `design/_tokens.css`;
    - the re-baked overlay;
  - vendored three.js r128 and Plotly 2.35.2;
  - Inter and JetBrains Mono woff2 with their OFL licences;
  - runner v14 with the E-stop patch, `hand.py` and `pole.py` (pinned), and the launch wrapper;
  - the RealMan SDK if its licence allows (D20). Otherwise Settings locates it once.
- **aarch64 wheels** (checked on PyPI, 13 Oct 2026):
  - **python-fcl:** 0.7.0.11 publishes a cp312 manylinux aarch64 wheel. Whether this is the version ALIX uses must be checked when the wheelhouse is built.
  - **toppra:** 0.6.10 publishes x86-64 wheels only and no source package; the newest source package is 0.6.3. The aarch64 build compiles toppra 0.6.10 from its git tag (Cython, gcc, numpy headers) in an arm64 build job or on the Jetson. One solve is timed on the Orin early in the project.
  - numpy and scipy have aarch64 wheels. The other pure-Python packages need no build.
  - Lite must start and work without toppra: if `import toppra` fails, TOPP-RA is greyed out ("TOPP-RA unavailable on this computer — using the default optimiser") and the self-test reports it as a warning, not a failure.
- **Install.**
  - `install.sh` needs no root. It unpacks to `~/.local/opt/alix-studio-lite/versions/<ver>/`, points a `current` link at it, adds a desktop entry, creates `~/ALIX-Lite/` and runs the self-test. It refuses a bundle built for the other architecture.
  - Uninstall never deletes data.
- **Update.** "Install update from file" checks the sha256, unpacks beside the current version, self-tests, switches on the next start and offers a **one-click rollback**. Schema migrations back up the data first.
- **Start-up.** UI interactive in about 3 s (target; 10 s hard limit). The simulation worker warms up in the background (15–30 s) behind a "3D getting ready" chip. Both start-up figures are to be re-measured on the Jetson.
- **Self-test at every start (fail closed).** It checks that:
  - the manifest hashes match;
  - there are 0 unresolved `package://` URIs;
  - the expected collision link and mesh counts are present;
  - the SRDF loads with its 1,824 disabled pairs;
  - commode body, lid and seat exist for all three lid/seat states;
  - the selected venue file's wall mesh loads through `load_wall_manager` (if no venue is selected, wall results read NOT CHECKED);
  - `hand_units.py` loads;
  - FK at fixed joint vectors is bit-identical to stored values;
  - the overlay's bin top is 0.398 m (BinShort);
  - the optimisers run on a golden file (default always; TOPP-RA when toppra imports).

  A failure shows a plain page and blocks both the 3D gate and robot features.
- **Version stamps.** The Lite version, architecture, ALIX commit, bundle sha, runner sha, SDK version, toppra version, EM version and venue file sha go into every certificate, run record and support bundle.

---

## 5. Reuse map

Modes:
- **Import:** run the original code from the vendored ALIX copy in `vendor/alix` (decided 13 Oct; no ALIX checkout at run time). Lite lives in `/home/newtonjeri/alix_studio_lite_ws/alix_studio_lite/` and is not a package inside the ALIX monorepo (decision F7).
- **Vendor:** a copy pinned by hash; byte-identical unless a named patch is listed (only R14's E-stop patch).
- **Copy:** an adapted copy with a provenance header and a drift test.
- **Port:** a re-implementation with golden parity tests, deleted once upstream offers an importable version.
- **Reference:** ideas or data only.

| Component | Source path | Mode | Adaptation |
|---|---|---|---|
| FK, Jacobian, IK, SRS | `WS/urdf_kinematics.py:218-344`, `WS/ik.py:150-334`, `WS/srs.py`, `WS/srs_ik.py` | Import | Pole locked. R9 never calls `IKSolver.solve`, whose random restarts hop between IK branches. |
| Collision model, package index, SRDF states | `WS/collision.py:55-60`, `WS/package_index.py:27-121`, `WS/srdf_states.py` | Import | Bundle URDF passed explicitly; `.git` marker; self-test |
| Dual-arm sweep, plan frames, arm-vs-arm separation | `WS/dual_arm_plan_collision.py:487-764, 1089-1283` | Import, plus seam S1 (`body_links`, `bin_state`, `pole_m`, in-memory timelines, explicit index); otherwise a Lite wrapper with a parity test | dt 0.02; full hands; keep `joint_timeline`; no hard-coded LOWERED bin (`:634-644`) |
| Walls | `WS/plan_scene_validation.py:117-270` (`load_wall_manager`, `PlanWallChecker`); the 800 mm cubicle `VENUE/washroom_800mm_wide_scene.yaml` + `washroom_800mm_planning_scene.stl`; a 1400 mm cubicle that Lite builds the same way (ALIX's 1400 mm venue is the whole washroom) | Import (honours `ALIX_REPO_ROOT`, `:31-59`); venue files as bundle data | The only scene item ALIX does not enforce today; Lite gates on it (decision F5). Cubicle selected in Settings (1400 mm default, 800 mm option; D21) and verified once by replaying known-good recordings (the 800 mm walls gave false wall hits in the wider cell, C3 §4.4 #12). Only the `washroom_structure` mesh and pose are taken from the venue file (`:117-156`); depth reported as approximate |
| Commode world | `TG/body_collision.py` (`fixture_manager :401-464`, `make_pose_filter :499-952`), `TG/scene_pose.py`, `TG/fixture_parts.py`, `TG/transforms.py`, `TG/paths.py`, `TG/constants.py` | Import (whitelist + import-boundary test) | Taught-track entry point (synthetic config); strict mode; hand joints from counts; bin state; rest baseline surfaced; commode C at the `commode_c` scene's authoritative pose (`CCS:29-37`, decisions F3 and F5). Done through seam or wrapper. |
| Singularity screens, pad axes | `TG/singularity.py`, `TG/glove_frames.py` + `TG/config/glove_frames.yaml` | Import | Warnings (R9, R12); press axis (R9, R14) |
| Transit sampling rule | `TG/approach_transit.py:60-73, 178-200` | Import (rule and "no world = not checked") | Used for transits and link moves |
| Fixture-state timeline logic | `TG/schedule_guards.py:8-37, 196-236, 449-600`; atomic write `:59-104` | Port the logic; copy the helper | Teaching order and lid/seat prompts (R1); program checks (R8) |
| Contact policy | `ALIX/ros/orchestration/alix_tasks/config/contact_links.yaml` | Bundle data | A list per hand preset (GRIP ruling, D8) |
| Hand scale | `ALIX/ros/hw/alix_hw/scripts/hand_units.py:30-60` | Bundle data (loaded by `DP/backends.py:31-47`) | Counts → radians. Never the `sim_bridge` table. |
| Stowed-bin foul band | `ALIX/ros/orchestration/alix_orchestrator/alix_orchestrator/core/task_config.py:58-124` | Copy the constant and function, with provenance and a parity test | Checks the whole travel interval, not only the target; union with the band computed at start-up from the URDF bin geometry (decision F5; `ALIX/core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf:188-359`) |
| Controller I/O library | `DP/connect.py`, `DP/track.py`, `DP/runmode.py`, `DP/ladder.py`, `DP/peripherals.py`, `DP/recording.py`, `DP/staging.py`, `DP/emulated_program.py`, `DP/backends.py`, `DP/limits.py` | Import from the bundle (seam S5 adds a `pyproject.toml`) | Stage A: status, identity, post-run read, fallback stop, ladder. Stage B: the agent. |
| Emulator | `ALIX/core/python/src/alix_emulator/` | Import (developers and CI only) | Plus a drag-teach seam fake |
| Runner v14 + helpers | `R14`, `UC/hand.py`, `UC/pole.py` | Vendor (sha256-pinned): `hand.py` and `pole.py` byte-identical; R14 with one small reviewed E-stop patch (§4.4), the unpatched R14 kept as test oracle | Started through `lite_runner_launch.py`; flags always explicit; patch reads the E-stop IO, refuses to start while it is held, reports "interrupted" instead of "RUN COMPLETE", stops a sequence, prints `ESTOP ENGAGED/CLEAR` lines (E-stop research §5.2) |
| IO E-stop status and recovery calls | RealMan SDK `RRI:2882 rm_set_io_mode` (read only: Lite never writes IO modes), `RRI:2927 rm_get_io_state`, `rm_clear_system_err`, `rm_set_joint_clear_err`, `rm_set_joint_en_state`, `rm_set_lift_speed(0)` (`DP/peripherals.py:189-199`); optional Modbus input register 302; ALIX field notes `ALIX/docs/TEST_PROCEDURE.md:216-242, 453` | Import (through `DP` where it wraps the call) | E-stop chip per arm, motion interlock, INTERRUPTED result and guided recovery (decision F4). Never from `arm_current_status == 9` alone (`ALIX/ros/hw/alix_hw/src/rm_side_system.cpp:1856-1860`). Read-back after every clear, because ret 0 does not prove the clear (`DP/program.py:236-240`) |
| v53 dual planner | `V53:9619-10017` (`_page2_program_chunks`, `_page2_build_program_rule_plan`, checkpoint stage `:9870-9905, 10141-10188`) | Port with parity (V6 FINAL pair → 7 stages) | Explicit `checkpoint` step; cross-arm abort added |
| v53 behaviour | `V53:3802-3810` (REAL confirmation), `:6026-6055` (one-click stop, disabled after one click), `:5654-5709` (barrier launch), `:11, 4279, 4472-4476, 8771` (batch upload with per-file program IDs; replacement confirmation showing existing programs); `R14:324-325` (program IDs 1–100) | Reference | Wording and behaviour only. Lite uploads to any program ID 1–100 and replaces an occupied slot after a confirmation (decision F6); warnings, no range refusal. The Tk UI and its palette are not ported. |
| TOPP-RA chain for taught paths (time-optimal optimiser) | `TT:891-912 smooth_taught` (5 Hz), `TT:915-951 strip_pauses` (R4); `LS:78-104 TaughtPath`, `LS:139-169 arclen_smooth` (30 mm), `LS:282-324 retime_toppra` (0.95 × ratings, 600 °/s², Interpolation scheme, seidel, zero end speed); driver recipe `ETR:306-330`; optional post low-pass `ETR:200-215`; sampling `PMT:375-394` (`resample_clock`); writer form `TT:737-745`; hardware record `PIPE:17-40, 42-55, 588-591` | Import through one adapter module; the retime function is a Lite-owned port of about 40 lines with explicit arguments and a parity test (FILE_01.1 → 3839 lines within 1 millidegree), because `LS.RATE_DEG_S` and `LS.ACC_DEG_S2` are module globals the emitter changes (`ETR:304-305`) | Called per side (the ALIX emitter is left-arm only, `ETR:71`; `TaughtPath` takes the side). Post low-pass off by default. Not ALIX's slow ladder, which re-solves with scaled limits (`ALIX/tools/taught_pipeline.py:117-121, 140`); Lite hits the target by uniform time scaling instead. Right-arm files get a supervised first run (`PIPE:556-557`) |
| toppra library | PyPI `toppra` 0.6.10 (the version ALIX's taught pipeline was tested with, `ALIX/tools/requirements_taught_canfd.txt:16`) | Wheel in the bundle: x86-64 from PyPI; aarch64 compiled from the git tag (PyPI has no aarch64 wheel and no source package for 0.6.10, checked 13 Oct 2026) | Guarded import: without it Lite runs with the default optimiser only |
| Taught-track maths and gates | `TT:94-138, 420-716, 748-832, 880-953, 958-991, 1083-1165, 1377-1483, 1817-1852, 4149-4162`; `CT:278-336, 1646-1750`; exp01 `predict_track_replay.py:76-136`; `ALIX/Resource/tracks/taught/EXECUTION_MODEL.json` | Import after seam S3; otherwise port with golden parity | Pass `clock_s=0.005` (the default is not 5 ms); local rating fit only |
| `.txt`→plan shim and timing envelopes | `DAV:150-224, 501-731, 862-973` | Import after seam S2; otherwise port | 7-integer check; in memory; envelopes re-fitted on Lite runs |
| Join primitive | `PMT:142-169` | Import | Integer millidegrees; gaps go to link moves instead of being refused |
| Stop-policy probe, dry in-place track | `DP/track.py:3273-3300`; `CT:22, 3729` | Run in P0 (commissioning) | Repeated after firmware changes |
| Studio server scaffold, jobs, page assembly | `TG/server.py:5178, 5961-6253, 6297-6469`; `TG/studio_generate.py:537-600, 861-1060`; `TG/scripts/visualize_generated_config.py:530-568` | Copy | Loopback only; token header; Host/Origin check; needs table with `robot_io:<arm>` |
| Robot geometry payload | `TG/server.py:1864-1998` | Copy, or import after seam S6 | Thin loader (no `_ik_context`, which imports X2 code, `TG/robot_model.py:179`); re-baked overlay |
| Theme tokens | `ALIX/design/_tokens.css`; `WE/editor.css:1-189, 1714-1724` | Reference + copied parity test (`TG/../test/test_design_tokens.py`) | "Lite additions" block; accessible variant tokens; distinct arm-identity colours |
| Shell, viewport, collision playback, Board | `WE/editor.js` ranges in §4.3; `WE/editor.css:331-341, 1409-1528, 1976-1998` | Copy + drift test (the Studio's own `_function_body` helper) | Token colours instead of literals; Cancel focused in motion confirmations; stop at the first collision; playback speeds |
| Studio guard tests | `test_design_tokens.py`, `test_studio_theme.py` (subset) | Copy | Pointed at Lite; scan JS hex and `rgb()`/`rgba()` literals too |
| Layout audit | `ALIX/tools/layout_audit.py` | Reuse | Lite pages at 1366×768 and 1920×1080 |
| Overlay bake | `ALIX/tools/meshes/bake_visual_overlay.py` | Run (seam S4) | BinShort and the wrist camera present |
| Launcher preflight | `ALIX/tools/run_studio.py:92-205` | Copy the pattern | Isolated Python; single instance |
| Fake-controller tests | `ALIX/core/python/alix_dispatch/test/test_rm_upload.py` | Pattern | A fake `Robotic_Arm` passed with `--sdk` runs the unchanged runner in CI |
| Trajectory prototype | `HTML` (stepper `:609-704`; pure functions `:1417-1620`, of which the optimiser is trim → smooth → scale `:1438-1616, 1676-1724`; compare `:1260-1322`; report `:1732-1755`; box-select `:872-892`; cursor `:1134-1142`) | Port: the algorithm becomes the **default optimiser** (decision F2), Python canonical + JS preview, with three minimal fixes: odd-reflection edge padding, a 0.25 s speed blend at segment boundaries, window 41 with a hard G4 path-change check. Copy and restyle the UI patterns | The prototype has no limit checks (`HTML:1676-1810`); Lite adds L1–L6 and the target-time rule. ALIX tokens; seconds and degrees; vendored Plotly; the 8 UI-state bugs fixed (source map 07) |
| Deck tooling | Deck-analysis scripts written during this analysis; the ID scheme in the requirements' Appendix A | Promote into an offline `deck_import` tool | Draft catalogue plus a diff on re-import; LibreOffice not needed at run time |
| Provenance schema | `ALIX/Resource/tracks/taught/source/README.md:8-27`; `TT:310-342, 3624-3634` | Reuse the schema | Adds the `lite` block |
| V6 package | `V6/` | Legacy data, golden tests | Not uploadable until re-certified through Lite's gates |
| **Not reused (X1/X2)** | Studio Regions painting, Workbench/Generate, Board "Generate missing", Workspace tab, atlas, reach and manipulability overlays, Gizmo, Schedule & Run back end; 35 X1 routes and 9 X2 routes (C7) | — | An import-boundary test, a route allow-list test and a release-content test enforce this |

**Upstream seams in ALIX.** These are small and keep today's defaults. They need the ALIX owner's agreement (D14). Lite works without them by porting, with parity tests, and deletes each port when its seam lands. Size uses the scale S/M/L/XL; every seam here is S.

| Seam | Change | Why | Size |
|---|---|---|---|
| S1 | `compute_dual_arm_plan_collisions`: add `body_links`, `bin_state`, `pole_m`, an in-memory timeline entry point and an explicit `PackageIndex`/URDF root. Guard: the Studio's collision payload stays byte-identical. | Bin control; commode, walls and fingers; warm re-checks | S |
| S2 | Promote the `.txt`→plan shim (`DAV:531-692`) into `WS/taught_plan.py` | One source, with mandatory seeding | S |
| S3 | Promote `strip_pauses`, `smooth_taught`, `uniform_millideg`, `capped_millideg`, the ceilings, `deviation_mm` and the exp01 rate/position/step reports into an importable module | ALIX's own rule: "Imported, never copied" (`TT:83-84`) | S |
| S4 | Re-bake `visual_overlay_baked.npz` | The current bake shows the old tall bin and has no camera | S (part of P0) |
| S5 | `pyproject.toml` for `alix_dispatch` | Today it is reached only through `sys.path` | S |
| S6 (optional) | Move the robot-geometry payload builder into `alix_workspace` | Removes Lite's largest Python copy | S |
| S7 (optional) | Lazy `alix_workspace/__init__.py` | The package init imports `reachability` (X2) eagerly (checked: `WS/__init__.py:8`) | S |
| S8 (optional) | Move `TaughtPath`, `arclen_smooth` and a `retime_toppra` with explicit limit arguments (no module globals) from `EXP01/` into an importable ALIX tools module | Lite then imports the TOPP-RA chain instead of keeping its own retime port; removes the thread-safety caveat (`ETR:304-305`) | S |

---

## 6. Workflows

### 6.1 Operator golden path (Release 1), for one region

| # | Screen | What the operator does | What Lite does | What moves |
|---|---|---|---|---|
| 0 | — | Clicks the desktop icon (operator PC now; on the Jetson in production, the same page opened from a local or an authenticated remote browser) | Runs the self-test, then opens the Dashboard in **Simulation**, Operator mode, with no arm armed | Nothing |
| 1 | Dashboard | Presses **Teach next missing**, or clicks a cell | Opens the next untaught region of commode C in teaching order: by arm within each lid/seat phase (requirements, Appendix A) | Nothing |
| 2 | App shell | Once per session: connects and confirms "Real robot" for teaching | Tests the connection; reads the run mode, error codes and arm identity; reads the hardware **E-stop** input on each arm and shows one chip per arm (CLEAR, E-STOP ENGAGED, RECOVERING, UNKNOWN, NOT CONFIGURED). REAL is offered only when both chips read CLEAR. Shows the red REAL banner | Nothing |
| 3 | Teach | Answers the photo question, for example "Is the lid UP and the seat DOWN?" | Keeps START disabled until the answer is Yes | Nothing |
| 4 | Teach | Ticks "the cloth touches nothing" and "hands clear", then presses **Prepare robot** | Checks the arm is still, both E-stop chips are CLEAR and the stowed-bin travel band is clear; moves the **pole, then the hand**; shows target against measured values | Pole, hand |
| 5 | Teach | Holds the arm, presses START (foot switch), wipes during the countdown, presses STOP | Enters drag mode; counts down with sounds at 10 s left and at 0 s; saves to a temporary name, checks the file, renames it, writes the sidecar | The arm, by hand |
| 6 | Teach | Presses **Keep**, Redo, or Skip with a reason | Shows a mini plot and warnings. After Keep it waits 3 s and opens the next card; it never moves the robot by itself | Nothing |
| 7 | Optimise | Checks the **target time** (pre-filled from the region's cleaning time in the catalogue; the operator may change it), keeps the default optimiser or picks **Time-optimal (TOPP-RA)**, presses **Optimise**, optionally fine-tunes, presses **Save version** | Runs the chosen optimiser against the target and checks the written bytes against the hard limits L1–L6. Shows the target against the achieved time (lines × 5 ms), the recorded time and the shortest feasible time. If the target is shorter than the limits allow, it delivers the shortest legal time and says what binds, in the form "Closest feasible: X s — limited by joint 6 acceleration in 2.9–3.1 s", with a **Use X s** button and that window shaded on the plots (§7.3, P5) | Nothing |
| 8 | 3D Check | Presses **Check in simulation** and watches; on a problem, **Back to editing** | Sweeps every row with both arms of the step, the commode in its state, the venue walls from the selected venue file and the bin in its declared state. Stops at the first problem, marks it on the joint plots and lists what was checked, including the venue file name | Nothing |
| 9 | Test on robot | Presses **Send to robot** (it runs in the background); confirms **Replace** if asked | Reads the program list for the region's program ID (any ID 1–100, set by the engineer). If the slot holds a different program, shows that program's name and size and asks before replacing it; warnings (for example "this slot holds a program from the V6 workflow") never block. Skips unchanged content; verifies stored size and name | Nothing |
| 10 | Test on robot | Presses **Dry run on robot** | Sets SIM mode and reads it back; skips pole and hand steps and says so; checks completion; restores the mode | Nothing (arms still) |
| 11 | Test on robot | Reads the confirmation (arm, mode, IP, region, both E-stop chips CLEAR, size of the move to start), confirms the checklist, presses **Run on real robot** | Checks the transit from the live pose, including the walls. Runs a slowed copy first for new content (an engineer setting). Shows progress in words; the software STOP is always visible, next to the text "The hardware E-stop is the safety stop" | The arm |
| 12 | Test on robot | Presses Looks good / Hit something / Missed a spot / Other | Writes the run record. If the arm stopped part-way, it offers a checked recovery move. **Back to editing** opens at the stop row | Recovery only, after confirmation |

**If the E-stop is pressed, at any step.** The hardware E-stop buttons, wired to an IO input on each arm controller, are the primary safety stop (NFR-SAFETY (c)).

| # | What the operator does | What Lite does | What moves |
|---|---|---|---|
| E1 | Presses an E-stop button | The chip turns red and the header strip turns red. A running region is marked **INTERRUPTED — E-STOP**, never "Complete". The rest of any sequence is cancelled. Lite sends pole stop and hand freeze on both sides, because the IO E-stop has been seen not to stop the hand (`ALIX/docs/TEST_PROCEDURE.md:453`). All motion on both arms is refused; upload stays allowed. The stop is mapped to a row of the file and shown on the plots as "E-stop pressed", not as a collision | Nothing more |
| E2 | Makes the cell safe, ticks the confirmation, releases the button | The chip goes to RECOVERING, never straight to CLEAR | Nothing |
| E3 | Follows the recovery wizard: **Clear errors**, then **Re-enable joints** only if Lite asks (with the wrist supported) | Shows the controller state in plain words; clears errors in at most two rounds and reads back each time; restores the global speed; re-homes the pole if it stopped part-way | Pole, after confirmation |
| E4 | Reviews the move from the current pose to the program's first point, confirms | Collision-checks that move against the commode, bin, poles and venue walls and shows its size. Only then does the chip go to CLEAR. An interrupted region is never resumed by itself; the operator restarts it on purpose | The arm, after confirmation |

The wizard steps follow the E-stop research (RealMan Gen-3 manuals; NFR-SAFETY (c)). Whether a re-enable or an error clear is needed after release is measured in the P0 drill (§7.3).

**What operators never handle.** File paths, slots, program IDs, millidegrees, SDK or runner terms. The replacement question in step 9 names the program in the slot in plain words.

**Confirmations.** Only motion and controller overwrites ask for confirmation. Edits never do; they can be undone.

### 6.2 Engineer workflow

**Once per site:**
1. Install the bundle for the machine that runs Lite (x86-64 on the operator PC now; aarch64 on the Jetson AGX Orin in production) and check the self-test, including whether the TOPP-RA optimiser is available on that machine.
2. In Settings, set the IP and port per arm and the SDK folder. Read the E-stop input on each arm: Lite checks that IO3, the E-stop input on both controllers, is in E-stop mode (mode 5), and the engineer records its active level per arm. Lite never writes IO modes.
3. Run **Test connection** and the identity check.
4. Record commissioning on both controllers: the E-stop drill (items DR1, DR3, DR9, DR10 and DR11 must pass before operators run real tests), G4 (does the software STOP end a running stored program), the program-finish event, speed restore, mode read-back, transit linearity, dual start skew, and whether stored sizes match exactly.
5. **Confirm the cubicle:** 1400 mm wide by default, or switch to 800 mm in Settings (D21). Verify it by replaying at least 3 known-good raw recordings through the 3D check: no wall or structural-link contact. The robot, UGV, poles and bin come from the URDF and the commode from the `commode_c` scene, so nothing is measured. A wrong cubicle shows up here: the 800 mm walls give false wall hits on 28 of 188 sampled rows of `R2_LIDTOP` (C3 §2.5).
6. Sign off the catalogue with the requester: IDs, the cleaning time per region (the optimiser's default target), pole and hand presets, the fixture part per region, the executed path budget and MAX DEV per region, and the contact list per hand preset.
7. Assign a program ID (any 1–100) per region and arm, or accept Lite's proposal.

**Per region:**
- Set budgets and presets.
- Choose the optimiser per region when the default is not wanted, and review cases where the target was not met.
- Review findings.
- Approve supervisor overrides, with reasons.
- Raise per-pair waivers only for proven model gaps.
- Investigate "Not confirmed" and "INTERRUPTED — E-STOP" runs.
- Compare robot memory (the slot list) with the ledger.
- During Release 1: run the slot check before any dual run from v53.

**Release 2:**
- Build programs and dual blocks in the Program screen and produce slowed first-run kits.
- Run TOGETHER blocks only after G4 has passed. Both arms get the same line count, and the dual-arm collision timeline is re-run on the retimed pair before upload.
- Apply offsets in this order: touch-off or typed value → solve → report → 3D check → dry run → slowed real run → full speed.
- Switch the executor to the agents once the switch criteria (§4.4) are signed.

**Maintenance:**
- Update and roll back from a file.
- Re-commission after any controller firmware change, and repeat the E-stop drill at the configured interval (Lite shows "E-stop drill overdue").
- Re-select and re-verify the venue file when the robot moves to a different cubicle.
- Pin the next ALIX commit for each release and re-run the parity suites, including the TOPP-RA parity test, on both architectures.
- Export support bundles.

---

## 7. Phased delivery plan

### 7.1 Delivery model, releases and scope

**Delivery order (decision D27, 13 Oct 2026).** The build starts from the GUI: UI/UX first, then the backend. The phases keep their numbers, scope and agent-day estimates (§7.2); only the order in which they are built changed.

1. **UI/UX first:** the operator workflow screens on the ALIX Studio theme, working against the existing optimiser and stand-ins for the robot (the fake SDK, the fake recorder and the emulator). This is the theme, shell and UX framework of P1 and the screens of P2–P7.
2. **Optimiser golden tests and integration** (P5): the golden tests of §8.1.1 on both architectures, and the optimiser wired to the Optimise screen.
3. **3D check with walls** (P6).
4. **Robot I/O with the IO3 E-stop monitor** (P3, and the robot layer and uploads of P7).
5. **The server and API** (P1): the local server, its request protection and the project store, and the API the screens call in place of the stand-ins.

The lab sessions run with the backend steps they need; teaching on the robot (P4) and the rest of Release 1 follow (§7.6). Release 2 (P8–P11) keeps its order. §7.7 lists the milestones in this order.

**Backend status.** The optimiser engine (both optimisers, the target-time search, limit checks) is implemented and smoke-tested but on hold. The checkpoint is in [`docs/STATUS.md`](STATUS.md).

**Delivery model (decision F8, 13 Oct 2026).**
- **Who builds.** Claude (Opus 5.5) implements Lite in `alix_studio_lite` (`LITE/alix_studio_lite`), with parallel agents working on separate tracks: foundation, data, optimise, 3D check and robot layer. Lite is not a package inside the ALIX monorepo; it imports ALIX libraries from the vendored ALIX copy in `alix_studio_lite/vendor/` (F7). The documents go to `alix_studio_lite/docs`.
- **What the engineer does.** An engineer reviews and merges each track, takes the decisions listed in §11, signs off the catalogue with the requester, and runs every robot session. No code reaches the robot without that review.
- **What coding speed cannot compress.** Robot and lab time: the P0 characterisation and E-stop drill, teaching on the robot, dry and real runs, the operator usability session and the field trial. Each needs the robot, a supervising engineer and, for usability, operators. The schedule (§7.6) places these sessions at fixed steps; a session that slips moves every gate behind it by the same amount.
- **Units.** An **agent day** is one working day of the agent-driven team: Claude with parallel agents, plus the engineer who reviews. Phases overlap inside a day, but the engineer's review limits the team to about one agent day of build per calendar day, so the phase figures in §7.2 add up to the calendar in §7.6. **Lab days** are robot sessions run by the engineer; they run alongside the agent build on the same calendar days.
- **Targets.** Lite runs on the operator PC (x86-64 Linux) now and on the robot's Jetson AGX Orin (aarch64, Ubuntu 24.04) in production (F1). Both bundles are built and tested from P1 onward.

| Release | Phases | Scope | Meanwhile |
|---|---|---|---|
| **Release 1: single-arm operator loop** | P0–P7 | • R1: the commode C catalogue (F3): 34 units in the first release, 26 cleaning regions and 8 lid/seat operations (not cleaning tasks); glove and bin tasks shown as Not in this release.<br>• R2 and INT-TRAJ with two optimisers (the prototype's default, with three fixes, and time-optimal TOPP-RA) and the operator's target time; R6, R11, R12.<br>• R3: region-level playback with both arms of the step.<br>• R4: single-arm runs, dry run and real, IP entry, three modes, the IO E-stop state per arm with guided recovery.<br>• R5: in simulation, including the venue walls; on the real arm, approximate rows from the post-run joint read.<br>• R7.<br>• R10: correct model, bin control, split view.<br>• R13: uploads to any program ID 1–100 with a replacement confirmation, ledger, skip unchanged, parallel arms, fewer points.<br>• INT-CTRL for single arms; X1/X2; the NFRs.<br>• Bundles for x86-64 and aarch64. | Dual flows keep running from v53 Page 02. They may use programs that Lite uploaded and verified; Lite exports the slot map and offers a slot check. Lite can write to any slot, including those that hold V6 programs (71–83), but only after the replacement confirmation, which warns about V6 and ALIX ranges. |
| **Release 2: agents, programs, dual, offsets** | P8–P11 | • Stage B agents: verified stop ladder, heartbeat, live mirror, exact real-arm collision rows, one upload session per arm, E-stop polling on the same connection.<br>• R8; R10 dual timeline and speed analysis.<br>• Dual coordinator: TOGETHER (equal line count on both arms) / RUN FIRST / WAIT OTHER / checkpoint, cross-arm abort, two-phase start.<br>• R9, R14.<br>• Field trial on the Jetson and hardening. | v53 is retired from dual flows once P9 passes its hardware acceptance tests. |

**Backlog: outside both releases and outside the schedule.** Relative size: S, M, L, XL as in the requirement sections.

| Item | Requirement | Size |
|---|---|---|
| Hand-event steps for tool and glove tasks | R7 | S |
| 3D preview of pole and hand | R7 | S |
| Toilet registration for a mesh-based press profile | R14 | M |
| Coverage % per region | R3, phase 2 | — |
| Pop-out plot window | R6, open question | — |
| Windows robot I/O | — | S, after Stage B is verified on Windows |
| CANFD streaming | R13 | Separate XL engineering lane |

### 7.2 Phase summary

Agent days and lab days are defined in §7.1. Lab time is engineer and robot time, and runs alongside the build at the steps shown in §7.6. The table lists the phases by number; since 13 Oct 2026 they are built in the order of §7.1 (D27).

| Phase | Goal | Requirements | Agent build (working days) | Robot/lab time | Exit gate |
|---|---|---|---|---|---|
| **P0** Ground truth, decisions, seams | Measure what is unproven, select and verify the venue wall file, prove the bundle fails closed, settle the remaining decisions | R3, R5, R10 (de-risk), INT-CTRL P0, NFR-SAFETY, NFR-OFFLINE-DEPLOY | 0.5 | 1 lab day: software STOP (G4) and IO E-stop drill, SIM-mode pole/hand, timing, venue/wall check with known-good recordings | Characterisation and drill report signed; venue verified; bundle self-test fails closed without a checkout |
| **P1** Foundation | Installable, themed, empty Lite with the project store and scope guards, on both architectures | NFR-THEME, NFR-UX, NFR-OFFLINE-DEPLOY, NFR-DATA, X1, X2, NFR-SAFETY (network) | 0.5 | — | Clean offline install on x86-64 and aarch64; token and store tests pass |
| **P2** Regions, dashboard, import | Operators see what to clean and what is done; v53 recordings can be imported | R1, R7 (data), NFR-DATA | 0.25 | — | 42 regions load with target times; import and status tests pass |
| **P3** Robot connection and safety layer | Connect by IP, three modes, E-stop state per arm, software STOP, gateway, Prepare (pole/hand), on the Lite runner copy | R4, R7, INT-CTRL, NFR-SAFETY | 0.5 | 0.5 lab day: connect, modes, E-stop chips, Prepare pole/hand on both arms | Fake-SDK suite passes; hardware checklist for Prepare and the E-stop chips passes on both arms |
| **P4** Guided teaching | Region-by-region teaching on the robot with countdown and auto-advance | R1, R7 | 0.5 | 1 lab day: teach 3 regions | 3 regions taught in a row on the robot; crash recovery shown once on hardware |
| **P5** Optimise | Both optimisers with the target time, limits L1–L6, spikes, undo, adjustable plots | R2, R6, R11, R12, INT-TRAJ | 1.0 | — | Golden tests pass on both architectures; one-click optimise gives a sendable version and names what binds |
| **P6** 3D check | Dual-arm 3D with commode, venue walls and bin toggle; stop at collision with joint markers on the plots | R3, R5, R10, R11 | 1.0 | — | Fail-closed verdicts; seeded collision found at the right row; wall check passes on the known-good recordings |
| **P7** Test on robot and upload | Ladder-gated dry run and real run, verdicts, E-stop interruption and recovery, uploads to any slot with overwrite confirmation; operator usability session | R4, R5, R13, INT-CTRL | 0.75 | 1 lab day: dry run and real runs, uploads to any slot with overwrite confirmation, an operator usability session | **Release 1** |
| **P8** Robot agents | Stage B executor on `alix_dispatch`; live mirror; exact collision rows; one upload session | R4, R5, R13, INT-CTRL (Phase 2) | 0.5 | 0.5 lab day | Switch criteria (§4.4) signed |
| **P9** Program builder and dual | Collision-checked merged programs; dual workflows and timeline | R8, R10, R5, INT-CTRL | 1.0 | 0.5 lab day | V6 golden and planner parity; TOGETHER block on hardware after G4 |
| **P10** Offsets | FK → offset → IK without jumps; touch-off; simulated first | R9, R14 | 1.0 | 0.5 lab day | Continuity and touch-off tests; offset ladder run on hardware |
| **P11** Hardening and Release 2 | Clean installs on both architectures, field trial on the Jetson, guides, usability re-test | NFR-OFFLINE-DEPLOY, NFR-UX, NFR-DATA | 0.5 | Operator field trial | **Release 2** |
| **Total** | | | **8** | **5 lab days + field trial** | |

- **Release 1 (P0–P7):** about **5 agent days** (range 4–6) plus about **3.5 lab days**.
- **Release 2 (P8–P11):** about **3 agent days** (range 2–4) plus about **1.5 lab days** and the operator field trial.
- **Core-loop milestone, about 2 agent days of build, after the UI/UX work (D27):** teach through the Lite runner copy (runner v14 with the E-stop fix), optimise with both optimisers and a target time, 3D check with walls, upload to any slot with the overwrite confirmation, and run. It is tested on the fake controller and the emulator, then by a supervised dry run on the robot (controller SIM mode; the arms do not move).
- The ranges cover rework after review and after the lab sessions. They do not cover waiting for the robot, the Jetson or a decision.

### 7.3 Phase details

#### P0 — Ground truth, decisions and upstream seams

**Build:** 0.5 agent day. **Robot/lab:** 1 day.

**Goal.** Before any design locks:
- answer the open questions about the controllers and the IO E-stop;
- select the venue wall file for the test cell and verify it with known-good recordings;
- prove the asset bundle fails closed;
- take the decisions that still block the design.

**Deliverables:**
- **Decision log and hazard workshop.**
  - Settled by the user on 13 Oct 2026 and recorded: D1 target computer (operator PC now, Jetson in production, F1), D2 program IDs (any ID 1–100 with a replacement confirmation, F6), D4 the cleaning time as the optimiser's target (F2), D7 the cell model from the URDF and the `commode_c` scene with no calibration project (F5), D15 code location (`alix_studio_lite`, F7).
  - To decide: D3 region IDs and the cleaning time per region (the optimiser's default target, F2), D13 executor policy, D14 seams.
  - A hazard list with a control and a test for each hazard (§9.1).
- **Supervised hardware characterisation and E-stop drill on both controllers.** One lab day; the agents write the scripts and the record forms beforehand.
  - **IO E-stop drill** (NFR-SAFETY (c); research items DR1–DR14): confirm IO3 is in mode 5 on each controller (D25) and record its active level; whether one button trips both controllers; stop during a running stored program (`run_state` after, error codes, what runner v14 reports); pole and hand behaviour; drag teach; refusals while held; no next sequence step after a quick release; the recovery steps that are actually needed. Items DR1, DR3, DR9, DR10 and DR11 must pass before operators run real tests with Lite.
  - **G4, software STOP:** `probe_stop_policy` (`DP/track.py:3273`) on the zero-exposure in-place dry track (`CT:22, 3729`). Does a slow stop end a running stored program? How far does the arm travel, and how long does it take?
  - **After a killed runner:** does the program keep playing, and is the global speed restored?
  - **Fresh connection:** does the stop ladder (`DP/track.py:3109`) work after the client died? Does a post-exit joint read work?
  - **Controller SIM mode:** what one pole command and one hand command do (no motion, a time-out, or real motion).
  - **Upload timing:** one file of about 600 KB, serial and with both arms in parallel; per-process overhead.
  - **Dual start skew** with a barrier launch.
  - **Per arm:** J4 drive limits (for the identity check) and collision-detection level (left 3, right 0 on 2026-08-13).
  - **Drag record period:** the time between `DRAG TEACH ACTIVE` and `STOPPED` against the point count.
  - **Bin:** fitted or not and its state. The geometry comes from the URDF; the stowed foul band (0.153–0.282 m) is checked against the physical bin because NFR-SAFETY (f) calls it provisional.
- **Zero-code harness and venue/wall check.**
  - Convert the V6 L/R tracks with the `DAV` shim into `.cache/lite_spike_plans` (with `ALIX_PLANS_DIR`) and inspect them in the unchanged Studio Dual-Arm Collisions tab.
  - **Cubicle walls.** The cell itself is already described by the URDF (robot, UGV, poles, bin) and the `commode_c` scene (commode pose and articulation; `ALIX/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml`), so no cell calibration is needed (F5). The only missing check is the wall. Lite builds the 1400 mm single-cubicle mesh (the default, D21; ALIX's 1400 mm venue is the whole washroom) the same way as ALIX's 800 and 1000 mm cubicles, and offers ALIX's 800 mm cubicle as the alternative, both loaded through `load_wall_manager` and `PlanWallChecker` (`WS/plan_scene_validation.py:117, 179`).
  - A script replays at least 3 known-good raw recordings through `BodyCollisionWorld` with the selected walls and reports wall and structural-link contacts.
  - Output: a **venue record** (file name, sha256, the recordings used, the result), stored in the site settings.
  - Run `TT screen` on the V6 and lid_top files for a legacy triage report. It writes nothing.
- **Upstream seams S1, S2, S3 and S5, with tests.** S6/S7 if time allows. A declined seam is recorded, and its item switches to a port.
- **Re-baked overlay and model tests:** BinShort top at 0.3982 m, `camera_link` present, served URDF sha equal to `WS/urdf/alix.urdf`.
- **Vendored ALIX copy and world self-test.** Done 13 Oct: `tools/vendor_from_alix.py` copies the kept subset in repo layout with `vendor/MANIFEST.json`, and 52 tests pass from the copy. The copy includes the `commode_c` scene, ALIX's 800 mm cubicle venue with its mesh, and Lite's 1400 mm cubicle files. Negative cases included. Run on a machine **without** the ALIX checkout.
- **Pinned environment and wheelhouse for x86-64 and aarch64.** toppra 0.6.10 on PyPI ships x86_64 wheels only and no source package, so the aarch64 wheel is compiled from the git tag (Cython, gcc, numpy headers); one TOPP-RA solve is timed on the Orin (research on optimisers, §1.6).

**Requirements served:** R3, R5, R10 (de-risk), INT-CTRL Phase 0, NFR-SAFETY, NFR-OFFLINE-DEPLOY.

**Exit criteria:**
- The characterisation and drill report is signed. G4, the speed restore, SIM pole/hand behaviour, the fresh-connection stop, the start skew and the E-stop drill results (IO number and level per arm, one chain or two, run state after a press, what needs clearing on recovery) are recorded as design inputs.
- **Cubicle verified:** with the 1400 mm cubicle (or the 800 mm one where that is the cell), no wall or structural-link contact on the replayed known-good recordings. The 800 mm walls reproduce the 28 false wall rows (C3 §2.5), which proves the wall check is active.
- The V6 pair has been inspected in the unchanged Studio.
- The bundle self-test passes without a checkout. Removing one mesh, the SRDF, the commode scene or the selected venue file makes it fail and name the file.
- The seams are merged with tests, or declined on record. After S1, the Studio's collision payload for a cached pair is byte-identical.
- D3, D13 and D14 are decided.
- The aarch64 toppra wheel builds and a solve time on the Orin is recorded. If it does not build, the aarch64 bundle ships with TOPP-RA disabled and the default optimiser, and this is recorded.

#### P1 — Foundation

**Build:** 0.5 agent day. **Robot/lab:** none.

**Goal.** An installable, themed, empty Lite with the project store, network protection and the scope guards in place, built for x86-64 and aarch64.

**Deliverables:**
- **Server:** package skeleton in `alix_studio_lite`, with `vendor_paths.py`, which runs the vendored ALIX copy from `vendor/alix`; copied server scaffold with routes and needs table, job registry and page assembly.
- **Request protection:** loopback bind; per-launch token in a custom header; Host and Origin checks; configured authentication for the remote-browser mode on a headless Jetson (NFR-OFFLINE-DEPLOY (a)).
- **Theme:**
  - tokens with a parity test;
  - base and shell CSS with the known fixes;
  - local fonts and icons;
  - `plotTheme()` and the joint palette;
  - accessible variant tokens and arm-identity tokens;
  - a dark toggle;
  - a literal scanner covering CSS and JS.
- **UX framework:**
  - shell: app bar, mode chip, E-stop chip slot per arm, STOP-bar slot, step rail;
  - stepper component; Operator/Engineer switch;
  - message catalogue with the controller-code table and a blocked-terms lint;
  - glossary; `confirmMotion()` with Cancel focused; presets infrastructure;
  - the layout audit in CI;
  - wireframes reviewed with the requester and one operator.
- **Packaging:**
  - launcher: single instance, isolated Python, app window with fallback;
  - `install.sh` and `uninstall.sh`; desktop entry;
  - update from file with rollback;
  - one release artifact per architecture (x86-64, aarch64), with the RealMan SDK build that matches the CPU;
  - self-check page; Settings skeleton.
- **Project store:**
  - schemas; atomic writes; read-only raw files; sha256; version counters;
  - conflict guard; backups; audit log; write boundary.
- **Scope guards:** import-boundary test (X1/X2 deny-list, light-module whitelist); route allow-list; release-content test.

**Requirements served:** NFR-THEME, NFR-UX, NFR-OFFLINE-DEPLOY, NFR-DATA, X1, X2, NFR-SAFETY (network).

**Exit criteria:**
- **Clean install.** On a clean, network-disabled x86-64 VM and on an aarch64 machine (the Orin, or an aarch64 VM until it is available): install without root; the UI is interactive within 3 s (target; 10 s hard limit); no external request is made.
- **Update.** Update and rollback are demonstrated.
- **Theme.** Token parity passes. There is no colour literal outside the tokens and `plotTheme()`. Fonts render offline.
- **Scope.** After every route has been called, no deny-listed module is loaded.
- **Store.** Kill -9 during a save (100 runs) always leaves either the old or the new complete file. A write outside the project folder fails.
- **Network.** A POST without the token, or with a foreign Origin, is refused.

#### P2 — Regions, dashboard and recording import

**Build:** 0.25 agent day. **Robot/lab:** none.

**Goal.** Operators see what must be cleaned and what is done, and recordings made in v53 can enter Lite with provenance.

**Deliverables:**
- **Catalogue:** catalogue, offline `deck_import` and the sub-step model (R1). Each region carries its `commode_c` fixture part, its cleaning time (the default target time for either optimiser, F2) and its MAX DEV where one is set.
- **Deck assets:** renders, crops and overlay boxes.
- **Regions tab.**
- **Dashboard and status engine.**
- **Pole and hand fields:** fields and supervisor editing, with defaults seeded from V6 and marked "proposed" (R7).
- **Import recording wizard.**
- **V6 legacy import:** split at the known sample indices and flagged LEGACY.

**Requirements served:** R1, R7 (data), NFR-DATA.

**Exit criteria:**
- **Catalogue.** It loads all 42 deck units (22 LEFT, 20 RIGHT; 34 in the first release) with unique keys in the deck's code format and a target time for each first-release unit. The validator rejects duplicates, missing fields and unknown states.
- **Status.** An empty project shows "0/34 taught". An imported recording appears as Taught after Keep. Practice files are never counted.
- **Legacy.** The V6 import marks covered, merged, partial and other-arm regions as in the requirements' Appendix A. LEGACY never counts as raw and can never be uploaded.
- **Pilot.** One non-technical tester finds and opens the next untaught region without help.

#### P3 — Robot connection and safety layer

**Build:** 0.5 agent day. **Robot/lab:** 0.5 day (connect, modes, E-stop chips, Prepare pole/hand).

**Goal.** Lite can connect to both arms by IP, read and set the three modes, show the IO E-stop state of each arm, stop through the software STOP and prepare pole and hand, all through the Lite runner copy behind the motion gateway.

**Deliverables:**
- **Lite runner copy:** runner v14, `hand.py` and `pole.py`, pinned by sha256, with opt-in flags (default off) and one non-optional change, the E-stop fix: read the E-stop IO, refuse to start while it is held, report "interrupted" instead of "RUN COMPLETE", never start the next sequence step after an E-stop, print parseable `ESTOP ENGAGED` / `ESTOP CLEAR` lines (R4; NFR-SAFETY (c)). The default paths are proven unchanged by diff, apart from that fix.
- **Robot channel:** runner channel with per-arm lock, logs and lifecycle. Shutdown stops runners first.
- **Event adapter:** the token table, plus a fake-SDK harness that runs the Lite runner copy.
- **Settings and identity:** IP and port per arm; SDK folder per architecture; E-stop IO number and active level per arm, read from the controller's IO modes (Lite never writes them); pin check; Test connection; identity check (J4 drive limits, stored binding, refusal of identical IPs, warning when a controller holds the other side's programs).
- **E-stop state:** one chip per arm (CLEAR, E-STOP ENGAGED, RECOVERING, UNKNOWN, NOT CONFIGURED), polled at 5–10 Hz by the process that owns the arm's connection; UNKNOWN and NOT CONFIGURED block motion like ENGAGED; while either chip is not CLEAR, every motion route on both arms is refused server-side and upload stays allowed.
- **Run mode:** read-back; switch with confirmation (REAL only with both chips CLEAR); restore after a dry run; pole/hand steps stripped in dry runs.
- **Software STOP:**
  - labelled "STOP (software)" next to "The hardware E-stop is the safety stop";
  - the launch wrapper: STOP or end-of-input is sent **once**;
  - STOP ALL: arm slow stop, pole stop, hand freeze, sequence cancel;
  - a browser heartbeat from a Web Worker; a stop when the backend dies;
  - an about-8 s confirmation window, then the fallback ladder.
- **Session and arming:** session state machine and arming with idle disarm and the REAL banner.
- **Motion gateway:** one-time tokens, reservation, error-code check, E-stop check, interlocks, process interlock, logging.
- **Tests and protocol:** emulator and fake-SDK tests; draft bench protocol.
- **Operator robot widgets.**
- **Engineer robot console, first part.**
- **R7 Prepare step:**
  - interlocks: arm still below 1.6 °/s, not in drag mode, not holding the fixture, both E-stop chips CLEAR, stowed-bin band checked over the whole travel, checklist;
  - order: pole then hand; read-back within ±5 mm and ±15 counts or settled;
  - read-only status, with fake-device, emulator and hardware checklists on both arms.

**Requirements served:** R4, R7, INT-CTRL, NFR-SAFETY.

**Exit criteria:**
- **Pins.** A runner whose sha256 differs from the pin, including the unpatched v14 and v13, is refused with a clear message.
- **Fake-SDK suite.** The Lite runner copy runs upload, run, workflow, drag start/stop/save, hand, pole, mode and list. Every command line contains `--sdk --side --ip --port --yes`.
- **Connection.** Test connection reports Connected, Unreachable or Error within 10 s. A swapped IP binding is refused by the identity check.
- **E-stop chip (fake SDK).** The chip shows ENGAGED within 0.5 s of the mode-5 IO going active; a stale reading over 1 s shows UNKNOWN; no mode-5 IO shows NOT CONFIGURED; `arm_current_status` = 9 alone never changes the chip. While either chip is not CLEAR, every motion route on both arms is refused with the reason, and upload still works (R4 acceptance).
- **Stop (on the fake controller).** One click sends one interrupt within 200 ms, never a second. Killing the backend makes the runner slow-stop and restore the speed.
- **Modes.** Lite opens in Simulation. REAL needs both E-stop chips CLEAR, a confirmation and a read-back.
- **Prepare.** It moves the pole before the hand and refuses on a moving arm, on an E-stop that is not CLEAR, and on a travel through the stowed band. Hardware checklist on both arms: 50 → 150 → 200 mm with the bin deployed; GRIP ↔ thumb-held open; Stop during a pole move; the chip follows a real E-stop press and release.
- **Process interlock.** Robot commands are refused while v53, or a runner Lite did not start, is running.

#### P4 — Guided teaching

**Build:** 0.5 agent day. **Robot/lab:** 1 day (teach 3 regions).

**Goal.** An operator teaches region after region on the robot, without a terminal and without losing a recording.

**Deliverables:**
- **Teach backend:** state machine, ordering by arm within each lid/seat phase, and resume.
- **Teach UI:** card, countdown, sounds, overtime, review, Keep/Redo/Skip, foot-switch key.
- **Teach adapter and crash recovery:** recovery from orphaned drag sessions included; an E-stop during drag teach is handled as the P0 drill (item DR8) showed.
- **Save path:** temporary name, 3 retries, point-count check, rename without overwrite, read-only file, sidecar with the measured record period.
- **Fake recorder:** practice mode and tests.
- **Hardware acceptance.**
- **Usability pilot.**

**Requirements served:** R1, R7.

**Exit criteria:**
- **Three regions on the robot.** An operator teaches 3 regions in a row on the robot, using only Prepare, START, STOP and Keep. Each file is named by region and read-only; each sidecar is complete.
- **Dashboard.** It updates live.
- **Failed saves.** A failed save is retried 3 times and never leaves a file under a final name. A second recording never overwrites the first.
- **Crash recovery.** Kill Lite during a recording, first on the fake recorder and then once on hardware. On restart, Lite takes the arm out of drag mode and offers Save or Discard for the recording the controller still holds.
- **Both-arm mode.** One START puts both arms in drag mode; STOP saves two files.
- **No command during drag.** While any arm is in drag mode, Lite sends no other robot command.
- **E-stop.** START is refused while either chip is not CLEAR.

#### P5 — Optimise

**Build:** 1.0 agent day. **Robot/lab:** none (the first robot use of the output is in P7).

**Goal.** A raw recording becomes a sendable version in one click, at the operator's target time or at the closest time the limits allow, with either of two optimisers. Engineers can fine-tune by segment with undo. Sources for this phase: the research on optimisers (13 Oct 2026) and the C2 analysis.

**Deliverables:**
- **Optimisers, in `alix_studio_lite/lite/optimise/`, behind one interface.** Each returns a result with the bytes, the target, the achieved time, T_min, whether the target was met, the binding constraints, every check result, and the recipe with library versions and the origin sha256.
  - **Default optimiser** = the trajectory prototype's algorithm: trim idle → moving mean → per-segment linear scale (`HTML:1438-1616, 1676-1724`). It has no limit checks today and three measured defects: an endpoint snap of 546–1005 millidegrees in one tick (`HTML:1513-1514, 1721-1723`), boundary spikes of 640–919 °/s² (`HTML:1568-1616`) and 10.6–20.9 mm of pad path change with the 101-point window. Three fixes keep it the default: odd-reflection padding at the window edges; the speed change at each segment boundary blended over 0.25 s (spikes fall to 200–280 °/s²); default window 41 plus a hard path-change check (pad moves 2.9–4.5 mm).
  - **Time-optimal optimiser (TOPP-RA)**, reusing ALIX's hardware-proven chain for taught paths (`ALIX/docs/motion/TAUGHT_CANFD_PIPELINE.md:17-40`): 5 Hz smoothing (`TT:891-912`), pause removal R4 (`TT:915-951`), 30 mm arc-length smoothing (`ALIX/experiments/exp01_teach_trajectory_reverse_engineering/src/local_speed.py:139-169`, below `LS`; 15 mm when the region has a MAX DEV), then TOPP-RA at 0.95 × the joint ratings, 600 °/s², Interpolation scheme, seidel solver, zero speed at both ends (`LS:282-324`). ALIX is called through one adapter module; the retime step is a Lite-owned function with explicit arguments, because ALIX's `LS.RATE_DEG_S` and `LS.ACC_DEG_S2` are module globals changed by the emitter (`emit_toppra_rung.py:304-305`, in the same `exp01…/src/` folder). Sampling uses the `resample_clock` form (`ALIX/tools/plan_min_time.py:375-394`), which lands exactly on the path end and is never faster than optimal; the 6 Hz post low-pass stays off by default. `toppra` is imported guarded: without it, the option is greyed out and the default optimiser still works.
  - **Target-time solver,** the same for both: the target is in seconds at 5 ms per line, so its line count is the target ÷ 5 ms (D26). T_min is the shortest time whose written bytes pass the hard limits: one solve plus a byte check for TOPP-RA, bisection on log k (about 25 iterations) for the default. At or above T_min, uniform time scaling hits the target to the tick; below it, Lite delivers T_min and names the binding constraint (joint, quantity, time window).
  - **Hard limits L1–L6 on the integer bytes at 5.0 ms per line** (`TT:94-102`): L1 the ALIX model's G1 margin ≥ 0.5° (`TT:466-480`); L2 20 Hz band-limited speed ≤ 0.97 × the ratings (`TT:229`); L3 acceleration ≤ 600 °/s² × 1.02 as a 50 ms windowed mean (advisory in ALIX, hard in Lite); L4 step ≤ 10° per line (`TT:127-128`); L5 pad path change from the raw recording (G4) ≤ 10 mm, or the region's MAX DEV if smaller; L6 the file ends at the recorded start and end poses. Checks only: G5 predicted departure (a warning only, D11), jerk, pad speed, single raw ticks over a rating.
  - **Segments:** default = time % (the prototype's meaning), with an optional pinned time; TOPP-RA = speed cap %, passed to the solver as a varying velocity constraint so that braking stays legal.
  - **TOGETHER alignment** (used by P9): both arms get N = max(target, left T_min, right T_min), and the binding arm is named.
  - The solver runs in a worker process, so the UI stays responsive and Cancel stays available.
- **INT-TRAJ:**
  - UI port and restyle;
  - routes and store;
  - units, the 5 ms clock and report;
  - dual-arm lane;
  - optimiser panel: optimiser choice; target time input (pre-filled from the catalogue); achieved time (lines × 5 ms); recorded time and shortest feasible time; an amber "Closest feasible: X s — limited by joint N speed/acceleration in a–b s" chip with a **Use X s** button and the window shaded on the plots; an L1–L6 pass/fail panel;
  - tests, including the 8 prototype UI-state bugs.
- **R2:**
  - recipe processor around the two optimisers: ALIX methods imported after S3; ramps of 0.01 per line; local rating fit; dwells; Classic methods behind flags;
  - history and versioning: at least 5 labelled steps, persisted;
  - segment editor.
- **R12:**
  - spike detectors SD1–SD6 on the output bytes at 5.0 ms;
  - rule table and one-click undoable fixes;
  - findings UI;
  - regression corpus.
- **R6:** splitters, resize bus, per-plot maximise with exact restore, linked zoom by time within a file and by raw-line position for before/after, layout persistence, Playwright tests.
- **R11, tier 0:** presets Off / Light 5 Hz / Medium 3 Hz / Strong 2 Hz; exact pad deviation in both directions against G4; update within 1 s.

**Requirements served:** R2, R6, R11, R12, INT-TRAJ.

**Exit criteria (from C2 and the research on optimisers, §6.3–6.4):**
- **File contract.** An identity recipe reproduces the raw file exactly. The output passes R14 `read_track` and `DP/track.parse_track`, as `{"point":[7 ints]}` lines (`TT:737-745`).
- **TOPP-RA parity.** FILE_01.1 gives 3839 lines (19.19 s at 5 ms), equal to ALIX slot 101 (`TAUGHT_CANFD_PIPELINE.md:588`), with bytes within ±1 millidegree of the ALIX chain (`emit_toppra_rung.py:417-427`). FILE_08 / FILE_10.1 / FILE_6 give 16.80 / 4.26 / 20.25 s at 5 ms, ±1 line.
- **Uniform scaling.** On FILE_10.1 at k = 1.3 and 2.0, the duration is k × T_min ± 1 tick and the path matches the k = 1 path.
- **Infeasible target.** FILE_10.1 with a 3.0 s target is not met; Lite delivers T_min and names joint 6 acceleration, about 38 % of the time, window 2.88–3.05 s.
- **Target met.** FILE_01.1 with the default optimiser and a 60 s target is achieved within ±1 line of 12,001 lines (60 s ÷ 5 ms + 1), and L1–L6 all pass.
- **Default optimiser fixes.** First and last steps ≤ 100 millidegrees; windowed acceleration ≤ 300 °/s² at every segment boundary; at window 41 the pad moves at most 4.6 mm on FILE_08, FILE_10.1 and FILE_01.1, and window 101 fails L5 with a "reduce smoothing" hint. Its T_min on those files is 27.42 / 8.14 / 38.30 s at 5 ms, ±1 %.
- **Monotone limits.** Every hard check passes for every k ≥ k_min (property test on 20 files).
- **Without toppra.** With the `toppra` import blocked, Lite runs, the TOPP-RA option is disabled with a message, and the default optimiser works.
- **Both architectures.** The golden tests pass on x86-64 and aarch64 (bytes within ±1 millidegree across platforms).
- **Smoothing.** Light moves the pad at most 1.0 mm on FILE_10.1 and at most 1.6 mm on FILE_08. End points are exact. Boundary accelerations stay within the AC5 bound.
- **Sendable gate.** A version is "sendable" only when L1–L6 pass on its bytes, with L5 at the region's MAX DEV where one is set; G5 is shown as a warning (D11).
- **Spike findings.**
  - `R_P06P07` gives a Must-fix: J6 at 312 °/s band-limited, 102 ticks.
  - `R10_HNGR` gives a Must-fix with the fix "slow the approach to ≤ 7.4 °/s".
  - Raw-only files give warnings, not Must-fix findings.
- **Undo.** At least 5 labelled undo steps per region, and they survive a restart.
- **Parity.** "Classic" reproduces `UC/zip/02…30pct` from `zip/01` within 1 mdeg. The JS preview equals the Python output within 1 mdeg on 20 cases.
- **Duration.** A 9,813-line output shows 49.1 s ((9,813 − 1) × 5 ms = 49.06 s).
- **Offline.** No CDN request.
- **One click.** Optimise on FILE_08 with each optimiser gives a sendable version and names the binding constraint.
- **Tester.** A tester produces a sendable file at a given target time in under 3 minutes.

#### P6 — 3D check

**Build:** 1.0 agent day. **Robot/lab:** none.

**Goal.** A trustworthy 3D check of both arms of a deck step: the commode from the `commode_c` scene in its state, the venue walls from the selected venue file, the bin from the URDF in its declared state (toggle), the contact policy applied. It stops at a collision and marks it on the joint plots.

**Deliverables:**
- **R3:**
  - viewport and scene port with a thin loader and manifest;
  - adapter and flow model, imported after S2;
  - FK frames from the server, windowed, with a payload measurement; a JS FK only if needed;
  - player and plot sync;
  - pad trail and wiped-area painting;
  - editor link and staleness;
  - verdict chip and tests.
- **R5:**
  - world assembly with full hands, curated self pairs, commode, venue walls (through `load_wall_manager`), bin and contact policy;
  - warm sweep worker: every row, quick mode, cache keyed by the files, pole, hand, bin, scene, venue file and mode;
  - contact classifier;
  - transit, pole and hand move checks, including the walls;
  - plot markers, stop and event list, with "E-stop pressed" events kept separate from collisions;
  - tests.
- **R10:** split layout; bin control with fitted/state and the stowed band computed from the model; tests.
- **R11, tiers 1–2:** the tier orchestrator; G5 refresh; windowed re-check of the changed part; overlays; verdict panel.

**Requirements served:** R3, R5, R10, R11.

**Exit criteria (from C3):**
- **Fail closed.** Every verdict lists what was checked, including the venue file name. A missing scene, mesh or venue file gives an error, never "clear". A venue file not yet verified in P0 carries "venue not verified".
- **Walls.** With the selected venue file, the known-good recordings show no wall hit. With the 800 mm default, `R2_LIDTOP` shows its 28 wall rows, so a wrong venue is caught.
- **Contact policy on `R2_LIDTOP`** (pole 150 mm, GRIP): it plays with no plan JSON. Finger-link contact with the lid counts as pad contact; `R_ConnectorLink` contact counts as a collision.
- **Full hands.** The V6 P4 thumb-to-`L_CameraHolderLink` contact (around rows 2788–2828) is found. The slim link set is never accepted as a pass.
- **Seeded collision.** The full check reports the same first row as a brute-force check, and playback stops there with 7 joint markers, a band and red links.
- **Playback timing.** 1× plays 3,751 rows in 18.75 s ((3,751 − 1) × 5 ms) ± 1 %. The plot cursor row equals the 3D row.
- **Bin.** The bin settings change both the view and the verdict.
- **Performance.** On the operator PC, a 60 s pair is checked at every row in 60 s or less, a quick check in 10 s or less, a cached result in under 1 s, and a changed-window re-check in 5 s or less. The same timings are measured and recorded on the Orin.
- **Layout.** The split view works at 1366 × 768.

#### P7 — Test on robot and upload — Release 1

**Build:** 0.75 agent day. **Robot/lab:** 1 day (dry run and real runs, uploads to any slot with overwrite confirmation, an operator usability session).

**Goal.** A version whose exact bytes passed the 3D check runs on the robot: a dry run first, then a real run. Uploads go to any program ID 1–100 with a replacement confirmation, skip unchanged content and run on both arms in parallel. An E-stop during a run is reported honestly and recovery is guided. Release 1 ends with an operator usability session.

**Deliverables:**
- **Robot layer:**
  - gate check and checklist, including the ledger, slot match and both E-stop chips CLEAR;
  - transit check from the live pose, with a size threshold;
  - completion witnesses, stop index, verdict, run log, back-to-editing;
  - E-stop interruption: "INTERRUPTED — E-STOP" verdict, sequence cancel, pole stop and hand freeze on both sides, state logging;
  - recovery flow, including the E-stop recovery wizard (§6.1, E1–E4);
  - stop escalation and latency measurement;
  - emulator and bench tests;
  - operator test screens;
  - engineer console, rest;
  - planned-motion mirror and post-run pose.
- **R5 real-arm path:** post-run joint read, ordered matcher and time prior; "during the approach move" when nothing matches; E-stop stops mapped to a row and labelled "E-stop pressed".
- **R13 upload:**
  - registry and program-ID policy as in the control program (UI v53 + runner v14): any ID 1–100 per arm, per-file IDs in a batch, run by ID or name (`UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py:9-15, 4458-4480, 4781-4800`). Before each write, one list read under the arm's lock; an occupied slot shows its current name and size with Replace / Skip / Cancel the rest. Soft warnings, never blocks: a program not in Lite's ledger, ALIX ranges, V6 (71–83);
  - naming: 32 bytes or fewer, side-tagged, hash fragment;
  - per-file runner orchestration;
  - per-arm queue and ETA UI;
  - skip, dedupe and busy interlock (an E-stop does not block upload; Lite shows a note);
  - tests and hardware measurement.
- **Operator usability session.**

**Requirements served:** R4, R5, R13, INT-CTRL.

**Exit criteria:**
- **Commissioning.** A commissioning record exists for both controllers, including the E-stop drill items DR1, DR3, DR9, DR10 and DR11. REAL stays disabled for operators until it does.
- **REAL refusals,** one test each:
  - no 3D pass;
  - a failed 3D pass;
  - a failed dry run (no override);
  - a missing dry run (allowed only with an engineer override and a reason);
  - limits L1–L6 failing;
  - slot name or size mismatch;
  - mode not REAL;
  - an E-stop chip not CLEAR on either arm;
  - a blocking code;
  - a program already running;
  - a failed transit check;
  - a pole/hand read-back mismatch;
  - an incomplete checklist.
- **Completion.** A fake run that ends at 40 % reads "Stopped at about 40 %". A run that played under 85 % of the expected time reads "Not confirmed".
- **E-stop interruption.** An E-stop during a fake run whose runner exits 0 is recorded "INTERRUPTED — E-STOP"; no later sequence, pole or hand step starts; pole stop and hand freeze are sent; the log holds the state fields.
- **Recovery.** After a stop or an E-stop, every new run stays disabled until the recovery steps are done: read-back after clearing, speed restored, and a checked move to the start or safe pose that includes the venue walls.
- **Upload.**
  - Store only.
  - IDs 1, 50 and 100 succeed on the fake controller; 0 and 101 are rejected.
  - An occupied slot is written only after Replace; Skip leaves it unchanged; Cancel stops the rest of the batch. A slot holding a V6 or an unknown program shows a warning and can still be replaced after the confirmation.
  - A batch of three files with three per-file IDs writes each to its own ID.
  - Unchanged content sends 0 bytes and finishes in under 2 s after the list read.
  - One changed file out of 7 sends exactly one.
  - The two-arm parallel time is measured.
- **Hardware.** On each arm: a dry run, then real runs of 3 regions, with verdicts. Uploads to an occupied slot with the replacement confirmation. STOP latency during REAL is measured.
- **Usability.** At least 5 non-technical participants, 6 tasks. At least 4 of 5 complete each task unaided; zero critical errors; STOP pressed within 3 s in every trial; mean SUS 70 or higher.

#### P8 — Robot agents on `alix_dispatch`

**Build:** 0.5 agent day. **Robot/lab:** 0.5 day.

**Goal.** The Stage B executor. It meets the switch criteria (§4.4) and becomes the default; the Lite runner copy remains the fallback and oracle.

**Deliverables:**
- **Agent:**
  - `ArmSession` with one persistent connection and reconnect;
  - E-stop and status polling on the same connection;
  - `ProgramStore`;
  - run with `global_speed_guard` and the `stop_program` ladder;
  - run mode and pole/hand control;
  - ports of the workflow, drag-teach (with save retry) and global-point verbs;
  - heartbeat thread;
  - emulator and hardware equivalence tests.
- **Live mirror.**
- **One-session batch upload:** skip decided by read-back, with the same replacement confirmation as P7 (R13).
- **UDP capture:** `DP/recording.RunRecorder` with `verify_replay_from_frames` and a ghost pose (R5).

**Requirements served:** R4, R5, R13, INT-CTRL Phase 2.

**Exit criteria:**
- **Switch criteria met** and signed (§4.4).
- **Live mirror** shown during a dry run and a real run.
- **Collision rows.** An injected or real 4109 is mapped to its row through capture; an E-stop frame is mapped and labelled separately.
- **Batch upload.** 7 files go over 1 connection with at most 2 list reads.

#### P9 — Program builder and dual-arm workflows

**Build:** 1.0 agent day. **Robot/lab:** 0.5 day.

**Goal.** Merged programs whose untaught motion has been collision-checked, and dual-arm workflows that execute safely and stay compatible with v53.

**Deliverables:**
- **R8:**
  - planner, splitting, IDs and names;
  - link moves, program ends, detours, with V6 golden tests;
  - gates on the written integers;
  - world adapter for untaught motion: strict mode, scene plausibility gate, clearance margins, device-move sweeps, end-pose check, venue walls included;
  - outputs, provenance, staleness and first-run kit;
  - Program Builder UI;
  - TOGETHER clearance;
  - time matching and timeline export, using the P5 TOGETHER alignment (equal line count on both arms; the dual-arm timeline re-run on the retimed pair, because a clearance verdict is tied to both tracks' sha256, `DAV:18-22`).
- **R5 drift window.**
- **R10:** timeline model (programs at (lines − 1) × 5 ms, measured laws for the other steps); headroom and speed controls; tests.
- **Dual robot:** planner port and golden tests; dual stages and cross-arm abort (an E-stop on either arm stops both); two-phase start with skew logging.
- **Package writer and verifier.**

**Requirements served:** R8, R10, R5, INT-CTRL.

**Exit criteria (from C5, C8 and R10):**
- **V6 golden.** Re-merging the V6 segments reproduces every parsed row of the 7 merged files.
- **Gates on V6.** G2 fails exactly `R_P06P07` and `R_P10P12`. `R10_HNGR` is BLOCKED.
- **Untaught motion.** Every link move, transit, pole step and hand change is checked. V6's 64.6° and 45.7° transits are flagged.
- **Planner parity.** The V6 FINAL pair plans as TIMELINE-1 L[71,73] R[71,74,75] → LID-PREP-1 → LID-CHECK-1 → TIMELINE-2 [76] → serial 78 → serial 79 → TIMELINE-3 [80,83]. The exported v1 JSON loads and plans identically in v53, and the `side` field is checked.
- **TOGETHER blocks.** Both arms carry the same line count and the binding arm is named. A block is not READY without at least 25 mm windowed clearance over the predicted skew band on the retimed pair. Below that, SERIAL is proposed.
- **Hardware, after G4.** Supervised runs of one TOGETHER block and one RUN FIRST / WAIT OTHER pair: start skew is measured (target 0.2 s or less), and cross-arm abort takes 1 s or less under fault injection.

#### P10 — Offsets: FK → offset → IK, touch-off and testing

**Build:** 1.0 agent day. **Robot/lab:** 0.5 day.

**Goal.** Offsets that stay continuous and inside the limits, are reported honestly where they cannot be met, and are always simulated first.

**Deliverables:**
- **R9:**
  - solver: feed-forward, null-space pull, smooth tapers with no floor, local re-solve; G1 shared with R2;
  - span report with press-axis classes;
  - pole option;
  - I/O, provenance and undo;
  - panel and batch mode;
  - regression corpus.
- **R14:**
  - ghost and joint overlays;
  - moved-room world;
  - differential world check, arm-vs-arm and press checks;
  - ladder hooks;
  - touch-off with a 4-DOF fit;
  - slowed-run hooks.

**Requirements served:** R9, R14.

**Exit criteria (from C6):**
- **Identity.** A zero offset gives an identical file.
- **Continuity at ±10 mm on every V6 and raw file:**
  - every step ≤ max(1.5 × the taught step, 0.5°) and ≤ 10°;
  - no 360° wrap;
  - the sign of q4 kept.
- **Accuracy and limits.** 1 mm / 0.5° accuracy wherever the full offset is applied. G1 is never worse than in the input.
- **Honest reduction.** `R2_LIDTOP` +10 mm Z reports every reduced span with its press-axis effect. A toilet-moved offset that would press more than 2 mm deeper is blocked.
- **Touch-off.** A known 10 mm shift is recovered within 0.5 mm; a 2° turn within 0.3° from 2 marks at least 300 mm apart.
- **Ladder.** An offset file needs a 3D pass, a dry run (or a logged override for a missing one) and a slowed real run before full speed.
- **Speed.** A ±10 mm solve finishes in 30 s or less on the operator PC; the time on the Orin is measured and recorded.

#### P11 — Hardening and Release 2

**Build:** 0.5 agent day. **Robot/lab:** the operator field trial.

**Deliverables:**
- **Deployment:** clean installs on x86-64 and aarch64; the production install on the Jetson, including the remote-browser mode if it has no screen; packaging documents and release checklist.
- **Usability:** re-test and copy fixes.
- **Data:** project export and import.
- **Field trial:** operator use on the Jetson with fixes; operator and engineer guides; support bundle. Its length is set by the team (the earlier plan assumed two weeks).

**Exit criteria:**
- **Clean install.** Install on the operator PC and on the Jetson with no network, no ROS and no ALIX checkout. The self-test passes, and a full journey on the emulator makes requests only to 127.0.0.1 (or, in the remote-browser mode, only to Lite on the Jetson).
- **Field use.** Operators complete the full loop for at least 6 regions in one session without engineer help.
- **Data safety.** No data is lost under fault injection.
- **Update.** An operator updates and rolls back from a USB file.

### 7.4 Reference: conventional sizing

Before decision F8, this plan sized the same scope at about **280 person-days** for one developer writing and testing everything by hand (Release 1 about 198, Release 2 about 82), re-estimated bottom-up from the verified analyses C1–C8 with overlaps counted once. That figure already counted the reuse of ALIX Studio, the ALIX workspace libraries and the code in this workspace (§5). It is kept only to show the relative size of the phases: P5 and P6 are the largest; P11, P0 and P2 the smallest. It is not the schedule.

The agent-driven schedule (§7.2, §7.6) shortens the build because Claude writes the code and the tests on parallel tracks. It does not shorten the robot sessions, the engineer's review, the decisions or the field trial.

### 7.5 What to cut first if time is short

The analyses' own MVP levers move these items from Release 1 to Release 2. None of them weakens a safety gate, the E-stop handling, the wall check or the target-time solver.

| Cut | Size |
|---|---|
| R1: crops instead of drawn highlights; pause/resume limited to crash recovery; tool and glove sub-steps kept BLOCKED (C1's MVP) | M |
| R12: ship only the gating detectors (SD1 band-limited, SD2, SD5) with markers and basic suggestions; defer SD3, SD4, SD6 and the advisory fixes | M |
| R3: defer whole-flow playback and wiped-area painting; keep the pad trail | S |
| R6: defer path-linked compare zoom and per-screen layout persistence | S |
| INT-TRAJ: defer the dual-arm lane | S |
| R11: defer the tier-2 background dual check (it belongs with P9) | S |
| NFR-THEME: defer the dark toggle | S |
| R13: defer background pre-staging | S |
| R4: defer the "reverse out" recovery option; keep the checked move to the safe pose and hand guiding | S |

If the aarch64 toppra build is not ready, the Jetson bundle can ship with the default optimiser only; the TOPP-RA option is then greyed out on that machine.

### 7.6 Agent schedule, step by step

The steps follow the order decided on 13 Oct 2026 (D27, §7.1): UI/UX first, then the backend. Each step carries the agent-day estimates of its phases (§7.2); only the order changed, so Release 1 still takes about 5 agent days (range 4–6) and Release 2 about 3 more. Lab sessions need the robot and the engineer and run alongside the build at the step shown. The agents work on the tracks in parallel; the engineer reviews and merges at the end of each day.

| Step | Agent build (parallel tracks) | Robot/lab session (engineer) | Result |
|---|---|---|---|
| **1. UI/UX first** | The operator workflow screens on the ALIX Studio theme, working against the existing optimiser and stand-ins for the robot (the fake SDK, the fake recorder and the emulator). **Foundation (P1), UI part:** skeleton, theme tokens, shell and UX framework, launcher; x86-64 and aarch64 build jobs. **Screens:** dashboard and Regions tab with the catalogue's cleaning times and recording import (P2); Teach card (P4); Optimise screen with the optimiser panel, segments, undo and adjustable plots (P5); 3D Check screen (P6); robot widgets, E-stop chips and Test on robot screens (P3, P7). This step uses the screen parts of the P1–P7 estimates. | Engineer: D3 (catalogue and cleaning times), D13, D14; candidate venue file; prepare the lab day; wireframe review with the requester and one operator (P1). | Operator screens on the theme, working on stand-ins |
| **2. Optimiser golden tests and integration** | **Optimise (P5: 1.0 agent day in all):** golden tests (§8.1.1) for limits L1–L6, the default optimiser with the three fixes, the TOPP-RA adapter with the parity test, and the target-time solver; the optimiser wired to the Optimise screen; spike findings. aarch64 bundle with toppra built from the tag; golden tests on both architectures; one solve timed on the Orin. | — | Optimisers pass their golden tests on both architectures |
| **3. 3D check with walls** | **3D check (P6: 1.0 agent day in all):** commode in its state, venue walls, bin toggle, sweep worker, joint markers, stop at first collision; contact policy, full hands, performance, split view, R11 tiers 1–2. | — | 3D check complete |
| **4. Robot I/O with the IO3 E-stop monitor** | **Robot (P3: 0.5):** Lite runner copy with the E-stop fix, fake-SDK harness, settings, the IO3 E-stop monitor (mode 5) behind the E-stop chips, gateway, software STOP. **Test and upload (P7 core):** upload to any ID 1–100 with the replacement confirmation, dry run and real run behind the gates, verdicts, E-stop interruption. **Ground truth (P0 software; P0: 0.5):** bundle builder and self-test, venue loader and wall check, characterisation and drill scripts. | **P0 lab day:** software STOP (G4) and IO E-stop drill, SIM-mode pole/hand, upload timing, start skew, venue/wall check with known-good recordings. Then the **P3 lab (0.5):** connect, modes, E-stop chips, Prepare pole/hand on both arms. | Robot connection proven; characterisation and drill report |
| **5. Server and API** | **Foundation (P1), server part:** server scaffold, request protection with the per-launch token, project store and scope guards; the API the screens call, replacing the stand-ins. **Integration:** the core loop end to end on the fake controller and the emulator. Fixes from the P0 lab findings (stop verb, E-stop level, recovery steps). | The **supervised core-loop dry run** (controller SIM mode). | **Core loop** on the fake controller and emulator; core loop dry run on the robot |
| **6. Teaching and Release 1** | **Teach (P4: 0.5)** on the fake recorder, then the P4 hardware fixes. **P7 completion (P7: 0.75 in all):** recovery wizard, completion witnesses, R13 queue, skip and parallel arms, run log. Release packaging for both architectures. | **P4 lab (1):** teach 3 regions in a row; crash recovery once on hardware. **P7 lab (1):** dry run and real runs on each arm, uploads to occupied slots with the replacement confirmation, STOP latency, the operator usability session. | First Lite teach session; **Release 1** |
| **7. Robot agents** | P8 (0.5): agents on `alix_dispatch` with E-stop polling on the same connection, live mirror, one-session upload, UDP capture. P9 starts. | **P8 lab (0.5):** agent equivalence on hardware, live mirror. | Agents meet the switch criteria |
| **8. Programs and dual** | P9 (1.0): program builder, dual coordinator, TOGETHER with equal line counts and the timeline re-check. P10 starts. | **P9 lab (0.5):** one TOGETHER block and one RUN FIRST / WAIT OTHER pair, after G4. | Programs and dual on hardware |
| **9. Offsets and Release 2** | P10 (1.0) completion: offsets, touch-off. P11 (0.5): installs on both architectures, guides, export/import. | **P10 lab (0.5):** touch-off and the offset ladder to a slowed real run. Then the **operator field trial** on the Jetson. | **Release 2** after the field trial |

**Lab-day dependencies:**
- The P0 lab day comes before any robot motion from Lite. Its E-stop drill items DR1, DR3, DR9, DR10 and DR11 must pass before operators run real tests (P7).
- The venue/wall check (P0 lab) comes before any 3D pass counts toward a real run; until then the verdict says "venue not verified".
- The P3 lab session (connection, modes, E-stop chips) comes before the core-loop dry run and before teaching on the robot (P4).
- The P7 lab day needs the P6 3D gate, the P0 commissioning record and taught regions from P4.
- The aarch64 work in step 2 needs access to an Orin or an aarch64 build machine.
- P9's TOGETHER hardware test needs the G4 result (P0) and P8's two-phase start.
- P10's lab session needs the P7 ladder.
- Each lab session that slips moves the result in its row, and every row that depends on it, by the same number of days. The field trial adds calendar time that the agent schedule does not cover.

### 7.7 Milestones

The milestones follow the order of §7.1 (D27). The agent-build column gives each milestone's existing estimate from §7.2; the steps are those of §7.6.

| Milestone | Shows | Agent build (§7.2) | Robot/lab session needed |
|---|---|---|---|
| M0 **UI/UX first** | The operator workflow screens on the ALIX Studio theme, working against the existing optimiser and stand-ins for the robot: dashboard, Teach card, Optimise, 3D Check and Test on robot, with the E-stop chips | The screen parts of the P1–P7 estimates (step 1) | — |
| M1 Optimiser golden tests and integration | Raw → sendable version at the target time or the closest feasible time, with what binds; both optimisers on both architectures; the golden tests of §8.1.1; undo; spike findings; resizable plots | P5: 1.0 | — |
| M2 3D check with walls | Both arms with commode, venue walls and bin; stop at the first collision on the joint plots | P6: 1.0 | — |
| M3 Robot I/O with the IO3 E-stop monitor | IP entry; three modes; the IO3 E-stop monitor and chips; software STOP; Prepare pole and hand on both arms; upload to any ID 1–100 with the replacement confirmation | P3: 0.5 | P3 lab (step 4) |
| M4 Server and API | The local server with its token, the project store and the API the screens call, replacing the stand-ins; install from file; the 34 first-release units with target times; imported v53 recordings; V6 legacy coverage | P1: 0.5; P2: 0.25 | — |
| M5 Ground truth | Characterisation and E-stop drill report; venue verified with known-good recordings; the V6 pair in the unchanged Studio; the bundle self-test failing closed on a clean machine | P0: 0.5 | P0 lab day (step 4) |
| M6 **Core loop** | Teach through the Lite runner copy; optimise with both optimisers and a target time; 3D check with walls; upload to any slot with the overwrite confirmation; run. On the fake controller and the emulator, then a supervised dry run on the robot | About 2 for the core-loop scope (§7.2) | Dry run after the P3 session (step 5) |
| M7 First Lite teach session | 3 regions taught in a row with countdown and auto-advance | P4: 0.5 | P4 lab (step 6) |
| M8 **Release 1** | Dry run and real run on the robot behind the ladder; verdicts; E-stop interruption and recovery; uploads to any slot with the replacement confirmation; usability session passed | About 5 in total (range 4–6) | P7 lab (step 6) |
| M9 Agents | Live mirror; exact collision rows; one upload session per arm | P8: 0.5 | P8 lab |
| M10 Programs and dual | A checked V6-equivalent program set; TOGETHER block on hardware after G4 | P9: 1.0 | P9 lab |
| M11 Offsets | Touch-off → offset → 3D → dry run → slowed real run | P10: 1.0 | P10 lab |
| M12 **Release 2** | Clean installs on both architectures; field trial on the Jetson | About 3 more (range 2–4) | Field trial |

---

## 8. Verification and test strategy

### 8.1 Test layers

All automated layers run on both target architectures: x86-64 (the operator PC used now) and aarch64 (the robot's Jetson AGX Orin, used in production). An aarch64 CI runner or a scheduled run on the Orin is part of P1.

| Layer | What it covers | How | Runs |
|---|---|---|---|
| **Algorithm golden tests** | Recipe processor, gates G1–G5, spike detectors SD1–SD6, merge planner, offset solver | Golden inputs: `UC/zip/`, `UC/plan_txt_files/`, the V6 programs and bridges, the ALIX commode C raw recordings. Expected outputs are frozen with their sha256. Parity checks: "Classic" reproduces `zip/02` from `zip/01` within 1 mdeg; the JS preview equals the Python output within 1 mdeg; Lite's `.proj` bytes equal R14 `write_project`; every output passes R14 `read_track` and `DP/track.parse_track`. | Every commit |
| **Optimiser tests** | The two optimisers behind one interface (`lite/optimise/`): the default (the prototype's trim → smooth → scale, with the three fixes) and TOPP-RA; the target-time rule; the shared hard limits L1–L6 | The golden set in §8.1.1, on the commode C recordings copied into Lite's test data with their sha256. All limits are judged on the integer bytes as written, at 5.0 ms per line (`TT:94-102`). | Every commit, on both architectures |
| **Library parity and drift** | Imported ALIX code, copied Studio code, design tokens | Tests pinned to one ALIX commit. Copied JS functions are compared with their source through the Studio's `_function_body` helper. Token parity against `ALIX/design/_tokens.css`. A nightly canary runs the suite against ALIX HEAD and reports drift, including the TOPP-RA parity test. | Every commit + nightly |
| **Scope guards** | X1/X2 stay out | Import-boundary test (deny-list plus light-module whitelist), route allow-list, release-content test | Every commit |
| **Collision regression** | The 3D check | Seeded collisions confirmed by a brute-force every-row check; the V6 thumb-to-wrist case; the `R2_LIDTOP` contact-policy case; bin settings change the verdict; FK bit-identical at fixed joint vectors; the world self-test's negative cases (missing mesh, SRDF, `commode_c` scene or venue file gives NOT CHECKED, never "clear") | Every commit |
| **Wall collision** | Walls enforced from the selected venue file (`ALIX/ros/orchestration/alix_tasks/config/scene/washroom_*.yaml`, loaded through `WS/plan_scene_validation.py` `load_wall_manager` / `PlanWallChecker`) | §8.1.2 | Every commit; the venue check again on site (§8.3) |
| **Robot I/O without hardware** | Runner wrapper, gateway, ladder, upload ledger, teach flow, E-stop state handling | A fake `Robotic_Arm` SDK, passed with `--sdk`, runs the pinned runner (v14 plus the small E-stop patch, §9.3) in CI (pattern: `ALIX/core/python/alix_dispatch/test/test_rm_upload.py`). The fake exposes IO1–IO4 with `io_mode` and `io_state`, so an E-stop can be pressed and released in a test. The ALIX emulator and `emulated_program` cover the library path. A fake recorder replays stored recordings for drag teaching. | Every commit |
| **E-stop handling (simulated)** | Chip states, blocking, interruption, recovery wizard | §8.1.3 | Every commit |
| **Robustness** | Data and stop paths | `kill -9` during save (100 runs); a crash during drag teaching; backend death during a run (the runner must slow-stop and restore the speed); network refusal (no token, foreign Origin) | Every commit |
| **UI and accessibility** | Operator flows | Playwright journeys on the fake controller: teach 3 regions, optimise to a target time, 3D check, send, dry run, real run, verdict, E-stop and recovery. Layout audit at 1366×768 and 1920×1080. Contrast and target-size checks. Assertion that no request leaves 127.0.0.1 (or the configured robot-network address in the Jetson's remote mode). | Every commit |
| **Hardware protocol** | Behaviour only the robot can show, including the IO E-stop drill and the software STOP test | Supervised bench sessions (§8.3) | P0, P3, P4, P7, P8, P9, P10 |
| **Usability** | Non-technical operators | Wireframe review, pilots and formal tests (§8.4) | P1, P2, P4, P7, P11 |

#### 8.1.1 Optimiser golden tests

The expected values were measured on 13 Oct 2026 with the ALIX venv (numpy 2.5.3, scipy 1.18.1, toppra 0.6.10) and are listed in the optimiser research notes (§6.3 there). Durations shown to the operator are (lines − 1) × 5 ms (D26), the same time base as the line counts below.

| Test | What it proves | Input | Expected |
|---|---|---|---|
| **TOPP-RA parity with ALIX** | Lite's adapter and its own `retime_toppra` reproduce the ALIX chain (5 Hz `smooth_taught` → `strip_pauses` → `TaughtPath` + 30 mm `arclen_smooth` → `retime_toppra` at 0.95 × ratings and 600 °/s², `Interpolation`, `seidel`; `ALIX/experiments/exp01_teach_trajectory_reverse_engineering/src/emit_toppra_rung.py:306-330`; `ALIX/docs/motion/TAUGHT_CANFD_PIPELINE.md:17-40`) | FILE_01.1, no post low-pass | 3839 lines (19.19 s at 5 ms), equal to slot 101 (`TAUGHT_CANFD_PIPELINE.md:588`); bytes within ±1 millidegree of the ALIX chain (library versions vary by up to 1 millidegree, `emit_toppra_rung.py:417-427`) |
| **TOPP-RA T_min** | The shortest legal time is found | FILE_08, FILE_10.1, FILE_6 | 3360 lines (16.80 s), 4.26 s, 20.25 s, each ±1 line |
| **Target-time accuracy, both optimisers** | A reachable target is hit to the tick | FILE_10.1 (TOPP-RA) at k = 1.3 and 2.0; FILE_01.1 (default) with a 60 s target | Achieved line count within ±1 line of the target's line count (target ÷ 5 ms + 1: 12,001 lines for 60 s); for TOPP-RA, speed use ≤ 0.95/k (+2 %) and windowed acceleration ≤ 1/k² of the limit (+2 %), and the path equal to the k = 1 path within 0.001° at matched fraction (uniform time scaling) |
| **Closest feasible time and binding limit** | An unreachable target returns T_min and names what binds | FILE_10.1, TOPP-RA, target 3.0 s; the default with a target below its T_min | `met = false`; achieved = T_min; TOPP-RA: binding[0] = joint 6 acceleration, about 38 % of the time, window 2.88–3.05 s; default: the check that fails at 0.995 × k_min is named with joint, quantity and window |
| **Default T_min** | The bisection on the scale factor finds the fastest legal time | FILE_08, FILE_10.1, FILE_01.1, window 41 | 27.42 s, 8.14 s, 38.30 s, each ±1 % |
| **Limit compliance, both optimisers** | Every output passes L1–L6 on its integer bytes: L1 G1 margin ≥ 0.5° (`TT:466-480`); L2 20 Hz band-limited speed ≤ 0.97 × ratings; L3 acceleration ≤ 600 °/s² × 1.02 as a 50 ms windowed mean; L4 step ≤ 10° per line; L5 path change against the raw recording ≤ 10 mm or the region's MAX DEV; L6 first and last line within 0.05° of the recorded poses | All golden recordings; a property test over 20 files and a grid of k ≥ k_min | Every hard limit passes for every k ≥ k_min (monotonicity); a file that fails any L-limit can be neither saved as "ready" nor uploaded |
| **Default reproduces the prototype** | The default optimiser is the prototype's algorithm | Golden recordings with the prototype's settings; the three fixes switched off | Lite's output equals the prototype's (`HTML:1438-1616, 1676-1724`, run headless from the extracted JS) within 1 millidegree. With the fixes on, only the documented differences remain (next three rows) |
| **Default fixes (regressions)** | The three measured defects stay fixed | FILE_08 smoothed with the old 101 window; FILE_01.1 with segments [75, 80, 95, 115, 75]; FILE_08, FILE_10.1, FILE_01.1 at window 41 | First and last step ≤ 100 millidegrees (was 546–1005); windowed acceleration ≤ 300 °/s² at every segment boundary (was 640–919); pad path change ≤ 4.6 mm, and window 101 fails L5 with a "reduce smoothing" hint |
| **TOGETHER blocks** | Both arms get one duration | A left and a right recording | Equal line count N = max(target, left T_min, right T_min); the report names the binding arm; the dual-arm collision timeline is re-run on the retimed pair (verdicts are bound to both tracks' sha256, `DAV:18-22`) |
| **Lite without toppra** | The aarch64 bundle still works if toppra is missing | `toppra` import blocked | Lite runs; TOPP-RA is greyed out with a message; the default optimiser works |
| **Writer format** | Output files load everywhere | Any result | `{"point":[7 ints]}` lines (`TT:737-745`); parses with `DP/track.parse_track`; `.proj` CRLF |

#### 8.1.2 Wall-collision tests

| Test | Expected |
|---|---|
| **Walls are enforced** | A seeded track that crosses a wall plane of the selected venue is reported as a wall collision at the right rows, with the venue file name in the verdict. |
| **Wrong venue is visible** | With the 800 mm venue (`washroom_800mm_wide_scene.yaml`), the ALIX lid-top recording shows the 28 of 188 wall rows found in the analysis (C3 §2.5); with the 1000 mm and 1400 mm venues it shows 0. This fixes the expected behaviour of the wall checker, not the choice of venue. |
| **Known-good recordings clear with the selected venue** | 3–5 recordings that ran on the test cell without contact give no wall or structural contact with the venue file chosen for that cell. Until this test has passed on site, wall results carry "venue not verified". |
| **Missing or unselected venue** | No venue selected, or the file missing, gives "Walls: NOT CHECKED", never a pass; the world manifest records the venue file and its sha256. |

#### 8.1.3 E-stop handling tests (fake SDK)

- The chip shows CLEAR, E-STOP ENGAGED, RECOVERING, UNKNOWN and NOT CONFIGURED from `io_mode`/`io_state` on IO3 (decision D25). It never derives ENGAGED from `arm_current_status == 9` alone, because that value is sticky after any stop (`ALIX/ros/hw/alix_hw/src/rm_side_system.cpp:1856-1860`).
- While either arm is not CLEAR, every motion route on both arms is refused on the server side (Run, Sequence, Test on robot, Move to start, Pole, Hand, Drag-teach START, switch to REAL). Uploads to slots 1–100 still work.
- An E-stop during a run marks the run "INTERRUPTED — E-STOP", cancels the rest of the sequence, sends pole stop (`rm_set_lift_speed(0)`) and hand freeze, and writes the event log. The patched runner prints `ESTOP ENGAGED <side> io=<n>` and never "RUN COMPLETE" for that run.
- A release during a sequence starts no further program or AUX step.
- The recovery wizard goes ENGAGED → RECOVERING → CLEAR only through its steps, reads back after every clear, and shows the collision-checked transit (walls included) before a re-run.

### 8.2 The validation ladder for every changed file

Each rung is recorded against the file's **fingerprint** (exact bytes plus context). Any edit makes later rungs stale, and stale is never a pass.

| Rung | Where | What it proves | What it does not prove |
|---|---|---|---|
| **Gates** (L1–L6, G1–G5, SD1–SD6) | Lite, instantly | Speeds, accelerations, steps, limit margins and path accuracy are within the rules | Anything about collisions |
| **3D check** | Lite simulation worker; scene from the URDF and the `commode_c` scene, walls from the selected venue file | No collision in the model with both arms, commode state, walls and bin | That the model matches the room (the venue check with known-good recordings covers the walls) |
| **Dry run on the robot** | Real controller in SIM mode; arms do not move; pole and hand steps skipped | The program loads, plays to the end, its timing, and no controller error codes | Contact, pole and hand behaviour |
| **Real run, slowed copy** | Real arm, first run of new or changed content | First physical contact behaviour at low risk | Full-speed behaviour |
| **Real run, full speed** | Real arm | Production behaviour, with an operator verdict | — |

### 8.3 Hardware test protocol

- **Before any session:**
  - a commissioning record exists for both controllers (P0): the IO E-stop drill items DR1, DR3, DR9, DR10 and DR11 passed on each arm; software STOP on a running stored program (G4); speed restore; mode read-back; transit behaviour; start skew;
  - both arms use the same collision-detection level (D9 in §11);
  - the hardware E-stop buttons are within reach and Lite shows both E-stop chips CLEAR;
  - the venue file for the cell is selected and verified (§8.1.2);
  - the area is clear and the bin state has been declared.
- **Staging:**
  - new or changed content runs from a slowed first-run copy;
  - transits run at 30 %;
  - full speed only after a "Looks good" verdict on the slowed copy;
  - right-arm TOPP-RA files get a supervised first run, because ALIX's TOPP-RA emitter is left-arm only and has run on hardware for left-arm recordings only (`TAUGHT_CANFD_PIPELINE.md:555-556`; `ALIX/experiments/exp01_teach_trajectory_reverse_engineering/src/emit_toppra_rung.py:71`);
  - TOGETHER dual blocks run only after software STOP behaviour (G4) has passed.
- **IO E-stop drill.** Per arm and with both arms, in REAL mode with the workspace clear, recording the UDP push and the polled state at 10 Hz. The drill items DR1–DR14 are numbered in the E-stop research notes (§5.6 there; not the decisions of §11): both arms idle, left then right button (DR1, DR2: one chain or two, IO number and level, enable, power and brake flags, chip red within 0.5 s); a running stored program on each arm (DR3, DR4); both arms running (DR5); a moving pole (DR6); a moving hand (DR7: the E-stop does not stop the hand, Lite must freeze it); drag teach (DR8); every command tried while the button is held (DR9); a quick release during a sequence (DR10: no next step starts); recovery with the wizard (DR11); PC or network failure during a run (DR12: the IO E-stop still stops the arm); an open-circuit wiring check with the electrician (DR13); and the software STOP test (DR14). DR1, DR3, DR9, DR10 and DR11 must pass before operators use Lite in REAL mode. Results go to the site config and the drill log; Lite flags an overdue drill.
- **Software STOP test (G4).** Lite's STOP during a stored-program run, with `slow_stop` and with `stop`: does the program end, what is the latency and stopping distance, do the pole and hand stop. Repeated at P3, P7 and P8 on each arm. Its result decides which verb Lite's STOP uses (`DP/emulated_program.py:51-56` supports both outcomes).
- **Records:** every run writes its record (fingerprint, slot, IP, mode read-back, codes, final joints, completion evidence, E-stop events, verdict). Sessions are logged in the audit file.
- **Re-commission** after any controller firmware change or E-stop wiring change. Re-select and re-verify the venue file when the robot moves to a different cubicle, and re-run the known-good check after a change to the commode or bin in the scene.

### 8.4 Usability testing

| When | Who | What | Pass mark |
|---|---|---|---|
| P1 | The requester and one operator | Wireframe review of Dashboard, Teach card, Optimise and Test on robot | Agreed changes recorded |
| P2, P4 | One non-technical tester | Find and open the next untaught region; teach 3 regions with Prepare / START / STOP / Keep | Done without help |
| P7 (Release 1) | At least 5 non-technical participants | 7 tasks: teach a region, optimise to a target time (and accept the closest feasible time when offered), run the 3D check and go back to editing, dry run, real run, stop, recover after an E-stop with the wizard | At least 4 of 5 finish each task unaided; zero critical errors; STOP pressed within 3 s in every trial; mean SUS 70 or higher |
| P11 (Release 2) | Operators on site | Two-week field trial and a re-test | The full loop for at least 6 regions in one session without engineer help |

### 8.5 Traceability

Every acceptance criterion in the requirements document maps to an automated test or a protocol step. The repository keeps a matrix (requirement ID → test IDs → last result). A release is cut only when every Must criterion for its scope passes or has a signed waiver.

---
## 9. Safety approach

### 9.1 Hazards and controls

The hazard workshop in P0 confirms and extends this list. Each hazard gets at least one control and one test.

| Hazard | Main causes | Controls in Lite | Test |
|---|---|---|---|
| Unexpected arm motion | A UI action moves the robot without the operator expecting it | Motion gateway: arming per arm, one-time confirmation tokens bound to the exact command; confirmations name arm, mode, IP and region; screen navigation and auto-advance never move the robot | A reused or expired token is refused; no motion route works without a token |
| A person in reach while the arm moves | Operator or bystander in the cell | **IO E-stop buttons on both controllers are the primary safety stop** (§9.3.1); Lite shows their state per arm and blocks all motion while either is not CLEAR; Lite's STOP is a secondary controlled stop (§9.3.2) | IO E-stop drill DR1–DR14 (§8.3); simulated E-stop tests (§8.1.3) |
| E-stop not seen by software | The runner watches only `run_state` and error codes, so an E-stop during a run would very likely read "RUN COMPLETE, no error" and a sequence would start its next step (`R14:219-271, 641-700`, inferred) | Chip from `rm_get_io_state` on the IO in mode 5; runner v14 E-stop patch; run marked INTERRUPTED; sequence cancelled; pole stop and hand freeze; never auto-resume | Drill DR3, DR10; fake-SDK E-stop tests |
| Collision with the toilet, walls, other arm or bin | Taught motion, merged joins, transits, offsets, recovery moves | Every-row 3D check in the world built from the ALIX URDF and the `commode_c` scene (commode pose and articulation, bin, UGV, poles), **with the walls enforced** from the selected venue file; full hands and a contact policy; transit check from the live pose (also after an E-stop); drift window for two arms; slowed first runs | Seeded-collision suite; wall-collision tests (§8.1.2); REAL refusals (§7.3, P7) |
| Wrong venue file | The venue does not match the test cell: false wall hits or missed wall hits (the 800 mm venue gives 28 of 188 false wall rows on a known-good recording, C3 §2.5) | Venue chosen in Engineer settings and recorded in every verdict; "venue not verified" until known-good recordings clear; NOT CHECKED when no venue is selected or the file is missing | Wall-collision tests; on-site venue check (§8.3) |
| Joint over-speed, over-acceleration or limit overshoot | Speed-up by either optimiser, smoothing artefacts, boundary jumps, a target time that is too short | Both optimisers must pass the hard limits L1–L6 on the written bytes; a target below the shortest legal time returns that time instead and names the binding limit; the hard limits L1–L6 and Must-fix spike findings (SD1–SD6) block saving, upload and runs, and G5 is a warning (D11); global speed restored | Optimiser golden tests (§8.1.1); V6 gate tests (`R_P06P07`, `R10_HNGR`) |
| The optimised path leaves the taught path | TOPP-RA chain smoothing (5 Hz, 30 mm arc length) or the default's moving mean bends the pad path | L5: pad path change against the **raw** recording at most 10 mm, or the region's MAX DEV; 15 mm arc-length smoothing for regions with a MAX DEV; L5 at the region's MAX DEV, G5 a warning (D11) | Default G4 regression; TOPP-RA L5 check on all golden files |
| Software STOP does not end a running program | Slow stop untested on stored programs; client died | The IO E-stop is the safety stop; P0 software STOP test (G4, drill DR14); one-signal STOP; fallback stop ladder on a fresh connection; "Press the E-stop now" escalation; REAL disabled for operators until commissioning | G4 probe record; stop-latency measurement |
| Two command sources on one arm | v53, the ROS stack or a stray runner active | One command source per arm; per-arm lock; process interlock | Robot commands refused while v53 or a foreign runner runs |
| Pole crushes or the arm hits the bin | Pole moved with the arm moving, in drag mode, holding the fixture, or through the stowed-bin band | Arm-still check; no pole/hand in drag mode; checklist; stowed-bin band checked over the whole travel; pole stop on any E-stop or STOP | Interlock unit tests; hardware checklist; drill DR6 |
| Hand pinch | Fingers closing near a person; the IO E-stop does not stop the hand (end-port power stays on, `ALIX/docs/TEST_PROCEDURE.md:453`) | Hand moves only after confirmation; "hands clear" checklist; never in drag mode; Lite freezes the hand on any E-stop or STOP | Checklist enforced in the Prepare step; drill DR7 |
| Wrong arm or wrong file; accidental overwrite | Swapped IPs; stale or foreign program in a slot; a slot reused by mistake | Identity check; refusal of identical IPs; any slot 1–100 can be written, as in the control program, but an occupied slot is replaced only after a confirmation that shows the current program's name and size; soft warnings (not blocks) for slots Lite's ledger did not write or slots in ranges others use; upload ledger; name and size read-back before every run | Overwrite-confirmation, swapped-binding and slot-mismatch tests |
| False confidence from simulation or dry run | Missing scene or venue file; unverified venue; SIM mode taken as a robot test | NOT CHECKED instead of pass; world manifest (scene, venue file, sha256) on every verdict; wording "Dry run: the arms do not move"; dry runs never count as robot tests | Self-test negative cases; UI copy review |
| Remote misuse | Another machine reaching the Lite server | Loopback bind; per-launch token; Host and Origin checks; authenticated remote mode only for a headless Jetson (§9.7) | Network refusal tests |

### 9.2 Gates, overrides and waivers

- **Never overridable:**
  - band-limited speed over a joint rating (G2 / SD1 / L2);
  - a step over 10° per line (G3 / SD2 / L4);
  - commanded joint limits;
  - the near-limit rule (G1 / SD5 / L1, Must fix);
  - an operator target time that would break a limit (Lite delivers the closest legal time instead);
  - a collision found with the complete scene (URDF, `commode_c` scene and a verified venue file);
  - a missing URDF, mesh, SRDF or `commode_c` scene (the self-test fails and the 3D check refuses to run);
  - an E-stop chip that is not CLEAR, including UNKNOWN (NOT CONFIGURED may be overridden by a supervisor in SIMULATION mode only);
  - a failed dry run;
  - a slot name or size mismatch at read-back;
  - a run-mode mismatch;
  - a blocking controller error code.
- **Changeable only by an engineer, as a recorded setting with a warning:**
  - the L3 acceleration limit (600 °/s², the arm's hardware limit, as a 50 ms windowed mean; decided, D23);
  - the 6 Hz post low-pass on TOPP-RA output (off by default; it raises the acceleration reading to 1.17–1.25 ×, measured 13 Oct).
- **Overridable by a supervisor or engineer, with a recorded reason:**
  - **Walls NOT CHECKED or "venue not verified".** No venue selected, or the selected venue not yet verified with known-good recordings. The first real run must then use the slowed copy.
  - **A missing dry run** (engineer only).
  - **Predicted path departure (G5).** A warning only (D11): ALIX's model reads 33–109 mm on the raw commode C recordings, which ran fine on the robot, so it cannot gate a file. A region's MAX DEV tightens the path-accuracy limit L5 instead, with no override.
- **Overwrite of an occupied slot.** Not a gate: Lite asks for confirmation (Replace / Skip this file / Cancel the rest of the batch), shows the program currently in the slot and logs the replacement. Soft warnings never block (decision D2).
- **Per-pair waivers** cover proven model gaps only, for example a known false contact. Only an engineer can raise one, scoped to one link pair, one fixture part and one world version (URDF, `commode_c` scene and venue file, by sha256), with an audit record. A waiver expires when any of these files changes.
- **Logging.** Every override, setting change, overwrite and waiver is written to the ladder and the audit log, with the fingerprint, the reason and the person.

### 9.3 STOP

#### 9.3.1 IO E-stop: the primary safety stop

- **What it is.** Each arm controller has E-stop buttons wired to an IO input. The site reports that they work well and stop everything without cutting power (decision F4). On these Gen-3 controllers any of IO1–IO4 (16-core IO cable, pins 5–8) can be set to function 5, "input emergency stop function multiplexing", on the pendant or through `set_IO_mode` / `rm_set_io_mode` (HW p.39–40; TP p.112, p.131; JSON p.118; SDK `rm_robot_interface.py:2882`). The input triggers on a HIGH level per the RealMan demo (DEMO p.82); the level on this robot is confirmed in the drill.
- **What it stops (from the documents and ALIX field notes).** No RealMan document gives its stop category, safety rating or reaction time. The pendant E-stop stops the arm "at maximum speed" (TP p.11), and the API groups the IO, pendant and JSON E-stops as one kind of external stop (CPP p.318), so the IO E-stop is very likely the controller's quick stop with power kept (inferred). During the 2026-08-06 E-stop both controllers rejected pole commands, one E-stop chain covered both arms, and end-port power survived so the hand kept working (`ALIX/docs/TEST_PROCEDURE.md:216-242, 453`). ALIX code assumes a physical E-stop drops joint power and enable flags (`ALIX/ros/hw/alix_hw/README.md:449`; `rm_side_system.cpp:1855`); this conflicts with "no power cut" and is measured in the drill (DR1).
- **Wiring caution.** RealMan advises connecting safety devices to safety interfaces, not normal IO (HW p.32). A HIGH-trigger input may not trip on a broken wire; drill DR13 checks this with the electrician, and the site decides whether to rewire (normally closed contact and safety relay). Lite never writes IO modes.
- **Detection.**
  - At connect, Lite reads IO3, the E-stop input on both controllers (decision D25), checks `io_mode == 5`, and reports the modes of IO1–IO4. It polls `rm_get_io_state(n)` at 5–10 Hz through the single connection owner per arm (`rm_robot_interface.py:2927`; JSON p.120). Not `rm_get_io_input`, which returns −1 for a port not in input mode (PY p.160).
  - Cross-checks: the joint enable flags, and optionally Modbus input register 302 "Emergency Stop Status" if the site enables Modbus-TCP (MB p.8; REL p.14; TP p.125).
  - Never from `arm_current_status == 9` alone: it is sticky after any stop (`rm_side_system.cpp:1856-1860`). There is no E-stop entry in the system-error table, so `err` most likely stays 0.
  - Start-up check: IO3 must be in mode 5 on both controllers, otherwise motion is blocked (RealMan's demo maps IO3 to "continue program" (mode 4) and IO4 to the E-stop (`alix/external/butterfli-realman-alix/Demo/RMDemo_Python/RMDemo_IOControl/readme.md:68`), so a controller left in that mapping would resume a program instead of stopping it). The E-stop state holds from the press until the button is released, which is the manual reset; Lite's chip follows the same rule and then shows RECOVERING until the recovery steps are done. The site config stores, per arm, the E-stop IO number, its active level and the last-drill date; Lite flags an overdue drill.
- **What Lite shows.** One chip per arm: CLEAR, E-STOP ENGAGED, RECOVERING (released, not yet recovered), UNKNOWN (no fresh reading for more than 1 s) or NOT CONFIGURED. UNKNOWN and NOT CONFIGURED block motion exactly like ENGAGED.
- **What Lite blocks.** While **either** chip is not CLEAR, the server refuses on **both** arms: Run, Sequence, Test on robot, Move to start, Pole, Hand, Drag-teach START and the switch to REAL. Uploads to slots 1–100 stay allowed, with a "robot in E-stop" note.
- **E-stop during a run.** Lite marks the run "INTERRUPTED — E-STOP" (never COMPLETE), cancels the rest of the sequence, sends pole stop (`rm_set_lift_speed(0)`) and hand freeze on both sides, and logs IO, enable, power, brake, `err` and `run_state` with timestamps.
- **Runner v14 patch (small).** Read the E-stop IO in `arm_state()` and in the run and sequence loops; refuse to start while it is held, with the reason; report "interrupted" instead of "RUN COMPLETE"; never start the next sequence step after an E-stop; print parseable `ESTOP ENGAGED <side> io=<n>` and `ESTOP CLEAR <side>` lines. The patched runner replaces the pinned v14 sha256 in Lite's manifest and passes the fake-SDK suite.
- **Guided recovery (wizard, per arm or both).**
  1. Confirm the cell is safe.
  2. Release the button; the chip goes to RECOVERING, never straight to CLEAR.
  3. Show the controller state in plain words (errors, enable, brake, power, run mode, pole, hand).
  4. Clear errors with `rm_clear_system_err` and `rm_set_joint_clear_err`, at most two rounds, reading back each time; a return of 0 is not trusted (`DP/program.py:236-240`).
  5. Re-enable joints with `rm_set_joint_en_state` with the wrist supported (joints 5–7 have no brake inside, HW p.34), or give the pendant or power-cycle instructions from ALIX's ladder (`TEST_PROCEDURE.md:240-242`).
  6. Restore the global speed.
  7. Collision-check the move from the current pose to the program's first point against the `commode_c` scene, the bin, the poles and the venue walls, and show its size; it needs an explicit confirmation.
  8. Re-home the pole if it stopped mid-stroke.

  An interrupted trajectory cannot be resumed (JSON p.61). Lite never auto-resumes; the operator restarts the region or a sequence step on purpose.

#### 9.3.2 Lite's STOP: a secondary controlled stop

- **Visibility.** STOP LEFT, STOP RIGHT and STOP ALL, labelled "STOP (software)", are visible whenever an arm is connected. They are never behind a dialog, never queued, and bypass every lock.
- **What it sends.** `rm_set_arm_slow_stop` on the moving arm or arms (or the verb proven by the G4 test), plus pole stop (`rm_set_lift_speed(0)`; the software arm stops do not stop the pole, which kept travelling for up to 2.6 s, `DP/peripherals.py:189-199`), hand freeze and sequence cancel. `rm_set_arm_emergency_stop` is not used: it is Gen-4 only and returns −4 on these Gen-3 controllers (`rm_robot_interface.py:7226-7240`).
- **Release 1 (runner v14).**
  1. STOP sends one signal to the launch wrapper.
  2. The wrapper raises one interrupt in the runner, which runs its existing slow stop and speed restore.
  3. STOP ALL also stops the pole and freezes the hand.
  4. If the stop is not confirmed within about 8 s, Lite ends the runner and runs the verified stop ladder (slow stop → pause → stop) on a fresh connection.
  5. If that fails too, the screen tells the operator to press the hardware E-stop.
- **If Lite itself fails.** If the Lite backend dies, the runner's input closes and the runner slow-stops. A browser heartbeat lets the backend stop moving arms when the UI disappears (exact policy set from the P0 measurements). The IO E-stop works without the PC (drill DR12).
- **Release 2 (agents).** Each agent runs the stop ladder itself when the web process is silent for 2 s while its arm moves.
- **Labelling.** Lite's STOP is labelled a *software, controlled slow stop*: "Stops the arm gently. In an emergency press the red E-stop button on the robot." **The IO E-stop is the safety stop.**

### 9.4 Motion gateway and interlocks

- Arming is per arm and per session, with idle disarm and a persistent red REAL banner.
- Before any motion, Lite checks:
  - both E-stop chips read CLEAR;
  - the run mode by read-back;
  - no blocking error code;
  - no program running;
  - arm identity;
  - the process interlock;
  - the slot match (name and size read back from the controller against Lite's ledger);
  - the required ladder rungs.
- While any arm is in drag mode, Lite sends no other robot command.
- Only one browser session holds control at a time.
- **Uploads.** Any program ID 1–100, as in the control program (`UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py:11, 930-950, 4279, 8771`). Before each write Lite reads the program list; for an occupied slot it shows the current name and size and asks Replace / Skip this file / Cancel the rest of the batch. Batches take per-file IDs and refuse duplicate IDs within one batch. The ledger records every write (file sha256, slot, arm, time, person) and drives soft warnings, for example "this slot holds a program from the V6 workflow" or "this slot was not written by Lite"; none of them blocks. Background pre-staging never replaces a program without a confirmation.

### 9.5 Speeds and first runs

- **Transit.** The approach to a program's first point runs at 30 % by default. Lite checks it from the live pose, walls included, and a large transit needs an explicit confirmation.
- **First runs.** New or changed content runs first from a slowed copy (an engineer setting, on by default). Right-arm TOPP-RA files get a supervised first run (§8.3).
- **Global speed.** Restored on every exit path and verified.
- **Speed-ups and target times.** Both optimisers (the default and TOPP-RA) stay within L1–L6 on the written bytes: G1 margin, band-limited speed at most 0.97 × the ratings, acceleration at most 600 °/s² as a 50 ms windowed mean, at most 10° per line, path change within budget, ends at the recorded poses. The operator's target time never overrides a limit; when it cannot be met, Lite delivers the shortest legal time and names the binding joint, quantity and time window.

### 9.6 Pole, hand and bin

- **Pole.** It moves only when:
  - both E-stop chips read CLEAR;
  - the arm is still (below 1.6 °/s);
  - the arm is not in drag mode;
  - the hand holds neither the fixture nor a tool mid-task;
  - the pad is off every surface.

  On an E-stop or STOP, Lite sends `rm_set_lift_speed(0)`. Whether the IO E-stop stops a moving pole by itself is measured in drill DR6.
- **Stowed bin.** With the bin stowed, the 153–282 mm band is checked over the whole travel. The band is provisional until it is re-measured on the installed bin (D6 in §11). The bin geometry comes from the URDF (`ALIX/core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf:188-359`); the bin state is declared once per session.
- **Hand.** Hand moves need a confirmation and are read back. Thumb-held presets are preferred with a glove. The IO E-stop does not stop the hand, so Lite freezes it on every E-stop (drill DR7).
- **Dry runs.** Controller SIM mode does not simulate the lift or the hand, so dry runs skip pole and hand steps and never count as a test of R7.

### 9.7 Network and remote access

- Lite binds to 127.0.0.1 only, uses a per-launch token header and checks Host and Origin. There is no CORS.
- **Jetson in production (decision D1).** Lite runs on the operator PC now and on the robot's Jetson AGX Orin in production. If the Jetson has its own screen, the loopback rules above apply unchanged. If it is headless (decision D22), Lite needs an authenticated remote mode, for example an SSH tunnel or TLS with a login, with a bind address limited to the robot network and the same Host, Origin and token checks.
- Only the Lite host talks to the controllers on the robot network.
- The audit log records every motion, confirmation, stop, E-stop event, overwrite and override, with the file sha256 and the controller IP.

---
## 10. Risks and mitigations

| # | Risk | Likelihood | Impact | Mitigation | When |
|---|---|---|---|---|---|
| 1 | Lite's software STOP (slow stop) does not end a running stored program (G4 fails) | Medium | Medium (the IO E-stop is the safety stop) | Software STOP test in P0 (drill DR14); use the verb that is proven; fallback stop ladder; IO E-stop primary; REAL only after commissioning; keep programs short until resolved | P0 |
| 2 | Wrong cubicle selected, or the real cubicle differs from the model (width or back-wall distance), giving false wall hits (28 of 188 rows with the 800 mm walls in the wider cell, C3 §2.5) or missed wall hits | Medium | High | 1400 mm cubicle by default, 800 mm selectable (decision D21), named in every verdict; verified once by replaying 3–5 known-good recordings; "venue not verified" and NOT CHECKED until then; slowed first runs; re-verify when the robot moves to another cubicle | P0, P6 |
| 3 | aarch64 packaging for the Jetson (production target, decision D1): toppra 0.6.10 has no aarch64 wheel and no source package on PyPI (newest sdist 0.6.3, checked 13 Oct); `python-fcl` ships wheels only and the dual-arm sweep has no non-FCL fallback (`ALIX/SETUP.md:469-474`; `ALIX/DEPENDENCIES.md:35-70`); snap Chromium may block the app window | Medium | Medium | Build an aarch64 wheelhouse early in P1: toppra compiled from its 0.6.10 git tag (Cython, gcc, numpy headers); check every pinned wheel, including `python-fcl`, for the Jetson's Python; gate TOPP-RA on a guarded import so Lite works without it; time one solve on the Orin (1.7–1.8 s per solve on the operator PC); run the test suite on both architectures; default-browser fallback | P1 |
| 4 | ALIX code changes break imports or parity (73 commits in 14 days on the Studio branch) | High | Medium | One ALIX commit vendored into `alix_studio_lite/vendor/` per release (recorded in `vendor/MANIFEST.json`); parity tests, including TOPP-RA against FILE_01.1 (3839 lines); nightly canary; upstream seams | Continuous |
| 5 | Upstream seams declined | Medium | Low–Medium | Ports with golden parity tests (size S) | P0 |
| 6 | Offset solver research risk: exact offsets are impossible near straight-elbow passages | Medium | Medium | Engineer-only tool; honest reduced-span reports; scheduled late (P10) | P10 |
| 7 | Drag-teach record period uncertain (5 ms or 10 ms) | Medium | Medium | Measure in P0 and per recording; store the measured period in the sidecar | P0, P4 |
| 8 | Region catalogue and cleaning times not signed off, which blocks teaching | Medium | Medium | Draft catalogue already built from the deck, which describes commode C (Appendix A of the requirements); sign-off before P2 (D3, D5); the catalogue time is the default target time (D4 decided) | P0–P2 |
| 9 | Dual-arm drift (no row lock between controllers; pole timing model wrong) | High | High | Drift-window checks; SERIAL proposed below 25 mm clearance; start skew measured; TOGETHER blocks get one line count for both arms and a re-run of the dual-arm timeline; v53 stays the dual executor in Release 1 | P9 |
| 10 | Uploads stay slow for long programs (controller-bound) | High | Low | Fewer points through optimisation (TOPP-RA gives 3.2–4.0× fewer lines, `ALIX/docs/motion/TAUGHT_CANFD_PIPELINE.md:42-55, 588-591`); skip unchanged; parallel arms; background queue; CANFD lane later | P7, P8 |
| 11 | Operators find Lite too complex | Medium | High | Operator-first design; pilots at P2 and P4; formal usability test at P7; field trial | P1–P11 |
| 12 | Lab time on the robot is scarce, and the agent-driven code build moves faster than the robot sessions | High | Medium | Book sessions per phase; emulator and fake SDK in CI; batch the hardware checks (E-stop drill, STOP test, venue check, upload timing) into planned lab days | All |
| 13 | The RealMan SDK licence prevents bundling | Low | Low | Settings locates the SDK once (D20); the SDK ships both `linux_x86` and `linux_arm` libraries | P1 |
| 14 | v53 stays in service with known defects (Multi Run `TypeError`, no IP entry) | Medium | Low | One-line fix by its owner (D16); Lite's process interlock refuses commands while v53 runs | P0 |
| 15 | Accidental overwrite of a production or V6 program, because Lite can write any slot 1–100 (as the control program does, D2) | Medium | High | Confirmation before replacing an occupied slot, showing the current program's name and size; per-file IDs and no duplicate IDs in a batch; upload ledger; soft warnings for slots Lite did not write and for ranges others use (ALIX 70–100, V6 71–83); name and size read-back before every run; every overwrite in the audit log | P7 |
| 16 | Knowledge of the ALIX libraries sits with one person | Medium | Medium | Seams, documentation and parity tests; pair reviews with the ALIX owner | Continuous |
| 17 | TOPP-RA moves the pad off the taught path: the chain smooths in pad arc length (30 mm) before solving, and ALIX's G4 reads 0.0 mm on its TOPP-RA slots because it compares the derived file with itself (`TAUGHT_CANFD_PIPELINE.md:320-324, 547-550`); executed pad was 2–3 mm p95 from the taught path on bowl-out (`:42-55`) | Medium | Medium | L5 path-change check against the **raw** recording (at most 10 mm or the region's MAX DEV); 15 mm arc-length smoothing for regions with a MAX DEV; L5 at the region's MAX DEV, G5 a warning (D11); slowed first runs; the default optimiser stays available | P5 |
| 18 | TOPP-RA output sits on the acceleration limit almost everywhere: it reads 1.007 × 600 °/s² as a 50 ms windowed mean but 1.18–1.22 × on ALIX's 20 Hz Butterworth reading (filter overshoot, measured 13 Oct) | Medium | Medium | Decision D23 on which reading Lite enforces; post low-pass off by default; hardware first runs watched for controller warnings | P0, P5 |
| 19 | E-stop integration: the IO E-stop's stop behaviour is undocumented (no stop category, rating or reaction time; RealMan advises safety interfaces over normal IO, HW p.32); ALIX code assumes it cuts joint power, the site says it does not; runner v14 would very likely report an E-stop as "RUN COMPLETE" and continue a sequence; a HIGH-trigger input may not trip on a broken wire | Medium | High | Drill DR1–DR14 per arm and both before REAL use (DR1, DR3, DR9, DR10, DR11 must pass); E-stop chip per arm from the mode-5 IO, fail closed on UNKNOWN; small runner v14 patch; pole stop and hand freeze from Lite; wiring review with the electrician (drill DR13); Lite never writes IO modes | P0, P3 |

---
## 11. Open decisions for the requester and the team

Decisions taken on 13 Oct 2026 are marked **Decided**; the row keeps the decision and what follows from it.

| ID | Decision | Options | Recommendation or outcome | Needed by | Owner |
|---|---|---|---|---|---|
| D1 | Target computer | Operator PC on the robot network (x86-64, Ubuntu 24.04) / robot's Jetson AGX Orin (aarch64, Ubuntu 24.04) | **Decided:** operator PC now; the Jetson in production. Both are supported; an aarch64 production build is required (§10 risk 3) | — | Requester |
| D2 | Program IDs Lite may write | A reserved range / any ID | **Decided:** any program ID 1–100, as in the control program (UI v53 + runner v14). An occupied slot is replaced after a confirmation that shows the current program; per-file IDs in a batch; run by ID or name. Ledger and soft warnings (for example "this slot holds a program from the V6 workflow"); no range refusal | — | Requester |
| D3 | Region catalogue | IDs, deck fixes, teaching order, first-release units | **Decided (codes and scope, 13 Oct):** fixture commode C; codes in the deck's own format (`L-R04`, `R-T03`, parts as `PART 1`) with the seven changes from the deck agreed; first release = 34 units (26 cleaning regions and 8 lid/seat operations, opening and closing, which are not cleaning tasks); glove and bin tasks excluded for now, and the drain hole with them. Still open: target times, pole and hand values, and the teaching order (requirements Appendix A; `docs/region_catalogue_commode_c.csv`) | Before P2 (values) | Requester |
| D4 | "Specified cleaning time" | Teach countdown / target run time after optimisation / both | **Decided:** the operator's time for a region is the optimiser's target duration (default: the catalogue time, which the operator can override). The optimiser meets it, or returns the closest legal (minimum) time and names the binding limit. During teaching the same time drives a countdown as guidance only. Times are in seconds at 5 ms per line (D26) | — | Requester |
| D5 | Pole and hand defaults per region | V6 values / ALIX commode C sheet; GRIP / thumb-held open | Seed from V6 with the ALIX values alongside; confirm per region | Before P4 | Requester |
| D6 | Bin state and physical bin | Stowed / extended / lowered / not fitted; is BinShort fitted? | Bin geometry from the URDF (`ALIX/core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf:188-359`); declare the state per session; re-measure the stowed foul band on the installed bin | P0 | ALIX hardware owner |
| D7 | Cell model | A measured cell profile / the URDF and `commode_c` scene | **Decided:** no calibration project. The URDF and the `commode_c` scene already hold the commode pose and articulation, bin, UGV and poles. Walls, the one part not enforced today, are enforced from the selected venue file (D21) and verified with known-good recordings | — | Requester |
| D8 | Contact list for the GRIP hand | ALIX `glove_frame_2` list / extended with finger links | Rule per hand preset from the P6 evidence | Before P6 | Requester |
| D9 | Collision-detection level | Same level on both arms (left 3, right 0 at last check) | Align both arms before any real test | P0 | ALIX hardware owner |
| D10 | Meaning of "low spikes" in R12 | Dips and hesitations / negative peaks | **Decided (13 Oct 2026):** Low spikes are sudden downward jumps of a joint curve or its speed (▼), the mirror of high spikes (▲); both get a "smooth this spot" suggestion. Hesitations are not spikes; they stay an information note (R12 (f)) | Before P5 | Requester |
| D11 | Gate strictness | G5 blocking / warning; executed budget 20 mm default or 5–10 mm for delicate regions | **Decided (13 Oct 2026):** G5 is a warning only: ALIX's model reads 33–109 mm on the raw commode C recordings, which ran fine on the robot, so it cannot gate a file. A region's MAX DEV tightens the path-accuracy limit L5 (the pad path against the raw recording), with no override. The executed-path budget per region stays in the catalogue for the warning | Before P5 | Requester |
| D12 | When a dry run is required | Every changed fingerprint / before upload only | Every changed fingerprint (ALIX ladder rule); engineer override for a missing one | Before P7 | Requester |
| D13 | Executor policy | Runner v14 first, then agents / agents from the start | Runner v14 (Stage A, with the small E-stop patch) in Release 1; agents after the switch criteria | P0 | Team |
| D14 | Upstream seams | Accept / decline S1–S7 | **Optional now.** With the vendored copy, Lite applies small patches itself and records them in `vendor/PATCHES.md` (one so far: a lazy `alix_workspace/__init__`). Upstream seams keep the copy close to ALIX and shrink the patch list | When convenient | ALIX owner |
| D15 | Code location | Monorepo package / separate folder | **Decided:** the folder `alix_studio_lite` in this workspace, with the documents in `alix_studio_lite/docs`. It is an independent workspace: everything Lite needs lives inside the folder. The reused ALIX code and data are vendored into `vendor/alix` by `tools/vendor_from_alix.py`, with the ALIX commit and every file's sha256 in `vendor/MANIFEST.json`; no ALIX checkout is used at run time | — | Requester |
| D16 | v53 defects | Fix / leave | One-line `@staticmethod` fix by its owner while v53 remains the dual executor | P0 | Control-app owner |
| D17 | Data location | `~/ALIX-Lite` / other; copy into ALIX's taught lane? | `~/ALIX-Lite`, with export to the ALIX lane on demand | P1 | Team |
| D18 | Legacy V6 on the dashboard | Show as LEGACY / hide | Show as LEGACY; never uploadable until re-certified | P2 | Requester |
| D19 | Operators and controls for teaching | One or two people; USB foot switch | One operator by default; foot switch recommended | P4 | Requester |
| D20 | RealMan SDK in the bundle | Bundle / locate once | Bundle if the licence allows; otherwise Settings locates it | P1 | Requester |
| D21 | Wall model for the test cell | Whole-washroom venue / single cubicle; width | **Decided:** a single cubicle, not the whole washroom. 1400 mm wide by default, with an option to switch to 800 mm. ALIX has an 800 mm cubicle (`washroom_800mm_wide_scene.yaml` + `washroom_800mm_planning_scene.stl`) but its 1400 mm venue is the whole washroom (`washroom_1400mm_wide.stl`, 4.3 × 8.1 m), so Lite builds a 1400 mm cubicle the same way (inner side faces at ±0.70 m). **Built 13 Oct:** `assets/scene/cubicles/cubicle_1400mm.stl` and `.yaml` (`tools/make_cubicle.py`; the same rule reproduces ALIX's 1000 mm cubicle exactly). Confirm the cubicle's width and back-wall distance once on site, and verify with 3–5 known-good recordings | — | Requester |
| D22 | Does the production Jetson have its own screen? | Screen and keyboard on the robot / headless, browser on another machine | If headless, plan the authenticated remote mode (§9.7) and the service start-up; if it has a screen, check 3D view speed in its browser | P1 | Requester, ALIX hardware owner |
| D23 | Acceleration limit used by TOPP-RA and the L3 check | 600 °/s² hard / softer | **Decided:** 600 °/s² is the arm's hardware limit. It is hard in Lite for both optimisers (50 ms windowed mean, × 1.02 tolerance) with no override; the 20 Hz Butterworth reading is shown as advisory | — | Requester |
| D24 | Who may choose the optimiser | Operators and engineers / engineers only | **Decided:** operators and engineers can both switch a region between the Default optimiser and the Time-optimal (TOPP-RA) optimiser; Default is preselected | — | Requester |
| D25 | E-stop input | Which IO, and how the state behaves | **Decided:** IO3 on each controller, in E-stop mode (mode 5). The E-stop state holds from the press until the button is released (the manual reset); Lite's chip does the same, then shows RECOVERING until recovery is done. Lite blocks motion if IO3 is not in mode 5 (RealMan's demo maps IO3 to "continue program") | — | Requester |
| D26 | Clock | Measured playback time / 5 ms per line | **Decided:** Clock: 5 ms per line is the only time base; durations and targets are (lines − 1) × 5 ms (13 Oct 2026). One line of a track is one 5 ms controller cycle, operator target times are in those seconds, and every limit is judged at 5 ms | — | Project owner |
| D27 | Delivery order | Backend first / UI/UX first | **Decided:** Order: UI/UX first; backend on hold at its checkpoint (13 Oct 2026). The first milestone is the operator workflow screens on the ALIX Studio theme, working against the existing optimiser and stand-ins for the robot; the backend items follow (§7.1, §7.7). The checkpoint is in [`docs/STATUS.md`](STATUS.md) | — | Project owner |
| D28 | Dual-arm timing margin | Drop it under D26 / keep it | **Decided:** Kept (13 Oct 2026). Arms running together never finish in perfect step, so the TOGETHER clearance check keeps the run-to-run timing spread measured by ALIX (R5, R8, R10) | — | Project owner |
| D29 | Fixture pool | Commode C only / every ALIX fixture | **Decided (13 Oct 2026):** choose the fixture from the pool: commodes A, B and C, urinal A and wash basin A, with commode C the default. Each fixture is placed with ALIX's own `scene_pose` (the venue's yaw 180 included) and has its own project. Commode C uses the signed-off deck catalogue; the others start with a provisional catalogue from ALIX's `regions.yaml` until a regions deck exists for them | — | Project owner |
| D30 | Modes | Simulation / dry run / real | **Decided (13 Oct 2026):** three modes. **Visual sim:** the robot moves in the 3D view, no hardware. **Hardware sim:** the controller's own SIM mode (`rm_set_arm_run_mode(0)`, read back with `rm_get_arm_run_mode`; runner v14 `--set-arm-mode simulation`), programs run and the arms stay still; **nothing moves physically** (clarified 13 Oct): the pole and the hand are emulated, never commanded, as R4 (b) states. **Real:** the arms, the pole and the hand move, after a confirmation | — | Project owner |
| D31 | Existing recordings | Teach every region live / use existing taught files | **Decided (13 Oct 2026):** when a taught `.txt` exists for a region (the fixture's recordings library, `assets/recordings/<fixture>.yaml`), Lite uses it by default; re-teaching live is an option, and any recording can be picked from the library or uploaded from the computer | — | Project owner |
| D32 | Layout | Left rail / tabs at the top | **Decided (13 Oct 2026):** tabs at the top, as in ALIX Studio: Regions · Teach · Optimise · 3D view · Run (+ Robot and Settings for engineers, and Help), with the current region chosen at the left of the tab bar. Every screen reflows with the window size; nothing scrolls sideways | — | Project owner |

---
## 12. First two weeks

The code is written with an AI coding agent (Claude Opus 5.5) under engineer review, so the build is not the pacing item; robot lab time, sign-offs and the E-stop drill are. §7 carries the agent-driven schedule, which starts with the UI/UX work (D27). The weeks below are the calendar frame for it: in §7.6 the aarch64 build comes with step 2, and the P0 lab day and the runner E-stop patch with step 4, so they fall in week 2 as listed here. Where the robot or the Orin cannot be booked then, every gate behind them in §7.6 moves by the same amount.

**Week 1**

1. **Start with the UI/UX work in `alix_studio_lite`** (`LITE/alix_studio_lite`, documents in `alix_studio_lite/docs`; decision D27), using the vendored ALIX copy in `alix_studio_lite/vendor/`:
   - package skeleton and CI on x86-64, with the scope guards;
   - the operator workflow screens on the ALIX Studio theme, working against the existing optimiser and stand-ins for the robot (the fake recorder, and the fake-SDK harness with simulated E-stop IO for the pinned runner);
   - then the backend from its checkpoint ([`docs/STATUS.md`](STATUS.md)), in the order of §7.6: the optimiser golden tests (§8.1.1), starting with TOPP-RA parity on FILE_01.1 (3839 lines); the 3D check with the `commode_c` scene and the walls; robot I/O with the IO3 E-stop monitor; the server and API with token parity. Together they give the core loop: import or teach a region → optimise to the operator's target time (the default optimiser and TOPP-RA behind one interface, with the closest-feasible report and the L1–L6 checks) → 3D check → upload to any slot 1–100 with the overwrite confirmation → dry run.
2. **Wireframes and pilot.** Review Dashboard, Teach card, Optimise (target time, closest-feasible chip) and Test on robot, including the E-stop chips and the recovery wizard, with the requester and one operator.
3. **Decision meeting** with the requester and the ALIX owner:
   - record the decisions already taken: D1, D2, D4, D7 and D15;
   - decide D13 and D14; decide D22 (Jetson screen), or set a date (D21 and D23–D27 were decided on 13 Oct);
   - hand over Appendix A of the requirements for sign-off (D3, now known to describe commode C).
4. **Cubicle and wall check.** Build the 1400 mm cubicle mesh (D21) and confirm the cubicle's width and back-wall distance on site. Replay 3–5 known-good recordings through the 3D check with it: no wall or structural-link contact. Confirm that the 800 mm cubicle reproduces the 28 false wall rows (C3 §2.5), which proves the wall checker is active. No cell calibration is needed: the rest of the cell comes from the URDF and the `commode_c` scene.
5. **Book a supervised lab day on both arms** and prepare the scripts:
   - **IO E-stop drill** DR1–DR14 per arm and with both arms (§8.3), with the electrician for the open-circuit check (DR13); confirm that IO3 is in mode 5 on each controller (D25) and record its active level;
   - **software STOP test** (G4, drill DR14): slow stop and stop on a running stored program, killed-runner behaviour and fresh-connection stop;
   - SIM-mode pole and hand behaviour;
   - upload timing (serial and parallel), overwrite of an occupied slot, and dual start skew;
   - J4 drive limits, collision-detection level, drag record period, bin measurements.
6. **Keep data collection moving.** Keep teaching the untaught regions in v53, with a teach sheet per recording (region ID, arm, pole, hand preset, bin state, operator, date), for import into Lite.
7. **Upstream seams.** Raise the change requests for S1, S2, S3 and S5.

**Week 2**

1. **Run the lab day** and write the characterisation report, including the drill results in the site config. DR1, DR3, DR9, DR10 and DR11 must pass before operators use Lite in REAL mode. The results are design inputs (for example the STOP verb and the E-stop chip's active level).
2. **Runner v14 E-stop patch.** Apply the small patch (§9.3.1), pin its sha256 and pass the fake-SDK suite; repeat drill items DR3 and DR10 on the robot with it.
3. **aarch64 build for the Jetson.** Build the wheelhouse with toppra 0.6.10 compiled from its git tag and check every pinned wheel, including `python-fcl`; run the test suite on the Orin and time one TOPP-RA solve. Confirm Lite runs with TOPP-RA disabled if the build fails.
4. **Vendored copy and self-test.** Done 13 Oct: the vendored ALIX copy and its tests (imports only from `vendor/`, manifest checksums, every mesh resolves, the commode C world builds with both cubicles, TOPP-RA reproduces the reference). Still to add: a start-up self-test that fails closed on a missing scene, mesh, SRDF or cubicle file.
5. **Overlay.** Re-bake the visual overlay (BinShort and wrist camera) with model tests.

P0 ends with milestone **M5 Ground truth** (§7.7): the characterisation and drill report, the verified venue, the V6 pair in the unchanged Studio, and the bundle self-test failing closed on a clean machine.
