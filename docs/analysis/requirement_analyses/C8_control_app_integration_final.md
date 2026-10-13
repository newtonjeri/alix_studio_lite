# C8 - Control application integration (INT-CTRL) - verified final

**Readers.** Engineers who will build ALIX Studio Lite, and the requester.

**Scope.** The upload-channel control application: the Tkinter UI (front end), the runner it starts (back end), and the helper files. The entry covers what each program does, which version is current, and how Lite should take it over.

**Depth.** The user asked for a **high-level** reading of these programs. This entry is not a bug audit. It lists only problems that block the plan, one line each (section 4).

**How it was checked.** I re-opened every claim in the analyst's draft at the cited lines. All checks were read-only: `sed -n`, `grep`, `md5sum`, `diff`, `comm`, and `python3 -I` on pure functions. I also re-ran the dual-arm planners from v48, v51, v52 and v53 on the V6 production JSONs. No robot, network or Tk was used.

**Verdict on the draft.** The core conclusions hold:
- UI v53 + runner v14 is the current pair.
- v53 contains all of v48.
- ALIX Studio has no execution channel.
- Wrapping the unchanged runner in a Lite backend is the right first step.

I made 3 factual corrections, added 6 items the draft missed (5 safety, 1 test approach), and changed 5 points of design or effort. All are listed in section 13.

| Key | Path |
|---|---|
| UC | `/home/newtonjeri/alix_studio_lite_ws/upload_channel/` |
| PP | `UC/py programs/` (the folder name contains a space) |
| v53 | `UC/rm_trajectory_ui_v53_safe_lid_checkpoint.py` (11,581 lines) |
| v48 | `UC/rm_trajectory_ui_v48_handoff_continue_fix.py` (11,068 lines; md5 identical to `PP/...v48...`) |
| r14 | `UC/rm_upload_fast_v14_safe_transit.py` (2,702 lines) |
| OPS | `/home/newtonjeri/alix_studio_lite_ws/RealMan Dual-Arm Operations UI.txt` |
| V6 | `/home/newtonjeri/alix_studio_lite_ws/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/` |
| ALIX | `/home/newtonjeri/alix_ws/src/alix/` |
| TG | `ALIX/core/python/alix_taskgen/` (ALIX Studio) |
| DP | `ALIX/core/python/alix_dispatch/src/alix_dispatch/` |

---

## 1. The answer in brief

**Original text.** "And the Control Application, latest of the ui and the server program, confirm whether this is the most comprehensive of all .../upload_channel/py programs/rm_trajectory_ui_v48_handoff_continue_fix.py". This comes from the user's brief, not from the 14 numbered items in the requirement inputs file. It goes with the operations notes in OPS, which say "Current version: UI v50 / Runner v13" (OPS:222-224).

**v48 is neither the latest nor the most comprehensive version.**
- **The current pair** is UI **v53** (`rm_trajectory_ui_v53_safe_lid_checkpoint.py`) plus runner **v14** (`rm_upload_fast_v14_safe_transit.py`). Both sit in the `upload_channel` root, not in `py programs/`.
- **v53 declares this pairing.** It sets `UI_PROGRAM_VERSION = "v53"` and `EXPECTED_RUNNER_VERSION = "v14"` (v53:45-46) and picks the v14 runner first (v53:6890-6891).
- **v48 pairs with the older runner.** It picks runner v13 first (v48:6853-6854).
- **v53 contains everything v48 has.** I compared the sets of function names, button labels and runner flags (`comm`). Only one inner helper (`unique_map`) and two "PROGRAM RULES" labels are gone. No runner flag was removed.

**Decisive test: the current production recipe.** I ran each UI's pure planner on the V6 FINAL pair (`UC/LEFT_V6_70SERIES_FINAL.json` and `UC/RIGHT_V6_70SERIES_FINAL.json`):

| UI | Result |
|---|---|
| v48 | **Rejected**: "PROGRAM RULES matches LEFT and RIGHT by Program ID. Missing on RIGHT: 73 Missing on LEFT: 74, 75" (v48:9576) |
| v51 | **Rejected** (same message) |
| v52 | Accepted, **4 stages**: `TIMELINE-1 L[71,73,76] R[71,74,75,76]`, then serial 78, serial 79, then `TIMELINE-2 [80,83]`. **There is no lid pause.** The 2 s WAIT is only a wait, so LEFT can start 76 while RIGHT is still in 74/75 |
| **v53** | Accepted, **7 stages**: `TIMELINE-1 L[71,73] R[71,74,75]`, then `LID-PREP-1`, then `LID-CHECK-1` (a manual lid prompt), then `TIMELINE-2 [76]`, serial 78 (LEFT first), serial 79 (LEFT first), and `TIMELINE-3 [80,83]` |

The V6 package's own README records the v51 limitation (V6/README.txt:69-80).

**The operations notes (OPS) are out of date.**
- They name UI v50 / Runner v13, and no v49 or v50 file exists.
- They describe the old TOGETHER rule, "matching Program IDs" (OPS:115-123).
- They omit Start %, timeline blocks and the lid checkpoint.
- They list Multi Run and Load Sequence (OPS:21-22), which fail in both v48 and v53.
- Otherwise, all 24 sections exist in v53/v14 (map 12: 136 of 158 bullets present, 20 different, 1 broken, 1 wrong).

**What Lite should do.**
1. Take v53 + v14 as the functional baseline.
2. **Phase 1 (Must):** run runner v14 unchanged as a subprocess behind a Lite backend.
3. **Phase 2 (Should):** move to a library built on `alix_dispatch`.
4. Rebuild the screens in the ALIX Studio web shell, split into an **operator** area and an **engineer** area.
5. Keep the `realman_workflow_sequence_v1` files and the v52/v53 dual-arm rules exactly.
6. **First, spend about an hour on the robot.** Measure whether a slow stop aborts a running stored program, and what pole and hand commands do in the controller's SIM mode (sections 4 and 7).

---

## 2. What the programs do (high level)

| File | Role | What it does | Status for Lite |
|---|---|---|---|
| `rm_trajectory_ui_v53_safe_lid_checkpoint.py` | **Front end**: a Tkinter desktop app, "RealMan // Dual-Arm Operations Console" (v53:6876-6882) | **Page 01** has two side-by-side arm panels. Each has the tabs RUN, POINTS, DRAG TEACH, HAND / POLE and TELEMETRY (v53:2149-2166), plus tools that act on both arms. **Page 02** has a recipe builder per arm and the button "RUN BOTH // SAFE TIMELINE". **It never talks to the robot itself**: every button builds `python -u <runner> --sdk <SDK> ... --side left\|right --yes` and starts it (v53:4255-4262, 5733-5760) | **Current UI.** Use it as the behaviour reference. Port its planner; rebuild the screens on the web |
| `rm_upload_fast_v14_safe_transit.py` | **Back end** (the "server program"). It is a command-line program, not a network server | One process opens one TCP connection to one controller, does one job and exits; 0 means done. Jobs: upload and verify a `.txt`; list the slots; run a slot or a sequence; run a Page-02 workflow over one connection; drag teach (record, save, playback); move the hand and the pole; read or set SIM/REAL mode; global points (r14:1-41, 2278-2346, 2606-2698) | **Current runner.** Phase 1 runs it unchanged |
| `..._v14_safe_transit_v52support.py`, `..._v53support.py` | Copies | Byte-identical to r14 (md5 `a0b285e6...`). The UI does not look for these names | Ignore |
| `rm_upload_fast_v13_overall_speed.py` | Older runner | Partner of v48. It reads the legacy `speed` field as the approach speed (r13:1679) | Never pair it with v51+ files (section 4) |
| `hand.py` (126 lines) | Helper | Drives the Inspire RH56 hand over end-port Modbus RTU: 6 channels, 0-1000, 1000 = open (hand.py:3-17). It is a trimmed copy of ALIX's `DP/peripherals.py` HandModbus, with the same registers and stroke law | Use unchanged; the runner loads it from its own folder (r14:365-390) |
| `pole.py` (58 lines) | Helper | Moves the lift and waits for arrival, using the measured slew, a 5 mm tolerance and a timeout (pole.py:3-58). A trimmed copy of `DP/peripherals.py` Pole | Use unchanged |
| `usage.py` (215 lines) | Standalone CLI | Pole and hand by hand. Its rules: "the pole moves only with the arm still; never move pole or hand with the pad on a surface" (usage.py:25). It has an `arm_at_rest` check (usage.py:44-60) and `stop_all` (usage.py:141-150). Neither the UI nor the runner uses it | Reference |
| `README_COMMANDS.md` | Doc | How the original uploader is used on the robot PC, which is Linux (user `alai-labs`, README_COMMANDS.md:3, 44) | Reference |
| `LEFT/RIGHT_V6_70SERIES_FINAL.json` | Data | The production dual-arm recipe in v1 format: pole and hand set-up, Programs 71-83 with dual rules, and a 2 s WAIT noted "MANUAL LID COVER OPEN" | Golden test data |
| `rm_trajectory_ui_v48/v51/v52_*.py` | Older UIs | v51 added Start %, the version bar and live status. v52 added timeline rules | Superseded |
| `upload/`, `upload.zip`, `py programs/` | History | `upload/rm_upload.py` is ALIX `tools/rm_upload.py`; only usage lines 4-7 differ. `py programs/` holds **28** UIs (v8 to v48) and the runners v4-v12 | Reference only |

**How the pieces work together at run time.**

```
Operator -> UI v53 (Tk) --- for every action: python -u <runner> --sdk <SDK> --side left|right --yes <verb ...>
                |                         (one process = one TCP connection = one job; it imports hand.py / pole.py)
                v
          runner v14 --RM API2 SDK--> LEFT 192.168.1.10:8080   |   RIGHT 192.168.1.103:8080
UI <- printed progress lines + the exit code  (the UI's only status channel)
STOP       = SIGINT to the runner's process group -> KeyboardInterrupt -> rm_set_arm_slow_stop()
Dual start = two launches released by threading.Barrier(2) just before Popen (software sync only)
```

**Key facts (all verified).**

- **Status.** The UI learns what happened only from printed line prefixes and the exit code. The Page-02 handoff relies on `rc == 0` (v53:5968-5972, 10715-10717).
- **Progress lines nobody reads.** The runner prints `WORKFLOW STEP START i/n` and `WORKFLOW PROGRAM RUNNING/COMPLETE i/n` (r14:1867, 1900, 1974, 2142). v53 parses none of them (`grep -c WORKFLOW` on v53 = 0).
- **Connections.** "Persistent connection" lasts only for one runner process (r14:1986-1997). RUN BOTH starts one runner per arm per stage. The V6 recipe therefore makes 12 runner processes, and each stage that contains a Program re-reads the program list (r14:2012-2033).
- **Speeds.** Stored programs play at their own point timing. The run speed scales only the approach move to the first point, and it stays set as the controller's global ratio (DP/track.py:21-24). The defaults are:
  - Page-01 approach: 20 %.
  - Page-02 Start %: 30 (r14:1679-1699).
  - Overall: 100 %.
  - Restored to 50 % after every run (r14:2263-2277, 2335-2342).
- **Flags the UI never passes:** `--ip`, `--port`, `--run-speed`, `--restore-speed`, `--timeout` (flag census of v53).
- **Workflow file.** `realman_workflow_sequence_v1` is `{format, side, steps[]}`. The step types are program, pole (0-315 mm), hand (6 x 0-1000), wait (0-3600 s) and global. The runner checks `format` and each step, drops `dual_rule` and notes, and never checks the `side` field (r14:1668-1763). The dual rules exist only in the UI planner (v53:9619-10017).

---

## 3. Version history (short)

| Version | Added |
|---|---|
| `rm_upload.py` (ALIX) | Upload, verify, run, list for one arm |
| UI v8 | GUI wrapper around it |
| UI v13 | Batch upload, Multi Run |
| UI v22 | Barrier dual run |
| UI v25-26 | Drag teach; recording stored as a `DT_` program |
| UI v28 | Global points; SIM/REAL/CHECK |
| UI v39-40 | Six-channel hand, GRIP / ALL OPEN presets |
| UI v41 | Page 02 builder |
| UI v43 | Overall speed |
| UI v46 | Per-Program dual rules |
| UI v48 | Handoff decided by exit code |
| **v51** | Start % (approach only), version bar, live status; prefers runner v14 |
| **v52** | Timeline rules: TOGETHER rows form one block per arm, and IDs may differ |
| **v53** | Manual lid checkpoint |

The runners only ever added things. Their core, r14 lines 1-268, is the original uploader apart from two docstring lines (`diff`).

**Dropped before v48, missing in both v48 and v53.** These matter only if operators still want them:
- `DT_` save-to-controller and "RUN DRAG .TXT" (v26-v32);
- the dual-run REAL-mode warning (v32);
- the single "% open" hand slider (v18-v38);
- the Page-02 MODE selector (v44-v46). About 520 dead lines of it remain in v53.

---

## 4. Problems that block the plan (one line each)

1. **Stop gate (ALIX "G4", driver plan) has never been measured.** Nobody has tested whether `rm_set_arm_slow_stop` aborts a running stored program (DP/emulated_program.py:51-52; ALIX/docs/motion/DRIVER_EXECUTION_ORCHESTRATION_PLAN.md:46). ALIX's own rule is "Program replay is single-arm-only until G4(b) passes" (ALIX/docs/motion/DRIVER_TRACK_CHANNEL_ARCHITECTURE.md:203). Lite's STOP and its TOGETHER blocks depend on this answer. A ready probe exists: `track.probe_stop_policy` (DP/track.py:3273).
2. **SIM mode and the pole and hand.** The controller does not execute lift or hand commands in SIM (ALIX/docs/PHASE_PLAN.md:116), and ALIX skips them in its own SIM runs (DP/session.py:1-8, 276-310). Runner v14 still sends them, so a SIM dry run of the V6 recipe will probably time out at its first pole step (inferred from pole.py:46-58). Verify on hardware.
3. **No IP entry, and a wrong-arm risk.** The IPs in the UI are display-only (v53:7506, 7588, 7600). `--ip` overrides `--side` (r14:2347), and neither the runner nor the v53 loader checks a file's `side` (r14:1751-1763; v53:6819-6861).
4. **The runner version is not enforced.** The banner parses the filename only (v53:7390-7405). A v13 runner reads the `"speed": 100` that v51+ writes, so approach moves would run at 100 % (r13:1679, 1860).
5. **One arm's failure does not stop the other.** In a TOGETHER stage the other arm finishes its whole block (v53:10273-10300). Lite's backend must add cross-arm abort.
6. **Program-ID ranges collide.** V6 uses 71-83, and ALIX reserves 70-100 on the same controllers (DP/track.py:13-16; V6/README.txt:101-104).
7. **The SDK path must be set explicitly.** The UI default points at another machine (v53:48-51), and the runner's repository fallback does not resolve from `upload_channel` (r14:62).
8. **Tk app only: Multi Run and Page-01 Load Sequence raise TypeError.** Four helpers lack `self`/`@staticmethod` (v53:4853, 4876, 4885, 5000; called at 4932-4934, 5083, 5159, 5212). Lite does not port them, but this matters if v53 stays as the fallback.
9. **Windows only:** CTRL_BREAK is not caught by the runner, so it would die without a slow stop (inferred; v53:6058-6059). The robot PC is Linux, so ship Phase 1 for Linux unless Windows is required.

---

## 5. Borrow verdicts

### 5.1 ALIX Studio (alix_taskgen): Partial. Borrow the shell; build all control functions.

- **ALIX Studio cannot drive the robot.** It has no connect, upload, run or stop route, and a test pins this: "The studio has no execution channel ... arms not connected" (TG/test/test_studio_interaction_guards.py:2204-2211).
- `server.py` never imports `alix_dispatch` or calls `rm_create_robot_arm`. Its mentions of `alix_dispatch` are comments (TG/src/alix_taskgen/server.py:1802, 1835). The SDK is used only for offline vendor IK (TG/src/alix_taskgen/stroke_gate.py:3586-3620).
- **Not borrowable:** connect, upload, run, stop, drag teach, pole, hand, SIM/REAL and dual sequencing.

**Borrowable:**
1. **Server scaffold.** The stdlib `ThreadingHTTPServer` scaffold (server.py:30, 6402-6548), including its **localhost-only default** and its warning about unauthenticated binding (server.py:6539-6544).
2. **Theme tokens.** editor.css:22-44.
3. **Feedback components.**
   - `setStatus` / `showErrorToast` (editor.js:5431-5462).
   - `showStudioModal`, a confirmation card with chips (editor.js:13786-13832). It suits the REAL-run confirmation, the lid prompt and the handoff prompts.
4. **The Board table, as a visual pattern only.** Its actions call the excluded generator, e.g. "Generate missing" on the C-space pipeline (TG/.../web_editor/template.html:542-546).
5. **The joint-feed endpoint.** `POST /api/sim/joints` with `/api/sim/pose` (server.py:5326-5341; docstring "future: the real arms"). It needs a joint source, and runner v14 prints no joints. A live 3D mirror of the real arms therefore belongs to Phase 2 (a persistent session polling joints).

### 5.2 Other sources

| Source | Verdict | What to take |
|---|---|---|
| UI v53 | **Partial: port the logic, rebuild the UI** | The planner `_page2_program_chunks` + `_page2_build_program_rule_plan` (v53:9619-10017). It reads `self._page2_builder(side).steps`, so it runs outside Tk with a one-line shim; I re-ran it this way. Also take the checkpoint stage (v53:9870-9905, 10141-10188), the exit-code handoff with optional confirmation (v53:10376-10453; on by default, v53:6989 bound at 9104-9108), and the status words. Do not take the Tk widgets or the dark palette (v53:56-71) |
| Runner v14 + hand.py + pole.py | **Borrow as-is (Phase 1)** | Run as a subprocess exactly as v53 does, adding `--ip/--port` (r14:2283-2284, 2347) and an explicit `--sdk`. It keeps: store-only upload with CRLF and size read-back (r14:76-175), pre-run refusals (r14:189-205, 2011-2063), 4105 as a warning in sequences, global-speed restore (r14:255-262, 2263-2277), and mode read-back (r14:1229-1262) |
| `alix_dispatch` (ALIX, ROS-free) | **Borrow with adaptation**: Phase 2, plus one probe now | `connect.open_arm(side, ip, port)` with a 1500 ms reply timeout (DP/connect.py:141-182). `track.build_project_file / send_project / program_list` (DP/track.py:85-152). `global_speed_guard` (DP/track.py:162-200). `stop_program`, a verified escalation (slow stop, then pause, then stop; DP/track.py:3109). `probe_stop_policy`, the G4 probe (DP/track.py:3273). `wait_until_at_rest` and `at_rest_barrier` (DP/track.py:1949, 2960). `preflight_slots` (DP/track.py:353-392). `assert_side` (DP/track.py:341-350). `runmode` (DP/runmode.py:56-160). **`peripherals.Pole` / `HandModbus`**, ROS-free and run on hardware and on the emulator (DP/peripherals.py:1-51). **`staging.stage_arm`**: the hardware-proven pre-stroke order (retreat, hand open, at-rest interlock, pole, hand with read-back; refusals returned as values), which ran 12/12 twice (DP/staging.py:1-15, 358-380). `emulated_program` for tests without a robot. **Caveats:** there is no `pyproject` (tools add it to `sys.path`, e.g. ALIX/tools/cleaning_tracks.py:176). `peripherals` loads `hand_units.py` from the repository's ROS tree by path (DP/backends.py:31-47). There is **no drag-teach or global-point code** in alix_dispatch, so Phase 2 ports those from r14 |
| ALIX `tools/dual_arm_flow.py` | Reference | ALIX's own dual-arm executive over the same upload channel. A "concurrent" step is conditional on G4 and D1, and when either is unmet it is demoted to serial (one arm runs, the partner is parked and held) (dual_arm_flow.py:49-53). Use it as the model for Lite's TOGETHER gating |
| ALIX `alix_dispatch/test/test_rm_upload.py` | **Borrow with adaptation** | A fake controller drives the same uploader core through a patched `load_sdk` (test_rm_upload.py:1-60). Because r14 loads the SDK from `--sdk DIR` (r14:60-73), a fake `Robotic_Arm` package lets the **unchanged** runner run in CI with real output |
| usage.py | Reference | The safe order and the at-rest check. It needs `Robotic_Arm` on `PYTHONPATH` and does no connection check (usage.py:31-41), so do not call it from Lite |
| `upload/rm_upload.py`, `py programs/` | Reference only | History |
| V6 FINAL JSONs | Test data | Golden planner input (7 stages, one checkpoint) |
| Trajectory HTML | Not applicable | It produces the `.txt` files only. Its export uses LF (HTML:1619); the runner rebuilds CRLF (r14:76-97) |

**Input claim check: inaccurate.** v48 is neither the latest nor the most comprehensive version.
- Three newer UIs (v51, v52, v53, all dated Oct 5 after v48) sit in the `upload_channel` root. v53 contains all of v48, and v53 + runner v14 is the current pair.
- With the production recipe, v48 and v51 reject it. v52 plans it without the lid pause. Only v53 plans it with the manual lid checkpoint.
- The companion OPS notes ("UI v50 / Runner v13") are also stale. No v49 or v50 file exists, the TOGETHER rule they describe was replaced in v52, and the Multi Run and Load Sequence features they list do not work in v48 or v53.

---

## 6. Revised requirement (INT-CTRL)

Lite shall integrate the upload-channel control application, taking UI v53 and runner v14 (with `hand.py` and `pole.py`) as the functional baseline. v48 is not the baseline.

**(a) Executor.**
- In Phase 1, Lite moves the robot only through runner v14 + `hand.py` + `pole.py`, pinned by checksum.
- UI v53 is the reference for behaviour and dual-arm rules.

**(b) Operator area.** Without seeing slots, Program IDs or codes, an operator can:
- see each arm's connection, mode and activity;
- apply a region's pole and hand preset while that arm is idle;
- drag-teach and save a recording named by region;
- run a named cleaning recipe on one or both arms, after one confirmation that names the arms, the mode and the recipe;
- answer plain-language prompts (lid checkpoint, handoff);
- stop one arm or both with one click at any time.

**(c) Engineer area:**
- connection settings: IP and port per arm, SDK path, and a test-connection button that shows the mode and the stored programs;
- controller programs: upload and verify, multi-upload, slot list, replace confirmation, single and dual test runs;
- recipe builder: Program, Pole, Hand, Wait and Global steps; dual rules; Start % and overall speed; plan preview;
- global points;
- SIM/REAL mode;
- raw logs;
- version information.

**(d) Compatibility.**
- Lite reads and writes `realman_workflow_sequence_v1` files that v53 loads and plans identically, using the v52 timeline semantics and the v53 checkpoint semantics.
- A golden test on the V6 70-series pair proves this.
- Lite refuses a file whose `side` does not match the arm.

**(e) Architecture.**
- **Phase 1 (Must):** one runner subprocess per action, behind a localhost-only backend. The backend passes `--ip/--port`, turns the output into structured events, owns the dual-arm orchestration and holds one lock per controller.
- **Phase 2 (Should):** a library on `alix_dispatch` with one persistent connection per arm. The screens and files do not change.

**(f) Safety.** Keep every runner safety behaviour, and add:
- cross-arm abort;
- a pole/hand interlock: no move while that arm is running or settling;
- REAL-mode visibility and confirmation;
- a pinned runner version;
- one command source per controller;
- a slot check before every recipe;
- stop-all on backend shutdown;
- pole and hand steps skipped (and said so) in controller SIM mode.

**(g) Hardware gate.**
- Before operators run recipes in REAL mode, the stop gate is measured on both controllers. The gate asks whether slow stop aborts a running stored program (ALIX G4).
- Its result decides whether TOGETHER blocks are offered to operators.

---

## 7. Recommendation and approach

### Phase 0: hardware gates (about 1 h on the arm, before operator use)

ALIX estimates "~1 h on the arm" for these gates (ALIX/docs/motion/DRIVER_EXECUTION_ORCHESTRATION_PLAN.md:347).

1. **G4.** On each controller, run `track.probe_stop_policy` (DP/track.py:3273) on a dry, in-place track from `tools/cleaning_tracks.py dry` (ALIX/tools/cleaning_tracks.py:3729). Record whether `slow_stop` aborts the running program, and how far the arm travels while it stops.
2. **SIM behaviour.** With the controller in SIM, send one pole and one hand command from the runner and observe the result: no motion, a time-out, or real motion.
3. **If G4 fails:** Phase 1 needs a small runner change before release. Escalate the stop to pause and then stop, as `stop_program` does (DP/track.py:3109).

### Phase 1 (Must): wrap runner v14

1. **Freeze the baseline.**
   - Copy r14, `hand.py` and `pole.py` unchanged into `alix_studio_lite/robot/runner/`.
   - Record their sha256 in `runner_manifest.json`, and refuse any other runner.
2. **Settings.**
   - `lite_settings.json` holds, per arm, `{ip, port}`, plus `sdk_dir` (default: ALIX's vendored SDK, `ALIX/external/butterfli-realman-alix/Python`).
   - Refuse identical LEFT and RIGHT IPs.
   - **Test connection** runs `--list --get-arm-mode` in one process (r14:2615-2624) with a 10 s time-out, and shows "Connected, mode, N stored programs (names)" or "Not reachable".
   - Always pass `--ip/--port`; that makes the `RM_*_IP` environment overrides irrelevant (r14:2347).
3. **ArmChannel** (one per arm, with a lock).
   - The command line is `[python, -u, runner, --sdk, SDK, --side, S, --ip, IP, --port, P, --yes] + verb`, started with `start_new_session=True`.
   - A thread reads the lines, and every line goes to `runs/<date>/<run-id>_<side>.log`.
   - Only one runner may be active per arm.
   - **Shutdown:** if any runner is active, stop it first (SIGINT, wait for it to exit), as v53 refuses to close while a command is active (v53:11509-11527).
   - **Binding:** listen on 127.0.0.1, and require a per-session token on motion routes.
4. **Event adapter.**
   - A regex table maps the v14 lines to typed events: upload verified or failed; run started, complete or not complete; `WORKFLOW STEP/PROGRAM i/n`; `!! arm reports <code> at <t> s`; `ARM RUN MODE`; `DRAG ...`; `MANUAL HAND/POLE ...`; slot rows; `GLOBAL POINT {json}`.
   - `i/n` counts within the stage file, so the orchestrator maps it back to the recipe step.
   - Unknown lines are kept as raw log.
   - The routes are `GET /api/lite/robot/state` and `GET /api/lite/robot/events?after=N`, polled every 200-250 ms.
5. **Stop.**
   - `POST /api/lite/robot/stop {side|both}` sends SIGINT **once** to each active process group, with no confirmation, and cancels pending stages.
   - A second press does not send another SIGINT, because a second signal can cut the runner's own slow-stop, settle and speed-restore clean-up (r14:253-262).
   - The stop counts as confirmed when the runner prints its slow-stop line and exits. Its settle can take up to 5 s (r14:178-186), so wait about 8 s. If neither has happened by then, show "Stop not confirmed: use the E-stop".
6. **Planner.**
   - Move the two pure planner functions into `planner.py`, working on plain step lists.
   - Keep v53's validation messages for engineers.
   - Golden tests: the 7-stage V6 plan, plus the checkpoint-mismatch and unmatched-serial errors.
7. **Orchestrator.** A server-side state machine replaces v53's Tk timer loops. It handles four kinds of stage:
   - **parallel:** a barrier launch of two runners, each with its stage's v1 JSON and `--workflow-overall-speed`;
   - **serial:** the first arm runs, then an optional prompt; the second arm starts only after `rc == 0`;
   - **checkpoint:** a prompt while the robots are idle;
   - **tail:** the trailing non-program steps.

   It adds what v53 lacks:
   - an explicit "launching" state;
   - **cross-arm abort**, triggered by the first non-4105 `!! arm reports` line or a non-zero exit, whichever comes first;
   - "Stopped", not "Complete", when a stop falls between stages;
   - a start time logged per arm from the "start ack" line, so the start skew can be measured;
   - in controller SIM mode, pole and hand steps stripped from the stage files *after* planning (skip any stage file left empty) and announced;
   - TOGETHER offered to operators only after G4 passes. Until then, engineer-supervised only.
8. **Slot check before a recipe.**
   - Run `--list`, and compare each slot's stored name and size with Lite's upload record.
   - Refuse on a mismatch. Slots persist across power cycles, and a short slot ends early with no error (DP/track.py:353-392).
9. **Recipes.**
   - A recipe is a folder with LEFT.json and RIGHT.json (v1) plus `recipe.json` (name, regions, checkpoint text).
   - Import and export v1 unchanged, including the legacy `"speed": 100` and `transit_speed` (default 30).
   - An explicit checkpoint exports as a positive WAIT on both arms.
   - Check `side` on load.
10. **Front end in the ALIX Studio shell.**
    - For operators: the Robot bar, Run, Teach hooks, and Hand & Pole presets.
    - For engineers: the screens listed in section 9.
    - Prompts use `showStudioModal` and post to `/api/lite/recipe/answer`.
11. **Mode.**
    - Read the mode on connect and after every switch.
    - Confirm before switching to REAL.
    - The confirmation before any dual run states both arms' modes, which restores the v32 warning.

**Route sketch:**
- `GET /api/lite/robot/{state,events,programs}`
- `POST /api/lite/robot/{connect-test,stop,upload,upload-batch,run-program,mode,hand,pole,drag/start,drag/stop,drag/save,drag/playback,global-move}`
- `POST /api/lite/recipe/{plan,run,answer}`
- `GET/POST /api/lite/settings`

### Phase 2 (Should): a library on alix_dispatch

Behind the same backend interface, replace the subprocess with these parts:
- **`ArmSession`:** `connect.open_arm`, one persistent connection per arm, reconnect, and a cached program list.
- **`ProgramStore`:** `send_project`, a size read-back, `preflight_slots`, skipping identical uploads, and batch uploads in one session.
- **`Runner`:** `global_speed_guard`, the `stop_program` ladder, and `wait_until_at_rest`; joints captured when an error code appears.
- **`RunMode`:** the run-mode read and set.
- **`Pole`/`Hand`:** `peripherals`, or `hand.py`/`pole.py`.
- **Ported from r14:** `Workflow` (r14:1766-2277), `DragTeach` (r14:912-1210) and `GlobalPoints` (r14:1265-1665), because alix_dispatch has none of them.

**Proving equivalence.** Use `emulated_program` for the program verbs. The emulator lacks the drag-teach and global-point verbs (map 04 section 4.4), so also use one recorded hardware session.

**What Phase 2 also delivers:** the live joint feed for the 3D view, and faster stage switching for R13.

### Compatibility rules (both phases)

- Keep `realman_workflow_sequence_v1` exactly.
- **v52 timeline:** consecutive TOGETHER rows form one block per arm; IDs and counts may differ; the arms re-synchronise only at block edges.
- **v53 checkpoint:** a positive WAIT before a TOGETHER Program on both arms becomes LID-PREP, then LID-CHECK, then the next block. A WAIT on one arm only is rejected.
- **RUN FIRST / WAIT OTHER** need the same Program ID at the same position on both arms.
- **The Tk app v53 stays as an engineering fallback.** It and Lite must never drive the same controller at the same time, and the ROS driver must be down (r14:26).

---

## 8. Acceptance criteria

1. **Runner pin.** The backend refuses a runner whose sha256 is not the pinned v14 value. Pointing it at `rm_upload_fast_v13_overall_speed.py` gives a clear refusal.
2. **Fake SDK.** The unchanged runner v14 runs every action against a fake `Robotic_Arm` package passed with `--sdk`. The actions are upload, run, workflow, drag start/stop/save, hand, pole, mode and list. The tests check that:
   - each action produces the expected events and final arm state;
   - every command line contains `--sdk`, `--side`, `--ip`, `--port` and `--yes`;
   - unknown lines go to the raw log.
3. **Planner golden test.** The V6 FINAL pair gives exactly `TIMELINE-1 L[71,73] R[71,74,75]`, then `LID-PREP-1`, `LID-CHECK-1`, `TIMELINE-2 [76]`, serial 78 (LEFT first), serial 79 (LEFT first), and `TIMELINE-3 [80,83]`. A WAIT on one arm only, and an unmatched RUN FIRST, each raise the v53 error.
4. **Round trip.** Loading and saving each V6 FINAL JSON gives a file that the v53 loader accepts and that plans the same stages. `format`, step order, `dual_rule` and `transit_speed` are unchanged. A `side=left` file is refused on the RIGHT channel.
5. **STOP ALL.** In a fake two-arm run, one click signals every active runner within 200 ms, with no dialog, and no later stage starts. A second click sends no second signal to the same runner.
6. **Cross-arm abort.** In a fake TOGETHER stage where LEFT prints a 4109 line or exits with rc=1, RIGHT is signalled within 500 ms, and the recipe ends "Stopped: left arm failed". In a serial handoff, the waiting arm never starts after a failure or a stop.
7. **Settings.**
   - Changing the LEFT IP changes `--ip` on every later LEFT command.
   - Identical LEFT and RIGHT IPs are refused.
   - Test connection reports Connected (with the mode and the stored programs) or Not reachable within 10 s.
8. **Slot check.** If a slot's stored size or name differs from Lite's upload record, the recipe refuses to start and names the slot (engineer view) or the region (operator view).
9. **SIM mode.** With a controller in SIM, the stage files Lite sends contain no pole or hand steps, and the run screen says they were skipped.
10. **Lifecycle and binding.**
    - Stopping the Lite server while a fake runner is active signals the runner and waits for it to exit before the server exits.
    - The server listens on 127.0.0.1 by default, and motion routes reject requests without the session token.
11. **Operator wording.** An automated text check of the operator screens finds no slot numbers, Program IDs, `rc=` or `TIMELINE-`.
12. **Logs.** Every run writes one log per arm. The logs survive a page reload and are not cleared between stages.
13. **Hardware**, in this order:
    1. G4 probe on both controllers, recorded.
    2. The SIM behaviour of pole and hand, recorded.
    3. Upload and verify one `.txt` per arm in the agreed slot range.
    4. The V6 recipe end to end in controller SIM, with the lid prompt, pole and hand skipped.
    5. One region's recipe in REAL.
    6. STOP ALL during REAL motion: both arms stop, and the start skew from the start-ack lines is recorded.

---

## 9. UX notes (non-technical operators)

**Operator area.**
- **Robot bar, always visible** (OPS sections 1, 9, 19).
  - Two large arm cards, e.g. "Left arm: Connected · Real robot · Cleaning 'Lid top' (step 3 of 9) · 01:12".
  - STOP LEFT, STOP RIGHT and a red STOP ALL. None asks for confirmation, and **all stay clickable above every dialog**, including the lid prompt.
- **Teach** (OPS section 5; the flow belongs to R1/R7). Start and stop recording per region, with an automatic save named by region.
  - If a save fails, retry it automatically, since the save lags (ALIX/experiments/exp01_teach_trajectory_reverse_engineering/notes/2026-08-25_phase_a.md:92-94). The runner tries only once (r14:957-989).
  - A new recording cannot start until the last one is saved: the controller keeps only its LAST recording.
  - START and STOP BOTH stay. Playback and file paths go to the engineer area.
- **Hand & Pole presets** (OPS sections 7, 8). GRIP and OPEN, plus the region's pole preset, usable only when that arm is idle. The six sliders and free mm entry go to the engineer area.
- **Run** (OPS sections 13-15, 21, 24).
  - Pick a named recipe and press Start.
  - Each arm shows the region name, "step k of n", any waiting reason ("Waiting for the left arm to finish") and the elapsed time.
  - Prompts appear only when a person must act, for example "Open the toilet lid fully, step out of the robot area, then press Continue".

**Engineer area.**
- Settings & Connection (OPS sections 1, 22).
- Controller Programs (OPS sections 2-4).
- Recipe Builder with plan preview (OPS sections 10-12, 18).
- Global Points (OPS section 6).
- Mode (OPS section 9).
- Logs (OPS section 20).
- OPS sections 16 and 17 stay in the backend. For section 23, the ALIX Studio theme replaces the Tk palette.

**Plain words.**
- TOGETHER becomes "Both arms together".
- RUN FIRST / WAIT OTHER becomes "Left arm first, then right".
- SIM becomes "Dry run on the controller: the arm does not move; pole and hand steps are skipped".
- REAL becomes "Real robot: the arm moves".

**Confirming and reporting.**
- Show one confirmation card before any motion, naming the arms, the mode and the recipe (`showStudioModal` chips).
- Never show exit codes or stage labels to operators.
- Write each failure as what happened plus the next action.

**Busy arms.** Disable an arm's controls on every screen while that arm is in a recipe, and show why. In v53, an arm's Page-01 controls work whenever it has no active runner, including between the stages of a RUN BOTH.

---

## 10. Safety notes

1. **Keep every runner safety behaviour unchanged:**
   - whole-file validation;
   - store-only upload with size read-back;
   - pre-run refusals (empty slot, arm error, busy controller);
   - 4105 treated as a warning only in sequences;
   - the global speed restored after every run;
   - the mode verified by read-back.
2. **STOP.**
   - One click, no confirmation, always visible and above every dialog, sent to every active arm, once per runner.
   - If the stop cannot be confirmed, tell the operator to use the hardware E-stop.
   - Lite's STOP is a convenience, not a safety function. The E-stop must be within reach in REAL mode.
3. **G4 first.** Whether slow stop aborts a running stored program is unmeasured (section 4, item 1). Run the probe before operators use REAL mode. Until it passes, TOGETHER blocks run only with an engineer present, as ALIX's own dual executive demotes concurrent steps (ALIX/tools/dual_arm_flow.py:49-53).
4. **Cross-arm abort.** In a block where both arms run, a failure or stop on one arm stops the other (v53 lets it finish, v53:10273-10300). Its effect also depends on G4.
5. **Pole and hand interlock.**
   - Never move the pole or hand while that arm's runner is active, or within a short guard time after it exits.
   - Never move them with the pad on a surface (usage.py:25).
   - Hardware facts behind this rule:
     - a lift command during a planned arm move truncates both (ALIX/docs/PHASE_PLAN.md:122; DP/peripherals.py:21-24);
     - "program idle" is not "at rest": arms ran at 13-96 deg/s 0.6 s after `run_state` 0 (DP/track.py:1953-1957).
   - Inside a recipe, the runner's settle before a following pole step lasts at most 1 s and is advisory (r14:1947-1981). Recipe authors must therefore end each Program clear of surfaces before a pole step.
   - Phase 2 adds `wait_until_at_rest`.
6. **SIM mode.** The controller does not execute lift or hand in SIM (ALIX/docs/PHASE_PLAN.md:116). Lite skips those steps in SIM and says so. Verify on hardware before writing the operator label.
7. **Modes visible.**
   - Show each arm's mode at all times.
   - Confirm before switching to REAL.
   - State both arms' modes in every run confirmation.
8. **One command source per controller.**
   - The ROS driver is down (r14:26), the Tk app is closed, and Lite holds the lock. This matches ALIX's "one owner per controller" rule (DP/track.py:29).
   - Lite cannot detect the ROS driver without ROS, so show it as a checklist item.
9. **Pinned runner.** A v13 runner with v51+ files would run approach moves at 100 %.
10. **Slots.**
    - Write only into Lite's agreed range.
    - Show what a slot holds before replacing it.
    - Check every slot's stored name and size before a recipe runs.
11. **Wrong arm.**
    - Refuse identical IPs.
    - Check each recipe file's `side`.
    - Show the stored program list on Test connection, so an engineer can see which controller answered.
12. **Backend lifecycle.**
    - On shutdown, stop all runners first.
    - If the backend crashes, stored programs keep playing ("the program KEEPS PLAYING if the client dies", DP/emulated_program.py:59-61). The E-stop is the fallback.
13. **Network.** Bind to 127.0.0.1 by default (the Studio warns against unauthenticated binding, TG/src/alix_taskgen/server.py:6539-6544), and require a session token on motion routes.
14. **Blocking moves.**
    - Global-point MoveJ/MoveL and drag playback are blocking SDK calls (r14:1498, 1522, 1182, 1190). A STOP acts only after they finish (inferred from Python's signal handling).
    - Keep them engineer-only, at low speed. Phase 1 gives operators no "Park" button.
15. **Lost browser.** Decide what happens if the browser disconnects during REAL motion (open question 8).

---

## 11. Dependencies, risks, open questions

**Dependencies:**
- **R4** (real-arm testing, IP entry, real/sim toggle). It uses the settings, mode and run APIs defined here.
- **R1 and R7.** The Teach flow and per-region presets use the drag, hand and pole actions. R7 also decides the exact pre-stroke order: V6 keeps GRIP across pole moves, while ALIX's staging opens the hand before a pole move.
- **R13.** Faster upload goes through this upload path; the Phase-2 `ProgramStore` serves it.
- **R8 and R10.** They must produce v1 files that this planner accepts.
- **R5.** It needs the arm-code events with elapsed time.
- **NFR-THEME and NFR-UX:** the ALIX Studio shell.
- **F13:** the architecture is favoured, not yet decided, as a standalone `alix_studio_lite` package.
- **Hardware.** Access to both RM75 controllers (192.168.1.10, 192.168.1.103) and about 1 h of arm time for Phase 0.
- **The RealMan SDK** for the operator PC. Its licence is an open question in ALIX (ALIX/DEPENDENCIES.md:64-71).
- **The ALIX team's agreement** on program-ID ranges and controller ownership.

**Risks:**
- **G4 is unmeasured.** If slow stop does not abort a running program, Phase-1 STOP is ineffective until the runner is changed.
- **SIM behaviour of the pole and hand is unknown.** A dry run may fail, or the pole and hand may move during a "dry run".
- **TOGETHER blocks run open-loop.**
  - The start skew is unmeasured; ALIX measured 27 ms in one process (DP/schedule.py:12), and a subprocess launch adds more.
  - The player clock varies by plus or minus 6 % from run to run (DP/track.py:6-8). ALIX verdicts use drift windows of 3.2-3.6 s.
  - Pole timing adds about 2-3 s of drift in V6.
- **Reconnecting at every stage** (12 connections for V6) adds latency until Phase 2.
- **Parsing stdout is fragile.** The checksum pin prevents silent drift.
- **Program-ID collisions** with ALIX's slots 70-100.
- **Operator expectations.** Operators trained on OPS expect TOGETHER to pair matching IDs.
- **Phase 2 coupling.** `alix_dispatch` has repository-layout couplings (it loads `hand_units.py` by path).

**Open questions:**
1. Which program-ID range does Lite own on each arm? The UI defaults to 25 and 99, the README says 40-59, ALIX reserves 70-100, and V6 uses 71-83.
2. If G4 fails, should TOGETHER blocks be demoted to one arm at a time (as ALIX does), or should the runner gain a stronger stop first?
3. What do a pole command and a hand command actually do while a controller is in SIM: no motion, a time-out, or real motion?
4. Should the checkpoint stay "a positive WAIT on both arms" (the v53 contract), or become an explicit step type exported as that WAIT? In v53, every such WAIT shows the lid text.
5. Will v53 stay as the engineering fallback? If so, who applies the `@staticmethod` fix?
6. Do operators still need any dropped feature: `DT_` save-to-controller, the LEFT-then-RIGHT or ALT modes, or the "% open" hand slider?
7. How are the operator and engineer roles separated: a role switch or a PIN?
8. Should REAL motion stop automatically when the browser stops polling, and after how many seconds? Background tabs throttle timers.
9. Must Lite run on Windows? The robot PC is Linux, so the default answer is no.
10. Phase 2: should `alix_dispatch` be vendored into Lite, or pinned to an ALIX revision?

---

## 12. Priority and effort

**Priority: Must** for Phase 1 and Phase 0. Phase 2 is **Should**.

**Effort: XL.**

| Part | Estimate | Content |
|---|---|---|
| Phase 0 | about 1 day incl. preparation | G4 probe and the SIM pole/hand check on both controllers |
| Phase 1 | **16-20 person-days** (XL on its own) | Channel, lock, logs and lifecycle (2). Event adapter and fake-SDK harness (3). Planner port and golden tests (1). Orchestrator with cross-arm abort, SIM stripping and the slot check (3-4). Settings and pin (1.5). Operator screens (2-3). Engineer screens (4-5; the recipe builder alone is about 800 lines of Tk to rebuild, v53:6074-6874) |
| Phase 2 | **8-12 person-days** (L) | Sessions on `alix_dispatch`; ports of the workflow, drag teach and global points; emulator and hardware equivalence |

**Lever.** Lite's first release can import recipes authored in v53's Page 02, which works, and defer its own recipe builder. That brings Phase 1 to about 12-15 person-days.

**Excluded:** the region Teach flow (R1), per-region preset data (R7), upload speed-ups (R13) and the 3D simulation (R3/R10).

---

## 13. Verifier change log

| # | Change | Reason | Evidence |
|---|---|---|---|
| 1 | "Only v53 can run the V6 recipe" became: v52 also plans it (4 stages) but without the lid pause; v53 is the only one that pauses | Re-ran all four planners | Planner re-run (v52 output in section 1); V6 JSON WAIT step |
| 2 | 26 UIs in `py programs/` became **28** | Count | `ls "UC/py programs" \| grep -c rm_trajectory_ui_` = 28 |
| 3 | "ALIX keeps hand and pole on ROS, so Lite keeps hand.py/pole.py" became: alix_dispatch has ROS-free `peripherals` (Pole, HandModbus) and `staging`; hand.py/pole.py are trimmed copies of them. The ROS rule in `track.py` is a production-pipeline policy | Missed modules | DP/peripherals.py:1-51; DP/staging.py:1-15, 358-380; DP/track.py:13-16; hand.py:3-17 |
| 4 | Added alix_dispatch items: `probe_stop_policy`, `wait_until_at_rest`, `at_rest_barrier`, `preflight_slots`, `assert_side`, `stop_program`; noted there is no drag-teach or global-point code | Missed, directly reusable | DP/track.py:341, 353, 1949, 2960, 3109, 3273; grep of DP |
| 5 | Added the SIM-mode pole/hand problem. Changed the SIM label and the hardware acceptance (pole/hand skipped in SIM) | The V6 SIM dry run would likely fail at its first pole step | ALIX/docs/PHASE_PLAN.md:116; DP/session.py:1-8, 276-310; pole.py:46-58 |
| 6 | Made G4 a gate before operator REAL use and reordered the hardware tests (G4 first). TOGETHER blocks are gated on it | ALIX policy: single-arm replay until G4(b); G4 never run; a probe exists | DRIVER_TRACK_CHANNEL_ARCHITECTURE.md:203, 295; DRIVER_EXECUTION_ORCHESTRATION_PLAN.md:46, 347; dual_arm_flow.py:49-53 |
| 7 | "Fake runner replaying recorded transcripts" became a fake `Robotic_Arm` package via `--sdk`, so the real runner code runs in CI | No transcripts exist in the workspace; ALIX already has the pattern | r14:60-73; ALIX/core/python/alix_dispatch/test/test_rm_upload.py:1-60 |
| 8 | Stop confirmation window: 3 s became "slow-stop line + exit, about 8 s"; the signal is sent once per runner | The runner settles for up to 5 s after a stop; a second SIGINT can cut its clean-up | r14:178-186, 253-262; v53:6053 |
| 9 | Added backend shutdown and crash handling | Programs keep playing if the client dies | DP/emulated_program.py:59-61; v53:11509-11527 |
| 10 | Added localhost binding and a session token | A robot-moving HTTP API must not be open to the network | TG/src/alix_taskgen/server.py:6539-6544 |
| 11 | Added wrong-arm protection; dropped "clear RM_*_IP" as a requirement | `--ip` overrides `--side`; `side` is never checked | r14:2347, 1751-1763; v53:6819-6861; DP/track.py:341-350 |
| 12 | Added the advisory settle, at-rest and F9 evidence to the pole/hand interlock | The runner sends a pole step after at most 1 s of advisory settle | r14:1947-1981; ALIX/docs/PHASE_PLAN.md:122; DP/track.py:1953-1957 |
| 13 | Added the slot check before a recipe | Stale or short slots run silently | DP/track.py:353-392; r14:2012-2033 (checks presence only) |
| 14 | Live joint mirror moved to Phase 2; Board pattern noted as visual only | The runner prints no joints; Board actions call the excluded generator | TG/server.py:5326-5341; TG/.../template.html:542-546 |
| 15 | Blocking global moves and drag playback made engineer-only; no operator "Park" button in Phase 1 | STOP acts only after a blocking call returns (inferred) | r14:1182, 1190, 1498, 1522 |
| 16 | Teach: retry the save, and do not start a new recording until the last one is saved | The save lags; the controller keeps only its last recording | exp01 notes 2026-08-25_phase_a.md:92-96; r14:957-989 |
| 17 | Effort: Phase 1 became 16-20 pd (was 12-15), Phase 2 became 8-12 pd (was 6-8); added the defer-builder lever; Phase 2 priority set to Should | The size of the builder and orchestrator to rebuild; drag and global ports | v53:6074-6874, 10055-10731; DP grep |
| 18 | Clarified that "G4" here is the driver-plan stop gate, not the taught-track deviation gate G4 | The name is overloaded in ALIX | map 06 (taught-track G1-G5); DRIVER_EXECUTION_ORCHESTRATION_PLAN.md:46 |
| 19 | "As decided in F13" became "favoured in F13" | Architecture not yet decided | Task facts F13 |

---

## Cross-cutting notes

- **Version baseline for every cluster.** The control app is UI v53 + runner v14, not v48 and not OPS's "UI v50 / Runner v13". On the V6 recipe, v48 and v51 reject it, v52 runs it with no lid pause, and v53 runs it with a manual lid checkpoint.
- **Naming.** The control app's "SIMULATION" is the controller's own dry run. The arm does not move, and the controller does not execute pole or hand. Call it "Dry run on the controller", and call the 3D view "3D preview" (R3, R4, R10).
- **Stop gate G4.** It is unmeasured on these controllers. Any concurrent dual-arm run (R8 merging, R10 dual timing, INT-CTRL TOGETHER blocks) depends on it. ALIX's own policy keeps program replay single-arm until it passes, and a ready probe exists (DP/track.py:3273).
- **Robot actions for other clusters come from this layer.** These are drag teach start/stop/save/playback (R1), pole 0-315 mm and hand 6 x 0-1000 with the GRIP 0,0,0,0,150,1000 and OPEN presets (R7), upload and verify (R13), and arm error-code events with elapsed time (R5). ALIX Studio has none of these.
- **Pole and hand.** For R7, alix_dispatch has ROS-free `peripherals` and a hardware-proven `staging` order with an at-rest interlock (DP/staging.py:358-380). `hand.py`/`pole.py` are trimmed copies of `peripherals`.
- **File contract.** Keep `realman_workflow_sequence_v1`, including the v52 timeline blocks and the v53 checkpoint rule, as the contract shared with the Tk app. R8 and R10 must emit files this planner accepts. Lite must check `side`.
- **Program IDs.** Agree the program-ID range before the first upload: ALIX reserves 70-100 on the same controllers, and V6 uses 71-83.
- **Theme.** Do not port the dark Tk palette (v53:56-71). Use the ALIX Studio tokens (TG/.../editor.css:22-44) and keep only the colour meaning: run = ok, upload = brand, stop = danger.
