# 09 — Control-application runner + helpers + RealMan SDK (INT-CTRL, R4, R7, R13)

Scope: the "upload channel" runner that the Tkinter control UI shells out to, its hardware helpers,
the original uploader it grew from, and the vendored RealMan RM API2 Python SDK functions it calls.
Everything below was read in place (read-only). Anything not seen directly is marked **(inferred)**.

## Path aliases used in citations

| alias | absolute path |
|---|---|
| `RUN` | /home/newtonjeri/alix_studio_lite_ws/upload_channel/rm_upload_fast_v14_safe_transit.py (2702 lines) |
| `RUN13` | /home/newtonjeri/alix_studio_lite_ws/upload_channel/rm_upload_fast_v13_overall_speed.py (2683 lines) |
| `HIST/` | "/home/newtonjeri/alix_studio_lite_ws/upload_channel/py programs/" (older runners v4..v12, UIs v8..v48) |
| `ORIG` | /home/newtonjeri/alix_studio_lite_ws/upload_channel/upload/rm_upload.py (344 lines) |
| `UI53` | /home/newtonjeri/alix_studio_lite_ws/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py |
| `UI48` | /home/newtonjeri/alix_studio_lite_ws/upload_channel/rm_trajectory_ui_v48_handoff_continue_fix.py |
| `POLE` / `HAND` / `USAGE` | /home/newtonjeri/alix_studio_lite_ws/upload_channel/{pole.py, hand.py, usage.py} |
| `README` | /home/newtonjeri/alix_studio_lite_ws/upload_channel/README_COMMANDS.md |
| `WFL` / `WFR` | /home/newtonjeri/alix_studio_lite_ws/upload_channel/{LEFT,RIGHT}_V6_70SERIES_FINAL.json |
| `FEAT` | "/home/newtonjeri/alix_studio_lite_ws/RealMan Dual-Arm Operations UI.txt" |
| `SDKI` | /home/newtonjeri/alix_ws/src/alix/external/butterfli-realman-alix/Python/Robotic_Arm/rm_robot_interface.py |
| `SDKW` | /home/newtonjeri/alix_ws/src/alix/external/butterfli-realman-alix/Python/Robotic_Arm/rm_ctypes_wrap.py |
| `ALIX/` | /home/newtonjeri/alix_ws/src/alix/ (repo-relative paths follow) |

## 0. Inventory, identity and lineage (read this first)

* `RUN`, `rm_upload_fast_v14_safe_transit_v52support.py` and `..._v53support.py` are byte-identical
  (md5 `a0b285e6336be2feac880e5dbe381db6` for all three; checked with `md5sum`). The UI only ever looks
  for the plain `rm_upload_fast_v14_safe_transit.py` name (`UI53:6890-6907`); the `_vNNsupport`
  copies are unreferenced backups.
* `pole.py`, `hand.py`, `usage.py`, `README_COMMANDS.md` in `upload_channel/` are identical to the
  copies in `upload_channel/upload/` (`cmp`), and `upload.zip` is a snapshot of `upload/`
  (zip listing). `ORIG` is identical to `HIST/rm_upload.py` and, apart from the usage-path lines
  4-7, to `ALIX/tools/rm_upload.py` (diff). Its git log has three commits: commit 19ff4cb8 added the
  standalone uploader, b8d2861e switched the program list to paged reads, and 22e15c08 converted the
  SDK's text error codes to integers (`git log -- tools/rm_upload.py`).
* **Core identity:** `RUN:1-268` (SDK loader, track parser, project writer, list read, arm state,
  upload, settle, run) is byte-identical to `ORIG:1-268` except two docstring lines (`RUN:7`,
  `RUN:26`) — confirmed by `diff`. All later functions are additions.
* Version ladder (function lists via `grep '^def '` per file):
  | runner | what it added |
  |---|---|
  | `HIST/rm_upload_fast_v4.py` (861 l) | `--sequence` "fast multi-run, one connection" + hand/pole per step (`run_sequence`, `parse_hand_target`, `load_aux_classes`) |
  | v5 | `manual_hand_control`, `manual_pole_control` |
  | v6 / v7 | drag teach session/save/playback; drag `.txt` -> controller program (`DT_` naming) |
  | v9 | run mode SIM/REAL get/set; global waypoints list/move/sequence |
  | v10 | `manual_hand_raw_control` (six independent channels) |
  | v11 | `--workflow-sequence-file` (program/pole/hand/wait/global steps) |
  | v12 "fast_workflow" | `run_workflow_program_fast` + `workflow_quick_settle` (see §3) |
  | v13 "overall_speed" | `--workflow-overall-speed` (default 100), `rm_set_plan_speed` before workflow and before each program (`diff HIST/v12 RUN13`) |
  | **v14 "safe_transit"** | per-program `transit_speed` (alias `start_speed`, default 30) decoupled from the legacy `speed` field (`diff RUN13 RUN`: 54 diff lines, all in `normalize_workflow_step` / `run_workflow_program_fast` / `run_workflow_sequence`, `RUN:1679-1700,1807-1873,2005-2009,2079-2080,2168`) |
* UI pairing: `UI48` prefers runner **v13** (`UI48:6854` first candidate) and has no
  `EXPECTED_RUNNER_VERSION`; UIs v51/v52/v53 set `EXPECTED_RUNNER_VERSION = "v14"` (`UI53:46`) and
  list v14 first (`UI53:6891`). So the latest pair is **UI v53 + runner v14**, not v48
  (`UI48` = `HIST/rm_trajectory_ui_v48_handoff_continue_fix.py`, identical by `cmp`).
* SDK: vendored at `ALIX/external/butterfli-realman-alix/Python/Robotic_Arm/` (git pin `0691092`),
  `libs/linux_x86/libapi_c.so` present; `Robotic_Arm` is **not** pip-installed on this machine
  (`importlib.util.find_spec('Robotic_Arm')` -> None).

## 1. CLI — every flag, and how the UI invokes it

### 1.1 How the UI calls the runner
* Every command is `[sys.executable, "-u", <runner>, "--sdk", <sdk dir>] + action flags`
  (`UI53:4255-4262`). The SDK dir defaults to env `ALIX_RM_SDK` else
  `/home/alai-labs/alix_ws/src/alix/external/butterfli-realman-alix/Python` (`UI53:48-51`) — a path
  from the robot PC (user `alai-labs`, `README:3,44`), absent on this workstation, so the UI's
  `_validate_environment` refuses until the env var is set or a folder is browsed (`UI53:4212-4224`).
* Popen: `stdout=PIPE, stderr=STDOUT, text=True, bufsize=1`, `start_new_session=True` on POSIX /
  `CREATE_NEW_PROCESS_GROUP` on Windows (`UI53:5737-5750`, `4394-4409`); a worker thread reads lines and
  pushes them to Tk with `self.after(0, ...)` (`UI53:5752-5760`).
* **Stop = signal, not an API call:** "Slow Stop" sends `os.killpg(os.getpgid(p.pid), signal.SIGINT)`
  (POSIX) or `CTRL_BREAK_EVENT` (Windows) to the runner (`UI53:6051-6061`; drag stop `UI53:3897-3902`).
  The runner turns `KeyboardInterrupt` into `rm_set_arm_slow_stop()` (§8). The runner installs no
  signal handlers (`grep signal RUN` = 0 hits).
* The UI **always passes `--side` and never `--ip`/`--port`** (flag census of `UI53`: `--side` x30,
  `--yes` x29, no `--ip`, `--port`, `--run-speed`, `--restore-speed`, `--timeout`, `--speed`). The
  IPs shown in the UI are hard-coded labels (`UI53:7588`, `7600`, `7506`); the real IP comes from the
  runner's `ARM_IPS` (env-overridable, `RUN:47-48`). **(inferred)** If `RM_LEFT_IP`/`RM_RIGHT_IP` are
  set, the UI labels lie.
* The UI confirms in its own dialogs and then passes `--yes` so the runner never prompts
  (e.g. batch comment `UI53:4605-4613`).
* `EXPECTED_RUNNER_VERSION` is only used to fill the version banner (`UI53:7390-7419`); nothing refuses
  a different runner. **Hazard (inferred from code):** if the v14 file is missing the UI silently falls
  back to v13 (`UI53:6890-6909`), whose `normalize_workflow_step` reads `speed` (`RUN13:1679`); the v53
  UI writes `"speed": 100` as a legacy field (`UI53:6537-6547`), so Program transits would run at 100 %
  instead of the intended 30 %.

### 1.2 Flags (argparse `RUN:2280-2346`)

| flag | type / default | validation | function | UI53 use |
|---|---|---|---|---|
| `TRACK SLOT` (positional) | Path, int | slot 1-100, file exists, `read_track` (`RUN:2350-2360`) | `upload` | upload only `UI53:4284-4291`; + `--run` `UI53:4319-4326`; batch `UI53:4607-4613` |
| `--side left\|right` | — | one of `--side`/`--ip` required (`RUN:2347-2349`) | IP via `ARM_IPS` | every command |
| `--ip` / `--port` | str / 8080 | none (no format check) | connect | **never** |
| `--speed` | int 0 (0-100) | `RUN:2356-2357` | `plan_speed` of `rm_send_project_t` (stored speed) | never (always 0) |
| `--run [SLOT]` | nargs `?`, const 0 | 1-100; `--run` alone = the uploaded slot (`RUN:2361-2369`) | `run` | run only `UI53:4723-4729`; dual `UI53:8912-8924`, `11328-11340` |
| `--run-speed` | int 20 (1-100) | `RUN:2368-2369` | transit speed in `run`/`run_sequence` | never (default 20) |
| `--restore-speed` | int 50 (1-100) | idem | `rm_set_plan_speed` after runs | never (default 50) |
| `--timeout` | float 300 s | — | run watchdog -> slow stop | never |
| `--sequence SLOT...` | ints | 1-100 | `run_sequence` (plain IDs) | never |
| `--sequence-file F` | JSON `realman_multi_run_sequence` | `load_sequence_file` `RUN:353-362` | `run_sequence` | Multi Run `UI53:5254-5260` |
| `--manual-hand PCT` | float 0-100 | `RUN:2452-2453` | `manual_hand_control` | never (v53 uses raw) |
| `--manual-hand-raw 6xINT` | 0-1000 each | `RUN:2455-2457` | `manual_hand_raw_control` | `UI53:4159-4166` (presets `UI53:2979-2998`) |
| `--manual-pole MM` | float 0-315 | `RUN:2459-2460` | `manual_pole_control` | `UI53:4196-4202` (UI clamps 0-315) |
| `--drag-session` | flag | one drag action at a time (`RUN:2381-2392`) | `drag_teach_session` | `UI53:3940-3945`; dual `UI53:7911-7917` |
| `--drag-save FILE` | Path | — | `drag_save_trajectory` | `UI53:4005-4011`; dual popup `UI53:8342-8349` |
| `--drag-playback` | flag | — | `drag_playback` | `UI53:4048-4053` |
| `--drag-store FILE --drag-slot ID [--drag-name N]` | | slot 1-100 (`RUN:2394-2398`) | `drag_store_current_to_controller` | not in v53 (used by UI v26-v32 only, grep of `HIST/`) |
| `--drag-run-file FILE --drag-slot ID [--drag-name N]` | | idem | `upload_drag_txt_program(run_after=True)` | not in v53 |
| `--get-arm-mode` | flag | exclusive with set (`RUN:2400-2402`) | `get_arm_run_mode` | CHECK button `UI53:3793` |
| `--set-arm-mode simulation\|real` | choice | — | `set_arm_run_mode` | SIM/REAL buttons `UI53:3814` |
| `--global-list` | flag | one global action (`RUN:2404-2413`) | `print_global_waypoints` | `UI53:3211-3216` |
| `--global-point NAME` `--global-move movej\|movel` `--global-speed 1-100 (20)` `--global-wait 0-3600 (0)` | | `RUN:2415-2422` | `move_global_waypoint` | `UI53:3365-3377` |
| `--global-sequence-file F` | JSON `realman_global_point_sequence` | `load_global_sequence_file` `RUN:1561-1612` | `run_global_sequence` | `UI53:3768-3774` |
| `--workflow-sequence-file F` | JSON `realman_workflow_sequence_v1` | `load_workflow_sequence_file` `RUN:1751-1763` | `run_workflow_sequence` | Page 02 `UI53:9313-9321`, `10089`, `10794` |
| `--workflow-overall-speed` | int 100 (1-100) | `RUN:2418-2419` | `rm_set_plan_speed` in workflow | always passed with workflow |
| `--list` | flag | — | prints `slot NNN  x KB  speed S  name` | `UI53:4536-4541`, `5499`; parsed by regex `UI53:4439-4442`, `5758-5762` |
| `--yes` | flag | — | skips `input()` prompts | always |
| `--sdk DIR` | Path | — | `load_sdk` | always |

Mutual exclusion groups and the "nothing to do" error: `RUN:2370-2604`. Dispatch precedence after
connecting (`RUN:2615-2696`): `--list` (prints, then continues) -> get mode -> set mode -> workflow ->
global list -> global point -> global sequence -> manual hand -> manual hand raw -> manual pole ->
drag session -> drag save -> drag playback -> drag store -> drag run file -> upload (+run) -> run ->
sequence. Exit codes: 0 ok, 1 refused/failed, 2 argparse error, 130 only from
`manual_hand_raw_control` on Ctrl+C (`RUN:855`); `sys.exit("msg")` paths exit 1 (`RUN:73,83,85,87,112,2359,2611`).

### 1.3 JSON input formats (all written by the UI to temp files)
* `realman_multi_run_sequence` v3: `{"programs":[{"id":1-100,"name","hand": null|"0-100"|"50%"|"v1,..,v6","pole_mm": null|0-315,"wait_s":0-3600}]}` — UI `UI53:1688-1700`; runner `RUN:274-362`.
  Per-step order is **Pole -> Hand -> Wait -> trajectory** (`RUN:402-406`).
* `realman_global_point_sequence` v1: `{"steps":[{"name"|"point_name","move":"movej|movel","speed":1-100 (20),"wait_s"|"wait":0-3600}]}` — `UI53:3740-3747`, `RUN:1561-1612`.
* `realman_workflow_sequence_v1`: ordered `steps` of
  `program {id 1-100, name, speed (legacy, default 100), transit_speed|start_speed (default 30)}`,
  `pole {mm 0-315}`, `hand {values: 6x0-1000 or "a,b,c,d,e,f"}`, `wait {seconds|wait_s 0-3600}`,
  `global {name, move, speed (20), wait_s}` (`RUN:1668-1748`). The UI also writes
  `dual_rule: TOGETHER|RUN FIRST|WAIT OTHER` (`UI53:6532-6548`) — **ignored by the runner**
  (dropped by `normalize_workflow_step`, `RUN:1694-1700`); the rules are executed by the UI, which
  splits the workflow into per-stage chunks and launches one runner process per arm per stage
  (`UI53:10190-10240`, `10570+`).
  Real production examples: `WFL` (14 steps) / `WFR` (16 steps): pole 200 mm -> hand
  `[0,0,0,0,150,1000]` ("GRIP PRESET", `UI53:2979-2981`) -> pole 150/50 mm -> Program 71 ... 83 with
  `transit_speed 30`, a 2 s `wait` "MANUAL LID COVER OPEN" (`WFL:2-92`).

### 1.4 The stdout protocol (the UI's only status channel)
The UI drives its state machine from line prefixes printed by the runner (census of
`clean.startswith(...)` / `in clean` in `UI53`, e.g. `UI53:5878-5960`, `5300-5380`):
`VERIFIED:` / `NOT VERIFIED` / `UPLOAD FAILED`, `running (` / `RUN COMPLETE: <s> s` (regex
`UI53:5956`) / `RUN NOT COMPLETE` / `NOT STARTED`, `SEQUENCE PROGRAM i/n: slot S (name)`,
`SEQUENCE RUNNING`, `SEQUENCE PROGRAM COMPLETE i/n: <s> s`, `SEQUENCE SETTLE`, `SEQUENCE COMPLETE:`,
`warning-only arm code`, `AUX POLE|HAND|WAIT ...`, `ARM RUN MODE:`, `ARM MODE SET COMPLETE:`,
`** ARM MODE`, `DRAG TEACH ACTIVE:` / `DRAG TEACH STOPPED:` / `DRAG SAVE COMPLETE:` /
`DRAG ORIGIN COMPLETE:` / `DRAG PLAYBACK START:` / `DRAG PLAYBACK COMPLETE:` / `** DRAG`,
`MANUAL HAND|POLE COMPLETE:` / `** MANUAL ... FAILED`, `GLOBAL POINT: {json}`, `GLOBAL MOVE ...`,
`GLOBAL SEQUENCE ...`, `WORKFLOW ...`, and the `--list` row `slot  NN  x KB  speed S  name`.
The first line of every run is the SDK's own `current c api version:  1.1.6` (`SDKI:7111-7113`,
`README:46`). A Lite backend that wraps the runner must parse exactly these tokens; a library
refactor should replace them with structured events.

## 2. Connection

* SDK load order: `--sdk DIR` -> an installed `Robotic_Arm` -> `Path(__file__).parents[1]/external/butterfli-realman-alix/Python` (`RUN:60-73`). The last fallback assumes the ALIX repo layout (`tools/`);
  from `upload_channel/` it resolves to `/home/newtonjeri/alix_studio_lite_ws/external/...`, which does
  not exist (`ls`), so **`--sdk` is mandatory here**. `README:7-14` tells the user to `export ALIX_RM_SDK`,
  but the runner itself never reads that variable (only the UI does, `UI53:48`).
* `arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)` (`RUN:2608`). The constructor calls the global
  `rm_init(mode)` and prints the C API version (`SDKI:7101-7113`). Triple mode = sender + receive-queue
  thread + UDP-push thread (`SDKI:7104-7108`; enum `SDKW:927-935`, value 2). ALIX notes that in single
  mode the `block` argument becomes a timeout in seconds, so every production path uses triple mode
  (`ALIX/core/python/alix_dispatch/src/alix_dispatch/connect.py:69-72`).
* `handle = arm.rm_create_robot_arm(ip, a.port, 3)` (`RUN:2609`): the third argument is the SDK **log
  level** (3 = error), not a timeout or retry count (`SDKI:7115-7130`). On success the SDK also reads
  `rm_get_robot_info` for DOF/controller generation (`SDKI:7139-7157`); failure is `id == -1`; the runner
  exits "cannot connect to the controller at ip:port" when `id <= 0` (`RUN:2610-2611`).
* `rm_set_timeout(1500)` when available (`RUN:2613-2614`): the reply timeout, process-global, vendor
  default 500 ms (`SDKI:7185-7191`). ALIX's rationale for 1500: a controller loading a 4000-point
  program, streaming, or in a Modbus transaction answers late, the SDK prints `revice error` and returns
  -2, which ended two 12-task flows on 2026-09-04 (`ALIX/core/python/alix_dispatch/src/alix_dispatch/connect.py:36-49`).
* Defaults: LEFT `192.168.1.10`, RIGHT `192.168.1.103`, overridable by env `RM_LEFT_IP`/`RM_RIGHT_IP`
  (`RUN:47-48`); port `8080` (`RUN:49`); `--ip`/`--port` override (`RUN:2285-2286,2347`).
* Lifetime: **one process = one TCP connection = one action**, closed in `finally:
  arm.rm_delete_robot_arm()` (`RUN:2697-2698`). There is **no reconnect, no retry, no heartbeat, no
  connect timeout knob** (whole-file read). "Persistent connection" only means "for the duration of one
  sequence/workflow process" (`RUN:483`, `2000-2004`).

## 3. Upload — `.txt` -> controller program

### 3.1 The exact path (`upload`, `RUN:147-175`; identical to `ORIG:146-174`)
1. `read_track` (`RUN:76-88`): splits on CR/LF, every non-empty line must be `{"point":[7 ints]}`;
   one bad line exits the whole file; fewer than 2 points refused. **No joint-limit, step or velocity
   check.**
2. `write_project` (`RUN:91-97`): `{"file":7}` CRLF + `{"name":"Folder","num":1,"type":<N points>,
   "enabled":true,"parent_number":0}` CRLF + the original point lines joined by CRLF + trailing CRLF
   (the controller rejects LF-only: docstring `RUN:18-19`; hardware evidence `err_line=1`,
   `ALIX/core/python/alix_dispatch/src/alix_dispatch/track.py:85-98`). Written to a temp dir as
   `<track stem>.proj`.
3. `slots(arm)` lists what the controller holds; the target slot is shown as "empty" or
   "<name> (x KB) — it will be REPLACED"; `input("write it? [y/N]")` unless `--yes` (`RUN:148-158`).
4. **`arm.rm_send_project(rm_send_project_t(str(proj), a.speed, 1, a.slot, 0, 0, 0))`** (`RUN:159`).
   Positional fields (`SDKW:2269-2323`): `project_path` (c_char*300), `plan_speed` = `--speed` (0),
   `only_save` = **1 (store only, never run)**, `save_id` = slot, `step_flag` = 0, `auto_start` = 0 (not
   the IO default program), `project_type` = 0 (online-program file, not "drag teach trajectory file").
5. Success test `ret == 0 and err_line == -1` (`RUN:160-164`); `SEND_RET` names -1..-6 (`RUN:51-55`).
   Note: the SDK wrapper returns `err_line = -1` whenever `ret == 0` (`SDKI:4813-4815`), so the
   "data length check failed / refused line N" branch for `ret == 0` (`RUN:161-162`) is dead code; the
   line number is only meaningful when `ret != 0` (it is printed).
6. **VERIFY** by reading the program list again: the slot must be listed and its stored `size` must be
   within `max(64 B, sent // 1000)` of the bytes sent (`RUN:165-174`), else "NOT VERIFIED". The name is
   printed, not checked.

### 3.2 Program list and IDs
* `slots()` reads `rm_get_program_trajectory_list(page, 10, "")` page by page until a short page or a
  page with nothing new, pages 1..11 (`RUN:102-119`). Page size 10 because one page of 100 with ~70
  programs overflowed the SDK's fixed buffer ("Error parsing JSON", ret -3, 2026-09-23) (`RUN:99-101`).
  List items: `id`, `size`, `speed`, `trajectory_name` (`c_char * 32`, i.e. **32 bytes**) (`SDKW:2338-2356`, `2383-2436`).
* Program IDs are 1-100 everywhere (`RUN:2354-2355,2366-2367,323-325,1031-1034,1677-1678`).
* Overwrite: by `save_id` — the runner never calls `rm_delete_program_trajectory` (`SDKI:4921`) first
  **(inferred: overwrite-in-place works, as the README session and ALIX fakes model it)**.
* Stored name: the controller strips the extension, prepends `<id>_` and keeps 32 bytes, e.g.
  `85_FILE_01.1.txt` (`README:51`); `70_70_commode_c_toplid_right_exe` (exactly 32)
  (`ALIX/tools/cleaning_tracks.py:838-862`, `ALIX/core/python/alix_dispatch/src/alix_dispatch/emulated_program.py:81-84,335-341`).
* Slot conventions elsewhere: ALIX reserves **70-100 for its production cleaning tracks**, 40-59 for
  taught tracks, 21-39 and 61-69 for experiments (`ALIX/tools/cleaning_tracks.py:2769,2870`;
  `ALIX/Resource/tracks/taught/README.md:164-169`). The control app's production V6 workflow also lives
  in **71-83** (`WFL`, `WFR`) on the same two controllers — a direct ownership collision for Lite to
  resolve (see gaps). The UI defaults the upload slot to 25 (`UI53:6933-6934`).

### 3.3 What "fast" means (and does not mean)
* The **upload itself is not faster** than `ORIG`: same writer, same `rm_send_project`, same two list
  reads (diff `ORIG` vs `RUN:1-268`).
* "fast" = sequencing over one connection: v4 `--sequence` "fast multi-run, one connection"
  (`RUN:7`, `run_sequence` reads the list once, `RUN:423`). v12 "FAST PROGRAM SWITCH"
  (`RUN:1986-1998`): program list read once, no nested `run_sequence()` per program, no speed restore
  between programs, 50 ms polling (`RUN:1938`), and `workflow_quick_settle` (4 deg/s, 2 calm samples at
  50 ms, 0.6 s timeout before another Program, 1.0 s before pole/hand/global, `RUN:1766-1804,1947-1955`)
  instead of `settle()` (2 deg/s, 3 calm samples at 0.1 s, up to 5 s, `RUN:178-186`). In v11 every
  Program step re-entered `run_sequence` (list read + settle + restore each time,
  `diff HIST/rm_upload_fast_v11_workflow_sequence.py HIST/rm_upload_fast_v12_fast_workflow.py`). Feature
  note wording: `FEAT:175-180`.
* Measured times: none in runner comments (only dated incidents, `RUN:101,125`). Elsewhere: README
  session 12,429 points / 712.8 KB uploaded and verified (no time) (`README:45-51`); ALIX: "Upload
  already costs 6.4 s for 290 KB" (`ALIX/docs/motion/MOTION_TRANSPORT_PLAN.md:210`) ≈ 45 KB/s.
  Sample files are ~58 B/point (425,057 B / 7,353 lines for `upload_channel/zip/01...idle_trimmed.txt`;
  85,047 B / 1,471 lines for the `_20pct` file). **(inferred)** at ~45 KB/s that is ≈9 s vs ≈2 s per
  upload — point count is the dominant upload-time lever.
* Upload overhead in the UI's batch mode (`UI53:4504-4660`): per file one `--list` process, then one
  upload process (which itself reads the list twice), plus one final `--list` -> 2N+1 processes /
  connections and 3N+1 full paged list reads for N files **(inferred count from code)**.

### 3.4 R13 options found in ALIX (reference)
* Skip uploads that are already in place: ALIX pins every track by `track_sha256`/`proj_sha256`/
  `proj_bytes` and preflights the controller's stored size and `trajectory_name` before trusting a slot
  (`ALIX/tools/cleaning_tracks.py:29-43`, `4046-4110`; driver pins "second run of a task pins",
  `ALIX/docs/motion/DRIVER_EXECUTION_CONTRACTS.md:580-584`).
* One session for many files: `taught_tracks.py batch ... --upload --run` "certify all, then one
  session" (`ALIX/Resource/tracks/taught/README.md:24-37`, batch line 34-35).
* Fewer points: the controller plays one point per fixed ~5.55 ms tick, so re-spacing points both
  speeds the motion and shrinks the upload (`ALIX/docs/motion/TRACK_CHANNEL_TIMING_REQUIREMENT.md:13-17`;
  certify ladder `ALIX/Resource/tracks/taught/README.md:40-75`).
* Upload timing instrumentation: `alix_dispatch.track.send_project` returns `(ret, err_line, seconds)`
  (`ALIX/core/python/alix_dispatch/src/alix_dispatch/track.py:101-115`).

## 4. Run — stored program execution, completion, speeds, stop, error codes

### 4.1 Single run (`run`, `RUN:189-269`)
* Gates: slot listed (`RUN:190-193`), arm state readable (`194-197`), **any** error code refuses —
  including warning-only 4105 (`198-200`), program run state must not be 1/2 (`201-205`).
* First-point gap: when the track is known (upload + `--run`), prints the largest joint gap between the
  current pose and point 0 ("keep that path clear" if > 1 deg) (`RUN:207-214`).
* Typed confirmation: the user must type the side (or `run`) unless `--yes` (`RUN:215-218`).
* **Start:** `arm.rm_set_program_id_run(slot, a.run_speed, 0)` — 0 = non-blocking in multi-thread mode
  (`RUN:223`; `SDKI:4857-4882`: speed 1-100, "0 = run at the stored speed"; return -4 = "stopped
  externally", never seen in non-blocking use).
* **Completion detection by polling** `rm_get_program_run_state()` every 0.1 s: `run_state` 1/2 = started
  (1 running, 2 paused), then 0 = ended (`RUN:228-237`; struct `SDKW:2437-2465`). Not started within 5 s
  -> fail (`238-240`); `--timeout` (300 s) -> slow stop (`241-244`); arm error codes polled every 0.5 s
  (`245-251`). The vendor also pushes an unsolicited `program_run_finish` event on the socket
  (`ALIX/docs/motion/DRIVER_TRACK_CHANNEL_ARCHITECTURE.md:18-27`; SDK `rm_get_arm_event_call_back`
  `SDKI:7256`) — not used. **(inferred)** `plan_num` will not give per-point progress: ALIX's emulator
  models it as constant 1 while running (`emulated_program.py:46-49`).
* `finally`: slow stop if stopped, `settle()` if started, then **always** `rm_set_plan_speed(--restore-speed 50)`
  (`RUN:255-262`). Verdict: complete only if ended, not stopped, and no non-4105 code seen
  (`RUN:263-269`).

### 4.2 Speeds — what each one really does
* Stored playback timing is fixed by point spacing: the controller plays **one line per tick of a fixed
  clock, 5.5545 ms nominal (5.4-6.2 measured)** (`ALIX/docs/motion/TRACK_CHANNEL_TIMING_REQUIREMENT.md:13-17`);
  "plan_speed scales the transit only" across a taught file, a re-spaced file and a MoveIt plan
  (`ALIX/experiments/exp01_teach_trajectory_reverse_engineering/notes/2026-08-27_canfd_matrix_runsheet.md:383-390`).
* **Transit speed** = the `speed` argument of `rm_set_program_id_run`: it scales only the controller's
  own joint move to point 0 **and becomes the controller-wide global ratio, which survives power cycles**
  (`ALIX/docs/motion/MOTION_TRANSPORT_PLAN.md:156-157`; F98 "a movej ran ~6x slower after a speed-10
  program; there is no getter", `emulated_program.py:36-42`). Then the player dwells ~0.35 s before
  streaming (`emulated_program.py:40-42,80`). Runner defaults: `--run-speed 20` for `run`/`run_sequence`
  (`RUN:2337-2339`); **v14 workflow `transit_speed` default 30** (`RUN:1680-1687`), UI default 30
  (`UI53:6093`, legacy JSON upgraded to 30 on load `UI53:6835-6851`).
* **Overall speed** (v13+) = `rm_set_plan_speed(--workflow-overall-speed, default 100)` once before the
  workflow (`RUN:2115-2131`) and again before each Program (`RUN:1855-1864`). **(inferred)** Because the
  run call immediately sets the same global ratio to the transit speed, the overall value mainly affects
  later non-program moves; the feature note's claim that it "helps prevent slow travel" (`FEAT:182-187`)
  is not supported by ALIX's measurements (see open questions). Global-point moves after a Program step
  may therefore run at `speed x last transit ratio` **(inferred from F98)**.
* **Restore speed**: `rm_set_plan_speed(--restore-speed 50)` after `run` (`RUN:260`), after the whole
  `run_sequence` (`RUN:740-752`, call at 748) and once after the whole workflow (`RUN:2263-2277`). Docstring: "the run
  speed sets it, it survives power cycles, and nothing can read it back" (`RUN:32-33`). The SDK's run
  state does expose a `plan_speed` field ("global planning speed ratio 1-100", `SDKW:2451`, `2463`) —
  unverified as a read-back.
* **Stored speed** (`--speed` -> `plan_speed` in `rm_send_project_t`) is always 0 and never used, because
  the runner always passes a non-zero run speed (`RUN:2287-2289`, `223`, `629`, `1873`).

### 4.3 Multi-run and workflow
* `run_sequence` (`RUN:393-753`): refuses missing slots up front (`423-428`), blocking (non-4105) codes,
  non-idle controller (`430-457`); loads Hand/Pole only if any step needs them (`459-481`); per step:
  re-check arm + idle (`515-542`) -> Pole (`546-572`) -> Hand (`574-599`) -> Wait (`601-609`) -> re-check
  (`611-624`) -> `rm_set_program_id_run(slot, a.run_speed, 0)` (`629`) -> poll (`641-676`) -> slow stop on
  timeout (`678-682`) -> `settle()` (`688-691`) -> fail on any non-4105 code (`693-697`).
* `run_workflow_sequence` (`RUN:1986-2277`): same up-front gates (`2011-2063`), prints the plan, typed
  confirmation unless `--yes` (`2104-2113`), sets overall speed (`2115-2131`), then per step:
  `run_workflow_program_fast` (`1807-1983`), `manual_pole_control`, `manual_hand_raw_control` (opens and
  closes Modbus each time), a 50 ms-sliced wait, or `move_global_waypoint` (`2133-2229`). Ctrl+C ->
  slow stop (`2248-2261`); `finally` restores the global speed (`2263-2277`).

### 4.4 Stop and error codes
* Stop = `rm_set_arm_slow_stop()` ("stop on the currently running trajectory", `SDKI:2069-2082`) on
  Ctrl+C/SIGINT or watchdog (`RUN:256-257,679,731,1921,2254`). Other vendor stops not used:
  `rm_set_arm_stop` (fastest, unrecoverable, `SDKI:2084`), `rm_set_arm_pause`/`rm_set_arm_continue`
  (`SDKI:2099,2114`), `rm_set_arm_emergency_stop` (`SDKI:7226`). **There is no program-scoped stop/pause
  API** (`ALIX/docs/motion/DRIVER_TRACK_CHANNEL_ARCHITECTURE.md:23-27`) and **whether slow_stop aborts a
  RUNNING uploaded program has never been tested on hardware (gate G4)**
  (`emulated_program.py:50-55`; `ALIX/Resource/reports/2026-10-01_remaining/T4_verdicts/show_final.txt:7`).
* Error codes: `arm_state()` converts the SDK's decimal TEXT codes to ints and drops zeros (a "0" kept as
  text read as an error, 2026-09-23) (`RUN:122-140`; SDK builds `f"{err[i]}"` strings `SDKW:1826-1850`).
  `KNOWN_ERR = {4105: joint speed over its limit (warning; run goes on), 4109: ARM COLLISION}`,
  `WARN_ONLY = {4105}` (`RUN:56-57`). 4105 = `0x1009` speed limit, 4109 = `0x100D` collision, 4114 =
  `0x1012` self-collision, 4115 = `0x1013` electronic fence (`ALIX/docs/hardware-sdk/ERROR_CODES.md:46,50,55,56`)
  — 4114/4115 are not named by the runner (still blocking, printed raw).

## 5. Drag & teach

* Start: `arm.rm_start_drag_teach(1)` (1 = record trajectory, `SDKI:3604-3620`) (`RUN:923`); the process
  then idles in `while True: time.sleep(0.2)` (`RUN:932-933`) while the operator guides the arm.
* Stop: the UI sends SIGINT (same button pressed again, `UI53:3880-3902`); `KeyboardInterrupt` ->
  `rm_stop_drag_teach()` -> "DRAG TEACH STOPPED: recorded trajectory retained in controller"
  (`RUN:935-943`); `finally` safety stop if anything else ends the process (`RUN:945-954`).
* Save: a **separate process/connection** runs `arm.rm_save_trajectory(str(path))` -> `(ret, point_count)`
  (`RUN:957-989`; `SDKI:3873-3890`). The UI default path is
  `~/Downloads/RealMan_DragTeach/<side>_drag_<YYYYmmdd_HHMMSS>.txt` (`UI53:3840-3849`),
  i.e. no region tag. ALIX observed that the save lags and needs a retry per second
  (`ALIX/experiments/exp01_teach_trajectory_reverse_engineering/notes/2026-08-25_phase_a.md:92-100`) —
  the runner tries once.
* File format: NDJSON, one `{"point":[j1..j7]}` per sample, integer millidegrees (inferred units), no
  timestamps, no frame (`ALIX/experiments/exp01_teach_trajectory_reverse_engineering/src/teachlab/teachfile.py:1-21`;
  sample `upload_channel/zip/01. lid_top_slow_01_idle_trimmed.txt`, LF-only). Record period unknown
  ("no vendor source states it") (`notes/2026-08-25_phase_a.md:99-100`).
* Playback of the latest recording: `rm_drag_trajectory_origin(1)` (blocking move to the recorded
  start at 20 %, `SDKI:3722-3745`) then `rm_run_drag_trajectory(1)` (blocking replay) (`RUN:1173-1210`);
  Ctrl+C -> `rm_stop_drag_trajectory()` + slow stop (`RUN:1198-1209`). Gen-3 replays only the
  controller's LAST drag recording; file-based replay is Gen-4 only (`ALIX/experiments/exp01_teach_trajectory_reverse_engineering/README.md:214-222`).
  All joints must be enabled before a drag session (F21a, `notes/2026-08-25_phase_a.md:98`).
* Drag file -> controller program (v7, not used by UI v53): `drag_program_name` builds `DT_<safe name>`
  cut to 56 chars + `.txt` (`RUN:992-1015`); `upload_drag_txt_program` = `read_track` + `write_project`
  + `rm_send_project(..., only_save=1, project_type=0)` + list verify, then, if the stored name differs,
  `rm_update_program_trajectory(slot, speed, name)` (`RUN:1018-1150`; `SDKI:4939-4957`). **(inferred)**
  With the controller's `<id>_` prefix and 32-byte field the names always differ, so the rename always
  fires and is truncated again; the UI detects drag programs with `"DT_" in name` (`UI53:1371`, `4739`).
* Dual drag: the UI starts both arms' `--drag-session` behind a `threading.Barrier(2)` (`UI53:7841-7960`)
  and offers one popup to save both files (`UI53` `dual_drag_save_*` state, `UI53:6953-6958`).

## 6. Global points, hand, pole, dual-arm synchronisation

### 6.1 Global waypoints (controller-stored named poses)
* List: `rm_get_global_waypoints_list(page, 20, "")` pages 1..100 with tolerant key search
  (`RUN:1265-1369`); printed as `GLOBAL POINT: {json}` (`RUN:1372-1391`).
* Read one: `rm_get_given_global_waypoint(name)`; requires >= 7 joints and >= 6 pose values
  (`RUN:1394-1417`).
* Move: MoveJ = `rm_movej(joints, speed, 0, 0, 1)`; MoveL = switch to the point's work/tool frame
  (`rm_change_work_frame`/`rm_change_tool_frame`) then `rm_movel(pose, speed, 0, 0, 1)`, restoring the
  original frames in `finally` (`RUN:1467-1558`; `SDKI:1460-1497`). Both are **blocking** (`block=1`).
* Sequence: validates every point first, then moves in order (`RUN:1615-1665`). The SDK can also add /
  update / delete global points (`rm_add_global_waypoint` etc., `SDKI:5001-5054`) — unused, relevant to R7.

### 6.2 Hand — Inspire RH56 over the end-port Modbus RTU (`HAND`)
* Port 1 (end interface board RS485), device 1, 115200 baud; registers ANGLE_SET 1486, FORCE_SET 1498,
  SPEED_SET 1522, ANGLE_ACT 1546; 6 channels in order little, ring, middle, index, thumb_bend,
  thumb_rot; 0-1000 with 1000 = open (`HAND:3-17`).
* `open()` = `rm_set_modbus_mode(1, 115200, 2)` (timeout in 100 ms units, `SDKI:4095-4108`) + SPEED_SET
  1000; `close()` = `rm_close_modbus_mode(1)` (`HAND:49-57`).
* Writes are one bulk `rm_write_registers` of all six (~63 ms; <= 10 registers per write,
  `SDKI:4299-4317`) with a per-register fallback `rm_write_single_register` (`HAND:59-69`); reads use
  `rm_read_multiple_holding_registers` (2 < num < 13, `SDKI:4366-4383`) (`HAND:71-75`).
* `command(counts)` holds `None` channels at their read-back value, computes a stroke law
  `0.115 s + 373 * span / speed / 1000` (measured), and returns when within 15 counts (arrived) or two
  reads within 3 counts after the law time (settled on force); timeout `3 x law + 1 s` (`HAND:88-121`).
  `freeze()` re-writes the current angles — "the SDK has no hand stop" (`HAND:123-126`).
  `fingers_pct_counts(pct)` = four fingers at pct x 10, thumbs held (`HAND:38-40`).
* Runner usage: `parse_hand_target` (percent or six counts) (`RUN:274-315`), `manual_hand_control`
  (`RUN:756-801`), `manual_hand_raw_control` (`RUN:804-862`), sequence hand step (`RUN:574-599`).
* ALIX hardware findings: hand arrival events never fire and the SDK's blocking hand call returns -4
  during arm motion; Modbus RTU is the true-feedback path; hand traffic during arm motion is safe
  (`ALIX/docs/PHASE_PLAN.md:117-118`, F4/F5).

### 6.3 Pole (lift) (`POLE`)
* `height_m()` = `rm_get_lift_state()["pos"]` mm / 1000 (`POLE:22-24`; struct `SDKW:2217-2240`:
  `pos`, `current`, `err_flag`, `mode` 0 idle).
* `move(h, speed_pct=100, block=False)` = `rm_set_lift_height(speed, mm, 0)` then polls at 50 ms until
  idle **and** within 5 mm **and** the modelled travel time has elapsed ("the getter leads the real pole
  by up to 73 mm"); raises on `err_flag`; on timeout (3 x expected) calls `stop()` =
  `rm_set_lift_speed(0)` (`POLE:29-58`; `SDKI:4652-4714`).
* Measured constants: stroke 315 mm, slew 0.059 m/s up / 0.097 m/s down at 100 % (vendor 0.185 never
  reached), final creep 1.5 s (`POLE:3-14`). Runner range 0-315 mm (`RUN:331-335`, `1703-1706`,
  `2459-2460`); the SDK accepts 0-2600 mm (`SDKI:4673-4690`).
* `manual_pole_control` (`RUN:865-909`) does **not** check that the arm is still; `usage.py` does
  (`arm_at_rest`, 1.6 deg/s for 6 polls, `USAGE:44-72`) and documents the rule "the pole moves only with
  the arm still; never move pole or hand with the pad on a surface" (`USAGE:25`). ALIX F9 notes lift and
  planned arm moves can abort each other (`ALIX/docs/PHASE_PLAN.md:114`).
* `usage.py prestroke` encodes the R7 order: hand open -> arm still -> pole -> hand to the taught state
  (`USAGE:128-138`); `stop_all` = arm slow stop + pole stop + hand freeze (`USAGE:141-150`).

### 6.4 Dual-arm synchronisation
* The runner has **no cross-arm logic**; each arm is a separate process/connection. Synchronisation is
  UI-side: `_start_process_synced` waits on a shared `threading.Barrier(2)` (8 s timeout) immediately
  before `Popen` (`UI53:5654-5709`), used by dual drag (`7934`), run both uploaded `.txt` (`8926`),
  Page-02 rule stages (`10209`, `10586`) and run both programs (`11343`).
* **(inferred)** Only process launch is aligned; each runner then loads the SDK, connects, reads the
  program list and checks state before `rm_set_program_id_run`, so motion-start skew is unmeasured.
  ALIX also lists dual-arm start skew as unmeasured (`ALIX/docs/motion/MOTION_TRANSPORT_PLAN.md:220-221`)
  and uses a nominal ±1.6 s barrier-release skew window in its pair verdicts
  (`ALIX/Resource/reports/2026-10-01_remaining/T4_verdicts/show_final.txt:23`).

## 7. SIMULATION / REAL / CHECK mode — controller run mode, not a 3D simulation

* Buttons SIM / REAL / CHECK per arm (`UI53:2080-2102`); REAL asks for confirmation (`UI53:3796-3816`);
  CHECK = `--get-arm-mode` (`UI53:3785-3794`); result shown as `MODE // SIMULATION|REAL` parsed from
  `ARM RUN MODE:` / `ARM MODE SET COMPLETE:` (`UI53:5878-5890`). Feature note: `FEAT:90-96`.
* Runner: `get_arm_run_mode` = `rm_get_arm_run_mode()` (`RUN:1213-1226`); `set_arm_run_mode` =
  `rm_set_arm_run_mode(mode)` then **verifies by read-back** and fails if the controller reports a
  different mode (`RUN:1229-1262`). SDK: "set real/simulation mode, 0 = simulation, 1 = real"
  (`SDKI:7193-7224`).
* **It is the controller's own simulation mode** — not Gazebo, not ROS, not a 3D view in the app
  (`ALIX/core/python/alix_dispatch/src/alix_dispatch/runmode.py:1-12`: "SIM is the CONTROLLER's own
  simulation mode ... SIM and REAL both require hardware access"). What it does physically:
  * the controller runs its planner and "executes" the arm virtually — joint state, UDP push, arrival
    events and TCP polls all report the simulated motion; the pendant/Web GUI animates its 3D model;
    the motors do not move and draw no load (`ALIX/docs/PHASE_PLAN.md:115` F2;
    `ALIX/docs/TEST_PROCEDURE.md:189-193`; F26 "SIM mode draws no load", `ALIX/docs/PHASE_PLAN.md:139`);
  * **lift and hand are not simulated** ("Sim mode does not execute lift or hand",
    `ALIX/docs/PHASE_PLAN.md:116`; `ALIX/docs/TEST_PROCEDURE.md:337-338`). **(inferred)** pole steps in a
    SIM workflow will therefore time out or behave differently; must be tested;
  * the controller's own collision models still run and can stop a SIM run with 4109 although nothing
    touched anything (F26, caused by a mis-configured tool payload, `ALIX/docs/PHASE_PLAN.md:139`;
    `ALIX/docs/hardware-sdk/ERROR_CODES.md:137`); the electronic fence works only in SIM mode
    (`ALIX/core/python/alix_taskgen/GENERATION_FINDINGS.md:3575-3581`); the controller has **no model of
    the commode** (F26 text);
  * the six-axis-force drag-teach variant refuses to start in SIM mode (`SDKI:3637-3661`, failure
    reason at `SDKI:3656`) **(inferred: plain drag teach is also a REAL-mode activity)**.
* The setter can fail silently ("a SIM request that silently stayed REAL would move real metal"), hence
  verify by read-back — which the runner does (`ALIX/core/python/alix_dispatch/src/alix_dispatch/runmode.py:81-86`).
* The ALIX emulator stores the flag but motion is identical in both modes
  (`ALIX/docs/EMULATOR.md:96-98`).
* Consequence for Lite: "Sim vs Real" in R3/R4/R10/R11 cannot be served by this toggle alone. Lite needs
  (a) a 3D, robot-free simulation (URDF + meshes + collision scene, from ALIX Studio), and optionally
  (b) the controller dry-run (SIM mode) as a pre-flight on real controllers, labelled as such.

## 8. Safety measures and failure handling

What the runner does (all with evidence above):
1. Whole-file track validation; never sends half a file (`RUN:76-88`).
2. Store-only uploads (`only_save=1`, `auto_start=0`) and replacement warning + confirmation
   (`RUN:148-159`); verify by list read-back with size tolerance (`RUN:165-174`).
3. Pre-run gates: slot exists, arm state readable, no blocking error, controller idle
   (`RUN:189-205`, `423-457`, `1823-1853`, `2011-2063`); per-step re-checks in sequences (`RUN:515-542`,
   `611-624`).
4. First-point gap warning (`RUN:207-214`) and typed-side confirmation (`RUN:215-218`, `496-504`,
   `2104-2113`), replaced by UI dialogs + `--yes` in practice.
5. Watchdogs: 5 s start, `--timeout` 300 s run (-> slow stop), error polling every 0.5 s.
6. Ctrl+C/SIGINT -> slow stop, pole stop (`rm_set_lift_speed(0)`), hand freeze (`RUN:714-738`); settle;
   global speed restored in `finally` on every run path (`RUN:255-262`, `740-752`, `2263-2277`).
7. Run mode verified by read-back (`RUN:1244-1262`); MoveL frames restored (`RUN:1556-1558`); drag
   teach stopped in `finally` (`RUN:945-954`); Modbus closed in `finally` (`RUN:795-800`, `857-862`).
8. 4105 (joint speed over limit) is warning-only during sequences/workflows; any other code fails the
   step (`RUN:56-57`, `693-697`, `1960-1970`).

Gaps and failure modes (for Lite to fix or at least not regress):
* **Stop cannot interrupt blocking SDK calls (inferred from CPython signal semantics):** the
  `KeyboardInterrupt` is only raised when control returns to Python, so a Slow Stop during
  `rm_movej/rm_movel(..., block=1)` (`RUN:1498,1522`), `rm_drag_trajectory_origin(1)` or
  `rm_run_drag_trajectory(1)` (`RUN:1182,1190`) takes effect only after that call returns. Program runs
  and pole moves are non-blocking and stop promptly.
* **Windows:** `CTRL_BREAK_EVENT` is delivered as SIGBREAK, which Python does not map to
  `KeyboardInterrupt`; the runner has no handler, so the process would die without `rm_set_arm_slow_stop`
  **(inferred)** (`UI53:6058-6059`).
* Whether `rm_set_arm_slow_stop` aborts a running stored program is untested on hardware (G4, §4.4).
* No reconnect/retry; any SDK `-2` (late reply) fails the action; `sys.exit` inside `read_track`/`slots`
  ends the process (`RUN:83-87,112`).
* No joint-limit / step (<= 10 deg/line) / rate (180 deg/s J1-J2, 225 deg/s J3-J7) validation before
  upload (`ALIX/docs/motion/TRACK_CHANNEL_TIMING_REQUIREMENT.md:27-38` lists those hard limits).
* "Did not start within 5 s" breaks without a slow stop (`RUN:238-240`, `1909-1913`); a paused program
  (state 2) is treated as running until the 300 s watchdog (`RUN:231`) **(inferred)**.
* `workflow_quick_settle` failure is ignored (only a log suffix, `RUN:1977-1981`), and pole steps do not
  check that the arm is still (`RUN:2174-2178`) **(inferred risk)**.
* Single `run()` refuses on 4105 present before start while sequences allow it (`RUN:198-200` vs
  `436-451`) — inconsistent.
* Mode switch does not check the arm/program is idle (`RUN:1229-1262`; the UI only checks that its own
  panel has no active command, `UI53:3796-3799`).
* Controller slot ownership: nothing stops the app overwriting ALIX's 70-100 production slots (or vice
  versa) except the replace warning (§3.2).

## 9. Reuse assessment

### 9.1 Can a Lite backend import `RUN` as a module?
Technically yes, practically only with shims:
* Import has no side effects beyond constants (`ARM_IPS` reads env at import, `RUN:47-48`).
* `upload` and `upload_drag_txt_program` use the module global `rm_send_project_t`, which is only bound
  inside `main()` via `global rm_send_project_t` (`RUN:2606-2607`; uses `RUN:159,1065`). An importer must
  set `module.rm_send_project_t` after `load_sdk`.
* Functions take the argparse `Namespace` `a` and read attributes from it: `upload` (track, slot, yes,
  speed), `run` (run_speed, side, yes, timeout, restore_speed), `run_sequence` (side, yes, run_speed,
  timeout, restore_speed), `upload_drag_txt_program` (yes, speed, run_speed), workflow functions
  (workflow_overall_speed, timeout, side, yes, restore_speed) (`grep a.<attr> RUN`). A
  `SimpleNamespace` works.
* Output is `print()` only (no return data, return codes 0/1/130); `input()` prompts unless `yes`;
  `sys.exit` on parse/list errors (`RUN:73,83,85,87,112`).
* **Cancellation depends on `KeyboardInterrupt` in the main thread.** A web server running these
  functions in a worker thread cannot deliver a signal there, so stop would not work without a refactor
  (stop token checked in the polling loops + `rm_set_arm_slow_stop()` issued directly).
* Every call re-reads the program list (`slots()`); `load_aux_classes` requires `hand.py`/`pole.py` in
  the runner's own directory (`RUN:365-390`).

### 9.2 Recommended approach
1. **MVP (lowest risk):** keep runner v14 + `hand.py` + `pole.py` byte-for-byte, call it as a subprocess
   exactly like `UI53` (`-u`, own session, SIGINT for stop), add `--ip/--port` from the Lite connection
   form (R4), confirm in Lite then pass `--yes`, and translate the §1.4 stdout tokens into UI events.
   Pass an explicit `--sdk` (the repo-copy fallback is broken here).
2. **Target:** extract the SDK-calling core into a small library (e.g. `lite_rm/`): `ArmSession`
   (persistent per-arm connection, reconnect, `rm_set_timeout(1500)` once — it is process-global),
   `ProgramStore` (list/upload/verify/rename/delete, batch in one session, sha/size pins to skip
   unchanged uploads), `ProgramRunner` (non-blocking run + stop token + event callbacks + joint
   snapshot on error codes), `DragTeach` (start/stop/save with retry/playback), `RunMode` (get/set +
   verify), `Workflow` (port of `run_workflow_sequence`), reusing `Hand`/`Pole` unchanged. Keep the CLI as
   a thin adapter so UI v53 keeps working, and prove equivalence with ALIX's fake-controller tests
   (`ALIX/core/python/alix_dispatch/test/test_rm_upload.py:1-212`, which already monkeypatches
   `load_sdk` and drives `main()`) and `emulated_program.py` (slot store, 5.5545 ms clock, global ratio,
   both stop policies).
3. Alternative/complement: ALIX's `alix_dispatch` already implements the same semantics as a library
   (`track.build_project_file`, `track.send_project`, `track.program_list`, `connect.set_ack_timeout`,
   `runmode`) (`ALIX/core/python/alix_dispatch/src/alix_dispatch/track.py:85-140`,
   `connect.py:36-66`, `runmode.py:1-110`) — candidate to share rather than fork.

### 9.3 Behaviour that must be preserved exactly (hardware safety)
* Project bytes: `{"file":7}` + Folder line with `type = point count` + CRLF everywhere (`RUN:91-97`).
* `rm_send_project_t(path, 0, only_save=1, save_id, step_flag=0, auto_start=0, project_type=0)`
  (`RUN:159`, `1064-1075`) — never `only_save=0` (that runs immediately) or `auto_start=1`.
* Whole-file validation (`RUN:76-88`) and list-read-back verification within `max(64, sent//1000)`
  (`RUN:165-174`), with `PAGE = 10` list paging (`RUN:99-119`).
* Error-code parsing from decimal text, zeros dropped; `WARN_ONLY = {4105}`; everything else blocks
  (`RUN:56-57`, `122-140`).
* Pre-run gates (slot present, state readable, no blocking codes, controller idle), non-blocking
  `rm_set_program_id_run(slot, 1..100, 0)`, 5 s start check, watchdog -> slow stop, error poll every 0.5 s.
* Always restore the global speed after any run/sequence/workflow (it persists across power cycles)
  (`RUN:255-262`, `740-752`, `2263-2277`); v14 transit default 30 % for workflow programs
  (`RUN:1680-1687`).
* Run-mode change verified by read-back (`RUN:1244-1262`); `rm_stop_drag_teach` in `finally`
  (`RUN:945-954`); MoveL frames restored (`RUN:1556-1558`).
* Pole: 5 mm tolerance + modelled travel wait + 3x timeout + `rm_set_lift_speed(0)` stop (`POLE`);
  Hand: Modbus port/device/baud/registers, hold-by-read-back, freeze on cancel, close after use (`HAND`).
* `rm_set_timeout(1500)` (`RUN:2613-2614`).

## Requirement relevance

* **INT-CTRL** — Runner v14 (+ hand.py, pole.py) is the hardware layer of the control app; UI v53 is a
  pure subprocess front end over the CLI/stdout protocol (§1). Integrate by subprocess first, library
  second (§9.2). Latest pair = UI v53 + runner v14, not v48 (§0).
* **R1** — Drag teach primitives exist (`--drag-session`, `--drag-save`, `--drag-playback`, dual-arm
  barrier start) (§5); there is no region concept, no timer, no auto-advance, and saved files are named
  `<side>_drag_<timestamp>.txt` — Lite must add region tagging/registry for the dashboard.
* **R3** — Controller SIM mode is a controller-side dry run, not a 3D simulation (§7); 3D playback must
  come from ALIX Studio.
* **R4** — Real-arm testing: `--ip/--port` exist in the runner but the UI never exposes them (§1.1, §2);
  SIM/REAL toggle exists with read-back verification (§7); pause/continue APIs exist but are unused
  (§4.4). No reconnect logic (§2).
* **R5** — Collision on the real/SIM controller surfaces only as error codes polled every 0.5 s
  (4109; 4114/4115 unnamed) with elapsed time, no joint values (`RUN:245-251`); add a joint snapshot
  (the same `rm_get_current_arm_state` call returns joints) or UDP push (`SDKI:5492-5528`) to map the
  event onto the plot. Per-point progress is not available from run state (§4.1).
* **R7** — Pole (0-315 mm) and hand (six 0-1000 channels, presets GRIP `[0,0,0,0,150,1000]` / ALL OPEN)
  are reusable as-is; `usage.py prestroke` gives the safe order (hand open -> arm still -> pole -> hand);
  global points can store per-region start poses (§6). ALIX's taught-track certify records pole height
  and hand state per recording (`ALIX/Resource/tracks/taught/README.md:43-47`).
* **R8** — The runner does not merge `.txt` files; sequencing of separate programs (workflow) is its
  alternative (§1.3, §4.3).
* **R9 / R14** — Nothing in the runner; the SDK ships offline FK/IK (`Algo`, `rm_algo_forward_kinematics`,
  `rm_algo_inverse_kinematics`, `rm_algo_inverse_kinematics_rm75_for_arm_angle`, joint-limit/velocity
  checks, `SDKI:6147-6530`) usable without a robot; ALIX warns the algo IK can return ret 0 with a wrong
  solution, so FK-verify every IK result (`ALIX/docs/EMULATOR.md:92`).
* **R10 / R11** — Speed of a stored program is set by point spacing at ~5.55 ms/point; run/overall
  speeds only affect the transit (§4.2). 3D dual-arm sim is not in this code.
* **R12** — Runtime signal 4105 = joint speed over limit (warning); hard limits 180/225 deg/s and
  10 deg/line from ALIX (§8).
* **R13** — Upload path is unchanged from the original uploader; "fast" means one-connection
  sequencing and fast program switching (§3.3). Real levers: fewer points (≈45 KB/s measured), one
  session for batches, skip-by-pin, run both arms' uploads in parallel (inferred), avoid redundant list
  reads (§3.3-3.4).
* **NFR-UX** — Today's protections are dialogs + `--yes`, typed confirmations, terse stdout tokens and
  `MODE // ...` badges; Lite should present plain-language states mapped from §1.4 and never expose
  speeds/slots without defaults (transit 30 %, restore 50 %).
* **NFR-THEME** — The Tk app uses its own dark "robotics" palette (`BG #06111F`, `CYAN #00E5FF`, ...,
  `UI53:57-71`); not the ALIX Studio theme — do not carry it over.
* **X1 / X2** — Not touched by the runner (it is joint-space only; no Cartesian path generation besides
  MoveL to stored global points).

## Reuse verdicts

| feature | verdict | what to lift | evidence |
|---|---|---|---|
| Track parser + project writer | reuse-as-is | `read_track` (raise instead of `sys.exit`), `write_project` byte format | `RUN:76-97`; `ORIG:75-96` |
| Program list paging | reuse-as-is | `slots()` with `PAGE=10`, `trajectory_name` key | `RUN:99-119`; `SDKW:2338-2436` |
| Upload + verify | reuse-with-adaptation | `rm_send_project_t(path,0,1,slot,0,0,0)` + size read-back; add batch-in-one-session, sha/size pin skip, no `input()` | `RUN:147-175`; `ALIX/tools/cleaning_tracks.py:4046-4110` |
| Single program run | reuse-with-adaptation | gates, non-blocking start, polling, watchdog, slow stop, speed restore; add stop token + joint snapshot | `RUN:189-269` |
| Fast workflow executor | reuse-with-adaptation | `run_workflow_sequence`, `run_workflow_program_fast`, `workflow_quick_settle`, v14 transit semantics | `RUN:1668-2277` |
| Multi-run sequence | reference-only | Pole -> Hand -> Wait -> Program step order; superseded by workflow | `RUN:393-753` |
| SIM/REAL run mode | reuse-as-is | get/set + read-back verify; label as "controller dry run" | `RUN:1213-1262`; `SDKI:7193-7224` |
| 3D simulation | not-available | — (take from ALIX Studio) | §7 |
| Drag teach start/stop/save/playback | reuse-with-adaptation | SDK calls + `finally` safety stop; add save retry, region naming, non-signal stop | `RUN:912-989`, `1173-1210` |
| Drag `.txt` -> `DT_` program | reuse-with-adaptation | naming + rename; respect 32-byte name field | `RUN:992-1150`; `SDKW:2355` |
| Pole control | reuse-as-is | `pole.py` + add arm-at-rest check from `usage.py` | `POLE:1-58`; `USAGE:44-72` |
| Hand control | reuse-as-is | `hand.py` (Modbus RTU, presets) | `HAND:1-126`; `UI53:2979-2998` |
| Region prep order (R7) | reference-only | `prestroke`, `stop_all` patterns | `USAGE:128-150` |
| Global waypoints | reuse-with-adaptation | list/read/move; prefer non-blocking moves so stop works | `RUN:1265-1665` |
| Error-code handling | reuse-as-is | `arm_state`, `codes_text`, `WARN_ONLY`; add 4114/4115 names | `RUN:51-57`, `122-144`; `ALIX/docs/hardware-sdk/ERROR_CODES.md:46-56` |
| SDK loader | reuse-with-adaptation | configurable SDK path (repo fallback broken here) | `RUN:60-73` |
| Connection handling | reuse-with-adaptation | triple mode, `rm_set_timeout(1500)`; add persistent sessions + reconnect | `RUN:2606-2614,2697-2698` |
| IP entry (R4) | reuse-as-is | `--ip/--port` (UI lacks the field) | `RUN:2284-2286,2347` |
| Dual-arm barrier | reference-only | move barrier into backend; measure start skew | `UI53:5654-5709` |
| CLI / stdout protocol | reuse-as-is (MVP) / reference-only (target) | flag set §1.2 and tokens §1.4 | `RUN:2280-2604`; `UI53:5752-5960` |
| Original `rm_upload.py` | reference-only | same core as v14 lines 1-268 | `ORIG:1-344` |
| Fake-controller tests | reuse-with-adaptation | test pattern for Lite backend without hardware | `ALIX/core/python/alix_dispatch/test/test_rm_upload.py:1-212` |
| Emulated program channel | reuse-with-adaptation | offline slot store / 5.5545 ms player for a no-robot mode | `ALIX/core/python/alix_dispatch/src/alix_dispatch/emulated_program.py:1-84` |
| Vendor FK/IK (R9/R14) | reuse-with-adaptation | `Algo` FK/IK with FK-verification guard | `SDKI:6147-6530`; `ALIX/docs/EMULATOR.md:92` |
| Faster upload (R13) | not-available (in runner) | approach in §3.3-3.4 | `ALIX/docs/motion/MOTION_TRANSPORT_PLAN.md:210`; `RUN:147-175` |

## Open questions

1. Does `rm_set_arm_slow_stop` actually abort a running stored program (ALIX gate G4)? Lite's stop
   button depends on it.
2. Does the v13/v14 "overall speed" have any effect, given that `rm_set_program_id_run` overwrites the
   same global ratio with the transit speed? Do global-point moves after a Program run at
   `speed x transit ratio`?
3. Can `rm_get_program_run_state()["plan_speed"]` read back the global ratio (the runner and ALIX say
   there is no getter)?
4. In controller SIM mode, what do pole and hand steps do (time out, move physically, or no-op)?
5. Which controller program-ID range will Lite own on each arm, given ALIX's 70-100 production range
   and the control app's 71-83 workflow?
6. What is the measured dual-arm motion-start skew with the barrier + per-process launch?
7. Upload throughput for 85-700 KB files on these controllers (only one data point: 6.4 s / 290 KB).
