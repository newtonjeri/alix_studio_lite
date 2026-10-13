# 06 — ALIX motion / upload-channel research map (taught tracks, upload, speed-up, smoothing, spikes, merging)

Key: `06_alix_motion_upload_research` · written 2026-10-13 · read-only survey (no robot, no network, no tests run).

## 0. Path aliases and conventions

| alias | absolute root |
|---|---|
| `alix/` (or no prefix: `tools/`, `docs/`, `Resource/`, `runs/`, `plan/`) | `/home/newtonjeri/alix_ws/src/alix/` |
| `dispatch/` | `alix/core/python/alix_dispatch/src/alix_dispatch/` |
| `exp01/` | `alix/experiments/exp01_teach_trajectory_reverse_engineering/` |
| `workspace/` | `alix/core/python/alix_workspace/src/alix_workspace/` |
| `lite/` | `/home/newtonjeri/alix_studio_lite_ws/` |
| `RUNNER` | `lite/upload_channel/rm_upload_fast_v14_safe_transit.py` |
| `HTML` | `lite/Alix_Studio_Lite_Trajectory_V2.html` |

Short doc names: `TCTR` = docs/motion/TRACK_CHANNEL_TIMING_REQUIREMENT.md · `TCP` = docs/motion/TAUGHT_CANFD_PIPELINE.md ·
`TPR` = docs/motion/TAUGHT_PIPELINE_README.md · `TQS` = docs/motion/TAUGHT_CANFD_QUICKSTART.md ·
`DTCA` = docs/motion/DRIVER_TRACK_CHANNEL_ARCHITECTURE.md · `MTA` = docs/motion/MOTION_TRANSPORT_AUDIT_2026-08.md ·
`MTP` = docs/motion/MOTION_TRANSPORT_PLAN.md · `TREADME` = Resource/tracks/taught/README.md ·
`EM` = Resource/tracks/taught/EXECUTION_MODEL.json · `TTJ` = Resource/tracks/taught/TAUGHT_TRACKS.json ·
`FIND` = plan/FINDINGS.md · `UPC` = Resource/reports/2026-09-17_upload_player_clock/README.md ·
`GIC` = Resource/reports/2026-09-20_geometric_intervention/RESULT_FROM_CORPUS.md ·
`LEAVE` = exp01/notes/2026-09-18_what_makes_the_arm_leave_the_path.md ·
`TT` = tools/taught_tracks.py · `CT` = tools/cleaning_tracks.py · `TP` = tools/taught_pipeline.py ·
`RU` = tools/rm_upload.py · `DAF` = tools/dual_arm_flow.py · `DAV` = tools/dual_arm_verdicts.py.

Markers: **(derived)** = computed by me from cited raw data with read-only `python3 -I -c`; **(inferred)** = my reading, not stated in a source.

Two lanes exist in ALIX and they must not be confused:
* **Upload lane** (`rm_send_project` -> controller slot -> `rm_set_program_id_run`): the controller's own point player plays the file. Slots 1-100. This is what Lite's RUNNER does.
* **CANFD lane** (`rm_movej_canfd` streamed from the host at 5.0 ms): nothing is uploaded, "slots" >= 101 are registry numbers only (TCP:254-261). ALIX's most recent taught pipeline (TP, 2026-09-22/23) targets this lane, left arm only.

## TL;DR for planners

1. Upload = `.proj` = `{"file":7}` + Folder line whose `type` is the point count + the NDJSON point lines, every line CRLF; `rm_send_project(rm_send_project_t(path, plan_speed, only_save=1, slot, 0, 0, 0))` -> success is `ret 0, err_line -1`; verify by re-listing (`rm_get_program_trajectory_list`, page size 10) and comparing stored `size` within max(64 B, 0.1 %) (RU:90-96,146-174; dispatch/track.py:85-112,353-390). Lite's RUNNER contains this code verbatim (it is a fork of RU, see §1.8).
2. **Upload time is proportional to bytes**: the controller acks every 2 KB chunk (FIND:5701-5706); six measured uploads of 180-325 KB took 4.04-7.15 s, i.e. 44-46.5 kB/s; fit 0.24 s + 21.3 us/byte (derived) ~ **1.25 ms per point** (56-60 B/point). "Faster upload" therefore = **fewer points** (pause removal: -16..-24 %; time-optimal re-timing: 3.2-3.9x fewer lines), **skip unchanged slots** (size + name readback, sha pins), **one connection per arm for a batch**, or **no upload at all** (CANFD streaming). No ALIX evidence of any faster transfer parameter.
3. The run `speed` (and `plan_speed`) **does not change playback speed** — it scales only the controller's implicit movej transit to the first point, and it becomes the controller's **persistent global ratio** (no getter, survives power cycles) that must be restored after every run (FIND:5854-5881; dispatch/track.py:21-24,155-198).
4. The player advances **at most one point per 5.0 ms controller cycle** and otherwise holds; whole-file average 5.55-6.15 ms/pt; fitted `replay_s = 0.2322 s + 5.6737 ms x (N-1)`; same file varies +-6 % run to run (UPC:6-10; dispatch/track.py:45-56,1537-1570). **Rates must be judged at 5.0 ms**, durations quoted as a band.
5. Hard limits: joint ratings **180/180/225/225/225/225/225 deg/s**, position limits **+-178/+-130/+-178/+-135/+-178/+-128/+-360 deg**, **<= 10 deg per line** (0x4000), acceleration 600 deg/s^2 (advisory on upload, a real following limit on CANFD) (exp01/src/teachlab/kinematics.py:32-61; TCTR:25-36; TCP:158-161).
6. Gates (TT `gate_report`): **G1** predicted executed joint margin >= 0.5 deg (overshoot = max(0.85 deg, 0.139 s x arrival rate)); **G2** 0 ticks over a rating at 5.0 ms, 20 Hz band-limited; **G3** step <= 10 deg; **G4** pad deviation of the bytes vs the raw recording <= 10 mm (5 preferred); **G5** predicted executed pad departure at every line <= 20 mm (FK(q + tau_j*qdot) x 2.0). (TCP:276-300; TT:420-604; EM:1-32.)
7. Smoothing evidence: 5 Hz order-2 zero-phase Butterworth on the 5 ms record clock moves the pad <= 1.4 mm and removes hand ripple; arc-length smoothing (30 mm) is the main path change (<= 3.6 mm) and what lets TOPP-RA go fast; executed-deviation benefit of smoothing is large on CANFD (0.36-0.41x) and **nil on the upload channel (median 1.04x)** (TCP:86-138; GIC:77-99).
8. Pauses: ALIX rule R4 = a run of >= 3 lines all within **0.05 deg of the run's FIRST line** collapses to that line (anchor rule, not step-to-step) (TT:915-951). Lite's HTML uses a step-to-step rolling-mean rule, which ALIX found cuts slow apexes by up to 9 mm (TT:926-928; HTML:1438-1480).
9. Merging: ALIX does **not** merge taught files into one program; it sequences rows (pole -> hand -> rest -> bridge movej -> stroke -> rest). Bridges/rest legs are **collision-UNchecked** joint moves; a reusable concatenation primitive exists (`plan_min_time.concatenate`, boundary gap <= 0.5 deg). Dual-arm concurrency is allowed only under a windowed FCL clearance >= 25 mm, arm-vs-arm (DAF:19-47,191-192; tools/plan_min_time.py:142-169).
10. Reuse: `alix_dispatch` is ROS-free, stdlib-only at import (numpy lazy), SDK imported lazily -> Lite can import `track`, `connect`, `runmode`, `limits`, `ladder`, `recording` directly. The analysis lane (TT/TP/exp01) needs numpy+scipy(+toppra) and FK via `alix_workspace`; port its ~10 pure functions rather than import the whole lane.

---

## 1. Upload mechanism (R13, INT-CTRL)

### 1.1 SDK call sequence (as implemented in ALIX)

| step | call | evidence |
|---|---|---|
| connect | `RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)`; `rm_create_robot_arm(ip, 8080, 3)`; `rm_set_timeout(1500)` | RU:320-327; dispatch/connect.py:69-72,141-182 |
| why 1500 ms | vendor default 500 ms; a controller "loading a 4000-point program, streaming one, or running a Modbus transaction" answers slower -> SDK returns -2 and two 12-task flows died on 2026-09-04 | dispatch/connect.py:37-49 |
| list slots | `rm_get_program_trajectory_list(page, 10, "")`, page until short; items keyed `id`, `size`, `speed`, `trajectory_name` (not `name`) | RU:99-118; dispatch/track.py:115-152 |
| why page 10 | a page of 100 with ~70 programs overflowed the SDK buffer -> "Error parsing JSON", ret -3 (2026-09-23) | dispatch/track.py:115-118; RU:99-101 |
| build file | `.proj` (see 1.2) | RU:90-96; dispatch/track.py:85-98 |
| upload | `rm_send_project(rm_send_project_t(str(proj), plan_speed, 1, slot, 0, 0, 0))` -> `(ret, err_line)`; ok = `ret == 0 and err_line == -1`; `only_save=1` is the normal path, running is a separate confirmed step | RU:158-164; dispatch/track.py:101-112 |
| ret codes | 1 controller false; -1 send failed; -2 no reply; -3 unparsable reply; -4 file name refused; -5 file unreadable; -6 Gen-4 controller unsupported; `err_line 0` = data length check failed, `>0` = that line refused | RU:49-53,159-163 |
| wire protocol | JSON `run_project` (`only_save`, `save_id` 1-100, `step_flag`), chunked transfer acked every 2 KB via `conduct_project`, verified by `download_project` err_line | FIND:5701-5710; exp01/src/live/upload_track.py:44-49 |
| verify | re-list; slot present and stored `size` within max(64, sent//1000) bytes | RU:165-173; dispatch/track.py:371-386; CT:4107-4159 |
| run | precheck: slot non-empty, no arm error code, no program running/paused; `rm_set_program_id_run(slot, run_speed, 0)`; poll `rm_get_program_run_state` (`run_state` 0 idle, 1 running, 2 paused) | RU:188-251; dispatch/track.py:3075-3087,3517-3610 |
| restore | `rm_set_plan_speed(restore_pct)` after the arm settles | RU:255-261; dispatch/track.py:161-198 |
| Gen | Gen-3 only (`-6` on Gen-4); proven on V1.7.4 | RU:53; exp01/src/live/upload_track.py:11-18; FIND:5799-5806 |

### 1.2 Program file format (`.proj`)
* Line 1 `{"file":7}`; line 2 `{"name":"Folder","num":1,"type":<point count>,"enabled":true,"parent_number":0}`; then each `{"point":[j1..j7]}` (integer millidegrees, URDF joint1..7 order) **verbatim**; **every terminator CRLF** (dispatch/track.py:85-98; RU:90-96; exp01/src/live/upload_track.py:20-26,77-109).
* LF-only was **rejected `err_line=1`** on V1.7.4 (2026-08-28): the controller splits on CRLF, so the whole file read as one bad line 1 (dispatch/track.py:19-20; FIND:5779-5786). The pendant's own export header is `{"file":7,"global_r":100}` (FIND:5783).
* Parser contract: refuse the whole file on any malformed line, require exactly 7 ints, >= 2 points ("a truncated file cannot be half-uploaded") (dispatch/track.py:66-82; RU:75-87).
* Plans/other joint sequences can become tracks **only by resampling at the player clock** — copying 10 ms waypoints would run 1.8x faster than planned (FIND:5734-5748).
* Size per point: 56-60 B (derived from TTJ `proj_bytes/points`, e.g. 180,441 B / 3,057 pts = 59.0 B; runs/20260828T213858_exp01_G1_upload_only 293,247 B / 5,252 pts = 55.8 B).

### 1.3 Slots, naming, identity
* 100 program slots per controller, **no auto-reclaim, persist across power cycles** (dispatch/track.py:25; MTP:196).
* ALIX allocation per arm: **70-100 = production cleaning** (taught rows refused there), 40-59 = taught upload rows, 21-39 and 61-69 = experiments, 60 = dry probe, >= 101 = CANFD-only registry numbers (CT:4-12,2763-2800,2855,2863-2871; TREADME:164-169; TCP:254-261).
* **Conflict to resolve (inferred):** Lite's V6 workflow runs program ids 71, 73, ... with `"speed": 100` (lite/upload_channel/LEFT_V6_70SERIES_FINAL.json, `steps[].id`) — inside ALIX's reserved 70-100 range. If ALIX and Lite share the two controllers, slots will collide.
* Stored name = `(f"{id}_" + basename forced to .txt)[:32]` — keep names short (TREADME:150-162; dispatch/emulated_program.py:81-84). `project_path` is `c_char*300`: a long temp path loses the upload (dispatch/emulated_program.py:85-87,113-117).
* Identity pins: `track_sha256`, `proj_sha256`, `proj_bytes`; `upload` refuses a row whose bytes no longer hash to its pin (incident B15: a slot played 2,218 points from bytes not on disk) (CT:30-43,4057-4098). Run-time `controller_slot_check` reads the slot back (stored size + `trajectory_name`) before running (CT:995; CT:45-50).
* "Truncation at upload is DEAD": on nine slots the stored size matched the sent `.proj` byte for byte; the size check stays as a guard (FIND:9336-9343).
* Certificate (taught lane): sha256 of `.proj`, `.txt` and source + every gate's value and bar; `upload`/`run` re-verify and refuse unless `--force` (TT:30-33,3170-3193,3210-3225; TREADME:128-133).

### 1.4 Measured upload times (exp01 lanes G/H) and what makes uploads faster

| run (runs/…) | points | bytes | upload s | kB/s (derived) | result |
|---|---:|---:|---:|---:|---|
| 20260828T212857_exp01_G1_upload_right_test | 5,252 | 293,245 | 6.62 | 44.3 | ret 1 / err_line 1 (LF header) |
| 20260828T213858_exp01_G1_upload_only | 5,252 | 293,247 | 6.31 | 46.5 | ret 0 / -1 |
| 20260828T215446_exp01_G2_upload_only | 5,835 | 325,446 | 7.15 | 45.5 | ret 0 / -1 |
| 20260828T221443_exp01_G4_upload_only | 5,047 | 290,066 | 6.43 | 45.1 | ret 0 / -1 |
| 20260829T191644_exp01_H1_upload_capped | 3,733 | 214,009 | 4.84 | 44.3 | ret 0 / -1 |
| 20260829T191701_exp01_H1_upload_fast | 3,142 | 180,153 | 4.04 | 44.6 | ret 0 / -1 |

Source: each folder's `run.json` -> `stages[stage=upload].t_end - t_start`, `project_file.bytes/points`, `transport.send_ret/err_line` (e.g. runs/20260829T191701_exp01_H1_upload_fast/run.json:83-105,132-139).
Linear fit (derived): `upload_s ~= 0.24 + 21.3e-6 x bytes` (~45 ms per 2 KB acked chunk; consistent with "293 KB in 6.62 s, ~143 x 2 KB chunks", FIND:5772-5775, and "Upload already costs 6.4 s for 290 KB", MTP:210).

What the ALIX evidence says makes an upload faster (no transport knob exists):
1. **Fewer points.** Same geometry, fewer lines: H-lane control 5,047 pts / 6.43 s vs fast 3,142 pts / 4.04 s (table). Taught bowl-outside: 25,426 raw lines -> 6,560 after pause removal + TOPP-RA (TCP:48-53), i.e. ~31 s -> ~8 s of upload (derived). Commode C files: 3.24-3.49x fewer lines (TCP:586-591).
2. **Do not re-upload what the slot already holds**: the readback size + `trajectory_name` and the sha pins make "ALREADY" decidable without sending (TPR:66-73; TT:4108-4118; dispatch/track.py:353-390).
3. **One connection, many slots**: `cleaning_tracks.py upload` loops every row on one handle then reads the list once (CT:4071-4159); `batch --upload` sends all certified files "to their arm in one connection" (TREADME:110-112). Parallel left/right uploads on two handles are plausible (two controllers) but never measured (inferred).
4. **No upload at all for test runs**: CANFD streaming, "canfd is not limited to 100 slots, because nothing need to be uploaded" (TCP:254-261) — different rules, see §1.7.

### 1.5 Size limits and controller-busy effects
* No documented maximum file size was found (grep over docs/motion, CT, TT, dispatch). Largest admitted upload rows: slot 43 735,603 B / 12,596 pts and slot 50 750,251 B / 12,864 pts (TTJ rows; derived from `proj_bytes`), and slots 43/50 have upload runs in the smoothing corpus (GIC:89-99, inferred that they were uploaded and played).
* **The controller is busy during and after an upload**: in all six captures that contained an upload the 5 ms UDP push degraded to mean 6.6-7.8 ms, p95 10.0-10.6 ms, **max gaps 400-600 ms** (runs/*_upload*/run.json `arrival_ms`; the one refused-before-upload capture, 20260828T212250, stayed at mean 4.6 ms, max 12.9 ms), and the push restore right after a 293 KB upload got no answer (ret -2), now retried with read-back (FIND:5816-5828). Implication: never upload while that controller is moving or being watched (inferred).

### 1.6 Run semantics that bite (upload lane)
* **`speed` scales only the transit**: G1/G3 transit 2.51 / 0.82 / 4.33 s at speed 20 / 100 / 10 while the track term stayed 29.16 / 29.54 / 29.56 s (FIND:5854-5857); same for the pendant slider (exp01/README.md:194-197) and plan-derived tracks (FIND:5908-5914).
* **Global ratio hazard**: the run speed becomes the persistent controller-wide ratio; a later movej of 60.9 deg took 7.37 s after speed 20, 1.95 s after 100, 11.6 s after 10 (FIND:5866-5881); a 73.7 deg transit took 52 s at an inherited ratio (dispatch/track.py:617-622). Always restore (`global_speed_guard`, dispatch/track.py:161-198). `speed=0` = the slot's stored speed (FIND:5810-5812; dispatch/emulated_program.py:33-36).
* **Transit is collision-UNchecked**: `rm_set_program_id_run` performs an implicit movej to the first point; ALIX announces the gap and optionally bridges with an explicit `rm_movej` at its own speed after setting the ratio to 100 (dispatch/track.py:323-338,592-667).
* **Completion evidence** (`ok` is NOT the duration band): blocking probe `rm_set_program_id_run(block=1)` returns 0 complete / -4 stopped externally (perfect classifier 118/7 over 125 runs); `program_run_finish` event (90/4 over 94); replay verdict from the capture (dispatch/track.py:447-490,1690-1794,3555-3584). `run_state 0` cannot tell completion from a joint-limit stop (FIND:5859-5864) and is not "at rest": 25.4 % of records still moving, up to 6.8 s (dispatch/track.py:769-775).
* **Stopping**: no program-scoped stop exists in the vendor API (DTCA:19-29). ALIX escalates `rm_set_arm_slow_stop -> rm_set_arm_pause -> rm_set_arm_stop` and proves each rung by run state + joint stillness (dispatch/track.py:3069-3130,719-751). Whether `rm_set_arm_stop` aborts a running program is **untested on hardware** (DTCA:185-190; dispatch/emulated_program.py:51-56).
* Codes: 4105 "speed out of limit" is flagged and the run continues (1,139 frames on a clean slot-42 run) (dispatch/track.py:1653-1661); 201 of 209 captures that crossed a joint's own rating raised 4105 (dispatch/track.py:3698-3703); with 4105 latched one program ended at 58 % (CT:211-218). 4109 = arm collision (RU:55).

### 1.7 CANFD alternative (no upload) — only for comparison
* `rm_movej_canfd(q, follow=high, 0, trajectory_mode=0, radio=0)` one line per 5.0 ms on absolute deadlines, GC off, last line re-sent 0.10 s (dispatch/track.py:3989-4099; TCP:406-466).
* Whole-file preflight refuses (never clamps): 10 deg/frame and rating x period; vendor documents a **flat 180 deg/s** with "the joint will not respond" as the consequence; ALIX defaults to the per-joint rating gate since 2026-09-21 (dispatch/track.py:3680-3705,3767-3864; TCP:447-450; CT:4397).
* Start gap <= 0.72 deg at 5 ms else bridge movej and re-measure (TCP:451-452; dispatch/track.py:573-582,640-654). Watch aborts on stop codes (not 4105), stall (>= 5 deg commanded in 0.25 s while arm <= 0.5 deg), blindness (> 0.5 s no push), > 2 periods late, 2 failed sends (dispatch/track.py:3737-3764,3938-3976).
* Upload vs CANFD trade-off: upload = controller-timed, rate up to the joint ratings (J4 221.8, J7 224.2 deg/s observed), no reactivity; CANFD = host-timed, flat 180 per vendor, in-flight abort (MTA:383-405; MTP:119-136).

### 1.8 Lite RUNNER vs ALIX (INT-CTRL)
* RUNNER lines 1-176 are identical to RU lines 1-175 except the usage lines and the added `--sequence` mode (diff of the two files) — `read_track`, `write_project`, `slots`, `upload` with size verification, `arm_state`, `run`, `rm_set_timeout(1500)` all came from ALIX (RUNNER:76-175,2606-2614).
* RUNNER adds drag teach (`rm_start_drag_teach`, `rm_stop_drag_teach`, `rm_save_trajectory`), DT_ uploads renamed with `rm_update_program_trajectory` (RUNNER:1018-1150), run-mode get/set (RUNNER:1213-1246,2621-2625), global waypoints, frames, and workflow sequences that restore the global speed once at the end (RUNNER:1807-1822).
* RUNNER's whole SDK surface (grep of `.rm_*(`) contains **no** `rm_set_realtime_push` (no UDP capture), **no** `rm_get_arm_event_call_back` (no finish event), **no** `rm_set_arm_pause`/`rm_set_arm_stop` (only `rm_set_arm_slow_stop`), and no blocking probe — i.e. none of ALIX's completeness or verified-stop evidence (dispatch/track.py:447-490,3069-3130).

---

## 2. Speed-up: ladders, execution model, gates, ratings, clocks

### 2.1 The clocks (the "5 ms vs 5.4 ms" question)
| constant | value | meaning | evidence |
|---|---|---|---|
| `CLOCK_RECORD_S` | 5.0 ms | drag-teach sampling: every recording lasts exactly lines x 5 ms; design any filter of a recording on this clock | TT:109-115; TCP:63-66 |
| `CLOCK_GATE_S` / `RATE_CEILING_CLOCK_S` | 5.0 ms | the fastest the upload player plays a line (one point per 5 ms cycle between holds); every rate/step gate is judged here | TT:94-102; CT:1636-1643; UPC:90-94,126-130 |
| `CLOCK_PREDICT_S` | 5.4 ms | the execution model was fitted/validated at 5.4 ms; G1/G5 predictions use it "on purpose"; do not change without a refit | TT:103-106; TCP:349-351 |
| `REPLAY_CLOCK_S` / `CLOCK_NOMINAL_S` | 5.5545 ms | upload player's whole-run AVERAGE (F98), human-facing durations only; "not a clock" (5 ms cycle + 10-13 % holds) | dispatch/track.py:45-51,3708-3719; CT:4376-4389 |
| player law | `replay_s = 0.2322 + 5.6737 ms x (N-1)` (R2 0.99924, 158 runs); de-ramped envelope 5.35-6.10 ms/pt | duration prediction / completeness witness | dispatch/track.py:1537-1570 |
| taught duration band | `[0.057 + 5.515 ms(N-1), 0.750 + 5.810 ms(N-1)]` | what a row's band means | TT:116-121,278-281 |
| run-to-run | same file +-6 %; 5.39 % apart in one session; drift up to +4.7 % across sessions | quote ratios not absolutes | dispatch/track.py:45-50; MTA:222-246; FIND:6688-6697 |
| CANFD period | 5.0 ms default (controller cycle); vendor range 2-20 ms; high follow needs <= 10 ms | streaming only | dispatch/track.py:3708-3719; CT:4400-4406 |

Holds: the player runs ~5.215 ms/pt (4.1 % held) where the file moves, 5.800 ms/pt (13.8 % held) on near-stationary lines; taught files hold more (6.9 %) than planned ones (UPC:6-10,90-113). Stale text: TREADME:72-73 and TTJ `convention.clock_gate_s: 0.0054` still say 5.4 ms for the gate clock (TCP:566-569 records this as stale).

### 2.2 Joint ratings and limits (RM75)
* Velocity ratings 180/180/225/225/225/225/225 deg/s (exp01/src/teachlab/kinematics.py:32-37; dispatch/track.py:3720-3728; CT:1633), confirmed as the firmware's own `libalgo_get_joint_vel_maxlim` (dispatch/track.py:3698-3705).
* Acceleration ceiling 600 deg/s^2 `joint_drive_max_acc` (kinematics.py:39-41; TP:138-139).
* Position limits (-178,178), (-130,130), (-178,178), (-135,135), (-178,178), (-128,128), (-360,360) deg (kinematics.py:43-46).
* Step <= 10 deg per line or error 0x4000 (TT:127-128; kinematics.py:56-61).
* Servo overshoot at an apex: 1.9 deg typical, up to 4.5 deg; J4 executed **+5.6/+5.9 deg past its command and 0.088 deg inside +-135** on a cold arm (exp01/src/predict_track_replay.py:77; FIND:6721-6737,6754-6760).
* Tool speed: RM75 body spec TCP <= 1.8 m/s; ALIX replay Cartesian caps 1.5 m/s, 5 rad/s, acc x4 (dispatch/limits.py:351-383) — the upload player's clock is unaffected by caps on/off (UPC:116-118).

### 2.3 The "no rate factors" rule (and where ALIX still designs below the bar)
* Ruling (Newton, 2026-09-15/16): "no rate factors — the manufacturer's rates are the limits; acceleration is not a gate; the only check is the deviation the hardware will produce" (TCTR:3-9,25-36). Hard limits per line: rate <= rating with no factor, step <= 10 deg, inside joint limits (TCTR:25-36).
* Design allowances that remain (instrument/rounding, not policy): `RATING_FILTER_ALLOWANCE = 0.97` because the 20 Hz filter overshoots a change of rate by up to 2 % (TT:227-229,1385-1390); `VEL_SCALE_DEFAULT = 0.97` in the planned-track emitter because integer rounding put ticks a hair over and 4105 latched (CT:211-219); the CANFD emitter runs at `--rate-scale 0.95` for the 6 Hz low-pass headroom (TCP:147,184-190).
* Acceleration: advisory on upload (files that run clean read 3-10x the 600 deg/s^2 planning figure) (TT:46-51; TCTR:33-36); **a real following limit on CANFD**: 1.5x on every joint made J6 overshoot (p95 23.4 / max 64 mm) (TCP:158-161); a 3x `+acc` slot tripped the controller's collision detection (4109) after 2.46 s, 40 mm off the path (TCP:675-682).

### 2.4 The execution model (EM, version 4, fitted 2026-09-16)
* `tau_path_s 0.056` (p90 overshoot / arrival rate), `tau_path_p50_s 0.011`, `tau_limit_s 0.139` (max, for limits), `floor_deg 0.85`, `limit_eps_deg 0.5`, `prominence_deg 0.5`, `approach_window_s 0.3`, `window_after_s 0.05`, per-joint lags `tau_joint_s [0.0678, 0.0324, 0.0536, 0.0378, 0.0533, 0.0603, 0.0654]`, `departure_calibration 2.0` (EM:2-31).
* Departure law: `p_pred(k) = FK(q(k) + tau_j * qdot(k))`, departure = distance to the commanded pad path, x 2.0; a lag common to all joints lands ALONG the path (0.00 mm perpendicular) so it must be formed per joint (TCTR:50-67; TT:1107-1165). Corpus: real p95 / predicted 1.56x taught, 1.99x planned (TCTR:69-83).
* Honest limits: budget is a p95 statement — worst case about twice it; 32-41 mm spots unexplained (TCTR:93-99); model is upload-fitted and does **not** predict CANFD, not even in rank order (TCP:345-356).
* Inverse (emitter): at speed factor lambda the departure scales linearly, so the per-line ceiling is `budget / departure(lambda=1)` (`departure_lambda_cap`, TT:1817-1852); the emitter measures what it wrote and pulls its ceiling down (over-relaxed, best pass kept) (TT:1855-1984).
* "Which joint owns the budget" at the worst line: `_departure_attribution` (TT:1083-1104).

### 2.5 Gates (what each checks, threshold, where)
| gate | measures | on | bar (default) | code |
|---|---|---|---|---|
| format | 7 integer millideg per line, >= 2 lines | bytes | parses | dispatch/track.py:66-82 |
| provenance | taught on this robot (`hardware: ok`), side declared | record | `ok` | Resource/tracks/taught/source/README.md:14-27; TCP:67-78 |
| slot | taught range | row | 40-59 or >= 101 CANFD-only; never 70-100 | CT:2763-2800 |
| **G1** | predicted executed joint margin at every extreme: commanded margin - max(0.85 deg, 0.139 s x arrival rate); "geometry" when even the floor overshoot breaks it (re-teach) | bytes + model | >= 0.5 deg | TT:466-487,1357-1365; EM:7-9 |
| **G2** | ticks over a joint RATING, 20 Hz 4th-order zero-phase band-limited, at 5.0 ms | bytes | 0 ticks | TT:440-444,594-599; exp01/src/predict_track_replay.py:82-114 |
| **G3** | largest step between lines | bytes | <= 10 deg | TT:600-602; predict_track_replay.py:131-136 |
| **G4** | exact point-to-polyline pad deviation (both directions) of bytes vs the recording at `{L,R}_glove_frame_2` | bytes + FK | <= 10 mm (5 preferred) | TT:129-132,607-667 |
| **G5** | predicted executed pad departure at EVERY line (joint-space law x 2.0) | bytes + model + FK | <= 20 mm (`--exec-budget-mm`) | TT:133-138,1258-1327 |
| advisory | accel / jerk / velocity utilisation, slowness census | bytes | never refuses | TT:517-526,954-991 |

Caveats: G4 compares against the file handed to certify, so a derived (smoothed/TOPP-RA) file reads 0.0 mm — check derived files against the raw recording separately (TCP:311-343,547-549). Legacy pre-model G1: margin >= 5 deg allowance, hard floor 2 deg, and inside the allowance the band-limited rate <= 10 deg/s (TT:162-182,481-487). A taught recording certifies in ~2-3 s; `screen` takes ~0.25 s per file (TREADME:93,98).

### 2.6 Speed ladders
**Upload lane (TT `shape`, default `--timing model`):**
* Rung 0 = verbatim recording (zero deviation by construction) (TT:2670-2681).
* `model:lambda` rungs for lambda in [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.5, 4.0, 5.0] + 1.0: the pause-stripped path at the joint ratings everywhere the model allows, the model's arrival rate at every reversal/limit, lambda = the ceiling on free stretches (TT:143-149,2746-2779).
* DIAL 1 legacy: `uniform:lambda` (linear interpolation ON the taught chords, path cannot move, 0.01 mm measured) and `guard:lambda` (uniform with near-limit rate caps) (TT:143-149,748-853).
* DIAL 2: TOPP-RA re-parameterisation at vel scales [1.0, 0.9, 0.8, 0.7, 0.6], only when no other rung reaches 1.3x (`SOLVER_AUTO_BELOW`), skipped under the model (TT:139-142,247-253,2803-2841).
* Last resort slow-down [0.9 ... 0.5] when nothing passes (TT:154-161,2843-2850).
* Pick: fastest rung that passes G1/G2/G3/G5, preferring G4 <= 5 mm, else the fastest inside 10 mm (TT:2852-2880); `--max-speed`, `--pick`, `--section A:B:F` (a stretch at factor F), `--dwell S:SEC`, `--keep-pauses` (TREADME:55-58,80-86; TT:1439-1466).
* Ramps: lambda may change 0.05 per source line (model) / 0.01 (guard) so slow-downs are ramps, not steps (TT:174-176,224-226,800-803).

**CANFD lane (TP, MAX DEV = two-way knob):** default recipe = 5 Hz de-jitter -> pause removal -> 30 mm arc-length smoothing -> TOPP-RA at 0.95 x ratings, 600 deg/s^2 -> 6 Hz post low-pass at 5 ms (TP:45-49; TCP:17-39). With a MAX DEV budget the ladder walks FASTER via rounder corners (smoothing 45/60/90/120 mm) or SLOWER via velocity+acceleration x (0.95, 0.8, 0.65, 0.5, 0.4, 0.3, 0.25, 0.2, 0.15, 0.1) at 15 mm smoothing; `+acc` tries 1.5-8x acceleration with an arrival hold (TP:107-141,254-262; TCP:620-666). Predicted executed departure = file vs raw + `min(0.5 + 0.057 x v_max, 7.5)` mm (tools/tcp_gate.py:52-80; TCP:683-692); measured runs came in at about half the prediction (TCP:702-716).

### 2.7 Measured speed-ups
| case | before -> after | speed-up | evidence |
|---|---|---|---|
| model rung, executed budget 5/10/15/20 mm, left bwl-out-01-slow (141.2 s taught) | 102.7 / 56.2 / 40.5 / 32.5 s | 1.38x / 2.51x / 3.49x / 4.34x | TCTR:196-205 |
| stroke wipes vs scrub at 20 mm | lid side 1.21x, rim top 1.18x; bowl outside 0.70x (reverses every 0.3 s) | — | TCTR:209-218 |
| TOPP-RA CANFD v1 / v3 | 127.1 -> 32.8 s / 91.2 -> 22.8 s | 3.88x / 3.99x (~2.9-3.0x after removing holds, which are ~24 % of nominal) | TCP:41-53 |
| commode C 101-104 | 62.1->19.2, 65.8->19.8, 60.0->17.8, 48.2->13.8 s | 3.24-3.49x | TCP:586-591 |
| upload lane H (planned toplid_right) | 29.13 -> 21.55 / 18.13 s | 1.35x / 1.61x, zero faults, 100 % reached | FIND:6683-6686 |
| CANFD-legal cap 176 deg/s | 1.599x for +0.48 % duration vs uncapped 1.607x | — | MTA:146-174 |

Cost of speed: median off-curve 1.34 -> 2.28 -> 3.00 mm at 1x / 1.35x / 1.61x; p95/max dominated by session noise; a cold arm doubles off-curve p95 (FIND:6699-6719).

### 2.8 How this can drive Lite's automatic speed optimisation and safety validation (recommendation, inferred)
1. Parse with ALIX's contract; refuse malformed files (dispatch/track.py:66-82).
2. Remove pauses with the anchor rule (0.05 deg, >= 3 lines, keep last line) (TT:915-951).
3. Optional de-jitter: 5 Hz order-2 zero-phase Butterworth **on the 5 ms record clock, before timing** (TT:880-912).
4. Re-time on the taught chords with per-joint caps = 0.97 x rating at 5.0 ms, ramped (`capped_millideg`, TT:767-832; `rate_fit_track` pattern CT:1675-1714), with a user ceiling per region/sub-region = `--section A:B:F` semantics (TT:1439-1451,1873-1876). This is the model rung without the FK-dependent departure cap; add `departure_lambda_cap` when Lite has FK (TT:1817-1852).
5. Validate every edit: G2, G3, joint margin (>= 5 deg allowance or G1 >= 0.5 deg), G4 <= 10 mm vs the raw recording; show predicted duration via the player law and band (dispatch/track.py:1566-1570; TT:278-281). Offer a per-region "accuracy" knob = MAX DEV 5/10/20 mm as in TP (TPR:33-40).
6. Upload only verified bytes (sha + size readback), run with global-speed restore, capture the run if possible (§7.3).

---

## 3. Smoothing (R2, R11)

| method | where | parameters | measured path change | effect on execution |
|---|---|---|---|---|
| temporal de-jitter `smooth_taught` | TT:880-912; TCP:82-90 | Butterworth order 2, `filtfilt`, 5 Hz, on the 5 ms record clock, before pauses/timing | pad p95 0.52 / max 1.43 mm (v1); 0.30 / 1.11 mm (TT:887-888); same filter on the emitted (2-2.75x faster) file moves ~6 mm | ripple 21-36 % -> ~5 %; CANFD motor-current ripple was 2.4-2.7x the upload channel's (TT:880-890) |
| pause removal R4 `strip_pauses` | TT:915-951; TCP:102-109 | >= 3 lines within 0.05 deg of the run's first line; keep last line | removed lines lie p95 0.37 / max 0.55 mm off (v3) | removes holds (~24 % of nominal, TCP:41-46); slow creep is kept as path |
| arc-length smoothing `arclen_smooth` | exp01/src/local_speed.py:139-169; TCP:111-138 | q(s) on 0.5 mm grid, Butterworth order 4 zero-phase, cutoff 1/lambda per mm, cubic back onto knots | lambda 30 mm: recording->file p95 2.55-2.71 / max 3.18-3.61 mm; 50 mm: 4.23 / 6.25 mm | the main path change; lowers curvature, which is what binds TOPP-RA (abs(q''(s)) x v^2 hits 600 deg/s^2 first) |
| TOPP-RA re-time `retime_toppra` | local_speed.py:282-324; TCP:140-167 | toppra 0.6.9, velocity <= rate_scale x ratings, accel <= 600 deg/s^2, `Interpolation` discretisation, seidel, zero end velocity, sampled at dt | samples on the smoothed path to 0.01 mm | time-optimal non-uniform speed; collocation would overshoot accel 1.68x (local_speed.py:313-317) |
| post low-pass `post_lowpass` | exp01/src/emit_toppra_rung.py:200-215; TCP:169-190 | Butterworth order 4 `sosfiltfilt`, 6 Hz, 1 s hold padding | +0.03-0.12 mm p95; max 3.61 -> 6.06 mm (v1) | hand shake -53 %, wrist force -72 %; lifts accel peaks ~1.27x (not gated) |
| uniform re-time (DIAL 1) | TT:748-764 | linear interpolation on taught chords | 0.01 mm (exact search) | speed only |
| local re-time to a deviation law `retime` | exp01/src/retime_to_budget.py:121-133 | s(u) profile low-passed at 2 Hz ("a step in s is a delta in a") | 0 (same geometry) | law REFUTED as a control law: slowing high-accel stretches 30 % changed deviation 0 %, speeding 40 % cost +15 % (LEAVE:338-355) |
| subdivision `subdivide` | exp01/src/respace_teach.py:50-61 | insert collinear points where a step exceeds max_step | 0 | "add points there, not slow the whole path" (exp01/README.md:223-233) |

What the evidence says about acceptable smoothing and budgets:
* Budgets: G4 bytes vs recording <= 10 mm, 5 preferred (TT:129-132); task MAX DEV in execution: **5 mm** lid seat open / seat only close, **10 mm** bowl inside / inner back (TPR:33-40; TCP:620-625); G5 executed <= 20 mm (TCTR:130-137).
* For a 10 mm end-to-end budget, "lambda <= 30 mm with the low-pass is the setting with evidence behind it" (worst measured 7.3 mm end to end; stack of input change + execution) (TCP:397-404).
* Smoothing does not help on the upload channel on the median: 11 paired cells TRACKING 0.92x, TOTAL 0.99x; CANFD taught 0.41x / 0.36x; upload 9 cells median 1.04x; low-deviation cells 1.06x (slightly worse) (GIC:77-99). Slot 43 -> 50 (5 Hz) took CANFD p95 from 12 mm to 1.7 mm (LEAVE:482-485).
* Deviation is not set by speed/acceleration magnitude on a straight path (single-joint probes 91-293x below the law), nor by corner size (sagitta exponent +0.04-0.10); multi-joint participation is the remaining suspect (GIC:3-33,62-65,105-120; LEAVE:487-495).
* Upload departs 1.5-4.4x more than CANFD on identical bytes (median 2.36x) because of player holds (LEAVE:451-461).

Cross-check of Lite's HTML (INT-TRAJ), evidence-based flags (inferred conclusions from cited code):
1. `trimIdle` marks a step idle when the rolling mean of abs(dq) is under a threshold on all joints (HTML:410-415,1438-1480) — the step-to-step rule ALIX abandoned because "a slow apex whose every step is under tol but drifts degrees over the run is path ... judged step by step it was collapsed, cutting the apex by up to 9 mm at the pad" (TT:926-928). Use the anchor rule or add a drift cap.
2. `meanSmooth` = 101-point centred moving mean (~0.5 s at 5 ms) + a per-joint constant re-centering + endpoints restored (HTML:382-383,416-418,1494-1517). No ALIX method shifts a whole joint by a constant; the re-centering is a path offset, and restoring rows 0 and n-1 after re-centering can create a step at both ends. Needs a G4-style deviation check (<= 10 mm, 5 preferred) and G2/G3 checks before upload.
3. `interpolateRegion`/`applyRegionScales` = per-region uniform linear resampling (HTML:1568-1616) = ALIX DIAL 1 (path cannot move, TT:748-764), but with no inter-region ramp (ALIX ramps lambda 0.01-0.05 per line, TT:174-176,224-226) and no rating/step gate -> region boundaries become speed-step spikes (§4).
4. Durations are `rows x sampleMs` with sampleMs default 5 (HTML:367,731,1176-1179): correct for the record clock, but upload playback runs ~5.55-6.15 ms/pt on average (law in §2.1).

---

## 4. Spike detection and suggestions (R12)

| detector | measures | threshold / bands | where |
|---|---|---|---|
| G2 band-limited | per-joint rate after 20 Hz 4th-order zero-phase low-pass, at 5.0 ms, vs rating; longest over-run in ms | 0 ticks | exp01/src/predict_track_replay.py:82-114; TT:440-444,594-599 |
| raw rate | same, unfiltered (ripple spikes are over a rating tick by tick even when BL is not) | recorded | CT:1646-1662 |
| G3 step | max abs(dq) between lines | <= 10 deg (0x4000) | predict_track_replay.py:131-136; TT:600-602 |
| CANFD per-frame | step and rating x period per joint, first offending line, min legal period | refuse | dispatch/track.py:3792-3864 |
| one-tick speed step | max abs(delta v) between consecutive ticks on the written integers | good emitter 3.06-3.6 deg/s per unit acc_scale; defects 19.98 and 78.85; limit 10 x acc_scale | CT:227-231,1743-1750 |
| acceleration flag | 20 Hz BL accel, % of moving lines (> 1 deg/s) above 600 / 900 deg/s^2 | 0 % = clean CANFD runs; 6.3 % = slot 55 degraded; 19.6 % = slot 54 failed (advisory) | CT:278-336 |
| accel/jerk/vel utilisation | `band_limited_util` vs plan limits | advisory | TT:489-526 |
| near-limit approach | joint inside 5 deg allowance moving > 10 deg/s; margin < 2 deg = geometry | refuse / guard (legacy) | TT:162-182,446-487 |
| G1 arrival | reversal = 20 Hz trace extreme with prominence 0.5 deg; arrival rate = max BL rate in the 0.3 s before | margin >= 0.5 deg after max(0.85, 0.139 x v) | TT:1002-1033,1357-1365 |
| G5 + attribution | per-line predicted departure; worst line, pad speed there, which joints own it | <= 20 mm | TT:1083-1104,1258-1327 |
| slowness census | time and longest stretch under 20 / 50 mm/s pad speed; still runs R4 would still remove | recorded | TT:954-991 |
| live: push codes | every error code in the capture with first/last time (4105 warn, 4109 collision) | warn / stop | dispatch/track.py:1653-1688 |
| live: CANFD watch | stop code, stall, blindness, lateness | abort | dispatch/track.py:3737-3764,3938-3976 |

Raw recordings in ALIX's commode C folder (derived, numpy only): FILE_01.1 / 02 / 04 / 6 / 10.1 have 16 / 19 / 22 / 19 / 16 % removable pause lines (anchor rule), max raw step 0.60-1.22 deg, peak raw rate utilisation at 5 ms 0.54-0.73 except FILE_6 at 1.08 (9 raw ticks over a rating) (exp01/data/teach_files/commode_c_taught_paths/FILE_*.txt).

Spike -> suggestion mapping with evidence (for Lite's plain-language advice):
* Rate over a rating (G2) -> "slow only this stretch": capped re-time at 0.97 x rating, tightening 2 % per pass until 0 ticks (CT:1675-1714); or add points where the step is large (exp01/src/respace_teach.py:50-61; exp01/README.md:223-233).
* Raw ripple spikes while BL is fine -> de-jitter 5 Hz on the record clock (TT:880-912).
* Corners forcing slow-downs (curvature-bound) -> round corners (arc-length 30 mm; more = faster, larger deviation): "rounder corners are the only speed-up inside the arm's limits" (TCP:122-125,626-636).
* Visible shake at speed (5-12 Hz) -> 6 Hz post low-pass (TCP:169-190).
* Fast arrival near a joint limit (G1 "rate") -> slow that approach (model cap / arrival hold 0.8) (TT:1377-1436; TCP:653-666); G1 "geometry" -> re-teach that part (TT:558-564).
* Step > 10 deg -> refuse; never upload (0x4000).
* Region/sub-region boundary speed jump (one-tick speed step) -> ramp the speed factor (<= 0.05 per line) (TT:224-226).
* Acceleration flagged (> 900 deg/s^2) -> do not raise acceleration beyond 600 deg/s^2 (1.5x departed 24-64 mm; 3x tripped collision detection) (TCP:158-161,675-682).
* Do NOT promise "slow down to be more accurate" in general: slowing high-acceleration stretches bought 0 % (LEAVE:338-355); slowing helps at reversals/limits only.

---

## 5. Merging, sequencing, transitions, collision checks (R8, R10)

* **No single-file merge of taught recordings in ALIX.** Taught rows run as a session: `taught_tracks.py flow 101 102 108 109 110 103 109 104 106 107 105 --side left --channel canfd --movej-speed 20` (TPR:84-104). Per row: certificate check -> confirm -> capture -> **pole then hand** (hand opened first if the pole moves) -> rest before (rm_movej) -> push check -> whole-file preflight -> bridge to start (<= 0.72 deg else movej) -> stream -> wait at rest -> rest after (TCP:421-466). Rest legs and bridges are joint-space movejs, **not collision-checked**, default speed 80 unless `--movej-speed` (TCP:563-565; CT:151-155).
* Upload-lane flow (`cleaning_tracks.py flow`): explicit movej bridge to each track's first point (default 80; 0 = controller's implicit transit), devices from each plan's `pre_stroke` (pole at 100 % feedback-verified, hand all-Modbus SPEED_SET 1000), hand released before a pole change, retreat along the plan's first post-stroke move; a failed verdict stops the flow; off-driver only (CT:137-159).
* Concatenation primitive: `plan_min_time.concatenate(parts, names, max_gap_deg=0.5)` refuses a boundary jump > 0.5 deg ("the player would step it in one tick"), drops a duplicated boundary sample (< 1 millideg), records spans; each part is parameterised with zero boundary velocity (tools/plan_min_time.py:142-169). Directly usable for R8 (bridges must be produced separately).
* Dual arm (`DAF`): steps are dual / serial / single / mechanism; concurrency only where the sidecar verdict says concurrent and the **windowed** FCL clearance >= 25 mm; idle arm parked at move_to_rest with its pole at 0.150 m (right) / 0.300 m (left) and held +-1 deg; bridges serialised under a barrier with the partner static; skew trip not armed until the stop distance (D1) is measured; `speed` cannot re-sync two players (DAF:19-47,97-104,146-215,3763-3830). Program replay is single-arm-only until gate G4(b) passes (DTCA:201-203,294-295). Dual start skew is unmeasured (MTP:220-221).
* Clearance source (`DAV`): FCL cross-arm separation computed from the track bytes with time = i x 5.5545 ms, seeded with each side's pole and hand joints (seeding mandatory: default pole/hand misplaces bases 150-225 mm and changes clearance, e.g. 8.52 vs 42.03 mm) (DAV:1-74). Arm-vs-arm only.
* Fixture/bin: `workspace/dual_arm_plan_collision.py:compute_dual_arm_plan_collisions` checks left-vs-right, self, a slim body set (`bin_link`, `glove_wearing_unit_arm`, toggled by `include_body`) and any `extra_managers` (fixture meshes passed by the studio server); returns colliding sample index/time/link pairs and +-window frames for the web overlay (workspace/dual_arm_plan_collision.py:586-648). The track lane does not pass fixture meshes (inferred) — the commode is not in its clearance numbers. Hand/pole seeding traps documented at workspace/dual_arm_plan_collision.py:385-424.

---

## 6. Run evidence (`runs/*exp01_*`)

| run | what | key numbers | source |
|---|---|---|---|
| 20260827T114056..122722 (5 push_test) | GUI replays of right_test.txt | clock ~5.1 ms/pt; ~half abort at J4 135.0 (3.27 deg margin, 4.5 deg overshoot) | exp01/notes/2026-08-27_phase_c_replay.md:1-17; exp01/README.md:171-197 |
| 20260828T212250 G1 | refused before upload: joints not enabled / e-stop latched | guard worked | run.json `refused` |
| 20260828T212857 G1 | first upload, LF header | 293 KB in 6.62 s, ret 1 err_line 1 | FIND:5768-5786 |
| 20260828T213858 G1 upload_only | CRLF upload slot 90 | 293,247 B, 6.31 s, ret 0/-1; push restore ret -2 (busy) | FIND:5799-5828 |
| 20260828T214135 / 214556 G1 run id 90 (speed 20) | uploaded track == GUI replay | track 29.16/29.54 s, dev p95 14.7/17.4 mm, 5.05-5.08 ms/pt | FIND:5839-5852 |
| 20260828T215215 G3 speed100 / 215303 speed10 | speed scales transit only; speed-10 run stopped at 46.1 % (J4 136.06) | transit 0.82 / 4.33 s | FIND:5843-5864 |
| 20260828T215446 G2 upload / 220731 run id 91 | respaced 5,835-pt file | 32.45 s measured vs 32.40 predicted (0.15 %) | FIND:5888-5900 |
| 20260828T221443 G4 upload / 221645, 221939 run id 93 | planned ruckig_on_s track (5,047 pts) | 28.12/28.25 s vs 28.03 planned; dev p95 11.1/10.6 mm; max 24-40 mm transients | FIND:5902-5933 |
| 20260829T191644 H1_upload_capped / 191701 H1_upload_fast | uploads for lane H (slots 95, 94) | 214,009 B 4.84 s; 180,153 B 4.04 s | run.json stages |
| 20260829T192028..192720 H0/H2/H3 runs (x2 each, speed 50) | speed test | 29.13 -> 21.55 -> 18.13 s (1.35x/1.61x), all complete, zero faults; control breached J4 135.055/135.366 | FIND:6677-6744 |

Note: `program` stage durations in run.json include transit + dwell (e.g. H3 19.95 / 19.60 s; H0 31.13 / 31.04 s) (runs/20260829T19*_exp01_H*/run.json `stages`, derived); the track terms above are from FIND.

---

## 7. Reuse assessment

### 7.1 Importable as-is (dependencies)
* `dispatch/` is ROS-free by ruling (dispatch/__init__.py:1-6); module-level imports are stdlib only (`track.py` imports `limits` and `runmode` at :42-43; numpy imported lazily inside functions); the SDK `Robotic_Arm` is imported lazily (dispatch/connect.py:157-162; dispatch/track.py:107). Needs `Robotic_Arm` (repo copy `external/butterfli-realman-alix/Python`, RU:59-72) and Linux x86_64/aarch64 for hardware (TQS:130-139; TCP:753-760).
  * `track`: `parse_track`, `build_project_file`, `send_project`, `program_list`, `preflight_slots`, `global_speed_guard`, `start_gap_deg`, `run_verified`, `run_program`, `stop_program`, `program_run_state`, `replay_verdict`, `verify_replay_from_frames`, `expected_replay_s`, `clock_screen`, `push_error_codes`, `UdpRecorder`, `ArmEventLog`, `canfd_preflight`, `stream_track`, `CanfdWatch`.
  * `connect.open_arm(side, ip)` with env overrides `RM_LEFT_IP`/`RM_RIGHT_IP`, ack timeout, `host_ip_for` (push target), emulated-program seam (dispatch/connect.py:25-35,85-118,141-182).
  * `runmode` (controller SIM=0 / REAL=1, apply/restore/require_real; "an arm left in SIMULATION is a phantom") (dispatch/runmode.py:1-33,205-252).
  * `ladder` (EMU -> SIM -> REAL gate keyed by program fingerprint; SIM truncation blocks REAL; emulator geometry not evidence) (dispatch/ladder.py:1-10,63-145).
  * `limits.guard` (Cartesian caps, read-back + restore) (dispatch/limits.py:386-458).
  * `recording.RunRecorder` (UDP push capture per arm, proves frames arrive) (dispatch/recording.py:1-21,44-60).
  * `peripherals.Pole.move` / `HandModbus.command` (feedback-verified pole, all-Modbus hand, `None` = hold channel) (dispatch/peripherals.py:1-50,227-262,601-625) — needs `ros/hw/alix_hw/scripts/hand_units.py` found by walking parents (dispatch/backends.py:31-47), so vendor that file for use outside the monorepo.
  * `emulated_program` (offline double of the five program verbs; for Lite's tests, not a user-facing sim) (dispatch/emulated_program.py:1-64).
* `tools/rm_upload.py`: stdlib + SDK only (RU:9-11) — already the basis of RUNNER.

### 7.2 Port (pure numpy/scipy functions) rather than import
* Importing TT pulls `cleaning_tracks`, exp01 `predict_track_replay` -> `teachlab` -> `alix_workspace.urdf_kinematics`, `plan_min_time`, scipy.signal/spatial (TT:79-92,257-261,438-439). The lane's own environment is six packages: numpy, scipy, toppra, PyYAML, matplotlib, tornado, in `.venv-taught` (TCP:718-746; TQS:114-128).
* On this machine the system python has numpy 2.4.6 with scipy 1.11.4 and `scipy.signal` fails to import (observed during this survey) — Lite must ship its own venv or pure-numpy ports (inferred risk).
* Port list (each small, deterministic, testable against ALIX outputs): `strip_pauses` (TT:915-951), `smooth_taught` (TT:894-912), `uniform_millideg` (TT:748-764), `capped_millideg` (TT:767-832), `rate_report`/`step_report`/`position_report` (exp01/src/predict_track_replay.py:92-136), `_point_to_polyline_mm` + `deviation_mm` (TT:607-716), `slowness_census` (TT:958-991), `rate_fit_track` (CT:1675-1714), `tick_step_check` (CT:1743-1750), acceleration-flag bands (CT:278-310), `screen_file` verdict logic (TT:4025-4132), `concatenate` (tools/plan_min_time.py:142-169), `tcp_gate.predict` (tools/tcp_gate.py:65-80).
* Optional (heavier): G1/G5 model (`path_reversals`, `pad_departure_joint_mm`, `model_predictions`, `model_caps`, `departure_lambda_cap`, `model_millideg`, TT:1002-1984) needs FK at `{L,R}_glove_frame_2` (exp01/src/teachlab/kinematics.py:139-212); TOPP-RA emitter (`emit_toppra_rung.py` + `local_speed.py`) needs toppra and is **left-arm only** (TCP:553-555).

### 7.3 Must preserve for hardware safety (Lite non-negotiables)
1. CRLF `.proj`, refuse malformed files, never half-upload (dispatch/track.py:66-98).
2. `only_save=1`, run as a separate confirmed step (dispatch/track.py:101-106).
3. Post-upload size (+ name) readback before any run; refuse mismatches (RU:165-173; CT:4107-4159).
4. Restore the global speed ratio after every run, on every exit path; it has no getter and persists (dispatch/track.py:161-198; FIND:5866-5881).
5. Pre-run refusals: empty slot, arm error code, program already running, unreadable state (RU:188-204; dispatch/track.py:592-600); announce the collision-unchecked transit gap (dispatch/track.py:323-338).
6. Ctrl+C / abort -> verified stop ladder (slow_stop -> pause -> stop); the program keeps playing if the client dies (dispatch/track.py:719-751,3109-3130; dispatch/emulated_program.py:58-62).
7. Byte gates before upload: G2 (0 ticks at 5.0 ms, 20 Hz BL), G3 (<= 10 deg), joint margin with overshoot budget (J4 executed past +-135), deviation vs the raw recording (G4) (§2.5; FIND:6754-6760).
8. Never write ALIX's production range 70-100 if controllers are shared; slots are persistent (CT:4-12,2763-2790).
9. One command source per arm; the ROS driver must be down (RU:25; DTCA:225-234; CT:4033-4043).
10. Check the controller run mode: SIM looks like a perfect run with nothing moving (dispatch/runmode.py:205-252).
11. Dual arm: concurrent program replay only with a measured windowed clearance >= 25 mm, transits serialised, otherwise serial (DAF:34-47,191-192; DTCA:201-203).
12. Do not upload while the same controller is moving/monitored (push gaps 400-600 ms during uploads, §1.5).
13. Provenance: side declared never guessed; `hardware: analysis_only` recordings never replayed (Resource/tracks/taught/source/README.md:14-27).
14. Do not exceed 600 deg/s^2 commanded acceleration for real runs (TCP:158-161,675-682).

---

## 8. Requirement relevance

* **R1** — ALIX's per-fixture `description.txt` (FILE / TASK / POLE / HAND / MAX DEV) is a ready region table (exp01/data/teach_files/commode_c_taught_paths/description.txt:1-20; TPR:11-40); `screen` verdicts READY / RETIME / NEEDS-PROVENANCE / ALREADY / REJECT give a per-region "raw .txt present and usable" dashboard (TREADME:95-114; TT:4025-4132). Provenance asked once per recording (TREADME:40-46). Timed prompting/auto-advance: not present (not-available).
* **R2** — per-region speed = TT `--section A:B:F` and lambda ladders with ramps; trimming = R4 anchor rule; smoothing recipes and budgets (§2.6, §3). Undo: not present (stateless CLI with immutable `source/` and rung files, TREADME:171-181) (not-available).
* **R3** — ALIX ladder EMU -> SIM -> REAL with fingerprint gating (dispatch/ladder.py:1-10,101-145); emulator geometry is not evidence (236 % path) (dispatch/ladder.py:68-72); browser sim/URDF playback is outside this lane (other maps).
* **R4** — `connect.open_arm(side, ip)` + env IP overrides + ack timeout (dispatch/connect.py:25-49,141-182; RU:45-48); controller SIM/REAL toggle via `rm_set_arm_run_mode` (dispatch/runmode.py:1-33,56-160).
* **R5** — real arm: 4109 collision code in the push (RU:55; dispatch/track.py:1653-1688), `phase_end` = commanded index nearest the arm's last captured position -> where to stop the plot (dispatch/track.py:2056-2069), CANFD watch trip carries the line index (dispatch/track.py:3945-3975); sim: colliding sample indices and link pairs (workspace/dual_arm_plan_collision.py:617-621). Caveat: high acceleration can trip 4109 with no contact (TCP:675-682).
* **R6** — not covered by this lane; ALIX analysis figures carry a <= 30-word description under every title (TCP:540-543).
* **R7** — pole/hand before motion: pole first then hand, hand released before a pole move, fingers % on channels 1-4 with the thumb held (TCP:431-445; TREADME:116-126); `Pole.move`, `HandModbus.command` (dispatch/peripherals.py:227-262,601-625); pole/hand recorded per row as `pre_stroke` (TPR:75-79; TTJ rows).
* **R8** — no taught-file merger; `concatenate` primitive (<= 0.5 deg boundary), flows with rest/bridge legs (collision-unchecked), dual-arm FCL clearance (arm-vs-arm) and fixture meshes only via `extra_managers` (§5).
* **R9** — no FK -> offset -> IK tooling in this lane (grep found none); validation half is reusable: FK (`ArmKinematics.fk_batch`), G4-style two-way deviation, G2/G3 jump checks, joint margins (exp01/src/teachlab/kinematics.py:139-212; TT:607-667).
* **R10** — dual executive rules, drift window 3.0 s / per-pair window / trip at 0.83 of it, idle pole heights, bin as `include_body` toggle (DAF:146-215; workspace/dual_arm_plan_collision.py:594-648).
* **R11** — gates are fast enough for interactive validation: certify 2-3 s, screen 0.25 s/file, exact deviation 5.7 s -> 0.2 s on 9.6k lines (TREADME:93,98; TT:682-685). Smoothing evidence §3.
* **R12** — §4 detectors and remedies.
* **R13** — §1: throughput ~45 kB/s fixed by 2 KB acks; fewer points, skip unchanged, one connection, restore speed; or CANFD (no upload).
* **R14** — same as R9; G1/G5 prediction gives a sim-side estimate of executed deviation before real runs (TT:1258-1374), upload-fitted only (TCP:345-356).
* **X1/X2** — ALIX's taught lane is pure joint space from the recording; "per-joint control is a path decision" belongs to authoring, out of the timing tool (TCTR:164-178) — consistent with Lite's exclusions.
* **NFR-UX** — plain-language refusal table and four-step quickstart are directly reusable copy (TPR:150-166; TQS:14-112); `MAX DEV` explained as one two-way knob (TQS:28-34).
* **NFR-THEME** — not touched by this lane.
* **INT-CTRL** — RUNNER is a fork of RU; add ALIX's evidence/stop/restore pieces (§1.8, §7.3).
* **INT-TRAJ** — HTML pipeline cross-check (§3, flags 1-4).

## 9. Reuse verdicts

| feature | verdict | what to lift | evidence |
|---|---|---|---|
| .proj builder + upload + size verify | reuse-as-is | `build_project_file`, `send_project`, `program_list` (page 10), size check | dispatch/track.py:85-152,371-386; RU:90-174 |
| connect with IP entry | reuse-as-is | `open_arm(side, ip)`, ack timeout 1500 ms, `host_ip_for` | dispatch/connect.py:37-49,85-118,141-182 |
| run + global speed restore | reuse-with-adaptation | `global_speed_guard`, `run_program`, start-gap announce/bridge | dispatch/track.py:161-198,323-338,592-667,3517-3610 |
| completeness evidence | reuse-with-adaptation | blocking probe (-4), finish event, `replay_verdict`, `verify_replay_from_frames` | dispatch/track.py:447-490,1690-1794,2037-2072 |
| verified stop ladder | reuse-as-is | `stop_program` | dispatch/track.py:3069-3130 |
| SIM/REAL controller mode | reuse-as-is | `runmode.apply_run_mode/require_real` | dispatch/runmode.py:56-160,225-252 |
| EMU->SIM->REAL gating | reuse-with-adaptation | fingerprint ledger logic | dispatch/ladder.py:63-145 |
| UDP run capture | reuse-with-adaptation | `UdpRecorder` / `RunRecorder`, `push_error_codes` | dispatch/track.py:974-1370,1653-1688; dispatch/recording.py:1-60 |
| pole / hand before teach | reuse-with-adaptation | `Pole.move`, `HandModbus.command` (vendor `hand_units.py`) | dispatch/peripherals.py:227-262,601-625; dispatch/backends.py:31-47 |
| pause removal | reuse-with-adaptation (port) | anchor rule 0.05 deg / 3 lines | TT:915-951 |
| de-jitter smoothing | reuse-with-adaptation (port) | 5 Hz order-2 zero-phase on 5 ms record clock | TT:880-912 |
| per-region speed scaling | reuse-with-adaptation (port) | `uniform_millideg`, `capped_millideg` with ramps, `--section` semantics | TT:748-832,1439-1451 |
| rating auto-fix | reuse-with-adaptation (port) | `rate_fit_track` | CT:1675-1714 |
| spike gates G2/G3 | reuse-with-adaptation (port) | `rate_report`, `step_report`, 20 Hz BL at 5.0 ms | exp01/src/predict_track_replay.py:82-136 |
| deviation G4 | reuse-with-adaptation (port) | `_point_to_polyline_mm`, `deviation_mm` (needs FK) | TT:607-716 |
| execution model G1/G5 | reference-only (optional port) | EM constants, departure law, attribution | EM:1-32; TT:1002-1374 |
| model speed ladder | reference-only | ladder structure, pick rule, ramps | TT:139-161,2559-2950 |
| TOPP-RA arc-length emitter | reference-only (left-arm only, toppra) | recipe and limits | exp01/src/local_speed.py:139-324; TCP:17-221 |
| MAX DEV two-way knob | reuse-with-adaptation (UX concept) | 5/10/20 mm accuracy knob, envelope formula | TP:107-141; tools/tcp_gate.py:52-80 |
| screen / dashboard verdicts | reuse-with-adaptation | READY/RETIME/NEEDS-PROVENANCE/ALREADY/REJECT | TT:4025-4132; TREADME:103-107 |
| description.txt region table | reuse-with-adaptation | FILE/TASK/POLE/HAND/MAX DEV columns | exp01/data/teach_files/commode_c_taught_paths/description.txt:1-20; TP:183-221 |
| concatenation for merges | reuse-with-adaptation | `concatenate` (<= 0.5 deg gap) | tools/plan_min_time.py:142-169 |
| dual-arm executive | reference-only | step classes, barriers, clearance >= 25 mm, idle hold | DAF:1-215 |
| dual-arm clearance sweep | reuse-with-adaptation | `compute_dual_arm_plan_collisions` (`include_body`, `extra_managers`) | workspace/dual_arm_plan_collision.py:586-648 |
| CANFD streaming | reference-only | preflight, watch, absolute-deadline streamer | dispatch/track.py:3680-4099 |
| config->program builder | not-applicable | — | dispatch/program.py:1-20 |
| offset FK->IK tool | not-available | — | (grep: none in this lane) |
| undo history, timed prompts | not-available | — | — |

## 10. Gaps, risks and open questions (short)

* G5's model is upload-fitted and optimistic in its tail (worst case ~2x the budget); it does not rank CANFD files (TCTR:93-99; TCP:345-356,551-552).
* The TOPP-RA emitter is left-arm only and reads only `SOURCE.json` (TCP:553-555).
* G4 on a derived file compares against itself (0.0 mm) (TCP:311-343).
* Player clock varies +-6 % (dual-arm drift, durations as bands); dual start skew unmeasured (dispatch/track.py:45-50; MTP:220-221).
* `rm_set_arm_stop` effect on a running program untested; no program-scoped stop (DTCA:19-29,185-190).
* J4 limit crossed by servo overshoot on the production track; cold arm doubles off-curve error (FIND:6699-6737).
* Lite V6 uses program ids 71+ (ALIX production range) and `speed: 100` (persistent global ratio).
* RUNNER lacks capture, completeness evidence and the verified stop ladder (§1.8).
* Environment: numpy/scipy ABI mismatch on the host python (observed).
* Open: upload-only or also CANFD for tests? Which gates hard vs advisory in Lite? Vendor ALIX modules into Lite or depend on the monorepo? Firmware V1.7.4 vs V1.7.5 on the deployed controllers? Shared controllers with ALIX production slots?
