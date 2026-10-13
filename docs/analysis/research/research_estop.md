# Research: RealMan IO-wired emergency stop (user decision F4)

Date: 2026-10-13. Scope: how the IO-wired E-stop on the two RM75-6FB Gen-3 controllers works, how
software sees it, how to recover, how it relates to the SDK's software stops, and what Lite must do.

Sources and citation style
- Manuals: `/home/newtonjeri/alix_ws/src/alix/docs/Manuals/Realman/*.pdf`, converted with
  `pdftotext -layout` to `.../scratchpad/estop/*.txt`. Page numbers below are PDF pages (they match the
  printed page numbers in this offline mirror, site docs V1.7.14, snapshot 2026-09-01).
  Short names: HW = 02_RM75_Hardware_and_Getting_Started; TP = 03_Teach_Pendant_and_Configuration;
  JSON = 04_JSON_Protocol; CPP = 05_API2_C_Cpp; PY = 06_API2_Python; ROS2 = 07_ROS2;
  DEMO = 08_Demos_C_and_Python; MB = 09_Modbus_FAQ_Downloads; REL = 01_Overview_and_Release_Notes;
  CN = 睿尔曼超轻量仿人机械臂(RM65&RM75系列)-V1.2.0-用户使用说明 (Chinese user manual V1.2.0).
- SDK: `/home/newtonjeri/alix_ws/src/alix/external/butterfli-realman-alix/Python/Robotic_Arm/`
  `rm_robot_interface.py` (RRI) and `rm_ctypes_wrap.py` (RCW); C header `C++/include/rm_interface.h`.
- ALIX: `/home/newtonjeri/alix_ws/src/alix/...` (paths given in full or relative to `alix/`).
- Runner: `/home/newtonjeri/alix_studio_lite_ws/upload_channel/rm_upload_fast_v14_safe_transit.py` (v14).
- Installed firmware: both arms Gen-3 controller V1.7.4, API2 1.1.6 on the host
  (`alix/external/butterfli-realman-alix/docs/arm_version_info.md` §1).

Legend: **[DOC]** stated by RealMan docs; **[ALIX]** measured/observed and written down in ALIX;
**[INF]** my inference; **[GAP]** not covered anywhere I could find — must be measured in the drill.

---

## 1. How an IO-wired E-stop works on these Gen-3 controllers

### 1.1 The input and its function
- **[DOC]** The controller's 16-core "multiplexing I/O" cable carries four multiplexable IOs, IO1–IO4 on
  pins 5–8 (HW p.39; CN p.24). The HW p.18 interface table names the external E-stop button box as the
  example use of this connector and refers to a separate "External Emergency Stop Button Box" page.
  That page is **not** in the offline mirror **[GAP]**.
- **[DOC]** Each IO can be switched by program command or by the web pendant to one of several functions.
  Function **5 = "Input emergency stop function multiplexing mode"** (HW p.40; CN p.26 "输入急停功能复用模式").
  - Pendant: Configuration → Controller IO → Digital Input/Output → "Input Emergency Stop Function
    Multiplexing" (TP p.112, p.131).
  - JSON: `{"command":"set_IO_mode","IO_Num":1-4,"IO_mode":5}` (JSON p.118), read back with `get_IO_state`
    (JSON p.120).
  - SDK: `rm_set_io_mode(io_num, 5)` / `rm_get_io_state(io_num)` (PY p.157, p.159; RRI:2882, RRI:2927).
- **[DOC]** Electrical: inputs are NPN type. They need the external input reference OUT_P_IN (pin 9) and
  OUT_P_GND (pin 11). Logic low is ≤ 1 V and logic high is ≥ Vin−0.5 V, up to 24 V (HW p.39–40).
- **[DOC]** Trigger level: the IO demo says "The IO is triggered by a high-level signal" for the
  start/pause/continue/E-stop functions on IO1–IO4 (DEMO p.82; the C demo puts the E-stop on IO4, DEMO p.25–26).
  - **[INF]** As documented, this is an "energise to stop" input. A broken wire or a lost OUT_P_IN reference
    would then not trigger a stop. Confirm how the robot's buttons are actually wired: NO or NC contact,
    and to which IO.
- **[DOC]** A second hardware path may exist. The Chinese manual V1.2.0 lists **pin 14 = E_STOP
  "电路式 IO 急停"** (circuit-type E-stop) (CN p.24). The English Gen-3 page lists pin 14 as
  "Blank / Reserved" (HW p.39). Which of the two (IO1–4 in mode 5, or pin 14) the robot uses is
  **[GAP]**. The user says the button is "wired to an IO", which suggests IO1–4 in mode 5. Lite can
  confirm this by reading `rm_get_io_state(1..4)` and looking for `io_mode == 5`.
- **[DOC]** RealMan's own Safety Guide says: "Connect safety devices to safety interfaces instead of
  normal I/O interfaces" (HW p.32; CN p.7 "不要将安全设备连接到正常的 I/O 接口上，只能使用安全型接口").
  - **[DOC/GAP]** The IO-multiplexed E-stop is a function of the controller software on a general I/O.
    No document states its stop category (IEC 60204-1 cat. 0/1/2), its PL/SIL rating or its reaction
    time.
  - This does not contradict F4: the user reports that it works well in practice. But it is the reason
    the drill in §5.6 must measure the stop behaviour and not assume it.

### 1.2 What it stops, whether power is kept, brakes
- **Arm joints.**
  - **[DOC]** The pendant E-stop "stops the robot at maximum speed. Pressing it again cancels the
    emergency stop" (TP p.11).
  - **[DOC]** The software quick stop `set_arm_stop` "Stop[s] joints at maximum allowable joint speed;
    trajectory cannot be resumed. The manipulator maintains power to perform controlled stop, and power
    supply remains active after stopping" (JSON p.61; PY p.84).
  - **[DOC]** The API2 error list groups "sending an E-Stop JSON protocol, triggering an IO E-Stop, or
    triggering the E-Stop on the teach pendant" as the same kind of external stop (CPP p.318).
  - **[DOC]** The UDP status for this state is `"stop": emergency stop state` (JSON p.131).
  - **[INF]** The IO E-stop is therefore most likely the same controller quick stop: maximum
    deceleration with power kept, roughly a stop category 2. That matches the user's observation that
    it "stops everything without power cut". The category itself is **[GAP]**.
- **Power kept or not: the sources conflict, so measure it.**
  - User (F4): no power cut.
  - **[ALIX]** `alix/ros/hw/alix_hw/README.md:449` says "the physical e-stop chain cuts joint power (all
    enable flags drop in the UDP frame)". `alix_hw/src/rm_side_system.cpp:1855,1921` says "a physical
    e-stop drops joint power", and `:2208` says "a physical e-stop chain drop is the only thing that
    clears them all" (the enable flags).
  - **[ALIX]** `docs/TEST_PROCEDURE.md:231` treats "Arm power OFF" as the "e-stop chain".
  - **[INF]** These can both be true. The controller, the network and the end port stay powered (no
    power cut at the supply), while the joint drives are disabled and the joint enable flags drop.
  - Which is the case for the IO input on these two controllers is **[GAP]**. Record
    `joint_en_flag`, `rm_get_arm_power_state` and `brake_state` in the drill.
- **Brakes.**
  - **[DOC]** RM75: joints 1–4 have a hard brake and joints 5–7 a soft brake (HW p.3). There is no brake
    inside joints 5–7, so the wrist can fall under gravity when unpowered (HW p.34).
  - Brake state can be read per joint with `get_joint_err_flag` → `brake_state` (1 = brake not engaged,
    0 = engaged) (JSON p.20; RRI:459).
  - Whether the IO E-stop engages the brakes is **[GAP]**.
- **Lift / pole.**
  - **[DOC]** Not covered.
  - **[ALIX]** On 2026-08-06 at 20:38, with the E-stop pressed, both controllers rejected every lift
    command with `set_state:false` / ret 1. Getters still worked. Releasing the E-stop recovered
    (`docs/TEST_PROCEDURE.md:216-242, 453`; emulated in `core/python/src/alix_emulator/rm_emulator.py:837`).
  - So new pole commands are refused. Whether a pole that is already moving is stopped is **[GAP]**.
  - Note: the software `rm_set_arm_slow_stop` does **not** stop the pole. The column kept travelling for
    up to 2.6 s, and the pole needs `rm_set_lift_speed(0)`
    (`core/python/alix_dispatch/src/alix_dispatch/peripherals.py:189-199`).
- **End tool / hand (Modbus on the end port).**
  - **[ALIX]** "end-port power survives, so the hand keeps working" during the E-stop
    (`docs/TEST_PROCEDURE.md:453`).
  - So the IO E-stop does **not** stop or disable the hand, and hand commands still execute.
  - **[INF]** "Stops everything" in F4 should therefore read as "stops the arm motion and refuses new
    arm and pole motion". Lite must freeze the hand itself.
- **One chain or two.**
  - **[ALIX]** "one e-stop chain covers both arms" (`docs/TEST_PROCEDURE.md:453`).
  - User (F4): a button is wired to an IO on each of the left and right controllers.
  - It is **[GAP]** whether one press trips both controllers. It could be one button wired to both
    inputs, or two buttons. The drill must answer this.

---

## 2. How software detects it

| Signal | Call / field | What it says during an IO E-stop | Reliability |
|---|---|---|---|
| **E-stop input level** | `rm_get_io_state(n)` → `{io_state, io_config:{io_mode}}` (RRI:2927; JSON `get_IO_state` p.120; PY p.159) | `io_mode == 5` identifies the E-stop port. `io_state` is 1 = high or 0 = low; per DEMO p.82, high = triggered | **Best direct signal [INF]**. Polarity must be confirmed in the drill |
| All 4 DI at once | `rm_get_io_input()` → list, 1/0, **-1 = "port not in input mode"** (RRI:2961; PY p.160; JSON p.124) | A mode-5 port may read -1, because it is not "general input" **[GAP]** | Use `rm_get_io_state` instead |
| **Modbus input register 302** "Emergency Stop Status" | Modbus-TCP, port 502, device 1 (MB p.3, p.8) | 0 = not in E-stop, 1 = in E-stop. "triggered by an emergency stop operation and directly cleared upon release" (REL p.14, added in V1.6.7; installed V1.7.4) | **Explicit E-stop flag.** Modbus-TCP must first be enabled on the pendant (TP p.125). It adds a second channel |
| UDP push `arm_current_status` | RCW:3272-3290 (`RM_STOP_E = 9`); JSON p.131 `"stop": emergency stop state`; ROS2 p.113; PY p.308 | 9 = STOP | **Not usable on its own.** [ALIX] it is a sticky "last run state". After **any** stop (software stops included) it stays 9 until the next motion, and `clear_system_err` does not reset it (`alix_hw/src/rm_side_system.cpp:1856-1860`; `experiments/exp01_.../src/live/common.py:203`) |
| Joint enable flags | UDP `joint_status.joint_en_flag[7]` (RCW:3385-3417); `rm_get_arm_all_state()['joint_en_flag']` (RRI:1298) | [ALIX] all drop on a physical E-stop chain (`rm_side_system.cpp:1854-1876, 2200-2215`) | Good if the IO E-stop really disables the drives **[GAP]** |
| Arm power | `rm_get_arm_power_state()` → 1/0 (RRI:2251; JSON p.104) | [ALIX] treated as OFF ⇒ e-stop chain (`TEST_PROCEDURE.md:231`) | **[GAP]** |
| Brake state | `rm_get_joint_err_flag()['brake_state']` (RRI:459; JSON p.20) | 0 = engaged | **[GAP]** |
| System / arm error code | `rm_get_current_arm_state()['err']['err']` (RRI:1150; RCW:1826-1874); `rm_get_controller_state()['system_error']` (RRI:2198) | **No E-stop code exists.** The controller system-error table (0x0000…0x5009) and the joint-error bitmask (0x0001…0xF000) contain no emergency-stop entry (JSON error appendix; `alix/docs/hardware-sdk/ERROR_CODES.md`) | An E-stop most likely leaves `err` = 0 **[INF]**. A secondary joint code (for example 0x2000 brake, 0x0004 undervoltage) is possible if the drives drop **[GAP]** |
| API2 return value **-6** | Blocking motion calls: "The robotic arm has stopped its motion planning due to an external stop command … triggering an IO E-Stop" (CPP p.318; RRI:1568 and similar; `ERROR_CODES.md:28`) | Returned by a **blocking** move that the E-stop interrupts | Does **not** apply to `rm_set_program_id_run` with `block=0` (the runner's call) |
| Command refusals | Drag teach start fails "in an IO emergency stop state" (JSON p.154; `rm_interface.h:2365, 2384`); lift commands ret 1 [ALIX] | ret 1 = "controller returned false: parameter error or arm-state error" | Indirect only, with no reason given |
| Event callback | `rm_get_arm_event_call_back` event types are only trajectory-arrival and program-finished (RCW:1025-1033) | No E-stop event | — |

### 2.1 Would runner v14 report it? Very likely not.
- The run loop (v14:219-251) and the sequence loop (v14:641-675) watch only
  `rm_get_program_run_state().run_state` and, every 0.5 s, the arm error codes from `arm_state()`
  (v14:122-138).
- The end-of-run verdict (v14:262-271) is "RUN COMPLETE … no error" when the program ended
  (`run_state` went to 0), it was not stopped by the runner, and no non-warning error code was seen.
- **[INF]** The IO E-stop ends the running program (the trajectory "cannot be resumed", JSON p.61) and
  raises no system error code (see above). The runner would then print
  `RUN COMPLETE: x s, no error` and **exit 0**. That is a false success.
- In `--sequence` it would print `SEQUENCE PROGRAM COMPLETE k/n` (v14:686-700) and go on to the next
  step:
  - a hand AUX step still runs, because the end port is alive [ALIX];
  - a pole step is refused (ret 1);
  - `rm_set_program_id_run` is refused while the E-stop is held, which prints
    `NOT STARTED: rm_set_program_id_run ret=1` with no reason (v14:628-630).
  - **If the operator has already released the button when the next start is issued, the next program
    starts by itself.**
  - All of this is **[INF]**: whether the program ends with `run_state` 0 or sits paused (2) is
    **[GAP]**. If it pauses, the runner waits until `--timeout` and then slow-stops (v14:240-244).
- Before a run, the runner refuses only on error codes or on a program already running
  (v14:192-205, 611-622). It has no E-stop or IO check. The v53 UI only shows text reminders such as
  "Software Slow Stop is not the hardware E-stop" (`rm_trajectory_ui_v53_safe_lid_checkpoint.py:3177`)
  and has no E-stop state.
- `SEND_RET[-6]` in v14:54 is a label for `rm_send_project` ("4th-generation controller does not
  support this"). It is not the -6 E-stop code of the motion calls.

---

## 3. Recovery

What the documents say:
- **[DOC]** The Modbus E-stop status "is triggered by an emergency stop operation and directly cleared
  upon release" (REL p.14). The IO E-stop is therefore level-based: the state ends when the button is
  released and the input returns to its idle level.
- **[DOC]** The pendant's own E-stop is a toggle ("Pressing it again cancels the emergency stop",
  TP p.11; Quick Menu "Robot Emergency Stop/Recover", TP p.3). This is the pendant's own stop, not the IO.
- **[DOC]** `clear_system_err`: "Manually clear the system error. If not cleared manually, the error will
  remain until a new motion command is issued" (JSON p.93; PY p.98; `rm_clear_system_err()` RRI:2427).
  - Per-joint errors need `rm_set_joint_clear_err(joint)` (RRI:220).
  - Re-enabling a joint uses `rm_set_joint_en_state(joint, 1)` (RRI:183), or the pendant at
    Configuration → Arm configuration → safety configuration (CN p.51 FAQ 3, "点击上使能按钮").
- **[DOC]** After `set_arm_stop` the trajectory cannot be resumed (JSON p.61). After pause, it can
  (`rm_set_arm_continue`, RRI:2114).
- **[GAP]** No document states whether the joints need re-enabling, or the errors need clearing, after
  an IO E-stop is released.

What ALIX does or found:
- **[ALIX]** Recovery ladder: "release/reset the physical e-stop → `--clear-errors` → Web GUI lift
  panel → power cycle" (`docs/TEST_PROCEDURE.md:240-242`; table row :453, "Release/reset the e-stop,
  rerun").
- **[ALIX]** `alix_hw` activation gate: up to two rounds of `rm_set_joint_clear_err` on every faulted
  joint, then `rm_clear_system_err`, each checked against a fresh frame. Otherwise: "if the physical
  e-stop is engaged, release it and relaunch; a persistent joint error code may need a power cycle
  from the pendant" (`alix_hw/src/rm_side_system.cpp:1919-1935`).
- **[ALIX]** Per-joint codes can survive the system clear (joint 3 error 16384, 2026-07-23, same file).
- **[ALIX]** A truncated move can latch 16384, and `rm_clear_system_err()` returns 0 without clearing it
  (`alix_dispatch/program.py:236-240`). So read back after clearing; never trust the return code alone.

State of the arm afterwards (**[INF]** unless marked):
- The arm stands where it stopped, part-way along the region path.
- The program is ended and cannot be resumed.
- `arm_current_status` stays 9 until the next motion [ALIX].
- The global speed ratio stays at the run speed. v14 restores it in `finally` (v14:259-261), but only
  if the runner is still alive.
- The pole is wherever it stopped [GAP]. The hand is unaffected [ALIX].
- **The next program run first makes the controller's own joint move from the current pose to the
  program's first point (v14 header lines 28-29, v14:207-214).** After a mid-path E-stop, this transit
  starts from an unusual pose close to the commode or wall. It is the most likely collision after
  recovery.

---

## 4. Software emergency stop vs slow stop (SDK)

| Call | What it does | Notes |
|---|---|---|
| `rm_set_arm_stop()` (RRI:2084; JSON `set_arm_stop` p.61; PY p.84) | "Trajectory emergency stop": joints stop at maximum allowable speed, the trajectory **cannot be resumed**, and **power stays on** | Gen-3 quick stop. Not latched: the next command can move the arm |
| `rm_set_arm_slow_stop()` (RRI:2069; JSON p.62; PY p.84) | "Stop on the currently running trajectory": a controlled deceleration along the path | Used by v14 for Ctrl+C, timeouts and v53 STOP (v14:257, 679, 731) |
| `rm_set_arm_pause()` / `rm_set_arm_continue()` (RRI:2099/2114) | Pause on the trajectory, resumable | — |
| `rm_set_arm_emergency_stop(state)` (RRI:7226-7240; `rm_interface.h:4518-4529`) | A latched E-stop set/recover | **Gen-4 only.** It returns **-4, "Gen-3 controllers do not support this interface"**. `arm_version_info.md:71` lists the "latched Gen-4 e-stop" as beyond the Gen-3 hardware |
| Modbus holding register 3 = 1 (MB p.5) | "Control program: 1 emergency stop, 2 pause, 3 continuation" for the online program | Another software path. It needs Modbus-TCP enabled |
| Pole `rm_set_lift_speed(0)` | Stops the pole | No arm stop does this (`peripherals.py:189-199`) |
| Hand freeze | ALIX's halt sends arm stop + lift stop + hand freeze together (`alix_hw/src/rm_side_system.cpp:1830-1850`) | — |

- **[ALIX/GAP]** Whether `slow_stop` or `stop` ends a **running stored program** (`rm_set_program_id_run`)
  is unproven: hardware gate G4 has not been run (`alix_dispatch/emulated_program.py:51-56`;
  `ALIX_Studio_Lite_Requirements_Revised.md:85`). F4 keeps this test on the plan.
- All software stops depend on the PC, the Lite process, the runner process, the network and the TCP
  connection being alive. The IO E-stop depends on none of them. This is why F4 makes the IO E-stop
  primary.

---

## 5. Recommendations for Lite

### 5.1 E-stop status chip, one per arm (LEFT, RIGHT)
- **States:**
  - `CLEAR` (green);
  - `E-STOP ENGAGED` (red, the whole header strip turns red);
  - `RECOVERING` (amber: released but not yet recovered);
  - `UNKNOWN` (grey: no fresh reading for more than 1 s, or the controller is unreachable);
  - `NOT CONFIGURED` (amber: no IO in mode 5 on this controller).
- **Sources, in order:**
  1. `rm_get_io_state(n)` on the IO whose `io_mode == 5`. At connect, scan IO1–4 and show "E-stop on
     IO4" in the tooltip. The active level comes from config, default high (DEMO p.82), and is confirmed
     by the drill.
  2. Optional Modbus input register 302, if the site enables Modbus-TCP. It is a cross-check.
  3. Joint enable flags from `rm_get_arm_all_state` or the UDP push: all disabled = "drives off".
- **Do not** drive the chip from `arm_current_status == 9` (it is sticky, see §2). Show it only as
  "last stop: STOP".
- Poll at 5–10 Hz through the **single connection owner** per arm (NFR-SAFETY). Keep the read-only
  status calls in the same process that runs programs, so polling does not open a second competing
  connection. The SDK ships both `libs/linux_x86` and `libs/linux_arm` (aarch64) `libapi_c.so`, so the
  same code runs on the operator PC and on the Jetson (F1).
- **Fail closed:** `UNKNOWN` and `NOT CONFIGURED` block motion exactly like `ENGAGED`. A supervisor
  override for `NOT CONFIGURED` may be allowed in SIMULATION mode only.

### 5.2 Block motion while it is active
- While **either** arm's chip is not `CLEAR`, Lite disables and refuses on the server side:
  Run, Sequence, Test-on-robot, Move-to-start, Pole, Hand, Drag-teach START, Run-mode REAL switch.
  - This covers dual-arm programs and both-arm teaching. The other arm is blocked too, because one chain
    may cover both arms [ALIX].
- Upload stays allowed: it does not move the arm (F6 slots 1–100). Show a "robot in E-stop" note.
- If the chip goes to ENGAGED **during** a run, sequence or teach, Lite:
  1. marks the run `INTERRUPTED — E-STOP`, never `COMPLETE`;
  2. cancels the rest of any sequence, so no next program and no AUX step is started;
  3. sends pole stop (`rm_set_lift_speed(0)`) and hand freeze on both sides, because the IO E-stop does
     not stop the hand [ALIX] and may not stop a moving pole [GAP];
  4. logs IO state, enable flags, power, brakes, err, `run_state`, `plan_num` and joints, with
     timestamps.
- **Runner v14 change (small, required):**
  - add the E-stop IO read to `arm_state()` and to the run/sequence loops;
  - make the verdict "interrupted" if the E-stop was seen at any time during the run;
  - also make it "interrupted" if `run_state` dropped to 0 well before the program's expected duration
    while the enable flags dropped;
  - refuse to start (with reason "E-stop engaged on LEFT, IO4") when the E-stop is active;
  - print a parseable line, for example `ESTOP ENGAGED <side> io=<n>` and `ESTOP CLEAR <side>`, for
    Lite to pick up.

### 5.3 Guided recovery (wizard, per arm or both)
1. "Make the cell safe. Check nobody is in the arm's reach." The operator ticks to confirm.
2. "Release the E-stop (twist or pull)." Wait for the chip to show the released level. The chip changes
   to RECOVERING, never straight to CLEAR.
3. Read and show the controller state in plain words: system err, per-joint err flags, enable flags,
   brake state, power state, run mode, `run_state`, pole height and error, hand state.
4. **Clear errors** button: `rm_clear_system_err`, then `rm_set_joint_clear_err` on each faulted joint,
   at most two rounds. **Read back each time** and do not trust ret 0 [ALIX].
5. If any joint is still disabled: offer **Re-enable joints** (`rm_set_joint_en_state`, one joint at a
   time, with the wrist supported). Wrist joints 5–7 have no brake (HW p.34). Otherwise give the pendant
   or power-cycle instruction text from ALIX's ladder.
6. Restore the global speed (`rm_set_plan_speed(restore)`), because the runner may have died before its
   `finally`.
7. Show the arm's **current pose** in the 3D view against the program's first point. Before any re-run,
   **collision-check the transit move** (current pose → first point) against the commode_c scene, the
   bin, the poles and the venue **walls** (F5), and show its size ("up to 38° on J2"). This needs
   explicit confirmation.
8. Re-home the pole if it stopped mid-stroke.
9. Only when everything above is clean does the chip go `CLEAR` and motion buttons re-enable.
   An "interrupted" run is never auto-resumed. The operator restarts the region (or a sequence from a
   chosen step) on purpose.

### 5.4 How the software STOP relates
- Lite's on-screen **STOP** (and its key or foot switch) sends:
  - `rm_set_arm_slow_stop` on the moving arm(s);
  - `rm_set_lift_speed(0)` and hand freeze;
  - cancel of the sequence.
- An optional **Quick stop** uses `rm_set_arm_stop`.
- Label it "STOP (software)". It is a convenience and a "stop the run cleanly" control, **not** the
  safety stop. Keep the text "Hardware E-stop is the safety stop" (as in v53).
- Keep the requirement that the software STOP path does not depend on the browser UI (NFR-SAFETY): a
  server-side or runner-side stop and a stop-on-disconnect watchdog.
- The planned one-hour hardware check of the software STOP (G4) stays. If `slow_stop` turns out **not**
  to end a running stored program, Lite's STOP must use whichever verb is proven. ALIX already supports
  both `aborts` and `ignores` policies (`alix_dispatch/emulated_program.py:51-56`).

### 5.5 Start-up / configuration checks
- On connect to each controller, read IO1–4 modes. Warn if no IO is in mode 5, and warn if the two
  controllers differ in which IO is used.
- Never let Lite **write** IO modes. Changing a safety input is a supervisor or pendant task.
- Store the configured E-stop IO number and active level per arm in the site config, with the date of
  the last passed drill. Show "E-stop drill overdue" after N days. RealMan asks for safety functions to
  be tested at least yearly (HW p.35); a weekly or per-site check is recommended here.

### 5.6 What to test: the E-stop drill
Run it per arm and with both arms, in REAL mode, with the workspace clear. Record the UDP push and the
polled state at 10 Hz throughout.

| # | Situation | Press | Observe / pass condition |
|---|---|---|---|
| DR1 | Both arms idle | LEFT button | Does LEFT trip? Does RIGHT trip too (one chain or two)? `io_state` level, `io_mode` = 5 on which IO, Modbus 302, `joint_en_flag`, power state, `brake_state`, `arm_current_status`, err codes. Lite chip turns red within 0.5 s |
| DR2 | Both arms idle | RIGHT button | As DR1, mirrored |
| DR3 | LEFT running a stored program (slow, safe region) | LEFT button | Arm stop distance and time. `run_state` after (0 or 2?). err codes. Runner v14 verdict (expect false "RUN COMPLETE" until fixed). Lite shows INTERRUPTED |
| DR4 | RIGHT running a stored program | RIGHT button | As DR3 |
| DR5 | Both arms running (dual-arm program / parallel runs) | one button, then repeat with the other | Do both arms stop? Lite cancels both |
| DR6 | Pole moving | button | Does the pole stop by itself? Does Lite's pole stop fire? |
| DR7 | Hand moving / closing | button | Confirm the hand is **not** stopped by the E-stop and that Lite freezes it |
| DR8 | Drag teach active | button | What happens to drag mode and the recording? Is the recording still saved? |
| DR9 | E-stop held | try every Lite motion action and a runner `--run` | All refused by Lite with the reason. The controller also refuses (ret 1) |
| DR10 | Sequence running, release the button quickly | button, release within 2 s | **No** next program or AUX step starts by itself (fix of v14 behaviour) |
| DR11 | After release | recovery wizard | Errors clear. Re-enable is or is not needed (record which). Transit check shown. Re-run works |
| DR12 | PC/Lite failure | unplug the network cable or kill Lite during a run, then press | The IO E-stop still stops the arm (independent of the PC) |
| DR13 | Wiring integrity (with the electrician) | open-circuit the E-stop wire | Does the controller trip? If not (high-trigger input), record it as a known limitation and decide whether to rewire (NC + safety relay) |
| DR14 | Software STOP (G4, kept per F4) | Lite STOP during a stored-program run, `slow_stop` and `stop` | Does the program end? Stop distance. Pole and hand also stop |

Record every result in the site config / drill log. DR1, DR3, DR9, DR10 and DR11 must pass before operators
run real tests with Lite.

---

## 6. Not covered by the documents (open)
- The "External Emergency Stop Button Box" page (HW p.18 reference) is missing from the mirror.
- Stop category, safety rating and reaction time of the IO E-stop.
- Brake engagement and drive disable on the IO E-stop. Whether power or the enable flags drop (ALIX code
  says the joints lose power; the user says no power cut).
- Whether the E-stop is latched in the controller after release, or whether a re-enable or clear is needed.
- Whether a moving pole is stopped. The hand is not (ALIX field).
- Whether `rm_get_io_input` reports -1 for a mode-5 port. The active level on this robot's wiring.
- Whether one button trips both controllers.
- Whether the program ends (`run_state` 0) or pauses (2) on an IO E-stop.
- Pin 14 `E_STOP` (CN manual) versus IO1–4 mode 5: which one the robot uses.
