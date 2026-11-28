# Runbook: connect the N100 and check its data (step A to C of PLAN.md), 2026-11-25

**Purpose.** Do the IMU's bytes, rates, axes and mount give data we can trust? If any check fails, the measurement
runs with the arms (plan step E) wait until it passes.

**Nothing in this runbook moves the robot.** The arms may be powered for the last noise captures, but no motion
command is sent.

## Verified offline (here, on the development laptop; no IMU here)

- `tools/imu_n100.py` compiles. `--help` and `ports` run (`ports`: "No USB serial device ... is plugged in", as
  expected here).
- `tests/test_imu_n100.py`: 15 passed with `/usr/bin/python3` and 15 passed with Lite's `.venv`. They cover:
  - the CRC tables match the drivers' tables and give the standard check values (CRC-8/MAXIM 0xA1, CRC-8/SMBUS 0xF4,
    CRC-16/XMODEM 0x31C3 on "123456789");
  - frames are found again past noise, a damaged frame and a cut frame;
  - the unit's CRC8 table is found from its frames;
  - the still, turn and tap reports are right on synthetic captures (lost frames, rate, gravity axis, gyro bias, a 90
    deg turn, a 25 Hz ring, and a loose mount flagged);
  - the exact command sequence of `config` against a stand-in unit;
  - the request frame's CRC and the version packet's decoding;
  - capture through a real serial device (a Linux pseudo-terminal at 921600, the same `Port` and recording loop).
- **Not checked here (what this run is for):**
  - which CRC8 table the unit really uses;
  - whether its firmware answers `#fmsg` and the request packet;
  - the real heartbeat frame;
  - the real rates and losses;
  - whether this user may open the port (dialout group);
  - the board's axis markings;
  - the mount.

- **Dry run, 27 Nov:** every step A command run in order against a stand-in N100 on a pseudo-terminal, with
  Lite's `.venv` and the tool's own serial code: all passed after two fixes to the tool (a reply wait now has an upper
  limit; frames are no longer read as a text reply). `checks/2026-11-27_step_a_dry_run/`.

## Setup

- **Machine:** the lab laptop that runs Lite, in `/home/alai-labs-dell/alix_studio_lite`.
- **Files:** `tools/imu_n100.py` (in Lite since commit 66b837bb), and this runbook with its plan in `docs/imu/`. On the
  lab laptop:

```
cd /home/alai-labs-dell/alix_studio_lite
```
```
git pull
```

- **Lite need not be running.** The tool and Lite never share anything but the logs folder.
- **Keep it still whenever it powers up or restarts.** It measures its gyro bias then.

## Commands

### A. As mounted (27 Nov: the mount is taken as done)

The IMU stays where it is, in its case on the plate between the poles. The robot may be powered (the IMU's USB hub is
on it), but nothing moves and nobody touches the robot during a capture.

1. Which device is the IMU. Expect one line `/dev/ttyUSB0  10c4:ea60 ... <- CP210x: the N100's chip` (on the lab
   laptop `lsusb` lists it as `Silicon Labs CP210x UART Bridge`). Every command finds it by that ID, so `--port` is
   never needed while it is the only CP210x plugged in. If the line says NO PERMISSION, run the command it prints,
   then log out and in.
```
.venv/bin/python tools/imu_n100.py ports
```
2. Is it alive? 20 s of live lines, about 100 a second before step 4. Tap the plate once near the IMU and watch the
   numbers jump.
```
.venv/bin/python tools/imu_n100.py watch --seconds 20
```
3. Read its output rates. This changes nothing; output pauses for a few seconds.
```
.venv/bin/python tools/imu_n100.py config
```
4. Set the IMU packet to 400 Hz and the AHRS to 100 Hz, save, restart and read back. Nobody touches the robot for
   10 s: it works out its gyro bias as it restarts.
```
.venv/bin/python tools/imu_n100.py config --set 40=400 --set 41=100
```
5. Ask for its version (hardware and firmware names) and its sensor ranges.
```
.venv/bin/python tools/imu_n100.py info
```
6. Wait until 15 minutes have passed since it was powered (warm-up). Then 60 s still:
```
.venv/bin/python tools/imu_n100.py capture --seconds 60 --test still --label mounted_still --note "mounted, robot still, 15 min warm"
```
7. The real rate and losses over 5 minutes, still:
```
.venv/bin/python tools/imu_n100.py capture --seconds 300 --label rate_5min --note "mounted, robot still"
```

The turn tests by hand (one per marked axis) need the board loose, so they are left out. The still capture gives the
gravity axis; X and Y are taken from the markings as mounted, and the run analysis fits each axis' sign itself.

### During S2 runs (from 28 Nov)

Nothing to type. With the IMU plugged in and **Record feed** ticked on the S2 page, Lite records the IMU from the
run's start to its end. The run's log says "Recording the IMU (...) with the feed, to <folder>" at the start and "IMU
recording ended" at the end. If the IMU is not detected when Run (or Release) is pressed, the page asks first: "Not
now" (connect it, then press Run again) or "Run without the IMU". A run the UGV starts has nobody to ask: its log says
"The IMU is not recorded: ..." and it goes on. Restart Lite once after pulling this change.
Commit and push `Resources/Logs/<day>/` as usual; the IMU folder is `imu/<HHMMSS>_s2_<mode>/`.

### B. Mount it (robot off): taken as done on 27 Nov

Follow section 2 of PLAN.md:
- screws or a clamp, no tape or foam under it;
- the cable tied to the plate 2-3 cm from the connector;
- the marked X to the robot's front and Y to its left.

Photograph the markings in place. Measure the board's centre from the robot's centre between the four wheels: mm
forward, mm left, mm up.

### C. Mounted

8. 60 s still, the robot off:
```
.venv/bin/python tools/imu_n100.py capture --seconds 60 --test still --label mounted_still --note "mounted, robot off"
```
9. Tap test: still 2 s, then tap the plate 10 cm from the board three times, 2 s apart. Then tap the board itself
   three times, 2 s apart.
```
.venv/bin/python tools/imu_n100.py capture --seconds 30 --test tap --label mounted_tap --note "plate x3, then board x3"
```
10. Noise floor, 2 min in each state (nothing moves).
    - (b) UGV powered, standing:
```
.venv/bin/python tools/imu_n100.py capture --seconds 120 --test still --label noise_b_ugv_on --note "UGV powered, standing"
```
    - (c) arms powered and holding a pose:
```
.venv/bin/python tools/imu_n100.py capture --seconds 120 --test still --label noise_c_arms_holding --note "arms powered, holding, no motion"
```
    - (d) (c) with the UGV drives on:
```
.venv/bin/python tools/imu_n100.py capture --seconds 120 --test still --label noise_d_drives_on --note "arms holding, UGV drives on"
```

## What will happen

- **Each capture** prints its folder, a byte count every 10 s, then its report. The report lines are PASS, CHECK
  (look at it) or FAIL.
- **Steps 3 and 4 (config)** print every command and the unit's reply. Output pauses while the unit is in
  configuration mode, and the tool always ends that mode. Step 4 restarts the unit, about 4 s.
- **Expected and harmless:**
  - "fit both by chance": about 1 header in 256 fits both CRC8 tables;
  - a few hundred "bytes between frames": the heartbeat, or a reply.
- **If the firmware does not know `#fmsg`,** step 3 says "No text reply to #fmsg". Then set the rates in
  FDIGroundStation (on Windows: Config, then MSGOUT; IMU 400 Hz, AHRS 100 Hz; Save to FDI, then Save to Permanent).
  Power-cycle the unit after using the PC tool, then go on from step 5.
- **If `info` gets no reply,** the request packet is not supported. Note the ranges from FDIGroundStation instead.

## What stops it

- **Nothing moves,** so there is no motion gate. Ctrl-C stops `watch`; a capture stops by itself.
- **If a capture says "No bytes from the IMU in 3 s",** a configuration was cut short and the unit is still in
  configuration mode. Run step 3 again (it always leaves that mode).
- **Stop and send what you have** if step 6 or 8 FAILs on gravity, or if any capture FAILs on lost or damaged frames.
  The later steps need those to pass.

## What to send back

- **The logs.** Commit and push `Resources/Logs/2026-11-25/imu/` (or the day you run it) as the lab does its logs.
  Each session folder holds:
  - raw.bin, chunks.jsonl, meta.json, report.txt and report.json;
  - config.txt for steps 3 and 4;
  - info.json for step 5.
- **The photos and the measurements:** the markings in place, the mount, and the three distances from step B.

**What I will read from them:**
- the CRC8 table, the losses and the real rate;
- the arrival spread and the clock drift;
- gravity and the axis signs (which way X, Y, Z point on the robot);
- the turn sign convention;
- the gyro bias and noise in each state;
- the tap settling time and its ringing frequency;
- the sensor ranges and the hardware name (DETA10 or not).

Then I'll set the bars for the arm runs (plan step E) from the noise floor.
