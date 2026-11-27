# IMU (Wheeltec N100) on the UGV: getting accurate, usable data (plan, 2026-11-25, revised the same day)

**Goal (the user's):** find the sections of the arm motions (the Programs, later the lid/seat plans) where the UGV body
jerks or rocks: from the arm links' mass, and from sharp turns. The measurements then:
- set the jerk and UGV-load limits of the automatic optimisation. At their defaults they never bind today: files read
  at most 257 rad/s^3 and 20 N m against 1500 and 52.9;
- check Lite's UGV-load model (`motion/base_load.py`), which works from the commanded motion and the URDF masses only
  and has never been measured;
- give the lab a measured way to compare the hand-tuned files with the automatic sets.

Nothing here commands motion. Every run with the arms is the user's, from a runbook.

## 1. What the N100 is (researched 25 Nov; no complete datasheet exists from the supplier)

**Who makes it.** The N100 runs FDISYSTEMS' (Anhui Feidi Aviation) FDILink protocol and is set up with their PC tool,
FDIGroundStation:
- The N100 articles describe the same tool and the same steps as the vendor's manual for the Epsilon/DETA series.
- They use the same wording: a Sigma-Point Kalman filter, sensor sampling up to 1000 Hz, coning and sculling
  compensation.

So the vendor's documents apply. The exact model inside (most likely the DETA10 family, the smallest module) is my
inference, not stated by Wheeltec.

| Fact | Source | Grade |
|---|---|---|
| Serial over USB (CP2102 chip; no driver needed on Linux); default 921600 baud, 8N1 | juejin N100 article; vendor manual; all three drivers | verified in sources |
| Frame: 0xFC, type, length, serial number (8-bit counter), CRC8, CRC16 high, CRC16 low, payload, 0xFD | sbgisen/fdilink_ahrs `fdilink_data_struct.h`, `ahrs_driver.cpp` (read) | verified in code |
| CRC8 over the first 4 bytes (0xFC, type, length, serial number); CRC16 over the payload, sent high byte first | same; RowSense Python driver agrees | verified in code |
| CRC16: polynomial 0x1021, start 0 (table 0x0000, 0x1021, ...) | sbgisen `crc_table.cpp`; RowSense | verified in code |
| CRC8: Maxim/Dallas table (0, 94, 188, ...) in sbgisen and the magnetometer repo; the RowSense driver uses a different table (0x00, 0x07, ...) | the three sources DISAGREE | settle on the unit (step A5) |
| All numbers little-endian | vendor, FDILink Data Types | vendor document |
| MSG_IMU 0x40, 56 bytes: gyro X, Y, Z rad/s (0-12); accel X, Y, Z m/s^2 WITH gravity (12-24); mag X, Y, Z mG (24-36); IMU temperature C (36); pressure Pa (40, 101325 if no barometer); pressure temperature (44); timestamp int64 us since power-on, from the MCU's external crystal (48) | vendor MSG_IMU page; same in the driver struct | vendor + code agree |
| MSG_IMU is calibrated: turntable (bias, scale, axis non-orthogonality) and temperature chamber (-40 to 85 C) compensation applied | vendor MSG_IMU page | vendor document |
| MSG_RAW_SENSORS 0x58 (48 bytes): the same sensors uncalibrated | vendor | vendor document |
| MSG_AHRS 0x41 (48 bytes): body rates, roll, pitch, heading, quaternion, timestamp | vendor list; driver struct | vendor + code agree |
| MSG_SENSOR_RANGES 0xA1: the unit's own accel range (g), gyro range (deg/s), mag range | vendor | read it from the unit (A6) |
| Body axes marked on the module; right-handed, as north-east-down (Z down); lying flat, accel Z reads about -9.8 m/s^2 | vendor manual and MSG_IMU page | check on the unit (A5) |
| A 1 Hz heartbeat frame of type 0xF0 is always sent | vendor manual FAQ; sbgisen skips type 0xF0 | vendor document |
| After the PC tool has been connected, power-cycle the module before reading data | vendor manual FAQ | vendor document |
| DETA10 ships sending MSG_IMU and MSG_AHRS at 100 Hz; the maximum per packet is 400 Hz | vendor manual; juejin N100 article | the default is a DETA10 figure: read the unit's |
| Text commands: `#fconfig` (configuration mode, output stops), `#fmsg` (list every packet and its rate), `#fmsg 40 400` (IMU packet at 400 Hz), `#fsave`, `#freboot` then `y`, `#fdeconfig`, `#freset` (factory settings); a reply is `#OK` | vendor FDILink command page (current) and 2021 manual (the `#fparam` form) | vendor; whether this unit's firmware knows `#fmsg` is checked in A3 |
| Calibrations: `#fimucal_gyro` (still), `#fimucal_acce` and `#fimucal_level` (level and still); active after a reboot | vendor command page; the PC tool's Gyro / Acc Tare and Level | vendor document |
| At power-on the module works out its gyro bias once (AID_GYO_TURN_ON_TARE_ENABLED, on by default), so it must be still at power-on | vendor manual | vendor document |
| Attitude 0.05 deg RMS static, 0.1 deg RMS dynamic; raw IMU up to 400 Hz, navigation data up to 200 Hz; 5 V | seller listings (youyeetoo / Amazon) | seller's claim |
| Gyro and accel range, noise, bias stability | not published anywhere found | read the ranges from the unit; measure noise and bias (C) |
| Keep it away from big motors (magnetometer) and from heat sources and fast temperature changes (gyro bias) | vendor manual; juejin article | vendor document |

**What this means for us:**
- Log MSG_IMU (0x40) at 400 Hz. It is the calibrated sensor output, so the Kalman filter does not alter it.
- MSG_AHRS can stay on at 100 Hz, as a cross-check on tilt.
- The magnetometer and heading are not used: next to the pole motors they cannot be trusted.

## 2. The mount as it is (photos, 25 Nov)

**What I see:**
- The board sits on the base plate between the two poles, held by masking (paper) tape laid over it, on a patch of
  tape.
- The plate around it is covered in bubble wrap.
- The USB cable comes off one edge of the board and is taped further away.

**The place is good:** it is central and on the plate the poles load. These need changing before any data counts:
1. **Taped down, the board moves on its own.** It can shift and rock on the tape, and the tape between the board and
   the plate acts as a soft spring. It filters out the sharp events we want, and can add a rattle of its own. Fix the
   board rigidly: screws through its holes into the plate (nylon screws and short spacers are fine), or a small rigid
   clamp screwed to the plate. If anything must go under it, use a thin hard glue (epoxy or cyanoacrylate), never foam
   tape or VHB. Any bubble wrap or foam under the board must go.
2. **The cable can pull the board.** Tie the USB cable to the plate 2-3 cm from the connector, so the cable's weight
   and its pull when the poles move never reach the board.
3. **Align it and write it down.** Square the board's marked X and Y to the robot's front and left, and photograph
   the markings once mounted.
4. **Measure where it sits:** millimetres from the robot's centre between the four wheels (base_link). Write it down.

**Check of the new mount:** a tap test. Tap the plate 10 cm from the board, then the board itself. A rigid mount
shows one short spike that dies within a few tens of milliseconds. A loose one rings on, at a frequency of its own.

## 3. Step by step

### A. The unit (on the lab laptop that runs Lite, robot still)

1. **Find its stable name.** Plug it in and look in `/dev/serial/by-id/` for the CP2102's own name. Use that path, or
   a udev rule on that serial number (the user's install step), not `/dev/ttyUSB0`, which can change.
2. **Record 60 s of raw bytes** to a file before anything else: the reference capture.
3. **Read the configuration.** Send `#fconfig`, then `#fmsg`, and save the reply: which packets are on, and at which
   rates. If this firmware does not know `#fmsg`, use FDIGroundStation (Config, then MSGOUT).
4. **Set it.** `#fmsg 40 400`, AHRS at 100 Hz, any other packets off. Then `#fsave` and `#freboot` (answer `y`).
   Read it back with `#fmsg`. Power-cycle after using the PC tool.
5. **Check the bytes:**
   - both CRCs pass with the Maxim CRC8 (or with the other table: whichever matches the unit is the one to use);
   - the serial number goes up by 1 per frame, and gaps are lost frames;
   - the timestamp goes up by 2500 us per IMU frame at 400 Hz;
   - lying flat and still, accel Z is about -9.81 m/s^2 and turns positive when the board is turned over;
   - turning the board 90 deg by hand about each marked axis integrates the gyro to about 90 deg, with the sign as
     the marking says.
6. **Ask for the sensor ranges** (MSG_SENSOR_RANGES 0xA1, by a request packet or from the PC tool) and write them
   down. The gyro range decides whether a sharp stop could clip.
7. **Measure the real rate.** Frames per second over 5 min, lost frames (serial-number and timestamp gaps), and how
   bunched the arrivals at the laptop are.

### B. Mount (robot off): section 2, then the tap test.

### C. Calibration and noise floor (robot still)

- **Power the IMU with everything still**: it works out its gyro bias at power-on. Plug the USB in only when the arms
  and the UGV are at rest, and leave it plugged in for the session.
- **Warm-up:** 15 min. The temperature field shows when it has settled.
- **Our own bias:** 60 s still at the start and at the end of each session. Record the change; the analysis
  subtracts it. Use `#fimucal_gyro` only if the bias is large (save, reboot, and note the date).
- **Do not use Level (`#fimucal_level`).** It rotates the module's axes inside the unit. We rotate to base_link in the
  analysis from the measured mount, so the raw axes stay as marked.
- **Noise floor:** 2 min in each of four states:
  - (a) only the IMU on;
  - (b) UGV powered, standing;
  - (c) arms powered and holding a pose;
  - (d) (c) with the UGV drives on.

  A section counts only above about 5 times the RMS of (c).
- **Note the floor** the UGV stands on.

### D. One clock with the arms

- **Two stamps per frame.** On arrival, the laptop's clock (`time.time()`, as Lite's logs; plus the monotonic clock
  for intervals). And the sensor's own microsecond timestamp: fit it to the laptop clock (offset plus drift) over the
  session. The crystal drifts a little, and USB delivers frames in bunches.
- **The arms with the 20 ms feed on** (the S2 default). Measure the lag: five short single-joint moves with a sharp
  start (in the runbook), lining up the IMU's response with Lite's predicted UGV load from the feed. Accept a spread of
  10 ms or less. The feed lagged 85 ms on 3 Nov.
- **Map IMU sections to Program lines** by matching the feed's joints to the file's lines in order, not by time. The
  controller plays about 5.6 ms a line with the realtime push on (about 4.8 off).

### E. Runs that teach (the user runs them, from a runbook)

1. **Single joints:** two speeds and two accelerations, one pose, one pole height. This shows which joint shakes which
   UGV axis, and by how much.
2. **Sharp stop:** gives the UGV's own rocking frequency and how fast it dies away.
3. **Pole heights:** a few of the above at 50, 150 and 200 mm.
4. **The Programs:** the hand-tuned files on the controllers (left 61-67, right 61-66), then each automatic set
   (`auto_19_nov_2026`: toppra, ruckig_s, custom). Each 3 times, one arm alone first, then in the S2 sequence. The lab
   compares; no set is picked here.
5. **Later:** the lid/seat tasks.

### F. From data to sections (offline, here)

- **Clean the signals.** Gyro with the bias removed and rotated to base_link (deg/s). Acceleration with the resting
  gravity removed. A low-pass to drop motor buzz, with the cut set from C and E2.
- **Find the sections.** For each run, the stretches above the bar, mapped to file lines. For each stretch:
  - the joints with the highest acceleration and jerk there (Lite's `checks.util_series`);
  - Lite's predicted UGV load (`base_load`);
  - whether the file turns sharply there.
- **Check the model.** Measured rocking against predicted load (one gain and one lag?). This decides whether the
  model can stand in for the IMU and set the optimiser's limits.
- **Output:** one table and one graph per Program (each graph with a short description naming its lines).

## 4. When the data counts

| Check | Bar |
|---|---|
| CRC failures | counted and dropped, under 0.1 % |
| Lost frames | under 0.1 % (serial number and timestamp); rate steady within 1 % |
| Configuration | read back (`#fmsg`) and kept with every session's log |
| Mount | tap test passed; mount pose and axes recorded |
| Gyro bias change over a session | recorded, small next to the spikes |
| IMU vs arm feed lag | known to within 10 ms |
| Spikes vs noise floor (c) | at least 5 times its RMS |
| Gyro range | peaks stay under 80 % of the range read in A6 |
| Repeatability | the same Program 3 times: the same sections, peaks within about 15 % |

## 5. Software

- **BUILT 25 Nov (user: "i need programs for connecting and verifying the above"):** `tools/imu_n100.py` in Lite,
  stand-alone and ROS-free. It uses no pyserial: Linux termios at 921600, and numpy for the checks.
  - Subcommands: `ports`, `watch`, `capture --test frames|still|turn|tap`, `check <folder>`, `config [--set ID=HZ]`,
    `info` (version and sensor ranges by request packet).
  - It checks both CRC8 tables and both CRCs, counts lost frames by serial number, and reads the sensor's timestamp
    against the laptop's clock.
  - It writes `Resources/Logs/<day>/imu/<HHMMSS>_<label>/`.
  - Tests: `tests/test_imu_n100.py`, 15, including a capture through a real serial device (a pseudo-terminal).
  - The lab's steps are in `RUNBOOK_step_A.md` next to this file.
- **No separate logger is needed.** S2 already records each arm's 20 ms feed (`Resources/Logs/<day>/canfd/`), stamped
  with the laptop's monotonic clock, the clock `imu_n100.py capture` stamps its reads with. A capture started before an
  S2 run (`capture --seconds 900`, robot still for the first and last 30 s) and the run's logs line up afterwards.
- **BUILT 27 Nov, step F (user: "Run analysis (step F)", tested on made-up data until the lab has data):**
  `tools/imu_runs.py`, offline, run here on the lab's logs.
  - `analyse <capture>`: the sensor clock fitted to the laptop's; bias from the still start and end; rates in base
    axes (`--mount`); the peaks above 5 x the noise floor (`--floor` = a step C capture); for each peak, the arms'
    steps (hardware.jsonl), the Program file and its lines (found by the listed name, confirmed by matching the feed's
    joints to the file's lines in order) and the joints with the most acceleration and jerk there; one gain and one
    lag per axis against Lite's model, alone and with the arms' weight moving (the model leaves that out). Writes
    `<capture>/analysis/`: report.md, sections.csv, summary.json, graphs.
  - `synth`: a made-up capture from a day's feeds with a known gain, lag, drift and bias, to test the analysis only.
  - The model (`motion/base_load.py`) is on the lite_auto_optimization branch; until it is merged, `--model-src` names
    a checkout of it. Without it the peaks, steps and lines are still reported.
  - Pole moves are not modelled (the model holds the pole still).
  - Tests: `tests/test_imu_runs.py` (4, a stand-in for the model). Check on the 24 Nov logs:
    `checks/2026-11-27_synthetic_24nov/` (everything put in came back: lag exact, gains within 1.2 %).
- Nothing inside Lite's run until the data has proved itself.
- The ROS drivers found (sbgisen `fdilink_ahrs`, NDHANA94 `ros2_wheeltec_n100_imu`) are references only: Lite stays
  ROS-free.

## Sources

- **FDISYSTEMS FDILink protocol** (https://doc.fdisystems.cn/developers/02-fdilink/): packet list, MSG_IMU,
  MSG_RAW_SENSORS, MSG_SENSOR_RANGES, MSG_PACKETS_PERIOD, MSG_ANGULAR_ACC pages, Data Types (little-endian), command
  page (`#fconfig`, `#fmsg`, `#fimucal_*`, `#fsave`, `#freboot`, `#freset`).
- **FDIGroundStation manual V21.0423** (https://doc.fdisystems.cn/pdfs/, "FDIsystem上位机使用手册V21.0423.pdf"): DETA10
  defaults 100 Hz, the AID switches (power-on gyro tare), body axes NED and marked, Tare/Level, `#fparam` commands, the
  1 Hz 0xF0 heartbeat, power-cycle after the PC tool.
- **juejin.cn/post/7153044162080669704** (a copy of the CSDN "win平台使用惯导模块：WHEELTEC N100" article; CSDN itself
  refused automated reads): CP2102, default 921600, Config, then Save to FDI, then Save to Permanent; maximum 400 Hz a
  packet; Level / Acc Tare / Gyro Tare; keep away from big motors and heat.
- **sbgisen/fdilink_ahrs** (GitHub; `fdilink_data_struct.h`, `ahrs_driver.cpp`, `crc_table.cpp`, read 25 Nov): frame,
  structs, CRC coverage and tables, serial-number loss count, 0xF0 skipped.
- **NDHANA94/ros2_wheeltec_n100_imu** (GitHub; README and the same struct header): ROS 2 port, 921600, `/dev/ttyACM0`
  default.
- **ViradMunir/RowSense** (GitHub; `wheeltec_n100_imu_py/imu_node.py`): a Python FDILink reader for the N100, using a
  different CRC8 table.
- **oliversipangkar/Magnetometer-Wheeltec-N100** (GitHub): 921600, 0x40 / 0x41 lengths, crc-8-maxim.
- **youyeetoo wiki** (https://wiki.youyeetoo.com/en/WHEETLEC/IMU): SPKF, 1000 Hz sampling, "compensated inertial
  measurements between 10 and 200 Hz".
- **Amazon listings** (youyeetoo WHEELTEC N100): 400 Hz IMU, 200 Hz navigation data, 0.05 / 0.1 deg RMS, 5 V.
- **Seeed wiki** (https://wiki.seeedstudio.com/wheeltec_imu/): the H30 family, not the N100.
- Lite: `motion/base_load.py`, `motion/checks.py`, `server/logbook.py` (`time.time()` stamps); memory: 5.6 / 4.8 ms a
  line, 85 ms feed lag.
