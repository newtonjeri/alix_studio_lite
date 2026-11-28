# The N100 at the lab, 27 Nov 2026: steps A and C (hardware)

The lab's captures: `Resources/Logs/2026-11-27/imu/` (commit dcefae57), on the lab laptop, 15:58 to 16:36 lab time.
The IMU was on the UGV casing (no motors in it) on 3M tape, a temporary mount. Re-read here with `recheck.py`
(output: `recheck_output.txt`); the lab's own report files are left as written.

## The unit

- **DETA-10_V2** (hardware and firmware names, `info`), the FDISYSTEMS module the vendor's documents describe.
- **CRC8: the Maxim table,** on every frame. No damaged frame in any capture.
- **Rates:** IMU 100 Hz and AHRS 100 Hz as delivered; set to IMU 400 Hz and AHRS 100 Hz and saved (`#fsave` OK). The
  restart after it failed (`y` answered `*#ERROR`), but the rates work at once and survived a power-up: the unit's
  "time since power-on" restarts at about 16:02, after the 15:59 setting, and every capture since runs at 400.0 and
  100.0 frames a second.
- **Sensor ranges:** not known. The reply to the ranges request is 4 bytes (`01 00 00 00`), not the documented 6.
  Taps on the board itself reached 417 deg/s and 6.3 g as single samples (no flat top), so the ranges are at least
  that.

## The data

| | Result |
|---|---|
| Frames lost | 0 in every capture (the IMU's own time stamps: 24,000 a minute, 0 missing) |
| Frames out of order | about 30 a minute: the counter goes back by one, then on by two |
| Laptop arrival after the earliest | 0.2-0.6 ms median, 1.8 ms worst over 5 min |
| Sensor clock against the laptop's | -4 to -103 ppm by capture; within one capture a straight line holds to about 1 ms |
| Gravity | 9.84 m/s^2 on -Z: lying flat, tilted 2.3-2.8 deg (roll) with the casing |
| Gyro bias over 33 min | within 0.06 deg/s on every axis |
| Noise, arms powered and holding | 0.044-0.053 deg/s RMS per axis (all bands); rocking rate 0.030 deg/s RMS at 30 Hz, 0.018 at 8 Hz |
| Bar for the arm runs | 5 x 0.030 = 0.15 deg/s (at the analysis' 30 Hz) |
| Taps on the casing | most of the ringing at 171 Hz; 84-95 % of the motion after a tap above 30 Hz |
| Taps on the IMU itself | settle over 0.5-1.3 s, mostly below 30 Hz: the IMU moving on the tape |

## What the first reports got wrong (the tools, fixed 28 Nov)

1. **"FAIL frame counter: 17-23 % lost"** in every report: each out-of-order frame (counter back by one) was counted
   as 254 lost. Every IMU frame was there. `counter_gaps` now unwraps the counter; out-of-order frames are counted
   apart.
2. **The 5-min capture's "23,618 missing", a 359 s span and a 59 s worst arrival:** its first IMU frame was 59 s old,
   left in the USB chip from the previous capture. `drop_stale_head` now leaves such frames out (both tools).

Re-read with the fixes, all ten captures pass; the two tap tests keep their CHECK (the tape).

## Not known yet

- The lag between the IMU and the arms' feed, and whether the model explains the rocking: no IMU capture ran during
  the 27 Nov S2 run (20:54).
- The sensor ranges (above).
- The IMU's place and axes on the robot, measured.
