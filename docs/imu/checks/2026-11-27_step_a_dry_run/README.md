# Step A dry run against a stand-in N100 (27 Nov 2026)

**No IMU was used.** `standin_n100.py` plays the unit on a Linux pseudo-terminal, as the vendor documents it
(PLAN.md section 1). Every step A command of `RUNBOOK_step_A.md` was run against it in order, with Lite's `.venv`
Python and the tool's own serial code (termios, raw, 921600): `dry_run_output.txt`. The captures were shortened
(15 s and 30 s for the runbook's 60 s and 5 min) and written to a scratch folder, not Resources/Logs.

```
python3 docs/imu/checks/2026-11-27_step_a_dry_run/standin_n100.py --seconds 200     # prints its port
.venv/bin/python tools/imu_n100.py watch --seconds 3 --port <that port>               # and so on, with --out <scratch>
```

## Results (stand-in, not hardware)

| Step | Result |
|---|---|
| ports | runs; finds nothing here (no USB serial device on this laptop), as expected |
| watch | live lines at 100/s, accel Z -9.79, CRC8 table maxim |
| config (read) | #fconfig, #fmsg (IMU 100 Hz, AHRS 100 Hz), #fdeconfig |
| config --set 40=400 --set 41=100 | set, saved, restarted, read back: IMU 400 Hz, AHRS 100 Hz |
| info | version and sensor ranges decoded (16 g, 2000 deg/s, 8 gauss: the stand-in's values) |
| capture still | all PASS: 400.0 IMU frames/s, 0 lost, gravity on -Z |
| capture frames | all PASS: 400.0/s, 0 lost, timestamps 2500 us |

## What the dry run found, and fixed in tools/imu_n100.py

1. **A reply wait had no upper limit.** It grew with every read, so a unit that keeps streaming frames after
   `#fconfig` (a command its firmware does not take) would have held `config` until Ctrl-C. Now a reply is read at
   most 3 s beyond its wait (`REPLY_CAP_S`); test `test_a_unit_that_keeps_streaming_does_not_hold_the_tool`.
2. **Bytes inside frames counted as a text reply,** so "No text reply to #fmsg" would never have shown with a
   streaming unit. Whole frames are now left out of a reply's text, and only a word counts as text.

(The first stand-in echoed its own output, a pseudo-terminal default; it is now set raw, as a USB serial chip is.)

## Not checked here

The real unit's CRC8 table, its answers to `#fmsg` and to the request packet, its real rates and losses, the
dialout permission on the lab laptop, and the axes as mounted. That is what step A on the lab laptop is for.
