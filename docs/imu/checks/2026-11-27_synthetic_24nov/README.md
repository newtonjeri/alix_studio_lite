# Check of tools/imu_runs.py on a made-up capture (27 Nov 2026)

**This measures nothing about the robot.** It checks that the analysis gives back what was put into a made-up IMU
signal, on the lab's real logs of 24 Nov 2026 (the S2 run that started at 21:03:28, both arms, 4 feed files).

## How it was made

The made-up gyro is the moment the UGV would feel from both arms (Lite's load model plus the arms' weight moving),
computed from the arms' recorded feeds, times a known gain, delayed by a known lag, plus noise. It has a gyro bias that
drifts from start to end, a sensor clock running 30 ppm fast, 0.05 % lost frames, and USB-style bunched reads.

```
python3 tools/imu_runs.py synth --logs Resources/Logs/2026-11-24 --match 20261124_2103 --out <folder> --model-src <lite_auto_optimization checkout>
python3 tools/imu_runs.py analyse <folder> --logs Resources/Logs/2026-11-24 --model-src <lite_auto_optimization checkout>
```

The model came from the lite_auto_optimization branch (commit dc37642d), read without checking it out
(`git archive`), with this checkout's `vendor/` beside it. The made-up capture (7 MB) is not kept: the commands above
remake it exactly (seed 1).

## What came back

| | Put in | Found |
|---|---|---|
| Sensor clock against the laptop's | +30 ppm | +30.1 ppm |
| Lost frames | 0.05 % (about 49) | 47 |
| Gyro bias, roll axis, start and end (deg/s) | +0.20, +0.24 | +0.203, +0.238 |
| Lag | 60 ms | 60 ms (both axes) |
| Gain, roll and pitch (deg per N m) | 0.02, -0.03 | 0.0198, -0.0297 |
| Share explained, model + weight (R^2) | | 0.998, 0.998 |
| Share explained, model alone | | 0.944, 0.958 (lower, as it should be: the signal held the weight term) |
| Program runs matched to their files | 10 | 10 (worst 0.44 deg between feed and file) |

At each peak, the measured rate at the model's 8 Hz band and the fitted prediction agree within about 2 %
(`report.md`, peaks table).

## Kept here

- `report.md`, `summary.json`: the analysis' own output (its paths point at the scratch folder it ran in; only two of
  its eleven graphs are kept).
- `overview.png`: the whole capture.
- `run_05_right_63_R_P2_pole200_cleaningpose_9_2.png`: one Program's graph.

## What it does not show

That the robot behaves like this. The real gain, lag, rocking frequency, and whether the model or the weight term
explains the rocking come only from a real capture (step A first, then the mount, then a capture across an S2 run).
