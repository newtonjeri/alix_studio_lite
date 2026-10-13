# Research — two optimisers with an operator target time (decision F2)

Date: 13 Oct 2026. Scope: F2 (two optimisers, the operator's time is a TARGET), applied with F1 (operator PC now, Jetson aarch64 in production), F3 (commode C), F5 (limits come from URDF and scene), F6 (any program ID 1–100) and F7 (code in `alix_studio_lite/`, ALIX imported from a pinned checkout or bundle).

Path aliases used below:

| alias | path |
|---|---|
| `PMT` | `/home/newtonjeri/alix_ws/src/alix/tools/plan_min_time.py` |
| `TT` | `/home/newtonjeri/alix_ws/src/alix/tools/taught_tracks.py` |
| `TP` | `/home/newtonjeri/alix_ws/src/alix/tools/taught_pipeline.py` |
| `TG` | `/home/newtonjeri/alix_ws/src/alix/tools/tcp_gate.py` |
| `CT` | `/home/newtonjeri/alix_ws/src/alix/tools/cleaning_tracks.py` |
| `DAV` | `/home/newtonjeri/alix_ws/src/alix/tools/dual_arm_verdicts.py` |
| `ETR` | `/home/newtonjeri/alix_ws/src/alix/experiments/exp01_teach_trajectory_reverse_engineering/src/emit_toppra_rung.py` |
| `LS` | `/home/newtonjeri/alix_ws/src/alix/experiments/exp01_teach_trajectory_reverse_engineering/src/local_speed.py` |
| `KIN` | `/home/newtonjeri/alix_ws/src/alix/experiments/exp01_teach_trajectory_reverse_engineering/src/teachlab/kinematics.py` |
| `TRK` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_dispatch/src/alix_dispatch/track.py` |
| `JL` | `/home/newtonjeri/alix_ws/src/alix/ros/manipulation/alix_moveit_config/config/joint_limits.yaml` |
| `PIPE` | `/home/newtonjeri/alix_ws/src/alix/docs/motion/TAUGHT_CANFD_PIPELINE.md` |
| `README` | `/home/newtonjeri/alix_ws/src/alix/docs/motion/TAUGHT_PIPELINE_README.md` |
| `REQ` | `/home/newtonjeri/alix_ws/src/alix/tools/requirements_taught_canfd.txt` |
| `H` | `/home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html` (the prototype) |
| `CC` | `/home/newtonjeri/alix_ws/src/alix/experiments/exp01_teach_trajectory_reverse_engineering/data/teach_files/commode_c_taught_paths/` |
| `C2` | `.../scratchpad/reqs/C2_trajectory_optimization_final.md` (earlier verified analysis) |

Measurements marked **[measured 13 Oct]** were run for this note with inline Python. They used ALIX's own functions under `/home/newtonjeri/alix_ws/src/alix/.venv-taught` (numpy 2.5.3, scipy 1.18.1, toppra 0.6.10) on the `CC` recordings, on this x86-64 PC. No file was written. The method is in Appendix A.

---

## 0. Summary

1. **ALIX already has a hardware-proven TOPP-RA chain for taught joint paths.** Lite should call it rather than write a new one. The chain is:
   - `TT.smooth_taught` (5 Hz, zero-phase);
   - `TT.strip_pauses` (R4);
   - `LS.TaughtPath` and `LS.arclen_smooth` (30 mm);
   - `LS.retime_toppra` (0.95 × ratings, 600 °/s², `Interpolation`, `seidel`);
   - optionally `ETR.post_lowpass` (6 Hz);
   - sampling every 5.0 ms, then integer millidegrees.

   `PIPE:17-40` lists it and `ETR:306-330` runs it. It gives commode C lid top 62.1 s → 19.2 s (3.24×, slot 101) (`PIPE:588`). Re-run here, it reproduces 3839 lines / 19.19 s exactly.
2. **The default optimiser is the prototype's trim → moving mean → per-region linear resample** (`H:1438-1616`, `H:1676-1724`). It has no limit checks at all (`H:1676-1810`). It has three measured defects:
   - endpoint snaps of 307–1005 millidegrees in one tick;
   - acceleration spikes of 640–919 °/s² at sub-region boundaries;
   - with the default 101-point window, the pad moves 9–21 mm from the taught path.

   Three minimal fixes keep it "the default algorithm":
   - pad the edges by odd reflection;
   - blend the speed ratio across each boundary over 0.25 s;
   - use a smaller default window (41) with a G4 path-change check.
3. **Target-time rule, the same for both:**
   - Compute the shortest duration that passes the limit set (T_min).
   - If T_target ≥ T_min, deliver T_target to the tick by **uniform time scaling** (velocity /k, acceleration /k²).
   - If T_target < T_min, deliver T_min and name the binding limit (joint, speed or acceleration, time window).

   For TOPP-RA, uniform scaling matches a re-solve with limits scaled (1/k, 1/k²) to within 0.0006°, and it needs no re-solve [measured 13 Oct]. For the default optimiser, T_min comes from a bisection on k against the limit checks, run on the bytes.
4. **Limit set (hard, shared by both, judged on the integer bytes at the 5.0 ms gate clock):**
   - position: G1, the ALIX model's predicted executed margin of at least 0.5° at every limit;
   - speed: band-limited speed within 0.97 × rating (180/180/225×5 °/s);
   - acceleration: within 600 °/s², read as a 50 ms windowed mean;
   - step: at most 10° per line;
   - path change from the recording (G4): at most 10 mm, or the region's MAX DEV if it is smaller.

   G5 (predicted executed departure) and jerk are checks only. G5 becomes a hard limit only when a region carries a MAX DEV.
5. **Durations the operator sees are wall-clock**, from the upload player law: 0.2322 s + 5.6737 ms × (N−1), with a 5.35–6.10 ms/line band (`TRK:1558-1563`). Limits are judged at 5.0 ms/line (`TT:94-102`). The target is converted to a line count N with that law.
6. **Measured gap between the two optimisers** [measured 13 Oct], fastest feasible design time at 5 ms, with comparable path change (about 3–4.5 mm):

   | file | TOPP-RA | default (41-pt window) |
   |---|---|---|
   | FILE_10.1 | 4.26 s | 8.14 s |
   | FILE_08 | 16.80 s | 27.42 s |
   | FILE_01.1 | 19.19 s | 38.30 s |

   TOPP-RA is 1.6–2.0× faster because it slows only where the curvature or the limits require it.
7. **Jetson risk:** PyPI ships **no aarch64 wheel and no source package for toppra 0.6.10.** The 0.6.10 files are x86_64 only, and the newest source release is 0.6.3 [checked 13 Oct via pypi.org JSON]. The aarch64 bundle must build toppra from its git tag (Cython plus a C compiler). Lite must still work without it: TOPP-RA is disabled with a message and the default optimiser remains.

---

## 1. ALIX's TOPP-RA for taught joint tracks (investigation item 1)

### 1.1 Which function does what

| step | function | location | what it does |
|---|---|---|---|
| read | `read_q_deg(path)` | `TT:284-288` | Reads the track through the upload parser (`TRK:66`) and returns degrees. |
| temporal denoise | `smooth_taught(q_deg, hz, clock_s=0.005)` | `TT:891-912` | Order-2 Butterworth, `filtfilt` (zero phase), `padlen` = min(450, n−1), designed at the 5 ms record clock (`TT:109-115`). |
| pause removal (R4) | `strip_pauses(q_deg, tol=0.05°, min_lines=3)` | `TT:915-951`, constants `TT:237-241` | A run of 3 or more lines, all within 0.05° of the run's first line, collapses to that first line. The end point is always kept. |
| path q(s) | `TaughtPath(q_deg, side, frame)` | `LS:78-104` | s = pad arc length in mm (FK at `L_glove_frame_2`). `CubicSpline` through every kept knot; repeated s values are dropped (`LS:88-92`). |
| geometric smoothing | `arclen_smooth(path, lam_mm, ds=0.5)` | `LS:139-169` | Resamples q(s) on a 0.5 mm grid, applies a 4th-order zero-phase Butterworth with cutoff 1/λ per mm, then maps back onto the source knots with a cubic (`LS:161-169`). This is "the main path change" (`PIPE:111-138`). |
| **time-optimal parametrisation** | `retime_toppra(path, v_star, dt=0.005, vcap=None)` | `LS:282-324` | x = s/total; `toppra.SplineInterpolator(x, q)`; `JointVelocityConstraintVarying` = min(rating × scale, demand, `vcap`); `JointAccelerationConstraint(600°/s²)` with `DiscretizationType.Interpolation` (`LS:313-319`: the default collocation overshot to 1007.7 °/s², 1.68×); `solver_wrapper="seidel"`; `compute_trajectory(0, 0)` (zero velocity at both ends); sampled as `traj(np.arange(0, duration, dt))` (`LS:324`). |
| demand | `demand("max", path)` | `LS:108-136` | "max" = no speed wish, only the limits (`LS:122-126`). |
| arrival hold (G1 helper) | `arrival_hold(q, rate_deg_s, headroom)` | `ETR:149-197` | Per-knot speed caps on the approach to turns near a joint limit, so G1 holds. Used at 0.8 for the `+acc` candidates (`TP:130-137`). |
| departure cap (optional) | `toppra_under_departure(q_s, path, budget_mm, dt)` | `ETR:101-146` | Turns the execution model's per-line λ cap into a speed bound. Used once (slot 52) (`PIPE:163-167`). |
| shake fix | `post_lowpass(Q, hz=6, dt)` | `ETR:200-215` | Order-4 `sosfiltfilt` on q(t), ends padded with a 1 s hold. |
| driver | `main()` | `ETR:226-478` | Chain at `ETR:306-330`; per-joint CANFD preflight at `ETR:351-384`; writes with `tt.write_track` (`ETR:430`); refuses to overwrite different bytes (`ETR:409-429`). |
| plan-JSON variant (not taught) | `LsqPath`, `solve`, `resample_clock` | `PMT:81-135`, `PMT:237-252`, `PMT:375-394` | A least-squares B-spline through planned waypoints; TOPP-RA with a fixed grid (3000 points); the point count rounded **up** so the result is never faster than optimal (`PMT:387-394`). |
| batch and ladder | `ladder`, `_ladder`, `acc_free_pick`, `evaluate`, `certify_gates` | `TP:290-422`, `TP:340-372` | The MAX DEV ladder: faster rungs with rounder corners (smoothing 45–120 mm); slower rungs with velocity and acceleration × the same factor `LADDER` (0.95…0.1) at 15 mm smoothing (`TP:107-141`). |
| file writer | `write_track(path, md)` | `TT:737-745` | `{"point":[…7 ints…]}` per line, LF on disk. The `.proj` builder converts to CRLF. |

Not relevant to taught tracks:
- `exp02_toppra_cartesian_pipeline`, `tools/exp02_offline_gate.py`: Cartesian planned strokes; ruckig_on_s is the candidate and TOPP-RA only the reference (`exp02 README:1-30`).
- `tools/track_emit_parity.py`: C++/Python byte parity for plan emits (`track_emit_parity.py:1-25`).
- `DAV`: inter-arm clearance verdicts computed from the emitted bytes. It matters for TOGETHER blocks (§6) but does no time parametrisation.

### 1.2 Path representation

- ALIX does not fit a spline through raw points. It runs 5 Hz zero-phase filtering, R4, a cubic spline in pad arc length, 30 mm arc-length low-pass, then a new cubic spline (`ETR:306-320`).
- TOPP-RA's grid is auto-proposed by toppra; no `gridpoints` argument is passed (`LS:320`; `PIPE:150`).
- The planned-path variant fixes 3000 gridpoints (`PMT:420`).
- Smoothing defaults:
  - `--smooth-mm` defaults to 10 mm (`ETR:234`), but every run slot used 30 mm;
  - rows with a TCP budget use 15 mm (`TP:141`, `PIPE:136-138`).
- Arc-length rather than temporal filtering is deliberate. Curvature binds before the speed ratings on 99.9 % of the path (`ETR:22-26`, `LS:143-149`).

### 1.3 Constraints

| quantity | value | source |
|---|---|---|
| joint speed ratings | 180, 180, 225, 225, 225, 225, 225 °/s | `TRK:3728`; `KIN:32-37` (rm_75_macro.xacro); `JL:24-31`, `JL:222-270` (π and 225π/180 rad/s) |
| TOPP-RA speed bound | rating × `--rate-scale`: 0.95 on the un-budgeted run slots (53–59, 101–104, 106, 108, 111); 0.5 / 0.25 / 0.2 on the TCP-budget slots | `PIPE:146-147`, `ETR:265-269` |
| design allowance for the 20 Hz filtered reading | 0.97 (`RATING_FILTER_ALLOWANCE`) | `TT:227-229` |
| acceleration | 600 °/s² = 10.472 rad/s² (the vendor's `libalgo_get_joint_acc_maxlim` / `joint_drive_max_acc`) | `KIN:39-41`; `JL:226`; `TP:138-139`; `LS:71` |
| acceleration evidence | 1.5× on every joint (slot 54) gave departure p95 2.0 → 23.4 mm, max 64 mm; 1.5× on all but J6 (slot 55) gave p95 7.6 mm. 600 is a real following limit. | `PIPE:157-161`; `ETR:244-248` |
| jerk | **Not bounded.** TOPP-RA is bang-bang, so sampled jerk ≈ 2·a_max/dt. The 1500 rad/s³ ceiling in `JL` is only reported. | `PMT:22-29`, `PMT:503-505`; `JL:3-22`; `PIPE:148` |
| position | ±178, ±130, ±178, ±135, ±178, ±128, ±360° | `KIN:43-46` |
| step | ≤ 10° per line (error 0x4000 "Position Command Step Jump") | `TT:127-128`; `TRK:3707` |

### 1.4 Resampling to the controller tick and the written file

- **Emit and gate clock: 5.0 ms.** `CLOCK_GATE_S = 0.005` (`TT:94-102`), `PMT CLOCK = 0.005` (`PMT:47-52`), `CT CLOCK_S = 0.005` (`CT:242`), `CANFD_DEFAULT_PERIOD_S = 0.005` (`TRK:3708-3719`).
  - The upload player advances at most one line per 5 ms controller cycle and holds otherwise. Its whole-run average is ~5.55 ms (`TT:94-108`).
  - The execution model's predictions run at 5.4 ms on purpose (`TT:103-106`).
- **Wall-clock law for the upload player:** `expected_replay_s = 0.2322 + 5.6737e-3 × (N−1)`, de-ramped rate envelope 5.35–6.10 ms/line (`TRK:1558-1570`).
  - The older band in `TT:116-121` (0.057 + 5.515 ms × (N−1) … 0.750 + 5.810 ms × (N−1)) is used by `duration_band_s` (`TT:278-281`).
- **Sampling.**
  - `retime_toppra` uses `arange(0, T, dt)`. The last sub-tick remainder is dropped, so the end point is reached to within ≤ one tick of travel; FILE_08 measured 0.0052° [measured 13 Oct].
  - `PMT.resample_clock` rounds the point count up and samples t_i = i·T/n. It reaches s = 1 exactly and is "marginally slower than optimal, never faster" (`PMT:375-394`). **Lite should use the PMT form.**
- **Write:** `np.round(Q*1000).astype(int64)` → `tt.write_track` (`ETR:351`, `ETR:430`; `TT:737-745`).
  - Gates are judged on the written integers, not the floats (`PMT:554-557`).
  - Raw finite differences mean nothing at this clock: one LSB of jitter is 2.26 rad/s² (`PMT:268-276`), so every rate and acceleration figure is read through a 20 Hz band limit (`TT:427-431`).
- **Step:** the per-joint preflight `canfd_preflight(rows, period_s, "per_joint")` checks rate and the 10° step on every line (`TRK:3792-3864`, used at `ETR:352-353`).

### 1.5 Measured speed-ups and path change

From ALIX (hardware, CANFD):

| recording | raw nominal | after TOPP-RA | speed-up | executed pad vs taught (p95 / max) |
|---|---|---|---|---|
| v1 bowl-out | 127.1 s | 32.8 s (slot 56) | 3.88× (~2.9× after R4) | 2.88–2.97 / 6.5–6.9 mm |
| v3 bowl-out | 91.2 s | 22.8 s (slot 59) | 3.99× (~3.0× after R4) | 2.03–2.08 / 4.5 mm |
| commode C lid top FILE_01.1 | 62.1 s | 19.2 s (slot 101) | 3.24× | file vs recording max 4.05 mm |
| commode C bowl outside FILE_02 | 65.8 s | 19.8 s (102) | 3.32× | 3.73 mm |
| commode C seat top FILE_03.1 | 60.0 s | 17.8 s (103) | 3.37× | 4.02 mm |
| commode C rim top FILE_04 | 48.2 s | 13.8 s (104) | 3.49× | 4.74 mm |

Sources: `PIPE:42-55`, `PIPE:588-591`. G4/G5 facts behind the table:
- File vs recording for the 30 mm chain: p95 2.2–2.8 / max 3.2–4.7 mm. The 6 Hz post-LP adds up to 6.06 mm (one cusp, v1) (`PIPE:329-335`, `PIPE:182-183`).
- **G4 reads 0.0 mm on every TOPP-RA slot.** Certify compares the derived file with itself; this is known gap 1 (`PIPE:320-324`, `PIPE:547-550`). **Lite must measure G4 against the raw recording** (the `origin` sha is in the sidecar, `ETR:438`).
- G5 (model predicted departure) is upload-fitted and does not rank CANFD files. Slot 52 predicted 28.1 mm and measured 4–5 mm (`PIPE:345-356`, `PIPE:551-553`).

**Re-measured 13 Oct** (ALIX chain, 0.95 × ratings, 600 °/s², 30 mm, no post-LP, x86-64 PC):

| file | raw | after R4 removal | T_min at 5 ms | lines | wall (player law) | TOPP-RA solve time |
|---|---|---|---|---|---|---|
| FILE_10.1 (lid seat open, MAX DEV 5 mm) | 3293 lines / 16.5 s | — | 4.26 s | — | — | 0.11 s |
| FILE_08 (bowl inner back, 10 mm) | 8344 / 41.7 s | 976 removed | 16.80 s | 3360 | 19.29 s | 1.69 s |
| FILE_01.1 (lid top) | 12429 / 62.2 s | 1918 removed | **19.19 s** | **3839** (= slot 101) | 22.01 s | 1.82 s |
| FILE_6 (bowl inside) | — | — | 20.25 s | — | — | — |

Further measurements on the same runs:
- Binding at T_min on FILE_10.1 (share of time at ≥ 98 % of a bound): J6 acceleration 38 % (longest 2.88–3.05 s), J1 acceleration 15 % (1.48–1.69 s), J3 acceleration 9 %; 17 % of the time at no bound.
- Acceleration binds almost everywhere; speed utilisation peaks 0.85–0.95 of the rating. This matches `ETR:22-24`.
- **The bytes keep the acceleration bound:**
  - the 50 ms windowed-mean acceleration reads 1.007 × 600 on all four files;
  - the 20 Hz Butterworth reading is 1.18–1.22 × (the filter's step overshoot, not the robot);
  - with the 6 Hz post-LP the windowed reading is 1.17–1.25 ×, which is real. This matches `PIPE:184-191`, where ALIX's emit does not refuse it.
- Band-limited speed 0.85–0.95 of the rating; largest step 0.97–1.05° per line.

### 1.6 Importability and dependencies

- **No ROS needed.** `REQ:1-30` lists numpy ≥ 1.26, scipy ≥ 1.11, toppra ≥ 0.6, PyYAML, matplotlib, tornado. Tested with numpy 2.5.3 / scipy 1.18.1 / toppra 0.6.10 (`REQ:13-18`). The venv is built by `tools/setup_taught_canfd.sh` (`README:3-4`).
- **Import cost:**
  - `import local_speed, emit_toppra_rung` takes 0.35 s, mostly scipy.interpolate [measured 13 Oct];
  - transitive ALIX modules: `alix_dispatch` and `cleaning_tracks` (via `taught_tracks`), plus `alix_workspace.urdf_kinematics` (FK in `cut_dossier.fk_tool`, `tools/cut_dossier.py:117-134`) and `teachlab` (exp01) for `deviation_mm` (`TT:649-652`).
  - All of them insert their own `sys.path` entries at import (`LS:51-56`, `TT:78-92`, `ETR:50-54`).
- **Caveats when importing:**
  1. `LS.RATE_DEG_S` and `LS.ACC_DEG_S2` are module globals that the emitter mutates (`ETR:304-305`). They are not safe to change from two threads at once. Lite needs either a lock or a Lite-owned 40-line `retime_toppra(path, vlim, alim, vcap)` with explicit arguments, held to byte parity with ALIX by a golden test.
  2. The emitter is **left-arm only** (`ETR:71`; `PIPE:556-557`, known gap 3). `TaughtPath(q, side, frame)` and `retime_toppra` take the side as an argument (`LS:81`), so Lite can call them per side. Lite should not shell out to `ETR main()`.
  3. Bytes differ by ≤ 1 millidegree across numpy, scipy and toppra versions (`ETR:417-427`). Golden tests must pin versions or allow ±1 millidegree. The same applies across x86-64 and aarch64.
  4. The system `python3` here has toppra **0.6.9** (scipy 1.17.1). Only `.venv-taught` has 0.6.10. Lite must ship its own environment.
- **aarch64 (Jetson, F1):**
  - toppra 0.6.10 on PyPI has cp39–cp314 **x86_64 manylinux and musllinux wheels only**, and no sdist. The last sdist is 0.6.3 [checked 13 Oct: `pypi.org/pypi/toppra/0.6.10/json`].
  - Installed, toppra is a compiled Cython package (`_CythonUtils.cpython-312-x86_64-linux-gnu.so`, Cython seidel solver).
  - The aarch64 bundle must build toppra 0.6.10 from the GitHub tag (needs Cython, numpy headers and gcc) in an arm64 build job or on the Jetson. numpy and scipy do have aarch64 wheels.
  - Lite must start with TOPP-RA disabled ("TOPP-RA unavailable on this computer — using the default optimiser") if `import toppra` fails.

---

## 2. The DEFAULT optimiser = the prototype pipeline (investigation item 2)

### 2.1 What it does

Parameters, from `H:729-737` with defaults at `H:367-383`:
- `sampleMs` 5;
- `idleWindow` 11;
- `idleThreshold` 8 millidegrees/line;
- `idleMinRun` 20;
- `smoothWindow` 101;
- all three stages on (`H:391-399`).

Stages (`processTrajectory`, `H:1676-1724`):
1. **Trim** (`trimIdle`, `H:1438-1492`). A per-joint centred rolling mean of |Δq| over 11 lines. A line counts as idle when every joint is ≤ 8. Interior rows of idle runs of 20 or more are removed; the run's endpoints are kept.
2. **Mean smooth** (`meanSmooth`, `H:1494-1517`):
   - a centred 101-point boxcar with edge-replicate padding (`centeredRollingMean`, `H:1417-1436`);
   - plus a constant mean correction (`H:1510`);
   - rounded to integers;
   - then first and last rows forced back to the input (`H:1513-1514`).
3. **Scale** (`applyRegionScales` / `interpolateRegion`, `H:1568-1616`):
   - each operator region is linearly resampled to round((len−1) × pct/100) + 1 lines (`H:1573`), endpoints pinned (`H:1587-1588`);
   - suggestions come from `analyzeMotionRegions` (`H:944-1050`): 5–10 equal slices (`H:962`), first and last at 75 %, high-dynamic slices at 115 %, and so on.
4. The global first and last rows are reset to the original (`H:1721-1723`).
5. Duration is shown as lines × `sampleMs` (`H:1726-1729`). There is no player law and **no limit check anywhere** (`H:1676-1810`).

### 2.2 Measured defects

| defect | evidence | cause |
|---|---|---|
| **Endpoint snap** | First or last step of 966 millidegrees (FILE_08, J3), 1005 (FILE_10.1, J6), 546 (FILE_01.1, J2) — larger than any interior step (`C2` facts 2 and App. V). With the snap present, the fastest the file may be played is **6.7–13.0× slower than the recording** (FILE_08 473 s instead of 22 s) [measured 13 Oct, bisection below]. | Edge-replicate padding biases the first and last 50 smoothed rows toward the end values; `H:1513-1514` then forces the exact end, a one-tick jump. |
| **Boundary acceleration spikes** | Applying the default suggestions [75, 80, 95, 115, 75] to FILE_01.1 gives one-tick jumps of 640 / 440 / 880 / 840 °/s² against a median of 80 (`C2` fact 3). Re-measured on the 5 Hz-smoothed file: piecewise 240 / 320 / 640 / 919 °/s² (median 80, p99 466) [measured 13 Oct]. | Each region is played at a constant speed, so the speed steps at the boundary. |
| **Path bending by the boxcar** | Pad deviation at `L_glove_frame_2`. Smooth only, FILE_08: new → raw 9.06 mm, raw → new **20.93 mm**. Trim + smooth, max of both directions: FILE_08 20.93, FILE_10.1 10.60, FILE_01.1 14.69 mm. Trim alone moves FILE_08 only 0.12 / 0.40 mm [measured 13 Oct]. The `C2` figures 9.05 and 10.39 mm are the new → raw direction. ALIX's G4 cap is 10 mm (5 preferred) (`TT:129-132`). | A 101-line boxcar (505 ms) averages across stroke reversals and cuts the tips. |

### 2.3 Minimal fixes that keep it "the default algorithm"

The stage order, the controls and the meaning of "%" all stay. Each fix replaces one internal detail.

| fix | change | measured effect [13 Oct] |
|---|---|---|
| F-a: edges | Pad the moving mean by **odd reflection** (2·x₀ − x[k]) instead of replicating. Drop the constant mean correction (`H:1510`) and the forced end rows (`H:1513-1514`, `H:1721-1723`). Odd reflection keeps the end value and the end slope, so no snap is needed. | End steps fall from 966 / 1005 / 546 to 99 / 93 / 56 millidegrees with the 101 window, and 18–58 with 21–41. |
| F-b: boundaries | Make the per-line time step piecewise constant per sub-region, then blend it with two passes of a 0.25 s moving average. Integrate the result (the ALIX `retime_lambda` form, `LS:198-219`; `TT.MODEL_RAMP` precedent `TT:224-226`) and resample linearly on the chords. Points stay on the polyline. | FILE_01.1 boundary peaks fall from 240 / 320 / 640 / 919 to 239 / 280 / 200 / 280 °/s². Duration is unchanged (54.69 s). |
| F-c: path bending | Default window **41** lines (205 ms) instead of 101, plus a hard G4 check (≤ 10 mm, or the region's MAX DEV) after smoothing. If G4 fails, offer "reduce smoothing". A "5 Hz zero-phase" option (`TT.smooth_taught`) gives the same as a 21-window. | Max pad deviation, three files: 101 → 10.6–20.9 mm; 41 → 2.9–4.5 mm; 21 → 1.1–2.1 mm; 5 Hz → 1.0–2.1 mm. |
| F-d: limits | Add the shared limit set (§4) and the target-time search (§3) on the output. | See §3.3. |

### 2.4 Fastest feasible time for the default algorithm

Uniform scale over the whole region, judged on integer bytes at 5 ms. Bars: 20 Hz band-limited speed ≤ 0.97 × rating, 20 Hz band-limited acceleration ≤ 600 °/s², step ≤ 10° [measured 13 Oct]. The table gives the design duration at 5 ms/line:

| file | trimmed | 101 + snap (prototype as is) | 101 odd-reflect | 41 odd-reflect | 21 odd-reflect | 5 Hz | TOPP-RA (§1.5) |
|---|---|---|---|---|---|---|---|
| FILE_08 | 38.79 s | 473 s (k 12.2) | 22.34 s (20.9 mm!) | 27.42 s (4.3 mm) | 30.67 s (2.1 mm) | 30.08 s (2.1 mm) | **16.80 s** |
| FILE_10.1 | 14.34 s | 186 s | 6.98 s (10.6 mm!) | 8.14 s (2.9 mm) | 10.07 s (1.1 mm) | 9.79 s (1.0 mm) | **4.26 s** |
| FILE_01.1 | 54.99 s | 367 s | 27.81 s (14.7 mm!) | 38.30 s (4.5 mm) | 44.04 s (1.9 mm) | 43.70 s (1.7 mm) | **19.19 s** |

Reading the table:
- Without fix F-a, the default's "closest feasible time" is absurd. The endpoint snap alone sets it.
- With the fixes, the default is 1.6–2.0× slower than TOPP-RA at a comparable path change (TOPP-RA's 30 mm chain moves the pad 3.7–4.7 mm, `PIPE:335`). That is expected: uniform scaling slows the whole file to suit its worst corner.
- This is why the two optimisers coexist: the default is predictable and "looks like the recording"; TOPP-RA is fastest.

---

## 3. Target-time semantics (investigation item 3)

### 3.1 Definitions

- **T_target**: the operator's region time (the deck's cleaning time per region, F3), in wall-clock seconds.
- **Line count:** N_target = round((T_target − 0.2322) / 0.0056737) + 1 (`TRK:1558-1563`). The bytes are designed and checked at dt_gate = 5.0 ms, so the design duration is D = (N−1) × 5 ms.
  - Show "≈ 22.0 s (21.0–23.6 s)" using the 5.35–6.10 ms/line envelope (`TRK:1565-1566`).
  - The control app's run speed % changes only the transit, not the track (`C2:992`; `C4:189`).
- **T_min**: the smallest D whose bytes pass every hard limit (§4). Shown as wall time via the law.

### 3.2 TOPP-RA: compute T_min, then hit T_target

1. Solve once with the ALIX chain (§1.1). Sample with the `PMT.resample_clock` form (`PMT:375-394`). Check the bytes against the hard limits (§4).
2. If a byte check fails, apply k₀ = max(acc_util^½, vel_util / 0.97, G1 fix). The usual cause is the 50 ms windowed acceleration at 1.007 > 1 + tol, or post-LP overshoot. Resample `traj(t/k₀)` and re-check (cheap; at most 3 iterations).
   - If G1 fails, first re-solve with `arrival_hold` caps (`ETR:154-197`; `TP:130-137`). Slowing also fixes G1, because G1 depends on the arrival rate.
   - T_min = k₀ × T_opt.
3. **If D_target ≥ T_min: uniform time scaling.** k = D_target / T_opt with n = N_target − 1 lines and samples t_i = i·T_opt/n. Velocity scales by 1/k and acceleration by 1/k². The path is unchanged and the duration is exact to the tick.
   - **Equivalent to re-solving with scaled limits:** for k = 1.3 the re-solve gives T = 5.544 s = k·T_min, and the profiles agree to 0.0001° (k = 2: 0.0006°) [measured 13 Oct]. Uniform scaling skips a second 0.1–1.8 s solve, so it wins.
   - **Not ALIX's slow ladder.** `TP LADDER` scales velocity and acceleration by the same factor and re-solves (`TP:117-121`, `TP:140`). That is a different profile, and it needs a bisection of solves to hit a time. Keep it out of the target path. It stays reachable only through a MAX DEV budget (§4).
4. **If D_target < T_min:** return the T_min bytes. Report "closest feasible: 22.0 s (target 18.0 s)" and the binding constraint (§3.4).

### 3.3 Default optimiser: hit T_target, else the closest feasible time

1. Trim, then smooth (fixed, §2.3). The trimmed-and-smoothed file has duration D_rec.
2. Sub-regions carry a weight w_i:
   - the operator's "%", relative time, as in the prototype; or
   - a pinned duration P_i (advanced).
3. With free sub-regions F and pinned ones P, solve D_target = Σ_{i∈P} P_i + k · Σ_{i∈F} w_i·D_i. With blending this becomes a 1-D root of a monotone function; use the piecewise form as the starting point and refine with a secant (2–3 evaluations).
4. Feasibility is monotone in k: speed ∝ 1/k, acceleration ∝ 1/k² (exact for linear resampling, apart from rounding); G1 arrival rates ∝ 1/k; G5 departure falls with speed.
   - **Bisection** on log k over [0.05, 20], 25 iterations, about 0.3 s for 12k lines, checks on the integer bytes.
   - The smallest feasible k is k_min. Snap up by 0.5 % steps if rounding breaks monotonicity at the boundary.
   - If k ≥ k_min, use it; otherwise use k_min and report the binding constraint.
5. A pinned sub-region that is infeasible on its own is raised to its own minimum. The remainder is redistributed and the report names the sub-region.

### 3.4 Binding-constraint report

The report is the same for both optimisers.
- At the delivered bytes, compute per-line utilisation U[t, c] for c ∈ {J1–J7 speed, J1–J7 acceleration, G1 J1–J7, G5}.
- **TOPP-RA:** many constraints are active at once. Report the top 1–3 by share of time at ≥ 98 % of their bound, each with its longest interval. Example: "limited mostly by joint 6 acceleration (38 % of the time; longest 2.88–3.05 s), then joint 1 acceleration (15 %)" [FILE_10.1, measured 13 Oct].
- **Default:** report the check that fails at 0.995 · k_min: joint, quantity, and the time window where its utilisation is over 0.98 × its maximum. Example: "closest feasible: 41.2 s — limited by joint 6 speed in 12.3–14.0 s".
- Report times on the delivered file's timeline (nominal 5.67 ms/line, so they match the plots), and as source-recording times for the editor.

---

## 4. The shared limit set: hard limits vs checks (investigation item 4)

All limits are judged on the **integer millidegrees as written**, at dt_gate = 5.0 ms (`TT:94-102`; `PMT:554-557`). Both optimisers must pass every hard limit before the file can be uploaded or run. The hard limits also define "feasible" in the target-time search.

| id | limit | value and reading | hard? | why / source |
|---|---|---|---|---|
| L1 position (G1) | URDF limits (`KIN:43-46`) with margin | ALIX model G1: predicted executed margin = commanded margin − max(floor, τ_limit × arrival rate) ≥ `limit_eps_deg` 0.5° (`TT:466-480`; model `TT:196-223`; live values in `Resource/tracks/taught/EXECUTION_MODEL.json`). Also warn if the commanded margin is < 5° (`TT:122-126`). | **Hard** (ALIX enforces G1, `TT:2138-2145`). | It improves with slowing, so it belongs in the search. "Geometry" failures, which no speed fixes, are refused with "re-teach" (`TT:558-564`). |
| L2 speed (G2) | ratings 180/180/225×5 °/s (`TRK:3728`) | 20 Hz band-limited per-joint rate ≤ **0.97 × rating** (design allowance `TT:229`). G2 itself counts ticks over 1.0 × rating (`TT:594-599`). TOPP-RA designs at 0.95 (`PIPE:146`). | **Hard** | A raw single tick over the rating can still latch warning 4105 (`C2` fact 4; `CT:211-218`). Report the raw per-tick peak as a warning. |
| L3 acceleration | 600 °/s² (`KIN:39-41`; `JL:226`) | **50 ms windowed mean** of the per-tick velocity difference, (v(t+50 ms) − v(t)) / 50 ms ≤ 600 × 1.02. It cannot overshoot a true bound and is immune to LSB noise (~8 °/s²). The ALIX 20 Hz Butterworth reading is shown as advisory, as ALIX does (`TT:517-526`). | **Hard in Lite** (advisory in ALIX: `TT:45-50`; `PIPE:184-191`, `PIPE:560-561`). | F2 requires "obeys the limits". The default optimiser has no other protection: the prototype's boundary spikes reach 919 °/s². TOPP-RA bytes read 1.007 [measured]. Raising it is a deliberate setting with a warning, as ALIX's `+acc` (`TP:122-137`). |
| L4 step | 10° per line | max \|Δq\| per line ≤ 10° (`TT:127-128`; `TRK:3707`, `TRK:3823`) | **Hard** | Controller error 0x4000. In practice L2 binds first (225 °/s × 5 ms = 1.125°). |
| L5 path change (G4) | the recording | Exact point-to-polyline pad distance, both ways, against the **raw** recording (`TT:607-667`), not the derived file (ALIX gap, `PIPE:547-550`). ≤ 10 mm (5 preferred), or the region's MAX DEV if smaller. | **Hard**, but independent of time | A geometry precondition on smoothing (window or `smooth_mm`), not part of the time search. |
| L6 ends | the recorded start and end poses | First and last line within 0.05° of the recording's (after R4, the rest/start pose, `README:93-104`) | **Hard** | Keeps the runner's transit and the rest-pose logic valid. Removes the endpoint-snap class of defect. |
| C1 predicted executed departure (G5) | per-region budget | ALIX model G5 (`TT:586-593`; `TT:2165-2176`), upload-fitted, which suits Lite's upload channel. For CANFD-style regions with a MAX DEV, `TG.predict` = file vs raw + min(0.5 + 0.057·v_max, 7.5) mm (`TG:52-79`). | **Check (warning)** by default. **Hard** only when the region has a MAX DEV (`README:33-40`). | The model under-reads by about 2× (`TT:213-217`) and does not hold for CANFD (`PIPE:551-553`). |
| C2 jerk | 1500 rad/s³ (`JL:3-22`) | band-limited, reported | Check | TOPP-RA is bang-bang (`PMT:22-29`). |
| C3 pad speed | — | linear and angular pad speed (`PMT:306-345`) | Check | Information for the operator. |
| C4 raw per-tick rate | ratings | raw ticks over the rating | Warning | 4105 risk (`C2` fact 4). |

**Post-LP (6 Hz shake fix).**
- Off by default in Lite. It was a CANFD hand-shake fix (`PIPE:169-180`), and it raises acceleration to 1.17–1.25 × [measured].
- When it is on, the checks run after it, so T_min rises by about 6–12 % (√1.25).

---

## 5. Per-region and per-sub-region targets (investigation item 5)

- **The target is per region.** One region = one recording = one program (deck commode C, F3; `C1`). The time in Lite's region catalogue is the default T_target. The operator may override it per region.
- **Sub-regions (the prototype's "scaling regions", renamed "segments" in `C1:101-102`):**
  - **Default optimiser:** the time % per segment keeps the prototype's meaning (`H:1568-1616`), with blended boundaries (fix F-b). A pinned duration per segment is an advanced option. The region target sets the global k (§3.3).
  - **TOPP-RA:** a segment control means a **speed cap %** of the ratings in that segment. It is passed to the solver as `vcap` through `JointVelocityConstraintVarying` (`LS:287-310`; ALIX already feeds `arrival_hold` caps this way, `ETR:325-328`), so braking into the segment respects the limits by construction. The region target then applies uniform scaling (§3.2).
  - Segment time % is not offered for TOPP-RA: "slower here" is a cap, and "faster than optimal" cannot exist.
- **Dual-arm TOGETHER blocks** (the control app's dual rules TOGETHER / RUN FIRST / WAIT OTHER, `C1:117`; `UI53:6604-6656`):
  - In a TOGETHER block both arms' programs start together. Lite should give both the **same line count**: N_block = max(N_target, N_min_L, N_min_R). The binding arm is named ("limited by LEFT arm J6 acceleration").
  - Equal N gives equal nominal durations under the same player law, so the arms finish within the clock jitter. Per-pair start skew and clock spread are what `DAV` windows over (`DAV:40-60`).
  - **Any retiming of either arm invalidates the inter-arm clearance verdict.** `DAV` binds each verdict to both tracks' sha256 (`DAV:18-22`). Lite must re-run its dual-arm collision timeline (C3) on the retimed pair before upload.
  - RUN FIRST and WAIT OTHER blocks are independent per arm.
- **UI needs:**
  - **Region panel:**
    - optimiser choice (Default | Time-optimal (TOPP-RA)), with TOPP-RA greyed out and a tooltip when unavailable;
    - target time input (s), prefilled from the catalogue;
    - the achieved time and its band ("22.0 s, 21.0–23.6 s");
    - the recorded time and the shortest feasible time, always shown;
    - a status chip: "Target met" (green) or "Closest feasible: 41.2 s — limited by joint 6 speed in 12.3–14.0 s" (amber), with a "show on plot" link that highlights the window.
  - **Segment table:** default = time %; TOPP-RA = speed cap %. A pinned-seconds column is hidden behind "Advanced".
  - **Limit panel:** L1–L6 pass/fail with values; C1–C4 as notes; acceleration and speed utilisation plots with the binding window shaded.
  - **TOGETHER blocks:** both arms' T_min side by side, the common N and the binding arm, plus "clearance re-check required" until C3 passes.
  - **Wording for non-technical operators:**
    - "The robot cannot clean this region in 18 s without exceeding its joint limits. The fastest safe time is 22.0 s.";
    - "Use 22.0 s" button;
    - never a raw constraint id without its plain-language label.

---

## 6. Design (investigation item 6)

### 6.1 Modules

All modules live in `alix_studio_lite/lite/optimise/` (F7). ALIX is imported from the pinned checkout or bundle through one adapter (`alix_adapter.py`) that sets the paths once.

```text
lite/optimise/
  limits.py      LimitSet, load_limits(side, urdf_path, model_path) -> LimitSet       # ratings, 600, positions, G1 model
  checks.py      check_bytes(md, side, limits, *, raw_ref=None, max_dev_mm=None) -> CheckReport
                 util_series(md, limits) -> dict[str, ndarray]                          # per-line utilisation, for binding + plots
  timing.py      PlayerLaw (ramp 0.2322 s, 5.6737 ms/line, envelope 5.35-6.10); lines_for(T_wall); wall_for(N); band_for(N)
  prep.py        trim_idle_proto(md, win=11, thr=8, min_run=20)                        # prototype trim (H:1438-1492)
                 smooth_moving_mean(md, window=41)                                      # odd-reflect boxcar (fix F-a)
                 alix_prepare(md, side, smooth_hz=5, smooth_mm=30) -> TaughtPath        # TT.smooth_taught, TT.strip_pauses, LS.arclen_smooth
  default_opt.py warp(md, seg_weights, k, blend_s=0.25) -> md                          # blended time warp (fix F-b), points on chords
                 optimise_default(md_raw, side, target_s, segments, opts, limits) -> OptResult
  toppra_opt.py  available() -> bool                                                   # import toppra guarded
                 solve_min_time(path, limits, vcap=None) -> (traj, T_opt)               # Lite-owned retime (LS:282-324 parity)
                 sample(traj, n_lines) -> md                                            # PMT:375-394 form, int millidegrees
                 optimise_toppra(md_raw, side, target_s, segments, opts, limits) -> OptResult
  target.py      fit_target(make_bytes: Callable[[float], md], feasible, target_lines) -> (k, md, report)   # bisection/secant, shared
  binding.py     binding_report(util: dict, dt, threshold=0.98) -> list[Binding]
  together.py    align_together(left: OptRequest, right: OptRequest) -> (OptResult, OptResult)          # common N
  sidecar.py     write_sidecar(path, result)                                            # recipe, versions, origin sha, checks
```

Key types:

```python
@dataclass(frozen=True)
class LimitSet:
    rate_deg_s: np.ndarray          # (7,) 180,180,225x5
    rate_bar: float = 0.97          # L2 bar on the 20 Hz reading
    rate_design: float = 0.95       # TOPP-RA design scale
    acc_deg_s2: np.ndarray = 600.0  # (7,)
    acc_tol: float = 0.02           # L3 windowed-mean tolerance
    acc_window_s: float = 0.050
    step_deg: float = 10.0
    pos_deg: np.ndarray             # (7,2) from URDF / KIN:43-46
    exec_model: dict                # TT.execution_model() for G1/G5
    dt_gate: float = 0.005

@dataclass
class OptResult:
    md: np.ndarray                   # (N,7) int64 millidegrees
    optimiser: Literal["default", "toppra"]
    target_s: float; achieved_s: float; band_s: tuple[float, float]
    t_min_s: float; met: bool
    binding: list["Binding"]         # empty when met with slack
    checks: "CheckReport"            # L1-L6 pass/fail + C1-C4 values
    recipe: dict                     # every knob, versions (numpy/scipy/toppra), origin sha256

@dataclass
class Binding:
    kind: Literal["speed", "acceleration", "joint_limit", "path_accuracy"]
    joint: int | None; share: float; t0_s: float; t1_s: float; text: str
```

### 6.2 Data flow

```text
raw .txt (+ SOURCE sidecar, side) ──parse (TRK:66)──► md_raw
  ├─ DEFAULT:  trim_idle_proto ─► smooth_moving_mean(41, odd-reflect) ─► G4 check vs md_raw
  │            └─► fit_target( k -> warp(md_s, segments, k) , feasible=check_bytes ) ─► OptResult
  └─ TOPP-RA:  alix_prepare (5 Hz, R4, 30 mm) ─► G4 check vs md_raw
               └─► solve_min_time(path, limits, vcap=segment caps [+arrival_hold]) ─► T_opt
                   └─► fit_target( k -> sample(traj, n=k*T_opt/dt) , feasible=check_bytes ) ─► OptResult
OptResult ─► (TOGETHER?) align_together ─► C3 dual-arm collision timeline ─► upload (any ID 1-100, F6)
```

Run the solver in a worker process: x86 solve 0.1–1.8 s; expect roughly 2–5× on the Jetson (not measured). The UI stays responsive, and Cancel stays available.

### 6.3 Golden tests

Run them on x86-64 and aarch64. Inputs are the `CC` files, copied into Lite's test data with their sha256.

| test | input | expected |
|---|---|---|
| G-T1 TOPP-RA parity | FILE_01.1, 0.95 × ratings, 30 mm, no LP | 3839 lines (19.19 s at 5 ms) = slot 101 (`PIPE:588`); bytes within ±1 millidegree of the ALIX chain (`ETR:417-427`) |
| G-T2 TOPP-RA T_min | FILE_08 / FILE_10.1 / FILE_6 | 3360 lines / 4.26 s / 20.25 s, ±1 line |
| G-T3 uniform scaling | FILE_10.1, k = 1.3 and 2.0 | duration = k·T_min ± 1 tick; speed util ≤ 0.95/k (+2 %); windowed acceleration ≤ 1/k² (+2 %); path identical to the k = 1 path to ≤ 0.001° at matched fraction |
| G-T4 infeasible target | FILE_10.1, T_target = 3.0 s | met = false; achieved = T_min; binding[0] = J6 acceleration, share ≈ 38 %, window 2.88–3.05 s |
| G-D1 default end snap | FILE_08 trim + smooth(101) | first and last steps ≤ 100 millidegrees (was 966); first and last line = recording ± 0.05° |
| G-D2 default boundaries | FILE_01.1, segments [75, 80, 95, 115, 75] | windowed acceleration at every boundary ≤ 300 °/s² (was 640–919) |
| G-D3 default G4 | FILE_08 / FILE_10.1 / FILE_01.1 at window 41 | max pad deviation ≤ 4.6 mm; window 101 → G4 fails (> 10 mm) with a "reduce smoothing" hint |
| G-D4 default T_min | same three files, window 41 | 27.42 / 8.14 / 38.30 s ± 1 % (design at 5 ms) |
| G-D5 target met | FILE_01.1 default, T_target wall 60 s | achieved within ±1 line of `lines_for(60)`; all L1–L6 pass |
| G-X1 limits monotone | any file, k grid | every hard check passes for all k ≥ k_min (property test, 20 random files) |
| G-X2 TOGETHER | two files | equal N; N = max of the three |
| G-X3 no toppra | `toppra` import blocked | Lite runs; TOPP-RA option disabled with a message; default works |
| G-X4 writer | any result | `{"point":[…]}` lines with 7 ints; parses with `TRK.parse_track`; `.proj` CRLF |

### 6.4 Acceptance criteria

1. Every file Lite uploads or runs passes L1–L6 on its integer bytes at 5.0 ms. Upload is blocked otherwise. Warnings C1–C4 are shown, not blocking (C1 blocks only with a MAX DEV).
2. When T_target ≥ T_min, the achieved wall time (player law) equals `wall_for(lines_for(T_target))` to within 1 line. When T_target < T_min, Lite delivers T_min and shows the binding constraint (joint, quantity, time window) in plain language.
3. TOPP-RA bytes match ALIX's chain within ±1 millidegree and ±1 line on G-T1/G-T2. The default optimiser reproduces the prototype's line counts (±1 line per segment) when every fix is switched off (compat mode, for regression comparison only).
4. The default optimiser has no endpoint step larger than its interior maximum, and no boundary acceleration above L3.
5. The TOGETHER pair is uploaded only after the C3 timeline passes on the retimed bytes.
6. The sidecar records the optimiser, every parameter, numpy, scipy and toppra versions, the origin sha256 and every check value. Re-running gives the same bytes (±1 millidegree across platforms).
7. Both builds pass the golden tests: x86-64 (operator PC) and aarch64 (Jetson AGX Orin, Ubuntu 24.04, toppra built from source).
8. Relative size: the default-optimiser fixes plus the target search are **M**; the TOPP-RA wrapper, target and binding are **M**; checks and limits are **M**; TOGETHER alignment is **S**; the aarch64 toppra build is **S–M**; the UI panel is **M**.

---

## 7. Open points

1. **Which time is the operator's target?** This note assumes wall-clock played time (player law). If the catalogue times are "file time at 5 ms", Lite's numbers will read ~13 % longer. Confirm with the deck owner.
2. **L3 acceleration hard in Lite but advisory in ALIX.** Confirm that Newton accepts the 50 ms windowed reading at ≤ 1.02 × 600 as Lite's bar. An `+acc` override with a warning, as ALIX's, is the escape hatch.
3. **G1 model file.** Lite needs `Resource/tracks/taught/EXECUTION_MODEL.json` from the pinned ALIX checkout, or the defaults at `TT:196-223`. Pin which one.
4. **Post-LP 6 Hz.** It was fitted for CANFD shake. Its value on the upload channel is unmeasured. Default it to off.
5. **Jetson solve time and the toppra aarch64 build** are unmeasured. Build and time one solve on the Orin early.
6. **Right arm:** ALIX's TOPP-RA chain has run on hardware for left-arm recordings only (`PIPE:556-557`). Right-arm files should get a supervised first run.
7. **G5 on the upload channel:** the model is upload-fitted but under-reads by about 2×. Whether to show G5 as a number or only as a traffic light is a UI decision.

---

## Appendix A — how the 13 Oct measurements were made

- Interpreter: `/home/newtonjeri/alix_ws/src/alix/.venv-taught/bin/python`, with stdin scripts only; nothing was written.
- Paths added: `core/python/alix_dispatch/src`, `tools`, `tools/motion_analysis`, `experiments/exp01_…/src`.
- TOPP-RA chain: `tt.strip_pauses(tt.smooth_taught(raw, 5.0, 0.005))` → `LS.TaughtPath` → `LS.arclen_smooth(…, 30)` → `LS.TaughtPath` → `LS.retime_toppra(path, LS.demand("max", path), 0.005)` with `LS.RATE_DEG_S = CANFD_RATED_DEG_S × 0.95`.
- Binding and scaling checks used a direct toppra call with the same constraints, sampled on 20001 points.
- Prototype re-implementation in numpy: `trimIdle`, `centeredRollingMean`, `meanSmooth`, the linear resample, and an odd-reflect variant.
- Deviation: FK `cut_dossier.fk_tool(q, 'left', 'L_glove_frame_2')`; densified at 0.25 mm; `cKDTree` nearest distance, both directions; the maximum is reported.
- Checks:
  - speed and acceleration through a 4th-order 20 Hz Butterworth `filtfilt` at 5 ms;
  - the windowed acceleration = (v[i+10] − v[i]) / 50 ms on the rounded bytes;
  - bisection on log k with 30 iterations.
- Boundary test: FILE_01.1 `smooth_taught` at 5 Hz, five equal slices, [75, 80, 95, 115, 75] %; blend = two passes of `uniform_filter1d` over 0.25 s on the per-line time step.
- PyPI: `curl https://pypi.org/pypi/toppra/0.6.10/json` and `…/toppra/json`, plus `pip download --platform manylinux2014_aarch64` (no match).
