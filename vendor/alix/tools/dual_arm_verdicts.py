#!/usr/bin/env python3
"""Regenerate the inter-arm collision verdict table from the EMITTED TRACK BYTES.

WHY THIS EXISTS. Until 2026-09-08 the verdicts were a hardcoded dict in
`tools/dual_arm_flow.py:73-86` with (a) no link to the bytes they describe, so a
re-emit silently kept a clearance measured on a different track; (b) no
regeneration path — `grep -rn recheck` found nothing; and (c) a gate,
`verdict_for` (`:167`), that read only `min_mm` while `frozen_mm` was `None` on
both row-3 legs and read by nobody. Findings B7, B8, H17, H18, H21, H22, H27 and
H29 of Resource/reports/2026-09-08_dual_arm_e2e/DESIGN.md.

    tools/dual_arm_verdicts.py recheck          # regenerate (hours; see --only)
    tools/dual_arm_verdicts.py verify           # sha + shape gate, non-zero on drift
    tools/dual_arm_verdicts.py show             # print the table
    tools/dual_arm_verdicts.py certify --right T --left T [--row N]
                                                # [N-12]: may two tasks run together
                                                #   now? JSON (`certify_pair`)

WHAT IT WRITES. `Resource/tracks/commode_c/PAIR_VERDICTS.json`. Every pair row
carries the sha256 of BOTH sides' track files, so a re-emit invalidates the row,
and the pole height and hand counts each side was swept AT, so the executive can
refuse when the live device state differs (B7).

THE SEAM. `alix_workspace.dual_arm_plan_collision.cross_arm_separation` takes
PLAN JSON; a track is millidegree NDJSON. The shim (2026-09-05 DESIGN §3.11) is
the whole bridge: positions = millidegrees x pi/180000, `time_from_start` =
i x 0.0055545, PLUS constant seed columns for that side's pole
(`{L,R}_sliding_plate_joint`) and its six hand joints. Seeding is MANDATORY:
`_build_home` puts both poles at 0.30 m and both hands OPEN, which sits every
arm base 150-225 mm too high and, on row 11, reports 8.52 mm where the commanded
hand state gives 42.03. Timelines are built in memory and written to a temp plan
only at the call, and every call asserts the joint-name prefix of both sides
first — swapping the two arguments returns 33.23 mm with no warning.

FOUR NUMBERS PER PAIR, and EXACTLY ONE OF THEM GATES:

  min_mm_window     ** THE GATE ** (`gates.window.gates_on: true`). Minimum over
                    |offset| <= window_s: the drift two free-running arms
                    actually produce. H21 makes that window PER PAIR, from the
                    ledger's own measured start lags and replay clocks, instead
                    of one global 3.0 s literal, and since 2026-09-09 it is
                    floored by what the MEASURED clock envelope permits for
                    those two tracks' point counts — a pair with three runs has
                    not sampled a clock that spans 5.5-6.7 ms/pt. It
                    DISCRIMINATES — rows 6 and 7 read 0.00 mm, the rest tens of
                    mm — which is the whole reason it and nothing else may
                    decide `run`.
  min_mm_aligned    both tracks started together, sample i against sample i.
                    Context, not a criterion.
  min_mm_invariant  DIAGNOSTIC (`gates_on: false`). `offset_window_s=None` — the
                    exact lower bound over EVERY relative phase, which is what a
                    frozen or cut partner produces (H17). It reads 0.00 mm for
                    rows 6, 7, 10 and 11 alike, exactly as
                    `cross_arm_separation`'s docstring (:673-679) says it does
                    on this fixture, so it cannot discriminate and Newton ruled
                    on 2026-09-08 that it must not gate. The unbounded freeze is
                    the partner-stop watchdog's job (each row's `requires`).
  min_mm_dilated    DIAGNOSTIC (`gates_on: false`). The window sweep on tracks
                    perturbed by that task's measured replay deviation (H18) —
                    held sustained, worst-direction and on both arms at once,
                    which real tracking error is not. It too returns 0.00 mm on
                    every row. See `dilated_method`.

NEWTON'S RULING, 2026-09-08 (late), sits on top of the gate: rows 6, 7 and 11
run SERIAL, and everything else in the schedule runs CONCURRENT. It is keyed on
TASK NAMES in `NEWTON_SERIAL_PAIRS` so a schedule renumbering cannot detach it,
and `decide` applies it — a row's `ruling` block carries it so the next
generation of this file cannot quietly re-derive it away.

COST. A windowed dt=0.02 sweep runs for minutes to tens of minutes; an unbounded
one can run for an hour and has DNF'd before. Every sweep is cached on disk by
the inputs that change its answer, `recheck` is shardable with `--only`, and
anything not computed lands in `skipped` with its reason and `null` in the
table. A null is honest; a guess is not.
"""
from __future__ import annotations

import argparse
import datetime
import glob
import hashlib
import importlib.util
import json
import math
import os
import pathlib
import re
import statistics
import sys
import tempfile
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
for _p in ("core/python/alix_workspace/src", "core/python/alix_taskgen/src"):
    if str(REPO / _p) not in sys.path:
        sys.path.insert(0, str(REPO / _p))

FIXTURE = "commode_c"
TRACK_DIR = REPO / "Resource/tracks" / FIXTURE
MANIFEST = TRACK_DIR / "MANIFEST.json"
SIDECAR = TRACK_DIR / "PAIR_VERDICTS.json"
SCHEDULE = REPO / "ros/orchestration/alix_orchestrator/config/schedule_commode.csv"
REGIONS = REPO / "ros/orchestration/alix_orchestrator/config/regions_commode.csv"
RUNS_ROOT = REPO / "Resource/runs/tracks"
DEFAULT_CACHE = pathlib.Path(
    os.environ.get("ALIX_VERDICT_CACHE", "/tmp/alix_dual_arm_verdicts"))

#: The uploaded track's own clock. Every emitted sidecar carries it as
#: `clock_s`; `track_clock_s()` reads that and falls back to this only when a
#: sidecar is missing.
TRACK_CLOCK_S = 0.0055545

#: THE PLAYER CLOCK, AS MEASURED, AND IT IS LENGTH-AWARE (Newton, 2026-09-09:
#: "the clock based on the runs we have so far is between 7ms and 5.5ms, so we
#: need to account for this in the concurrent arm operation").
#:
#: WHERE THESE CAME FROM. `Resource/runs/tracks/2026-09-0*/run_slot*.json`,
#: every record with `complete is True` and a `replay_s` — 129 of the 157 on
#: file as of 2026-09-09. Ordinary least squares of `replay_s` on `points - 1`
#: gives
#:
#:     replay_s = 0.283036 s + 5.662173 ms/pt x (N - 1),   R^2 = 0.99777
#:
#: which reproduces the relation already on record (0.283 + 5.6622 ms x (N-1)).
#: The intercept is a REAL fixed cost — the program start ramp — and it is why
#: a naive ms/pt figure is length-dependent: over 129 complete runs the raw
#: `replay_s / (N-1)` spans 5.5317..6.6542 ms/pt, but SHORT tracks (<1500 pts)
#: sit at 5.72..6.65 and LONG ones (>=3000 pts) at 5.53..5.98, because the same
#: ~0.283 s ramp is 4 % of a short track and 1 % of a long one. Divide the ramp
#: out and the per-point rate has no length trend at all (median 5.686 short,
#: 5.660 medium, 5.661 long) and no side trend either (median 5.6765 ms/pt over
#: 102 right runs, 5.6461 over 27 left). So: ONE ramp, ONE rate band, applied
#: to each track's own point count.
#:
#: WHAT THEY ARE. The tightest pair of straight lines that CONTAIN all 166
#: (N-1, replay_s) points, then rounded OUTWARD so the stored constants still
#: contain the exact hull (verified: 0 of 166 runs fall outside):
#:
#:     replay_s(N) in [0.057 + 5.515 ms/pt x (N-1),  0.750 + 5.810 ms/pt x (N-1)]
#:
#: REFITTED 2026-09-10, on 166 complete runs against the 129 the first fit had.
#: The LOWER line moved and the upper did not: a complete run of
#: `bowl_inside_ring_right` (1799 pts, 9.9741 s, 5.5473 ms/pt) came in 11.5 ms
#: UNDER the old floor — the fastest of that track's seven complete runs, and a
#: clean record (`complete` true, `verdict` in_band, `capture_truncated` false,
#: blocking probe ret 0, so the finish EVENT arrived). Same criterion, same
#: exclusions, same point span (588..4281); only more data.
#: THIS IS WHY THE TEST RE-DERIVES IT EVERY RUN. Widening the floor WIDENS every
#: window the envelope permits, so the refit forced rows 3:0 and 3:1 to be
#: re-swept (they needed 2.9840 s against a swept 2.9732 s). Row 10's need FELL
#: 5.5 ms, and rows 6/7/11 run serial. Never widen this envelope without
#: re-checking `clock_envelope_window_s` against every pair's swept `window_s`.
#:
#: WHAT THEY ASSUME. (1) That the full observed range, not a percentile, is the
#: right bound — the same choice `drift_window_s` already made, because a
#: window that only usually holds is not a gate. (2) That the envelope may be
#: read at any N inside the measured span (588..4281 points); every commode_c
#: track is inside it, and NOTHING here extrapolates outside it — `_clock_band`
#: refuses to. (3) That the two controllers draw from the same distribution,
#: which the per-side medians above support and which pooling then makes the
#: right thing to do.
#:
#: WHAT IS DELIBERATELY EXCLUDED. Runs where `complete` is not True. A cut
#: stroke's `replay_s` is the time to the CUT while `points` is the whole
#: track, so `replay_s/(N-1)` is not a clock reading at all: the four cut runs
#: above 5.0 ms/pt read 5.1157..5.5461 and the 5.1157 one (slot 78,
#: `bowl_inside_front_right`, 3556 pts) is the single reading that drags a
#: naive "long-track minimum" from 5.53 down to 5.12 ms/pt. Including cut runs
#: would inflate row 10's divergence from 2.85 s to 4.55 s on the strength of a
#: stroke that never ran. A cut partner is a real hazard and it is the
#: partner-stop watchdog's job (each row's `requires`), not the clock model's.
CLOCK_ENVELOPE_HI = (0.750, 0.005810)
CLOCK_ENVELOPE_LO = (0.057, 0.005515)
CLOCK_FIT = (0.283036, 0.005662173)
CLOCK_ENVELOPE_RUNS = 166
CLOCK_ENVELOPE_POINTS = (588, 4281)

#: THE START LAG, pooled the same way and for the same reason: a pair whose two
#: tracks have only a handful of runs has an UNDER-SAMPLED skew, and its own
#: samples are not a bound.
#:
#: `start_lag_s` is the `go` -> first `run_state = 1` transition (see
#: `replay_stats`). Over the 118 complete runs from 2026-09-03 on:
#:
#:     S(N) = 0.3449 s + 0.4156 ms/pt x N,  R^2 = 0.683, residual sd 0.346 s,
#:     residuals in [-0.8545, +0.9742] s
#:
#: which reproduces the pooled fit on record (0.345 + 0.418 ms/pt x N, sd
#: ~0.35). The residual BAND is what is used, rounded outward.
#:
#: THE 2026-09-03 CUT IS NOT COSMETIC. The 11 complete runs from 09-01 and
#: 09-02 lag 0.017..0.069 s; every run from 09-03 on lags 0.379..2.958 s. That
#: is a launcher generation change, not clock noise, and pooling across it puts
#: the fit through two populations (intercept 0.205, R^2 0.53, sd 0.487). The
#: model describes the launcher the executive USES. The older runs are not
#: thrown away: `drift_window_s` UNIONS each track's own observed lags into the
#: band, so a 0.017 s lag still widens the window of a pair that has one.
#: REFITTED 2026-09-10, on 156 complete runs against the 118 the first fit used.
#: The fit itself barely moved (0.3449 -> 0.3514 s, 0.4156 -> 0.4104 ms/pt); the
#: RESIDUAL FLOOR dropped -0.855 -> -1.0674, and it was moved by exactly ONE
#: run: Newton's `commode_c_toplid_left` on 2026-09-10 started at 0.0601 s, the
#: only run under 0.30 s anywhere in the 2026-09-03+ population.
#:
#: The launcher-generation exclusion is KEPT (2026-09-01/02 runs, 0.017-0.069 s,
#: stay out). Including them instead would drop the floor to -1.765 and widen
#: every window by 0.92 s rather than 0.20 s — a far bigger claim resting on
#: runs already ruled out of this population.
#:
#: WHAT THE SPREAD ACTUALLY IS, and the fit does not know it: 88 of 167 complete
#: runs record a FAILED FIRST POLL (`transitions[0].error_ret = -2`) before
#: `run_state = 1`, and those read p50 1.836 s against 0.730 s for the runs
#: whose first poll answered. So most of this band is the cost of an unanswered
#: `rm_get_program_run_state`, not the launcher. It is still bounded the same
#: way — the full observed range, because a window that only usually holds is
#: not a gate — but a future session should measure the two separately rather
#: than fit one line through both.
#:
#: A LOWER FLOOR WIDENS EVERY WINDOW. This refit put all three concurrent rows
#: ~0.20 s outside their swept windows and forced a re-sweep. Never move these
#: without re-checking `clock_envelope_window_s` against each pair's `window_s`.
START_LAG_FIT = (0.351429, 0.00041)
START_LAG_RESID_S = (-1.0674, 0.9881)
START_LAG_RESID_SD_S = 0.346
START_LAG_RUNS = 156

#: THE CANFD DRIFT MODEL — the window for CANFD-CLOCKED (5.0 ms) tracks of ANY length (Newton, 2026-10-02: "Fit it
#: from the CANFD runs (Recommended)"; until it existed, rows on the nominal window were OPEN). The upload player's
#: envelope above stays for 5.5545 ms tracks; it is the wrong model for a 5.0 ms track, which CANFD streams
#: host-timed from its own first send on absolute deadlines and never re-times — so rows 3:0 (toplid_right, 7198
#: points) and 11 (top_right, 482) of the 10-02 re-plan fell outside its 588..4281-point span and were swept over
#: H22's NOMINAL +-1.6 s.
#:
#:     W = CANFD_SKEW_S + (E_max - E_min) + 2 x CANFD_RHO_PPM x max(L_a, L_b),    L = (N - 1) x 0.005 s
#:
#: FITTED by `tools/motion_analysis/canfd_drift.py` (evidence Resource/reports/2026-10-02_canfd_window/S2_model/)
#: from the corpus's CANFD hardware records, no new runs. 140 records, 125 complete host-timed streams, ALL
#: single-arm. What they measure:
#:   * THE HOST STREAM DOES NOT DRIFT: (t_last_line - t_first_send) - (lines - 1) x period = 0.021..0.166 ms
#:     (median 0.063) over 2.7..71.4 s runs, slope 0.2 ppm of duration, r = 0.21 — the last line's lateness,
#:     not accumulation; per-line lateness <= 3.18 ms, 0 frames late by a period.
#:   * THE ARM FOLLOWS ITS STREAM ~25 ms BEHIND: per pushed frame, E = the COVERING phase error (the time shift
#:     nearest 25.5 ms among the commanded poses within 0.05 deg of the arm's true-phase match). Per-joint
#:     constant lags, pooled within arm and joint (never across): left J1-J7 medians 24.5-27.5 ms, max 53 ms
#:     (J6, slot 54); right J1-J7 26-33 ms (5 runs at 5.5545 ms).
#:   * E = [-77, 210] ms, the full observed range (rounded outward) over the POPULATION: 66 complete runs of
#:     multi-joint strokes (dominant joint <= 0.5 of the travel; production tracks 0.183-0.395) with a 20 Hz
#:     commanded peak acceleration <= 2500 deg/s^2 (production 857-1400) — 61 left incl. all 31 of Newton's
#:     named runs (09-21 slots 52-59, 09-22 101-104), 5 right. Newton's named runs ALONE give the same range:
#:     both ends are slot 54 (TOPP-RA at 1.5x the rated acceleration on every joint — the run that failed on
#:     its TCP departure, 4.7 deg off-path p99), so the bound is the worst following on record.
#:   * CANFD_RHO_PPM = 172: the covering phase's first-fifth -> last-fifth movement on population runs of
#:     >= 20 s (slot 54: +4.2 ms over 30 s), against 13.9 ppm for the stream alone. It is added for EVERY
#:     length — the length-aware term that lets the model be read beyond the corpus's 71 s.
#:   * CANFD_SKEW_S = 0.046 s, the RELEASE skew, from the release path's MECHANISM, because the corpus holds no
#:     two-arm CANFD run: the two concurrent release services (arm_executive.cpp:2531) land in the same or the
#:     next controller-manager update (100 Hz, controllers_real.yaml:14) = 10 ms; each StrokeChannel polls its
#:     release flag every 2 ms (stroke_channel.cpp:1771); each then waits for a push frame newer than the release
#:     before line 0 (`fresh_gap`, stroke_channel.cpp:2348) = up to the longest push ARRIVAL gap on record,
#:     33.1 ms (slot 56; median per-run maximum 7.4 ms). The "0.50-0.52 ms" figure
#:     (docs/motion/CANFD_PRODUCTION_DRIVER.md:325) is two runs on the DOUBLE whose records were not kept: it is
#:     the same-tick case of this mechanism, not a bound.
#: ASSUMPTIONS (carried in the table): (1) both arms stream on CANFD — a 5.0 ms track routed to the upload
#: player plays on the player's clock and this window does not describe it; (2) the right arm at 5.0 ms follows
#: inside E — measured only at 5.5545 ms (5 runs, cover 20.3-36.6 ms, per-joint 26-33 ms), so the two-arm window
#: is DERIVED from per-arm ranges plus the release skew, never observed; (3) a cut, aborted or stalled stream is
#: the partner-stop watchdog's job, as for the upload model; (4) the C++ driver (ported 1:1, 252-case parity)
#: times its stream as the Python lane that made these records did.
CANFD_CLOCK_S = 0.005
CANFD_E_MS = (-77.0, 210.0)
CANFD_SKEW_S = 0.046
CANFD_RHO_PPM = 172.0
CANFD_MODEL_RUNS = 66
#: What fitted it, per (track sha, period): (sha256[:16], name, side, period_s, points, complete runs,
#: Newton-named, covering phase min ms, max ms). The test re-derives the model from the corpus every run.
CANFD_MODEL_TRACKS = (
    ("c238c5edfc5c0d7c", "50_taught_bwl_out_01_slow_left", "left", 0.0055545, 12864, 3, False, 2.494, 141.397),
    ("4c1de99922f85698", "51_taught_bwl_out_01_slow_left", "left", 0.00611, 9074, 3, False, -14.72, 132.738),
    ("a712981797bedbea", "commode_c_toplid_right_spline_0917", "right", 0.0055545, 790, 2, False, 20.936, 35.916),
    ("2bf9a9562018e22b", "spline_retimed_both_ways_right", "right", 0.0055545, 795, 3, False, 20.3, 36.632),
    ("4c1de99922f85698", "51_taught_bwl_out_01_slow_left", "left", 0.005, 9074, 2, False, -56.152, 183.744),
    ("03aab3a0cea706cc", "52_taught_bwl_out_01_slow_left_toppra30mm_dep16_5m_left", "left", 0.005, 8637, 3, True,
     19.041, 38.163),
    ("03aab3a0cea706cc", "52_taught_bwl_out_01_slow_left_toppra30mm_dep16_5m_left", "left", 0.0055, 8637, 1, True,
     19.152, 38.972),
    ("acb986039150558c", "53_taught_bwl_out_01_slow_left_toppra30mm_rate0_95_left", "left", 0.005, 7425, 3, True,
     18.53, 38.769),
    ("8f387f47e865b4d9", "54_taught_bwl_out_01_slow_left_toppra30mm_acc1_5_r_left", "left", 0.005, 6068, 3, True,
     -76.729, 209.268),
    ("8f0053fe7d6046b9", "55_taught_bwl_out_01_slow_left_toppra_acc15_j6x1_left", "left", 0.005, 6764, 3, True,
     -1.411, 74.125),
    ("9a60caa93b442908", "56_taught_bwl_out_01_slow_left_lp6hz_left", "left", 0.005, 6560, 2, True, 3.118, 53.836),
    ("50d44a22b815b200", "58_taught_bwl_out_v3_toppra_left", "left", 0.005, 4568, 2, True, 14.971, 34.977),
    ("963b86fb442408aa", "59_taught_bwl_out_v3_lp6hz_left", "left", 0.005, 4568, 2, True, 18.692, 40.045),
    ("6e9f06df971171ce", "101_taught_commode_c_lid_top_left", "left", 0.005, 3839, 6, True, 13.326, 43.897),
    ("4c0acf498a2dca06", "102_taught_commode_c_bowl_outside_left", "left", 0.005, 3958, 2, True, 14.572, 47.986),
    ("d77db5558e0e60aa", "103_taught_commode_c_seat_top_left", "left", 0.005, 3561, 2, True, 13.846, 43.667),
    ("08b2ef98903ae7a0", "104_taught_commode_c_rim_top_left", "left", 0.005, 2764, 2, True, 18.467, 37.133),
    ("89156013a0afd4e7", "105_taught_commode_c_bowl_inside_left", "left", 0.005, 6077, 1, False, 18.241, 42.958),
    ("31b360190e1e8b4a", "106_taught_commode_c_rim_side_left", "left", 0.005, 3249, 1, False, 15.745, 53.919),
    ("ed5429e25b53e5e4", "107_taught_commode_c_bowl_inner_back_left", "left", 0.005, 4995, 1, False, 19.529,
     50.184),
    ("d4f904e0fa741e37", "108_taught_commode_c_hinge_area_left", "left", 0.005, 4436, 1, False, 18.255, 40.049),
    ("d142b44a09cf0a58", "109_taught_commode_c_lid_seat_open_left", "left", 0.005, 1905, 2, False, 18.802, 37.57),
    ("76a086eaa560eda6", "110_taught_commode_c_seat_only_close_left", "left", 0.005, 3616, 1, False, 19.278, 39.404),
    ("95dc520b3ebcc90d", "111_taught_commode_c_lid_top_v2_left", "left", 0.005, 3058, 9, False, 14.387, 44.529),
    ("a2c1e7ca02131565", "113_taught_commode_c_lid_top_v2_2x_faster_left", "left", 0.005, 2511, 6, False, 14.937,
     43.224),
)
#: The model's provenance, written into every sidecar as `canfd_drift_model` (runs, shas, fit, coverage, and
#: the assumptions above), so a reader of the JSON alone can see what the CANFD windows rest on.
CANFD_MODEL = {
    "name": "canfd_drift",
    "fitted": "2026-10-02",
    "ruling": "Newton, 2026-10-02: \"Fit it from the CANFD runs (Recommended)\" — no new runs; use it for "
              "5 ms tracks of any length; the upload-player envelope stays for 5.5545 ms tracks",
    "formula": "W = skew_s + (E_max - E_min) + 2 x rho x max(L_a, L_b), L = (N - 1) x 0.005 s",
    "fit": {"E_ms": list(CANFD_E_MS), "E_observed_ms": [-76.729, 209.268],
            "E_named_only_ms": [-76.729, 209.268], "skew_s": CANFD_SKEW_S,
            "skew_terms_ms": {"cm_period": 10.0, "stroke_poll": 2.0, "push_arrival_gap_max": 33.055},
            "rho_ppm": CANFD_RHO_PPM, "rho_terms_ppm": {"stream": 13.94, "covering_phase": 171.72},
            "stream_end_drift_ms": [0.0212, 0.0626, 0.166], "stream_late_ms_max": 3.1757,
            "per_joint_lag_ms": {"left": "J1-J7 medians 24.5-27.5, max 53.0 (J6, slot 54)",
                                 "right": "J1-J7 26.0-33.0 (5 runs, 5.5545 ms)"}},
    "coverage": {"runs": CANFD_MODEL_RUNS, "named_runs": 31, "tracks": len({t[0] for t in CANFD_MODEL_TRACKS}),
                 "sides": {"left": 61, "right": 5}, "points": [790, 12864], "duration_s": [4.38, 71.45],
                 "periods_s": [0.005, 0.0055, 0.0055545, 0.00611], "f20_peak_acc_deg_s2": [152, 2131],
                 "dominant_joint_share_max": 0.5,
                 "population_rule": "complete host-timed CANFD runs of multi-joint strokes (dominant joint <= 0.5 "
                                    "of the total joint travel) with a 20 Hz commanded peak acceleration <= 2500 "
                                    "deg/s^2; the 59 others (single-joint probe sweeps, raw taught 43/49) are "
                                    "validation, listed with their reason in the evidence",
                 "length": "any: rho x L is added for every length"},
    "runs": [{"sha256_16": t[0], "name": t[1], "side": t[2], "period_s": t[3], "points": t[4],
              "complete_runs": t[5], "named": t[6], "cover_ms": [t[7], t[8]]} for t in CANFD_MODEL_TRACKS],
    "assumptions": [
        "both arms stream on CANFD: a 5.0 ms track routed to the upload player plays on the player's clock and "
        "this window does not describe it",
        "the right arm at 5.0 ms follows inside E: measured only at 5.5545 ms (5 runs, cover 20.3-36.6 ms, "
        "per-joint 26-33 ms); the corpus holds no two-arm CANFD run, so the pair window is DERIVED from the "
        "per-arm ranges plus the release skew",
        "the release skew is the release path's mechanism bound (one CM tick + the 2 ms stroke poll + the "
        "longest push arrival gap); the 0.50-0.52 ms on the double is its same-tick case, not a bound",
        "a cut, aborted or stalled stream is the partner-stop watchdog's job, as for the upload model",
        "the C++ driver (ported 1:1, 252-case parity) times its stream as the Python lane that made these "
        "records did",
    ],
    "tool": "tools/motion_analysis/canfd_drift.py",
    "evidence": "Resource/reports/2026-10-02_canfd_window/S2_model/tool_run/model.json",
    "evidence_sha256": "e4dd13d87079bfe5effe6be5dc563453deda4c4032cd22b81660e50aefa3db43",
}

#: dt is a CORRECTNESS parameter, not a speed knob (2026-09-05 §1): the maximum
#: relative link displacement between grid samples is 109 mm at dt=0.05 on row 7
#: and 44-47 mm at dt=0.02, against clearances of 35-80 mm. Never coarsen it to
#: make a sweep finish — drop the number and say so instead.
DT_S = 0.02
LINK_SET = "full"

#: Grids a figure may be ACCEPTED from, finest first. dt=0.02 is the target and
#: the only one the floor should be judged on; a coarser grid samples a SUBSET
#: of the configurations, so its minimum can only be too LARGE. That makes a
#: coarse below-floor answer a sound refusal and a coarse above-floor answer no
#: certification at all — the same epistemic status as the dilated witness, and
#: `gates.<kind>.dt` records which grid every figure came from.
ACCEPT_DT = (0.02, 0.05, 0.1)

#: Below this a pair is refused however the sweep labelled it. The same 25.0 mm
#: `CrossArmClearance` threshold as test_mode1_dual_schedule.py:444.
CLEARANCE_FLOOR_MM = 25.0

#: How far a side's plans may disagree about the move_to_rest pose and still be ONE rest pose for the
#: idle table (2026-10-02): cleaning_tracks.side_rest_pose's own tolerance. An OMPL-planned
#: move_to_rest ends within MoveIt's 1e-4 rad joint-goal tolerance (measured 0.0057 deg on the
#: P18 + staging side_right), never on the pose to the 4th decimal. See `rest_pose`.
REST_POSE_TOL_DEG = 0.01

#: THE ONE GATING FIGURE. Everything else in `gates` is a recorded diagnostic
#: and carries `gates_on: false`. See `GATE_POLICY` for why.
GATE_KIND = "window"
DIAGNOSTIC_KINDS = ("invariant", "dilated")

#: Why each diagnostic may not gate. One line each, copied into every row so a
#: reader of the JSON alone never has to come back here.
DIAGNOSTIC_NOTE = {
    "invariant": (
        "DIAGNOSTIC, NEVER A GATE: `cross_arm_separation`'s own docstring "
        "(dual_arm_plan_collision.py:673-679) records that unbounded, the "
        "answer on this fixture is 0.00 mm for EVERY concurrent pair — this "
        "run reproduces that on rows 6, 7, 10 and 11 alike. A criterion that "
        "fails everything cannot discriminate. The unbounded freeze case is "
        "answered by the partner-stop watchdog and the rung-0 detector, which "
        "is what `requires` names, not by refusing pairs."),
    "dilated": (
        "DIAGNOSTIC, NEVER A GATE: the model holds the measured p99 deviation "
        "as a SUSTAINED, worst-direction, gap-closing offset on BOTH arms for "
        "the whole stroke. Real tracking error is transient and not "
        "adversarially aligned, so this bounds nothing the robot does — and it "
        "too reads 0.00 mm on every row it was computed for, so it "
        "discriminates nothing. Before it may gate anything it has to be made "
        "transient/bounded AND shown to separate rows."),
}

#: NEWTON'S RULING, 2026-09-08 (late), verbatim in substance: three pairs run
#: SERIAL, one arm at a time, and "anything outside of this is not correct".
#:
#: Keyed on the two TASK NAMES, not on (row, leg): the schedule renumbers when
#: a region is added or dropped, and a ruling that fell off its row silently
#: would be re-derived away by the gate — which is the exact failure this
#: constant exists to prevent. A pair Newton did not name is not covered by the
#: ruling and is decided by the windowed gate alone.
NEWTON_RULING_DATE = "2026-09-08"
NEWTON_SERIAL_PAIRS = {
    frozenset(("commode_c_bottomlid_right", "commode_c_seat_ring_left")):
        "bottomlid_right <-> seat_ring_left",
    frozenset(("commode_c_seat_ring_right", "commode_c_bottomlid_left")):
        "seat_ring_right <-> bottomlid_left",
    frozenset(("commode_c_top_right", "commode_c_top_left")):
        "top_right || top_left (the commode top pairing)",
}

#: The other half of the same ruling, written out rather than inferred as "not
#: in the serial list". The difference matters on a RE-EMIT: a schedule change
#: can create a pair Newton has never seen, and treating it as ruled concurrent
#: because it is absent from a three-line list would be inventing his ruling.
#: A pair in NEITHER list is UNRULED and is decided by the windowed gate alone.
NEWTON_CONCURRENT_PAIRS = {
    frozenset(("commode_c_toplid_right", "commode_c_side_left")):
        "toplid_right || side_left (row 3 leg A)",
    frozenset(("commode_c_side_right", "commode_c_toplid_left")):
        "side_right || toplid_left (row 3 leg B)",
    frozenset(("commode_c_seat_ring_bottom_right",
               "commode_c_seat_ring_bottom_left")):
        "seat_ring_bottom R || L (row 10)",
}

#: The preconditions concurrency rests on. These are NOT satisfied by anything
#: in this file — they are ladder gates someone has to run — and they are
#: carried per row so an executive cannot read `run: "concurrent"` as "cleared".
CONCURRENCY_REQUIRES = (
    "G4: slow_stop actually aborts a RUNNING uploaded program (never tested)",
    "D1: the stop distance a partner-stop leaves, measured",
)

#: What the sidecar states about its own gate, so a reader of the JSON alone
#: cannot mistake a diagnostic for a criterion.
GATE_POLICY = {
    "gate": "min_mm_window",
    "gate_definition": (
        "the minimum over |relative offset| <= the pair's own measured drift "
        "window (`window_s`: H21's per-pair term, floored 2026-09-09 by the "
        "measured clock envelope for those two point counts; for two 5.0 ms "
        "CANFD-clocked tracks the CANFD drift model, 2026-10-02), at dt<=%s on "
        "link_set %r, with BOTH poles and BOTH hands seeded from the pair's "
        "own `pre_stroke` (B7), against the %.1f mm floor. A figure swept over "
        "a NARROWER window than `window_s` is an upper bound and certifies "
        "nothing." % (DT_S, LINK_SET, CLEARANCE_FLOOR_MM)),
    "why_this_one": (
        "it DISCRIMINATES: rows 6 and 7 read 0.00 mm (the two arms share the "
        "volume at every phase, aligned included) while the other rows read "
        "tens of mm. A gate that fails every row is not a gate."),
    "not_gates": {
        "min_mm_invariant": DIAGNOSTIC_NOTE["invariant"],
        "min_mm_dilated": DIAGNOSTIC_NOTE["dilated"],
    },
    "ruled_out_by": (
        "alix_workspace/dual_arm_plan_collision.py:673-679 — "
        "\"Unbounded, the answer on this fixture is 0.00 mm for every "
        "concurrent pair measured (2026-08-20) ... A criterion that fails "
        "everything cannot discriminate. Bounded to a realistic drift it "
        "becomes a usable gate.\""),
    "ruling": (
        "Newton, %s: three pairs run SERIAL — %s — and the other three in this "
        "schedule run CONCURRENT — %s. Both halves are written out and keyed "
        "on TASK NAMES (NEWTON_SERIAL_PAIRS / NEWTON_CONCURRENT_PAIRS), so a "
        "schedule renumbering cannot detach the ruling from its pair and a "
        "pair Newton never saw is not silently counted as ruled. `decide` "
        "applies the ruling on top of the windowed gate; a ruled-concurrent "
        "pair whose gate later falls below the floor comes back `refused` with "
        "the disagreement stated, never quietly re-derived."
        % (NEWTON_RULING_DATE,
           "; ".join(sorted(NEWTON_SERIAL_PAIRS.values())),
           "; ".join(sorted(NEWTON_CONCURRENT_PAIRS.values())))),
    "requires": list(CONCURRENCY_REQUIRES),
}

#: H22's barrier release skew, from the 2026-09-08 design's own prescription:
#: "re-sweep at +-1.6 s around zero" (DESIGN.md rows H22, D3.5 and open
#: question 9). Used ONLY to give a pair whose ledger cannot supply a measured
#: window (row 3 leg B — the left half's bytes have never been played) a real
#: geometric number. Such a row carries `window_s: null`, a `requires` entry
#: naming B15, and a `skipped` note saying its figure rests on this NOMINAL
#: window; per Newton's 2026-09-08 ruling the caveat is recorded, not used to
#: refuse the pair.
BARRIER_SKEW_S = 1.6

#: Idle-arm park heights swept for B8. 0.075 and 0.150 are the only heights any
#: task commands, so the low one is the state the flow always leaves; 0.300 is
#: the proposed parked state.
IDLE_POLE_M = (0.075, 0.150, 0.300)

MILLIDEG_TO_RAD = math.pi / 180000.0
SIDE_PREFIX = {"right": "R", "left": "L"}


# --------------------------------------------------------------- joint naming
def arm_joint_names(side: str) -> list:
    return [f"{SIDE_PREFIX[side]}_joint{i}" for i in range(1, 8)]


def pole_joint(side: str) -> str:
    return f"{SIDE_PREFIX[side]}_sliding_plate_joint"


def _hand_units():
    """`ros/hw/alix_hw/scripts/hand_units.py`, loaded by path.

    It is a script in a ROS package, not an importable module, and it is the
    repo's single source for the 0-1000 <-> radian law (it mirrors
    `alix_hw/include/alix_hw/conversions.hpp`). Re-deriving the law here would
    be a second copy of exactly the constant that must not fork.
    """
    if getattr(_hand_units, "_mod", None) is None:
        path = REPO / "ros/hw/alix_hw/scripts/hand_units.py"
        spec = importlib.util.spec_from_file_location("alix_hand_units", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _hand_units._mod = mod
    return _hand_units._mod


def hand_rad(side: str, counts) -> dict:
    """The six actuated hand joints, in radians, from the SDK's 0-1000 counts.

    Input order is the SDK/register order the MANIFEST records:
    little, ring, middle, index, thumb_bend, thumb_rot.
    """
    u = _hand_units()
    # None / negative = a channel the plan did not model (exclude_thumb): it
    # sets no joint, so the full model keeps that thumb at its default.
    return {u.joint_name(side, ch): u.hw_to_rad(float(c), ch)
            for ch, c in enumerate(counts) if c is not None and c >= 0}


# -------------------------------------------------------------- the manifest
def sha256_file(path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for blk in iter(lambda: fh.read(1 << 20), b""):
            h.update(blk)
    return h.hexdigest()


def _sha_or_reason(path):
    """(sha, None) or (None, why). `verify_sidecar` must not raise on I/O."""
    try:
        return sha256_file(path), None
    except Exception as exc:                                    # noqa: BLE001
        return None, f"{type(exc).__name__}: {exc}"


def load_manifest() -> dict:
    return json.loads(MANIFEST.read_text())


def emitted_tasks(man: dict) -> dict:
    """name -> task, EMITTED rows only."""
    return {t["name"]: t for t in man["tasks"] if t.get("status") == "EMITTED"}


def track_clock_s(task) -> float:
    side_json = REPO / str(task["track"]).replace(".txt", ".json")
    if side_json.exists():
        c = json.loads(side_json.read_text()).get("clock_s")
        if c:
            return float(c)
    return TRACK_CLOCK_S


# ---------------------------------------------------------------- timelines
def track_points(path) -> list:
    """The track's integer millidegree rows, in file order."""
    rows = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append([int(v) for v in json.loads(line)["point"]])
    return rows


def build_timeline(task, points=None, offsets_rad=None) -> tuple:
    """A track, as the `(times, rows)` timeline `cross_arm_separation` consumes.

    `offsets_rad` is H18's dilation: seven signed radians added to the ARM
    joints only, for the whole stroke. The pole and the hand are staged before
    the stroke and held by their own controllers; a replay deviation on the arm
    says nothing about them, so they never move here.
    """
    side = task["side"]
    if points is None:
        points = track_points(REPO / task["track"])
    clock = track_clock_s(task)
    arm = arm_joint_names(side)
    off = [0.0] * 7 if offsets_rad is None else [float(x) for x in offsets_rad]
    pre = task.get("pre_stroke") or {}
    const = {}
    if pre.get("pole_m") is not None:
        const[pole_joint(side)] = float(pre["pole_m"])
    if pre.get("hand_counts"):
        const.update(hand_rad(side, pre["hand_counts"]))
    times, rows = [], []
    for i, pt in enumerate(points):
        row = dict(const)
        for j, name in enumerate(arm):
            row[name] = pt[j] * MILLIDEG_TO_RAD + off[j]
        times.append(i * clock)
        rows.append(row)
    return times, rows


def hold_timeline(side: str, joints_rad: dict, pole_m: float, hand_counts) -> tuple:
    """One sample: the arm parked at a fixed pose for all time.

    `load_plan_timeline` clamps outside its range, so a single waypoint IS a
    constant pose over whatever duration the partner's timeline sets. This is
    B8's idle arm.
    """
    row = dict(joints_rad)
    row[pole_joint(side)] = float(pole_m)
    row.update(hand_rad(side, hand_counts))
    return [0.0], [row]


def timeline_to_plan(timeline, name: str) -> dict:
    """The one-stage plan JSON shim.

    Times round to 6 dp (1 us against a 5.5545 ms clock); POSITIONS do not
    round, so the shim carries exactly `millidegrees x pi/180000` and the sweep
    sees the uploaded bytes and nothing else.
    """
    times, rows = timeline
    joint_names = sorted({k for r in rows for k in r})
    return {
        "task_name": f"shim::{name}",
        "num_sub_trajectories": 1,
        "sub_trajectories": [{
            "index": 0, "stage_name": "execute_path",
            "joint_names": joint_names, "num_waypoints": len(times),
            "waypoints": [{"time_from_start": round(t, 6),
                           "positions": [float(r.get(j, 0.0))
                                         for j in joint_names]}
                          for t, r in zip(times, rows)],
        }],
    }


# ------------------------------------------------- the swapped-argument gate
class SwappedArmsError(AssertionError):
    """A timeline was handed to the wrong side of `cross_arm_separation`."""


def timeline_side(timeline) -> str:
    """"left"/"right" from the joint names the timeline actually drives.

    THE CHECK THAT MATTERS. `cross_arm_separation(left_plan, right_plan)` FK-
    poses the LEFT manager from whatever it is handed; give it a right-arm
    timeline and no joint name matches, so the left arm silently stays at its
    home pose and the call returns a plausible number. 33.23 mm — which is, not
    coincidentally, the figure `test_mode1_dual_schedule.py:400` asserts for
    both arms at home.
    """
    _times, rows = timeline
    names = {k for r in rows for k in r}
    left = {n for n in names if n.startswith(("L_", "ILH_"))}
    right = {n for n in names if n.startswith(("R_", "IRH_"))}
    if left and right:
        raise SwappedArmsError(
            f"timeline drives BOTH arms ({sorted(left)[:2]} and "
            f"{sorted(right)[:2]}) — it cannot be either argument")
    if not left and not right:
        raise SwappedArmsError("timeline drives no L_/R_ arm joints")
    return "left" if left else "right"


def assert_arm_sides(left_timeline, right_timeline) -> None:
    ls, rs = timeline_side(left_timeline), timeline_side(right_timeline)
    if ls != "left" or rs != "right":
        raise SwappedArmsError(
            f"cross_arm_separation(left={ls}, right={rs}): the arguments are "
            "swapped or same-sided")


# ------------------------------------------------------------------ the sweep
def separation_context():
    """The FCL managers + FK model, built once per process (lru-cached upstream).

    Exposed here so `sweep` touches it AFTER the side assertion — building it
    loads and decimates every collision mesh on both arms, which is ~10 s that
    a swapped call must never pay.
    """
    from alix_workspace.dual_arm_plan_collision import (_DEFAULT_URDF,
                                                        _separation_context)
    from alix_workspace.srdf_states import find_srdf
    urdf = str(_DEFAULT_URDF)
    return _separation_context(urdf, find_srdf(str(pathlib.Path(urdf).parent)),
                               LINK_SET)


def sweep(left_timeline, right_timeline, dt: float = DT_S, window_s=None,
          invariant: bool = True, progress=None, link_set=None) -> dict:
    """`cross_arm_separation` on two in-memory timelines.

    `window_s` is a +- HALF-WIDTH (`offset_window_s` pairs left sample i with
    right sample j while |i-j| <= window/dt); `None` is the offset-INVARIANT
    sweep over every relative phase.

    `link_set` overrides the module's screen set for ONE call. It exists so the
    `hands` phase can be swept without changing what the arms figure was
    measured on — see `cmd_recheck`'s `--phases hands`.
    """
    assert_arm_sides(left_timeline, right_timeline)
    separation_context()
    from alix_workspace.dual_arm_plan_collision import cross_arm_separation
    with tempfile.TemporaryDirectory(prefix="alix_shim_") as td:
        d = pathlib.Path(td)
        (d / "L.json").write_text(json.dumps(timeline_to_plan(left_timeline, "L")))
        (d / "R.json").write_text(json.dumps(timeline_to_plan(right_timeline, "R")))
        return cross_arm_separation(
            str(d / "L.json"), str(d / "R.json"), dt=dt,
            link_set=(link_set or LINK_SET),
            offset_invariant=invariant, offset_window_s=window_s,
            progress=progress)


def pose_distance_mm(joints: dict) -> float:
    """FCL left-vs-right minimum distance for ONE merged joint map, in mm.

    Used for the dilation's finite differences; the same managers the sweep
    uses, so the two numbers are on the same scale.
    """
    ctx = separation_context()
    q = dict(ctx["home"])
    q.update(joints)
    T = ctx["model"].link_world_transforms(q)
    ctx["left_cm"].pose(T)
    ctx["right_cm"].pose(T)
    d, _names = ctx["left_cm"].mgr.min_distance_other(ctx["right_cm"].mgr,
                                                     return_names=True)
    return max(0.0, float(d)) * 1000.0


def closest_pair_label(res) -> str:
    p = (res or {}).get("min_pair") or (res or {}).get("aligned_pair")
    return f"{p[0]} <-> {p[1]}" if p else "unknown"


# ------------------------------------------------------------------- caching
def cached(cache_dir, key: dict, fn):
    """Memoise `fn()` on disk under everything in `key` that changes the answer.

    The key is the whole dict, canonicalised — a sweep is hours, and the one
    thing worse than recomputing it is silently REUSING it after an input
    changed. Written through a temp file so shards never read a half-write.
    """
    cache_dir = pathlib.Path(cache_dir)
    cache_dir.mkdir(parents=True, exist_ok=True)
    blob = json.dumps(key, sort_keys=True, separators=(",", ":"), default=str)
    path = cache_dir / f"{hashlib.sha256(blob.encode()).hexdigest()[:32]}.json"
    if path.exists():
        try:
            return json.loads(path.read_text())["value"]
        except Exception:                                       # noqa: BLE001
            pass
    value = fn()
    if value is not None:
        tmp = path.with_suffix(f".tmp{os.getpid()}")
        tmp.write_text(json.dumps({"key": key, "value": value}, default=str))
        os.replace(tmp, path)
    return value


# ------------------------------------------------------------------ the ledger
def _ledger_records() -> list:
    out = []
    for p in sorted(glob.glob(str(RUNS_ROOT / "*" / "run_slot*.json"))):
        try:
            d = json.loads(pathlib.Path(p).read_text())
        except Exception:                                       # noqa: BLE001
            continue
        d["_path"] = p
        out.append(d)
    return out


def _canfd_ledger_records() -> list:
    """Every HARDWARE CANFD run record (`canfd_slot*.json`, not under `emulated/`) — read for ONE question only:
    B15's byte question (`canfd_completed_runs`). ONE SET OF BYTES (Newton 2026-10-02: "any plans can be
    channeled to CANFD or track upload channel irregardless of the channel set during generation"): a completed
    CANFD run streams the very lines the upload player would play, identified by the same `track_sha256`, so it
    settles "have these bytes been played" as an upload run does. Never a TIMING record: the per-pair window term
    and the clock envelope stay on `_ledger_records` (the upload player's runs)."""
    out = []
    for p in sorted(glob.glob(str(RUNS_ROOT / "*" / "canfd_slot*.json"))):
        if "emulated" in pathlib.Path(p).parts:
            continue
        try:
            d = json.loads(pathlib.Path(p).read_text())
        except Exception:                                       # noqa: BLE001
            continue
        if str(d.get("sdk") or "") != "hardware":
            continue
        d["_path"] = p
        out.append(d)
    return out


def canfd_completed_runs(canfd_recs, name: str, sha: str) -> int:
    """How many COMPLETED hardware CANFD runs played task `name`'s bytes `sha` (`complete` is the finish-event
    label, as for an upload run — a cut stream does not settle anything)."""
    return sum(1 for d in canfd_recs or ()
               if d.get("name") == name and d.get("track_sha256") == sha and d.get("complete") is True)


def replay_stats(recs, name: str, sha: str) -> dict:
    """Measured start lag and replay clock for ONE task's CURRENT bytes.

    H27: partitioned by `track_sha256`. A run of retired bytes describes a track
    that no longer exists — 4 of 12 stale-byte runs cut against 3 of 113 current
    ones (Fisher p = 0.0015) — and where a task has NO current-byte run its
    timing is UNMEASURED, which is not the same as safe. `toplid_left` is that
    case: every recorded run used bytes that are not the ones on disk.

    `start_lag_s` is the `go` -> first `run_state = 1` transition. It is NOT
    `+ transit_s` as the free-running model in the 2026-09-08 design writes it:
    under the row barrier both arms are held at their verified stroke start and
    the two `go`s are released together, so the bridge and its `transit_s` sit
    inside each arm's own pre-stroke block, before the release. It matters —
    `transit_s` spans 0.05-6.43 s across flow generations and including it
    inflates row 7's envelope from 2.03 s to 13.40 s on records whose 10-11 s
    bridge no dual executive will reproduce.
    """
    mine = [d for d in recs if d.get("name") == name]
    cur = [d for d in mine if d.get("track_sha256") == sha]
    good = [d for d in cur if d.get("complete") is True and d.get("replay_s")]
    lags, replays = [], []
    for d in good:
        s = next((float(t["t"]) for t in (d.get("transitions") or [])
                  if t.get("run_state") == 1), None)
        if s is None:
            continue
        lags.append(s)
        replays.append(float(d["replay_s"]))
    return {
        "n_runs_any_bytes": len(mine),
        "n_runs_current_bytes": len(good),
        "start_lag_s": lags, "replay_s": replays,
        "median_start_lag_s": statistics.median(lags) if lags else None,
        "median_replay_s": statistics.median(replays) if replays else None,
    }


def _clock_band(points: int) -> tuple:
    """(min, max) real seconds a track of `points` points can replay in.

    The measured envelope of `CLOCK_ENVELOPE_LO/HI`, evaluated at this track's
    own length. Raises outside the measured span rather than extrapolating: a
    bound read off a line nobody measured there is not a bound.
    """
    n = int(points)
    lo_n, hi_n = CLOCK_ENVELOPE_POINTS
    if not lo_n <= n <= hi_n:
        raise ValueError(
            f"{n} points is outside the measured clock envelope "
            f"({lo_n}..{hi_n} points, {CLOCK_ENVELOPE_RUNS} complete runs). "
            "The envelope is not extrapolated — measure a track of this "
            "length, or state the window as unmeasured.")
    return (CLOCK_ENVELOPE_LO[0] + CLOCK_ENVELOPE_LO[1] * (n - 1),
            CLOCK_ENVELOPE_HI[0] + CLOCK_ENVELOPE_HI[1] * (n - 1))


def _start_lag_band(points: int) -> tuple:
    """(min, max) seconds from `go` to `run_state = 1` for a track that long.

    The pooled fit plus the pooled RESIDUAL band, not this pair's own samples —
    which is the whole point: a pair with three runs has seen three draws from
    a distribution whose measured spread is 1.83 s wide.
    """
    s_hat = START_LAG_FIT[0] + START_LAG_FIT[1] * int(points)
    return (s_hat + START_LAG_RESID_S[0], s_hat + START_LAG_RESID_S[1])


def _phase_intervals(t, lag, dur, length, total):
    """Where on ITS OWN nominal timeline an arm can be said to be, at time `t`.

    An INTERVAL, not a point, and that is what makes the answer honest. The
    sweep pairs `left` at nominal time `tau` with `right` at `tau + offset`,
    and `_sample_at` CLAMPS past the end of a track — so once an arm has
    finished, every `tau >= length` names the same held pose and the offset
    needed to reach it collapses. Without the clamp a pair of tracks 6 s apart
    in nominal length would "need" a 6 s window to describe two arms that
    simply finished at different times, which the aligned (offset 0) sweep
    already covers.
    """
    u = (t - lag) * length / dur
    if u < 0.0:
        return (0.0, 0.0)                  # not started: pinned at sample 0
    if u > length:
        return (length, total)             # finished: holding its last pose
    return (u, u)


def clock_envelope_window_s(points_a, points_b, a=None, b=None) -> float:
    """The window the MEASURED clock envelope permits for these two tracks.

    THE MODEL. Arm x is released at wall time `A_x` and replays its `N_x`
    commanded points in `T_x` real seconds, so at wall time `t` it stands at
    nominal-timeline position `(t - A_x) * L_x / T_x`, clamped into
    `[0, L_x]`, where `L_x = (N_x - 1) * TRACK_CLOCK_S` is the timeline
    `build_timeline` writes. The sweep is parameterised on exactly that axis:
    at offset `d` it pairs left nominal time `tau` with right nominal time
    `tau + d`. So the window this pair needs is the largest
    `|tau_right - tau_left|` the envelope can produce, over every wall time and
    every corner of the parameter box.

    WHERE THE BOX COMES FROM. `A_x` from `_start_lag_band` (pooled fit +
    pooled residual band), `T_x` from `_clock_band` (the measured replay
    envelope). Each is UNIONED with this pair's own observed samples when it
    has any (`a`/`b` are `replay_stats` dicts), so the floor never discards a
    measurement — including the 2026-09-01/02 runs the start-lag fit excludes.

    WHY THE CORNERS. `|tau_r - tau_left|` is piecewise linear in `t` with
    breakpoints only at the four release and finish times, and monotone in each
    of the four parameters between them, so enumerating the 16 corners at their
    own breakpoints is EXACT for this box — not a sampled approximation.

    WHAT IT ASSUMES. That the two arms are released together (the row barrier
    holds them at their verified stroke starts and releases both `go`s at once
    — see `replay_stats` on why `transit_s` is not in this), that the player's
    rate is constant within a stroke (the ramp is the intercept, and R^2 =
    0.998 says the residual is small), and that both controllers draw from the
    pooled distribution. It does NOT assume anything about a CUT: a partner
    that stops dead is outside every bounded window and is the partner-stop
    watchdog's job.
    """
    la, ha = _start_lag_band(points_a)
    lb, hb = _start_lag_band(points_b)
    ta = _clock_band(points_a)
    tb = _clock_band(points_b)
    if a:
        if a.get("start_lag_s"):
            la, ha = min(la, min(a["start_lag_s"])), max(ha, max(a["start_lag_s"]))
        if a.get("replay_s"):
            ta = (min(ta[0], min(a["replay_s"])), max(ta[1], max(a["replay_s"])))
    if b:
        if b.get("start_lag_s"):
            lb, hb = min(lb, min(b["start_lag_s"])), max(hb, max(b["start_lag_s"]))
        if b.get("replay_s"):
            tb = (min(tb[0], min(b["replay_s"])), max(tb[1], max(b["replay_s"])))

    La = (int(points_a) - 1) * TRACK_CLOCK_S
    Lb = (int(points_b) - 1) * TRACK_CLOCK_S
    total = max(La, Lb)
    worst = 0.0
    for Ta in ta:
        for Tb in tb:
            for Aa in (la, ha):
                for Ab in (lb, hb):
                    for t in (Aa, Ab, Aa + Ta, Ab + Tb):
                        alo, ahi = _phase_intervals(t, Aa, Ta, La, total)
                        blo, bhi = _phase_intervals(t, Ab, Tb, Lb, total)
                        worst = max(worst, 0.0, alo - bhi, blo - ahi)
    return round(worst, 4)


def is_canfd_clock(clock_s) -> bool:
    """Is this the CANFD stream's own clock (5.0 ms)? A missing or unreadable clock is NOT: a row written
    before 2026-10-02 carries no `clock_s` and was measured on the upload model, which stays its model."""
    try:
        return abs(float(clock_s) - CANFD_CLOCK_S) <= 1e-9
    except (TypeError, ValueError):
        return False


def canfd_window_s(points_a, points_b, clock_s: float = CANFD_CLOCK_S) -> float:
    """THE CANFD DRIFT MODEL's window (s) for two CANFD-clocked tracks of ANY length (see `CANFD_MODEL`).

        W = CANFD_SKEW_S + (E_max - E_min) + 2 x CANFD_RHO_PPM x max(L_a, L_b),  L = (N - 1) x clock_s

    Each arm stands e in [E_min, E_max] behind its OWN stream clock and the two streams start within
    CANFD_SKEW_S of each other, so the two arms' nominal times differ by at most S + (E_max - E_min); RHO bounds
    any growth of e with elapsed time, for every length. Rounded UP (to 0.1 ms): it is a bound."""
    n = max(int(points_a), int(points_b))
    if n < 1:
        raise ValueError(f"no points ({points_a}, {points_b})")
    length_s = (n - 1) * float(clock_s)
    w = (CANFD_SKEW_S + (CANFD_E_MS[1] - CANFD_E_MS[0]) / 1000.0
         + 2.0 * CANFD_RHO_PPM * 1e-6 * length_s)
    return math.ceil(w * 1e4 - 1e-9) / 1e4


def clock_floor_s(points_a, points_b, clock_a=None, clock_b=None, a=None, b=None) -> tuple:
    """(floor_s, clock_model): the window the MEASURED clock of these two tracks permits.

    'canfd_drift' when BOTH tracks are CANFD-clocked (5.0 ms) — `canfd_window_s`, any length; 'upload_player'
    when NEITHER is — `clock_envelope_window_s` (raises outside its 588..4281-point span, as it always has; a
    missing clock is the upload clock, which is what every row before 2026-10-02 was measured on). A MIXED pair
    raises: no model covers two arms streamed at two clocks, and the CANFD driver refuses a track whose clock is
    not its own (clock_mismatch), so such a pair cannot run on CANFD at all.
    """
    ca = TRACK_CLOCK_S if clock_a is None else clock_a
    cb = TRACK_CLOCK_S if clock_b is None else clock_b
    ia, ib = is_canfd_clock(ca), is_canfd_clock(cb)
    if ia and ib:
        return canfd_window_s(points_a, points_b), "canfd_drift"
    if ia or ib:
        raise ValueError(f"mixed clocks ({ca} s and {cb} s): no clock model covers a pair whose tracks "
                         "are timed at two different clocks")
    return clock_envelope_window_s(points_a, points_b, a, b), "upload_player"


def pair_dev_window_s(a: dict, b: dict):
    """H21's ORIGINAL per-pair term, unchanged, in seconds.

        dev(phi) = [(A_a - median A_a) + (T_a - median T_a) phi]
                 - [(A_b - median A_b) + (T_b - median T_b) phi],  phi in [0, 1]

    over every observed a x b run combination — the FULL range, not a
    percentile. dev is linear in phi, so the envelope is attained at
    phi in {0, 1}. `None` when either side has no run on its current bytes.

    It is kept because it is a direct measurement of THIS pair, and because a
    pair whose own samples somehow exceed the pooled envelope must not be
    quietly narrowed to it. It is no longer the answer on its own: see
    `drift_window_s`.
    """
    if not (a.get("start_lag_s") and b.get("start_lag_s")
            and a.get("replay_s") and b.get("replay_s")):
        return None
    mAa, mTa = statistics.median(a["start_lag_s"]), statistics.median(a["replay_s"])
    mAb, mTb = statistics.median(b["start_lag_s"]), statistics.median(b["replay_s"])
    worst = 0.0
    for aa in a["start_lag_s"]:
        for ab in b["start_lag_s"]:
            for ta in a["replay_s"]:
                for tb in b["replay_s"]:
                    for phi in (0.0, 1.0):
                        dev = ((aa - mAa) + (ta - mTa) * phi) \
                            - ((ab - mAb) + (tb - mTb) * phi)
                        worst = max(worst, abs(dev))
    return round(worst, 4)


def drift_window_s(a: dict, b: dict, points_a=None, points_b=None, clock_a=None, clock_b=None):
    """The pair's drift window in seconds: TWO TERMS, and the answer is the max.

    1. `pair_dev_window_s` — H21's per-pair term, from this pair's own observed
       start lags and replay clocks, re-referenced to their medians.
    2. `clock_envelope_window_s` — the FLOOR (Newton, 2026-09-09). What the
       MEASURED clock envelope permits for two tracks of these point counts,
       length-aware, pooled over every complete run on file.

    WHY THE FLOOR EXISTS. Term 1 alone is only as wide as the runs that pair
    happens to have. Most tracks have a handful, so it badly understates the
    real spread: on row 10 (4281 x 4119 points) it reads 2.2493 s while the
    measured envelope permits 3.51 s — the 78.62 mm clearance published on
    2026-09-08 was swept over less than two thirds of the divergence the
    ledger's own clock allows. A window that only usually holds is not a gate,
    which is the same reason term 1 takes a full range rather than a
    percentile; the floor is that principle applied to the SAMPLE SIZE as well
    as to the spread.

    THE FLOOR IS ALSO WHY A PAIR CAN NO LONGER BE WINDOWLESS. Term 1 is `None`
    when a track has never been played on its current bytes (B15, row 3 leg B).
    The envelope does not need that track's own runs — it needs its POINT
    COUNT, which the bytes on disk give — so such a pair now gets a real
    measured window instead of H22's nominal +-1.6 s. That the bytes have never
    been played is still true and still recorded in `requires`; it is a
    statement about the track, not about the clock.

    Returns `None` only when neither term can be computed — no samples AND no
    usable point counts.

    CANFD-CLOCKED PAIRS (2026-10-02, `clock_a`/`clock_b` both 0.005): the floor is the CANFD DRIFT MODEL
    (`canfd_window_s`, any length) and term 1 is NOT used — `a`/`b` come from the upload ledger's run_slot
    records, whose start lags and replay clocks measure the upload PLAYER (seconds of program start ramp),
    not a host-timed stream that never re-times. Clocks omitted = the upload clock, as before.
    """
    canfd = is_canfd_clock(clock_a) and is_canfd_clock(clock_b)
    per_pair = None if canfd else pair_dev_window_s(a, b)
    floor = None
    if points_a and points_b:
        try:
            floor, _model = clock_floor_s(points_a, points_b, clock_a, clock_b, a, b)
        except ValueError:
            floor = None
    if floor is None:
        return per_pair
    return round(max(floor, per_pair or 0.0), 4)


def executed_deviation_deg(recs, name: str, sha: str):
    """H18's dilation radius for one task, in degrees, or None if unmeasured.

    `replay_check.dev_p99_deg` is the 99th percentile of the INFINITY-NORM
    distance from each captured frame to the nearest commanded point
    (`track.py:1300-1302` maxes over the joints before the argmin), so it
    already bounds EVERY joint of that frame — there is no per-joint breakdown
    in the ledger and this scalar applied to each joint is the honest reading of
    what was recorded.

    Only COMPLETE runs on the current bytes count. A cut run's deviation is
    measured against a path the arm stopped following: `bottomlid_right` reads
    94.36 deg over all its runs and 19.59 over the complete ones. The worst
    complete run is taken, not the median — the dilation is meant to cover the
    executed box, not its centre.
    """
    vals = [float((d.get("replay_check") or {}).get("dev_p99_deg"))
            for d in recs
            if d.get("name") == name and d.get("track_sha256") == sha
            and d.get("complete") is True
            and (d.get("replay_check") or {}).get("dev_p99_deg") is not None]
    return round(max(vals), 4) if vals else None


# ----------------------------------------------------------------- the schedule
def region_keys() -> dict:
    import csv
    out = {}
    with REGIONS.open() as fh:
        for row in csv.DictReader(l for l in fh if not l.startswith("#")):
            if (row.get("region_id") or "").strip().isdigit():
                out[int(row["region_id"])] = row["region_key"].strip()
    return out


def schedule_rows() -> list:
    """The schedule's DATA rows, numbered 0..16.

    H26: `concurrent_rows` in `dual_arm_flow.py` numbers by CSV FILE LINE, so
    its rows print as 78-94 and `--row 10` — the 2026-09-08 design's own
    prescription — selects nothing and exits 1, while `--row 88` silently
    works. The offset is not even constant: `schedule_csv_text` regenerates the
    comment header on every Studio save and its length tracks the region counts.
    `schedule_loader`, the CSV's own header and every design number are 0-based
    over the data rows, so that is what this returns.

    Each row: {"row", "raw", "legs", "serial", "compound"}. `legs[i]` holds the
    i-th task of each cell's chain: a LEADING ">" on col2 means col2 WAITS
    (serial, so the arms are never in contact and the row is not a collision
    question); a ">" INSIDE a cell means that arm chains two tasks, so the row
    is concurrent BETWEEN the arms and has two legs, each its own pair.
    """
    keys = region_keys()
    rows, n = [], 0
    for line in SCHEDULE.open():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("col1"):
            continue
        cells = [c.strip() for c in line.split(",")]
        serial = any(c.startswith(">") for c in cells[1:])
        seqs, compound = [], False
        for cell in cells:
            seq = []
            for item in cell.lstrip(">").split(">"):
                m = re.match(r"^(\d+)([RLC])$", item.strip())
                if not m:
                    continue
                if m.group(2) == "C":
                    compound = True
                    continue
                seq.append(({"R": "right", "L": "left"}[m.group(2)],
                            keys.get(int(m.group(1)), f"region{m.group(1)}")))
            if seq:
                seqs.append(seq)
        n_legs = max((len(s) for s in seqs), default=0)
        rows.append(dict(row=n, raw=line.rstrip(","),
                         legs=[[s[i] for s in seqs if i < len(s)]
                               for i in range(n_legs)],
                         serial=serial, compound=compound))
        n += 1
    return rows


def concurrent_pairs(tasks: dict) -> list:
    """Every (row, leg) that puts a RIGHT and a LEFT task in motion together.

    B10: `dual_arm_flow` reported `leg_pairs[0]` and dropped a chained row's
    second leg, so row 3's `side_right` + `toplid_left` had no verdict at all
    and the row still reported complete. Both legs are here.
    """
    out = []
    for r in schedule_rows():
        if r["serial"] or r["compound"]:
            continue
        for i, leg in enumerate(r["legs"]):
            pair = {}
            for side, key in leg:
                nm = f"{FIXTURE}_{key}_{side}"
                if nm in tasks:
                    pair[side] = tasks[nm]
            if set(pair) == {"right", "left"}:
                out.append(dict(row=r["row"], leg=i, row_label=r["raw"],
                                right=pair["right"], left=pair["left"]))
    return out


def rest_pose(side: str, tasks: dict) -> dict:
    """That arm's `move_to_rest` pose, in radians, from the source plans.

    Cross-checked against the ledger: `post_run.deg` on the parked flow reads
    [0.0, -81.9903, -26.0008, 97.993, 5.9989, 61.9998, 69.0013] on the right,
    which is this pose to 4 dp. Every plan on a side agrees -- within
    REST_POSE_TOL_DEG since 2026-10-02 (an OMPL-planned rest leg ends inside
    MoveIt's joint-goal tolerance) -- so a larger disagreement is a caller
    error, not a choice: it raises.
    """
    poses, count = {}, {}
    for name, t in tasks.items():
        if t["side"] != side:
            continue
        plan = json.loads((REPO / t["plan"]).read_text())
        running, found = {}, None
        for st in plan.get("sub_trajectories", []):
            names = st.get("joint_names") or []
            for wp in st.get("waypoints") or []:
                pos = wp.get("positions") or []
                for i, nm in enumerate(names):
                    if i < len(pos):
                        running[nm] = float(pos[i])
            if st.get("stage_name") == "move_to_rest":
                found = {j: running[j] for j in arm_joint_names(side)
                         if j in running}
        if found:
            key = tuple(round(v, 4) for v in found.values())
            poses[key] = found
            count[key] = count.get(key, 0) + 1
    if not poses:
        raise ValueError(f"{side}: no plan carries a move_to_rest stage; the idle table "
                         "needs the rest pose")
    # AGREE WITHIN A TOLERANCE, not to the 4th decimal (2026-10-02). A move_to_rest that the OMPL
    # rescue plans ends inside MoveIt's joint-goal tolerance (1e-4 rad), not on the pose: the
    # P18 + staging side_right (Resource/reports/2026-10-02_rulings_apply) ends 0.0057 deg off the
    # rest pose the other 12 right plans end on exactly, and that refused the whole idle table.
    # The tolerance is cleaning_tracks.side_rest_pose's own (0.01 deg); the pose returned is the one
    # most plans end on (the parked flow's post_run.deg above); a real disagreement still raises.
    ref = poses[max(count, key=lambda k: count[k])]
    spread = max(abs(math.degrees(p[j] - ref[j])) for p in poses.values() for j in ref)
    if spread > REST_POSE_TOL_DEG:
        raise ValueError(f"{side}: {len(poses)} distinct move_to_rest poses across the plans, "
                         f"{spread:.4f} deg apart (> {REST_POSE_TOL_DEG} deg); the idle table assumes one")
    return ref


# ------------------------------------------------------------------ the verdict
def _coarse(row: dict, kind: str) -> bool:
    """Was this figure computed on a grid coarser than `DT_S`?"""
    gdt = ((row.get("gates") or {}).get(kind) or {}).get("dt")
    try:
        return gdt is not None and float(gdt) > DT_S
    except (TypeError, ValueError):
        return True          # an unreadable grid is not a grid we may trust


def _window_floor_for(row: dict):
    """Today's clock-envelope floor for this row, from its own point counts.

    Self-contained on purpose: an executive calls `decide` on the JSON alone,
    so the check that a figure was swept over a WIDE ENOUGH window has to be
    computable from the row. Returns `None` when the counts are missing or
    outside the measured span, which makes `_narrow` fall back to "not known
    to be narrow" rather than inventing a refusal.

    The CLOCK comes from the row too (2026-10-02): each side block carries the
    `clock_s` its track was swept at, and two CANFD-clocked sides take the CANFD
    drift model (`clock_floor_s`). A side block with no `clock_s` was written
    before the CANFD model existed, on the upload model, and keeps it — which
    can only make the requirement WIDER, never certify a narrower sweep.
    """
    try:
        r, l = row["right"], row["left"]
        floor, _model = clock_floor_s(r["points"], l["points"], r.get("clock_s"), l.get("clock_s"))
        return floor
    except (KeyError, TypeError, ValueError, AttributeError):
        return None


def _window_need_str(row: dict) -> str:
    """The window this row must be swept over, for a message: the LARGER of the
    clock-envelope floor and the row's own `window_s`. `_narrow` trips on
    whichever binds, so the message has to name that one."""
    need = _window_floor_for(row)
    w = row.get("window_s")
    vals = [float(v) for v in (need, w) if isinstance(v, (int, float))]
    return f"{max(vals):.4g}" if vals else "unmeasured"


def _narrow(row: dict) -> bool:
    """Was this figure swept over a NARROWER window than today's model needs?

    `min_mm_window` is a minimum over |offset| <= the swept window, and that
    minimum is NON-INCREASING in the window: widening it can only add
    configurations. So a figure swept at a narrower window is an UPPER BOUND on
    the figure today's window requires — which refuses soundly when it is below
    the floor and certifies NOTHING when it is above it. Exactly the epistemic
    status of a coarse-dt figure, and handled the same way.

    This is what makes the 2026-09-09 floor safe to introduce without
    re-sweeping every row at once: rows carried from the 2026-09-08 sidecar
    keep their numbers and lose their certification, individually and visibly.
    """
    swept = row.get("swept_window_s")
    if swept is None:
        return False
    need = _window_floor_for(row)
    w = row.get("window_s")
    for v in (need, w):
        if v is not None:
            try:
                if float(swept) < float(v) - 1e-6:
                    return True
            except (TypeError, ValueError):
                return True
    return False


def _effective(row: dict, kind: str, key: str):
    """(value, ok) for one figure, where a recorded `gates` entry informs it.

    `decide` calls this for `GATE_KIND` and for nothing else. It is still
    written per-kind because `validate_sidecar` reads the diagnostics' blocks
    for internal consistency — a diagnostic that claims a pass while carrying a
    below-floor number is a lie about a measurement even when nothing gates on
    it.

    A figure may only be reportable at a coarse dt while its floor test ran at
    dt=0.02 — the fine grid decides, because dt is a correctness parameter here
    and a coarse sweep can step 109 mm between samples.

    THREE STATES, and the third is the one that matters:
      ok=False  a number BELOW the floor. Sound at any grid: a coarse grid
                samples a subset of the configurations, so its minimum can only
                be too LARGE, and a below-floor answer on it is a fortiori.
      ok=True   a number at or above the floor, on a grid at least as fine as
                `DT_S`. Only this certifies.
      ok=None   no number at all, OR an above-floor number on a COARSE grid.
                The module docstring has always said a coarse pass "certifies
                nothing"; before this, `_effective` returned it as ok=True and
                `--cache-only`/`--dt 0.1` could publish `run:"concurrent"` off a
                dt=0.1 sweep. It cannot now.

    The recorded `ge_floor` flag never overrules the NUMBER: a row that claims a
    pass while carrying a below-floor figure is treated as the figure says.
    """
    gate = (row.get("gates") or {}).get(kind) or {}
    value = row.get(key)
    # A narrow window and a coarse grid are the SAME epistemic defect: both
    # sample a subset of the configurations, so both can only report a minimum
    # that is too large. Below the floor they refuse soundly; above it they
    # certify nothing.
    coarse = _coarse(row, kind) or (kind == GATE_KIND and _narrow(row))
    if gate.get("ge_floor") is False:
        v = gate.get("min_mm")
        v = value if v is None else v
        return (None, None) if v is None else (v, False)
    if gate.get("ge_floor") is True:
        v = value if value is not None else gate.get("min_mm")
        if v is None:
            return None, None                    # a pass with no number
        if v < CLEARANCE_FLOOR_MM:
            return v, False                      # the number wins over the flag
        return (v, None) if coarse else (v, True)
    if value is None:
        return None, None
    if value < CLEARANCE_FLOOR_MM:
        return value, False
    return (value, None) if coarse else (value, True)


def _coarse_why(label: str, row: dict, kind: str, value) -> str:
    gdt = ((row.get("gates") or {}).get(kind) or {}).get("dt")
    why = []
    if _coarse(row, kind):
        why.append(f"on a dt={gdt} grid, not dt={DT_S}")
    if kind == GATE_KIND and _narrow(row):
        # Print the REQUIREMENT that was missed, not `window_s`. `_narrow` trips
        # on `swept < max(need, window_s)`, and when the clock-envelope floor is
        # the binding term the old message printed `window_s` — which could be
        # the same number as `swept`, so it read "1.6886 narrower than 1.6886"
        # and named neither the real bound nor the reason.
        why.append(f"over a +-{row.get('swept_window_s')} s window, narrower "
                   f"than the +-{_window_need_str(row)} s this pair's clock "
                   "envelope now requires (2026-09-09)")
    return (f"the {label} minimum is {value:.2f} mm "
            + " and ".join(why or ["on an unreadable grid"])
            + ": that samples a SUBSET of the configurations, so an "
              "above-floor answer on it certifies nothing (it can only be too "
              f"large). Re-run this phase at dt={DT_S} over the full window.")


def decide(row: dict) -> tuple:
    """(run, why) for one pair row — the gate, and it NEVER RAISES.

    An executive calls this per row, possibly from a worker thread, on a file it
    did not write. A row it cannot read must come back `refused` with the reason
    in `why`, not as a traceback that unwinds a running schedule; and the
    fallback of an unreadable row can only be the conservative verdict.
    """
    try:
        return _decide(row)
    except Exception as exc:                                    # noqa: BLE001
        return "refused", (f"this row could not be read as a verdict "
                           f"({type(exc).__name__}: {exc}) — a row that cannot "
                           "be parsed cannot clear an arm to move")


def ruling_for(row: dict):
    """Newton's 2026-09-08 class for this pair, or `None` if he did not name it.

    Returns `("serial"|"concurrent", label)`. Keyed on the two task names so a
    schedule renumbering cannot detach the ruling from its pair.
    """
    names = frozenset(str((row.get(s) or {}).get("name"))
                      for s in ("right", "left"))
    label = NEWTON_SERIAL_PAIRS.get(names)
    if label is not None:
        return "serial", label
    label = NEWTON_CONCURRENT_PAIRS.get(names)
    if label is not None:
        return "concurrent", label
    return None                    # not a pair Newton ruled on; the gate decides


def _window_phrase(row: dict, win) -> str:
    w = row.get("window_s")
    if w is None:
        sw = row.get("swept_window_s")
        where = f"+-{sw} s" if sw is not None else "the swept window"
        return (f"{win:.2f} mm over {where} — a NOMINAL window (H22's barrier "
                "release skew), because this pair has no measured one")
    if (row.get("window_terms") or {}).get("clock_model") == "canfd_drift":
        return (f"{win:.2f} mm over the +-{w:.3f} s CANFD drift window (the 2026-10-02 model, "
                "derived from single-arm hardware runs)")
    return f"{win:.2f} mm over the measured +-{w:.3f} s drift window"


def _decide(row: dict) -> tuple:
    """The ladder itself, from the row's numbers alone.

    ONE FIGURE GATES: `min_mm_window` (H21/H29). Bounded to the drift two
    free-running arms actually produce, it discriminates — rows 6 and 7 read
    0.00 mm, the others tens of mm. `min_mm_invariant` and `min_mm_dilated` are
    RECORDED DIAGNOSTICS and are read by nothing here; see `DIAGNOSTIC_NOTE`
    and the `gates_on: false` flag each carries. An earlier version of this
    function gated on both and refused or serialised every pair in the
    schedule, which is what a criterion that fails everything does.

    On top of the gate sits Newton's ruling (`NEWTON_SERIAL_PAIRS`), which is
    operational truth and not something this table may re-derive:

    1. A pair Newton ruled SERIAL is serial, whatever the gate says. Serial is
       the strictly more conservative execution, so no gate figure can
       contradict it and none is needed to reach it.
    2. Otherwise the windowed gate decides: below the floor -> serial (the two
       arms share the volume); at or above it on a dt<=DT_S grid ->
       concurrent; no number, or a number only on a coarser grid -> refused.
    3. A pair Newton ruled CONCURRENT whose gate says otherwise is a CONFLICT
       between the ruling and the bytes. It comes back `refused` with that
       said plainly — not silently serialised (which would re-derive the
       ruling away) and not silently run (which would ignore the measurement).
       Nothing in the current table reaches this branch.

    A null `window_s` does NOT refuse. Row 3 leg B has no measured window
    (`toplid_left` has zero completed runs on its on-disk bytes, B15) and its
    figure was swept over H22's nominal barrier skew instead; the caveat is
    recorded in `requires` and `skipped`, and per the ruling it is not used to
    refuse the pair.
    """
    ruled = ruling_for(row)
    win, win_ok = _effective(row, GATE_KIND, "min_mm_window")

    if ruled and ruled[0] == "serial":
        # THE RULING IS THE REASON, UNCONDITIONALLY — it does not depend on the
        # windowed figure, and it must not read as though it did.
        #
        # This used to bury "not by a computed refusal" inside the one branch
        # where the figure was fresh AND above the floor, so the attribution
        # vanished the moment the number moved. It did move: the 2026-09-10
        # clock-envelope refit widened row 11's required window by 1.6 ms, the
        # carried figure became under-swept, and the row started describing
        # itself by an under-swept measurement instead of by Newton's call. That
        # is the exact failure this test's docstring predicted — "the next
        # person would re-derive it away when the number moved".
        #
        # For a ruled-serial row the arms NEVER overlap, so the windowed
        # minimum is a hypothetical. It is reported as commentary, with its
        # caveats, after the reason.
        if win_ok is False:
            measured = f"the windowed gate agrees: {win:.2f} mm, below the floor"
        elif win_ok is True:
            measured = f"the windowed gate would have passed it at {win:.2f} mm"
        elif win is not None:
            measured = ("the windowed figure is not usable as a gate here — "
                        + _coarse_why("windowed", row, GATE_KIND, win)
                        + " That does not change this row: it never runs the two "
                          "arms together, so the windowed minimum is a "
                          "hypothetical about a configuration set it does not "
                          "enter.")
        else:
            measured = "the windowed gate has no figure for this pair"
        return "serial", (f"Newton's ruling, {NEWTON_RULING_DATE}: "
                          f"{ruled[1]} runs SERIAL, one arm at a time — this row "
                          f"is serialised by the RULING, not by a computed "
                          f"refusal; {measured}")

    if win_ok is None:
        detail = (_coarse_why("windowed", row, GATE_KIND, win)
                  if win is not None else
                  "the windowed sweep — the ONE gating figure — was not "
                  "computed for this pair")
        if ruled:
            return "refused", (
                f"Newton ruled this pair CONCURRENT on {NEWTON_RULING_DATE}, "
                f"but {detail} A ruling this table cannot check is not a "
                "clearance to release two arms together; re-run `recheck "
                f"--phases {GATE_KIND}`.")
        return "refused", detail
    if win_ok is False:
        if ruled:
            return "refused", (
                f"Newton ruled this pair CONCURRENT on {NEWTON_RULING_DATE}, "
                f"but the gate now measures {win:.2f} mm over the swept "
                f"window, below the {CLEARANCE_FLOOR_MM:.0f} mm floor. The "
                "ruling and the bytes DISAGREE and this table will not "
                "resolve that on its own — take it back to Newton.")
        return "serial", ("the gate is below the "
                          f"{CLEARANCE_FLOOR_MM:.0f} mm floor: "
                          + _window_phrase(row, win))

    why = (f"the gate is at or above the {CLEARANCE_FLOOR_MM:.0f} mm floor: "
           + _window_phrase(row, win))
    if ruled:
        why = (f"Newton's ruling, {NEWTON_RULING_DATE}: this pair is "
               f"CONCURRENT, and {why}")
    return "concurrent", why + (
        ". Concurrency is conditional on " + "; ".join(CONCURRENCY_REQUIRES))


# ---------------------------------------------------------------- validation
_PAIR_KEYS = ("row", "leg", "row_label", "right", "left", "window_s",
              "min_mm_aligned", "min_mm_window", "min_mm_invariant",
              "min_mm_dilated", "closest_pair", "run", "why",
              # additive, and required since Newton's 2026-09-08 ruling
              "ruling", "requires")
_SIDE_KEYS = ("name", "slot", "points", "track_sha256", "pole_m", "hand",
              "track_s")
_IDLE_KEYS = ("working_task", "working_side", "idle_side", "idle_pole_m",
              "idle_pose", "min_mm", "ok")
_TOP_KEYS = ("generated", "dt_s", "link_set", "clearance_floor_mm", "pairs",
             "idle", "skipped", "gate_policy")


def validate_sidecar(doc: dict) -> list:
    """Every way the sidecar departs from the shared contract. [] == clean.

    Shape only, plus the one semantic rule the executive cannot re-derive: the
    written `run` must be the one `decide` produces from the written numbers,
    so a hand-edited verdict cannot outlive its evidence.
    """
    problems = []
    for k in _TOP_KEYS:
        if k not in doc:
            problems.append(f"missing top-level key {k!r}")
    if doc.get("dt_s") is not None and float(doc["dt_s"]) > DT_S:
        problems.append(f"dt_s {doc['dt_s']} is coarser than {DT_S}")
    # Newton, 2026-09-09: "there is already existing dual arms collision checking
    # in the alix studio, which will take much less time provided you only check
    # the arm links". `slim` IS that set (Studio's own 10/11 links). It drops the
    # fingers, pads and tips, so it is an UPPER bound in general — but an upper
    # bound that is TIGHT exactly where the binding contact is arm-to-arm, since
    # the links that would have lowered the figure are the ones it omits and
    # they are not the ones in contact. So the rule is not "slim is banned": it
    # is that slim certifies a pair only when THAT pair's closest contact lies
    # inside the slim set. A pair whose minimum is hand-to-hand (row 11's is
    # ILH_left_thumb_3 <-> IRH_right_little_1) is NOT certified by slim and says so.
    ds = doc.get("link_set")
    if ds not in (None, "slim", "full"):
        problems.append(f"link_set {ds!r} is neither 'slim' nor 'full'")
    elif ds == "slim":
        # WITHDRAWN 2026-09-09, and this is why. The first version of this rule
        # let `slim` certify a pair when its `closest_pair` was arm-to-arm — but
        # that closest pair CAME FROM THE SLIM SWEEP, which carries no hand
        # links and therefore can never report one. Circular: it asked the set
        # that cannot see hands whether hands mattered, and always heard no.
        # Newton found it by asking for the hand screen on row 10, where the
        # hands read 48.90 mm against the arms' 78.62 mm — the slim answer was
        # 29.7 mm too optimistic and this rule would have certified it.
        #
        # The sound test needs an INDEPENDENT hand measurement per pair. A pair
        # certified on slim must carry a `hands` block, and the gate is the
        # MINIMUM of the two, not the arm figure.
        for p_ in doc.get("pairs") or []:
            if p_.get("run") != "concurrent":
                continue
            tag = f"row {p_.get('row')} leg {p_.get('leg')}"
            h = p_.get("hands") or {}
            hm = h.get("min_mm_window")
            if hm is None:
                problems.append(
                    f"{tag}: certified on the 'slim' ARM set with no `hands` block. "
                    "slim carries no hand links, so it cannot see a hand-to-hand "
                    "contact closer than the arms — on row 10 that contact is "
                    "48.90 mm against the arms' 78.62 mm. Sweep this pair at "
                    "link_set 'hands' before it certifies.")
                continue
            arms = p_.get("min_mm_window")
            binding = min(v for v in (arms, hm) if v is not None)
            if p_.get("binding_min_mm") is None:
                problems.append(f"{tag}: no `binding_min_mm` — the gate is the MINIMUM "
                                "of the arm and hand figures, and it must be written down")
            elif abs(float(p_["binding_min_mm"]) - binding) > 1e-6:
                problems.append(f"{tag}: binding_min_mm {p_['binding_min_mm']} is not "
                                f"min(arms {arms}, hands {hm}) = {binding:.2f}")
            elif binding < CLEARANCE_FLOOR_MM:
                problems.append(f"{tag}: binding minimum {binding:.2f} mm is below the "
                                f"{CLEARANCE_FLOOR_MM:.0f} mm floor")
    if doc.get("clearance_floor_mm") not in (None, CLEARANCE_FLOOR_MM):
        problems.append(f"clearance_floor_mm {doc['clearance_floor_mm']} "
                        f"!= {CLEARANCE_FLOOR_MM}")
    gp = doc.get("gate_policy")
    if gp is not None:
        if not isinstance(gp, dict):
            problems.append("gate_policy is not an object")
        else:
            if gp.get("gate") != "min_mm_window":
                problems.append(
                    f"gate_policy.gate is {gp.get('gate')!r}; the gate is "
                    "'min_mm_window' and `decide` reads nothing else")
            for key in ("min_mm_invariant", "min_mm_dilated"):
                if key not in (gp.get("not_gates") or {}):
                    problems.append(
                        f"gate_policy.not_gates does not name {key} — a "
                        "diagnostic the policy does not disclaim is a "
                        "diagnostic the next reader will gate on")
            if not gp.get("ruling"):
                problems.append("gate_policy carries no `ruling` — Newton's "
                                f"{NEWTON_RULING_DATE} serial list is what "
                                "`run` is set from")
    for p in doc.get("pairs", []):
        tag = f"row {p.get('row')} leg {p.get('leg')}"
        for k in _PAIR_KEYS:
            if k not in p:
                problems.append(f"{tag}: missing pair key {k!r}")
        for side in ("right", "left"):
            blk = p.get(side)
            if not isinstance(blk, dict):
                problems.append(f"{tag}: {side} block is not an object")
                continue
            for k in _SIDE_KEYS:
                if k not in blk:
                    problems.append(f"{tag}: {side} missing {k!r}")
            if blk.get("hand") is not None and len(blk["hand"]) != 6:
                problems.append(f"{tag}: {side} hand is not six counts")
        for k in ("min_mm_aligned", "min_mm_window", "min_mm_invariant",
                  "min_mm_dilated", "window_s"):
            v = p.get(k)
            if v is not None and (not isinstance(v, (int, float))
                                  or isinstance(v, bool) or math.isnan(v)):
                problems.append(f"{tag}: {k} is neither null nor a number")
        if p.get("run") not in (None, "concurrent", "serial", "refused"):
            problems.append(f"{tag}: run {p.get('run')!r} is not one of "
                            "concurrent/serial/refused")
        # The `gates` block is what `_effective` reads, so an internally
        # inconsistent one is a way to certify a below-floor number by hand.
        for kind, key in (("window", "min_mm_window"),
                          ("invariant", "min_mm_invariant"),
                          ("dilated", "min_mm_dilated")):
            g = (p.get("gates") or {}).get(kind)
            if g is None:
                continue
            if not isinstance(g, dict) or set(g) < {"dt", "ge_floor", "min_mm"}:
                problems.append(f"{tag}: gates.{kind} is not "
                                "{dt, ge_floor, min_mm}")
                continue
            gv, v = g.get("min_mm"), p.get(key)
            if g.get("ge_floor") is True:
                for label, num in (("gates.%s.min_mm" % kind, gv), (key, v)):
                    if num is not None and num < CLEARANCE_FLOOR_MM:
                        problems.append(
                            f"{tag}: gates.{kind} claims ge_floor but {label} "
                            f"is {num} mm, below {CLEARANCE_FLOOR_MM}")
            if g.get("ge_floor") is False and gv is None and v is None:
                problems.append(f"{tag}: gates.{kind} claims below-floor with "
                                "no number anywhere")
            if g.get("ge_floor") is None and (gv is not None or v is not None):
                problems.append(f"{tag}: gates.{kind} says the figure was "
                                f"never computed but {key} is {v!r}")
            # After Newton's 2026-09-08 ruling exactly ONE figure gates. A
            # diagnostic that claims otherwise is how the invariant became a
            # criterion that refused every pair in the schedule.
            want = (kind == GATE_KIND)
            if "gates_on" in g and bool(g["gates_on"]) is not want:
                problems.append(
                    f"{tag}: gates.{kind}.gates_on is {g['gates_on']!r}; the "
                    f"only gating figure is {GATE_KIND!r} "
                    f"(gate_policy.gate = min_mm_window)")
            if kind in DIAGNOSTIC_KINDS and "gates_on" not in g:
                problems.append(
                    f"{tag}: gates.{kind} carries no `gates_on: false` — a "
                    "diagnostic with no flag reads as a criterion")
        # Only the GATING figure has to be on a fine grid to certify. Requiring
        # it of the diagnostics would make them gate again by the back door.
        if p.get("run") == "concurrent" and _coarse(p, GATE_KIND):
            problems.append(
                f"{tag}: run is 'concurrent' on a "
                f"dt={((p.get('gates') or {}).get(GATE_KIND) or {}).get('dt')}"
                f" {GATE_KIND} figure — only dt<={DT_S} certifies")
        if p.get("run") == "concurrent" and _narrow(p):
            problems.append(
                f"{tag}: run is 'concurrent' on a figure swept over "
                f"+-{p.get('swept_window_s')} s while this pair needs "
                f"+-{_window_need_str(p)} s. A minimum over a window cannot rise "
                "when the window grows, so the narrower sweep is an upper "
                "bound and certifies nothing — re-run `recheck --phases "
                f"{GATE_KIND}` for this pair.")
        if p.get("run") == "concurrent":
            missing = [r for r in CONCURRENCY_REQUIRES
                       if r not in (p.get("requires") or [])]
            if missing:
                problems.append(
                    f"{tag}: run is 'concurrent' but `requires` does not name "
                    + "; ".join(missing)
                    + " — concurrency is conditional on the partner-stop being "
                      "proven and nothing here proves it")
        ruled = ruling_for(p) if isinstance(p.get("right"), dict) \
            and isinstance(p.get("left"), dict) else None
        if "ruling" in p and not isinstance(p.get("ruling"), dict):
            problems.append(f"{tag}: `ruling` is not an object")
        elif "ruling" in p and "run" not in (p.get("ruling") or {}):
            problems.append(f"{tag}: `ruling` carries no `run`")
        elif ruled and not (p.get("ruling") or {}).get("run"):
            problems.append(
                f"{tag}: no `ruling` block — Newton ruled on this pair on "
                f"{NEWTON_RULING_DATE} and a row that does not carry the "
                "ruling invites the next generation to re-derive it away")
        elif ruled and (p.get("ruling") or {}).get("run") != ruled[0]:
            problems.append(
                f"{tag}: `ruling.run` is {(p.get('ruling') or {}).get('run')!r} "
                f"but Newton's {NEWTON_RULING_DATE} ruling makes this pair "
                f"{ruled[0]!r}")
        elif ruled is None and (p.get("ruling") or {}).get("run") is not None:
            problems.append(
                f"{tag}: `ruling.run` claims "
                f"{(p.get('ruling') or {}).get('run')!r} for a pair Newton "
                f"never ruled on. A ruling may not be INFERRED for a pair that "
                "was not in the schedule he saw.")
        if all(k in p for k in _PAIR_KEYS):
            run, why = decide(p)
            if p["run"] != run:
                problems.append(
                    f"{tag}: written run {p['run']!r} but the written numbers "
                    f"decide {run!r} — {why}")
    seen = {}
    for e in doc.get("idle", []):
        for k in _IDLE_KEYS:
            if k not in e:
                problems.append(f"idle {e.get('working_task')}: missing {k!r}")
        where = f"idle {e.get('working_task')} @ {e.get('idle_pole_m')}"
        key = (e.get("working_task"), e.get("idle_pole_m"))
        if key in seen:
            problems.append(f"{where}: two rows for the same task and pole — "
                            "an executive looking one up gets whichever it "
                            "reaches first")
        seen[key] = True
        if e.get("min_mm") is None:
            # B8's whole point: the parked partner is UNPROVEN until swept, and
            # an unswept combination must never read as clear.
            if e.get("ok") is not False:
                problems.append(f"{where}: min_mm is null but ok is "
                                f"{e.get('ok')!r} — a combination nobody swept "
                                "is not a safe one")
        elif e.get("ok") is not None:
            if bool(e["ok"]) != (e["min_mm"] >= CLEARANCE_FLOOR_MM):
                problems.append(f"{where}: ok disagrees with min_mm against "
                                "the floor")
    return problems


def verify_pair_row(p: dict, tasks: dict) -> list:
    """Every reason ONE pair row no longer describes the bytes on disk. [] == current.

    The per-row half of `verify_sidecar`, factored out unchanged (2026-09-23) so
    an executive can ask it of the one row it is about to dispatch ([N-12],
    `certify_pair`) instead of re-deriving it. `tasks` is `emitted_tasks(man)`.
    """
    problems = []
    tag = f"row {p.get('row')} leg {p.get('leg')}"
    for side in ("right", "left"):
        blk = p.get(side) or {}
        name = blk.get("name")
        t = tasks.get(name)
        if t is None:
            problems.append(f"{tag}: {side} task {name!r} is not EMITTED "
                            "in the manifest")
            continue
        path = REPO / t["track"]
        if not path.exists():
            problems.append(f"{tag}: {side} track {t['track']} is missing")
            continue
        live, why = _sha_or_reason(path)
        if live is None:
            problems.append(f"{tag}: the {side} track {t['track']} could "
                            f"not be read ({why}), so this row could not "
                            "be checked against its bytes")
            continue
        if blk.get("track_sha256") != live:
            problems.append(
                f"{tag}: the {side} bytes this verdict was measured on are "
                f"no longer there — {t['track']} hashes {live[:12]} and the "
                f"row records {str(blk.get('track_sha256'))[:12]}; re-run "
                "`recheck`")
        pre = t.get("pre_stroke") or {}
        if blk.get("pole_m") != pre.get("pole_m"):
            problems.append(f"{tag}: {side} pole_m {blk.get('pole_m')} is "
                            f"not the manifest's {pre.get('pole_m')} (B7)")
        if pre.get("hand_counts") and list(blk.get("hand") or []) != \
                list(pre["hand_counts"]):
            problems.append(f"{tag}: {side} hand counts differ from the "
                            "manifest (B7)")
        if blk.get("slot") != t.get("slot"):
            problems.append(f"{tag}: {side} slot {blk.get('slot')} is not "
                            f"the manifest's {t.get('slot')}")
        # THE CLOCK CHOSE THE WINDOW MODEL (2026-10-02). The bytes' sha does not cover the sidecar that
        # declares the clock, so a row swept on the CANFD model must not survive its track being re-declared
        # at another clock (or the reverse). Rows written before the field existed carry none: not checked.
        if "clock_s" in blk:
            try:
                live_clock = track_clock_s(t)
                if abs(float(blk["clock_s"]) - float(live_clock)) > 1e-9:
                    problems.append(f"{tag}: the {side} track now declares clock_s {live_clock} and this row "
                                    f"was measured at {blk['clock_s']} — its window model no longer applies; "
                                    "re-run `recheck`")
            except Exception as exc:                            # noqa: BLE001
                problems.append(f"{tag}: the {side} track's clock could not be read ({exc}), so the "
                                "window model this row was measured on could not be checked")
    return problems


def verify_sidecar(doc: dict, man=None) -> list:
    """Every reason the sidecar no longer describes the bytes on disk.

    This is the gate `verdict_for` never had. H29: today's literals carry 0
    staleness and 0 DETECTION — a re-emit would keep a clearance measured on
    different bytes, and B15 is that class made real (the controller held 2218
    points where the file holds 1891, and nothing compared them).
    """
    problems = list(validate_sidecar(doc))
    # This is a GATE an executive calls before it releases two arms, possibly
    # from a worker thread. Every input it reads is a file on disk that can be
    # missing, truncated or mid-write, and a traceback out of here unwinds a
    # running schedule. An unreadable input is a REASON THE TABLE CANNOT BE
    # TRUSTED, which is exactly what this function returns.
    try:
        tasks = emitted_tasks(man or load_manifest())
    except Exception as exc:                                    # noqa: BLE001
        problems.append(
            f"the manifest could not be read ({type(exc).__name__}: {exc}), so "
            "nothing in this table could be checked against the bytes on disk")
        return problems

    # B10, at the gate: a candidate with NO row is a pair with no verdict, and
    # `verdict_for`'s ancestor read a missing key as "nothing to check". A task
    # newly EMITTED into the manifest creates a candidate the table has never
    # seen, so coverage is checked against the schedule, not against the file.
    have = {(p.get("row"), p.get("leg")) for p in doc.get("pairs", [])}
    try:
        candidates = concurrent_pairs(tasks)
    except Exception as exc:                                    # noqa: BLE001
        candidates = []
        problems.append(
            f"the schedule could not be read ({type(exc).__name__}: {exc}), so "
            "this table's COVERAGE could not be checked — a pair the schedule "
            "runs together may have no row here at all (B10)")
    for cand in candidates:
        if (cand["row"], cand["leg"]) not in have:
            problems.append(
                f"row {cand['row']} leg {cand['leg']} "
                f"({cand['right']['name']} || {cand['left']['name']}) puts both "
                "arms in motion together and has NO row in this table; run "
                "`recheck`")
    if len(have) != len(doc.get("pairs", [])):
        problems.append("two pair rows share a (row, leg) — a lookup returns "
                        "whichever comes first")

    for p in doc.get("pairs", []):
        problems += verify_pair_row(p, tasks)

    # ---- the idle table is pinned to the bytes too (H27/H29).
    # The pair rows carried a sha from the start; the idle rows did not, so a
    # re-emitted track silently kept its parked-arm verdict. B8 is the finding
    # that says the parked partner is a real collision question, and a verdict
    # about bytes that are gone answers it about nothing.
    idle_rows = doc.get("idle") or []
    covered = {}
    for e in idle_rows:
        name = e.get("working_task")
        where = f"idle {name} @ {e.get('idle_pole_m')} m"
        t = tasks.get(name)
        if t is None:
            problems.append(f"{where}: working task {name!r} is not EMITTED "
                            "in the manifest")
            continue
        covered.setdefault(name, set()).add(e.get("idle_pole_m"))
        pre = t.get("pre_stroke") or {}
        if "work_pole_m" in e and e.get("work_pole_m") != pre.get("pole_m"):
            problems.append(f"{where}: the working arm was swept at pole "
                            f"{e.get('work_pole_m')} m, the manifest now "
                            f"stages {pre.get('pole_m')} m (B7)")
        if e.get("work_hand") is not None and pre.get("hand_counts") and \
                list(e["work_hand"]) != list(pre["hand_counts"]):
            problems.append(f"{where}: the working arm's hand counts differ "
                            "from the manifest (B7)")
        recorded = e.get("work_track_sha256")
        if recorded is None:
            problems.append(f"{where}: no work_track_sha256 — this row is "
                            "pinned to nothing and a re-emit cannot "
                            "invalidate it (H27)")
            continue
        path = REPO / t["track"]
        if not path.exists():
            problems.append(f"{where}: track {t['track']} is missing")
            continue
        live, why = _sha_or_reason(path)
        if live is None:
            problems.append(f"{where}: track {t['track']} could not be read "
                            f"({why}), so this row could not be checked "
                            "against its bytes")
            continue
        if recorded != live:
            problems.append(
                f"{where}: measured on bytes that are no longer there — "
                f"{t['track']} hashes {live[:12]}, the row records "
                f"{str(recorded)[:12]}; re-run `recheck --phases idle`")
    if idle_rows:
        for name in sorted(tasks):
            missing = sorted(set(IDLE_POLE_M) - covered.get(name, set()))
            if missing:
                problems.append(
                    f"idle: {name} has no parked-partner verdict at "
                    f"{missing} m — B8 is unanswered for that height and an "
                    "executive must not park there")
    return problems


# ------------------------------------------------- the executive's question
#: [N-12], Newton, closed 2026-09-23: this table is THE SOURCE OF TRUTH for
#: whether two tasks may run TOGETHER, read by EVERY executive and fail-closed.
#: A paired row runs concurrently only when its verdict says so for the bytes on
#: disk now, a `serial` verdict runs serially, and a missing or stale verdict
#: REFUSES the row. The schedule CSV keeps saying what is SCHEDULED, not what is
#: safe: row 6 is written `10R,3L` (paired) and its verdict is `serial`.
#:
#: `load_sidecar` and `pair_row` are the reader `dual_arm_flow` has always used,
#: moved here unchanged so the ROS lane reads the table through the same code;
#: `certify_pair` is the one call an executive makes before it releases two arms.
_FIRST_CELL_RE = re.compile(r"^>?(\d+)([RL])$", re.IGNORECASE)


def sidecar_path(fixture: str = FIXTURE, repo=None) -> pathlib.Path:
    """Where a fixture's verdict table lives: beside the MANIFEST whose track
    bytes its shas pin (`Resource/tracks/<fixture>/PAIR_VERDICTS.json`)."""
    return pathlib.Path(repo or REPO) / "Resource/tracks" / str(fixture) / "PAIR_VERDICTS.json"


def load_sidecar(path=None) -> dict:
    """The pair table, or a document that says why there is none.

    A missing or malformed sidecar is not "no constraint" — it is "no verdict",
    and a pair with no verdict may not run concurrently. The failure is carried
    in ``_problem`` rather than raised, because this is read from a gate that a
    refusal must survive.
    """
    p = pathlib.Path(path or SIDECAR)
    try:
        doc = json.loads(p.read_text())
    except Exception as exc:                                           # noqa: BLE001
        return {"pairs": [], "idle": [], "_problem":
                f"{p.name} could not be read ({type(exc).__name__}: {exc}) — no pair "
                f"in this schedule has a verdict, so none may run concurrently"}
    if not isinstance(doc, dict) or not isinstance(doc.get("pairs"), list):
        return {"pairs": [], "idle": [], "_problem":
                f"{p.name} does not carry a 'pairs' list — it is not a verdict table"}
    doc.setdefault("idle", [])
    return doc


def pair_row(doc: dict, right_name: str, left_name: str):
    """The sidecar row for a (right, left) task-name pair, or None."""
    want = {str(right_name), str(left_name)}
    for row in doc.get("pairs", []):
        got = {str((row.get(s) or {}).get("name")) for s in ("right", "left")}
        if got == want:
            return row
    return None


def serial_order(row) -> list:
    """The order a pair runs in when it runs ONE ARM AT A TIME.

    `dual_arm_flow._type_step`'s rule for a paired row — the arm of the row's
    FIRST cell goes first (`10R,3L` and `3R,10L` -> right, `6L,6R` -> left) —
    read from the row's own `row_label`, the schedule text it was measured for.
    Right first when the label names no single first arm.
    """
    first = str((row or {}).get("row_label") or "").split(",")[0].strip()
    m = _FIRST_CELL_RE.match(first)
    if m is None:
        return ["right", "left"]
    lead = "right" if m.group(2).upper() == "R" else "left"
    return [lead, "left" if lead == "right" else "right"]


def _display(path) -> str:
    p = pathlib.Path(path)
    try:
        return str(p.resolve().relative_to(REPO.resolve()))
    except ValueError:
        return str(p)


def certify_pair(right_name: str, left_name: str, *, sidecar=None, doc=None,
                 man=None, row=None, leg: int = 0, plans=None,
                 track_sha=None) -> dict:
    """May these two tasks run TOGETHER now? [N-12]'s one question, fail-closed.

    Every executive that can release both arms at once asks this before it does,
    and it NEVER RAISES — an input it cannot read is a refusal naming the reason.

      right_name, left_name  the manifest task each arm runs
                             (`commode_c_bottomlid_right`)
      sidecar / doc          the table (default: `SIDECAR`)
      man                    the manifest its shas pin (default: the table's
                             sibling MANIFEST.json)
      row, leg               the schedule position being dispatched; None for a
                             pair that is not a schedule row (a direct plan pair)
      plans                  {side: plan path} the executive will run: each must
                             be the plan the manifest EMITTED that task's track
                             from, byte for byte (its `plan_sha256`)
      track_sha              {side: sha256} of the track bytes the executive will
                             play: each must be the bytes the verdict pinned

    Returns ``{"run", "why", "kind", "problems", "verdict", "row_label",
    "order", "requires", "sidecar"}`` with ``run`` one of

      concurrent  the verdict says so and every identity it records matches. It
                  is CONDITIONAL on ``requires`` (G4/D1): an executive that
                  cannot meet them runs the pair serially;
      serial      the verdict says so (Newton's 2026-09-08 ruling, or the gate
                  below the floor): the two run one at a time, in ``order``;
      refused     ``kind`` says why — "missing" (no table, or no row pairs these
                  two tasks), "table" (the table fails its own contract or
                  could not be evaluated), "stale" (the row no longer describes
                  what would run: ``problems`` lists every mismatch), or
                  "verdict" (`decide` itself refuses the row).

    The identity is the one the table records, per side: the track's sha256,
    slot, pole height and hand counts against the manifest and the file on
    disk (`verify_pair_row`, the rule `verify_sidecar` applies to every row),
    plus the row's own (row, leg) — the schedule position `verify_sidecar`
    keys its coverage on — and, for an executive that runs PLANS, the manifest's
    link from each track to the plan it was emitted from.
    """
    path = pathlib.Path(sidecar or SIDECAR)
    where = _display(path)
    out = {"run": "refused", "why": "", "kind": None, "problems": [],
           "verdict": None, "row_label": None, "order": ["right", "left"],
           "requires": [], "sidecar": str(path)}

    def refuse(kind: str, why: str, problems=()) -> dict:
        out.update(run="refused", kind=kind, why=why, problems=list(problems))
        return out

    try:
        doc = doc if doc is not None else load_sidecar(path)
        if doc.get("_problem"):
            return refuse("missing", f"no verdict table — {doc['_problem']} ({where})")
        shape = validate_sidecar(doc)
        if shape:
            return refuse("table", f"{where} fails its own contract ({shape[0]}"
                          + (f"; +{len(shape) - 1} more" if len(shape) > 1 else "")
                          + ") — a table that does not validate clears no pair", shape)
        v = pair_row(doc, right_name, left_name)
        if v is None:
            return refuse("missing", f"no verdict — {where} has no row pairing "
                          f"{right_name} (right) with {left_name} (left); an unmeasured "
                          "pair may not run together (`tools/dual_arm_verdicts.py "
                          "recheck` measures it)")
        out.update(verdict=v, row_label=v.get("row_label"), order=serial_order(v),
                   requires=list(v.get("requires") or []))
        tag = f"row {v.get('row')} leg {v.get('leg')} `{v.get('row_label')}`"
        if man is None:
            man = json.loads((path.parent / "MANIFEST.json").read_text())
        tasks = emitted_tasks(man)
        stale = []
        if str((v.get("right") or {}).get("name")) != str(right_name):
            stale.append(f"{tag}: the verdict has {(v.get('right') or {}).get('name')} "
                         f"on the RIGHT arm and this dispatch puts {right_name} there — "
                         "the arms are swapped against what was measured")
        stale += verify_pair_row(v, tasks)
        if row is not None and (v.get("row"), v.get("leg")) != (row, leg):
            stale.append(f"{tag}: measured for schedule row {v.get('row')} leg "
                         f"{v.get('leg')}, and this dispatch is row {row} leg {leg} — "
                         "the table was generated against another schedule; re-run "
                         "`recheck`")
        for side, plan in sorted((plans or {}).items()):
            name = right_name if side == "right" else left_name
            t = tasks.get(name)
            if t is None:
                continue                          # verify_pair_row has said so
            src = t.get("plan")
            if not src or pathlib.Path(src).name != pathlib.Path(plan).name:
                stale.append(f"{tag}: the {side} arm would run "
                             f"{pathlib.Path(plan).name}, and the manifest emitted "
                             f"{name}'s track (the bytes this verdict measured) from "
                             f"{src} — the verdict does not describe that plan; run "
                             "the manifest's plan, or emit this one and `recheck`")
                continue
            live, why = _sha_or_reason(plan)
            if live is None:
                stale.append(f"{tag}: the {side} plan {plan} could not be read ({why})")
            elif live != t.get("plan_sha256"):
                stale.append(f"{tag}: the {side} plan {pathlib.Path(plan).name} hashes "
                             f"{live[:12]} and the manifest emitted the track from "
                             f"{str(t.get('plan_sha256'))[:12]} — the plan changed "
                             "after the bytes this verdict measured; re-emit, then "
                             "`recheck`")
        for side, sha in sorted((track_sha or {}).items()):
            pinned = (v.get(side) or {}).get("track_sha256")
            if str(sha) != str(pinned):
                stale.append(f"{tag}: the {side} arm would play track bytes "
                             f"{str(sha)[:12]}, and the verdict was measured on "
                             f"{str(pinned)[:12]}; `recheck` measures the bytes that "
                             "would play")
        if stale:
            first = re.sub(r"^row \S+ leg \S+( `[^`]*`)?: ", "", stale[0])
            return refuse("stale", f"STALE verdict — {where} {tag} no longer describes "
                          f"what would run: {first}"
                          + (f" (+{len(stale) - 1} more)" if len(stale) > 1 else ""),
                          stale)
        run, why = decide(v)
        out.update(run=run, kind="verdict", why=f"{where} {tag}: {why}")
        return out
    except Exception as exc:                                    # noqa: BLE001
        return refuse("table", f"the verdict could not be evaluated "
                      f"({type(exc).__name__}: {exc}) — a certificate that cannot be "
                      f"read clears no pair ({where})")


# ------------------------------------------------------------- the generation
#: `hands` is the certifying companion to `window`: the same windowed sweep on
#: the 31-link-per-side HAND set, written to the row's `hands` block rather
#: than over the arms figure. `verify` refuses a concurrent pair that carries a
#: `slim` arms figure without one, so a shard that recomputes `window` must
#: recompute `hands` with it — a widened window cannot carry the old block.
PHASES = ("window", "hands", "invariant", "dilated", "idle")


def _side_block(task: dict, stats: dict, canfd_runs: int = 0) -> dict:
    """B7: the device state each side was swept AT, carried into the row.

    Row 3's leg boundary moves the two poles 75 mm in OPPOSITE directions. Swept
    as staged the leg reads 39.07 mm; with neither pole moved 8.49 mm, with only
    the right 8.58, with only the left 8.58 — three of three mis-staged
    configurations are contact class on `L_Link3 <-> R_Link2`, two bare upper
    arms. A verdict that does not say which pole heights it assumed cannot be
    checked against the live device state, and the precedent is in the ledger:
    `2026-09-03/run_slot72_211316.json` ran with the pole 165 mm off plan.
    """
    pre = task.get("pre_stroke") or {}
    return {
        "name": task["name"], "slot": task["slot"], "points": task["points"],
        "track_sha256": task.get("track_sha256"),
        "pole_m": pre.get("pole_m"),
        "hand": list(pre.get("hand_counts") or []) or None,
        "hand_state": pre.get("hand_state"),
        "track_s": task.get("track_s"),
        # THE CLOCK the window model was chosen by (2026-10-02): 0.005 = CANFD drift model, else the upload
        # player's envelope. Carried so `decide` can re-derive the requirement from the row alone.
        "clock_s": track_clock_s(task),
        "runs_on_current_bytes": stats["n_runs_current_bytes"],
        "runs_on_any_bytes": stats["n_runs_any_bytes"],
        # B15'S BYTE QUESTION COUNTS BOTH CHANNELS (Newton 2026-10-02, one set of bytes): the completed HARDWARE
        # CANFD runs of these bytes (`canfd_completed_runs`). `runs_on_current_bytes` stays the UPLOAD count
        # (the per-pair window term measures the player); `_nominal_window_caveat` reads the two together.
        "canfd_runs_on_current_bytes": int(canfd_runs or 0),
    }


def _gate(value, dt, kind=None):
    """One `gates` entry.

    `gates_on` is the whole point of the block after Newton's 2026-09-08
    ruling: exactly one figure decides `run`, and the other two are recorded
    diagnostics that must not be mistaken for criteria by a reader of the JSON.
    """
    if value is None:
        out = {"dt": dt, "ge_floor": None, "min_mm": None}
    else:
        ok = value >= CLEARANCE_FLOOR_MM
        out = {"dt": dt, "ge_floor": ok,
               "min_mm": None if ok else round(value, 2)}
    if kind is not None:
        out["gates_on"] = (kind == GATE_KIND)
        if kind in DIAGNOSTIC_NOTE:
            out["note"] = DIAGNOSTIC_NOTE[kind]
    return out


def dilation_offsets(left_tl, right_tl, at: dict, delta_l_deg, delta_r_deg,
                     probe_deg: float = 0.5) -> dict:
    """H18's per-joint offsets: the constant perturbation that CLOSES the gap.

    The literal reading of "dilate and re-sweep" is 2^14 sign combinations
    against every sample pair — days of FCL. What is affordable and still sound
    AS A REFUSAL is a witness: at the configuration where the commanded sweep is
    thinnest, push each joint to +delta or -delta by the sign of a central
    difference on the FCL distance, apply that constant offset to the WHOLE
    track on both sides, and re-run the ordinary windowed sweep. Both resulting
    trajectories lie inside the measured deviation envelope, so a below-floor
    answer is a real refusal; it is an UPPER bound on the true dilated minimum,
    so an above-floor answer certifies nothing, and the sidecar says so.
    """
    from alix_workspace.dual_arm_plan_collision import _sample_at
    ql = _sample_at(left_tl[0], left_tl[1], at["left_s"])
    qr = _sample_at(right_tl[0], right_tl[1], at["right_s"])
    base = dict(ql)
    base.update(qr)
    eps = math.radians(probe_deg)
    out = {}
    for side, delta in (("left", delta_l_deg), ("right", delta_r_deg)):
        offs = [0.0] * 7
        if delta:
            d = math.radians(delta)
            for k, jn in enumerate(arm_joint_names(side)):
                if jn not in base:
                    continue
                up, dn = dict(base), dict(base)
                up[jn] += eps
                dn[jn] -= eps
                grad = pose_distance_mm(up) - pose_distance_mm(dn)
                offs[k] = -d if grad > 0 else (d if grad < 0 else 0.0)
        out[side] = offs
    return out


def compute_pair(cand, recs, cache_dir, phases, dt=DT_S, verbose=True,
                 cache_only=False, canfd_recs=None) -> tuple:
    """One `pairs` row of the shared contract, plus the reasons for any nulls.

    `recs` are the UPLOAD ledger's records (`_ledger_records`: timing); `canfd_recs` the hardware CANFD
    records (`_canfd_ledger_records`, read here when not given) — B15's byte question only."""
    skipped = []
    R, L = cand["right"], cand["left"]
    tag = f"r{cand['row']}l{cand['leg']}"
    sR = replay_stats(recs, R["name"], R["track_sha256"])
    sL = replay_stats(recs, L["name"], L["track_sha256"])
    if canfd_recs is None:
        canfd_recs = _canfd_ledger_records()
    cR = canfd_completed_runs(canfd_recs, R["name"], R["track_sha256"])
    cL = canfd_completed_runs(canfd_recs, L["name"], L["track_sha256"])
    # THE CLOCK DECIDES THE MODEL (2026-10-02): two CANFD-clocked (5.0 ms) tracks take the CANFD drift model,
    # any length, and H21's per-pair term is not used (its run_slot records measure the upload player);
    # 5.5545 ms tracks keep the upload player's envelope and the per-pair term, exactly as before.
    clk_l, clk_r = track_clock_s(L), track_clock_s(R)
    canfd = is_canfd_clock(clk_l) and is_canfd_clock(clk_r)
    pair_w = None if canfd else pair_dev_window_s(sL, sR)
    clock_model = None
    try:
        floor_w, clock_model = clock_floor_s(L["points"], R["points"], clk_l, clk_r, sL, sR)
    except ValueError as exc:                                   # noqa: BLE001
        floor_w = None
        skipped.append(f"{tag}: no clock-model floor — {exc}")
    window_s = drift_window_s(sL, sR, L["points"], R["points"], clk_l, clk_r)
    sweep_w = window_s if window_s is not None else BARRIER_SKEW_S
    if canfd and window_s is not None:
        skipped.append(
            f"{tag}: the window is the CANFD DRIFT MODEL ({window_s} s for {L['points']} x "
            f"{R['points']} points at 5.0 ms; Newton 2026-10-02 \"Fit it from the CANFD runs\"): "
            f"release skew {CANFD_SKEW_S} s + covering phase error {CANFD_E_MS[0]:.0f}..{CANFD_E_MS[1]:.0f} ms "
            f"+ {CANFD_RHO_PPM:.0f} ppm of the longer track, from {CANFD_MODEL_RUNS} complete single-arm "
            "CANFD hardware runs (`canfd_drift_model`). It is DERIVED for two arms — the corpus has no two-arm "
            "CANFD run — and holds only if both arms stream on CANFD.")
    if window_s is None:
        side = "left" if sL["n_runs_current_bytes"] == 0 else "right"
        skipped.append(
            f"{tag}: window_s is NULL — the {side} track has "
            f"{sL['n_runs_current_bytes'] if side == 'left' else sR['n_runs_current_bytes']}"
            f" completed runs on its CURRENT bytes (H27/B15: unmeasured, not "
            f"safe) AND no clock-envelope floor could be read. The geometry is "
            f"still swept, at H22's +-{BARRIER_SKEW_S} s barrier release skew, "
            "so min_mm_window is a real number over a NOMINAL window rather "
            f"than an observed one. Per Newton's {NEWTON_RULING_DATE} ruling "
            "that is RECORDED — see the row's `requires` — and is not used to "
            "refuse the pair.")
    elif pair_w is None and not canfd:
        side = "left" if sL["n_runs_current_bytes"] == 0 else "right"
        skipped.append(
            f"{tag}: this pair has NO per-pair window term — the {side} track "
            f"has 0 completed runs on its CURRENT bytes (H27/B15). Its "
            f"+-{window_s} s window is the CLOCK-ENVELOPE FLOOR alone, which "
            "needs the track's point count and not its runs, so the figure is "
            "measured rather than nominal. That the bytes have never been "
            "played is a separate open fact and stays in `requires`.")
    elif floor_w is not None and floor_w > (pair_w or 0.0) and not canfd:
        skipped.append(
            f"{tag}: the window is the CLOCK-ENVELOPE FLOOR ({floor_w} s), not "
            f"this pair's own samples ({pair_w} s over "
            f"{sL['n_runs_current_bytes']} left x {sR['n_runs_current_bytes']} "
            "right runs). Newton, 2026-09-09: the measured player clock spans "
            "5.5-6.7 ms/pt and a handful of runs does not sample it.")

    ltl = build_timeline(L)
    rtl = build_timeline(R)
    base_key = {"left_sha": L["track_sha256"], "right_sha": R["track_sha256"],
                "left_pole": (L.get("pre_stroke") or {}).get("pole_m"),
                "right_pole": (R.get("pre_stroke") or {}).get("pole_m"),
                "left_hand": list((L.get("pre_stroke") or {}).get("hand_counts") or []),
                "right_hand": list((R.get("pre_stroke") or {}).get("hand_counts") or []),
                "link_set": LINK_SET}

    def run(kind, window, ltl_, rtl_, extra=None, link_set=None):
        """The finest ACCEPTED grid this pair has an answer on.

        A cached figure at `dt` or finer wins outright. A cached figure that is
        only COARSER than `dt` does not stop the finer sweep from running —
        it is a fallback, and `cache_only` is what says "take the fallback and
        do not start an hour of FCL".
        """
        # THE LINK SET IS PART OF THE ANSWER'S IDENTITY. A `hands` figure and a
        # `slim` figure of the same pair, window and grid are different numbers,
        # so the override has to reach the cache key or the second sweep would
        # be served the first one's result.
        if link_set:
            extra = dict(extra or {}, link_set=link_set)
        coarse = {}
        for grid in sorted(set(ACCEPT_DT) | {dt}):
            key = dict(base_key, kind=kind, window_s=window, dt=grid)
            key.update(extra or {})
            hit = cached(cache_dir, key, lambda: None)
            if hit is None:
                continue
            if grid <= dt:
                return hit, grid          # already have it at least this fine
            coarse[grid] = hit
        if cache_only:
            if coarse:
                g = min(coarse)
                return coarse[g], g
            return None, None
        key = dict(base_key, kind=kind, window_s=window, dt=dt)
        key.update(extra or {})

        def go():
            t0 = time.monotonic()
            if verbose:
                print(f"    {tag} {kind}: sweeping window={window} dt={dt}",
                      flush=True)
            res = sweep(ltl_, rtl_, dt=dt, window_s=window, link_set=link_set)
            res["_wall_s"] = round(time.monotonic() - t0, 1)
            if verbose:
                print(f"    {tag} {kind}: {res['min_mm']:.2f} mm "
                      f"({res['_wall_s']} s)", flush=True)
            return res
        return cached(cache_dir, key, go), dt

    win, win_dt = run("window", sweep_w, ltl, rtl) if "window" in phases \
        else (None, None)
    # THE HANDS PHASE (added 2026-09-10). `verify` refuses a concurrent pair
    # certified on the `slim` ARM set with no hands block, and until now nothing
    # in this tool could produce one: the block was written by an ad-hoc script,
    # so the only way to keep it was `_carry_phases`, and a widened window makes
    # carrying unsound. The result was a certification that depended on a number
    # no command could regenerate — it went stale on rows 3:0, 3:1, 10:0 and 11
    # the moment the clock-envelope refit moved their windows.
    #
    # It is a PHASE and not a `--link-set` value because it does not replace the
    # arms figure: `binding_min_mm = min(arms, hands)` needs both, and the doc's
    # own shape check requires `link_set` to stay 'slim' or 'full'. On row 10 the
    # hands are what bind — 48.90 mm against the arms' 78.62 mm.
    hnd, hnd_dt = run("hands", sweep_w, ltl, rtl, link_set="hands") \
        if "hands" in phases else (None, None)
    if "hands" in phases and hnd is None:
        skipped.append(f"{tag}: the HANDS sweep at +-{sweep_w} s was not computed, so "
                       "this pair cannot certify on a slim arms figure alone")
    inv, inv_dt = run("invariant", None, ltl, rtl) if "invariant" in phases \
        else (None, None)
    if "window" in phases and win is None:
        skipped.append(f"{tag}: windowed sweep at +-{sweep_w} s not computed")
    if "invariant" in phases and inv is None:
        skipped.append(f"{tag}: offset-INVARIANT sweep not computed. It is a "
                       "recorded DIAGNOSTIC and gates nothing, so its absence "
                       "does not change `run`.")
    for kind, grid in (("window", win_dt), ("invariant", inv_dt)):
        if grid is not None and grid > DT_S:
            skipped.append(
                f"{tag}: the {kind} minimum is a dt={grid} figure, not dt="
                f"{DT_S} — a coarser grid samples a SUBSET of the "
                "configurations, so the number can only be too large. It "
                "refuses soundly and certifies nothing."
                + ("" if kind == GATE_KIND else
                   " (a diagnostic; it gates nothing either way)"))

    dev_r = executed_deviation_deg(recs, R["name"], R["track_sha256"])
    dev_l = executed_deviation_deg(recs, L["name"], L["track_sha256"])
    dil, dil_dt = None, None
    if "dilated" in phases:
        if win is None:
            skipped.append(f"{tag}: dilation needs the windowed sweep's own "
                           "thinnest configuration; not computed")
        elif dev_r is None or dev_l is None:
            side = "right" if dev_r is None else "left"
            skipped.append(f"{tag}: dilated sweep skipped — no completed run "
                           f"with a replay_check on the {side} track's CURRENT "
                           "bytes (H27: unmeasured, not safe)")
        else:
            # The signs depend on WHERE the commanded sweep was thinnest, so
            # `at` is part of their identity: a re-swept window moves the
            # critical configuration and the old directions no longer apply.
            sign_key = dict(base_key, kind="dilation_signs", at=win["at"],
                            dev_l=dev_l, dev_r=dev_r)
            offs = cached(cache_dir, sign_key,
                          (lambda: None) if cache_only else
                          (lambda: dilation_offsets(ltl, rtl, win["at"],
                                                    dev_l, dev_r)))
            if offs is None:
                skipped.append(f"{tag}: dilation directions not computed")
            else:
                dil, dil_dt = run(
                    "dilated", sweep_w,
                    build_timeline(L, offsets_rad=offs["left"]),
                    build_timeline(R, offsets_rad=offs["right"]),
                    extra={"dev_l": dev_l, "dev_r": dev_r,
                           "off_l": [round(x, 6) for x in offs["left"]],
                           "off_r": [round(x, 6) for x in offs["right"]]})
                if dil is None:
                    skipped.append(f"{tag}: dilated sweep not computed")
                elif dil_dt > DT_S:
                    skipped.append(
                        f"{tag}: the dilated minimum is a dt={dil_dt} figure, "
                        f"not dt={DT_S}")

    def mm(res, key="min_mm"):
        return None if res is None else round(float(res[key]), 2)

    aligned_src = win or inv
    # WHERE each minimum was found, verbatim from the sweep that found it. The
    # 2026-09-08 review flagged rows 10 and 11 as reporting their INVARIANT
    # minimum at equal left/right times (4.18/4.18, 4.80/4.80) while the
    # aligned sweep on the same grid read 79.87 and 42.03 mm — which one
    # `query(i, i)` cannot do. Those two figures are the sweeps' `aligned_at`,
    # not their `at`: the cached results hold `at` = 1.32/20.58 s (row 10) and
    # 2.26/12.68 s (row 11), both far off the diagonal, exactly as an
    # off-diagonal minimum must be. There is no defect in the sweep and none in
    # the ladder; there was no `at` in the sidecar at all, which is what let the
    # two be confused. Both are written out now, per figure and labelled.
    at = {}
    for kind, res in (("aligned", aligned_src), ("window", win),
                      ("invariant", inv), ("dilated", dil)):
        if res is None:
            at[kind] = None
        elif kind == "aligned":
            at[kind] = res.get("aligned_at")
        else:
            at[kind] = res.get("at")
    row = {
        "row": cand["row"], "leg": cand["leg"], "row_label": cand["row_label"],
        "right": _side_block(R, sR, cR), "left": _side_block(L, sL, cL),
        "window_s": window_s,
        "min_mm_aligned": mm(aligned_src, "aligned_min_mm"),
        "min_mm_window": mm(win),
        "min_mm_invariant": mm(inv),
        "min_mm_dilated": mm(dil),
        "closest_pair": closest_pair_label(win or inv),
        "run": None, "why": None,
        "gates": {"window": _gate(mm(win), win_dt, "window"),
                  "invariant": _gate(mm(inv), inv_dt, "invariant"),
                  "dilated": _gate(mm(dil), dil_dt, "dilated")},
        "swept_window_s": sweep_w,
        #: THE TWO TERMS OF `window_s`, kept apart so a reader can see which
        #: one decided and can tell a re-sampled pair from a re-modelled one.
        "window_terms": {
            "pair_dev_s": pair_w,
            #: the CLOCK MODEL's floor: the upload player's envelope for 5.5545 ms tracks, the CANFD drift
            #: model for 5.0 ms ones (`clock_model` says which; 2026-10-02)
            "clock_envelope_s": floor_w,
            "clock_model": clock_model,
            "governing": ("clock_envelope"
                          if floor_w is not None and floor_w >= (pair_w or 0.0)
                          else "pair_dev" if pair_w is not None else None),
            "model": (
                "window_s = the CANFD DRIFT MODEL (Newton, 2026-10-02): W = %.3f s release skew + "
                "(%.0f - %.0f) ms covering phase error + 2 x %.0f ppm x the longer track, fitted on %d "
                "complete single-arm CANFD hardware runs (see the sidecar's `canfd_drift_model`: runs, "
                "shas, fit, coverage, assumptions). H21's per-pair term is not used: the upload ledger's "
                "run_slot records measure the upload player, not a host-timed stream."
                % (CANFD_SKEW_S, CANFD_E_MS[1], CANFD_E_MS[0], CANFD_RHO_PPM, CANFD_MODEL_RUNS)
            ) if clock_model == "canfd_drift" else (
                "window_s = max(H21's per-pair dev envelope, the measured "
                "clock-envelope floor). The floor is length-aware: replay_s in "
                "[%.3f + %.4f ms/pt x (N-1), %.3f + %.4f ms/pt x (N-1)] over "
                "%d complete runs, start lag S(N) = %.4f + %.4f ms/pt x N with "
                "residuals in [%+.3f, %+.3f] s over %d runs from 2026-09-03. "
                "Newton, 2026-09-09: the clock is 5.5-6.7 ms/pt and one pair's "
                "few runs do not sample it."
                % (CLOCK_ENVELOPE_LO[0], CLOCK_ENVELOPE_LO[1] * 1e3,
                   CLOCK_ENVELOPE_HI[0], CLOCK_ENVELOPE_HI[1] * 1e3,
                   CLOCK_ENVELOPE_RUNS, START_LAG_FIT[0],
                   START_LAG_FIT[1] * 1e3, START_LAG_RESID_S[0],
                   START_LAG_RESID_S[1], START_LAG_RUNS)),
        },
        "dilate_deg": {"right": dev_r, "left": dev_l},
        "at": at,
        "ruling": None,
        "requires": [],
    }
    if hnd is not None:
        row["hands"] = {
            "link_set": "hands",
            "dt_s": hnd_dt,
            "window_s": window_s,
            "min_mm_aligned": mm(hnd, "aligned_min_mm"),
            "min_mm_window": mm(hnd),
            "closest_pair": closest_pair_label(hnd),
            "why": ("Newton, 2026-09-09: 'you can as well screen the hands for the "
                    "seat ring bottom only'. 31 hand links per side against 10-11 arm "
                    "links, so 961 pairs per query vs slim's 110 — the hand MESHES are "
                    "simplified, the link COUNT is what costs. It found the binding "
                    "contact on row 10: 48.90 mm hand-to-hand against 78.62 mm "
                    "arm-to-arm."),
        }
    ruled = ruling_for(row)
    row["ruling"] = {
        "run": ruled[0] if ruled else None,
        "date": NEWTON_RULING_DATE,
        "pair": ruled[1] if ruled else None,
        "source": (
            "Newton, 2026-09-08 late: three pairs run SERIAL — "
            + "; ".join(sorted(NEWTON_SERIAL_PAIRS.values()))
            + " — and the other three run CONCURRENT: "
            + "; ".join(sorted(NEWTON_CONCURRENT_PAIRS.values()))
            + ". \"Anything outside of this is not correct.\" This field "
              "exists so the ruling travels with the row and cannot be "
              "silently re-derived away by a future gate."),
    } if ruled else {
        "run": None, "date": NEWTON_RULING_DATE, "pair": None,
        "source": (f"Newton's {NEWTON_RULING_DATE} ruling names neither of "
                   "these tasks — this pair did not exist in the schedule he "
                   "ruled on — so it is decided by the windowed gate alone"),
    }
    _set_verdict(row)                    # sets `run`, `why` and `requires`
    return row, skipped


def _renormalise(row: dict) -> None:
    """Bring a row carried from an older sidecar onto today's contract.

    Numbers are never touched — they are what a sweep measured. Only the
    derived fields are restamped: the `gates_on`/`note` flags, the `ruling`
    block, `requires`, and the verdict itself, which older sidecars set from a
    ladder that gated on the offset-invariant diagnostic.
    """
    gates = row.setdefault("gates", {})
    for kind in ("window",) + tuple(DIAGNOSTIC_KINDS):
        g = gates.get(kind)
        if not isinstance(g, dict):
            continue
        g["gates_on"] = (kind == GATE_KIND)
        if kind in DIAGNOSTIC_NOTE:
            g["note"] = DIAGNOSTIC_NOTE[kind]
    # THE WINDOW IS A MODEL FIGURE AND THE MODEL MOVED (2026-09-09). A carried
    # row's `window_s` was computed by the model of the day it was swept; the
    # clock-envelope floor does not need that pair's runs, only its point
    # counts, so it can be re-derived here. The MINIMUM is left exactly as
    # measured — `swept_window_s` still says what it was measured over, and
    # `_narrow` is what turns the mismatch into a lost certification.
    need = _window_floor_for(row)
    if need is not None:
        old_w = row.get("window_s")
        if old_w is None or float(need) > float(old_w):
            row["window_s"] = need
            row.setdefault("window_terms", {}).update(
                clock_envelope_s=need,
                governing="clock_envelope",
                restamped=("2026-09-09: window_s re-derived from the measured "
                           "clock envelope for these two point counts. The "
                           "figures below it were swept over "
                           f"+-{row.get('swept_window_s')} s and are upper "
                           "bounds until re-swept."))
    ruled = ruling_for(row)
    row["ruling"] = {
        "run": ruled[0] if ruled else None,
        "date": NEWTON_RULING_DATE,
        "pair": ruled[1] if ruled else None,
        "source": (row.get("ruling") or {}).get("source") or GATE_POLICY["ruling"],
    }
    row.setdefault("requires", [])
    _set_verdict(row)


def _set_verdict(row: dict) -> None:
    """Set `run`/`why` from the numbers, and keep `requires` consistent with it.

    Called anywhere the numbers can change (a fresh compute, a carried phase).
    The caveats already in `requires` are kept; only the concurrency
    preconditions are added and removed with the verdict, so a row that stops
    being concurrent stops claiming they matter and one that becomes concurrent
    cannot arrive without them.
    """
    row["run"], row["why"] = decide(row)
    caveat = _nominal_window_caveat(row)
    req = [r for r in (row.get("requires") or [])
           if r not in CONCURRENCY_REQUIRES
           and not r.startswith(WINDOW_CAVEAT_PREFIXES)]
    if caveat:
        req.insert(0, caveat)
    if row["run"] == "concurrent":
        req.extend(CONCURRENCY_REQUIRES)
    row["requires"] = req


#: Every `requires` entry `_nominal_window_caveat` DERIVES, by prefix. `_set_verdict` drops all of them and
#: re-derives, so a recomputed row can never keep a stale one or stack two.
WINDOW_CAVEAT_PREFIXES = ("B15", "Drift window OPEN")


def _nominal_window_caveat(row: dict):
    """B15, as a precondition rather than a refusal — and a NOMINAL window, which is always OPEN.

    A pair one of whose tracks has never been played on its on-disk bytes has
    no PER-PAIR window term. Since 2026-09-09 that no longer leaves it without
    a window — the clock-envelope floor needs the track's point count, not its
    runs — but the fact stands and is a different one: nobody has seen those
    bytes play at all. Newton's ruling says record that and run the pair; this
    is where it is recorded, and it is derived from the row so a carried or
    hand-assembled row cannot lose it.

    A NULL `window_s` IS OPEN, NEVER SETTLED (Newton, 2026-10-02: "until it
    exists, rows on the nominal window are OPEN"). Until 2026-10-02 this branch
    wrote "B15 byte question settled" for exactly the rows whose window nobody
    had measured or modelled — rows 3:0 (toplid_right, 7198 points) and 11
    (top_right, 482) of the 10-02 re-plan, both outside the upload player's
    588..4281-point clock envelope and swept over H22's NOMINAL +-1.6 s skew —
    while `test_the_shipped_b15_caveat_agrees_with_the_ledger` expected OPEN.
    The entry now names the unplayed tasks (dual_arm_flow resolves a B15 entry
    to the tasks it NAMES and asks the ledger about each), and a nominal window
    whose bytes HAVE been played still says OPEN, under its own prefix, because
    there the window is the open question, not the bytes.
    """
    if row.get("min_mm_window") is None:
        return None
    # BOTH CHANNELS SETTLE THE BYTES (Newton 2026-10-02, one set of bytes): a side is unplayed when it has no
    # completed run on its on-disk bytes on EITHER channel — the upload count plus the completed hardware CANFD
    # runs (`canfd_runs_on_current_bytes`; a row written before the field existed carries none: 0).
    unplayed = [side for side in ("right", "left")
                if (row.get(side) or {}).get("runs_on_current_bytes") == 0
                and not int((row.get(side) or {}).get("canfd_runs_on_current_bytes") or 0)]
    names = ", ".join((row.get(s) or {}).get("name") or s for s in unplayed)
    canfd = (row.get("window_terms") or {}).get("clock_model") == "canfd_drift"
    if unplayed and row.get("window_s") is not None:
        return ("B15 byte question OPEN: %s has 0 completed runs on its "
                "on-disk bytes (upload or CANFD), so this pair contributes no per-pair window "
                "term and its +-%s s window is %s alone. That the %s covers "
                "the clock does not make these bytes played — one clean "
                "single-arm run settles it."
                % (names, row.get("window_s"),
                   "the CANFD DRIFT MODEL's (2026-10-02)" if canfd else
                   "the measured CLOCK-ENVELOPE FLOOR",
                   "model" if canfd else "envelope"))
    if row.get("window_s") is not None:
        return None
    nominal = ("this pair has NO measured drift window — no clock model covers "
               "both of its tracks and no per-pair term exists — so its gate "
               "figure was swept over the NOMINAL +-%s s barrier release skew "
               "(H22), not an observed or modelled envelope. A nominal window "
               "stays OPEN until a clock model or a per-pair term measures it "
               "(Newton, 2026-10-02): re-run `recheck` once one covers both "
               "tracks"
               % (row.get("swept_window_s", BARRIER_SKEW_S)))
    # A side whose run count is not on record is NOT a played side: only a count > 0 on both sides moves the
    # caveat off B15 (a hand-assembled or carried row may lack the field; the bytes question stays open).
    unknown = [side for side in ("right", "left")
               if not isinstance((row.get(side) or {}).get("runs_on_current_bytes"), int)]
    if unplayed or unknown:
        named = ", ".join((row.get(s) or {}).get("name") or s
                          for s in ("right", "left") if s in unplayed or s in unknown)
        return ("B15 byte question OPEN: %s has 0 completed runs on its "
                "on-disk bytes (upload or CANFD; or none on record), and %s." % (named, nominal))
    return "Drift window OPEN: %s." % nominal


def idle_hand_states(tasks, side: str) -> list:
    """Every hand state that side is COMMANDED to, in a deterministic order.

    The parked arm is left in whichever of these its last task staged, so the
    set — not any one member — is what the idle table has to cover.
    """
    seen = []
    for name in sorted(tasks):
        t = tasks[name]
        if t["side"] != side:
            continue
        h = list((t.get("pre_stroke") or {}).get("hand_counts") or [])
        if h and h not in seen:
            seen.append(h)
    return sorted(seen)


def _hand_tag(hand) -> str:
    """A JSON-object-key form of a hand state ("129-129-129-129-133-944")."""
    return "-".join(str(int(c)) for c in hand)


def _hand_of(states, tag):
    for h in states:
        if _hand_tag(h) == tag:
            return list(h)
    return None


def idle_table(tasks, cache_dir, dt=DT_S, only=None, verbose=True,
               cache_only=False) -> tuple:
    """B8: every task swept against the OTHER arm parked at `move_to_rest`.

    Newton's ruling makes SERIAL the normal class for a colliding pair, so the
    parked partner is not a footnote — it is the state the flow is in for most
    of the pass, and nothing has ever checked it. The prior sweep found 3 of 12
    task x idle-pole combinations below the floor at idle pole 0.075 m (worst
    7.71 mm, `L_Link1 <-> R_Link3`, on unpadded meshes) and 0 of 12 at 0.300 m.
    Since EVERY task commands 0.075 or 0.150 and none commands 0.300, the low,
    unsafe idle pole is the state the flow always leaves behind.

    THE PARKED HAND IS NOT ONE STATE. Three distinct hand states are commanded
    per side ([129]x4, [516]x4 and [750]x4 with their own thumb/spread), nothing
    re-opens the hand after a stroke, and which one the partner is parked in
    depends on which task it last ran — which the table cannot know. The first
    version of this function took `next(x for x in tasks.values() ...)`, i.e.
    whichever task happened to come first in manifest order, and said so
    nowhere; that is an arbitrary pick on a quantity the module's own docstring
    shows is worth 33 mm on row 11. Every commanded state is swept and the
    WORST is reported, with the per-state figures in `by_hand` for an executive
    that actually knows how the partner was left. A row missing any commanded
    state carries `min_mm: null, ok: false` — B8 is about a partner nobody
    checked, and a partly-checked one is not a checked one.
    """
    out, skipped = [], []
    rest = {s: rest_pose(s, tasks) for s in ("right", "left")}
    hands = {s: idle_hand_states(tasks, s) for s in ("right", "left")}
    for name in sorted(tasks):
        if only and name not in only:
            continue
        t = tasks[name]
        work_side = t["side"]
        idle_side = "left" if work_side == "right" else "right"
        wtl = build_timeline(t)
        for pole in IDLE_POLE_M:
            by_hand, pairs, missing = {}, {}, []
            for hand in hands[idle_side]:
                itl = hold_timeline(idle_side, rest[idle_side], pole, hand)
                ltl, rtl = ((itl, wtl) if idle_side == "left" else (wtl, itl))
                key = {"kind": "idle", "work": t["track_sha256"],
                       "work_side": work_side, "idle_side": idle_side,
                       "idle_pole": pole, "idle_hand": hand,
                       "dt": dt, "link_set": LINK_SET}

                def go(ltl=ltl, rtl=rtl, name=name, pole=pole, hand=hand):
                    if cache_only:
                        return None
                    if verbose:
                        print(f"    idle {name} vs {pole} m hand {hand[0]}",
                              flush=True)
                    return sweep(ltl, rtl, dt=dt, window_s=None,
                                 invariant=False)
                res = cached(cache_dir, key, go)
                if res is None:
                    missing.append(hand)
                    skipped.append(
                        f"idle: {name} x idle {idle_side} pole {pole} m hand "
                        f"{hand} not computed")
                    continue
                tag = _hand_tag(hand)
                by_hand[tag] = round(float(res["aligned_min_mm"]), 2)
                pairs[tag] = closest_pair_label(
                    {"min_pair": res.get("aligned_pair")})
            worst_tag = min(by_hand, key=by_hand.get) if by_hand else None
            complete = bool(by_hand) and not missing
            v = by_hand[worst_tag] if complete else None
            row = {
                "working_task": name, "working_side": work_side,
                "idle_side": idle_side, "idle_pole_m": pole,
                "idle_pose": "move_to_rest",
                "min_mm": v,
                "ok": bool(v is not None and v >= CLEARANCE_FLOOR_MM),
                "closest_pair": pairs.get(worst_tag, "unknown"),
                "work_track_sha256": t["track_sha256"],
                # B7 for the single-arm case: the working arm was swept with
                # ITS staged pole and hand, so a row that does not say which
                # cannot be checked against the live device either.
                "work_pole_m": (t.get("pre_stroke") or {}).get("pole_m"),
                "work_hand": list((t.get("pre_stroke") or {}).get(
                    "hand_counts") or []) or None,
                "idle_pose_deg": [round(math.degrees(rest[idle_side][j]), 4)
                                  for j in arm_joint_names(idle_side)],
                "idle_hand": (_hand_of(hands[idle_side], worst_tag)
                              if complete else None),
                "hand_states_swept": [_hand_of(hands[idle_side], k)
                                      for k in sorted(by_hand)],
                "hand_states_missing": missing,
                "by_hand": by_hand,
            }
            if not complete:
                skipped.append(
                    f"idle: {name} @ {pole} m reports NULL — "
                    f"{len(missing) or len(hands[idle_side])} of "
                    f"{len(hands[idle_side])} commanded parked hand states were "
                    "not swept, and the worst of a subset is not the worst")
            out.append(row)
    return out, skipped


def parked_summary(idle_rows, tasks) -> dict:
    """B8's answer: the lowest idle height that is clear for EVERY task.

    The design's prescription is "raise the idle pole to 0.300 m as the parked
    state". That is a literal; this is the same statement derived from the
    table, per side, so a re-emit that changes a stroke can change the answer
    instead of leaving a stale number in a docstring. `pole_m: null` means no
    swept height clears every task and the executive must go row by row.
    """
    out = {}
    for idle_side in ("right", "left"):
        rows = [r for r in idle_rows if r.get("idle_side") == idle_side]
        need = {n for n, t in (tasks or {}).items()
                if t["side"] != idle_side}
        best = None
        for pole in sorted({r.get("idle_pole_m") for r in rows
                            if r.get("idle_pole_m") is not None}):
            at = [r for r in rows if r.get("idle_pole_m") == pole]
            covered = {r["working_task"] for r in at}
            if need - covered:
                continue
            if all(r.get("ok") for r in at):
                worst = min(r["min_mm"] for r in at)
                best = {"pole_m": pole, "min_mm": round(worst, 2),
                        "tasks": len(at)}
                break
        out[idle_side] = best or {
            "pole_m": None, "min_mm": None, "tasks": 0,
            "why": "no swept idle height clears every task of the other arm"}
    return out


def _carryable(prev_row: dict, cand: dict) -> bool:
    """May a previous pair row stand in for one this run did not compute?

    Only if it describes the SAME BYTES in the SAME STAGED STATE. `track_sha256`
    is the whole point of the sidecar (H27/H29) and the pole and hand are B7:
    row 3's leg boundary moves the two poles 75 mm in opposite directions and
    three of three mis-staged configurations are contact class, so a clearance
    measured at another pole height is a clearance for another robot.
    """
    for side in ("right", "left"):
        old, new = (prev_row.get(side) or {}), cand[side]
        pre = new.get("pre_stroke") or {}
        if old.get("name") != new["name"]:
            return False
        if old.get("track_sha256") != new.get("track_sha256"):
            return False
        if old.get("pole_m") != pre.get("pole_m"):
            return False
        if list(old.get("hand") or []) != list(pre.get("hand_counts") or []):
            return False
    return True


def _set_binding(row: dict) -> None:
    """`binding_min_mm` = min(arm figure, hand figure) for a concurrent row.

    DERIVED, never stored by hand. `verify` gates on this number and used to be
    the ONLY code that mentioned it — so like `hands` and `idle_link_set` it was
    a field a human had put in the sidecar that no generator wrote, and the
    first shard to rebuild a row left it null (2026-09-10). Anything that can
    be computed from two numbers already in the row must be computed, or it
    drifts from them.

    A row with no hand figure gets NO binding number rather than the arm figure:
    `verify` refuses a slim-certified concurrent pair without a hands block, and
    quietly filling this in with the arm figure would hand it the answer it is
    supposed to withhold.
    """
    if row.get("run") != "concurrent":
        return
    arms = row.get("min_mm_window")
    hand_blk = row.get("hands") or {}
    hands = hand_blk.get("min_mm_window")
    if arms is None or hands is None:
        return
    # ALL THREE, from the same comparison. `binding_link_set` and
    # `binding_pair` are read straight into every run record by the executive
    # (`verdict_used.binding_*`), and they were the same defect this function
    # exists to fix one layer down: a shard rebuilt the row, wrote
    # `binding_min_mm`, and left the two fields that SAY WHICH SET AND WHICH
    # LINKS produced it null — so a record could carry 45.93 mm with nothing
    # to name the hand-to-hand contact it came from.
    hands_bind = float(hands) < float(arms)
    row["binding_min_mm"] = round(min(float(arms), float(hands)), 2)
    row["binding_link_set"] = "hands" if hands_bind else (LINK_SET or "slim")
    row["binding_pair"] = (hand_blk.get("closest_pair") if hands_bind
                           else row.get("closest_pair"))


def _carry_phases(row: dict, prev_row: dict, phases: set, stamp: str) -> list:
    """Fill the phases this run did not ask for from the previous sidecar.

    A shard (`--phases invariant`) must not blank the figures the other shards
    measured — but it must not silently inherit them either. Only a phase that
    was NOT requested is carried, only onto a null, and the row records where
    each carried number came from.
    """
    notes = []
    for kind, key in (("window", "min_mm_window"),
                      ("invariant", "min_mm_invariant"),
                      ("dilated", "min_mm_dilated")):
        if kind in phases or row.get(key) is not None:
            continue
        if prev_row.get(key) is None:
            continue
        row[key] = prev_row[key]
        g = dict((prev_row.get("gates") or {}).get(kind)
                 or _gate(prev_row[key], None, kind))
        # A block carried off an OLD sidecar predates `gates_on`, and a missing
        # flag on a diagnostic is exactly the mistake the flag exists to stop.
        g["gates_on"] = (kind == GATE_KIND)
        if kind in DIAGNOSTIC_NOTE:
            g["note"] = DIAGNOSTIC_NOTE[kind]
        row.setdefault("gates", {})[kind] = g
        row.setdefault("carried", {})[kind] = stamp
        notes.append(f"r{row['row']}l{row['leg']}: the {kind} figure "
                     f"({prev_row[key]} mm) was CARRIED from the sidecar "
                     f"generated {stamp} — this run did not request that phase "
                     "and the track bytes, pole and hand are unchanged")
    if row.get("min_mm_aligned") is None and prev_row.get("min_mm_aligned") \
            is not None:
        row["min_mm_aligned"] = prev_row["min_mm_aligned"]
        row.setdefault("carried", {})["aligned"] = stamp
    # THE HANDS BLOCK, which no phase can regenerate. `verify` REFUSES a
    # concurrent pair certified on `slim` without one, and nothing in this tool
    # can write one (there is no `--link-set hands`), so a shard that recomputed
    # a row used to drop the evidence and leave the row uncertifiable — twice on
    # 2026-09-10, the second time from a regeneration I thought was read-only.
    #
    # It is carried ONLY when the window it was measured at still matches the
    # row's window: the hands minimum is a windowed sweep, so a widened window
    # invalidates it. When it does not match, the row is left WITHOUT the block
    # and a note says why — `verify` then refuses, which is the correct outcome
    # and is exactly what caught the clock-envelope refit.
    prev_h = prev_row.get("hands") or {}
    if row.get("hands") is None and prev_h.get("min_mm_window") is not None:
        same_window = (row.get("window_s") is not None
                       and prev_h.get("window_s") is not None
                       and abs(float(row["window_s"]) - float(prev_h["window_s"])) <= 1e-9)
        if same_window:
            row["hands"] = dict(prev_h)
            row.setdefault("carried", {})["hands"] = stamp
            notes.append(f"r{row['row']}l{row['leg']}: the HANDS figure "
                         f"({prev_h['min_mm_window']} mm) was CARRIED from the sidecar "
                         f"generated {stamp} — same window {prev_h['window_s']} s, same "
                         "track bytes, pole and hand")
        else:
            notes.append(f"r{row['row']}l{row['leg']}: the hands block from {stamp} was "
                         f"measured at window {prev_h.get('window_s')} s but this row is "
                         f"swept at {row.get('window_s')} s, so it CANNOT be carried — a "
                         "widened window can only lower a windowed minimum. Re-sweep this "
                         "pair at link_set 'hands'; until then `verify` refuses it.")
    _set_binding(row)
    if row.get("carried"):
        _set_verdict(row)
    return notes


def build_sidecar(args) -> dict:
    man = load_manifest()
    tasks = emitted_tasks(man)
    recs = _ledger_records()
    canfd_recs = _canfd_ledger_records()       # B15's byte question only (one set of bytes, 2026-10-02)
    phases = set(args.phases)
    skipped = []
    if set(PHASES) - phases:
        skipped.append("phases NOT requested this run: "
                       + ", ".join(sorted(set(PHASES) - phases)))

    # ---- MERGE, never truncate.
    # `recheck` is shardable by design (`--only`, `--only-tasks`, `--phases`) —
    # the table is hours of FCL and sharding is the only way it gets re-run. It
    # also WROTE THE WHOLE FILE from whatever the shard computed, so
    # `recheck --only 10:0` replaced six pair rows and 72 idle rows with one
    # row, and `--phases invariant` nulled every window and refused the table.
    # A shard now merges onto what is already on disk, and carries a previous
    # figure only when the bytes and the staged state it was measured at are
    # still the ones the manifest names.
    prev, prev_stamp = {}, None
    if not getattr(args, "replace", False):
        p = pathlib.Path(args.sidecar)
        if p.exists():
            try:
                prev = json.loads(p.read_text())
                prev_stamp = prev.get("generated")
            except Exception as exc:                            # noqa: BLE001
                skipped.append(f"the sidecar already on disk could not be read "
                               f"({exc}); nothing was carried forward")
                prev = {}
    prev_pairs = {(r.get("row"), r.get("leg")): r
                  for r in (prev.get("pairs") or [])}
    prev_idle = {(r.get("working_task"), r.get("idle_pole_m")): r
                 for r in (prev.get("idle") or [])}

    pairs = []
    for cand in concurrent_pairs(tasks):
        sel = f"{cand['row']}:{cand['leg']}"
        pr = prev_pairs.get((cand["row"], cand["leg"]))
        if args.only and sel not in args.only:
            if pr is not None and _carryable(pr, cand):
                row = json.loads(json.dumps(pr))
                row.setdefault("carried", {})["row"] = prev_stamp
                # A row carried off an OLDER sidecar can carry a verdict from an
                # older ladder — pre-2026-09-08 rows were refused by the
                # offset-invariant diagnostic. The NUMBERS are what was
                # measured and are kept; the VERDICT is re-derived from them by
                # today's gate, and the ruling/flags are restamped.
                _renormalise(row)
                pairs.append(row)
                skipped.append(
                    f"r{cand['row']}l{cand['leg']}: not selected by --only; "
                    f"the whole row was CARRIED from the sidecar generated "
                    f"{prev_stamp} (same track bytes, pole and hand)")
                continue
            # Nothing carryable: assemble from cache if it is there, and let
            # the nulls and `decide` speak if it is not. Never drop the row —
            # a candidate missing from `pairs` is a pair with no verdict at all,
            # which is the B10 failure this table exists to end.
            row, sk = compute_pair(cand, recs, args.cache, phases, dt=args.dt,
                                   verbose=False, cache_only=True, canfd_recs=canfd_recs)
            pairs.append(row)
            skipped.extend(sk)
            skipped.append(
                f"r{cand['row']}l{cand['leg']}: not selected by --only and no "
                "usable previous row (absent, or measured on other bytes or "
                "another staged state) — assembled from cache only")
            continue
        if not args.quiet:
            print(f"  row {cand['row']} leg {cand['leg']} ({cand['row_label']}): "
                  f"{cand['right']['name']} || {cand['left']['name']}", flush=True)
        row, sk = compute_pair(cand, recs, args.cache, phases, dt=args.dt,
                               verbose=not args.quiet,
                               cache_only=args.cache_only, canfd_recs=canfd_recs)
        skipped.extend(sk)
        if pr is not None and _carryable(pr, cand):
            skipped.extend(_carry_phases(row, pr, phases, prev_stamp))
        pairs.append(row)

    idle, isk = [], []
    only_tasks = set(args.only_tasks or []) or None
    if "idle" in phases:
        idle, isk = idle_table(tasks, args.cache, dt=args.dt,
                               only=only_tasks,
                               verbose=not args.quiet,
                               cache_only=args.cache_only)
    if "idle" in phases and only_tasks:
        skipped.append("idle: only " + ", ".join(sorted(only_tasks))
                       + " was recomputed; every other row was carried from "
                         f"the sidecar generated {prev_stamp}")
    if "idle" not in phases:
        skipped.append("idle: table not requested this run; carried from the "
                       f"sidecar generated {prev_stamp}" if prev_idle else
                       "idle: table not requested this run and none was on "
                       "disk to carry (B8 unanswered)")
    # The table's provenance travels WITH the table: recomputed here -> this
    # pass's set and dt; carried -> whatever measured the rows we carried.
    if "idle" in phases and not only_tasks:
        idle_link_set, idle_dt_s = LINK_SET, args.dt
    else:
        idle_link_set = prev.get("idle_link_set")
        idle_dt_s = prev.get("idle_dt_s")
        if idle_link_set is None and prev_idle:
            skipped.append(
                "idle: the carried table records no link set — the sidecar it came "
                "from predates this field being written, so WHICH set produced the "
                "72 idle figures is unrecorded. Re-run with `--phases idle` at the "
                "set you want on record.")
    if "idle" not in phases or only_tasks:
        have = {(r["working_task"], r["idle_pole_m"]) for r in idle}
        for key, r in sorted(prev_idle.items(), key=lambda kv: str(kv[0])):
            if key in have:
                continue
            name = r.get("working_task")
            t = tasks.get(name)
            if t is None or r.get("work_track_sha256") != t.get("track_sha256"):
                skipped.append(
                    f"idle: the previous row for {name} @ "
                    f"{r.get('idle_pole_m')} m was DROPPED — it is pinned to "
                    "bytes or a task the manifest no longer names")
                continue
            r = dict(r)
            r.setdefault("carried", prev_stamp)
            idle.append(r)
        idle.sort(key=lambda r: (str(r.get("working_task")),
                                 r.get("idle_pole_m") or 0.0))
    skipped.extend(isk)

    # ---- what the two demoted figures would have done, stated FROM THE DATA
    # rather than asserted. If a future re-sweep ever makes one of them
    # discriminate, this note stops saying "every row" and somebody can look
    # again at whether it may gate.
    for kind, key in (("invariant", "min_mm_invariant"),
                      ("dilated", "min_mm_dilated")):
        vals = [p.get(key) for p in pairs if p.get(key) is not None]
        below = [v for v in vals if v < CLEARANCE_FLOOR_MM]
        if vals:
            skipped.append(
                f"the {kind} figure is RECORDED AND NOT GATED: {len(below)} of "
                f"{len(vals)} computed rows are below the "
                f"{CLEARANCE_FLOOR_MM:.0f} mm floor "
                f"(min {min(vals):.2f}, max {max(vals):.2f} mm)"
                + (". A criterion that fails every row cannot discriminate, "
                   "which is why it sets nothing."
                   if len(below) == len(vals) else
                   ". It separates SOME rows now — if that holds up it is "
                   "worth asking Newton whether it may gate again.")
                + " " + DIAGNOSTIC_NOTE[kind])
        else:
            skipped.append(f"the {kind} diagnostic was not computed for any "
                           "row this run; it gates nothing either way")

    # B8 stated from the table rather than from the design's literal.
    unsafe = [r for r in idle if r.get("min_mm") is not None and not r["ok"]]
    if unsafe:
        worst = min(unsafe, key=lambda r: r["min_mm"])
        by_pole = {}
        for r in unsafe:
            by_pole[r["idle_pole_m"]] = by_pole.get(r["idle_pole_m"], 0) + 1
        skipped.append(
            "B8: %d of %d task x idle-height combinations are below the "
            "%.0f mm floor against a PARKED partner (%s). The worst is %s vs "
            "an idle %s arm at %s m: %.2f mm on %s. The parked hand is part of "
            "this — every commanded hand state is swept and the WORST is "
            "reported, and the closing states are what put %s into contact "
            "class, which an open-hand-only sweep would have missed."
            % (len(unsafe), len(idle), CLEARANCE_FLOOR_MM,
               ", ".join(f"{n} at {p} m" for p, n in sorted(by_pole.items())),
               worst["working_task"], worst["idle_side"],
               worst["idle_pole_m"], worst["min_mm"],
               worst.get("closest_pair"), worst["working_task"]))

    passing = [p for p in pairs
               if p.get("min_mm_window") is not None
               and p["min_mm_window"] >= CLEARANCE_FLOOR_MM]
    if passing:
        thin = min(passing, key=lambda p: p["min_mm_window"])
        skipped.append(
            f"the THINNEST figure that passes the gate is row {thin['row']} "
            f"leg {thin['leg']} ({thin['row_label']}) at "
            f"{thin['min_mm_window']:.2f} mm on {thin.get('closest_pair')} — "
            f"{thin['min_mm_window'] - CLEARANCE_FLOOR_MM:.2f} mm of margin, on "
            "DECIMATED collision meshes with no MoveIt link padding. Where "
            f"that row's verdict is 'serial' it is Newton's "
            f"{NEWTON_RULING_DATE} ruling on exactly that thin margin, not a "
            "computed refusal, and this table does not dress it up as one.")

    for p in pairs:
        if p.get("window_s") is None and p.get("min_mm_window") is not None:
            skipped.append(
                f"r{p['row']}l{p['leg']}: its {p['min_mm_window']:.2f} mm gate "
                f"figure rests on a NOMINAL window — H22's "
                f"+-{p.get('swept_window_s')} s barrier release skew, NOT a "
                "measured drift envelope: one side has no completed run on its "
                "current bytes to measure one from (B15/H27). The figure is "
                "real geometry over a window nobody has observed. Per Newton's "
                "ruling that caveat is RECORDED (see the row's `requires`), "
                "not used to refuse the pair.")

    skipped.append(
        "row 3 is NOT reported as a concatenated chain. `_concat_timelines` "
        "(dual_arm_plan_collision.py:214-231) joins the legs with zero "
        "inter-leg overhead, so the 13.805 mm chain figure sits ~5.7 s away in "
        "offset space from the phase the row produces — the measured leg-2 "
        "relative phase is ~0.25 s, not 6.0044 s. Each leg is swept ALIGNED "
        "over its own release skew instead (H22).")
    skipped.append(
        "the dilation is the corner of the INFINITY-BALL the ledger records. "
        "`dev_p99_deg` bounds max_j |dq_j| for a frame; it does not say how "
        "many joints were at that bound at once, so pushing all seven to it "
        "simultaneously is admissible under the measurement and is much more "
        "displacement than any single observed frame need have had. That is "
        "why it drives most pairs to 0.00 mm. A PER-JOINT p99 — the ledger "
        "keeps none, and `verify_replay_from_frames` already computes the "
        "per-joint residual before it takes the max (track.py:1300-1302) — is "
        "the one measurement that would shrink it. It needs no new hardware "
        "run, only a re-read of the existing frame captures. Until then the "
        "dilation gates nothing (`gates.dilated.gates_on: false`): it fails "
        "every row, and a criterion that fails every row cannot discriminate. "
        "Making it transient and bounded, AND showing that it separates rows, "
        "is the precondition for it ever gating again.")
    skipped.append(
        "the TRANSITS are not swept. Every pair figure here is the uploaded "
        "STROKE against the uploaded stroke, which is what the track channel "
        "carries; the plan's `move_to_start` / approach / `move_to_rest` legs "
        "run over a different channel and are not in any of these numbers. The "
        "2026-09-08 design's H29 asks for the WHOLE-PLAN figure and this is "
        "the stroke-only one. Under the row barrier each arm reaches its "
        "verified stroke start before either is released, so the strokes are "
        "what run together — but two arms transiting AT THE SAME TIME are an "
        "unswept configuration set, and an executive must serialise the "
        "pre-stroke and post-stroke transits until they are. The idle table "
        "below covers only the other case: one arm working, the partner "
        "stationary at `move_to_rest`.")
    skipped.append(
        "the fixture is NOT in the scene: `cross_arm_separation` cannot take "
        "one (`_separation_context` builds only the two arms and the two "
        "mounts). Arm-vs-commode is a separate, unrun check — and "
        "`bottomlid_right` already reads 0.00 mm against `commode_seat` on a "
        "SOLO run at the slim link set.")
    return {
        "generated": datetime.datetime.now().isoformat(timespec="seconds"),
        # The grid this run TARGETED, not the constant: `--dt 0.1` used to write
        # `dt_s: 0.02` and the table then passed its own dt check while every
        # figure in it came off a grid that steps 109 mm between samples.
        "dt_s": float(getattr(args, "dt", DT_S) or DT_S),
        "link_set": LINK_SET,
        "clearance_floor_mm": CLEARANCE_FLOOR_MM,
        "generator": "tools/dual_arm_verdicts.py",
        "manifest_sha256": sha256_file(MANIFEST),
        "dt_note": (
            "`dt_s` is the TARGET grid. `gates.<kind>.dt` is the grid each "
            "figure was actually computed on, and every figure coarser than "
            "dt_s is named in `skipped`. A coarser grid samples a subset of the "
            "configurations: it can only report a minimum that is too large, so "
            "it refuses soundly and certifies nothing."),
        #: THE CANFD DRIFT MODEL's provenance (2026-10-02): runs, shas, fit, coverage, assumptions. Every row
        #: whose `window_terms.clock_model` is "canfd_drift" rests on it.
        "canfd_drift_model": CANFD_MODEL,
        "window_model": (
            "THE CLOCK CHOOSES THE MODEL (2026-10-02). A pair of CANFD-clocked (5.0 ms) tracks takes the CANFD "
            "DRIFT MODEL, any length: window_s = %.3f s release skew + (%.0f - %.0f) ms covering phase error + "
            "2 x %.0f ppm x the longer track (`canfd_drift_model`; Newton 2026-10-02 \"Fit it from the CANFD "
            "runs\") — host-timed streams do not drift (end drift 0.02-0.17 ms over 2.7-71 s runs), so the "
            "window is the arms' following error plus the release skew, not a clock envelope. A pair of "
            "upload-clocked (5.5545 ms) tracks keeps the upload player's model: "
            % (CANFD_SKEW_S, CANFD_E_MS[1], CANFD_E_MS[0], CANFD_RHO_PPM) +
            "TWO TERMS, and `window_s` is the max of them (2026-09-09). "
            "(1) H21's per-pair term: the envelope of dev(phi) over every "
            "observed left x right run on the CURRENT track bytes, "
            "re-referenced to each track's measured median start lag and "
            "median replay. (2) THE CLOCK-ENVELOPE FLOOR, which is what "
            "Newton's 2026-09-09 note added: term 1 is only as wide as the "
            "runs a pair happens to have, and most tracks have a handful, so "
            "it understates the real clock spread — on row 10 it read "
            "2.2493 s while the measured envelope permits 3.5105 s. The floor "
            "is LENGTH-AWARE, because the ~0.283 s program start ramp makes a "
            "single global ms/pt figure length-dependent (short tracks read "
            "5.72-6.65 ms/pt, long ones 5.53-5.98). Over the 129 COMPLETE runs "
            "on file: replay_s in [0.084 + 5.507 ms/pt x (N-1), 0.750 + 5.810 "
            "ms/pt x (N-1)] and start lag S(N) = 0.3449 + 0.4156 ms/pt x N "
            "with residuals in [-0.855, +0.975] s (n=118, from 2026-09-03, the "
            "launcher generation the executive uses). Each band is UNIONED "
            "with the pair's own samples, so no measurement is discarded. Cut "
            "runs are excluded: their replay_s is the time to the cut against "
            "the whole point count, which is not a clock reading. "
            "`window_s` is a +- half-width; `swept_window_s` is what the "
            "sweep actually used, and a row where it is SMALLER carries a "
            "figure that is an UPPER BOUND only — a minimum over a window is "
            "non-increasing in that window, so it refuses soundly and "
            "certifies nothing."),
        "dilated_method": (
            "H18, and it GATES NOTHING (`gates.dilated.gates_on: false`). A "
            "constant per-joint offset of that task's measured "
            "replay_check.dev_p99_deg (an infinity-norm over joints), signed "
            "by a central difference on the FCL distance at the thinnest "
            "commanded configuration, applied to the whole track on BOTH sides "
            "at once, then the ordinary windowed sweep. That is a SUSTAINED, "
            "worst-direction, gap-closing offset held for a whole stroke on "
            "both arms; real tracking error is transient and not adversarially "
            "aligned, so it bounds nothing the robot does — and it returns "
            "0.00 mm on every row it was computed for, so it separates "
            "nothing either. Read `min_mm_dilated: 0.00` as \"the recorded "
            "statistic does not exclude contact\", never as \"the arms hit\". "
            "Before this may gate anything it has to be made transient and "
            "bounded AND shown to discriminate; the per-joint p99 named in "
            "`skipped` is the measurement that would do it, and it needs no "
            "hardware."),
        "invariant_method": (
            "H17, and it GATES NOTHING "
            "(`gates.invariant.gates_on: false`). `offset_window_s=None`: the "
            "exact minimum over EVERY relative phase, which is the lower bound "
            "a frozen or cut partner can reach. It is recorded because it is "
            "the honest answer to \"how bad can an unbounded freeze be\" — but "
            "it reads 0.00 mm for rows 6, 7, 10 and 11 alike and 11-21 mm on "
            "the row-3 legs, i.e. it fails or nearly fails everything, which "
            "is what `cross_arm_separation`'s own docstring "
            "(dual_arm_plan_collision.py:673-679) says it does on this "
            "fixture. The unbounded freeze is answered by the partner-stop "
            "watchdog and the rung-0 detector — see each row's `requires` — "
            "not by refusing pairs."),
        "gate_policy": GATE_POLICY,
        "verdict_model": (
            "`run` answers ONE question: may these two strokes be released "
            "together? It is set from the WINDOWED minimum (`min_mm_window`, "
            "the only figure with `gates_on: true`) plus Newton's %s ruling. "
            "`serial` = Newton ruled the pair serial, or the gate measures "
            "below the %.0f mm floor on a dt<=%s grid. `concurrent` = the gate "
            "is at or above the floor and Newton did not rule the pair "
            "serial — and it is CONDITIONAL on everything in that row's "
            "`requires`. `refused` = the table cannot answer, or the ruling "
            "and the measurement disagree. A `refused` or `serial` row is not "
            "a row that may never run: its two tasks are ordinary single-arm "
            "tasks and running them one at a time is covered by the `idle` "
            "table below, at a parked height that table clears. "
            "`min_mm_invariant` and `min_mm_dilated` are RECORDED "
            "DIAGNOSTICS: both read 0.00 mm on rows the gate separates, so "
            "neither discriminates, and `decide` does not read either."
            % (NEWTON_RULING_DATE, CLEARANCE_FLOOR_MM, DT_S)),
        "parked": parked_summary(idle, tasks),
        "parked_model": (
            "B8, as data instead of a literal. For each side: the lowest swept "
            "idle height at which EVERY task the other arm runs clears the "
            "floor against that arm parked at `move_to_rest`, taken over every "
            "hand state the side is commanded to. `null` means no swept height "
            "is clear for every task and the executive must consult the `idle` "
            "rows task by task. A height that is not in `idle` was never swept "
            "and is not a candidate."),
        "pairs": [(_set_binding(p_) or p_) for p_ in pairs],
        "idle": idle,
        # WHICH SET PRODUCED THE IDLE TABLE. The 72 idle rows carry no link_set
        # of their own, so this is the ONLY record of it — and B8's four
        # below-floor findings are read off those rows, so "measured at which
        # set" is safety-relevant provenance, not decoration.
        #
        # It used to be hand-added to the sidecar and NOTHING wrote it, so the
        # first shard that regenerated the doc silently dropped it (found
        # 2026-09-10, after `--phases window dilated` carried all 72 rows but
        # not the field that says how they were measured). Written here now,
        # and CARRIED when the table is carried, so the two can never disagree.
        "idle_link_set": idle_link_set,
        "idle_dt_s": idle_dt_s,
        "skipped": skipped,
    }


# ------------------------------------------------------------------- the CLI
def cmd_recheck(args) -> int:
    # Newton, 2026-09-09: screen the ARM links first — the sequence is far apart
    # and the full 37/38-link set costs ~14x more per query for finger detail a
    # screen does not need. The chosen set is recorded in the sidecar, and
    # `verify` treats a slim figure as a SCREEN, never as a certification.
    global LINK_SET
    LINK_SET = getattr(args, "link_set", None) or LINK_SET
    out = pathlib.Path(args.sidecar)
    before = None
    if out.exists():
        try:
            before = json.loads(out.read_text())
        except Exception:                                       # noqa: BLE001
            before = None
    doc = build_sidecar(args)
    out.parent.mkdir(parents=True, exist_ok=True)
    # Write through a temp file: a sweep that dies (or a session that is
    # killed) mid-write must not leave half a verdict table behind, and a
    # concurrent `verify` must read either the old file or the new one.
    tmp = out.with_suffix(f".tmp{os.getpid()}")
    tmp.write_text(json.dumps(doc, indent=1) + "\n")
    os.replace(tmp, out)
    print(f"wrote {out} — {len(doc['pairs'])} pairs, {len(doc['idle'])} idle "
          f"rows, {len(doc['skipped'])} notes")
    if before:
        lost = ({(r.get("working_task"), r.get("idle_pole_m"))
                 for r in (before.get("idle") or [])}
                - {(r.get("working_task"), r.get("idle_pole_m"))
                   for r in doc["idle"]})
        gone = ({(r.get("row"), r.get("leg")) for r in (before.get("pairs") or [])}
                - {(r.get("row"), r.get("leg")) for r in doc["pairs"]})
        if lost or gone:
            print(f"  NOTE: {len(gone)} pair row(s) and {len(lost)} idle row(s) "
                  "that the previous table carried are not in this one — they "
                  "are pinned to bytes or tasks the manifest no longer names. "
                  "See `skipped`.")
    problems = validate_sidecar(doc)
    if problems:
        print(f"  {len(problems)} SHAPE PROBLEM(S) in what was just written:")
        for p in problems:
            print(f"    * {p}")
        return 1
    return 0


def cmd_verify(args) -> int:
    out = pathlib.Path(args.sidecar)
    if not out.exists():
        print(f"MISSING {out} — run `recheck`")
        return 2
    # This is the gate. It reports; it does not traceback. A sidecar that is
    # not JSON, is JSON of the wrong shape, or names a manifest that is not
    # there is exactly the state the exit code exists to signal.
    try:
        doc = json.loads(out.read_text())
        if not isinstance(doc, dict):
            raise TypeError(f"the sidecar is a {type(doc).__name__}, "
                            "not an object")
        problems = verify_sidecar(doc)
    except Exception as exc:                                    # noqa: BLE001
        print(f"UNREADABLE {out} — {type(exc).__name__}: {exc}")
        return 2
    if problems:
        print(f"STALE — {len(problems)} problem(s):")
        for p in problems:
            print(f"  * {p}")
        return 1
    print(f"OK — {out} matches the tracks on disk")
    return 0


def cmd_show(args) -> int:
    doc = json.loads(pathlib.Path(args.sidecar).read_text())
    print(f"generated {doc['generated']}  dt={doc['dt_s']}  "
          f"link_set={doc['link_set']}  floor={doc['clearance_floor_mm']} mm")
    print(f"gate = min_mm_window (the 'window' column); 'invar' and 'dilated' "
          f"are DIAGNOSTICS and set nothing")
    print(f"{'row':>3} {'leg':>3} {'aligned':>9} {'window*':>9} {'invar~':>9} "
          f"{'dilated~':>9} {'W s':>7}  {'verdict':<11} closest pair")

    def f(v, w=9):
        return " " * (w - 2) + "--" if v is None else f"{v:{w}.2f}"
    for p in doc["pairs"]:
        print(f"{p['row']:>3} {p['leg']:>3} {f(p['min_mm_aligned'])}"
              f"{f(p['min_mm_window'])}{f(p['min_mm_invariant'])}"
              f"{f(p['min_mm_dilated'])} {f(p['window_s'], 7)}  "
              f"{p['run']:<11} {p['closest_pair']}")
        print(f"      {p['right']['name']} || {p['left']['name']}")
        wt = p.get("window_terms") or {}
        if wt:
            sw = p.get("swept_window_s")
            print(f"      W = max(pair {wt.get('pair_dev_s')} s, clock envelope "
                  f"{wt.get('clock_envelope_s')} s) -> {p.get('window_s')} s"
                  + ("" if sw is None or p.get("window_s") is None
                     or float(sw) >= float(p["window_s"]) - 1e-6
                     else f"; SWEPT AT ONLY {sw} s — upper bound"))
        print(f"      {p['why']}")
    # `min_mm` is null for a combination nobody swept — the honest state this
    # whole table is built around — and sorting a mixed list of None and float
    # raised TypeError right here, i.e. `show` crashed on exactly the rows a
    # reader most needs to see. Nulls sort first and print as "not swept".
    bad = [i for i in doc["idle"] if not i["ok"]]
    unswept = [i for i in bad if i.get("min_mm") is None]
    print(f"\nidle table: {len(doc['idle'])} rows, "
          f"{len(bad) - len(unswept)} below the "
          f"{doc['clearance_floor_mm']:.0f} mm floor, "
          f"{len(unswept)} NOT SWEPT")
    for i in sorted(bad, key=lambda r: (r.get("min_mm") is not None,
                                        r.get("min_mm") or 0.0)):
        mm = ("not swept" if i.get("min_mm") is None
              else f"{i['min_mm']:7.2f} mm")
        print(f"  {mm:>11}  {i['working_task']} vs idle "
              f"{i['idle_side']} @ {i['idle_pole_m']} m  "
              f"({i.get('closest_pair')})")
    by_pole = {}
    for i in doc["idle"]:
        by_pole.setdefault(i["idle_pole_m"], []).append(i["ok"])
    for pole in sorted(by_pole):
        v = by_pole[pole]
        print(f"  idle pole {pole} m: {len(v) - sum(v)} of {len(v)} below floor")
    if doc["skipped"]:
        print("\nskipped / notes:")
        for s in doc["skipped"]:
            print(f"  * {s}")
    return 0


def cmd_certify(args) -> int:
    """`certify_pair` for an executive that is not Python (the C++ arm_executive).

    Prints ONE JSON object on stdout and exits 0 whenever it printed one — the
    answer is its `run` field, never the exit code, so a caller that sees a
    non-zero exit or no JSON has no answer and must refuse.
    """
    plans = {s: p for s, p in (("right", args.right_plan), ("left", args.left_plan)) if p}
    shas = {s: h for s, h in (("right", args.right_track_sha),
                              ("left", args.left_track_sha)) if h}
    res = certify_pair(args.right, args.left, sidecar=args.sidecar, row=args.row,
                       leg=args.leg, plans=plans or None, track_sha=shas or None)
    v = res.get("verdict") or {}
    print(json.dumps({k: res[k] for k in ("run", "why", "kind", "problems", "row_label",
                                          "order", "requires", "sidecar")}
                     | {"verdict_row": v.get("row"), "verdict_leg": v.get("leg")}))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("recheck", "verify", "show"):
        p = sub.add_parser(name)
        p.add_argument("--sidecar", default=str(SIDECAR))
        if name == "recheck":
            p.add_argument("--dt", type=float, default=DT_S)
            p.add_argument("--link-set", choices=("slim", "full"), default=LINK_SET,
                           help="'full' (37/38 links, fingers and pads included) is the "
                                "certifying set. 'slim' is Studio's own 10/11-link ARM set: "
                                "no fingers, so it is an UPPER bound on separation and a "
                                "SCREEN. Newton, 2026-09-09: screen the arms first, because "
                                "the sequence is far apart and the full set costs ~14x more "
                                "per query for finger detail the screen does not need.")
            p.add_argument("--cache", default=str(DEFAULT_CACHE))
            p.add_argument("--phases", nargs="+", default=list(PHASES),
                           choices=PHASES)
            p.add_argument("--only", nargs="*", default=None,
                           help="row:leg selectors, e.g. 10:0 3:1")
            p.add_argument("--only-tasks", nargs="*", default=None)
            p.add_argument("--cache-only", action="store_true",
                           help="assemble from cached sweeps only; never start "
                                "one. Anything missing is null + `skipped`.")
            p.add_argument("--replace", action="store_true",
                           help="do NOT merge onto the sidecar already on "
                                "disk. Without this a shard (--only / "
                                "--only-tasks / --phases) carries every row it "
                                "did not recompute, provided the bytes and the "
                                "staged state it was measured at are still the "
                                "ones the manifest names.")
            p.add_argument("--quiet", action="store_true")
    c = sub.add_parser("certify", help="[N-12]: may these two tasks run together "
                       "now? One JSON object on stdout (see cmd_certify)")
    c.add_argument("--sidecar", default=str(SIDECAR))
    c.add_argument("--right", required=True, help="the right arm's manifest task name")
    c.add_argument("--left", required=True, help="the left arm's manifest task name")
    c.add_argument("--row", type=int, default=None,
                   help="the schedule row being dispatched (omit for a direct pair)")
    c.add_argument("--leg", type=int, default=0)
    c.add_argument("--right-plan", default=None, help="the plan the right arm will run")
    c.add_argument("--left-plan", default=None, help="the plan the left arm will run")
    c.add_argument("--right-track-sha", default=None,
                   help="sha256 of the track bytes the right arm will play")
    c.add_argument("--left-track-sha", default=None,
                   help="sha256 of the track bytes the left arm will play")
    try:
        args = ap.parse_args(argv)
    except SystemExit as exc:
        # argparse's own exit. An executive that calls `main([...])` in process
        # — which is the documented way to use `verify` — must get an exit CODE
        # back, not a SystemExit unwinding whatever thread it was called on.
        return int(exc.code or 2)
    return {"recheck": cmd_recheck, "verify": cmd_verify,
            "show": cmd_show, "certify": cmd_certify}[args.cmd](args)


if __name__ == "__main__":
    raise SystemExit(main())
