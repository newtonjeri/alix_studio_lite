#!/usr/bin/env python3
"""EMIT A TIME-OPTIMAL RUNG: arc-length denoise, then TOPP-RA. Writes a track and a sidecar.

    python3 .../emit_toppra_rung.py                            # price it, write nothing
    python3 .../emit_toppra_rung.py --write --smooth-mm 10

WHY THIS EXISTS. Every hardware run of the bowl-out path to date is DIAL 1 — `capped_millideg`,
the lambda re-time. It varies the advance per source line (so it is not uniform), but it carries
NO ACCELERATION MODEL: the only protection is a low-pass on the scale and a ramp bound on
dlambda/dline, which are proxies for a limit, not a limit. The taught lane's DIAL 2 (TOPP-RA) is
skipped whenever the model rung is active (`taught_tracks.py`: "solver skipped: its rungs carry
none of the model's caps"), so it has never been emitted for this path, and `ruckig_on_s` — the
canfd channel's own parameteriser per `channel_policy.yaml` — has never touched a taught path.

WHAT THE 2026-09-21 RUNS SHOWED, and why this is the next artefact. The dominant departure event
was an OVERSHOOT at a 53.6 mm radius corner, 32.4 mm, reproducible across both runs, at 96.9 %
through. Lambda carried 441 mm/s into that corner because nothing told it not to. TOPP-RA prices
curvature by construction and commands 300 mm/s there — 32 % less — while the 10 mm arc-length
denoise lowers |d2q/ds2| enough that the whole run still comes out 1.09x FASTER than lambda's.
So this is not a speed-for-accuracy trade: it is faster AND gentler where the failure was.

THE ORDER MATTERS. Denoise FIRST, in the arc-length domain, because the binding constraint on a
taught path is its own curvature: |d2q/ds2| v^2 reaches the 600 deg/s^2 limit at p95 while the
joint SPEED ratings still permit ~5x the speed (acceleration binds before the ratings on 99.9 % of
this path). A TEMPORAL filter cannot do it evenly — a drag-teach recording's speed varies ~9x
within one recording, so one cutoff in Hz is a wildly varying cutoff in mm.

PROVENANCE IS NOT OPTIONAL. A derived streamable file that carries no source sidecar reads back
`hardware: unknown`, which only WARNS at stream time instead of refusing — that is how an
`analysis_only` recording (another arm mount) could be laundered into something runnable. The
review that found this in `respace_teach.py` recorded the rule for every future tool that derives
a streamable file, so this one writes `source_hardware` and the source sha and REFUSES an
analysis_only source outright.

INSTALL IT VERBATIM. The emitted file is already timed at the target period, so
`taught_tracks.py certify <file> --side left --slot NN --pick verbatim --keep-pauses` is the
install: `verbatim` stops the lane re-timing it (which would discard the parameterisation that is
the entire point), and `--keep-pauses` stops R4 collapsing its start/end zero-velocity lines.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve()
REPO = HERE.parents[3]
for _p in ("core/python/alix_dispatch/src", "tools", "tools/motion_analysis"):
    sys.path.insert(0, str(REPO / _p))
sys.path.insert(0, str(HERE.parent))

import taught_tracks as tt                                            # noqa: E402
import cut_dossier as cd                                              # noqa: E402
import local_speed as LS                                              # noqa: E402
from alix_dispatch import track as tr                                 # noqa: E402
from scipy.spatial import cKDTree                                     # noqa: E402

#: `--source rung`: the filtered bowl-out rung at taught speed — the geometry slots 51-58 came from.
#: It is the 5 Hz certify ladder's model lambda 1.0 rung, smoothed on CLOCK_NOMINAL_S (an effective
#: 5.55 Hz) and a MUTABLE file: any re-shape of the recording rewrites it. Kept to reproduce 52-58.
SRC = REPO / "Resource/tracks/taught/rungs/left-bwl-out-01_slow_smooth5hz/track_model1.00.txt"
#: The recording it descends from, and its provenance sidecar.
ORIGIN = REPO / "Resource/tracks/taught/source/left-bwl-out-01_slow.txt"
#: What slot 51 (the lambda rung that ran) is, for the comparison.
LAMBDA_RUNG = REPO / "Resource/tracks/taught/rungs/left-bwl-out-01_slow_smooth5hz/track_model2.20.txt"
OUT_DIR = REPO / "Resource/tracks/taught/derived"
SIDE, FRAME = "left", "L_glove_frame_2"


def sha(p: pathlib.Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


#: The provenance keys `taught_tracks.source_provenance` actually reads. A derived file that omits
#: them is not "missing a nicety": `certify` refuses it, and if it were somehow streamed the
#: hardware verdict would read `unknown`, which only WARNS. Note the key is `hardware` — writing
#: only `source_hardware` (respace_teach's own name for it) is NOT enough, which is how the first
#: emission of this file got refused.
PROV_KEYS = ("side", "frame", "pole_m", "hand_state", "hand_counts", "hardware",
             "taught_by", "date", "units", "note")


def origin_provenance() -> dict:
    """The origin recording's provenance, INHERITED rather than re-declared.

    Declaring it fresh (`certify --taught-here --pole-mm ... --hand-state ...`) would assert the
    devices from the command line and break the chain back to the recording, which is exactly what
    the audit trail is for. Inheriting keeps `hardware: ok` attributable to the original teach.
    """
    s = ORIGIN.parent / (ORIGIN.stem + ".SOURCE.json")
    if not s.is_file():
        raise SystemExit(f"no provenance sidecar beside {ORIGIN.name} — REFUSING to derive from it")
    d = json.loads(s.read_text())
    return {k: d.get(k) for k in PROV_KEYS}


def toppra_under_departure(q_s: np.ndarray, path, budget_mm: float, dt: float) -> np.ndarray:
    """TOPP-RA with the lane's departure budget as a per-point velocity bound.

    `departure_lambda_cap` gives, per SOURCE LINE, the largest lambda that keeps the predicted
    departure inside `budget_mm`. At lambda the file advances lambda lines per tick, so the implied
    TCP speed cap at line i is  lam_i * |dP_i| / clock.  min(that, the joint ratings) is the bound.

    THE MAPPING IS BY INDEX, NOT BY INTERPOLATION. The cap is one entry per source line and
    `path.s` is the arc length of those same lines, so `[path.kept]` aligns them exactly. The first
    version interpolated against `linspace(s0, s1, n)`, which assumes the lines are uniform in arc
    length; on this recording they vary 7.3x (p90/p10), so caps landed several lines from the
    reversal they belonged to and the result came out slower AND worse on departure at once —
    an outcome no correct constraint can produce, which is how it was caught (Newton, 2026-09-21:
    "there is no way we can lose on both tracking accuracy and the speed").
    """
    import toppra as ta
    import toppra.algorithm as algo
    import toppra.constraint as constraint
    lam_cap, _ = tt.departure_lambda_cap(q_s, SIDE, FRAME, tt.execution_model(), budget_mm,
                                         tt.CLOCK_PREDICT_S)
    P, _ = cd.fk_tool(q_s, SIDE, FRAME)
    P = P * 1000.0
    dP = np.concatenate([[0.0], np.linalg.norm(np.diff(P, axis=0), axis=1)])
    v_dep = np.clip(np.asarray(lam_cap, float) * np.maximum(dP, 1e-9) / tt.CLOCK_PREDICT_S,
                    1.0, 1e6)[path.kept]
    if len(v_dep) != len(path.s):
        raise SystemExit(f"cap/sample misalignment: {len(v_dep)} caps vs {len(path.s)} samples")
    total = path.total()
    x = path.s / total
    dq_dx = np.abs(path.dq(path.s)) * total
    vlim = np.maximum(np.minimum(np.array(tr.CANFD_RATED_DEG_S)[None, :],
                                 dq_dx * (v_dep[:, None] / total)), 1e-3)

    def vlim_at(xx):
        col = np.array([np.interp(xx, x, vlim[:, j]) for j in range(7)])
        return np.column_stack([-col, col])

    inst = algo.TOPPRA(
        [constraint.JointVelocityConstraintVarying(vlim_at),
         constraint.JointAccelerationConstraint(
             LS.ACC_DEG_S2, discretization_scheme=constraint.DiscretizationType.Interpolation)],
        ta.SplineInterpolator(x, path.q_src[path.kept]), solver_wrapper="seidel")
    traj = inst.compute_trajectory(0, 0)
    if traj is None:
        raise SystemExit("TOPP-RA returned no trajectory under this departure budget")
    return traj(np.arange(0.0, float(traj.duration), dt))


#: the arrival hold covers this many times the joint travel G1's approach window takes at the held
#: speed: crossing it takes 1.5 x 0.3 s, so the whole window lies inside the hold
HOLD_SPAN = 1.5


def arrival_hold(q: np.ndarray, rate_deg_s, headroom: float, model: dict | None = None) -> tuple:
    """THE ARRIVAL HOLD (2026-09-23): per-knot joint speed caps that keep certify's G1 — the
    PREDICTED executed margin at every joint limit — however hard the file accelerates.

    G1 (taught_tracks.path_reversals, model_predictions): at every extreme of a joint the arm
    overshoots by max(floor, tau_limit x the ARRIVAL rate), where the arrival rate is the fastest the
    joint moves in the `approach_window_s` (0.3 s) BEFORE the extreme, and the executed margin to
    the limit must keep `limit_eps_deg`. So a joint may approach an extreme `margin` deg from its
    limit at (margin - eps) / tau_limit, over that whole window, not only at the extreme. For every
    extreme where that is under the joint's rate, this caps the joint at headroom x it over the
    knots before the extreme within HOLD_SPAN x (the capped speed x the window) degrees of it: at
    the capped speed the joint needs HOLD_SPAN windows to cross them, so G1's window lies inside.
    The departure side stays free: G1 does not read it, and the joint moves away from the limit.
    G1 reads rates on its 5.4 ms prediction clock, 7 % under the 5 ms file's own: a margin more.

    Why: past the acceleration limit (`+acc`) TOPP-RA carries a joint into a turn near its limit at
    full speed and brakes at the last moment. On LID TOP V2, J1 turns 12.6 deg from its -178 deg
    limit, and every file at 1.5-8x the acceleration limit failed G1 (the 3x file kept 0.012 of the
    0.5 deg). At 1x the braking starts early enough on its own (the same turn keeps 2.0 deg).
    Returns (caps, one row of 7 per knot, deg/s; the extremes held)."""
    from scipy.signal import find_peaks
    q = np.asarray(q, float)
    model = model or tt.execution_model()
    lim = np.array(tt._ptr().K.JOINT_LIMITS_DEG, dtype=float)        # the limits G1 reads
    tau, eps = float(model["tau_limit_s"]), float(model["limit_eps_deg"])
    win, prom = float(model["approach_window_s"]), float(model["prominence_deg"])
    rate = np.broadcast_to(np.asarray(rate_deg_s, float), (q.shape[1],))
    caps = np.broadcast_to(rate, q.shape).copy()
    held = []
    for j in range(q.shape[1]):
        for sgn in (1.0, -1.0):
            for k in find_peaks(sgn * q[:, j], prominence=prom)[0]:
                margin = float(lim[j, 1] - q[k, j]) if sgn > 0 else float(q[k, j] - lim[j, 0])
                v = max(float(headroom) * (margin - eps) / tau, 1.0)
                if v >= rate[j]:
                    continue                                 # far enough from the limit: no cap
                span = HOLD_SPAN * v * win
                i0 = int(k)
                while i0 > 0 and abs(q[i0 - 1, j] - q[k, j]) <= span:
                    i0 -= 1
                caps[i0:k + 1, j] = np.minimum(caps[i0:k + 1, j], v)
                held.append(dict(joint=j + 1, knot=int(k), margin_deg=round(margin, 3), hold_deg_s=round(v, 1),
                                 travel_deg=round(float(np.abs(q[i0:k + 1, j] - q[k, j]).max()), 1)))
    return caps, held


def post_lowpass(Q: np.ndarray, hz: float, dt: float, order: int = 4, hold_s: float = 1.0) -> np.ndarray:
    """ZERO-PHASE LOW-PASS OF THE TIMED FILE, q(t), after TOPP-RA (2026-09-21, the CANFD shaking
    study). The hand's visible shake is 5-12 Hz: mainly the file's own 5-8 Hz content executed
    faithfully, plus ~2-2.5x in 8-12 Hz from compliance between the joint encoders and the wrist;
    TOPP-RA's full-acceleration switching and the taught path's texture played 2-4x faster put it
    there. A temporal cutoff attenuates it whatever the playback speed, which no arc-length cutoff
    can: at 6 Hz (order 4, zero phase) it halves the content at the cutoff and all but removes
    8-12 Hz (amplitude x0.09 at 8 Hz, x0.02 at 10 Hz), while 5 Hz passes at ~x0.8.
    Padded with `hold_s` of the first and last line and trimmed back, so the ends are filtered
    against a still signal: the file still starts and ends NEAR rest (the end lines are filter
    output, not held exactly) and keeps its line count."""
    from scipy.signal import butter, sosfiltfilt
    n = int(round(hold_s / dt))
    qq = np.r_[np.repeat(Q[:1], n, 0), Q, np.repeat(Q[-1:], n, 0)]
    sos = butter(order, float(hz) / (0.5 / dt), output="sos")
    return sosfiltfilt(sos, qq, axis=0)[n:-n]


def _rel(p: pathlib.Path) -> str:
    """repo-relative when inside the repo (the derived folder), else absolute (a --out-dir scratch)"""
    try:
        return str(p.resolve().relative_to(REPO))
    except ValueError:
        return str(p)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--write", action="store_true", help="write the track and its sidecar")
    ap.add_argument("--out-dir", default=None,
                    help="write into this folder instead of Resource/tracks/taught/derived — for a "
                         "CANDIDATE that is measured and thrown away (taught_pipeline.py's speed "
                         "ladder for a row with a TCP budget)")
    ap.add_argument("--smooth-mm", type=float, default=10.0,
                    help="arc-length denoise cutoff, mm (default 10; 5 keeps the path tighter)")
    ap.add_argument("--dt-ms", type=float, default=tr.CANFD_DEFAULT_PERIOD_S * 1e3,
                    help="target stream period, ms (default 5.0, the controller's cycle)")
    ap.add_argument("--dep-budget-mm", type=float, default=None,
                    help="optimise SUBJECT TO the predicted-departure budget, mm (the lane's own "
                         "`departure_lambda_cap`, pulled back onto TOPP-RA's velocity bound). "
                         "Without it TOPP-RA knows only velocity and acceleration, and a "
                         "budget-respecting lambda rung is not a fair comparison. 16 beat slot 51 "
                         "on both speed and p95 departure in the model, 2026-09-21.")
    ap.add_argument("--acc-scale", type=float, nargs="+", default=[1.0],
                    help="multiple of the arm's rated 600 deg/s^2 handed to TOPP-RA: ONE value for "
                         "every joint, or SEVEN (J1..J7). 1.0 is the rating. Measured 2026-09-21 on "
                         "CANFD: 1.0 follows as a clean 25 ms delay; 1.5 on every joint (slot 54) "
                         "made J6 miss its direction reversals, departure p95 2.0 -> 23.4 mm.")
    ap.add_argument("--origin", default=None,
                    help="the taught recording to derive from (default: left-bwl-out-01_slow). Its "
                         "provenance is read from <stem>.SOURCE.json beside it, as for the default")
    ap.add_argument("--source", choices=("recording", "rung"), default="recording",
                    help="recording (default since 2026-09-21): derive the source from the taught "
                         "RECORDING itself — smooth_taught at --smooth-hz on its own 5 ms clock, then "
                         "R4 — pinned by the recording's sha, no mutable intermediate. rung: the old "
                         "5.55 Hz ladder rung that slots 52-58 were emitted from (to reproduce them)")
    ap.add_argument("--smooth-hz", type=float, default=5.0,
                    help="with --source recording: the temporal low-pass, Hz, at the 5 ms record "
                         "clock (default 5, Newton 2026-09-17)")
    ap.add_argument("--post-lp-hz", type=float, default=None,
                    help="zero-phase low-pass of the TIMED file q(t) at this cutoff, after TOPP-RA "
                         "(4th order, ends padded with a 1 s hold and trimmed back). 6 Hz is the "
                         "shaking study's pick: predicted "
                         "hand shake -50 %% for path p95 0.8 / max 2.9 mm and no time cost")
    ap.add_argument("--rate-scale", type=float, default=1.0,
                    help="multiple of each joint's RATING (180/180/225x5 deg/s) handed to TOPP-RA "
                         "as its velocity bound. 1.0 lets the file sit exactly on a rating, where "
                         "4105 is untested and executed overshoot (ratio up to 1.03 on slot 52) "
                         "goes past it; 0.95 leaves that margin.")
    ap.add_argument("--arrival-hold", type=float, default=None, metavar="H",
                    help="hold each joint, on its approach to every extreme near a joint limit, to H x "
                         "the arrival speed certify's G1 allows there ((margin - eps) / tau_limit), so "
                         "the file keeps G1 however hard it accelerates (`arrival_hold`). "
                         "taught_pipeline.py's `+acc` candidates use 0.8. Off by default.")
    args = ap.parse_args()
    if args.arrival_hold is not None and not 0.0 < args.arrival_hold <= 1.0:
        raise SystemExit(f"--arrival-hold {args.arrival_hold:g}: a fraction of G1's allowance, 0 < H <= 1")
    if args.arrival_hold and args.dep_budget_mm:
        raise SystemExit("--arrival-hold is for the plain TOPP-RA re-time, not --dep-budget-mm")
    global ORIGIN
    default_origin = args.origin is None
    if not default_origin:
        ORIGIN = (REPO / args.origin) if not pathlib.Path(args.origin).is_absolute() \
            else pathlib.Path(args.origin)
        if args.source == "rung":
            raise SystemExit("--source rung exists only for the default recording (slots 52-58)")
    dt = float(args.dt_ms) / 1e3
    if len(args.acc_scale) not in (1, 7):
        raise SystemExit(f"--acc-scale takes 1 or 7 values, got {len(args.acc_scale)}")
    # NOT `acc`: main() later rebinds that name to the emitted file's acceleration array
    acc_x = np.broadcast_to(np.asarray(args.acc_scale, float), (7,)).copy()
    uniform = bool(np.all(acc_x == acc_x[0]))
    acc_txt = f"{acc_x[0]:g}" if uniform else "/".join(f"{a:g}" for a in acc_x)

    prov = origin_provenance()
    if prov.get("side") and prov["side"] != SIDE:
        raise SystemExit(f"{ORIGIN.name} was taught on the {prov['side']} arm; this emitter is "
                         f"{SIDE}-arm only")
    hw = str(prov.get("hardware") or "unknown")
    if hw == "analysis_only":
        raise SystemExit(f"REFUSED: {ORIGIN.name} is hardware: analysis_only (another arm mount). "
                         f"Deriving a streamable file from it would only WARN at stream time.")

    LS.ACC_DEG_S2 = 600.0 * acc_x
    LS.RATE_DEG_S = np.array(tr.CANFD_RATED_DEG_S) * float(args.rate_scale)
    if args.source == "recording":
        # THE LANE'S OWN TWO STEPS, in the lane's order: the temporal low-pass on the record clock,
        # then R4. No lambda pass: TOPP-RA discards the timing, so only the geometry matters.
        q_src_deg, r4 = tt.strip_pauses(tt.smooth_taught(tt.read_q_deg(ORIGIN),
                                                         float(args.smooth_hz), tt.CLOCK_RECORD_S))
        src_path, src_sha = ORIGIN, sha(ORIGIN)
        src_desc = (f"the recording, smooth_taught {args.smooth_hz:g} Hz on the 5 ms record clock, "
                    f"R4 ({r4['removed_lines']} lines in {r4['runs']} runs removed)")
    else:
        q_src_deg, src_path, src_sha = tt.read_q_deg(SRC), SRC, sha(SRC)
        src_desc = ("source rung: smooth_taught 5 Hz on CLOCK_NOMINAL_S (effective 5.55 Hz), "
                    "R4 pauses removed, model lambda 1.0")
    base = LS.TaughtPath(q_src_deg)
    q_s = LS.arclen_smooth(base, float(args.smooth_mm))
    path = LS.TaughtPath(q_s)
    held = []
    if args.dep_budget_mm:
        Q = toppra_under_departure(q_s, path, float(args.dep_budget_mm), dt)
    else:
        caps = None
        if args.arrival_hold:
            caps, held = arrival_hold(path.q_src[path.kept], LS.RATE_DEG_S, float(args.arrival_hold))
        Q = LS.retime_toppra(path, LS.demand("max", path), dt, vcap=caps)
    if args.post_lp_hz:
        Q = post_lowpass(Q, float(args.post_lp_hz), dt)

    # ── score it against the lambda rung that actually ran, on the ORIGINAL geometry
    # the lambda rung slot 51 ran is only a comparison for the default recording
    Ql = tt.read_q_deg(LAMBDA_RUNG) if default_origin else None
    # THE YARDSTICK IS THE RECORDING AS TAUGHT. Until 2026-09-21 this measured against
    # `base` — the 5 Hz-smoothed SOURCE — while printing "vs the UNSMOOTHED taught path", so every
    # sidecar's path_moved_vs_taught_mm before then is against the smoothed source, not the
    # recording. The report's `plot.py` "taught" column was always against the recording.
    P0, _ = cd.fk_tool(tt.read_q_deg(ORIGIN), SIDE, FRAME)
    P0 = P0 * 1000.0
    Pn, _ = cd.fk_tool(Q, SIDE, FRAME)
    Pn = Pn * 1000.0

    def dens(P, step=0.25):
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
        k = np.concatenate([[True], np.diff(s) > 1e-9])
        su = np.arange(0.0, s[-1], step)
        return np.column_stack([np.interp(su, s[k], P[k, i]) for i in range(3)])

    moved = cKDTree(dens(P0)).query(Pn)[0]                            # vs the RECORDING
    rows = [tuple(int(x) for x in r) for r in np.round(Q * 1000.0).astype(np.int64)]
    pf_flat = tr.canfd_preflight(rows, dt, "flat")
    pf_perj = tr.canfd_preflight(rows, dt, "per_joint")
    dur = (len(Q) - 1) * dt
    dur_l = (len(Ql) - 1) * dt if Ql is not None else None
    step = np.abs(np.diff(Q, axis=0))
    acc = np.diff(Q, n=2, axis=0) / dt ** 2

    print(f"source   {src_desc}  ({base.q_src[base.kept].shape[0]} pts)")
    print(f"origin   {ORIGIN.relative_to(REPO)}  hardware: {hw}")
    print(f"denoise  {args.smooth_mm:g} mm arc-length   TOPP-RA acc {acc_txt}x600 "
          f"deg/s^2   rate {args.rate_scale:g}x rating   period {dt*1e3:g} ms\n")
    if args.arrival_hold:
        print(f"  arrival hold {args.arrival_hold:g}x G1's allowance: {len(held)} extreme(s) near a limit"
              + "".join(f"\n    J{e['joint']} turns {e['margin_deg']:.1f} deg from its limit: held to "
                        f"{e['hold_deg_s']:.0f} deg/s over the last {e['travel_deg']:.1f} deg before it"
                        for e in held[:8])
              + (f"\n    ... and {len(held) - 8} more" if len(held) > 8 else ""))
    print(f"  points            {len(Q)}   duration {dur:.2f} s")
    if Ql is not None:
        print(f"  vs lambda (slot 51, {len(Ql)} pts, {dur_l:.2f} s): {dur_l/dur:.2f}x faster")
    print(f"  path vs the taught RECORDING: p50 {np.percentile(moved,50):.3f}  "
          f"p95 {np.percentile(moved,95):.3f}  max {moved.max():.3f} mm")
    jb = max(range(7), key=lambda k: pf_perj["peak_rate_util"][k])
    print(f"  peak rate J{jb+1} {pf_perj['peak_rate_deg_s'][jb]:.1f} deg/s = "
          f"{pf_perj['peak_rate_util'][jb]:.3f} of its rating")
    print(f"  max step {step.max():.3f} deg (fault 10.0)   commanded accel peak "
          f"{np.abs(acc).max():.0f} deg/s^2")
    print(f"  CANFD gate: flat {'PASS' if pf_flat['ok'] else 'REFUSED'} "
          f"({pf_flat['lines_over_rate']} over)   per-joint "
          f"{'PASS' if pf_perj['ok'] else 'REFUSED'} ({pf_perj['lines_over_rate']} over)")
    if not pf_perj["ok"]:
        print(f"  ** {pf_perj['reasons'][0][:160]}")
        return 2

    if not args.write:
        print("\n--report only; nothing written. Add --write to emit.")
        return 0

    out_dir = pathlib.Path(args.out_dir).resolve() if args.out_dir else OUT_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    name = (("bwl_out_01_slow_left" if default_origin else f"{ORIGIN.stem}_{SIDE}")
            + f"_toppra{args.smooth_mm:g}mm"
            + (f"_dep{args.dep_budget_mm:g}" if args.dep_budget_mm else "")
            # EVERY KNOB IS IN THE NAME. Before 2026-09-21 acc was not, so acc 1.0 and 1.5 at the
            # same smoothing wrote the same file. The defaults stay out, so existing names hold.
            + ("" if (uniform and acc_x[0] == 1.0) else
               f"_acc{acc_x[0]:g}" if uniform else "_acc" + "-".join(f"{a:g}" for a in acc_x))
            + (f"_rate{args.rate_scale:g}" if args.rate_scale != 1.0 else "")
            + (f"_hold{args.arrival_hold:g}" if args.arrival_hold else "")
            # THE RE-SAMPLING IS IN THE NAME TOO. arclen_smooth went from linear to cubic
            # re-interpolation on 2026-09-21 (local_speed.py); the same knobs now make different
            # bytes, and slots 52-56 were certified from the linear files under the plain names.
            + (f"_rec{args.smooth_hz:g}hz" if args.source == "recording" else "")
            + "_cubic"
            + (f"_lp{args.post_lp_hz:g}hz" if args.post_lp_hz else "")
            + f"_{dt*1e3:g}ms")
    out = out_dir / f"{name}.txt"
    # NEVER SILENTLY REPLACE A DERIVED FILE WITH DIFFERENT BYTES: a certified slot's provenance
    # points at it by path and sha.
    if out.exists():
        tmp = out.with_suffix(".check.txt")
        tt.write_track(tmp, np.round(Q * 1000.0).astype(np.int64))
        same = tmp.read_bytes() == out.read_bytes()
        tmp.unlink()
        if not same:
            # THE USUAL CAUSE IS THE PYTHON ENVIRONMENT, not the recipe: numpy/scipy/toppra versions
            # move the last digit of a few lines (2026-09-23: the system python vs .venv-taught gave
            # 1 millidegree on 33 of 2511 lines, and .venv-taught reproduced the file exactly)
            old = np.round(tt.read_q_deg(out) * 1000.0).astype(np.int64)
            new = np.round(Q * 1000.0).astype(np.int64)
            hint = ""
            if old.shape == new.shape and np.abs(old - new).max() <= 1:
                hint = (f" It differs by at most 1 millidegree, on {int((old != new).any(axis=1).sum())} of "
                        f"{len(new)} lines: the same recipe under other numpy/scipy/toppra versions. Run "
                        f"from the environment that made it (the lane's: `source .venv-taught/bin/activate`)"
                        f" and it comes out identical.")
            raise SystemExit(f"REFUSED: {_rel(out)} exists with DIFFERENT bytes — a "
                             f"certified slot may point at it. Move it aside first.{hint}")
    tt.write_track(out, np.round(Q * 1000.0).astype(np.int64))
    side_car = out.with_suffix(".json")
    side_car.write_text(json.dumps({
        # THE INHERITED PROVENANCE FIRST, under the key names `source_provenance` reads.
        **prov,
        "kind": "derived_streamable",
        "emitter": "experiments/exp01_.../src/emit_toppra_rung.py",
        "source": {"path": str(src_path.relative_to(REPO)), "sha256": src_sha, "kind": args.source},
        "origin": {"path": str(ORIGIN.relative_to(REPO)), "sha256": sha(ORIGIN)},
        # the rule from the respace_teach review: a derived streamable file carries the
        # source's hardware verdict so `hardware_use` inherits it rather than reading "unknown"
        "source_hardware": hw,          # respace_teach's name for it, kept for that convention
        "ik_frame": FRAME,
        "provenance_inherited_from": str((ORIGIN.parent / (ORIGIN.stem + ".SOURCE.json"))
                                         .relative_to(REPO)),
        # WHAT THE SOURCE RUNG ACTUALLY IS (audit 2026-09-21; the old text said only "5 Hz"):
        # the smooth-5hz certify ladder's model lambda 1.0 rung -- smooth_taught at 5 Hz evaluated
        # on CLOCK_NOMINAL_S (an effective 5.55 Hz), then R4 (6189 lines of pauses removed), then
        # the model re-time at lambda 1.0.
        "pipeline": [src_desc,
                     f"arclen_smooth {args.smooth_mm:g} mm, cubic re-sampling onto the source knots",
                     (f"departure budget {args.dep_budget_mm:g} mm -> TOPP-RA velocity bound"
                      if args.dep_budget_mm else "NO departure constraint"),
                     *([f"arrival hold {args.arrival_hold:g} x G1's allowance at {len(held)} extreme(s) "
                        f"near a joint limit -> TOPP-RA velocity bound"] if args.arrival_hold else []),
                     f"TOPP-RA, vel = {args.rate_scale:g} x joint ratings, "
                     f"acc = {acc_txt} x 600 deg/s^2",
                     f"sampled at {dt*1e3:g} ms",
                     (f"zero-phase low-pass of q(t) at {args.post_lp_hz:g} Hz (4th order, ends held)"
                      if args.post_lp_hz else "no post low-pass")],
        **({"arrival_hold": {"headroom": float(args.arrival_hold), "extremes": held}}
           if args.arrival_hold else {}),
        "period_s": dt, "points": int(len(Q)), "duration_s": float(dur),
        "path_moved_vs_taught_mm": {"p50": float(np.percentile(moved, 50)),
                                    "p95": float(np.percentile(moved, 95)),
                                    "max": float(moved.max())},
        "canfd": {"flat_ok": bool(pf_flat["ok"]), "per_joint_ok": bool(pf_perj["ok"]),
                  "peak_rate_deg_s": pf_perj["peak_rate_deg_s"],
                  "peak_rate_util": pf_perj["peak_rate_util"]},
        "compare_lambda": ({"rung": str(LAMBDA_RUNG.relative_to(REPO)),
                            "points": int(len(Ql)), "duration_s": float(dur_l),
                            "speedup": float(dur_l / dur)} if Ql is not None else None),
    }, indent=2) + "\n")
    print(f"\nwrote {_rel(out)}  ({len(Q)} lines)")
    print(f"      {_rel(side_car)}  (source_hardware: {hw})")
    print(f"\nINSTALL IT VERBATIM — the file is already timed, so the lane must not re-time it:")
    print(f"  python3 tools/taught_tracks.py certify {_rel(out)} \\\n"
          f"      --side {SIDE} --slot 52 --pick verbatim --keep-pauses")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
