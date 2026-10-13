#!/usr/bin/env python3
"""LOCAL SPEED CONTROL ON A TAUGHT PATH — the three parameterisers, one interface.

    python3 .../local_speed.py --report                      # the three-way, bowl-outside-slow
    python3 .../local_speed.py --report --profile flat:1.4    # a different demand
    python3 .../local_speed.py --build ruckig --out track.txt

THE OBJECTIVE THIS SERVES, in Newton's words: "to be able to control speed at any point, of course
its neighbouring points included, without deviations from the TCP path". So the quantity being
commanded is TCP SPEED AS A FUNCTION OF POSITION ALONG THE PATH, and the path itself must not move.

WHY THREE BACKENDS AND NOT ONE. Every one of them re-samples along the SAME geometry, so none of
them can move the path; they differ entirely in what they bound while doing it:

  lambda     `taught_tracks.capped_millideg` / `retime_to_budget.retime`. dt/du = period/s(u),
             integrated, then resampled. Path-exact BY CONSTRUCTION (every output sample is a convex
             combination of two ADJACENT input samples). Bounds NOTHING: a step in s is a delta in
             acceleration, and the only protection is a low-pass on s(u) plus a ramp bound on
             dlambda/dline. Those are PROXIES for a limit, not a limit.
  toppra      time-optimal parameterisation subject to velocity AND acceleration constraints,
             including the centripetal term. No jerk bound. This is the `track` channel's own
             parameteriser (channel_policy.yaml).
  ruckig_on_s Ruckig Community on the scalar path coordinate s: per piece, (s, sdot, sddot) solved
             to a target under constant (v_cap, a_cap, j_cap), then q(t) = q(s(t)). Bounds velocity,
             acceleration AND JERK. This is the `canfd` channel's own parameteriser, and it has
             never been applied to a taught path — the taught lane skips the solver whenever the
             model rung is active (`taught_tracks.py`, "solver skipped: its rungs carry none of the
             model's caps"), which is why every taught result to date is lambda.

s IS TCP ARC LENGTH IN MILLIMETRES, not joint arc length and not a sample index. That is the
parameter the objective is stated in, and it makes the demand profile mean the same thing on every
path. The joint limits are then pulled back onto s through |dq/ds|.

THE ONE APPROXIMATION, STATED AND THEN CHECKED. A polyline has no second derivative at its knots,
so q(s) is carried by an INTERPOLATING cubic spline: it passes through every taught sample exactly,
and between samples it departs from the chord by its own curvature. That departure is MEASURED at
the ik_frame and reported next to lambda's, so the cost of having a differentiable path is never
assumed. Likewise `a_cap` is a first-order pull-back that ignores ddq*sdot^2 (as the shipping C++
does), so the REALISED joint acceleration is measured after composition and reported against the
limit rather than trusted.
"""
from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np
from scipy.interpolate import CubicSpline

HERE = pathlib.Path(__file__).resolve()
REPO = HERE.parents[3]
for _p in ("core/python/alix_dispatch/src", "tools", "tools/motion_analysis",
           "experiments/exp02_toppra_cartesian_pipeline/src"):
    sys.path.insert(0, str(REPO / _p))
sys.path.insert(0, str(HERE.parent))

import taught_tracks as tt                                            # noqa: E402
import cut_dossier as cd                                              # noqa: E402
from alix_dispatch import track as tr                                 # noqa: E402

#: The source: the filtered bowl-outside-slow rung ladder slot 51 was installed from.
SRC_DIR = REPO / "Resource/tracks/taught/rungs/left-bwl-out-01_slow_smooth5hz"
SRC_RUNG = "track_model1.00.txt"          #: lambda 1.0 = the taught speed, the common baseline
SIDE, FRAME = "left", "L_glove_frame_2"
#: The controller's cycle, and CANFD's period (`tr.CANFD_DEFAULT_PERIOD_S`).
DT = tr.CANFD_DEFAULT_PERIOD_S
#: Joint limits: the ratings, the vendor's acceleration bound, and the jerk figure the
#: planned lane uses. `plan_min_time` cites libalgo_get_joint_acc_maxlim = 10.472 rad/s^2.
RATE_DEG_S = np.array(tr.CANFD_RATED_DEG_S)
ACC_DEG_S2 = np.full(7, 600.0)
JERK_DEG_S3 = np.full(7, 1500.0 * 180.0 / np.pi / 10.0)   # 1500 rad/s^3 is the planned-lane figure
#: Piece length for the scalar solve, mm. Short enough that constant caps per piece are honest.
PIECE_MM = 10.0


# ───────────────────────────────────────────────────────────── the path as q(s)
class TaughtPath:
    """A taught polyline as a differentiable path q(s), s = TCP arc length in mm."""

    def __init__(self, q_deg: np.ndarray, side: str = SIDE, frame: str = FRAME):
        self.q_src = np.asarray(q_deg, float)
        P, _ = cd.fk_tool(self.q_src, side, frame)
        if P is None:
            raise SystemExit("FK unavailable — cannot define s as TCP arc length")
        self.P_src = P * 1000.0
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(self.P_src, axis=0), axis=1))])
        # a held sample repeats s; the spline needs a strictly increasing parameter
        keep = np.concatenate([[True], np.diff(s) > 1e-9])
        self.s = s[keep]
        self._cs = CubicSpline(self.s, self.q_src[keep], axis=0)
        self.kept = keep

    def q(self, s):
        return self._cs(np.clip(s, self.s[0], self.s[-1]))

    def dq(self, s):
        return self._cs(np.clip(s, self.s[0], self.s[-1]), 1)        # deg per mm

    def ddq(self, s):
        return self._cs(np.clip(s, self.s[0], self.s[-1]), 2)

    def total(self):
        return float(self.s[-1])


# ─────────────────────────────────────────────── the demand: speed along the path
def demand(kind: str, path: TaughtPath):
    """A desired TCP speed profile v*(s), mm/s, from a short spec.

    `taught`            the recording's own speed (the identity demand)
    `flat:<x>`          x times the taught speed everywhere
    `bothways:<lo>:<hi>` the slot-68 design: the HIGH-acceleration stretches slowed to lo, the
                        LOW-acceleration stretches sped up to hi, so one run moves the demand in
                        BOTH directions and a per-run intercept cannot explain the result.
    """
    ds = np.gradient(path.s)
    v_taught = np.full_like(path.s, np.nan)
    # the taught speed at each kept sample: one source interval per DT of recording
    v_taught = ds / tt.CLOCK_RECORD_S
    v_taught = np.maximum(v_taught, 1.0)
    if kind == "max":
        # NO demand cap: the joint ratings and the acceleration limit are the only ceilings, so
        # the parameteriser is free to push every point to the fastest the geometry allows. This
        # is the "go as fast as is legal, non-uniformly" case.
        return np.full_like(path.s, 1e6)
    if kind == "taught":
        return v_taught
    if kind.startswith("flat:"):
        return v_taught * float(kind.split(":", 1)[1])
    if kind.startswith("bothways:"):
        _, lo, hi = kind.split(":")
        a = np.abs(np.gradient(v_taught, path.s))                    # along-path accel proxy
        cut = np.percentile(a, 50.0)
        return v_taught * np.where(a > cut, float(lo), float(hi))
    raise SystemExit(f"unknown --profile {kind!r}")


def arclen_smooth(path: "TaughtPath", lam_mm: float, ds: float = 0.5) -> np.ndarray:
    """DENOISE IN THE ARC-LENGTH DOMAIN: a low-pass whose cutoff is in 1/mm, so the smoothing
    scale is constant in millimetres everywhere regardless of how fast the hand was moving.

    This is the half that makes a time-optimal pass worth running. Measured 2026-09-21: the
    binding constraint on a taught path is its own curvature — |d2q/ds2| v^2 reaches the 600
    deg/s^2 limit at p95 while the joint SPEED ratings still permit ~5x the speed (acceleration
    binds before the ratings on 99.9 % of the bowl path). Lowering |d2q/ds2| is therefore the
    only way to raise the legal speed, and a TEMPORAL filter cannot do it evenly because a
    drag-teach recording's speed varies ~9x within one recording, so one cutoff in Hz is a
    wildly varying cutoff in mm.
    """
    from scipy.signal import butter, filtfilt
    su = np.arange(0.0, path.total(), ds)
    if len(su) < 32:
        return path.q_src[path.kept]
    qi = path.q(su)
    nyq, fc = 0.5 / ds, 1.0 / float(lam_mm)
    if not 0.0 < fc < nyq:
        return path.q_src[path.kept]
    b, a = butter(4, fc / nyq)
    qf = filtfilt(b, a, qi, axis=0, padlen=min(3 * max(len(a), len(b)) * 50, len(qi) - 1))
    # BACK ONTO THE SOURCE KNOTS BY A CUBIC, NOT np.interp (fixed 2026-09-21). Linear
    # interpolation puts a slope kink at every 0.5 mm grid node, and wherever the source knots are
    # bunched -- the operator's creep and the flanks of removed pauses, down to 0.0024 mm apart --
    # the next TaughtPath's cubic spline resolves each kink over that tiny spacing into a curvature
    # spike (|q''| to 28 deg/mm^2). TOPP-RA had to slow for every one: the cubic changes no joint
    # by more than 0.003 deg and made the 30 mm rung 37.12 -> 32.80 s and the 50 mm rung
    # 33.98 -> 30.11 s, and it removed a single-line J6 command of 2600 deg/s^2 from slot 56.
    # Clipped to the grid exactly as before: the last < ds of the path holds qf[-1].
    return CubicSpline(su, qf, axis=0)(np.clip(path.s, su[0], su[-1]))


# ───────────────────────────────────────────────────────── the pull-back onto s
def caps_on_s(path: TaughtPath, s0: float, s1: float):
    """(v_cap, a_cap, j_cap) in mm/s, mm/s^2, mm/s^3 over [s0, s1].

    The joint limits divided by the worst |dq_j/ds| on the piece. First order: it ignores the
    centripetal term ddq*sdot^2 in qddot, exactly as the shipping `ruckig_on_s.cpp` does, so the
    REALISED joint acceleration is measured afterwards instead of being assumed.
    """
    ss = np.linspace(s0, s1, 8)
    g = np.abs(path.dq(ss)).max(axis=0)                              # deg per mm, per joint
    g = np.maximum(g, 1e-9)
    return (float((RATE_DEG_S / g).min()),
            float((ACC_DEG_S2 / g).min()),
            float((JERK_DEG_S3 / g).min()))


class _Piece:
    __slots__ = ("s0", "s1", "v_cap_eff", "a_cap", "j_cap", "v_end", "a_end", "kind")

    def __init__(self, s0, s1, v_cap_eff, a_cap, j_cap, v_end):
        self.s0, self.s1 = float(s0), float(s1)
        self.v_cap_eff, self.a_cap, self.j_cap = float(v_cap_eff), float(a_cap), float(j_cap)
        self.v_end, self.a_end, self.kind = float(v_end), 0.0, "taught"


# ─────────────────────────────────────────────────────────────── the parameterisers
def retime_lambda(path: TaughtPath, v_star: np.ndarray, dt: float = DT,
                  profile_hz: float = 2.0) -> np.ndarray:
    """BACKEND 1 — the shipping approach. dt/du = period/s(u), integrated, resampled.

    Path-exact by construction, bounds nothing; `profile_hz` low-passes the scale as a PROXY for an
    acceleration limit, which is the whole point of the comparison.
    """
    from scipy.signal import butter, filtfilt
    v_taught = np.maximum(np.gradient(path.s) / tt.CLOCK_RECORD_S, 1.0)
    scale = np.clip(v_star / v_taught, 0.05, 20.0)
    if profile_hz and len(scale) > 32:
        b, a = butter(4, min(profile_hz / (0.5 / tt.CLOCK_RECORD_S), 0.99))
        scale = np.clip(filtfilt(b, a, scale), 0.05, 20.0)
    q = path.q_src[path.kept]
    n = len(q)
    u = np.arange(n, dtype=float)
    per = tt.CLOCK_RECORD_S
    step = per / np.maximum(scale, 1e-6)
    t_of_u = np.concatenate([[0.0], np.cumsum(0.5 * (step[:-1] + step[1:]))])
    m = int(np.floor(float(t_of_u[-1]) / dt)) + 1
    u_new = np.interp(np.arange(m) * dt, t_of_u, u)
    return np.column_stack([np.interp(u_new, u, q[:, j]) for j in range(q.shape[1])])


def retime_ruckig(path: TaughtPath, v_star: np.ndarray, dt: float = DT,
                  piece_mm: float = PIECE_MM) -> np.ndarray:
    """BACKEND 3 — Ruckig on the scalar path coordinate s. Bounds v, a AND jerk.

    The CANFD channel's own parameteriser (`channel_policy.yaml`), reached through exp02's
    `ruckig_s.time_path`, which is the original the shipping C++ was ported from.
    """
    from exp02 import ruckig_s                                        # the C++'s own original
    edges = np.arange(0.0, path.total(), float(piece_mm))
    edges = np.append(edges, path.total())
    raw = []
    for k in range(len(edges) - 1):
        s0, s1 = edges[k], edges[k + 1]
        if s1 - s0 < 1e-6:
            continue
        v_cap, a_cap, j_cap = caps_on_s(path, s0, s1)
        want = float(np.interp(0.5 * (s0 + s1), path.s, v_star))
        raw.append(dict(s0=s0, s1=s1, v_cap=min(v_cap, want), a_cap=a_cap, j_cap=j_cap))
    if not raw:
        raise RuntimeError("no pieces")

    # THE BACKWARD PASS, which per-piece Ruckig does not do for you and without which this fails.
    # Each boundary speed must be (a) under BOTH adjoining caps and (b) slow enough to brake to the
    # next boundary inside the next piece: v_k^2 <= v_{k+1}^2 + 2*a*L. Ruckig accelerates forward
    # happily, but asked to enter a slower piece too fast it brakes past zero and the profile
    # reverses ("scalar profile reverses in piece 28: sdot_min=-108.6"), which is a REFUSAL, not a
    # bad trajectory. TOPP-RA has this pass built in; this is the equivalent for the scalar solve.
    n = len(raw)
    v_end = [min(raw[k]["v_cap"], raw[k + 1]["v_cap"]) for k in range(n - 1)] + [0.0]
    for k in range(n - 2, -1, -1):
        L = raw[k + 1]["s1"] - raw[k + 1]["s0"]
        reach = float(np.sqrt(max(v_end[k + 1] ** 2 + 2.0 * raw[k + 1]["a_cap"] * L, 0.0)))
        v_end[k] = min(v_end[k], reach)

    # THE BACKWARD PASS ABOVE IS JERK-BLIND, and that is not a detail I can wave away. It uses
    # v^2 <= v_next^2 + 2*a*L, which is the bound for UNLIMITED jerk; under a finite j_cap the
    # distance needed to change speed is strictly greater, so a boundary speed it declares
    # reachable can still be infeasible. Ruckig then returns a profile that undershoots the
    # position target and comes back — `sdot_min < 0` — which `time_path` raises as a REFUSAL.
    # A jerk-aware closed-form bound does not exist for the 7-phase profile, so the honest thing
    # is to relax the boundary speeds until the solve succeeds AND REPORT BY HOW MUCH, because
    # that factor is the price of applying a per-piece scalar solver to a 19k-sample recording
    # chopped into arbitrary pieces. It is not free and it should not look free.
    relax, prof, err = 1.0, None, None
    for _ in range(14):
        pieces = [_Piece(r["s0"], r["s1"], r["v_cap"], r["a_cap"], r["j_cap"], v_end[k] * relax)
                  for k, r in enumerate(raw)]
        try:
            prof = ruckig_s.time_path(pieces, dt=dt, s_start=0.0, v_start=0.0)
            break
        except RuntimeError as e:                                     # noqa: PERF203
            err = e
            relax *= 0.85
    if prof is None:
        raise RuntimeError(f"Ruckig-on-s could not time this path even at {relax:.3f} of the "
                           f"backward-pass boundary speeds — last: {err}")
    retime_ruckig.last_relax = relax
    return path.q(prof.s)


def retime_toppra(path: TaughtPath, v_star: np.ndarray, dt: float = DT,
                  vcap: np.ndarray | None = None) -> np.ndarray:
    """BACKEND 2 — TOPP-RA, time-optimal under velocity AND acceleration constraints.

    The `track` channel's own parameteriser. The demand enters as a per-point VELOCITY bound, so
    TOPP-RA returns the fastest profile that honours both it and the joint limits. `vcap`, one row
    of seven joint caps (deg/s) per kept knot, is taken under both as well — emit_toppra_rung.py's
    arrival hold.
    """
    import toppra as ta
    import toppra.algorithm as algo
    import toppra.constraint as constraint
    q = path.q_src[path.kept]
    total = path.total()
    # PARAMETERISE BY ARC LENGTH, normalised. Then x in [0,1], s = x*total, and the chain rule is
    # one factor: dq/dx = (dq/ds)*total, and a TCP speed v corresponds to xdot = v/total.
    x = path.s / total
    ppath = ta.SplineInterpolator(x, q)
    dq_dx = np.abs(path.dq(path.s)) * total                           # deg per unit x, per joint
    # The demand is a TCP SPEED, so pull it back to a JOINT velocity bound, and take whichever of
    # the demand and the joint's own rating binds first at each point.
    vlim = np.minimum(RATE_DEG_S[None, :], dq_dx * (v_star[:, None] / total))
    if vcap is not None:
        vlim = np.minimum(vlim, np.asarray(vcap, float))
    vlim = np.maximum(vlim, 1e-3)

    def vlim_at(xx):
        return np.column_stack([-np.array([np.interp(xx, x, vlim[:, j]) for j in range(7)]),
                                np.array([np.interp(xx, x, vlim[:, j]) for j in range(7)])])

    pc_vel = constraint.JointVelocityConstraintVarying(vlim_at)
    # INTERPOLATION, NOT COLLOCATION. The default discretisation enforces an acceleration
    # constraint only AT the grid points, and between them the profile sails past it: measured
    # 2026-09-21, TOPP-RA's own analytic qddot peaked at 1007.7 deg/s^2 against the 600 it was
    # given, 1.68x, with no warning. `Interpolation` is the conservative scheme that holds the
    # bound across each interval, and it is what makes the returned trajectory actually legal.
    pc_acc = constraint.JointAccelerationConstraint(
        ACC_DEG_S2, discretization_scheme=constraint.DiscretizationType.Interpolation)
    inst = algo.TOPPRA([pc_vel, pc_acc], ppath, solver_wrapper="seidel")
    traj = inst.compute_trajectory(0, 0)
    if traj is None:
        raise RuntimeError("TOPP-RA returned no trajectory for this demand")
    return traj(np.arange(0.0, float(traj.duration), dt))


# ──────────────────────────────────────────────────────────────────── the scoring
def perp_mm(Q: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Perpendicular distance from each FK(Q) to the taught ik_frame polyline, mm."""
    P, _ = cd.fk_tool(Q, SIDE, FRAME)
    P = P * 1000.0
    A, B = poly[:-1], poly[1:]
    AB = B - A
    L2 = np.einsum("ij,ij->i", AB, AB)
    L2[L2 == 0] = 1e-18
    out = np.empty(len(P))
    for i, p in enumerate(P):
        t = np.clip(np.einsum("ij,ij->i", p - A, AB) / L2, 0.0, 1.0)
        d = (A + t[:, None] * AB) - p
        out[i] = np.sqrt(np.einsum("ij,ij->i", d, d).min())
    return out


#: WHY THERE IS NO BAND-LIMIT HERE, though the house rule says band-limit at 20 Hz before
#: differencing. That rule is for RECORDED TELEMETRY, which carries sensor noise and millideg
#: quantisation — there, one LSB at 5 ms is 40 deg/s^2 and 8000 deg/s^3, so a raw third difference
#: is mostly noise. What is differenced HERE is a SYNTHESISED float trajectory, before any
#: rounding: it has no noise, the finite difference is exact, and it is the acceleration each
#: parameteriser actually designed. Filtering it does active harm — measured 2026-09-21 on the
#: TOPP-RA result, a 4th-order 20 Hz zero-phase filter put its acceleration PEAK at 0.6 % through
#: the run, which is the filter ringing at the start transient, not the motion.
#: (The arm does receive millideg-rounded positions, so the DELIVERED acceleration has quantisation
#: in it and would need the rule. That is a different quantity from the DESIGNED one compared here.)


def score(tag: str, Q: np.ndarray, path: TaughtPath, v_star: np.ndarray, dt: float = DT) -> dict:
    """What each backend actually delivered: fidelity to the demand, path error, and the
    commanded joint rate / acceleration / jerk against the limits it was supposed to honour.

    Everything is judged on the RAW float samples: the rate because that is literally what the wire
    carries and what the CANFD gate measures, the acceleration and jerk because these are
    synthesised trajectories with no noise to remove (see the note above `score`)."""
    step = np.abs(np.diff(Q, axis=0))
    rate = step / dt
    P, _ = cd.fk_tool(Q, SIDE, FRAME)
    P = P * 1000.0
    sv = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    v = np.gradient(sv, dt)                                           # achieved TCP speed, mm/s
    acc = np.diff(Q, n=2, axis=0) / (dt ** 2)
    jrk = np.diff(Q, n=3, axis=0) / (dt ** 3)
    # FIDELITY IS JUDGED AGAINST A SMOOTHED DEMAND, because the raw one is not deliverable and
    # measuring against it says more about the demand than about the backend. v*(s) is built from
    # the recording's own ds/dt, which is ragged, and `bothways` switches on a percentile cut, so it
    # carries steps. No parameteriser can follow a step in speed — a step in v is an unbounded
    # acceleration, which is the thing being limited. The fair question is whether the backend
    # tracks the demand's SHAPE, so both are smoothed over the same 50 mm of path before comparing,
    # and the unsmoothed version is reported next to it so the gap is visible.
    from scipy.ndimage import uniform_filter1d
    sv_c = np.clip(sv, path.s[0], path.s[-1])
    want = np.interp(sv_c, path.s, v_star)
    w = max(int(50.0 / max(np.median(np.diff(sv)), 1e-6)), 3)
    want_s = uniform_filter1d(want, w, mode="nearest")
    v_s = uniform_filter1d(v, w, mode="nearest")
    m = want_s > 5.0
    dev = perp_mm(Q, path.P_src)
    return dict(
        tag=tag, points=len(Q), duration_s=(len(Q) - 1) * dt,
        dev_p95_mm=float(np.percentile(dev, 95)), dev_max_mm=float(dev.max()),
        speed_err_p95=float(np.percentile(np.abs(v_s[m] / want_s[m] - 1.0), 95) * 100) if m.any() else np.nan,
        speed_err_raw_p95=float(np.percentile(np.abs(v[m] / want[m] - 1.0), 95) * 100) if m.any() else np.nan,
        v_p50=float(np.median(v[m])) if m.any() else np.nan,
        want_p50=float(np.median(want[m])) if m.any() else np.nan,
        rate_peak=float(rate.max()), rate_util=float((rate / RATE_DEG_S[None, :]).max()),
        over_flat180=int((rate[:, 2:] > 180.0).sum()),
        over_rated=int((rate > RATE_DEG_S[None, :]).sum()),
        acc_peak=float(np.abs(acc).max()), acc_util=float((np.abs(acc) / ACC_DEG_S2[None, :]).max()),
        jerk_peak=float(np.abs(jrk).max()),
        jerk_util=float((np.abs(jrk) / JERK_DEG_S3[None, :]).max()),
        step_max=float(step.max()),
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--report", action="store_true", help="score all three backends (writes nothing)")
    ap.add_argument("--build", choices=("lambda", "toppra", "ruckig"), help="emit one track")
    ap.add_argument("--out", type=pathlib.Path, help="where --build writes")
    ap.add_argument("--profile", default="bothways:0.70:1.40",
                    help="the speed demand: taught | flat:<x> | bothways:<lo>:<hi>")
    ap.add_argument("--rung", default=SRC_RUNG)
    ap.add_argument("--dt", type=float, default=DT * 1e3, help="stream period, ms")
    args = ap.parse_args()
    dt = float(args.dt) / 1e3

    src = SRC_DIR / args.rung
    if not src.is_file():
        raise SystemExit(f"no such rung: {src}")
    path = TaughtPath(tt.read_q_deg(src))
    v_star = demand(args.profile, path)
    print(f"source     {src.relative_to(REPO)}")
    print(f"path       {len(path.s)} samples, {path.total():.1f} mm at the ik_frame ({FRAME})")
    print(f"spline     interpolating cubic in s; departure from the taught polyline "
          f"p95 {np.percentile(perp_mm(path.q(path.s), path.P_src), 95):.4f} mm "
          f"max {perp_mm(path.q(path.s), path.P_src).max():.4f} mm")
    print(f"demand     {args.profile}: v* p10 {np.percentile(v_star, 10):.0f} "
          f"p50 {np.median(v_star):.0f} p90 {np.percentile(v_star, 90):.0f} mm/s")
    print(f"period     {dt * 1e3:.4f} ms\n")

    backends = [("lambda", retime_lambda), ("toppra", retime_toppra), ("ruckig_on_s", retime_ruckig)]
    if args.build:
        pick = {"lambda": retime_lambda, "toppra": retime_toppra, "ruckig": retime_ruckig}[args.build]
        Q = pick(path, v_star, dt)
        out = args.out or (SRC_DIR / f"local_{args.build}.txt")
        tt.write_track(out, np.round(Q * 1000.0).astype(np.int64))
        print(f"wrote {out}  ({len(Q)} lines)")
        return 0

    rows = []
    for tag, fn in backends:
        try:
            Q = fn(path, v_star, dt)
            r = score(tag, Q, path, v_star, dt)
            r["relax"] = getattr(fn, "last_relax", 1.0)
            rows.append(r)
        except Exception as e:                                        # noqa: BLE001
            print(f"  {tag:12s} FAILED: {type(e).__name__}: {str(e)[:160]}")
    if not rows:
        return 1
    hdr = (f"{'backend':12s} {'pts':>6s} {'dur_s':>7s} {'devp95':>7s} {'devmax':>7s} "
           f"{'v_err%':>6s} {'raw%':>6s} | {'rate':>6s} {'util':>5s} {'>180':>5s} {'>rat':>5s} | "
           f"{'accel':>6s} {'util':>5s} | {'jerk':>8s} {'util':>6s}")
    print(hdr); print("-" * len(hdr))
    for r in rows:
        print(f"{r['tag']:12s} {r['points']:6d} {r['duration_s']:7.2f} {r['dev_p95_mm']:7.3f} "
              f"{r['dev_max_mm']:7.3f} {r['speed_err_p95']:6.1f} {r['speed_err_raw_p95']:6.1f} | "
              f"{r['rate_peak']:6.1f} {r['rate_util']:5.3f} {r['over_flat180']:5d} "
              f"{r['over_rated']:5d} | {r['acc_peak']:6.0f} {r['acc_util']:5.2f} | "
              f"{r['jerk_peak']:8.0f} {r['jerk_util']:6.2f}"
              + (f"   relaxed to {r['relax']:.2f}" if r.get("relax", 1.0) < 0.999 else ""))
    print("-" * len(hdr))
    print("devp95/max = perpendicular departure from the TAUGHT ik_frame polyline, mm")
    print("v_err%  = p95 |achieved/demanded - 1| of TCP speed, both smoothed over 50 mm of path")
    print("raw%    = the same against the UNSMOOTHED demand (a step in v is an unbounded accel,")
    print("          so no parameteriser can follow it; the gap between the two is that fact)")
    print("rate/accel/jerk = exact differences of the SYNTHESISED float samples (no noise to")
    print("          filter; these are what each parameteriser designed). util = of the limit")
    print(f"limits: rate {RATE_DEG_S.tolist()}, accel {ACC_DEG_S2[0]:g}, jerk {JERK_DEG_S3[0]:.0f} deg/s^3")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
