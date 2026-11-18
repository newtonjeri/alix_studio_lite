"""RUCKIG ON S: the same path preparation as Time-optimal, timed by Ruckig on the path coordinate,
so the speed along the path is jerk-limited by construction.

1. to 4. the path exactly as ``toppra_opt`` prepares it (5 Hz filter, pause removal, arc-length
   smoothing, ends put back);
5. TOPP-RA's speed along that path, within the joint speeds, the acceleration limit, the arrival
   hold and the UGV load limits: the fastest speed the limits allow at every point. It prices the
   path's curvature, which a first-order pull-back of the joint limits onto s does not: ALIX's own
   taught-path trial of Ruckig on s (``local_speed.retime_ruckig``) read 4.5 to 7.8 x the
   acceleration limit on Lite's recordings (measured 18 Nov) for that reason;
6. the path coordinate is then the joints' travel, not the hand's (``toppra_opt.JointCoordinate``):
   along it the joint limits pulled back onto the coordinate are nearly constant, where along the
   hand's travel a wrist turn with the hand almost still makes them collapse;
7. the path is cut at every dip of that speed. A dip is where the path forces the arm to slow, so
   the acceleration along the path is zero there, and each stretch between two dips is one Ruckig
   move (Community, one degree of freedom, as ALIX's ``ruckig_s.time_path``): from the speed at one
   dip to the speed at the next, under the stretch's own speed, acceleration and jerk caps (the
   joint limits pulled back onto the coordinate). The speeds at the dips are first made reachable
   under the jerk limit, forwards and backwards, with the exact S-curve distance;
8. the timed file is read as the checks read it. Where a joint's speed, acceleration or jerk, or the
   UGV load, is over its limit, that stretch's caps are tightened and the timing is made again
   (ALIX's stroke pipeline does the same: tighten the offending pieces, re-time);
9. sampled every 5 ms, low-passed where asked, the ends kept, as Time-optimal's file.

Needs the ``ruckig`` and ``toppra`` packages; ``available()`` is False without them.
"""
from __future__ import annotations

import math

import numpy as np

from ..motion import track
from ..motion.checks import acc_deg_s2, jerk_deg_s3, joint_turns, speed_deg_s, ugv_load, util_series
from ..motion.limits import LimitSet
from . import toppra_opt
from .types import ToppraOptions

UNAVAILABLE_TEXT = "Ruckig on s is not available on this computer (ruckig or toppra is not installed)."
ENVELOPE_SHARE = 0.98          # the speeds aimed for sit this far under TOPP-RA's
DIP_SHARE = 0.01               # a dip counts when the speed falls by this share of itself
MIN_SPAN = 0.004               # the shortest stretch, in the path coordinate's seconds
TIGHTEN = 0.8                  # a stretch over a limit has its acceleration and jerk caps cut to this share
TIGHTEN_SPEED = 0.93
MAX_ROUNDS = 16


def available() -> bool:
    try:
        import ruckig  # noqa: F401
    except Exception:                                                      # noqa: BLE001
        return False
    return toppra_opt.available()


def ruckig_version() -> str | None:
    try:
        from importlib.metadata import version

        return version("ruckig")
    except Exception:                                                      # noqa: BLE001
        return None


def scurve_dist(v0: float, v1: float, a: float, j: float) -> float:
    """The shortest distance in which the speed can go from ``v0`` to ``v1`` with the acceleration
    zero at both ends, under the acceleration cap ``a`` and the jerk cap ``j``."""
    dv = abs(v1 - v0)
    if dv <= 0.0:
        return 0.0
    t = dv / a + a / j if dv >= a * a / j else 2.0 * math.sqrt(dv / j)
    return 0.5 * (v0 + v1) * t


def reach(v0: float, length: float, a: float, j: float, v_cap: float) -> float:
    """The highest speed, up to ``v_cap``, reachable from ``v0`` within ``length``."""
    if v_cap <= v0 or scurve_dist(v0, v_cap, a, j) <= length:
        return max(v_cap, v0) if v_cap > v0 else v0
    lo, hi = v0, v_cap
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if scurve_dist(v0, mid, a, j) <= length:
            lo = mid
        else:
            hi = mid
    return lo


def cuts(s: np.ndarray, e: np.ndarray, also=()) -> np.ndarray:
    """Where the path is cut into stretches: its two ends, the speed's dips, and the positions in
    ``also`` (the edges of every arrival hold: the speed is held from one edge to the other, so
    the arm has to be at the held speed already when it enters)."""
    from scipy.signal import find_peaks

    idx, _ = find_peaks(-e, prominence=DIP_SHARE * np.maximum(e, 1e-9))
    inner = np.sort(np.concatenate([s[idx], np.asarray(also, dtype=float)]))
    out = [float(s[0])]
    for x in inner:
        if x - out[-1] >= MIN_SPAN and s[-1] - x >= MIN_SPAN:
            out.append(float(x))
    out.append(float(s[-1]))
    return np.array(out)


def hold_edges(path, caps, vlim) -> np.ndarray:
    """The path positions (mm) where an arrival hold starts and where it ends."""
    if caps is None:
        return np.zeros(0)
    out = []
    for j in range(7):
        on = (np.asarray(caps)[:, j] < vlim[j] - 1e-9).astype(int)
        for i in np.nonzero(np.diff(np.concatenate([[0], on, [0]])))[0]:
            out.append(float(path.s[min(int(i), len(path.s) - 1)]))
    return np.array(out)


def time_spans(coord, bounds_s, vb, v_cap, a_cap, j_cap, fine_dt: float = 0.00125):
    """One Ruckig move per stretch, chained. Returns the timing, or the index of a stretch whose
    move backs up (its end speed was not reachable after all)."""
    import ruckig

    otg = ruckig.Ruckig(1)
    ts, ss, vs, starts = [0.0], [float(bounds_s[0])], [float(vb[0])], [0.0]
    t0 = 0.0
    for k in range(len(bounds_s) - 1):
        s0, s1 = float(bounds_s[k]), float(bounds_s[k + 1])
        inp = ruckig.InputParameter(1)
        inp.current_position, inp.current_velocity, inp.current_acceleration = [s0], [float(vb[k])], [0.0]
        inp.target_position, inp.target_velocity, inp.target_acceleration = [s1], [float(vb[k + 1])], [0.0]
        inp.max_velocity = [max(float(v_cap[k]), float(vb[k]), float(vb[k + 1])) * (1.0 + 1e-9) + 1e-9]
        inp.max_acceleration, inp.max_jerk = [float(a_cap[k])], [float(j_cap[k])]
        traj = ruckig.Trajectory(1)
        res = otg.calculate(inp, traj)
        if res not in (ruckig.Result.Working, ruckig.Result.Finished):
            return None, k
        ext = traj.position_extrema[0]
        if ext.min < s0 - 1e-6 or ext.max > s1 + 1e-6:
            return None, k
        d = float(traj.duration)
        n = max(int(math.ceil(d / fine_dt)), 2)
        for i in range(1, n + 1):
            p, v, _ = traj.at_time(d * i / n)
            ts.append(t0 + d * i / n)
            ss.append(p[0])
            vs.append(max(v[0], 0.0))
        t0 += d
        starts.append(t0)
    return toppra_opt.PathTiming(coord, np.array(ts), np.array(ss), np.array(vs), np.array(starts)), None


class RuckigPlan:
    optimiser = "ruckig_s"

    def __init__(self, md_raw: np.ndarray, limits: LimitSet, opts: ToppraOptions):
        if not available():
            raise RuntimeError(UNAVAILABLE_TEXT)
        self.limits = limits
        self.opts = opts
        path, info = toppra_opt.prepared(md_raw, limits, opts)
        vlim, acc, caps, held, load = toppra_opt.design_limits(path, limits, opts)
        rating = np.asarray(limits.rating_deg_s, dtype=float)
        self.held_caps = caps
        self._build(path, info, vlim, acc, caps, held, load, limits.rate_bar * rating, np.full(7, float(limits.acc_deg_s2)))

    @classmethod
    def on_path(cls, path, limits: LimitSet, opts: ToppraOptions, vlim, acc, speed_mm_s=None) -> "RuckigPlan":
        """The same timing on a path that is already what it should be (a planned move: nothing is
        smoothed or removed), within the joint speeds ``vlim`` and accelerations ``acc`` given, which
        are also what the file is then read against. ``speed_mm_s`` is a ceiling on the hand's speed
        along the path (mm/s, one value or one per knot of the path): a planned move's own top hand
        speed, as ALIX's stroke timing takes a line speed."""
        if not available():
            raise RuntimeError(UNAVAILABLE_TEXT)
        self = cls.__new__(cls)
        self.limits, self.opts = limits, opts
        vlim, acc = np.asarray(vlim, dtype=float), np.asarray(acc, dtype=float)
        caps, held = None, []
        if opts.arrival_hold:
            import emit_toppra_rung as ETR

            caps, held = ETR.arrival_hold(path.q_src[path.kept], vlim, float(opts.arrival_hold), model=limits.model)
        self.held_caps = caps
        if speed_mm_s is not None:                                 # each joint's own speed at that hand speed
            top = np.broadcast_to(np.asarray(speed_mm_s, dtype=float), path.s.shape)
            mine = np.abs(path.dq(path.s)) * top[:, None]
            caps = mine if caps is None else np.minimum(caps, mine)
        ease = limits.rate_bar / limits.rate_design                # read with the same margin the design leaves
        self._build(path, {"path_mm": round(path.total(), 1)}, vlim, acc, caps, held,
                    toppra_opt.ugv_load_constraint(limits), vlim * ease, acc)
        return self

    def _build(self, path, info, vlim, acc, caps, held, load, v_bar, a_bar) -> None:
        limits = self.limits
        self.path = path
        self.v_bar, self.a_bar = v_bar, a_bar                      # what the timed file is read against
        s_mm, e_mm = toppra_opt.solve_profile(path, vlim, acc, vcap=caps, extra=(load,))
        self.coord = coord = toppra_opt.JointCoordinate(path, vlim)
        s, e = coord.sigma(s_mm), e_mm * coord.rate(s_mm)      # the same speed, along the joints' travel
        keep = np.concatenate([[True], np.diff(s) > 1e-12])
        s, e = s[keep], e[keep]
        jerk = np.full(7, math.degrees(limits.jerk_max_rad_s3 or limits.jerk_ref_rad_s3))
        bs = self.bounds_s = cuts(s, e, coord.sigma(hold_edges(path, self.held_caps, vlim)))
        n = len(bs) - 1
        v_peak, a_first, j_first = np.empty(n), np.empty(n), np.empty(n)
        for k in range(n):
            inside = (s > bs[k]) & (s < bs[k + 1])
            g = np.maximum(np.abs(coord.dq(np.linspace(bs[k], bs[k + 1], 16))).max(axis=0), 1e-9)
            v_peak[k] = ENVELOPE_SHARE * float(max(e[inside].max(initial=0.0), np.interp(bs[k:k + 2], s, e).max()))
            a_first[k] = float((acc / g).min())
            j_first[k] = float((jerk / g).min())
        vb_env = ENVELOPE_SHARE * np.interp(bs, s, e)
        vb_env[0] = vb_env[-1] = 0.0
        gv, ga = np.ones(n), np.ones(n)
        timing, rounds, left = None, 0, 0
        for rounds in range(1, MAX_ROUNDS + 1):
            timing = self._time(vb_env, gv * v_peak, ga * a_first, ga * j_first)
            over_speed, over_rest = self._over(timing)
            left = int(over_speed.sum() + over_rest.sum())
            if not left:
                break
            ga[over_rest] *= TIGHTEN
            gv[over_rest | over_speed] *= TIGHTEN_SPEED
        self.timing = timing
        self.T = timing.T
        self.n_floor = max(1, math.ceil(self.T / limits.dt_gate - 1e-9))
        self.n_start = self.n_floor
        self.n_cap = max(20 * self.n_floor, 100)
        self.info = dict(info, t_opt_s=round(self.T, 4), t_toppra_s=round(float(self._toppra_time(s, e)), 4),
                         arrival_holds=len(held), stretches=n, rounds=rounds, stretches_left_over=left,
                         jerk_design_rad_s3=float(limits.jerk_max_rad_s3 or limits.jerk_ref_rad_s3))

    @staticmethod
    def _toppra_time(s: np.ndarray, e: np.ndarray) -> float:
        """The time of TOPP-RA's own profile (constant acceleration between its grid points)."""
        return float((2.0 * np.diff(s) / np.maximum(e[:-1] + e[1:], 1e-9)).sum())

    def _time(self, vb_env, v_cap, a_cap, j_cap):
        bs = self.bounds_s
        n = len(bs) - 1
        vb = vb_env.copy()
        for _ in range(40):                                    # until no move backs up
            for k in range(1, n):
                vb[k] = min(vb[k], v_cap[k - 1], v_cap[k])
            for k in range(n):                                 # reachable going forwards
                vb[k + 1] = min(vb[k + 1], reach(vb[k], bs[k + 1] - bs[k], a_cap[k], j_cap[k], max(vb[k + 1], vb[k])))
            for k in range(n - 1, -1, -1):                     # and able to slow in time
                vb[k] = min(vb[k], reach(vb[k + 1], bs[k + 1] - bs[k], a_cap[k], j_cap[k], max(vb[k], vb[k + 1])))
            timing, bad = time_spans(self.coord, bs, vb, v_cap, a_cap, j_cap)
            if timing is not None:
                return timing
            hi = bad if vb[bad] > vb[bad + 1] else bad + 1     # the faster end of the stretch that backed up
            vb[hi] *= 0.9
        raise RuntimeError("Ruckig on s found no timing for this path")

    def _over(self, timing) -> tuple[np.ndarray, np.ndarray]:
        """Which stretches are over a limit in the file as the checks read it: (speed, the rest)."""
        lim, dt = self.limits, self.limits.dt_gate
        n = max(1, math.ceil(timing.T / dt - 1e-9))
        t = np.arange(n + 1) * (timing.T / n)
        q = toppra_opt.post_filter(timing.q(t), self.opts.post_lp_hz, dt)
        spans = len(timing.bounds_t) - 1
        where = lambda rows, shift: np.clip(np.searchsorted(timing.bounds_t, t[np.minimum(rows + shift, n)],   # noqa: E731
                                                            side="right") - 1, 0, spans - 1)
        over_speed, over_rest = np.zeros(spans, bool), np.zeros(spans, bool)
        over_speed[where(np.nonzero((speed_deg_s(q, dt) > self.v_bar).any(axis=1))[0], 1)] = True
        late = np.array([tn["line"] for tn in joint_turns(q, lim) if not tn["geometry"] and tn["util"] > 1.0], dtype=int)
        if late.size:                                            # a turn arrived at too fast: slow its approach
            over_speed[where(late, 0)] = True
            over_speed[where(np.maximum(late - int(round(0.15 / dt)), 0), 0)] = True
        over_rest[where(np.nonzero((acc_deg_s2(q, dt) > self.a_bar).any(axis=1))[0], 1)] = True
        if lim.jerk_max_rad_s3 is not None:
            jl = math.degrees(lim.jerk_max_rad_s3)
            over_rest[where(np.nonzero((jerk_deg_s3(q, dt) > 0.98 * jl).any(axis=1))[0], 2)] = True
        if lim.ugv_limited:
            force, moment = ugv_load(q, lim)
            bad = np.zeros(len(q), bool)
            if lim.ugv_force_n is not None:
                bad |= force > 0.98 * lim.ugv_force_n
            if lim.ugv_moment_nm is not None:
                bad |= moment > 0.98 * lim.ugv_moment_nm
            over_rest[where(np.nonzero(bad)[0], 0)] = True
        return over_speed, over_rest

    def make(self, n: int) -> np.ndarray:
        q = self.timing.sample(max(int(n), self.n_floor))
        return track.to_md(toppra_opt.post_filter(q, self.opts.post_lp_hz, self.limits.dt_gate))

    def util(self, n: int) -> dict:
        return util_series(self.make(n), self.limits)

    def recipe(self) -> dict:
        return dict(self.info)
