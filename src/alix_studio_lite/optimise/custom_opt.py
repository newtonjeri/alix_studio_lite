"""LITE'S OWN optimiser: the fastest timing along the prepared path with the joint jerk and the UGV
load as limits inside the solve, next to the joint speeds and the acceleration.

The path coordinate s is the joints' travel (``toppra_opt.JointCoordinate``), which a plain grid
resolves where the hand's travel does not. The unknown is the squared speed along it,
b(s) = sdot^2, on a grid of path positions. In it:

- a joint's speed is q' sqrt(b): a ceiling on b;
- the acceleration along the path is b'/2, so a joint's acceleration q' b'/2 + q'' b is a straight
  line in b, and so is the UGV load A b'/2 + B b (``base_load.ArmLoad.path_coeffs``);
- a joint's jerk is sdot (q' b''/2 + 3 q'' b'/2 + q''' b): a straight line in b once sdot is taken
  from the previous pass. The solve holds the first two terms, the jerk the TIMING makes. The
  third is the path's own shape played at speed; its analytic value is dominated by texture far
  above the 20 Hz the checks read (it read 400 x the limit on a recording that passes), so it is
  held by the reading below: where the file reads over the jerk limit, the speed is cut there.

So each pass is a linear programme (scipy's HiGHS: no further package), which shortens the time,
linearised about the previous pass, within all of those limits at once. The first pass starts
from TOPP-RA's speed. From rest the jerk limit allows b to grow no faster than the constant-jerk
start, b <= 2.726 J^(2/3) s^(4/3), and the same into the stop: that bound, on a grid refined
towards both ends, is what makes the start and the stop jerk-limited.

After each pass the timed file is read as the checks read it (sampled every 5 ms, low-passed
where asked, 20 Hz band-limited readings). Where a reading is over its limit, that limit is
tightened around that spot only, and the next pass solves again. So a spike the grid does not see
slows the arm there, not over the whole file. What is still over after the last pass is left to
the engine, which slows the file uniformly.

Needs ``toppra`` (the path's starting speed); ``available()`` is False without it.
"""
from __future__ import annotations

import math

import numpy as np

from ..motion import base_load, track
from ..motion.checks import acc_deg_s2, jerk_deg_s3, joint_turns, speed_deg_s, ugv_load, util_series
from ..motion.limits import LimitSet
from . import toppra_opt
from .types import ToppraOptions

UNAVAILABLE_TEXT = "Lite's own optimiser is not available on this computer (toppra is not installed)."
GRID_S = 0.003                 # grid spacing along the path, in the coordinate's seconds (0.6 deg of the fastest joint)
GRID_MAX = 2000                # most grid points (a longer path gets a wider spacing)
END_FIRST_S = 1e-5             # the first spacing at each end; it grows by END_GROWTH up to the grid's
END_GROWTH = 1.6
PASSES = 10
SETTLED = 0.003                # stop when a pass changes the time by less than this share and nothing reads over
ACC_SHARE = 0.97               # the limits are held a little inside, for what lies between grid points
JERK_SHARE = 0.90
TIGHTEN = 0.88                 # a limit read over is cut to this share around the spot
TIGHTEN_SPEED = 0.94           # and, for a jerk reading, the speed there to this share
WINDOW_S = 0.04                # "around the spot": the grid points within this time of the reading
START_C = 6.0 ** (4.0 / 3.0) / 4.0


def available() -> bool:
    return toppra_opt.available()


def grid(total: float) -> np.ndarray:
    """Path positions from 0 to ``total``, refined towards both ends."""
    h = max(GRID_S, total / GRID_MAX)
    ramp, step = [0.0], END_FIRST_S
    while step < h and ramp[-1] + step < 0.25 * total:
        ramp.append(ramp[-1] + step)
        step *= END_GROWTH
    ramp = np.array(ramp)
    inner = np.linspace(ramp[-1], total - ramp[-1], max(int(round((total - 2 * ramp[-1]) / h)), 2) + 1)
    return np.concatenate([ramp[:-1], inner, (total - ramp[:-1])[::-1]])


def near(t_nodes: np.ndarray, times, window: float) -> np.ndarray:
    """Which grid points are within ``window`` seconds of any of ``times``."""
    times = np.sort(np.asarray(times, dtype=float))
    if not times.size:
        return np.zeros(len(t_nodes), dtype=bool)
    i = np.searchsorted(times, t_nodes)
    gap = np.minimum(np.abs(t_nodes - times[np.clip(i - 1, 0, times.size - 1)]),
                     np.abs(times[np.clip(i, 0, times.size - 1)] - t_nodes))
    return gap <= window


def solve(coord, limits: LimitSet, vlim, acc, jerk, caps, s_env, e_env, read=None) -> tuple:
    """(the timing, what the solve did). ``s_env``, ``e_env``: the speed the first pass starts from,
    along the same coordinate. ``read(timing)`` gives the times at which the timed file reads over
    each limit, ``{"speed": [...], "acc": [...], "jerk": [...], "load": [...]}``."""
    from scipy import sparse
    from scipy.optimize import linprog

    path = coord.path
    total = coord.total
    s = grid(total)
    k = len(s)
    q1 = np.gradient(coord.q(s), s, axis=0)                  # from the grid's own poses: what a sampled file holds
    q2 = np.gradient(q1, s, axis=0)                          # (the chain rule divides by ~0 where the joints rest)
    g1 = np.maximum(np.abs(q1), 1e-9)
    hl, hr = np.diff(s)[:-1], np.diff(s)[1:]                 # spacing left and right of each inner point
    c_m, c_0, c_p = -hr / (hl * (hl + hr)), (hr - hl) / (hl * hr), hl / (hr * (hl + hr))          # b'
    d_m, d_0, d_p = 2.0 / (hl * (hl + hr)), -2.0 / (hl * hr), 2.0 / (hr * (hl + hr))             # b''
    inner = np.arange(1, k - 1)

    # the ceiling on b: joint speeds, the arrival hold, the jerk-limited start and stop
    v_joint = np.broadcast_to(vlim, (k, 7)).copy()
    if caps is not None:
        v_joint = np.minimum(v_joint, np.column_stack([np.interp(coord.s(s), path.s, caps[:, j]) for j in range(7)]))
    b_max = ((v_joint / g1).min(axis=1)) ** 2
    j0, j1 = float((jerk / g1[0]).min()), float((jerk / g1[-1]).min())
    b_max = np.minimum(b_max, START_C * (JERK_SHARE * j0) ** (2.0 / 3.0) * s ** (4.0 / 3.0))
    b_max = np.minimum(b_max, START_C * (JERK_SHARE * j1) ** (2.0 / 3.0) * (total - s) ** (4.0 / 3.0))
    b_max[0] = b_max[-1] = 0.0

    def block(cols, vals, rhs, node):
        """Rows ``sum(vals * b[cols]) <= rhs`` and their mirror, scaled so each reads <= 1, with the
        grid point each row belongs to. ``cols`` and ``vals`` are (rows, entries)."""
        n_rows, n_ent = vals.shape
        m = sparse.csr_matrix(((vals / rhs[:, None]).ravel(), (np.repeat(np.arange(n_rows), n_ent), cols.ravel())),
                              shape=(n_rows, k))
        return sparse.vstack([m, -m]), np.concatenate([node, node])

    def per_interval(lin_a, lin_b, rhs):
        """A limit of the form ``lin_a * sdd + lin_b * b`` (per column), held at both ends of every
        grid interval with that interval's own acceleration sdd = (b[m+1] - b[m]) / 2h. Holding it
        per interval, not by a centred difference, leaves a sawtooth in b nowhere to hide."""
        h2 = 2.0 * np.diff(s)[:, None]
        n_col = lin_a.shape[1]
        m_idx = np.repeat(np.arange(k - 1), n_col)
        cols = np.stack([m_idx, m_idx + 1], axis=1)
        left = np.stack([(-lin_a[:-1] / h2 + lin_b[:-1]).ravel(), (lin_a[:-1] / h2).ravel()], axis=1)
        right = np.stack([(-lin_a[1:] / h2).ravel(), (lin_a[1:] / h2 + lin_b[1:]).ravel()], axis=1)
        r = np.tile(rhs, k - 1)
        a_l, n_l = block(cols, left, r, m_idx)
        a_r, n_r = block(cols, right, r, m_idx + 1)
        return sparse.vstack([a_l, a_r]).tocsr(), np.concatenate([n_l, n_r])

    def at_inner_points(coef_m, coef_0, coef_p, rhs):
        n_col = coef_0.shape[1]
        node = np.repeat(inner, n_col)
        cols = np.stack([node - 1, node, node + 1], axis=1)
        return block(cols, np.stack([coef_m.ravel(), coef_0.ravel(), coef_p.ravel()], axis=1), np.tile(rhs, len(inner)), node)

    a_acc, n_acc = per_interval(q1, q2, ACC_SHARE * acc)
    a_load, n_load = None, np.zeros(0, dtype=int)
    if limits.ugv_limited:
        arm = base_load.arm_load(limits.side, round(float(limits.pole_m), 4))
        dense = np.linspace(0.0, total, max(2001, 3 * k))
        a_d, b_d = arm.path_coeffs(coord.q(dense), dense)
        a_w = np.column_stack([np.interp(s, dense, a_d[:, c]) for c in range(6)])
        b_w = np.column_stack([np.interp(s, dense, b_d[:, c]) for c in range(6)])
        f_rows, g_rows = toppra_opt.load_rows(limits)
        one = slice(0, None, 2)                                # one of each mirrored pair: block() adds the mirror
        a_load, n_load = per_interval(a_w @ f_rows[one].T, b_w @ f_rows[one].T, g_rows[one])
    q1i, q2i = q1[inner], q2[inner]

    w = np.zeros(k)
    w[1:-1] = 0.5 * (hl + hr)
    b = np.minimum(np.interp(s, s_env, e_env) ** 2, b_max)
    sd_lin = np.sqrt(b)
    cut = {name: np.ones(k) for name in ("speed", "acc", "jerk", "load")}     # each limit's share, per grid point
    t_prev, timing, passes, left = None, None, 0, 0
    for passes in range(1, PASSES + 1):
        sd = sd_lin[inner][:, None]
        a_j, n_j = at_inner_points(sd * (0.5 * q1i * d_m[:, None] + 1.5 * q2i * c_m[:, None]),
                                   sd * (0.5 * q1i * d_0[:, None] + 1.5 * q2i * c_0[:, None]),
                                   sd * (0.5 * q1i * d_p[:, None] + 1.5 * q2i * c_p[:, None]), JERK_SHARE * jerk)
        mats = [a_acc, a_j] + ([a_load] if a_load is not None else [])
        ubs = [cut["acc"][n_acc], cut["jerk"][n_j]] + ([cut["load"][n_load]] if a_load is not None else [])
        floor = np.maximum(b, 1e-4 * float(b.max()) + 1e-12)
        cost = -w / (2.0 * floor ** 1.5)                         # the time, linearised about this pass
        cost /= np.abs(cost).max()
        top = b_max * cut["speed"] ** 2
        res = linprog(cost, A_ub=sparse.vstack(mats).tocsr(), b_ub=np.concatenate(ubs),
                      bounds=np.column_stack([np.zeros(k), top]), method="highs")
        if res.status != 0:
            raise RuntimeError(f"Lite's own optimiser found no timing for this path ({res.message})")
        b = np.clip(res.x, 0.0, top)
        sd_new = np.sqrt(b)
        t = np.concatenate([[0.0], np.cumsum(2.0 * np.diff(s) / np.maximum(sd_new[:-1] + sd_new[1:], 1e-9))])
        timing = toppra_opt.PathTiming(coord, t, s, sd_new)
        sd_lin = 0.5 * (sd_lin + sd_new)
        over = read(timing) if read is not None else {}
        left = int(sum(len(v) for v in over.values()))
        for name, times in over.items():
            cut[name][near(t, times, WINDOW_S)] *= TIGHTEN
        if len(over.get("jerk", ())):                            # the path's shape at speed: slower there
            cut["speed"][near(t, over["jerk"], WINDOW_S)] *= TIGHTEN_SPEED
        if not left and t_prev is not None and abs(timing.T - t_prev) <= SETTLED * t_prev:
            break
        t_prev = timing.T
    return timing, dict(grid_points=k, passes=passes, lines_left_over=left)


class CustomPlan:
    optimiser = "custom"

    def __init__(self, md_raw: np.ndarray, limits: LimitSet, opts: ToppraOptions):
        if not available():
            raise RuntimeError(UNAVAILABLE_TEXT)
        self.limits = limits
        self.opts = opts
        path, info = toppra_opt.prepared(md_raw, limits, opts)
        self.path = path
        vlim, acc, caps, held, load = toppra_opt.design_limits(path, limits, opts)
        s_mm, e_mm = toppra_opt.solve_profile(path, vlim, acc, vcap=caps, extra=(load,))
        coord = toppra_opt.JointCoordinate(path, vlim)
        s_env, e_env = coord.sigma(s_mm), e_mm * coord.rate(s_mm)
        keep = np.concatenate([[True], np.diff(s_env) > 1e-12])
        jerk = np.full(7, math.degrees(limits.jerk_max_rad_s3 or limits.jerk_ref_rad_s3))
        self.timing, did = solve(coord, limits, vlim, acc, jerk, caps, s_env[keep], e_env[keep], read=self._read)
        self.T = self.timing.T
        self.n_floor = max(1, math.ceil(self.T / limits.dt_gate - 1e-9))
        self.n_start = self.n_floor
        self.n_cap = max(20 * self.n_floor, 100)
        self.info = dict(info, t_opt_s=round(self.T, 4), arrival_holds=len(held), **did,
                         jerk_design_rad_s3=float(limits.jerk_max_rad_s3 or limits.jerk_ref_rad_s3))

    def _read(self, timing) -> dict:
        """The times at which the timed file, read as the checks read it, is over each limit."""
        lim, dt = self.limits, self.limits.dt_gate
        n = max(1, math.ceil(timing.T / dt - 1e-9))
        t = np.arange(n + 1) * (timing.T / n)
        q = toppra_opt.post_filter(timing.q(t), self.opts.post_lp_hz, dt)
        at = lambda rows, shift: t[np.minimum(np.asarray(rows, dtype=int) + shift, n)]        # noqa: E731
        rating = np.asarray(lim.rating_deg_s, dtype=float)
        late = [tn["line"] for tn in joint_turns(q, lim) if not tn["geometry"] and tn["util"] > 1.0]
        out = {"speed": np.concatenate([at(np.nonzero((speed_deg_s(q, dt) > lim.rate_bar * rating).any(axis=1))[0], 1),
                                        at(late, 0), at(np.maximum(np.asarray(late, dtype=int) - int(round(0.15 / dt)), 0), 0)]),
               "acc": at(np.nonzero((acc_deg_s2(q, dt) > lim.acc_deg_s2 * (1.0 + 0.5 * lim.acc_tol)).any(axis=1))[0], 1)}
        if lim.jerk_max_rad_s3 is not None:
            out["jerk"] = at(np.nonzero((jerk_deg_s3(q, dt) > 0.99 * math.degrees(lim.jerk_max_rad_s3)).any(axis=1))[0], 2)
        if lim.ugv_limited:
            force, moment = ugv_load(q, lim)
            bad = np.zeros(len(q), dtype=bool)
            if lim.ugv_force_n is not None:
                bad |= force > 0.99 * lim.ugv_force_n
            if lim.ugv_moment_nm is not None:
                bad |= moment > 0.99 * lim.ugv_moment_nm
            out["load"] = t[bad]
        return out

    def make(self, n: int) -> np.ndarray:
        q = self.timing.sample(max(int(n), self.n_floor))
        return track.to_md(toppra_opt.post_filter(q, self.opts.post_lp_hz, self.limits.dt_gate))

    def util(self, n: int) -> dict:
        return util_series(self.make(n), self.limits)

    def recipe(self) -> dict:
        return dict(self.info)
