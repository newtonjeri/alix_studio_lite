"""The TIME-OPTIMAL optimiser: ALIX's hardware-proven chain for taught paths.

1. 5 Hz zero-phase low-pass of the recording (``taught_tracks.smooth_taught``);
2. pause removal (``taught_tracks.strip_pauses`` rule, R4);
3. the path as joint angles against hand arc length, smoothed over 30 mm
   (``local_speed.TaughtPath``, ``local_speed.arclen_smooth``);
4. Lite only: the first and last pose put back exactly on the recording's (the arc-length
   smoothing moves the end by up to 0.06 deg), blended out over the last 30 mm of path;
5. TOPP-RA: the fastest timing within 0.95 x the joint ratings and 600 deg/s^2
   (``solve_min_time``, a copy of ``local_speed.retime_toppra`` that takes its limits as
   arguments and returns the trajectory instead of reading module globals);
6. sampled every 5 ms with the point count rounded up, so the file is never faster than the
   optimum (``plan_min_time.resample_clock``). A longer target stretches the timing uniformly.

TOPP-RA is optional: ``available()`` is False when the ``toppra`` package is missing (it has no
aarch64 wheel and is built from source on the Jetson), and Lite then offers the default only.
"""
from __future__ import annotations

import math

import numpy as np

from ..motion import track
from ..motion.checks import util_series
from ..motion.limits import LimitSet
from ..motion.timing import DT_RECORD_S
from . import prep
from .types import ToppraOptions

UNAVAILABLE_TEXT = "Time-optimal is not available on this computer (toppra is not installed)."


def available() -> bool:
    try:
        import toppra  # noqa: F401
        import toppra.algorithm  # noqa: F401
        import toppra.constraint  # noqa: F401
    except Exception:                                                      # noqa: BLE001
        return False
    return True


def toppra_version() -> str | None:
    try:
        from importlib.metadata import version

        return version("toppra")
    except Exception:                                                      # noqa: BLE001
        return None


def pin_ends(q: np.ndarray, s: np.ndarray, start: np.ndarray, end: np.ndarray,
             blend_mm: float) -> np.ndarray:
    """Add the start and end errors back, each faded out with a raised cosine over
    ``blend_mm`` of path, so the path meets the recording's poses exactly and stays smooth."""
    q = np.asarray(q, dtype=float).copy()
    total = float(s[-1] - s[0])
    if total <= 0:
        return q
    lam = max(min(float(blend_mm), 0.5 * total), 1e-9)
    u0 = np.clip((s - s[0]) / lam, 0.0, 1.0)
    u1 = np.clip((s[-1] - s) / lam, 0.0, 1.0)
    w0 = 0.5 * (1.0 + np.cos(np.pi * u0))
    w1 = 0.5 * (1.0 + np.cos(np.pi * u1))
    q += w0[:, None] * (np.asarray(start, float) - q[0])[None, :]
    q += w1[:, None] * (np.asarray(end, float) - q[-1])[None, :]
    return q


def solve_min_time(path, vlim_deg_s, acc_deg_s2, vcap=None):
    """The time-optimal trajectory along ``path`` (a ``local_speed.TaughtPath``).

    Same construction as ``local_speed.retime_toppra`` with the ``max`` demand (only the
    limits bound the speed): spline in normalised arc length, a per-knot joint velocity bound,
    the acceleration bound with the ``Interpolation`` scheme (the default scheme overshot 600 to
    1007.7 deg/s^2 between grid points), the ``seidel`` solver, rest at both ends. Returns the
    toppra trajectory object; ``traj.duration`` is the minimum time."""
    import toppra as ta
    import toppra.algorithm as algo
    import toppra.constraint as constraint

    q = path.q_src[path.kept]
    total = path.total()
    x = path.s / total
    ppath = ta.SplineInterpolator(x, q)
    dq_dx = np.abs(path.dq(path.s)) * total
    v_star = np.full_like(path.s, 1e6)                   # local_speed.demand("max", path)
    vlim = np.minimum(np.asarray(vlim_deg_s, dtype=float)[None, :], dq_dx * (v_star[:, None] / total))
    if vcap is not None:
        vlim = np.minimum(vlim, np.asarray(vcap, dtype=float))
    vlim = np.maximum(vlim, 1e-3)

    def vlim_at(xx):
        return np.column_stack([-np.array([np.interp(xx, x, vlim[:, j]) for j in range(7)]),
                                np.array([np.interp(xx, x, vlim[:, j]) for j in range(7)])])

    pc_vel = constraint.JointVelocityConstraintVarying(vlim_at)
    pc_acc = constraint.JointAccelerationConstraint(
        np.asarray(acc_deg_s2, dtype=float),
        discretization_scheme=constraint.DiscretizationType.Interpolation)
    inst = algo.TOPPRA([pc_vel, pc_acc], ppath, solver_wrapper="seidel")
    traj = inst.compute_trajectory(0, 0)
    if traj is None:
        raise RuntimeError("TOPP-RA found no feasible timing for this path")
    return traj


def sample(traj, n: int) -> np.ndarray:
    """``n + 1`` poses at t_i = i T / n (degrees). With n = ceil(T / 5 ms) this is
    ``plan_min_time.resample_clock``; a larger n stretches the timing uniformly."""
    T = float(traj.duration)
    return traj(np.arange(int(n) + 1) * (T / int(n)))


class ToppraPlan:
    optimiser = "toppra"

    def __init__(self, md_raw: np.ndarray, limits: LimitSet, opts: ToppraOptions, segments=()):
        if not available():
            raise RuntimeError(UNAVAILABLE_TEXT)
        import local_speed as LS
        import taught_tracks as tt

        self.limits = limits
        self.opts = opts
        raw = track.to_deg(md_raw)
        q = tt.smooth_taught(raw, opts.smooth_hz, DT_RECORD_S) if opts.smooth_hz else raw.copy()
        rows = np.arange(raw.shape[0])
        removed = 0
        if opts.strip_pauses:
            keep = prep.pause_mask(q, tt.PAUSE_TOL_DEG, tt.PAUSE_MIN_LINES)
            removed = int((~keep).sum())
            q, rows = q[keep], rows[keep]
        p0 = LS.TaughtPath(q, limits.side, limits.frame)
        rows = rows[p0.kept]
        qs = LS.arclen_smooth(p0, opts.smooth_mm) if opts.smooth_mm else p0.q_src[p0.kept]
        end_shift = float(np.abs(qs[-1] - raw[-1]).max())
        if opts.pin_ends:
            qs = pin_ends(qs, p0.s, raw[0], raw[-1], max(float(opts.smooth_mm), 30.0))
        path = LS.TaughtPath(qs, limits.side, limits.frame)
        rows = rows[path.kept]
        self.path = path
        rating = np.asarray(limits.rating_deg_s, dtype=float)
        self.vlim = limits.rate_design * rating
        self.acc = np.full(7, float(limits.acc_deg_s2))
        self.segments = []
        vcap = None
        for s in sorted(segments, key=lambda s: s.start):
            m = (rows >= s.start - 1) & (rows <= s.end - 1)
            if not m.any():
                continue
            if vcap is None:
                vcap = np.full((rows.shape[0], 7), np.inf)
            vcap[m] = np.minimum(vcap[m], float(s.pct) / 100.0 * self.vlim)
            self.segments.append(dict(start=s.start, end=s.end, pct=s.pct, label=s.label,
                                      knots=int(m.sum())))
        self.traj = solve_min_time(path, self.vlim, self.acc, vcap)
        self.T = float(self.traj.duration)
        self.n_floor = max(1, math.ceil(self.T / limits.dt_gate - 1e-9))
        self.n_start = self.n_floor
        self.n_cap = max(20 * self.n_floor, 100)
        self.info = dict(paused_lines_removed=removed, path_mm=round(path.total(), 1),
                         t_opt_s=round(self.T, 4), end_shift_before_pin_deg=round(end_shift, 4),
                         segments=self.segments)

    def make(self, n: int) -> np.ndarray:
        return track.to_md(sample(self.traj, max(int(n), self.n_floor)))

    def util(self, n: int) -> dict:
        """Utilisation from the trajectory itself at the delivered timing (exact, no filter):
        speed against 0.95 x rating, acceleration against 600 deg/s^2. Falls back to the
        byte readings when the file was slowed past the optimum by a byte check."""
        n = max(int(n), self.n_floor)
        k = n * self.limits.dt_gate / self.T                      # time stretch, >= 1
        t = np.arange(n + 1) * (self.T / n)
        v = np.abs(self.traj(t, 1)) / k / self.vlim[None, :]
        a = np.abs(self.traj(t, 2)) / k ** 2 / self.acc[None, :]
        out = {}
        for j in range(7):
            out[f"speed_J{j + 1}"] = v[:, j]
            out[f"acceleration_J{j + 1}"] = a[:, j]
        if max(float(x.max()) for x in out.values()) < 0.98:
            return util_series(self.make(n), self.limits)
        return out

    def recipe(self) -> dict:
        return dict(self.info)


def smoothing_hint(md_raw: np.ndarray, limits: LimitSet, opts: ToppraOptions) -> str | None:
    """When L5 fails, a lighter arc-length smoothing that keeps the hand on the path."""
    import taught_tracks as tt
    from dataclasses import replace

    raw = track.to_deg(md_raw)
    for mm in (20.0, 15.0, 10.0, 5.0):
        if mm >= opts.smooth_mm:
            continue
        plan = ToppraPlan(md_raw, limits, replace(opts, smooth_mm=mm))
        dev = tt.deviation_mm(raw, track.to_deg(plan.make(plan.n_floor)), limits.side,
                              limits.frame)["max_mm"]
        if dev <= limits.max_dev_mm:
            return (f"Use lighter path smoothing ({mm:g} mm): the hand then stays within {dev:.1f} mm "
                    f"of the taught path, and the file is {plan.T:.1f} s at its fastest.")
    return "Re-teach this region, or use the Default optimiser with light smoothing."
