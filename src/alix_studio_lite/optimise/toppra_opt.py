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
   optimum (``plan_min_time.resample_clock``). A longer target stretches the timing uniformly;
7. where asked (``post_lp_hz``): ALIX's zero-phase low-pass of the timed file
   (``emit_toppra_rung.post_lowpass``, 6 Hz there), the only step of the chain that calms TOPP-RA's
   abrupt acceleration switching; Lite then puts the first and last line back on the recording's.

Where asked (``arrival_hold``), step 5 also takes ALIX's arrival hold
(``emit_toppra_rung.arrival_hold``): a joint nearing a turn close to its end stop is slowed there,
so the predicted overshoot keeps L1 without slowing the whole file.

TOPP-RA is optional: ``available()`` is False when the ``toppra`` package is missing (it has no
aarch64 wheel and is built from source on the Jetson), and Lite then offers manual tuning only.
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


#: directions a load limit is held along: the axes, the face diagonals and the body diagonals. Holding
#: the load under the limit along all 13 (both ways) keeps its size within about 1.1 x the limit, so
#: the limit is held at LOAD_SHARE of its value there and the check on the file settles the rest.
_DIRS = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, -1, 0], [1, 0, 1], [1, 0, -1], [0, 1, 1],
                  [0, 1, -1], [1, 1, 1], [1, 1, -1], [1, -1, 1], [1, -1, -1]], dtype=float)
_DIRS /= np.linalg.norm(_DIRS, axis=1)[:, None]
LOAD_SHARE = 0.93


def load_rows(limits: LimitSet) -> tuple[np.ndarray, np.ndarray]:
    """(F (k, 6), g (k,)) with F @ [force, moment] <= g: the UGV load limits as straight-line bounds."""
    rows, g = [], []
    for lim, col in ((limits.ugv_force_n, 0), (limits.ugv_moment_nm, 3)):
        if lim is None:
            continue
        for d in _DIRS:
            for sign in (1.0, -1.0):
                r = np.zeros(6)
                r[col:col + 3] = sign * d
                rows.append(r)
                g.append(LOAD_SHARE * float(lim))
    return np.array(rows), np.array(g)


def ugv_load_constraint(limits: LimitSet):
    """The UGV load limits as a TOPP-RA constraint, or None when the limit set carries none. Along
    the path the load is A(x) xdd + B(x) xd^2 (``base_load.ArmLoad.path_coeffs``), the same form as
    a joint acceleration, so TOPP-RA holds it the same way."""
    if not limits.ugv_limited:
        return None
    import toppra.constraint as constraint
    from toppra.constraint.linear_constraint import LinearConstraint, canlinear_colloc_to_interpolate

    from ..motion import base_load

    arm = base_load.arm_load(limits.side, round(float(limits.pole_m), 4))
    f_rows, g_rows = load_rows(limits)

    class UgvLoad(LinearConstraint):
        def __init__(self):
            super().__init__()
            self.identical = True
            self.set_discretization_type(constraint.DiscretizationType.Interpolation)
            self._format_string = "    UGV load limit\n"

        def compute_constraint_params(self, path, gridpoints, *args, **kwargs):
            xx = np.linspace(gridpoints[0], gridpoints[-1], max(2001, 4 * len(gridpoints)))
            a_d, b_d = arm.path_coeffs(path(xx), xx)
            a = np.column_stack([np.interp(gridpoints, xx, a_d[:, k]) for k in range(6)])
            b = np.column_stack([np.interp(gridpoints, xx, b_d[:, k]) for k in range(6)])
            return canlinear_colloc_to_interpolate(a, b, np.zeros_like(a), f_rows, g_rows, None, None,
                                                   gridpoints, identical=True)

    return UgvLoad()


def problem(path, vlim_deg_s, acc_deg_s2, vcap=None, extra=()):
    """The TOPP-RA problem along ``path`` (a ``local_speed.TaughtPath``).

    Same construction as ``local_speed.retime_toppra`` with the ``max`` demand (only the
    limits bound the speed): spline in normalised arc length, a per-knot joint velocity bound,
    the acceleration bound with the ``Interpolation`` scheme (the default scheme overshot 600 to
    1007.7 deg/s^2 between grid points), the ``seidel`` solver. ``extra`` are further constraints
    (the UGV load)."""
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
    return algo.TOPPRA([pc_vel, pc_acc] + [c for c in extra if c is not None], ppath, solver_wrapper="seidel")


def solve_min_time(path, vlim_deg_s, acc_deg_s2, vcap=None, extra=()):
    """The time-optimal trajectory along ``path``, at rest at both ends. Returns the toppra
    trajectory object; ``traj.duration`` is the minimum time."""
    traj = problem(path, vlim_deg_s, acc_deg_s2, vcap, extra).compute_trajectory(0, 0)
    if traj is None:
        raise RuntimeError("TOPP-RA found no feasible timing for this path")
    return traj


def solve_profile(path, vlim_deg_s, acc_deg_s2, vcap=None, extra=()) -> tuple[np.ndarray, np.ndarray]:
    """The same optimum as its speed along the path: (arc length (K,) in mm, speed (K,) in mm/s), at
    rest at both ends. It is the fastest speed the limits allow at every point of the path."""
    inst = problem(path, vlim_deg_s, acc_deg_s2, vcap, extra)
    out = inst.compute_parameterization(0, 0)
    if out is None or out[1] is None:
        raise RuntimeError("TOPP-RA found no feasible timing for this path")
    total = path.total()
    return np.asarray(inst.problem_data.gridpoints, dtype=float) * total, np.asarray(out[1], dtype=float) * total


def pin_ends_in_time(q: np.ndarray, start: np.ndarray, end: np.ndarray, dt: float,
                     blend_s: float = 0.25) -> np.ndarray:
    """Add the start and end errors of a timed file back, each faded out with a raised cosine over
    ``blend_s`` seconds: after a zero-phase filter the end lines are filter output, not the poses."""
    q = np.asarray(q, dtype=float).copy()
    t = np.arange(q.shape[0]) * float(dt)
    lam = max(min(float(blend_s), 0.5 * float(t[-1])), 1e-9)
    w0 = 0.5 * (1.0 + np.cos(np.pi * np.clip(t / lam, 0.0, 1.0)))
    w1 = 0.5 * (1.0 + np.cos(np.pi * np.clip((t[-1] - t) / lam, 0.0, 1.0)))
    q += w0[:, None] * (np.asarray(start, float) - q[0])[None, :]
    q += w1[:, None] * (np.asarray(end, float) - q[-1])[None, :]
    return q


def post_filter(q: np.ndarray, hz: float, dt: float) -> np.ndarray:
    """ALIX's low-pass of a timed file, with the first and last line kept exactly."""
    import emit_toppra_rung as ETR

    if not hz or q.shape[0] < 16:
        return q
    return pin_ends_in_time(ETR.post_lowpass(q, float(hz), float(dt)), q[0], q[-1], dt)


def sample(traj, n: int) -> np.ndarray:
    """``n + 1`` poses at t_i = i T / n (degrees). With n = ceil(T / 5 ms) this is
    ``plan_min_time.resample_clock``; a larger n stretches the timing uniformly."""
    T = float(traj.duration)
    return traj(np.arange(int(n) + 1) * (T / int(n)))


def prepared(md_raw: np.ndarray, limits: LimitSet, opts: ToppraOptions) -> tuple:
    """Steps 1 to 4, the same for every automatic optimiser: (the path, what was done to it)."""
    import local_speed as LS
    import taught_tracks as tt

    raw = track.to_deg(md_raw)
    q = tt.smooth_taught(raw, opts.smooth_hz, DT_RECORD_S) if opts.smooth_hz else raw.copy()
    removed = 0
    if opts.strip_pauses:
        keep = prep.pause_mask(q, tt.PAUSE_TOL_DEG, tt.PAUSE_MIN_LINES)
        removed = int((~keep).sum())
        q = q[keep]
    p0 = LS.TaughtPath(q, limits.side, limits.frame)
    qs = LS.arclen_smooth(p0, opts.smooth_mm) if opts.smooth_mm else p0.q_src[p0.kept]
    end_shift = float(np.abs(qs[-1] - raw[-1]).max())
    if opts.pin_ends:
        qs = pin_ends(qs, p0.s, raw[0], raw[-1], max(float(opts.smooth_mm), 30.0))
    path = LS.TaughtPath(qs, limits.side, limits.frame)
    return path, dict(paused_lines_removed=removed, path_mm=round(path.total(), 1),
                      end_shift_before_pin_deg=round(end_shift, 4))


def design_limits(path, limits: LimitSet, opts: ToppraOptions) -> tuple:
    """(joint speeds (7,), accelerations (7,), the arrival hold's per-knot speed caps or None, the turns
    it holds, the UGV load constraint or None): what the timing is designed within."""
    vlim = limits.rate_design * np.asarray(limits.rating_deg_s, dtype=float)
    acc = np.full(7, float(limits.acc_deg_s2))
    caps, held = None, []
    if opts.arrival_hold:
        import emit_toppra_rung as ETR

        caps, held = ETR.arrival_hold(path.q_src[path.kept], vlim, float(opts.arrival_hold), model=limits.model)
    return vlim, acc, caps, held, ugv_load_constraint(limits)


class JointCoordinate:
    """The prepared path measured by how far the joints turn, not by how far the hand travels.

    sigma grows by sqrt(sum((dq_j / rating_j)^2)): in seconds, the time the step would take with the
    joints at their ratings. Along the hand's travel s, a part where the wrist turns and the hand
    barely moves is squeezed into almost no distance, and the joint motion per millimetre is huge
    there (TOPP-RA's own grid needs 6,000 to 9,000 points on Lite's recordings for it). Along sigma
    the joint motion per unit is close to the ratings everywhere, so limits pulled back onto it are
    nearly constant and a plain grid resolves the path."""

    def __init__(self, path, vlim: np.ndarray):
        from scipy.interpolate import PchipInterpolator

        self.path = path
        self.vlim = np.asarray(vlim, dtype=float)
        ss = np.unique(np.concatenate([path.s, 0.5 * (path.s[:-1] + path.s[1:])]))
        rate = np.maximum(np.sqrt(((path.dq(ss) / self.vlim) ** 2).sum(axis=1)), 1e-9)       # d sigma / d s
        sig = np.concatenate([[0.0], np.cumsum(0.5 * (rate[:-1] + rate[1:]) * np.diff(ss))])
        self.total = float(sig[-1])
        self._s_of = PchipInterpolator(sig, ss)
        self._sig_of = PchipInterpolator(ss, sig)

    def s(self, sigma) -> np.ndarray:
        return self._s_of(np.clip(sigma, 0.0, self.total))

    def sigma(self, s) -> np.ndarray:
        return self._sig_of(np.clip(s, self.path.s[0], self.path.s[-1]))

    def q(self, sigma) -> np.ndarray:
        return self.path.q(self.s(sigma))

    def rate(self, s) -> np.ndarray:
        return np.maximum(np.sqrt(((self.path.dq(s) / self.vlim) ** 2).sum(axis=1)), 1e-9)

    def dq(self, sigma) -> np.ndarray:
        """dq/dsigma at ``sigma`` ((K, 7), degrees): each joint's share of its rating, at most 1."""
        s = self.s(sigma)
        return self.path.dq(s) / self.rate(s)[:, None]


class PathTiming:
    """A timing of the prepared path: where on it the arm is at any time."""

    def __init__(self, coord: JointCoordinate, t: np.ndarray, sigma: np.ndarray, rate: np.ndarray, bounds_t=None):
        from scipy.interpolate import CubicHermiteSpline

        self.coord = coord
        self.T = float(t[-1])
        self.bounds_t = np.array([0.0, self.T]) if bounds_t is None else np.asarray(bounds_t, dtype=float)
        self._sig = CubicHermiteSpline(t, sigma, rate)

    def q(self, t: np.ndarray) -> np.ndarray:
        return self.coord.q(self._sig(np.clip(t, 0.0, self.T)))

    def sample(self, n: int) -> np.ndarray:
        """``n + 1`` poses at t_i = i T' / n, T' >= T: more lines play the same timing more slowly."""
        return self.q(np.arange(int(n) + 1) * (self.T / int(n)))


class ToppraPlan:
    optimiser = "toppra"

    def __init__(self, md_raw: np.ndarray, limits: LimitSet, opts: ToppraOptions):
        if not available():
            raise RuntimeError(UNAVAILABLE_TEXT)
        self.limits = limits
        self.opts = opts
        path, info = prepared(md_raw, limits, opts)
        self.path = path
        self.vlim, self.acc, caps, held, load = design_limits(path, limits, opts)
        self.traj = solve_min_time(path, self.vlim, self.acc, vcap=caps, extra=(load,))
        self.T = float(self.traj.duration)
        self.n_floor = max(1, math.ceil(self.T / limits.dt_gate - 1e-9))
        self.n_start = self.n_floor
        self.n_cap = max(20 * self.n_floor, 100)
        self.info = dict(info, t_opt_s=round(self.T, 4), arrival_holds=len(held))

    def make(self, n: int) -> np.ndarray:
        q = sample(self.traj, max(int(n), self.n_floor))
        return track.to_md(post_filter(q, self.opts.post_lp_hz, self.limits.dt_gate))

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
    return "Re-teach this region, or tune it manually with light smoothing."
