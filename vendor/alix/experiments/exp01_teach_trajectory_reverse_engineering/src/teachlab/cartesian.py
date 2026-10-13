"""Cartesian characterisation of the (smoothed) taught path.

All geometry is from FK of the smoothed joint signal, in the arm's base
frame. Quantities:

    s          arc length (chord sum)
    v          TCP velocity vector, |v| speed          np.gradient of position
    a          acceleration vector
    kappa      curvature |v x a| / |v|^3               (undefined where |v| ~ 0)
    a_tan      d|v|/dt                                 tangential acceleration
    a_lat      |v x a| / |v| = v^2 kappa               observed centripetal accel
    omega      relative-rotation angle / dt, centred   orientation angular rate

The speed-curvature relationship is fitted as v = c * kappa^(-beta) on moving
samples. CAUTION: because kappa = |v x a|/|v|^3 is itself a function of v,
ANY smooth path yields beta ~ 0.3 with r^2 ~ 0.5 (the noise-induced
"two-thirds power law", Maoz/Berthoz/Flash). The fit is therefore reported
beside a NULL CONTROL — phase-randomised surrogates of the same path (same
spectrum, no kinematic law) — and only the difference from the null means
anything.

Corners are curvature peaks taken while moving; peaks that sit against the
low-speed mask are stroke-end TURNAROUNDS whose radius the data cannot
resolve (curvature diverges as the hand stops) and are counted separately.
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np


def tcp_path(kin, q_rad: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    return kin.fk_batch(q_rad)


def arc_length(pos: np.ndarray) -> np.ndarray:
    d = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)])


def angular_rate_deg_s(rot: np.ndarray, fs: float) -> np.ndarray:
    """|omega| from consecutive rotation matrices, CENTRED on the samples:
    omega[i] = mean of the step rates i-1->i and i->i+1 (ends: one step)."""
    R = np.asarray(rot)
    if R.shape[0] < 2:
        return np.zeros(R.shape[0])
    rel = np.einsum("nji,njk->nik", R[:-1], R[1:])            # R_i^T R_{i+1}
    tr = np.trace(rel, axis1=1, axis2=2)
    ang = np.arccos(np.clip((tr - 1.0) / 2.0, -1.0, 1.0))
    w = np.degrees(ang) * fs                                  # step i -> i+1
    out = np.empty(R.shape[0])
    out[0] = w[0]; out[-1] = w[-1]
    out[1:-1] = 0.5 * (w[:-1] + w[1:])
    return out


def _stats(x, mask=None):
    x = np.asarray(x, dtype=float)
    if mask is not None:
        x = x[mask]
    x = x[np.isfinite(x)]
    if x.size == 0:
        return {"max": None, "p99": None, "p95": None, "median": None, "mean": None}
    return {"max": float(x.max()), "p99": float(np.percentile(x, 99)),
            "p95": float(np.percentile(x, 95)), "median": float(np.median(x)),
            "mean": float(x.mean())}


def path_metrics(pos: np.ndarray, rot: Optional[np.ndarray], fs: float, *,
                 v_moving_mps: float = 0.05, v_min_for_kappa_mps: float = 0.02,
                 v_fast_mps: float = 0.10) -> dict:
    dt = 1.0 / fs
    p = np.asarray(pos, dtype=float)
    n = p.shape[0]
    t = np.arange(n) * dt
    v = np.gradient(p, dt, axis=0, edge_order=2)
    a = np.gradient(v, dt, axis=0, edge_order=2)
    speed = np.linalg.norm(v, axis=1)
    cross = np.cross(v, a)
    cn = np.linalg.norm(cross, axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        kappa = np.where(speed > v_min_for_kappa_mps, cn / np.maximum(speed, 1e-12) ** 3, np.nan)
        a_lat = np.where(speed > v_min_for_kappa_mps, cn / np.maximum(speed, 1e-12), np.nan)
    a_tan = np.gradient(speed, dt, edge_order=2)
    omega = angular_rate_deg_s(rot, fs) if rot is not None else np.zeros(n)
    s = arc_length(p)
    moving = speed > v_moving_mps
    fast = speed >= v_fast_mps
    dur = t[-1] if n > 1 else 0.0

    sp_mov = speed[moving]
    cv = float(sp_mov.std() / sp_mov.mean()) if sp_mov.size > 1 and sp_mov.mean() > 0 else None
    edges = np.diff(moving.astype(int))
    starts = list(np.where(edges == 1)[0] + 1); ends = list(np.where(edges == -1)[0] + 1)
    if moving.size and moving[0]:
        starts = [0] + starts
    if moving.size and moving[-1]:
        ends = ends + [n]
    segs = [(int(a_), int(b_)) for a_, b_ in zip(starts, ends) if b_ > a_]
    first_last = (segs[0][0], segs[-1][1]) if segs else (0, n)
    span_s = (first_last[1] - 1 - first_last[0]) * dt if first_last[1] > first_last[0] else 0.0
    span_len = float(s[first_last[1] - 1] - s[first_last[0]]) if n > 1 else 0.0

    return {
        "n": n, "fs_hz": fs, "duration_s": float(dur),
        "path_length_m": float(s[-1]),
        "mean_speed_full_mps": float(s[-1] / dur) if dur > 0 else None,      # incl. dwell
        "active_span_s": float(span_s),
        "mean_speed_active_span_mps": float(span_len / span_s) if span_s > 0 else None,
        "moving_fraction": float(moving.mean()), "v_moving_threshold_mps": v_moving_mps,
        "v_fast_threshold_mps": v_fast_mps, "fast_fraction": float(fast.mean()),
        "n_moving_segments": len(segs),
        "speed": _stats(speed), "speed_moving": _stats(speed, moving), "speed_cv_moving": cv,
        "kappa_1_per_m": _stats(kappa, moving), "kappa_at_speed_1_per_m": _stats(kappa, fast),
        "a_lat_mps2": _stats(a_lat, moving), "a_lat_at_speed_mps2": _stats(a_lat, fast),
        "a_tan_mps2": _stats(np.abs(a_tan), moving), "omega_deg_s": _stats(omega, moving),
        "bbox_m": {"min": [float(x) for x in p.min(axis=0)], "max": [float(x) for x in p.max(axis=0)]},
        "start_m": [float(x) for x in p[0]], "end_m": [float(x) for x in p[-1]],
        "_arrays": {"t": t, "s": s, "speed": speed, "kappa": kappa, "a_lat": a_lat,
                    "a_tan": a_tan, "omega": omega, "moving": moving, "fast": fast, "v": v, "a": a},
    }


def power_law_fit(speed: np.ndarray, kappa: np.ndarray, moving: np.ndarray, *,
                  kappa_pct: Tuple[float, float] = (5.0, 95.0)) -> dict:
    """Least squares of log v = log c - beta * log kappa on moving samples with
    finite kappa inside the central percentile band. Descriptive only — see
    the module docstring and `power_law_null`."""
    sp = np.asarray(speed, dtype=float); k = np.asarray(kappa, dtype=float)
    sel = moving & np.isfinite(k) & (k > 0) & (sp > 0)
    if sel.sum() < 20:
        return {"beta": None, "c": None, "r2": None, "n": int(sel.sum())}
    lo, hi = np.percentile(k[sel], kappa_pct)
    sel &= (k >= lo) & (k <= hi)
    x = np.log(k[sel]); y = np.log(sp[sel])
    A = np.vstack([x, np.ones_like(x)]).T
    (slope, icpt), *_ = np.linalg.lstsq(A, y, rcond=None)
    yhat = A @ np.array([slope, icpt])
    ss_res = float(((y - yhat) ** 2).sum()); ss_tot = float(((y - y.mean()) ** 2).sum()) or 1e-300
    return {"beta": float(-slope), "c": float(np.exp(icpt)), "r2": float(1.0 - ss_res / ss_tot),
            "n": int(sel.sum()), "kappa_range_1_per_m": [float(lo), float(hi)],
            "reference_two_thirds_power_law_beta": 1.0 / 3.0}


def power_law_null(pos: np.ndarray, fs: float, *, n_surrogates: int = 5, seed: int = 0,
                   v_moving_mps: float = 0.05, v_min_for_kappa_mps: float = 0.02) -> dict:
    """The same fit on phase-randomised surrogates of the path: each axis keeps
    its amplitude spectrum and gets random phases, so the surrogate has the
    same smoothness but no speed-curvature coupling beyond what the estimator
    itself induces. Reports the null band for beta and r2."""
    rng = np.random.default_rng(seed)
    p = np.asarray(pos, dtype=float) - np.asarray(pos, dtype=float).mean(axis=0, keepdims=True)
    n = p.shape[0]
    betas, r2s = [], []
    for _ in range(n_surrogates):
        sur = np.empty_like(p)
        for ax in range(p.shape[1]):
            X = np.fft.rfft(p[:, ax])
            ph = rng.uniform(0, 2 * np.pi, X.size)
            ph[0] = 0.0
            if n % 2 == 0:
                ph[-1] = 0.0
            sur[:, ax] = np.fft.irfft(np.abs(X) * np.exp(1j * ph), n=n)
        m = path_metrics(sur, None, fs, v_moving_mps=v_moving_mps, v_min_for_kappa_mps=v_min_for_kappa_mps)
        A = m["_arrays"]
        fit = power_law_fit(A["speed"], A["kappa"], A["moving"])
        if fit["beta"] is not None:
            betas.append(fit["beta"]); r2s.append(fit["r2"])
    if not betas:
        return {"n_surrogates": 0}
    return {"n_surrogates": len(betas), "method": "phase-randomised surrogates (same spectrum per axis)",
            "beta_mean": float(np.mean(betas)), "beta_std": float(np.std(betas)),
            "beta_min": float(min(betas)), "beta_max": float(max(betas)),
            "r2_mean": float(np.mean(r2s))}


def corners(metrics: dict, *, kappa_min_1_per_m: float = 5.0, min_separation_s: float = 0.3,
            v_min_mps: float = 0.03, v_fast_mps: Optional[float] = None, guard_samples: int = 2) -> dict:
    """Direction changes = local maxima of curvature above `kappa_min` (radius
    below 1/kappa_min = 0.2 m by default), at least `min_separation_s` apart.

    A peak whose +/- `guard_samples` neighbourhood touches the low-speed mask
    (speed <= v_min) is a stroke-end TURNAROUND: curvature is still rising
    into the stop and the radius is unresolved by this data. Those are
    counted, not measured. Resolved corners report the blend radius the hand
    used and the speed it carried; the `at speed` subset (>= v_fast) is what a
    generator should read.
    """
    from scipy.signal import find_peaks
    A = metrics["_arrays"]
    fs = metrics["fs_hz"]
    v_fast = metrics["v_fast_threshold_mps"] if v_fast_mps is None else v_fast_mps
    kap = np.nan_to_num(A["kappa"], nan=0.0)
    ok = (A["speed"] > v_min_mps)
    kap_ok = np.where(ok, kap, 0.0)
    idx, _ = find_peaks(kap_ok, height=kappa_min_1_per_m, distance=max(1, int(min_separation_s * fs)))
    resolved, turnarounds = [], []
    n = kap.size
    for i in idx:
        lo, hi = max(0, i - guard_samples), min(n, i + guard_samples + 1)
        entry = {"t_s": float(A["t"][i]), "s_m": float(A["s"][i]), "speed_mps": float(A["speed"][i])}
        if ok[lo:hi].all():
            r = 1.0 / kap[i]; v = float(A["speed"][i])
            entry.update(radius_m=float(r), a_lat_mps2=float(v * v * kap[i]))
            resolved.append(entry)
        else:
            entry.update(radius_m=None, note="turnaround: curvature unresolved against the low-speed mask")
            turnarounds.append(entry)
    radii = [c["radius_m"] for c in resolved]
    fast = [c for c in resolved if c["speed_mps"] >= v_fast]
    return {"count": len(resolved), "kappa_min_1_per_m": kappa_min_1_per_m, "v_min_mps": v_min_mps,
            "guard_samples": guard_samples,
            "min_blend_radius_m": float(min(radii)) if radii else None,
            "median_blend_radius_m": float(np.median(radii)) if radii else None,
            "v_fast_mps": v_fast, "count_at_speed": len(fast),
            "min_blend_radius_at_speed_m": float(min(c["radius_m"] for c in fast)) if fast else None,
            "median_blend_radius_at_speed_m": float(np.median([c["radius_m"] for c in fast])) if fast else None,
            "max_a_lat_at_speed_mps2": float(max(c["a_lat_mps2"] for c in fast)) if fast else None,
            "turnarounds": {"count": len(turnarounds), "list": turnarounds},
            "corners": resolved}


def summarize(kin, q_smooth_rad: np.ndarray, fs: float, **kw) -> dict:
    pos, rot = tcp_path(kin, q_smooth_rad)
    keys = ("v_moving_mps", "v_min_for_kappa_mps", "v_fast_mps")
    m = path_metrics(pos, rot, fs, **{k: v for k, v in kw.items() if k in keys})
    A = m["_arrays"]
    m["power_law"] = power_law_fit(A["speed"], A["kappa"], A["moving"])
    m["power_law"]["null_control"] = power_law_null(pos, fs, v_moving_mps=m["v_moving_threshold_mps"])
    m["corners"] = corners(m)
    m["frame"] = kin.frame; m["base_link"] = kin.base_link
    m["_pos"] = pos; m["_rot"] = rot
    return m
