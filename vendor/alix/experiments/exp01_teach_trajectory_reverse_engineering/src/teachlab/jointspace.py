"""Joint-space characterisation: derivatives, spectra, reversals, utilization.

Derivative scheme is stated in every result because it changes the answer by
an order of magnitude on raw data (np.gradient x3 gives 185e3 deg/s^3 on
F3_LDTPS04; repeated forward differences 1.5e6; a 5-point central stencil
5.9e5). On a smoothed signal all three agree.
"""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

import numpy as np

DERIVATIVE_SCHEME = "np.gradient applied successively (2nd-order central interior, 2nd-order one-sided edges)"


def derivatives(q_deg: np.ndarray, fs: float) -> dict:
    dt = 1.0 / fs
    q = np.asarray(q_deg, dtype=float)
    v = np.gradient(q, dt, axis=0, edge_order=2)
    a = np.gradient(v, dt, axis=0, edge_order=2)
    j = np.gradient(a, dt, axis=0, edge_order=2)
    return {"vel": v, "acc": a, "jerk": j, "scheme": DERIVATIVE_SCHEME, "fs_hz": fs}


def peak_abs(x: np.ndarray) -> list:
    return [float(v) for v in np.abs(np.asarray(x)).max(axis=0)]


def rms(x: np.ndarray) -> list:
    return [float(v) for v in np.sqrt((np.asarray(x, dtype=float) ** 2).mean(axis=0))]


SPECTRUM_WINDOW = "hann"


def spectrum(x: np.ndarray, fs: float) -> Tuple[np.ndarray, np.ndarray]:
    """One-sided power spectrum (|rfft|^2) of the mean-removed, Hann-windowed
    columns of x. Returns (freqs, power (F, J)).

    The window matters here: a joint that ends 80 deg from where it started
    is a ramp to a rectangular window, and its leakage alone puts the 99 %
    energy point at ~1.5 Hz (a pure ramp gives f99 = 1.53 Hz unwindowed).
    """
    x = np.asarray(x, dtype=float)
    x = x - x.mean(axis=0, keepdims=True)
    w = np.hanning(x.shape[0])[:, None]
    p = np.abs(np.fft.rfft(x * w, axis=0)) ** 2
    f = np.fft.rfftfreq(x.shape[0], d=1.0 / fs)
    return f, p


def energy_fraction_freq(freqs: np.ndarray, power: np.ndarray, frac: float = 0.99) -> dict:
    """Frequency below which `frac` of the (DC-excluded) energy lies — per
    column and pooled over columns."""
    p = np.asarray(power, dtype=float)[1:]          # drop DC
    f = np.asarray(freqs)[1:]
    out_per = []
    for j in range(p.shape[1]):
        c = np.cumsum(p[:, j]); tot = c[-1] if c[-1] > 0 else 1.0
        k = int(np.searchsorted(c / tot, frac))
        out_per.append(float(f[min(k, f.size - 1)]))
    pooled = p.sum(axis=1)
    c = np.cumsum(pooled); tot = c[-1] if c[-1] > 0 else 1.0
    k = int(np.searchsorted(c / tot, frac))
    return {"per_joint_hz": out_per, "pooled_hz": float(f[min(k, f.size - 1)]), "fraction": frac}


def welch_psd(x: np.ndarray, fs: float, nperseg: int = 1024) -> Tuple[np.ndarray, np.ndarray]:
    from scipy.signal import welch
    x = np.asarray(x, dtype=float)
    nper = min(nperseg, x.shape[0])
    f, p = welch(x - x.mean(axis=0, keepdims=True), fs=fs, nperseg=nper, axis=0)
    return f, p


def reversal_stats(raw: np.ndarray) -> dict:
    """Direction reversals of the RAW integer steps, per joint.

    moving step  = |dq| > 0 LSB; reversal = sign(dq) differs between consecutive
    moving steps; fraction = reversals / (n_moving - 1). Also reported: the
    fraction of steps at or below 1 LSB (the quantisation floor) and the
    median |dq| — a joint whose median step is 1-2 LSB is dithering, not
    moving, and its reversal fraction says nothing about the hand.
    """
    dq = np.diff(np.asarray(raw, dtype=float), axis=0)
    out = {"per_joint": []}
    for j in range(dq.shape[1]):
        d = dq[:, j]
        moving = d[d != 0]
        s = np.sign(moving)
        rev = int((s[1:] != s[:-1]).sum()) if moving.size > 1 else 0
        out["per_joint"].append({
            "moving_steps": int(moving.size),
            "reversal_fraction": float(rev / max(moving.size - 1, 1)),
            "frac_le_1lsb": float((np.abs(d) <= 1).mean()),
            "frac_zero": float((d == 0).mean()),
            "median_abs_step_lsb": float(np.median(np.abs(d))),
            "mean_abs_step_lsb": float(np.abs(d).mean()),
        })
    return out


def utilization(vel_deg_s: np.ndarray, qdot_max_deg_s: Sequence[float]) -> dict:
    """peak |qdot_j| / qdot_max_j per joint, plus the binding joint."""
    pk = peak_abs(vel_deg_s)
    u = [p / float(m) for p, m in zip(pk, qdot_max_deg_s)]
    k = int(np.argmax(u))
    return {"peak_vel_deg_s": pk, "qdot_max_deg_s": [float(m) for m in qdot_max_deg_s],
            "utilization": u, "max_utilization": float(u[k]), "binding_joint": k + 1}


def summarize(raw: np.ndarray, q_deg: np.ndarray, q_smooth_deg: np.ndarray, fs: float,
              qdot_max_deg_s: Sequence[float]) -> dict:
    """Everything joint-space, raw vs smoothed, in one dict."""
    d_raw = derivatives(q_deg, fs)
    d_sm = derivatives(q_smooth_deg, fs)
    f, p_pos = spectrum(q_deg, fs)
    _, p_pos_sm = spectrum(q_smooth_deg, fs)
    _, p_vel = spectrum(d_raw["vel"], fs)
    _, p_vel_sm = spectrum(d_sm["vel"], fs)
    return {
        "scheme": DERIVATIVE_SCHEME,
        "spectrum_window": SPECTRUM_WINDOW,
        "utilization_basis": "peak |d(q_smoothed)/dt| (the smoothing named in the profile) / qdot_max (xacro)",
        "raw": {"peak_vel_deg_s": peak_abs(d_raw["vel"]), "peak_acc_deg_s2": peak_abs(d_raw["acc"]),
                "peak_jerk_deg_s3": peak_abs(d_raw["jerk"]), "rms_jerk_deg_s3": rms(d_raw["jerk"])},
        "smoothed": {"peak_vel_deg_s": peak_abs(d_sm["vel"]), "peak_acc_deg_s2": peak_abs(d_sm["acc"]),
                     "peak_jerk_deg_s3": peak_abs(d_sm["jerk"]), "rms_jerk_deg_s3": rms(d_sm["jerk"])},
        "position_f99": energy_fraction_freq(f, p_pos, 0.99),
        "position_f90": energy_fraction_freq(f, p_pos, 0.90),
        "position_smoothed_f99": energy_fraction_freq(f, p_pos_sm, 0.99),
        "velocity_f99": energy_fraction_freq(f, p_vel, 0.99),
        "velocity_smoothed_f99": energy_fraction_freq(f, p_vel_sm, 0.99),
        "reversals": reversal_stats(raw),
        "utilization": utilization(d_sm["vel"], qdot_max_deg_s),
        "range_deg": [[float(a), float(b)] for a, b in zip(q_deg.min(axis=0), q_deg.max(axis=0))],
        "_derivatives_smoothed": d_sm,       # arrays, for plots; stripped from YAML
        "_derivatives_raw": d_raw,
        "_spectra": {"freqs": f, "pos_raw": p_pos, "pos_smooth": p_pos_sm,
                     "vel_raw": p_vel, "vel_smooth": p_vel_sm},
    }
