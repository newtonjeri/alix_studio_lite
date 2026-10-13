"""Zero-phase smoothing — the reference signal every constraint is read from.

Raw teach samples are a polled, quantised joint state with a 1x/2x step beat
(teachfile.beat_test); differentiating them characterises the poll, not the
motion. Both filters here are zero-phase (no lag, so corners stay where the
hand put them):

* Butterworth low-pass, `sosfiltfilt` — the primary; cutoff 2.5 Hz sits an
  octave above the 99 %-energy frequency of the position signal (0.84 Hz
  pooled on F3_LDTPS04) and two octaves below the beat.
* Savitzky-Golay, 200 ms window, cubic — local polynomial; smaller residual,
  more high-frequency leakage (its jerk is 5x the Butterworth's).
"""
from __future__ import annotations

from typing import Tuple

import numpy as np
from scipy.signal import butter, savgol_filter, sosfiltfilt


def butter_zero_phase(x: np.ndarray, fs: float, fc_hz: float = 2.5, order: int = 2) -> np.ndarray:
    if fc_hz >= 0.5 * fs:
        raise ValueError(f"cutoff {fc_hz} Hz is not below Nyquist ({0.5*fs} Hz)")
    sos = butter(order, fc_hz, btype="low", fs=fs, output="sos")
    return sosfiltfilt(sos, np.asarray(x, dtype=float), axis=0)


def savgol_zero_phase(x: np.ndarray, fs: float, window_s: float = 0.2, polyorder: int = 3) -> np.ndarray:
    win = int(round(window_s * fs))
    if win % 2 == 0:
        win += 1
    win = max(win, polyorder + 2 if (polyorder + 2) % 2 == 1 else polyorder + 3)
    return savgol_filter(np.asarray(x, dtype=float), win, polyorder, axis=0)


def smooth(x: np.ndarray, fs: float, method: str = "butter", **kw) -> Tuple[np.ndarray, dict]:
    """Dispatch by name; returns (smoothed, meta) with meta describing exactly
    what was applied so the profile can carry it."""
    if method == "butter":
        fc = float(kw.get("fc_hz", 2.5)); order = int(kw.get("order", 2))
        return butter_zero_phase(x, fs, fc, order), {
            "method": "butterworth_sosfiltfilt", "cutoff_hz": fc, "order": order,
            "zero_phase": True}
    if method == "savgol":
        w = float(kw.get("window_s", 0.2)); p = int(kw.get("polyorder", 3))
        return savgol_zero_phase(x, fs, w, p), {
            "method": "savitzky_golay", "window_s": w, "polyorder": p, "zero_phase": True}
    if method == "none":
        return np.asarray(x, dtype=float).copy(), {"method": "none", "zero_phase": True}
    raise ValueError(f"unknown smoothing method {method!r}")


def residual_report(raw: np.ndarray, smoothed: np.ndarray) -> dict:
    """What the smoother removed, per joint — the noise floor being discarded."""
    r = np.asarray(raw, dtype=float) - np.asarray(smoothed, dtype=float)
    return {
        "rms_deg": [float(v) for v in np.sqrt((r ** 2).mean(axis=0))],
        "rms_overall_deg": float(np.sqrt((r ** 2).mean())),
        "max_abs_deg": [float(v) for v in np.abs(r).max(axis=0)],
    }
