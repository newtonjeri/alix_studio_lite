"""Run-vs-run comparison: does a generated trajectory execute like drag replay?

Metrics per recorded run (all on the EXECUTED joint stream from the UDP push):

* velocity spectrum — ALWAYS np.gradient of the executed position, after
  resampling the window onto a uniform grid from the real arrival times (a
  dropped frame is a gap, not a step); Hann-windowed, pooled over joints,
  DC included; f90/f99 and the fraction of velocity energy below 2 Hz (the
  brief's criterion). The controller's joint_speed field is only checked
  for consistency and reported beside it — one estimator for both runs, or
  the verdict compares estimators instead of motion.
* resonance — Welch PSD peaks in 2–20 Hz (same nperseg for both runs) that
  stand >= 10 dB above a LOCAL median floor AND >= 10 dB above the
  reference at that frequency AND, when a commanded spectrum exists,
  >= 10 dB above the command. Without a commanded spectrum, lines on the
  candidate's own stroke comb are excluded (a raster's harmonics are what
  was asked for) — which cannot see a resonance that sits exactly on a
  harmonic, so the command spectrum is what makes the test complete.
* TCP speed — FK of the executed joints (Arm_Tip, arm base frame) with a
  zero-phase 5 Hz Butterworth on position before differentiating; CV over
  moving samples; AVERAGE speed over the full window incl. ramps/turnarounds
  (path length / window duration — the productivity metric)
* per-joint utilization — peak |d(position)/dt| after a 10 Hz zero-phase
  filter / qdot_max (the controller's speed field is reported beside it)
* post-filter jerk — jerk of the 5 Hz-smoothed executed joints (peak, RMS)
* commanded stream (command.csv) — tracking lag (min-RMS search), RMS/max
  error after lag, max commanded step per cycle against 180 deg/s x period
  and the vendor's 10 deg

Verdict, candidate vs reference: fraction of velocity energy below 2 Hz
>= max(0.90, reference - 0.02), no reference/command-relative resonance
peaks, TCP speed CV <= 1.05 x the reference (a 5 % allowance on "equal or
better"), the candidate traced >= 90 % of the reference path length (same
trajectory assumed), no selected stage reported ok=False, and both windows
are at least MIN_WINDOW_S long (a shorter one has no spectral bin below
2 Hz). The utilization flag (> 70 %) is reported, not judged.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from teachlab import kinematics as K
from teachlab.recordings import CommandLog, Recording
from teachlab.smoothing import butter_zero_phase

LOW_BAND_HZ = 2.0
RES_BAND_HZ = (2.0, 20.0)
RES_PEAK_DB = 10.0
WELCH_NPERSEG = 2048        # 0.05 Hz bins at 100 Hz: a 0.5 Hz stroke comb is 10 bins wide
MIN_WINDOW_S = 3.0          # below this the rfft has no bin under 2 Hz worth the name
PATH_COMPLETION_MIN = 0.90  # candidate path length / reference path length


def uniform_resample(t: np.ndarray, q: np.ndarray, fs: float):
    """Put the window on a uniform grid at `fs` from its REAL arrival times.
    Dropped frames become interpolated gaps instead of a doubled step."""
    t = np.asarray(t, dtype=float); q = np.asarray(q, dtype=float)
    if t.size < 2:
        return t, q, 0
    tg = np.arange(t[0], t[-1] + 0.5 / fs, 1.0 / fs)
    tg = tg[tg <= t[-1] + 1e-9]
    qg = np.stack([np.interp(tg, t, q[:, j]) for j in range(q.shape[1])], axis=1)
    gaps = int((np.diff(t) > 1.5 / fs).sum())
    return tg, qg, gaps


def _pooled_velocity(rec: Recording, mask: np.ndarray, fs: float):
    """The executed joint velocity to analyse: np.gradient of the uniformly
    resampled position — the SAME estimator for every run, so two runs are
    compared on motion, not on which estimator a consistency gate picked.
    The controller's joint_speed is checked against it and reported (a SIM
    run carries a populated column that is noise, f99 = 50 Hz on
    20260821T210644; a REAL run's column agrees, corr ~0.99)."""
    tg, qg, gaps = uniform_resample(rec.t[mask], rec.pos_deg[mask], fs)
    v_grad = np.gradient(qg, 1.0 / fs, axis=0, edge_order=2)
    note = f"np.gradient(position) on a uniform {fs:.0f} Hz grid ({gaps} gap(s) interpolated)"
    speed_note = "no controller speed field"
    if rec.speed_deg_s is not None:
        v = np.stack([np.interp(tg, rec.t[mask], rec.speed_deg_s[mask][:, j]) for j in range(7)], axis=1)
        a = v.ravel() - v.mean(); b = v_grad.ravel() - v_grad.mean()
        rms_v = float(np.sqrt((v ** 2).mean())); rms_g = float(np.sqrt((v_grad ** 2).mean())) or 1e-12
        corr = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12))
        if 0.5 <= rms_v / rms_g <= 2.0 and corr > 0.8:
            speed_note = f"controller joint_speed agrees (corr {corr:.2f})"
        else:
            speed_note = f"controller joint_speed INCONSISTENT with d(position)/dt (rms ratio {rms_v/rms_g:.2f}, corr {corr:.2f}) — ignored"
    return v_grad, note, speed_note, tg, qg, gaps


def welch_pooled(v: np.ndarray, fs: float, nperseg: Optional[int] = None):
    """Pooled Welch PSD. Pass the SAME nperseg for the runs being compared —
    a line's density scales with the bin width, so two grids cannot be
    compared by level."""
    from scipy.signal import welch
    x = v - v.mean(axis=0, keepdims=True)
    nper = min(nperseg or WELCH_NPERSEG, x.shape[0])
    f, p = welch(x, fs=fs, nperseg=nper, axis=0)
    return f, p.sum(axis=1)


def velocity_spectrum(v: np.ndarray, fs: float, nperseg: Optional[int] = None) -> dict:
    """Hann-windowed pooled velocity spectrum. The low-band fraction INCLUDES
    the DC term: a constant-speed stroke has all its velocity energy at 0 Hz
    and is as smooth as motion gets; removing the mean would judge it on
    its quantisation noise."""
    x = np.asarray(v, dtype=float) * np.hanning(v.shape[0])[:, None]
    p = (np.abs(np.fft.rfft(x, axis=0)) ** 2).sum(axis=1)
    f = np.fft.rfftfreq(x.shape[0], d=1.0 / fs)
    c = np.cumsum(p); tot = c[-1] if c[-1] > 0 else 1.0
    f99 = float(f[min(int(np.searchsorted(c / tot, 0.99)), f.size - 1)])
    f90 = float(f[min(int(np.searchsorted(c / tot, 0.90)), f.size - 1)])
    frac_low = float(p[f <= LOW_BAND_HZ].sum() / tot)
    fw, pw = welch_pooled(v, fs, nperseg)
    return {"f99_hz": f99, "f90_hz": f90, "fraction_below_2hz": frac_low, "window": "hann (DC included)",
            "nperseg": int(min(nperseg or WELCH_NPERSEG, v.shape[0])),
            "welch": {"f": fw, "p": pw}, "rfft": {"f": f[1:], "p": p[1:]}}


def resonance_peaks(fw: np.ndarray, pw: np.ndarray, *, f_ref: Optional[np.ndarray] = None,
                    p_ref: Optional[np.ndarray] = None, f_cmd: Optional[np.ndarray] = None,
                    p_cmd: Optional[np.ndarray] = None, peak_db: float = RES_PEAK_DB) -> list:
    """Candidate resonance lines in RES_BAND_HZ of a Welch PSD.

    A line must (1) stand `peak_db` above a running-median floor of its own
    PSD (local excess, not the global band median); (2) when a reference
    and/or commanded PSD is given, exceed EACH by `peak_db` at the same
    frequency — energy the reference or the command also carries is not the
    candidate's resonance; (3) when NO commanded PSD is given, not sit on
    the candidate's own stroke comb (harmonics of the dominant line below
    LOW_BAND_HZ) — a raster's harmonics are what was asked for. With a
    commanded PSD the comb exclusion is not applied: a resonance that sits
    on a harmonic is still caught by its level over the command. Entries
    without either spectrum are tagged `fallback` (advisory).
    """
    from scipy.ndimage import median_filter
    from scipy.signal import find_peaks
    fw = np.asarray(fw); pw = np.asarray(pw, dtype=float)
    if fw.size < 8:
        return []
    df = float(fw[1] - fw[0])
    kern = max(3, int(round(1.0 / df)) | 1)                    # ~1 Hz running median
    floor = median_filter(np.maximum(pw, 1e-300), size=kern, mode="nearest")
    db_local = 10 * np.log10(np.maximum(pw, 1e-300) / np.maximum(floor, 1e-300))
    band = (fw >= RES_BAND_HZ[0]) & (fw <= RES_BAND_HZ[1])
    if band.sum() < 5:
        return []
    idx_band = np.where(band)[0]
    peaks, _ = find_peaks(db_local[band], height=peak_db, prominence=peak_db / 2)
    # the stroke fundamental (dominant line below 2 Hz), parabolic-refined
    low = (fw > 0) & (fw < LOW_BAND_HZ)
    f0 = None
    if low.any() and pw[low].max() > 0:
        i0 = int(np.where(low)[0][np.argmax(pw[low])])
        f0 = float(fw[i0])
        if 0 < i0 < fw.size - 1 and pw[i0 - 1] > 0 and pw[i0 + 1] > 0:
            a, b, c = np.log(pw[i0 - 1]), np.log(pw[i0]), np.log(pw[i0 + 1])
            den = a - 2 * b + c
            if den < 0:
                f0 = float(fw[i0] + 0.5 * (a - c) / den * df)
    # Fundamental candidates: the parabolic estimate, and the spacing of the
    # STRONG lines (>= 30 dB local excess — only real harmonics reach that;
    # leakage skirts and weak even harmonics do not). A candidate peak is a
    # stroke harmonic if it sits within one bin of a multiple of EITHER
    # estimate, or within three bins of a harmonic line >= 20 dB stronger
    # (the skirt of a strong line).
    f0s = [f0] if f0 else []
    strong_idx, _ = find_peaks(db_local, height=30.0, prominence=10.0)
    strong_f = np.sort(fw[strong_idx]) if strong_idx.size else np.zeros(0)
    if f0 and strong_f.size >= 4:
        sp = np.diff(strong_f)
        near = sp[(sp > 0.7 * f0) & (sp < 1.3 * f0)]
        if near.size >= 3:
            f0s.append(float(np.median(near)))
        near2 = sp[(sp > 1.7 * f0) & (sp < 2.3 * f0)]           # odd-only combs (triangle strokes)
        if near2.size >= 3:
            f0s.append(float(np.median(near2) / 2.0))

    def is_harmonic(freq):
        for f0_ in f0s:
            if f0_ >= 2.0 * df and abs(freq - round(freq / f0_) * f0_) <= 1.0 * df and round(freq / f0_) >= 1:
                return True
        return False

    harmonic_lines = np.array([ff for ff in strong_f if is_harmonic(ff)]) if strong_f.size else np.zeros(0)
    # comb membership, judged LOCALLY: a line with lines one stroke-spacing
    # away on both sides (two bins of tolerance) belongs to the stroke comb
    # whatever the fundamental's estimation error at high order
    all_idx, _ = find_peaks(db_local, prominence=3.0)
    all_f = np.sort(fw[all_idx]) if all_idx.size else np.zeros(0)

    def comb_member(freq):
        if not f0 or all_f.size == 0 or f0 < 2.0 * df:
            return False
        lo_ok = bool(np.any(np.abs(all_f - (freq - f0)) <= 2.0 * df))
        hi_ok = bool(np.any(np.abs(all_f - (freq + f0)) <= 2.0 * df)) or (freq + f0 > RES_BAND_HZ[1])
        return lo_ok and hi_ok

    have_ref = any(ff is not None and pp is not None for ff, pp in ((f_ref, p_ref), (f_cmd, p_cmd)))
    out = []
    for k in peaks:
        i = idx_band[k]; f = float(fw[i])
        entry = {"hz": f, "db_above_local_floor": float(db_local[i])}
        if have_ref:
            # LEVEL is the test: a line the reference/command also carries at
            # this frequency is not the candidate's resonance. Without a
            # commanded spectrum the candidate's own stroke comb is excluded
            # too (a different stroke rate than the reference is not a fault).
            keep = True
            for tag, ff, pp in (("ref", f_ref, p_ref), ("cmd", f_cmd, p_cmd)):
                if ff is not None and pp is not None:
                    pr = float(np.interp(f, np.asarray(ff), np.asarray(pp)))
                    d = 10 * np.log10(max(pw[i], 1e-300) / max(pr, 1e-300))
                    entry[f"db_vs_{tag}"] = float(d)
                    if d < peak_db:
                        keep = False
            if keep and (f_cmd is None or p_cmd is None) and (is_harmonic(f) or comb_member(f)):
                keep = False
                entry["excluded"] = "on the candidate's own stroke comb (no commanded spectrum to test its level against)"
            if keep:
                out.append(entry)
            continue
        # FALLBACK (no reference, no command): frequency-based comb exclusion.
        # Advisory only — it cannot separate a resonance that sits on a harmonic.
        if is_harmonic(f) or comb_member(f):
            continue
        if harmonic_lines.size:
            j = int(np.argmin(np.abs(harmonic_lines - f)))
            fh = harmonic_lines[j]
            ih = int(np.argmin(np.abs(fw - fh)))
            if abs(fh - f) <= 3.0 * df and db_local[ih] - db_local[i] >= 20.0:
                continue                                        # skirt of a much stronger harmonic line
        entry["fallback"] = "no reference/command spectrum: comb-excluded only"
        out.append(entry)
    return out


def contiguous_window(mask: np.ndarray):
    """The largest contiguous run of a boolean mask, and how many runs it had.
    Differentiating across a gap between two stages manufactures a step."""
    m = np.asarray(mask, dtype=bool)
    if not m.any():
        return m, 0
    edges = np.diff(m.astype(int))
    starts = list(np.where(edges == 1)[0] + 1); ends = list(np.where(edges == -1)[0] + 1)
    if m[0]:
        starts = [0] + starts
    if m[-1]:
        ends = ends + [m.size]
    segs = [(a, b) for a, b in zip(starts, ends) if b > a]
    a, b = max(segs, key=lambda s: s[1] - s[0])
    out = np.zeros_like(m); out[a:b] = True
    return out, len(segs)


def run_metrics(rec: Recording, kin: K.ArmKinematics, mask: Optional[np.ndarray] = None, *,
                v_moving_mps: float = 0.05, tcp_fc_hz: float = 5.0, jerk_fc_hz: float = 5.0,
                nperseg: Optional[int] = None) -> dict:
    if mask is None:
        mask = np.ones(rec.n, dtype=bool)
    mask, n_segments = contiguous_window(mask)
    if mask.sum() < 20:
        raise ValueError("window has fewer than 20 samples")
    fs = rec.fs_hz
    if not np.isfinite(fs) or fs <= 0:
        raise ValueError(f"cannot establish a sample rate from the arrival times (fs={fs})")
    v, vsrc, speed_note, t, q, gaps = _pooled_velocity(rec, mask, fs)      # uniform grid from here on
    v_grad = v
    spec = velocity_spectrum(v, fs, nperseg)
    peaks_self = resonance_peaks(spec["welch"]["f"], spec["welch"]["p"])
    v_pos10 = np.gradient(butter_zero_phase(q, fs, 10.0, 2), 1.0 / fs, axis=0, edge_order=2) if 10.0 < 0.5 * fs else v_grad
    util = [float(np.abs(v_pos10[:, j]).max() / K.QDOT_MAX_DEG_S[j]) for j in range(7)]
    util_ctrl = ([float(np.abs(rec.speed_deg_s[mask][:, j]).max() / K.QDOT_MAX_DEG_S[j]) for j in range(7)]
                 if rec.speed_deg_s is not None else None)
    window_s = float(t[-1] - t[0]) if t.size > 1 else 0.0
    qs = butter_zero_phase(q, fs, jerk_fc_hz, 2) if jerk_fc_hz < 0.5 * fs else q
    dt = 1.0 / fs
    jerk = np.gradient(np.gradient(np.gradient(qs, dt, axis=0, edge_order=2), dt, axis=0, edge_order=2), dt, axis=0, edge_order=2)
    q_tcp = qs if tcp_fc_hz == jerk_fc_hz else butter_zero_phase(q, fs, tcp_fc_hz, 2)
    pos, rot = kin.fk_batch(np.radians(q_tcp))
    vel = np.gradient(pos, dt, axis=0, edge_order=2); speed = np.linalg.norm(vel, axis=1)
    d = np.linalg.norm(np.diff(pos, axis=0), axis=1); path = float(d.sum())
    dur = float(t[-1] - t[0]) if t.size > 1 else 0.0
    moving = speed > v_moving_mps
    sp_mov = speed[moving]
    cv = float(sp_mov.std() / sp_mov.mean()) if sp_mov.size > 1 and sp_mov.mean() > 0 else None
    idx = np.where(moving)[0]
    if idx.size >= 2:
        i0, i1 = int(idx[0]), int(idx[-1])
        span_s = float(t[i1] - t[i0]); span_len = float(d[i0:i1].sum())
    else:
        span_s, span_len = 0.0, 0.0
    max_exec_step = [float(x) for x in np.abs(np.diff(q, axis=0)).max(axis=0)] if q.shape[0] > 1 else [0.0] * 7
    return {
        "run_id": rec.run_id, "n": int(mask.sum()), "fs_hz": fs, "duration_s": dur,
        "window_segments_requested": n_segments,
        "window_note": ("largest contiguous segment of a multi-segment window" if n_segments > 1 else "contiguous"),
        "window_too_short": bool(window_s < MIN_WINDOW_S), "min_window_s": MIN_WINDOW_S,
        "gaps_interpolated": gaps,
        "interarrival_ms": rec.interarrival_stats_ms(), "velocity_source": vsrc, "speed_field": speed_note,
        "spectrum": {"f99_hz": spec["f99_hz"], "f90_hz": spec["f90_hz"],
                     "fraction_below_2hz": spec["fraction_below_2hz"], "window": spec["window"],
                     "nperseg": spec["nperseg"], "resonance_peaks_self": peaks_self},
        "_spectrum_arrays": {"welch": spec["welch"], "rfft": spec["rfft"]},
        "utilization": util, "max_utilization": float(max(util)), "binding_joint": int(np.argmax(util)) + 1,
        "utilization_basis": "peak |d(position)/dt| after 10 Hz zero-phase Butterworth / qdot_max (xacro)",
        "utilization_controller_speed_field": util_ctrl,
        "peak_vel_deg_s": [float(x) for x in np.abs(v_pos10).max(axis=0)],
        "post_filter_jerk": {"fc_hz": jerk_fc_hz, "peak_deg_s3": [float(x) for x in np.abs(jerk).max(axis=0)],
                             "rms_deg_s3": [float(x) for x in np.sqrt((jerk ** 2).mean(axis=0))]},
        "tcp": {"frame": kin.frame, "path_length_m": path, "mean_speed_full_mps": path / dur if dur > 0 else None,
                "active_span_s": span_s, "mean_speed_active_span_mps": span_len / span_s if span_s > 0 else None,
                "speed_cv_moving": cv, "moving_fraction": float(moving.mean()),
                "speed_max_mps": float(speed.max()), "speed_p95_mps": float(np.percentile(speed, 95)),
                "speed_median_moving_mps": float(np.median(sp_mov)) if sp_mov.size else None},
        "max_executed_step_deg": max_exec_step,
        "arm_status_values": sorted(set(int(x) for x in rec.arm_status[mask])) if rec.arm_status is not None else None,
        "errors_nonzero": int((rec.err[mask] != 0).sum()) if rec.err is not None else None,
        "_arrays": {"t": t, "speed": speed, "moving": moving, "pos": pos, "q": q, "v": v},
    }


def teach_velocity_psd(q_deg: np.ndarray, period_s: float, fs: float, nperseg: Optional[int] = None):
    """Welch PSD of the velocity a teach file COMMANDS when the controller
    replays it at `period_s`, resampled to the executed rate — the commanded
    spectrum of a drag replay, for the resonance comparison."""
    n = q_deg.shape[0]
    t_src = np.arange(n) * period_s
    t = np.arange(0.0, t_src[-1], 1.0 / fs)
    q = np.stack([np.interp(t, t_src, q_deg[:, j]) for j in range(q_deg.shape[1])], axis=1)
    v = np.gradient(q, 1.0 / fs, axis=0, edge_order=2)
    return welch_pooled(v, fs, nperseg)


MIN_COMMAND_FRAMES = 10


def commanded_velocity_psd(cmd: CommandLog, fs: float, nperseg: Optional[int] = None):
    """Welch PSD of the commanded joint velocity, resampled onto a uniform grid
    at the executed rate so it lives on the same frequency axis. None for a
    log too short to differentiate."""
    if cmd.n < MIN_COMMAND_FRAMES or (cmd.t_sent[-1] - cmd.t_sent[0]) < 3.0 / fs:
        return None
    t = np.arange(cmd.t_sent[0], cmd.t_sent[-1], 1.0 / fs)
    q = np.stack([np.interp(t, cmd.t_sent, cmd.q_deg[:, j]) for j in range(7)], axis=1)
    if q.shape[0] < 3:
        return None
    v = np.gradient(q, 1.0 / fs, axis=0, edge_order=2)
    return welch_pooled(v, fs, nperseg)


def tracking_metrics(rec: Recording, cmd: CommandLog, period_s: float, *, max_lag_s: float = 0.5,
                     max_lead_s: float = 0.2) -> dict:
    """Commanded (command.csv) vs executed (stream) joints on the executed
    time base; lag = the integer-sample shift that minimises the RMS error,
    searched from -max_lead_s (a log stamped after the fact) to +max_lag_s."""
    t_exec = rec.t
    if cmd.n < MIN_COMMAND_FRAMES:
        return {"available": False, "reason": f"command log has {cmd.n} frames (< {MIN_COMMAND_FRAMES})"}
    # errors are only meaningful where a real command exists: commanded[i] is
    # compared with executed[i + L], so the window ends at the last command
    lo, hi = cmd.t_sent[0], cmd.t_sent[-1]
    m = (t_exec >= lo) & (t_exec <= hi)
    te = t_exec[m]; qe = rec.pos_deg[m]
    if te.size < 20:
        return {"available": False, "reason": "no executed samples overlap the command window"}
    qc = np.stack([np.interp(te, cmd.t_sent, cmd.q_deg[:, j]) for j in range(7)], axis=1)
    fs = rec.fs_hz
    max_lag = int(max_lag_s * fs)
    lags, rms, mx = [], [], []
    n = qc.shape[0]
    for j in range(7):
        if (qc[:, j].max() - qc[:, j].min()) < 0.05:  # joint barely commanded: lag undefined
            lags.append(None); rms.append(float(np.sqrt(((qc[:, j] - qe[:, j]) ** 2).mean()))); mx.append(float(np.abs(qc[:, j] - qe[:, j]).max()))
            continue
        best, best_r = 0, np.inf
        max_lead = int(max_lead_s * fs)
        for L in range(-min(max_lead, n - 10), min(max_lag, n - 10) + 1):
            e = (qc[:n - L, j] - qe[L:, j]) if L >= 0 else (qc[-L:, j] - qe[:n + L, j])
            r = float(np.sqrt((e ** 2).mean()))
            if r < best_r:
                best, best_r = L, r
        shifted_err = (qc[:n - best, j] - qe[best:, j]) if best >= 0 else (qc[-best:, j] - qe[:n + best, j])
        lags.append(best / fs); rms.append(best_r); mx.append(float(np.abs(shifted_err).max()))
    steps = cmd.max_step_deg()
    lim_rate = K.CANFD_MAX_RATE_DEG_S * period_s
    return {"available": True, "lag_s": lags, "rms_after_lag_deg": rms, "max_after_lag_deg": mx,
            "raw_rms_deg": [float(np.sqrt(((qc[:, j] - qe[:, j]) ** 2).mean())) for j in range(7)],
            "max_cmd_step_deg": steps, "step_limit_rate_deg": lim_rate, "step_limit_vendor_deg": K.CANFD_MAX_STEP_DEG,
            "step_flags": [("OVER_VENDOR_10deg" if s > K.CANFD_MAX_STEP_DEG else
                            "OVER_180deg_s" if s > lim_rate else "ok") for s in steps],
            "period_stats_ms": cmd.period_stats_ms(), "frames": cmd.n,
            "sdk_errors": int((cmd.ret != 0).sum())}


def verdict(ref: dict, cand: dict, cand_cmd_psd=None, *, same_trajectory: bool = True,
            cand_stage_ok: Optional[bool] = None) -> dict:
    """Candidate vs reference — the success criterion from the brief.

    `cand_cmd_psd` = (f, p) of the candidate's commanded velocity when a
    command.csv exists; resonance is then judged against the reference AND
    the command. `same_trajectory` enables the path-completion gate (the
    candidate must trace >= PATH_COMPLETION_MIN of the reference path);
    `cand_stage_ok` = False when a selected stage reported ok=False."""
    rs, cs = ref["spectrum"], cand["spectrum"]
    fr, pr = ref["_spectrum_arrays"]["welch"]["f"], ref["_spectrum_arrays"]["welch"]["p"]
    fc, pc = cand["_spectrum_arrays"]["welch"]["f"], cand["_spectrum_arrays"]["welch"]["p"]
    kw = {"f_ref": fr, "p_ref": pr}
    if cand_cmd_psd is not None:
        kw.update(f_cmd=cand_cmd_psd[0], p_cmd=cand_cmd_psd[1])
    peaks = resonance_peaks(fc, pc, **kw)
    low_ok = cs["fraction_below_2hz"] >= max(0.90, rs["fraction_below_2hz"] - 0.02)
    spectrum_ok = bool(low_ok and not peaks)
    cv_r, cv_c = ref["tcp"]["speed_cv_moving"], cand["tcp"]["speed_cv_moving"]
    cv_ok = bool(cv_c is not None and cv_r is not None and cv_c <= cv_r * 1.05)
    util_flag = cand["max_utilization"] > 0.70
    pr_, pc_ = ref["tcp"].get("path_length_m"), cand["tcp"].get("path_length_m")
    if pr_ is None or pc_ is None:
        same_trajectory = False                       # no path lengths to compare (synthetic metrics)
    path_ratio = (pc_ / pr_) if pr_ and pr_ > 0 else None
    path_ok = (not same_trajectory) or bool(path_ratio is not None and path_ratio >= PATH_COMPLETION_MIN)
    window_ok = not (ref.get("window_too_short") or cand.get("window_too_short"))
    stage_ok = cand_stage_ok is not False
    ok = bool(spectrum_ok and cv_ok and path_ok and window_ok and stage_ok)
    notes = [
        f"velocity energy < 2 Hz: ref {rs['fraction_below_2hz']:.3f} vs cand {cs['fraction_below_2hz']:.3f} (f99 {rs['f99_hz']:.2f} / {cs['f99_hz']:.2f} Hz)",
        f"resonance peaks vs reference{'/command' if cand_cmd_psd is not None else ' (candidate stroke comb excluded, no command spectrum)'}: {len(peaks)}",
        f"TCP speed CV ref {cv_r} vs cand {cv_c}",
        "utilization > 70% — revisit the timing scheme (exp02 trigger)" if util_flag else "utilization under 70%",
    ]
    if same_trajectory:
        notes.append(f"path completion: cand/ref = {path_ratio:.3f}" if path_ratio is not None else "path completion: reference path is zero")
    if not window_ok:
        notes.append(f"window shorter than {MIN_WINDOW_S:.0f} s on at least one side — no verdict is possible on such a window")
    if cand_stage_ok is False:
        notes.append("a selected candidate stage reported ok=False (the SDK call failed)")
    return {"spectrum_ok": spectrum_ok, "low_band_ok": bool(low_ok), "resonance_peaks_vs_reference": peaks,
            "cv_ok": cv_ok, "path_ok": bool(path_ok), "path_ratio": path_ratio, "window_ok": bool(window_ok),
            "stage_ok": bool(stage_ok), "pass": ok,
            "utilization_over_70pct": bool(util_flag),
            "criteria": {"low_band": "fraction of velocity energy < 2 Hz (DC incl.) >= max(0.90, ref - 0.02); one estimator (position gradient, uniform grid) for both runs",
                         "resonance": ">= 10 dB local peak in 2-20 Hz AND >= 10 dB above the reference (and the command when present) at that frequency; without a command spectrum the candidate's own stroke comb is excluded",
                         "cv": "TCP speed CV (moving) <= 1.05 x reference",
                         "path": f"candidate path length >= {PATH_COMPLETION_MIN:.2f} x reference (same trajectory)" if same_trajectory else "not applied (different trajectories)",
                         "window": f"both windows >= {MIN_WINDOW_S:.0f} s and no selected stage with ok=False"},
            "notes": notes}


def _f(x, nd=3):
    return "—" if x is None else (f"{x:.{nd}f}" if isinstance(x, float) else str(x))


def markdown_table(ref: dict, cand: dict, vd: dict, ref_track: Optional[dict] = None,
                   cand_track: Optional[dict] = None, labels=("reference", "candidate")) -> str:
    rows = [
        ("samples / fs Hz", f"{ref['n']} / {ref['fs_hz']:.1f}", f"{cand['n']} / {cand['fs_hz']:.1f}"),
        ("window", ref["window_note"], cand["window_note"]),
        ("arrival p99 / max ms", f"{_f(ref['interarrival_ms'].get('p99'),2)} / {_f(ref['interarrival_ms'].get('max'),2)}",
         f"{_f(cand['interarrival_ms'].get('p99'),2)} / {_f(cand['interarrival_ms'].get('max'),2)}"),
        ("velocity source", ref["velocity_source"], cand["velocity_source"]),
        ("controller speed field", ref.get("speed_field", "—"), cand.get("speed_field", "—")),
        ("velocity f90 / f99 Hz", f"{ref['spectrum']['f90_hz']:.2f} / {ref['spectrum']['f99_hz']:.2f}",
         f"{cand['spectrum']['f90_hz']:.2f} / {cand['spectrum']['f99_hz']:.2f}"),
        ("velocity energy < 2 Hz", _f(ref["spectrum"]["fraction_below_2hz"]), _f(cand["spectrum"]["fraction_below_2hz"])),
        ("resonance peaks 2–20 Hz (self-relative, advisory)", str(len(ref["spectrum"]["resonance_peaks_self"])), str(len(cand["spectrum"]["resonance_peaks_self"]))),
        ("resonance peaks vs reference/command (the verdict)", "—", str(len(vd["resonance_peaks_vs_reference"]))),
        ("TCP path m / window s", f"{ref['tcp']['path_length_m']:.3f} / {ref['duration_s']:.2f}", f"{cand['tcp']['path_length_m']:.3f} / {cand['duration_s']:.2f}"),
        ("AVERAGE TCP speed incl. ramps m/s (productivity; target 0.35–0.5, advisory)", _f(ref["tcp"]["mean_speed_full_mps"]), _f(cand["tcp"]["mean_speed_full_mps"])),
        ("path completion cand/ref (>= 0.90 when same trajectory)", "—", _f(vd.get("path_ratio"))),
        ("TCP speed active span m/s", _f(ref["tcp"]["mean_speed_active_span_mps"]), _f(cand["tcp"]["mean_speed_active_span_mps"])),
        ("TCP speed median moving / max m/s", f"{_f(ref['tcp']['speed_median_moving_mps'])} / {_f(ref['tcp']['speed_max_mps'])}",
         f"{_f(cand['tcp']['speed_median_moving_mps'])} / {_f(cand['tcp']['speed_max_mps'])}"),
        ("TCP speed CV (moving)", _f(ref["tcp"]["speed_cv_moving"]), _f(cand["tcp"]["speed_cv_moving"])),
        ("max utilization (joint) [10 Hz pos-derivative]", f"{ref['max_utilization']:.2f} (j{ref['binding_joint']})", f"{cand['max_utilization']:.2f} (j{cand['binding_joint']})"),
        ("max utilization from controller speed field", _f(max(ref['utilization_controller_speed_field']) if ref['utilization_controller_speed_field'] else None, 2),
         _f(max(cand['utilization_controller_speed_field']) if cand['utilization_controller_speed_field'] else None, 2)),
        ("post-filter jerk peak deg/s³", f"{max(ref['post_filter_jerk']['peak_deg_s3']):.0f}", f"{max(cand['post_filter_jerk']['peak_deg_s3']):.0f}"),
        ("max executed step deg/cycle", f"{max(ref['max_executed_step_deg']):.3f}", f"{max(cand['max_executed_step_deg']):.3f}"),
    ]
    for lab, tr in (("ref", ref_track), ("cand", cand_track)):
        if tr and tr.get("available"):
            rows.append((f"{lab}: max commanded step deg/cycle", ", ".join(f"{s:.2f}" for s in tr["max_cmd_step_deg"]), "/".join(tr["step_flags"])))
            rows.append((f"{lab}: tracking lag s (per joint)", ", ".join(_f(l, 3) for l in tr["lag_s"]), ""))
            rows.append((f"{lab}: tracking RMS after lag deg", ", ".join(f"{v:.3f}" for v in tr["rms_after_lag_deg"]), f"max {max(tr['max_after_lag_deg']):.3f}"))
    out = [f"| metric | {labels[0]} ({ref['run_id']}) | {labels[1]} ({cand['run_id']}) |", "|---|---|---|"]
    out += [f"| {a} | {b} | {c} |" for a, b, c in rows]
    out += ["", f"**Verdict:** {'PASS' if vd['pass'] else 'FAIL'} — low band {'ok' if vd['low_band_ok'] else 'NOT ok'}, "
            f"resonance {'none' if not vd['resonance_peaks_vs_reference'] else 'FOUND'}, "
            f"TCP speed CV {'ok' if vd['cv_ok'] else 'NOT ok'}, path {'ok' if vd.get('path_ok', True) else 'NOT ok'}, "
            f"window {'ok' if vd.get('window_ok', True) and vd.get('stage_ok', True) else 'NOT ok'}; " + "; ".join(vd["notes"]),
            "", "Criteria: " + "; ".join(f"{k}: {v}" for k, v in vd["criteria"].items())]
    return "\n".join(out)
