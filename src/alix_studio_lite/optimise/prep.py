"""Preparation steps from the trajectory prototype (``docs/reference/Alix_Studio_Lite_Trajectory_V2.html``)
and from ALIX's taught-path chain.

The prototype functions are ported line for line so that ``prototype_reference`` reproduces
the prototype's output exactly (for regression comparison). Lite's fixes are options on top:

- F-a: ``rolling_mean(..., edge="odd")`` mirrors the series through its end value instead of
  repeating it, so the smoothed file keeps the recording's first and last pose and slope with
  no forced end rows (the prototype's ends jumped 0.5-1.0 deg in one line);
- F-c: a 41-line window instead of 101 (the 101 window moved the hand 10-21 mm off the path).

F-b (blended boundaries) is in ``default_opt``.
"""
from __future__ import annotations

import math

import numpy as np

from .types import Segment

# ─────────────────────────────────────────────────────────────── prototype ports


def js_round(x) -> np.ndarray:
    """JavaScript's ``Math.round``: halves go up (towards +infinity)."""
    return np.floor(np.asarray(x, dtype=float) + 0.5)


def centered_rolling_mean_proto(values: np.ndarray, window: int) -> np.ndarray:
    """``centeredRollingMean`` exactly: ``window // 2`` either side, edges repeated, and the
    sum divided by ``window`` (for an even window it sums ``window + 1`` values, as the
    prototype does)."""
    x = np.asarray(values, dtype=float)
    n = x.shape[0]
    half = int(window) // 2
    c = np.concatenate([[0.0], np.cumsum(x)])
    i = np.arange(n)
    raw_start, raw_end = i - half, i + half
    start, end = np.maximum(0, raw_start), np.minimum(n - 1, raw_end)
    s = c[end + 1] - c[start]
    s = s + np.where(raw_start < 0, -raw_start, 0) * x[0]
    s = s + np.where(raw_end >= n, raw_end - (n - 1), 0) * x[-1]
    return s / float(window)


def rolling_mean(x: np.ndarray, window: int, edge: str = "odd") -> np.ndarray:
    """Centred moving mean along axis 0 over an odd ``window``.

    ``edge="odd"`` (fix F-a) pads with ``2 x[0] - x[k]`` and ``2 x[-1] - x[-1-k]``: the mean of a
    window centred on an end is then the end value itself, so the ends stay put. ``edge="edge"``
    repeats the end values (the prototype's padding)."""
    x = np.asarray(x, dtype=float)
    n = x.shape[0]
    w = int(window)
    if w % 2 == 0:
        w += 1
    half = min(w // 2, n - 1)
    if half < 1:
        return x.copy()
    w = 2 * half + 1
    if edge == "odd":
        left = 2.0 * x[0] - x[half:0:-1]
        right = 2.0 * x[-1] - x[-2:-half - 2:-1]
    elif edge == "edge":
        left = np.repeat(x[:1], half, axis=0)
        right = np.repeat(x[-1:], half, axis=0)
    else:
        raise ValueError(f"edge must be 'odd' or 'edge', got {edge!r}")
    xp = np.concatenate([left, x, right], axis=0)
    c = np.concatenate([np.zeros((1,) + x.shape[1:]), np.cumsum(xp, axis=0)], axis=0)
    return (c[w:] - c[:-w]) / w


def trim_idle(md: np.ndarray, window: int = 11, threshold_md: float = 8.0,
              min_run: int = 20) -> tuple[np.ndarray, np.ndarray, list]:
    """``trimIdle``: a line step is idle when the centred mean of |step| over ``window`` steps is
    at most ``threshold_md`` on every joint. In each run of at least ``min_run`` idle steps the
    interior lines are removed and the run's two end lines kept.

    Returns (kept lines, kept 0-based recording rows, runs as (first, last) 0-based rows)."""
    md = np.asarray(md)
    n = md.shape[0]
    if n < 2:
        return md.copy(), np.arange(n), []
    d = np.abs(np.diff(md.astype(float), axis=0))
    sm = np.column_stack([centered_rolling_mean_proto(d[:, j], window) for j in range(md.shape[1])])
    idle = (sm <= float(threshold_md)).all(axis=1)
    remove = np.zeros(n, dtype=bool)
    runs = []
    i, m = 0, n - 1
    while i < m:
        if not idle[i]:
            i += 1
            continue
        a = i
        while i + 1 < m and idle[i + 1]:
            i += 1
        b = i
        if b - a + 1 >= int(min_run):
            remove[a + 1:b + 1] = True
            runs.append((a, b + 1))
        i += 1
    keep = ~remove
    return md[keep], np.flatnonzero(keep), runs


def mean_smooth_proto(md: np.ndarray, window: int) -> np.ndarray:
    """``meanSmooth`` exactly: repeated-edge moving mean, a constant correction that restores
    each joint's mean, JavaScript rounding, then the first and last rows forced back."""
    md = np.asarray(md)
    out = np.empty(md.shape, dtype=float)
    for j in range(md.shape[1]):
        y = md[:, j].astype(float)
        ys = centered_rolling_mean_proto(y, window)
        out[:, j] = js_round(ys + (y.mean() - ys.mean()))
    out[0], out[-1] = md[0], md[-1]
    return out.astype(np.int64)


def map_segments(segments, kept_rows: np.ndarray) -> list[tuple[int, int, Segment]]:
    """``mapRegionsToTrimmed``: each segment's (first, last) index into the kept lines.
    Segments left without lines are dropped; overlaps after trimming are an error."""
    out = []
    for s in sorted(segments, key=lambda s: s.start):
        lo = int(np.searchsorted(kept_rows, s.start - 1, side="left"))
        hi = int(np.searchsorted(kept_rows, s.end - 1, side="right")) - 1
        if lo >= kept_rows.shape[0] or hi < lo:
            continue
        if out and lo <= out[-1][1]:
            raise ValueError("two segments overlap; adjust them so they do not share lines")
        out.append((lo, hi, s))
    return out


def interpolate_region_proto(region: np.ndarray, pct: float) -> np.ndarray:
    """``interpolateRegion`` exactly: linear resample to round((len-1) x pct/100) + 1 lines."""
    old = region.shape[0]
    if old <= 1:
        return region.copy()
    new_int = max(1, int(js_round((old - 1) * float(pct) / 100.0)))
    k = np.arange(new_int + 1)
    pos = k / new_int * (old - 1)
    i0 = np.floor(pos).astype(int)
    i1 = np.minimum(old - 1, i0 + 1)
    a = (pos - i0)[:, None]
    out = js_round(region[i0] + a * (region[i1] - region[i0]))
    out[0], out[-1] = region[0], region[-1]
    return out.astype(np.int64)


def prototype_reference(md_raw: np.ndarray, segments=(), *, trim: bool = True, smooth: bool = True,
                        idle_window: int = 11, idle_threshold_md: float = 8.0, idle_min_run: int = 20,
                        smooth_window: int = 101) -> np.ndarray:
    """The prototype's ``processTrajectory`` output, byte for byte (its defaults: window 101,
    every stage on). For comparison only: it has no limit checks."""
    md_raw = np.asarray(md_raw, dtype=np.int64)
    if trim:
        data, kept, _ = trim_idle(md_raw, idle_window, idle_threshold_md, idle_min_run)
    else:
        data, kept = md_raw.copy(), np.arange(md_raw.shape[0])
    if smooth:
        data = mean_smooth_proto(data, smooth_window)
    if segments:
        parts, cursor = [], 0
        for lo, hi, s in map_segments(segments, kept):
            parts.append(data[cursor:lo])
            parts.append(interpolate_region_proto(data[lo:hi + 1].astype(float), s.pct))
            cursor = hi + 1
        parts.append(data[cursor:])
        data = np.concatenate([p for p in parts if p.shape[0]], axis=0).astype(np.int64)
    data = data.copy()
    data[0], data[-1] = md_raw[0], md_raw[-1]
    return data


# ─────────────────────────────────────────────────────────── segment suggestions


def _percentile_js(values, p: float) -> float:
    v = sorted(values)
    if not v:
        return 0.0
    pos = (len(v) - 1) * p
    lo, hi = math.floor(pos), math.ceil(pos)
    return v[lo] if lo == hi else v[lo] * (1 - (pos - lo)) + v[hi] * (pos - lo)


def suggest_segments(md_raw: np.ndarray, sample_ms: float = 5.0) -> list[dict]:
    """``analyzeMotionRegions``: 5-10 equal slices with a suggested time % and the reason, for
    the Default optimiser's segment table. Each item has ``segment`` (a ``Segment``) and the
    prototype's ``motion_type``, ``action`` and ``reason`` texts."""
    data = np.asarray(md_raw, dtype=float)
    n = data.shape[0]
    step = np.zeros(n)
    step[1:] = np.sqrt((np.diff(data, axis=0) ** 2).sum(axis=1))
    accel = np.zeros(n)
    accel[2:] = np.abs(step[2:] - step[1:-1])
    count = min(10, max(5, int(js_round(n / 3000))))
    raw = []
    for r in range(count):
        s0 = (r * n) // count
        e0 = max(s0, ((r + 1) * n) // count - 1)
        seg = slice(s0, e0 + 1)
        cnt = max(1, e0 - s0 + 1)
        raw.append(dict(start=s0 + 1, end=e0 + 1, avg_step=step[seg].sum() / cnt,
                        max_step=step[seg].max(), avg_accel=accel[seg].sum() / cnt,
                        original_s=cnt * sample_ms / 1000.0))
    q25s = _percentile_js([r["avg_step"] for r in raw], 0.25)
    q50s = _percentile_js([r["avg_step"] for r in raw], 0.50)
    q75m = _percentile_js([r["max_step"] for r in raw], 0.75)
    q25a = _percentile_js([r["avg_accel"] for r in raw], 0.25)
    q50a = _percentile_js([r["avg_accel"] for r in raw], 0.50)
    q75a = _percentile_js([r["avg_accel"] for r in raw], 0.75)
    out = []
    for idx, r in enumerate(raw):
        kind, action, pct = "General continuous motion", "Slightly shorter", 95
        reason = "Balanced motion; a small reduction, checked in simulation."
        if idx == 0:
            kind, action, pct = "Approach", "Shorter / faster", 75
            reason = "The start is usually a transition, less sensitive than the cleaning path."
        elif idx == len(raw) - 1:
            kind, action, pct = "Exit", "Shorter / faster", 75
            reason = "The end is usually a transition away from the work area."
        elif r["avg_accel"] >= q75a and r["avg_accel"] > 0:
            kind, action, pct = "Direction changes", "Longer / slower", 115
            reason = "Many speed changes suggest turns; slow down here."
        elif r["max_step"] >= q75m and r["avg_accel"] >= q50a:
            kind, action, pct = "High peaks", "Keep or slightly slower", 105
            reason = "Peak joint movement is high; avoid compressing further."
        elif r["avg_step"] <= q25s:
            kind, action, pct = "Slow / possibly redundant", "Shorter", 80
            reason = "Little movement: it may contain waiting that trimming did not remove."
        elif r["avg_accel"] <= q25a and r["avg_step"] >= q50s:
            kind, action, pct = "Smooth pass", "Moderately shorter", 85
            reason = "Smooth continuous motion with few direction changes."
        out.append(dict(segment=Segment(r["start"], r["end"], pct, kind), motion_type=kind,
                        action=action, reason=reason, original_s=r["original_s"]))
    return out


# ─────────────────────────────────────────────────────────── ALIX chain helpers


def pause_mask(q_deg: np.ndarray, tol_deg: float = 0.05, min_lines: int = 3) -> np.ndarray:
    """The rows ``taught_tracks.strip_pauses`` keeps (it returns only the kept rows; Lite also
    needs which ones, to map segments). A run of at least ``min_lines`` lines all within
    ``tol_deg`` of the run's first line collapses to that first line; the last line is kept.
    Parity with ALIX is tested."""
    q = np.asarray(q_deg, dtype=float)
    n = q.shape[0]
    keep = np.ones(n, dtype=bool)
    if n < 3:
        return keep
    k = 0
    while k < n - 1:
        anchor = q[k]
        j = k + 1
        while j < n and np.abs(q[j] - anchor).max() <= tol_deg:
            j += 1
        if j - k >= int(min_lines):
            keep[k + 1:j] = False
            k = j
        else:
            k += 1
    keep[-1] = True
    return keep
