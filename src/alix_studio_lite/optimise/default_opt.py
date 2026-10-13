"""The DEFAULT optimiser: the trajectory prototype's algorithm with Lite's fixes.

1. Trim idle stretches (the prototype's rule, unchanged).
2. Smooth with a centred moving mean, mirrored at the ends (fix F-a), 41 lines by default
   (fix F-c; the path change is checked as L5).
3. Time: each line keeps its recorded 5 ms, times the segment's % where the operator set one.
   The % steps are blended over 0.25 s (fix F-b), so the speed changes gradually at a segment
   boundary instead of in one line (the prototype's boundaries reached 640-919 deg/s^2).
4. The whole time axis is then stretched or compressed uniformly to the line count the target
   needs; every new line lies on the smoothed polyline (linear interpolation on its chords).
"""
from __future__ import annotations

import numpy as np

from ..motion import track
from ..motion.checks import util_series
from ..motion.limits import LimitSet
from ..motion.timing import DT_RECORD_S
from . import prep
from .types import DefaultOptions


class DefaultPlan:
    optimiser = "default"

    def __init__(self, md_raw: np.ndarray, limits: LimitSet, opts: DefaultOptions, segments=()):
        from scipy.ndimage import uniform_filter1d

        self.limits = limits
        self.opts = opts
        md_raw = np.asarray(md_raw, dtype=np.int64)
        if opts.trim:
            md_t, rows, runs = prep.trim_idle(md_raw, opts.idle_window, opts.idle_threshold_md,
                                              opts.idle_min_run)
        else:
            md_t, rows, runs = md_raw, np.arange(md_raw.shape[0]), []
        x = track.to_deg(md_t)
        if opts.smooth_window and int(opts.smooth_window) > 1:
            x = prep.rolling_mean(x, int(opts.smooth_window), edge="odd")
        self.P = x
        self.rows = rows
        w = np.ones(x.shape[0] - 1)
        self.segments = prep.map_segments(segments, rows)
        for lo, hi, s in self.segments:
            w[lo:hi] = float(s.pct) / 100.0                  # intervals lo .. hi-1
        if self.segments and opts.blend_s > 0:
            size = max(1, int(round(opts.blend_s / DT_RECORD_S))) | 1
            for _ in range(2):
                w = uniform_filter1d(w, size=size, mode="nearest")
        self.tau = np.concatenate([[0.0], np.cumsum(w)]) * DT_RECORD_S
        self.info = dict(trimmed_lines=int(md_raw.shape[0] - md_t.shape[0]), idle_runs=len(runs),
                         kept_lines=int(md_t.shape[0]),
                         smooth_window=int(opts.smooth_window) | 1 if opts.smooth_window > 1 else 0,
                         segments=[dict(start=s.start, end=s.end, pct=s.pct, label=s.label,
                                        first_kept=int(lo), last_kept=int(hi))
                                   for lo, hi, s in self.segments])
        # the search starts at the recorded pace (with the segment %)
        self.n_start = max(1, int(round(self.tau[-1] / limits.dt_gate)))
        self.n_floor = 1
        self.n_cap = max(20 * self.n_start, 100)

    def make(self, n: int) -> np.ndarray:
        """``n`` line intervals (``n + 1`` lines) on the blended time axis."""
        t = np.linspace(0.0, self.tau[-1], int(n) + 1)
        q = np.column_stack([np.interp(t, self.tau, self.P[:, j]) for j in range(self.P.shape[1])])
        return track.to_md(q)

    def util(self, n: int) -> dict:
        return util_series(self.make(n), self.limits)

    def recipe(self) -> dict:
        return dict(self.info)


def smoothing_hint(md_raw: np.ndarray, limits: LimitSet, opts: DefaultOptions) -> str | None:
    """When L5 fails, the largest smaller window that keeps the hand on the taught path."""
    import taught_tracks as tt
    from dataclasses import replace

    raw = track.to_deg(md_raw)
    for w in (31, 21, 15, 11, 7):
        if w >= opts.smooth_window:
            continue
        plan = DefaultPlan(md_raw, limits, replace(opts, smooth_window=w))
        dev = tt.deviation_mm(raw, plan.P, limits.side, limits.frame)["max_mm"]
        if dev <= limits.max_dev_mm:
            return (f"Reduce smoothing to {w} lines: the hand then stays within {dev:.1f} mm of the "
                    f"taught path (limit {limits.max_dev_mm:g} mm).")
    return "Turn smoothing off, or re-teach this region: even light smoothing moves the hand too far."
