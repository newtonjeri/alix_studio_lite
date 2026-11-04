"""A trajectory slowed over parts of its time, along the same joint path (the user, 4 Nov: "reduce the speed of those
trajectory files at those specific regions, since 5% speed reduction causes them to pass").

Each window plays ``factor`` times slower. Its lines are resampled linearly between the file's own lines, so every
new line lies on the segment between two of the file's lines, as the manual tuner's Scale does
(optimise/manual.interpolate_region). The slow-down comes in and goes out over ``ramp`` lines (a half cosine), so
the speed changes smoothly. Scale's step of 5 % at 170 deg/s would be some 1700 deg/s2 in one 5 ms line.

The first and last lines stay as they are, so the program starts and ends where it did. The time added is a whole
number of 5 ms lines (``timing.CYCLE_S``).
"""
from __future__ import annotations

import numpy as np

from . import timing

SUB = 16                    # integration steps per line for the time warp


def merge(windows) -> list[tuple[float, float]]:
    """Overlapping or touching windows (start, end) joined, sorted."""
    out: list[list[float]] = []
    for a, b in sorted((float(a), float(b)) for a, b in windows if float(b) > float(a)):
        if out and a <= out[-1][1]:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return [(a, b) for a, b in out]


def weight(x, windows, ramp) -> np.ndarray:
    """How much of the slow-down applies at positions ``x`` (0 outside the windows, 1 inside, a half cosine over
    ``ramp`` at each end, inside the window). ``x``, ``windows`` and ``ramp`` in one unit (lines or seconds)."""
    x = np.asarray(x, dtype=float)
    w = np.zeros_like(x)
    for a, b in merge(windows):
        r = min(float(ramp), (b - a) / 2.0)
        inside = (x >= a) & (x <= b)
        if r <= 0:
            w = np.where(inside, 1.0, w)
            continue
        edge = np.minimum(x - a, b - x)
        part = np.where(edge >= r, 1.0, 0.5 * (1.0 - np.cos(np.pi * np.clip(edge, 0.0, r) / r)))
        w = np.where(inside, np.maximum(w, part), w)
    return w


def area(n_lines: int, windows, ramp) -> float:
    """The slowed time in lines: the integral of ``weight`` over the file's intervals (0 .. n_lines - 1)."""
    if n_lines < 2:
        return 0.0
    grid = np.linspace(0.0, n_lines - 1.0, (n_lines - 1) * SUB + 1)
    w = weight(grid, windows, ramp)
    return float(np.sum((w[1:] + w[:-1]) * 0.5 * np.diff(grid)))


def added_lines(n_lines: int, windows, ramp, factor: float) -> int:
    """The whole lines a slow-down by ``factor`` over ``windows`` adds."""
    return int(round((float(factor) - 1.0) * area(n_lines, windows, ramp)))


def positions(n_lines: int, windows, ramp, added: int) -> np.ndarray:
    """Each new line's position among the old ones (floats from 0 to n_lines - 1), n_lines + added of them: the old
    time stretched by 1 + k x weight, with k such that exactly ``added`` lines are gained."""
    n = int(n_lines)
    if n < 2 or added <= 0:
        return np.arange(n, dtype=float)
    grid = np.linspace(0.0, n - 1.0, (n - 1) * SUB + 1)
    w = weight(grid, windows, ramp)
    a = float(np.sum((w[1:] + w[:-1]) * 0.5 * np.diff(grid)))
    if a <= 0:
        raise ValueError("the windows lie outside the file: nothing to slow")
    s = 1.0 + (added / a) * w
    tau = np.concatenate([[0.0], np.cumsum((s[1:] + s[:-1]) * 0.5 * np.diff(grid))])
    tau *= (n - 1 + added) / tau[-1]
    pos = np.interp(np.arange(n + added, dtype=float), tau, grid)
    pos[0], pos[-1] = 0.0, n - 1.0
    return pos


def resample(md, pos) -> np.ndarray:
    """The file's lines (N, 7 integer millidegrees) at ``pos``: linear between neighbours, rounded as ALIX's emitters
    round (``np.round``), the first and last lines exactly."""
    md = np.asarray(md, dtype=np.int64)
    pos = np.asarray(pos, dtype=float)
    i0 = np.clip(np.floor(pos).astype(int), 0, len(md) - 1)
    i1 = np.minimum(len(md) - 1, i0 + 1)
    a = (pos - i0)[:, None]
    q = md.astype(float)
    out = np.round(q[i0] + a * (q[i1] - q[i0])).astype(np.int64)
    out[0], out[-1] = md[0], md[-1]
    return out


def slow(md, windows, ramp, factor: float, added: int | None = None) -> tuple[np.ndarray, int]:
    """``md`` slowed by ``factor`` over ``windows`` (line positions), ramped over ``ramp`` lines: (the new lines, the
    lines added). ``added`` forces the count (a partner arm gains the same time as the arm it runs with)."""
    md = np.asarray(md, dtype=np.int64)
    n = len(md)
    k = added_lines(n, windows, ramp, factor) if added is None else int(added)
    if k <= 0:
        return md.copy(), 0
    return resample(md, positions(n, windows, ramp, k)), k


def rates(md, line_s: float = timing.CYCLE_S) -> np.ndarray:
    """deg/s of each joint over each interval, the lines played every ``line_s``."""
    return np.diff(np.asarray(md, dtype=float), axis=0) / 1000.0 / float(line_s)


def path_gap_md(old, new) -> float:
    """The largest distance (millidegrees, the worst joint) of a new line from the nearest segment of the old file's
    polyline around its place in time: a check, made without the warp, that the joint path is the one the file had
    (rounding only, at most 0.5)."""
    old = np.asarray(old, dtype=float)
    new = np.asarray(new, dtype=float)
    n, m = len(old), len(new)
    if n < 2 or m < 1:
        return 0.0
    base = np.round(np.arange(m) * (n - 1) / max(1, m - 1)).astype(int)
    reach = abs(m - n) + 3
    best = np.full(m, np.inf)
    for k in range(-reach, reach + 1):
        i = np.clip(base + k, 0, n - 2)
        d = old[i + 1] - old[i]
        dd = np.einsum("ij,ij->i", d, d)
        t = np.where(dd > 0, np.einsum("ij,ij->i", new - old[i], d) / np.where(dd > 0, dd, 1.0), 0.0)
        t = np.clip(t, 0.0, 1.0)
        gap = np.abs(old[i] + t[:, None] * d - new).max(axis=1)
        best = np.minimum(best, gap)
    return float(best.max())
