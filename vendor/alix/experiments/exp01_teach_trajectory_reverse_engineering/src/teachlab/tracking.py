"""Commanded-vs-executed comparison: what the controller did to the path.

exp01 asks what a RealMan controller puts *between* a joint-space point list
and the motion that comes out.  A teach file is timeless geometry (seven
integers per line, no clock); a UDP recording is the executed motion at 5 ms.
The two are not sample-aligned and never can be, because the controller
re-times the track (F98) — so nothing here compares sample i to sample i.

Everything is anchored on ARC LENGTH along the commanded path:

    project_onto_polyline  every executed sample -> (station s, deviation d)
                           by monotone nearest-point projection
    deviation_frame        d split into tangent / in-work-plane / out-of-plane
    menger_curvature       curvature at a fixed arc-length aperture, so it is
                           a property of the SHAPE and not of the sample rate
    station_time_map       s(t) inverted -> the executed time each commanded
                           step was given: the re-timing law, measured without
                           ever assuming the record period

`d` is the tracking error only in the sense of "distance from the commanded
polyline".  Where the commanded path is coarsely sampled through a turn, the
polyline cuts the corner itself, so a positive `d` there is the controller
being *rounder* than the chord, not worse.  `corner_bias` reports the sign.
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np


# ── projection ─────────────────────────────────────────────────────────────

def polyline_arclength(C: np.ndarray) -> np.ndarray:
    d = np.linalg.norm(np.diff(C, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)])


def align_monotone(E: np.ndarray, C: np.ndarray, *, window: int = 60,
                   back: int = 1) -> dict:
    """Greedy forward-only alignment of an executed stream onto an ordered
    commanded point list — in JOINT space, where it is well posed.

    A nearest-point projection is NOT usable on a scrub path: right_test
    revisits its own neighbourhood constantly (88 % of its TCP samples come
    within 20 mm of a part of the path 200+ samples away, median 5 mm), so an
    unconstrained search jumps between passes.  What is unambiguous is that a
    track replay reproduces the commanded points IN ORDER.  So the match index
    is forced non-decreasing and searched only `window` points ahead of the
    previous one (`back` allows a small retreat for dwell jitter).

    Returns frac (M,) — the fractional commanded index, refined onto the
    neighbouring segments — and resid (M,), the distance left over, which is
    the controller's own deviation and NOT an alignment error.
    """
    E = np.asarray(E, dtype=float); C = np.asarray(C, dtype=float)
    M, N = E.shape[0], C.shape[0]
    frac = np.empty(M); resid = np.empty(M)
    lo = 0
    for i in range(M):
        a = max(0, lo - back); b = min(N, a + window)
        d = np.linalg.norm(C[a:b] - E[i], axis=1)
        k = int(np.argmin(d)); j = a + k
        # refine onto the two adjacent segments
        best_t, best_d, best_j = 0.0, float(d[k]), j
        for j0 in (j - 1, j):
            if 0 <= j0 < N - 1:
                seg = C[j0 + 1] - C[j0]
                L2 = float(seg @ seg)
                if L2 <= 0:
                    continue
                t = float(np.clip((E[i] - C[j0]) @ seg / L2, 0.0, 1.0))
                dd = float(np.linalg.norm(E[i] - (C[j0] + t * seg)))
                if dd < best_d:
                    best_t, best_d, best_j = t, dd, j0
        frac[i] = best_j + best_t; resid[i] = best_d
        lo = j
    frac = np.maximum.accumulate(frac)
    return {"frac": frac, "resid": resid}


def align_dtw(E: np.ndarray, C: np.ndarray, *, max_advance: int = 8,
              start_slack: int = 20) -> dict:
    """Optimal monotone alignment of an executed stream onto an ordered
    commanded list, by dynamic programming.

    Why not a greedy forward search: a scrub REVERSES over its own segments,
    so the locally nearest commanded point stops advancing at every turn and a
    greedy match pins there for the rest of the run.  DTW is global — it pays
    a little extra distance through the turn to buy a much better fit after
    it.

    Each executed sample advances the commanded index by 0..`max_advance`
    (0 = the arm is dwelling or the stream is faster than the track; >1 = the
    controller is running the track faster than one commanded point per
    sample).  Because every path charges exactly one cost per executed
    sample, the accumulated costs of different endpoints are comparable — so
    the end is OPEN, which is what makes an ABORTED run alignable: it simply
    ends at the commanded index it reached.

    Returns frac (M,) fractional commanded index (integer path refined onto
    the adjacent segments), resid (M,) leftover distance, and last_idx.
    """
    E = np.asarray(E, dtype=float); C = np.asarray(C, dtype=float)
    M, N = E.shape[0], C.shape[0]
    K = int(max_advance)
    INF = np.inf
    back = np.empty((M, N), dtype=np.uint8)
    prev = np.full(N, INF)
    d0 = np.linalg.norm(C - E[0], axis=1)
    lim = min(N, max(1, start_slack + 1))
    prev[:lim] = d0[:lim]
    back[0, :] = 0
    for i in range(1, M):
        cost = np.linalg.norm(C - E[i], axis=1)
        best = prev.copy()
        arg = np.zeros(N, dtype=np.uint8)
        for k in range(1, K + 1):
            shifted = np.full(N, INF)
            shifted[k:] = prev[:-k]
            better = shifted < best
            best = np.where(better, shifted, best)
            arg = np.where(better, np.uint8(k), arg)
        prev = best + cost
        back[i, :] = arg
    j = int(np.argmin(prev))
    path = np.empty(M, dtype=np.int64)
    for i in range(M - 1, -1, -1):
        path[i] = j
        j -= int(back[i, j])
        if j < 0:
            j = 0
    # refine each integer index onto the adjacent commanded segments
    frac = path.astype(float); resid = np.empty(M)
    for i in range(M):
        j0 = int(path[i]); best_d = float(np.linalg.norm(E[i] - C[j0])); best = (j0, 0.0)
        for a in (j0 - 1, j0):
            if 0 <= a < N - 1:
                seg = C[a + 1] - C[a]; L2 = float(seg @ seg)
                if L2 <= 0:
                    continue
                t = float(np.clip((E[i] - C[a]) @ seg / L2, 0.0, 1.0))
                dd = float(np.linalg.norm(E[i] - (C[a] + t * seg)))
                if dd < best_d:
                    best_d, best = dd, (a, t)
        frac[i] = best[0] + best[1]; resid[i] = best_d
    frac = np.maximum.accumulate(frac)
    return {"frac": frac, "resid": resid, "last_idx": int(path[-1]),
            "max_advance": K, "cost": float(np.min(prev)) / max(M, 1)}


def project_onto_polyline(P: np.ndarray, C: np.ndarray, *, window: int = 400,
                          back: int = 25, anchor: Optional[np.ndarray] = None) -> dict:
    """Nearest point on the polyline C for every point of P.

    With `anchor` (a per-sample fractional index, normally from
    `align_monotone` in joint space) the search is a small neighbourhood of
    that index — the only trustworthy mode on a self-revisiting path.  Without
    it the search is forward-monotone with the same caveat as
    `align_monotone`, and is fine only on paths that do not revisit.

    Returns station (M,), dist (M,), seg (M,), t (M,), foot (M,3).
    """
    P = np.asarray(P, dtype=float); C = np.asarray(C, dtype=float)
    if C.shape[0] < 2:
        raise ValueError("commanded polyline needs >= 2 points")
    A = C[:-1]; D = np.diff(C, axis=0)
    L2 = np.einsum("ij,ij->i", D, D)
    L2 = np.where(L2 > 0, L2, 1e-18)
    s = polyline_arclength(C)
    seglen = np.sqrt(L2)

    M = P.shape[0]; nseg = A.shape[0]
    out_seg = np.empty(M, dtype=int); out_t = np.empty(M); out_d = np.empty(M)
    anchor = None if anchor is None else np.asarray(anchor, dtype=float)
    lo = 0
    for i in range(M):
        if anchor is not None:
            c = int(anchor[i])
            a = max(0, c - window // 2); b = min(nseg, c + window // 2 + 1)
        else:
            a = max(0, lo - back); b = min(nseg, a + window)
        if b <= a:
            a, b = max(0, nseg - 1), nseg
        w = P[i] - A[a:b]
        t = np.clip(np.einsum("ij,ij->i", w, D[a:b]) / L2[a:b], 0.0, 1.0)
        foot = A[a:b] + t[:, None] * D[a:b]
        dist = np.linalg.norm(P[i] - foot, axis=1)
        k = int(np.argmin(dist))
        j = a + k
        out_seg[i] = j; out_t[i] = t[k]; out_d[i] = dist[k]
        lo = j
    station = s[out_seg] + out_t * seglen[out_seg]
    foot = C[out_seg] + out_t[:, None] * np.diff(C, axis=0)[out_seg]
    return {"station_m": station, "dist_m": out_d, "seg": out_seg, "t": out_t,
            "foot_m": foot, "cmd_s_m": s}


def deviation_frame(P: np.ndarray, proj: dict, C: np.ndarray,
                    plane_normal: Optional[np.ndarray] = None) -> dict:
    """Split the deviation vector (executed − foot) into components.

    tangent   along the commanded segment (a projection residual, ~0)
    normal_n  along `plane_normal` when given — for a surface task this is
              LIFT-OFF / PRESS DEPTH, the component that matters physically
    in_plane  the remainder: sideways within the work plane
    """
    C = np.asarray(C, dtype=float)
    D = np.diff(C, axis=0)
    tang = D[proj["seg"]]
    tang = tang / np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-12)
    e = np.asarray(P, dtype=float) - proj["foot_m"]
    e_t = np.einsum("ij,ij->i", e, tang)
    out = {"tangent_m": e_t}
    if plane_normal is not None:
        n = np.asarray(plane_normal, dtype=float)
        n = n / np.linalg.norm(n)
        e_n = e @ n
        e_rest = e - e_t[:, None] * tang - e_n[:, None] * n
        out["normal_m"] = e_n
        out["in_plane_m"] = np.linalg.norm(e_rest, axis=1) * np.sign(
            np.einsum("ij,ij->i", e_rest, np.cross(n, tang)))
    else:
        e_rest = e - e_t[:, None] * tang
        out["in_plane_m"] = np.linalg.norm(e_rest, axis=1)
    return out


# ── shape ──────────────────────────────────────────────────────────────────

def menger_curvature(C: np.ndarray, aperture_m: float = 0.005) -> np.ndarray:
    """Curvature at a FIXED arc-length aperture (Menger, circle through three
    points ±aperture away along the path).  Time-free and rate-free, so a
    5 ms recording and a 10 ms teach file are directly comparable — unlike
    |v×a|/|v|³, which needs a clock and blows up wherever the arm is slow."""
    C = np.asarray(C, dtype=float)
    s = polyline_arclength(C)
    n = C.shape[0]
    lo = np.searchsorted(s, s - aperture_m, side="left")
    hi = np.searchsorted(s, s + aperture_m, side="right") - 1
    lo = np.clip(lo, 0, n - 1); hi = np.clip(hi, 0, n - 1)
    ok = (hi - lo) >= 2
    a = C[lo]; b = C; c = C[hi]
    ab = np.linalg.norm(b - a, axis=1); bc = np.linalg.norm(c - b, axis=1)
    ca = np.linalg.norm(a - c, axis=1)
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    denom = ab * bc * ca
    # A dwell packs many samples into microns of path: the three points are
    # then collinear-to-noise and 4A/(abc) explodes. Demand a real aperture.
    wide = (ab >= 0.3 * aperture_m) & (bc >= 0.3 * aperture_m) & (ca >= 0.3 * aperture_m)
    kappa = np.where(ok & wide & (denom > 1e-15), 4.0 * area / np.maximum(denom, 1e-15), np.nan)
    return kappa


def resample_by_arclength(C: np.ndarray, ds_m: float) -> Tuple[np.ndarray, np.ndarray]:
    """Uniform-arc-length resampling — the fair way to compare two paths that
    were sampled by different clocks.  Returns (stations, points)."""
    C = np.asarray(C, dtype=float)
    s = polyline_arclength(C)
    if s[-1] <= 0:
        return s[:1], C[:1]
    grid = np.arange(0.0, s[-1], ds_m)
    pts = np.stack([np.interp(grid, s, C[:, k]) for k in range(C.shape[1])], axis=1)
    return grid, pts


def rotation_angle_between(Ra: np.ndarray, Rb: np.ndarray) -> np.ndarray:
    """Geodesic angle (deg) between stacks of rotation matrices."""
    rel = np.einsum("nji,njk->nik", Ra, Rb)
    tr = np.trace(rel, axis1=1, axis2=2)
    return np.degrees(np.arccos(np.clip((tr - 1.0) / 2.0, -1.0, 1.0)))


# ── timing ─────────────────────────────────────────────────────────────────

def index_time_map(t: np.ndarray, frac: np.ndarray) -> dict:
    """Invert the alignment: the executed instant each commanded INDEX was
    passed.  Index, not arc length — arc length is degenerate where the path
    dwells, and the index is what the controller actually steps through."""
    t = np.asarray(t, dtype=float)
    f = np.maximum.accumulate(np.asarray(frac, dtype=float))
    lo = int(np.ceil(f[0])); hi = int(np.floor(f[-1]))
    if hi - lo < 2:
        return {"idx": np.array([], dtype=int), "t_at": np.array([])}
    idx = np.arange(lo, hi + 1)
    return {"idx": idx, "t_at": np.interp(idx, f, t), "first_idx": lo, "last_idx": hi}


def retiming_law(travel_per_step: np.ndarray, idx: np.ndarray, t_at: np.ndarray, *,
                 nominal_period_s: float, block: int = 25, bins=None,
                 unit: str = "deg", min_demand: float = 10.0) -> dict:
    """How fast the controller was WILLING to run each part of the track.

    Measured over blocks of `block` commanded steps, because at 5 ms the
    executed clock resolves a single commanded step only to 0 or 1 sample —
    per-step timing is pure quantisation.  Per block:

        travel        how far the track asks the arm to move (deg or m)
        demand_rate   travel / (block x nominal_period)   — the rate the file
                      asks for IF the record period were `nominal_period_s`
        achieved_rate travel / (executed time for the block)
        stretch       executed time / nominal time

    `demand_rate` inherits the record-period assumption; `achieved_rate` and
    `ms_per_point` do NOT — they are measured.  A plateau in achieved_rate
    against demand_rate would be a velocity ceiling; a flat `ms_per_point`
    instead means the controller is a fixed-clock point player and the point
    SPACING alone sets the speed.

    Blocks below `min_demand` are dropped and counted: where the track barely
    moves, no alignment can say which commanded index the arm is on, so their
    timing is undefined rather than fast or slow.  Including them puts a
    spurious 0.6 ms and a spurious 15 ms in the same bin.
    """
    travel = np.asarray(travel_per_step, dtype=float)
    idx = np.asarray(idx, dtype=int); t_at = np.asarray(t_at, dtype=float)
    cum = np.concatenate([[0.0], np.cumsum(travel)])          # cum[i] = travel to index i
    rows = []
    for a in range(0, idx.size - block, block):
        b = a + block
        i0, i1 = int(idx[a]), int(idx[b])
        dt = float(t_at[b] - t_at[a])
        if dt <= 0 or i1 >= cum.size:
            continue
        tr = float(cum[i1] - cum[i0])
        nom = (i1 - i0) * nominal_period_s
        rows.append((tr / nom if nom > 0 else np.nan, tr / dt, dt / nom if nom > 0 else np.nan,
                     i0, 1e3 * dt / max(i1 - i0, 1)))
    if not rows:
        return {"blocks": [], "bins": []}
    demand = np.array([r[0] for r in rows]); achieved = np.array([r[1] for r in rows])
    stretch = np.array([r[2] for r in rows]); at = np.array([r[3] for r in rows])
    mspp = np.array([r[4] for r in rows])
    keep = demand >= min_demand
    n_dropped = int((~keep).sum())
    demand, achieved, stretch, at, mspp = (demand[keep], achieved[keep], stretch[keep],
                                           at[keep], mspp[keep])
    if demand.size == 0:
        return {"blocks": [], "bins": [], "n_below_min_demand": n_dropped}
    if bins is None:
        bins = ([0, 20, 40, 60, 90, 140, 200, 300, np.inf] if unit == "deg"
                else [0, 0.05, 0.10, 0.20, 0.35, 0.60, 1.0, np.inf])
    bins = np.asarray(bins, dtype=float)
    which = np.digitize(demand, bins) - 1
    out = []
    for k in range(len(bins) - 1):
        m = which == k
        if m.sum() < 3:
            continue
        out.append({"demand_lo": float(bins[k]), "demand_hi": float(bins[k + 1]),
                    "n_blocks": int(m.sum()),
                    "demand_median": float(np.median(demand[m])),
                    "achieved_median": float(np.median(achieved[m])),
                    "achieved_p90": float(np.percentile(achieved[m], 90)),
                    "stretch_median": float(np.median(stretch[m])),
                    "stretch_p10": float(np.percentile(stretch[m], 10)),
                    "stretch_p90": float(np.percentile(stretch[m], 90)),
                    "ms_per_point_median": float(np.median(mspp[m])),
                    "ms_per_point_p10": float(np.percentile(mspp[m], 10)),
                    "ms_per_point_p90": float(np.percentile(mspp[m], 90))})
    return {"unit": unit, "block": int(block), "nominal_period_s": float(nominal_period_s),
            "n_blocks": int(demand.size), "n_below_min_demand": n_dropped,
            "min_demand": float(min_demand),
            "ms_per_point_median": float(np.median(mspp)),
            "ms_per_point_iqr": [float(np.percentile(mspp, 25)), float(np.percentile(mspp, 75))],
            "stretch_overall_median": float(np.median(stretch)),
            "achieved_p95": float(np.percentile(achieved, 95)),
            "achieved_max": float(achieved.max()),
            "bins": out,
            "_arrays": {"demand": demand, "achieved": achieved, "stretch": stretch,
                        "at_index": at, "ms_per_point": mspp}}
