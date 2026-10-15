"""Automatic speed-optimisation analysis: the Trajectory V2 processor's (analyzeMotionRegions in
docs/reference/ui_v32/Alix_Studio_Lite_Trajectory_V2.html), as Tune's Analyze step shows it.

The recording is cut into 5 to 10 equal review regions. Each is measured by the joint-space step
between lines (its mean and its peak) and by how much that step changes (direction changes), and
classed against the other regions' quartiles:

- the first and the last region: approach and exit, 75 % of their time;
- much change: a direction change or a dynamic transition, slower, 115 %;
- a high peak with some change: kept or slightly slower, 105 %;
- little movement: dwell or redundant motion, shorter, 80 %;
- smooth continuous motion: a possible straight cleaning pass, moderately shorter, 85 %;
- anything else: slightly shorter, 95 %.

Motion only: it cannot see contact or collisions, so every suggestion is the operator's to keep or
change. The percentages are manual tuning's Scale (time as a % of the region's), as in the HTML.
"""
from __future__ import annotations

import numpy as np

CLASSES = {   # motion type, recommendation, peak guidance, scale %, reason (the processor's words)
    "approach": ("Approach candidate", "Shorter / faster", "Can increase carefully", 75,
                 "Beginning transition candidate; typically less accuracy-sensitive than the cleaning path."),
    "exit": ("Exit candidate", "Shorter / faster", "Can increase carefully", 75,
             "End transition candidate; move away from the work area efficiently."),
    "dynamic": ("Direction-change / high-dynamic region", "Longer / slower", "Limit peak speed", 115,
                "High acceleration/deceleration activity suggests a direction change or dynamic transition."),
    "peak": ("High-peak motion", "Keep or slightly slower", "Limit peak speed", 105,
             "Peak joint movement is high; avoid further compression until hardware/simulation validation."),
    "slow": ("Slow / possibly redundant", "Shorter / review for trim", "Can increase after review", 80,
             "Low average movement may contain dwell or redundant motion; review together with Trim results."),
    "smooth": ("Smooth / possible straight cleaning pass", "Moderately shorter", "Can increase moderately", 85,
               "Smooth continuous motion with relatively low direction-change activity is a candidate for moderate "
               "speed-up."),
    "general": ("General continuous motion", "Slightly shorter", "Keep moderate", 95,
                "Balanced motion region; use a conservative reduction and review in simulation."),
}
METHOD = "Motion-only heuristic using joint step magnitude and acceleration/change activity"


def _q(values, p: float) -> float:
    """The processor's percentile: linear between the two nearest sorted values."""
    return float(np.percentile(np.asarray(values, dtype=float), p * 100.0)) if len(values) else 0.0


def region_count(n: int) -> int:
    """5 to 10 review regions, one per ~3,000 lines (the processor's Math.round)."""
    return min(10, max(5, int(np.floor(n / 3000 + 0.5))))


def analyse(md, sample_ms: float = 5.0) -> dict:
    """The review regions of a recording ((N, 7) joint values; any unit, the rules are relative)
    with a suggested time % each, and the totals the processor shows."""
    data = np.asarray(md, dtype=float)
    n = int(data.shape[0])
    if n < 2:
        raise ValueError("a recording needs at least two lines")
    dt = float(sample_ms) / 1000.0
    step = np.zeros(n)
    step[1:] = np.linalg.norm(np.diff(data, axis=0), axis=1)
    accel = np.zeros(n)
    accel[2:] = np.abs(step[2:] - step[1:-1])
    k = region_count(n)
    raw = []
    for r in range(k):
        s0 = (r * n) // k
        e0 = max(s0, ((r + 1) * n) // k - 1)
        seg = slice(s0, e0 + 1)
        c = max(1, e0 - s0 + 1)
        raw.append(dict(start=s0 + 1, end=e0 + 1, avg_step=float(step[seg].sum() / c), max_step=float(step[seg].max()),
                        avg_accel=float(accel[seg].sum() / c), original_s=c * dt))
    avg_steps = [r["avg_step"] for r in raw]
    q25s, q50s = _q(avg_steps, 0.25), _q(avg_steps, 0.50)
    q75m = _q([r["max_step"] for r in raw], 0.75)
    accels = [r["avg_accel"] for r in raw]
    q25a, q50a, q75a = _q(accels, 0.25), _q(accels, 0.50), _q(accels, 0.75)
    out = []
    for i, r in enumerate(raw):
        if i == 0:
            kind = "approach"
        elif i == len(raw) - 1:
            kind = "exit"
        elif r["avg_accel"] >= q75a and r["avg_accel"] > 0:
            kind = "dynamic"
        elif r["max_step"] >= q75m and r["avg_accel"] >= q50a:
            kind = "peak"
        elif r["avg_step"] <= q25s:
            kind = "slow"
        elif r["avg_accel"] <= q25a and r["avg_step"] >= q50s:
            kind = "smooth"
        else:
            kind = "general"
        motion, action, peak, pct, reason = CLASSES[kind]
        out.append(dict(r, kind=kind, motion_type=motion, action=action, peak_guidance=peak, scale_pct=pct,
                        selected=pct != 100, estimated_s=r["original_s"] * pct / 100.0, reason=reason))
    original = sum(r["original_s"] for r in out)
    estimated = sum(r["estimated_s"] if r["selected"] else r["original_s"] for r in out)
    return dict(method=METHOD, contact_data=False, sample_ms=float(sample_ms), lines=n, regions=out,
                totals=dict(regions=len(out), faster=sum(1 for r in out if r["selected"] and r["scale_pct"] < 100),
                            slower=sum(1 for r in out if r["selected"] and r["scale_pct"] > 100),
                            original_s=original, estimated_s=estimated, change_s=original - estimated))


def _totals(regions: list[dict]) -> dict:
    original = sum(r["original_s"] for r in regions)
    estimated = sum(r["estimated_s"] if r["selected"] else r["original_s"] for r in regions)
    return dict(regions=len(regions), faster=sum(1 for r in regions if r["selected"] and r["scale_pct"] < 100),
                slower=sum(1 for r in regions if r["selected"] and r["scale_pct"] > 100),
                original_s=original, estimated_s=estimated, change_s=original - estimated)


def fit_target(result: dict, target_s: float, counts: list[int] | None = None, kept_total: int | None = None,
               lo: float = 1.0, hi: float = 300.0) -> dict:
    """The suggestions fitted to a total time (the owner, 15 Oct: "if the user types 20 seconds, it
    should give suggestions for the scale regions"). One factor multiplies every region's suggested %,
    so each keeps the analysis' guidance relative to the others (a region suggested slower stays
    slower than one suggested faster), each a whole percent within the HTML's scale range (1-300 %).

    With ``counts`` (the rows of each region that Trim keeps, ``manual.trimmed_counts``) and
    ``kept_total``, the factor is found so the GENERATED file comes to ``target_s``: Trim first, then
    each region resampled to its %, exactly as manual tuning makes it (the owner, 15 Oct: a 15 s target
    came out 36 s; Trim's idle rows were not counted, and the limits then stretched the file). Without
    them, the HTML's motion-only estimate. ``reached`` is False when the scale range cannot get there."""
    from .manual import final_lines
    from ..motion.timing import duration_for

    target = float(target_s)
    if not target > 0:
        raise ValueError("the target time must be above 0 s")
    base = [float(r["scale_pct"]) for r in result["regions"]]
    orig = [float(r["original_s"]) for r in result["regions"]]
    after_trim = counts is not None and kept_total is not None

    def pcts(k: float) -> list[int]:
        return [int(min(hi, max(lo, np.floor(p * k + 0.5)))) for p in base]

    def est(k: float) -> float:
        ps = pcts(k)
        if after_trim:
            return duration_for(final_lines(counts, kept_total, ps, [x != 100 for x in ps]))
        return sum(o * x / 100.0 for o, x in zip(orig, ps))

    k_lo, k_hi = lo / max(base) * 0.5, hi / min(base) * 1.5
    if est(k_lo) >= target:
        k = k_lo
    elif est(k_hi) <= target:
        k = k_hi
    else:
        for _ in range(80):                                      # the smallest factor that reaches the target
            mid = (k_lo + k_hi) / 2.0
            if est(mid) < target:
                k_lo = mid
            else:
                k_hi = mid
        k = k_hi if abs(est(k_hi) - target) <= abs(est(k_lo) - target) else k_lo
    regions = []
    for r, pct in zip(result["regions"], pcts(k)):
        action = "Shorter / faster" if pct < 100 else "Longer / slower" if pct > 100 else "Unchanged"
        regions.append(dict(r, scale_pct=pct, selected=pct != 100, action=action, estimated_s=r["original_s"] * pct / 100.0))
    out = dict(result, regions=regions, totals=_totals(regions))
    got = est(k)
    reached = abs(got - target) <= max(0.05, 0.01 * target)
    out["target"] = dict(target_s=target, factor=round(k, 4), estimated_s=round(got, 3), reached=reached, after_trim=after_trim,
                         shortest_s=round(est(lo / max(base) * 0.5), 3), longest_s=round(est(hi / min(base) * 1.5), 3))
    return out
