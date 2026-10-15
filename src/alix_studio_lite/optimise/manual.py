"""MANUAL tuning, Tune's default: the trajectory HTML's algorithm, as it is
(``docs/reference/ui_v32/Alix_Studio_Lite_Trajectory_V2.html``, its ``processTrajectory``).

Ported line for line, with JavaScript's arithmetic (sums in order, ``Math.round``), so the same
recording and settings give the same bytes as the HTML (tests/test_manual.py holds the HTML's own
output). Its stages, each on or off, always in this order:

1. Trim: the absolute change between consecutive rows of each joint, as a centred rolling mean; a
   step is idle when all 7 joints are at or below the threshold; a run of at least the minimum
   idle run keeps its first and last row and loses the rows between.
2. Mean Smooth: a centred moving mean (101 points by default, the ends repeated), each joint
   recentred so its mean is kept, rounded; the first and last rows exactly as they were.
3. Scale: each region (original row numbers) resampled linearly, all 7 joints together, to its
   percentage of the region's intervals (100 % = unchanged, 150 % = slower, 50 % = faster).

The file's first and last rows are the recording's. Nothing is retimed to the limits: the time is
the time these stages give. The limits L1-L6 are checked on the result and reported, never
enforced (the owner, 15 Oct: "the lite manual optimizer never enforces any limits, it is up to the
hardware"). Time-optimal (TOPP-RA, ``engine``) is the other choice and never goes through this.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from ..motion import timing
from ..motion.checks import CheckReport

SAMPLE_MS = 5.0          # the HTML's sample interval: Lite's one clock, 5 ms per line (D26)
DEFAULTS = dict(idle_window=11, idle_threshold_md=8.0, idle_min_run=20, smooth_window=101)
STAGE_NAMES = (("trim", "Trim", "trim"), ("smooth", "Mean Smooth", "mean"), ("scale", "Scale", "scale"))


# ───────────────────────────────────────────────────────── JavaScript's arithmetic

def js_round(x) -> np.ndarray:
    """``Math.round``: the nearest integer, halves towards +infinity (exact: the fraction is
    compared, so 0.49999999999999994 stays 0 as it does in JavaScript)."""
    x = np.asarray(x, dtype=float)
    r = np.floor(x)
    return r + ((x - r) >= 0.5)


def _js_number(value, fallback: float) -> float:
    """``Number(value) || fallback``: empty, not a number or 0 gives the fallback."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return float(fallback)
    return v if np.isfinite(v) and v != 0 else float(fallback)


def validate_odd(value, fallback: int) -> int:
    """``validateOdd``: at least 3, odd (an even window is made one larger)."""
    n = max(3, int(js_round(_js_number(value, fallback))))
    return n + 1 if n % 2 == 0 else n


def params(settings: dict) -> dict:
    """``getParams`` from a region's settings, as the HTML reads its fields."""
    s = settings or {}
    return dict(sample_ms=SAMPLE_MS,
                idle_window=validate_odd(s.get("idle_window"), DEFAULTS["idle_window"]),
                idle_threshold_md=max(0.0, _js_number(s.get("idle_threshold_md"), DEFAULTS["idle_threshold_md"])),
                idle_min_run=max(1, int(js_round(_js_number(s.get("idle_min_run"), DEFAULTS["idle_min_run"])))),
                smooth_window=validate_odd(s.get("smooth_window"), DEFAULTS["smooth_window"]))


def stages(settings: dict) -> dict:
    s = settings or {}
    return dict(trim=bool(s.get("trim", True)), smooth=bool(s.get("smooth", True)), scale=bool(s.get("scale", True)))


def stage_label(st: dict) -> str:
    names = [label for key, label, _ in STAGE_NAMES if st.get(key)]
    return " + ".join(names) if names else "None"


def output_suffix(st: dict) -> str:
    """The HTML's file-name part: ``trim_mean_scale`` for every stage."""
    names = [short for key, _, short in STAGE_NAMES if st.get(key)]
    return "_".join(names) if names else "unchanged"


def processed_name(source_name: str, st: dict) -> str:
    """The HTML's processed file name: ``<recording>_<stages>_processed.txt``."""
    base = str(source_name or "trajectory")
    base = base[:base.rfind(".")] if "." in base else base
    return f"{base}_{output_suffix(st)}_processed.txt"


# ───────────────────────────────────────────────────────── the HTML's functions

def centered_rolling_mean(values, window: int) -> np.ndarray:
    """``centeredRollingMean``: ``window // 2`` values either side, the end values repeated past
    the ends, the sum divided by ``window``. Prefix sums in order, as the HTML adds them."""
    x = np.asarray(values, dtype=float)
    n = x.shape[0]
    half = int(window) // 2
    prefix = np.concatenate([[0.0], np.cumsum(x)])           # cumsum adds in order, as the loop does
    i = np.arange(n)
    raw_start, raw_end = i - half, i + half
    start, end = np.maximum(0, raw_start), np.minimum(n - 1, raw_end)
    s = prefix[end + 1] - prefix[start]
    s = s + np.where(raw_start < 0, (-raw_start) * x[0], 0.0)
    s = s + np.where(raw_end >= n, (raw_end - (n - 1)) * x[-1], 0.0)
    return s / float(window)


def trim_idle(data: np.ndarray, p: dict) -> dict:
    """``trimIdle``: the rows kept (with their 0-based original rows) and the idle regions."""
    data = np.asarray(data, dtype=np.int64)
    n = data.shape[0]
    if n < 2:
        return dict(data=data.copy(), kept=np.arange(n), regions=[])
    d = np.abs(np.diff(data.astype(float), axis=0))
    sm = np.column_stack([centered_rolling_mean(d[:, j], p["idle_window"]) for j in range(data.shape[1])])
    idle = (sm <= p["idle_threshold_md"]).all(axis=1)
    diff_len = n - 1
    remove = np.zeros(n, dtype=bool)
    regions = []
    i = 0
    while i < diff_len:
        if not idle[i]:
            i += 1
            continue
        a = i
        while i + 1 < diff_len and idle[i + 1]:
            i += 1
        b = i
        transitions = b - a + 1
        if transitions >= p["idle_min_run"]:
            remove[a + 1:b + 1] = True                       # rows a and b + 1 stay
            regions.append(dict(originalStartRow=a + 1, originalEndRow=b + 2, transitions=int(transitions),
                                removedPoints=int(max(0, transitions - 1))))
        i += 1
    keep = ~remove
    return dict(data=data[keep], kept=np.flatnonzero(keep), regions=regions)


def mean_smooth(data: np.ndarray, window: int) -> np.ndarray:
    """``meanSmooth``: the moving mean, recentred to each joint's mean (both sums in order),
    ``Math.round``, then the first and last rows exactly."""
    data = np.asarray(data, dtype=np.int64)
    n = data.shape[0]
    out = np.empty(data.shape, dtype=float)
    for j in range(data.shape[1]):
        y = data[:, j].astype(float)
        mean_orig = float(np.cumsum(y)[-1]) / n
        ys = centered_rolling_mean(y, window)
        mean_smooth_value = float(np.cumsum(ys)[-1]) / n
        out[:, j] = js_round(ys + (mean_orig - mean_smooth_value))
        out[0, j], out[-1, j] = data[0, j], data[-1, j]
    return out.astype(np.int64)


def map_regions(regions: list[dict], kept: np.ndarray) -> list[dict]:
    """``mapRegionsToTrimmed``: each region's rows among the kept ones; a region with none left is
    skipped (and said); two that overlap after trimming are refused."""
    mapped = []
    for r in sorted(regions, key=lambda r: r["start"]):
        s = int(np.searchsorted(kept, r["start"] - 1, side="left"))
        e = int(np.searchsorted(kept, r["end"] - 1, side="right")) - 1
        if s >= kept.shape[0] or e < s:
            mapped.append(dict(r, skipped=True, reason="Region contained no retained samples after idle trimming."))
            continue
        mapped.append(dict(r, trimmed_start=s, trimmed_end=e, skipped=False))
    active = [r for r in mapped if not r["skipped"]]
    for a, b in zip(active, active[1:]):
        if b["trimmed_start"] <= a["trimmed_end"]:
            raise ValueError("Two selected regions overlap after idle trimming. Adjust the original selections and try again.")
    return mapped


def interpolate_region(region: np.ndarray, scale_pct: float) -> tuple[np.ndarray, np.ndarray]:
    """``interpolateRegion``: linear resampling to round(intervals x pct / 100) intervals, the
    first and last row exactly. Also returns each new row's position in the region (0-based)."""
    region = np.asarray(region, dtype=np.int64)
    old = region.shape[0]
    if old <= 1:
        return region.copy(), np.arange(old, dtype=float)
    factor = float(scale_pct) / 100.0
    new_int = max(1, int(js_round((old - 1) * factor)))
    k = np.arange(new_int + 1, dtype=float)
    t = k / new_int
    pos = t * (old - 1)
    i0 = np.floor(pos).astype(int)
    i1 = np.minimum(old - 1, i0 + 1)
    a = (pos - i0)[:, None]
    r = region.astype(float)
    out = js_round(r[i0] + a * (r[i1] - r[i0])).astype(np.int64)
    out[0], out[-1] = region[0], region[-1]
    return out, pos


@dataclass
class Processed:
    md: np.ndarray                     # (N, 7) int64, the processed file
    source_rows: np.ndarray            # (N,) float: each line's row in the recording (1-based)
    report: dict                       # the HTML's processing report (its keys)


def process(md_raw: np.ndarray, settings: dict, source_name: str = "", regions: list[dict] | None = None) -> Processed:
    """``processTrajectory``: the checked stages on the recording ((N, 7) integers).
    ``regions``: [{start, end, pct}] on the recording's rows (by default the settings' segments)."""
    original = np.asarray(md_raw, dtype=np.int64)
    st, p = stages(settings), params(settings)
    if not (st["trim"] or st["smooth"] or st["scale"]):
        raise ValueError("Select at least one processing stage: Trim, Mean Smooth, or Scale.")
    if regions is None:
        regions = [dict(start=int(r["start"]), end=int(r["end"]), pct=float(r["pct"]))
                   for r in (settings or {}).get("segments") or []]
    n0 = original.shape[0]
    trim = trim_idle(original, p) if st["trim"] else dict(data=original.copy(), kept=np.arange(n0), regions=[])
    smooth = mean_smooth(trim["data"], p["smooth_window"]) if st["smooth"] else trim["data"].copy()
    kept = trim["kept"]
    src = (kept + 1).astype(float)                          # each smoothed line's recording row (1-based)
    mapped, region_report = [], []
    if st["scale"]:
        mapped = map_regions(regions, kept)
        parts, rows, cursor = [], [], 0
        for r in (m for m in mapped if not m["skipped"]):
            parts.append(smooth[cursor:r["trimmed_start"]])
            rows.append(src[cursor:r["trimmed_start"]])
            seg = smooth[r["trimmed_start"]:r["trimmed_end"] + 1]
            out, pos = interpolate_region(seg, r["pct"])
            parts.append(out)
            seg_rows = src[r["trimmed_start"]:r["trimmed_end"] + 1]
            rows.append(np.interp(pos, np.arange(seg_rows.shape[0], dtype=float), seg_rows))
            region_report.append(dict(originalStartRow=int(r["start"]), originalEndRow=int(r["end"]),
                                      trimmedStartRow=int(r["trimmed_start"]) + 1, trimmedEndRow=int(r["trimmed_end"]) + 1,
                                      scalePct=_num(r["pct"]), inputRows=int(seg.shape[0]), outputRows=int(out.shape[0])))
            cursor = r["trimmed_end"] + 1
        parts.append(smooth[cursor:])
        rows.append(src[cursor:])
        data = np.concatenate([q for q in parts if q.shape[0]], axis=0)
        src = np.concatenate([q for q in rows if q.shape[0]])
    else:
        data = smooth.copy()
    data[0], data[-1] = original[0], original[-1]            # the recording's exact first and last rows
    sec = timing.duration_for                                # Lite's one clock (D26): N lines last (N - 1) x 5 ms
    skipped = [m for m in mapped if m["skipped"]]
    report = dict(
        sourceFile=source_name, generatedAt=time.strftime("%Y-%m-%dT%H:%M:%S"),
        selectedStages=st, selectedStageLabel=stage_label(st),
        parameters=dict(sampleMs=p["sample_ms"], idleWindow=p["idle_window"], idleThreshold=p["idle_threshold_md"],
                        idleMinRun=p["idle_min_run"], smoothWindow=p["smooth_window"]),
        sourceValidRows=int(n0), skippedSourceLines=[],
        idleRegions=trim["regions"] if st["trim"] else [],
        idlePointsRemoved=int(n0 - trim["data"].shape[0]) if st["trim"] else 0,
        rowsAfterTrim=int(trim["data"].shape[0]), rowsAfterMeanSmooth=int(smooth.shape[0]),
        scalingRegions=region_report if st["scale"] else [],
        skippedRegions=[dict(originalStartRow=int(m["start"]), originalEndRow=int(m["end"]), reason=m["reason"])
                        for m in skipped],
        finalRows=int(data.shape[0]),
        originalTimeSeconds=sec(n0), afterTrimTimeSeconds=sec(trim["data"].shape[0]),
        afterMeanSmoothTimeSeconds=sec(smooth.shape[0]), finalTimeSeconds=sec(data.shape[0]),
        endpointPreserved=dict(first=bool((data[0] == original[0]).all()), last=bool((data[-1] == original[-1]).all())),
        outputName=processed_name(source_name, st),
    )
    return Processed(md=data.astype(np.int64), source_rows=src, report=report)


def _num(v):
    f = float(v)
    return int(f) if f == int(f) else f


def status_message(rep: dict, regions_set: int) -> str:
    """The HTML's message after Confirm & Generate, line for line."""
    st, prm = rep["selectedStages"], rep["parameters"]
    lines = ["Generated successfully.", f"Stages applied: {rep['selectedStageLabel']}",
             "Trim: " + (f"ON — {len(rep['idleRegions'])} idle region(s), {rep['idlePointsRemoved']} point(s) removed"
                         if st["trim"] else "OFF"),
             "Mean Smooth: " + (f"ON — {prm['smoothWindow']}-point centered moving mean" if st["smooth"] else "OFF"),
             "Scale: " + (f"ON — {len(rep['scalingRegions'])} region(s) applied" if st["scale"] else "OFF"),
             f"Final rows: {rep['finalRows']}",
             f"Final time @ {prm['sampleMs']:g} ms/row: {format_seconds(timing.duration_for(rep['finalRows']))}"]
    if rep["skippedRegions"]:
        lines.append(f"Warning: {len(rep['skippedRegions'])} selected region(s) had no retained points after trimming "
                     "and were skipped.")
    if st["scale"] and regions_set == 0:
        lines.append("Note: Scale was enabled but no regions were selected, so no scaling change was made.")
    return "\n".join(lines)


def format_seconds(seconds: float) -> str:
    """The HTML's ``formatSeconds``."""
    if not np.isfinite(seconds):
        return "-"
    mins = int(seconds // 60)
    if mins > 0:
        return f"{seconds:.3f} s ({mins} min {seconds - mins * 60:.1f} s)"
    return f"{seconds:.3f} s"


# ───────────────────────────────────────────────────────── the result Lite keeps

@dataclass
class ManualResult:
    """A manual tune as the rest of Lite reads an optimiser's result. ``uploadable`` is always
    True (nothing is enforced); ``limits_ok`` says whether every hard limit passes, and the
    report lists what breaks."""
    md: np.ndarray
    side: str
    recorded_s: float
    report: dict
    checks: CheckReport = field(default_factory=CheckReport)
    message: str = ""
    source_rows: np.ndarray | None = None
    optimiser: str = "manual"

    @property
    def lines(self) -> int:
        return int(self.md.shape[0])

    @property
    def achieved_s(self) -> float:
        return timing.duration_for(self.lines)

    @property
    def limits_ok(self) -> bool:
        return self.checks.passed

    @property
    def recipe(self) -> dict:
        return dict(optimiser="manual", side=self.side, cycle_s=timing.CYCLE_S, report=self.report,
                    enforced=False, plan=dict(kept_lines=self.report["rowsAfterTrim"],
                                              trimmed_lines=self.report["idlePointsRemoved"],
                                              smooth_window=self.report["parameters"]["smoothWindow"]
                                              if self.report["selectedStages"]["smooth"] else 0,
                                              segments=self.report["scalingRegions"]))

    def summary(self) -> dict:
        return {"optimiser": "manual", "side": self.side, "target_s": None, "achieved_s": round(self.achieved_s, 3),
                "lines": self.lines, "t_min_s": None, "lines_min": None, "recorded_s": round(self.recorded_s, 3),
                "met": True, "uploadable": True, "limits_ok": self.limits_ok, "enforced": False,
                "limits_text": violations_text(self.checks), "message": self.message, "hints": [], "binding": []}


def violations_text(checks: CheckReport) -> str:
    """The hard limits that break, worded as Tune shows them ("" when every one passes)."""
    from ..motion.checks import failures_text

    return failures_text(checks.to_dict())


def optimise(md_raw: np.ndarray, side: str, settings: dict, source_name: str = "", limits=None) -> ManualResult:
    """The HTML's processing of a recording, with every limit checked and reported, never enforced."""
    from ..motion.checks import check_bytes
    from ..motion.limits import load_limits

    raw = np.asarray(md_raw, dtype=np.int64)
    out = process(raw, settings, source_name)
    lim = limits if limits is not None else load_limits(side)
    if settings.get("max_dev_mm"):
        lim = lim.with_max_dev(settings["max_dev_mm"])
    checks = check_bytes(out.md, lim, raw_md=raw)
    res = ManualResult(md=out.md, side=side, recorded_s=timing.duration_for(raw.shape[0]), report=out.report,
                       checks=checks, source_rows=out.source_rows)
    msg = status_message(out.report, len((settings or {}).get("segments") or []))
    if not checks.passed:
        msg += f"\nLimits (shown, not enforced: the hardware decides): {violations_text(checks)}."
    res.message = msg
    return res


def progress_pairs(res: ManualResult, samples: int = 1200) -> list:
    """[line, recording row] pairs (both 1-based): where each line of the processed file comes
    from in the recording, exactly (the playhead on the recording's plot)."""
    n = res.lines
    lines = np.unique(np.linspace(0, n - 1, min(samples, n)).astype(int))
    rows = res.source_rows if res.source_rows is not None else np.arange(1, n + 1, dtype=float)
    return [[int(i) + 1, int(round(float(rows[i])))] for i in lines]


# ───────────────────────────────────────────────────────── the optimised time, after Trim

def trimmed_counts(md_raw: np.ndarray, settings: dict, regions: list[dict]) -> tuple[list[int], int]:
    """How many rows of each region (original rows) Trim keeps with these settings, and how many
    rows Trim keeps in all (every row when Trim is off)."""
    raw = np.asarray(md_raw, dtype=np.int64)
    st, p = stages(settings), params(settings)
    kept = trim_idle(raw, p)["kept"] if st["trim"] else np.arange(raw.shape[0])
    counts = []
    for r in regions:
        s = int(np.searchsorted(kept, int(r["start"]) - 1, side="left"))
        e = int(np.searchsorted(kept, int(r["end"]) - 1, side="right")) - 1
        counts.append(max(0, e - s + 1))
    return counts, int(kept.shape[0])


def final_lines(counts: list[int], kept_total: int, pcts: list[float], selected: list[bool]) -> int:
    """The processed file's line count when these regions are scaled (Mean Smooth keeps the count)."""
    total = kept_total
    for c, pct, use in zip(counts, pcts, selected):
        if use and c >= 2:
            total += max(1, int(js_round((c - 1) * (float(pct) / 100.0)))) + 1 - c
    return int(total)
