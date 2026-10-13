"""Run an optimisation: one region on one arm (``optimise``), or a TOGETHER pair that must
start and finish together (``optimise_together``)."""
from __future__ import annotations

import json
import time
from dataclasses import asdict

import numpy as np

from .. import __version__, vendor_paths
from ..motion import timing, track
from ..motion.checks import check_bytes, motion_ok
from ..motion.limits import LimitSet, load_limits
from . import binding as bnd
from . import default_opt, toppra_opt
from .target import Search, find_min_lines, settle
from .types import OPTIMISERS, OptRequest, OptResult


BRIDGE_DEG_S = 20.0          # a cut's gap is crossed at this joint speed (largest joint), then smoothed
BRIDGE_MIN_LINES = 20        # and never in less than 0.1 s


def apply_cuts(req: OptRequest) -> tuple[OptRequest, dict | None]:
    """Remove the cut stretches from the recording before anything else.

    Where a cut leaves a gap, the two sides are joined by a straight joint-space move at
    ``BRIDGE_DEG_S`` (smoothing then rounds its corners), so the slowdown stays local instead of
    the whole file being stretched to survive a jump. The segments' line numbers move onto the
    new recording, which is then the reference for the path (L5) and end (L6) checks. Whether a
    bridge clears the toilet is the 3D check's job.
    """
    from dataclasses import replace

    if not req.cuts:
        return req, None
    md = np.asarray(req.md_raw, dtype=np.int64)
    n = int(md.shape[0])
    keep = np.ones(n, dtype=bool)
    cuts = []
    for a, b in sorted(req.cuts):
        a, b = max(1, int(a)), min(n, int(b))
        if a <= b:
            keep[a - 1:b] = False
            cuts.append([a, b])
    if int(keep.sum()) < 2:
        raise ValueError("the cuts remove the whole recording")
    edges = np.flatnonzero(np.diff(np.concatenate([[0], keep.astype(np.int8), [0]])))
    runs = list(zip(edges[0::2], edges[1::2] - 1))                    # kept stretches, inclusive
    new_row = np.full(n, -1, dtype=np.int64)
    pieces, bridges, pos = [], [], 0
    for i, (a, b) in enumerate(runs):
        if i:
            q0, q1 = md[runs[i - 1][1]].astype(float), md[a].astype(float)
            jump = float(np.abs(q1 - q0).max()) / 1000.0
            k = max(BRIDGE_MIN_LINES, int(np.ceil(jump / BRIDGE_DEG_S / timing.CYCLE_S)))
            t = np.arange(1, k)[:, None] / k
            pieces.append(np.rint(q0 + (q1 - q0) * t).astype(np.int64))
            bridges.append(dict(after_line=int(runs[i - 1][1]) + 1, before_line=int(a) + 1,
                                jump_deg=round(jump, 2), seconds=round(k * timing.CYCLE_S, 3)))
            pos += k - 1
        pieces.append(md[a:b + 1])
        new_row[a:b + 1] = np.arange(pos, pos + b - a + 1)
        pos += b - a + 1
    kept = np.flatnonzero(keep)
    segs = []
    for sg in req.segments:
        lo = kept[kept >= sg.start - 1]
        hi = kept[kept <= sg.end - 1]
        if lo.size and hi.size and new_row[hi[-1]] >= new_row[lo[0]]:
            segs.append(replace(sg, start=int(new_row[lo[0]]) + 1, end=int(new_row[hi[-1]]) + 1))
    info = dict(cuts=cuts, removed_lines=int(n - keep.sum()), recorded_lines=n, bridges=bridges)
    return replace(req, md_raw=np.vstack(pieces), segments=tuple(segs), cuts=()), info


def make_plan(req: OptRequest, limits: LimitSet):
    if req.optimiser == "default":
        return default_opt.DefaultPlan(req.md_raw, limits, req.default, req.segments)
    if req.optimiser == "toppra":
        return toppra_opt.ToppraPlan(req.md_raw, limits, req.toppra, req.segments)
    raise ValueError(f"optimiser must be one of {OPTIMISERS}, got {req.optimiser!r}")


def _versions() -> dict:
    import scipy

    out = {"alix_studio_lite": __version__, "numpy": np.__version__, "scipy": scipy.__version__,
           "toppra": toppra_opt.toppra_version()}
    try:
        man = json.loads((vendor_paths.VENDOR / "MANIFEST.json").read_text(encoding="utf-8"))
        out["vendored_alix_commit"] = (man.get("alix_git") or {}).get("commit")
    except Exception:                                                      # noqa: BLE001
        out["vendored_alix_commit"] = None
    return out


class _Solved:
    """A plan with its fastest feasible line count."""

    def __init__(self, req: OptRequest, limits: LimitSet):
        t0 = time.perf_counter()
        self.req = req
        self.limits = limits
        self.plan = make_plan(req, limits)
        t1 = time.perf_counter()
        self.search = Search(self.plan.make, lambda md: motion_ok(md, limits))
        self.n_min = find_min_lines(self.search, self.plan.n_start, self.plan.n_floor, self.plan.n_cap)
        if self.n_min is not None:
            self.n_min = settle(self.search, self.n_min)
        self.timings = {"prepare_s": round(t1 - t0, 3), "search_s": round(time.perf_counter() - t1, 3)}

    def target_n(self) -> int | None:
        if self.req.target_s is None:
            return self.plan.n_start if self.req.natural else None
        return timing.lines_for(self.req.target_s) - 1


def _finish(s: _Solved, n: int, n_target: int | None, *, n_min_shared: int | None = None,
            binding_note: str | None = None, own_binding: bool = True, cut: dict | None = None) -> OptResult:
    """Build the result at ``n`` intervals. For a TOGETHER pair ``n_min_shared`` is the pair's
    minimum, and only the arm that sets it reports the binding limit."""
    req, limits = s.req, s.limits
    t0 = time.perf_counter()
    md = s.plan.make(n)
    checks = check_bytes(md, limits, raw_md=req.md_raw)
    n_min = s.n_min if n_min_shared is None else n_min_shared
    lines, lines_min = n + 1, n_min + 1
    met = n_target is None or n_target >= n_min
    exact = n_target is None or n == n_target or (n_target < n_min and n == n_min)
    bindings = []
    if own_binding and (n_target is None or n_target < n_min):
        bindings = bnd.binding_report(s.plan.util(s.n_min))
    hints = []
    l5 = checks.get("L5")
    if l5 is not None and l5.passed is False:
        hint = (default_opt.smoothing_hint(req.md_raw, limits, req.default) if req.optimiser == "default"
                else toppra_opt.smoothing_hint(req.md_raw, limits, req.toppra))
        if hint:
            hints.append(hint)
    for c in checks.failures():
        if c.speed_fixes is False and c.id == "L1":
            hints.append("A joint goes too close to its end stop: re-teach that part of the region.")
    achieved = timing.duration_for(lines)
    t_min = timing.duration_for(lines_min)
    recorded = timing.duration_for(cut["recorded_lines"] if cut else len(req.md_raw))
    unit = "s"
    why = bnd.describe(bindings)
    if not checks.passed:
        first = checks.failures()[0]
        message = f"Not ready to upload: {first.name.lower()} - {first.detail}"
    elif req.target_s is None and req.natural:
        message = (f"Recorded pace kept: {achieved:.2f} {unit}." if exact and met else
                   f"The recorded pace is faster than the limits allow here. The fastest safe time is "
                   f"{t_min:.2f} {unit}" + (f", {why}." if why else "."))
    elif req.target_s is None:
        message = f"Fastest safe time: {achieved:.2f} {unit}" + (f", {why}." if why else ".")
    elif not met:
        message = (f"The robot cannot do this region in {req.target_s:.2f} {unit} within its limits. "
                   f"The fastest safe time is {t_min:.2f} {unit}" + (f", {why}." if why else "."))
    elif not exact:
        message = (f"Target {req.target_s:.2f} {unit} adjusted to {achieved:.2f} {unit} to stay within "
                   "the limits after rounding.")
    else:
        message = f"Target met: {achieved:.2f} {unit}."
    if binding_note:
        message = f"{message} {binding_note}"
    recipe = dict(
        optimiser=req.optimiser, side=req.side, frame=limits.frame, cycle_s=timing.CYCLE_S,
        target_s=req.target_s,
        options=asdict(req.default if req.optimiser == "default" else req.toppra),
        segments=[asdict(sg) for sg in req.segments],
        max_dev_mm=req.max_dev_mm,
        limits=dict(rating_deg_s=list(limits.rating_deg_s), rate_bar=limits.rate_bar,
                    rate_design=limits.rate_design, acc_deg_s2=limits.acc_deg_s2, acc_tol=limits.acc_tol,
                    acc_window_s=limits.acc_window_s, step_deg=limits.step_deg,
                    max_dev_mm=limits.max_dev_mm, ends_tol_deg=limits.ends_tol_deg,
                    dt_gate_s=limits.dt_gate, execution_model=limits.model.get("fitted")),
        plan=s.plan.recipe(), lines=lines, lines_min=lines_min,
        recorded_lines=int(cut["recorded_lines"] if cut else len(req.md_raw)), natural=req.natural, cut=cut,
        speedup_vs_recording=round((cut["recorded_lines"] if cut else len(req.md_raw)) / lines, 3),
        search_evaluations=s.search.evaluations,
        timings=dict(s.timings, finish_s=round(time.perf_counter() - t0, 3)),
        origin=req.origin, origin_sha256=req.origin_sha256, versions=_versions())
    return OptResult(md=md, optimiser=req.optimiser, side=req.side, target_s=req.target_s,
                     achieved_s=achieved, lines=lines, t_min_s=t_min, lines_min=lines_min,
                     recorded_s=recorded, met=met, uploadable=checks.passed, binding=bindings, checks=checks,
                     hints=hints, message=message, recipe=recipe)


def _limits_for(req: OptRequest, limits: LimitSet | None) -> LimitSet:
    lim = limits if limits is not None else load_limits(req.side)
    if lim.side != req.side:
        raise ValueError(f"limits are for the {lim.side} arm, the request is for the {req.side} arm")
    return lim.with_max_dev(req.max_dev_mm)


def optimise(req: OptRequest, limits: LimitSet | None = None) -> OptResult:
    """Optimise one recording: the target time if the limits allow it, else the closest."""
    vendor_paths.activate()
    req, cut = apply_cuts(req)
    s = _Solved(req, _limits_for(req, limits))
    if s.n_min is None:
        raise RuntimeError("no timing within the limits was found, even 20x slower than recorded")
    n_target = s.target_n()
    n = s.n_min if n_target is None or n_target < s.n_min else settle(s.search, n_target)
    return _finish(s, n, n_target, cut=cut)


def optimise_together(left: OptRequest, right: OptRequest,
                      limits: tuple[LimitSet, LimitSet] | None = None) -> tuple[OptResult, OptResult]:
    """A TOGETHER block: both arms start together, so both files get the same line count,
    max(target, left minimum, right minimum). The arm with the longer minimum is named."""
    vendor_paths.activate()
    if (left.side, right.side) != ("left", "right"):
        raise ValueError("optimise_together takes the left arm's request first, then the right's")
    sl = _Solved(left, _limits_for(left, None if limits is None else limits[0]))
    sr = _Solved(right, _limits_for(right, None if limits is None else limits[1]))
    if sl.n_min is None or sr.n_min is None:
        raise RuntimeError("no timing within the limits was found for one of the arms")
    targets = [t for t in (sl.target_n(), sr.target_n()) if t is not None]
    n_target = max(targets) if targets else None
    n = max([sl.n_min, sr.n_min] + ([n_target] if n_target is not None else []))
    for _ in range(20):                       # both must pass at the same n
        nl, nr = settle(sl.search, n), settle(sr.search, n)
        if nl == nr == n:
            break
        n = max(nl, nr)
    slow, fast = (sl, sr) if sl.n_min >= sr.n_min else (sr, sl)
    n_min_shared = slow.n_min
    note_fast = (f"Timed to match the {slow.req.side} arm, which sets the shared time."
                 if n_target is None or n_target < n_min_shared else None)
    res = {}
    for s in (sl, sr):
        r = _finish(s, n, n_target, n_min_shared=n_min_shared,
                    binding_note=note_fast if s is fast else None, own_binding=s is slow)
        r.recipe["together"] = dict(lines=n + 1, left_lines_min=sl.n_min + 1, right_lines_min=sr.n_min + 1,
                                    binding_arm=slow.req.side)
        res[s.req.side] = r
    return res["left"], res["right"]


def request_from_file(path, side: str, **kw) -> OptRequest:
    md = track.read_md(path)
    return OptRequest(md_raw=md, side=side, origin=str(path), origin_sha256=track.sha256_file(path), **kw)
