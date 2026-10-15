"""TIME-OPTIMAL tuning (TOPP-RA), the alternative to manual tuning: one region on one arm
(``optimise``), or a TOGETHER pair that must start and finish together (``optimise_together``).

It works on the recording as taught, through ALIX's own chain (``toppra_opt``), and never goes
through the manual tuning's Trim, Mean Smooth or Scale, its suggestions or its scale regions
(the owner, 15 Oct: "the other option is toppra, and should never undergo the approach in the
html"). Its limits stay enforced: the file is the target time when the limits allow it, else the
fastest time they allow, and a file that still fails a hard limit is never uploadable.
"""
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
from . import toppra_opt
from .target import Search, find_min_lines, settle
from .types import OPTIMISERS, OptRequest, OptResult


def make_plan(req: OptRequest, limits: LimitSet):
    if req.optimiser == "toppra":
        return toppra_opt.ToppraPlan(req.md_raw, limits, req.toppra)
    raise ValueError(f"the time-optimal engine runs {OPTIMISERS[1]!r} only, got {req.optimiser!r} "
                     "(manual tuning is optimise.manual)")


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
            binding_note: str | None = None, own_binding: bool = True) -> OptResult:
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
        hint = toppra_opt.smoothing_hint(req.md_raw, limits, req.toppra)
        if hint:
            hints.append(hint)
    for c in checks.failures():
        if c.speed_fixes is False and c.id == "L1":
            hints.append("A joint goes too close to its end stop: re-teach that part of the region.")
    achieved = timing.duration_for(lines)
    t_min = timing.duration_for(lines_min)
    recorded = timing.duration_for(len(req.md_raw))
    why = bnd.describe(bindings)
    if not checks.passed:
        first = checks.failures()[0]
        message = f"Not ready to upload: {first.name.lower()} - {first.detail}"
    elif req.target_s is None:
        message = f"Fastest safe time: {achieved:.2f} s" + (f", {why}." if why else ".")
    elif not met:
        message = (f"The robot cannot do this region in {req.target_s:.2f} s within its limits. "
                   f"The fastest safe time is {t_min:.2f} s" + (f", {why}." if why else "."))
    elif not exact:
        message = f"Target {req.target_s:.2f} s adjusted to {achieved:.2f} s to stay within the limits after rounding."
    else:
        message = f"Target met: {achieved:.2f} s."
    if binding_note:
        message = f"{message} {binding_note}"
    recipe = dict(
        optimiser=req.optimiser, side=req.side, frame=limits.frame, cycle_s=timing.CYCLE_S,
        target_s=req.target_s, options=asdict(req.toppra), max_dev_mm=req.max_dev_mm, enforced=True,
        limits=dict(rating_deg_s=list(limits.rating_deg_s), rate_bar=limits.rate_bar,
                    rate_design=limits.rate_design, acc_deg_s2=limits.acc_deg_s2, acc_tol=limits.acc_tol,
                    acc_reading="20 Hz band-limited (ALIX F99c)", step_deg=limits.step_deg,
                    max_dev_mm=limits.max_dev_mm, ends_tol_deg=limits.ends_tol_deg,
                    dt_gate_s=limits.dt_gate, execution_model=limits.model.get("fitted")),
        plan=s.plan.recipe(), lines=lines, lines_min=lines_min, recorded_lines=int(len(req.md_raw)),
        natural=req.natural, speedup_vs_recording=round(len(req.md_raw) / lines, 3),
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
    """Time-optimise one recording: the target time if the limits allow it, else the closest."""
    vendor_paths.activate()
    s = _Solved(req, _limits_for(req, limits))
    if s.n_min is None:
        raise RuntimeError("no timing within the limits was found, even 20x slower than recorded")
    n_target = s.target_n()
    n = s.n_min if n_target is None or n_target < s.n_min else settle(s.search, n_target)
    return _finish(s, n, n_target)


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
