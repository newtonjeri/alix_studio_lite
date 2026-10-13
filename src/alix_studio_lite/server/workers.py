"""Work that runs in the optimiser process, away from the web server.

numpy, scipy and toppra load here, not in the server, so the page stays responsive and a
long solve never blocks a STOP.
"""
from __future__ import annotations

import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

_POOL: ProcessPoolExecutor | None = None


def pool() -> ProcessPoolExecutor:
    global _POOL
    if _POOL is None:
        _POOL = ProcessPoolExecutor(max_workers=1, mp_context=mp.get_context("spawn"))
    return _POOL


def shutdown() -> None:
    global _POOL
    if _POOL is not None:
        _POOL.shutdown(wait=False, cancel_futures=True)
        _POOL = None


def _maxpool(u: np.ndarray, buckets: int = 1500) -> list[float]:
    u = np.asarray(u, dtype=float)
    if u.shape[0] <= buckets:
        return [round(float(x), 4) for x in u]
    edges = np.linspace(0, u.shape[0], buckets + 1).astype(int)
    return [round(float(u[a:b].max()), 4) for a, b in zip(edges[:-1], edges[1:])]


def optimise_unit(raw_path: str, out_path: str, side: str, settings: dict) -> dict:
    """Optimise one recording with the unit's settings and write the candidate file."""
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from dataclasses import replace

    from alix_studio_lite.motion.checks import util_series
    from alix_studio_lite.optimise import (DefaultOptions, Segment, optimise, request_from_file,
                                           toppra_available, write_result)

    opt = settings.get("optimiser") or "default"
    if opt == "toppra" and not toppra_available():
        return dict(error="Time-optimal is not available on this computer.",
                    hint="Use the Default optimiser, or install toppra (see README).")
    segs = tuple(Segment(int(s["start"]), int(s["end"]), float(s["pct"]), s.get("label", ""))
                 for s in settings.get("segments") or [])
    kw = dict(optimiser=opt, target_s=settings.get("target_s"), segments=segs,
              max_dev_mm=settings.get("max_dev_mm"))
    if opt == "default" and settings.get("smooth_window"):
        kw["default"] = replace(DefaultOptions(), smooth_window=int(settings["smooth_window"]))
    res = optimise(request_from_file(raw_path, side, **kw))
    out, side_file = write_result(res, out_path)
    from alix_studio_lite.motion.limits import load_limits

    util = util_series(res.md, load_limits(side))
    speed = np.max(np.vstack([util[f"speed_J{j}"] for j in range(1, 8)]), axis=0)
    acc = np.max(np.vstack([util[f"acceleration_J{j}"] for j in range(1, 8)]), axis=0)
    return dict(summary=res.summary(), checks=res.checks.to_dict(), recipe=res.recipe,
                file=str(out), record=str(side_file),
                util=dict(speed=_maxpool(speed), acceleration=_maxpool(acc), lines=res.lines))


def track_arrays(path: str) -> dict:
    """A track for plotting: degrees per joint (rounded to 0.01 deg) at 5 ms per line."""
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.motion import track

    md = track.read_md(Path(path))
    q = np.round(md / 1000.0, 2)
    return dict(lines=int(md.shape[0]), cycle_s=0.005, joints=[q[:, j].tolist() for j in range(7)])
