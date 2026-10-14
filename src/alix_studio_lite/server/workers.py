"""Work that runs in the optimiser process, away from the web server.

numpy, scipy and toppra load here, not in the server, so the page stays responsive and a
long solve never blocks a STOP.
"""
from __future__ import annotations

import multiprocessing as mp
import signal
import threading
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

_POOL: ProcessPoolExecutor | None = None
_POOL_LOCK = threading.Lock()          # two first tunes at once still make one pool, one worker


def _worker_start() -> None:
    """Ctrl+C in Lite's terminal reaches every process in it: the worker leaves it to the server,
    which ends it in shutdown() (it printed a KeyboardInterrupt traceback on its way out)."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def pool() -> ProcessPoolExecutor:
    global _POOL
    with _POOL_LOCK:
        if _POOL is None:
            _POOL = ProcessPoolExecutor(max_workers=1, mp_context=mp.get_context("spawn"), initializer=_worker_start)
        return _POOL


def shutdown() -> None:
    """The server is stopping: a tune still running is ended, not waited for."""
    global _POOL
    with _POOL_LOCK:
        if _POOL is not None:
            procs = list((getattr(_POOL, "_processes", None) or {}).values())
            _POOL.shutdown(wait=False, cancel_futures=True)
            for p in procs:
                if p.is_alive():
                    p.terminate()
            _POOL = None


def _maxpool(u: np.ndarray, buckets: int = 1500) -> list[float]:
    u = np.asarray(u, dtype=float)
    if u.shape[0] <= buckets:
        return [round(float(x), 4) for x in u]
    edges = np.linspace(0, u.shape[0], buckets + 1).astype(int)
    return [round(float(u[a:b].max()), 4) for a, b in zip(edges[:-1], edges[1:])]


def request_kwargs(settings: dict) -> dict:
    """The optimiser request for a unit's settings. Legacy Lite's settings have no switches, so
    they keep their meaning (everything on, no target = as fast as the limits allow); the Lite
    UI adds its processing switches (Trim with cuts, Mean Smooth, Scale) and a time mode."""
    from dataclasses import replace

    from alix_studio_lite.optimise import DefaultOptions, Segment

    opt = settings.get("optimiser") or "default"
    scale = settings.get("scale", True)
    segs = tuple(Segment(int(s["start"]), int(s["end"]), float(s["pct"]), s.get("label", ""))
                 for s in (settings.get("segments") or []) if scale)
    trim = settings.get("trim", True)
    mode = settings.get("time_mode") or ("target" if settings.get("target_s") else "fastest")
    kw = dict(optimiser=opt, segments=segs, max_dev_mm=settings.get("max_dev_mm"),
              target_s=settings.get("target_s") if mode == "target" else None, natural=mode == "natural",
              cuts=tuple((int(c["start"]), int(c["end"])) for c in (settings.get("cuts") or []) if trim))
    if opt == "default":
        window = (int(settings.get("smooth_window") or DefaultOptions().smooth_window)
                  if settings.get("smooth", True) else 0)
        kw["default"] = replace(DefaultOptions(), trim=bool(trim), smooth_window=window)
    return kw


def progress_rows(md_raw: np.ndarray, md_out: np.ndarray, cuts=(), samples: int = 1200) -> list:
    """Where the delivered file is on the recording: [delivered line, recording line] pairs
    (both 1-based) at the same share of the path travelled in joint space. Pauses and cut
    stretches of the recording take no share, so the plot's playhead skips them as the tuned
    file does."""
    from scipy.ndimage import uniform_filter1d

    n = md_raw.shape[0]
    keep = np.ones(n, dtype=bool)
    for a, b in cuts:
        keep[max(1, int(a)) - 1:min(n, int(b))] = False
    rows = np.flatnonzero(keep)

    def travelled(md):
        q = uniform_filter1d(np.asarray(md, dtype=float) / 1000.0, size=9, axis=0, mode="nearest")
        return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(q, axis=0), axis=1))])

    d_raw, d_out = travelled(md_raw[keep]), travelled(md_out)
    out_lines = np.unique(np.linspace(0, md_out.shape[0] - 1, min(samples, md_out.shape[0])).astype(int))
    share = d_out[out_lines] / max(d_out[-1], 1e-9)
    idx = np.clip(np.searchsorted(d_raw, share * d_raw[-1]), 0, rows.shape[0] - 1)
    return [[int(o) + 1, int(rows[i]) + 1] for o, i in zip(out_lines, idx)]


def optimise_unit(raw_path: str, out_path: str, side: str, settings: dict) -> dict:
    """Optimise one recording with the unit's settings and write the candidate file."""
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.motion.checks import util_series
    from alix_studio_lite.optimise import optimise, request_from_file, toppra_available, write_result

    kw = request_kwargs(settings)
    if kw["optimiser"] == "toppra" and not toppra_available():
        return dict(error="Time-optimal is not available on this computer.",
                    hint="Use the Default optimiser, or install toppra (see README).")
    req = request_from_file(raw_path, side, **kw)
    try:
        res = optimise(req)
    except ValueError as exc:                                    # e.g. cuts that remove everything
        return dict(error=f"These settings cannot be used: {exc}.", hint="Check the trim and scale regions.")
    out, side_file = write_result(res, out_path)
    from alix_studio_lite.motion.limits import load_limits

    util = util_series(res.md, load_limits(side))
    speed = np.max(np.vstack([util[f"speed_J{j}"] for j in range(1, 8)]), axis=0)
    acc = np.max(np.vstack([util[f"acceleration_J{j}"] for j in range(1, 8)]), axis=0)
    return dict(summary=res.summary(), checks=res.checks.to_dict(), recipe=res.recipe,
                file=str(out), record=str(side_file),
                util=dict(speed=_maxpool(speed), acceleration=_maxpool(acc), lines=res.lines),
                progress=progress_rows(req.md_raw, res.md, kw["cuts"]))


def track_arrays(path: str) -> dict:
    """A track for plotting: degrees per joint (rounded to 0.01 deg) at 5 ms per line."""
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.motion import track

    md = track.read_md(Path(path))
    q = np.round(md / 1000.0, 2)
    return dict(lines=int(md.shape[0]), cycle_s=0.005, joints=[q[:, j].tolist() for j in range(7)])
