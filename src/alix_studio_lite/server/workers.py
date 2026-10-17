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


def kill_now() -> None:
    """Lite is quitting at once (a second Ctrl+C, or its exit past its limit): the tuning worker is ended
    without waiting for any lock (a signal handler calls this). It ignores Ctrl+C, so it would outlive Lite,
    holding the terminal open."""
    pool = _POOL
    for p in list((getattr(pool, "_processes", None) or {}).values()) if pool is not None else []:
        try:
            p.kill()
        except Exception:                                                   # noqa: BLE001
            pass


def _maxpool(u: np.ndarray, buckets: int = 1500) -> list[float]:
    u = np.asarray(u, dtype=float)
    if u.shape[0] <= buckets:
        return [round(float(x), 4) for x in u]
    edges = np.linspace(0, u.shape[0], buckets + 1).astype(int)
    return [round(float(u[a:b].max()), 4) for a, b in zip(edges[:-1], edges[1:])]


def is_manual(settings: dict) -> bool:
    """Manual tuning (the trajectory HTML's algorithm) unless Time-optimal is chosen. A region set
    before 15 Oct says "default" (the earlier name): it tunes manually now."""
    return (settings.get("optimiser") or "manual") != "toppra"


def request_kwargs(settings: dict) -> dict:
    """The time-optimal request for a region's settings: the recording as taught, the target time
    when one is set (else the fastest the limits allow). Nothing of the manual tuning goes in."""
    mode = settings.get("time_mode") or ("target" if settings.get("target_s") else "fastest")
    return dict(optimiser="toppra", max_dev_mm=settings.get("max_dev_mm"),
                target_s=settings.get("target_s") if mode == "target" else None)


def progress_rows(md_raw: np.ndarray, md_out: np.ndarray, samples: int = 1200) -> list:
    """Where a time-optimal file is on the recording: [delivered line, recording line] pairs
    (both 1-based) at the same share of the path travelled in joint space, so pauses of the
    recording take no share and the plot's playhead skips them as the tuned file does. (A manual
    file knows each line's row exactly: ``manual.progress_pairs``.)"""
    from scipy.ndimage import uniform_filter1d

    rows = np.arange(md_raw.shape[0])

    def travelled(md):
        q = uniform_filter1d(np.asarray(md, dtype=float) / 1000.0, size=9, axis=0, mode="nearest")
        return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(q, axis=0), axis=1))])

    d_raw, d_out = travelled(md_raw), travelled(md_out)
    out_lines = np.unique(np.linspace(0, md_out.shape[0] - 1, min(samples, md_out.shape[0])).astype(int))
    share = d_out[out_lines] / max(d_out[-1], 1e-9)
    idx = np.clip(np.searchsorted(d_raw, share * d_raw[-1]), 0, rows.shape[0] - 1)
    return [[int(o) + 1, int(rows[i]) + 1] for o, i in zip(out_lines, idx)]


def optimise_unit(raw_path: str, out_path: str, side: str, settings: dict, source_name: str = "") -> dict:
    """Tune one recording with the region's settings and write the candidate file: manually (the
    HTML's algorithm, limits reported) or time-optimally (TOPP-RA, limits kept)."""
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.motion import track
    from alix_studio_lite.motion.checks import util_series
    from alix_studio_lite.motion.limits import load_limits
    from alix_studio_lite.optimise import manual, optimise, request_from_file, toppra_available, write_result

    if is_manual(settings):
        md_raw = track.read_md(raw_path)
        try:
            res = manual.optimise(md_raw, side, settings, source_name=source_name or Path(raw_path).name)
        except ValueError as exc:                                # no stage on, regions overlapping after Trim
            return dict(error=str(exc), hint="Check the processing stages and the scale regions.")
        progress = manual.progress_pairs(res)
    else:
        if not toppra_available():
            return dict(error="Time-optimal is not available on this computer.",
                        hint="Tune it manually, or install toppra (see README).")
        req = request_from_file(raw_path, side, **request_kwargs(settings))
        res = optimise(req)
        progress = progress_rows(req.md_raw, res.md)
    out, side_file = write_result(res, out_path)
    util = util_series(res.md, load_limits(side))
    speed = np.max(np.vstack([util[f"speed_J{j}"] for j in range(1, 8)]), axis=0)
    acc = np.max(np.vstack([util[f"acceleration_J{j}"] for j in range(1, 8)]), axis=0)
    return dict(summary=res.summary(), checks=res.checks.to_dict(), recipe=res.recipe,
                file=str(out), record=str(side_file),
                util=dict(speed=_maxpool(speed), acceleration=_maxpool(acc), lines=res.lines),
                progress=progress)


def track_arrays(path: str, counts: bool = False) -> dict:
    """A track for plotting: degrees per joint (rounded to 0.01 deg) at 5 ms per line, or with
    ``counts`` the file's own integers (millidegrees)."""
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.motion import track

    md = track.read_md(Path(path))
    q = md if counts else np.round(md / 1000.0, 2)
    return dict(lines=int(md.shape[0]), cycle_s=0.005, units="counts" if counts else "deg",
                joints=[q[:, j].tolist() for j in range(7)])
