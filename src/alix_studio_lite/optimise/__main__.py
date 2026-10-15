"""Command line: tune one recording.

    python3 -m alix_studio_lite.optimise RECORDING.txt --side left --segments "1-2000:75,2001-4000:115" \
        --out tuned/FILE_01.1_trim_mean_scale_processed.txt
    python3 -m alix_studio_lite.optimise RECORDING.txt --side left --optimiser toppra --target 25

Manual (the default) is the trajectory HTML's algorithm; its limits are reported, never enforced.
Time-optimal (``--optimiser toppra``) keeps every limit. Prints what was achieved; with ``--out``
writes the track and its ``.lite.json`` record.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def _segments(text: str) -> list[dict]:
    out = []
    for part in filter(None, (p.strip() for p in (text or "").split(","))):
        rng, _, pct = part.partition(":")
        a, _, b = rng.partition("-")
        out.append(dict(start=int(a), end=int(b or a), pct=float(pct)))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python3 -m alix_studio_lite.optimise", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", type=Path)
    ap.add_argument("--side", required=True, choices=("left", "right"))
    ap.add_argument("--optimiser", default="manual", choices=("manual", "toppra"))
    ap.add_argument("--target", type=float, default=None,
                    help="time-optimal: target seconds; N lines last (N - 1) x 5 ms (omit: fastest safe)")
    ap.add_argument("--segments", default="", help='manual: scale regions, e.g. "1-2000:75,2001-4000:115" (recording rows)')
    ap.add_argument("--window", type=int, default=None, help="manual: the mean smoothing window (default 101)")
    ap.add_argument("--no-trim", action="store_true", help="manual: skip Trim")
    ap.add_argument("--no-smooth", action="store_true", help="manual: skip Mean Smooth")
    ap.add_argument("--max-dev-mm", type=float, default=None, help="the region's MAX DEV")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--json", action="store_true", help="print the full record as JSON")
    a = ap.parse_args(argv)

    from .. import vendor_paths

    vendor_paths.activate()
    from ..motion import track
    from .sidecar import record, write_result

    if a.optimiser == "manual":
        from . import manual

        settings = dict(trim=not a.no_trim, smooth=not a.no_smooth, scale=True, segments=_segments(a.segments),
                        max_dev_mm=a.max_dev_mm)
        if a.window is not None:
            settings["smooth_window"] = a.window
        res = manual.optimise(track.read_md(a.recording), a.side, settings, source_name=a.recording.name)
        ok = True
    else:
        from .engine import optimise, request_from_file

        res = optimise(request_from_file(a.recording, a.side, optimiser="toppra", target_s=a.target, max_dev_mm=a.max_dev_mm))
        ok = res.uploadable
    if a.out:
        out, side = write_result(res, a.out)
        print(f"wrote {out} and {side.name}")
    if a.json:
        print(json.dumps(record(res, track.sha256_bytes(track.track_bytes(res.md))), indent=2, default=float))
        return 0 if ok else 2
    print(res.message)
    print(f"  {res.lines} lines = {res.achieved_s:.2f} s at 5 ms per line; recorded {res.recorded_s:.2f} s"
          + (f"; fastest safe {res.t_min_s:.2f} s" if a.optimiser == "toppra" else ""))
    for c in res.checks.checks:
        mark = {True: "ok  ", False: ("FAIL" if a.optimiser == "toppra" else "over") if c.hard else "warn", None: "--  "}[c.passed]
        print(f"  [{mark}] {c.id} {c.name}: {c.detail}")
    for n in res.checks.notes:
        print(f"  note: {n}")
    for h in getattr(res, "hints", []) or []:
        print(f"  hint: {h}")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
