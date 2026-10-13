"""Command line: optimise one recording.

    python3 -m alix_studio_lite.optimise RECORDING.txt --side left --optimiser toppra --target 25 \
        --out optimised/FILE_01.1_lite.txt

Prints what was achieved; with ``--out`` writes the track and its ``.lite.json`` record.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def _segments(text: str):
    from .types import Segment

    out = []
    for part in filter(None, (p.strip() for p in (text or "").split(","))):
        rng, _, pct = part.partition(":")
        a, _, b = rng.partition("-")
        out.append(Segment(int(a), int(b or a), float(pct)))
    return tuple(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python3 -m alix_studio_lite.optimise", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", type=Path)
    ap.add_argument("--side", required=True, choices=("left", "right"))
    ap.add_argument("--optimiser", default="default", choices=("default", "toppra"))
    ap.add_argument("--target", type=float, default=None,
                    help="target seconds; N lines last (N - 1) x 5 ms (omit: fastest safe)")
    ap.add_argument("--segments", default="", help='e.g. "1-2000:75,2001-4000:115" (recording lines)')
    ap.add_argument("--max-dev-mm", type=float, default=None, help="the region's MAX DEV")
    ap.add_argument("--window", type=int, default=None, help="default optimiser: smoothing window")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--json", action="store_true", help="print the full record as JSON")
    a = ap.parse_args(argv)

    from .. import vendor_paths

    vendor_paths.activate()
    from dataclasses import replace

    from .engine import optimise, request_from_file
    from .sidecar import record, write_result
    from .types import DefaultOptions

    kw = dict(optimiser=a.optimiser, target_s=a.target, segments=_segments(a.segments),
              max_dev_mm=a.max_dev_mm)
    if a.window is not None:
        kw["default"] = replace(DefaultOptions(), smooth_window=a.window)
    res = optimise(request_from_file(a.recording, a.side, **kw))
    if a.out:
        out, side = write_result(res, a.out)
        print(f"wrote {out} and {side.name}")
    if a.json:
        from ..motion import track

        print(json.dumps(record(res, track.sha256_bytes(track.track_bytes(res.md))), indent=2, default=float))
        return 0 if res.uploadable else 2
    print(res.message)
    print(f"  {res.lines} lines = {res.achieved_s:.2f} s at 5 ms per line; recorded {res.recorded_s:.2f} s; "
          f"fastest safe {res.t_min_s:.2f} s")
    for c in res.checks.checks:
        mark = {True: "ok  ", False: "FAIL" if c.hard else "warn", None: "--  "}[c.passed]
        print(f"  [{mark}] {c.id} {c.name}: {c.detail}")
    for n in res.checks.notes:
        print(f"  note: {n}")
    for h in res.hints:
        print(f"  hint: {h}")
    return 0 if res.uploadable else 2


if __name__ == "__main__":
    sys.exit(main())
