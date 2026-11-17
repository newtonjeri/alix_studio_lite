#!/usr/bin/env python3
"""How long Lite's lid and seat readings take on this computer, alone and with the wrist camera running beside them in
the same process, as in a run. No robot, no motion: the frames are the ones this computer recorded (Resources/Logs/<day>/
checks), read again by Lite's own gap check; the camera is opened by Lite's own D435 class and only streams.

    python3 tools/reading_timing.py                     # the newest day with recorded frames
    python3 tools/reading_timing.py --day 2026-11-15
    python3 tools/reading_timing.py --no-camera         # the readings alone (no camera here, or another program holds it)

Stop Lite first: two programs cannot open the D435 at once.

It prints the median time of a lid and of a seat reading alone, then with the camera open, and the camera thread's
frame rate (Lite filters and aligns every frame: spatial, temporal, align; 30 a second when it keeps up). On 15 Nov,
the Jetson's frames read on the lab's laptop: lid 0.66 s alone, 0.82 s with a camera thread beside it; seat 0.20 s and
0.27 s. In Lite on the Jetson they took 2.1 s and 1.8 s. Nothing is written, changed or sent."""
from __future__ import annotations

import argparse
import statistics as st
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "tools")]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--day", default=None, help="a day folder of Resources/Logs (YYYY-MM-DD); default the newest with frames")
    ap.add_argument("--frames", type=int, default=6, help="lid frames and seat frames each (default 6)")
    ap.add_argument("--no-camera", action="store_true")
    a = ap.parse_args()

    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    import parts_replay as pr

    pr._init("parts")
    days = sorted(p.name for p in (ROOT / "Resources" / "Logs").glob("2026-*") if any((p / "checks").glob("*_depth.npy")))
    day = a.day or (days[-1] if days else None)
    if not day:
        print("No recorded frames with depth under Resources/Logs: run a lid or seat check first.")
        return 1
    rows = pr.frames([day])
    lid = [x for x in rows if x["expect"] in ("lid_closed", "lid_open", "lid_lifted")][: a.frames]
    seat = [x for x in rows if x["expect"] in ("seat_closed", "seat_open") or x["stage"] == "verify_seat_with_lid"][: a.frames]
    if not lid and not seat:
        print(f"No lid or seat frames on {day}.")
        return 1
    print(f"{day}: {len(lid)} lid and {len(seat)} seat frames recorded on this computer")
    for x in lid + seat:                                    # loaded once, as Lite's warm-up does
        pr._one(x)

    def timed(xs):
        out = []
        for x in xs:
            t = time.perf_counter()
            pr._one(x)
            out.append(time.perf_counter() - t)
        return st.median(out) if out else float("nan")

    alone = (timed(lid), timed(seat))
    print(f"alone:              lid {alone[0]:.2f} s, seat {alone[1]:.2f} s")
    if a.no_camera:
        return 0
    from alix_studio_lite.server.camera_d435 import D435

    cam = D435(log=lambda text: print(f"  camera: {text}"))
    try:
        cam.open()
    except Exception as exc:                                                        # noqa: BLE001
        print(f"The camera could not be opened ({exc}); the readings alone are above. If Lite is running, stop it (two "
              "programs cannot open the D435 at once), then run this again.")
        return 1
    try:
        time.sleep(2.0)                                     # its warm-up, as in a check
        seq0, t0 = cam._seq, time.monotonic()
        stop, ran = threading.Event(), []

        def count():                                        # the camera's frame rate while the readings run
            while not stop.wait(0.5):
                ran.append((time.monotonic(), cam._seq))

        th = threading.Thread(target=count, daemon=True)
        th.start()
        busy = (timed(lid), timed(seat))
        stop.set()
        th.join(1.0)
        fps = (cam._seq - seq0) / max(1e-6, time.monotonic() - t0)
    finally:
        cam.close()
    print(f"with the camera on: lid {busy[0]:.2f} s ({busy[0] / alone[0]:.1f}x), seat {busy[1]:.2f} s ({busy[1] / alone[1]:.1f}x); "
          f"the camera thread kept {fps:.1f} frames a second (30 when it keeps up)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
