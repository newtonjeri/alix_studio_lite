#!/usr/bin/env python3
"""The commode's place, read again from the lab's recorded checks (Resources/Logs/<day>/checks): every frame through
ALIX's place reading (GapCheck.place: alix_perception_core.fixture_place), grouped by the arm pose it was taken from and
the state of the lid and the seat, and how it held frame to frame, check to check and day to day.

    python3 tools/placement_replay.py                     # every day
    python3 tools/placement_replay.py --day 2026-10-31    # one day (repeatable)
    --json PATH  every frame's place

Its ABSOLUTE place is off by the wrist camera's mount (a CAD value): what it says is a CHANGE of place, between frames
from the same arm pose (the named pose within 1 deg, the same pole height) with the lid and the seat in the same state.
A frame whose part is between closed and open (held) gives no place. Each frame is read on its own, posed as
tools/parts_replay.py poses it (a stale first arm reading replaced by its check's settled one).

On 31 Oct, over the 1155 frames of 21-31 Oct (inspection pose 4 on 24-26 Oct, pose 1 on 26-31 Oct): the frames of
one check agreed to 0.4 / 0.6 / 0.3 mm and 0.2 deg (median); the twelve starts of Close the seat of 30 Oct, over three
hours, to 0.24 / 0.25 mm; the change from one day to the next read alike in two states from one pose (26 to 27 Oct,
pose 1: y -3.3 mm with the seat down, -3.6 with it open); pose 4 and pose 1 read the same commode 10.7 / -14.8 mm and
-3.2 deg apart (the mount). Weak: the lid shut seen from pose 4 (the glove hides its front edge: frames up to 5 to 8 mm
apart along the commode) and from pose 1 (nothing to use); an open seat resting otherwise (lifted with the lid) moves
the reading by up to 5 mm. The 21-24 Oct lid lifts that missed stood where the ones that lifted stood (within 4 mm):
those misses were the plan's (revised 24 Oct), not the commode's place.
"""
from __future__ import annotations

import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse  # noqa: E402
import json  # noqa: E402
import sys  # noqa: E402
from collections import defaultdict  # noqa: E402
from concurrent.futures import ProcessPoolExecutor  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import parts_replay  # noqa: E402

_GC = None
AXES = ("dx_m", "dy_m", "dz_m", "dyaw_deg")


def _gc():
    global _GC
    if _GC is None:
        from alix_studio_lite import vendor_paths
        from alix_studio_lite.server.gap_check import GapCheck, load_params

        vendor_paths.activate()
        _GC = GapCheck(params={**load_params(), "gap_cross_check": False}, records=None)
    return _GC


def pose_of(q) -> str:
    """The right arm's named pose (ALIX's group states) within 1 deg on every joint, else "other"."""
    if not q:
        return "unknown"
    gc = _gc()
    for name, joints in gc.named_poses().items():
        want = np.degrees([joints.get(f"R_joint{i + 1}", np.nan) for i in range(7)])
        if np.all(np.abs(np.asarray(q, dtype=float)[:7] - want) <= 1.0):
            return name
    return "other"


def _one(x: dict) -> dict:
    from alix_studio_lite.server.gap_check import DepthFrame, Intrinsics

    gc = _gc()
    depth = np.load(x["path"] + "_depth.npy").astype(np.float32)
    k = x["intr"]
    frame = DepthFrame(depth=depth, intrinsics=Intrinsics(k["fx"], k["fy"], k["cx"], k["cy"], depth.shape[1], depth.shape[0]),
                       cam_to_base=np.asarray(x["cam"], float), hands=x["hands"], source="native depth")
    rec = json.loads(Path(x["path"] + ".json").read_text())
    right = ((rec.get("context") or {}).get("arms") or {}).get("right") or {}
    p = gc.place(frame)
    return dict(stamp=x["stamp"], day=x["day"], check_id=x["check_id"], stage=x["stage"], path=x["path"],
                pose=pose_of(x["q"]) if x["q"] else "unknown (no arm reading)",
                pole_mm=right.get("pole_mm"), ok=bool(p["ok"]), state=p["state"], note=p["note"],
                lid_deg=p.get("lid_deg"), seat_deg=p.get("seat_deg"), **{a: float(p.get(a) or 0.0) for a in AXES},
                sharp=(p.get("record") or {}).get("sharp"))


def _fmt(v) -> str:
    return " ".join(f"{v[i] * (1000.0 if i < 3 else 1.0):+6.1f}" for i in range(4))


def report(res: list[dict]) -> None:
    ok = [r for r in res if r["ok"]]
    print(f"{len(res)} frames: {len(ok)} with a place; the others: "
          + ", ".join(f"{n} {why}" for why, n in sorted(_why(res).items(), key=lambda kv: -kv[1])))
    print("(x / y / z mm, yaw deg; the pole height with the pose: a pose's place compares only with its own)")
    by_check = defaultdict(list)
    for r in ok:
        by_check[(r["check_id"], r["pose"], r["pole_mm"], r["state"])].append(r)
    spread = np.array([np.ptp([[x[a] for a in AXES] for x in g], axis=0) for g in by_check.values() if len(g) > 1])
    if len(spread):
        print(f"\nthe frames of one check ({len(spread)} checks): spread median {_fmt(np.median(spread, 0))}  "
              f"90th percentile {_fmt(np.percentile(spread, 90, 0))}")
    days = defaultdict(list)
    for (cid, pose, pole, state), g in by_check.items():
        days[(pose, pole, state, g[0]["day"])].append(np.median([[x[a] for a in AXES] for x in g], axis=0))
    print("\nper pose and state, each day: checks, the median of their medians, their standard deviation, "
          "the change from the first day")
    first: dict = {}
    for (pose, pole, state, day), ms in sorted(days.items(), key=lambda kv: (str(kv[0][0]), str(kv[0][1]), kv[0][2], kv[0][3])):
        key = (pose, pole, state)
        a = np.asarray(ms)
        m = np.median(a, 0)
        first.setdefault(key, m)
        if first[key] is m:
            print(f"  {pose}, pole {pole} mm, {state}")
        print(f"     {day} {len(a):3d}  {_fmt(m)}   sd {_fmt(a.std(0))}   change {_fmt(m - first[key])}")
    print("\nthe same pose and day, two states: the commode's change from the pose's first day, read in each")
    keyed = {k: np.median(v, 0) for k, v in days.items()}
    for (pose, pole, day) in sorted({(k[0], k[1], k[3]) for k in keyed}, key=str):
        states = sorted({k[2] for k in keyed if (k[0], k[1], k[3]) == (pose, pole, day)})
        rows = []
        for s in states:
            d0 = min(k[3] for k in keyed if k[:3] == (pose, pole, s))
            if d0 != day and (pose, pole, s, d0) in keyed:
                rows.append((s, keyed[(pose, pole, s, day)] - keyed[(pose, pole, s, d0)], d0))
        if len(rows) > 1:
            print(f"  {pose} {day}: " + " | ".join(f"{s} (since {d0}) {_fmt(c)}" for s, c, d0 in rows))
    print("\nthe mount: inspection pose 4 against pose 1, the same day and state")
    for (pose, pole, state, day), m in sorted(keyed.items(), key=lambda kv: str(kv[0])):
        if pose == "inspection_pose_4":
            for (p1, pole1, s1, d1), m1 in keyed.items():
                if p1 == "inspection_pose_1" and s1 == state and d1 == day:
                    print(f"  {day} {state}: pose 4 minus pose 1 {_fmt(m - m1)}")


def _why(res: list[dict]) -> dict:
    out: dict = defaultdict(int)
    for r in res:
        if not r["ok"]:
            note = r["note"]
            out["a part held" if "between closed and open" in note else "at the search's edge" if "edge" in note
                else "too few pixels" if "pixels" in note else note[:60]] += 1
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--day", action="append", help="a day's folder name (YYYY-MM-DD); default every day")
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--workers", type=int, default=8)
    args = ap.parse_args()
    rows = parts_replay.frames(args.day)
    with ProcessPoolExecutor(args.workers) as pool:
        res = list(pool.map(_one, rows, chunksize=4))
    report(res)
    if args.json:
        args.json.write_text(json.dumps(res))
    return 0


if __name__ == "__main__":
    sys.exit(main())
