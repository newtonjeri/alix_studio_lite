#!/usr/bin/env python3
"""The lab's recorded checks asked again: every frame Lite recorded (Resources/Logs/<day>/checks) through the gap check,
asked the question its check asked, as the runner asks it, and its vote set beside the one the lab's camera gave.

    python3 tools/parts_replay.py                         # every day, the parts reading deciding (gap_decider "parts")
    python3 tools/parts_replay.py --day 2026-10-31        # one day
    python3 tools/parts_replay.py --decider windows       # the windows deciding, as they did at the lab
    python3 tools/parts_replay.py --sheets /tmp/sheets    # contact sheets of the frames whose vote differs, to look at
    --json PATH  every frame's answer

Each frame is judged on its own (nothing carried from frame to frame: the parts reading learns nothing). A frame whose
arm reading was taken before the arm stopped (its check's first, 31 Oct) is posed by its check's settled reading. A
closing's end check (and the seat question after Open the lid) is asked with the closing allowance, as the runner asks
it; before 30 Oct, a closed-part check from inspection pose 1 was a closing's. The vote is lift_check.frame_vote's.
On 31 Oct, over the 1155 frames of 21-31 Oct: the parts reading voted as the lab's camera on 1081, never the other way;
of the 74 others it decided 66 the lab could not (every one as the photos show) and left 8 seats under a lifted lid
undecided, by rule.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
LOGS = Path(__file__).resolve().parents[1] / "Resources" / "Logs"
_GC = None
_DECIDER = "parts"


def frames(days: list[str] | None = None) -> list[dict]:
    """Every recorded frame with its depth: the check it belongs to, what the lab decided, its pose (a stale first
    reading replaced by the check's settled one)."""
    out = []
    for day in sorted(LOGS.glob("2026-*")):
        if days and day.name not in days:
            continue
        rows = []
        for r in sorted((day / "checks").glob("*_gap_check_*.json")):
            if not Path(str(r)[:-5] + "_depth.npy").is_file():
                continue
            d = json.loads(r.read_text())
            ctx = d.get("context") or {}
            chk = ctx.get("check") or {}
            rows.append(dict(path=str(r)[:-5], day=day.name, stamp=r.name[:19], check_id=chk.get("check_id") or r.name[:15],
                             stage=chk.get("stage") or "", closing=bool(chk.get("closing")), lab=d.get("class"),
                             expect=d.get("expect") or chk.get("expect") or r.name.split("_gap_check_")[1].rsplit("_", 1)[0],
                             cam=d.get("camera_to_world"), intr=d.get("intrinsics"), hands=d.get("hands") or {},
                             q=((ctx.get("arms") or {}).get("right") or {}).get("q_deg")))
        by: dict = {}
        for x in rows:
            by.setdefault(x["check_id"], []).append(x)
        for group in by.values():
            last = group[-1]
            for x in group:
                if x["q"] and last["q"] and np.abs(np.asarray(x["q"]) - np.asarray(last["q"])).max() > 1.0:
                    x["cam"], x["hands"] = last["cam"], last["hands"]
        out.extend(x for x in rows if x["cam"] is not None and x["intr"])
    return out


def closing_of(x: dict) -> bool:
    if x["closing"] or x["stage"] == "verify_seat_with_lid":
        return True
    pose1 = np.asarray(x["cam"])[2, 3] > 0.93                  # inspection pose 1's camera stands 0.96 m up, pose 4's 0.89
    return bool(x["stage"] in ("verify_lid_closed", "verify_seat_closed") and pose1)


def _one(x: dict) -> dict:
    global _GC
    from alix_studio_lite.server.gap_check import DepthFrame, GapCheck, Intrinsics, load_params

    if _GC is None:
        _GC = GapCheck(params={**load_params(), "gap_decider": _DECIDER, "gap_cross_check": False}, records=None)
    _GC.session._registration.clear()
    depth = np.load(x["path"] + "_depth.npy").astype(np.float32)
    k = x["intr"]
    frame = DepthFrame(depth=depth, intrinsics=Intrinsics(k["fx"], k["fy"], k["cx"], k["cy"], depth.shape[1], depth.shape[0]),
                       cam_to_base=np.asarray(x["cam"], float), hands=x["hands"], source="native depth")
    asked = "seat_closed" if x["stage"] == "verify_seat_with_lid" else x["expect"]
    a = _GC.ask(asked, frame, closing=closing_of(x))
    parts = (a.record or {}).get("parts") or {}
    return dict(stamp=x["stamp"], path=x["path"], day=x["day"], stage=x["stage"], asked=asked, closing=closing_of(x), lab=x["lab"],
                got=a.fixture_state if a.success else "refused", conf=float(a.state_confidence or 0.0),
                lid_deg=parts.get("lid_deg"), seat_deg=parts.get("seat_deg"), message=a.message or "")


def _init(decider: str) -> None:
    global _DECIDER
    _DECIDER = decider
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()


def vote(asked: str, state: str, conf: float) -> str:
    from alix_studio_lite.server import mechanism
    from alix_studio_lite.server.gap_check import Answer

    if state in ("refused", "not_lifted", "", None):
        return "none"
    a = Answer(success=True, fixture_state=state, state_confidence=conf)
    return mechanism.lift_check().frame_vote({"check": "fixture_gap", "expect": asked}, a) or "none"


def sheets(rows: list[dict], out: Path) -> None:
    from PIL import Image, ImageDraw

    out.mkdir(parents=True, exist_ok=True)
    W, H, COLS = 400, 225, 4
    for n in range(0, len(rows), 16):
        page = rows[n:n + 16]
        sheet = Image.new("RGB", (W * COLS, (H + 30) * ((len(page) + COLS - 1) // COLS)))
        for i, r in enumerate(page):
            tile = Image.new("RGB", (W, H + 30))
            tile.paste(Image.open(r["path"] + "_color.png").convert("RGB").resize((W, H)), (0, 30))
            d = ImageDraw.Draw(tile)
            d.text((3, 1), f"{r['stamp']} {r['stage'] or r['asked']}{' (closing)' if r['closing'] else ''}: {r['asked']}?", fill=(255, 255, 0))
            d.text((3, 15), f"lab {r['lab']} ({r['lab_vote']})  now {r['got']} ({r['vote']})  lid {r['lid_deg']} seat {r['seat_deg']}",
                   fill=(0, 255, 255))
            sheet.paste(tile, ((i % COLS) * W, (i // COLS) * (H + 30)))
        sheet.save(out / f"differ_{n // 16:02d}.jpg", quality=80)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--day", action="append", help="a day's folder name (YYYY-MM-DD); default every day")
    ap.add_argument("--decider", choices=("parts", "windows"), default="parts")
    ap.add_argument("--sheets", type=Path, default=None)
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--workers", type=int, default=8)
    args = ap.parse_args()
    _init(args.decider)
    rows = frames(args.day)
    with ProcessPoolExecutor(args.workers, initializer=_init, initargs=(args.decider,)) as pool:
        res = list(pool.map(_one, rows, chunksize=4))
    tab: dict = defaultdict(Counter)
    for r in res:
        r["lab_vote"] = vote(r["asked"], r["lab"], 0.9)
        r["vote"] = vote(r["asked"], r["got"], r["conf"])
        tab[(r["stage"] or r["asked"]) + (" (closing)" if r["closing"] else "")][(r["lab_vote"], r["vote"])] += 1
    same = sum(r["lab_vote"] == r["vote"] for r in res)
    against = sum({r["lab_vote"], r["vote"]} == {"yes", "no"} for r in res)
    print(f"{len(res)} frames, the {args.decider} deciding: the lab's vote {same}, another {len(res) - same} "
          f"({against} the opposite way)")
    for k in sorted(tab):
        print(f"  {k}: " + ", ".join(f"lab {a} / now {b} x{n}" for (a, b), n in sorted(tab[k].items(), key=lambda kv: -kv[1])))
    if args.json:
        args.json.write_text(json.dumps(res))
    if args.sheets:
        sheets([r for r in res if r["lab_vote"] != r["vote"]], args.sheets)
        print(f"contact sheets of the {len(res) - same} frames whose vote differs: {args.sheets}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
