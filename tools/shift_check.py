#!/usr/bin/env python3
"""Which moves of the commode a plan can follow (server/plan_shift.py): each arm's working stretch moved by IK from the
plan's own joints, every gate, and the collision check with the commode and the room moved with it.

    python3 tools/shift_check.py seat_close --move 20 -20 0 -2     # x y z mm, yaw deg: the report of each moved entry
    python3 tools/shift_check.py lid_open --grid                   # the owner's bounds (30 mm, 3 deg, 15 mm): followed or why not
    --plan PATH        another plan of the task (a re-planned one, before it is vendored)
    --no-collisions    without the collision check (FCL: a second or two a move)

On 31 Oct, ALIX's vendored plans: the seat plans follow most of the bounds (Open the seat not 30 mm toward the robot or
to the left, nor turned -3 deg: its wrists would come within 15 deg of their singularity; Close the seat not 30 mm
away from the robot: its left arm another branch); the lid plans follow a height only, every move sideways refused:
their carries start at the wrist's singularity (J6 near 0), and a 5 mm move flips the wrist to its other branch.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

GRID = [(15, 0, 0, 0), (30, 0, 0, 0), (-15, 0, 0, 0), (-30, 0, 0, 0), (0, 15, 0, 0), (0, 30, 0, 0), (0, -15, 0, 0),
        (0, -30, 0, 0), (0, 0, 0, 1.5), (0, 0, 0, 3), (0, 0, 0, -1.5), (0, 0, 0, -3), (20, 20, 0, 2), (-20, -20, 0, -2),
        (20, -20, 0, -2), (-20, 20, 0, 2), (0, 0, 10, 0), (0, 0, -10, 0)]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("task", choices=("lid_open", "seat_open", "lid_close", "seat_close"))
    ap.add_argument("--move", nargs=4, type=float, metavar=("X_MM", "Y_MM", "Z_MM", "YAW_DEG"))
    ap.add_argument("--grid", action="store_true")
    ap.add_argument("--plan", type=Path, default=None)
    ap.add_argument("--no-collisions", action="store_true")
    args = ap.parse_args()
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.server import mechanism, plan_shift
    from alix_studio_lite.server.jobs import UserError

    plan = mechanism.Plan(args.task, args.plan) if args.plan else mechanism.Plan(args.task)
    moves = [tuple(args.move)] if args.move else GRID if args.grid else []
    if not moves:
        ap.error("give --move or --grid")
    print(f"{plan.title} ({plan.path.name}, made {plan.made})")
    for x, y, z, yaw in moves:
        move = plan_shift.Move(x / 1000.0, y / 1000.0, z / 1000.0, yaw)
        try:
            moved = plan_shift.shift(plan, move)
        except UserError as exc:
            print(f"  {move.words():48s} REFUSED: {str(exc).split(': ', 1)[-1]}")
            continue
        rows = [r for r in moved.shift_report if "hand_off_mm" in r]
        worst = max((r["hand_off_mm"] for r in rows), default=0.0)
        rate = max((r["rate_util"] for r in rows), default=0.0)
        wrist = min((r["wrist_min_deg"][1] for r in rows), default=None)
        text = f"followed: hands within {worst:.2f} mm, peak rate {rate:.0%} of rated, wrist at least {wrist:.1f} deg from J6 0"
        if not args.no_collisions:
            c = plan_shift.collisions(plan, moved)
            text += ("; no new collision" if c["ok"] else
                     "; NEW COLLISIONS: " + "; ".join(f"{e['first_t']:.1f} s {', '.join(e['what'])}" for e in c["new"][:3]))
        print(f"  {move.words():48s} {text}")
        if args.move:
            for r in moved.shift_report:
                print("     ", r)
    return 0


if __name__ == "__main__":
    sys.exit(main())
