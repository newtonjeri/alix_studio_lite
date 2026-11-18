#!/usr/bin/env python3
"""Time the lid and seat plans' arm moves with Ruckig on s: the copies Lite runs.

    PYTHONPATH=src python3 tools/retime_mechanism.py                    # make the copies Lite runs (Resources/RetimedPlans)
    PYTHONPATH=src python3 tools/retime_mechanism.py --check            # which tasks have no copy of their plan as it is now
    PYTHONPATH=src python3 tools/retime_mechanism.py --plans lid_open   # one task's
    PYTHONPATH=src python3 tools/retime_mechanism.py --out DIR --jerk 300 --ugv-moment 20   # a trial, into another folder

It reads ALIX's plans as Lite has them (vendor/alix/Resource/plans/commode_c/hardware, never changed) and writes a
re-timed copy of each plan and each retry plan, with its strokes' 5 ms tracks and a table of every arm move
(<task>_toppra.ruckig_s.moves.json). The rules are server/mechanism_timing.py's: the path is each move's own
waypoints, its speed class is kept, pole and hand entries are left as they are. It needs the ruckig package.

Without --out the copies go where Lite reads them, made with Lite's own settings (the joint limits file's jerk limit,
the UGV moment limit the UGV's weight gives). Lite's start makes the same copies for a plan that has none, so this
tool is for making them by hand, or on another computer. The closings' contact checks are of the copies: make them
after (tools/contact_check.py --store seat_close lid_close), or let Lite's start do it.

Per move the table gives the time before and after, the readings (speed, acceleration, jerk, UGV load), how far the
hand's path moved (the spline against the straight lines the plan is streamed as), and "x plan speed": the most the
hand is faster than the plan took it at the same point of the path (1.00 = nowhere faster). A Ruckig move is one
smooth rise, cruise and fall per stretch, so its speed along the path is not the plan's shape even when its top speed
and its time are: that column shows by how much.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path


def show(table: list) -> None:
    print(f"{'plan':34s} {'#':>3s} {'move':20s} arm  {'before':>7s} {'after':>7s}  class v/a   speed  acc   jerk    F N   M N m  path mm  x plan speed")
    for r in table:
        if not r["retimed"]:
            print(f"{r['plan'][:34]:34s} {r['stage_index']:3d} {r['stage'][:20]:20s} {r['side'][0].upper()}    {r['seconds_before']:7.2f}    "
                  f"left as it is ({r['waypoints']} waypoints, travels {r['travel_deg']:g} deg)")
            continue
        b, a = r["before"], r["after"]
        print(f"{r['plan'][:34]:34s} {r['stage_index']:3d} {r['stage'][:20]:20s} {r['side'][0].upper()}    {b['seconds']:7.2f} {a['seconds']:7.2f}  "
              f"{r['speed_class']:.2f}/{r['acc_class']:.2f}  {a['speed']:5.2f} {a['acc']:5.2f} {b['jerk']:5.2f}>{a['jerk']:<5.2f} "
              f"{b['force_n']:4.1f}>{a['force_n']:<4.1f} {b['moment_nm']:4.1f}>{a['moment_nm']:<4.1f} {r['path_moved_mm']:5.2f}   {r['fastest_against_plan']:5.2f}")
    done = [r for r in table if r["retimed"]]
    main_rows = [r for r in done if ".rung" not in r["plan"]]
    print(f"\n{len(done)} moves re-timed, {len(table) - len(done)} left as they are (none of them travels). The plans' own arm "
          f"moves: {sum(r['before']['seconds'] for r in main_rows):.1f} s before, {sum(r['after']['seconds'] for r in main_rows):.1f} s after.")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=None, help="another folder for the copies (default: where Lite reads them)")
    ap.add_argument("--plans", nargs="+", default=None, help="tasks to re-time (default: all four)")
    ap.add_argument("--no-rungs", action="store_true", help="leave the retry plans (rungs/) out (a trial only)")
    ap.add_argument("--check", action="store_true", help="only say which tasks have no copy of their plan as it is now")
    ap.add_argument("--jerk", type=float, default=None, help="jerk limit, rad/s^3 (default: the joint limits file's 1500)")
    ap.add_argument("--ugv-force", type=float, default=None, help="the most force on the UGV, N (default: no limit)")
    ap.add_argument("--ugv-moment", type=float, default=None, help="the most moment on the UGV, N m (default: what the UGV's weight holds)")
    args = ap.parse_args()

    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.optimise import ruckig_opt
    from alix_studio_lite.server import app, mechanism, mechanism_timing     # noqa: F401  (app first: the route modules' order)

    tasks = args.plans or list(mechanism.TASKS)
    if args.check:
        gone = mechanism_timing.missing(tasks)
        for task in tasks:
            print(f"{task}: " + (f"NO COPY ({gone[task]})" if task in gone else "its copy is of its plan as it is now"))
        print(f"Lite's timing: {mechanism_timing.timing()}; the copies: {mechanism_timing.store_dir()}")
        return 1 if gone else 0
    if not ruckig_opt.available():
        print(ruckig_opt.UNAVAILABLE_TEXT, file=sys.stderr)
        return 2
    trial = any(v is not None for v in (args.jerk, args.ugv_force, args.ugv_moment)) or args.no_rungs
    if trial and args.out is None:
        print("Other limits, or no retries, make a trial, not the copies Lite runs: give it a folder with --out.", file=sys.stderr)
        return 2
    made_with = mechanism_timing.settings()
    if args.jerk is not None:
        made_with["jerk_rad_s3"] = float(args.jerk)
    if args.ugv_force is not None:
        made_with["ugv_force_n"] = float(args.ugv_force)
    if args.ugv_moment is not None:
        made_with["ugv_moment_nm"] = float(args.ugv_moment)
    store = Path(args.out) if args.out else mechanism_timing.store_dir()
    table = []
    for task in tasks:
        table += mechanism_timing.make(task, store, made_with, rungs=not args.no_rungs)
    show(table)
    print(f"Made with {made_with}. Written to {store}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
