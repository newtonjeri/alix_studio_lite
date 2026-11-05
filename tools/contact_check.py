#!/usr/bin/env python3
"""The hands' contact forces in a lid or seat closing, before the robot runs it (server/contact_sim.py says the model).

    python3 tools/contact_check.py lid_close                  # Lite's plan (vendored), ALIX's room (no wall known)
    python3 tools/contact_check.py lid_close --back-mm 0      # the wall touching the commode's back, as the lab measured it
    python3 tools/contact_check.py seat_close --config ~/alix_ws/src/alix/ros/orchestration/alix_tasks/config/mechanism_tasks/commode_c/seat_close.yaml
                                                              # an edited config, before it is re-planned (contact_preview: the
                                                              # plan's moves moved by IK to the config's new poses and points)
    --band MM      also with the commode MM further off each way along x, y and z (the force moves about 3.5 N a mm)
    --shift X Y Z  the commode off ALIX's place (mm), instead of the lab's of 29 Oct (contact_sim.LAB_SHIFT_MM)
    --json PATH    the whole result (the trace for a chart)

    python3 tools/contact_check.py --store                    # THE OFFLINE STEP (the user, 5 Nov): every closing plan with no
    python3 tools/contact_check.py --store seat_close         # stored check for its file as it is now, checked and stored in
                                                              # Resources/ContactChecks (commit it with the plan); a run reads
                                                              # it and never computes it (contact_sim.gate / require)

Exit 0 when nothing is refused (under 40 N for 0.2 s), 1 when Lite would refuse it in Real.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from alix_studio_lite import vendor_paths  # noqa: E402

vendor_paths.activate()
from alix_studio_lite.server import contact_preview, contact_sim, mechanism  # noqa: E402


def one(task: str, cfg, back_mm, shift) -> dict:
    if cfg is not None:
        return contact_preview.check(task, cfg, back_mm=back_mm, shift_mm=shift)
    return contact_sim.check_plan(task, back_mm=back_mm, shift_mm=shift)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("task", nargs="*",
                    help="lid_close or seat_close (with --store: any number, none = every closing whose plan has no check)")
    ap.add_argument("--store", action="store_true", help="make and keep the run's check of each plan (the offline step)")
    ap.add_argument("--config", type=Path, default=None, help="an edited config of the task (previewed by IK)")
    ap.add_argument("--back-mm", type=float, default=None, help="the gap from the commode's back to the wall (mm)")
    ap.add_argument("--band", type=float, default=0.0, help="also check with the commode this many mm off its place")
    ap.add_argument("--shift", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                    help="the commode off ALIX's place (mm; default the lab's of 29 Oct, contact_sim.LAB_SHIFT_MM)")
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args()
    bad = [t for t in args.task if t not in contact_sim.TASK_PART]
    if bad:
        ap.error(f"not a checked closing: {', '.join(bad)} (one of {', '.join(sorted(contact_sim.TASK_PART))})")
    if args.store:
        todo = args.task or contact_sim.missing()
        if not todo:
            print(f"Every closing plan has its stored check ({contact_sim.store_dir()}).")
            return 0
        for t in todo:
            out = contact_sim.make(t)
            sha = contact_sim._gate_key(t)[1]
            print(f"{t}: {out['level'].upper()}, peak {out['peak_n']:.1f} N -> {contact_sim.stored_path(t, sha)}")
        return 0
    if len(args.task) != 1:
        ap.error("one task to check (or --store)")
    args.task = args.task[0]
    base = tuple(args.shift) if args.shift else contact_sim.LAB_SHIFT_MM
    shifts = [base]
    if args.band:
        b = float(args.band)
        shifts += [tuple(v + d for v, d in zip(base, off)) for off in ((b, 0, 0), (-b, 0, 0), (0, b, 0), (0, -b, 0), (0, 0, b), (0, 0, -b))]
    results = []
    for shift in shifts:
        out = one(args.task, args.config, args.back_mm, shift)
        results.append(out)
        at = (lambda t, s, st=out["stages"]: contact_sim.stage_at(st, t, s))       # noqa: E731
        where = "the commode " + ", ".join(f"{a} {v:+g}" for a, v in zip("xyz", shift)) + " mm off ALIX's place"
        print(f"[{where}] {out['level'].upper()}: " + contact_sim.words(out, mechanism.TASKS[args.task]["title"], at))
        for note in out.get("notes") or []:
            print(f"    preview: {note}")
    if args.json:
        args.json.write_text(json.dumps(results if len(results) > 1 else results[0]))
    worst = max(results, key=lambda o: o["peak_n"])
    if len(results) > 1:
        print(f"Worst within {args.band:g} mm: {worst['peak_n']:.0f} N with the commode {worst['shift_mm']} mm off its place.")
    return 1 if any(o["level"] == "stop" for o in results) else 0


if __name__ == "__main__":
    sys.exit(main())
