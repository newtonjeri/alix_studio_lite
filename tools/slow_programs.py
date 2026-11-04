#!/usr/bin/env python3
"""Slow the S2 Programs where they ended early on the controller, and the other arm's Programs over the same time
(server/slow_programs.py; the user, 4 Nov). It reads Lite's logs (Resources/Logs) and the project's last S2
simulation, writes the slowed files and a report beside the project, and sends nothing.

    python3 tools/slow_programs.py                        # the project with the newest S2 simulation
    python3 tools/slow_programs.py --project Commode_C_17_10_2026 --factor 1.05 --before 1.0 --after 0.5
    --dry        the report only, no files
    --since DAY  only early ends from that day on (YYYY-MM-DD); by default every one after its file was sent
    --data DIR   Lite's data folder, which holds the files Lite sent (default ~/ALIX-Lite)

Then, for each file the report lists: Advanced Robot Controls, the arm, Upload Only to its slot with Replace, with
the pole and hand the report gives. Then simulate S2 again before a run. The controller shows the same
28-character name as before; Lite tells the files apart by their bytes (the report's sha256).
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--project", default=None)
    ap.add_argument("--logs", type=Path, default=ROOT / "Resources" / "Logs")
    ap.add_argument("--projects", type=Path, default=ROOT / "Resources" / "Projects")
    ap.add_argument("--data", type=Path, default=Path.home() / "ALIX-Lite")
    ap.add_argument("--factor", type=float, default=None)
    ap.add_argument("--before", type=float, default=None)
    ap.add_argument("--after", type=float, default=None)
    ap.add_argument("--ramp", type=float, default=None)
    ap.add_argument("--since", default=None)
    ap.add_argument("--dry", action="store_true")
    args = ap.parse_args()

    from alix_studio_lite import vendor_paths
    vendor_paths.activate()
    from alix_studio_lite.server import slow_programs as sp

    if args.project:
        folder = args.projects / args.project
    else:
        sims = sorted(args.projects.glob("*/sequence/s2_sim.json"), key=lambda p: p.stat().st_mtime)
        if not sims:
            print(f"No S2 simulation in {args.projects}: simulate S2 first.")
            return 2
        folder = sims[-1].parents[1]
    sim_path = folder / "sequence" / "s2_sim.json"
    if not sim_path.exists():
        print(f"{folder.name} has no S2 simulation ({sim_path}): simulate S2 first.")
        return 2
    sim = json.loads(sim_path.read_text())
    since = None
    if args.since:
        since = dt.datetime.strptime(args.since, "%Y-%m-%d").replace(tzinfo=sp.LAB_TZ).timestamp()
    ends = sp.early_ends(args.logs, since=since)
    opts = {k: v for k, v in dict(factor=args.factor, before_s=args.before, after_s=args.after, ramp_s=args.ramp).items()
            if v is not None}
    p = sp.plan(sim, ends, data_dir=args.data, project_dir=folder, **opts)
    print(f"{folder.name}: the S2 simulation of {sim.get('made')}; {len(ends)} early end(s) in the logs, "
          f"{len(p['placed'])} placed in this simulation's Programs.")
    for e in p["placed"]:
        print(f"  {e['end']}: line {e['line']} ({e['file_s']} s into its file, {e['gap_deg']} deg off), "
              f"window S2 {e['window_s'][0]}-{e['window_s'][1]} s")
    for n in p["notes"] + p["others"]:
        print("  note:", n)
    if not p["files"]:
        print("Nothing to slow.")
        return 1
    if args.dry:
        for f in p["files"]:
            a = f["act"]
            print(f"  would slow {a.side.upper()} program {a.slot} ({a.file_name}): +{f['added']} lines")
        return 0
    out = folder / "slowed" / dt.datetime.now(sp.LAB_TZ).strftime("%Y%m%d_%H%M%S")
    rep = sp.write(p, out)
    for r in rep["files"]:
        b, a = r["peak_deg_s_at_4_8ms"]
        print(f"  {r['side'].upper()} slot {r['slot']}: {r['name']} ({r['lines'][0]} -> {r['lines'][1]} lines, "
              f"+{r['added_s']} s; peak {max(b)} -> {max(a)} deg/s at 4.8 ms a line; pole {r['pole_mm']} mm, hand {r['hand']})")
    print(f"Written: {out} (report.md lists what to upload).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
