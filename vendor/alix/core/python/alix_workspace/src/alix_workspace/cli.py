"""Command-line workspace analysis report (ROS-free).

Runs the analyses — URDF reachability, sensor degradation, and (optionally)
mesh coverage — and prints a consolidated JSON report.

Usage
-----
    workspace_analysis                       # reachability + sensor report
    workspace_analysis --fog 0.9 --lux 200
    workspace_analysis --skip-reachability
    workspace_analysis --waypoints pts.yaml --scene scene.yaml \\
                       --models-dir models/ --region top
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys

from alix_workspace.config_loader import default_config_path, load_config
from alix_workspace.reachability import ReachabilityAnalyzer
from alix_workspace.sensor_model import SensorDegradationModel


def main(argv=None) -> int:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(name)s: %(message)s")
    parser = argparse.ArgumentParser(description="Butterfli workspace analysis report")
    parser.add_argument("--config", default=None, help="path to config YAML")
    parser.add_argument("--fog", type=float, default=0.3, help="fog density 0..1")
    parser.add_argument("--lux", type=float, default=500.0, help="ambient illuminance")
    parser.add_argument("--waypoints", default=None, help="alix_taskgen YAML")
    parser.add_argument("--scene", default=None, help="fixture scene.yaml (for coverage)")
    parser.add_argument("--models-dir", default=None,
                        help="dir with regions.yaml + labeled OBJs (for coverage)")
    parser.add_argument("--region", default=None, help="region key for coverage")
    parser.add_argument("--fixture", default="toilet", help="fixture for reporting")
    parser.add_argument("--skip-reachability", action="store_true",
                        help="skip the (slow) reachability sweep")
    args = parser.parse_args(argv)

    config_path = args.config or default_config_path()
    config = load_config(config_path)
    base_dir = os.path.dirname(os.path.abspath(config_path))
    report: dict = {}

    sensor = SensorDegradationModel(config)
    report["sensor"] = {
        f: a.as_dict() for f, a in sensor.assess_all(args.fog, args.lux).items()
    }

    if not args.skip_reachability:
        analyzer = ReachabilityAnalyzer(config, base_dir=base_dir)
        maps = analyzer.compute_all()
        report["reachability"] = {
            "grid": analyzer.grid_info(),
            "reachable_voxels": {k: int(v.sum()) for k, v in maps.items()},
        }

    if args.waypoints and args.scene and args.models_dir and args.region:
        # Mesh-based coverage, reusing alix_taskgen end-to-end.
        from alix_workspace.coverage import CoverageAnalyzer

        cov = CoverageAnalyzer(config)
        report["coverage"] = cov.analyze(
            args.waypoints, args.scene, args.models_dir, args.region
        )
    elif args.waypoints:
        logging.getLogger("alix_workspace.cli").warning(
            "Coverage needs --waypoints AND --scene AND --models-dir AND --region; skipping"
        )

    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
