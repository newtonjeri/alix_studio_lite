"""Per-waypoint analysis of a recorded plan (joint-space trajectory JSON).

Plans (``Resource/plans/<fixture>/<source>/*.json``) are pure joint-space: per stage a
``joint_names`` list and ``waypoints`` with positions/velocities/accelerations.
Before this module nothing evaluated what those joint states actually mean for
the end effector — reachability/manipulability tooling only answered questions
about a precomputed voxel envelope.

``analyze_plan`` walks every waypoint of every stage that moves a configured
chain and reports, at the ACTUAL commanded joint configuration:

* the FK tip position (base_link),
* manipulability + condition number (``IKSolver.manipulability(q)`` — the
  dexterity of the commanded pose, NOT the voxel map's per-cell best case),
* joint-limit margin (rad, min over the chain's joints),
* velocity / acceleration limit violations vs the URDF ``<limit>`` tags,
* optional reachability-envelope membership of the tip position.

Usage::

    from alix_workspace.config_loader import load_config
    from alix_workspace.plan_analysis import analyze_plan
    report = analyze_plan("Resource/plans/commode_c/hardware/toplid_right_totg_ruckig.json",
                          load_config())

or ``python -m alix_workspace.plan_analysis <plan.json> [chain]``.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from alix_workspace.ik import IKSolver
from alix_workspace.urdf_kinematics import UrdfModel

logger = logging.getLogger("alix_workspace.plan_analysis")


def _resolve_urdf(spec: dict, base_dir: Optional[str]) -> str:
    """Resolve a chain's URDF path with the same convention as
    ReachabilityAnalyzer: relative paths are relative to the config file's
    directory (``base_dir``; defaults to the packaged config's dir)."""
    urdf = spec.get("urdf") or spec.get("urdf_path")
    path = Path(urdf)
    if not path.is_absolute():
        if base_dir is None:
            from alix_workspace.config_loader import default_config_path
            base_dir = Path(default_config_path()).parent
        path = Path(base_dir) / path
    return str(path)


def _chain_solvers(config: dict, base_dir: Optional[str], only_chain: Optional[str]):
    """{chain_name: (IKSolver, chain_joint_names)} from the config's chains."""
    chains = (config.get("reachability") or {}).get("chains") or {}
    models: Dict[str, UrdfModel] = {}
    out = {}
    for name, spec in chains.items():
        if only_chain and name != only_chain:
            continue
        urdf = _resolve_urdf(spec, base_dir)
        if urdf not in models:
            models[urdf] = UrdfModel.from_file(urdf)
        solver = IKSolver(models[urdf], spec["base_link"], spec["tip_link"])
        out[name] = solver
    return out


def analyze_plan(plan_path: str, config: dict, base_dir: Optional[str] = None,
                 chain: Optional[str] = None, analyzer=None,
                 manip_warn: float = 0.01) -> dict:
    """Per-waypoint report for every stage that moves a configured chain."""
    data = json.loads(Path(plan_path).read_text())
    all_subs = data.get("sub_trajectories", [])
    solvers = _chain_solvers(config, base_dir, chain)
    if not solvers:
        raise ValueError(f"no matching chains in config (requested {chain!r})")

    stages_out: List[dict] = []
    skipped = 0
    running: Dict[str, float] = {}   # cumulative joint state across stages
    for st in all_subs:
        names = st.get("joint_names") or []
        wps = st.get("waypoints") or []
        if not names or not wps:
            skipped += 1
            continue
        # Which configured chain (if any) does this stage move?
        stage_chain = None
        for cname, solver in solvers.items():
            if set(names) & set(solver.names):
                stage_chain = cname
                break
        if stage_chain is None:
            # A stage of some other group (hand, pole, bin, other arm when
            # filtered) — still update the cumulative joint state.
            for wp in wps:
                for i, n in enumerate(names):
                    pos = wp.get("positions", [])
                    if i < len(pos):
                        running[n] = float(pos[i])
            continue

        solver = solvers[stage_chain]
        limits = {jt.name: jt for jt in solver.movable}
        rows = []
        for wp in wps:
            pos = wp.get("positions", [])
            vel = wp.get("velocities", [])
            acc = wp.get("accelerations", [])
            for i, n in enumerate(names):
                if i < len(pos):
                    running[n] = float(pos[i])
            q = np.array([running.get(n, 0.0) for n in solver.names])
            tip = solver._fk_frames(q)[0][:3, 3]
            manip = float(solver.manipulability(q))
            cond = float(solver.condition_number(q))
            # Joint-limit margin (rad) over this chain's joints.
            margins = [min(running.get(n, 0.0) - solver.lower[i],
                           solver.upper[i] - running.get(n, 0.0))
                       for i, n in enumerate(solver.names)]
            limit_margin = float(min(margins)) if margins else None
            # Velocity / acceleration vs URDF limits, for the joints this
            # stage actually commands.
            vel_violations, acc_violations = [], []
            for i, n in enumerate(names):
                jt = limits.get(n)
                if jt is None or jt.velocity <= 0:
                    continue
                if i < len(vel) and abs(float(vel[i])) > jt.velocity * (1 + 1e-6):
                    vel_violations.append(n)
            row = {
                "timeFromStart": float(wp.get("time_from_start", 0.0)),
                "tipPosition": [round(float(v), 5) for v in tip],
                "manipulability": round(manip, 6),
                "conditionNumber": round(cond, 2),
                "jointLimitMarginRad": round(limit_margin, 4) if limit_margin is not None else None,
                "velocityViolations": vel_violations,
            }
            if analyzer is not None:
                try:
                    row["inEnvelope"] = bool(analyzer.is_reachable(tip, stage_chain))
                except Exception:  # noqa: BLE001 — envelope optional
                    row["inEnvelope"] = None
            rows.append(row)

        manips = [r["manipulability"] for r in rows]
        worst = int(np.argmin(manips)) if manips else None
        stages_out.append({
            "index": st.get("index"),
            "stage": st.get("stage_name"),
            "chain": stage_chain,
            "waypoints": rows,
            "summary": {
                "count": len(rows),
                "minManipulability": min(manips) if manips else None,
                "worstWaypoint": worst,
                "lowManipCount": sum(1 for m in manips if m < manip_warn),
                "minJointLimitMarginRad": min((r["jointLimitMarginRad"] for r in rows
                                               if r["jointLimitMarginRad"] is not None),
                                              default=None),
                "velocityViolationCount": sum(1 for r in rows if r["velocityViolations"]),
                "outOfEnvelopeCount": sum(1 for r in rows if r.get("inEnvelope") is False),
            },
        })

    declared = data.get("num_sub_trajectories")
    manip_mins = [s["summary"]["minManipulability"] for s in stages_out
                  if s["summary"]["minManipulability"] is not None]
    return {
        "plan": str(plan_path),
        "cleaningPathMode": data.get("cleaning_path_mode"),
        "stages": stages_out,
        "summary": {
            "stagesAnalyzed": len(stages_out),
            "stagesSkippedNoJoints": skipped,
            "stagesDeclared": declared,
            "minManipulability": min(manip_mins) if manip_mins else None,
            "velocityViolationCount": sum(s["summary"]["velocityViolationCount"] for s in stages_out),
            "outOfEnvelopeCount": sum(s["summary"]["outOfEnvelopeCount"] for s in stages_out),
        },
    }


def main(argv: Optional[List[str]] = None) -> int:  # pragma: no cover — thin CLI
    import argparse
    from alix_workspace.config_loader import load_config

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("plan", help="Plan JSON (Resource/plans/<fixture>/<source>/...)")
    parser.add_argument("chain", nargs="?", default=None, help="Chain name (default: all)")
    parser.add_argument("--config", default=None, help="alix_workspace config YAML")
    args = parser.parse_args(argv)

    config = load_config(args.config) if args.config else load_config()
    report = analyze_plan(args.plan, config, chain=args.chain)
    s = report["summary"]
    print(f"{Path(args.plan).name}: {s['stagesAnalyzed']} arm stage(s) analyzed "
          f"({s['stagesSkippedNoJoints']} skipped without joints)")
    for st in report["stages"]:
        ss = st["summary"]
        print(f"  [{st['index']}] {st['stage']} ({st['chain']}): "
              f"min manip {ss['minManipulability']:.4f} @wp{ss['worstWaypoint']}, "
              f"min limit margin {ss['minJointLimitMarginRad']} rad, "
              f"{ss['velocityViolationCount']} velocity violation(s)")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
