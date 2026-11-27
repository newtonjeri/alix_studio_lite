#!/usr/bin/env python3
"""The vendor's own Arm_Tip FK for Lite's tests: tests/data/vendor_arm_tip_fk.json.

rm_algo_forward_kinematics through ALIX's alix_taskgen.stroke_gate.VendorAlgo (RM_75, ISF, the "ArmTip" tool frame and
the "Base" work frame zeroed, exactly as rm_kinematics_plugin sets them), at random joint angles inside each arm's URDF
limits. Lite's tests cannot load the vendor library without SDK-level code, so they check Lite's Arm_Tip (the URDF's
<X>_Link7 moved 2.8 mm back along the tool axis, motion/program_offset.py) against these samples.

Needs an ALIX checkout (the vendor wrapper is not vendored into Lite), as tools/vendor_from_alix.py does:

    /usr/bin/python3 tools/vendor_arm_tip_samples.py [--alix ~/alix_ws/src/alix] [--n 16]
"""
import argparse
import json
import sys
from pathlib import Path

LITE = Path(__file__).resolve().parent.parent
DEFAULT_ALIX = Path.home() / "alix_ws" / "src" / "alix"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--alix", type=Path, default=DEFAULT_ALIX, help="ALIX repo root (default ~/alix_ws/src/alix)")
    ap.add_argument("--out", type=Path, default=LITE / "tests" / "data" / "vendor_arm_tip_fk.json")
    ap.add_argument("--n", type=int, default=16, help="samples an arm")
    a = ap.parse_args(argv)
    taskgen = a.alix / "core" / "python" / "alix_taskgen" / "src"
    if not (taskgen / "alix_taskgen" / "stroke_gate.py").exists():
        raise SystemExit(f"No ALIX checkout with alix_taskgen at {a.alix}: give it with --alix.")
    sys.path.insert(0, str(taskgen))
    from alix_taskgen.stroke_gate import VendorAlgo       # before Lite's paths: Lite vendors a smaller alix_taskgen

    sys.path.insert(0, str(LITE / "src"))
    import numpy as np

    from alix_studio_lite.motion import program_offset as po

    v = VendorAlgo.get()
    if not v.ok:
        raise SystemExit(f"The vendor library did not load: {v.why}")
    rng = np.random.default_rng(20261125)
    out = dict(source="rm_algo_forward_kinematics via ALIX alix_taskgen.stroke_gate.VendorAlgo (RM_75, ISF, ArmTip tool "
                      "and Base work frames zeroed), made by tools/vendor_arm_tip_samples.py (seed 20261125)",
               units="q deg; position m and rotation matrix of Arm_Tip in the arm's Base frame", arms={})
    for side in ("right", "left"):
        arm = po.Arm(side, 0.15)
        rows = []
        for _ in range(a.n):
            q = np.round(rng.uniform(arm.lower_deg + 2.0, arm.upper_deg - 2.0), 3)
            T = v.fk(q)
            rows.append(dict(q_deg=[float(x) for x in q], p_m=[round(float(x), 7) for x in T[:3, 3]],
                             R=[[round(float(x), 7) for x in r] for r in T[:3, :3]]))
        out["arms"][side] = rows
    a.out.write_text(json.dumps(out, indent=1))
    print(f"wrote {a.out}: {a.n} samples an arm")


if __name__ == "__main__":
    main()
