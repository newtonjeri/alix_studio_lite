"""Kinematic screens a wiping path needs, and the package did not have.

Audited 2026-08-22 against what a motion-geometry designer actually asks. Four
answers were missing here, and the workarounds in use were not equivalent:

* **The singularity law.** `grep -rn "singular"` over this package returns three
  docstring mentions and no implementation. The four RM75 vendor singular sets
  live in `alix_taskgen.singularity`, and nothing here imported them. This
  module does not re-derive them — there is ONE implementation and a second one
  would drift from it — it re-exports and applies them.

* **Why the dexterity index cannot stand in for that law.** `IKSolver.manipulability`
  defaults to `kind="translation"`, a 3x7 Jacobian which stays comfortably above
  zero at a vendor-singular pose: the arm can still translate its tool there, it
  just cannot be *steered through* it, and the controller refuses the chain
  (ret=1) rather than tracking it. `kind="full"` does collapse, but nothing in
  the package passes it. `dexterity_and_margin` reports both, side by side, so
  the difference is visible instead of assumed.

* **Joint-velocity headroom.** Nothing mapped a Cartesian speed to joint rates
  and compared them against the URDF's own `velocity` limits, so "can the arm
  actually run this stroke at 0.8 m/s" had no offline answer.

* **A good wrist yaw about the press normal.** Contact fixes one tool axis to
  the surface; the rotation about it is free, and no API here could sweep it.
  That single angle is worth 15-27 degrees of singularity margin on the shipped
  commode_c corpus (measured, 15 regions x 24 yaws), and today nothing chooses
  it — `pose_generator` folds the twist to whatever the START POSE happened to
  carry.

Everything here is a screen, not a planner: it answers "is this pose or this
speed safe for this arm", and leaves the geometry to `alix_taskgen`.
"""

from __future__ import annotations

import math
from typing import Callable, Iterable, Optional, Sequence

import numpy as np

try:                                     # ONE implementation of the vendor law
    from alix_taskgen.singularity import (GATE_DEG, LABELS, NEAR_DEG,  # noqa: F401
                                          VENDOR_URL, distances, margin_deg, worst)
    HAVE_SINGULARITY_LAW = True
except Exception:                        # pragma: no cover - taskgen not installed
    HAVE_SINGULARITY_LAW = False
    GATE_DEG = 20.0
    VENDOR_URL = ("https://develop.realman-robotics.com/en/robot/robotParameter/"
                  "RM75OntologyParameters/#motion-singularities")


def require_singularity_law():
    """Fail loudly rather than screen against nothing.

    An absent law must not read as a pass. Every earlier version of this
    mistake cost a hardware run: `sdk_reach_check` reported `pct = 100` for a
    path with five waypoints sitting on q4 = 0, because reachability is a
    POSITION question and the singularity is a STEERABILITY one.
    """
    if not HAVE_SINGULARITY_LAW:
        raise RuntimeError(
            "alix_taskgen.singularity is not importable, so no singularity "
            f"screen can be run. The law it implements is at {VENDOR_URL}. "
            "Refusing to report a pass without it.")


# ── dexterity, next to the law it cannot replace ─────────────────────────────
def dexterity_and_margin(solver, q) -> dict:
    """Both dexterity indices and the vendor margin for one configuration.

    Returned together on purpose. `translation` is what the package computes by
    default and is the one that will mislead: it measures whether the tool can
    be moved, not whether the arm can be driven through the pose.
    """
    require_singularity_law()
    q = np.asarray(q, dtype=float)
    kind_margin = worst(np.degrees(q) if _looks_like_radians(q) else q)
    return {
        "manipulability_translation": float(solver.manipulability(q, kind="translation")),
        "manipulability_full": float(solver.manipulability(q, kind="full")),
        "condition_number_full": float(solver.condition_number(q, kind="full")),
        "singularity_kind": kind_margin[0],
        "singularity_margin_deg": float(kind_margin[1]),
        "singularity_ok": float(kind_margin[1]) >= GATE_DEG,
    }


def _looks_like_radians(q) -> bool:
    """RM75 joint limits run to 365 deg, so a vector inside +-2pi is ambiguous.

    Treated as radians only when EVERY element is inside pi, which no degree-
    valued RM75 pose satisfies unless it is within 3 degrees of the home pose.
    """
    return bool(np.all(np.abs(np.asarray(q, dtype=float)) <= math.pi))


# ── joint-velocity headroom ──────────────────────────────────────────────────
def joint_rates_for_twist(solver, q, twist) -> np.ndarray:
    """Joint rates the least-norm solution needs for a Cartesian twist.

    `twist` is [vx, vy, vz, wx, wy, wz] in the Jacobian's frame. The pseudo-
    inverse is the redundancy resolution the offline solver uses; a real
    controller may pick another branch, so read this as a LOWER bound on the
    rates the move demands, not a prediction of what the arm will command.
    """
    J = np.asarray(solver.jacobian(q), dtype=float)
    return np.linalg.pinv(J) @ np.asarray(twist, dtype=float)


def speed_headroom(solver, q, direction, joint_limits_deg_s,
                   angular_rate_rad_s: float = 0.0) -> dict:
    """The largest Cartesian speed along `direction` no joint limit forbids.

    Rates scale linearly with speed, so the binding joint and the ceiling both
    fall straight out of a unit-speed solve. Returns the ceiling, which joint
    sets it, and the fraction of that joint's limit a 0.80 m/s stroke would
    ask for — 0.80 being the cap the arm is run against.
    """
    d = np.asarray(direction, dtype=float)
    n = float(np.linalg.norm(d))
    if n < 1e-12:
        raise ValueError("direction must be a non-zero vector")
    twist = np.concatenate([d / n, np.zeros(3)])
    rates = np.degrees(joint_rates_for_twist(solver, q, twist))      # deg/s per 1 m/s

    limits = np.asarray(joint_limits_deg_s, dtype=float)
    if limits.shape != rates.shape:
        raise ValueError(f"expected {rates.shape[0]} joint limits, got {limits.shape[0]}")
    with np.errstate(divide="ignore", invalid="ignore"):
        ceilings = np.where(np.abs(rates) > 1e-9, limits / np.abs(rates), np.inf)
    worst_joint = int(np.argmin(ceilings))
    ceiling = float(ceilings[worst_joint])

    return {
        "max_speed_mps": ceiling,
        "binding_joint": worst_joint + 1,                 # 1-based, as the SDK numbers them
        "rates_deg_s_per_mps": rates.tolist(),
        "fraction_of_limit_at_0p8": float(min(1e9, 0.8 / ceiling)) if ceiling else float("inf"),
        "angular_rate_rad_s": float(angular_rate_rad_s),
    }


# ── the free angle nobody chooses ────────────────────────────────────────────
def yaw_scan(solve_at_yaw: Callable[[float], Optional[Sequence[float]]],
             yaws_deg: Iterable[float] = tuple(range(0, 360, 15)),
             gate_deg: Optional[float] = None) -> dict:
    """Sweep the rotation about the press normal and report where it is safe.

    Contact is indifferent to this angle — it turns the pad in the plane it is
    already pressed flat against — so it is free to spend on kinematics. The
    caller supplies `solve_at_yaw(deg) -> joints or None`, which keeps the IK
    backend (this package's `IKSolver`, or the vendor SDK oracle) out of here.

    Reports contiguous feasible WINDOWS, not just the best angle: a 15-degree
    island is a number that will stop working the first time the fixture moves,
    and a 90-degree window is one that will not. Measured on commode_c, every
    one of the 15 regions had a window, and 14 of 15 improved on the margin the
    shipped config achieves.
    """
    require_singularity_law()
    gate = GATE_DEG if gate_deg is None else float(gate_deg)
    yaws = [float(y) for y in yaws_deg]
    if len(yaws) < 2:
        raise ValueError("a scan needs at least two yaws")
    step = min(abs(b - a) for a, b in zip(yaws, yaws[1:])) or 15.0

    rows = []
    for yaw in yaws:
        joints = solve_at_yaw(yaw)
        if joints is None:
            rows.append({"yaw_deg": yaw, "solved": False, "margin_deg": None, "ok": False})
            continue
        kind, deg, _ = worst(joints)
        rows.append({"yaw_deg": yaw, "solved": True, "margin_deg": float(deg),
                     "kind": kind, "ok": float(deg) >= gate})

    feasible = [r["yaw_deg"] for r in rows if r["ok"]]
    best = max((r for r in rows if r["ok"]), key=lambda r: r["margin_deg"], default=None)
    return {"scan": rows, "gate_deg": gate, "best": best,
            "windows": contiguous_windows(feasible, step),
            "n_feasible": len(feasible), "n_scanned": len(rows)}


def contiguous_windows(yaws_deg, step_deg) -> list:
    """Feasible arcs, joined across the 360 -> 0 wrap.

    Wrap matters: a window running 330 -> 30 is one 60-degree arc, and reported
    as two it looks like two narrow islands and gets rejected.
    """
    vals = sorted(float(y) % 360.0 for y in yaws_deg)
    if not vals:
        return []
    runs, current = [], [vals[0]]
    for y in vals[1:]:
        if y - current[-1] <= step_deg + 1e-6:
            current.append(y)
        else:
            runs.append(current)
            current = [y]
    runs.append(current)
    if len(runs) > 1 and runs[0][0] == 0.0 and runs[-1][-1] + step_deg >= 360.0:
        runs[0] = runs[-1] + runs[0]
        runs.pop()
    return [{"from_deg": r[0], "to_deg": r[-1], "width_deg": len(r) * step_deg}
            for r in runs]


# ── path-level screen ────────────────────────────────────────────────────────
def screen_path(joint_path, gate_deg: Optional[float] = None) -> dict:
    """Worst singularity margin over a whole path, naming where it is worst.

    A screen reporting only a count is why the same fault reached hardware
    three times: it has to name the waypoint.
    """
    require_singularity_law()
    gate = GATE_DEG if gate_deg is None else float(gate_deg)
    rows = []
    for i, entry in enumerate(joint_path):
        # `(name, joints)` or bare joints. Keyed on the NAME being a string:
        # `np.isscalar("point3")` is True, so a scalar test silently treats the
        # pair as a 2-element joint vector and hands the name to float().
        if isinstance(entry, (tuple, list)) and len(entry) == 2 and isinstance(entry[0], str):
            name, joints = entry
        else:
            name, joints = f"#{i}", entry
        kind, deg, _ = worst(joints)
        rows.append({"name": str(name), "margin_deg": float(deg), "kind": kind,
                     "ok": float(deg) >= gate})
    blocked = [r for r in rows if not r["ok"]]
    return {"ok": not blocked, "gate_deg": gate, "points": rows, "blocked": blocked,
            "min_margin_deg": min((r["margin_deg"] for r in rows), default=None),
            "vendor_url": VENDOR_URL}
