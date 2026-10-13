"""RM75 motion singularities — the four vendor types, as a gate.

Source of truth, quoted verbatim from RealMan's own parameter page:

    https://develop.realman-robotics.com/en/robot/robotParameter/
        RM75OntologyParameters/#motion-singularities

    Type 1   "When q2=0 and q6=0"                 [x,0,x,x,x,0,x]
    Type 2   "When q4=0"                          [x,x,x,0,x,x,x]
    Type 3   "When q2=0 and q3=90 (or q3=-90)"    [x,0,90 or -90,x,x,x,x]
    Type 4   "When q6=0 and q5=90 (or q5=-90)"    [x,x,x,x,90 or -90,0,x]

The vendor lists the conditions and no margins. Margins are ours, and they are
measured rather than chosen — see GATE_DEG.

**Why a scalar distance, and why `max`.** A type is singular only where EVERY
one of its conditions holds, so the pose is safe as soon as ONE of them is far
away. The distance to the singular set is therefore the LARGEST of the
per-condition distances, not the smallest and not their sum. Getting this
backwards would flag half the workspace: q2=0 alone is not a singularity, it is
one of two coordinates of type 1 and one of two of type 3.

Nothing here solves IK or edits a path. It answers "how close is this
configuration to a place the arm cannot move through", so the generator can
refuse a waypoint before it reaches hardware.
"""

from __future__ import annotations

VENDOR_URL = ("https://develop.realman-robotics.com/en/robot/robotParameter/"
              "RM75OntologyParameters/#motion-singularities")

#: Detection band — "this pose is NEAR a singularity", for reporting. Matches
#: `verify_task_motion.SINGULARITY_NEAR_DEG`, which is what the gate reports
#: have always used.
NEAR_DEG = 10.0

#: GENERATION gate. Wider than the detection band because a generator can
#: still choose, where a screen can only report.
#:
#: MEASURED, not picked. On toplid_right, 2026-08-21:
#:
#:     config                        min margin   outcome
#:     regenerated 5x3 grid              0.0 deg  SIM refused at 0.8, 0.32, 0.2
#:     SNAPSHOT-2026-08-20              22.5 deg  ran on hardware at 0.6-0.8
#:     PRE-GROUPFRAME backup            32.3 deg  ran
#:
#: 20 sits under the smallest margin that has ever completed a stroke and far
#: above the one that never did. Raise it when a wider band earns its keep;
#: do not lower it without a run that says so.
GATE_DEG = 20.0
#: THE GENERATION GATE AS RULED 2026-09-22 (Newton: "find the vendor's rule, but
#: we can loosen it with the task's requirements: start from the most
#: conservative 15 deg, then narrow per task, never below 5 deg").  The 20 above
#: was measured on SIM refusals of CARTESIAN moves (August: rm_ik / movel, which
#: the controller singularity-checks); the executive now streams JOINT points,
#: which the controller does not check -- MEASURED on the 200 Hz frames of the
#: complete hardware runs (tools/calibrate_oracle_on_hardware.py): the arm ran at
#: 11.0 deg (bowl_inside_front R), 12.1-12.7 (hinge_area), 13.8 (rim L) and 0-2
#: (seat_ring L/R) under this metric with no refusal.  What the gate protects now
#: is OUR continuation solver's conditioning (seat_ring L: "IK failed at s=0.994"
#: is the DLS solve at a near-singular pose) and the joint rates near a singular
#: set.  So: 15 by default (`GATE_DEFAULT_DEG`), a region may declare
#: `singular_margin_deg` in its manifest (the corrected copy carries the
#: evidence), and nothing goes under `GATE_FLOOR_DEG`.
GATE_DEFAULT_DEG = 15.0
GATE_FLOOR_DEG = 5.0
#: ...and a region may also declare MORE than the default, up to the August
#: measurement: MEASURED 2026-09-22, `front` right at 15 -- the map admits a
#: corridor whose start sits at 15.8 deg of margin and the DLS continuation
#: cannot converge 14 mm into it ("IK failed at s=0.0140 m", every ramp slope);
#: at 20 the same region authors.  The gate protects the solver's conditioning,
#: and this solver needs 20 there.
GATE_MAX_DEG = 20.0

#: q4 = 0 is the STRAIGHT ELBOW, and it is the one that bites this corpus,
#: because these paths live near full extension. Type 2 is the only type whose
#: singular set is a single condition, so it is also the easiest to fall into.
TYPES = ("type1", "type2", "type3", "type4")

LABELS = {
    "type1": "type1 (q2=0, q6=0)",
    "type2": "type2 STRAIGHT ELBOW (q4=0)",
    "type3": "type3 (q2=0, q3=+-90)",
    "type4": "type4 (q6=0, q5=+-90)",
}


def _at(value: float, target: float) -> float:
    """Angular distance from *value* to +-*target*, in degrees."""
    return abs(abs(float(value)) - float(target))


def distances(joints) -> dict:
    """`{type: degrees}` — how far this pose is from each singular set.

    *joints* is q1..q7 in DEGREES, the order the controller and
    `verify_task_motion` both use. Larger is safer; 0 means exactly singular.
    """
    q = [float(x) for x in joints]
    if len(q) < 7:
        raise ValueError(f"need 7 joint angles in degrees, got {len(q)}")
    q2, q3, q4, q5, q6 = q[1], q[2], q[3], q[4], q[5]
    return {
        # BOTH conditions must hold, so one far coordinate is enough: max.
        "type1": max(_at(q2, 0.0), _at(q6, 0.0)),
        "type2": _at(q4, 0.0),
        "type3": max(_at(q2, 0.0), _at(q3, 90.0)),
        "type4": max(_at(q6, 0.0), _at(q5, 90.0)),
    }


def distances_many(joints_deg) -> dict:
    """`distances` over ANY leading shape: *joints_deg* ``(..., 7)`` in DEGREES ->
    ``{type: array (...)}``.  The same four formulas, elementwise -- the stroke
    gate's PATH SINGULARITY FLOOR (Newton 2026-09-29: never below `GATE_FLOOR_DEG`
    at any gate sample of the stroke, `stroke_gate.singular_floor_along`) asks
    this of every 2 mm sample, where the scalar `distances` would be a Python loop
    over hundreds of poses; test_p17_rulings_floors pins the two equal."""
    import numpy as np

    q = np.asarray(joints_deg, dtype=float)
    if q.shape[-1] < 7:
        raise ValueError(f"need 7 joint angles in degrees on the last axis, got shape {q.shape}")
    q2, q3, q4, q5, q6 = q[..., 1], q[..., 2], q[..., 3], q[..., 4], q[..., 5]

    def at(v, target):
        return np.abs(np.abs(v) - float(target))

    return {"type1": np.maximum(at(q2, 0.0), at(q6, 0.0)),
            "type2": at(q4, 0.0),
            "type3": np.maximum(at(q2, 0.0), at(q3, 90.0)),
            "type4": np.maximum(at(q6, 0.0), at(q5, 90.0))}


def margin_deg(joints) -> float:
    """The distance to the NEAREST singular set — one number for a pose.

    This is the min across types, which is the opposite of the max WITHIN a
    type and for the opposite reason: a pose is unsafe if it is near ANY type.
    """
    return min(distances(joints).values())


def worst(joints):
    """`(type, degrees, label)` for the nearest singularity."""
    d = distances(joints)
    kind = min(d, key=lambda k: d[k])
    return kind, d[kind], LABELS[kind]


def hits(joints, near_deg: float = NEAR_DEG) -> list:
    """Labels for every type within *near_deg*. Reporting shape, not a gate.

    Type 2 carries its measured q4 in the text because that is the number an
    operator acts on; the others are conjunctions and a single number would be
    misleading.
    """
    d = distances(joints)
    q4 = float(joints[3])
    out = []
    for kind in TYPES:
        if d[kind] <= near_deg:
            out.append(f"type2 STRAIGHT ELBOW (q4={q4:.1f})"
                       if kind == "type2" else LABELS[kind])
    return out


def clear(joints, gate_deg: float = GATE_DEG) -> bool:
    """Whether a pose has enough room from every singular set to be generated."""
    return margin_deg(joints) >= float(gate_deg)


def screen(poses, gate_deg: float = GATE_DEG, near_deg: float = NEAR_DEG) -> dict:
    """Screen a whole path. *poses* is `[(name, joints_deg), ...]`.

    Returns `{ok, gate_deg, min_margin_deg, worst, blocked, near, per_point}` —
    `blocked` are the waypoints under the gate, `near` those merely inside the
    reporting band. A caller that only checks `ok` still gets the right answer;
    one that wants to explain itself has the names.
    """
    per, blocked, near = [], [], []
    for name, joints in poses:
        kind, deg, label = worst(joints)
        row = {"name": name, "margin_deg": round(deg, 2), "type": kind,
               "label": label, "q4": round(float(joints[3]), 2)}
        per.append(row)
        if deg < float(gate_deg):
            blocked.append(row)
        elif deg <= float(near_deg):
            near.append(row)
    worst_row = min(per, key=lambda r: r["margin_deg"]) if per else None
    return {
        "ok": not blocked,
        "gate_deg": float(gate_deg),
        "min_margin_deg": worst_row["margin_deg"] if worst_row else None,
        "worst": worst_row,
        "blocked": blocked,
        "near": near,
        "per_point": per,
        "vendor": VENDOR_URL,
    }


def describe(report: dict) -> str:
    """One-paragraph verdict for a `screen` result, for logs and CLI output."""
    if report.get("ok"):
        return ("singularity screen PASSED: closest approach %.1f deg (%s) "
                "against a %.0f deg gate"
                % (report["min_margin_deg"], report["worst"]["label"],
                   report["gate_deg"]))
    rows = report["blocked"]
    names = ", ".join(r["name"] for r in rows[:6])
    if len(rows) > 6:
        names += f", +{len(rows) - 6} more"
    return ("singularity screen FAILED: %d waypoint(s) inside the %.0f deg "
            "gate, closest %.1f deg (%s) at %s. The arm can hold these poses "
            "but cannot move THROUGH them under Cartesian control — the "
            "Jacobian loses rank and the controller refuses the chain. %s"
            % (len(rows), report["gate_deg"], report["min_margin_deg"],
               report["worst"]["label"], names, VENDOR_URL))
