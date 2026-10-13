"""Contract A.1 / A.2.1 / A.2.2 — what a path's own geometry permits.

Closes gaps-register G6. Until this module existed, `contract_motion` chose a
task's line speed from a per-FAMILY lookup table and nothing anywhere looked at
the segments that speed would actually be commanded over. A config could
satisfy every clause it was checked against and still contain the junction that
threw `Joint4 overspeed` on hardware at 0.90 m/s (contract A.7).

Three laws, all from the contract, all pure arithmetic on the geometry the
config already carries:

  A.1   reachable speed on a segment      v_reach(L)
  A.2.1 arc speed limit                   v_arc <= sqrt(a_lat * R)
  A.2.2 junction speed-step limit         cut >= |v_out^2 - v_in^2| / (2a)

Nothing here plans or edits a path. It answers "what can this geometry hold?"
so the generator can write the truth into the config instead of a family
average. Where a number is a contract constant it is imported, not re-typed.
"""

from __future__ import annotations

import math

from alix_taskgen.contract_motion import DESIGN_LINE_SPEED

# C2: `line_acc = max(A_FLOOR, A_SLOPE * v)`. The floor is the shipped default
# and H62 is the reason it is not raised casually.
A_FLOOR = 1.600      # m/s^2
A_SLOPE = 3.0        # 1/s

# A.1's knee: the length at which the two branches of v_reach meet.
# sqrt(A_FLOOR * L) == A_SLOPE * L  =>  L = A_FLOOR / A_SLOPE^2
KNEE_M = A_FLOOR / (A_SLOPE ** 2)          # 0.1778 m
KNEE_V = A_SLOPE * KNEE_M                  # 0.5333 m/s

# A.2's blend coefficient, re-measured 2026-08-19 over 1454 corners. r is a
# PERCENTAGE 0-100, not millimetres.
BLEND_C = {10: 1.70, 25: 1.57, 50: 1.33}
BLEND_C_DEFAULT = 1.388                    # pooled proportional fit


# C2's angular pair. The coupling is BY RATIO and one-directional: a rise in
# linear requires the same-magnitude rise in angular. The shipped 1.0 rad/s is
# the tested envelope and acts as a floor, so a task slower than the design
# speed keeps 1.0 rather than throttling itself further (H67 is double-edged —
# a lower cap protects J4 on rotating segments but time-scales them).
ANGULAR_FLOOR_RAD_S = 1.0
ANGULAR_ACC_FLOOR = 4.0      # the SHIPPED default = a FLOOR, not a ceiling
ANGULAR_ACC_SLOPE = 3.0      # the vendor ratio: angular_acc >= 3 * omega


def line_acc(v: float) -> float:
    """C2: `line_acc = max(1.6, 3*v)` m/s^2. The default is the FLOOR.

    Reproduces the commanded ladder exactly (`limits_in_force.line_acc`):
    v = 0.45 0.50 0.60 0.70 0.80 0.90 1.00
    a = 1.60 1.60 1.81 2.11 2.41 2.71 3.01
    """
    return max(A_FLOOR, A_SLOPE * v)


def angular_cap(v: float, design: float = DESIGN_LINE_SPEED) -> float:
    """C2's ratio coupling: omega_cap [rad/s] for a commanded line speed."""
    return ANGULAR_FLOOR_RAD_S * max(1.0, v / design)


def angular_acc(omega: float) -> float:
    """C2: `angular_acc = max(4.0, 3*omega)` rad/s^2. A FLOOR, not a ceiling.

    The former 1.3333 rad/s ceiling on omega was the consequence of PINNING
    this at the shipped 4.0; the vendor constraint is the RATIO, which a
    rising acceleration satisfies at any cap.
    """
    return max(ANGULAR_ACC_FLOOR, ANGULAR_ACC_SLOPE * omega)


def v_reach(length_m: float, cap: float = DESIGN_LINE_SPEED) -> float:
    """A.1 — peak speed reachable on a segment of `length_m`, at rest at both
    ends, under `a(v) = max(1.6, 3v)`, then clamped to `cap`.

    Rest-to-rest is the correct model for this corpus, not a pessimistic one:
    at r=10 the measured corner minima on the 2026-08-19 ramp were 0-55 mm/s
    and the harness reported "0 of 19 corners measurable ... retained ~0 % is
    a full stop" (contract C2).
    """
    if length_m <= 0.0:
        return 0.0
    v = A_SLOPE * length_m if length_m >= KNEE_M else math.sqrt(A_FLOOR * length_m)
    return min(v, cap)


def min_length_for(v: float) -> float:
    """A.1 inverted — the shortest segment that reaches `v` rest-to-rest."""
    if v <= 0.0:
        return 0.0
    return v * v / line_acc(v)


def segment_time(length_m: float, cap: float = DESIGN_LINE_SPEED) -> float:
    """Rest-to-rest traversal time, used for the C1 budget."""
    if length_m <= 0.0:
        return 0.0
    v = v_reach(length_m, cap)
    a = line_acc(v)
    ramp = v * v / a
    return 2.0 * v / a + max(0.0, length_m - ramp) / v


def arc_radius(chord_m: float, sagitta_m: float) -> float:
    """A.2.1 — circular radius from a chord and its sagitta."""
    if sagitta_m <= 0.0:
        return float("inf")
    return (chord_m ** 2 / 4.0 + sagitta_m ** 2) / (2.0 * sagitta_m)


def v_arc_max(radius_m: float, a_lat: float) -> float:
    """A.2.1 — `v_arc(max) = sqrt(a_lat * R)`.

    An arc is geometry, not a blended corner, so A.2's cut does not apply to
    it; what applies is lateral acceleration. `a_lat` is the commanded
    `line_acc`.
    """
    if radius_m <= 0.0:
        return 0.0
    if math.isinf(radius_m):
        return float("inf")
    return math.sqrt(a_lat * radius_m)


def blend_c(r_pct: float) -> float:
    """A.2's coefficient `c(r)`. It DECLINES with r — it is not constant.

    INTERPOLATED between the three measured anchors, clamped to the measured
    domain r in [10, 50]. It used to be `BLEND_C.get(int(r_pct), 1.388)`, which
    returned the measured value on an exact hit and the pooled fallback one step
    later — so `blend_cut` FELL as r rose across every anchor (r=25 -> 57.15 mm,
    r=26 -> 52.54 mm). Any search for the smallest feasible r assumes
    monotonicity and was wrong at those three steps. Found by alix-ws-61,
    2026-08-19 (F82).

    This is deliberately the same piecewise-linear form as the emulator's
    `_corner_model` at RM_API2 commit 2fbc9f4, so the two implementations of
    A.2 agree numerically — C17's 1:1 rule applied to a coefficient.
    """
    rp = min(max(float(r_pct), 10.0), 50.0)
    if rp <= 25.0:
        return 1.70 + (rp - 10.0) * (1.57 - 1.70) / 15.0
    return 1.57 + (rp - 25.0) * (1.33 - 1.57) / 25.0


def blend_cut(r_pct: float, len_in_m: float, len_out_m: float) -> float:
    """A.2 — `cut = c(r) * (r/100) * min(L_in, L_out)`, in metres."""
    return blend_c(r_pct) * (r_pct / 100.0) * min(len_in_m, len_out_m)


def step_cut_required(v_in: float, v_out: float, a: float) -> float:
    """A.2.2 — the blend length a junction needs to make a speed step."""
    if a <= 0.0:
        return float("inf")
    return abs(v_out * v_out - v_in * v_in) / (2.0 * a)


def check_junction(v_in, v_out, len_in_m, len_out_m, r_pct=10, a=None):
    """A.2.2 with its FEASIBILITY test. Returns (ok, detail).

    The largest cut a junction can offer is `c(r)*(r/100)*min(L_in,L_out)` with
    r <= 100, so a junction can be infeasible at EVERY r. When it is, the
    contract is explicit: the LOWER SPEED binds — never a larger r, and never
    "run it anyway".
    """
    a = line_acc(max(v_in, v_out)) if a is None else a
    need = step_cut_required(v_in, v_out, a)
    # PHYSICAL CLAMP. A cut is arc-length removed from the polyline, so it can
    # never exceed the shorter adjoining segment — `c(r)*(r/100)*min(L)` alone
    # passes that bound at large r (r=100 on a 145.6 mm approach gives 202 mm).
    # The `feasible` diagnostic below always applied this bound; the ACCEPTANCE
    # test did not, so a junction could pass on a cut that cannot exist.
    # Found by alix-ws-61, 2026-08-19 (F82).
    physical_max = min(len_in_m, len_out_m)
    have = min(blend_cut(r_pct, len_in_m, len_out_m), physical_max)
    best = min(blend_cut(100, len_in_m, len_out_m), physical_max)
    if need <= have:
        return True, {"need_m": need, "have_m": have, "feasible": True}
    return False, {
        "need_m": need,
        "have_m": have,
        "feasible": need <= best,          # `best` is already clamped
        "v_bind": math.sqrt(max(0.0, min(v_in, v_out) ** 2 + 2.0 * a * have)),
    }


def path_profile(seg_lengths_m, cap: float = DESIGN_LINE_SPEED) -> dict:
    """What a whole path's geometry permits at `cap`. Pure summary, no policy.

    `peak_mps` is the fastest any one segment reaches — the number a family
    ceiling is implicitly claiming. `effective_mps` is length-weighted, i.e.
    what the task actually averages, and it is the honest one for C1.
    """
    segs = [s for s in seg_lengths_m if s > 0.0]
    if not segs:
        return {}
    total = sum(segs)
    reach = [v_reach(s, cap) for s in segs]
    return {
        "segments": len(segs),
        "path_length_m": round(total, 4),
        "median_segment_m": round(sorted(segs)[len(segs) // 2], 4),
        "peak_mps": round(max(reach), 3),
        "effective_mps": round(sum(s * v for s, v in zip(segs, reach)) / total, 3),
        "stroke_time_s": round(sum(segment_time(s, cap) for s in segs), 2),
        "segments_at_cap": sum(1 for v in reach if v >= cap - 1e-9),
        "min_length_for_cap_m": round(min_length_for(cap), 4),
    }


# --------------------------------------------------------------------------
# PER-SEGMENT vs GLOBAL motion plans (Newton, 2026-08-20).
#
# "for every task we save data for per-segment and the global cap, and compare,
# and adopt the best per that task" — with per-segment as the DEFAULT.
#
# Per-segment is faster by construction: the global cap is min-over-segments,
# per-segment is each segment at its own max, so on time it always wins and a
# time-only comparison is not a choice at all. What per-segment introduces that
# a global cap cannot have is a SPEED STEP at a junction — and that is the one
# failure we have measured on hardware: on planar_speed_ramp_001 the arc->stroke
# step (25 % -> 100 %) needed 126 mm of blend against ~20 mm delivered, the TCP
# went 278 -> 924 mm/s in 60 ms, and J4 went 18.9 % -> 99.3 % (contract A.2.2).
# So the score carries the junction cost, not just the clock.
# --------------------------------------------------------------------------

def segment_caps(seg_lengths_m, seg_rot_rad, cap=DESIGN_LINE_SPEED,
                 omega_cap=ANGULAR_FLOOR_RAD_S):
    """Per-segment (v, omega, line_acc) — each segment at its own maximum.

    v = min(cap, A.1 reachable from the segment's own length,
                 A.6 curvature limit omega_cap / (dtheta/L)).
    """
    out = []
    for L, dth in zip(seg_lengths_m, seg_rot_rad):
        if L <= 0.0:
            out.append((0.0, omega_cap, A_FLOOR))
            continue
        v_geom = v_reach(L, cap=cap)
        kappa = (dth / L) if L > 0 else 0.0
        v_curv = (omega_cap / kappa) if kappa > 1e-9 else cap
        v = min(cap, v_geom, v_curv)
        out.append((v, angular_cap(v), line_acc(v)))
    return out


def motion_plans(seg_lengths_m, seg_rot_rad, cap=DESIGN_LINE_SPEED,
                 r_pct=10, omega_cap=ANGULAR_FLOOR_RAD_S):
    """Both plans plus the comparison Newton asked every task to carry.

    Returns {"global": {...}, "per_segment": {...}, "adopted": ...}.
    `worst_step_deficit_m` is the A.2.2 shortfall at the worst junction: how
    much more blend the speed step needs than the geometry can deliver. It is
    0 for the global plan by construction, which is the whole point of scoring
    it.
    """
    per = segment_caps(seg_lengths_m, seg_rot_rad, cap=cap, omega_cap=omega_cap)
    v_per = [p[0] for p in per]
    v_glob = min(v_per) if v_per else 0.0

    def _steps(vs):
        """Worst A.2.2 deficit over the junctions between consecutive moves."""
        worst = 0.0
        for i in range(len(vs) - 1):
            a = line_acc(max(vs[i], vs[i + 1]))
            need = step_cut_required(vs[i], vs[i + 1], a)
            have = min(blend_cut(r_pct, seg_lengths_m[i], seg_lengths_m[i + 1]),
                       min(seg_lengths_m[i], seg_lengths_m[i + 1]))
            worst = max(worst, need - have)
        return worst

    g = {"mode": "global", "v_mps": round(v_glob, 4),
         "omega_rad_s": round(angular_cap(v_glob), 4),
         "line_acc_mps2": round(line_acc(v_glob), 4),
         "stroke_time_s": round(sum(segment_time(L, cap=v_glob)
                                    for L in seg_lengths_m), 3),
         "worst_step_deficit_m": 0.0}
    p = {"mode": "per_segment",
         "v_mps": [round(x, 4) for x in v_per],
         "omega_rad_s": [round(x[1], 4) for x in per],
         "line_acc_mps2": [round(x[2], 4) for x in per],
         "stroke_time_s": round(sum(segment_time(L, cap=v)
                                    for L, v in zip(seg_lengths_m, v_per)), 3),
         "worst_step_deficit_m": round(_steps(v_per), 4)}
    # DEFAULT is per_segment (Newton). Global wins only when per-segment buys
    # too little to be worth a junction the geometry cannot absorb.
    gain = (g["stroke_time_s"] - p["stroke_time_s"]) / max(g["stroke_time_s"], 1e-9)
    adopted = "per_segment"
    why = "default; %.0f %% faster" % (100 * gain)
    if p["worst_step_deficit_m"] > 0 and gain < 0.10:
        adopted = "global"
        why = ("per-segment gains only %.0f %% but needs %.0f mm more blend "
               "than any junction can deliver (A.2.2)"
               % (100 * gain, p["worst_step_deficit_m"] * 1000))
    return {"global": g, "per_segment": p, "adopted": adopted, "why": why}


# --------------------------------------------------------------------------
# BLEND vs MOVEC, per turn (Newton, 2026-08-20).
#
# "for every task we can use blended turns or movec turns, or both depending
# on the geometry requirements and reachability."
#
# This adds NO law. Both branches are already decided by clauses in the frozen
# contract, and each has had a module-level function here since G6 closed:
#
#   blend   A.2 + A.2.2   cut = c(r)*(r/100)*min(L_in,L_out), clamped to
#                         min(L_in,L_out); the step needs |dv^2|/(2a)
#   arc     A.2.1         v_arc <= sqrt(a_lat * R), R from chord and sagitta
#
# So the selector is arithmetic on clauses already written: compute what each
# branch PERMITS through the turn and take the higher. What it deliberately
# does NOT do is prefer one primitive a priori. Measured on toplid_right
# 2026-08-20, movec turnarounds came out 2.4 % SLOWER than blended ones
# (17.70 s vs 17.28 s) because the emulator's corner model retains speed
# through a blend, while the same primitive measured ~2x FASTER at ring
# junctions on hardware 2026-08-15. A rule that preferred either would have
# been wrong on half the corpus.
# --------------------------------------------------------------------------

def arc_length(chord_m: float, radius_m: float) -> float:
    """Length along a circular arc of *chord_m* on a circle of *radius_m*."""
    if not math.isfinite(radius_m) or radius_m <= 0.0:
        return float(chord_m)
    ratio = min(1.0, max(-1.0, chord_m / (2.0 * radius_m)))
    return 2.0 * radius_m * math.asin(ratio)


def turn_plan(chord_m, len_in_m, len_out_m, sagitta_m=0.003,
              cap: float = DESIGN_LINE_SPEED, r_pct: float = 10.0) -> dict:
    """What each turn primitive permits over one hop. Returns both, and a pick.

    *chord_m* is the hop being turned through; *len_in_m* / *len_out_m* are the
    strokes either side of it.

    The BLEND branch traverses the hop as a straight move with an A.2 corner at
    each end, so its speed is whatever A.2.2 lets those two junctions carry —
    when a junction is infeasible at *r_pct*, the contract is explicit that the
    LOWER SPEED binds, and `check_junction` returns that bound as `v_bind`.

    The ARC branch traverses the hop as one `rm_movec` on the circle through
    its endpoints and a via at *sagitta_m*. A.2.1 governs it — an arc is
    geometry, not a blended corner, so it takes no A.2 cut at all; what limits
    it is lateral acceleration. It is also LONGER than the chord it replaces,
    which is why speed alone would flatter it and this reports time too.
    """
    v_in = v_reach(len_in_m, cap)
    v_hop = v_reach(chord_m, cap)
    v_out = v_reach(len_out_m, cap)

    ok_entry, d_entry = check_junction(v_in, v_hop, len_in_m, chord_m, r_pct)
    ok_exit, d_exit = check_junction(v_hop, v_out, chord_m, len_out_m, r_pct)
    v_blend = min(v_hop,
                  d_entry.get("v_bind", v_hop) if not ok_entry else v_hop,
                  d_exit.get("v_bind", v_hop) if not ok_exit else v_hop)
    blend = {
        "v_mps": round(v_blend, 4),
        "feasible": bool(ok_entry and ok_exit),
        "entry_deficit_m": round(max(0.0, d_entry["need_m"] - d_entry["have_m"]), 5),
        "exit_deficit_m": round(max(0.0, d_exit["need_m"] - d_exit["have_m"]), 5),
        "path_len_m": round(chord_m, 5),
        "time_s": round(segment_time(chord_m, cap=max(v_blend, 1e-6)), 4),
        "clause": "A.2/A.2.2",
    }

    radius = arc_radius(chord_m, sagitta_m)
    v_arc = min(cap, v_arc_max(radius, line_acc(v_hop)))
    a_len = arc_length(chord_m, radius)
    arc = {
        "v_mps": round(v_arc, 4),
        "radius_m": round(radius, 5),
        "sagitta_m": round(float(sagitta_m), 5),
        "path_len_m": round(a_len, 5),
        "time_s": round(segment_time(a_len, cap=max(v_arc, 1e-6)), 4),
        "clause": "A.2.1",
    }

    # TIME decides, not speed: the arc is the longer path, so the faster
    # primitive is not automatically the quicker one through the turn.
    if arc["time_s"] < blend["time_s"]:
        adopted, why = "arc", (
            "movec %.3f m/s over %.1f mm beats a blended %.3f m/s over %.1f mm "
            "(%.0f ms)" % (arc["v_mps"], a_len * 1000, blend["v_mps"],
                           chord_m * 1000, (blend["time_s"] - arc["time_s"]) * 1000))
    else:
        adopted, why = "blend", (
            "blended %.3f m/s over %.1f mm beats movec %.3f m/s over %.1f mm "
            "(%.0f ms)" % (blend["v_mps"], chord_m * 1000, arc["v_mps"],
                           a_len * 1000, (arc["time_s"] - blend["time_s"]) * 1000))
    if not blend["feasible"] and adopted == "blend":
        why += "; NOTE the blend is infeasible at r=%g and is bound to %.3f m/s" % (
            r_pct, blend["v_mps"])
    return {"chord_m": round(chord_m, 5), "blend": blend, "arc": arc,
            "adopted": adopted, "why": why}


def turn_plans(seg_lengths_m, sagitta_m=0.003, cap: float = DESIGN_LINE_SPEED,
               r_pct: float = 10.0, max_hop_m: float = 0.120) -> list:
    """`turn_plan` over every turnaround of a path. One entry per qualifying hop.

    Qualification matches `arc_geometry.turnaround_arcs` exactly — a short hop
    (< *max_hop_m*) with a longer stroke on both sides — so the two modules
    name the same turns and a plan can be attached to the span the generator
    emits. Any divergence here would silently plan one turn and arc another.
    """
    L = [float(x) for x in seg_lengths_m]
    out = []
    for i in range(1, len(L) - 1):
        if L[i] >= max_hop_m or L[i] <= 1e-6:
            continue
        if not (L[i - 1] > L[i] and L[i + 1] > L[i]):
            continue
        plan = turn_plan(L[i], L[i - 1], L[i + 1], sagitta_m=sagitta_m,
                         cap=cap, r_pct=r_pct)
        plan["edge_index"] = i
        out.append(plan)
    return out


def arc_span_plan(seg_lengths_m, radius_m, cap: float = DESIGN_LINE_SPEED,
                  r_pct: float = 10.0) -> dict:
    """Blend vs movec over a MULTI-SEGMENT span — the ring case.

    `turn_plan` above compares two ways of crossing ONE hop, which is the
    serpentine turnaround. A ring span is a different shape and a different
    sum: `rm_movec` replaces a whole run of polygon chords with one circle, so
    it does not merely soften N corners, it DELETES them. That is what
    CLEANING_MOTION_SPEC §3 means by the biggest wiping-action win, and it is
    why a per-hop comparison understates the ring case badly.

    The asymmetry is A.1, not a preference. A 48 mm chord reaches only
    sqrt(1.6 * 0.048) = 0.28 m/s rest-to-rest, so on a short hop NEITHER
    primitive's ceiling binds and the arc's only effect is to be ~1 % longer —
    exactly the 2.4 % loss measured on toplid_right. Chain twenty such chords
    into one 157 mm quarter-circle and the arc reaches its A.2.1 ceiling while
    every individual chord still cannot, which is where the ~2x hardware
    figure of 2026-08-15 comes from.
    """
    chords = [float(x) for x in seg_lengths_m if x > 0.0]
    if not chords:
        return {}

    # BLEND: every chord is its own rest-to-rest move, and every interior
    # vertex is an A.2 junction that must carry the step between them.
    v_chord = [v_reach(c, cap) for c in chords]
    t_blend = sum(segment_time(c, cap=cap) for c in chords)
    infeasible = 0
    for i in range(len(chords) - 1):
        ok, _ = check_junction(v_chord[i], v_chord[i + 1],
                               chords[i], chords[i + 1], r_pct)
        infeasible += 0 if ok else 1

    # ARC: one circle through the whole span. The chords subtend it, so the
    # traced path is slightly LONGER than their sum — counted, not ignored.
    a_len = sum(arc_length(c, radius_m) for c in chords)
    v_arc = min(cap, v_arc_max(radius_m, line_acc(min(cap, max(v_chord)))))
    t_arc = segment_time(a_len, cap=max(v_arc, 1e-6))

    blend = {"v_mps": round(max(v_chord), 4), "corners": len(chords) - 1,
             "infeasible_corners": infeasible, "path_len_m": round(sum(chords), 5),
             "time_s": round(t_blend, 4), "clause": "A.2/A.2.2"}
    arc = {"v_mps": round(v_arc, 4), "corners": 0,
           "radius_m": round(float(radius_m), 5),
           "path_len_m": round(a_len, 5), "time_s": round(t_arc, 4),
           "clause": "A.2.1"}
    if t_arc < t_blend:
        adopted = "arc"
        why = ("movec holds %.3f m/s over one %.0f mm arc where %d chords hold "
               "only %.3f m/s (%.0f ms, %d corners deleted)"
               % (arc["v_mps"], a_len * 1000, len(chords), blend["v_mps"],
                  (t_blend - t_arc) * 1000, blend["corners"]))
    else:
        adopted = "blend"
        why = ("%d blended chords beat one %.0f mm arc (%.0f ms); the chords are "
               "too short for either ceiling to bind"
               % (len(chords), a_len * 1000, (t_arc - t_blend) * 1000))
    if infeasible and adopted == "blend":
        why += "; NOTE %d of %d corners are infeasible at r=%g" % (
            infeasible, blend["corners"], r_pct)
    return {"segments": len(chords), "blend": blend, "arc": arc,
            "adopted": adopted, "why": why}


# ---------------------------------------------------------------------------
# CORNER TURNS — a junction's speed is an OUTPUT of its radius
# ---------------------------------------------------------------------------
# A.2 gives the cut a blend removes. A.2.1 gives the speed an arc can hold.
# Together they say something neither says alone: a corner blended with cut `d`
# through a deviation `phi` IS a circular fillet, of radius
#
#       R = d / tan(phi/2)
#
# so the speed that corner holds is `sqrt(a_lat * R)`. This is why a "turning
# velocity" cannot be an independent config field. Asking for a turn speed is
# asking for a radius, and a radius is bought with either SURFACE (a fillet
# cuts the corner off the path) or ROOM (the turn leaves the workpiece).
#
# Measured on toplid_right, 2026-08-21: 17 of 23 junctions are 90 degrees
# between a ~197 mm row and a 48.5 mm row hop. At r=10 the cut is 8.2 mm, so
# R = 8.2 mm and the corner holds 0.162 m/s. r=50 buys 0.320 m/s and discards
# 13.2 mm of corner. The U-turn below buys 0.279 m/s and discards NOTHING,
# because it turns in free space past the edge of the lid.

TURN_R_MAX_PCT = 50.0        # A.2's measured domain ends at r=50
TURN_DEVIATION_RAD = math.radians(30.0)   # below this a corner is not a turn

# How much corner a fillet may throw away before the loss stops being covered
# by the pad's own footprint. A.3's pad is 40 mm across, so 5 mm off the vertex
# is still inside the contact patch that passes over it; 13 mm is not.
SURFACE_TOL_M = 0.005


def corner_deviation(u, w) -> float:
    """Deviation angle between two consecutive edge vectors, in radians.

    0 is collinear (no turn at all), pi is a full reversal. This is the
    supplement of the interior angle, and it is the quantity A.2's fillet
    geometry is written in.
    """
    nu = math.sqrt(sum(float(x) * float(x) for x in u))
    nw = math.sqrt(sum(float(x) * float(x) for x in w))
    if nu <= 0.0 or nw <= 0.0:
        return 0.0
    dot = sum(float(a) * float(b) for a, b in zip(u, w)) / (nu * nw)
    return math.acos(min(1.0, max(-1.0, dot)))


def fillet_radius(cut_m: float, phi_rad: float) -> float:
    """Radius of the circular fillet an A.2 cut of *cut_m* makes at *phi_rad*.

    `R = cut / tan(phi/2)`. A near-collinear corner (phi -> 0) needs no fillet
    and returns inf; a reversal (phi -> pi) cannot be filleted at all and
    returns 0.
    """
    half = 0.5 * float(phi_rad)
    if half <= 1e-9:
        return float("inf")
    if half >= 0.5 * math.pi - 1e-9:
        return 0.0
    return float(cut_m) / math.tan(half)


def fillet_cut(radius_m: float, phi_rad: float) -> float:
    """The A.2 cut needed to fillet *phi_rad* at *radius_m*. Inverse of above."""
    half = 0.5 * float(phi_rad)
    if half <= 1e-9:
        return 0.0
    return float(radius_m) * math.tan(min(half, 0.5 * math.pi - 1e-9))


def fillet_offset(radius_m: float, phi_rad: float) -> float:
    """How far the fillet passes from the corner it replaces, in metres.

    This is the SURFACE the blend throws away, and it is the honest cost of
    every "just raise r" answer: `R * (sec(phi/2) - 1)`.
    """
    half = 0.5 * float(phi_rad)
    if half <= 1e-9:
        return 0.0
    c = math.cos(min(half, 0.5 * math.pi - 1e-9))
    if c <= 1e-9:
        return float("inf")
    return float(radius_m) * (1.0 / c - 1.0)


def radius_for_offset(offset_m: float, phi_rad: float) -> float:
    """The largest fillet that stays within *offset_m* of the corner it cuts.

    Inverse of `fillet_offset`. This is the clamp a near-REVERSAL needs and the
    reason a turn policy cannot just take the biggest r it can afford: at
    phi=161 degrees `sec(phi/2)-1` is 4.97, so a 25 mm fillet swings 123 mm
    clear of the vertex. On toplid_right that is the last junction of the
    stroke, and 123 mm off the lid is not a rounded corner, it is a different
    path. A reversal is where the tool must simply slow down.
    """
    half = 0.5 * float(phi_rad)
    if half <= 1e-9:
        return float("inf")
    c = math.cos(min(half, 0.5 * math.pi - 1e-9))
    if c <= 1e-9:
        return 0.0
    return float(offset_m) / (1.0 / c - 1.0)


def r_pct_for_radius(radius_m, phi_rad, len_in_m, len_out_m,
                     r_max: float = TURN_R_MAX_PCT):
    """Smallest r whose A.2 cut fillets *phi_rad* to at least *radius_m*.

    Returns `(r_pct, radius_achieved)`. When even *r_max* falls short the
    achieved radius is returned with `r_max` — the caller then knows the corner
    is radius-bound and must either slow down or leave the surface, which is
    the choice `corner_plan` makes explicit.

    SMALLEST, not largest, for the reason `_segment_policy` already gives: a
    cut is arc-length removed from the path, so an oversized one rounds off
    surface the tool is supposed to clean.
    """
    need = fillet_cut(radius_m, phi_rad)
    best_r, best_R = float(r_max), 0.0
    for step in range(10, int(r_max) + 1):
        have = min(blend_cut(step, len_in_m, len_out_m),
                   min(len_in_m, len_out_m))
        R = fillet_radius(have, phi_rad)
        if R > best_R:
            best_r, best_R = float(step), R
        if have >= need:
            return float(step), R
    return best_r, best_R


def _r_pct_for_cut(cut_m, len_in_m, len_out_m, r_max: float = TURN_R_MAX_PCT):
    """Smallest r whose A.2 cut is at least *cut_m*, clamped to [1, r_max].

    Searches BELOW r=10, which is outside A.2's measured domain — `blend_c`
    clamps there and returns c(10)=1.70. The extrapolation is deliberate and
    it is the unsafe direction: c DECLINES with r, so the true c below 10 is
    probably larger than 1.70, and the real cut therefore larger than modelled.
    A caller that lands here is told so, because the alternative is worse: the
    161-degree junction on toplid_right needs r=2 to stay within 6 mm of its
    vertex, and r=10 would put it 32 mm off.
    """
    for step in range(1, int(r_max) + 1):
        if min(blend_cut(step, len_in_m, len_out_m),
               min(len_in_m, len_out_m)) >= cut_m:
            return float(step)
    return float(r_max)


def turn_speed(radius_m: float, a_lat: float, cap=None) -> float:
    """A.2.1 applied to a corner: `v = sqrt(a_lat * R)`, capped."""
    v = v_arc_max(radius_m, a_lat)
    return float(min(v, cap)) if cap is not None else float(v)


def uturn_plan(pitch_m: float, a_lat: float, cap: float = DESIGN_LINE_SPEED) -> dict:
    """The serpentine row-turnaround done as ONE arc, outside the workpiece.

    A raster turnaround joins the end of one row to the start of the next,
    antiparallel and *pitch_m* apart. There is exactly one circular arc that
    is TANGENT to both rows — the semicircle of radius `pitch/2` — and it
    overhangs the row end by that same `pitch/2`.

    That tangency is the whole point, and it is what the 3 mm-sagitta spans in
    the corpus do NOT have. A 3 mm bulge on a 48.5 mm hop is a 99.5 mm radius,
    which reads fast, but the arc still meets both rows at 90 degrees, so the
    two corners survive untouched and the turn is still bound by their 8 mm
    fillets. The semicircle deletes both corners instead of softening a hop
    that was nearly straight to begin with.

    Radius is FIXED by the row pitch; there is no free parameter. Exceeding
    `v_mps` needs a multi-arc teardrop, and `overtravel_for_speed` prices it.
    """
    R = 0.5 * float(pitch_m)
    return {
        "radius_m": R,
        "overtravel_m": R,          # the semicircle's apex, past the row end
        "path_len_m": math.pi * R,  # vs `pitch_m` straight across
        "v_mps": turn_speed(R, a_lat, cap),
        "surface_lost_m": 0.0,      # it turns in free space, not on the lid
        "tangent": True,
    }


def overtravel_for_speed(v_mps: float, a_lat: float) -> float:
    """Room past the workpiece a turn at *v_mps* needs, in metres.

    The radius that holds *v_mps* is `v^2/a_lat`, and a tangent U-turn overhangs
    by exactly its radius. When this exceeds `pitch/2` the single-arc semicircle
    cannot deliver the speed and a multi-arc teardrop is required — this
    function then reports the FLOOR of what that teardrop would need.
    """
    if a_lat <= 0.0:
        return float("inf")
    return float(v_mps) * float(v_mps) / float(a_lat)


def corner_plan(phi_rad, len_in_m, len_out_m, *, a_lat,
                cap: float = DESIGN_LINE_SPEED,
                radius_target=None, r_max: float = TURN_R_MAX_PCT,
                overtravel_m: float = 0.0, pitch_m=None,
                surface_tol_m: float = SURFACE_TOL_M) -> dict:
    """What one junction can hold, and what each answer costs.

    Returns the fillet branch always, the U-turn branch when *overtravel_m*
    allows one, and `adopted` naming the faster of the two. `v_mps` is the
    speed the TURN holds, not the speed of either leg.

    *radius_target* is the policy ask. It is an ask, not a promise: a corner
    that cannot reach it at *r_max* comes back radius-bound, with `deficit_m`
    saying how much room the ask was short by.
    """
    phi = float(phi_rad)
    if phi < TURN_DEVIATION_RAD:
        # Not a turn. A near-collinear junction holds the cruise speed and the
        # blend exists only to keep the controller from stopping at it.
        return {"phi_deg": round(math.degrees(phi), 2), "is_turn": False,
                "adopted": "cruise", "v_mps": float(cap), "r_pct": 10.0,
                "radius_m": float("inf"), "surface_lost_m": 0.0}

    want = float(radius_target) if radius_target else fillet_radius(
        min(blend_cut(r_max, len_in_m, len_out_m), min(len_in_m, len_out_m)), phi)
    # CLAMP the ask by what the surface will bear. A fillet's offset from the
    # vertex grows as sec(phi/2), so the sharper the corner the less radius a
    # given tolerance buys — and at a reversal it buys essentially none.
    # Without this the policy answers "r=50" to a 161-degree turn and throws
    # the tool 123 mm off the lid to do it.
    ceiling = radius_for_offset(surface_tol_m, phi)
    want = min(want, ceiling)
    r_pct, R_fillet = r_pct_for_radius(want, phi, len_in_m, len_out_m, r_max)
    extrapolated = False
    if R_fillet > ceiling:
        R_fillet = ceiling
        r_pct = max(1.0, min(r_pct, _r_pct_for_cut(
            fillet_cut(ceiling, phi), len_in_m, len_out_m)))
        extrapolated = r_pct < 10.0
    fillet = {
        "r_pct": r_pct,
        "radius_m": R_fillet,
        "v_mps": turn_speed(R_fillet, a_lat, cap),
        "surface_lost_m": fillet_offset(R_fillet, phi),
        "deficit_m": max(0.0, want - R_fillet),
        "offset_bound": R_fillet >= ceiling - 1e-9,
        "extrapolated_r": extrapolated,
        "clause": "A.2 + A.2.1",
    }
    out = {"phi_deg": round(math.degrees(phi), 2), "is_turn": True,
           "fillet": fillet, "adopted": "fillet",
           "r_pct": fillet["r_pct"], "v_mps": fillet["v_mps"],
           "radius_m": fillet["radius_m"],
           "surface_lost_m": fillet["surface_lost_m"]}

    # The U-turn branch only exists when there is room to leave the surface.
    if pitch_m and overtravel_m > 0.0:
        u = uturn_plan(pitch_m, a_lat, cap)
        u["clause"] = "A.2.1"
        u["affordable"] = u["overtravel_m"] <= float(overtravel_m) + 1e-9
        if radius_target and float(radius_target) > u["radius_m"] + 1e-9:
            # More overtravel does NOT buy more radius here: a tangent
            # semicircle's radius is pinned to pitch/2. Exceeding it needs a
            # multi-arc teardrop, which `rm_movec` cannot express in one call.
            u["capped_by"] = "row_pitch"
            u["radius_deficit_m"] = float(radius_target) - u["radius_m"]
        out["uturn"] = u
        # COVERAGE decides the tie, not speed. Both branches turn the corner;
        # only one of them stops cleaning it. A fillet keeps the junction only
        # when it is genuinely faster AND stays inside the pad's own footprint,
        # because a loss smaller than that is still swept by the pad edge.
        if u["affordable"] and (u["v_mps"] >= fillet["v_mps"]
                                or fillet["surface_lost_m"] > surface_tol_m):
            out.update({"adopted": "uturn", "r_pct": 0.0, "v_mps": u["v_mps"],
                        "radius_m": u["radius_m"], "surface_lost_m": 0.0})
        elif not u["affordable"]:
            u["need_overtravel_m"] = u["overtravel_m"]
    return out
