"""CLEANING_MOTION_SPEC laws, in one place.

The cleaning-motion contract (`RM_API2/.../RMDemo_DualArmConcept/
CLEANING_MOTION_SPEC.md`) is the source of truth for HOW a cleaning path
moves — stroke spacing, per-move speed, blend radius, which regions are arcs,
and which glove frame may touch a given surface. Those laws were previously
implicit: the generator planned a generic boustrophedon at a spacing nobody
had tied to the contract, and emitted no per-move speed at all.

This module encodes them so a generated path can be checked against the
contract mechanically instead of by reading. Every constant below cites the
section it comes from; when the contract changes, change it HERE and let the
generator follow.

Nothing in this module plans a path. It answers the questions a planner has
to ask: how far apart may strokes be, how fast may this family run, is this
region an arc, and may this glove frame press this surface.
"""

from __future__ import annotations

import math

from alix_taskgen import glove_frames as _glove_frames

# --------------------------------------------------------------------------
# §0 / §2 — the per-family line-speed ceiling table.
#
# `v_ceil` is the commanded line speed (m/s) putting the worst segment at 90 %
# of the J4 limit, from the exact redundancy-invariant J4 screen at 0.25 m/s.
# The design target is 0.8 m/s but it is a PER-MOVE property: a family may
# carry it only where its own screen passes. Everything else runs at its
# ceiling. The vendor 1.8 m/s is a hard maximum, never a target.
#
# CEILINGS BOUND J4 ONLY. They are upper bounds on J4, not task ceilings —
# a family can be bound by another joint well below its J4 number, which is
# exactly what happened to `top` (see below). Treat these as "no faster
# than", never as "this is safe".
# --------------------------------------------------------------------------
DESIGN_LINE_SPEED = 0.8      # §0 item 1 — per-move design target
VENDOR_MAX_LINE_SPEED = 1.8  # §0 item 1 — hard maximum, never a target

FAMILY_V_CEIL = {
    "lidsides_back":     1.28,
    "bowl_inside_back":  1.10,
    "seat_body_hinge":   1.07,
    "lid_seat_hinge":    0.87,
    "bowl_inside_ring":  0.69,
    "seat_ring_bottom":  0.53,
    "bottomlid":         0.45,
    "lid_side":          0.45,
    "bowl_inside_rim":   0.38,
    # `top` is J1-BOUND, not J4-bound (§2, corrected 2026-08-16). Its
    # J4-derived ceiling of 0.35 does NOT hold: nine REAL runs at a commanded
    # 0.25 put J1 above J4 on every one. Held at 0.25 and must not be raised.
    "top":               0.25,
    "toplid":            0.32,
    # The four over-limit families. OVER at today's 0.25 — they stay at these
    # speeds for SAFETY (§0 item 1), independent of either clock, until their
    # screen-guided redesigns land (§2, §8 step 3).
    "deep":              0.19,
    "side":              0.16,
    "seat_ring":         0.14,
    "front":             0.08,
}

# §2 — illegal at today's 0.25 m/s. ~23.5 m of path, ~36 % of the total.
# These, not toplid, decide the 4-minute cleaning question.
OVER_LIMIT_FAMILIES = frozenset({"front", "seat_ring", "side", "deep"})

# §3 — annular regions authored as 10-20-segment polygons, where every vertex
# is a corner that fights the angular cap and the blend engine. As chained
# movec arcs they have NO corners. Hardware-verified 2026-08-15: 0.37-0.43 mm
# radial error, no stops, junctions ~2x faster than blended corners.
ANNULAR_FAMILIES = frozenset({"seat_ring", "bowl_inside_rim", "bowl_inside_ring"})

# §0 "Coverage" — the 1.5 cm dimensional tolerance USED to derate the footprint
# here.  DELETED 2026-09-07 (Newton R5, DESIGN D1/B1): there is now ONE spacing
# parameter, `stroke_passes`, and the pitch is `pad_swath_m / stroke_passes`.
# Subtracting a fixed 15 mm on top of a passes divisor derates twice and hides
# the one number the caller is meant to choose.  The tolerance itself is not a
# spacing law — it is a dimensional allowance on the FIXTURE, and the coverage
# measurement (`path_layout._swept_mask`) is where a footprint uncertainty
# belongs.  A tombstone, so a re-add is a decision and not a drift.

# Contract A.3 — the contact-fraction gate. Newton moved it 0.5 -> 0.40 on
# 2026-08-19: 4 mm of contact band (32 vs 36 mm of an 80 mm pad) is a small
# price for the manipulability the extra tilt buys, and at g = 0.40 the tilt
# ceiling rises 30.0 -> 38.7 deg on frames 1/2. It is JUDGED, not measured —
# there is no measurement of contact fraction, pressure or wipe quality at any
# f anywhere in the sources, exactly as 0.5 was not.
CONTACT_GATE = 0.40

# §0 "Blend radius" / §7b — hardware-confirmed DETERMINISTIC planner freezes at
# r >= 25 near short segments; dense paths spent up to 37 % of runtime in >1 s
# freezes at r = 25/50, and zero at r = 10.
# ⚠ `r` IS A DIMENSIONLESS PERCENTAGE 0-100, NOT MILLIMETRES (contract A.2;
# vendor: "交融半径百分比系数，0~100"). These constants carried a `# mm` comment
# and the accessor was named `blend_radius_mm`, which is how a reader ends up
# treating 10 as a length. The length is carried by `min(L_in, L_out)` in
# A.2's cut law, not by r.
BLEND_RADIUS_DENSE = 10      # percent — dense geometry, always
BLEND_RADIUS_SPARSE = 35     # percent — only on sparse padded geometry
DENSE_GEOMETRY_SEGMENT_M = 0.05   # segments shorter than this count as dense

# §2b — the working angular cap, adopted after the SIM+REAL ladder on
# top_left (H63 dwell 0 ms at every cap, all peaks <= 86 %). DOUBLE-EDGED:
# throttling protects J4-critical strokes, so raising it requires re-screening
# J4 at the new cap (65 -> 80 % on top_left).
ANGULAR_CAP_RAD_S = 1.0

# §7 — measured efficiency factor (average achieved TCP speed / commanded
# cap): 0.70 at 0.25, 0.72 at the v45 mixed baseline. Used for time estimates.
EFFICIENCY_FACTOR = 0.70

# §1 — the CLEANING budget. Distinct from the 6-minute WHOLE-ACTIVITY figure,
# which additionally covers bin drop/raise and the glove instances and does
# NOT relieve this one (§0, §1 scope note).
CLEANING_BUDGET_S = 240.0
CLEANING_MOTION_BUDGET_S = 180.0   # 240 s less realistic in-cleaning overheads


def is_screened(family: str) -> bool:
    """Has *family* actually been through the J4 screen?

    The distinction matters because :func:`family_v_ceil`'s fallback is a
    SAFETY default, not a measurement, and writing it into a config as though
    it were one is misleading (Newton, 2026-08-19: `lid_seat_front` shipped
    `cleaning_line_speed_mps: 0.08` — the `front` family's screened number,
    silently inherited by a region nobody has screened).
    """
    return str(family) in FAMILY_V_CEIL


def family_v_ceil(family: str) -> float:
    """Line-speed ceiling (m/s) for *family*.

    Unknown families fall back to the slowest over-limit ceiling rather than
    the design target: an unscreened family has NOT earned 0.8, and guessing
    high is the failure mode the contract's item-1 law exists to prevent.

    This is the SAFETY answer for planning-time triage. It is deliberately NOT
    what gets written into a config — see :func:`motion_parameters`, which
    refuses to pass a fallback off as a screened ceiling.
    """
    return FAMILY_V_CEIL.get(str(family), FAMILY_V_CEIL["front"])


def stroke_spacing_m(ik_frame: str, stroke_passes: float = 1.0,
                     wipe_yaw_deg: float = 0.0) -> float:
    """Row pitch (m) for *ik_frame* at *stroke_passes* passes per point.

    ONE SPACING PARAMETER (Newton R5, 2026-09-07).  ``pitch = swath /
    stroke_passes``: at 1.0 the rows abut and every point is wiped once, at 2.0
    they overlap by half and every point is wiped twice.  The five constants
    that used to set spacing (`DIMENSIONAL_TOLERANCE_M` here,
    `LayoutParams.overlap_fraction`, `cspace_author.MAX_ROW_PITCH_M`,
    `constants.AUTO_SPACING_M`, `path_layout.row_pitch_m`) are gone; this is the
    only law, and it is the same one the LATTICE is built at
    (`coverable_map.SurfaceLane.surface`) and the one the rows are laid at
    (`path_layout.LayoutLane._build_direction`), so the two halves of the
    pipeline can no longer disagree.

    ``stroke_passes`` below 1.0 is refused: it would leave bands of the surface
    untouched between rows, which is not a spacing choice but a coverage
    failure, and the coverage number would not say so (the swept mask only
    measures what the rows DO cover).

    The swath is the
    pad's extent ACROSS the direction of travel — the width one stroke lays
    down — at the alignment the pose actually has.

    That alignment is not a naming convention, and it is no longer guessed. The
    pad's long edge L lies on tool Y (measured;
    `alix_workspace.contact.PAD_LONG_AXIS_INDEX`), and
    `path_emission.knot_poses_from` lays the OTHER in-plane axis on the travel
    direction, so at zero wipe yaw the stroke runs along the pad's short edge
    and the swath is L = 80 mm. `wipe_yaw_deg` is the start pose's spin about
    the press axis, which is what fixes the alignment for a whole stroke; pass
    it and the swath follows.

    This read ``footprint[0] - 0.015`` = 0.020 m for frames 1/2, on the prose
    in glove_frames.yaml calling W "the brush width". Under the pose the
    emitter builds, W is the edge along the TRAVEL, not across it — the number
    was ~3x tighter than the geometry, which cost rows and time and could never
    show up as a coverage failure. `path_layout._build_direction` had already
    been using the across-wipe extent, so the two halves of the pipeline were
    laying different rasters from the same pad.
    """
    from alix_workspace.contact import pad_geometry_for_axis, pad_swath_m

    spec = _glove_frames.resolve(ik_frame)
    pad = pad_geometry_for_axis(spec["contact_axis"], spec["footprint"],
                                spec.get("depth_tol", 0.02))
    passes = float(stroke_passes)
    if not (passes >= 1.0):
        raise ValueError(
            f"stroke_passes must be >= 1.0, got {passes!r}: below 1.0 the rows do not "
            "abut, so bands of the surface between them are never touched. That is a "
            "coverage failure, not a spacing choice, and the swept-mask coverage "
            "figure cannot report it (it measures what the rows cover, not the gaps).")
    return max(pad_swath_m(pad, wipe_yaw_deg) / passes, 0.005)


def contact_fraction(theta_deg: float, ik_frame: str, convex_radius_m=None) -> float:
    """Fraction of the pad in contact at misalignment *theta_deg* (§4).

    A flat pad conforms up to its thickness ``t``, so the contact band along
    the tilted axis of dimension ``D`` is ``min(D, t / sin theta)``:

        f = min(1, t / (D . sin theta))

    Tilt is taken about the pad's SHORT axis, so the band runs along the
    80 mm length — the case that actually binds. Tilt about the LONG axis
    never drops f below 0.57 even at 90 degrees, so it cannot fail the
    half-area rule and is not modelled here.

    *convex_radius_m* applies the curvature correction: on a CONVEX surface of
    radius R the flat-pad contact length is ~2*sqrt(2tR), which below
    R ~ 40 mm caps contact more tightly than tilt does (seat-ring outer edges,
    rim lips). Concave surfaces only help, so pass None for them.
    """
    frame = _glove_frames.resolve(ik_frame)
    thickness = float(frame["depth_tol"])
    length = float(frame["footprint"][1])

    theta = abs(math.radians(float(theta_deg)))
    sin_theta = math.sin(theta)
    f_tilt = 1.0 if sin_theta <= 1e-9 else min(1.0, thickness / (length * sin_theta))

    if convex_radius_m is None or convex_radius_m <= 0:
        return f_tilt
    f_curve = min(1.0, 2.0 * math.sqrt(2.0 * thickness * float(convex_radius_m)) / length)
    return min(f_tilt, f_curve)


def max_tilt_deg(ik_frame: str, gate: float = CONTACT_GATE) -> float:
    """Largest misalignment (deg) still meeting the contact gate.

    Contract A.3, GENERAL FORM: `f_tilt = min(1, t/(D sin th)) >= g` inverts to
    `th_max = asin(t / (D * g))`. The old body hard-coded `asin(2t/L)`, which
    is that expression with `g` FROZEN AT 0.5 — the superseded gate. At the
    contract's `g = 0.40` the ceiling is 38.7 deg for frames 1/2 (was 30.0)
    and 35.0 deg at t = 18 mm / 38.7 deg at the standardised t = 20 mm for
    frames 3/4 (was 26.7). Passing `gate` re-derives it for any g.
    """
    frame = _glove_frames.resolve(ik_frame)
    thickness = float(frame["depth_tol"])
    length = float(frame["footprint"][1])
    ratio = min(1.0, thickness / (length * float(gate)))
    return math.degrees(math.asin(ratio))


def frame_is_legal(theta_deg: float, ik_frame: str, convex_radius_m=None,
                   gate: float = CONTACT_GATE) -> bool:
    """Does *ik_frame* meet the contract A.3 contact gate at this tilt?"""
    return contact_fraction(theta_deg, ik_frame, convex_radius_m) >= gate


def blend_radius_pct(segment_length_m: float) -> int:
    """Per-move blend radius `r`, a PERCENTAGE 0-100 (contract A.2), not mm.

    r = 10 on dense geometry; larger radii ONLY on sparse padded geometry.
    The freeze is deterministic, not a tail risk, so this errs dense.
    The resulting cut is `c(r) * (r/100) * min(L_in, L_out)` — see
    `junction_limits.blend_cut`, which is the single implementation.
    """
    return (BLEND_RADIUS_SPARSE if float(segment_length_m) >= DENSE_GEOMETRY_SEGMENT_M
            else BLEND_RADIUS_DENSE)


def is_annular(family: str) -> bool:
    """Should *family* be authored as chained movec arcs rather than a polygon?"""
    return str(family) in ANNULAR_FAMILIES


def estimate_duration_s(path_length_m: float, family: str) -> float:
    """Wall-clock estimate (s) for *path_length_m* of *family* path.

    Uses the §7 method: family ceiling x the measured 0.70 efficiency factor.
    This is the number to compare against CLEANING_MOTION_BUDGET_S, and it is
    an ESTIMATE — §7's own projection lands over budget at today's angular cap
    and closes only with the cap raise plus the slow-family redesigns.

    An UNSCREENED family is estimated at the DESIGN speed, matching what
    :func:`motion_parameters` actually writes into its config — the estimate
    and the commanded speed must not disagree. Such an estimate is provisional
    until the family is screened; treat it as the optimistic bound and screen
    the family before believing a budget built on it.
    """
    v = DESIGN_LINE_SPEED if not is_screened(family) else family_v_ceil(family)
    v_eff = v * EFFICIENCY_FACTOR
    return float(path_length_m) / v_eff if v_eff > 0 else float("inf")


def family_for_region(region: str) -> str:
    """Map a region key onto its §2 speed family.

    Region keys are finer-grained than the screen's families — `seat_ring_sides`
    and `seat_ring_bottom` are both screened under the `seat_ring` pose family,
    `lidsides_back` under its own. Longest-prefix wins so the more specific
    entry (`seat_ring_bottom`, 0.53) beats the general one (`seat_ring`, 0.14)
    when the table actually distinguishes them.

    A region matching nothing returns the region name itself, which
    :func:`family_v_ceil` then resolves to the slowest ceiling — an unscreened
    region does not get to inherit a fast neighbour's speed by accident.
    """
    key = str(region)
    best = ""
    for family in FAMILY_V_CEIL:
        if key.startswith(family) and len(family) > len(best):
            best = family
    return best or key


def motion_parameters(region: str, ik_frame: str,
                      seg_lengths_m=None) -> dict:
    """Contract-derived motion parameters for a region's task_parameters block.

    These carry the §0/§2 speed program into the config so the RealMan dispatch
    layer reads a per-task line speed instead of inheriting one global value —
    the thing §0 item 1 forbids ("0.8 m/s is a per-move property, not a global
    setting; a global 0.8 would abort exactly like the old 0.45 toplid did").

    MoveIt does not consume these — it takes speed from the velocity scaling
    factors — so emitting them changes no ROS-side plan. They are the record of
    what the hardware phase must command, computed from the contract rather
    than re-derived by hand per task.
    """
    from alix_taskgen import junction_limits as _jl   # the ONE implementation
    family = family_for_region(region)                  # of A.1 / A.2 / C2
    screened = is_screened(family)
    # Contract C3: v = min(design target, the family's OWN 90 %-J4 screened
    # ceiling, a blend-entry-safe value). An unscreened family has no own
    # ceiling to min against, so the design value stands and the config SAYS
    # it is unscreened — the previous behaviour silently borrowed the `front`
    # family's 0.08 and presented it as this task's measured speed
    # (Newton, 2026-08-19: "they are misleading").
    v = DESIGN_LINE_SPEED if not screened else min(family_v_ceil(family),
                                                   DESIGN_LINE_SPEED)
    params = {
        "cleaning_family": family,
        "cleaning_line_speed_mps": round(v, 3),
        # C2's coupling is one-directional and by RATIO: a RISE in linear
        # requires the same-magnitude rise in angular (0.8 -> 1.0 m/s, x1.25,
        # carries the cap 1.0 -> 1.25 rad/s). Running SLOWER than the tested
        # 0.8 does not license a lower cap — 1.0 rad/s is the tested envelope
        # and §2b warns that moving this number at all requires re-screening
        # J4 against it. So: scale up above the design speed, never down.
        "cleaning_angular_cap_rad_s": round(_jl.angular_cap(v), 3),
        # C2's acceleration pair, DERIVED not chosen. The config used to carry
        # a speed with no acceleration, so the one number that decides whether
        # a segment can reach that speed at all (A.1) was implicit. Both laws
        # are floors, not ceilings. Reproduces the commanded ladder:
        #   v    0.45 0.50 0.60 0.70 0.80 0.90 1.00
        #   a    1.60 1.60 1.81 2.11 2.41 2.71 3.01   (measured, to rounding)
        "cleaning_line_acc_mps2": round(_jl.line_acc(v), 3),
        # REMOVED 2026-09-01 (Newton, deprecated-key ruling; audit
        # config-template-audit-2026-08-31.md): `cleaning_angular_acc_rad_s2`
        # and `cleaning_min_segment_for_speed_m` had NO reader anywhere — not
        # in the C++ (no parse in config_loader.cpp), not in alix_dispatch, not
        # in taskgen. Both were derived-and-written only. The laws they
        # recorded are unchanged and still live in _jl (angular_acc,
        # min_length_for); what is gone is a config key that read like a
        # commanded value and bound nothing. Its sibling
        # `cleaning_angular_cap_rad_s` above STAYS — that one is live, consumed
        # on the hardware movel chain at alix_dispatch/cli.py:570.
    }
    # ---- G6: what this path's OWN geometry permits, at the speed above ----
    # Contract A.1 / A.2.1 / A.2.2. Before this existed the speed came from a
    # per-FAMILY table and nothing looked at the segments it would be
    # commanded over, so a config could pass every check and still carry the
    # junction that threw `Joint4 overspeed` at 0.90 m/s (A.7). These keys are
    # DERIVED from `cleaning_points` + `cleaning_sequence` in this same file —
    # they add no new policy and no new free parameter, which is the standard
    # the four deleted keys failed (Newton, 2026-08-19).
    if seg_lengths_m:
        prof = _jl.path_profile(seg_lengths_m, cap=v)
        if prof:
            # REMOVED 2026-09-01 (same ruling): `cleaning_reachable_peak_mps`
            # and `cleaning_reachable_effective_mps` were written here and read
            # by nothing. Worse than inert — a config carrying a peak figure
            # reads as SCREENED, and the A.2.1/A.2.2 junction checks that would
            # actually screen it are a separate step. The profile itself is
            # still computed, because the one thing it says that nobody can
            # misread is the flag below.
            if prof["segments_at_cap"] == 0:
                # NOT a warning about the speed being wrong — the cap is a
                # cap. It records that no segment in this task is long enough
                # to reach it, so the number must not be read as a rate.
                params["cleaning_cap_unreachable"] = True
    if not screened:
        # Load-bearing: this task's speed is the DESIGN value, not a
        # measurement. C3 wants the family screened before it is believed.
        params["cleaning_speed_unscreened"] = True
    # REMOVED 2026-09-01 (same ruling), and this pair was the sharpest of the
    # six. `cleaning_speed_below_design` / `cleaning_speed_capped_for_safety`
    # had no consumer, but being BOOLEANS they were swept into the generic
    # extra_flags pool (config_loader.cpp:681-693) — and the only reader of
    # that pool is shouldExecuteStage via a stage's `condition_param`
    # (task_base.cpp:2250-2252). So a stage that mistyped its condition_param
    # into one of these names would have been silently SKIPPED rather than
    # erroring: a dead key that can turn into a missing motion. The C3 ruling
    # they annotated still stands and is unaffected — "a family whose screened
    # ceilings are low across the board is a geometry problem: it goes to C4
    # redesign (de-rotation, frame, standoff, direction), not into the tree as
    # a slow config" — it simply is not recorded as a config key.
    # NOT emitted, deliberately (Newton, 2026-08-19):
    #   cleaning_blend_radius_mm — the NAME is a unit error (r is a percentage,
    #     not mm) and A.2 gives a per-CORNER law: cut = c(r) x (r/100) x
    #     min(L_in, L_out) with c DECLINING in r (1.70 / 1.57 / 1.33 at
    #     r = 10 / 25 / 50) — not the flat 1.3-1.5 band this note used to
    #     quote, which A.2's own correction retires. `blend_radius_pct`
    #     computes r per segment. One task-level scalar cannot express
    #     a per-turn quantity, and 10 was the dense-geometry freeze value from
    #     a corpus C4b has just de-densified. Per-move r belongs in a per-move
    #     slot; the config format has none yet (that is the open C3/C4 work
    #     item), and a wrong scalar is worse than an absent one.
    #   cleaning_stroke_spacing_m — no contract clause sets stroke spacing.
    #     It came from CLEANING_MOTION_SPEC §0's coverage note, which the
    #     contract supersedes; writing it implied a law that does not exist.
    return params
