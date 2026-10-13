"""Fleet, placement and gate constants — the single source of truth (F73).

Newton, 2026-08-14: *"the source of the standoff distance should be the scene
yaml, and only modified by the task config, but should never be less than
250mm, so we can have a file to define the constants and should be the solo
source of truth for those constants, for the alix studio/taskgen."*

Ownership model:

* **This module owns the numbers.** Every Python consumer imports from here;
  nothing re-declares a value.
* **scene.yaml owns each fixture's standoff** (``default_ugv_standoff_m``) —
  a task config may override it (``task_parameters.ugv_standoff_m``), and
  BOTH are validated against the floor below.
* **The client (editor.js) may not import Python**, so its literals
  (COVERAGE_TARGET_PCT, the standoff input min attributes) are pinned to this
  module by ``test_constants_contract.py`` — the same numeric-contract
  pattern the design tokens use. Change a value HERE and the test names every
  client literal that must move with it.
"""

# ── UGV geometry ────────────────────────────────────────────────────────────
#: Chassis length; base_link sits at its centre (commode_c/scene.yaml's own
#: placement formula: half of this + standoff + fixture front extent).
UGV_LENGTH_M = 0.45
UGV_HALF_LENGTH_M = UGV_LENGTH_M / 2.0

# ── Standoff (UGV front face → fixture front tip) ──────────────────────────
#: The fleet placement standard AND the hard floor. Newton: a fixture is
#: never closer than 250 mm to the UGV front face — a scene.yaml or task
#: config declaring less is a data error (clamped, loudly, at use).
UGV_STANDOFF_MIN_M = 0.25
#: Default when nothing declares one (fleet standard = the floor).
DEFAULT_UGV_STANDOFF_M = 0.25
#: Upper sanity bound (catches mm-instead-of-m slips like 380).
UGV_STANDOFF_MAX_M = 2.0

# ── Frames ─────────────────────────────────────────────────────────────────
#: The frame every pose in a task config is expressed in. A real link —
#: `alix.urdf.xacro:142`, parented to `R_Pole`. All five
#: `task_parameters_defaults.yaml` pin it, so an omitted task key inherits the
#: right value today; the generator emits it anyway so a config is
#: self-describing and does not depend on which directory it is loaded from.
#: `washtub_cleaning/` has no defaults file at all, which is the case that
#: would otherwise have no value at any layer.
DEFAULT_REFERENCE_FRAME = "alix_ref_frame"

# ── Coverage gates (the Studio's own targets) ──────────────────────────────
COVERAGE_PASS_PCT = 95.0     # editor.js COVERAGE_TARGET_PCT — contract-pinned
COVERAGE_WARN_PCT = 85.0     # cov-warn boundary; auto-generate's write floor

# ── Reach gate (G18) ───────────────────────────────────────────────────────
#: auto-generate's SECOND write floor, on the reach number ``_reach_check``
#: already computes. It used to gate the write on coverage alone and reduce
#: reach to a sentence — so `toplid_left` was written at coverage 100.0% and
#: reach 44/91, and then failed on the live stack with "69 of 90 segments
#: unreachable". Covering a surface the arm cannot get to is not coverage.
#:
#: 95.0 is not a new number: auto_generate already used it as the line between
#: "reach <n>/<n>" and the "unreachable points are a placement signal" prose.
#: The change is that it now GATES rather than merely phrases.
REACH_WRITE_PCT = 95.0

# ── Glove contact patch (drives coverage tracing + spacing budgets) ────────
GLOVE_FOOTPRINT_M = (0.035, 0.08)   # (row-step edge, wipe edge)

# ── Sampling ───────────────────────────────────────────────────────────────
COVERAGE_SAMPLES = 6000        # generate/validate coverage estimates
BOARD_COVERAGE_SAMPLES = 2000  # board cells + the auto-generate sweep

# ── Path defaults ──────────────────────────────────────────────────────────
DEFAULT_SPACING_M = 0.03       # suggested_spacing_m fallback
DEFAULT_MAX_POINTS = 250       # decimation cap for a single task

# ── Auto-generate point budget (F73) ───────────────────────────────────────
#: Boustrophedon redundancy = brush width / row step. The 3 cm manual default
#: gives x2.67 ideal (x3-4.7 measured on urinal_a — Newton: too many points);
#: the hand-authored corpus implies ~1.3-2. Coverage traces CONTINUOUSLY
#: between waypoints (coverage.py), so only the ROW step affects coverage —
#: a coarser spacing costs points, not percent.
AUTO_TARGET_REDUNDANCY = 1.6
#: ROW step (the coverage-bearing dimension).
#:
#: CLEANING_MOTION_SPEC §0 "Coverage" sets this by LAW, not by a redundancy
#: target: ``stroke spacing <= effective glove band = footprint WIDTH minus the
#: 1.5 cm dimensional tolerance``, i.e. 0.035 - 0.015 = **0.020 m** for frames
#: 1/2 — the number the contract states outright.
#:
#: This read ``GLOVE_FOOTPRINT_M[1] / 1.6 = 0.05`` and called 0.08 the "brush
#: width". 0.08 is the WIPE edge — the direction the pad travels ALONG. The
#: tuple's own comment above names ``[0]`` the row-step edge, and
#: glove_frames.yaml says the same: "width (W) = edge perpendicular to the wipe
#: direction (brush width)". A stroke lays down the 35 mm width, so the row
#: step was derived from the one footprint dimension that does not bear
#: coverage, making every generated path 2.5x too coarse against the contract.
#:
#: Kept as the contract rule rather than ``[0] / 1.6`` (= 0.022, nearly the
#: same number) so the law is the thing written down and the tolerance is
#: explicit. A region's declared ``spacing_m`` in regions.yaml still outranks
#: this; per-frame spacing comes from contract_motion.stroke_spacing_m().
AUTO_SPACING_M = round(GLOVE_FOOTPRINT_M[0] - 0.015, 3)
#: ALONG-row step: one glove length. Coverage traces continuously between
#: waypoints, so along-row spacing costs points, not percent — the
#: hand-authored corpus median along-path spacing is 12.4 cm.
AUTO_ALONG_STEP_M = GLOVE_FOOTPRINT_M[1]

# ── Paired arm split: the seam band ────────────────────────────────────────
#: How far past the fixture's sagittal plane an `arm_overlap` region hands
#: each arm, so the row landing ON the seam is wiped by both arms and nothing
#: falls between the two halves. That is the documented purpose of the overlap
#: ("to ensure the seam area is cleaned by both arms", alix_taskgen
#: DESCRIPTION.md) and ONE STROKE is what it takes to meet it.
#:
#: It used to be ``extreme_opposite_y * 0.5`` — half of the OPPOSITE half's
#: full extent. That is not a seam band: it gave each arm ~75% of the region
#: width, so the two arms swept the same middle 50% twice (measured on toplid:
#: 125 -> 96 points, arm surface 0.1029 of 0.1337 m²). It cost point count and
#: path length against R1, and it put both arms in the same volume on exactly
#: the RL_paired rows whose cross-arm collisions R4 says are never disabled
#: and must be CHECKED.
#:
#: LEGACY FALLBACK ONLY since P16 S4b (2026-09-23): it is a HALF-width (each
#: arm keeps d <= +band / d >= -band, `arm_filter._arm_threshold`), i.e. a
#: 40 mm intersection, read only by `arm_filter.filter_waypoints_for_arm` when
#: its caller hands no seam (a region with no loader spec). Since the S4b fix
#: pass the legacy plane lane (scripts/generate_cleaning_config.py, and
#: alix_workspace.coverage's reconstruction of it) hands the spec's resolved
#: `seam_intersection_m` / 2 on EVERY paired region, as the C-space cut does
#: (`SEAM_INTERSECTION_*` below, `region_mesh_loader._resolve_seam`).
ARM_SEAM_OVERLAP_M = AUTO_SPACING_M

#: [N-3] RULED (Newton, 2026-09-22 evening, verbatim): "20mm might be too small
#: let us put a minimum 30mm but the advisable intersection of 50mm to 70mm".
#: The requirement is the INTERSECTION W the two arms' shares of a PAIRED region
#: have in common -- each arm's share ends W/2 past the fixture's sagittal
#: plane -- and it is a requirement of EVERY paired task (plan/P16 section 1).
#: The loader refuses W below the minimum and records W outside the advised
#: band as `seamBand` below|above, never silently.
SEAM_INTERSECTION_MIN_M = 0.030
SEAM_INTERSECTION_ADVISED_M = (0.050, 0.070)
#: The default W, inside the advised band.  MEASURED (Resource/reports/
#: 2026-09-22_p16_rulings/REPORT.md section 1.2-1.4; data/seam/seam_widths.json,
#: config_overlap.json) on the 13 paired commode_c task regions at W = 30..70 mm:
#: no reach loss for either arm at any width; at 60 mm the share edge sits 30 mm
#: past the plane, which puts the nearest rows at the plane +- 8 mm (the
#: 22-38 mm lattice inset of the 40 mm generator, measured) and reproduces the
#: archive's hardware-proven 87.5-110 mm pad-swept overlap on the flat tops;
#: 50 mm leaves rows up to 13 mm short of the plane, 70 mm adds +1.5 % share
#: per arm nothing asks for and drops seat_body_hinge_area R's bare-envelope
#: admission of the gained band 0.805 -> 0.760.
SEAM_INTERSECTION_DEFAULT_M = 0.060

# ── Approach / retreat (pre_start_pose) ───────────────────────────────────
#: How far ``pre_start_pose`` stands off ``start_pose`` along the glove's press
#: normal. The generator used to emit pre_start = start_pose exactly AND
#: ``use_prestart_stage: false``, so a task had no approach clearance and never
#: retracted — `forbid_collision` then fired with the hand still on the surface
#: (GENERATION_FINDINGS G24).
#:
#: 0.035 was the hand-authored corpus's value (archived `toplid_left`: start
#: z −0.075, pre_start z −0.040).  NEWTON 2026-09-21 (P15.3c): the pre-start
#: is DERIVED from the start, "a retreat from the normal axis away from the
#: surface and at a higher z axis value, say 5 cm to 10 cm" — the default is
#: the bottom of that band, which is also the archive header's own rule
#: ("start_pose retreated 5 cm along start_pose local +Z axis", hardware-proven).
PRE_START_RETRACT_M = 0.050
#: The world +Z RISE added to the normal retreat.  Newton, 2026-09-21,
#: clarified: "I meant 5 to 10 cm of z rise as well as retreat along the axis
#: from the surface" — BOTH are in the band, so the default rise is its
#: bottom like the retract.  `cspace_author._healthy_pre_start` climbs the
#: band (50, 75, 100 mm) before any rung outside it.  On a surface that faces
#: DOWN the literal +Z would push back into it, so there the rise is projected
#: onto the surface's tangent plane (`pre_start.derive_pre_start_from_normal`)
#: and the retreat alone carries the clearance.  Artefact start only: the entry
#: screen's provisional start has no surface normal and keeps the press rule.
PRE_START_LIFT_M = 0.050
PRE_START_LIFT_MAX_M = 0.100
#: Floor. Not a preference — ``thickness_m`` in glove_frames.yaml is the pad's
#: COMPLIANCE (20 mm on frames 1/2, 18 mm on 3/4) and a surface point within it
#: still counts as wiped, so a shorter retract does not break contact at all.
PRE_START_RETRACT_MIN_M = 0.020
#: Ceiling — the top of Newton's 5-10 cm band (2026-09-21). Past this the
#: approach stops being a standoff and becomes a second transit move, with its
#: own reach and collision exposure near the fixture.
PRE_START_RETRACT_MAX_M = 0.100
