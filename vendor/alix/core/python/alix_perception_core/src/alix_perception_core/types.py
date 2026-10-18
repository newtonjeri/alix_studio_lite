"""The vocabulary the core and the node agree on.

These are plain dataclasses on purpose. `AnalyzeScene.srv` is the ROS contract,
and the node translates to it — the core never imports a ROS message type, so
every one of these is constructible in a test without a graph
(LAYERS.md §1, and §6: "what the layering buys, in tests").
"""

from __future__ import annotations

from dataclasses import dataclass, field

# ── Fixture taxonomy ────────────────────────────────────────────────
# Matches AnalyzeScene.srv's documented values and the fixture directories under
# alix_tasks/config. `regions.py` checks emitted regions against the fixture
# model's own regions.yaml rather than against a second list here.

FIXTURE_TYPES = ("commode", "urinal", "washtab")

FIXTURE_STATES = {
    "commode": ("closed", "lid_open", "lid_seat_open"),
    "urinal": ("default",),
    "washtab": ("default",),
}

# ── What a model class MEANS ────────────────────────────────────────
# butterfli derived this by substring-matching class names against five
# hard-coded pattern lists (`_STATUS_CLASS_PATTERNS`, `_FIXTURE_TYPE_PATTERNS`,
# `_STAIN_CLASS_PATTERNS`, `_PERMANENT_STAIN_CLASS_PATTERNS`,
# `_ESCALATE_CLASS_PATTERNS`, ai_client.py:600-640), two of which carried the
# comment "Placeholder — update with actual class names from the target model".
# A retrained model with different labels therefore needed a code change, and a
# class the patterns did not match was silently filed as a cleanable stain.
#
# Here a model DECLARES the kind of each class in `models.yaml`. Same five
# meanings, same downstream behaviour, but a new model is a manifest entry and an
# undeclared class is a loud error instead of a guess.

KIND_FIXTURE = "fixture"      # "there is a commode here" — gives the bbox regions are measured against
KIND_STATUS = "status"        # "...and its lid is open" — gives fixture_state
KIND_STAIN = "stain"          # cleanable; drives the recommendation
KIND_PERMANENT = "permanent"  # real but not removable: recorded, never acted on
KIND_ESCALATE = "escalate"    # a person must look at this; outranks everything
KIND_IGNORE = "ignore"        # a class that is none of the above (fiducials, labels)
KIND_WASTE = "waste"          # an OBJECT, not a mark: tissue, a wrapper, a sharp.
                              # The response is an ACTION (clear / escalate), never a wipe

KINDS = (KIND_FIXTURE, KIND_STATUS, KIND_STAIN, KIND_PERMANENT, KIND_ESCALATE,
         KIND_IGNORE, KIND_WASTE)

# ── Severity ────────────────────────────────────────────────────────
# butterfli graded on confidence alone: "moderate" at >= 0.60, else "light"
# (ai_client.py:1125), with "permanent" for the non-removable set and "heavy"
# only ever produced by the cloud backend. Carried unchanged, including the
# threshold, because it is the grade the report format expects — but see
# `SEVERITY_IS_CONFIDENCE_NOT_AREA` below for what it does and does not mean.

LIGHT = "light"
MODERATE = "moderate"
HEAVY = "heavy"
PERMANENT = "permanent"

SEVERITIES = (LIGHT, MODERATE, HEAVY, PERMANENT)

#: Confidence at or above which a local detection is graded `moderate`.
MODERATE_CONFIDENCE = 0.60

#: Read this before using `severity` for anything that moves an arm. On the local
#: path severity is a function of DETECTOR CONFIDENCE, not of how dirty the
#: surface is: a faint-but-unmistakable mark grades `moderate` and a large
#: ambiguous smear grades `light`. It is fit for sorting a report and for
#: colouring an overlay. It is not a cleaning-effort estimate, and the physical
#: size gate in `analysis.py` is the thing that actually decides whether a
#: detection is worth an arm.
SEVERITY_IS_CONFIDENCE_NOT_AREA = True


def severity_for(confidence: float) -> str:
    """butterfli's grade (ai_client.py:1125), as a function rather than a literal."""
    return MODERATE if confidence >= MODERATE_CONFIDENCE else LIGHT


# ── Recommendations, as AnalyzeScene.srv spells them ────────────────
PASS = "pass"
RE_CLEAN = "re-clean"
TARGET_CLEAN = "target-clean"
ESCALATE = "escalate_to_human"
#: Added 2026-10-05 (Newton: "detect and command an action of detected and
#: identified waste"). Waste that the hand can take is on the fixture or the
#: floor and must be CLEARED BEFORE any wiping: a pad passed over a tissue
#: cleans nothing and drags the tissue through the soil. So this outranks
#: re-clean / target-clean, and only a person-required condition outranks it.
#: `AnalyzeScene.srv` documents the first four as examples, not as an enum;
#: the one consumer (`dual_arm_trajectory_sender.call_analyze_scene`) prints
#: the value and filters ROIs by region, so an added value breaks nothing.
CLEAR_WASTE = "clear-waste"

RECOMMENDATIONS = (PASS, RE_CLEAN, TARGET_CLEAN, CLEAR_WASTE, ESCALATE)


@dataclass
class Detection:
    """One 2D detection, in PIXELS of the image it was found in.

    `bbox` is (x_min, y_min, x_max, y_max). Pixels, not normalised: normalising
    needs the image size, and carrying the size alongside a normalised box is two
    things to keep in step. The backend already has the image; it converts once.
    """

    bbox: tuple[float, float, float, float]
    confidence: float
    label: str = ""
    region: str = ""

    #: One of `KINDS`. The backend fills it from what the model declared, so
    #: everything downstream can tell a cleanable stain from a permanent mark
    #: from a biohazard without re-reading class names.
    kind: str = KIND_STAIN

    #: One of `SEVERITIES`. See `SEVERITY_IS_CONFIDENCE_NOT_AREA`.
    severity: str = ""

    def __post_init__(self) -> None:
        if not self.severity:
            self.severity = (PERMANENT if self.kind == KIND_PERMANENT
                             else severity_for(self.confidence))

    @property
    def center(self) -> tuple[float, float]:
        x0, y0, x1, y1 = self.bbox
        return (x0 + x1) / 2.0, (y0 + y1) / 2.0

    @property
    def size_px(self) -> tuple[float, float]:
        x0, y0, x1, y1 = self.bbox
        return abs(x1 - x0), abs(y1 - y0)


@dataclass
class ROI3D:
    """A detection lifted into the reference frame, metres.

    `depth_source` records HOW the range was obtained, because the difference
    matters to whoever consumes the position: a measured pixel is worth a few mm,
    a fallback constant is worth nothing better than the guess it came from, and
    a caller that cannot tell them apart will treat both as a measurement.
    """

    center: tuple[float, float, float]
    width: float
    height: float
    confidence: float
    region: str
    depth_m: float
    depth_source: str = "measured"      # measured | fallback | invalid
    label: str = ""
    kind: str = KIND_STAIN
    severity: str = ""

    #: The pixel box this was lifted from, (x0, y0, x1, y1) in the ORIGINAL
    #: image. Carried because the only way to show a person what the robot
    #: reacted to is to draw it back on the frame, and recomputing the box by
    #: re-projecting the 3D centre loses the shape and re-introduces every
    #: approximation the lift made.
    bbox_px: tuple[float, float, float, float] | None = None

    #: Why this one was not acted on, for the entries in `SceneAssessment.rejected`.
    #: Empty on anything in `rois`.
    rejected_reason: str = ""

    #: For WASTE: what the robot is told to do about this item (`clear` or
    #: `escalate`, from `sdg.waste.action_for`). Written into
    #: `AnalyzeScene.Response.roi_recommendations` for that ROI. Empty for a
    #: stain, whose per-ROI entry is the scene recommendation as before.
    action: str = ""

    #: For WASTE: this item needs the hazard handling (sanitary waste, sharps, a
    #: glove, a mask — `waste_classes.yaml`). The robot still clears it (Newton,
    #: 2026-10-05); the flag is so the orchestrator can route it to that
    #: handling rather than the ordinary pick-up.
    hazard: bool = False

    #: Where `region` came from: "mesh" (the fixture's region meshes and a
    #: supplied fixture pose, `region3d`), "bands" (`region_bands.yaml` on the
    #: status model's box), or "" (unrouted). The two disagree at every pose
    #: the bands were not authored for, and a consumer should know which it got.
    region_source: str = ""


@dataclass
class FixtureStatus:
    """What the status pass established, and what the stain pass needs from it.

    This exists to replace an instance attribute. butterfli's client cached the
    fixture box on `self._last_fixture_bbox` between the two calls
    (ai_client.py:1290-1301), which made `detect_stains` silently depend on a
    `check_status` having happened first, on the same object, for the same image.
    Call them out of order and the stains were mapped against the PREVIOUS
    frame's fixture — no error, just wrong regions.

    Passing it makes the dependency a parameter. `detect_stains(image, status)`
    with no status is then an honest "I have no fixture box", which yields
    unrouted detections instead of stale ones.
    """

    fixture_type: str = ""
    fixture_state: str = ""
    confidence: float = 0.0

    #: The fixture's box in the image, (x0, y0, x1, y1) pixels, or None if the
    #: model found no fixture. Region bands are fractions of THIS, not of the
    #: frame, which is what makes the mapping survive a camera distance change.
    bbox: tuple[float, float, float, float] | None = None

    #: True when the state is the fixture type's default rather than something
    #: the model saw. butterfli returned `state_confidence = 0.0` to signal this
    #: (ai_client.py:755-763), which a caller comparing against a threshold reads
    #: as "low confidence" and treats the same as a bad detection. They are not
    #: the same: one is a guess, the other is a measurement that came out weak.
    state_is_default: bool = False

    def __bool__(self) -> bool:
        return bool(self.fixture_type)


@dataclass
class SceneAssessment:
    """What one analysis produced — the core's half of an AnalyzeScene response."""

    fixture_type: str = ""
    fixture_state: str = ""
    state_confidence: float = 0.0
    #: Who decided `fixture_state`: "depth" (the fixture's geometry, `region3d.
    #: state_from_depth`, when its margin was clear) or "colour" (the status
    #: model). The diagnostics carry both answers.
    state_source: str = ""
    recommendation: str = PASS
    rois: list[ROI3D] = field(default_factory=list)
    success: bool = False
    message: str = ""

    #: Regions that were detected but that this fixture model cannot clean —
    #: reported rather than dropped, so the caller sees a gap instead of silence.
    #: butterfli's most-reported region (`bowl_inside_front`, 6 of 8 field
    #: localisations) has no shipped cleaning config on alix.
    unservable_regions: list[str] = field(default_factory=list)

    #: Image roll at capture, degrees (see `camera.image_roll_deg`). Carried so a
    #: consumer can tell a rolled observation from an upright one after the fact.
    image_roll_deg: float = 0.0

    #: Real, located, and NOT robot-cleanable: permanent discolouration. Kept
    #: separate from `rois` because the orchestrator acts on `rois`, and a mark
    #: that will still be there after a perfect clean must never drive a
    #: re-clean. butterfli summarised these into one string
    #: (`permanent_discoloration_noted`); they are structured here so a report
    #: can say WHERE, and so the same mark can be recognised on the next visit.
    permanent: list[ROI3D] = field(default_factory=list)

    #: Detections that were found and then discarded, each carrying its
    #: `rejected_reason`. butterfli counted these; keeping them is what lets an
    #: annotated frame show the gate working rather than assert that it did.
    rejected: list[ROI3D] = field(default_factory=list)

    #: Waste items — objects, not marks — each carrying its `action`. Kept out
    #: of `rois` because `rois` is what gets WIPED, and a pad passed over a
    #: tissue cleans nothing. The orchestrator clears these first.
    waste: list[ROI3D] = field(default_factory=list)

    #: Set when an escalate-class detection forced `ESCALATE`, naming what did
    #: it. An escalation with no reason attached is not actionable by the person
    #: it escalates to.
    escalation_reason: str = ""

    #: Free-form per-backend diagnostics. Never load-bearing.
    diagnostics: dict = field(default_factory=dict)
