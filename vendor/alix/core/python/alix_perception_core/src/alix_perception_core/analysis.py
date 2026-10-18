"""One scene analysis, end to end, with no ROS in sight.

`PerceptionCore.analyse(...)` takes an image, an optional depth image, the
intrinsics and one 4x4 transform, and returns a `SceneAssessment`. The node's
whole job is to gather those four things from the graph and translate the result
into `AnalyzeScene.srv` — which is what LAYERS.md §1 asks for and what makes this
testable against a directory of JPEGs with no robot present.

The order of operations is the interesting part, and each step earns its place:

1. **De-rotate** by the measured image roll, if the caller supplied a transform.
   The camera is rigid to the wrist roll, and alix's own inspection poses put the
   scene 64-106 degrees over. Detections are mapped back to original pixels
   before anything geometric touches them, because the intrinsics belong to the
   original image.
2. **Status** first, because the fixture state decides which regions can
   legitimately be reported at all.
3. **Stains**, then lifted to 3D — the range is needed by step 4.
4. **The physical-size gate.** Discard anything smaller than `min_stain_mm` on
   the surface. This is the gate against the failure that made the inherited
   model unusable: 17 detections on a clean fixture, every one an ~8-10 mm
   fiducial dot. A pixel threshold cannot express "8 mm"; a millimetre threshold
   survives a range change, a resolution change and a retrain.
5. **Region validation** against the fixture's own `regions.yaml`: a region this
   model does not have, or cannot be in given the state, is dropped with a
   reason. A region that is real but has no cleaning task is kept and reported
   separately as unservable, because that is a gap someone should see.
"""

from __future__ import annotations

import logging

import numpy as np

from . import geometry
from .backends.base import Backend, BackendError, create
from .camera import DEFAULT_DEPTH_PROFILE, Intrinsics, image_roll_deg, CANONICAL_UP_WORLD
from .regions import RegionVocabulary, load_vocabulary
from .types import (CLEAR_WASTE, ESCALATE, KIND_ESCALATE, KIND_PERMANENT,
                    KIND_STAIN, KIND_WASTE, PASS, RE_CLEAN, TARGET_CLEAN,
                    FixtureStatus, ROI3D, SceneAssessment)

logger = logging.getLogger("alix_perception_core.analysis")

#: Below this the state classification is not acted on. Matches butterfli's
#: `confidence_threshold`, which was tuned against the cloud backend; a local
#: model should be re-tuned against `tools/perception/eval_models.py` rather than
#: inheriting this number as if it were measured for it.
DEFAULT_STATE_CONFIDENCE = 0.65

#: Roll beyond which de-rotation is applied. Below it the warp costs more than it
#: buys, and interpolation on a nearly-upright image only softens edges.
ROLL_DEADBAND_DEG = 5.0


class PerceptionCore:
    """Backend + geometry + vocabulary, composed."""

    def __init__(self, backend: Backend | None = None,
                 vocabulary: RegionVocabulary | None = None,
                 state_confidence_threshold: float = DEFAULT_STATE_CONFIDENCE,
                 min_stain_mm: float | None = None,
                 fallback_depth_m: float = 0.9,
                 derotate: bool = True,
                 canonical_up=CANONICAL_UP_WORLD,
                 locator=None,
                 state_fusion: bool = True):
        self.backend = backend if backend is not None else create()
        self.vocabulary = vocabulary if vocabulary is not None else load_vocabulary()
        self.state_confidence_threshold = float(state_confidence_threshold)
        self.fallback_depth_m = float(fallback_depth_m)
        self.derotate = bool(derotate)
        #: What points UP after de-rotation, in the frame `camera_to_world` maps
        #: into (see `camera.CANONICAL_UP_WORLD`). None = gravity only.
        self.canonical_up = None if canonical_up is None else tuple(float(v) for v in canonical_up)
        #: A `region3d.RegionLocator` (anything with `locate(point, state)`).
        #: With it AND a `fixture_pose` in `analyse`, regions come from the
        #: fixture's meshes; without either, from the bands.
        self.locator = locator
        #: THE STATE POLICY (Newton, 2026-10-08): the depth and the colour model both
        #: answer wherever they can. Agreement = the state, surer than either alone;
        #: the depth alone = the depth (it abstains rather than errs: 0 wrong in 404
        #: real decisions); the colour alone = the colour at its own confidence;
        #: DISAGREEMENT = no state ("uncertain"), for the executive to re-check from
        #: another pole height rather than for this code to pick. False = the old
        #: rule, the depth overriding the colour whenever it is confident.
        self.state_fusion = bool(state_fusion)
        self.min_stain_mm = float(
            min_stain_mm if min_stain_mm is not None
            else getattr(self.backend, "min_stain_mm", 15.0))

    # ------------------------------------------------------------------
    def check_status(self, image: np.ndarray,
                     camera_to_world: np.ndarray | None = None,
                     fixture_hint: str = "",
                     prompt_appendix: str = "",
                     depth_image: np.ndarray | None = None,
                     intrinsics: Intrinsics | None = None,
                     camera_to_target: np.ndarray | None = None,
                     fixture_pose: np.ndarray | None = None,
                     depth_registration_m: float = 0.0) -> SceneAssessment:
        """Fixture type and state.

        `camera_to_world` is the GRAVITY-ALIGNED transform used to measure the
        image roll — see `analyse`. None disables de-rotation.

        `fixture_hint` is `AnalyzeScene.srv`'s field of the same name: the caller
        telling the backend which fixture it is standing in front of.

        With `depth_image`, `intrinsics`, `camera_to_target` and `fixture_pose`
        (and a `locator`), THE DEPTH DECIDES THE STATE when its margin is clear
        (`region3d.state_from_depth`); the colour model's answer is kept in the
        diagnostics and used when the depth cannot decide. Without them, colour
        only, as before.
        """
        out = SceneAssessment()
        image, _, out.image_roll_deg = self._prepare(image, camera_to_world)

        try:
            status = self.backend.check_status(
                image, fixture_hint, **self._appendix_kwargs(prompt_appendix))
        except BackendError as exc:
            out.message = str(exc)
            return out

        self._apply_status(out, status)
        out.recommendation = PASS
        ds = self._decide_state(out, status, depth_image, intrinsics, camera_to_target, fixture_pose,
                                depth_registration_m)
        verdict = self._fuse_state(out, status, ds)
        if verdict in ("agree", "depth", "disagree"):
            out.success = True
            out.message = self._state_message(out, status, ds, verdict)
            return out

        if not status.fixture_type:
            out.message = "no fixture recognised in the frame"
            return out
        if status.state_is_default:
            # NOT a confidence failure. The model has no class that reads state,
            # so no number of retries and no better frame will produce one.
            out.message = (f"{status.fixture_type}: state not classified by this "
                           f"model; assuming '{status.fixture_state}'")
            out.success = True
            return out
        if status.confidence < self.state_confidence_threshold:
            out.message = (f"state confidence {status.confidence:.2f} below threshold "
                           f"{self.state_confidence_threshold:.2f}")
            return out

        out.success = True
        out.message = f"{status.fixture_type}: {status.fixture_state}"
        return out

    def _appendix_kwargs(self, appendix: str) -> dict:
        """`{'prompt_appendix': ...}` only for a backend that says it takes one.

        `AnalyzeScene.srv`'s `custom_prompt_appendix` means something to a backend
        that builds a prompt and nothing to one that runs a detector. Passing it
        unconditionally would make every local backend's signature grow a
        parameter it must accept and ignore — and a parameter accepted and
        ignored is how a caller comes to believe their instruction was followed.
        """
        if not (appendix or "").strip():
            return {}
        if not getattr(self.backend, "accepts_prompt_appendix", False):
            logger.info("prompt appendix ignored: the '%s' backend has no prompt",
                        getattr(self.backend, "name", "?"))
            return {}
        return {"prompt_appendix": appendix}

    @staticmethod
    def _apply_status(out: SceneAssessment, status: FixtureStatus) -> None:
        out.fixture_type = status.fixture_type
        out.fixture_state = status.fixture_state
        out.state_confidence = status.confidence
        out.state_source = "colour" if status.fixture_type else ""
        out.diagnostics["fixture_bbox_px"] = (
            tuple(round(float(v), 1) for v in status.bbox) if status.bbox else None)
        out.diagnostics["state_is_default"] = status.state_is_default

    def _decide_state(self, out: SceneAssessment, status: FixtureStatus,
                      depth_image, intrinsics, camera_to_target, fixture_pose,
                      registration_m: float = 0.0):
        """Let the depth decide the state when it can. -> DepthState or None.

        None = the colour answer stands (no locator, no depth, no pose, or a
        margin too small to decide); the diagnostics say which. When the depth
        decides, `out.fixture_state` / `state_confidence` / `state_source` are
        the depth's, the colour answer is kept beside them, and a fixture the
        colour model did not find is still reported — the geometry found it.
        """
        if self.locator is None or depth_image is None or intrinsics is None \
                or camera_to_target is None or fixture_pose is None \
                or not (hasattr(self.locator, "cells") and hasattr(self.locator, "nearest")):
            return None
        from .region3d import state_from_depth
        depth_m = geometry.to_metres(depth_image)
        # `registration_m`: the session's registration at this pose — how much higher the
        # real fixture sits than its nominal pose along the body's z, learned by the caller
        # from an earlier confident decision (`depth_offset_mm`). MEASURED 2026-10-08: +9 mm
        # at pose 4, most of the way from the seat ring to the lid top; with it the depth
        # decided every open state at pose 4 at every pole height, none without it.
        ds = state_from_depth(depth_m, intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy,
                              camera_to_target, fixture_pose, self.locator, registration_m=float(registration_m or 0.0))
        out.diagnostics.update({
            "colour_state": status.fixture_state, "colour_confidence": round(float(status.confidence or 0.0), 3),
            "depth_state": ds.state, "depth_margin_mm": ds.margin_mm, "depth_residual_mm": ds.residual_mm,
            "depth_points": ds.points, "depth_reason": ds.reason,
            "depth_registration_mm": round(float(registration_m or 0.0) * 1000.0, 1)})
        if ds.confident:
            out.diagnostics["depth_offset_mm"] = round(float(ds.offset_m) * 1000.0, 1)
        return ds

    def _colour_found(self, status: FixtureStatus) -> bool:
        """The colour model named a state it is sure enough of."""
        return bool(status.fixture_type) and bool(status.fixture_state) and not status.state_is_default \
            and float(status.confidence or 0.0) >= self.state_confidence_threshold

    def _fuse_state(self, out: SceneAssessment, status: FixtureStatus, ds) -> str:
        """Combine the colour answer (already on `out`) with the depth's. -> the verdict:
        'agree' | 'depth' | 'colour' | 'disagree' | 'none' (also `diagnostics['fusion']`).

        MEASURED on 362 real frames of both robot sessions (v10 colour, registered
        depth): 334 agree and are right, 28 without depth are right from colour,
        no disagreement, no wrong state."""
        depth_sure = ds is not None and ds.confident
        colour_sure = self._colour_found(status)
        fixture_type = out.fixture_type or getattr(self.vocabulary, "fixture_type", "") or "commode"
        if depth_sure and not self.state_fusion:            # the old rule: the depth overrides
            verdict = "depth"
        elif depth_sure and colour_sure:
            verdict = "agree" if ds.state == status.fixture_state else "disagree"
        elif depth_sure:
            verdict = "depth"
        elif colour_sure:
            verdict = "colour"
        else:
            verdict = "none"
        if verdict == "agree":
            out.fixture_type = fixture_type
            out.fixture_state = ds.state
            out.state_confidence = min(0.99, 1.0 - (1.0 - ds.confidence) * (1.0 - float(status.confidence)))
            out.state_source = "depth+colour"
        elif verdict == "depth":
            out.fixture_type = fixture_type
            out.fixture_state = ds.state
            out.state_confidence = ds.confidence
            out.state_source = "depth"
        elif verdict == "disagree":
            out.fixture_type = fixture_type
            out.fixture_state = ""
            out.state_confidence = 0.0
            out.state_source = "disagree"
        # 'colour' and 'none': what `_apply_status` set stands
        out.diagnostics["fusion"] = verdict
        out.diagnostics["state_source"] = out.state_source
        return verdict

    @staticmethod
    def _state_message(out: SceneAssessment, status: FixtureStatus, ds, verdict: str) -> str:
        colour = f"colour said '{status.fixture_state or '?'}' at {float(status.confidence or 0.0):.2f}"
        if verdict == "disagree":
            return (f"{out.fixture_type}: state uncertain — depth says '{ds.state}' (margin {ds.margin_mm:.0f} mm), "
                    f"{colour}; re-check from another pole height")
        if verdict == "agree":
            return (f"{out.fixture_type}: {out.fixture_state} (depth and colour agree; depth margin "
                    f"{ds.margin_mm:.0f} mm, colour {float(status.confidence or 0.0):.2f})")
        return f"{out.fixture_type}: {out.fixture_state} (from depth, margin {ds.margin_mm:.0f} mm; {colour})"

    # ------------------------------------------------------------------
    def analyse(self, image: np.ndarray, intrinsics: Intrinsics,
                camera_to_target: np.ndarray,
                depth_image: np.ndarray | None = None,
                regions_to_assess: tuple[str, ...] = (),
                camera_to_world: np.ndarray | None = None,
                fixture_hint: str = "",
                prompt_appendix: str = "",
                fixture_pose: np.ndarray | None = None,
                depth_state: bool = True,
                depth_registration_m: float = 0.0) -> SceneAssessment:
        """Full analysis: state, stains, 3D positions, gating.

        Args:
            camera_to_target: camera optical frame -> the OUTPUT frame
                (`alix_ref_frame`). Used for projection.
            fixture_pose: 4x4 pose of the fixture BODY frame in the OUTPUT
                frame. With a `locator`, every size-gated detection is routed
                to a region by the fixture's own meshes (`region3d`) and the
                bands are not consulted: a point no visible region cell is
                near is unrouted, which is what a detection on the cloth
                beside the fixture should be. None = the bands, as before.
            camera_to_world: camera optical frame -> a GRAVITY-ALIGNED frame
                (`base_link`). Used only to measure the image roll. These are two
                different transforms and must not be conflated: `alix_ref_frame`
                is parented to `R_Pole` and rides the pole, so a roll measured
                against it is meaningless. None disables de-rotation, which is
                the honest behaviour when no gravity reference is available.
        """
        out = SceneAssessment()
        work, inverse, out.image_roll_deg = self._prepare(image, camera_to_world)

        extra = self._appendix_kwargs(prompt_appendix)
        try:
            status = self.backend.check_status(work, fixture_hint, **extra)
            # The status is PASSED, not cached on the backend: detect_stains needs
            # the fixture box to express a region as a fraction of the fixture
            # rather than of the frame, and butterfli's instance-attribute version
            # of this handoff silently mapped against the previous frame's box
            # whenever the two calls came out of order.
            detections = self.backend.detect_stains(work, status, **extra)
        except BackendError as exc:
            out.message = str(exc)
            return out

        self._apply_status(out, status)
        # `depth_state` False: the depth still lifts the detections to 3D below, but
        # the STATE stays with colour (the node says so at every pose but the
        # inspection pose the depth state is for).
        ds = (self._decide_state(out, status, depth_image, intrinsics, camera_to_target, fixture_pose,
                                 depth_registration_m)
              if depth_state else None)
        verdict = self._fuse_state(out, status, ds)
        depth_state = ds if verdict in ("agree", "depth", "disagree") else None
        # the state the stains are routed by: the fused one; when the two disagree
        # the DEPTH's (it has never been wrong where it decided) while the reported
        # state stays empty for the executive to re-check
        state = out.fixture_state or (ds.state if (ds is not None and ds.confident) else status.fixture_state)

        if not out.fixture_type:
            out.message = "no fixture recognised in the frame"
            return out

        if inverse is not None:
            detections = [geometry.unrotate_detection(d, inverse) for d in detections]

        if depth_image is not None:
            depth_image = geometry.to_metres(depth_image)

        kept: list[ROI3D] = []
        permanent: list[ROI3D] = []
        rejected: list[ROI3D] = []
        escalations: list[ROI3D] = []
        waste: list[ROI3D] = []
        dropped_small = 0
        dropped_region: list[str] = []
        unservable: list[str] = []
        wanted = {self.vocabulary.canonical(r) for r in regions_to_assess}
        route_by_mesh = self.locator is not None and fixture_pose is not None
        body_from_target = (np.linalg.inv(np.asarray(fixture_pose, dtype=float))
                            if route_by_mesh else None)
        mesh_routed = mesh_unrouted = 0

        def _reject(roi: ROI3D, reason: str) -> None:
            """Keep a discarded detection, with why. See SceneAssessment.rejected."""
            rejected.append(ROI3D(**{**roi.__dict__, "rejected_reason": reason}))

        for det in detections:
            roi = geometry.roi_from_detection(
                det, intrinsics, camera_to_target,
                depth_image=depth_image,
                fallback_depth_m=self.fallback_depth_m)

            # ESCALATION IS NOT SIZE-GATED. The gate exists to stop a fiducial dot
            # becoming a cleaning task; a small biohazard is still a biohazard, and
            # a size threshold tuned against stains has no authority over it.
            if det.kind == KIND_ESCALATE:
                escalations.append(roi)
                continue

            # WASTE IS AN OBJECT, AND THE RESPONSE IS AN ACTION (Newton,
            # 2026-10-05). It is not size-gated against min_stain_mm — that gate
            # is tuned for fiducial dots — and it is not region-validated: a
            # tissue on the floor has no fixture region and is still something
            # to clear. Its action comes from the waste table, where every
            # shipped class is the robot's job, hazards included (Newton, the
            # same day: "sanitary waste and sharps should also be handled by the
            # robot"); the hazard flag rides along so the orchestrator can pick
            # the handling. A label the table does not know ESCALATES, because
            # "I do not know what this is" must read as "a person looks", never
            # as "pick it up".
            if det.kind == KIND_WASTE:
                from .sdg.waste import ACTION_ESCALATE, action_for, is_hazard
                action = action_for(det.label)
                roi = ROI3D(**{**roi.__dict__, "action": action,
                               "hazard": is_hazard(det.label)})
                waste.append(roi)
                if action == ACTION_ESCALATE:
                    escalations.append(roi)
                continue

            if 1000.0 * max(roi.width, roi.height) < self.min_stain_mm:
                dropped_small += 1
                _reject(roi, f"below the {self.min_stain_mm:.0f} mm size gate")
                continue

            if route_by_mesh:
                # THE MESHES DECIDE. The bands assume a face-on view with the
                # fixture's back at the top of its box; a wrist camera at
                # inspection_pose_4 sees neither. A point near no visible
                # region cell stays unrouted — the bands would have put the
                # cloth beside the fixture on the fixture.
                point = body_from_target @ np.array([*roi.center, 1.0])
                region3d, _distance = self.locator.locate(point[:3], state)
                if region3d:
                    mesh_routed += 1
                    roi = ROI3D(**{**roi.__dict__, "region": region3d, "region_source": "mesh"})
                else:
                    mesh_unrouted += 1
                    roi = ROI3D(**{**roi.__dict__, "region": "", "region_source": ""})
            elif roi.region:
                roi = ROI3D(**{**roi.__dict__, "region_source": "bands"})

            if roi.region:
                region = self.vocabulary.canonical(roi.region)
                ok, why = self.vocabulary.validate(region, state)
                if not ok:
                    dropped_region.append(why)
                    _reject(roi, why)
                    continue
                roi = ROI3D(**{**roi.__dict__, "region": region})
                entry = self.vocabulary.get(region)
                if entry is not None and not entry.servable:
                    unservable.append(region)
                if wanted and region not in wanted:
                    _reject(roi, f"'{region}' was not among the regions requested")
                    continue

            # PERMANENT MARKS ARE RECORDED, NEVER ACTED ON (butterfli ai_client.py:
            # 1132-1190, 869-875). A discolouration that will still be there after
            # a perfect clean must not drive a re-clean, or the robot cleans the
            # same spot until it gives up. Kept located and structured so a report
            # can say where, and so the next visit can recognise the same mark.
            if det.kind == KIND_PERMANENT:
                permanent.append(roi)
                continue

            kept.append(roi)

        # A detection with no region is a real observation the orchestrator
        # cannot act on: `RunCleaningTask` is keyed by region. Counted rather
        # than dropped, so "perception saw things and nothing happened" is
        # visible instead of mysterious. Resolving these from geometry needs a
        # measured fixture pose — see the report's §6.1.
        unrouted = sum(1 for r in kept if not r.region)

        out.rois = kept
        out.permanent = permanent
        out.rejected = rejected
        out.waste = waste
        out.unservable_regions = sorted(set(unservable))
        if escalations:
            out.escalation_reason = self._escalation_reason(escalations)
        out.recommendation = self._recommend(kept, out.unservable_regions,
                                             bool(escalations),
                                             clearable=sum(1 for w in waste
                                                           if w.action != "escalate"))
        out.success = True
        if depth_state is not None:
            out.message = self._state_message(out, status, depth_state, verdict)
        out.diagnostics.update({
            "detections_raw": len(detections),
            "dropped_below_min_stain_mm": dropped_small,
            "dropped_invalid_region": len(dropped_region),
            "unrouted_no_region": unrouted,
            "routed_by_mesh": mesh_routed,
            "mesh_unrouted": mesh_unrouted,
            "permanent_found": len(permanent),
            "escalation_found": len(escalations),
            "waste_found": len(waste),
            "hazard_found": sum(1 for w in waste if w.hazard),
            "depth_used": depth_image is not None,
            "depth_fallback_used": sum(1 for r in kept if r.depth_source != "measured"),
            "min_depth_m": min((r.depth_m for r in kept), default=None),
            "min_z_m": DEFAULT_DEPTH_PROFILE.min_z_m,
            # Capability, so a caller can tell "this model cannot answer that"
            # from "it answered and was unsure".
            "supports_state_classification": getattr(
                self.backend, "supports_state_classification", False),
            "supports_escalation": getattr(self.backend, "supports_escalation", False),
            "supports_permanent_detection": getattr(
                self.backend, "supports_permanent_detection", False),
        })
        if dropped_region:
            out.diagnostics["region_reasons"] = dropped_region[:5]

        out.message = self._summarise(out, dropped_small)
        return out

    @staticmethod
    def _escalation_reason(escalations: list[ROI3D]) -> str:
        """Name what forced the escalation. A bare ESCALATE is not actionable."""
        labels = sorted({r.label or "unnamed" for r in escalations})
        where = sorted({r.region for r in escalations if r.region})
        text = f"{len(escalations)} detection(s) needing a person: {', '.join(labels)}"
        return text + (f" at {', '.join(where)}" if where else "")

    # ------------------------------------------------------------------
    def _prepare(self, image: np.ndarray, camera_to_world: np.ndarray | None):
        """De-rotate if the roll is worth removing. -> (image, inverse, roll).

        `camera_to_world` must be gravity-aligned, +X toward the fixture
        (base_link). Passing the output-frame transform here would rotate by an
        angle that means nothing. The roll is SIGNED; the dead band is on its
        magnitude.
        """
        if camera_to_world is None or not self.derotate:
            return image, None, 0.0
        roll = image_roll_deg(np.asarray(camera_to_world, dtype=float), up_world=self.canonical_up)
        if abs(roll) < ROLL_DEADBAND_DEG:
            return image, None, roll
        rotated, inverse = geometry.rotate_image_upright(image, roll)
        logger.debug("de-rotated by %.1f deg", roll)
        return rotated, inverse, roll

    def _recommend(self, rois: list[ROI3D], unservable: list[str],
                   escalated: bool = False, clearable: int = 0) -> str:
        """Map what was found onto AnalyzeScene's four decisions.

        Kept simple and legible on purpose. The thresholds are a starting policy,
        not a measurement — there is no labelled corpus that would justify tuned
        ones, and pretending otherwise would give the number false authority.
        """
        # Escalation outranks everything, including a clean frame (butterfli
        # ai_client.py:852-866). Ordered first on purpose: a biohazard beside
        # zero stains is still a biohazard, and deciding PASS from an empty
        # stain list would bury it.
        if escalated:
            return ESCALATE
        # Clearable waste outranks every wipe decision: a pad passed over a
        # tissue cleans nothing and drags the tissue through the soil. The
        # orchestrator clears first, then asks again.
        if clearable:
            return CLEAR_WASTE
        if not rois:
            return PASS
        if unservable and len(unservable) == len({r.region for r in rois}):
            # Everything found is real but nothing can be cleaned: a person has
            # to decide, and silently passing would report a clean fixture.
            return ESCALATE
        return RE_CLEAN if len(rois) > 4 else TARGET_CLEAN

    @staticmethod
    def _summarise(out: SceneAssessment, dropped_small: int) -> str:
        bits = [f"{out.fixture_type}: {out.fixture_state}",
                f"{len(out.rois)} stain(s)"]
        if dropped_small:
            bits.append(f"{dropped_small} below the size gate")
        unrouted = out.diagnostics.get("unrouted_no_region", 0)
        if unrouted:
            bits.append(f"{unrouted} with no region (not actionable)")
        if out.permanent:
            bits.append(f"{len(out.permanent)} permanent (recorded, not cleaned)")
        if out.waste:
            actions = sorted({f"{w.label or 'waste'}->{w.action}"
                              f"{'(hazard)' if w.hazard else ''}" for w in out.waste})
            bits.append(f"{len(out.waste)} waste item(s): {', '.join(actions)}")
        if out.escalation_reason:
            bits.append(out.escalation_reason)
        if out.unservable_regions:
            bits.append("no cleaning task for " + ", ".join(out.unservable_regions))
        if abs(out.image_roll_deg) >= ROLL_DEADBAND_DEG:
            bits.append(f"de-rotated {out.image_roll_deg:.0f} deg")
        return "; ".join(bits)
