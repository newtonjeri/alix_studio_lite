"""The gap check — is the lid / the seat closed, lifted or open? — decided from one depth frame, ROS-free.

ONE CODE FOR BOTH (the owner, 2026-10-24: "what ALIX ... does should be the same as what happens in Lite ... the same
result for the same input in both"). ALIX's perception node (the AnalyzeScene service) and ALIX Studio Lite
(pyrealsense2) call `GapCheckSession.check` with the same inputs — the depth frame in metres, its intrinsics, the
camera's pose in the world frame, the hands' positions, the named pose the camera is at — and get the same decision
and the same check-record fields. Each caller keeps only what is its own: getting those inputs (ROS messages and TF;
the D435 and FK), refusing when it has none, the colour models' call (`models`), the record's file and the clock
stamps. Moved here from perception_node.py unchanged in what it decides; its history is in the comments it carries.

Geometry from depth: the ROI readings (the lid's centre, the seat's band, the lid's structural cues), the seat
against its own closed frame (seat_rise.py), the lid's and the seat's state from the whole fixture
(seat_state.fit_fixture_state), and the colour models' answer as a cross-check. A session learns where the fixture
stands (its registration, per named pose and placement), the seat's closed reference and the last lid decision.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

import numpy as np


class _NullLog:
    def info(self, *_a, **_k): pass
    def warn(self, *_a, **_k): pass
    def debug(self, *_a, **_k): pass


@dataclass
class GapDecision:
    """One frame's answer. `record` is the check record's decision fields (the caller adds the clock stamps and
    writes the file with the depth it passed in)."""
    part: str
    expect: str
    success: bool
    state: str                      # a class (lid_closed, lid_lifted, lid_open, lid_between, seat_closed, seat_lifted,
    confidence: float               #  uncertain) or "refused" (success False)
    recommendation: str             # "pass" when the state meets the expectation, else "re-clean"
    message: str
    record: dict = field(default_factory=dict)


def load_params(path) -> dict:
    """perception.yaml's parameters (the `ros__parameters` block) as a dict: what the node launches with. Its
    defaults in code equal them (2026-10-24: all 63 the gap check reads), so a caller without ROS reads this."""
    import yaml
    with open(path) as handle:
        data = yaml.safe_load(handle) or {}

    def find(d):
        for k, v in d.items():
            if k == "ros__parameters":
                return v
            if isinstance(v, dict):
                got = find(v)
                if got is not None:
                    return got
        return None
    return dict(find(data) or {})


def load_fixture(param: Callable[[str], Any], sim_time: bool = False, log=None) -> tuple:
    """(RegionLocator or None, the fixture's pose in the world frame or None), as ALIX's node places it: the scene
    file (fixture_scene_yaml, else the fixture model's own), the calibration (fixture_calibration "auto": the
    measured file on the robot, none under sim time: the simulator's fixture IS the model), the UGV standoff
    (ugv_standoff_m < 0: the scene's own) and the venue's rotation (fixture_venue_yaml; the washroom turns the
    fixture 180 deg to face the robot). Either missing means the regions route by bands, and the log says so.
    Shared since 2026-10-24: ALIX Studio Lite placed the fixture without the venue's yaw, 0.5 m off."""
    log = log or _NullLog()
    if not bool(param("route_by_mesh")):
        log.info("route_by_mesh false: regions come from region_bands.yaml")
        return None, None
    try:
        from .region3d import RegionLocator, default_calibration_file, load_fixture_pose
        scene = param("fixture_scene_yaml") or ""
        choice = str(param("fixture_calibration") or "auto").strip()
        if choice.lower() == "none" or (choice.lower() == "auto" and sim_time):
            calibration = None
        elif choice.lower() == "auto":
            calibration = default_calibration_file(str(param("fixture_model")))
        else:
            calibration = choice
        if scene:
            from pathlib import Path
            locator = RegionLocator(Path(scene).parent / "models", scene, calibration=calibration)
        else:
            locator = RegionLocator.for_fixture(param("fixture_model"), param("fixture_type"), calibration=calibration)
        log.info("fixture calibration: " + (f"{locator.calibration} ({calibration})" if locator.calibration
                                            else ("NONE — the simulator's fixture is the model" if sim_time and choice.lower() == "auto"
                                                  else "none (raw meshes)")))
        standoff = float(param("ugv_standoff_m"))
        venue = param("fixture_venue_yaml") or ""
        pose = load_fixture_pose(locator.scene_yaml, None if standoff < 0 else standoff,
                                 venue_yaml=None if str(venue).lower() == "none" else venue)
    except Exception as exc:                        # noqa: BLE001
        log.warn(f"region meshes unavailable ({exc}); regions come from region_bands.yaml")
        return None, None
    log.info(f"regions routed by the fixture meshes: {len(locator.regions)} regions from "
             f"{locator.models_dir}, fixture at {pose[:3, 3].round(3).tolist()} in {param('world_frame')}")
    return locator, pose


class GapCheckSession:
    """The gap check's decision and the state it learns over a session. `param(name)` reads perception.yaml's
    values (the node's parameters; Lite reads the same file); `locator()` and `fixture_in_world()` give the
    fixture model and where it stands (region3d.RegionLocator and load_fixture_pose, the venue's yaw included);
    `now()` is the clock the seat's reference ages by."""

    #: a closed decision this far from the stored registration means the robot or the
    #: fixture MOVED: the stored value is replaced, not averaged (MEASURED 2026-10-08:
    #: the registration varies by < 2 mm across pole heights at one placement)
    REGISTRATION_RESET_M = 0.006

    #: gap classes the fused state can confirm or contradict, and the fused states each agrees with
    GAP_CLASS_AGREES = {"lid_closed": ("closed",), "lid_open": ("lid_open", "lid_seat_open"),
                        "seat_closed": ("closed", "lid_open"), "seat_open": ("lid_seat_open",)}

    #: the seat comparison's and the fixture fit's parameters (written into every check record beside the gap_* ones)
    SEAT_RISE_PARAMS = ("gap_seat_rise_enabled", "gap_seat_rise_region_m", "gap_seat_rise_hand_m", "gap_seat_rise_min_px",
                        "gap_seat_rise_closed_m", "gap_seat_rise_closed_frac", "gap_seat_rise_moved_m",
                        "gap_seat_rise_moved_frac", "gap_seat_lift_min_deg", "gap_seat_rise_hinge_tol_m",
                        "gap_seat_rise_down_frac", "gap_seat_rise_hinge", "gap_seat_ref_box", "gap_seat_ref_frames",
                        "gap_seat_ref_ring_m", "gap_seat_ref_margin_m", "gap_seat_ref_max_age_s", "gap_seat_ref_lid_z",
                        "gap_seat_ref_lid_m", "gap_seat_ref_lid_frac", "gap_seat_ref_lid_min_px", "gap_seat_ref_replace_m",
                        "gap_seat_fit_enabled", "gap_seat_fit_stride", "gap_seat_fit_step_deg", "gap_seat_fit_tol_m",
                        "gap_seat_fit_min_px", "gap_seat_fit_margin", "gap_lid_fit_enabled", "gap_lid_fit_min_deg",
                        "gap_lid_fit_open_min_deg", "gap_fit_height_search_m", "gap_fit_height_step_m")
    _SEAT_RISE_PARAMS = SEAT_RISE_PARAMS

    #: every parameter a check record carries
    RECORD_PARAMS = ("gap_lid_top_z", "gap_seat_top_z", "gap_lift_m", "gap_roi", "gap_seat_roi", "gap_seat_band_z",
                     "gap_seat_min_band", "gap_seat_absent_m", "gap_hand_mask_m", "gap_lid_closed_tol_m", "gap_lid_open_m",
                     "gap_lid_wide_roi", "gap_lid_deep_m", "gap_lid_hole_min", "gap_lid_top_rows",
                     "gap_lid_standing_range_m", "gap_lid_standing_min", *SEAT_RISE_PARAMS)

    #: lid decisions after which the closed seat frame no longer describes the fixture
    _SEAT_REF_CLEARING_LID = ("lid_closed", "lid_lifted", "lid_between")

    def __init__(self, param: Callable[[str], Any], locator: Callable[[], Any] = lambda: None,
                 fixture_in_world: Callable[[], Optional[np.ndarray]] = lambda: None, world_frame: str = "base_link",
                 log=None, now: Callable[[], float] = time.time):
        self._p = param
        self.locator = locator
        self.fixture_in_world = fixture_in_world
        self.world_frame = world_frame
        self.log = log or _NullLog()
        self.now = now
        self._registration: dict = {}           # registration key -> metres (body +z), learned from closed decisions
        self._seat_refs: dict = {}              # registration key -> the seat's closed reference (seat_rise)
        self._last_lid: dict = {}               # registration key -> the last lid class decided there
        self._seat_geo = None                   # seat_rise.SeatGeometry, built once
        self._hinged = None                     # seat_state.HingedFixture, built once (False: none)

    def record_params(self) -> dict:
        return {k: self._p(k) for k in self.RECORD_PARAMS}

    def _models_verdict(self, gap_class: str, models, at_pose: str) -> dict:
        """The fused state (colour + registered depth) for this frame from `models()` — the caller's
        PerceptionCore.check_status on the same moment's colour frame and aligned depth, or None — and whether it
        contradicts `gap_class`. {} when the class is a lifted one (no model knows a part at -30 deg) or nothing
        could be asked. A confident closed decision there teaches the registration (the fused path's rule)."""
        agrees = self.GAP_CLASS_AGREES.get(gap_class)
        if not agrees or models is None:
            return {}
        try:
            result = models()
        except Exception as exc:                                   # noqa: BLE001 — the geometry answers alone
            self.log.warn(f"gap_check: the models could not be asked ({exc}); the geometry answers alone")
            return {}
        if result is None:
            return {}
        self._learn_registration(at_pose, result)
        fused = result.fixture_state or ""
        confident = bool(fused) and float(result.state_confidence) >= float(self._p("confidence_threshold"))
        return {"fused_state": fused, "state_source": result.state_source, "confidence": round(float(result.state_confidence), 3),
                "fusion": result.diagnostics.get("fusion", ""), "colour_state": result.diagnostics.get("colour_state", ""),
                "depth_state": result.diagnostics.get("depth_state", ""),
                "contradiction": bool(confident and fused not in agrees)}

    def refusal(self, asked: str, text: str, record_extra: dict | None = None) -> GapDecision:
        """A refusal (success False, class "refused"): the caller's own (no frame, no pose, no intrinsics) or this
        module's (too little depth, an implausible height). Never "NOT lifted"."""
        part = "seat" if "seat" in (asked or "").lower() else "lid"
        self.log.warn(f"gap_check: {text}")
        record = {"class": "refused", "success": False, "message": text}
        record.update(record_extra or {})
        return GapDecision(part=part, expect=asked or "", success=False, state="refused", confidence=0.0,
                           recommendation="re-clean", message=text, record=record)

    def check(self, asked: str, depth, intrinsics, camera_to_world, hands: dict | None = None, at_pose: str = "",
              source: str = "native depth", frame: str = "camera_depth_optical_frame", models=None,
              color_used: bool = False, color_note: str = "") -> GapDecision:
        """mode 'gap_check' (P20, 2026-10-05): is the lid / seat LIFTED?

        Geometry from depth, not a model. The central ROI of the depth frame is
        deprojected through the camera intrinsics and the camera -> world_frame
        pose; the median height of those points is compared with the part's
        CLOSED top (gap_lid_top_z / gap_seat_top_z), and the part counts as
        lifted when it sits gap_lift_m higher. The part is named by `asked`
        ('lid_lifted' / 'seat_lifted' from the executive's check, or 'lid' /
        'seat'); default lid. Too little depth or an implausible height refuses
        (success False). Measured in Isaac from inspection_pose_4 (pole half): lid
        closed median z 0.45, lifted to -30 deg 0.61 — a 160 mm rise against a 50 mm
        bar. Answers are CLASSES, not yes/no: lid_closed / lid_lifted / lid_open
        (absent, the bowl shows) and seat_closed / seat_lifted (absent, the rim
        shows); `asked` names what the caller expects and the executive compares
        (lift_check.LIFTED_STATES). Since 2026-10-23 the seat, and since 2026-10-24
        the lid, are decided by their state in the whole fixture (seat_state), the
        ROI readings cross-checking it.

        `depth` in metres (the caller's frame, as it came); `intrinsics` (fx, fy, cx, cy) sized to it;
        `camera_to_world` the 4x4 camera -> world_frame pose; `hands` {frame name: position in world_frame} of the
        gap_hand_frames the caller could place (a missing one is named); `at_pose` the named pose the camera is at
        ('' = any); `source` / `frame` name the depth stream; `models()` -> the colour models' fused result on this
        moment's colour frame, or None when there is no colour frame."""
        asked = asked or ""
        part = "seat" if "seat" in asked.lower() else "lid"
        record_extra: dict = {}
        depth = np.asarray(depth, dtype=np.float64)
        h, w = depth.shape
        hands = {str(k): np.asarray(v, dtype=np.float64)[:3] for k, v in (hands or {}).items()}
        hand_frames = [str(f) for f in self._p("gap_hand_frames")]

        def keep(decision):  # NEVER rebind this name below (2026-10-07: a seat mask named `keep` shadowed it; every seat question killed the node)
            decision = dict(decision)
            decision.update(record_extra)
            decision.update({"source": source, "frame": frame, "world_frame": self.world_frame,
                             "camera_to_world": None if camera_to_world is None else np.asarray(camera_to_world, dtype=float).round(6).tolist(),
                             "params": self.record_params()})
            return decision

        def refuse(text):
            self.log.warn(f"gap_check: {text}")
            return GapDecision(part=part, expect=asked, success=False, state="refused", confidence=0.0, recommendation="re-clean",
                               message=text, record=keep({"class": "refused", "success": False, "message": text}))

        nominal_top = float(self._p("gap_seat_top_z" if part == "seat" else "gap_lid_top_z"))
        lid_registration = self._registration_for(at_pose)
        seat_key = self._registration_key(at_pose) + ("seat",)
        registration_source = "lid" if abs(lid_registration) > 1e-9 else "none"
        registration = lid_registration
        if part == "seat" and seat_key in self._registration:
            registration = float(self._registration[seat_key])
            registration_source = "seat"
        record_extra["intrinsics"] = {"fx": float(intrinsics.fx), "fy": float(intrinsics.fy), "cx": float(intrinsics.cx),
                                      "cy": float(intrinsics.cy), "width": int(w), "height": int(h)}
        r0, r1, c0, c1 = [float(v) for v in self._p("gap_seat_roi" if part == "seat" else "gap_roi")]
        rows = slice(int(r0 * h), max(int(r0 * h) + 1, int(r1 * h)))
        cols = slice(int(c0 * w), max(int(c0 * w) + 1, int(c1 * w)))
        z = depth[rows, cols]
        vs, us = np.mgrid[rows, cols]
        valid = np.isfinite(z) & (z > 0.05) & (z < 3.0)
        coverage = float(valid.mean()) if valid.size else 0.0
        # THE SEAT'S BAND READING MAY NOT JUDGE, AND THAT IS NOT "NOT LIFTED" (Newton 2026-10-23): a seat
        # refusal no longer returns here. It is kept as `seat_refusal` ("cannot judge ..."), and the
        # comparison with the closed frame (seat_rise) is consulted below; only when that cannot answer
        # either does the check refuse.
        seat_refusal = ""
        if coverage < 0.3:
            if part != "seat":
                return refuse(f"only {coverage * 100:.0f} % of the ROI carries depth -> NOT lifted")
            seat_refusal = f"cannot judge (only {coverage * 100:.0f} % of the seat ROI carries depth)"
        d = z[valid]
        u = us[valid].astype(np.float64)
        v = vs[valid].astype(np.float64)
        pts = np.stack([(u - intrinsics.cx) * d / intrinsics.fx,
                        (v - intrinsics.cy) * d / intrinsics.fy, d, np.ones_like(d)])
        world = np.asarray(camera_to_world, dtype=np.float64) @ pts
        z_world = world[2]
        detail = ""
        seat_hands: dict = {}                     # hand frame -> its position (world), for the mask and the comparison
        height = None
        if part == "seat":
            # Mask the hands: every point within gap_hand_mask_m of a hand frame
            # (a frame the caller could not place is skipped and named).
            radius = float(self._p("gap_hand_mask_m"))
            unmasked = np.ones(z_world.shape, dtype=bool)
            missing = []
            for frame_name in hand_frames:
                if frame_name not in hands:
                    missing.append(frame_name)
                    continue
                seat_hands[frame_name] = hands[frame_name]
                unmasked &= np.linalg.norm(world[:3] - seat_hands[frame_name][:, None], axis=0) > radius
            masked_frames = list(seat_hands)
            record_extra["hands"] = {k: [round(float(c), 4) for c in v] for k, v in seat_hands.items()}
            lo, hi = [float(v) + registration for v in self._p("gap_seat_band_z")]   # the band follows the placement
            band = unmasked & (z_world >= lo) & (z_world <= hi)
            band_frac = float(band.mean()) if band.size else 0.0
            covered = float((~unmasked).mean()) if unmasked.size else 0.0
            if not seat_refusal and band_frac < float(self._p("gap_seat_min_band")):
                seat_refusal = (f"cannot judge ({covered * 100:.0f} % of the seat's place covered by the hand; only "
                                f"{band_frac * 100:.0f} % of the seat ROI lies in the {lo:.2f}-{hi:.2f} m seat band)")
            if not seat_refusal:
                zb = z_world[band]
                height = float(np.median(zb))
                q10, q90 = np.percentile(zb, [10, 90])
                # The seat's own placement, from the ring itself (see gap_seat_register_min_band).
                seat_offset = height - nominal_top
                if band_frac >= float(self._p("gap_seat_register_min_band")) \
                        and abs(seat_offset) <= float(self._p("gap_register_m")) \
                        and abs(seat_offset - lid_registration) <= float(self._p("gap_seat_register_tol_m")) \
                        and abs(seat_offset - registration) > self.REGISTRATION_RESET_M:
                    old_reg = self._registration.get(seat_key)
                    self._registration[seat_key] = float(seat_offset)
                    self.log.warn(
                        f"seat registration at {at_pose or '(any pose)'}: {seat_offset * 1000:+.1f} mm (the ring read {height:.3f} m over "
                        f"{band_frac * 100:.0f} % of the ROI, nominal {nominal_top:.3f}; the lid's offset is {lid_registration * 1000:+.1f} mm)"
                        + (f" — replaces {old_reg * 1000:+.1f} mm" if old_reg is not None else ""))
                    if old_reg is not None:
                        self._clear_seat_references(at_pose, "the seat's registration was reset")
                    registration, registration_source = float(seat_offset), "seat"
                detail = (f"; seat band {band_frac * 100:.0f} % of the ROI, 10/90 % heights {q10:.3f}/{q90:.3f}, "
                          f"{covered * 100:.0f} % masked around {len(masked_frames)} hand frames"
                          + (f" ({len(missing)} missing: {missing[:3]})" if missing else ""))
            else:
                detail = (f"; {covered * 100:.0f} % of the seat ROI masked around {len(masked_frames)} hand frames"
                          + (f" ({len(missing)} missing: {missing[:3]})" if missing else ""))
        else:
            height = float(np.median(z_world))
        top = nominal_top + registration
        lift = float(self._p("gap_lift_m"))
        rise = None if height is None else height - top
        if height is not None and not (0.1 < height < 1.5):
            return refuse(f"{part} surface at an implausible {height:.3f} m in {self.world_frame} -> NOT lifted")
        # Classify what the frame shows; the executive compares the class with the
        # expectation it asked for (lift_check.LIFTED_STATES), so one measurement
        # answers "lifted?", "open?" and "closed?" alike.
        absent = False
        seat_note = ""
        band_state, band_conf = "", 0.0
        if part == "seat":
            if seat_refusal:
                state, conf = "refused", 0.0
            else:
                # the low band holds ring or rim only; "raised" is not read there
                absent_m = float(self._p("gap_seat_absent_m"))
                # a thin band claims "absent" only when the reading is clearly rim-deep
                absent = rise <= -absent_m and (band_frac >= float(self._p("gap_seat_absent_min_band")) or rise <= -2.0 * absent_m)
                state = "seat_lifted" if absent else "seat_closed"
                # seat_closed confidence: full within gap_seat_closed_tol_m below the top (the ring
                # reads 0.435-0.442 at one placement depending on which part of it the hand mask
                # leaves), then falling to zero at the absent boundary.
                closed_tol = float(self._p("gap_seat_closed_tol_m"))
                conf = float(np.clip(abs(rise) / (2.0 * absent_m), 0.0, 1.0)) if absent \
                    else float(np.clip(1.0 - max(0.0, -rise - closed_tol) / max(1e-6, absent_m - closed_tol), 0.0, 1.0))
            # The band reading, then the seat against its own closed frame (seat_rise): band refused ->
            # the comparison's answer; both agree -> kept; they conflict -> "uncertain" (no vote).
            band_state, band_conf = state, conf
            state, conf, seat_note = self._consult_seat_rise(state, conf, depth, intrinsics, camera_to_world, seat_hands,
                                                             top, at_pose, record_extra)
            # the seat's own state from this frame decides; the readings above cross-check it
            state, conf, fit_note = self._consult_seat_fit(state, conf, band_state, band_conf, depth, intrinsics,
                                                           camera_to_world, seat_hands, registration, record_extra)
            seat_note = "; ".join(n for n in (fit_note, seat_note and f"vs its closed frame: {seat_note}") if n)
        else:
            closed_tol = float(self._p("gap_lid_closed_tol_m"))
            open_m = float(self._p("gap_lid_open_m"))
            # Structural cues over the whole frame (not the centre ROI alone)
            wr0, wr1, wc0, wc1 = [float(v) for v in self._p("gap_lid_wide_roi")]
            wrows = slice(int(wr0 * h), max(int(wr0 * h) + 1, int(wr1 * h)))
            wcols = slice(int(wc0 * w), max(int(wc0 * w) + 1, int(wc1 * w)))
            wz = depth[wrows, wcols]
            wvs, wus = np.mgrid[wrows, wcols]
            wvalid = np.isfinite(wz) & (wz > 0.05) & (wz < 3.0)
            # THE HANDS ARE NOT THE LID (hardware 2026-10-09 20:23: the left hand at the arc end,
            # 13 % of the top rows within 0.35 m, was taken for a standing lid while the lid
            # itself read its closed height, and a missed lift PASSED). Every point within
            # gap_hand_mask_m of a hand frame is left out of both structural cues, as the
            # seat question already does; a frame the caller could not place is skipped and named.
            radius = float(self._p("gap_hand_mask_m"))
            hand_centres = []
            hands_missing = []
            lid_hands: dict = {}                  # hand frame -> its position (world): the fixture fit leaves them out
            for frame_name in hand_frames:
                if frame_name not in hands:
                    hands_missing.append(frame_name)
                else:
                    hand_centres.append(hands[frame_name][:, None])
                    lid_hands[frame_name] = hands[frame_name]
            record_extra["hands"] = {k: [round(float(c), 4) for c in v] for k, v in lid_hands.items()}
            M = np.asarray(camera_to_world, dtype=np.float64)

            def not_hand(d, u, v):
                """True where the pixel (u, v, depth d) is farther than the mask radius from every hand frame."""
                pts = np.stack([(u - intrinsics.cx) * d / intrinsics.fx, (v - intrinsics.cy) * d / intrinsics.fy, d, np.ones_like(d)])
                world_pts = M @ pts
                keep_px = np.ones(d.shape, dtype=bool)
                for centre in hand_centres:
                    keep_px &= np.linalg.norm(world_pts[:3] - centre, axis=0) > radius
                return keep_px, world_pts[2]

            hole_frac = 0.0
            if wvalid.any():
                wd = wz[wvalid]
                wu = wus[wvalid].astype(np.float64); wv = wvs[wvalid].astype(np.float64)
                wkeep, wz_world = not_hand(wd, wu, wv)
                deep = (wz_world <= top - float(self._p("gap_lid_deep_m"))) \
                    & (wz_world >= float(self._p("gap_lid_bowl_floor_m"))) & wkeep
                hole_frac = float(np.mean(deep))
            trows = slice(0, max(1, int(float(self._p("gap_lid_top_rows")) * h)))
            tz = depth[trows, :]
            tvs, tus = np.mgrid[trows, 0:w]
            tvalid = np.isfinite(tz) & (tz > 0.05) & (tz < 3.0)
            standing_frac = 0.0
            if tvalid.any():
                td = tz[tvalid]
                tkeep, _ = not_hand(td, tus[tvalid].astype(np.float64), tvs[tvalid].astype(np.float64))
                standing_frac = float(np.mean((td < float(self._p("gap_lid_standing_range_m"))) & tkeep))
            opening_visible = hole_frac >= float(self._p("gap_lid_hole_min"))
            # A lid standing near the camera can only be claimed when the centre ROI does
            # NOT read a lid at its closed height: a surface at the closed top under the
            # camera is the closed lid, whatever sits in the top rows (a hand, the pole).
            standing = standing_frac >= float(self._p("gap_lid_standing_min")) \
                and abs(rise) > float(self._p("gap_register_m"))
            if lift <= rise <= float(self._p("gap_lid_lifted_max_m")):
                state, conf = "lid_lifted", float(np.clip(rise / (2.0 * lift), 0.0, 1.0))
            elif opening_visible or standing or rise <= -open_m:
                state, absent = "lid_open", True
                conf = float(np.clip(max(hole_frac / (2.0 * float(self._p("gap_lid_hole_min"))),
                                        standing_frac / (2.0 * float(self._p("gap_lid_standing_min"))),
                                        -rise / (2.0 * open_m)), 0.0, 1.0))
            elif abs(height - nominal_top) <= float(self._p("gap_register_m")):
                # The closed lid, and it teaches the placement: a lid this close to its
                # nominal closed top with no opening and no standing lid is closed, and
                # how far it sits from the nominal is where the fixture stands this
                # session (hardware 2026-10-09: +21 mm). Stored under the fused path's
                # key so both paths read one value; replaced, not averaged, when it moves.
                learned = height - nominal_top
                if abs(learned - registration) > self.REGISTRATION_RESET_M:
                    self._learn_gap_registration(at_pose, learned, height, nominal_top)
                    registration = learned
                    top = nominal_top + registration
                    rise = height - top
                state, conf = "lid_closed", float(np.clip(1.0 - abs(rise) / closed_tol, 0.0, 1.0))
            else:
                # between the classes: say so, and let no expectation match it
                state, conf = "lid_between", 0.0
            detail = (f"; opening visible {hole_frac * 100:.0f} % of the wide ROI, standing-lid pixels "
                      f"{standing_frac * 100:.0f} % of the top rows (hands masked around {len(hand_centres)} frames"
                      + (f", {len(hands_missing)} missing" if hands_missing else "") + ")")
            # the lid's own state from the whole fixture decides; the centre-ROI reading above cross-checks it
            state, conf, lid_note = self._consult_lid_fit(state, conf, depth, intrinsics, camera_to_world, lid_hands,
                                                          registration, record_extra)
            if lid_note:
                detail += f"; lid: {lid_note}"
        # THE MODELS' ANSWER beside the geometry's (Newton, 2026-10-08): the colour model
        # and the registered depth state on the same moment's colour frame and aligned
        # depth. A closed/open class the fused state confidently contradicts is not a
        # proof of anything: "uncertain", which no expectation matches.
        cross = self._models_verdict(state, models, at_pose) \
            if (bool(self._p("gap_cross_check")) and models is not None) else {}
        if cross.get("contradiction"):
            self.log.warn(f"gap_check: {state} contradicted by the models ({cross['fused_state']} from "
                          f"{cross['state_source']} at {cross['confidence']:.2f}) -> uncertain")
            state, conf = "uncertain", 0.0
        # The session state the seat comparison lives on: every lid decision is remembered (a seat
        # reference is filled only after lid_open), and a seat_closed frame both readings agree on
        # becomes (part of) the closed reference for this camera pose.
        if part == "lid":
            self._note_lid_decision(at_pose, state)
        elif band_state == "seat_closed" and band_conf >= 0.5 and state == "seat_closed":
            self._fill_seat_reference(at_pose, depth, intrinsics, camera_to_world, seat_hands, top, record_extra)
        median_range = float(np.median(d)) if d.size else float("nan")
        if state == "refused":
            # Neither reading could judge: a refusal that says so, never "NOT lifted" (Newton 2026-10-23:
            # a check that cannot judge stops for a person, it does not start a re-lift).
            text = seat_refusal + (f"; seat: {seat_note}" if seat_note else "") + f"; {source}, {frame}{detail}"
            self.log.warn(f"gap_check: {text}")
            return GapDecision(part=part, expect=asked, success=False, state="refused", confidence=0.0,
                               recommendation="re-clean", message=text,
                               record=keep({"class": "refused", "success": False, "message": text, "part": part, "closed_top_m": top,
                                            "nominal_top_m": nominal_top, "registration_m": registration,
                                            "registration_source": registration_source, "registration_pose": at_pose,
                                            "roi_valid": coverage, "median_range_m": median_range}))
        expected = asked
        lifted = state == expected or (expected in ("lid_open",) and state == "lid_lifted") \
            or (expected in ("seat_open",) and state == "seat_lifted")
        if height is not None:
            measured = (f"{part} surface median z {height:.3f} m in {self.world_frame} "
                        f"(closed top {top:.3f}" + (f" = nominal {nominal_top:.3f} registered {registration * 1000:+.0f} mm by the {registration_source}" if abs(registration) > 1e-6 else "")
                        + f", rise {rise * 1000:.0f} mm vs {lift * 1000:.0f} mm bar"
                        + (f"; ABSENT from its closed place" + (", the rim shows" if part == "seat" else ", the bowl shows") if absent else "") + "); ")
        else:
            measured = f"{part}: the band reading {seat_refusal} (closed top {top:.3f}); "
        message = (measured
                   + f"{coverage * 100:.0f} % of the ROI valid, median range {median_range:.3f} m; "
                   f"{source}, {frame}{detail}"
                   + (f"; seat: {seat_note}" if seat_note else "")
                   + (f"; models: {cross.get('fused_state') or 'no state'} from {cross.get('state_source') or '-'} "
                      f"at {cross.get('confidence', 0.0):.2f}" + (" — CONTRADICTS the geometry: uncertain" if cross.get("contradiction") else "")
                      if cross else "")
                   + (f"; colour: {color_note}" if color_note else ""))
        self.log.info(f"gap_check: {message} -> {state}")
        return GapDecision(part=part, expect=asked, success=True, state=state, confidence=conf,
                           recommendation="pass" if lifted else "re-clean", message=message,
                           record=keep({"class": state, "success": True, "confidence": conf, "passes_expectation": bool(lifted),
                                        "part": part, "height_m": height, "closed_top_m": top, "nominal_top_m": nominal_top,
                                        "color_used": bool(color_used), "color_note": color_note,
                                        "registration_m": registration, "registration_source": registration_source,
                                        "registration_pose": at_pose, "rise_m": rise, "roi_valid": coverage,
                                        "median_range_m": median_range, "message": message, "models": cross}))

    def _seat_rise_params(self):
        from alix_perception_core.seat_rise import SeatRiseParams
        p = self._p
        return SeatRiseParams(
            ring_band_m=float(p("gap_seat_ref_ring_m")), footprint_margin_m=float(p("gap_seat_ref_margin_m")),
            region_m=tuple(float(v) for v in p("gap_seat_rise_region_m")), hand_rise_m=float(p("gap_seat_rise_hand_m")),
            hand_mask_m=float(p("gap_hand_mask_m")), min_pixels=int(p("gap_seat_rise_min_px")),
            closed_change_m=float(p("gap_seat_rise_closed_m")), closed_frac=float(p("gap_seat_rise_closed_frac")),
            moved_m=float(p("gap_seat_rise_moved_m")), moved_frac=float(p("gap_seat_rise_moved_frac")),
            min_deg=float(p("gap_seat_lift_min_deg")), hinge_tol_m=float(p("gap_seat_rise_hinge_tol_m")),
            down_frac=float(p("gap_seat_rise_down_frac")), max_age_s=float(p("gap_seat_ref_max_age_s")),
            lid_min_z_m=float(p("gap_seat_ref_lid_z")), lid_unchanged_m=float(p("gap_seat_ref_lid_m")),
            lid_unchanged_frac=float(p("gap_seat_ref_lid_frac")), lid_min_pixels=int(p("gap_seat_ref_lid_min_px")),
            max_frames=int(p("gap_seat_ref_frames")), replace_m=float(p("gap_seat_ref_replace_m")))

    def _seat_geometry(self):
        """The seat's hinge and footprint in world_frame: from the fixture meshes and pose (the seat's
        cells placed closed, calibration applied, its joint from scene.yaml), else the fixed hinge and
        xy box parameters. Built once; the fixture pose is fixed for the node's life."""
        if self._seat_geo is not None:
            return self._seat_geo
        from alix_perception_core import seat_rise as sr
        locator = self.locator()
        geo, source = None, ""
        if locator is not None and self.fixture_in_world() is not None and "seat" in getattr(locator, "fields", {}):
            try:
                import yaml
                from alix_perception_core.sdg.export import part_transforms
                with open(locator.scene_yaml) as handle:
                    joint = (((yaml.safe_load(handle) or {}).get("fixture") or {}).get("seat") or {}).get("joint") or {}
                rotation, translation = part_transforms(locator.scene_yaml, "closed")["seat"]
                cells = locator.fields["seat"].centers @ rotation.T + translation
                geo = sr.SeatGeometry.from_fixture(self.fixture_in_world(), joint.get("origin") or [0.0, 0.0, 0.0],
                                                   joint.get("axis") or [0.0, 1.0, 0.0], cells)
                source = f"the seat mesh ({len(cells)} cells) and its joint in {locator.scene_yaml}"
            except Exception as exc:                                   # noqa: BLE001 — the fixed geometry answers
                self.log.warn(f"seat geometry from the meshes failed ({exc}); using gap_seat_rise_hinge / gap_seat_ref_box")
        if geo is None:
            geo = sr.SeatGeometry.from_fixture(np.eye(4), [float(v) for v in self._p("gap_seat_rise_hinge")], [0.0, 1.0, 0.0],
                                               None, box_xy=tuple(float(v) for v in self._p("gap_seat_ref_box")))
            source = "gap_seat_rise_hinge and gap_seat_ref_box (no fixture meshes)"
        self.log.info(f"seat comparison: hinge {np.round(geo.hinge, 3).tolist()} in {self.world_frame}, seat towards "
                               f"{np.round(geo.toward_seat, 3).tolist()}, from {source}")
        self._seat_geo = geo
        return geo

    def _consult_seat_rise(self, band_state, band_conf, depth, intrinsics, camera_to_world, hands, seat_top, at_pose,
                           record_extra) -> tuple:
        """-> (state, confidence, note): the band reading combined with the comparison against the
        closed frame. Band refused -> the comparison's answer; both agree -> kept (the surer of the
        two confidences); conflict -> "uncertain"; no comparison answer -> the band reading stands."""
        if not bool(self._p("gap_seat_rise_enabled")):
            return band_state, band_conf, ""
        from alix_perception_core import seat_rise as sr
        ref = self._seat_refs.get(self._registration_key(at_pose))
        try:
            reading = sr.read_seat(ref, depth, camera_to_world, intrinsics, self._seat_geometry(), float(self.now()),
                                   hands=list(hands.values()), params=self._seat_rise_params())
        except Exception as exc:                                       # noqa: BLE001 — the band reading answers alone
            self.log.warn(f"gap_check: the seat comparison failed ({exc}); the band reading stands")
            record_extra["seat_rise"] = {"error": str(exc), "band_reading": band_state}
            return band_state, band_conf, f"failed ({exc})"
        info = reading.as_dict()
        info["reference"] = ref.summary() if ref is not None else None
        new = reading.state
        if not new:
            state, conf = band_state, band_conf
            note = f"no answer ({reading.reason})"
        elif band_state == "refused":
            state, conf = new, float(reading.confidence)
            note = f"{new} at {conf:.2f} against the closed frame ({reading.reason})"
        elif band_state == new:
            state, conf = band_state, max(float(band_conf), float(reading.confidence))
            note = f"agrees: {new} ({reading.reason})"
        else:
            state, conf = "uncertain", 0.0
            note = f"{new} CONTRADICTS the band reading's {band_state} -> uncertain ({reading.reason})"
            self.log.warn(f"gap_check: the seat against its closed frame says {new}, the band says {band_state} -> uncertain")
        info.update({"band_reading": band_state, "outcome": state})
        record_extra["seat_rise"] = info
        return state, conf, note

    def _hinged_fixture(self):
        """The fixture model's body and hinged parts (calibrated cells + scene.yaml joints), or None
        without mesh routing. Built once."""
        if self._hinged is None:
            locator = self.locator()
            if locator is None or self.fixture_in_world() is None:
                return None
            from alix_perception_core.seat_state import HingedFixture
            try:
                self._hinged = HingedFixture.from_locator(locator)
            except Exception as exc:                                   # noqa: BLE001 — the other readings answer
                self.log.warn(f"gap_check: no hinged fixture model ({exc}); the seat is read without its own fit")
                self._hinged = False
        return self._hinged or None

    def _consult_seat_fit(self, state, conf, band_state, band_conf, depth, intrinsics, camera_to_world, hands,
                          registration, record_extra) -> tuple:
        """-> (state, confidence, note): the seat's own state from this frame (seat_state.fit_seat_state)
        decides when it answers; the band reading or the closed-frame comparison confidently saying the
        opposite makes it "uncertain". No answer from it: the earlier readings stand as combined."""
        if not bool(self._p("gap_seat_fit_enabled")):
            return state, conf, ""
        fixture = self._hinged_fixture()
        if fixture is None:
            return state, conf, ""
        try:
            whole = self._fixture_fit(depth, intrinsics, camera_to_world, fixture, registration, hands)
        except Exception as exc:                                       # noqa: BLE001 — the other readings answer
            self.log.warn(f"gap_check: the seat fit failed ({exc}); the other readings stand")
            record_extra["seat_fit"] = {"error": str(exc)}
            return state, conf, f"fit failed ({exc})"
        fit = whole.seat
        info = whole.as_dict()
        if not fit.state:
            info["outcome"] = state
            record_extra["seat_fit"] = info
            return state, conf, f"its own fit gives no answer ({fit.reason})"
        rise = record_extra.get("seat_rise") or {}
        others = [("the band reading", band_state, band_conf), ("its closed frame", rise.get("state"), rise.get("confidence") or 0.0)]
        against = [f"{name} says {s} at {c:.2f}" for name, s, c in others
                   if s in ("seat_closed", "seat_lifted") and float(c) >= 0.5 and s != fit.state]
        if against:
            out_state, out_conf = "uncertain", 0.0
            note = f"its own fit says {fit.state} ({fit.reason}) but {' and '.join(against)} -> uncertain"
            self.log.warn(f"gap_check: seat {note}")
        else:
            agree = [float(c) for _n, s, c in others if s == fit.state]
            out_state, out_conf = fit.state, max([float(fit.confidence)] + agree)
            note = f"{fit.state} at {out_conf:.2f} from its own fit ({fit.reason})"
        info["outcome"] = out_state
        record_extra["seat_fit"] = info
        return out_state, out_conf, note

    def _fixture_fit(self, depth, intrinsics, camera_to_world, fixture, registration, hands):
        """seat_state.fit_fixture_state with this node's parameters: the lid's and the seat's state from one fit (the
        renders made once per camera pose, so a check's later frames and later checks from the same pose reuse them)."""
        from alix_perception_core import seat_state as sst
        params = sst.SeatFitParams(stride=int(self._p("gap_seat_fit_stride")), step_deg=float(self._p("gap_seat_fit_step_deg")),
                                   tol_m=float(self._p("gap_seat_fit_tol_m")), min_zone_px=int(self._p("gap_seat_fit_min_px")),
                                   min_margin=float(self._p("gap_seat_fit_margin")), min_deg=float(self._p("gap_seat_lift_min_deg")),
                                   hand_mask_m=float(self._p("gap_hand_mask_m")), lid_min_deg=float(self._p("gap_lid_fit_min_deg")),
                                   lid_open_min_deg=float(self._p("gap_lid_fit_open_min_deg")),
                                   height_search_m=float(self._p("gap_fit_height_search_m")),
                                   height_step_m=float(self._p("gap_fit_height_step_m")))
        return sst.fit_fixture_state(depth, intrinsics, camera_to_world, self.fixture_in_world(), fixture,
                                     registration_m=float(registration), hands=list(hands.values()), params=params)

    def _consult_lid_fit(self, state, conf, depth, intrinsics, camera_to_world, hands, registration, record_extra) -> tuple:
        """-> (state, confidence, note): the lid's state from the whole fixture in this frame decides when the fit
        answers; the centre-ROI reading confidently on the other side (closed against lifted or open) makes it
        "uncertain". No answer from the fit: the ROI reading stands."""
        if not bool(self._p("gap_lid_fit_enabled")):
            return state, conf, ""
        fixture = self._hinged_fixture()
        if fixture is None:
            return state, conf, ""
        try:
            whole = self._fixture_fit(depth, intrinsics, camera_to_world, fixture, registration, hands)
        except Exception as exc:                                       # noqa: BLE001 — the ROI reading answers
            self.log.warn(f"gap_check: the lid fit failed ({exc}); the ROI reading stands")
            record_extra["lid_fit"] = {"error": str(exc)}
            return state, conf, f"fit failed ({exc})"
        fit = whole.lid
        info = whole.as_dict()
        if not fit.state:
            info["outcome"] = state
            record_extra["lid_fit"] = info
            return state, conf, f"the fixture fit gives no answer ({fit.reason})"
        closed = lambda s: s == "lid_closed"
        if state in ("lid_closed", "lid_lifted", "lid_open") and float(conf) >= 0.5 and closed(state) != closed(fit.state):
            out_state, out_conf = "uncertain", 0.0
            note = f"the fixture fit says {fit.state} ({fit.reason}) but the ROI reading says {state} at {conf:.2f} -> uncertain"
            self.log.warn(f"gap_check: lid {note}")
        else:
            out_state = fit.state
            out_conf = max(float(fit.confidence), float(conf) if state == fit.state else 0.0)
            note = f"{fit.state} at {out_conf:.2f} from the fixture fit ({fit.reason})" + (
                f"; the ROI reading: {state}" if state != fit.state else "")
        info["outcome"] = out_state
        record_extra["lid_fit"] = info
        return out_state, out_conf, note

    def _fill_seat_reference(self, at_pose, depth, intrinsics, camera_to_world, hands, seat_top, record_extra) -> None:
        """A seat_closed frame joins (or replaces) the closed reference of this placement, from wherever
        the camera is — only when the last lid decision at this placement was lid_open: the reference
        carries the open lid's points for the lid guard, and a seat_closed under a closed lid has none."""
        if not bool(self._p("gap_seat_rise_enabled")):
            return
        from alix_perception_core import seat_rise as sr
        key = self._registration_key(at_pose)
        info = record_extra.setdefault("seat_rise", {})
        last = self._last_lid.get(key)
        if last != "lid_open":
            info["reference_update"] = f"not filled: the last lid decision here was {last or 'none'}, not lid_open"
            return
        ref, action, why = sr.update_reference(self._seat_refs.get(key), depth, camera_to_world, intrinsics, seat_top,
                                               self._seat_geometry(), float(self.now()), hands=list(hands.values()),
                                               params=self._seat_rise_params())
        if action != "refused":
            self._seat_refs[key] = ref
        info["reference_update"] = f"{action}: {why}"
        self.log.info(f"seat closed reference at {at_pose or '(any pose)'}: {action} — {why}")

    def _clear_seat_references(self, at_pose, why: str) -> None:
        if self._seat_refs.pop(self._registration_key(at_pose), None):
            self.log.info(f"seat closed reference at {at_pose or '(any pose)'} cleared: {why}")

    def _note_lid_decision(self, at_pose, state: str) -> None:
        """Remember the lid class decided at this placement; lid_closed / lid_lifted / lid_between
        clear the seat's closed reference (the lid it was guarded by is no longer standing open)."""
        self._last_lid[self._registration_key(at_pose)] = state
        if state in self._SEAT_REF_CLEARING_LID:
            self._clear_seat_references(at_pose, f"lid decision {state}")

    def _registration_key(self, pose: str) -> tuple:
        """A registration belongs to a pose AND a placement: the fixture (scene) and
        the UGV standoff the pose was nominal for. Change either and it is a new key."""
        return (pose or "", str(self._p("fixture_scene_yaml") or ""), round(float(self._p("ugv_standoff_m")), 3))

    def _registration_for(self, pose: str) -> float:
        """The session's registration (metres, body +z) learned at `pose`; 0 until a closed decision taught it."""
        return float(self._registration.get(self._registration_key(pose), 0.0))

    #: a closed decision this far from the stored registration means the robot or the
    #: fixture MOVED: the stored value is replaced, not averaged (MEASURED 2026-10-08:
    #: the registration varies by < 2 mm across pole heights at one placement)
    REGISTRATION_RESET_M = 0.006

    def _learn_gap_registration(self, pose: str, value: float, height: float, nominal_top: float) -> None:
        """The gap check's own placement lesson: the closed lid read `height` against the
        nominal `nominal_top`, so the fixture sits `value` higher (metres, body +z) at
        `pose`. Same key and semantics as the fused path's registration."""
        if not bool(self._p("depth_registration_learn")):
            return
        key = self._registration_key(pose)
        old = self._registration.get(key)
        self._registration[key] = float(value)
        self.log.warn(
            f"gap registration at {pose or '(any pose)'}: {value * 1000:+.1f} mm (the closed lid read {height:.3f} m, nominal "
            f"{nominal_top:.3f})" + (f" — replaces {old * 1000:+.1f} mm" if old is not None else "")
            + "; every later lid/seat top at this placement carries it")
        self._clear_seat_references(pose, "the registration was reset")

    def _learn_registration(self, pose: str, result) -> None:
        """Keep a confident CLOSED decision's offset as the registration at `pose`:
        half-way towards it when one exists, REPLACED when it jumped (a new placement)."""
        if not bool(self._p("depth_registration_learn")) or result.fixture_state != "closed" \
                or result.state_source not in ("depth", "depth+colour"):
            return
        offset_mm = result.diagnostics.get("depth_offset_mm")
        if offset_mm is None:
            return
        key = self._registration_key(pose)
        new = float(offset_mm) / 1000.0
        old = self._registration.get(key)
        if old is None:
            value, why = new, "learned"
        elif abs(new - old) > self.REGISTRATION_RESET_M:
            value, why = new, f"RESET (jumped from {old * 1000:+.1f} mm: a new placement)"
            self._clear_seat_references(pose, "the registration was reset (a new placement)")
        else:
            value, why = 0.5 * (old + new), "updated"
        self._registration[key] = value
        if old is None or abs(value - old) > 0.002:
            self.log.info(f"depth registration at {pose or '(any pose)'}: {value * 1000:+.1f} mm "
                                   f"({why}; closed decision at {offset_mm:+.1f} mm)")

