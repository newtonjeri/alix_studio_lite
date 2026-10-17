"""The lid and seat checks with the wrist D435, without ROS: ALIX's gap check, ported.

ALIX's lid and seat plans (vendor/alix/Resource/plans/commode_c/hardware) carry `check_fixture_state`
actions with `check: fixture_gap`: before a plan moves, and after the left arm's lift, the right wrist's
D435 is asked what it sees. On the robot the question goes to ROS (`/perception/analyze_scene`, mode
`gap_check`, ALIX's ros/perception/alix_perception/alix_perception/perception_node.py, `_gap_check`
at lines 851-1159, commit 842c0e59). Lite has no ROS (the owner, 18 Oct: "Lite should be ROS 2
independent"), so this is that method ported line for line, with what ROS gave it given here instead:

- the depth: the D435's NATIVE depth stream (848 x 480, z16 mm), as the node's default
  `gap_depth_source: depth`, and its own intrinsics (camera_d435.py, pyrealsense2);
- the camera's pose in base_link: forward kinematics on ALIX's URDF with the arm's, the pole's and the
  hands' live readings (robot_fk.py; the node had it from TF, which robot_state_publisher computes on
  the same URDF: on 12 Oct at pose 4 the two agree within 0.3 mm and 0.08 degrees);
- the hands' 17 frames, masked out of the depth as the node does (same radius, same frames).

Same parameters (the vendored perception.yaml, read here, not copied), same regions, same classes,
same registration (learned from a closed lid, and the seat's own), same refusals (fail-closed), same
messages, and the same record of each check (its decision, the depth it used, a picture) in Lite's
logs, Resources/Logs/<day>/checks/.

Not ported: the cross-check with the colour models (`gap_cross_check`, lines 1162-1190). It needs
ALIX's models and ultralytics, which Lite does not install; on the robot it never contributed either
(every check of 10 and 12 Oct records no fused state: the geometry answered alone, as it does here).

The executive's side (what proves an expectation, what a failure asks for) is ALIX's lift_check.py,
vendored as it is and used by mechanism.py.
"""
from __future__ import annotations

import json
import struct
import threading
import time
import zlib
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable

import numpy as np

from .. import vendor_paths

PARAMS_FILE = vendor_paths.ALIX / "ros" / "perception" / "alix_perception" / "config" / "perception.yaml"
WORLD_FRAME = "base_link"
DEPTH_FRAME = "camera_depth_optical_frame"

#: the 17 parameters the node writes into each record (perception_node.py lines 911-918)
RECORD_PARAMS = ("gap_lid_top_z", "gap_seat_top_z", "gap_lift_m", "gap_roi", "gap_seat_roi", "gap_seat_band_z",
                 "gap_seat_min_band", "gap_seat_absent_m", "gap_hand_mask_m", "gap_lid_closed_tol_m", "gap_lid_open_m",
                 "gap_lid_wide_roi", "gap_lid_deep_m", "gap_lid_hole_min", "gap_lid_top_rows",
                 "gap_lid_standing_range_m", "gap_lid_standing_min")


def load_params(path: Path = PARAMS_FILE) -> dict:
    """The perception node's parameters, as ALIX's bringup gives them (``perception_node.ros__parameters``)."""
    import yaml

    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return dict((data.get("perception_node") or {}).get("ros__parameters") or {})


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole intrinsics of one stream, in pixels (alix_perception_core's ``Intrinsics``)."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def scaled_to(self, width: int, height: int) -> "Intrinsics":
        sx, sy = float(width) / float(self.width), float(height) / float(self.height)
        return Intrinsics(self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy, int(width), int(height))

    @classmethod
    def from_fov(cls, width: int, height: int, hfov_deg: float = 87.0, vfov_deg: float = 58.0) -> "Intrinsics":
        """The D435 depth stream's nominal intrinsics from its specified field of view (ALIX's d435.yaml:
        87 x 58 degrees, +-3): for replaying a record, which carries none, as ALIX's own replay tools do."""
        import math

        return cls(width / 2.0 / math.tan(math.radians(hfov_deg) / 2.0), height / 2.0 / math.tan(math.radians(vfov_deg) / 2.0),
                   width / 2.0, height / 2.0, int(width), int(height))


@dataclass
class DepthFrame:
    """What one check looks at: the depth image and where the camera and the hands were."""
    depth: np.ndarray                          # metres (float) or the camera's z16 millimetres (uint16)
    intrinsics: Intrinsics | None
    cam_to_base: np.ndarray | None             # 4 x 4, base_link <- the depth optical frame
    hands: dict | None = None                  # {hand frame: (x, y, z) in base_link}; a frame absent is "missing"
    colour: np.ndarray | None = None           # H x W x 3, BGR, the same moment's colour frame (kept in the record)
    source: str = "native depth (D435, pyrealsense2)"
    frame: str = DEPTH_FRAME
    stamp_s: float | None = None               # the frame's own time (the camera's clock), when known
    age_s: float | None = None                 # how old the frame was when the check used it
    colour_age_s: float | None = None
    colour_note: str = ""


@dataclass
class Answer:
    """The node's AnalyzeScene response, as the executive reads it (lift_check.evaluate_fixture_check)."""
    success: bool = False
    fixture_type: str = "commode"
    fixture_state: str = "not_lifted"
    state_confidence: float = 0.0
    recommendation: str = "re-clean"
    message: str = ""
    record: dict = field(default_factory=dict)
    record_path: str | None = None


class GapCheck:
    """One camera's gap check, with the session's registration (the node's ``_registration``)."""

    #: a closed decision this far from the stored registration means the robot or the fixture MOVED:
    #: the stored value is replaced, not averaged (perception_node.py line 1331)
    REGISTRATION_RESET_M = 0.006

    def __init__(self, params: dict | None = None, records: Path | Callable[[], Path] | None = None,
                 log: Callable[[str], None] | None = None):
        self.p = dict(params if params is not None else load_params())
        self._records = records
        self._log = log or (lambda _text: None)
        self._registration: dict = {}          # (named pose, fixture key) -> metres (body +z)
        self._lock = threading.Lock()          # one check at a time: they share the registration

    # ── the registration (perception_node.py lines 1319-1345)
    def registration_key(self, pose: str) -> tuple:
        """A registration belongs to a pose AND a placement: the fixture (scene) and the UGV standoff."""
        return (pose or "", str(self.p.get("fixture_scene_yaml") or ""), round(float(self.p.get("ugv_standoff_m", -1.0)), 3))

    def registration_for(self, pose: str) -> float:
        return float(self._registration.get(self.registration_key(pose), 0.0))

    def registration_view(self) -> dict:
        """What the session has learned, for the page and the run's log: lid and seat, in millimetres."""
        with self._lock:
            lid = self._registration.get(self.registration_key(""))
            seat = self._registration.get(self.registration_key("") + ("seat",))
        return dict(lid_mm=None if lid is None else round(lid * 1000.0, 1), seat_mm=None if seat is None else round(seat * 1000.0, 1))

    def forget(self) -> None:
        """A new placement (the UGV moved): learned again from the next closed lid and seat."""
        with self._lock:
            self._registration.clear()

    def _learn_gap_registration(self, pose: str, value: float, height: float, nominal_top: float) -> None:
        if not bool(self.p.get("depth_registration_learn", True)):
            return
        key = self.registration_key(pose)
        old = self._registration.get(key)
        self._registration[key] = float(value)
        self._log(f"gap registration at {pose or '(any pose)'}: {value * 1000:+.1f} mm (the closed lid read {height:.3f} m, nominal "
                  f"{nominal_top:.3f})" + (f" — replaces {old * 1000:+.1f} mm" if old is not None else "")
                  + "; every later lid/seat top at this placement carries it")

    # ── the check (perception_node.py `_gap_check`, lines 851-1159)
    def ask(self, expect: str, frame: DepthFrame | None, at_pose: str = "") -> Answer:
        with self._lock:
            return self._ask(str(expect or ""), frame, at_pose)

    def _ask(self, expect: str, frame: DepthFrame | None, at_pose: str) -> Answer:
        p = self.p
        mode = "gap_check"
        part = "seat" if "seat" in expect.lower() else "lid"
        response = Answer()
        source = frame.source if frame is not None else "native depth (D435)"
        frame_name = frame.frame if frame is not None else DEPTH_FRAME
        roll_tf = None if frame is None else frame.cam_to_base
        rois_for_record = {"lid_roi": [float(v) for v in p["gap_roi"]], "seat_roi": [float(v) for v in p["gap_seat_roi"]],
                           "lid_wide_roi": [float(v) for v in p["gap_lid_wide_roi"]]}
        record_depth = {"arr": None}
        part_expect = expect

        def keep(decision):                    # the node's `keep` (lines 888-922): the record of this decision
            if not bool(p.get("save_check_frames", True)) or self._records is None:
                return
            decision = dict(decision)
            decision.update({
                "decision_wall_s": round(time.time(), 3),
                "depth_header_stamp_s": None if frame is None or frame.stamp_s is None else round(frame.stamp_s, 6),
                "depth_age_s": None if frame is None or frame.age_s is None else round(frame.age_s, 3),
                "color_age_s": None if frame is None or frame.colour_age_s is None else round(frame.colour_age_s, 3),
                "depth_frame_id": frame_name,
            })
            decision.update({"source": source, "frame": frame_name, "world_frame": WORLD_FRAME,
                             "camera_to_world": None if roll_tf is None else np.asarray(roll_tf, dtype=float).round(6).tolist(),
                             "intrinsics": None if frame is None or frame.intrinsics is None else frame.intrinsics.__dict__,
                             "hands": None if frame is None or frame.hands is None
                             else {k: [round(float(c), 4) for c in v] for k, v in frame.hands.items()},
                             "params": {k: p.get(k) for k in RECORD_PARAMS}})
            root = self._records() if callable(self._records) else self._records
            written = write_check_record(root, mode, part_expect, decision, record_depth["arr"],
                                         None if frame is None else frame.colour, rois_for_record)
            if written:
                response.record_path = str(written)
                self._log(f"check record: {written}")
            response.record = decision

        def refuse(text):
            response.success = False
            response.message = text
            self._log(f"gap_check: {text}")
            keep({"class": "refused", "success": False, "message": text})
            return response

        if frame is None or frame.depth is None:
            return refuse(f"no depth frame ({source}; nothing received) -> NOT lifted")
        if roll_tf is None:
            return refuse(f"no TF {frame_name} -> {WORLD_FRAME} -> NOT lifted")
        if frame.intrinsics is None:
            return refuse(f"no camera_info for the {source} -> NOT lifted")
        depth = np.asarray(frame.depth)
        if depth.ndim != 2:
            return refuse(f"a depth image of shape {depth.shape} is not a depth frame -> NOT lifted")
        depth = depth.astype(np.float64) * (0.001 if depth.dtype == np.uint16 else 1.0)
        record_depth["arr"] = depth
        h, w = depth.shape
        nominal_top = float(p["gap_seat_top_z"] if part == "seat" else p["gap_lid_top_z"])
        lid_registration = self.registration_for(at_pose)
        seat_key = self.registration_key(at_pose) + ("seat",)
        registration_source = "lid" if abs(lid_registration) > 1e-9 else "none"
        registration = lid_registration
        if part == "seat" and seat_key in self._registration:
            registration = float(self._registration[seat_key])
            registration_source = "seat"
        intrinsics = frame.intrinsics
        if (intrinsics.width, intrinsics.height) != (w, h):
            intrinsics = intrinsics.scaled_to(w, h)
        r0, r1, c0, c1 = [float(v) for v in p["gap_seat_roi" if part == "seat" else "gap_roi"]]
        rows = slice(int(r0 * h), max(int(r0 * h) + 1, int(r1 * h)))
        cols = slice(int(c0 * w), max(int(c0 * w) + 1, int(c1 * w)))
        z = depth[rows, cols]
        vs, us = np.mgrid[rows, cols]
        valid = np.isfinite(z) & (z > 0.05) & (z < 3.0)
        coverage = float(valid.mean()) if valid.size else 0.0
        if coverage < 0.3:
            return refuse(f"only {coverage * 100:.0f} % of the ROI carries depth -> NOT lifted")
        d = z[valid]
        u = us[valid].astype(np.float64)
        v = vs[valid].astype(np.float64)
        pts = np.stack([(u - intrinsics.cx) * d / intrinsics.fx, (v - intrinsics.cy) * d / intrinsics.fy, d, np.ones_like(d)])
        world = np.asarray(roll_tf, dtype=np.float64) @ pts
        z_world = world[2]
        hands = frame.hands or {}
        detail = ""
        absent = False
        band_frac = 0.0
        if part == "seat":
            # the hands masked: every point within gap_hand_mask_m of a hand frame (a frame not known is named)
            radius = float(p["gap_hand_mask_m"])
            unmasked = np.ones(z_world.shape, dtype=bool)
            masked_frames, missing = [], []
            for frame_hand in [str(f) for f in p["gap_hand_frames"]]:
                if frame_hand not in hands:
                    missing.append(frame_hand)
                    continue
                centre = np.asarray(hands[frame_hand], dtype=np.float64).reshape(3, 1)
                unmasked &= np.linalg.norm(world[:3] - centre, axis=0) > radius
                masked_frames.append(frame_hand)
            lo, hi = [float(x) + registration for x in p["gap_seat_band_z"]]   # the band follows the placement
            band = unmasked & (z_world >= lo) & (z_world <= hi)
            band_frac = float(band.mean()) if band.size else 0.0
            if band_frac < float(p["gap_seat_min_band"]):
                return refuse(f"only {band_frac * 100:.0f} % of the seat ROI lies in the {lo:.2f}-{hi:.2f} m "
                              f"seat band after masking the hands -> NOT lifted")
            zb = z_world[band]
            height = float(np.median(zb))
            q10, q90 = np.percentile(zb, [10, 90])
            seat_offset = height - nominal_top           # the seat's own placement, from the ring itself
            if band_frac >= float(p["gap_seat_register_min_band"]) \
                    and abs(seat_offset) <= float(p["gap_register_m"]) \
                    and abs(seat_offset - lid_registration) <= float(p["gap_seat_register_tol_m"]) \
                    and abs(seat_offset - registration) > self.REGISTRATION_RESET_M:
                old_reg = self._registration.get(seat_key)
                self._registration[seat_key] = float(seat_offset)
                self._log(f"seat registration at {at_pose or '(any pose)'}: {seat_offset * 1000:+.1f} mm (the ring read {height:.3f} m over "
                          f"{band_frac * 100:.0f} % of the ROI, nominal {nominal_top:.3f}; the lid's offset is {lid_registration * 1000:+.1f} mm)"
                          + (f" — replaces {old_reg * 1000:+.1f} mm" if old_reg is not None else ""))
                registration, registration_source = float(seat_offset), "seat"
            detail = (f"; seat band {band_frac * 100:.0f} % of the ROI, 10/90 % heights {q10:.3f}/{q90:.3f}, "
                      f"{(~unmasked).mean() * 100:.0f} % masked around {len(masked_frames)} hand frames"
                      + (f" ({len(missing)} missing: {missing[:3]})" if missing else ""))
        else:
            height = float(np.median(z_world))
        top = nominal_top + registration
        lift = float(p["gap_lift_m"])
        rise = height - top
        if not (0.1 < height < 1.5):
            return refuse(f"{part} surface at an implausible {height:.3f} m in {WORLD_FRAME} -> NOT lifted")
        if part == "seat":
            absent_m = float(p["gap_seat_absent_m"])
            # a thin band claims "absent" only when the reading is clearly rim-deep
            absent = rise <= -absent_m and (band_frac >= float(p["gap_seat_absent_min_band"]) or rise <= -2.0 * absent_m)
            state = "seat_lifted" if absent else "seat_closed"
            closed_tol = float(p["gap_seat_closed_tol_m"])
            conf = float(np.clip(abs(rise) / (2.0 * absent_m), 0.0, 1.0)) if absent \
                else float(np.clip(1.0 - max(0.0, -rise - closed_tol) / max(1e-6, absent_m - closed_tol), 0.0, 1.0))
        else:
            closed_tol = float(p["gap_lid_closed_tol_m"])
            open_m = float(p["gap_lid_open_m"])
            wr0, wr1, wc0, wc1 = [float(x) for x in p["gap_lid_wide_roi"]]
            wrows = slice(int(wr0 * h), max(int(wr0 * h) + 1, int(wr1 * h)))
            wcols = slice(int(wc0 * w), max(int(wc0 * w) + 1, int(wc1 * w)))
            wz = depth[wrows, wcols]
            wvs, wus = np.mgrid[wrows, wcols]
            wvalid = np.isfinite(wz) & (wz > 0.05) & (wz < 3.0)
            # THE HANDS ARE NOT THE LID (the robot, 9 Oct 20:23): left out of both structural cues
            radius = float(p["gap_hand_mask_m"])
            hand_centres, hands_missing = [], []
            for frame_hand in [str(f) for f in p["gap_hand_frames"]]:
                if frame_hand in hands:
                    hand_centres.append(np.asarray(hands[frame_hand], dtype=np.float64).reshape(3, 1))
                else:
                    hands_missing.append(frame_hand)
            M = np.asarray(roll_tf, dtype=np.float64)

            def not_hand(dd, uu, vv):
                ps = np.stack([(uu - intrinsics.cx) * dd / intrinsics.fx, (vv - intrinsics.cy) * dd / intrinsics.fy, dd, np.ones_like(dd)])
                world_pts = M @ ps
                keep_px = np.ones(dd.shape, dtype=bool)
                for centre in hand_centres:
                    keep_px &= np.linalg.norm(world_pts[:3] - centre, axis=0) > radius
                return keep_px, world_pts[2]

            hole_frac = 0.0
            if wvalid.any():
                wd = wz[wvalid]
                wu = wus[wvalid].astype(np.float64)
                wv = wvs[wvalid].astype(np.float64)
                wkeep, wz_world = not_hand(wd, wu, wv)
                deep = (wz_world <= top - float(p["gap_lid_deep_m"])) & (wz_world >= float(p["gap_lid_bowl_floor_m"])) & wkeep
                hole_frac = float(np.mean(deep))
            trows = slice(0, max(1, int(float(p["gap_lid_top_rows"]) * h)))
            tz = depth[trows, :]
            tvs, tus = np.mgrid[trows, 0:w]
            tvalid = np.isfinite(tz) & (tz > 0.05) & (tz < 3.0)
            standing_frac = 0.0
            if tvalid.any():
                td = tz[tvalid]
                tkeep, _ = not_hand(td, tus[tvalid].astype(np.float64), tvs[tvalid].astype(np.float64))
                standing_frac = float(np.mean((td < float(p["gap_lid_standing_range_m"])) & tkeep))
            opening_visible = hole_frac >= float(p["gap_lid_hole_min"])
            # a lid standing near the camera is claimed only when the centre does NOT read the closed height
            standing = standing_frac >= float(p["gap_lid_standing_min"]) and abs(rise) > float(p["gap_register_m"])
            if lift <= rise <= float(p["gap_lid_lifted_max_m"]):
                state, conf = "lid_lifted", float(np.clip(rise / (2.0 * lift), 0.0, 1.0))
            elif opening_visible or standing or rise <= -open_m:
                state, absent = "lid_open", True
                conf = float(np.clip(max(hole_frac / (2.0 * float(p["gap_lid_hole_min"])),
                                         standing_frac / (2.0 * float(p["gap_lid_standing_min"])),
                                         -rise / (2.0 * open_m)), 0.0, 1.0))
            elif abs(height - nominal_top) <= float(p["gap_register_m"]):
                # the closed lid, and it teaches the placement (replaced, not averaged, when it moves)
                learned = height - nominal_top
                if abs(learned - registration) > self.REGISTRATION_RESET_M:
                    self._learn_gap_registration(at_pose, learned, height, nominal_top)
                    registration = learned
                    top = nominal_top + registration
                    rise = height - top
                state, conf = "lid_closed", float(np.clip(1.0 - abs(rise) / closed_tol, 0.0, 1.0))
            else:
                state, conf = "lid_between", 0.0   # between the classes: no expectation matches it
            detail = (f"; opening visible {hole_frac * 100:.0f} % of the wide ROI, standing-lid pixels "
                      f"{standing_frac * 100:.0f} % of the top rows (hands masked around {len(hand_centres)} frames"
                      + (f", {len(hands_missing)} missing" if hands_missing else "") + ")")
        expected = expect
        lifted = state == expected or (expected in ("lid_open",) and state == "lid_lifted") \
            or (expected in ("seat_open",) and state == "seat_lifted")
        response.success = True
        response.fixture_state = state
        response.state_confidence = conf
        response.recommendation = "pass" if lifted else "re-clean"
        response.message = (f"{part} surface median z {height:.3f} m in {WORLD_FRAME} "
                            f"(closed top {top:.3f}" + (f" = nominal {nominal_top:.3f} registered {registration * 1000:+.0f} mm by the {registration_source}" if abs(registration) > 1e-6 else "")
                            + f", rise {rise * 1000:.0f} mm vs {lift * 1000:.0f} mm bar"
                            + ("; ABSENT from its closed place" + (", the rim shows" if part == "seat" else ", the bowl shows") if absent else "") + "); "
                            f"{coverage * 100:.0f} % of the ROI valid, median range {float(np.median(d)):.3f} m; "
                            f"{source}, {frame_name}{detail}; models: not installed (the geometry answers alone)"
                            + (f"; colour: {frame.colour_note}" if frame.colour_note else ""))
        self._log(f"gap_check: {response.message} -> {response.fixture_state}")
        keep({"class": state, "success": True, "confidence": conf, "passes_expectation": bool(lifted),
              "part": part, "height_m": height, "closed_top_m": top, "nominal_top_m": nominal_top,
              "color_used": frame.colour is not None, "color_note": frame.colour_note,
              "registration_m": registration, "registration_source": registration_source, "registration_pose": at_pose,
              "rise_m": rise, "roi_valid": coverage, "median_range_m": float(np.median(d)), "message": response.message,
              "models": {}})
        return response


# ── the record of a check (perception_node.py `write_check_record`, lines 71-129)
def write_check_record(root, mode: str, expect: str, decision: dict, depth=None, image=None,
                       rois: dict | None = None, stamp: str | None = None) -> Path | None:
    """``<root>/<stamp>_<mode>_<expect>_<class>.json`` (the decision), ``..._depth.npy`` (float32 metres, the array
    the check used), ``..._depth.png`` (0.25-0.75 m colour-mapped, the regions drawn) and ``..._color.png``. ALIX's
    names and contents; the pictures are drawn with numpy here (no OpenCV in Lite). Never fails a check."""
    try:
        out = Path(root)
        out.mkdir(parents=True, exist_ok=True)
        stamp = stamp or datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        cls = str(decision.get("class") or decision.get("fixture_state") or "refused")
        safe = lambda s: "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(s)) or "none"   # noqa: E731
        stem = f"{stamp}_{safe(mode)}_{safe(expect)}_{safe(cls)}"
        files = {}
        if depth is not None:
            arr = np.asarray(depth, dtype=np.float32)
            np.save(out / f"{stem}_depth.npy", arr)
            files["depth_npy"] = f"{stem}_depth.npy"
            try:
                (out / f"{stem}_depth.png").write_bytes(png_bytes(depth_view(arr, rois)))
                files["depth_png"] = f"{stem}_depth.png"
            except Exception:                                   # noqa: BLE001 - the npy is the record
                pass
        if image is not None:
            try:
                im = np.asarray(image)
                (out / f"{stem}_color.png").write_bytes(png_bytes(im[:, :, ::-1] if im.ndim == 3 else im))   # BGR -> RGB
                files["color_png"] = f"{stem}_color.png"
            except Exception:                                   # noqa: BLE001
                pass
        record = dict(decision)
        record.update({"stamp": stamp, "mode": mode, "expect": expect, "class": cls, "files": files})
        path = out / f"{stem}.json"
        path.write_text(json.dumps(record, indent=2, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
        return path
    except Exception:                                           # noqa: BLE001 - a record must never fail a check
        return None


def depth_view(arr: np.ndarray, rois: dict | None = None) -> np.ndarray:
    """The depth as the record's picture: 0.25 m red .. 0.75 m blue (OpenCV's JET, as ALIX draws it), no depth black,
    each region of interest outlined in white. H x W x 3 RGB, uint8."""
    v = np.asarray(arr, dtype=np.float64).copy()
    v[~np.isfinite(v)] = 0.0
    x = 1.0 - np.clip((v - 0.25) / 0.5, 0.0, 1.0)            # 1 near (red), 0 far (blue)
    rgb = np.stack([np.clip(1.5 - np.abs(4.0 * x - 3.0), 0, 1), np.clip(1.5 - np.abs(4.0 * x - 2.0), 0, 1),
                    np.clip(1.5 - np.abs(4.0 * x - 1.0), 0, 1)], axis=-1)
    img = (255 * rgb).astype(np.uint8)
    img[v <= 0.05] = 0
    h, w = v.shape
    for (r0, r1, c0, c1) in (rois or {}).values():
        y0, y1, x0, x1 = int(r0 * h), min(h - 1, int(r1 * h)), int(c0 * w), min(w - 1, int(c1 * w))
        img[y0:y0 + 2, x0:x1 + 1] = img[y1 - 1:y1 + 1, x0:x1 + 1] = 255
        img[y0:y1 + 1, x0:x0 + 2] = img[y0:y1 + 1, x1 - 1:x1 + 1] = 255
    return img


def png_bytes(img: np.ndarray) -> bytes:
    """A PNG of an H x W (grey) or H x W x 3 (RGB) uint8 image, with zlib only."""
    a = np.ascontiguousarray(np.asarray(img, dtype=np.uint8))
    h, w = a.shape[:2]
    colour = 2 if a.ndim == 3 else 0
    raw = b"".join(b"\x00" + a[y].tobytes() for y in range(h))

    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, colour, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
