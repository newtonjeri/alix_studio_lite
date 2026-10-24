"""The lid and seat checks with the wrist D435, without ROS: ALIX's own gap check, run here.

ALIX's lid and seat plans (vendor/alix/Resource/plans/commode_c/hardware) carry `check_fixture_state`
actions with `check: fixture_gap`: before a plan moves, and after the left arm's lift, the right wrist's
D435 is asked what it sees. On the robot ALIX's perception node answers (`/perception/analyze_scene`, mode
`gap_check`); since 24 Oct its decision lives in ALIX's ROS-free alix_perception_core/gap_check.py
(`GapCheckSession`), vendored here and called as the node calls it (the owner, 24 Oct: "what ALIX ... does
should be the same as what happens in Lite ... the same result for the same input in both"). Lite has no ROS
(the owner, 18 Oct), so it gives that code what ROS gave the node:

- the depth: the D435's NATIVE depth stream (848 x 480, z16 mm), as the node's default
  `gap_depth_source: depth`, and its own intrinsics (camera_d435.py, pyrealsense2);
- the camera's pose in base_link: forward kinematics on ALIX's URDF with the arm's, the pole's and the
  hands' live readings (robot_fk.py; the node had it from TF, which robot_state_publisher computes on
  the same URDF: on 12 Oct at pose 4 the two agree within 0.3 mm and 0.08 degrees);
- the hands' frames (FK, the node's TF), the named pose the right arm is at (the node's joint states), the
  fixture placed by the same `load_fixture` (the scene, the venue's yaw, the calibration);
- the colour models' call on the colour frame and the depth aligned to it (camera_d435.py): ALIX's
  alix_perception_core, vendored, with the robot bringup's models (models_synthetic_real_v10.yaml); without
  ultralytics they cannot be asked and the geometry answers alone, as the node does then.

Same parameters (the vendored perception.yaml, read here, not copied; the node's defaults equal it), the same
decision, the same session state (the registration, the seat's closed reference, the last lid decision), the
same record fields, written into Lite's logs, Resources/Logs/<day>/checks/, with Lite's training-row fields.
The status_check mode (the fixture's state from the colour model, the depth deciding when its margin is
clear) stays Lite's port of the node's (``status``).

The executive's side (what proves an expectation, the vote over frames, what a failure asks for) is ALIX's
lift_check.py, vendored as it is and used by mechanism.py.
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
    colour: np.ndarray | None = None           # H x W x 3, BGR, the same moment's colour frame (the models' input)
    aligned: np.ndarray | None = None          # the depth aligned to the colour frame (z16 mm), filtered as the driver does
    colour_intrinsics: Intrinsics | None = None
    colour_to_base: np.ndarray | None = None   # 4 x 4, base_link <- the colour optical frame
    ref_to_base: np.ndarray | None = None      # 4 x 4, base_link <- alix_ref_frame (the node's target frame)
    source: str = "native depth (D435, pyrealsense2)"
    frame: str = DEPTH_FRAME
    stamp_s: float | None = None               # the frame's own time (the camera's clock), when known
    age_s: float | None = None                 # how old the frame was when the check used it
    colour_age_s: float | None = None
    colour_note: str = ""
    context: dict | None = None                # the run's side of the frame (both arms' readings, which check and which
    #                                            frame, the camera): written into the record as it is (a training row)


def _sha256(path) -> str | None:
    import hashlib

    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return None


_PROVENANCE: dict = {}


def provenance() -> dict:
    """What made a record, so it can serve as a training row (the owner, 23 Oct: every check feeds the models): Lite's
    commit, the ALIX commit Lite vendored (vendor/MANIFEST.json), the models' sha256. Worked out once."""
    if not _PROVENANCE:
        import subprocess

        try:
            head = subprocess.run(["git", "-C", str(vendor_paths.LITE_ROOT), "rev-parse", "HEAD"], capture_output=True,
                                  text=True, timeout=5).stdout.strip()
        except Exception:                                                   # noqa: BLE001 - not a checkout
            head = ""
        try:
            alix = json.loads((vendor_paths.LITE_ROOT / "vendor" / "MANIFEST.json").read_text(encoding="utf-8")).get("alix_git")
        except Exception:                                                   # noqa: BLE001
            alix = None
        models = MANIFEST.parents[3] / "models"
        _PROVENANCE.update(lite_commit=head or None, alix_vendored=alix,
                           models={p.name: _sha256(p) for p in sorted(models.glob("*.pt"))})
    return dict(_PROVENANCE)


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


MANIFEST = (vendor_paths.ALIX / "core" / "python" / "alix_perception_core" / "src" / "alix_perception_core" / "config"
            / "models_synthetic_real_v10.yaml")     # the robot bringup's models (perception.launch.py: models:=synthetic_real_v10)


class _Log:
    """The session's log (info / warn / debug) onto Lite's one-line log."""

    def __init__(self, log):
        self._log = log

    def info(self, text, *_a, **_k):
        self._log(text)

    warn = info

    def debug(self, *_a, **_k):
        pass


class GapCheck:
    """One camera's checks (the node's gap_check and status_check): ALIX's GapCheckSession with its session state
    (the registration, the seat's closed reference, the last lid decision), and its core (the colour models, the
    region locator), built as the node builds them."""

    def __init__(self, params: dict | None = None, records: Path | Callable[[], Path] | None = None,
                 log: Callable[[str], None] | None = None, core=None, manifest: Path | None = None):
        self.p = dict(params if params is not None else load_params())
        self._records = records
        self._log = log or (lambda _text: None)
        self._lock = threading.Lock()          # one check at a time: they share the session
        self._core, self._core_error, self._fixture_in_world = core, None, None
        self._locator = None
        self._manifest = Path(manifest) if manifest else MANIFEST
        self._named: dict | None = None        # the right arm's named poses (ALIX's MoveIt group states), read once
        vendor_paths.activate()
        from alix_perception_core.gap_check import GapCheckSession

        self.session = GapCheckSession(self.p.get, locator=lambda: self._locator,
                                       fixture_in_world=lambda: self._fixture_in_world, world_frame=WORLD_FRAME,
                                       log=_Log(self._log), now=time.time)

    @property
    def _registration(self) -> dict:
        return self.session._registration

    # ── the core (perception_node.py `_build_core`, `_load_locator`), built at the first question
    def core(self):
        if self._core is None and self._core_error is None:
            self._build_core()
        return self._core

    def _build_core(self) -> None:
        vendor_paths.activate()
        try:
            from alix_perception_core import PerceptionCore, load_vocabulary
            from alix_perception_core.backends import BackendError, create
        except Exception as exc:                                          # noqa: BLE001 - said in every answer
            self._core_error = f"perception core unavailable ({exc})"
            return
        try:
            backend = create(str(self.p.get("inference_backend") or "local"), manifest_path=str(self._manifest))
        except BackendError as exc:
            self._core_error = str(exc)
            self._log(f"inference backend unavailable: {exc}")
            return
        vocabulary = load_vocabulary(model=self.p.get("fixture_model", "commode_c"), fixture_type=self.p.get("fixture_type", "commode"))
        locator, self._fixture_in_world = self._load_locator()
        self._locator = locator
        self._core = PerceptionCore(
            backend=backend, vocabulary=vocabulary,
            state_confidence_threshold=float(self.p["confidence_threshold"]),
            min_stain_mm=float(self.p.get("min_stain_mm", 15.0)), fallback_depth_m=float(self.p.get("fallback_depth_m", 0.9)),
            derotate=bool(self.p.get("derotate", True)), canonical_up=tuple(float(v) for v in self.p.get("canonical_up_world", (1, 0, 1))),
            locator=locator, state_fusion=bool(self.p.get("state_fusion", True)))
        self._log(f"perception core: {backend.name} backend, {self._manifest.name}, {len(vocabulary)} regions"
                  + ("" if locator is None else f", fixture at {self._fixture_in_world[:3, 3].round(3).tolist()} in base_link"))

    def fixture_record(self) -> dict:
        """The fixture as this check placed it (a training row's projection needs it): the model, the scene file and
        its sha256, the region meshes' folder, the measured calibration, the pose in base_link, the standoff."""
        loc = self._locator
        pose = self._fixture_in_world
        return dict(model=self.p.get("fixture_model", "commode_c"), fixture_type=self.p.get("fixture_type", "commode"),
                    scene_yaml=None if loc is None else str(loc.scene_yaml),
                    scene_sha256=None if loc is None else _sha256(loc.scene_yaml),
                    regions_dir=None if loc is None else str(loc.models_dir),
                    calibration=None if loc is None else getattr(loc, "calibration", None),
                    pose_in_world=None if pose is None else np.asarray(pose, dtype=float).round(6).tolist(),
                    ugv_standoff_m=self.p.get("ugv_standoff_m"))

    def _training_row(self, frame, at_pose: str) -> dict:
        """What a record needs beyond the node's to be a training row (the owner, 23 Oct: every check feeds the models):
        the colour camera (its intrinsics and pose), the target frame, the named pose and its registration, the fixture
        as placed, the run's context (both arms, which check and frame, the camera), what made it. ``split`` is the
        dataset's to set."""
        def tf(m):
            return None if m is None else np.asarray(m, dtype=float).round(6).tolist()

        key = self.registration_key(at_pose)
        return {"colour_intrinsics": None if frame is None or frame.colour_intrinsics is None else frame.colour_intrinsics.__dict__,
                "colour_to_world": None if frame is None else tf(frame.colour_to_base),
                "ref_to_world": None if frame is None else tf(frame.ref_to_base),
                "named_pose": at_pose or None,
                "registration": {"lid_m": self.registration_for(at_pose), "seat_m": self._registration.get(key + ("seat",))},
                "fixture": self.fixture_record(), "context": None if frame is None else frame.context,
                "provenance": provenance(), "split": ""}

    def _load_locator(self):
        """ALIX's own placement (alix_perception_core.gap_check.load_fixture): the scene, the venue's yaw (until 24 Oct
        Lite read an empty fixture_venue_yaml as "no venue" and placed the commode 0.5 m beyond the real one), the
        measured calibration (Lite runs the real fixture: never the simulator's)."""
        from alix_perception_core.gap_check import load_fixture

        return load_fixture(self.p.get, sim_time=False, log=_Log(self._log))

    def _geometry(self, frame: "DepthFrame") -> dict:
        """The node's `_geometry`: camera -> alix_ref_frame, the colour intrinsics, the colour image, the aligned depth,
        the fixture's pose in alix_ref_frame."""
        transform = fixture_pose = None
        if frame.ref_to_base is not None and frame.colour_to_base is not None:
            ref_from_base = np.linalg.inv(np.asarray(frame.ref_to_base, dtype=float))
            transform = ref_from_base @ np.asarray(frame.colour_to_base, dtype=float)
            if self._fixture_in_world is not None:
                fixture_pose = ref_from_base @ self._fixture_in_world
        intrinsics = None
        if frame.colour_intrinsics is not None:
            from alix_perception_core import Intrinsics as CoreIntrinsics

            c = frame.colour_intrinsics
            intrinsics = CoreIntrinsics(fx=c.fx, fy=c.fy, cx=c.cx, cy=c.cy, width=c.width, height=c.height)
        return {"transform": transform, "intrinsics": intrinsics, "image": frame.colour, "depth": frame.aligned,
                "fixture_pose": fixture_pose}

    def _learn_registration(self, pose: str, result) -> None:
        self.session._learn_registration(pose, result)

    def _models(self, frame: "DepthFrame", roll_tf, at_pose: str = "", depth_ok: bool = True):
        """-> the colour models' call for GapCheckSession.check (the node's `_gap_models`): PerceptionCore.check_status
        on this frame's colour image and aligned depth (the session judges whether it contradicts the geometry), None
        without the aligned depth; the depth state only where the pose gate allows it, as the node."""
        def models():
            if frame.aligned is None:
                return None
            core = self.core()
            if core is None:
                raise RuntimeError(self._core_error or "perception core unavailable")
            geo = self._geometry(frame)
            return core.check_status(geo["image"], camera_to_world=roll_tf, fixture_hint="commode",
                                     depth_image=geo["depth"] if depth_ok else None, intrinsics=geo["intrinsics"],
                                     camera_to_target=geo["transform"], fixture_pose=geo["fixture_pose"],
                                     depth_registration_m=self.registration_for(at_pose))
        return models

    # ── status_check (perception_node.py's status mode): the fixture's state, the colour model's, the depth deciding
    def status(self, frame: "DepthFrame", expect: str = "", q_deg=None) -> "Answer":
        """``q_deg``: the right arm's joints (degrees) when read: the node's pose gate on the depth state."""
        with self._lock:
            return self._status(frame, expect, q_deg)

    def _status(self, frame, expect: str, q_deg=None) -> "Answer":
        response = Answer(fixture_state="", recommendation="")
        core = self.core()
        if core is None:
            response.message = self._core_error or "perception core unavailable"
            return response
        if frame is None or frame.colour is None:
            response.message = "no colour frame from the wrist camera — refusing (a stale frame is not this pose)"
            return response
        geo = self._geometry(frame)
        depth_ok, why, at_pose = self.pose_gate(q_deg)
        try:
            result = core.check_status(geo["image"], camera_to_world=frame.colour_to_base, fixture_hint="commode",
                                       depth_image=geo["depth"] if depth_ok else None, intrinsics=geo["intrinsics"],
                                       camera_to_target=geo["transform"], fixture_pose=geo["fixture_pose"],
                                       depth_registration_m=self.registration_for(at_pose))
        except Exception as exc:                                              # noqa: BLE001 - a runtime not installed (OpenCV)
            response.message = (f"the colour model could not be asked ({exc}): install the optional ultralytics "
                                "(python3 tools/setup_lite.py)")
            return response
        result.diagnostics["depth_state_gate"] = why
        self._learn_registration(at_pose, result)
        if bool(self.p.get("save_check_frames", True)) and self._records is not None:
            root = self._records() if callable(self._records) else self._records
            written = write_check_record(root, "status_check", expect,
                                         {"class": getattr(result, "fixture_state", ""), "fixture_type": getattr(result, "fixture_type", ""),
                                          "confidence": float(getattr(result, "state_confidence", 0.0) or 0.0),
                                          "backend": getattr(getattr(core, "backend", None), "name", ""),
                                          "state_source": result.state_source,
                                          "colour_state": result.diagnostics.get("colour_state", ""),
                                          "depth_state": result.diagnostics.get("depth_state", ""),
                                          "fusion": result.diagnostics.get("fusion", ""),
                                          "depth_state_gate": why, "pose": at_pose, **self._training_row(frame, at_pose)},
                                         None, frame.colour, None, aligned=frame.aligned)
            if written:
                response.record_path = str(written)
        response.fixture_type, response.fixture_state = result.fixture_type, result.fixture_state
        response.state_confidence, response.recommendation = float(result.state_confidence), result.recommendation
        response.success, response.message = result.success, result.message
        response.record = dict(state_source=result.state_source, colour_state=result.diagnostics.get("colour_state", ""),
                               depth_state=result.diagnostics.get("depth_state", ""), fusion=result.diagnostics.get("fusion", ""),
                               gate=why, pose=at_pose)
        return response

    # ── the registration: the session's (a pose AND a placement: the fixture's scene and the UGV standoff)
    def registration_key(self, pose: str) -> tuple:
        return self.session._registration_key(pose)

    def registration_for(self, pose: str) -> float:
        return self.session._registration_for(pose)

    def registration_view(self) -> dict:
        """What the session has learned, for the page and the run's log: lid and seat, in millimetres."""
        with self._lock:
            lid = self._registration.get(self.registration_key(""))
            seat = self._registration.get(self.registration_key("") + ("seat",))
        return dict(lid_mm=None if lid is None else round(lid * 1000.0, 1), seat_mm=None if seat is None else round(seat * 1000.0, 1))

    def forget(self) -> None:
        """A new placement (the UGV moved): the registration, the seat's closed reference and the last lid decision
        are learned again from the next checks."""
        with self._lock:
            self.session._registration.clear()
            self.session._seat_refs.clear()
            self.session._last_lid.clear()

    # ── the named poses (perception_node.py `_load_named_poses` and `_depth_state_pose_name`): ALIX's MoveIt group
    #    states (inspection_pose_1, inspection_pose_4, rest_pose ...), read by alix_perception_core's named_poses from
    #    the vendored tree, matched to the right arm's joints as the node matches /joint_states
    def named_poses(self) -> dict:
        """{pose: {R_jointN: radians}} of the right arm, or {} when the group states cannot be read."""
        if self._named is None:
            try:
                from alix_perception_core import named_poses                     # noqa: PLC0415 - vendored

                path = named_poses.default_group_states_file()
                self._named = named_poses.load_group_states(path, "rm75_arm_right") if path else {}
            except Exception as exc:                                              # noqa: BLE001
                self._log(f"named poses unavailable ({exc})")
                self._named = {}
        return self._named

    @staticmethod
    def _joints(q_deg) -> dict:
        """The right arm's joints as the node has them from /joint_states (radians by name)."""
        if q_deg is None or len(q_deg) < 7:
            return {}
        return {f"R_joint{i + 1}": float(np.radians(v)) for i, v in enumerate(list(q_deg)[:7])}

    def _match(self, joints: dict, poses: dict) -> str:
        from alix_perception_core import named_poses                             # noqa: PLC0415 - vendored

        return named_poses.match_named_pose(joints, poses, float(self.p.get("depth_state_pose_tolerance_rad", 0.1)))

    def named_pose(self, q_deg) -> str:
        """The named pose the right arm is at (every joint within the node's tolerance), or ''."""
        joints, poses = self._joints(q_deg), self.named_poses()
        return self._match(joints, poses) if joints and poses else ""

    def pose_gate(self, q_deg) -> tuple[bool, str, str]:
        """(may the depth decide the state here, why, the named pose or ''): the node's `_depth_state_pose_name`.
        `depth_state_poses` empty, as the robot's perception.yaml has it, means at any pose, and the pose is '' (the
        node's own value then); names in it restore the gate at those poses only."""
        wanted = [str(p) for p in (self.p.get("depth_state_poses") or []) if str(p)]
        if not wanted:
            return True, "depth_state_poses empty", ""
        poses = self.named_poses()
        if not poses:
            return False, "named poses unavailable", ""
        gate = {p: poses[p] for p in wanted if p in poses}
        if not gate:
            return False, f"depth_state_poses not in the group states: {[p for p in wanted if p not in poses]}", ""
        joints = self._joints(q_deg)
        if not joints:
            return False, "no joint states (the right arm was not read)", ""
        at = self._match(joints, gate)
        return (True, f"at {at}", at) if at else (False, f"not at any of {sorted(gate)}", "")

    # ── the check: ALIX's GapCheckSession.check, given what the node gives it
    def ask(self, expect: str, frame: DepthFrame | None, q_deg=None) -> Answer:
        """``q_deg``: the right arm's joints (degrees) when read, the node's /joint_states: the named pose the
        registration belongs to, and the depth-state gate of the models' cross-check."""
        with self._lock:
            depth_ok, _why, at_pose = self.pose_gate(q_deg)
            return self._ask(str(expect or ""), frame, at_pose, depth_ok)

    def _ask(self, expect: str, frame: DepthFrame | None, at_pose: str, depth_ok: bool = True) -> Answer:
        p = self.p
        mode = "gap_check"
        response = Answer()
        if self.core() is None:                         # the node answers nothing without its core
            response.success, response.message = False, self._core_error or "perception core unavailable"
            return response
        source = frame.source if frame is not None else "native depth (D435)"
        frame_name = frame.frame if frame is not None else DEPTH_FRAME
        roll_tf = None if frame is None else frame.cam_to_base
        rois_for_record = {"lid_roi": [float(v) for v in p["gap_roi"]], "seat_roi": [float(v) for v in p["gap_seat_roi"]],
                           "lid_wide_roi": [float(v) for v in p["gap_lid_wide_roi"]]}
        record_depth = {"arr": None}

        def keep(decision):                    # the node's `keep`: the record of this decision, Lite's clock and training row
            decision = dict(decision)
            decision.update({
                "decision_wall_s": round(time.time(), 3),
                "depth_header_stamp_s": None if frame is None or frame.stamp_s is None else round(frame.stamp_s, 6),
                "depth_age_s": None if frame is None or frame.age_s is None else round(frame.age_s, 3),
                "color_age_s": None if frame is None or frame.colour_age_s is None else round(frame.colour_age_s, 3),
                "depth_frame_id": frame_name,
            })
            decision.setdefault("source", source)
            decision.setdefault("frame", frame_name)
            decision.setdefault("world_frame", WORLD_FRAME)
            decision.setdefault("camera_to_world", None if roll_tf is None else np.asarray(roll_tf, dtype=float).round(6).tolist())
            decision.setdefault("params", self.session.record_params())
            decision.update(self._training_row(frame, at_pose))
            response.record = decision           # in the answer always; written when the records have a place
            if not bool(p.get("save_check_frames", True)) or self._records is None:
                return
            root = self._records() if callable(self._records) else self._records
            written = write_check_record(root, mode, expect, decision, record_depth["arr"],
                                         None if frame is None else frame.colour, rois_for_record,
                                         aligned=None if frame is None else frame.aligned)
            if written:
                response.record_path = str(written)
                self._log(f"check record: {written}")

        def answer(decision):
            response.success, response.message = decision.success, decision.message
            if decision.success:
                response.fixture_state, response.state_confidence = decision.state, decision.confidence
                response.recommendation = decision.recommendation
            keep(decision.record)
            return response

        if frame is None or frame.depth is None:
            return answer(self.session.refusal(expect, f"no depth frame ({source}; nothing received) -> NOT lifted"))
        colour_note = frame.colour_note or ("" if frame.colour is not None else
                                            "no colour frame from the wrist camera — the depth decides alone, the models are "
                                            "not asked, no colour frame is recorded")
        if roll_tf is None:
            return answer(self.session.refusal(expect, f"no TF {frame_name} -> {WORLD_FRAME} -> NOT lifted"))
        if frame.intrinsics is None:
            return answer(self.session.refusal(expect, f"no camera_info for the {source} -> NOT lifted"))
        depth = np.asarray(frame.depth)
        if depth.ndim != 2:
            return answer(self.session.refusal(expect, f"a depth image of shape {depth.shape} is not a depth frame -> NOT lifted"))
        depth = depth.astype(np.float64) * (0.001 if depth.dtype == np.uint16 else 1.0)
        record_depth["arr"] = depth
        h, w = depth.shape
        intrinsics = frame.intrinsics
        if (intrinsics.width, intrinsics.height) != (w, h):
            intrinsics = intrinsics.scaled_to(w, h)
        models = self._models(frame, roll_tf, at_pose, depth_ok) if frame.colour is not None else None
        decision = self.session.check(expect, depth, intrinsics, roll_tf, frame.hands or {}, at_pose=at_pose,
                                      source=source, frame=frame_name, models=models,
                                      color_used=frame.colour is not None, color_note=colour_note)
        return answer(decision)


# ── the record of a check (perception_node.py `write_check_record`, lines 71-129)
def write_check_record(root, mode: str, expect: str, decision: dict, depth=None, image=None,
                       rois: dict | None = None, stamp: str | None = None, aligned=None) -> Path | None:
    """``<root>/<stamp>_<mode>_<expect>_<class>.json`` (the decision), ``..._depth.npy`` (float32 metres, the array
    the check used), ``..._depth.png`` (0.25-0.75 m colour-mapped, the regions drawn) and ``..._color.png``. ALIX's
    names and contents; the pictures are drawn with numpy here (no OpenCV in Lite). ``aligned``: the depth aligned to
    the colour frame, kept as ``..._aligned_depth.npz`` (uint16 millimetres, compressed) for training rows (the owner,
    23 Oct). Never fails a check."""
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
        if aligned is not None:
            try:
                a = np.asarray(aligned)
                mm = a if a.dtype == np.uint16 else np.clip(np.nan_to_num(a.astype(np.float64)) * 1000.0, 0, 65535).astype(np.uint16)
                np.savez_compressed(out / f"{stem}_aligned_depth.npz", depth_mm=mm)
                files["aligned_depth_npz"] = f"{stem}_aligned_depth.npz"
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
