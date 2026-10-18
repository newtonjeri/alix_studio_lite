"""The camera specification, loaded from `config/cameras/<model>.yaml`.

Newton, 2026-10-03: the camera's own specification is the single source of
truth; resolution is a choice among the profiles the camera supports; topics
must be consistent everywhere.

Before this module those facts lived in four places that had already drifted
apart — the Isaac rig, the perception core, the real bringup launch, and the
Gazebo plugin — with a depth FOV of 80 deg in one and 87 in the others, a colour
FOV of 62.2 deg in three (all tracing to one unsourced `camera_fx: 1591.41`)
against a datasheet 69.4, and a depth profile of 1280x720 on hardware against
848x480 in simulation. `test_camera_config.py` is what keeps them together now.

**This module imports only `yaml`, `math` and `pathlib` — deliberately.**
It must be importable by Isaac Sim's bundled interpreter from
`ros/simulation/isaacsim/scripts/`, which is standalone and does not have the
alix core installed. Importing the PACKAGE (`alix_perception_core`) would run
`__init__.py`, which pulls numpy, the backends and the model registry. So the
Isaac side loads this file directly by path — and the registration line is
REQUIRED, not decoration:

    spec = importlib.util.spec_from_file_location("camera_config", <path>)
    module = importlib.util.module_from_spec(spec)
    sys.modules["camera_config"] = module      # BEFORE exec_module
    spec.loader.exec_module(module)

Without that line `@dataclass` raises `AttributeError: 'NoneType' object has no
attribute '__dict__'`: this module uses `from __future__ import annotations`, so
dataclass resolves its field types by looking the module up in `sys.modules`,
and a module_from_spec is not registered there. `alix_paths.load_camera_config`
does it correctly; copy that rather than re-deriving it.

This arrangement only stays true while this module's imports stay this small.
Do not add a numpy import here.

Reading a value:

    cam = load_camera("d435")
    cam.depth.active.width            # 1280
    cam.depth.min_z_m()               # 0.268 — computed, not stored
    cam.color.intrinsics()            # Intrinsics-shaped, from spec FOV
    cam.topic("depth", "image")       # /camera/camera/depth/image_rect_raw
    cam.rs_launch_args()              # what rs_launch.py needs
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

#: Where the shipped specifications live.
CONFIG_DIR = Path(__file__).resolve().parent / "config" / "cameras"

#: The model fitted to alix today. A swap is a one-word change here plus a new
#: yaml — and the test asserts the fitted model can actually see the tool.
DEFAULT_MODEL = "d435"


class CameraConfigError(ValueError):
    """A camera specification is missing, malformed, or asks for the impossible."""


@dataclass(frozen=True)
class Profile:
    """One resolution/rate the camera can be run at."""

    width: int
    height: int
    fps: int

    def __str__(self) -> str:
        return f"{self.width}x{self.height}x{self.fps}"

    @property
    def aspect(self) -> float:
        return self.width / self.height


@dataclass(frozen=True)
class Stream:
    """One image stream: its optics, the profiles it supports, the one it runs."""

    name: str
    frame_id: str
    enabled: bool
    active: Profile | None
    supported: tuple[Profile, ...]
    hfov_deg: float | None
    vfov_deg: float | None
    baseline_m: float | None
    max_disparity: int | None
    subpixel: float | None
    recommended: Profile | None = None
    recommended_reason: str = ""

    # -- optics ------------------------------------------------------
    def focal_px(self, profile: Profile | None = None) -> float:
        """Horizontal focal length in pixels, from the SPEC field of view.

        Derived rather than stored because it changes with resolution: a focal
        length is only meaningful beside the width it was computed for, and
        storing one invites it being reused at another resolution. That is
        exactly how `fx = 1591.41` escaped into three files at three different
        resolutions.
        """
        p = profile or self.active
        if p is None or self.hfov_deg is None:
            raise CameraConfigError(
                f"stream '{self.name}' has no active profile or no FOV, so it "
                f"has no focal length")
        return (p.width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

    def intrinsics(self, profile: Profile | None = None) -> dict:
        """Pinhole intrinsics for a profile — fx, fy, cx, cy, width, height.

        The principal point is the image centre: a nominal specification has no
        decentring to describe. A real unit's `camera_info` does, and on
        hardware that is what the perception core uses.
        """
        p = profile or self.active
        if p is None:
            raise CameraConfigError(f"stream '{self.name}' has no active profile")
        f = self.focal_px(p)
        return {"fx": f, "fy": f, "cx": p.width / 2.0, "cy": p.height / 2.0,
                "width": p.width, "height": p.height}

    # -- depth-only --------------------------------------------------
    def min_z_m(self, profile: Profile | None = None) -> float:
        """Closest measurable range: ``focal_px * baseline / max_disparity``.

        Only meaningful on a stereo depth stream. This is the number that makes
        the profile choice a geometry decision rather than a quality setting —
        the D435 cannot see closer than 268 mm at 1280x720 and 177 mm at
        848x480, and alix's tool sits at 186 mm.
        """
        if self.baseline_m is None or self.max_disparity is None:
            raise CameraConfigError(
                f"stream '{self.name}' is not a stereo depth stream; it has no MinZ")
        return self.focal_px(profile) * self.baseline_m / self.max_disparity

    def sigma_z_m(self, depth_m: float, profile: Profile | None = None) -> float:
        """Depth noise floor at a range: ``z^2 * subpixel / (focal * baseline)``."""
        if self.baseline_m is None or self.subpixel is None:
            raise CameraConfigError(f"stream '{self.name}' has no noise model")
        return depth_m * depth_m * self.subpixel / (self.focal_px(profile) * self.baseline_m)

    def sees(self, depth_m: float, profile: Profile | None = None) -> bool:
        return depth_m >= self.min_z_m(profile)

    def supports(self, profile: Profile) -> bool:
        return profile in self.supported


@dataclass(frozen=True)
class CameraModel:
    """One camera's full specification."""

    model: str
    family: str
    description: str
    streams: dict
    topic_namespace: str
    topic_map: dict
    frames: dict
    mechanical: dict
    targets: dict
    raw: dict

    # -- stream access ----------------------------------------------
    def __getitem__(self, name: str) -> Stream:
        try:
            return self.streams[name]
        except KeyError:
            raise CameraConfigError(
                f"{self.model} has no stream '{name}'; it has: "
                f"{', '.join(sorted(self.streams))}") from None

    @property
    def color(self) -> Stream:
        return self["color"]

    @property
    def depth(self) -> Stream:
        return self["depth"]

    @property
    def aligned_depth(self) -> Stream:
        return self["aligned_depth"]

    def enabled_streams(self) -> tuple[str, ...]:
        return tuple(n for n, s in self.streams.items() if s.enabled)

    # -- topics: one naming scheme, the real driver's ----------------
    def topic(self, stream: str, kind: str = "image") -> str:
        """Fully-qualified topic, e.g. ``/camera/camera/depth/image_rect_raw``.

        Every consumer asks here rather than spelling a topic out, which is what
        stops a fourth spelling appearing. Gazebo currently publishes depth on
        ``/camera/camera/depth``, a name no consumer subscribes to and which
        this map does not contain.
        """
        entry = self.topic_map.get(stream)
        if not entry or kind not in entry:
            raise CameraConfigError(
                f"{self.model} declares no '{kind}' topic for stream '{stream}'")
        return f"{self.topic_namespace.rstrip('/')}/{entry[kind]}"

    def frame_id(self, stream: str, optical: bool = True) -> str:
        entry = self.frames.get(stream)
        if not entry:
            raise CameraConfigError(f"{self.model} declares no frames for '{stream}'")
        return entry["optical"] if optical else entry["frame"]

    # -- per-target -------------------------------------------------
    def target(self, name: str) -> dict:
        return dict(self.targets.get(name) or {})

    def rs_launch_args(self) -> dict:
        """Arguments for `realsense2_camera`'s rs_launch.py.

        Profiles are derived from the config so the real camera and the
        simulated one cannot drift; everything else comes from
        `targets.hardware.extra_args` verbatim.
        """
        args = {}
        if self.color.active:
            args["rgb_camera.color_profile"] = str(self.color.active)
        if self.depth.active:
            args["depth_module.depth_profile"] = str(self.depth.active)
        args.update({k: str(v) for k, v in (self.target("hardware").get("extra_args") or {}).items()})
        return args

    # -- the question the fitted camera has to answer ---------------
    def clears(self, near_m: float, far_m: float) -> tuple[bool, str]:
        """Can the depth stream measure across a range? -> (ok, explanation).

        `(0.186, 0.271)` is alix's wipe tool, measured from the URDF. This is
        the check that turns "which camera is fitted" into something a test can
        fail on, and it is the reason a D455 is disqualified for this arm.
        """
        min_z = self.depth.min_z_m()
        if min_z <= near_m:
            return True, (f"{self.model} depth at {self.depth.active} has MinZ "
                          f"{min_z * 1000:.0f} mm, which clears {near_m * 1000:.0f} mm")
        covered = "none of" if min_z >= far_m else "only part of"
        return False, (f"{self.model} depth at {self.depth.active} has MinZ "
                       f"{min_z * 1000:.0f} mm and so sees {covered} the "
                       f"{near_m * 1000:.0f}-{far_m * 1000:.0f} mm range")


# ── loading ─────────────────────────────────────────────────────────
def _profile(value, where: str) -> Profile:
    if not (isinstance(value, (list, tuple)) and len(value) == 3):
        raise CameraConfigError(f"{where}: expected [width, height, fps], got {value!r}")
    return Profile(int(value[0]), int(value[1]), int(value[2]))


def _build_stream(name: str, spec: dict, doc: dict) -> Stream:
    streams = doc.get("streams") or {}
    optics = doc.get("spec") or {}
    asic = doc.get("asic") or {}

    # `follows: color` means this stream inherits the other's resolution — which
    # is what aligned depth IS (depth reprojected into the colour frame), and
    # what the infra streams are (the depth sensor's own images).
    source_name = spec.get("follows", name)
    source = streams.get(source_name, spec)

    active = spec.get("active_profile") or source.get("active_profile")
    supported = spec.get("supported") or source.get("supported") or []

    # Optics come from the SPEC block, keyed by which physical sensor the stream
    # is on: anything following depth is on the stereo pair, aligned depth is
    # resampled onto the colour sensor's optics.
    sensor = "color" if source_name == "color" else "depth"
    sensor_spec = optics.get(sensor) or {}
    fov = sensor_spec.get("fov_deg") or [None, None]

    is_stereo = sensor == "depth"
    return Stream(
        name=name,
        frame_id=spec.get("frame_id", ""),
        enabled=bool(spec.get("enabled", True)),
        active=_profile(active, f"{name}.active_profile") if active else None,
        supported=tuple(_profile(p, f"{name}.supported") for p in supported),
        hfov_deg=fov[0],
        vfov_deg=fov[1] if len(fov) > 1 else None,
        baseline_m=sensor_spec.get("baseline_m") if is_stereo else None,
        max_disparity=asic.get("max_disparity") if is_stereo else None,
        subpixel=asic.get("subpixel_accuracy") if is_stereo else None,
        recommended=(_profile(spec["recommended_profile"], f"{name}.recommended_profile")
                     if spec.get("recommended_profile") else None),
        recommended_reason=spec.get("recommended_reason", "") or "",
    )


def available_models(directory: Path | None = None) -> tuple[str, ...]:
    d = Path(directory) if directory else CONFIG_DIR
    if not d.is_dir():
        return ()
    return tuple(sorted(p.stem for p in d.glob("*.yaml")))


def load_camera(model: str = DEFAULT_MODEL,
                path: str | Path | None = None) -> CameraModel:
    """Load a camera specification by model name, or from an explicit path."""
    source = Path(path) if path else CONFIG_DIR / f"{model}.yaml"
    if not source.is_file():
        raise CameraConfigError(
            f"no specification for camera '{model}' at {source}; available: "
            f"{', '.join(available_models()) or 'none'}")

    with source.open() as handle:
        doc = yaml.safe_load(handle) or {}

    raw_streams = doc.get("streams") or {}
    streams = {name: _build_stream(name, spec or {}, doc)
               for name, spec in raw_streams.items()}

    topics = dict(doc.get("topics") or {})
    namespace = topics.pop("namespace", "/camera/camera")

    cam = CameraModel(
        model=doc.get("model", model),
        family=doc.get("family", ""),
        description=doc.get("description", ""),
        streams=streams,
        topic_namespace=namespace,
        topic_map=topics,
        frames=doc.get("frames") or {},
        mechanical=doc.get("mechanical") or {},
        targets=doc.get("targets") or {},
        raw=doc,
    )
    _validate(cam)
    return cam


def _validate(cam: CameraModel) -> None:
    """Refuse a specification that contradicts itself.

    A profile outside the supported set is the error this exists for: "adjust
    the resolution" is only safe while the camera actually supports the result,
    and a silent clamp would put the sim and the hardware on different cameras
    again by a different route.
    """
    for name, stream in cam.streams.items():
        if not stream.enabled or stream.active is None:
            continue
        if stream.supported and not stream.supports(stream.active):
            raise CameraConfigError(
                f"{cam.model}: stream '{name}' is set to {stream.active}, which "
                f"is not one of its supported profiles "
                f"({', '.join(str(p) for p in stream.supported)})")
    for stream in ("color", "depth"):
        if stream not in cam.streams:
            raise CameraConfigError(f"{cam.model}: no '{stream}' stream declared")
