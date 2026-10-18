"""An SDG run, declared: what to render, how many, with which seed.

Synthetic data generation exists here for one reason, and the reason is a number:
the two shipped models score **7/14 on fixture state** and **76 false positives
over 13 clean images**, and the entire labelled corpus is **16 phone photographs**
(`Resource/Data/corpus.yaml`), several with stains painted on as flat ellipses.
There are zero real D435 wrist frames of a real soiled fixture. Rendering is the
only route to a labelled dataset in the near term.

This module is deliberately **ROS-free, Isaac-free and GPU-free**: it is the plan,
not the renderer. That means the validation below runs in `pytest` on any machine
in milliseconds, and the thing that fails a bad plan is a unit test rather than a
twenty-minute render that produces an unusable dataset.

WHAT IS VALIDATED HERE, AND WHY EACH ONE EARNS ITS PLACE
--------------------------------------------------------
**The resolution must be one the camera really supports.** The whole value of
this dataset over the phone photographs is that it carries the robot's own
extrinsics and the specification's intrinsics. A dataset rendered at a resolution
the D435 cannot produce is a dataset of a camera we do not own. `camera_rig`
already refuses an unsupported profile rather than clamping it; this refuses the
same way, from the same specification file, before the renderer starts.

**A plan must declare its label map.** An unlabelled prim is not an error in
Replicator — it is simply absent from `idToLabels`. So a plan that forgets to
label anything renders perfectly and silently produces thousands of images with
no ground truth. Requiring the map makes that a refusal at load time.

**The seed is mandatory and recorded.** Not for bit-exact regeneration — that was
MEASURED to be unavailable (identical seeds give identical scene state to six
decimal places but different pixels under RTX RealTime at any `rt_subframes`).
The contract is scene-and-label reproducibility with the images archived and a
hash per row, which is what `manifest.py` writes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

#: Annotators this package knows how to consume. Named here so a typo in a plan
#: is refused at load rather than producing a frame with one AOV silently missing
#: — Replicator does not complain about an annotator nobody attached.
KNOWN_AOVS = (
    "rgb",
    "semantic_segmentation",
    "instance_segmentation",
    "instance_segmentation_fast",
    "bounding_box_2d_tight",
    "bounding_box_2d_loose",
    "bounding_box_3d",
    "distance_to_image_plane",
    "distance_to_camera",
    "normals",
    "camera_params",
)

#: The minimum any useful plan asks for: the picture, and something that says
#: what is in it.
REQUIRED_AOVS = ("rgb",)

#: Fixture states the sampler may be asked to render. Same taxonomy as
#: `types.FIXTURE_STATES['commode']` and `fixture_rig.STATES`; restated as a
#: default rather than imported so this module keeps no dependency on the
#: inference side of the package.
DEFAULT_STATES = ("closed", "lid_open", "lid_seat_open")


class SdgPlanError(ValueError):
    """A plan is missing, malformed, or asks for something the camera cannot do."""


@dataclass(frozen=True)
class Resolution:
    width: int
    height: int

    def __str__(self) -> str:
        return f"{self.width}x{self.height}"

    @property
    def pixels(self) -> int:
        return self.width * self.height


@dataclass
class SdgPlan:
    """One declared synthetic-data run."""

    name: str
    seed: int
    frames: int
    camera_model: str = "d435"
    stream: str = "color"
    resolution: Resolution | None = None      # None = the camera's active profile
    aovs: tuple[str, ...] = ("rgb",)

    #: prim-path glob -> semantic label. REQUIRED and non-empty: see the module
    #: docstring on why an empty map is a silent catastrophe rather than an error.
    labels: dict = field(default_factory=dict)

    #: Fixture states to render, and their relative weights. Fixture state is the
    #: cheapest high-value axis in the whole exercise — the geometry and the
    #: articulation already exist and the label cannot be wrong — and it is the
    #: worst number we have (7/14).
    states: tuple[str, ...] = DEFAULT_STATES

    #: Hinge angle range in degrees for an OPEN lid/seat. `fixture_rig` reads a
    #: single angle (-110), so every synthetic `lid_open` frame would otherwise
    #: show the lid at exactly -110 while a real lid rests wherever gravity and
    #: hinge friction leave it. Train on that and we ship an angle regressor
    #: wearing a state classifier's name.
    open_angle_deg: tuple[float, float] = (-115.0, -85.0)

    #: RTX accumulation subframes per capture. More is cleaner and slower.
    rt_subframes: int = 1

    #: Free-form, passed through to the renderer and recorded in the manifest.
    randomisation: dict = field(default_factory=dict)

    #: SOIL on the fixture, per frame: `blobs: [min, max]` deposits drawn from
    #: `soil_classes.yaml`, optionally `class_weights:` and a `regions:` map of
    #: state -> allowed region names. Empty = a clean fixture, which is the
    #: state-only dataset of 2026-10-04.
    soil: dict = field(default_factory=dict)

    #: WASTE items per frame: `items: [min, max]`, `floor_xy: [[x0,x1],[y0,y1]]`
    #: in stage coordinates around the fixture. Empty = none.
    waste: dict = field(default_factory=dict)

    #: CAMERA pose: `poses:` names from the right arm's group states (the three
    #: inspection poses the real robot uses), `joint_noise_rad:` added per
    #: joint per frame, `settle_steps:` physics steps before capture. Empty =
    #: the start pose, every frame — one viewpoint, which the 2026-10-04 report
    #: named as that dataset's main weakness.
    camera: dict = field(default_factory=dict)

    #: PORCELAIN material ranges: `roughness: [lo, hi]`, `coat_roughness:`.
    #: Empty = the probe's values, fixed. Without a material at all the fixture
    #: is flat displayColor and has NO specular highlight — the strongest
    #: candidate for the shipped model's 76 false positives.
    porcelain: dict = field(default_factory=dict)

    #: Scene light intensities, drawn per frame: `{dome: [lo, hi], distant: [lo, hi]}`.
    #: MEASURED 2026-10-05: at the scene default (1000 / 3000) the porcelain
    #: renders clipped white and every mark on it is pale; a dataset needs the
    #: exposure the real camera would see, and a range of it.
    lighting: dict = field(default_factory=dict)

    #: Where the plan came from, for the manifest's provenance.
    source: Path | None = None

    def __post_init__(self) -> None:
        if self.frames <= 0:
            raise SdgPlanError(f"plan '{self.name}': frames must be positive")
        if not self.labels:
            raise SdgPlanError(
                f"plan '{self.name}' declares no `labels:`. Replicator does not "
                f"complain about an unlabelled prim — it is simply absent from "
                f"idToLabels — so a plan with no label map renders perfectly and "
                f"produces images with no ground truth. Declare at least one "
                f"prim-path glob -> label.")
        unknown = [a for a in self.aovs if a not in KNOWN_AOVS]
        if unknown:
            raise SdgPlanError(
                f"plan '{self.name}' asks for unknown annotator(s) "
                f"{', '.join(unknown)}; known: {', '.join(KNOWN_AOVS)}")
        missing = [a for a in REQUIRED_AOVS if a not in self.aovs]
        if missing:
            raise SdgPlanError(
                f"plan '{self.name}' omits required annotator(s) "
                f"{', '.join(missing)}")
        bad_states = [s for s in self.states if s not in DEFAULT_STATES]
        if bad_states:
            raise SdgPlanError(
                f"plan '{self.name}' asks for fixture state(s) "
                f"{', '.join(bad_states)}; known: {', '.join(DEFAULT_STATES)}")
        for key in ("dome", "distant"):
            rng_ = self.lighting.get(key)
            if rng_ is not None and (len(rng_) != 2 or rng_[0] < 0 or rng_[0] > rng_[1]):
                raise SdgPlanError(
                    f"plan '{self.name}': lighting.{key} must be [min, max] with 0 <= min <= max")
        markers = self.soil.get("markers") or {}
        for name, block, key in (("soil", self.soil, "blobs"), ("waste", self.waste, "items"),
                                 ("soil.markers", markers, "count"), ("soil.markers", markers, "diameter_mm")):
            rng_ = block.get(key)
            if rng_ is not None and (len(rng_) != 2 or rng_[0] < 0 or rng_[0] > rng_[1]):
                raise SdgPlanError(
                    f"plan '{self.name}': {name}.{key} must be [min, max] with 0 <= min <= max")
        low, high = self.open_angle_deg
        if low > high:
            raise SdgPlanError(
                f"plan '{self.name}': open_angle_deg is ({low}, {high}) — "
                f"low must not exceed high")

    # ------------------------------------------------------------------
    def resolve_resolution(self, camera=None) -> Resolution:
        """The resolution to render at, CHECKED against the camera specification.

        An unsupported profile is refused, not clamped. The dataset's whole
        advantage over the phone corpus is that it carries this camera's real
        optics; rendering at a size the D435 cannot produce throws that away
        while still looking like a D435 frame, which is the worst of both.

        `camera` is a `camera_config.CameraModel`. None loads it by model name —
        import is deferred so this module is usable with no specification present
        (a test, a plan lint on a machine with no config).
        """
        if camera is None:
            from ..camera_config import load_camera
            camera = load_camera(self.camera_model)

        stream = camera.streams.get(self.stream)
        if stream is None:
            raise SdgPlanError(
                f"plan '{self.name}': camera '{self.camera_model}' has no "
                f"'{self.stream}' stream; it has: "
                f"{', '.join(sorted(camera.streams))}")

        if self.resolution is None:
            if stream.active is None:
                raise SdgPlanError(
                    f"plan '{self.name}': no resolution given and "
                    f"{self.camera_model}.{self.stream} declares no active profile")
            return Resolution(stream.active.width, stream.active.height)

        matches = [p for p in stream.supported
                   if p.width == self.resolution.width
                   and p.height == self.resolution.height]
        if stream.supported and not matches:
            raise SdgPlanError(
                f"plan '{self.name}': {self.resolution} is not a profile "
                f"{self.camera_model}.{self.stream} supports "
                f"({', '.join(str(p) for p in stream.supported)}). A dataset "
                f"rendered at a resolution this camera cannot produce is a "
                f"dataset of a camera we do not own.")
        return self.resolution

    def describe(self) -> str:
        bits = [f"{self.name}: {self.frames} frame(s), seed {self.seed}",
                f"{self.camera_model}.{self.stream}",
                f"AOVs {'+'.join(self.aovs)}",
                f"states {'/'.join(self.states)}"]
        if self.resolution is not None:
            bits.insert(2, str(self.resolution))
        return "  |  ".join(bits)


def load_plan(path: str | Path) -> SdgPlan:
    """Read an SDG plan yaml."""
    path = Path(path)
    if not path.is_file():
        raise SdgPlanError(f"no SDG plan at {path}")
    with path.open() as handle:
        doc = yaml.safe_load(handle) or {}

    resolution = doc.get("resolution")
    if resolution is not None:
        if isinstance(resolution, str):               # "1920x1080"
            try:
                width, height = (int(v) for v in resolution.lower().split("x", 1))
            except ValueError as exc:
                raise SdgPlanError(
                    f"{path}: resolution {resolution!r} is not WIDTHxHEIGHT") from exc
        else:
            width, height = (int(v) for v in resolution)
        resolution = Resolution(width, height)

    angles = doc.get("open_angle_deg") or (-115.0, -85.0)
    try:
        return SdgPlan(
            name=str(doc.get("name") or path.stem),
            seed=int(doc["seed"]),
            frames=int(doc["frames"]),
            camera_model=str(doc.get("camera_model") or "d435"),
            stream=str(doc.get("stream") or "color"),
            resolution=resolution,
            aovs=tuple(doc.get("aovs") or ("rgb",)),
            labels=dict(doc.get("labels") or {}),
            states=tuple(doc.get("states") or DEFAULT_STATES),
            open_angle_deg=(float(angles[0]), float(angles[1])),
            rt_subframes=int(doc.get("rt_subframes", 1)),
            randomisation=dict(doc.get("randomisation") or {}),
            soil=dict(doc.get("soil") or {}),
            waste=dict(doc.get("waste") or {}),
            camera=dict(doc.get("camera") or {}),
            porcelain=dict(doc.get("porcelain") or {}),
            lighting=dict(doc.get("lighting") or {}),
            source=path,
        )
    except KeyError as exc:
        raise SdgPlanError(f"{path}: missing required key {exc}") from exc
