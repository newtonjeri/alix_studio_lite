"""The fitted camera: intrinsics, and where it is blind.

**The specification lives in `config/cameras/<model>.yaml`, not here**
(Newton, 2026-10-03: the camera's own specification is the single source of
truth). `camera_config.load_camera()` reads it; this module holds the geometry
that CONSUMES a specification — back-projection intrinsics, the MinZ and noise
models, and the image-roll measure — plus module constants kept as D435-shaped
defaults so existing callers and tests keep working.

One number changed when the specification became the authority, and it was
wrong everywhere before: the colour field of view is **69.4 deg** (the D435
datasheet), not 62.2. The 62.2 figure traced to a single unsourced
`camera_fx: 1591.41` in butterfli's `perception_params.yaml`, and had propagated
into the Gazebo plugin and the Isaac rig. See the config header.

The rest was measured or derived in
`Resource/reports/2026-09-08_perception_port`.

Two facts drive every decision downstream.

**The camera is rigid to `R_joint7`.** It is not merely "on the wrist" — it is a
child of the wrist ROLL, the same axis the wipe is free to spin about. So the
image rolls with the arm, and the roll is large: alix's `inspection_pose_1` (the
colour viewpoint re-taught 2026-10-08) rolls it -87 degrees, `inspection_pose_4`
(the depth viewpoint) 1.5; the three poses retired that day sat at 64-106. A
model trained or prompted on upright
imagery sees a scene on its side. `image_roll_deg` computes it from the camera
transform so a caller can de-rotate rather than discover this in the field.

**The tool is 0.19-0.27 m in front of the lens**, and MinZ decides whether the
depth camera can see it at all. `MinZ = f_px * baseline / max_disparity` — so it
scales with resolution, and the profile matters more than anything in the tuning
guide:

    1280x720 -> 268 mm   the tool is INSIDE the blind zone
     848x480 -> 177 mm   the tool is measurable
     640x480 -> 134 mm

butterfli ran 1280x720. Prefer `DEPTH_848x480`: it costs about 1 mm of extra
noise at 0.9 m, which is an order of magnitude below the hand-eye error that
already dominates (1 degree = 15.7 mm at that range).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

#: Stereo baseline, metres. Sets both MinZ and the depth noise floor.
BASELINE_M = 0.050

#: The D4xx ASIC's disparity search range at <=1280 px wide. MinZ falls out of it.
MAX_DISPARITY = 126

#: Sub-pixel disparity accuracy used for the noise model. Intel's own figure for
#: a well-tuned D435; real porcelain is worse, so treat sigma_z as a FLOOR.
SUBPIXEL = 0.08

#: Depth (stereo IR) horizontal FOV. Much wider than colour — which is why the
#: depth stream sees the tool and the colour stream does not.
DEPTH_HFOV_DEG = 87.0

#: Colour horizontal field of view, degrees — the D435 DATASHEET figure.
#: Previously this module carried `COLOR_FX_1920 = 1591.41` (62.2 deg), copied
#: from butterfli's fallback parameter, which had no stated source and which the
#: Gazebo plugin and the Isaac rig had both inherited. The specification says
#: 69.4; a focal length is derived from it per resolution rather than stored,
#: because a stored fx is only valid beside the width it was computed for, and
#: that is precisely how 1591.41 escaped into three files at three resolutions.
COLOR_HFOV_DEG = 69.4

#: Kept so `Intrinsics.color_fallback` has a single place to scale from, and so
#: a reader can see the relationship rather than a magic number.
COLOR_FX_1920 = (1920 / 2.0) / math.tan(math.radians(COLOR_HFOV_DEG) / 2.0)


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole intrinsics for one stream, in pixels."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_camera_info_k(cls, k, width: int, height: int) -> "Intrinsics":
        """From a ROS ``CameraInfo.k`` (row-major 3x3), the preferred source.

        Live intrinsics beat any constant in this file: they carry the unit's own
        calibration, and on a resolution change they follow.
        """
        return cls(fx=float(k[0]), fy=float(k[4]),
                   cx=float(k[2]), cy=float(k[5]),
                   width=int(width), height=int(height))

    @classmethod
    def color_fallback(cls, width: int = 1280, height: int = 720) -> "Intrinsics":
        """Nominal colour intrinsics from the SPEC field of view, at any width.

        For use only when `camera_info` has not arrived, and a caller that
        reaches for it should say so in its log: a missing `camera_info` means
        the driver is not up, which is a fault to report rather than a mode to
        run in. On hardware the live `camera_info` carries that unit's own
        factory calibration, and the datasheet FOV is nominal to +/-3 deg —
        librealsense itself ships no per-model FOV table and computes it from
        the device (`rs2_fov`). In simulation there is no unit to calibrate, so
        the specification IS the camera.
        """
        f = COLOR_FX_1920 * width / 1920.0
        return cls(fx=f, fy=f, cx=width / 2.0, cy=height / 2.0,
                   width=width, height=height)

    @property
    def hfov_deg(self) -> float:
        return 2.0 * math.degrees(math.atan(self.width / 2.0 / self.fx))

    @property
    def vfov_deg(self) -> float:
        return 2.0 * math.degrees(math.atan(self.height / 2.0 / self.fy))

    def scaled_to(self, width: int, height: int) -> "Intrinsics":
        """The same camera, expressed for a resized image.

        Needed because inference on a 1920x1080 frame is several times the cost
        of inference on a 640x360 one for no gain on a model trained at 640, and
        the moment the image is resized the intrinsics MUST follow it. Scaling
        the image without scaling fx is the classic way to produce 3D positions
        that are wrong by the scale factor and look entirely plausible.
        """
        sx = float(width) / float(self.width)
        sy = float(height) / float(self.height)
        return Intrinsics(fx=self.fx * sx, fy=self.fy * sy,
                          cx=self.cx * sx, cy=self.cy * sy,
                          width=int(width), height=int(height))

    def mm_per_pixel(self, depth_m: float) -> float:
        """Lateral size of one pixel at a given range — the detection floor.

        At 1280x720 and 0.9 m this is 0.85 mm/px, so a 10 mm stain is ~12 px.
        Anything smaller is not a detection problem, it is a sampling problem.
        """
        return 1000.0 * depth_m / self.fx


@dataclass(frozen=True)
class DepthProfile:
    """A depth stream configuration, and the two limits it implies."""

    width: int
    height: int
    label: str

    @property
    def focal_px(self) -> float:
        return (self.width / 2.0) / math.tan(math.radians(DEPTH_HFOV_DEG / 2.0))

    @property
    def min_z_m(self) -> float:
        """Closest measurable range. Below this the stream returns nothing."""
        return self.focal_px * BASELINE_M / MAX_DISPARITY

    def sigma_z_m(self, depth_m: float) -> float:
        """Depth noise floor at a range: ``z^2 * subpixel / (f * B)``."""
        return depth_m * depth_m * SUBPIXEL / (self.focal_px * BASELINE_M)

    def sees(self, depth_m: float) -> bool:
        return depth_m >= self.min_z_m


DEPTH_1280x720 = DepthProfile(1280, 720, "1280x720")
DEPTH_848x480 = DepthProfile(848, 480, "848x480")
DEPTH_640x480 = DepthProfile(640, 480, "640x480")

#: The profile to use on this robot, and the reason is geometric rather than a
#: preference: it is the widest one whose MinZ (177 mm) clears the tool at
#: 0.19 m. Changing it is a decision about whether close range matters.
DEFAULT_DEPTH_PROFILE = DEPTH_848x480

#: Range from the colour lens to the wipe tool frames, metres — measured from the
#: URDF, and constant at every arm configuration because the camera and the tool
#: are both rigid to `R_joint7`. The near end is `R_glove_frame_1`, the wipe
#: `ik_frame`; the far end is `R_tip_frame`.
TOOL_RANGE_M = (0.186, 0.271)


#: The direction that must point UP in a canonical image, expressed in the
#: gravity-aligned robot frame the roll is measured against: up (+Z) and away
#: from the robot toward the fixture (+X). Gravity alone is undefined for a
#: camera looking straight down — `inspection_pose_4` does, the lid fills the
#: frame and the cistern sits at the bottom (Newton, 2026-10-06) — and the
#: fixture's own axis alone is undefined face-on. Their sum is defined at every
#: attitude a wrist camera inspecting a fixture in front of the robot can take,
#: and agrees with each where the other is defined: looking down, the far side
#: (the cistern, +X) ends up at the top of the image; face-on, the floor ends up
#: at the bottom. That is the orientation `region_bands.yaml` assumes.
CANONICAL_UP_WORLD = (1.0, 0.0, 1.0)


def image_roll_deg(camera_to_world: np.ndarray, up_world=None) -> float:
    """SIGNED roll of the image about the line of sight, degrees, 0 = upright.

    The angle from the image's DOWN direction (+Y of a ROS optical frame) to the
    reference "down" (`-up_world`), both projected onto the image plane, so the
    measure is independent of how far the camera is tilted. The sign is the one
    `geometry.rotate_image_upright` needs: pass the result straight through.
    (Until 2026-10-06 this was an unsigned `arccos`, which de-rotated a
    negative roll the wrong way — 2x the error instead of none.)

    This exists because it is large and invisible. The camera hangs off the wrist
    roll, so `wipe_yaw` and the choice of inspection pose both rotate the scene:
    13.2 degrees at butterfli's `ready` pose, 64-106 at alix's own inspection
    poses. Every reference image the VLM path ships is upright.

    Args:
        camera_to_world: 4x4 pose of the camera OPTICAL frame in a
            gravity-aligned frame (world / base_link — +Z up; for the default
            `up_world` also +X toward the fixture, which base_link is).
        up_world: the direction that must point up in the canonical image.
            None = gravity only (world +Z), the pre-2026-10-06 behaviour, which
            is undefined (returns 0) when the camera looks straight down. The
            core passes `CANONICAL_UP_WORLD`.
    """
    rotation = np.asarray(camera_to_world, dtype=float)[:3, :3]
    up = np.asarray((0.0, 0.0, 1.0) if up_world is None else up_world, dtype=float)
    norm = np.linalg.norm(up)
    if norm < 1e-9:
        return 0.0
    # the reference "down" as the camera sees it: x right, y down, z forward
    down_cam = rotation.T @ (-up / norm)
    vx, vy = float(down_cam[0]), float(down_cam[1])
    if math.hypot(vx, vy) < 1e-9:
        # Boresight along the reference: every roll is equally "upright" and the
        # measure is undefined. 0.0 rather than NaN so a caller's threshold works.
        return 0.0
    return math.degrees(math.atan2(vx, vy))
