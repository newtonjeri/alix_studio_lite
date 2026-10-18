"""Pixels to metres: the one part of butterfli's perception that was correct.

Ported near-verbatim from `butterfli_perception/scripts/frame_transformer.py`,
with the ROS taken out — it took a `tf2_ros.Buffer` and `TransformStamped`, so it
could not be exercised without a graph. Here it takes a 4x4 matrix and returns
tuples, which is the same arithmetic with a testable boundary.

Three behaviours are kept deliberately, because each was right:

* **Depth is sampled as a patch median, not a pixel.** A single pixel on wet
  porcelain is frequently a dropout or a specular spike. The median of a 7x7
  patch over valid samples only is the difference between a usable range and an
  outlier that lands the arm somewhere else.
* **Colour and depth resolutions are allowed to differ**, and the sample point is
  rescaled. They *do* differ here — the recommended pairing is 1280x720 colour
  with 848x480 depth (`camera.DEFAULT_DEPTH_PROFILE`).
* **An invalid sample falls back rather than failing.** But unlike the original,
  the fallback is *labelled* (`ROI3D.depth_source`), because a caller that cannot
  distinguish a measurement from a guess will act on both.

What is NOT kept: the original inferred a stain's physical size by projecting its
bbox corners at the CENTRE's depth. On a curved bowl that is wrong at the edges,
and it was reported to 1 mm. It is still the only estimate available from one
view, so it is still computed — but `roi_from_detection` says so in a comment and
the value is documented as an upper bound on a plane through the centre.
"""

from __future__ import annotations

import math

import numpy as np

from .camera import Intrinsics
from .types import Detection, ROI3D


def pixel_to_camera(u: float, v: float, depth_m: float, intr: Intrinsics
                    ) -> tuple[float, float, float]:
    """Back-project a pixel to the camera optical frame (+X right, +Y down, +Z fwd)."""
    return (
        (u - intr.cx) * depth_m / intr.fx,
        (v - intr.cy) * depth_m / intr.fy,
        depth_m,
    )


def transform_point(point: tuple[float, float, float], transform: np.ndarray
                    ) -> tuple[float, float, float]:
    """Apply a 4x4 homogeneous transform to a point."""
    p = np.asarray(transform, dtype=float) @ np.array(
        [point[0], point[1], point[2], 1.0], dtype=float)
    return float(p[0]), float(p[1]), float(p[2])


def sample_depth(depth_image: np.ndarray, u: float, v: float,
                 color_width: int, color_height: int,
                 patch_radius: int = 3) -> float:
    """Median valid depth near a COLOUR pixel, in metres. NaN if nothing valid.

    `u, v` are colour-image coordinates; the depth image may be a different
    resolution and is rescaled. Zero, negative and non-finite samples are
    rejected before the median — on a RealSense, 0 IS the no-data value, so
    averaging without that filter pulls every range toward the camera.
    """
    if depth_image is None or depth_image.size == 0:
        return float("nan")

    dh, dw = depth_image.shape[:2]
    u_d = int(round(u * dw / color_width))
    v_d = int(round(v * dh / color_height))
    u_d = max(0, min(u_d, dw - 1))
    v_d = max(0, min(v_d, dh - 1))

    y0, y1 = max(0, v_d - patch_radius), min(dh, v_d + patch_radius + 1)
    x0, x1 = max(0, u_d - patch_radius), min(dw, u_d + patch_radius + 1)
    patch = np.asarray(depth_image[y0:y1, x0:x1], dtype=np.float64).ravel()

    valid = patch[(patch > 0.0) & np.isfinite(patch)]
    if valid.size == 0:
        return float("nan")
    return float(np.median(valid))


def to_metres(depth_image: np.ndarray) -> np.ndarray:
    """Normalise a depth image to metres.

    16UC1 is millimetres (the RealSense driver's native encoding), 32FC1 is
    already metres (what the Isaac bridge and the Gazebo plugin publish). Getting
    this backwards is a factor of 1000, which is obvious — the dangerous case is
    a pipeline that only ever sees one of them and hard-codes the other.
    """
    arr = np.asarray(depth_image)
    if arr.dtype == np.uint16:
        return arr.astype(np.float32) / 1000.0
    return arr.astype(np.float32, copy=False)


def roi_from_detection(det: Detection, intr: Intrinsics,
                       camera_to_target: np.ndarray,
                       depth_image: np.ndarray | None = None,
                       fallback_depth_m: float = 0.9) -> ROI3D:
    """Lift one detection into the target frame.

    Args:
        det: the 2D detection, in pixels of an image with `intr`'s dimensions.
        intr: intrinsics of the image the detection came from.
        camera_to_target: 4x4 transform, camera optical frame -> target frame.
        depth_image: aligned depth in metres. None uses `fallback_depth_m`.
        fallback_depth_m: range used when depth is unavailable or invalid.
            Default 0.9 m — the measured range from the inspection pose to the
            bowl interior (0.892-1.032 m over 8 field runs). butterfli's 0.5 m
            was roughly 0.4 m short.
    """
    u, v = det.center
    x0, y0, x1, y1 = det.bbox

    if depth_image is None:
        depth, source = fallback_depth_m, "fallback"
    else:
        depth = sample_depth(depth_image, u, v, intr.width, intr.height)
        if math.isnan(depth) or depth <= 0.0:
            depth, source = fallback_depth_m, "invalid"
        else:
            source = "measured"

    center_cam = pixel_to_camera(u, v, depth, intr)

    # Physical extent, from the corners projected at the CENTRE's depth. Exact
    # only for a fronto-parallel plane through the centre; on a bowl wall it
    # over-states the far edge. Kept because one view supports nothing better,
    # and flagged because it is reported in millimetres.
    c_min = pixel_to_camera(x0, y0, depth, intr)
    c_max = pixel_to_camera(x1, y1, depth, intr)

    return ROI3D(
        center=transform_point(center_cam, camera_to_target),
        width=abs(c_max[0] - c_min[0]),
        height=abs(c_max[1] - c_min[1]),
        confidence=det.confidence,
        region=det.region,
        depth_m=depth,
        depth_source=source,
        label=det.label,
        kind=det.kind,
        severity=det.severity,
        # The pixels, carried forward. Everything downstream that has to SHOW a
        # person what the robot reacted to needs the box, and re-deriving it from
        # the 3D centre would round-trip through every approximation above.
        bbox_px=(float(x0), float(y0), float(x1), float(y1)),
    )


def rotate_image_upright(image: np.ndarray, roll_deg: float
                         ) -> tuple[np.ndarray, np.ndarray]:
    """De-rotate an image by its measured roll, and return the inverse mapping.

    The camera is rigid to the wrist roll (`camera.image_roll_deg`), so at alix's
    own inspection poses the scene arrives 64-106 degrees over. Every reference
    image and every prompt in the inherited VLM path assumes upright, and a
    detector trained on upright crops has the same assumption whether or not
    anyone wrote it down.

    Returns `(rotated, inverse_2x3)`. Apply `inverse_2x3` to any point found in
    the rotated image to get back to original pixel coordinates — detections must
    be mapped back before they are projected, because the intrinsics belong to the
    ORIGINAL image.

    The canvas is expanded so nothing is cropped; the caller's intrinsics are
    therefore not valid in the rotated frame, which is exactly why the inverse is
    returned rather than left to the caller to reconstruct.
    """
    import cv2

    h, w = image.shape[:2]
    center = (w / 2.0, h / 2.0)
    m = cv2.getRotationMatrix2D(center, -roll_deg, 1.0)

    cos, sin = abs(m[0, 0]), abs(m[0, 1])
    new_w = int(h * sin + w * cos)
    new_h = int(h * cos + w * sin)
    m[0, 2] += new_w / 2.0 - center[0]
    m[1, 2] += new_h / 2.0 - center[1]

    rotated = cv2.warpAffine(image, m, (new_w, new_h),
                             flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT,
                             borderValue=(0, 0, 0))
    inverse = cv2.invertAffineTransform(m)
    return rotated, inverse


def map_points(points, affine_2x3: np.ndarray) -> list[tuple[float, float]]:
    """Apply a 2x3 affine to (x, y) points — the inverse from `rotate_image_upright`."""
    out = []
    for x, y in points:
        p = np.asarray(affine_2x3, dtype=float) @ np.array([x, y, 1.0])
        out.append((float(p[0]), float(p[1])))
    return out


def unrotate_detection(det: Detection, inverse_2x3: np.ndarray) -> Detection:
    """Map a detection found in a de-rotated image back to original pixels.

    The rotated box's corners do not stay axis-aligned, so the result is the
    axis-aligned bound of the mapped corners. That is slightly larger than the
    true footprint — the safe direction for something that will be cleaned.
    """
    x0, y0, x1, y1 = det.bbox
    corners = map_points([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], inverse_2x3)
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    return Detection(bbox=(min(xs), min(ys), max(xs), max(ys)),
                     confidence=det.confidence, label=det.label, region=det.region,
                     kind=det.kind, severity=det.severity)
