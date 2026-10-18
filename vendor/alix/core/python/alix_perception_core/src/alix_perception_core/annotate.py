"""Draw an assessment back onto the frame it came from.

Ported from `butterfli_perception/scripts/ai_visualizer.py` (the whole file) and
its `save_annotated_image` entry point. The capability is the point: a perception
result is a list of numbers, and the only way a person can check whether the
robot reacted to a stain or to a reflection is to see the boxes on the picture.
Every field debugging session in butterfli started with one of these.

THREE DIFFERENCES FROM THE ORIGINAL, each fixing something that bit:

**Pixels, not a 0-1000 normalisation.** butterfli stored every box as thousandths
of the image and multiplied back at draw time (`ai_visualizer.py:80-110`), so the
drawing was only correct if the image handed to the annotator was the same size
as the one handed to the model. It frequently was not — the node resized before
inference — and the boxes landed on the wrong things with no error. `ROI3D` now
carries `bbox_px` in the ORIGINAL image's pixels, which is the only canvas this
function ever draws on.

**Rejections are drawn, not just counted.** A diagnostic saying "8 below the size
gate" asserts the gate ran. Drawing the eight dotted boxes shows what it caught,
which is the difference between trusting the number and checking it. This is the
view that would have made the fiducial-dot failure obvious in one glance instead
of in a 16-image evaluation run.

**Permanent marks are visually distinct from stains.** They are drawn, because a
person looking at the frame needs to see that the robot SAW the mark; and they
are drawn differently, because the robot deliberately did not clean it, and a
frame that cannot show that difference invites someone to "fix" the omission.

PIL is imported inside the functions, so the core still imports with numpy and
yaml alone and a node that never annotates never pays for the dependency.
"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

from .types import (ESCALATE, HEAVY, KIND_PERMANENT, LIGHT, MODERATE, PASS,
                    PERMANENT, RE_CLEAN, TARGET_CLEAN, ROI3D, SceneAssessment)

logger = logging.getLogger("alix_perception_core.annotate")

#: butterfli's palette, extended. The first three are unchanged so an archived
#: frame and a new one read the same way.
SEVERITY_COLORS = {
    LIGHT: (251, 191, 36),        # amber
    MODERATE: (249, 115, 22),     # orange
    HEAVY: (239, 68, 68),         # red
    PERMANENT: (148, 163, 184),   # slate — real, located, deliberately not cleaned
}
REJECTED_COLOR = (107, 114, 128)  # grey: found, then discarded, and why

BANNER_COLORS = {
    PASS: (16, 185, 129),         # green
    RE_CLEAN: (249, 115, 22),     # orange
    TARGET_CLEAN: (59, 130, 246), # blue
    ESCALATE: (239, 68, 68),      # red
}

BANNER_LABELS = {
    PASS: "CLEAN - no actionable stains",
    RE_CLEAN: "RE-CLEAN REQUIRED",
    TARGET_CLEAN: "TARGET-CLEAN - spot stains detected",
    ESCALATE: "ESCALATE TO HUMAN OPERATOR",
}

_FONT_DIR = "/usr/share/fonts/truetype/dejavu"


def _fonts(scale: float):
    """Three sizes, scaled to the image. Falls back to PIL's bitmap font.

    Scaled because the same code annotates a 1920x1080 colour frame and a 640x480
    evaluation crop, and a fixed 18 px label is unreadable on one and covers the
    fixture on the other.
    """
    from PIL import ImageFont

    sizes = (max(10, int(24 * scale)), max(9, int(18 * scale)),
             max(8, int(14 * scale)))
    try:
        return (ImageFont.truetype(f"{_FONT_DIR}/DejaVuSans-Bold.ttf", sizes[0]),
                ImageFont.truetype(f"{_FONT_DIR}/DejaVuSans-Bold.ttf", sizes[1]),
                ImageFont.truetype(f"{_FONT_DIR}/DejaVuSans.ttf", sizes[2]))
    except OSError:
        default = ImageFont.load_default()
        return default, default, default


def _tagged(draw, xy, text, font, fill, bg) -> None:
    """Text on its own plate, so a label is readable over porcelain or shadow."""
    box = draw.textbbox(xy, text, font=font)
    draw.rectangle([box[0] - 4, box[1] - 2, box[2] + 4, box[3] + 2], fill=bg)
    draw.text(xy, text, fill=fill, font=font)


def _draw_roi(draw, roi: ROI3D, index: int, color, fonts, *, dotted: bool,
              note: str = "") -> None:
    if roi.bbox_px is None:
        return
    x0, y0, x1, y1 = (int(round(v)) for v in roi.bbox_px)
    _, label_font, small_font = fonts

    if dotted:
        # A rejected detection is drawn as a thin dashed outline: present and
        # legible, but impossible to mistake for something the robot acted on.
        step = 10
        for x in range(x0, x1, step * 2):
            draw.line([x, y0, min(x + step, x1), y0], fill=(*color, 200), width=2)
            draw.line([x, y1, min(x + step, x1), y1], fill=(*color, 200), width=2)
        for y in range(y0, y1, step * 2):
            draw.line([x0, y, x0, min(y + step, y1)], fill=(*color, 200), width=2)
            draw.line([x1, y, x1, min(y + step, y1)], fill=(*color, 200), width=2)
    else:
        draw.rectangle([x0, y0, x1, y1], fill=(*color, 55),
                       outline=(*color, 240), width=3)
        cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
        draw.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], fill=(*color, 255))

    head = f"#{index} {roi.label or roi.region or 'detection'}"
    tail = note or (f"{roi.region or 'UNROUTED'}  -  {roi.severity.upper()}  -  "
                    f"{int(roi.confidence * 100)}%  -  "
                    f"{roi.depth_m:.2f} m ({roi.depth_source})")
    top = max(y0 - 2 * label_font.size - 8, 0)
    _tagged(draw, (x0, top), head, label_font, "white", (*color, 220))
    _tagged(draw, (x0, top + label_font.size + 4), tail, small_font, "white",
            (0, 0, 0, 180))


def annotate(image: np.ndarray, assessment: SceneAssessment,
             show_rejected: bool = True) -> np.ndarray:
    """Return a copy of `image` with the assessment drawn on it.

    `image` must be the ORIGINAL frame, in the resolution the detections were
    expressed against — `ROI3D.bbox_px` is in its pixels. RGB or BGR both work
    for geometry; the colours are authored for RGB.

    Pure: no file is written, nothing is logged as a side effect. A caller that
    wants it on disk calls `save`.
    """
    from PIL import Image, ImageDraw

    array = np.asarray(image)
    if array.ndim == 2:
        array = np.stack([array] * 3, axis=-1)
    canvas = Image.fromarray(array[:, :, :3].astype(np.uint8), "RGB")
    draw = ImageDraw.Draw(canvas, "RGBA")
    width, height = canvas.size
    fonts = _fonts(min(width / 1280.0, 2.0))
    big_font, label_font, small_font = fonts

    # The fixture box first and underneath, so it reads as context rather than as
    # a detection. Without it a reader cannot tell whether a region was wrong
    # because the band table is wrong or because the fixture was mislocated.
    box = assessment.diagnostics.get("fixture_bbox_px")
    if box:
        draw.rectangle([int(box[0]), int(box[1]), int(box[2]), int(box[3])],
                       outline=(255, 255, 255, 140), width=2)
        _tagged(draw, (int(box[0]), max(int(box[1]) - small_font.size - 6, 0)),
                f"{assessment.fixture_type or '?'} / "
                f"{assessment.fixture_state or '?'}"
                + (" (assumed)" if assessment.diagnostics.get("state_is_default") else ""),
                small_font, "white", (0, 0, 0, 160))

    index = 0
    if show_rejected:
        for roi in assessment.rejected:
            index += 1
            _draw_roi(draw, roi, index, REJECTED_COLOR, fonts, dotted=True,
                      note=f"REJECTED: {roi.rejected_reason}")
    for roi in assessment.permanent:
        index += 1
        _draw_roi(draw, roi, index, SEVERITY_COLORS[PERMANENT], fonts,
                  dotted=False,
                  note=f"{roi.region or 'UNROUTED'}  -  PERMANENT, not cleaned")
    for roi in assessment.rois:
        index += 1
        _draw_roi(draw, roi, index,
                  SEVERITY_COLORS.get(roi.severity, SEVERITY_COLORS[LIGHT]),
                  fonts, dotted=False)

    # ── banner ──────────────────────────────────────────────────────
    banner_h = big_font.size + 30
    colour = BANNER_COLORS.get(assessment.recommendation, (107, 114, 128))
    draw.rectangle([0, 0, width, banner_h], fill=(*colour, 230))
    draw.text((14, 14),
              f"{BANNER_LABELS.get(assessment.recommendation, assessment.recommendation)}"
              f"    {len(assessment.rois)} actionable"
              f"    {assessment.fixture_type or '?'} | "
              f"{assessment.fixture_state or '?'}",
              fill="white", font=big_font)

    if assessment.escalation_reason:
        _tagged(draw, (14, banner_h + 8), assessment.escalation_reason,
                label_font, "white", (239, 68, 68, 230))

    # ── summary bar ─────────────────────────────────────────────────
    bar_h = small_font.size + 24
    draw.rectangle([0, height - bar_h, width, height], fill=(0, 0, 0, 200))
    draw.text((12, height - bar_h + 10), assessment.message[:220],
              fill="white", font=small_font)

    return np.asarray(canvas)


def save(image: np.ndarray, assessment: SceneAssessment, path: str | Path,
         show_rejected: bool = True) -> Path | None:
    """Annotate and write. Returns the path written, or None on failure.

    Never raises: an annotated frame is an aid, and a perception service that
    stops answering because a disk is full has failed at its actual job. The
    failure is logged.
    """
    from PIL import Image

    path = Path(path)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(annotate(image, assessment, show_rejected)).save(path)
    except Exception as exc:                       # noqa: BLE001 — see docstring
        logger.warning("annotated frame not written to %s: %s", path, exc)
        return None
    return path


def save_frame(image: np.ndarray, path: str | Path) -> Path | None:
    """Write the RAW frame, unannotated. Returns the path written, or None.

    butterfli's `perception_node._save_frame` (:750-771). Separate from `save`
    on purpose: the raw frame is the evidence, and the annotation is this
    package's INTERPRETATION of it. Keeping both is what makes a disputed result
    re-runnable — the 294-frame `used_frames` corpus exists because of this, and
    it is the only labelled data either workspace has.
    """
    from PIL import Image

    path = Path(path)
    try:
        array = np.asarray(image)
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(array[:, :, :3].astype(np.uint8)).save(path)
    except Exception as exc:                       # noqa: BLE001
        logger.warning("frame not written to %s: %s", path, exc)
        return None
    return path
