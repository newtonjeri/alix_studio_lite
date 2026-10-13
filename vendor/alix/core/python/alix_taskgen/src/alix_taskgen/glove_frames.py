"""Glove cleaning-area + contact-axis definitions (single editable source).

Reads ``config/glove_frames.yaml`` and resolves, per IK (tool) frame:

* ``contact_axis`` — the tool axis pressed into the surface ("z"/"x"),
* ``footprint``    — ``(in_plane_width, in_plane_height)`` of the contact patch (m),
* ``depth_tol``    — contact tolerance (m) = the glove **thickness** (YAML key
  ``thickness_m``; the legacy ``depth_tol_m`` is still accepted).

Resolution order for each property: the frame's own entry → the file ``defaults``
→ a hardcoded fallback. So coverage stays correct even for frames not listed,
and editing the YAML is all that's needed to tune the cleaning area.
"""

from __future__ import annotations

import logging
from functools import lru_cache
from pathlib import Path

import yaml

logger = logging.getLogger("alix_taskgen.glove_frames")

GLOVE_FRAMES_PATH = Path(__file__).resolve().parent / "config" / "glove_frames.yaml"

# Hardcoded last-resort fallback (used only if the YAML is missing/incomplete).
# Matches config/glove_frames.yaml defaults (3.5 x 8 cm patch) — the yaml is
# authoritative; this fallback had drifted to 2.5x7/4.5x7 (F73 audit) and
# silently changed coverage whenever the yaml failed to load.
_FALLBACK_FOOTPRINT = {"z": (0.035, 0.08), "x": (0.035, 0.08)}
_FALLBACK_DEPTH_TOL = 0.02


def axis_letter(contact_axis) -> str:
    """Bare axis letter of a possibly-signed contact axis ("-x" -> "x")."""
    return str(contact_axis).strip().lstrip("+-").lower() or "z"


def axis_sign(contact_axis) -> float:
    """+1.0 / -1.0 press direction along :func:`axis_letter`'s axis.

    Unsigned axes mean positive, which is what every frame except
    ``L_glove_frame_1`` uses — see the sign rationale in glove_frames.yaml.
    """
    return -1.0 if str(contact_axis).strip().startswith("-") else 1.0


@lru_cache(maxsize=1)
def _load(path_str: str) -> dict:
    path = Path(path_str)
    if not path.is_file():
        logger.warning("glove_frames.yaml not found at %s; using built-in defaults", path)
        return {}
    return yaml.safe_load(path.read_text()) or {}


def load(path: str | None = None) -> dict:
    """Return the parsed glove_frames.yaml ({frames, defaults})."""
    return _load(str(path or GLOVE_FRAMES_PATH))


def reload() -> None:
    """Drop the cached YAML (call after editing the file at runtime)."""
    _load.cache_clear()


def _contact_axis_from_name(ik_frame: str) -> str:
    """Legacy heuristic: ``*_glove_frame_2`` presses Z, everything else X.

    The X-pressing frames are arm-mirrored, so the left one presses ``-x``
    (glove_frames.yaml documents why). Only reached when the YAML is missing or
    does not list the frame.
    """
    name = str(ik_frame).strip().strip('"')
    if name.endswith("_2"):
        return "z"
    return "-x" if name.startswith("L_") else "x"


def resolve(ik_frame: str, path: str | None = None) -> dict:
    """Return ``{contact_axis, footprint, depth_tol}`` for an IK frame."""
    data = _load(str(path or GLOVE_FRAMES_PATH))
    frames = data.get("frames", {}) or {}
    defaults = data.get("defaults", {}) or {}
    entry = frames.get(str(ik_frame), {}) or {}

    contact_axis = entry.get("contact_axis") or _contact_axis_from_name(ik_frame)

    # Footprint/default lookups are keyed on the bare letter: the contact patch
    # is the same rectangle whichever way the glove faces.
    letter = axis_letter(contact_axis)

    footprint = entry.get("footprint_m")
    if footprint is None:
        by_axis = (defaults.get("by_contact_axis", {}) or {}).get(letter, {})
        footprint = by_axis.get("footprint_m") or _FALLBACK_FOOTPRINT.get(
            letter, _FALLBACK_FOOTPRINT["z"]
        )

    # The glove "thickness" is the contact band (a.k.a. depth tolerance).
    # Prefer thickness_m; accept the legacy depth_tol_m key for back-compat.
    depth_tol = entry.get("thickness_m", entry.get("depth_tol_m"))
    if depth_tol is None:
        depth_tol = defaults.get("thickness_m", defaults.get("depth_tol_m", _FALLBACK_DEPTH_TOL))

    return {
        "contact_axis": contact_axis,
        "axis_letter": letter,
        "axis_sign": axis_sign(contact_axis),
        "footprint": (float(footprint[0]), float(footprint[1])),
        "depth_tol": float(depth_tol),
    }


def footprint_for_axis(axis: str, path: str | None = None):
    """Default contact footprint (W, H) for a contact axis, from the YAML's
    ``defaults.by_contact_axis`` (fallback: built-in constants). Lets callers
    that only know the axis (coverage) share the tuned numbers."""
    data = _load(str(path or GLOVE_FRAMES_PATH))
    letter = axis_letter(axis)
    by_axis = ((data.get("defaults", {}) or {}).get("by_contact_axis", {}) or {}).get(letter, {})
    fp = by_axis.get("footprint_m") or _FALLBACK_FOOTPRINT.get(letter, _FALLBACK_FOOTPRINT["z"])
    return (float(fp[0]), float(fp[1]))


def default_depth_tol(path: str | None = None) -> float:
    """Default glove thickness / contact band from the YAML defaults."""
    data = _load(str(path or GLOVE_FRAMES_PATH))
    defaults = data.get("defaults", {}) or {}
    return float(defaults.get("thickness_m", defaults.get("depth_tol_m", _FALLBACK_DEPTH_TOL)))


def contact_axis(ik_frame: str, path: str | None = None) -> str:
    return resolve(ik_frame, path)["contact_axis"]


def footprint(ik_frame: str, path: str | None = None):
    return resolve(ik_frame, path)["footprint"]


def hand_state(ik_frame: str, path: str | None = None, default: str = "open_tenth") -> str:
    """SRDF group state the inspire hand takes to present this frame's pad."""
    data = _load(str(path or GLOVE_FRAMES_PATH))
    entry = (data.get("frames", {}) or {}).get(str(ik_frame), {}) or {}
    return str(entry.get("hand_state") or default)


def hand_state_for_frames(ik_frames, path: str | None = None,
                          default: str = "open_tenth") -> str:
    """ONE hand state for a task that may present several pads.

    The hand is re-posed for a pad, not for a waypoint, so a task that switches
    ik_frame between bands must not re-grasp at the seam — a mid-path hand move
    is a stage the arm has to hold still for, right where it is closest to the
    fixture. Newton's rule (2026-08-17): **hold the FIRST used frame's state**.

    Within `palm_pads` (frames 1+2) this costs nothing — both now declare
    `open_tenth`, measured 9.1% apart in alix.srdf before they were unified.
    Within `finger_pads` (frames 3+4) it is a real choice: `half_grasp` and
    `quarter_grasp` are 42.4% apart, so the second pad is presented at the
    first pad's finger pose. That is deliberate — one settled hand beats a
    re-pose mid-task — but it means a frames-3+4 task should be authored with
    the more demanding pad FIRST.
    """
    for frame in ik_frames or ():
        if frame:
            return hand_state(frame, path, default)
    return default
