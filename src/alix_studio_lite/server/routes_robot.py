"""Robot model routes: the 3D robot with movable joints. ``robot_model.py`` builds the meshes the way
ALIX Studio does and poses every link with ALIX's FK; ``web/js/robot3d.js`` applies one 4x4 per link."""
from __future__ import annotations

import math
import threading

from . import robot_model
from .app import route
from .jobs import UserError

_LOCK = threading.Lock()
_PAYLOADS: dict = {}                 # cache dir -> payload, so a page load does not re-read the file


def _payload(app) -> dict:
    """The first call on a new machine builds the payload (seconds); the lock makes a second
    caller wait for that build instead of repeating it."""
    cache_dir = app.data_dir / "cache"
    with _LOCK:
        if cache_dir not in _PAYLOADS:
            _PAYLOADS[cache_dir] = robot_model.build_robot_model(cache_dir)
        return _PAYLOADS[cache_dir]


def _mesh_links(p: dict) -> list[str]:
    return [ln for ln, ids in p["links"].items() if ids]


def _values(raw) -> dict:
    try:
        return robot_model.si_values(raw, [j.name for j in robot_model.fk_model().joints_by_child.values()])
    except (ValueError, TypeError) as exc:
        raise UserError(str(exc))


@route("GET", "/api/lite/robot-model")
def get_robot_model(app, q, body, m):
    """Meshes (link frame), ``homePose`` (link -> 16 numbers, row-major), joints with limits, home
    values and the panel's joint groups."""
    return _payload(app)


@route("POST", "/api/lite/robot-pose")
def post_robot_pose(app, q, body, m):
    """``{joints: {name: value}}`` in radians and metres -> ``{pose: link -> 16 numbers}`` for every
    link with a mesh (Studio's poseMap). Joints not given are 0; mimic joints follow their source."""
    p = _payload(app)
    return {"pose": robot_model.pose(_values(body.get("joints") or {}), _mesh_links(p))}


@route("POST", "/api/lite/robot-track-frames")
def post_track_frames(app, q, body, m):
    """A whole arm track in one request: ``{side, joints_deg: [7][N], every: k, joints: {...}}`` ->
    frames of every k-th line and the last, for the links the arm moves. ``joints`` (radians and
    metres) holds the rest of the robot, by default the home pose."""
    side = body.get("side")
    if side not in ("left", "right"):
        raise UserError("Say which arm: left or right.")
    q7 = body.get("joints_deg")
    if not isinstance(q7, list) or len(q7) != 7 or not all(isinstance(r, list) for r in q7):
        raise UserError("joints_deg must be 7 lists of joint angles in degrees, one per joint.")
    n = len(q7[0])
    if not n or any(len(r) != n for r in q7):
        raise UserError("The 7 joint lists must have the same number of lines.")
    try:
        q7 = [[float(v) for v in r] for r in q7]
    except (TypeError, ValueError):
        raise UserError("joints_deg must hold numbers.")
    if not all(math.isfinite(v) for r in q7 for v in r):
        raise UserError("joints_deg must hold numbers.")
    try:
        every = int(body.get("every") or 1)
    except (TypeError, ValueError):
        raise UserError("every must be a whole number of lines.")
    if every < 1:
        raise UserError("every must be 1 or more.")
    if math.ceil(n / every) + 1 > robot_model.MAX_TRACK_FRAMES:
        raise UserError(f"That is more than {robot_model.MAX_TRACK_FRAMES} frames.",
                        f"Use every={math.ceil(n / (robot_model.MAX_TRACK_FRAMES - 1))} or more.")
    p = _payload(app)
    base = dict(p["home"])
    base.update(_values(body.get("joints") or {}))
    links = robot_model.moving_links(side, _mesh_links(p))
    return robot_model.track_frames(side, q7, every, base, links)
