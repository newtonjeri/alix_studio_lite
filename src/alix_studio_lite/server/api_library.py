"""The Trajectory Viewer's routes, ``/api/library…``: taught .txt files to upload, check and replay.

    GET    /api/library                       uploads and built-in sets
    POST   /api/library                       upload one file {name, content, arm?, path?}
    POST   /api/library/<id>                  set an upload's arm or name
    DELETE /api/library/<id>                  move an upload to the library's trash
    GET    /api/library/<id>/track            joint data for the plot
    GET    /api/library/<id>/frames           3D frames for the arm it was taught on
    GET    /api/library/<id>/checks           every limit on the file as taught
    POST   /api/library/<id>/use              make it a region's recording {pid, key}

Nothing here moves the robot; every route needs the session.
"""
from __future__ import annotations

import math
from pathlib import Path

from . import workers
from .api_units import unit_view, units_of
from .api_v2 import session
from .app import route
from .jobs import UserError

L = r"/api/library/([us]_[0-9a-f]{12})"


def _arm(meta: dict, q) -> str:
    side = (q.get("side") or [meta.get("arm")])[0]
    if side not in ("left", "right"):
        raise UserError("Say which arm this file was taught on: left or right.", code="arm")
    return side


@route("GET", "/api/library")
def get_library(app, q, body, m):
    session(app)
    return app.library.listing()


@route("POST", "/api/library")
def add_file(app, q, body, m):
    session(app)
    meta = app.library.add(body.get("name"), body.get("content"), arm=body.get("arm") or None, path=body.get("path"))
    return app.library.listing() | {"added": meta["id"]}


@route("POST", L)
def update_file(app, q, body, m):
    session(app)
    app.library.update(m.group(1), arm=body.get("arm"), name=body.get("name"))
    return app.library.listing()


@route("DELETE", L)
def delete_file(app, q, body, m):
    session(app)
    app.library.delete(m.group(1))
    return app.library.listing()


@route("GET", L + "/track")
def get_track(app, q, body, m):
    session(app)
    return workers.track_arrays(app.library.get(m.group(1))["path"])


@route("GET", L + "/frames")
def get_frames(app, q, body, m):
    from ..motion import track
    from . import robot_model
    from .routes_robot import _mesh_links, _payload

    session(app)
    meta = app.library.get(m.group(1))
    side = _arm(meta, q)
    md = track.read_md(Path(meta["path"]))
    n = md.shape[0]
    every = max(int((q.get("every") or ["4"])[0]), math.ceil(n / (robot_model.MAX_TRACK_FRAMES - 1)))
    p = _payload(app)
    out = robot_model.track_frames(side, (md.T / 1000.0).tolist(), every, dict(p["home"]),
                                   robot_model.moving_links(side, _mesh_links(p)))
    out["total_lines"] = int(n)
    return out


@route("GET", L + "/checks")
def get_checks(app, q, body, m):
    session(app)
    meta = app.library.get(m.group(1))
    return app.library.checks(meta["id"], _arm(meta, q))


@route("POST", L + "/use")
def use_file(app, q, body, m):
    """Make the file a region's active recording; the region's arm must be the file's."""
    meta = app.library.get(m.group(1))
    units = units_of(app, str(body.get("pid") or ""))
    key = str(body.get("key") or "")
    if key not in units.units:
        raise UserError(f"There is no region {key!r} in this project.")
    u = units.units[key]
    if meta.get("arm") and meta["arm"] != u["arm"]:
        raise UserError(f"{meta['name']} was taught on the {meta['arm'].upper()} arm; {key} is for the "
                        f"{u['arm'].upper()} arm.")
    units.add_recording(key, Path(meta["path"]), note=f"from the Trajectory Viewer: {meta['name']}",
                        source="upload", name=meta["name"])
    return {"unit": unit_view(app, units, key)}
