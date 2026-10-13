"""The Trajectory Viewer's routes, ``/api/library…``: taught .txt files to upload, check and replay.

    GET    /api/library                       uploads and built-in sets
    POST   /api/library                       upload one file {name, content, arm?, path?, pole_mm?, hand?}
    POST   /api/library/<id>                  set an upload's arm, name, or pole and hand
    DELETE /api/library/<id>                  move an upload to the library's trash
    GET    /api/library/<id>/track            joint data for the plot
    GET    /api/library/<id>/timeline         the replay in 3D: pole, hand, move to start, trajectory
    GET    /api/library/<id>/checks           every limit on the file as taught
    POST   /api/library/<id>/use              make it a region's recording {pid, key}

A file replays, and becomes a region's recording, only on the arm it was taught on and with its
stage (pole height and hand pose) known (``library.TaughtLibrary.require_ready``). Nothing here
moves the robot; every route needs the session.
"""
from __future__ import annotations

from pathlib import Path

from . import staging, workers
from .api_units import unit_view, units_of
from .api_v2 import session
from .app import route
from .jobs import UserError

L = r"/api/library/([us]_[0-9a-f]{12})"


@route("GET", "/api/library")
def get_library(app, q, body, m):
    session(app)
    return app.library.listing()


@route("POST", "/api/library")
def add_file(app, q, body, m):
    session(app)
    meta = app.library.add(body.get("name"), body.get("content"), arm=body.get("arm") or None, path=body.get("path"),
                           pole_mm=body.get("pole_mm"), hand=body.get("hand"))
    return app.library.listing() | {"added": meta["id"]}


@route("POST", L)
def update_file(app, q, body, m):
    session(app)
    app.library.update(m.group(1), arm=body.get("arm"), name=body.get("name"), pole_mm=body.get("pole_mm"),
                       hand=body.get("hand"))
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


@route("GET", L + "/timeline")
def get_timeline(app, q, body, m):
    """The replay as the robot would do it, on the file's own arm: pole to its height, hand to its
    pose, arm to the first line, then the trajectory (``staging.execution_timeline``)."""
    from ..motion import track
    from ..motion.limits import load_limits
    from .routes_robot import _mesh_links, _payload

    session(app)
    meta = app.library.require_ready(m.group(1), (q.get("side") or [None])[0])
    side = meta["arm"]
    p = _payload(app)
    return staging.execution_timeline(side, track.read_md(Path(meta["path"])), staging.stage_from(meta["stage"]),
                                      home=dict(p["home"]), links=staging.staged_links(side, _mesh_links(p)),
                                      rating_deg_s=load_limits(side).rating_deg_s)


@route("GET", L + "/checks")
def get_checks(app, q, body, m):
    session(app)
    return app.library.checks(m.group(1))


@route("POST", L + "/use")
def use_file(app, q, body, m):
    """Make the file a region's active recording; the region's arm must be the file's."""
    units = units_of(app, str(body.get("pid") or ""))
    key = str(body.get("key") or "")
    if key not in units.units:
        raise UserError(f"There is no region {key!r} in this project.")
    meta = app.library.require_ready(m.group(1), units.units[key]["arm"])
    units.add_recording(key, Path(meta["path"]), note=f"from the Trajectory Viewer: {meta['name']}",
                        source="upload", name=meta["name"], taught_arm=meta["arm"], stage=meta["stage"])
    return {"unit": unit_view(app, units, key)}
