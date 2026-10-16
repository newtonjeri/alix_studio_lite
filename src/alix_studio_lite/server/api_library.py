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


@route("POST", "/api/library/collisions")
def post_pair_collisions(app, q, body, m):
    """The Trajectory Viewer's collision check, on demand (the owner, 16 Oct): its LEFT and RIGHT files
    (either may be left out) as it replays them, each arm staging its pole and hand, the arm ready first
    waiting at its start, both trajectories starting on one tick; every 20 ms, each arm against the walls
    and the commode in ``phase`` (the viewer's choice), and the two arms against each other
    (collision_check.check_states). ``pid``: the open project, whose room is used (else ALIX's cubicle).
    Times are on the viewer's clock (0 = both arms standing)."""
    import numpy as np

    from ..motion import track
    from ..motion.limits import load_limits
    from . import collision_check, placement, workflow
    from .lite_units import PHASES
    from .routes_robot import _payload

    session(app)
    metas = {s: app.library.require_ready(str(body[s]), s) for s in ("left", "right") if body.get(s)}
    if not metas:
        raise UserError("Load a LEFT or a RIGHT file first.")
    phase = str(body.get("phase") or "lid_closed")
    if phase not in PHASES:
        raise UserError("The commode's state is lid closed, lid open or seat open.")
    home = dict(_payload(app)["home"])
    fid, room = "commode_c", None
    if body.get("pid"):
        units = units_of(app, str(body["pid"]))
        fid, room = units.fixture, placement.load(units)["room"]
    clocks = {}
    for side, meta in metas.items():
        md = track.read_md(Path(meta["path"]))
        st = staging.stage_from(meta["stage"])
        clocks[side] = (meta, md, st, staging.execution_times(side, md.astype(float) / 1000.0, st, home, load_limits(side).rating_deg_s))
    T = max(c[3]["traj_t0"] for c in clocks.values())          # both trajectories start here
    acts, starts, total = {}, {}, 0.0
    for side, (meta, md, st, c) in clocks.items():
        arm0 = [float(v) for v in c["arm0"]]
        first, last = (md[0] / 1000.0).tolist(), (md[-1] / 1000.0).tolist()
        pole0, hand0, hand1, P = float(c["pole0"]), list(c["hand0"]), list(c["hand1"]), float(st.pole_mm)
        t1, t2, t3 = c["t_pole"], c["t_pole"] + c["t_hand"], c["traj_t0"]
        a = [dict(kind="pole", t0=0.0, t1=t1, pole0=pole0, pole1=P, hand0=hand0, hand1=hand0, q0=arm0, q1=arm0),
             dict(kind="hand", t0=t1, t1=t2, pole0=P, pole1=P, hand0=hand0, hand1=hand1, q0=arm0, q1=arm0),
             dict(kind="global", drawn=True, move_s=c["t_move"], t0=t2, t1=t3, pole0=P, pole1=P, hand0=hand1, hand1=hand1, q0=arm0, q1=first)]
        if T > t3:
            a.append(dict(kind="wait", t0=t3, t1=T, pole0=P, pole1=P, hand0=hand1, hand1=hand1, q0=first, q1=first))
        a.append(dict(kind="program", drawn=True, file=meta["path"], t0=T, t1=T + c["traj_s"], traj_t0=T, pole0=P, pole1=P,
                      hand0=hand1, hand1=hand1, q0=first, q1=last))
        acts[side], starts[side] = a, dict(pole_mm=pole0, hand=hand0, q_deg=arm0)
        total = max(total, T + c["traj_s"])
    times = np.arange(0.0, total + 1e-9, workflow.SAMPLE_S).tolist()
    md_of = workflow._md_reader()
    arms = {}
    for side in acts:
        pole, hand, qq = workflow.sample(acts[side], times, starts[side], md_of)
        arms[side] = dict(pole=pole.tolist(), hand=hand.tolist(), q=qq.tolist())
    base = dict(home)
    for side in acts:
        base = staging.start_values(side, base, starts[side])
    scene = placement.scene(fid, room, None)
    cub = int(app.ui.get("cubicle_mm", 1400))
    names = " and ".join(f"{s.upper()} {clocks[s][0]['name']}" for s in clocks)

    def work(job):
        job.progress(f"Checking {names}: {len(times):,} moments", 0, 1)
        fut = workers.pool().submit(collision_check.check_states, times, arms, base, scene, fid, [phase] * len(times), cub)
        while not fut.done():
            job.sleep(0.2)
        return dict(fut.result(), start_s=round(T, 4), total_s=round(total, 4), phase=phase,
                    files={s: clocks[s][0]["name"] for s in clocks})

    return app.jobs.start("collisions", f"Collision check: {names}", work).view()


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
