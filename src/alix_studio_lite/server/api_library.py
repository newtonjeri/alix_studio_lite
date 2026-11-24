"""The Trajectory Viewer's routes, ``/api/library…``: taught .txt files to upload, check and replay.

    GET    /api/library                       uploads and built-in sets
    POST   /api/library                       upload one file {name, content, arm?, path?, pole_mm?, hand?}
    POST   /api/library/<id>                  set an upload's arm, name, or pole and hand
    DELETE /api/library/<id>                  move an upload to the library's trash
    GET    /api/library/<id>/track            joint data for the plot
    GET    /api/library/<id>/timeline         the replay in 3D: pole, hand, move to start, trajectory
    GET    /api/library/<id>/checks           every limit on the file as taught
    POST   /api/library/<id>/use              make it a region's recording {pid, key}
    GET    /api/library/programs              the programs on both arms' controllers, to search {pid?, refresh?}
    POST   /api/library/programs/<arm>/<n>    state the pole and hand of one nothing is known of {pid, pole_mm, hand}
    GET    /api/library/programs/<arm>/<n>/timeline, /track, /checks    one of them, replayed as a file is {pid?}

A file replays, and becomes a region's recording, only on the arm it was taught on and with its
stage (pole height and hand pose) known (``library.TaughtLibrary.require_ready``). A program on a
controller (the owner, 21 Oct: "The tab can search the uploaded tasks") replays only when Lite sent
it there: its own pole and hand are Lite's record of what it sent, and its file the very bytes sent
(``_program_meta``); the Viewer runs it from its slot through Advanced's route
(``POST /api/robots/<arm>/programs/run``), which stages that same pole and hand first. Nothing here
moves the robot; every route needs the session.

A program Lite did not send is listed with the pole and hand it runs with all the same, where they are known (the
user, 15 Nov; ``program_stage``): its file's, found by its name among your files here, or the ones the project keeps
for it (program_links: another computer's record or file, or stated here). One nothing is known of can have them
stated on its row (``set_program_stage``): they are kept with the project, so S2 and Run set them before it on every
computer that runs the project.
"""
from __future__ import annotations

import re
from pathlib import Path

from . import staging, workers
from .api_units import unit_view, units_of
from .api_v2 import session
from .app import route
from .jobs import UserError

L = r"/api/library/([us]_[0-9a-f]{12})"
P = r"/api/library/programs/(left|right)/(\d{1,3})"
PROGRAM_ID = re.compile(r"^p_(left|right)_(\d{1,3})$")      # a program's id in the Viewer's slots


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
    session(app)
    return _timeline(app, app.library.require_ready(m.group(1), (q.get("side") or [None])[0]))


def _timeline(app, meta: dict) -> dict:
    from ..motion import track
    from ..motion.limits import load_limits
    from .routes_robot import _mesh_links, _payload

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
    metas = {s: _ready(app, str(body[s]), s, body.get("pid")) for s in ("left", "right") if body.get(s)}
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


# ── the programs on the arms' controllers (the owner, 21 Oct: "The tab can search the uploaded tasks"; "run a saved
# project, from the slot number where it is stored, on the real hardware")

def _ready(app, fid: str, side: str, pid=None) -> dict:
    """A slot's file for the collision check: a library file, or a program on a controller (``p_<arm>_<n>``)."""
    hit = PROGRAM_ID.match(fid)
    if not hit:
        return app.library.require_ready(fid, side)
    if hit.group(1) != side:
        raise UserError(f"Program {hit.group(2)} is on the {hit.group(1).upper()} controller: it is never shown on the "
                        f"{side.upper()} arm.", code="arm")
    return _program_meta(app, side, int(hit.group(2)), pid)


def _units(app, pid):
    """The open project's units (``pid``), or None."""
    if not pid:
        return None
    try:
        return units_of(app, str(pid))
    except UserError:
        return None


def _files(app, side: str, pid) -> dict | None:
    """The open project's files of an arm by their bytes (workflow._project_files), or None."""
    if not pid:
        return None
    from . import workflow

    try:
        return workflow._project_files(units_of(app, str(pid)), side, every=True)
    except UserError:
        return None


def _program_file(app, rec: dict, files: dict | None, *, verify: bool) -> dict | None:
    """The file of a program Lite sent (``rec``: its record of it), wherever it is now: the very file it sent (the
    record's path, while it holds the same bytes), else a file of those bytes in the open project (``files``), in your
    files in this Viewer, or in the built-in sets. {path, name, where}, or None. ``verify``: the record's own file is
    compared by its bytes (sha256), not only its size (the list)."""
    from ..motion import track
    from . import content, library

    sha = rec.get("sha256")
    if not sha:
        return None
    own = rec.get("path")
    if own and Path(own).is_file():
        try:
            same = track.sha256_file(own) == sha if verify else Path(own).stat().st_size == int(rec.get("bytes") or -1)
        except (OSError, ValueError):
            same = False
        if same:
            return dict(path=str(own), name=Path(own).name, where="the file Lite sent")
    f = content.file_for(files, sha, rec.get("name"))
    if f.get("path") and Path(f["path"]).is_file():
        return dict(path=str(f["path"]), name=f.get("name") or Path(f["path"]).name, where="the open project's file of those bytes")
    for u in app.library.uploads():
        if u.get("sha256") == sha:
            return dict(path=str(app.library.root / f"{u['id']}.txt"), name=u["name"], where="your file of those bytes in this Viewer")
    b = library.builtin_by_sha(sha)
    if b:
        return dict(path=str(b["path"]), name=b["name"], where="the built-in file of those bytes")
    return None


def _real_list(app, side: str) -> bool:
    """Whether the arm's list is its real controller's, read with the arm connected: a program's pole and hand are kept
    in the project for the program as that list names it (program_links)."""
    from . import program_mirror

    return program_mirror.real_controller(app, side) and bool(app.robot.arm(side).connected)


def _slot_info(app, side: str, p: dict) -> dict:
    """What Lite knows of a listed program's slot, as program_links and workflow.found_by_name take it."""
    rec = app.robot.arm(side).programs.get(int(p["id"])) or {}
    ours = bool(p.get("sha256")) and rec.get("sha256") == p.get("sha256")
    held = dict(name=p.get("name"), bytes=p.get("bytes")) if _real_list(app, side) and p.get("name") else None
    return dict(id=int(p["id"]), name=p.get("name"), sent=ours, stage=rec.get("stage") if ours else None,
                sha256=p.get("sha256") if ours else None, **({"held": held} if held else {}))


def program_stage(app, side: str, p: dict, units=None, *, named: dict | None = None, links: dict | None = None) -> dict | None:
    """The pole and hand a listed program runs with and where they are from (workflow.own_stage: Lite's record of
    sending it; the project's entry by another computer's record; its file found by its name here; the project's entry
    by a file or stated by hand), or None when nothing is known of them. ``units``: the open project (its files and
    its entries); ``named``, ``links``: read once for a whole list."""
    from . import program_links, workflow

    info = _slot_info(app, side, p)
    found = None
    if staging.stage_from(info.get("stage")) is None:      # as S2 looks (workflow.own_of): one Lite sent, by its bytes; another,
        named = workflow.files_by_name(app, units, side) if named is None else named       # by its name, not compared by size
        found = workflow.found_by_name(named, int(p["id"]), {}, dict(name=p.get("name"), sent=info["sent"],   # here (a list of
                                                                     sha256=info.get("sha256")))              # thirty programs)
    if links is None:
        links = program_links.load(units, side) if units is not None else {}
    return workflow.own_stage(info, found, program_links.link_for(links, int(p["id"]), info))


def _stage_words(got: dict) -> str:
    """Where a program's pole and hand are from, for its row."""
    from . import program_links

    if got["how"] == "record":
        return "as Lite sent it"
    if got["how"] == "file":
        return f"of its file found by its name ({got.get('file_name')})"
    return f"kept with the project: {program_links.where_from(got['link'])}"


def _program_row(app, side: str, p: dict, files: dict | None, *, verify: bool = False, units=None, named: dict | None = None,
                 links: dict | None = None) -> tuple[dict, dict | None]:
    """One program as the Viewer lists it (``p``: content.content's), with Lite's record of what it sent there when the
    slot still holds it: its own pole and hand (the ones a run from the slot stages first) and whether Lite has its file.
    One Lite did not send, or sent without them: the pole and hand it runs with where they are known otherwise
    (``program_stage``; ``stage_by``, ``stage_from``), and whether they can be stated on its row (``can_state``: nothing
    known of them, or stated before; a real controller's list and an open project). (row, its file)."""
    rec = app.robot.arm(side).programs.get(int(p["id"])) or {}
    ours = bool(p.get("sha256")) and rec.get("sha256") == p.get("sha256")
    got = program_stage(app, side, p, units, named=named, links=links)
    st = got["stage"] if got else None
    own = bool(got and got["how"] == "record")       # Lite's record: the only one it shows a program with
    f = _program_file(app, rec, files, verify=verify) if ours else None
    from . import program_links

    stated = bool(got and got["how"] == "link" and program_links.by_hand(got["link"]))   # stated, or stated then set by its row
    can = (not got or stated) and _real_list(app, side) and units is not None
    runs = f"It runs with {staging.describe(st)} ({_stage_words(got)})." if got and not own else None
    if ours and own and f:
        why = None
    elif ours and own:
        why = "Its file is no longer on this computer: it can be run, not shown."
    elif not ours:
        why = (f"Lite did not send it here, so it is not shown. {runs}" if runs else
               "Lite did not send it here: its file, pole and hand are not known. "
               + ("Set its pole and hand on this row, or Run asks for them." if can else "Run asks for its pole and hand."))
    else:
        why = (f"Lite sent it without its pole and hand. {runs}" if runs else
               "Lite sent it without its pole and hand: " + ("set them on this row, or Run asks for them." if can else "Run asks for them."))
    row = dict(id=int(p["id"]), name=p.get("name"), lines=p.get("lines") if ours else None, seconds=p.get("seconds") if ours else None,
               bytes=p.get("bytes"), sent=ours, stage=st.as_dict() if st else None, file_name=(f or {}).get("name"),
               uploaded=rec.get("uploaded") if ours else None, viewable=bool(ours and own and f), why=why,
               stage_by=(got["link"].get("by") if got["how"] == "link" else got["how"]) if got else None,
               stage_from=_stage_words(got) if got else None, can_state=bool(can))
    return row, f


def _program_meta(app, side: str, n: int, pid=None) -> dict:
    """A program on the arm's controller as the Viewer replays it, like a file ({id, name, arm, stage, path}): Lite's
    record of what it sent to that slot, while the slot holds it; refused, saying why, otherwise."""
    from . import content

    if not 1 <= int(n) <= 100:
        raise UserError("Program numbers are 1 to 100.")
    files = _files(app, side, pid)
    c = content.content(app, side, files=files, units=_units(app, pid))
    p = next((x for x in c["programs"] if int(x["id"]) == int(n)), None)
    if p is None:
        raise UserError(f"The {side.upper()} controller holds no program {n}" + (f": {c['note']}" if c.get("note") else "."),
                        "Refresh the list.")
    row, f = _program_row(app, side, p, files, verify=True)
    if not row["viewable"]:
        raise UserError(f"Program {n} ({row['name']}) cannot be shown: {row['why']}", code="stage" if row["sent"] and not row["stage"] else None)
    return dict(id=f"p_{side}_{int(n)}", name=row["name"], arm=side, stage=row["stage"], path=f["path"], program=int(n),
                file_name=f["name"], where=f["where"], source="program")


@route("GET", "/api/library/programs")
def get_controller_programs(app, q, body, m):
    """Both arms' controllers' programs, for the Viewer's search: each as last read (content.py: read again when the
    arm is linked and idle, or ``refresh``), with what Lite knows of each (``_program_row``)."""
    from . import content

    session(app)
    pid = (q.get("pid") or [None])[0]
    refresh = str((q.get("refresh") or [""])[0]) in ("1", "true")
    out = {}
    from . import program_links, workflow

    units = _units(app, pid)
    for side in ("left", "right"):
        files = _files(app, side, pid)
        c = content.content(app, side, refresh=refresh, files=files, units=units)
        named = workflow.files_by_name(app, units, side)
        links = program_links.load(units, side) if units is not None else {}
        out[side] = dict(connected=bool(app.robot.arm(side).connected), live=c["live"], standin=c["standin"], read=c["read"],
                         note=c["note"], programs=[_program_row(app, side, p, files, units=units, named=named, links=links)[0]
                                                   for p in c["programs"]])
    return {"arms": out, "backend": app.robot.backend, "project": units is not None}


@route("POST", P)
def set_program_stage(app, q, body, m):
    """The operator states the pole and hand of a program on a controller that nothing is known of (the user, 15 Nov:
    "the read programs from the controller can have if no matcher ... their pole/hand values set manually in the
    trajectory view"): kept with the open project for the program as its controller lists it (program_links.state), so
    S2 and Run set them before it, here and on every computer that runs the project. Refused for a program whose pole
    and hand are known already (Lite's record, its file, the project's entry from one of them): a program runs with
    its own. The Program's Pole row in the project's sequence takes the pole stated (``rows``: workflow.follow_pole; the
    user, 24 Nov: both places hold one value)."""
    from . import content, program_links

    session(app)
    side, n = m.group(1), int(m.group(2))
    S = side.upper()
    units = _units(app, body.get("pid"))
    if units is None:
        raise UserError("Open a project first: a program's pole and hand are kept with the project, so that every computer "
                        "that runs it has them.", code="project")
    if not _real_list(app, side):
        raise UserError(f"The {S} arm's controller is not read: a program's pole and hand are kept for the program as its "
                        "controller lists it.", "Connect the arm (Lite started with --robot realman), then refresh the list.",
                        code="connect")
    c = content.content(app, side, refresh=True, files=_files(app, side, body.get("pid")), units=units)
    p = next((x for x in c["programs"] if int(x["id"]) == n), None)
    if p is None or not p.get("name"):
        raise UserError(f"The {S} controller holds no program {n}" + (f": {c['note']}" if c.get("note") else "."), "Refresh the list.")
    got = program_stage(app, side, p, units)
    if got and not (got["how"] == "link" and program_links.by_hand(got["link"])):
        raise UserError(f"Program {n} ({p['name']}) runs with its own pole and hand: {staging.describe(got['stage'])} "
                        f"({_stage_words(got)}). They are not replaced by hand.",
                        "To change its pole, change its Pole row in the sequence. To change the ones of a file, set them on "
                        "the file's row.", code="own")
    kept = program_links.state(units, side, n, dict(name=p["name"], bytes=p.get("bytes")), body.get("pole_mm"), body.get("hand"))
    from . import workflow

    rows = workflow.follow_pole(units, side, n, kept["pole_mm"])     # one value: its Pole row in the sequence follows
    content.forget(side)                             # the list is read again: the row shows what was stated
    return {"stated": kept, "rows": rows} | get_controller_programs(app, {"pid": [str(body.get("pid"))]}, {}, None)


@route("GET", P + "/timeline")
def get_program_timeline(app, q, body, m):
    session(app)
    return _timeline(app, _program_meta(app, m.group(1), int(m.group(2)), (q.get("pid") or [None])[0])) | {"program": int(m.group(2))}


@route("GET", P + "/track")
def get_program_track(app, q, body, m):
    session(app)
    return workers.track_arrays(_program_meta(app, m.group(1), int(m.group(2)), (q.get("pid") or [None])[0])["path"])


@route("GET", P + "/checks")
def get_program_checks(app, q, body, m):
    session(app)
    return app.library.checks_of(_program_meta(app, m.group(1), int(m.group(2)), (q.get("pid") or [None])[0]))
