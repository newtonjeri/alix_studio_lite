"""What each arm's controller holds, as Data Management lists it (the owner, 17 Oct: "The options in those
dropdowns and text boxes should come from the Data Management content ... The reload/refresh button should
completely reload the content from the controller"): its programs and its global points, read from the
controller and kept with the time they were read. The lists Lite offers (S2's Stored Program and Global
Point, S1's Program ID, the start poses) come from here, so a program deleted on the controller is gone from
them at the next read (the lab, 17 Oct: S2 still offered program 80, which the controller no longer held,
and nothing it simulated could be played); a refresh reads the controller again, completely. Data
Management's own read, and every live read Lite makes of these lists, keep it up to date. Read only."""
from __future__ import annotations

import threading
import time

from .jobs import UserError

_LOCK = threading.Lock()
_CACHE: dict = {}              # side -> {"at", "read", "programs": [...], "points": [...]}
FRESH_S = 30.0                 # a list read this recently is used as it is; a refresh reads the controller at once


def store(side: str, programs: list | None = None, points: list | None = None) -> None:
    """A read of the controller: its programs ([{id, name, bytes, sent, sha256}]) and/or its global points."""
    with _LOCK:
        c = dict(_CACHE.get(side) or {})
        if programs is not None:
            c["programs"] = [dict(p) for p in programs]
        if points is not None:
            c["points"] = [dict(p) for p in points]
        c["at"], c["read"] = time.time(), time.strftime("%Y-%m-%d %H:%M:%S")
        _CACHE[side] = c


def forget(side: str | None = None) -> None:
    with _LOCK:
        if side is None:
            _CACHE.clear()
        else:
            _CACHE.pop(side, None)


def _idle(app, side: str) -> bool:
    from .robot_standin import CONTROLLER_KINDS

    return not any(j.kind in CONTROLLER_KINDS for j in app.jobs.running(arm=side)) and not app.robot.arm(side).teaching


def file_for(files: dict | None, sha256: str | None, sent_as: str | None = None) -> dict:
    """The project's file of these bytes (``files``: workflow._project_files(..., every=True)); when several
    hold them (a merge made again of the same recordings), the one Lite sent the program as."""
    f = (files or {}).get(sha256) or {}
    stem = lambda n: (n[:-4] if n.lower().endswith(".txt") else n) if n else n        # noqa: E731 - sent as "L_sync"
    if sent_as:
        for g in (f, *(f.get("also") or [])):
            if stem(g.get("name")) == stem(sent_as):
                return g
    return f


def _named(programs: list, files: dict | None, mine: dict | None = None) -> list:
    """Each program Lite sent, with the project's file of those bytes (``files``: {sha256: the project's file});
    ``mine``: Lite's record of what it sent ({id: {name, sha256}}), which names the file it was sent as."""
    def sent_as(p):
        r = (mine or {}).get(p.get("id")) or {}
        return r.get("name") if p.get("sha256") and r.get("sha256") == p.get("sha256") else None

    from ..motion import track

    return [dict(p, file_name=file_for(files, p.get("sha256"), sent_as(p)).get("name"), seconds=track.duration_s(p.get("lines")))
            for p in programs]


def read_now(app, side: str) -> None:
    """The controller's programs and global points, read now (the arm linked and idle)."""
    robot = app.robot
    programs = [dict(id=int(x["id"]), name=x["name"], bytes=x.get("bytes"), sent=bool(x.get("sha256")), sha256=x.get("sha256"),
                     lines=x.get("lines"))
                for x in robot.programs(side) if x.get("name")]
    store(side, programs, robot.global_points(side) or [])


def content(app, side: str, refresh: bool = False, files: dict | None = None) -> dict:
    """{side, live, standin, read, programs, points, note}: the controller's content as last read, read again
    when ``refresh``, or when older than FRESH_S, if the arm is linked and idle (a command that holds the
    controller is never interrupted: the list stays as last read, and ``note`` says so)."""
    robot = app.robot
    a = robot.arm(side)
    if robot._real(side) is None:                   # the stand-in: what Lite sent to it
        programs = [dict(id=int(i), name=r.get("stored_name") or r.get("name"), bytes=r.get("bytes"), sent=True, sha256=r.get("sha256"),
                         lines=r.get("lines"))
                    for i, r in sorted(a.programs.items())]
        return dict(side=side, live=False, standin=True, read=None, programs=_named(programs, files, a.programs), points=[],
                    note="The stand-in robot has no controller: the programs Lite sent to it are listed.")
    c, note = _CACHE.get(side), None
    stale = not c or refresh or time.time() - c.get("at", 0) > FRESH_S
    if a.connected and stale:
        if _idle(app, side):
            try:
                read_now(app, side)
            except UserError as exc:                # a command holds the controller: as last read
                note = f"{exc} As last read."
            c = _CACHE.get(side)
        elif refresh:
            note = f"The {side.upper()} arm is busy: its controller is read once it has stopped. As last read."
    elif not a.connected:
        note = f"Connect the {side.upper()} arm to read its controller." + (" As last read." if c else "")
    c = c or {}
    return dict(side=side, live=bool(a.connected), standin=False, read=c.get("read"), programs=_named(c.get("programs") or [], files, a.programs),
                points=c.get("points") or [], note=note)
