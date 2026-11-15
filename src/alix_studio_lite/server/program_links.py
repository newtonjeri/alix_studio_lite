"""Each program on an arm's controller linked to the pole height and hand pose it runs with, kept in the project (the
user, 15 Nov: "link a program to its taught pole/hand value, or the pole and hand value set during upload in the
trajectory viewer, or say when a new arm is connected, the read programs from the controller can have if no matcher
... their pole/hand values set manually in the trajectory view").

Why: a controller stores a program's points only. The pole and hand it runs with were known to one computer alone: the
one that sent it (Lite's record of what it sent, in its data folder) or that holds a file of its name with them stated
(the Trajectory Viewer's uploads, in its data folder too). A second computer on the same arms knew neither: on the
Jetson (14-15 Nov) S2 ran the grip programs with the hands open, because it set no pole or hand before them.

Written to ``<project>/controller_exports/<arm>/pole_and_hand.json``, one entry per slot: the program as its
controller lists it (name, stored bytes), its pole and hand, and where they are from (``by``):

- ``record``: Lite's record of sending it: the ones it was taught or sent with;
- ``file``: its file found by its name (the pole and hand stated for it, as at its upload in the Trajectory Viewer);
- ``stated``: stated by hand in the Trajectory Viewer, for a program nothing else is known of.

A computer that knows a program's pole and hand writes them here whenever it reads the controller for a run (S2's
simulation, Real Machine Mode); one that does not uses what is here, for as long as the slot holds that very program
(the same name, the same stored bytes): another program in the slot is not it, and its entry goes. What applies to a
program, in this order (``workflow.own_of``, ``api_library``, Advanced's Run Stored): this computer's record of sending
it, the project's entry by another computer's record, its file found by name on this computer, the project's entry by
a file or stated by hand. Only ever written with a real controller's list read (program_mirror.real_controller): the
simulation's arm and an arm not connected change nothing."""
from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path

from . import staging
from .jobs import UserError

FILE = "pole_and_hand.json"
RANK = {"stated": 0, "file": 1, "record": 2}        # what an entry is from: a later one never replaces a higher one
_LOCK = threading.Lock()


def held_as(info: dict | None) -> dict | None:
    """The program a slot holds as its controller lists it ({name, bytes}), from what Lite knows of the slot
    (workflow.programs_known); None where the controller's list was not read (the simulation's arm, an arm not
    connected) or the slot no longer holds what Lite sent."""
    if not info or info.get("gone"):
        return None
    h = info.get("held")
    if isinstance(h, dict) and h.get("name"):
        return dict(name=str(h["name"]), bytes=h.get("bytes"))
    if info.get("sent") is False and info.get("name"):
        return dict(name=str(info["name"]), bytes=info.get("size"))
    return None


def same_program(entry: dict, held: dict) -> bool:
    """Whether an entry is of the program the controller holds: its name there, and its stored bytes where both are
    known."""
    if str(entry.get("name") or "") != str(held.get("name") or ""):
        return False
    a, b = entry.get("bytes"), held.get("bytes")
    return a is None or b is None or int(a) == int(b)


def where_from(entry: dict) -> str:
    """Where an entry's pole and hand are from, in words."""
    by = entry.get("by")
    when = f", {entry['kept']}" if entry.get("kept") else ""
    if by == "record":
        return f"Lite's record of sending it{when}"
    if by == "file":
        return f"its file found by its name{' (' + str(entry['file']) + ')' if entry.get('file') else ''}{when}"
    return f"stated in the Trajectory Viewer{when}"


def _path(units, side: str) -> Path:
    from .program_mirror import FOLDER

    return Path(units.root) / FOLDER / side / FILE


def load(units, side: str) -> dict:
    """{slot: entry} as kept in the project; entries that are not a pole and hand of a named program are left out."""
    try:
        d = json.loads(_path(units, side).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    out = {}
    for e in d.get("programs") or []:
        try:
            slot = int(e["slot"])
        except (KeyError, TypeError, ValueError):
            continue
        if e.get("name") and e.get("by") in RANK and staging.stage_from(e) is not None:
            out[slot] = dict(e, slot=slot)
    return out


def _write(units, side: str, entries: dict) -> None:
    f = _path(units, side)
    f.parent.mkdir(parents=True, exist_ok=True)
    doc = dict(arm=side, updated=time.strftime("%Y-%m-%d %H:%M:%S"), programs=[entries[k] for k in sorted(entries)])
    tmp = f.with_name(f.name + ".tmp")
    tmp.write_text(json.dumps(doc, indent=1), encoding="utf-8")
    os.replace(tmp, f)


def _entry(slot: int, held: dict, stage: "staging.Stage", by: str, file: str | None = None) -> dict:
    e = dict(slot=int(slot), name=str(held["name"]), bytes=int(held["bytes"]) if held.get("bytes") is not None else None,
             pole_mm=stage.pole_mm, hand=list(stage.hand), text=staging.describe(stage), by=by)
    if by == "file" and file:
        e["file"] = str(file)
    return e


def _same_entry(a: dict, b: dict) -> bool:
    keys = ("slot", "name", "bytes", "pole_mm", "hand", "by", "file")
    return all(a.get(k) == b.get(k) for k in keys)


def link_for(links: dict | None, pid: int, info: dict | None) -> dict | None:
    """The project's entry for the program slot ``pid`` holds (``info``: what Lite knows of the slot), with
    ``compared``: whether the controller's list was read and names that very program; None when it holds another, no
    longer holds it, or nothing is kept for it. Not compared (the simulation's arm, an arm not connected): used as it
    is, since Real Machine Mode reads the controller before anything moves."""
    e = (links or {}).get(int(pid))
    if not e or (info or {}).get("gone"):
        return None
    held = held_as(info)
    if held is None:
        return dict(e, compared=False)
    return dict(e, compared=True) if same_program(e, held) else None


def keep(units, side: str, known: dict, own: dict | None = None) -> dict | None:
    """The project's entries brought up to date with what this computer knows (``known``: workflow.programs_known's;
    ``own``: workflow.own_of's for this arm, the run's Programs whose pole and hand came from a file found by name).
    -> the entries, or None when the controller's list was not read (nothing is changed then). A program this computer
    knows by its record, or by its file, gets its entry; one it does not know keeps the entry it has while the slot
    holds that very program; an entry of a slot that holds another program, or none, goes. An entry by a record is
    never replaced by a file's, nor a file's by a stated one."""
    read = {int(pid): h for pid, k in (known or {}).items() if (h := held_as(k))}
    if not read:
        return None
    with _LOCK:
        old = load(units, side)
        new = {}
        for pid, held in sorted(read.items()):
            k, prev = known[pid], old.get(pid)
            if prev is not None and not same_program(prev, held):
                prev = None                                    # another program in the slot since
            st = staging.stage_from(k.get("stage")) if k.get("sent") else None
            e = _entry(pid, held, st, "record") if st is not None else None
            if e is None:
                o = (own or {}).get(pid) or {}
                st = staging.stage_from(o.get("stage")) if o.get("how") == "file" else None
                if st is not None:
                    e = _entry(pid, held, st, "file", o.get("file_name"))
            if e is not None and prev is not None and RANK[prev["by"]] > RANK[e["by"]]:
                e = None                                       # the project's is from a record: a file's never replaces it
            if e is None:
                if prev is not None:
                    new[pid] = prev
            elif prev is not None and _same_entry(prev, e):
                new[pid] = prev                                # as it is: its date stays
            else:
                new[pid] = dict(e, kept=time.strftime("%Y-%m-%d %H:%M:%S"))
        if new != old:
            _write(units, side, new)
    return new


def keep_quietly(app, units, side: str, known: dict, own: dict | None = None) -> dict | None:
    """``keep``, never raising (it rides along a simulation or a run), and only with a real controller."""
    from .program_mirror import real_controller

    if app is not None and not real_controller(app, side):
        return None
    try:
        return keep(units, side, known, own)
    except Exception:                                                               # noqa: BLE001 - a record, never a stop
        return None


def state(units, side: str, pid: int, held: dict | None, pole_mm, hand) -> dict:
    """The operator states the pole and hand of the program slot ``pid`` holds (``held``: {name, bytes} as its
    controller lists it), in the Trajectory Viewer: kept with the project. Refused when the controller's list was not
    read, and when the project already holds them from a record or a file (they are not the operator's to replace: a
    program runs with its own)."""
    pid, S = int(pid), side.upper()
    if not 1 <= pid <= 100:
        raise UserError("Program numbers are 1 to 100.")
    if not held or not held.get("name"):
        raise UserError(f"The {S} controller's program {pid} is not read: its pole and hand are kept for the program as "
                        "its controller lists it.", "Connect the arm and refresh the list.", code="connect")
    st = staging.make_stage(pole_mm, hand, "as the operator stated it")
    with _LOCK:
        entries = load(units, side)
        prev = entries.get(pid)
        if prev is not None and same_program(prev, held) and prev["by"] != "stated":
            raise UserError(f"Program {pid} ({held['name']}) already has its pole and hand in this project: {prev['text']} "
                            f"({where_from(prev)}). A program runs with its own; they are not replaced by hand.",
                            code="own")
        entries[pid] = dict(_entry(pid, held, st, "stated"), kept=time.strftime("%Y-%m-%d %H:%M:%S"))
        _write(units, side, entries)
        return entries[pid]
