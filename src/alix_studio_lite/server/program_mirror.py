"""The programs on the arms' controllers, kept in the project (the user, 4 Nov: "since the data management have access to
the controllers, uploaded files, it should have an on-disk storage where it loads or downloads those plans from the
hardware. And saves a copy of them in this repo. So that we can be able to do the edits").

RealMan's API gives no program's points back: rm_get_program_trajectory_list returns each slot's number, name, size and
speed only, and download_project is the upload's direction. So a program is kept from the bytes Lite sent it as. Those
are the file Lite kept (Advanced's uploads in Lite's data folder, S1's in the project), matched to the slot by its sha256
in Lite's record of what it sent there (workflow.programs_known). A program Lite did not send (the control program's) is
listed by its name: its points are not on this computer.

Written to ``<project>/controller_exports/<arm>/``:
- ``<NN>_<its name on the controller>.txt``: the bytes the slot holds, copied only when they changed;
- ``programs.json``: each slot's name on the controller, sha256, lines, seconds, the pole and hand it runs with, where
  its file came from, and the slots whose points Lite does not have.
Kept whenever Data Management reads an arm with a project open and whenever S2 is simulated, so the logs pushed from the
lab carry the files the controllers run; with them, ``points``: the controller's global points as last read.

Only ever from a REAL controller (the user, 5 Nov: a save in Visual sim mirrored the stand-in, which holds none of the
lab's programs, and deleted 31 of the left arm's files): on the stand-in or ALIX's emulator nothing is written. There it
is the other way round: the simulation's arm holds what the real controller held when it was last kept (``exported``:
its programs and global points, for S2, Data Management and the Viewer), so the simulation is the hardware's."""
from __future__ import annotations

import json
import os
import re
import shutil
import time
from pathlib import Path

from ..motion import track

FOLDER = "controller_exports"


def _safe(name: str) -> str:
    s = re.sub(r"\.(t|txt)$", "", str(name or "").strip())
    return re.sub(r"[^A-Za-z0-9_.+-]+", "_", s)[:80] or "program"


def real_controller(app, side: str) -> bool:
    """Whether ``side`` is linked to a real controller: not the stand-in, not ALIX's emulator."""
    link = app.robot.links.get(side)
    return app.robot._real(side) is not None and link is not None and not getattr(link, "emulated", False)


def exported(units, side: str) -> dict | None:
    """What ``side``'s real controller held when it was last kept in the project: {kept, programs: {slot: row with
    ``path`` (the kept file)}, points: {name: joints in degrees}} (programs.json), or None when nothing was kept."""
    folder = Path(units.root) / FOLDER / side
    try:
        man = json.loads((folder / "programs.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    progs = {}
    for r in man.get("programs") or []:
        f = folder / str(r.get("file") or "")
        if r.get("file") and f.is_file():
            progs[int(r["slot"])] = dict(r, path=str(f))
    return dict(kept=man.get("kept"), programs=progs, points=dict(man.get("points") or {}))


def keep(app, units, side: str, known: dict | None = None, points: dict | None = None) -> dict:
    """The programs ``side``'s controller holds, as Lite knows them (``known``: workflow.programs_known's, read again
    when None), kept under the project's controller_exports/<side>/, with its global points (``points``: {name: joints
    in degrees} as last read; the kept ones stay when not given). -> the manifest (programs.json's). Refused unless the
    arm is linked to a real controller (real_controller): nothing is written or deleted then."""
    from . import workflow
    from .jobs import UserError

    if app is not None and not real_controller(app, side):
        raise UserError(f"Not saved: the {side.upper()} arm is the simulation's, not a real controller, and its programs are "
                        "the project's kept copy of the real one's. Nothing was changed.",
                        "Save them from the real controller: Real robot mode, Lite started with --robot realman, the arm connected.",
                        code="sim")
    known = workflow.programs_known(app, units, side) if known is None else known
    folder = Path(units.root) / FOLDER / side
    before = exported(units, side) or {}
    folder.mkdir(parents=True, exist_ok=True)
    rows, not_kept, names = [], [], set()
    for pid, k in sorted(known.items()):
        if k.get("gone"):                              # Lite sent it there; the controller no longer holds it
            continue
        src = Path(k["file"]) if k.get("file") else None
        if not k.get("sent") or src is None or not src.is_file():
            not_kept.append(dict(slot=int(pid), name=k.get("name"),
                                 why="not sent by Lite: the controller gives no program's points back" if not k.get("sent")
                                 else "Lite sent it, but the file it sent is no longer on this computer"))
            continue
        name = f"{int(pid):02d}_{_safe(k.get('name'))}.txt"
        dst = folder / name
        names.add(name)
        if not dst.is_file() or track.sha256_file(dst) != k.get("sha256"):
            tmp = dst.with_name(dst.name + ".tmp")
            shutil.copyfile(src, tmp)
            os.replace(tmp, dst)
        rows.append(dict(slot=int(pid), name=k.get("name"), file=name, sha256=k.get("sha256"), lines=k.get("lines"),
                         seconds=k.get("seconds"), stage=k.get("stage"), source=str(src)))
    for old in folder.glob("[0-9][0-9]_*.txt"):        # a slot emptied or given another program since
        if old.name not in names:
            old.unlink()
    manifest = dict(arm=side, kept=time.strftime("%Y-%m-%d %H:%M:%S"), programs=rows, not_kept=not_kept,
                    points={str(k): [round(float(v), 4) for v in q] for k, q in (points if points is not None else before.get("points") or {}).items()})
    tmp = folder / "programs.json.tmp"
    tmp.write_text(json.dumps(manifest, indent=1), encoding="utf-8")
    os.replace(tmp, folder / "programs.json")
    return manifest


def keep_quietly(app, units, side: str, known: dict | None = None, points: dict | None = None) -> dict | None:
    """``keep``, never raising (it rides along a read or a simulation): the manifest, or None (the stand-in, ALIX's
    emulator: nothing written)."""
    if app is not None and not real_controller(app, side):
        return None
    try:
        return keep(app, units, side, known, points)
    except Exception:                                                               # noqa: BLE001 - a copy, never a stop
        return None
