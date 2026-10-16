"""The Lite UI's projects: one folder per project, owned by the backend.

The handoff asks for projects that own their files on disk (its sections 5.4 and 20); the
layout is the one it recommends. The folders are in ``Resources/Projects/`` in the repository
(the owner, 16 Oct), each named exactly as its project was named when it was created (a rename
later renames the project, not its folder); the project's id, in its record, is what the page and
the server use::

    Resources/Projects/<project name>/
        project.json                 the page's project record, plus "lite" (kept by the server)
        trajectories/left|right/     recordings and loaded files of each arm
        merged/left|right/
        sequence/
        simulation/reports|logs/
        controller_exports/
        telemetry/

The page owns its record (name, archived flag, its saved ``state``); the server keeps its own
fields under ``"lite"`` (the fixture, the store version) and never lets a page write replace
them. A project file (``.alixproj``) is a ZIP of the folder; the designer's JSON export
(schema ``alix_studio_lite_project_v1``) is accepted on import too. This computer's own (the last
project opened, the trash, an import being unpacked) stays in ``<data>/projects/``: delete moves
the folder to ``<data>/projects/.trash/`` rather than removing the files. Projects kept there by
id before (``<data>/projects/<id>/``) are copied in once, the old folders left as they are.
"""
from __future__ import annotations

import io
import json
import os
import re
import shutil
import threading
import time
import uuid
import zipfile
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath

from .jobs import UserError

SCHEMA = "alix_studio_lite_project_v1"      # the page's project schema (handoff section 5.3)
STORE_VERSION = 1
ID_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
SUBDIRS = ("trajectories/left", "trajectories/right", "merged/left", "merged/right", "sequence",
           "simulation/reports", "simulation/logs", "controller_exports", "telemetry")
MAX_RECORD = 4 * 1024 * 1024                # one project.json
MAX_UNZIPPED = 1024 * 1024 * 1024           # everything inside an imported project file
MAX_MEMBERS = 20_000
NOT_IN_A_NAME = re.compile(r"[/\\\x00-\x1f\x7f]")   # what cannot be in a folder's name


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _write_json(path: Path, obj) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f"{path.name}.{os.getpid()}.{threading.get_ident()}.tmp")   # one per writer
    tmp.write_text(json.dumps(obj, indent=1, ensure_ascii=False), encoding="utf-8")
    os.replace(tmp, path)


def new_id() -> str:
    return "alix_" + uuid.uuid4().hex[:12]


def folder_problem(name) -> str | None:
    """Why ``name`` cannot name a new project's folder, or None."""
    if not isinstance(name, str) or not name.strip() or len(name) > 120:
        return "A project needs a name of 1 to 120 characters."
    if NOT_IN_A_NAME.search(name):
        return "A project name cannot have / or \\ in it: it is the name of the project's folder."
    if name.startswith("."):
        return "A project name cannot start with a dot: it is the name of the project's folder."
    if len(name.encode("utf-8")) > 240:
        return "This project name is too long to name its folder."
    return None


def usable(name: str) -> str:
    """A name Lite chose (a copy's, an import's, a project kept before) made fit to name a folder."""
    out = NOT_IN_A_NAME.sub("-", str(name or "")).strip().lstrip(".").strip()[:120]
    while len(out.encode("utf-8")) > 240:
        out = out[:-1]
    return out or "Project"


class ProjectStore:
    def __init__(self, root: Path, default_fixture: str = "commode_c", home: Path | None = None):
        self.root = Path(root)                              # the projects' folders
        self.home = Path(home) if home is not None else self.root   # this computer's: last.json, .trash, .import
        self.default_fixture = default_fixture
        self._lock = threading.RLock()
        self._where: dict[str, Path] = {}                   # id -> its folder, as last found

    # ── paths

    def _records(self):
        """(folder, record) of every project folder, the damaged ones left out (never fatal)."""
        if not self.root.is_dir():
            return
        for d in sorted(self.root.iterdir()):
            if d.is_dir() and not d.name.startswith(".") and (d / "project.json").is_file():
                try:
                    rec = json.loads((d / "project.json").read_text(encoding="utf-8"))
                except (OSError, ValueError):
                    continue
                pid = rec.get("id") if isinstance(rec, dict) else None
                if isinstance(pid, str) and ID_RE.match(pid):
                    yield d, rec

    def _find(self, pid: str) -> Path | None:
        if not isinstance(pid, str) or not ID_RE.match(pid):
            raise UserError("That is not a valid project id.")
        d = self._where.get(pid)
        if d is not None and (d / "project.json").is_file():
            return d
        self.list()                                         # a folder renamed, added or gone: look again
        return self._where.get(pid)

    def folder(self, pid: str) -> Path:
        d = self._find(pid)
        if d is None:
            raise UserError("Project not found.")
        return d

    def _record_path(self, pid: str) -> Path:
        return self.folder(pid) / "project.json"

    def _taken(self, name: str) -> str | None:
        """The folder already named ``name`` (letter case aside), or None."""
        key = name.casefold()
        if self.root.is_dir():
            for d in self.root.iterdir():
                if d.name.casefold() == key:
                    return d.name
        return None

    def name_problem(self, name) -> str | None:
        """Why a new project cannot be named ``name`` (its folder's name), or None."""
        problem = folder_problem(name)
        if problem:
            return problem
        taken = self._taken(name)
        if taken:
            owner = next((r.get("name") for d, r in self._records() if d.name == taken), None)
            whose = f" (project “{owner}”, renamed since)" if owner and owner != taken else ""
            return f"A project folder named “{taken}” is already in {self.root}{whose}: choose another name."
        return None

    def _claim(self, name: str) -> Path:
        """A new project's folder, named ``name``."""
        problem = self.name_problem(name)
        if problem:
            raise UserError(problem)
        d = self.root / name
        d.mkdir(parents=True)
        return d

    def _free(self, name: str) -> str:
        """``name``, or the first of ``name 2``, ``name 3`` ... no folder has (a name Lite chose)."""
        if not self._taken(name):
            return name
        for n in range(2, 1000):
            cand = f"{name[:112]} {n}"
            if not self._taken(cand):
                return cand
        raise UserError("Too many projects with this name.")

    # ── read

    def list(self) -> list[dict]:
        out, where = [], {}
        for d, rec in self._records():
            if rec["id"] not in where:                      # a folder copied by hand: the first one counts
                where[rec["id"]] = d
                out.append(rec)
        self._where = where
        out.sort(key=lambda p: str(p.get("updatedAt") or ""), reverse=True)
        return out

    def get(self, pid: str) -> dict:
        d = self._find(pid)
        if d is None:
            raise UserError("Project not found.")
        return json.loads((d / "project.json").read_text(encoding="utf-8"))

    def exists(self, pid: str) -> bool:
        return self._find(pid) is not None

    @property
    def last_id(self) -> str | None:
        try:
            pid = json.loads((self.home / "last.json").read_text(encoding="utf-8")).get("id")
        except (OSError, ValueError):
            return None
        return pid if isinstance(pid, str) and ID_RE.match(pid) and self.exists(pid) else None

    def set_last(self, pid: str) -> None:
        if not isinstance(pid, str) or not ID_RE.match(pid):
            raise UserError("That is not a valid project id.")
        with self._lock:
            _write_json(self.home / "last.json", {"id": pid})

    # ── write

    def put(self, pid: str, record: dict, *, fixture: str | None = None) -> dict:
        """Create or replace the page's record of project ``pid``; ``lite`` stays the server's."""
        if not isinstance(record, dict):
            raise UserError("A project must be a JSON object.")
        name = record.get("name")
        if not isinstance(name, str) or not name.strip() or len(name) > 120:
            raise UserError("A project needs a name of 1 to 120 characters.")
        if "state" in record and not isinstance(record["state"], dict):
            raise UserError("The project's saved state is not valid.")
        body = json.dumps(record, ensure_ascii=False)
        if len(body.encode("utf-8")) > MAX_RECORD:
            raise UserError("This project's saved state is too large.")
        with self._lock:
            folder = self._find(pid)
            old = self.get(pid) if folder is not None else None
            if folder is None:                              # a new project: its folder takes its name
                folder = self._claim(name)
            return self._save(folder, pid, record, old, fixture)

    def _save(self, folder: Path, pid: str, record: dict, old: dict | None, fixture: str | None) -> dict:
        rec = dict(record)
        rec["id"] = pid
        rec.setdefault("createdAt", (old or {}).get("createdAt") or _now())
        rec.setdefault("updatedAt", _now())
        rec.setdefault("archived", False)
        lite = dict((old or {}).get("lite") or {})
        lite.setdefault("store", STORE_VERSION)
        lite.setdefault("fixture", fixture or self.default_fixture)
        lite["folder"] = folder.name
        rec["lite"] = lite
        for sub in SUBDIRS:
            (folder / sub).mkdir(parents=True, exist_ok=True)
        _write_json(folder / "project.json", rec)
        self._where[pid] = folder
        return rec

    def set_lite(self, pid: str, **fields) -> dict:
        """Update the server's own fields of a project (never written by the page)."""
        with self._lock:
            rec = self.get(pid)
            lite = dict(rec.get("lite") or {})
            lite.update(fields)
            rec["lite"] = lite
            _write_json(self._record_path(pid), rec)
            return rec

    def delete(self, pid: str) -> str:
        """Move the project's folder to the trash; returns where it went."""
        with self._lock:
            folder = self.folder(pid)
            trash = self.home / ".trash" / f"{folder.name}-{time.strftime('%Y%m%d-%H%M%S')}"
            trash.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(folder), str(trash))
            self._where.pop(pid, None)
            return str(trash)

    def duplicate(self, pid: str, name: str | None = None) -> dict:
        """A copy with its files, in a folder named as the copy is (``<name> – Copy``, numbered when taken)."""
        with self._lock:
            src, source = self.get(pid), self.folder(pid)
            new = name if name else self._free(usable(f"{src.get('name', 'Project')} – Copy"))
            folder = self._claim(new)
            shutil.copytree(source, folder, dirs_exist_ok=True)
            now = _now()
            rec = dict(src, id=new_id(), name=new, createdAt=now, updatedAt=now, archived=False)
            rec["lite"] = dict(src.get("lite") or {}, folder=folder.name)
            _write_json(folder / "project.json", rec)
            self._where[rec["id"]] = folder
            return rec

    # ── project files

    def export_zip(self, pid: str) -> bytes:
        folder = self.folder(pid)
        if not (folder / "project.json").is_file():
            raise UserError("Project not found.")
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
            for f in sorted(folder.rglob("*")):
                if f.is_file() and not f.name.endswith(".tmp"):
                    z.write(f, f.relative_to(folder).as_posix())
        return buf.getvalue()

    def import_file(self, data: bytes, filename: str = "") -> dict:
        """A new project from a ``.alixproj`` ZIP or the designer's JSON export."""
        if data[:2] == b"PK":
            return self._import_zip(data, filename)
        try:
            obj = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            raise UserError("Invalid ALIX project file.") from None
        if not isinstance(obj, dict):
            raise UserError("Invalid ALIX project file.")
        if obj.get("schema") and obj["schema"] != SCHEMA:
            raise UserError("Unsupported project schema.")
        incoming = obj.get("project") if isinstance(obj.get("project"), dict) else obj
        return self._import_record(incoming, filename, folder_from=None)

    def _import_zip(self, data: bytes, filename: str) -> dict:
        try:
            z = zipfile.ZipFile(io.BytesIO(data))
        except zipfile.BadZipFile:
            raise UserError("Invalid ALIX project file.") from None
        with z:
            infos = z.infolist()
            if len(infos) > MAX_MEMBERS or sum(i.file_size for i in infos) > MAX_UNZIPPED:
                raise UserError("This project file is too large.")
            for i in infos:
                p = PurePosixPath(i.filename)
                if p.is_absolute() or ".." in p.parts or "\\" in i.filename:
                    raise UserError("This project file contains unsafe paths.")
            try:
                record = json.loads(z.read("project.json").decode("utf-8"))
            except (KeyError, UnicodeDecodeError, ValueError):
                raise UserError("This project file has no valid project.json.") from None
            staging = self.home / ".import" / uuid.uuid4().hex
            try:
                z.extractall(staging)
                return self._import_record(record, filename, folder_from=staging)
            finally:
                shutil.rmtree(staging, ignore_errors=True)

    def _import_record(self, incoming: dict, filename: str, folder_from: Path | None) -> dict:
        if not isinstance(incoming, dict) or not isinstance(incoming.get("state"), dict):
            raise UserError("Invalid ALIX project file.")
        stem = re.sub(r"\.(alixproj|json|zip)$", "", Path(filename or "").name, flags=re.I)
        nid = new_id()
        with self._lock:
            name = self._free(usable(f"{incoming.get('name') or stem or 'Project'} – Imported"))
            folder = self._claim(name)
            if folder_from is not None:
                shutil.copytree(folder_from, folder, dirs_exist_ok=True)
                (folder / "project.json").unlink(missing_ok=True)
                # evidence made on another computer is not trusted here: the tuned versions, the
                # work in progress and the robot results go; the recordings and settings stay, and
                # each region is tuned (and so checked against every limit) again on this computer
                for unit in (u for u in [*(folder / "trajectories").glob("*/*"), *(folder / "merged" / "tune").glob("*")]
                             if u.is_dir()):
                    for made in ("versions", "work"):
                        shutil.rmtree(unit / made, ignore_errors=True)
                    (unit / "results.json").unlink(missing_ok=True)
            fixture = ((incoming.get("lite") or {}).get("fixture")) or self.default_fixture
            rec = {k: v for k, v in incoming.items() if k != "lite"}
            rec.update(id=nid, name=name, updatedAt=_now(), archived=False)
            return self._save(folder, nid, rec, None, fixture)

    # ── projects kept by id before

    def adopt(self, old: Path) -> list[dict]:
        """Copies the projects ``old`` keeps by id (Lite before 16 October: ``<data>/projects/<id>/``)
        into this store, each in a folder named as the project is now, once (``copied.json`` in
        ``home`` remembers them, so one deleted here is not copied again). The old folders are left
        as they are. A name that cannot name a folder, or that one has already, is changed to one
        that can (the project's name with it). Returns [{id, name, folder, renamed}]."""
        old = Path(old)
        if not old.is_dir() or old.resolve() == self.root.resolve():
            return []
        mark = self.home / "copied.json"
        try:
            done = json.loads(mark.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            done = {}
        out = []
        with self._lock:
            for d in sorted(old.iterdir()):
                if not (d.is_dir() and ID_RE.match(d.name) and d.name not in done and (d / "project.json").is_file()):
                    continue
                if (d / "units").is_dir():                  # Legacy Lite's, not the Lite UI's
                    continue
                try:
                    rec = json.loads((d / "project.json").read_text(encoding="utf-8"))
                except (OSError, ValueError):
                    continue
                if not isinstance(rec, dict) or rec.get("id") != d.name:
                    continue
                if self._find(d.name) is not None:          # here already
                    done[d.name] = str(self._find(d.name))
                    continue
                was = str(rec.get("name") or d.name)
                name = self._free(usable(was))
                folder = self._claim(name)
                shutil.copytree(d, folder, dirs_exist_ok=True)
                rec["name"] = name
                rec["lite"] = dict(rec.get("lite") or {}, folder=name)
                _write_json(folder / "project.json", rec)
                self._where[d.name] = folder
                done[d.name] = str(folder)
                out.append({"id": d.name, "name": name, "folder": str(folder), "renamed": was if was != name else None})
            if out or not mark.is_file():
                _write_json(mark, done)
        return out
