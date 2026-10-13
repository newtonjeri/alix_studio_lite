"""The Lite UI's projects: one folder per project, owned by the backend.

The handoff asks for projects that own their files on disk (its sections 5.4 and 20); the
layout is the one it recommends::

    <data>/projects/<id>/
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
(schema ``alix_studio_lite_project_v1``) is accepted on import too. Delete moves the folder to
``<data>/projects/.trash/`` rather than removing the files.
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


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _write_json(path: Path, obj) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(obj, indent=1, ensure_ascii=False), encoding="utf-8")
    os.replace(tmp, path)


def new_id() -> str:
    return "alix_" + uuid.uuid4().hex[:12]


class ProjectStore:
    def __init__(self, root: Path, default_fixture: str = "commode_c"):
        self.root = Path(root)
        self.default_fixture = default_fixture
        self._lock = threading.RLock()

    # ── paths

    def folder(self, pid: str) -> Path:
        if not isinstance(pid, str) or not ID_RE.match(pid):
            raise UserError("That is not a valid project id.")
        return self.root / pid

    def _record_path(self, pid: str) -> Path:
        return self.folder(pid) / "project.json"

    # ── read

    def list(self) -> list[dict]:
        out = []
        if self.root.is_dir():
            for d in self.root.iterdir():
                if d.is_dir() and ID_RE.match(d.name) and (d / "project.json").is_file():
                    try:
                        out.append(json.loads((d / "project.json").read_text(encoding="utf-8")))
                    except (OSError, ValueError):
                        continue                    # a damaged record is skipped, never fatal
        out.sort(key=lambda p: str(p.get("updatedAt") or ""), reverse=True)
        return out

    def get(self, pid: str) -> dict:
        path = self._record_path(pid)
        if not path.is_file():
            raise UserError("Project not found.")
        return json.loads(path.read_text(encoding="utf-8"))

    def exists(self, pid: str) -> bool:
        return self._record_path(pid).is_file()

    @property
    def last_id(self) -> str | None:
        try:
            pid = json.loads((self.root / "last.json").read_text(encoding="utf-8")).get("id")
        except (OSError, ValueError):
            return None
        return pid if isinstance(pid, str) and ID_RE.match(pid) and self.exists(pid) else None

    def set_last(self, pid: str) -> None:
        self.folder(pid)
        _write_json(self.root / "last.json", {"id": pid})

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
            folder = self.folder(pid)
            old = self.get(pid) if self.exists(pid) else None
            rec = dict(record)
            rec["id"] = pid
            rec.setdefault("createdAt", (old or {}).get("createdAt") or _now())
            rec.setdefault("updatedAt", _now())
            rec.setdefault("archived", False)
            lite = dict((old or {}).get("lite") or {})
            lite.setdefault("store", STORE_VERSION)
            lite.setdefault("fixture", fixture or self.default_fixture)
            rec["lite"] = lite
            for sub in SUBDIRS:
                (folder / sub).mkdir(parents=True, exist_ok=True)
            _write_json(folder / "project.json", rec)
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
            if not folder.is_dir():
                raise UserError("Project not found.")
            trash = self.root / ".trash" / f"{pid}-{time.strftime('%Y%m%d-%H%M%S')}"
            trash.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(folder), str(trash))
            return str(trash)

    def duplicate(self, pid: str, name: str | None = None) -> dict:
        with self._lock:
            src = self.get(pid)
            nid = new_id()
            shutil.copytree(self.folder(pid), self.folder(nid))
            now = _now()
            rec = dict(src, id=nid, name=(name or f"{src.get('name', 'Project')} – Copy")[:120],
                       createdAt=now, updatedAt=now, archived=False)
            _write_json(self.folder(nid) / "project.json", rec)
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
            staging = self.root / ".import" / uuid.uuid4().hex
            try:
                z.extractall(staging)
                return self._import_record(record, filename, folder_from=staging)
            finally:
                shutil.rmtree(staging, ignore_errors=True)

    def _import_record(self, incoming: dict, filename: str, folder_from: Path | None) -> dict:
        if not isinstance(incoming, dict) or not isinstance(incoming.get("state"), dict):
            raise UserError("Invalid ALIX project file.")
        stem = re.sub(r"\.(alixproj|json|zip)$", "", Path(filename or "").name, flags=re.I)
        name = f"{incoming.get('name') or stem or 'Project'} – Imported"[:120]
        nid = new_id()
        with self._lock:
            folder = self.folder(nid)
            if folder_from is not None:
                shutil.copytree(folder_from, folder)
                (folder / "project.json").unlink(missing_ok=True)
            fixture = ((incoming.get("lite") or {}).get("fixture")) or self.default_fixture
            rec = {k: v for k, v in incoming.items() if k != "lite"}
            rec.update(id=nid, name=name, updatedAt=_now(), archived=False)
            return self.put(nid, rec, fixture=fixture)
