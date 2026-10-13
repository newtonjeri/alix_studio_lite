"""Taught trajectory files for the Lite UI's Trajectory Viewer.

The operator's uploads are kept in ``<data>/library/`` (``u_<id>.txt`` with ``u_<id>.json``:
name, arm, folder, size, sha256, when), shared by every project. Beside them the viewer lists the
built-in sets of ``assets/recordings/sets.yaml`` (read-only). Every file is checked line by line
on upload: one line per 5 ms, ``{"point":[7 whole millidegrees]}``.

A file does not say which arm it was taught on, so the arm comes from its name or folder (``L_``,
``LEFT``, ``R_``, ``RIGHT``), else the operator sets it.
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import tempfile
import threading
import time
import uuid
from pathlib import Path

from .. import vendor_paths
from .jobs import UserError

SETS_FILE = vendor_paths.ASSETS / "recordings" / "sets.yaml"
ID_RE = re.compile(r"^[us]_[0-9a-f]{12}$")
MAX_BYTES = 20_000_000


def detect_arm(path_or_name: str) -> str | None:
    """LEFT / RIGHT from a file's name or its folders, the way the files are named in practice."""
    parts = [p for p in re.split(r"[\\/]", str(path_or_name)) if p]
    for p in reversed(parts):
        u = p.upper()
        if re.match(r"^(L[_\-]|LEFT\b|LEFT[_\-])", u) or u in ("LEFT", "L"):
            return "left"
        if re.match(r"^(R[_\-]|RIGHT\b|RIGHT[_\-])", u) or u in ("RIGHT", "R"):
            return "right"
    for p in reversed(parts):
        u = p.upper()
        if "LEFT" in u:
            return "left"
        if "RIGHT" in u:
            return "right"
    return None


def _lines(path: Path) -> int:
    return sum(1 for ln in path.read_bytes().split(b"\n") if ln.strip())


def _meta_view(meta: dict) -> dict:
    out = {k: v for k, v in meta.items() if k != "path"}
    out["seconds"] = round(0.005 * max(int(meta.get("lines") or 1) - 1, 0), 3)
    return out


class TaughtLibrary:
    def __init__(self, data_dir: Path):
        self.root = Path(data_dir) / "library"
        self._lock = threading.Lock()
        self._sets_cache: tuple[float, list] | None = None

    # ── the operator's uploads

    def _meta_path(self, fid: str) -> Path:
        return self.root / f"{fid}.json"

    def uploads(self) -> list[dict]:
        out = []
        if self.root.is_dir():
            for m in sorted(self.root.glob("u_*.json")):
                try:
                    meta = json.loads(m.read_text(encoding="utf-8"))
                except (OSError, ValueError):
                    continue
                if (self.root / f"{meta['id']}.txt").is_file():
                    out.append(meta)
        out.sort(key=lambda m: m.get("uploaded", ""), reverse=True)
        return out

    def add(self, name: str, content: str, *, arm: str | None = None, path: str | None = None) -> dict:
        """Check and keep an uploaded file; returns its record."""
        from ..motion import track

        name = Path(str(name or "taught.txt")).name[:120]
        if not isinstance(content, str) or not content.strip():
            raise UserError(f"{name} is empty.")
        if len(content.encode("utf-8")) > MAX_BYTES:
            raise UserError(f"{name} is too large for a trajectory (over 20 MB).")
        if arm not in (None, "", "left", "right"):
            raise UserError("The arm is left or right.")
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td) / name
            tmp.write_text(content, encoding="utf-8")
            try:
                md = track.read_md(tmp)
            except ValueError as exc:
                msg = str(exc).replace(str(tmp), name)
                raise UserError(f"{name} is not a trajectory Lite can use: {msg}",
                                "A trajectory has one line per 5 ms: {\"point\":[7 whole numbers]}.") from None
            data = tmp.read_bytes()
        sha = hashlib.sha256(data).hexdigest()
        with self._lock:
            for m in self.uploads():                     # the same file again: keep one copy
                if m.get("sha256") == sha:
                    return m
            fid = "u_" + uuid.uuid4().hex[:12]
            self.root.mkdir(parents=True, exist_ok=True)
            (self.root / f"{fid}.txt").write_bytes(data)
            meta = dict(id=fid, name=name, folder=str(Path(path).parent) if path and Path(path).parent != Path(".") else "",
                        arm=arm or detect_arm(path or name), lines=int(md.shape[0]), bytes=len(data), sha256=sha,
                        uploaded=time.strftime("%Y-%m-%d %H:%M"), source="upload")
            self._write_meta(meta)
            return meta

    def _write_meta(self, meta: dict) -> None:
        tmp = self._meta_path(meta["id"]).with_suffix(".json.tmp")
        tmp.write_text(json.dumps(meta, indent=1), encoding="utf-8")
        os.replace(tmp, self._meta_path(meta["id"]))

    def update(self, fid: str, *, arm: str | None = None, name: str | None = None) -> dict:
        meta = self.get(fid)
        if meta.get("source") != "upload":
            raise UserError("Built-in files cannot be changed.")
        if arm is not None:
            if arm not in ("left", "right"):
                raise UserError("The arm is left or right.")
            meta["arm"] = arm
        if name:
            meta["name"] = Path(str(name)).name.strip()[:120] or meta["name"]
        with self._lock:
            self._write_meta({k: v for k, v in meta.items() if k != "path"})
        return self.get(fid)

    def delete(self, fid: str) -> None:
        meta = self.get(fid)
        if meta.get("source") != "upload":
            raise UserError("Built-in files cannot be deleted.")
        trash = self.root / ".trash"
        trash.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        with self._lock:
            for suffix in (".txt", ".json"):
                f = self.root / f"{fid}{suffix}"
                if f.is_file():
                    shutil.move(str(f), str(trash / f"{fid}-{stamp}{suffix}"))

    # ── built-in sets

    def sets(self) -> list[dict]:
        import yaml

        from .project import library

        try:
            mtime = SETS_FILE.stat().st_mtime
        except OSError:
            return []
        if self._sets_cache and self._sets_cache[0] == mtime:
            return self._sets_cache[1]
        d = yaml.safe_load(SETS_FILE.read_text(encoding="utf-8")) or {}
        out = []
        for s in d.get("sets") or []:
            files = []
            if s.get("library"):
                for name, meta in library(s["library"])["files"].items():
                    files.append(dict(name=name, arm=meta.get("arm"), path=meta["path"], task=meta.get("task"),
                                      unit=meta.get("unit")))
            for arm, folder in (s.get("folders") or {}).items():
                base = vendor_paths.LITE_ROOT / folder
                for f in sorted(base.glob("*.txt")) if base.is_dir() else []:
                    files.append(dict(name=f.name, arm=arm, path=str(f), task=None, unit=None))
            rows = []
            for f in files:
                rel = os.path.relpath(f["path"], vendor_paths.LITE_ROOT)
                fid = "s_" + hashlib.sha1(rel.encode("utf-8")).hexdigest()[:12]
                rows.append(dict(id=fid, name=f["name"], arm=f["arm"], folder=str(Path(rel).parent), task=f["task"],
                                 unit=f["unit"], lines=_lines(Path(f["path"])), source="set", set=s["id"], path=f["path"]))
            out.append(dict(id=s["id"], name=s["name"], note=s.get("note", ""), files=rows))
        self._sets_cache = (mtime, out)
        return out

    # ── one file, wherever it is

    def get(self, fid: str) -> dict:
        if not isinstance(fid, str) or not ID_RE.match(fid):
            raise UserError("That is not a library file.")
        if fid.startswith("u_"):
            try:
                meta = json.loads(self._meta_path(fid).read_text(encoding="utf-8"))
            except (OSError, ValueError):
                raise UserError("That file is not in the library any more.") from None
            meta["path"] = str(self.root / f"{fid}.txt")
            return meta
        for s in self.sets():
            for f in s["files"]:
                if f["id"] == fid:
                    return dict(f)
        raise UserError("That file is not in the library any more.")

    def listing(self) -> dict:
        return {"files": [_meta_view(m) for m in self.uploads()],
                "sets": [dict(s, files=[_meta_view(f) for f in s["files"]]) for s in self.sets()],
                "folder": str(self.root)}

    def checks(self, fid: str, side: str) -> dict:
        """Every limit on the file as taught (L5 and L6 compare a tuned file with its recording,
        so they do not apply). Cached per file and arm."""
        from ..motion import track
        from ..motion.checks import check_bytes
        from ..motion.limits import load_limits

        meta = self.get(fid)
        key = hashlib.sha1(f"{meta['path']}:{side}".encode()).hexdigest()[:16]
        cache = self.root / "cache" / f"checks_{key}.json"
        path = Path(meta["path"])
        stamp = path.stat().st_mtime
        try:
            c = json.loads(cache.read_text(encoding="utf-8"))
            if c.get("mtime") == stamp:
                return c["report"]
        except (OSError, ValueError):
            pass
        md = track.read_md(path)
        rep = check_bytes(md, load_limits(side))
        out = dict(rep.to_dict(), lines=int(md.shape[0]), seconds=round(0.005 * (md.shape[0] - 1), 3), side=side)
        cache.parent.mkdir(parents=True, exist_ok=True)
        cache.write_text(json.dumps(dict(mtime=stamp, report=out), default=float), encoding="utf-8")
        return out
