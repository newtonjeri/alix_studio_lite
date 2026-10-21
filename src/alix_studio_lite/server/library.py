"""Taught trajectory files for the Lite UI's Trajectory Viewer.

The operator's uploads are kept in ``<data>/library/`` (``u_<id>.txt`` with ``u_<id>.json``),
shared by every project. Beside them the viewer lists the built-in sets of
``assets/recordings/sets.yaml`` (read-only). Every file is checked line by line on upload: one
line per 5 ms, ``{"point":[7 whole millidegrees]}``.

Every file carries what it takes to run it correctly (``staging.py``):

- the ARM it was taught on. A file does not say, so it comes from the set's records, or from its
  name or folder (``L_``, ``LEFT``, ``R_``, ``RIGHT``), else the operator states it. A file is
  only ever shown or used on that arm.
- its STAGE: the pole height and hand pose it was taught (or ran) with. Built-in files take it
  from their records (the taught set's pole and hand) or their workflow (the V6 package's pole
  and hand steps); an upload's stage is stated by the operator. A file without one is listed but
  cannot be replayed or used.
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
from . import staging
from .jobs import UserError

SETS_FILE = vendor_paths.ASSETS / "recordings" / "sets.yaml"
ID_RE = re.compile(r"^[us]_[0-9a-f]{12}$")
MAX_BYTES = 20_000_000


def detect_arm(path_or_name: str) -> str | None:
    """LEFT / RIGHT from a file's name or its folders, the way the files are named in practice.
    ``R15_…`` is a region number, not the right arm, so only ``R_``/``R-``/``RIGHT`` count."""
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


def _lines_and_sha(path: Path) -> tuple[int, str]:
    data = path.read_bytes()
    return sum(1 for ln in data.split(b"\n") if ln.strip()), hashlib.sha256(data).hexdigest()


def _stage_dict(pole_mm, hand, source: str) -> dict | None:
    if pole_mm is None or hand is None:
        return None
    try:
        return staging.make_stage(pole_mm, hand, source).as_dict()
    except UserError:
        return None


def _workflow_stages(path: Path) -> dict:
    """Program name -> stage: the last pole and hand steps before each program step."""
    try:
        d = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    pole = hand = None
    out = {}
    for st in d.get("steps") or []:
        if st.get("type") == "pole":
            pole = st.get("mm")
        elif st.get("type") == "hand":
            hand = st.get("values")
        elif st.get("type") == "program" and st.get("name"):
            out[st["name"]] = _stage_dict(pole, hand, f"its workflow ({path.name})")
    return out


_SETS: tuple[float, list] | None = None
_SETS_LOCK = threading.Lock()


def builtin_sets() -> list[dict]:
    """The built-in sets with their files (id, name, arm, stage, path, …), cached per edit."""
    import yaml

    from .project import library

    global _SETS
    try:
        mtime = SETS_FILE.stat().st_mtime
    except OSError:
        return []
    with _SETS_LOCK:
        if _SETS and _SETS[0] == mtime:
            return _SETS[1]
        d = yaml.safe_load(SETS_FILE.read_text(encoding="utf-8")) or {}
        out = []
        for s in d.get("sets") or []:
            files = []
            if s.get("library"):
                for name, meta in library(s["library"])["files"].items():
                    files.append(dict(name=name, arm=meta.get("arm"), path=meta["path"], task=meta.get("task"),
                                      unit=meta.get("unit"),
                                      stage=_stage_dict(meta.get("pole_mm"), meta.get("hand"), "the taught set's record")))
            stages = {}
            for arm, wf in (s.get("workflows") or {}).items():
                stages[arm] = _workflow_stages(vendor_paths.LITE_ROOT / wf)
            for arm, folder in (s.get("folders") or {}).items():
                base = vendor_paths.LITE_ROOT / folder
                for f in sorted(base.glob("*.txt")) if base.is_dir() else []:
                    files.append(dict(name=f.name, arm=arm, path=str(f), task=None, unit=None,
                                      stage=(stages.get(arm) or {}).get(f.name)))
            rows = []
            for f in files:
                rel = os.path.relpath(f["path"], vendor_paths.LITE_ROOT)
                fid = "s_" + hashlib.sha1(rel.encode("utf-8")).hexdigest()[:12]
                lines, sha = _lines_and_sha(Path(f["path"]))
                rows.append(dict(id=fid, name=f["name"], arm=f["arm"], stage=f["stage"], folder=str(Path(rel).parent),
                                 task=f["task"], unit=f["unit"], lines=lines, sha256=sha, source="set",
                                 set=s["id"], path=f["path"]))
            out.append(dict(id=s["id"], name=s["name"], note=s.get("note", ""), files=rows))
        _SETS = (mtime, out)
        return out


def builtin_by_name(name: str) -> dict | None:
    """A built-in file by its file name."""
    for s in builtin_sets():
        for f in s["files"]:
            if f["name"] == name:
                return f
    return None


def builtin_by_sha(sha256: str | None) -> dict | None:
    """The built-in file with exactly these bytes. A name proves nothing (an operator can teach a
    new file of the same name at another pole height); the bytes do."""
    if not sha256:
        return None
    for s in builtin_sets():
        for f in s["files"]:
            if f["sha256"] == sha256:
                return f
    return None


def view(meta: dict) -> dict:
    out = {k: v for k, v in meta.items() if k != "path"}
    out["seconds"] = round(0.005 * max(int(meta.get("lines") or 1) - 1, 0), 3)
    out["ready"] = bool(meta.get("arm") and meta.get("stage"))
    return out


class TaughtLibrary:
    def __init__(self, data_dir: Path):
        self.root = Path(data_dir) / "library"
        self._lock = threading.Lock()

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

    def add(self, name: str, content: str, *, arm: str | None = None, path: str | None = None,
            pole_mm=None, hand=None) -> dict:
        """Check and keep an uploaded file; returns its record."""
        from ..motion import track

        name = Path(str(name or "taught.txt")).name[:120]
        if not isinstance(content, str) or not content.strip():
            raise UserError(f"{name} is empty.")
        if len(content.encode("utf-8")) > MAX_BYTES:
            raise UserError(f"{name} is too large for a trajectory (over 20 MB).")
        if arm not in (None, "", "left", "right"):
            raise UserError("The arm is left or right.")
        stage = None
        if pole_mm is not None or hand is not None:
            stage = staging.make_stage(pole_mm, hand, "stated when it was uploaded").as_dict()
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
                    said = arm or detect_arm(path or name)
                    if (said and said != m.get("arm")) or (stage and not staging.same_stage(stage, m.get("stage"))):
                        was = (f"{m['arm'].upper()} arm" if m.get("arm") else "no arm stated") + ", " + (
                            (m.get("stage") or {}).get("text") or "no pole and hand stated")
                        raise UserError(f"{name} is already in the library as {m['name']} ({was}): loading it "
                                        "again does not change its arm, pole or hand.",
                                        "Change them on that file in the list instead.")
                    return m
            fid = "u_" + uuid.uuid4().hex[:12]
            self.root.mkdir(parents=True, exist_ok=True)
            (self.root / f"{fid}.txt").write_bytes(data)
            folder = str(Path(path).parent) if path and Path(path).parent != Path(".") else ""
            meta = dict(id=fid, name=name, folder=folder, arm=arm or detect_arm(path or name), stage=stage,
                        lines=int(md.shape[0]), bytes=len(data), sha256=sha,
                        uploaded=time.strftime("%Y-%m-%d %H:%M"), source="upload")
            self._write_meta(meta)
            return meta

    def _write_meta(self, meta: dict) -> None:
        tmp = self._meta_path(meta["id"]).with_suffix(".json.tmp")
        tmp.write_text(json.dumps(meta, indent=1), encoding="utf-8")
        os.replace(tmp, self._meta_path(meta["id"]))

    def update(self, fid: str, *, arm: str | None = None, name: str | None = None, pole_mm=None, hand=None) -> dict:
        meta = self.get(fid)
        if meta.get("source") != "upload":
            raise UserError("Built-in files cannot be changed.")
        if arm is not None:
            if arm not in ("left", "right"):
                raise UserError("The arm is left or right.")
            meta["arm"] = arm
        if name:
            meta["name"] = Path(str(name)).name.strip()[:120] or meta["name"]
        if pole_mm is not None or hand is not None:
            meta["stage"] = staging.make_stage(pole_mm, hand, "stated for this file").as_dict()
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
        for s in builtin_sets():
            for f in s["files"]:
                if f["id"] == fid:
                    return dict(f)
        raise UserError("That file is not in the library any more.")

    def listing(self) -> dict:
        return {"files": [view(m) for m in self.uploads()],
                "sets": [dict(s, files=[view(f) for f in s["files"]]) for s in builtin_sets()],
                "folder": str(self.root)}

    def require_ready(self, fid: str, side: str | None = None) -> dict:
        """The file's record when it can be replayed or used: its arm and its stage are known,
        and ``side`` (when given) is its arm. Refuses otherwise, saying what is missing."""
        meta = self.get(fid)
        if meta.get("arm") not in ("left", "right"):
            raise UserError(f"Say which arm {meta['name']} was taught on first.", code="arm")
        if side is not None and side != meta["arm"]:
            raise UserError(f"{meta['name']} was taught on the {meta['arm'].upper()} arm: it is never shown "
                            f"or run on the {side.upper()} arm.", code="arm")
        if staging.stage_from(meta.get("stage")) is None:
            raise UserError(f"Say the pole height and hand pose {meta['name']} was taught with first: "
                            "without them its motion is not where the robot would put it.", code="stage")
        return meta

    def checks(self, fid: str) -> dict:
        """Every limit on the file as taught, on its own arm (L5 and L6 compare a tuned file with
        its recording, so they do not apply). Cached per file."""
        return self.checks_of(self.get(fid))

    def checks_of(self, meta: dict) -> dict:
        """``checks`` of a file by its record ({name, arm, path}): a library file's, or the file of a
        program on a controller (the Viewer's programs, api_library)."""
        from ..motion import track
        from ..motion.checks import check_bytes
        from ..motion.limits import load_limits

        side = meta.get("arm")
        if side not in ("left", "right"):
            raise UserError(f"Say which arm {meta['name']} was taught on first.", code="arm")
        key = hashlib.sha1(f"{meta['path']}:{side}:band-limited".encode()).hexdigest()[:16]
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


def standin_take(arm: str, unit: str | None = None, fixture: str | None = None) -> dict:
    """What the robot stand-in saves as a "taught" recording: a real file of the SAME arm with a
    known stage, never one of the other arm. The region's own existing recording first, else the
    first built-in file of that arm. Refuses when there is none."""
    from .project import library

    if unit and fixture:
        for name, meta in library(fixture)["files"].items():
            if meta.get("unit") == unit and meta.get("arm") == arm:
                st = _stage_dict(meta.get("pole_mm"), meta.get("hand"), "the taught set's record")
                if st:
                    return dict(path=meta["path"], name=name, arm=arm, stage=st)
    for s in builtin_sets():
        for f in s["files"]:
            if f["arm"] == arm and f.get("stage"):
                return dict(path=f["path"], name=f["name"], arm=arm, stage=f["stage"])
    raise UserError(f"The robot stand-in has no {arm.upper()}-arm recording to stand in for drag teaching.",
                    "Load a recording taught on this arm instead (Save & Review).")
