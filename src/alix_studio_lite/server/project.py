"""The project on disk: one folder per fixture, one sub-folder per unit (arm x region).

    <data>/projects/<project>/
        project.json
        units/<KEY>/unit.json          settings (target, optimiser, segments, program ID), history
        units/<KEY>/raw/*.txt          recordings, never edited (+ .source.json)
        units/<KEY>/versions/vNNN.txt  saved optimised files (+ .lite.json)
        units/<KEY>/work/candidate.txt the latest optimiser result, not yet saved
        units/<KEY>/results.json       3D check and robot test results, tied to a file's sha256

Every write goes through a temporary file and a rename. A unit's status is derived from what
is on disk, never stored on its own.
"""
from __future__ import annotations

import copy
import json
import os
import shutil
import threading
import time
from pathlib import Path

from .. import vendor_paths

LIBRARIES = vendor_paths.ASSETS / "recordings"         # <fixture>.yaml: existing taught recordings
HISTORY_MAX = 20                      # at least 5 undo steps (R2); kept short and labelled

#: the pipeline, in order; a unit's status is the furthest step its CURRENT file has reached
STATUSES = [
    ("not_taught", "Not taught"),
    ("taught", "Taught"),
    ("optimised", "Optimised"),
    ("check_ok", "3D check OK"),
    ("uploaded", "On the robot"),
    ("dryrun_ok", "Hardware sim OK"),
    ("real_ok", "Tested on robot"),
]

DEFAULT_SETTINGS = dict(optimiser="default", target_s=None, segments=[], smooth_window=41,
                        program_id=None, max_dev_mm=None)



def _atomic_write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(text, encoding="utf-8")
    os.replace(tmp, path)


def _read_json(path: Path, default):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return copy.deepcopy(default)


class Project:
    def __init__(self, root: Path, fixture: str = "commode_c"):
        from . import fixtures

        self.root = Path(root)
        self.fixture = fixture
        self.lock = threading.RLock()
        self.catalogue = fixtures.catalogue(fixture)
        self.units = {u["key"]: u for u in self.catalogue["units"]}
        self.meta = _read_json(self.root / "project.json", {})

    # ── folders
    def unit_dir(self, key: str) -> Path:
        if key not in self.units:
            raise KeyError(f"no unit {key!r} in the catalogue")
        return self.root / "units" / key

    # ── settings and history
    def settings(self, key: str) -> dict:
        st = _read_json(self.unit_dir(key) / "unit.json", {})
        s = dict(DEFAULT_SETTINGS)
        s.update(st.get("settings") or {})
        if s.get("target_s") is None and self.units[key].get("target_s"):
            s["target_s"] = self.units[key]["target_s"]
        return s

    def _state(self, key: str) -> dict:
        st = _read_json(self.unit_dir(key) / "unit.json", {})
        st.setdefault("settings", {})
        st.setdefault("history", [])
        st.setdefault("cursor", len(st["history"]) - 1)
        return st

    def update_settings(self, key: str, changes: dict, label: str) -> dict:
        """Apply a settings change as one labelled undo step."""
        with self.lock:
            st = self._state(key)
            cur = self.settings(key)
            new = dict(cur)
            new.update({k: v for k, v in changes.items() if k in DEFAULT_SETTINGS})
            if new == cur:
                return self.history_view(key)
            hist = st["history"][: st["cursor"] + 1] if st["history"] else [dict(label="Start", settings=cur)]
            hist.append(dict(label=label, settings=new, at=time.time()))
            st["history"] = hist[-HISTORY_MAX:]
            st["cursor"] = len(st["history"]) - 1
            st["settings"] = new
            _atomic_write(self.unit_dir(key) / "unit.json", json.dumps(st, indent=1))
            return self.history_view(key)

    def step_history(self, key: str, delta: int) -> dict:
        with self.lock:
            st = self._state(key)
            if not st["history"]:
                return self.history_view(key)
            c = max(0, min(len(st["history"]) - 1, st["cursor"] + int(delta)))
            st["cursor"] = c
            st["settings"] = dict(st["history"][c]["settings"])
            _atomic_write(self.unit_dir(key) / "unit.json", json.dumps(st, indent=1))
            return self.history_view(key)

    def history_view(self, key: str) -> dict:
        st = self._state(key)
        h, c = st["history"], st["cursor"]
        return dict(settings=self.settings(key), can_undo=c > 0, can_redo=0 <= c < len(h) - 1,
                    undo_label=h[c]["label"] if c > 0 else None,
                    redo_label=h[c + 1]["label"] if 0 <= c < len(h) - 1 else None,
                    steps=[dict(label=x["label"], current=i == c) for i, x in enumerate(h)])

    # ── files
    def recordings(self, key: str) -> list[dict]:
        d = self.unit_dir(key) / "raw"
        out = []
        for p in sorted(d.glob("*.txt")) if d.is_dir() else []:
            src = _read_json(p.with_suffix(".source.json"), {})
            out.append(dict(file=p.name, path=str(p), taken=src.get("taken"), lines=src.get("lines"),
                            note=src.get("note"), sha256=src.get("sha256"), source=src.get("source", "teach"),
                            original=src.get("original")))
        return out

    def current_recording(self, key: str) -> Path | None:
        r = self.recordings(key)
        return Path(r[-1]["path"]) if r else None

    def add_recording(self, key: str, src_path: Path, note: str = "", source: str = "teach") -> Path:
        """Keep a recording for a unit, read-only. ``source``: teach | library | upload."""
        from ..motion import track

        md = track.read_md(src_path)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        dst = self.unit_dir(key) / "raw" / f"{stamp}_{key}.txt"
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src_path, dst)
        os.chmod(dst, 0o444)                                    # raw recordings are never edited
        _atomic_write(dst.with_suffix(".source.json"), json.dumps(dict(
            unit=key, arm=self.units[key]["arm"], taken=time.strftime("%Y-%m-%d %H:%M"),
            lines=int(md.shape[0]), sha256=track.sha256_file(dst), note=note, source=source,
            original=Path(src_path).name,
            pole_mm=self.units[key].get("pole_mm"), hand_counts=self.units[key].get("hand_counts")), indent=1))
        return dst

    def versions(self, key: str) -> list[dict]:
        d = self.unit_dir(key) / "versions"
        out = []
        for p in sorted(d.glob("v*.txt")) if d.is_dir() else []:
            rec = _read_json(p.with_name(p.stem + ".lite.json"), {})
            res = rec.get("result") or {}
            out.append(dict(version=p.stem, file=p.name, path=str(p), sha256=rec.get("track_sha256"),
                            optimiser=res.get("optimiser"), lines=res.get("lines"),
                            achieved_s=res.get("achieved_s"), uploadable=res.get("uploadable"),
                            saved=rec.get("written_utc")))
        return out

    def current_version(self, key: str) -> dict | None:
        v = self.versions(key)
        return v[-1] if v else None

    def candidate_path(self, key: str) -> Path:
        return self.unit_dir(key) / "work" / "candidate.txt"

    def save_candidate(self, key: str) -> dict:
        with self.lock:
            cand = self.candidate_path(key)
            if not cand.is_file():
                raise FileNotFoundError("nothing to save: optimise first")
            d = self.unit_dir(key) / "versions"
            d.mkdir(parents=True, exist_ok=True)
            n = len(list(d.glob("v*.txt"))) + 1
            dst = d / f"v{n:03d}.txt"
            shutil.copyfile(cand, dst)
            side = cand.with_name(cand.stem + ".lite.json")
            if side.is_file():
                shutil.copyfile(side, dst.with_name(dst.stem + ".lite.json"))
            return self.current_version(key)

    # ── results (3D check, robot tests, uploads), tied to the file's sha256
    def results(self, key: str) -> dict:
        return _read_json(self.unit_dir(key) / "results.json", {"events": []})

    def add_result(self, key: str, kind: str, sha256: str | None, ok: bool, detail: dict | None = None) -> None:
        with self.lock:
            r = self.results(key)
            r["events"].append(dict(kind=kind, sha256=sha256, ok=bool(ok), at=time.strftime("%Y-%m-%d %H:%M:%S"),
                                    detail=detail or {}))
            _atomic_write(self.unit_dir(key) / "results.json", json.dumps(r, indent=1))

    # ── status
    def status(self, key: str) -> dict:
        unit = self.units[key]
        rec = self.recordings(key)
        ver = self.current_version(key)
        status, flags = "not_taught", []
        if rec:
            status = "taught"
        if ver:
            status = "optimised"
            sha = ver.get("sha256")
            ev = [e for e in self.results(key)["events"] if e.get("sha256") == sha and e.get("ok")]
            kinds = {e["kind"] for e in ev}
            for k in ("check_ok", "uploaded", "dryrun_ok", "real_ok"):
                if k in kinds:
                    status = k
        if unit.get("legacy_v6") in ("covered", "partial", "merged") and not rec:
            flags.append("legacy_v6")
        if not unit["first_release"]:
            flags.append("later")
        idx = [s for s, _ in STATUSES].index(status)
        return dict(status=status, label=dict(STATUSES)[status], index=idx, flags=flags,
                    recordings=len(rec), version=ver["version"] if ver else None)

    def summary(self) -> dict:
        units = []
        for key, u in self.units.items():
            s = self.status(key)
            units.append(dict(u, **s, settings=self.settings(key)))
        first = [u for u in units if u["first_release"]]
        counts = {s: sum(1 for u in first if u["status"] == s) for s, _ in STATUSES}
        return dict(project=self.meta, phases=self.catalogue["phases"], statuses=[dict(id=s, label=l) for s, l in STATUSES],
                    hand_presets=self.catalogue["hand_presets"], units=units, counts=counts,
                    first_release=len(first))


def library(fixture: str) -> dict:
    """Existing taught recordings for a fixture (``assets/recordings/<fixture>.yaml``):
    ``{folder, files: {name: {task, arm, pole_mm, hand, max_dev_mm, unit}}}``. Empty if none."""
    import yaml

    f = LIBRARIES / f"{fixture}.yaml"
    if not f.is_file():
        return {"folder": None, "files": {}}
    d = yaml.safe_load(f.read_text(encoding="utf-8")) or {}
    folder = Path(d.get("folder") or "")
    if not folder.is_absolute():
        folder = vendor_paths.LITE_ROOT / folder
    files = {}
    for name, meta in (d.get("files") or {}).items():
        path = folder / name
        if path.is_file():
            lines = sum(1 for ln in path.read_bytes().split(b"\n") if ln.strip())
            files[name] = dict(meta or {}, file=name, path=str(path), lines=lines)
    return {"folder": str(folder), "files": files}


def open_project(data_dir: Path, fixture: str = "commode_c") -> Project:
    """Open the fixture's project, creating it on first start. A new project uses the existing
    taught recordings of the fixture's library for the regions they are mapped to (decision D31)."""
    from . import fixtures

    root = Path(data_dir) / "projects" / fixture
    new = not (root / "project.json").is_file()
    root.mkdir(parents=True, exist_ok=True)
    if new:
        _atomic_write(root / "project.json", json.dumps(dict(
            name=next(f["name"] for f in fixtures.list_fixtures() if f["id"] == fixture), fixture=fixture,
            created=time.strftime("%Y-%m-%d %H:%M")), indent=1))
    p = Project(root, fixture)
    if new:
        for name, meta in library(fixture)["files"].items():
            key = meta.get("unit")
            if key and key in p.units and p.units[key]["arm"] == meta.get("arm"):
                p.add_recording(key, Path(meta["path"]), note=f"existing recording {name}: {meta.get('task', '')}",
                                source="library")
    return p
