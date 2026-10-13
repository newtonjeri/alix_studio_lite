"""A Lite UI project's regions: the files of each arm step, and the task queue the page shows.

Each arm step is a unit of the fixture's region catalogue (``L-R01``, ``R-R05`` …). Its files
live in the project folder's handoff layout::

    <project>/trajectories/<arm>/<KEY>/
        unit.json             settings (with undo history) and the active recording
        raw/*.txt             recordings: taught, existing (library) or loaded; never edited
        work/candidate.*      the latest tuning result, not yet kept
        versions/vNNN.*       kept tuned files, the ones that can be uploaded
        results.json          3D checks, uploads, runs and verdicts, tied to a file's sha256

This reuses Legacy Lite's unit store (``project.Project``) with the folder moved, plus what the
page adds: its processing switches, a time mode, the active recording (a loaded file never
replaces a new recording) and the operator's name for a file.

The task queue (decision D35) pairs the catalogue's LEFT and RIGHT units the way the page
does: commode C by deck task (the same 22 tasks as the page, TASK 9 as the deck has it), other
fixtures by catalogue step. Arm steps outside Release 1 are marked ``later``.
"""
from __future__ import annotations

import json
import re
import time
from pathlib import Path

from .project import DEFAULT_SETTINGS, Project, _atomic_write, _read_json, library

#: the page's processing switches and the time mode; smoothing 41 lines is R11's Standard
LITE_DEFAULTS = dict(DEFAULT_SETTINGS, trim=True, smooth=True, smooth_window=41, scale=True, cuts=[],
                     time_mode="natural")
TIME_MODES = ("natural", "target", "fastest")


class LiteUnits(Project):
    DEFAULTS = LITE_DEFAULTS

    def __init__(self, project_dir: Path, fixture: str):
        super().__init__(project_dir, fixture)

    def unit_dir(self, key: str) -> Path:
        if key not in self.units:
            raise KeyError(f"no region {key!r} in this project's catalogue")
        return self.root / "trajectories" / self.units[key]["arm"] / key

    # ── the active recording and the operator's file names

    def _unit_state(self, key: str) -> dict:
        return _read_json(self.unit_dir(key) / "unit.json", {})

    def _write_state(self, key: str, st: dict) -> None:
        _atomic_write(self.unit_dir(key) / "unit.json", json.dumps(st, indent=1))

    def recordings(self, key: str) -> list[dict]:
        out = super().recordings(key)
        for r in out:
            src = _read_json(Path(r["path"]).with_suffix(".source.json"), {})
            r["name"] = src.get("name") or r.get("original") or r["file"]
            r["seconds"] = round(0.005 * max(int(r.get("lines") or 1) - 1, 0), 3)
        return out

    def current_recording(self, key: str) -> Path | None:
        recs = self.recordings(key)
        if not recs:
            return None
        active = self._unit_state(key).get("active")
        pick = next((r for r in recs if r["file"] == active), recs[-1])
        return Path(pick["path"])

    def set_active(self, key: str, file: str) -> None:
        with self.lock:
            if not any(r["file"] == file for r in self.recordings(key)):
                raise KeyError(f"no recording {file!r} for {key}")
            st = self._unit_state(key)
            st["active"] = file
            self._write_state(key, st)

    def add_recording(self, key: str, src_path: Path, note: str = "", source: str = "teach",
                      name: str | None = None) -> Path:
        with self.lock:
            dst = super().add_recording(key, src_path, note=note, source=source)
            if name:
                self.rename_recording(key, dst.name, name)
            st = self._unit_state(key)
            st["active"] = dst.name
            self._write_state(key, st)
            return dst

    def rename_recording(self, key: str, file: str, name: str) -> None:
        name = Path(str(name)).name.strip()[:120]
        if not name:
            raise ValueError("a file name is needed")
        side = self.unit_dir(key) / "raw" / file
        meta = side.with_suffix(".source.json")
        if not side.is_file():
            raise KeyError(f"no recording {file!r} for {key}")
        d = _read_json(meta, {})
        d["name"] = name
        _atomic_write(meta, json.dumps(d, indent=1))

    def import_library(self) -> list[str]:
        """Existing taught recordings of the fixture for the units they are mapped to (D31)."""
        done = []
        for name, meta in library(self.fixture)["files"].items():
            key = meta.get("unit")
            if key and key in self.units and self.units[key]["arm"] == meta.get("arm") and not self.recordings(key):
                self.add_recording(key, Path(meta["path"]), note=f"existing recording {name}: {meta.get('task', '')}",
                                   source="library", name=name)
                done.append(key)
        return done

    def library_file(self, key: str) -> dict | None:
        files = library(self.fixture)["files"]
        return next((f for f in files.values() if f.get("unit") == key and f.get("arm") == self.units[key]["arm"]), None)

    # ── results

    def last_run(self, key: str, sha256: str | None) -> dict | None:
        runs = [e for e in self.results(key)["events"]
                if e["kind"] in ("dryrun_ok", "real_ok", "run_failed") and e.get("sha256") == sha256]
        return runs[-1] if runs else None

    # ── the page's view of one arm step

    def step_view(self, key: str) -> dict:
        u = self.units[key]
        st = self.status(key)
        return dict(key=key, code=u["code"], part=u.get("part"), name=u["name"], display=display_name(u),
                    arm=u["arm"], later=not u.get("first_release", True), type=u.get("type"), kind=u.get("kind"),
                    phase=u.get("phase"), pole_mm=u.get("pole_mm"), hand_preset=u.get("hand_preset"),
                    hand_counts=u.get("hand_counts"), target_s=u.get("target_s"), picture=u.get("picture"),
                    status=st["status"], status_label=st["label"], recordings=st["recordings"],
                    version=st["version"], library=self.library_file(key) is not None)


def display_name(u: dict) -> str:
    """The page's way of naming an arm step: code, then name (``L-R05 SEAT TOP PART 1``)."""
    return f"{u['code']} {u['name']}".strip()


def _expand(deck_task: str | None) -> list[str]:
    """``10A-10B`` -> ``['10A', '10B']``: one action that spans two tasks."""
    if not deck_task:
        return []
    if "-" not in deck_task:
        return [deck_task]
    a, b = deck_task.split("-", 1)
    ma, mb = re.fullmatch(r"(\d+)([A-Z]?)", a), re.fullmatch(r"(\d+)([A-Z]?)", b)
    if ma and mb and ma.group(1) == mb.group(1) and ma.group(2) and mb.group(2):
        return [ma.group(1) + chr(c) for c in range(ord(ma.group(2)), ord(mb.group(2)) + 1)]
    return [a, b]


def _kind(left: dict | None, right: dict | None) -> str:
    steps = [s for s in (left, right) if s]
    if all(s["later"] for s in steps):
        return "procedure"
    if any("Lid/seat" in (s.get("type") or "") for s in steps):
        return "action"
    if any(s["later"] for s in steps) or len(steps) == 1:
        return "mixed"
    return "clean"


def task_list(units: LiteUnits) -> list[dict]:
    """The task queue: one entry per task with its LEFT and RIGHT arm steps (either may be None)."""
    cat = list(units.units.values())
    by_task: dict[str, dict] = {}
    order: list[str] = []
    deck = any(u.get("deck_task") for u in cat)
    for u in sorted(cat, key=lambda u: (u.get("step") or 0, u["arm"] != "left")):
        labels = _expand(u.get("deck_task")) if deck else [str(u.get("step"))]
        for lab in labels:
            if lab not in by_task:
                by_task[lab] = dict(label=f"TASK {lab}", slide=u.get("deck_slide"), left=None, right=None)
                order.append(lab)
            t = by_task[lab]
            if t[u["arm"]] is None:
                t[u["arm"]] = units.step_view(u["key"])
            if t["slide"] is None:
                t["slide"] = u.get("deck_slide")
    out = []
    for lab in order:
        t = by_task[lab]
        t["kind"] = _kind(t["left"], t["right"])
        later = [s for s in (t["left"], t["right"]) if s and s["later"]]
        t["note"] = "Not in Release 1: added later." if later and t["kind"] != "procedure" else (
            "Procedure to be added later" if t["kind"] == "procedure" else None)
        out.append(t)
    return out


def open_units(store, pid: str) -> LiteUnits:
    """The regions of a project in the store, importing the fixture's existing recordings once."""
    rec = store.get(pid)
    lite = rec.get("lite") or {}
    units = LiteUnits(store.folder(pid), lite.get("fixture") or store.default_fixture)
    if not lite.get("library_imported"):
        imported = units.import_library()
        store.set_lite(pid, library_imported=time.strftime("%Y-%m-%d %H:%M"), library_units=imported)
    return units
