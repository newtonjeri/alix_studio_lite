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

Every recording also carries what it takes to execute it correctly (``staging.py``): the ARM it
was taught on (a recording of the other arm is refused, so a LEFT trajectory can never reach a
RIGHT-arm region) and its STAGE, the pole height and hand pose it was taught with. Tuned files
and kept versions inherit both from their recording. A region also has a SETUP: the pole and hand
it is taught with, from the catalogue or the existing recording, changed in Prepare.

The task queue (decision D35) pairs the catalogue's LEFT and RIGHT units the way the page
does: commode C by deck task (the same 22 tasks as the page, TASK 9 as the deck has it), other
fixtures by catalogue step. Arm steps outside Release 1 are marked ``later``.
"""
from __future__ import annotations

import json
import re
import time
from pathlib import Path

from . import staging
from .jobs import UserError
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
            r["taught_arm"], r["stage"] = self._identity(src)
        return out

    def _identity(self, src: dict) -> tuple[str | None, dict | None]:
        """The arm a recording was taught on and its stage. A recording kept before they were
        recorded with the file gets them back only when its bytes are a built-in file's (its
        record says both); an upload's arm can still come from its name, its stage cannot."""
        from .library import builtin_by_sha, detect_arm

        arm, stage = src.get("taught_arm"), src.get("stage")
        if arm is None or stage is None:
            known = builtin_by_sha(src.get("sha256"))
            if known:
                arm = arm or known.get("arm")
                stage = stage or known.get("stage")
            elif arm is None and src.get("source") == "upload":
                arm = detect_arm(src.get("original") or src.get("name") or "")
        return arm, stage

    # ── every file name in a project is used once (the owner, 14 Oct): two recordings of one name
    # could not be told apart in the lists, the project folder or the controller's program names

    def name_owner(self, name: str, skip: tuple | None = None) -> str | None:
        """The region whose recording already has ``name`` (any case), ``skip`` = (key, file) aside."""
        want = Path(str(name)).name.strip().casefold()
        for k, u in self.units.items():
            if (self.root / "trajectories" / u["arm"] / k / "raw").is_dir():
                if any((k, r["file"]) != skip and r["name"].casefold() == want for r in self.recordings(k)):
                    return k
        return None

    def free_name(self, name: str) -> str:
        """``name``, or ``<stem>_2``, ``_3``… when the project already has it."""
        p = Path(Path(str(name)).name.strip() or "recording.txt")
        stem, suffix, cand, n = p.stem, p.suffix or ".txt", p.name, 1
        while self.name_owner(cand):
            n += 1
            cand = f"{stem}_{n}{suffix}"
        return cand

    def taken_text(self, name: str, owner: str) -> str:
        return f"{Path(str(name)).name.strip()} is already the name of a recording in this project ({display_name(self.units[owner])})"

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
                      name: str | None = None, *, taught_arm: str | None = None, stage=None) -> Path:
        """Keep a recording for a region and make it the active one. ``taught_arm`` must be the
        region's arm (a trajectory never runs on the other arm), and ``stage`` (a staging.Stage or
        its dict) is the pole and hand it was taught with: without both, nothing is kept."""
        arm = self.units[key]["arm"]
        label = name or Path(src_path).name
        if taught_arm not in ("left", "right"):
            raise UserError(f"Say which arm {label} was taught on.", code="arm")
        if taught_arm != arm:
            raise UserError(f"{label} was taught on the {taught_arm.upper()} arm; {key} is a {arm.upper()}-arm "
                            f"region. A trajectory never runs on the other arm.", code="arm")
        st = stage if isinstance(stage, staging.Stage) else staging.stage_from(stage)
        if st is None:
            raise UserError(f"Say the pole height and hand pose {label} was taught with.", code="stage")
        with self.lock:
            if name and self.name_owner(name):              # a take or a file loaded again (a correction) is
                name = self.free_name(name)                 # kept under a numbered name, never refused
            dst = super().add_recording(key, src_path, note=note, source=source)
            self._update_source(dst, taught_arm=taught_arm, stage=st.as_dict())
            if name:
                self.rename_recording(key, dst.name, name)
            state = self._unit_state(key)
            state["active"] = dst.name
            self._write_state(key, state)
            return dst

    def _update_source(self, path: Path, **fields) -> None:
        meta = path.with_suffix(".source.json")
        d = _read_json(meta, {})
        d.update(fields)
        _atomic_write(meta, json.dumps(d, indent=1))

    def set_recording_stage(self, key: str, file: str, stage: staging.Stage, taught_arm: str | None = None) -> None:
        """State the stage (and arm) of a recording that has none recorded, or only one that is not
        valid (identity() asks for it then). A known stage is the recording's own record and is
        never overwritten."""
        rec = next((r for r in self.recordings(key) if r["file"] == file), None)
        if rec is None:
            raise KeyError(f"no recording {file!r} for {key}")
        known = staging.stage_from(rec["stage"])
        if known is not None:
            raise UserError(f"{rec['name']} already records its stage ({staging.describe(known)}); it cannot change.",
                            "If it is wrong, load the file again with the right pole and hand: that is a new "
                            "recording, tuned and kept on its own.")
        arm = rec["taught_arm"] or taught_arm
        if arm != self.units[key]["arm"]:
            raise UserError(f"{rec['name']} must have been taught on the {self.units[key]['arm'].upper()} arm to "
                            f"belong to {key}.", code="arm")
        self._update_source(self.unit_dir(key) / "raw" / file, taught_arm=arm, stage=stage.as_dict())

    def set_recording_hand(self, key: str, file: str, hand) -> dict:
        """The hand a recording runs with, changed by the operator after teaching (the owner, 14 Oct:
        what was forgotten in Prepare is set there and carries on to the later stages). Only the hand:
        the arm is mounted on the pole, so a recording runs only at the pole height it was taught
        at. The stage it had is kept in its record."""
        rec = next((r for r in self.recordings(key) if r["file"] == file), None)
        if rec is None:
            raise KeyError(f"no recording {file!r} for {key}")
        old = staging.stage_from(rec["stage"])
        if old is None:
            raise UserError(f"{rec['name']} has no pole and hand yet.", "State them first (State pole & hand…).",
                            code="stage")
        new = staging.make_stage(old.pole_mm, hand, "the hand set in Prepare after teaching")
        if staging.same_stage(new, old):
            return old.as_dict()
        path = self.unit_dir(key) / "raw" / file
        history = list(_read_json(path.with_suffix(".source.json"), {}).get("stage_history") or [])
        history.append(dict(at=time.strftime("%Y-%m-%d %H:%M:%S"), was=old.as_dict(), why=new.source))
        self._update_source(path, stage=new.as_dict(), stage_history=history)
        return new.as_dict()

    # ── the region's setup: the pole and hand it is taught with

    def setup(self, key: str) -> dict | None:
        st = self._unit_state(key).get("setup")
        if st:
            return st
        lib = self.library_file(key)
        if lib and lib.get("pole_mm") is not None and lib.get("hand") is not None:
            return staging.make_stage(lib["pole_mm"], lib["hand"], "the existing recording's record").as_dict()
        u = self.units[key]
        hand = u.get("hand_counts") or u.get("hand_preset")
        if u.get("pole_mm") is not None and hand:
            return staging.make_stage(u["pole_mm"], hand, "the region catalogue").as_dict()
        return None

    def set_setup(self, key: str, stage: staging.Stage) -> dict:
        with self.lock:
            state = self._unit_state(key)
            state["setup"] = stage.as_dict()
            self._write_state(key, state)
            return state["setup"]

    # ── kept versions carry their recording's arm and stage

    def save_candidate(self, key: str) -> dict:
        import shutil

        with self.lock:
            ver = super().save_candidate(key)
            extra = self.candidate_path(key).with_name("candidate.extra.json")
            if extra.is_file():
                shutil.copyfile(extra, Path(ver["path"]).with_name(Path(ver["path"]).stem + ".extra.json"))
            return self.current_version(key)

    def versions(self, key: str) -> list[dict]:
        out = super().versions(key)
        for v in out:
            extra = _read_json(Path(v["path"]).with_name(Path(v["path"]).stem + ".extra.json"), {})
            v["taught_arm"], v["stage"] = extra.get("taught_arm"), extra.get("stage")
        return out

    def rename_recording(self, key: str, file: str, name: str) -> None:
        name = Path(str(name)).name.strip()[:120]
        if not name:
            raise ValueError("a file name is needed")
        if file not in {r["file"] for r in self.recordings(key)}:     # one of this region's files, nothing else
            raise KeyError(f"no recording {file!r} for {key}")
        owner = self.name_owner(name, skip=(key, file))
        if owner:
            raise ValueError(self.taken_text(name, owner))
        side = self.unit_dir(key) / "raw" / file
        meta = side.with_suffix(".source.json")
        if not side.is_file():
            raise KeyError(f"no recording {file!r} for {key}")
        d = _read_json(meta, {})
        d["name"] = name
        _atomic_write(meta, json.dumps(d, indent=1))

    def import_library(self) -> list[str]:
        """Existing taught recordings of the fixture for the units they are mapped to (D31), each
        with the arm and the pole and hand its record gives."""
        done = []
        for name, meta in library(self.fixture)["files"].items():
            key = meta.get("unit")
            if key and key in self.units and self.units[key]["arm"] == meta.get("arm") and not self.recordings(key):
                self.add_recording(key, Path(meta["path"]), note=f"existing recording {name}: {meta.get('task', '')}",
                                   source="library", name=name, taught_arm=meta["arm"],
                                   stage=staging.make_stage(meta["pole_mm"], meta["hand"], "the taught set's record"))
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
        setup = self.setup(key)
        return dict(key=key, code=u["code"], part=u.get("part"), name=u["name"], display=display_name(u),
                    arm=u["arm"], later=not u.get("first_release", True), type=u.get("type"), kind=u.get("kind"),
                    phase=u.get("phase"), pole_mm=setup and setup["pole_mm"], hand_preset=u.get("hand_preset"),
                    hand_counts=setup and setup["hand"], setup=setup, target_s=u.get("target_s"),
                    picture=u.get("picture"), status=st["status"], status_label=st["label"],
                    recordings=st["recordings"], version=st["version"], library=self.library_file(key) is not None)


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
