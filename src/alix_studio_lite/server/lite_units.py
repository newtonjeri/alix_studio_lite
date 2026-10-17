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

#: manual tuning's stages and parameters, the trajectory HTML's defaults (the owner, 15 Oct: "with
#: everything adopted, including the default values"), and Time-optimal's time: as fast as the limits
#: allow, or a target. A region already tuned keeps the values it was saved with.
LITE_DEFAULTS = dict(DEFAULT_SETTINGS, trim=True, smooth=True, smooth_window=101, scale=True,
                     time_mode="fastest", idle_window=11, idle_threshold_md=8.0, idle_min_run=20)
TIME_MODES = ("fastest", "target")


PHASES = ("lid_closed", "lid_open", "seat_open")


def phase_default(u: dict) -> str:
    """A region's commode state as attached (the owner, 16 Oct: "every region should have attached
    commode state"): the catalogue's; for the lid and seat operations, which it leaves without one, the
    state the robot meets at their start; the glove tasks, lid closed."""
    if u.get("phase") in PHASES:
        return u["phase"]
    name = str(u.get("name") or "").upper()
    if "OPEN LID" in name:
        return "lid_closed"
    if "CLOSE SEAT ONLY" in name:
        return "seat_open"
    if "OPEN SEAT" in name:
        return "lid_open"
    if "CLOSE SEAT" in name and "LID" in name:
        return "seat_open"
    return "lid_closed"


class _Units(dict):
    """The catalogue's arm steps by key, and the merges opened in Tune + Simulation (``extra``): a key
    of either resolves, so every unit route works on a merge in Tune, but listing them (the task
    queue, the summaries, the project's file names) gives the catalogue's alone."""

    def __init__(self, catalogue: dict):
        super().__init__(catalogue)
        self.extra: dict = {}

    def __getitem__(self, key):
        return self.extra[key] if key in self.extra else super().__getitem__(key)

    def __contains__(self, key):
        return key in self.extra or super().__contains__(key)

    def get(self, key, default=None):
        return self.extra[key] if key in self.extra else super().get(key, default)


class LiteUnits(Project):
    DEFAULTS = LITE_DEFAULTS

    def __init__(self, project_dir: Path, fixture: str):
        super().__init__(project_dir, fixture)
        self.units = _Units(self.units)
        self._load_tune_units()

    def unit_dir(self, key: str) -> Path:
        if key not in self.units:
            raise KeyError(f"no region {key!r} in this project's catalogue")
        if key in self.units.extra:
            return self.tune_dir() / key
        return self.root / "trajectories" / self.units[key]["arm"] / key

    # ── the commode's state a region is taught and run in: attached (phase_default), set in Prepare
    def phase_of(self, key: str) -> str:
        st = self._unit_state(key).get("phase")
        return st if st in PHASES else phase_default(self.units[key])

    def set_phase(self, key: str, phase: str) -> str:
        if phase not in PHASES:
            raise UserError("The commode's state is lid closed, lid open or seat open.")
        with self.lock:
            state = self._unit_state(key)
            state["phase"] = phase
            self._write_state(key, state)
        return phase

    # ── a merge opened in Tune + Simulation (feedback v5, slide 9; the owner, 16 Oct: Tune opens the
    # merged file as its source, and what is kept there goes back to S1 as a new merge). It is a unit
    # of its own, kept with the merges (merged/tune/<KEY>/), its one recording the merged file with the
    # merge's arm, pole and hand, and placement, so Tune's every step works on it as on a region.

    def tune_dir(self) -> Path:
        return self.root / "merged" / "tune"

    def is_merge_unit(self, key: str) -> bool:
        return key in self.units.extra

    def _load_tune_units(self) -> None:
        d = self.tune_dir()
        for f in sorted(d.glob("*/merge.json")) if d.is_dir() else []:
            m = _read_json(f, {})
            if m.get("key") and m.get("side") in ("left", "right"):
                self.units.extra[m["key"]] = self._tune_unit(m)

    @staticmethod
    def _tune_unit(m: dict) -> dict:
        st = m.get("stage") or {}
        return dict(key=m["key"], code="MERGE", name=Path(str(m["name"])).stem, arm=m["side"], step=None, deck_task=None,
                    phase=m.get("phase"), first_release=True, type="merge", kind="merge", hand_preset=None,
                    pole_mm=st.get("pole_mm"), hand_counts=st.get("hand"), target_s=None, picture=None,
                    merge=dict(side=m["side"], file=m["file"], name=m["name"], sha256=m.get("sha256"),
                               placement=m.get("placement")))

    def tune_unit_for(self, merge: dict) -> str:
        """The Tune unit of a merge (``merges.get``), made the first time it is opened there."""
        side = merge["side"]
        with self.lock:
            for k, u in self.units.extra.items():
                if u["merge"]["side"] == side and u["merge"]["file"] == merge["file"] and u["merge"]["sha256"] == merge["sha256"]:
                    return k
            prefix = f"MERGE-{side[0].upper()}-"
            n = 1 + max([int(k[len(prefix):]) for k in self.units.extra if k.startswith(prefix) and k[len(prefix):].isdigit()]
                        or [0])
            key = f"{prefix}{n:03d}"
            meta = dict(key=key, side=side, file=merge["file"], name=merge["name"], sha256=merge["sha256"],
                        stage=merge["stage"], placement=merge.get("placement"), phase=merge.get("phase"),
                        made=time.strftime("%Y-%m-%d %H:%M:%S"))
            d = self.tune_dir() / key
            d.mkdir(parents=True, exist_ok=True)
            _atomic_write(d / "merge.json", json.dumps(meta, indent=1))
            self.units.extra[key] = self._tune_unit(meta)
            dst = self.add_recording(key, Path(merge["path"]), note=f"the {side.upper()} merge {merge['name']}", source="merge",
                                     taught_arm=side, stage=merge["stage"], placement=merge.get("placement"))
            self._update_source(dst, name=merge["name"])
            state = self._unit_state(key)
            state["setup"] = merge["stage"]
            self._write_state(key, state)
            return key

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
            r["placement"] = src.get("placement")           # the task's UGV placement it was taught at
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
            d = self.root / "trajectories" / u["arm"] / k
            if (d / "raw").is_dir():
                if any((k, r["file"]) != skip and r["name"].casefold() == want for r in self.recordings(k)):
                    return k
            if (d / "versions").is_dir():                   # the tuned files kept under a name too
                if any((k, f"version:{v['version']}") != skip and (v.get("name") or "").casefold() == want
                       for v in self.versions(k)):
                    return k
            t = self.root / "trajectories" / "transitions" / u["arm"] / k
            if t.is_dir() and any(str(_read_json(m, {}).get("name") or "").casefold() == want
                                  for m in t.glob("*.source.json")):           # and the transition trajectories
                return k
        for side in ("left", "right"):                                          # and the merges
            d = self.root / "merged" / side
            for m in sorted(d.glob("*.source.json")) if d.is_dir() else []:
                file = m.name[:-len(".source.json")] + ".txt"
                if ("merge", f"{side}/{file}") != skip and str(_read_json(m, {}).get("name") or "").casefold() == want:
                    return f"merge:{side}"
            for m in sorted((d / "loaded").glob("*.source.json")) if (d / "loaded").is_dir() else []:   # and S1's loaded files
                if str(_read_json(m, {}).get("name") or "").casefold() == want:
                    return f"loaded:{side}"
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
        if owner.startswith("loaded:"):
            return f"{Path(str(name)).name.strip()} is already the name of a {owner[7:].upper()} file loaded into S1 in this project"
        if owner.startswith("merge:"):
            return f"{Path(str(name)).name.strip()} is already the name of a {owner[6:].upper()} merge in this project"
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
            was = self.current_recording(key)
            st = self._unit_state(key)
            st["active"] = file
            self._write_state(key, st)
            if was is None or was.name != file:
                self._regions_cleared(key, file)

    def _regions_cleared(self, key: str, file: str) -> None:
        """The scale regions are rows of the recording they were set on: another recording starts
        without them, as the trajectory HTML clears them with every file it loads (one undo step)."""
        if self.settings(key).get("segments"):
            name = next((r.get("name") or r["file"] for r in self.recordings(key) if r["file"] == file), file)
            self.update_settings(key, {"segments": []}, f"Scale regions cleared: another recording ({name})")

    def add_recording(self, key: str, src_path: Path, note: str = "", source: str = "teach",
                      name: str | None = None, *, taught_arm: str | None = None, stage=None,
                      placement: dict | None = None) -> Path:
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
            if placement:
                self._update_source(dst, placement=placement)
            if name:
                self.rename_recording(key, dst.name, name)
            state = self._unit_state(key)
            state["active"] = dst.name
            self._write_state(key, state)
            self._regions_cleared(key, dst.name)
            return dst

    def trim_recording(self, key: str, file: str, start: int, end: int, name: str | None = None) -> Path:
        """Save & Review's Trim (feedback v5, slide 1; the controller's Track clip): keep lines ``start``
        to ``end`` (1-based, both kept) of a recording. With ``name``, a new recording (Save As) that
        keeps the original's arm, stage and placement and becomes the active one; without it the
        recording itself is overwritten (Save: the owner's choice, 16 Oct), its record updated. Either
        way its scale regions are cleared: they were rows of the lines before."""
        import os
        import tempfile

        from ..motion import track

        rec = next((r for r in self.recordings(key) if r["file"] == file), None)
        if rec is None:
            raise KeyError(f"no recording {file!r} for {key}")
        md = track.read_md(rec["path"])
        n, a, b = int(md.shape[0]), int(start), int(end)
        if not 1 <= a < b <= n:
            raise UserError(f"Keep lines 1 to {n:,} of {rec['name']}: the first before the last, two lines at least.")
        kept = md[a - 1:b]
        note = f"trimmed to lines {a:,}–{b:,} of {n:,} ({(b - a) * 0.005:.3f} s)"
        with self.lock:
            if name:
                tmp = Path(tempfile.mkdtemp(prefix="lite-trim-")) / Path(name).name
                track.write_md(tmp, kept)
                return self.add_recording(key, tmp, note=note, source="trim", name=name, taught_arm=rec["taught_arm"],
                                          stage=rec["stage"], placement=rec.get("placement"))
            path = Path(rec["path"])
            track.write_md(path, kept)                      # replaced whole: the read-only file is never edited
            os.chmod(path, 0o444)
            self._update_source(path, lines=int(kept.shape[0]), sha256=track.sha256_file(path), note=note,
                                trimmed=dict(first=a, last=b, of=n, at=time.strftime("%Y-%m-%d %H:%M:%S")))
            if self.settings(key).get("segments"):
                self.update_settings(key, {"segments": []}, f"Scale regions cleared: {rec['name']} was trimmed")
            return path

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

    def mark_applied(self, key: str, kind: str | None, stage: staging.Stage) -> dict:
        """Prepare's Apply Pole / Apply Hand pressed and done (the owner, 15 Oct: teaching starts only
        after both): the values each one applied."""
        with self.lock:
            state = self._unit_state(key)
            done = state.get("applied") or {}
            for k in ((kind,) if kind in ("pole", "hand") else ("pole", "hand")):
                done[k] = {"pole_mm": stage.pole_mm, "hand": list(stage.hand), "at": time.strftime("%Y-%m-%d %H:%M:%S")}
            state["applied"] = done
            self._write_state(key, state)
            return done

    def applied(self, key: str) -> dict:
        return self._unit_state(key).get("applied") or {}

    def set_setup(self, key: str, stage: staging.Stage) -> dict:
        with self.lock:
            state = self._unit_state(key)
            state["setup"] = stage.as_dict()
            self._write_state(key, state)
            return state["setup"]

    # ── kept versions carry their recording's arm and stage

    def save_candidate(self, key: str) -> dict:
        """Keep the candidate as the next version; it becomes the version the region runs."""
        import shutil

        with self.lock:
            super().save_candidate(key)
            newest = super().versions(key)[-1]
            extra = self.candidate_path(key).with_name("candidate.extra.json")
            if extra.is_file():
                shutil.copyfile(extra, Path(newest["path"]).with_name(Path(newest["path"]).stem + ".extra.json"))
            state = self._unit_state(key)
            state["version"] = newest["version"]
            self._write_state(key, state)
            return self.current_version(key)

    def current_version(self, key: str) -> dict | None:
        """The kept version the region runs (Real Test, the full sequence): the newest, unless another
        was chosen in the flow table (the owner, 15 Oct)."""
        vs = self.versions(key)
        if not vs:
            return None
        chosen = self._unit_state(key).get("version")
        return next((v for v in vs if v["version"] == chosen), vs[-1])

    def choose_version(self, key: str, version: str) -> dict:
        with self.lock:
            v = next((v for v in self.versions(key) if v["version"] == version), None)
            if v is None:
                raise UserError(f"There is no version {version} of {key}.")
            state = self._unit_state(key)
            state["version"] = version
            self._write_state(key, state)
            return v

    def versions(self, key: str) -> list[dict]:
        out = super().versions(key)
        for v in out:
            extra = _read_json(Path(v["path"]).with_name(Path(v["path"]).stem + ".extra.json"), {})
            v["taught_arm"], v["stage"] = extra.get("taught_arm"), extra.get("stage")
            v["name"] = extra.get("name")                 # the tuned file's name (V32's Save Tuned File As…)
        return out

    def name_version(self, key: str, version: str, name: str) -> dict:
        """Names a kept version (V32's "Save Tuned File As…"): one name per file in a project, as
        for the recordings."""
        name = Path(str(name)).name.strip()[:120]
        if not name:
            raise UserError("A file name is needed.")
        if not name.lower().endswith(".txt"):
            name += ".txt"
        with self.lock:
            ver = next((v for v in self.versions(key) if v["version"] == version), None)
            if ver is None:
                raise UserError(f"There is no version {version} of {key}.")
            owner = self.name_owner(name, skip=(key, f"version:{version}"))
            if owner:
                raise UserError(f"{self.taken_text(name, owner)}.", "Every file name in a project is used once: "
                                "choose another name.", code="name")
            extra = Path(ver["path"]).with_name(Path(ver["path"]).stem + ".extra.json")
            d = _read_json(extra, {})
            d["name"] = name
            _atomic_write(extra, json.dumps(d, indent=1))
            return next(v for v in self.versions(key) if v["version"] == version)

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

    # ── a transition trajectory per arm step (V32's Prepare, Task Transition & Start Point Tools): a
    # file that takes the arm from where the previous task left it toward this task's start. Kept in
    # the project where the handoff puts it (trajectories/transitions/<arm>/<KEY>/), with the arm and
    # the stage it was taught with, like every recording; the latest is the step's transition.

    def transition_dir(self, key: str) -> Path:
        if key not in self.units:
            raise KeyError(f"no region {key!r} in this project's catalogue")
        return self.root / "trajectories" / "transitions" / self.units[key]["arm"] / key

    def transition(self, key: str) -> dict | None:
        """The step's transition: its record (name, arm, stage, lines, sha256, first and last line,
        limits) with ``file`` and ``path``; None when it has none."""
        f = self._unit_state(key).get("transition")
        path = self.transition_dir(key) / str(f or "")
        if not f or not path.is_file():
            return None
        return dict(_read_json(path.with_suffix(".source.json"), {}), file=f, path=str(path))

    def set_transition(self, key: str, src: Path, name: str, *, taught_arm: str | None, stage) -> dict:
        """Keep ``src`` as the step's transition. Like a recording, it must be this arm's file and
        say the pole and hand it was taught with; its name is numbered when the project has it."""
        import shutil
        from dataclasses import asdict

        from ..motion import track
        from ..motion.checks import check_bytes, failures_text
        from ..motion.limits import load_limits

        arm = self.units[key]["arm"]
        if taught_arm not in ("left", "right"):
            raise UserError(f"Say which arm {name} was taught on.", code="arm")
        if taught_arm != arm:
            raise UserError(f"{name} was taught on the {taught_arm.upper()} arm; it cannot be the {arm.upper()} arm's "
                            "transition. A trajectory never runs on the other arm.", code="arm")
        st = stage if isinstance(stage, staging.Stage) else staging.stage_from(stage)
        if st is None:
            raise UserError(f"Say the pole height and hand pose {name} was taught with.", code="stage")
        md = track.read_md(src)
        if md.shape[0] < 2:
            raise UserError(f"{name} has fewer than two lines.")
        rep = check_bytes(md, load_limits(arm), with_departure=False)
        bad = [asdict(c) for c in rep.checks if c.hard and c.passed is False and c.id in ("L1", "L2", "L3", "L4")]
        with self.lock:
            if self.name_owner(name):
                name = self.free_name(name)
            d = self.transition_dir(key)
            d.mkdir(parents=True, exist_ok=True)
            stem = re.sub(r"[^A-Za-z0-9_.\-]+", "_", Path(name).stem)[:60] or "transition"
            dst = d / f"{time.strftime('%Y%m%d_%H%M%S')}_{stem}.txt"
            shutil.copyfile(src, dst)
            n = int(md.shape[0])
            meta = dict(name=name, taught_arm=taught_arm, stage=st.as_dict(), sha256=track.sha256_file(dst), lines=n,
                        seconds=round(0.005 * (n - 1), 3), first_millideg=[int(v) for v in md[0]],
                        end_millideg=[int(v) for v in md[-1]], limits_ok=not bad,
                        limits_text=failures_text({"checks": bad}), added=time.strftime("%Y-%m-%d %H:%M:%S"))
            _atomic_write(dst.with_suffix(".source.json"), json.dumps(meta, indent=1))
            state = self._unit_state(key)
            state["transition"] = dst.name
            self._write_state(key, state)
        return self.transition(key)

    # ── results

    def last_run(self, key: str, sha256: str | None) -> dict | None:
        runs = [e for e in self.results(key)["events"]
                if e["kind"] in ("dryrun_ok", "real_ok", "run_failed") and e.get("sha256") == sha256]
        return runs[-1] if runs else None

    def passed(self, key: str, kind: str, sha256: str | None) -> dict | None:
        """The last ``kind`` result (dryrun_ok: run to its end in Hardware sim) of these bytes, or None."""
        hits = [e for e in self.results(key)["events"] if e["kind"] == kind and sha256 and e.get("sha256") == sha256]
        return hits[-1] if hits else None

    # ── the page's view of one arm step

    def step_view(self, key: str) -> dict:
        u = self.units[key]
        st = self.status(key)
        setup = self.setup(key)
        return dict(key=key, code=u["code"], part=u.get("part"), name=u["name"], display=display_name(u),
                    arm=u["arm"], later=not u.get("first_release", True), type=u.get("type"), kind=u.get("kind"),
                    phase=self.phase_of(key), phase_attached=phase_default(u), pole_mm=setup and setup["pole_mm"],
                    hand_preset=u.get("hand_preset"),
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
