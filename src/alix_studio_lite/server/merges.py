"""Trajectory merges, one arm at a time: Merge + Sequence's S1, Merge & Upload (feedback v5,
slide 9; before it, feedback v2, slide 10: download, upload to the controller, rename).

S1 merges the project's own files (``sources``: every region's recordings and kept versions of the
arm, and the arm's merges), chosen in order. Only files of one pole height and hand pose AND one UGV
placement and room merge (the owner, 16 Oct: the whole placement, the walls and the commode, each
measurement within ±10 mm, ``placement.differences``): the merged program runs from one UGV position,
with one pole and hand. The merge keeps that stage, placement and the region's lid/seat phase, and
plays and is checked in 3D there.

The files are put end to end in the order given. Where one ends away from where the next begins,
a join is put between them: a smooth joint-space move (a cosine: still at both ends) at 30 % of
each joint's rating (staging.TRANSIT_FRACTION), as the robot would move between them. The merge runs as one program, so it has one pole height and hand
pose: the operator states them, for the whole merge. Its limits L1-L4 are checked on the merged
bytes and shown, never enforced, as for the manually tuned files it is made of (the owner, 15 Oct):
sending one that breaks a limit asks once.

Kept in the project where the page's folder has them (``merged/<arm>/``), each with its record:
name, parts, joins, arm, stage, lines, time, sha256 and limits. One name per file in a project.
"""
from __future__ import annotations

import json
import re
import time
from pathlib import Path

import numpy as np

from . import staging
from .jobs import UserError

JOIN_TOL_DEG = 0.05                       # parts closer than this need no join


def folder(units, side: str) -> Path:
    return Path(units.root) / "merged" / side


def _meta(path: Path) -> dict:
    try:
        return json.loads(path.with_suffix(".source.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def listing(units, side: str) -> list[dict]:
    """The arm's merges, each with the files in it that were tuned already (``tuned``: S1's Send to Tune
    says them first)."""
    d = folder(units, side)
    out = []
    for p in sorted(d.glob("*.txt")) if d.is_dir() else []:
        m = _meta(p)
        if m:
            out.append(dict(m, file=p.name))
    by = {m["file"]: m for m in out}
    for m in out:
        m["tuned"] = _tuned(units, side, m, by)
    return out


def _tuned(units, side: str, m: dict, by: dict) -> list[str]:
    """The files of merge ``m`` that were tuned already (kept versions, merges kept from Tune, at any depth and
    from any project), or the merge itself when Tune kept it."""
    if m.get("made_from"):
        return [f"{m['name']} itself (kept from Tune, tuned from {(m.get('made_from') or {}).get('merge') or 'a merge'})"]
    out = []
    for x in m.get("sources") or []:
        g = x.get("grade") or _class(units, side, str(x.get("ref") or ""), by)[1]
        if g in ("processed", "mixed"):
            out.append(str(x.get("name") or x.get("ref")))
    return out


def get(units, side: str, file: str) -> dict:
    p = folder(units, side) / Path(str(file)).name
    m = _meta(p) if p.is_file() else {}
    if not m:
        raise UserError(f"There is no {side.upper()} merge {file!r} in this project.")
    return dict(m, file=p.name, path=str(p))


JOIN_ACC_FRACTION = 0.5                   # a join's peak acceleration: half the hardware limit (L3 judges it band-limited)


def join_lines(a_deg, b_deg, rating_deg_s, acc_deg_s2: float = 600.0) -> np.ndarray:
    """The lines between two poses (excluding both ends): a cosine at most TRANSIT_FRACTION of each
    joint's rating fast and JOIN_ACC_FRACTION of the acceleration limit, whichever is slower."""
    a, b = np.asarray(a_deg, dtype=float), np.asarray(b_deg, dtype=float)
    d = np.abs(b - a)
    v = staging.TRANSIT_FRACTION * np.asarray(rating_deg_s, dtype=float)
    t_speed = float(np.max(d / v)) * np.pi / 2                       # a cosine's peak speed is pi/2 x its mean
    t_acc = np.pi * np.sqrt(float(np.max(d)) / (2 * JOIN_ACC_FRACTION * acc_deg_s2))   # its peak acceleration: (d/2)(pi/t)^2
    t = max(0.2, t_speed, t_acc)
    n = int(round(t / staging.CYCLE_S)) + 1
    s = (1 - np.cos(np.linspace(0.0, np.pi, n))) / 2
    return a + np.outer(s[1:-1], b - a)


def build(parts: list[np.ndarray], rating_deg_s, acc_deg_s2: float = 600.0) -> tuple[np.ndarray, list[dict]]:
    """The parts (millidegree lines) end to end, joined where they do not meet."""
    out, joins = [parts[0].astype(float) / 1000.0], []
    for i, md in enumerate(parts[1:], 1):
        q = md.astype(float) / 1000.0
        a, b = out[-1][-1], q[0]
        gap = float(np.max(np.abs(b - a)))
        if gap > JOIN_TOL_DEG:
            j = join_lines(a, b, rating_deg_s, acc_deg_s2)
            out.append(j)
            joins.append(dict(after=i, lines=int(j.shape[0]), seconds=round((j.shape[0] + 1) * staging.CYCLE_S, 3),
                              gap_deg=round(gap, 2)))
        out.append(q)
    return np.round(np.vstack(out) * 1000.0).astype(np.int64), joins


def save(units, side: str, name: str, parts: list[dict], stage: "staging.Stage", *, placement: dict | None = None,
         phase: str | None = None, sources: list | None = None, made_from: dict | None = None,
         tasks: list | None = None, grade: str | None = None) -> dict:
    """Merge ``parts`` ({name, md}) for ``side`` under ``name`` (numbered when the project has it), with
    the UGV placement and phase its parts were taught at, what it was made of (``sources``, each with its
    tasks and grade), for one kept in Tune the merge it was tuned from (``made_from``), and what S1's rules
    read of it (``tasks``: every task in it, ``grade``: raw or processed), kept as they were when it was made."""
    from dataclasses import asdict

    from ..motion import track
    from ..motion.checks import check_bytes, failures_text
    from ..motion.limits import load_limits

    if not parts:
        raise UserError("Choose at least one file to merge.")
    lim = load_limits(side)
    md, joins = build([p["md"] for p in parts], lim.rating_deg_s, lim.acc_deg_s2)
    rep = check_bytes(md, lim, with_departure=False)
    bad = [asdict(c) for c in rep.checks if c.hard and c.passed is False and c.id in ("L1", "L2", "L3", "L4")]
    name = Path(str(name or "").strip()).name or f"{side.upper()}_merge.txt"
    if not name.lower().endswith(".txt"):
        name += ".txt"
    with units.lock:
        if units.name_owner(name):
            name = units.free_name(name)
        d = folder(units, side)
        d.mkdir(parents=True, exist_ok=True)
        stem = re.sub(r"[^A-Za-z0-9_.\-]+", "_", Path(name).stem)[:60] or "merge"
        dst = d / f"{time.strftime('%Y%m%d_%H%M%S')}_{stem}.txt"
        n = 2
        while dst.exists():                         # two merges in one second whose names clean to the same stem
            dst = dst.with_name(f"{time.strftime('%Y%m%d_%H%M%S')}_{stem}_{n}.txt")
            n += 1
        track.write_md(dst, md)
        n = int(md.shape[0])
        meta = dict(name=name, side=side, taught_arm=side, stage=stage.as_dict(), lines=n, seconds=round((n - 1) * staging.CYCLE_S, 3),
                    sha256=track.sha256_file(dst), parts=[dict(name=p["name"], lines=int(p["md"].shape[0])) for p in parts],
                    joins=joins, limits_ok=not bad, limits_text=failures_text({"checks": bad}),
                    placement=placement, phase=phase, sources=sources or [], made_from=made_from,
                    made=time.strftime("%Y-%m-%d %H:%M:%S"), **({"tasks": list(tasks or []), "grade": grade} if grade else {}))
        dst.with_suffix(".source.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    return dict(meta, file=dst.name, path=str(dst))


def loaded_folder(units, side: str) -> Path:
    return folder(units, side) / "loaded"


MAX_LOADED_BYTES = 20 * 1024 * 1024


def load_file(units, side: str, name: str, content, pole_mm, hand, path: str | None = None) -> dict:
    """A trajectory file loaded into S1 from this computer (the owner, 16 Oct: "a browse option to load from
    the computer folder but warn the user that we do not know the pole and hand states for those tasks that
    are not taught in the lite app"): checked as a track and kept in the project to merge, with the pole and
    hand the operator says it was taught with (the guards need them); the arm is the card's, and a name or
    folder that says the other arm is refused. No UGV placement or room is known for it: it merges only with
    files that have none (``placement.differences``). The same bytes loaded again are the file kept; stated
    with another pole or hand, the latest statement is the file's (``was``: the one before)."""
    import hashlib
    import tempfile

    from ..motion import track
    from .library import detect_arm

    name = Path(str(name or "")).name.strip()[:120]
    if not name:
        raise UserError("A file name is needed.")
    if not name.lower().endswith(".txt"):
        raise UserError(f"{name} is not a .txt trajectory file.")
    if not isinstance(content, str) or not content.strip():
        raise UserError(f"{name} is empty.")
    if len(content.encode("utf-8")) > MAX_LOADED_BYTES:
        raise UserError(f"{name} is too large for a trajectory (over 20 MB).")
    for where in filter(None, (path, name)):                 # the folder and the name: either may say the arm
        said = detect_arm(where)
        if said and said != side:
            raise UserError(f"{name}: its name says the {said.upper()} arm, so it is never merged on the {side.upper()} arm.", code="arm")
    stage = staging.make_stage(pole_mm, hand, "stated when it was loaded into S1 (not taught in Lite)")
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td) / name
        tmp.write_text(content, encoding="utf-8")
        try:
            md = track.read_md(tmp)
        except ValueError as exc:
            raise UserError(f"{name} is not a trajectory Lite can use: {str(exc).replace(str(tmp), name)}",
                            "A trajectory has one line per 5 ms: {\"point\":[7 whole numbers]}.") from None
        data = tmp.read_bytes()
    sha = hashlib.sha256(data).hexdigest()
    with units.lock:
        d = loaded_folder(units, side)
        for m in sorted(d.glob("*.source.json")) if d.is_dir() else []:   # the same bytes again: the one kept
            kept = m.with_name(m.name[:-len(".source.json")] + ".txt")
            meta = _meta(kept)
            if meta.get("sha256") != sha:
                continue
            if staging.same_stage(meta.get("stage"), stage):
                return dict(meta, file=kept.name, again=True)
            # stated again with another pole or hand: the latest statement is the file's (a merge made from it
            # before keeps the stage it was made with)
            was = staging.stage_from(meta.get("stage"))
            meta.update(stage=stage.as_dict(), restated=time.strftime("%Y-%m-%d %H:%M:%S"))
            m.write_text(json.dumps(meta, indent=1), encoding="utf-8")
            return dict(meta, file=kept.name, again=True, was=staging.describe(was) if was else "pole and hand not known")
        if units.name_owner(name):
            name = units.free_name(name)
        d.mkdir(parents=True, exist_ok=True)
        stem = re.sub(r"[^A-Za-z0-9_.\-]+", "_", Path(name).stem)[:60] or "loaded"
        dst = d / f"{time.strftime('%Y%m%d_%H%M%S')}_{stem}.txt"
        n = 2
        while dst.exists():
            dst = dst.with_name(f"{time.strftime('%Y%m%d_%H%M%S')}_{stem}_{n}.txt")
            n += 1
        dst.write_bytes(data)
        lines = int(md.shape[0])
        meta = dict(name=name, side=side, taught_arm=side, stage=stage.as_dict(), lines=lines,
                    seconds=round((lines - 1) * staging.CYCLE_S, 3), sha256=sha, loaded=time.strftime("%Y-%m-%d %H:%M:%S"),
                    folder=str(Path(path).parent) if path and Path(path).parent != Path(".") else "")
        dst.with_suffix(".source.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    return dict(meta, file=dst.name)


def loaded(units, side: str) -> list[dict]:
    d = loaded_folder(units, side)
    out = []
    for m in sorted(d.glob("*.source.json")) if d.is_dir() else []:
        f = m.with_name(m.name[:-len(".source.json")] + ".txt")
        if f.is_file():
            out.append(dict(_meta(f), file=f.name, path=str(f)))
    return out


def rename(units, side: str, file: str, name: str) -> dict:
    m = get(units, side, file)
    name = Path(str(name or "").strip()).name[:120]
    if not name:
        raise UserError("A file name is needed.")
    if not name.lower().endswith(".txt"):
        name += ".txt"
    with units.lock:
        owner = units.name_owner(name, skip=("merge", f"{side}/{m['file']}"))
        if owner and name.casefold() != str(m["name"]).casefold():
            raise UserError(f"{units.taken_text(name, owner)}.", "Every file name in a project is used once: choose another name.",
                            code="name")
        p = Path(m["path"])
        meta = _meta(p)
        meta["name"] = name
        p.with_suffix(".source.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    return get(units, side, file)


def checked(units, side: str, file: str) -> dict:
    """A merge a robot step sends or runs: its bytes still the ones made, and L1-L4 read again on
    them (``limits_ok``, ``limits_text``) for the page to show; never refused for them."""
    from dataclasses import asdict

    from ..motion import track
    from ..motion.checks import check_bytes, failures_text
    from ..motion.limits import load_limits

    m = get(units, side, file)
    p = Path(m["path"])
    if track.sha256_file(p) != m.get("sha256"):
        raise UserError(f"{m['name']} is not the file that was merged: its bytes have changed.", "Merge it again.")
    rep = check_bytes(track.read_md(p), load_limits(side), with_departure=False)
    bad = [asdict(c) for c in rep.checks if c.hard and c.passed is False and c.id in ("L1", "L2", "L3", "L4")]
    return dict(m, limits_ok=not bad, limits_text=failures_text({"checks": bad}))


# ── S1: the project's files, merged by reference (feedback v5, slide 9)

def _ref(kind: str, key: str | None, which: str) -> str:
    return f"{kind}:{key}/{which}" if key else f"{kind}:{which}"


def sources(units, side: str) -> list[dict]:
    """The files S1 can merge for ``side``: each region of that arm with its recordings and kept
    versions, then the arm's merges. Each says what merging needs: the arm it was taught on, its pole
    and hand, its UGV placement and room, and why it cannot be merged at all (``why``)."""
    from . import placement as pl
    from .lite_units import display_name

    out = []
    for key, u in units.units.items():                      # the catalogue's regions only
        if u["arm"] != side or not u.get("first_release", True):
            continue
        recs = units.recordings(key)
        by_file = {r["file"]: r for r in recs}
        for r in recs:
            out.append(_item(units, side, "rec", key, r["file"], display_name(u), r["name"], r.get("lines"), r.get("taught_arm"),
                             r.get("stage"), r.get("placement"), units.phase_of(key), r.get("sha256"), r.get("taken")))
        for v in units.versions(key):
            extra_p = Path(v["path"]).with_name(Path(v["path"]).stem + ".extra.json")
            try:
                extra = json.loads(extra_p.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                extra = {}
            behind = by_file.get(extra.get("raw_file")) or {}
            out.append(_item(units, side, "ver", key, v["version"], display_name(u), v.get("name") or f"{display_name(u)} {v['version']}",
                             v.get("lines"), v.get("taught_arm"), v.get("stage"), behind.get("placement"), units.phase_of(key),
                             v.get("sha256"), v.get("saved")))
            out[-1]["auto"] = v.get("optimiser") == "toppra"            # made by Tune's Time-optimal (TOPP-RA)
    for m in listing(units, side):
        out.append(_item(units, side, "merge", None, m["file"], "Merges", m["name"], m.get("lines"), m.get("taught_arm"),
                         m.get("stage"), m.get("placement"), m.get("phase"), m.get("sha256"), m.get("made")))
    for m in loaded(units, side):                          # loaded from this computer: not taught in Lite
        out.append(_item(units, side, "file", None, m["file"], "Loaded from this computer", m["name"], m.get("lines"),
                         m.get("taught_arm"), m.get("stage"), None, None, m.get("sha256"), m.get("loaded")))
        out[-1]["auto"] = auto_optimized(m["name"])
    merges_by_file = {m["file"]: m for m in listing(units, side)}
    for it in out:
        it["placement_text"] = pl.words(it["placement"])
        tasks, it["grade"] = _class(units, side, it["ref"], merges_by_file)
        it["tasks"] = list(dict.fromkeys(tasks))
        it["grade_text"] = "Auto optimized (TOPP-RA)" if it.get("auto") else GRADE_SHORT.get(it["grade"], it["grade"])
        if it["kind"] == "merge" and not it["why"]:
            it["why"] = _broken(tasks, it["grade"])
    # each arm's files in its own panel only (the owner, 16 Oct: "A right-arm trajectory should never be available
    # in the left-arm panel"): a file taught on the other arm is not listed at all
    return [it for it in out if it["taught_arm"] not in ("left", "right") or it["taught_arm"] == side]


GRADE_WORDS = {"raw": "a recording (raw)", "processed": "an optimized file (processed)", "mixed": "a merge of raw and processed files",
               "unknown": "a merge whose files are not known"}
GRADE_SHORT = {"raw": "raw", "processed": "processed", "mixed": "raw and processed", "unknown": "not known"}


def auto_optimized(name: str) -> bool:
    """A file made by TOPP-RA (ALIX's processed files are named so, e.g. ``…_toppra30mm_rate0_95_left.txt``): it
    reads as Auto optimized and counts as processed (the owner, 16 Oct), even when loaded from this computer."""
    return "toppra" in str(name or "").lower()


def _class(units, side: str, ref: str, merges_by_file: dict, depth: int = 0) -> tuple[list, str]:
    """What a file is for S1's guards (the owner, 16 Oct): the tasks (regions) in it, a task as often as it is
    in it, and its grade: raw (a recording, a file loaded into S1), processed (a kept version, a merge kept
    from Tune), "mixed" (a merge of both) or "unknown" (a merge made before Lite kept what a merge is made
    of). A merge made since 16 Oct says both itself, as they were when it was made (its files may be another
    project's, or gone); an older one is read from its files."""
    kind, rest = ref.split("|", 1)[-1].split(":", 1)
    if kind in ("rec", "ver"):
        return [rest.partition("/")[0]], ("raw" if kind == "rec" else "processed")
    if kind == "file":                                       # loaded from this computer (its stored name keeps the name it had)
        return [], ("processed" if auto_optimized(rest) else "raw")
    if kind == "tune":                                       # a merge's file kept in Tune (its merge says the tasks)
        return [], "processed"
    m = merges_by_file.get(rest) if kind == "merge" and "|" not in ref else None
    if m is None or depth > 8:
        return [], "unknown"
    if m.get("grade"):
        return list(m.get("tasks") or []), str(m["grade"])
    if m.get("made_from"):                                   # kept from Tune: processed, with the tasks of what it was made from
        src = merges_by_file.get(str((m.get("made_from") or {}).get("file") or ""))
        tasks = _class(units, side, f"merge:{src['file']}", merges_by_file, depth + 1)[0] if src else []
        return tasks, "processed"
    if not m.get("sources"):
        return [], "unknown"
    tasks, grades = [], set()
    for x in m["sources"]:
        t, g = (list(x.get("tasks") or []), str(x["grade"])) if x.get("grade") else \
            _class(units, side, str(x.get("ref") or ""), merges_by_file, depth + 1)
        tasks += t
        grades.add(g)
    return tasks, ("unknown" if "unknown" in grades else grades.pop() if len(grades) == 1 else "mixed")


def _broken(tasks: list, grade: str) -> str | None:
    """Why a merge made before S1's rules (16 Oct) cannot go into another merge: the rules hold for what is
    in it too."""
    if grade == "unknown":
        return "what it is made of is not known (merged before 16 Oct): merge its files again"
    if grade == "mixed":
        return "it has recordings and tuned versions in it (merged before 16 Oct): all raw or all processed"
    twice = sorted({t for t in tasks if tasks.count(t) > 1})
    if twice:
        return f"it has {', '.join(twice)} more than once (merged before 16 Oct): one version per task"
    return None


def classify(units, side: str, ref: str) -> tuple[list, str]:
    """``_class`` of one file of this project, its tasks once each."""
    tasks, grade = _class(units, side, ref, {m["file"]: m for m in listing(units, side)})
    return list(dict.fromkeys(tasks)), grade


def _item(units, side, kind, key, which, region, name, lines, arm, stage, placement, phase, sha, when) -> dict:
    st = staging.stage_from(stage)
    why = (f"taught on the {arm.upper()} arm" if arm in ("left", "right") and arm != side
           else "the arm it was taught on is not known" if arm not in ("left", "right")
           else "its pole height and hand pose are not known" if st is None else None)
    n = int(lines or 0)
    return dict(ref=_ref(kind, key, which), kind=kind, key=key, region=region, name=name, lines=n,
                seconds=round(max(n - 1, 0) * staging.CYCLE_S, 3), taught_arm=arm, stage=st.as_dict() if st else None,
                stage_text=staging.describe(st) if st else "pole and hand not known", placement=placement, phase=phase,
                sha256=sha, when=when, why=why)


def differences(a: dict, b: dict) -> list[str]:
    """Why two S1 files cannot be merged (``sources`` items): another pole, hand, UGV placement or room."""
    from . import placement as pl

    out = []
    sa, sb = staging.stage_from(a.get("stage")), staging.stage_from(b.get("stage"))
    if sa is None or sb is None:
        out.append("a pole height and hand pose are not known")
    else:
        if sa.pole_mm != sb.pole_mm:
            out.append(f"pole {sa.pole_mm:g} mm vs {sb.pole_mm:g} mm")
        if sa.hand != sb.hand:
            out.append(f"hand {staging.hand_words(sa.hand)} vs {staging.hand_words(sb.hand)}")
    return out + pl.differences(a.get("placement"), b.get("placement"))


def sources_all(units, side: str, others: list | None = None) -> list[dict]:
    """The files S1 can merge: this project's (``sources``), then, for "All projects" (the owner, 16 Oct:
    "all projects taught files, so that user has access to all projects"), every other project's
    (``others``: [(project id, its name, its units)]), each named with its project. The guards stay: a file
    of a project on another commode cannot join (the commode is part of the setup a merge runs in)."""
    out = sources(units, side)
    for pid, pname, other in others or []:
        for it in sources(other, side):
            it.update(ref=f"p:{pid}|{it['ref']}", project_id=pid, project=pname)
            if other.fixture != units.fixture and not it["why"]:
                it["why"] = f"taught on another commode ({other.fixture})"
            out.append(it)
    return out


def with_compatibility(items: list[dict], picked: list[str]) -> list[dict]:
    """``items`` with, against the files picked, whether each can join the merge (``compatible``) and why not
    (``differs``): the first file's pole, hand, UGV placement and room; one version per task (the owner, 16 Oct:
    "The same task's trajectories can't be merged with each other"); all raw or all processed ("either all raw
    or all optimized"). A file picked is held against the ones picked before it."""
    from . import placement as pl

    by = {it["ref"]: it for it in items}
    chosen = [by[r] for r in picked if r in by]
    first = chosen[0] if chosen else None
    for it in items:
        if first is None or it is first:
            it["differs"] = []
        else:
            earlier = chosen[:chosen.index(it)] if it in chosen else chosen
            why = differences(first, it)
            if not why:                                      # within its tolerance of every file before it, not only the first
                for e in earlier[1:]:
                    d = pl.differences(e.get("placement"), it.get("placement"))
                    if d:
                        why += [f"{x}, against {e['name']}" for x in d]
                        break
            for t in it.get("tasks") or []:
                had = next((e for e in earlier if t in (e.get("tasks") or [])), None)
                if had:
                    why.append(f"{t} is in the merge already ({had['name']}): one version per task")
            if it.get("grade") != first.get("grade"):
                words = "an auto-optimized file (TOPP-RA, processed)" if it.get("auto") else GRADE_WORDS.get(it.get("grade"), it.get("grade"))
                why.append(f"{words}, and the merge is of "
                           f"{'recordings (raw)' if first.get('grade') == 'raw' else 'optimized files (processed)' if first.get('grade') == 'processed' else 'mixed files'}: "
                           "all raw or all processed")
            it["differs"] = why
        it["compatible"] = not it["why"] and not it["differs"]
    return items


def from_project(units, side: str, name: str, refs: list, others: list | None = None) -> dict:
    """S1's merge: the files ``refs`` (from ``sources_all``: this project's, and with ``others`` every other
    project's), in that order, each of ``side`` with its pole and hand known, all of one stage and one
    placement (each measurement within ±10 mm). The merge, kept in this project, keeps the first file's
    stage, placement and phase."""
    from ..motion import track

    if not isinstance(refs, list) or not refs or len(refs) > 50:
        raise UserError("Choose 1 to 50 files to merge.")
    items = {it["ref"]: it for it in sources_all(units, side, others)}
    owner = {pid: other for pid, _, other in others or []}
    chosen = []
    for r in refs:
        it = items.get(str(r))
        if it is None:
            raise UserError(f"{r} is not one of this project's {side.upper()} files.", "Refresh the list, then choose again.")
        if it["why"]:
            raise UserError(f"{it['name']} cannot be merged: {it['why']}.", code="arm" if "arm" in it["why"] else "stage")
        chosen.append(it)
    first = chosen[0]
    if len({it["ref"] for it in chosen}) != len(chosen):
        raise UserError("A file is in the merge twice.", "Each file once: one version per task.", code="task")
    with_compatibility(chosen, [it["ref"] for it in chosen])
    for it in chosen[1:]:
        if it["differs"]:
            raise UserError(f"{it['name']} cannot be merged with {first['name']}: {'; '.join(it['differs'])}.",
                            "One version per task, all raw or all processed, with the same pole, hand, UGV placement and room "
                            "(each measurement within ±10 mm).", code="mismatch")
    parts = [dict(name=it["name"], md=track.read_md(_path_of(owner.get(it.get("project_id"), units), side, it))) for it in chosen]
    grades = {it.get("grade") for it in chosen}
    return save(units, side, name, parts, staging.stage_from(first["stage"]), placement=first["placement"], phase=first["phase"],
                sources=[dict(ref=it["ref"], name=it["name"], sha256=it["sha256"], tasks=list(it.get("tasks") or []), grade=it.get("grade"))
                         for it in chosen],
                tasks=[t for it in chosen for t in it.get("tasks") or []], grade=grades.pop() if len(grades) == 1 else "mixed")


def _path_of(units, side: str, it: dict) -> Path:
    from ..motion import track

    kind, rest = it["ref"].split("|", 1)[-1].split(":", 1)       # another project's: "p:<id>|<its ref>"
    if kind == "merge":
        return Path(checked(units, side, rest)["path"])
    if kind == "file":
        path = loaded_folder(units, side) / Path(rest).name
        if not path.is_file():
            raise UserError(f"{it['name']} is no longer in the project.", "Load it again.")
        if it.get("sha256") and track.sha256_file(path) != it["sha256"]:
            raise UserError(f"{it['name']} is not the file that was loaded: its bytes have changed.")
        return path
    key, _, which = rest.partition("/")
    if kind == "rec":
        rec = next((r for r in units.recordings(key) if r["file"] == which), None)
        path = Path(rec["path"]) if rec else None
    else:
        ver = next((v for v in units.versions(key) if v["version"] == which), None)
        path = Path(ver["path"]) if ver else None
    if path is None or not path.is_file():
        raise UserError(f"{it['name']} is no longer in the project.", "Refresh the list, then choose again.")
    if it.get("sha256") and track.sha256_file(path) != it["sha256"]:
        raise UserError(f"{it['name']} is not the file that was kept: its bytes have changed.")
    return path
