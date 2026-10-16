"""Trajectory merges, one arm at a time: Merge + Full Sequence's S1, Merge & Upload (feedback v5,
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
    d = folder(units, side)
    out = []
    for p in sorted(d.glob("*.txt")) if d.is_dir() else []:
        m = _meta(p)
        if m:
            out.append(dict(m, file=p.name))
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
         phase: str | None = None, sources: list | None = None, made_from: dict | None = None) -> dict:
    """Merge ``parts`` ({name, md}) for ``side`` under ``name`` (numbered when the project has it), with
    the UGV placement and phase its parts were taught at, what it was made of (``sources``) and, for
    one kept in Tune, the merge it was tuned from (``made_from``)."""
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
                    made=time.strftime("%Y-%m-%d %H:%M:%S"))
        dst.with_suffix(".source.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    return dict(meta, file=dst.name, path=str(dst))


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
                             r.get("stage"), r.get("placement"), u.get("phase"), r.get("sha256"), r.get("taken")))
        for v in units.versions(key):
            extra_p = Path(v["path"]).with_name(Path(v["path"]).stem + ".extra.json")
            try:
                extra = json.loads(extra_p.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                extra = {}
            behind = by_file.get(extra.get("raw_file")) or {}
            out.append(_item(units, side, "ver", key, v["version"], display_name(u), v.get("name") or f"{display_name(u)} {v['version']}",
                             v.get("lines"), v.get("taught_arm"), v.get("stage"), behind.get("placement"), u.get("phase"),
                             v.get("sha256"), v.get("saved")))
    for m in listing(units, side):
        out.append(_item(units, side, "merge", None, m["file"], "Merges", m["name"], m.get("lines"), m.get("taught_arm"),
                         m.get("stage"), m.get("placement"), m.get("phase"), m.get("sha256"), m.get("made")))
    for it in out:
        it["placement_text"] = pl.words(it["placement"])
    return out


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


def with_compatibility(items: list[dict], picked: list[str]) -> list[dict]:
    """``items`` with, against the first file picked, whether each can join the merge (``compatible``)
    and why not (``differs``)."""
    first = next((it for it in items if picked and it["ref"] == picked[0]), None)
    for it in items:
        it["differs"] = [] if first is None or it is first else differences(first, it)
        it["compatible"] = not it["why"] and not it["differs"]
    return items


def from_project(units, side: str, name: str, refs: list) -> dict:
    """S1's merge: the project's files ``refs`` (from ``sources``), in that order, each of ``side`` with
    its pole and hand known, all of one stage and one placement (each measurement within ±10 mm).
    The merge keeps the first file's stage, placement and phase."""
    from ..motion import track

    if not isinstance(refs, list) or not refs or len(refs) > 50:
        raise UserError("Choose 1 to 50 files to merge.")
    items = {it["ref"]: it for it in sources(units, side)}
    chosen = []
    for r in refs:
        it = items.get(str(r))
        if it is None:
            raise UserError(f"{r} is not one of this project's {side.upper()} files.", "Refresh the list, then choose again.")
        if it["why"]:
            raise UserError(f"{it['name']} cannot be merged: {it['why']}.", code="arm" if "arm" in it["why"] else "stage")
        chosen.append(it)
    first = chosen[0]
    for it in chosen[1:]:
        why = differences(first, it)
        if why:
            raise UserError(f"{it['name']} cannot be merged with {first['name']}: {'; '.join(why)}.",
                            "Only files with the same pole, hand, UGV placement and room merge (each measurement within ±10 mm).",
                            code="mismatch")
    parts = [dict(name=it["name"], md=track.read_md(_path_of(units, side, it))) for it in chosen]
    return save(units, side, name, parts, staging.stage_from(first["stage"]), placement=first["placement"], phase=first["phase"],
                sources=[dict(ref=it["ref"], name=it["name"], sha256=it["sha256"]) for it in chosen])


def _path_of(units, side: str, it: dict) -> Path:
    from ..motion import track

    kind, rest = it["ref"].split(":", 1)
    if kind == "merge":
        return Path(checked(units, side, rest)["path"])
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
