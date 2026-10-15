"""Custom trajectory merges, one arm at a time (Merge + Full Sequence's "Custom Trajectory Merge &
Single-Arm Test"; feedback v2, slide 10: download, upload to the controller, rename).

The files are put end to end in the order given. Where one ends away from where the next begins,
a join is put between them: a smooth joint-space move (a cosine: still at both ends) at the speed
Lite moves an arm to a trajectory's start, staging.TRANSIT_FRACTION of each joint's rating, as the
robot would move between them. The merge runs as one program, so it has one pole height and hand
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


def save(units, side: str, name: str, parts: list[dict], stage: "staging.Stage") -> dict:
    """Merge ``parts`` ({name, md}) for ``side`` under ``name`` (numbered when the project has it)."""
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
        track.write_md(dst, md)
        n = int(md.shape[0])
        meta = dict(name=name, side=side, taught_arm=side, stage=stage.as_dict(), lines=n, seconds=round((n - 1) * staging.CYCLE_S, 3),
                    sha256=track.sha256_file(dst), parts=[dict(name=p["name"], lines=int(p["md"].shape[0])) for p in parts],
                    joins=joins, limits_ok=not bad, limits_text=failures_text({"checks": bad}),
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
