"""The S2 Programs slowed where they ended early on the controller, and the partner arm slowed with them (the user,
4 Nov: "reduce the speed of those trajectory files at those specific regions, since 5% speed reduction causes them
to pass, the same time margin should be added to the partner arm trajectory running together, to avoid collisions").

The evidence (Resources/Logs, 24 Oct to 4 Nov): 814 Program ends, 10 of them early with "it was stopped on the
controller". The six of 3-4 Nov came with no code, all on the right arm:
- Program 2, four times, at J2's closest approach to 0 (-0.6 to -2.5 deg), where J2 turns back from some
  160 deg/s;
- Program 3 at J3 crossing 0 at some 170 deg/s;
- Program 6 at J2 -53 deg with J6 at some 150 deg/s.
All six came with the controllers' feed off, when the program player runs at some 4.8 ms a line. None of 16 runs
of the same three Programs with the 20 ms feed on (some 5 % slower) ended early.

What this does:
- Each early end is placed in its file: the line nearest the joints the arm stopped at (Lite's status rows).
- A window of ``before_s`` before that line and ``after_s`` after it, in the S2 simulation's time, plays
  ``factor`` slower (motion/retime.py: the same joint path, the slow-down ramped in and out).
- Every Program the other arm runs during that window is slowed over the same window by the same factor. Both arms
  then gain the same time there, and their timing after it is the one the simulation checked.

Nothing is sent: the slowed files are written beside the project (``slowed/<stamp>/``) with a report. Upload each to
its slot with Advanced Robot Controls, stating the pole and hand the report lists, then simulate S2 again before a run.
"""
from __future__ import annotations

import datetime as dt
import json
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..motion import retime, track

LAB_TZ = dt.timezone(dt.timedelta(hours=8))          # lite.log's clock (lab time)
FACTOR = 1.05
BEFORE_S = 1.0
AFTER_S = 0.5
RAMP_S = 0.15
MATCH_DEG = 5.0                                       # the stop's joints within this of a line of the file
PLAYER_LINE_S = 0.0048                                # the program player's line with the feed off (21 Oct)

_DIAG = re.compile(r"(LEFT|RIGHT): program (\d+) ended after ([\d.]+) s of the ([\d.]+) s its points take: "
                   r"it was stopped on the controller")
_ALERT = re.compile(r"^(\d\d:\d\d:\d\d\.\d+) \[BOTH\] ALERT ERROR · S2 Real Machine")
_NAMED = re.compile(r"^(\d\d:\d\d:\d\d\.\d+) \[(LEFT|RIGHT)\] runner v14 · WORKFLOW PROGRAM \d+/\d+: slot (\d+) \((.+?)\)")
_STAMP = re.compile(r"^(\d{8}_\d{6})_")


def _epoch(day: str, hms: str) -> float:
    base = dt.datetime.strptime(day, "%Y-%m-%d").replace(tzinfo=LAB_TZ).timestamp()
    return base + int(hms[:2]) * 3600 + int(hms[3:5]) * 60 + float(hms[6:])


def _stamp_epoch(name: str) -> float | None:
    m = _STAMP.match(Path(name).name)
    if not m:
        return None
    return dt.datetime.strptime(m[1], "%Y%m%d_%H%M%S").replace(tzinfo=LAB_TZ).timestamp()


def early_ends(logs: Path, since: float | None = None) -> list[dict]:
    """Every S2 Program end Lite judged "stopped on the controller" (its own diagnosis in lite.log): the day and time,
    the arm, the slot, the controller's name of the program, how long it ran of how long its points take, and the
    joints the arm stood at (the status row nearest the end)."""
    out = []
    for d in sorted(p for p in Path(logs).glob("20??-??-??") if p.is_dir()):
        f = d / "lite.log"
        if not f.exists():
            continue
        named: dict[str, tuple[str, str]] = {}
        found = []
        for line in f.read_text(errors="replace").splitlines():
            m = _NAMED.match(line)
            if m:
                named[m[2]] = (m[3], m[4])
                continue
            m = _ALERT.match(line)
            if not m:
                continue
            for side, slot, ran, of in _DIAG.findall(line):
                t = _epoch(d.name, m[1])
                if since is not None and t < since:
                    continue
                name = named.get(side, (None, None))
                found.append(dict(day=d.name, at=m[1][:8], t=t, side=side.lower(), slot=int(slot), ran_s=float(ran),
                                  of_s=float(of), name=name[1] if name[0] == slot else None))
        if not found:
            continue
        rows = []
        st = d / "status.jsonl"
        if st.exists():
            for x in st.read_text(errors="replace").splitlines():
                try:
                    rows.append(json.loads(x))
                except ValueError:
                    pass
        for e in found:                       # the first row after Lite's alert: the arm at rest where it stopped
            after = [r for r in rows if 0 <= r.get("t", 0) - e["t"] <= 2.0 and ((r.get("arms") or {}).get(e["side"]) or {}).get("joints_deg")]
            if after:
                r = min(after, key=lambda r: r["t"])
                e["q_end"] = [float(v) for v in r["arms"][e["side"]]["joints_deg"]]
                e["errors"] = r["arms"][e["side"]].get("errors") or []
            else:
                e["q_end"] = None
        out.extend(found)
    return out


def _bare(name: str | None, slot: int) -> str:
    """A program's name without the slot it is in and the controller's ending ("2_R_P2_R04....t" -> "R_P2_R04...")."""
    s = re.sub(r"\.t(xt)?$", "", str(name or "").strip())
    return s[len(f"{slot}_"):] if s.startswith(f"{slot}_") else s


@dataclass
class Act:
    side: str
    slot: int
    file: Path | None
    file_name: str
    lines: int
    line_s: float
    traj_t0: float
    text: str
    pole: float | None
    hand: list | None
    sha256: str | None
    windows_s: list = field(default_factory=list)

    @property
    def traj_t1(self) -> float:
        return self.traj_t0 + (self.lines - 1) * self.line_s

    def line_at(self, t: float) -> float:
        return (t - self.traj_t0) / self.line_s


def _kept(project_dir: Path | None, side: str, sha: str | None) -> Path | None:
    """The file of those bytes among the controller's programs kept in the project (program_mirror's
    controller_exports/<side>/), or None."""
    if project_dir is None or not sha:
        return None
    for f in sorted((Path(project_dir) / "controller_exports" / side).glob("*.txt")):
        if track.sha256_file(f) == sha:
            return f
    return None


def program_acts(sim: dict, data_dir: Path | None = None, project_dir: Path | None = None) -> list[Act]:
    """The simulation's Programs, both arms, with their trajectory's place in S2's time and the bytes the simulation
    ran (identity.used). A file not where the simulation read it is looked for in Lite's data folder, then among the
    controller's programs kept in the project (``project_dir``: program_mirror), by its bytes."""
    used = ((sim.get("identity") or {}).get("used") or {})
    acts = []
    for side in ("left", "right"):
        for a in (sim.get("acts") or {}).get(side) or []:
            if a.get("kind") != "program" or a.get("traj_t0") is None or not a.get("lines"):
                continue
            f = Path(a["file"]) if a.get("file") else None
            if (f is None or not f.exists()) and data_dir is not None and a.get("file_name"):
                alt = Path(data_dir) / "advanced" / side / a["file_name"]
                f = alt if alt.exists() else f
            sha = ((used.get(side) or {}).get("programs") or {}).get(str(a["program"]))
            if f is None or not f.exists():
                f = _kept(project_dir, side, sha) or f
            acts.append(Act(side=side, slot=int(a["program"]), file=f, file_name=a.get("file_name") or (f.name if f else ""),
                            lines=int(a["lines"]), line_s=float(a.get("line_s") or 0.005), traj_t0=float(a["traj_t0"]),
                            text=str(a.get("text") or ""), pole=a.get("pole0"), hand=a.get("hand0"), sha256=sha))
    return acts


def _same_program(end: dict, act: Act) -> bool:
    if end["side"] != act.side or end["slot"] != act.slot:
        return False
    shown = act.text.split("//", 1)[-1].strip()
    bare = _bare(end.get("name"), act.slot)
    return bool(bare) and (shown.startswith(bare) or bare.startswith(shown) or Path(act.file_name).stem.find(bare) >= 0)


def stop_line(md, q_end) -> tuple[int, float]:
    """The file's line nearest the joints the arm stopped at, and how far (deg, the worst joint)."""
    q = np.asarray(md, dtype=float) / 1000.0
    gap = np.abs(q - np.asarray(q_end, dtype=float)[None, :]).max(axis=1)
    i = int(np.argmin(gap))
    return i, float(gap[i])


def _new_name(file_name: str, seconds: float) -> str:
    """The file's own name for the slowed copy: Lite's upload stamp dropped (an upload adds its own), the stated
    duration updated, and its version's last number raised (v1.1 -> v1.2); "_slow" added when it has no version."""
    name = _STAMP.sub("", Path(file_name).name)
    stem, ext = (name[:-4], name[-4:]) if name.lower().endswith(".txt") else (name, ".txt")
    stem = re.sub(r"(\d+(?:\.\d+)?)s(_v\d+(?:\.\d+)*)$", lambda m: f"{seconds:.1f}s{m[2]}", stem)
    m = re.search(r"_v(\d+)\.(\d+)$", stem)
    stem = stem[:m.start()] + f"_v{m[1]}.{int(m[2]) + 1}" if m else stem + "_slow"
    return stem + ext


def plan(sim: dict, ends: list[dict], *, data_dir: Path | None = None, project_dir: Path | None = None,
         factor: float = FACTOR, before_s: float = BEFORE_S, after_s: float = AFTER_S, ramp_s: float = RAMP_S) -> dict:
    """Which windows of S2's time to slow and what each Program gains. Reads the files; changes nothing."""
    acts = program_acts(sim, data_dir, project_dir)
    notes, placed = [], []
    md_of: dict[int, np.ndarray] = {}

    def md(act):
        if id(act) not in md_of:
            md_of[id(act)] = track.read_md(act.file)
        return md_of[id(act)]

    for e in ends:
        mine = [a for a in acts if _same_program(e, a)]
        tag = f"{e['day']} {e['at']} {e['side'].upper()} program {e['slot']} ({e.get('name')})"
        if not mine:
            notes.append(f"{tag}: not a Program of this simulation (another version or another sequence); left out.")
            continue
        a = mine[0]
        if a.file is None or not a.file.exists():
            notes.append(f"{tag}: its file {a.file_name} is not on this computer; left out.")
            continue
        sent = _stamp_epoch(a.file_name)
        if sent is not None and e["t"] < sent:
            notes.append(f"{tag}: before this file was sent ({a.file_name}); left out.")
            continue
        if not e.get("q_end"):
            notes.append(f"{tag}: no status row gives the joints it stopped at; left out.")
            continue
        if a.sha256 and track.sha256_file(a.file) != a.sha256:
            notes.append(f"{tag}: {a.file_name} is not the file the simulation ran (its bytes differ); left out.")
            continue
        i, gap = stop_line(md(a), e["q_end"])
        if gap > MATCH_DEG:
            notes.append(f"{tag}: the joints it stopped at are {gap:.1f} deg from every line of {a.file_name}; left out.")
            continue
        t = a.traj_t0 + i * a.line_s
        w = (max(a.traj_t0, t - before_s), min(a.traj_t1, t + after_s))
        placed.append(dict(end=tag, side=a.side, slot=a.slot, line=i, file_s=round(i * a.line_s, 3), gap_deg=round(gap, 2),
                           s2_s=round(t, 3), window_s=[round(w[0], 3), round(w[1], 3)]))
    windows = retime.merge([p["window_s"] for p in placed])
    files = []
    for a in acts:
        ws = [(max(w0, a.traj_t0), min(w1, a.traj_t1)) for w0, w1 in windows if w1 > a.traj_t0 and w0 < a.traj_t1]
        if not ws:
            continue
        if a.file is None or not a.file.exists():
            notes.append(f"{a.side.upper()} program {a.slot} runs in a slowed window, but its file {a.file_name} is "
                         "not on this computer: it is not slowed, and the arms' timing after the window shifts.")
            continue
        a.windows_s = ws
        files.append(a)
    # each window's gain, from the arm whose Program covers it most: the other arm's Program gains the same lines
    gains = []
    for w0, w1 in windows:
        cover = [(min(w1, a.traj_t1) - max(w0, a.traj_t0), a) for a in files if w1 > a.traj_t0 and w0 < a.traj_t1]
        best = max(cover, key=lambda c: c[0])[1]
        wl = [(best.line_at(w0), best.line_at(w1))]
        gains.append(retime.added_lines(len(md(best)), wl, ramp_s / best.line_s, factor))
    others = []
    for side in ("left", "right"):
        for a in (sim.get("acts") or {}).get(side) or []:
            if a.get("kind") == "program":
                continue
            for w0, w1 in windows:
                if a.get("t1", 0) > w0 and a.get("t0", 0) < w1 and a.get("kind") not in ("wait",):
                    others.append(f"{side.upper()} {a.get('text')} ({a.get('t0'):.2f}-{a.get('t1'):.2f} s) runs during "
                                  f"the window {w0:.2f}-{w1:.2f} s: not a trajectory, not slowed.")
    out = []
    for a in files:
        n = len(md(a))
        wl = [(a.line_at(w0), a.line_at(w1)) for w0, w1 in a.windows_s]
        add = 0
        for (w0, w1), g in zip(windows, gains):
            if any(x0 < w1 and x1 > w0 for x0, x1 in a.windows_s):
                full = (min(w1, a.traj_t1) - max(w0, a.traj_t0)) >= (w1 - w0) - 1e-6
                add += g if full else retime.added_lines(n, [(a.line_at(max(w0, a.traj_t0)), a.line_at(min(w1, a.traj_t1)))],
                                                       ramp_s / a.line_s, factor)
        out.append(dict(act=a, windows_lines=wl, added=add))
    return dict(placed=placed, windows=windows, gains=gains, files=out, notes=notes, others=others, factor=factor,
                before_s=before_s, after_s=after_s, ramp_s=ramp_s, md=md)


def _peak(md, windows_lines, line_s):
    r = np.abs(retime.rates(md, line_s))
    keep = np.zeros(len(r), bool)
    for a, b in windows_lines:
        keep[max(0, int(np.floor(a))):int(np.ceil(b))] = True
    return r[keep].max(axis=0) if keep.any() else np.zeros(7)


def write(p: dict, out_dir: Path) -> dict:
    """The slowed files and the report (report.json, report.md) in ``out_dir``."""
    out_dir = Path(out_dir)
    rows = []
    for f in p["files"]:
        a: Act = f["act"]
        md = p["md"](a)
        new, add = retime.slow(md, f["windows_lines"], p["ramp_s"] / a.line_s, p["factor"], added=f["added"])
        seconds = (len(new) - 1) * a.line_s
        name = _new_name(a.file_name, seconds)
        path = track.write_md(out_dir / a.side / name, new)
        new_windows = []
        pos = retime.positions(len(md), f["windows_lines"], p["ramp_s"] / a.line_s, add)
        for x0, x1 in f["windows_lines"]:
            new_windows.append((float(np.interp(x0, pos, np.arange(len(pos)))), float(np.interp(x1, pos, np.arange(len(pos))))))
        before = _peak(md, f["windows_lines"], PLAYER_LINE_S)
        after = _peak(new, new_windows, PLAYER_LINE_S)
        rows.append(dict(side=a.side, slot=a.slot, source=str(a.file), file=str(path), name=name,
                         lines=[len(md), len(new)], added_s=round(add * a.line_s, 3),
                         windows_file_s=[[round(x0 * a.line_s, 2), round(x1 * a.line_s, 2)] for x0, x1 in f["windows_lines"]],
                         windows_s2_s=[[round(w0, 2), round(w1, 2)] for w0, w1 in a.windows_s],
                         peak_deg_s_at_4_8ms=[np.round(before).astype(int).tolist(), np.round(after).astype(int).tolist()],
                         path_gap_md=round(retime.path_gap_md(md, new), 2),
                         same_ends=bool((new[0] == md[0]).all() and (new[-1] == md[-1]).all()),
                         pole_mm=a.pole, hand=a.hand, sha256=track.sha256_file(path)))
    rep = dict(made=dt.datetime.now(LAB_TZ).strftime("%Y-%m-%d %H:%M:%S"), factor=p["factor"], before_s=p["before_s"],
               after_s=p["after_s"], ramp_s=p["ramp_s"], early_ends=p["placed"],
               windows_s2_s=[[round(a, 2), round(b, 2)] for a, b in p["windows"]],
               files=rows, notes=p["notes"], not_slowed=p["others"])
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "report.json").write_text(json.dumps(rep, indent=1))
    (out_dir / "report.md").write_text(report_md(rep))
    return rep


def report_md(rep: dict) -> str:
    pct = round((rep["factor"] - 1) * 100)
    lines = [f"# S2 Programs slowed {pct} % where they ended early ({rep['made']})", "",
             f"Each early end's window ({rep['before_s']:g} s before the line the arm stopped at to {rep['after_s']:g} s "
             f"after it) plays {pct} % slower, ramped in and out over {rep['ramp_s']:g} s. Every Program the other arm "
             "runs in the same window is slowed there by the same amount, so both arms gain the same time.", "",
             "## The early ends placed", ""]
    for e in rep["early_ends"]:
        lines.append(f"- {e['end']}: line {e['line']} ({e['file_s']} s into its file, {e['gap_deg']} deg from where "
                     f"the arm stopped), S2 {e['window_s'][0]}-{e['window_s'][1]} s")
    lines += ["", "## Upload these files", "",
              "Advanced Robot Controls, the arm, Upload Only to the slot, Replace, with the pole and hand below. "
              "Then simulate S2 again before a run.", "",
              "| Arm | Slot | File | Pole | Hand | Lines | Added | Slowed (s into the file) | Peak deg/s at 4.8 ms a line, before -> after |",
              "|---|---|---|---|---|---|---|---|---|"]
    for r in rep["files"]:
        b, a = r["peak_deg_s_at_4_8ms"]
        lines.append(f"| {r['side'].upper()} | {r['slot']} | {r['name']} | {r['pole_mm']} mm | {r['hand']} | "
                     f"{r['lines'][0]} -> {r['lines'][1]} | {r['added_s']} s | "
                     f"{', '.join(f'{x0}-{x1}' for x0, x1 in r['windows_file_s'])} | {max(b)} -> {max(a)} |")
    if rep["notes"] or rep["not_slowed"]:
        lines += ["", "## Notes", ""] + [f"- {n}" for n in rep["notes"] + rep["not_slowed"]]
    return "\n".join(lines) + "\n"
