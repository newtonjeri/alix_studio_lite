"""The full cleaning sequence (Merge + Full Sequence): which file each arm runs at each step, and
when, the way the robot would run it.

A row of the page's flow table names a task by its number (``step``, 1-based); the task's LEFT and
RIGHT region steps are the project's (``lite_units.task_list``). Each arm step runs its region's
kept version, the file the robot runs. With ``source="recording"`` it runs the active recording
as taught instead: a preview, which never approves the sequence. The gates are those of every
execution (``api_units.identity``): the file was taught on that arm and its stage is known. An arm
step that fails them blocks the sequence (``ready`` is false; the preview still plays the arm steps
that pass, the others standing still: ``playable``); a step outside Release 1 is skipped on that arm.

Each arm continues from where its previous step left it: the pole and the hand stage from there,
then the arm moves to the next file's first line (``staging.execution_times``). The rules:

    TOGETHER          both arms stage; the arm ready first waits; both trajectories start together
    LEFT RUN FIRST    both arms stage; LEFT runs; RIGHT starts when LEFT has finished
    RIGHT RUN FIRST   the same, RIGHT first
    ONE BY ONE        LEFT stages and runs, then RIGHT stages and runs: one arm moves at a time
    PROCEDURE         no robot motion (an operator procedure)

then the row's checkpoint wait. Times are on the 5 ms clock: a file of N lines runs (N - 1) x 5 ms.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from . import staging
from .jobs import UserError

RULES = ("TOGETHER", "LEFT RUN FIRST", "RIGHT RUN FIRST", "ONE BY ONE", "PROCEDURE")
ARMS = ("left", "right")


def _arm_step(units, task: dict | None, side: str, which: str, identity) -> dict | None:
    """One arm's action in a row, or None when that arm does nothing there."""
    from ..motion import track

    unit = (task or {}).get(side)
    if not unit:
        return None
    out = dict(side=side, key=unit["key"], display=unit["display"], phase=unit.get("phase"), which=which, ready=False)
    if unit.get("later"):
        out.update(skip=True, why="Not in Release 1: added later.")
        return out
    try:
        info = identity(units, unit["key"], which)
    except UserError as exc:
        out.update(why=" ".join(x for x in (str(exc), exc.hint or "") if x))
        return out
    q = track.read_md(Path(info["path"])).astype(float) / 1000.0
    out.update(ready=True, file=info["name"], stage=info["stage"].as_dict(), lines=int(q.shape[0]), _q=q,
               _stage=info["stage"])
    return out


def _entries(units, steps: list[dict], which: str) -> list[dict]:
    """The flow table's included rows as (task, label, rule, wait, which), each checked."""
    from .lite_units import task_list

    tasks = task_list(units)
    out = []
    for i, s in enumerate(steps):
        try:                                              # no step given: the row's place in the table
            n = i + 1 if s.get("step") in (None, "") else int(str(s.get("step")).strip())
        except ValueError:
            n = None
        rule = str(s.get("rule") or "TOGETHER").strip().upper()[:40]
        try:
            wait = min(3600.0, max(0.0, float(s.get("checkpointWait") or 0)))
        except (TypeError, ValueError):
            wait = 0.0
        task = tasks[n - 1] if n is not None and 1 <= n <= len(tasks) else None
        label = str(s.get("label") or (task or {}).get("label") or f"Step {n or i + 1}")[:60]
        why = (f"“{str(s.get('step'))[:20]}” is not a task number: give the task's number in the flow table."
               if n is None else f"Task {n} is not in this project's task list." if task is None
               else f"“{rule}” is not a rule: choose {', '.join(RULES[:-1])} or {RULES[-1]}." if rule not in RULES
               else None)                                 # never guessed: the row blocks, its arms stand still
        out.append(dict(step=n, task=task, label=label, rule=rule, wait=wait, which=which, why=why))
    return out


def _file_entries(units, items: list[dict], only: str | None) -> list[dict]:
    """Single files, one after another on their arm (feedback v2, slide 11): ``{key, which}`` with
    ``which`` raw, version, version:vNNN or transition."""
    from . import merges

    out = []
    for it in items[:50]:
        if it.get("merge"):                              # a custom merge of one arm
            side = str(it.get("side") or only or "")
            if side not in ARMS:
                raise UserError("Say which arm's merge: left or right.")
            if only and side != only:
                raise UserError(f"A {side.upper()} merge never runs on the {only.upper()} arm.", code="arm")
            m = merges.get(units, side, it["merge"])
            view = dict(key=None, display=m["name"], phase=None, later=False)
            out.append(dict(step=None, task={side: view}, label=m["name"], rule="ONE BY ONE", wait=0.0,
                            which=f"merge:{side}/{m['file']}", why=None))
            continue
        key = str(it.get("key") or "")
        if key not in units.units:
            raise UserError(f"There is no region {key!r} in this project.")
        side = units.units[key]["arm"]
        if only and side != only:
            raise UserError(f"{key} is a {side.upper()}-arm region: it never runs on the {only.upper()} arm.", code="arm")
        which = str(it.get("which") or "version")
        if not (which in ("raw", "version", "transition") or which.startswith("version:")):
            raise UserError("A file is raw, version, version:vNNN or transition.")
        view = units.step_view(key)
        out.append(dict(step=None, task={side: view}, label=str(it.get("label") or view["display"])[:60], rule="ONE BY ONE",
                        wait=0.0, which=which, why=None))
    return out


def plan(units, steps: list[dict], *, source: str, home: dict, rating: dict, identity,
         only: str | None = None, items: list[dict] | None = None) -> dict:
    """The schedule of the included rows ``steps`` (the page's flow table, in order): for each
    arm step its file, its start state and its times on one clock from the standing pose.
    ``only``: one arm's steps, the other arm standing (an arm's full sequence alone); ``items``:
    single files instead of the flow table's rows."""
    if source not in ("version", "recording"):
        raise UserError("The source is version or recording.")
    if only not in (None, *ARMS):
        raise UserError("The arm is left or right.")
    which = "version" if source == "version" else "raw"
    entries = _file_entries(units, items, only) if items is not None else _entries(units, steps, which)
    state = {side: None for side in ARMS}                 # where each arm was left: {pole_mm, hand, q_deg}
    rows, blocking = [], []
    T = 0.0
    pct = staging.move_pct()                              # the speed slider now: every timeline asked for keeps it
    for i, e in enumerate(entries):
        n, task, label, rule, wait, why = e["step"], e["task"], e["label"], e["rule"], e["wait"], e["why"]
        row = dict(index=i, step=n, label=label, rule=rule, wait_s=wait, arms={}, t0=round(T, 4),
                   task=(task or {}).get("label"))          # its task: the 3D shows the task's UGV placement
        if why:
            row["note"] = why
            blocking.append(f"{label}: {why}")
        if rule == "PROCEDURE" or why:
            row.update(t1=round(T, 4), note=row.get("note") or "Operator procedure: no robot motion.")
            T += wait
            rows.append(row)
            continue
        acts = {}
        for side in ARMS:
            if only and side != only:
                continue
            a = _arm_step(units, task, side, e["which"], identity)
            if a is None:
                continue
            row["arms"][side] = a
            if a.get("skip"):
                continue
            if not a["ready"]:
                blocking.append(f"{label} · {side.upper()} {a['display']}: {a['why']}")
                continue
            values = staging.start_values(side, home, state[side])
            clock = staging.execution_times(side, a.pop("_q"), a.pop("_stage"), values, rating[side], pct)
            a.update(start=state[side], end=clock["end"], t_pole=round(clock["t_pole"], 4),
                     t_hand=round(clock["t_hand"], 4), t_move=round(clock["t_move"], 4),
                     traj_t0=round(clock["traj_t0"], 4), traj_s=round(clock["traj_s"], 4))
            state[side] = clock["end"]
            acts[side] = a
        for a in row["arms"].values():                    # drop what only the server needs
            a.pop("_q", None)
            a.pop("_stage", None)
        end = _schedule(rule, acts, T)
        row["t1"] = round(end, 4)
        T = end + wait
        rows.append(row)
    runs = any(a.get("ready") for r in rows for a in r["arms"].values())
    return dict(source=source, rows=rows, total_s=round(T, 4), playable=runs, only=only, files=items is not None, move_pct=pct,
                ready=source == "version" and not blocking and runs,        # a preview never approves
                blocking=blocking)


def _schedule(rule: str, acts: dict, T: float) -> float:
    """Sets each action's stage_at, traj_at and end_at from row start T; returns the row's end."""
    def at(a, stage_at, traj_at):
        a.update(stage_at=round(stage_at, 4), traj_at=round(traj_at, 4), end_at=round(traj_at + a["traj_s"], 4))
        return traj_at + a["traj_s"]

    if not acts:
        return T
    if rule == "ONE BY ONE":
        t = T
        for side in ARMS:
            if side in acts:
                t = at(acts[side], t, t + acts[side]["traj_t0"])
        return t
    if rule in ("LEFT RUN FIRST", "RIGHT RUN FIRST") and len(acts) == 2:
        first = "left" if rule.startswith("LEFT") else "right"
        second = "right" if first == "left" else "left"
        f_end = at(acts[first], T, T + acts[first]["traj_t0"])
        s_end = at(acts[second], T, max(T + acts[second]["traj_t0"], f_end))
        return max(f_end, s_end)
    start = T + max(a["traj_t0"] for a in acts.values())  # TOGETHER, or one arm under any rule
    return max(at(a, T, start) for a in acts.values())


def check_start(start) -> dict | None:
    """A start state from the page ({pole_mm, hand[6], q_deg[7]}), checked; None = standing pose."""
    if start is None:
        return None
    try:
        pole = float(start["pole_mm"])
        hand = [None if c is None else int(c) for c in start["hand"]]
        q = [float(v) for v in start["q_deg"]]
    except (KeyError, TypeError, ValueError, OverflowError):
        raise UserError("A start state is {pole_mm, hand: 6 counts, q_deg: 7 joints}.") from None
    if (not 0 <= pole <= staging.POLE_MAX_MM or len(hand) != 6 or len(q) != 7 or not all(np.isfinite(q))
            or any(c is not None and not 0 <= c <= 1000 for c in hand)):
        raise UserError("That start state is out of range.")
    return dict(pole_mm=pole, hand=hand, q_deg=q)
