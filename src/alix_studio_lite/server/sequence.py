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


def plan(units, steps: list[dict], *, source: str, home: dict, rating: dict, identity) -> dict:
    """The schedule of the included rows ``steps`` (the page's flow table, in order): for each
    arm step its file, its start state and its times on one clock from the standing pose."""
    from .lite_units import task_list

    if source not in ("version", "recording"):
        raise UserError("The source is version or recording.")
    which = "version" if source == "version" else "raw"
    tasks = task_list(units)
    state = {side: None for side in ARMS}                 # where each arm was left: {pole_mm, hand, q_deg}
    rows, blocking = [], []
    T = 0.0
    for i, s in enumerate(steps):
        try:
            n = int(s.get("step") or i + 1)
        except (TypeError, ValueError):
            n = i + 1
        rule = str(s.get("rule") or "TOGETHER").upper()
        if rule not in RULES:
            rule = "TOGETHER"
        try:
            wait = min(3600.0, max(0.0, float(s.get("checkpointWait") or 0)))
        except (TypeError, ValueError):
            wait = 0.0
        task = tasks[n - 1] if 1 <= n <= len(tasks) else None
        label = str(s.get("label") or (task or {}).get("label") or f"Step {n}")[:60]
        row = dict(index=i, step=n, label=label, rule=rule, wait_s=wait, arms={}, t0=round(T, 4))
        if task is None:
            row["note"] = f"Task {n} is not in this project's task list."
            blocking.append(f"{label}: {row['note']}")
        if rule == "PROCEDURE" or task is None:
            row.update(t1=round(T, 4), note=row.get("note") or "Operator procedure: no robot motion.")
            T += wait
            rows.append(row)
            continue
        acts = {}
        for side in ARMS:
            a = _arm_step(units, task, side, which, identity)
            if a is None:
                continue
            row["arms"][side] = a
            if a.get("skip"):
                continue
            if not a["ready"]:
                blocking.append(f"{label} · {side.upper()} {a['display']}: {a['why']}")
                continue
            values = staging.start_values(side, home, state[side])
            clock = staging.execution_times(side, a.pop("_q"), a.pop("_stage"), values, rating[side])
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
    return dict(source=source, rows=rows, total_s=round(T, 4), ready=not blocking and runs, playable=runs,
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
    except (KeyError, TypeError, ValueError):
        raise UserError("A start state is {pole_mm, hand: 6 counts, q_deg: 7 joints}.") from None
    if not 0 <= pole <= staging.POLE_MAX_MM or len(hand) != 6 or len(q) != 7 or not all(np.isfinite(q)):
        raise UserError("That start state is out of range.")
    return dict(pole_mm=pole, hand=hand, q_deg=q)
