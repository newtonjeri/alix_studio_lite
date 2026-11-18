"""The lid and seat tasks' arm moves timed with Ruckig on s (the user, 18 Nov: "all arm motions for the mechanism
tasks, all lid and seat tasks except the hands and pole, should use ruckig on s").

ALIX's plans stay ALIX's: nothing in vendor/alix is changed and the link with ALIX is as it was. Lite keeps a re-timed
COPY of each plan and of each retry plan in Resources/RetimedPlans (beside the closings' contact checks), and a task
runs its copy. A copy is of one plan file: it records that file's sha256 and the settings and rules it was made with,
and it is only used while all three are what they are now. A plan that came with alix_sync has no copy yet: Lite's
start makes it (a few seconds a plan, needs the ruckig package), before the contact checks and the two-arm checks,
which are then made for the copy as for any plan file. A run of a task with no copy for its plan as it is now is
refused before anything moves (``require``): it never falls back to the planned timing on its own.

``LITE_MECHANISM_TIMING=planned`` runs ALIX's own timing instead (the lab's before 18 Nov), for a comparison or a
computer without ruckig. ``LITE_RETIMED_PLANS``: another folder (the tests).

What a copy changes, per arm move with at least MIN_POINTS waypoints, and nothing else:

- the path is the move's own: a cubic spline through its waypoints, in the order planned. Nothing is smoothed,
  removed or added, and the first and last pose are the plan's;
- its speed class is kept, three ways: the hand never goes faster along the path than the fastest the plan took it
  in this move, no joint goes faster than the move's highest joint speed as a share of its rating, and the
  acceleration stays within the move's highest as a share of the joint limits file's (never over 1.0, so a stroke
  the plan timed over the limit comes within it). How fast a lid is lifted or an approach is made is the task
  configs' choice. A move that still comes out shorter than planned is slowed uniformly to the planned time;
- the timing is ``ruckig_opt.RuckigPlan.on_path``: jerk-limited speed along the path, read as the checks read it,
  within the jerk limit and the UGV load limits;
- the move's waypoints are written every 5 ms on the new timing, so Lite's own law (``mechanism.emit_rows``) gives
  exactly these lines; a stroke's 5 ms track is written again with its sha256.

A move with fewer waypoints (the planner's single point the arm already stands on) is left as it is, and so are the
pole and hand entries. Every move then starts and ends at rest, so a run joins a copy's moves into a stream WITHOUT
taking the stops at their junctions out (mechanism.joined_moves): passing a corner of the path at speed is what read 6
to 11 times the acceleration limit in the streams of the planned timing.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import time
from pathlib import Path

import numpy as np

from .. import vendor_paths
from .jobs import UserError

METHOD = "ruckig_s"
WORDS = "Ruckig on s"
RULES = 1                           #: the rules above, by number: a copy made under other rules is made again
MIN_POINTS = 6
PAD_LINES = 40                      # a move is read with this much standing still before and after it
TIMINGS = (METHOD, "planned")


def timing() -> str:
    """How the lid and seat tasks' arm moves are timed on this Lite: ``ruckig_s`` (the copies), or ``planned``."""
    asked = (os.environ.get("LITE_MECHANISM_TIMING") or METHOD).strip().lower()
    if asked not in TIMINGS:
        raise UserError(f"LITE_MECHANISM_TIMING is {asked!r}.", f"One of: {', '.join(TIMINGS)}.")
    return asked


def store_dir() -> Path:
    return Path(os.environ.get("LITE_RETIMED_PLANS") or Path(vendor_paths.LITE_ROOT) / "Resources" / "RetimedPlans")


def settings() -> dict:
    """What the copies are made with: the joint limits file's jerk limit, and the UGV moment limit the UGV's own
    weight gives (half of what it holds: base_load.ugv_stand), as an automatic optimiser with nothing set."""
    from . import workers

    return dict(rules=RULES, jerk_rad_s3=float(workers.JERK_DEFAULT_RAD_S3), ugv_force_n=None,
                ugv_moment_nm=float(workers.ugv_moment_default()))


def _sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _plans() -> Path:
    from . import mechanism

    return mechanism.PLANS


def copy_name(source_name: str) -> str:
    from . import mechanism

    return source_name.replace(f"_{mechanism.MODE}", f"_{mechanism.MODE}.{METHOD}", 1)


def copy_of(source: Path, store: Path | None = None) -> Path:
    """Where the copy of ALIX's plan file ``source`` (a task's plan, or a retry's in rungs/) is kept."""
    source = Path(source)
    store = store_dir() if store is None else Path(store)
    return (store / "rungs" if source.parent.name == "rungs" else store) / copy_name(source.name)


def source_of(copy_path: Path, data: dict) -> Path:
    """ALIX's plan file a copy was made from, where Lite has it now."""
    name = str((data.get("retimed") or {}).get("source") or "")
    return (_plans() / "rungs" if Path(copy_path).parent.name == "rungs" else _plans()) / name


_KNOWN: dict = {}                 # (the copy and its plan as files: path, time, size; the settings) -> why_not's answer


def _state(path: Path) -> tuple:
    st = Path(path).stat()
    return str(path), st.st_mtime_ns, st.st_size


def why_not(copy_path: Path, source: Path, data: dict | None = None) -> str | None:
    """Why the copy at ``copy_path`` is not ``source``'s as that file is now, or None when it is. The answer for
    two files as they are is remembered, so a run's gates do not read them again."""
    copy_path, source = Path(copy_path), Path(source)
    if not copy_path.is_file():
        return "none has been made"
    key = None
    if source.is_file():
        key = (_state(copy_path), _state(source), tuple(sorted(settings().items(), key=lambda kv: kv[0])))
        if key in _KNOWN:
            return _KNOWN[key]
    why = _why_not(copy_path, source, data)
    if key is not None:
        if len(_KNOWN) > 64:
            _KNOWN.clear()
        _KNOWN[key] = why
    return why


def _why_not(copy_path: Path, source: Path, data: dict | None) -> str | None:
    if data is None:
        try:
            data = json.loads(copy_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return "its file cannot be read"
    made = data.get("retimed") or {}
    if made.get("method") != METHOD:
        return "its file is not a Ruckig on s copy"
    if not source.is_file():
        return f"ALIX's {source.name} it was made from is not in Lite"
    if made.get("source_sha256") != _sha(source):
        return f"it was made from another {source.name} (the plan has changed since)"
    now = settings()
    if any(made.get(k) != v for k, v in now.items()):
        return "it was made with other settings or rules"
    return None


def resolve(name: str) -> tuple[Path, str | None]:
    """(the plan file task ``name`` runs, why that is not its Ruckig on s copy): the copy when Lite holds one of
    ALIX's plan as it is now, else ALIX's own file with the reason (None when ``planned`` was asked for)."""
    from . import mechanism

    source = mechanism.PLANS / f"{name}_{mechanism.MODE}.json"
    if timing() != METHOD:
        return source, None
    mine = copy_of(source)
    why = why_not(mine, source)
    return (mine, None) if why is None else (source, why)


def read(name: str) -> tuple[Path, dict, str | None]:
    """``resolve`` with the file read: (the plan file task ``name`` runs, its contents, why that is not its copy).
    A copy is read once, for its validity and its contents."""
    from . import mechanism

    source = mechanism.PLANS / f"{name}_{mechanism.MODE}.json"
    why = None
    if timing() == METHOD:
        mine = copy_of(source)
        why = "none has been made"
        if mine.is_file():
            try:
                data = json.loads(mine.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                data, why = None, "its file cannot be read"
            if data is not None:
                why = why_not(mine, source, data)
                if why is None:
                    return mine, data, None
    return source, json.loads(source.read_text(encoding="utf-8")), why


def check_copy(copy_path: Path, data: dict) -> None:
    """A copy read by its own path (a retry's): refused when it is not its plan's as that file is now."""
    why = why_not(copy_path, source_of(copy_path, data), data)
    if why is not None:
        raise UserError(f"{Path(copy_path).name} is not used: {why}.",
                        "Make the copies again: python3 tools/retime_mechanism.py (or start Lite again).", code="timing")


def sources(task: str) -> list:
    """ALIX's plan files of ``task``: its plan, then its retry plans."""
    from . import mechanism

    stem = f"{task}_{mechanism.MODE}"
    main = mechanism.PLANS / f"{stem}.json"
    rungs = sorted(p for p in (mechanism.PLANS / "rungs").glob(f"{stem}.rung*.json")
                   if ".g" not in p.name.split(".rung", 1)[1])          # not a stroke's sidecar
    return ([main] if main.is_file() else []) + rungs


def missing(tasks=None) -> dict:
    """{task: why} for the tasks whose plan, or one of its retry plans, has no copy as it is now. Empty when the
    planned timing was asked for."""
    from . import mechanism

    if timing() != METHOD:
        return {}
    out = {}
    for task in (tasks if tasks is not None else list(mechanism.TASKS)):
        for src in sources(task):
            why = why_not(copy_of(src), src)
            if why is not None:
                out[task] = f"{src.name}: {why}"
                break
    return out


def require(tasks, what: str = "This run") -> None:
    """Before anything moves: refused when a task the run plays (``tasks``) has no Ruckig on s copy of its plan as
    it is now. It is never run on the planned timing instead without that being asked for."""
    from . import mechanism

    gone = missing([t for t in dict.fromkeys(tasks) if t in mechanism.TASKS])
    if gone:
        titles = " and ".join(mechanism.TASKS[t]["title"] for t in gone)
        raise UserError(f"Not run: {titles} has no {WORDS} timing for its plan as it is now ({'; '.join(gone.values())}). "
                        f"{what} needs it. Nothing moved.",
                        "Make it: python3 tools/retime_mechanism.py (a few seconds a plan; it needs the ruckig package), "
                        "or start Lite again and let its start make it, then run this again. To run ALIX's own timing "
                        "instead, start Lite with LITE_MECHANISM_TIMING=planned.", code="timing")


# ── making a copy ──────────────────────────────────────────────────────────────────────────────────────────────────
def track_text(rows) -> str:
    """A stroke's 5 ms track as ALIX's planner writes it: one {"point":[...]} per line, LF."""
    return "".join('{"point":[' + ",".join(str(int(v)) for v in r) + "]}\n" for r in rows)


def readings(rows_md, lim, jerk_rad_s3: float) -> dict:
    """What a move reads, standing still before and after it (20 Hz band-limited, as the checks)."""
    from ..motion import track
    from ..motion.checks import acc_deg_s2, jerk_deg_s3, speed_deg_s, ugv_load

    q = track.to_deg(np.asarray(rows_md))
    q = np.concatenate([np.repeat(q[:1], PAD_LINES, 0), q, np.repeat(q[-1:], PAD_LINES, 0)])
    dt = lim.dt_gate
    rating = np.asarray(lim.rating_deg_s, dtype=float)
    force, moment = ugv_load(q, lim)
    return dict(seconds=round((len(rows_md) - 1) * dt, 3),
                speed=round(float((speed_deg_s(q, dt) / rating).max()), 3),
                acc=round(float(acc_deg_s2(q, dt).max() / lim.acc_deg_s2), 3),
                jerk=round(float(np.radians(jerk_deg_s3(q, dt).max()) / jerk_rad_s3), 3),
                force_n=round(float(force.max()), 2), moment_nm=round(float(moment.max()), 2))


def hand_speed(rows_md, lim) -> tuple[np.ndarray, np.ndarray]:
    """(share of the path travelled (N,), the hand's speed there in mm/s (N,)) for a move's lines."""
    import taught_tracks as tt

    from ..motion import track
    from ..motion.checks import bandlimit

    q = track.to_deg(np.asarray(rows_md))
    q = bandlimit(np.concatenate([np.repeat(q[:1], PAD_LINES, 0), q, np.repeat(q[-1:], PAD_LINES, 0)]), lim.dt_gate)
    p = np.asarray(tt.pad_path_m(q, lim.side, lim.frame)) * 1000.0
    step = np.linalg.norm(np.diff(p, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(step)])
    return s / max(float(s[-1]), 1e-9), np.gradient(s, lim.dt_gate)


def fastest_against_plan(rows_before, rows_after, lim) -> float:
    """The most the re-timed move's hand speed is over the plan's at the same point of the path (1.0 = nowhere
    faster), read where the plan moves at a fifth of its own top speed or more."""
    u0, v0 = hand_speed(rows_before, lim)
    u1, v1 = hand_speed(rows_after, lim)
    u = np.linspace(0.02, 0.98, 400)
    plan, mine = np.interp(u, u0, v0), np.interp(u, u1, v1)
    moving = plan >= 0.2 * float(v0.max())
    return float((mine[moving] / plan[moving]).max()) if moving.any() else 1.0


def within(r: dict, speed: float, acc: float, lim) -> bool:
    return (r["speed"] <= speed * 1.02 and r["acc"] <= max(acc, 0.05) * (1.0 + lim.acc_tol) and r["jerk"] <= 1.0
            and (lim.ugv_force_n is None or r["force_n"] <= lim.ugv_force_n)
            and (lim.ugv_moment_nm is None or r["moment_nm"] <= lim.ugv_moment_nm))


def retime_move(entry, lim, jerk_rad_s3: float) -> dict | None:
    """One arm move on its own path, re-timed. None: the move is left as it is."""
    import local_speed as LS
    import taught_tracks as tt

    from ..motion import track
    from ..optimise import ToppraOptions, ruckig_opt

    q_deg = np.degrees(np.asarray(entry.q, dtype=float))
    keep = np.concatenate([[True], np.abs(np.diff(q_deg, axis=0)).max(axis=1) > 1e-9])
    if int(keep.sum()) < MIN_POINTS:
        return None
    before = readings(entry.rows, lim, jerk_rad_s3)
    speed, acc = max(before["speed"], 0.02), min(max(before["acc"], 0.05), 1.0)        # the move's own class
    rating = np.asarray(lim.rating_deg_s, dtype=float)
    path = LS.TaughtPath(q_deg[keep], lim.side, lim.frame)
    if len(path.s) < MIN_POINTS or not path.kept.all():             # the hand stands still somewhere: nothing to time along
        return None
    hand_top = float(np.abs(np.gradient(path.s, np.asarray(entry.t, dtype=float)[keep])).max())   # mm/s, as planned
    opts = ToppraOptions(smooth_hz=0.0, strip_pauses=False, smooth_mm=0.0, pin_ends=False, post_lp_hz=0.0, arrival_hold=0.8)
    plan = ruckig_opt.RuckigPlan.on_path(path, lim, opts, speed * rating, np.full(7, acc * lim.acc_deg_s2),
                                         speed_mm_s=hand_top)
    n = max(plan.n_floor, len(entry.rows) - 1)                      # never shorter than the plan made it
    rows = plan.make(n)
    after = readings(rows, lim, jerk_rad_s3)
    slowed = 0
    while not within(after, speed, acc, lim) and slowed < 400:      # what the loop left: slow the whole move a little
        n = n + max(1, math.ceil(0.01 * n))
        rows = plan.make(n)
        after = readings(rows, lim, jerk_rad_s3)
        slowed += 1
    rows = np.asarray(rows)
    first, last = np.asarray(entry.rows[0]), np.asarray(entry.rows[-1])
    if not (np.array_equal(rows[0], first) and np.array_equal(rows[-1], last)):
        raise RuntimeError(f"{entry.label()}: the re-timed move does not start and end on the plan's poses")
    dev = tt.deviation_mm(track.to_deg(np.asarray(entry.rows)), track.to_deg(rows), lim.side, lim.frame)
    return dict(rows=rows, before=before, after=after, speed_class=round(speed, 3), acc_class=round(acc, 3),
                hand_top_mm_s=round(hand_top, 1), own_fastest_s=round(plan.T, 3),
                fastest_against_plan=round(fastest_against_plan(entry.rows, rows, lim), 2),
                path_moved_mm=round(float(dev["max_mm"]), 2), within=within(after, speed, acc, lim),
                stretches=plan.info["stretches"], rounds=plan.info["rounds"], slowed_steps=slowed)


def waypoints_of(rows, clock_s: float, first=None, last=None) -> list:
    """A move's waypoints every ``clock_s``: the time and the positions in radians, which is all Lite reads of a
    waypoint (a copy carries no speeds or accelerations: they would be differences of these lines, three times the
    file). The last one is a tenth of a microsecond early, so Lite's law (n = ceil(T / clock) + 1 lines) counts
    exactly these lines whatever the division rounds to. ``first`` and ``last``: the plan's own end poses, written
    as the plan has them (the lines are on the millidegree grid, the plan's poses a hair off it), so one move still
    ends exactly where the next of that arm starts."""
    q = np.round(np.radians(np.asarray(rows, dtype=float) / 1000.0), 9)     # a nanoradian: 6e-5 of a millidegree
    if first is not None:
        q[0] = np.asarray(first, dtype=float)
    if last is not None:
        q[-1] = np.asarray(last, dtype=float)
    end = len(q) - 1
    return [dict(time_from_start=round(i * clock_s - (1e-7 if i == end else 0.0), 9), positions=[float(x) for x in q[i]])
            for i in range(len(q))]


def limits_for(made_with: dict) -> dict:
    from ..motion.limits import load_limits

    return {side: load_limits(side).with_auto(jerk_rad_s3=made_with["jerk_rad_s3"], ugv_force_n=made_with.get("ugv_force_n"),
                                              ugv_moment_nm=made_with.get("ugv_moment_nm")) for side in ("left", "right")}


def retime_plan(src: Path, out_dir: Path, name: str, limits: dict, made_with: dict) -> list:
    """ALIX's plan file ``src`` (task ``name``'s plan or one of its retries) re-timed into ``out_dir``: its copy and
    its strokes' tracks, the copy written last. The table of its arm moves. Raises when a move does not load back
    as the lines made, or is not within its limits."""
    from . import mechanism

    src, out_dir = Path(src), Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    plan = mechanism.Plan(name, src)
    data = copy.deepcopy(plan.data)
    jerk = float(made_with["jerk_rad_s3"])
    stages = [st for st in data["sub_trajectories"] if st.get("joint_names") and st.get("waypoints")]
    table, new_rows = [], {}
    for e in plan.entries:
        if e.kind != "arm":
            continue
        got = retime_move(e, limits[e.side], jerk)
        row = dict(plan=src.name, stage_index=e.index, stage=e.stage, side=e.side, waypoints=int(len(e.t)), group=e.group)
        if got is None:
            travel = float(np.abs(np.asarray(e.rows[-1]) - np.asarray(e.rows[0])).max()) / 1000.0
            table.append(dict(row, retimed=False, seconds_before=round(e.seconds, 3), travel_deg=round(travel, 3)))
            continue
        stages[e.i]["waypoints"] = waypoints_of(got["rows"], mechanism.CLOCK_S, first=e.q[0], last=e.q[-1])
        stages[e.i]["num_waypoints"] = len(got["rows"])
        new_rows[e.i] = got["rows"]
        table.append(dict(row, retimed=True, **{k: v for k, v in got.items() if k != "rows"}))
    by_pos = {pos: st for pos, st in enumerate(data["sub_trajectories"])}
    place = {id(st): k for k, st in enumerate(stages)}
    for g in data.get("track_groups") or []:
        rows = [r for pos in g["stage_indices"] for r in new_rows.get(place.get(id(by_pos[int(pos)]), -1), [])]
        if not rows:
            raise RuntimeError(f"{src.name}: track group {g['group']} has no re-timed move")
        text = track_text(rows)
        g["track_path"] = Path(g["track_path"]).name.replace(".g", f".{METHOD}.g", 1)
        (out_dir / g["track_path"]).write_text(text, encoding="utf-8", newline="\n")
        g.update(sha256=hashlib.sha256(text.encode("utf-8")).hexdigest(), points=len(rows),
                 duration_nominal_s=(len(rows) - 1) * mechanism.CLOCK_S, duration_plan_s=(len(rows) - 1) * mechanism.CLOCK_S,
                 first_row_millideg=[int(v) for v in rows[0]], last_row_millideg=[int(v) for v in rows[-1]],
                 proj_path=None, proj_sha256=None, sidecar_path=None)
    data["retimed"] = dict(method=METHOD, tool="alix_studio_lite/tools/retime_mechanism.py", source=src.name,
                           source_sha256=_sha(src), **made_with,
                           note="timing only: every arm move on its own waypoints' path, its speed class kept")
    out = out_dir / copy_name(src.name)
    for e in mechanism._entries(data, out_dir, out.name):           # as Lite will read it: tracks' sha256 and moves agree
        if e.kind == "arm" and e.i in new_rows and [list(map(int, r)) for r in new_rows[e.i]] != e.rows:
            raise RuntimeError(f"{out.name}: {e.label()} does not load back as the lines made")
    bad = [f"{r['stage']} ({r['side']})" for r in table if r["retimed"] and not r["within"]]
    if bad:
        raise RuntimeError(f"{out.name}: not within their limits after re-timing: {', '.join(bad)}")
    left = [f"{r['stage']} ({r['side']}, {r['travel_deg']:g} deg)" for r in table if not r["retimed"] and r["travel_deg"] > 0.05]
    if left:                                                        # a move that travels yet could not be timed: not a copy
        raise RuntimeError(f"{out.name}: moves that travel were left on the planned timing: {', '.join(left)}")
    tmp = out.with_name(out.name + ".tmp")
    tmp.write_text(json.dumps(data, separators=(",", ":")), encoding="utf-8")
    tmp.replace(out)                                                # the copy last: a stop before here leaves none
    return table


def make(task: str, store: Path | None = None, made_with: dict | None = None, rungs: bool = True) -> list:
    """Task ``task``'s plan and its retry plans re-timed now, into the store (the offline step; a few seconds). The
    table of every arm move, also kept beside the copies. An older copy of the task is removed first."""
    from . import mechanism

    store = store_dir() if store is None else Path(store)
    made_with = settings() if made_with is None else dict(made_with)
    limits = limits_for(made_with)
    stem = f"{task}_{mechanism.MODE}"
    for folder in (store, store / "rungs"):                         # the task's files as made before, whatever its retries were
        for old in (folder.glob(f"{stem}.*") if folder.is_dir() else []):
            old.unlink()
    table = []
    for src in sources(task):
        if src.parent.name == "rungs" and not rungs:
            continue
        table += retime_plan(src, copy_of(src, store).parent, task, limits, made_with)
    store.mkdir(parents=True, exist_ok=True)
    (store / f"{stem}.{METHOD}.moves.json").write_text(
        json.dumps(dict(task=task, made=time.strftime("%Y-%m-%d %H:%M:%S"), made_with=made_with, moves=table), indent=1),
        encoding="utf-8")
    mechanism.cached.cache_clear()                                  # the task now runs another file
    return table


def make_missing(say=print) -> list:
    """At Lite's start, after alix_sync and before the contact and two-arm checks: the tasks with no copy of their
    plan as it is now, each made now (a few seconds). The tasks made. Said, never raised: a task left without its
    copy is refused when it is run (``require``)."""
    from ..optimise import ruckig_opt

    todo = missing()
    if not todo:
        return []
    if not ruckig_opt.available():
        say(f"Lid and seat timing: {', '.join(todo)} have no {WORDS} copy of their plan as it is now, and it cannot be made "
            "here (the ruckig package or toppra is not installed in this Python): runs of them are refused. Make the "
            "copies on a computer that has it (python3 tools/retime_mechanism.py) and bring Resources/RetimedPlans here, "
            "or start Lite with LITE_MECHANISM_TIMING=planned to run ALIX's own timing.")
        return []
    done = []
    for task, why in todo.items():
        t0 = time.monotonic()
        try:
            table = make(task)
            moved = [r for r in table if r["retimed"]]
            say(f"Lid and seat timing: {task} timed with {WORDS} ({why}): {len(moved)} arm moves in "
                f"{time.monotonic() - t0:.1f} s")
            done.append(task)
        except Exception as exc:                                    # noqa: BLE001 - a note at the start, a refusal at the run
            say(f"Lid and seat timing: {task} NOT timed with {WORDS} ({type(exc).__name__}: {exc}): runs of it are refused")
    return done
