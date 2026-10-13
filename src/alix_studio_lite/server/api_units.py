"""The Lite UI's region loop: ``/api/projects/<id>/…`` for one project's tasks and arm steps.

    GET  tasks                                the task queue (decision D35)
    GET  units/<KEY>                          one arm step: setup, recordings, tuning, versions, gates
    POST units/<KEY>/setup                    the region's pole and hand (Prepare), staged on the robot
    POST units/<KEY>/recordings               add a recording: existing (library) or a loaded .txt
    POST units/<KEY>/recordings/<FILE>/stage  state the pole and hand of a recording that has none
    POST units/<KEY>/active | name            which recording is used; the operator's file name
    GET  units/<KEY>/track                    joint data for the plot
    GET  units/<KEY>/timeline                 the execution in 3D: pole, hand, move to start, trajectory
    POST units/<KEY>/timeline                 the same from a start state {which, start} (a sequence step)
    POST sequence                             the full sequence's schedule {steps, source} (sequence.py)
    POST units/<KEY>/settings | undo | redo   tuning settings, one undo step per change
    POST units/<KEY>/optimise                 a job: the tuned candidate, with every limit checked
    POST units/<KEY>/keep                     keep the candidate as the version to upload
    POST units/<KEY>/teach/start|stop|cancel  drag-teach (the robot stand-in until robot I/O exists)
    POST units/<KEY>/move-start | playback    robot moves, in Hardware sim or Real only
    POST units/<KEY>/upload | run | verdict   Real Test

THE GATES, checked here on every execution (simulation, Hardware sim and Real), never only on
the page: the trajectory was taught on THIS arm (a LEFT trajectory never reaches a RIGHT-arm
region); its stage (pole height and hand pose) is known and is staged first, read back, before
the arm moves (``staging.py``); a run needs a kept version that passed every limit; and the
robot's own rules hold (Hardware sim or Real, a Real move ``confirmed``, every motion done in the
mode it was asked in, motion refused on both arms while either E-stop is not clear; STOP always
works).
"""
from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path

from . import fixtures, sequence, staging, workers
from .api_v2 import session
from .app import MODE_TEXT, MODES, READ_ONLY_REAL, route
from .jobs import Cancelled, UserError
from .lite_units import TIME_MODES, LiteUnits, open_units, task_list
from .project import _atomic_write
from .robot_standin import ARMS

_LOCK = threading.Lock()
_UNITS: dict[str, LiteUnits] = {}
_TUNING: dict = {}                    # (project folder, region) -> the id of its tune, one at a time
P = r"/api/projects/([A-Za-z0-9_\-]+)"
U = P + r"/units/([A-Za-z0-9_\-]+)"


def units_of(app, pid: str) -> LiteUnits:
    session(app)
    if not app.store.exists(pid):
        _UNITS.pop(pid, None)
        raise UserError("Project not found.", "Open it again from the Project Manager.")
    with _LOCK:
        u = _UNITS.get(pid)
        if u is None or not u.root.is_dir():
            u = _UNITS[pid] = open_units(app.store, pid)
        return u


def _unit(units: LiteUnits, key: str) -> dict:
    if key not in units.units:
        raise UserError(f"There is no region {key!r} in this project.")
    return units.units[key]


def _strip(rows: list[dict]) -> list[dict]:
    return [{k: v for k, v in r.items() if k != "path"} for r in rows]


def _read(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _candidate_files(units: LiteUnits, key: str) -> tuple[Path, Path, Path]:
    cand = units.candidate_path(key)
    return cand, cand.with_name(cand.stem + ".lite.json"), cand.with_name(cand.stem + ".extra.json")


def _fresh(units: LiteUnits, key: str, extra: dict | None) -> bool:
    """Whether the candidate was made from the current settings and the active recording: the
    same file, bytes, arm and stage (the same bytes loaded again with another stage are another
    recording, and its tuning is another file to keep)."""
    raw = units.current_recording(key)
    rec = next((r for r in units.recordings(key) if raw and r["file"] == raw.name), None)
    if not extra or rec is None:
        return False
    tuning = {k: v for k, v in units.settings(key).items() if k != "program_id"}
    return (extra.get("settings") == tuning and extra.get("raw_sha256") == rec.get("sha256")
            and extra.get("raw_file") == rec["file"] and extra.get("taught_arm") == rec["taught_arm"]
            and staging.same_stage(extra.get("stage"), rec["stage"]))


def identity(units: LiteUnits, key: str, which: str) -> dict:
    """The file a step would execute (``raw``: the active recording; ``candidate``; ``version``)
    with the arm it was taught on and its stage. Refuses, saying why, when the file is of the
    other arm, or its arm or stage is not known: then it is never shown moving nor run."""
    u = _unit(units, key)
    if which == "raw":
        rec = next((r for r in units.recordings(key) if r["file"] == (units.current_recording(key) or Path("")).name), None)
        if rec is None:
            raise UserError("This region has no recording yet.", "Record it, or use an existing or loaded file.")
        info = dict(path=Path(rec["path"]), name=rec["name"], taught_arm=rec["taught_arm"], stage=rec["stage"])
    elif which == "candidate":
        cand, _, extra_path = _candidate_files(units, key)
        extra = _read(extra_path) or {}
        if not cand.is_file():
            raise UserError("There is no tuned trajectory yet.", "Run the simulation to make one.")
        info = dict(path=cand, name="the tuned trajectory", taught_arm=extra.get("taught_arm"), stage=extra.get("stage"))
    elif which == "version":
        ver = units.current_version(key)
        if not ver:
            raise UserError("There is no tuned version yet.", "Finish Tune + Simulation first.")
        info = dict(path=Path(ver["path"]), name=f"version {ver['version']}", taught_arm=ver.get("taught_arm"),
                    stage=ver.get("stage"))
    else:
        raise UserError("which must be raw, candidate or version.")
    if info["taught_arm"] != u["arm"]:
        if info["taught_arm"] in ("left", "right"):
            raise UserError(f"{info['name']} was taught on the {info['taught_arm'].upper()} arm: it never runs on "
                            f"the {u['arm'].upper()} arm.", code="arm")
        raise UserError(f"It is not known which arm {info['name']} was taught on, so it cannot run.",
                        "Load it again and say the arm and the pole and hand it was taught with.", code="arm")
    st = staging.stage_from(info["stage"])
    if st is None:
        raise UserError(f"The pole height and hand pose {info['name']} was taught with are not known, so it cannot "
                        "run: its motion would not be where the robot puts it.",
                        "State them on Save & Review.", code="stage")
    info["stage"] = st
    return info


def checked_version(units, key: str, arm: str) -> dict:
    """The kept version a robot step sends or runs, checked at that moment whatever the files
    beside it say: it passed every limit when kept, its bytes are still the ones kept (sha256),
    and L1-L4 pass again on those bytes. An edited or imported project cannot slip a file past the
    gate. (L5 and L6 compare with the recording, so they are judged when the version is kept.)"""
    from ..motion import track
    from ..motion.checks import check_bytes
    from ..motion.limits import load_limits

    ver = units.current_version(key)
    if not ver:
        raise UserError("There is no tuned version yet.", "Finish Tune + Simulation (Simulation Looks Good).")
    again = "Tune the region again and keep the new version."
    if not ver.get("uploadable"):
        raise UserError(f"Version {ver['version']} did not pass every limit, so it never goes to the robot.", again)
    path = Path(ver["path"])
    if track.sha256_file(path) != ver.get("sha256"):
        raise UserError(f"Version {ver['version']} is not the file that was kept: its bytes have changed.", again)
    rep = check_bytes(track.read_md(path), load_limits(arm), with_departure=False)
    bad = [c.id for c in rep.checks if c.hard and c.passed is False and c.id in ("L1", "L2", "L3", "L4")]
    if bad:
        raise UserError(f"Version {ver['version']} breaks {', '.join(bad)} on its own bytes, so it never goes "
                        "to the robot.", again)
    return ver


def gates(app, units: LiteUnits, key: str, view: dict) -> list[dict]:
    """What stands between this step and an execution, each with a plain verdict. Once a version
    is kept, the arm and the stage are the version's: it is the file every robot step runs."""
    u = units.units[key]
    out = []
    ver = view["versions"][-1] if view["versions"] else None
    rec = next((r for r in view["recordings"] if r["file"] == view["active"]), None)
    if ver is not None:
        rec = dict(taught_arm=ver.get("taught_arm"), stage=ver.get("stage"))
    if rec is None:
        out.append(dict(id="recording", ok=False, text="No recording yet"))
    else:
        arm_ok = rec["taught_arm"] == u["arm"]
        out.append(dict(id="arm", ok=arm_ok, text=(f"Taught on the {u['arm'].upper()} arm" if arm_ok else
                                                   f"Taught on the {str(rec['taught_arm'] or '?').upper()} arm: "
                                                   f"never runs on the {u['arm'].upper()} arm")))
        st = staging.stage_from(rec["stage"])
        whose = f" ({ver['version']})" if ver else ""
        out.append(dict(id="stage", ok=st is not None,
                        text=(f"Staged first{whose}: {staging.describe(st)}" if st else
                              f"Pole and hand of {ver['version']} not known: tune and keep it again" if ver else
                              "Pole and hand not known: state them")))
    out.append(dict(id="limits", ok=bool(ver and ver.get("uploadable")),
                    text=(f"Version {ver['version']} passes every limit" if ver and ver.get("uploadable")
                          else f"Version {ver['version']} does not pass every limit: tune and keep again" if ver
                          else "No kept version yet (Tune: Simulation Looks Good)")))
    block = app.robot.motion_block()
    mode = app.ui["mode"]
    out.append(dict(id="robot", ok=mode != "visual" and not block,
                    text=(block or ("Visual sim: the 3D only" if mode == "visual" else MODE_TEXT[mode]))))
    return out


def unit_view(app, units: LiteUnits, key: str) -> dict:
    _unit(units, key)
    u = units.step_view(key)
    u["settings_view"] = units.history_view(key)
    u["recordings"] = _strip(units.recordings(key))
    raw = units.current_recording(key)
    u["active"] = raw.name if raw else None
    cand, rec, extra_path = _candidate_files(units, key)
    extra = _read(extra_path) if cand.is_file() else None
    u["candidate"] = _read(rec) if cand.is_file() else None
    u["candidate_extra"] = {k: v for k, v in (extra or {}).items() if k in ("util", "progress", "made")} or None
    u["fresh"] = _fresh(units, key, extra)
    u["versions"] = _strip(units.versions(key))
    ver = units.current_version(key)
    u["last_run"] = units.last_run(key, ver.get("sha256")) if ver else None
    u["results"] = units.results(key)["events"][-20:]
    u["toppra"] = app.toppra()
    u["gates"] = gates(app, units, key, u)
    return u


# ── tasks and one arm step

@route("GET", P + "/tasks")
def get_tasks(app, q, body, m):
    units = units_of(app, m.group(1))
    fx = next((f for f in fixtures.list_fixtures() if f["id"] == units.fixture), {})
    return {"fixture": units.fixture, "fixture_name": fx.get("name"), "has_deck": fx.get("has_deck", False),
            "hand_presets": units.catalogue.get("hand_presets"), "tasks": task_list(units)}


@route("GET", r"/api/fixtures/([a-z0-9_]+)/tasks")
def get_fixture_tasks(app, q, body, m):
    """A fixture's task queue before any project has files (the New Project panel)."""
    session(app)
    fid = m.group(1)
    if fid not in fixtures.FIXTURES:
        raise UserError(f"Unknown fixture {fid!r}.")
    units = LiteUnits(app.data_dir / "projects" / ".none", fid)
    fx = next((f for f in fixtures.list_fixtures() if f["id"] == fid), {})
    return {"fixture": fid, "fixture_name": fx.get("name"), "has_deck": fx.get("has_deck", False),
            "hand_presets": units.catalogue.get("hand_presets"), "tasks": task_list(units)}


@route("GET", U)
def get_unit(app, q, body, m):
    units = units_of(app, m.group(1))
    return unit_view(app, units, m.group(2))


@route("POST", U + "/recordings")
def add_recording(app, q, body, m):
    """``{source: "library"}``: the region's existing taught recording (D31), with the arm and the
    pole and hand its record gives.
    ``{source: "upload", name, content, arm, pole_mm, hand}``: a .txt from this computer, checked
    line by line. Its arm comes from its name (L_, R_, LEFT, RIGHT) or ``arm``, and must be this
    region's arm; ``pole_mm`` and ``hand`` are the stage it was taught with, and are required.
    Either becomes the active recording; nothing earlier is replaced."""
    import tempfile

    from ..motion import track
    from .library import detect_arm

    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    if body.get("source") == "library":
        f = units.library_file(key)
        if not f:
            raise UserError("This region has no existing recording.", "Teach it, or load a file from this computer.")
        path = units.add_recording(key, Path(f["path"]), note=f"existing recording {f['file']}: {f.get('task', '')}",
                                   source="library", name=f["file"], taught_arm=f.get("arm"),
                                   stage=staging.make_stage(f.get("pole_mm"), f.get("hand"), "the taught set's record"))
    elif body.get("source") == "upload":
        content = body.get("content")
        if not isinstance(content, str) or not content.strip():
            raise UserError("The file is empty.")
        if len(content) > 20_000_000:
            raise UserError("The file is too large for a recording (over 20 MB).")
        name = Path(str(body.get("name") or "loaded.txt")).name
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td) / name
            tmp.write_text(content, encoding="utf-8")
            try:
                md = track.read_md(tmp)
            except ValueError as exc:
                raise UserError(f"This file is not a trajectory Lite can use: {exc}",
                                "A trajectory has one line per 5 ms: {\"point\":[7 whole numbers]}.")
            if md.shape[0] < 2:
                raise UserError("This file has fewer than two lines.")
            named = detect_arm(str(body.get("path") or name))
            said = body.get("arm") or None
            if said not in (None, "left", "right"):
                raise UserError("The arm is left or right.")
            if named and said and named != said:
                raise UserError(f"{name}'s name says the {named.upper()} arm, but the {said.upper()} arm was given.",
                                code="arm")
            arm = said or named
            if arm is None:
                raise UserError(f"Which arm was {name} taught on?", "Its name does not say (L_ or R_).", code="arm")
            if body.get("pole_mm") is None or body.get("hand") is None:
                raise UserError(f"Say the pole height and hand pose {name} was taught with.", code="stage")
            stage = staging.make_stage(body["pole_mm"], body["hand"], "stated when it was loaded")
            path = units.add_recording(key, tmp, note=f"loaded file {name}", source="upload", name=name,
                                       taught_arm=arm, stage=stage)
    else:
        raise UserError("Say where the recording comes from: library or upload.")
    return {"saved": path.name, "arm": u["arm"], "unit": unit_view(app, units, key)}


@route("POST", U + r"/recordings/([A-Za-z0-9_.\-]+)/stage")
def state_recording_stage(app, q, body, m):
    """The pole and hand of a recording that has none recorded (kept before stages were)."""
    units, key, file = units_of(app, m.group(1)), m.group(2), m.group(3)
    _unit(units, key)
    stage = staging.make_stage(body.get("pole_mm"), body.get("hand"), "stated for this recording")
    try:
        units.set_recording_stage(key, file, stage, body.get("arm"))
    except KeyError:
        raise UserError("That recording is not in this region.") from None
    return unit_view(app, units, key)


@route("POST", U + "/setup")
def set_setup(app, q, body, m):
    """The region's pole and hand (Prepare): kept for teaching it, and in Hardware sim or Real
    also staged on the robot (pole, then hand, read back). ``{pole_mm, hand, apply}``."""
    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    stage = staging.make_stage(body.get("pole_mm"), body.get("hand"), "set in Prepare")
    mode, job = app.ui["mode"], None
    if body.get("apply") and mode != "visual":         # confirmed and accepted by the robot first,
        require_confirmed(app, body, "Moving the pole and the hand")     # so a refusal changes nothing
        job = app.robot.stage(u["arm"], stage, f"Stage {key}: {staging.describe(stage)}", mode=mode)
    units.set_setup(key, stage)
    out = {"unit": unit_view(app, units, key)}
    if job is not None:
        out["job"] = job.view()
    return out


@route("POST", U + "/active")
def set_active(app, q, body, m):
    units, key = units_of(app, m.group(1)), m.group(2)
    _unit(units, key)
    units.set_active(key, str(body.get("file", "")))
    return unit_view(app, units, key)


@route("POST", U + "/name")
def name_recording(app, q, body, m):
    """The operator's name for the active recording (the page's “Save TXT”)."""
    units, key = units_of(app, m.group(1)), m.group(2)
    raw = units.current_recording(key)
    if raw is None:
        raise UserError("There is no recording to save yet.", "Record the region, or load a file.")
    try:
        units.rename_recording(key, str(body.get("file") or raw.name), str(body.get("name") or ""))
    except ValueError as exc:
        raise UserError(f"{exc}.")
    return unit_view(app, units, key)


def _track_path(units: LiteUnits, key: str, q) -> Path:
    which = (q.get("which") or ["raw"])[0]
    if which == "raw":
        f = (q.get("file") or [None])[0]
        recs = units.recordings(key)
        path = next((Path(r["path"]) for r in recs if r["file"] == f), None) if f else units.current_recording(key)
    elif which == "candidate":
        path = units.candidate_path(key)
    elif which == "version":
        ver = units.current_version(key)
        path = Path(ver["path"]) if ver else None
    else:
        raise UserError("which must be raw, candidate or version.")
    if not path or not Path(path).is_file():
        raise UserError("That file does not exist yet.")
    return Path(path)


@route("GET", U + "/track")
def get_track(app, q, body, m):
    units, key = units_of(app, m.group(1)), m.group(2)
    _unit(units, key)
    return workers.track_arrays(str(_track_path(units, key, q)))


def _timeline(app, units, key: str, which: str, start=None) -> dict:
    from .routes_robot import _mesh_links, _payload

    from ..motion import track
    from ..motion.limits import load_limits

    u = _unit(units, key)
    info = identity(units, key, which)
    md = track.read_md(info["path"])
    p = _payload(app)
    return staging.execution_timeline(u["arm"], md, info["stage"],
                                      home=staging.start_values(u["arm"], dict(p["home"]), start),
                                      links=staging.staged_links(u["arm"], _mesh_links(p)),
                                      rating_deg_s=load_limits(u["arm"]).rating_deg_s)


@route("GET", U + "/timeline")
def get_timeline(app, q, body, m):
    """The execution as the 3D view shows it (``which``: raw, candidate or version): the pole to
    the stage height, the hand to the stage pose, the arm to the first line, then the trajectory,
    every link the pole moves posed by ALIX's FK. Refused for a file of the other arm or with no
    stage, exactly as a run would be."""
    units = units_of(app, m.group(1))
    return _timeline(app, units, m.group(2), (q.get("which") or ["raw"])[0])


@route("POST", U + "/timeline")
def post_timeline(app, q, body, m):
    """The same execution from a start state ({which, start: {pole_mm, hand, q_deg}}): a step of
    the full sequence, starting where the arm's previous step left it."""
    units = units_of(app, m.group(1))
    return _timeline(app, units, m.group(2), str(body.get("which") or "raw"), sequence.check_start(body.get("start")))


@route("POST", P + "/sequence")
def post_sequence(app, q, body, m):
    """The full sequence's schedule for the page's included flow rows ({steps, source}): each arm
    step's file, start state and times, with what blocks it (sequence.plan)."""
    from ..motion.limits import load_limits
    from .routes_robot import _payload

    units = units_of(app, m.group(1))
    steps = body.get("steps")
    if not isinstance(steps, list) or not steps or len(steps) > 200:
        raise UserError("Include at least one step in the sequence.")
    p = _payload(app)
    return sequence.plan(units, [s for s in steps if isinstance(s, dict)], source=str(body.get("source") or "version"),
                         home=dict(p["home"]), rating={s: load_limits(s).rating_deg_s for s in ARMS},
                         identity=identity)


# ── tuning

def _check_settings(changes: dict) -> dict:
    out = {}
    for k, v in changes.items():
        if k == "optimiser":
            if v not in ("default", "toppra"):
                raise UserError("Choose Default or Time-optimal.")
        elif k == "time_mode":
            if v not in TIME_MODES:
                raise UserError("Choose: recorded pace, a target time, or as fast as safe.")
        elif k == "target_s":
            if v is not None:
                v = round(float(v), 3)
                if not 0.05 <= v <= 600:
                    raise UserError("A target time must be between 0.05 and 600 seconds.")
        elif k == "smooth_window":
            v = int(v)
            if not 1 <= v <= 501:
                raise UserError("The smoothing window is 1 to 501 points.")
            v |= 1                                               # the prototype's windows are odd
        elif k in ("trim", "smooth", "scale"):
            v = bool(v)
        elif k in ("segments", "cuts"):
            rows = []
            for r in v or []:
                a, b = int(r["start"]), int(r["end"])
                if a < 1 or b < a:
                    raise UserError("A region needs a start row of 1 or more and an end row after it.")
                row = dict(start=a, end=b)
                if k == "segments":
                    pct = float(r["pct"])
                    if not 5 <= pct <= 400:
                        raise UserError("Scale is 5 % to 400 %.")
                    row["pct"] = pct
                rows.append(row)
            v = rows
        elif k == "program_id":
            if v is not None and not 1 <= int(v) <= 100:
                raise UserError("Program IDs are 1 to 100.")
            v = None if v is None else int(v)
        elif k == "max_dev_mm":
            v = None if v is None else float(v)
        else:
            continue
        out[k] = v
    return out


@route("POST", U + "/settings")
def post_settings(app, q, body, m):
    units, key = units_of(app, m.group(1)), m.group(2)
    _unit(units, key)
    units.update_settings(key, _check_settings(body.get("changes") or {}), str(body.get("label") or "Change settings")[:80])
    return unit_view(app, units, key)


@route("POST", U + r"/(undo|redo)")
def post_undo(app, q, body, m):
    units, key = units_of(app, m.group(1)), m.group(2)
    _unit(units, key)
    units.step_history(key, -1 if m.group(3) == "undo" else 1)
    return unit_view(app, units, key)


@route("POST", U + "/optimise")
def post_optimise(app, q, body, m):
    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    info = identity(units, key, "raw")            # this arm's recording, with a known stage
    raw = info["path"]
    settings = units.settings(key)
    tuning = {k: v for k, v in settings.items() if k != "program_id"}
    max_dev = u.get("max_dev_mm") or (units.library_file(key) or {}).get("max_dev_mm")   # else the taught set's
    if max_dev and not settings.get("max_dev_mm"):
        settings["max_dev_mm"] = max_dev
    cand, rec, extra_path = _candidate_files(units, key)
    cand.parent.mkdir(parents=True, exist_ok=True)
    src = _read(raw.with_suffix(".source.json")) or {}
    name = "Time-optimal" if settings["optimiser"] == "toppra" else "Default"

    def work(job):
        job.progress(f"Tuning with {name}", 1, 2)
        tmp = cand.with_name(f"candidate.{job.id}.txt")       # this tune's own files until it is done
        fut = workers.pool().submit(workers.optimise_unit, str(raw), str(tmp), u["arm"], settings)
        try:
            while not fut.done():
                job.sleep(0.1)
        except Cancelled:                                     # stopped: what the worker still writes is dropped
            fut.cancel()
            fut.add_done_callback(lambda _f: _discard(tmp))
            raise
        try:
            res = fut.result()
            if res.get("error"):
                raise UserError(res["error"], res.get("hint"))
            job.progress("Checked against the limits", 2, 2)
            extra = dict(util=res.get("util"), progress=res.get("progress"), settings=tuning,
                         raw_file=raw.name, raw_sha256=src.get("sha256"), made=time.strftime("%Y-%m-%d %H:%M:%S"),
                         taught_arm=info["taught_arm"], stage=info["stage"].as_dict())
            with units.lock:                                  # the candidate's three files change together,
                job.check()                                   # and never for a stopped tune
                extra_path.unlink(missing_ok=True)
                os.replace(tmp, cand)
                os.replace(tmp.with_name(tmp.stem + ".lite.json"), rec)
                _atomic_write(extra_path, json.dumps(extra))
            job.log(res["summary"]["message"])
            return dict(summary=res["summary"], checks=res["checks"], cut=res["recipe"].get("cut"))
        finally:
            _discard(tmp)

    with _LOCK:
        busy = app.jobs.get(_TUNING.get((units.root, key)) or "")
        if busy is not None and busy.state == "running":
            raise UserError(f"{key} is already being tuned.", "Wait for that tuning to finish, or stop it, then try again.",
                            code="busy")
        job = app.jobs.start("optimise", f"Tune {key}", work, unit=key, arm=u["arm"])
        _TUNING[(units.root, key)] = job.id
    return job.view()


def _discard(tmp: Path) -> None:
    """A tune's own files (``candidate.<job>.txt``, its record, a half-written temporary)."""
    for f in tmp.parent.glob(f"{tmp.stem}.*"):
        f.unlink(missing_ok=True)


@route("POST", U + "/keep")
def post_keep(app, q, body, m):
    """Keep the tuned candidate as the version to upload: only when every hard limit passes and
    it was made from the current settings and recording."""
    units, key = units_of(app, m.group(1)), m.group(2)
    _unit(units, key)
    cand, rec, extra_path = _candidate_files(units, key)
    with units.lock:                              # a tune finishing now cannot swap the files under us
        if not cand.is_file():
            raise UserError("There is no tuned trajectory yet.", "Run the simulation to make one.")
        if not _fresh(units, key, _read(extra_path)):
            raise UserError("The settings or the recording changed after the last tuning.",
                            "Re-run the simulation first.")
        info = identity(units, key, "candidate")  # the tuned file's arm and stage are known and right
        result = (_read(rec) or {}).get("result") or {}
        if not result.get("uploadable"):
            raise UserError("This trajectory does not pass every limit, so it cannot go to the robot.",
                            "Follow the hint under the results, then re-run the simulation.")
        ver = units.current_version(key)
        if (ver and ver.get("sha256") == (_read(rec) or {}).get("track_sha256")
                and ver.get("taught_arm") == info["taught_arm"] and staging.same_stage(ver.get("stage"), info["stage"])):
            return unit_view(app, units, key)                        # already kept, with this arm and stage
        units.save_candidate(key)
    return unit_view(app, units, key)


# ── robot steps (Hardware sim or Real only)

def require_robot_mode(app) -> str:
    if app.ui["mode"] == "visual":
        raise UserError("Lite is in Visual sim, so it does not use the robot.",
                        "Choose Hardware sim or Real robot in the status pill at the top.", code="mode")
    return app.ui["mode"]


def require_confirmed(app, body, what: str) -> None:
    if app.ui["mode"] == "real" and not body.get("confirmed"):
        raise UserError(f"{what} on the real robot needs your confirmation.", code="confirm")


@route("POST", U + r"/teach/(start|stop|cancel)")
def post_teach(app, q, body, m):
    units, key, verb = units_of(app, m.group(1)), m.group(2), m.group(3)
    u = _unit(units, key)
    side = u["arm"]
    if u.get("first_release") is False:
        raise UserError("This step is not in Release 1.", "It will be added later.")
    if verb == "start":
        mode = require_robot_mode(app)
        require_confirmed(app, body, "Drag teaching")
        setup = staging.stage_from(units.setup(key))
        return {"arm": app.robot.teach_start(side, setup, mode=mode)}
    if verb == "cancel":
        app.robot.arm(side).teaching = False
        return {"arm": app.robot.arm(side).view()}
    secs = app.robot.teach_stop(side)
    # stand-in: a real recording of the SAME arm, saved with the stage it was taught with
    from .library import standin_take

    take = standin_take(side, key, units.fixture)
    name = str(body.get("name") or f"{key}_taught.txt")
    path = units.add_recording(key, Path(take["path"]), source="teach", name=name, taught_arm=take["arm"],
                               stage=take["stage"],
                               note=f"stand-in recording ({secs:.0f} s of teaching): {take['name']}, "
                                    f"taught at {take['stage']['text']}")
    return {"saved": path.name, "seconds": round(secs, 1), "take": take["name"], "stage": take["stage"],
            "unit": unit_view(app, units, key)}


@route("POST", U + "/move-start")
def post_move_start(app, q, body, m):
    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    mode = require_robot_mode(app)
    require_confirmed(app, body, "Moving the arm")
    which = body.get("which") or "raw"
    info = identity(units, key, "version" if which == "version" else "raw")
    what = "the tuned trajectory's start" if which == "version" else "the recorded start"
    return app.robot.move_to(u["arm"], what, f"{u['arm'].upper()} to {what} ({key})", stage=info["stage"],
                             mode=mode).view()


@route("POST", U + "/playback")
def post_playback(app, q, body, m):
    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    mode = require_robot_mode(app)
    if mode == "real":
        raise UserError("A recording as taught never runs on the real arms.",
                        "Tune it and keep the version: Real Test runs that. In Hardware sim, Playback is a "
                        "dry run in which nothing moves.", code="mode")
    info = identity(units, key, "raw")
    lines = sum(1 for ln in info["path"].read_bytes().split(b"\n") if ln.strip())
    return app.robot.playback(u["arm"], lines, dry_run=mode != "real", title=f"Playback {key}",
                              stage=info["stage"], mode=mode).view()


def _program_check(pid) -> int:
    try:
        pid = int(pid)
    except (TypeError, ValueError):
        raise UserError("Enter a Program ID from 1 to 100.") from None
    if not 1 <= pid <= 100:
        raise UserError("Program IDs are 1 to 100.")
    return pid


@route("POST", U + "/upload")
def post_upload(app, q, body, m):
    """Upload the kept version to a Program ID (1–100, checked here, not only on the page).
    A slot holding another program needs ``replace`` (the page asks first)."""
    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    require_robot_mode(app)
    pid = _program_check(body.get("program_id"))
    info = identity(units, key, "version")         # never send another arm's trajectory to this arm
    ver = checked_version(units, key, u["arm"])    # every limit, on the very bytes sent
    if units.settings(key).get("program_id") != pid:
        units.update_settings(key, {"program_id": pid}, f"Program ID {pid}")
    cur = app.robot.arm(u["arm"]).programs.get(pid)
    if cur and cur.get("sha256") != ver["sha256"] and not body.get("replace"):
        raise UserError(f"Program {pid} on the {u['arm'].upper()} controller already holds “{cur.get('name')}”.",
                        "Confirm to replace it, or choose another Program ID.", code="replace")
    name = f"{key}_{ver['version']}"[:32]
    job = app.robot.upload(u["arm"], pid, Path(ver["path"]), name, ver["sha256"], True, f"Upload {key} to program {pid}",
                           stage=info["stage"].as_dict())
    if job is None:
        units.add_result(key, "uploaded", ver["sha256"], True, {"program_id": pid, "unchanged": True})
        return {"unchanged": True, "program_id": pid, "unit": unit_view(app, units, key)}

    def watch():
        while job.state == "running":
            time.sleep(0.2)
        if job.state == "done":
            units.add_result(key, "uploaded", ver["sha256"], True, {"program_id": pid})

    threading.Thread(target=watch, daemon=True).start()
    return job.view()


@route("POST", U + "/run")
def post_run(app, q, body, m):
    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    mode = require_robot_mode(app)
    require_confirmed(app, body, "A run")
    pid = units.settings(key).get("program_id")
    if not units.current_version(key) or not pid:
        raise UserError("Upload the trajectory first.")
    info = identity(units, key, "version")         # this arm's trajectory, with its stage
    ver = checked_version(units, key, u["arm"])
    prog = app.robot.arm(u["arm"]).programs.get(int(pid))
    if not prog or prog.get("sha256") != ver["sha256"]:
        raise UserError(f"Program {pid} does not hold this region's current version.", "Upload it again.")
    dry = mode != "real"

    def done(out):
        units.add_result(key, "dryrun_ok" if dry else "real_ok", ver["sha256"], True, out)

    job = app.robot.run(u["arm"], int(pid), dry_run=dry, title=("Hardware sim run " if dry else "Real run ") + key,
                        on_done=done, stage=info["stage"], mode=mode)    # refused if the mode changed meanwhile

    def watch():
        while job.state == "running":
            time.sleep(0.2)
        if job.state != "done":
            units.add_result(key, "run_failed", ver["sha256"], False, {"state": job.state, "error": job.error})

    threading.Thread(target=watch, daemon=True).start()
    return job.view()


@route("POST", U + "/verdict")
def post_verdict(app, q, body, m):
    """PASS approves the arm step only after a completed run of its current version; Needs
    Tuning and Record Again are recorded as they are."""
    units, key = units_of(app, m.group(1)), m.group(2)
    _unit(units, key)
    v = body.get("verdict")
    if v not in ("pass", "tune", "again"):
        raise UserError("Choose PASS, Needs Tuning or Record Again.")
    ver = units.current_version(key)
    if v == "pass":
        run = units.last_run(key, ver.get("sha256")) if ver else None
        if not run or not run.get("ok"):
            raise UserError("Run the test first: PASS needs a completed run of this version.",
                            "Upload, then Run Real Test (in Hardware sim or Real).")
    units.add_result(key, "verdict", ver["sha256"] if ver else None, v == "pass",
                     {"verdict": v, "note": str(body.get("note") or "")[:500], "mode": app.ui["mode"]})
    return unit_view(app, units, key)


# ── robots: mode, connection, pole, hand, E-stop (shared with Legacy Lite's stand-in)

def _arm(m) -> str:
    side = m.group(1)
    if side not in ARMS:
        raise UserError("Say which arm: left or right.")
    return side


@route("POST", "/api/robots/mode")
def post_mode(app, q, body, m):
    """Visual sim, Hardware sim or Real (D30). Lite starts in Visual sim."""
    session(app)
    mode = body.get("mode")
    if mode not in MODES:
        raise UserError("Unknown mode.")
    with app.robot.lock:                          # the robot starts no motion between this check and the switch
        moving = app.robot.busy()
        if moving and mode != app.ui["mode"]:
            raise UserError(f"Something is running on the robot: {moving}.",
                            "Let it finish, or press STOP, before changing the mode.", code="busy")
        if mode != "visual":
            if not any(a["connected"] for a in app.robot.view().values()):
                raise UserError("Connect at least one arm first.", code="connect")
            why = app.robot.motion_block()
            if why and mode == "real":
                raise UserError(why, "Release the E-stop and press Recover.")
            if mode == "real" and app.robot.read_only:
                raise UserError(READ_ONLY_REAL[0], READ_ONLY_REAL[1], code="mode")
            if mode == "real" and not body.get("confirmed"):
                raise UserError("Switching to the real robot needs your confirmation.", code="confirm")
        app.set_mode(mode)
    return {"mode": mode, "mode_text": MODE_TEXT[mode]}


@route("POST", r"/api/robots/(left|right)/(connect|disconnect)")
def post_connect(app, q, body, m):
    session(app)
    side = _arm(m)
    if m.group(2) == "connect":
        ip = str(body.get("ip") or "").strip() or None
        return app.robot.connect(side, ip)
    out = app.robot.disconnect(side)
    if not any(a["connected"] for a in app.robot.view().values()):
        app.set_mode("visual")
    return out


@route("POST", r"/api/robots/(left|right)/pole")
def post_pole(app, q, body, m):
    session(app)
    side = _arm(m)
    mode = require_robot_mode(app)
    require_confirmed(app, body, "Moving the pole")
    try:
        mm = float(body.get("mm"))
    except (TypeError, ValueError):
        raise UserError("Enter the pole height in mm (0 to 315).") from None
    if not 0 <= mm <= 315:
        raise UserError("The pole height is 0 to 315 mm.")
    return app.robot.move_pole(side, mm, f"Pole {side.upper()} to {mm:.0f} mm", mode=mode).view()


@route("POST", r"/api/robots/(left|right)/hand")
def post_hand(app, q, body, m):
    session(app)
    side = _arm(m)
    mode = require_robot_mode(app)
    require_confirmed(app, body, "Moving the hand")
    return app.robot.set_hand(side, body.get("counts") or [], f"Hand {side.upper()}", mode=mode).view()


@route("POST", r"/api/robots/(left|right)/move-pose")
def post_move_pose(app, q, body, m):
    """A start pose by profile name (the page's Setup & Safety and Redo)."""
    session(app)
    side = _arm(m)
    mode = require_robot_mode(app)
    require_confirmed(app, body, "Moving the arm")
    name = str(body.get("name") or "Default Start Pose")[:80]
    return app.robot.move_to(side, f"“{name}”", f"{side.upper()} to {name}", mode=mode).view()


@route("POST", r"/api/robots/(left|right)/controller-mode")
def post_controller_mode(app, q, body, m):
    """Engineer: the controller's own run mode, SIM or REAL (Advanced: SIM / REAL / CHECK)."""
    side = _arm(m)
    session(app, engineer=True)
    mode = str(body.get("mode") or "").upper()
    if mode == "CHECK":
        return app.robot.arm(side).view()
    if mode == "REAL" and not body.get("confirmed"):
        raise UserError("Switching the controller to REAL needs your confirmation.", code="confirm")
    return app.robot.set_controller_mode(side, mode)


@route("POST", r"/api/robots/(left|right)/fault")
def post_fault(app, q, body, m):
    """Engineer, stand-in only: make the pole or the hand fail to arrive, to test that nothing
    runs when staging fails. ``{pole: true|false, hand: true|false}``."""
    side = _arm(m)
    session(app, engineer=True)
    if app.robot.backend != "stand-in":
        raise UserError("Faults can only be simulated on the robot stand-in.")
    return app.robot.set_fault(side, pole=body.get("pole"), hand=body.get("hand"))


@route("POST", r"/api/robots/(left|right)/estop-test")
def post_estop_test(app, q, body, m):
    side = _arm(m)
    session(app, engineer=True)
    out = app.robot.press_estop(side, bool(body.get("pressed")))
    if out["estop"] != "CLEAR" and app.ui["mode"] == "real":
        app.set_mode("hwsim")
    return out


@route("POST", r"/api/robots/(left|right)/recover")
def post_recover(app, q, body, m):
    session(app)
    return app.robot.recover(_arm(m))


@route("GET", r"/api/robots/(left|right)/programs")
def get_programs(app, q, body, m):
    session(app)
    side = _arm(m)
    return {"arm": side, "programs": [p for p in app.robot.programs(side) if p.get("name")]}


# ── jobs and the 3D scene

@route("GET", r"/api/jobs/([A-Za-z0-9_\-]+)")
def get_job(app, q, body, m):
    session(app)
    job = app.jobs.get(m.group(1))
    if not job:
        raise UserError("That job is no longer known.")
    return job.view(int((q.get("since") or ["0"])[0]))


@route("POST", r"/api/jobs/([A-Za-z0-9_\-]+)/cancel")
def cancel_job(app, q, body, m):
    session(app)
    job = app.jobs.get(m.group(1))
    if job:
        job.cancel()
    return {"ok": True}


@route("GET", r"/api/fixtures/([a-z0-9_]+)/scene")
def get_scene(app, q, body, m):
    session(app)
    fid = m.group(1)
    if fid not in fixtures.FIXTURES:
        raise UserError(f"Unknown fixture {fid!r}.")
    cub = int((q.get("cubicle") or [app.ui.get("cubicle_mm", 1400)])[0])
    if cub not in (1400, 800):
        raise UserError("The cubicle is 1400 mm or 800 mm wide.")
    return {"fixture": fid, "cubicle_mm": cub, "spec": fixtures.spec(fid, cub)}
