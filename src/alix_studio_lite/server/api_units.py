"""The Lite UI's region loop: ``/api/projects/<id>/…`` for one project's tasks and arm steps.

    GET  tasks                                the task queue (decision D35)
    GET  units/<KEY>                          one arm step: setup, recordings, tuning, versions, gates
    POST units/<KEY>/setup                    the region's pole and hand (Prepare), staged on the robot
    GET  placement | POST placement/room|task the room, the commode in it and each task's UGV placement
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
    GET|POST units/<KEY>/transition           the step's transition trajectory (V32 Prepare), kept in the project
    POST units/<KEY>/transition/upload | run  sent to a Program ID and run on this arm alone
    GET  units/<KEY>/end                      the last line of the step's file (a start point: Previous Task End)

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

from . import fixtures, merges, placement, sequence, staging, workers
from .api_v2 import session
from .app import MODE_TEXT, MODES, route
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
    if which.startswith("merge:"):                     # a custom merge of one arm (merges.py)
        side, _, file = which[6:].partition("/")
        mm = merges.get(units, side, file)
        st = staging.stage_from(mm.get("stage"))
        if st is None:
            raise UserError(f"The pole height and hand pose of {mm['name']} are not known.", "Merge it again.", code="stage")
        return dict(path=Path(mm["path"]), name=mm["name"], taught_arm=side, stage=st)
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
    elif which == "version" or which.startswith("version:"):            # the one it runs, or a named one
        ver = _version_of(units, key, which)
        info = dict(path=Path(ver["path"]), name=f"version {ver['version']}", taught_arm=ver.get("taught_arm"),
                    stage=ver.get("stage"))
    elif which == "transition":
        tv = units.transition(key)
        if not tv:
            raise UserError("This step has no transition trajectory yet.",
                            "Choose its .txt in Prepare: Task Transition & Start Point Tools.")
        info = dict(path=Path(tv["path"]), name=tv["name"], taught_arm=tv.get("taught_arm"), stage=tv.get("stage"))
    else:
        raise UserError("which must be raw, candidate, version or transition.")
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


def _version_of(units, key: str, which: str = "version") -> dict:
    """``version``: the kept version the region runs; ``version:v002``: that one."""
    want = which.split(":", 1)[1] if ":" in which else None
    ver = next((v for v in units.versions(key) if v["version"] == want), None) if want else units.current_version(key)
    if not ver:
        raise UserError(f"There is no version {want} of {key}." if want else "There is no tuned version yet.",
                        "Finish Tune + Simulation first.")
    return ver


def checked_version(units, key: str, arm: str, which: str = "version") -> dict:
    """The kept version a robot step sends or runs, checked at that moment whatever the files
    beside it say: its bytes are still the ones kept (sha256), so an edited or imported project
    cannot slip another file past. A time-optimal version must also have passed every limit when
    kept and pass L1-L4 again on those bytes; a manual one is held to no limit (the owner, 15 Oct:
    "the lite manual optimizer never enforces any limits, it is up to the hardware"): what it
    breaks is shown, and sending it asks once (``require_limits_seen``)."""
    from ..motion import track
    from ..motion.checks import check_bytes
    from ..motion.limits import load_limits

    ver = _version_of(units, key, which)
    again = "Tune the region again and keep the new version."
    path = Path(ver["path"])
    if track.sha256_file(path) != ver.get("sha256"):
        raise UserError(f"Version {ver['version']} is not the file that was kept: its bytes have changed.", again)
    if not ver.get("enforced"):
        return ver
    if not ver.get("uploadable"):
        raise UserError(f"Time-optimal version {ver['version']} did not pass every limit, so it never goes to the robot.", again)
    rep = check_bytes(track.read_md(path), load_limits(arm), with_departure=False)
    bad = [c.id for c in rep.checks if c.hard and c.passed is False and c.id in ("L1", "L2", "L3", "L4")]
    if bad:
        raise UserError(f"Time-optimal version {ver['version']} breaks {', '.join(bad)} on its own bytes, so it never goes "
                        "to the robot.", again)
    return ver


def require_limits_seen(body: dict, what: str, limits_ok, limits_text: str) -> None:
    """A manual file that breaks a limit goes to the robot once the operator has seen what it
    breaks: the page asks, then sends ``limits_seen`` (the owner, 15 Oct: shown, not enforced)."""
    if limits_ok is False and not body.get("limits_seen"):
        raise UserError(f"{what} breaks {limits_text or 'a limit'}. Lite does not hold manual tuning to the limits: "
                        "the robot's own protections decide.", "Send it anyway, or tune it again.", code="limits")


def gates(app, units: LiteUnits, key: str, view: dict) -> list[dict]:
    """What stands between this step and an execution, each with a plain verdict. Once a version
    is kept, the arm and the stage are the version's: it is the file every robot step runs."""
    u = units.units[key]
    out = []
    ver = next((v for v in view["versions"] if v["version"] == view.get("version")), view["versions"][-1] if view["versions"] else None)
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
    if ver is None:
        out.append(dict(id="limits", ok=False, text="No kept version yet (Tune: Simulation Looks Good)"))
    elif ver.get("enforced"):                          # time-optimal: every limit, or it never runs
        out.append(dict(id="limits", ok=bool(ver.get("uploadable")),
                        text=(f"Version {ver['version']} passes every limit" if ver.get("uploadable")
                              else f"Version {ver['version']} does not pass every limit: tune and keep again")))
    else:                                              # manual: shown, never enforced
        out.append(dict(id="limits", ok=True, warn=not ver.get("limits_ok", True),
                        text=(f"Version {ver['version']} passes every limit" if ver.get("limits_ok", True)
                              else f"Version {ver['version']} breaks {ver.get('limits_text') or 'a limit'} "
                                   "(shown, not enforced: the hardware decides)")))
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
    u["version"] = (units.current_version(key) or {}).get("version")          # the one it runs
    ver = units.current_version(key)
    u["last_run"] = units.last_run(key, ver.get("sha256")) if ver else None
    u["results"] = units.results(key)["events"][-20:]
    u["toppra"] = app.toppra()
    u["gates"] = gates(app, units, key, u)
    u["applied"] = units.applied(key)
    return u


# ── the flow table's files (feedback v2, slide 8): each task's LEFT and RIGHT file, its versions, the upload

def _flow_arm(app, units: LiteUnits, step: dict | None) -> dict | None:
    if not step:
        return None
    key = step["key"]
    cur = units.current_version(key)
    held = app.robot.arm(units.units[key]["arm"]).programs
    def uploaded(sha):
        hit = [(pid, p) for pid, p in held.items() if sha and p.get("sha256") == sha]
        return [dict(program_id=pid, name=p.get("stored_name") or p.get("name"), at=p.get("uploaded")) for pid, p in sorted(hit)]
    vs = [dict(version=v["version"], name=v.get("name") or f"{key}_{v['version']}.txt", achieved_s=v.get("achieved_s"),
               lines=v.get("lines"), uploadable=bool(v.get("uploadable")), uploaded=uploaded(v.get("sha256")), sha256=v.get("sha256"),
               limits_ok=v.get("limits_ok"), limits_text=v.get("limits_text") or "", optimiser=v.get("optimiser"))
          for v in units.versions(key)]
    rec = units.current_recording(key)
    tv = units.transition(key)
    return dict(key=key, display=step["display"], later=step["later"], versions=vs, version=cur and cur["version"],
                recording=_name_of(units, key, rec) if rec else None,
                transition=dict(name=tv["name"], sha256=tv.get("sha256"), limits_ok=tv.get("limits_ok")) if tv else None)


@route("GET", P + "/flow-files")
def get_flow_files(app, q, body, m):
    """Each task's LEFT and RIGHT file for the flow table: the kept versions (name, time, limits), the
    one each region runs, and the controller programs that hold it."""
    units = units_of(app, m.group(1))
    return {"tasks": [dict(label=t["label"], left=_flow_arm(app, units, t["left"]), right=_flow_arm(app, units, t["right"]))
                      for t in task_list(units)]}


@route("POST", U + "/version")
def post_version(app, q, body, m):
    """The kept version a region runs (Real Test, the full sequence): ``{version}``."""
    units, key = units_of(app, m.group(1)), m.group(2)
    _unit(units, key)
    units.choose_version(key, str(body.get("version") or ""))
    return unit_view(app, units, key)


# ── the room, the commode in it, and each task's UGV placement (V32 feedback v2, slide 1)

def _placement_view(units: LiteUnits, task: str | None) -> dict:
    d = placement.load(units)
    lo, hi = placement.standoff_range_mm()
    return {"room": d["room"], "tasks": d["tasks"], "task": task, "standoff_range_mm": [lo, hi],
            "scene": scene_of(units, None, None, task)}


@route("GET", P + "/placement")
def get_placement(app, q, body, m):
    """``task``: the scene for that task's UGV placement (the room alone without it)."""
    return _placement_view(units_of(app, m.group(1)), (q.get("task") or [None])[0])


@route("POST", P + "/placement/room")
def post_placement_room(app, q, body, m):
    """The room: ``{width_mm, length_mm, left_mm, right_mm, front_mm}``, all required."""
    units = units_of(app, m.group(1))
    placement.save_room(units, body)
    return _placement_view(units, body.get("task"))


@route("POST", P + "/placement/task")
def post_placement_task(app, q, body, m):
    """A task's UGV placement: ``{task, standoff_mm (required), front_mm, rear_mm, left_mm, right_mm}``."""
    units = units_of(app, m.group(1))
    placement.save_task(units, body.get("task"), body)
    return _placement_view(units, body.get("task"))


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
                                   placement=placement.for_task(units, body.get("task")),
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
                                       taught_arm=arm, stage=stage, placement=placement.for_task(units, body.get("task")))
    else:
        raise UserError("Say where the recording comes from: library or upload.")
    return {"saved": path.name, "name": _name_of(units, key, path), "arm": u["arm"], "unit": unit_view(app, units, key)}


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


@route("POST", U + r"/recordings/([A-Za-z0-9_.\-]+)/hand")
def change_recording_hand(app, q, body, m):
    """The hand a recording runs with, changed in Prepare after teaching, on the operator's word: every
    run then stages that hand (its tuning is made again before it is kept). ``{hand, confirmed}``."""
    units, key, file = units_of(app, m.group(1)), m.group(2), m.group(3)
    _unit(units, key)
    if not body.get("confirmed"):
        raise UserError("Changing the hand of a recording needs your confirmation.", code="confirm")
    try:
        units.set_recording_hand(key, file, body.get("hand"))
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
    kind = body.get("kind")                            # which Apply was pressed: pole | hand (it alone moves)
    mode, job = app.ui["mode"], None
    mark = lambda _out: units.mark_applied(key, kind, stage)     # noqa: E731  applied once the arm is in it (read back),
    if body.get("apply") and mode != "visual":         # before its job reports done; confirmed and accepted by
        if kind == "pole":                             # the robot first, so a refusal changes nothing
            require_confirmed(app, body, "Moving the pole")
            job = app.robot.stage_part(u["arm"], stage, "pole", f"Pole {key}: {stage.pole_mm:g} mm", mode=mode, on_done=mark)
        elif kind == "hand":
            require_confirmed(app, body, "Moving the hand")
            job = app.robot.stage_part(u["arm"], stage, "hand", f"Hand {key}: {staging.hand_words(stage.hand)}", mode=mode,
                                       on_done=mark)
        else:
            require_confirmed(app, body, "Moving the pole and the hand")
            job = app.robot.stage(u["arm"], stage, f"Stage {key}: {staging.describe(stage)}", mode=mode, on_done=mark)
    units.set_setup(key, stage)
    if job is None:
        units.mark_applied(key, kind, stage)
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
    """A file's joints for a plot: degrees, or ``units=counts`` the file's own values (the trajectory
    HTML plots those: "Joint Value" against "Sample / Row Number")."""
    units, key = units_of(app, m.group(1)), m.group(2)
    _unit(units, key)
    return workers.track_arrays(str(_track_path(units, key, q)), counts=(q.get("units") or [""])[0] == "counts")


def _recording_behind(units: LiteUnits, key: str, which: str) -> dict | None:
    """The recording a step's file was made from (itself for ``raw``)."""
    recs = units.recordings(key)
    if which == "raw":
        cur = units.current_recording(key)
        return next((r for r in recs if cur and r["file"] == cur.name), None)
    if which == "candidate":
        extra = _read(_candidate_files(units, key)[2]) or {}
    elif which == "version" or which.startswith("version:"):
        try:
            ver = _version_of(units, key, which)
        except UserError:
            return None
        extra = _read(Path(ver["path"]).with_name(Path(ver["path"]).stem + ".extra.json")) or {}
    else:
        return None
    return next((r for r in recs if r["file"] == extra.get("raw_file")), None)


def scene_of(units: LiteUnits, key: str | None, which: str | None, task: str | None) -> dict:
    """The scene a file plays in: the project's room, and the UGV placement its recording was taught
    at, else the task's as entered now (a recording kept before placements were has none)."""
    rec = _recording_behind(units, key, which) if key and which else None
    kept = (rec or {}).get("placement")
    pl = kept or placement.for_task(units, task)
    out = placement.scene(units.fixture, placement.load(units)["room"], pl)
    out["from"] = "the recording" if kept else "the task" if pl else None
    return out


def _timeline(app, units, key: str, which: str, start=None, max_frames: int = 5000, task: str | None = None) -> dict:
    from .routes_robot import _mesh_links, _payload

    from ..motion import track
    from ..motion.limits import load_limits

    u = _unit(units, key)
    info = identity(units, key, which)
    md = track.read_md(info["path"])
    p = _payload(app)
    out = staging.execution_timeline(u["arm"], md, info["stage"],
                                     home=staging.start_values(u["arm"], dict(p["home"]), start),
                                     links=staging.staged_links(u["arm"], _mesh_links(p)),
                                     rating_deg_s=load_limits(u["arm"]).rating_deg_s, max_frames=max_frames)
    out["scene"] = scene_of(units, key, which, task)
    return out


@route("GET", U + "/timeline")
def get_timeline(app, q, body, m):
    """The execution as the 3D view shows it (``which``: raw, candidate or version): the pole to
    the stage height, the hand to the stage pose, the arm to the first line, then the trajectory,
    every link the pole moves posed by ALIX's FK. Refused for a file of the other arm or with no
    stage, exactly as a run would be. ``coarse=1``: at most 400 frames, for a picture of where the
    arm starts and the path its hand takes (the arm cards), a tenth of the size."""
    units = units_of(app, m.group(1))
    coarse = (q.get("coarse") or ["0"])[0] == "1"
    return _timeline(app, units, m.group(2), (q.get("which") or ["raw"])[0], max_frames=400 if coarse else 5000,
                     task=(q.get("task") or [None])[0])


@route("POST", U + "/timeline")
def post_timeline(app, q, body, m):
    """The same execution from a start state ({which, start: {pole_mm, hand, q_deg}}): a step of
    the full sequence, starting where the arm's previous step left it."""
    units = units_of(app, m.group(1))
    return _timeline(app, units, m.group(2), str(body.get("which") or "raw"), sequence.check_start(body.get("start")),
                     task=body.get("task"))


@route("POST", P + "/sequence")
def post_sequence(app, q, body, m):
    """The full sequence's schedule for the page's included flow rows ({steps, source}): each arm
    step's file, start state and times, with what blocks it (sequence.plan)."""
    from ..motion.limits import load_limits
    from .routes_robot import _payload

    units = units_of(app, m.group(1))
    items = body.get("items")
    steps = body.get("steps")
    if items is not None:                                # single files (feedback v2, slide 11)
        if not isinstance(items, list) or not items:
            raise UserError("Choose at least one file.")
        steps = []
    elif not isinstance(steps, list) or not steps or len(steps) > 200:
        raise UserError("Include at least one step in the sequence.")
    p = _payload(app)
    out = sequence.plan(units, [s for s in steps if isinstance(s, dict)], source=str(body.get("source") or "version"),
                        home=dict(p["home"]), rating={s: load_limits(s).rating_deg_s for s in ARMS},
                        identity=identity, only=body.get("only") or None,
                        items=None if items is None else [x for x in items if isinstance(x, dict)])
    for row in out["rows"]:                              # a merge's 3D frames come from its own route
        for a in row["arms"].values():
            if str(a.get("which") or "").startswith("merge:") and a.get("ready"):
                side, _, file = a["which"][6:].partition("/")
                a["timeline"] = dict(url=f"/api/projects/{m.group(1)}/merges/{side}/{file}/timeline", body=dict(start=a.get("start")))
    return out


def _points_path(app, side: str, points: list, speed: float):
    """An arm's path through global points of its controller, as the 5 ms lines a 3D view plays:
    from the first point, joint by joint to each next one at ``speed`` % of each joint's rating, with
    a smooth start and stop. (On the robot a MoveL goes in a straight line; the 3D shows it joint by
    joint.)"""
    import numpy as np

    from ..motion.limits import load_limits

    names = [str((x.get("name") if isinstance(x, dict) else x) or "") for x in (points or [])][:100]
    if not names:
        raise UserError("Choose at least one global point.")
    pts = app.robot.global_points(side)
    if pts is None:
        raise UserError(f"Global points are stored on the {side.upper()} arm's controller: the stand-in has none.",
                        "Start Lite with --robot realman and connect the arm.", code="connect")
    by = {p["name"]: p for p in pts}
    missing = [n for n in names if n not in by]
    if missing:
        raise UserError(f"The {side.upper()} controller has no global point {', '.join(missing)}.", "Refresh the global points.")
    v = np.asarray(load_limits(side).rating_deg_s, dtype=float) * max(1.0, min(100.0, float(speed))) / 100.0
    q = [np.asarray(by[n]["joints"][:7], dtype=float) for n in names]
    lines = [q[0]]
    for a, b in zip(q, q[1:]):
        t = max(0.2, float(np.max(np.abs(b - a) / v)) * np.pi / 2)        # a cosine's peak speed is pi/2 x its mean
        n = int(round(t / 0.005)) + 1
        s = (1 - np.cos(np.linspace(0.0, np.pi, n))) / 2
        lines.extend(a + np.outer(s[1:], b - a))
    if len(lines) < 2:
        lines.append(q[0])                                # one point: the arm goes there and stays
    return names, np.round(np.asarray(lines) * 1000.0).astype(np.int64)


def _points_stage(app, side: str):
    from .routes_robot import _payload

    pole0, hand0 = staging.home_stage(side, dict(_payload(app)["home"]))
    return staging.make_stage(pole0, hand0, "as the arm stands: global points move the arm only")


@route("POST", P + "/points-plan")
def post_points_plan(app, q, body, m):
    """Global points in the 3D (feedback v2, slide 11): ``{side, points: [names], task}``, at the speed
    slider's %, as a one-step plan (sequence.js plays it like a sequence) whose 3D frames come from
    points-timeline."""
    from ..motion.limits import load_limits
    from .routes_robot import _payload

    units_of(app, m.group(1))
    side = str(body.get("side") or "")
    if side not in ARMS:
        raise UserError("Say which arm: left or right.")
    speed = float(app.robot.speed_pct)                # the speed slider: the moves to the points go at it
    names, md = _points_path(app, side, body.get("points"), speed)
    st = _points_stage(app, side)
    clock = staging.execution_times(side, md.astype(float) / 1000.0, st, staging.start_values(side, dict(_payload(app)["home"])),
                                    load_limits(side).rating_deg_s)
    act = dict(side=side, key=None, display=f"Global points: {' → '.join(names)}", phase=None, which="points", ready=True,
               file=f"{len(names)} global point{'s' if len(names) != 1 else ''} at {speed:g} %", stage=st.as_dict(),
               lines=int(md.shape[0]), start=None, end=clock["end"], t_pole=round(clock["t_pole"], 4),
               t_hand=round(clock["t_hand"], 4), t_move=round(clock["t_move"], 4), traj_t0=round(clock["traj_t0"], 4),
               traj_s=round(clock["traj_s"], 4), stage_at=0.0, traj_at=round(clock["traj_t0"], 4),
               end_at=round(clock["traj_t0"] + clock["traj_s"], 4),
               timeline=dict(url=f"/api/projects/{m.group(1)}/points-timeline",
                             body=dict(side=side, points=names, speed=speed, task=body.get("task"))))
    row = dict(index=0, step=None, label=f"{side.upper()} global points", rule="ONE BY ONE", wait_s=0.0, arms={side: act},
               t0=0.0, t1=act["end_at"], note="Shown joint by joint; on the robot a MoveL moves the tool in a straight line.")
    return dict(source="points", rows=[row], total_s=act["end_at"], playable=True, ready=False, only=side, files=True, blocking=[])


@route("POST", P + "/points-timeline")
def post_points_timeline(app, q, body, m):
    from ..motion.limits import load_limits
    from .routes_robot import _mesh_links, _payload

    units = units_of(app, m.group(1))
    side = str(body.get("side") or "")
    if side not in ARMS:
        raise UserError("Say which arm: left or right.")
    _, md = _points_path(app, side, body.get("points"), float(app.robot.speed_pct))
    p = _payload(app)
    out = staging.execution_timeline(side, md, _points_stage(app, side), home=dict(p["home"]),
                                     links=staging.staged_links(side, _mesh_links(p)), rating_deg_s=load_limits(side).rating_deg_s)
    out["scene"] = scene_of(units, None, None, body.get("task"))
    return out


# ── tuning

def _check_settings(changes: dict) -> dict:
    """A region's tuning settings as the trajectory HTML takes them (its windows odd and at least 3,
    a threshold of 0 or more, a minimum idle run of 1 or more, a scale of 1 % or more, regions of two
    rows or more that do not overlap), and Time-optimal's time."""
    from ..optimise.manual import validate_odd

    out = {}
    for k, v in changes.items():
        if k == "optimiser":
            if v == "default":                                   # the earlier name of manual tuning
                v = "manual"
            if v not in ("manual", "toppra"):
                raise UserError("Choose manual tuning or Time-optimal (TOPP-RA).")
        elif k == "time_mode":
            if v not in TIME_MODES:
                raise UserError("Choose: as fast as the limits allow, or a target time.")
        elif k == "target_s":
            if v is not None:
                v = round(float(v), 3)
                if not 0.05 <= v <= 600:
                    raise UserError("A target time must be between 0.05 and 600 seconds.")
        elif k in ("smooth_window", "idle_window"):
            v = validate_odd(v, 101 if k == "smooth_window" else 11)
            if v > 20001:
                raise UserError("A window is at most 20,001 rows.")
        elif k in ("trim", "smooth", "scale"):
            v = bool(v)
        elif k == "idle_threshold_md":
            v = round(float(v), 3)
            if not 0 <= v <= 1e6:
                raise UserError("The idle threshold is 0 counts per sample or more.")
        elif k == "idle_min_run":
            v = int(v)
            if not 1 <= v <= 10_000_000:
                raise UserError("The minimum idle run is 1 or more.")
        elif k == "segments":
            rows = []
            for r in v or []:
                a, b = int(r["start"]), int(r["end"])
                if a < 1 or b <= a:                              # as the HTML: two different rows at least
                    raise UserError(f"Rows {a}–{b} are not a region: it needs a start row of 1 or more and an end "
                                    "row after it.")
                pct = float(r["pct"])
                if not 1 <= pct <= 10000:
                    raise UserError("A scale is 1 % or more (100 % = unchanged).")
                rows.append(dict(start=a, end=b, pct=pct))
            rows.sort(key=lambda r: r["start"])
            for x, y in zip(rows, rows[1:]):
                if y["start"] <= x["end"]:
                    raise UserError(f"Rows {y['start']}–{y['end']} overlap rows {x['start']}–{x['end']}: regions may not overlap.",
                                    "Delete or adjust the existing region first.")
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
    name = "manual tuning" if workers.is_manual(settings) else "Time-optimal (TOPP-RA)"

    def work(job):
        job.progress(f"Tuning with {name}", 1, 2)
        tmp = cand.with_name(f"candidate.{job.id}.txt")       # this tune's own files until it is done
        fut = workers.pool().submit(workers.optimise_unit, str(raw), str(tmp), u["arm"], settings, info["name"])
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
            return dict(summary=res["summary"], checks=res["checks"])
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
    """Keep the tuned candidate as the version to upload, when it was made from the current
    settings and recording. A manual tune is kept whatever its limits say (they are shown, never
    enforced); a time-optimal one only when every hard limit passes."""
    from ..motion.checks import failures_text

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
        made = _read(rec) or {}
        result = made.get("result") or {}
        if not result.get("uploadable"):        # say which limits, and the optimiser's own hints
            failed = failures_text(made.get("checks"))
            raise UserError("This trajectory does not pass every limit, so it cannot go to the robot"
                            + (f": {failed}." if failed else "."),
                            " ".join((result.get("hints") or [])[:2])
                            or "See the limits under the results, then change the tuning and re-run the simulation.")
        ver = units.current_version(key)
        name = str(body.get("name") or "").strip()                    # V32's "Save Tuned File As…"
        if (ver and ver.get("sha256") == made.get("track_sha256")
                and ver.get("taught_arm") == info["taught_arm"] and staging.same_stage(ver.get("stage"), info["stage"])):
            if name and name != ver.get("name"):
                units.name_version(key, ver["version"], name)        # already kept: it gets the name
            return unit_view(app, units, key)                        # already kept, with this arm and stage
        if name:
            name = name if name.lower().endswith(".txt") else name + ".txt"
            owner = units.name_owner(name)
            if owner and body.get("numbered"):                        # the suggested name (the HTML's): numbered
                name = units.free_name(name)
            elif owner:                                              # a typed one is refused before anything is kept
                raise UserError(f"{units.taken_text(name, owner)}.", "Every file name in a project is used once: "
                                "choose another name.", code="name")
        ver = units.save_candidate(key)
        if name:
            units.name_version(key, ver["version"], name)
    return unit_view(app, units, key)


@route("GET", U + "/analysis")
def get_analysis(app, q, body, m):
    """Tune's automatic optimisation (its Analyze step): the trajectory HTML's review regions of the
    active recording, each with a suggested time % (optimise/analysis.py). ``target_s``: the
    suggestions fitted so the generated file, after the region's Trim, comes to that time."""
    from ..motion import track
    from ..optimise import manual
    from ..optimise.analysis import analyse, fit_target

    units, key = units_of(app, m.group(1)), m.group(2)
    _unit(units, key)
    raw = units.current_recording(key)
    if raw is None:
        raise UserError("There is no recording to analyse yet.", "Record the region, or load a file.")
    md = track.read_md(raw)
    out = analyse(md)
    target = (q.get("target_s") or [None])[0]
    if target not in (None, ""):
        try:
            target = float(target)
        except ValueError:
            raise UserError("Enter the target time in seconds, above 0.") from None
        if not target > 0:
            raise UserError("Enter the target time in seconds, above 0.")
        counts, kept_total = manual.trimmed_counts(md, units.settings(key), out["regions"])
        out = fit_target(out, target, counts=counts, kept_total=kept_total)
    out["file"] = _name_of(units, key, raw)
    return out


@route("GET", U + "/file")
def get_file(app, q, body, m):
    """A step's file as it is on disk (``which``: raw | candidate | version), to save on this
    computer: the processor's "Download processed TXT"."""
    from .app import RawResponse

    units, key = units_of(app, m.group(1)), m.group(2)
    _unit(units, key)
    which = (q.get("which") or ["candidate"])[0]
    if which == "raw":
        path = units.current_recording(key)
        name = _name_of(units, key, path) if path else None
    elif which == "candidate":                                  # the HTML's name: <recording>_trim_mean_scale_processed.txt
        path = units.candidate_path(key)
        made = _read(path.with_name(path.stem + ".lite.json")) or {}
        name = ((made.get("recipe") or {}).get("report") or {}).get("outputName") or f"{key}_tuned.txt"
    elif which == "version":
        ver = units.current_version(key)
        path = Path(ver["path"]) if ver else None
        name = ver and (ver.get("name") or f"{key}_{ver['version']}.txt")
    else:
        raise UserError("Say which file: raw, candidate or version.")
    if path is None or not path.is_file():
        raise UserError("That file is not there yet.", "Record, tune or keep it first.")
    return RawResponse(path.read_bytes(), "text/plain; charset=utf-8", name)


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
        return {"arm": app.robot.teach_cancel(side)}
    out = app.robot.teach_stop(side)
    secs = out["seconds"]
    name = str(body.get("name") or f"{key}_taught.txt")
    if out["file"] is not None:                     # the controller's own recording (the SDK wrote it)
        st = out["stage"].as_dict()
        path = units.add_recording(key, Path(out["file"]), source="teach", name=name, taught_arm=side, stage=st,
                                   placement=placement.for_task(units, body.get("task")),
                                   note=f"drag-taught on the {side.upper()} arm: {out['points']} points in "
                                        f"{secs:.0f} s, at {st['text']}")
        return {"saved": path.name, "name": _name_of(units, key, path), "seconds": round(secs, 1), "take": None,
                "stage": st, "unit": unit_view(app, units, key)}
    # stand-in: a real recording of the SAME arm, saved with the stage it was taught with
    from .library import standin_take

    take = standin_take(side, key, units.fixture)
    path = units.add_recording(key, Path(take["path"]), source="teach", name=name, taught_arm=take["arm"],
                               placement=placement.for_task(units, body.get("task")),
                               stage=take["stage"],
                               note=f"stand-in recording ({secs:.0f} s of teaching): {take['name']}, "
                                    f"taught at {take['stage']['text']}")
    return {"saved": path.name, "name": _name_of(units, key, path), "seconds": round(secs, 1), "take": take["name"],
            "stage": take["stage"], "unit": unit_view(app, units, key)}


def _name_of(units: LiteUnits, key: str, path: Path) -> str:
    """A kept recording's name: the one asked for, or numbered when the project already had it."""
    return next((r["name"] for r in units.recordings(key) if r["file"] == path.name), path.name)


@route("POST", U + "/move-start")
def post_move_start(app, q, body, m):
    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    mode = require_robot_mode(app)
    require_confirmed(app, body, "Moving the arm")
    which = body.get("which") or "raw"
    info = identity(units, key, "version" if which == "version" else "raw")
    what = "the tuned trajectory's start" if which == "version" else "the recorded start"
    first = json.loads(Path(info["path"]).read_text(encoding="utf-8").split("\n", 1)[0])["point"]
    return app.robot.move_to(u["arm"], what, f"{u['arm'].upper()} to {what} ({key})", stage=info["stage"],
                             mode=mode, target_millideg=first).view()


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
    ver = checked_version(units, key, u["arm"])    # the very bytes kept (and a time-optimal one's limits)
    require_limits_seen(body, ver.get("name") or f"Version {ver['version']}", ver.get("limits_ok"), ver.get("limits_text"))
    if units.settings(key).get("program_id") != pid:
        units.update_settings(key, {"program_id": pid}, f"Program ID {pid}")
    cur = app.robot.arm(u["arm"]).programs.get(pid)
    if cur and cur.get("sha256") != ver["sha256"] and not body.get("replace"):
        raise UserError(f"Program {pid} on the {u['arm'].upper()} controller already holds “{cur.get('name')}”.",
                        "Confirm to replace it, or choose another Program ID.", code="replace")
    name = f"{key}_{ver['version']}"[:32]
    job = app.robot.upload(u["arm"], pid, Path(ver["path"]), name, ver["sha256"], bool(body.get("replace")),
                           f"Upload {key} to program {pid}", stage=info["stage"].as_dict())
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
                        on_done=done, stage=info["stage"], mode=mode,    # refused if the mode changed meanwhile
                        alone=bool(body.get("alone")))                 # an arm's full sequence: the other arm stays stopped

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


# ── V32's Task Transition & Start Point Tools (Prepare, optional)

def _transition_view(units: LiteUnits, key: str) -> dict | None:
    tv = units.transition(key)
    if not tv:
        return None
    out = {k: v for k, v in tv.items() if k != "path"}
    out["end_joints_deg"] = [round(v / 1000.0, 3) for v in tv.get("end_millideg") or []]
    out["project_path"] = Path(tv["path"]).relative_to(units.root).as_posix()
    return out


@route("GET", U + "/transition")
def get_transition(app, q, body, m):
    units, key = units_of(app, m.group(1)), m.group(2)
    _unit(units, key)
    return {"transition": _transition_view(units, key)}


@route("POST", U + "/transition")
def post_transition(app, q, body, m):
    """Keep a transition trajectory for this arm step: ``{name, content, arm, pole_mm, hand}``, as a
    loaded recording (the arm from its name or ``arm``, and the stage it was taught with, both
    required). Its L1-L4 are checked on its bytes and shown; a file that fails them can still be
    simulated, but is never sent to the robot."""
    import tempfile

    from ..motion import track
    from .library import detect_arm

    units, key = units_of(app, m.group(1)), m.group(2)
    _unit(units, key)
    content = body.get("content")
    if not isinstance(content, str) or not content.strip():
        raise UserError("The file is empty.")
    if len(content) > 20_000_000:
        raise UserError("The file is too large for a trajectory (over 20 MB).")
    name = Path(str(body.get("name") or "transition.txt")).name
    named, said = detect_arm(name), body.get("arm") or None
    if said not in (None, "left", "right"):
        raise UserError("The arm is left or right.")
    if named and said and named != said:
        raise UserError(f"{name}'s name says the {named.upper()} arm, but the {said.upper()} arm was given.", code="arm")
    if body.get("pole_mm") is None or body.get("hand") is None:
        raise UserError(f"Say the pole height and hand pose {name} was taught with.", code="stage")
    stage = staging.make_stage(body["pole_mm"], body["hand"], "stated when the transition was loaded")
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td) / name
        tmp.write_text(content, encoding="utf-8")
        try:
            track.read_md(tmp)
        except ValueError as exc:
            raise UserError(f"This file is not a trajectory Lite can use: {exc}",
                            "A trajectory has one line per 5 ms: {\"point\":[7 whole numbers]}.") from None
        units.set_transition(key, tmp, name, taught_arm=said or named, stage=stage)
    return {"transition": _transition_view(units, key)}


def _checked_transition(units: LiteUnits, key: str, arm: str) -> dict:
    """The transition a robot step sends or runs: its bytes still the ones kept, and L1-L4 passed
    again on them (a transition is not tuned, so nothing else vouches for it)."""
    from dataclasses import asdict

    from ..motion import track
    from ..motion.checks import check_bytes, failures_text
    from ..motion.limits import load_limits

    tv = units.transition(key)
    if not tv:
        raise UserError("This step has no transition trajectory yet.", "Choose its .txt first.")
    path = Path(tv["path"])
    if track.sha256_file(path) != tv.get("sha256"):
        raise UserError(f"{tv['name']} is not the file that was kept: its bytes have changed.", "Choose it again.")
    rep = check_bytes(track.read_md(path), load_limits(arm), with_departure=False)
    bad = [asdict(c) for c in rep.checks if c.hard and c.passed is False and c.id in ("L1", "L2", "L3", "L4")]
    if bad:
        raise UserError(f"{tv['name']} fails {failures_text({'checks': bad})}, so it never goes to the robot.",
                        "Use a transition within the limits (tune it as a region's recording first), or watch it in "
                        "the simulation only.")
    return tv


@route("POST", U + "/transition/upload")
def post_transition_upload(app, q, body, m):
    """Send the step's transition to a Program ID (1-100), as Real Test sends a kept version: its
    arm and stage checked, its limits checked on the bytes sent; a slot holding another program
    needs ``replace``. ``{unchanged: true}`` when the slot already holds it."""
    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    require_robot_mode(app)
    pid = _program_check(body.get("program_id"))
    info = identity(units, key, "transition")
    tv = _checked_transition(units, key, u["arm"])
    job = app.robot.upload(u["arm"], pid, Path(tv["path"]), f"{key}_transition"[:32], tv["sha256"], bool(body.get("replace")),
                           f"Upload the {u['arm'].upper()} transition of {key} to program {pid}", stage=info["stage"].as_dict())
    return {"unchanged": True, "program_id": pid} if job is None else job.view()


@route("POST", U + "/transition/run")
def post_transition_run(app, q, body, m):
    """Run the step's transition, once sent: on this arm alone, the other arm stopped before and
    while it runs (V32: "the opposite arm must remain stopped"), the pole and hand staged first; in
    Hardware sim the controller's SIM mode, in Real confirmed."""
    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    mode = require_robot_mode(app)
    require_confirmed(app, body, "The transition")
    pid = _program_check(body.get("program_id"))
    info = identity(units, key, "transition")
    tv = _checked_transition(units, key, u["arm"])
    prog = app.robot.arm(u["arm"]).programs.get(pid)
    if not prog or prog.get("sha256") != tv["sha256"]:
        raise UserError(f"Program {pid} does not hold this transition.", "Send it first.")
    dry = mode != "real"
    return app.robot.run(u["arm"], pid, dry_run=dry, stage=info["stage"], mode=mode, alone=True,
                         title=f"{'Hardware sim' if dry else 'Real'} transition {u['arm'].upper()} ({key})").view()


# ── custom merges, one arm at a time (feedback v2, slide 10; merges.py)

M = P + r"/merges/(left|right)/([A-Za-z0-9_.\-]+)"


@route("GET", P + "/merges")
def get_merges(app, q, body, m):
    units = units_of(app, m.group(1))
    return {s: merges.listing(units, s) for s in ARMS}


@route("POST", P + r"/merges/(left|right)")
def post_merge(app, q, body, m):
    """Merge files of one arm, in order: ``{name, files: [{name, content}], pole_mm, hand}``. Each must be
    that arm's (by its name, or as said); the pole and hand are the merged program's, for all of it."""
    import tempfile

    from ..motion import track
    from .library import detect_arm

    units, side = units_of(app, m.group(1)), m.group(2)
    files = body.get("files")
    if not isinstance(files, list) or not files or len(files) > 50:
        raise UserError("Choose 1 to 50 files to merge.")
    if body.get("pole_mm") is None or body.get("hand") is None:
        raise UserError("Say the pole height and hand pose the merged program runs with.", code="stage")
    stage = staging.make_stage(body["pole_mm"], body["hand"], "stated for the merge")
    parts = []
    with tempfile.TemporaryDirectory() as td:
        for i, f in enumerate(files):
            name = Path(str((f or {}).get("name") or f"part_{i + 1}.txt")).name
            content = (f or {}).get("content")
            if not isinstance(content, str) or not content.strip():
                raise UserError(f"{name} is empty.")
            arm = detect_arm(name)
            if arm and arm != side:
                raise UserError(f"{name}'s name says the {arm.upper()} arm: it never merges into the {side.upper()} arm's file.", code="arm")
            tmp = Path(td) / f"{i}_{name}"
            tmp.write_text(content, encoding="utf-8")
            try:
                parts.append(dict(name=name, md=track.read_md(tmp)))
            except ValueError as exc:
                raise UserError(f"{name} is not a trajectory Lite can use: {exc}") from None
    out = merges.save(units, side, str(body.get("name") or ""), parts, stage)
    return {k: v for k, v in out.items() if k != "path"}


@route("POST", M + "/name")
def post_merge_name(app, q, body, m):
    units = units_of(app, m.group(1))
    out = merges.rename(units, m.group(2), m.group(3), body.get("name"))
    return {k: v for k, v in out.items() if k != "path"}


@route("GET", M + "/download")
def get_merge_download(app, q, body, m):
    from .app import RawResponse

    mm = merges.get(units_of(app, m.group(1)), m.group(2), m.group(3))
    return RawResponse(Path(mm["path"]).read_bytes(), "text/plain; charset=utf-8", mm["name"])


@route("POST", M + "/timeline")
def post_merge_timeline(app, q, body, m):
    """A merge in 3D: staged first, then the file on the 5 ms clock (``start``: where the arm was left)."""
    from ..motion import track
    from ..motion.limits import load_limits
    from .routes_robot import _mesh_links, _payload

    units, side = units_of(app, m.group(1)), m.group(2)
    info = identity(units, None, f"merge:{side}/{m.group(3)}")
    p = _payload(app)
    out = staging.execution_timeline(side, track.read_md(info["path"]), info["stage"],
                                     home=staging.start_values(side, dict(p["home"]), sequence.check_start(body.get("start"))),
                                     links=staging.staged_links(side, _mesh_links(p)), rating_deg_s=load_limits(side).rating_deg_s)
    out["scene"] = scene_of(units, None, None, body.get("task"))
    return out


@route("POST", M + "/upload")
def post_merge_upload(app, q, body, m):
    """Send a merge to a Program ID (its limits read on the bytes sent and shown; ``replace`` as for Real Test)."""
    units, side = units_of(app, m.group(1)), m.group(2)
    require_robot_mode(app)
    pid = _program_check(body.get("program_id"))
    mm = merges.checked(units, side, m.group(3))
    require_limits_seen(body, mm["name"], mm["limits_ok"], mm["limits_text"])
    job = app.robot.upload(side, pid, Path(mm["path"]), Path(mm["name"]).stem[:32], mm["sha256"], bool(body.get("replace")),
                           f"Upload {mm['name']} to program {pid}", stage=mm["stage"])
    return {"unchanged": True, "program_id": pid} if job is None else job.view()


@route("POST", M + "/run")
def post_merge_run(app, q, body, m):
    """Run a merge, once sent, on its arm alone (the other arm stays stopped)."""
    units, side = units_of(app, m.group(1)), m.group(2)
    mode = require_robot_mode(app)
    require_confirmed(app, body, "The merged run")
    pid = _program_check(body.get("program_id"))
    mm = merges.checked(units, side, m.group(3))
    prog = app.robot.arm(side).programs.get(pid)
    if not prog or prog.get("sha256") != mm["sha256"]:
        raise UserError(f"Program {pid} does not hold {mm['name']}.", "Upload it to the controller first.")
    dry = mode != "real"
    return app.robot.run(side, pid, dry_run=dry, stage=staging.stage_from(mm["stage"]), mode=mode, alone=True,
                         title=f"{'Hardware sim' if dry else 'Real'} run of {mm['name']} ({side.upper()})").view()


# ── single files and controller programs on one arm (feedback v2, slide 11)

def _sendable(units: LiteUnits, key: str, arm: str, which: str) -> dict:
    """A file a robot step may send: a kept version or the transition, each checked on its bytes;
    never a recording as taught."""
    if which == "transition":
        tv = _checked_transition(units, key, arm)
        return dict(path=Path(tv["path"]), sha256=tv["sha256"], name=f"{key}_transition"[:32], what=tv["name"])
    if which == "version" or which.startswith("version:"):
        ver = checked_version(units, key, arm, which)
        return dict(path=Path(ver["path"]), sha256=ver["sha256"], name=f"{key}_{ver['version']}"[:32],
                    what=ver.get("name") or f"{key} {ver['version']}", limits_ok=ver.get("limits_ok"),
                    limits_text=ver.get("limits_text"))
    raise UserError("A recording as taught never runs on the real arms.",
                    "Tune it and keep the version: that file can be sent. In 3D every file plays.", code="mode")


@route("POST", U + "/file/upload")
def post_file_upload(app, q, body, m):
    """Send one of the region's files to a Program ID: ``{which: version | version:vNNN | transition,
    program_id, replace}`` (its arm, stage and limits checked; a slot holding another program needs
    ``replace``)."""
    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    require_robot_mode(app)
    pid = _program_check(body.get("program_id"))
    which = str(body.get("which") or "version")
    info = identity(units, key, which)
    f = _sendable(units, key, u["arm"], which)
    require_limits_seen(body, f["what"], f.get("limits_ok"), f.get("limits_text"))
    job = app.robot.upload(u["arm"], pid, f["path"], f["name"], f["sha256"], bool(body.get("replace")),
                           f"Upload {f['what']} to program {pid}", stage=info["stage"].as_dict())
    return {"unchanged": True, "program_id": pid} if job is None else job.view()


@route("POST", U + "/file/run")
def post_file_run(app, q, body, m):
    """Run one of the region's files, once sent, on its arm alone: ``{which, program_id, confirmed}``."""
    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    mode = require_robot_mode(app)
    require_confirmed(app, body, "The run")
    pid = _program_check(body.get("program_id"))
    which = str(body.get("which") or "version")
    info = identity(units, key, which)
    f = _sendable(units, key, u["arm"], which)
    prog = app.robot.arm(u["arm"]).programs.get(pid)
    if not prog or prog.get("sha256") != f["sha256"]:
        raise UserError(f"Program {pid} does not hold {f['what']}.", "Send it first.")
    dry = mode != "real"
    return app.robot.run(u["arm"], pid, dry_run=dry, stage=info["stage"], mode=mode, alone=True,
                         title=f"{'Hardware sim' if dry else 'Real'} run of {f['what']} ({u['arm'].upper()})").view()


@route("POST", r"/api/robots/(left|right)/programs/run")
def post_program_run(app, q, body, m):
    """Run a program stored on the arm's controller, on that arm alone: ``{program_id, confirmed}``,
    and for one Lite did not send, the ``pole_mm`` and ``hand`` it runs with (staged first; Lite
    cannot check its limits or show it in 3D, and the controller moves the arm to its first point)."""
    session(app)
    side = _arm(m)
    mode = require_robot_mode(app)
    require_confirmed(app, body, "Running the program")
    pid = _program_check(body.get("program_id"))
    held = next((p for p in app.robot.programs(side) if int(p["id"]) == pid), None)
    if not held or not held.get("name"):
        raise UserError(f"Program {pid} is empty on the {side.upper()} controller.")
    dry = mode != "real"
    title = f"{'Hardware sim' if dry else 'Real'} run of program {pid} ({side.upper()})"
    if held.get("sha256"):                               # Lite sent it, and the controller still holds it
        st = staging.stage_from(held.get("stage"))
        if st is None:
            raise UserError(f"Lite has no pole and hand recorded for program {pid}.", "Send it again from its region.", code="stage")
        return app.robot.run(side, pid, dry_run=dry, stage=st, mode=mode, alone=True, title=title).view()
    if body.get("pole_mm") is None or body.get("hand") is None:
        raise UserError(f"Lite did not send program {pid} (“{held['name']}”): say the pole height and hand pose it runs with.",
                        "Lite cannot check its limits or show it in 3D; the controller moves the arm to its first point "
                        "in its own joint move.", code="stage")
    st = staging.make_stage(body["pole_mm"], body["hand"], "stated for a program Lite did not send")
    return app.robot.run_foreign(side, pid, stage=st, mode=mode, title=title).view()


@route("GET", U + "/end")
def get_end(app, q, body, m):
    """Where the step's file ends (V32's Previous Task End): the last line of its kept version, or of
    its active recording before one is kept, as joints (deg), with the file and its stage."""
    from ..motion import track

    units, key = units_of(app, m.group(1)), m.group(2)
    _unit(units, key)
    which = "version" if units.current_version(key) else "raw"
    info = identity(units, key, which)
    md = track.read_md(info["path"])
    ver = units.current_version(key) if which == "version" else None
    name = (ver.get("name") or f"{key} {ver['version']}") if ver else info["name"]
    return {"which": which, "name": name, "sha256": track.sha256_file(info["path"]), "lines": int(md.shape[0]),
            "joints_deg": [round(float(v) / 1000.0, 3) for v in md[-1]], "stage": info["stage"].as_dict()}


@route("GET", r"/api/robots/(left|right)/capture")
def get_capture(app, q, body, m):
    """V32's Capture Current Pose: what the linked controller reads now (J1-J7, the tool pose, the
    pole, the hand, the time and the controller)."""
    session(app)
    out = app.robot.capture(_arm(m))
    app.log.hardware(out["arm"], "capture pose", ok=True, args=dict(joints_deg=out["joints_deg"], pose=out["pose"],
                                                                     pole_mm=out["pole_mm"], hand=out["hand"]))
    return out


@route("POST", r"/api/robots/(left|right)/move-joints")
def post_move_joints(app, q, body, m):
    """A move to a start point's joints (V32's Move to Start Point): ``{joints_deg: [7], name,
    confirmed}``, each joint within the arm's limits; on a linked arm at 30 % and read back."""
    from ..motion.limits import load_limits

    session(app)
    side = _arm(m)
    mode = require_robot_mode(app)
    q_deg = body.get("joints_deg")
    try:
        q_deg = [float(v) for v in q_deg]
    except (TypeError, ValueError):
        q_deg = None
    if not q_deg or len(q_deg) != 7:
        raise UserError("A start point needs its seven joint values.", "Capture it again.")
    for i, (v, (lo, hi)) in enumerate(zip(q_deg, load_limits(side).pos_deg)):
        if not lo <= v <= hi:
            raise UserError(f"J{i + 1} of that start point is {v:.1f}°, outside the arm's range ({lo:g}° to {hi:g}°).")
    require_confirmed(app, body, "Moving the arm")
    name = str(body.get("name") or "the start point")[:80]
    return app.robot.move_to(side, f"start point “{name}”", f"{side.upper()} to {name}", mode=mode,
                             target_millideg=[round(v * 1000) for v in q_deg]).view()


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


@route("GET", r"/api/robots/(left|right)/points")
def get_points(app, q, body, m):
    """The global points stored on a linked arm's controller (``search``: part of the name, any
    case). The stand-in has none: ``points`` is empty and ``note`` says why."""
    session(app)
    side = _arm(m)
    if not app.robot.arm(side).connected:
        return {"arm": side, "points": [], "note": f"Connect the {side.upper()} arm to read its global points."}
    pts = app.robot.global_points(side)
    if pts is None:
        return {"arm": side, "points": [], "note": "The stand-in robot has no controller, so no global points."}
    want = str((q.get("search") or [""])[0]).strip().casefold()
    return {"arm": side, "total": len(pts), "points": [p for p in pts if want in p["name"].casefold()]}


@route("POST", r"/api/robots/(left|right)/points/save")
def post_point_save(app, q, body, m):
    """The arm's pose now, saved on its controller as a global point (feedback v2, slide 14):
    ``{name, joints_deg (the pose the operator saw), replace}``. In Hardware sim or Real (it writes to
    the controller); a point of that name is replaced only after a confirmation."""
    import re

    session(app)
    side = _arm(m)
    require_robot_mode(app)
    name = str(body.get("name") or "").strip()
    if not re.fullmatch(r"[A-Za-z0-9_\-]{1,32}", name):
        raise UserError("A global point's name is 1 to 32 letters, digits, _ or -.", "For example STP_SEAT_1.")
    expect = body.get("joints_deg")
    if expect is not None:
        try:
            expect = [float(v) for v in expect][:7]
        except (TypeError, ValueError):
            raise UserError("The joints shown must be numbers.") from None
        if len(expect) != 7:
            expect = None
    return app.robot.save_global_point(side, name, expect, bool(body.get("replace")))


@route("POST", r"/api/robots/(left|right)/points/move")
def post_point_move(app, q, body, m):
    """MoveJ or MoveL to a global point at the speed slider's %: ``{name, move: movej|movel, wait_s, confirmed}``."""
    session(app)
    side = _arm(m)
    mode = require_robot_mode(app)
    name = str(body.get("name") or "").strip()
    if not name:
        raise UserError("Choose a global point first.")
    move = str(body.get("move") or "movej").lower()
    if move not in ("movej", "movel"):
        raise UserError("The move is MoveJ or MoveL.")
    speed = int(app.robot.speed_pct)                  # the speed slider, the same for every MoveJ / MoveL
    wait_s = float(body.get("wait_s") or 0)
    if not 0 <= wait_s <= 3600:
        raise UserError("The wait is 0 to 3,600 seconds.")
    require_confirmed(app, body, "Moving the arm to a global point")
    label = "MoveL" if move == "movel" else "MoveJ"
    return app.robot.move_point(side, name, move, speed, wait_s, f"{side.upper()} {label} to {name}", mode=mode).view()


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
