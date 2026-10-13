"""The Lite UI's region loop: ``/api/projects/<id>/…`` for one project's tasks and arm steps.

    GET  tasks                                the task queue (decision D35)
    GET  units/<KEY>                          one arm step: settings, recordings, tuning, versions, results
    POST units/<KEY>/recordings               add a recording: existing (library) or a loaded .txt
    POST units/<KEY>/active | name            which recording is used; the operator's file name
    GET  units/<KEY>/track | frames           joint data for the plot; 3D frames for the simulation
    POST units/<KEY>/settings | undo | redo   tuning settings, one undo step per change
    POST units/<KEY>/optimise                 a job: the tuned candidate, with every limit checked
    POST units/<KEY>/keep                     keep the candidate as the version to upload
    POST units/<KEY>/teach/start|stop|cancel  drag-teach (the robot stand-in until robot I/O exists)
    POST units/<KEY>/move-start | playback    robot moves, in Hardware sim or Real only
    POST units/<KEY>/upload | run | verdict   Real Test

Robot steps need Hardware sim or Real (decision D30), and a Real move needs ``confirmed``. The
E-stop rules are the stand-in's (``robot_standin.py``): motion on either arm is refused while
either E-stop is not clear; STOP always works.
"""
from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path

from . import fixtures, workers
from .api_v2 import session
from .app import MODE_TEXT, MODES, route
from .jobs import UserError
from .lite_units import TIME_MODES, LiteUnits, open_units, task_list
from .robot_standin import ARMS

_LOCK = threading.Lock()
_UNITS: dict[str, LiteUnits] = {}
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
    """Whether the candidate was made from the current settings and the active recording."""
    raw = units.current_recording(key)
    if not extra or raw is None:
        return False
    src = json.loads(raw.with_suffix(".source.json").read_text(encoding="utf-8")) if raw.with_suffix(".source.json").is_file() else {}
    tuning = {k: v for k, v in units.settings(key).items() if k != "program_id"}
    return extra.get("settings") == tuning and extra.get("raw_sha256") == src.get("sha256")


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
    """``{source: "library"}``: the region's existing taught recording (D31).
    ``{source: "upload", name, content}``: a .txt from this computer, checked line by line.
    Either becomes the active recording; nothing earlier is replaced."""
    import tempfile

    from ..motion import track

    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    if body.get("source") == "library":
        f = units.library_file(key)
        if not f:
            raise UserError("This region has no existing recording.", "Teach it, or load a file from this computer.")
        path = units.add_recording(key, Path(f["path"]), note=f"existing recording {f['file']}: {f.get('task', '')}",
                                   source="library", name=f["file"])
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
            path = units.add_recording(key, tmp, note=f"loaded file {name}", source="upload", name=name)
    else:
        raise UserError("Say where the recording comes from: library or upload.")
    return {"saved": path.name, "arm": u["arm"], "unit": unit_view(app, units, key)}


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


@route("GET", U + "/frames")
def get_frames(app, q, body, m):
    """The 3D frames of the arm's links for a whole file (``which`` as for ``track``): every
    ``every``-th line and the last; ``every`` is chosen to stay within the frame limit."""
    from . import robot_model
    from .routes_robot import _mesh_links, _payload

    from ..motion import track

    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    md = track.read_md(_track_path(units, key, q))
    n = md.shape[0]
    every = max(int((q.get("every") or ["4"])[0]), math.ceil(n / (robot_model.MAX_TRACK_FRAMES - 1)))
    p = _payload(app)
    links = robot_model.moving_links(u["arm"], _mesh_links(p))
    out = robot_model.track_frames(u["arm"], (md.T / 1000.0).tolist(), every, dict(p["home"]), links)
    out["total_lines"] = int(n)
    return out


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
    raw = units.current_recording(key)
    if raw is None:
        raise UserError("This region has no recording yet.", "Record it, or use an existing or loaded file.")
    settings = units.settings(key)
    tuning = {k: v for k, v in settings.items() if k != "program_id"}
    if u.get("max_dev_mm") and not settings.get("max_dev_mm"):
        settings["max_dev_mm"] = u["max_dev_mm"]
    cand, _, extra_path = _candidate_files(units, key)
    cand.parent.mkdir(parents=True, exist_ok=True)
    src = _read(raw.with_suffix(".source.json")) or {}
    name = "Time-optimal" if settings["optimiser"] == "toppra" else "Default"

    def work(job):
        job.progress(f"Tuning with {name}", 1, 2)
        fut = workers.pool().submit(workers.optimise_unit, str(raw), str(cand), u["arm"], settings)
        while not fut.done():
            job.sleep(0.1)
        res = fut.result()
        if res.get("error"):
            raise UserError(res["error"], res.get("hint"))
        job.progress("Checked against the limits", 2, 2)
        job.log(res["summary"]["message"])
        extra = dict(util=res.get("util"), progress=res.get("progress"), settings=tuning,
                     raw_file=raw.name, raw_sha256=src.get("sha256"), made=time.strftime("%Y-%m-%d %H:%M:%S"))
        extra_path.write_text(json.dumps(extra), encoding="utf-8")
        return dict(summary=res["summary"], checks=res["checks"], cut=res["recipe"].get("cut"))

    return app.jobs.start("optimise", f"Tune {key}", work, unit=key, arm=u["arm"]).view()


@route("POST", U + "/keep")
def post_keep(app, q, body, m):
    """Keep the tuned candidate as the version to upload: only when every hard limit passes and
    it was made from the current settings and recording."""
    units, key = units_of(app, m.group(1)), m.group(2)
    _unit(units, key)
    cand, rec, extra_path = _candidate_files(units, key)
    if not cand.is_file():
        raise UserError("There is no tuned trajectory yet.", "Run the simulation to make one.")
    if not _fresh(units, key, _read(extra_path)):
        raise UserError("The settings changed after the last tuning.", "Re-run the simulation first.")
    result = (_read(rec) or {}).get("result") or {}
    if not result.get("uploadable"):
        raise UserError("This trajectory does not pass every limit, so it cannot go to the robot.",
                        "Follow the hint under the results, then re-run the simulation.")
    ver = units.current_version(key)
    if ver and ver.get("sha256") == (_read(rec) or {}).get("track_sha256"):
        return unit_view(app, units, key)                            # already kept
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
        require_robot_mode(app)
        require_confirmed(app, body, "Drag teaching")
        return {"arm": app.robot.teach_start(side)}
    if verb == "cancel":
        app.robot.arm(side).teaching = False
        return {"arm": app.robot.arm(side).view()}
    secs = app.robot.teach_stop(side)
    # stand-in: the "recording" is the region's existing one, else any of the same arm
    from .project import library

    files = list(library(units.fixture)["files"].values()) or list(library("commode_c")["files"].values())
    pick = next((f for f in files if f.get("unit") == key), None) or next((f for f in files if f.get("arm") == side), None)
    from .. import vendor_paths

    src = Path(pick["path"]) if pick else vendor_paths.LITE_ROOT / "tests/data/recordings/commode_c_taught_paths/FILE_09.txt"
    name = str(body.get("name") or f"{key}_taught.txt")
    path = units.add_recording(key, src, note=f"stand-in recording ({secs:.0f} s of teaching)", source="teach", name=name)
    return {"saved": path.name, "seconds": round(secs, 1), "unit": unit_view(app, units, key)}


@route("POST", U + "/move-start")
def post_move_start(app, q, body, m):
    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    require_robot_mode(app)
    require_confirmed(app, body, "Moving the arm")
    which = body.get("which") or "raw"
    if which == "version" and not units.current_version(key):
        raise UserError("There is no tuned version yet.", "Finish Tune + Simulation first.")
    if which == "raw" and units.current_recording(key) is None:
        raise UserError("There is no recording yet.")
    what = "the tuned trajectory's start" if which == "version" else "the recorded start"
    return app.robot.move_to(u["arm"], what, f"{u['arm'].upper()} to {what} ({key})").view()


@route("POST", U + "/playback")
def post_playback(app, q, body, m):
    units, key = units_of(app, m.group(1)), m.group(2)
    u = _unit(units, key)
    mode = require_robot_mode(app)
    require_confirmed(app, body, "Playback")
    raw = units.current_recording(key)
    if raw is None:
        raise UserError("There is no recording to play back.")
    lines = sum(1 for ln in raw.read_bytes().split(b"\n") if ln.strip())
    return app.robot.playback(u["arm"], lines, dry_run=mode != "real", title=f"Playback {key}").view()


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
    ver = units.current_version(key)
    if not ver:
        raise UserError("There is no tuned version to upload.", "Finish Tune + Simulation (Simulation Looks Good).")
    if units.settings(key).get("program_id") != pid:
        units.update_settings(key, {"program_id": pid}, f"Program ID {pid}")
    cur = app.robot.arm(u["arm"]).programs.get(pid)
    if cur and cur.get("sha256") != ver["sha256"] and not body.get("replace"):
        raise UserError(f"Program {pid} on the {u['arm'].upper()} controller already holds “{cur.get('name')}”.",
                        "Confirm to replace it, or choose another Program ID.", code="replace")
    name = f"{key}_{ver['version']}"[:32]
    job = app.robot.upload(u["arm"], pid, Path(ver["path"]), name, ver["sha256"], True, f"Upload {key} to program {pid}")
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
    ver = units.current_version(key)
    pid = units.settings(key).get("program_id")
    if not ver or not pid:
        raise UserError("Upload the trajectory first.")
    prog = app.robot.arm(u["arm"]).programs.get(int(pid))
    if not prog or prog.get("sha256") != ver["sha256"]:
        raise UserError(f"Program {pid} does not hold this region's current version.", "Upload it again.")
    dry = mode != "real"

    def done(out):
        units.add_result(key, "dryrun_ok" if dry else "real_ok", ver["sha256"], True, out)

    job = app.robot.run(u["arm"], int(pid), dry_run=dry, title=("Hardware sim run " if dry else "Real run ") + key,
                        on_done=done)

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
    if mode != "visual":
        if not any(a["connected"] for a in app.robot.view().values()):
            raise UserError("Connect at least one arm first.", code="connect")
        why = app.robot.motion_block()
        if why and mode == "real":
            raise UserError(why, "Release the E-stop and press Recover.")
        if mode == "real" and not body.get("confirmed"):
            raise UserError("Switching to the real robot needs your confirmation.", code="confirm")
    app.ui["mode"] = mode
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
        app.ui["mode"] = "visual"
    return out


@route("POST", r"/api/robots/(left|right)/pole")
def post_pole(app, q, body, m):
    session(app)
    side = _arm(m)
    require_robot_mode(app)
    require_confirmed(app, body, "Moving the pole")
    try:
        mm = float(body.get("mm"))
    except (TypeError, ValueError):
        raise UserError("Enter the pole height in mm (0 to 315).") from None
    if not 0 <= mm <= 315:
        raise UserError("The pole height is 0 to 315 mm.")
    return app.robot.move_pole(side, mm, f"Pole {side.upper()} to {mm:.0f} mm").view()


@route("POST", r"/api/robots/(left|right)/hand")
def post_hand(app, q, body, m):
    session(app)
    side = _arm(m)
    require_robot_mode(app)
    require_confirmed(app, body, "Moving the hand")
    return app.robot.set_hand(side, body.get("counts") or [], f"Hand {side.upper()}").view()


@route("POST", r"/api/robots/(left|right)/move-pose")
def post_move_pose(app, q, body, m):
    """A start pose by profile name (the page's Setup & Safety and Redo)."""
    session(app)
    side = _arm(m)
    require_robot_mode(app)
    require_confirmed(app, body, "Moving the arm")
    name = str(body.get("name") or "Default Start Pose")[:80]
    return app.robot.move_to(side, f"“{name}”", f"{side.upper()} to {name}").view()


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


@route("POST", r"/api/robots/(left|right)/estop-test")
def post_estop_test(app, q, body, m):
    side = _arm(m)
    session(app, engineer=True)
    out = app.robot.press_estop(side, bool(body.get("pressed")))
    if out["estop"] != "CLEAR" and app.ui["mode"] == "real":
        app.ui["mode"] = "hwsim"
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
