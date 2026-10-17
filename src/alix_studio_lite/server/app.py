"""ALIX Studio Lite web server: the pages, their static files and the JSON API.

Two front ends share one backend:

- ``/``: the Lite UI, the designer's page (``web/index.html``) plus Lite's bridge
  (``web/lite/``), talking to ``/api/*`` (``api_v2.py``);
- ``/legacy/``: Legacy Lite (``web_legacy/``), kept working, talking to ``/api/lite/*`` (below).

A stdlib ``ThreadingHTTPServer`` bound to 127.0.0.1, as ALIX Studio does. Refusals are HTTP 200
``{"error": ..., "hint": ...}`` (the Studio convention), so the page always gets a sentence to
show. Long work (optimise, upload, run, prepare) is a job the page polls.
"""
from __future__ import annotations

import hashlib
import json
import mimetypes
import os
import re
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, quote, urlparse

from .. import __version__, code_id, vendor_paths
from . import workers
from .auth import Auth
from .jobs import Jobs, UserError
from .library import TaughtLibrary
from .logbook import Logbook
from .project import Project
from .robot_standin import ARMS, StandInRobot
from .store import ProjectStore

WEB = Path(__file__).resolve().parents[1] / "web"                  # the Lite UI
WEB_LEGACY = Path(__file__).resolve().parents[1] / "web_legacy"    # Legacy Lite, at /legacy/
STUDIO_VENDOR = vendor_paths.ALIX / "core/python/alix_taskgen/src/alix_taskgen/web_editor/vendor"
VENDOR_FILES = {"plotly.min.js", "three.min.js", "OrbitControls.js"}
DECK = vendor_paths.ASSETS / "regions/deck"
MESHES = {
    "commode_c/body.stl": vendor_paths.TASKS_PKG / "config/commode_cleaning/commode_c/models/body.stl",
    "commode_c/lid.stl": vendor_paths.TASKS_PKG / "config/commode_cleaning/commode_c/models/lid.stl",
    "commode_c/seat.stl": vendor_paths.TASKS_PKG / "config/commode_cleaning/commode_c/models/seat.stl",
    "cubicle/1400.stl": vendor_paths.CUBICLES / "cubicle_1400mm.stl",
    "cubicle/800.stl": vendor_paths.CUBICLES / "cubicle_800mm.stl",
}
#: decision D30: Visual sim (the 3D robot moves, no hardware) / Hardware sim (the controller's own SIM
#: mode, rm_set_arm_run_mode(0): programs run, the arms stay still) / Real (the arms move)
MODES = ("visual", "hwsim", "real")
CODE_ID = code_id()                      # the code this server runs, as loaded at its start
#: jobs that drive the robot: their failures are hardware alerts (the page shows them, bottom right)
ROBOT_JOB_KINDS = ("run", "prepare", "teach", "move", "playback", "pole", "hand", "stage", "upload")
#: the only /api/ requests without the session: logging in and out, and STOP (never refused)
OPEN_ROUTES = re.compile(r"^/api/(auth/login|auth/logout|robots/stop-both|robots/(left|right)/stop|lite/robot/stop|lite/hello|audit)$")
MODE_TEXT = {"visual": "Visual sim: the 3D robot moves, no hardware",
             "hwsim": "Hardware sim: nothing moves (the controllers' SIM mode; pole and hand emulated)",
             "real": "Real robot: the arms, the pole and the hand move"}


class LiteApp:
    def __init__(self, project: Project, data_dir: Path, robot_link: str | None = None, log_dir: Path | None = None,
                 projects_dir: Path | None = None):
        self.project = project
        self.data_dir = Path(data_dir)
        self.log = Logbook(Path(log_dir) if log_dir else self.data_dir / "logs")   # every activity, and the hardware's
        self.jobs = Jobs(on_end=self._job_ended)
        self.robot = StandInRobot(self.data_dir, self.jobs, link=robot_link, log=self.log)
        self.auth = Auth(self.data_dir)                       # both PINs, hashed (handoff section 19)
        # the Lite UI's projects: in projects_dir (run_lite: Resources/Projects in the repository), each
        # in a folder named as it was created; this computer's last project and trash in <data>/projects
        home = self.data_dir / "projects"
        self.store = ProjectStore(Path(projects_dir) if projects_dir else home, home=home)
        self.adopted = self.store.adopt(home) if projects_dir else []   # kept by id there before: copied once
        for p in self.adopted:
            self.log.activity(f"project {p['id']} not copied: {p['error']}" if p.get("error") else f"project {p['id']} copied to {p['folder']}",
                              detail=p, ok=not p.get("error"))
        self.library = TaughtLibrary(self.data_dir)            # the Trajectory Viewer's files
        self.settings_path = self.data_dir / "settings.json"
        self.ui = {"mode": "visual", "role": "operator", "cubicle_mm": 1400, "theme": "light",
                   "fixture": project.fixture}
        try:
            self.ui.update(json.loads(self.settings_path.read_text(encoding="utf-8")))
        except (OSError, ValueError):
            pass
        self.ui.pop("engineer_pin", None)                     # the PINs now live in auth.json
        self.ui["mode"] = "visual"              # Lite always starts in Visual sim (NFR-UX f)
        self.ui["role"] = "operator"
        self._toppra = None
        threading.Thread(target=self._status_loop, name="lite-status-log", daemon=True).start()

    def _status_loop(self) -> None:
        """The unit's status, twice a second while an arm is connected (logbook.status)."""
        while True:
            time.sleep(0.5)
            try:
                arms = {}
                for side, a in self.robot.arms.items():
                    if not a.connected:
                        continue
                    rec = dict(estop=a.estop, teaching=a.teaching)
                    link = self.robot.links.get(side)
                    if link is not None:
                        fb = link.view()["feedback"]
                        rec.update({k: fb.get(k) for k in ("joints_deg", "errors", "run_mode", "pole_mm", "hand", "io3",
                                                           "age_s", "busy", "stale")})
                    else:
                        rec.update(pole_mm=round(a.pole_mm, 1), hand=list(a.hand), stand_in=True)
                    arms[side] = rec
                if arms:
                    jobs = [dict(kind=j.kind, arm=j.arm, title=j.title, stage=j.stage) for j in self.jobs.running()]
                    self.log.status(dict(t=round(time.time(), 3), mode=self.ui["mode"], arms=arms, jobs=jobs))
            except Exception:                                   # noqa: BLE001 - a log never stops Lite
                pass

    def _job_ended(self, job) -> None:
        """Every job's end goes to the log; a robot job that failed is a hardware alert too. A
        tune's says what it came to, so a Keep it refused is explained in the log."""
        v = job.view()
        detail, came = dict(id=job.id, unit=job.unit, arm=job.arm, elapsed_s=v["elapsed_s"]), ""
        res = v.get("result") if isinstance(v.get("result"), dict) else {}
        if job.kind == "optimise" and isinstance(res.get("summary"), dict):
            from ..motion.checks import failures_text

            s = res["summary"]
            bad = failures_text(res.get("checks")) or "a limit"
            came = ("passes every limit" if s.get("limits_ok", s.get("uploadable"))
                    else f"breaks {bad}, shown, not enforced" if s.get("optimiser") == "manual" else f"fails {bad}")
            detail.update(optimiser=s.get("optimiser"), achieved_s=s.get("achieved_s"), outcome=came)
        self.log.activity(f"job {job.kind} {v['state']}: {job.title}" + (f" ({came})" if came else ""), detail=detail,
                          ok=v["state"] != "failed", error=v["error"])
        if v["state"] == "failed" and job.kind in ROBOT_JOB_KINDS:
            self.log.alert("error", f"{job.title}: {v['error']}", hint=v["hint"], side=job.arm, source=job.kind)

    def set_mode(self, mode: str) -> None:
        """Visual sim, Hardware sim or Real (D30). The robot backend follows it: in Hardware sim it
        moves nothing (the controllers' SIM mode, the pole and the hand emulated). Under the robot's
        lock, so a job never starts while the two differ."""
        with self.robot.lock:
            self.ui["mode"] = mode
            self.robot.mode = mode

    def save_ui(self) -> None:
        self.data_dir.mkdir(parents=True, exist_ok=True)
        keep = {k: v for k, v in self.ui.items() if k in ("cubicle_mm", "theme", "fixture")}
        self.settings_path.write_text(json.dumps(keep, indent=1), encoding="utf-8")

    def toppra(self) -> bool:
        if self._toppra is None:
            try:
                import importlib.util

                self._toppra = importlib.util.find_spec("toppra") is not None
            except Exception:                                              # noqa: BLE001
                self._toppra = False
        return self._toppra

    def state(self) -> dict:
        return dict(version=__version__, data_dir=str(self.data_dir), mode=self.ui["mode"], mode_text=MODE_TEXT[self.ui["mode"]],
                    role="engineer" if _is_engineer(self) else "operator", cubicle_mm=self.ui["cubicle_mm"], theme=self.ui["theme"],
                    project=self.project.meta, fixture=self.project.fixture, arms=self.robot.view(), robot_backend=self.robot.backend,
                    robot_label=self.robot.label, motion_block=self.robot.motion_block(),
                    toppra=self.toppra(), jobs=[j.view() for j in self.jobs.running()],
                    server_time=time.time())


    def switch_fixture(self, fid: str) -> None:
        from . import fixtures
        from .project import open_project

        if fid not in fixtures.FIXTURES:
            raise UserError(f"Unknown fixture {fid!r}.")
        if self.jobs.running():
            raise UserError("Wait for the running tasks to finish, or press STOP, before changing the fixture.")
        if fid != self.project.fixture:
            self.project = open_project(self.data_dir, fid)
        self.ui["fixture"] = fid
        self.save_ui()


# ───────────────────────────────────────────────────────────── routing

ROUTES: list[tuple[str, re.Pattern, callable]] = []

#: the request being handled on this thread (ThreadingHTTPServer: one thread per request);
#: routes that need headers (the Lite UI's session token) read ``REQUEST.headers``
REQUEST = threading.local()


class RawResponse:
    """A route's non-JSON answer, e.g. a project file download."""

    def __init__(self, data: bytes, ctype: str, filename: str | None = None):
        self.data, self.ctype, self.filename = data, ctype, filename


def route(method: str, pattern: str):
    def deco(fn):
        ROUTES.append((method, re.compile("^" + pattern + "$"), fn))
        return fn
    return deco


def _unit(app: LiteApp, key: str) -> dict:
    p = app.project
    if key not in p.units:
        raise UserError(f"There is no region {key!r}.")
    u = dict(p.units[key])
    u.update(p.status(key))
    u["settings_view"] = p.history_view(key)
    u["recordings"] = p.recordings(key)
    u["versions"] = p.versions(key)
    cand = p.candidate_path(key)
    rec = cand.with_name(cand.stem + ".lite.json")
    u["candidate"] = json.loads(rec.read_text(encoding="utf-8")) if rec.is_file() else None
    ut = cand.with_name(cand.stem + ".util.json")
    u["candidate_util"] = json.loads(ut.read_text(encoding="utf-8")) if (u["candidate"] and ut.is_file()) else None
    u["results"] = p.results(key)["events"][-20:]
    return u


@route("GET", "/api/lite/hello")
def get_hello(app, q, body, m):
    """Who serves this port, with no log-in: how a second start finds the Lite already running, and
    whether it runs the same code."""
    return {"app": "ALIX Studio Lite", "version": __version__, "code": CODE_ID, "pid": os.getpid(),
            "data_dir": str(app.data_dir)}


@route("GET", "/api/lite/state")
def get_state(app, q, body, m):
    return app.state()


@route("GET", "/api/lite/fixtures")
def get_fixtures(app, q, body, m):
    from . import fixtures

    return {"fixtures": fixtures.list_fixtures(), "current": app.project.fixture,
            "spec": fixtures.spec(app.project.fixture, int(app.ui["cubicle_mm"]))}


@route("POST", "/api/lite/fixture")
def post_fixture(app, q, body, m):
    app.switch_fixture(str(body.get("id", "")))
    return app.state()


@route("GET", "/api/lite/library")
def get_library(app, q, body, m):
    from .project import library

    lib = library(app.project.fixture)
    return {"fixture": app.project.fixture, "files": [
        {k: v for k, v in f.items() if k != "path"} for f in lib["files"].values()]}


@route("POST", r"/api/lite/unit/([A-Za-z0-9_\-]+)/import")
def post_import(app, q, body, m):
    """Use an existing recording for a region: one from the fixture's library, or a .txt file
    uploaded from this computer. The file is checked line by line and kept read-only."""
    import tempfile

    from ..motion import track
    from .project import library

    key = m.group(1)
    p = app.project
    if key not in p.units:
        raise UserError(f"There is no region {key!r}.")
    if body.get("library_file"):
        f = library(p.fixture)["files"].get(body["library_file"])
        if not f:
            raise UserError("That recording is not in the library.")
        if f.get("arm") and f["arm"] != p.units[key]["arm"]:
            raise UserError(f"That recording was taught with the {f['arm'].upper()} arm; this region is for the "
                            f"{p.units[key]['arm'].upper()} arm.")
        path = p.add_recording(key, Path(f["path"]), note=f"existing recording {f['file']}: {f.get('task', '')}",
                               source="library")
    else:
        content = body.get("content")
        if not isinstance(content, str) or not content.strip():
            raise UserError("The file is empty.")
        if len(content) > 20_000_000:
            raise UserError("The file is too large for a recording (over 20 MB).")
        name = Path(str(body.get("name") or "uploaded.txt")).name
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td) / name
            tmp.write_text(content, encoding="utf-8")
            try:
                track.read_md(tmp)
            except ValueError as exc:
                raise UserError(f"This file is not a recording Lite can use: {exc}",
                                "A recording has one line per 5 ms: {\"point\":[7 whole numbers]}.")
            path = p.add_recording(key, tmp, note=f"uploaded file {name}", source="upload")
    return {"saved": path.name, "unit": _unit(app, key)}


@route("GET", "/api/lite/catalogue")
def get_catalogue(app, q, body, m):
    return app.project.summary()


@route("GET", r"/api/lite/unit/([A-Za-z0-9_\-]+)")
def get_unit(app, q, body, m):
    return _unit(app, m.group(1))


@route("GET", r"/api/lite/unit/([A-Za-z0-9_\-]+)/track")
def get_track(app, q, body, m):
    key = m.group(1)
    which = (q.get("which") or ["raw"])[0]
    p = app.project
    if which == "raw":
        path = p.current_recording(key)
    elif which == "candidate":
        path = p.candidate_path(key)
    else:
        path = next((Path(v["path"]) for v in p.versions(key) if v["version"] == which), None)
    if not path or not Path(path).is_file():
        raise UserError("That file does not exist yet.")
    return workers.track_arrays(str(path))


@route("POST", r"/api/lite/unit/([A-Za-z0-9_\-]+)/settings")
def post_settings(app, q, body, m):
    key = m.group(1)
    changes = body.get("changes") or {}
    if "target_s" in changes and changes["target_s"] is not None:
        t = float(changes["target_s"])
        if not 0.05 <= t <= 600:
            raise UserError("A target time must be between 0.05 and 600 seconds.")
        changes["target_s"] = round(t, 3)
    if changes.get("optimiser") == "default":           # the earlier name of manual tuning
        changes["optimiser"] = "manual"
    if "optimiser" in changes and changes["optimiser"] not in ("manual", "toppra"):
        raise UserError("Choose manual tuning or Time-optimal.")
    if "program_id" in changes and changes["program_id"] is not None:
        if not 1 <= int(changes["program_id"]) <= 100:
            raise UserError("Program numbers are 1 to 100.")
    return app.project.update_settings(key, changes, body.get("label") or "Change settings")


@route("POST", r"/api/lite/unit/([A-Za-z0-9_\-]+)/(undo|redo)")
def post_undo(app, q, body, m):
    return app.project.step_history(m.group(1), -1 if m.group(2) == "undo" else 1)


@route("POST", r"/api/lite/unit/([A-Za-z0-9_\-]+)/optimise")
def post_optimise(app, q, body, m):
    key = m.group(1)
    p = app.project
    raw = p.current_recording(key)
    if raw is None:
        raise UserError("This region has no recording yet.", "Teach it first.")
    settings = p.settings(key)
    side = p.units[key]["arm"]
    out = p.candidate_path(key)
    out.parent.mkdir(parents=True, exist_ok=True)
    name = "manual tuning" if workers.is_manual(settings) else "Time-optimal"

    def work(job):
        job.progress(f"Optimising with {name}", 1, 2)
        fut = workers.pool().submit(workers.optimise_unit, str(raw), str(out), side, settings)
        while not fut.done():
            job.sleep(0.1)
        res = fut.result()
        if res.get("error"):
            raise UserError(res["error"], res.get("hint"))
        job.progress("Checked against the limits", 2, 2)
        job.log(res["summary"]["message"])
        out.with_name(out.stem + ".util.json").write_text(json.dumps(res.get("util") or {}), encoding="utf-8")
        return res

    return app.jobs.start("optimise", f"Optimise {key}", work, unit=key).view()


@route("POST", r"/api/lite/unit/([A-Za-z0-9_\-]+)/save-version")
def post_save_version(app, q, body, m):
    key = m.group(1)
    cand = app.project.candidate_path(key)
    rec = cand.with_name(cand.stem + ".lite.json")
    if rec.is_file() and not json.loads(rec.read_text(encoding="utf-8")).get("result", {}).get("uploadable"):
        raise UserError("This result does not pass every limit, so it cannot be kept.",
                        "Follow the hint on the Optimise screen, then optimise again.")
    return app.project.save_candidate(key)


@route("POST", r"/api/lite/unit/([A-Za-z0-9_\-]+)/check3d")
def post_check3d(app, q, body, m):
    key = m.group(1)
    p = app.project
    ver = p.current_version(key)
    if not ver:
        raise UserError("Save an optimised version first.", "Go to Optimise and press Keep this version.")

    def work(job):
        lines = int(ver.get("lines") or 1000)
        steps = 20
        for i in range(steps):
            job.progress("Checking every line against the toilet, the walls, the bin and the other arm "
                         "(stand-in)", i + 1, steps)
            job.sleep(0.12)
        res = dict(ok=True, standin=True, lines=lines, cubicle_mm=app.ui["cubicle_mm"],
                   checked=["robot (both arms, hands)", "commode C in this step's lid and seat state",
                            f"cubicle walls, {app.ui['cubicle_mm']} mm", "bin (stowed)", "the other arm"],
                   note="Stand-in result: the real 3D check arrives with the backend (docs/STATUS.md).")
        p.add_result(key, "check_ok", ver.get("sha256"), True, res)
        return res

    return app.jobs.start("check3d", f"3D check {key}", work, unit=key).view()


@route("GET", r"/api/lite/jobs/([A-Za-z0-9_\-]+)")
def get_job(app, q, body, m):
    job = app.jobs.get(m.group(1))
    if not job:
        raise UserError("That job is no longer known.")
    return job.view(int((q.get("since") or ["0"])[0]))


@route("POST", r"/api/lite/jobs/([A-Za-z0-9_\-]+)/cancel")
def post_job_cancel(app, q, body, m):
    job = app.jobs.get(m.group(1))
    if job:
        job.cancel()
    return {"ok": True}


# ── app mode, role, settings

@route("POST", "/api/lite/mode")
def post_mode(app, q, body, m):
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
                raise UserError("Connect at least one arm first.", "Use Connect in the bar at the top.")
            why = app.robot.motion_block()
            if why and mode == "real":
                raise UserError(why, "Release the E-stop and press Recover.")
            if mode == "real" and not body.get("confirmed"):
                raise UserError("Switching to the real robot needs your confirmation.", code="confirm")
        app.set_mode(mode)
    return app.state()


def _token():
    headers = getattr(REQUEST, "headers", None)
    return headers.get("X-Lite-Token") if headers is not None else None


def _is_engineer(app) -> bool:
    """The engineer level belongs to the session that entered the PIN, never to the server."""
    s = app.auth.session(_token())
    return bool(s and s.get("engineer"))


def _require_engineer(app, what: str) -> None:
    if not _is_engineer(app):
        raise UserError(f"Only an engineer can {what}.", "Enter the engineer PIN.", code="engineer")


def _require_confirmed(app, body, what: str, mode: str | None = None) -> None:
    if (mode or app.ui["mode"]) == "real" and not body.get("confirmed"):
        raise UserError(f"{what} on the real robot needs your confirmation.", code="confirm")


@route("POST", "/api/lite/role")
def post_role(app, q, body, m):
    """The engineer level for this session (the engineer PIN), or back to operator."""
    role = body.get("role")
    if role not in ("operator", "engineer"):
        raise UserError("Unknown role.")
    if role == "engineer":
        app.auth.elevate(_token(), str(body.get("pin", "")))
    else:
        app.auth.drop_engineer(_token())
    return app.state()


@route("POST", "/api/lite/settings")
def post_app_settings(app, q, body, m):
    if "cubicle_mm" in body:
        if int(body["cubicle_mm"]) not in (1400, 800):
            raise UserError("The cubicle is 1400 mm or 800 mm wide.")
        app.ui["cubicle_mm"] = int(body["cubicle_mm"])
    if body.get("theme") in ("light", "dark"):
        app.ui["theme"] = body["theme"]
    if "ip" in body and body.get("arm") in ARMS:
        app.robot.arm(body["arm"]).ip = str(body["ip"]).strip()
    app.save_ui()
    return app.state()


# ── robot (stand-in)

def _arm_arg(body, allow_all=False) -> str:
    side = body.get("arm")
    ok = ARMS + (("all", "both") if allow_all else ())
    if side not in ok:
        raise UserError("Say which arm: left or right.")
    return side


@route("POST", "/api/lite/robot/connect")
def post_connect(app, q, body, m):
    side = _arm_arg(body)
    return app.robot.connect(side, body.get("ip"))


@route("POST", "/api/lite/robot/disconnect")
def post_disconnect(app, q, body, m):
    out = app.robot.disconnect(_arm_arg(body))
    if not any(a["connected"] for a in app.robot.view().values()):
        app.set_mode("visual")
    return out


@route("POST", "/api/lite/robot/stop")
def post_stop(app, q, body, m):
    """STOP (no session): like the E-stop, a linked controller is stopped too and the arms are held
    until Recover; a controller that did not confirm it is an error."""
    out = app.robot.stop(_arm_arg(body, allow_all=True))
    if not out["ok"]:
        out["error"] = "STOP not confirmed: " + " ".join(r["text"] + "." for r in out["controllers"].values() if not r["ok"])
        out["hint"] = "Press the red E-stop button on the robot if it is still moving."
    return out


@route("POST", "/api/lite/robot/estop-test")
def post_estop_test(app, q, body, m):
    _require_engineer(app, "simulate the E-stop")
    out = app.robot.press_estop(_arm_arg(body), bool(body.get("pressed")))
    if out["estop"] != "CLEAR" and app.ui["mode"] == "real":
        app.set_mode("hwsim")
    return out


@route("POST", "/api/lite/robot/recover")
def post_recover(app, q, body, m):
    return app.robot.recover(_arm_arg(body))


@route("GET", "/api/lite/robot/programs")
def get_programs(app, q, body, m):
    side = (q.get("arm") or ["left"])[0]
    return {"arm": side, "programs": app.robot.programs(side)}


@route("POST", "/api/lite/robot/pole")
def post_pole(app, q, body, m):
    mode = _require_robot_mode(app)
    _require_confirmed(app, body, "Moving the pole", mode)
    side = _arm_arg(body)
    return app.robot.move_pole(side, float(body.get("mm", 0)), f"Pole {side.upper()} to {body.get('mm')} mm",
                               mode=mode).view()


@route("POST", "/api/lite/robot/hand")
def post_hand(app, q, body, m):
    mode = _require_robot_mode(app)
    _require_confirmed(app, body, "Moving the hand", mode)
    side = _arm_arg(body)
    return app.robot.set_hand(side, body.get("counts") or [], f"Hand {side.upper()}", mode=mode).view()


@route("POST", "/api/lite/robot/run-program")
def post_run_program(app, q, body, m):
    """Engineer: run a stored program by number (both arms together with arm='both'), staged with
    the pole and hand recorded when it was uploaded; nothing starts unless every arm can."""
    from . import staging

    mode = _require_robot_mode(app)
    _require_engineer(app, "run programs by number")
    dry = mode != "real"
    _require_confirmed(app, body, "A run", mode)
    side = _arm_arg(body, allow_all=True)
    if side == "all":
        raise UserError("Say which arm: left, right or both.")
    if side == "both":
        ids = body.get("program_ids") or {}
    else:
        ids = {side: body.get("program_id")}
    plan = {}
    for s, pid in ids.items():                      # every arm checked before any arm starts
        if s not in ARMS:
            raise UserError("Say which arm: left or right.")
        try:
            pid = int(pid)
        except (TypeError, ValueError):
            raise UserError(f"Give the {s.upper()} arm's program number.") from None
        prog = app.robot.arm(s).programs.get(pid)
        if not prog:
            raise UserError(f"Program {pid} is empty on the {s.upper()} arm.")
        stage = staging.stage_from(prog.get("stage"))
        if stage is None:
            raise UserError(f"Program {pid} has no recorded pole height and hand pose, so it cannot run.",
                            "Send it again from its region: Lite records the stage with the upload.", code="stage")
        app.robot.require_motion(s)
        plan[s] = (pid, stage)
    together = " together" if len(plan) > 1 else ""
    return {"jobs": [app.robot.run(s, pid, dry_run=dry, title=f"Program {pid} ({s.upper()}{together})", stage=st,
                                   mode=mode).view()
                     for s, (pid, st) in plan.items()]}


@route("GET", "/api/lite/robot/log")
def get_robot_log(app, q, body, m):
    side = (q.get("arm") or ["left"])[0]
    jobs = [j for j in list(app.jobs._jobs.values()) if j.arm in (side, "both")][-15:]
    lines = []
    for j in jobs:
        v = j.view()
        lines.append(f"[{v['state']:>9}] {v['title']}: {v['stage']}" + (f" — {v['error']}" if v["error"] else ""))
        lines += [f"           {e['t']:6.1f} s  {e['text']}" for e in v["events"]]
    return {"arm": side, "lines": lines}


def _require_robot_mode(app) -> str:
    """The mode a robot request is decided in (Hardware sim or Real); its job runs only in that mode."""
    mode = app.ui["mode"]
    if mode == "visual":
        raise UserError("Lite is in Visual sim, so it does not use the robot hardware.",
                        "Switch to Hardware sim or Real robot in the bar at the top.")
    return mode


@route("POST", r"/api/lite/unit/([A-Za-z0-9_\-]+)/prepare")
def post_prepare(app, q, body, m):
    key = m.group(1)
    mode = _require_robot_mode(app)
    _require_confirmed(app, body, "Moving the pole and the hand", mode)
    u = app.project.units[key]
    stage = _legacy_stage(u)
    if stage is None:
        raise UserError("This region has no pole height and hand pose in its catalogue, so Lite will not guess them.",
                        "Use the Lite UI's Prepare to set them.", code="stage")
    return app.robot.stage(u["arm"], stage, f"Prepare {key}", mode=mode).view()


@route("POST", r"/api/lite/unit/([A-Za-z0-9_\-]+)/teach/(start|stop|cancel)")
def post_teach(app, q, body, m):
    key, verb = m.group(1), m.group(2)
    u = app.project.units[key]
    side = u["arm"]
    if verb == "start":
        mode = _require_robot_mode(app)
        _require_confirmed(app, body, "Drag teaching", mode)
        return app.robot.teach_start(side, _legacy_stage(u), mode=mode)
    if verb == "cancel":
        return app.robot.teach_cancel(side)
    out = app.robot.teach_stop(side)
    secs = out["seconds"]
    if out["file"] is not None:                     # the controller's own recording (the SDK wrote it)
        path = app.project.add_recording(key, Path(out["file"]), source="teach",
                                         note=f"drag-taught on the {side.upper()} arm: {out['points']} points in {secs:.0f} s")
        return {"saved": path.name, "seconds": round(secs, 1), "unit": _unit(app, key)}
    # stand-in: a real recording of the SAME arm (never the other arm's)
    from .library import standin_take

    take = standin_take(side, key, app.project.fixture)
    path = app.project.add_recording(key, Path(take["path"]), note=f"stand-in recording ({secs:.0f} s of teaching)",
                                     source="teach")
    return {"saved": path.name, "seconds": round(secs, 1), "unit": _unit(app, key)}


@route("POST", r"/api/lite/unit/([A-Za-z0-9_\-]+)/upload")
def post_upload(app, q, body, m):
    key = m.group(1)
    _require_robot_mode(app)
    from .api_units import checked_version

    p = app.project
    if not p.current_version(key):
        raise UserError("There is no saved version to send.", "Optimise and keep a version first.")
    stage = _legacy_stage(p.units[key])
    if stage is None:
        raise UserError("This region has no pole height and hand pose in the catalogue, so it cannot run.")
    ver = checked_version(p, key, p.units[key]["arm"])
    s = p.settings(key)
    pid = body.get("program_id") or s.get("program_id")
    if not pid:
        raise UserError("This region has no program number yet.", "An engineer sets it on this screen.")
    side = p.units[key]["arm"]
    name = f"{key}_{ver['version']}"[:32]
    job = app.robot.upload(side, int(pid), Path(ver["path"]), name, ver["sha256"], bool(body.get("replace")),
                           f"Send {key} to program {pid}", stage=stage.as_dict())
    if job is None:
        p.add_result(key, "uploaded", ver["sha256"], True, {"program_id": int(pid), "unchanged": True})
        return {"unchanged": True, "program_id": int(pid)}
    def watch():
        while job.state == "running":
            time.sleep(0.2)
        if job.state == "done":
            p.add_result(key, "uploaded", ver["sha256"], True, {"program_id": int(pid)})
    threading.Thread(target=watch, daemon=True).start()
    return job.view()


@route("POST", r"/api/lite/unit/([A-Za-z0-9_\-]+)/run")
def post_run(app, q, body, m):
    key = m.group(1)
    from .api_units import checked_version

    p = app.project
    mode = _require_robot_mode(app)
    dry = mode != "real"
    _require_confirmed(app, body, "A run", mode)
    s = p.settings(key)
    side = p.units[key]["arm"]
    pid = s.get("program_id")
    if not p.current_version(key) or not pid:
        raise UserError("Send this region to the robot first.")
    ver = checked_version(p, key, side)
    prog = app.robot.arm(side).programs.get(int(pid))
    if not prog or prog.get("sha256") != ver["sha256"]:
        raise UserError(f"Program {pid} does not hold this region's current version.", "Send it to the robot again.")

    def done(out):
        p.add_result(key, "dryrun_ok" if dry else "real_ok", ver["sha256"], True, out)

    stage = _legacy_stage(p.units[key])
    if stage is None:
        raise UserError("This region has no pole height and hand pose in the catalogue, so it cannot run.")
    job = app.robot.run(side, int(pid), dry_run=dry, title=("Dry run " if dry else "Run ") + key, on_done=done,
                        stage=stage, mode=mode)
    return job.view()


def _legacy_stage(u: dict):
    """Legacy Lite's regions are staged with their catalogue pole and hand."""
    from . import staging

    hand = u.get("hand_counts") or u.get("hand_preset")
    if u.get("pole_mm") is None or not hand:
        return None
    return staging.make_stage(u["pole_mm"], hand, "the region catalogue")


@route("POST", r"/api/lite/unit/([A-Za-z0-9_\-]+)/verdict")
def post_verdict(app, q, body, m):
    key = m.group(1)
    p = app.project
    ver = p.current_version(key)
    v = body.get("verdict")
    if v not in ("good", "hit", "missed", "other"):
        raise UserError("Choose one of the answers.")
    p.add_result(key, "verdict", ver["sha256"] if ver else None, v == "good", {"verdict": v, "note": body.get("note", "")})
    return _unit(app, key)


# ───────────────────────────────────────────────────────────── HTTP

class Handler(BaseHTTPRequestHandler):
    server_version = f"ALIXStudioLite/{__version__}"
    app: LiteApp = None                                   # set by make_server

    def log_message(self, fmt, *args):                    # quiet: only errors reach the terminal
        pass

    def _send(self, code: int, body: bytes, ctype: str, cache: bool = False):
        if len(body) > 8192 and "gzip" in (self.headers.get("Accept-Encoding") or "") and not ctype.startswith(("image/",)):
            import gzip

            body = gzip.compress(body, 5)
            gz = True
        else:
            gz = False
        self.send_response(code)
        if gz:
            self.send_header("Content-Encoding", "gzip")
            self.send_header("Vary", "Accept-Encoding")
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "max-age=3600" if cache else "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, json.dumps(obj, default=str).encode("utf-8"), "application/json; charset=utf-8")

    def _host_ok(self) -> bool:
        """This computer only (no DNS rebinding), and a browser page only from Lite itself: an
        Origin on another port is another program's page."""
        host = (self.headers.get("Host") or "").split(":")[0]
        if host not in ("127.0.0.1", "localhost"):
            return False
        origin = self.headers.get("Origin")
        if origin is None:
            return True
        m = re.match(r"^http://(127\.0\.0\.1|localhost)(?::(\d+))?$", origin)
        return m is not None and int(m.group(2) or 80) == self.server.server_address[1]

    def _redirect(self, location: str):
        self.send_response(302)
        self.send_header("Location", location)
        self.send_header("Content-Length", "0")
        self.end_headers()

    def _static(self, path: str):
        if path == "/legacy":
            return self._redirect("/legacy/")
        if path == "/favicon.ico":                      # neither page has one
            self.send_response(204)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        if path in ("/", "/index.html"):
            f = WEB / "index.html"
        elif path.startswith("/legacy/"):
            f = (WEB_LEGACY / (path[len("/legacy/"):] or "index.html")).resolve()
            if not f.is_relative_to(WEB_LEGACY.resolve()):
                return self._send(404, b"not found", "text/plain")
        elif path.startswith("/vendor/") and path[8:] in VENDOR_FILES:
            f = STUDIO_VENDOR / path[8:]
        elif path.startswith("/deck/"):
            f = DECK / Path(path[6:]).name
        elif path.startswith("/mesh/") and path[6:] in MESHES:
            f = MESHES[path[6:]]
        elif path.startswith("/mesh/fixture/"):
            from . import fixtures

            parts = path[len("/mesh/fixture/"):].split("/")
            f = fixtures.mesh_path(parts[0], parts[-1]) if len(parts) == 2 and parts[0] in fixtures.FIXTURES else None
            if f is None:
                return self._send(404, b"not found", "text/plain")
        else:
            f = (WEB / path.lstrip("/")).resolve()
            if not f.is_relative_to(WEB.resolve()):
                return self._send(404, b"not found", "text/plain")
        if not f.is_file():
            return self._send(404, b"not found", "text/plain")
        ctype = mimetypes.guess_type(f.name)[0] or "application/octet-stream"
        if f.suffix == ".js":
            ctype = "text/javascript; charset=utf-8"
        if f.suffix == ".stl":
            ctype = "model/stl"
        cache = path.startswith(("/vendor/", "/deck/", "/mesh/"))
        self._send(200, f.read_bytes(), ctype, cache)

    def _send_raw(self, out: RawResponse):
        headers = [("Content-Type", out.ctype), ("Content-Length", str(len(out.data))),
                   ("Cache-Control", "no-store"), ("X-Content-Type-Options", "nosniff")]
        if out.filename:                       # an ASCII name, plus the exact one (RFC 6266 / 5987)
            ascii_name = re.sub(r"[^A-Za-z0-9._ -]", "_", out.filename).strip() or "download"
            headers.append(("Content-Disposition",
                            f"attachment; filename=\"{ascii_name}\"; filename*=UTF-8''{quote(out.filename)}"))
        self.send_response(200)
        for k, v in headers:
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(out.data)

    MAX_JSON = 8 * 1024 * 1024
    MAX_RAW = 256 * 1024 * 1024           # a project file (zip) with all its trajectories

    def _dispatch(self, method: str):
        if not self._host_ok():
            return self._send(403, b"forbidden", "text/plain")
        url = urlparse(self.path)
        if not url.path.startswith("/api/"):
            if method == "GET":
                return self._static(url.path)
            return self._send(405, b"method not allowed", "text/plain")
        body = {}
        if method in ("POST", "PUT"):
            raw_n = (self.headers.get("Content-Length") or "0").strip()
            if not raw_n.isdigit():
                return self._send(400, b"bad Content-Length", "text/plain")
            n = int(raw_n)
            ctype = (self.headers.get("Content-Type") or "").split(";")[0].strip().lower()
            if ctype in ("application/zip", "application/octet-stream"):
                if n > self.MAX_RAW:
                    return self._json({"error": "The file is too large."})
                body = {"_raw": self.rfile.read(n)}
            else:
                if n > self.MAX_JSON:
                    return self._json({"error": "Request too large."})
                try:
                    body = json.loads(self.rfile.read(n) or b"{}")
                except ValueError:
                    return self._json({"error": "The request was not valid JSON."})
                if not isinstance(body, dict):
                    return self._json({"error": "The request must be a JSON object."})
        q = parse_qs(url.query)
        REQUEST.headers = self.headers
        if not OPEN_ROUTES.match(url.path):              # everything but log-in and STOP needs the session
            from .api_v2 import session

            try:
                session(self.app)
            except UserError as exc:
                return self._json({"error": str(exc), "hint": exc.hint, "code": exc.code})
        for meth, pat, fn in ROUTES:
            mm = pat.match(url.path)
            if mm and meth == method:
                err = None
                try:
                    out = fn(self.app, q, body, mm)
                except UserError as exc:
                    out = {"error": str(exc), "hint": exc.hint}
                    if exc.code:
                        out["code"] = exc.code
                    err = out
                except KeyError as exc:
                    out = err = {"error": str(exc).strip("'\"")}
                except Exception:                                   # noqa: BLE001
                    traceback.print_exc()
                    out = err = {"error": "Something went wrong on the server.", "hint": "Details are in the terminal."}
                if method != "GET" and url.path not in ("/api/activity", "/api/audit"):   # every change and refusal (those two log themselves)
                    autosave = method == "PUT" and url.path.startswith("/api/projects/")   # the whole project, often
                    self.app.log.activity(f"{method} {url.path}", who=self._who(), detail=None if autosave else body,
                                          ok=err is None, error=err and err["error"])
                if err is not None:
                    return self._json(err)
                if isinstance(out, RawResponse):        # once sending starts, no second reply on this socket
                    return self._send_raw(out)
                return self._json(out)
        return self._json({"error": f"Unknown request {url.path}."}, 404)

    def _who(self) -> str:
        """The session behind a request, for the log: its level and a short tag (never the token)."""
        tok = self.headers.get("X-Lite-Token")
        if not tok:
            return "-"
        s = self.app.auth.session(tok)
        role = "ended" if s is None else "engineer" if s.get("engineer") else "operator"
        return f"{role}:{hashlib.sha256(tok.encode()).hexdigest()[:6]}"

    def do_GET(self):
        self._dispatch("GET")

    def do_POST(self):
        self._dispatch("POST")

    def do_PUT(self):
        self._dispatch("PUT")

    def do_DELETE(self):
        self._dispatch("DELETE")


class _Server(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        """A page that went away before its answer (a reload, a closed window) is nothing to report: the
        lab's terminal filled with BrokenPipeError tracebacks for it (17 Oct). Anything else is printed."""
        if isinstance(sys.exc_info()[1], (BrokenPipeError, ConnectionResetError, ConnectionAbortedError)):
            return
        super().handle_error(request, client_address)


def make_server(app: LiteApp, port: int) -> ThreadingHTTPServer:
    Handler.app = app
    return _Server(("127.0.0.1", int(port)), Handler)


# Routes kept in their own modules register themselves on import (after ROUTES exists).
from . import api_advanced, api_library, api_units, api_v2, api_workflow, routes_robot  # noqa: E402,F401
