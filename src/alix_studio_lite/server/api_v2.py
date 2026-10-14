"""The Lite UI's API, ``/api/*``, named after the boundary the designer's handoff suggests
(its section 17); Legacy Lite keeps ``/api/lite/*``.

Every route needs the session token from ``POST /api/auth/login`` in the ``X-Lite-Token``
header, except the login itself and STOP: ``POST /api/robots/stop-both`` and
``POST /api/robots/<arm>/stop`` always work, whatever the page's state (handoff sections 21
and 24). Engineer routes also need the engineer PIN on the session.
"""
from __future__ import annotations

from . import fixtures
from .app import MODE_TEXT, REQUEST, RawResponse, route
from .jobs import UserError
from .robot_standin import ARMS


def _token() -> str | None:
    headers = getattr(REQUEST, "headers", None)
    return headers.get("X-Lite-Token") if headers is not None else None


def session(app, engineer: bool = False) -> dict:
    s = app.auth.session(_token())
    if s is None:
        raise UserError("Please log in again.", "Your session has ended.", code="login")
    if engineer and not s["engineer"]:
        raise UserError("Advanced Robot Controls are locked.", "Enter the engineer PIN.", code="engineer")
    return s


# ── sign-in

@route("POST", "/api/auth/login")
def login(app, q, body, m):
    token = app.auth.login(str(body.get("pin", "")))
    return {"token": token, "role": "operator", "default_pin": app.auth.is_default("operator")}


@route("POST", "/api/auth/logout")
def logout(app, q, body, m):
    app.auth.logout(_token())
    return {"ok": True}


@route("POST", "/api/auth/engineer")
def engineer_unlock(app, q, body, m):
    app.auth.elevate(_token(), str(body.get("pin", "")))
    return {"engineer": True, "default_pin": app.auth.is_default("engineer")}


@route("POST", "/api/auth/engineer/lock")
def engineer_lock(app, q, body, m):
    app.auth.drop_engineer(_token())
    return {"engineer": False}


# ── projects

@route("GET", "/api/projects")
def list_projects(app, q, body, m):
    session(app)
    return {"projects": app.store.list(), "last_id": app.store.last_id,
            "folder": str(app.store.root)}


@route("POST", "/api/projects")
def create_project(app, q, body, m):
    session(app)
    from .store import new_id

    rec = dict(body.get("project") or body)
    pid = rec.get("id") or new_id()
    if app.store.exists(pid):
        raise UserError("A project with this id already exists.")
    fixture = body.get("fixture") or rec.pop("fixture", None)
    if fixture is not None and fixture not in fixtures.FIXTURES:
        raise UserError(f"Unknown fixture {fixture!r}.")
    out = app.store.put(pid, rec, fixture=fixture)
    app.store.set_last(pid)
    return out


@route("GET", r"/api/projects/([A-Za-z0-9_\-]+)")
def get_project(app, q, body, m):
    session(app)
    return app.store.get(m.group(1))


@route("PUT", r"/api/projects/([A-Za-z0-9_\-]+)")
def put_project(app, q, body, m):
    """Create or update a project; ``?fixture=`` sets a new project's fixture (D29)."""
    session(app)
    pid = m.group(1)
    fixture = (q.get("fixture") or [None])[0]
    if fixture is not None and fixture not in fixtures.FIXTURES:
        raise UserError(f"Unknown fixture {fixture!r}.")
    out = app.store.put(pid, body.get("project") if isinstance(body.get("project"), dict) else body, fixture=fixture)
    if body.get("last", True):
        app.store.set_last(pid)
    return out


@route("DELETE", r"/api/projects/([A-Za-z0-9_\-]+)")
def delete_project(app, q, body, m):
    session(app)
    return {"deleted": m.group(1), "moved_to": app.store.delete(m.group(1))}


@route("POST", r"/api/projects/([A-Za-z0-9_\-]+)/duplicate")
def duplicate_project(app, q, body, m):
    session(app)
    return app.store.duplicate(m.group(1), body.get("name"))


@route("GET", r"/api/projects/([A-Za-z0-9_\-]+)/export")
def export_project(app, q, body, m):
    session(app)
    rec = app.store.get(m.group(1))
    safe = "".join(c if c.isalnum() or c in "_-" else "_" for c in str(rec.get("name") or "ALIX_Project")).strip("_")
    return RawResponse(app.store.export_zip(m.group(1)), "application/zip", (safe or "ALIX_Project") + ".alixproj")


@route("POST", "/api/projects/import")
def import_project(app, q, body, m):
    session(app)
    data = body.get("_raw")
    if data is None:
        content = body.get("content")
        if not isinstance(content, str):
            raise UserError("Choose a project file to load.")
        data = content.encode("utf-8")
    name = (q.get("name") or [body.get("name") or ""])[0]
    return app.store.import_file(data, name)


# ── robots

@route("GET", "/api/robots/status")
def robots_status(app, q, body, m):
    session(app)
    return {"mode": app.ui["mode"], "mode_text": MODE_TEXT[app.ui["mode"]], "arms": app.robot.view(),
            "backend": app.robot.backend, "label": app.robot.label, "motion_block": app.robot.motion_block(),
            "jobs": [j.view() for j in app.jobs.running()]}


@route("POST", "/api/robots/stop-both")
def stop_both(app, q, body, m):
    """STOP: no session needed, so it works from any page state (handoff sections 21 and 24). It
    acts like the E-stop: linked controllers are stopped too, and the arms are held until Recover."""
    return stop_reply(app.robot.stop("all"), ["left", "right"])


@route("POST", r"/api/robots/(left|right)/stop")
def stop_arm(app, q, body, m):
    return stop_reply(app.robot.stop(m.group(1)), [m.group(1)])


def stop_reply(out: dict, sides: list) -> dict:
    """What STOP did, for the page's line; a controller that did not confirm it is an error."""
    reply = {"stopped": sides, "result": out}
    if out.get("text"):
        reply["message"] = out["text"]
    if not out.get("ok", True):
        reply["error"] = "STOP not confirmed: " + " ".join(r["text"] + "." for r in out["controllers"].values() if not r["ok"])
    return reply


# ── reference data

@route("GET", "/api/fixtures")
def get_fixtures(app, q, body, m):
    session(app)
    return {"fixtures": fixtures.list_fixtures(), "default": fixtures.DEFAULT, "arms": list(ARMS)}
