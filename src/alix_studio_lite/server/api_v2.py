"""The Lite UI's API, ``/api/*``, named after the boundary the designer's handoff suggests
(its section 17); Legacy Lite keeps ``/api/lite/*``.

Every route needs the session token from ``POST /api/auth/login`` in the ``X-Lite-Token``
header, except the login itself and STOP: ``POST /api/robots/stop-both`` and
``POST /api/robots/<arm>/stop`` always work, whatever the page's state (handoff sections 21
and 24). Engineer routes also need the engineer PIN on the session.
"""
from __future__ import annotations

import json

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
            "jobs": [j.view() for j in app.jobs.running()],
            "alerts": app.log.alerts_view(), "instance": app.log.instance}


@route("POST", "/api/activity")
def post_activity(app, q, body, m):
    """The page's own steps, for the activity log (a stage, a task, an arm, a window opened)."""
    s = session(app)
    event = str(body.get("event") or "")[:80]
    if not event:
        raise UserError("Say what happened.")
    app.log.activity(f"page: {event}", who="engineer" if s.get("engineer") else "operator", detail=body.get("detail"))
    return {"ok": True}


# ── V32's audit trail (its 05 Activity Logs page): the server is the audit authority (handoff V30, V31)

AUDIT_FIELDS = ("id", "timestamp", "user", "sessionId", "projectId", "projectName", "page", "stage", "task", "arm",
                "category", "event", "details", "result", "source")


def _who_now(app) -> str:
    """The session behind this request, as the log names it (its level and a short tag, never the token)."""
    import hashlib

    tok = _token()
    if not tok:
        return "-"
    s = app.auth.session(tok)
    role = "ended" if s is None else "engineer" if s.get("engineer") else "operator"
    return f"{role}:{hashlib.sha256(tok.encode()).hexdigest()[:6]}"


@route("POST", "/api/audit")
def post_audit(app, q, body, m):
    """The page's own audit events (V32's auditLog: navigation, buttons, inputs, the 30 s heartbeat,
    log-in attempts), kept in Lite's activity log beside the server's own records. No session is
    needed: the page logs its log-in screen too. At most 200 events per call, each field cut to 400
    characters; PIN values never reach it (the page redacts them)."""
    rows = body.get("events")
    if not isinstance(rows, list):
        raise UserError("Send the events as a list.")
    who, kept = _who_now(app), 0
    for r in rows[:200]:
        if not isinstance(r, dict):
            continue
        row = {k: ("" if r.get(k) is None else str(r.get(k)))[:400] for k in AUDIT_FIELDS}
        row["category"] = (row["category"].upper() or "SYSTEM")[:20]
        row["result"] = (row["result"].upper() or "INFO")[:12]
        row["source"] = (row["source"].upper() or "FRONTEND")[:12]
        app.log.activity(f"page: {row['category']} · {row['event'][:120]}", who=who, detail={"audit": row},
                         ok=row["result"] != "ERROR", error=row["details"] if row["result"] == "ERROR" else None)
        kept += 1
    return {"ok": True, "kept": kept}


_CATEGORY = (("/stop", "SAFETY"), ("/recover", "SAFETY"), ("/estop", "SAFETY"), ("/api/auth", "AUTH"),
             ("/teach/", "TEACH"), ("/verdict", "TEACH"), ("/recordings", "TRAJECTORY"), ("/name", "TRAJECTORY"),
             ("/active", "TRAJECTORY"), ("/optimise", "TUNING"), ("/settings", "TUNING"), ("/undo", "TUNING"),
             ("/redo", "TUNING"), ("/keep", "TUNING"), ("/timeline", "SIMULATION"), ("/sequence", "SEQUENCE"),
             ("/api/robots", "ROBOT"), ("/upload", "ROBOT"), ("/run", "ROBOT"), ("/setup", "ROBOT"), ("/move", "ROBOT"),
             ("/api/projects", "PROJECT"), ("/api/library", "TRAJECTORY"))
_JOB_CATEGORY = {"optimise": "TUNING", "teach": "TEACH"}


def _audit_rows(app, t0: float, t1: float) -> list[dict]:
    """Lite's activity and hardware records as V32's audit rows, oldest first."""
    import re
    from datetime import datetime, timezone

    names = {p.get("id"): p.get("name") for p in app.store.list()}
    iso = lambda t: datetime.fromtimestamp(t, timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")  # noqa: E731
    out = []
    for r in app.log.records("activity", t0, t1):
        d = r.get("detail") if isinstance(r.get("detail"), dict) else {}
        if "audit" in d and isinstance(d["audit"], dict):                       # the page's own event, as it sent it
            row = dict(d["audit"], id=d["audit"].get("id") or f"a{r['t']}")
            row["timestamp"] = row.get("timestamp") or iso(r["t"])
            out.append(row)
            continue
        who, what = str(r.get("who") or "-"), str(r.get("what") or "")
        role, _, tag = who.partition(":")
        pid = (re.search(r"/api/projects/([^/]+)", what) or [None, None])[1]
        unit = (re.search(r"/units/([^/]+)", what) or [None, None])[1]
        event = re.sub(r"/api/projects/[^/]+/units/", "", what)
        if what.startswith("job "):
            kind = what.split()[1]
            cat = _JOB_CATEGORY.get(kind, "ROBOT")
            result = "SUCCESS" if " done:" in what else "STOPPED" if " cancelled:" in what else "ERROR"
            unit = unit or d.get("unit")
        else:
            cat = next((c for k, c in _CATEGORY if k in what), "SYSTEM")
            err = str(r.get("error") or "")
            result = "SUCCESS" if r.get("ok") else "REQUESTED" if "confirmation" in err else "WARNING"
        details = r.get("error") or ("" if r.get("detail") in (None, {}) else json.dumps(r.get("detail"), ensure_ascii=False))
        out.append(dict(id=f"s{r['t']}", timestamp=iso(r["t"]), user={"operator": "Operator", "engineer": "Engineer"}.get(role, "—"),
                        sessionId=tag or "server", projectId=pid or "", projectName=names.get(pid, "") if pid else "",
                        page="", stage="", task=unit or "", arm=d.get("arm") or "", category=cat, event=event,
                        details=details, result=result, source="BACKEND"))
    for r in app.log.records("hardware", t0, t1):
        verb = str(r.get("verb") or "")
        alert = verb.startswith("ALERT")
        result = ("ERROR" if verb == "ALERT ERROR" else "WARNING" if alert else "ERROR" if r.get("ok") is False
                  else "SUCCESS" if r.get("ok") else "INFO")
        bits = [str(r.get("text") or "")] + ([json.dumps(r["args"], ensure_ascii=False)] if r.get("args") is not None else []) \
            + ([f"ret={r['ret']}"] if r.get("ret") is not None else []) + ([f"{r['elapsed_s']} s"] if r.get("elapsed_s") is not None else [])
        out.append(dict(id=f"h{r['t']}", timestamp=iso(r["t"]), user="—", sessionId="controller", projectId="", projectName="",
                        page="", stage="", task="", arm=r.get("side") or "", category="SAFETY" if alert or "stop" in verb.lower() else "ROBOT",
                        event=verb, details=" · ".join(b for b in bits if b), result=result, source="ROBOT"))
    out.sort(key=lambda x: x["timestamp"])
    return out


@route("GET", "/api/logs")
def get_logs(app, q, body, m):
    """The audit trail for the Activity Logs page, from Lite's own logs: ``from`` and ``to`` (epoch
    seconds; the last 24 hours by default), the newest 20,000 at most."""
    import time as _time

    session(app)
    now = _time.time()
    t1 = float((q.get("to") or [now])[0])
    t0 = float((q.get("from") or [now - 86400])[0])
    rows = _audit_rows(app, t0, t1)
    return {"events": rows[-20000:], "from": t0, "to": t1, "dropped": max(0, len(rows) - 20000)}


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
