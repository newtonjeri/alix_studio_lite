"""The UGV's link to the arm side: one action, CleanFixture, as JSON over HTTP (the user, 11 Nov; the contract agreed
with the UGV side: "UGV-Arm Cleaning Link: Action Contract").

The UGV is the action client and this is the action server, with a ROS 2 action's steps and no ROS (Lite stays without
it): the UGV drives to the fixture, stops, and sends a GOAL with how far its base_link stands from the target pose; the
arm side ACCEPTS it or REJECTS it with what the UGV does next; while the cleaning runs the UGV polls the FEEDBACK; the
RESULT says how it ended and whether the UGV may drive (``safe_to_move``); the UGV may CANCEL, which is a STOP.

    GET  /link/v1/status                      mode, ready, safe_to_move, active_goal_id
    POST /link/v1/goals                       the goal -> the goal reply
    GET  /link/v1/goals/<goal_id>?since=<seq> the state, the feedback, the events from <seq>, the result once ended
    POST /link/v1/goals/<goal_id>/cancel      CANCELING, then poll for the result

Every request carries the header X-Link-Token. A malformed request, a wrong token or an unknown goal answers HTTP 400,
401 or 404; a rejected goal is HTTP 200 with ``accepted: false``.

THE OFFSET (the user, 11 Nov): the actual base_link pose in the target base_link frame (x forward, towards the fixture;
y to the robot's left; yaw counter-clockwise from above), metres and degrees. Inside +-10 mm, +-10 mm and +-1 deg the
stored plan runs as it is: the offset only gates, it is not passed into the run. Outside any of them the goal is rejected
and the UGV approaches again, three attempts a visit; re-planning for a larger offset is a later option.

THE RULES this side keeps: one goal at a time; the same goal_id sent again is the same goal once it was accepted (a
lost reply is resent, nothing starts twice), while a rejected goal was never taken, so sending it again is asking
again (RESEND_LATER means exactly that); a goal is taken only while the arm side is released for automatic running and ready; the arm side
does not stop because the UGV went quiet, it finishes and keeps the result; the last KEEP goals are kept in a file, so an
outcome can be fetched after a restart (a goal that was running when Lite stopped ends as ABORTED, ARM_RESTARTED).

This module is the protocol alone, on the standard library, with nothing of Lite's: ``Link`` talks to a BACKEND, the
thing that cleans (Lite's: ugv_link_lite.LiteBackend; a stand-in that moves nothing: tools/ugv_link_standin.py). A
backend has:

    mode() -> "auto" | "manual"
    blocked(fixture_id) -> (reject_code, detail) | None      why no goal is taken now (a third item: the axes outside)
    safe_to_move() -> (bool, detail)                         both arms at their travel pose, nothing moving
    start(goal) -> {"handle", "expected_duration_s"}         starts the cleaning, or raises Refused(code, detail)
    feedback(handle, since) -> {"stage_index", "stage_count", "stage_name", "elapsed_s", "seq", "events"}
    stop(handle)                                             STOP: the arms stop where they are

and calls ``Link.ended`` once when the cleaning ends.

THE LEDGER (the UGV side, 12 Nov: "logging offset_yaw_deg with each result, so both sides can see whether 1 deg holds in
practice"): every goal's outcome, rejected or ended, is one row given to ``record``: the offset the UGV reported, how
it was measured (``offset_source``: the UGV's "lidar", "lidar_fit") and what came of it. Lite keeps the rows in its
day's logs (ugv_link.jsonl).

FIXTURE NAMES are the UGV's cubicle names ("Cubicle 1"), typed once more on the arm side's page: they are compared
without case and without counting spaces (``fixture_key``).
"""
from __future__ import annotations

import hmac
import json
import math
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

PROTOCOL = 1
PREFIX = "/link/v1"
LIMIT_X_M, LIMIT_Y_M, LIMIT_YAW_DEG = 0.010, 0.010, 1.0     #: the user, 11 Nov: +-10 mm in X, +-10 mm in Y, +-1 deg about Z
MAX_ATTEMPTS = 3                                           #: approaches a visit to a fixture; the third is the last
BAD_M, BAD_DEG = 0.5, 30.0                                 #: an offset beyond these is not an offset (a unit mistake): BAD_GOAL
KEEP = 50                                                  #: goals kept, the newest
MAX_BODY = 64 * 1024
ENDED = ("SUCCEEDED", "CANCELED", "ABORTED")
#: what the UGV does after each rejection
ACTIONS = {"OFFSET_OUT_OF_TOLERANCE": "RETRY_APPROACH", "ATTEMPTS_EXHAUSTED": "CALL_OPERATOR", "BUSY": "RESEND_LATER",
           "ARM_NOT_READY": "CALL_OPERATOR", "MANUAL_MODE": "CALL_OPERATOR", "UNKNOWN_FIXTURE": "CALL_OPERATOR",
           "BAD_GOAL": "CALL_OPERATOR"}
ABORT_CODES = ("INSPECTION_UNDECIDED", "MECHANISM_FAILED", "MOTION_STOPPED", "ESTOP", "OPERATOR_STOP", "DEVICE_FAULT",
               "ARM_RESTARTED", "INTERNAL_ERROR")
_ID = re.compile(r"^[A-Za-z0-9_.:\-]{1,128}$")


#: a goal's own fields, as the ledger and the log carry them beside its outcome
GOAL_FIELDS = ("fixture_id", "attempt", "offset_x_m", "offset_y_m", "offset_yaw_deg", "offset_source")


def fixture_key(name) -> str:
    """A fixture name as it is compared: "Cubicle 1", "cubicle 1" and "CUBICLE  1" are one fixture."""
    return " ".join(str(name).split()).casefold()


class Refused(Exception):
    """A backend's refusal to start: a reject code of ACTIONS and its reason in plain words."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code, self.detail = code, detail


class BadGoal(Exception):
    pass


def limits() -> dict:
    return {"x_m": LIMIT_X_M, "y_m": LIMIT_Y_M, "yaw_deg": LIMIT_YAW_DEG}


def gate(x_m: float, y_m: float, yaw_deg: float) -> list:
    """The axes outside the tolerance, of "x", "y", "yaw" (none: the stored plan runs as it is)."""
    eps = 1e-9
    return [name for name, v, lim in (("x", x_m, LIMIT_X_M), ("y", y_m, LIMIT_Y_M), ("yaw", yaw_deg, LIMIT_YAW_DEG))
            if abs(v) > lim + eps]


def _number(body: dict, key: str, bound: float, unit: str) -> float:
    v = body.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise BadGoal(f"{key} must be a number of {unit}.")
    if abs(v) > bound:
        raise BadGoal(f"{key} is {v:g} {unit}: beyond {bound:g}, not an offset from the target pose (the offsets are in "
                      "metres and degrees).")
    return float(v)


def parse_goal(body: dict) -> dict:
    """The goal's fields as the contract names them, or BadGoal with what is wrong."""
    if body.get("protocol", PROTOCOL) != PROTOCOL:
        raise BadGoal(f"protocol {body.get('protocol')!r} is not this side's ({PROTOCOL}).")
    gid, fixture = body.get("goal_id"), body.get("fixture_id")
    if not isinstance(gid, str) or not _ID.match(gid):
        raise BadGoal("goal_id must be 1 to 128 letters, digits or _ . : - (a UUID).")
    if not isinstance(fixture, str) or not fixture.strip() or len(fixture) > 128:
        raise BadGoal("fixture_id must name the fixture the UGV stands at.")
    attempt = body.get("attempt", 1)
    if isinstance(attempt, bool) or not isinstance(attempt, int) or attempt < 1:
        raise BadGoal("attempt must be a whole number from 1.")
    source = body.get("offset_source", "")
    if not isinstance(source, str) or len(source) > 64:
        raise BadGoal("offset_source must be a short word for how the offset was measured.")
    return dict(goal_id=gid, fixture_id=fixture.strip(), attempt=attempt,
                offset_x_m=_number(body, "offset_x_m", BAD_M, "m"), offset_y_m=_number(body, "offset_y_m", BAD_M, "m"),
                offset_yaw_deg=_number(body, "offset_yaw_deg", BAD_DEG, "deg"), offset_source=source)


def offset_words(goal: dict, source: bool = False) -> str:
    return (f"x {goal['offset_x_m'] * 1000:+.1f} mm, y {goal['offset_y_m'] * 1000:+.1f} mm, "
            f"yaw {goal['offset_yaw_deg']:+.2f} deg" + (f", {goal['offset_source']}" if source and goal.get("offset_source") else ""))


def _outside_words(goal: dict, exceeded: list) -> str:
    said = {"x": f"X {goal['offset_x_m'] * 1000:+.1f} mm is outside {LIMIT_X_M * 1000:.0f} mm",
            "y": f"Y {goal['offset_y_m'] * 1000:+.1f} mm is outside {LIMIT_Y_M * 1000:.0f} mm",
            "yaw": f"Yaw {goal['offset_yaw_deg']:+.2f} deg is outside {LIMIT_YAW_DEG:.2f} deg"}
    return "; ".join(said[k] for k in exceeded)


class Link:
    """The action server's state: the goals and the one running. A rejected goal leaves nothing behind."""

    def __init__(self, backend, store: Path | None = None, say=None, record=None):
        self.backend = backend
        self._store = Path(store) if store else None
        self._say = say or (lambda text, **kw: None)          # the log: say(text, ok=..., detail=...)
        self._record = record or (lambda row: None)           # the ledger: record(row), one row a goal's outcome
        self._lock = threading.RLock()
        self._goals: dict = {}                               # goal_id -> its record, oldest first
        self._active: str | None = None
        self._load()

    def _note(self, gid, goal: dict | None, outcome: str, **kw) -> None:
        """One row of the ledger: goal ``gid``'s ``outcome`` (REJECTED, or how it ended) beside the offset it came with."""
        row = dict(t=round(time.time(), 3), goal_id=gid if isinstance(gid, str) else "")
        row.update({k: (goal or {}).get(k) for k in GOAL_FIELDS})
        row.update(dict(outcome=outcome, reject_code="", exceeded=[], abort_code="", safe_to_move=None, ugv_action="",
                        stages_done=0, duration_s=0.0, detail=""), **kw)
        try:
            self._record(row)
        except Exception:                                     # noqa: BLE001 - a ledger that cannot be written fails no goal
            pass

    # ── the file
    def _load(self) -> None:
        if self._store is None:
            return
        try:
            rows = json.loads(self._store.read_text(encoding="utf-8")).get("goals") or []
        except (OSError, ValueError, AttributeError):
            return
        lost = []
        for g in rows:
            if not isinstance(g, dict) or not isinstance(g.get("goal_id"), str):
                continue
            if g.get("status") not in ENDED:                  # it was running when this program stopped
                g["status"] = "ABORTED"
                g["result"] = dict(status="ABORTED", safe_to_move=False, abort_code="ARM_RESTARTED", ugv_action="CALL_OPERATOR",
                                   stages_done=int((g.get("feedback") or {}).get("stage_index") or 0), duration_s=0.0,
                                   detail="The arm program restarted while this goal was running. The arms stopped where "
                                          "they were: a person looks before anything moves.")
                lost.append(g["goal_id"])
                self._note(g["goal_id"], g, "ABORTED", **{k: g["result"][k] for k in
                                                         ("abort_code", "safe_to_move", "ugv_action", "stages_done", "detail")})
            g.pop("handle", None)
            self._goals[g["goal_id"]] = g
        if lost:
            self._save()
            self._say(f"UGV link: goal {', '.join(lost)} was running when Lite stopped: ended as ABORTED, ARM_RESTARTED", ok=False)

    def _save(self) -> None:
        if self._store is None:
            return
        rows = [{k: v for k, v in g.items() if k != "handle"} for g in list(self._goals.values())[-KEEP:]]
        try:
            self._store.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._store.with_suffix(self._store.suffix + ".tmp")
            tmp.write_text(json.dumps({"goals": rows}, indent=1), encoding="utf-8")
            os.replace(tmp, self._store)
        except OSError as exc:                                # the outcome is still held in memory
            self._say(f"UGV link: the goals file could not be written ({exc})", ok=False)

    # ── status
    def status(self) -> dict:
        mode = self.backend.mode()
        safe, why = self.backend.safe_to_move()
        with self._lock:
            active = self._active
        block = None if active else self.backend.blocked(None)
        if active:
            block = ("BUSY", f"Goal {active} is running.")
        return dict(protocol=PROTOCOL, mode=mode, ready=block is None, not_ready=block[0] if block else "",
                    detail=block[1] if block else "", safe_to_move=bool(safe), safe_to_move_detail=why,
                    active_goal_id=active or "", limits=limits(), max_attempts=MAX_ATTEMPTS)

    # ── a goal
    def _reject(self, gid, code: str, detail: str, goal: dict | None = None, exceeded: list | None = None) -> dict:
        reply = dict(protocol=PROTOCOL, goal_id=gid if isinstance(gid, str) else "", accepted=False, reject_code=code,
                     ugv_action=ACTIONS[code], exceeded=list(exceeded or []), limits=limits(), detail=detail)
        self._say(f"UGV link: goal {gid!r} rejected, {code}: {detail}" + (f" (offset {offset_words(goal, True)})" if goal else ""),
                  ok=False, detail=dict(goal or {}, reject_code=code))
        self._note(gid, goal, "REJECTED", reject_code=code, exceeded=reply["exceeded"], ugv_action=reply["ugv_action"], detail=detail)
        return reply

    def send_goal(self, body: dict) -> dict:
        """The goal reply (the contract's): accepted, or why not and what the UGV does next. Nothing moves on a rejection."""
        try:
            goal = parse_goal(body)
        except BadGoal as exc:
            return self._reject(body.get("goal_id"), "BAD_GOAL", str(exc))
        gid = goal["goal_id"]
        with self._lock:
            known = self._goals.get(gid)
            if known is not None:                             # the same id is the same goal: its reply, its state now
                return dict(known["reply"], status=known["status"])
            if self._active is not None:
                return self._reject(gid, "BUSY", f"Goal {self._active} is running: one goal at a time.", goal)
            block = self.backend.blocked(goal["fixture_id"])
            if block is not None:
                return self._reject(gid, block[0], block[1], goal, block[2] if len(block) > 2 else None)
            exceeded = gate(goal["offset_x_m"], goal["offset_y_m"], goal["offset_yaw_deg"])
            if goal["attempt"] > MAX_ATTEMPTS:
                return self._reject(gid, "ATTEMPTS_EXHAUSTED", f"Attempt {goal['attempt']}: a visit has {MAX_ATTEMPTS}.", goal,
                                    exceeded)
            if exceeded:
                last = goal["attempt"] >= MAX_ATTEMPTS
                return self._reject(gid, "ATTEMPTS_EXHAUSTED" if last else "OFFSET_OUT_OF_TOLERANCE",
                                    f"{_outside_words(goal, exceeded)}. Attempt {goal['attempt']} of {MAX_ATTEMPTS}.", goal, exceeded)
            try:
                started = self.backend.start(goal)
            except Refused as exc:
                return self._reject(gid, exc.code if exc.code in ACTIONS else "ARM_NOT_READY", exc.detail, goal)
            expected = started.get("expected_duration_s")
            reply = dict(protocol=PROTOCOL, goal_id=gid, accepted=True, reject_code="", ugv_action="HOLD", exceeded=[],
                         limits=limits(), detail="Within tolerance. Cleaning started.")
            if expected:
                reply["expected_duration_s"] = round(float(expected), 1)
            self._goals[gid] = dict(goal, received=round(time.time(), 3), status="EXECUTING", reply=reply,
                                    handle=started.get("handle"), feedback=None, result=None, cancel_requested=False)
            for k in list(self._goals)[:-KEEP]:
                self._goals.pop(k, None)
            self._active = gid
            self._save()
        self._say(f"UGV link: goal {gid} accepted for {goal['fixture_id']} (attempt {goal['attempt']}, offset "
                  f"{offset_words(goal, True)})", ok=True, detail=goal)
        return dict(reply, status="ACCEPTED")

    def goal(self, gid: str, since: int = 0) -> dict | None:
        """A goal's state, its feedback, its events from ``since`` and, once ended, its result; None: no such goal."""
        with self._lock:
            g = self._goals.get(gid)
            if g is None:
                return None
            status, handle, result, last = g["status"], g.get("handle"), g.get("result"), g.get("feedback")
        if status in ENDED or handle is None:
            fb = dict(last or {})
        else:
            try:
                fb = dict(self.backend.feedback(handle, since) or {})
            except Exception:                                 # noqa: BLE001 - a poll never fails the goal
                fb = dict(last or {})
        events = fb.pop("events", None) or []
        if status in ENDED:
            events = [e for e in (last or {}).get("events") or [] if int(e.get("seq", 0)) >= since]
        feedback = dict(status=status if status in ("EXECUTING", "CANCELING") else "",
                        stage_index=int(fb.get("stage_index") or 0), stage_count=int(fb.get("stage_count") or 0),
                        stage_name=str(fb.get("stage_name") or ""), elapsed_s=float(fb.get("elapsed_s") or 0.0),
                        seq=int(fb.get("seq") or 0))
        out = dict(protocol=PROTOCOL, goal_id=gid, status=status, seq=feedback["seq"], feedback=feedback, events=events)
        if status in ENDED:
            out["result"] = result
        return out

    def cancel(self, gid: str) -> dict | None:
        """The UGV's cancel: STOP. The arms stop where they are; the goal ends as CANCELED once they have."""
        with self._lock:
            g = self._goals.get(gid)
            if g is None:
                return None
            running = g["status"] not in ENDED
            if running:
                g["cancel_requested"], g["status"] = True, "CANCELING"
                handle = g.get("handle")
        if running:
            self._say(f"UGV link: goal {gid} cancelled by the UGV: STOP to both arms", ok=False)
            try:
                self.backend.stop(handle)
            except Exception as exc:                          # noqa: BLE001 - said, and the goal still ends with its run
                self._say(f"UGV link: STOP for goal {gid} did not go through ({type(exc).__name__}: {exc})", ok=False)
        return self.goal(gid, 10 ** 9)

    def ended(self, gid: str, outcome: str, *, abort_code: str = "", detail: str = "", stages_done: int = 0,
              duration_s: float = 0.0, safe_to_move: bool = False, feedback: dict | None = None) -> None:
        """The backend's word that goal ``gid``'s cleaning ended: ``outcome`` "done", "stopped" (STOP was pressed) or
        "failed" (with its ``abort_code``). Once a goal: a second call changes nothing."""
        with self._lock:
            g = self._goals.get(gid)
            if g is None or g["status"] in ENDED:
                return
            if outcome == "done":
                status, code = "SUCCEEDED", ""
            elif g.get("cancel_requested"):
                status, code = "CANCELED", ""
            else:
                status = "ABORTED"
                code = "OPERATOR_STOP" if outcome == "stopped" else abort_code if abort_code in ABORT_CODES else "INTERNAL_ERROR"
            g["status"] = status
            g["result"] = dict(status=status, safe_to_move=bool(safe_to_move), abort_code=code,
                               ugv_action="CONTINUE_MISSION" if safe_to_move else "CALL_OPERATOR",
                               stages_done=int(stages_done), duration_s=round(float(duration_s), 1), detail=str(detail or ""))
            if feedback is not None:
                g["feedback"] = feedback
            g.pop("handle", None)
            if self._active == gid:
                self._active = None
            self._save()
            result, goal = dict(g["result"]), {k: g.get(k) for k in GOAL_FIELDS}
        self._say(f"UGV link: goal {gid} {status}" + (f", {result['abort_code']}" if result["abort_code"] else "")
                  + f"; safe_to_move {str(result['safe_to_move']).lower()}; {result['duration_s']:.1f} s; offset "
                  + offset_words(goal, True) + (f"; {result['detail']}" if result["detail"] else ""),
                  ok=status == "SUCCEEDED", detail=dict(goal, **result))
        self._note(gid, goal, status, **{k: result[k] for k in ("abort_code", "safe_to_move", "ugv_action", "stages_done",
                                                                "duration_s", "detail")})

    def recent(self, n: int = 10) -> list:
        """The newest goals, newest first, for the page."""
        with self._lock:
            rows = list(self._goals.values())[-n:]
        return [dict(goal_id=g["goal_id"], fixture_id=g["fixture_id"], attempt=g["attempt"], received=g.get("received"),
                     status=g["status"], offset=offset_words(g, True), result=g.get("result")) for g in reversed(rows)]

    @property
    def active(self) -> str | None:
        with self._lock:
            return self._active


# ───────────────────────────────────────────────────────────── HTTP

class _Handler(BaseHTTPRequestHandler):
    server_version = "ALIXArmLink/1"
    link: Link = None
    token: str = ""
    extra = None                 # a stand-in's own controls: extra(method, path, body) -> its answer, or None (not its)

    def log_message(self, fmt, *args):
        pass

    def _json(self, obj: dict, code: int = 200) -> None:
        body = json.dumps(obj, default=str).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _error(self, code: int, text: str) -> None:
        self._json({"protocol": PROTOCOL, "error": text}, code)

    def _dispatch(self, method: str) -> None:
        url = urlparse(self.path)
        got = self.headers.get("X-Link-Token") or ""
        if not self.token or not hmac.compare_digest(got.encode("utf-8"), self.token.encode("utf-8")):
            return self._error(401, "The X-Link-Token header is missing or is not this arm side's token.")
        path = url.path.rstrip("/")
        own = type(self).extra
        if not path.startswith(PREFIX) and own is None:
            return self._error(404, f"Unknown path {url.path}: the link is under {PREFIX}.")
        rest = path[len(PREFIX):]
        body = {}
        if method == "POST":
            raw_n = (self.headers.get("Content-Length") or "0").strip()
            if not raw_n.isdigit() or int(raw_n) > MAX_BODY:
                return self._error(400, "Bad or too large Content-Length.")
            try:
                body = json.loads(self.rfile.read(int(raw_n)) or b"{}")
            except ValueError:
                return self._error(400, "The request body is not JSON.")
            if not isinstance(body, dict):
                return self._error(400, "The request body must be a JSON object.")
        try:
            if not path.startswith(PREFIX):
                try:
                    out = own(method, path, body)
                except ValueError as exc:                     # a setting it does not take: said, as a bad request
                    return self._error(400, str(exc))
                if out is None:
                    return self._error(404, f"Unknown path {url.path}: the link is under {PREFIX}.")
                return self._json(out)
            if method == "GET" and rest == "/status":
                return self._json(self.link.status())
            if method == "POST" and rest == "/goals":
                return self._json(self.link.send_goal(body))
            m = re.match(r"^/goals/([A-Za-z0-9_.:\-]{1,128})(/cancel)?$", rest)
            if m and ((method == "GET" and not m.group(2)) or (method == "POST" and m.group(2))):
                if m.group(2):
                    out = self.link.cancel(m.group(1))
                else:
                    try:
                        since = max(0, int((parse_qs(url.query).get("since") or ["0"])[0]))
                    except ValueError:
                        return self._error(400, "since must be a whole number.")
                    out = self.link.goal(m.group(1), since)
                if out is None:
                    return self._error(404, f"No goal {m.group(1)} is known here.")
                return self._json(out)
        except Exception as exc:                              # noqa: BLE001 - the UGV always gets an answer
            import traceback

            traceback.print_exc()
            return self._error(500, f"The arm side failed on this request ({type(exc).__name__}).")
        self._error(404 if method in ("GET", "POST") else 405, f"Unknown request {method} {url.path}.")

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
    allow_reuse_address = True

    def handle_error(self, request, client_address):          # a client that went away mid-answer is nothing to report
        import sys

        if isinstance(sys.exc_info()[1], (BrokenPipeError, ConnectionResetError, ConnectionAbortedError)):
            return
        super().handle_error(request, client_address)


def make_server(link: Link, host: str, port: int, token: str, extra=None) -> ThreadingHTTPServer:
    """The link's own listener: only these routes, only with ``token`` (``extra``: a stand-in's controls beside them,
    behind the same token; Lite gives none)."""
    handler = type("LinkHandler", (_Handler,), {"link": link, "token": str(token),
                                                "extra": None if extra is None else staticmethod(extra)})
    return _Server((host, int(port)), handler)


def serve(link: Link, host: str, port: int, token: str) -> ThreadingHTTPServer:
    """``make_server`` served on a thread of its own; returns the server (``shutdown()`` ends it)."""
    srv = make_server(link, host, port, token)
    threading.Thread(target=lambda: srv.serve_forever(poll_interval=0.25), name="lite-ugv-link", daemon=True).start()
    return srv


def read_token(path: Path) -> str:
    """The link's token in ``path``, made at the first start (readable by this user only)."""
    import secrets

    path = Path(path)
    try:
        tok = path.read_text(encoding="utf-8").strip()
        if tok:
            return tok
    except OSError:
        pass
    tok = secrets.token_urlsafe(24)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(tok + "\n", encoding="utf-8")
    try:
        os.chmod(path, 0o600)
    except OSError:
        pass
    return tok
