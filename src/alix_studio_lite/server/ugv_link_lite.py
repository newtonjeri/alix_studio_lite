"""Lite behind the UGV link (ugv_link: the CleanFixture action; the user, 11 Nov): what a goal runs, when one is taken,
and what its end says.

MANUAL AND AUTOMATIC. Lite starts in manual mode, as it always has: the page starts everything and the UGV's goals are
rejected (MANUAL_MODE). An operator RELEASES S2's sequence for automatic running from the page: the release passes every
gate a run from the page passes (workflow.run_real, with the same confirmations, and starts nothing), keeps the run's
identity (its rows, its Programs, its points, the speed) and takes the arms' pose at that moment as the TRAVEL POSE, the
pose the UGV drives with. From then on a goal starts that sequence on both arms without a click. The release ends when
the operator ends it, when the mode leaves Real, when Lite stops, and when a goal finds the sequence no longer as it
was released (edited, a Program sent again, the speed moved): nothing runs that a person did not release.

IT ALSO ENDS WHENEVER A PERSON HAS TO STEP IN: STOP (the page's, or the UGV's cancel), an E-stop, and a goal that ends
with an arm away from the travel pose. Someone is then at the robot putting the arms back, and neither the arms nor the
UGV may start again by themselves once they are: "nothing restarts by itself" (the E-stop's own rule), a gate, not a
note (the owner, 26 Oct). The person releases the sequence again when the cell is clear, the arms parked. A goal that
ends aborted with the arms back at the travel pose by themselves (an inspection that could not decide) needs no one:
the release holds.

WHAT A GOAL RUNS: S2's sequence on both arms, as RUN BOTH runs it, with its own gates again at that moment. The UGV's
offset is not passed into the run: inside the tolerance the plans run as they were made (the user, 11 Nov), so it only
gates (ugv_link.gate).

SAFE TO MOVE: both arms within TRAVEL_TOL_DEG of the travel pose on every joint, their readings fresh, nothing running
on them. Not ALIX's rest_pose: the lab's sequence starts and ends at its own taught pose, 45 to 58 degrees from it, and
ends within 0.05 degrees of where it started (37 runs, 4 to 7 Nov). Read from the controllers, never assumed: after a
run that stopped the arms stay where they stopped (Lite never moves them back by itself) and the UGV is told to stay.

HOW A RUN'S END IS SAID: done -> SUCCEEDED; STOP from the page -> ABORTED, OPERATOR_STOP (the UGV's own cancel:
CANCELED, ugv_link); a failure by what it was (``abort_code``).
"""
from __future__ import annotations

import threading
import time

from . import ugv_link, workflow
from .api_units import P, units_for, units_of
from .api_v2 import session
from .app import route
from .jobs import UserError
from .robot_standin import ARMS, CONTROLLER_KINDS

TRAVEL_TOL_DEG = 3.0       #: a joint this far from the travel pose is not at it (a crept wrist reads up to 1.1 deg: 26 Oct)
SETTLE_S = 2.0             #: how long a run's end waits for the readings to show the arms at the travel pose
EVENTS_KEPT = 200          #: a goal's last events kept with its result
#: a run's refusals that mean the release no longer holds (the sequence is not what a person released): it ends
RELEASE_ENDS = ("changed", "unsimulated", "warnings", "confirm", "mode", "contact", "place", "stage")


def abort_code(job) -> str:
    """Why a run that failed did, as the contract's abort codes."""
    text, code = str(job.error or ""), getattr(job, "code", None)
    if "E-STOP" in text:
        return "ESTOP"
    if code == "inspect" or (code == "escalate" and "cannot judge" in text):
        return "INSPECTION_UNDECIDED"
    if code == "escalate":
        return "MECHANISM_FAILED"
    if code in ("camera", "hot") or "does not answer" in text:
        return "DEVICE_FAULT"
    if job.hint == "Details are in the server log.":           # an exception that was not a refusal (jobs.py)
        return "INTERNAL_ERROR"
    return "MOTION_STOPPED"


def fixture_names(value) -> list:
    """The fixture names a release is for, from the page's field ("Cubicle 1, Cubicle 2": the UGV's cubicle names, the
    UGV side, 12 Nov) or a list; none: any. A name typed twice, in whatever case or spacing, is one name."""
    parts = value if isinstance(value, list) else str(value or "").replace(";", ",").split(",")
    names, seen = [], set()
    for p in parts:
        name = " ".join(str(p).split())
        if name and ugv_link.fixture_key(name) not in seen:
            if len(name) > 128:
                raise UserError("A fixture name is at most 128 characters.")
            seen.add(ugv_link.fixture_key(name))
            names.append(name)
    return names


class LiteBackend:
    """ugv_link.Link's backend on Lite's robot, and the release the page gives."""

    def __init__(self, app):
        self.app = app
        self.link: ugv_link.Link | None = None                # set by LiteApp, with this as its backend
        self.listening: dict | None = None                    # {"host", "port"} or {"error"}: the link's own listener (--link)
        self.release: dict | None = None
        self.ended_why: str | None = None                     # why the last release ended, for the page
        self._goal_of: dict = {}                              # a run's job id -> its goal id
        self._stopped: str | None = None                      # STOP or an E-stop press since the last look (person_stopped)
        self._lock = threading.RLock()

    # ── the release
    def _release(self) -> dict | None:
        """The release, if it still holds: it ends here when the mode has left Real, and when STOP or an E-stop holds
        the arms (looked at with every request, and twice a second by LiteApp's status loop: ``watch``)."""
        with self._lock:
            rel = self.release
        stopped, self._stopped = self._stopped, None
        if rel is None:
            return None
        if stopped:                                           # even one recovered since: a person releases again
            self.end_release(f"{stopped}: a person releases the sequence again once the cell is clear")
            return None
        if self.app.ui["mode"] != "real":
            self.end_release("the mode left Real robot")
            return None
        robot = self.app.robot
        robot.motion_block()                                  # the arms' E-stop states as their controllers read them now
        for side, a in robot.arms.items():                    # pressed, or held until Recover: a person stepped in (an
            if a.connected and a.estop in ("ENGAGED", "RECOVERING"):   # E-stop that cannot be read only blocks: ARM_NOT_READY)
                what = "was stopped by STOP" if a.stop_latched and a.estop == "RECOVERING" else "had its E-stop pressed"
                self.end_release(f"the {side.upper()} arm {what}: a person releases the sequence again once the cell is clear")
                return None
        return rel

    def person_stopped(self, why: str) -> None:
        """The robot's word that STOP or an E-stop press holds the arms (robot_standin._person_stopped), from whatever
        thread and under whatever lock: only noted here, the release ends at the next look (``_release``)."""
        self._stopped = why

    def watch(self) -> None:
        """Twice a second (LiteApp._status_loop): a STOP or an E-stop pressed and recovered between two of the UGV's
        requests still ends the release."""
        self._release()

    def joints(self, side: str) -> tuple:
        """(the arm's 7 joints in degrees, "") as its controller last read them, or (None, why not)."""
        robot = self.app.robot
        link = robot.links.get(side)
        if link is None or not robot.arms[side].connected:
            return None, f"the {side.upper()} arm is not connected to its controller"
        fb = link.view()["feedback"]
        q = fb.get("joints_deg")
        if not q or len(q) < 7 or fb.get("stale"):
            return None, f"the {side.upper()} arm's joints are not being read"
        return [float(v) for v in q[:7]], ""

    def release_now(self, units, pid: str, body: dict) -> dict:
        """S2's sequence released for automatic running, with the confirmations a run from the page takes (``body``:
        unsimulated_ok, warnings_seen, confirmed; refused with the run's own codes) and the fixture names it is for."""
        names = fixture_names(body.get("fixtures"))
        if self.link is not None and self.link.active:
            raise UserError(f"The UGV's goal {self.link.active} is running.", "Release again once it has ended.", code="busy")
        got = workflow.run_real(self.app, units, "both", None, body, check_only=True)
        travel = {}
        for side in ARMS:
            q, why = self.joints(side)
            if q is None:
                raise UserError(f"Not released: {why}, so the travel pose cannot be taken.", "Connect both arms, then release again.")
            travel[side] = [round(v, 3) for v in q]
        rel = dict(pid=pid, fixtures=names, hash=got["hash"], simulated=bool(got["simulated"]), stages=int(got["stages"]),
                   seconds=got.get("seconds"), travel=travel, at=time.strftime("%Y-%m-%d %H:%M:%S"),
                   body={k: body[k] for k in ("unsimulated_ok", "warnings_seen", "record_feed") if body.get(k) is not None})
        with self._lock:
            self._stopped = None                              # every gate passed just now: nothing holds the arms
            self.release, self.ended_why = rel, None
        self.app.log.activity("UGV link: the sequence released for automatic running (" + (", ".join(names) or "any fixture")
                              + f"; {rel['stages']} stage(s)" + ("" if rel["simulated"] else "; not simulated, as confirmed") + ")",
                              detail=dict(pid=pid, fixtures=names, hash=rel["hash"], travel=travel))
        return self.view()

    def end_release(self, why: str) -> None:
        with self._lock:
            had, self.release = self.release, None
            if had is not None:
                self.ended_why = why
        if had is not None:
            self.app.log.activity(f"UGV link: the release ended ({why}): manual mode", ok=True)

    def view(self) -> dict:
        """What the page shows of the link."""
        rel = self._release()
        safe, why = self.safe_to_move()
        return dict(listening=self.listening, mode="auto" if rel else "manual", ended_why=self.ended_why,
                    release=None if rel is None else {k: rel[k] for k in ("pid", "fixtures", "simulated", "stages", "seconds", "at")},
                    safe_to_move=safe, safe_to_move_detail=why, active_goal_id=(self.link.active if self.link else None) or "",
                    goals=self.link.recent(6) if self.link else [], limits=ugv_link.limits(), max_attempts=ugv_link.MAX_ATTEMPTS,
                    travel_tol_deg=TRAVEL_TOL_DEG)

    # ── ugv_link's backend
    def mode(self) -> str:
        return "auto" if self._release() else "manual"

    def safe_to_move(self) -> tuple:
        rel = self._release()
        if rel is None:
            return False, ("The arm side is in manual mode" + (f" ({self.ended_why})" if self.ended_why else "")
                           + ": no sequence is released, so no travel pose is set.")
        busy = self.app.robot.busy()
        if busy:
            return False, f"Something is running on the arms: {busy}."
        for side in ARMS:
            q, why = self.joints(side)
            if q is None:
                return False, f"{why[0].upper()}{why[1:]}."
            off = [abs(a - b) for a, b in zip(q, rel["travel"][side])]
            j = max(range(7), key=lambda i: off[i])
            if off[j] > TRAVEL_TOL_DEG:
                return False, (f"The {side.upper()} arm's joint {j + 1} is {off[j]:.1f} deg from the travel pose "
                               f"(within {TRAVEL_TOL_DEG:g} deg counts as at it).")
        return True, "Both arms are at the travel pose and nothing is running."

    def blocked(self, fixture_id) -> tuple | None:
        rel = self._release()
        if rel is None:
            return ("MANUAL_MODE", "No sequence is released for automatic running: the arm side is in manual mode"
                    + (f" ({self.ended_why})" if self.ended_why else "") + ".")
        if (fixture_id is not None and rel["fixtures"]
                and ugv_link.fixture_key(fixture_id) not in {ugv_link.fixture_key(f) for f in rel["fixtures"]}):
            return ("UNKNOWN_FIXTURE", f"No sequence is released for fixture {fixture_id!r}: the release is for "
                    + ", ".join(rel["fixtures"]) + ".")
        robot = self.app.robot
        busy = robot.busy() or next((j.title for j in self.app.jobs.running() if j.kind in CONTROLLER_KINDS), None)
        if busy:
            return ("BUSY", f"The arms are busy: {busy}.")
        try:
            robot.require_motion("both")
        except UserError as exc:
            return ("BUSY" if exc.code == "busy" else "ARM_NOT_READY", str(exc))
        safe, why = self.safe_to_move()
        return None if safe else ("ARM_NOT_READY", why)

    def start(self, goal: dict) -> dict:
        rel = self._release()
        if rel is None:
            raise ugv_link.Refused("MANUAL_MODE", "No sequence is released for automatic running.")
        with self._lock:                 # held until the run's goal is noted: a run that ends at once finds it (job_ended)
            try:
                job = workflow.run_real(self.app, units_for(self.app, rel["pid"]), "both", None,
                                        dict(rel["body"], confirmed=True), expect=rel["hash"])
            except UserError as exc:
                if exc.code in RELEASE_ENDS:
                    self.end_release(str(exc))
                    raise ugv_link.Refused("MANUAL_MODE", f"The release no longer holds: {exc}") from None
                raise ugv_link.Refused("BUSY" if exc.code == "busy" else "ARM_NOT_READY", str(exc)) from None
            self._goal_of[job.id] = goal["goal_id"]
        return dict(handle=job.id, expected_duration_s=rel.get("seconds"))

    def feedback(self, handle, since: int = 0) -> dict:
        job = self.app.jobs.get(handle)
        if job is None:
            return {}
        v = job.view(since)
        n = int(v.get("n") or 0)
        return dict(stage_index=int(v.get("i") or 0) + 1 if n else 0, stage_count=n,
                    stage_name=str((v.get("extra") or {}).get("status") or v.get("stage") or ""), elapsed_s=v["elapsed_s"],
                    seq=len(job.events), events=[dict(seq=e["seq"], t=e["t"], text=e["text"], level=e["level"]) for e in v["events"]])

    def stop(self, handle) -> None:
        self.app.robot.stop("both")

    def job_ended(self, job) -> None:
        """Every job's end comes here (LiteApp._job_ended); a goal's run ends its goal."""
        with self._lock:
            gid = self._goal_of.pop(job.id, None)
        if gid is None or self.link is None:
            return
        v = job.view(0)
        outcome = "done" if v["state"] == "done" else "stopped" if v["state"] == "cancelled" else "failed"
        fb = self.feedback(job.id, 0)
        fb["events"] = fb["events"][-EVENTS_KEPT:]
        done = int((v.get("result") or {}).get("stages") or fb["stage_count"]) if outcome == "done" else max(0, fb["stage_index"] - 1)
        t0 = time.monotonic()
        safe, why = self.safe_to_move()                        # a STOP or an E-stop ended the release: not safe, said
        while not safe and self._release() is not None and time.monotonic() - t0 < SETTLE_S:
            time.sleep(0.2)                                    # the readings a moment behind the arms
            safe, why = self.safe_to_move()
        detail = v.get("error") or ("" if outcome == "done" else "Stopped by STOP on the arm side.")
        if not safe:
            detail = (detail + " " if detail else "The sequence completed. ") + why
        self.link.ended(gid, outcome, abort_code=abort_code(job) if outcome == "failed" else "", detail=detail,
                        stages_done=done, duration_s=v["elapsed_s"], safe_to_move=safe, feedback=fb)
        if not safe:                                           # a person steps in: nothing starts again by itself
            self.end_release(f"goal {gid} ended with the arms not at the travel pose: a person puts them back and "
                             "releases the sequence again")


# ───────────────────────────────────────────────────────────── the page's routes

@route("GET", "/api/link")
def get_link(app, q, body, m):
    """The UGV link as the page shows it: listening or not, manual or released, the last goals."""
    return app.ugv.view()


@route("POST", P + "/workflow/release")
def post_release(app, q, body, m):
    """S2's sequence released for automatic running (the same confirmations as a run on the robot)."""
    return app.ugv.release_now(units_of(app, m.group(1)), m.group(1), body)


@route("POST", "/api/link/release/end")
def post_release_end(app, q, body, m):
    """Back to manual mode: the UGV's goals are rejected. A goal already running goes on to its end."""
    session(app)
    app.ugv.end_release("ended by the operator")
    return app.ugv.view()
