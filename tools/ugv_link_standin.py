#!/usr/bin/env python3
"""A stand-in for the arm side of the UGV link, for the UGV side to develop its client against: it answers the
contract's calls (server/ugv_link.py: status, goal, feedback, result, cancel) with the arm side's own code and moves
nothing. No robot, no Lite: only Python's standard library and one file of this repository.

    python3 tools/ugv_link_standin.py --port 8780
    python3 tools/ugv_link_standin.py --port 8780 --seconds 30 --outcomes succeeded,aborted:MECHANISM_FAILED,unsafe
    python3 tools/ugv_link_standin.py --port 8780 --outcomes reject:ARM_NOT_READY,reject:OFFSET_OUT_OF_TOLERANCE,succeeded
    python3 tools/ugv_link_standin.py --host 0.0.0.0 --port 8780 --token my-token --fixtures "Cubicle 1,Cubicle 2"

    --seconds S     how long a "cleaning" takes (default 20)
    --stages N      how many stages it reports (default 9)
    --outcomes ...  what the goals get, in turn, then again from the first (default: succeeded). EVERY REJECT CODE AND
                    EVERY ABORT CODE CAN BE ASKED FOR (the UGV side, 12 Nov):
                      succeeded            accepted; SUCCEEDED, safe_to_move true
                      unsafe               accepted; SUCCEEDED, but an arm not back at its travel pose: safe_to_move false
                      aborted:<CODE>       accepted; ABORTED with that abort code (INSPECTION_UNDECIDED, MECHANISM_FAILED,
                                           MOTION_STOPPED, ESTOP, OPERATOR_STOP, DEVICE_FAULT, ARM_RESTARTED,
                                           INTERNAL_ERROR), the arms where they stopped: safe_to_move false
                      aborted-safe:<CODE>  accepted; ABORTED with the arms back at the travel pose: safe_to_move true
                      reject:<CODE>        REJECTED with that reject code (OFFSET_OUT_OF_TOLERANCE, ATTEMPTS_EXHAUSTED,
                                           BUSY, ARM_NOT_READY, MANUAL_MODE, UNKNOWN_FIXTURE, BAD_GOAL) whatever the goal
                                           says; nothing "runs". The two offset codes name the axis as outside:
                                           yaw, or the ones given, reject:OFFSET_OUT_OF_TOLERANCE:x+y
    --recover S     after an end with safe_to_move false (an abort, "unsafe", a cancel) the arm side is in MANUAL MODE,
                    as Lite is: a person has to step in, and nothing starts again by itself. Goals are rejected
                    (MANUAL_MODE) until "the person has put the arms back and released the sequence again", S seconds
                    later (default 30)
    --manual        start in manual mode and stay in it: every goal is rejected (MANUAL_MODE)
    --fixtures a,b  the fixture names goals are taken for, compared without case (default: any)
    --store FILE    keep the goals in FILE, as Lite does: a goal running when this program is stopped ends as ABORTED,
                    ARM_RESTARTED, and is answered so after the next start
    --token T       the X-Link-Token the client must send (default: standin-token)

Without --outcomes the codes still come the way the arm side gives them: an offset outside +-10 mm, +-10 mm, +-1 deg is
OFFSET_OUT_OF_TOLERANCE (ATTEMPTS_EXHAUSTED on attempt 3), a second goal while one runs is BUSY, a malformed goal is
BAD_GOAL, a name not in --fixtures is UNKNOWN_FIXTURE. The gate, the three attempts, one goal at a time, the same
goal_id as the same goal and the cancel (the goal ends CANCELED, safe_to_move false) are the arm side's own, unchanged.

WHILE IT RUNS, a test can set it without a restart, with the same token (these two paths are the stand-in's own, not
the contract's; Lite has none):

    GET  /standin    what it is set to, and which outcome is next
    POST /standin    {"outcomes": "reject:BUSY,succeeded"}   the outcomes from now on, from the first
                     {"manual": true} / {"manual": false}    manual mode; false is also "released again" at once
                     {"seconds": 5, "recover": 2, "fixtures": "Cubicle 1,Cubicle 2"}

    curl -H 'X-Link-Token: standin-token' -d '{"outcomes": "aborted:ESTOP"}' http://127.0.0.1:8780/standin

Away from a checkout this needs two files side by side: this one and src/alix_studio_lite/server/ugv_link.py.
"""
from __future__ import annotations

import argparse
import importlib.util
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent


def load_link():
    """server/ugv_link.py by its file (beside this one, or in the checkout): nothing else of Lite is imported."""
    for path in (HERE / "ugv_link.py", HERE.parent / "src" / "alix_studio_lite" / "server" / "ugv_link.py"):
        if path.is_file():
            spec = importlib.util.spec_from_file_location("ugv_link", path)
            mod = importlib.util.module_from_spec(spec)
            sys.modules["ugv_link"] = mod
            spec.loader.exec_module(mod)
            return mod
    raise SystemExit("ugv_link.py is not beside this file nor in src/alix_studio_lite/server/: copy it here.")


ugv_link = load_link()
AXES = ("x", "y", "yaw")
OFFSET_CODES = ("OFFSET_OUT_OF_TOLERANCE", "ATTEMPTS_EXHAUSTED")


def parse_outcomes(text) -> list:
    """``--outcomes`` as (kind, code, axes) items; ValueError with what is wrong."""
    out = []
    for item in str(text).split(","):
        kind, _, rest = item.strip().partition(":")
        code, _, axes = rest.partition(":")
        if kind in ("succeeded", "unsafe") and not rest:
            out.append((kind, "", ()))
        elif kind in ("aborted", "aborted-safe") and code in ugv_link.ABORT_CODES and not axes:
            out.append((kind, code, ()))
        elif kind == "reject" and code in ugv_link.ACTIONS and (not axes or code in OFFSET_CODES):
            named = tuple(a for a in AXES if a in axes.split("+")) if axes else ("yaw",) if code in OFFSET_CODES else ()
            if axes and len(named) != len(axes.split("+")):
                raise ValueError(f"outcomes: {item!r}: the axes are x, y, yaw, joined by + (reject:{code}:x+yaw).")
            out.append((kind, code, named))
        else:
            raise ValueError(f"outcomes: {item!r} is not succeeded, unsafe, aborted:<CODE>, aborted-safe:<CODE> or "
                             f"reject:<CODE> (abort codes: {', '.join(ugv_link.ABORT_CODES)}; reject codes: "
                             f"{', '.join(ugv_link.ACTIONS)}).")
    return out


def outcome_words(item) -> str:
    kind, code, axes = item
    return kind + (f":{code}" if code else "") + (f":{'+'.join(axes)}" if axes and axes != ("yaw",) else "")


def names(value) -> list:
    parts = value if isinstance(value, list) else str(value or "").split(",")
    return [" ".join(str(p).split()) for p in parts if str(p).strip()]


class Standin:
    """A backend that cleans nothing: each goal gets the next of ``outcomes``; an accepted one "runs" for ``seconds``."""

    def __init__(self, seconds: float, stages: int, outcomes: list, recover_s: float, manual: bool, fixtures: list):
        self.seconds, self.stages, self.outcomes, self.recover_s = float(seconds), int(stages), outcomes, float(recover_s)
        self.manual, self.fixtures = manual, fixtures
        self.link = None
        self._n = 0
        self._runs: dict = {}
        self._manual_until, self._manual_why = 0.0, ""      # after an end with the arms out: until "released again"
        self._lock = threading.Lock()

    # ── the arm side's state
    def _stepped_in(self) -> bool:
        return time.time() < self._manual_until

    def mode(self):
        return "manual" if self.manual or self._stepped_in() else "auto"

    def _manual_words(self) -> str:
        return "the arm side is in manual mode" + (f" ({self._manual_why})" if self._stepped_in() and not self.manual else "")

    def _running(self):
        with self._lock:
            return next((r for r in self._runs.values() if not r["ended"]), None)

    def safe_to_move(self):
        if self.mode() == "manual":
            return False, f"T{self._manual_words()[1:]}: no sequence is released, so no travel pose is set."
        if self._running():
            return False, "Something is running on the arms: the cleaning sequence."
        return True, "Both arms are at the travel pose and nothing is running."

    def _asked_for(self, fixture_id):
        """The rejection asked for with ``reject:<CODE>``, if that is the next outcome (it is used up), else None."""
        with self._lock:
            kind, code, axes = self.outcomes[self._n % len(self.outcomes)]
            if kind != "reject":
                return None
            self._n += 1
        said = {"OFFSET_OUT_OF_TOLERANCE": "the offset is counted as outside the tolerance",
                "ATTEMPTS_EXHAUSTED": "the offset is counted as outside the tolerance on the visit's last attempt",
                "BUSY": "the arms are busy: Run Program 51, started from the arm side's page",
                "ARM_NOT_READY": "the RIGHT arm's joint 2 is 41.5 deg from the travel pose (within 3 deg counts as at it)",
                "MANUAL_MODE": "no sequence is released for automatic running: the arm side is in manual mode",
                "UNKNOWN_FIXTURE": f"no sequence is released for fixture {fixture_id!r}",
                "BAD_GOAL": "the goal is counted as malformed"}[code]
        return (code, f"Stand-in, asked for with reject:{code}: {said}.", list(axes))

    def blocked(self, fixture_id):
        if self.mode() == "manual":
            return ("MANUAL_MODE", f"No sequence is released for automatic running: {self._manual_words()}.")
        if fixture_id is not None and self.fixtures and ugv_link.fixture_key(fixture_id) not in {
                ugv_link.fixture_key(f) for f in self.fixtures}:
            return ("UNKNOWN_FIXTURE", f"No sequence is released for fixture {fixture_id!r}: the release is for "
                    + ", ".join(self.fixtures) + ".")
        if self._running():
            return ("BUSY", "The arms are busy: the cleaning sequence.")
        return None if fixture_id is None else self._asked_for(fixture_id)     # the status asks with no fixture: uses nothing up

    # ── a cleaning
    def start(self, goal):
        with self._lock:
            outcome = self.outcomes[self._n % len(self.outcomes)]
            self._n += 1
            run = dict(goal_id=goal["goal_id"], t0=time.time(), stop=threading.Event(), ended=False, outcome=outcome,
                       seconds=self.seconds, stages=self.stages,
                       events=[dict(seq=0, t=0.0, text=f"Stand-in: {self.stages} stage(s), {self.seconds:g} s, ending "
                                    f"{outcome_words(outcome).replace(':', ' ')}. Nothing moves.", level="info")])
            self._runs[goal["goal_id"]] = run
        threading.Thread(target=self._clean, args=(run,), name="standin-clean", daemon=True).start()
        return dict(handle=goal["goal_id"], expected_duration_s=run["seconds"])

    @staticmethod
    def _stage(run):
        return min(run["stages"], int((time.time() - run["t0"]) / (run["seconds"] / run["stages"])) + 1)

    def _clean(self, run):
        kind, code, _ = run["outcome"]
        ends = run["seconds"] * (0.4 if kind.startswith("aborted") else 1.0)        # an abort stops part of the way
        said = 0
        while time.time() - run["t0"] < ends and not run["stop"].is_set():
            stage = self._stage(run)
            if stage != said:
                said = stage
                run["events"].append(dict(seq=len(run["events"]), t=round(time.time() - run["t0"], 2),
                                          text=f"Stage {stage} of {run['stages']} started.", level="info"))
            time.sleep(0.05)
        stopped = run["stop"].is_set()
        safe = not stopped and kind in ("succeeded", "aborted-safe")
        if not safe:                                         # a person steps in: nothing starts again by itself (as Lite)
            self._manual_why = ("STOP: a person releases the sequence again once the cell is clear" if stopped else
                                f"goal {run['goal_id']} ended with the arms not at the travel pose: a person puts them "
                                "back and releases the sequence again")
            self._manual_until = time.time() + self.recover_s
        outcome = "stopped" if stopped else "done" if kind in ("succeeded", "unsafe") else "failed"
        detail = ("Stopped by STOP on the arm side." if stopped else "" if kind == "succeeded"
                  else "The sequence completed. The RIGHT arm's joint 2 is 41.5 deg from the travel pose." if kind == "unsafe"
                  else f"Stand-in abort ({code}).")
        fb = self.feedback(run["goal_id"], 0)
        run["ended"] = True
        self.link.ended(run["goal_id"], outcome, abort_code=code, detail=detail,
                        stages_done=run["stages"] if outcome == "done" else max(0, fb["stage_index"] - 1),
                        duration_s=time.time() - run["t0"], safe_to_move=safe, feedback=fb)

    def feedback(self, handle, since=0):
        run = self._runs.get(handle)
        if run is None:
            return {}
        return dict(stage_index=self._stage(run), stage_count=run["stages"], stage_name=f"Stand-in stage {self._stage(run)}",
                    elapsed_s=round(time.time() - run["t0"], 1), seq=len(run["events"]),
                    events=[e for e in run["events"] if e["seq"] >= since])

    def stop(self, handle):
        run = self._runs.get(handle)
        if run is not None:
            run["stop"].set()

    # ── the stand-in's own controls (GET / POST /standin): not the contract's
    def view(self) -> dict:
        with self._lock:
            nxt = self.outcomes[self._n % len(self.outcomes)]
            return dict(standin=True, mode=self.mode(), manual=self.manual, outcomes=[outcome_words(o) for o in self.outcomes],
                        next_outcome=outcome_words(nxt), seconds=self.seconds, stages=self.stages, recover_s=self.recover_s,
                        fixtures=list(self.fixtures))

    def control(self, method: str, path: str, body: dict):
        if path != "/standin" or method not in ("GET", "POST"):
            return None
        if method == "POST":
            unknown = sorted(set(body) - {"outcomes", "manual", "seconds", "recover", "fixtures"})
            if unknown:
                raise ValueError(f"The stand-in is set with outcomes, manual, seconds, recover, fixtures; not {', '.join(unknown)}.")
            new = parse_outcomes(body["outcomes"]) if "outcomes" in body else None
            for key in ("seconds", "recover"):
                v = body.get(key)
                if key in body and (isinstance(v, bool) or not isinstance(v, (int, float)) or v < 0 or (key == "seconds" and v <= 0)):
                    raise ValueError(f"{key} must be a number of seconds" + (" over 0." if key == "seconds" else "."))
            if "manual" in body and not isinstance(body["manual"], bool):
                raise ValueError("manual must be true or false.")
            with self._lock:
                if new is not None:
                    self.outcomes, self._n = new, 0
                if "seconds" in body:
                    self.seconds = float(body["seconds"])
                if "recover" in body:
                    self.recover_s = float(body["recover"])
                if "fixtures" in body:
                    self.fixtures = names(body["fixtures"])
                if "manual" in body:
                    self.manual = body["manual"]
                    if not self.manual:
                        self._manual_until = 0.0             # "a person has released the sequence again"
            print(time.strftime("%H:%M:%S"), "Stand-in set:", body, flush=True)
        return self.view()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="A stand-in for the arm side of the UGV link. It moves nothing.")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--token", default="standin-token")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--stages", type=int, default=9)
    ap.add_argument("--outcomes", default="succeeded")
    ap.add_argument("--recover", type=float, default=30.0)
    ap.add_argument("--manual", action="store_true")
    ap.add_argument("--fixtures", default="")
    ap.add_argument("--store", type=Path, default=None)
    a = ap.parse_args(argv)
    if a.seconds <= 0 or a.stages < 1 or a.recover < 0:
        raise SystemExit("--seconds must be over 0, --stages at least 1 and --recover not negative.")
    try:
        outcomes = parse_outcomes(a.outcomes)
    except ValueError as exc:
        raise SystemExit(f"--{exc}") from None
    backend = Standin(a.seconds, a.stages, outcomes, a.recover, a.manual, names(a.fixtures))
    backend.link = ugv_link.Link(backend, store=a.store, say=lambda text, **kw: print(time.strftime("%H:%M:%S"), text, flush=True))
    try:
        srv = ugv_link.make_server(backend.link, a.host, a.port, a.token, extra=backend.control)
    except OSError as exc:
        raise SystemExit(f"{a.host}:{a.port} could not be opened ({exc}).") from None
    print(f"Arm side stand-in (it moves nothing): http://{a.host}:{a.port}{ugv_link.PREFIX}   X-Link-Token: {a.token}", flush=True)
    print(f"Mode: {backend.mode()}; a cleaning takes {a.seconds:g} s in {a.stages} stage(s); outcomes in turn: "
          f"{', '.join(outcome_words(o) for o in outcomes)}. Ctrl+C to stop.", flush=True)
    try:
        srv.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
