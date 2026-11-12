"""The UGV's link to the arm side (server/ugv_link.py; the user, 11 Nov): the CleanFixture action's protocol on a
backend that moves nothing. The offset gate (+-10 mm, +-10 mm, +-1 deg), the retries (three attempts a visit), a goal
taken once however often it is sent, one goal at a time, cancel as STOP, the result with safe_to_move, the goals kept
across a restart, and the HTTP routes with their token. (Lite behind it, on the emulated arms: test_ugv_link_emulated.py.)"""
import json
import threading
import urllib.error
import urllib.request

import pytest

from alix_studio_lite.server import ugv_link
from alix_studio_lite.server.ugv_link import Link, Refused

EVENTS = [dict(seq=i, t=float(i), text=f"line {i}", level="info") for i in range(3)]


class Backend:
    """A backend that cleans nothing: what it is told, kept."""

    def __init__(self):
        self.auto, self.block, self.safe, self.refuse = True, None, True, None
        self.started, self.stopped = [], []

    def mode(self):
        return "auto" if self.auto else "manual"

    def blocked(self, fixture_id):
        return self.block

    def safe_to_move(self):
        return self.safe, "words"

    def start(self, goal):
        if self.refuse:
            raise Refused(*self.refuse)
        self.started.append(goal["goal_id"])
        return dict(handle=goal["goal_id"], expected_duration_s=235.04)

    def feedback(self, handle, since=0):
        return dict(stage_index=2, stage_count=9, stage_name="Open the lid", elapsed_s=12.5, seq=len(EVENTS),
                    events=[e for e in EVENTS if e["seq"] >= since])

    def stop(self, handle):
        self.stopped.append(handle)


def goal(gid="g1", x=0.004, y=-0.006, yaw=0.4, attempt=1, **more):
    return dict(dict(protocol=1, goal_id=gid, fixture_id="cubicle_1", attempt=attempt, offset_x_m=x, offset_y_m=y,
                     offset_yaw_deg=yaw, offset_source="marker"), **more)


@pytest.fixture
def made(tmp_path):
    backend = Backend()
    return Link(backend, store=tmp_path / "goals.json"), backend, tmp_path / "goals.json"


def test_the_gate_is_the_three_limits_each_on_its_own():
    assert ugv_link.gate(0.010, -0.010, 1.0) == [] and ugv_link.gate(0.0, 0.0, -1.0) == []      # on a limit is inside
    assert ugv_link.gate(0.0101, 0.0, 0.0) == ["x"] and ugv_link.gate(0.0, -0.0101, 0.0) == ["y"]
    assert ugv_link.gate(0.0, 0.0, -1.01) == ["yaw"] and ugv_link.gate(0.02, 0.02, 2.0) == ["x", "y", "yaw"]
    assert ugv_link.limits() == {"x_m": 0.010, "y_m": 0.010, "yaw_deg": 1.0} and ugv_link.MAX_ATTEMPTS == 3


def test_a_goal_inside_the_tolerance_is_accepted_runs_and_ends_with_its_result(made):
    link, backend, _ = made
    reply = link.send_goal(goal())
    assert reply["accepted"] is True and reply["ugv_action"] == "HOLD" and reply["reject_code"] == "" and reply["exceeded"] == []
    assert reply["status"] == "ACCEPTED" and reply["expected_duration_s"] == 235.0 and reply["limits"]["yaw_deg"] == 1.0
    assert backend.started == ["g1"] and link.active == "g1"
    assert link.status()["active_goal_id"] == "g1" and link.status()["ready"] is False and link.status()["not_ready"] == "BUSY"
    view = link.goal("g1")
    assert view["status"] == "EXECUTING" and "result" not in view and view["seq"] == 3
    assert view["feedback"] == dict(status="EXECUTING", stage_index=2, stage_count=9, stage_name="Open the lid", elapsed_s=12.5, seq=3)
    assert [e["seq"] for e in view["events"]] == [0, 1, 2] and [e["seq"] for e in link.goal("g1", 2)["events"]] == [2]
    link.ended("g1", "done", stages_done=9, duration_s=231.44, safe_to_move=True, feedback=backend.feedback("g1"))
    done = link.goal("g1")
    assert done["status"] == "SUCCEEDED" and link.active is None and link.status()["ready"] is True
    assert done["result"] == dict(status="SUCCEEDED", safe_to_move=True, abort_code="", ugv_action="CONTINUE_MISSION",
                                  stages_done=9, duration_s=231.4, detail="")
    link.ended("g1", "failed", abort_code="ESTOP")                       # once ended, it stays as it ended
    assert link.goal("g1")["result"]["status"] == "SUCCEEDED"
    assert link.goal("nobody") is None


def test_outside_the_tolerance_the_ugv_is_asked_to_approach_again_three_attempts_a_visit(made):
    link, backend, _ = made
    r1 = link.send_goal(goal("a1", yaw=1.6))
    assert r1["accepted"] is False and r1["reject_code"] == "OFFSET_OUT_OF_TOLERANCE" and r1["ugv_action"] == "RETRY_APPROACH"
    assert r1["exceeded"] == ["yaw"] and "Yaw +1.60 deg is outside 1.00 deg" in r1["detail"] and "Attempt 1 of 3" in r1["detail"]
    r2 = link.send_goal(goal("a2", x=0.012, y=-0.02, attempt=2))
    assert r2["reject_code"] == "OFFSET_OUT_OF_TOLERANCE" and r2["exceeded"] == ["x", "y"]
    r3 = link.send_goal(goal("a3", x=0.012, attempt=3))                 # the third is the last
    assert r3["reject_code"] == "ATTEMPTS_EXHAUSTED" and r3["ugv_action"] == "CALL_OPERATOR"
    r4 = link.send_goal(goal("a4", attempt=4))                          # a fourth is not taken, even inside
    assert r4["reject_code"] == "ATTEMPTS_EXHAUSTED" and r4["accepted"] is False
    assert backend.started == [] and link.active is None and link.goal("a1") is None      # nothing moved, nothing kept as a goal
    ok = link.send_goal(goal("a5", attempt=3))                          # the third attempt inside the tolerance runs
    assert ok["accepted"] is True and backend.started == ["a5"]


def test_the_same_goal_id_is_the_same_goal_and_a_second_goal_waits(made):
    link, backend, _ = made
    first = link.send_goal(goal("s1"))
    again = link.send_goal(goal("s1"))                                  # a lost reply, sent again
    assert again["accepted"] is True and again["status"] == "EXECUTING" and backend.started == ["s1"]
    assert {k: v for k, v in again.items() if k != "status"} == {k: v for k, v in first.items() if k != "status"}
    busy = link.send_goal(goal("s2"))
    assert busy["reject_code"] == "BUSY" and busy["ugv_action"] == "RESEND_LATER" and "s1" in busy["detail"]
    link.ended("s1", "done", safe_to_move=True)
    assert link.send_goal(goal("s2"))["accepted"] is True and backend.started == ["s1", "s2"]   # asked again later: taken
    assert link.send_goal(goal("s1"))["status"] == "SUCCEEDED"          # an ended goal's id still names that goal


def test_readiness_is_asked_before_the_offset_and_a_refused_start_is_a_rejection(made):
    link, backend, _ = made
    backend.block = ("MANUAL_MODE", "No sequence is released.")
    r = link.send_goal(goal("m1", yaw=5.0))                             # out of tolerance too: the mode is what is said
    assert r["reject_code"] == "MANUAL_MODE" and r["ugv_action"] == "CALL_OPERATOR" and r["exceeded"] == []
    assert link.status()["ready"] is False and link.status()["not_ready"] == "MANUAL_MODE"
    backend.block = ("UNKNOWN_FIXTURE", "Not released for it.")
    assert link.send_goal(goal("m2"))["reject_code"] == "UNKNOWN_FIXTURE"
    backend.block = ("ARM_NOT_READY", "The LEFT arm's E-stop is pressed.")
    assert link.send_goal(goal("m3"))["ugv_action"] == "CALL_OPERATOR"
    backend.block, backend.refuse = None, ("ARM_NOT_READY", "The wrist camera cannot be read.")
    r = link.send_goal(goal("m4"))
    assert r["reject_code"] == "ARM_NOT_READY" and "camera" in r["detail"] and link.active is None
    backend.refuse = ("SOMETHING_ELSE", "an unknown code")
    assert link.send_goal(goal("m5"))["reject_code"] == "ARM_NOT_READY"
    backend.block = ("OFFSET_OUT_OF_TOLERANCE", "As this backend reads it.", ["yaw"])       # a backend may name the axes
    r = link.send_goal(goal("m6"))
    assert r["reject_code"] == "OFFSET_OUT_OF_TOLERANCE" and r["exceeded"] == ["yaw"] and r["ugv_action"] == "RETRY_APPROACH"
    assert backend.started == []


def test_every_outcome_is_a_ledger_row_with_the_offset_it_came_with(tmp_path):
    """The UGV side, 12 Nov: the offset logged with each result, so both sides see what the tolerance does in practice."""
    rows, said, backend = [], [], Backend()
    link = Link(backend, store=tmp_path / "goals.json", say=lambda text, **kw: said.append(text), record=rows.append)
    link.send_goal(goal("l1", yaw=1.6, offset_source="lidar"))
    link.send_goal(goal("l2", x=0.003, y=-0.002, yaw=0.4, attempt=2, offset_source="lidar_fit"))
    assert len(rows) == 1                                               # an accepted goal's row is its end's
    link.ended("l2", "done", stages_done=9, duration_s=219.74, safe_to_move=True)
    link.send_goal(dict(goal_id="l3"))                                  # not a goal: nothing to say of its offset
    rej, done, bad = rows
    assert rej["goal_id"] == "l1" and rej["outcome"] == "REJECTED" and rej["reject_code"] == "OFFSET_OUT_OF_TOLERANCE"
    assert rej["exceeded"] == ["yaw"] and rej["offset_yaw_deg"] == 1.6 and rej["offset_source"] == "lidar"
    assert rej["ugv_action"] == "RETRY_APPROACH" and rej["safe_to_move"] is None and rej["abort_code"] == ""
    assert done["goal_id"] == "l2" and done["outcome"] == "SUCCEEDED" and done["fixture_id"] == "cubicle_1" and done["attempt"] == 2
    assert (done["offset_x_m"], done["offset_y_m"], done["offset_yaw_deg"], done["offset_source"]) == (0.003, -0.002, 0.4, "lidar_fit")
    assert done["safe_to_move"] is True and done["ugv_action"] == "CONTINUE_MISSION" and done["stages_done"] == 9
    assert done["duration_s"] == 219.7 and done["reject_code"] == "" and done["t"] > 0
    assert bad["outcome"] == "REJECTED" and bad["reject_code"] == "BAD_GOAL" and bad["offset_yaw_deg"] is None
    assert set(rej) == set(done) == set(bad)                            # one shape, whatever the outcome
    assert any("goal l2 SUCCEEDED" in t and "offset x +3.0 mm, y -2.0 mm, yaw +0.40 deg, lidar_fit" in t for t in said)
    link.send_goal(goal("l4", yaw=-0.9))
    again = []
    Link(Backend(), store=tmp_path / "goals.json", record=again.append)   # this program stopped while l4 ran
    assert [(r["goal_id"], r["outcome"], r["abort_code"], r["offset_yaw_deg"]) for r in again] == [("l4", "ABORTED", "ARM_RESTARTED", -0.9)]

    def broken(row):
        raise OSError("disk full")

    link = Link(Backend(), record=broken)                               # a ledger that cannot be written fails no goal
    assert link.send_goal(goal("l5"))["accepted"] is True
    link.ended("l5", "done", safe_to_move=True)
    assert link.goal("l5")["status"] == "SUCCEEDED"


def test_fixture_names_are_compared_without_case_or_spaces():
    key = ugv_link.fixture_key
    assert key("Cubicle 1") == key("cubicle 1") == key("  CUBICLE   1 ") and key("Cubicle 1") != key("Cubicle 11")
    assert key("Cubicle 1") != key("Cubicle1")                          # a space is still a space


@pytest.mark.parametrize("change, said", [
    (dict(offset_x_m=4.0), "metres"),                                   # millimetres sent as metres: never accepted
    (dict(offset_yaw_deg=45.0), "beyond 30"),
    (dict(offset_y_m="0.004"), "offset_y_m must be a number"),
    (dict(offset_x_m=True), "offset_x_m must be a number"),
    (dict(offset_x_m=float("nan")), "offset_x_m must be a number"),
    (dict(goal_id=""), "goal_id"), (dict(goal_id="has space"), "goal_id"),
    (dict(fixture_id="  "), "fixture_id"), (dict(attempt=0), "attempt"), (dict(attempt=1.5), "attempt"),
    (dict(protocol=2), "protocol"), (dict(offset_source=7), "offset_source"),
])
def test_a_goal_that_is_not_one_is_a_bad_goal(made, change, said):
    link, backend, _ = made
    r = link.send_goal({**goal("b1"), **change})
    assert r["accepted"] is False and r["reject_code"] == "BAD_GOAL" and r["ugv_action"] == "CALL_OPERATOR" and said in r["detail"]
    assert backend.started == []


def test_a_missing_offset_is_a_bad_goal_and_the_optional_fields_have_their_defaults(made):
    link, backend, _ = made
    body = goal("d1")
    del body["offset_yaw_deg"]
    assert link.send_goal(body)["reject_code"] == "BAD_GOAL"
    body = goal("d2")
    del body["attempt"], body["offset_source"], body["protocol"]
    assert link.send_goal(body)["accepted"] is True


def test_cancel_is_stop_and_a_stop_from_the_arm_side_is_an_abort(made):
    link, backend, _ = made
    link.send_goal(goal("c1"))
    out = link.cancel("c1")
    assert out["status"] == "CANCELING" and backend.stopped == ["c1"] and link.active == "c1"
    link.ended("c1", "stopped", stages_done=3, duration_s=40.0, safe_to_move=False)
    res = link.goal("c1")["result"]
    assert res["status"] == "CANCELED" and res["safe_to_move"] is False and res["ugv_action"] == "CALL_OPERATOR" and res["abort_code"] == ""
    assert link.cancel("c1")["status"] == "CANCELED" and backend.stopped == ["c1"]         # an ended goal: nothing stopped again
    assert link.cancel("nobody") is None
    link.send_goal(goal("c2"))
    link.ended("c2", "stopped", safe_to_move=False)                     # STOP on the page: the arm side's
    assert link.goal("c2")["result"]["status"] == "ABORTED" and link.goal("c2")["result"]["abort_code"] == "OPERATOR_STOP"
    link.send_goal(goal("c3"))
    link.ended("c3", "failed", abort_code="MECHANISM_FAILED", detail="Open the lid: the lid did not open.", stages_done=2,
               duration_s=96.44, safe_to_move=False)
    res = link.goal("c3")["result"]
    assert res == dict(status="ABORTED", safe_to_move=False, abort_code="MECHANISM_FAILED", ugv_action="CALL_OPERATOR",
                       stages_done=2, duration_s=96.4, detail="Open the lid: the lid did not open.")
    link.send_goal(goal("c4"))
    link.ended("c4", "failed", abort_code="not a code", safe_to_move=True)   # aborted with the arms back: the UGV may drive
    res = link.goal("c4")["result"]
    assert res["abort_code"] == "INTERNAL_ERROR" and res["safe_to_move"] is True and res["ugv_action"] == "CONTINUE_MISSION"


def test_the_goals_are_kept_across_a_restart_and_a_running_one_ends_as_restarted(made):
    link, backend, store = made
    link.send_goal(goal("k1"))
    link.ended("k1", "done", stages_done=9, duration_s=230.0, safe_to_move=True, feedback=backend.feedback("k1"))
    link.send_goal(goal("k2"))                                          # running when the program stops
    said = []
    after = Link(Backend(), store=store, say=lambda text, **kw: said.append(text))
    assert after.goal("k1")["result"]["status"] == "SUCCEEDED" and [e["seq"] for e in after.goal("k1", 1)["events"]] == [1, 2]
    lost = after.goal("k2")
    assert lost["status"] == "ABORTED" and lost["result"]["abort_code"] == "ARM_RESTARTED" and lost["result"]["safe_to_move"] is False
    assert after.active is None and any("k2" in t and "ARM_RESTARTED" in t for t in said)
    assert after.send_goal(goal("k2"))["status"] == "ABORTED"           # its id still names it: not started again
    assert json.loads(store.read_text())["goals"][1]["status"] == "ABORTED"
    for i in range(ugv_link.KEEP + 5):                                  # the newest KEEP are kept
        after.send_goal(goal(f"n{i}"))
        after.ended(f"n{i}", "done", safe_to_move=True)
    assert after.goal("k1") is None and after.goal(f"n{ugv_link.KEEP + 4}") is not None
    assert len(json.loads(store.read_text())["goals"]) == ugv_link.KEEP
    assert [g["goal_id"] for g in after.recent(2)] == [f"n{ugv_link.KEEP + 4}", f"n{ugv_link.KEEP + 3}"]


def test_a_file_that_cannot_be_read_starts_an_empty_link(tmp_path):
    (tmp_path / "goals.json").write_text("not json")
    link = Link(Backend(), store=tmp_path / "goals.json")
    assert link.recent() == [] and link.send_goal(goal("f1"))["accepted"] is True


# ── over HTTP

@pytest.fixture
def served(made):
    link, backend, _ = made
    srv = ugv_link.make_server(link, "127.0.0.1", 0, "secret-token")
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
    yield f"http://127.0.0.1:{srv.server_address[1]}", link, backend
    srv.shutdown()
    srv.server_close()


def http(base, method, path, body=None, token="secret-token", raw=None):
    headers = {"Content-Type": "application/json"}
    if token is not None:
        headers["X-Link-Token"] = token
    data = raw if raw is not None else None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(base + path, method=method, headers=headers, data=data)
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def test_the_routes_carry_the_action_and_need_the_token(served):
    base, link, backend = served
    assert http(base, "GET", "/link/v1/status", token=None)[0] == 401
    assert http(base, "GET", "/link/v1/status", token="wrong")[0] == 401
    assert http(base, "POST", "/link/v1/goals", goal("h0"), token="wrong")[0] == 401 and backend.started == []
    code, st = http(base, "GET", "/link/v1/status")
    assert code == 200 and st["protocol"] == 1 and st["mode"] == "auto" and st["ready"] is True and st["safe_to_move"] is True
    assert st["active_goal_id"] == "" and st["limits"] == {"x_m": 0.010, "y_m": 0.010, "yaw_deg": 1.0} and st["max_attempts"] == 3
    code, rej = http(base, "POST", "/link/v1/goals", goal("h1", yaw=1.6))
    assert code == 200 and rej["accepted"] is False and rej["ugv_action"] == "RETRY_APPROACH"     # a rejection is an answer
    code, reply = http(base, "POST", "/link/v1/goals", goal("h2", attempt=2))
    assert code == 200 and reply["accepted"] is True and reply["goal_id"] == "h2"
    code, view = http(base, "GET", "/link/v1/goals/h2?since=1")
    assert code == 200 and view["status"] == "EXECUTING" and [e["seq"] for e in view["events"]] == [1, 2]
    assert http(base, "GET", "/link/v1/goals/h2?since=x")[0] == 400
    code, out = http(base, "POST", "/link/v1/goals/h2/cancel")
    assert code == 200 and out["status"] == "CANCELING" and backend.stopped == ["h2"]
    link.ended("h2", "stopped", safe_to_move=False)
    code, view = http(base, "GET", "/link/v1/goals/h2")
    assert view["status"] == "CANCELED" and view["result"]["ugv_action"] == "CALL_OPERATOR"
    assert http(base, "GET", "/link/v1/goals/nobody")[0] == 404 and http(base, "POST", "/link/v1/goals/nobody/cancel")[0] == 404
    assert http(base, "POST", "/link/v1/goals", raw=b"{not json")[0] == 400
    assert http(base, "POST", "/link/v1/goals", raw=b"[1, 2]")[0] == 400
    assert http(base, "GET", "/api/robots/status")[0] == 404           # only the link's routes are here
    assert http(base, "GET", "/standin")[0] == 404                      # a stand-in's controls are no part of Lite's link
    assert http(base, "GET", "/link/v1/goals")[0] == 404 and http(base, "PUT", "/link/v1/status")[0] == 405


def test_the_token_is_made_once_and_kept(tmp_path):
    tok = ugv_link.read_token(tmp_path / "t" / "token.txt")
    assert len(tok) >= 24 and ugv_link.read_token(tmp_path / "t" / "token.txt") == tok
    assert (tmp_path / "t" / "token.txt").stat().st_mode & 0o077 == 0


def test_a_stand_ins_own_controls_sit_beside_the_link_behind_the_same_token(made):
    link, backend, _ = made

    def own(method, path, body):
        if path != "/standin":
            return None
        if body.get("bad"):
            raise ValueError("That is not a setting.")
        return dict(standin=True, method=method, got=body)

    srv = ugv_link.make_server(link, "127.0.0.1", 0, "secret-token", extra=own)
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        assert http(base, "GET", "/standin", token="wrong")[0] == 401
        assert http(base, "GET", "/standin") == (200, dict(standin=True, method="GET", got={}))
        assert http(base, "POST", "/standin", {"seconds": 5})[1]["got"] == {"seconds": 5}
        code, out = http(base, "POST", "/standin", {"bad": 1})
        assert code == 400 and out["error"] == "That is not a setting."
        assert http(base, "GET", "/elsewhere")[0] == 404
        assert http(base, "GET", "/link/v1/status")[1]["mode"] == "auto"     # the link's own routes as before
    finally:
        srv.shutdown()
        srv.server_close()
