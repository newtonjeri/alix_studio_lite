"""The UGV link on ALIX's emulated arms (server/ugv_link.py, ugv_link_lite.py; the user, 11 Nov): a Lite started with
``--link``, its listener and token; manual until S2's sequence is released, with the confirmations a run from the page
takes; a goal inside the tolerance runs that sequence on both arms and ends with its result; the UGV's cancel is STOP;
STOP on the page is the arm side's abort; a sequence edited after its release is not run; and the UGV is told to stay
when an arm does not end at the travel pose. (The protocol alone: test_ugv_link.py.)"""
import json
import socket
import time
import urllib.error
import urllib.request

import pytest

from mech_emulated import P, call, start, wait

WAIT_1 = {"type": "wait", "seconds": 1}
WAIT_LONG = {"type": "wait", "seconds": 300}            # 30 s on the wall here: long enough to be stopped in


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    ctx, proc = start(tmp_path_factory, "lite-ugv-link", [], extra=("--link", str(port)))
    ctx["link"] = f"http://127.0.0.1:{port}"
    ctx["link_tok"] = (ctx["data"] / "ugv_link_token.txt").read_text().strip()
    yield ctx
    proc.terminate()
    proc.wait(10)


@pytest.fixture(autouse=True)
def arms_left_free(server):
    """After each test nothing runs and no arm is left latched (a test's own failure is its own)."""
    yield
    try:
        if call(server, "GET", "/api/robots/status").get("jobs"):
            stop(server)
        recover(server)
    except OSError:
        pass


def link(ctx, method, path, body=None, token=True):
    headers = {"Content-Type": "application/json"}
    if token:
        headers["X-Link-Token"] = ctx["link_tok"]
    req = urllib.request.Request(ctx["link"] + path, method=method, headers=headers,
                                 data=None if body is None else json.dumps(body).encode())
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def goal(gid, x=0.004, y=-0.006, yaw=0.4, attempt=1, fixture="cubicle_1"):
    return dict(protocol=1, goal_id=gid, fixture_id=fixture, attempt=attempt, offset_x_m=x, offset_y_m=y, offset_yaw_deg=yaw,
                offset_source="marker")


def result_of(ctx, gid, limit=120):
    t0 = time.time()
    while True:
        code, view = link(ctx, "GET", f"/link/v1/goals/{gid}")
        assert code == 200, view
        if "result" in view:
            return view
        assert time.time() - t0 < limit, view
        time.sleep(0.2)


def stop(ctx):
    req = urllib.request.Request(ctx["base"] + "/api/robots/stop-both", method="POST", data=b"{}",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=60) as resp:
        out = json.loads(resp.read())
    for _ in range(200):
        if not call(ctx, "GET", "/api/robots/status").get("jobs"):
            break
        time.sleep(0.1)
    return out


def recover(ctx):
    for side, arm in call(ctx, "GET", "/api/robots/status")["arms"].items():
        if arm.get("estop") == "RECOVERING":
            call(ctx, "POST", f"/api/robots/{side}/recover", {})


def sequence(ctx, left, right):
    call(ctx, "PUT", f"{P}/workflow/left", {"steps": left})
    call(ctx, "PUT", f"{P}/workflow/right", {"steps": right})


def release(ctx, fixtures=""):
    out = call(ctx, "POST", f"{P}/workflow/release", {"fixtures": fixtures, "unsimulated_ok": True, "confirmed": True})
    assert out.get("mode") == "auto", out
    return out


def test_the_link_listens_with_its_own_token_and_is_manual_until_a_release(server):
    assert link(server, "GET", "/link/v1/status", token=False)[0] == 401
    code, st = link(server, "GET", "/link/v1/status")
    assert code == 200 and st["mode"] == "manual" and st["ready"] is False and st["not_ready"] == "MANUAL_MODE"
    assert st["safe_to_move"] is False and "manual mode" in st["safe_to_move_detail"]
    code, r = link(server, "POST", "/link/v1/goals", goal("m-1"))
    assert code == 200 and r["accepted"] is False and r["reject_code"] == "MANUAL_MODE" and r["ugv_action"] == "CALL_OPERATOR"
    page = call(server, "GET", "/api/link")
    assert page["mode"] == "manual" and page["release"] is None and page["listening"]["host"] == "127.0.0.1"
    assert page["limits"] == {"x_m": 0.010, "y_m": 0.010, "yaw_deg": 1.0} and page["goals"] == []
    assert link(server, "GET", "/api/link")[0] == 404                   # the page's routes are not on the link's listener
    with pytest.raises(urllib.error.HTTPError) as refused:              # nor the link's routes on the page's
        call(server, "GET", "/link/v1/status")
    assert refused.value.code == 404


def test_a_release_takes_a_runs_confirmations_and_the_arms_pose(server):
    sequence(server, [WAIT_1], [WAIT_1])
    asked = call(server, "POST", f"{P}/workflow/release", {"fixtures": "cubicle_1, cubicle_2"})
    assert asked["code"] == "unsimulated"                               # as a run from the page: not simulated as it is
    asked = call(server, "POST", f"{P}/workflow/release", {"fixtures": "cubicle_1, cubicle_2", "unsimulated_ok": True})
    assert asked["code"] == "confirm"
    assert link(server, "GET", "/link/v1/status")[1]["mode"] == "manual"      # nothing released by asking
    out = release(server, "cubicle_1, cubicle_2, Cubicle 3 ;cubicle  3")      # a name typed twice is one name
    assert out["release"]["fixtures"] == ["cubicle_1", "cubicle_2", "Cubicle 3"] and out["release"]["stages"] == 1 and not out["release"]["simulated"]
    assert out["safe_to_move"] is True
    st = link(server, "GET", "/link/v1/status")[1]
    assert st["mode"] == "auto" and st["ready"] is True and st["not_ready"] == "" and st["safe_to_move"] is True
    assert not call(server, "GET", "/api/robots/status").get("jobs")     # a release starts nothing


def test_a_goal_inside_the_tolerance_runs_the_sequence_and_one_outside_moves_nothing(server):
    r = link(server, "POST", "/link/v1/goals", goal("far-1", yaw=1.6))[1]
    assert r["accepted"] is False and r["ugv_action"] == "RETRY_APPROACH" and r["exceeded"] == ["yaw"]
    r = link(server, "POST", "/link/v1/goals", goal("other-1", fixture="cubicle_9"))[1]
    assert r["reject_code"] == "UNKNOWN_FIXTURE" and "cubicle_1, cubicle_2" in r["detail"]
    assert not call(server, "GET", "/api/robots/status").get("jobs")
    r = link(server, "POST", "/link/v1/goals", goal("go-1", attempt=2, fixture="cubicle 3"))[1]     # the UGV's "Cubicle 3"
    assert r["accepted"] is True and r["ugv_action"] == "HOLD" and r["status"] == "ACCEPTED", r
    busy = link(server, "POST", "/link/v1/goals", goal("go-2"))[1]
    assert busy["reject_code"] == "BUSY" and busy["ugv_action"] == "RESEND_LATER"
    again = link(server, "POST", "/link/v1/goals", goal("go-1", attempt=2, fixture="cubicle 3"))[1]      # the reply lost, sent again
    assert again["accepted"] is True and again["status"] in ("EXECUTING", "SUCCEEDED")
    view = result_of(server, "go-1")
    res = view["result"]
    assert view["status"] == "SUCCEEDED" and res["status"] == "SUCCEEDED" and res["abort_code"] == "", view
    assert res["safe_to_move"] is True and res["ugv_action"] == "CONTINUE_MISSION" and res["stages_done"] == 1
    assert res["duration_s"] > 0 and view["feedback"]["stage_count"] == 1
    said = " | ".join(e["text"] for e in view["events"])
    assert "Sequence complete" in said and "without a simulation of it, as confirmed" in said
    st = link(server, "GET", "/link/v1/status")[1]
    assert st["ready"] is True and st["active_goal_id"] == "" and st["safe_to_move"] is True
    page = call(server, "GET", "/api/link")
    assert [g["goal_id"] for g in page["goals"]] == ["go-1"] and page["goals"][0]["status"] == "SUCCEEDED"
    kept = json.loads((server["data"] / "ugv_link_goals.json").read_text())["goals"]
    assert [g["goal_id"] for g in kept] == ["go-1"] and kept[0]["result"]["status"] == "SUCCEEDED"
    # the ledger (the UGV side, 12 Nov): each outcome beside the offset it came with, in the day's logs
    rows = {}
    for path in (server["data"] / "logs").glob("*/ugv_link.jsonl"):
        rows.update({r["goal_id"]: r for r in map(json.loads, path.read_text().splitlines())})
    assert rows["far-1"]["outcome"] == "REJECTED" and rows["far-1"]["reject_code"] == "OFFSET_OUT_OF_TOLERANCE"
    assert rows["far-1"]["exceeded"] == ["yaw"] and rows["far-1"]["offset_yaw_deg"] == 1.6
    assert rows["other-1"]["reject_code"] == "UNKNOWN_FIXTURE" and rows["go-2"]["reject_code"] == "BUSY"
    done = rows["go-1"]
    assert done["outcome"] == "SUCCEEDED" and done["safe_to_move"] is True and done["fixture_id"] == "cubicle 3"
    assert (done["offset_x_m"], done["offset_y_m"], done["offset_yaw_deg"], done["offset_source"]) == (0.004, -0.006, 0.4, "marker")
    assert done["attempt"] == 2 and done["stages_done"] == 1 and done["duration_s"] == res["duration_s"]


def test_a_sequence_edited_after_its_release_is_not_run(server):
    sequence(server, [WAIT_LONG], [WAIT_LONG])                          # edited: not what was released
    r = link(server, "POST", "/link/v1/goals", goal("ed-1"))[1]
    assert r["accepted"] is False and r["reject_code"] == "MANUAL_MODE" and "no longer as it was released" in r["detail"], r
    assert not call(server, "GET", "/api/robots/status").get("jobs")
    st = link(server, "GET", "/link/v1/status")[1]
    assert st["mode"] == "manual" and st["safe_to_move"] is False       # the release ended with it
    assert "no longer as it was released" in call(server, "GET", "/api/link")["ended_why"]


def test_the_ugvs_cancel_is_stop_and_a_person_releases_again(server):
    sequence(server, [WAIT_LONG], [WAIT_LONG])
    release(server)                                                     # for any fixture
    assert link(server, "POST", "/link/v1/goals", goal("can-1", fixture="anywhere"))[1]["accepted"] is True
    code, out = link(server, "POST", "/link/v1/goals/can-1/cancel")
    assert code == 200 and out["status"] in ("CANCELING", "CANCELED")
    view = result_of(server, "can-1")
    res = view["result"]
    assert res["status"] == "CANCELED" and res["abort_code"] == "" and res["duration_s"] < 25, view
    assert res["safe_to_move"] is False and res["ugv_action"] == "CALL_OPERATOR" and "stopped by STOP" in res["detail"]
    st = link(server, "GET", "/link/v1/status")[1]                      # STOP: a person steps in, so the release ended
    assert st["mode"] == "manual" and st["not_ready"] == "MANUAL_MODE" and st["safe_to_move"] is False
    assert "stopped by STOP" in call(server, "GET", "/api/link")["ended_why"]
    recover(server)                                                     # recovered is not released: nothing restarts by itself
    r = link(server, "POST", "/link/v1/goals", goal("can-2"))[1]
    assert r["reject_code"] == "MANUAL_MODE" and r["ugv_action"] == "CALL_OPERATOR"
    release(server)
    assert link(server, "GET", "/link/v1/status")[1]["ready"] is True


def test_stop_on_the_arm_side_is_an_abort_and_ends_the_release(server):
    assert link(server, "POST", "/link/v1/goals", goal("op-1"))[1]["accepted"] is True
    assert link(server, "GET", "/link/v1/goals/op-1")[1]["status"] == "EXECUTING"
    stop(server)
    res = result_of(server, "op-1")["result"]
    assert res["status"] == "ABORTED" and res["abort_code"] == "OPERATOR_STOP" and "STOP" in res["detail"]
    assert res["safe_to_move"] is False and res["ugv_action"] == "CALL_OPERATOR"
    assert link(server, "GET", "/link/v1/status")[1]["mode"] == "manual"


def test_stop_between_two_goals_ends_the_release_too(server):
    recover(server)
    sequence(server, [WAIT_1], [WAIT_1])
    release(server)
    stop(server)                                                        # nothing running: STOP all the same
    recover(server)                                                     # and recovered before the UGV asks again
    time.sleep(1.2)                                                     # (Lite looks twice a second)
    st = link(server, "GET", "/link/v1/status")[1]
    assert st["mode"] == "manual" and "stopped by STOP" in call(server, "GET", "/api/link")["ended_why"]


def test_arms_left_away_from_the_travel_pose_from_the_page_are_not_ready(server):
    sequence(server, [WAIT_1], [WAIT_1])
    release(server)
    away = {"type": "global", "name": "CLEAN_A", "move": "movej", "speed": 50, "wait_s": 0.0}          # joint 2 by 5 degrees
    sequence(server, [WAIT_1], [away])                                  # a run from the page, while released
    j = wait(server, call(server, "POST", f"{P}/workflow/run", {"mode": "both", "unsimulated_ok": True, "confirmed": True}))
    assert j["state"] == "done", j
    st = link(server, "GET", "/link/v1/status")[1]
    assert st["mode"] == "auto" and st["safe_to_move"] is False and st["ready"] is False and st["not_ready"] == "ARM_NOT_READY"
    assert "RIGHT arm's joint 2" in st["detail"] and "travel pose" in st["detail"]
    r = link(server, "POST", "/link/v1/goals", goal("left-out-1"))[1]
    assert r["reject_code"] == "ARM_NOT_READY" and r["ugv_action"] == "CALL_OPERATOR" and "travel pose" in r["detail"]
    assert not call(server, "GET", "/api/robots/status").get("jobs")


def test_a_goal_that_ends_with_an_arm_away_from_the_travel_pose_keeps_the_ugv_and_ends_the_release(server):
    point = {"type": "global", "name": "STP_LID_OPEN", "move": "movej", "speed": 50, "wait_s": 0.0}    # joint 1 by 10 degrees
    sequence(server, [WAIT_1], [point])
    release(server)                                                     # the travel pose: where the arms stand now
    assert link(server, "POST", "/link/v1/goals", goal("away-1"))[1]["accepted"] is True
    view = result_of(server, "away-1")
    res = view["result"]
    assert res["status"] == "SUCCEEDED" and res["safe_to_move"] is False and res["ugv_action"] == "CALL_OPERATOR", view
    assert "RIGHT arm's joint 1" in res["detail"] and "travel pose" in res["detail"]
    st = link(server, "GET", "/link/v1/status")[1]                      # a person puts the arm back: released again by them
    assert st["mode"] == "manual" and st["safe_to_move"] is False and st["not_ready"] == "MANUAL_MODE"
    assert "not at the travel pose" in call(server, "GET", "/api/link")["ended_why"]
    r = link(server, "POST", "/link/v1/goals", goal("away-2"))[1]
    assert r["reject_code"] == "MANUAL_MODE"


def test_leaving_real_mode_ends_the_release(server):
    release(server)
    assert link(server, "GET", "/link/v1/status")[1]["mode"] == "auto"
    assert call(server, "POST", "/api/robots/mode", {"mode": "visual"})["mode"] == "visual"
    st = link(server, "GET", "/link/v1/status")[1]
    assert st["mode"] == "manual" and st["not_ready"] == "MANUAL_MODE"
    assert call(server, "GET", "/api/link")["ended_why"] == "the mode left Real robot"
    assert call(server, "POST", f"{P}/workflow/release", {"unsimulated_ok": True, "confirmed": True})["code"] == "mode"
