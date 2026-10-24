"""The lid and the seat in Real on ALIX's emulated arms: ALIX's lid_open plan streamed over each arm's CanFD
pass-through at 5 ms (alix_dispatch's lane: the 5 ms capture, its watch), the pole and the hand, a re-lift from the
pre-planned rung and the bridged move after it, and STOP in the middle of a stream. The camera's answers are
scripted (LITE_CHECK_SCRIPT, a test harness only): what the gap check decides is tests/test_gap_check.py's."""
import json
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import pytest

from alix_studio_lite import vendor_paths

PID = "alix_lid"
P = f"/api/projects/{PID}"


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    data = tmp_path_factory.mktemp("lite-mech")
    script = data / "answers.json"
    # lid_open: closed (it runs); the lift missed (re-lift 1 from rung 1); lifted; the seat stayed down; its final check
    # sees the lid open. Then the STOP test's lid_open: closed, then STOP before its next question.
    script.write_text(json.dumps(["lid_closed", "lid_closed", "lid_lifted", "seat_closed", "lid_open", "lid_closed"]))
    env = dict(__import__("os").environ, LITE_CHECK_SCRIPT=str(script))
    proc = subprocess.Popen([sys.executable, str(Path(__file__).with_name("emulated_lite.py")), "--robot", "emulated",
                             "--no-browser", "--port", str(port), "--data", str(data), "--logs", str(data / "logs"),
                             "--projects", str(data / "projects")],
                            cwd=str(vendor_paths.LITE_ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
    ctx = {"base": f"http://127.0.0.1:{port}", "data": data}
    for _ in range(150):
        try:
            ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
            break
        except OSError:
            time.sleep(0.2)
    ctx["script"] = script
    call(ctx, "PUT", P, {"id": PID, "name": "Lid", "archived": False, "state": {"currentTask": 0}})
    for side in ("left", "right"):
        assert call(ctx, "POST", f"/api/robots/{side}/connect", {})["connected"]
    assert call(ctx, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})["mode"] == "real"
    yield ctx
    proc.terminate()
    proc.wait(10)


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json"}
    if ctx.get("tok"):
        headers["X-Lite-Token"] = ctx["tok"]
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=120) as resp:
        return json.loads(resp.read())


def refused(ctx, method, path, body=None) -> dict:
    out = call(ctx, method, path, body)                         # Lite answers a refusal with its words, not an HTTP error
    assert "error" in out, out
    return out


def wait(ctx, job, limit=300, until=None):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running" or (until and until(j)):
            return j
        assert time.time() - t0 < limit, j
        time.sleep(0.2)


def hardware(ctx) -> list:
    day = time.strftime("%Y-%m-%d")
    path = Path(ctx["data"]) / "logs" / day / "hardware.jsonl"
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def test_the_catalogue_lists_the_four_tasks(server):
    cat = call(server, "GET", "/api/mechanism")
    assert [t["name"] for t in cat["tasks"]] == ["lid_open", "seat_close", "seat_open", "lid_close"]
    assert cat["standoff_mm"] == 250 and cat["rows"]["L-T01"] == "lid_open"


def test_real_lid_open_streams_each_move_and_re_lifts_from_rung_1(server):
    said = refused(server, "POST", "/api/mechanism/lid_open/run", {})
    assert said.get("code") == "confirm"                         # Real asks first, as every motion does
    job = call(server, "POST", "/api/mechanism/lid_open/run", {"confirmed": True})
    done = wait(server, job)
    from alix_studio_lite.server import mechanism

    if mechanism.Plan("lid_open").rungs_note():          # the vendored re-lifts planned for another version of the plan
        assert done["state"] == "failed" and "planned for another version of Open the lid" in done["error"], done.get("error")
        labels = [r["args"]["label"] for r in hardware(server) if r.get("verb") == "canfd stream"]
        assert labels and not any("re-lift" in x for x in labels)              # stopped before any of it moved
        server["script"].write_text(json.dumps(["lid_closed"]))                # the STOP test's answer, from the first
        return
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-8:])
    res = done["result"]
    assert [(p["plan"], p["outcome"], p["relifts"]) for p in res["plans"]] == [("lid_open", "done", 1)]
    assert [c["verdict"] for c in res["checks"]] == ["PASS", "NO", "PASS", "no", "PASS"]
    assert res["checks"][-1]["check"] == "verify_lid_open_after" and res["checks"][-1]["final"]     # the end state, seen
    said = " | ".join(e["text"] for e in done["events"])
    assert "the right arm to its inspection pose (Open the seat's way in)" in said and "at where Open the lid ended" in said
    streams = [r for r in hardware(server) if r.get("verb") == "canfd stream"]
    assert streams and all(r["ok"] for r in streams), [r.get("text") for r in streams if not r["ok"]]
    labels = [r["args"]["label"] for r in streams]
    assert any("re-lift 1, pre-planned" in x for x in labels)
    # the left's next move after the re-lift (since 24 Oct 21:39 its way out, after the right's push) starts where the
    # ORIGINAL lift ended: 148.8 degrees from rung 1's end, bridged by ALIX's MoveJ
    left = [r for r in streams if r["args"]["label"].startswith("LEFT")]
    after = left[max(i for i, r in enumerate(left) if "re-lift 1" in r["args"]["label"]) + 1]
    assert "bridged" in after["text"], after["args"]["label"]
    assert all(r["args"]["period_ms"] == 5.0 and r["args"]["rate_gate"] == "per_joint" for r in streams)
    # each arm's 5 ms feed on once for the run (24 Oct: on and off around every move, 1.1 s each), every move watched on
    # it over its own stretch, and off at the end; an arm's back-to-back moves are one stream
    caps = sorted(p.name for p in (Path(server["data"]) / "logs" / time.strftime("%Y-%m-%d") / "canfd").glob("*.frames.jsonl"))
    assert sorted(c.split("_", 2)[2] for c in caps) == ["LEFT_lid_open.frames.jsonl", "RIGHT_lid_open.frames.jsonl"], caps
    assert {r["args"]["capture"] for r in streams} == set(caps) and all(len(r["args"]["t_mono"]) == 2 for r in streams)
    feeds = [r for r in hardware(server) if r.get("verb") == "canfd feed"]
    assert sorted(r["side"] for r in feeds if "kept for the run" in (r["text"] or "")) == ["left", "right"]
    assert sorted(r["side"] for r in feeds if (r["text"] or "").startswith("off")) == ["left", "right"] and all(r["ok"] for r in feeds)
    assert "LEFT arm move_to_pre_start + move_to_start + execute_path" in labels
    # ALIX's emulator reports a 100 % global ratio, so each stream sets 50 and puts 100 back (the robot's controllers
    # report none: 50 %, held across the run's moves; tests/test_robot_real.py)
    assert not any(r["args"]["ratio_held"] for r in streams)
    assert any(x.startswith("RIGHT arm back along move_to_inspect") for x in labels)   # the final check, back along its way in
    hands = [r for r in hardware(server) if r.get("verb") == "hand move" and isinstance(r["args"]["speed_set"], list)]
    assert hands and all(len(r["args"]["speed_set"]) == 6 for r in hands)   # the plan's hand moves: ALIX's executive's speeds


def test_real_seat_close_pulls_with_the_left_at_full_pole_and_the_right_lets_the_seat_down(server):
    """25 Oct, the compound Close the seat on the emulated arms: two camera questions (the seat up; after all the
    moves, the seat down), no pole move (150 mm throughout) and no hand command (the hands as they are: the sequence's
    rule), the left's pull, the right's hold and release."""
    server["script"].write_text(json.dumps(["seat_lifted", "seat_closed"]))
    before = len(hardware(server))                                         # this run's rows alone (the server is shared)
    job = call(server, "POST", "/api/mechanism/seat_close/run", {"confirmed": True})
    done = wait(server, job)
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-8:])
    res = done["result"]
    assert [(p["plan"], p["outcome"]) for p in res["plans"]] == [("seat_close", "done")]
    assert [(c["check"], c["verdict"]) for c in res["checks"]] == [("verify_seat_open", "PASS"), ("verify_seat_closed", "PASS")]
    assert not [c for c in res["checks"] if c.get("final")]
    rows = hardware(server)[before:]
    streams = [r["args"]["label"] for r in rows if r.get("verb") == "canfd stream"]
    # the right's release runs on into its last look (one stream: nothing between them); the end check rides
    # move_to_rest, so rest is its own stream after the check
    assert streams[:1] == ["RIGHT arm move_to_inspect"]
    assert streams[-2:] == ["RIGHT arm execute_path + move_to_inspect", "RIGHT arm move_to_rest"]
    assert any(lab.startswith("LEFT arm") and "execute_path" in lab for lab in streams)
    assert any(lab.startswith("RIGHT arm") and "execute_path" in lab for lab in streams)
    assert not [r for r in rows if r.get("verb") == "pole move"], "no pole command: 150 mm throughout"
    assert not [r for r in rows if r.get("verb") in ("hand move", "pole and hand", "hand open")], "no hand command: the hands stay as they are"
    said = " | ".join(e["text"] for e in done["events"])
    assert "its own last check looked at the end state" in said
    server["script"].write_text(json.dumps(["lid_closed"]))                # the STOP test's answer, from the first


def test_stop_in_the_middle_of_a_stream_stops_sending(server):
    before = len(hardware(server))
    job = call(server, "POST", "/api/mechanism/lid_open/run", {"confirmed": True})
    running = wait(server, job, until=lambda j: "move_to_inspect" in (j.get("stage") or "") and "move_to_inspect_pole" not in (j.get("stage") or ""))
    assert running["state"] == "running", running
    deadline = time.monotonic() + 30                             # ALIX's "CANFD: N lines ..." comes as the sending starts
    while not any(str(r.get("text") or "").startswith("CANFD:") for r in hardware(server)[before:]):
        assert time.monotonic() < deadline, "the stream never started"
        time.sleep(0.02)
    time.sleep(0.3)                                              # some of its 200 lines (1 s) sent
    call(server, "POST", "/api/robots/stop-both", {})
    done = wait(server, job, limit=60)
    assert done["state"] in ("cancelled", "failed"), done
    last = [r for r in hardware(server) if r.get("verb") == "canfd stream"][-1]
    sent = int(last["text"].split("/")[0])
    assert not last["ok"] and 0 < sent < last["args"]["lines"], last  # the lines it sent, not "0/200" (23 Oct, the lab)
    for side in ("left", "right"):                               # STOP holds the arms until Recover, as everywhere
        call(server, "POST", f"/api/robots/{side}/recover", {})


def test_s2_runs_a_lid_open_put_in_its_sequence(server):
    """The owner, 18 Oct: a pre-recorded task injected at a point in the sequence: Open the lid on both arms' lists,
    simulated (its stage on the run's clock, the lid open after it, left out of the collision check and said so), then
    run in Real by S2 on the emulated arms."""
    lid = {"type": "mechanism", "task": "lid_open"}
    wait_1 = {"type": "wait", "seconds": 1}
    call(server, "PUT", f"{P}/workflow/left", {"steps": [lid, wait_1]})
    call(server, "PUT", f"{P}/workflow/right", {"steps": [lid]})
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    assert j["state"] == "done", j
    sim = j["result"]
    assert [s["kind"] for s in sim["stages"]] == ["mechanism", "parallel"] and sim["counts"]["mechanisms"] == 1   # then the left's Wait
    kinds = {a["kind"] for side in ("left", "right") for a in sim["acts"][side]}
    assert {"stream", "pole", "hand", "global", "check"} <= kinds
    assert sim["phases"][-1][1] == "lid_open"
    right = sim["acts"]["right"]                 # drawn as it runs: the seat under the lifted lid not judged, Close the seat
    assert any(a["kind"] == "stream" and a.get("file") == "mechanism:seat_close:0" for a in right)   # looks first
    assert [a["expect"] for a in right if a["kind"] == "check"][-1] == "seat_open"                   # and ends there
    looks = [a for a in right if a["kind"] == "stream" and a.get("file") == "mechanism:seat_close:0"][0]
    assert looks["t0"] > next(t for t, ph in sim["phases"] if ph == "lid_open")      # with the lid open
    assert any("Close the seat runs after it and looks first" in f["text"] for f in sim["findings"])
    assert any("left out of this collision check" in f["text"] for f in sim["findings"])
    assert call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})["completed"]
    server["script"].write_text(json.dumps(["lid_closed", "lid_lifted", "seat_closed", "lid_open"]))
    job = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    done = wait(server, job)
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-6:])
    said = " | ".join(e["text"] for e in done["events"])
    assert "Open the lid: Open the lid done." in said and "CHECK verify_lid_lifted" in said


def test_s2_refuses_a_lid_or_seat_task_with_no_camera_before_anything_moves(server):
    """The owner, 23 Oct: with no camera every check read NOT, and Open the lid's "is it closed?" NOT ended it as done,
    the lid closed (the lab, 14:38). A run with a lid or seat task now needs the wrist camera before anything moves.
    Here the harness's answers are taken away and this computer has no D435."""
    lid = {"type": "mechanism", "task": "lid_open"}
    call(server, "PUT", f"{P}/workflow/left", {"steps": [lid]})
    call(server, "PUT", f"{P}/workflow/right", {"steps": [lid]})
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    assert j["state"] == "done", j
    sim = j["result"]
    assert call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})["completed"]
    before = len(hardware(server))
    away = server["script"].with_suffix(".away")
    server["script"].rename(away)
    try:
        out = refused(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    finally:
        away.rename(server["script"])
    assert out["code"] == "camera" and "wrist camera cannot be read" in out["error"] and "Nothing moved" in out["error"], out
    moved = [r for r in hardware(server)[before:] if r.get("verb") in ("canfd stream", "stage", "move to pose", "pole move",
                                                                      "hand move", "workflow")]
    assert not moved, moved


def test_s2_after_a_lid_task_with_a_wait_next_each_arm_stays_where_its_plan_ended(server):
    """The owner, 24 Oct: after a lid or seat task each arm goes to its next task's start (in place of 22 Oct's pole and
    hand put back as before the tasks). With a Wait next (no row that moves the arm) there is no start to go to: nothing
    is put back, the arms stay where the plan left them, and the stage after starts both arms together, in Simulation
    and on the robot. (A Program next: test_s2_lid_emulated.py.)"""
    lid = {"type": "mechanism", "task": "lid_open"}
    left = [{"type": "pole", "mm": 120}, {"type": "hand", "values": [1000, 1000, 1000, 1000, 0, 1000]}, lid, {"type": "wait", "seconds": 0.5}]
    right = [{"type": "pole", "mm": 150}, {"type": "hand", "values": [0, 0, 0, 0, 150, 1000]}, lid, {"type": "wait", "seconds": 0.5}]
    call(server, "PUT", f"{P}/workflow/left", {"steps": left})
    call(server, "PUT", f"{P}/workflow/right", {"steps": right})
    plan = call(server, "POST", f"{P}/workflow/plan", {"mode": "both"})
    assert [s["kind"] for s in plan["stages"]] == ["parallel", "mechanism", "parallel"] and plan["stages"][1]["title"] == "Open the lid"
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    assert j["state"] == "done", j
    sim = j["result"]
    assert sim["stages"][1]["title"] == "Open the lid" and sim["stages"][1]["t1_plan"] == sim["stages"][1]["t1"]
    nxt = {side: min(a["t0"] for a in sim["acts"][side] if a["stage"] == 2) for side in ("left", "right")}
    assert nxt["left"] == pytest.approx(nxt["right"], abs=1e-6)          # both arms start the next stage together
    assert call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})["completed"]
    server["script"].write_text(json.dumps(["lid_closed", "lid_lifted", "seat_closed", "lid_open"]))
    job = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    done = wait(server, job)
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-6:])
    said = " | ".join(e["text"] for e in done["events"])
    assert "back, as before the tasks" not in said and "at the start of" not in said
    arms = call(server, "GET", "/api/robots/status")["arms"]
    assert abs(arms["left"]["link"]["feedback"]["pole_mm"] - 150) < 3 and abs(arms["right"]["link"]["feedback"]["pole_mm"] - 150) < 3


