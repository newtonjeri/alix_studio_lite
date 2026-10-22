"""S2's Real Machine Mode (feedback v5, slide 9) on ALIX's emulated arms: only after the same run
completed in Simulation; each arm's part of each stage through runner v14's own
run_workflow_sequence on its controller, a call per stretch between its Wait-for rows; a Wait idles its own arm; a
Wait-for holds its arm until the other arm's runner has finished the row it names, and a Meet holds the first arm there
for the other (the owner, 22 Oct); STOP interrupts the runner the way V53's STOP does (its own Ctrl+C path) and latches
the arms."""
import json
import os
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np
import pytest

from alix_studio_lite import vendor_paths

PID = "alix_wf"
P = f"/api/projects/{PID}"
GRIP = [0, 0, 0, 0, 150, 1000]


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    data = tmp_path_factory.mktemp("lite-wf")
    proc = subprocess.Popen([sys.executable, str(Path(__file__).with_name("emulated_lite.py")), "--robot", "emulated",
                             "--no-browser", "--port", str(port), "--data", str(data), "--logs", str(data / "logs"), "--projects", str(data / "projects")],
                            cwd=str(vendor_paths.LITE_ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            env=dict(os.environ, LITE_EMU_4105=str(data / "emu_4105")))     # the lab's 4105 (emulated_lite)
    ctx = {"base": f"http://127.0.0.1:{port}", "data": data}
    for _ in range(150):
        try:
            ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
            break
        except OSError:
            time.sleep(0.2)
    call(ctx, "PUT", P, {"id": PID, "name": "WF", "archived": False, "state": {"currentTask": 0}})
    for side in ("left", "right"):
        assert call(ctx, "POST", f"/api/robots/{side}/connect", {})["connected"]
    assert call(ctx, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})["mode"] == "real"
    for side, key, name, j1 in (("left", "L-R01", "L_wf.txt", 8.0), ("right", "R-R02", "R_wf.txt", -8.0)):
        program(ctx, side, key, name, j1)
    yield ctx
    proc.terminate()
    proc.wait(10)


def call(ctx, method, path, body=None, tok=True):
    headers = {"Content-Type": "application/json"}
    if tok and ctx.get("tok"):
        headers["X-Lite-Token"] = ctx["tok"]
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=120) as resp:
        return json.loads(resp.read())


def wait(ctx, job, limit=240, until=None):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running" or (until and until(j)):
            return j
        assert time.time() - t0 < limit, j
        time.sleep(0.15)


def program(ctx, side, key, name, j1):
    """A short file of the arm as program 71 on its controller, sent by Lite (S1: merged, then uploaded)."""
    from alix_studio_lite.motion import track

    fb = call(ctx, "GET", "/api/robots/status")["arms"][side]["link"]["feedback"]
    t = np.linspace(0.0, 1.0, 240)[:, None]
    q0 = np.asarray(fb["joints_deg"][:7], dtype=float)
    md = np.round((q0 + np.array([[j1, 0, 0, 0, 0, 0, 0]]) * np.sin(np.pi * t)) * 1000).astype(np.int64)
    out = call(ctx, "POST", f"{P}/units/{key}/recordings", {"source": "upload", "name": name, "content": track.track_bytes(md).decode(),
                                                            "pole_mm": 150, "hand": "GRIP"})
    assert "error" not in out, out
    item = next(i for i in call(ctx, "POST", f"{P}/merge-sources/{side}", {})["items"] if i["name"] == name)
    m = call(ctx, "POST", f"{P}/merges/{side}/from-project", {"name": name.replace(".txt", "_m"), "files": [item["ref"]]})
    j = wait(ctx, call(ctx, "POST", f"{P}/merges/{side}/{m['file']}/upload", {"program_id": 71, "confirmed": True, "limits_seen": True}))
    assert j["state"] == "done", j


def steps(*mid, tail=()):
    return [{"type": "pole", "mm": 150}, {"type": "hand", "values": GRIP}, *mid,
            {"type": "program", "id": 71, "transit_speed": 30, "dual_rule": "TOGETHER"}, *tail]


def simulated(ctx, mode="both"):
    j = wait(ctx, call(ctx, "POST", f"{P}/workflow/simulate", {"mode": mode}))
    assert j["state"] == "done", j
    assert call(ctx, "POST", f"{P}/workflow/played", {"hash": j["result"]["hash"]})["completed"]
    return j["result"]


def events(job):
    return " | ".join(e["text"] for e in job["events"])


def test_real_runs_the_simulated_sequence_through_runner_v14(server):
    call(server, "PUT", f"{P}/workflow/left", {"steps": steps({"type": "global", "name": "STP_HOME", "move": "movej", "speed": 30,
                                                                "wait_s": 0})})
    call(server, "PUT", f"{P}/workflow/right", {"steps": steps()})
    first = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True})
    assert first["code"] == "unsimulated" and "it has not been simulated" in first["error"]     # asked, not refused (17 Oct)
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    sim = j["result"]
    assert sim["start_from"] == {"left": "now", "right": "now"} and sim["points"]["left"]          # read from the controllers
    la = sim["acts"]["left"]
    assert [a["kind"] for a in la] == ["pole", "hand", "wait", "global", "program"] and la[3]["drawn"] and la[4]["drawn"]
    assert "4105" in la[2]["text"]                               # the run's start: its errors read before the move
    assert not [f for f in sim["findings"] if "program 71" in f["text"]]                         # its stage is the steps'
    refused = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True})
    assert refused["code"] == "unsimulated" and "not played to its end" in refused["error"]
    call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})
    first = call(server, "POST", f"{P}/workflow/run", {"mode": "both"})
    assert first["code"] in ("warnings", "confirm")
    job = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    j = wait(server, job)
    assert j["state"] == "done", j
    assert j["extra"]["live"] == {"left": "SEQUENCE COMPLETE", "right": "SEQUENCE COMPLETE"}
    assert j["extra"]["status"].startswith("PAGE 02 SEQUENCE COMPLETE")
    said = events(j)
    assert "LEFT: WORKFLOW COMPLETE: 2/2 step(s)" in said and "RIGHT: WORKFLOW COMPLETE: 3/3 step(s)" in said
    hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
    runs = [r for r in hw if r["verb"] == "workflow"]
    assert len(runs) == 3 and all(r["ok"] for r in runs)       # the left's pole and hand, then its clear, then the rest
    assert any("WORKFLOW SPEED RESTORE" in (r["text"] or "") for r in hw if r["verb"] == "runner v14")

    call(server, "PUT", f"{P}/workflow/right", {"steps": steps(tail=[{"type": "wait", "seconds": 1}])})     # changed after
    again = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    assert again["code"] == "unsimulated" and "its steps" in again["error"]
    asked = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "unsimulated_ok": True})        # the first answer given
    assert asked["code"] == "confirm"                                                         # then the real-robot one
    j = wait(server, call(server, "POST", f"{P}/workflow/run", {"mode": "both", "unsimulated_ok": True, "confirmed": True}))
    assert j["state"] == "done", j
    assert "without a simulation of it, as confirmed" in events(j)


def test_a_wait_idles_its_own_arm_and_stop_interrupts_the_runner(server):
    """A Wait is a task of its own (the owner, 21 Oct): on one arm only, before a TOGETHER Program (the lab's pop-up:
    V53 refused it, "SAFE TIMELINE checkpoint mismatch"), it idles that arm there, and nothing stops for the operator.
    STOP cuts a long Wait short and latches the arms."""
    hold = {"type": "wait", "seconds": 0.5}
    long_wait = {"type": "wait", "seconds": 60}
    call(server, "PUT", f"{P}/workflow/left", {"steps": steps(hold, tail=[long_wait])})
    call(server, "PUT", f"{P}/workflow/right", {"steps": steps(tail=[long_wait])})
    plan = call(server, "POST", f"{P}/workflow/plan", {"mode": "both"})
    assert [s["label"] for s in plan["stages"]] == ["PART 1"] and plan["counts"]["parts"] == 1
    sim = simulated(server)
    held = [a for a in sim["acts"]["left"] if a["kind"] == "wait" and a["stage"] == 0]
    assert held and held[0]["t1"] - held[0]["t0"] == pytest.approx(0.5, abs=0.01)       # the left arm idles 0.5 s there
    job = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    j = wait(server, job, until=lambda j: "Wait 60 s" in j["extra"]["live"]["left"])
    assert j["state"] == "running" and not j["extra"].get("question"), j                # no operator question
    m = next(x for x in call(server, "GET", f"{P}/merges")["left"])                 # nothing is sent to an arm that runs
    busy = call(server, "POST", f"{P}/merges/left/{m['file']}/upload", {"program_id": 72, "confirmed": True, "limits_seen": True})
    assert busy["code"] == "busy" and "S2 Real Machine" in busy["error"]
    t0 = time.time()
    call(server, "POST", "/api/robots/stop-both", {}, tok=False)
    j = wait(server, job, limit=15)
    assert j["state"] == "cancelled" and time.time() - t0 < 5, j                  # not the 60 s wait
    assert "WORKFLOW STOP REQUESTED" in events(j)
    st = call(server, "GET", "/api/robots/status")["arms"]
    assert st["left"]["estop"] == "RECOVERING" and st["right"]["estop"] == "RECOVERING"     # latched until Recover
    for side in ("left", "right"):
        assert call(server, "POST", f"/api/robots/{side}/recover", {})["estop"] == "CLEAR"


def test_a_task_after_waits_on_both_arms_runs_with_no_question(server):
    """Waits on both arms before the Program (V53's manual lid cover checkpoint until 21 Oct) are each arm's own idle:
    a Single task picked after them runs as it is, with nothing asked first."""
    hold = {"type": "wait", "seconds": 0.5}
    call(server, "PUT", f"{P}/workflow/left", {"steps": steps(hold)})
    call(server, "PUT", f"{P}/workflow/right", {"steps": steps(hold)})
    left = call(server, "GET", f"{P}/workflow")["left"]
    row = next(i + 1 for i, st in enumerate(left) if st["type"] == "program")
    pick = {"scope": "task", "side": "left", "row": row, "id": 71}
    plan = call(server, "POST", f"{P}/workflow/plan", {"mode": "left", "pick": pick})
    assert [s["kind"] for s in plan["stages"]] == ["single"]
    full = call(server, "POST", f"{P}/workflow/plan", {"mode": "both"})
    assert [s["kind"] for s in full["stages"]] == ["parallel"] and full["counts"]["meets"] == 0
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "left", "pick": pick}))
    assert j["state"] == "done", j
    call(server, "POST", f"{P}/workflow/played", {"hash": j["result"]["hash"]})
    job = call(server, "POST", f"{P}/workflow/run", {"mode": "left", "pick": pick, "confirmed": True, "warnings_seen": j["result"]["hash"]})
    j = wait(server, job)
    assert j["state"] == "done" and not j["extra"].get("question"), j
    assert "WORKFLOW STEP START" in events(j)


def test_a_global_point_after_a_program_clears_the_4105_it_left(server):
    """The lab, 21 Oct 22:29 and 22:41, 22 Oct 21:12: the Home MoveJ after the last Program returned 1 at once and the
    sequence failed at its last row, though the arm went Home; each Program had raised 4105, which runner v14 lets stand,
    and the SDK's blocking move returns 1 while any code is latched (emulated here as on the robot: emulated_lite). Lite
    reads the arm's errors before that move and clears a 4105 alone (a clear step between two runner calls, in Simulation
    and on the robot), and the sequence runs to its end."""
    flag = server["data"] / "emu_4105"
    home = {"type": "global", "name": "STP_HOME", "move": "movej", "speed": 30, "wait_s": 0}
    try:
        flag.write_text("left")
        left = call(server, "GET", f"{P}/workflow")["left"]
        call(server, "PUT", f"{P}/workflow/left", {"steps": steps()})       # the lab's failure, first: a Program alone...
        row = next(i + 1 for i, st in enumerate(steps()) if st["type"] == "program")
        pick = {"scope": "task", "side": "left", "row": row, "id": 71}
        j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "left", "pick": pick}))
        call(server, "POST", f"{P}/workflow/played", {"hash": j["result"]["hash"]})
        j = wait(server, call(server, "POST", f"{P}/workflow/run", {"mode": "left", "pick": pick, "confirmed": True,
                                                                   "warnings_seen": j["result"]["hash"]}))
        assert j["state"] == "done", j
        assert call(server, "GET", "/api/robots/status")["arms"]["left"]["link"]["feedback"]["errors"] == [4105]
        moved = wait(server, call(server, "POST", "/api/robots/left/points/move", {"name": "STP_HOME", "move": "movej",
                                                                                  "confirmed": True}))
        assert moved["state"] == "failed" and "ret=1" in moved["error"], moved      # ...then a MoveJ: 1, as in the lab
        call(server, "PUT", f"{P}/workflow/left", {"steps": steps(tail=[home])})
        call(server, "PUT", f"{P}/workflow/right", {"steps": steps()})
        assert [s["kind"] for s in call(server, "POST", f"{P}/workflow/plan", {"mode": "both"})["stages"]] == ["parallel"]
        sim = simulated(server)
        tail = [a for a in sim["acts"]["left"] if a["row"] == 4]
        assert [a["kind"] for a in tail] == ["wait", "global"] and "4105" in tail[0]["text"]
        assert tail[0]["t1"] - tail[0]["t0"] < 1.0                                  # no 2 s wait for the Program's end
        t0 = time.time()
        j = wait(server, call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]}))
        assert j["state"] == "done", j
        assert "LEFT row 4: cleared 4105" in events(j)
        hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
        said = [r["text"] for r in hw if r["verb"] == "runner v14" and r["side"] == "left" and r["t"] >= t0]
        g = next(i for i, s in enumerate(said) if s.startswith("GLOBAL MOVE START: STP_HOME"))
        assert said[g - 1].startswith("WORKFLOW STEP START 1/1: GLOBAL")             # a runner call of its own, after the clear
        assert any(s.startswith("GLOBAL MOVE COMPLETE: STP_HOME") for s in said[g:])
        cleared = [r for r in hw if r["verb"] == "clear warning" and r["side"] == "left" and r["t"] >= t0 and r.get("args")]
        assert [r["args"]["codes"] for r in cleared] == [["4105"]] and cleared[0]["ok"]
    finally:
        flag.unlink(missing_ok=True)
        call(server, "PUT", f"{P}/workflow/left", {"steps": left})
        for side in ("left", "right"):
            call(server, "POST", f"/api/robots/{side}/clear-errors", {})


def test_a_stage_that_fails_says_each_arms_errors_after_the_stop(server):
    """The lab, 22 Oct (12:55, 16:14): the LEFT arm's Program 86 ended with 4107 (a joint's brake release failed) as it
    started, and the run stopped; four times that day the RIGHT arm's Program 82 ended on its controller at that moment
    with no code the runner reads. When a stage fails, each arm's errors as its controller holds them then (its joints'
    own flags too) are said and kept in the hardware log; a code that is not a warning is not cleared."""
    flag = server["data"] / "emu_4105"
    left = call(server, "GET", f"{P}/workflow")["left"]
    try:
        flag.write_text("left 4107")
        call(server, "PUT", f"{P}/workflow/left", {"steps": steps()})
        call(server, "PUT", f"{P}/workflow/right", {"steps": steps()})
        sim = simulated(server)
        t0 = time.time()
        j = wait(server, call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]}))
        assert j["state"] == "failed" and "LEFT rc=1" in j["error"], j
        said = events(j)
        assert "LEFT: ** WORKFLOW PROGRAM NOT COMPLETE 3/3: 4107" in said
        assert "LEFT controller's errors after the stop: system 4107" in said and "RIGHT controller's errors after the stop: clean" in said
        hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
        after = {r["side"]: r["text"] for r in hw if r["verb"] == "errors after the stop" and r["t"] >= t0}
        assert after["left"].startswith("system 4107") and after["right"] == "clean"
        st = call(server, "GET", "/api/robots/status")["arms"]["left"]["link"]["feedback"]["errors"]
        assert st == [4107]                                                     # not a warning: left for a person
    finally:
        flag.unlink(missing_ok=True)
        call(server, "PUT", f"{P}/workflow/left", {"steps": left})
        for side in ("left", "right"):
            call(server, "POST", f"/api/robots/{side}/clear-errors", {})


def test_a_wait_for_and_a_meet_order_the_two_arms_on_the_robot(server):
    """The owner, 22 Oct: the right arm's Program waits for the left's (a Wait-for naming it) and both meet after
    them. On the robot the right's Program starts only once the left's runner has said its own complete, and neither
    arm goes on from the Meet before both are there."""
    left = steps(tail=[{"type": "meet"}, {"type": "wait", "seconds": 0.3}])
    call(server, "PUT", f"{P}/workflow/left", {"steps": left})
    left = call(server, "GET", f"{P}/workflow")["left"]
    target = next(x for x in left if x["type"] == "program")
    right = steps(tail=[{"type": "meet"}, {"type": "wait", "seconds": 0.3}])
    right.insert(2, {"type": "wait_for", "target": {"uid": target["uid"]}})
    call(server, "PUT", f"{P}/workflow/right", {"steps": right})
    plan = call(server, "POST", f"{P}/workflow/plan", {"mode": "both"})
    assert [s["label"] for s in plan["stages"]] == ["PART 1", "PART 2"] and plan["counts"]["waits_for"] == 1
    sim = simulated(server)
    held = next(a for a in sim["acts"]["right"] if a.get("waits_for"))
    left_prog = next(a for a in sim["acts"]["left"] if a["kind"] == "program")
    assert held["t1"] == pytest.approx(left_prog["t1"], abs=1e-3) and "Wait for LEFT row 3" in held["text"]
    t0 = time.time()
    j = wait(server, call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]}))
    assert j["state"] == "done", j
    assert "RIGHT row 3: waits for LEFT row 3." in events(j)
    hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
    lines = [(r["t"], r["side"], r["text"]) for r in hw if r["verb"] == "runner v14" and r["t"] >= t0]
    left_done = next(t for t, side, text in lines if side == "left" and text.startswith("WORKFLOW PROGRAM COMPLETE"))
    right_start = next(t for t, side, text in lines if side == "right" and text.startswith("WORKFLOW PROGRAM "))
    assert right_start > left_done                                     # the right's Program after the left's
    ends = {side: max(t for t, s_, text in lines if s_ == side and text.startswith("WORKFLOW PROGRAM COMPLETE")) for side in ("left", "right")}
    waits = {side: min(t for t, s_, text in lines if s_ == side and text.startswith("WORKFLOW WAIT: 0.3 s")) for side in ("left", "right")}
    assert min(waits.values()) > max(ends.values())                    # the Meet: neither goes on before both are there
