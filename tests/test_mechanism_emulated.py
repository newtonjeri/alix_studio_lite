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
    assert [t["name"] for t in cat["tasks"]] == ["lid_open", "seat_close_right", "seat_open", "lid_close_right"]
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
    retract = [r for r in streams if "move_to_retract" in r["args"]["label"]]
    assert retract and "bridged" in retract[0]["text"]          # 7.7 degrees from rung 1's end, by ALIX's MoveJ
    assert all(r["args"]["period_ms"] == 5.0 and r["args"]["rate_gate"] == "per_joint" for r in streams)
    caps = list((Path(server["data"]) / "logs" / time.strftime("%Y-%m-%d") / "canfd").glob("*.frames.jsonl"))
    assert len(caps) == len(streams)                            # each stream's 5 ms capture is kept


def test_stop_in_the_middle_of_a_stream_stops_sending(server):
    job = call(server, "POST", "/api/mechanism/lid_open/run", {"confirmed": True})
    running = wait(server, job, until=lambda j: "move_to_inspect" in (j.get("stage") or "") and "move_to_inspect_pole" not in (j.get("stage") or ""))
    assert running["state"] == "running", running
    time.sleep(0.15)
    call(server, "POST", "/api/robots/stop-both", {})
    done = wait(server, job, limit=60)
    assert done["state"] in ("cancelled", "failed"), done
    last = [r for r in hardware(server) if r.get("verb") == "canfd stream"][-1]
    assert not last["ok"] and int(last["text"].split("/")[0]) < last["args"]["lines"]
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
    right = sim["acts"]["right"]                                 # the final check drawn as it runs: Open the seat's way in
    assert any(a["kind"] == "stream" and a.get("file") == "mechanism:seat_open:2" for a in right)
    assert any(a["kind"] == "check" and "the final check" in a["text"] for a in right)
    looks = [a for a in right if a["kind"] == "stream" and a.get("file") == "mechanism:seat_open:2"][0]
    assert looks["t0"] > next(t for t, ph in sim["phases"] if ph == "lid_open")      # with the lid open
    assert any("left out of this collision check" in f["text"] for f in sim["findings"])
    assert call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})["completed"]
    server["script"].write_text(json.dumps(["lid_closed", "lid_lifted", "seat_closed", "lid_open"]))
    job = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    done = wait(server, job)
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-6:])
    said = " | ".join(e["text"] for e in done["events"])
    assert "Open the lid: Open the lid done." in said and "CHECK verify_lid_lifted" in said


def test_s2_puts_each_arms_pole_and_hand_back_after_its_lid_and_seat_tasks(server):
    """The owner, 22 Oct: after the lid and seat opening tasks each arm's pole and hand go back to where they were
    before them, and the dual-arm tasks after start on both arms together: in Simulation (the stage ends with each
    arm's Pole and Hand back, the next stage starting both at once) and on the robot."""
    lid = {"type": "mechanism", "task": "lid_open"}
    left = [{"type": "pole", "mm": 120}, {"type": "hand", "values": [1000, 1000, 1000, 1000, 0, 1000]}, lid, {"type": "wait", "seconds": 0.5}]
    right = [{"type": "pole", "mm": 150}, {"type": "hand", "values": [0, 0, 0, 0, 150, 1000]}, lid, {"type": "wait", "seconds": 0.5}]
    call(server, "PUT", f"{P}/workflow/left", {"steps": left})
    call(server, "PUT", f"{P}/workflow/right", {"steps": right})
    plan = call(server, "POST", f"{P}/workflow/plan", {"mode": "both"})
    assert [s["kind"] for s in plan["stages"]] == ["parallel", "mechanism", "parallel"]
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    assert j["state"] == "done", j
    sim = j["result"]
    assert "pole and hand back to before Open the lid" in sim["stages"][1]["title"]
    for side, pole, hand in (("left", 120.0, left[1]["values"]), ("right", 150.0, right[1]["values"])):
        mine = [a for a in sim["acts"][side] if a["stage"] == 1]
        assert [a["kind"] for a in mine[-2:]] == ["pole", "hand"], [a["kind"] for a in mine[-4:]]
        assert mine[-2]["pole1"] == pole and mine[-1]["hand1"] == hand
    nxt = {side: min(a["t0"] for a in sim["acts"][side] if a["stage"] == 2) for side in ("left", "right")}
    assert nxt["left"] == pytest.approx(nxt["right"], abs=1e-6)          # both arms start the next stage together
    assert call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})["completed"]
    server["script"].write_text(json.dumps(["lid_closed", "lid_lifted", "seat_closed", "lid_open"]))
    job = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    done = wait(server, job)
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-6:])
    said = " | ".join(e["text"] for e in done["events"])
    assert "LEFT: pole 120 mm and hand 1000,1000,1000,1000,0,1000 back, as before the tasks." in said
    assert "RIGHT: pole 150 mm and hand 0,0,0,0,150,1000 back, as before the tasks." in said
    arms = call(server, "GET", "/api/robots/status")["arms"]
    assert abs(arms["left"]["link"]["feedback"]["pole_mm"] - 120) < 3 and abs(arms["right"]["link"]["feedback"]["pole_mm"] - 150) < 3

