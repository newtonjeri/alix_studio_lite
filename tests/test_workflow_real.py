"""S2's Real Machine Mode (feedback v5, slide 9) on ALIX's emulated arms: only after the same run
completed in Simulation; each arm's part of each stage through runner v14's own
run_workflow_sequence on its controller, as V53 runs it; the lid checkpoint asks the operator; STOP
interrupts the runner the way V53's STOP does (its own Ctrl+C path) and latches the arms."""
import json
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
                            cwd=str(vendor_paths.LITE_ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
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
    assert call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True})["code"] == "simulate"
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    sim = j["result"]
    assert sim["start_from"] == {"left": "now", "right": "now"} and sim["points"]["left"]          # read from the controllers
    la = sim["acts"]["left"]
    assert [a["kind"] for a in la] == ["pole", "hand", "global", "program"] and la[2]["drawn"] and la[3]["drawn"]
    assert not [f for f in sim["findings"] if "program 71" in f["text"]]                         # its stage is the steps'
    refused = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True})
    assert refused["code"] == "simulate" and "not been played to its end" in refused["error"]
    call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})
    first = call(server, "POST", f"{P}/workflow/run", {"mode": "both"})
    assert first["code"] in ("warnings", "confirm")
    job = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    j = wait(server, job)
    assert j["state"] == "done", j
    assert j["extra"]["live"] == {"left": "SEQUENCE COMPLETE", "right": "SEQUENCE COMPLETE"}
    assert j["extra"]["status"].startswith("PAGE 02 PROGRAM RULES COMPLETE")
    said = events(j)
    assert "LEFT: WORKFLOW COMPLETE: 4/4 step(s)" in said and "RIGHT: WORKFLOW COMPLETE: 3/3 step(s)" in said
    hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
    runs = [r for r in hw if r["verb"] == "workflow"]
    assert len(runs) == 2 and all(r["ok"] for r in runs)
    assert any("WORKFLOW SPEED RESTORE" in (r["text"] or "") for r in hw if r["verb"] == "runner v14")

    call(server, "PUT", f"{P}/workflow/right", {"steps": steps(tail=[{"type": "wait", "seconds": 1}])})     # changed after
    again = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    assert again["code"] == "simulate" and "its steps" in again["error"]


def test_the_lid_checkpoint_asks_and_stop_interrupts_the_runner(server):
    lid = {"type": "wait", "seconds": 0.5}
    long_wait = {"type": "wait", "seconds": 60}
    call(server, "PUT", f"{P}/workflow/left", {"steps": steps(lid, tail=[long_wait])})
    call(server, "PUT", f"{P}/workflow/right", {"steps": steps(lid, tail=[long_wait])})
    plan = call(server, "POST", f"{P}/workflow/plan", {"mode": "both"})
    assert [s["label"] for s in plan["stages"]] == ["LID-PREP-1", "LID-CHECK-1", "TIMELINE-1", ""]
    sim = simulated(server)
    job = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    j = wait(server, job, until=lambda j: j["extra"].get("question"))
    q = j["extra"]["question"]
    assert q["title"] == "MANUAL LID COVER CHECKPOINT" and "Open the toilet lid cover manually" in q["text"]
    assert j["extra"]["live"]["left"] == "PAUSED // MANUAL LID CHECK"
    call(server, "POST", f"/api/jobs/{job['id']}/answer", {"question": q["id"], "yes": True})
    j = wait(server, job, until=lambda j: "FINAL AUX" in j["extra"]["live"]["left"] and "Wait 60 s" in j["extra"]["live"]["left"])
    assert j["state"] == "running", j
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
