"""Lite commanding the arms (server/robot_real.py), against ALIX's emulated arms: Real robot with
the link; the pole and the hand staged and read back on the controller; drag teaching recorded by
the controller and saved byte for byte; upload with the size read back; a Real run and a Hardware
sim run (the controller's SIM mode); STOP mid-run; and every command in the hardware log."""
import json
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import pytest

from alix_studio_lite import vendor_paths

U = "/api/projects/alix_real/units"


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    data = tmp_path_factory.mktemp("lite-real")
    proc = subprocess.Popen([sys.executable, str(Path(__file__).with_name("emulated_lite.py")), "--robot", "emulated",
                             "--no-browser", "--port", str(port), "--data", str(data)],
                            cwd=str(vendor_paths.LITE_ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    ctx = {"base": f"http://127.0.0.1:{port}", "data": data}
    for _ in range(150):
        try:
            ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
            break
        except OSError:
            time.sleep(0.2)
    call(ctx, "PUT", "/api/projects/alix_real", {"id": "alix_real", "name": "Real", "createdAt": "2026-10-14T10:00:00.000Z",
                                                 "updatedAt": "2026-10-14T10:00:00.000Z", "archived": False,
                                                 "state": {"currentTask": 0}})
    for side in ("left", "right"):
        assert call(ctx, "POST", f"/api/robots/{side}/connect", {})["connected"]
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


def wait(ctx, job, limit=180):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < limit, j
        time.sleep(0.2)


def text(job):
    return " | ".join(e["text"] for e in job["events"])


def test_real_stages_the_pole_and_hand_on_the_controller(server):
    assert call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})["mode"] == "real"
    setup = call(server, "GET", f"{U}/L-T01")["setup"]
    r = call(server, "POST", f"{U}/L-T01/setup", {"pole_mm": setup["pole_mm"], "hand": setup["hand"], "apply": True,
                                                    "confirmed": True})
    j = wait(server, r["job"])
    assert j["state"] == "done" and "read back on the controller" in text(j), j
    fb = call(server, "GET", "/api/robots/status")["arms"]["left"]["link"]["feedback"]      # read at once
    assert abs(fb["pole_mm"] - setup["pole_mm"]) <= 5 and fb["hand"][:4] == [500, 500, 500, 500]


def test_drag_teaching_records_on_the_controller(server):
    from emulated_lite import POINTS

    assert call(server, "POST", f"{U}/L-T01/teach/start", {"confirmed": True})["arm"]["teaching"]
    r = call(server, "POST", f"{U}/L-T01/teach/stop", {"name": "taught_here.txt"})
    assert r["take"] is None and "50 % open" in r["stage"]["text"]
    rec = call(server, "GET", f"{U}/L-T01")["recordings"][-1]
    assert rec["source"] == "teach" and rec["taught_arm"] == "left" and rec["lines"] == POINTS
    saved = next((server["data"] / "projects" / "alix_real").rglob(rec["file"]))
    raw = saved.read_bytes()                                     # as the SDK wrote it, CRLF and all
    assert raw.count(b"\r\n") == POINTS and raw.startswith(b'{"point":[')


def test_upload_and_runs_on_the_controller(server):
    u = call(server, "GET", f"{U}/L-T01")
    lib = next(x for x in u["recordings"] if x["source"] == "library")
    call(server, "POST", f"{U}/L-T01/active", {"file": lib["file"]})
    assert wait(server, call(server, "POST", f"{U}/L-T01/optimise", {}))["state"] == "done"
    assert call(server, "POST", f"{U}/L-T01/keep", {})["versions"][-1]["version"] == "v001"

    j = wait(server, call(server, "POST", f"{U}/L-T01/upload", {"program_id": 9, "confirmed": True}))
    assert j["state"] == "done" and "bytes read back" in text(j), j
    progs = {p["id"]: p for p in call(server, "GET", "/api/robots/left/programs")["programs"]}
    assert progs[9]["name"] == "9_L-T01_v001.txt" and progs[9]["sha256"]

    j = wait(server, call(server, "POST", f"{U}/L-T01/run", {"confirmed": True}), 300)
    assert j["state"] == "done" and "Controller mode REAL (read back)" in text(j) and "read back on the controller" in text(j), j
    assert call(server, "GET", f"{U}/L-T01")["last_run"]["kind"] == "real_ok"

    assert call(server, "POST", "/api/robots/mode", {"mode": "hwsim"})["mode"] == "hwsim"
    j = wait(server, call(server, "POST", f"{U}/L-T01/run", {}), 300)
    assert j["state"] == "done" and "Controller mode SIM (read back)" in text(j) and "emulated" in text(j), j


def test_stop_ends_a_run_on_the_controller(server):
    call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})
    job = call(server, "POST", f"{U}/L-T01/run", {"confirmed": True})
    deadline = time.time() + 60                                  # until the program runs on the controller
    while "running at" not in text(call(server, "GET", f"/api/jobs/{job['id']}")):
        assert time.time() < deadline
        time.sleep(0.3)
    r = call(server, "POST", "/api/robots/stop-both", {}, tok=False)
    assert r["result"]["ok"] and "press Recover" in r["message"]
    j = wait(server, job)
    assert j["state"] == "cancelled" and "ended at" in text(j), j
    for side in ("left", "right"):
        assert call(server, "POST", f"/api/robots/{side}/recover", {})["estop"] == "CLEAR"


def test_named_poses_are_refused_and_every_command_is_logged(server):
    r = call(server, "POST", "/api/robots/left/move-pose", {"name": "Default Start Pose", "confirmed": True})
    assert "not stored on the controller" in r["error"]
    hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
    verbs = {r["verb"] for r in hw}
    assert {"stage", "rm_start_drag_teach", "rm_stop_drag_teach", "rm_save_trajectory", "rm_send_project",
            "program read back", "run mode REAL", "run mode SIM", "run program"} <= verbs
    assert all(r["ret"] == 0 for r in hw if r["verb"] in ("rm_send_project", "rm_save_trajectory"))
