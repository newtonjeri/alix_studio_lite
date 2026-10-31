"""S2 with the UGV 12 mm further from the commode than ALIX's plans were made at (its Programs taught at a standoff of
262 mm), on ALIX's emulated arms: Open the lid's and Open the seat's plans moved with the commode are made in a process
of their own from the run's start, one after the other: the first while the first Program runs, the second while Open
the lid runs (the owner, 1 Nov: "generate the first task, as it executes be generating the other tasks"; "proceed to
second task planning as the first executes"). The camera's answers are scripted (LITE_CHECK_SCRIPT)."""
import json
import os
import re
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

from alix_studio_lite import vendor_paths
from test_s2_lid_emulated import call, program, wait

PID = "alix_s2_lid"                       # test_s2_lid_emulated's helpers address this project
P = f"/api/projects/{PID}"
ROOM = {"width_mm": 800, "length_mm": 1600, "left_mm": 220, "right_mm": 220, "front_mm": 1062, "back_mm": 0}   # the lab's


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    data = tmp_path_factory.mktemp("lite-s2-place-ahead")
    script = data / "answers.json"
    script.write_text(json.dumps([]))                                  # the run writes the answers it needs
    proc = subprocess.Popen([sys.executable, str(Path(__file__).with_name("emulated_lite.py")), "--robot", "emulated",
                             "--no-browser", "--port", str(port), "--data", str(data), "--logs", str(data / "logs"),
                             "--projects", str(data / "projects")],
                            cwd=str(vendor_paths.LITE_ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            env=dict(os.environ, LITE_CHECK_SCRIPT=str(script)))
    ctx = {"base": f"http://127.0.0.1:{port}", "data": data, "script": script}
    for _ in range(150):
        try:
            ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
            break
        except OSError:
            time.sleep(0.2)
    call(ctx, "PUT", P, {"id": PID, "name": "S2 place ahead", "archived": False, "state": {"currentTask": 0}})
    call(ctx, "POST", f"{P}/placement/room", ROOM)
    call(ctx, "POST", f"{P}/placement/task", {"task": "TASK 1", "standoff_mm": 262})
    for side in ("left", "right"):
        assert call(ctx, "POST", f"/api/robots/{side}/connect", {})["connected"]
    assert call(ctx, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})["mode"] == "real"
    for side, key, pid, j1, pole, hand in (("left", "L-R01", 71, 8.0, 150, "GRIP"), ("right", "R-R02", 71, -8.0, 150, "GRIP"),
                                           ("left", "L-R03", 72, 6.0, 200, "CLEANING"), ("right", "R-R04", 72, -6.0, 200, "CLEANING")):
        program(ctx, side, key, pid, j1, pole, hand, task="TASK 1")
    yield ctx
    proc.terminate()
    proc.wait(10)


def test_the_second_tasks_plan_is_made_while_the_first_runs(server):
    prog = lambda pid: {"type": "program", "id": pid, "transit_speed": 30}               # noqa: E731
    tasks = [{"type": "mechanism", "task": "lid_open"}, {"type": "mechanism", "task": "seat_open"}]
    for side in ("left", "right"):
        call(server, "PUT", f"{P}/workflow/{side}", {"steps": [prog(71), *tasks, prog(72)]})
    server["script"].write_text(json.dumps(["lid_lifted", "lid_open", "seat_lifted", "seat_open"]))   # a sequence's checks
    j = wait(server, call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "unsimulated_ok": True}))
    said = " | ".join(e["text"] for e in j.get("events") or [])
    assert j["state"] == "done", (j.get("error"), said[-1500:])
    assert "2 plan(s) being made from now on, in the run's order" in said, said[-3000:]
    for title in ("Open the lid", "Open the seat"):
        assert f"{title}: each arm's working stretch moved with the commode (x +12.0" in said, title
        assert f"{title}: {title} done" in said
    assert "made now in" not in said
    # the second made while the first ran: ready as it started, nothing waited for
    assert re.search(r"Open the seat: each arm's working stretch moved with the commode \(x \+12\.0[^)]*\), made while the "
                     r"run went on: the hands", said), said[-3000:]
