"""S2 through a lid task on ALIX's emulated arms (the owner, 24 Oct): each Program runs with its own pole and hand, set
by Lite before it with no row of the sequence's (the pole and the hand together, the Program straight after); after
the lid task (and its follow-up, which looks first and ends there) each arm goes to its next Program's start, its own
pole and hand and then its first point, both arms at once, so that the next stage starts both Programs where they
begin; each arm's part of a stage has one recording of its controller's feed. The camera's answers are scripted
(LITE_CHECK_SCRIPT, a test harness only)."""
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

PID = "alix_s2_lid"
P = f"/api/projects/{PID}"
GRIP = [0, 0, 0, 0, 150, 1000]
CLEAN = [1000, 1000, 1000, 1000, 0, 1000]


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    data = tmp_path_factory.mktemp("lite-s2-lid")
    script = data / "answers.json"
    # Open the lid: closed, lifted (the seat under it not asked, 26 Oct), its end check: the lid open
    script.write_text(json.dumps(["lid_lifted", "lid_open"]))          # a sequence: no "lid closed" asked (27 Oct)
    proc = subprocess.Popen([sys.executable, str(Path(__file__).with_name("emulated_lite.py")), "--robot", "emulated",
                             "--no-browser", "--port", str(port), "--data", str(data), "--logs", str(data / "logs"),
                             "--projects", str(data / "projects")],
                            cwd=str(vendor_paths.LITE_ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            env=dict(os.environ, LITE_CHECK_SCRIPT=str(script)))
    ctx = {"base": f"http://127.0.0.1:{port}", "data": data}
    for _ in range(150):
        try:
            ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
            break
        except OSError:
            time.sleep(0.2)
    call(ctx, "PUT", P, {"id": PID, "name": "S2 lid", "archived": False, "state": {"currentTask": 0}})
    for side in ("left", "right"):
        assert call(ctx, "POST", f"/api/robots/{side}/connect", {})["connected"]
    assert call(ctx, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})["mode"] == "real"
    for side, key, pid, j1, pole, hand in (("left", "L-R01", 71, 8.0, 150, "GRIP"), ("right", "R-R02", 71, -8.0, 150, "GRIP"),
                                           ("left", "L-R03", 72, 6.0, 200, "CLEANING"), ("right", "R-R04", 72, -6.0, 200, "CLEANING")):
        program(ctx, side, key, pid, j1, pole, hand)
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


def wait(ctx, job, limit=400):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < limit, j
        time.sleep(0.2)


def program(ctx, side, key, pid, j1, pole, hand):
    """A short file of the arm, from where it stands, as program ``pid`` on its controller with its own pole and hand
    (S1: merged, then uploaded)."""
    from alix_studio_lite.motion import track

    fb = call(ctx, "GET", "/api/robots/status")["arms"][side]["link"]["feedback"]
    t = np.linspace(0.0, 1.0, 240)[:, None]
    q0 = np.asarray(fb["joints_deg"][:7], dtype=float)
    md = np.round((q0 + np.array([[j1, 0, 0, 0, 0, 0, 0]]) * np.sin(np.pi * t)) * 1000).astype(np.int64)
    ctx.setdefault("first", {})[(side, pid)] = (md[0] / 1000.0).tolist()
    name = f"{side[0].upper()}_{pid}.txt"
    out = call(ctx, "POST", f"{P}/units/{key}/recordings", {"source": "upload", "name": name, "content": track.track_bytes(md).decode(),
                                                            "pole_mm": pole, "hand": hand})
    assert "error" not in out, out
    item = next(i for i in call(ctx, "POST", f"{P}/merge-sources/{side}", {})["items"] if i["name"] == name)
    m = call(ctx, "POST", f"{P}/merges/{side}/from-project", {"name": name.replace(".txt", "_m"), "files": [item["ref"]]})
    j = wait(ctx, call(ctx, "POST", f"{P}/merges/{side}/{m['file']}/upload", {"program_id": pid, "confirmed": True, "limits_seen": True}))
    assert j["state"] == "done", j


def hardware(ctx, since: float) -> list:
    path = next((Path(ctx["data"]) / "logs").iterdir()) / "hardware.jsonl"
    return [r for r in (json.loads(x) for x in path.read_text().splitlines() if x.strip()) if r["t"] >= since]


def test_a_lid_task_then_each_arm_to_its_next_programs_start_with_its_own_pole_and_hand(server):
    prog = lambda pid: {"type": "program", "id": pid, "transit_speed": 30}               # noqa: E731
    lid = {"type": "mechanism", "task": "lid_open"}
    for side in ("left", "right"):                                    # no Pole or Hand row: each Program's own
        call(server, "PUT", f"{P}/workflow/{side}", {"steps": [prog(71), lid, prog(72)]})
    sim = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    assert sim["state"] == "done", sim
    res = sim["result"]
    assert call(server, "POST", f"{P}/workflow/played", {"hash": res["hash"]})["completed"]
    texts = " | ".join(f["text"] for f in res["findings"])
    assert "Close the seat" not in texts and "move to its next task's start" in texts    # no closing after Open the lid (26 Oct)
    assert not [f for f in res["findings"] if f["level"] == "stop"]
    lid_row = next(r for r in res["stages"] if r["kind"] == "mechanism")
    assert lid_row["title"] == "Open the lid, then LEFT to Program 72's start and RIGHT to Program 72's start"
    assert lid_row["t0"] < lid_row["t1_plan"] < lid_row["t1"]
    for side in ("left", "right"):
        acts = res["acts"][side]
        first = next(a for a in acts if a["kind"] == "pole_hand")       # Program 71's own, before it, nothing between
        assert first["pole1"] == 150.0 and first["hand1"] == GRIP and acts[acts.index(first) + 1]["kind"] == "program"
        then = [a for a in acts if a["stage"] == lid_row["index"] and a["t0"] >= lid_row["t1_plan"] - 1e-6 and a["t1"] > a["t0"]]
        assert [a["kind"] for a in then][::2] == ["pole_hand", "global"] and then[0]["pole1"] == 200.0 and then[0]["hand1"] == CLEAN
        nxt = next(a for a in acts if a["kind"] == "program" and a["program"] == 72)
        assert nxt["transit"] == [0.0, 0.0, 0.0]                       # at its first point: no move of the controller's

    t0 = time.time()
    j = wait(server, call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": res["hash"],
                                                                "record_feed": True}))                 # the feed: only when asked (26 Oct)
    said = " | ".join(e["text"] for e in j.get("events") or [])
    assert j["state"] == "done", (j.get("error"), said[-1500:])
    assert "Open the lid done" in said and "Close the seat" not in said   # its sequence plan (27 Oct); no closing follows (26 Oct)
    assert "RIGHT hand hand_release: not sent, the RIGHT hand is set for its next task's start right after" in said   # 26 Oct
    for side in ("LEFT", "RIGHT"):
        assert f"{side}: at the start of Program 72 (its own pole and hand, its first point)." in said
    hw = hardware(server, t0)
    pairs = [(r["side"], r["args"]["mm"], r["args"]["hand"]) for r in hw if r["verb"] == "pole and hand"]
    # 71's own; Open the lid's start (its pole and the hand its first move needs, one step since 26 Oct); 72's own after it
    from alix_studio_lite.server import mechanism                     # in a sequence (no start check): the left's pole and half
    lid = mechanism.Plan("lid_open")                                  # grasp; the right's grasp after its look (29 Oct)
    first = lid.first_move_targets(mechanism.without_start_checks(list(lid.entries))[0])
    start = {s: (first[s]["pole_mm"], first[s]["hand"]) for s in ("left", "right")}
    assert start["left"][1] == [516, 516, 516, 516, 133, 944] and start["right"][1] == [1000] * 6
    for side in ("left", "right"):
        assert [(mm, hand) for s, mm, hand in pairs if s == side] == [(150.0, GRIP), start[side], (200.0, CLEAN)]
    for side in ("left", "right"):                                    # its last MoveJ: to 72's first point
        last = [r for r in hw if r["verb"] == "move to pose" and r["side"] == side][-1]
        assert last["ok"] and last["args"]["deg"] == pytest.approx(server["first"][(side, 72)], abs=0.01)
    starts = [r for r in hw if r["verb"] == "runner v14" and str(r.get("text") or "").startswith("WORKFLOW PROGRAM 1/1: slot 72")]
    assert len(starts) == 2
    # the two arms at once where the plan allows it (27 Oct night): the right's look, pole and grasp with the left's set-up,
    # the left's approach and lift once the right's fingers are closed (29 Oct); the left's retreat with the right's grasp and look
    spans = lambda label: [r["args"]["t_mono"] for r in hw if r["verb"] == "canfd stream" and r["args"]["label"] == label]   # noqa: E731
    looks, lift = spans("RIGHT arm move_to_inspect"), spans("LEFT arm move_to_pre_start + move_to_start + execute_path")[0]
    assert looks[0][1] <= lift[0], (looks, lift)                                        # the first look, then the lift
    retreat = spans("LEFT arm move_to_pre_start + move_to_rest")[0]
    assert retreat[0] < looks[-1][1] and looks[-1][0] < retreat[1], (retreat, looks)                   # the last look, in the retreat's
    assert "both arms at once: LEFT move_to_half_pole_length + hand_grasp || RIGHT move_to_inspect" in said
    feeds = [r for r in hw if r["verb"] == "s2 feed" and r.get("text") == "recording"]
    assert len(feeds) == 4                                            # one an arm a stage (2 stages), not one a runner call
