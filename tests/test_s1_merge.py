"""Merge + Full Sequence, S1: Merge & Upload (feedback v5, slide 9; the owner, 16 Oct). The project's
own files of one arm merge by reference, in order, only when they share the pole, the hand, the UGV
placement and the room (each measurement within ±10 mm); the merge keeps them and plays there. "Send
to Tune + Simulation" opens a merge in Tune as a unit of its own, and what Tune keeps comes back to S1
as a new merge."""
import json
import threading
import time
import urllib.request

import numpy as np
import pytest

PID = "alix_s1"
P = f"/api/projects/{PID}"
ROOM = {"width_mm": 1400, "length_mm": 1940, "left_mm": 516, "right_mm": 516, "front_mm": 1400}


@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-s1")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app, "data": data}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    call(ctx, "PUT", P, {"id": PID, "name": "S1", "archived": False, "state": {"currentTask": 0}})
    call(ctx, "POST", P + "/placement/room", ROOM)
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json", "X-Lite-Token": ctx.get("tok") or ""}
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=180) as resp:
        return json.loads(resp.read())


def wait(ctx, job, timeout=180):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < timeout, j
        time.sleep(0.2)


def upload(ctx, key, name, task, pole=150, hand="GRIP", start=0.0):
    from alix_studio_lite.motion import track

    t = np.linspace(0.0, 1.0, 400)[:, None]
    md = np.round((np.array([[0, -60, 0, 90, 0, 30, 0]]) + start + 8.0 * np.sin(2 * np.pi * t)) * 1000).astype(np.int64)
    body = {"source": "upload", "name": name, "content": track.track_bytes(md).decode(), "pole_mm": pole, "hand": hand,
            "task": task}
    out = call(ctx, "POST", f"{P}/units/{key}/recordings", body)
    assert "error" not in out, out
    return next(r for r in out["unit"]["recordings"] if r["file"] == out["saved"])


def sources(ctx, side, picked=()):
    return {it["name"]: it for it in call(ctx, "POST", f"{P}/merge-sources/{side}", {"picked": list(picked)})["items"]}


def test_s1_merges_only_files_of_one_stage_and_placement(lite):
    call(lite, "POST", P + "/placement/task", {"task": "TASK 1A", "standoff_mm": 380})
    call(lite, "POST", P + "/placement/task", {"task": "TASK 2", "standoff_mm": 386, "front_mm": 4})   # within 10 mm
    call(lite, "POST", P + "/placement/task", {"task": "TASK 3", "standoff_mm": 391})                # 11 mm off
    upload(lite, "L-R01", "L_s1_a.txt", "TASK 1A")
    upload(lite, "L-R02", "L_s1_b.txt", "TASK 2", start=3.0)
    upload(lite, "L-R03", "L_s1_far.txt", "TASK 3")
    upload(lite, "L-R04", "L_s1_pole.txt", "TASK 1A", pole=50)

    items = sources(lite, "left")
    a = items["L_s1_a.txt"]
    assert a["kind"] == "rec" and a["stage"]["pole_mm"] == 150 and a["placement"]["standoff_mm"] == 380
    assert a["placement"]["room"]["width_mm"] == 1400 and "standoff 380 mm" in a["placement_text"] and not a["why"]
    assert not any(it["taught_arm"] == "right" for it in items.values())          # the LEFT arm's files only
    picked = sources(lite, "left", [a["ref"]])
    assert picked["L_s1_b.txt"]["compatible"]                                     # 6 mm and 4 mm: within 10 mm
    assert picked["L_s1_far.txt"]["differs"] == ["UGV standoff 380 mm vs 391 mm"]
    assert picked["L_s1_pole.txt"]["differs"] == ["pole 150 mm vs 50 mm"]

    bad = call(lite, "POST", f"{P}/merges/left/from-project", {"name": "L_bad", "files": [a["ref"], picked["L_s1_far.txt"]["ref"]]})
    assert bad["code"] == "mismatch" and "UGV standoff 380 mm vs 391 mm" in bad["error"]
    assert "same pole, hand, UGV placement and room" in bad["hint"]

    m = call(lite, "POST", f"{P}/merges/left/from-project", {"name": "L_s1_merge", "files": [a["ref"], picked["L_s1_b.txt"]["ref"]]})
    assert m["name"] == "L_s1_merge.txt" and [p["name"] for p in m["parts"]] == ["L_s1_a.txt", "L_s1_b.txt"]
    assert len(m["joins"]) == 1 and m["lines"] > 800                              # 3 degrees apart: joined
    assert m["placement"]["standoff_mm"] == 380 and m["stage"]["pole_mm"] == 150 and m["phase"]
    assert [s["name"] for s in m["sources"]] == ["L_s1_a.txt", "L_s1_b.txt"] and all(s["sha256"] for s in m["sources"])
    tl = call(lite, "POST", f"{P}/merges/left/{m['file']}/timeline", {})
    assert tl["scene"]["from"] == "the merge" and tl["scene"]["standoff_mm"] == 380

    room = dict(ROOM, width_mm=1420)                                              # the room measured again, 20 mm wider
    call(lite, "POST", P + "/placement/room", room)
    call(lite, "POST", P + "/placement/task", {"task": "TASK 4", "standoff_mm": 380})
    upload(lite, "L-R06", "L_s1_room.txt", "TASK 4")
    picked = sources(lite, "left", [a["ref"]])
    assert picked["L_s1_room.txt"]["differs"] == ["room width 1,400 mm vs 1,420 mm"]
    assert sources(lite, "left")["L_s1_merge.txt"]["kind"] == "merge"             # a merge merges again
    call(lite, "POST", P + "/placement/room", ROOM)


def test_send_to_tune_and_back_to_s1(lite):
    m = next(x for x in call(lite, "GET", f"{P}/merges")["left"] if x["name"] == "L_s1_merge.txt")
    u = call(lite, "POST", f"{P}/merges/left/{m['file']}/tune", {})
    key = u["key"]
    assert key.startswith("MERGE-L-") and u["arm"] == "left"
    rec = next(r for r in u["recordings"] if r["file"] == u["active"])
    assert rec["name"] == "L_s1_merge.txt" and rec["lines"] == m["lines"] and rec["placement"]["standoff_mm"] == 380
    assert rec["taught_arm"] == "left" and rec["stage"]["pole_mm"] == 150
    assert call(lite, "POST", f"{P}/merges/left/{m['file']}/tune", {})["key"] == key            # the same unit again
    tasks = call(lite, "GET", f"{P}/tasks")
    assert key not in json.dumps(tasks)                                                       # never a task of the queue

    job = call(lite, "POST", f"{P}/units/{key}/optimise", {})
    assert wait(lite, job)["state"] == "done"
    kept = call(lite, "POST", f"{P}/units/{key}/keep", {"name": "L_s1_merge_tuned"})
    back = kept["merge"]
    assert back["name"] == "L_s1_merge_tuned.txt" and back["made_from"]["merge"] == "L_s1_merge.txt"
    assert back["placement"]["standoff_mm"] == 380 and back["stage"]["pole_mm"] == 150
    again = call(lite, "POST", f"{P}/units/{key}/keep", {})                                   # kept once: the same merge
    assert again["merge"]["file"] == back["file"]
    names = [x["name"] for x in call(lite, "GET", f"{P}/merges")["left"]]
    assert names.count("L_s1_merge_tuned.txt") == 1 and "L_s1_merge.txt" in names

    from alix_studio_lite.server.lite_units import LiteUnits                                  # it survives a restart
    units = LiteUnits(lite["data"] / "projects" / PID, "commode_c")
    assert units.is_merge_unit(key) and units.units[key]["merge"]["name"] == "L_s1_merge.txt"
    assert key not in list(units.units) and units.name_owner("L_s1_merge_tuned.txt") == "merge:left"


def test_a_merge_is_renamed_downloaded_and_sent(lite):
    m = next(x for x in call(lite, "GET", f"{P}/merges")["left"] if x["name"] == "L_s1_merge.txt")
    taken = call(lite, "POST", f"{P}/merges/left/{m['file']}/name", {"name": "L_s1_a.txt"})          # a recording's name
    assert taken["code"] == "name"
    assert call(lite, "POST", f"{P}/merges/left/{m['file']}/name", {"name": "L_s1_lid"})["name"] == "L_s1_lid.txt"
    r = urllib.request.Request(f"{lite['base']}{P}/merges/left/{m['file']}/download", headers={"X-Lite-Token": lite["tok"]})
    with urllib.request.urlopen(r, timeout=30) as resp:
        assert resp.read().count(b"\n") == m["lines"] and "L_s1_lid.txt" in resp.headers.get("Content-Disposition", "")
    for arm in ("left", "right"):
        call(lite, "POST", f"/api/robots/{arm}/connect", {})
    call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})
    job = call(lite, "POST", f"{P}/merges/left/{m['file']}/upload", {"program_id": 30, "limits_seen": True})
    assert wait(lite, job)["state"] == "done"
    held = lite["app"].robot.arm("left").programs[30]
    assert held["sha256"] == m["sha256"] and held["stage"]["pole_mm"] == 150                        # sent with its stage
    call(lite, "POST", "/api/robots/mode", {"mode": "visual"})
