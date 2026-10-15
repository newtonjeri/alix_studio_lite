"""Merge + Full Sequence's flow table (feedback v2, slide 8): each task's LEFT and RIGHT file, the
kept version each region runs, chosen from its versions, and the controller programs that hold it."""
import json
import threading
import time
import urllib.request

import pytest


@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-flow")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()   # shuts down at once
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    call(ctx, "PUT", "/api/projects/alix_fl", {"id": "alix_fl", "name": "Flow", "createdAt": "2026-10-15T10:00:00.000Z",
                                               "updatedAt": "2026-10-15T10:00:00.000Z", "archived": False,
                                               "state": {"currentTask": 0}})
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json", "X-Lite-Token": ctx.get("tok") or ""}
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=120) as resp:
        return json.loads(resp.read())


def unit(ctx, key, verb="", body=None):
    return call(ctx, "GET" if body is None else "POST", f"/api/projects/alix_fl/units/{key}" + (f"/{verb}" if verb else ""), body)


def wait(ctx, job, limit=180):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < limit, j
        time.sleep(0.2)


def task(flow, label):
    return next(t for t in flow["tasks"] if t["label"] == label)


def test_each_row_shows_its_files_and_a_version_is_chosen(lite):
    key = "L-T01"                                        # TASK 3, LEFT
    a = task(call(lite, "GET", "/api/projects/alix_fl/flow-files"), "TASK 3")["left"]
    assert a["key"] == key and a["versions"] == [] and a["recording"] == "FILE_10.1.txt"
    for label in ("first", "second"):
        if label == "second":
            unit(lite, key, "settings", {"changes": {"segments": [{"start": 1, "end": 500, "pct": 90}]}, "label": "Scale"})
        assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done"
        unit(lite, key, "keep", {})
    a = task(call(lite, "GET", "/api/projects/alix_fl/flow-files"), "TASK 3")["left"]
    assert [v["version"] for v in a["versions"]] == ["v001", "v002"] and a["version"] == "v002"      # a newly kept one runs
    assert all(v["uploadable"] and v["uploaded"] == [] for v in a["versions"])

    u = unit(lite, key, "version", {"version": "v001"})
    assert u["version"] == "v001" and "v001" in next(g for g in u["gates"] if g["id"] == "limits")["text"]
    assert "no version v009" in unit(lite, key, "version", {"version": "v009"})["error"]
    for arm in ("left", "right"):
        call(lite, "POST", f"/api/robots/{arm}/connect", {})
    call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})
    assert wait(lite, unit(lite, key, "upload", {"program_id": 12, "limits_seen": True}))["state"] == "done"   # the chosen one is sent
    a = task(call(lite, "GET", "/api/projects/alix_fl/flow-files"), "TASK 3")["left"]
    v1, v2 = a["versions"]
    assert a["version"] == "v001" and [u["program_id"] for u in v1["uploaded"]] == [12] and v2["uploaded"] == []
    plan = call(lite, "POST", "/api/projects/alix_fl/sequence", {"source": "version", "steps": [{"step": 5, "rule": "TOGETHER"}]})
    left = plan["rows"][0]["arms"]["left"]
    assert left["ready"] and "v001" in left["file"], left                          # the full sequence runs it too


def test_one_arm_single_files_and_programs(lite):
    """Feedback v2, slide 11: an arm's full sequence alone, single files of the project, and programs on
    the controller, each on one arm (the other arm stays stopped)."""
    P = "/api/projects/alix_fl"
    plan = call(lite, "POST", P + "/sequence", {"source": "version", "only": "left", "steps": [{"step": 5, "rule": "TOGETHER"}]})
    assert plan["only"] == "left" and list(plan["rows"][0]["arms"]) == ["left"]
    plan = call(lite, "POST", P + "/sequence", {"source": "version", "items": [{"key": "L-T01", "which": "version:v002"}]})
    a = plan["rows"][0]["arms"]["left"]
    assert plan["files"] and a["ready"] and a["file"] == "version v002" and a["which"] == "version:v002"
    assert call(lite, "POST", P + "/sequence", {"source": "version", "only": "left", "items": [{"key": "R-T01"}]})["code"] == "arm"
    tl = call(lite, "POST", P + "/units/L-T01/timeline", {"which": "version:v002"})
    assert tl["lines"] == a["lines"]
    assert call(lite, "POST", P + "/points-plan", {"side": "left", "points": ["STP_HOME"]})["code"] == "connect"   # stand-in

    assert "never runs on the real arms" in unit(lite, "L-T01", "file/upload", {"which": "raw", "program_id": 13})["error"]
    assert unit(lite, "L-T01", "file/upload", {"which": "version:v002", "program_id": 13})["code"] == "limits"
    assert wait(lite, unit(lite, "L-T01", "file/upload", {"which": "version:v002", "program_id": 13, "limits_seen": True}))["state"] == "done"
    assert "does not hold" in unit(lite, "L-T01", "file/run", {"which": "version:v001", "program_id": 13})["error"]
    run = unit(lite, "L-T01", "file/run", {"which": "version:v002", "program_id": 13})
    assert run["state"] == "running" and "alone" in call(lite, "POST", "/api/robots/right/pole", {"mm": 150})["error"]
    assert wait(lite, run)["state"] == "done"

    assert "empty" in call(lite, "POST", "/api/robots/left/programs/run", {"program_id": 77})["error"]
    j = wait(lite, call(lite, "POST", "/api/robots/left/programs/run", {"program_id": 12}))      # Real Test sent v001 there
    assert j["state"] == "done" and "program 12" in j["title"], j


def _smooth(start_deg, j1_deg, lines=401):
    import numpy as np

    from alix_studio_lite.motion import track

    s = (1 - np.cos(np.linspace(0, np.pi, lines))) / 2
    q = np.repeat(np.asarray(start_deg, dtype=float)[None, :], lines, axis=0)
    q[:, 0] += s * j1_deg
    return track.track_bytes(np.round(q * 1000).astype(np.int64)).decode()


def test_a_custom_merge_of_one_arm(lite):
    """Feedback v2, slide 10: files of one arm merged in order, joined where they do not meet, kept in
    the project (merged/<arm>/) under a name of its own; renamed, downloaded, played in 3D, sent to
    the controller and run on that arm alone."""
    P = "/api/projects/alix_fl"
    end = unit(lite, "L-T01", "end")["joints_deg"]
    a = _smooth(end, 4)
    away = list(end)
    away[1] += 6                                                     # the second part starts elsewhere: a join
    b = _smooth(away, -3)
    body = {"name": "L_lid_pass", "files": [{"name": "a.txt", "content": a}, {"name": "b.txt", "content": b}], "pole_mm": 200, "hand": "OPEN"}
    assert call(lite, "POST", P + "/merges/left", {**body, "pole_mm": None})["code"] == "stage"
    assert call(lite, "POST", P + "/merges/left", {**body, "files": [{"name": "R_x.txt", "content": a}]})["code"] == "arm"
    mm = call(lite, "POST", P + "/merges/left", body)
    assert mm["name"] == "L_lid_pass.txt" and [x["name"] for x in mm["parts"]] == ["a.txt", "b.txt"] and mm["limits_ok"], mm["limits_text"]
    assert len(mm["joins"]) == 1 and mm["joins"][0]["gap_deg"] > 3 and mm["lines"] > 802
    again = call(lite, "POST", P + "/merges/left", body)
    assert again["name"] == "L_lid_pass_2.txt"                                       # one name per file in a project
    assert call(lite, "POST", f"{P}/merges/left/{again['file']}/name", {"name": "FILE_10.1.txt"})["code"] == "name"
    assert call(lite, "POST", f"{P}/merges/left/{again['file']}/name", {"name": "L_lid_pass_B"})["name"] == "L_lid_pass_B.txt"
    listed = call(lite, "GET", P + "/merges")
    assert {x["name"] for x in listed["left"]} == {"L_lid_pass.txt", "L_lid_pass_B.txt"} and listed["right"] == []
    r = urllib.request.Request(f"{lite['base']}{P}/merges/left/{mm['file']}/download", headers={"X-Lite-Token": lite["tok"]})
    with urllib.request.urlopen(r, timeout=30) as resp:
        assert resp.read().count(b"\n") == mm["lines"] and "L_lid_pass.txt" in resp.headers.get("Content-Disposition", "")

    plan = call(lite, "POST", P + "/sequence", {"source": "version", "only": "left", "items": [{"merge": mm["file"], "side": "left"}]})
    act = plan["rows"][0]["arms"]["left"]
    assert act["ready"] and act["timeline"]["url"].endswith(f"/merges/left/{mm['file']}/timeline")
    assert call(lite, "POST", act["timeline"]["url"], act["timeline"]["body"])["lines"] == mm["lines"]

    for arm in ("left", "right"):
        call(lite, "POST", f"/api/robots/{arm}/connect", {})
    call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})
    assert "does not hold" in call(lite, "POST", f"{P}/merges/left/{mm['file']}/run", {"program_id": 30})["error"]
    assert wait(lite, call(lite, "POST", f"{P}/merges/left/{mm['file']}/upload", {"program_id": 30, "limits_seen": True}))["state"] == "done"
    run = call(lite, "POST", f"{P}/merges/left/{mm['file']}/run", {"program_id": 30})
    assert "alone" in call(lite, "POST", "/api/robots/right/pole", {"mm": 160})["error"]
    assert wait(lite, run)["state"] == "done"
