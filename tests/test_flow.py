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
    threading.Thread(target=srv.serve_forever, daemon=True).start()
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
    assert wait(lite, unit(lite, key, "upload", {"program_id": 12}))["state"] == "done"            # the chosen one is sent
    a = task(call(lite, "GET", "/api/projects/alix_fl/flow-files"), "TASK 3")["left"]
    v1, v2 = a["versions"]
    assert a["version"] == "v001" and [u["program_id"] for u in v1["uploaded"]] == [12] and v2["uploaded"] == []
    plan = call(lite, "POST", "/api/projects/alix_fl/sequence", {"source": "version", "steps": [{"step": 5, "rule": "TOGETHER"}]})
    left = plan["rows"][0]["arms"]["left"]
    assert left["ready"] and "v001" in left["file"], left                          # the full sequence runs it too
