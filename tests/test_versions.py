"""A region's kept versions: a newly kept one is the one it runs, another can be chosen and is the
one sent; and the execution in 3D from a start state (where an earlier execution left the arm), at
the speed its plan was made at."""
import base64
import json
import threading
import time
import urllib.request

import numpy as np
import pytest

PID = "alix_ver"
P = f"/api/projects/{PID}"


@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-ver")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    call(ctx, "PUT", P, {"id": PID, "name": "Versions", "archived": False, "state": {"currentTask": 0}})
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json", "X-Lite-Token": ctx.get("tok") or ""}
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=120) as resp:
        return json.loads(resp.read())


def unit(ctx, key, verb="", body=None):
    return call(ctx, "GET" if body is None else "POST", f"{P}/units/{key}" + (f"/{verb}" if verb else ""), body)


def wait(ctx, job, limit=180):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < limit, j
        time.sleep(0.2)


def test_a_kept_version_is_chosen_and_sent(lite):
    key = "L-T01"                                        # TASK 3, LEFT: the existing recording FILE_10.1.txt
    u = unit(lite, key)
    assert u["versions"] == [] and next(r for r in u["recordings"] if r["file"] == u["active"])["name"] == "FILE_10.1.txt"
    for label in ("first", "second"):
        if label == "second":
            unit(lite, key, "settings", {"changes": {"segments": [{"start": 1, "end": 500, "pct": 90}]}, "label": "Scale"})
        assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done"
        unit(lite, key, "keep", {})
    u = unit(lite, key)
    assert [v["version"] for v in u["versions"]] == ["v001", "v002"] and u["version"] == "v002"   # a newly kept one runs
    u = unit(lite, key, "version", {"version": "v001"})
    assert u["version"] == "v001" and "v001" in next(g for g in u["gates"] if g["id"] == "limits")["text"]
    assert "no version v009" in unit(lite, key, "version", {"version": "v009"})["error"]
    for arm in ("left", "right"):
        call(lite, "POST", f"/api/robots/{arm}/connect", {})
    call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})
    assert wait(lite, unit(lite, key, "upload", {"program_id": 12, "limits_seen": True}))["state"] == "done"
    v1 = next(v for v in unit(lite, key)["versions"] if v["version"] == "v001")
    assert lite["app"].robot.arm("left").programs[12]["sha256"] == v1["sha256"]                 # the chosen one is sent
    call(lite, "POST", "/api/robots/mode", {"mode": "visual"})


def test_the_timeline_from_a_start_state_and_at_its_plans_speed(lite):
    from alix_studio_lite.server import staging

    end = unit(lite, "L-T01", "end")                     # the kept version chosen above: its last line
    assert end["which"] == "version" and end["name"].endswith("v001") and len(end["joints_deg"]) == 7
    first = unit(lite, "L-T01", "timeline", {"which": "raw"})
    start = {"pole_mm": first["stage"]["pole_mm"], "hand": first["end"]["hand"], "q_deg": first["end"]["q_deg"]}
    tl = unit(lite, "L-T01", "timeline", {"which": "raw", "start": start})
    assert tl["phases"][0]["text"].startswith("Pole already at") and tl["phases"][1]["text"] == "Hand already in pose"
    assert tl["end"] == first["end"] and tl["lines"] == first["lines"]
    f = np.frombuffer(base64.b64decode(tl["frames"]), dtype="<f4").reshape(len(tl["times"]), len(tl["links"]), 16)
    assert f.shape[0] == len(tl["times"])
    assert unit(lite, "L-T01", "timeline", {"which": "raw", "start": {"pole_mm": 900}})["error"]
    was = staging.move_pct()
    staging.set_move_pct(100)
    try:
        own = unit(lite, "L-T01", "timeline", {"which": "raw", "move_pct": was})
        now = unit(lite, "L-T01", "timeline", {"which": "raw"})
    finally:
        staging.set_move_pct(was)
    assert own["move_pct"] == was and now["move_pct"] == 100 and now["traj_t0"] < own["traj_t0"]
