"""Every 3D view's collision check, on demand (the owner, 16 Oct: "a button, to do the collision check, on
every trajectory, the raw, and the optimized"), with the robot's colliding links at every colliding moment
for the 3D to turn red; the commode state each region has attached, and a view's own choice of it."""
import json
import threading
import time
import urllib.request

import pytest

PID = "alix_views"
P = f"/api/projects/{PID}"


@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-views")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    call(ctx, "PUT", P, {"id": PID, "name": "Views", "archived": False, "state": {"currentTask": 0}})
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json", "X-Lite-Token": ctx.get("tok") or ""}
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=240) as resp:
        return json.loads(resp.read())


def wait(ctx, job, timeout=240):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < timeout, j
        time.sleep(0.2)


def test_every_region_has_a_commode_state_and_prepare_sets_it(lite):
    assert call(lite, "GET", f"{P}/units/L-T01")["phase"] == "lid_closed"          # OPEN LID + SEAT: met closed
    assert call(lite, "GET", f"{P}/units/L-T02")["phase"] == "seat_open"           # CLOSE SEAT ONLY
    assert call(lite, "GET", f"{P}/units/L-T03")["phase"] == "lid_open"            # OPEN SEAT
    u = call(lite, "GET", f"{P}/units/R-R04")
    assert u["phase"] == u["phase_attached"] == "lid_open"                          # the catalogue's own
    u = call(lite, "POST", f"{P}/units/R-R04/phase", {"phase": "seat_open"})
    assert u["phase"] == "seat_open" and u["phase_attached"] == "lid_open"
    assert "lid closed, lid open or seat open" in call(lite, "POST", f"{P}/units/R-R04/phase", {"phase": "open"})["error"]
    call(lite, "POST", f"{P}/units/R-R04/phase", {"phase": "lid_open"})


def test_the_raw_and_the_tuned_file_are_checked_with_their_links(lite):
    key = "L-T01"                                                                  # its existing recording, FILE_10.1.txt
    raw = wait(lite, call(lite, "POST", f"{P}/units/{key}/collisions", {"which": "raw"}))["result"]
    assert raw["which"] == "raw" and raw["phase"] == "lid_closed" and isinstance(raw["spans"], list)
    for a, b, links in raw["spans"]:
        assert 1 <= a <= b <= raw["lines"] and links and all(isinstance(x, str) for x in links)
    assert len(raw["spans"]) >= (0 if raw["ok"] else 1)
    seat = wait(lite, call(lite, "POST", f"{P}/units/{key}/collisions", {"which": "raw", "phase": "seat_open"}))["result"]
    assert seat["phase"] == "seat_open"                                            # the view's own choice of state
    assert "lid closed, lid open or seat open" in call(lite, "POST", f"{P}/units/{key}/collisions", {"phase": "up"})["error"]
    assert wait(lite, call(lite, "POST", f"{P}/units/{key}/optimise", {}))["state"] == "done"
    tuned = wait(lite, call(lite, "POST", f"{P}/units/{key}/collisions", {"which": "candidate"}))["result"]
    assert tuned["which"] == "candidate" and tuned["lines"] != raw["lines"]
    call(lite, "POST", f"{P}/units/{key}/keep", {})
    kept = wait(lite, call(lite, "POST", f"{P}/units/{key}/collisions", {"which": "version"}))["result"]
    assert kept["lines"] == tuned["lines"] and kept["ok"] == tuned["ok"]


def test_a_merge_and_the_viewers_two_files_are_checked(lite):
    item = next(i for i in call(lite, "POST", f"{P}/merge-sources/left", {})["items"] if i["kind"] == "rec" and not i["why"])
    m = call(lite, "POST", f"{P}/merges/left/from-project", {"name": "L_view_merge", "files": [item["ref"]]})
    r = wait(lite, call(lite, "POST", f"{P}/merges/left/{m['file']}/collisions", {}))["result"]
    assert r["which"] == "merge" and r["lines"] == m["lines"] and "spans" in r

    v6 = next(s for s in call(lite, "GET", "/api/library")["sets"] if s["id"] == "v6_legacy")["files"]
    left = next(f for f in v6 if f["arm"] == "left" and f["name"] == "L_P01P02.txt")
    right = next(f for f in v6 if f["arm"] == "right" and f["name"] == "R_P01P03.txt")
    pair = wait(lite, call(lite, "POST", "/api/library/collisions", {"left": left["id"], "right": right["id"], "pid": PID}))["result"]
    assert pair["files"] == {"left": "L_P01P02.txt", "right": "R_P01P03.txt"} and pair["start_s"] > 0
    assert pair["samples"] > 100 and pair["checked"]["two_arms"] and set(pair["checked"]["arms"]) == {"left", "right"}
    for t0, t1, links in pair["spans"]:
        assert 0 <= t0 <= t1 <= pair["total_s"] + 0.05 and links
    one = wait(lite, call(lite, "POST", "/api/library/collisions", {"right": right["id"]}))["result"]
    assert one["checked"]["arms"] == ["right"] and "cubicle" in one["checked"]["walls"]
    assert "Load a LEFT or a RIGHT file" in call(lite, "POST", "/api/library/collisions", {})["error"]
