"""Save & Review's tools (feedback v5): Trim and save (slide 1; Save overwrites the recording, the
owner's choice, Save As keeps a new one), and the collision check of a recording (slide 5): every
5 ms line against the walls measured in Prepare (or ALIX's cubicle), the commode in the region's
state and the other arm, with ALIX's collision code, the first colliding line reported."""
import json
import threading
import time
import urllib.request

import pytest

PID = "alix_review"


@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-review")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app, "data": data}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    call(ctx, "PUT", f"/api/projects/{PID}", {"id": PID, "name": "Review", "archived": False, "state": {"currentTask": 0}})
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json", "X-Lite-Token": ctx.get("tok") or ""}
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=180) as resp:
        return json.loads(resp.read())


def unit(ctx, key, verb="", body=None):
    path = f"/api/projects/{PID}/units/{key}" + (f"/{verb}" if verb else "")
    return call(ctx, "POST" if verb else "GET", path, body if verb else None)


def wait(ctx, job, timeout=180):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < timeout, j
        time.sleep(0.2)


def test_trim_saves_as_a_new_recording_or_overwrites(lite):
    key = "L-T01"
    u = unit(lite, key)
    rec = next(r for r in u["recordings"] if r["file"] == u["active"])
    n = rec["lines"]
    assert "two lines at least" in unit(lite, key, f"recordings/{rec['file']}/trim", {"start": 10, "end": 10})["error"]
    unit(lite, key, "settings", {"changes": {"segments": [{"start": 1, "end": 500, "pct": 80}]}})

    out = unit(lite, key, f"recordings/{rec['file']}/trim", {"start": 101, "end": n - 100, "name": "L_base_trimmed"})["unit"]
    new = next(r for r in out["recordings"] if r["file"] == out["active"])
    assert new["name"] == "L_base_trimmed.txt" and new["lines"] == n - 200 and new["source"] == "trim"
    assert new["taught_arm"] == rec["taught_arm"] and new["stage"]["text"] == rec["stage"]["text"]
    assert any(r["file"] == rec["file"] and r["lines"] == n for r in out["recordings"])      # Save As keeps the original
    assert out["settings_view"]["settings"]["segments"] == []

    out = unit(lite, key, f"recordings/{new['file']}/trim", {"start": 1, "end": 1000})["unit"]   # Save: overwritten
    same = next(r for r in out["recordings"] if r["file"] == new["file"])
    assert same["lines"] == 1000 and same["name"] == "L_base_trimmed.txt" and same["sha256"] != new["sha256"]
    assert "trimmed to lines 1–1,000" in same["note"]
    path = next((lite["data"] / "projects" / PID).rglob(new["file"]))
    assert len(path.read_bytes().splitlines()) == 1000 and not path.stat().st_mode & 0o222   # still read-only


def test_collision_check_names_the_first_line(lite):
    """The check's facts: every line, the arm with its hand, the commode as contact for the hand and a
    collision for the arm's other links, and the walls: ALIX's cubicle without a room, then Prepare's."""
    key = "L-T01"
    j = wait(lite, unit(lite, key, "collisions", {}))
    assert j["state"] == "done", j
    r = j["result"]
    assert r["lines"] == 1000 and "cubicle" in r["checked"]["walls"] and r["checked"]["links"] > 30
    assert r["checked"]["other_arm"] == "the RIGHT arm, standing" and r["checked"]["contact_depth_mm"] == 20.0
    assert r["ok"] == (r["first"] is None) and r["seconds"] < 60
    first = wait(lite, unit(lite, key, "collisions", {}))["result"]                    # the same: kept
    assert first == r

    call(lite, "POST", f"/api/projects/{PID}/placement/room",
         {"width_mm": 800, "length_mm": 2400, "left_mm": 80, "right_mm": 80, "front_mm": 1300})
    narrow = wait(lite, unit(lite, key, "collisions", {}))["result"]
    assert narrow["checked"]["walls"] == "the room measured in Prepare"
    if narrow["first"]:
        f = narrow["first"]
        assert 1 <= f["first_line"] <= f["last_line"] <= narrow["lines"] and f["pairs"] and f["what"]
