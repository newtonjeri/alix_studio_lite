"""The web API the UI is built on: one region through every step, with the robot stand-in.

Starts Lite's server on a free port with a temporary data folder, so it never touches
~/ALIX-Lite and needs no robot.
"""
import json
import threading
import time
import urllib.request

import pytest

from alix_studio_lite import vendor_paths


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    from alix_studio_lite.server import workers
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-data")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    port = srv.server_address[1]
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.1}, daemon=True).start()
    yield f"http://127.0.0.1:{port}/api/lite/"
    srv.shutdown()
    srv.server_close()
    workers.shutdown()


def call(base, path, body=None):
    if body is None:
        return json.load(urllib.request.urlopen(base + path, timeout=60))
    req = urllib.request.Request(base + path, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    return json.load(urllib.request.urlopen(req, timeout=60))


def wait(base, job, timeout=120):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = call(base, f"jobs/{job['id']}")
        if v["state"] != "running":
            return v
        time.sleep(0.1)
    raise TimeoutError(job)


def test_starts_safe(server):
    s = call(server, "state")
    assert s["mode"] == "visual" and s["role"] == "operator"            # NFR-UX (f): starts without hardware
    assert not any(a["connected"] for a in s["arms"].values())
    cat = call(server, "catalogue")
    assert cat["first_release"] == 34 and len(cat["units"]) == 42
    assert cat["counts"]["taught"] == 8                                 # the library's existing recordings (D31)


def test_refusals_are_sentences(server):
    r = call(server, "unit/L-R01/optimise", {})
    assert r["error"] and r["hint"]                                    # no recording yet
    r = call(server, "mode", {"mode": "real"})
    assert "Connect" in r["error"]


@pytest.mark.slow
def test_region_through_every_step(server):
    key = "L-T01"
    h = call(server, f"unit/{key}/settings", {"changes": {"target_s": 3.0, "optimiser": "default"}, "label": "Target 3 s"})
    assert h["can_undo"] and h["settings"]["target_s"] == 3.0
    h = call(server, f"unit/{key}/undo", {})
    assert h["settings"]["target_s"] is None
    h = call(server, f"unit/{key}/redo", {})
    assert h["settings"]["target_s"] == 3.0

    v = wait(server, call(server, f"unit/{key}/optimise", {}))
    assert v["state"] == "done", v
    res = v["result"]["summary"]
    assert res["met"] is False and res["uploadable"] is True           # 3 s is too fast: closest safe time
    assert res["binding"] and res["achieved_s"] == res["t_min_s"]
    u = call(server, f"unit/{key}")
    assert u["candidate_util"]["speed"]
    assert call(server, f"unit/{key}/save-version", {})["version"] == "v001"
    assert wait(server, call(server, f"unit/{key}/check3d", {}))["state"] == "done"
    assert call(server, f"unit/{key}")["status"] == "check_ok"

    # robot steps need a robot mode, a connection and a program number
    assert "Visual sim" in call(server, f"unit/{key}/upload", {})["error"]
    for arm in ("left", "right"):
        assert call(server, "robot/connect", {"arm": arm})["estop"] == "CLEAR"
    assert call(server, "mode", {"mode": "hwsim"})["mode"] == "hwsim"
    assert "program number" in call(server, f"unit/{key}/upload", {})["error"]
    call(server, f"unit/{key}/settings", {"changes": {"program_id": 7}, "label": "Program 7"})
    assert wait(server, call(server, f"unit/{key}/upload", {}))["state"] == "done"
    time.sleep(0.5)
    assert call(server, f"unit/{key}")["status"] == "uploaded"

    # an E-stop on either arm blocks motion on both; STOP is never refused
    call(server, "role", {"role": "engineer", "pin": "0000"})
    call(server, "robot/estop-test", {"arm": "right", "pressed": True})
    r = call(server, f"unit/{key}/run", {})
    assert "E-stop" in r["error"]
    assert "stopped" in call(server, "robot/stop", {"arm": "all"})
    call(server, "robot/estop-test", {"arm": "right", "pressed": False})
    assert call(server, "robot/recover", {"arm": "right"})["estop"] == "CLEAR"

    # dry run completes and moves the status on; a real run needs confirmation
    job = call(server, f"unit/{key}/run", {})
    assert wait(server, job)["state"] == "done"
    assert call(server, f"unit/{key}")["status"] == "dryrun_ok"
    assert "confirmation" in call(server, "mode", {"mode": "real"})["error"]
    assert call(server, "mode", {"mode": "real", "confirmed": True})["mode"] == "real"
    assert "confirmation" in call(server, f"unit/{key}/run", {})["error"]


def test_static_files(server):
    root = server.replace("/api/lite/", "/")
    for path in ("", "js/app.js", "css/tokens.css", "vendor/plotly.min.js", "deck/s07-T1C-L.jpg",
                 "mesh/commode_c/body.stl", "mesh/cubicle/1400.stl"):
        with urllib.request.urlopen(root + path, timeout=30) as r:
            assert r.status == 200, path
    with pytest.raises(urllib.error.HTTPError):
        urllib.request.urlopen(root + "../vendor/MANIFEST.json", timeout=10)


def test_fixture_pool_and_existing_recordings(server, tmp_path):
    d = call(server, "fixtures")
    assert [f["id"] for f in d["fixtures"]] == ["commode_a", "commode_b", "commode_c", "urinal_a", "washtub_a"]
    assert d["current"] == "commode_c"
    lib = call(server, "library")
    assert len(lib["files"]) == 11 and sum(1 for f in lib["files"] if f.get("unit")) == 8
    u = call(server, "unit/L-R04")
    assert u["recordings"][-1]["source"] == "library" and u["recordings"][-1]["original"] == "FILE_09.txt"
    # use another file: from the library (same arm only), or uploaded from this computer
    assert "RIGHT" in call(server, "unit/R-R02/import", {"library_file": "FILE_09.txt"})["error"]
    r = call(server, "unit/L-R01/import", {"library_file": "FILE_02.txt"})
    assert r["unit"]["recordings"][-1]["original"] == "FILE_02.txt"
    good = (vendor_paths.LITE_ROOT / "tests/data/recordings/commode_c_taught_paths/FILE_10.1.txt").read_text(encoding="utf-8")
    r = call(server, "unit/L-R02/import", {"name": "mine.txt", "content": good})
    assert r["unit"]["recordings"][-1]["source"] == "upload"
    assert "not a recording" in call(server, "unit/L-R03/import", {"name": "bad.txt", "content": "hello"})["error"]
    # another fixture opens its own project with its provisional regions, and back
    s = call(server, "fixture", {"id": "urinal_a"})
    assert s["fixture"] == "urinal_a"
    cat = call(server, "catalogue")
    assert len(cat["units"]) == 8 and all(u.get("provisional") for u in cat["units"])
    spec = call(server, "fixtures")["spec"]
    assert spec["parts"][0]["mesh"] == "/mesh/fixture/urinal_a/urinal.stl"
    root = server.replace("/api/lite/", "/")
    with urllib.request.urlopen(root + "mesh/fixture/urinal_a/urinal.stl", timeout=30) as r:
        assert r.status == 200
    assert call(server, "fixture", {"id": "commode_c"})["fixture"] == "commode_c"
