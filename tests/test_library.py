"""The Trajectory Viewer's library (/api/library): taught .txt files uploaded and checked, the
built-in sets, 3D frames and plot data, the limits as taught, and using a file for a region."""
import json
import threading
import urllib.request

import pytest

from alix_studio_lite import vendor_paths

V6 = vendor_paths.LITE_ROOT / "tests/data/legacy_v6"


@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    vendor_paths.activate()
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-lib")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app, "data": data}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json"}
    if ctx.get("tok"):
        headers["X-Lite-Token"] = ctx["tok"]
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=120) as resp:
        return json.loads(resp.read())


def upload(ctx, path, name=None, arm=None):
    return call(ctx, "POST", "/api/library", {"name": name or path.name, "content": path.read_text(),
                                              "path": f"{path.parent.name}/{path.name}", "arm": arm})


def test_arm_from_names_and_folders():
    from alix_studio_lite.server.library import detect_arm

    assert detect_arm("L_P01P02.txt") == "left" and detect_arm("RIGHT/M3_ASOPN01.txt") == "right"
    assert detect_arm("left_drag_teach.txt") == "left" and detect_arm("r_spillage.txt") == "right"
    assert detect_arm("LEFT/R15_BUPFUL.txt") == "left"        # R15 is a region, not the right arm
    assert detect_arm("R2_LIDTOP.txt") is None and detect_arm("lid_top.txt") is None


def test_builtin_sets(lite):
    d = call(lite, "GET", "/api/library")
    sets = {s["id"]: s for s in d["sets"]}
    assert len(sets["commode_c_taught"]["files"]) == 11 and len(sets["v6_legacy"]["files"]) == 15
    v6 = sets["v6_legacy"]["files"]
    assert {f["arm"] for f in v6} == {"left", "right"} and all("path" not in f for f in v6)
    again = call(lite, "GET", "/api/library")
    assert [f["id"] for f in again["sets"][1]["files"]] == [f["id"] for f in v6]       # stable ids


def test_upload_check_and_arm(lite):
    d = upload(lite, V6 / "RIGHT/R2_LIDTOP.txt")
    f = next(x for x in d["files"] if x["id"] == d["added"])
    assert f["arm"] == "right" and f["lines"] == 3751 and f["seconds"] == 18.75
    assert upload(lite, V6 / "RIGHT/R2_LIDTOP.txt")["added"] == f["id"]             # same file: one copy
    bad = call(lite, "POST", "/api/library", {"name": "notes.txt", "content": "hello\n"})
    assert "not a trajectory" in bad["error"]
    d = call(lite, "POST", "/api/library", {"name": "lid_top.txt", "content": (V6 / "LEFT/L_P01P02.txt").read_text()})
    plain = next(x for x in d["files"] if x["id"] == d["added"])
    assert plain["arm"] is None
    assert call(lite, "GET", f"/api/library/{plain['id']}/frames")["code"] == "arm"
    d = call(lite, "POST", f"/api/library/{plain['id']}", {"arm": "left"})
    assert next(x for x in d["files"] if x["id"] == plain["id"])["arm"] == "left"


def test_frames_track_and_checks(lite):
    d = call(lite, "GET", "/api/library")
    left = next(f for f in d["sets"][1]["files"] if f["name"] == "L_P01P02.txt")
    fr = call(lite, "GET", f"/api/library/{left['id']}/frames?every=50")
    assert fr["side"] == "left" and fr["total_lines"] == 7005 and fr["lines"][-1] == 7004
    tr = call(lite, "GET", f"/api/library/{left['id']}/track")
    assert tr["lines"] == 7005 and len(tr["joints"]) == 7
    ch = call(lite, "GET", f"/api/library/{left['id']}/checks")
    by = {c["id"]: c for c in ch["checks"]}
    assert ch["seconds"] == 35.02 and by["L3"]["passed"] is False and by["L5"]["passed"] is None
    assert isinstance(by["C2"]["passed"], bool)                                      # plain JSON, no numpy


def test_use_for_a_region_and_delete(lite):
    rec = {"id": "alix_lib", "name": "Lib", "archived": False, "state": {}}
    call(lite, "PUT", "/api/projects/alix_lib", rec)
    d = upload(lite, V6 / "RIGHT/R2_LIDTOP.txt")
    fid = d["added"]
    r = call(lite, "POST", f"/api/library/{fid}/use", {"pid": "alix_lib", "key": "L-R01"})
    assert "RIGHT arm" in r["error"]
    r = call(lite, "POST", f"/api/library/{fid}/use", {"pid": "alix_lib", "key": "R-R01_PART1"})
    u = r["unit"]
    assert u["recordings"][-1]["name"] == "R2_LIDTOP.txt" and u["active"] == u["recordings"][-1]["file"]
    d = call(lite, "DELETE", f"/api/library/{fid}")
    assert fid not in [f["id"] for f in d["files"]]
    assert (lite["data"] / "library/.trash").is_dir()
    builtin = d["sets"][0]["files"][0]["id"]
    assert "cannot be deleted" in call(lite, "DELETE", f"/api/library/{builtin}")["error"]
