"""The Lite UI's API (/api/*): log-in and PINs, projects as folders, STOP without a session."""
import io
import json
import threading
import urllib.request
import zipfile

import pytest


@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-v2")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app, "data": data}
    srv.shutdown()


def req(lite, method, path, body=None, token=None, raw=None):
    headers = {"Content-Type": "application/json"}
    data = None if body is None else json.dumps(body).encode()
    if raw is not None:
        headers["Content-Type"], data = "application/zip", raw
    if token:
        headers["X-Lite-Token"] = token
    r = urllib.request.Request(lite["base"] + path, data=data, method=method, headers=headers)
    with urllib.request.urlopen(r, timeout=30) as resp:
        payload = resp.read()
        return payload if resp.headers.get("Content-Type") == "application/zip" else json.loads(payload)


def login(lite, pin="1234"):
    return req(lite, "POST", "/api/auth/login", {"pin": pin})


def test_login_needs_the_operator_pin(lite):
    assert "Incorrect" in login(lite, "0000")["error"]
    r = login(lite)
    assert r["token"] and r["role"] == "operator" and r["default_pin"] is True
    assert req(lite, "GET", "/api/projects")["code"] == "login"            # no token
    assert req(lite, "GET", "/api/projects", token="forged")["code"] == "login"
    assert "projects" in req(lite, "GET", "/api/projects", token=r["token"])


def test_pins_are_hashed_and_lock_out(lite):
    auth = lite["app"].auth
    stored = json.loads((lite["data"] / "auth.json").read_text()) if (lite["data"] / "auth.json").is_file() else auth._pins
    assert "1234" not in json.dumps(stored)
    from alix_studio_lite.server.auth import Auth
    from alix_studio_lite.server.jobs import UserError

    a = Auth(lite["data"] / "lockout")
    for _ in range(5):
        assert a.check("engineer", "9999") is False
    with pytest.raises(UserError, match="Too many wrong PINs"):
        a.check("engineer", "1234")
    a.set_pin("operator", "4321")
    assert Auth(lite["data"] / "lockout").check("operator", "4321") is True       # saved, reloaded
    with pytest.raises(ValueError):
        a.set_pin("operator", "12ab")


def test_engineer_pin_raises_the_session(lite):
    tok = login(lite)["token"]
    assert "Incorrect" in req(lite, "POST", "/api/auth/engineer", {"pin": "1111"}, tok)["error"]
    assert req(lite, "POST", "/api/auth/engineer", {"pin": "1234"}, tok)["engineer"] is True
    assert lite["app"].auth.session(tok)["engineer"] is True
    req(lite, "POST", "/api/auth/engineer/lock", {}, tok)
    assert lite["app"].auth.session(tok)["engineer"] is False
    req(lite, "POST", "/api/auth/logout", {}, tok)
    assert lite["app"].auth.session(tok) is None


def test_legacy_engineer_pin_is_the_same_pin(lite):
    base = "/api/lite/role"
    assert "not right" in req(lite, "POST", base, {"role": "engineer", "pin": "0000"})["error"]
    assert req(lite, "POST", base, {"role": "engineer", "pin": "1234"})["role"] == "engineer"
    req(lite, "POST", base, {"role": "operator"})


def test_stop_needs_no_session(lite):
    assert req(lite, "POST", "/api/robots/stop-both", {})["stopped"] == ["left", "right"]
    assert req(lite, "POST", "/api/robots/right/stop", {})["stopped"] == ["right"]
    assert req(lite, "GET", "/api/robots/status")["code"] == "login"


def page_record(pid, name, **state):
    return {"id": pid, "name": name, "createdAt": "2026-10-13T10:00:00.000Z", "updatedAt": "2026-10-13T10:00:00.000Z",
            "archived": False, "state": {"currentTask": 0, "currentView": "setup", **state}}


def test_projects_are_folders_the_server_owns(lite):
    tok = login(lite)["token"]
    rec = req(lite, "PUT", "/api/projects/alix_test1", page_record("alix_test1", "Toilet 01"), tok)
    folder = lite["data"] / "projects" / "alix_test1"
    assert rec["lite"]["fixture"] == "commode_c" and (folder / "trajectories/left").is_dir()
    # a page write never replaces the server's own fields
    forged = dict(page_record("alix_test1", "Toilet 01 renamed", currentTask=5), lite={"fixture": "urinal_a"})
    rec = req(lite, "PUT", "/api/projects/alix_test1", forged, tok)
    assert rec["name"] == "Toilet 01 renamed" and rec["lite"]["fixture"] == "commode_c"
    listing = req(lite, "GET", "/api/projects", token=tok)
    assert listing["last_id"] == "alix_test1"
    assert [p["state"]["currentTask"] for p in listing["projects"] if p["id"] == "alix_test1"] == [5]
    from alix_studio_lite.server.jobs import UserError

    with pytest.raises(UserError, match="valid project id"):
        lite["app"].store.folder("../x")
    assert "name" in req(lite, "PUT", "/api/projects/alix_bad", {"id": "alix_bad", "name": ""}, tok)["error"]

    # duplicate copies the files; delete moves the folder to the trash
    (folder / "trajectories/left/L_TASK_1A_trajectory.txt").write_text('{"point":[1,2,3,4,5,6,7]}\n')
    dup = req(lite, "POST", "/api/projects/alix_test1/duplicate", {}, tok)
    assert dup["name"].endswith("– Copy")
    assert (lite["data"] / "projects" / dup["id"] / "trajectories/left/L_TASK_1A_trajectory.txt").is_file()
    out = req(lite, "DELETE", f"/api/projects/{dup['id']}", token=tok)
    assert ".trash" in out["moved_to"] and not (lite["data"] / "projects" / dup["id"]).exists()
    assert dup["id"] not in [p["id"] for p in req(lite, "GET", "/api/projects", token=tok)["projects"]]


def test_project_file_round_trip(lite):
    tok = login(lite)["token"]
    req(lite, "PUT", "/api/projects/alix_trip", page_record("alix_trip", "Round trip", currentTask=3), tok)
    f = lite["data"] / "projects/alix_trip/trajectories/right/R_TASK_2_trajectory.txt"
    f.write_text('{"point":[7,6,5,4,3,2,1]}\n')
    blob = req(lite, "GET", "/api/projects/alix_trip/export", token=tok)
    names = zipfile.ZipFile(io.BytesIO(blob)).namelist()
    assert "project.json" in names and "trajectories/right/R_TASK_2_trajectory.txt" in names
    imp = req(lite, "POST", "/api/projects/import?name=Round_trip.alixproj", token=tok, raw=blob)
    assert imp["name"] == "Round trip – Imported" and imp["id"] != "alix_trip" and imp["state"]["currentTask"] == 3
    assert (lite["data"] / "projects" / imp["id"] / "trajectories/right/R_TASK_2_trajectory.txt").read_text() == f.read_text()
    # the designer's JSON export (schema alix_studio_lite_project_v1) is accepted as well
    page_json = {"schema": "alix_studio_lite_project_v1", "version": 1, "project": page_record("alix_x", "From the page")}
    imp2 = req(lite, "POST", "/api/projects/import", {"content": json.dumps(page_json), "name": "x.alixproj"}, tok)
    assert imp2["name"] == "From the page – Imported"
    # unsafe paths in a project file are refused
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr("project.json", json.dumps(page_record("alix_evil", "Evil")))
        z.writestr("../escape.txt", "x")
    assert "unsafe" in req(lite, "POST", "/api/projects/import", token=tok, raw=buf.getvalue())["error"]


def test_legacy_projects_move_once(tmp_path):
    from alix_studio_lite.server.project import migrate_legacy_store

    old = tmp_path / "projects" / "commode_c"
    (old / "units").mkdir(parents=True)
    (old / "project.json").write_text("{}")
    new_style = tmp_path / "projects" / "alix_abc"
    (new_style / "trajectories").mkdir(parents=True)
    (new_style / "project.json").write_text("{}")
    assert migrate_legacy_store(tmp_path) == ["commode_c"]
    assert (tmp_path / "legacy/projects/commode_c/units").is_dir() and new_style.is_dir()
    assert migrate_legacy_store(tmp_path) == []
