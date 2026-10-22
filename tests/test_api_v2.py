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
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()   # shuts down at once
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
    assert all(v != "1234" for rec in stored.values() for v in rec.values())   # the PIN itself is never stored
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
    """Legacy Lite needs the log-in too, and its engineer level belongs to the session that
    entered the PIN, not to the server."""
    base = "/api/lite/role"
    assert req(lite, "POST", base, {"role": "engineer", "pin": "1234"})["code"] == "login"
    tok = req(lite, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    other = req(lite, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    assert "Incorrect PIN" in req(lite, "POST", base, {"role": "engineer", "pin": "0000"}, tok)["error"]
    assert req(lite, "POST", base, {"role": "engineer", "pin": "1234"}, tok)["role"] == "engineer"
    assert req(lite, "GET", "/api/lite/state", token=other)["role"] == "operator"     # not every client
    assert req(lite, "POST", "/api/lite/robot/estop-test", {"arm": "left", "pressed": True}, other)["code"] == "engineer"
    req(lite, "POST", base, {"role": "operator"}, tok)


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
    folder = lite["data"] / "projects" / "Toilet 01"                 # named as the project was created
    assert rec["lite"]["fixture"] == "commode_c" and (folder / "trajectories/left").is_dir()
    assert rec["lite"]["folder"] == "Toilet 01" and lite["app"].store.folder("alix_test1") == folder
    # a page write never replaces the server's own fields
    forged = dict(page_record("alix_test1", "Toilet 01 renamed", currentTask=5), lite={"fixture": "urinal_a"})
    rec = req(lite, "PUT", "/api/projects/alix_test1", forged, tok)
    assert rec["name"] == "Toilet 01 renamed" and rec["lite"]["fixture"] == "commode_c"
    assert rec["lite"]["folder"] == "Toilet 01" and folder.is_dir()   # a rename keeps the folder's name
    listing = req(lite, "GET", "/api/projects", token=tok)
    assert listing["last_id"] == "alix_test1"
    assert [p["state"]["currentTask"] for p in listing["projects"] if p["id"] == "alix_test1"] == [5]
    from alix_studio_lite.server.jobs import UserError

    with pytest.raises(UserError, match="valid project id"):
        lite["app"].store.folder("../x")
    assert "name" in req(lite, "PUT", "/api/projects/alix_bad", {"id": "alix_bad", "name": ""}, tok)["error"]
    # a new project's name names its folder: one a folder has (letter case aside) or with / or \ is refused
    assert "already" in req(lite, "PUT", "/api/projects/alix_twin", page_record("alix_twin", "toilet 01"), tok)["error"]
    for bad in ("Left/Right", "a\\b", ".hidden"):
        assert "folder" in req(lite, "PUT", "/api/projects/alix_bad", page_record("alix_bad", bad), tok)["error"]
        assert req(lite, "POST", "/api/projects/name-check", {"name": bad}, tok)["ok"] is False
    check = req(lite, "POST", "/api/projects/name-check", {"name": "TOILET 01"}, tok)
    assert check["ok"] is False and "renamed since" in check["problem"]
    assert req(lite, "POST", "/api/projects/name-check", {"name": "Toilet 02"}, tok) == {
        "ok": True, "problem": None, "folder": str(lite["data"] / "projects")}
    assert not (lite["data"] / "projects" / "Left").exists()

    # duplicate copies the files; delete moves the folder to the trash
    (folder / "trajectories/left/L_TASK_1A_trajectory.txt").write_text('{"point":[1,2,3,4,5,6,7]}\n')
    dup = req(lite, "POST", "/api/projects/alix_test1/duplicate", {}, tok)
    assert dup["name"] == "Toilet 01 renamed – Copy" and dup["lite"]["folder"] == dup["name"]
    assert (lite["data"] / "projects" / dup["name"] / "trajectories/left/L_TASK_1A_trajectory.txt").is_file()
    dup2 = req(lite, "POST", "/api/projects/alix_test1/duplicate", {}, tok)
    assert dup2["name"] == "Toilet 01 renamed – Copy 2" and (lite["data"] / "projects" / dup2["name"]).is_dir()
    out = req(lite, "DELETE", f"/api/projects/{dup['id']}", token=tok)
    assert ".trash" in out["moved_to"] and not (lite["data"] / "projects" / dup["name"]).exists()
    assert dup["id"] not in [p["id"] for p in req(lite, "GET", "/api/projects", token=tok)["projects"]]


def test_a_project_is_seen_without_opening_it(lite):
    """Open / Manage Projects (the owner, 22 Oct: "open and view existing projects"): what a project holds, read from
    its folder (GET /api/projects/<id>/overview); and a project folder put in the projects folder by hand, or by git,
    while Lite runs is listed by the next read."""
    tok = login(lite)["token"]
    rec = req(lite, "PUT", "/api/projects/alix_view1", page_record("alix_view1", "Viewed 01", completed={"0": True, "2": True}), tok)
    folder = lite["data"] / "projects" / rec["lite"]["folder"]
    (folder / "sequence").mkdir(exist_ok=True)
    (folder / "sequence" / "s2.json").write_text(json.dumps({"left": [{"type": "wait", "seconds": 1}] * 3, "right": [],
                                                               "updated": "2026-10-22 21:08:15"}))
    v = req(lite, "GET", "/api/projects/alix_view1/overview", token=tok)
    assert v["name"] == "Viewed 01" and v["folder"] == str(folder) and v["fixture"] == "commode_c" and v["fixture_name"]
    assert v["done"] == [0, 2] and len(v["tasks"]) >= 20 and v["tasks"][0]["left"]["code"].startswith("L-")
    assert v["sequence"] == {"left": 3, "right": 0, "updated": "2026-10-22 21:08:15"} and v["simulation"] is None
    assert v["merges"] == {"left": 0, "right": 0} and v["files"]["count"] >= 2 and v["files"]["bytes"] > 0
    assert req(lite, "GET", "/api/projects/alix_nothere/overview", token=tok)["error"] == "Project not found."
    assert req(lite, "GET", "/api/projects/alix_view1/overview")["code"] == "login"
    pulled = lite["data"] / "projects" / "Pulled 01"                 # as git pull or a copy would bring it
    pulled.mkdir()
    (pulled / "project.json").write_text(json.dumps(dict(page_record("alix_pulled1", "Pulled 01"), lite={"folder": "Pulled 01"})))
    assert "alix_pulled1" in [p["id"] for p in req(lite, "GET", "/api/projects", token=tok)["projects"]]
    assert req(lite, "GET", "/api/projects/alix_pulled1/overview", token=tok)["name"] == "Pulled 01"


def test_project_file_round_trip(lite):
    tok = login(lite)["token"]
    req(lite, "PUT", "/api/projects/alix_trip", page_record("alix_trip", "Round trip", currentTask=3), tok)
    f = lite["data"] / "projects/Round trip/trajectories/right/R_TASK_2_trajectory.txt"
    f.write_text('{"point":[7,6,5,4,3,2,1]}\n')
    blob = req(lite, "GET", "/api/projects/alix_trip/export", token=tok)
    names = zipfile.ZipFile(io.BytesIO(blob)).namelist()
    assert "project.json" in names and "trajectories/right/R_TASK_2_trajectory.txt" in names
    imp = req(lite, "POST", "/api/projects/import?name=Round_trip.alixproj", token=tok, raw=blob)
    assert imp["name"] == "Round trip – Imported" and imp["id"] != "alix_trip" and imp["state"]["currentTask"] == 3
    assert (lite["data"] / "projects" / "Round trip – Imported" / "trajectories/right/R_TASK_2_trajectory.txt").read_text() == f.read_text()
    again = req(lite, "POST", "/api/projects/import?name=Round_trip.alixproj", token=tok, raw=blob)
    assert again["name"] == again["lite"]["folder"] == "Round trip – Imported 2"
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


def test_projects_kept_by_id_are_copied_once(tmp_path):
    """Projects Lite kept as <data>/projects/<id>/ before 16 October are copied into the projects
    folder once, each in a folder named as the project is; the old folders stay."""
    from alix_studio_lite.server.store import ProjectStore

    home, root = tmp_path / "home" / "projects", tmp_path / "Resources" / "Projects"
    def old(pid, name, **lite):
        (home / pid / "trajectories/left").mkdir(parents=True)
        (home / pid / "trajectories/left/L.txt").write_text(pid)
        (home / pid / "project.json").write_text(json.dumps(dict(page_record(pid, name), lite=lite or {"fixture": "commode_c"})))
    old("alix_a1", "Toilet A")
    old("alix_b2", "Left/Right test")
    old("alix_c3", "toilet a")                               # the same name as another, letter case aside
    (home / "commode_c" / "units").mkdir(parents=True)        # Legacy Lite's: not the Lite UI's
    (home / "commode_c" / "project.json").write_text("{}")
    store = ProjectStore(root, home=home)
    got = {p["id"]: p for p in store.adopt(home)}
    assert set(got) == {"alix_a1", "alix_b2", "alix_c3"}
    assert (root / "Toilet A/trajectories/left/L.txt").read_text() == "alix_a1"
    assert got["alix_b2"]["name"] == "Left-Right test" and got["alix_b2"]["renamed"] == "Left/Right test"
    assert got["alix_c3"]["name"] == "toilet a 2" and store.get("alix_c3")["name"] == "toilet a 2"
    assert store.get("alix_a1")["lite"] == {"fixture": "commode_c", "folder": "Toilet A"}
    assert (home / "alix_a1/project.json").is_file()          # the old folders stay
    assert store.adopt(home) == []                            # once
    store.delete("alix_a1")
    assert store.adopt(home) == [] and not store.exists("alix_a1")    # one deleted here is not copied again
    store.set_last("alix_b2")
    assert json.loads((home / "last.json").read_text()) == {"id": "alix_b2"} and store.last_id == "alix_b2"
    assert not (root / "last.json").exists() and not (root / ".trash").exists()


def test_project_folders_hold_on_every_computer_and_to_their_project(tmp_path, monkeypatch):
    """A project's folder name must be one Windows can have too (the folders go between computers with the
    repository); a folder copied or swapped by hand never takes another project's place; a copy cut short
    leaves no project behind, and is made again at the next start."""
    from alix_studio_lite.server import store as st
    from alix_studio_lite.server.jobs import UserError

    s = st.ProjectStore(tmp_path / "Projects", home=tmp_path / "home")
    for bad in ("Room: 3", "a|b", 'q"x', "<x>", "Test?", "CON", "com1.txt", "Trailing.", "Trailing "):
        assert st.folder_problem(bad), bad
        with pytest.raises(UserError, match="folder"):
            s.put("alix_bad", page_record("alix_bad", bad))
    assert st.usable("Room: 3?") == "Room- 3-" and st.usable("CON") == "CON project" and st.usable("Ends. ") == "Ends"
    assert not list((tmp_path / "Projects").glob(".making-*"))

    s.put("alix_one", page_record("alix_one", "Toilet 01"))
    s.put("alix_two", page_record("alix_two", "Toilet 02"))
    import shutil
    shutil.copytree(tmp_path / "Projects/Toilet 01", tmp_path / "Projects/A backup of Toilet 01")   # by hand, sorting first
    s.list()
    assert s.folder("alix_one").name == "Toilet 01"
    fresh = st.ProjectStore(tmp_path / "Projects", home=tmp_path / "home")          # a restart: the one its record names
    assert fresh.folder("alix_one").name == "Toilet 01"
    shutil.rmtree(tmp_path / "Projects/A backup of Toilet 01")
    (tmp_path / "Projects/Toilet 01").rename(tmp_path / "Projects/swap")             # the two swapped by hand
    (tmp_path / "Projects/Toilet 02").rename(tmp_path / "Projects/Toilet 01")
    (tmp_path / "Projects/swap").rename(tmp_path / "Projects/Toilet 02")
    assert s.get("alix_one")["id"] == "alix_one" and s.folder("alix_one").name == "Toilet 02"
    assert s.get("alix_two")["name"] == "Toilet 02" and s.folder("alix_two").name == "Toilet 01"

    old = tmp_path / "home"                                                           # copied once, even when cut short
    (old / "alix_old1" / "trajectories/left").mkdir(parents=True)
    (old / "alix_old1/trajectories/left/L.txt").write_text("x")
    (old / "alix_old1/project.json").write_text(json.dumps(page_record("alix_old1", "Old one")))
    real = shutil.copytree
    monkeypatch.setattr(st.shutil, "copytree", lambda *a, **k: (_ for _ in ()).throw(OSError("No space left on device")))
    out = s.adopt(old)
    assert out == [{"id": "alix_old1", "name": "Old one", "error": "No space left on device"}]
    assert not s.exists("alix_old1") and not (tmp_path / "Projects/Old one").exists()
    assert not list((tmp_path / "Projects").glob(".making-*"))
    monkeypatch.setattr(st.shutil, "copytree", real)
    assert [p["name"] for p in s.adopt(old)] == ["Old one"] and s.folder("alix_old1").name == "Old one"
