"""Fixes from the final review: what needs a session, the E-stop latch and what an E-stop ends, no
mode change while something moves, a kept version used only with its kept bytes, imported projects
without another computer's evidence, region files renamed only inside their region, PINs reloaded
and never guessed, a stop with a reason, and the request guards."""
import http.client
import io
import json
import threading
import time
import urllib.request
import zipfile

import pytest


@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-fixes")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app, "data": data}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"}, tok=False)["token"]
    rec = {"id": "alix_fix", "name": "Fixes", "createdAt": "2026-10-13T10:00:00.000Z",
           "updatedAt": "2026-10-13T10:00:00.000Z", "archived": False, "state": {"currentTask": 0}}
    call(ctx, "PUT", "/api/projects/alix_fix", rec)
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None, tok=True, raw=None):
    headers = {"Content-Type": "application/json"}
    if tok:
        headers["X-Lite-Token"] = ctx["tok"]
    data = None if body is None else json.dumps(body).encode()
    if raw is not None:
        headers["Content-Type"], data = "application/zip", raw
    r = urllib.request.Request(ctx["base"] + path, data=data, method=method, headers=headers)
    with urllib.request.urlopen(r, timeout=120) as resp:
        payload = resp.read()
        return payload if resp.headers.get("Content-Type") == "application/zip" else json.loads(payload)


def unit(ctx, key, verb="", body=None, pid="alix_fix"):
    path = f"/api/projects/{pid}/units/{key}" + (f"/{verb}" if verb else "")
    return call(ctx, "POST" if verb else "GET", path, body if verb else None)


def wait(ctx, job, timeout=120):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < timeout, j
        time.sleep(0.2)


def test_only_log_in_stop_and_hello_need_no_session(lite):
    for path in ("/api/lite/state", "/api/robots/status", "/api/library", "/api/projects/alix_fix/tasks"):
        assert call(lite, "GET", path, tok=False)["code"] == "login", path
    assert call(lite, "POST", "/api/lite/role", {"role": "engineer", "pin": "1234"}, tok=False)["code"] == "login"
    assert call(lite, "GET", "/api/lite/hello", tok=False)["app"] == "ALIX Studio Lite"     # how a second start finds Lite
    assert call(lite, "POST", "/api/robots/stop-both", {}, tok=False)["stopped"] == ["left", "right"]
    assert "error" not in call(lite, "POST", "/api/lite/robot/stop", {"arm": "all"}, tok=False)


def test_a_bad_content_length_is_refused(lite):
    host, port = lite["base"].rsplit("/", 1)[1].split(":")
    c = http.client.HTTPConnection(host, int(port), timeout=10)
    c.putrequest("POST", "/api/auth/login")
    c.putheader("Content-Type", "application/json")
    c.putheader("Content-Length", "12abc")
    c.endheaders()
    assert c.getresponse().status == 400
    c.close()


def test_estop_latches_and_ends_motion_on_both_arms(lite):
    call(lite, "POST", "/api/auth/engineer", {"pin": "1234"})
    for arm in ("left", "right"):
        call(lite, "POST", f"/api/robots/{arm}/connect", {})
    assert call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})["mode"] == "hwsim"
    job = call(lite, "POST", "/api/robots/left/pole", {"mm": 300})
    assert job["state"] == "running"
    assert call(lite, "POST", "/api/robots/mode", {"mode": "visual"})["code"] == "busy"    # not while it moves
    time.sleep(0.3)
    assert call(lite, "POST", "/api/robots/right/estop-test", {"pressed": True})["estop"] == "ENGAGED"
    j = wait(lite, job)                                      # the RIGHT E-stop ends the LEFT pole move too
    assert j["state"] == "failed" and j["error"].startswith("INTERRUPTED — E-STOP") and "Recover" in j["hint"]

    call(lite, "POST", "/api/robots/right/estop-test", {"pressed": False})
    st = call(lite, "GET", "/api/robots/status")
    assert st["arms"]["right"]["estop"] == "RECOVERING" and st["motion_block"]           # released is not recovered
    assert "E-stop" in call(lite, "POST", "/api/robots/left/pole", {"mm": 100})["error"]
    call(lite, "POST", "/api/robots/right/disconnect", {})
    assert call(lite, "POST", "/api/robots/right/connect", {})["estop"] == "RECOVERING"   # reconnecting keeps the latch
    assert call(lite, "POST", "/api/robots/right/recover", {})["estop"] == "CLEAR"
    assert not call(lite, "GET", "/api/robots/status")["motion_block"]
    assert call(lite, "POST", "/api/robots/mode", {"mode": "visual"})["mode"] == "visual"
    call(lite, "POST", "/api/auth/engineer/lock", {})


def test_a_version_is_used_only_as_kept_and_imports_drop_evidence(lite):
    key = "L-T01"
    assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done"
    assert unit(lite, key, "keep", {})["versions"][-1]["version"] == "v001"
    call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})
    kept = sorted((lite["data"] / "projects" / "alix_fix").rglob("versions/*.txt"))[-1]
    good = kept.read_bytes()
    kept.write_bytes(good.replace(b"\n", b"\n\n", 1))         # changed on disk after it was kept
    r = unit(lite, key, "upload", {"program_id": 9, "limits_seen": True})
    assert "bytes have changed" in r["error"]
    kept.write_bytes(good)
    assert wait(lite, unit(lite, key, "upload", {"program_id": 9, "limits_seen": True}))["state"] == "done"
    call(lite, "POST", "/api/robots/mode", {"mode": "visual"})

    blob = call(lite, "GET", "/api/projects/alix_fix/export")
    names = zipfile.ZipFile(io.BytesIO(blob)).namelist()
    assert any("/versions/" in n for n in names)
    imp = call(lite, "POST", "/api/projects/import?name=Fixes.alixproj", raw=blob)
    u = unit(lite, key, pid=imp["id"])
    assert u["versions"] == [] and u["last_run"] is None and u["results"] == []   # another computer's evidence
    assert u["active"] == unit(lite, key)["active"]                              # the recordings come along
    assert not list((lite["data"] / "projects" / imp["id"]).rglob("versions/*"))


def test_a_region_renames_only_its_own_files(lite):
    sentinel = lite["data"] / "sentinel.txt"
    sentinel.write_text("x")
    r = unit(lite, "L-T01", "name", {"file": "../../../../../sentinel.txt", "name": "moved.txt"})
    assert "error" in r and sentinel.read_text() == "x"


def test_pins_reload_and_a_damaged_file_is_never_guessed(tmp_path):
    from alix_studio_lite.server.auth import Auth

    a = Auth(tmp_path)
    assert a.check("operator", "1234")
    Auth(tmp_path).set_pin("operator", "2468")               # as `run_lite --set-pin` does while Lite runs
    assert a.check("operator", "2468") and not a.check("operator", "1234")
    (tmp_path / "auth.json").write_text("{ damaged")
    with pytest.raises(RuntimeError):                         # at start-up: refuse, never fall back to 1234
        Auth(tmp_path)
    assert a.check("operator", "2468")                        # a running Lite keeps the PINs it had


def test_a_stop_with_a_reason_ends_the_job_as_failed():
    from alix_studio_lite.server.jobs import Jobs

    def long(job):
        for _ in range(200):
            job.sleep(0.05)

    def until_done(j):
        t0 = time.time()
        while j.state == "running" and time.time() - t0 < 5:
            time.sleep(0.05)
        return j

    jobs = Jobs()
    j = jobs.start("move", "Moving", long, arm="left")
    time.sleep(0.1)
    assert jobs.cancel_arm("all", "INTERRUPTED — E-STOP: it did not finish.", "Release the E-stop.") == 1
    assert until_done(j).state == "failed" and j.error.startswith("INTERRUPTED") and j.hint == "Release the E-stop."
    j2 = jobs.start("move", "Moving", long, arm="right")
    time.sleep(0.1)
    jobs.cancel_arm("right")
    assert until_done(j2).state == "cancelled" and j2.error is None


def test_a_second_start_tells_this_lite_an_older_one_and_another_program(lite, tmp_path, capsys):
    """Starting Lite on a port in use opens it when it is this same Lite; an older Lite (whose server
    still runs its old code) must be stopped first; anything else is another program."""
    from http.server import BaseHTTPRequestHandler, HTTPServer

    from alix_studio_lite import code_id
    from alix_studio_lite.server import _running_lite, main

    port = int(lite["base"].rsplit(":", 1)[1])
    me = _running_lite(port)
    assert me["code"] == code_id() and me["pid"] > 0
    assert main(["--port", str(port), "--no-browser", "--data", str(tmp_path / "d1"), "--logs", str(tmp_path / "logs")]) == 0
    assert "already running" in capsys.readouterr().out

    class Old(BaseHTTPRequestHandler):                       # a Lite from before /api/lite/hello
        server_version = "ALIXStudioLite/0.0.9"

        def do_GET(self):
            self.send_error(404)

        def log_message(self, *a):
            pass

    class Other(Old):
        server_version = "SimpleHTTP/0.6"

    for cls, want, said in ((Old, {}, "older ALIX Studio Lite"), (Other, None, "another program")):
        srv = HTTPServer(("127.0.0.1", 0), cls)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        try:
            assert _running_lite(srv.server_address[1]) == want
            assert main(["--port", str(srv.server_address[1]), "--no-browser", "--data", str(tmp_path / "d2"), "--logs", str(tmp_path / "logs")]) == 2
            assert said in capsys.readouterr().err
        finally:
            srv.shutdown()


def test_lite_opens_in_a_chrome_window_of_its_own_that_ignores_the_gpu_blocklist(tmp_path, monkeypatch):
    """Where the graphics run in software, Chrome turns WebGL off for every page (the lab laptop):
    Lite's window ignores that block, in a profile of its own so the switch applies while another
    Chrome is open. A snap Chromium cannot use that profile, and without Chrome a tab opens."""
    import shutil
    import subprocess
    import webbrowser

    from alix_studio_lite import server

    started, tabs = [], []
    monkeypatch.setattr(server, "CHROME_PROFILE", tmp_path / "chrome")
    monkeypatch.setattr(subprocess, "Popen", lambda argv, **kw: started.append((argv, kw)))
    monkeypatch.setattr(webbrowser, "open", tabs.append)
    found = {"google-chrome": "/usr/bin/google-chrome"}
    monkeypatch.setattr(shutil, "which", lambda name: found.get(name))
    server._open_browser("http://127.0.0.1:8766/")
    (argv, kw), = started
    assert argv[0] == "/usr/bin/google-chrome" and argv[-1] == "http://127.0.0.1:8766/"
    assert "--ignore-gpu-blocklist" in argv and f"--user-data-dir={tmp_path / 'chrome'}" in argv
    assert kw["start_new_session"] and (tmp_path / "chrome").is_dir() and not tabs

    found = {"chromium": "/snap/bin/chromium"}
    server._open_browser("http://127.0.0.1:8766/")
    found = {}
    server._open_browser("http://127.0.0.1:8766/")
    assert len(started) == 1 and tabs == ["http://127.0.0.1:8766/"] * 2
