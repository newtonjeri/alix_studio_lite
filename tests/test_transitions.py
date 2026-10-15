"""V32's Task Transition & Start Point Tools (Prepare), against the robot stand-in: a transition
trajectory kept in the project with its arm and stage, its 3D timeline, its limits checked before
it is sent, run on its arm alone (the other arm stays stopped); a start point at the previous
task's end; Move to Start Point by joints, within the arm's range; Capture needs a controller."""
import json
import threading
import time
import urllib.request

import numpy as np
import pytest


@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-transitions")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()   # shuts down at once
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app, "data": data}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    call(ctx, "PUT", "/api/projects/alix_tr", {"id": "alix_tr", "name": "Transitions", "createdAt": "2026-10-14T10:00:00.000Z",
                                               "updatedAt": "2026-10-14T10:00:00.000Z", "archived": False,
                                               "state": {"currentTask": 0}})
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json", "X-Lite-Token": ctx.get("tok") or ""}
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=120) as resp:
        return json.loads(resp.read())


def unit(ctx, key, verb, body=None):
    return call(ctx, "GET" if body is None else "POST", f"/api/projects/alix_tr/units/{key}/{verb}", body)


def wait(ctx, job, limit=120):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < limit, j
        time.sleep(0.2)


def smooth_file(start_millideg, j1_deg=4.0, lines=401):
    """A transition within every limit: J1 eases ``j1_deg`` over (lines - 1) x 5 ms."""
    from alix_studio_lite.motion import track

    s = (1 - np.cos(np.linspace(0, np.pi, lines))) / 2
    md = np.repeat(np.asarray(start_millideg, dtype=np.int64)[None, :], lines, axis=0)
    md[:, 0] += np.round(s * j1_deg * 1000).astype(np.int64)
    return track.track_bytes(md).decode()


def test_a_transition_is_kept_with_its_arm_and_stage(lite):
    end = unit(lite, "L-T01", "end")                   # Previous Task End: the existing recording's last line
    assert end["which"] == "raw" and end["name"] == "FILE_10.1.txt" and len(end["joints_deg"]) == 7
    assert end["stage"]["pole_mm"] == 200
    start = [round(v * 1000) for v in end["joints_deg"]]
    text = smooth_file(start)
    assert unit(lite, "L-T01", "transition", {"name": "R_to_seat.txt", "content": text, "pole_mm": 200, "hand": "OPEN"})["code"] == "arm"
    assert unit(lite, "L-T01", "transition", {"name": "to_seat.txt", "content": text, "arm": "left"})["code"] == "stage"
    assert "not a trajectory" in unit(lite, "L-T01", "transition", {"name": "x.txt", "content": "hello", "arm": "left",
                                                                    "pole_mm": 200, "hand": "OPEN"})["error"]
    assert unit(lite, "L-T01", "transition")["transition"] is None
    tv = unit(lite, "L-T01", "transition", {"name": "to_seat.txt", "content": text, "arm": "left", "pole_mm": 200,
                                            "hand": "OPEN"})["transition"]
    assert tv["name"] == "to_seat.txt" and tv["lines"] == 401 and tv["seconds"] == 2.0 and tv["limits_ok"]
    assert tv["project_path"].startswith("trajectories/transitions/left/L-T01/") and tv["stage"]["pole_mm"] == 200
    assert abs(tv["end_joints_deg"][0] - (end["joints_deg"][0] + 4)) < 0.01
    assert unit(lite, "L-T01", "transition")["transition"]["sha256"] == tv["sha256"]   # kept: still there after a reload
    # one name per file in a project: the recording's name is numbered
    again = unit(lite, "L-T01", "transition", {"name": "FILE_10.1.txt", "content": text, "arm": "left", "pole_mm": 200,
                                               "hand": "OPEN"})["transition"]
    assert again["name"] == "FILE_10.1_2.txt"
    # the 3D: staged first, then the file on the 5 ms clock
    tl = call(lite, "GET", "/api/projects/alix_tr/units/L-T01/timeline?which=transition")
    assert tl["lines"] == 401 and tl["stage"]["pole_mm"] == 200


def test_a_transition_runs_on_its_arm_alone(lite):
    end = unit(lite, "L-T01", "end")
    text = smooth_file([round(v * 1000) for v in end["joints_deg"]])
    unit(lite, "L-T01", "transition", {"name": "run_me.txt", "content": text, "arm": "left", "pole_mm": 200, "hand": "OPEN"})
    assert unit(lite, "L-T01", "transition/upload", {"program_id": 31})["code"] == "mode"
    assert call(lite, "GET", "/api/robots/left/capture")["code"] == "connect"          # not connected
    for arm in ("left", "right"):
        call(lite, "POST", f"/api/robots/{arm}/connect", {})
    cap = call(lite, "GET", "/api/robots/left/capture")
    assert cap["code"] == "connect" and "stand-in" in cap["error"]                     # no controller to read
    assert call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})["mode"] == "hwsim"

    # Move to Start Point: seven joints within the arm's range
    assert "seven joint values" in call(lite, "POST", "/api/robots/left/move-joints", {"joints_deg": [0, 0]})["error"]
    bad = list(end["joints_deg"])
    bad[3] = 400
    assert "outside the arm's range" in call(lite, "POST", "/api/robots/left/move-joints", {"joints_deg": bad})["error"]
    j = wait(lite, call(lite, "POST", "/api/robots/left/move-joints", {"joints_deg": end["joints_deg"], "name": "A"}))
    assert j["state"] == "done" and "start point “A”" in " ".join(e["text"] for e in j["events"]), j

    assert "1 to 100" in unit(lite, "L-T01", "transition/upload", {"program_id": 0})["error"]
    assert "does not hold this transition" in unit(lite, "L-T01", "transition/run", {"program_id": 31})["error"]
    assert wait(lite, unit(lite, "L-T01", "transition/upload", {"program_id": 31}))["state"] == "done"
    assert unit(lite, "L-T01", "transition/upload", {"program_id": 31})["unchanged"]
    run = unit(lite, "L-T01", "transition/run", {"program_id": 31})
    assert run["state"] == "running", run
    refused = call(lite, "POST", "/api/robots/right/pole", {"mm": 150})               # the other arm stays stopped
    assert "alone" in refused["error"] and "RIGHT arm stays stopped" in refused["error"], refused
    j = wait(lite, run)
    assert j["state"] == "done" and "Hardware sim transition LEFT" in j["title"], j

    pole = call(lite, "POST", "/api/robots/right/pole", {"mm": 120})                   # the other arm moving
    assert pole["state"] == "running"
    busy = unit(lite, "L-T01", "transition/run", {"program_id": 31})
    assert "RIGHT arm is moving" in busy["error"], busy
    assert wait(lite, pole)["state"] == "done"


def test_a_transition_beyond_the_limits_is_never_sent(lite):
    end = unit(lite, "L-T01", "end")
    start = [round(v * 1000) for v in end["joints_deg"]]
    lines = json.loads("[" + ",".join(smooth_file(start).strip().splitlines()) + "]")
    for row in lines[200:]:
        row["point"][1] += 15000                         # a 15 degree jump on one line: L4
    text = "\n".join(json.dumps(r, separators=(",", ":")) for r in lines) + "\n"
    tv = unit(lite, "L-T01", "transition", {"name": "jump.txt", "content": text, "arm": "left", "pole_mm": 200,
                                            "hand": "OPEN"})["transition"]
    assert not tv["limits_ok"] and "L4" in tv["limits_text"]
    assert call(lite, "GET", "/api/projects/alix_tr/units/L-T01/timeline?which=transition")["lines"] == 401   # 3D still
    r = unit(lite, "L-T01", "transition/upload", {"program_id": 32})
    assert "L4" in r["error"] and "never goes to the robot" in r["error"], r


def test_the_project_folder_has_every_kept_file(lite):
    """V32's Save Project Folder copies what Lite keeps for the project: the recordings, the kept
    files and the transitions with their records; never the record itself (the page writes it) nor an
    unkept tuning; only the folder's own files are served."""
    import hashlib

    files = call(lite, "GET", "/api/projects/alix_tr/folder")["files"]
    paths = [f["path"] for f in files]
    assert any(p.startswith("trajectories/left/L-T01/raw/") and p.endswith(".txt") for p in paths), paths
    assert any(p.startswith("trajectories/left/L-T01/raw/") and p.endswith(".source.json") for p in paths)
    assert any(p.startswith("trajectories/transitions/left/L-T01/") and p.endswith(".txt") for p in paths)
    assert "project.json" not in paths and not any("/work/" in p for p in paths)
    f = next(f for f in files if f["path"].startswith("trajectories/transitions/") and f["path"].endswith(".txt"))
    r = urllib.request.Request(lite["base"] + "/api/projects/alix_tr/folder/file?path=" + urllib.request.quote(f["path"]),
                               headers={"X-Lite-Token": lite["tok"]})
    with urllib.request.urlopen(r, timeout=30) as resp:
        data = resp.read()
    assert len(data) == f["bytes"] and hashlib.sha256(data).hexdigest() == f["sha256"]
    for bad in ("project.json", "../../alix_tr/project.json", "/etc/passwd"):
        assert "not in the project" in call(lite, "GET", "/api/projects/alix_tr/folder/file?path=" + urllib.request.quote(bad))["error"]
