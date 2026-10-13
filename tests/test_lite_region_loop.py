"""The Lite UI's region loop over HTTP, against the robot stand-in: the task queue (D35), an
existing recording used by default (D31), tuning with every limit checked, keeping a version,
upload with replace confirmation, Hardware sim and Real runs (D30), the verdict, the E-stop
interlock, and the optimiser's cuts and recorded-pace mode."""
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

    data = tmp_path_factory.mktemp("lite-loop")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    ctx = {"base": base, "app": app, "data": data}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    rec = {"id": "alix_loop", "name": "Loop", "createdAt": "2026-10-13T10:00:00.000Z",
           "updatedAt": "2026-10-13T10:00:00.000Z", "archived": False, "state": {"currentTask": 0}}
    call(ctx, "PUT", "/api/projects/alix_loop", rec)
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json"}
    if ctx.get("tok"):
        headers["X-Lite-Token"] = ctx["tok"]
    data = None if body is None else json.dumps(body).encode()
    r = urllib.request.Request(ctx["base"] + path, data=data, method=method, headers=headers)
    with urllib.request.urlopen(r, timeout=120) as resp:
        return json.loads(resp.read())


def unit(ctx, key, verb="", body=None, method=None):
    path = f"/api/projects/alix_loop/units/{key}" + (f"/{verb}" if verb else "")
    return call(ctx, method or ("POST" if verb else "GET"), path, body if verb else None)


def wait(ctx, job, timeout=120):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < timeout, j
        time.sleep(0.2)


def test_task_queue_follows_the_deck(lite):
    d = call(lite, "GET", "/api/projects/alix_loop/tasks")
    tasks = d["tasks"]
    assert d["fixture"] == "commode_c" and d["has_deck"] and len(tasks) == 22
    labels = [t["label"] for t in tasks]
    assert labels[:4] == ["TASK 1A", "TASK 2A", "TASK 1C", "TASK 2"] and labels[-1] == "TASK 15"
    t9 = tasks[labels.index("TASK 9")]                      # the deck's sides (slide 17), not V19's
    assert t9["left"]["display"] == "L-R08 SEAT UNDERSIDE LEFT HALF"
    assert t9["right"]["display"] == "R-R08 FLOOR SPILLAGE RIGHT PART 2"
    for lab in ("TASK 10A", "TASK 10B"):                    # one action spanning two tasks
        t = tasks[labels.index(lab)]
        assert t["right"]["key"] == "R-T04" and t["right"]["later"] and not t["left"]["later"]
    assert tasks[labels.index("TASK 13")]["kind"] == "procedure"
    later = {s["key"] for t in tasks for s in (t["left"], t["right"]) if s and s["later"]}
    assert later == {"L-T06", "L-T04", "R-T04", "R-T10", "R-T05_PART1", "R-T05_PART2", "R-T06", "R-T07"}
    # existing recordings are in place from the start (D31)
    t3 = tasks[labels.index("TASK 3")]
    assert t3["left"]["key"] == "L-T01" and t3["left"]["library"] and t3["left"]["status"] == "taught"


def test_one_arm_step_through_the_loop(lite):
    key = "L-T01"
    u = unit(lite, key)
    assert u["active"] and u["recordings"][0]["source"] == "library" and u["recordings"][0]["name"] == "FILE_10.1.txt"
    s = u["settings_view"]["settings"]
    assert (s["trim"], s["smooth"], s["smooth_window"], s["scale"], s["time_mode"]) == (True, True, 41, True, "natural")
    assert "no tuned" in unit(lite, key, "keep", {})["error"]

    # tune: the candidate passes every limit, then is kept
    j = wait(lite, unit(lite, key, "optimise", {}))
    assert j["state"] == "done", j
    u = unit(lite, key)
    assert u["fresh"] and u["candidate"]["result"]["uploadable"] and u["candidate_extra"]["progress"]
    assert unit(lite, key, "keep", {})["versions"][-1]["version"] == "v001"

    # a settings change makes the candidate stale until re-run; undo brings it back
    u = unit(lite, key, "settings", {"changes": {"segments": [{"start": 1, "end": 500, "pct": 80}]}, "label": "Scale"})
    assert not u["fresh"] and u["settings_view"]["can_undo"]
    assert "changed" in unit(lite, key, "keep", {})["error"]
    assert unit(lite, key, "undo", {})["fresh"]
    assert "Scale is" in unit(lite, key, "settings", {"changes": {"segments": [{"start": 1, "end": 5, "pct": 900}]}})["error"]

    # robot steps need a robot mode and a connected arm (D30)
    assert unit(lite, key, "upload", {"program_id": 7})["code"] == "mode"
    assert call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})["code"] == "connect"
    for arm in ("left", "right"):
        assert call(lite, "POST", f"/api/robots/{arm}/connect", {})["estop"] == "CLEAR"
    assert call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})["mode"] == "hwsim"
    assert "1 to 100" in unit(lite, key, "upload", {"program_id": 101})["error"]
    assert "PASS needs a completed run" in unit(lite, key, "verdict", {"verdict": "pass"})["error"]
    assert wait(lite, unit(lite, key, "upload", {"program_id": 7}))["state"] == "done"
    time.sleep(0.4)

    # a second version to the same slot needs the replace confirmation
    unit(lite, key, "settings", {"changes": {"smooth_window": 21}, "label": "Light smoothing"})
    assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done"
    assert unit(lite, key, "keep", {})["versions"][-1]["version"] == "v002"
    r = unit(lite, key, "upload", {"program_id": 7})
    assert r["code"] == "replace" and "already holds" in r["error"]
    assert "Upload it again" in unit(lite, key, "run", {})["hint"]          # never runs a stale program
    assert wait(lite, unit(lite, key, "upload", {"program_id": 7, "replace": True}))["state"] == "done"
    time.sleep(0.4)

    # Hardware sim run, then PASS
    assert wait(lite, unit(lite, key, "run", {}))["state"] == "done"
    time.sleep(0.4)
    u = unit(lite, key)
    assert u["last_run"]["kind"] == "dryrun_ok" and u["status"] == "dryrun_ok"
    assert unit(lite, key, "verdict", {"verdict": "pass"})["results"][-1]["detail"]["verdict"] == "pass"

    # Real needs confirmation for the mode and for every motion
    assert call(lite, "POST", "/api/robots/mode", {"mode": "real"})["code"] == "confirm"
    assert call(lite, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})["mode"] == "real"
    assert unit(lite, key, "run", {})["code"] == "confirm"
    assert call(lite, "POST", "/api/robots/left/pole", {"mm": 120})["code"] == "confirm"
    call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})


def test_estop_blocks_both_arms_and_stop_always_works(lite):
    assert call(lite, "POST", "/api/robots/right/estop-test", {"pressed": True})["code"] == "engineer"
    call(lite, "POST", "/api/auth/engineer", {"pin": "1234"})
    assert call(lite, "POST", "/api/robots/right/estop-test", {"pressed": True})["estop"] == "ENGAGED"
    r = unit(lite, "L-T01", "move-start", {"which": "version"})
    assert "E-stop" in r["error"]
    assert call(lite, "GET", "/api/robots/status")["motion_block"]
    assert call(lite, "POST", "/api/robots/stop-both", {})["stopped"] == ["left", "right"]
    assert "still pressed" in call(lite, "POST", "/api/robots/right/recover", {})["error"]
    call(lite, "POST", "/api/robots/right/estop-test", {"pressed": False})
    assert call(lite, "POST", "/api/robots/right/recover", {})["estop"] == "CLEAR"
    assert wait(lite, unit(lite, "L-T01", "move-start", {"which": "version"}))["state"] == "done"
    call(lite, "POST", "/api/auth/engineer/lock", {})


def test_loaded_file_never_replaces_the_recording(lite):
    key = "L-R01"
    assert unit(lite, key)["recordings"] == []
    assert "not a trajectory" in unit(lite, key, "recordings", {"source": "upload", "name": "x.txt", "content": "hello"})["error"]
    from alix_studio_lite import vendor_paths

    txt = (vendor_paths.LITE_ROOT / "tests/data/recordings/commode_c_taught_paths/FILE_09.txt").read_text()
    u = unit(lite, key, "recordings", {"source": "upload", "name": "old_base_left.txt", "content": txt})["unit"]
    first = u["active"]
    u = unit(lite, key, "recordings", {"source": "upload", "name": "older.txt", "content": txt})["unit"]
    assert len(u["recordings"]) == 2 and u["active"] != first
    assert unit(lite, key, "active", {"file": first})["active"] == first
    u = unit(lite, key, "name", {"name": "L_TASK_1A_trajectory.txt"})
    assert [r["name"] for r in u["recordings"] if r["file"] == first] == ["L_TASK_1A_trajectory.txt"]
    tr = call(lite, "GET", f"/api/projects/alix_loop/units/{key}/track?which=raw")
    assert tr["lines"] > 1000 and len(tr["joints"]) == 7
    fr = call(lite, "GET", f"/api/projects/alix_loop/units/{key}/frames?which=raw&every=40")
    assert fr["side"] == "left" and fr["total_lines"] == tr["lines"] and fr["lines"][-1] == tr["lines"] - 1


def test_later_steps_cannot_be_taught(lite):
    assert "not in Release 1" in unit(lite, "R-T04", "teach/start", {})["error"]


def test_cuts_are_bridged_and_segments_follow():
    import numpy as np

    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.optimise import OptRequest, Segment
    from alix_studio_lite.optimise.engine import apply_cuts

    md = np.zeros((100, 7), dtype=np.int64)
    md[:, 0] = np.arange(100) * 10                      # 0.01 deg per line on joint 1
    md[60:, 0] += 5000                                  # a 5 deg jump after the cut
    req = OptRequest(md_raw=md, side="left", cuts=((41, 60),), segments=(Segment(30, 70, 50),))
    out, info = apply_cuts(req)
    bridge = info["bridges"][0]
    assert info["removed_lines"] == 20 and bridge["after_line"] == 40 and bridge["before_line"] == 61
    jump_md = int(md[60, 0] - md[39, 0])                # 5.21 deg across the gap
    k = round(bridge["seconds"] / 0.005)
    assert k == int(np.ceil(jump_md / 1000 / 20 / 0.005)) and out.md_raw.shape[0] == 80 + k - 1
    assert np.abs(np.diff(out.md_raw[:, 0])).max() <= np.ceil(jump_md / k) + 1     # crossed at 20 deg/s
    seg = out.segments[0]                               # rows 30..70 -> 30..40 then 61..70 after the bridge
    assert (seg.start, seg.end) == (30, 40 + (k - 1) + 10)


def test_recorded_pace_mode():
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.optimise import optimise, request_from_file

    f = vendor_paths.LITE_ROOT / "tests/data/recordings/commode_c_taught_paths/FILE_10.1.txt"
    nat = optimise(request_from_file(f, "left", natural=True))
    fast = optimise(request_from_file(f, "left"))
    assert nat.uploadable and fast.uploadable
    assert nat.achieved_s > fast.achieved_s and "Recorded pace kept" in nat.message
    assert nat.achieved_s <= nat.recorded_s + 0.01
