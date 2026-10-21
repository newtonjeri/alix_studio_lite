"""The Lite UI's region loop over HTTP, against the robot stand-in: the task queue (D35), an
existing recording used by default (D31), tuning with every limit checked, keeping a version,
upload with replace confirmation, Hardware sim and Real runs (D30), the verdict, the E-stop
interlock, and Time-optimal's own time rule.

And the gates on every execution: a trajectory runs only on the arm it was taught on, only with
its stage (pole height, hand pose) known, and only after the pole and then the hand are staged
and read back; when either fails, nothing else runs."""
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
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()   # shuts down at once
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


def log_text(job):
    return " | ".join(e["text"] for e in job["events"])


def gate(u, gid):
    return next(g for g in u["gates"] if g["id"] == gid)


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
        assert t["right"]["key"] == "R-T04" and not t["right"]["later"] and not t["left"]["later"]
    # every step released (the owner, 21 Oct: the drain brush and glove steps were held): the deck's procedure
    # tasks are taught as the others, with no note saying "later"
    for lab in ("TASK 13", "TASK 14"):
        assert tasks[labels.index(lab)]["kind"] == "mixed" and not tasks[labels.index(lab)]["note"]
    assert not [s["key"] for t in tasks for s in (t["left"], t["right"]) if s and s["later"]]
    # existing recordings are in place from the start (D31)
    t3 = tasks[labels.index("TASK 3")]
    assert t3["left"]["key"] == "L-T01" and t3["left"]["library"] and t3["left"]["status"] == "taught"
    assert t3["left"]["own"] == 0                           # the fixture's taught set: not the operator's work (IN PROGRESS)


def test_one_arm_step_through_the_loop(lite):
    key = "L-T01"
    u = unit(lite, key)
    assert u["active"] and u["recordings"][0]["source"] == "library" and u["recordings"][0]["name"] == "FILE_10.1.txt"
    s = u["settings_view"]["settings"]
    assert (s["optimiser"], s["trim"], s["smooth"], s["smooth_window"], s["scale"]) == ("manual", True, True, 101, True)  # the HTML's
    assert (s["idle_window"], s["idle_threshold_md"], s["idle_min_run"], s["time_mode"]) == (11, 8.0, 20, "fastest")
    assert "no tuned" in unit(lite, key, "keep", {})["error"]
    rec = u["recordings"][0]                        # the existing recording keeps its arm and stage
    assert rec["taught_arm"] == "left" and rec["stage"]["text"] == "pole 200 mm, fingers 50 % open, thumb held"
    assert u["setup"]["pole_mm"] == 200
    assert gate(u, "arm")["ok"] and gate(u, "stage")["ok"] and "pole 200 mm" in gate(u, "stage")["text"]
    assert not gate(u, "limits")["ok"] and not gate(u, "robot")["ok"]

    # tune: the candidate passes every limit, then is kept
    j = wait(lite, unit(lite, key, "optimise", {}))
    assert j["state"] == "done", j
    u = unit(lite, key)
    r = u["candidate"]["result"]
    assert u["fresh"] and r["uploadable"] and r["optimiser"] == "manual" and u["candidate_extra"]["progress"]
    assert r["enforced"] is False and (r["limits_ok"] or r["limits_text"])            # shown, never enforced
    v = unit(lite, key, "keep", {})["versions"][-1]
    assert v["version"] == "v001" and v["taught_arm"] == "left" and v["stage"]["pole_mm"] == 200

    # a settings change makes the candidate stale until re-run; undo brings it back
    u = unit(lite, key, "settings", {"changes": {"segments": [{"start": 1, "end": 500, "pct": 80}]}, "label": "Scale"})
    assert not u["fresh"] and u["settings_view"]["can_undo"]
    assert "changed" in unit(lite, key, "keep", {})["error"]
    assert unit(lite, key, "undo", {})["fresh"]
    # as the HTML takes them: a scale below 1 % is 1 %; a window kept as typed, read odd when the file is made
    from alix_studio_lite.optimise import manual

    tiny = unit(lite, key, "settings", {"changes": {"segments": [{"start": 1, "end": 5, "pct": 0.5}]}})
    assert tiny["settings_view"]["settings"]["segments"][0]["pct"] == 1.0
    assert unit(lite, key, "undo", {})["fresh"]
    over = [{"start": 1, "end": 50, "pct": 80}, {"start": 50, "end": 90, "pct": 80}]
    assert "may not overlap" in unit(lite, key, "settings", {"changes": {"segments": over}})["error"]
    s40 = unit(lite, key, "settings", {"changes": {"smooth_window": 40}})["settings_view"]["settings"]
    assert s40["smooth_window"] == 40 and manual.params(s40)["smooth_window"] == 41
    assert unit(lite, key, "undo", {})["fresh"]

    # robot steps need a robot mode and a connected arm (D30)
    assert unit(lite, key, "upload", {"program_id": 7})["code"] == "mode"
    assert call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})["code"] == "connect"
    for arm in ("left", "right"):
        assert call(lite, "POST", f"/api/robots/{arm}/connect", {})["estop"] == "CLEAR"
    assert call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})["mode"] == "hwsim"
    assert "1 to 100" in unit(lite, key, "upload", {"program_id": 101})["error"]
    assert "PASS needs a completed run" in unit(lite, key, "verdict", {"verdict": "pass"})["error"]
    r = unit(lite, key, "upload", {"program_id": 7})               # manual: what it breaks is shown, then it is sent
    assert r["code"] == "limits" and "does not hold manual tuning to the limits" in r["error"], r
    assert wait(lite, unit(lite, key, "upload", {"program_id": 7, "limits_seen": True}))["state"] == "done"
    time.sleep(0.4)

    # a second version to the same slot needs the replace confirmation
    unit(lite, key, "settings", {"changes": {"smooth_window": 21}, "label": "Light smoothing"})
    assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done"
    assert unit(lite, key, "keep", {})["versions"][-1]["version"] == "v002"
    r = unit(lite, key, "upload", {"program_id": 7, "limits_seen": True})
    assert r["code"] == "replace" and "already holds" in r["error"]
    assert "Upload it again" in unit(lite, key, "run", {})["hint"]          # never runs a stale program
    assert wait(lite, unit(lite, key, "upload", {"program_id": 7, "replace": True, "limits_seen": True}))["state"] == "done"
    time.sleep(0.4)

    # Hardware sim run: the pole and then the hand first, emulated (nothing moves); then PASS
    j = wait(lite, unit(lite, key, "run", {}))
    assert j["state"] == "done", j
    text = log_text(j)
    assert text.index("Pole at 200 mm (emulated") < text.index("Hand in pose (emulated") < text.index("Finished")
    assert call(lite, "GET", "/api/robots/status")["arms"]["left"]["pole_mm"] == 200
    time.sleep(0.4)
    u = unit(lite, key)
    assert u["last_run"]["kind"] == "dryrun_ok" and u["status"] == "dryrun_ok"
    assert gate(u, "limits")["ok"] and gate(u, "robot")["ok"]
    assert unit(lite, key, "verdict", {"verdict": "pass"})["results"][-1]["detail"]["verdict"] == "pass"

    # Real needs confirmation for the mode and for every motion
    assert call(lite, "POST", "/api/robots/mode", {"mode": "real"})["code"] == "confirm"
    assert call(lite, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})["mode"] == "real"
    assert unit(lite, key, "run", {})["code"] == "confirm"
    assert call(lite, "POST", "/api/robots/left/pole", {"mm": 120})["code"] == "confirm"
    call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})


def test_staging_failure_means_nothing_runs(lite):
    """The pole or the hand not arriving ends the job before the arm moves, in every execution."""
    key = "L-T01"                                    # v002 is on program 7 of the LEFT arm (above)
    assert call(lite, "POST", "/api/robots/left/fault", {"pole": True})["code"] == "engineer"
    call(lite, "POST", "/api/auth/engineer", {"pin": "1234"})
    before = len(unit(lite, key)["results"])

    assert wait(lite, call(lite, "POST", "/api/robots/left/pole", {"mm": 100}))["state"] == "done"
    assert call(lite, "POST", "/api/robots/left/fault", {"pole": True})["faults"] == ["pole"]
    j = wait(lite, unit(lite, key, "run", {}))
    assert j["state"] == "failed" and "LEFT pole did not reach 200 mm" in j["error"] and "Nothing else ran" in j["error"]
    assert "Hand" not in log_text(j) and "Running program" not in (j["stage"] or "")
    time.sleep(0.4)
    u = unit(lite, key)
    assert u["results"][-1]["kind"] == "run_failed" and len(u["results"]) == before + 1
    assert "pole did not reach" in wait(lite, unit(lite, key, "move-start", {"which": "version"}))["error"]
    assert "pole did not reach" in wait(lite, unit(lite, key, "playback", {}))["error"]

    call(lite, "POST", "/api/robots/left/fault", {"pole": False, "hand": True})
    assert wait(lite, call(lite, "POST", "/api/robots/left/hand", {"counts": [0, 0, 0, 0, 150, 1000]}))["state"] == "done"
    j = wait(lite, unit(lite, key, "run", {}))
    assert j["state"] == "failed" and "LEFT hand did not reach its pose" in j["error"]
    assert "Pole at 200 mm (emulated" in log_text(j) and "Finished" not in log_text(j)

    call(lite, "POST", "/api/robots/left/fault", {"hand": False})
    j = wait(lite, unit(lite, key, "run", {}))
    assert j["state"] == "done" and "Hand in pose (emulated" in log_text(j)
    call(lite, "POST", "/api/auth/engineer/lock", {})


def test_estop_blocks_both_arms_and_stop_always_works(lite):
    assert call(lite, "POST", "/api/robots/right/estop-test", {"pressed": True})["code"] == "engineer"
    call(lite, "POST", "/api/auth/engineer", {"pin": "1234"})
    assert call(lite, "POST", "/api/robots/right/estop-test", {"pressed": True})["estop"] == "ENGAGED"
    r = unit(lite, "L-T01", "move-start", {"which": "version"})
    assert "E-stop" in r["error"]
    assert call(lite, "GET", "/api/robots/status")["motion_block"]
    r = call(lite, "POST", "/api/robots/stop-both", {})
    assert r["stopped"] == ["left", "right"] and "press Recover" in r["message"]
    arms = call(lite, "GET", "/api/robots/status")["arms"]          # STOP holds the arms, like the E-stop
    assert arms["left"]["stopped"] and arms["left"]["estop"] == "RECOVERING"
    assert not arms["right"]["stopped"] and arms["right"]["estop"] == "ENGAGED"   # the button keeps its own words
    assert "still pressed" in call(lite, "POST", "/api/robots/right/recover", {})["error"]
    call(lite, "POST", "/api/robots/right/estop-test", {"pressed": False})
    assert call(lite, "POST", "/api/robots/right/recover", {})["estop"] == "CLEAR"
    assert "stopped by STOP" in call(lite, "GET", "/api/robots/status")["motion_block"]
    assert call(lite, "POST", "/api/robots/left/recover", {})["estop"] == "CLEAR"
    assert wait(lite, unit(lite, "L-T01", "move-start", {"which": "version"}))["state"] == "done"
    call(lite, "POST", "/api/auth/engineer/lock", {})


def test_loaded_file_never_replaces_the_recording(lite):
    key = "L-R01"
    assert unit(lite, key)["recordings"] == []
    assert "not a trajectory" in unit(lite, key, "recordings", {"source": "upload", "name": "x.txt", "content": "hello"})["error"]
    from alix_studio_lite import vendor_paths

    txt = (vendor_paths.LITE_ROOT / "tests/data/recordings/commode_c_taught_paths/FILE_09.txt").read_text()
    stage = {"pole_mm": 140, "hand": "100% open"}
    r = unit(lite, key, "recordings", {"source": "upload", "name": "old_base_left.txt", "content": txt})
    assert r["code"] == "stage"                                         # the stage it was taught with first
    r = unit(lite, key, "recordings", {"source": "upload", "name": "older.txt", "content": txt} | stage)
    assert r["code"] == "arm" and "Which arm" in r["error"]            # the name does not say the arm
    for bad in ({"name": "R_spillage.txt"}, {"name": "spill.txt", "arm": "right"}, {"name": "L_x.txt", "arm": "right"}):
        r = unit(lite, key, "recordings", {"source": "upload", "content": txt} | stage | bad)
        assert r["code"] == "arm", bad                                  # a RIGHT file never reaches a LEFT region
    u = unit(lite, key, "recordings", {"source": "upload", "name": "old_base_left.txt", "content": txt} | stage)["unit"]
    first = u["active"]
    u = unit(lite, key, "recordings", {"source": "upload", "name": "older.txt", "content": txt, "arm": "left",
                                       "pole_mm": 50, "hand": "GRIP"})["unit"]
    assert len(u["recordings"]) == 2 and u["active"] != first
    assert [r["stage"]["text"] for r in u["recordings"]] == ["pole 140 mm, fingers 100 % open, thumb held",
                                                             "pole 50 mm, hand grip"]
    tasks = call(lite, "GET", "/api/projects/alix_loop/tasks")["tasks"]
    assert next(t["left"] for t in tasks if t["left"] and t["left"]["key"] == key)["own"] == 2     # the operator's own files
    assert unit(lite, key, "active", {"file": first})["active"] == first
    u = unit(lite, key, "name", {"name": "L_TASK_1A_trajectory.txt"})
    assert [r["name"] for r in u["recordings"] if r["file"] == first] == ["L_TASK_1A_trajectory.txt"]
    tr = call(lite, "GET", f"/api/projects/alix_loop/units/{key}/track?which=raw")
    assert tr["lines"] > 1000 and len(tr["joints"]) == 7
    tl = call(lite, "GET", f"/api/projects/alix_loop/units/{key}/timeline?which=raw")
    assert tl["side"] == "left" and tl["lines"] == tr["lines"] and tl["stage"]["pole_mm"] == 140
    assert [p["name"] for p in tl["phases"]] == ["pole", "hand", "move", "trajectory"]
    assert tl["phases"][0]["text"] == "Pole 300 → 140 mm"


def test_a_recording_without_its_stage_never_runs(lite):
    """A recording kept before stages were recorded is shown with its stage unknown: it is not
    simulated, tuned or run until the operator states it (once; a recorded stage never changes)."""
    key = "L-R08"
    from alix_studio_lite import vendor_paths

    txt = (vendor_paths.LITE_ROOT / "tests/data/legacy_v6/LEFT/L_P06P07.txt").read_text()
    u = unit(lite, key, "recordings", {"source": "upload", "name": "L_P06P07.txt", "content": txt,
                                       "pole_mm": 150, "hand": "GRIP"})["unit"]
    file = u["active"]
    src = lite["app"].store.folder("alix_loop") / "trajectories/left" / key / "raw" / file
    meta = json.loads(src.with_suffix(".source.json").read_text())
    meta.pop("stage")
    src.with_suffix(".source.json").write_text(json.dumps(meta))
    u = unit(lite, key)
    assert u["recordings"][-1]["stage"] is None and not gate(u, "stage")["ok"]
    assert call(lite, "GET", f"/api/projects/alix_loop/units/{key}/timeline?which=raw")["code"] == "stage"
    assert unit(lite, key, "optimise", {})["code"] == "stage"
    u = unit(lite, key, f"recordings/{file}/stage", {"pole_mm": 150, "hand": "GRIP"})
    assert u["recordings"][-1]["stage"]["text"] == "pole 150 mm, hand grip" and gate(u, "stage")["ok"]
    assert "cannot change" in unit(lite, key, f"recordings/{file}/stage", {"pole_mm": 50, "hand": "GRIP"})["error"]
    assert call(lite, "GET", f"/api/projects/alix_loop/units/{key}/timeline?which=raw")["stage"]["pole_mm"] == 150


def test_teaching_needs_real_the_setup_and_keeps_the_arm(lite):
    """Drag teaching needs Real (in Hardware sim the pole and the hand are only emulated, so a
    recording's stage could not be true), starts only with the arm in the region's setup, read
    back, and the stand-in's take is a recording of the SAME arm: a LEFT trajectory never becomes a
    RIGHT region's recording."""
    key = "R-R01_PART1"
    u = unit(lite, key)
    assert u["setup"]["text"] == "pole 150 mm, hand grip" and u["arm"] == "right"
    for arm in ("left", "right"):
        call(lite, "POST", f"/api/robots/{arm}/connect", {})
    assert call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})["mode"] == "hwsim"
    r = unit(lite, key, "teach/start", {})
    assert r["code"] == "mode" and "needs Real robot" in r["error"]
    j = wait(lite, call(lite, "POST", "/api/robots/right/pole", {"mm": 260}))
    assert j["state"] == "done" and "emulated" in log_text(j)                          # nothing moved
    assert call(lite, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})["mode"] == "real"
    assert unit(lite, key, "teach/start", {})["code"] == "confirm"
    r = unit(lite, key, "teach/start", {"confirmed": True})
    assert r["code"] == "stage" and "not in this region's setup" in r["error"]
    assert unit(lite, key, "setup", {"pole_mm": 150, "hand": "GRIP", "apply": True})["code"] == "confirm"
    out = unit(lite, key, "setup", {"pole_mm": 150, "hand": "GRIP", "apply": True, "confirmed": True})
    j = wait(lite, out["job"])
    assert j["state"] == "done" and "Pole at 150 mm (read back" in log_text(j)
    assert unit(lite, key, "teach/start", {"confirmed": True})["arm"]["teaching"]
    time.sleep(0.3)
    out = unit(lite, key, "teach/stop", {})
    rec = out["unit"]["recordings"][-1]
    assert rec["taught_arm"] == "right" and out["take"].startswith(("R", "M3")) and rec["stage"]["pole_mm"] in (50, 150)
    assert gate(out["unit"], "arm")["ok"] and gate(out["unit"], "stage")["ok"]
    from alix_studio_lite.server.library import builtin_by_name

    assert builtin_by_name(out["take"])["arm"] == "right"
    assert unit(lite, key, "playback", {})["code"] == "mode"                          # never as taught on the arms
    call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})


def test_every_step_can_be_taught(lite):
    """The owner, 21 Oct: "release all the tasks" (the drain brush and glove steps were held, "not in Release 1")."""
    for key in ("R-T04", "R-T05_PART1", "L-T04", "R-T07"):
        assert "Release 1" not in str(unit(lite, key, "teach/start", {}).get("error"))


def test_time_optimal_keeps_its_limits():
    """TOPP-RA works on the recording as taught: the fastest time within the limits, which it keeps."""
    pytest.importorskip("toppra")
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.optimise import optimise, request_from_file

    f = vendor_paths.LITE_ROOT / "tests/data/recordings/commode_c_taught_paths/FILE_10.1.txt"
    fast = optimise(request_from_file(f, "left"))
    s = fast.summary()
    assert s["optimiser"] == "toppra" and s["enforced"] and s["uploadable"] == s["limits_ok"]
    assert fast.uploadable and "Fastest safe time" in fast.message and fast.achieved_s < fast.recorded_s
