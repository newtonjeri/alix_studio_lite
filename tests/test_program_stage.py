"""A program run from its slot runs with its own pole and hand (the owner, 21 Oct: "the project's own pole and hand
values must be applied. They must not be replaced by the default pole and hand positions. This applies wherever the
project runs"). Lite records the pole and hand with every program it sends (the stage of the very file sent, the one it
was taught with); a run from the slot (Advanced's Run Stored, the Trajectory Viewer) stages them first; one Lite did not
send runs only with the pole and hand the operator states; in S2 a Program whose own are not set before it stops Real
Machine Mode."""
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

    data = tmp_path_factory.mktemp("lite-stage")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    assert call(ctx, "POST", "/api/robots/left/connect", {})["connected"]
    assert call(ctx, "POST", "/api/robots/mode", {"mode": "hwsim"})["mode"] == "hwsim"
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json", "X-Lite-Token": ctx.get("tok") or ""}
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=120) as resp:
        return json.loads(resp.read())


def wait(ctx, job, limit=120):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < limit, j
        time.sleep(0.1)


def content(start=0.0):
    from alix_studio_lite.motion import track

    t = np.linspace(0.0, 1.0, 120)[:, None]
    md = np.round((np.array([[10, -40, 0, 80, 0, 40, 0]]) + start + 4.0 * np.sin(2 * np.pi * t)) * 1000).astype(np.int64)
    return track.track_bytes(md).decode()


def said(job) -> str:
    return " | ".join(e["text"] for e in job.get("events") or [])


def test_a_program_sent_with_its_pole_and_hand_runs_with_them(lite):
    up = call(lite, "POST", "/api/robots/left/programs/upload", {"program_id": 41, "name": "L_own.txt", "content": content(),
                                                                "pole_mm": 120, "hand": "GRIP", "confirmed": True})
    assert wait(lite, up)["state"] == "done"
    rec = lite["app"].robot.arm("left").programs[41]
    assert rec["stage"]["pole_mm"] == 120 and rec["stage"]["hand"] == [0, 0, 0, 0, 150, 1000]      # recorded with it
    run = call(lite, "POST", "/api/robots/left/programs/run", {"program": 41, "confirmed": True})
    done = wait(lite, run)
    assert done["state"] == "done", done
    assert "Program 41: pole and hand pole 120 mm, hand grip (its own, as Lite sent it) first." in said(done)
    # stated values never replace its own
    run = call(lite, "POST", "/api/robots/left/programs/run", {"program": 41, "pole_mm": 30, "hand": "OPEN", "confirmed": True})
    assert "pole 120 mm, hand grip (its own" in said(wait(lite, run))


def test_a_program_lite_does_not_know_runs_only_with_what_the_operator_states(lite):
    up = call(lite, "POST", "/api/robots/left/programs/upload", {"program_id": 42, "name": "L_unknown.txt", "content": content(1.0),
                                                                "confirmed": True})
    assert wait(lite, up)["state"] == "done"
    assert lite["app"].robot.arm("left").programs[42].get("stage") is None
    refused = call(lite, "POST", "/api/robots/left/programs/run", {"program": 42, "confirmed": True})
    assert refused["code"] == "stage" and "does not know the pole height and hand pose program 42 runs with" in refused["error"]
    run = call(lite, "POST", "/api/robots/left/programs/run", {"program": 42, "pole_mm": 80, "hand": "CLEANING", "confirmed": True})
    done = wait(lite, run)
    assert done["state"] == "done" and "pole 80 mm, hand cleaning pose (as stated) first." in said(done), said(done)


def test_s2_stops_a_program_whose_own_pole_and_hand_are_not_set_before_it():
    """V53 stages nothing itself: the sequence's own Pole and Hand steps must set the Program's; otherwise Real Machine
    Mode does not run it (a stop, not a question), with or without a simulation."""
    from types import SimpleNamespace

    from alix_studio_lite.server import workflow

    own = {"pole_mm": 50.0, "hand": [0, 0, 0, 0, 150, 1000]}
    programs = {"left": {71: {"stage": own}}, "right": {}}
    robot = SimpleNamespace(_measured=lambda side: (200.0, [1000] * 6))
    run = lambda steps: {"steps": {"left": [dict(s, _row=i + 1) for i, s in enumerate(steps)], "right": []}}   # noqa: E731
    stops = workflow._stage_stops(run([{"type": "program", "id": 71}]), programs, robot, ["left"])
    assert stops == ["LEFT row 1, program 71: taught at pole 50 mm, hand grip, but the pole is at 200 mm here (as the arm "
                     "stands: no Pole step before it in this run) and the hand is open here (as the arm stands: no Hand "
                     "step before it). It runs only with its own: set them before it (a Pole step and a Hand step)."]
    ok = [{"type": "pole", "mm": 50}, {"type": "hand", "values": [0, 0, 0, 0, 150, 1000]}, {"type": "program", "id": 71}]
    assert workflow._stage_stops(run(ok), programs, robot, ["left"]) == []
    unknown = {"left": {71: {"stage": None}}, "right": {}}                   # Lite did not send it: nothing to hold it to
    assert workflow._stage_stops(run([{"type": "program", "id": 71}]), unknown, robot, ["left"]) == []
