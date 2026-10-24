"""A program run from its slot runs with its own pole and hand (the owner, 21 Oct: "the project's own pole and hand
values must be applied. They must not be replaced by the default pole and hand positions. This applies wherever the
project runs"). Lite records the pole and hand with every program it sends (the stage of the very file sent, the one it
was taught with); a run from the slot (Advanced's Run Stored, the Trajectory Viewer) stages them first; one Lite did not
send runs only with the pole and hand the operator states; in S2 Lite sets each Program's own before it (24 Oct)."""
import json
import threading
import time
import urllib.request
from pathlib import Path

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


def test_s2_sets_each_programs_own_pole_and_hand_before_it():
    """V53 stages nothing itself, and S2 stopped a Program whose own pole and hand its rows did not set (21 Oct); since
    24 Oct (the owner: "for uploaded tasks, the app applies the stored pole and hand itself") Lite sets them before it:
    own_of reads each Program's own from what Lite recorded when it sent it, and its first point; one Lite did not send
    has none, and runs with its rows (the operator's), as before."""
    from alix_studio_lite.server import workflow

    grip = [0, 0, 0, 0, 150, 1000]
    programs = {"left": {71: {"stage": {"pole_mm": 50.0, "hand": grip}, "sent": True, "first_row": [1000, -2000, 0, 0, 0, 0, 0]},
                         72: {"stage": None, "sent": False, "name": "72_other"}}, "right": {}}
    rows = [{"type": "program", "id": 71}, {"type": "pole", "mm": 120}, {"type": "program", "id": 72}]
    steps = {"left": workflow.check_steps(rows, "left"), "right": []}
    own = workflow.own_of(programs, {"left": {}, "right": {}}, lambda _p: None, steps)
    assert own["left"][71]["stage"]["pole_mm"] == 50.0 and own["left"][71]["stage"]["hand"] == grip
    assert own["left"][71]["first_deg"] == [1.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert own["left"][72] == {"stage": None, "first_deg": None}
    run = workflow.plan_for(workflow._numbered(steps), "left", own=own)
    lp = run["plan"][0]["left"]["steps"]
    assert [(x["type"], x.get("_own")) for x in lp] == [("pole_hand", 71), ("program", None), ("pole", None), ("program", None)]
    assert lp[0]["mm"] == 50.0 and lp[0]["values"] == grip


def test_the_viewer_finds_the_programs_on_the_controllers_and_replays_one_with_its_own_pole_and_hand(lite):
    """The Trajectory Viewer (the owner, 21 Oct: "The tab can search the uploaded tasks"): the programs on both arms'
    controllers, each with Lite's record of what it sent there; one sent with its pole and hand replays like a file,
    staged with its own, from the very file sent (its path is kept with the record)."""
    up = call(lite, "POST", "/api/robots/left/programs/upload", {"program_id": 43, "name": "L_viewer.txt", "content": content(2.0),
                                                                "pole_mm": 60, "hand": "CLEANING", "confirmed": True})
    assert wait(lite, up)["state"] == "done"
    rec = lite["app"].robot.arm("left").programs[43]
    assert Path(rec["path"]).is_file() and rec["path"].endswith("_L_viewer.txt")
    listed = call(lite, "GET", "/api/library/programs")
    assert set(listed["arms"]) == {"left", "right"} and listed["backend"] == "stand-in"
    row = next(p for p in listed["arms"]["left"]["programs"] if p["id"] == 43)
    assert row["viewable"] and row["why"] is None and row["sent"]
    assert row["stage"]["text"] == "pole 60 mm, hand cleaning pose" and row["lines"] == 120
    tl = call(lite, "GET", "/api/library/programs/left/43/timeline")
    assert tl["program"] == 43 and tl["lines"] == 120 and tl["stage"]["pole_mm"] == 60 and tl["traj_t0"] > 0     # staged first
    assert call(lite, "GET", "/api/library/programs/left/43/track")["lines"] == 120
    assert {"L1", "L2", "L3", "L4"} <= {c["id"] for c in call(lite, "GET", "/api/library/programs/left/43/checks")["checks"]}
    # the collision check takes it as a slot's file, on its own arm only
    j = call(lite, "POST", "/api/library/collisions", {"left": "p_left_43", "phase": "lid_closed"})
    assert wait(lite, j)["state"] == "done"
    other = call(lite, "POST", "/api/library/collisions", {"right": "p_left_43"})
    assert other["code"] == "arm" and "never shown on the RIGHT arm" in other["error"]


def test_the_viewer_shows_only_what_lite_knows_of_a_program(lite):
    """Sent without its pole and hand: listed and run (asked for them then), never shown. Its file gone: run, not shown,
    until a file of the same bytes is on this computer again. A slot that holds nothing: refused."""
    up = call(lite, "POST", "/api/robots/left/programs/upload", {"program_id": 44, "name": "L_bare.txt", "content": content(3.0),
                                                                "confirmed": True})
    assert wait(lite, up)["state"] == "done"
    row = lambda n: next(p for p in call(lite, "GET", "/api/library/programs")["arms"]["left"]["programs"] if p["id"] == n)  # noqa: E731
    assert not row(44)["viewable"] and "without its pole and hand" in row(44)["why"]
    refused = call(lite, "GET", "/api/library/programs/left/44/timeline")
    assert refused["code"] == "stage" and "cannot be shown" in refused["error"]

    up = call(lite, "POST", "/api/robots/left/programs/upload", {"program_id": 45, "name": "L_moved.txt", "content": content(4.0),
                                                                "pole_mm": 90, "hand": "GRIP", "confirmed": True})
    assert wait(lite, up)["state"] == "done"
    sent = Path(lite["app"].robot.arm("left").programs[45]["path"])
    sent.rename(sent.with_suffix(".gone"))
    assert not row(45)["viewable"] and "no longer on this computer" in row(45)["why"] and row(45)["stage"]["pole_mm"] == 90
    assert "cannot be shown" in call(lite, "GET", "/api/library/programs/left/45/timeline")["error"]
    call(lite, "POST", "/api/library", {"name": "L_again.txt", "content": content(4.0), "arm": "left", "pole_mm": 10, "hand": "OPEN"})
    assert row(45)["viewable"]                     # the same bytes again: shown, still with its own pole and hand
    assert call(lite, "GET", "/api/library/programs/left/45/timeline")["stage"]["pole_mm"] == 90

    assert "holds no program 99" in call(lite, "GET", "/api/library/programs/left/99/timeline")["error"]


def test_visual_sim_shows_a_program_lite_sent_with_its_own_pole_and_hand(lite):
    """Nothing is sent in Visual sim, but a program is shown as its run from the slot runs: its own pole and hand first."""
    pid = "stage_vis"
    call(lite, "PUT", f"/api/projects/{pid}", {"id": pid, "name": "Stage visual", "createdAt": "2026-10-21T10:00:00.000Z",
                                               "updatedAt": "2026-10-21T10:00:00.000Z", "archived": False, "state": {"currentTask": 0}})
    txt = content(5.0)
    r = call(lite, "POST", f"/api/projects/{pid}/units/L-R01/recordings", {"source": "upload", "name": "L_vis.txt", "content": txt,
                                                                          "pole_mm": 70, "hand": "GRIP"})
    assert "error" not in r, r
    up = call(lite, "POST", "/api/robots/left/programs/upload", {"program_id": 46, "name": "L_vis.txt", "content": txt,
                                                                "pole_mm": 70, "hand": "GRIP", "confirmed": True})
    assert wait(lite, up)["state"] == "done"
    shown = call(lite, "POST", "/api/robots/left/simulate-move", {"program": 46, "pid": pid})
    assert "its own pole and hand first: pole 70 mm, hand grip" in shown["what"], shown.get("what") or shown
    assert shown["stage"]["pole_mm"] == 70


def test_a_program_the_stand_in_holds_is_deleted_only_when_asked(lite):
    """Delete from a controller (the owner, 21 Oct): the stand-in's programs are the ones Lite sent it; the question is
    asked first, every time; it has no global points or tracks to delete."""
    up = call(lite, "POST", "/api/robots/left/programs/upload", {"program_id": 47, "name": "L_del.txt", "content": content(6.0),
                                                                "pole_mm": 40, "hand": "GRIP", "confirmed": True})
    assert wait(lite, up)["state"] == "done"
    D = "/api/robots/left/stored/delete"
    assert call(lite, "POST", D, {"kind": "program", "program_id": 47})["code"] == "confirm_delete"
    assert 47 in lite["app"].robot.arm("left").programs
    out = call(lite, "POST", D, {"kind": "program", "program_id": 47, "confirmed": True})
    assert out["said"] == "Program 47 (L_del) is deleted from the LEFT stand-in robot."
    assert 47 not in lite["app"].robot.arm("left").programs
    assert 47 not in {p["id"] for p in call(lite, "GET", "/api/robots/left/programs")["programs"]}
    assert call(lite, "POST", D, {"kind": "point", "name": "STP_HOME", "confirmed": True})["code"] == "connect"
