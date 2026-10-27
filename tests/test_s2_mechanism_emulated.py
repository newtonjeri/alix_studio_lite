"""S2's runs of a lid task on ALIX's emulated arms (the plan streamed at 5 ms as in test_mechanism_emulated.py, which has
its own Lite): a lid task put in the sequence, the camera required before anything moves, and the arms left where the
plan ended when a Wait comes next. The camera's answers are scripted (LITE_CHECK_SCRIPT, a test harness only)."""
import json

import pytest

from mech_emulated import P, call, hardware, refused, start, wait


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    ctx, proc = start(tmp_path_factory, "lite-s2-mech", [])          # each run writes the answers it needs
    yield ctx
    proc.terminate()
    proc.wait(10)


def test_s2_runs_a_lid_open_put_in_its_sequence(server):
    """The owner, 18 Oct: a pre-recorded task injected at a point in the sequence: Open the lid on both arms' lists,
    simulated (its stage on the run's clock, the lid open after it, left out of the collision check and said so), then
    run in Real by S2 on the emulated arms."""
    lid = {"type": "mechanism", "task": "lid_open"}
    wait_1 = {"type": "wait", "seconds": 1}
    call(server, "PUT", f"{P}/workflow/left", {"steps": [lid, wait_1]})
    call(server, "PUT", f"{P}/workflow/right", {"steps": [lid]})
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    assert j["state"] == "done", j
    sim = j["result"]
    assert [s["kind"] for s in sim["stages"]] == ["mechanism", "parallel"] and sim["counts"]["mechanisms"] == 1   # then the left's Wait
    kinds = {a["kind"] for side in ("left", "right") for a in sim["acts"][side]}
    assert {"stream", "pole", "hand", "global", "check"} <= kinds
    assert sim["phases"][-1][1] == "lid_open"
    right = sim["acts"]["right"]                 # drawn as it runs: the plan's own end check (lid open) from inspection pose 1
    assert [a["expect"] for a in right if a["kind"] == "check"] == ["lid_lifted", "lid_open"]   # no seat under the lid; no start check in a sequence (27 Oct)
    looks = [a for a in right if a["kind"] == "stream" and str(a.get("file", "")).startswith("mechanism:lid_open_seq:")]   # the sequence's own plan
    joined = sorted((a for a in sim["acts"]["left"] + right if ":joined:" in str(a.get("file"))), key=lambda a: a["t0"])
    assert [a["file"].split(":")[-1] for a in joined] == ["3", "3", "2"]              # the run's streams (27 Oct): the left's
    assert "move_to_pre_start + move_to_start + execute_path as one stream" in joined[0]["text"]   # approach and lift, the right's
    assert "move_to_hold_direct + move_to_pre_start + move_to_start as one stream" in joined[1]["text"]   # way from rest to its
    assert any("stops at its junctions taken out" in a["text"] for a in joined)
    last_check = [a for a in right if a["kind"] == "check"][-1]
    assert looks[-2]["t0"] < last_check["t0"] < looks[-1]["t0"]            # the look to pose 1, the camera, then the rest
    assert not any("seat_close" in str(a.get("file")) or "seat_open" in str(a.get("file")) for a in right)
    assert not any(str(a.get("file", "")).startswith("mechanism:lid_open:") for a in right)      # not the plan with the look
    assert any("left out of this collision check" in f["text"] for f in sim["findings"])
    assert call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})["completed"]
    server["script"].write_text(json.dumps(["lid_lifted", "lid_open"]))          # a sequence: no "lid closed" asked (27 Oct)
    job = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    done = wait(server, job)
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-6:])
    said = " | ".join(e["text"] for e in done["events"])
    assert "Open the lid: Open the lid (in a sequence) done." in said and "CHECK verify_lid_lifted" in said   # the sequence's plan


def test_s2_refuses_a_lid_or_seat_task_with_no_camera_before_anything_moves(server):
    """The owner, 23 Oct: with no camera every check read NOT, and Open the lid's "is it closed?" NOT ended it as done,
    the lid closed (the lab, 14:38). A run with a lid or seat task now needs the wrist camera before anything moves.
    Here the harness's answers are taken away and this computer has no D435."""
    lid = {"type": "mechanism", "task": "lid_open"}
    call(server, "PUT", f"{P}/workflow/left", {"steps": [lid]})
    call(server, "PUT", f"{P}/workflow/right", {"steps": [lid]})
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    assert j["state"] == "done", j
    sim = j["result"]
    assert call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})["completed"]
    before = len(hardware(server))
    away = server["script"].with_suffix(".away")
    server["script"].rename(away)
    try:
        out = refused(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    finally:
        away.rename(server["script"])
    assert out["code"] == "camera" and "wrist camera cannot be read" in out["error"] and "Nothing moved" in out["error"], out
    moved = [r for r in hardware(server)[before:] if r.get("verb") in ("canfd stream", "stage", "move to pose", "pole move",
                                                                      "hand move", "workflow")]
    assert not moved, moved


def test_s2_after_a_lid_task_with_a_wait_next_each_arm_stays_where_its_plan_ended(server):
    """The owner, 24 Oct: after a lid or seat task each arm goes to its next task's start (in place of 22 Oct's pole and
    hand put back as before the tasks). With a Wait next (no row that moves the arm) there is no start to go to: nothing
    is put back, the arms stay where the plan left them, and the stage after starts both arms together, in Simulation
    and on the robot. (A Program next: test_s2_lid_emulated.py.)"""
    lid = {"type": "mechanism", "task": "lid_open"}
    left = [{"type": "pole", "mm": 120}, {"type": "hand", "values": [1000, 1000, 1000, 1000, 0, 1000]}, lid, {"type": "wait", "seconds": 0.5}]
    right = [{"type": "pole", "mm": 150}, {"type": "hand", "values": [0, 0, 0, 0, 150, 1000]}, lid, {"type": "wait", "seconds": 0.5}]
    call(server, "PUT", f"{P}/workflow/left", {"steps": left})
    call(server, "PUT", f"{P}/workflow/right", {"steps": right})
    plan = call(server, "POST", f"{P}/workflow/plan", {"mode": "both"})
    assert [s["kind"] for s in plan["stages"]] == ["parallel", "mechanism", "parallel"] and plan["stages"][1]["title"] == "Open the lid"
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    assert j["state"] == "done", j
    sim = j["result"]
    assert sim["stages"][1]["title"] == "Open the lid" and sim["stages"][1]["t1_plan"] == sim["stages"][1]["t1"]
    nxt = {side: min(a["t0"] for a in sim["acts"][side] if a["stage"] == 2) for side in ("left", "right")}
    assert nxt["left"] == pytest.approx(nxt["right"], abs=1e-6)          # both arms start the next stage together
    assert call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})["completed"]
    server["script"].write_text(json.dumps(["lid_lifted", "lid_open"]))          # a sequence: no "lid closed" asked (27 Oct)
    job = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    done = wait(server, job)
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-6:])
    said = " | ".join(e["text"] for e in done["events"])
    assert "back, as before the tasks" not in said and "at the start of" not in said
    arms = call(server, "GET", "/api/robots/status")["arms"]
    assert abs(arms["left"]["link"]["feedback"]["pole_mm"] - 150) < 3 and abs(arms["right"]["link"]["feedback"]["pole_mm"] - 150) < 3


