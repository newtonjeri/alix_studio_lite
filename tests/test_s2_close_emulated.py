"""S2 with the closings on ALIX's emulated arms (its own Lite): TASK 15's Close the seat then Close the lid after Open the
seat, each on its own plan with the check before the pull not asked (the look move kept); and TASK 4's Close the seat right after
Open the lid, which the sequence's own order makes needless (the seat is down): not run, nothing moves. The camera's
answers are scripted (LITE_CHECK_SCRIPT, a test harness only)."""
import json

import pytest

from mech_emulated import P, call, hardware, start, wait


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    ctx, proc = start(tmp_path_factory, "lite-s2-close", [])            # each run writes the answers it needs
    yield ctx
    proc.terminate()
    proc.wait(10)


def test_s2_closes_the_seat_then_the_lid_after_opening_the_seat(server):
    seat = {"type": "mechanism", "task": "seat_open"}
    sc = {"type": "mechanism", "task": "seat_close"}
    lc = {"type": "mechanism", "task": "lid_close"}
    for side in ("left", "right"):
        call(server, "PUT", f"{P}/workflow/{side}", {"steps": [seat, sc, lc, {"type": "wait", "seconds": 0.5}]})
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    assert j["state"] == "done", j
    sim = j["result"]
    assert [s["kind"] for s in sim["stages"]] == ["mechanism"] * 3 + ["parallel"] and sim["counts"]["mechanisms"] == 3
    assert [s["title"] for s in sim["stages"][:3]] == ["Open the seat", "Close the seat", "Close the lid"]     # none held: each changes the state
    assert sim["phases"][-1][1] == "lid_closed"
    right = sim["acts"]["right"]
    assert [a["expect"] for a in right if a["kind"] == "check"] == ["seat_lifted", "seat_open", "seat_closed", "lid_closed"]   # no check before a pull
    files = {str(a.get("file", "")).split(":")[1] for a in sim["acts"]["left"] + right if str(a.get("file", "")).startswith("mechanism:")}
    assert files == {"seat_open_seq", "seat_close", "lid_close"}           # the closings: their own plans, first check dropped
    for task in ("seat_close", "lid_close"):
        mine = [a for a in right if str(a.get("file", "")).startswith(f"mechanism:{task}:") and a["kind"] == "stream"]
        assert "move_to_inspect" in mine[0]["text"] and "move_to_rest" in mine[-1]["text"]   # the look move first, the rest last (a Wait next)
    assert call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})["completed"]
    server["script"].write_text(json.dumps(["seat_lifted", "seat_open", "seat_closed", "lid_closed"]))
    n0 = len(hardware(server))
    job = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    done = wait(server, job, limit=400)
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-8:])
    said = " | ".join(e["text"] for e in done["events"])
    for words in ("Open the seat: Open the seat (in a sequence) done", "Close the seat: Close the seat done",
                  "Close the lid: Close the lid done", "verify_seat_open: not asked", "verify_lid_open: not asked",
                  "CHECK verify_seat_closed", "CHECK verify_lid_closed"):
        assert words in said, words
    assert "verify_seat_open" not in said.replace("CHECK verify_seat_open: expect seat_open", "") or "not asked" in said
    assert "CHECK verify_lid_open" not in said                                              # the sequence's own order says it
    labels = [r["args"]["label"] for r in hardware(server)[n0:] if r.get("verb") == "canfd stream"]
    assert labels.count("LEFT arm execute_path + move_to_rest") == 2 and labels.count("RIGHT arm execute_path + move_to_inspect") == 2


def test_s2_does_not_run_a_closing_whose_end_state_the_sequence_already_made(server):
    """TASK 4 CLOSE SEAT ONLY after TASK 3, which runs Open the lid alone: the seat is down already. The stage says so, no
    check is asked, nothing moves, in the simulation and on the robot."""
    lid = {"type": "mechanism", "task": "lid_open"}
    sc = {"type": "mechanism", "task": "seat_close"}
    for side in ("left", "right"):
        call(server, "PUT", f"{P}/workflow/{side}", {"steps": [lid, sc, {"type": "wait", "seconds": 0.5}]})
    plan = call(server, "POST", f"{P}/workflow/plan", {"mode": "both"})
    assert [s["kind"] for s in plan["stages"]] == ["mechanism", "mechanism", "parallel"]
    assert plan["stages"][1]["title"] == "Close the seat (not run: the lid is open and the seat down already)"
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    assert j["state"] == "done", j
    sim = j["result"]
    held = [a for side in ("left", "right") for a in sim["acts"][side] if a["stage"] == 1]
    assert len(held) == 1 and "not run" in held[0]["text"] and held[0]["t1"] == held[0]["t0"]
    assert sim["stages"][1]["t1"] == sim["stages"][1]["t0"] and sim["phases"][-1][1] == "lid_open"
    assert not any(str(a.get("file", "")).startswith("mechanism:seat_close") for side in ("left", "right") for a in sim["acts"][side])
    assert call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})["completed"]
    server["script"].write_text(json.dumps(["lid_lifted", "lid_open"]))
    n0 = len(hardware(server))
    job = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    done = wait(server, job, limit=400)
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-8:])
    said = " | ".join(e["text"] for e in done["events"])
    assert "Close the seat: not run, the sequence's own order says the lid is open and the seat down already." in said
    assert "Close the seat done" not in said and "verify_seat" not in said
    labels = [r["args"]["label"] for r in hardware(server)[n0:] if r.get("verb") == "canfd stream"]
    assert labels and not any("execute_path + move_to_rest" in lb for lb in labels)         # no pull of the closing
