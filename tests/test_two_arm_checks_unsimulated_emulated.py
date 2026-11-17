"""An S2 run not simulated as it is now (the operator's "run without simulating"), on ALIX's emulated arms: the task's
stretches are still read from the two-arm store (their verdict is a pure function of the plan: kept by fingerprint by
any simulation or start that checked them), but its two MoveJs to its start, whose check depends on where the arms
stand, go one after the other, said plainly. Nothing is checked in the run (the user, 16 Nov). Its own Lite: the arms
where no run has moved them, so both MoveJs move. test_two_arm_checks_ahead_emulated.py: the simulated run."""
import json

import pytest

from mech_emulated import P, call, start, wait

ANSWERS = ["lid_closed", "lid_lifted", "lid_open"]


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    ctx, proc = start(tmp_path_factory, "lite-two-arm-unsimulated", [])
    yield ctx
    proc.terminate()
    proc.wait(10)


def test_a_run_not_simulated_as_it_is_reads_the_stretches_but_moves_the_movejs_one_after_the_other(server):
    lid = {"type": "mechanism", "task": "lid_open"}
    for side in ("left", "right"):
        call(server, "PUT", f"{P}/workflow/{side}", {"steps": [lid]})
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))   # its stretches checked (kept);
    assert j["state"] == "done" and all(r["ok"] for r in j["result"]["ahead"]["stretches"]), j   # not played to its end
    server["script"].write_text(json.dumps(ANSWERS))
    done = wait(server, call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "unsimulated_ok": True}),
                limit=500)
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-8:])
    said = " | ".join(e["text"] for e in done["events"])
    assert "Run on the robot without a simulation of it" in said
    assert "(2 of 2 checked clear before the run)" in said and said.count(", checked before the run)") == 2
    moves = [t for t in said.split(" | ") if t.startswith("MoveJs to Open the lid's start")]
    assert moves == ["MoveJs to Open the lid's start one after the other: their paths were not checked before the run "
                     "(the S2 simulation checks a sequence's, Lite's start a task's on its own)"], moves
