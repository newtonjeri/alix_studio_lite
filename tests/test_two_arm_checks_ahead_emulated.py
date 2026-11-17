"""The two arms at once inside an S2 lid task, checked before the run and never in it (the user, 16 Nov: "we have already
verified the collisions ... remove that during the actual run"), on ALIX's emulated arms. The simulation checks the
task's stretches as the run has them (kept by their fingerprint in the two-arm store) and its two MoveJs to its start
from where it has the arms (kept with the simulation); the run reads both and says so. A run not simulated as it is
now: test_two_arm_checks_unsimulated_emulated.py (a Lite of its own, its arms where no run has moved them). The
camera's answers are scripted (LITE_CHECK_SCRIPT, a test harness only)."""
import json
import os
from pathlib import Path

import pytest

from mech_emulated import P, call, start, wait

ANSWERS = ["lid_closed", "lid_lifted", "lid_open"]


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    ctx, proc = start(tmp_path_factory, "lite-two-arm-ahead", [])
    yield ctx
    proc.terminate()
    proc.wait(10)


def put(server, left, right):
    call(server, "PUT", f"{P}/workflow/left", {"steps": left})
    call(server, "PUT", f"{P}/workflow/right", {"steps": right})


def run(server, answers, sim=None) -> str:
    server["script"].write_text(json.dumps(answers))
    body = {"mode": "both", "confirmed": True}
    if sim is not None:
        assert call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})["completed"]
        body["warnings_seen"] = sim["hash"]
    else:
        body["unsimulated_ok"] = True
    done = wait(server, call(server, "POST", f"{P}/workflow/run", body), limit=500)
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-8:])
    return " | ".join(e["text"] for e in done["events"])


def test_the_simulation_checks_the_two_arms_at_once_and_the_run_only_reads_it(server):
    lid = {"type": "mechanism", "task": "lid_open"}
    put(server, [lid], [lid])
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    assert j["state"] == "done", j
    sim = j["result"]
    ahead = sim["ahead"]
    # the task's two stretches, checked here and kept by fingerprint in the store this test process shares with its server
    assert [(r["plan"], r["stretch"], r["ok"]) for r in ahead["stretches"]] == [
        ("lid_open", "LEFT move_to_half_pole_length + hand_grasp + execute_path || RIGHT move_to_inspect_pole_length + hand_grasp", True),
        ("lid_open", "LEFT move_to_rest || RIGHT hand_grasp + move_to_inspect", True)]
    assert len(list(Path(os.environ["LITE_TWO_ARM_STORE"]).glob("stretch_*.json"))) >= 2
    # its MoveJs to its start from where the simulation has the arms: stage 0, both arms, 49 pairs of moments
    (a,) = ahead["approaches"]
    assert (a["stage"], a["plan"], a["at"], sorted(a["q0"]), sorted(a["moves"])) == (0, "lid_open", "start", ["left", "right"], ["left", "right"])
    notes = " | ".join(f["text"] for f in sim["findings"])
    assert "their clearance checked here, before the run (never within 50 mm" in notes
    said = run(server, ANSWERS, sim)
    assert "Open the lid: 2 stretch(es) the two arms can run at once" in said and "(2 of 2 checked clear before the run)" in said
    assert said.count(", checked before the run)") == 2 and "one arm after the other" not in said
    # clear in the simulation (both arms from where the emulated arms stand): at once by its verdict, read in the run
    assert a["ok"] and a["why"] == "their paths clear at 49 pairs of moments", a
    moves = [t for t in said.split(" | ") if t.startswith("MoveJs to Open the lid's start")]
    assert moves == ["MoveJs to Open the lid's start both at once: their paths clear at 49 pairs of moments (the simulation's check)"], moves
    assert "not checked before the run" not in said
