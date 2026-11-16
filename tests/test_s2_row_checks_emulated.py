"""S2 with lid and seat rows that keep only some camera checks (their ``checks``; the user, 16 Nov), on ALIX's emulated
arms: no INSPECT, Open the lid keeping its mid-lift and end checks, Open the seat and Close the lid none. The simulation
draws only the kept checks and is shorter by the others; the run asks the camera exactly those (the scripted camera
answers only them: any other question would stop the run as undecided), and what they find decides as before.
The camera's answers are scripted (LITE_CHECK_SCRIPT, a test harness only)."""
import json

import pytest

from mech_emulated import P, call, start, wait

LID_ONLY = ["verify_lid_lifted", "verify_lid_open"]


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    ctx, proc = start(tmp_path_factory, "lite-s2-row-checks", [])
    yield ctx
    proc.terminate()
    proc.wait(10)


def rows(keep: bool, tasks=("lid_open", "seat_open", "lid_close")) -> dict:
    """Both arms' lists: the tasks in order, a Wait between them on the left; each task's ``checks`` when ``keep``."""
    setting = {"lid_open": LID_ONLY, "seat_open": [], "lid_close": []}
    left, right = [], []
    for k, task in enumerate(tasks):
        row = {"type": "mechanism", "task": task, **({"checks": setting[task]} if keep else {})}
        left += [dict(row)] + ([{"type": "wait", "seconds": 1}] if k + 1 < len(tasks) else [])
        right.append(dict(row))
    return {"left": left, "right": right}


def simulate(server, steps: dict) -> dict:
    for side in ("left", "right"):
        call(server, "PUT", f"{P}/workflow/{side}", {"steps": steps[side]})
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    assert j["state"] == "done", j
    return j["result"]


def run(server, sim: dict, answers: list) -> str:
    assert call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})["completed"]
    server["script"].write_text(json.dumps(answers))
    job = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    done = wait(server, job, limit=500)
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-8:])
    return " | ".join(e["text"] for e in done["events"])


def checks_of(sim: dict) -> list:
    return [a["expect"] for a in sim["acts"]["right"] if a["kind"] == "check"]


def test_only_the_kept_checks_are_simulated_and_asked(server):
    every = simulate(server, rows(keep=False))
    assert checks_of(every) == ["lid_closed", "lid_lifted", "lid_open", "seat_lifted", "seat_open", "lid_closed"]
    sim = simulate(server, rows(keep=True))
    assert checks_of(sim) == ["lid_lifted", "lid_open"]                     # Open the lid's mid-lift and end checks only
    notes = " | ".join(f["text"] for f in sim["findings"])
    assert "Open the lid: verify_lid_closed not asked: the sequence row keeps verify_lid_lifted, verify_lid_open" in notes
    assert "Open the seat: verify_seat_lifted, verify_seat_open not asked: the sequence row keeps no camera check" in notes
    assert "Close the lid: verify_lid_closed not asked: the sequence row keeps no camera check" in notes
    assert sim["stages"][-1]["t1"] < every["stages"][-1]["t1"] - 3 * 0.5   # the camera's moments and its settle not spent
    said = run(server, sim, ["lid_lifted", "lid_open"])
    assert "CHECK verify_lid_lifted" in said and "CHECK verify_lid_open" in said
    assert "verify_lid_closed: not asked, the sequence row keeps verify_lid_lifted, verify_lid_open (the user, 16 Nov)" in said
    assert "verify_seat_lifted: not asked, the sequence row keeps no camera check" in said
    assert "CHECK verify_seat" not in said and "CHECK verify_lid_closed" not in said
    assert "Open the seat: Open the seat done." in said and "Close the lid: Close the lid done." in said
    assert "SEAT WITH THE LID" in said                                      # the end check's own look at the seat stays


def test_the_seat_that_came_up_with_the_lid_still_decides(server):
    sim = simulate(server, rows(keep=True, tasks=("lid_open", "seat_open")))
    said = run(server, sim, ["lid_lifted", "lid_open", {"also": "seat_lifted"}])
    assert "the seat is up (it came up with the lid)" in said
    assert "Open the seat is not run" in said and "CHECK verify_seat" not in said
