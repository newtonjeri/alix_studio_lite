"""S2's simulation checks a closing's contact forces on the run's clock (contact_sim; the owner, 29 Oct) and Real refuses
a closing that presses over 40 N for 0.2 s: on ALIX's emulated arms, its own Lite with the contact check on, Open the
seat then Close the seat (the owner's plan of 29 Oct 13:25: the left's fingers on the lid through the seat's opening,
about 100 N for 4 s in the model, 57 to 81 N at the lab with the plan before it). Nothing is streamed when Real refuses
it."""
import pytest

from mech_emulated import P, call, hardware, start, wait


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    ctx, proc = start(tmp_path_factory, "lite-s2-contact", ["seat_lifted", "seat_open", "seat_closed"], contact=True)
    yield ctx
    proc.terminate()
    proc.wait(10)


def test_s2_says_a_closings_contact_forces_and_real_refuses_one_that_presses(server):
    so, sc = {"type": "mechanism", "task": "seat_open"}, {"type": "mechanism", "task": "seat_close"}
    for side in ("left", "right"):
        call(server, "PUT", f"{P}/workflow/{side}", {"steps": [so, sc, {"type": "wait", "seconds": 0.5}]})
    j = wait(server, call(server, "POST", f"{P}/workflow/simulate", {"mode": "both"}), limit=400)
    assert j["state"] == "done", j
    sim = j["result"]
    assert [c["task"] for c in sim["contact"]] == ["seat_close"]          # the closing only (Open the seat is not checked)
    c = sim["contact"][0]
    row = next(r for r in sim["stages"] if r.get("task") == "seat_close")
    assert row["t0"] <= c["trace"][0]["t"] and c["trace"][-1]["t"] <= row.get("t1_plan", row["t1"]) + 1e-6   # on the run's clock
    found = [f for f in sim["findings"] if f.get("kind") == "contact"]
    assert len(found) == 1 and found[0]["level"] == c["level"] and found[0]["text"].startswith("Close the seat: the hands press up to")
    assert "Close the seat" in " ".join(f["text"] for f in sim["findings"] if "left out of this collision check" in f["text"])
    assert call(server, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})["completed"]
    n0 = len(hardware(server))
    out = call(server, "POST", f"{P}/workflow/run", {"mode": "both", "confirmed": True, "warnings_seen": sim["hash"]})
    if c["level"] == "stop":                                              # the plan of 29 Oct 13:25 presses
        assert out.get("code") == "contact" and out["error"].startswith("Not run: Close the seat: the hands press up to"), out
        assert not [r for r in hardware(server)[n0:] if r.get("verb") == "canfd stream"]         # nothing streamed
    else:                                                                 # a plan that presses little runs
        assert out.get("code") != "contact", out
