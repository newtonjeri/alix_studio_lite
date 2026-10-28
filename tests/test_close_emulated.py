"""The closings in Real on ALIX's emulated arms (their own Lite, beside the lid's and the seat's): the owner's compound
seat_close (26 Oct) and lid_close (28 Oct) streamed over each arm's CanFD pass-through at 5 ms: the right arm's look from
inspection pose 1 (the part up: scripted here, LITE_CHECK_SCRIPT, a test harness only), the left's hook and pull as one
stream with its retreat, the right's release with its look, the end check (the part down) and the right's rest."""
import pytest

from mech_emulated import call, hardware, start, wait


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    # seat_close: the seat up (it runs), then down; lid_close: the lid up, then down
    ctx, proc = start(tmp_path_factory, "lite-close", ["seat_open", "seat_closed", "lid_open", "lid_closed"])
    yield ctx
    proc.terminate()
    proc.wait(10)


@pytest.mark.parametrize("task,up,down", [("seat_close", "seat_open", "seat_closed"), ("lid_close", "lid_open", "lid_closed")])
def test_real_closing_streams_every_move_with_its_two_looks(server, task, up, down):
    from alix_studio_lite.server import mechanism

    plan = mechanism.Plan(task)
    t0 = len(hardware(server))
    job = call(server, "POST", f"/api/mechanism/{task}/run", {"confirmed": True})
    done = wait(server, job)
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-8:])
    res = done["result"]
    assert [(p["plan"], p["outcome"], p["relifts"]) for p in res["plans"]] == [(task, "done", 0)]
    assert [(c["check"], c["verdict"]) for c in res["checks"]] == [(f"verify_{up}", "PASS"), (f"verify_{down}", "PASS")]
    hw = hardware(server)[t0:]
    streams = [r for r in hw if r.get("verb") == "canfd stream"]
    assert streams and all(r["ok"] for r in streams), [r.get("text") for r in streams if not r["ok"]]
    labels = [r["args"]["label"] for r in streams]           # each arm's streams in the plan's order (the two arms' first stretch
    assert [lb for lb in labels if lb.startswith("LEFT")] == ["LEFT arm move_to_pre_start + move_to_start", "LEFT arm execute_path + move_to_rest"]
    right = [lb for lb in labels if lb.startswith("RIGHT")]                 # each arm's streams as the owner's plan has them (the two
    want = [f"{j.side.upper()} arm {j.stage}" for j, _ in plan.streams()]   # arms' first stretch runs at once, so the logging
    assert [lb.split(" (")[0] for lb in right] == [w for w in want if w.startswith("RIGHT")]   # order across the arms varies)
    assert [lb.split(" (")[0] for lb in labels if lb.startswith("LEFT")] == [w for w in want if w.startswith("LEFT")]
    assert labels[0].startswith("RIGHT arm move_to_inspect") and labels[-1] == want[-1]
    poles = [(r["side"], r["args"]["mm"]) for r in hw if r.get("verb") == "pole and hand"]
    assert sorted(poles) == [("left", 149.6), ("right", 149.6)], poles    # both poles set at its start (28 Oct: the lab ran them at 200)
    assert sum(len(lb.split(" (")[0].split(" + ")) for lb in labels) == sum(1 for e in plan.entries if e.kind == "arm")   # every move named
    assert not [r for r in hw if r.get("verb") == "hand move"]                              # no hand entry: the hands stay as they are
    said = " | ".join(e["text"] for e in done["events"])
    assert "cannot judge" not in said and f"CHECK verify_{down}" in said
    assert "not sent, the" not in said                                                       # a task on its own: its rest streamed
