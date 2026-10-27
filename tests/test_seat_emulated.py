"""The seat in Real on ALIX's emulated arms: ALIX's seat_open plan streamed over each arm's CanFD pass-through at 5 ms,
its two start checks from inspection pose 4 (lid open, then seat closed, judged on the same frames in Real; scripted
here: LITE_CHECK_SCRIPT, a test harness only), the lift, the right's hold, the left's release and retract, the right's
opening and its end look from pose 1. Its own Lite, so it runs beside the lid's (tests/test_mechanism_emulated.py)."""
import json
import time
from pathlib import Path

import pytest

from mech_emulated import call, hardware, refused, start, wait


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    # seat_open: the lid open and the seat closed (it runs); the lift seen; its end look sees the seat open
    ctx, proc = start(tmp_path_factory, "lite-seat", ["lid_open", "seat_closed", "seat_lifted", "seat_open"])
    yield ctx
    proc.terminate()
    proc.wait(10)


def test_real_seat_open_streams_every_move_and_ends_with_its_own_look(server):
    from alix_studio_lite.server import mechanism

    plan = mechanism.Plan("seat_open")
    said = refused(server, "POST", "/api/mechanism/seat_open/run", {})
    assert said.get("code") == "confirm"
    job = call(server, "POST", "/api/mechanism/seat_open/run", {"confirmed": True})
    done = wait(server, job)
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-8:])
    res = done["result"]
    assert [(p["plan"], p["outcome"], p["relifts"]) for p in res["plans"]] == [("seat_open", "done", 0)]
    assert [(c["check"], c["verdict"]) for c in res["checks"]] == [("verify_lid_open", "PASS"), ("verify_seat_closed", "PASS"),
                                                                   ("verify_seat_lifted", "PASS"), ("verify_seat_open", "PASS")]
    hw = hardware(server)
    streams = [r for r in hw if r.get("verb") == "canfd stream"]
    assert streams and all(r["ok"] for r in streams), [r.get("text") for r in streams if not r["ok"]]
    labels = [r["args"]["label"] for r in streams]
    assert labels[0] == "RIGHT arm move_to_inspect" and labels[-1] == "RIGHT arm move_to_rest"
    assert "LEFT arm move_to_pre_start + move_to_start + execute_path" in labels
    assert "LEFT arm move_to_retract + move_to_retract_2 + move_to_rest" in labels         # 26 Oct trims: one stream, no fist between
    assert sum(len(r["args"]["label"].split(" + ")) for r in streams) == sum(1 for e in plan.entries if e.kind == "arm")   # every move named
    settles = [r for r in hw if r.get("verb") == "canfd settle"]                           # the ends waited for: on the feed's frames
    assert settles and all(r["args"]["settle"]["by"] == "feed" and r["args"]["settle"]["at_rest"] for r in settles)
    when = lambda verb, label: next(r["t"] for r in hw if r.get("verb") == verb and r["args"]["label"] == label)   # noqa: E731
    # ... and only when something needs the arm still (27 Oct): the left's retract ends before the right pushes, its end
    # is waited for at the check after, not before the push
    assert when("canfd settle", "LEFT arm move_to_retract + move_to_retract_2 + move_to_rest") > when("canfd stream", "RIGHT arm execute_path")
    assert not any("bridged" in (r.get("text") or "") for r in streams)                    # the emulated arm does not creep
    said = " | ".join(e["text"] for e in done["events"])
    assert "LEFT hand free" in said and "cannot judge" not in said                          # the release gate passed
    assert "not sent, the" not in said                                                      # a task on its own: its final releases sent
    hands = [r for r in hw if r.get("verb") == "hand move"]
    assert [r["side"] for r in hands].count("left") >= 3 and [r["side"] for r in hands].count("right") >= 3
    feeds = [r for r in hw if r.get("verb") == "canfd feed" and str(r.get("text") or "").startswith("off")]
    assert sorted(r["side"] for r in feeds) == ["left", "right"] and abs(feeds[0]["t"] - feeds[1]["t"]) < 1.0   # both off at once
    caps = sorted(p.name for p in (Path(server["data"]) / "logs" / time.strftime("%Y-%m-%d") / "canfd").glob("*.frames.jsonl"))
    assert sorted(c.split("_", 2)[2] for c in caps) == ["LEFT_seat_open.frames.jsonl", "RIGHT_seat_open.frames.jsonl"], caps
    # the status log followed the arms through the run (the readings' own connection)
    rows = [json.loads(l) for l in (Path(server["data"]) / "logs" / time.strftime("%Y-%m-%d") / "status.jsonl").read_text().splitlines() if l.strip()]
    during = [r for r in rows if streams[0]["t"] <= r["t"] <= streams[-1]["t"]]
    assert during and all((9 if r["arms"][s]["age_s"] is None else r["arms"][s]["age_s"]) < 1.0 for r in during for s in ("left", "right"))
