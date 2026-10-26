"""The lid in Real on ALIX's emulated arms: ALIX's lid_open plan streamed over each arm's CanFD pass-through at 5 ms
(alix_dispatch's lane: the 5 ms capture, its watch), the pole and the hand, a re-lift from the pre-planned rung and the
bridged move after it, and STOP in the middle of a stream. The camera's answers are scripted (LITE_CHECK_SCRIPT, a test
harness only): what the gap check decides is tests/test_gap_check.py's. S2's runs of a lid task: test_s2_mechanism_emulated.py
(its own Lite, so the two files run side by side under pytest -n)."""
import json
import time
from pathlib import Path

import pytest

from mech_emulated import call, hardware, refused, start, wait


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    # lid_open: closed (it runs); the lift missed (re-lift 1 from rung 1); lifted (the seat under it not asked, 26 Oct);
    # its final check sees the lid open. Then the STOP test's lid_open: closed, then STOP before its next question.
    ctx, proc = start(tmp_path_factory, "lite-mech", ["lid_closed", "lid_closed", "lid_lifted", "lid_open", "lid_closed"])
    yield ctx
    proc.terminate()
    proc.wait(10)


def test_the_catalogue_lists_the_two_openings(server):
    cat = call(server, "GET", "/api/mechanism")
    assert [t["name"] for t in cat["tasks"]] == ["lid_open", "seat_open"]        # the closings withdrawn (the owner, 26 Oct)
    assert cat["standoff_mm"] == 250 and cat["rows"]["L-T01"] == "lid_open"


def test_real_lid_open_streams_each_move_and_re_lifts_from_rung_1(server):
    said = refused(server, "POST", "/api/mechanism/lid_open/run", {})
    assert said.get("code") == "confirm"                         # Real asks first, as every motion does
    job = call(server, "POST", "/api/mechanism/lid_open/run", {"confirmed": True})
    done = wait(server, job)
    from alix_studio_lite.server import mechanism

    if mechanism.Plan("lid_open").rungs_note():          # the vendored re-lifts planned for another version of the plan
        assert done["state"] == "failed" and "planned for another version of Open the lid" in done["error"], done.get("error")
        labels = [r["args"]["label"] for r in hardware(server) if r.get("verb") == "canfd stream"]
        assert labels and not any("re-lift" in x for x in labels)              # stopped before any of it moved
        server["script"].write_text(json.dumps(["lid_closed"]))                # the STOP test's answer, from the first
        return
    assert done["state"] == "done", (done.get("error"), [e["text"] for e in done.get("events", [])][-8:])
    res = done["result"]
    assert [(p["plan"], p["outcome"], p["relifts"]) for p in res["plans"]] == [("lid_open", "done", 1)]
    assert [c["verdict"] for c in res["checks"]] == ["PASS", "NO", "PASS", "PASS"]       # no second question (26 Oct)
    assert res["checks"][-1]["check"] == "verify_lid_open" and not res["checks"][-1].get("final")   # the plan's own end look (26 Oct)
    said = " | ".join(e["text"] for e in done["events"])
    assert "Open the seat's way in" not in said and "at where Open the lid ended" not in said   # no look of Lite's own (26 Oct)
    streams = [r for r in hardware(server) if r.get("verb") == "canfd stream"]
    assert streams and all(r["ok"] for r in streams), [r.get("text") for r in streams if not r["ok"]]
    labels = [r["args"]["label"] for r in streams]
    assert any("re-lift 1, pre-planned" in x for x in labels)
    # the left's next move after the re-lift (since 24 Oct 21:39 its way out, after the right's push) starts where the
    # ORIGINAL lift ended: 148.8 degrees from rung 1's end, bridged by ALIX's MoveJ
    left = [r for r in streams if r["args"]["label"].startswith("LEFT")]
    after = left[max(i for i, r in enumerate(left) if "re-lift 1" in r["args"]["label"]) + 1]
    assert "bridged" in after["text"], after["args"]["label"]
    assert all(r["args"]["period_ms"] == 5.0 and r["args"]["rate_gate"] == "per_joint" for r in streams)
    assert all((r["args"]["settle"] or {}).get("by") == "feed" and r["args"]["settle"]["at_rest"] for r in streams)   # its end from the feed (26 Oct)
    assert all(", end " in r["text"] and r["args"]["settle"]["waited_s"] < 0.5 for r in streams), [r["text"] for r in streams]
    # each arm's 5 ms feed on once for the run (24 Oct: on and off around every move, 1.1 s each), every move watched on
    # it over its own stretch, and off at the end; an arm's back-to-back moves are one stream
    caps = sorted(p.name for p in (Path(server["data"]) / "logs" / time.strftime("%Y-%m-%d") / "canfd").glob("*.frames.jsonl"))
    assert sorted(c.split("_", 2)[2] for c in caps) == ["LEFT_lid_open.frames.jsonl", "RIGHT_lid_open.frames.jsonl"], caps
    assert {r["args"]["capture"] for r in streams} == set(caps) and all(len(r["args"]["t_mono"]) == 2 for r in streams)
    feeds = [r for r in hardware(server) if r.get("verb") == "canfd feed"]
    assert sorted(r["side"] for r in feeds if "kept for the run" in (r["text"] or "")) == ["left", "right"]
    assert sorted(r["side"] for r in feeds if (r["text"] or "").startswith("off")) == ["left", "right"] and all(r["ok"] for r in feeds)
    assert "LEFT arm move_to_pre_start + move_to_start + execute_path" in labels
    # ALIX's emulator reports a 100 % global ratio, so each stream sets 50 and puts 100 back (the robot's controllers
    # report none: 50 %, held across the run's moves; tests/test_robot_real.py)
    assert not any(r["args"]["ratio_held"] for r in streams)
    assert labels.count("RIGHT arm move_to_inspect") == 2 and labels[-1] == "RIGHT arm move_to_rest"   # the plan's two looks; its own end check rides rest
    hands = [r for r in hardware(server) if r.get("verb") == "hand move" and isinstance(r["args"]["speed_set"], list)]
    assert hands and all(len(r["args"]["speed_set"]) == 6 for r in hands)   # the plan's hand moves: ALIX's executive's speeds


def test_stop_in_the_middle_of_a_stream_stops_sending(server):
    server["script"].write_text(json.dumps(["lid_closed"]))                # its first answer: the lid closed, the lift follows
    before = len(hardware(server))
    job = call(server, "POST", "/api/mechanism/lid_open/run", {"confirmed": True})
    running = wait(server, job, until=lambda j: "move_to_inspect" in (j.get("stage") or "") and "move_to_inspect_pole" not in (j.get("stage") or ""))
    assert running["state"] == "running", running
    deadline = time.monotonic() + 30                             # ALIX's "CANFD: N lines ..." comes as the sending starts
    while not any(str(r.get("text") or "").startswith("CANFD:") for r in hardware(server)[before:]):
        assert time.monotonic() < deadline, "the stream never started"
        time.sleep(0.02)
    time.sleep(0.3)                                              # some of its 200 lines (1 s) sent
    call(server, "POST", "/api/robots/stop-both", {})
    done = wait(server, job, limit=60)
    assert done["state"] in ("cancelled", "failed"), done
    last = [r for r in hardware(server) if r.get("verb") == "canfd stream"][-1]
    sent = int(last["text"].split("/")[0])
    assert not last["ok"] and 0 < sent < last["args"]["lines"], last  # the lines it sent, not "0/200" (23 Oct, the lab)
    for side in ("left", "right"):                               # STOP holds the arms until Recover, as everywhere
        call(server, "POST", f"/api/robots/{side}/recover", {})
