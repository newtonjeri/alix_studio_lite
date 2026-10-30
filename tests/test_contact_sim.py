"""The lid and seat closings' contact forces (server/contact_sim.py; the owner, 29 Oct: a contact that makes sense is a few
newtons, and the simulation must say so before the robot moves). The plan the lab ran on 29 Oct at 15:17:57 (kept in
tests/data/contact/lid_close_20261029 with its config) pressed 50 to 100 N in three places: the left glove on the lid's
edge at its start, the left's pull, and the right hand lifting the lid on its way up after a release that left it under
the lid. The model finds all three and refuses the plan; the hold, the lid on the back of the right hand, stays a few
newtons. An edited config is previewed by IK before it is re-planned (server/contact_preview.py)."""
from pathlib import Path

import numpy as np
import pytest
import yaml

from alix_studio_lite import vendor_paths
from alix_studio_lite.server import alix_sync, contact_preview, contact_sim, mechanism
from alix_studio_lite.server.jobs import UserError

DATA = Path(__file__).with_name("data") / "contact" / "lid_close_20261029"


@pytest.fixture(scope="module")
def lab():
    return contact_sim.check_plan("lid_close", back_mm=0.0, plan_path=DATA / "lid_close_toppra.json", cfg_path=DATA / "lid_close.yaml")


def span(out, side, stage, nth=0):
    """(start, end) of the plan's ``nth`` entry ``stage`` of ``side`` on its own clock."""
    t0, found = 0.0, []
    for st in out["stages"]:
        if st["side"] == side and st["stage"] == stage:
            found.append((t0, st["t1"]))
        t0 = st["t1"]
    return found[nth]


def most(out, side, t0, t1) -> float:
    return max([(r.get(side) or {}).get("n", 0.0) for r in out["trace"] if t0 - 1e-9 <= r["t"] <= t1 + 1e-9] or [0.0])


def test_the_29_oct_lid_closing_presses_where_the_lab_measured_it(lab):
    assert lab["level"] == "stop" and lab["wall"] and lab["open_deg"] == pytest.approx(-105.57, abs=0.01)
    a0, a1 = span(lab, "left", "move_to_start")
    p0, p1 = span(lab, "left", "execute_path")
    assert most(lab, "left", a1, p0) >= 40.0                       # A: at its start, while the right goes to its hold (66 N)
    assert most(lab, "left", p0, p1) >= 80.0                       # B: the pull until the lid tips (50 to 90 N)
    r0, r1 = span(lab, "right", "execute_path")
    i0, i1 = span(lab, "right", "move_to_inspect", 1)
    held = lab["held"]                                             # C: the release leaves the hand under the lid,
    assert held and held[0]["side"] == "right" and held[0]["want_deg"] == 0.0 and held[0]["at_deg"] <= -10.0
    assert most(lab, "right", i0, i1) >= 40.0                      # and the way up lifts it (68 to 100 N)
    assert lab["over_stop_s"] >= contact_sim.SUSTAIN_S
    placed = lab["placed"][0]                                      # the lid tipped onto the right hand: the hold
    assert placed["tipped"] and placed["deg"] == -30.0
    hold = [(r.get("right") or {}).get("n", 0.0) for r in lab["trace"] if placed["t"] <= r["t"] <= r0]
    assert 3.0 <= float(np.median(hold)) <= contact_sim.WARN_N     # a few newtons, under the warning (6 to 8 N measured; 14.6 in
                                                                   # the model, the lid's 1.2 kg assumed), the left's way to rest
                                                                   # brushing the lid aside (0.1 s)
    text = contact_sim.words(lab, "Close the lid", lambda t, s: contact_sim.stage_at(lab["stages"], t, s))
    assert "held at" in text and "refuses Real over 40 N" in text


def test_a_closing_run_on_its_own_is_refused_in_real_when_it_presses(lab, monkeypatch):
    import hashlib

    plan = mechanism.cached("lid_close")
    key = ("lid_close", hashlib.sha256(plan.path.read_bytes()).hexdigest())
    monkeypatch.setitem(contact_sim._GATE, key, dict(lab))               # the check's result for this plan, as if it were it
    with pytest.raises(UserError) as e:
        contact_sim.gate("lid_close")
    assert e.value.code == "contact" and "Not run: Close the lid: the hands press up to" in str(e.value)
    monkeypatch.setitem(contact_sim._GATE, key, dict(lab, level="warn"))
    assert contact_sim.gate("lid_close")["level"] == "warn"              # a warning runs
    assert contact_sim.gate("lid_open") is None                          # the openings are not checked (the owner: closings only)


def test_an_edited_config_is_previewed_before_it_is_re_planned(tmp_path):
    cfg = vendor_paths.ALIX / alix_sync.CONFIGS_REL / "lid_close.yaml"
    plan = mechanism.cached("lid_close")
    same, notes = contact_preview.edited("lid_close", cfg)
    assert not notes and all(a.rows == b.rows for a, b in zip(same.entries, plan.entries) if a.kind == "arm")
    data = yaml.safe_load(cfg.read_text())
    data["left"]["cartesian_poses"]["start_pose"][1] += 0.01                # the left's start 10 mm further out
    moved = tmp_path / "lid_close.yaml"
    moved.write_text(yaml.safe_dump(data))
    out, notes = contact_preview.edited("lid_close", moved)
    assert any("LEFT move_to_start" in n for n in notes) and any("LEFT execute_path" in n for n in notes)
    fk = contact_preview._fk_ref("left", "L_glove_frame_3", 0.15)
    k = next(i for i, e in enumerate(plan.entries) if e.side == "left" and e.stage == "move_to_start")
    end = fk(np.asarray(out.entries[k].rows[-1]) / 1000.0)[0]
    mm, deg = alix_sync._err(end, alix_sync._pose_T(data["left"]["cartesian_poses"]["start_pose"]))
    assert mm < 1.0 and deg < 0.5
    p = next(i for i, e in enumerate(plan.entries) if e.side == "left" and e.stage == "execute_path")
    old, new = fk(np.asarray(plan.entries[p].rows[-1]) / 1000.0)[0], fk(np.asarray(out.entries[p].rows[-1]) / 1000.0)[0]
    assert new[1, 3] - old[1, 3] == pytest.approx(0.01, abs=0.002)       # the stroke moves with its start (its points are relative)
    with pytest.raises(ValueError):                                       # a stage added: re-plan it first
        data["stage_sequence"].append({"stage": "move_to_rest", "arm": "left", "pose_name": "rest_pose", "planning_group": "rm75_arm_left"})
        moved.write_text(yaml.safe_dump(data))
        contact_preview.edited("lid_close", moved)


def test_the_owner_accepts_one_plan_file_over_a_refusal(lab, monkeypatch):
    """The owner, 29 Oct: Close the seat of 13:25 runs although it reads over the limit (the Inspire fingers fold when
    pressed from their back); the acceptance is that file's, so a plan made again is checked again (30 Oct 09:43 was
    accepted on its own). Here on the vendored plan's own file, whichever is vendored."""
    import hashlib

    plan = mechanism.cached("seat_close")
    sha = hashlib.sha256(plan.path.read_bytes()).hexdigest()
    monkeypatch.setitem(contact_sim.ACCEPTED, "seat_close", [dict(sha256=sha, made=plan.made, by="the owner", why="a test")])
    assert contact_sim.accepted("seat_close", plan.path)["made"] == plan.made
    key = ("seat_close", sha)
    monkeypatch.setitem(contact_sim._GATE, key, dict(lab, task="seat_close", level="stop"))
    out = contact_sim.gate("seat_close")                                    # a stop, accepted: it runs, said
    assert out["level"] == "warn" and "accepted by the owner" in contact_sim.accepted_words(out)
    assert contact_sim.accept(dict(lab), "lid_close", DATA / "lid_close_toppra.json")["level"] == "stop"   # another file
    monkeypatch.setenv("LITE_CONTACT_ACCEPTED", "0")
    with pytest.raises(UserError):
        contact_sim.gate("seat_close")

