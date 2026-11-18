"""The lid and seat tasks' arm moves timed with Ruckig on s (the user, 18 Nov: "all arm motions for the mechanism
tasks, all lid and seat tasks except the hands and pole, should use ruckig on s"): the copies Lite keeps in
Resources/RetimedPlans (server/mechanism_timing.py), what a task reads, what a run streams of them, and what happens
when a plan has no copy. The whole runs on ALIX's emulated arms are the emulated files' (they run the copies)."""
import json
import shutil

import numpy as np
import pytest

from alix_studio_lite.motion.limits import load_limits
from alix_studio_lite.server import app, mechanism, mechanism_timing as mt  # noqa: F401  (app first: the route modules' order)
from alix_studio_lite.server.jobs import UserError

TASKS = list(mechanism.TASKS)


@pytest.fixture()
def planned(monkeypatch):
    monkeypatch.setenv("LITE_MECHANISM_TIMING", "planned")


def limits():
    return {s: load_limits(s).with_auto(jerk_rad_s3=1500.0) for s in ("left", "right")}


def over(r: dict, lim) -> bool:
    return r["speed"] > 1.0 or r["acc"] > 1.0 + lim.acc_tol or r["jerk"] > 1.0


def test_the_repositorys_copies_are_of_its_plans_as_they_are_now():
    """A plan vendored again without its copy made again fails here, before a run is refused at the lab."""
    assert mt.timing() == "ruckig_s" and mt.missing() == {}
    made_with = mt.settings()
    assert made_with == dict(rules=mt.RULES, jerk_rad_s3=1500.0, ugv_force_n=None, ugv_moment_nm=52.9)   # half of what the UGV's weight holds


@pytest.mark.parametrize("name", TASKS)
def test_a_task_reads_its_copy_and_only_the_arm_moves_differ(name, monkeypatch):
    mine = mechanism.Plan(name)
    assert mine.timing == "ruckig_s" and mine.timing_note is None and mine.retimed["source"] == f"{name}_toppra.json"
    assert mine.path.parent == mt.store_dir() and mine.source == mechanism.PLANS / f"{name}_toppra.json"
    view = mine.view()
    assert view["file"] == f"{name}_toppra.json" and view["timing"] == "ruckig_s" and view["timing_note"] is None
    monkeypatch.setenv("LITE_MECHANISM_TIMING", "planned")
    alix = mechanism.Plan(name)
    assert alix.timing == "planned" and alix.timing_note is None and alix.path == alix.source
    assert [(e.stage, e.kind, e.side, e.index) for e in mine.entries] == [(e.stage, e.kind, e.side, e.index) for e in alix.entries]
    assert mine.final_actions == alix.final_actions and mine.checks() == alix.checks() and mine.made == alix.made
    travelled = 0
    for a, b in zip(mine.entries, alix.entries):
        assert a.pre_actions == b.pre_actions
        if a.kind != "arm":                                        # the poles and the hands: as planned, to the last digit
            assert np.array_equal(a.t, b.t) and np.array_equal(a.q, b.q) and not a.timed
            continue
        assert a.timed == "ruckig_s" and not b.timed
        assert a.rows[0] == b.rows[0] and a.rows[-1] == b.rows[-1]                 # the plan's own poses at both ends
        if b.note.startswith("one point"):                         # the planner's single point the arm stands on: left
            assert a.rows == b.rows and a.rows[0] == a.rows[-1]
            continue
        travelled += 1
        assert len(a.rows) >= len(b.rows)                          # never shorter than planned
        step = np.abs(np.diff(np.asarray(a.rows), axis=0)).max(axis=1)
        assert step[0] <= 12 and step[-1] <= 12                    # it leaves and arrives at rest (millidegrees a line)
    assert travelled >= 8


@pytest.mark.parametrize("name", TASKS)
def test_what_a_run_streams_reads_within_the_limits(name, monkeypatch):
    """The streams as a run joins them, read as the checks read (20 Hz band-limited, 5 ms): every one inside the joint
    limits file's speed, acceleration and jerk, and no junction's stop taken out. On the planned timing the junction
    blend passes the path's corner at speed: that is what reads many times the acceleration limit, and why a copy's
    moves keep their stop."""
    lim = limits()
    for start_checks in (True, False):
        for joined, _used in mechanism.Plan(name).streams(start_checks):
            r = mt.readings(joined.rows, lim[joined.side], 1500.0)
            assert joined.timed == "ruckig_s" and joined.blended_s == 0.0
            assert not over(r, lim[joined.side]), (name, joined.stage, r)
            assert r["moment_nm"] <= 0.5 * 52.9, (name, joined.stage, r)            # half of the limit (a quarter of what the UGV's weight holds)
    monkeypatch.setenv("LITE_MECHANISM_TIMING", "planned")
    as_planned = [(j, mt.readings(j.rows, lim[j.side], 1500.0)) for j, _ in mechanism.Plan(name).streams()]
    assert any(j.blended_s > 0 for j, _r in as_planned) and max(r["acc"] for _j, r in as_planned) > 5.0


@pytest.mark.parametrize("name", TASKS)
def test_each_moves_speed_class_is_the_plans(name):
    table = json.loads((mt.store_dir() / f"{name}_toppra.ruckig_s.moves.json").read_text(encoding="utf-8"))
    assert table["made_with"] == mt.settings()
    done = [r for r in table["moves"] if r["retimed"]]
    left = [r for r in table["moves"] if not r["retimed"]]
    assert done and all(r["travel_deg"] == 0 for r in left)        # only a move that goes nowhere is left as planned
    for r in done:
        b, a = r["before"], r["after"]
        assert r["within"] and a["seconds"] >= b["seconds"] - 1e-9, r
        assert a["speed"] <= max(b["speed"], 0.02) * 1.02 and a["acc"] <= 1.0 + 0.02 and a["jerk"] <= 1.0, r
        assert r["path_moved_mm"] <= 0.1, r                        # the hand's path: the plan's


def test_the_retries_are_copies_too():
    for name in TASKS:
        plan = mechanism.Plan(name)
        assert plan.rungs_note() is None, name
        for k in plan.view()["rungs"]:
            src = mechanism.PLANS / "rungs" / f"{name}_toppra.rung{k}.json"
            data = json.loads(mt.copy_of(src).read_text(encoding="utf-8"))
            assert data["retimed"]["source"] == src.name and mt.why_not(mt.copy_of(src), src) is None
            if (data.get("task_flags") or {}).get("replan_block"):
                assert all(e.timed == "ruckig_s" for e in plan.rung_block(k, "left") if e.kind == "arm")
            else:
                rung = plan.task_rung(k)
                assert rung.timing == "ruckig_s" and rung.source == src
    assert mechanism.Plan("lid_open").view()["rungs"] == [1, 2, 3] and mechanism.Plan("seat_close").view()["rungs"] == [1, 2]


def test_a_plan_that_changed_is_not_run_on_its_old_copy(tmp_path, monkeypatch):
    plans = tmp_path / "hardware"
    shutil.copytree(mechanism.PLANS, plans)
    edited = plans / "lid_close_toppra.json"
    data = json.loads(edited.read_text(encoding="utf-8"))
    data["timestamp_human"] = "2026-11-18 20:00:00"                # ALIX made it again
    edited.write_text(json.dumps(data), encoding="utf-8")
    monkeypatch.setattr(mechanism, "PLANS", plans)
    mechanism.cached.cache_clear()
    try:
        plan = mechanism.Plan("lid_close")
        assert plan.timing == "planned" and plan.path == edited and "the plan has changed since" in plan.timing_note
        assert plan.view()["timing_note"] == plan.timing_note
        assert list(mt.missing()) == ["lid_close"] and mechanism.Plan("seat_close").timing == "ruckig_s"
        with pytest.raises(UserError) as said:
            mt.require(["seat_close", "lid_close"], "This run")
        assert said.value.code == "timing" and "Close the lid has no Ruckig on s timing" in str(said.value)
        assert "Nothing moved" in str(said.value) and "LITE_MECHANISM_TIMING=planned" in said.value.hint
        mt.require(["seat_close", "lid_open"])                    # the others: theirs are as they were
        monkeypatch.setenv("LITE_MECHANISM_TIMING", "planned")     # asked for: ALIX's own timing, nothing refused
        mt.require(["lid_close"])
        assert mechanism.Plan("lid_close").timing_note is None
    finally:
        mechanism.cached.cache_clear()


def test_a_copy_is_made_again_for_a_plan_and_its_retries(tmp_path, monkeypatch):
    monkeypatch.setenv("LITE_RETIMED_PLANS", str(tmp_path / "store"))
    mechanism.cached.cache_clear()
    try:
        assert set(mt.missing()) == set(TASKS) and mechanism.Plan("lid_open").timing == "planned"
        said, real = [], mt.missing
        with monkeypatch.context() as only:                        # Lite's start, with one task to make (a few seconds)
            only.setattr(mt, "missing", lambda tasks=None: {k: v for k, v in real(tasks).items() if k == "lid_open"})
            assert mt.make_missing(said.append) == ["lid_open"] and "lid_open timed with Ruckig on s" in said[0]
        assert "lid_open" not in mt.missing() and mechanism.Plan("lid_open").timing == "ruckig_s"
        for f in ("lid_open_toppra.ruckig_s.json", "lid_open_toppra.ruckig_s.g0.txt", "lid_open_toppra.ruckig_s.moves.json",
                  "rungs/lid_open_toppra.ruckig_s.rung2.json", "rungs/lid_open_toppra.rung3.ruckig_s.g0.txt"):
            assert (tmp_path / "store" / f).is_file(), f
        block = mechanism.Plan("lid_open").rung_block(1, "left")           # its first re-lift: the left arm's block, re-timed
        assert block and all(e.timed == "ruckig_s" for e in block if e.kind == "arm")
        copy = tmp_path / "store" / "lid_open_toppra.ruckig_s.json"
        data = json.loads(copy.read_text(encoding="utf-8"))
        data["retimed"]["jerk_rad_s3"] = 300.0                     # made with another jerk limit: not Lite's copy
        copy.write_text(json.dumps(data), encoding="utf-8")
        assert mt.why_not(copy, mechanism.PLANS / "lid_open_toppra.json") == "it was made with other settings or rules"
    finally:
        mechanism.cached.cache_clear()


def test_the_made_copy_is_the_repositorys_to_the_byte(tmp_path):
    """Made again here from the same plan, with the ruckig this Python has: the repository's files (made 18 Nov with
    ruckig 0.9.2; 0.19.4 gives the same bytes)."""
    pytest.importorskip("ruckig")
    repo = mt.store_dir()
    mt.make("seat_open", tmp_path)
    mechanism.cached.cache_clear()
    for f in ("seat_open_toppra.ruckig_s.json", "seat_open_toppra.ruckig_s.g0.txt", "seat_open_toppra.ruckig_s.g1.txt"):
        assert (tmp_path / f).read_bytes() == (repo / f).read_bytes(), f


def test_without_ruckig_the_start_says_so_and_the_run_is_refused(tmp_path, monkeypatch):
    from alix_studio_lite.optimise import ruckig_opt

    monkeypatch.setenv("LITE_RETIMED_PLANS", str(tmp_path / "none"))
    monkeypatch.setattr(ruckig_opt, "available", lambda: False)
    said = []
    assert mt.make_missing(said.append) == [] and "cannot be made here" in said[0] and "LITE_MECHANISM_TIMING=planned" in said[0]
    with pytest.raises(UserError) as refused:
        mt.require(["lid_open"])
    assert refused.value.code == "timing"


def test_the_planned_timing_is_alixs_plan_with_its_junction_blend(planned):
    plan = mechanism.Plan("seat_open")
    assert plan.timing == "planned" and plan.path == mechanism.PLANS / "seat_open_toppra.json" and plan.rungs_stem == "seat_open_toppra"
    assert not any(e.timed for e in plan.entries)
    assert plan.streamed()["blended_s"] > 0.5                      # the stops at its junctions taken out, as before 18 Nov
    assert mt.missing() == {}
