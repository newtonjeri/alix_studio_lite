"""S2 Programs slowed where they ended early on the controller, and the other arm's Programs over the same time (the
user, 4 Nov): motion/retime.py and server/slow_programs.py."""
import importlib.util
import json

import numpy as np

from alix_studio_lite import vendor_paths
from alix_studio_lite.motion import retime, track
from alix_studio_lite.server import slow_programs as sp


def _sweep(n, seed=0):
    """A smooth 7-joint trajectory in millidegrees, fast enough to matter (some 150 deg/s at its peak)."""
    t = np.arange(n) * 0.005
    rng = np.random.default_rng(seed)
    amp, freq, ph = rng.uniform(20, 60, 7), rng.uniform(0.3, 0.6, 7), rng.uniform(0, 6, 7)
    return np.round(1000 * amp * np.sin(2 * np.pi * freq * t[:, None] + ph)).astype(np.int64)


def test_a_window_plays_five_percent_slower_on_the_same_path_and_nothing_else_changes():
    md = _sweep(2000)
    win = [(800.0, 1100.0)]
    new, add = retime.slow(md, win, 30, 1.05)
    assert add == retime.added_lines(len(md), win, 30, 1.05) and add == round(0.05 * (300 - 30))
    assert len(new) == len(md) + add
    assert (new[0] == md[0]).all() and (new[-1] == md[-1]).all()
    assert (new[:800] == md[:800]).all()                         # before the window: the same lines
    assert (new[1100 + add + 1:] == md[1100 + 1:]).all()         # after it: the same lines, later by what was added
    assert retime.path_gap_md(md, new) <= 1.0                    # the same joint path (rounding only)
    r0, r1 = np.abs(retime.rates(md)), np.abs(retime.rates(new))
    core0, core1 = r0[830:1070].max(axis=0), r1[831:1070 + add - 1].max(axis=0)
    assert np.allclose(core1, core0 / 1.05, rtol=0.02, atol=1.0)  # the window's core 5 % slower
    a0 = np.abs(np.diff(md / 1000, 2, axis=0)).max() / 0.005 ** 2
    a1 = np.abs(np.diff(new / 1000, 2, axis=0)).max() / 0.005 ** 2
    assert a1 <= a0 + 250                                         # ramped: no step in speed at the window's edges


def test_a_partner_gains_the_lines_it_is_given_and_none_without_a_window():
    md = _sweep(1500, seed=3)
    new, add = retime.slow(md, [(200.0, 500.0)], 30, 1.05, added=14)
    assert add == 14 and len(new) == 1514
    same, none = retime.slow(md, [], 30, 1.05)
    assert none == 0 and (same == md).all()


def _day(logs, day, lines, rows):
    d = logs / day
    d.mkdir(parents=True)
    (d / "lite.log").write_text("\n".join(lines) + "\n")
    (d / "status.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")


def _t(day, hms):
    return sp._epoch(day, hms)


def test_the_early_ends_are_lites_own_diagnoses_with_the_joints_the_arm_stopped_at(tmp_path):
    day = "2026-11-04"
    moving, rest = [1.0] * 7, [-6.1, -0.6, 19.4, 78.7, -17.5, -57.8, 0.8]
    _day(tmp_path, day, [
        "13:29:49.900 [RIGHT] runner v14 · WORKFLOW PROGRAM 1/5: slot 2 (2_R_P2_R04_pole200_grip_7.6s_v.t) // START/TRANSIT 30%",
        "13:29:50.100 [LEFT] runner v14 · WORKFLOW PROGRAM 1/3: slot 2 (2_L_P2_R05_PART1+R05_PART2_pol.t) // START/TRANSIT 30%",
        "13:29:54.300 [RIGHT] runner v14 · WORKFLOW PROGRAM COMPLETE 1/5: 4.090 s",
        "13:29:54.400 [BOTH] ALERT ERROR · S2 Real Machine: both arms: PART 3 STOPPED // LEFT rc=1 // RIGHT rc=1: LEFT: stopped "
        "because the RIGHT arm's part failed; RIGHT: program 2 ended after 4.1 s of the 7.6 s its points take: it was "
        "stopped on the controller (Check the arms and their controllers' errors before anything else runs.)",
    ], [
        dict(t=_t(day, "13:29:54.150"), arms=dict(right=dict(joints_deg=moving, errors=[]))),
        dict(t=_t(day, "13:29:54.650"), arms=dict(right=dict(joints_deg=rest, errors=[]))),
    ])
    (e,) = sp.early_ends(tmp_path)
    assert (e["side"], e["slot"], e["ran_s"], e["of_s"]) == ("right", 2, 4.1, 7.6)
    assert e["name"] == "2_R_P2_R04_pole200_grip_7.6s_v.t"
    assert e["q_end"] == rest                          # the row after the alert: the arm at rest, not still moving
    assert sp.early_ends(tmp_path, since=_t(day, "13:30:00.000")) == []


def _sim(tmp_path, right_md, left_md, right_name, left_name):
    adv = tmp_path / "advanced"
    rf = track.write_md(adv / "right" / right_name, right_md)
    lf = track.write_md(adv / "left" / left_name, left_md)
    act = lambda side, slot, f, md, t0, text, pole, hand: dict(       # noqa: E731
        side=side, kind="program", program=slot, file=str(f), file_name=f.name, lines=len(md), line_s=0.005,
        traj_t0=t0, t0=t0 - 0.4, t1=t0 + (len(md) - 1) * 0.005 + 0.3, text=text, pole0=pole, hand0=hand)
    return dict(made="2026-11-04 16:00:00", acts=dict(
        right=[act("right", 2, rf, right_md, 60.0, "Program 2 // R_P2_R04_pole200_grip_7.6s_v", 200.0, [0, 0, 0, 0, 150, 1000])],
        left=[dict(side="left", kind="hand", t0=58.0, t1=59.0, text="Hand 1000"),
              act("left", 2, lf, left_md, 59.5, "Program 2 // L_P2_R05_PART1+R05_PART2_pol", 200.0, [1000] * 4 + [0, 1000])]),
        identity=dict(used=dict(right=dict(programs={"2": track.sha256_file(rf)}),
                                left=dict(programs={"2": track.sha256_file(lf)})))), rf, lf


def test_the_stop_is_placed_in_its_file_and_both_arms_gain_the_same_time_there(tmp_path):
    right_md, left_md = _sweep(1512, seed=1), _sweep(2778, seed=2)
    sim, rf, lf = _sim(tmp_path, right_md, left_md, "20261029_182812_R_P2_R04_pole200_grip_7.6s_v1.1.txt",
                       "20261029_182552_L_P2_R05_PART1+R05_PART2_pole200_cleaningpose_13.9s_v1.1.txt")
    stop = 770
    end = dict(day="2026-11-04", at="13:29:54", t=_t("2026-11-04", "13:29:54.400"), side="right", slot=2, ran_s=4.1,
               of_s=7.6, name="2_R_P2_R04_pole200_grip_7.6s_v.t", q_end=(right_md[stop] / 1000 + 0.3).tolist())
    old = dict(end, day="2026-10-28", t=_t("2026-10-28", "10:00:00.000"))              # before the file was sent
    other = dict(end, slot=5, name="5_R_other.t")                                       # not in this simulation
    p = sp.plan(sim, [end, old, other])
    (placed,) = p["placed"]
    assert placed["line"] == stop and placed["gap_deg"] <= 0.31
    assert placed["window_s"] == [round(60.0 + stop * 0.005 - 1.0, 3), round(60.0 + stop * 0.005 + 0.5, 3)]
    assert any("before this file was sent" in n for n in p["notes"])
    assert any("not a Program of this simulation" in n for n in p["notes"])
    rep = sp.write(p, tmp_path / "out")
    by = {r["side"]: r for r in rep["files"]}
    assert set(by) == {"left", "right"}
    added = {s: r["lines"][1] - r["lines"][0] for s, r in by.items()}
    assert added["left"] == added["right"] > 0                       # the same time for both arms
    assert by["right"]["windows_s2_s"] == by["left"]["windows_s2_s"]   # over the same stretch of S2's time
    for r in by.values():
        assert r["same_ends"] and r["path_gap_md"] <= 1.0
        assert max(r["peak_deg_s_at_4_8ms"][1]) < max(r["peak_deg_s_at_4_8ms"][0])
    assert by["right"]["name"] == "R_P2_R04_pole200_grip_7.6s_v1.2.txt"
    left_s = (2778 + added["left"] - 1) * 0.005                       # the name states the file's new time
    assert by["left"]["name"] == f"L_P2_R05_PART1+R05_PART2_pole200_cleaningpose_{left_s:.1f}s_v1.2.txt"
    assert (by["right"]["pole_mm"], by["right"]["hand"]) == (200.0, [0, 0, 0, 0, 150, 1000])
    new = track.read_md(tmp_path / "out" / "right" / by["right"]["name"])
    assert len(new) == 1512 + added["right"] and (new[:stop - 200] == right_md[:stop - 200]).all()
    assert (tmp_path / "out" / "report.md").read_text().count("| RIGHT | 2 |") == 1
    assert rf.read_bytes() == track.track_bytes(right_md) and lf.read_bytes() == track.track_bytes(left_md)  # untouched


def test_a_file_the_simulation_did_not_run_is_left_out(tmp_path):
    right_md, left_md = _sweep(1512, seed=1), _sweep(2778, seed=2)
    sim, rf, _ = _sim(tmp_path, right_md, left_md, "20261029_182812_R_P2_R04_pole200_grip_7.6s_v1.1.txt",
                      "20261029_182552_L_P2_R05_pole200_13.9s_v1.1.txt")
    track.write_md(rf, right_md[::-1])                                  # replaced since the simulation
    end = dict(day="2026-11-04", at="13:29:54", t=_t("2026-11-04", "13:29:54.400"), side="right", slot=2, ran_s=4.1,
               of_s=7.6, name="2_R_P2_R04_pole200_grip_7.6s_v.t", q_end=(right_md[770] / 1000).tolist())
    p = sp.plan(sim, [end])
    assert p["placed"] == [] and p["files"] == []
    assert any("not the file the simulation ran" in n for n in p["notes"])


def test_the_tool_reports_what_it_would_slow(tmp_path, capsys, monkeypatch):
    spec = importlib.util.spec_from_file_location("slow_programs_tool", vendor_paths.LITE_ROOT / "tools" / "slow_programs.py")
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    right_md, left_md = _sweep(1512, seed=1), _sweep(2778, seed=2)
    sim, _, _ = _sim(tmp_path, right_md, left_md, "20261029_182812_R_P2_R04_pole200_grip_7.6s_v1.1.txt",
                     "20261029_182552_L_P2_R05_pole200_13.9s_v1.1.txt")
    proj = tmp_path / "Projects" / "P" / "sequence"
    proj.mkdir(parents=True)
    (proj / "s2_sim.json").write_text(json.dumps(sim))
    day = "2026-11-04"
    _day(tmp_path / "Logs", day, [
        "13:29:49.900 [RIGHT] runner v14 · WORKFLOW PROGRAM 1/5: slot 2 (2_R_P2_R04_pole200_grip_7.6s_v.t) // x",
        "13:29:54.400 [BOTH] ALERT ERROR · S2 Real Machine: both arms: PART 3 STOPPED // RIGHT: program 2 ended after 4.1 s "
        "of the 7.6 s its points take: it was stopped on the controller (Check.)",
    ], [dict(t=_t(day, "13:29:54.650"), arms=dict(right=dict(joints_deg=(right_md[770] / 1000).tolist())))])
    monkeypatch.setattr("sys.argv", ["slow_programs.py", "--logs", str(tmp_path / "Logs"), "--projects",
                                     str(tmp_path / "Projects"), "--dry"])
    assert tool.main() == 0
    out = capsys.readouterr().out
    assert "1 placed" in out and "would slow RIGHT program 2" in out and "would slow LEFT program 2" in out
    assert not (tmp_path / "Projects" / "P" / "slowed").exists()
