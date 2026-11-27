"""tools/imu_runs.py: the IMU against the arms' runs (docs/imu/PLAN.md step F), on a small made-up day.

The UGV-load model is replaced by a stand-in (the model is on another branch, and the URDF is slow): what is tested is
the plumbing around it. The clock fit, bias, lag and gain fit, peaks, steps, Program files and line matching are all
exercised through ``synth`` then ``analyse``, as the lab's logs would be."""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from alix_studio_lite.motion import track

TOOL = Path(__file__).resolve().parents[1] / "tools" / "imu_runs.py"
T0 = 1000.0                  # monotonic seconds at the feed's start
OFF = 1.795e9                # wall clock minus monotonic on the made-up laptop
NAME = "61_R_TEST_prog_for_imu_runs_analysis.txt"
LISTED = NAME[:32]           # the controller lists names cut at 32 characters
PLAY = 0.0056                # the controller plays about 5.6 ms a line


@pytest.fixture(scope="module")
def runs():
    spec = importlib.util.spec_from_file_location("imu_runs", TOOL)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    yield mod
    sys.modules.pop(spec.name, None)


def program() -> np.ndarray:
    """(1600, 7) degrees: J2 swings 20 deg out and back between lines 400 and 800; the rest holds."""
    q = np.tile(np.array([10.0, -90.0, 0.0, 90.0, 0.0, 80.0, 5.0]), (1600, 1))
    k = np.arange(400, 801)
    q[k, 1] += 20.0 * np.sin(np.pi * (k - 400) / 400.0) ** 2
    return q


def make_day(root: Path) -> tuple[Path, Path]:
    day, progs = root / "logs" / "2026-12-01", root / "programs"
    (day / "canfd").mkdir(parents=True)
    (progs / "right").mkdir(parents=True)
    q = program()
    track.write_md(progs / "right" / NAME, track.to_md(q))
    # the feed: still at line 1 for 2 s, then the file at 5.6 ms a line, then still at its last line
    feed = day / "canfd" / "20261201_120000_RIGHT_s2_01_TEST.frames.jsonl"
    rows, start = [], T0 + 2.0
    for t in np.arange(T0, T0 + 16.0, 0.02):
        line = int(np.clip((t - start) / PLAY, 0, len(q) - 1))
        jp = [int(round(v * 1000)) for v in q[line]]
        rows.append(json.dumps({"t_mono": round(float(t), 6), "frame": {"joint_status": {"joint_position": jp},
                                                                       "arm_current_status": "x"}}))
    feed.write_text("\n".join(rows) + "\n")
    end = start + len(q) * PLAY
    hw = [dict(t=T0 + OFF, side="right", verb="s2 feed", args={"cycle_ms": 20, "file": feed.name}, text="recording"),
          dict(t=T0 + 0.5 + OFF, side="right", verb="runner v14", text="WORKFLOW START: 1 step(s) on 10.0.0.1"),
          dict(t=T0 + 1.0 + OFF, side="right", verb="runner v14", text="WORKFLOW STEP START 1/1: PROGRAM"),
          dict(t=T0 + 1.1 + OFF, side="right", verb="runner v14",
               text=f"WORKFLOW PROGRAM 1/1: slot 61 ({LISTED}) // START/TRANSIT 30%"),
          dict(t=end + 0.1 + OFF, side="right", verb="runner v14", text="WORKFLOW PROGRAM COMPLETE 1/1: 9.0 s"),
          dict(t=end + 0.2 + OFF, side="right", verb="runner v14", text="WORKFLOW STEP COMPLETE 1/1: PROGRAM"),
          dict(t=end + 0.3 + OFF, side="right", verb="runner v14", text="WORKFLOW COMPLETE: 1/1 step(s)"),
          # a left workflow that ended without completing, and Lite's own pole move
          dict(t=T0 + 0.2 + OFF, side="left", verb="runner v14", text="WORKFLOW STEP START 1/2: PROGRAM"),
          dict(t=T0 + 0.3 + OFF, side="left", verb="runner v14", text="WORKFLOW PROGRAM 1/2: slot 62 (62_L_X)"),
          dict(t=T0 + 0.9 + OFF, side="left", verb="runner v14", text="WORKFLOW SPEED RESTORE: 50% // ret=0"),
          dict(t=T0 + 14.0 + OFF, side="left", verb="pole and hand",
               text="pole 150.0 -> 200.0 mm in 1.5 s, the hand moving meanwhile")]
    (day / "hardware.jsonl").write_text("".join(json.dumps(r) + "\n" for r in hw))
    st = [dict(t=float(t) + OFF, arms={"right": {"pole_mm": "200.0"}, "left": {"pole_mm": "150.0"}})
          for t in np.arange(T0 - 30, T0 + 40, 0.5)]
    (day / "status.jsonl").write_text("".join(json.dumps(r) + "\n" for r in st))
    return day, progs


def stand_in(side, q_deg, dt, pole_m):
    """No dynamic load; a 'weight' moment that follows J2 (5 N m a degree about x, 3 about y)."""
    q = np.asarray(q_deg, dtype=float)
    w = np.column_stack([5.0 * q[:, 1], 3.0 * q[:, 1], np.zeros(len(q))])
    return np.zeros_like(w), w


def test_lines_are_matched_in_order_through_a_hold_and_a_slower_clock(runs):
    q = program()
    q[900:1200] = q[899]                                       # a hold: 300 equal lines
    t = np.arange(0.0, 12.0, 0.02)
    truth = np.clip((t - 1.0) / PLAY, 0, len(q) - 1).astype(int)
    qf = q[truth] + np.random.default_rng(3).normal(0, 0.02, (len(t), 7))
    moving = (truth > 450) & (truth < 750)                   # where J2 moves (at the swing's ends lines are equal)
    lines, err = runs.match_lines(qf, q)                       # 600 frames x 1600 lines: every line is tried
    assert np.all(np.diff(lines) >= 0)
    assert np.max(np.abs(lines[moving] - truth[moving])) <= 5     # at the turn's top 4 lines differ by the noise
    assert np.median(err[moving]) < 0.1
    # thinned to every 20th line, then refined: at the swing's turn the arm passes the same pose twice, so a frame
    # there can take the line on the other side of the turn (here up to 10 lines, 50 ms)
    lines, err = runs.match_lines(qf, q, max_cells=50_000)
    assert np.all(np.diff(lines) >= 0)
    assert np.max(np.abs(lines[moving] - truth[moving])) <= 12
    assert np.median(np.abs(lines[moving] - truth[moving])) <= 1


def test_steps_are_read_with_their_ends(runs, tmp_path):
    day, _ = make_day(tmp_path)
    steps = runs.read_steps(day / "hardware.jsonl", OFF)
    by = {(w["side"], w["kind"]): w for w in steps}
    prog = by[("right", "PROGRAM")]
    assert prog["name"] == LISTED and prog["label"].startswith("Program 61")
    assert prog["t0"] == pytest.approx(T0 + 1.0) and prog["t1"] == pytest.approx(T0 + 2.0 + 1600 * PLAY + 0.2)
    assert by[("left", "PROGRAM")]["label"].endswith("(ended early)")
    assert by[("left", "POLE+HAND")]["t0"] == pytest.approx(T0 + 12.5)


def test_a_made_up_capture_gives_back_its_clock_bias_lag_gain_and_the_program_lines(runs, tmp_path, monkeypatch):
    monkeypatch.setattr(runs, "arm_moments", stand_in)
    monkeypatch.setattr(runs, "model_available", lambda: None)
    day, progs = make_day(tmp_path)
    cap = tmp_path / "capture"
    assert runs.main(["synth", "--logs", str(day), "--out", str(cap), "--pad", "12", "--lag-ms", "40",
                      "--gain-roll", "0.01", "--gain-pitch", "-0.02", "--drift-ppm", "30", "--noise", "0.02"]) == 0
    assert json.loads((cap / "meta.json").read_text())["synthetic"] is True
    assert runs.main(["analyse", str(cap), "--logs", str(day), "--programs", str(progs), "--still-s", "8"]) == 0
    out = cap / "analysis"
    s = json.loads((out / "summary.json").read_text())
    assert s["synthetic"] and s["drift_ppm"] == pytest.approx(30.0, abs=2.0)
    assert s["bias_start"][0] == pytest.approx(0.20, abs=0.01) and s["bias_end"][0] == pytest.approx(0.24, abs=0.01)
    fit = {f["axis"]: f for f in s["fits"] if f["prediction"] == "model + weight"}
    assert fit["roll"]["lag_ms"] == pytest.approx(40.0, abs=10.0)
    assert fit["roll"]["gain"] == pytest.approx(0.01, rel=0.1) and fit["pitch"]["gain"] == pytest.approx(-0.02, rel=0.1)
    assert fit["roll"]["r2"] > 0.9
    run = next(r for r in s["runs"] if r["side"] == "right")
    assert run["file"] == NAME and run["match_deg"] < 0.3
    top = max(s["peaks"], key=lambda q: q["peak_deg_s"])
    assert "Program 61" in top["right"]
    lo, hi = (int(v) for v in top["lines"][0].rsplit("lines ", 1)[1].split("-"))
    assert 400 <= lo and hi <= 820                              # inside J2's swing
    report = (out / "report.md").read_text()
    assert "MADE-UP CAPTURE" in report and (out / "overview.png").exists()
    assert len(list(out.glob("run_*.png"))) == 2                # a graph for each Program step, the left's too


def test_without_the_model_the_report_says_why(runs, tmp_path, monkeypatch):
    monkeypatch.setattr(runs, "arm_moments", stand_in)
    monkeypatch.setattr(runs, "model_available", lambda: None)
    day, progs = make_day(tmp_path)
    cap = tmp_path / "capture"
    runs.main(["synth", "--logs", str(day), "--out", str(cap), "--pad", "12"])
    monkeypatch.setattr(runs, "model_available", lambda: "the model is on another branch")
    assert runs.main(["analyse", str(cap), "--logs", str(day), "--programs", str(progs)]) == 0
    report = (cap / "analysis" / "report.md").read_text()
    assert "Not compared: the model is on another branch" in report
    assert "61_R_TEST_prog_for_imu_runs_analysis.txt" in report      # the lines are still found
