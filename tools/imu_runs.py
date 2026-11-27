#!/usr/bin/env python3
"""The IMU against the arms' runs (docs/imu/PLAN.md, step F): where the UGV body rocks or jerks, during which arm step
and which Program lines, and how well Lite's UGV-load model predicts it. Offline, on the lab's logs: no robot, no
serial port.

    python3 tools/imu_runs.py analyse Resources/Logs/<day>/imu/<HHMMSS>_<label>
        [--floor Resources/Logs/<day>/imu/<step C capture: arms powered, holding a pose>]
        [--mount 180,0,0]  [--still-s 30]  [--lowpass 30]  [--model-lowpass 8]
        [--programs <folder of Program files> ...]  [--model-src <a Lite checkout with motion/base_load.py>]
    python3 tools/imu_runs.py synth --logs Resources/Logs/2026-11-24 --match 20261124_2103 --out <folder>
        # a MADE-UP capture from a day's arm feeds, with a known gain and lag: it tests this analysis, it measures nothing

What it reads, all written on the same laptop:
- the capture (tools/imu_n100.py capture --seconds ...; started before the run with the robot still, stopped after it
  with the robot still): the sensor's own microsecond clock, fitted to the laptop's monotonic clock through the reads'
  arrival times (offset and drift; the earliest arrivals set the offset);
- the Lite logs of the same day (the folder above imu/): hardware.jsonl (each arm's workflow steps: Program, MoveJ,
  pole, hand, wait), status.jsonl (each arm's pole height), canfd/*.frames.jsonl (each arm's feed during S2, stamped
  with the same monotonic clock);
- the Program files: Resources/Programs/<set>/<arm>/ and any --programs folder, found by the name the controller lists
  (cut at 32 characters) and confirmed by matching the feed's joints to the file's lines in order (not by time).

The signals: the gyro, its bias taken from the still seconds at the start and at the end (linear between them) and
rotated to the UGV's base frame by --mount (roll, pitch, yaw in degrees, sensor to base; the default 180,0,0 is the
board flat with its marked X to the robot's front: the N100's axes are north-east-down). Rotation rates are the same
anywhere on a rigid body, so the board's place does not enter them. The rocking rate is the size of the roll and pitch
rates; a section is a stretch where it is above the bar: 5 x the noise floor's RMS (from --floor, else from the run's
quietest seconds).

The model check needs Lite's UGV-load model (motion/base_load.py, on the lite_auto_optimization branch until it is
merged). Two predictions of what tilts the base, both arms added, from the feed's joints and the status log's poles:
- "model": the moment Lite's model computes, dL/dt (the dynamic load the optimiser limits);
- "model + weight": that and the arm's weight moving over the base (sum of r x m g over the same links), which the
  model leaves out on purpose.
Each is compared, as its rate of change, with the measured roll and pitch rates: one gain and one lag per axis, and
the share of the measured signal it explains (R^2).

Written into <capture>/analysis/: report.md, sections.csv, summary.json, overview.png and one graph per Program run.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import struct
import sys
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parent
LITE_ROOT = TOOLS.parent
RAD = 180.0 / math.pi
G = 9.80665
BAR_X = 5.0                  #: a section is above BAR_X x the noise floor's RMS (PLAN.md section 4)
MERGE_S = 0.25               #: sections closer than this are one
PEAK_GAP_S = 1.0             #: peaks reported at least this far apart (the strongest wins)
AROUND_S = 0.25              #: a peak's lines, acceleration and jerk are read this far either side of it
REPORT_PEAKS = 20            #: the report lists the strongest; sections.csv has them all
VARIANTS = (("model", "model"), ("both", "model + weight"))
FEED_DT = 0.02               #: the S2 feed's period (Lite asks the controller for 20 ms)
MODEL_DT = 0.01              #: the grid the model check runs on
LAGS_S = (-0.2, 0.5)         #: the lags tried: the IMU up to 0.2 s before, 0.5 s after the prediction
MATCH_OK_DEG = 1.0           #: a file matches the feed when the matched lines are this close (median, all joints)
FILE_DT = 0.005              #: a Program file's own clock (5 ms a line)


# ──────────────────────────────────────────────────────────────────────── Lite and the IMU tool

def _lite(model_src: str | None = None) -> None:
    """Lite's package and the ALIX code it vendors on the path. --model-src puts another checkout's first."""
    if model_src:
        sys.path.insert(0, str(Path(model_src).resolve() / "src"))
    try:
        import alix_studio_lite  # noqa: F401
    except ImportError:
        sys.path.append(str(LITE_ROOT / "src"))
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()


def _n100():
    if str(TOOLS) not in sys.path:
        sys.path.insert(0, str(TOOLS))
    import imu_n100

    return imu_n100


def _bandlimit(x: np.ndarray, dt: float, fc: float) -> np.ndarray:
    from alix_studio_lite.motion.checks import bandlimit

    return bandlimit(x, dt, fc)


def model_available() -> str | None:
    """None when Lite's UGV-load model can be used, else why not."""
    try:
        from alix_studio_lite.motion import base_load  # noqa: F401
    except ImportError:
        return ("Lite's UGV-load model (motion/base_load.py) is not in this checkout: it is on the "
                "lite_auto_optimization branch. Give --model-src a checkout of that branch to compare.")
    return None


def arm_moments(side: str, q_deg: np.ndarray, dt: float, pole_m: float) -> tuple[np.ndarray, np.ndarray]:
    """(dynamic, weight), each (N, 3) N m about the UGV base frame's origin, for one arm: Lite's model's dL/dt, and
    the moment of the arm's weight (sum of r x m g over the same moving links). The UGV feels weight - dynamic."""
    from alix_studio_lite.motion import base_load

    arm = base_load.arm_load(side, round(float(pole_m), 2))
    _, dyn = arm.wrench(q_deg, dt)
    com, _ = arm.frames(q_deg)
    gvec = np.array([0.0, 0.0, -base_load.G])
    weight = sum(seg.mass * np.cross(com[k], gvec) for k, seg in enumerate(arm.segments))
    return dyn, weight


# ──────────────────────────────────────────────────────────────────────── the capture

def load_imu(folder: Path) -> dict:
    """The capture's IMU frames on the laptop's monotonic clock."""
    n = _n100()
    buf, chunks = n.load(folder)
    frames, st, _ = n.scan(buf)
    imu = [f for f in frames if f.type == n.T_IMU and len(f.payload) >= 56]
    if len(imu) < 100:
        raise SystemExit(f"{folder}: {len(imu)} IMU frames (0x40), too few to analyse. Check it with "
                         f"`python3 tools/imu_n100.py check {folder}`.")
    if not chunks:
        raise SystemExit(f"{folder}: chunks.jsonl is empty, so the frames cannot be put on the laptop's clock.")
    rec = [n.decode_imu(f.payload) for f in imu]
    ts = np.array([r["ts_us"] for r in rec], dtype=float) / 1e6
    if np.any(np.diff(ts) <= 0):
        raise SystemExit(f"{folder}: the sensor's clock went back {int(np.sum(np.diff(ts) <= 0))} time(s) (the "
                         "module restarted?): capture the run again.")
    ends = np.array([c["o"] + c["n"] for c in chunks])
    k = np.minimum(np.searchsorted(ends, [f.end for f in imu], side="left"), len(chunks) - 1)
    arrival = np.array([chunks[i]["m"] for i in k])
    x = ts - ts[0]
    slope, icpt = np.polyfit(x, arrival, 1)
    res = arrival - (slope * x + icpt)
    early = res <= np.percentile(res, 10)              # the frames read soonest after they were made
    slope, icpt = np.polyfit(x[early], arrival[early], 1)
    res = arrival - (slope * x + icpt)
    icpt += float(np.percentile(res, 1))               # the earliest arrivals: made, then read at once
    t = slope * x + icpt
    lat_ms = (arrival - t) * 1000.0
    step = float(np.median(np.diff(ts)))
    lost = int(np.sum(np.round(np.diff(ts) / step) - 1))
    meta = json.loads((folder / "meta.json").read_text()) if (folder / "meta.json").exists() else {}
    wall = np.array([c["t"] for c in chunks]) - np.array([c["m"] for c in chunks])
    return dict(
        folder=folder, meta=meta, t=t,
        gyro=np.array([r["gyro"] for r in rec], dtype=float) * RAD,          # deg/s, sensor axes
        acc=np.array([r["acc"] for r in rec], dtype=float),                  # m/s^2 with gravity, sensor axes
        temp=np.array([r["temp"] for r in rec], dtype=float),
        rate_hz=1.0 / step, lost=lost, frames=len(imu), crc_bad=st.crc16_bad + st.end_bad,
        drift_ppm=(1.0 / slope - 1.0) * 1e6, lat_p50_ms=float(np.percentile(lat_ms, 50)),
        lat_p99_ms=float(np.percentile(lat_ms, 99)), wall_minus_mono=float(np.median(wall)))


def rot_rpy(deg: tuple) -> np.ndarray:
    """Sensor axes to base axes: Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = np.radians(deg)
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def signals(cap: dict, mount: tuple, still_s: float, lowpass: float) -> dict:
    """Rates and accelerations in the base frame on a uniform grid: bias out, low-passed."""
    t = cap["t"]
    dt = float(np.median(np.diff(t)))
    grid = np.arange(t[0], t[-1], dt)
    r = rot_rpy(mount)
    g = np.column_stack([np.interp(grid, t, cap["gyro"][:, i]) for i in range(3)]) @ r.T
    a = np.column_stack([np.interp(grid, t, cap["acc"][:, i]) for i in range(3)]) @ r.T
    span = grid[-1] - grid[0]
    still_s = min(still_s, span / 2)
    start, end = grid < grid[0] + still_s, grid > grid[-1] - still_s
    b0, b1 = np.median(g[start], axis=0), np.median(g[end], axis=0)
    q0 = float(np.sqrt(np.mean((g[start] - b0) ** 2)))
    q1 = float(np.sqrt(np.mean((g[end] - b1) ** 2)))
    end_still = q1 <= 3.0 * q0 + 0.05
    if end_still:
        w = (grid - grid[0]) / max(span, 1e-9)
        bias = b0 + np.outer(w, b1 - b0)
    else:
        bias = np.broadcast_to(b0, g.shape)
    rates = _bandlimit(g - bias, dt, lowpass)
    acc = _bandlimit(a - np.median(a[start], axis=0), dt, lowpass)
    jerk = np.gradient(acc, dt, axis=0)
    return dict(t=grid, dt=dt, rates=rates, acc=acc, rock=np.hypot(rates[:, 0], rates[:, 1]),
                acc_h=np.hypot(acc[:, 0], acc[:, 1]), jerk_h=np.hypot(jerk[:, 0], jerk[:, 1]),
                bias_start=b0, bias_end=b1, end_still=end_still, still_s=still_s,
                gravity_base=np.median(a[start], axis=0))


def noise_floor(sig: dict, floor_sig: dict | None) -> tuple[float, str]:
    """The rocking rate's noise RMS: from the step C capture, else the run's quietest seconds (10th percentile)."""
    if floor_sig is not None:
        return float(np.sqrt(np.mean(floor_sig["rock"] ** 2))), "the step C capture given with --floor"
    n = max(1, int(round(1.0 / sig["dt"])))
    k = len(sig["rock"]) // n
    rms = np.sqrt(np.mean(sig["rock"][: k * n].reshape(k, n) ** 2, axis=1)) if k else np.array([np.inf])
    return float(np.percentile(rms, 10)), "the run's quietest seconds (no --floor given: a step C capture is better)"


def find_sections(sig: dict, bar: float) -> list[dict]:
    above = sig["rock"] > bar
    idx = np.flatnonzero(np.diff(np.concatenate([[0], above.astype(int), [0]])))
    runs = [[int(a), int(b)] for a, b in zip(idx[::2], idx[1::2])]
    gap = int(MERGE_S / sig["dt"])
    merged: list[list[int]] = []
    for a, b in runs:
        if merged and a - merged[-1][1] <= gap:
            merged[-1][1] = b
        else:
            merged.append([a, b])
    t = sig["t"]
    return [dict(t0=float(t[a]), t1=float(t[b - 1])) for a, b in merged]


def find_peaks_above(sig: dict, bar: float, sections: list) -> list[dict]:
    """The rocking rate's local peaks above the bar, at least PEAK_GAP_S apart: where the base jerks or rocks most."""
    from scipy.signal import find_peaks

    t, dt = sig["t"], sig["dt"]
    pk, _ = find_peaks(sig["rock"], height=bar, distance=max(1, int(PEAK_GAP_S / dt)))
    half = int(AROUND_S / dt)
    out = []
    for k in pk:
        a, b = max(0, k - half), min(len(t), k + half + 1)
        sec = next((s for s in sections if s["t0"] <= t[k] <= s["t1"]), dict(t0=float(t[k]), t1=float(t[k])))
        out.append(dict(t_peak=float(t[k]), peak_deg_s=float(sig["rock"][k]), roll_deg_s=float(sig["rates"][k, 0]),
                        pitch_deg_s=float(sig["rates"][k, 1]), acc_h=float(sig["acc_h"][a:b].max()),
                        jerk_h=float(sig["jerk_h"][a:b].max()), section_t0=sec["t0"], section_t1=sec["t1"]))
    return out


# ──────────────────────────────────────────────────────────────────────── Lite's logs

STEP_START = re.compile(r"WORKFLOW STEP START (\d+)/(\d+): (\w+)")
STEP_DONE = re.compile(r"WORKFLOW STEP COMPLETE (\d+)/(\d+)")
PROGRAM = re.compile(r"WORKFLOW PROGRAM (\d+)/\d+: slot (\d+) \((.*?)\)")
GLOBAL = re.compile(r"GLOBAL MOVE START: (\S+)")
POLE_HAND = re.compile(r"pole ([\d.]+) -> ([\d.]+) mm in ([\d.]+) s")
WORKFLOW_ENDS = (("WORKFLOW COMPLETE", "workflow ended"), ("WORKFLOW STOP REQUESTED", "stopped"),
                 ("WORKFLOW FAILED", "failed"), ("WORKFLOW REFUSED", "refused"),
                 ("WORKFLOW SPEED RESTORE", "ended early"), ("WORKFLOW START:", "not completed"))


def _rows(path: Path) -> list[dict]:
    if not path.exists():
        return []
    out = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            out.append(json.loads(line))
        except ValueError:
            continue
    return out


def read_steps(hw: Path, wall_minus_mono: float) -> list[dict]:
    """Each arm's steps from hardware.jsonl, on the monotonic clock."""
    steps, open_, last = [], {}, {}
    for r in _rows(hw):
        side, text = r.get("side"), (r.get("text") or "").strip()
        if side not in ("left", "right") or not isinstance(r.get("t"), (int, float)):
            continue
        t = float(r["t"]) - wall_minus_mono
        if r.get("verb") == "pole and hand" and (m := POLE_HAND.search(text)):
            steps.append(dict(side=side, kind="POLE+HAND", t0=t - float(m.group(3)), t1=t,
                              label=f"pole {float(m.group(1)):.0f} -> {float(m.group(2)):.0f} mm and hand"))
            continue
        if r.get("verb") != "runner v14":
            continue
        last[side] = t
        end = next((why for mark, why in WORKFLOW_ENDS if text.startswith(mark)), None)
        if end:
            for key in [k for k in open_ if k[0] == side]:
                w = open_.pop(key)
                w.update(t1=t, label=w["label"] + f" ({end})")
            continue
        if m := STEP_START.match(text):
            w = dict(side=side, kind=m.group(3), step=int(m.group(1)), steps=int(m.group(2)), t0=t, t1=None,
                     label=m.group(3))
            open_[(side, w["step"])] = w
            steps.append(w)
        elif m := PROGRAM.match(text):
            w = open_.get((side, int(m.group(1))))
            if w:
                w.update(slot=int(m.group(2)), name=m.group(3).strip(), label=f"Program {m.group(2)} {m.group(3).strip()}")
        elif m := GLOBAL.match(text):
            for w in reversed(steps):
                if w["side"] == side and w["kind"] == "GLOBAL" and w["t1"] is None:
                    w["label"] = f"MoveJ {m.group(1)}"
                    break
        elif m := STEP_DONE.match(text):
            w = open_.pop((side, int(m.group(1))), None)
            if w:
                w["t1"] = t
    for w in steps:
        if w["t1"] is None:
            w["t1"] = last.get(w["side"], w["t0"])
            w["label"] += " (no end logged)"
    return sorted(steps, key=lambda w: w["t0"])


def read_poles(status: Path, wall_minus_mono: float) -> dict:
    """{side: (t, pole_mm)} from status.jsonl."""
    out = {"left": ([], []), "right": ([], [])}
    for r in _rows(status):
        for side in out:
            arm = (r.get("arms") or {}).get(side) or {}
            try:
                mm = float(arm.get("pole_mm"))
            except (TypeError, ValueError):
                continue
            out[side][0].append(float(r["t"]) - wall_minus_mono)
            out[side][1].append(mm)
    return {s: (np.array(t), np.array(v)) for s, (t, v) in out.items()}


def feed_side(name: str) -> str | None:
    up = name.upper()
    return "left" if "_LEFT_" in up else "right" if "_RIGHT_" in up else None


def _first_last_t(path: Path) -> tuple[float, float] | None:
    try:
        with open(path, "rb") as fh:
            first = json.loads(fh.readline())["t_mono"]
            fh.seek(0, 2)
            size = fh.tell()
            fh.seek(max(0, size - 65536))
            tail = fh.read().splitlines()
        for line in reversed(tail):
            try:
                return float(first), float(json.loads(line)["t_mono"])
            except (ValueError, KeyError):
                continue
    except (OSError, ValueError, KeyError):
        return None
    return None


def read_feeds(day: Path, t_lo: float, t_hi: float, match: str = "") -> dict:
    """{side: [dict(name, t, q deg (N, 7), status)]} for the feed files that overlap [t_lo, t_hi]."""
    from alix_dispatch import track

    out: dict = {"left": [], "right": []}
    for f in sorted((day / "canfd").glob("*.frames.jsonl")):
        side = feed_side(f.name)
        span = _first_last_t(f)
        if side is None or span is None or match not in f.name or span[1] < t_lo or span[0] > t_hi:
            continue
        ts, st, qs = track.read_capture(f)
        keep = [i for i, q in enumerate(qs) if q is not None]
        if len(keep) < 10:
            continue
        out[side].append(dict(name=f.name, t=np.array([ts[i] for i in keep]), q=np.array([qs[i] for i in keep]),
                              status=[st[i] for i in keep]))
    return out


# ──────────────────────────────────────────────────────────────────────── Program files

def program_candidates(side: str, listed: str, extra: list[Path]) -> list[Path]:
    """Files whose name starts with the name the controller listed (it cuts names at 32 characters)."""
    dirs = sorted((LITE_ROOT / "Resources" / "Programs").glob(f"*/{side}"))
    for d in extra:
        dirs.append(d / side if (d / side).is_dir() else d)
    key = listed.strip()
    seen, out = set(), []
    for d in dirs:
        for f in sorted(d.glob("*.txt")):
            if f.name.startswith(key) and f.resolve() not in seen:
                seen.add(f.resolve())
                out.append(f)
    return out


def read_program(path: Path) -> np.ndarray:
    """(N, 7) degrees."""
    from alix_studio_lite.motion import track

    return track.to_deg(track.read_md(path))


def match_lines(qf: np.ndarray, qfile: np.ndarray, max_cells: float = 8e6) -> tuple[np.ndarray, np.ndarray]:
    """Each feed frame's file line (0-based, never going back) and its distance in degrees (all joints): the
    assignment with the least total distance. Lines are thinned to keep frames x lines under ``max_cells``."""
    f_n, l_n = len(qf), len(qfile)
    s = max(1, int(math.ceil(f_n * l_n / max_cells)))
    q = qfile[::s]
    idx = np.arange(len(q))
    ptr = np.zeros((f_n, len(q)), dtype=np.int32)
    d = np.linalg.norm(q - qf[0], axis=1)
    for i in range(1, f_n):
        cm = np.minimum.accumulate(d)
        ptr[i] = np.maximum.accumulate(np.where(d <= cm, idx, 0))
        d = np.linalg.norm(q - qf[i], axis=1) + cm
    js = np.empty(f_n, dtype=np.int64)
    js[-1] = int(np.argmin(d))
    for i in range(f_n - 1, 0, -1):
        js[i - 1] = ptr[i][js[i]]
    lines = np.minimum(js * s, l_n - 1)
    if s > 1:                                          # the nearest full-resolution line around each thinned one
        off = np.arange(-s, s + 1)
        cand = np.clip(lines[:, None] + off[None, :], 0, l_n - 1)
        dist = np.linalg.norm(qfile[cand] - qf[:, None, :], axis=2)
        lines = np.maximum.accumulate(cand[np.arange(f_n), np.argmin(dist, axis=1)])
    return lines, np.linalg.norm(qfile[lines] - qf, axis=1)


def map_programs(steps: list[dict], feeds: dict, extra: list[Path]) -> list[dict]:
    """For every Program step with a feed: its file, and the feed frames' file lines."""
    out = []
    for w in steps:
        if w["kind"] != "PROGRAM" or not w.get("name"):
            continue
        frames = [(fd["t"], fd["q"]) for fd in feeds[w["side"]]]
        t = np.concatenate([a for a, _ in frames]) if frames else np.zeros(0)
        q = np.concatenate([b for _, b in frames]) if frames else np.zeros((0, 7))
        sel = (t >= w["t0"]) & (t <= w["t1"])
        run = dict(step=w, file=None, why="", t=t[sel], lines=None, err_deg=None, n_lines=None)
        if sel.sum() < 10:
            run["why"] = "no feed recorded for it"
            out.append(run)
            continue
        cands = program_candidates(w["side"], w["name"], extra)
        if not cands:
            run["why"] = f"no file named {w['name']}... in Resources/Programs or --programs"
            out.append(run)
            continue
        best = None
        for f in cands:
            qfile = read_program(f)
            lines, err = match_lines(q[sel], qfile)
            inner = (lines > 0) & (lines < len(qfile) - 1)
            med = float(np.median(err[inner])) if inner.sum() >= 10 else float("inf")
            if best is None or med < best[0]:
                best = (med, f, lines, err, qfile)
        med, f, lines, err, qfile = best
        run.update(err_deg=med, n_lines=len(qfile))
        if med > MATCH_OK_DEG:
            run["why"] = f"{f.name} does not follow the feed (median {med:.2f} deg off)"
        else:
            run.update(file=f, lines=lines, qfile=qfile)
        out.append(run)
    return out


def file_stats(qfile: np.ndarray, l0: int, l1: int) -> dict:
    """In the file's lines l0..l1 (its own 5 ms clock, band-limited as Lite's checks read it): the joint with the
    highest acceleration, the one with the highest jerk, and the joints whose speed turns back."""
    q = _bandlimit(qfile, FILE_DT, 20.0)
    v = np.gradient(q, FILE_DT, axis=0)
    a = np.gradient(v, FILE_DT, axis=0)
    j = np.gradient(a, FILE_DT, axis=0)
    lo, hi = max(0, l0 - 20), min(len(q), l1 + 21)
    if hi - lo < 2:
        return {}
    am, jm = np.abs(a[lo:hi]).max(axis=0), np.abs(j[lo:hi]).max(axis=0)
    turns = []
    for k in range(7):
        s = v[lo:hi, k]
        fast = np.abs(s) > 2.0
        sg = np.sign(s[fast])
        if len(sg) > 1 and np.any(sg[1:] != sg[:-1]):
            turns.append(f"J{k + 1}")
    ka, kj = int(np.argmax(am)), int(np.argmax(jm))
    return dict(acc_joint=f"J{ka + 1}", acc_deg_s2=float(am[ka]), jerk_joint=f"J{kj + 1}",
                jerk_deg_s3=float(jm[kj]), turns=" ".join(turns))


# ──────────────────────────────────────────────────────────────────────── the model check

def predicted(feeds: dict, poles: dict, grid: np.ndarray, lowpass: float) -> dict | None:
    """What tilts the base, both arms added, on ``grid``: d/dt of the moment the UGV feels, from the model alone and
    from model + weight. Each feed file is differentiated on its own (an arm's weight moment is tens of N m, so a
    recording's edge must never read as a step); the grid is covered where every recorded arm has a feed."""
    model = np.zeros((len(grid), 3))
    both = np.zeros((len(grid), 3))
    cover = np.ones(len(grid), dtype=bool)
    sides = 0
    for side, files in feeds.items():
        if not files:
            continue
        sides += 1
        side_cover = np.zeros(len(grid), dtype=bool)
        pt, pv = poles[side]
        for fd in files:
            tg = np.arange(fd["t"][0], fd["t"][-1], FEED_DT)
            if len(tg) < 20:
                continue
            qg = np.column_stack([np.interp(tg, fd["t"], fd["q"][:, k]) for k in range(7)])
            qg = _bandlimit(qg, FEED_DT, 20.0)              # as Lite's checks read joints; the band is set below
            mm = np.interp(tg, pt, pv) if len(pt) else np.full(len(tg), 150.0)
            band = np.round(mm / 10.0) * 10.0
            d_s, w_s = np.zeros((len(tg), 3)), np.zeros((len(tg), 3))
            cuts = np.flatnonzero(np.diff(band)) + 1
            for a, b in zip(np.r_[0, cuts], np.r_[cuts, len(tg)]):
                if b - a >= 5:
                    d_s[a:b], w_s[a:b] = arm_moments(side, qg[a:b], FEED_DT, band[a] / 1000.0)
                elif a > 0:
                    d_s[a:b], w_s[a:b] = d_s[a - 1], w_s[a - 1]
            # the UGV feels weight - dL/dt; its rate of change tilts it
            m_rate = -np.gradient(d_s, FEED_DT, axis=0)
            b_rate = np.gradient(w_s - d_s, FEED_DT, axis=0)
            inside = (grid >= tg[0]) & (grid <= tg[-1])
            for k in range(3):
                model[inside, k] += np.interp(grid[inside], tg, m_rate[:, k])
                both[inside, k] += np.interp(grid[inside], tg, b_rate[:, k])
            side_cover |= inside
        cover &= side_cover
    if not sides or not cover.any():
        return None
    return dict(model=_bandlimit(model, MODEL_DT, lowpass), both=_bandlimit(both, MODEL_DT, lowpass), cover=cover)


def fit_lag_gain(meas: np.ndarray, pred: np.ndarray, mask: np.ndarray, dt: float) -> dict:
    """meas(t) ~ gain x pred(t - lag): the lag with the best fit, its gain and R^2."""
    best = None
    for k in range(int(LAGS_S[0] / dt), int(LAGS_S[1] / dt) + 1):
        p = np.roll(pred, k)
        m = mask & np.roll(mask, k)
        if k > 0:
            m[:k] = False
        elif k < 0:
            m[k:] = False
        if m.sum() < 50:
            continue
        pp = float(p[m] @ p[m])
        if pp <= 0:
            continue
        g = float(meas[m] @ p[m]) / pp
        sst = float(np.sum((meas[m] - meas[m].mean()) ** 2))
        r2 = 1.0 - float(np.sum((meas[m] - g * p[m]) ** 2)) / sst if sst > 0 else 0.0
        if best is None or r2 > best["r2"]:
            best = dict(lag_ms=k * dt * 1000.0, gain=g, r2=r2, n=int(m.sum()))
    return best or dict(lag_ms=float("nan"), gain=float("nan"), r2=float("nan"), n=0)


# ──────────────────────────────────────────────────────────────────────── analyse

def _steps_at(steps: list[dict], side: str, a: float, b: float) -> str:
    hit = [w["label"] for w in steps if w["side"] == side and w["t0"] <= b and w["t1"] >= a]
    return "; ".join(dict.fromkeys(hit)) or "-"


def _graph(path: Path, title: str, sig: dict, bar: float, sections: list, peaks: list, t0: float, a: float, b: float,
           pred: dict | None) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    sel = (sig["t"] >= a) & (sig["t"] <= b)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6.2), sharex=True)
    fig.suptitle(title + (f" (dashed: {pred['variant']}, fitted)" if pred else ""), fontsize=11, x=0.01, ha="left",
                 y=0.985)
    band = f"at {pred['band']:g} Hz" if pred else "rates"
    fig.text(0.01, 0.935, "Top: base rocking rate measured (black), the bar (red dashed), peaks (dots); shaded: above "
             f"the bar. Bottom: roll (blue) and pitch (orange) {band}, solid measured, dashed predicted.", fontsize=8.5,
             ha="left", color="#444")
    x = sig["t"][sel] - t0
    ax1.plot(x, sig["rock"][sel], color="black", lw=0.8)
    ax1.axhline(bar, color="#c0392b", ls="--", lw=1)
    pk = [q for q in peaks if a <= q["t_peak"] <= b]
    ax1.plot([q["t_peak"] - t0 for q in pk], [q["peak_deg_s"] for q in pk], "o", color="black", ms=3.5)
    for s in sections:
        if s["t1"] >= a and s["t0"] <= b:
            for ax in (ax1, ax2):
                ax.axvspan(s["t0"] - t0, s["t1"] - t0, color="#999", alpha=0.25, lw=0)
    ax1.set_ylabel("rocking rate, deg/s")
    if pred is None:
        ax2.plot(x, sig["rates"][sel, 0], color="#1f77b4", lw=0.8)
        ax2.plot(x, sig["rates"][sel, 1], color="#ff7f0e", lw=0.8)
    else:                                              # like with like: the measurement at the model's band
        ps = (pred["t"] >= a) & (pred["t"] <= b)
        xp = pred["t"][ps] - t0
        ax2.plot(xp, pred["meas_roll"][ps], color="#1f77b4", lw=0.9)
        ax2.plot(xp, pred["meas_pitch"][ps], color="#ff7f0e", lw=0.9)
        ax2.plot(xp, pred["roll"][ps], color="#1f77b4", ls="--", lw=1.3)
        ax2.plot(xp, pred["pitch"][ps], color="#ff7f0e", ls="--", lw=1.3)
    ax2.set_ylabel("rate, deg/s")
    ax2.set_xlabel("seconds from the capture's start")
    ax2.set_xlim(a - t0, b - t0)
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(path, dpi=110)
    plt.close(fig)


def _fitted(p: dict, fits: list, variant: str, label: str) -> dict:
    """The prediction ``variant`` with its own fitted gain and lag per axis, in deg/s (NaN where not covered)."""
    out = {}
    for k, axis in enumerate(("roll", "pitch")):
        f = next(f for f in fits if f["prediction"] == label and f["axis"] == axis)
        lag = int(round(f["lag_ms"] / 1000.0 / MODEL_DT)) if math.isfinite(f["lag_ms"]) else 0
        y = (f["gain"] if math.isfinite(f["gain"]) else 0.0) * np.roll(p[variant][:, k], lag)
        y[~np.roll(p["cover"], lag)] = np.nan
        out[axis] = y
    return out


def cmd_analyse(a) -> int:
    _lite(a.model_src)
    folder = Path(a.capture).resolve()
    cap = load_imu(folder)
    mount = tuple(float(v) for v in a.mount.split(","))
    sig = signals(cap, mount, a.still_s, a.lowpass)
    floor_sig = signals(load_imu(Path(a.floor).resolve()), mount, 1e9, a.lowpass) if a.floor else None
    floor, floor_src = noise_floor(sig, floor_sig)
    bar = BAR_X * floor
    sections = find_sections(sig, bar)
    peaks = find_peaks_above(sig, bar, sections)
    t0 = float(sig["t"][0])

    day = Path(a.logs).resolve() if a.logs else folder.parents[1]
    off = cap["wall_minus_mono"]
    steps = read_steps(day / "hardware.jsonl", off)
    steps = [w for w in steps if w["t1"] >= sig["t"][0] and w["t0"] <= sig["t"][-1]]
    poles = read_poles(day / "status.jsonl", off)
    feeds = read_feeds(day, float(sig["t"][0]), float(sig["t"][-1]))
    runs = map_programs(steps, feeds, [Path(p) for p in a.programs])

    # the model check
    why_no_model = model_available()
    fits, curves, shown = [], {}, None
    if why_no_model is None and any(feeds.values()):
        grid = np.arange(sig["t"][0], sig["t"][-1], MODEL_DT)
        p = predicted(feeds, poles, grid, a.model_lowpass)
        if p is not None:
            meas = np.column_stack([np.interp(grid, sig["t"], _bandlimit(sig["rates"][:, k], sig["dt"],
                                                                          a.model_lowpass)) for k in range(2)])
            for variant, label in VARIANTS:
                for k, axis in enumerate(("roll", "pitch")):
                    f = fit_lag_gain(meas[:, k], p[variant][:, k], p["cover"], MODEL_DT)
                    fits.append(dict(prediction=label, axis=axis, **f))
                curves[label] = _fitted(p, fits, variant, label)
            r2 = {label: [f["r2"] for f in fits if f["prediction"] == label and math.isfinite(f["r2"])]
                  for _, label in VARIANTS}
            best = max(r2, key=lambda lb: float(np.mean(r2[lb])) if r2[lb] else -math.inf)
            shown = dict(t=grid, variant=best, band=a.model_lowpass, meas=np.hypot(meas[:, 0], meas[:, 1]),
                         meas_roll=meas[:, 0], meas_pitch=meas[:, 1], **curves[best])
        else:
            why_no_model = "the arm feeds do not cover this capture together, so there is nothing to predict from."
    elif why_no_model is None:
        why_no_model = "no arm feed (canfd/*.frames.jsonl) overlaps this capture, so there is nothing to predict from."

    # each peak: the arms' steps, the Program lines around it, what the file does there, what the model predicted
    for q in peaks:
        lo, hi = q["t_peak"] - AROUND_S, q["t_peak"] + AROUND_S
        q["left"] = _steps_at(steps, "left", lo, hi)
        q["right"] = _steps_at(steps, "right", lo, hi)
        q["lines"], q["file_notes"] = [], []
        for r in runs:
            if r["file"] is None or not (r["step"]["t0"] <= hi and r["step"]["t1"] >= lo):
                continue
            l0, l1 = int(np.interp(lo, r["t"], r["lines"])), int(np.interp(hi, r["t"], r["lines"]))
            side = r["step"]["side"].upper()
            q["lines"].append(f"{side} {r['file'].name} lines {l0 + 1}-{l1 + 1}")
            st = file_stats(r["qfile"], l0, l1)
            if st:
                q["file_notes"].append(f"{side}: most acceleration {st['acc_joint']} {st['acc_deg_s2']:.0f} deg/s^2, "
                                       f"most jerk {st['jerk_joint']} {st['jerk_deg_s3']:.0f} deg/s^3"
                                       + (f", turns back: {st['turns']}" if st["turns"] else ""))
        m = (shown["t"] >= lo) & (shown["t"] <= hi) if shown else None
        q["meas model band"] = float(shown["meas"][m].max()) if m is not None and m.any() else None
        for _, label in VARIANTS:
            c = curves.get(label)
            v = np.hypot(c["roll"][m], c["pitch"][m]) if c is not None and m is not None and m.any() else np.array([])
            q[f"pred {label}"] = float(np.nanmax(v)) if v.size and np.isfinite(v).any() else None

    out = folder / "analysis"
    out.mkdir(exist_ok=True)
    for old in out.glob("*.png"):
        old.unlink()

    # report
    L = []
    say = L.append
    say(f"# IMU against the arms: {folder.name}\n")
    if cap["meta"].get("synthetic"):
        say("**MADE-UP CAPTURE** (`tools/imu_runs.py synth`): this tests the analysis; it measures nothing.\n")
    say(f"- Capture: {cap['frames']:,} IMU frames at {cap['rate_hz']:.1f} per second, {cap['lost']} lost, "
        f"{cap['crc_bad']} damaged; {sig['t'][-1] - sig['t'][0]:.1f} s.")
    say(f"- Clocks: the sensor's runs {cap['drift_ppm']:+.1f} ppm against the laptop's; frames reach the laptop "
        f"{cap['lat_p50_ms']:.1f} ms (median) and {cap['lat_p99_ms']:.1f} ms (99 %) after the earliest.")
    b0, b1 = sig["bias_start"], sig["bias_end"]
    say(f"- Gyro bias (base axes, deg/s): start {b0[0]:+.3f} {b0[1]:+.3f} {b0[2]:+.3f}; end {b1[0]:+.3f} {b1[1]:+.3f} "
        f"{b1[2]:+.3f}" + ("" if sig["end_still"] else " (the end was not still: the start's bias is used throughout)")
        + f"; from the first and last {sig['still_s']:.0f} s.")
    gb = sig["gravity_base"]
    say(f"- Gravity at rest in base axes (m/s^2): {gb[0]:+.2f} {gb[1]:+.2f} {gb[2]:+.2f}. With the mount right it is "
        f"about 0 0 +9.81; anything else means --mount ({a.mount}) is wrong.")
    say(f"- Noise floor {floor:.4f} deg/s RMS, from {floor_src}; the bar is {BAR_X:.0f} x that: {bar:.4f} deg/s.")
    say(f"- Low-pass {a.lowpass:g} Hz for the peaks, {a.model_lowpass:g} Hz for the model check.\n")

    say("## Does Lite's model predict the rocking?\n")
    if fits:
        say("| Prediction | Axis | Lag, ms (IMU later +) | Gain, deg per N m | Share explained (R^2) |")
        say("|---|---|---|---|---|")
        for f in fits:
            say(f"| {f['prediction']} | {f['axis']} | {f['lag_ms']:+.0f} | {f['gain']:.4g} | {f['r2']:.3f} |")
        say("\n\"model\" is the dynamic load Lite's optimiser limits; \"model + weight\" adds the arm's weight moving over "
            "the base, which the model leaves out. Each is compared as its rate of change; the gain is the tilt per "
            "newton metre (its sign follows the axes). Pole moves are not modelled: the model holds the pole still.\n")
    else:
        say(f"Not compared: {why_no_model}\n")

    say("## Program runs\n")
    if runs:
        say("| Arm | Program | File | Match, deg | Note |")
        say("|---|---|---|---|---|")
        for r in runs:
            w = r["step"]
            match = "-" if r["err_deg"] is None else f"{r['err_deg']:.2f}"
            say(f"| {w['side'].upper()} | {w['label']} | {r['file'].name if r['file'] else '-'} | {match} | "
                f"{r['why'] or '-'} |")
        say("\nMatch: how far the feed's joints are from the file's matched lines (median, all joints).\n")
    else:
        say("No Program step of either arm is logged during this capture.\n")

    top = sorted(sorted(peaks, key=lambda q: -q["peak_deg_s"])[:REPORT_PEAKS], key=lambda q: q["t_peak"])
    say(f"## Peaks above the bar ({len(peaks)}; the {len(top)} strongest here, in time order; all in sections.csv)\n")
    if top:
        say("| Time s | Peak deg/s | Roll, pitch | Above the bar from s, for s | Horiz. acc m/s^2 | Jerk m/s^3 | "
            f"At {a.model_lowpass:g} Hz, deg/s: measured, model, model + weight | LEFT step | RIGHT step | "
            "Lines (+-0.25 s) | In the file |")
        say("|---|---|---|---|---|---|---|---|---|---|---|")
        for q in top:
            pr = ", ".join("-" if q.get(k) is None else f"{q[k]:.3f}"
                           for k in ["meas model band"] + [f"pred {lb}" for _, lb in VARIANTS])
            say(f"| {q['t_peak'] - t0:.2f} | {q['peak_deg_s']:.3f} | {q['roll_deg_s']:+.3f}, {q['pitch_deg_s']:+.3f} | "
                f"{q['section_t0'] - t0:.2f}, {q['section_t1'] - q['section_t0']:.2f} | {q['acc_h']:.3f} | "
                f"{q['jerk_h']:.2f} | {pr} | {q['left']} | {q['right']} | {'; '.join(q['lines']) or '-'} | "
                f"{'; '.join(q['file_notes']) or '-'} |")
    say("")

    # graphs
    graphs = ["overview.png"]
    _graph(out / "overview.png", f"{folder.name}: the whole capture", sig, bar, sections, peaks, t0, sig["t"][0],
           sig["t"][-1], shown)
    for i, r in enumerate(runs, 1):
        w = r["step"]
        name = f"run_{i:02d}_{w['side']}_{re.sub(r'[^A-Za-z0-9]+', '_', w.get('name', w['label']))[:40]}.png"
        _graph(out / name, f"{w['side'].upper()} {w['label']}", sig, bar, sections, peaks, t0, w["t0"] - 1.0,
               w["t1"] + 1.0, shown)
        graphs.append(name)
    say("## Graphs\n")
    for gname in graphs:
        say(f"![{gname}]({gname})")
    (out / "report.md").write_text("\n".join(L) + "\n", encoding="utf-8")

    with open(out / "sections.csv", "w", newline="", encoding="utf-8") as fh:
        wr = csv.writer(fh)
        wr.writerow(["time_s", "peak_deg_s", "roll_deg_s", "pitch_deg_s", "above_from_s", "above_for_s", "acc_h_m_s2",
                     "jerk_h_m_s3", "meas_model_band_deg_s", "pred_model_deg_s", "pred_model_weight_deg_s", "left_step", "right_step", "lines",
                     "in_the_file"])
        for q in peaks:
            wr.writerow([f"{q['t_peak'] - t0:.3f}", f"{q['peak_deg_s']:.4f}", f"{q['roll_deg_s']:.4f}",
                         f"{q['pitch_deg_s']:.4f}", f"{q['section_t0'] - t0:.3f}", f"{q['section_t1'] - q['section_t0']:.3f}",
                         f"{q['acc_h']:.4f}", f"{q['jerk_h']:.3f}", q.get("meas model band"), q.get("pred model"),
                         q.get("pred model + weight"),
                         q["left"], q["right"], "; ".join(q["lines"]), "; ".join(q["file_notes"])])
    summary = dict(capture=str(folder), synthetic=bool(cap["meta"].get("synthetic")), rate_hz=cap["rate_hz"],
                   lost=cap["lost"], drift_ppm=cap["drift_ppm"], bias_start=list(b0), bias_end=list(b1),
                   floor_deg_s=floor, floor_from=floor_src, bar_deg_s=bar, fits=fits, model_note=why_no_model,
                   runs=[dict(side=r["step"]["side"], label=r["step"]["label"], file=r["file"].name if r["file"] else None,
                              match_deg=r["err_deg"], note=r["why"]) for r in runs],
                   sections=[dict(from_s=s["t0"] - t0, for_s=s["t1"] - s["t0"]) for s in sections],
                   peaks=[{k: (v - t0 if k in ("t_peak", "section_t0", "section_t1") else v) for k, v in q.items()}
                          for q in peaks])
    (out / "summary.json").write_text(json.dumps(summary, indent=1, default=float), encoding="utf-8")
    print("\n".join(L[:9]))
    print(f"{len(peaks)} peak(s) above the bar in {len(sections)} stretch(es); written: {out}")
    return 0


# ──────────────────────────────────────────────────────────────────────── synth (tests the analysis only)

def _wall_offset(day: Path) -> float:
    """wall - monotonic on the lab laptop that day, from Lite's 's2 feed ... recording' rows and the files' first
    frames (the row is written as the recording starts: good to a few tens of ms)."""
    offs = []
    for r in _rows(day / "hardware.jsonl"):
        args = r.get("args") or {}
        if r.get("verb") == "s2 feed" and (r.get("text") or "").startswith("recording") and isinstance(args, dict):
            span = _first_last_t(day / "canfd" / str(args.get("file")))
            if span:
                offs.append(float(r["t"]) - span[0])
    if not offs:
        raise SystemExit(f"{day}: no 's2 feed ... recording' row with its file, so the day's clocks cannot be lined up.")
    return float(np.median(offs))


def cmd_synth(a) -> int:
    _lite(a.model_src)
    if (why := model_available()) is not None:
        raise SystemExit(why)
    n = _n100()
    day = Path(a.logs).resolve()
    off = _wall_offset(day)
    feeds = read_feeds(day, -math.inf, math.inf, a.match)
    spans = [(fd["t"][0], fd["t"][-1]) for files in feeds.values() for fd in files]
    if not spans:
        raise SystemExit(f"{day}: no arm feed file matches '{a.match}'.")
    t_lo, t_hi = min(s[0] for s in spans) - a.pad, max(s[1] for s in spans) + a.pad
    grid = np.arange(t_lo, t_hi, MODEL_DT)
    poles = read_poles(day / "status.jsonl", off)
    p = predicted(feeds, poles, grid, a.model_lowpass)
    truth = p["both" if a.truth == "both" else "model"]
    rng = np.random.default_rng(a.seed)
    dt = 1.0 / a.rate
    t = np.arange(t_lo, t_hi, dt)
    lag = a.lag_ms / 1000.0
    rates_b = np.zeros((len(t), 3))
    for k, gain in enumerate((a.gain_roll, a.gain_pitch)):
        rates_b[:, k] = gain * np.interp(t - lag, grid, truth[:, k], left=0.0, right=0.0)
    bias = np.outer(np.linspace(0, 1, len(t)), np.array(a.bias_end) - np.array(a.bias_start)) + np.array(a.bias_start)
    rates_b += bias + rng.normal(0.0, a.noise, (len(t), 3))
    r = rot_rpy(tuple(float(v) for v in a.mount.split(",")))
    rates_s = rates_b @ r                                           # base -> sensor: R^T, as rows
    acc_s = (np.array([0.0, 0.0, G]) + rng.normal(0.0, 0.01, (len(t), 3))) @ r
    keep = rng.random(len(t)) >= a.drop
    folder = Path(a.out).resolve()
    folder.mkdir(parents=True, exist_ok=True)
    ts0 = 7_000_000
    made = []
    for i in np.flatnonzero(keep):
        ts_us = int(round(ts0 + (t[i] - t_lo) * (1.0 + a.drift_ppm * 1e-6) * 1e6))
        payload = struct.pack("<12fq", *(rates_s[i] / RAD), *acc_s[i], 0.0, 0.0, 0.0, 30.0, 101325.0, 30.0, ts_us)
        made.append((t[i], n.build_frame(n.T_IMU, payload, sn=int(i) & 0xFF)))
    # the reads: every 8-12 ms, each bringing the frames made at least 1 ms (plus a random wait) before it
    raw, chunks, off_b, j = bytearray(), [], 0, 0
    tr = t_lo
    while j < len(made):
        tr += rng.uniform(0.008, 0.012)
        data = bytearray()
        while j < len(made) and made[j][0] + 0.001 + rng.exponential(0.0005) <= tr:
            data += made[j][1]
            j += 1
        if data:
            raw += data
            chunks.append(dict(o=off_b, n=len(data), t=round(tr + off, 6), m=round(tr, 6)))
            off_b += len(data)
    (folder / "raw.bin").write_bytes(bytes(raw))
    (folder / "chunks.jsonl").write_text("".join(json.dumps(c) + "\n" for c in chunks))
    truth_meta = dict(gain_roll=a.gain_roll, gain_pitch=a.gain_pitch, lag_ms=a.lag_ms, drift_ppm=a.drift_ppm,
                      bias_start=a.bias_start, bias_end=a.bias_end, noise_deg_s=a.noise, drop=a.drop, truth=a.truth,
                      mount=a.mount, logs=str(day), match=a.match)
    (folder / "meta.json").write_text(json.dumps(dict(synthetic=True, tool="tools/imu_runs.py synth",
                                                      started=t_lo + off, seconds=t_hi - t_lo, truth=truth_meta),
                                                 indent=1))
    print(f"made-up capture: {len(made):,} frames, {t_hi - t_lo:.1f} s, from {sum(len(v) for v in feeds.values())} "
          f"feed file(s); {folder}")
    print(f"analyse it with: python3 tools/imu_runs.py analyse {folder} --logs {day}")
    return 0


# ──────────────────────────────────────────────────────────────────────── main

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="The IMU against the arms' runs (docs/imu/PLAN.md step F).")
    sub = ap.add_subparsers(dest="cmd", required=True)
    an = sub.add_parser("analyse", help="sections, steps, Program lines and the model check for one capture")
    an.add_argument("capture", help="the capture folder (tools/imu_n100.py capture)")
    an.add_argument("--floor", help="a step C capture (arms powered, holding a pose): sets the bar")
    an.add_argument("--logs", help="the Lite logs of the day (default: the folder above imu/)")
    an.add_argument("--programs", action="append", default=[], help="another folder of Program files (<arm>/ inside, or flat)")
    sy = sub.add_parser("synth", help="a MADE-UP capture from a day's arm feeds, to test this analysis")
    sy.add_argument("--logs", required=True, help="the Lite logs of a day with S2 feeds (canfd/)")
    sy.add_argument("--match", default="", help="only the feed files whose name holds this (e.g. 20261124_2103)")
    sy.add_argument("--out", required=True, help="the folder to write the made-up capture to (not Resources/Logs)")
    sy.add_argument("--pad", type=float, default=35.0, help="still seconds before and after the feeds")
    sy.add_argument("--rate", type=float, default=400.0)
    sy.add_argument("--gain-roll", type=float, default=0.02, help="deg of tilt per N m")
    sy.add_argument("--gain-pitch", type=float, default=-0.03)
    sy.add_argument("--lag-ms", type=float, default=60.0)
    sy.add_argument("--drift-ppm", type=float, default=30.0)
    sy.add_argument("--bias-start", type=float, nargs=3, default=[0.20, -0.10, 0.05])
    sy.add_argument("--bias-end", type=float, nargs=3, default=[0.24, -0.12, 0.05])
    sy.add_argument("--noise", type=float, default=0.05, help="deg/s RMS")
    sy.add_argument("--drop", type=float, default=0.0005, help="share of frames lost")
    sy.add_argument("--truth", choices=["both", "model"], default="both")
    sy.add_argument("--seed", type=int, default=1)
    sy.add_argument("--model-lowpass", type=float, default=20.0,
                    help="the made-up signal's band: wider than the analysis' 8 Hz, so it is filtered once, as a real one")
    an.add_argument("--model-lowpass", type=float, default=8.0)
    for s in (an, sy):
        s.add_argument("--mount", default="180,0,0", help="sensor to base: roll,pitch,yaw in degrees")
        s.add_argument("--model-src", help="a Lite checkout whose src has motion/base_load.py")
    an.add_argument("--still-s", type=float, default=30.0, help="still seconds at the start and the end (the bias)")
    an.add_argument("--lowpass", type=float, default=30.0)
    a = ap.parse_args(argv)
    return cmd_analyse(a) if a.cmd == "analyse" else cmd_synth(a)


if __name__ == "__main__":
    sys.exit(main())
