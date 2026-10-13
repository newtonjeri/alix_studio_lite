#!/usr/bin/env python3
"""Predict how a teach file will behave on the UPLOAD TRACK channel, and price the fix.

    python3 src/predict_track_replay.py data/teach_files/<file>.txt
    python3 src/predict_track_replay.py FILE --json out.json --compare right_test.txt

WHY THIS IS NOT analyze_teach.py.  `analyze_teach.py` reads a teach file as a
RECORDING: it reports the motion the operator taught, at the assumed record
period, and every speed there scales 1/T with a period nobody has measured.
This tool asks the other question — the one that decides a hardware run:

    if these bytes are uploaded verbatim and played by the controller's own
    fixed-clock point player, what is the arm actually COMMANDED to do?

That question has no unknown period in it.  `rm_send_project_t` +
`rm_set_program_id_run` hand the controller a point list which it plays at a
measured, fixed **5.5545 ms/point** (F98 lane G, three tracks of different
origin, +0.15..+0.8 %).  So the file's own point SPACING is the whole velocity
profile, the duration is (N-1) x clock, and nothing about the recording period
survives into the run.

WHAT IT CHECKS, and on what evidence.

* **Commanded joint rate**, per joint, against the xacro velocity ratings.
  Reported twice: RAW on the written integers (what the player is literally
  handed) and 20 Hz BAND-LIMITED (what the servo can physically be asked for).
  Both are needed and neither alone is honest — see [[track-rate-metrics-are-
  band-limited]].  The calibration bar is `right_test.txt`: 7 hardware replays
  through this exact channel, band-limited peak util 0.732, ZERO ticks over any
  rating.  A file far above that bar is in territory the channel has never been
  asked to enter — F98 addendum 6 is explicit that the player swallowing 14
  ONE-TICK spikes says nothing about a SUSTAINED over-rating demand.

* **Position margin** per joint, minus the 1.9 deg of tracking overshoot F98
  measured at an apex.  This is right_test's actual failure mode: J4 at 3.27 deg
  margin, roughly half of all replays stopping at the limit.

* **Step size** against the 10 deg/frame that latches error 0x4000.  The upload
  player has no 180 deg/s guard (that is a CANFD rule), but 0x4000 is a
  target-vs-motor check and applies to any commanded step.

* **The minimum-time re-parameterisation** of the SAME geometry under the
  production joint limits, via tools/plan_min_time.py (TOPP-RA + LSQ B-spline
  + the player-clock emitter).  That is the "optimal run speed": not a uniform
  slow-down, which pays everywhere for a peak that occurs in a few places, but
  a profile that rides the limit where it can.

* **Uniform slow-down**, priced alongside, so the gap between the two is visible.

ACCELERATION IS ADVISORY on this channel and this tool says so rather than
refusing: every human-taught track and several hardware-proven emitted ones read
3-10x the vendor's 10.472 rad/s^2 planning figure while running clean, and
Newton's 2026-09-07 ruling put the track accel gate at ADVISORY against a
measured 15.7 rad/s^2 bound.  Velocity and position are the gates with teeth.

Offline.  Never touches an arm.
"""
from __future__ import annotations

import argparse
import itertools
import json
import pathlib
import subprocess
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from teachlab import EXP_DIR, kinematics as K                       # noqa: E402
from teachlab.teachfile import interpretation_for, load_teach_file  # noqa: E402

REPO = pathlib.Path(__file__).resolve().parents[3]
CLOCK_S = 0.0055545          # the online-program player's measured fixed clock (F98 lane G)
BAND_HZ = 20.0               # the basis every production rate figure already uses
OVERSHOOT_DEG = 1.9          # tracking overshoot at an apex, measured (F98 addendum 5)
STEP_FAULT_DEG = 10.0        # 0x4000 "Position Command Step Jump"
CALIBRATION = "right_test.txt"


def _bandlimit(q: np.ndarray, clock: float, fc_hz: float = BAND_HZ) -> np.ndarray:
    from scipy.signal import butter, filtfilt
    b, a = butter(4, min(fc_hz / (0.5 / clock), 0.99), "low")
    return filtfilt(b, a, q, axis=0)


def _runs(mask: np.ndarray) -> list:
    return [sum(1 for _ in g) for k, g in itertools.groupby(mask.tolist()) if k]


def rate_report(q_deg: np.ndarray, clock: float, vmax: np.ndarray) -> dict:
    """Commanded joint rate at the player clock, raw and band-limited."""
    out = {}
    for tag, qq in (("raw", q_deg), ("band_limited_20hz", _bandlimit(q_deg, clock))):
        v = np.abs(np.diff(qq, axis=0)) / clock
        u = v / vmax
        umax = u.max(axis=1)
        r = _runs(umax > 1.0)
        out[tag] = {
            "per_joint_peak_deg_s": [float(x) for x in v.max(axis=0)],
            "per_joint_util": [float(x) for x in v.max(axis=0) / vmax],
            "peak_util": float(umax.max()),
            "peak_joint": int(u.max(axis=0).argmax()) + 1,
            "util_p50": float(np.percentile(umax, 50)),
            "util_p90": float(np.percentile(umax, 90)),
            "util_p99": float(np.percentile(umax, 99)),
            "ticks_over_rating": int((umax > 1.0).sum()),
            "ticks_total": int(umax.size),
            "pct_over_rating": float(100.0 * (umax > 1.0).mean()),
            "longest_over_run_ticks": int(max(r, default=0)),
            "longest_over_run_ms": float(max(r, default=0) * clock * 1e3),
        }
    return out


def position_report(q_deg: np.ndarray, limits) -> dict:
    joints = []
    for i, (lo, hi) in enumerate(limits):
        margin = float(min(q_deg[:, i].min() - lo, hi - q_deg[:, i].max()))
        joints.append({"joint": i + 1, "min_deg": float(q_deg[:, i].min()),
                       "max_deg": float(q_deg[:, i].max()), "limit_deg": float(hi),
                       "margin_deg": margin,
                       "margin_after_overshoot_deg": margin - OVERSHOOT_DEG})
    binding = min(joints, key=lambda j: j["margin_deg"])
    return {"per_joint": joints, "binding_joint": binding["joint"],
            "binding_margin_deg": binding["margin_deg"],
            "overshoot_allowance_deg": OVERSHOOT_DEG}


def step_report(q_deg: np.ndarray) -> dict:
    d = np.abs(np.diff(q_deg, axis=0))
    return {"max_step_deg": float(d.max()),
            "per_joint_max_step_deg": [float(x) for x in d.max(axis=0)],
            "fault_threshold_deg": STEP_FAULT_DEG,
            "ticks_over_fault": int((d > STEP_FAULT_DEG).any(axis=1).sum())}


def uniform_ladder(q_deg: np.ndarray, clock: float, vmax: np.ndarray, targets=(1.0, 0.9, 0.8)) -> list:
    """What a single --speed-style factor costs.  Priced for contrast only: it
    pays for the worst tick everywhere along the file."""
    n = q_deg.shape[0]
    peak = float((np.abs(np.diff(q_deg, axis=0)) / clock / vmax).max())
    rows = []
    for t in targets:
        k = peak / t
        rows.append({"target_util": t, "factor": float(k),
                     "points": int(round((n - 1) * k)) + 1,
                     "duration_s": float((n - 1) * clock * k)})
    return rows


def velocity_floor(q_deg: np.ndarray, vmax: np.ndarray) -> float:
    """Exact velocity-limited minimum duration for this waypoint polyline:
    every interval takes at least max_j(|dq_j| / vmax_j).  Ignores acceleration,
    so it is a floor the real optimum never reaches."""
    return float((np.abs(np.diff(q_deg, axis=0)) / vmax).max(axis=1).sum())


def toppra_ladder(q_deg: np.ndarray, side: str, out_dir: pathlib.Path, rungs, ik_frame: str,
                  max_acc_util: float, source: pathlib.Path, source_hardware: str,
                  source_sha: str) -> list:
    """Drive tools/plan_min_time.py — the proven TOPP-RA + player-clock emitter.

    It reads a plan, so the teach file is wrapped as a one-stage plan whose
    waypoint times are the player clock.  Nothing about the parameterisation is
    reimplemented here.

    PROVENANCE.  Wrapping a teach file as a plan makes plan_min_time stamp the
    emitted sidecar `source_kind: ros2_plan`, `source_hardware: ok`, "generated
    by our task node against our description" — three claims that are false for
    a recording, and the last of which would let a file taught on ANOTHER ARM
    MOUNT become streamable.  That is precisely the laundering respace_teach.py
    was given a refusal for.  So: refuse an `analysis_only` source outright, and
    rewrite the sidecar to name the teach file it actually came from."""
    if source_hardware == "analysis_only":
        raise SystemExit(
            f"REFUSED: {source.name} is hardware: analysis_only — recorded under another arm "
            f"mount. Emitting a track from it would produce a file whose sidecar claims "
            f"source_hardware: ok. Analysis is fine; a runnable track is not.")
    out_dir.mkdir(parents=True, exist_ok=True)
    plan = out_dir / "teach_as_plan.json"
    qr = np.radians(q_deg)
    plan.write_text(json.dumps({"sub_trajectories": [{
        "stage_name": "execute_path",
        "joint_names": [f"{side}_joint{i}" for i in range(1, 8)],
        "waypoints": [{"positions": list(map(float, r)), "time_from_start": i * CLOCK_S}
                      for i, r in enumerate(qr)]}]}))
    rows = []
    for vs in rungs:
        track = out_dir / f"track_vel{vs:.2f}.txt"
        js = out_dir / f"min_time_vel{vs:.2f}.json"
        cmd = [sys.executable, str(REPO / "tools/plan_min_time.py"), str(plan),
               "--stage", "execute_path", "--side", side, "--ik-frame", ik_frame,
               "--vel-scale", f"{vs:g}", "--max-acc-util", f"{max_acc_util:g}",
               "--emit-track", str(track), "--json", str(js)]
        p = subprocess.run(cmd, capture_output=True, text=True)
        row = {"vel_scale": vs, "returncode": p.returncode, "track": str(track)}
        if js.is_file():
            row["min_time"] = json.loads(js.read_text())
        side_car = track.with_suffix(".json")
        if side_car.is_file():
            d = json.loads(side_car.read_text())
            d["source"] = str(source)
            d["source_sha256"] = source_sha
            d["source_kind"] = "teach_file"
            d["source_hardware"] = source_hardware
            d["why_ok"] = (f"re-parameterised from the teach file {source.name}, whose MANIFEST "
                           f"entry reads hardware: {source_hardware}; the wrapper plan is scaffolding, "
                           f"not a task-node plan")
            side_car.write_text(json.dumps(d, indent=2))
            row["track_report"] = d
        row["stdout_tail"] = p.stdout.strip().splitlines()[-14:]
        rows.append(row)
    return rows


def analyse(path: pathlib.Path, args) -> dict:
    interp = interpretation_for(path, overrides={"side": args.side, "units": args.units})
    side = (interp.get("side") or "L").upper()[0]
    tf = load_teach_file(path, units=interp.get("units") or "millideg", side=side,
                         limits_deg=K.JOINT_LIMITS_DEG)
    q = tf.q_deg
    vmax = np.array(K.QDOT_MAX_DEG_S)
    rep = {
        "schema": "exp01.track_replay_prediction/1",
        "source": {"file": path.name, "sha256": tf.sha256, "samples": int(q.shape[0]),
                   "side": side, "units": tf.units,
                   "hardware": interp.get("hardware", "unknown"),
                   "in_manifest": bool(interp.get("in_manifest"))},
        "channel": {"transport": "rm_send_project + rm_set_program_id_run",
                    "clock_s": CLOCK_S,
                    "as_authored_duration_s": float((q.shape[0] - 1) * CLOCK_S),
                    "clock_basis": "F98 lane G, measured on three tracks of different origin"},
        "limits": {"velocity_deg_s": [float(x) for x in vmax],
                   "acceleration_deg_s2": float(K.QDDOT_MAX_DEG_S2),
                   "position_deg": [[float(a), float(b)] for a, b in K.JOINT_LIMITS_DEG]},
        "rate": rate_report(q, CLOCK_S, vmax),
        "position": position_report(q, K.JOINT_LIMITS_DEG),
        "step": step_report(q),
        "velocity_floor_s": velocity_floor(q, vmax),
        "uniform_ladder": uniform_ladder(q, CLOCK_S, vmax),
    }
    if args.compare:
        cpath = path.parent / args.compare
        if cpath.is_file():
            ci = interpretation_for(cpath)
            ctf = load_teach_file(cpath, units=ci.get("units") or "millideg",
                                  side=(ci.get("side") or "R").upper()[0],
                                  limits_deg=K.JOINT_LIMITS_DEG)
            rep["calibration"] = {"file": cpath.name, "samples": int(ctf.q_deg.shape[0]),
                                  "duration_s": float((ctf.q_deg.shape[0] - 1) * CLOCK_S),
                                  "rate": rate_report(ctf.q_deg, CLOCK_S, vmax),
                                  "position": position_report(ctf.q_deg, K.JOINT_LIMITS_DEG),
                                  "why": "the only teach file with hardware replay evidence on "
                                         "this channel (7 attempts, 3 complete; every abort a "
                                         "J4 position stop, never a rate refusal)"}
    if not args.no_toppra:
        rep["optimal"] = toppra_ladder(q, side, args.work / path.stem, args.vel_scales,
                                       args.ik_frame or f"{side}_glove_frame_2", args.max_acc_util,
                                       path, rep["source"]["hardware"], tf.sha256)
    return rep


def render(rep: dict) -> str:
    L = []
    s, ch, r = rep["source"], rep["channel"], rep["rate"]
    L.append(f"{s['file']}  {s['samples']} points  ->  {ch['as_authored_duration_s']:.3f} s "
             f"at the player's fixed {ch['clock_s']*1e3:.4f} ms/point")
    L.append("")
    L.append("COMMANDED JOINT RATE")
    L.append(f"  {'basis':<20} {'p50':>6} {'p90':>6} {'peak':>6} {'joint':>6} {'deg/s':>8} "
             f"{'over':>7} {'%':>6} {'longest':>9}")
    for tag in ("raw", "band_limited_20hz"):
        d = r[tag]
        L.append(f"  {tag:<20} {d['util_p50']:6.3f} {d['util_p90']:6.3f} {d['peak_util']:6.3f} "
                 f"{'J'+str(d['peak_joint']):>6} {max(d['per_joint_peak_deg_s']):8.1f} "
                 f"{d['ticks_over_rating']:7d} {d['pct_over_rating']:6.2f} "
                 f"{d['longest_over_run_ms']:7.0f}ms")
    if "calibration" in rep:
        c = rep["calibration"]
        L.append(f"  -- calibration: {c['file']} (hardware-proven on this channel) --")
        for tag in ("raw", "band_limited_20hz"):
            d = c["rate"][tag]
            L.append(f"  {tag:<20} {d['util_p50']:6.3f} {d['util_p90']:6.3f} {d['peak_util']:6.3f} "
                     f"{'J'+str(d['peak_joint']):>6} {max(d['per_joint_peak_deg_s']):8.1f} "
                     f"{d['ticks_over_rating']:7d} {d['pct_over_rating']:6.2f} "
                     f"{d['longest_over_run_ms']:7.0f}ms")
    L.append("")
    L.append("POSITION MARGIN (right_test's actual failure mode)")
    for j in rep["position"]["per_joint"]:
        mark = "   <-- binding" if j["joint"] == rep["position"]["binding_joint"] else ""
        L.append(f"  J{j['joint']}  {j['min_deg']:9.3f} .. {j['max_deg']:8.3f}  of +-{j['limit_deg']:.0f}"
                 f"   margin {j['margin_deg']:8.3f}  after {OVERSHOOT_DEG} deg overshoot "
                 f"{j['margin_after_overshoot_deg']:8.3f}{mark}")
    L.append("")
    L.append(f"LARGEST COMMANDED STEP {rep['step']['max_step_deg']:.3f} deg "
             f"({rep['step']['ticks_over_fault']} over the {STEP_FAULT_DEG:.0f} deg 0x4000 threshold)")
    L.append("")
    L.append(f"OPTIMAL RUN SPEED  (velocity-only floor {rep['velocity_floor_s']:.2f} s — "
             f"ignores acceleration, never reachable)")
    L.append(f"  {'rung':<12} {'pts':>7} {'duration':>9} {'x taught':>9} {'u_vel':>6} {'u_acc':>6} "
             f"{'peak deg/s':>11} {'>180':>6} {'J6 margin':>10}")
    authored = rep["channel"]["as_authored_duration_s"]
    for row in rep.get("optimal", []):
        tr = row.get("track_report")
        if not tr:
            L.append(f"  vel {row['vel_scale']:.2f}: " + " ".join(row.get("stdout_tail", [])[-1:]))
            continue
        b = tr["band_limited_util"]
        L.append(f"  vel {row['vel_scale']:.2f}     {tr['points']:7d} {tr['track_duration_s']:8.3f}s "
                 f"{authored/tr['track_duration_s']:9.2f} {b['vel']:6.3f} {b['acc']:6.3f} "
                 f"{tr['max_joint_rate_deg_s']:11.1f} {tr['ticks_over_180_deg_s']:6d} "
                 f"{tr['position_margin_min_deg']:9.2f}d")
    L.append("")
    L.append("UNIFORM SLOW-DOWN, for contrast (pays the worst tick everywhere)")
    for u in rep["uniform_ladder"]:
        L.append(f"  to util {u['target_util']:.2f} everywhere: x{u['factor']:.2f} -> "
                 f"{u['duration_s']:.2f} s, {u['points']} pts")
    return "\n".join(L)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", type=pathlib.Path)
    ap.add_argument("--side", default=None)
    ap.add_argument("--units", default=None)
    ap.add_argument("--ik-frame", default=None, help="tool frame for the Cartesian report "
                                                     "(default <side>_glove_frame_2)")
    ap.add_argument("--compare", default=CALIBRATION,
                    help=f"teach file to use as the hardware-proven bar (default {CALIBRATION}); "
                         f"'' to skip")
    ap.add_argument("--vel-scales", type=float, nargs="*", default=[1.0, 0.8, 0.7])
    ap.add_argument("--max-acc-util", type=float, default=1.5,
                    help="accept acceleration up to this multiple of the 10.472 rad/s^2 planning "
                         "figure; 1.5 is the measured 15.7 rad/s^2 bound (Newton, 2026-09-07)")
    ap.add_argument("--no-toppra", action="store_true", help="skip the re-parameterisation ladder")
    ap.add_argument("--work", type=pathlib.Path, default=EXP_DIR / "generated" / "track_replay")
    ap.add_argument("--json", type=pathlib.Path, default=None)
    args = ap.parse_args(argv)

    rep = analyse(args.file, args)
    print(render(rep))
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(rep, indent=2))
        print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
