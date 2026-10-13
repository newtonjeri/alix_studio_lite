#!/usr/bin/env python3
"""track_report.py — analyse upload-channel tracks and recorded runs, OFFLINE.

STANDALONE AND INERT BY CONSTRUCTION (Newton, 2026-09-01: "without interfering
with the execution program"). This tool reads FILES only:

  * it never imports the SDK, never opens a controller connection, never binds
    the UDP port and never changes the global speed ratio — so it is safe to
    run on the robot host WHILE a run is in progress, or afterwards, or on a
    laptop with no robot at all;
  * `--assert-inert` (default on) fails loudly if `Robotic_Arm` ever ends up
    imported, so the guarantee cannot rot silently.

Three questions, three modes:

    # What SHOULD this track do on the arm? (before you run it)
    tools/track_report.py predict 70
    tools/track_report.py predict Resource/tracks/commode_c/70_*.txt

    # What DID this run actually do? (after --record-joints)
    tools/track_report.py review Resource/runs/tracks/2026-09-01/run_slot70_190758.json

    # Commanded vs executed, per joint (where did the arm fall behind?)
    tools/track_report.py compare Resource/runs/tracks/2026-09-01/run_slot70_190758.json

    # Every run in the ledger, one line each
    tools/track_report.py ledger

Clock caveat carried everywhere (F108b): the UPLOAD player's tick is NOMINALLY
5.5545 ms but measured 5.51-6.02 ms/pt, +-6% run to run on the same track, so
every predicted duration is a RANGE over the hardware envelope, never a value.
A track's OWN sample clock is a different number: its sidecar's `clock_s`
(`track_clock_s`) — 5.5545 ms for every track emitted before 2026-10-02 and
5.0 ms (the CANFD stream period, Newton 2026-10-02) after. The file's nominal
duration and its commanded rates are read at THAT clock; the upload player's
pace (CLOCK_S below) is used only where upload playback is being judged.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "core/python/alix_dispatch/src"))
sys.path.insert(0, str(REPO / "tools"))

FIXTURE = "commode_c"
TRACKS = REPO / "Resource/tracks" / FIXTURE
RUNS = REPO / "Resource/runs/tracks"
#: ONE AUTHORITY. These were local literals — a third and fourth copy of the
#: same two numbers, so editing the envelope in `track.py` left this file
#: reporting the old one. `sys.path` already carries alix_dispatch (:41).
from alix_dispatch.track import (                                    # noqa: E402
    CLOCK_ENVELOPE_MS as ENVELOPE_MS, REPLAY_CLOCK_S as CLOCK_S)

#: Frame diagnostic scaling. The wire carries integers; the header claims
#: 0.001 units and the values are consistent with milli-units (temperature
#: 32000 -> 32.0 C, voltage 21000 -> 21.0 V, current -605000 -> -0.605 A if
#: microamps). NOTE the standing finding that these arrive INTEGER-QUANTISED
#: regardless of the claimed resolution — treat as indicative, not metrology.
DIAG_SCALE = {"joint_temperature": (1e-3, "C"), "joint_voltage": (1e-3, "V"),
              "joint_current": (1e-6, "A")}

#: UDP wire scales, pinned empirically against the captures (2026-09-01):
#:   joint_position  millidegrees      (known)
#:   joint_speed     0.01 deg/s        (ratio 100.6 vs a 50 ms smoothed
#:                                      derivative, 2.8% median error; a RAW
#:                                      finite difference reads ~16% high and
#:                                      suggested a bogus x86)
#:   waypoint.position  micrometres    (path sums to metres; base-frame TCP)
#:   waypoint.quat      x1e6           (norm is exactly 1e6 => unit quaternion)
#: six_force_sensor units are NOT verified — reported raw.
JOINT_SPEED_SCALE = 0.01          # -> deg/s
TCP_POS_SCALE = 1e-6              # -> m
QUAT_SCALE = 1e-6


def _np():
    import numpy as np
    return np


def assert_inert() -> None:
    """The safety property this tool exists to keep: no SDK, ever."""
    bad = [m for m in sys.modules if m.startswith("Robotic_Arm")]
    if bad:
        raise SystemExit(f"track_report is meant to be inert but the SDK is loaded: {bad}")


# --------------------------------------------------------------- loading
def resolve_track(token: str) -> pathlib.Path:
    """Slot number, region key or path -> the track file."""
    p = pathlib.Path(token)
    if p.is_file():
        return p
    man_path = TRACKS / "MANIFEST.json"
    if man_path.is_file():
        man = json.loads(man_path.read_text())
        for t in man["tasks"]:
            if t.get("status") != "EMITTED":
                continue
            if (token.isdigit() and t.get("slot") == int(token)) or token in (t.get("name") or ""):
                return REPO / t["track"]
    raise SystemExit(f"no track for {token!r} (try a slot number, a region key, or a path)")


def load_track(path: pathlib.Path):
    np = _np()
    rows = [json.loads(l)["point"] for l in path.open() if l.strip()]
    return np.asarray(rows, float) / 1000.0          # degrees


def track_clock_s(path, sha: str = None) -> tuple:
    """(clock_s, source) — the clock the track file was SAMPLED at.

    It is the sidecar's `clock_s` (beside the .txt; every emitter writes it). It
    was the hard-coded 5.5545 ms here until 2026-10-02, when the planner moved to
    the 5.0 ms CANFD period (Newton: "the planner should emit at 5ms"), so a
    5.0 ms file would have read 11 % slow. With `sha` (a run record's
    `track_sha256`) the sidecar is believed only while the file still hashes to
    it — a re-emit replaces both. Fallback: the upload player's REPLAY_CLOCK_S,
    the clock every track was emitted at before 2026-10-02.
    """
    import hashlib
    p = pathlib.Path(path) if path else None
    if p is not None and p.is_file():
        if sha and hashlib.sha256(p.read_bytes()).hexdigest() != sha:
            return CLOCK_S, "the player's 5.5545 (the file on disk is not the run's bytes)"
        sc = p.with_suffix(".json")
        try:
            c = json.loads(sc.read_text()).get("clock_s")
            if c:
                return float(c), f"{sc.name} clock_s"
        except (OSError, ValueError, AttributeError):
            pass
    return CLOCK_S, "the player's 5.5545 (no sidecar clock_s)"


def load_frames(frames_path: pathlib.Path):
    """(t, q_deg, raw_frames) from a --record-joints capture."""
    np = _np()
    ts, qs, raw = [], [], []
    for line in frames_path.open():
        try:
            d = json.loads(line)
            fr = d["frame"]
            jp = fr["joint_status"]["joint_position"]
        except Exception:                                              # noqa: BLE001
            continue
        if len(jp) != 7:
            continue
        ts.append(d["t_mono"])
        qs.append([v / 1000.0 for v in jp])
        raw.append(fr)
    return np.asarray(ts), np.asarray(qs), raw


# --------------------------------------------------------------- predict
def cmd_predict(args) -> int:
    np = _np()
    path = resolve_track(args.track)
    q = load_track(path)
    n = len(q)
    clk, clk_src = track_clock_s(path)                     # the file's OWN sample clock
    print(f"TRACK {path.name}")
    print(f"  {n} points | duration {(n-1)*clk:.3f} s nominal at its own {clk*1e3:g} ms clock "
          f"({clk_src}); on the UPLOAD player {(n-1)*ENVELOPE_MS[0]/1e3:.3f}-{(n-1)*ENVELOPE_MS[1]/1e3:.3f} s "
          f"over the hardware envelope ({ENVELOPE_MS[0]}-{ENVELOPE_MS[1]} ms/pt, F108b)")

    v = np.diff(q, axis=0) / clk                           # deg/s, commanded at the file's clock
    a = np.diff(v, axis=0) / clk                           # deg/s^2
    print("\n  COMMANDED MOTION (raw, per joint)")
    print(f"    {'joint':>6} {'peak |v|':>10} {'peak |a|':>12} {'range':>22}")
    for j in range(q.shape[1]):
        print(f"    {'J'+str(j+1):>6} {np.abs(v[:, j]).max():9.1f}° "
              f"{np.abs(a[:, j]).max():10.0f}°/s² "
              f"[{q[:, j].min():8.1f}, {q[:, j].max():7.1f}]°")
    peak = np.abs(v).max()
    over = int((np.abs(v) > 180).sum())
    print(f"    peak rate {peak:.1f}°/s"
          + (f" — {over} tick-samples over 180°/s: fine for the PLAYER (no rate "
             f"limit) but NOT CANFD-streamable" if over else " — under 180°/s everywhere"))

    # the one-tick self-consistency check the emitter gates on
    step = float(np.abs(np.diff(v, axis=0)).max())
    print(f"    one-tick speed step {step:.2f}°/s "
          f"({'OK' if step <= 10 else '** ABOVE the 10°/s emitter gate'})")

    print("\n  LIMIT + SINGULARITY PROXIMITY")
    try:
        import plan_min_time as pmt
        side = "R" if "_right" in path.name else "L"
        margins = pmt.check_positions(np.radians(q), side)
        worst = int(np.argmin(margins))
        print(f"    joint-position margin min {min(margins):.2f}° on J{worst+1} "
              f"({'OK' if min(margins) >= 5 else '** under the 5° emit gate'})")
    except Exception as exc:                                           # noqa: BLE001
        print(f"    (position margins unavailable: {exc})")
    j4 = q[:, 3]
    cross = int((np.abs(j4) >= 135).sum())
    print(f"    J4 range [{j4.min():.1f}, {j4.max():.1f}]° | samples at/over ±135°: {cross}"
          + ("  — F103 abort class CANNOT fire" if cross == 0 else
             "  ** F103: aborts depend on the approach RATE at the crossing"))

    # --- motion parameters -------------------------------------------------
    jerk = np.diff(a, axis=0) / clk
    print("\n  MOTION PARAMETERS (commanded)")
    print(f"    joint jerk peak {np.abs(jerk).max():.0f}°/s³ "
          f"(TOPP-RA optima are bang-bang in acceleration, so sampled jerk is "
          f"~2a/dt and is NOT a meaningful bound — F99d)")
    side = "R" if "_right" in path.name else "L"
    sidecar = path.with_suffix(".json")
    if sidecar.is_file():
        sc = json.loads(sidecar.read_text())
        u = sc.get("band_limited_util") or {}
        print(f"    emitter utilisation @{u.get('fc_hz', 20)} Hz band-limited: "
              f"vel {u.get('vel', float('nan')):.3f}  acc {u.get('acc', float('nan')):.3f}  "
              f"jerk {u.get('jerk', float('nan')):.3f}  (of joint_limits.yaml; >1 acc is the "
              f"bang-bang readback overshoot, tolerated to 1.30)")
        tsp = (sc.get("tool_speed_on_contact_stages") or {}).get("execute_path")
        if tsp:
            lin, ang = tsp["linear_mps"], tsp["angular_rad_s"]
            print(f"    TCP linear  p50 {lin['p50']:.3f} m/s   max {lin['max']:.3f} m/s")
            print(f"    TCP angular p50 {ang['p50']:.3f} rad/s max {ang['max']:.3f} rad/s")
            print(f"    ik_frame {sc.get('frame') or sc.get('ik_frame') or '?'} | "
                  f"emitted at acc-scale {sc.get('acc_scale', 1.0)}")
    else:
        print("    (no sidecar beside the track — TCP speed unavailable offline; "
              "re-emit to regenerate it)")

    if args.hotspots:
        print("\n  HOTSPOTS (fastest tenth of the path)")
        speed = np.abs(v).max(axis=1)
        idx = np.argsort(speed)[-max(1, n // 200):][::-1][:args.hotspots]
        for i in sorted(idx):
            print(f"    pt {i:>5} ({100*i/(n-1):5.1f}% of path, t≈{i*clk:6.2f}s): "
                  f"{speed[i]:6.1f}°/s on J{int(np.argmax(np.abs(v[i])))+1}")
    return 0


# ---------------------------------------------------------------- review
def _diag_block(raw, np):
    """Per-joint diagnostics carried by the UDP frames, which nothing else reads."""
    out = {}
    for key, (scale, unit) in DIAG_SCALE.items():
        vals = [f["joint_status"].get(key) for f in raw if f.get("joint_status", {}).get(key)]
        vals = [v for v in vals if v and len(v) == 7]
        if not vals:
            continue
        arr = np.asarray(vals, float) * scale
        out[key] = (arr, unit)
    return out


def executed_motion(raw, ts, np):
    """Executed motion parameters straight off the wire — no FK needed: the
    controller reports its OWN TCP pose (`waypoint`) and joint velocities."""
    out = {}
    sp = [f["joint_status"].get("joint_speed") for f in raw]
    sp = np.asarray([v for v in sp if v and len(v) == 7], float) * JOINT_SPEED_SCALE
    if len(sp):
        out["joint_speed_deg_s"] = sp
    pos = [(f.get("waypoint") or {}).get("position") for f in raw]
    quat = [(f.get("waypoint") or {}).get("quat") for f in raw]
    ok = [i for i, (p_, q_) in enumerate(zip(pos, quat))
          if p_ and q_ and len(p_) == 3 and len(q_) == 4]
    if len(ok) > 30:
        t = ts[ok]
        P = np.asarray([pos[i] for i in ok], float) * TCP_POS_SCALE
        Q = np.asarray([quat[i] for i in ok], float) * QUAT_SCALE
        # SMOOTHED over ~50 ms. Differencing raw 5 ms samples inflates the
        # tails ~2x (measured: TCP p99 0.950 raw vs 0.508 at 50 ms, against a
        # commanded max of 0.499) because the timestamp is RECEIPT time and
        # jitters +-0.5 ms on a 5 ms interval. 50 ms is the window that
        # reproduced the controller's own joint_speed to 2.8%.
        k = max(1, int(round(0.050 / max(np.median(np.diff(t)), 1e-4))))
        out["tcp_window_ms"] = float(np.median(np.diff(t)) * k * 1e3)
        out["tcp_linear_mps"] = np.linalg.norm(P[k:] - P[:-k], axis=1) / (t[k:] - t[:-k])
        dot = np.abs(np.sum(Q[k:] * Q[:-k], axis=1)).clip(0, 1)
        out["tcp_angular_rad_s"] = 2.0 * np.arccos(dot) / (t[k:] - t[:-k])
        out["tcp_path_m"] = float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())
    ff = [(f.get("six_force_sensor") or {}).get("force") for f in raw]
    ff = np.asarray([v for v in ff if v and len(v) == 6], float)
    if len(ff):
        out["force_raw"] = ff
    return out


def cmd_review(args) -> int:
    np = _np()
    rec_path = pathlib.Path(args.record)
    rec = json.loads(rec_path.read_text())
    frames_path = rec_path.with_name(rec_path.stem + "_frames.ndjson")
    print(f"RUN slot {rec['slot']} — {rec['name']}  ({rec.get('when', '?')})")
    print(f"  speed {rec.get('speed')} | gap {rec.get('gap_deg', float('nan')):.1f}° | "
          f"verdict {rec.get('verdict')} | frames {rec.get('frames')}")
    if rec.get("split_source"):
        print(f"  split: {rec['split_source']}")
    if rec.get("movej_s") is not None:
        print(f"  movej {rec['movej_s']:.2f} s + dwell {rec['dwell_s']:.2f} s + "
              f"REPLAY {rec['replay_s']:.3f} s -> {rec['implied_ms_per_pt']:.4f} ms/pt "
              f"({'inside' if ENVELOPE_MS[0] <= rec['implied_ms_per_pt'] <= ENVELOPE_MS[1] else '** OUTSIDE'}"
              f" the {ENVELOPE_MS[0]}-{ENVELOPE_MS[1]} envelope)")
    if not frames_path.is_file():
        print("  (no capture — run with --record-joints for the rest)")
        return 0

    ts, q, raw = load_frames(frames_path)
    t0 = ts[0] + rec.get("movej_s", 0.0) + rec.get("dwell_s", 0.0)
    t1 = t0 + rec.get("replay_s", ts[-1] - t0)
    m = (ts >= t0) & (ts <= t1)
    print(f"\n  CAPTURE: {len(ts)} frames, {len(raw)} decoded | replay window {m.sum()} frames")
    dt = np.diff(ts[m]) * 1e3
    print(f"  push cadence median {np.median(dt):.3f} ms, p99 {np.percentile(dt, 99):.3f}, "
          f"max {dt.max():.1f} ({'steady' if np.median(dt) < 5.2 else 'CHECK'})")

    rc = rec.get("replay_check")
    if rc:
        print(f"\n  COVERAGE  {rc['phase_first']*100:.1f}% -> {rc['phase_last']*100:.1f}% "
              f"({'COMPLETE' if rc['phase_first'] <= 0.01 and rc['phase_last'] >= 0.99 else '** INCOMPLETE'})"
              f" | monotone {rc['monotone_frac']*100:.0f}%")
        print(f"  FIDELITY  path deviation p50 {rc['dev_p50_deg']:.2f}° "
              f"p99 {rc['dev_p99_deg']:.2f}° max {rc['dev_max_deg']:.2f}°"
              f"  (mostly servo LAG at speed, not path error — unseparated)")

    print("\n  EXECUTED MOTION (replay window)")
    tw, qw = ts[m], q[m]
    print(f"    J4 range [{qw[:, 3].min():.1f}, {qw[:, 3].max():.1f}]°, "
          f"{int((np.abs(qw[:, 3]) >= 135).sum())} samples over ±135°  "
          f"(joint rates below come from the controller's own field, not from "
          f"differencing positions — that reads ~2x high at 5 ms)")

    raww = [r for r, keep in zip(raw, m) if keep]
    em = executed_motion(raww, tw, np)
    print("\n  MOTION PARAMETERS (executed, from the controller's own fields)")
    print("    NB the controller reports `waypoint` at ITS configured tool frame, which "
          "we neither set nor read;\n        the COMMANDED figure in `predict` is at our "
          "ik_frame. They agreed within 5-8% on both\n        2026-09-01 slots, which is "
          "evidence they are the same point — not proof.")
    if "tcp_linear_mps" in em:
        lin, ang = em["tcp_linear_mps"], em["tcp_angular_rad_s"]
        print(f"    TCP linear  p50 {np.median(lin):.3f}  p99 {np.percentile(lin,99):.3f}  "
              f"max {lin.max():.3f} m/s   | TCP path {em['tcp_path_m']:.2f} m "
              f"[{em['tcp_window_ms']:.0f} ms window]")
        print(f"    TCP angular p50 {np.median(ang):.3f}  p99 {np.percentile(ang,99):.3f}  "
              f"max {ang.max():.3f} rad/s")
    if "joint_speed_deg_s" in em:
        js = np.abs(em["joint_speed_deg_s"])
        print(f"    joint speed (controller) p99 {np.percentile(js,99):.1f}  max {js.max():.1f} °/s"
              f" | per-joint max " + " ".join(f"J{j+1} {js[:,j].max():.0f}" for j in range(7)))
        acc = np.diff(em["joint_speed_deg_s"], axis=0) / np.diff(tw)[:len(em["joint_speed_deg_s"])-1, None]
        print(f"    joint accel from it: p99 {np.percentile(np.abs(acc),99):.0f}  "
              f"max {np.abs(acc).max():.0f} °/s²")
    if "force_raw" in em:
        fr = em["force_raw"]
        print("    wrist force (RAW wire units, unverified): per-axis max "
              + " ".join(f"{fr[:,k].max():.0f}" for k in range(6)))
    diags = _diag_block(raww, np)
    if diags:
        print("\n  JOINT DIAGNOSTICS (integer-quantised on the wire — indicative)")
        for key, (arr, unit) in diags.items():
            per_joint = " ".join(f"{arr[:, j].max():.2f}" for j in range(arr.shape[1]))
            print(f"    {key.replace('joint_', 'peak |'):>18}| {unit}: {per_joint}"
                  if key == "joint_current" else
                  f"    {key.replace('joint_', ''):>18} {unit}: max per joint {per_joint}")
    errs = {tuple(f.get("err") or []) for f in raw}
    states = {f.get("arm_current_status") for f in raw}
    jerr = {tuple(f["joint_status"].get("joint_err_code", [])) for f in raw if f.get("joint_status")}
    print(f"\n  HEALTH: arm status seen {sorted(x for x in states if x)} | "
          f"controller err codes {sorted(errs)} (the push latches the run's FIRST code only) "
          f"| joint err codes {'all zero' if jerr <= {tuple([0]*7)} else sorted(jerr)}")
    # ---- F111: did the program run its file out, or end early? ----------------
    pts = rec.get("points")
    rep = rec.get("replay_s")
    if pts and rep:
        # The file's nominal is at ITS OWN sample clock (the hard-coded 5.5545 until 2026-10-02);
        # F111's early-end flag judges the UPLOAD PLAYER, which plays any file at its own ~5.5545 ms
        # pace, so it stays on that pace (identical to before for every 5.5545 ms file).
        clk, clk_src = track_clock_s(REPO / rec["track"] if rec.get("track") else None,
                                     rec.get("track_sha256"))
        nominal = (pts - 1) * clk
        frac = rep / nominal
        player_frac = rep / ((pts - 1) * CLOCK_S)
        cov = (rec.get("replay_check") or {}).get("phase_last")
        print(f"\n  FILE TIME: replay {rep:.2f} s = {frac * 100:.0f}% of the file's nominal "
              f"{nominal:.2f} s ({pts} pts x {clk * 1e3:g} ms, {clk_src})"
              + (f" | arm reached {cov * 100:.1f}% of the path" if cov is not None else "")
              + ("  ** PROGRAM ENDED BEFORE THE FILE — the controller stopped it early"
                 if player_frac < 0.9 else
                 ("  ** file ran out with the arm behind" if (cov is not None and cov < 0.99) else "")))
    # ---- devices: what the controller pushed, and what the run commanded ------
    lift = [f.get("lift_state", {}).get("height") for f in raw if isinstance(f.get("lift_state"), dict)]
    lift = [x for x in lift if x is not None]
    hands = {json.dumps(f.get("hand"), sort_keys=True) for f in raw if f.get("hand") is not None}
    print("\n  DEVICES (controller push during the capture)")
    if lift:
        changes = sum(1 for a, b in zip(lift, lift[1:]) if a != b)
        print(f"    pole height: first {lift[0]} mm, last {lift[-1]} mm, range [{min(lift)}, {max(lift)}] mm, "
              f"{changes} changes  -> {'MOVED' if max(lift) - min(lift) > 2 else 'did not move'}")
    zero_hand = (len(hands) == 1 and all(
        not any(v) if isinstance(v, list) else not v
        for v in (json.loads(next(iter(hands))) or {}).values()))
    print(f"    hand push section: {len(hands)} distinct value(s)"
          + (" — all zero: the protocol-path feedback is dead in modbus mode, so the push says "
             "nothing about finger motion; ANGLE_ACT (below) is the only hand feedback" if zero_hand else ""))
    pre = rec.get("pre_run")
    if isinstance(pre, dict):
        for key in ("retreat", "hand_open", "pole", "hand"):
            v = pre.get(key)
            if v is None:
                continue
            if key == "pole" and isinstance(v, dict):
                print(f"    run commanded pole: {v.get('start_m')} -> {v.get('target_m')} m, read back "
                      f"{v.get('final_m')} m after {v.get('t_s', 0):.2f} s  ({'ok' if v.get('ok') else 'FAILED'})")
            elif key in ("hand_open", "hand") and isinstance(v, dict):
                print(f"    run commanded {key:<9}: ANGLE_SET {v.get('target')} | ANGLE_ACT before {v.get('before')} "
                      f"after {v.get('after')} ({'arrived' if v.get('arrived') else 'settled' if v.get('settled') else 'NOT settled'}, "
                      f"{v.get('t_s', 0):.2f} s)")
            elif key == "retreat":
                print(f"    run commanded retreat movej: ret {v}")
    else:
        print("    run commanded no device (single `run`, or flow --no-devices): the pole and hand "
              "stayed as found — check the pole height above against the task's pre_stroke")
    # ---- controller snapshots (F111) -------------------------------------------
    def _snap(label, c):
        if not isinstance(c, dict):
            return
        a = c.get("arm_state") if isinstance(c.get("arm_state"), dict) else {}
        def one(k):
            v = c.get(k)
            return v[1] if isinstance(v, (list, tuple)) and len(v) == 2 else v
        tf = one("tool_frame")
        tfs = (f"{tf.get('name') or tf.get('frame_name')} payload {tf.get('payload')} kg centroid "
               f"({tf.get('x')}, {tf.get('y')}, {tf.get('z')})" if isinstance(tf, dict) else str(tf)[:40])
        jf = c.get("joint_err_flag")
        jf = jf.get("err_flag") if isinstance(jf, dict) else jf
        print(f"    {label:<7} err {a.get('err_codes')} joint_err {jf} collision stage {one('collision_stage')} | "
              f"tool {tfs} | joint max {one('joint_max_speed_deg_s')} deg/s, {one('joint_max_acc_deg_s2')} deg/s² | "
              f"TCP {one('arm_max_line_speed')} m/s, {one('arm_max_angular_speed')} rad/s")
    if rec.get("controller_before") or rec.get("controller_after"):
        print("\n  CONTROLLER (read directly, before / after the program)")
        _snap("before", rec.get("controller_before"))
        _snap("after", rec.get("controller_after"))
    return 0


# --------------------------------------------------------------- compare
def cmd_compare(args) -> int:
    np = _np()
    rec_path = pathlib.Path(args.record)
    rec = json.loads(rec_path.read_text())
    frames_path = rec_path.with_name(rec_path.stem + "_frames.ndjson")
    if not frames_path.is_file():
        raise SystemExit("compare needs a --record-joints capture beside the record")
    track = REPO / rec["track"]
    cmd = load_track(track)
    ts, q, _ = load_frames(frames_path)
    t0 = ts[0] + rec.get("movej_s", 0.0) + rec.get("dwell_s", 0.0)
    t1 = t0 + rec.get("replay_s", 0.0)
    m = (ts >= t0) & (ts <= t1)
    tw, qw = ts[m], q[m]
    # phase per frame by nearest commanded point (whole track — a windowed
    # matcher races forward on one bad match; that bug cost an hour on
    # 2026-09-01 by reporting 40° median deviation on a clean run)
    dev = np.empty(len(qw)); near = np.empty(len(qw), int)
    for lo in range(0, len(qw), 200):
        blk = qw[lo:lo+200]
        d = np.abs(blk[:, None, :] - cmd[None, :, :]).max(axis=2)
        dev[lo:lo+len(blk)] = d.min(axis=1); near[lo:lo+len(blk)] = d.argmin(axis=1)
    print(f"COMMANDED vs EXECUTED — slot {rec['slot']}, {len(cmd)} pts vs {len(qw)} frames\n")
    print(f"  {'decile':>7} {'phase pt':>9} {'t (s)':>8} {'dev°':>7} {'worst joint':>12}")
    for f in np.arange(0.1, 1.01, 0.1):
        target = int(f * (len(cmd) - 1))
        k = int(np.argmin(np.abs(near - target)))
        j = int(np.argmax(np.abs(qw[k] - cmd[near[k]])))
        print(f"  {f:6.0%} {near[k]:9d} {tw[k]-t0:8.2f} {dev[k]:7.2f} {'J'+str(j+1):>12}")
    per_joint = np.abs(qw - cmd[near]).max(axis=0)
    print("\n  peak per-joint deviation: " +
          " ".join(f"J{j+1} {per_joint[j]:.1f}°" for j in range(7)))
    print(f"  overall p50 {np.median(dev):.2f}° p99 {np.percentile(dev, 99):.2f}° "
          f"max {dev.max():.2f}°")
    return 0


# ---------------------------------------------------------------- ledger
def cmd_ledger(args) -> int:
    rows = sorted(RUNS.rglob("run_slot*.json"))
    if not rows:
        print(f"no runs recorded under {RUNS}")
        return 0
    print(f"{'when':>19} {'slot':>4} {'name':26s} {'acc':>4} {'replay s':>9} {'file%':>5} {'ms/pt':>6} "
          f"{'coverage':>9} {'dev p99':>7} {'pole mm':>9} {'dev':>3} {'outcome':<11}verdict")

    def pole_span(rec_path):
        """first/last lift height from the frames file, reading two lines only."""
        fp = rec_path.with_name(rec_path.stem + "_frames.ndjson")
        if not fp.is_file():
            return "-"
        try:
            with open(fp, "rb") as fh:
                first = json.loads(fh.readline())
                fh.seek(0, 2)
                size = fh.tell()
                fh.seek(max(0, size - 4096))
                last = json.loads(fh.read().splitlines()[-1])
            a = (first.get("frame", {}).get("lift_state") or {}).get("height")
            b = (last.get("frame", {}).get("lift_state") or {}).get("height")
            return f"{a}->{b}" if a != b else f"{a}"
        except Exception:                                              # noqa: BLE001
            return "?"

    for path in rows:
        r = json.loads(path.read_text())
        rc = r.get("replay_check") or {}
        replay = f"{r['replay_s']:.3f}" if r.get("replay_s") else "-"
        clock = f"{r['implied_ms_per_pt']:.3f}" if r.get("implied_ms_per_pt") else "-"
        cover = f"{rc['phase_first']*100:.0f}->{rc['phase_last']*100:.0f}%" if rc else "-"
        dev = f"{rc['dev_p99_deg']:.2f}" if rc else "-"
        when = (r.get("when") or "?")[-19:]
        pts = r.get("points")
        clk = track_clock_s(REPO / r["track"] if r.get("track") else None, r.get("track_sha256"))[0]
        filepct = (f"{r['replay_s'] / ((pts - 1) * clk) * 100:.0f}"
                   if (r.get("replay_s") and pts) else "-")
        acc = f"{r['acc_scale']:g}" if r.get("acc_scale") else "-"
        devices = "yes" if r.get("pre_run") else "no"
        # The outcome, not just the band verdict: a capture that stopped while
        # the arm was still moving proves nothing about where the run ended.
        # A run that never started is not a cut. The refusal verdicts leave
        # complete=False because nothing traced the track, so they have to be
        # read off the verdict, not off the coverage.
        refused = str(r.get("verdict", "")).startswith(("refused", "did_not_start"))
        outcome = ("REFUSED" if refused else
                   "COMPLETE" if r.get("complete") else
                   "UNRESOLVED" if r.get("complete") is None and r.get("capture_truncated")
                   else "CUT" if r.get("complete") is False else "-")
        print(f"{when:>19} {r.get('slot', '?'):>4} {(r.get('name') or '')[10:36]:26s} {acc:>4} "
              f"{replay:>9} {filepct:>5} {clock:>6} {cover:>9} {dev:>7} {pole_span(path):>9} {devices:>3} "
              f"{outcome:<11}{r.get('verdict', '?')}")
    print("\n  file% = replay / (points-1 x the track's own clock_s: 5.5545 ms before 2026-10-02, 5.0 after). "
          "coverage is the ORDERED matcher (F112): the older "
          "global-nearest number over-read it by up to 40 points and is kept as replay_check_legacy_global. "
          "UNRESOLVED = the capture stopped while the arm was still moving, so where that run ended was "
          "never recorded — it is not evidence of a cut. pole = lift height first->last from the push. "
          "dev column = the run commanded the devices (flow) or left them as found (single run).")
    return 0


# ---------------------------------------------------------------- report
def cmd_report(args) -> int:
    days = sorted(p.name for p in RUNS.iterdir() if p.is_dir() and p.name[:4].isdigit())
    if not days:
        print(f"no run days under {RUNS}")
        return 1
    if args.date_from:
        lo, hi = args.date_from, args.date_to or args.date_from
        chosen = [d for d in days if lo <= d <= hi]
        period = lo if lo == hi else f"{lo}_to_{hi}"
    else:
        day = args.day or days[-1]
        chosen = [d for d in days if d == day]
        period = day
    recs = []
    for d in chosen:
        recs += sorted((RUNS / d).glob("run_slot*.json"))
    if args.slots:
        recs = [r for r in recs if int(json.loads(r.read_text()).get("slot", -1)) in args.slots]
    if args.setting:
        import track_report_pdf as pdfmod
        recs = [r for r in recs if args.setting in pdfmod.setting_of(json.loads(r.read_text()))]
    if not recs:
        print(f"no records for {period}")
        return 1
    out = pathlib.Path(args.out) if args.out else REPO / "Resource/reports/track_runs" / f"{period}.pdf"
    import track_report_pdf as pdfmod
    info = pdfmod.build(recs, out, period, REPO / "Resource/reports/track_runs" / ".track_cache")
    print(f"{info['runs']} runs ({info['complete']} complete, {info['cut']} cut) · settings "
          f"{info['groups']} · {info['pages']} pages -> {out}")
    return 0


# ---------------------------------------------------------------- devices
def cmd_devices(args) -> int:
    """Did the pole and the hand actually MOVE on each run?

    Newton, 2026-09-03: "the pole and hand did nothing". The run record's
    before/after pair cannot answer that — a setpoint echo and a real device
    have the same endpoints — so this reads the evidence that can:

      * the pole from the UDP push's own `lift_state.height` series, which is
        independent of anything this tool commanded. A real lift walks through
        the intermediate millimetres; an echo steps once.
      * the hand from the record's ANGLE_ACT poll series where present (added
        2026-09-03) and from the before/after pair otherwise, per channel.

    It also says, per run, whether the devices were COMMANDED at all: a single
    `run` never touches them, which is by design and was the state of 20 of the
    23 runs on 2026-09-02.
    """
    rows = sorted((REPO / "Resource/runs/tracks").glob("*/run_slot*.json"))
    if args.day:
        rows = [r for r in rows if r.parent.name in args.day]
    hdr = (f"{'when':<17}{'slot':<5}{'task':<22}{'cmd?':<6}{'pole mm':<18}"
           f"{'steps':<7}{'span s':<8}{'hand travel per channel':<40}{'stuck'}")
    print(hdr)
    print("-" * len(hdr))
    for path in rows:
        rec = json.loads(path.read_text())
        pre = rec.get("pre_run") or {}
        commanded = "yes" if pre else "no"
        fp = path.with_name(path.stem + "_frames.ndjson")
        pole_txt, steps, span = "-", "-", "-"
        if fp.is_file():
            hs, tsl = [], []
            with open(fp) as fh:
                for line in fh:
                    try:
                        d = json.loads(line)
                    except Exception:                                  # noqa: BLE001
                        continue
                    h = (d.get("frame", {}).get("lift_state") or {}).get("height")
                    if h is not None:
                        hs.append(float(h))
                        tsl.append(d["t_mono"])
            if hs:
                lo, hi = min(hs), max(hs)
                pole_txt = f"{hs[0]:.0f}->{hs[-1]:.0f}"
                ch = [i for i in range(1, len(hs)) if hs[i] != hs[i - 1]]
                steps = str(len(ch))
                span = (f"{tsl[ch[-1]] - tsl[ch[0]]:.2f}" if len(ch) > 1 else
                        ("0.00" if ch else "-"))
                if hi - lo == 0:
                    pole_txt += " (still)"
        hand_txt, stuck = "-", "-"
        h = pre.get("hand")
        if h:
            mv = h.get("moved")
            if mv:
                hand_txt = f"travel {mv}"
                stuck = str(h.get("stuck") or [])
            else:
                b, a = h.get("before"), h.get("after")
                if b and a:
                    d = [abs(x - y) for x, y in zip(a, b)]
                    hand_txt = f"changed {d}"
                    stuck = str([i + 1 for i, v in enumerate(d) if v <= 3])
        print(f"{(rec.get('when') or '?')[5:16]:<17}{rec.get('slot', '?'):<5}"
              f"{(rec.get('name') or '')[10:31]:<22}{commanded:<6}{pole_txt:<18}"
              f"{steps:<7}{span:<8}{hand_txt:<40}{stuck}")
    print("\n  cmd? = the run COMMANDED the devices (`flow`) or left them as found "
          "(`run`, which never touches them). pole steps = how many times the pushed "
          "height changed: a real lift walks through every millimetre, a setpoint echo "
          "steps once. hand `stuck` = channels that were commanded to move and did not "
          "(1-6; channel 6 is the thumb rotation, dead at 985 on this hand).")
    return 0


# ------------------------------------------------------------------ fkcheck
def cmd_fkcheck(args) -> int:
    """Validate our URDF FK against the controller's OWN reported TCP.

    Every Cartesian number in this tool rests on pushing joint angles through
    `alix_workspace.urdf_kinematics`. The push also carries the controller's
    own `waypoint.position`, so the chain can be checked end to end rather than
    trusted: fit a rigid transform (Kabsch) from our FK onto that stream and
    report the residual. The frames differ by a pure 90 deg rotation — the
    controller's base is ours rotated about Y — which is a convention, not an
    error, so the residual AFTER the fit is the number that matters.

    Measured 2026-09-04: R_glove_frame_2 residual 0.005 mm median, every other
    candidate frame 15-29 mm. So the FK is exact and the controller's
    supervised TCP is R_glove_frame_2.
    """
    import numpy as np
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import cut_dossier as cd
    rows = sorted((REPO / "Resource/runs/tracks").glob("*/run_slot*.json"))
    if args.day:
        rows = [r for r in rows if r.parent.name in args.day]
    rows = rows[:args.limit]

    def kabsch(A, B):
        ca, cb = A.mean(0), B.mean(0)
        U, _S, Vt = np.linalg.svd((A - ca).T @ (B - cb))
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        R = Vt.T @ np.diag([1, 1, d]) @ U.T
        return R, cb - R @ ca

    frames = args.frames or ["R_glove_frame_1", "R_glove_frame_2",
                             "R_glove_frame_3", "R_glove_frame_4"]
    hdr = f"{'run':<26}{'frame':<20}{'residual median mm':<21}{'p95':<11}{'rotation deg'}"
    print(hdr)
    print("-" * len(hdr))
    for path in rows:
        fp = path.with_name(path.stem + "_frames.ndjson")
        if not fp.is_file():
            continue
        rec = json.loads(path.read_text())
        ts, q, raw = load_frames(fp)
        try:
            wp = np.array([x["waypoint"]["position"] for x in raw], float) / 1e6
        except Exception:                                              # noqa: BLE001
            print(f"{path.stem[:25]:<26}no waypoint.position in the push")
            continue
        sub = slice(0, len(q), max(1, len(q) // 400))
        for frame in frames:
            try:
                P, _ = cd.fk_tool(q[sub], rec.get("side", "right"), frame)
            except Exception:                                          # noqa: BLE001
                continue
            R, t = kabsch(P, wp[sub])
            d = np.linalg.norm((P @ R.T) + t - wp[sub], axis=1) * 1000.0
            ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
            print(f"{path.stem[:25]:<26}{frame:<20}{np.median(d):<21.4f}"
                  f"{np.percentile(d, 95):<11.4f}{ang:.3f}")
    print("\n  Residual AFTER a rigid fit onto the controller's own waypoint.position. "
          "A near-zero residual means our FK reproduces the controller's kinematics exactly; "
          "the rotation column is the fixed base-frame convention difference, not an error.")
    return 0


# -------------------------------------------------------------- cartesian
def cmd_cartesian(args) -> int:
    """How far the TCP actually strays from the commanded Cartesian path.

    Newton, 2026-09-04: the joint-space deviation was measured from the start
    but the CARTESIAN one never was, and for a wiping task that is the number
    that decides whether the pad is on the surface. Both the commanded track
    and the executed joints are pushed through the URDF FK at the task's own
    ik_frame, and each executed sample is scored against the NEAREST point of
    the commanded path — so this is a LOWER bound on the true error (an arm
    that has drifted onto a neighbouring raster stripe scores as close).
    """
    import numpy as np
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import track_report_pdf as trp
    import cut_dossier as cd
    rows = sorted((REPO / "Resource/runs/tracks").glob("*/run_slot*.json"))
    if args.day:
        rows = [r for r in rows if r.parent.name in args.day]
    cache = REPO / ".track_cache"
    hdr = (f"{'when':<20}{'task':<22}{'acc':<6}{'outcome':<10}{'xtrk50':<9}{'xtrk95':<9}"
           f"{'time50':<10}{'time95':<10}{'ms/pt':<10}{'cmd m/s':<9}{'exec m/s'}")
    print(hdr)
    print("-" * len(hdr))
    for path in rows:
        rec = json.loads(path.read_text())
        # BY NAME (cut_dossier.manifest_row): keyed by slot, 21 of 116 records
        # were measured at another region's ik_frame.
        row = cd.manifest_row(rec)
        if rec.get("complete") is None or row is None:
            continue
        fp = path.with_name(path.stem + "_frames.ndjson")
        if not fp.is_file():
            continue
        a = trp.analyse(path, cache)
        T = a.get("track")
        if T is None or rec.get("movej_s") is None or not rec.get("replay_s"):
            continue
        frame = row["ik_frame"]
        Pc, _ = cd.fk_tool(T, rec.get("side", "right"), frame)
        ts, q, _raw = load_frames(fp)
        t0 = ts[0] + rec["movej_s"] + (rec.get("dwell_s") or 0.0)
        m = (ts >= t0) & (ts <= t0 + rec["replay_s"])
        qe = q[m][::args.stride]
        if len(qe) < 20:
            continue
        Pe, _ = cd.fk_tool(qe, rec.get("side", "right"), frame)
        # CROSS-TRACK: nearest commanded point inside the F112 lag budget. An
        # unbounded search jumps raster stripes (it implied 3.9-12.7 s of lag),
        # so the window is mandatory, not an optimisation.
        n = len(Pc)
        tw = ts[m][::args.stride] - t0
        d = np.empty(len(Pe))
        for i, p in enumerate(Pe):
            nom = min(int(tw[i] / CLOCK_S), n - 1)
            lo, hi = max(0, nom - 400), min(n, nom + 150)
            d[i] = np.linalg.norm(Pc[lo:hi] - p, axis=1).min() * 1000.0
        # TIME-ALIGNED: where the arm IS against where it should BE at that
        # instant. The nominal 5.5545 ms/pt is not the arm's own rate, so an
        # affine clock is fitted first — alpha is the arm's measured rate and
        # a fixed clock drifts by hundreds of mm without it (F118).
        def terr(alpha, beta):
            k = np.clip(((tw + beta) / (CLOCK_S * alpha)).astype(int), 0, n - 1)
            return np.linalg.norm(Pe - Pc[k], axis=1) * 1000.0
        best = (float("inf"), 1.0, 0.0)
        for al in np.arange(0.94, 1.09, 0.005):
            for be in np.arange(-0.20, 0.40, 0.02):
                v = float(np.median(terr(al, be)))
                if v < best[0]:
                    best = (v, al, be)
        _, alpha, beta = best
        te = terr(alpha, beta)
        cmdv = float((np.linalg.norm(np.diff(Pc, axis=0), axis=1) / CLOCK_S).max())
        dt = np.diff(ts[m][::args.stride])
        ev = float(np.percentile(np.linalg.norm(np.diff(Pe, axis=0), axis=1)
                                 / np.maximum(dt, 1e-9), 99))
        out = ("COMPLETE" if rec.get("complete") else "CUT")
        print(f"{rec['when']:<20}{rec['name'][10:][:21]:<22}{str(rec.get('acc_scale')):<6}"
              f"{out:<10}{np.percentile(d, 50):<9.1f}{np.percentile(d, 95):<9.1f}"
              f"{np.percentile(te, 50):<10.1f}{np.percentile(te, 95):<10.1f}"
              f"{CLOCK_S * alpha * 1000:<10.4f}{cmdv:<9.3f}{ev:.3f}")
    print("\n  xtrk = CROSS-TRACK, mm: distance to the nearest commanded point inside the lag "
          "budget — how far the arm is OFF THE PATH.\n  time = TIME-ALIGNED, mm: distance to "
          "where the arm should be at that instant, after fitting the arm's own replay rate "
          "(the ms/pt column; the nominal is 5.5545).\n  Both at the task's own ik_frame, whose "
          "FK is validated to 0.005 mm against the controller's own reported TCP "
          "(`track_report.py fkcheck`).")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("predict", help="expected on-arm behaviour of a track (offline)")
    p.add_argument("track", help="slot number, region key, or path to a track file")
    p.add_argument("--hotspots", type=int, default=5, help="list N fastest path points")
    p.set_defaults(fn=cmd_predict)
    r = sub.add_parser("review", help="what a recorded run actually did")
    r.add_argument("record", help="path to a run_slotNN_*.json record")
    r.set_defaults(fn=cmd_review)
    c = sub.add_parser("compare", help="commanded vs executed, per joint")
    c.add_argument("record", help="path to a run_slotNN_*.json record")
    c.set_defaults(fn=cmd_compare)
    l = sub.add_parser("ledger", help="one line per recorded run")
    l.set_defaults(fn=cmd_ledger)
    fk = sub.add_parser("fkcheck", help="validate our FK against the controller's own TCP")
    fk.add_argument("--day", nargs="*")
    fk.add_argument("--limit", type=int, default=3)
    fk.add_argument("--frames", nargs="*")
    fk.set_defaults(fn=cmd_fkcheck)
    ca = sub.add_parser("cartesian", help="executed TCP vs the commanded Cartesian path")
    ca.add_argument("--day", nargs="*")
    ca.add_argument("--stride", type=int, default=4, help="sample every Nth frame (default 4)")
    ca.set_defaults(fn=cmd_cartesian)
    dv = sub.add_parser("devices", help="did the pole and the hand actually move, per run")
    dv.add_argument("--day", nargs="*")
    dv.set_defaults(fn=cmd_devices)
    g = sub.add_parser("report", help="PDF bundle of a day's (or period's) runs — graphs, grouped by setting")
    g.add_argument("--day", default=None, help="YYYY-MM-DD (default: the latest day with records)")
    g.add_argument("--from", dest="date_from", default=None, help="YYYY-MM-DD, with --to: a period")
    g.add_argument("--to", dest="date_to", default=None)
    g.add_argument("--slots", nargs="*", type=int, default=None, help="restrict to these slots")
    g.add_argument("--setting", default=None,
                   help="restrict to runs whose setting label contains this text, e.g. 'acc 2'")
    g.add_argument("--out", default=None, help="output PDF (default Resource/reports/track_runs/<period>.pdf)")
    g.set_defaults(fn=cmd_report)
    args = ap.parse_args(argv)
    rc = args.fn(args)
    assert_inert()
    return rc


if __name__ == "__main__":
    sys.exit(main())
