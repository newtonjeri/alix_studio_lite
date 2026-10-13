#!/usr/bin/env python3
"""Minimum execution time for a planned joint path, under joint limits ONLY.

Takes the GEOMETRY of a stage in a task-node plan (the joint waypoints, timing
discarded) and re-parameterises it time-optimally with TOPP-RA under the
production joint velocity and acceleration limits. Answers "how much faster can
this exact path be run without violating anything real?" — no Cartesian speed,
angular or lateral-acceleration cap is involved anywhere.

EMITTING A TRACK. --emit-track writes the point list a fixed-clock consumer plays,
one point per CLOCK (5.0 ms since 2026-10-02: the CANFD stream period, Newton "the
planner should emit at 5ms"; the upload player plays the same file at its own
~5.5545 ms average), so the SPACING of the points IS the velocity profile:
v_i = (s_{i+1} - s_i)/clock. That makes the
spacing the complete timing degree of freedom, not merely a uniform brake — the
emitter can realise any s(t) it can compute, including one faster than the
stroke pipeline produces, because it is not bound by that pipeline's per-piece
structure. --jerk-fit then widens a smoothing window on sdot until the SAMPLED
point sequence (which is literally what the arm is commanded) fits the jerk
ceiling, and reports what that costs.

WHAT THIS DOES NOT ANSWER. TOPP-RA's optimum is BANG-BANG in acceleration: at
every switch the acceleration steps between its bounds, so the solution's jerk
is infinite in the continuum and, when sampled, simply reads 2*a_max/dt. Any
jerk figure taken from it measures the sampling interval, not the robot. The
number below is therefore a FLOOR that ignores jerk entirely: the real
jerk-limited optimum is slower, and how much slower is a question only a
jerk-limited parameteriser (the stroke pipeline) can answer. Use this to size
the prize, not to set a duration.

    python3 tools/plan_min_time.py <plan.json> --stage execute_path

Limits come from alix_moveit_config/config/joint_limits.yaml (velocity there is
kept equal to the xacro by tools/kinematics/check_kinematics_sources.py; acceleration
10.472 rad/s^2 is the vendor's own libalgo_get_joint_acc_maxlim; jerk 1500
rad/s^3 is the measured tracking ceiling — see that file's header).
"""
import argparse
import hashlib
import json
import pathlib
import sys

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
#: THE EMIT CLOCK, 5.0 ms = the CANFD stream period (Newton 2026-10-02: "the rate should be 5ms, so
#: the planner should emit at 5ms"). It was 0.0055545, the upload player's whole-run AVERAGE (F98: the
#: 5 ms cycle plus its holds), and the production CANFD driver refuses any track whose declared clock_s
#: is not its 5.0 ms period (CANFD_PRODUCTION_DRIVER.md §0). The TOPP-RA time law is unchanged: only
#: the sampling of it is finer. Same number as track_emit.hpp kClockS and cleaning_tracks.CLOCK_S.
CLOCK = 0.005
LIMITS = REPO / "ros/manipulation/alix_moveit_config/config/joint_limits.yaml"


def load_limits(side, path=LIMITS):
    import yaml
    jl = yaml.safe_load(path.read_text())["joint_limits"]
    names = [f"{side}_joint{i}" for i in range(1, 8)]
    missing = [n for n in names if n not in jl]
    if missing:
        raise SystemExit(f"{path}: no limits for {missing}")
    v = np.array([jl[n]["max_velocity"] for n in names])
    a = np.array([jl[n]["max_acceleration"] for n in names])
    j = np.array([jl[n].get("max_jerk", np.inf) for n in names])
    return names, v, a, j


def stage_of(plan, name, index):
    subs = plan.get("sub_trajectories") or plan.get("stages") or []
    cands = [s for s in subs if s.get("waypoints") and len(s.get("joint_names", [])) == 7]
    if index is not None:
        return subs[index]
    for s in cands:
        if s.get("stage_name") == name:
            return s
    raise SystemExit(f"no 7-joint stage named {name!r}; have "
                     f"{[s.get('stage_name') for s in cands]}")


class LsqPath:
    """A least-squares B-spline through ALL the plan's waypoints, in toppra's
    geometric-path interface.

    Interpolating a cubic through every waypoint of a 10 ms-sampled plan
    OSCILLATES: on the archive toplid stroke |q''(s)| reaches 51608 rad^-1
    against a path median of 0.55, purely from representation noise at 12 mrad
    spacing. Since the acceleration constraint carries a centripetal term
    |q''| * sdot^2, that noise dominates the parameterisation and TOPP-RA
    returns a solution violating the acceleration limit 100-fold between grid
    points. Thinning the control points fixes the oscillation but MOVES the
    path (2.8 deg at 0.05 rad spacing). Approximating all of them in least
    squares fixes both: 0.5 deg of error and |q''| max 66.
    """

    MIN_PTS_PER_KNOT = 5

    def __init__(self, q_all, knot_spacing):
        seg = np.linalg.norm(np.diff(q_all, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        self.arc = float(s[-1])
        u = s / self.arc
        # Schoenberg-Whitney: a least-squares B-spline needs enough data in every
        # knot interval or the normal equations go singular and the fit returns
        # NaN silently. Keep >= MIN_PTS_PER_KNOT waypoints per interval.
        want = max(6, int(self.arc / knot_spacing))
        cap = max(6, q_all.shape[0] // self.MIN_PTS_PER_KNOT)
        n_knots = min(want, cap)
        self.knots_capped = n_knots < want
        knots = np.concatenate([[0.0] * 4, np.linspace(0, 1, n_knots)[1:-1], [1.0] * 4])
        from scipy.interpolate import make_lsq_spline
        self._spl = make_lsq_spline(u, q_all, knots, k=3)
        if not np.isfinite(self._spl.c).all():
            raise SystemExit(f"LSQ fit is singular at {n_knots} knots for "
                             f"{q_all.shape[0]} waypoints — raise --knot-spacing")
        self._d = {0: self._spl, 1: self._spl.derivative(1), 2: self._spl.derivative(2)}
        self.n_control = n_knots
        self.dof = q_all.shape[1]
        self.path_interval = np.array([0.0, 1.0])

    def __call__(self, ss, order=0):
        return self._d[order](np.clip(ss, 0.0, 1.0))

    def fidelity(self, q_all, samples=40000):
        """Max joint-space distance from every ORIGINAL waypoint to the fitted path."""
        pts = self(np.linspace(0, 1, samples))
        out = np.empty(q_all.shape[0])
        for i in range(0, q_all.shape[0], 128):
            c = q_all[i:i + 128]
            out[i:i + 128] = np.sqrt(((c[:, None, :] - pts[None, :, :]) ** 2).sum(-1)).min(1)
        return out

    def curvature(self, samples=20000):
        n2 = np.linalg.norm(self(np.linspace(0, 1, samples), 2), axis=1) / self.arc ** 2
        return float(np.percentile(n2, 95)), float(n2.max())


def waypoints(stage):
    return np.array([w["positions"] for w in stage["waypoints"]], dtype=float)


def concatenate(parts, names, max_gap_deg=0.5):
    """Join per-stage point blocks into one track.

    The player has no notion of stages: a gap at a boundary would be commanded
    as a single-tick step, i.e. an impulse. Continuity is CHECKED and the
    duplicated boundary sample dropped — same contract as plan_to_track.py.
    Each stage is parameterised SEPARATELY with zero boundary velocity, which
    preserves the plan's own semantics: move_to_start comes to rest before the
    stroke touches the surface, and move_to_rest starts from rest.
    """
    out, spans, cursor = [], [], 0
    for k, (q, name) in enumerate(zip(parts, names)):
        if k:
            gap = float(np.degrees(np.abs(q[0] - out[-1][-1]).max()))
            if gap > max_gap_deg:
                raise SystemExit(f"boundary {names[k-1]} -> {name} jumps {gap:.3f} deg — "
                                 f"the player would step it in one tick")
            # The drop threshold used to be 1e-6 while `gap` is in DEGREES and the
            # docstring said radians — 5.7e4 too tight, so it never fired and every
            # join emitted a duplicated point: a one-tick zero-velocity command in
            # the middle of continuous motion. One millidegree is the wire
            # resolution; anything under it is the same commanded point.
            if gap < 1e-3:
                q = q[1:]
        spans.append({"stage": name, "points": [cursor, cursor + len(q)]})
        cursor += len(q)
        out.append(q)
    return np.vstack(out), spans


class ToolSpeed:
    """|dp/du| AND |dw/du| along the path, for capping tool speed inside TOPP-RA.

    Capping translation alone is not enough: on this family the WRIST, not the
    TCP, sets the stroke time (12 of 15 commode_c regions are angular-limited),
    and a translation-only cap leaves the pad free to spin faster. v2 did exactly
    that — 1.158 rad/s against the config's cleaning_angular_cap_rad_s of 1.0 and
    G4's demonstrated 0.852.

    Tool speed is |dp/du|*udot and joint speed |dq_j/du|*udot, so capping the
    tool at V is exactly the varying joint bound |dq_j/du| * V/|dp/du|. Feeding
    it to the solver keeps the result feasible; capping udot afterwards does not
    (the profile can no longer brake into the cap).
    """

    def __init__(self, path, side, frame, pole_m=0.075, urdf=None):
        import sys as _s
        _s.path.insert(0, str(REPO / "core/python/alix_workspace/src"))
        from alix_workspace.urdf_kinematics import UrdfModel
        model = UrdfModel.from_file(str(urdf or URDF))
        chain = model.chain("base_link", frame)
        # the chain carries the POLE joint as well as the seven arm joints; the
        # plan's move_pole_to_quarter_height stage pins it at 0.075 m
        movable = [j.name for j in chain if j.movable]
        arm = [f"{side}_joint{i}" for i in range(1, 8)]
        fixed = {f"{side}_sliding_plate_joint": pole_m}
        missing = [n for n in movable if n not in arm and n not in fixed]
        if missing:
            raise SystemExit(f"chain base_link -> {frame} has un-pinned joints {missing}")
        if not any(n in arm for n in movable):
            raise SystemExit(
                f"--ik-frame {frame} shares NO joint with the plan's {side} arm: every chain "
                f"joint would be pinned and the Cartesian caps would bind nothing "
                f"(audit 2026-08-31). Use the matching {side}_glove_frame_N.")
        idx = {n: i for i, n in enumerate(arm)}
        self.u = np.linspace(0.0, 1.0, 4000)
        q = path(self.u)
        full = np.zeros((q.shape[0], len(movable)))
        for c, n in enumerate(movable):
            full[:, c] = q[:, idx[n]] if n in idx else fixed[n]
        pos = np.array([model.fk(chain, qq)[:3, 3] for qq in full])
        du = self.u[1] - self.u[0]
        self.dp_du = np.linalg.norm(np.gradient(pos, du, axis=0), axis=1)
        rot = np.array([model.fk(chain, qq)[:3, :3] for qq in full])
        dR = np.einsum("nij,nkj->nik", rot[1:], rot[:-1])
        tr = np.clip((np.trace(dR, axis1=1, axis2=2) - 1.0) / 2.0, -1.0, 1.0)
        w = np.arccos(tr) / du
        self.dw_du = np.concatenate([[w[0]], w])
        self.frame = frame

    def bounds(self, path, vlim, v_cart, w_cart=0.0):
        dp = np.maximum(self.dp_du, 1e-9)
        dw = np.maximum(self.dw_du, 1e-9)
        def f(u):
            uu = np.atleast_1d(u)
            dq = np.abs(path(uu, 1))
            scale = v_cart / np.interp(uu, self.u, dp)
            if w_cart > 0:
                scale = np.minimum(scale, w_cart / np.interp(uu, self.u, dw))
            lim = np.minimum(vlim[None, :], dq * scale[:, None])
            out = np.stack([-lim, lim], axis=-1)
            return out[0] if np.isscalar(u) or np.ndim(u) == 0 else out
        return f


def solve(path, vlim, alim, gridpoints, vlim_varying=None):
    import toppra.algorithm as algo
    import toppra.constraint as constraint
    import toppra as ta
    ta.setup_logging("ERROR")
    cons = [constraint.JointAccelerationConstraint(np.column_stack([-alim, alim]))]
    cons.append(constraint.JointVelocityConstraintVarying(vlim_varying)
                if vlim_varying is not None
                else constraint.JointVelocityConstraint(np.column_stack([-vlim, vlim])))
    grid = np.linspace(0, 1, gridpoints)
    inst = algo.TOPPRA(cons, path, gridpoints=grid, solver_wrapper="seidel")
    traj = inst.compute_trajectory(0, 0)
    if traj is None:
        raise SystemExit("TOPP-RA found no feasible parameterisation")
    _, sd_grid, _ = inst.compute_parameterization(0, 0)
    return traj, grid, np.asarray(sd_grid, dtype=float)


def sample(traj, dt=0.002):
    n = int(np.ceil(traj.duration / dt)) + 1
    t = np.linspace(0, traj.duration, n)
    qd = traj(t, 1)
    qdd = traj(t, 2)
    jrk = np.gradient(qdd, t, axis=0)
    return t, qd, qdd, jrk


def util(x, lim):
    return np.abs(x).max(axis=0) / lim


def band_limited_util(q, clock, vlim, alim, jceil, fc_hz):
    """v/a/j of the emitted sequence, band-limited.

    RAW finite differences on a point list are meaningless: points go on the
    wire as integer millidegrees, and ONE LSB of jitter between consecutive
    points is 2.26 rad/s^2 and 814 rad/s^3 at this clock. Measured raw, the
    tracks that ALREADY run on this arm read 23.7x (G4) and 50x (right_test)
    the acceleration limit. Band-limit first — F99c.
    """
    from scipy.signal import butter, filtfilt
    fs = 1.0 / clock
    b, a_ = butter(4, min(fc_hz / (fs / 2), 0.99), "low")
    qf = filtfilt(b, a_, q, axis=0)
    v = np.diff(qf, axis=0) / clock
    ac = np.diff(v, axis=0) / clock
    j = np.diff(ac, axis=0) / clock
    return (float(util(v, vlim).max()), float(util(ac, alim).max()),
            float(np.abs(j).max() / jceil))


URDF = REPO / "core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf"


def check_rate(q, clock, vlim):
    """Per-tick commanded joint rate, band-limited AND raw. Returns (worst_deg_s, n_over).

    The control track (G4) has ZERO ticks above 180 deg/s; the earlier emit had
    149 raw / 148 band-limited in 9 sustained episodes, peaking at 221 deg/s.
    The upload player enforces no rate limit, so nothing downstream catches this.
    """
    from scipy.signal import butter, filtfilt
    fs = 1.0 / clock
    b, a_ = butter(4, min(20.0 / (fs / 2), 0.99), "low")
    v = np.degrees(np.abs(np.diff(filtfilt(b, a_, q, axis=0), axis=0)) / clock)
    per_joint = v.max(axis=0)
    return float(per_joint.max()), int((v > 180.0).any(axis=1).sum()), per_joint


def tool_profile(q, spans, side, frame, clock, contact, urdf=URDF):
    """Linear and ANGULAR tool speed on the contact stages.

    Reporting linear speed alone is misleading on this family: the repo's own
    record is that 12 of 15 commode_c regions are ANGULAR-limited, so the wrist
    is the binding axis. A track can match the proven linear peak exactly while
    rotating the pad 37% faster, which is precisely what happened once.
    """
    import sys as _s
    _s.path.insert(0, str(REPO / "core/python/alix_workspace/src"))
    from alix_workspace.urdf_kinematics import UrdfModel
    model = UrdfModel.from_file(str(urdf))
    chain = model.chain("base_link", frame)
    movable = [j.name for j in chain if j.movable]
    idx = {f"{side}_joint{i+1}": i for i in range(7)}
    if not any(n in idx for n in movable):
        # A LEFT plan against an R frame (or vice versa) pins EVERY chain joint
        # to a constant and the tool-speed report silently reads 0.000 while the
        # track still emits (audit 2026-08-31). Refuse instead.
        raise SystemExit(
            f"--ik-frame {frame} shares NO joint with the plan's {side} arm — tool speed "
            f"would silently read 0.000. Use the matching {side}_glove_frame_N.")
    out = {}
    for sp in spans:
        if sp["stage"] not in contact:
            continue
        lo, hi = sp["points"]
        blk = q[lo:hi]
        full = np.zeros((blk.shape[0], len(movable)))
        for c, n in enumerate(movable):
            full[:, c] = blk[:, idx[n]] if n in idx else 0.075
        T = [model.fk(chain, x) for x in full]
        pos = np.array([t[:3, 3] for t in T])
        rot = np.array([t[:3, :3] for t in T])
        v = np.linalg.norm(np.diff(pos, axis=0), axis=1) / clock
        dR = np.einsum("nij,nkj->nik", rot[1:], rot[:-1])
        w = np.arccos(np.clip((np.trace(dR, axis1=1, axis2=2) - 1) / 2, -1, 1)) / clock
        out[sp["stage"]] = {"linear_mps": {"p50": float(np.percentile(v, 50)), "max": float(v.max())},
                            "angular_rad_s": {"p50": float(np.percentile(w, 50)), "max": float(w.max())}}
    return out


def check_positions(q, side, urdf=URDF):
    """Margin to the nearest joint-POSITION limit, per joint, in degrees.

    joint_limits.yaml carries velocity/acceleration/jerk only, so this must come
    from the robot description. It is not decoration: right_test.txt grazes J4's
    +-135 deg and roughly half its replays abort there (F98).
    """
    import xml.etree.ElementTree as ET
    if not urdf.is_file():
        raise SystemExit(f"no expanded URDF at {urdf} — cannot check position limits; refusing to emit")
    root = ET.parse(urdf).getroot()
    lim = {}
    for j in root.findall("joint"):
        n = j.get("name")
        e = j.find("limit")
        if n and e is not None:
            lim[n] = (float(e.get("lower")), float(e.get("upper")))
    out = []
    for i in range(7):
        n = f"{side}_joint{i+1}"
        if n not in lim:
            raise SystemExit(f"{urdf}: no limit for {n} — refusing to emit")
        lo, hi = lim[n]
        out.append(float(np.degrees(min(q[:, i].min() - lo, hi - q[:, i].max()))))
    return out


def resample_clock(traj, clock):
    """Sample the parameterisation onto the player's fixed clock.

    Sample toppra's own trajectory object at clock multiples. An earlier version
    inverted t(s) by hand with np.interp over the solver's gridpoint grid; near a
    stop one grid interval spans ~8.7 player ticks, so the whole interval was
    emitted at its AVERAGE speed and the ramp-from-rest became a VELOCITY STEP
    (measured: J4 0 -> 11.34 deg/s in one tick, 3.4x the acceleration limit, at
    the surface-contact join). It also forced the last sample to s=1 without
    shortening the tick, compressing the sub-tick remainder into a ~2x spike.
    Both are gone: traj(t) is exact at any t.

    The duration is almost never a whole number of ticks. We round the point
    count UP and sample t_i = i*T/n, so the emitted path is played over n*clock
    >= T — marginally SLOWER than optimal, never faster.
    """
    T = float(traj.duration)
    n = int(np.ceil(T / clock))
    t = np.arange(n + 1) * (T / n)
    return traj(t), n * clock


def sampled_limits(q, clock):
    """v, a, j of the emitted point sequence itself — what the arm is commanded."""
    v = np.diff(q, axis=0) / clock
    a = np.diff(v, axis=0) / clock
    j = np.diff(a, axis=0) / clock
    return v, a, j


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("plan", type=pathlib.Path)
    ap.add_argument("--stage", default="execute_path")
    ap.add_argument("--index", type=int, default=None, help="pick the stage by index instead")
    ap.add_argument("--side", default=None, help="R or L (default: from the joint names)")
    ap.add_argument("--vel-scale", type=float, default=1.0)
    ap.add_argument("--acc-scale", type=float, default=1.0)
    ap.add_argument("--jerk-limit", type=float, default=None,
                    help="override the ceiling from joint_limits.yaml (reported, not enforced)")
    ap.add_argument("--knot-spacing", type=float, default=0.03,
                    help="B-spline knot spacing in joint space (rad). Smaller follows the "
                         "waypoints more closely but readmits oscillation; watch the "
                         "reported fidelity AND |q''|.")
    ap.add_argument("--gridpoints", type=int, default=3000)
    ap.add_argument("--stages", default=None,
                    help="comma list of stages to emit as ONE track, e.g. "
                         "move_to_start,execute_path,move_to_rest")
    ap.add_argument("--v-cart-max", type=float, default=0.0,
                    help="cap tool speed at the ik_frame [m/s]; 0 disables. 0.45 is what "
                         "the hardware-proven G4 track actually ran on this surface.")
    ap.add_argument("--w-cart-max", type=float, default=0.0,
                    help="cap tool ANGULAR speed at the ik_frame [rad/s]; 0 disables. "
                         "0.85 is what the hardware-proven G4 track actually ran "
                         "(peak 0.852); the region config declares 1.0.")
    ap.add_argument("--contact-stages", default="execute_path",
                    help="stages where the pad is on the surface, so the Cartesian "
                         "speed caps apply; others are bounded by joint limits alone")
    ap.add_argument("--ik-frame", default="R_glove_frame_2")
    ap.add_argument("--min-position-margin-deg", type=float, default=5.0,
                    help="matches the upload preflight's existing J4 warning threshold; "
                         "the hardware-proven G4 track has 5.502 deg")
    ap.add_argument("--max-acc-util", type=float, default=1.0)
    ap.add_argument("--max-jerk-util", type=float, default=1.0,
                    help="readback tolerance on the band-limited jerk of the WRITTEN track, "
                         "as a fraction of the jerk ceiling (default 1.0 = the old hard gate). "
                         "The ceiling is a joint_limits.yaml number that the 2026-08 audit "
                         "found to measure an ACCELERATION cap (finding 9), and a bang-bang "
                         "optimum's sampled jerk scales with --acc-scale, so a channel policy "
                         "that raises acc-scale must raise this in step or every emit is refused.")
    ap.add_argument("--max-ticks-over-rate", type=int, default=-1,
                    help="refuse if more than this many ticks exceed 180 deg/s. "
                         "180 is the CANFD TRANSPORT guard, not a machine limit "
                         "(J3-J7 are rated 225), and the upload player does not "
                         "enforce it — so this is OFF by default and the count is "
                         "reported instead. Set 0 for a CANFD-destined track.")
    ap.add_argument("--max-joint-rate-deg-s", type=float, default=0.0,
                    help="cap the per-joint velocity used for the emit solve [deg/s]; "
                         "180 is the envelope the hardware-proven G4 track stays inside "
                         "(zero ticks above it). 0 uses the full rated limits.")
    ap.add_argument("--emit-fc-hz", type=float, default=20.0,
                    help="band-limit for judging the emitted sequence")
    ap.add_argument("--emit-track", type=pathlib.Path, default=None,
                    help="write the NDJSON point list for the upload player")
    ap.add_argument("--jerk-fit", action="store_true",
                    help="widen the sdot smoother until the SAMPLED sequence fits the jerk ceiling")
    ap.add_argument("--json", type=pathlib.Path, default=None)
    a = ap.parse_args(argv)

    plan = json.loads(a.plan.read_text())
    stage = stage_of(plan, a.stage, a.index)
    side = a.side or stage["joint_names"][0][0]
    names, vlim, alim, jlim = load_limits(side)
    if stage["joint_names"] != names:
        raise SystemExit(f"joint order mismatch: plan has {stage['joint_names']}, limits are for {names}")
    jceil = a.jerk_limit if a.jerk_limit is not None else float(jlim.min())

    q_all = waypoints(stage)
    path = LsqPath(q_all, a.knot_spacing)
    dev = path.fidelity(q_all)
    k95, kmax = path.curvature()
    t_plan = float(stage["waypoints"][-1]["time_from_start"])
    arc = path.arc
    print(f"{a.plan.name} :: {stage.get('stage_name')}  ({q_all.shape[0]} waypoints, "
          f"joint arc {arc:.3f} rad, LSQ B-spline with {path.n_control} knots"
          f"{' — CAPPED by waypoint density' if path.knots_capped else ''})")
    print(f"  path fidelity vs every original waypoint: p95 "
          f"{np.degrees(np.percentile(dev,95)):.4f} deg, max {np.degrees(dev.max()):.4f} deg"
          f"   |q''| p95 {k95:.1f} max {kmax:.1f}")
    print(f"  as planned: {t_plan:.3f} s")
    print(f"  limits: vel {np.round(np.degrees(vlim),1)} deg/s | acc "
          f"{np.degrees(alim[0]):.0f} deg/s^2 | jerk ceiling {jceil:.0f} rad/s^3")

    def run(vs, acs):
        traj, _, _ = solve(path, vlim * vs, alim * acs, a.gridpoints)
        t, qd, qdd, jrk = sample(traj)
        return dict(T=float(traj.duration), v=util(qd, vlim), a=util(qdd, alim),
                    j_peak=float(np.abs(jrk).max()), j_p99=float(np.percentile(np.abs(jrk), 99)))

    rows = [("vel+acc optimal (no jerk limit)", a.vel_scale, a.acc_scale,
             run(a.vel_scale, a.acc_scale))]

    print(f"\n  {'case':30s} {'T (s)':>8s} {'vs plan':>9s} {'vel util':>9s} {'acc util':>9s} "
          f"{'jerk p99':>10s} {'jerk max':>10s}")
    for label, vs, acs, r in rows:
        print(f"  {label:30s} {r['T']:8.3f} {t_plan / r['T']:8.2f}x {r['v'].max():9.3f} "
              f"{r['a'].max():9.3f} {r['j_p99']:10.0f} {r['j_peak']:10.0f}")
    print(f"\n  NOTE the jerk columns are NOT a robot property: TOPP-RA's optimum switches\n"
          f"  acceleration bang-bang, so sampled jerk is ~2*a_max/dt. The {jceil:.0f} rad/s^3 ceiling\n"
          f"  is NOT enforced here. Treat T as a FLOOR; a jerk-limited profile is slower.")
    r0 = rows[0][3]
    print(f"\n  binding joint at the optimum: velocity J{int(np.argmax(r0['v'])) + 1} "
          f"({r0['v'].max():.3f}), acceleration J{int(np.argmax(r0['a'])) + 1} ({r0['a'].max():.3f})")
    print("  per-joint velocity util:    ", np.round(r0["v"], 3))
    print("  per-joint acceleration util:", np.round(r0["a"], 3))

    if a.emit_track:
        stage_names = [x.strip() for x in a.stages.split(",")] if a.stages else [stage.get("stage_name")]
        blocks, used, knots_used = [], [], {}
        print(f"\n  emitting at the {CLOCK * 1000:.4f} ms emit clock (the CANFD stream period), "
              f"acc scale {a.acc_scale} (per-stage, zero boundary velocity):")
        print(f"    {'stage':18s} {'wpts':>6s} {'planned':>9s} {'emitted':>9s} {'pts':>6s} "
              f"{'u_vel':>7s} {'u_acc':>7s} {'u_jerk':>7s}")
        for nm in stage_names:
            stg = stage_of(plan, nm, None)
            if stg["joint_names"] != names:
                raise SystemExit(f"stage {nm}: joint order {stg['joint_names']} != {names}")
            qa = waypoints(stg)
            pth = LsqPath(qa, a.knot_spacing)
            knots_used[nm] = {"knots": int(pth.n_control),
                              "achieved_spacing_rad": round(pth.arc / max(pth.n_control - 1, 1), 4),
                              "capped_by_waypoint_density": bool(pth.knots_capped)}
            v_emit = vlim * a.vel_scale
            if a.max_joint_rate_deg_s > 0:
                v_emit = np.minimum(v_emit, np.radians(a.max_joint_rate_deg_s))
            # The Cartesian caps are a CONTACT requirement — the pad is on the
            # surface — not a free-space one. Applied to the transits they cost
            # more than they buy: at w_cart 0.85 move_to_start went 0.705 -> 1.939 s,
            # SLOWER than the plan, because a free-space transit reorients the
            # wrist fast on purpose.
            contact = nm in {x.strip() for x in a.contact_stages.split(",")}
            vv = None
            if contact and a.v_cart_max > 0:
                vv = ToolSpeed(pth, side, a.ik_frame).bounds(
                    pth, v_emit, a.v_cart_max, a.w_cart_max)
            trj = solve(pth, v_emit, alim * a.acc_scale, a.gridpoints, vv)[0]
            qs, T_stage = resample_clock(trj, CLOCK)
            uv, ua, uj = band_limited_util(qs, CLOCK, vlim, alim, jceil, a.emit_fc_hz)
            t_stage = float(stg["waypoints"][-1]["time_from_start"])
            print(f"    {nm:18s} {qa.shape[0]:6d} {t_stage:8.3f}s {T_stage:8.3f}s "
                  f"{len(qs):6d} {uv:7.3f} {ua:7.3f} {uj:7.3f}")
            blocks.append(qs); used.append(nm)
        track, spans = concatenate(blocks, used)
        clk = CLOCK
        T = (track.shape[0] - 1) * clk
        tool = tool_profile(track, spans, side, a.ik_frame, CLOCK,
                            {x.strip() for x in a.contact_stages.split(",")})
        pos_ok = check_positions(track, side)
        md = np.round(np.degrees(track) * 1000.0).astype(np.int64)
        # judge the INTEGER millidegrees actually written, not the float array
        q_written = np.radians(md / 1000.0)
        uv, ua, uj = band_limited_util(q_written, clk, vlim, alim, jceil, a.emit_fc_hz)
        worst_rate, n_over, rate_per_joint = check_rate(q_written, clk, vlim)
        problems = []
        if min(pos_ok) < a.min_position_margin_deg:
            problems.append(f"joint-position margin {min(pos_ok):.3f} deg on "
                            f"J{int(np.argmin(pos_ok))+1} < {a.min_position_margin_deg} required")
        if ua > a.max_acc_util:
            problems.append(f"band-limited acceleration {ua:.3f} > {a.max_acc_util} of the limit")
        if uj > a.max_jerk_util:
            problems.append(f"band-limited jerk {uj:.3f} > {a.max_jerk_util} of the ceiling")
        if a.max_ticks_over_rate >= 0 and n_over > a.max_ticks_over_rate:
            problems.append(f"{n_over} ticks command over 180 deg/s (peak {worst_rate:.1f}); "
                            f"at most {a.max_ticks_over_rate} allowed")
        if problems:
            print("\n  REFUSED — the emitted track fails its own gates:")
            for x in problems:
                print(f"    * {x}")
            print("  Nothing was written. Lower --acc-scale and/or --v-cart-max and retry.")
            return 3
        with open(a.emit_track, "w", newline="") as fh:
            for row in md:
                fh.write('{"point":[' + ",".join(str(int(x)) for x in row) + "]}\n")
        car = a.emit_track.with_suffix(".json")
        car.write_text(json.dumps({
            "source": str(a.plan.resolve()),
            "source_sha256": hashlib.sha256(a.plan.read_bytes()).hexdigest(),
            "source_kind": "ros2_plan",
            "source_hardware": "ok",
            "why_ok": "generated by our task node against our description — not a recording from another mount",
            "emitter": "tools/plan_min_time.py --emit-track (TOPP-RA on the plan's GEOMETRY; the plan's own timing is discarded)",
            "stages": spans, "stage_names": used, "side": side, "frame": None,
            "clock_s": clk, "points": int(track.shape[0]), "track_duration_s": T,
            "vel_scale": a.vel_scale, "acc_scale": a.acc_scale,
            "knot_spacing_rad_requested": a.knot_spacing,
            "band_limited_util": {"fc_hz": a.emit_fc_hz, "vel": uv, "acc": ua, "jerk": uj,
                                  "measured_on": "the integer millidegrees written to the file"},
            "max_joint_rate_deg_s": worst_rate,
            "ticks_over_180_deg_s": n_over,
            "canfd_streamable": bool(n_over == 0),
            "per_joint_rate_deg_s": [float(x) for x in rate_per_joint],
            "v_cart_max_mps": a.v_cart_max, "w_cart_max_rad_s": a.w_cart_max,
            "contact_stages": a.contact_stages,
            "max_joint_rate_deg_s_requested": a.max_joint_rate_deg_s,
            "knots_per_stage": knots_used,
            "tool_speed_on_contact_stages": tool,
            "position_margin_deg": pos_ok,
            "position_margin_min_deg": float(min(pos_ok)),
            "position_margin_min_joint": int(np.argmin(pos_ok)) + 1,
        }, indent=2))
        t_seq = sum(float(stage_of(plan, n, None)["waypoints"][-1]["time_from_start"]) for n in used)
        print(f"\n    TRACK {track.shape[0]} points, {T:.3f} s   (the plan runs "
              f"{','.join(used)} in {t_seq:.3f} s -> {t_seq / T:.2f}x)")
        print(f"    band-limited at {a.emit_fc_hz:.0f} Hz (on the written integers): "
              f"u_vel {uv:.3f}  u_acc {ua:.3f}  u_jerk {uj:.3f}")
        print(f"    peak joint rate {worst_rate:.1f} deg/s, {n_over} ticks over 180 "
              f"({'CANFD-streamable' if n_over == 0 else 'UPLOAD TRANSPORT ONLY — over the CANFD guard'})")
        for st, d in tool.items():
            print(f"    {st}: tool linear p50 {d['linear_mps']['p50']:.3f} max "
                  f"{d['linear_mps']['max']:.3f} m/s | ANGULAR p50 "
                  f"{d['angular_rad_s']['p50']:.3f} max {d['angular_rad_s']['max']:.3f} rad/s")
        print(f"    closest joint-position limit: {min(pos_ok):.3f} deg (J{int(np.argmin(pos_ok))+1})")
        print(f"    wrote {a.emit_track} and {car.name}")

    if a.json:
        a.json.write_text(json.dumps(
            {"plan": str(a.plan), "stage": stage.get("stage_name"), "planned_s": t_plan,
             "arc_rad": arc, "jerk_ceiling": jceil,
             "knot_spacing_rad_requested": a.knot_spacing, "knots": int(path.n_control),
             "fidelity_max_deg": float(np.degrees(dev.max())),
             "cases": [{"label": l, "vel_scale": vs, "acc_scale": acs,
                        "T_s": r["T"], "speedup": t_plan / r["T"],
                        "vel_util": r["v"].tolist(), "acc_util": r["a"].tolist(),
                        "jerk_p99": r["j_p99"], "jerk_max": r["j_peak"]}
                       for l, vs, acs, r in rows]}, indent=2))
        print(f"\n  wrote {a.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
