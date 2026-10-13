"""`python3 -m alix_dispatch.cli CONFIG.yaml` — build and show the program.

Dry run by default: it resolves everything and prints the exact call sequence
without opening a connection. That is the offline half of contract C11 step 4b.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
from typing import Optional, Sequence

from alix_dispatch import config as cfgmod
from alix_dispatch import program as progmod
from alix_dispatch import schedule as schedmod
from alix_dispatch import connect, ladder, limits, recording, runmode, verdict


def _default_srdf() -> str:
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        p = parent / "ros" / "manipulation" / "alix_moveit_config" / "config" / "alix.srdf"
        if p.is_file():
            return str(p)
    raise FileNotFoundError("alix.srdf not found above " + str(here))


def _default_urdf() -> str:
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        p = (parent / "core" / "python" / "alix_workspace" / "src" / "alix_workspace"
             / "urdf" / "alix.urdf")
        if p.is_file():
            return str(p)
    raise FileNotFoundError("alix.urdf not found above " + str(here))


def _repo_root() -> pathlib.Path:
    """The alix checkout this module lives in, for suggesting a real path.

    By marker (ladder.repo_root), not parents[N]: from this file parents[4]
    is `core/`, so the "did you mean" search used to look under core/ only.
    """
    return ladder.repo_root()


def _line_acc_for_run(args, cfg, line_speed):
    """The single acceleration a run uses — written AND planned against.

    The task's `cleaning_line_acc_mps2` when it has one, floored by C2's
    `max(1.6, 3v + 0.01)`; that floor is not decoration, the controller
    refuses a boundary pair with a bare ret=1 and silently keeps the old value.
    """
    if not line_speed:
        return None
    floor = limits.line_acc_for(float(line_speed))
    want = cfg.cleaning_line_acc_mps2
    return floor if want is None else max(float(want), floor)


def _write_verdict(recorder, prog, side, rung, sim):
    """Run C9's completion check and put it IN run.json. `(ok, text, detail)`.

    Called on BOTH paths. A run that raised is the one you most want the number
    for — front_right's two SIM runs returned 0 from the SDK, traced 4 % of the
    stroke, and read as passes for days — and until 2026-08-21 a failed run
    wrote no verdict at all, because the exception path returned before the
    check.
    """
    if recorder is None:
        return True, "no recording — completion NOT checked", {}
    commanded = verdict.commanded_path(prog, side=side)
    commanded = verdict.commanded_path(prog, side=side,
                                       stage=commanded["chain_stage"])
    window = recorder.stage_window(side, commanded["chain_stage"])
    ok, text, detail = verdict.verdict(
        rung.lower(), commanded, recorder.rows(side), recorder.header(),
        window, sim=bool(sim))
    try:
        rec = pathlib.Path(str(getattr(recorder, "dir", ""))) / "run.json"
        if rec.is_file():
            doc = json.loads(rec.read_text())
            doc["verdict"] = {
                "rung": rung, "ok": bool(ok), "text": text,
                "tracedRatio": detail.get("traced_ratio"),
                "tracedWindowedM": detail.get("traced_windowed_m"),
                "commandedM": detail.get("commanded_m"),
                "finalWaypointMissM": detail.get("final_waypoint_miss_m"),
                "claims": detail.get("claims"),
                "tracedFloor": detail.get("traced_floor",
                                          verdict.TRACED_FLOOR),
            }
            rec.write_text(json.dumps(doc, indent=2))
    except Exception as exc:  # noqa: BLE001
        print(f"  [WARN] verdict not written into run.json: {exc!r}")
    return ok, text, detail


def _resolve_motion(args, cfg):
    """`(line_speed, motion, error)` — the motion policy both builds share.

    Resolved from the CONFIG as well as the flags, so it has to be worked out
    before `build`, not after: a per-segment speed is a fraction of the TCP
    constraint that will be written, and a turn radius is only meaningful once
    that constraint is known. Returning it lets the preview build and the
    executing rebuild plan the SAME program — before this, a dry run printed a
    program with none of the motion policy in it and read as though that were
    what would be sent.
    """
    line_speed = None
    if args.line_speed is not None:
        if str(args.line_speed).strip().lower() == "config":
            line_speed = cfg.cleaning_line_speed_mps
            if line_speed is None:
                return None, None, ("--line-speed config: the task carries no "
                                    "`cleaning_line_speed_mps`")
        else:
            try:
                line_speed = float(args.line_speed)
            except ValueError:
                return None, None, (f"--line-speed takes a number or `config` "
                                    f"(got {args.line_speed!r})")

    turn = None
    if args.turn_radius is not None:
        if str(args.turn_radius).strip().lower() == "config":
            turn = cfg.turn_policy()
            if turn is None:
                return None, None, ("--turn-radius config: the task carries no "
                                    "`cleaning_turn_radius_mm`")
        else:
            try:
                turn = {"radius_m": float(args.turn_radius) / 1000.0}
            except ValueError:
                return None, None, (f"--turn-radius takes millimetres or "
                                    f"`config` (got {args.turn_radius!r})")
        # Explicit flags win over the file, so a config policy can be probed
        # one field at a time without editing it.
        if args.turn_speed is not None:
            turn["speed_cap_mps"] = args.turn_speed
        if args.turn_overtravel is not None:
            turn["overtravel_m"] = args.turn_overtravel / 1000.0
        if args.turn_surface_tol is not None:
            turn["surface_tol_m"] = args.turn_surface_tol / 1000.0
        if args.turn_allow_step:
            turn["allow_step"] = True
        turn.setdefault("cruise_mps", line_speed)
        # ONE acceleration. `a_lat` here and the value the session writes must
        # be the same number, or the policy prices corners against physics the
        # arm was never given (2026-08-21: policy 3.2, written 2.41).
        turn.setdefault("a_lat", _line_acc_for_run(args, cfg, line_speed))

    wants = args.per_segment_speed or args.auto_blend or turn
    if wants and not line_speed:
        return None, None, ("--per-segment-speed / --auto-blend / --turn-radius "
                            "need --line-speed: all three are relative to the "
                            "TCP constraint that will be written.")
    motion = None
    if wants:
        motion = {"line_speed": line_speed,
                  "per_segment": args.per_segment_speed,
                  "auto_blend_cap": args.auto_blend,
                  "turn": turn}
    return line_speed, motion, None


def build(path: str, *, srdf: Optional[str] = None, urdf: Optional[str] = None,
          blend_pct: Optional[int] = None, pole_start_m: Optional[float] = None,
          exec_mode: int = 1, plan_path: Optional[str] = None,
          budget_mm: float = 10.0, max_targets: int = 12,
          placement: str = "greedy", stream_rate_hz: float = 100.0,
          stream_duration_s: float = 4.0, motion: Optional[dict] = None):
    """Resolve a config into a program for one of C16's three execution modes.

    All three share mode 1's builder and differ only in how a FREE-SPACE arm
    move is expressed — planned primitive (1), a few chained joint targets from
    MoveIt's plan (2), or a CANFD passthrough stream (3). Cleaning is a
    `movel` chain in every one of them; that is C16's design, not a shortcut.
    """
    from alix_workspace.srdf_states import SrdfStates
    from alix_workspace.urdf_kinematics import UrdfModel
    cfg = cfgmod.load_file(path)
    states = SrdfStates.from_file(srdf or _default_srdf())
    model = UrdfModel.from_file(urdf or _default_urdf())
    if exec_mode == 1:
        prog = progmod.build_program(cfg, states, model, blend_pct=blend_pct,
                                     pole_start_m=pole_start_m, motion=motion)
        prog.mode = 1                                    # type: ignore[attr-defined]
        return cfg, prog
    if exec_mode == 2:
        from alix_dispatch import mode2
        if not plan_path:
            raise progmod.DispatchError(
                "mode 2 needs --plan: MoveIt's plan IS the free-space "
                "description it executes sparsely. Without one there is nothing "
                "to reduce and it would silently be mode 1.")
        return cfg, mode2.build_program(
            cfg, states, model, plan_path, budget_mm=budget_mm,
            max_targets=max_targets, placement=placement,
            blend_pct=blend_pct, pole_start_m=pole_start_m)
    if exec_mode == 3:
        from alix_dispatch import mode3
        return cfg, mode3.build_program(
            cfg, states, model, plan_path=plan_path, rate_hz=stream_rate_hz,
            duration_s=stream_duration_s, blend_pct=blend_pct,
            pole_start_m=pole_start_m)
    raise progmod.DispatchError(f"unknown execution mode {exec_mode!r}; C16 has 1, 2, 3")


def render(prog) -> str:
    out = [f"program: {prog.task}  [{prog.side}]   {prog.summary()}"]
    for i, c in enumerate(prog.calls, 1):
        d = c.detail
        if c.kind == "scene":
            line = f"{i:>3}. {c.stage:<28} scene   (no device)"
        elif c.kind == "lift":
            line = (f"{i:>3}. {c.stage:<28} lift    {d['height_m']*1000:6.1f} mm "
                    f"({d['state']}) @ {d['speed_pct']}%")
        elif c.kind == "hand":
            line = (f"{i:>3}. {c.stage:<28} hand    {d['state']:<12} "
                    f"{len(d['joints_rad'])} joints @ {d['speed_pct']}%")
        elif c.kind == "movej":
            j = ", ".join(f"{v:7.2f}" for v in d["joints_deg"])
            line = f"{i:>3}. {c.stage:<28} movej   [{j}] deg @ {d['speed_pct']}%"
        elif c.kind in ("movel", "movej_p"):
            p = d["pose"]
            line = (f"{i:>3}. {c.stage:<28} {c.kind:<7} "
                    f"({p['x']:.4f}, {p['y']:.4f}, {p['z']:.4f}) m "
                    f"rpy({p['rx']:.3f}, {p['ry']:.3f}, {p['rz']:.3f}) @ {d['speed_pct']}%")
        elif c.kind == "movel_chain":
            first, last = d["moves"][0]["pose"], d["moves"][-1]["pose"]
            # COUNT THE MOVES, not the points: an arc span collapses the
            # waypoints between its endpoints into one rm_movec, so `points`
            # (traversed) and the number of commands stopped being the same
            # number once ring spans started dispatching (2026-08-20).
            n_mv, n_arc = len(d["moves"]), d.get("arc_moves", 0)
            shape = (f"{n_mv - n_arc} movel + {n_arc} movec"
                     if n_arc else f"{n_mv} movel")
            line = (f"{i:>3}. {c.stage:<28} chain   {shape} over "
                    f"{d['points']} points, "
                    f"connect=1 x{n_mv-1} then 0 | "
                    f"first ({first['x']:.3f}, {first['y']:.3f}, {first['z']:.3f}) "
                    f"last ({last['x']:.3f}, {last['y']:.3f}, {last['z']:.3f})"
                    + (f" | line_speed {d['line_speed_mps']} m/s" if d.get("line_speed_mps") else ""))
        elif c.kind == "movej_chain":
            line = (f"{i:>3}. {c.stage:<28} jchain  {d['targets']} joint targets "
                    f"(from {d['dense']} waypoints, {d['placement']}), "
                    f"connect=1 x{d['targets']-1} then 0 | "
                    f"deviation {d['max_tip_dev_mm']:.2f} mm of "
                    f"{d['budget_mm']:.0f} mm | replaces {d['replaces']}")
        elif c.kind == "movej_canfd":
            rate = float(d.get("rate_hz") or 100.0)
            frames = int(round(d["duration_s"] * rate))
            src = d.get("source", "cosine ease")
            line = (f"{i:>3}. {c.stage:<28} canfd   {d['duration_s']:.2f} s "
                    f"commanded @ {rate:.0f} Hz (~{frames} frames), {src}"
                    + (f" of {d['dense']} waypoints" if d.get("dense") else "")
                    + f" | replaces {d['replaces']}")
        else:
            line = f"{i:>3}. {c.stage:<28} {c.kind}"
        out.append(line)
    for w in prog.warnings:
        out.append(f"  ! {w}")
    return "\n".join(out)


def render_schedule(plan) -> str:
    """The dual plan, with what each arm does between rendezvous points."""
    lead, follow = plan.arms
    width = max(len(lead), len(follow))
    out = [f"schedule: {plan.mode}   leader={lead}  follower={follow}"]
    for i, name in enumerate(plan.rendezvous):
        out.append(f"  phase {i} -> rendezvous {name or '<tail>'}")
        for side in (lead, follow):
            stages = plan.phases[side][i].stages
            out.append(f"      {side:>{width}}: " + (", ".join(stages) or "-"))
    for w in plan.warnings:
        out.append(f"  ! {w}")
    return "\n".join(out)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("config", nargs="+",
                    help="one task config, or TWO single-arm configs to schedule "
                         "across both controllers")
    ap.add_argument("--srdf"), ap.add_argument("--urdf")
    ap.add_argument("--exec-mode", type=int, choices=(1, 2, 3), default=1,
                    help="C16 execution mode: 1 = config drives the controller "
                         "primitives, 2 = MoveIt plans free space and the "
                         "controller executes it as a few chained joint targets, "
                         "3 = free space as CANFD passthrough. Cleaning is a "
                         "movel chain in all three.")
    ap.add_argument("--plan", default=None,
                    help="saved MoveIt plan JSON. Required by mode 2; mode 3 "
                         "uses it to stream MoveIt's own joint path (and it is "
                         "the only way a Cartesian stage can stream at all).")
    ap.add_argument("--budget-mm", type=float, default=10.0,
                    help="mode 2: how far a joint-linear shortcut may deviate "
                         "from MoveIt's path")
    ap.add_argument("--max-targets", type=int, default=12,
                    help="mode 2: give up past this many targets per stage")
    ap.add_argument("--placement", choices=("greedy", "even"), default="greedy",
                    help="mode 2: greedy puts targets where the path BENDS "
                         "(22%% fewer for the same budget, F87)")
    ap.add_argument("--stream-rate", type=float, default=100.0,
                    help="mode 3: CANFD frame rate (Hz)")
    ap.add_argument("--stream-duration", type=float, default=4.0,
                    help="mode 3: commanded duration for a stage with no plan")
    ap.add_argument("--blend", type=int, default=None,
                    help="per-move r (0-100). Omitted = not specified, per C16.4")
    ap.add_argument("--pole", type=float, default=None,
                    help="pole height (m) in effect before the first pole stage")
    ap.add_argument("--dual-mode", choices=schedmod.MODES, default="sequential",
                    help="how two single-arm programs share the two controllers "
                         "(RM_API2 C2/C3/C4 semantics)")
    ap.add_argument("--leader", choices=("left", "right"), default=None,
                    help="which arm leads in chained mode")
    ap.add_argument("--cross-arm-mm", type=float, default=None,
                    help="measured MINIMUM left-to-right separation over the "
                         "concurrent window. Required by every concurrent mode: "
                         "cross-arm collision validity is R6, still open, and no "
                         "default is invented here (C16.4)")
    ap.add_argument("--cross-arm-threshold-mm", type=float, default=None,
                    help="the margin you are accepting, in mm")
    ap.add_argument("--cross-arm-source", default=None,
                    help="what produced --cross-arm-mm, so a green run is "
                         "attributable")
    # ── C9's ladder: emulator -> SIM -> REAL ──────────────────────────────
    ap.add_argument("--execute", action="store_true",
                    help="open connections and RUN the program "
                         "(default: render only, opening nothing)")
    ap.add_argument("--mode", type=runmode.parse_mode_arg, default=None,
                    help="engage and VERIFY the run mode before any motion "
                         "(restored on exit); without it the run goes as-found")
    ap.add_argument("--left-ip", default=None)
    ap.add_argument("--right-ip", default=None)
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--udp-port", type=int, default=connect.DEFAULT_UDP_PORT)
    ap.add_argument("--udp-cycle", type=int, default=2)
    ap.add_argument("--no-record", action="store_true",
                    help="skip the UDP recording. REFUSED on SIM and REAL: C9 "
                         "makes the completion check mandatory, and a run with "
                         "no recording has no evidence at all")
    ap.add_argument("--run-dir", default=None)
    ap.add_argument("--runs-root", default=None)
    ap.add_argument("--gui-watcher", default=None,
                    help="name of the human watching the controller GUI. The "
                         "failure code is GUI-only, so an unwatched REAL run "
                         "cannot see its own abort")
    ap.add_argument("--line-speed", default=None, metavar="MPS|config",
                    help="WRITE the controller's TCP speed constraint before "
                         "the run (restored after). Without it, a commanded "
                         "v%% scales against whatever the arm already holds — "
                         "which is why a config saying 0.8 ran at 0.5. "
                         "Pass `config` to take the value from the task file")
    ap.add_argument("--angular-speed", type=float, default=None, metavar="RADS")
    ap.add_argument("--per-segment-speed", action="store_true",
                    help="command each stroke segment at the speed A.1 says "
                         "its own length can reach, instead of one global v. "
                         "Turning points are excluded — they are short hops "
                         "that reach no cap and are traced by movec")
    ap.add_argument("--auto-blend", type=int, default=None, metavar="MAX_R",
                    help="compute each corner's blend radius from A.2 "
                         "(cut = c(r)*(r/100)*min(L_in,L_out), c piecewise) "
                         "capped at MAX_R percent. Without it every corner is "
                         "sent r=0, which is a FULL STOP")
    ap.add_argument("--turn-radius", default=None, metavar="MM|config",
                    help="THREE-FIELD TURN POLICY. Cruise on the straights, "
                         "this radius at the corners. A turn speed is not an "
                         "input — A.2.1 makes it sqrt(a_lat*R) — so what you "
                         "set is the radius, and A.2 picks the smallest blend "
                         "r that delivers it. `config` reads "
                         "`cleaning_turn_radius_mm` from the task")
    ap.add_argument("--turn-speed", type=float, default=None, metavar="MPS",
                    help="ceiling on any one turn, for when the geometry "
                         "would allow more than you want to give it. Never "
                         "raises a turn — the radius still binds")
    ap.add_argument("--turn-overtravel", type=float, default=None, metavar="MM",
                    help="how far past the workpiece a turnaround may reach. "
                         "With room a serpentine turn becomes the tangent "
                         "semicircle of radius pitch/2, which deletes both "
                         "corners and costs NO surface; without it every "
                         "corner is a fillet that eats its own vertex")
    ap.add_argument("--turn-surface-tol", type=float, default=None,
                    metavar="MM",
                    help="how much corner a fillet may discard, default 5 mm "
                         "(inside the pad's own footprint, so what it drops is "
                         "still swept). THIS is usually what limits a turn, not "
                         "the radius: a fillet misses its vertex by "
                         "R*(sec(phi/2)-1) — 0.41*R at 90 degrees, 4.97*R at "
                         "161. Raising it trades coverage for speed")
    ap.add_argument("--turn-allow-step", action="store_true",
                    help="DO NOT enforce A.2.2 — let a junction carry a speed "
                         "step bigger than its blend cut. The 2026-08-21 SIM "
                         "run is evidence against it: the chain came back "
                         "ret=1 and the arm stopped inside the first such "
                         "blend, 12.3 mm short of point15 against a 12.8 mm "
                         "cut that needed 90.2 mm. Kept only to reproduce that")
    ap.add_argument("--turn-strict-step", action="store_true",
                    help=argparse.SUPPRESS if False else "no-op: A.2.2 is now "
                         "enforced by default. Was: apply A.2.2 as a GATE. The "
                         "contract confines a speed step to the blend itself, "
                         "which binds an 0.80 m/s row entering an 0.28 m/s "
                         "turn down to 0.53. Off by default because the "
                         "2026-08-20 run showed the controller braking along "
                         "the straight, which A.2.2's model does not allow")
    ap.add_argument("--joint-speed", action="append", default=None,
                    metavar="J:DEG_S",
                    help="cap one joint's velocity for this run, e.g. "
                         "--joint-speed 1:150 (restored afterwards). Lowers "
                         "only. Asks the controller whether it RETIMES a "
                         "trajectory to respect a joint limit or refuses it — "
                         "the offline solver cannot answer that")
    ap.add_argument("--allow-real-without-sim", default=None, metavar="REASON",
                    help="attempt REAL with no SIM record for this exact "
                         "program. Does NOT override a SIM run that truncated")
    args = ap.parse_args(argv)

    try:
        progs, cfgs = [], []
        for path in args.config:
            _, motion, err = _resolve_motion(args, cfgmod.load_file(path))
            if err:
                print(err, file=sys.stderr)
                return 2
            cfg, prog = build(path, srdf=args.srdf, urdf=args.urdf,
                              blend_pct=args.blend, pole_start_m=args.pole,
                              exec_mode=args.exec_mode, plan_path=args.plan,
                              budget_mm=args.budget_mm,
                              max_targets=args.max_targets,
                              placement=args.placement,
                              stream_rate_hz=args.stream_rate,
                              stream_duration_s=args.stream_duration,
                              motion=motion)
            progs.append(prog)
            cfgs.append(cfg)
    except FileNotFoundError as exc:
        # A path typo is the commonest way to run this wrong, and a traceback
        # is a poor way to say so.
        print(f"no such config: {exc.filename}", file=sys.stderr)
        near = pathlib.Path(exc.filename or "").name
        hits = sorted(pathlib.Path(_repo_root()).rglob(near))[:5] if near else []
        for hit in hits:
            print(f"  did you mean: {hit}", file=sys.stderr)
        return 2
    except progmod.DispatchError as exc:
        print(f"DISPATCH REFUSED: {exc}", file=sys.stderr)
        return 2

    for prog in progs:
        print(render(prog))
        if args.exec_mode == 2:
            from alix_dispatch import mode2
            print(mode2.describe(prog))
        elif args.exec_mode == 3:
            from alix_dispatch import mode3
            print(mode3.describe(prog))
            ok, _, probs = mode3.enforce_stream_interlock(prog)
            if not ok:
                print("STREAM INTERLOCK VIOLATED:", file=sys.stderr)
                for why in probs:
                    print(f"  {why}", file=sys.stderr)
                return 2
        print()

    # ── refusals that must happen BEFORE anything opens a connection ──────
    if args.mode is not None and not args.execute:
        print("--mode without --execute would print text and change nothing, "
              "while reading afterwards as though the run happened in that "
              "mode. Add --execute, or drop --mode.", file=sys.stderr)
        return 2
    if args.execute and args.no_record and args.mode is not None:
        print(f"--no-record is refused on the "
              f"{runmode.mode_label(args.mode)} rung: C9 makes the completion "
              "check mandatory, and a run with no recording has no evidence.",
              file=sys.stderr)
        return 2
    if args.execute and args.mode == runmode.REAL and not args.gui_watcher:
        print("a REAL rung needs --gui-watcher NAME: the controller's failure "
              "code is GUI-only, so an unwatched run cannot see its own abort.",
              file=sys.stderr)
        return 2

    if args.execute:
        return _execute(progs, cfgs, args)

    if len(progs) == 1:
        if args.dual_mode != "sequential":
            print("--dual-mode needs two configs; one config is one arm.",
                  file=sys.stderr)
            return 2
        return 0

    clearance = None
    if args.cross_arm_mm is not None:
        if args.cross_arm_threshold_mm is None or not args.cross_arm_source:
            print("--cross-arm-mm needs --cross-arm-threshold-mm and "
                  "--cross-arm-source: an unattributed margin is not a verdict.",
                  file=sys.stderr)
            return 2
        clearance = schedmod.CrossArmClearance(
            args.cross_arm_mm, args.cross_arm_threshold_mm, args.cross_arm_source)
    try:
        plan = schedmod.plan_dual({p.side: p for p in progs}, args.dual_mode,
                                  clearance=clearance, leader=args.leader)
    except schedmod.ScheduleError as exc:
        print(f"SCHEDULE REFUSED: {exc}", file=sys.stderr)
        return 2
    print(render_schedule(plan))
    return 0


def _execute(progs, cfgs, args) -> int:
    """Run the program up one rung of C9's ladder.

    Exit codes: 0 pass, 1 the run failed, 2 refused before dispatch,
    3 the ladder blocked this rung, 4 hardware not reachable (a SKIP).
    """
    from alix_dispatch.session import RunSession
    from alix_dispatch.runmode import RunModeError

    if len(progs) != 1:
        print("--execute currently drives ONE program; dual scheduling through "
              "a session is not wired yet.", file=sys.stderr)
        return 2

    prog = progs[0]
    side = prog.side
    if side == "dual":
        print("--execute does not yet drive a compound (two-arm) program.",
              file=sys.stderr)
        return 2

    ips = {"left": args.left_ip, "right": args.right_ip}
    ips = {k: v for k, v in ips.items() if v}
    fingerprint = verdict.program_fingerprint(prog, blend_pct=args.blend)
    root = ladder.runs_root(args.runs_root)
    rung = "SIM" if args.mode == runmode.SIMULATION else (
        "REAL" if args.mode == runmode.REAL else "EMU")

    # ── the ladder is ADVISORY (Newton, 2026-08-20): "I am the one who
    # decides the mode to run, either EMU, SIM or REAL; do not force a run of
    # SIM in REAL." So the prior rung is REPORTED and recorded — the evidence
    # trail C9 wants is still written — but the operator picks the rung.
    if rung == "SIM":
        clear, why = ladder.gate_sim(root, fingerprint)
    elif rung == "REAL":
        clear, why, _ = ladder.gate_real(
            root, fingerprint, override_reason=args.allow_real_without_sim)
    else:
        clear, why = True, "emulator rung is not gated"
    print(f"ladder: {rung} — {why}")
    if not clear:
        print(f"  [WARN] proceeding anyway on the operator's decision")

    # The line speed the run will WRITE. `--line-speed config` takes the
    # task's own `cleaning_line_speed_mps`, which is the number the config has
    # been claiming all along without anything acting on it.
    cfg = cfgs[0]
    line_speed = None
    line_speed, motion, err = _resolve_motion(args, cfg)
    if err:
        print(err, file=sys.stderr)
        return 2
    angular = args.angular_speed
    if line_speed and angular is None:
        angular = cfg.params.get("cleaning_angular_cap_rad_s")

    joint_caps = {}
    for spec in (args.joint_speed or []):
        try:
            joint, value = spec.split(":")
            joint_caps[int(joint)] = float(value)
        except ValueError:
            print(f"--joint-speed takes J:DEG_S, e.g. 1:150 (got {spec!r})",
                  file=sys.stderr)
            return 2

    # REBUILD with the motion policy. `main` already built this program with
    # the SAME policy — `_resolve_motion` is shared precisely so the preview
    # and the run cannot diverge — but `prog` was passed in from that build and
    # rebuilding here keeps `_execute` runnable on its own.
    if motion:
        _, prog = build(str(args.config[0]), srdf=args.srdf, urdf=args.urdf,
                        blend_pct=args.blend, pole_start_m=args.pole,
                        motion=motion)
        chain = next((c for c in prog.calls if c.kind == "movel_chain"), None)
        if chain:
            mv = chain.detail["moves"]
            vs = sorted({m["v_pct"] for m in mv})
            rs = sorted({(m["r"] or 0) for m in mv})
            print(f"motion policy: v% {vs[0]}..{vs[-1]} over {len(mv)} moves, "
                  f"r% {rs[0]}..{rs[-1]}, {chain.detail['arc_moves']} movec")
        for w in prog.warnings:
            if w.startswith(("turn policy:", "A.2.2 ", "TURN POLICY")) \
                    or "turnaround(s)" in w or "bound by SURFACE" in w:
                print(f"  {w}")

    recorder = None
    arm_ips = {side: ips.get(side) or connect.DEFAULT_IPS[side]}
    if not args.no_record:
        recorder = recording.RunRecorder(
            {side: None}, task=prog.task, arm_ips=arm_ips,
            host_ips={side: connect.host_ip_for(arm_ips[side])},
            udp_port=args.udp_port, cycle=args.udp_cycle, out_dir=args.run_dir)

    try:
        with RunSession([side], task=prog.task, mode=args.mode, ips=ips,
                        port=args.port, recorder=recorder,
                        line_speed=line_speed, angular_speed=angular,
                        line_acc=_line_acc_for_run(args, cfg, line_speed),
                        joint_speed=joint_caps) as session:
            session.execute(prog)
            record = session.run_record()
            program_ok = True
    except RunModeError as exc:
        print(f"NOT REACHABLE / MODE REFUSED: {exc}", file=sys.stderr)
        return 4 if "not reachable" in str(exc) else 1
    except progmod.DispatchError as exc:
        print(f"DISPATCH REFUSED: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        # A PROGRAM failure — a crash or a rejected command. C9 gates SIM on
        # this happening at the emulator rung, so it is recorded either way.
        # The completion check runs HERE TOO: how far it got before the refusal
        # is the whole diagnosis, and leaving it out is what made three
        # rejected runs look identical when they stopped in different places.
        try:
            _, text, det = _write_verdict(recorder, prog, side, rung, True)
            got = det.get("traced_windowed_m")
            want = det.get("commanded_m")
            if got is not None and want:
                print(f"  reached {got:.3f} m of {want:.3f} m "
                      f"({100.0 * got / want:.1f} %) before the failure — {text}")
        except Exception:  # noqa: BLE001 — diagnosis must not mask the failure
            pass
        ladder.record_rung(root, fingerprint=fingerprint, rung=rung, ok=False,
                           program_ok=False, reason=repr(exc),
                           run_dir=str(getattr(recorder, "dir", "")),
                           sdk="unknown",
                           run_id=getattr(recorder, "run_id", ""))
        print(f"RUN FAILED: {exc}", file=sys.stderr)
        return 1

    # ── C9's mandatory completion check ──────────────────────────────────
    ok, text, detail = _write_verdict(recorder, prog, side, rung,
                                      record.get("sim"))
    print(f"verdict [{rung}]: {'PASS' if ok else 'FAIL'} — {text}")
    print(f"  claims: {detail.get('claims', '')}")
    print(f"  joints: {detail.get('joint_criteria', '')}")

    ladder.record_rung(root, fingerprint=fingerprint, rung=rung, ok=bool(ok),
                       program_ok=program_ok, reason=text,
                       run_dir=str(getattr(recorder, "dir", "")),
                       sdk=(record.get("sdk") or {}).get(side, "unknown"),
                       run_id=getattr(recorder, "run_id", ""),
                       geometry_note=("the emulator cannot validate arc geometry"
                                      if rung == "EMU" else ""))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
