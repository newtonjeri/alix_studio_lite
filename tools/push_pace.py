"""How fast a stored program plays with the controller's realtime push off, at 10 ms and at 20 ms: the lab
measurement before S2 records the controllers' feed during its runs (the owner, 23 Oct: "We can use 10ms or 20ms").
With the push on at 5 ms, programs played some 13 % slower (21 Oct), past S2's 105 % stop.

    PYTHONPATH=src python3 tools/push_pace.py --side left --slot 59 --track <the slot's .txt> [--cycles 0,2,4]

Close Lite first: this connects to the arm itself. Each run, as S2 runs a Program:
- the overall speed set as S2 sets it (--overall, the speed slider; put back to --restore at the end);
- a lone 4105 warning cleared, as Lite's clear step does (any other code stops the measurement);
- the arm to the program's first point (MoveJ at --speed %, checked within 1 degree);
- the push set: cycle 0 = off; else on toward this computer every cycle x 5 ms, recorded to a file;
- the program started (non-blocking, as Lite starts it) and timed from the controller's "running" to its end:
  ms per line = that time / (lines - 1).
The push goes back to what it was, whatever happens (ALIX's UdpRecorder). The summary gives each cycle's slowdown
against the push off and the verdict (``summary``): S2 stops a Program whose lines take more than 105 % of 5 ms each.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

from alix_studio_lite import vendor_paths

vendor_paths.activate()

FRESH = "LITE_PUSH_PACE_FRESH"                 # set when this tool restarted itself in a fresh process


def first_point_and_lines(track: Path) -> tuple[list[float], int]:
    rows = [json.loads(line)["point"] for line in track.read_text(encoding="utf-8").splitlines() if line.strip()]
    return [v / 1000.0 for v in rows[0]], len(rows)


def play(robot, slot: int, transit_pct: int, timeout_s: float) -> tuple[float | None, str]:
    """Start ``slot`` (non-blocking) and time it from the controller's "running" to its end: (seconds, what happened)."""
    ret = robot.rm_set_program_id_run(slot, transit_pct, 0)
    if ret != 0:
        return None, f"not started (rm_set_program_id_run ret={ret})"
    t0, t_run = time.monotonic(), None
    while True:
        now = time.monotonic()
        r, st = robot.rm_get_program_run_state()
        rs = st.get("run_state") if r == 0 else None
        if rs in (1, 2) and t_run is None:
            t_run = now
        if t_run is not None and rs == 0:
            return now - t_run, "ended"
        if t_run is None and now - t0 > 10.0:
            return None, "did not start within 10 s"
        if now - t0 > timeout_s:
            robot.rm_set_arm_slow_stop()
            return None, f"still running after {timeout_s:g} s: slow-stopped"
        time.sleep(0.02)


def frame_period_ms(capture: Path) -> float | None:
    """The push's period as it arrived: the recorded frames' own times (None: fewer than 2 frames)."""
    try:
        ts = [json.loads(line)["t_mono"] for line in capture.read_text(encoding="utf-8").splitlines() if line.strip()]
    except (OSError, ValueError, KeyError):
        return None
    return (ts[-1] - ts[0]) / (len(ts) - 1) * 1000.0 if len(ts) > 1 else None


def lite_running() -> list[int]:
    """Lite's servers on this computer (their command line), this tool aside."""
    out = []
    for d in Path("/proc").glob("[0-9]*"):
        try:
            cmd = (d / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if "alix_studio_lite.server" in cmd and int(d.name) != os.getpid():
            out.append(int(d.name))
    return out


def clear_lone_warning(arm) -> str | None:
    """A lone 4105 cleared as Lite's clear step does (RealArm.clear_warning): None when the arm is clean after it, else
    what stays latched. The SDK's blocking MoveJ returns 1 at once while any code is latched."""
    from alix_studio_lite.server.robot_real import _dac

    dac = _dac()
    st = dac.error_state(arm)
    if not st["readable"]:
        return "the arm's errors could not be read"
    if dac.error_state_clean(st):
        return None
    if set(st["sys"]) - {"4105"} or st["joints"] or st.get("lift_err") or st.get("disabled"):
        return dac.describe_error_state(st)
    dac.clear_errors(arm)
    back = dac.error_state(arm)
    return None if dac.error_state_clean(back) else dac.describe_error_state(back)


def main(argv=None) -> int:
    from alix_dispatch import connect

    from alix_studio_lite.server import udp_push
    from alix_studio_lite.server.robot_real import UDP_PORT, CycleRecorder

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--side", choices=("left", "right"), required=True)
    ap.add_argument("--ip", help="the controller's address (default: ALIX's for the side, 192.168.1.10 / .103)")
    ap.add_argument("--slot", type=int, required=True, help="the stored program to play")
    ap.add_argument("--track", type=Path, required=True, help="the program's .txt: its lines and its first point")
    ap.add_argument("--cycles", default="0,2,4", help="push cycles to compare, in 5 ms units (0 = off): default 0,2,4")
    ap.add_argument("--runs", type=int, default=1, help="runs per cycle")
    ap.add_argument("--overall", type=int, default=15, help="the overall speed %% S2 runs at (its speed slider)")
    ap.add_argument("--restore", type=int, default=50, help="the overall speed %% put back at the end")
    ap.add_argument("--speed", type=int, default=15, help="MoveJ speed %% to the first point")
    ap.add_argument("--transit", type=int, default=15, help="the program's start/transit speed %%")
    ap.add_argument("--yes", action="store_true", help="do not ask before moving the arm")
    a = ap.parse_args(argv)
    cycles = [int(c) for c in a.cycles.split(",")]
    first, lines = first_point_and_lines(a.track)
    if lite_running():
        print("** Lite is running (process " + ", ".join(map(str, lite_running())) + "): close it first, so nothing else "
              "commands the arm or holds its feed's port")
        return 1
    h = connect.open_arm(a.side, ip=a.ip)
    if h is None:
        print(f"** the {a.side.upper()} controller could not be reached" + (f" at {a.ip}" if a.ip else ""))
        return 1
    robot = h.robot
    left = udp_push.left_on(h)
    if left is not None:
        if left["holders"]:
            print(f"** the controller's push is on toward this computer's port {left['port']}, which another program holds "
                  f"({udp_push.held_text(left['holders'])}): stop it first")
            return 1
        if left["cleared"] and not os.environ.get(FRESH):
            print("  the push had been left on toward this computer: turned off; starting again in a fresh process "
                  "(this one's SDK holds the port now)")
            os.environ[FRESH] = "1"
            os.execv(sys.executable, [sys.executable] + sys.argv)
    print(f"{a.side.upper()} {h.ip}: program {a.slot}, {lines} lines; push cycles {cycles} x 5 ms (0 = off); "
          f"overall {a.overall} %")
    if not a.yes and input(f"  the arm moves to the program's first point and plays it {len(cycles) * a.runs} times: "
                           f"type '{a.side}' to start: ").strip().lower() != a.side:
        print("  not started")
        return 1
    arm = SimpleNamespace(robot=robot, side=a.side)
    out = []
    folder = Path(tempfile.mkdtemp(prefix="push_pace_"))
    try:
        print(f"  overall speed {a.overall} % (rm_set_plan_speed ret={robot.rm_set_plan_speed(a.overall)})")
        for c in cycles:
            for k in range(a.runs):
                latched = clear_lone_warning(arm)
                if latched:
                    print(f"  ** the arm holds {latched}: clear it at the pendant, then run this again")
                    return 1
                ret = robot.rm_movej(first, a.speed, 0, 0, 1)
                r, st = robot.rm_get_current_arm_state()
                gap = max(abs(x - y) for x, y in zip(st["joint"], first)) if r == 0 else None
                if ret != 0 or gap is None or gap > 1.0:
                    print(f"  ** the arm did not reach the program's first point (MoveJ ret={ret}, "
                          f"{'?' if gap is None else f'{gap:.2f}'} deg away): stopping")
                    return 1
                rec = cap = None
                if c:
                    cap = folder / f"{a.side}_cycle{c}_{k + 1}.frames.jsonl"
                    rec = CycleRecorder(cap, h.host_ip, port=UDP_PORT[a.side], log=print, cycle=c)
                    rec.start(robot)
                    r, cfg = robot.rm_get_realtime_push()          # the controller's own word for what it pushes
                    got = cfg.get("cycle") if r == 0 and isinstance(cfg, dict) else None
                    if got != c:
                        rec.stop(robot)
                        print(f"  ** the controller reads back push cycle {got}, not {c}: stopping")
                        return 1
                try:
                    secs, said = play(robot, a.slot, a.transit, timeout_s=lines * 0.006 * 2 + 30)
                finally:
                    if rec is not None:
                        rec.stop(robot)
                ms = None if secs is None else secs / max(lines - 1, 1) * 1000.0
                period = None if cap is None else frame_period_ms(cap)
                out.append((c, ms))
                print(f"  push {'off' if not c else f'{c * 5} ms'} run {k + 1}: {said}"
                      + ("" if secs is None else f", {secs:.2f} s = {ms:.3f} ms a line")
                      + ("" if cap is None else f"; feed frames every {period:.1f} ms" if period else "; no feed frame came"))
    finally:
        try:
            print(f"  overall speed back to {a.restore} % (ret={robot.rm_set_plan_speed(a.restore)})")
            robot.rm_delete_robot_arm()
        except Exception:                                                       # noqa: BLE001
            pass
    summary(out)
    print(f"captures: {folder}")
    return 0


#: The judgement (workflow.STOP_PROGRAM_OVER): S2 stops a Program whose lines take more than 105 % of 5 ms each. With the
#: push off the lab's programs play 4.5-4.9 ms a line (22 Oct), so the push may add at most about 7 % before a 4.9 ms
#: program crosses 5.25 ms; 3 % or less keeps a margin for the usual run-to-run spread. The times measured here include
#: the controller's start and end (a few tenths of a second, the same with the push on or off), so the slowdown is
#: what counts: each cycle's ms a line against the push off's.
OK_SLOWDOWN, MAX_SLOWDOWN = 0.03, 0.07


def summary(out: list) -> list[tuple]:
    """Each cycle's pace against the push off's and the verdict: [(cycle, ms a line, slowdown, verdict)]."""
    base = [m for c, m in out if c == 0 and m]
    if not base:
        print("\nno run with the push off: nothing to compare with")
        return []
    off = sum(base) / len(base)
    print(f"\npush off: {off:.3f} ms a line (the controller's start and end included)")
    rows = []
    for c in sorted({c for c, _ in out if c}):
        ms = [m for cc, m in out if cc == c and m]
        if not ms:
            continue
        pace = sum(ms) / len(ms)
        slow = pace / off - 1.0
        verdict = ("OK for S2" if slow <= OK_SLOWDOWN else "usable, little margin" if slow <= MAX_SLOWDOWN
                   else "too slow for S2")
        rows.append((c, pace, slow, verdict))
        print(f"push {c * 5} ms: {pace:.3f} ms a line, {slow * 100:+.1f} % against push off -> {verdict}")
    best = next((c for c, _, s, _ in rows if s <= OK_SLOWDOWN), None)
    print(f"S2 can record at {best * 5} ms" if best else "no cycle tried keeps S2's margin: S2 should not record its feed")
    return rows


if __name__ == "__main__":
    sys.exit(main())
