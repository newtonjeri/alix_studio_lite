"""How fast a stored program plays with the controller's realtime push off, at 10 ms and at 20 ms: the lab
measurement before S2 records the controllers' feed during its runs (the owner, 23 Oct: "We can use 10ms or 20ms").
With the push on at 5 ms, programs played some 13 % slower (21 Oct), past S2's 105 % stop.

    PYTHONPATH=src python3 tools/push_pace.py --side left --slot 59 --track <the slot's .txt> [--cycles 0,2,4]

Close Lite first (or disconnect the arm in it): this connects to the arm itself. Each run moves the arm to the
program's first point (a MoveJ at --speed %), sets the push (cycle 0 = off; else on toward this computer every
cycle x 5 ms, recorded to a file), starts the program and times it from the controller's "running" to its end:
ms per line = that time / (lines - 1). The push goes back to what it was, whatever happens (ALIX's UdpRecorder).
"""
from __future__ import annotations

import argparse
import json
import sys
import tempfile
import time
from pathlib import Path

from alix_studio_lite import vendor_paths

vendor_paths.activate()

UDP_PORT = {"left": 8089, "right": 8090}         # robot_real.UDP_PORT: each arm's capture port on this computer


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


def main(argv=None) -> int:
    from alix_dispatch import connect

    from alix_studio_lite.server.robot_real import CycleRecorder

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--side", choices=("left", "right"), required=True)
    ap.add_argument("--ip", help="the controller's address (default: ALIX's for the side)")
    ap.add_argument("--slot", type=int, required=True, help="the stored program to play")
    ap.add_argument("--track", type=Path, required=True, help="the program's .txt: its lines and its first point")
    ap.add_argument("--cycles", default="0,2,4", help="push cycles to compare, in 5 ms units (0 = off): default 0,2,4")
    ap.add_argument("--runs", type=int, default=1, help="runs per cycle")
    ap.add_argument("--speed", type=int, default=15, help="MoveJ speed %% to the first point")
    ap.add_argument("--transit", type=int, default=15, help="the program's start/transit speed %%")
    ap.add_argument("--yes", action="store_true", help="do not ask before moving the arm")
    a = ap.parse_args(argv)
    cycles = [int(c) for c in a.cycles.split(",")]
    first, lines = first_point_and_lines(a.track)
    h = connect.open_arm(a.side, ip=a.ip)
    if h is None:
        print(f"** the {a.side.upper()} controller could not be reached" + (f" at {a.ip}" if a.ip else ""))
        return 1
    robot = h.robot
    print(f"{a.side.upper()} {h.ip}: program {a.slot}, {lines} lines; push cycles {cycles} x 5 ms (0 = off)")
    if not a.yes and input(f"  the arm moves to the program's first point and plays it {len(cycles) * a.runs} times: "
                           f"type '{a.side}' to start: ").strip().lower() != a.side:
        print("  not started")
        return 1
    out = []
    folder = Path(tempfile.mkdtemp(prefix="push_pace_"))
    try:
        for c in cycles:
            for k in range(a.runs):
                ret = robot.rm_movej(first, a.speed, 0, 0, 1)
                if ret != 0:
                    print(f"  ** MoveJ to the first point refused (ret={ret}): stopping")
                    return 1
                rec = None
                if c:
                    rec = CycleRecorder(folder / f"{a.side}_cycle{c}_{k}.frames.jsonl", h.host_ip, port=UDP_PORT[a.side],
                                        log=print, cycle=c)
                    rec.start(robot)
                try:
                    secs, said = play(robot, a.slot, a.transit, timeout_s=lines * 0.006 * 2 + 30)
                finally:
                    if rec is not None:
                        rec.stop(robot)
                ms = None if secs is None else secs / max(lines - 1, 1) * 1000.0
                period = None if rec is None or not secs else secs / max(rec.frames, 1) * 1000.0
                out.append((c, k, secs, ms, period, said))
                print(f"  push {'off' if not c else f'{c * 5} ms'} run {k + 1}: {said}"
                      + ("" if secs is None else f", {secs:.2f} s = {ms:.3f} ms a line")
                      + ("" if period is None else f"; frames every {period:.1f} ms"))
    finally:
        try:
            robot.rm_delete_robot_arm()
        except Exception:                                                       # noqa: BLE001
            pass
    base = [m for c, _, _, m, _, _ in out if c == 0 and m]
    if base:
        off = sum(base) / len(base)
        print(f"\npush off: {off:.3f} ms a line")
        for c in sorted({c for c, *_ in out if c}):
            ms = [m for cc, _, _, m, _, _ in out if cc == c and m]
            if ms:
                pace = sum(ms) / len(ms)
                print(f"push {c * 5} ms: {pace:.3f} ms a line = {pace / off * 100:.1f} % of push off"
                      + ("  (within S2's 105 %)" if pace / off <= 1.05 else "  ** past S2's 105 %"))
    print(f"captures: {folder}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
