"""How fast a stored program plays with the controller's realtime push off, at 10 ms and at 20 ms: the lab
measurement before S2 records the controllers' feed during its runs (the owner, 23 Oct: "We can use 10ms or 20ms").
With the push on at 5 ms, programs played some 13 % slower (21 Oct), past S2's 105 % stop.

    PYTHONPATH=src python3 tools/push_pace.py --side left --slot 50 --track <the slot's .txt> [--pole 50]
        [--hand 1000 1000 1000 1000 0 1000] [--cycles 0,2,4] [--runs 2]

Close Lite first: this connects to the arm itself. Then, on one connection:
- the pole and the hand set, when asked (--pole, --hand: the control program's own manual pole and hand moves);
- each run, as S2 runs a Program, from the program's first point:
  - a lone 4105 warning cleared, as Lite's clear step does (any other code stops the measurement);
  - the arm moved to the first point when it is not there: ALIX's bridge (the global ratio at 100 % for the move,
    MoveJ at --speed %), sent NON-BLOCKING and read back from the arm's joints until it is within 0.5 degrees and
    still. A blocking MoveJ waits for the controller's arrival message, and a stale one ends it early (ALIX F89):
    at the lab on 23 Oct the move back after run 1 ended at once with "Move command result is: false";
  - the push set: cycle 0 = off; else on toward this computer every cycle x 5 ms, recorded to a file;
  - the overall speed set and the program started (non-blocking), as S2 starts it, and timed from the controller's
    "running" to its end: ms per line = that time / (lines - 1);
  - the arm left to settle (the control program's rule) and 3 s more: the controller's own end of a program comes
    0.6-3.0 s after its run state reads 0 (ALIX F114).
- The move back to the first point is a straight joint move from where the program ends. Only a program that ends
  where its file ends is moved back (2 degrees): that is the move checked against the commode for the programs below.
  Ctrl+C slow-stops the arm; the overall speed is put back to --restore however the run ends.

Lid open, seat down (the owner, 23 Oct: no one opens or closes anything during the test), programs checked with
ALIX's FK against the commode model, the lid open: L-R01 (BASE LEFT, slot 50, pole 50 mm) stays 136 mm from the
open lid and its move back starts 9 mm from the base and moves away; R-R05 (PEDESTAL UPPER RIGHT, slot 55, pole
200 mm): 241 mm from the lid, its move back starts 15 mm from the pedestal and moves away. The seat-top programs
(L-R05) and R-R04 / R-R06 pass within 0-2 mm of the commode on their way back: not for this test.

Everything is logged in the repo, Resources/Logs/<day>/push_pace/ (--log-dir): <stamp>_<SIDE>.log (what the terminal
showed, the SDK's own lines included), <stamp>_<SIDE>.events.jsonl (each call to the controller: its arguments, what
it returned, how long it took; each measurement), <stamp>_<SIDE>.summary.json and the feed captures,
<stamp>_<SIDE>_cycle<c>_run<k>.frames.jsonl. The summary gives each cycle's slowdown against the push off and the
verdict (``summary``): S2 stops a Program whose lines take more than 105 % of 5 ms each.
"""
from __future__ import annotations

import argparse
import ctypes
import io
import json
import os
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

from alix_studio_lite import vendor_paths

vendor_paths.activate()

FRESH = "LITE_PUSH_PACE_FRESH"                 # set when this tool restarted itself in a fresh process
LOG_STEM = "LITE_PUSH_PACE_LOG"                # the log's path stem, kept across that restart
AT_START_DEG = 0.5                             # at the program's first point: every joint within this
END_MATCH_DEG = 2.0                            # the program ended where its file ends (the checked move back)
SETTLE_AFTER_S = 3.0                           # ALIX F114: the program's own end, up to 3.0 s after run_state 0
TRANSIT_RATIO = 100                            # ALIX's bridge: the global ratio for the move to the first point


# ── the record ───────────────────────────────────────────────────────────────────────────────────────────────────────
def log_folder(given: Path | None) -> Path:
    return given if given is not None else vendor_paths.LITE_ROOT / "Resources" / "Logs" / time.strftime("%Y-%m-%d") / "push_pace"


def _plain(v):
    """A value as JSON can hold it (the SDK's tuples, dicts and ctypes values)."""
    if isinstance(v, (str, int, float, bool)) or v is None:
        return v
    if isinstance(v, dict):
        return {str(k): _plain(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_plain(x) for x in v]
    return repr(v)


class Record:
    """<stem>.events.jsonl: one line per call to the controller and per measurement, with the lab's clock."""

    def __init__(self, stem: Path, side: str):
        self.stem, self.side = stem, side
        stem.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(f"{stem}.events.jsonl", "a", encoding="utf-8")

    def event(self, what: str, **kv) -> None:
        row = dict(t=round(time.time(), 3), at=time.strftime("%H:%M:%S"), side=self.side, what=what)
        row.update({k: _plain(v) for k, v in kv.items()})
        self._fh.write(json.dumps(row) + "\n")
        self._fh.flush()

    def call(self, what: str, fn, *args):
        """``fn(*args)``, recorded: its arguments, what it returned or raised, how long it took."""
        t0 = time.monotonic()
        try:
            out = fn(*args)
        except Exception as exc:                                                 # noqa: BLE001
            self.event(what, args=list(args), raised=repr(exc), elapsed_s=round(time.monotonic() - t0, 3))
            raise
        self.event(what, args=list(args), ret=out, elapsed_s=round(time.monotonic() - t0, 3))
        return out

    def close(self) -> None:
        self._fh.close()


def tee_terminal(path: Path):
    """What the terminal shows, also into ``path``, each line with the lab's clock: stdout and stderr copied at the
    file-descriptor level, so the SDK's own C lines ("Move command result is: ...") are kept too (its stdout made
    line-buffered, so they stay in order). Returns the function that puts the terminal back."""
    sys.stdout.flush()
    sys.stderr.flush()
    term = os.dup(1)
    r, w = os.pipe()
    os.dup2(w, 1)
    os.dup2(w, 2)
    os.close(w)
    libc = None
    try:
        libc = ctypes.CDLL(None)
        libc.setvbuf(ctypes.c_void_p.in_dll(libc, "stdout"), None, 1, 0)        # _IOLBF
    except Exception:                                                              # noqa: BLE001
        libc = None
    saved = sys.stdout, sys.stderr                     # Python's own streams to the pipe too, whatever they were
    sys.stdout, sys.stderr = (io.TextIOWrapper(io.FileIO(os.dup(fd), "w"), encoding="utf-8", errors="replace",
                                               line_buffering=True) for fd in (1, 2))
    fh = open(path, "a", encoding="utf-8")

    def stamp(text: str) -> None:
        now = time.time()
        fh.write(f"{time.strftime('%H:%M:%S', time.localtime(now))}.{int(now * 1000) % 1000:03d} {text}\n")

    def pump():
        part = b""
        while True:
            data = os.read(r, 65536)
            if not data:
                break
            os.write(term, data)
            part += data
            *lines, part = part.split(b"\n")
            for ln in lines:
                stamp(ln.decode("utf-8", errors="replace").rstrip("\r"))
            fh.flush()
        if part:
            stamp(part.decode("utf-8", errors="replace"))
        fh.close()
        os.close(r)

    th = threading.Thread(target=pump, name="push_pace log", daemon=True)
    th.start()

    def restore():
        for stream in (sys.stdout, sys.stderr):        # closed: their copies of the pipe's end must go for its EOF
            stream.flush()
            stream.close()
        sys.stdout, sys.stderr = saved
        if libc is not None:
            libc.fflush(None)
        os.dup2(term, 1)                                # the pipe's last write ends go: the copy drains, then ends
        os.dup2(term, 2)
        th.join(timeout=5)
        os.close(term)
    return restore


# ── the arm ──────────────────────────────────────────────────────────────────────────────────────────────────────────
def first_point_and_lines(track: Path) -> tuple[list[float], int]:
    rows = [json.loads(line)["point"] for line in track.read_text(encoding="utf-8").splitlines() if line.strip()]
    return [v / 1000.0 for v in rows[0]], len(rows)


def last_point(track: Path) -> list[float]:
    rows = [line for line in track.read_text(encoding="utf-8").splitlines() if line.strip()]
    return [v / 1000.0 for v in json.loads(rows[-1])["point"]]


def gap_deg(joints, target) -> tuple[float, int]:
    """The largest joint difference (degrees) and its joint (1-based)."""
    d = [abs(float(a) - float(b)) for a, b in zip(joints, target)]
    k = max(range(len(d)), key=d.__getitem__)
    return d[k], k + 1


def joints_now(robot) -> list[float] | None:
    r, st = robot.rm_get_current_arm_state()
    return [float(v) for v in st["joint"]] if r == 0 and st and st.get("joint") else None


def move_to(robot, target: list[float], speed_pct: int, rec: Record, timeout_s: float) -> tuple[bool, str]:
    """The arm to ``target`` (degrees): ALIX's bridge MoveJ (the global ratio at 100 %, MoveJ at ``speed_pct`` of
    it), non-blocking, its arrival read from the joints: within AT_START_DEG and still for 0.3 s. Stops short (2 s
    without moving), a new error code but 4105, or the time running out ends it with the arm slow-stopped."""
    from alix_studio_lite.server.robot_real import _dac

    rec.call("rm_set_plan_speed", robot.rm_set_plan_speed, TRANSIT_RATIO)
    ret = rec.call("rm_movej (non-blocking)", robot.rm_movej, list(target), int(speed_pct), 0, 0, 0)
    if ret != 0:
        return False, f"the controller refused the MoveJ (ret={ret})"
    t0 = last_t = time.monotonic()
    last, calm, next_note = None, 0, t0
    arm = SimpleNamespace(robot=robot, side=rec.side)
    while True:
        now = time.monotonic()
        q = joints_now(robot)
        if q is not None:
            gap, j = gap_deg(q, target)
            moving = last is None or max(abs(a - b) for a, b in zip(q, last)) > 0.02
            if moving:
                last_t = now
            calm = 0 if moving else calm + 1
            last = q
            if now >= next_note:
                rec.event("move to the first point", t_s=round(now - t0, 2), gap_deg=round(gap, 2), joint=j, joints=q)
                next_note = now + 1.0
            if gap <= AT_START_DEG and calm >= 3:
                rec.event("at the first point", t_s=round(now - t0, 2), gap_deg=round(gap, 3), joints=q)
                return True, f"{now - t0:.1f} s"
            if now - last_t > 2.0 and now - t0 > 3.0:
                st = _dac().error_state(arm)
                rec.event("stopped short", gap_deg=round(gap, 2), joint=j, error_state=st)
                return False, (f"the arm stopped {gap:.1f} deg (J{j}) short of it after {now - t0:.1f} s"
                               + ("" if _dac().error_state_clean(st) else f"; it holds {_dac().describe_error_state(st)}"))
        if now - t0 > timeout_s:
            rec.call("rm_set_arm_slow_stop", robot.rm_set_arm_slow_stop)
            return False, f"not there after {timeout_s:g} s: slow-stopped"
        time.sleep(0.1)


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


def settle(robot, still_deg_s: float = 2.0, timeout_s: float = 5.0) -> float:
    """The control program's settle (runner v14 ``settle``): still for 3 readings 0.1 s apart, at most 5 s; then
    SETTLE_AFTER_S more for the controller's own end of the program (F114). Returns the seconds it took."""
    t0 = time.monotonic()
    t_end, last, calm = t0 + timeout_s, None, 0
    while time.monotonic() < t_end and calm < 3:
        q = joints_now(robot)
        if q and last:
            calm = calm + 1 if max(abs(p - r) for p, r in zip(q, last)) / 0.1 < still_deg_s else 0
        last = q
        time.sleep(0.1)
    time.sleep(SETTLE_AFTER_S)
    return time.monotonic() - t0


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
    what stays latched."""
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


# ── the measurement ──────────────────────────────────────────────────────────────────────────────────────────────────
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--side", choices=("left", "right"), required=True)
    ap.add_argument("--ip", help="the controller's address (default: ALIX's for the side, 192.168.1.10 / .103)")
    ap.add_argument("--slot", type=int, required=True, help="the stored program to play")
    ap.add_argument("--track", type=Path, required=True, help="the program's .txt: its lines, first and last point")
    ap.add_argument("--pole", type=float, help="set the pole first (mm), as the control program's --manual-pole")
    ap.add_argument("--hand", type=int, nargs=6, metavar="N", help="set the hand first (six counts 0-1000: little "
                    "ring middle index thumb-bend thumb-rotate), as the control program's --manual-hand-raw")
    ap.add_argument("--cycles", default="0,2,4", help="push cycles to compare, in 5 ms units (0 = off): default 0,2,4")
    ap.add_argument("--runs", type=int, default=1, help="runs per cycle")
    ap.add_argument("--overall", type=int, default=15, help="the overall speed %% S2 runs at (its speed slider)")
    ap.add_argument("--restore", type=int, default=50, help="the overall speed %% put back at the end")
    ap.add_argument("--speed", type=int, default=15, help="MoveJ speed %% to the first point (of a 100 %% ratio)")
    ap.add_argument("--transit", type=int, default=15, help="the program's start/transit speed %%")
    ap.add_argument("--log-dir", type=Path, help="where the record goes (default Resources/Logs/<day>/push_pace)")
    ap.add_argument("--yes", action="store_true", help="do not ask before moving the arm")
    a = ap.parse_args(argv)
    stem = Path(os.environ.get(LOG_STEM) or log_folder(a.log_dir) / f"{time.strftime('%Y%m%d_%H%M%S')}_{a.side.upper()}")
    stem.parent.mkdir(parents=True, exist_ok=True)
    os.environ[LOG_STEM] = str(stem)
    untee = tee_terminal(Path(f"{stem}.log"))
    rec = Record(stem, a.side)
    try:
        return _measure(a, stem, rec, untee)
    except Exception as exc:                                                     # noqa: BLE001 - kept in the record
        import traceback

        traceback.print_exc()
        rec.event("raised", error=repr(exc))
        return 1
    finally:
        rec.close()
        print(f"record: {stem}.log, .events.jsonl, .summary.json and the captures beside them")
        untee()


def _measure(a, stem: Path, rec: Record, untee) -> int:
    from alix_dispatch import connect

    from alix_studio_lite.server import udp_push
    from alix_studio_lite.server.robot_real import UDP_PORT, CycleRecorder, _runner

    rec.event("start", argv=sys.argv, args=vars(a), fresh=bool(os.environ.get(FRESH)))
    cycles = [int(c) for c in a.cycles.split(",")]
    first, lines = first_point_and_lines(a.track)
    last = last_point(a.track)
    if lite_running():
        print("** Lite is running (process " + ", ".join(map(str, lite_running())) + "): close it first, so nothing else "
              "commands the arm or holds its feed's port")
        return 1
    h = connect.open_arm(a.side, ip=a.ip)
    if h is None:
        print(f"** the {a.side.upper()} controller could not be reached" + (f" at {a.ip}" if a.ip else ""))
        rec.event("connect", ok=False)
        return 1
    robot = h.robot
    rec.event("connect", ok=True, ip=h.ip, host_ip=h.host_ip)
    left = udp_push.left_on(h)
    rec.event("push left on?", found=left)
    if left is not None:
        if left["holders"]:
            print(f"** the controller's push is on toward this computer's port {left['port']}, which another program holds "
                  f"({udp_push.held_text(left['holders'])}): stop it first")
            return 1
        if left["cleared"] and not os.environ.get(FRESH):
            print("  the push had been left on toward this computer: turned off; starting again in a fresh process "
                  "(this one's SDK holds the port now)")
            os.environ[FRESH] = "1"
            rec.close()
            untee()
            os.execv(sys.executable, [sys.executable] + sys.argv)
    q0 = joints_now(robot)
    gap0, j0 = gap_deg(q0, first) if q0 else (None, None)
    print(f"{a.side.upper()} {h.ip}: program {a.slot}, {lines} lines; push cycles {cycles} x 5 ms (0 = off); "
          f"overall {a.overall} %" + ("" if a.pole is None else f"; pole to {a.pole:g} mm")
          + ("" if a.hand is None else f"; hand to {a.hand}"))
    print("  the arm first moves " + ("? deg (its joints could not be read)" if gap0 is None else f"{gap0:.1f} deg (J{j0})")
          + f" to the program's first point at {a.speed} %, then plays the program {len(cycles) * a.runs} times, moving "
          "back to the first point after each run: keep the cell clear")
    if not a.yes:
        answer = input(f"  type '{a.side}' to start: ").strip().lower()
        print(f"  answer: {answer}")
        if answer != a.side:
            print("  not started")
            return 1
    arm = SimpleNamespace(robot=robot, side=a.side)
    out, rows = [], []
    try:
        if a.pole is not None or a.hand is not None:
            rt = _runner()
            if a.pole is not None:
                try:
                    rc = rec.call("manual pole (runner v14)", rt.manual_pole_control, robot, float(a.pole))
                except Exception as exc:                                         # noqa: BLE001
                    rc = f"raised {exc}"
                if rc != 0:
                    print(f"  ** the pole did not reach {a.pole:g} mm ({rc}): run this again")
                    return 1
            if a.hand is not None:
                rc = rec.call("manual hand raw (runner v14)", rt.manual_hand_raw_control, robot, list(a.hand))
                if rc != 0:
                    print(f"  ** the hand did not reach {a.hand} ({rc}): run this again")
                    return 1
        for c in cycles:
            for k in range(a.runs):
                latched = clear_lone_warning(arm)
                rec.event("errors before the run", latched=latched)
                if latched:
                    print(f"  ** the arm holds {latched}: clear it at the pendant, then run this again")
                    return 1
                q = joints_now(robot)
                if q is None:
                    print("  ** the arm's joints could not be read: stopping")
                    return 1
                gap, j = gap_deg(q, first)
                if gap > AT_START_DEG:
                    if out:                                      # a move back: only from where the program ends
                        off, jo = gap_deg(q, last)
                        if off > END_MATCH_DEG:
                            print(f"  ** the arm is {off:.1f} deg (J{jo}) from where the program ends, not where it ended "
                                  f"before: the move back was checked from there only. Move it at the pendant, then run "
                                  f"this again")
                            rec.event("not where the program ends", off_deg=off, joint=jo, joints=q)
                            return 1
                    print(f"  to the first point: {gap:.1f} deg (J{j}) at {a.speed} %")
                    ok, said = move_to(robot, first, a.speed, rec, timeout_s=20.0 + gap * 0.5)
                    if not ok:
                        print(f"  ** the arm did not reach the program's first point: {said}. Stopping")
                        return 1
                    print(f"  at the first point ({said})")
                rec_push = cap = None
                if c:
                    cap = Path(f"{stem}_cycle{c}_run{k + 1}.frames.jsonl")
                    rec_push = CycleRecorder(cap, h.host_ip, port=UDP_PORT[a.side], log=print, cycle=c)
                    rec_push.start(robot)
                    r, cfg = rec.call("rm_get_realtime_push", robot.rm_get_realtime_push)
                    got = cfg.get("cycle") if r == 0 and isinstance(cfg, dict) else None
                    if got != c:
                        rec_push.stop(robot)
                        print(f"  ** the controller reads back push cycle {got}, not {c}: stopping")
                        return 1
                try:
                    rec.call("rm_set_plan_speed", robot.rm_set_plan_speed, int(a.overall))
                    t0 = time.monotonic()
                    secs, said = play(robot, a.slot, a.transit, timeout_s=lines * 0.006 * 2 + 30)
                    rec.event("program run", slot=a.slot, transit=a.transit, said=said, seconds=secs,
                              wall_s=round(time.monotonic() - t0, 3))
                finally:
                    if rec_push is not None:
                        rec_push.stop(robot)
                settled = settle(robot)
                q = joints_now(robot)
                st = clear_state = None
                try:
                    from alix_studio_lite.server.robot_real import _dac
                    st = _dac().error_state(arm)
                    clear_state = _dac().error_state_clean(st)
                except Exception as exc:                                         # noqa: BLE001
                    st = repr(exc)
                rec.event("after the run", settle_s=round(settled, 2), joints=q, error_state=st, clean=clear_state,
                          from_end_deg=None if q is None else round(gap_deg(q, last)[0], 2))
                ms = None if secs is None else secs / max(lines - 1, 1) * 1000.0
                period = None if cap is None else frame_period_ms(cap)
                out.append((c, ms))
                rows.append(dict(cycle=c, run=k + 1, said=said, seconds=secs, ms_a_line=ms, feed_period_ms=period,
                                 capture=None if cap is None else cap.name))
                print(f"  push {'off' if not c else f'{c * 5} ms'} run {k + 1}: {said}"
                      + ("" if secs is None else f", {secs:.2f} s = {ms:.3f} ms a line")
                      + ("" if cap is None else f"; feed frames every {period:.1f} ms" if period else "; no feed frame came"))
                if secs is None:
                    print("  ** the program did not run to its end: stopping")
                    return 1
    except KeyboardInterrupt:
        print("\n  ** stopped by Ctrl+C: slow-stopping the arm")
        try:
            rec.call("rm_set_arm_slow_stop", robot.rm_set_arm_slow_stop)
        except Exception:                                                        # noqa: BLE001
            pass
        return 1
    finally:
        try:
            ret = rec.call("rm_set_plan_speed", robot.rm_set_plan_speed, int(a.restore))
            print(f"  overall speed back to {a.restore} % (ret={ret})")
            robot.rm_delete_robot_arm()
        except Exception:                                                        # noqa: BLE001
            pass
        verdicts = summary(out)
        Path(f"{stem}.summary.json").write_text(json.dumps(dict(
            side=a.side, slot=a.slot, track=str(a.track), lines=lines, overall=a.overall, transit=a.transit,
            runs=rows, verdicts=[dict(cycle=c, ms_a_line=p, slowdown=s, verdict=v) for c, p, s, v in verdicts],
            off_ms_a_line=_off(out)), indent=1), encoding="utf-8")
    return 0


#: The judgement (workflow.STOP_PROGRAM_OVER): S2 stops a Program whose lines take more than 105 % of 5 ms each. With the
#: push off the lab's programs play 4.5-4.9 ms a line (22 Oct), so the push may add at most about 7 % before a 4.9 ms
#: program crosses 5.25 ms; 3 % or less keeps a margin for the usual run-to-run spread. The times measured here include
#: the controller's start and end (a few tenths of a second, the same with the push on or off), so the slowdown is
#: what counts: each cycle's ms a line against the push off's.
OK_SLOWDOWN, MAX_SLOWDOWN = 0.03, 0.07


def _off(out: list) -> float | None:
    base = [m for c, m in out if c == 0 and m]
    return sum(base) / len(base) if base else None


def summary(out: list) -> list[tuple]:
    """Each cycle's pace against the push off's and the verdict: [(cycle, ms a line, slowdown, verdict)]."""
    off = _off(out)
    if off is None:
        print("\nno run with the push off: nothing to compare with")
        return []
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
