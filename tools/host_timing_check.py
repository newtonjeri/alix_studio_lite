#!/usr/bin/env python3
"""How late this computer wakes for a 5 ms deadline: the timing an arm stream needs from its host. No robot, no network.

Lite streams a lid or seat move to an arm one line every 5 ms on absolute deadlines (ALIX's stream_track), and stops the
stream when the host is ever more than 2 periods (10 ms) late at a line. This runs the same loop with nothing sent and
says how late the host was. Run it on the computer that runs Lite, with Lite and its page open as in a real run:

    python3 host_timing_check.py                 # 20 s as an ordinary process: what Lite's streams get today
    python3 host_timing_check.py --rt            # the same loop at real-time priority (needs the right, see its message)
    python3 host_timing_check.py --seconds 60

It prints the CPU settings it found (cores online, governor, frequency), then the result. A PASS means no line was over
one period late; a stream would have STOPPED where it says so. Nothing is changed on the computer."""
import argparse
import glob
import os
import time


def cpu_state() -> str:
    def read(p):
        try:
            return open(p).read().strip()
        except OSError:
            return None

    online = read("/sys/devices/system/cpu/online")
    govs, freqs = {}, []
    for d in sorted(glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq")):
        g = read(d + "/scaling_governor")
        if g:
            govs[g] = govs.get(g, 0) + 1
        cur, top = read(d + "/scaling_cur_freq"), read(d + "/cpuinfo_max_freq")
        if cur and top:
            freqs.append((int(cur) / 1000.0, int(top) / 1000.0))
    out = [f"cores online: {online or 'not read'} ({os.cpu_count()} in all)"]
    out.append("governor: " + (", ".join(f"{g} on {n}" for g, n in govs.items()) or "not read"))
    if freqs:
        out.append(f"frequency now: {min(f[0] for f in freqs):.0f} to {max(f[0] for f in freqs):.0f} MHz of {max(f[1] for f in freqs):.0f} MHz")
    try:
        out.append("load average: " + " ".join(f"{v:.2f}" for v in os.getloadavg()))
    except OSError:
        pass
    return "; ".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--period-ms", type=float, default=5.0)
    ap.add_argument("--rt", action="store_true", help="run the loop at real-time priority (SCHED_FIFO 50)")
    a = ap.parse_args()
    print(cpu_state())
    how = "an ordinary process"
    if a.rt:
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
            how = "real-time priority (SCHED_FIFO 50)"
        except (PermissionError, AttributeError, OSError) as exc:
            print(f"Real-time priority is not allowed for this user ({exc}): ran as an ordinary process instead.\n"
                  "To allow it for this user: a line '<user> - rtprio 60' in /etc/security/limits.d/, then log in again; "
                  "or run this once with sudo to see what it would give.")
    p = a.period_ms / 1000.0
    n = int(a.seconds / p)
    late_max = late_sum = 0.0
    over1 = over2 = 0
    first_stop = None
    t0 = time.monotonic()
    for k in range(n):
        deadline = t0 + k * p
        now = time.monotonic()
        if deadline > now:
            time.sleep(deadline - now)
            now = time.monotonic()
        late = now - deadline
        if late > 2 * p:                               # the stream's rule: stopped here; this loop re-times itself and goes on
            over2 += 1
            first_stop = k if first_stop is None else first_stop
            t0 += late
        elif late > p:
            over1 += 1
        late_max, late_sum = max(late_max, late), late_sum + min(late, 2 * p)
    print(f"{n} lines at {a.period_ms:g} ms as {how}: lateness mean {1000 * late_sum / n:.2f} ms, worst {1000 * late_max:.2f} ms; "
          f"over one period: {over1} lines; over two periods (a stream stops): {over2}")
    if over2:
        print(f"RESULT: a stream would have STOPPED, first at line {first_stop} ({first_stop * p:.1f} s in), {over2} time(s) in {a.seconds:g} s.")
        return 2
    if over1:
        print(f"RESULT: MARGINAL: no stop, but {over1} line(s) over one period late (the lab PC, 13 Nov: none in 35 streams).")
        return 1
    print("RESULT: PASS: no line over one period late.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
