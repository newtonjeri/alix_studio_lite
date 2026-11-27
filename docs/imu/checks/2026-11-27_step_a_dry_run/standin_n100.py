#!/usr/bin/env python3
"""A stand-in N100 on a Linux pseudo-terminal, for dry runs of tools/imu_n100.py (no IMU on this laptop).

It behaves as the vendor documents (docs/imu/PLAN.md section 1): FDILink frames with the Maxim CRC8, MSG_IMU 0x40 and
MSG_AHRS 0x41 at 100 Hz until told otherwise, a 1 Hz heartbeat 0xF0; text commands #fconfig (output stops), #fmsg
(the list), #fmsg <id> <Hz>, #fsave, #freboot then y (2 s off, then the saved rates), #fdeconfig (output resumes);
the request packet 0xA0 for VERSION 0x39 and SENSOR_RANGES 0xA1. Lying flat and still: accel Z about -9.8.

    python3 standin_n100.py --seconds 400        # prints the port to give the tool (--port), then serves
What the real unit does differently is what step A is for.
"""
import argparse
import os
import select
import struct
import sys
import time
import tty
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "tools"))
import imu_n100 as n  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=400.0)
    a = ap.parse_args()
    master, slave = os.openpty()
    tty.setraw(slave)                                           # a USB serial chip echoes nothing
    os.set_blocking(master, False)                              # nobody reading: bytes are lost, as on the real port
    print(os.ttyname(slave), flush=True)
    rng = np.random.default_rng(0)
    rates, saved = {0x40: 100, 0x41: 100}, {0x40: 100, 0x41: 100}
    config, off_until, sn, inbuf = False, 0.0, 0, b""
    t0 = time.monotonic()
    nxt = {0x40: t0, 0x41: t0, 0xF0: t0}
    version = (struct.pack("<4I", 1, 0, 0, 0) + struct.pack("<I", 0x0100) + b"DETA10-SIM\x00"
               + struct.pack("<I", 0x0100) + b"STANDIN\x00\x00\x00\x00")
    ranges = struct.pack("<3H", 16, 2000, 8)

    def put(data):
        try:
            os.write(master, data)
        except BlockingIOError:
            pass

    def send(ftype, payload):
        nonlocal sn
        put(n.build_frame(ftype, payload, sn=sn))
        sn = (sn + 1) & 0xFF

    def say(text):
        put(text.encode() + b"\r\n")

    while time.monotonic() < t0 + a.seconds:
        now = time.monotonic()
        r, _, _ = select.select([master], [], [], 0.002)           # what the tool sent
        if r:
            try:
                inbuf += os.read(master, 4096)
            except (BlockingIOError, OSError):
                pass
        while True:
            if inbuf.startswith(b"\xfc") and len(inbuf) >= 8 and len(inbuf) >= 8 + inbuf[2]:
                frames, _, _ = n.scan(inbuf[:8 + inbuf[2]])
                inbuf = inbuf[8 + inbuf[2]:]
                for f in frames:
                    if f.type == n.T_REQUEST and f.payload[0] == n.T_VERSION:
                        send(n.T_VERSION, version)
                    elif f.type == n.T_REQUEST and f.payload[0] == n.T_RANGES:
                        send(n.T_RANGES, ranges)
                continue
            if b"\n" in inbuf and not inbuf.startswith(b"\xfc"):
                line, inbuf = inbuf.split(b"\n", 1)
                cmd = line.decode(errors="replace").strip()
                if cmd == "#fconfig":
                    config = True
                    say("#OK")
                elif cmd == "#fdeconfig":
                    config = False
                    say("#OK")
                elif cmd == "#fmsg":
                    for pid, name in ((0x40, "MSG_IMU"), (0x41, "MSG_AHRS")):
                        say(f"{name} [{pid:02X}] {rates[pid]}Hz")
                elif cmd.startswith("#fmsg "):
                    _, pid, hz = cmd.split()
                    rates[int(pid, 16)] = int(hz)
                    say("#OK")
                elif cmd == "#fsave":
                    saved = dict(rates)
                    say("#OK")
                elif cmd == "#freboot":
                    say("Are you sure? (y/n)")
                elif cmd == "y":
                    say("#OK rebooting")
                    rates, config, off_until = dict(saved), False, now + 2.0
                    nxt = {k: now + 2.0 for k in nxt}
                continue
            break
        if config or now < off_until:
            continue
        for pid in (0x40, 0x41, 0xF0):
            hz = rates.get(pid, 1)
            while nxt[pid] <= now:
                ts = int((nxt[pid] - t0) * 1e6)
                if pid == 0x40:
                    g = rng.normal(0, 0.0008, 3) + np.array([0.002, -0.001, 0.0005])
                    acc = np.array([0.02, -0.01, -9.79]) + rng.normal(0, 0.008, 3)
                    send(0x40, struct.pack("<12fq", *g, *acc, 300.0, 20.0, -400.0, 31.0, 101325.0, 31.0, ts))
                elif pid == 0x41:
                    send(0x41, struct.pack("<10fq", 0, 0, 0, 0.1, -0.2, 45.0, 1, 0, 0, 0, ts))
                else:
                    send(0xF0, b"\x00" * 8)
                nxt[pid] += 1.0 / hz
    return 0


if __name__ == "__main__":
    sys.exit(main())
