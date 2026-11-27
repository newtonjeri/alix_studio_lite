#!/usr/bin/env python3
"""The IMU on the UGV (Wheeltec N100, an FDISYSTEMS FDILink module): connect, capture, verify. No robot motion, no ROS,
nothing to install (the serial port through Linux termios; numpy for the checks, as Lite has it).

    python3 tools/imu_n100.py ports                               # the USB serial devices; which one is the IMU
    python3 tools/imu_n100.py watch                               # live values twice a second (is it alive?)
    python3 tools/imu_n100.py capture --seconds 60                # raw bytes + arrival times, then the frame checks
    python3 tools/imu_n100.py capture --seconds 60 --test still   # lying still: gravity, gyro bias, noise
    python3 tools/imu_n100.py capture --seconds 40 --test turn    # turned 90 deg by hand about one marked axis, and back
    python3 tools/imu_n100.py capture --seconds 30 --test tap     # mounted: tap the plate, then the board
    python3 tools/imu_n100.py check Resources/Logs/<day>/imu/<session> [--test still|turn|tap]   # again, on a kept one
    python3 tools/imu_n100.py config                              # read the output rates (#fmsg); changes nothing
    python3 tools/imu_n100.py config --set 40=400 --set 41=100    # set them, save, restart, read back
    python3 tools/imu_n100.py info                                # ask the unit its version and its sensor ranges

The port is found by itself when exactly one CP210x (the N100's USB chip) is plugged in; else name it with --port.
Everything is kept under Resources/Logs/<day>/imu/<HHMMSS>_<what>/: raw.bin (the bytes as received), chunks.jsonl (when
each read arrived), meta.json, report.txt / report.json, and the text of every command exchange.

Protocol (vendor: doc.fdisystems.cn/developers/02-fdilink; drivers sbgisen/fdilink_ahrs, RowSense): 921600 8N1; a frame
is 0xFC, type, length, serial number, CRC8 (over those four bytes), CRC16 high, CRC16 low (over the payload), payload,
0xFD; little-endian. 0x40 IMU (56 bytes): gyro x, y, z rad/s, accel x, y, z m/s^2 with gravity, mag x, y, z mG,
temperature C, pressure Pa, pressure temperature C, timestamp int64 us since power-on. 0x41 AHRS (48 bytes). 0xF0 a 1 Hz
heartbeat. Two CRC8 tables are in use by the drivers found (Maxim and SMBus): every header is checked against both and
the report says which the unit uses. Sources and the plan: docs/imu/PLAN.md; the lab's steps: docs/imu/RUNBOOK_step_A.md.
"""
from __future__ import annotations

import argparse
import bisect
import glob
import json
import os
import select
import struct
import sys
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

LITE_ROOT = Path(__file__).resolve().parent.parent
BAUD = 921600
HEAD, END = 0xFC, 0xFD
T_IMU, T_AHRS, T_HEART, T_REQUEST, T_VERSION, T_RANGES = 0x40, 0x41, 0xF0, 0xA0, 0x39, 0xA1
NAMES = {0x40: "IMU", 0x41: "AHRS", 0x42: "INSGPS", 0xF0: "heartbeat", 0x39: "VERSION", 0xA1: "SENSOR_RANGES",
         0x53: "STATUS"}
CP210X = ("10c4", "ea60")
G = 9.80665
RAD = 57.29577951308232
STILL_S = 2.0                 #: the first seconds of a turn or tap capture: held still, for the bias and the noise
WAIT = 1.0                    #: scales every wait on the unit's replies (the tests shrink it)
REPLY_CAP_S = 3.0             #: a reply is read at most this much longer than its wait


# ─────────────────────────────────────────────────────────────────────────── CRCs

def _table8_reflected(poly: int) -> list:
    out = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ poly if c & 1 else c >> 1
        out.append(c)
    return out


def _table8(poly: int) -> list:
    out = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        out.append(c)
    return out


def _table16(poly: int = 0x1021) -> list:
    out = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        out.append(c)
    return out


#: Maxim/Dallas (sbgisen fdilink_ahrs, the magnetometer repo: 0, 94, 188, ...) and SMBus (RowSense: 0x00, 0x07, ...)
CRC8_TABLES = {"maxim": _table8_reflected(0x8C), "smbus": _table8(0x07)}
CRC16_TABLE = _table16()


def crc8(data: bytes, table: str = "maxim") -> int:
    t, c = CRC8_TABLES[table], 0
    for b in data:
        c = t[c ^ b]
    return c


def crc16(data: bytes) -> int:
    c = 0
    for b in data:
        c = CRC16_TABLE[((c >> 8) ^ b) & 0xFF] ^ ((c << 8) & 0xFFFF)
    return c


def build_frame(ftype: int, payload: bytes, sn: int = 0, table: str = "maxim") -> bytes:
    head = bytes([HEAD, ftype, len(payload), sn & 0xFF])
    c16 = crc16(payload)
    return head + bytes([crc8(head, table), c16 >> 8, c16 & 0xFF]) + bytes(payload) + bytes([END])


# ─────────────────────────────────────────────────────────────────────────── frames

@dataclass
class Frame:
    type: int
    sn: int
    payload: bytes
    start: int                 #: byte offset of its 0xFC in the stream
    end: int                   #: byte offset just after its 0xFD
    crc8_table: str | None     #: the CRC8 table its header matched ("both": either fits, 1 in 256 by chance; None:
                               #: neither, accepted on CRC16 and 0xFD)


@dataclass
class ScanStats:
    frames: int = 0
    crc16_bad: int = 0          #: a header that matched a CRC8 table, then a payload whose CRC16 failed
    end_bad: int = 0            #: ... or whose last byte was not 0xFD
    crc8_none: int = 0          #: frames accepted on CRC16 and 0xFD whose header matched neither CRC8 table
    skipped: int = 0            #: bytes between frames (noise, a cut frame, a text reply)
    crc8_match: Counter = field(default_factory=Counter)


def scan(buf: bytes, stats: ScanStats | None = None, base: int = 0) -> tuple[list, ScanStats, int]:
    """Every frame in ``buf``. Returns (frames, stats, offset where an incomplete frame starts, or len(buf))."""
    st = stats or ScanStats()
    out, i, n = [], 0, len(buf)
    while True:
        j = buf.find(b"\xfc", i)
        if j < 0:
            st.skipped += n - i
            return out, st, n
        st.skipped += j - i
        if j + 7 > n:
            return out, st, j
        ftype, ln, sn, c8 = buf[j + 1], buf[j + 2], buf[j + 3], buf[j + 4]
        total = 7 + ln + 1
        if j + total > n:
            return out, st, j
        head = buf[j:j + 4]
        fits = [name for name in CRC8_TABLES if crc8(head, name) == c8]
        table = fits[0] if len(fits) == 1 else ("both" if fits else None)
        payload = bytes(buf[j + 7:j + 7 + ln])
        ok16 = crc16(payload) == ((buf[j + 5] << 8) | buf[j + 6])
        ok_end = buf[j + total - 1] == END
        if ok16 and ok_end:
            out.append(Frame(ftype, sn, payload, base + j, base + j + total, table))
            st.frames += 1
            if table is None:
                st.crc8_none += 1
            else:
                st.crc8_match[table] += 1
            i = j + total
            continue
        if table is not None:                 # a real header: the frame was damaged
            if not ok16:
                st.crc16_bad += 1
            else:
                st.end_bad += 1
        st.skipped += 1
        i = j + 1


def decode_imu(p: bytes) -> dict:
    v = struct.unpack_from("<12fq", p, 0)
    return dict(gyro=v[0:3], acc=v[3:6], mag=v[6:9], temp=v[9], pressure=v[10], ptemp=v[11], ts_us=v[12])


def decode_ahrs(p: bytes) -> dict:
    v = struct.unpack_from("<10fq", p, 0)
    return dict(rates=v[0:3], roll=v[3], pitch=v[4], heading=v[5], quat=v[6:10], ts_us=v[10])


def decode_version(p: bytes) -> dict:
    """MSG_VERSION (0x39, 24 + 2X bytes): serial number (4 x uint32), hardware version, hardware name (X bytes),
    firmware version, firmware name (X bytes)."""
    x = (len(p) - 24) // 2
    if x < 0:
        return dict(raw=p.hex(" "))
    sn = struct.unpack_from("<4I", p, 0)
    hw = struct.unpack_from("<I", p, 16)[0]
    fw = struct.unpack_from("<I", p, 20 + x)[0]
    text = lambda b: b.split(b"\x00")[0].decode("ascii", "replace").strip()   # noqa: E731
    return dict(serial="-".join(f"{s:08X}" for s in sn), hardware_version=hw, hardware_name=text(p[20:20 + x]),
                firmware_version=fw, firmware_name=text(p[24 + x:24 + 2 * x]), raw=p.hex(" "))


def decode_ranges(p: bytes) -> dict:
    """MSG_SENSOR_RANGES (0xA1, 6 bytes). The vendor's table gives the three fields as uint16 at offsets it does not
    write consistently; read here as three little-endian uint16 (accel g, gyro deg/s, magnetometer gauss), raw kept."""
    out = dict(raw=p.hex(" "))
    if len(p) >= 6:
        a, g, m = struct.unpack_from("<3H", p, 0)
        out.update(accel_g=a, gyro_deg_s=g, mag_gauss=m)
    return out


# ─────────────────────────────────────────────────────────────────────────── the port

def usb_serial_devices() -> list[dict]:
    by_id = {os.path.realpath(p): p for p in glob.glob("/dev/serial/by-id/*")}
    out = []
    for dev in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        info = dict(dev=dev, by_id=by_id.get(dev), access=os.access(dev, os.R_OK | os.W_OK))
        p = os.path.realpath(f"/sys/class/tty/{os.path.basename(dev)}/device")
        for _ in range(6):
            if os.path.exists(os.path.join(p, "idVendor")):
                for k in ("idVendor", "idProduct", "manufacturer", "product", "serial"):
                    try:
                        info[k] = open(os.path.join(p, k)).read().strip()
                    except OSError:
                        pass
                break
            p = os.path.dirname(p)
        info["cp210x"] = (info.get("idVendor"), info.get("idProduct")) == CP210X
        out.append(info)
    return out


NO_TTY = ("The IMU's USB chip is plugged in (lsusb: 10c4:ea60 Silicon Labs CP210x UART Bridge), but Linux gave it no "
          "serial port (/dev/ttyUSB*). `sudo dmesg | grep -i -E 'cp210x|brltty'` says why. On older Ubuntu the braille "
          "service brltty takes this chip: `sudo apt remove brltty`, then unplug and plug the IMU in again.")


def cp210x_on_usb() -> bool:
    """A CP210x on the USB bus, as lsusb lists it (10c4:ea60), whether or not it has a serial port."""
    for f in glob.glob("/sys/bus/usb/devices/*/idVendor"):
        try:
            ids = (open(f).read().strip(), open(os.path.join(os.path.dirname(f), "idProduct")).read().strip())
        except OSError:
            continue
        if ids == CP210X:
            return True
    return False


def find_port(arg: str | None) -> str:
    """The IMU's port: the one serial device whose USB chip is a CP210x (10c4:ea60), by its stable name."""
    if arg and arg != "auto":
        return arg
    devs = [d for d in usb_serial_devices() if d["cp210x"]]
    if len(devs) == 1:
        return devs[0]["by_id"] or devs[0]["dev"]
    if not devs:
        if cp210x_on_usb():
            raise SystemExit(NO_TTY)
        raise SystemExit("No CP210x USB serial device is plugged in (the N100's USB chip, 10c4:ea60 in lsusb). Check "
                         "the cable, then `python3 tools/imu_n100.py ports`.")
    raise SystemExit(f"{len(devs)} CP210x devices are plugged in; name the IMU with --port (`python3 tools/imu_n100.py "
                     "ports` lists them).")


class Port:
    """The serial port, raw 8N1 at 921600 through termios (Linux)."""

    def __init__(self, path: str, baud: int = BAUD):
        import termios
        self.path = path
        try:
            self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except PermissionError:
            raise SystemExit(f"No permission to open {path}. Add this user to the dialout group "
                             "(sudo usermod -aG dialout $USER), then log out and in again.") from None
        except FileNotFoundError:
            raise SystemExit(f"{path} is not there: the IMU is unplugged or has another name (`python3 "
                             "tools/imu_n100.py ports`).") from None
        a = termios.tcgetattr(self.fd)
        speed = getattr(termios, f"B{baud}")
        a[0] = 0                                           # iflag: no translation, no flow control
        a[1] = 0                                           # oflag
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[3] = 0                                           # lflag: raw
        a[4] = a[5] = speed
        a[6][termios.VMIN] = 0
        a[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def read(self, timeout: float = 0.05) -> bytes:
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return b""
        try:
            return os.read(self.fd, 65536)
        except BlockingIOError:
            return b""

    def write(self, data: bytes) -> None:
        view = memoryview(data)
        while view:
            try:
                n = os.write(self.fd, view)
                view = view[n:]
            except BlockingIOError:
                select.select([], [self.fd], [], 0.1)

    def close(self) -> None:
        os.close(self.fd)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# ─────────────────────────────────────────────────────────────────────────── sessions

def session_dir(out: str | None, what: str) -> Path:
    root = Path(out) if out else LITE_ROOT / "Resources" / "Logs"
    d = root / time.strftime("%Y-%m-%d") / "imu" / f"{time.strftime('%H%M%S')}_{what}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def record(port, seconds: float, folder: Path, say=print) -> dict:
    """``seconds`` of bytes as they come: raw.bin and chunks.jsonl (offset, size, time.time() and time.monotonic() of
    each read), the same clock Lite stamps its logs with."""
    folder.mkdir(parents=True, exist_ok=True)
    off, t0 = 0, time.monotonic()
    warned, next_say = False, t0 + 10.0
    with open(folder / "raw.bin", "wb") as raw, open(folder / "chunks.jsonl", "w") as ch:
        while (now := time.monotonic()) < t0 + seconds:
            b = port.read(0.05)
            if b:
                t, m = time.time(), time.monotonic()
                raw.write(b)
                ch.write(json.dumps(dict(o=off, n=len(b), t=round(t, 6), m=round(m, 6))) + "\n")
                off += len(b)
            if not warned and off == 0 and now - t0 > 3.0:
                say("No bytes from the IMU in 3 s: check the cable and the port. If a configuration was cut short, the "
                    "module may still be in configuration mode: `python3 tools/imu_n100.py config` ends it.")
                warned = True
            if now >= next_say:
                say(f"  {now - t0:5.0f} s  {off:,} bytes")
                next_say += 10.0
    return dict(bytes=off, seconds=round(time.monotonic() - t0, 3))


def load(folder: Path) -> tuple[bytes, list]:
    buf = (folder / "raw.bin").read_bytes()
    chunks = [json.loads(line) for line in (folder / "chunks.jsonl").read_text().splitlines() if line.strip()]
    return buf, chunks


# ─────────────────────────────────────────────────────────────────────────── the checks

def _pct(a: int, b: int) -> float:
    return 100.0 * a / b if b else 0.0


def analyse(buf: bytes, chunks: list, test: str = "frames") -> tuple[list, dict]:
    """The report on a capture: (lines of text, the same as a dict)."""
    import numpy as np

    frames, st, _ = scan(buf)
    lines, rep = [], dict(test=test)
    say = lines.append
    if not frames:
        say("FAIL  no frame found in the capture: wrong port or baud, or the module sent nothing.")
        rep["ok"] = False
        return lines, rep

    # framing
    tried = st.frames + st.crc16_bad + st.end_bad
    bad = st.crc16_bad + st.end_bad
    mark = "PASS" if _pct(bad, tried) < 0.1 else "FAIL"
    say(f"{mark:5} frames: {st.frames:,} good; CRC16 failed {st.crc16_bad}, end byte wrong {st.end_bad} "
        f"({_pct(bad, tried):.3f} %); {st.skipped:,} bytes between frames")
    table = unit_table(st)
    if table is None:
        say(f"CHECK CRC8: no header matched either table ({st.crc8_none:,} frames passed on CRC16 and 0xFD alone)")
    else:
        other = sum(v for k, v in st.crc8_match.items() if k not in (table, "both")) + st.crc8_none
        say(f"{'PASS' if other == 0 else 'CHECK':5} CRC8: the unit uses the {table} table "
            f"({st.crc8_match[table]:,} headers fit it alone, {st.crc8_match['both']:,} fit both by chance; "
            f"other table or neither: {other})")
    rep.update(frames=st.frames, crc16_bad=st.crc16_bad, end_bad=st.end_bad, skipped=st.skipped, crc8_table=table,
               crc8_other=sum(v for k, v in st.crc8_match.items() if k not in (table, "both")) + st.crc8_none)

    # the frame counter, all types in order (the drivers count it across types)
    lost = sum((f.sn - p.sn - 1) % 256 for p, f in zip(frames, frames[1:]))
    mark = "PASS" if _pct(lost, len(frames) + lost) < 0.1 else "FAIL"
    say(f"{mark:5} frame counter: {lost:,} lost of {len(frames) + lost:,} frames ({_pct(lost, len(frames) + lost):.3f} %)")
    rep["lost"] = lost

    # arrival time of each frame: the read that brought its last byte
    ends = [c["o"] + c["n"] for c in chunks]
    t_host = np.array([chunks[min(bisect.bisect_left(ends, f.end), len(chunks) - 1)]["t"] for f in frames]) \
        if chunks else None
    per_chunk = Counter(min(bisect.bisect_left(ends, f.end), len(chunks) - 1) for f in frames) if chunks else Counter()

    counts = Counter(f.type for f in frames)
    span_host = (t_host[-1] - t_host[0]) if t_host is not None and len(t_host) > 1 else 0.0
    say("      types: " + ", ".join(f"{NAMES.get(k, hex(k))} (0x{k:02X}) {v:,}"
                                    + (f" = {v / span_host:.1f}/s" if span_host > 0 else "")
                                    for k, v in sorted(counts.items())))
    rep["types"] = {f"0x{k:02X}": v for k, v in counts.items()}

    imu_i = [i for i, f in enumerate(frames) if f.type == T_IMU and len(f.payload) >= 56]
    if not imu_i:
        say("FAIL  no IMU frame (0x40): turn it on (`config --set 40=400`)")
        rep["ok"] = False
        return lines, rep
    imu = [decode_imu(frames[i].payload) for i in imu_i]
    ts = np.array([r["ts_us"] for r in imu], dtype=np.int64)
    gyro = np.array([r["gyro"] for r in imu], dtype=float)          # rad/s
    acc = np.array([r["acc"] for r in imu], dtype=float)            # m/s^2
    temp = np.array([r["temp"] for r in imu], dtype=float)
    steps = np.diff(ts)
    if len(steps):
        med = float(np.median(steps))
        gaps = int(np.sum(np.round(steps[steps > 1.5 * med] / med) - 1)) if med > 0 else 0
        back = int(np.sum(steps <= 0))
        rate = 1e6 / med if med > 0 else 0.0
        mark = "PASS" if gaps == 0 and back == 0 else "CHECK"
        say(f"{mark:5} IMU timestamps: step {med:.0f} us median = {rate:.1f} frames/s by its own clock; "
            f"{gaps} missing, {back} going backwards; span {(ts[-1] - ts[0]) / 1e6:.1f} s")
        rep.update(imu_rate_hz=round(rate, 2), imu_step_us=med, imu_ts_gaps=gaps, imu_ts_back=back)
        if rate < 390:
            say(f"CHECK IMU rate {rate:.0f}/s: the plan wants 400 (`config --set 40=400`)")
    if t_host is not None and len(imu_i) > 10:
        th = t_host[imu_i]
        x = (ts - ts[0]) / 1e6
        slope, icpt = np.polyfit(x, th - th[0], 1)
        res = (th - th[0]) - (slope * x + icpt)
        lat = (res - res.min()) * 1000.0
        bunch = max(per_chunk.values()) if per_chunk else 0
        say(f"      clocks: the sensor's clock runs {(1.0 / slope - 1.0) * 1e6:+.0f} ppm against the laptop's; frames "
            f"arrive {np.percentile(lat, 50):.1f} ms (median), {np.percentile(lat, 99):.1f} ms (99 %), "
            f"{lat.max():.1f} ms (worst) after the earliest; up to {bunch} frames in one read")
        rep.update(drift_ppm=round((1.0 / slope - 1.0) * 1e6, 1), arrival_p50_ms=round(float(np.percentile(lat, 50)), 2),
                   arrival_p99_ms=round(float(np.percentile(lat, 99)), 2), arrival_max_ms=round(float(lat.max()), 2),
                   frames_per_read_max=bunch)
    say(f"      temperature {temp[0]:.1f} -> {temp[-1]:.1f} C")
    rep["temp_c"] = [round(float(temp[0]), 2), round(float(temp[-1]), 2)]

    t_s = (ts - ts[0]) / 1e6
    if test == "still":
        _still(say, rep, gyro, acc, frames)
    elif test == "turn":
        _turn(say, rep, t_s, gyro)
    elif test == "tap":
        _tap(say, rep, t_s, gyro, acc)
    rep["ok"] = not any(line.startswith("FAIL") for line in lines)
    return lines, rep


def _still(say, rep, gyro, acc, frames):
    import numpy as np
    a = acc.mean(axis=0)
    mag = float(np.linalg.norm(a))
    k = int(np.argmax(np.abs(a)))
    axis = "XYZ"[k]
    mark = "PASS" if abs(mag - G) / G < 0.02 else "FAIL"
    say(f"{mark:5} gravity: |a| = {mag:.3f} m/s^2 (expected {G:.3f}, within 2 %); mean a = "
        f"({a[0]:+.3f}, {a[1]:+.3f}, {a[2]:+.3f})")
    flat = "lying flat, face up, as the vendor describes it (Z reads about -9.8)" if axis == "Z" and a[2] < 0 else \
        f"gravity on {'+' if a[k] > 0 else '-'}{axis}: not lying flat face up (the vendor: flat, Z reads about -9.8)"
    say(f"      {flat}; tilt from that axis {np.degrees(np.arccos(min(1.0, abs(a[k]) / mag))):.2f} deg")
    bias = gyro.mean(axis=0) * RAD
    noise = gyro.std(axis=0) * RAD
    anoise = acc.std(axis=0)
    move = float(np.abs((gyro - gyro.mean(axis=0)) * RAD).max())
    say(f"      gyro bias ({bias[0]:+.3f}, {bias[1]:+.3f}, {bias[2]:+.3f}) deg/s; noise RMS ({noise[0]:.3f}, "
        f"{noise[1]:.3f}, {noise[2]:.3f}) deg/s; accel noise RMS ({anoise[0]:.4f}, {anoise[1]:.4f}, {anoise[2]:.4f}) m/s^2")
    if move > 2.0:
        say(f"CHECK it moved: a gyro reading {move:.1f} deg/s away from the bias (a still capture should stay within a "
            "few tenths)")
    rep.update(acc_mean=[round(float(v), 4) for v in a], acc_norm=round(mag, 4), gravity_axis=f"{'+' if a[k] > 0 else '-'}{axis}",
               gyro_bias_deg_s=[round(float(v), 4) for v in bias], gyro_noise_deg_s=[round(float(v), 4) for v in noise],
               acc_noise=[round(float(v), 5) for v in anoise], still_max_dev_deg_s=round(move, 3))
    ah = [decode_ahrs(f.payload) for f in frames if f.type == T_AHRS and len(f.payload) >= 48]
    if ah:
        roll = np.degrees(np.mean([r["roll"] for r in ah]))
        pitch = np.degrees(np.mean([r["pitch"] for r in ah]))
        say(f"      the unit's own tilt (AHRS): roll {roll:+.2f} deg, pitch {pitch:+.2f} deg")
        rep.update(ahrs_roll_deg=round(float(roll), 3), ahrs_pitch_deg=round(float(pitch), 3))


def _turn(say, rep, t_s, gyro):
    import numpy as np
    still = t_s < STILL_S
    if still.sum() < 10:
        say(f"CHECK turn: fewer than 10 frames in the first {STILL_S:.0f} s held still; the bias is not known")
        return
    bias = gyro[still].mean(axis=0)
    w = (gyro - bias) * RAD
    dt = np.diff(t_s, prepend=t_s[0])
    ang = np.cumsum(w * dt[:, None], axis=0)
    peak = ang[np.argmax(np.abs(ang), axis=0), [0, 1, 2]]
    end = ang[-1]
    k = int(np.argmax(np.abs(peak)))
    others = [abs(peak[j]) for j in range(3) if j != k]
    ok = 85.0 <= abs(peak[k]) <= 95.0 and max(others) < 10.0
    say(f"{'PASS' if ok else 'CHECK':5} turn: the turned axis is {'XYZ'[k]}, peak {peak[k]:+.1f} deg (expected 90); the "
        f"other two peak {peak[(k + 1) % 3]:+.1f} and {peak[(k + 2) % 3]:+.1f} deg")
    say(f"      angles at the end (back where it started?): X {end[0]:+.1f}, Y {end[1]:+.1f}, Z {end[2]:+.1f} deg. "
        "A positive angle is a turn counter-clockwise seen from the tip of the marked axis (right-hand rule).")
    rep.update(turn_axis="XYZ"[k], turn_peak_deg=[round(float(v), 2) for v in peak],
               turn_end_deg=[round(float(v), 2) for v in end])


def _tap(say, rep, t_s, gyro, acc):
    import numpy as np
    still = t_s < STILL_S
    if still.sum() < 10:
        say(f"CHECK tap: fewer than 10 frames in the first {STILL_S:.0f} s held still; the noise is not known")
        return
    w = (gyro - gyro[still].mean(axis=0)) * RAD
    s = np.linalg.norm(w, axis=1)
    sigma = float(s[still].std()) or 1e-3
    a_dev = np.linalg.norm(acc - acc[still].mean(axis=0), axis=1)
    floor = float(s[still].mean())
    bar = max(floor + 10.0 * sigma, 0.5)
    quiet = floor + 3.0 * sigma
    rate = 1.0 / float(np.median(np.diff(t_s))) if len(t_s) > 1 else 0.0
    events, i, n = [], int(still.sum()), len(s)
    while i < n:
        if s[i] < bar:
            i += 1
            continue
        j_end = min(n, i + int(0.3 * rate) + 1)
        p = i + int(np.argmax(s[i:j_end]))
        k = p
        hold = max(1, int(0.05 * rate))
        while k < n and not np.all(s[k:k + hold] < quiet):
            k += 1
        settle_ms = (t_s[min(k, n - 1)] - t_s[p]) * 1000.0
        win = slice(p, min(n, p + int(0.5 * rate)))
        ax = int(np.argmax(np.abs(w[p])))
        seg = w[win, ax] - w[win, ax].mean()
        freq = None
        if len(seg) >= 16 and rate > 0:
            spec = np.abs(np.fft.rfft(seg, n=1024))     # no taper: a tap's ringing is strongest at the window's start
            f = np.fft.rfftfreq(1024, 1.0 / rate)
            spec[f < 2.0] = 0.0
            freq = float(f[int(np.argmax(spec))])
        events.append(dict(t_s=round(float(t_s[p]), 3), peak_deg_s=round(float(s[p]), 2),
                           acc_peak=round(float(a_dev[win].max()), 3), settle_ms=round(float(settle_ms), 1),
                           ring_hz=None if freq is None else round(freq, 1), axis="XYZ"[ax]))
        i = max(k, p + int(0.3 * rate))
    say(f"      tap: noise {sigma:.3f} deg/s RMS; an event is over {bar:.2f} deg/s; settled = under {quiet:.2f} deg/s "
        "for 50 ms")
    for e in events:
        say(f"      at {e['t_s']:7.2f} s: peak {e['peak_deg_s']:7.2f} deg/s ({e['axis']}), accel {e['acc_peak']:.3f} m/s^2, "
            f"settled in {e['settle_ms']:.0f} ms, rings at {e['ring_hz']} Hz")
    if not events:
        say("CHECK tap: no event over the bar: tap harder, or check the board is on the plate being tapped")
    elif max(e["settle_ms"] for e in events) > 150:
        say("CHECK tap: an event took over 150 ms to settle: a loose mount rings on; a rigid one dies in a few tens of ms")
    rep.update(tap_noise_deg_s=round(sigma, 4), tap_events=events)


def write_report(folder: Path, lines: list, rep: dict) -> None:
    (folder / "report.txt").write_text("\n".join(lines) + "\n")
    (folder / "report.json").write_text(json.dumps(rep, indent=1))


# ─────────────────────────────────────────────────────────────────────────── text commands, requests

def text_of(b: bytes) -> str:
    """The readable part of a reply: whole frames still in flight left out, other binary runs shown as their size,
    printable text kept."""
    frames, _, _ = scan(b)
    if frames:
        keep, at = [], 0
        for f in frames:
            keep.append(b[at:f.start])
            at = f.end
        b = b"".join(keep) + b[at:]
    out, run = [], 0
    for c in b:
        if 32 <= c < 127 or c in (9, 10, 13):
            if run:
                out.append(f"[{run} binary bytes]")
                run = 0
            out.append(chr(c))
        else:
            run += 1
    if run:
        out.append(f"[{run} binary bytes]")
    return "".join(out).replace("\r\n", "\n").replace("\r", "\n")


def has_text(text: str) -> bool:
    """Whether a reply carries words (#OK, a packet's name, its rate), not only bytes of frames cut at its ends."""
    import re
    return bool(re.search(r"#\w+|[A-Za-z0-9]{3,}", re.sub(r"\[\d+ binary bytes\]", " ", text)))


def text_command(port, cmd: str, wait: float = 1.0) -> bytes:
    """Send ``cmd`` + CR LF; read the reply until ``wait`` s pass, longer while it keeps coming, but never more than
    ``wait`` + REPLY_CAP_S: a unit that keeps streaming frames (a command it did not take) must not hold the tool."""
    port.write((cmd + "\r\n").encode("ascii"))
    start = time.monotonic()
    out, t_end, cap = b"", start + wait * WAIT, start + (wait + REPLY_CAP_S) * WAIT
    while True:                                        # reads at least once
        b = port.read(0.05 * WAIT)
        if b:
            out += b
            t_end = max(t_end, time.monotonic() + 0.2 * WAIT)
        if time.monotonic() >= min(t_end, cap):
            return out


def configure(port, sets: list[tuple[int, int]], log, reopen=None) -> dict:
    """Read the packets' output rates (#fmsg) and, with ``sets``, set each (#fmsg <id> <Hz>), save (#fsave), restart
    (#freboot, y) and read back. Configuration mode is always left (#fdeconfig). ``reopen()`` gives a fresh port after
    the restart."""
    def ask(p, cmd, wait=1.0):
        r = text_command(p, cmd, wait)
        log(f">>> {cmd}\n{text_of(r).strip()}\n")
        return r

    out = {}
    try:
        ask(port, "#fconfig", 1.0)
        out["before"] = text_of(ask(port, "#fmsg", 2.0))
        if sets:
            for pid, hz in sets:
                ask(port, f"#fmsg {pid:02X} {hz}", 1.0)
            ask(port, "#fsave", 2.0)
            port.write(b"#freboot\r\n")
            time.sleep(0.3 * WAIT)
            ask(port, "y", 1.0)
            log("restarting: keep the IMU still (it works out its gyro bias as it starts)\n")
            if reopen is not None:
                port.close()
                time.sleep(4.0 * WAIT)
                port = reopen()
            else:
                time.sleep(4.0 * WAIT)
            ask(port, "#fconfig", 1.0)
            out["after"] = text_of(ask(port, "#fmsg", 2.0))
    finally:
        ask(port, "#fdeconfig", 1.0)
    out["port"] = port
    return out


def unit_table(st: ScanStats) -> str | None:
    """The CRC8 table the unit uses: the one most headers fit alone (a header both fit says nothing)."""
    single = {k: v for k, v in st.crc8_match.items() if k != "both"}
    return max(single, key=single.get) if single else None


def learn_crc8(port, seconds: float = 1.5) -> str | None:
    buf, t_end = b"", time.monotonic() + seconds * WAIT
    while True:
        buf += port.read(0.05 * WAIT)
        if time.monotonic() >= t_end:
            break
    _, st, _ = scan(buf)
    return unit_table(st)


def request(port, pid: int, table: str, sn: int = 0, wait: float = 1.5) -> Frame | None:
    """Ask for packet ``pid`` once (MSG_REQUEST_PACKET 0xA0: ID, 0, 0, 0) and return its frame when it comes."""
    port.write(build_frame(T_REQUEST, bytes([pid, 0, 0, 0]), sn=sn, table=table))
    buf, t_end = b"", time.monotonic() + wait * WAIT
    while True:
        buf += port.read(0.05 * WAIT)
        frames, _, _ = scan(buf)
        for f in frames:
            if f.type == pid:
                return f
        if time.monotonic() >= t_end:
            return None


# ─────────────────────────────────────────────────────────────────────────── commands

def cmd_ports(a):
    devs = usb_serial_devices()
    if not devs:
        print(NO_TTY if cp210x_on_usb() else "No USB serial device (/dev/ttyUSB*, /dev/ttyACM*) is plugged in.")
        return 1
    for d in devs:
        tag = "  <- CP210x: the N100's chip" if d["cp210x"] else ""
        print(f"{d['dev']}  {d.get('idVendor', '?')}:{d.get('idProduct', '?')}  {d.get('manufacturer', '')} "
              f"{d.get('product', '')}  serial {d.get('serial', '-')}{tag}")
        print(f"    stable name: {d['by_id'] or '(none in /dev/serial/by-id)'}")
        if not d["access"]:
            print("    NO PERMISSION for this user: sudo usermod -aG dialout $USER, then log out and in")
    return 0


def cmd_watch(a):
    path = find_port(a.port)
    print(f"{path}: {a.seconds:.0f} s, one line every 0.5 s (Ctrl-C stops)")
    with Port(path) as p:
        buf, base, t0, nxt, last_n = b"", 0, time.monotonic(), time.monotonic() + 0.5, 0
        frames_all, st = [], ScanStats()
        try:
            while time.monotonic() < t0 + a.seconds:
                buf += p.read(0.05)
                fr, st, cut = scan(buf, st)
                frames_all += fr
                buf = buf[cut:]
                if time.monotonic() >= nxt:
                    imu = [f for f in frames_all[last_n:] if f.type == T_IMU and len(f.payload) >= 56]
                    last_n = len(frames_all)
                    if imu:
                        r = decode_imu(imu[-1].payload)
                        g = [v * RAD for v in r["gyro"]]
                        ac = r["acc"]
                        print(f"IMU {len(imu) * 2:4d}/s  gyro {g[0]:+8.2f} {g[1]:+8.2f} {g[2]:+8.2f} deg/s   accel "
                              f"{ac[0]:+7.3f} {ac[1]:+7.3f} {ac[2]:+7.3f} m/s^2 |a| {sum(v * v for v in ac) ** 0.5:6.3f}"
                              f"  {r['temp']:5.1f} C")
                    else:
                        print(f"no IMU frame in the last 0.5 s ({st.frames} frames, {st.skipped} other bytes so far)")
                    nxt += 0.5
        except KeyboardInterrupt:
            pass
        table = unit_table(st) or "not known yet"
        print(f"{st.frames} frames, CRC16 failed {st.crc16_bad}, CRC8 table: {table}")
    return 0


def cmd_capture(a):
    path = find_port(a.port)
    what = a.label or a.test
    folder = session_dir(a.out, what)
    meta = dict(port=path, baud=BAUD, test=a.test, label=a.label, seconds=a.seconds, started=time.time(),
                started_local=time.strftime("%Y-%m-%d %H:%M:%S"), tool="tools/imu_n100.py",
                note=a.note or "")
    print(f"recording {a.seconds:.0f} s from {path} into {folder}")
    if a.test in ("turn", "tap"):
        print(f"hold it still for the first {STILL_S:.0f} s, then "
              + ("turn it 90 deg about ONE marked axis, hold 2 s, and turn it back" if a.test == "turn"
                 else "tap the plate 10 cm from the board a few times, then the board itself"))
    with Port(path) as p:
        meta.update(record(p, a.seconds, folder))
    (folder / "meta.json").write_text(json.dumps(meta, indent=1))
    buf, chunks = load(folder)
    lines, rep = analyse(buf, chunks, a.test)
    write_report(folder, lines, rep)
    print("\n".join(lines))
    print(f"kept: {folder}")
    return 0 if rep.get("ok") else 1


def cmd_check(a):
    folder = Path(a.folder)
    test = a.test
    if test is None:
        try:
            test = json.loads((folder / "meta.json").read_text()).get("test") or "frames"
        except OSError:
            test = "frames"
    buf, chunks = load(folder)
    lines, rep = analyse(buf, chunks, test)
    write_report(folder, lines, rep)
    print("\n".join(lines))
    return 0 if rep.get("ok") else 1


def _parse_set(s: str) -> tuple[int, int]:
    pid, hz = s.split("=")
    return int(pid, 16), int(hz)


def cmd_config(a):
    path = find_port(a.port)
    sets = [_parse_set(s) for s in a.set or []]
    folder = session_dir(a.out, "config")
    log_path = folder / "config.txt"
    with open(log_path, "w") as log_f:
        def log(text):
            print(text, end="" if text.endswith("\n") else "\n")
            log_f.write(text if text.endswith("\n") else text + "\n")
        log(f"{time.strftime('%Y-%m-%d %H:%M:%S')}  {path}  sets: {sets or 'none (read only)'}\n")
        p = Port(path)
        out = configure(p, sets, log, reopen=lambda: Port(path))
        out["port"].close()
    if not has_text(out.get("before") or ""):
        print("No text reply to #fmsg: this firmware may not know the command. Use FDIGroundStation (Config, then MSGOUT) "
              "and note what it shows.")
    print(f"kept: {log_path}")
    return 0


def cmd_info(a):
    path = find_port(a.port)
    folder = session_dir(a.out, "info")
    out = {}
    with Port(path) as p:
        table = learn_crc8(p)
        if table is None:
            print("No frame in 1.5 s, so the CRC8 table for the request is not known: is the IMU sending? (`watch`)")
            return 1
        print(f"the unit's CRC8 table: {table}")
        for k, (pid, dec) in enumerate(((T_VERSION, decode_version), (T_RANGES, decode_ranges))):
            f = request(p, pid, table, sn=k)
            name = NAMES[pid]
            if f is None:
                print(f"{name} (0x{pid:02X}): no reply in 1.5 s. The PC tool shows the same (FDIGroundStation).")
                out[name] = None
            else:
                out[name] = dec(f.payload)
                print(f"{name} (0x{pid:02X}): " + ", ".join(f"{k2} {v}" for k2, v in out[name].items()))
    out["crc8_table"] = table
    (folder / "info.json").write_text(json.dumps(out, indent=1))
    print(f"kept: {folder / 'info.json'}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ports", help="the USB serial devices; which is the IMU")
    w = sub.add_parser("watch", help="live values twice a second")
    w.add_argument("--seconds", type=float, default=30.0)
    c = sub.add_parser("capture", help="record, keep and check")
    c.add_argument("--seconds", type=float, default=60.0)
    c.add_argument("--test", choices=["frames", "still", "turn", "tap"], default="frames")
    c.add_argument("--label", default="", help="a name for the session folder (default: the test)")
    c.add_argument("--note", default="", help="kept in meta.json (e.g. 'on the desk', 'mounted, screws')")
    k = sub.add_parser("check", help="the checks again on a kept capture")
    k.add_argument("folder")
    k.add_argument("--test", choices=["frames", "still", "turn", "tap"])
    g = sub.add_parser("config", help="read (and with --set, change) the packets' output rates")
    g.add_argument("--set", action="append", metavar="ID=HZ", help="hex packet id = rate, e.g. 40=400 (IMU), 41=100")
    sub.add_parser("info", help="ask the unit its version and sensor ranges")
    for s in (w, c, g, sub.choices["info"]):
        s.add_argument("--port", default="auto", help="the serial device (default: the one CP210x plugged in)")
    for s in (c, g, sub.choices["info"]):
        s.add_argument("--out", default=None, help="the Logs folder (default: Lite's Resources/Logs)")
    a = ap.parse_args(argv)
    return {"ports": cmd_ports, "watch": cmd_watch, "capture": cmd_capture, "check": cmd_check, "config": cmd_config,
            "info": cmd_info}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
