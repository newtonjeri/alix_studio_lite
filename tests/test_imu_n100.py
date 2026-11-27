"""tools/imu_n100.py: the N100 IMU's frames, CRCs, the checks on a capture, and the configuration and request exchanges,
on synthetic bytes and a stand-in port (the unit itself is at the lab)."""
import importlib.util
import json
import math
import struct
import sys

import numpy as np
import pytest

from alix_studio_lite import vendor_paths

TOOL = vendor_paths.LITE_ROOT / "tools" / "imu_n100.py"


@pytest.fixture(scope="module")
def imu():
    spec = importlib.util.spec_from_file_location("imu_n100", TOOL)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod                                      # dataclasses look their module up there
    spec.loader.exec_module(mod)
    yield mod
    sys.modules.pop(spec.name, None)


def imu_payload(gyro, acc, ts_us, temp=30.0):
    return struct.pack("<12fq", *gyro, *acc, 0.0, 0.0, 0.0, temp, 101325.0, temp, ts_us)


def synth(imu, seconds=10.0, rate=400, gyro_fn=None, acc=(0.0, 0.0, -9.79), drop=(), table="maxim", seed=0,
          per_read=8, noise=0.0005):
    """A capture as the tool keeps it: IMU frames at ``rate``, AHRS at 100 Hz, a 1 Hz heartbeat, one counter across
    all, reads of ``per_read`` IMU frames stamped on the laptop's clock."""
    rng = np.random.default_rng(seed)
    buf, chunks, sn, last = bytearray(), [], 0, 0
    n = int(seconds * rate)
    for i in range(n):
        t = i / rate
        ts = int(round(t * 1e6))
        g = gyro_fn(t) if gyro_fn else (0.0, 0.0, 0.0)
        g = [v + rng.normal(0.0, noise) for v in g]
        a = [v + rng.normal(0.0, 0.01) for v in acc]
        fr = imu.build_frame(0x40, imu_payload(g, a, ts), sn, table)
        sn = (sn + 1) % 256
        if i not in drop:
            buf += fr
        if i % (rate // 100) == 0:
            buf += imu.build_frame(0x41, struct.pack("<10fq", *([0.0] * 10), ts), sn, table)
            sn = (sn + 1) % 256
        if i % rate == 0:
            buf += imu.build_frame(0xF0, b"\x01", sn, table)
            sn = (sn + 1) % 256
        if (i + 1) % per_read == 0 or i == n - 1:
            chunks.append(dict(o=last, n=len(buf) - last, t=1.8e9 + t + 0.003, m=100.0 + t + 0.003))
            last = len(buf)
    return bytes(buf), chunks


# ─────────────────────────────────────────────────────────────── CRCs and frames

def test_crc_tables_are_the_drivers_and_give_the_standard_check_values(imu):
    assert imu.CRC8_TABLES["maxim"][:4] == [0, 94, 188, 226]          # sbgisen fdilink_ahrs crc_table.cpp
    assert imu.CRC8_TABLES["smbus"][:4] == [0x00, 0x07, 0x0E, 0x09]   # RowSense imu_node.py
    assert imu.CRC16_TABLE[:4] == [0x0000, 0x1021, 0x2042, 0x3063]    # sbgisen
    assert imu.crc8(b"123456789", "maxim") == 0xA1                    # CRC-8/MAXIM check value
    assert imu.crc8(b"123456789", "smbus") == 0xF4                    # CRC-8/SMBUS
    assert imu.crc16(b"123456789") == 0x31C3                          # CRC-16/XMODEM


def test_frames_round_trip_and_the_scan_resyncs_past_noise_and_a_damaged_frame(imu):
    good = [imu.build_frame(0x40, imu_payload((0.1, -0.2, 0.3), (1.0, 2.0, -9.8), 1000 * k), k) for k in range(5)]
    bad = bytearray(good[2])
    bad[20] ^= 0xFF                                                   # a payload byte flipped
    stream = b"\x00\xfc\x13" + good[0] + good[1] + bytes(bad) + b"junk" + good[3] + good[4][:30]
    frames, st, cut = imu.scan(stream)
    assert [f.sn for f in frames] == [0, 1, 3]
    assert st.crc16_bad == 1 and st.frames == 3
    assert cut == len(stream) - 30                                    # the cut frame waits for more bytes
    r = imu.decode_imu(frames[0].payload)
    assert r["gyro"] == pytest.approx((0.1, -0.2, 0.3), abs=1e-6) and r["ts_us"] == 0
    assert r["acc"][2] == pytest.approx(-9.8, abs=1e-6)


def test_the_unit_s_crc8_table_is_found_from_its_frames(imu):
    buf, chunks = synth(imu, seconds=1.0, table="smbus")
    lines, rep = imu.analyse(buf, chunks)
    assert rep["crc8_table"] == "smbus" and rep["crc8_other"] == 0
    assert any("smbus table" in line for line in lines)


# ─────────────────────────────────────────────────────────────── the checks

def test_still_capture_rates_lost_frames_gravity_and_bias(imu):
    buf, chunks = synth(imu, seconds=10.0, drop=(100, 2000, 2001), gyro_fn=lambda t: (0.001, 0.0, -0.002))
    lines, rep = imu.analyse(buf, chunks, "still")
    assert rep["lost"] == 3 and rep["imu_ts_gaps"] == 3
    assert rep["imu_rate_hz"] == pytest.approx(400.0, abs=0.5)
    assert rep["types"] == {"0x40": 3997, "0x41": 1000, "0xF0": 10}
    assert rep["gravity_axis"] == "-Z" and rep["acc_norm"] == pytest.approx(9.79, abs=0.01)
    assert rep["gyro_bias_deg_s"][0] == pytest.approx(math.degrees(0.001), abs=0.01)
    assert rep["drift_ppm"] == pytest.approx(0.0, abs=50)
    assert rep["ok"], lines


def test_too_many_lost_frames_fail(imu):
    buf, chunks = synth(imu, seconds=2.0, drop=tuple(range(100, 110)))
    lines, rep = imu.analyse(buf, chunks)
    assert not rep["ok"] and any(line.startswith("FAIL  frame counter") for line in lines)


def test_turn_of_90_deg_about_x_and_back(imu):
    w = math.radians(90.0)

    def gyro(t):
        if 2.0 <= t < 3.0:
            return (w, 0.0, 0.0)
        if 4.0 <= t < 5.0:
            return (-w, 0.0, 0.0)
        return (0.0, 0.0, 0.0)

    lines, rep = imu.analyse(*synth(imu, seconds=6.0, gyro_fn=gyro), "turn")
    assert rep["turn_axis"] == "X"
    assert rep["turn_peak_deg"][0] == pytest.approx(90.0, abs=1.0)
    assert abs(rep["turn_end_deg"][0]) < 1.0
    assert any(line.startswith("PASS  turn") for line in lines)


@pytest.mark.parametrize("decay_s, loose", [(0.02, False), (0.4, True)])
def test_tap_finds_the_event_its_ringing_and_a_loose_mount(imu, decay_s, loose):
    def gyro(t):
        if t < 3.0:
            return (0.0, 0.0, 0.0)
        v = math.radians(20.0) * math.exp(-(t - 3.0) / decay_s) * math.sin(2 * math.pi * 25.0 * (t - 3.0) + 1.0)
        return (0.0, v, 0.0)

    lines, rep = imu.analyse(*synth(imu, seconds=5.0, gyro_fn=gyro), "tap")
    ev = rep["tap_events"]
    assert len(ev) >= 1 and ev[0]["t_s"] == pytest.approx(3.0, abs=0.05) and ev[0]["axis"] == "Y"
    assert ev[0]["ring_hz"] == pytest.approx(25.0, abs=2.0)
    assert (ev[0]["settle_ms"] > 150) is loose
    assert any("over 150 ms" in line for line in lines) is loose


def test_check_command_on_a_kept_capture(imu, tmp_path):
    buf, chunks = synth(imu, seconds=3.0)
    (tmp_path / "raw.bin").write_bytes(buf)
    (tmp_path / "chunks.jsonl").write_text("".join(json.dumps(c) + "\n" for c in chunks))
    (tmp_path / "meta.json").write_text(json.dumps(dict(test="still")))
    assert imu.main(["check", str(tmp_path)]) == 0
    rep = json.loads((tmp_path / "report.json").read_text())
    assert rep["test"] == "still" and rep["gravity_axis"] == "-Z"
    assert (tmp_path / "report.txt").read_text().startswith("PASS  frames")


# ─────────────────────────────────────────────────────────────── the port, commands and requests

def test_port_is_found_only_when_one_cp210x_is_plugged_in(imu, monkeypatch):
    one = dict(dev="/dev/ttyUSB0", by_id="/dev/serial/by-id/usb-Silicon_Labs_CP2102-if00-port0", cp210x=True)
    other = dict(dev="/dev/ttyACM0", by_id=None, cp210x=False)
    monkeypatch.setattr(imu, "usb_serial_devices", lambda: [one, other])
    assert imu.find_port("auto") == one["by_id"]
    assert imu.find_port("/dev/ttyUSB3") == "/dev/ttyUSB3"
    monkeypatch.setattr(imu, "usb_serial_devices", lambda: [other])
    monkeypatch.setattr(imu, "cp210x_on_usb", lambda: False)
    with pytest.raises(SystemExit, match="No CP210x"):
        imu.find_port("auto")
    monkeypatch.setattr(imu, "cp210x_on_usb", lambda: True)          # on the USB bus, but no /dev/ttyUSB*
    with pytest.raises(SystemExit, match="no serial port"):
        imu.find_port("auto")
    monkeypatch.setattr(imu, "usb_serial_devices", lambda: [one, dict(one, dev="/dev/ttyUSB1")])
    with pytest.raises(SystemExit, match="2 CP210x"):
        imu.find_port("auto")


class StandInUnit:
    """A port that answers like the unit: text replies to # commands, frames on a request."""

    def __init__(self, imu, table="maxim", replies=None, version=b"", stream=b""):
        self.imu, self.table, self.sent, self.queue = imu, table, [], bytearray(stream)
        self.replies = replies or {}
        self.version = version

    def write(self, data):
        self.sent.append(bytes(data))
        if data.startswith(b"#") or data in (b"y\r\n",):
            self.queue += self.replies.get(data.decode().strip(), b"#OK\r\n")
            return
        frames, _, _ = self.imu.scan(bytes(data))
        for f in frames:
            if f.type == 0xA0 and f.payload[0] == 0x39:
                self.queue += self.imu.build_frame(0x39, self.version, 7, self.table)

    def read(self, timeout=0.05):
        out = bytes(self.queue)
        self.queue.clear()
        return out

    def close(self):
        pass


def test_config_reads_sets_saves_restarts_and_reads_back(imu, monkeypatch):
    monkeypatch.setattr(imu, "WAIT", 0.002)
    unit = StandInUnit(imu, replies={"#fmsg": b"IMU   [40]  100Hz\r\nAHRS  [41]  100Hz\r\n"})
    log = []
    out = imu.configure(unit, [(0x40, 400), (0x41, 100)], log.append, reopen=lambda: unit)
    sent = [s.decode().strip() for s in unit.sent]
    assert sent == ["#fconfig", "#fmsg", "#fmsg 40 400", "#fmsg 41 100", "#fsave", "#freboot", "y", "#fconfig", "#fmsg",
                    "#fdeconfig"]
    assert "IMU   [40]  100Hz" in out["before"] and "after" in out
    assert any(">>> #fmsg 40 400" in line for line in log)


def test_config_read_only_changes_nothing_and_always_leaves_configuration_mode(imu, monkeypatch):
    monkeypatch.setattr(imu, "WAIT", 0.002)
    unit = StandInUnit(imu)
    imu.configure(unit, [], lambda _t: None)
    assert [s.decode().strip() for s in unit.sent] == ["#fconfig", "#fmsg", "#fdeconfig"]


def test_request_uses_the_unit_s_crc8_and_decodes_its_version(imu, monkeypatch):
    monkeypatch.setattr(imu, "WAIT", 0.002)
    name = b"DETA10-A\x00\x00\x00"                                     # 11 bytes, as the vendor says for DETA10
    version = struct.pack("<4I", 1, 2, 3, 4) + struct.pack("<I", 0x0102) + name + struct.pack("<I", 0x0305) + \
        b"FW-N100\x00\x00\x00\x00"
    stream = b"".join(imu.build_frame(0x40, imu_payload((0, 0, 0), (0, 0, -9.8), k), k, "smbus") for k in range(5))
    unit = StandInUnit(imu, table="smbus", version=version, stream=stream)
    table = imu.learn_crc8(unit, seconds=0.05)
    assert table == "smbus"
    f = imu.request(unit, 0x39, table)
    sent = imu.scan(unit.sent[-1])[0]
    assert sent and sent[0].type == 0xA0 and sent[0].crc8_table == "smbus" and sent[0].payload == b"\x39\x00\x00\x00"
    v = imu.decode_version(f.payload)
    assert v["hardware_name"] == "DETA10-A" and v["firmware_name"] == "FW-N100" and v["hardware_version"] == 0x0102


def test_capture_through_a_real_serial_device(imu, tmp_path):
    """The Port (termios, 921600, raw) and the recording loop on a pseudo-terminal, a thread playing the unit."""
    import os
    import threading

    buf, _ = synth(imu, seconds=1.0)
    master, slave = os.openpty()
    path = os.ttyname(slave)

    def unit():
        for k in range(0, len(buf), 640):                     # ten IMU frames' worth every 25 ms
            os.write(master, buf[k:k + 640])
            threading.Event().wait(0.025)

    with imu.Port(path) as p:
        th = threading.Thread(target=unit)
        th.start()
        got = imu.record(p, 1.6, tmp_path, say=lambda _t: None)
        th.join()
    os.close(master)
    os.close(slave)
    raw, chunks = imu.load(tmp_path)
    assert got["bytes"] == len(buf) and raw == buf
    assert chunks[0]["o"] == 0 and sum(c["n"] for c in chunks) == len(buf)
    _, rep = imu.analyse(raw, chunks)
    assert rep["lost"] == 0 and rep["types"]["0x40"] == 400


def test_sensor_ranges_and_text_replies(imu):
    assert imu.decode_ranges(struct.pack("<3H", 16, 2000, 8)) == dict(raw="10 00 d0 07 08 00", accel_g=16,
                                                                     gyro_deg_s=2000, mag_gauss=8)
    t = imu.text_of(b"\xfc\x40\x38" + b"#OK\r\nIMU [40] 400Hz\r\n")
    assert t.startswith("[1 binary bytes]") and "IMU [40] 400Hz" in t
    assert imu.has_text(t) and not imu.has_text("[12 binary bytes]\n")
