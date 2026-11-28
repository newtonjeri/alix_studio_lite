"""server/imu_record.py: the IMU recorded alongside an S2 run's feed, on a pseudo-terminal playing the unit."""
from __future__ import annotations

import json
import os
import struct
import threading
import time

import pytest

from alix_studio_lite.server import imu_record


@pytest.fixture
def unit(monkeypatch):
    """A pseudo-terminal sending IMU frames at about 400 a second; the tool finds it as the IMU."""
    n = imu_record.n100()
    master, slave = os.openpty()
    path = os.ttyname(slave)
    os.set_blocking(master, False)
    done = threading.Event()

    def play():
        k = 0
        while not done.is_set():
            data = b"".join(n.build_frame(0x40, struct.pack("<12fq", 0, 0, 0, 0, 0, -9.8, 0, 0, 0, 30, 101325, 30,
                                                            (k + i) * 2500), sn=(k + i) & 0xFF) for i in range(4))
            try:
                os.write(master, data)
            except BlockingIOError:
                pass
            k += 4
            time.sleep(0.01)

    th = threading.Thread(target=play, daemon=True)
    th.start()
    monkeypatch.setattr(n, "find_port", lambda arg: path)
    yield path
    done.set()
    th.join()
    os.close(master)
    os.close(slave)


def logger():
    lines = []
    return lines, lambda text, level="info": lines.append((level, text))


def test_a_run_s_recording_is_a_capture_the_tools_can_read(unit, tmp_path):
    lines, log = logger()
    rec = imu_record.start(tmp_path, "s2_both", log)
    assert rec is not None and rec.folder.parent == tmp_path / "imu" and rec.folder.name.endswith("_s2_both")
    time.sleep(0.6)
    rec.stop()
    rec.thread.join(10)
    meta = json.loads((rec.folder / "meta.json").read_text())
    assert meta["tool"] == "Lite, S2 run" and meta["bytes"] > 0
    assert (rec.folder / "report.txt").read_text().startswith("PASS  frames")
    texts = " ".join(t for _, t in lines)
    assert "Recording the IMU" in texts and "IMU recording ended" in texts
    assert imu_record._ACTIVE is None


def test_two_runs_share_one_recording_which_ends_with_the_last(unit, tmp_path):
    lines, log = logger()
    first = imu_record.start(tmp_path, "s2_left", log)
    second = imu_record.start(tmp_path, "s2_right", log)
    assert second is first and any("already being recorded" in t for _, t in lines)
    first.stop()
    assert not first.recorded.is_set()                        # the other run still records
    second.stop()
    assert first.recorded.wait(2)
    first.thread.join(10)


def test_no_imu_or_a_busy_port_is_said_and_the_run_goes_on(unit, tmp_path, monkeypatch):
    n = imu_record.n100()
    lines, log = logger()
    with n.Port(unit):                                        # a hand-made capture holds the port
        assert imu_record.start(tmp_path, "s2_both", log) is None
    assert "its port is in use by another program" in lines[-1][1]
    assert imu_record.missing() is None                       # free again: it can be recorded

    def none(arg):
        raise SystemExit("No CP210x USB serial device is plugged in (the N100's USB chip, 10c4:ea60 in lsusb).")

    monkeypatch.setattr(n, "find_port", none)
    assert imu_record.start(tmp_path, "s2_both", log) is None
    assert lines[-1] == ("warn", "The IMU is not recorded: no IMU is plugged into this computer (its USB chip, "
                                 "10c4:ea60, is not listed).")
    assert not (tmp_path / "imu").exists()


def test_a_run_asks_when_the_imu_is_not_detected(monkeypatch):
    """The user, 28 Nov: the operator is told, and asked to connect it or to run without it (code "imu")."""
    from types import SimpleNamespace

    from alix_studio_lite.server import workflow
    from alix_studio_lite.server.jobs import UserError

    app = SimpleNamespace(feed_ms=20)
    arm = SimpleNamespace(link=SimpleNamespace(emulated=False))
    robot = SimpleNamespace(_real=lambda s: arm)
    monkeypatch.setattr(imu_record, "missing", lambda: "No CP210x USB serial device is plugged in.")
    with pytest.raises(UserError) as asked:
        workflow.ask_imu(app, robot, ["left", "right"], {"record_feed": True})
    assert asked.value.code == "imu" and str(asked.value).startswith("The IMU would not be recorded with the arms' feed")
    workflow.ask_imu(app, robot, ["left", "right"], {"record_feed": True, "imu_absent_ok": True})   # run without it
    workflow.ask_imu(app, robot, ["left", "right"], {"record_feed": False})                          # no feed: no IMU
    arm.link.emulated = True
    workflow.ask_imu(app, robot, ["left", "right"], {"record_feed": True})                           # emulated arms
    arm.link.emulated = False
    monkeypatch.setattr(imu_record, "missing", lambda: None)
    workflow.ask_imu(app, robot, ["left", "right"], {"record_feed": True})                           # plugged in
