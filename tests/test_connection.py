"""Each arm's connection details and each device judged on its own (the user, 6 Nov).

The lab, 6 Nov (a new robot): the LEFT hand never answered its Modbus. Each failed read held the readings' connection
about a second, so every reading came 1.2 to 3 s late, the E-stop read UNKNOWN, and Real and Recover were refused
naming the E-stop, never the hand. Now the hand, the pole, the E-stop input and the controller are each checked and
named; a hand or a pole that does not answer blocks every motion (the user: "no execution should be done because it
will affect the motion quality"); and the details a new setup needs (address, port, feed port, E-stop input) are
entered once, on the Connection page, and kept."""
import json
import threading
import time
from types import SimpleNamespace

import pytest

from alix_studio_lite import vendor_paths

vendor_paths.activate()


# ── the details, checked and kept

def test_the_details_are_checked_kept_and_each_arm_has_its_own(tmp_path):
    from alix_studio_lite.server.connection import DEFAULTS, ConnectionSettings
    from alix_studio_lite.server.jobs import UserError

    c = ConnectionSettings(tmp_path)
    assert c.arm("left") == DEFAULTS["left"] and c.arm("right")["ip"] == "192.168.1.103"       # today's until saved
    with pytest.raises(UserError, match="'192.168.1.1099' is not an IPv4 address"):          # the lab's typo, 6 Nov
        c.update("left", {"ip": "192.168.1.1099"})
    with pytest.raises(UserError, match="port must be from 1 to 65535"):
        c.update("left", {"port": 70000})
    with pytest.raises(UserError, match="E-stop input must be from 1 to 4"):
        c.update("left", {"estop_io": 5})
    with pytest.raises(UserError, match="each arm has a controller of its own"):
        c.update("left", {"ip": "192.168.1.103"})
    with pytest.raises(UserError, match="a port of its own"):
        c.update("right", {"feed_port": 8089})
    c.update("left", {"ip": " 192.168.1.20 ", "feed_port": "9001", "estop_io": 2, "estop_level": 1})
    again = ConnectionSettings(tmp_path)                                                    # kept for the next start
    assert again.arm("left") == dict(ip="192.168.1.20", port=8080, feed_port=9001, estop_io=2, estop_level=1, feed_follow_host=True)
    c.update("right", {"feed_follow_host": "false"})                                     # the feed follows this computer: default on
    assert ConnectionSettings(tmp_path).arm("right")["feed_follow_host"] is False
    assert again.view()["hand"] == dict(port=1, port_name="the end's RS485", device=1, baud=115200)   # shown, fixed by the runner
    (tmp_path / "connection.json").write_text(json.dumps({"left": {"ip": "nonsense"}}))
    assert ConnectionSettings(tmp_path).arm("left") == DEFAULTS["left"]                      # a bad file: that arm's defaults


# ── the link: each device on its own

def _link(monkeypatch, robot, **kw):
    from alix_dispatch import peripherals, runmode

    from alix_studio_lite.server import robot_link
    from alix_studio_lite.server.robot_link import ArmLink

    monkeypatch.setattr(robot_link, "_joint_floats", lambda *a, **k: None)
    monkeypatch.setattr(robot_link, "_controller_state", lambda robot: None)
    monkeypatch.setattr(runmode, "read_run_mode", lambda robot: (0, 1))
    link = ArmLink("left", "192.168.1.10", **kw)
    link.handle = link.reader = SimpleNamespace(robot=robot, host_ip="192.168.1.50")
    link.emulated = False
    return link


class Robot:
    """A controller: the arm, IO3 (mode 5, released), the pole, and a hand that answers or not (``hand``)."""

    def __init__(self, hand=True, hand_s=0.0, pole=True, io_mode=5):
        self.hand, self.hand_s, self.pole, self.io_mode = hand, hand_s, pole, io_mode
        self.hand_reads, self.sets, self.io_asked = 0, 0, []
        self.robot_controller_version = 3

    def rm_get_current_arm_state(self):
        return 0, {"joint": [0.0] * 7, "pose": [0.0] * 6, "err": {"err": ["0"]}}

    def rm_get_io_state(self, io):
        self.io_asked.append(io)
        return 0, {"io_state": 1, "io_config": {"io_mode": self.io_mode}}

    def rm_get_lift_state(self):
        return (0, {"pos": 210, "current": 0, "err_flag": 0, "mode": 0}) if self.pole else (-2, {})

    def rm_set_modbus_mode(self, port, baud, timeout):
        self.sets += 1
        return 0

    def rm_read_multiple_holding_registers(self, params):
        self.hand_reads += 1
        time.sleep(self.hand_s)
        if not self.hand:
            return 1, []
        out = []
        for v in (10, 20, 30, 40, 50, 60):
            out += [(v >> 8) & 0xFF, v & 0xFF]
        return 0, out

    def rm_get_tool_voltage(self):
        return 0, 3

    def rm_get_tool_rs485_mode(self):
        return {"return_code": 0, "mode": 1, "baudrate": 115200, "timeout": 2}

    def rm_get_realtime_push(self):
        return 0, {"cycle": 1, "enable": False, "port": 8089, "ip": "192.168.1.50", "force_coordinate": 0}

    def rm_get_wired_net(self):
        return {"return_code": 0, "ip": "192.168.1.10", "mask": "255.255.255.0", "mac": "00:00:37:9c:85:a2"}

    def rm_get_wifi_net(self):
        return 0, {"channel": 8, "ip": "192.168.33.80", "mac": "20:00:37:9c:85:a2", "mask": "255.255.255.0", "mode": "ap",
                   "password": "secret", "ssid": "RM75-6F-3668"}

    def rm_get_robot_info(self):
        return 0, {"arm_dof": 7, "arm_model": "RM_75", "force_type": 0, "robot_controller_version": 3}


def test_a_silent_hand_is_named_tried_now_and_then_and_never_holds_the_estop_back(monkeypatch):
    """The 6 Nov lab: the hand is judged on its own after two reads that fail right after the end port was put back in
    Modbus mode, read again every HAND_RETRY_S only, and its slow read follows the published reading, so the E-stop
    stays fresh (CLEAR, not UNKNOWN) through it."""
    from alix_studio_lite.server import robot_link

    monkeypatch.setattr(robot_link, "HAND_REOPEN_S", 0.0)
    monkeypatch.setattr(robot_link, "HAND_RETRY_S", 0.6)
    robot = Robot(hand=False)
    link = _link(monkeypatch, robot)
    link._read_all(robot)
    assert link.devices()["hand"]["ok"] is None                     # one failed read after the reopen: not judged yet
    link._read_all(robot)
    hand = link.devices()["hand"]
    assert hand["ok"] is False and "no reply on the end's RS485 (Modbus ID 1, 115200 baud)" in hand["why"]
    reads = robot.hand_reads
    for _ in range(3):
        link._read_all(robot)
    assert robot.hand_reads == reads                                # not in every reading now
    assert link.view()["feedback"]["hand"] is None
    time.sleep(0.65)
    robot.hand_s = 1.3                                              # a read that takes longer than STALE_S
    t = threading.Thread(target=link._read_all, args=(robot,))
    t.start()
    time.sleep(1.15)
    fb = link.view()["feedback"]
    assert robot.hand_reads > reads and not fb["stale"] and fb["estop"] == "CLEAR"   # the E-stop read before it, kept fresh
    t.join()
    fb = link.view()["feedback"]
    assert not fb["stale"] and fb["age_s"] < 0.2                    # the rest read again at once after the slow hand read
    robot.hand, robot.hand_s = True, 0.0                            # the cable plugged in: it answers at its next try
    time.sleep(0.65)
    link._read_all(robot)
    assert link.devices()["hand"]["ok"] is True and link.view()["feedback"]["hand"] == [10, 20, 30, 40, 50, 60]
    assert set(robot.io_asked) == {3}


def test_a_controller_that_does_not_answer_is_not_its_hands_or_poles_silence(monkeypatch):
    from alix_studio_lite.server import robot_link

    monkeypatch.setattr(robot_link, "HAND_REOPEN_S", 0.0)
    robot = Robot(hand=False, pole=False)
    robot.rm_get_current_arm_state = lambda: (-2, None)
    link = _link(monkeypatch, robot)
    for _ in range(4):
        link._read_all(robot)
    assert link.devices()["hand"]["ok"] is None and link.devices()["pole"]["ok"] is None


def test_the_pole_is_silent_after_three_failed_reads_over_a_moment(monkeypatch):
    """The lab's logs, 24 Oct to 6 Nov: never more than one pole read missed in a row."""
    from alix_studio_lite.server import robot_link

    monkeypatch.setattr(robot_link, "POLE_SILENT_S", 0.2)
    robot = Robot(pole=False)
    link = _link(monkeypatch, robot)
    link._read_all(robot)
    link._read_all(robot)
    link._read_all(robot)
    assert link.devices()["pole"]["ok"] is None                     # three, but within POLE_SILENT_S
    time.sleep(0.25)
    link._read_all(robot)
    assert link.devices()["pole"]["ok"] is False
    robot.pole = True
    link._read_all(robot)
    assert link.devices()["pole"]["ok"] is True


def test_the_estop_input_and_its_level_are_the_connections(monkeypatch):
    robot = Robot()
    link = _link(monkeypatch, robot, estop_io=2, estop_active=1)
    fb = link._read_all(robot)
    assert robot.io_asked == [2] and fb["estop"] == "ENGAGED" and fb["io3"]["io"] == 2   # level 1 is pressed on this wiring
    assert link.view()["estop_io"] == 2


def test_test_connection_names_each_device_and_why_the_hand_does_not_answer(monkeypatch):
    robot = Robot()
    link = _link(monkeypatch, robot, feed_port=8089)
    r = link.check_devices()
    rows = {x["device"]: x for x in r["rows"]}
    assert r["ok"] and set(rows) == {"controller", "estop", "pole", "hand", "feed"}
    assert "answers at 192.168.1.10:8080" in rows["controller"]["text"] and "RM_75" in rows["controller"]["text"]
    assert rows["estop"]["text"].startswith("IO3 in the E-stop mode (5)") and "released" in rows["estop"]["text"]
    assert rows["pole"]["text"] == "answers: 210 mm"
    assert rows["hand"]["text"].endswith("10, 20, 30, 40, 50, 60") and "UDP port 8089" in rows["feed"]["text"]
    assert r["read"]["wired"] == {"ip": "192.168.1.10", "mask": "255.255.255.0", "mac": "00:00:37:9c:85:a2"}
    assert r["read"]["wifi"]["ssid"] == "RM75-6F-3668" and "password" not in r["read"]["wifi"]   # as the vendor's app shows
    assert r["read"]["tool_v"] == 24 and r["read"]["tool_rs485"]["baudrate"] == 115200
    # the hand silent: the check says why, and judges it at once
    robot.hand = False
    r = link.check_devices()
    hand = next(x for x in r["rows"] if x["device"] == "hand")
    assert not r["ok"] and not hand["ok"]
    assert hand["text"] == ("no reply on the end's RS485 (Modbus ID 1, 115200 baud), though the end's power is 24 V and its RS485 "
                            "is in Modbus mode at 115200 baud")
    assert "the hand's own Modbus ID is 1" in hand["hint"] and link.devices()["hand"]["ok"] is False
    robot.rm_get_tool_voltage = lambda: (0, 0)
    hand = next(x for x in link.check_devices()["rows"] if x["device"] == "hand")
    assert "the end's power is off (0 V)" in hand["text"] and "End Control" in hand["hint"]
    robot.rm_get_tool_voltage = lambda: (0, 3)
    robot.rm_get_tool_rs485_mode = lambda: {"return_code": 0, "mode": 1, "baudrate": 9600, "timeout": 2}
    hand = next(x for x in link.check_devices()["rows"] if x["device"] == "hand")
    assert "runs at 9600 baud" in hand["text"]
    # the E-stop input in another mode, the pole silent
    robot.io_mode, robot.pole = 4, False
    rows = {x["device"]: x for x in link.check_devices()["rows"]}
    assert not rows["estop"]["ok"] and "IO3 is in mode 4" in rows["estop"]["text"]
    assert not rows["pole"]["ok"] and link.devices()["pole"]["ok"] is False


def test_a_4th_generation_controller_is_asked_its_own_rs485_call(monkeypatch):
    """robot_real._fourth_gen: a call of the other generation answers -4 and keeps the SDK's lock (every later call hangs)."""
    robot = Robot()
    robot.robot_controller_version = 4
    robot.rm_get_tool_rs485_mode = lambda: pytest.fail("the 3rd-generation call on a 4th-generation controller")
    robot.rm_get_tool_rs485_mode_v4 = lambda: (0, {"mode": 1, "baudrate": 115200})
    r = _link(monkeypatch, robot).check_devices()
    assert r["read"]["tool_rs485"] == {"mode": 1, "baudrate": 115200}


# ── the robot: a device that does not answer blocks every motion, named

class FakeLink:
    emulated = False

    def __init__(self, devices, estop="CLEAR"):
        self.devices, self.estop = devices, estop

    def view(self):
        return dict(feedback=dict(estop=self.estop, age_s=0.1, stale=False, busy=None), devices=self.devices,
                    quality=dict(level="good", text="ok"))


def test_a_silent_hand_blocks_motion_on_both_arms_named_and_said_once(tmp_path):
    from alix_studio_lite.server.jobs import Jobs, UserError
    from alix_studio_lite.server.robot_standin import StandInRobot

    alerts = []
    log = SimpleNamespace(alert=lambda level, text, **k: alerts.append((level, text, k.get("hint"))), hardware=lambda *a, **k: None)
    r = StandInRobot(tmp_path, Jobs(), log=log)
    why = "no reply on the end's RS485 (Modbus ID 1, 115200 baud)"
    for s in ("left", "right"):
        r.arms[s].connected = True
    r.links["left"] = FakeLink({"hand": dict(ok=False, why=why), "pole": dict(ok=True)})
    r.links["right"] = FakeLink({"hand": dict(ok=True), "pole": dict(ok=True)})
    block = r.motion_block()
    assert block == f"The LEFT hand does not answer: {why}. Robot motion is blocked on both arms until it answers."
    assert r.motion_block_kind() == "device"                                           # the top bar: BLOCKED, not E-STOP
    with pytest.raises(UserError) as e:
        r.require_motion("right")
    assert "LEFT hand does not answer" in str(e.value) and "Test connection" in e.value.hint
    r.view()
    said = [a for a in alerts if "LEFT hand does not answer" in a[1]]
    assert len(said) == 1 and said[0][0] == "error"
    # an E-stop that cannot be read is named after the device that needs looking at; a pressed one before it
    r.links["left"].estop = "UNKNOWN"
    assert "LEFT hand does not answer" in r.motion_block()
    r.links["right"].estop = "ENGAGED"
    assert "RIGHT arm's E-stop is e-stop pressed" in r.motion_block() and r.motion_block_kind() == "estop"
    r.links["right"].estop = "CLEAR"
    r.arms["right"].estop_latched = False
    r.recover("right")
    with pytest.raises(UserError) as e:
        r.recover("left")
    assert "its hand does not answer" in str(e.value)
    r.links["left"] = FakeLink({"hand": dict(ok=True), "pole": dict(ok=True)})       # it answers again
    assert r.motion_block() is None


def test_saved_details_apply_at_the_next_connect(tmp_path):
    from alix_studio_lite.server.jobs import Jobs
    from alix_studio_lite.server.robot_standin import StandInRobot

    r = StandInRobot(tmp_path, Jobs())
    out = r.set_connection("left", {"ip": "192.168.1.20", "estop_io": 2})
    assert out == dict(arm="left", saved=dict(ip="192.168.1.20", port=8080, feed_port=8089, estop_io=2, estop_level=0,
                                              feed_follow_host=True), pending=False)
    assert r.arms["left"].ip == "192.168.1.20" and r.arms["left"].view()["estop_io"] == "IO2"
    assert StandInRobot(tmp_path, Jobs()).arms["left"].ip == "192.168.1.20"                 # kept for the next start
    r.arms["left"].connected = True
    r.links["left"] = FakeLink({})
    r._opened_with["left"] = dict(r.conn.arm("left"))
    assert r.set_connection("left", {"feed_port": 9100})["pending"] is True                 # linked: at its next Connect


# ── the feed push, turned off (the user, 6 Nov: the new controllers' pushes on toward other computers)

def test_turn_the_feed_push_off_keeps_its_target_reads_it_back_and_refuses_when_it_stays_on(monkeypatch):
    from alix_studio_lite.server.jobs import UserError

    robot = Robot()
    robot.push = {"cycle": 1, "enable": True, "port": 8089, "ip": "192.168.120.11", "force_coordinate": -1}
    written = []

    def set_push(cfg):
        written.append(dict(enable=cfg.enable, ip=cfg.ip.decode() if isinstance(cfg.ip, bytes) else cfg.ip, port=cfg.port,
                            cycle=cfg.cycle))
        robot.push = dict(robot.push, enable=bool(cfg.enable))
        return 0

    robot.rm_get_realtime_push = lambda: (0, dict(robot.push))
    robot.rm_set_realtime_push = set_push
    link = _link(monkeypatch, robot)
    out = link.feed_push_off()
    assert out["cleared"] and out["was"]["ip"] == "192.168.120.11" and out["now"]["enable"] is False
    assert written == [dict(enable=False, ip="192.168.120.11", port=8089, cycle=1)]          # only enable changed
    assert link.feed_push_off() == dict(cleared=False, was=out["now"], now=out["now"], said=["the push was already off"])
    robot.push["enable"] = True
    robot.rm_set_realtime_push = lambda cfg: -2                                              # never lands
    with pytest.raises(UserError, match="did not turn off"):
        link.feed_push_off()


def test_the_arm_connects_again_after_its_feed_push_is_turned_off(tmp_path):
    from alix_studio_lite.server.jobs import Jobs
    from alix_studio_lite.server.robot_standin import StandInRobot

    said = []
    log = SimpleNamespace(alert=lambda *a, **k: None, hardware=lambda side, verb, **k: said.append((side, verb, k.get("text"))))
    r = StandInRobot(tmp_path, Jobs(), log=log)
    r.arms["left"].connected = True
    link = FakeLink({})
    link.feed_push_off = lambda: dict(cleared=True, was={"ip": "192.168.120.11", "port": 8089, "cycle": 1},
                                      now={"enable": False}, said=[])
    r.links["left"] = link
    connected = []
    r.connect = lambda side, ip=None: connected.append(side)
    out = r.feed_push_off("left")
    assert out["reconnected"] and connected == ["left"]
    assert ("left", "feed push off", "was on toward 192.168.120.11:8089 every 5 ms; now off (read back)") in said


def test_a_push_toward_another_address_is_pointed_at_this_computer_when_the_feed_follows_it():
    """The user, 6 Nov: "by default the feed uses the IP address of the host computer, so when the address changes it
    changes with it". The lab's new controllers pushed to 192.168.120.11:8089 and 192.168.1.12:8090 while its PC was
    192.168.1.20: as the arm connects, such a push is pointed at this computer's address and the arm's feed port, and off
    until a run records; without the setting it is another computer's, left alone."""
    from alix_studio_lite.server.udp_push import left_on

    push = {"cycle": 1, "enable": True, "port": 8089, "ip": "192.168.120.11", "force_coordinate": -1}
    written = []

    def set_push(cfg):
        ip = cfg.ip.decode() if isinstance(cfg.ip, bytes) else cfg.ip
        written.append(dict(enable=cfg.enable, ip=ip, port=cfg.port))
        push.update(enable=bool(cfg.enable), ip=ip, port=int(cfg.port))
        return 0

    robot = SimpleNamespace(rm_get_realtime_push=lambda: (0, dict(push)), rm_set_realtime_push=set_push)
    handle = SimpleNamespace(robot=robot, host_ip="192.168.1.20")
    assert left_on(handle) is None and not written                                      # not following: left alone
    out = left_on(handle, follow_host=True, port=9089)
    assert out["cleared"] and out["elsewhere"] and out["ip"] == "192.168.120.11" and out["host"] == "192.168.1.20"
    assert written == [dict(enable=False, ip="192.168.1.20", port=9089)]               # this computer, its feed port, off
    assert left_on(handle, follow_host=True, port=9089) is None                          # off now: nothing more to do
