"""The live link to the arms (server/robot_link.py), through ALIX's own robot I/O code: connecting
each controller, the feedback of the arm, the pole and the hand, the E-stop input (fail closed),
STOP reaching the controllers like the E-stop, and Real robot available with it."""
import json
import socket
import subprocess
import sys
import time
import urllib.request

import pytest

from alix_studio_lite import vendor_paths


def test_estop_input_fails_closed():
    from alix_studio_lite.server.robot_link import error_codes, estop_state

    io = lambda mode, level, ret=0: (lambda: (ret, {"io_state": level, "io_config": {"io_mode": mode}}))  # noqa: E731
    # a normally-closed button to ground (RealMan's circuit-type IO E-stop): 1 released, 0 pressed
    assert estop_state(io(5, 1))[0] == "CLEAR" and estop_state(io(5, 0))[0] == "ENGAGED"
    assert estop_state(io(5, 1), active_level=1)[0] == "ENGAGED"                       # wired the other way
    assert estop_state(io(4, 0))[0] == "NOT_CONFIGURED"                                # RealMan's demo mapping
    assert estop_state(io(5, 0, ret=-2))[0] == "UNKNOWN"

    def boom():
        raise OSError("no reply")

    assert estop_state(boom)[0] == "UNKNOWN"
    assert error_codes({"err": ["0"]}) == [] and error_codes({"err": ["4109", "0x100D", "x"]}) == [4109, 4109]


def test_a_command_waits_for_a_reading_under_way_and_none_starts_meanwhile():
    """On the lab's first run a reading overlapping the hand's staging made the hand's command time
    out: a command starts only once the reading under way has ended, and the loop reads nothing
    while a command runs."""
    import threading

    from alix_studio_lite.server.robot_link import ArmLink

    link = ArmLink("left", "127.0.0.1")
    link.handle = object()
    reads = []

    def slow_read():
        t0 = time.time()
        time.sleep(0.3)
        reads.append((t0, time.time()))
        return {}

    link._read_all = slow_read
    reader = threading.Thread(target=link._read_if_idle)
    reader.start()
    time.sleep(0.05)                                         # the reading is under way
    with link.busy("moving the hand"):
        started = time.time()
        assert reads and started >= reads[0][1]               # it ended before the command started
        link._read_if_idle()                                  # the loop's turn comes: nothing is read
        assert len(reads) == 1
    reader.join()
    assert len(reads) == 2                                    # read again as the command ended


def test_with_its_own_connection_the_link_reads_through_a_command_and_the_hand_between_them(monkeypatch):
    """The owner, 26 Oct: the vendor's GUI shows the joints, temperatures and currents while a program plays, on a
    connection of its own; a reading on its own connection costs the commands nothing and the controller's clock
    nothing. So each link opens a second connection for its readings: the arm, the pole, the controller and IO3 are
    read whether a command runs or not; the hand (Modbus on the end port) only between commands, as before."""
    from types import SimpleNamespace

    from alix_studio_lite.server.robot_link import ArmLink

    link = ArmLink("left", "127.0.0.1")
    cmd, rd = SimpleNamespace(robot=object()), SimpleNamespace(robot=object())
    link.handle, link.reader = cmd, rd
    link.feedback = {"hand": [1, 2, 3, 4, 5, 6]}
    reads = []
    link._read_all = lambda robot=None, hand=True: reads.append((robot, hand)) or {}
    link._read_if_idle()
    assert reads == [(rd.robot, True)]                           # idle: everything, on the readings' connection
    with link.busy("running program 81"):
        link._read_if_idle()
        link._read_if_idle()
    assert reads[1:3] == [(rd.robot, False), (rd.robot, False)]   # a command runs: read still, the hand left out
    # as the command ends nothing is read on its connection (27 Oct: that reading held every command's caller 0.1 to
    # 0.35 s); the loop is woken to read what it left, on the readings' connection, the caller not waiting for it
    assert len(reads) == 3 and link._wake.is_set()
    assert link.view()["reader"] is True
    link.reader = None
    with link.busy("running program 81"):
        link._read_if_idle()
        assert len(reads) == 3                                    # no reader: nothing while busy, as before
    assert reads[-1] == (None, True) and len(reads) == 4          # ... and busy()'s own reading as the command ended, as before
    assert link.view()["reader"] is False


def test_the_hand_is_read_in_a_lull_and_a_command_waits_only_for_a_hand_read_under_way(monkeypatch):
    """27 Oct: a command arriving during the loop's reading waited for it to end (up to 0.2 s, some 60 times a run of
    S2). On a link with its own readings connection only the hand's Modbus read is shared with the commands (the end
    port), so a command waits for that alone; and the hand is read only in a lull, HAND_LULL_S after the last command
    ended (a run's commands come closer than that, so none waits for a hand read). Without the readings' connection
    everything is as before: a command waits for the whole reading under way."""
    import threading
    from types import SimpleNamespace

    from alix_studio_lite.server import robot_link
    from alix_studio_lite.server.robot_link import ArmLink

    monkeypatch.setattr(robot_link, "HAND_LULL_S", 0.3)
    link = ArmLink("left", "127.0.0.1")
    link.handle, link.reader = SimpleNamespace(robot=object()), SimpleNamespace(robot=object())
    hands = []
    link._read_hand = lambda robot: hands.append(time.monotonic()) or [7] * 6
    monkeypatch.setattr(robot_link, "_joint_floats", lambda *a, **k: None)
    monkeypatch.setattr(robot_link, "_controller_state", lambda robot: None)
    from alix_dispatch import peripherals, runmode

    monkeypatch.setattr(runmode, "read_run_mode", lambda robot: (0, 1))
    monkeypatch.setattr(peripherals, "Pole", lambda robot, log=None: SimpleNamespace(height_m=lambda: 0.15))
    link.feedback = {"t": time.time(), "hand": [1] * 6}
    with link.busy("moving the hand"):
        pass
    link._read_if_idle()
    assert not hands and link.feedback["hand"] == [1] * 6        # right after a command: the hand is not read (the lull)
    time.sleep(0.35)
    link._read_if_idle()
    assert len(hands) == 1 and link.feedback["hand"] == [7] * 6   # in the lull: read

    def slow_hand(robot):                                         # a hand read under way as a command arrives
        time.sleep(0.3)
        hands.append(time.monotonic())
        return [8] * 6

    link._read_hand = slow_hand
    reading = threading.Thread(target=link._read_if_idle)
    reading.start()
    time.sleep(0.1)
    t0 = time.monotonic()
    with link.busy("moving the hand"):
        waited = time.monotonic() - t0
    reading.join()
    assert 0.15 < waited < 0.6                                    # it waited for the hand read, not longer
    link._read_hand = lambda robot: [9] * 6
    link._ended = 0.0

    def slow_arm_state():                                         # a reading under way, past its hand: not waited for
        time.sleep(0.3)
        return 0, {"joint": [0.0] * 7, "pose": [0.0] * 6, "err": None}

    link.reader = SimpleNamespace(robot=SimpleNamespace(rm_get_current_arm_state=slow_arm_state))
    reading = threading.Thread(target=link._read_if_idle)
    reading.start()
    time.sleep(0.05)
    t0 = time.monotonic()
    with link.busy("moving the pole"):
        waited = time.monotonic() - t0
    reading.join()
    assert waited < 0.1


def test_the_readings_connection_is_opened_first_and_a_refused_one_leaves_the_old_way(monkeypatch):
    """ALIX's emulator stamps its arrival events with the handle made last, which must be the commands'; on the robot the
    order does not matter. A second connection the controller refuses is not an error: the readings then pause while a
    command runs, as before. LITE_LINK_READER=0 asks for that."""
    from types import SimpleNamespace

    from alix_dispatch import connect

    from alix_studio_lite.server.robot_link import ArmLink

    opened, answers = [], []
    monkeypatch.setattr(connect, "open_arm", lambda side, ip=None, port=None, **kw: opened.append((side, ip)) or answers.pop(0))
    monkeypatch.setattr(connect, "sdk_provenance", lambda robot: "emulated")
    monkeypatch.setattr(connect, "close_arm", lambda h: None)

    def make():
        link = ArmLink("left", "127.0.0.1")
        link._read_all = lambda robot=None, hand=True: {}
        return link

    answers[:] = [SimpleNamespace(robot=SimpleNamespace(), tag="reader"), SimpleNamespace(robot=SimpleNamespace(), tag="commands")]
    link = make().open()
    try:
        assert len(opened) == 2 and link.reader.tag == "reader" and link.handle.tag == "commands"
    finally:
        link.close()
    answers[:] = [None, SimpleNamespace(robot=SimpleNamespace(), tag="commands")]        # the second connection refused
    link = make().open()
    try:
        assert link.reader is None and link.handle.tag == "commands" and link.view()["reader"] is False
    finally:
        link.close()
    monkeypatch.setenv("LITE_LINK_READER", "0")
    opened.clear()
    answers[:] = [SimpleNamespace(robot=SimpleNamespace(), tag="commands")]
    link = make().open()
    try:
        assert len(opened) == 1 and link.reader is None
    finally:
        link.close()


def test_the_end_of_a_long_command_is_not_a_link_that_stopped_reading():
    """The lab's logs (16 Oct): after every long command (a program run, an upload, a saved take) "No fresh
    reading" was raised in the moment between the command's end and the reading that follows it, and the
    E-stop shown UNKNOWN. The end of a command counts as fresh for STALE_S; a controller that then stays
    silent is stale as before."""
    from alix_studio_lite.server.robot_link import STALE_S, ArmLink

    link = ArmLink("left", "127.0.0.1")
    link.handle = None                                        # no reading follows here: the moment after the end
    link.feedback = {"t": time.time(), "estop": "CLEAR", "busy": None}         # fresh as the command begins
    with link.busy("running program 81"):
        link.feedback["t"] -= 20.0                            # ... and 20 s long: nothing is read meanwhile
        fb = link.view()["feedback"]
        assert not fb["stale"] and fb["estop"] == "CLEAR" and fb["busy"] == "running program 81"
    fb = link.view()["feedback"]
    assert fb["age_s"] > 19 and not fb["stale"] and fb["estop"] == "CLEAR"          # just ended: its reading is coming
    link._freed -= STALE_S + 0.1                              # ... and none came
    fb = link.view()["feedback"]
    assert fb["stale"] and fb["estop"] == "UNKNOWN"
    link.feedback = {"t": time.time() - 3.0, "estop": "CLEAR", "busy": None}
    link._freed = 0.0
    assert link.view()["feedback"]["stale"]                   # no command at all: stale after STALE_S as before
    link.feedback = {"t": time.time() - 120.0, "estop": "CLEAR", "busy": None}     # stale before a short command
    with link.busy("reading the program list"):
        pass
    fb = link.view()["feedback"]
    assert fb["stale"] and fb["estop"] == "UNKNOWN"           # a command never makes a stale link fresh


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    """Lite with a live link to ALIX's emulated arms, in its own process (the emulator replaces
    the SDK module for the whole process)."""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    data = tmp_path_factory.mktemp("lite-link")
    proc = subprocess.Popen([sys.executable, "-m", "alix_studio_lite.server", "--robot", "emulated", "--no-browser",
                             "--port", str(port), "--data", str(data), "--logs", str(data / "logs"), "--projects", str(data / "projects")],
                            cwd=str(vendor_paths.LITE_ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    ctx = {"base": f"http://127.0.0.1:{port}"}
    for _ in range(100):
        try:
            ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
            break
        except OSError:
            time.sleep(0.2)
    yield ctx
    proc.terminate()
    proc.wait(10)


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json"}
    if ctx.get("tok"):
        headers["X-Lite-Token"] = ctx["tok"]
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=60) as resp:
        return json.loads(resp.read())


def test_connect_and_read_the_three_devices(server):
    for side in ("left", "right"):
        arm = call(server, "POST", f"/api/robots/{side}/connect", {})
        f = arm["link"]["feedback"]
        assert arm["connected"] and arm["link"]["emulated"] and arm["estop"] == "CLEAR"
        assert len(f["joints_deg"]) == 7 and f["errors"] == [] and f["run_mode"] in ("SIM", "REAL")   # the arm
        assert f["pole_mm"] == 100.0                                                                    # the pole
        assert len(f["hand"]) == 6 and all(0 <= c <= 1000 for c in f["hand"])                          # the hand
    time.sleep(0.6)
    st = call(server, "GET", "/api/robots/status")
    assert "Live link" in st["label"] and st["backend"] == "emulated"
    assert all(st["arms"][s]["link"]["feedback"]["age_s"] < 1 for s in ("left", "right"))
    assert all(st["arms"][s]["link"]["reader"] for s in ("left", "right"))           # the readings' own connection (26 Oct)


def test_real_robot_is_available_with_the_link(server):
    """Lite commands the linked arms (robot_real.py; tests/test_robot_real.py runs the loop)."""
    assert call(server, "POST", "/api/robots/mode", {"mode": "real"})["code"] == "confirm"
    assert call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})["mode"] == "real"
    assert call(server, "POST", "/api/robots/mode", {"mode": "hwsim"})["mode"] == "hwsim"


def test_the_estop_interlock_follows_the_arm(server):
    call(server, "POST", "/api/auth/engineer", {"pin": "1234"})
    assert call(server, "POST", "/api/robots/left/estop-test", {"pressed": True})["estop"] == "ENGAGED"
    assert "E-stop" in call(server, "GET", "/api/robots/status")["motion_block"]
    assert "still pressed" in call(server, "POST", "/api/robots/left/recover", {})["error"]
    assert call(server, "POST", "/api/robots/left/estop-test", {"pressed": False})["estop"] == "RECOVERING"
    assert call(server, "POST", "/api/robots/left/recover", {})["estop"] == "CLEAR"
    assert call(server, "GET", "/api/robots/status")["motion_block"] is None


def test_stop_reaches_the_controllers_and_holds_like_the_estop(server):
    """STOP BOTH: ALIX's stop on each controller the E-stop's way (the arm, the pole, the hand
    frozen), proven still, no log-in needed; then Lite holds both arms until Recover."""
    r = call({"base": server["base"]}, "POST", "/api/robots/stop-both", {})
    for side in ("left", "right"):
        c = r["result"]["controllers"][side]
        assert c["ok"] and c["arm"]["verb"] == "rm_set_arm_stop" and c["arm"]["ret"] == 0
        assert c["pole"]["ok"] and c["hand"]["ok"] and c["still"]
    assert "error" not in r and "press Recover on the LEFT and RIGHT arms" in r["message"]
    st = call(server, "GET", "/api/robots/status")
    assert all(st["arms"][s]["estop"] == "RECOVERING" and st["arms"][s]["stopped"] for s in ("left", "right"))
    assert "stopped by STOP" in st["motion_block"]
    assert all(st["arms"][s]["link"]["feedback"]["age_s"] < 1 for s in ("left", "right"))   # still reading
    for side in ("left", "right"):
        assert call(server, "POST", f"/api/robots/{side}/recover", {})["estop"] == "CLEAR"
    one = call({"base": server["base"]}, "POST", "/api/robots/left/stop", {})     # one arm: the slow stop on its path
    assert one["result"]["controllers"]["left"]["arm"]["verb"] == "rm_set_arm_slow_stop"
    assert list(one["result"]["controllers"]) == ["left"] and one["result"]["held"] == ["left"]
    assert call(server, "POST", "/api/robots/left/recover", {})["estop"] == "CLEAR"


def test_an_unreachable_controller_is_refused_plainly(server):
    call(server, "POST", "/api/robots/right/disconnect", {})
    r = call(server, "POST", "/api/robots/right/connect", {"ip": "192.168.1.77"})
    assert "cannot reach the RIGHT arm's controller at 192.168.1.77:8080" in r["error"]
    assert not call(server, "GET", "/api/robots/status")["arms"]["right"]["connected"]


def test_a_hand_read_that_fails_puts_the_end_port_back_in_modbus_mode(monkeypatch):
    """27 Oct lab: the panel's hand was empty for 21 to 56 % of the readings, for minutes at a time: runner v14's Hand steps
    (and Lite's own pole and hand step, until that night) close the end port's Modbus mode after each use, and the
    readings' connection set it only once per link, so every read after a close failed until Lite's next hand command.
    A failed read now sets the mode again, at most every HAND_REOPEN_S, and reads again; a hand that stays silent is
    not hammered. The vendor's web GUI never closes the mode, which is why it never loses the hand."""
    from types import SimpleNamespace

    from alix_dispatch import peripherals

    from alix_studio_lite.server import robot_link
    from alix_studio_lite.server.robot_link import ArmLink

    monkeypatch.setattr(robot_link, "HAND_REOPEN_S", 0.3)
    state = {"mode": False, "sets": 0}

    class Hand:
        def __init__(self, robot, side, log=None):
            self.robot = robot

        def read(self):
            return [1, 2, 3, 4, 5, 6] if state["mode"] else None

    def set_mode(port, baud, timeout):
        state["sets"] += 1
        state["mode"] = True
        return 0

    monkeypatch.setattr(peripherals, "HandModbus", Hand)
    link = ArmLink("left", "127.0.0.1")
    robot = SimpleNamespace(rm_set_modbus_mode=set_mode)
    assert link._read_hand(robot) == [1, 2, 3, 4, 5, 6] and state["sets"] == 1       # closed: set, then read
    assert link._read_hand(robot) == [1, 2, 3, 4, 5, 6] and state["sets"] == 1       # open: read alone
    state["mode"] = False                                                           # a Hand step of runner v14 closed it
    assert link._read_hand(robot) is None and state["sets"] == 1                     # within HAND_REOPEN_S of the last set: not yet
    time.sleep(0.35)
    assert link._read_hand(robot) == [1, 2, 3, 4, 5, 6] and state["sets"] == 2       # set again, read
    silent = SimpleNamespace(rm_set_modbus_mode=lambda *a: state.__setitem__("sets", state["sets"] + 1) or 0)
    link._hand = None
    monkeypatch.setattr(peripherals, "HandModbus", type("Dead", (), {"__init__": lambda self, robot, side, log=None: setattr(self, "robot", robot),
                                                                       "read": lambda self: None}))
    time.sleep(0.35)
    assert link._read_hand(silent) is None and state["sets"] == 3                     # a dead hand: one set per HAND_REOPEN_S
    assert link._read_hand(silent) is None and state["sets"] == 3


def test_a_stop_goes_through_the_readings_connection_and_a_stuck_command_call_does_not_hold_it(monkeypatch):
    """27 Oct night: a STOP went through the commands' connection, behind whatever call was stuck on it (20 s before the
    watch gave the link up). With the readings' connection (26 Oct) the STOP goes through it; the commands' connection
    carries it only when the readings' has a stuck call of its own; with both stuck nothing passes (a new connection)."""
    from types import SimpleNamespace

    from alix_dispatch import staging, track

    from alix_studio_lite.server.robot_link import ArmLink

    cmd, rdr, used = SimpleNamespace(name="cmd"), SimpleNamespace(name="reader"), []
    monkeypatch.setattr(staging, "stop_devices", lambda h, **kw: used.append((h.robot, kw.get("hand").robot if kw.get("hand") else None))
                        or dict(stop=dict(arm=dict(ok=True, verb="rm_set_arm_stop", ret=0), pole=dict(ok=True), hand=dict(ok=True))))
    monkeypatch.setattr(track, "stop_program", lambda robot, **kw: dict(stopped=True))
    k = ArmLink("right", "192.168.1.103", period_s=10.0)
    k.handle, k.reader = SimpleNamespace(robot=cmd), SimpleNamespace(robot=rdr)
    k.feedback = {"hand": [0, 0, 0, 0, 150, 1000]}
    out = k.stop(hard=True)
    assert out["ok"] and out["through"] == "reader" and used[-1] == (rdr, rdr)     # the hand frozen through the same connection
    k._calls[1] = ("rm_set_plan_speed", time.monotonic() - 100, "cmd", False)      # a command's call that never came back
    assert k.stuck() and k.stuck("cmd") and k.stuck("reader") is None
    out = k.stop(hard=True)
    assert out["ok"] and out["through"] == "reader" and used[-1][0] is rdr
    k._calls[2] = ("rm_get_current_arm_state", time.monotonic() - 100, "reader", False)   # the readings' connection too
    out = k.stop(hard=True)
    assert not out["ok"] and out["stuck"] and out["stuck_call"][0] in ("rm_set_plan_speed", "rm_get_current_arm_state")
    assert "its connections are stuck" in out["text"] and len(used) == 2
    del k._calls[1]                                                                # the readings' alone: the commands' carries it
    out = k.stop(hard=False)
    assert out["ok"] and out["through"] == "cmd" and used[-1][0] is cmd
    assert k.leave_drag() == -1                                                    # through a handle whose robot has no SDK: -1, no raise


def test_a_stops_own_call_is_stuck_after_three_seconds():
    """A STOP's own call (made while the STOP is being sent) is bounded by STOP_STUCK_S, not the 20 s of a command's
    (26 Oct 10:09: rm_set_arm_stop unanswered 20 s); the watch then gives the link up and the STOP goes again."""
    from types import SimpleNamespace

    from alix_studio_lite.server.robot_link import STOP_STUCK_S, ArmLink

    rdr = SimpleNamespace(name="reader")
    k = ArmLink("right", "192.168.1.103", period_s=10.0)
    k.handle, k.reader = SimpleNamespace(robot=SimpleNamespace()), SimpleNamespace(robot=rdr)
    assert STOP_STUCK_S == 3.0 and k._conn_of(rdr) == "reader" and k._conn_of(k.handle.robot) == "cmd" and k._conn_of(None) == "cmd"
    k._stopping.set()
    k._enter("rm_set_arm_stop", rdr)
    tid = next(iter(k._calls))
    name, t0, conn, stopping = k._calls[tid]
    assert (name, conn, stopping) == ("rm_set_arm_stop", "reader", True)
    k._calls[tid] = (name, time.monotonic() - 2.0, conn, stopping)
    assert k.stuck() is None and k.in_flight()[0] == "rm_set_arm_stop"
    k._calls[tid] = (name, time.monotonic() - 3.5, conn, stopping)
    assert k.stuck()[0] == "rm_set_arm_stop" and k.stuck("reader") and k.stuck("cmd") is None
    k._stopping.clear()
    k._calls[tid] = ("rm_get_current_arm_state", time.monotonic() - 3.5, "reader", False)    # a reading's call: the 20 s bound
    assert k.stuck() is None
