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
    assert reads[-1] == (None, True) and len(reads) == 4          # busy()'s own reading as the command ended (read_once: the reader's robot inside)
    assert link.view()["reader"] is True
    link.reader = None
    with link.busy("running program 81"):
        link._read_if_idle()
        assert len(reads) == 4                                    # no reader: nothing while busy, as before
    assert reads[-1] == (None, True) and len(reads) == 5
    assert link.view()["reader"] is False


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
