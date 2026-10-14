"""The live link to the arms (server/robot_link.py), through ALIX's own robot I/O code: connecting
each controller, the feedback of the arm, the pole and the hand, the E-stop input (fail closed),
STOP reaching the controllers like the E-stop, and nothing else commanded (no Real robot while
linked)."""
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
    assert estop_state(io(5, 0))[0] == "CLEAR" and estop_state(io(5, 1))[0] == "ENGAGED"
    assert estop_state(io(5, 0), active_level=0)[0] == "ENGAGED"                       # a site wired active-low
    assert estop_state(io(4, 0))[0] == "NOT_CONFIGURED"                                # RealMan's demo mapping
    assert estop_state(io(5, 0, ret=-2))[0] == "UNKNOWN"

    def boom():
        raise OSError("no reply")

    assert estop_state(boom)[0] == "UNKNOWN"
    assert error_codes({"err": ["0"]}) == [] and error_codes({"err": ["4109", "0x100D", "x"]}) == [4109, 4109]


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    """Lite with a live link to ALIX's emulated arms, in its own process (the emulator replaces
    the SDK module for the whole process)."""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    data = tmp_path_factory.mktemp("lite-link")
    proc = subprocess.Popen([sys.executable, "-m", "alix_studio_lite.server", "--robot", "emulated", "--no-browser",
                             "--port", str(port), "--data", str(data)],
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
    assert "read only" in st["label"] and all(st["arms"][s]["link"]["feedback"]["age_s"] < 1 for s in ("left", "right"))


def test_lite_commands_nothing_through_the_link(server):
    assert call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})["code"] == "mode"
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
