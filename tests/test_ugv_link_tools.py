"""The UGV link's two helpers against each other (the user, 11 Nov): tools/ugv_link_standin.py, the arm side's stand-in
for the UGV side to develop against (the arm side's own protocol code, nothing of Lite's, nothing moved), and
tools/ugv_link_client.py, the UGV's stand-in for the lab. And how a failed run's reason becomes an abort code."""
import contextlib
import json
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from types import SimpleNamespace

import pytest

from alix_studio_lite import vendor_paths

TOOLS = vendor_paths.LITE_ROOT / "tools"


@contextlib.contextmanager
def running(*args, port=None):
    """A stand-in arm side on a free port (or ``port``), with the token "tok": its URL, until the block ends."""
    if port is None:
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
    # -I: no PYTHONPATH, no site of the user's: the stand-in needs only the standard library and ugv_link.py's file
    proc = subprocess.Popen([sys.executable, "-I", str(TOOLS / "ugv_link_standin.py"), "--port", str(port), "--token", "tok", *args],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    url = f"http://127.0.0.1:{port}"
    try:
        for _ in range(100):
            if client(url, "status", check=False).returncode in (0, 1):
                break
            time.sleep(0.1)
        yield url
    finally:
        proc.terminate()
        proc.wait(10)


@pytest.fixture(scope="module")
def standin():
    with running("--seconds", "1.2", "--stages", "3", "--recover", "1.5", "--fixtures", "cubicle_1,Cubicle 2",
                 "--outcomes", "succeeded,aborted:MECHANISM_FAILED,unsafe,aborted-safe:INSPECTION_UNDECIDED") as url:
        yield url


def setting(url, body=None, token="tok"):
    """The stand-in's own controls: GET /standin, or POST it ``body``; (HTTP code, the answer)."""
    req = urllib.request.Request(url + "/standin", method="GET" if body is None else "POST",
                                 headers={"Content-Type": "application/json", "X-Link-Token": token},
                                 data=None if body is None else json.dumps(body).encode())
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def client(url, *args, token="tok", check=True):
    out = subprocess.run([sys.executable, "-I", str(TOOLS / "ugv_link_client.py"), "--url", url, "--token", token, *args],
                         capture_output=True, text=True, timeout=60)
    if check:
        assert out.returncode in (0, 1), out.stdout + out.stderr
    return out


def test_the_stand_ins_run_a_goal_each_way_it_can_end(standin):
    st = client(standin, "status")
    assert st.returncode == 0 and "mode auto; ready: yes" in st.stdout and "safe_to_move: yes" in st.stdout
    assert "x +-10 mm, y +-10 mm, yaw +-1 deg" in st.stdout and "attempts a visit: 3" in st.stdout
    far = client(standin, "clean", "--fixture", "cubicle_1", "--yaw-deg", "1.6")
    assert far.returncode == 1 and "REJECTED OFFSET_OUT_OF_TOLERANCE; the UGV: RETRY_APPROACH; outside: yaw" in far.stdout
    assert "REJECTED UNKNOWN_FIXTURE" in client(standin, "clean", "--fixture", "cubicle_9").stdout
    ok = client(standin, "clean", "--fixture", "cubicle_1", "--x-mm", "3", "--y-mm", "-2", "--yaw-deg", "0.4", "--attempt", "2")
    assert ok.returncode == 0, ok.stdout + ok.stderr
    assert "ACCEPTED; the UGV: HOLD; expected 1.2 s" in ok.stdout and "EXECUTING: stage 1 of 3" in ok.stdout
    assert "RESULT SUCCEEDED; safe_to_move: yes; the UGV: CONTINUE_MISSION; 3 stage(s) done" in ok.stdout
    bad = client(standin, "clean", "--fixture", "cubicle  2")                # the second outcome: an abort, the arms out
    assert bad.returncode == 1 and "RESULT ABORTED, MECHANISM_FAILED; safe_to_move: NO; the UGV: CALL_OPERATOR" in bad.stdout
    held = client(standin, "clean", "--fixture", "Cubicle 2")                # as Lite: manual until a person releases again
    assert "REJECTED MANUAL_MODE; the UGV: CALL_OPERATOR" in held.stdout and "releases the sequence again" in held.stdout
    st = client(standin, "status").stdout
    assert "mode manual; ready: no, MANUAL_MODE" in st and "safe_to_move: NO" in st
    time.sleep(1.7)
    assert "mode auto; ready: yes" in client(standin, "status").stdout
    unsafe = client(standin, "clean", "--fixture", "cubicle_1")              # the third: completed, an arm not back
    assert unsafe.returncode == 1 and "RESULT SUCCEEDED; safe_to_move: NO; the UGV: CALL_OPERATOR" in unsafe.stdout
    time.sleep(1.7)
    safe = client(standin, "clean", "--fixture", "cubicle_1", "--goal-id", "kept-1")     # the fourth: aborted, the arms back
    assert "RESULT ABORTED, INSPECTION_UNDECIDED; safe_to_move: yes; the UGV: CONTINUE_MISSION" in safe.stdout
    again = client(standin, "result", "kept-1")                              # a result is kept: asked again by its id
    assert "RESULT ABORTED, INSPECTION_UNDECIDED" in again.stdout
    started = client(standin, "clean", "--fixture", "cubicle_1", "--goal-id", "stop-1", "--no-wait")
    assert started.returncode == 0 and "ACCEPTED" in started.stdout
    stopped = client(standin, "cancel", "stop-1")
    assert "RESULT CANCELED; safe_to_move: NO; the UGV: CALL_OPERATOR" in stopped.stdout
    after = client(standin, "clean", "--fixture", "cubicle_1")               # a cancel is a STOP: a person releases again
    assert "REJECTED MANUAL_MODE" in after.stdout and "STOP" in after.stdout


def test_every_reject_code_and_every_abort_code_can_be_asked_of_the_stand_in():
    """The UGV side, 12 Nov: "to be able to return each reject code and abort code on demand"."""
    from alix_studio_lite.server import ugv_link

    asked = ("reject:OFFSET_OUT_OF_TOLERANCE,reject:ATTEMPTS_EXHAUSTED:x+y,reject:BUSY,reject:ARM_NOT_READY,"
             "reject:MANUAL_MODE,reject:UNKNOWN_FIXTURE,reject:BAD_GOAL")
    with running("--seconds", "0.3", "--stages", "2", "--recover", "0", "--outcomes", asked) as url:
        code, view = setting(url)
        assert code == 200 and view["standin"] is True and view["next_outcome"] == "reject:OFFSET_OUT_OF_TOLERANCE"
        assert client(url, "status").returncode == 0 and setting(url)[1]["next_outcome"] == "reject:OFFSET_OUT_OF_TOLERANCE"
        for want, outside in (("OFFSET_OUT_OF_TOLERANCE", "yaw"), ("ATTEMPTS_EXHAUSTED", "x, y"), ("BUSY", ""), ("ARM_NOT_READY", ""),
                              ("MANUAL_MODE", ""), ("UNKNOWN_FIXTURE", ""), ("BAD_GOAL", "")):
            out = client(url, "clean", "--fixture", "Cubicle 1", "--x-mm", "1").stdout          # a goal Lite would take
            assert f"REJECTED {want}; the UGV: {ugv_link.ACTIONS[want]}" + (f"; outside: {outside};" if outside else ";") in out, out
            assert f"asked for with reject:{want}" in out
        assert sorted(o.split(":")[1] for o in view["outcomes"]) == sorted(ugv_link.ACTIONS)      # all seven, each once
        for want in ugv_link.ABORT_CODES:                                    # set while it runs, with no restart
            code, view = setting(url, {"outcomes": f"aborted:{want}"})
            assert code == 200 and view["next_outcome"] == f"aborted:{want}"
            out = client(url, "clean", "--fixture", "Cubicle 1").stdout
            assert f"RESULT ABORTED, {want}; safe_to_move: NO; the UGV: CALL_OPERATOR" in out, out
        assert setting(url, {"outcomes": "aborted-safe:DEVICE_FAULT"})[0] == 200
        assert "RESULT ABORTED, DEVICE_FAULT; safe_to_move: yes; the UGV: CONTINUE_MISSION" in client(
            url, "clean", "--fixture", "Cubicle 1").stdout
        code, view = setting(url, {"manual": True, "fixtures": "Cubicle 1, Cubicle 2", "outcomes": "succeeded", "seconds": 0.2})
        assert view["mode"] == "manual" and view["fixtures"] == ["Cubicle 1", "Cubicle 2"] and view["seconds"] == 0.2
        assert "REJECTED MANUAL_MODE" in client(url, "clean", "--fixture", "cubicle 1").stdout
        assert setting(url, {"manual": False})[1]["mode"] == "auto"
        assert "REJECTED UNKNOWN_FIXTURE" in client(url, "clean", "--fixture", "Cubicle 3").stdout
        assert "RESULT SUCCEEDED; safe_to_move: yes" in client(url, "clean", "--fixture", "cubicle 1").stdout
        for body, said in (({"outcomes": "reject:NOT_A_CODE"}, "is not succeeded"), ({"outcomes": "reject:BUSY:x"}, "is not succeeded"),
                           ({"outcomes": "reject:OFFSET_OUT_OF_TOLERANCE:z"}, "the axes are x, y, yaw"),
                           ({"seconds": 0}, "seconds must be"), ({"speed": 2}, "not speed"), ({"manual": "yes"}, "true or false")):
            code, out = setting(url, body)
            assert code == 400 and said in out["error"], out
        assert setting(url, token="wrong")[0] == 401
        assert setting(url)[1]["outcomes"] == ["succeeded"]                  # a setting refused changed nothing


def test_a_stand_in_stopped_while_a_goal_runs_answers_arm_restarted_after(tmp_path):
    store = str(tmp_path / "goals.json")
    with running("--seconds", "30", "--store", store) as url:
        port = int(url.rsplit(":", 1)[1])
        assert "ACCEPTED" in client(url, "clean", "--fixture", "Cubicle 1", "--goal-id", "r-1", "--no-wait").stdout
    with running("--seconds", "30", "--store", store, port=port) as url:
        out = client(url, "result", "r-1").stdout
        assert "RESULT ABORTED, ARM_RESTARTED; safe_to_move: NO; the UGV: CALL_OPERATOR" in out, out


def test_the_client_says_when_the_arm_side_does_not_take_its_request(standin):
    out = client(standin, "status", token="wrong", check=False)
    assert out.returncode == 2 and "HTTP 401" in out.stderr
    out = client(standin, "result", "nobody", check=False)
    assert out.returncode == 2 and "HTTP 404" in out.stderr
    out = client("http://127.0.0.1:9", "status", check=False)               # nothing listens there
    assert out.returncode == 2 and "no answer" in out.stderr


@pytest.mark.parametrize("error, code, hint, want", [
    ("INTERRUPTED — E-STOP: it did not finish.", None, "Release the E-stop and follow Recover.", "ESTOP"),
    ("Not run: the camera cannot judge the lid. Nothing else moved.", "inspect", None, "INSPECTION_UNDECIDED"),
    ("verify_lid_lifted: the camera cannot judge whether it is lid lifted (no depth).", "escalate", None, "INSPECTION_UNDECIDED"),
    ("Open the lid: the lid is not lifted after 3 tries.", "escalate", "A person must look at the commode", "MECHANISM_FAILED"),
    ("Open the seat STOPPED: the wrist camera cannot be read (unplugged).", "camera", None, "DEVICE_FAULT"),
    ("Open the lid STOPPED: RIGHT joint 3 at 71 C.", "hot", None, "DEVICE_FAULT"),
    ("The LEFT hand does not answer: no reply. Robot motion is blocked on both arms until it answers.", None, None, "DEVICE_FAULT"),
    ("KeyError: 'x'", None, "Details are in the server log.", "INTERNAL_ERROR"),
    ("53 STOPPED // LEFT rc=1 // RIGHT rc=1: LEFT: Program 53 ended at 61 % of its lines' time", None, "Check the arms.", "MOTION_STOPPED"),
])
def test_a_failed_runs_reason_becomes_its_abort_code(error, code, hint, want):
    from alix_studio_lite.server import app  # noqa: F401 - Lite's routes load in their own order, from the app
    from alix_studio_lite.server import ugv_link, ugv_link_lite

    assert ugv_link_lite.abort_code(SimpleNamespace(error=error, code=code, hint=hint)) == want and want in ugv_link.ABORT_CODES


def test_the_release_s_fixture_names():
    from alix_studio_lite.server import app  # noqa: F401
    from alix_studio_lite.server.ugv_link_lite import fixture_names

    assert fixture_names(" cubicle_1, cubicle_2 ;cubicle_1,, ") == ["cubicle_1", "cubicle_2"]
    assert fixture_names("Cubicle 1, cubicle  1 ;Cubicle 2") == ["Cubicle 1", "Cubicle 2"]       # the UGV's cubicle names
    assert fixture_names("") == [] and fixture_names(None) == [] and fixture_names(["a", " b "]) == ["a", "b"]


def test_a_stop_or_an_estop_press_ends_the_release_and_an_unreadable_estop_does_not():
    from alix_studio_lite.server import app  # noqa: F401
    from alix_studio_lite.server.ugv_link_lite import LiteBackend

    said = []
    arm = lambda estop, latched=False: SimpleNamespace(connected=True, estop=estop, stop_latched=latched)      # noqa: E731
    robot = SimpleNamespace(motion_block=lambda: None, arms={"left": arm("CLEAR"), "right": arm("CLEAR")})
    lite = SimpleNamespace(ui={"mode": "real"}, robot=robot, log=SimpleNamespace(activity=lambda text, **kw: said.append(text)))
    backend = LiteBackend(lite)
    backend.release = dict(pid="p", fixtures=[])
    assert backend._release() is not None
    robot.arms["right"] = arm("UNKNOWN")                                # not read: motion is blocked, the release holds
    assert backend._release() is not None and backend.mode() == "auto"
    robot.arms["right"] = arm("ENGAGED")
    assert backend._release() is None and "RIGHT arm had its E-stop pressed" in backend.ended_why and backend.mode() == "manual"
    robot.arms["right"] = arm("CLEAR")
    assert backend._release() is None                                   # released again by a person, not by the button coming up
    backend.release = dict(pid="p", fixtures=[])
    robot.arms["left"] = arm("RECOVERING", latched=True)
    backend.watch()
    assert backend.release is None and "LEFT arm was stopped by STOP" in backend.ended_why
    assert sum("the release ended" in t for t in said) == 2
