"""One caller at a time on each controller, and a Lite that always closes (the lab, 17 Oct): the RIGHT
controller saved a 24 s recording while three MoveJ's switched its run mode (each read back "not taken")
and a global point list was read (-2); three STOP BOTH presses sent stop after stop to each controller at
once, none confirmed, the links were closed under them and no Connect answered after; Ctrl+C did not end
Lite and it took the system monitor; the terminal filled with BrokenPipeError tracebacks. And a deadlock
found here: the mode switch held the robot's lock while it waited for a link's reading, which ends under
that lock."""
import io
import sys
import threading
import time
from contextlib import redirect_stderr
from types import SimpleNamespace

import pytest


def link(side="right"):
    from alix_studio_lite.server.robot_link import ArmLink

    return ArmLink(side, "192.168.1.103", period_s=10.0)


def test_one_command_at_a_time_on_a_controller(monkeypatch):
    from alix_studio_lite.server import robot_link
    from alix_studio_lite.server.jobs import UserError

    monkeypatch.setattr(robot_link, "CMD_WAIT_S", 0.3)
    k, held, out = link(), threading.Event(), []

    def save():
        with k.busy("saving the recording"):
            held.set()
            time.sleep(1.0)

    saving = threading.Thread(target=save, daemon=True)
    saving.start()
    held.wait(2)
    t0 = time.monotonic()
    with pytest.raises(UserError) as exc:
        with k.busy("switching to REAL"):
            out.append("ran")
    assert out == [] and exc.value.code == "busy" and 0.25 < time.monotonic() - t0 < 0.9
    assert str(exc.value) == "The RIGHT controller is busy: saving the recording."
    with pytest.raises(UserError, match="busy: saving the recording"):
        k.read_fresh()                                                   # Capture: never alongside a command
    saving.join(3)

    def points():
        with k.busy("reading the global points"):
            held.set()
            time.sleep(0.15)

    held.clear()
    threading.Thread(target=points, daemon=True).start()
    held.wait(2)
    with k.busy("switching to REAL"):                                    # a short one before it: waited for
        with k.busy("reading back"):                                     # one caller's own steps nest
            out.append("ran")
    assert out == ["ran"] and not k._busy


def test_one_stop_at_a_time_and_stop_both_acts_at_once():
    k, sent, arm_stops = link(), [], []
    k.handle = SimpleNamespace(robot=SimpleNamespace(rm_set_arm_stop=lambda: arm_stops.append("rm_set_arm_stop")))

    def stop_now(hard):
        sent.append(hard)
        time.sleep(0.4)
        return dict(ok=True, text="RIGHT: stopped on its controller; the arm is still")

    k._stop_now = stop_now
    got = []
    first = threading.Thread(target=lambda: got.append(k.stop(hard=False)), daemon=True)
    first.start()
    time.sleep(0.1)
    again = [threading.Thread(target=lambda: got.append(k.stop(hard=True)), daemon=True) for _ in range(2)]
    for t in again:
        t.start()
    for t in [first, *again]:
        t.join(3)
    assert sent == [False]                                    # one stop sent, not three alongside each other
    assert arm_stops == ["rm_set_arm_stop"] * 2               # STOP BOTH: the arm the E-stop's way at once, each press
    assert [g["ok"] for g in got] == [True] * 3 and k.stop(hard=True)["ok"] and sent == [False, True]


def test_a_handle_is_never_deleted_under_a_call(monkeypatch):
    from alix_dispatch import connect

    closed = []
    monkeypatch.setattr(connect, "close_arm", lambda h: closed.append((h, time.monotonic())))
    k = link()
    k.handle = SimpleNamespace(robot=None)
    k._thread = threading.Thread(target=k._loop, daemon=True)
    k._thread.start()
    held, ended = threading.Event(), {}

    def stop():                                               # a STOP still being sent as the arm is disconnected
        with k._stop_lock:
            held.set()
            time.sleep(0.6)
            ended["t"] = time.monotonic()

    threading.Thread(target=stop, daemon=True).start()
    held.wait(2)
    k.close()
    k._thread.join(3)
    assert len(closed) == 1 and closed[0][1] >= ended["t"]    # deleted once the STOP had ended, not under it


def test_a_connect_that_gets_no_answer_is_said(tmp_path, monkeypatch):
    from alix_studio_lite.server import robot_standin
    from alix_studio_lite.server.jobs import Jobs, UserError

    monkeypatch.setattr(robot_standin, "CONNECT_WAIT_S", 0.2)
    r = robot_standin.StandInRobot(tmp_path, Jobs())

    class Slow:
        side, closed = "left", False

        def open(self):
            time.sleep(0.6)
            return self

        def close(self):
            self.closed = True

    s = Slow()
    with pytest.raises(UserError, match="The LEFT controller did not answer within 0.2 s, so the arm is not connected"):
        r._open_link(s)
    time.sleep(0.8)
    assert s.closed                                           # it answered later: let go at once, not left open


def test_the_mode_switch_never_waits_for_a_reading_under_the_robots_lock(tmp_path, monkeypatch):
    from alix_studio_lite.server import app as _routes  # noqa: F401 - the routes' modules load through it
    from alix_studio_lite.server import api_units
    from alix_studio_lite.server.jobs import Jobs
    from alix_studio_lite.server.robot_real import RealArm
    from alix_studio_lite.server.robot_standin import StandInRobot

    r = StandInRobot(tmp_path, Jobs())
    k = link("left")
    r.links["left"] = k
    r.arms["left"].connected = True
    monkeypatch.setattr(r, "view", lambda: {"left": {"connected": True}, "right": {"connected": False}})
    monkeypatch.setattr(r, "motion_block", lambda: None)

    def set_run_mode(self, mode):
        with self.link.busy(f"switching to {mode}"):
            return {self.side: 1}

    monkeypatch.setattr(RealArm, "set_run_mode", set_run_mode)
    monkeypatch.setattr(api_units, "session", lambda app: None)
    app = SimpleNamespace(robot=r, ui={"mode": "visual"})
    app.set_mode = lambda mode: app.ui.update(mode=mode)

    def reading():                                            # the link's reading: on_read takes the robot's lock
        with k._read_lock:
            time.sleep(0.3)
            with r.lock:
                pass

    threading.Thread(target=reading, daemon=True).start()
    time.sleep(0.05)
    done = {}
    t = threading.Thread(target=lambda: done.update(api_units.post_mode(app, {}, {"mode": "hwsim"}, None)), daemon=True)
    t.start()
    t.join(5)
    assert done.get("controllers") == {"left": "SIM"} and app.ui["mode"] == "hwsim" and r.switching is None


def test_a_page_gone_before_its_answer_prints_nothing():
    from alix_studio_lite.server.app import _Server

    srv = _Server.__new__(_Server)
    for exc, said in ((BrokenPipeError(32, "Broken pipe"), False), (ConnectionResetError(), False), (ValueError("x"), True)):
        err = io.StringIO()
        with redirect_stderr(err):
            try:
                raise exc
            except Exception:
                srv.handle_error(None, ("127.0.0.1", 47026))
        assert ("Traceback" in err.getvalue()) is said


def test_lite_always_closes(monkeypatch):
    """Its exit is bounded: past EXIT_LIMIT_S it quits as it is (what each thread was doing written first), and
    once STOP is out a second Ctrl+C quits at once."""
    from alix_studio_lite import server

    exits, handlers, dumps = [], {}, []
    monkeypatch.setattr(server, "EXIT_LIMIT_S", 0.3)
    monkeypatch.setattr(server.os, "_exit", lambda code: exits.append(code))
    monkeypatch.setattr(server, "_dump_threads", lambda root: dumps.append(root) or "threads-120000.txt")
    import signal

    monkeypatch.setattr(signal, "signal", lambda sig, fn: handlers.__setitem__(sig, fn))
    stopped = []
    robot = SimpleNamespace(stop=lambda which: stopped.append(which), close=lambda: time.sleep(0.8))
    app = SimpleNamespace(robot=robot, log=SimpleNamespace(root="/logs"))
    out = io.StringIO()
    monkeypatch.setattr(sys, "stdout", out)
    server._close(app, SimpleNamespace(server_close=lambda: None))
    assert stopped == ["all"] and exits == [1] and dumps == ["/logs"]               # the close that hung: quit
    assert "did not close within 0.3 s, so it quits now (what each thread was doing: threads-120000.txt)" in out.getvalue()
    handlers[signal.SIGINT](signal.SIGINT, None)                                    # Ctrl+C again, STOP being out
    assert exits == [1, 130] and "Quitting now" in out.getvalue()
