"""Each arm's connection in a process of its own (server/robot_process.py): the link and the commands run in the
arm's worker, their log written here; a call into the SDK that never comes back is seen through the worker's
heartbeat, the worker is KILLED (the call dies with it) and the arm is connected again in a fresh process; a STOP
pressed meanwhile is sent through the new one. The fake controller comes from tests/fake_controller_setup.py,
which the worker runs before it opens (LITE_WORKER_SETUP)."""
import os
import signal
import threading
import time
from pathlib import Path

from alix_studio_lite.server import robot_standin
from alix_studio_lite.server.jobs import Jobs
from alix_studio_lite.server.logbook import Logbook
from alix_studio_lite.server.robot_process import LinkProcess, RealArmProxy


def test_a_stuck_call_dies_with_its_process_and_the_arm_is_connected_again(tmp_path, monkeypatch):
    marker = tmp_path / "stuck"
    marker.write_text("")
    monkeypatch.setenv("LITE_WORKER_SETUP", str(Path(__file__).with_name("fake_controller_setup.py")))
    monkeypatch.setenv("LITE_FAKE_STUCK_MARKER", str(marker))
    monkeypatch.setenv("LITE_CALL_SLOW_S", "0.1")                            # the workers read these as they start
    monkeypatch.setenv("LITE_CALL_STUCK_S", "0.6")
    monkeypatch.setattr(robot_standin, "WATCH_PERIOD_S", 0.2)
    monkeypatch.setattr(robot_standin, "LINK_PROCESS", True)
    log, jobs = Logbook(tmp_path / "logs"), Jobs()
    r = robot_standin.StandInRobot(tmp_path, jobs, link="realman", log=log)
    try:
        r.connect("right")
        first = r.links["right"]
        assert isinstance(first, LinkProcess) and first._proc.is_alive() and not marker.exists()
        assert isinstance(r._real("right"), RealArmProxy)
        fb = first.read_fresh()
        assert fb["joints_deg"] == [0.0] * 7 and fb["estop"] == "CLEAR"           # read in the worker, answered here
        assert any(x["verb"] == "read global speed" for x in log.records("hardware", time.time() - 120, time.time() + 1))   # its log written here
        # a command whose SDK call never answers, in the worker (the lab's MoveJ to a global point)
        job = jobs.start("move", "RIGHT MoveJ to R_HME_POS", lambda job: r._real("right").restore_speed(50), arm="right")
        t_end = time.time() + 3                                                   # the worker's next heartbeat says it
        while time.time() < t_end and "rm_set_plan_speed has not answered" not in r.view()["right"]["quality"]["text"]:
            time.sleep(0.05)
        q = r.view()["right"]["quality"]
        assert "rm_set_plan_speed has not answered" in q["text"], q
        t_end = time.time() + 15
        while time.time() < t_end and (r.links.get("right") in (None, first) or "right" in r._recovering):
            time.sleep(0.1)
        new = r.links.get("right")
        assert new is not None and new is not first and new._proc.is_alive()
        assert first.abandoned and not first._proc.is_alive()                     # killed: the stuck call died with it
        assert job.state == "failed" and ("stopped answering" in job.error or "connection" in job.error)
        said = [a["text"] for a in log.alerts_view()]
        assert any("has not answered rm_set_plan_speed for" in s for s in said) and any("connected again" in s for s in said)
        assert list((tmp_path / "logs").glob("*/threads-right-*.txt"))           # the worker's own threads, written by it
        assert r.arms["right"].estop == "RECOVERING" and r.arms["right"].stop_latched
        # the new connection answers, reads, commands and stops
        assert new.read_fresh()["joints_deg"] == [0.0] * 7
        r._real("right").restore_speed(50)
        time.sleep(0.6)
        assert r.view()["right"]["quality"]["level"] == "good"
        out = r.stop("right")
        assert "RIGHT" in out["controllers"]["right"]["text"] and new._proc.is_alive()
    finally:
        r.close()
    assert not new._proc.is_alive()


def test_ctrl_c_is_the_servers_and_stop_still_passes_through_the_workers(tmp_path, monkeypatch, capfd):
    """Ctrl+C in Lite's terminal reaches every process started from it. The arm workers leave it to the server, whose
    shutdown sends STOP through them, then closes them (the lab, 21 Oct: both died of it, a traceback each, and the
    shutdown's STOP was "not confirmed" on both arms at every Ctrl+C since 20 Oct)."""
    monkeypatch.setenv("LITE_WORKER_SETUP", str(Path(__file__).with_name("fake_controller_setup.py")))
    monkeypatch.setattr(robot_standin, "LINK_PROCESS", True)
    log, jobs = Logbook(tmp_path / "logs"), Jobs()
    r = robot_standin.StandInRobot(tmp_path, jobs, link="realman", log=log)
    try:
        for side in ("left", "right"):
            r.connect(side)
        procs = {s: r.links[s]._proc for s in ("left", "right")}
        for p in procs.values():
            os.kill(p.pid, signal.SIGINT)                                 # as the terminal's Ctrl+C reaches them
        time.sleep(1.0)
        assert all(p.is_alive() for p in procs.values())
        out = r.stop("all")                                               # the shutdown's first step (server._close)
        for s in procs:
            text = out["controllers"][s]["text"]
            assert "no answer from its connection's process" not in text and "stuck" not in text, out
        assert r.links["left"].read_fresh()["estop"] == "CLEAR"           # they still answer
    finally:
        r.close()
    assert all(p.exitcode == 0 for p in procs.values())                  # ended by the server, not by the signal
    err = capfd.readouterr().err
    assert "KeyboardInterrupt" not in err and "Traceback" not in err, err


def test_a_ctrl_c_while_an_arm_worker_starts_is_dropped(monkeypatch, capfd):
    """The worker starts with SIGINT held (LinkProcess.open) until it has set Ctrl+C aside: a Ctrl+C in the moments
    it is starting (Python, its imports) neither kills it nor prints anything."""
    monkeypatch.setenv("LITE_WORKER_SETUP", str(Path(__file__).with_name("fake_controller_setup.py")))
    link = LinkProcess("left", "192.168.1.18", kind="realman")
    pressed = []

    def press():
        t_end = time.monotonic() + 10
        while time.monotonic() < t_end and (link._proc is None or link._proc.pid is None):
            time.sleep(0.0005)
        if link._proc is None or link._proc.pid is None:
            return
        t_end = time.monotonic() + 0.6
        while time.monotonic() < t_end:                                   # Ctrl+C again and again while it starts
            try:
                os.kill(link._proc.pid, signal.SIGINT)
            except ProcessLookupError:
                return
            pressed.append(time.monotonic())
            time.sleep(0.01)

    th = threading.Thread(target=press)
    th.start()
    try:
        link.open()
        th.join()
        assert len(pressed) > 20 and link._proc.is_alive()
        assert link.read_fresh()["joints_deg"] == [0.0] * 7
        assert signal.SIGINT not in signal.pthread_sigmask(signal.SIG_BLOCK, [])     # this thread's own mask is back
    finally:
        th.join()
        link.close()
    assert link._proc.exitcode == 0
    err = capfd.readouterr().err
    assert "KeyboardInterrupt" not in err and "Traceback" not in err, err
