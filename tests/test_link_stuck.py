"""A call into the controller's SDK that never comes back (server/robot_link.py, server/robot_standin.py). The lab,
19 Oct: three times a call made right after "run mode REAL" never returned; that arm stayed "busy", STOP was never
confirmed, new connections hung, and only killing Lite cleared it. Lite now sees which call has not answered and for
how long, writes every thread's stack to the day's log folder, ends the job on that arm, gives the connection up (the
stuck call left behind, its handle never deleted) and connects the arm again on a new handle; a STOP pressed meanwhile
goes through the new connection; Connect is never refused as "busy" by the stuck call; the arm is held until Recover."""
import os
import threading
import time
from contextlib import contextmanager

from alix_dispatch import connect

from alix_studio_lite.server import robot_link, robot_standin
from alix_studio_lite.server.jobs import Jobs
from alix_studio_lite.server.logbook import Logbook


class Controller:
    """Enough of the SDK for a reading, and one call that never answers on the first controller (``stuck``); anything
    else answers -1."""

    def __init__(self, stuck: bool = False):
        self.block, self.calls = threading.Event(), []
        if not stuck:
            self.block.set()

    def rm_get_current_arm_state(self):
        return 0, {"joint": [0.0] * 7, "pose": [0.0] * 6, "err": ["0"]}

    def rm_get_arm_run_mode(self):
        return 0, 1

    def rm_get_lift_state(self):
        return 0, {"pos": 50}

    def rm_set_modbus_mode(self, *a):
        return 0

    def rm_read_multiple_holding_registers(self, *a):
        return -1, []

    def rm_get_io_state(self, n):
        return 0, {"io_state": 1, "io_config": {"io_mode": 5}}

    def rm_set_plan_speed(self, pct):                       # the call that never comes back
        self.calls.append("rm_set_plan_speed")
        self.block.wait()
        return 0

    def rm_delete_robot_arm(self):
        return 0

    def __getattr__(self, name):
        if name.startswith("rm_"):
            return lambda *a, **kw: (-1, None)
        raise AttributeError(name)


@contextmanager
def lite(tmp_path, monkeypatch, watch_s):
    """A robot linked to fake controllers (a new one per connection), its thresholds short; every blocked call is
    released at the end so no thread outlives the test."""
    monkeypatch.setattr(robot_link, "CALL_SLOW_S", 0.1)
    monkeypatch.setattr(robot_link, "CALL_STUCK_S", 0.4)
    monkeypatch.setattr(robot_standin, "WATCH_PERIOD_S", watch_s)
    monkeypatch.setattr(robot_standin, "LINK_PROCESS", False)             # the link in this process (robot_process has its own test)
    monkeypatch.setattr(robot_link.ArmLink, "_stop_now",
                        lambda self, hard, drag=False, h=None: dict(ok=True, text=f"{self.side.upper()}: stopped on its controller",
                                                                     through="reader" if h is not None and h is self.reader else "cmd"))
    made, conns = [], []                                      # the commands' controllers (one per link); every connection
    per_link = 1 if os.environ.get("LITE_LINK_READER") == "0" else 2     # a link opens the readings' connection first (26 Oct)

    def open_arm(side, ip=None, port=None, **kw):
        commands = len(conns) % per_link == per_link - 1          # the last of a link's connections is the commands'
        c = Controller(stuck=commands and not made)               # the first link's commands' controller is the one that hangs
        conns.append(c)
        if commands:
            made.append(c)
        return connect.ArmHandle(side, ip or "192.168.1.103", port or 8080, c, len(conns), "127.0.0.1")

    monkeypatch.setattr(connect, "open_arm", open_arm)
    monkeypatch.setattr(connect, "close_arm", lambda h: None)
    log, jobs = Logbook(tmp_path / "logs"), Jobs()
    r = robot_standin.StandInRobot(tmp_path, jobs, link="realman", log=log)
    try:
        yield r, jobs, log, made
    finally:
        for c in conns:
            c.block.set()
        time.sleep(0.3)


def stuck_move(r, jobs):
    """The lab's MoveJ to a global point: a plain call that never answers, inside the command's hold on the link."""
    first = r.links["right"]

    def work(job):
        with first.busy("MoveJ to R_HME_POS", expect_s=120):
            first.handle.robot.rm_set_plan_speed(100)

    return first, jobs.start("move", "RIGHT MoveJ to R_HME_POS", work, arm="right")


def recovered(r, first, limit_s=5.0):
    t_end = time.time() + limit_s
    while time.time() < t_end and (r.links.get("right") in (None, first) or "right" in r._recovering):
        time.sleep(0.05)
    return r.links.get("right")


def test_a_call_that_never_comes_back_is_seen_given_up_and_the_arm_connected_again(tmp_path, monkeypatch):
    with lite(tmp_path, monkeypatch, watch_s=0.1) as (r, jobs, log, made):
        r.connect("right")
        assert type(made[0]).__module__ == __name__ and type(made[0]).__name__ == "Controller"   # ALIX's type checks hold
        first, job = stuck_move(r, jobs)
        time.sleep(0.25)
        v = r.view()["right"]
        assert v["quality"]["level"] == "weak" and "rm_set_plan_speed has not answered" in v["quality"]["text"]
        assert v["link"]["feedback"]["call"]["name"] == "rm_set_plan_speed"
        new = recovered(r, first)
        assert new is not None and new is not first and r.arms["right"].connected and len(made) == 2
        assert first.abandoned and not made[0].block.is_set()    # the stuck call is left behind, never deleted under it
        assert job.state == "failed" and "stopped answering (rm_set_plan_speed" in job.error
        assert r.arms["right"].estop == "RECOVERING" and r.arms["right"].stop_latched       # held until Recover
        said = [a["text"] for a in log.alerts_view()]
        assert any("has not answered rm_set_plan_speed for" in s and "What each thread was doing: " in s for s in said)
        assert any("connected again" in s for s in said)
        dumps = list((tmp_path / "logs").glob("*/threads-*.txt"))
        assert dumps and "rm_set_plan_speed" in dumps[0].read_text()
        time.sleep(0.5)
        assert r.view()["right"]["quality"]["level"] == "good"
        made[0].block.set()                                      # the lost thread ends; the job's ending stays as given up
        time.sleep(0.3)
        assert job.state == "failed" and any("came back" in e["text"] for e in job.events)


def test_stop_pressed_while_a_call_is_stuck_goes_through_the_readings_connection(tmp_path, monkeypatch):
    """27 Oct night: a STOP went through the commands' connection alone, so a call stuck there held it until the new
    connection (20 s). The link's readings' connection (26 Oct) is never inside a command's call: the STOP goes through it
    at once; the stuck connection is the watch's business."""
    with lite(tmp_path, monkeypatch, watch_s=30.0) as (r, jobs, log, made):         # the watch inert: STOP alone acts
        r.connect("right")
        first, job = stuck_move(r, jobs)
        time.sleep(0.6)                                          # past CALL_STUCK_S
        q = r.view()["right"]["quality"]
        assert q["level"] == "none" and q["text"].startswith("stuck: rm_set_plan_speed has not answered")
        assert first.reader is not None and first.stuck("cmd") and first.stuck("reader") is None
        out = r.stop("right")
        c = out["controllers"]["right"]
        assert out["ok"] and c["ok"] and c["through"] == "reader" and "stopped on its controller" in c["text"]
        assert r.links.get("right") is first and "right" not in r._stop_wanted   # nothing waits for a new connection
        assert first.stuck("reader") is None                      # the STOP's own calls came back


def test_stop_pressed_while_a_call_is_stuck_goes_through_the_new_connection_without_a_readings_connection(tmp_path, monkeypatch):
    """Without the readings' connection (LITE_LINK_READER=0) the stuck commands' connection is the only one: the STOP
    never goes through it; the link is given up at once and the STOP goes through the new connection."""
    monkeypatch.setenv("LITE_LINK_READER", "0")
    with lite(tmp_path, monkeypatch, watch_s=30.0) as (r, jobs, log, made):         # the watch inert: STOP finds it
        r.connect("right")
        first, job = stuck_move(r, jobs)
        time.sleep(0.6)                                          # past CALL_STUCK_S
        assert first.reader is None
        out = r.stop("right")                                    # never through the stuck handle
        assert not out["ok"] and "is stuck (rm_set_plan_speed" in out["controllers"]["right"]["text"]
        assert "right" in r._recovering or r.links.get("right") is not first
        new = recovered(r, first)
        assert new is not None and new is not first
        said = [a["text"] for a in log.alerts_view()]
        assert any("STOP sent through the new connection: RIGHT: stopped on its controller" in s for s in said)
        assert job.state == "failed"


def test_a_stops_own_call_that_never_comes_back_is_given_up_in_seconds_and_the_stop_sent_again(tmp_path, monkeypatch):
    """26 Oct 10:09: rm_set_arm_stop was not answered for 20 s before the connection was given up. A STOP's own calls are
    bounded by STOP_STUCK_S: the watch gives the link up, and the new connection sends the STOP again."""
    monkeypatch.setattr(robot_link, "STOP_STUCK_S", 0.3)
    monkeypatch.setattr(robot_standin, "STOP_WAIT_S", 0.5)
    block, stops = threading.Event(), []

    def stop_now(self, hard, drag=False, h=None):
        stops.append(self)
        if len(stops) == 1:                                      # the first STOP's own call never comes back
            self._stopping.set()
            self._enter("rm_set_arm_stop", h.robot)
            block.wait()
            self._leave()
            self._stopping.clear()
            return dict(ok=False, text="RIGHT: STOP not confirmed")
        return dict(ok=True, text=f"{self.side.upper()}: stopped on its controller")

    try:
        with lite(tmp_path, monkeypatch, watch_s=0.2) as (r, jobs, log, made):
            monkeypatch.setattr(robot_link.ArmLink, "_stop_now", stop_now)
            r.connect("right")
            first = r.links["right"]
            out = r.stop("right")
            assert not out["ok"] and "not confirmed by its controller within 0.5 s" in out["controllers"]["right"]["text"]
            new = recovered(r, first)                            # the watch: the STOP's call stuck after 0.3 s, the link given up
            assert new is not None and new is not first and first.abandoned
            said = [a["text"] for a in log.alerts_view()]
            assert any("has not answered rm_set_arm_stop for" in s for s in said)
            assert any("STOP sent through the new connection: RIGHT: stopped on its controller" in s for s in said)
            assert len(stops) == 2 and stops[1] is new
    finally:
        block.set()


def test_connect_again_is_not_refused_by_the_stuck_call(tmp_path, monkeypatch):
    with lite(tmp_path, monkeypatch, watch_s=30.0) as (r, jobs, log, made):
        r.connect("right")
        first, job = stuck_move(r, jobs)
        time.sleep(0.6)
        view = r.connect("right")                                # on 19 Oct this read "busy: RIGHT MoveJ to R_HME_POS"
        assert view["connected"] and r.links["right"] is not first and first.abandoned and len(made) == 2
        assert job.state == "failed" and "right" not in r._recovering
        assert r.disconnect("right")["connected"] is False


def test_the_long_calls_are_judged_by_the_command_s_own_expectation(monkeypatch):
    """A blocking MoveJ lasts as long as the move; a file sent as long as the transfer: not stuck at CALL_STUCK_S."""
    monkeypatch.setattr(robot_link, "CALL_STUCK_S", 0.05)
    k = robot_link.ArmLink("right", "192.168.1.103", period_s=10.0)
    with k.busy("MoveJ to R_HME_POS", expect_s=120):
        k._enter("rm_movej")
        time.sleep(0.1)
        assert k.stuck() is None and k.in_flight()[0] == "rm_movej"
        k._leave()
        k._enter("rm_get_current_tool_frame")
        time.sleep(0.1)
        assert k.stuck()[0] == "rm_get_current_tool_frame"
        k._leave()
    assert k.stuck() is None and k.in_flight() is None
