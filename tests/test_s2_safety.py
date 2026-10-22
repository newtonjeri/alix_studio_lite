"""S2's safety paths found by the double check (16 Oct): a job never stays "running" after an exit; a STOP
reaches a hook added as it comes; the runner's own exit ends its part; a program that ends before its
points could have played is a stop (an E-stop raises no error code); a global point's wait becomes a Wait
step a STOP can cut; an E-stop read between two runner steps stops every motion job; a simulation with
nothing to watch is never "played"."""
import json
import threading
import time
from contextlib import contextmanager
from types import SimpleNamespace

import pytest


def test_a_job_never_stays_running_and_a_stop_reaches_late_hooks():
    from alix_studio_lite.server.jobs import Jobs

    jobs = Jobs()
    j = jobs.start("run", "exits", lambda job: (_ for _ in ()).throw(SystemExit("program list did not answer")))
    for _ in range(50):
        if j.state != "running":
            break
        time.sleep(0.02)
    assert j.state == "failed" and "SystemExit" in j.error

    hit = []
    j2 = jobs.start("run", "waits", lambda job: job.sleep(5))
    j2.on_cancel(lambda: hit.append("before"))
    j2.cancel()
    j2.on_cancel(lambda: hit.append("after"))                 # added once stopped: called at once
    assert hit == ["before", "after"]


class FakeLink:
    def __init__(self, robot=None):
        self.side, self.ip, self.handle = "left", "192.168.1.10", SimpleNamespace(robot=robot)

    @contextmanager
    def busy(self, what, wait_s=None, expect_s=None):
        yield


def test_the_runners_own_exit_ends_its_part(monkeypatch):
    from alix_studio_lite.server import robot_real

    class Runner:
        @staticmethod
        def normalize_workflow_step(st):
            return st

        @staticmethod
        def run_workflow_sequence(arm, a, ip, steps):
            raise SystemExit("the controller's program list could not be read (page 1, ret=-2)")

    monkeypatch.setattr(robot_real, "_runner", lambda: Runner)
    out = robot_real.RealArm(FakeLink()).run_workflow([{"type": "wait", "seconds": 1}], 50, 50)
    assert not out["ok"] and "program list could not be read" in out["why"]


def test_a_program_that_ends_early_was_stopped_and_waits_are_split():
    from alix_studio_lite.server.robot_standin import ended_early
    from alix_studio_lite.server.workflow import split_waits

    assert ended_early(3.0, 2001) == "after 3.0 s of the 10.0 s its points take"      # 2,001 lines: 10 s
    assert ended_early(10.4, 2001) is None and ended_early(0.2, 100) is None        # whole, or too short to judge
    assert ended_early(1.0, 2001, scale=10.0) is None                              # an emulator at 10x
    steps = [{"type": "global", "name": "HOME", "move": "movej", "speed": 20, "wait_s": 2.5, "_row": 3},
             {"type": "global", "name": "A", "move": "movel", "speed": 20, "wait_s": 0.0, "_row": 4}]
    assert split_waits(steps) == [dict(steps[0], wait_s=0.0), {"type": "wait", "seconds": 2.5, "_row": 3}, steps[1]]


def test_an_estop_read_between_two_steps_stops_every_motion_job(tmp_path):
    from alix_studio_lite.server.jobs import Jobs
    from alix_studio_lite.server.robot_standin import StandInRobot

    robot = StandInRobot(tmp_path, Jobs())
    robot.links["left"] = SimpleNamespace(emulated=False, estop_now=lambda: "ENGAGED", view=lambda: {"feedback": {}})
    robot.arms["left"].connected = True
    job = robot.jobs.start("run", "S2 Real Machine: both arms", lambda j: j.sleep(10), arm="both")
    assert robot.estop_check("left") is True
    for _ in range(50):
        if job.state != "running":
            break
        time.sleep(0.02)
    assert job.state == "failed" and "E-STOP" in job.error and robot.arms["left"].estop_latched
    robot.links["left"] = SimpleNamespace(emulated=False, estop_now=lambda: "CLEAR", view=lambda: {"feedback": {}})
    assert robot.estop_check("left") is False


def test_an_estop_read_by_a_runner_stops_both_arms_before_its_own_interrupt(tmp_path):
    """The check runs in a runner's thread, and that runner's stop hook interrupts its own thread (robot_real's
    stop): every other stop (the other arm's hook, the other arm's jobs) is still sent, then the runner stops."""
    from alix_studio_lite.server.jobs import Jobs
    from alix_studio_lite.server.robot_real import _interrupt
    from alix_studio_lite.server.robot_standin import StandInRobot

    robot = StandInRobot(tmp_path, Jobs())
    robot.links["left"] = SimpleNamespace(emulated=False, estop_now=lambda: "ENGAGED", view=lambda: {"feedback": {}})
    robot.arms["left"].connected = True
    hooks, out = [], {}
    run = robot.jobs.start("run", "S2 Real Machine: LEFT", lambda j: j.sleep(10), arm="left")
    other = robot.jobs.start("run", "Real Test: RIGHT", lambda j: j.sleep(10), arm="right")

    def runner():
        me = threading.get_ident()
        run.on_cancel(lambda: (hooks.append("left"), _interrupt(me)))      # this arm's stop: interrupts the runner
        run.on_cancel(lambda: hooks.append("left 2"))
        other.on_cancel(lambda: hooks.append("right"))
        try:
            robot.estop_check("left", on_press=lambda: out.setdefault("said", True))
            for _ in range(100):                                             # the interrupt lands at its next step
                time.sleep(0.01)
            out["ran on"] = True
        except KeyboardInterrupt:
            out["interrupted"] = True

    t = threading.Thread(target=runner)
    t.start()
    t.join(10)
    for _ in range(100):
        if run.state != "running" and other.state != "running":
            break
        time.sleep(0.02)
    assert out == {"said": True, "interrupted": True}
    assert sorted(hooks) == ["left", "left 2", "right"]
    assert run.state == other.state == "failed" and "E-STOP" in other.error


def test_a_simulation_with_nothing_to_watch_is_never_played(tmp_path):
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.server import workflow
    from alix_studio_lite.server.jobs import UserError
    from alix_studio_lite.server.lite_units import LiteUnits

    units = LiteUnits(tmp_path / "p", "commode_c")
    f = workflow._sim_file(units)
    f.parent.mkdir(parents=True, exist_ok=True)
    act = dict(kind="program", t0=0.0, t1=0.0, drawn=False, stage=0)
    f.write_text(json.dumps(dict(hash="h1", total_s=0.0, acts={"left": [act], "right": []})))
    with pytest.raises(UserError, match="Nothing in this run can be shown"):
        workflow.played(units, "h1")
    f.write_text(json.dumps(dict(hash="h2", total_s=3.0, acts={"left": [dict(act, kind="pole", t1=3.0, drawn=None)], "right": []})))
    assert workflow.played(units, "h2")["completed"]
    # a Global Point not drawn (its point not read) and a Wait: nothing moved, so never "played" (the owner's report)
    f.write_text(json.dumps(dict(hash="h3", total_s=1.0, acts={"left": [dict(act, kind="global", t1=0.0), dict(act, kind="wait", t1=1.0, drawn=None)],
                                                               "right": []})))
    with pytest.raises(UserError, match="Nothing in this run can be shown"):
        workflow.played(units, "h3")


def test_each_step_on_the_robot_is_timed_against_its_range(monkeypatch):
    """The owner, 22 Oct: a step outside its range is said; one far outside it (any step but a Program more than 10 %
    past the most it takes, or that much sooner than the least; a Program whose lines run past 105 % of their time)
    stops both arms, as the timings the simulation checked no longer hold."""
    from alix_studio_lite.server import workflow

    stopped, said = [], []
    steps = [{"type": "hand", "values": [0] * 6, "_row": 3, "_t": {"times": (0.05, 0.08, 0.1)}},
             {"type": "pole", "mm": 50.0, "_row": 4, "_t": {"times": (1.0, 1.1, 1.2)}},
             {"type": "wait", "seconds": 0.2, "_row": 5, "_t": {"times": (0.2, 0.205, 0.22)}},
             {"type": "program", "id": 7, "_row": 6, "_t": {"times": (1, 1, 1), "drawn": True, "play_s": 0.2, "lines": 41,
                                                           "transit": (0, 0, 0)}}]
    monkeypatch.setitem(workflow.ROBOT_TIMES, "load_s_per_line", (0.0, 0.0, 0.0))
    monkeypatch.setitem(workflow.ROBOT_TIMES, "program_extra_s", (0.0, 0.0, 0.05))
    on_line, close = workflow._watch(None, "left", steps, stopped.append, said.append)
    try:
        on_line("WORKFLOW STEP START 1/4: HAND")
        time.sleep(0.6)                                              # past 0.1 s x 1.1 (+ the lines' latency)
        assert stopped and "LEFT row 3 (Hand 0,0,0,0,0,0) ran past the most it takes (0.1 s) by 10%" in stopped[0]
        on_line("WORKFLOW STEP START 2/4: POLE")
        on_line("WORKFLOW STEP COMPLETE 2/4: POLE")                   # at once: much sooner than its least
        assert len(stopped) == 2 and "less than the least it takes (1.00 s)" in stopped[1]
        on_line("WORKFLOW STEP START 3/4: WAIT")
        time.sleep(0.3)
        on_line("WORKFLOW STEP COMPLETE 3/4: WAIT")                   # a little long: said, not stopped
        assert len(stopped) == 2 and said and "outside its range on the robot (0.20-0.22 s)" in said[0]
        on_line("WORKFLOW STEP START 4/4: PROGRAM")
        time.sleep(0.9)                                              # 0.2 s of lines x 1.05 + its allowances
        assert len(stopped) == 3 and "ran past 105% of its lines' time" in stopped[2]
    finally:
        close()


def test_a_real_run_is_refused_while_a_controllers_realtime_push_is_on(monkeypatch):
    """The programs' times on the robot were measured with the controllers' realtime push off (Lite turns it off as an
    arm connects): with it left on (another program holds its port, or it would not turn off) they run some 13 % slower,
    past the 105 % at which an S2 run stops. Such a run is refused before anything moves, saying why."""
    from alix_studio_lite.server import workflow
    from alix_studio_lite.server.robot_process import LinkProcess

    link = LinkProcess("right", "192.168.1.103")
    link._say_push({"port": 8090, "ip": "192.168.1.5", "holders": [{"pid": 4242, "command": "ros2 run alix_hw driver"}]}, reopened=False)
    assert link.push_on["port"] == 8090                       # left on: kept (no log needed)
    turned_off = LinkProcess("left", "192.168.1.10")
    turned_off._say_push({"port": 8089, "ip": "192.168.1.5", "cleared": True}, reopened=True)
    assert turned_off.push_on is None                          # turned off as it connected: nothing to refuse
    prog = lambda i: {"type": "program", "id": i, "name": "", "speed": 100, "transit_speed": 30}     # noqa: E731
    steps = {"left": workflow.check_steps([prog(1)], "left"), "right": workflow.check_steps([prog(2)], "right")}
    monkeypatch.setattr(workflow, "load", lambda units: steps)
    robot = SimpleNamespace(_real=lambda s: object(), arm=lambda s: SimpleNamespace(connected=True), require_motion=lambda m: None,
                            links={"left": turned_off, "right": link})
    with pytest.raises(workflow.UserError) as got:
        workflow.run_real(SimpleNamespace(robot=robot, ui={"mode": "real"}), None, "both", None, {})
    assert got.value.code == "push" and "The RIGHT controller's realtime push is on" in str(got.value)
    assert "pid 4242 (ros2 run alix_hw driver) holds" in str(got.value) and "Nothing moved" in str(got.value)
