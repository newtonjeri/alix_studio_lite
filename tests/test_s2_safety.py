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
