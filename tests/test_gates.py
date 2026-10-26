"""The gates of 26 Oct (the owner: "what will make the arm stop the motion execution? ... informative indicators of
something that went wrong, instead of preventing it going wrong"): a hand that did not let go stops the run before the
next move; a wrist still loaded stops it before a retract; a stream stops on a wrist load over FORCE_CUT_N; no run starts
with a hot joint; the joints' temperatures and currents are read with every status reading."""
import threading
from types import SimpleNamespace

import pytest

from alix_studio_lite import vendor_paths
from alix_studio_lite.server import mechanism, robot_link, robot_real
from alix_studio_lite.server.app import LiteApp
from alix_studio_lite.server.jobs import UserError

vendor_paths.activate()


def _frame(force_n, zero_n=(0.0, 0.0, 0.0), joint_position=None):
    return {"six_force_sensor": {"force": [int(v * 1000) for v in force_n] + [0, 0, 0],
                                 "zero_force": [int(v * 1000) for v in zero_n] + [0, 0, 0]},
            "joint_status": {"joint_position": joint_position or [0] * 7, "joint_err_code": [0] * 7}, "err": [0]}


def test_the_wrist_load_is_the_force_less_the_sensors_zero():
    assert robot_real.wrist_load_n(_frame((3.0, 4.0, 0.0))) == pytest.approx(5.0)
    assert robot_real.wrist_load_n(_frame((-72.3, -28.2, 38.6), (-13.26, -1.9, 4.05))) == pytest.approx(73.3, abs=0.1)   # 26 Oct, the seat on the hand
    assert robot_real.wrist_load_n({"joint_status": {}}) is None                                # an emulated arm: no sensor


def test_a_stream_stops_when_the_wrist_carries_more_than_the_limit_for_50_ms():
    seen = []
    rows = [[0] * 7 for _ in range(50)]
    watch = robot_real._StopWatch(rows, 0.005, threading.Event(), tap=lambda t, f: seen.append(t))
    t = 100.0
    for k in range(robot_real.FORCE_CUT_FRAMES - 1):                     # nine frames over: a spike, not a load
        watch.on_frame(t + k * 0.005, _frame((130.0, 0.0, 0.0)))
    assert watch.check(t + 1, 10) is None
    watch.on_frame(t + 0.05, _frame((80.0, 0.0, 0.0)))                  # under again: the count starts over
    for k in range(robot_real.FORCE_CUT_FRAMES):
        watch.on_frame(t + 0.1 + k * 0.005, _frame((125.0, 10.0, 0.0)))
    trip = watch.check(t + 2, 20)
    assert trip and trip["why"] == "force" and "125 N on the wrist for 50 ms" in trip["detail"]
    assert len(seen) == 2 * robot_real.FORCE_CUT_FRAMES                  # the tap saw every frame
    assert robot_real._StopWatch(rows, 0.005, threading.Event()).check(t, 0) is None


def test_hand_load_reads_the_kept_feeds_latest_frame():
    arm = robot_real.RealArm.__new__(robot_real.RealArm)
    arm._tap = robot_real._FrameTap()
    assert arm.hand_load() is None
    arm._tap.on_frame(1.0, _frame((30.0, 40.0, 0.0)))
    load = arm.hand_load()
    assert load["load_n"] == 50.0 and load["age_s"] >= 0


def _released(after, load_n, next_stage="move_to_retract"):
    plan = mechanism.Plan("seat_open")
    e = next(x for x in plan.entries if x.kind == "hand" and x.side == "left" and x.stage == "hand_release")
    said = []
    fake = SimpleNamespace(_hand_results={"left": {"after": after}},
                           _arm=lambda side: SimpleNamespace(hand_load=lambda: None if load_n is None else dict(load_n=load_n, age_s=0.1)),
                           say=lambda text, *a: said.append(text))
    mechanism.RealExec.released(fake, e, next_stage)
    return e, said


def test_a_hand_that_did_not_let_go_stops_the_run_before_the_next_move():
    """26 Oct 10:54:45: the left release read [1000, 544, 995, 993, 980, 975] against [993, 993, 993, 993, 981, 992]:
    the ring finger (channel 2) stayed under the seat, and the retract was next."""
    with pytest.raises(UserError, match=r"LEFT hand did not let go: channel 2 \(ring\) 449 counts short of 993") as e:
        _released([1000, 544, 995, 993, 980, 975], 3.0)
    assert e.value.code == "escalate"
    e, said = _released([1000, 1000, 1000, 1000, 970, 977], 3.0)         # the thumb's usual few counts: let go
    assert [int(v) for v in e.hand_counts()] == [993, 993, 993, 993, 981, 992] and said == ["LEFT hand free: 3 N on the wrist"]


def test_a_wrist_still_loaded_after_the_release_stops_the_run_before_a_retract_or_rest():
    with pytest.raises(UserError, match="LEFT hand is still loaded after it let go: 56 N on its wrist") as e:
        _released([1000, 1000, 1000, 1000, 970, 977], 56.0)
    assert e.value.code == "escalate" and "move_to_retract was not started" in str(e.value)
    with pytest.raises(UserError, match="still loaded"):
        _released([1000, 1000, 1000, 1000, 970, 977], 56.0, "move_to_rest")
    _, said = _released([1000, 1000, 1000, 1000, 970, 977], 56.0, "execute_path")   # the right's push carries the part: no gate
    assert said == []
    _, said = _released([1000, 1000, 1000, 1000, 970, 977], None)                  # no feed reading (an emulated arm): said, not stopped
    assert said == ["LEFT hand free: no wrist reading"]


def test_the_runner_asks_the_release_gate_after_each_hand_release():
    import importlib.util
    from pathlib import Path

    spec = importlib.util.spec_from_file_location("lite_test_mechanism", Path(__file__).with_name("test_mechanism.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    FakeExec = module.FakeExec

    class Exec(FakeExec):
        def released(self, e, next_stage):
            self.calls.append(("released", e.side, e.stage, next_stage))

    ex = Exec(["lid_open", "seat_closed", "seat_lifted", "seat_open"])
    mechanism.Runner(ex).run("seat_open")
    assert [c[1:] for c in ex.calls if c[0] == "released"] == [("left", "hand_release", "move_to_pre_start"),       # before the lift: no retract next
                                                               ("right", "hand_release", "execute_path"),
                                                               ("left", "hand_release", "move_to_retract"),
                                                               ("left", "hand_release", None),                      # its last: nothing after
                                                               ("right", "hand_release", None)]
    ex = FakeExec(["lid_open", "seat_closed", "seat_lifted", "seat_open"])        # an executor without the gate runs as before
    assert mechanism.Runner(ex).run("seat_open")["plans"][0]["outcome"] == "done"


def _robot(temps):
    link = SimpleNamespace(view=lambda: {"feedback": {"temps_c": temps}})
    return SimpleNamespace(links={"right": link, "left": SimpleNamespace(view=lambda: {"feedback": {"temps_c": None}})})


def test_no_run_starts_with_a_joint_at_65_degrees_or_more():
    with pytest.raises(UserError, match=r"Not run: the RIGHT arm's joint 3 at 72 °C \(a run needs every joint under 65 °C\)") as e:
        mechanism.joints_cool(_robot([38.0, 44.0, 72.0, 47.0, 43.0, 39.0, 39.0]))
    assert e.value.code == "hot"
    with pytest.raises(UserError, match="Open the lid STOPPED: the RIGHT arm's joint 3 at 65 °C"):
        mechanism.joints_cool(_robot([38.0, 44.0, 65.0, 47.0, 43.0, 39.0, 39.0]), "Open the lid")
    assert mechanism.joints_cool(_robot([38.0, 44.0, 54.0, 47.0, 43.0, 39.0, 39.0])) == {"right": [38.0, 44.0, 54.0, 47.0, 43.0, 39.0, 39.0], "left": None}
    assert mechanism.joints_cool(SimpleNamespace(links={})) == {}


def test_each_status_reading_carries_the_joints_temperatures_and_currents():
    robot = SimpleNamespace(rm_get_current_joint_temperature=lambda: (0, [38.0, 44.2, 54.1, 47.0, 43.0, 39.0, 39.0]),
                            rm_get_current_joint_current=lambda: (0, [191.0, -1014.0, 6283.0, -2740.0, 235.0, -411.0, -63.0]))
    assert robot_link._joint_floats(robot, "rm_get_current_joint_temperature", 1.0, 1) == [38.0, 44.2, 54.1, 47.0, 43.0, 39.0, 39.0]
    assert robot_link._joint_floats(robot, "rm_get_current_joint_current", 0.001, 2) == [0.19, -1.01, 6.28, -2.74, 0.24, -0.41, -0.06]
    assert robot_link._joint_floats(SimpleNamespace(), "rm_get_current_joint_temperature", 1.0, 1) is None    # ALIX's emulator
    assert robot_link._joint_floats(SimpleNamespace(rm_get_current_joint_current=lambda: (-2, [])), "rm_get_current_joint_current", 0.001, 2) is None


def test_the_controllers_own_voltage_current_and_temperature_are_read_as_the_vendors_gui_reads_them():
    """26 Oct: the vendor's GUI's Controller Info (22.4 V, 0.7 A, 47 C) and its per-joint voltages come from the plain
    reads over the connection, no realtime push; Lite's status readings carry the same (robot_link._read_all)."""
    from alix_studio_lite.server import robot_link

    robot = SimpleNamespace(rm_get_controller_state=lambda: dict(return_code=0, voltage=22.39, current=0.713, temperature=47.04, system_error=0),
                            rm_get_current_joint_voltage=lambda: (0, [22.0, 22.0, 22.0, 22.0, 22.0, 21.0, 21.0]))
    assert robot_link._controller_state(robot) == dict(voltage_v=22.4, current_a=0.71, temp_c=47.0, sys_err=0)
    assert robot_link._joint_floats(robot, "rm_get_current_joint_voltage", 1.0, 1) == [22.0, 22.0, 22.0, 22.0, 22.0, 21.0, 21.0]
    assert robot_link._controller_state(SimpleNamespace()) is None                                    # ALIX's emulator
    assert robot_link._controller_state(SimpleNamespace(rm_get_controller_state=lambda: dict(return_code=-2))) is None


def test_a_joint_at_60_degrees_is_one_alert_until_it_has_cooled():
    alerts = []
    fake = SimpleNamespace(log=SimpleNamespace(alert=lambda level, text, **kw: alerts.append((level, text, kw.get("side")))))
    LiteApp._hot_joint_alert(fake, "right", [38.0, 44.0, 54.0, 47.0, 43.0, 39.0, 39.0])
    assert alerts == []
    LiteApp._hot_joint_alert(fake, "right", [38.0, 44.0, 61.0, 47.0, 43.0, 39.0, 39.0])
    LiteApp._hot_joint_alert(fake, "right", [38.0, 44.0, 72.0, 47.0, 43.0, 39.0, 39.0])                 # still hot: said once
    assert alerts == [("warn", "RIGHT: joint 3 at 61 C", "right")]
    LiteApp._hot_joint_alert(fake, "right", [38.0, 44.0, 57.0, 47.0, 43.0, 39.0, 39.0])                 # not yet 5 under
    LiteApp._hot_joint_alert(fake, "right", [38.0, 44.0, 62.0, 47.0, 43.0, 39.0, 39.0])
    assert len(alerts) == 1
    LiteApp._hot_joint_alert(fake, "right", [38.0, 44.0, 54.0, 47.0, 43.0, 39.0, 39.0])                 # cooled: the latch opens
    LiteApp._hot_joint_alert(fake, "right", [38.0, 44.0, 66.0, 47.0, 43.0, 39.0, 39.0])
    assert len(alerts) == 2
