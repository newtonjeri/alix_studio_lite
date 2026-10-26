"""The delays taken out on 26 Oct (the owner, after the 15:07 run's analysis): a MoveJ to a pose the arm already stands
at is not sent; a hand already at its counts is not sent; a plan's pole and hand are set together, to the state its
first move needs; Lite's pole step is done when the pole reads idle within 5 mm after its ramp, not after the control
program's 1.5 s creep allowance; the S2 feed recording is off unless ticked."""
import contextlib
from types import SimpleNamespace

import pytest

from alix_studio_lite import vendor_paths
from alix_studio_lite.server import mechanism, robot_real, staging, workflow

vendor_paths.activate()


def _arm(joints_deg, hw):
    arm = robot_real.RealArm.__new__(robot_real.RealArm)
    arm.side, arm.SIDE = "left", "LEFT"
    robot = SimpleNamespace(rm_get_current_arm_state=lambda: (0, {"joint": list(joints_deg)}))
    arm.link = SimpleNamespace(busy=lambda *a, **k: contextlib.nullcontext(), emulated=True, handle=SimpleNamespace(robot=robot))
    arm._ratio, arm._hand, arm._hand_speeds = None, None, None
    arm._hw = lambda verb, **kw: hw.append((verb, kw))
    arm._alix_log = lambda: (lambda *a: None)
    return arm


def test_a_movej_to_a_pose_the_arm_already_stands_at_is_not_sent(monkeypatch):
    """15:07 run: 12 MoveJs of 0.19-0.41 deg, 0.9-1.4 s each (the controller's MoveJ has a floor of about a second)."""
    from alix_dispatch import staging as alix_staging

    hw, parked = [], []
    monkeypatch.setattr(alix_staging, "park_arm", lambda *a, **k: parked.append(k) or dict(ok=True))
    rest = [0.0, -82.0, -26.0, 98.0, 6.0, 62.0, 69.0]
    arm = _arm([0.0, -81.99, -26.0, 97.99, 6.0, 61.68, 69.0], hw)                 # 0.32 deg away, as on 26 Oct 15:08:36
    out = arm.move_to([round(v * 1000) for v in rest], speed=50, restore_pct=50)
    assert out["skipped"] and out["gap_deg"] == pytest.approx(0.32, abs=0.01) and not parked
    assert hw[-1][0] == "move to pose" and "already there: 0.32 deg away" in hw[-1][1]["text"]
    arm = _arm([0.0, -81.99, -26.0, 97.99, 6.0, 61.0, 69.0], hw)                  # 1.0 deg: sent
    arm.move_to([round(v * 1000) for v in rest], speed=50, restore_pct=50)
    assert parked and parked[0]["rest_speed"] == 50
    assert staging.MOVEJ_SKIP_DEG == 0.5


def test_a_hand_already_at_its_counts_is_not_sent(monkeypatch):
    from alix_dispatch import peripherals

    sent, hw = [], []

    class FakeHand:
        poll_s, speed_set = 0.01, 500

        def __init__(self, robot, side, log=None):
            pass

        def open(self, speed_set=500):
            return 0

        def read(self):
            return [1000, 993, 996, 994, 1000, 990]

        def set_speed(self, v):
            return 0

        def command(self, counts, wait=True):
            sent.append(list(counts))
            return dict(ok=True, arrived=True, after=list(counts), ret=0, t_s=0.3)

    monkeypatch.setattr(peripherals, "HandModbus", FakeHand)
    arm = _arm([0.0] * 7, hw)
    out = arm.hand_to([1000, 1000, 1000, 1000, 1000, 1000])                        # the plan's "open": within 20 counts
    assert out["skipped"] and not sent and "nothing sent" in hw[-1][1]["text"]
    arm = _arm([0.0] * 7, hw)
    out = arm.hand_to([33, 33, 33, 33, 133, 944])                                    # the grasp: sent
    assert sent == [[33, 33, 33, 33, 133, 944]] and not out.get("skipped")


class _Clock:
    """A clock the pole test turns by hand: time.monotonic() and time.sleep() as robot_real sees them."""

    def __init__(self):
        self.t = 100.0

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        self.t += max(float(dt), 0.01)


def _pole(monkeypatch, reads_target_after_s, idle_after_s):
    clock = _Clock()
    monkeypatch.setattr(robot_real, "time", clock)
    t0 = {"v": None}

    def set_height(speed, mm, block):
        t0["v"] = clock.t
        return 0

    def lift_state():
        e = 0.0 if t0["v"] is None else clock.t - t0["v"]
        return 0, {"pos": 199.0 if e >= reads_target_after_s else 150.0, "mode": 0 if e >= idle_after_s else 1, "err_flag": 0}

    robot = SimpleNamespace(rm_set_lift_height=set_height, rm_get_lift_state=lift_state, rm_set_lift_speed=lambda v: 0)
    Pole = robot_real._runner().load_aux_classes()[2]
    pole = robot_real._quick_pole(Pole)(robot)
    pole.poll_s = 0.01
    return pole, clock, t0


def test_the_pole_step_is_done_idle_within_5_mm_after_its_ramp(monkeypatch):
    """26 Oct captures: 49 mm done in 1.3 s, the step held 2.54 s by the model's 1.5 s creep; now 0.75 s + d / 0.097.
    The getter leads a moving pole, so a target read early still waits the slew floor; a slow pole is waited for."""
    pole, clock, t0 = _pole(monkeypatch, reads_target_after_s=0.05, idle_after_s=0.2)    # the getter says 199 at once
    ok, final = pole.move(0.199, speed_pct=100)
    assert ok and final == pytest.approx(0.199)
    assert clock.t - t0["v"] == pytest.approx(0.75 + 0.049 / 0.097, abs=0.03)             # the slew floor, 1.26 s (was 2.0)
    pole, clock, t0 = _pole(monkeypatch, reads_target_after_s=2.0, idle_after_s=2.0)     # a slow pole: waited for
    assert pole.move(0.199, speed_pct=100)[0] and clock.t - t0["v"] == pytest.approx(2.0, abs=0.03)
    pole, clock, t0 = _pole(monkeypatch, reads_target_after_s=1e9, idle_after_s=0.0)      # the pole at 150 already
    assert pole.move(0.150, speed_pct=100) == (True, 0.150) and t0["v"] is None             # already there: nothing sent
    assert staging.pole_quick_s(150, 199) == pytest.approx(0.75 + 0.049 / 0.097) and staging.pole_quick_s(150, 153) == 0.0
    assert staging.pole_travel_s(150, 199) == pytest.approx(1.5 + 0.049 / 0.097)           # runner v14's own POLE steps keep it


def test_a_plans_pole_and_hand_are_set_together_to_the_state_its_first_move_needs():
    """lid_open: the right opens, sets the pole and grasps before its move_to_inspect; the left opens, sets the pole and
    half-grasps before its lift. One pole+hand step per arm to those states; the plan's entries then find them met."""
    first = mechanism.Plan("lid_open").first_move_targets()
    assert first["right"]["pole_mm"] == pytest.approx(150.1, abs=0.1) and first["right"]["hand"] == [33, 33, 33, 33, 133, 944]
    assert first["left"]["pole_mm"] == pytest.approx(150.0, abs=0.1) and first["left"]["hand"] == [516, 516, 516, 516, 133, 944]
    assert first["right"]["arm_deg"] == mechanism.Plan("lid_open").targets("start")["right"]["arm_deg"]
    calls = []

    class Arm:
        def __init__(self, side):
            self.side = side

        def drop_hand(self):
            calls.append((self.side, "drop_hand"))

        def pole_and_hand(self, mm, hand, stop_hook=None):
            calls.append((self.side, "pole_and_hand", round(mm, 1), list(hand)))
            return dict(pole_ok=True, hand_ok=True, pole_mm=mm, hand=list(hand))

        def move_to(self, target, speed, restore_pct):
            calls.append((self.side, "move_to", speed))
            return dict(gap_deg=0.1)

    plan = mechanism.Plan("lid_open")
    fake = SimpleNamespace(robot=SimpleNamespace(speed_pct=50, _measured=lambda side: (150.0, [1000] * 6), found_speed=lambda side: 50),
                           job=SimpleNamespace(progress=lambda *a, **k: None, on_cancel=None), check_stop=lambda: None,
                           say=lambda *a, **k: None, _arm=lambda side: Arm(side))
    mechanism.RealExec.approach(fake, plan, plan.targets("start"), "start")
    steps = [c for c in calls if c[1] == "pole_and_hand"]
    assert sorted(steps) == [("left", "pole_and_hand", 150.0, [516, 516, 516, 516, 133, 944]),
                             ("right", "pole_and_hand", 150.1, [33, 33, 33, 33, 133, 944])]
    assert [c for c in calls if c[1] == "move_to"] == [("right", "move_to", 50), ("left", "move_to", 50)]   # the camera arm first
    assert all(("left", "drop_hand") in calls for _ in [0]) and ("right", "drop_hand") in calls


def test_the_s2_feed_recording_is_off_unless_ticked():
    app = SimpleNamespace(feed_ms=20)
    assert workflow.feed_wanted(app, {}) is False and workflow.feed_wanted(app, {"record_feed": False}) is False
    assert workflow.feed_wanted(app, {"record_feed": True}) is True
    assert workflow.feed_wanted(SimpleNamespace(feed_ms=None), {"record_feed": True}) is False


class _Tap:
    """A feed as _stream_end reads it: a frame every 5 ms of the fake clock, at ``at`` (millidegrees) with ``speed``."""

    def __init__(self, clock, at, speed_millideg_s=0):
        self.clock, self.at, self.speed, self._n = clock, list(at), speed_millideg_s, 0

    @property
    def t_mono(self):
        self._n += 1
        return self.clock.t + 0.005 * self._n

    @property
    def frame(self):
        return {"joint_status": {"joint_position": list(self.at), "joint_speed": [self.speed] * 7}}


def test_a_stream_ends_on_the_feeds_frames_at_its_last_line(monkeypatch):
    """26 Oct captures, 72 streams: the arm within 0.08 deg of its end by the last line and under 1.6 deg/s after it, every
    time; ALIX's settle (six polls 25 ms apart, 0.24 s a stream) never had to wait. Now three frames at the last line and
    still end the stream (15 ms); the polled wait takes over only when the feed does not show it within a second."""
    from alix_dispatch import track

    clock = _Clock()
    monkeypatch.setattr(robot_real, "time", clock)
    polled = []
    monkeypatch.setattr(track, "wait_until_at_rest", lambda robot, **kw: polled.append(robot) or dict(waited_s=0.17, at_rest=True, peak_deg_s=0.3))
    last = [0, -82000, -26000, 98000, 6000, 62000, 69000]
    arm = _arm([0.0] * 7, [])
    arm._tap = _Tap(clock, [0, -81990, -26005, 97992, 6003, 61995, 69003])       # 0.01 deg off, speed 0: at rest at once
    t0 = clock.t
    out = arm._stream_end(last, t_after=t0)
    assert out["by"] == "feed" and out["at_rest"] and out["gap_deg"] == 0.01 and not polled
    assert clock.t - t0 < 0.05 and out["waited_s"] < 0.05                         # three frames, not 0.24 s
    arm._tap = _Tap(clock, [0, -81990, -26005, 97992, 6003, 61995, 69003], speed_millideg_s=3000)   # 3 deg/s: still moving
    t0 = clock.t
    out = arm._stream_end(last, t_after=t0)
    assert out["by"] == "polled" and out["at_rest"] and polled == [arm.robot] and clock.t - t0 == pytest.approx(1.0, abs=0.05)
    arm._tap = _Tap(clock, [0, -81000, -26000, 98000, 6000, 62000, 69000])         # 1 deg from the last line: not there
    assert arm._stream_end(last, t_after=clock.t)["by"] == "polled" and len(polled) == 2
    arm._tap = robot_real._FrameTap()                                              # no frame at all (the feed quiet): polled
    assert arm._stream_end(last, t_after=clock.t)["by"] == "polled" and len(polled) == 3
    assert (robot_real.END_TOL_DEG, robot_real.END_STILL_DEG_S, robot_real.END_FRAMES, robot_real.END_WAIT_S) == (0.5, 1.6, 3, 1.0)


def test_a_move_the_arm_has_crept_from_is_bridged_at_40_percent_up_to_3_degrees():
    """26 Oct 17:14: the left wrist, at its 9 A plateau holding the seat up for the 9 s the right arm needs, crept 1.0 deg
    from the retract's first line, and ALIX refused the stream (one 5 ms line's step, 0.72 deg at Lite's gate). The owner:
    the left arm lifts and cannot let go before the right has moved, so the move is bridged first, at 40 % (a short move);
    beyond 3 deg something else is wrong and the stream is refused as before, the gap said."""
    from types import SimpleNamespace

    plan = mechanism.Plan("seat_open")
    retract = next(e for e in plan.entries if e.kind == "arm" and e.side == "left" and e.stage == "move_to_retract")
    first = [v / 1000.0 for v in retract.rows[0]]
    streams, said = [], []

    class Arm:
        def stream(self, rows, label, speed, restore_pct, bridge_speed, stop_hook, records):
            streams.append(bridge_speed)
            return dict(lines=len(rows), lines_sent=len(rows), seconds=0.8, bridged_deg=None)

    def fake(joints):
        link = SimpleNamespace(read_fresh=lambda: {"joints_deg": joints})
        return SimpleNamespace(_arm=lambda side: Arm(), job=SimpleNamespace(progress=lambda *a, **k: None, on_cancel=None),
                               live=lambda *a, **k: None, say=lambda text, level="info": said.append((level, text)),
                               _keep_feed=lambda side: None, _keep_hand=lambda side: None,
                               robot=SimpleNamespace(links={"left": link}, speed_pct=15, found_speed=lambda side: 50),
                               kit=SimpleNamespace(records_dir=lambda what: None))

    def note():                                                            # what the run said about the first line
        return [x for x in said if "first line" in x[1]][-1]

    crept = list(first); crept[5] += 1.01                                  # J6, as at 17:14
    mechanism.RealExec.move(fake(crept), retract)
    assert streams == [40] and "1.01 degrees from its first line" in note()[1] and "40 %" in note()[1]
    near = list(first); near[5] += 0.5                                     # within one line's step: nothing added
    mechanism.RealExec.move(fake(near), retract)
    assert streams == [40, None]
    far = list(first); far[5] += 4.0                                       # beyond 3 degrees: refused at its first line, said
    mechanism.RealExec.move(fake(far), retract)
    assert streams == [40, None, None] and note()[0] == "warn" and "4.0 degrees from its first line, more than the 3" in note()[1]
    mechanism.RealExec.move(fake(crept), retract, bridge=True)             # a re-lift's own bridge keeps the slider's speed
    assert streams[-1] == 15
    assert (mechanism.BRIDGE_MAX_DEG, mechanism.BRIDGE_SPEED_PCT) == (3.0, 40) and mechanism.start_tol_deg() == pytest.approx(0.72, abs=0.01)
