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
    arm._ratio, arm._hand, arm._hand_speeds, arm._hand_last = None, None, None, None
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
    """lid_open (the owner's order since 29 Oct: the right's look and check first, then its pole and grasp, then the
    left's pole and half grasp before its lift): one pole+hand step per arm at the approach to the state its first move
    needs, the plan's entries then find them met. Only what comes before the run's first check (nothing but the look
    moves before it): in a sequence, which does not ask the start check, the left's; run on its own, nothing (both
    arms' come after the check); the right's grasp after its look runs where the plan has it, in both."""
    plan = mechanism.Plan("lid_open")
    start = plan.targets("start")
    in_sequence = mechanism.without_start_checks(list(plan.entries))[0]
    first = plan.first_move_targets(in_sequence)
    assert first["left"]["pole_mm"] == pytest.approx(150.1, abs=0.1) and first["left"]["hand"] == [516, 516, 516, 516, 133, 944]
    assert first["right"] == start["right"] and start["right"]["hand"] == [1000] * 6           # its look is its first entry
    assert plan.first_move_targets() == start                                                # on its own: all after the check
    assert mechanism.first_check_at(plan.entries) == 1 and plan.entries[0].stage == "move_to_inspect"
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

    fake = SimpleNamespace(robot=SimpleNamespace(speed_pct=50, _measured=lambda side: (150.0, [1000] * 6), found_speed=lambda side: 50),
                           job=SimpleNamespace(progress=lambda *a, **k: None, on_cancel=None), check_stop=lambda: None,
                           say=lambda *a, **k: None, _arm=lambda side: Arm(side), _settle_all=lambda: None)
    for entries, left_hand in ((in_sequence, [516, 516, 516, 516, 133, 944]), (None, start["left"]["hand"])):
        calls.clear()
        mechanism.RealExec.approach(fake, plan, start, "start", entries=entries)
        steps = [c for c in calls if c[1] == "pole_and_hand"]
        assert sorted(steps) == [("left", "pole_and_hand", round(start["left"]["pole_mm"], 1), left_hand),
                                 ("right", "pole_and_hand", round(start["right"]["pole_mm"], 1), [1000] * 6)]
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
        def stream(self, rows, label, speed, restore_pct, bridge_speed, stop_hook, records, settle=True):
            streams.append(bridge_speed)
            return dict(lines=len(rows), lines_sent=len(rows), seconds=0.8, bridged_deg=None)

    def fake(joints):
        link = SimpleNamespace(read_fresh=lambda: {"joints_deg": joints})
        return SimpleNamespace(_arm=lambda side: Arm(), job=SimpleNamespace(progress=lambda *a, **k: None, on_cancel=None),
                               live=lambda *a, **k: None, say=lambda text, level="info": said.append((level, text)),
                               _keep_feed=lambda side: None, _keep_hand=lambda side: None,
                               _joints=lambda side: link.read_fresh().get("joints_deg"),      # the feed's frame, else a reading (27 Oct)
                               _settle=lambda side: None, _unsettled={},
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


HAND_ARRIVE_COUNTS, HAND_SETTLE_COUNTS, HAND_DOF, HAND_DEADBAND_COUNTS = 15, 3, 6, 10     # ALIX's hand rules, for the stand-in below


class _HandBase:
    """ALIX's HandModbus as _quick_hand needs it: command(wait=False) writes and reports, read() follows a script on the
    fake clock, _sleep turns the clock."""
    poll_s = 0.05

    def __init__(self, clock, series, before, log):
        self.clock, self.series, self.before, self.log, self.t0 = clock, series, before, log, None

    def _sleep(self, dt):
        self.clock.sleep(dt)

    def read(self):
        if self.t0 is None:
            return list(self.before)
        e = self.clock.t - self.t0
        return list(next((v for t, v in reversed(self.series) if e >= t), self.before))

    def command(self, counts, wait=True, timeout_s=None):
        self.t0 = self.clock.t
        return dict(ok=True, ret=0, target=list(counts), before=list(self.before), after=None, settled=False, arrived=False,
                    t_s=0.0, law_s=0.9, timeout_s=3.7, series=[], moved=[0] * 6, stuck=[], miss=None, held=[], write=None, partial=False)


def test_a_hand_stopped_on_the_glove_is_done_before_the_strokes_time(monkeypatch):
    """26 Oct: a grasp the glove stops at about 450 counts within 0.3 s waited ALIX's whole 0.9 s stroke before two alike
    reads counted; now three reads in a row alike after the hand moved, from 0.25 s on, end it (the owner). A hand that
    arrives is arrived; one that never moves waits the stroke and is reported stuck, as ALIX reports it."""
    clock = _Clock()
    monkeypatch.setattr(robot_real, "time", clock)
    said = []
    Hand = robot_real._quick_hand(_HandBase)
    grasp = [33, 33, 33, 33, 133, 944]
    glove = [(0.0, [1000] * 6), (0.1, [800, 800, 800, 800, 600, 950]), (0.2, [470, 460, 475, 500, 180, 940]), (0.3, [470, 460, 475, 500, 180, 940])]
    h = Hand(clock, glove, [1000] * 6, said.append)
    out = h.command(grasp)
    assert out["ok"] and out["settled"] and out["stalled"] and not out["arrived"] and 0.3 <= out["t_s"] <= 0.45
    assert out["after"] == [470, 460, 475, 500, 180, 940] and "stopped on force, done before the stroke's 0.90 s" in said[0]
    said.clear()
    arrive = [(0.0, [1000] * 6), (0.1, [800] * 6), (0.2, [600] * 6), (0.3, [400] * 6), (0.4, [40, 35, 33, 33, 140, 944])]
    out = Hand(clock, arrive, [1000] * 6, said.append).command(grasp)
    assert out["ok"] and out["arrived"] and not out.get("stalled") and 0.4 <= out["t_s"] <= 0.5
    said.clear()
    dead = [(0.0, [1000] * 6)]
    out = Hand(clock, dead, [1000] * 6, said.append).command(grasp)
    assert out["ok"] and out["settled"] and not out.get("stalled") and out["t_s"] >= 0.9                 # ALIX's rule: the stroke, then alike
    assert out["stuck"] == [1, 2, 3, 4, 5, 6] and any("did NOT" in x for x in said)
    assert (robot_real.HAND_STALL_PAIRS, robot_real.HAND_STALL_MIN_S) == (3, 0.25)


def test_a_hand_asked_for_the_counts_it_settled_short_of_is_not_sent_again(monkeypatch):
    """The plan's half-grasp after the approach's: the hand stands at 582 where the glove stopped it, 66 counts from the
    516 asked, so the within-20-counts rule sent it again (1.25 s); asked for the very counts its last move settled
    short of, and still there, it is not sent (26 Oct). Anything else that moves the hand forgets that."""
    from alix_dispatch import peripherals

    sent, hw = [], []

    class FakeHand:
        poll_s, speed_set = 0.01, 500
        at = [582, 534, 541, 577, 445, 927]
        reads: list = []

        def __init__(self, robot, side, log=None):
            pass

        def open(self, speed_set=500):
            return 0

        def read(self):
            FakeHand.reads.append(list(self.at))
            return list(self.at)

        def set_speed(self, v):
            return 0

        def command(self, counts, wait=True):
            sent.append(list(counts))
            return dict(ok=True, arrived=False, settled=True, after=list(self.at), ret=0, t_s=0.9)

    monkeypatch.setattr(peripherals, "HandModbus", FakeHand)
    half = [516, 516, 516, 516, 133, 944]
    arm = _arm([0.0] * 7, hw)
    arm._hand_last = None
    out = arm.hand_to(half)                                                    # sent: settled short on the glove
    assert sent == [half] and out["settled"] and arm._hand_last == dict(counts=half, after=FakeHand.at)
    reads = len(FakeHand.reads)
    out = arm.hand_to(half)                                                    # asked again: not sent, and not read either (27 Oct)
    assert out["skipped"] and sent == [half] and out["after"] == FakeHand.at
    assert "already commanded to these counts and settled at" in hw[-1][1]["text"] and "nothing read" in hw[-1][1]["text"]
    assert len(FakeHand.reads) == reads
    arm._hand_last = None                                                      # a runner call or a staging moved it since: forgotten
    arm.hand_to(half)
    assert sent == [half, half] and len(FakeHand.reads) == reads + 1
    arm._hand_last = None
    FakeHand.at = [993, 993, 993, 993, 981, 992]                               # it moved (something else opened it): read, sent
    arm.hand_to(half)
    assert sent == [half, half, half]


def test_the_approachs_two_movejs_go_at_once_when_their_paths_are_checked_clear(monkeypatch):
    """26 Oct: the two MoveJs to a plan's start went one after the other, 1.6 to 3.2 s a task. With S2's world (home,
    scene, fixture) each arm's straight joint-space path is sampled and every pair of the two samples checked
    (collision_check.check_states, 49 states) while the poles and hands move; clear, both MoveJs go at once; not clear,
    or no world (a task run on its own), one after the other as before, with the reason said."""
    import threading
    import time as _time

    from alix_studio_lite.server import collision_check

    plan = mechanism.Plan("lid_open")
    asked, said, moved = [], [], []

    def check_states(times, arms, home, scene, fid, phases, cubicle_mm=1400, progress=None):
        asked.append(dict(n=len(times), sides=sorted(arms), phase=phases[0], fid=fid, q0=arms["left"]["q"][0], q1=arms["left"]["q"][-1],
                          pole=arms["right"]["pole"][0], hand=arms["right"]["hand"][0]))
        return dict(ok=not scene.get("blocked"), seconds=0.1, first=dict(what=["the two arms"]), events=[1])

    monkeypatch.setattr(collision_check, "check_states", check_states)

    class Arm:
        def __init__(self, side):
            self.side = side

        def drop_hand(self):
            pass

        def pole_and_hand(self, mm, hand, stop_hook=None):
            _time.sleep(0.05)
            return dict(pole_ok=True, hand_ok=True, pole_mm=mm, hand=list(hand))

        def move_to(self, target, speed, restore_pct):
            t0 = _time.monotonic()
            _time.sleep(0.1)
            moved.append((self.side, t0, _time.monotonic(), threading.current_thread().name))
            return dict(gap_deg=0.1)

    def fake(scene):
        here = {"left": [0.0, -82.0, 26.0, 98.0, -6.0, 62.0, 111.0], "right": [10.0, -80.0, -26.0, 98.0, 6.0, 62.0, 69.0]}
        links = {s: SimpleNamespace(view=lambda s=s: dict(reader=True, view_age_s=0.0, feedback=dict(age_s=0.1, joints_deg=here[s])),
                                    read_fresh=lambda s=s: dict(joints_deg=here[s])) for s in ("left", "right")}
        return SimpleNamespace(robot=SimpleNamespace(speed_pct=40, _measured=lambda side: (150.0, [1000] * 6), found_speed=lambda side: 50, links=links),
                               job=SimpleNamespace(progress=lambda *a, **k: None, on_cancel=None), check_stop=lambda: None,
                               say=lambda text, level="info": said.append(text), _arm=lambda side: Arm(side), _frames=None,
                               _settle_all=lambda: None,
                               room=None if scene is None else dict(home={}, scene=scene, fid="commode_c", cubicle_mm=1400),
                               _reading=lambda link: link.view()["feedback"],
                               _paths_clear=lambda *a, **k: mechanism.RealExec._paths_clear(fake_self[0], *a, **k))
    fake_self = [None]
    for scene, together in ((None, False), ({"blocked": True}, False), ({"blocked": False}, True)):
        moved.clear(); said.clear(); asked.clear()
        ex = fake(scene); fake_self[0] = ex
        mechanism.RealExec.approach(ex, plan, plan.targets("start"), "start")
        assert len(moved) == 2
        overlap = moved[0][1] < moved[1][2] and moved[1][1] < moved[0][2]
        assert overlap == together, (scene, moved, said)
        if scene is not None:
            assert asked and asked[0]["n"] == 49 and asked[0]["phase"] == "lid_closed" and asked[0]["sides"] == ["left", "right"]
            assert asked[0]["q1"] == plan.targets("start")["left"]["arm_deg"] and asked[0]["q0"] == [0.0, -82.0, 26.0, 98.0, -6.0, 62.0, 111.0]
            assert asked[0]["pole"] == pytest.approx(150.0, abs=0.1) and asked[0]["hand"] == [1000] * 6   # its grasp after its look
        if scene is not None and not together:
            assert any("one after the other: the paths would meet: the two arms" in t for t in said)
    assert mechanism.APPROACH_GRID == 7 and mechanism.PHASE_BEFORE["lid_open"] == "lid_closed" and mechanism.PHASE_BEFORE["seat_open"] == "lid_open"


def test_the_pole_asked_for_the_height_lite_last_confirmed_is_neither_sent_nor_read(monkeypatch):
    """27 Oct: before every plan entry with a pole target the pole's getter was asked (0.07 to 0.17 s), and none of them
    moved the pole: the approach had set it. Asked for the height Lite itself last confirmed (its own pole moves), the
    getter is not asked either; a runner call or a staging forgets it, and the getter decides as before."""
    from alix_dispatch import peripherals

    hw, asked = [], []

    class FakePole:
        def __init__(self, robot, log=None):
            self.log = log or (lambda *a: None)

        def state(self):
            asked.append("state")
            return {"pos": 150.0 * peripherals.LIFT_UNITS_PER_METRE / 1000.0, "mode": 0, "err_flag": 0}

        def move(self, height_m, speed_pct=100):
            asked.append(("move", height_m))
            return dict(ok=True, ret=0, final_m=height_m, t_s=1.0, verified_by="getter")

    monkeypatch.setattr(peripherals, "Pole", FakePole)
    arm = _arm([0.0] * 7, hw)
    arm._pole_last, arm._pole_done = 150.0, None
    out = arm.pole_to(150.093)
    assert out["no_op"] and out["no_op_source"] == "lite" and not asked and "nothing sent, nothing read" in hw[-1][1]["text"]
    arm._pole_last = None                                                      # forgotten (a runner call, a staging): the getter decides
    out = arm.pole_to(150.093)
    assert out["no_op"] and out["no_op_source"] == "getter" and asked == ["state"] and "already there" in hw[-1][1]["text"]
    assert arm._pole_last == pytest.approx(150.0)                              # ... and what it read is what Lite now knows
    out = arm.pole_to(200.0)                                                   # a move: sent, its final height kept
    assert asked[-1] == ("move", 0.2) and arm._pole_last == pytest.approx(200.0) and not out.get("no_op")


def test_the_hands_modbus_session_is_kept_across_the_pole_and_hand_step(monkeypatch):
    """27 Oct: S2's pole and hand step closed the end port's Modbus mode as it ended, so the plan's first hand move
    after it opened it again (0.3 s) and set its speed (0.07 s), although that move had nothing to send; and the next
    step opened it again too. The step leaves the mode on at the hand's SPEED_SET, the run's hand (keep_hand) takes
    that session as its own with nothing sent, and a step that follows does not open it again. A runner call forgets
    it (its Hand steps close the mode)."""
    import contextlib
    from types import SimpleNamespace

    from alix_dispatch import peripherals

    Hand0, _counts, _pole = robot_real._runner().load_aux_classes()
    opened, closed = [], []

    class Hand(Hand0):
        def open(self, speed_set=1000):
            opened.append(speed_set)
            self.speed_set = speed_set

        def close(self):
            closed.append(1)
            return 0

        def read(self):
            return [0, 0, 0, 0, 150, 1000]

        def _write(self, address, values):
            return 0

    Hand.__module__ = Hand0.__module__          # its arrival rule's numbers are the control program's (this module has stand-ins)

    class Pole:
        def __init__(self, robot):
            pass

        def height_m(self):
            return 0.15

        def move(self, height_m, speed_pct=100, block=False):
            return True, height_m

    class ModbusHand:
        poll_s, speed_set = 0.01, None
        opens: list = []

        def __init__(self, robot, side, log=None):
            self.opened = False

        def open(self, speed_set=1000):
            ModbusHand.opens.append(speed_set)
            self.opened = True
            return 0

    monkeypatch.setattr(robot_real, "_runner", lambda: SimpleNamespace(load_aux_classes=lambda: (Hand, None, Pole)))
    monkeypatch.setattr(peripherals, "HandModbus", ModbusHand)
    arm = robot_real.RealArm(SimpleNamespace(side="left", handle=SimpleNamespace(robot=object()), emulated=True,
                                             busy=lambda *a, **k: contextlib.nullcontext()))
    arm._hw = lambda verb, **kw: None
    arm._alix_log = lambda: (lambda *a: None)
    grip = [0, 0, 0, 0, 150, 1000]
    assert arm.pole_and_hand(150.0, grip)["hand_ok"] and opened == [1000] and not closed       # opened once, never closed
    assert arm._hand_port == dict(speeds=[1000] * 6) and arm._pole_last == pytest.approx(150.0)
    assert arm.pole_and_hand(150.0, grip)["hand_ok"] and opened == [1000]                      # the step after: not opened again
    assert arm.keep_hand() and arm._hand is not None and arm._hand.opened and not ModbusHand.opens   # the run's hand: nothing sent
    assert arm._hand_speeds == [1000] * 6 and arm._hand.speed_set == 1000
    arm.drop_hand()
    arm._hand_port = None                                                                      # as a runner call leaves it
    assert arm.keep_hand() and ModbusHand.opens == [1000]                                      # not known: opened, as before
    assert arm._hand_port == dict(speeds=[1000] * 6)


def test_the_arms_joints_come_from_the_feeds_frame_when_it_is_fresh():
    """27 Oct: before every stream the arm was read for the start check (0.1 to 0.2 s). With the run's feed kept on,
    its latest frame, under FEED_FRESH_S old, is that reading; a stale frame or no feed falls back to one reading."""
    from types import SimpleNamespace

    arm = _arm([1.0, -82.0, -26.0, 98.0, 6.0, 62.0, 69.0], [])
    arm.link.read_fresh = lambda: {"joints_deg": [1.0, -82.0, -26.0, 98.0, 6.0, 62.0, 69.0]}
    arm._feed = (object(), object())
    arm._tap = SimpleNamespace(t_mono=robot_real.time.monotonic(), frame={"joint_status": {"joint_position": [0, -81990, -26005, 97992, 6003, 61995, 69003]}})
    out = arm.joints_now()
    assert out["by"] == "feed" and out["joints_deg"] == pytest.approx([0.0, -81.99, -26.005, 97.992, 6.003, 61.995, 69.003])
    arm._tap = SimpleNamespace(t_mono=robot_real.time.monotonic() - 0.5, frame=arm._tap.frame)                 # stale: read
    assert arm.joints_now()["by"] == "read" and arm.joints_now()["joints_deg"][0] == 1.0
    arm._feed = None                                                                                      # no feed kept: read
    arm._tap = SimpleNamespace(t_mono=robot_real.time.monotonic(), frame=arm._tap.frame)
    assert arm.joints_now()["by"] == "read" and robot_real.FEED_FRESH_S == 0.1


def test_the_feed_is_turned_on_or_off_while_the_pole_travels(monkeypatch):
    """27 Oct: a task's feed was turned on after the pole and hand step (0.5 s on the slower arm's path) and off at the
    task's end with the camera's close (1.4 to 1.8 s, both arms standing). The step takes the feed's call in the pole's
    travel, between the pole's command and its arrival (the ramp alone is 0.75 s); a pole already at its height has it
    done right after; a feed that could not be turned on is raised after the pole and the hand are judged."""
    from types import SimpleNamespace

    from alix_studio_lite.server.jobs import UserError

    Hand0, _counts, _Pole0 = robot_real._runner().load_aux_classes()
    quick = robot_real._quick_pole

    def fresh_pole(**kw):                             # a QuickPole on a fake clock (_pole), handed to pole_and_hand as its pole
        monkeypatch.setattr(robot_real, "_quick_pole", quick)
        pole, clock, t0 = _pole(monkeypatch, **kw)
        pole.poll_s = 0.01
        monkeypatch.setattr(robot_real, "_quick_pole", lambda P: (lambda robot: pole))
        return pole, clock, t0

    pole, clock, t0 = fresh_pole(reads_target_after_s=0.05, idle_after_s=0.2)

    class Hand(Hand0):
        def open(self, speed_set=1000):
            self.speed_set = speed_set

        def read(self):
            return [0, 0, 0, 0, 150, 1000]

        def _write(self, address, values):
            return 0

    Hand.__module__ = Hand0.__module__
    arm = robot_real.RealArm(SimpleNamespace(side="left", handle=SimpleNamespace(robot=pole.robot), emulated=True,
                                             busy=lambda *a, **k: contextlib.nullcontext()))
    arm._hw = lambda verb, **kw: None
    arm._alix_log = lambda: (lambda *a: None)
    when = []
    arm.keep_feed = lambda label, records=None: when.append(("on", label, records, clock.t)) or f"{label}.frames.jsonl"
    arm.drop_feed = lambda: when.append(("off", None, None, clock.t)) or dict(frames=5)
    monkeypatch.setattr(robot_real, "_runner", lambda: SimpleNamespace(load_aux_classes=lambda: (Hand, None, type(pole))))
    out = arm.pole_and_hand(199.0, [0, 0, 0, 0, 150, 1000], feed=dict(on="LEFT_lid_open_seq", records="/tmp/x"))
    assert out["pole_ok"] and out["hand_ok"] and out["feed"] == "LEFT_lid_open_seq.frames.jsonl"
    assert when[0][:2] == ("on", "LEFT_lid_open_seq") and str(when[0][2]) == "/tmp/x" and t0["v"] is not None
    assert when[0][3] - t0["v"] < 0.05                                       # right after the pole's command, in its travel
    pole, clock, t0 = fresh_pole(reads_target_after_s=1e9, idle_after_s=0.0)             # the pole at its height already
    when.clear()
    out = arm.pole_and_hand(150.0, [0, 0, 0, 0, 150, 1000], feed="off")
    assert out["pole_ok"] and out["feed"] == dict(frames=5, off=True) and when == [("off", None, None, clock.t)] and t0["v"] is None
    when.clear()
    out = arm.pole_and_hand(150.0, [0, 0, 0, 0, 150, 1000])                  # no feed asked: nothing of it
    assert out["pole_ok"] and "feed" not in out and not when

    def refused(label, records=None):
        raise UserError("The LEFT controller's 5 ms feed to this computer could not be set up (port taken).", "Check.")

    arm.keep_feed = refused
    with pytest.raises(UserError, match="5 ms feed"):
        arm.pole_and_hand(150.0, [0, 0, 0, 0, 150, 1000], feed=dict(on="LEFT_lid_open_seq"))


def test_a_plans_settle_hold_counts_the_arms_own_settle():
    """27 Oct night: the plans hold 0.5 s before the lifted check (settle_before_check) on top of the stream's own end,
    which already waits for the arm to be within 0.5 degrees of its last line and still (0.37 to 0.65 s after a lift).
    The hold counts that measured rest: 0.49 s settled leaves 0.01 s to hold; an arm still at once (a short move)
    leaves the whole hold; a hold of another name is as the plan says."""
    from types import SimpleNamespace

    slept, said = [], []
    ex = mechanism.RealExec.__new__(mechanism.RealExec)
    ex.job = SimpleNamespace(progress=lambda *a, **k: None, sleep=lambda s: slept.append(round(s, 3)), log=lambda *a, **k: None)
    ex._log, ex._frames, ex._settled_s, ex._unsettled = None, None, 0.49, {}
    ex.say = lambda text, level="info": said.append(text)
    ex.hold(0.5, "settle_before_check")
    assert slept == [0.01] and "the arm's own settle took 0.49 s" in said[-1] and "holding 0.01 s more" in said[-1]
    ex._settled_s = 0.008                                                  # a short move: still at once, the hold stays
    ex.hold(0.5, "settle_before_check")
    assert slept[-1] == pytest.approx(0.492, abs=1e-6)
    ex._settled_s = 0.6                                                    # settled longer than the hold: nothing more
    ex.hold(0.5, "settle_before_check")
    assert len(slept) == 2 and "nothing more to wait" in said[-1]
    ex.hold(0.5, "hold_pose")                                              # another hold: the plan's seconds
    assert slept[-1] == 0.5


def test_the_colour_models_are_asked_once_a_check(tmp_path):
    """27 Oct night: the open check's two frames each ran the colour model, 0.45 s a frame, on frames 0.15 s apart at
    the same pose. With the check's id the first frame's result serves the check's later frames (said in their records);
    another check asks again."""
    from types import SimpleNamespace

    from alix_studio_lite.server.gap_check import GapCheck

    gc = GapCheck.__new__(GapCheck)
    calls = []
    gc._models_kept = None
    gc.core = lambda: SimpleNamespace(check_status=lambda image, **kw: calls.append(kw.get("depth_registration_m")) or dict(n=len(calls)))
    gc._geometry = lambda frame: {"image": None, "depth": None, "intrinsics": None, "transform": None, "fixture_pose": None}
    gc.registration_for = lambda pose: 0.0
    frame = SimpleNamespace(aligned=object())
    reused = {"models": False}
    first = gc._models(frame, None, "inspection_pose_1", True, "c1", reused)()
    assert first == dict(n=1) and calls == [0.0] and not reused["models"]
    reused = {"models": False}
    again = gc._models(frame, None, "inspection_pose_1", True, "c1", reused)()     # the check's second frame: no call
    assert again is first and calls == [0.0] and reused["models"]
    reused = {"models": False}
    other = gc._models(frame, None, "inspection_pose_1", True, "c2", reused)()      # another check: asked again
    assert other == dict(n=2) and len(calls) == 2 and not reused["models"]
    assert gc._models(SimpleNamespace(aligned=None), None, "", True, "c2", reused)() is None     # no aligned depth: no models
    plain = gc._models(frame, None, "", True, None, reused)()                      # no check id: asked, kept for none
    assert plain == dict(n=3) and gc._models_kept == ("c2", other)


def test_a_streams_end_is_waited_for_only_where_something_needs_the_arm_still():
    """27 Oct night: every stream's end (the arm within 0.5 degrees of its last line and still) was waited for before the
    next entry, 0.25 to 0.65 s after a lift, a push or a retreat, 1.6 s a run, although the next entry was the other
    arm's move or this arm's hand. Now the stream returns as its last line goes and the end is waited for (settle_end)
    before a check, a hold, this arm's next stream or pole move, and at the plan's end; the hold then counts it."""
    from types import SimpleNamespace

    calls, slept = [], []

    class Arm:
        def __init__(self, side):
            self.side = side

        def stream(self, rows, label, speed, restore_pct, bridge_speed, stop_hook, records, settle=True):
            calls.append((self.side, "stream", label, settle))
            return dict(lines=len(rows), lines_sent=len(rows), seconds=0.5, bridged_deg=None, settle=None)

        def settle_end(self, last_row, label=""):
            calls.append((self.side, "settle", label))
            return dict(waited_s=0.4, at_rest=True, by="feed", gap_deg=0.3, peak_deg_s=2.0)

        def hand_to(self, counts, duration_s=None):
            calls.append((self.side, "hand"))
            return dict(ok=True, arrived=True, after=list(counts))

        def pole_to(self, mm, duration_s=None):
            calls.append((self.side, "pole", round(mm)))
            return dict(ok=True)

        def joints_now(self):
            return dict(joints_deg=[0.0] * 7, by="feed")

    arms = {s: Arm(s) for s in ("left", "right")}
    ex = mechanism.RealExec.__new__(mechanism.RealExec)
    ex.job = SimpleNamespace(progress=lambda *a, **k: None, sleep=lambda s: slept.append(round(s, 3)), log=lambda *a, **k: None,
                             on_cancel=None, check=lambda: None)
    ex.robot = SimpleNamespace(speed_pct=60, found_speed=lambda side: 50, links={})
    ex.kit = SimpleNamespace(records_dir=lambda what: None, scripted=lambda expect: None)
    ex._log, ex._frames, ex._settled_s, ex._unsettled = None, None, 0.0, {}
    ex._feeds, ex._hands, ex.feeds_after, ex.live = {"left", "right"}, {"left", "right"}, True, lambda *a, **k: None
    ex.say = lambda text, level="info": None
    ex._arm = lambda side: arms[side]
    ex._keep_feed = lambda side: None
    ex._keep_hand = lambda side: None

    def entry(side, kind, stage, rows=None):
        import numpy as np

        rows = rows or [[0] * 7, [10] * 7]
        q = np.radians(np.asarray(rows, dtype=float) / 1000.0) if kind == "arm" else np.zeros((2, 7 if kind == "arm" else 6 if kind == "hand" else 1))
        e = mechanism.Entry(0, 0, stage, kind, side, [f"j{i}" for i in range(q.shape[1])], np.array([0.0, 0.5]), q, [], rows=rows if kind == "arm" else None)
        if kind == "hand":
            e.hand_counts = lambda at="last": [1000] * 6
        return e

    ex.move(entry("right", "arm", "move_to_inspect"))                 # a stream: returned without its settle
    assert calls[-1] == ("right", "stream", "RIGHT arm move_to_inspect", False) and "right" in ex._unsettled
    ex.move(entry("left", "arm", "lift"))                             # the other arm's stream: the right's end not waited for
    assert [c[1] for c in calls] == ["stream", "stream"]
    ex.move(entry("left", "hand", "hand_release"))                    # this arm's hand: not waited for either
    assert [c[1] for c in calls] == ["stream", "stream", "hand"] and set(ex._unsettled) == {"left", "right"}
    ex.move(entry("left", "arm", "retract"))                          # this arm's next stream: its end first
    assert calls[-2:] == [("left", "settle", "LEFT arm lift"), ("left", "stream", "LEFT arm retract", False)]
    ex.move(entry("right", "pole", "pole_half", rows=None))           # this arm's pole: the arm must stand first (the manufacturer)
    assert calls[-2] == ("right", "settle", "RIGHT arm move_to_inspect") and calls[-1] == ("right", "pole", 0)
    assert set(ex._unsettled) == {"left"}
    ex.hold(0.5, "settle_before_check")                               # a hold: every arm still first, and the hold counts it
    assert calls[-1] == ("left", "settle", "LEFT arm retract") and slept == [0.1] and not ex._unsettled
    ex.move(entry("left", "arm", "rest"))
    ex.finish = mechanism.RealExec.finish.__get__(ex)
    ex._arm("left").drop_feed = lambda: None
    ex._arm("right").drop_feed = lambda: None
    ex._arm("left").drop_hand = lambda: None
    ex._arm("right").drop_hand = lambda: None
    ex.kit.gap = SimpleNamespace(flush=lambda: None)
    ex.finish()                                                       # the plan's end: nothing moves on while an arm settles
    assert calls[-1] == ("left", "settle", "LEFT arm rest") and not ex._unsettled


def test_the_poles_command_goes_out_before_the_hands_whose_read_before_angle_set_is_lites_last(monkeypatch):
    """27 Oct night: the pole does not move for the first 0.75 s after its command (the push), and the hand's command before
    it, with its read, put every pole step's start 0.1 to 0.2 s later. The pole's command goes first; the hand's follows in
    the pole's travel, its read before ANGLE_SET being what Lite last read of the hand (its last move's readback) when Lite
    knows it, the device otherwise; a pole at its height already has the hand commanded right after; the pole's own move
    takes the step's one height read."""
    from types import SimpleNamespace

    Hand0, _counts, _Pole0 = robot_real._runner().load_aux_classes()
    quick, order = robot_real._quick_pole, []

    def fresh_pole(**kw):
        monkeypatch.setattr(robot_real, "_quick_pole", quick)
        pole, clock, t0 = _pole(monkeypatch, **kw)
        pole.poll_s = 0.01
        set_height, gets = pole.robot.rm_set_lift_height, []
        pole.robot.rm_set_lift_height = lambda speed, mm, block: order.append("pole command") or set_height(speed, mm, block)
        pole.height_m = lambda h=pole.height_m: gets.append(1) or h()
        monkeypatch.setattr(robot_real, "_quick_pole", lambda P: (lambda robot: pole))
        return pole, clock, t0, gets

    class Hand(Hand0):
        def open(self, speed_set=1000):
            self.speed_set = speed_set

        def read(self):
            order.append("hand read")
            return [0, 0, 0, 0, 150, 1000]

        def _write(self, address, values):
            order.append("hand write")
            return 0

    Hand.__module__ = Hand0.__module__
    pole, clock, t0, gets = fresh_pole(reads_target_after_s=0.05, idle_after_s=0.2)
    arm = robot_real.RealArm(SimpleNamespace(side="left", handle=SimpleNamespace(robot=pole.robot), emulated=True,
                                             busy=lambda *a, **k: contextlib.nullcontext()))
    arm._hw = lambda verb, **kw: None
    arm._alix_log = lambda: (lambda *a: None)
    monkeypatch.setattr(robot_real, "_runner", lambda: SimpleNamespace(load_aux_classes=lambda: (Hand, None, type(pole))))
    grip = [0, 0, 0, 0, 150, 1000]
    arm._hand_last = dict(counts=[1000] * 6, after=[1000, 1000, 1000, 997, 489, 990])    # the hand as Lite last read it
    out = arm.pole_and_hand(199.0, grip)
    assert out["pole_ok"] and out["hand_ok"]
    assert order[:2] == ["pole command", "hand write"] and "hand read" not in order[:2]   # the pole first; no read before ANGLE_SET
    assert order.count("hand read") >= 1 and len(gets) == 1                              # read back after the pole; one height read
    order.clear()
    pole, clock, t0, gets = fresh_pole(reads_target_after_s=0.05, idle_after_s=0.2)
    monkeypatch.setattr(robot_real, "_runner", lambda: SimpleNamespace(load_aux_classes=lambda: (Hand, None, type(pole))))
    assert arm._hand_last is not None and arm._hand_last["counts"] == grip
    arm._hand_last = None                                                                 # not known (a runner call forgot it)
    arm.pole_and_hand(199.0, grip)                                                        # (the fake pole stands at 150 mm again)
    assert order[:3] == ["pole command", "hand read", "hand write"]                       # the device read, after the pole's command
    order.clear()
    pole, clock, t0, gets = fresh_pole(reads_target_after_s=1e9, idle_after_s=0.0)        # at its height already: nothing sent
    monkeypatch.setattr(robot_real, "_runner", lambda: SimpleNamespace(load_aux_classes=lambda: (Hand, None, type(pole))))
    out = arm.pole_and_hand(150.0, grip)
    assert out["pole_ok"] and out["hand_ok"] and t0["v"] is None and order[0] == "hand write"   # the hand right after


def test_a_hand_move_takes_what_lite_last_read_for_its_read_before_angle_set(monkeypatch):
    """hand_to: Lite's own check that the hand is not there already, and ALIX's read before ANGLE_SET, take the readback
    of Lite's last hand move when it is known (27 Oct night: two reads, 0.1 s, before every hand command); the device is
    read as before when it is not, and read back after the command as always."""
    from types import SimpleNamespace

    from alix_dispatch import peripherals

    events = []

    class Fake:
        poll_s, speed_set = 0.01, 1000

        def __init__(self, robot, side, log=None):
            self.opened, self._sleep, self.log = True, lambda dt: None, log or (lambda *a: None)

        def open(self, speed_set=1000):
            return 0

        def set_speed(self, v):
            return 0

        def read(self):
            events.append("read")
            return [0, 0, 0, 0, 150, 1000]

        def command(self, counts, wait=True, timeout_s=None):          # ALIX's: its one read before the write
            before = self.read()
            events.append("write")
            return dict(ok=True, ret=0, target=list(counts), before=before, after=None, settled=False, arrived=False, t_s=0.0,
                        law_s=0.3, timeout_s=1.9, series=[], moved=[0] * 6, stuck=[], miss=None, held=[], write=None, partial=False)

    Fake.__module__ = peripherals.__name__                                # wrapped by _quick_hand as ALIX's hand is
    monkeypatch.setattr(peripherals, "HandModbus", Fake)
    arm = _arm([0.0] * 7, [])
    arm._hand_last = dict(counts=[1000] * 6, after=[1000, 1000, 1000, 997, 489, 990])
    out = arm.hand_to([0, 0, 0, 0, 150, 1000])
    assert out["ok"] and events[0] == "write" and events.count("read") >= 1               # no read before the write; read back after
    events.clear()
    arm._hand_last = None
    arm.hand_to([1000] * 6)                                                            # the device reads the grip: a real move
    assert events[:2] == ["read", "read"] and events[2] == "write"                       # not known: Lite's check and ALIX's read
    events.clear()
    arm._hand_last = dict(counts=[0, 0, 0, 0, 150, 1000], after=[0, 3, 0, 9, 150, 1000])
    out = arm.hand_to([0, 0, 0, 0, 160, 990])                                          # within 20 counts of the last readback
    assert out["skipped"] and not events                                               # nothing read, nothing sent


def test_a_settle_hold_counts_the_longest_settle_of_the_arms_since_the_last_stream_started():
    """28 Oct runs: with the two-arm stretch the last stream settled before the lifted check was the right's move to
    inspect pose (0.01 s, long done), and the hold took that: it held 0.49 s more after the left's lift had settled
    0.52 s, at every lifted check. The hold counts the longest settle since the last stream started, once; the next
    hold counts only the settles after it."""
    from types import SimpleNamespace

    slept, said = [], []
    ex = mechanism.RealExec.__new__(mechanism.RealExec)
    ex.job = SimpleNamespace(progress=lambda *a, **k: None, sleep=lambda s: slept.append(round(s, 3)), log=lambda *a, **k: None)
    ex._log, ex._frames, ex._settled_s = None, None, 0.0
    ex.say = lambda text, level="info": said.append(text)
    arms = {side: SimpleNamespace(settle_end=lambda last, label, w=w: dict(waited_s=w, by="feed", at_rest=True, gap_deg=0.4))
            for side, w in (("left", 0.52), ("right", 0.008))}
    ex.arms = arms
    ex._unsettled = {"left": ([1, 2, 3], "LEFT lift"), "right": ([4, 5, 6], "RIGHT inspect")}   # settled in this order at the hold
    ex.hold(0.5, "settle_before_check")
    assert not ex._unsettled and "the arm's own settle took 0.52 s" in said[-1] and "nothing more to wait" in said[-1]
    assert not slept                                                       # the lift's own settle was the hold
    ex.hold(0.5, "settle_before_check")                                    # no stream since: the whole hold
    assert slept == [0.5]
