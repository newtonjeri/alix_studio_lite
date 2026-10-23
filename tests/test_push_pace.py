"""S2's controller feed at 10 or 20 ms (the owner, 23 Oct): robot_real.CycleRecorder writes ALIX's push config at the
asked cycle, and tools/push_pace.py times a stored program from the controller's "running" to its end."""
import importlib.util
import json

from alix_studio_lite import vendor_paths


def _tool():
    spec = importlib.util.spec_from_file_location("push_pace", vendor_paths.LITE_ROOT / "tools" / "push_pace.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_recorder_writes_the_push_at_its_cycle_and_restores_as_alix_does(monkeypatch, tmp_path):
    vendor_paths.activate()
    from alix_dispatch import track
    from Robotic_Arm.rm_robot_interface import rm_realtime_push_config_t, rm_udp_custom_config_t

    from alix_studio_lite.server.robot_real import CycleRecorder

    seen = []
    monkeypatch.setattr(track.UdpRecorder, "_apply_push",
                        lambda self, robot, cfg, want, what: (seen.append((what, cfg.cycle)), (True, 0))[1])
    rec = CycleRecorder(tmp_path / "f.jsonl", "127.0.0.1", port=8089, cycle=4)

    def cfg():
        return rm_realtime_push_config_t(cycle=1, enable=True, port=8089, force_coordinate=-1, ip="127.0.0.1",
                                         custom_config=rm_udp_custom_config_t())

    assert rec._apply_push(None, cfg(), {}, "capture") == (True, 0) and seen == [("capture", 4)]       # 20 ms
    rec._apply_push(None, cfg(), {}, "restore")
    assert seen[-1] == ("restore", 1)                         # the config it puts back is ALIX's, untouched
    assert rec.cycle == 4 and isinstance(rec, track.UdpRecorder)


def test_the_verdict_is_the_slowdown_against_the_push_off():
    """S2 stops a Program past 105 % of 5 ms a line; with the push off the lab's play 4.5-4.9 ms, so a cycle may slow
    them 3 % with a margin, 7 % at most; the shortest cycle within the margin is the one S2 records at."""
    tool = _tool()
    rows = tool.summary([(0, 5.00), (0, 5.02), (2, 5.20), (4, 5.06)])          # off 5.01; 10 ms +3.8 %; 20 ms +1.0 %
    assert [(c, v) for c, _, _, v in rows] == [(2, "usable, little margin"), (4, "OK for S2")]
    rows = tool.summary([(0, 5.0), (2, 5.6), (4, 5.4)])                          # +12 %, +8 %
    assert [v for *_, v in rows] == ["too slow for S2", "too slow for S2"]
    assert tool.summary([(2, 5.0)]) == []                                        # nothing to compare with


class _Arm:
    """An arm whose blocking MoveJ ends at once on a stale arrival, as both arms' did at the lab on 23 Oct ("Move
    command result is: false", ret 1), while a non-blocking one is taken and the joints get there 10 degrees a read."""

    def __init__(self, at):
        self.q, self.target, self.calls = list(at), None, []

    def rm_set_plan_speed(self, pct):
        self.calls.append(("ratio", pct))
        return 0

    def rm_movej(self, joint, v, r, connect, block):
        self.calls.append(("movej", block))
        if block:
            return 1
        self.target = list(joint)
        return 0

    def rm_get_current_arm_state(self):
        if self.target is not None:
            self.q = [q + max(-10.0, min(10.0, t - q)) for q, t in zip(self.q, self.target)]
        return 0, {"joint": list(self.q), "err": {"err": ["0"]}}


def test_the_move_to_the_first_point_is_alix_bridge_non_blocking_and_read_from_the_joints(tmp_path):
    tool = _tool()
    arm = _Arm([0.0] * 7)
    first = [35.0, -20.0, 0.0, 10.0, 0.0, 5.0, 0.0]
    rec = tool.Record(tmp_path / "x_LEFT", "left")
    ok, said = tool.move_to(arm, first, 15, rec, timeout_s=10.0)
    rec.close()
    assert ok, said
    assert arm.calls == [("ratio", 100), ("movej", 0)]               # the ratio at 100 %, then MoveJ non-blocking
    assert max(abs(a - b) for a, b in zip(arm.q, first)) <= tool.AT_START_DEG
    events = [json.loads(line) for line in (tmp_path / "x_LEFT.events.jsonl").read_text().splitlines()]
    assert [e["what"] for e in events][:2] == ["rm_set_plan_speed", "rm_movej (non-blocking)"]
    assert events[-1]["what"] == "at the first point"


def test_the_record_keeps_what_the_terminal_shows_the_sdks_own_lines_included(tmp_path):
    import ctypes
    import os

    tool = _tool()
    path = tmp_path / "x.log"
    restore = tool.tee_terminal(path)
    try:
        print("a line from Python")
        os.write(1, b"a line written to fd 1\n")
        libc = ctypes.CDLL(None)
        libc.printf(b"Move command result is: false, current Device: %d\n", 0)       # the SDK's C printf
        libc.fflush(None)
    finally:
        restore()
    text = path.read_text()
    for want in ("a line from Python", "a line written to fd 1", "Move command result is: false, current Device: 0"):
        assert want in text
    assert all(line[2] == ":" and line[12] == " " for line in text.splitlines())   # each line with the lab's clock


def test_a_program_is_timed_from_running_to_its_end(tmp_path):
    tool = _tool()

    class Robot:
        states = [0, 0, 1, 1, 1, 0]                           # loading, loading, running x3, ended

        def rm_set_program_id_run(self, slot, speed, block):
            assert block == 0                                 # non-blocking, as Lite starts programs
            return 0

        def rm_get_program_run_state(self):
            return 0, {"run_state": self.states.pop(0) if len(self.states) > 1 else self.states[0]}

    secs, said = tool.play(Robot(), 59, 15, timeout_s=10)
    assert said == "ended" and 0.0 < secs < 0.5
    track = tmp_path / "t.txt"
    track.write_text("\n".join(json.dumps({"point": [i * 1000, 0, 0, 0, 0, 0, 0]}) for i in range(5)))
    first, lines = tool.first_point_and_lines(track)
    assert lines == 5 and first == [0.0] * 7
