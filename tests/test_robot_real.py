"""Lite commanding the arms (server/robot_real.py), against ALIX's emulated arms: Real robot with
the link; the pole and the hand staged and read back on the controller; drag teaching recorded by
the controller and saved byte for byte; upload with the size read back; a Real run and a Hardware
sim run (the controller's SIM mode); STOP mid-run; and every command in the hardware log."""
import json
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import pytest

from alix_studio_lite import vendor_paths

U = "/api/projects/alix_real/units"


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    data = tmp_path_factory.mktemp("lite-real")
    proc = subprocess.Popen([sys.executable, str(Path(__file__).with_name("emulated_lite.py")), "--robot", "emulated",
                             "--no-browser", "--port", str(port), "--data", str(data), "--logs", str(data / "logs"), "--projects", str(data / "projects")],
                            cwd=str(vendor_paths.LITE_ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    ctx = {"base": f"http://127.0.0.1:{port}", "data": data}
    for _ in range(150):
        try:
            ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
            break
        except OSError:
            time.sleep(0.2)
    call(ctx, "PUT", "/api/projects/alix_real", {"id": "alix_real", "name": "Real", "createdAt": "2026-10-14T10:00:00.000Z",
                                                 "updatedAt": "2026-10-14T10:00:00.000Z", "archived": False,
                                                 "state": {"currentTask": 0}})
    for side in ("left", "right"):
        assert call(ctx, "POST", f"/api/robots/{side}/connect", {})["connected"]
    yield ctx
    proc.terminate()
    proc.wait(10)


def call(ctx, method, path, body=None, tok=True):
    headers = {"Content-Type": "application/json"}
    if tok and ctx.get("tok"):
        headers["X-Lite-Token"] = ctx["tok"]
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=120) as resp:
        return json.loads(resp.read())


def wait(ctx, job, limit=180):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < limit, j
        time.sleep(0.2)


def text(job):
    return " | ".join(e["text"] for e in job["events"])


def test_real_stages_the_pole_and_hand_on_the_controller(server):
    assert call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})["mode"] == "real"
    setup = call(server, "GET", f"{U}/L-T01")["setup"]
    r = call(server, "POST", f"{U}/L-T01/setup", {"pole_mm": setup["pole_mm"], "hand": setup["hand"], "apply": True,
                                                    "confirmed": True})
    j = wait(server, r["job"])
    assert j["state"] == "done" and "read back on the controller" in text(j), j
    fb = call(server, "GET", "/api/robots/status")["arms"]["left"]["link"]["feedback"]      # read at once
    assert abs(fb["pole_mm"] - setup["pole_mm"]) <= 5 and fb["hand"][:4] == [500, 500, 500, 500]


def test_drag_teaching_records_on_the_controller(server):
    from emulated_lite import POINTS

    assert call(server, "POST", f"{U}/L-T01/teach/start", {"confirmed": True})["arm"]["teaching"]
    r = call(server, "POST", f"{U}/L-T01/teach/stop", {"name": "taught_here.txt"})
    assert r["take"] is None and "50 % open" in r["stage"]["text"]
    rec = call(server, "GET", f"{U}/L-T01")["recordings"][-1]
    assert rec["source"] == "teach" and rec["taught_arm"] == "left" and rec["lines"] == POINTS
    saved = next((server["data"] / "projects" / "Real").rglob(rec["file"]))
    raw = saved.read_bytes()                                     # as the SDK wrote it, CRLF and all
    assert raw.count(b"\r\n") == POINTS and raw.startswith(b'{"point":[')


def test_start_counts_down_before_drag_teaching(server):
    """START counts down first (the owner, 16 Oct: time to walk to the arm): every check is made at once,
    nothing else moves meanwhile, drag teaching starts at 0; a cancel during it records nothing."""
    from emulated_lite import POINTS

    r = call(server, "POST", f"{U}/L-T01/teach/start", {"confirmed": True, "delay_s": 0.6})
    assert r["job"]["state"] == "running" and not r["arm"]["teaching"]
    busy = call(server, "POST", "/api/robots/left/pole", {"mm": 120, "confirmed": True})
    assert "busy" in busy["error"] and "Drag teaching on the LEFT arm" in busy["error"]
    assert "0 to 30 seconds" in call(server, "POST", f"{U}/L-T01/teach/start", {"confirmed": True, "delay_s": 99})["error"]
    j = wait(server, r["job"])
    assert j["state"] == "done" and j["result"]["teaching"], j
    assert call(server, "GET", "/api/robots/status")["arms"]["left"]["teaching"]
    assert call(server, "POST", f"{U}/L-T01/teach/stop", {"name": "after_countdown.txt"})["unit"]["recordings"][-1]["lines"] == POINTS

    r = call(server, "POST", f"{U}/L-T01/teach/start", {"confirmed": True, "delay_s": 5})
    call(server, "POST", f"{U}/L-T01/teach/cancel", {})                 # Reset during the countdown
    assert wait(server, r["job"])["state"] == "cancelled"
    assert not call(server, "GET", "/api/robots/status")["arms"]["left"]["teaching"]
    hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
    assert sum(1 for x in hw if x["verb"] == "rm_start_drag_teach") == 2          # the first test's, and this one's once


def test_upload_and_runs_on_the_controller(server):
    u = call(server, "GET", f"{U}/L-T01")
    lib = next(x for x in u["recordings"] if x["source"] == "library")
    call(server, "POST", f"{U}/L-T01/active", {"file": lib["file"]})
    assert wait(server, call(server, "POST", f"{U}/L-T01/optimise", {}))["state"] == "done"
    assert call(server, "POST", f"{U}/L-T01/keep", {})["versions"][-1]["version"] == "v001"

    j = wait(server, call(server, "POST", f"{U}/L-T01/upload", {"program_id": 9, "confirmed": True, "limits_seen": True}))
    assert j["state"] == "done" and "bytes read back" in text(j), j
    progs = {p["id"]: p for p in call(server, "GET", "/api/robots/left/programs")["programs"]}
    assert progs[9]["name"] == "9_L-T01_v001.txt" and progs[9]["sha256"]

    # Real Test's step 4 opens once this version has run to its end in Hardware sim (the owner, 17 Oct)
    refused = call(server, "POST", f"{U}/L-T01/run", {"confirmed": True})
    assert refused["code"] == "hwsim" and "Hardware sim first" in refused["error"]
    assert not call(server, "GET", f"{U}/L-T01")["hwsim_passed"]
    switched = call(server, "POST", "/api/robots/mode", {"mode": "hwsim"})
    assert switched["mode"] == "hwsim" and switched["controllers"] == {"left": "SIM", "right": "SIM"}   # the controllers follow
    j = wait(server, call(server, "POST", f"{U}/L-T01/run", {}), 300)
    assert j["state"] == "done" and "Controller mode SIM (read back)" in text(j) and "emulated" in text(j), j
    assert call(server, "GET", f"{U}/L-T01")["hwsim_passed"]["kind"] == "dryrun_ok"

    switched = call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})
    assert switched["controllers"] == {"left": "REAL", "right": "REAL"}
    j = wait(server, call(server, "POST", f"{U}/L-T01/run", {"confirmed": True}), 300)
    assert j["state"] == "done" and "Controller mode REAL (read back)" in text(j) and "read back on the controller" in text(j), j
    assert call(server, "GET", f"{U}/L-T01")["last_run"]["kind"] == "real_ok"


def test_stop_ends_a_run_on_the_controller(server):
    call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})
    job = call(server, "POST", f"{U}/L-T01/run", {"confirmed": True})
    deadline = time.time() + 60                                  # until the program runs on the controller
    while "running (" not in text(call(server, "GET", f"/api/jobs/{job['id']}")):      # rm_upload.run's own line
        assert time.time() < deadline
        time.sleep(0.3)
    r = call(server, "POST", "/api/robots/stop-both", {}, tok=False)
    assert r["result"]["ok"] and "press Recover" in r["message"]
    j = wait(server, job)
    assert j["state"] == "cancelled", j
    for side in ("left", "right"):
        assert call(server, "POST", f"/api/robots/{side}/recover", {})["estop"] == "CLEAR"


def test_named_poses_are_refused_and_every_command_is_logged(server):
    r = call(server, "POST", "/api/robots/left/move-pose", {"name": "Default Start Pose", "confirmed": True})
    assert "not stored on the controller" in r["error"]
    hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
    verbs = {r["verb"] for r in hw}
    assert {"stage", "rm_start_drag_teach", "rm_stop_drag_teach", "rm_save_trajectory", "upload", "rm_upload",
            "run mode REAL", "run mode SIM", "run"} <= verbs
    assert all(r["ret"] == 0 for r in hw if r["verb"] in ("upload", "rm_save_trajectory"))
    assert any("VERIFIED" in (r["text"] or "") for r in hw if r["verb"] == "rm_upload")      # its read-back


def test_a_hand_settled_short_on_force_is_in_pose(monkeypatch):
    """The lab's manual hand move (14 Oct): channel 5 stopped 714 counts short of 0, on force. The
    control program's Hand.command takes ARRIVED or SETTLED as done, as staging already did."""
    from contextlib import contextmanager
    from types import SimpleNamespace

    vendor_paths.activate()
    from alix_dispatch import peripherals

    from alix_studio_lite.server.jobs import UserError
    from alix_studio_lite.server.robot_real import RealArm

    class Link:
        side, handle = "left", SimpleNamespace(robot=object())

        @contextmanager
        def busy(self, what):
            yield

    said = {}

    class Hand:
        def __init__(self, robot, side, log=None):
            pass

        def open(self, speed_set=None):
            return 0

        def command(self, counts, wait=True):
            return said["out"]

    monkeypatch.setattr(peripherals, "HandModbus", Hand)
    arm, pose = RealArm(Link()), [1000, 1000, 1000, 1000, 0, 1000]
    said["out"] = dict(ok=True, arrived=False, settled=True, ret=0, after=[1000, 999, 1000, 1000, 714, 992], t_s=0.89)
    assert arm.hand_to(pose)["after"][4] == 714
    said["out"] = dict(ok=False, arrived=False, settled=False, ret=0, after=None, t_s=3.0)       # timed out
    with pytest.raises(UserError, match="did not reach its pose"):
        arm.hand_to(pose)


def test_a_pole_already_where_it_is_asked_is_not_commanded(monkeypatch):
    """The owner, 23 Oct: every controller code 4117 (undocumented) came within a second of a pole command, three of
    them a command to where the pole already stood (lid open's 150 -> 150 mm). A pole idle within its 5 mm goal
    tolerance is not commanded, as ALIX's stage_arm does; a real move still goes to ALIX's Pole.move, and a pole
    still moving is commanded even inside the band (its reading leads it, ALIX B6)."""
    from contextlib import contextmanager
    from types import SimpleNamespace

    vendor_paths.activate()
    from alix_dispatch import peripherals

    from alix_studio_lite.server.robot_real import RealArm

    sent, moved = [], []

    class Robot:
        mm, mode = 150.0, 0

        def rm_get_lift_state(self):
            return 0, {"pos": self.mm, "mode": self.mode, "err_flag": 0}

        def rm_set_lift_height(self, speed, mm, block):
            sent.append(mm)
            return 0

    robot = Robot()

    class Link:
        side, handle = "left", SimpleNamespace(robot=robot)

        @contextmanager
        def busy(self, what):
            yield

    def move(self, height_m, speed_pct=100, **kw):
        moved.append(round(height_m * 1000))
        return dict(ok=True, ret=0, final_m=height_m, t_s=1.6, verified_by="getter")

    monkeypatch.setattr(peripherals.Pole, "move", move)
    arm = RealArm(Link())
    out = arm.pole_to(149.6)                                  # the plans' 150 mm, standing at 150
    assert out["ok"] and out["verified_by"] == "no_op" and out["sent"] is False and not moved and not sent
    assert arm.pole_to(156.0)["verified_by"] == "getter" and moved == [156]      # 6 mm away: a real move
    robot.mode = 1                                            # moving, its reading inside the band: commanded
    assert arm.pole_to(150.0)["verified_by"] == "getter" and moved == [156, 150]


def _workflow_arm(monkeypatch, recorder):
    """A RealArm on a real (not emulated) link whose runner v14 and ALIX recorder are fakes: what S2's recording does
    around a runner call, in order."""
    from contextlib import contextmanager
    from types import SimpleNamespace

    vendor_paths.activate()
    from alix_studio_lite.server import robot_real

    order = []

    class Runner:
        @staticmethod
        def normalize_workflow_step(st):
            return st

        @staticmethod
        def run_workflow_sequence(robot, a, ip, norm):
            order.append("runner")
            print("WORKFLOW STEP START 1/1: PROGRAM")
            print("WORKFLOW STEP COMPLETE 1/1: PROGRAM")
            return 0

    class Link:
        side, ip, emulated = "left", "192.168.1.10", False
        handle = SimpleNamespace(robot=object(), host_ip="192.168.1.239")

        @contextmanager
        def busy(self, what, **kw):
            yield

    monkeypatch.setattr(robot_real, "_runner", lambda: Runner)
    monkeypatch.setattr(robot_real, "CycleRecorder", lambda *a, **kw: recorder(order, *a, **kw))
    return robot_real.RealArm(Link()), order


def test_s2_records_a_real_controllers_feed_every_20_ms_around_each_runner_call(monkeypatch, tmp_path):
    """The owner, 23 Oct: S2 records the controllers' feed at 20 ms ("which is more than enough, that is 50Hz"): ALIX's
    recorder at cycle 4 on the arm's own port, started before the runner's call and stopped (the push put back) after."""
    from alix_studio_lite.server.robot_real import S2_FEED_CYCLE, UDP_PORT

    made = {}

    class Recorder:
        def __init__(self, order, out_path, host_ip, port=None, log=print, cycle=1):
            self.order, self.out_path, self.frames = order, out_path, 0
            made.update(host_ip=host_ip, port=port, cycle=cycle)

        def start(self, robot):
            self.order.append("start")

        def stop(self, robot):
            self.order.append("stop")
            self.frames = 412

    arm, order = _workflow_arm(monkeypatch, Recorder)
    cap = tmp_path / "canfd" / "x_LEFT_s2_01_SEQUENCE.frames.jsonl"
    out = arm.run_workflow([{"type": "program", "id": 71}], 15, 50, record=cap)
    assert out["ok"] and order == ["start", "runner", "stop"]
    assert S2_FEED_CYCLE == 4 and made == dict(host_ip="192.168.1.239", port=UDP_PORT["left"], cycle=4)
    assert out["feed"] == dict(file=str(cap), frames=412, cycle_ms=20) and cap.parent.is_dir()
    assert arm.run_workflow([{"type": "program", "id": 71}], 15, 50)["feed"] is None and order[-1] == "runner"   # not asked


def test_an_s2_stages_feed_is_kept_across_its_runner_calls(monkeypatch, tmp_path):
    """24 Oct: S2's recording, turned on and off around each runner call, took 0.46 s to start and as long to stop, at
    every call. A stage keeps one on for its whole part (keep_s2_feed): its runner calls start none of their own, and it
    is stopped once (drop_s2_feed, which never raises)."""
    class Recorder:
        def __init__(self, order, out_path, host_ip, port=None, log=print, cycle=1):
            self.order, self.out_path, self.frames = order, out_path, 0

        def start(self, robot):
            self.order.append("start")

        def stop(self, robot):
            self.order.append("stop")
            self.frames = 977

    arm, order = _workflow_arm(monkeypatch, Recorder)
    cap = tmp_path / "canfd" / "x_LEFT_s2_02_PART_2.frames.jsonl"
    assert arm.keep_s2_feed(cap) == cap.name and arm.keep_s2_feed(cap) == cap.name        # on once
    for _ in range(2):
        assert arm.run_workflow([{"type": "program", "id": 71}], 15, 50, record=tmp_path / "per_call.frames.jsonl")["feed"] is None
    assert order == ["start", "runner", "runner"]
    assert arm.drop_s2_feed() == dict(file=str(cap), frames=977, cycle_ms=20) and order[-1] == "stop"
    assert arm.drop_s2_feed() is None and order.count("stop") == 1


def test_s2s_pole_and_hand_confirms_the_hand_from_the_command_it_moved_by(monkeypatch):
    """24 Oct: S2's Pole with the Hand took 0.6-1.0 s after every pole: its hand sent again and waited for from the
    start, though it had moved while the pole travelled. It is confirmed by runner v14's own rule (arrived, or settled
    once a stroke's time has passed), timed from the command it moved by, and not sent again; a channel held (None) is
    not judged."""
    from contextlib import contextmanager
    from types import SimpleNamespace

    vendor_paths.activate()
    from alix_studio_lite.server import robot_real

    Hand0, _counts, _pole = robot_real._runner().load_aux_classes()
    writes = []

    class Hand(Hand0):
        reads: list = []

        def open(self, speed_set=1000):
            self.speed_set = speed_set

        def close(self):
            return 0

        def read(self):
            return list(Hand.reads.pop(0) if len(Hand.reads) > 1 else Hand.reads[0])

        def _write(self, address, values):
            writes.append(list(values))
            return 0

    class Pole:
        def __init__(self, robot):
            pass

        def height_m(self):
            return 0.2

        def move(self, height_m, speed_pct=100, block=False):
            time.sleep(0.6)                                        # the pole travels longer than the hand's stroke
            return True, height_m

    class Link:
        side, handle = "left", SimpleNamespace(robot=object())

        @contextmanager
        def busy(self, what, **kw):
            yield

    monkeypatch.setattr(robot_real, "_runner", lambda: SimpleNamespace(load_aux_classes=lambda: (Hand, None, Pole)))
    arm = robot_real.RealArm(Link())
    grip, opened = [0, 0, 0, 0, 150, 1000], [1000] * 6
    for reads, target in (([opened, [0, 3, 0, 9, 150, 1000]], grip),          # arrived while the pole travelled
                          ([opened, [0, 0, 0, 0, 471, 1000]], grip),          # its thumb stopped on force: settled
                          ([opened, [505, 498, 500, 503, 222, 931]], [500, 500, 500, 500, None, None])):   # the thumb held
        Hand.reads, t0 = reads, time.monotonic()
        writes.clear()
        out = arm.pole_and_hand(150.0, target)
        assert out["hand_ok"] and not out.get("hand_sent_again") and time.monotonic() - t0 < 0.95
        assert len(writes) == 1                                    # sent once, before the pole moved


def test_a_feed_that_cannot_be_recorded_is_said_and_the_run_goes_on(monkeypatch, tmp_path):
    class Recorder:
        def __init__(self, order, *a, **kw):
            pass

        def start(self, robot):
            raise RuntimeError("UDP port 8089 is already bound ON THIS HOST")

    arm, order = _workflow_arm(monkeypatch, Recorder)
    lines = []
    out = arm.run_workflow([{"type": "program", "id": 71}], 15, 50, on_line=lines.append, record=tmp_path / "x.frames.jsonl")
    assert out["ok"] and order == ["runner"] and out["feed"] is None
    assert any(ln.startswith("!! LEFT: its feed is not recorded for this part (UDP port 8089 is already bound") for ln in lines)


def test_global_points_are_the_controllers_and_moved_to_as_the_control_program_does(server):
    """The owner (14 Oct): the start-pose lists and Advanced show the controller's global points,
    found by name, and MoveJ / MoveL go to one (the control program's runner v14 functions)."""
    call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})
    pts = call(server, "GET", "/api/robots/left/points")
    assert pts["total"] == 3 and [p["name"] for p in pts["points"]] == ["STP_HOME", "STP_LID_OPEN", "CLEAN_A"]
    assert len(pts["points"][0]["joints"]) == 7 and len(pts["points"][0]["pose"]) == 6
    assert [p["name"] for p in call(server, "GET", "/api/robots/left/points?search=lid")["points"]] == ["STP_LID_OPEN"]
    assert call(server, "POST", "/api/robots/left/points/move", {"name": "STP_LID_OPEN", "move": "movej"})["code"] == "confirm"
    j = wait(server, call(server, "POST", "/api/robots/left/points/move", {"name": "STP_LID_OPEN", "move": "movej", "confirmed": True}))
    assert j["state"] == "done" and "At global point STP_LID_OPEN (MoveJ" in text(j), j
    fb = call(server, "GET", "/api/robots/status")["arms"]["left"]["link"]["feedback"]
    assert abs(fb["joints_deg"][0] - pts["points"][1]["joints"][0]) < 0.5
    j = wait(server, call(server, "POST", "/api/robots/left/points/move", {"name": "STP_HOME", "move": "movel", "confirmed": True}))
    assert j["state"] == "done" and "(MoveL" in text(j), j
    j = wait(server, call(server, "POST", "/api/robots/left/points/move", {"name": "NO_SUCH", "move": "movej", "confirmed": True}))
    assert j["state"] == "failed" and "did not reach global point 'NO_SUCH'" in j["error"]
    hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
    assert {"global points", "movej global point", "movel global point"} <= {r["verb"] for r in hw}
    assert any("GLOBAL MOVE COMPLETE: STP_LID_OPEN" in (r["text"] or "") for r in hw if r["verb"] == "runner v14")


def test_capture_reads_the_controller_and_a_start_point_is_moved_to(server):
    """V32's Capture Current Pose reads the linked controller (J1-J7, the tool pose, the pole, the
    hand, the controller); Move to Start Point goes back to those joints, read back."""
    c = call(server, "GET", "/api/robots/left/capture")
    assert len(c["joints_deg"]) == 7 and len(c["pose"]) == 6 and len(c["hand"]) == 6 and c["pole_mm"] is not None
    assert c["controller"]["side"] == "left" and c["controller"]["emulated"] and c["at"]
    call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})
    target = list(c["joints_deg"])
    target[0] += 5
    assert call(server, "POST", "/api/robots/left/move-joints", {"joints_deg": target, "name": "P1"})["code"] == "confirm"
    j = wait(server, call(server, "POST", "/api/robots/left/move-joints", {"joints_deg": target, "name": "P1", "confirmed": True}))
    assert j["state"] == "done" and "start point “P1”" in text(j), j
    again = call(server, "GET", "/api/robots/left/capture")
    assert max(abs(a - b) for a, b in zip(again["joints_deg"], target)) < 2.0
    hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
    assert {"capture pose", "move to pose"} <= {r["verb"] for r in hw}


def test_the_current_pose_is_saved_as_a_global_point(server):
    """Feedback v2, slide 14: the arm's pose now, saved on its controller as a global point (RealMan's
    add / update, read back with the runner); a name it has is replaced only after a confirmation."""
    call(server, "POST", "/api/robots/mode", {"mode": "hwsim"})
    c = call(server, "GET", "/api/robots/left/capture")
    assert "letters, digits" in call(server, "POST", "/api/robots/left/points/save", {"name": "bad name!"})["error"]
    moved = list(c["joints_deg"])
    moved[0] += 3
    assert "no longer at the pose" in call(server, "POST", "/api/robots/left/points/save",
                                           {"name": "STP_SEAT_1", "joints_deg": moved})["error"]
    out = call(server, "POST", "/api/robots/left/points/save", {"name": "STP_SEAT_1", "joints_deg": c["joints_deg"]})
    assert out["name"] == "STP_SEAT_1" and not out["replaced"] and len(out["pose"]) == 6
    pts = call(server, "GET", "/api/robots/left/points?search=SEAT")["points"]
    assert [p["name"] for p in pts] == ["STP_SEAT_1"] and abs(pts[0]["joints"][0] - c["joints_deg"][0]) < 0.01
    assert call(server, "POST", "/api/robots/left/points/save", {"name": "STP_SEAT_1"})["code"] == "replace"
    assert call(server, "POST", "/api/robots/left/points/save", {"name": "STP_SEAT_1", "replace": True})["replaced"]
    hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
    assert {"add global point", "update global point"} <= {r["verb"] for r in hw}


def test_the_speed_slider_moves_both_arms_and_each_controller_gets_its_speed_back(server):
    """Feedback v3 (15 Oct): one speed for every MoveJ / MoveL of both arms. On connecting, Lite reads the
    controller's global speed (the emulated arms report it; the real ones do not, then 50 % is taken, as
    ALIX and the runner leave it); each move runs with the global speed at 100 % and the slider's % (ALIX's
    park move and guard), and the speed found is put back after it, on disconnecting and on closing."""
    sp = call(server, "GET", "/api/robots/speed")
    assert sp["found"]["left"]["read"] and sp["found"]["left"]["pct"] == 100 and sp["assumed"] == 50
    assert sp["pct"] == 15                      # the slider starts at 15 % (the owner, 17 Oct) and connecting leaves it there
    assert "1 to 100" in call(server, "POST", "/api/robots/speed", {"pct": 0})["error"]
    assert call(server, "POST", "/api/robots/speed", {"pct": 35})["pct"] == 35
    assert call(server, "GET", "/api/robots/status")["speed"]["pct"] == 35
    call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})
    j = wait(server, call(server, "POST", "/api/robots/left/points/move", {"name": "STP_HOME", "move": "movej", "confirmed": True}))
    assert j["state"] == "done", j
    c = call(server, "GET", "/api/robots/left/capture")
    target = list(c["joints_deg"])
    target[1] += 4
    assert wait(server, call(server, "POST", "/api/robots/left/move-joints", {"joints_deg": target, "name": "P2", "confirmed": True}))["state"] == "done"
    hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
    assert [r for r in hw if r["verb"] == "read global speed"][0]["text"] == "100 %"
    assert [r for r in hw if r["verb"] == "movej global point"][-1]["args"]["speed"] == 35
    assert {"speed": 35, "restore_speed": 100}.items() <= [r for r in hw if r["verb"] == "move to pose"][-1]["args"].items()
    assert sum("global speed restored to 100" in (r.get("text") or "") for r in hw) >= 2       # after each move
    call(server, "POST", "/api/robots/left/disconnect", {})
    hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
    back = [r for r in hw if r["verb"] == "restore global speed"]
    assert back and back[-1]["args"]["pct"] == 100 and back[-1]["ok"]
    assert call(server, "POST", "/api/robots/left/connect", {})["connected"]
    assert call(server, "GET", "/api/robots/speed")["pct"] == 35          # the other arm stayed connected: the slider stays


def hardware(server) -> list[dict]:
    return [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]


def test_each_apply_moves_its_own_part_only(server):
    """Prepare's Apply beside the pole moves the pole only, and the one beside the hand the hand only (the
    owner, 15 Oct, twice): on the controller, the hand stays as it was while the pole moves, and the pole
    while the hand moves; each Apply stays applied."""
    call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})
    pole = call(server, "GET", f"{U}/L-T01")["setup"]["pole_mm"]
    grip = [0, 0, 0, 0, 150, 1000]
    j = wait(server, call(server, "POST", f"{U}/L-T01/setup", {"pole_mm": pole, "hand": grip, "apply": True, "kind": "hand",
                                                               "confirmed": True})["job"])
    assert j["state"] == "done", j
    held = call(server, "GET", "/api/robots/status")["arms"]["left"]["link"]["feedback"]["hand"]
    seen = len(hardware(server))
    other = pole - 100 if pole > 150 else pole + 100
    j = wait(server, call(server, "POST", f"{U}/L-T01/setup", {"pole_mm": other, "hand": grip, "apply": True, "kind": "pole",
                                                               "confirmed": True})["job"])
    assert j["state"] == "done", j
    fb = call(server, "GET", "/api/robots/status")["arms"]["left"]["link"]["feedback"]
    assert abs(fb["pole_mm"] - other) <= 5 and all(abs(a - b) <= 10 for a, b in zip(fb["hand"], held))   # the hand as it was
    new = hardware(server)[seen:]
    assert [r["verb"] for r in new if r["verb"] in ("pole move", "hand move", "stage")] == ["pole move"]
    assert not any("OPEN before the pole moves" in (r.get("text") or "") for r in new)
    assert {"pole", "hand"} <= set(call(server, "GET", f"{U}/L-T01")["applied"])           # both still applied
    j = wait(server, call(server, "POST", f"{U}/L-T01/setup", {"pole_mm": other, "hand": [1000] * 6, "apply": True, "kind": "hand",
                                                               "confirmed": True})["job"])
    assert j["state"] == "done", j
    fb = call(server, "GET", "/api/robots/status")["arms"]["left"]["link"]["feedback"]
    assert abs(fb["pole_mm"] - other) <= 5 and min(fb["hand"]) >= 975                      # the pole where it was


def test_a_linked_arm_is_not_let_go_while_a_job_runs_on_it(server):
    """A move or run under way keeps its link: its own finally puts the controller's speed back, which a
    closed link cannot. Disconnecting waits for STOP (or the end); then the speed is put back and the link
    closes."""
    call(server, "POST", "/api/robots/mode", {"mode": "hwsim"})
    now = call(server, "GET", "/api/robots/status")["arms"]["right"]["pole_mm"]
    job = call(server, "POST", "/api/robots/right/pole", {"mm": 0 if now > 150 else 300})
    r = call(server, "POST", "/api/robots/right/disconnect", {})
    assert r.get("code") == "busy" and "STOP" in r["hint"], r
    assert call(server, "GET", "/api/robots/status")["arms"]["right"]["connected"]
    call(server, "POST", "/api/robots/right/stop", {})
    assert wait(server, job)["state"] in ("cancelled", "failed")
    assert not call(server, "POST", "/api/robots/right/disconnect", {})["connected"]
    assert [r for r in hardware(server) if r["verb"] == "restore global speed"][-1]["side"] == "right"
    assert call(server, "POST", "/api/robots/right/connect", {})["connected"]
    assert call(server, "POST", "/api/robots/right/recover", {})["estop"] == "CLEAR"


def test_arm_drag_mode_frees_one_arm_and_holds_everything_else(server):
    """Arm Drag Mode (feedback v5, slide 8): the arm free to be moved by hand for recovery, nothing
    recorded; every other motion of both arms refused until Normal Mode, which checks the arm is safe;
    STOP in Drag Mode leaves it, and the arm stays held until Normal Mode."""
    call(server, "POST", "/api/robots/mode", {"mode": "hwsim"})
    assert call(server, "POST", "/api/robots/left/drag-mode", {"on": True})["code"] == "mode"
    call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})
    assert call(server, "POST", "/api/robots/left/drag-mode", {"on": True})["code"] == "confirm"
    assert call(server, "POST", "/api/robots/left/drag-mode", {"on": True, "confirmed": True})["recovery"] == "drag"
    assert call(server, "GET", "/api/robots/status")["arms"]["left"]["recovery"] == "drag"
    for side in ("left", "right"):
        r = call(server, "POST", f"/api/robots/{side}/pole", {"mm": 150, "confirmed": True})
        assert "Arm Drag Mode" in r["error"], r
    assert call(server, "POST", "/api/robots/left/disconnect", {})["code"] == "busy"
    assert [x for x in hardware(server) if x["verb"] == "rm_start_drag_teach"][-1]["args"] == [0]     # recording off
    back = call(server, "POST", "/api/robots/left/drag-mode", {"on": False})
    assert back["recovery"] is None and back["check"]["ok"], back
    assert [x for x in hardware(server) if x["verb"] == "rm_stop_drag_teach"]

    call(server, "POST", "/api/robots/left/drag-mode", {"on": True, "confirmed": True})
    call(server, "POST", "/api/robots/left/stop", {})
    held = call(server, "GET", "/api/robots/status")["arms"]["left"]
    assert held["recovery"] == "held" and "STOP" in held["recovery_note"]
    assert call(server, "POST", "/api/robots/right/pole", {"mm": 150, "confirmed": True})["error"]     # held, and STOP's hold
    assert call(server, "POST", "/api/robots/left/drag-mode", {"on": False})["recovery"] is None
    assert call(server, "POST", "/api/robots/left/recover", {})["estop"] == "CLEAR"


def test_drag_sensitivity_is_read_set_and_given_back_and_errors_cleared(server):
    """The drag sensitivity (feedback v5, slide 2): read when the arm is connected, set and read back,
    and the controller's own given back when Lite lets go; its errors read and cleared (nothing moves)."""
    sv = call(server, "GET", "/api/robots/sensitivity")
    assert sv["left"]["found"] == 100 and sv["left"]["pct"] == 100 and sv["left"]["linked"]
    assert "0 to 100" in call(server, "POST", "/api/robots/left/sensitivity", {"pct": 101})["error"]
    assert call(server, "POST", "/api/robots/left/sensitivity", {"pct": 60})["pct"] == 60
    assert call(server, "GET", "/api/robots/status")["sensitivity"]["left"]["pct"] == 60
    call(server, "POST", "/api/robots/left/disconnect", {})
    sets = [x["args"] for x in hardware(server) if x["verb"] == "rm_set_drag_teach_sensitivity"]
    assert sets[-2:] == [[60], [100]]                                     # set, then given back on letting go
    assert call(server, "POST", "/api/robots/left/connect", {})["connected"]
    e = call(server, "GET", "/api/robots/left/errors")
    assert e["readable"] and e["clean"] and e["text"] == "clean"
    c = call(server, "POST", "/api/robots/left/clear-errors", {})
    assert c["ok"] and c["clean"]


def test_data_management_lists_what_each_controller_stores(server):
    """Data Management (the owner, 16 Oct): each arm's controller as RealMan's own web page lists it, read only:
    its programs (slot, name, size, stored speed; Lite's record of the ones it sent, named by the project's
    file of those bytes), the program its IO starts, its global points and its tracks (a 4th-generation call:
    -4 on the lab's controllers); never while the arm moves."""
    call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})
    u = call(server, "GET", f"{U}/L-T01")
    if not u["versions"]:                                  # as test_upload_and_runs_on_the_controller leaves it
        lib = next(x for x in u["recordings"] if x["source"] == "library")
        call(server, "POST", f"{U}/L-T01/active", {"file": lib["file"]})
        assert wait(server, call(server, "POST", f"{U}/L-T01/optimise", {}))["state"] == "done"
        call(server, "POST", f"{U}/L-T01/keep", {})
    if not {p["id"]: p for p in call(server, "GET", "/api/robots/left/programs")["programs"]}.get(9, {}).get("sha256"):
        j = wait(server, call(server, "POST", f"{U}/L-T01/upload", {"program_id": 9, "confirmed": True, "limits_seen": True}))
        assert j["state"] == "done", j
    d = call(server, "GET", "/api/robots/left/data?pid=alix_real")
    assert d["linked"] and d["connected"] and d["ip"] and d["read"]
    progs = {p["num"]: p for p in d["programs"]}
    assert progs[9]["name"] == "9_L-T01_v001.txt" and progs[9]["bytes"] > 0 and "speed" in progs[9]
    assert progs[9]["lite"]["sent"] and progs[9]["lite"]["project_file"] and progs[9]["lite"]["lines"] > 0
    assert progs[9]["lite"]["seconds"] == round(0.005 * (progs[9]["lite"]["lines"] - 1), 3)            # its execution time
    assert d["default_program"] == 0
    assert [p["name"] for p in d["points"]][:3] == ["STP_HOME", "STP_LID_OPEN", "CLEAN_A"] and len(d["points"][0]["joints"]) == 7
    assert not d["tracks"]["available"] and "4th-generation" in d["tracks"]["why"]
    assert set(d["not_in_api"]) == {"script", "models"}
    assert call(server, "GET", "/api/robots/right/data")["programs"] is not None

    job = call(server, "POST", f"{U}/L-T01/run", {"confirmed": True})
    time.sleep(0.3)
    busy = call(server, "GET", "/api/robots/left/data")
    assert busy.get("code") == "busy" and "busy" in busy["error"], busy
    wait(server, job, 300)
    hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
    assert "controller data" in {r["verb"] for r in hw}


def test_what_a_controller_stores_is_deleted_only_when_asked_and_read_back(server):
    """The owner, 21 Oct: delete anything stored on the controller (Data Management, S1's merged files), always asked
    first. RealMan's own delete calls, each read back; Lite's record of a program it sent goes with it; never while
    the arm runs something; a track only on a 4th-generation controller."""
    call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})
    D = "/api/robots/left/stored/delete"
    progs = lambda: {p["num"]: p for p in call(server, "GET", "/api/robots/left/data")["programs"]}       # noqa: E731
    if 9 not in progs():                                   # as test_data_management_lists_what_each_controller_stores leaves it
        assert wait(server, call(server, "POST", f"{U}/L-T01/upload", {"program_id": 9, "confirmed": True,
                                                                       "limits_seen": True}))["state"] == "done"
    asked = call(server, "POST", D, {"kind": "program", "program_id": 9})
    assert asked["code"] == "confirm_delete" and 9 in progs()                    # nothing deleted without the answer
    job = call(server, "POST", f"{U}/L-T01/run", {"confirmed": True})
    time.sleep(0.3)
    busy = call(server, "POST", D, {"kind": "program", "program_id": 9, "confirmed": True})
    assert busy.get("code") == "busy", busy
    wait(server, job, 300)
    assert 9 in progs()                                                          # still there: refused while it ran
    out = call(server, "POST", D, {"kind": "program", "program_id": 9, "confirmed": True})
    assert out["kind"] == "program" and out["which"] == 9 and out["said"].startswith("Program 9 (9_L-T01_v001.txt) is deleted"), out
    assert 9 not in progs()
    assert not call(server, "GET", "/api/robots/left/programs")["programs"] or 9 not in {
        p["id"] for p in call(server, "GET", "/api/robots/left/programs")["programs"]}
    assert "holds no program 9" in call(server, "POST", D, {"kind": "program", "program_id": 9, "confirmed": True})["error"]

    names = lambda: [p["name"] for p in call(server, "GET", "/api/robots/left/data")["points"]]           # noqa: E731
    assert "CLEAN_A" in names()
    out = call(server, "POST", D, {"kind": "point", "name": "CLEAN_A", "confirmed": True})
    assert out["said"] == "Global point “CLEAN_A” is deleted from the LEFT controller." and "CLEAN_A" not in names()
    assert "not a global point" in call(server, "POST", D, {"kind": "point", "name": "CLEAN_A", "confirmed": True})["error"]
    assert "4th-generation" in call(server, "POST", D, {"kind": "track", "name": "T1", "confirmed": True})["error"]

    call(server, "POST", "/api/robots/mode", {"mode": "visual"})                   # as an upload: never in Visual sim
    assert call(server, "POST", D, {"kind": "point", "name": "STP_HOME", "confirmed": True})["code"] == "mode"
    call(server, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})
    hw = [json.loads(x) for x in next((server["data"] / "logs").iterdir()).joinpath("hardware.jsonl").read_text().splitlines()]
    done = {r["verb"]: r for r in hw if r["verb"].startswith("delete")}
    assert done["delete program"]["ret"] == 0 and done["delete global point"]["ret"] == 0 and done["delete track"]["ret"] == -4


def test_a_lid_or_seat_run_keeps_one_feed_on_and_holds_the_global_ratio(monkeypatch, tmp_path):
    """24 Oct: every move turned the controller's 5 ms feed on and off (four controller calls, 1.1 s) and set and put back
    the global ratio (0.17 s). A run keeps one feed on (keep_feed ... drop_feed), each move's watch attached to it in
    turn; once the run has left the ratio at the streams' 50 %, and 50 % is what goes back, a move neither sets nor
    puts it back (ALIX's run_verified, ratio_held)."""
    import time as _time
    from contextlib import contextmanager
    from types import SimpleNamespace

    vendor_paths.activate()
    from alix_dispatch import staging as alix_staging, track

    from alix_studio_lite.server import robot_real

    made, calls = [], []

    class Recorder:
        def __init__(self, out_path, host_ip, port=None, log=print, **kw):
            self.out_path, self.frames, self.monitor, self.stopped = Path(out_path), 0, None, False
            made.append(self)

        def start(self, robot):
            pass

        def stop(self, robot):
            self.stopped = True

        def attach_monitor(self, m):
            self.monitor = m
            if m is not None:                                 # the feed is running: a frame comes at once
                m.on_frame(_time.monotonic(), {"err": [0], "arm_current_status": "idle",
                                                "joint_status": {"joint_position": [0] * 7, "joint_err_code": [0] * 7}})

    def run_verified(robot, **kw):
        calls.append((kw["ratio_held"], kw["restore_pct"]))
        held = kw["ratio_held"]
        return dict(ok=True, verdict="host_timed", canfd={"lines_sent": kw["points"], "replay_s": 0.01},
                    restored={"global_speed": {} if held else {"restored": True}})

    class Link:
        side, ip, emulated = "left", "192.168.1.10", False
        handle = SimpleNamespace(robot=object(), host_ip="192.168.1.239")

        @contextmanager
        def busy(self, what, **kw):
            yield

    monkeypatch.setattr(track, "UdpRecorder", Recorder)
    monkeypatch.setattr(track, "run_verified", run_verified)
    monkeypatch.setattr(alix_staging, "park_arm", lambda *a, **kw: dict(ok=True))
    monkeypatch.setattr(track, "start_gap_deg", lambda *a, **kw: 0.0)
    arm = robot_real.RealArm(Link())
    rows = [[0] * 7] * 3
    assert arm.keep_feed("LEFT_seat_open", records=tmp_path) == made[0].out_path.name and len(made) == 1
    for _ in range(3):
        arm.stream(rows, "LEFT arm move", speed=50, restore_pct=50, records=tmp_path)
    arm.move_to([0] * 7, speed=15, restore_pct=50)            # a MoveJ puts 50 back too
    arm.stream(rows, "LEFT arm move", speed=50, restore_pct=50, records=tmp_path)
    arm.stream(rows, "LEFT arm move", speed=50, restore_pct=100, records=tmp_path)   # 100 to put back: set, then put back
    assert len(made) == 1 and not made[0].stopped and made[0].monitor is None       # one feed, kept on, nothing watching
    assert calls == [(False, 50), (True, 50), (True, 50), (True, 50), (False, 100)]
    assert arm.drop_feed()["capture"] == made[0].out_path.name and made[0].stopped
    arm.stream(rows, "LEFT arm move", speed=50, restore_pct=50, records=tmp_path)   # no run: its own feed, as before
    assert len(made) == 2 and made[1].stopped and calls[-1] == (False, 50)
