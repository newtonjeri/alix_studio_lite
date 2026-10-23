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
