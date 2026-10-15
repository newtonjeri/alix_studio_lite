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
                             "--no-browser", "--port", str(port), "--data", str(data), "--logs", str(data / "logs")],
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
    saved = next((server["data"] / "projects" / "alix_real").rglob(rec["file"]))
    raw = saved.read_bytes()                                     # as the SDK wrote it, CRLF and all
    assert raw.count(b"\r\n") == POINTS and raw.startswith(b'{"point":[')


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

    j = wait(server, call(server, "POST", f"{U}/L-T01/run", {"confirmed": True}), 300)
    assert j["state"] == "done" and "Controller mode REAL (read back)" in text(j) and "read back on the controller" in text(j), j
    assert call(server, "GET", f"{U}/L-T01")["last_run"]["kind"] == "real_ok"

    assert call(server, "POST", "/api/robots/mode", {"mode": "hwsim"})["mode"] == "hwsim"
    j = wait(server, call(server, "POST", f"{U}/L-T01/run", {}), 300)
    assert j["state"] == "done" and "Controller mode SIM (read back)" in text(j) and "emulated" in text(j), j


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


def test_global_points_play_in_3d(server):
    """Feedback v2, slide 11: the controller's global points as a one-step plan, the arm going from
    point to point at 20 % of each joint's rating, with the 3D frames to show it."""
    names = ["STP_HOME", "STP_LID_OPEN", "CLEAN_A"]
    plan = call(server, "POST", "/api/projects/alix_real/points-plan", {"side": "left", "points": names})
    a = plan["rows"][0]["arms"]["left"]
    assert a["ready"] and a["lines"] > 100 and a["timeline"]["url"] == "/api/projects/alix_real/points-timeline"
    assert plan["total_s"] == a["end_at"] and "joint by joint" in plan["rows"][0]["note"]
    tl = call(server, "POST", a["timeline"]["url"], a["timeline"]["body"])
    assert tl["lines"] == a["lines"] and "scene" in tl
    assert "no global point NOPE" in call(server, "POST", "/api/projects/alix_real/points-plan", {"side": "left", "points": ["NOPE"]})["error"]
