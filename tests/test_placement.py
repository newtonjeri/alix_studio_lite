"""The room, the commode in it and each task's UGV placement (V32 feedback v2, slide 1): measured
by the operator, kept in the project, and the 3D scene built from them in ALIX's robot base frame
(X forward, Y left); a recording keeps the placement it was taught at; Prepare's Apply record."""
import json
import threading
import urllib.request

import pytest


@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-placement")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app, "data": data}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    call(ctx, "PUT", "/api/projects/alix_pl", {"id": "alix_pl", "name": "Placement", "createdAt": "2026-10-15T10:00:00.000Z",
                                               "updatedAt": "2026-10-15T10:00:00.000Z", "archived": False,
                                               "state": {"currentTask": 0}})
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json", "X-Lite-Token": ctx.get("tok") or ""}
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=120) as resp:
        return json.loads(resp.read())


P = "/api/projects/alix_pl"
ROOM = {"width_mm": 1400, "length_mm": 1950, "left_mm": 516, "right_mm": 516, "front_mm": 1400}


def test_the_footprint_is_alixs():
    from alix_studio_lite.server import fixtures

    fp = fixtures.footprint("commode_c")
    assert abs(fp["front_x"] - 0.4703) < 0.001 and abs(fp["back_x"] - 1.0085) < 0.001     # 1.005 m calibrated, 535 mm long
    assert abs((fp["left_y"] - fp["right_y"]) - 0.368) < 0.001 and fp["standoff_m"] == 0.25


def test_the_room_and_a_tasks_placement_are_checked_and_kept(lite):
    v = call(lite, "GET", P + "/placement")
    assert v["room"] is None and v["tasks"] == {} and v["standoff_range_mm"] == [250.0, 2000.0]
    assert v["scene"]["walls"] is None and v["scene"]["offset_m"] == [0.0, 0.0] and "No UGV standoff" in v["scene"]["notes"][0]
    assert "Enter the room width" in call(lite, "POST", P + "/placement/room", {**ROOM, "width_mm": ""})["error"]
    assert "Enter the UGV standoff" in call(lite, "POST", P + "/placement/task", {"task": "TASK 1A"})["error"]
    assert "250 to 2000" in call(lite, "POST", P + "/placement/task", {"task": "TASK 1A", "standoff_mm": 200})["error"]
    v = call(lite, "POST", P + "/placement/room", ROOM)
    assert v["room"]["width_mm"] == 1400 and v["scene"]["walls"] and not v["scene"]["notes"][1:]
    v = call(lite, "POST", P + "/placement/task", {"task": "TASK 1A", "standoff_mm": 380, "front_mm": 30, "left_mm": 20})
    assert v["tasks"]["TASK 1A"]["standoff_mm"] == 380 and v["tasks"]["TASK 1A"]["rear_mm"] is None
    s = v["scene"]
    assert s["offset_m"] == [round(0.13 - 0.03, 4), -0.02] and s["notes"] == [] and s["standoff_mm"] == 380
    walls = {w["name"]: w for w in s["walls"]}
    left_face = walls["left"]["center"][1] - 0.05
    assert abs(left_face - (0.1838 - 0.02 + 0.516)) < 0.001                    # the commode's left edge + 516 mm
    assert abs(s["front_x"] - (0.4703 + 0.10 - 1.4)) < 0.001                    # front tip - 1400 mm, with the UGV moved
    assert abs((walls["back"]["center"][0] - 0.05) - (s["front_x"] + 1.95)) < 0.001
    assert walls["left"]["size"][2] == 2.0 and walls["back"]["size"][0] == 0.1  # ALIX's cubicle walls


def test_measurements_that_disagree_are_said(lite):
    v = call(lite, "POST", P + "/placement/room", {**ROOM, "width_mm": 1500, "length_mm": 1700})
    notes = " ".join(v["scene"]["notes"])
    assert "room width is 1500 mm" in notes and "make 1400 mm" in notes
    v = call(lite, "POST", P + "/placement/room", {**ROOM, "length_mm": 1500, "front_mm": 1400, "task": "TASK 1A"})
    assert "back wall" in " ".join(v["scene"]["notes"])
    call(lite, "POST", P + "/placement/room", ROOM)


def test_a_recording_keeps_the_placement_it_was_taught_at(lite):
    from alix_studio_lite.motion import track
    import numpy as np

    call(lite, "POST", P + "/placement/task", {"task": "TASK 3", "standoff_mm": 300})
    md = np.repeat(np.array([[0, -90000, 0, 90000, 0, 0, 0]], dtype=np.int64), 50, axis=0)
    body = {"source": "upload", "name": "L_place.txt", "content": track.track_bytes(md).decode(), "pole_mm": 200, "hand": "OPEN",
            "task": "TASK 3"}
    out = call(lite, "POST", P + "/units/L-T01/recordings", body)
    rec = next(r for r in out["unit"]["recordings"] if r["file"] == out["saved"])
    assert rec["placement"]["standoff_mm"] == 300 and rec["placement"]["task"] == "TASK 3"
    call(lite, "POST", P + "/placement/task", {"task": "TASK 3", "standoff_mm": 500})        # changed later
    tl = call(lite, "GET", P + "/units/L-T01/timeline?which=raw&task=TASK%203")
    assert tl["scene"]["from"] == "the recording" and tl["scene"]["offset_m"][0] == 0.05     # 300 - 250 mm
    assert call(lite, "GET", P + "/placement?task=TASK%203")["scene"]["offset_m"][0] == 0.25  # the task now: 500 mm


def test_prepare_records_each_apply(lite):
    u = call(lite, "GET", P + "/units/L-R01")
    assert u["applied"] == {}
    u = call(lite, "POST", P + "/units/L-R01/setup", {"pole_mm": 120, "hand": "OPEN", "kind": "pole", "apply": False})["unit"]
    assert set(u["applied"]) == {"pole"} and u["applied"]["pole"]["pole_mm"] == 120
    u = call(lite, "POST", P + "/units/L-R01/setup", {"pole_mm": 120, "hand": "GRIP", "kind": "hand", "apply": False})["unit"]
    assert set(u["applied"]) == {"pole", "hand"} and u["applied"]["hand"]["hand"] == [0, 0, 0, 0, 150, 1000]


def test_each_apply_moves_its_own_part_and_the_cleaning_pose(lite):
    """Feedback v3 (15 Oct): Apply beside the pole moves the pole alone, Apply beside the hand the hand
    alone (the pole's Apply moved the hand too); the Cleaning Pose preset is 1000, 1000, 1000, 1000, 0, 1000."""
    import time

    from alix_studio_lite.server import staging

    assert staging.parse_hand("Cleaning Pose") == (1000, 1000, 1000, 1000, 0, 1000)
    assert staging.hand_words([1000, 1000, 1000, 1000, 0, 1000]) == "cleaning pose"
    for arm in ("left", "right"):
        call(lite, "POST", f"/api/robots/{arm}/connect", {})
    assert call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})["mode"] == "hwsim"
    arm = lite["app"].robot.arm("left")

    def apply(kind, pole_mm, hand):
        out = call(lite, "POST", P + "/units/L-R02/setup", {"pole_mm": pole_mm, "hand": hand, "kind": kind, "apply": True})
        assert out["job"]["kind"] == kind, out["job"]          # the pole's or the hand's own job, not a stage
        t0 = time.time()
        while call(lite, "GET", f"/api/jobs/{out['job']['id']}")["state"] == "running":
            assert time.time() - t0 < 30
            time.sleep(0.1)
        return out

    hand0 = list(arm.hand)
    apply("pole", 150, "CLEANING")
    assert arm.pole_mm == 150 and arm.hand == hand0              # the hand did not move
    apply("hand", 250, "Cleaning Pose")
    assert arm.hand == [1000, 1000, 1000, 1000, 0, 1000] and arm.pole_mm == 150   # nor the pole
    held = apply("hand", 150, [600, None, 600, 600, None, None])["unit"]
    assert arm.hand == [600, 1000, 600, 600, 0, 1000]              # a held channel keeps what it reads
    assert set(held["applied"]) >= {"pole", "hand"}
    call(lite, "POST", "/api/robots/mode", {"mode": "visual"})
