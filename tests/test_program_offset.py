"""A recorded Program moved for the UGV's offset (motion/program_offset.py, tools/program_offset.py): the conventions,
the file kept as written, the lab's five-point test file at zero and at the lab's test offset, an offset past reach,
the command line and the page's server."""
import importlib.util
import json
import sys
import threading
import urllib.error
import urllib.request

import numpy as np
import pytest

from alix_studio_lite import vendor_paths
from alix_studio_lite.motion import program_offset as po

FIVE_POINT = vendor_paths.LITE_ROOT / "Resources" / "Programs" / "offset_tests" / "OFF_5P_TES.txt"
TOOL = vendor_paths.LITE_ROOT / "tools" / "program_offset.py"


@pytest.fixture(scope="module")
def five_point():
    text = FIVE_POINT.read_text()
    md, rows = po.read_program(text)
    return text, md, rows


@pytest.fixture(scope="module")
def lab_case(five_point):
    """The lab's test: the UGV 20 mm back and 20 mm left of where it stood at the recording (right arm, pole 150)."""
    _, md, _ = five_point
    return po.shift_program(md, "right", 0.150, po.UgvOffset(-20.0, 20.0, 0.0))


@pytest.fixture(scope="module")
def tool():
    spec = importlib.util.spec_from_file_location("program_offset_tool", TOOL)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    yield mod
    sys.modules.pop(spec.name, None)


# ─────────────────────────────────────────────────────────────── conventions and the file

def test_the_ugv_back_and_left_moves_the_path_forward_and_right():
    off = po.UgvOffset(-20.0, 20.0, 0.0)
    M = off.path_move()
    assert M[:3, 3] * 1000 == pytest.approx([20.0, -20.0, 0.0])
    assert np.allclose(M[:3, :3], np.eye(3))
    assert off.words() == "UGV 20 mm back, 20 mm left -> the hand path moves 20 mm forward, 20 mm right"


def test_a_ugv_turn_turns_the_path_the_other_way_about_the_robot_s_centre():
    M = po.UgvOffset(0.0, 0.0, 2.0).path_move()
    p = M @ np.array([0.7, 0.0, 0.6, 1.0])                  # a point 700 mm ahead turns clockwise: to the right
    assert p[0] == pytest.approx(0.7 * np.cos(np.radians(2)))
    assert p[1] == pytest.approx(-0.7 * np.sin(np.radians(2)))
    assert p[2] == pytest.approx(0.6)
    assert "turns 2 deg clockwise about the robot's centre" in po.UgvOffset(0, 0, 2).words()


def test_the_file_is_written_back_as_it_was_read(five_point):
    text, md, rows = five_point
    assert md.shape == (8099, 7)
    assert po.write_program(rows, md) == text


@pytest.mark.parametrize("text, said", [
    ('{"point":[1,2,3,4,5,6,7]}\nnot json\n', "Line 2 is not a Program line"),
    ('{"point":[1,2,3,4,5,6,7]}\n{"point":[1,2,3]}\n', "Line 2 has no seven-joint"),
    ('{"point":[1,2,3,4,5,6,7]}\n', "fewer than two"),
])
def test_files_that_are_not_programs_are_refused_with_the_line(text, said):
    with pytest.raises(po.ProgramError, match=said):
        po.read_program(text)


# ─────────────────────────────────────────────────────────────── the IK frame: the vendor's Arm_Tip

VENDOR_FK = vendor_paths.LITE_ROOT / "tests" / "data" / "vendor_arm_tip_fk.json"


@pytest.mark.parametrize("side", ["right", "left"])
def test_the_ik_frame_is_the_vendor_s_arm_tip_on_both_arms_at_every_pole(side):
    """The user, 25 Nov: the IK on the vendor's Arm_Tip, the same on every arm at any pole. The vendor's own FK
    (tests/data/vendor_arm_tip_fk.json) against Lite's Arm_Tip (URDF Link7 moved 2.8 mm back along the tool axis)."""
    rows = json.loads(VENDOR_FK.read_text())["arms"][side]
    base0 = None
    for pole in (0.0, 0.05, 0.15, 0.20, 0.30):
        arm = po.Arm(side, pole)
        for r in rows:
            T = arm.arm_tip(np.array(r["q_deg"]))[0]
            assert np.linalg.norm(T[:3, 3] - r["p_m"]) < 1e-6                      # 0.001 mm
            Rd = T[:3, :3].T @ np.array(r["R"])                                     # small angles: the skew part
            ang = np.degrees(np.arcsin(min(1.0, np.linalg.norm([Rd[2, 1] - Rd[1, 2], Rd[0, 2] - Rd[2, 0],
                                                                Rd[1, 0] - Rd[0, 1]]) / 2.0)))
            assert ang < 1e-3 and np.trace(Rd) > 2.9                               # (and not a half turn)
        if base0 is None:
            base0 = arm.T_base.copy()
        lift = arm.T_base.copy()
        lift[2, 3] -= pole                                                         # the pole only lifts the base
        assert np.allclose(lift, base0, atol=1e-9)
    assert base0[:3, 3] * 1000 == pytest.approx([45.0, -124.0 if side == "right" else 124.0, 648.5])
    assert po.Arm(side, 0.15).reach_m == pytest.approx(0.466)


def test_a_left_arm_lab_program_moves_by_the_offset_too():
    path = vendor_paths.LITE_ROOT / "Resources" / "Programs" / "lab_25_pct_19_nov" / "left" / \
        "63_L_P2_pole200_grip_10.5s_25pct_new_slowstart.txt"
    md, _ = po.read_program(path.read_text())
    md = md[::4]
    off = po.UgvOffset(-20.0, 20.0, 1.5)
    r = po.shift_program(md, "left", 0.200, off)
    rep = r["report"]
    assert rep["verdict"] != "CANNOT" and rep["lines_unsolved"] == 0 and rep["ik_frame"].startswith("Arm_Tip")
    arm = po.Arm("left", 0.200)
    want = off.path_move() @ (arm.T_base @ arm.arm_tip(md / 1000.0))           # base_link
    got = arm.T_base @ arm.arm_tip(r["md"] / 1000.0)
    assert np.linalg.norm(got[:, :3, 3] - want[:, :3, 3], axis=1).max() < 5e-5


def test_the_pole_height_does_not_change_the_moved_joints(five_point):
    _, md, _ = five_point
    off = po.UgvOffset(-20.0, 20.0, 1.5)
    a = po.shift_program(md[::8], "right", 0.050, off)["md"]
    b = po.shift_program(md[::8], "right", 0.250, off)["md"]
    assert (a == b).all()


# ─────────────────────────────────────────────────────────────── the lab's five-point file

def test_no_offset_gives_the_file_back_unchanged(five_point):
    _, md, _ = five_point
    r = po.shift_program(md[:800], "right", 0.150, po.UgvOffset())
    assert (r["md"] == md[:800]).all() and r["report"]["verdict"] == "FOLLOWS"


def test_the_lab_case_moves_every_touch_by_the_offset_and_keeps_the_timing(lab_case):
    rep = lab_case["report"]
    assert rep["verdict"] == "FOLLOWS", rep["reasons"]
    assert rep["lines_unsolved"] == 0 and rep["lines"] == 8099
    assert rep["hand_error_mm"] < 0.05 and rep["hand_error_deg"] < 0.05
    assert len(rep["touches"]) == 6
    for tp in rep["touches"]:
        assert tp["change_mm"] == pytest.approx([20.0, -20.0, 0.0], abs=0.15)
    t, m = rep["taught"], rep["moved"]
    for k in ("speed", "acceleration", "jerk"):
        assert m[k]["share"] <= 1.10 * t[k]["share"]
    assert m["step_deg"] < 1.0 and max(rep["joint_change_deg"]) < 15.0


def test_the_moved_file_is_a_program_of_the_same_length_and_form(five_point, lab_case):
    _, md, rows = five_point
    out = po.write_program(rows, lab_case["md"])
    md2, rows2 = po.read_program(out)
    assert md2.shape == md.shape and rows2[0].keys() == rows[0].keys()
    assert out.splitlines()[0].startswith('{"point":[') and " " not in out.splitlines()[0]


def test_an_offset_past_the_arm_s_reach_cannot_follow(five_point):
    _, md, _ = five_point
    r = po.shift_program(md[::8][:300], "right", 0.150, po.UgvOffset(-250.0, 0.0, 0.0))    # the box 250 mm farther
    rep = r["report"]
    assert rep["verdict"] == "CANNOT" and rep["lines_unsolved"] > 0
    assert "past reach" in rep["first_unsolved"]["why"]


# ─────────────────────────────────────────────────────────────── the tool

def test_the_command_line_keeps_input_output_and_report(tool, tmp_path, capsys):
    small = tmp_path / "five.txt"
    small.write_text("".join(FIVE_POINT.read_text().splitlines(keepends=True)[:600]))
    assert tool.main([str(small), "--side", "right", "--pole", "150", "--x", "-20", "--y", "20", "--out", str(tmp_path)]) == 0
    said = capsys.readouterr().out
    assert said.startswith("FOLLOWS: UGV 20 mm back, 20 mm left")
    run = next((tmp_path).glob("*/program_offset/*_five"))
    assert sorted(p.name for p in run.iterdir()) == ["five.txt", "five_x-20_y+20_yaw+0.txt", "report.json"]
    rep = json.loads((run / "report.json").read_text())
    assert rep["output"] == "five_x-20_y+20_yaw+0.txt" and "picture" not in rep


def test_the_page_s_server_moves_a_program_and_refuses_a_bad_file(tool, tmp_path):
    with pytest.raises(SystemExit, match="Lite's"):
        tool.serve(8766)
    srv = tool.serve(0, str(tmp_path))
    port = srv.server_address[1]
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    try:
        page = urllib.request.urlopen(f"http://127.0.0.1:{port}/").read().decode()
        assert "<title>Program Offset Test</title>" in page and 'id="go"' in page

        def post(body):
            req = urllib.request.Request(f"http://127.0.0.1:{port}/api/move", data=json.dumps(body).encode(),
                                         headers={"Content-Type": "application/json"})
            return json.loads(urllib.request.urlopen(req).read())

        text = "".join(FIVE_POINT.read_text().splitlines(keepends=True)[:400])
        out = post(dict(name="OFF_5P_TES.txt", text=text, side="right", pole_mm=150, x_mm=-20, y_mm=20, yaw_deg=0))
        assert out["report"]["verdict"] == "FOLLOWS" and out["output_name"] == "OFF_5P_TES_x-20_y+20_yaw+0.txt"
        assert len(out["output_text"].splitlines()) == 400 and len(out["picture"]["tip_moved"]) > 10
        with pytest.raises(urllib.error.HTTPError) as e:
            post(dict(name="x.txt", text="hello\nworld\n", side="right", pole_mm=150))
        assert e.value.code == 400 and "not a Program line" in json.loads(e.value.read())["error"]
    finally:
        srv.shutdown()
        srv.server_close()
