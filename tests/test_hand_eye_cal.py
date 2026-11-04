"""The wrist camera's calibration (server/hand_eye_cal.py; the user, 4 Nov): the views Lite plans around rest,
inspection pose 1 and inspection pose 4 in the lab's measured room, a session Lite writes solved by ALIX's own solver
(a planted mount error recovered from board images rendered through the planned views), and the API's refusals."""
import json
import threading
import urllib.request
from types import SimpleNamespace

import numpy as np
import pytest

from alix_studio_lite.server import contact_sim, hand_eye_cal as hc, placement

ROOT = __import__("pathlib").Path(__file__).resolve().parents[1]
LAB = SimpleNamespace(root=str(ROOT / "Resources/Projects/Commode_C_17_10_2026"), fixture="commode_c")
LEFT_REST = [0.0, -82.0, 26.0, 98.0, -6.0, 62.0, 111.0]


@pytest.fixture(scope="module")
def lab_plan():
    room = dict(home=contact_sim.default_home(), scene=placement.scene("commode_c", placement.load(LAB)["room"], None),
                fid="commode_c", cubicle_mm=1400)
    poses = hc.named_poses()
    state = {"right": {"q_deg": poses["rest_pose"], "pole_mm": hc.POLE_MM, "hand": [1000] * 6},
             "left": {"q_deg": LEFT_REST, "pole_mm": 150.0, "hand": [1000] * 6}}
    return hc.plan(state, room), room, state


def test_the_three_poses_give_fifteen_views_clear_of_the_lab_room(lab_plan):
    planned, room, state = lab_plan
    views = planned["views"]
    assert len(views) == 15
    assert {b: sum(v["base"] == b for v in views) for b in hc.BASES} == {b: 5 for b in hc.BASES}
    assert planned["diversity"]["enough"]                          # RealMan: over 30 deg about each END axis
    assert planned["slack_m"] == pytest.approx(0.02)                # the 300 x 210 mm board: within 20 mm of the lid's middle
    for v in views:                                                 # each one: the board whole in view (any turn), off the lid
        L = hc.links_at(v["q_deg"], hc.POLE_MM, state["left"])
        assert hc.view_of(L[hc.COLOUR_FRAME], planned["board"], planned["slack_m"])["ok"], v
        assert hc.above_lid(L)[0], v
    assert any(w["name"] == "calibration board" for w in planned["scene"]["walls"])      # the board is in the scene
    legs = hc.check_paths(views, state["right"]["q_deg"], planned["ends_at"], state, dict(room, scene=planned["scene"]))
    assert all(g["ok"] for g in legs), [g for g in legs if not g["ok"]]


def test_the_collision_gate_refuses_a_hand_on_the_commode(lab_plan):
    _, room, state = lab_plan
    from alix_studio_lite.server import mechanism, plan_shift
    from alix_studio_lite.server.contact_preview import _fk_ref, _ik

    e = next(e for e in mechanism.Plan("seat_close").entries if e.kind == "arm" and e.side == "right" and e.stage == "execute_path")
    q0 = (np.asarray(e.rows, float) / 1000.0)[59]
    frame = plan_shift.ik_frames("seat_close")["right"]
    T = _fk_ref("right", frame, 0.15)(np.asarray([q0]))[0]
    T[2, 3] -= 0.15
    T[1, 3] += 0.20
    q, ok, *_ = _ik("right", frame, 0.15)(T, q0)
    assert ok
    st = dict(state, right=dict(state["right"], pole_mm=150.0))
    old = hc.POLE_MM
    hc.POLE_MM = 150.0
    try:
        legs = hc.check_paths([dict(q_deg=np.asarray(q).tolist())], q0.tolist(), q0.tolist(), st, room)
    finally:
        hc.POLE_MM = old
    assert not legs[0]["ok"] and "commode" in legs[0]["why"]


# ── a session Lite writes, solved by ALIX ────────────────────────────────

def _texture(board, sq=40):
    tex = np.full(((board.rows + 3) * sq, (board.cols + 3) * sq), 255, np.uint8)
    for r in range(board.rows + 1):
        for c in range(board.cols + 1):
            if (r + c) % 2 == 0:
                tex[(r + 1) * sq:(r + 2) * sq, (c + 1) * sq:(c + 2) * sq] = 0
    m = board.square_m / sq
    return tex, np.array([[m, 0, -2 * sq * m], [0, m, -2 * sq * m], [0, 0, 1.0]])


def test_alix_solves_a_session_lite_writes(lab_plan, tmp_path):
    import cv2

    from alix_perception_core import hand_eye as he

    planned, _, state = lab_plan
    board = he.Board(hc.BOARD_COLS, hc.BOARD_ROWS, hc.BOARD_SQUARE_M)
    nominal = hc.nominal_frames()
    T_true = nominal["T_end_cam"] @ he.make_T(he.from_rotvec(np.radians([-1.0, 0.8, 1.5])), [0.004, -0.003, 0.002])
    # the board face-up on the lid (its z down, away from the camera: ALIX's tests' convention), turned 35 deg and laid
    # 15 mm off the lid's middle (the user's photo, 4 Nov: its placement is not exact): every planned view still sees it
    R = he.matrix_from_rpy(0.0, 0.0, np.radians(35.0)) @ he.from_rotvec([np.pi, 0.0, 0.0])
    half = np.array([(board.cols - 1) / 2 * board.square_m, (board.rows - 1) / 2 * board.square_m, 0.0])
    T_base_board = he.make_T(R, np.asarray(hc.BOARD_CENTRE_M) + np.array([0.012, -0.009, 0.0]) - R @ half)
    K = hc.nominal_K()
    tex, A = _texture(board)
    s = hc.Session(tmp_path / "hand_eye_test", planned)
    s.header = dict(created="test", board=dict(cols=board.cols, rows=board.rows, square_m=board.square_m),
                    end_frame=hc.END_FRAME, camera_frame=hc.COLOUR_FRAME, base_frame="base_link",
                    camera_info=dict(width=hc.IMAGE_W, height=hc.IMAGE_H, K=K.ravel().tolist(), D=[0.0] * 5), nominal=nominal)
    for k, v in enumerate(planned["views"]):
        L = hc.links_at(v["q_deg"], hc.POLE_MM, state["left"])
        T_cam_board = np.linalg.inv(L[hc.END_FRAME] @ T_true) @ T_base_board
        H = K @ T_cam_board[:3, [0, 1, 3]] @ A
        img = cv2.cvtColor(cv2.warpPerspective(tex, H, (hc.IMAGE_W, hc.IMAGE_H), borderValue=170), cv2.COLOR_GRAY2BGR)
        corners = he.detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), board)
        assert corners is not None, f"view {k} ({v['base']}): the rendered board was not found"
        hc.record(s, k, v, img, corners, 0.0, v["q_deg"], hc.POLE_MM)
    assert (s.folder / "samples.yaml").exists() and len(list((s.folder / "images").glob("*.png"))) == 15
    out = hc.solve(s)
    assert out["accepted"], out
    T = np.asarray(s.result["runs"]["factory"]["refined"]["T_end_cam"], float)
    assert np.linalg.norm(T[:3, 3] - T_true[:3, 3]) * 1000 < 1.0
    assert he.rot_angle_deg(T_true[:3, :3].T @ T[:3, :3]) < 0.1
    assert (s.folder / "hand_eye_result.yaml").exists()
    assert out["translation_norm_mm"] == pytest.approx(np.linalg.norm([4, -3, 2]), abs=1.0)


# ── the API ──────────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-cal")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
    yield {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app}
    srv.shutdown()


def _req(lite, method, path, body=None, token=None):
    headers = {"Content-Type": "application/json", **({"X-Lite-Token": token} if token else {})}
    r = urllib.request.Request(lite["base"] + path, data=None if body is None else json.dumps(body).encode(), method=method,
                               headers=headers)
    with urllib.request.urlopen(r, timeout=60) as resp:
        return json.loads(resp.read())


def test_the_api_plans_and_refuses_what_it_cannot_do(lite):
    tok = _req(lite, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    got = _req(lite, "GET", "/api/calibration", token=tok)
    assert got["session"] is None and got["pole_mm"] == 250.0 and got["speed_pct"] == 40 and got["board"]["squares"] == [10, 7]
    assert "No calibration session" in _req(lite, "POST", "/api/calibration/run", {}, tok)["error"]
    assert "Open the project" in _req(lite, "POST", "/api/calibration/plan", {"square_mm": 30.0}, tok)["error"]
    assert "not a calibration board" in _req(lite, "POST", "/api/calibration/plan", {"pid": "x", "square_mm": 5}, tok)["error"]
    assert "one count must be even" in _req(lite, "POST", "/api/calibration/plan", {"pid": "x", "squares": [10, 8]}, tok)["error"]
    page = {"id": "alix_cal1", "name": "Cal", "createdAt": "2026-11-04T10:00:00.000Z", "updatedAt": "2026-11-04T10:00:00.000Z",
            "archived": False, "state": {"currentTask": 0, "currentView": "setup"}}
    _req(lite, "PUT", "/api/projects/alix_cal1", page, tok)
    planned = _req(lite, "POST", "/api/calibration/plan", {"pid": "alix_cal1", "squares": [12, 9], "square_mm": 20.01}, tok)
    assert planned["session"]["views"] == 15 and planned["session"]["board"] == dict(squares=[12, 9], square_mm=20.01, tolerance_mm=40)
    tl = _req(lite, "POST", "/api/calibration/preview", {}, tok)  # the tour in 3D, nothing sent
    assert tl["frames"] and "15 views at 40 %" in tl["what"]
    r = _req(lite, "POST", "/api/calibration/run", {}, tok)       # Visual sim: Lite does not use the robot
    assert r["code"] == "mode"
    assert "Only 0 view" in _req(lite, "POST", "/api/calibration/solve", {}, tok)["error"]


# ── the run: one job, every view, back to rest, the solve ─────────────────

class _Job:
    def __init__(self):
        self.logs = []

    def progress(self, *a, **k):
        pass

    def log(self, text, level="info"):
        self.logs.append(text)

    def check(self):
        pass

    def sleep(self, s):
        pass


class _Arm:
    """The right arm's controller: the MoveJs at the speed asked, the joints where the last one went."""
    def __init__(self, link):
        self.link, self.moves, self.modes = link, [], []

    def set_run_mode(self, mode):
        self.modes.append(mode)
        return {}

    def restore_run_mode(self, originals):
        pass

    def move_to(self, target, speed, restore_pct):
        self.moves.append((list(target), speed))
        self.link.q = [v / 1000.0 for v in target]
        return dict(gap_deg=0.0)

    def pole_to(self, mm):
        self.link.pole = mm


class _Link:
    def __init__(self, q, pole=150.0):
        self.q, self.pole = list(q), pole

    def read_fresh(self):
        return dict(joints_deg=list(self.q), pole_mm=self.pole, hand=[1000] * 6)


def _robot(state):
    links = {"right": _Link(state["right"]["q_deg"], 150.0), "left": _Link(LEFT_REST)}
    arm = _Arm(links["right"])
    robot = SimpleNamespace(links=links, _same_mode=lambda *a: None, _real=lambda side: arm, found_speed=lambda side: 50,
                            _measured=lambda side: (links[side].pole, [1000] * 6),
                            _start=lambda kind, title, work, side, mode: SimpleNamespace(result=work(_Job()), title=title))
    return robot, arm


def test_one_run_takes_every_view_at_40_percent_and_solves(lab_plan, monkeypatch, tmp_path):
    import contextlib

    import cv2
    from alix_perception_core import hand_eye as he

    from alix_studio_lite.server import workers
    from alix_studio_lite.server.gap_check import Intrinsics

    planned, room, state = lab_plan
    monkeypatch.setattr(workers, "pool", lambda *a, **k: SimpleNamespace(submit=lambda fn, *a: SimpleNamespace(result=lambda timeout=None: fn(*a))))
    robot, arm = _robot(state)
    board = he.Board(hc.BOARD_COLS, hc.BOARD_ROWS, hc.BOARD_SQUARE_M)
    T_true = hc.nominal_frames()["T_end_cam"] @ he.make_T(he.from_rotvec(np.radians([0.6, -1.1, 0.4])), [-0.003, 0.002, 0.004])
    R = he.from_rotvec([np.pi, 0.0, 0.0])
    half = np.array([(board.cols - 1) / 2 * board.square_m, (board.rows - 1) / 2 * board.square_m, 0.0])
    T_base_board = he.make_T(R, np.asarray(hc.BOARD_CENTRE_M) - R @ half)
    K = hc.nominal_K()
    tex, A = _texture(board)

    class Cam:                                  # renders the board from wherever the fake arm stands
        _cintr = Intrinsics(K[0, 0], K[1, 1], K[0, 2], K[1, 2], hc.IMAGE_W, hc.IMAGE_H)
        colour_distortion, device = None, dict(name="rendered")

        def available(self):
            return dict(ok=True)

        def using(self):
            return contextlib.nullcontext()

        def open(self):
            pass

        def shot(self):
            L = hc.links_at(robot.links["right"].q, robot.links["right"].pole, None)
            T_cam_board = np.linalg.inv(L[hc.END_FRAME] @ T_true) @ T_base_board
            img = cv2.warpPerspective(tex, K @ T_cam_board[:3, [0, 1, 3]] @ A, (hc.IMAGE_W, hc.IMAGE_H), borderValue=170)
            return SimpleNamespace(colour=cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))

    monkeypatch.setattr(hc, "camera_of", lambda robot: Cam())
    s = hc.Session(tmp_path / "run", planned, room)
    job = hc.run_session(robot, s, "real")
    out = job.result
    assert out["taken"] == 15 and not out["missed"] and out["result"]["accepted"], out
    assert job.title.endswith("at 40 %")
    assert [sp for _, sp in arm.moves] == [40] * 16                          # 15 views and the way back, all at 40 %
    assert arm.moves[-1][0] == [round(v * 1000) for v in planned["ends_at"]] and set(arm.modes) == {"REAL"}
    assert robot.links["right"].pole == hc.POLE_MM                          # the pole to 250 mm before the first move
    T = np.asarray(s.result["runs"]["factory"]["refined"]["T_end_cam"], float)
    assert np.linalg.norm(T[:3, 3] - T_true[:3, 3]) * 1000 < 1.0


def test_hardware_sim_runs_the_same_moves_in_sim_mode_and_keeps_nothing(lab_plan, monkeypatch, tmp_path):
    from alix_studio_lite.server import workers

    planned, room, state = lab_plan
    monkeypatch.setattr(workers, "pool", lambda *a, **k: SimpleNamespace(submit=lambda fn, *a: SimpleNamespace(result=lambda timeout=None: fn(*a))))
    robot, arm = _robot(state)
    s = hc.Session(tmp_path / "dry", planned, room)
    out = hc.run_session(robot, s, "hwsim").result
    assert out["reached"] == 15 and set(arm.modes) == {"SIM"} and len(arm.moves) == 16
    assert robot.links["right"].pole == 150.0                               # the pole is not moved in Hardware sim
    assert s.next_index == 0 and not s.rows                                 # the real run still has every view to take


def test_each_move_is_checked_again_from_where_the_arms_stand(lab_plan, monkeypatch, tmp_path):
    """leg_clear: the move about to be sent, from the right arm's joints read now (the left's too), refused when it
    is not clear (here the arm swung up into the room's side wall); nothing is sent then."""
    from alix_studio_lite.server import jobs, workers

    planned, room, state = lab_plan
    monkeypatch.setattr(workers, "pool", lambda *a, **k: SimpleNamespace(submit=lambda fn, *a: SimpleNamespace(result=lambda timeout=None: fn(*a))))
    robot, _ = _robot(state)
    s = hc.Session(tmp_path / "s", planned, room)
    hc.leg_clear(robot, s, planned["views"][0]["q_deg"], "view 1")          # the first planned move: clear
    with pytest.raises(jobs.UserError, match="Not moved"):
        hc.leg_clear(robot, s, [0.0, 10.0, 0.0, 10.0, 0.0, 10.0, 0.0], "a pose up against the side wall")
