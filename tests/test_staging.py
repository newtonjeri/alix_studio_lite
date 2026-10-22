"""Staging (server/staging.py) and the L3 reading: the pole and the hand every execution sets
first, timed by ALIX's own models and posed by ALIX's FK, and joint acceleration read the way
ALIX reads it (20 Hz band-limited, finding F99c), not as raw differences."""
import base64
from pathlib import Path

import numpy as np
import pytest

from alix_studio_lite import vendor_paths

vendor_paths.activate()

TAUGHT = vendor_paths.LITE_ROOT / "tests/data/recordings/commode_c_taught_paths"


def test_hand_settings_and_stages():
    from alix_studio_lite.server import staging
    from alix_studio_lite.server.jobs import UserError

    assert staging.parse_hand("GRIP") == (0, 0, 0, 0, 150, 1000)
    assert staging.parse_hand("open") == (1000,) * 6
    assert staging.parse_hand("100% open") == (1000, 1000, 1000, 1000, None, None)
    assert staging.parse_hand("50 % open")[:4] == (500, 500, 500, 500)
    assert staging.parse_hand([0, 0, 0, 0, -1, 1000]) == (0, 0, 0, 0, None, 1000)       # -1 holds, as the SDK
    for bad in ([1, 2], [1001] * 6, "fist", [None] * 6):
        with pytest.raises(UserError):
            staging.parse_hand(bad)
    for pole in (-1, 316, "high"):
        with pytest.raises(UserError):
            staging.make_stage(pole, "GRIP")
    st = staging.make_stage(140, "100% open", "test")
    assert staging.describe(st) == "pole 140 mm, fingers 100 % open, thumb held"
    assert staging.stage_from(st.as_dict()) == st
    assert staging.stage_from({"pole_mm": 140}) is None and staging.stage_from(None) is None
    assert staging.same_hand([1000, 990, 1000, 975, 3, 7], st.hand)                    # held channels not judged
    assert not staging.same_hand([1000, 960, 1000, 1000, 0, 0], st.hand)               # 40 counts off


def test_times_follow_alix_models():
    from alix_studio_lite.server import staging

    peripherals, backends = staging._alix()
    assert staging.pole_travel_s(300, 300) == 0.0 and staging.pole_travel_s(140, 144) == 0.0   # within 5 mm
    assert staging.pole_travel_s(300, 140) == pytest.approx(peripherals.lift_travel_s(0.160, up=False))
    assert staging.pole_travel_s(140, 300) == pytest.approx(peripherals.lift_travel_s(0.160, up=True))
    # the same up and down (ALIX's ledger on the pole's fine position, 22 Oct: the slower rise was the whole-mm reading)
    assert staging.pole_travel_s(140, 300) == pytest.approx(staging.pole_travel_s(300, 140))
    assert staging.pole_travel_s(140, 300) == pytest.approx(1.5 + 0.160 / 0.097)
    b = backends.SdkBackend
    assert staging.hand_move_s([1000] * 6, [0, 0, 0, 0, 150, 1000]) == pytest.approx(1000 / b.HAND_COUNTS_PER_S + b.HAND_LATENCY_S)
    assert staging.hand_move_s([1000] * 6, [1000, 1000, 1000, 1000, None, None]) == 0.0


def test_timeline_stages_then_runs(tmp_path):
    """FILE_04 (LEFT, taught at pole 140 mm): the arm base drops 160 mm from the standing pose,
    the hand reaches its counts, the arm reaches line 1, then 5 ms per line to the last line."""
    from alix_studio_lite.motion import track
    from alix_studio_lite.motion.limits import load_limits
    from alix_studio_lite.server import robot_model, staging

    p = robot_model.build_robot_model(tmp_path)
    home = dict(p["home"])
    links = staging.staged_links("left", [ln for ln, ids in p["links"].items() if ids])
    assert links and not any(ln.startswith("R_") for ln in links)
    md = track.read_md(TAUGHT / "FILE_04.txt")
    st = staging.make_stage(140, "GRIP", "test")
    tl = staging.execution_timeline("left", md, st, home=home, links=links, rating_deg_s=load_limits("left").rating_deg_s)
    t = tl["times"]
    assert [ph["name"] for ph in tl["phases"]] == ["pole", "hand", "move", "trajectory"]
    assert [ph["text"] for ph in tl["phases"][:2]] == ["Pole 300 → 140 mm", "Hand → grip"]
    assert t == sorted(t) and t[0] == 0.0 and tl["traj_t0"] in t
    assert tl["total_s"] == pytest.approx(tl["traj_t0"] + (md.shape[0] - 1) * 0.005)
    assert tl["phases"][0]["t1"] == pytest.approx(staging.pole_travel_s(300, 140), abs=1e-3)
    f = np.frombuffer(base64.b64decode(tl["frames"]), dtype="<f4").reshape(len(t), len(links), 16)
    base = links.index(next(ln for ln in links if "base" in ln.lower()))
    k = t.index(tl["traj_t0"])
    assert f[0, base, 11] - f[k, base, 11] == pytest.approx(0.160, abs=1e-4)          # 300 -> 140 mm
    assert np.allclose(f[k:, base], f[k, base])                                       # the pole holds while it runs

    # line 1 and the last line are where FK puts them with the pole and hand staged
    model = robot_model.fk_model()
    q = np.radians(md / 1000.0)
    for line, frame in ((0, k), (md.shape[0] - 1, len(t) - 1)):
        values = dict(home) | staging.joint_values("left", 140, st.hand)
        values.update({f"L_joint{j + 1}": float(q[line, j]) for j in range(7)})
        tf = model.link_world_transforms(values)
        for i, ln in enumerate(links):
            assert np.allclose(f[frame, i], tf[ln].reshape(-1), atol=1e-5), (line, ln)


def test_l3_reads_as_alix_does():
    """The same number ALIX's band_limited_util gives; raw differences read recorder jitter."""
    import plan_min_time as pmt

    from alix_studio_lite.motion import track
    from alix_studio_lite.motion.checks import acc_deg_s2, check_bytes
    from alix_studio_lite.motion.limits import load_limits

    q = track.read_md(TAUGHT / "FILE_04.txt") / 1000.0
    ones = np.ones(7)
    _, a_util, _ = pmt.band_limited_util(q, 0.005, 180 * ones, 600 * ones, 1.0, 20.0)
    assert float(acc_deg_s2(q, 0.005).max()) == pytest.approx(a_util * 600, rel=1e-9)
    raw = float((np.abs(np.diff(q, n=2, axis=0)) / 0.005 ** 2).max())
    assert raw > 10 * a_util * 600                                                    # jitter, not motion

    # a smooth 1 s move peaking at 400 deg/s^2, plus 3 millidegrees of recorder jitter: raw
    # differences read it above the limit; band-limited, L3 reads the motion and passes
    T, a_max = 1.0, 400.0
    s = np.linspace(0, 1, int(T / 0.005) + 1)
    D = a_max * T ** 2 / (2 * np.pi)                              # a cycloid peaks at 2 pi D / T^2
    md = np.zeros((s.size, 7), dtype=np.int64)
    md[:, 0] = np.round(D * (s - np.sin(2 * np.pi * s) / (2 * np.pi)) * 1000).astype(np.int64) + 3 * (np.arange(s.size) % 2)
    q = md / 1000.0
    assert (np.abs(np.diff(q, n=2, axis=0)) / 0.005 ** 2).max() > 612
    assert float(acc_deg_s2(q, 0.005).max()) == pytest.approx(a_max, rel=0.03)
    l3 = next(c for c in check_bytes(md, load_limits("left")).to_dict()["checks"] if c["id"] == "L3")
    assert l3["passed"] is True, l3


def test_a_hand_that_settles_on_the_tool_is_done():
    """The control program's rule (vendor/control_app/hand.py, Hand.command): ARRIVED or SETTLED
    is done, "fingers closing on a tool stop early on force". The robot, 14 Oct: Grip settled at
    842, 762, 754, 765, 714, 992; then the setup check takes the hand while it still reads that."""
    from pathlib import Path

    from alix_studio_lite.server.jobs import Jobs
    from alix_studio_lite.server.robot_real import _settled
    from alix_studio_lite.server.robot_standin import StandInRobot
    from alix_studio_lite.server.staging import make_stage

    settled = {"ok": False, "refusal": "hand_unverified",
               "steps": [{"step": "hand", "result": {"ok": True, "arrived": False, "settled": True}}]}
    timed_out = {"ok": False, "refusal": "hand_unverified",
                 "steps": [{"step": "hand", "result": {"ok": False, "arrived": False, "settled": False}}]}
    assert _settled(settled) and not _settled(timed_out)

    import tempfile
    robot = StandInRobot(Path(tempfile.mkdtemp()), Jobs())
    grip = make_stage(200, "GRIP")
    a = robot.arm("left")
    a.pole_mm, a.hand = 200.0, [842, 762, 754, 765, 714, 992]
    assert not robot.in_stage("left", grip)                              # never staged: not the grip
    a.staged_hand = dict(target=list(grip.hand), read=list(a.hand))      # staging the grip left it there
    assert robot.in_stage("left", grip)
    a.hand = [1000] * 6                                                  # opened since: not in the setup
    assert not robot.in_stage("left", grip)
