"""The three automatic optimisers (Time-optimal, Ruckig on s, Lite's own) and their two settings: the
jerk limit and the load the arm's motion puts on the UGV. One short recording of commode C
(FILE_10.1, the left arm), each optimiser run once per setting and shared by the tests."""
import numpy as np
import pytest

pytest.importorskip("toppra")

from alix_studio_lite.motion import track                                    # noqa: E402
from alix_studio_lite.motion.checks import check_bytes, jerk_deg_s3, motion_ok, ugv_load   # noqa: E402
from alix_studio_lite.motion.limits import load_limits, yaml_joint_limits    # noqa: E402
from alix_studio_lite.optimise import OptRequest, ToppraOptions, optimise    # noqa: E402

JERK = 300.0                                    # rad/s^3: well under the joint limits file's 1500
OPTIMISERS = ("toppra", "ruckig_s", "custom")


@pytest.fixture(scope="module")
def recording(commode_c_recordings):
    return track.read_md(commode_c_recordings / "FILE_10.1.txt")


@pytest.fixture(scope="module")
def made(recording):
    """{(optimiser, setting): result}, each made once."""
    kept = {}

    def get(optimiser: str, **settings):
        if optimiser == "ruckig_s":
            pytest.importorskip("ruckig")
        key = (optimiser, tuple(sorted(settings.items())))
        if key not in kept:
            kept[key] = optimise(OptRequest(md_raw=recording, side="left", optimiser=optimiser, **settings))
        return kept[key]

    return get


def test_joint_limits_file_is_what_lite_holds():
    """The constraints are ALIX's joint_limits.yaml: its speeds, acceleration and jerk are the ones
    Lite's limit set carries, for both arms."""
    for side in ("left", "right"):
        from_file, lim = yaml_joint_limits(side), load_limits(side)
        assert from_file["speed_deg_s"] == pytest.approx(lim.rating_deg_s, abs=1e-6)
        assert from_file["acc_deg_s2"] == pytest.approx([lim.acc_deg_s2] * 7, rel=1e-4)
        assert from_file["jerk_rad_s3"] == pytest.approx([lim.jerk_ref_rad_s3] * 7)


def test_jerk_and_ugv_load_are_limits_only_when_the_limit_set_carries_them():
    t = np.arange(0.0, 3.0, 0.005)
    q = np.array([10.0, -60.0, 20.0, 80.0, -15.0, 40.0, 30.0])[None, :] + np.outer(
        0.5 - 0.5 * np.cos(2 * np.pi * t / 3.0), [40, 30, -35, 45, 20, -30, 50])
    md = track.to_md(q)
    plain = load_limits("left")
    rep = check_bytes(md, plain, with_departure=False)
    assert rep.get("L7") is None and rep.get("L8") is None and motion_ok(md, plain)

    peak = float(np.radians(jerk_deg_s3(q, 0.005).max()))
    force, moment = ugv_load(q, plain)
    loose = plain.with_auto(jerk_rad_s3=2 * peak, ugv_force_n=2 * force.max(), ugv_moment_nm=2 * moment.max())
    rep = check_bytes(md, loose, with_departure=False)
    assert rep.get("L7").passed and rep.get("L8").passed and motion_ok(md, loose)

    for tight in (plain.with_auto(jerk_rad_s3=0.5 * peak), plain.with_auto(ugv_force_n=0.5 * force.max()),
                  plain.with_auto(ugv_moment_nm=0.5 * moment.max())):
        rep = check_bytes(md, tight, with_departure=False)
        assert not rep.passed and not motion_ok(md, tight)
        failed = rep.failures()[0]
        assert failed.id in ("L7", "L8") and failed.speed_fixes is True


def test_the_post_filter_keeps_both_ends_and_calms_the_file(recording):
    from alix_studio_lite.optimise import toppra_opt

    lim = load_limits("left")
    plain = toppra_opt.ToppraPlan(recording, lim, ToppraOptions(post_lp_hz=0.0, arrival_hold=0.0))
    calm = toppra_opt.ToppraPlan(recording, lim, ToppraOptions(post_lp_hz=6.0, arrival_hold=0.0))
    a, b = track.to_deg(plain.make(plain.n_floor)), track.to_deg(calm.make(calm.n_floor))
    assert a.shape == b.shape
    assert np.abs(b[0] - a[0]).max() <= 0.001 and np.abs(b[-1] - a[-1]).max() <= 0.001
    assert jerk_deg_s3(b, 0.005).max() < 0.8 * jerk_deg_s3(a, 0.005).max()


@pytest.mark.parametrize("optimiser", OPTIMISERS)
def test_each_optimiser_makes_a_file_within_every_limit(made, recording, optimiser):
    res = made(optimiser, jerk_rad_s3=JERK)
    assert res.uploadable, res.message
    assert res.optimiser == optimiser and res.recipe["limits"]["jerk_max_rad_s3"] == JERK
    for check in ("L1", "L2", "L3", "L4", "L5", "L6", "L7"):
        assert res.checks.get(check).passed, check
    assert res.checks.get("L7").value <= 1.0
    assert res.achieved_s < len(recording) * 0.005                 # faster than it was taught
    lim = load_limits("left").with_auto(jerk_rad_s3=JERK)
    assert motion_ok(res.md, lim)


def test_a_tighter_jerk_limit_costs_the_new_optimisers_less_than_slowing_the_whole_file(made):
    """Time-optimal can only meet a jerk limit by slowing the whole file; the other two slow the
    arm where the jerk is."""
    slowed = made("toppra", jerk_rad_s3=JERK).achieved_s
    assert made("custom", jerk_rad_s3=JERK).achieved_s < slowed
    assert made("ruckig_s", jerk_rad_s3=JERK).achieved_s < slowed


@pytest.mark.parametrize("optimiser", OPTIMISERS)
def test_a_ugv_load_limit_is_held(made, optimiser):
    free = made(optimiser, jerk_rad_s3=JERK)
    lim = load_limits("left")
    force, moment = ugv_load(track.to_deg(free.md), lim)
    f_lim, m_lim = round(0.6 * float(force.max()), 2), round(0.6 * float(moment.max()), 2)
    held = made(optimiser, jerk_rad_s3=JERK, ugv_force_n=f_lim, ugv_moment_nm=m_lim)
    assert held.uploadable, held.message
    assert held.checks.get("L8").passed
    force, moment = ugv_load(track.to_deg(held.md), lim)
    assert force.max() <= f_lim and moment.max() <= m_lim
    assert held.achieved_s > free.achieved_s


@pytest.mark.parametrize("optimiser", ("ruckig_s", "custom"))
def test_the_servers_worker_makes_the_file_the_region_asked_for(tmp_path, commode_c_recordings, optimiser):
    """The function the server's Confirm & Generate runs, with a region's settings as the page saves them."""
    if optimiser == "ruckig_s":
        pytest.importorskip("ruckig")
    from alix_studio_lite.server import app, workers          # noqa: F401  (app first: the route modules' order)

    out = tmp_path / "candidate.txt"
    got = workers.optimise_unit(str(commode_c_recordings / "FILE_10.1.txt"), str(out), "left",
                                dict(optimiser=optimiser, jerk_rad_s3=JERK, ugv_force_n=9.0, ugv_moment_nm=None))
    assert "error" not in got, got
    assert out.is_file() and got["summary"]["optimiser"] == optimiser and got["summary"]["uploadable"]
    checks = {c["id"]: c for c in got["checks"]["checks"]}
    assert checks["L7"]["passed"] and checks["L8"]["passed"]
    assert got["recipe"]["limits"]["jerk_max_rad_s3"] == JERK and got["recipe"]["limits"]["ugv_force_n"] == 9.0
    assert track.read_md(out).shape[0] == got["summary"]["lines"]


def test_an_optimiser_this_computer_lacks_is_said_plainly(tmp_path, commode_c_recordings, monkeypatch):
    from alix_studio_lite.server import app, workers          # noqa: F401

    monkeypatch.setattr(workers, "optimiser_missing", lambda name: "ruckig")
    got = workers.optimise_unit(str(commode_c_recordings / "FILE_10.1.txt"), str(tmp_path / "c.txt"), "left",
                                dict(optimiser="ruckig_s"))
    assert got["error"] == "Ruckig on s is not available on this computer." and "install ruckig" in got["hint"]
    assert not (tmp_path / "c.txt").exists()


def test_a_regions_settings_choose_the_optimiser_and_carry_the_two_settings():
    from alix_studio_lite.server import app, workers          # app first: it loads the route modules in their order
    from alix_studio_lite.server.api_units import _check_settings

    UserError = app.UserError

    assert workers.is_manual({}) and workers.is_manual({"optimiser": "manual"}) and workers.is_manual({"optimiser": "default"})
    for name in OPTIMISERS:
        assert not workers.is_manual({"optimiser": name})
        kw = workers.request_kwargs({"optimiser": name})
        assert kw["optimiser"] == name and kw["jerk_rad_s3"] == 1500.0 and kw["ugv_force_n"] is None
    kw = workers.request_kwargs({"optimiser": "custom", "jerk_rad_s3": 250, "ugv_force_n": 9, "ugv_moment_nm": 6})
    assert (kw["jerk_rad_s3"], kw["ugv_force_n"], kw["ugv_moment_nm"]) == (250, 9, 6)

    ok = _check_settings({"optimiser": "ruckig_s", "jerk_rad_s3": "250", "ugv_force_n": "", "ugv_moment_nm": 6})
    assert ok == {"optimiser": "ruckig_s", "jerk_rad_s3": 250.0, "ugv_force_n": None, "ugv_moment_nm": 6.0}
    for bad in ({"optimiser": "fastest"}, {"jerk_rad_s3": 2000}, {"jerk_rad_s3": 1}, {"ugv_force_n": 0}):
        with pytest.raises(UserError):
            _check_settings(bad)
