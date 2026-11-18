"""The load a moving arm puts on the UGV (motion/base_load.py), against cases with a known answer."""
import numpy as np
import pytest

from alix_studio_lite.motion import base_load

DT = 0.005
POSE = np.array([10.0, -60.0, 20.0, 80.0, -15.0, 40.0, 30.0])


@pytest.fixture(scope="module")
def arm():
    return base_load.ArmLoad("left")


def _rigid_about_joint1(arm, pose_deg):
    """The arm held in one pose, as one rigid body: (mass, centre of mass, inertia about it), in the
    UGV's base frame, and joint 1's axis as (a point on it, its direction)."""
    com, rot = arm.frames(pose_deg[None, :])
    parts = [(s.mass, com[k, 0], rot[k, 0] @ s.inertia @ rot[k, 0].T) for k, s in enumerate(arm.segments)]
    m, c, ine = base_load._lump(parts)
    chain = arm.chains[0]
    t_pre = arm.model.fk_frames(chain, [arm.pole_m, 0.0])[1][-1][1]
    axis = t_pre[:3, :3] @ np.asarray(chain[-1].axis, dtype=float)
    return m, c, ine, t_pre[:3, 3], axis / np.linalg.norm(axis)


def test_moving_mass_is_the_urdfs(arm):
    assert arm.mass == pytest.approx(6.055 + 0.618, abs=0.06)          # links 1 to 7, and what the wrist carries
    assert all(s.mass > 0 for s in arm.segments)


def test_still_arm_puts_no_load_on_the_ugv(arm):
    f, m = arm.wrench(np.repeat(POSE[None, :], 200, axis=0), DT)
    assert np.abs(f).max() < 1e-9 and np.abs(m).max() < 1e-9


def test_one_joint_swinging_matches_the_rigid_body_formulas(arm):
    """Joint 1 alone turns the whole arm as one rigid body about joint 1's axis: the force is
    m (alpha x r + w x (w x r)) and the moment about the axis is the body's inertia about it times alpha."""
    t = np.arange(0.0, 2.0, DT)
    th = np.radians(25.0) * np.sin(2 * np.pi * 0.8 * t)                # joint 1's own motion, rad
    w = np.gradient(th, DT)
    al = np.gradient(w, DT)
    q = np.repeat(POSE[None, :], len(t), axis=0)
    q[:, 0] = POSE[0] + np.degrees(th)
    f, mom = arm.wrench(q, DT)
    inner = slice(20, -20)
    for i in (150, 233, 310):
        m, c, ine, p0, ax = _rigid_about_joint1(arm, q[i])
        r = c - p0
        f_ref = m * (np.cross(al[i] * ax, r) + np.cross(w[i] * ax, np.cross(w[i] * ax, r)))
        assert f[i] == pytest.approx(f_ref, abs=0.02 * np.abs(f[inner]).max())
        r_perp = r - (r @ ax) * ax
        i_axis = float(ax @ ine @ ax) + m * float(r_perp @ r_perp)
        m_axis = (mom[i] - np.cross(p0, f[i])) @ ax                    # the moment taken about the axis
        assert m_axis == pytest.approx(i_axis * al[i], abs=0.02 * np.abs(mom[inner]).max())


def test_slower_by_k_is_lighter_by_k_squared(arm):
    t = np.arange(0.0, 1.5, DT)
    shape = lambda tt: POSE[None, :] + np.outer(np.sin(2 * np.pi * 0.7 * tt), [20, 15, -25, 30, 10, -20, 40])   # noqa: E731
    f1, m1 = arm.wrench(shape(t), DT)
    t2 = np.arange(0.0, 3.0, DT)
    f2, m2 = arm.wrench(shape(t2 / 2.0), DT)
    inner1, inner2 = slice(20, -20), slice(40, -40)
    assert np.linalg.norm(f2, axis=1)[inner2].max() == pytest.approx(np.linalg.norm(f1, axis=1)[inner1].max() / 4, rel=0.02)
    assert np.linalg.norm(m2, axis=1)[inner2].max() == pytest.approx(np.linalg.norm(m1, axis=1)[inner1].max() / 4, rel=0.02)


def test_path_coefficients_give_the_same_load_as_the_timed_trajectory(arm):
    """load = A xdd + B xd^2 along the path, with x(t) a smooth start and stop."""
    x = np.linspace(0.0, 1.0, 801)
    q_x = POSE[None, :] + np.outer(np.sin(np.pi * x), [30, -20, 25, 15, -30, 20, 35]) + np.outer(x, [10, 5, -8, 12, 6, -9, 4])
    a, b = arm.path_coeffs(q_x, x)
    t = np.arange(0.0, 2.0 + 1e-9, DT)
    u = t / 2.0
    xt = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5
    xd, xdd = np.gradient(xt, DT), np.gradient(np.gradient(xt, DT), DT)
    q_t = np.column_stack([np.interp(xt, x, q_x[:, j]) for j in range(7)])
    f, m = arm.wrench(q_t, DT)
    a_t = np.column_stack([np.interp(xt, x, a[:, k]) for k in range(6)])
    b_t = np.column_stack([np.interp(xt, x, b[:, k]) for k in range(6)])
    pred = a_t * xdd[:, None] + b_t * (xd ** 2)[:, None]
    inner = slice(10, -10)
    assert pred[inner, :3] == pytest.approx(f[inner], abs=0.03 * np.abs(f).max())
    assert pred[inner, 3:] == pytest.approx(m[inner], abs=0.03 * np.abs(m).max())


def test_the_right_arm_carries_a_little_more():
    """The model's two arms differ by 24 g beyond the right wrist (6.743 kg against 6.720 kg)."""
    left, right = base_load.ArmLoad("left").mass, base_load.ArmLoad("right").mass
    assert 0.0 < right - left < 0.05


def test_the_ugv_weight_is_the_urdfs_and_gives_the_moment_limit():
    """The user, 18 Nov: "take the weight of the UGV with the mass set in the URDF". The UGV's own links
    (its body's 90 kg and 8.3 kg of suspensions, wheels and cover) on the wheel links' origins, ALIX's
    275 x 221 mm support polygon; the centre of mass's shortest distance to a wheel line times the weight
    is the moment the weight holds, and half of it ("say half", the same day) the limit when a region sets none."""
    st = base_load.ugv_stand()
    assert st.body_kg == 90.0 and abs(st.mass_kg - 98.258) < 0.01
    assert abs(st.weight_n - st.mass_kg * 9.81) < 1e-9
    xs, ys = sorted({round(x, 4) for x, _ in st.wheels}), sorted({round(y, 4) for _, y in st.wheels})
    assert xs == [-0.1375, 0.1375] and ys == [-0.1104, 0.1104]
    assert set(st.levers_m) == {"front", "rear", "left", "right"}
    assert abs(st.levers_m["front"] + st.levers_m["rear"] - 0.275) < 1e-9          # the two add up to the wheelbase
    assert abs(st.levers_m["left"] + st.levers_m["right"] - 0.2208) < 1e-9
    assert st.lever_m == min(st.levers_m.values()) and 0.105 < st.lever_m < 0.1104  # a side wheel line is the nearest
    assert abs(st.tip_moment_nm - st.weight_n * st.lever_m) < 1e-9
    assert round(st.tip_moment_nm, 1) == 105.8 and base_load.UGV_MOMENT_SHARE == 0.5
    assert base_load.weight_moment_limit_nm() == 52.9
    view = st.view()
    assert view["tip_moment_nm"] == 105.8 and view["share"] == 0.5 and view["limit_nm"] == 52.9 and view["lever_mm"] == 110


def test_the_moment_about_each_wheel_line_against_the_weight():
    """tip_shares: the load's moment about a wheel line on the floor, over what the weight holds about that line. A
    pure moment about the base frame reads the same about a parallel line; an up-and-down force adds its own times
    the distance to the line; a sideways force its own times the base frame's height over the floor (the wheel
    links' origins are 60 mm up and ALIX's wheel radius is 85 mm, so 25 mm); a turn about the upright axis nothing."""
    st = base_load.ugv_stand()
    assert abs(st.floor_z - (0.0598 - 0.085)) < 1e-4
    zero = np.zeros((1, 3))
    roll = base_load.tip_shares(zero, np.array([[10.0, 0.0, 0.0]]))
    assert abs(roll["left"][0] - 10.0 / (st.weight_n * st.levers_m["left"])) < 1e-12
    assert abs(roll["right"][0] - 10.0 / (st.weight_n * st.levers_m["right"])) < 1e-12
    assert roll["front"][0] == 0.0 and roll["rear"][0] == 0.0
    pitch = base_load.tip_shares(zero, np.array([[0.0, 10.0, 0.0]]))
    assert pitch["left"][0] == 0.0 and abs(pitch["front"][0] - 10.0 / (st.weight_n * st.levers_m["front"])) < 1e-12
    lift = base_load.tip_shares(np.array([[0.0, 0.0, 20.0]]), zero)             # 20 N upward, at the base frame's origin
    assert abs(lift["left"][0] - 20.0 * 0.1104 / (st.weight_n * st.levers_m["left"])) < 1e-9
    assert abs(lift["front"][0] - 20.0 * 0.1375 / (st.weight_n * st.levers_m["front"])) < 1e-9
    side = base_load.tip_shares(np.array([[30.0, 30.0, 0.0]]), np.array([[0.0, 0.0, 40.0]]))   # sideways force, and a turn
    for name in ("left", "right", "front", "rear"):
        assert abs(side[name][0] - 30.0 * -st.floor_z / (st.weight_n * st.levers_m[name])) < 1e-12
    assert all(v[0] == 0.0 for v in base_load.tip_shares(zero, np.array([[0.0, 0.0, 40.0]])).values())
    assert max(v[0] for v in base_load.tip_shares(zero, np.array([[st.tip_moment_nm, 0.0, 0.0]])).values()) == pytest.approx(1.0)


def test_a_region_without_a_moment_limit_takes_the_weights():
    from alix_studio_lite.server import app, workers                      # noqa: F401  (app first: the route modules' order)

    asked = workers.request_kwargs(dict(optimiser="custom"))
    assert asked["ugv_moment_nm"] == 52.9 and asked["ugv_force_n"] is None
    assert workers.request_kwargs(dict(optimiser="custom", ugv_moment_nm=5.0))["ugv_moment_nm"] == 5.0
