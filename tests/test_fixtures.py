"""The fixture pool (D29): every fixture's parts are posed with ALIX's own scene_pose, including the
venue's yaw 180 that ALIX composes into every part (without it a commode faces the wrong way)."""
import numpy as np
import pytest

from alix_studio_lite.server import fixtures as F


def _m(part, phase="lid_closed"):
    return np.array(part["matrices"][phase]).reshape(4, 4)


def _yaw_deg(M):
    return float(np.degrees(np.arctan2(M[1, 0], M[0, 0])))


@pytest.mark.parametrize("fid", list(F.FIXTURES))
def test_every_part_has_a_mesh_and_poses(fid):
    s = F.spec(fid)
    assert s["parts"]
    for p in s["parts"]:
        urls = [p["mesh"]] if p.get("mesh") else []
        urls += list((p.get("state_meshes") or {}).values())
        assert urls, p["name"]
        for u in urls:
            assert F.mesh_path(fid, u.rsplit("/", 1)[1]) is not None, u
        for phase in ("lid_closed", "lid_open", "seat_open"):
            M = _m(p, phase)
            assert np.allclose(M[3], [0, 0, 0, 1]) and np.allclose(M[:3, :3] @ M[:3, :3].T, np.eye(3), atol=1e-6)


def test_commode_c_faces_the_robot_and_its_seat_hinge_is_where_alix_measured_it():
    parts = {p["name"]: p for p in F.spec("commode_c")["parts"]}
    assert abs(abs(_yaw_deg(_m(parts["body"]))) - 180.0) < 1e-6        # the venue's yaw 180 is composed in
    assert abs(_m(parts["seat"])[0, 3] - 0.890) < 0.005                  # body_collision.py: live hinge at x=0.890
    assert not np.allclose(_m(parts["lid"], "lid_closed"), _m(parts["lid"], "lid_open"))
    assert np.allclose(_m(parts["seat"], "lid_open"), _m(parts["seat"], "lid_closed"))
    assert not np.allclose(_m(parts["seat"], "seat_open"), _m(parts["seat"], "lid_closed"))


def test_commode_a_uses_one_mesh_per_state():
    (body,) = F.spec("commode_a")["parts"]
    assert set(body["state_meshes"]) == {"lid_closed", "lid_open", "seat_open"}
    assert abs(_yaw_deg(_m(body))) < 1e-6                                # its own yaw 180 and the venue's cancel
    assert body["scale"] == [0.001, 0.001, 0.001]


@pytest.mark.parametrize("fid", list(F.FIXTURES))
def test_catalogues(fid):
    c = F.catalogue(fid)
    keys = [u["key"] for u in c["units"]]
    assert keys and len(keys) == len(set(keys))
    assert all(u["arm"] in ("left", "right") for u in c["units"])
    if fid != "commode_c":
        assert all(u["provisional"] for u in c["units"])
