"""The robot the 3D view draws (server/robot_model.py, web/js/robot3d.js), built the way ALIX Studio
builds its overlay: baked links come from Lite's bake (ALIX's own bake tool), any other link at full
resolution from the URDF's visual meshes; every mesh decodes, the cache is reused, and every pose is
ALIX's FK exactly."""
import base64
import json
import math
import time
from types import SimpleNamespace

import numpy as np
import pytest

from alix_studio_lite import vendor_paths as vp


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    from alix_studio_lite.server.robot_model import build_robot_model

    data = tmp_path_factory.mktemp("lite-data")
    t0 = time.time()
    payload = build_robot_model(data / "cache")
    return data, payload, time.time() - t0


@pytest.fixture(scope="module")
def urdf():
    from alix_workspace.urdf_kinematics import UrdfModel

    return UrdfModel.from_file(str(vp.URDF))


def _arrays(m):
    return (np.frombuffer(base64.b64decode(m["positions"]), "<f4").reshape(-1, 3),
            np.frombuffer(base64.b64decode(m["indices"]), "<u4").reshape(-1, 3))


def _by_source(p):
    out = {"bake": set(), "live": set()}
    for link, ids in p["links"].items():
        for i in ids:
            out[p["meshes"][i]["source"]].add(link)
    return out


@pytest.mark.slow
def test_payload_builds_from_vendor(built, urdf):
    _, p, _ = built
    assert p["root"] == "base_link"
    assert sorted(i for ids in p["links"].values() for i in ids) == sorted(p["meshes"])
    for link in ("UGV_body", "L_Pole", "R_Pole", "L_Link1", "R_Link7", "ILH_plam", "IRH_plam", "bin_link", "camera_link"):
        assert p["links"][link], f"{link} has no mesh"
    joints = {j["name"]: j for j in p["joints"]}
    assert len(joints) == len(urdf.joints_by_child)
    groups = {g["key"]: g["joints"] for g in p["groups"]}
    for s, side in (("L", "left"), ("R", "right")):
        names = [f"{s}_joint{i}" for i in range(1, 8)]
        assert groups[f"{side}_arm"] == names
        for name in names:
            ref = next(j for j in urdf.joints_by_child.values() if j.name == name)
            assert joints[name]["type"] == "revolute"
            assert joints[name]["lower"] == ref.lower < 0 < ref.upper == joints[name]["upper"], name
        assert len(groups[f"{side}_hand"]) == 6                 # the inspire hand's six drives
    assert groups["poles"] == ["L_sliding_plate_joint", "R_sliding_plate_joint"]
    listed = [n for g in groups.values() for n in g]
    assert len(listed) == len(set(listed)) and not any(joints[n]["mimic"] for n in listed)
    assert p["home"]["L_joint7"] == pytest.approx(3.1416) and p["home"]["R_joint2"] == pytest.approx(-1.885)
    assert p["home"]["L_sliding_plate_joint"] == pytest.approx(0.3)           # SRDF dual_poles/full_length
    assert p["home"]["bin_drop_joint"] == pytest.approx(0.45)                 # SRDF bin_rail/BIN_LOWERED


@pytest.mark.slow
def test_meshes_come_from_lites_bake(built):
    from alix_studio_lite.server.robot_model import BAKE

    _, p, _ = built
    assert p["stats"]["bake_file"] == "assets/robot/visual_overlay_baked.npz"
    src = _by_source(p)
    assert src["live"] == set() and p["stats"]["stale"] == {}           # the bake covers today's URDF
    z = np.load(BAKE)
    meta = json.loads(str(z["meta"]))
    for link in src["bake"]:
        ours = [_arrays(p["meshes"][i]) for i in p["links"][link]]
        theirs = [(z[f"V{k}"], z[f"F{k}"]) for k, m in enumerate(meta) if m["link"] == link]
        assert len(ours) == len(theirs), link
        for (v, f), (bv, bf) in zip(ours, theirs):
            assert np.array_equal(v, bv) and np.array_equal(f, bf), link
    assert p["links"]["camera_link"]                                      # the D435
    bin_top = max(_arrays(p["meshes"][i])[0][:, 2].max() for i in p["links"]["bin_link"])
    assert bin_top < 0.40                    # BinShort: 40 mm lower than the old bake's 0.438 m


def _write_bake(src, dst, drop=(), shift=None):
    z = np.load(src)
    keep = [(m, z[f"V{i}"], z[f"F{i}"]) for i, m in enumerate(json.loads(str(z["meta"]))) if m["link"] not in drop]
    blobs = {}
    for k, (m, v, f) in enumerate(keep):
        blobs[f"V{k}"] = v + np.float32((shift or {}).get(m["link"], 0.0))
        blobs[f"F{k}"] = f
    np.savez_compressed(dst, meta=json.dumps([m for m, _, _ in keep]), **blobs)


@pytest.mark.slow
def test_links_missing_from_the_bake_are_full_resolution(tmp_path, monkeypatch):
    """Studio's fall-through: a link the bake lacks is served from local_meshes(max_faces=0). A baked
    link that no longer matches its mesh stays baked and is named in the notes."""
    from alix_workspace.collision import RobotCollisionModel
    from alix_studio_lite.server import robot_model

    bake = tmp_path / "bake.npz"
    _write_bake(robot_model.BAKE, bake, drop={"camera_link"}, shift={"bin_link": 0.01})
    monkeypatch.setattr(robot_model, "BAKE", bake)
    p = robot_model.build_robot_model(tmp_path / "cache")
    src = _by_source(p)
    assert src["live"] == {"camera_link"} and "bin_link" in src["bake"]
    vcm = RobotCollisionModel.from_file(str(vp.URDF), tag="visual")
    full = vcm.local_meshes(max_faces=0, split_materials=True, link_filter=lambda ln: ln == "camera_link")
    ours = [_arrays(p["meshes"][i]) for i in p["links"]["camera_link"]]
    assert [len(f) for _, f in ours] == [len(m["faces"]) for m in full] and sum(len(m["faces"]) for m in full) > 200_000
    assert set(p["stats"]["stale"]) == {"bin_link"} and p["stats"]["stale"]["bin_link"] == pytest.approx(10, abs=0.5)
    assert any("Re-bake" in n for n in p["notes"])


@pytest.mark.slow
def test_every_mesh_decodes(built):
    _, p, _ = built
    for mid, m in p["meshes"].items():
        v, f = _arrays(m)
        assert len(v) and len(f), mid
        assert f.max() < len(v), mid
        assert np.isfinite(v).all() and np.abs(v).max() < 2.0, mid             # link frame, metres
        assert m["color"] is None or (len(m["color"]) == 3 and all(0 <= c <= 1 for c in m["color"])), mid


@pytest.mark.slow
def test_cache_is_reused(built):
    from alix_studio_lite.server.robot_model import CACHE_NAME, build_robot_model

    data, p, build_s = built
    assert (data / "cache" / CACHE_NAME).is_file()
    t0 = time.time()
    again = build_robot_model(data / "cache")
    assert again == p
    assert time.time() - t0 < 1.0


@pytest.mark.slow
def test_poses_are_alix_fk_exactly(built, urdf):
    from alix_studio_lite.server import routes_robot

    data, p, _ = built
    app = SimpleNamespace(data_dir=data)
    assert routes_robot.get_robot_model(app, {}, {}, None) == p                 # from the cache
    mesh_links = [ln for ln, ids in p["links"].items() if ids]
    tf = urdf.link_world_transforms(p["home"])
    assert p["homePose"] == {ln: tf[ln].reshape(-1).round(6).tolist() for ln in mesh_links}
    rng = np.random.default_rng(7)
    for _ in range(3):
        pose = {j["name"]: float(rng.uniform(j["lower"], j["upper"])) for j in p["joints"]
                if j["lower"] is not None and not j["mimic"]}
        out = routes_robot.post_robot_pose(app, {}, {"joints": pose}, None)["pose"]
        ref = urdf.link_world_transforms(pose)
        assert set(out) == set(mesh_links)
        for ln in mesh_links:
            assert out[ln] == ref[ln].reshape(-1).tolist(), ln               # same code path: no difference


@pytest.mark.slow
def test_track_frames(built, urdf):
    from alix_studio_lite.server import routes_robot
    from alix_studio_lite.server.jobs import UserError

    data, p, _ = built
    app = SimpleNamespace(data_dir=data)
    n = 23
    t = np.linspace(0, 1, n)
    q = [list(10 * (k + 1) * np.sin(2 * math.pi * t + k)) for k in range(7)]
    rest = {"L_sliding_plate_joint": 0.12, "ILH_left_index_1_joint": 0.8}
    r = routes_robot.post_track_frames(app, {}, {"side": "left", "joints_deg": q, "every": 5, "joints": rest}, None)
    assert r["lines"] == [0, 5, 10, 15, 20, 22]
    assert "L_Link1" in r["links"] and "ILH_plam" in r["links"] and "R_Link1" not in r["links"]
    frames = np.frombuffer(base64.b64decode(r["frames"]), "<f4").reshape(len(r["lines"]), len(r["links"]), 16)
    for f, line in enumerate(r["lines"]):
        values = dict(p["home"], **rest)
        values.update({f"L_joint{k + 1}": math.radians(q[k][line]) for k in range(7)})
        ref = urdf.link_world_transforms(values)
        for k, ln in enumerate(r["links"]):
            assert np.abs(frames[f, k] - ref[ln].reshape(-1)).max() < 1e-6, (line, ln)    # float32
    for bad in ({"side": "up", "joints_deg": q}, {"side": "left", "joints_deg": q[:6]},
                {"side": "left", "joints_deg": q, "joints": {"L_joint9": 1}}):
        with pytest.raises(UserError):
            routes_robot.post_track_frames(app, {}, bad, None)
