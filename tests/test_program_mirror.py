"""The controllers' programs kept in the project (server/program_mirror.py; the user, 4 Nov): the bytes Lite sent each
slot, by its sha256, under <project>/controller_exports/<arm>/, and found there by the slow-down tool."""
import json
from types import SimpleNamespace

import numpy as np

from alix_studio_lite.motion import track
from alix_studio_lite.server import program_mirror as pm
from alix_studio_lite.server import slow_programs as sp


def _md(n, seed):
    rng = np.random.default_rng(seed)
    return np.cumsum(rng.integers(-40, 40, size=(n, 7)), axis=0).astype(np.int64)


def test_each_slot_lite_sent_is_kept_by_its_bytes_and_the_others_listed(tmp_path):
    sent = track.write_md(tmp_path / "data/advanced/right/20261029_182812_R_P2_R04_pole200_grip_7.6s_v1.1.txt", _md(50, 1))
    units = SimpleNamespace(root=tmp_path / "project")
    known = {2: dict(sent=True, file=str(sent), name="2_R_P2_R04_pole200_grip_7.6s_v.t", sha256=track.sha256_file(sent),
                     lines=50, seconds=0.245, stage={"pole_mm": 200.0, "hand": [0, 0, 0, 0, 150, 1000]}),
             9: dict(sent=False, name="9_control_program.t"),
             4: dict(sent=True, file=str(tmp_path / "gone.txt"), name="4_x.t", sha256="ab"),
             7: dict(sent=True, gone=True, name="7_old.t")}
    m = pm.keep(None, units, "right", known)
    folder = tmp_path / "project/controller_exports/right"
    kept = folder / "02_2_R_P2_R04_pole200_grip_7.6s_v.txt"
    assert kept.read_bytes() == sent.read_bytes() and [r["slot"] for r in m["programs"]] == [2]
    assert m["programs"][0]["stage"]["pole_mm"] == 200.0 and m["programs"][0]["source"] == str(sent)
    assert {x["slot"] for x in m["not_kept"]} == {4, 9}                       # not sent; its file no longer here
    assert json.loads((folder / "programs.json").read_text())["programs"][0]["sha256"] == track.sha256_file(kept)
    stamp = kept.stat().st_mtime_ns
    pm.keep(None, units, "right", known)                                       # the same bytes: not copied again
    assert kept.stat().st_mtime_ns == stamp
    pm.keep(None, units, "right", {})                                          # the slot emptied: its copy goes
    assert not kept.exists()


def test_the_slow_down_tool_finds_a_file_kept_in_the_project(tmp_path):
    md = _md(400, 2)
    kept = track.write_md(tmp_path / "project/controller_exports/right/02_2_R_P2.txt", md)
    sim = dict(acts=dict(right=[dict(kind="program", program=2, file="/home/alai-labs/ALIX-Lite/advanced/right/x.txt",
                                     file_name="x.txt", lines=len(md), line_s=0.005, traj_t0=10.0, text="Program 2 // R_P2")],
                         left=[]),
               identity=dict(used=dict(right=dict(programs={"2": track.sha256_file(kept)}))))
    (a,) = sp.program_acts(sim, project_dir=tmp_path / "project")
    assert a.file == kept
    (b,) = sp.program_acts(sim)                                                # without the project: not on this computer
    assert not b.file.exists()


def test_data_management_saves_and_downloads_the_kept_programs(tmp_path):
    import io
    import threading
    import urllib.request
    import zipfile

    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path / "data"
    app = LiteApp(open_project(data, "commode_c"), data, projects_dir=tmp_path / "Projects")
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"

    def call(method, path, body=None, token=None, raw=False):
        r = urllib.request.Request(base + path, data=None if body is None else json.dumps(body).encode(), method=method,
                                   headers={"Content-Type": "application/json", **({"X-Lite-Token": token} if token else {})})
        with urllib.request.urlopen(r, timeout=30) as resp:
            out = resp.read()
            return out if raw else json.loads(out)

    try:
        tok = call("POST", "/api/auth/login", {"pin": "1234"})["token"]
        assert "Open a project first" in call("POST", "/api/robots/right/stored/keep", {}, tok)["error"]
        page = {"id": "alix_keep1", "name": "Keep", "createdAt": "2026-11-04T10:00:00.000Z", "updatedAt": "2026-11-04T10:00:00.000Z",
                "archived": False, "state": {"currentTask": 0, "currentView": "setup"}}
        call("PUT", "/api/projects/alix_keep1", page, tok)
        m = call("POST", "/api/robots/right/stored/keep", {"pid": "alix_keep1"}, tok)
        assert m["arm"] == "right" and m["folder"].endswith("controller_exports/right") and "programs" in m
        z = zipfile.ZipFile(io.BytesIO(call("GET", "/api/robots/right/stored/kept.zip?pid=alix_keep1", token=tok, raw=True)))
        assert "right/programs.json" in z.namelist()
    finally:
        srv.shutdown()
