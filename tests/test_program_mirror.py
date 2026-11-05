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


def test_the_simulations_arm_never_overwrites_the_kept_programs_and_holds_them(tmp_path):
    """The user, 5 Nov: a save in Visual sim mirrored the stand-in, which holds none of the lab's programs, and deleted
    31 of the left arm's kept files. Only a real controller's programs are kept now: on the stand-in Data Management's
    save is refused, its reads and S2's simulation write nothing, and the download gives the kept copy. The other way
    round, the simulation's arm holds what the real controller held when it was last kept: its programs (S2's sources,
    Data Management, the simulation's programs) and its global points."""
    import io
    import threading
    import urllib.request
    import zipfile

    from alix_studio_lite.server import content, workflow
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
        from alix_studio_lite.server.api_units import open_units
        units = open_units(app.store, "alix_keep1")                     # the project's folder (no request's session here)
        folder = units.root / "controller_exports/right"                  # the lab's kept copy, as Data Management left it
        lab = track.write_md(folder / "02_2_R_P2_R04_pole200_grip_7.6s_v.txt", _md(60, 3))
        man = dict(arm="right", kept="2026-11-05 01:11:37", not_kept=[], points={"R_HME_POS": [-4.12, -99.67, -0.26, 91.47, 5.14, 93.61, 0.04]},
                   programs=[dict(slot=2, name="2_R_P2_R04_pole200_grip_7.6s_v.t", file=lab.name, sha256=track.sha256_file(lab), lines=60,
                                  seconds=0.295, stage={"pole_mm": 200.0, "hand": [0, 0, 0, 0, 150, 1000]})])
        (folder / "programs.json").write_text(json.dumps(man))
        refused = call("POST", "/api/robots/right/stored/keep", {"pid": "alix_keep1"}, tok)
        assert refused["code"] == "sim" and "Nothing was changed" in refused["error"]
        call("GET", "/api/robots/right/data?pid=alix_keep1", token=tok)                     # a read: writes nothing either
        assert pm.keep_quietly(app, units, "right") is None
        assert lab.is_file() and json.loads((folder / "programs.json").read_text())["programs"][0]["slot"] == 2
        z = zipfile.ZipFile(io.BytesIO(call("GET", "/api/robots/right/stored/kept.zip?pid=alix_keep1", token=tok, raw=True)))
        assert {"right/programs.json", f"right/{lab.name}"} <= set(z.namelist())
        # the simulation's controller is the real one's, as kept
        src = call("GET", "/api/projects/alix_keep1/workflow/right/sources", token=tok)
        assert [p["id"] for p in src["programs"]] == [2] and src["points"] == ["R_HME_POS"] and "last kept" in src["note"]
        known = workflow.programs_known(app, units, "right")
        assert known[2]["sent"] and known[2]["sha256"] == track.sha256_file(lab) and known[2]["file"] == str(lab)
        assert known[2]["stage"]["pole_mm"] == 200.0
        assert workflow.points_known(app, "right", units)["points"]["R_HME_POS"][0] == -4.12
        dm = call("GET", "/api/robots/right/data?pid=alix_keep1", token=tok)
        assert [p["num"] for p in dm["programs"]] == [2] and dm["points"][0]["name"] == "R_HME_POS" and dm["kept_copy"]
        assert content.content(app, "right", units=units)["standin"]
    finally:
        srv.shutdown()
