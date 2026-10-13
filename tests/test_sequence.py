"""The full cleaning sequence (server/sequence.py): each row's arm steps, their files, and their
times on one clock: the rules (TOGETHER, RUN FIRST, ONE BY ONE, PROCEDURE), checkpoint waits, each
arm continuing from where its previous step left it, and what blocks a row."""
import json
import threading
import urllib.request

import pytest

from alix_studio_lite import vendor_paths

vendor_paths.activate()

TAUGHT = vendor_paths.LITE_ROOT / "tests/data/recordings/commode_c_taught_paths"
V6 = vendor_paths.LITE_ROOT / "tests/data/legacy_v6"


@pytest.fixture(scope="module")
def units(tmp_path_factory):
    from alix_studio_lite.server.lite_units import LiteUnits

    return LiteUnits(tmp_path_factory.mktemp("seq") / "p", "commode_c")


def task_no(units, code):
    """The flow table's step number (1-based) of the task whose LEFT or RIGHT step is ``code``."""
    from alix_studio_lite.server.lite_units import task_list

    for i, t in enumerate(task_list(units), 1):
        if any(t[s] and t[s]["key"] == code for s in ("left", "right")):
            return i
    raise KeyError(code)


def fake_identity(files):
    """identity() over a few known files: {key: (path, arm, pole_mm, hand)}; the rest have no version."""
    from alix_studio_lite.server import staging
    from alix_studio_lite.server.jobs import UserError

    def identity(units, key, which):
        if key not in files:
            raise UserError("There is no tuned version yet.", "Finish Tune + Simulation first.")
        path, arm, pole, hand = files[key]
        return dict(path=path, name=f"{key} file", taught_arm=arm, stage=staging.make_stage(pole, hand, "test"))
    return identity


FILES = {
    "L-T01": (TAUGHT / "FILE_10.1.txt", "left", 200, "50% open"),
    "R-T01": (V6 / "RIGHT/R_P06P07.txt", "right", 150, "GRIP"),
}


def plan(units, rows, source="version"):
    from alix_studio_lite.motion.limits import load_limits
    from alix_studio_lite.server import sequence

    return sequence.plan(units, rows, source=source, home={}, identity=fake_identity(FILES),
                         rating={s: load_limits(s).rating_deg_s for s in ("left", "right")})


def test_together_starts_both_trajectories_on_one_tick(units):
    n = task_no(units, "L-T01")
    p = plan(units, [{"step": n, "rule": "TOGETHER", "checkpointWait": 2}])
    row = p["rows"][0]
    L, R = row["arms"]["left"], row["arms"]["right"]
    assert p["ready"] and L["ready"] and R["ready"] and L["stage_at"] == R["stage_at"] == 0
    assert L["traj_at"] == R["traj_at"] == pytest.approx(max(L["traj_t0"], R["traj_t0"]), abs=1e-3)
    assert row["t1"] == pytest.approx(L["traj_at"] + max(L["traj_s"], R["traj_s"]), abs=1e-3)
    assert p["total_s"] == pytest.approx(row["t1"] + 2, abs=1e-3)
    assert L["traj_s"] == pytest.approx((L["lines"] - 1) * 0.005)                     # the 5 ms clock
    assert L["start"] is None and L["t_pole"] > 0 and L["stage"]["pole_mm"] == 200   # from the standing pose


def test_run_first_and_one_by_one(units):
    n = task_no(units, "L-T01")
    lf = plan(units, [{"step": n, "rule": "LEFT RUN FIRST"}])["rows"][0]["arms"]
    assert lf["right"]["traj_at"] == pytest.approx(max(lf["right"]["traj_t0"], lf["left"]["end_at"]), abs=1e-3)
    rf = plan(units, [{"step": n, "rule": "RIGHT RUN FIRST"}])["rows"][0]["arms"]
    assert rf["left"]["traj_at"] == pytest.approx(max(rf["left"]["traj_t0"], rf["right"]["end_at"]), abs=1e-3)
    ob = plan(units, [{"step": n, "rule": "ONE BY ONE"}])["rows"][0]["arms"]
    assert ob["right"]["stage_at"] == pytest.approx(ob["left"]["end_at"], abs=1e-3)   # one arm moves at a time


def test_each_arm_continues_where_it_stopped(units):
    n = task_no(units, "L-T01")
    p = plan(units, [{"step": n, "rule": "TOGETHER", "checkpointWait": 1.5}, {"step": n, "rule": "TOGETHER"}])
    first, second = p["rows"]
    a, b = first["arms"]["left"], second["arms"]["left"]
    assert b["start"] == a["end"] and a["end"]["pole_mm"] == 200 and len(a["end"]["q_deg"]) == 7
    assert b["t_pole"] == 0 and b["t_hand"] == 0                                        # already staged
    import numpy as np

    from alix_studio_lite.motion import track
    from alix_studio_lite.motion.limits import load_limits

    first_line = track.read_md(FILES["L-T01"][0])[0] / 1000.0                           # last line back to the first,
    span = np.abs(first_line - np.asarray(a["end"]["q_deg"])) / (0.3 * np.asarray(load_limits("left").rating_deg_s))
    assert b["t_move"] == pytest.approx(max(1.0, float(span.max()) + 0.5), abs=1e-3)    # at 30 % of the rating
    assert second["t0"] == pytest.approx(first["t1"] + 1.5, abs=1e-3)


def test_what_blocks_and_what_is_skipped(units):
    from alix_studio_lite.server.lite_units import task_list

    later = next(i for i, t in enumerate(task_list(units), 1)
                 if t["right"] and t["right"]["later"] and t["left"] and not t["left"]["later"])
    p = plan(units, [{"step": task_no(units, "L-T01"), "rule": "TOGETHER"}, {"step": later, "rule": "TOGETHER"},
                     {"step": 1, "rule": "PROCEDURE", "checkpointWait": 3}, {"step": 99}])
    t3, t10, proc, missing = p["rows"]
    assert t10["arms"]["right"]["skip"] and "Release 1" in t10["arms"]["right"]["why"]  # LATER: skipped
    assert not t10["arms"]["left"]["ready"] and "no tuned version" in t10["arms"]["left"]["why"]
    assert proc["arms"] == {} and proc["t1"] == proc["t0"] and missing["t0"] == pytest.approx(proc["t0"] + 3)
    assert "not in this project's task list" in missing["note"]
    assert not p["ready"] and p["playable"] and len(p["blocking"]) == 2                   # a preview still plays
    assert all(not b.startswith(t3["label"]) for b in p["blocking"])


@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-seq")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}"}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    call(ctx, "PUT", "/api/projects/alix_seq", {"id": "alix_seq", "name": "Seq", "archived": False, "state": {}})
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json"}
    if ctx.get("tok"):
        headers["X-Lite-Token"] = ctx["tok"]
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=120) as resp:
        return json.loads(resp.read())


def test_sequence_and_step_timeline_over_http(lite):
    import base64

    import numpy as np

    rows = [{"step": 5, "label": "TASK 3", "rule": "TOGETHER"}]
    p = call(lite, "POST", "/api/projects/alix_seq/sequence", {"steps": rows, "source": "version"})
    assert not p["ready"] and any("no tuned version" in b for b in p["blocking"])      # nothing tuned yet
    p = call(lite, "POST", "/api/projects/alix_seq/sequence", {"steps": rows, "source": "recording"})
    a = p["rows"][0]["arms"]["left"]
    assert p["playable"] and a["ready"] and a["file"] == "FILE_10.1.txt"                # the existing recording
    assert call(lite, "POST", "/api/projects/alix_seq/sequence", {"steps": []})["error"]
    # the step from where an earlier step left the arm: the pole already there, nothing to stage
    start = {"pole_mm": 200, "hand": [500, 500, 500, 500, 1000, 1000], "q_deg": a["end"]["q_deg"]}
    tl = call(lite, "POST", "/api/projects/alix_seq/units/L-T01/timeline", {"which": "raw", "start": start})
    assert tl["phases"][0]["text"] == "Pole already at 200 mm" and tl["phases"][1]["text"] == "Hand already in pose"
    assert tl["end"] == a["end"]
    f = np.frombuffer(base64.b64decode(tl["frames"]), dtype="<f4").reshape(len(tl["times"]), len(tl["links"]), 16)
    assert np.allclose(f[0], f[-1], atol=1e-4)                                         # it ends where it started
    bad = call(lite, "POST", "/api/projects/alix_seq/units/L-T01/timeline", {"which": "raw", "start": {"pole_mm": 900}})
    assert bad["error"]
