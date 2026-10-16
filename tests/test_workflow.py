"""Merge + Full Sequence S2 (feedback v5, slide 9): V53's Page 02 ported as it is. Its planner is
V53's own code: run here against V53's methods (read from the reference file, not imported: it is a
Tk program) on the owner's V6 files and on many generated sequences; the steps are checked as runner
v14 checks them; V53's JSON loads per arm and never into the other arm; Partial is a row range per
arm; Simulation Mode times both arms through the plan, says what would not be right on the robot and
checks collisions; Real Machine Mode only after the same run completed in Simulation."""
import ast
import json
import random
import threading
import time
import urllib.request
from types import SimpleNamespace

import numpy as np
import pytest

from alix_studio_lite import vendor_paths

V53 = vendor_paths.LITE_ROOT / "vendor/control_app/reference/rm_trajectory_ui_v53_safe_lid_checkpoint.py"
V6 = vendor_paths.LITE_ROOT / "tests/data/legacy_v6"


def v53_planner():
    """V53's _page2_program_chunks and _page2_build_program_rule_plan, compiled from its source, on a
    stand-in for its window that holds the two arms' steps as its builders do."""
    tree = ast.parse(V53.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "DualArmTrajectoryUI")
    fns = [f for f in cls.body if isinstance(f, ast.FunctionDef)
           and f.name in ("_page2_program_chunks", "_page2_build_program_rule_plan")]
    assert len(fns) == 2
    ns = {"json": json}
    exec(compile(ast.Module(body=fns, type_ignores=[]), str(V53), "exec"), ns)

    class Page2:
        _page2_program_chunks = ns["_page2_program_chunks"]
        _page2_build_program_rule_plan = ns["_page2_build_program_rule_plan"]

        def __init__(self, left, right):
            self.b = {"left": SimpleNamespace(steps=left), "right": SimpleNamespace(steps=right)}

        def _page2_builder(self, side):
            return self.b[side]

    return lambda left, right: Page2(left, right)._page2_build_program_rule_plan()


def outcome(fn, left, right):
    try:
        return ("plan", fn(left, right))
    except ValueError as exc:
        return ("refused", str(exc))


def v6():
    return [json.loads((V6 / f"{s}_V6_70SERIES_FINAL.json").read_text())["steps"] for s in ("LEFT", "RIGHT")]


def generated(rng: random.Random) -> list:
    steps = []
    for _ in range(rng.randint(0, 9)):
        k = rng.choice(["program", "program", "program", "pole", "hand", "wait", "global"])
        if k == "program":
            steps.append({"type": "program", "id": rng.choice([71, 72, 73, 74, 78, 79]), "name": "", "speed": 100,
                          "transit_speed": 30, "dual_rule": rng.choice(["TOGETHER", "TOGETHER", "RUN FIRST", "WAIT OTHER"])})
        elif k == "pole":
            steps.append({"type": "pole", "mm": rng.choice([50.0, 150.0, 200.0])})
        elif k == "hand":
            steps.append({"type": "hand", "values": [0, 0, 0, 0, 150, 1000]})
        elif k == "wait":
            steps.append({"type": "wait", "seconds": rng.choice([0.0, 2.0])})
        else:
            steps.append({"type": "global", "name": "HOME", "move": "movej", "speed": 20, "wait_s": 0.0})
    return steps


def test_the_planner_is_v53s_own():
    from alix_studio_lite.server import workflow

    theirs = v53_planner()
    left, right = v6()
    mine = workflow.build_plan(left, right)
    assert mine == theirs(left, right)
    assert [s["kind"] for s in mine] == ["parallel", "parallel", "checkpoint", "parallel", "serial", "serial", "parallel"]
    assert [s["program_id"] for s in mine] == ["TIMELINE-1", "LID-PREP-1", "LID-CHECK-1", "TIMELINE-2", 78, 79, "TIMELINE-3"]
    assert workflow.counts(mine) == dict(coordinated=6, together=4, one_arm=2, checkpoints=1, tail=0, stages=7)
    rng, refused = random.Random(53), 0
    for _ in range(3000):
        a, b = generated(rng), generated(rng)
        ours, v53 = outcome(workflow.build_plan, a, b), outcome(theirs, a, b)
        assert ours == v53, (a, b)
        refused += ours[0] == "refused"
    assert 300 < refused < 2900                                  # both ways well exercised


def test_steps_are_checked_and_v53s_json_loads_per_arm():
    from alix_studio_lite.server import workflow
    from alix_studio_lite.server.jobs import UserError

    left_doc = json.loads((V6 / "LEFT_V6_70SERIES_FINAL.json").read_text())
    steps = workflow.from_document("left", left_doc)
    assert len(steps) == 14 and steps[3] == {"type": "program", "id": 71, "name": "L_P01P02.txt", "speed": 100, "transit_speed": 30,
                                             "dual_rule": "TOGETHER"}
    assert steps[7] == {"type": "wait", "seconds": 2.0, "note": "MANUAL LID COVER OPEN"}
    with pytest.raises(UserError, match="LEFT sequence: it never loads into the RIGHT arm"):
        workflow.from_document("right", left_doc)
    with pytest.raises(UserError, match="wrong sequence format"):
        workflow.from_document("left", {"steps": []})
    old = {"format": workflow.FORMAT, "side": "left", "steps": [{"type": "program", "id": 5, "speed": 40}]}
    assert workflow.from_document("left", old)[0]["transit_speed"] == 30                 # V53's v51 compatibility
    for bad, said in (({"type": "pole", "mm": 400}, "row 1: pole target must be 0-315 mm"),
                      ({"type": "hand", "values": [1, 2, 3]}, "row 1: hand step requires exactly 6 values"),
                      ({"type": "program", "id": 7, "dual_rule": "FIRST"}, "row 1: the Dual Rule 'FIRST'"),
                      ({"type": "jump"}, "row 1: workflow step type must be")):
        with pytest.raises(UserError, match=said.replace("(", r"\(")):
            workflow.check_steps([bad], "left")


def test_partial_is_a_row_range_per_arm():
    from alix_studio_lite.server import workflow
    from alix_studio_lite.server.jobs import UserError

    left, right = (workflow.check_steps(s, side) for s, side in zip(v6(), ("left", "right")))
    steps = {"left": left, "right": right}
    run = workflow.plan_for(steps, "both", {"left": [9, 12], "right": [11, 14]})        # Program 76 to Program 80 on both
    assert run["first_row"] == {"left": 9, "right": 11}
    assert [s["kind"] for s in run["plan"]] == ["parallel", "serial", "serial", "parallel"]
    one = workflow.plan_for(steps, "right", {"right": [1, 4]})
    assert [st["type"] for st in one["plan"][0]["right"]["steps"]] == ["pole", "hand", "pole", "program"] and "left" not in one["steps"]
    with pytest.raises(UserError, match="Rows 3 to 99 are not in a sequence of 14"):
        workflow.plan_for(steps, "both", {"left": [3, 99]})
    with pytest.raises(UserError, match="Cannot start dual sequence: RUN FIRST / WAIT OTHER serial handoffs must use the same"):
        workflow.plan_for(steps, "both", {"left": [10, 10], "right": [13, 13]})            # 78 RUN FIRST vs 80 TOGETHER... refused as V53


# ── Simulation Mode and the gate to Real Machine Mode, on the stand-in (no controller)

PID = "alix_s2"
P = f"/api/projects/{PID}"


@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    vendor_paths.activate()
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-s2")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app, "data": data}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    call(ctx, "PUT", P, {"id": PID, "name": "S2", "archived": False, "state": {"currentTask": 0}})
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json", "X-Lite-Token": ctx.get("tok") or ""}
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=240) as resp:
        return json.loads(resp.read())


def wait(ctx, job, timeout=240):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < timeout, j
        time.sleep(0.2)


def sent(ctx, side, pid, name, pole, hand, start):
    """A file the stand-in holds as program ``pid``, as Lite sends it, with the project's file of its bytes."""
    from alix_studio_lite.motion import track

    key = {"left": "L-R01", "right": "R-R02"}[side] if pid < 75 else {"left": "L-R02", "right": "R-R03"}[side]
    t = np.linspace(0.0, 1.0, 300)[:, None]
    md = np.round((np.array([start]) + 6.0 * np.sin(2 * np.pi * t)) * 1000).astype(np.int64)
    out = call(ctx, "POST", f"{P}/units/{key}/recordings", {"source": "upload", "name": name, "content": track.track_bytes(md).decode(),
                                                            "pole_mm": pole, "hand": hand})
    assert "error" not in out, out
    rec = next(r for r in out["unit"]["recordings"] if r["file"] == out["saved"])
    arm = ctx["app"].robot.arm(side)
    arm.programs[pid] = dict(name=name[:-4], lines=300, bytes=1, sha256=rec["sha256"], stage=rec["stage"], first_row=md[0].tolist(),
                             uploaded="2026-10-16 12:00")
    arm.save()


def test_simulation_times_both_arms_and_gates_real(lite):
    sent(lite, "left", 71, "L_s2_a.txt", 150, "GRIP", [10, -40, 0, 80, 0, 40, 0])
    sent(lite, "right", 71, "R_s2_a.txt", 50, "GRIP", [-10, -40, 0, 80, 0, 40, 0])
    left = [{"type": "pole", "mm": 200}, {"type": "hand", "values": [0, 0, 0, 0, 150, 1000]}, {"type": "pole", "mm": 150},
            {"type": "program", "id": 71, "transit_speed": 30, "dual_rule": "TOGETHER"},
            {"type": "program", "id": 72, "transit_speed": 30, "dual_rule": "TOGETHER"}]
    right = [{"type": "pole", "mm": 200}, {"type": "program", "id": 71, "transit_speed": 30, "dual_rule": "TOGETHER"},
             {"type": "wait", "seconds": 1.5}]
    call(lite, "PUT", f"{P}/workflow/left", {"steps": left})
    assert call(lite, "PUT", f"{P}/workflow/right", {"steps": right})["right"][2] == {"type": "wait", "seconds": 1.5}
    plan = call(lite, "POST", f"{P}/workflow/plan", {"mode": "both"})
    assert [s["kind"] for s in plan["stages"]] == ["parallel", "tail"] and plan["counts"]["together"] == 1

    j = wait(lite, call(lite, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    assert j["state"] == "done", j
    sim = j["result"]
    st = sim["stages"]
    assert st[0]["t0"] == 0 and st[0]["t1"] == st[1]["t0"] and st[1]["t1"] == sim["total_s"]
    la, ra = sim["acts"]["left"], sim["acts"]["right"]
    assert [a["kind"] for a in la] == ["pole", "hand", "pole", "program", "program"] and [a["row"] for a in la] == [1, 2, 3, 4, 5]
    assert la[3]["drawn"] and la[3]["lines"] == 300 and la[3]["t1"] - la[3]["traj_t0"] == pytest.approx(299 * 0.005, abs=1e-3)
    assert not la[4]["drawn"]                                    # program 72: Lite never sent it
    assert ra[1]["t0"] == ra[0]["t1"] and ra[2]["kind"] == "wait" and ra[2]["stage"] == 1   # the tail, after TIMELINE-1
    texts = " | ".join(f["text"] for f in sim["findings"])
    assert "LEFT row 5: program 72 is not drawn (Lite has not sent this program)" in texts
    assert "From here the arm's pose is not known" in texts             # the check leaves that arm out after it
    assert "RIGHT row 2, program 71: taught at pole 50 mm, hand grip, but the pole is at 200 mm here" in texts
    assert "hand" in texts and "no Hand step before it" in texts   # the right arm's hand was never set in this run
    col = sim["collisions"]
    assert col["samples"] > 100 and col["checked"]["two_arms"] and set(col["checked"]["arms"]) == {"left", "right"}
    fr = call(lite, "POST", f"{P}/workflow/frames", {"side": "left", "stage": 0})
    assert fr["times"][0] == 0 and fr["times"][-1] == pytest.approx(la[4]["t1"], abs=1e-3) and len(fr["links"]) > 20
    assert call(lite, "POST", f"{P}/workflow/frames", {"side": "right", "stage": -1})["times"] == [0.0]

    view = call(lite, "GET", f"{P}/workflow")["sim"]
    assert view["hash"] == sim["hash"] and not view["completed"] and "q0" not in view["acts"]["left"][0]
    refused = call(lite, "POST", f"{P}/workflow/run", {"mode": "both"})
    assert refused["code"] == "mode"                             # Visual sim: never on the robot
    assert "no longer the last" in call(lite, "POST", f"{P}/workflow/played", {"hash": "x"})["error"]
    assert call(lite, "POST", f"{P}/workflow/played", {"hash": sim["hash"]})["completed"]

    from alix_studio_lite.server import workflow
    from alix_studio_lite.server.lite_units import LiteUnits

    units = LiteUnits(lite["data"] / "projects" / PID, "commode_c")
    run = workflow.plan_for(workflow._numbered(workflow.load(units)), "both", None)
    progs = {s: workflow.programs_known(lite["app"], units, s) for s in workflow.ARMS}
    same = workflow.identity_of(run, progs, {"left": None, "right": None}, int(lite["app"].robot.speed_pct))
    assert workflow.digest(same) == sim["hash"]
    other = dict(same, overall=same["overall"] + 1)
    assert "the speed (simulated at" in workflow._changed(workflow.load_sim(units), other)
    call(lite, "PUT", f"{P}/workflow/right", {"steps": right[:2]})
    run2 = workflow.plan_for(workflow._numbered(workflow.load(units)), "both", None)
    assert "its steps" in workflow._changed(workflow.load_sim(units), workflow.identity_of(run2, progs, {"left": None, "right": None},
                                                                                           same["overall"]))


def test_a_simulation_is_checked_again_in_one_commode_state(lite):
    """S2's 3D (view3d.js): a commode state chosen under the view checks the same run again with the commode in
    that state throughout (the owner, 16 Oct: an override in the view only); the run and its own check stay."""
    sim = call(lite, "GET", f"{P}/workflow")["sim"]
    r = wait(lite, call(lite, "POST", f"{P}/workflow/collisions", {"hash": sim["hash"], "phase": "seat_open"}))
    assert r["state"] == "done", r
    out = r["result"]
    assert out["phase"] == "seat_open" and out["hash"] == sim["hash"] and isinstance(out["spans"], list)
    assert out["samples"] > 100 and set(out["checked"]["arms"]) == {"left", "right"}
    for t0, t1, links in out["spans"]:
        assert 0 <= t0 <= t1 <= sim["total_s"] + 0.05 and links
    assert "no longer the last" in call(lite, "POST", f"{P}/workflow/collisions", {"hash": "x", "phase": "seat_open"})["error"]
    assert "lid closed, lid open or seat open" in call(lite, "POST", f"{P}/workflow/collisions", {"hash": sim["hash"], "phase": "up"})["error"]
    assert call(lite, "GET", f"{P}/workflow")["sim"]["hash"] == sim["hash"]
