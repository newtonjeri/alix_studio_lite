"""Merge + Sequence S2 (feedback v5, slide 9): V53's Page 02 ported as it is. Its planner is
V53's own code: run here against V53's methods (read from the reference file, not imported: it is a
Tk program) on the owner's V6 files and on many generated sequences; the steps are checked as runner
v14 checks them; V53's JSON loads per arm and never into the other arm; a run is a single task, a
dual-arm task or the sequences (Full and Partial are gone: the owner, 16 Oct); Simulation Mode times both
arms through the plan, says what would not be right on the robot and checks collisions; Real Machine Mode
only after the same run completed in Simulation."""
import ast
import json
import random
import threading
import time
import urllib.request
from pathlib import Path
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


def test_a_tracks_execution_time_is_its_lines_less_one_on_the_5_ms_clock():
    from alix_studio_lite.motion import track

    assert track.duration_s(300) == 1.495 and track.duration_s(1) == 0.0 and track.duration_s(0) is None
    assert track.duration_s(None) is None and track.duration_s("600") == 2.995


def test_the_planner_is_v53s_own(monkeypatch):
    """With V53's own Wait checkpoint rule (off in Lite since 21 Oct: WAIT_CHECKPOINTS), the plan is V53's own."""
    from alix_studio_lite.server import workflow

    monkeypatch.setattr(workflow, "WAIT_CHECKPOINTS", True)
    theirs = v53_planner()
    left, right = v6()
    mine = workflow.build_plan(left, right)
    assert mine == theirs(left, right)
    assert [s["kind"] for s in mine] == ["parallel", "parallel", "checkpoint", "parallel", "serial", "serial", "parallel"]
    assert [s["program_id"] for s in mine] == ["TIMELINE-1", "LID-PREP-1", "LID-CHECK-1", "TIMELINE-2", 78, 79, "TIMELINE-3"]
    assert workflow.counts(mine) == dict(coordinated=6, together=4, one_arm=2, checkpoints=1, tail=0, mechanisms=0, stages=7)
    rng, refused = random.Random(53), 0
    for _ in range(3000):
        a, b = generated(rng), generated(rng)
        ours, v53 = outcome(workflow.build_plan, a, b), outcome(theirs, a, b)
        assert ours == v53, (a, b)
        refused += ours[0] == "refused"
    assert 300 < refused < 2900                                  # both ways well exercised


def zero_waits(steps: list) -> list:
    return [dict(st, seconds=0.0) if st.get("type") == "wait" else st for st in steps]


def plan_without_seconds(plan):
    """The plan with every Wait at 0 s, wherever its steps are (a stage's arms, a serial stage's first and second)."""
    if isinstance(plan, list):
        return [plan_without_seconds(x) for x in plan]
    if isinstance(plan, dict):
        out = {k: plan_without_seconds(v) for k, v in plan.items()}
        if out.get("type") == "wait":
            out["seconds"] = 0.0
        return out
    return plan


def test_a_wait_is_its_arms_own_idle_and_never_a_checkpoint():
    """The owner, 21 Oct (the lab's pop-up: "SAFE TIMELINE checkpoint mismatch ... A positive WAIT exists ... on only one
    arm"): a Wait is "an independent task, idle task, wait for 5 sec ... a user defined stage in the sequence". On one arm
    or both, anywhere, it stays among its arm's steps (its arm idles for its seconds there) and never stops the other arm
    or asks the operator: V53's plan as V53 makes it with the Waits at 0 s, each Wait keeping its own seconds."""
    from alix_studio_lite.server import workflow

    assert workflow.WAIT_CHECKPOINTS is False
    theirs = v53_planner()
    prog = lambda i, rule="TOGETHER": {"type": "program", "id": i, "name": "", "speed": 100, "transit_speed": 30,   # noqa: E731
                                       "dual_rule": rule}
    plan = workflow.build_plan([prog(71), {"type": "wait", "seconds": 5.0}, prog(72)], [prog(81), prog(82)])
    assert [(st["kind"], st["program_id"]) for st in plan] == [("parallel", "TIMELINE-1")]
    assert [(x["type"], x.get("id", x.get("seconds"))) for x in plan[0]["left"]["steps"]] == [("program", 71), ("wait", 5.0),
                                                                                              ("program", 72)]
    left, right = v6()                                  # V6's 2 s "MANUAL LID COVER OPEN" Waits on both arms: idles now
    mine = workflow.build_plan(left, right)
    assert "checkpoint" not in [st["kind"] for st in mine] and workflow.counts(mine)["checkpoints"] == 0
    rng, refused = random.Random(21), 0
    for _ in range(3000):
        a, b = generated(rng), generated(rng)
        ours = outcome(workflow.build_plan, a, b)
        v53 = outcome(theirs, zero_waits(a), zero_waits(b))
        assert (ours[0], plan_without_seconds(ours[1]) if ours[0] == "plan" else ours[1]) == v53, (a, b)
        assert ours[0] == "refused" or not any(st["kind"] == "checkpoint" for st in ours[1])
        assert ours[0] == "plan" or "checkpoint" not in ours[1]                       # never refused for a Wait
        refused += ours[0] == "refused"
    assert refused < 3000


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


def test_a_single_task_a_dual_arm_task_or_the_sequences(monkeypatch):
    """What is run (the owner, 16 Oct: "options to play a single task arm, dual arm task and sequences in both
    simulation and real"): a Program row alone on its arm with the Pole and Hand in force before it; one
    coordinated stage of V53's plan with both arms' Programs, each arm staged the same way; or the sequences. With
    V53's Wait checkpoint rule on (off since 21 Oct), a task after the checkpoint asks it first."""
    from alix_studio_lite.server import workflow
    from alix_studio_lite.server.jobs import UserError

    monkeypatch.setattr(workflow, "WAIT_CHECKPOINTS", True)

    left, right = (workflow.check_steps(s, side) for s, side in zip(v6(), ("left", "right")))
    steps = workflow._numbered({"left": left, "right": right})
    task = workflow.plan_for(steps, "both", {"scope": "task", "side": "left", "row": 6})     # Program 73: hand (row 2), pole 50 (row 5)
    assert task["mode"] == "left" and task["pick"] == {"scope": "task", "side": "left", "row": 6} and "right" not in task["steps"]
    assert [(st["type"], st["_row"]) for st in task["steps"]["left"]] == [("hand", 2), ("pole", 5), ("program", 6)]
    assert task["plan"][0]["kind"] == "single" and task["plan"][0]["program_id"] == 73
    with pytest.raises(UserError, match="LEFT row 1 is not a Program row"):
        workflow.plan_for(steps, "both", {"scope": "task", "side": "left", "row": 1})

    full = workflow.build_plan(steps["left"], steps["right"])
    assert [st["kind"] for st in workflow.plan_for(steps, "both")["plan"]] == [st["kind"] for st in full]   # the sequences, every row
    later = next(i for i, st in enumerate(full) if st["kind"] == "parallel" and i > 2)                     # after the lid checkpoint
    dual = workflow.plan_for(steps, "both", {"scope": "dual", "stage": later})
    # after the manual lid cover checkpoint: the task's run asks it first (the owner, 16 Oct), as the sequence does
    assert dual["mode"] == "both" and [st["kind"] for st in dual["plan"]] == ["checkpoint", "parallel"]
    check = dual["plan"][0]
    assert check["title"] == "MANUAL LID COVER CHECKPOINT" and check["message"].startswith("This task comes after the manual lid")
    assert "Lid cover is fully open" in check["message"] and dual["counts"]["checkpoints"] == 1
    row = next(x["_row"] for x in full[later]["left"]["steps"] if x["type"] == "program")
    alone = workflow.plan_for(steps, "both", {"scope": "task", "side": "left", "row": row})
    assert [st["kind"] for st in alone["plan"]] == ["checkpoint", "single"] and alone["mode"] == "left"
    assert [st["kind"] for st in task["plan"]] == ["single"]                                          # before it: no question
    for side in ("left", "right"):                                     # the pole and hand in force, then the stage's own rows
        kinds = [st["type"] for st in dual["steps"][side]]
        assert "program" in kinds and kinds.index("program") > 0 and {"pole", "hand"} <= set(kinds[:kinds.index("program")])
    serial = next(i for i, st in enumerate(full) if st["kind"] == "serial")
    asks = [["checkpoint"]] if any(st["kind"] == "checkpoint" for st in full[:serial]) else [[]]
    assert [st["kind"] for st in workflow.plan_for(steps, "both", {"scope": "dual", "stage": serial})["plan"]] == asks[0] + ["serial"]
    check = next(i for i, st in enumerate(full) if st["kind"] == "checkpoint")
    with pytest.raises(UserError, match="not a dual-arm task"):
        workflow.plan_for(steps, "both", {"scope": "dual", "stage": check})

    # the page says what it chose (the Program, the stage's Programs): a row or a stage that holds others since is
    # refused, so the one confirmation never names another Program than the one that runs
    assert workflow.plan_for(steps, "both", {"scope": "task", "side": "left", "row": 6, "id": 73})["pick"]["row"] == 6
    with pytest.raises(UserError, match="LEFT row 6 holds Program 73 now, not Program 72") as e:
        workflow.plan_for(steps, "both", {"scope": "task", "side": "left", "row": 6, "id": 72})
    assert e.value.code == "pick"
    ids = {side: [x["id"] for x in (full[later][side]["steps"]) if x["type"] == "program"] for side in ("left", "right")}
    assert workflow.plan_for(steps, "both", {"scope": "dual", "stage": later, "ids": ids})["mode"] == "both"
    with pytest.raises(UserError, match=f"Stage {later + 1} holds LEFT Program") as e:
        workflow.plan_for(steps, "both", {"scope": "dual", "stage": later, "ids": dict(ids, left=[99])})
    assert e.value.code == "pick"


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
    assert sim["programs"]["left"]["71"]["seconds"] == pytest.approx(1.495)                  # its execution time travels with it
    src = call(lite, "GET", f"{P}/workflow/left/sources")
    assert next(p for p in src["programs"] if p["id"] == 71)["seconds"] == pytest.approx(1.495)
    assert not la[4]["drawn"]                                    # program 72: Lite never sent it
    assert ra[1]["t0"] == ra[0]["t1"] and ra[2]["kind"] == "wait" and ra[2]["stage"] == 1   # the tail, after TIMELINE-1
    texts = " | ".join(f["text"] for f in sim["findings"])
    assert "LEFT row 5: program 72 is not drawn (Lite has not sent this program; no file of its name in the project" in texts
    assert "From here the arm's pose is not known" in texts             # the check leaves that arm out after it
    assert "RIGHT row 2, program 71: taught at pole 50 mm, hand grip, but the pole is at 200 mm here" in texts
    stop = next(f for f in sim["findings"] if "RIGHT row 2, program 71: taught at" in f["text"])
    assert stop["level"] == "stop" and "It runs only with its own" in stop["text"]     # Real Machine Mode does not run it
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

    units = LiteUnits(lite["app"].store.folder(PID), "commode_c")
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


def test_a_pulled_simulation_plays_from_the_projects_own_folder(lite):
    """A simulation keeps its program files relative to the project's folder, so one made on another computer (the lab's,
    pulled with the project) plays here; an older one with that computer's absolute paths is read under the project's
    folder here; a file that is nowhere refuses with the program named, instead of failing on the server."""
    from alix_studio_lite.server import workflow
    from alix_studio_lite.server.lite_units import LiteUnits

    units = LiteUnits(lite["app"].store.folder(PID), "commode_c")
    f = workflow._sim_file(units)
    kept = f.read_text(encoding="utf-8")
    sim = json.loads(kept)
    drawn = [a for a in sim["acts"]["left"] if a["kind"] == "program" and a.get("drawn")]
    assert drawn and all(not Path(a["file"]).is_absolute() for a in drawn)                 # relative to the project's folder
    assert all((Path(units.root) / a["file"]).is_file() for a in drawn)
    assert workflow.resolve_file(drawn[0]["file"], units.root) == Path(units.root) / drawn[0]["file"]
    try:
        for a in drawn:                                                                    # the lab's copy of this project
            a["file"] = f"/home/alai-labs/alix_studio_lite/Resources/Projects/{Path(units.root).name}/{a['file']}"
        f.write_text(json.dumps(sim), encoding="utf-8")
        fr = call(lite, "POST", f"{P}/workflow/frames", {"side": "left", "stage": 0})
        assert len(fr["times"]) > 10 and len(fr["links"]) > 20
        for a in drawn:                                                                    # a file that is nowhere
            a["file"] = f"/nowhere/{Path(a['file']).name}"
        f.write_text(json.dumps(sim), encoding="utf-8")
        r = call(lite, "POST", f"{P}/workflow/frames", {"side": "left", "stage": 0})
        assert r["code"] == "files" and "LEFT row 4, Program 71 (L_s2_a.txt) is not on this computer" in r["error"]
        assert "Simulation Mode" in r["hint"]
        j = wait(lite, call(lite, "POST", f"{P}/workflow/collisions", {"hash": sim["hash"], "phase": "lid_open"}))   # not cached yet
        assert j["state"] == "failed" and "is not on this computer" in j["error"], j
    finally:
        f.write_text(kept, encoding="utf-8")


def test_a_program_lite_did_not_send_is_drawn_from_its_file_found_by_name(lite):
    """The owner (16 Oct): the sequence must play in the simulation. A Program the control program sent (not
    Lite) is drawn from the file of the name the sequence gives it (V53's JSON), here the Trajectory Viewer's
    V6 package, and said so; a name found nowhere is said too."""
    left = [{"type": "pole", "mm": 150}, {"type": "hand", "values": [0, 0, 0, 0, 150, 1000]},
            {"type": "program", "id": 81, "name": "L_P01P02.txt", "transit_speed": 30, "dual_rule": "TOGETHER"},
            {"type": "program", "id": 82, "name": "L_NO_SUCH_FILE.txt", "transit_speed": 30, "dual_rule": "TOGETHER"}]
    call(lite, "PUT", f"{P}/workflow/left", {"steps": left})
    j = wait(lite, call(lite, "POST", f"{P}/workflow/simulate", {"mode": "left"}))
    assert j["state"] == "done", j
    acts = [a for a in j["result"]["acts"]["left"] if a["kind"] == "program"]
    assert acts[0]["drawn"] and acts[0]["by_name"] and acts[0]["file_name"] == "L_P01P02.txt" and acts[0]["lines"] > 100
    assert not acts[1]["drawn"]
    texts = " | ".join(f["text"] for f in j["result"]["findings"])
    assert "program 81: drawn from L_P01P02.txt (the Trajectory Viewer's" in texts and "found by its name; not compared" in texts
    assert "program 82 is not drawn (Lite has not sent this program; no file of its name (L_NO_SUCH_FILE.txt)" in texts
    fr = call(lite, "POST", f"{P}/workflow/frames", {"side": "left", "stage": 0, "hash": j["result"]["hash"]})
    assert len(fr["times"]) > 10                                   # the 3D has its motion


def test_a_file_found_by_name_is_compared_with_the_controllers_program():
    """A Program on a linked controller that Lite did not send: the file found by name is checked against the
    size the controller holds (the bytes the control program's upload makes of it); a controller's name cut
    to 31 characters is matched by its start."""
    vendor_paths.activate()
    from alix_studio_lite.server import library, workflow

    f = library.builtin_by_name("L_P01P02.txt")
    named = {"l_p01p02.txt": [dict(path=f["path"], name=f["name"], stage=f["stage"], phase=None, placement=None, region=None,
                                   source="the V6 package")]}
    size = workflow.proj_bytes(f["path"])
    assert size and size > Path(f["path"]).stat().st_size                 # CRLF, the header and the Folder line

    sim = workflow._ArmSim("left", {"pole_mm": 150, "hand": [0, 0, 0, 0, 150, 1000], "q_deg": [0] * 7}, [180] * 7, 50,
                           {}, None, workflow._md_reader(), [], named)
    same = sim._found(81, {"id": 81}, {"name": "81_L_P01P02.txt", "size": size})
    assert same["name"] == "L_P01P02.txt" and same["how"] == "its bytes make the size the controller holds"
    other = sim._found(81, {"id": 81}, {"name": "81_L_P01P02.txt", "size": size // 2})
    assert other["differs"] and "they may differ" in other["how"]
    cut = sim._found(81, {"id": 81}, {"name": "81_L_P01P0", "size": None})
    assert cut is None                                                     # short and not cut: no match by its start
    named["l_p01p02_with_a_very_long_name_for_it.txt"] = named["l_p01p02.txt"]
    held = "81_L_P01P02_with_a_very_long_name_for_it.txt"[:31]               # RealMan's API: 31 characters at most
    long = sim._found(81, {"id": 81}, {"name": held, "size": None})
    assert long is not None and long["differs"] is None
