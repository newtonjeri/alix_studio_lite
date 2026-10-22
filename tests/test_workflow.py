"""Merge + Sequence S2 (feedback v5, slide 9; its plan: the owner, 22 Oct): each arm's rows run top to bottom, both
arms starting together, and an arm waits only at a Meet (both arms meet), at a Wait-for (until the other arm has
finished the row it names) and at a lid or seat task (flow.py). V53's Dual Rule and its SAFE TIMELINE planner are gone;
a V53 file loads with its RUN FIRST / WAIT OTHER pairs as Meets and a Wait-for, which run them as V53 did (the owner's
V6 files here). The steps are checked as runner v14 checks them, each row with its uid; a run is a single task, a
dual-arm task or the sequences; Simulation Mode times both arms through the plan as the robot runs (each step's range,
measured), says what would not be right on the robot and checks collisions, the two arms against each other at every
timing their ranges allow; Real Machine Mode only after the same run completed in Simulation."""
import json
import threading
import time
import urllib.request
from pathlib import Path

import numpy as np
import pytest

from alix_studio_lite import vendor_paths

V6 = vendor_paths.LITE_ROOT / "tests/data/legacy_v6"


def v6():
    return [json.loads((V6 / f"{s}_V6_70SERIES_FINAL.json").read_text())["steps"] for s in ("LEFT", "RIGHT")]


def prog(i):
    return {"type": "program", "id": i, "name": "", "speed": 100, "transit_speed": 30}


def numbered(left, right):
    from alix_studio_lite.server import workflow

    return workflow._numbered({"left": workflow.check_steps(left, "left"), "right": workflow.check_steps(right, "right")})


def kinds(plan, side):
    from alix_studio_lite.server import workflow

    return [[(x["type"], x.get("id")) for x in part] for st in plan for s, part in workflow._items(st) if s == side]


def test_a_tracks_execution_time_is_its_lines_less_one_on_the_5_ms_clock():
    from alix_studio_lite.motion import track

    assert track.duration_s(300) == 1.495 and track.duration_s(1) == 0.0 and track.duration_s(0) is None
    assert track.duration_s(None) is None and track.duration_s("600") == 2.995


def test_each_arm_runs_its_rows_and_waits_only_where_the_sequence_says():
    """Both arms start together and go through their own rows; a Meet is a point both arms reach before either goes
    on (the plan's stages end there); a Wait-for holds its arm until the other arm has finished the row it names (its
    uid, kept as rows move); nothing else waits: no tail, no refusal for a Program on one arm only or used twice."""
    from alix_studio_lite.server import flow, workflow

    wait5 = {"type": "wait", "seconds": 5.0}
    steps = numbered([prog(80), wait5, prog(80), {"type": "meet"}, prog(84)], [prog(80), {"type": "meet"}, prog(82)])
    run = workflow.plan_for(steps, "both")
    assert [(st["kind"], st["program_id"]) for st in run["plan"]] == [("parallel", "PART 1"), ("parallel", "PART 2")]
    assert kinds(run["plan"], "left") == [[("program", 80), ("wait", None), ("program", 80)], [("program", 84)]]
    assert run["plan"][0]["meet_rows"] == {"left": 4, "right": 2} and run["counts"]["parts"] == 2
    left, right = workflow.check_steps([prog(84), prog(85)], "left"), workflow.check_steps([prog(81), prog(82)], "right")
    right.insert(1, {"type": "wait_for", "target": {"uid": left[0]["uid"]}})          # the right's 82 after the left's 84
    steps = workflow._numbered({"left": workflow.check_steps(left, "left"), "right": workflow.check_steps(right, "right")})
    plan = workflow.plan_for(steps, "both")["plan"]
    wait = plan[0]["right"]["steps"][1]
    assert wait["type"] == "wait_for" and wait["_target"] == {"side": "left", "row": 1, "index": 0, "done": False}
    moved = [left[1], left[0]]                                                           # the target moves: the wait follows it
    plan = workflow.plan_for(workflow._numbered({"left": moved, "right": workflow.check_steps(right, "right")}), "both")["plan"]
    assert plan[0]["right"]["steps"][1]["_target"]["row"] == 2
    alone = workflow.plan_for(steps, "right")                                          # RUN RIGHT: nothing to wait for
    assert [x["type"] for x in alone["plan"][0]["right"]["steps"]] == ["program", "program"] and "passed over" in alone["said"][0]
    for left, right, said in (
            ([prog(1), {"type": "meet"}], [prog(2)], "Each Meet needs its partner on the other arm: the LEFT list has 1"),
            ([{"type": "wait_for", "target": {"uid": "gone"}}, prog(1)], [prog(2)], "no longer there"),
            ([{"type": "wait_for", "target": {"program": 9}}, prog(1)], [prog(2)], "Program 9, which is not in the RIGHT list"),
            ([{"type": "meet"}, {"type": "mechanism", "task": "lid_open"}], [{"type": "mechanism", "task": "lid_open"}, {"type": "meet"}],
             "must come in the same order on both arms")):
        with pytest.raises(workflow.UserError, match=said):
            workflow.plan_for(numbered(left, right), "both")
    # each arm waiting for the other: refused, saying where each waits
    left = workflow.check_steps([prog(1), prog(2)], "left")
    right = workflow.check_steps([prog(3), prog(4)], "right")
    left.insert(0, {"type": "wait_for", "target": {"uid": right[1]["uid"]}})
    right.insert(0, {"type": "wait_for", "target": {"uid": left[2]["uid"]}})
    with pytest.raises(workflow.UserError, match="The arms would wait for each other for ever: LEFT row 1 waits for RIGHT row 3; "
                                                 "RIGHT row 1 waits for LEFT row 3"):
        workflow.plan_for(workflow._numbered({"left": workflow.check_steps(left, "left"),
                                              "right": workflow.check_steps(right, "right")}), "both")
    left = workflow.check_steps([prog(1), {"type": "meet"}, prog(2)], "left")              # waiting past a Meet: never ends
    right = workflow.check_steps([prog(3), {"type": "meet"}], "right")
    right.insert(0, {"type": "wait_for", "target": {"uid": left[2]["uid"]}})
    with pytest.raises(workflow.UserError, match="wait for each other for ever"):
        workflow.plan_for(workflow._numbered({"left": left, "right": workflow.check_steps(right, "right")}), "both")
    assert flow.counts(run["plan"])["waits_for"] == 0


def test_a_v53_file_loads_with_its_handoffs_as_meets_and_a_wait_for():
    """The owner's V6 files (V53's): TOGETHER needs nothing; Programs 78 and 79 (RUN FIRST on the left, WAIT OTHER on
    the right) run one arm at a time as V53 ran them: both arms meet before and after each, and the right waits for the
    left's. Lite's own files keep the rows and uids; a file of the other arm is refused."""
    from alix_studio_lite.server import workflow

    left_doc, right_doc = (json.loads((V6 / f"{s}_V6_70SERIES_FINAL.json").read_text()) for s in ("LEFT", "RIGHT"))
    left, right = workflow.from_document("left", left_doc), workflow.from_document("right", right_doc)
    assert all("dual_rule" not in x and x["uid"] for x in left + right)
    assert [x["type"] for x in left].count("meet") == 4 == [x["type"] for x in right].count("meet")
    run = workflow.plan_for(workflow._numbered({"left": left, "right": right}), "both")
    assert [st["program_id"] for st in run["plan"]] == ["PART 1", "PART 2", "PART 3", "PART 4"]
    assert kinds(run["plan"], "left")[1:3] == [[("program", 78)], [("program", 79)]]
    assert kinds(run["plan"], "right")[1:3] == [[("wait_for", None), ("program", 78)], [("wait_for", None), ("program", 79)]]
    waits = [st["right"]["steps"][0]["_target"] for st in run["plan"][1:3]]
    assert [(w["side"], left[w["index"]]["id"]) for w in waits] == [("left", 78), ("left", 79)]
    saved = workflow.document("left", left)
    assert saved["format"] == workflow.LITE_FORMAT and workflow.from_document("left", saved) == left
    with pytest.raises(workflow.UserError, match="LEFT sequence: it never loads into the RIGHT arm"):
        workflow.from_document("right", saved)


def test_rows_are_checked_with_their_uids_and_a_kept_sequence_is_brought_up_to_date(tmp_path):
    from alix_studio_lite.server import workflow
    from alix_studio_lite.server.jobs import UserError

    left_doc = json.loads((V6 / "LEFT_V6_70SERIES_FINAL.json").read_text())
    steps = workflow.from_document("left", left_doc)
    assert steps[3] == {"type": "program", "id": 71, "name": "L_P01P02.txt", "speed": 100, "transit_speed": 30, "uid": steps[3]["uid"]}
    assert [x for x in steps if x["type"] == "wait"][0] == {"type": "wait", "seconds": 2.0, "note": "MANUAL LID COVER OPEN",
                                                            "uid": [x for x in steps if x["type"] == "wait"][0]["uid"]}
    with pytest.raises(UserError, match="wrong sequence format"):
        workflow.from_document("left", {"steps": []})
    old = {"format": workflow.FORMAT, "side": "left", "steps": [{"type": "program", "id": 5, "speed": 40}]}
    assert workflow.from_document("left", old)[0]["transit_speed"] == 30                 # V53's v51 compatibility
    same = workflow.check_steps([dict(prog(1), uid="abc"), dict(prog(2), uid="abc"), prog(3)], "left")
    assert same[0]["uid"] == "abc" and len({x["uid"] for x in same}) == 3                 # a copy gets its own
    for bad, said in (({"type": "pole", "mm": 400}, "row 1: pole target must be 0-315 mm"),
                      ({"type": "hand", "values": [1, 2, 3]}, "row 1: hand step requires exactly 6 values"),
                      ({"type": "wait_for"}, "row 1: a Wait-for names the other arm's row it waits for"),
                      ({"type": "wait_for", "target": {"program": 300}}, "row 1: a Wait-for names a Program 1-100"),
                      ({"type": "jump"}, "row 1: workflow step type must be")):
        with pytest.raises(UserError, match=said.replace("(", r"\(")):
            workflow.check_steps([bad], "left")
    assert workflow.check_steps([dict(prog(1), dual_rule="TOGETHER")], "left")[0].get("dual_rule") is None
    # a sequence kept before 22 Oct (V53's Dual Rule, no uids) is brought to today's form once, and kept so
    units = type("U", (), {"root": str(tmp_path), "lock": threading.RLock()})()
    (tmp_path / "sequence").mkdir()
    l6, r6 = v6()
    (tmp_path / "sequence" / "s2.json").write_text(json.dumps({"left": l6, "right": r6, "updated": "then"}))
    first = workflow.load(units)
    assert all("dual_rule" not in x and x.get("uid") for s in ("left", "right") for x in first[s])
    assert workflow.load(units) == first                                               # the same uids every time
    assert [x["type"] for x in first["right"]].count("wait_for") == 2


def test_a_single_task_or_a_dual_arm_task():
    """What is run (the owner, 16 Oct): a Program row alone on its arm with the Pole and Hand in force before it; one
    part of the plan between two meeting points, each arm staged the same way, a Wait-for whose row is outside it done."""
    from alix_studio_lite.server import workflow
    from alix_studio_lite.server.jobs import UserError

    left, right = (workflow.check_steps(s, side) for s, side in zip(v6(), ("left", "right")))
    steps = workflow._numbered({"left": left, "right": right})
    task = workflow.plan_for(steps, "both", {"scope": "task", "side": "left", "row": 6})     # Program 73: hand (row 2), pole 5
    assert task["mode"] == "left" and task["pick"] == {"scope": "task", "side": "left", "row": 6} and "right" not in task["steps"]
    assert [(st["type"], st["_row"]) for st in task["steps"]["left"]] == [("hand", 2), ("pole", 5), ("program", 6)]
    assert [st["kind"] for st in task["plan"]] == ["single"] and task["plan"][0]["program_id"] == 73
    with pytest.raises(UserError, match="LEFT row 1 is not a Program row"):
        workflow.plan_for(steps, "both", {"scope": "task", "side": "left", "row": 1})
    full = workflow.plan_for(steps, "both")["plan"]
    dual = workflow.plan_for(steps, "both", {"scope": "dual", "stage": 1})                  # PART 2: left 78, right waits, 78
    assert [st["kind"] for st in dual["plan"]] == ["parallel"] and dual["mode"] == "both"
    for side in ("left", "right"):                                   # the pole and hand in force, then the stage's own rows
        kinds_ = [st["type"] for st in dual["steps"][side]]
        assert "program" in kinds_ and {"pole", "hand"} <= set(kinds_[:kinds_.index("program")])
    assert dual["plan"][0]["right"]["steps"][-2]["_target"]["done"] is False                # the left's 78 is in it: waited for
    ids = {side: [x["id"] for x in full[1][side]["steps"] if x["type"] == "program"] for side in ("left", "right")}
    assert workflow.plan_for(steps, "both", {"scope": "dual", "stage": 1, "ids": ids})["mode"] == "both"
    with pytest.raises(UserError, match="Stage 2 holds LEFT Program") as e:
        workflow.plan_for(steps, "both", {"scope": "dual", "stage": 1, "ids": dict(ids, left=[99])})
    assert e.value.code == "pick"
    with pytest.raises(UserError, match="LEFT row 6 holds Program 73 now, not Program 72") as e:
        workflow.plan_for(steps, "both", {"scope": "task", "side": "left", "row": 6, "id": 72})


def test_a_move_after_a_program_clears_the_arms_4105_first():
    """The lab, 21 Oct 22:29 and 22:41, 22 Oct 21:12: the Home MoveJ after an arm's last Program returned 1 at once and the
    arm went Home anyway; each Program had raised 4105 (a joint over its speed limit), which runner v14 lets stand, and the
    SDK's blocking move returns 1 while any code is latched (a 2 s wait after the Program changed nothing). Before such a
    move, after a Program or a lid or seat task on that arm, and at the run's start, Lite reads the arm's errors and clears
    a 4105 alone: a clear step of its own, between two runner calls; across stages; and before a lid or seat task."""
    from alix_studio_lite.server import workflow

    assert workflow.WARN_ONLY == {"4105"}
    home = {"type": "global", "name": "HOME", "move": "movej", "speed": 20, "wait_s": 0.0}
    left = [home, prog(80), {"type": "wait", "seconds": 0.5}, home, home, prog(81), {"type": "hand", "values": [1000] * 6}, home,
            {"type": "meet"}, prog(83)]
    right = [home, prog(80), prog(54), {"type": "meet"}, home]
    run = workflow.plan_for(numbered(left, right), "both")
    assert [st["kind"] for st in run["plan"]] == ["parallel", "parallel"]
    lp = run["plan"][0]["left"]["steps"]
    # row 1: the run's start; row 4: after Program 80 (a Wait between changes nothing); row 5: no Program since row 4's
    # clear; row 8: after Program 81 (a Hand between changes nothing)
    assert [x["_row"] for x in lp if x["type"] == "clear"] == [1, 4, 8]
    assert [x["type"] for x in lp[:3]] == ["clear", "global", "program"]
    rp = run["plan"][1]["right"]["steps"]                          # after a Meet, from the Program before it
    assert [x["type"] for x in rp] == ["clear", "global"] and rp[0]["_row"] == 5
    assert [w for w, _ in workflow._segments(lp)] == ["clear", "steps", "clear", "steps", "clear", "steps"]   # a runner call each
    assert "4105" in workflow._step_words(rp[0])
    alone = workflow.plan_for(numbered(left, [home, prog(80), prog(54), home]), "right")["plan"][0]["right"]["steps"]
    assert [x["type"] for x in alone] == ["clear", "global", "program", "program", "clear", "global"]
    task = {"type": "mechanism", "task": "lid_open"}
    after = workflow.plan_for(numbered([prog(80), task], [prog(80), dict(home), task]), "both")
    assert [(st["kind"], st.get("clear")) for st in after["plan"]] == [("parallel", None), ("mechanism", ["left"])]   # the right's Home's
    first = workflow.plan_for(numbered([task, prog(80)], [task, prog(81)]), "both")
    assert [(st["kind"], st.get("clear")) for st in first["plan"]] == [("mechanism", ["left", "right"]), ("parallel", None)]
    ready = workflow.plan_for(numbered([home, task], [home, task]), "both")      # the Homes' clears were the last motion
    assert ready["plan"][-1]["kind"] == "mechanism" and "clear" not in ready["plan"][-1]


def test_after_the_lid_and_seat_tasks_each_arm_gets_its_pole_and_hand_back():
    """The owner, 22 Oct: "after the lid and seat opening tasks, each arm moves back to the pole and hand state it had
    before those tasks started; then the dual-arm tasks that follow are triggered on both arms at the same time". On the
    last task of each run of them (the stages keep their numbers): each arm's last Pole and Hand rows before the run's
    first task, None where no row set them (then as the arm stood). A single task picked alone puts nothing back."""
    from alix_studio_lite.server import workflow

    lid, seat = {"type": "mechanism", "task": "lid_open"}, {"type": "mechanism", "task": "seat_open"}
    grip, opened = [0, 0, 0, 0, 150, 1000], [1000] * 6
    left = [{"type": "pole", "mm": 50}, {"type": "hand", "values": grip}, {"type": "meet"}, prog(80), {"type": "pole", "mm": 120},
            lid, seat, prog(81), seat]
    right = [{"type": "pole", "mm": 200}, {"type": "meet"}, prog(80), lid, seat, {"type": "hand", "values": opened}, prog(82), seat]
    run = workflow.plan_for(numbered(left, right), "both")
    kinds = [(st["kind"], st.get("task"), bool(st.get("restore"))) for st in run["plan"]]
    assert kinds == [("parallel", None, False), ("parallel", None, False), ("mechanism", "lid_open", False),
                     ("mechanism", "seat_open", True), ("parallel", None, False), ("mechanism", "seat_open", True)]
    back = run["plan"][3]["restore"]
    assert back["since"] == "Open the lid"
    assert back["left"] == {"pole_mm": 120.0, "hand": grip} and back["right"] == {"pole_mm": 200.0, "hand": None}
    again = run["plan"][5]["restore"]                           # the second run: the rows since the first one count
    assert again["left"] == {"pole_mm": 120.0, "hand": grip} and again["right"] == {"pole_mm": 200.0, "hand": opened}
    st = run["plan"][3]
    assert workflow.restore_steps(st, "right", 75.0, [500] * 6) == [
        {"type": "pole", "mm": 200.0, "_row": 5, "restore": True}, {"type": "hand", "values": [500] * 6, "_row": 5, "restore": True}]
    assert "pole and hand back to before Open the lid" in workflow.restore_title(st)
    alone = workflow.plan_for(numbered(left, right), "both", {"scope": "dual", "stage": 3})
    assert alone["plan"][0]["kind"] == "mechanism" and "restore" not in alone["plan"][0]


def test_the_same_steps_planned_otherwise_are_simulated_again():
    """A simulation is of a plan, not only of steps: one made before Lite planned the same steps otherwise (V53's planner
    gone on 22 Oct) no longer lets Real Machine Mode run them, and the refusal says so."""
    from alix_studio_lite.server import workflow

    run = workflow.plan_for(numbered([prog(71), {"type": "wait", "seconds": 2.0}, prog(72)], [prog(81), prog(82)]), "both")
    none = {"left": {}, "right": {}}, {"left": None, "right": None}
    now = workflow.identity_of(run, *none, 15)
    old = dict(now, plan=[{"kind": "parallel", "left": [1, 2], "right": [1]}, {"kind": "tail", "left": [3], "right": [2]}])
    assert old["steps"] == now["steps"] and workflow.digest(old) != workflow.digest(now)
    said = workflow._changed({"identity": old}, now)
    assert said == "changed since it was simulated: how its steps are planned into stages (Lite plans them otherwise since)"
    assert "planned into stages" in workflow._changed({"identity": {k: v for k, v in now.items() if k != "plan"}}, now)


def test_each_step_is_timed_as_the_robot_runs_it():
    """The lab's logs of 21 Oct (ROBOT_TIMES): a Program takes the controller's read of its file, its lines at 96-102 % of
    5 ms each, then its end; a move to its first point only when the arm is not there; a Hand runner v14's 0.75-1.3 s; a
    Pole ALIX's law and a little; a Global Point where the arm already is next to nothing. Each step keeps its range,
    and a timing ``lam`` puts every step at that point of it."""
    from alix_studio_lite.server import workflow

    md = np.zeros((2001, 7))
    md[:, 0] = np.linspace(0, 20000, 2001)                           # 0 to 20 deg on J1 over 10 s
    files = {"/f.txt": md}
    progs = {80: dict(file="/f.txt", sent=True, lines=2001)}
    pts = {"points": {"HOME": [0.0] * 7}}
    start = dict(pole_mm=200.0, hand=[1000] * 6, q_deg=[0.0] * 7)

    def sim(lam=None):
        return workflow._ArmSim("left", start, [180, 180, 225, 225, 225, 225, 225], 15, progs, pts, files.get, [], lam=lam)

    steps = [dict(prog(80), _row=1), {"type": "hand", "values": [0, 0, 0, 0, 150, 1000], "_row": 2},
             {"type": "pole", "mm": 50.0, "_row": 3}, {"type": "global", "name": "HOME", "move": "movej", "speed": 20, "wait_s": 0.0,
                                                       "_row": 4},
             {"type": "global", "name": "HOME", "move": "movej", "speed": 20, "wait_s": 0.0, "_row": 5}]
    a = sim()
    a.run(steps, 0.0, 0)
    p, h, pl, g, there = a.acts
    assert p["traj_t0"] == p["move_t0"] == pytest.approx(0.05 + 2001 * 0.23e-3, abs=1e-3)    # no move to its first point
    assert p["times"][0] == pytest.approx(0.05 + 2001 * 0.17e-3 + 0.96 * 10 + 0.05)
    assert p["times"][2] == pytest.approx(0.05 + 2001 * 0.37e-3 + 1.02 * 10 + 0.55)
    assert p["line_s"] == pytest.approx(0.005, abs=1e-6) and steps[0]["_t"]["play_s"] == 10.0
    assert h["times"] == pytest.approx((0.75, 0.82 + 0.25, 1.05 + 0.25))
    assert pl["times"][0] > 3.0 and pl["times"][2] - pl["times"][0] == pytest.approx(0.15)
    assert g["times"][1] > 2.0 and there["times"] == pytest.approx((0.05, 0.08, 0.15))
    fast, slow = sim(0.0), sim(1.0)
    for x in (fast, slow):
        x.run(steps, 0.0, 0)
    assert fast.acts[-1]["t1"] < a.acts[-1]["t1"] < slow.acts[-1]["t1"]
    assert fast.acts[0]["line_s"] == pytest.approx(0.0048, abs=1e-6) and slow.acts[0]["line_s"] == pytest.approx(0.0051, abs=1e-6)
    away = workflow._ArmSim("left", dict(start, q_deg=[-30.0, 0, 0, 0, 0, 0, 0]), [180, 180, 225, 225, 225, 225, 225], 15, progs,
                            pts, files.get, said := [])
    away.run([dict(prog(80), _row=1)], 0.0, 0)
    tr = away.acts[0]["transit"]
    assert tr[0] == pytest.approx(30 / (0.3 * 180) + 0.5, abs=1e-3) and tr[2] == pytest.approx(30 / (0.3 * 0.15 * 180) + 0.5, abs=1e-3)
    assert away.acts[0]["traj_t0"] > away.acts[0]["move_t0"] and "first moves the arm 30 deg" in said[0]["text"]


def test_a_wait_for_holds_its_arm_until_the_other_has_finished_its_row():
    """In the simulation a Wait-for is a hold until the other arm's row ends (an action, so that every timing of the run
    has the same actions); the arm that ends a stage first holds for the other at the Meet."""
    from alix_studio_lite.server import workflow

    md = np.zeros((1001, 7))
    progs = {80: dict(file="/f.txt", sent=True, lines=1001), 81: dict(file="/f.txt", sent=True, lines=1001)}
    left = workflow.check_steps([prog(80), {"type": "meet"}, prog(81)], "left")
    right = workflow.check_steps([{"type": "wait", "seconds": 1.0}, prog(81), {"type": "meet"}], "right")
    right.insert(1, {"type": "wait_for", "target": {"uid": left[0]["uid"]}})
    run = workflow.plan_for(workflow._numbered({"left": left, "right": workflow.check_steps(right, "right")}), "both")
    start = dict(pole_mm=200.0, hand=[1000] * 6, q_deg=[0.0] * 7)
    sims = {s: workflow._ArmSim(s, start, [180] * 2 + [225] * 5, 15, progs, None, {"/f.txt": md}.get, []) for s in ("left", "right")}
    stages, total = workflow.schedule(run, sims, 15)
    L, R = sims["left"].acts, sims["right"].acts
    hold = next(a for a in R if a.get("waits_for"))
    assert hold["t0"] == pytest.approx(R[0]["t1"], abs=1e-3) and hold["t1"] == pytest.approx(L[0]["t1"], abs=1e-3)   # until 80 ends
    assert R[2]["t0"] > L[0]["t1"]                                                                     # its 81 after that
    meet = [a for a in L if a.get("gate") and a["stage"] == 0][0]
    assert meet["text"] == "Meet the RIGHT arm" and meet["t1"] == pytest.approx(stages[0]["t1"], abs=1e-3)
    assert stages[1]["t0"] == stages[0]["t1"] and total == pytest.approx(max(L[-1]["t1"], R[-1]["t1"]), abs=1e-3)


def _drift_case(wait_for: bool):
    """The lab's near-collision of 21 Oct 23:18 in small (the E-stop pressed just before the arms touched): the left arm
    late in its program 84, the right arm's program 82 after six Hand steps and a short Wait (tests/data/s2_drift)."""
    from alix_workspace.collision import RobotCollisionModel

    from alix_studio_lite.motion import track
    from alix_studio_lite.server import robot_model, staging, workflow

    vendor_paths.activate()
    data = vendor_paths.LITE_ROOT / "tests/data/s2_drift"
    L, R = (np.asarray(track.read_md(data / f), float) for f in ("L84_tail.txt", "R82_head.txt"))
    md, grip, open_ = {"/L.txt": L, "/R.txt": R}, [0, 0, 0, 0, 150, 1000], [1000] * 6
    left = workflow.check_steps([prog(1)], "left")
    right = [{"type": "hand", "values": v} for v in (open_, grip) * 3] + [{"type": "wait", "seconds": 0.774}, prog(2)]
    if wait_for:
        right.insert(len(right) - 1, {"type": "wait_for", "target": {"uid": left[0]["uid"]}})
    run = workflow.plan_for(workflow._numbered({"left": left, "right": workflow.check_steps(right, "right")}), "both")
    progs = {"left": {1: dict(file="/L.txt", sent=True, lines=len(L))}, "right": {2: dict(file="/R.txt", sent=True, lines=len(R))}}
    starts = {"left": dict(pole_mm=200.0, hand=grip, q_deg=(L[0] / 1000).tolist()),
              "right": dict(pole_mm=200.0, hand=grip, q_deg=(R[0] / 1000).tolist())}

    def make_sims(lam):
        return {s: workflow._ArmSim(s, starts[s], [180, 180, 225, 225, 225, 225, 225], 15, progs[s], None, md.get, [], lam=lam.get(s))
                for s in ("left", "right")}

    sims = make_sims({"left": None, "right": None})
    workflow.schedule(run, sims, 15)
    model = robot_model.fk_model()
    base = robot_model._home(model, RobotCollisionModel.from_file(str(vendor_paths.URDF), tag="visual").index)
    for s in ("left", "right"):
        base = staging.start_values(s, base, starts[s])
    job = type("Job", (), {"progress": lambda self, *a: None, "sleep": lambda self, x: time.sleep(x), "check": lambda self: None})()
    return workflow._drift(dict(run, start=starts, md_of=md.get), make_sims, 15, {s: sims[s].acts for s in sims}, base, job)


def test_the_two_arms_are_checked_at_every_timing_their_ranges_allow():
    """The owner, 22 Oct: with each step's range known, the arms are checked against each other at every timing those
    ranges allow, CLEARANCE_MM apart. The lab's programs 84 and 82: as timed as a rule the right arm's opening swing
    passes the left hand clear, but with the right arm's Hand steps at their quickest and the left's program at its
    slowest it reaches the left hand: said, with how far ahead the right arm was then. A Wait-for the left's 84 before
    the right's 82 keeps them apart at every timing."""
    out = _drift_case(wait_for=False)
    assert out["timings"] >= 10 and out["clearance_mm"] == 50 and out["band_s"] > 5
    assert out["events"] and not any(e["lam"] is None for e in out["events"])           # as a rule: clear
    e = out["events"][0]
    assert e["right_ahead_s"] > 1.0 and e["min_mm"] < 50 and "IRH_right_thumb_2" in e["links"]
    said = out["findings"][0]["text"]
    assert "The two arms come closer than 50 mm: LEFT row 1 (Program 1" in said and "when the RIGHT arm runs" in said
    assert "keep the arms apart with a Meet or a Wait-for" in said and out["findings"][0]["level"] == "warn"
    kept = _drift_case(wait_for=True)
    assert kept["events"] == [] and kept["findings"] == [] and kept["timings"] >= 10


def test_the_clearance_check_measures_the_gap_between_the_arms():
    """At the lab's stop of 21 Oct 23:18:53 the two arms stood 12 mm apart (the left wrist's camera holder, the right
    thumb): closer than 50 mm, said with the exact gap; at the model's standing pose they are far apart."""
    from alix_workspace.collision import RobotCollisionModel

    from alix_studio_lite.server import collision_check, robot_model, staging

    vendor_paths.activate()
    model = robot_model.fk_model()
    home = robot_model._home(model, RobotCollisionModel.from_file(str(vendor_paths.URDF), tag="visual").index)
    stop = {"left": [-25.458, -1.705, -15.515, 35.885, -129.774, -64.576, 157.298],
            "right": [36.811, -11.173, -28.265, 112.847, -19.411, -75.141, 75.4]}
    hands = {"left": [555, 524, 512, 543, 525, 986], "right": [998, 1000, 1000, 1000, 995, 987]}
    standing = {s: [float(np.degrees(home[f"{staging.SIDE_PREFIX[s]}_joint{i}"])) for i in range(1, 8)] for s in ("left", "right")}

    def case(q):
        return dict(times=[0.0, 0.04], arms={s: dict(pole=np.array([200.0, 200.0]), hand=np.array([hands[s]] * 2, float),
                                                      q=np.array([q[s]] * 2, float), known=[True, True]) for s in ("left", "right")})

    near, far = collision_check.clearance_states([case(stop), case(standing)], home, 50.0)
    assert len(near["events"]) == 1 and far["events"] == []
    e = near["events"][0]
    assert e["exact_mm"] == pytest.approx(12.2, abs=1.0) and e["min_mm"] <= e["exact_mm"] + 0.1        # a hull is never farther
    assert e["first_t"] == 0.0 and e["last_t"] == 0.04


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
    kept = call(lite, "PUT", f"{P}/workflow/right", {"steps": right})["right"]
    assert {k: v for k, v in kept[2].items() if k != "uid"} == {"type": "wait", "seconds": 1.5} and kept[2]["uid"]
    assert all("dual_rule" not in x for x in kept)                   # V53's Dual Rule: gone (22 Oct)
    plan = call(lite, "POST", f"{P}/workflow/plan", {"mode": "both"})
    assert [s["kind"] for s in plan["stages"]] == ["parallel"] and plan["counts"]["parts"] == 1      # no tail of V53's

    j = wait(lite, call(lite, "POST", f"{P}/workflow/simulate", {"mode": "both"}))
    assert j["state"] == "done", j
    sim = j["result"]
    st = sim["stages"]
    assert st[0]["t0"] == 0 and st[0]["t1"] == pytest.approx(sim["total_s"], abs=1e-3)
    assert st[0]["t1_range"][0] <= st[0]["t1"] <= st[0]["t1_range"][1]                  # as fast and as slow as they run
    assert sim["drift"]["clearance_mm"] == 50 and sim["drift"]["timings"] >= 1          # both arms, at every timing
    la, ra = sim["acts"]["left"], sim["acts"]["right"]
    assert [a["kind"] for a in la] == ["pole", "hand", "pole", "program", "program"] and [a["row"] for a in la] == [1, 2, 3, 4, 5]
    assert la[3]["drawn"] and la[3]["lines"] == 300 and 299 * la[3]["line_s"] == pytest.approx(299 * 0.005, abs=1e-3)
    assert la[3]["t1"] > la[3]["traj_t0"] + 1.495 and la[3]["traj_t0"] > la[3]["t0"]      # the file read first, the end after
    assert sim["programs"]["left"]["71"]["seconds"] == pytest.approx(1.495)                  # its execution time travels with it
    src = call(lite, "GET", f"{P}/workflow/left/sources")
    assert next(p for p in src["programs"] if p["id"] == 71)["seconds"] == pytest.approx(1.495)
    assert not la[4]["drawn"]                                    # program 72: Lite never sent it
    assert ra[1]["t0"] == ra[0]["t1"] and ra[2]["kind"] == "wait" and ra[2]["stage"] == 0   # its own, straight after its 71
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


def test_a_program_sent_from_advanced_is_drawn_from_the_file_lite_kept(lite):
    """The lab, 21 Oct: Programs 20, 99 and 100 were sent from Advanced, whose files Lite keeps in its data folder, not in
    the project, so the simulation could not draw them and left the right arm out of the check for 48 s. The very file
    Lite sent is drawn while it holds the same bytes."""
    from alix_studio_lite.motion import track
    from alix_studio_lite.server import workflow
    from alix_studio_lite.server.lite_units import LiteUnits

    md = np.round(np.tile(np.array([[36.8, -60.6, -37.0, 116.0, -32.9, -52.7, 90.3]]), (120, 1)) * 1000).astype(np.int64)  # a hold
    path = lite["data"] / "advanced" / "right" / "20261021_225804_delay_12s.txt"
    path.parent.mkdir(parents=True, exist_ok=True)
    track.write_md(path, md)
    arm = lite["app"].robot.arm("right")
    arm.programs[100] = dict(name="delay_12s", lines=120, bytes=path.stat().st_size, sha256=track.sha256_file(path), stage=None,
                             first_row=md[0].tolist(), uploaded="2026-10-21 22:58", path=str(path))
    units = LiteUnits(lite["app"].store.folder(PID), "commode_c")
    try:
        known = workflow.programs_known(lite["app"], units, "right")[100]
        assert known["file"] == str(path) and known["lines"] == 120 and known["sent"]
        path.write_bytes(path.read_bytes() + b"{}\n")                       # other bytes since: not the program it sent
        assert workflow.programs_known(lite["app"], units, "right")[100]["file"] is None
    finally:
        arm.programs.pop(100, None)


def test_a_saved_sequence_is_the_file_and_loads_back(lite):
    """The lab, 22 Oct: S2's Save wrote an empty JSON file and Load said it could not be read. The sequence is a JSON
    file, and the page took it for an answer to read (its own refusals are JSON too) instead of the file's bytes: the
    server marks every file it sends (X-Lite-Raw), which the page keeps as bytes. A file saved empty before that is
    said to be empty."""
    steps = [{"type": "wait", "seconds": 1.5}, {"type": "pole", "mm": 120}]
    call(lite, "PUT", f"{P}/workflow/left", {"steps": steps})
    r = urllib.request.Request(lite["base"] + f"{P}/workflow/left/download", headers={"X-Lite-Token": lite["tok"]})
    with urllib.request.urlopen(r, timeout=60) as resp:
        assert resp.headers.get("X-Lite-Raw") == "1" and resp.headers.get("Content-Type") == "application/json"
        assert "left_workflow_sequence.json" in resp.headers.get("Content-Disposition", "")
        text = resp.read().decode()
    doc = json.loads(text)
    assert doc["side"] == "left" and len(doc["steps"]) == 2
    call(lite, "PUT", f"{P}/workflow/left", {"steps": []})
    out = call(lite, "POST", f"{P}/workflow/left/load", {"document": text})
    assert [(s["type"], s.get("seconds"), s.get("mm")) for s in out["left"]] == [("wait", 1.5, None), ("pole", None, 120.0)]
    empty = call(lite, "POST", f"{P}/workflow/left/load", {"document": "  "})
    assert "the file is empty" in empty["error"] and "save the sequence again" in empty["hint"]
    refused = call(lite, "POST", f"{P}/workflow/right/load", {"document": text})
    assert "error" in refused                                          # another arm's file
    call(lite, "PUT", f"{P}/workflow/right", {"steps": []})
    r = urllib.request.Request(lite["base"] + f"{P}/workflow/right/download", headers={"X-Lite-Token": lite["tok"]})
    with urllib.request.urlopen(r, timeout=60) as resp:                 # a refusal is not marked as a file
        assert resp.headers.get("X-Lite-Raw") is None and "empty" in json.loads(resp.read())["error"]
