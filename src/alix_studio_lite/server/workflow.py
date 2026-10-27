"""Merge + Sequence, S2: Sequence (feedback v5, slide 9; the owner, 16 Oct; its plan, 22 Oct).

The control program's Page 02 (``rm_trajectory_ui_v53_safe_lid_checkpoint.py``, run by runner v14) in Lite: each arm's
ordered rows (Program with its Start %: the controller's move to its first point, the stored timing kept; Global
Point; Pole; Hand; Wait), and Lite's own: Meet, Wait-for and the lid and seat tasks. Each arm's rows run top to bottom,
both arms starting together; an arm waits only at a Meet (both arms meet there), at a Wait-for (until the other arm has
finished the row it names) and at a lid or seat task (it uses both arms): ``flow.py`` (the owner, 22 Oct: V53's Dual
Rule and its SAFE TIMELINE planner are gone; a V53 file's RUN FIRST / WAIT OTHER pairs load as Meets and a Wait-for). A
Wait is a task of its own: its arm idles for its seconds. Each arm's part of a stage runs through runner v14's own
``run_workflow_sequence`` on that arm's controller, a call per stretch between its Wait-for rows. Before a move that
waits for its arrival, after a Program, Lite clears the arm's 4105 warning when it holds one (``clear_before_moves``:
the lab, 21 and 22 Oct). Saved and loaded as Lite's JSON (``LITE_FORMAT``); V53's own (``FORMAT``) loads too.

What Lite moves itself (the owner, 24 Oct): each Program's own pole and hand, set before it, the pole and the hand
together (``own_stages``: in place of the Pole and Hand rows right before it); after a lid or seat task, or a run of
them, each arm to its next task's start, its pole and hand and then its arm (``next_after_tasks``: in place of 22 Oct's
pole and hand put back as before the tasks).

What Lite adds, none of it motion:
- Simulation Mode: both arms in 3D through the same plan, with the collision check (walls, commode, the two arms
  against each other), and what would not be right on the robot: a Program whose pole or hand is not the one it was
  taught with, a Program Lite did not send (not drawn), a global point that could not be read;
- timing as the robot keeps it (the owner, 22 Oct): every step with the time it takes at least and at most (measured
  in the lab's logs: ``ROBOT_TIMES``), and the two arms checked against each other at every timing those ranges allow
  between their meeting points, with CLEARANCE_MM between them;
- Real Machine Mode only after the same run (its steps, its plan, programs, points, speed) completed in Simulation, or
  with the operator's word that it runs without; a step that runs outside its range is said, and stops both arms when
  far outside it (STOP_PROGRAM_OVER / STOP_PROGRAM_UNDER, STOP_STEP_MARGIN);
- before the first move, every Program slot and global point checked on its controller; a loaded file of the other arm
  refused; the overall speed is the top bar's slider, and each controller's own speed, found when it was connected, is
  put back after; STOP latches, as everywhere in Lite.
"""
from __future__ import annotations

import copy
import hashlib
import json
import time
from pathlib import Path

from . import flow
from .jobs import UserError

FORMAT = "realman_workflow_sequence_v1"           # V53's JSON: loaded (its Dual Rule as Meets and a Wait-for)
LITE_FORMAT = "alix_studio_lite_sequence_v1"       # Lite's: V53's rows, Lite's Meet and Wait-for, each row's uid
#: A move that waits for its arrival (a Global Point's MoveJ / MoveL; a lid or seat task's moves to its start, ALIX's park
#: MoveJ and its stream's bridge) returns 1 at once while the arm holds any error code, the warning 4105 (a joint over its
#: speed limit) too: once the controller has taken the move, the SDK's blocking call reads the arm's state and returns 1
#: on any code (RealMan's libapi_c, rm_movej_block; ALIX's dual_arm_common.preflight_error_gate: "every later motion
#: command is then rejected with ret=1"), and the controller goes on to the point all the same. Programs raise 4105 now
#: and then and runner v14 lets it stand (its WARN_ONLY), so the lab's Home after the last Program "failed" as the arm
#: went Home (21 Oct 22:29 and 22:41, 22 Oct 21:12: each Program had raised 4105; a 2 s wait after it changed nothing, as
#: the SDK drops the controller's end-of-program message and clears old arrivals before such a move). Before such a move,
#: after a Program or a lid or seat task on that arm (or at the run's start), Lite reads the arm's errors and clears a
#: 4105 alone, as the controller's Clear Error button does (the plan's ``clear`` step, ``clear_before_moves``); any other
#: code is not a warning and stops the run there.
WARN_ONLY = frozenset({"4105"})
ARMS = ("left", "right")
MAX_STEPS = 500

#: What each step takes on the robot: (least, as a rule, most), from runner v14's own lines in the lab's logs of 21 Oct
#: 2026 with the controllers' realtime push off (Lite turns it off as it connects: with it on, programs ran some 13 %
#: slower). A Program's lines against (lines - 1) x 5 ms: the owner's 96-102 % (22 Oct). Before the arm moves, the
#: controller reads the file (the "start ack": 0.17-0.37 ms a line, 36 runs); its checks, the end's report and the
#: runner's settle after it add 0.1-0.6 s. Runner v14's Hand (its raw hand at SPEED_SET 1000, opened and closed for the
#: step): 0.82-1.22 s over 32 steps, longer with the stroke. A Pole: ALIX's law plus 0.02-0.10 s (21 steps). A Global
#: Point's MoveJ: the controller's MoveJ law (``movej_s``) within MOVEJ_BAND; 0.08 s when the arm is already there (6
#: moves). A Wait: its seconds. Each runner call (a stage's part, or its stretch to a Wait-for) starts in 0.10-0.30 s.
#: Lite's read of an arm's errors before a move, with a clear of a 4105 (``clear``): four reads, then a clear and four
#: reads again (not yet timed in the lab). A Pole with the Hand after it, or a Program's own pole and hand (``pole_hand``,
#: Lite's step): the hand's port opened at its speed, the hand sent and the pole read (``pair_open_s``), then the longer
#: of the pole's travel (ALIX's law, which the control program's Pole.move waits out) and the hand's stroke (hand.py:
#: 0.115 s + 0.373 s per 1000 counts at SPEED_SET 1000, at most its own Hand step), then the hand confirmed and its port
#: closed (``pair_end_s``). The lab, 24 Oct (8 steps): 1.0-1.3 s with no pole travel, the law + 1.0-1.5 s with it, as its
#: hand was then sent again and waited for from the start; since, it is timed from the command it moved by.
ROBOT_TIMES = dict(
    play=(0.96, 1.00, 1.02),
    load_s_per_line=(0.17e-3, 0.23e-3, 0.37e-3),
    program_extra_s=(0.10, 0.20, 0.60),
    hand_s=(0.75, 0.82, 1.05), hand_s_per_1000=0.25,
    pole_extra_s=(0.0, 0.06, 0.15),
    global_there_s=(0.05, 0.08, 0.15),
    wait_extra_s=(0.0, 0.005, 0.02),
    call_start_s=(0.10, 0.17, 0.30),
    clear_s=(0.03, 0.10, 0.60),
    pair_open_s=(0.30, 0.45, 0.60), pair_end_s=(0.05, 0.20, 1.00), hand_stroke_s=(0.115, 0.373),
)
#: The controller's MoveJ (a Global Point's, and Lite's own to a start), as the lab's 133 MoveJs of 20-24 Oct ran (2 to
#: 286 deg, at 15 to 100 %): each joint cruises at the move's % of its rating (the move's speed x the global speed), after
#: MOVEJ_ACCEL_S of acceleration at any speed, the slowest joint setting the time; a move too short to cruise is a
#: triangle (24 Oct, 31.9 deg on J6 at 100 %: 0.56 s; at 15 %, 7.6 deg in 0.73 s and 286 deg in 9.1 s). Against the 133:
#: within -14 % (short moves at 15 %) and +9 % (J2, under load), MOVEJ_BAND.
MOVEJ_ACCEL_S = 0.57
MOVEJ_CALL_S = 0.03
MOVEJ_BAND = (0.85, 1.0, 1.12)
#: A Program's move to its first point when the arm is not there (the controller's own, at the Program's Start %): slower
#: than a MoveJ, and not by the global speed: at a Start % of 30 its joints cruise at 0.2077 x 30 % of their rating (14.0
#: deg/s on J6) after TRANSIT_ACCEL_S; the lab's three (32, 51 and 139 deg on J6 in 3.0, 4.4 and 10.6 s, at a global 100
#: and 15 %) within 2 %. Another Start % is taken as proportional (not measured).
TRANSIT_RATING_SHARE = 0.2077
TRANSIT_ACCEL_S = 0.69
TRANSIT_BAND = (0.90, 1.0, 1.15)
TRANSIT_MIN_DEG = 0.5
#: The owner, 22 Oct: on the robot, a step outside its range is said; a Program whose lines run past 105 % of their
#: time, or that ends before 90 % of it (stopped on the controller: an E-stop gives no code), stops both arms; any other
#: step more than 10 % outside its range stops both arms.
STOP_PROGRAM_OVER = 1.05
STOP_PROGRAM_UNDER = 0.90
STOP_STEP_MARGIN = 0.10
#: The two arms are kept this far apart at every timing their ranges allow (the owner, 22 Oct: 50 mm; the gloves, the
#: pad, the brush and a held tool are not in the model).
CLEARANCE_MM = 50.0


def clear_before_moves(plan: list[dict]) -> list[dict]:
    """``plan`` with Lite's ``clear`` step (WARN_ONLY) before each move that waits for its arrival on an arm that has
    run a Program or a lid or seat task since its last one, in the plan's order across its stages (the run's start
    counts as one: a run before it may have left a 4105): a ``clear`` step before such a Global Point (the point's row),
    or ``clear`` on a lid or seat task (the arms whose errors are read before it starts)."""
    moved = {s: True for s in ARMS}                # each arm: a Program or a lid or seat task since its last clear
    out = []
    for st in plan:
        if st["kind"] == "mechanism":
            mine = [s for s in ARMS if moved[s]]
            out.append(dict(st, clear=mine) if mine else st)
            moved = {s: True for s in ARMS}        # its own streams may leave one too
            continue
        st = copy.deepcopy(st)
        for side, steps in _items(st):
            mine = []
            for x in steps:
                if x["type"] == "global" and moved[side]:
                    mine.append({"type": "clear", "_row": x.get("_row")})
                    moved[side] = False
                elif x["type"] == "program":
                    moved[side] = True
                mine.append(x)
            steps[:] = mine
        out.append(st)
    return out


def next_after_tasks(plan: list[dict]) -> list[dict]:
    """``plan`` (a whole sequence of both arms) with ``then`` on the last lid or seat task of each run of them (the owner,
    24 Oct: "after any lid or seat task, open or close, move to the start state of the next task: pole, hand and arm", in
    place of 22 Oct's pole and hand put back as they were before the tasks): for each arm, the next row that moves it,
    past its Pole and Hand rows (that start's own: taken out of the plan, the move does them) and past a Meet that
    neither arm moves before:
    - a Program: its own pole and hand (``with_next_starts``; the rows' where Lite does not know its own), then the arm
      to its first point; both arms at once, so that the next stage starts both arms at their Programs' first points;
    - a Global Point: the rows' pole and hand; the point's own row moves the arm;
    - a Wait, a Wait-for, another task, nothing: no ``then``; the arm ends at its plan's end, as before.
    An arm with a ``then`` does not first go to its plan's end (a skipped plan's end, the final check's way back to where
    the plan ended). On the task's stage, not a stage of its own: the stages keep the numbers the Dual-arm task picks
    them by (``flow.build``)."""
    plan = copy.deepcopy(plan)
    for i, st in enumerate(plan):
        if st["kind"] != "mechanism" or (i + 1 < len(plan) and plan[i + 1]["kind"] == "mechanism"):
            continue
        then = {s: t for s in ARMS if (t := _next_task(plan, i, s)) is not None}
        for side, t in then.items():                    # the rows its start does
            for j, x in t.pop("_taken"):
                plan[j][side]["steps"][:] = [y for y in plan[j][side]["steps"] if y is not x]
        if then:
            st["then"] = then
    return plan


def _next_task(plan: list[dict], i: int, side: str) -> dict | None:
    """An arm's next task after the lid or seat task of stage ``i`` (next_after_tasks): its row, kind (program, global),
    Program id or point name, the pole and hand its rows set before it (None: not set), those rows, and ``_taken``
    (stage, step) of each; None when the next row that moves the arm is neither."""
    taken, pole, hand = [], None, None
    for j in range(i + 1, len(plan)):
        st = plan[j]
        if st["kind"] != "parallel":
            return None                                 # another lid or seat task
        for x in (st.get(side) or {}).get("steps") or []:
            if x["type"] == "pole":
                taken.append((j, x))
                pole = float(x["mm"])
            elif x["type"] == "hand":
                taken.append((j, x))
                hand = [int(v) for v in x["values"]]
            elif x["type"] in ("program", "global"):
                return dict(row=x.get("_row"), kind=x["type"], id=int(x["id"]) if x["type"] == "program" else None,
                            name=x.get("name"), pole_mm=pole, hand=hand, rows=[y.get("_row") for _, y in taken], _taken=taken)
            else:
                return None                             # a Wait, a Wait-for: its rows run as they are
        other = (st.get(flow.OTHER[side]) or {}).get("steps") or []
        if st.get("meet_kind") != "meet" or any(y["type"] not in ("pole", "hand") for y in other):
            return None                                 # past this Meet the other arm would meet it elsewhere than it does
    return None


def with_next_starts(plan: list[dict], own: dict | None) -> list[dict]:
    """Each ``then`` (next_after_tasks) with what its arm moves to: a next Program's own pole and hand (``own``: what Lite
    recorded when it sent it), else its rows', and its first point (``arm_deg``) when Lite knows the Program's bytes; a
    Global Point's start: its rows' pole and hand (its row moves the arm)."""
    for st in plan:
        for side, t in (st.get("then") or {}).items():
            mine = ((own or {}).get(side) or {}).get(t["id"]) or {} if t["kind"] == "program" else {}
            if mine.get("stage"):
                t.update(pole_mm=float(mine["stage"]["pole_mm"]), hand=list(mine["stage"]["hand"]), own=True)
            if mine.get("first_deg") is not None:
                t["arm_deg"] = [round(float(v), 3) for v in mine["first_deg"]]
    return plan


def then_title(st: dict) -> str:
    """A lid or seat task's title, with where its ``then`` takes each arm."""
    then = st.get("then") or {}
    if not then:
        return st["title"]
    where = [f"{s.upper()} to " + (f"Program {t['id']}'s start" if t["kind"] == "program" else f"{t.get('name')}'s pole and hand")
             for s, t in sorted(then.items())]
    return f"{st['title']}, then {' and '.join(where)}"


def own_stages(plan: list[dict], own: dict | None) -> list[dict]:
    """``plan`` with each Program's own pole and hand set before it (the owner, 24 Oct: "every task runs with its own pole
    and hand state, applied before the arm moves ... for uploaded tasks, the app applies the stored pole and hand"):
    before a Program whose pole and hand Lite knows (``own``: what it recorded when it sent it, or its file's found by
    name), the Pole and Hand rows right before it give way to its own, set where they differ from what the plan leaves
    the arm at: a Pole and a Hand (pole_with_hand makes them one step: the hand moving while the pole travels, the
    Program straight after), or the one that differs. Followed through the whole plan: each arm's rows, and a lid or
    seat task's end (its ``then``; without one, not known). Where not known (the run's start), both are set. A Program
    Lite knows no pole and hand of runs with its rows' (the operator's), as before."""
    if not own:
        return plan
    plan = copy.deepcopy(plan)
    now = {s: [None, None] for s in ARMS}              # each arm's pole and hand as the plan leaves it (None: not known)
    for st in plan:
        if st["kind"] == "mechanism":
            for s in ARMS:
                t = (st.get("then") or {}).get(s) or {}
                now[s] = [t.get("pole_mm"), t.get("hand")]
            continue
        for side, steps in _items(st):
            mine, rows = [], []

            def flush():
                for y in rows:
                    if y["type"] == "pole":
                        now[side][0] = float(y["mm"])
                    else:
                        now[side][1] = [int(v) for v in y["values"]]
                mine.extend(rows)
                rows.clear()

            for x in steps:
                if x["type"] in ("pole", "hand"):
                    rows.append(x)
                    continue
                stg = (((own.get(side) or {}).get(int(x["id"])) or {}).get("stage")) if x["type"] == "program" else None
                if stg:
                    mine.extend(_own_steps(x, stg, now[side], rows))
                    rows.clear()
                else:
                    flush()
                mine.append(x)
            flush()
            steps[:] = mine
    return plan


def _own_steps(x: dict, stg: dict, now: list, rows: list) -> list[dict]:
    """The steps that put an arm at Program ``x``'s own pole and hand ``stg`` from ``now`` ([pole mm, hand], updated), in
    place of the Pole and Hand rows right before it (``rows``; the Program keeps what they were, ``_own_stage``): a Pole
    and a Hand, or the one that differs; with a hand channel held (None), Lite's own step (runner v14 sets all six)."""
    from . import staging

    pid, row = int(x["id"]), x.get("_row")
    pole, hand = float(stg["pole_mm"]), list(stg["hand"])
    pole_off = now[0] is None or abs(float(now[0]) - pole) > staging.POLE_TOL_MM
    hand_off = now[1] is None or not staging.same_hand(list(now[1]), hand)
    differ = [y.get("_row") for y in rows if (y["type"] == "pole" and abs(float(y["mm"]) - pole) > staging.POLE_TOL_MM)
              or (y["type"] == "hand" and not staging.same_hand([int(v) for v in y["values"]], hand))]
    x["_own_stage"] = dict(pole_mm=pole, hand=hand, rows=[y.get("_row") for y in rows], differ=differ)
    mark = {"_row": row, "_own": pid}
    out = []
    if any(v is None for v in hand) and (pole_off or hand_off):
        out.append({"type": "pole_hand", "mm": pole, "values": hand, "_rows": [], **mark})
    else:
        if pole_off:
            out.append({"type": "pole", "mm": pole, **mark})
        if hand_off:
            out.append({"type": "hand", "values": [int(v) for v in hand], **mark})
    now[0], now[1] = pole, [h if v is None else int(v) for v, h in zip(hand, now[1] or [None] * 6)]
    return out


def own_for(app, units, programs: dict, steps: dict) -> dict:
    """own_of for a project's run, as Simulation and Real Machine Mode plan it (the same plan, the same identity)."""
    return own_of(programs, {s: files_by_name(app, units, s) for s in ARMS}, _md_reader(units.root), steps)


def own_of(programs: dict, named: dict, md_of, steps: dict) -> dict:
    """{side: {program id: {stage, first_deg}}}: each Program of the run's rows with its own pole and hand as Lite knows
    them (what it recorded when it sent the program; for one it did not send, its file's, found by name) and its first
    point (deg) when Lite knows its very bytes (its record's first line, or a file of those bytes)."""
    from . import staging

    out = {}
    for side in ARMS:
        mine = {}
        for x in steps.get(side) or []:
            if x["type"] != "program" or int(x["id"]) in mine:
                continue
            pid = int(x["id"])
            info = (programs.get(side) or {}).get(pid) or {}
            found = None if info.get("file") or info.get("gone") else found_by_name((named or {}).get(side) or {}, pid, x, info)
            stg = staging.stage_from(info.get("stage")) or (staging.stage_from(found.get("stage")) if found else None)
            first = None
            if info.get("sent") and not info.get("gone") and info.get("first_row"):
                first = [float(v) / 1000.0 for v in info["first_row"]]
            else:
                path = info.get("file") if info.get("sent") and not info.get("gone") else (
                    found["path"] if found and found.get("differs") is False else None)
                md = md_of(path) if path else None
                if md is not None and len(md):
                    first = [float(v) / 1000.0 for v in md[0]]
            mine[pid] = dict(stage=stg.as_dict() if stg else None, first_deg=first)
        out[side] = mine
    return out


def pole_with_hand(plan: list[dict]) -> list[dict]:
    """``plan`` with each Pole row followed straight by a Hand row on the same arm as one step (``pole_hand``; the
    owner, 22 Oct: "there is a delay when the sequence moves from the pole movement to the hand movement"): the hand
    moves while the pole travels (``RealArm.pole_and_hand``), and what comes after waits for both; the arm does not
    move during it. Runner v14 ended its Pole step only when the pole's travel model had run out (1.5 s of creep plus
    the distance at the pole's measured speed), then began the Hand: 3.1 to 4.6 s from the pole's start to the hand's in
    the lab's logs. The two rows keep their numbers (``_rows``)."""
    out = []
    for st in plan:
        if st["kind"] == "mechanism":
            out.append(st)
            continue
        st = copy.deepcopy(st)
        for side, steps in _items(st):
            mine, i = [], 0
            while i < len(steps):
                x = steps[i]
                if x["type"] == "pole" and i + 1 < len(steps) and steps[i + 1]["type"] == "hand":
                    h = steps[i + 1]
                    own = x.get("_own") or h.get("_own")      # a Program's own: it finishes no row (its Program does)
                    mine.append({"type": "pole_hand", "mm": float(x["mm"]), "values": [int(v) for v in h["values"]],
                                 "_row": h.get("_row"), "_rows": [] if own else [x.get("_row"), h.get("_row")],
                                 **({"_own": own} if own else {})})
                    i += 2
                    continue
                mine.append(x)
                i += 1
            steps[:] = mine
        out.append(st)
    return out


def _rows_done(step: dict) -> list:
    """The rows a step finishes (a Wait-for on one is released by it): a Pole with the Hand after it, both its rows; a
    Program's own pole and hand (own_stages), none: its Program finishes its row."""
    rows = step.get("_rows")
    return list(rows) if rows is not None else [step.get("_row")]


def _rows_words(step: dict) -> str:
    rows = step.get("_rows")
    return f"rows {rows[0]}-{rows[1]}" if rows else f"row {step.get('_row')}"


def plan_shape(plan: list[dict]) -> list[dict]:
    """The plan as a simulation's identity keeps it: each stage's kind and each arm's steps in it by row (a clear with its
    row, a Wait-for with the row it waits for, a Program's own pole and hand with what they set, a lid or seat task's
    ``then``). The same steps planned otherwise (Lite's planner changed, as on 21, 22 and 24 Oct) are not the run
    simulated."""
    out = []
    for st in plan:
        row = {"kind": st["kind"]}
        if st["kind"] == "mechanism":
            row.update(task=st["task"], clear=st.get("clear") or [], then=st.get("then"))
        else:
            for side, steps in _items(st):
                row[side] = [["clear", x.get("_row")] if x["type"] == "clear"
                             else ["own", x["type"], x.get("_row"), x.get("mm"), x.get("values")] if x.get("_own")
                             else ["pole_hand", *x["_rows"]] if x["type"] == "pole_hand"
                             else ["wait_for", x.get("_row"), (x.get("_target") or {}).get("row")] if x["type"] == "wait_for"
                             else x.get("_row") for x in steps]
        out.append(row)
    return out


# ── the steps: checked as runner v14 checks them, kept in the project, saved and loaded as JSON

def _runner():
    from .robot_real import _runner as runner

    return runner()


def _v53_rules(steps) -> bool:
    """Whether a list still carries V53's Dual Rule other than TOGETHER (a V53 file, a sequence kept before 22 Oct)."""
    return any(isinstance(x, dict) and str(x.get("dual_rule") or "TOGETHER").strip().upper() != "TOGETHER" for x in steps)


def check_steps(steps, side: str) -> list[dict]:
    """An arm's rows, each as the runner will take it (its ``normalize_workflow_step``), with Lite's own rows (Meet,
    Wait-for, a lid or seat task), each row's ``uid`` and a Wait's note kept: refused with the row that is wrong. A Dual
    Rule of V53's (a file, or a page of before 22 Oct) becomes Meets and a Wait-for (flow.from_v53); TOGETHER is dropped."""
    if not isinstance(steps, list):
        raise UserError(f"The {side.upper()} sequence must be a list of steps.")
    if _v53_rules(steps):
        steps, _ = flow.from_v53(steps)
    if len(steps) > MAX_STEPS:
        raise UserError(f"The {side.upper()} sequence has {len(steps)} steps: at most {MAX_STEPS}.")
    rt = _runner()
    out = []
    for i, raw in enumerate(steps, 1):
        kind = raw.get("type") if isinstance(raw, dict) else None
        if kind == "mechanism":                     # a lid or seat task of ALIX's (mechanism.py)
            from . import mechanism

            task = str(raw.get("task") or "")
            if task not in mechanism.TASKS:
                raise UserError(f"{side.upper()} row {i}: {task or 'no task'} is not a lid or seat task.",
                                f"One of: {', '.join(t['title'] for t in mechanism.TASKS.values())}.", code="step")
            step = {"type": "mechanism", "task": task}
        elif kind == "meet":                        # both arms meet here (flow.py)
            step = {"type": "meet"}
        elif kind == "wait_for":                    # this arm waits for a row of the other's (flow.py)
            tg = raw.get("target") if isinstance(raw.get("target"), dict) else {}
            if tg.get("uid"):
                step = {"type": "wait_for", "target": {"uid": str(tg["uid"])[:32]}}
            elif tg.get("program") is not None:
                try:
                    pid = int(tg["program"])
                except (TypeError, ValueError):
                    pid = 0
                if not 1 <= pid <= 100:
                    raise UserError(f"{side.upper()} row {i}: a Wait-for names a Program 1-100.", code="step")
                step = {"type": "wait_for", "target": {"program": pid}}
            else:
                raise UserError(f"{side.upper()} row {i}: a Wait-for names the other arm's row it waits for.",
                                "Choose that row, or remove the Wait-for.", code="step")
        else:
            try:
                step = dict(rt.normalize_workflow_step(raw))
            except (TypeError, ValueError, KeyError) as exc:
                raise UserError(f"{side.upper()} row {i}: {exc}.", "Correct or remove that row.", code="step") from None
        if raw.get("note"):
            step["note"] = str(raw["note"])[:200]
        if raw.get("uid"):
            step["uid"] = str(raw["uid"])[:32]
        out.append(step)
    return flow.with_uids(out)


def _file(units) -> Path:
    return Path(units.root) / "sequence" / "s2.json"


def load(units) -> dict:
    """Both arms' rows as kept. Rows kept before 22 Oct (no uid; V53's Dual Rule) are brought to today's form once and
    kept so: a Wait-for names a row by its uid, which must not change between two reads."""
    from .project import _atomic_write

    with units.lock:
        try:
            d = json.loads(_file(units).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            d = {}
        out = {side: list(d.get(side) or []) for side in ARMS} | {"updated": d.get("updated")}
        stale = [s for s in ARMS if out[s] and (any("uid" not in x or "dual_rule" in x for x in out[s] if isinstance(x, dict)))]
        if stale:
            for s in stale:
                try:
                    out[s] = check_steps(out[s], s)
                except UserError:
                    continue                         # kept as it is; the page says what is wrong when it is saved
            _file(units).parent.mkdir(parents=True, exist_ok=True)
            _atomic_write(_file(units), json.dumps({**d, **{s: out[s] for s in ARMS}}, indent=1))
    return out


def save_side(units, side: str, steps) -> dict:
    from .project import _atomic_write

    steps = check_steps(steps, side)
    with units.lock:
        d = load(units)
        d[side] = steps
        d["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
        _file(units).parent.mkdir(parents=True, exist_ok=True)
        _atomic_write(_file(units), json.dumps(d, indent=1))
    return d


def document(side: str, steps: list) -> dict:
    """SAVE: an arm's rows as Lite keeps them (V53's rows, Lite's Meet and Wait-for, each row's uid)."""
    return {"format": LITE_FORMAT, "side": side, "steps": steps}


def from_document(side: str, data) -> list[dict]:
    """LOAD: Lite's JSON, or V53's (its v51 compatibility too: a Program row without a Start % gets 30 %; its Dual Rule
    as Meets and a Wait-for). A file saved for the other arm is refused: Program 71 is another trajectory on each
    controller."""
    if not isinstance(data, dict) or data.get("format") not in (FORMAT, LITE_FORMAT):
        raise UserError("Could not load the sequence: wrong sequence format.",
                        f"A sequence JSON has \"format\": \"{LITE_FORMAT}\" (Lite's) or \"{FORMAT}\" (V53's).")
    steps = data.get("steps")
    if not isinstance(steps, list):
        raise UserError("Could not load the sequence: steps must be a list.")
    other = str(data.get("side") or "").lower()
    if other in ARMS and other != side:
        raise UserError(f"This is a {other.upper()} sequence: it never loads into the {side.upper()} arm.",
                        f"Load it into the {other.upper()} side, or load the {side.upper()} file.", code="arm")
    steps = copy.deepcopy(steps)
    for step in steps:
        if isinstance(step, dict) and step.get("type") == "program" and "transit_speed" not in step and "start_speed" not in step:
            step["transit_speed"] = 30
    return check_steps(steps, side)


def _staged(steps: list, upto: int) -> list:
    """The Pole and Hand steps in force before row ``upto`` (1-based): the last of each above it, in their
    order, so a Program run on its own starts at the pole and hand the sequence puts it at (V53 stages
    nothing itself: its own Pole and Hand steps do)."""
    pole = hand = None
    for st in steps[:max(0, upto - 1)]:
        if st["type"] == "pole":
            pole = st
        elif st["type"] == "hand":
            hand = st
    return sorted([x for x in (pole, hand) if x is not None], key=lambda x: x.get("_row") or 0)


def _with_stage(steps: list, part: list) -> list:
    """``part`` (an arm's rows of one stage, numbered) with the Pole and Hand in force before it, when it
    does not set them itself before its first Program."""
    first = min(x.get("_row") or 1 for x in part)
    head = []
    for x in part:
        if x["type"] == "program":
            break
        head.append(x["type"])
    return [x for x in _staged(steps, first) if x["type"] not in head] + list(part)


def _items(st: dict) -> list:
    """(side, steps) of each arm's part of a stage, for the arms that have one."""
    return [(side, st[side]["steps"]) for side in ARMS if isinstance(st.get(side), dict)]


def _plan_error(exc) -> UserError:
    return UserError(f"Cannot run the sequence: {exc}", "Correct the Meet or Wait-for rows it names.", code="plan")


def _finish(plan: list[dict], own: dict | None) -> list[dict]:
    """A plan's own steps of Lite's: each Program's own pole and hand before it (own_stages, with each next task's start
    after the lid and seat tasks, with_next_starts), the 4105 clears (clear_before_moves), a Pole with the Hand after it
    as one step (pole_with_hand)."""
    return pole_with_hand(clear_before_moves(own_stages(with_next_starts(plan, own), own)))


def plan_for(steps: dict, mode: str, pick: dict | None = None, own: dict | None = None) -> dict:
    """What a run does (the owner, 16 Oct: "options to play a single task arm, dual arm task and sequences in both
    simulation and real"), with each Program's own pole and hand (``own``, own_of; the owner, 24 Oct):
    - a sequence: ``mode`` both (RUN BOTH: both arms' rows, ``flow.build``) or left / right (RUN LEFT / RIGHT: that
      arm's rows in one runner call; its Meet and Wait-for rows passed over, the other arm not running);
    - a single task: ``pick`` {scope: task, side, row}, that Program row alone on its arm, with the Pole and Hand in
      force before it;
    - a dual-arm task: ``pick`` {scope: dual, stage}, one stage of both arms' plan (their rows between two meeting
      points, each arm with the Pole and Hand in force before its part; a Wait-for whose row came earlier is already
      done), or one lid or seat task.
    Steps are numbered (``_row``)."""
    scope = str((pick or {}).get("scope") or "sequence")
    if scope == "task":
        side, row = str(pick.get("side") or ""), pick.get("row")
        if side not in ARMS:
            raise UserError("Say which arm the task is on.")
        try:
            row = int(row)
        except (TypeError, ValueError):
            raise UserError("Choose the task: a Program row of the arm.") from None
        mine = steps[side]
        if not 1 <= row <= len(mine) or mine[row - 1]["type"] != "program":
            raise UserError(f"{side.upper()} row {row} is not a Program row.", "Choose a Program row of the arm.")
        if pick.get("id") is not None and str(mine[row - 1]["id"]) != str(pick.get("id")):     # the page's word: what it showed
            raise UserError(f"{side.upper()} row {row} holds Program {mine[row - 1]['id']} now, not Program {pick.get('id')}: "
                            "the steps changed since the task was chosen.", "Choose the task again.", code="pick")
        picked = {side: _staged(mine, row) + [mine[row - 1]]}
        plan = _finish([{"kind": "single", "program_id": int(mine[row - 1]["id"]), side: {"side": side, "steps": picked[side]}}], own)
        return dict(mode=side, pick=dict(scope="task", side=side, row=row), first_row={side: row}, steps=picked, plan=plan,
                    counts=flow.counts(plan), said=[])
    if scope == "dual":
        if not steps["left"] or not steps["right"]:
            raise UserError("Build or load BOTH LEFT and RIGHT sequences first.")
        try:
            full = flow.build(steps["left"], steps["right"])
            k = int(pick.get("stage"))
        except flow.PlanError as exc:
            raise _plan_error(exc) from None
        except (TypeError, ValueError):
            raise UserError("Choose the dual-arm task: a stage of the plan.") from None
        if not 0 <= k < len(full) or full[k]["kind"] not in ("parallel", "mechanism"):
            raise UserError("That stage is not a dual-arm task.", "Choose a part with both arms' Programs, or a lid or seat task.")
        st = full[k]
        if st["kind"] == "mechanism":                       # one lid or seat task on its own: both arms, its own plan
            if pick.get("task") and pick.get("task") != st["task"]:
                raise UserError(f"Stage {k + 1} is {st['title']} now: the steps changed since it was chosen.", "Choose it again.",
                                code="pick")
            return dict(mode="both", pick=dict(scope="dual", stage=k, label=st["program_id"]), first_row=dict(st["rows"]),
                        steps={"left": [], "right": []}, plan=[st], counts=flow.counts([st]), said=[])
        parts = {side: st[side]["steps"] for side in ARMS if st.get(side)}
        if set(parts) != set(ARMS) or not all(any(x["type"] == "program" for x in parts[side]) for side in ARMS):
            raise UserError("That stage is not a dual-arm task: it has no Program on both arms.", "Choose a part with both arms' Programs.")
        want = pick.get("ids")                              # the page's word: the Programs it showed for the stage
        have = {side: [str(x["id"]) for x in parts[side] if x["type"] == "program"] for side in ARMS}
        if isinstance(want, dict) and any([str(i) for i in (want.get(side) or [])] != have[side] for side in ARMS):
            raise UserError(f"Stage {k + 1} holds LEFT Program {', '.join(have['left'])} and RIGHT Program {', '.join(have['right'])} now: "
                            "the steps changed since the dual-arm task was chosen.", "Choose the dual-arm task again.", code="pick")
        picked = {side: _with_stage(steps[side], parts[side]) for side in ARMS}
        stage = {"kind": "parallel", "program_id": st["program_id"],
                 **{side: {"side": side, "steps": [dict(x) for x in picked[side]]} for side in ARMS}}
        for side in ARMS:                                   # the rows of this part are run; a Wait-for's earlier row is done
            for x in stage[side]["steps"]:
                if x["type"] == "wait_for" and not (x.get("_target") or {}).get("done"):
                    rows = {y.get("_row") for y in picked[flow.OTHER[side]]}
                    if (x.get("_target") or {}).get("row") not in rows:
                        x["_target"] = dict(x.get("_target") or {}, done=True)
            stage[f"{side}_program_ids"] = [int(x["id"]) for x in picked[side] if x["type"] == "program"]
        plan = _finish([stage], own)
        first = {side: min(x.get("_row") or 1 for x in parts[side]) for side in ARMS}
        return dict(mode="both", pick=dict(scope="dual", stage=k, label=st.get("program_id") or st.get("title")), first_row=first,
                    steps=picked, plan=plan, counts=flow.counts(plan), said=[])
    if mode not in ("both", *ARMS):
        raise UserError("Run both arms, or the left or the right arm.")
    picked = {side: list(steps[side]) for side in ARMS if mode in ("both", side)}
    said = []
    if mode != "both" and any(st["type"] == "mechanism" for st in picked[mode]):
        raise UserError(f"The {mode.upper()} sequence has a lid or seat task, which uses both arms.",
                        "Run both arms (RUN BOTH), or remove the task from the list.", code="plan")
    if mode == "both":
        if not picked["left"] or not picked["right"]:
            raise UserError("Build or load BOTH LEFT and RIGHT sequences first.")
        try:
            plan = flow.build(picked["left"], picked["right"])
        except flow.PlanError as exc:
            raise _plan_error(exc) from None
    else:
        if not picked[mode]:
            raise UserError(f"{mode.upper()} sequence is empty.")
        mine, said = flow.single(mode, picked[mode])
        if not mine:
            raise UserError(f"The {mode.upper()} sequence has only Meet and Wait-for rows: nothing to run on its own.")
        plan = [{"kind": "single", "program_id": None, mode: {"side": mode, "steps": mine}}]
    if mode == "both":
        plan = next_after_tasks(plan)
    plan = _finish(plan, own)
    return dict(mode=mode, pick=dict(scope="sequence"), first_row={side: 1 for side in picked}, steps=picked, plan=plan,
                counts=flow.counts(plan), said=said)


def digest(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


# ── what Lite knows of each arm's controller: the programs it sent and the global points it read

_POINTS: dict = {}                     # side -> {"read": when, "points": {name: joints deg}}: the last read


def _project_files(units, side: str, every: bool = False) -> dict:
    """The project's files of an arm by sha256: every region's recordings and kept versions, its
    transitions, the merges and the merges' tuned versions; each with its name, stage, phase (the
    commode's lid/seat state its region needs) and the UGV placement it was taught at. ``every``: each
    also lists the project's other files of the same bytes ("also"), so that the one a program was sent
    as is named (content.file_for)."""
    from . import merges
    from .lite_units import display_name

    out = {}

    def add(sha, path, name, stage, phase, place, region):
        if not sha:
            return
        f = dict(path=str(path), name=name, stage=stage, phase=phase, placement=place, region=region)
        if sha not in out:
            out[sha] = f
        elif every:
            out[sha].setdefault("also", []).append(f)

    for m in merges.listing(units, side):                 # first: a program sent from S1 is named as the merge is now
        add(m.get("sha256"), merges.folder(units, side) / m["file"], m["name"], m.get("stage"), m.get("phase"), m.get("placement"),
            "Merges")
    keys = [k for k, u in units.units.items() if u["arm"] == side]    # the regions (a merge's copy in Tune is its merge)
    for key in keys:
        u = units.units[key]
        recs = units.recordings(key)
        by_file = {r["file"]: r for r in recs}
        for r in recs:
            add(r.get("sha256"), r["path"], r["name"], r.get("stage"), units.phase_of(key), r.get("placement"), display_name(u))
        for v in units.versions(key):
            try:
                extra = json.loads(Path(v["path"]).with_name(Path(v["path"]).stem + ".extra.json").read_text(encoding="utf-8"))
            except (OSError, ValueError):
                extra = {}
            add(v.get("sha256"), v["path"], v.get("name") or f"{display_name(u)} {v['version']}", v.get("stage"), units.phase_of(key),
                (by_file.get(extra.get("raw_file")) or {}).get("placement"), display_name(u))
        tv = units.transition(key)
        if tv:
            add(tv.get("sha256"), tv["path"], tv["name"], tv.get("stage"), units.phase_of(key), None, display_name(u))
    return out


def programs_known(app, units, side: str) -> dict:
    """{program id: what Lite knows}: the programs it sent to this arm's controller (its record: name,
    lines, sha256, stage) with the project's file of those bytes, when it is still there, else the very file
    Lite sent (a file sent from Advanced is kept in Lite's data folder, not in the project), when it still holds
    those bytes. On a linked controller, the slots it holds now too: a program Lite did not send, or one that
    replaced Lite's, is known by its name only (``sent`` False)."""
    from ..motion import track
    from .content import file_for

    files = _project_files(units, side, every=True)
    out = {}
    for pid, r in app.robot.arm(side).programs.items():
        f = file_for(files, r.get("sha256"), r.get("name"))
        if not f.get("path") and r.get("path") and r.get("sha256"):      # the file it sent, where Lite kept it
            try:
                if Path(r["path"]).is_file() and track.sha256_file(r["path"]) == r["sha256"]:
                    f = dict(path=str(r["path"]), name=Path(r["path"]).name)
            except OSError:
                pass
        out[int(pid)] = dict(id=int(pid), name=r.get("name"), lines=r.get("lines"), seconds=track.duration_s(r.get("lines")),
                             sha256=r.get("sha256"), stage=r.get("stage"), first_row=r.get("first_row"),
                             sent=True, file=f.get("path"), file_name=f.get("name"), phase=f.get("phase"),
                             placement=f.get("placement"), region=f.get("region"))
    held = None
    if app.robot._real(side) is not None and app.robot.arm(side).connected and not app.jobs.running(arm=side):
        try:
            held = {int(x["id"]): x for x in app.robot.programs(side) if x.get("name")}
        except UserError:
            held = None
        if held is not None:                              # the lists Lite offers follow this read too (content.py)
            from . import content

            content.store(side, programs=[dict(id=pid, name=x["name"], bytes=x.get("bytes"), sent=bool(x.get("sha256")),
                                               sha256=x.get("sha256"), lines=x.get("lines")) for pid, x in sorted(held.items())])
    if held is not None:
        for pid, x in held.items():
            if not x.get("sha256"):                       # the controller's own, not what Lite sent
                out[pid] = dict(id=pid, name=x.get("name"), size=x.get("bytes"), lines=None, seconds=None, sha256=None, stage=None,
                                sent=False, file=None)
        for pid in [p for p in out if p not in held]:
            out[pid] = dict(out[pid], gone=True)          # Lite sent it, the controller no longer holds it
    return out


_UPLOAD = None


def proj_bytes(path) -> int | None:
    """The bytes a controller stores for a track file: the control program's project file built from it
    (vendor/control_app/rm_upload.py: read_track, write_project; the module's file builders need no SDK), so
    a program on the controller can be compared with a file by its size. None: not a track file."""
    import importlib.util
    import tempfile

    from .. import vendor_paths

    global _UPLOAD
    if _UPLOAD is None:
        spec = importlib.util.spec_from_file_location("lite_rm_upload_files", vendor_paths.RM_UPLOAD)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _UPLOAD = mod
    try:
        lines = _UPLOAD.read_track(Path(path))
    except (SystemExit, OSError):
        return None
    with tempfile.TemporaryDirectory() as tmp:
        return int(_UPLOAD.write_project(lines, Path(tmp) / "track.proj"))


def files_by_name(app, units, side: str) -> dict:
    """The arm's track files by name (lower case), for a Program Lite did not send (the control program
    sent it): the project's own first (its merges, the regions' recordings and kept versions, the
    transitions), then the Trajectory Viewer's library (the operator's files, then the built-in sets).
    {name: [{path, name, stage, phase, placement, region, source}, ...]}, in that order."""
    from . import library

    out: dict = {}

    def add(name, f):
        out.setdefault(str(name).lower(), []).append(f)

    for sha, f in _project_files(units, side).items():
        add(f["name"], dict(f, sha256=sha, source="the project"))
    lib = getattr(app, "library", None)
    if lib is not None:
        for m in lib.uploads():
            if m.get("arm") == side:
                add(m["name"], dict(path=str(lib.root / f"{m['id']}.txt"), name=m["name"], stage=m.get("stage"), phase=None,
                                    placement=None, region=None, sha256=m.get("sha256"), source="your files in the Trajectory Viewer"))
    for st in library.builtin_sets():
        for f in st["files"]:
            if f.get("arm") == side:
                add(f["name"], dict(path=f["path"], name=f["name"], stage=f.get("stage"), phase=None, placement=None, region=None,
                                    sha256=f.get("sha256"), source=f"the Trajectory Viewer's {st['name']}"))
    return out


def points_known(app, side: str) -> dict | None:
    """The global points of a linked arm's controller, read now when the arm is idle, else as last read."""
    if app.robot._real(side) is not None and app.robot.arm(side).connected and not app.jobs.running(arm=side):
        try:
            pts = app.robot.global_points(side) or []
            _POINTS[side] = dict(read=time.strftime("%Y-%m-%d %H:%M:%S"), points={p["name"]: p["joints"] for p in pts})
            from . import content

            content.store(side, points=pts)
        except UserError:
            pass
    return _POINTS.get(side)


def _fingerprint(info: dict | None) -> str | None:
    """What a program slot holds, as far as Lite can tell: the bytes Lite sent (sha256), else the
    controller's own name and size, else nothing known."""
    if not info:
        return None
    if info.get("gone"):
        return "gone"
    if info.get("sent") and info.get("sha256"):
        return info["sha256"]
    return f"controller:{info.get('name')}|{info.get('size')}" if info.get("name") else None


def identity_of(run: dict, programs: dict, points: dict, overall: int) -> dict:
    """What makes a run the same run (Real only after the same one completed in Simulation): its
    steps, mode and rows, how they are planned into stages (``plan_shape``), the programs it uses as Lite
    knows them, the global points it moves to as read, and the overall speed."""
    used = {}
    for side, steps in run["steps"].items():
        pids = sorted({int(s["id"]) for s in steps if s["type"] == "program"})
        names = sorted({s["name"] for s in steps if s["type"] == "global"})
        pts = (points.get(side) or {}).get("points") or {}
        used[side] = dict(programs={str(p): _fingerprint(programs[side].get(p)) for p in pids},
                          points={n: [round(float(v), 2) for v in pts[n]] if n in pts else None for n in names})
    tasks = sorted({st["task"] for st in run["plan"] if st.get("kind") == "mechanism"})
    if tasks:                                       # ALIX's plan files as they are: another plan is another run
        from . import mechanism

        used["mechanisms"] = {t: mechanism.digest(t) for t in sorted(set(tasks))}
    return dict(mode=run["mode"], pick=run["pick"], steps={s: [{k: v for k, v in st.items() if k != "_row"} for st in steps]
                                                               for s, steps in run["steps"].items()},
                plan=plan_shape(run["plan"]), used=used, overall=int(overall))


# ── Simulation Mode: each arm's actions on one clock, through the plan

CYCLE_S = 0.005
SAMPLE_S = 0.02                  # the collision check's clock: every 4th 5 ms line
MAX_SAMPLES = 90000
FRAME_S = 0.04                   # the 3D's frames
MAX_FRAMES = 6000


def _ease(f):
    import numpy as np

    return 0.5 - 0.5 * np.cos(np.pi * np.clip(f, 0.0, 1.0))


def _trapezoid(x: float, accel_s: float) -> float:
    """A move of ``x`` s at its cruise speed with ``accel_s`` of acceleration and as much to stop: x + accel_s, or a
    triangle when it is too short to cruise."""
    import math

    return x + accel_s if x >= accel_s else 2.0 * math.sqrt(max(0.0, x) * accel_s)


def _cruise_s(q0, q1, share: float, rating) -> float:
    """The slowest joint's travel from ``q0`` to ``q1`` (deg) at ``share`` of its rating."""
    import numpy as np

    d = np.abs(np.asarray(q1, dtype=float) - np.asarray(q0, dtype=float))
    return float(np.max(d / (max(1e-3, float(share)) * np.asarray(rating, dtype=float))))


def movej_s(q0, q1, pct: float, rating) -> float:
    """The controller's MoveJ from ``q0`` to ``q1`` at ``pct`` % (the move's speed x the global speed), as the lab timed
    it (MOVEJ_ACCEL_S, MOVEJ_CALL_S)."""
    share = max(1.0, min(100.0, float(pct))) / 100.0
    return _trapezoid(_cruise_s(q0, q1, share, rating), MOVEJ_ACCEL_S) + MOVEJ_CALL_S


def transit_s(q0, q1, start_pct: float, rating) -> float:
    """A Program's own move to its first point at its Start % (TRANSIT_RATING_SHARE, TRANSIT_ACCEL_S): the global speed
    does not change it."""
    share = max(1.0, min(100.0, float(start_pct))) / 100.0 * TRANSIT_RATING_SHARE
    return _trapezoid(_cruise_s(q0, q1, share, rating), TRANSIT_ACCEL_S)


def _at(r, lam) -> float:
    """A step's time from its range ``r`` (least, as a rule, most): as a rule (``lam`` None), or ``lam`` of the way from
    its least (0) to its most (1)."""
    return float(r[1]) if lam is None else float(r[0]) + float(lam) * (float(r[2]) - float(r[0]))


def _plus(*rs):
    return tuple(sum(float(r[i]) for r in rs) for i in range(3))


def found_by_name(named: dict, pid: int, step: dict, info: dict) -> dict | None:
    """A Program Lite did not send (the control program, or V53, sent it), found by its file's name (the
    owner, 16 Oct: the sequence must play in the simulation): the name the sequence gives it (V53's JSON),
    else the controller's ("<slot>_<file>", cut to 31 characters by RealMan's API). With the controller's
    stored size known, the file whose project file has that size; else the first found, said so."""
    import re

    sha = info.get("sent") and info.get("sha256")
    if sha:                                      # Lite sent it: the file of those very bytes, wherever it is now
        for fs in named.values():
            for f in fs:
                if f.get("sha256") == sha:
                    return dict(f, how="the very bytes Lite sent to the controller", differs=False)
    names = [str(step["name"])] if step.get("name") else []
    held = str(info.get("name") or "")
    if held:
        names.append(re.sub(rf"^{pid}_", "", held))
    cands = []
    for n in names:
        low = n.lower()
        hit = named.get(low) or named.get(low + ".txt") or []
        if not hit and held and len(held) >= 31 and n is names[-1]:     # cut to 31 characters: its start
            hit = [f for k, fs in named.items() if k.startswith(low) for f in fs]
        cands += [f for f in hit if f not in cands]
    if not cands:
        return None
    if sha:
        return dict(cands[0], how="Lite sent other bytes to this slot: they may differ", differs=True)
    size = info.get("size")
    if size:
        for f in cands:
            b = proj_bytes(f["path"])
            if b is not None and abs(int(size) - b) <= max(64, b // 1000):
                return dict(f, how="its bytes make the size the controller holds")
        b = proj_bytes(cands[0]["path"])
        return dict(cands[0], how=f"the controller's program is {int(size):,} bytes, this file makes {b or 0:,}: they may differ",
                    differs=True)
    return dict(cands[0], how="not compared with the controller's program (connect the arm to compare)", differs=None)


class _ArmSim:
    """One arm through its steps: what each step does and when (its actions), from a start state, each step with its
    range of times on the robot (ROBOT_TIMES); ``lam`` None times the run as a rule, 0 to 1 at that point of every
    step's range (0: as fast as the robot runs it, 1: as slow)."""

    def __init__(self, side, start, rating, overall, programs, points, md_of, findings, named=None, lam=None):
        self.side, self.rating, self.overall, self.lam = side, rating, overall, lam
        self.pole, self.hand, self.q = float(start["pole_mm"]), [int(c) for c in start["hand"]], [float(v) for v in start["q_deg"]]
        self.programs, self.points, self.md_of, self.findings = programs, points, md_of, findings
        self.named = named or {}                     # the arm's files by name (files_by_name)
        self.acts: list[dict] = []
        self.set_pole = self.set_hand = False        # a Pole / Hand step ran in this run
        self.known = True                            # where the arm is, as far as the simulation can tell

    def _act(self, stage, step, kind, t0, dur, text, **more):
        a = dict(side=self.side, stage=stage, row=step.get("_row"), kind=kind, t0=round(t0, 4), t1=round(t0 + dur, 4), text=text,
                 pole0=self.pole, hand0=list(self.hand), q0=list(self.q), known=self.known, **more)
        self.acts.append(a)
        return a

    def _say(self, level, step, text):
        self.findings.append(dict(level=level, side=self.side, row=step.get("_row"), text=text))

    def _unknown(self, step, why):
        """After a step the simulation cannot draw, where the arm is is not known: it is shown where it
        was, and the collision check leaves it out until a drawn Program puts it on a known line again, or a
        Global Point it knows on a known point."""
        self._say("warn", step, f"{self.side.upper()} row {step.get('_row')}: {why}. From here the arm's pose is not known: "
                  "it is shown where it was, and the collision check leaves this arm out until a drawn Program or a Global Point "
                  "it knows.")
        self.known = False

    def hold(self, stage, step, t0, t1, text, **more) -> dict:
        """The arm still where it is from ``t0`` to ``t1`` (a Meet, a Wait-for): an action, even of no time, so that every
        timing of the run has the same actions in the same order."""
        a = self._act(stage, step, "wait", t0, max(0.0, t1 - t0), text, gate=True, times=(0.0, 0.0, 0.0), **more)
        a.update(pole1=self.pole, hand1=list(self.hand), q1=list(self.q))
        return a

    def run(self, steps, t, stage, done=None) -> float:
        """The steps one after the other from ``t``: their actions; ``done(step, t1)`` as each ends. The end's time."""
        from . import staging

        S, T, lam = self.side.upper(), ROBOT_TIMES, self.lam
        for step in steps:
            kind = step["type"]
            if kind == "pole":
                mm = float(step["mm"])
                law = staging.pole_travel_s(self.pole, mm)
                r = tuple(law + x for x in T["pole_extra_s"])
                a = self._act(stage, step, "pole", t, _at(r, lam), f"Pole {mm:g} mm", times=r)
                self.pole = mm
                self.set_pole = True
            elif kind == "hand":
                vals = [int(v) for v in step["values"]]
                k = T["hand_s_per_1000"] * max(abs(int(a_) - int(b_)) for a_, b_ in zip(self.hand, vals)) / 1000.0
                r = (T["hand_s"][0], T["hand_s"][1] + k, T["hand_s"][2] + k)
                a = self._act(stage, step, "hand", t, _at(r, lam), "Hand " + ",".join(str(v) for v in vals), times=r)
                self.hand = vals
                self.set_hand = True
            elif kind == "wait":
                r = tuple(float(step["seconds"]) + x for x in T["wait_extra_s"])
                a = self._act(stage, step, "wait", t, _at(r, lam), _step_words(step), times=r)
            elif kind == "pole_hand":                # the hand while the pole travels (pole_with_hand), then the next call
                mm = float(step["mm"])
                vals = [int(h) if v is None else int(v) for v, h in zip(step["values"], self.hand)]   # None: held
                law = staging.pole_quick_s(self.pole, mm)      # Lite's step: idle within 5 mm after the ramp (26 Oct)
                rp = tuple(law + x for x in T["pole_extra_s"]) if law > 0 else (0.0, 0.0, 0.0)
                span = max(abs(int(a_) - int(b_)) for a_, b_ in zip(self.hand, vals))
                stroke = T["hand_stroke_s"][0] + T["hand_stroke_s"][1] * span / 1000.0      # the control program's stroke law
                rh = (stroke, stroke + 0.05, T["hand_s"][2] + T["hand_s_per_1000"] * span / 1000.0) if span else (0.0, 0.0, 0.0)
                own = _plus(T["pair_open_s"], tuple(max(x, y) for x, y in zip(rp, rh)), T["pair_end_s"])   # Lite times it
                r = own if step.get("_then") else _plus(own, T["call_start_s"])        # then the next runner call
                opened = _at(T["pair_open_s"], lam)
                a = self._act(stage, step, "pole_hand", t, _at(r, lam), _step_words(step), times=r, own_times=own,
                              pole_s=round(opened + _at(rp, lam), 4), hand_s=round(opened + _at(rh, lam), 4))
                self.pole, self.hand = mm, vals
                self.set_pole = self.set_hand = True
            elif kind == "move_to":                  # Lite's MoveJ to a pose (a next task's start): the speed slider's %, the
                q1 = [float(v) for v in step["deg"]]  # global speed at 100 % for it (staging.park_arm)
                import numpy as np

                if float(np.max(np.abs(np.asarray(q1) - np.asarray(self.q)))) <= staging.MOVEJ_SKIP_DEG:
                    r = T["global_there_s"]                       # already there: no MoveJ is sent (26 Oct)
                else:
                    r = tuple(movej_s(self.q, q1, float(step["speed"]), self.rating) * f for f in MOVEJ_BAND)
                mv = _at(r, lam)
                a = self._act(stage, step, "global", t, mv, _step_words(step), move_s=round(mv, 4), drawn=True, times=r,
                              move_times=r)
                self.q, self.known = q1, True
            elif kind == "clear":                    # Lite's read (and clear) between runner calls, then the next call
                r = T["clear_s"] if step.get("_alone") else _plus(T["clear_s"], T["call_start_s"])
                a = self._act(stage, step, "wait", t, _at(r, lam), _step_words(step), times=r)
            elif kind == "global":
                pts = (self.points or {}).get("points") or {}
                name, pct, hold = step["name"], float(step["speed"]) * self.overall / 100.0, float(step["wait_s"])
                if name in pts:
                    import numpy as np

                    q1 = [float(v) for v in pts[name]]
                    if float(np.max(np.abs(np.asarray(q1) - np.asarray(self.q)))) < 0.2:
                        r = T["global_there_s"]                   # already there: the controller answers at once
                    else:
                        r = tuple(movej_s(self.q, q1, pct, self.rating) * f for f in MOVEJ_BAND)
                        if not self.known:                         # from a pose not known: from no move to the move from
                            r = (T["global_there_s"][0], r[1], r[2])   # where the arm was last known
                    mv = _at(r, lam)
                    a = self._act(stage, step, "global", t, mv + hold, f"{step['move'].upper()} {name} ({step['speed']} %)",
                                  move_s=round(mv, 4), drawn=True, times=_plus(r, (hold, hold, hold)), move_times=r)
                    if step["move"] == "movel":
                        self._say("note", step, f"{S} row {step.get('_row')}: MoveL to {name} is drawn as a joint move "
                                  "(the controller moves the tool in a straight line).")
                    self.q = q1
                    self.known = True                        # at the point: known again (its move to it is not)
                else:
                    r = (0.05 + hold, 1.5 + hold, 10.0 + hold)
                    a = self._act(stage, step, "global", t, _at(r, lam), f"{step['move'].upper()} {name}: not drawn", drawn=False,
                                  times=r, move_times=(0.05, 1.5, 10.0))
                    self._unknown(step, f"global point {name} is not drawn: "
                                  + ("the controller's global points were not read (connect the arm)" if not self.points
                                     else "the controller has no point of that name"))
            else:
                a = self._program(step, t, stage)
            a.update(pole1=self.pole, hand1=list(self.hand), q1=list(self.q))
            if self.lam is None:                     # as a rule: the step keeps its range (Real Machine Mode times it)
                step["_t"] = {k: a.get(k) for k in ("kind", "times", "own_times", "move_times", "play_s", "transit", "lines", "drawn")}
            t = a["t1"]
            if done is not None:
                done(step, t)
        return t

    def _found(self, pid: int, step: dict, info: dict) -> dict | None:
        return found_by_name(self.named, pid, step, info)

    def _transit(self, q_first, transit_pct: float):
        """The controller's own move to a Program's first point at its Start % (``transit_s``; the global speed does not
        change it): (least, as a rule, most) and the joint gap, or no move when the arm is within TRANSIT_MIN_DEG of it."""
        import numpy as np

        gap = float(np.max(np.abs(np.asarray(q_first, dtype=float) - np.asarray(self.q, dtype=float))))
        if gap < TRANSIT_MIN_DEG:
            return (0.0, 0.0, 0.0), gap
        t = transit_s(self.q, q_first, float(transit_pct), self.rating)
        return tuple(t * f for f in TRANSIT_BAND), gap

    def _program(self, step, t, stage) -> dict:
        from . import staging

        S, pid, lam, T = self.side.upper(), int(step["id"]), self.lam, ROBOT_TIMES
        info = self.programs.get(pid) or {}
        label = f"Program {pid}" + (f" // {info.get('name') or step.get('name')}" if (info.get("name") or step.get("name")) else "")
        found = None
        if not info.get("file") and not info.get("gone"):        # no file of its own in the project
            found = self._found(pid, step, info)
            if found is not None:                    # its file as found; the stage it was taught with is that file's
                info = dict(info, file=found["path"], file_name=found["name"], stage=found.get("stage"), phase=found.get("phase"),
                            placement=found.get("placement"))
                level = "warn" if found.get("differs") else "note"
                self._say(level, step, f"{S} row {step.get('_row')}, program {pid}: drawn from {found['name']} ({found['source']}), "
                          f"found by its name; {found['how']}.")
        st = staging.stage_from(info.get("stage"))
        mine = step.get("_own_stage") or {}
        if mine.get("differ") and self.lam is None:  # its own in place of the rows before it (own_stages)
            self._say("note", step, f"{S} row {step.get('_row')}, program {pid}: it runs with its own pole and hand "
                      f"({staging.describe(staging.make_stage(mine['pole_mm'], mine['hand']))}), which Lite sets before it "
                      f"in place of row{'s' if len(mine['differ']) > 1 else ''} {', '.join(str(r) for r in mine['differ'])}.")
        if st is not None:                           # its own set before it (own_stages); a Pole or Hand step after that
            said = stage_off(S, step.get("_row"), pid, st, self.pole, self.hand, self.set_pole, self.set_hand)
            if said:                                 # a stop: Real Machine Mode does not run it (the owner, 21 Oct)
                self._say("stop", step, said)
        md = self.md_of(info.get("file")) if info.get("file") and not info.get("gone") else None
        lead = lambda n: tuple(0.05 + n * x for x in T["load_s_per_line"])                      # noqa: E731
        tail = tuple(max(0.0, x - 0.05) for x in T["program_extra_s"])
        if md is None:
            named = step.get("name") or info.get("name")
            nowhere = f"; no file of its name{f' ({named})' if named else ''} in the project or the Trajectory Viewer's library"
            why = ("the controller no longer holds the program Lite sent" if info.get("gone")
                   else "the controller holds a program Lite did not send" + nowhere if info and not info.get("sent")
                   else "Lite has not sent this program" + nowhere if not info
                   else "its file is no longer in the project")
            lines = int(info.get("lines") or 0)
            r = _plus(lead(lines), tuple((max(lines, 1) - 1) * CYCLE_S * x for x in T["play"]), tail) if lines else (0.0, 0.0, 0.0)
            a = self._act(stage, step, "program", t, _at(r, lam), label + ": not drawn", program=pid, drawn=False, why=why, phase=None,
                          times=r, lines=lines or None, play_s=round((max(lines, 1) - 1) * CYCLE_S, 4) if lines else None)
            self._unknown(step, f"program {pid} is not drawn ({why}" + (f"; {_at(r, None):.1f} s allowed for it)" if lines else ")"))
            return a
        n = int(md.shape[0])
        q_first = (md[0] / 1000.0).tolist()
        transit, gap = self._transit(q_first, float(step["transit_speed"]))
        play = tuple((n - 1) * CYCLE_S * x for x in T["play"])
        r_lead = lead(n)
        if transit[2] > 0 and not self.known:      # from a pose not known (a step not drawn): as a rule none, as a sequence
            transit = (0.0, 0.0, transit[2])         # is made (the lab's delays held Program 82's first point, 21 Oct), up to
            if self.lam is None:                     # the move from where the arm was last known
                self._say("note", step, f"{S} row {step.get('_row')}, program {pid}: where the arm is before it is not known (a "
                          "step before it is not drawn), so the controller's move to the program's first point is not known either: "
                          f"none as a rule, up to {transit[2]:.1f} s (from where the arm was last known, {gap:.0f} deg away).")
        elif transit[2] > 0 and self.lam is None:
            self._say("note", step, f"{S} row {step.get('_row')}, program {pid}: the controller first moves the arm {gap:.0f} deg to "
                      f"the program's first point, at its Start % ({step['transit_speed']} %): that move takes "
                      f"{transit[0]:.1f}-{transit[2]:.1f} s on the robot, which widens this arm's timing from here.")
        move_t0 = t + _at(r_lead, lam)
        traj_t0 = move_t0 + _at(transit, lam)
        play_s = _at(play, lam)
        t1 = traj_t0 + play_s + _at(tail, lam)
        a = self._act(stage, step, "program", t, t1 - t, label, program=pid, drawn=True, file=info["file"],
                      file_name=info.get("file_name"), lines=n, move_t0=round(move_t0, 4), traj_t0=round(traj_t0, 4),
                      line_s=round(play_s / max(1, n - 1), 7), play_s=round((n - 1) * CYCLE_S, 4), transit=[round(x, 3) for x in transit],
                      gap_deg=round(gap, 2), phase=info.get("phase"), placement=info.get("placement"), by_name=bool(found),
                      times=_plus(r_lead, transit, play, tail))
        self.q = (md[-1] / 1000.0).tolist()
        self.known = True                            # on the program's own lines: known again (its move to them is not)
        return a


def stage_off(S: str, row, pid: int, st, pole, hand, set_pole: bool, set_hand: bool) -> str | None:
    """A Program's own pole and hand (what Lite recorded when it sent it: the ones it was taught with) against the ones in
    effect before it; what is off, or None. The owner, 21 Oct: a program runs with its own pole and hand, wherever it
    runs, never with others; in S2 the sequence's own Pole and Hand steps set them, as V53 runs it."""
    from . import staging

    off = []
    if pole is None or abs(float(pole) - st.pole_mm) > staging.POLE_TOL_MM:
        off.append((f"the pole is at {float(pole):g} mm here" if pole is not None else "the pole height here is not known")
                   + ("" if set_pole else " (as the arm stands: no Pole step before it in this run)"))
    if hand is None or not staging.same_hand(list(hand), st.hand):
        off.append((f"the hand is {staging.hand_words(list(hand))} here" if hand is not None else "the hand here is not known")
                   + ("" if set_hand else " (as the arm stands: no Hand step before it)"))
    if not off:
        return None
    return (f"{S} row {row}, program {pid}: taught at {staging.describe(st)}, but " + " and ".join(off)
            + ". It runs only with its own: set them before it (a Pole step and a Hand step).")


#: The commode as each task leaves it, in the camera checks' words: the expectations that hold there (a plan that runs
#: after another, a follow-up, has its checks answered by them in the simulation; a plan's own are answered as expected).
HOLDS_AFTER = {"lid_open": {"lid_open", "seat_closed"}, "seat_open": {"lid_open", "seat_open"}}


def hand_set(st: dict) -> set:
    """The arms whose next task's start sets the hand (``then`` with a hand): the plan's final release of that hand is
    not sent (mechanism.trailing_releases)."""
    return {s for s, t in (st.get("then") or {}).items() if t.get("hand") is not None}


def _sim_mechanism(st: dict, sims: dict, T: float, stage: int, overall: int, leave=(), hand_set=(), start_checks=False) -> float:
    """A lid or seat task in the simulation, as Lite runs it (mechanism.Runner) with its checks answered as expected (no
    camera here): each arm to the plan's start (its pole and hand, then the MoveJ at the speed slider's speed), then the
    plan's entries one after the other on the run's clock: each arm move from the plan's own 5 ms lines, the pole and the
    hand as Lite moves them, a hold where the plan holds, a moment where the camera is asked. The commode is in the
    task's end state from its end; the plan's own end look (its last check, since 26 Oct) is among its entries.
    The arms in ``leave`` (their next task's start follows, next_after_tasks) go to no plan's end."""
    from . import mechanism, staging

    plan = mechanism.cached(st["task"])
    title, rows = plan.title, st.get("rows") or {}
    mark = {s: {"_row": rows.get(s)} for s in ARMS}
    stale = plan.rungs_note()
    if stale:                                        # a missed lift would stop the run at its first re-lift
        sims["left"]._say("warn", mark["left"], stale)
    camera = "right"                                 # the D435 rides the right wrist
    clock = [T]

    def act(side, kind, dur, text, **more):
        sim = sims[side]
        a = sim._act(stage, mark[side], kind, clock[0], dur, text, mechanism=plan.name, **more)
        return sim, a

    def done(sim, a, dur):
        a.update(pole1=sim.pole, hand1=list(sim.hand), q1=list(sim.q))
        clock[0] += dur

    def approach(targets, where):
        for side, tg in targets.items():
            sim = sims[side]
            if "pole_mm" in tg or "hand" in tg:            # one step: the hand moving while the pole travels (26 Oct)
                mm = float(tg["pole_mm"]) if "pole_mm" in tg else sim.pole
                vals = [int(c) if c is not None else int(h) for c, h in zip(tg.get("hand") or [None] * 6, sim.hand)]
                dur = max(staging.pole_quick_s(sim.pole, mm), staging.hand_move_s(sim.hand, vals))
                sim, a = act(side, "pole_hand", dur, f"{title}: pole {mm:g} mm with the hand " + ",".join(str(v) for v in vals) + f" ({where})")
                sim.pole, sim.hand = mm, vals
                done(sim, a, dur)
        t_start, ends = clock[0], []                       # both arms' MoveJs at once (26 Oct: their paths checked in Real)
        for side, tg in targets.items():
            if "arm_deg" in tg:
                sim = sims[side]
                clock[0] = t_start
                if sim.known and max(abs(a - b) for a, b in zip(sim.q, tg["arm_deg"])) <= staging.MOVEJ_SKIP_DEG:
                    mv = ROBOT_TIMES["global_there_s"][1]       # already there: one reading, no MoveJ (26 Oct)
                    sim, a = act(side, "wait", mv, f"{title}: already at {where}, no MoveJ")
                else:
                    mv = movej_s(sim.q, tg["arm_deg"], float(overall), sim.rating)
                    sim, a = act(side, "global", mv, f"{title}: MoveJ to {where} ({overall} %, both arms at once)", move_s=round(mv, 4), drawn=True)
                sim.q, sim.known = [float(v) for v in tg["arm_deg"]], True
                done(sim, a, mv)
                ends.append(clock[0])
        clock[0] = max(ends) if ends else clock[0]

    def check(expect, words):
        sim, a = act(camera, "check", CHECK_S, f"{title}: the camera checks {expect}{words} (assumed here)", expect=expect)
        done(sim, a, CHECK_S)

    def entry(owner, e):
        sim = sims[e.side]
        if e.kind == "arm":
            sim, a = act(e.side, "stream", e.seconds, f"{title}: {e.stage}" + (f" ({owner.title}'s)" if owner is not plan else ""),
                         file=f"mechanism:{owner.name}:{e.i}", drawn=True, lines=len(e.rows), traj_t0=round(clock[0], 4))
            sim.q, sim.known = [v / 1000.0 for v in e.rows[-1]], True
            dur = e.seconds
        elif e.kind == "pole":
            mm = round(e.last()[0] * 1000.0, 1)
            dur = staging.pole_travel_s(sim.pole, mm)
            sim, a = act(e.side, "pole", dur, f"{title}: {e.stage} (pole {mm:g} mm)")
            sim.pole = mm
        else:
            vals = [int(c) if c is not None else int(h) for c, h in zip(e.hand_counts(), sim.hand)]
            dur = staging.hand_move_s(sim.hand, vals)
            sim, a = act(e.side, "hand", dur, f"{title}: {e.stage} (hand " + ",".join(str(v) for v in vals) + ")")
            sim.hand = vals
        done(sim, a, dur)

    def back(owner, e):                              # an arm move played backwards (the final check's way out)
        sim, a = act(e.side, "stream", e.seconds, f"{title}: back along {e.stage} ({owner.title}'s way in)",
                     file=f"mechanism:{owner.name}:{e.i}:back", drawn=True, lines=len(e.rows), traj_t0=round(clock[0], 4))
        sim.q, sim.known = [v / 1000.0 for v in e.rows[0]], True
        done(sim, a, e.seconds)

    def play(owner, holds=None) -> tuple:
        """``owner`` from its start: its entries and their checks, these answered by ``holds`` when given (ended at the
        first that does not hold and skips). True when it skipped."""
        approach(owner.targets("start"), "its start" if owner is plan else f"{owner.title}'s start")
        entries = list(owner.entries)
        if not start_checks and owner is plan:         # a sequence: the checks before the first lift are not asked (27 Oct)
            entries, _dropped = mechanism.without_start_checks(entries)
        unsent = mechanism.trailing_releases(entries, hand_set if owner is plan else ())
        for e in entries:
            for pa in e.pre_actions:
                if pa.get("type") == "hold_pose" and float(pa.get("seconds") or 0) > 0:
                    dur = float(pa["seconds"])
                    sim, a = act(e.side, "wait", dur, f"{title}: {pa.get('stage') or 'hold'} {dur:g} s")
                    done(sim, a, dur)
                elif pa.get("type") == "check_fixture_state":
                    expect = pa.get("expect")
                    check(expect, "" if owner is plan else f" ({owner.title}'s)")
                    if holds is not None and expect not in holds and pa.get("on_fail") == "skip":
                        return True
            if id(e) in unsent:                      # its hand is set for its next task's start right after
                continue
            entry(owner, e)
        return False

    play(plan)
    sim, a = act(plan.sides()[0], "wait", 0.0, f"{title}: done, {mechanism.TASKS[plan.name]['end']}",
                 phase=mechanism.PHASE_AFTER[plan.name])
    done(sim, a, 0.0)
    return clock[0]


def _sim_then(st: dict, sims: dict, T: float, stage: int, overall: int) -> float:
    """A run of lid and seat tasks' ``then`` (next_after_tasks) from ``T``: each arm to its next task's start, both arms
    at once, as Lite moves them in Real: its pole and hand together (Lite's pole and hand step), then, to a Program's
    first point, its errors read (a 4105 the plan's streams left cleared) and its MoveJ at the speed slider's speed (the
    global speed at 100 % for it). The end's time."""
    ends = [T]
    for s, t in sorted((st.get("then") or {}).items()):
        sim, row = sims[s], (st.get("rows") or {}).get(s)
        steps = []
        if t.get("pole_mm") is not None or t.get("hand") is not None:
            steps.append({"type": "pole_hand", "mm": float(t["pole_mm"]) if t.get("pole_mm") is not None else sim.pole,
                          "values": list(t["hand"]) if t.get("hand") is not None else list(sim.hand), "_row": row, "_then": t})
        if t.get("arm_deg"):
            steps += [{"type": "clear", "_row": row, "_alone": True},
                      {"type": "move_to", "deg": list(t["arm_deg"]), "speed": int(overall), "_row": row, "_then": t}]
        ends.append(sim.run(steps, T, stage))
    return max(ends)


CHECK_S = 0.5                    # a camera question, in the simulation (a frame, the check)


def _start_state(app, side: str, home: dict) -> tuple[dict, str]:
    """Where an arm starts: as it stands now (a linked arm's live reading), else the standing pose."""
    from . import staging

    link = app.robot.links.get(side)
    fb = (link.feedback if link is not None else {}) or {}
    pole0, hand0 = staging.home_stage(side, home)
    q_home = [round(float(__import__("math").degrees(home.get(f"{staging.SIDE_PREFIX[side]}_joint{i}", 0.0))), 4) for i in range(1, 8)]
    if fb.get("joints_deg") and len(fb["joints_deg"]) >= 7 and fb.get("pole_mm") is not None:
        hand = fb.get("hand") or hand0
        return dict(pole_mm=float(fb["pole_mm"]), hand=[int(c) for c in hand], q_deg=[float(v) for v in fb["joints_deg"][:7]]), "now"
    return dict(pole_mm=float(pole0), hand=[int(c) for c in hand0], q_deg=q_home), "standing"


def _segments(steps: list) -> list[tuple[str, object]]:
    """An arm's part of a stage as its runner calls run it: the stretches of rows between its Wait-for rows and
    Lite's own steps (("steps", [...])), each Wait-for (("wait", the row)), each clear (("clear", the step)) and each
    Pole with the Hand after it (("pair", the step))."""
    out, cur = [], []
    for x in steps:
        if x["type"] in ("wait_for", "clear", "pole_hand"):
            if cur:
                out.append(("steps", cur))
                cur = []
            out.append(({"wait_for": "wait", "clear": "clear", "pole_hand": "pair"}[x["type"]], x))
        else:
            cur.append(x)
    if cur:
        out.append(("steps", cur))
    return out


def _row_words(steps: list, row) -> str:
    st = next((x for x in steps if x.get("_row") == row and x["type"] != "clear"), None)
    return f"row {row}" + (f" ({_step_words(st)})" if st else "")


def _sim_part(st: dict, sims: dict, T: float, stage: int, words) -> dict:
    """A stage's parts on one clock from ``T``: each arm through its rows in its runner calls (each starting in
    ROBOT_TIMES call_start_s); at a Wait-for the arm holds until the other arm has finished the row it names (``words``
    says that row). Each arm's end time."""
    parts = {s: st[s]["steps"] for s in ARMS if st.get(s)}
    segs = {s: _segments(parts[s]) for s in parts}
    pos = {s: 0 for s in parts}
    clock = {s: T + _at(ROBOT_TIMES["call_start_s"], sims[s].lam) for s in parts}
    done: dict = {s: {} for s in parts}                 # each arm's rows finished in this stage: row -> when

    def mark(s):
        def one(step, t1):
            for r in _rows_done(step):
                done[s][r] = t1
        return one

    while any(pos[s] < len(segs[s]) for s in parts):
        moved = False
        for s in parts:
            while pos[s] < len(segs[s]):
                what, x = segs[s][pos[s]]
                if what == "clear":                     # Lite's, between two runner calls
                    clock[s] = sims[s].run([x], clock[s], stage)
                elif what == "pair":                    # Lite's Pole with the Hand, between two runner calls
                    clock[s] = sims[s].run([x], clock[s], stage, done=mark(s))
                elif what == "wait":
                    tg = x.get("_target") or {}
                    other = tg.get("side")
                    if tg.get("done") or other not in parts:
                        release = clock[s]                      # its row was finished before this stage
                    elif tg.get("row") in done[other]:
                        release = max(clock[s], done[other][tg["row"]])
                    else:
                        break                                   # the other arm has not finished that row yet
                    sims[s].hold(stage, x, clock[s], release, f"Wait for {str(other or '').upper()} {words(other, tg.get('row'))}",
                                 waits_for=dict(side=other, row=tg.get("row")))
                    done[s][x.get("_row")] = release
                    clock[s] = release + _at(ROBOT_TIMES["call_start_s"], sims[s].lam)   # the next runner call
                else:
                    clock[s] = sims[s].run(x, clock[s], stage, done=mark(s))
                pos[s] += 1
                moved = True
        if not moved:                                   # flow.build walked this already: never here
            raise UserError("The simulation found the arms waiting for each other for ever.", code="plan")
    return clock


def schedule(run: dict, sims: dict, overall: int = 100) -> tuple[list[dict], float]:
    """The plan on one clock: a stage between two meeting points starts both arms' parts at once (each through its
    own rows, holding at its Wait-for rows) and ends when both have, the first one holding at its Meet (an action, so
    every timing has the same actions); a single arm's run is one part; a lid or seat task takes both arms after both
    have finished what came before it."""
    T, out = 0.0, []
    every = {s: [x for st in run["plan"] for side, part in _items(st) if side == s for x in part] + list((run.get("steps") or {}).get(s) or [])
             for s in ARMS}
    words = lambda side, row: _row_words(every.get(side) or [], row)                  # noqa: E731
    plan = run["plan"]
    for i, st in enumerate(plan):
        kind = st["kind"]
        row = dict(index=i, kind=kind, label=str(st.get("program_id") or ""), t0=round(T, 4))
        if kind == "mechanism":                     # the arms' errors read first (clear_before_moves), then its own moves
            ready = T
            for s in st.get("clear") or []:
                ready = max(ready, sims[s].run([{"type": "clear", "_row": (st.get("rows") or {}).get(s), "_alone": True}], T, i))
            T = _sim_mechanism(st, sims, ready, i, overall, leave=set(st.get("then") or {}), hand_set=hand_set(st), start_checks=False)
            row["t1_plan"] = round(T, 4)                # ALIX's plans end here; Lite's own moves to the next starts follow
            if st.get("then"):
                T = _sim_then(st, sims, T, i, overall)
            row.update(t1=round(T, 4), title=then_title(st), task=st["task"], task_title=st["title"])
        else:
            ends = _sim_part(st, sims, T, i, words) if any(st.get(s) for s in ARMS) else {}
            T = max(ends.values() or [T])
            if kind == "parallel" and i + 1 < len(plan):          # the one that ended first holds for the other at the Meet
                nxt = plan[i + 1]
                for s, e in ends.items():
                    meet = (st.get("meet_rows") or {}).get(s)
                    sims[s].hold(i, {"_row": meet}, e, T, f"Meet the {flow.OTHER[s].upper()} arm" if nxt["kind"] == "parallel"
                                 else f"Wait for the {flow.OTHER[s].upper()} arm before {nxt.get('title') or 'the task'}")
            row.update(t1=round(T, 4))
            if kind == "single":
                row["label"] = f"{[s for s in ARMS if st.get(s)][0].upper()} SEQUENCE"
        for s in ARMS:
            if st.get(s):
                row[f"{s}_programs"] = [int(x["id"]) for x in st[s]["steps"] if x["type"] == "program"]
        out.append(row)
    return out, T


def sample(acts: list[dict], times, start: dict, md_of, with_known: bool = False):
    """An arm's pole (mm), hand (counts) and joints (deg) at ``times`` (sorted), from its actions; with
    ``with_known``, also whether its pose is known at each (after a step not drawn it is not, until a
    drawn Program's own lines)."""
    import numpy as np

    n = len(times)
    pole, hand, q = np.empty(n), np.empty((n, 6)), np.empty((n, 7))
    known = np.ones(n, dtype=bool)
    cur = (float(start["pole_mm"]), np.asarray(start["hand"], dtype=float), np.asarray(start["q_deg"], dtype=float))
    cur_known = True
    j = 0
    for k, t in enumerate(times):
        while j < len(acts) and acts[j]["t1"] <= t:
            a = acts[j]
            cur = (float(a["pole1"]), np.asarray(a["hand1"], dtype=float), np.asarray(a["q1"], dtype=float))
            cur_known = _known_after(a)
            j += 1
        if j < len(acts) and acts[j]["t0"] <= t:
            a = acts[j]
            pole[k], hand[k], q[k] = _inside(a, t, md_of)
            known[k] = (a.get("known", True) or (a["kind"] == "program" and a.get("drawn") and t >= a["traj_t0"])) \
                and not (a.get("drawn") is False)
        else:
            pole[k], hand[k], q[k] = cur
            known[k] = cur_known
    return (pole, hand, q, known) if with_known else (pole, hand, q)


def _known_after(a: dict) -> bool:
    if a.get("drawn") is False:
        return False
    if a["kind"] in ("program", "stream", "global"):     # on its own lines, or at its point
        return True
    return bool(a.get("known", True))


def _inside(a: dict, t: float, md_of):
    import numpy as np

    p0, h0, q0 = float(a["pole0"]), np.asarray(a["hand0"], dtype=float), np.asarray(a["q0"], dtype=float)
    p1, h1, q1 = float(a["pole1"]), np.asarray(a["hand1"], dtype=float), np.asarray(a["q1"], dtype=float)
    span = max(1e-9, a["t1"] - a["t0"])
    f = (t - a["t0"]) / span
    if a["kind"] == "pole":
        return p0 + float(_ease(f)) * (p1 - p0), h0, q0
    if a["kind"] == "hand":
        return p0, h0 + min(1.0, max(0.0, f)) * (h1 - h0), q0
    if a["kind"] == "pole_hand":                 # both from its start: the pole over its travel, the hand over its stroke
        dt = t - a["t0"]
        fp = min(1.0, max(0.0, dt / max(1e-9, float(a.get("pole_s") or span))))
        fh = min(1.0, max(0.0, dt / max(1e-9, float(a.get("hand_s") or span))))
        return p0 + float(_ease(fp)) * (p1 - p0), h0 + fh * (h1 - h0), q0
    if a["kind"] == "global" and a.get("drawn"):
        g = (t - a["t0"]) / max(1e-9, a["move_s"])
        return p0, h0, q0 + float(_ease(g)) * (q1 - q0)
    if a["kind"] == "stream":                    # a lid or seat plan's arm move: its own 5 ms lines
        md = md_of(a["file"])
        if md is None:
            raise MissingFile(a)
        i = int(round((t - a["t0"]) / CYCLE_S))
        return p0, h0, md[max(0, min(md.shape[0] - 1, i))] / 1000.0
    if a["kind"] == "program" and a.get("drawn"):
        md = md_of(a["file"])
        if md is None:
            raise MissingFile(a)
        if t < a["traj_t0"]:                     # still where it was while the controller reads the file, then its move
            m0 = a.get("move_t0", a["t0"])      # to the first point (a simulation made before 22 Oct: from the start)
            g = (t - m0) / max(1e-9, a["traj_t0"] - m0)
            return p0, h0, q0 + float(_ease(g)) * (md[0] / 1000.0 - q0)
        i = int(round((t - a["traj_t0"]) / float(a.get("line_s") or CYCLE_S)))
        return p0, h0, md[max(0, min(md.shape[0] - 1, i))] / 1000.0
    return p0, h0, q0


def _numbered(steps: dict) -> dict:
    """Each step with its row in its arm's list (``_row``), carried through V53's plan."""
    return {s: [dict(st, _row=i) for i, st in enumerate(steps[s], 1)] for s in ARMS}


def _sim_file(units) -> Path:
    return Path(units.root) / "sequence" / "s2_sim.json"


def load_sim(units) -> dict | None:
    try:
        return json.loads(_sim_file(units).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def plans_changed(sim: dict | None) -> list[str]:
    """The lid and seat tasks whose ALIX plan is not the one ``sim`` was made with (revised and vendored since, as on
    21 Oct): a simulation keeps their moves by their place in the plan, so it no longer plays. Their titles."""
    from . import mechanism

    made = (((sim or {}).get("identity") or {}).get("used") or {}).get("mechanisms") or {}
    out = []
    for task, sha in sorted(made.items()):
        try:
            same = mechanism.digest(task) == sha
        except UserError:
            same = False
        if not same:
            out.append(mechanism.TASKS.get(task, {}).get("title", task))
    return out


def _require_plans(sim: dict) -> None:
    changed = plans_changed(sim)
    if changed:
        raise UserError(f"ALIX's plan of {' and '.join(changed)} is not the one this simulation was made with (it was revised "
                        "since), so the simulation no longer plays.", "Run the sequence again in Simulation Mode.", code="plans")


PROJECT_DIRS = ("trajectories", "merged", "sequence", "setup", "simulation", "controller_exports")


def portable(path, root) -> str:
    """A file's path as the simulation keeps it: relative to the project's folder when the file is in it (the
    simulation then plays on any computer the project is copied to), else as it is (a library file)."""
    try:
        return str(Path(str(path)).relative_to(Path(root)))
    except ValueError:
        return str(path)


def resolve_file(path, root) -> Path:
    """A simulation's file on this computer: a relative path is under the project's folder; an absolute one is
    taken as it is, or, when it is another computer's copy of this project (a simulation made in the lab and
    pulled with the project), the same file under the project's folder here."""
    p = Path(str(path))
    if not p.is_absolute():
        return Path(root) / p if root else p
    if p.exists() or not root:
        return p
    root = Path(root)
    parts = list(p.parts)
    if root.name in parts:                                   # .../Projects/<this project>/trajectories/...: after its name
        tail = parts[len(parts) - parts[::-1].index(root.name):]
        if tail and (root / Path(*tail)).exists():
            return root / Path(*tail)
    for i, part in enumerate(parts):                         # a project folder renamed: by the folder the file is in
        if part in PROJECT_DIRS and (root / Path(*parts[i:])).exists():
            return root / Path(*parts[i:])
    return p


class MissingFile(Exception):
    """A drawn Program's file is not on this computer (the simulation came from another)."""

    def __init__(self, act: dict):
        super().__init__(act.get("file"))
        self.act = act


def _md_reader(root=None):
    """A reader of the files a simulation draws, cached; ``root``: the project's folder, which a relative path
    (portable) is under, and under which another computer's absolute path is looked for (resolve_file)."""
    import numpy as np

    from ..motion import track

    cache: dict = {}

    def md_of(path):
        if not path:
            return None
        if path not in cache and str(path).startswith("mechanism:"):     # mechanism:<task>:<entry>: ALIX's plan, its lines
            from . import mechanism

            part = str(path).split(":")                                  # ...:back: the move played backwards
            rows = mechanism.cached(part[1]).entries[int(part[2])].rows
            cache[path] = np.asarray(rows[::-1] if part[3:] == ["back"] else rows, dtype=float)
        if path not in cache:
            try:
                cache[path] = np.asarray(track.read_md(resolve_file(path, root)), dtype=float)
            except (OSError, ValueError):
                cache[path] = None
        return cache[path]

    return md_of


def _phase_changes(acts: dict) -> list[tuple[float, str]]:
    """The commode's lid/seat state through the run: each drawn Program's region's, from its start."""
    out = sorted((a["t0"], a["phase"]) for side in acts.values() for a in side
                 if a.get("phase") and (a.get("mechanism") or (a["kind"] == "program" and a.get("drawn"))))
    return out or [(0.0, "lid_closed")]


def _phase_at(changes, t: float) -> str:
    ph = changes[0][1]
    for t0, p in changes:
        if t0 <= t + 1e-9:
            ph = p
        else:
            break
    return ph


DRIFT_SAMPLE_S = 0.04          # the arms checked against each other at every timing: every 8th 5 ms line
DRIFT_STEP_S = 0.5             # timings at most this far apart, from one end of the two arms' ranges to the other,
DRIFT_MAX_TIMINGS = 21         # but no more of them than this (each is the whole run checked again): a coarser check is said


def _band(acts: dict, stage: int) -> float:
    """How far apart the two arms' timings can drift in a stage: each arm's steps' ranges added up, both arms."""
    return sum(float((a.get("times") or (0, 0, 0))[2]) - float((a.get("times") or (0, 0, 0))[0])
               for s in ARMS for a in acts.get(s) or [] if a["stage"] == stage)


def _drift(run, make_sims, overall, acts_nominal, base, job) -> dict:
    """The two arms against each other at every timing their ranges allow (the owner, 22 Oct): the run scheduled again
    with one arm at a point of its range and the other at the opposite point (from the left as fast as it runs and the
    right as slow, to the reverse), each stage's both-arm parts sampled every DRIFT_SAMPLE_S, and every moment checked
    for CLEARANCE_MM between the arms (collision_check.clearance_states). Each pair of rows that comes closer: the
    least gap, the links, when, and how far one arm was ahead of the other then."""
    import numpy as np

    from . import collision_check, workers

    both = [i for i, st in enumerate(run["plan"]) if st["kind"] == "parallel" and st.get("left") and st.get("right")]
    if not both:
        return dict(timings=0, band_s=0.0, events=[], findings=[])
    t_start = time.monotonic()
    bands = {i: _band(acts_nominal, i) for i in both}
    widest = max(bands, key=bands.get)
    band = bands[widest]
    n = 1 if band < 0.05 else int(min(DRIFT_MAX_TIMINGS, max(3, np.ceil(band / DRIFT_STEP_S) + 1)))
    lams = [None] + ([] if n == 1 else [float(u) for u in np.linspace(0.0, 1.0, n)])
    step_s = band / (n - 1) if n > 1 else 0.0
    cases, scenarios = [], []
    for lam in lams:
        sims = make_sims({"left": lam, "right": None if lam is None else 1.0 - lam})
        stages, total = schedule(run, sims, overall)
        acts = {s: sims[s].acts for s in ARMS}
        spans = [(r["t0"], r["t1"]) for r in stages if r["index"] in both]
        times = np.concatenate([np.arange(t0, t1, DRIFT_SAMPLE_S) for t0, t1 in spans] or [np.zeros(0)])
        arms = {}
        for s in ARMS:
            pole, hand, q, known = sample(acts[s], times.tolist(), run["start"][s], run["md_of"], with_known=True)
            arms[s] = dict(pole=pole.astype(np.float32), hand=hand.astype(np.float32), q=q.astype(np.float32), known=known)
        cases.append(dict(lam=lam, acts=acts, stages=stages))
        scenarios.append(dict(times=times.tolist(), arms=arms))
    job.progress(f"Checking the two arms {CLEARANCE_MM:g} mm apart at {len(lams)} timings: "
                 f"{sum(len(x['times']) for x in scenarios):,} moments", 3, 4)
    fut = workers.pool().submit(collision_check.clearance_states, scenarios, base, CLEARANCE_MM)
    while not fut.done():
        job.sleep(0.2)
    found = fut.result()
    job.check()

    def ahead(case, t):
        """How far ahead of its time as a rule each arm is at ``t`` in this timing (by its action then)."""
        out = {}
        for s in ARMS:
            i = next((k for k, a in enumerate(case["acts"][s]) if a["t0"] <= t < a["t1"]), None)
            if i is None or i >= len(acts_nominal[s]):
                out[s] = 0.0
                continue
            a, b = case["acts"][s][i], acts_nominal[s][i]
            f = (t - a["t0"]) / max(1e-9, a["t1"] - a["t0"])
            out[s] = (b["t0"] + f * (b["t1"] - b["t0"])) - t       # positive: where the arm as a rule is later
        return out

    pairs: dict = {}
    for case, res in zip(cases, found):
        for e in res["events"]:
            t = e["first_t"] + 0.5 * (e["last_t"] - e["first_t"])
            at = {s: _where(case["acts"][s], t) for s in ARMS}
            key = tuple((at[s] or {}).get("row") for s in ARMS)
            lead = ahead(case, t)
            rel = lead["right"] - lead["left"]                    # positive: the right arm ahead of the left
            ev = dict(e, at=[at[s] for s in ARMS if at[s]], lam=case["lam"], right_ahead_s=round(rel, 2), t=round(t, 2))
            prev = pairs.get(key)
            if prev is None or e["min_mm"] < prev["min_mm"]:              # the closest kept, every timing counted once
                pairs[key] = dict(ev, lams=(prev or {}).get("lams", set()), ahead=(prev or {}).get("ahead", []),
                                  rule_mm=(prev or {}).get("rule_mm"))
            pairs[key]["lams"].add(case["lam"])
            pairs[key]["ahead"].append(rel)
            if case["lam"] is None:                                       # as timed as a rule: its own least gap
                had = pairs[key]["rule_mm"]
                pairs[key]["rule_mm"] = e["exact_mm"] if had is None else min(had, e["exact_mm"])
    events, findings = sorted(pairs.values(), key=lambda e: e["min_mm"]), []
    gap = lambda mm: f"{mm:.0f} mm apart" if mm > 0 else "touching"           # noqa: E731
    for e in events:
        rows = " and ".join(f"{w['side'].upper()} row {w['row']} ({w['text']}"
                            + (f", line {w['line']:,}" if w.get("line") else "") + ")" for w in e["at"])
        span, lams_hit = sorted(e.pop("ahead")), e.pop("lams")
        e["timings"], e["as_rule"] = len(lams_hit), None in lams_hit
        lead = (f"when the RIGHT arm runs {e['right_ahead_s']:.1f} s ahead of the LEFT" if e["right_ahead_s"] > 0.05 else
                f"when the LEFT arm runs {-e['right_ahead_s']:.1f} s ahead of the RIGHT" if e["right_ahead_s"] < -0.05 else
                "with the arms on time")
        counted = f"in {e['timings']} of the {len(lams)} timings checked, from {min(span):+.1f} to {max(span):+.1f} s"
        if e["timings"] == len(lams):                     # whatever their timing: the rows themselves put them there
            text = (f"{gap(e['exact_mm'])} ({', '.join(e['links']) or 'their links'}) at every timing checked: those rows put the arms "
                    "that close whatever their timing. Change those steps (the arms' poses or hands there).")
        elif e["as_rule"]:
            text = (f"{gap(e['rule_mm'])} as timed as a rule, {gap(e['exact_mm'])} ({', '.join(e['links']) or 'their links'}) {lead} "
                    f"({counted}). Keep the arms apart with a Meet or a Wait-for between those rows.")
        else:
            text = (f"{gap(e['exact_mm'])} ({', '.join(e['links']) or 'their links'}) {lead} ({counted}). Those timings are within the "
                    "robot's ranges: keep the arms apart with a Meet or a Wait-for between those rows.")
        findings.append(dict(level="warn", side=None, row=None, text=(
            f"The two arms come closer than {CLEARANCE_MM:g} mm: {rows}, about {e['t']:.0f} s in: {text}")))
    if step_s > DRIFT_STEP_S + 1e-6:                               # said, never a silent cap: what the check can miss
        st = run["plan"][widest]
        findings.append(dict(level="note", side=None, row=None, text=(
            f"{st.get('program_id') or 'A part'} has its two arms going on their own for long (no Meet in it): their timings can drift "
            f"up to {band:.1f} s apart (each arm's steps' ranges added up), so the {n} timings checked are up to {step_s:.1f} s apart "
            "and a closer approach between two of them can be missed. A Meet or a Wait-for in it makes the check finer.")))
    ranges = {}
    for lam in ((0.0, 0.0), (1.0, 1.0)):                           # each stage as fast and as slow as both arms run
        sims = make_sims({"left": lam[0], "right": lam[1]})
        for r in schedule(run, sims, overall)[0]:
            ranges.setdefault(r["index"], []).append(r["t1"])
    return dict(timings=len(lams), band_s=round(band, 2), step_s=round(step_s, 2), clearance_mm=CLEARANCE_MM, events=events, findings=findings,
                stage_t1={str(k): [round(min(v), 2), round(max(v), 2)] for k, v in ranges.items()},
                samples=sum(r["samples"] for r in found), seconds=round(time.monotonic() - t_start, 1))


def simulate(app, units, mode: str, pick: dict | None):
    """Simulation Mode: the run (``plan_for``) through its plan on one clock, both arms, in the room and at the UGV
    placement its Programs were taught at, each step as long as it takes on the robot as a rule; the collision check
    over all of it; the two arms against each other at every timing their ranges allow (``_drift``); and what would not
    be right on the robot (``findings``). Kept as the project's last simulation, with what makes it the same run
    (``identity_of``)."""
    from . import collision_check, placement, staging, workers
    from ..motion.limits import load_limits
    from .routes_robot import _payload

    steps = _numbered(load(units))
    run = plan_for(steps, mode, pick)                 # refused here when it cannot run, before the job
    overall = int(app.robot.speed_pct)
    home = dict(_payload(app)["home"])
    cub = int(app.ui.get("cubicle_mm", 1400))         # the cubicle chosen, when the room is not measured

    def work(job):
        import numpy as np

        nonlocal run
        job.progress("Reading what the controllers hold", 0, 3)
        sides = list(run["steps"])
        programs = {s: programs_known(app, units, s) for s in ARMS}
        points = {s: points_known(app, s) for s in ARMS}
        findings, md_of = [], _md_reader(units.root)
        named = {s: files_by_name(app, units, s) for s in ARMS}
        run = plan_for(steps, mode, pick, own=own_for(app, units, programs, run["steps"]))   # each Program's own pole and hand
        ident = identity_of(run, programs, points, overall)
        starts, start_how = {}, {}
        for s in ARMS:
            starts[s], start_how[s] = _start_state(app, s, home)

        def make_sims(lam, said=None):
            said = findings if said is None else said
            return {s: _ArmSim(s, starts[s], load_limits(s).rating_deg_s, overall, programs[s], points[s], md_of, said, named[s],
                               lam=lam.get(s)) for s in ARMS}

        sims = make_sims({"left": None, "right": None})
        findings.extend(dict(level="note", side=None, row=None, text=t) for t in run.get("said") or [])
        job.progress("Timing the stages", 1, 4)
        stages, total = schedule(run, sims, overall)
        acts = {s: sims[s].acts for s in ARMS}
        places = [a.get("placement") for s in sides for a in acts[s] if a["kind"] == "program" and a.get("drawn")]
        place = next((p for p in places if p), None)
        if any(p and placement.differences(place, p) for p in places):
            findings.append(dict(level="warn", side=None, row=None, text="The Programs were taught at different UGV placements: "
                                 f"the 3D and the check use the first one's ({placement.words(place)})."))
        if any(row["kind"] == "mechanism" for row in stages):
            from . import mechanism

            plans_at = {"standoff_mm": float(mechanism.STANDOFF_MM), "front_mm": 0.0, "rear_mm": 0.0, "left_mm": 0.0, "right_mm": 0.0}
            off = sorted({placement.words(p) for p in places
                          if p and placement.differences(plans_at, {k: v for k, v in p.items() if k != "room"})})
            findings.append(dict(level="warn" if off else "note", side=None, row=None, text=(
                f"The lid and seat tasks are ALIX's plans, made with the UGV at {mechanism.STANDOFF_MM} mm from the commode and no "
                "offset" + (f"; Programs of this run were taught at {'; '.join(off)}: the UGV cannot be at both, so one of them will "
                            "be off its commode." if off else ": place the UGV there for them (Lite cannot see the placement)."))))
        scene = placement.scene(units.fixture, placement.load(units)["room"], place)
        base = dict(home)
        for s in ARMS:
            base = staging.start_values(s, base, starts[s])
        changes = _phase_changes(acts)
        dt = max(SAMPLE_S, total / MAX_SAMPLES)
        times = np.arange(0.0, total + 1e-9, dt).tolist() if total > 0 else [0.0]
        arms = {}
        spans = [(row["t0"], row.get("t1_plan", row["t1"]), row.get("task_title") or row["title"], row["t1"])
                 for row in stages if row["kind"] == "mechanism"]
        for s in sides:
            pole, hand, q, known = sample(acts[s], times, starts[s], md_of, with_known=True)
            for t0, t1, _title, _end in spans:     # ALIX's planner checked these moves, the hands on the lid by design
                known[(np.asarray(times) >= t0) & (np.asarray(times) <= t1)] = False
            arms[s] = dict(pole=pole.tolist(), hand=hand.tolist(), q=q.tolist(), known=known.tolist())
        for t0, t1, title, end in spans:
            findings.append(dict(level="note", side=None, row=None, text=f"{title} ({t0:.1f}-{t1:.1f} s) is ALIX's plan: its "
                                 "hands hold the lid and the seat by design, so it is left out of this collision check (ALIX's "
                                 "planner checked it with the commode's contact allowances); its camera checks are assumed "
                                 "here, and asked of the camera in Real."
                                 + (f" Each arm's move to its next task's start after it ({t1:.1f}-{end:.1f} s) is Lite's own, "
                                    "checked here." if end > t1 + 1e-6 else "")))
        job.progress(f"Checking collisions: {len(times):,} moments, both arms", 2, 4)
        phases = [_phase_at(changes, t) for t in times]
        fut = workers.pool().submit(collision_check.check_states, times, arms, base, scene, units.fixture, phases, cub)
        while not fut.done():                         # STOP is heard while it checks
            job.sleep(0.2)
        check = fut.result()
        job.check()
        for e in check["events"]:                         # where in the sequence: each arm's row and line then
            e["at"] = [_where(acts[s], e["first_t"]) for s in (ARMS if "both" in e["sides"] else e["sides"]) if s in acts]
        drift = _drift(dict(run, start=starts, md_of=md_of), lambda lam: make_sims(lam, []), overall, acts, base, job) \
            if run["mode"] == "both" else dict(timings=0, band_s=0.0, events=[], findings=[])
        findings.extend(drift.pop("findings"))
        for row in stages:                                # each stage's end as fast and as slow as both arms run it
            row["t1_range"] = drift.get("stage_t1", {}).get(str(row["index"]))
        for s in acts:                                    # the files as the project holds them: the record plays elsewhere too
            for a in acts[s]:
                if a.get("file") and not str(a["file"]).startswith("mechanism:"):
                    a["file"] = portable(a["file"], units.root)
        out = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), hash=digest(ident), identity=ident, completed=False,
                   mode=run["mode"], pick=run["pick"], overall=overall, counts=run["counts"], stages=stages,
                   total_s=round(total, 3), acts=acts, start=starts, start_from=start_how,
                   findings=findings, collisions=check, drift=drift, scene=scene, phases=changes,
                   programs={s: {str(k): {kk: v.get(kk) for kk in ("name", "sent", "file_name", "region", "gone", "lines", "seconds")}
                                 for k, v in programs[s].items()} for s in ARMS},
                   points={s: (points[s] or {}).get("read") for s in ARMS})
        from .project import _atomic_write

        _sim_file(units).parent.mkdir(parents=True, exist_ok=True)
        _atomic_write(_sim_file(units), json.dumps(out))
        job.log(f"Simulated: {len(stages)} stage(s), {total:.1f} s; " + ("no collision" if check["ok"] else
                f"{len(check['events']) + check['more']} collision event(s), the first at {check['first']['first_t']:.2f} s")
                + (f"; the two arms checked {CLEARANCE_MM:g} mm apart at {drift['timings']} timing(s): "
                   + (f"{len(drift['events'])} place(s) closer" if drift["events"] else "never closer") if drift.get("timings") else "")
                + f"; {sum(1 for f in findings if f['level'] == 'warn')} warning(s).")
        return out

    return app.jobs.start("simulate", f"S2 simulation ({'both arms' if mode == 'both' else mode.upper() + ' arm'})", work)


def _where(acts: list[dict], t: float) -> dict | None:
    a = next((x for x in acts if x["t0"] <= t < x["t1"]), None) or next((x for x in reversed(acts) if x["t0"] <= t), None)
    if a is None:
        return None
    out = dict(side=a["side"], row=a["row"], kind=a["kind"], text=a["text"], stage=a["stage"])
    if a["kind"] in ("program", "stream") and a.get("drawn") and t >= a["traj_t0"]:
        out["line"] = 1 + int(round((t - a["traj_t0"]) / float(a.get("line_s") or CYCLE_S)))
    if a.get("mechanism"):
        out["mechanism"] = a["mechanism"]
    return out


def _not_here(a: dict) -> UserError:
    """The refusal for a simulation whose drawn file is not on this computer (made elsewhere, pulled with the project)."""
    what = f"Program {a['program']}" if a.get("program") is not None else a.get("text") or "a step"
    name = a.get("file_name") or a.get("file")
    return UserError(f"The simulation's file for {a['side'].upper()} row {a.get('row')}, {what}" + (f" ({name})" if name else "")
                     + " is not on this computer.",
                     "Run the sequence again in Simulation Mode: it draws the project's files as they are here.", code="files")


def frames(app, units, side: str, stage: int, want_hash: str | None = None) -> dict:
    """One arm's part of a simulated stage as 3D frames (the shape of staging.execution_timeline):
    every link the pole moves, at times on the run's clock. ``stage`` -1: where the arm starts."""
    import base64

    import numpy as np

    from . import robot_model, staging
    from .routes_robot import _mesh_links, _payload

    sim = load_sim(units)
    if not sim:
        raise UserError("There is no simulation yet.", "Run the sequence in Simulation Mode.")
    if want_hash and sim.get("hash") != want_hash:          # another simulation was made since (another page)
        raise UserError("That simulation is no longer the last one.", "Run it again in Simulation Mode.", code="stale")
    _require_plans(sim)
    acts = sim["acts"][side]
    mine = [a for a in acts if a["stage"] == int(stage)]
    if int(stage) == -1:
        times = [0.0]
    elif not mine:
        raise UserError(f"The {side.upper()} arm has nothing in that stage.")
    else:                                   # from the stage's start: its runner call starts before its first step
        t0 = next((r["t0"] for r in sim.get("stages") or [] if r.get("index") == int(stage)), mine[0]["t0"])
        t1 = mine[-1]["t1"]
        dt = max(FRAME_S, (t1 - t0) / MAX_FRAMES)
        times = np.arange(t0, t1, dt).tolist() + [t1]
    try:
        pole, hand, q = sample(acts, times, sim["start"][side], _md_reader(units.root))
    except MissingFile as exc:
        raise _not_here(exc.act) from None
    p = _payload(app)
    links = staging.staged_links(side, _mesh_links(p))
    model = robot_model.fk_model()
    base = dict(p["home"])
    for s in ARMS:
        base = staging.start_values(s, base, sim["start"][s])
    hu = staging.hand_units()
    s_ = staging.SIDE_PREFIX[side]
    arm_names = [f"{s_}_joint{i}" for i in range(1, 8)]
    out = np.empty((len(times), len(links), 16), dtype="<f4")
    for f in range(len(times)):
        v = dict(base)
        v[f"{s_}_sliding_plate_joint"] = float(pole[f]) / 1000.0
        for ch in range(6):
            v[hu.joint_name(side, ch)] = hu.hw_to_rad(float(hand[f][ch]), ch)
        v.update(zip(arm_names, np.radians(q[f]).tolist()))
        tf = model.link_world_transforms(v)
        out[f] = [tf[ln].reshape(-1) for ln in links]
    return dict(side=side, stage=int(stage), links=list(links), times=[round(t, 4) for t in times],
                frames=base64.b64encode(out.tobytes()).decode("ascii"), total_s=round(times[-1], 4), lines=None,
                phases=[], traj_t0=0.0, stage_info=None)


_IN_STATE: dict = {}                     # the last simulation checked in one commode state, by what it was made of


def check_in(app, units, want_hash: str, phase: str):
    """The last simulation checked again with the commode in one state throughout (S2's 3D, its Commode
    choice: the owner, 16 Oct, "for manual override of the preset state", in the view only; the run and
    its own check stay as they are): the same moments, both arms, against the same walls. A job."""
    from . import collision_check, staging, workers
    from .lite_units import PHASES
    from .routes_robot import _payload

    if phase not in PHASES:
        raise UserError("The commode's state is lid closed, lid open or seat open.")
    sim = load_sim(units)
    if not sim:
        raise UserError("There is no simulation yet.", "Run the sequence in Simulation Mode.")
    if want_hash and sim.get("hash") != want_hash:
        raise UserError("That simulation is no longer the last one.", "Run it again in Simulation Mode.", code="stale")
    _require_plans(sim)
    cub = int(app.ui.get("cubicle_mm", 1400))
    home = dict(_payload(app)["home"])
    made_of = json.dumps([sim["hash"], sim.get("made"), phase, cub, sim.get("scene"), units.fixture], sort_keys=True)
    words = phase.replace("_", " ")

    def work(job):
        import numpy as np

        out = _IN_STATE.get(made_of)
        if out is None:
            total = float(sim.get("total_s") or 0.0)
            dt = max(SAMPLE_S, total / MAX_SAMPLES)
            times = np.arange(0.0, total + 1e-9, dt).tolist() if total > 0 else [0.0]
            md_of = _md_reader(units.root)
            arms = {}
            for s in (ARMS if sim["mode"] == "both" else [sim["mode"]]):
                try:
                    pole, hand, q, known = sample(sim["acts"][s], times, sim["start"][s], md_of, with_known=True)
                except MissingFile as exc:
                    raise _not_here(exc.act) from None
                arms[s] = dict(pole=pole.tolist(), hand=hand.tolist(), q=q.tolist(), known=known.tolist())
            base = dict(home)
            for s in ARMS:
                base = staging.start_values(s, base, sim["start"][s])
            job.progress(f"Checking collisions with the {words}: {len(times):,} moments", 0, 1)
            fut = workers.pool().submit(collision_check.check_states, times, arms, base, sim["scene"], units.fixture,
                                        [phase] * len(times), cub)
            while not fut.done():                     # STOP is heard while it checks
                job.sleep(0.2)
            out = fut.result()
            if len(_IN_STATE) > 16:
                _IN_STATE.pop(next(iter(_IN_STATE)))
            _IN_STATE[made_of] = out
        return dict(out, phase=phase, hash=sim["hash"])

    return app.jobs.start("collisions", f"S2 simulation checked with the {words}", work)


def played(units, h: str) -> dict:
    """The page played the simulation ``h`` to its end: Real Machine Mode may run that same sequence."""
    from .project import _atomic_write

    with units.lock:
        sim = load_sim(units)
        if not sim or sim.get("hash") != h:
            raise UserError("That simulation is no longer the last one.", "Run it again in Simulation Mode.")
        _require_plans(sim)
        if not (sim.get("total_s") or 0) > 0 or not any(a.get("drawn") is not False and a["t1"] > a["t0"] and a.get("kind") != "wait"
                                                        for side in sim["acts"].values() for a in side):     # a Wait moves nothing
            raise UserError("Nothing in this run can be shown in 3D, so it cannot be checked in Simulation Mode.",
                            "Send its programs from Lite (S1 or Real Test), then simulate it again.", code="nothing")
        sim["completed"] = time.strftime("%Y-%m-%d %H:%M:%S")
        _atomic_write(_sim_file(units), json.dumps(sim))
    return dict(hash=h, completed=sim["completed"])


def summary(units) -> dict:
    """The page's view: both arms' steps and the last simulation without its heavy parts."""
    sim = load_sim(units)
    if sim:
        sim = {k: v for k, v in sim.items() if k not in ("acts", "identity", "start")} | {
            "acts": {s: [{k: a.get(k) for k in ("row", "kind", "t0", "t1", "text", "stage", "drawn", "program", "traj_t0", "lines", "gate")}
                         for a in sim["acts"][s]] for s in ARMS},
            "steps": sim["identity"]["steps"], "plans_changed": plans_changed(sim)}
    return dict(load(units), sim=sim)


# ── Real Machine Mode: V53's run, stage by stage, each arm's part through runner v14 on its controller

def _changed(sim: dict | None, ident: dict) -> str:
    """What is not as it was simulated, in words."""
    if not sim:
        return "it has not been simulated"
    old = sim.get("identity") or {}
    what = []
    other_run = old.get("mode") != ident["mode"] or old.get("pick") != ident["pick"]
    if other_run:
        what.append("what is run (the sequence, the task or the arms)")
    if old.get("steps") != ident["steps"]:
        what.append("its steps")
    elif not other_run and old.get("plan") != ident.get("plan"):  # the same steps, planned otherwise: Lite was updated
        what.append("how its steps are planned into stages (Lite plans them otherwise since)")
    for side, u in ident["used"].items() if not other_run else ():   # another run uses other programs and points
        if side not in ARMS:                        # "mechanisms": the lid and seat plans, below
            continue
        o = (old.get("used") or {}).get(side) or {}
        if o.get("programs") != u["programs"]:
            what.append(f"the {side.upper()} programs Lite sent")
        if o.get("points") != u["points"]:
            what.append(f"the {side.upper()} global points (as read from the controller)")
    om, nm = (old.get("used") or {}).get("mechanisms") or {}, ident["used"].get("mechanisms") or {}
    if not other_run and om != nm:
        from . import mechanism

        names = [mechanism.TASKS.get(t, {}).get("title", t) for t in sorted(set(om) | set(nm)) if om.get(t) != nm.get(t)]
        what.append(f"ALIX's plan of {' and '.join(names)} (revised since)")
    if old.get("overall") != ident["overall"]:
        what.append(f"the speed (simulated at {old.get('overall')} %, now {ident['overall']} %)")
    return "changed since it was simulated: " + ", ".join(what) if what else "changed since it was simulated"


_PROGRAM_RE = __import__("re").compile(r"WORKFLOW PROGRAM \d+/\d+: slot (\d+)")
_COMPLETE_RE = __import__("re").compile(r"WORKFLOW PROGRAM COMPLETE \d+/\d+: ([0-9.]+) s")


def split_waits(steps: list) -> list:
    """A global point's wait as a Wait step of its own after it (the same motion and the same wait): the
    runner sleeps a point's wait in one call, which a stop cannot cut short, and a Wait step in 50 ms
    slices (V53's SIGINT cut both). Each keeps its row."""
    out = []
    for st in steps:
        if st.get("type") == "global" and float(st.get("wait_s") or 0) > 0:
            out.append(dict(st, wait_s=0.0))
            out.append({"type": "wait", "seconds": float(st["wait_s"]), "_row": st.get("_row")})
        else:
            out.append(st)
    return out


def _step_words(step: dict) -> str:
    k = step["type"]
    if step.get("_own") and k in ("pole", "hand", "pole_hand"):        # a Program's own, set before it (own_stages)
        return f"Program {step['_own']}'s own " + _step_words({x: v for x, v in step.items() if x != "_own"}).replace(
            "Pole", "pole", 1).replace("Hand", "hand", 1)
    if step.get("_then") and k in ("pole_hand", "move_to"):            # a next task's start (next_after_tasks)
        t = step["_then"]
        where = f"Program {t['id']}'s" if t["kind"] == "program" else f"{t.get('name')}'s"
        return (f"to {where} start: MoveJ to its first point ({step['speed']} %)" if k == "move_to"
                else f"{where} pole and hand: " + _step_words({x: v for x, v in step.items() if x != "_then"}))
    if k == "program":
        return f"Program {step['id']}" + (f" // {step['name']}" if step.get("name") else "")
    if k == "pole":
        return f"Pole {float(step['mm']):g} mm"
    if k == "hand":
        return "Hand " + ",".join(str(v) for v in step["values"])
    if k == "wait":
        return f"Wait {float(step['seconds']):g} s"
    if k == "clear":
        return "Clear the arm's 4105 warning, if it holds one, before the move"
    if k == "pole_hand":
        return (f"Pole {float(step['mm']):g} mm with the Hand " + ",".join("hold" if v is None else str(v) for v in step["values"])
                + " moving meanwhile")
    if k == "move_to":
        return f"MoveJ to a pose ({step['speed']} %)"
    if k == "meet":
        return "Meet the other arm"
    if k == "wait_for":
        tg = step.get("_target") or {}
        return f"Wait for the {str(tg.get('side') or 'other').upper()} arm's row {tg.get('row', '?')}"
    if k == "mechanism":
        from . import mechanism

        return mechanism.TASKS.get(step.get("task"), {}).get("title", "Lid or seat task")
    return f"{step['move'].upper()} {step['name']}"


def _times_now(app, units, run: dict, programs: dict, points: dict, overall: int) -> None:
    """Each step of the run's plan with its range of times on the robot (``_t``), from the arms as they stand now: the
    simulation's own timing as a rule (_ArmSim, schedule), with no 3D and no collision check (Real Machine Mode's
    watchdog reads them)."""
    from ..motion.limits import load_limits
    from .routes_robot import _payload

    home = dict(_payload(app)["home"])
    md_of = _md_reader(units.root)
    sims = {}
    for s in ARMS:
        start, _ = _start_state(app, s, home)
        sims[s] = _ArmSim(s, start, load_limits(s).rating_deg_s, overall, programs[s], points[s], md_of, [],
                          files_by_name(app, units, s))
    schedule(run, sims, overall)


def _watch(job, side: str, steps: list, say_stop, warn) -> tuple:
    """Each step of a runner call timed against its range (``_t``, the owner, 22 Oct): ``on_line`` sees the runner's
    lines; a step outside its range is said (``warn``); a Program whose lines run past STOP_PROGRAM_OVER of their time,
    or any other step more than STOP_STEP_MARGIN past the most it takes, is stopped at that moment (``say_stop`` with
    why: it stops both arms); a step other than a Program that ends more than STOP_STEP_MARGIN sooner than the least it
    takes is stopped as it ends. (A Program ending before STOP_PROGRAM_UNDER of its lines' time is the early-end check.)
    Returns (on_line, close)."""
    import threading

    cur, lock, dead = {}, threading.Lock(), threading.Event()

    def limit(st) -> float | None:
        t = st.get("_t") or {}
        r = t.get("move_times") if st["type"] == "global" else t.get("times")
        if st["type"] == "wait" and not t:          # a global point's wait split from it: its own seconds
            sec = float(st.get("seconds") or 0)
            r = tuple(sec + x for x in ROBOT_TIMES["wait_extra_s"])
        if not r:
            return None
        if st["type"] == "program":
            if not t.get("drawn") or not t.get("play_s"):
                return None
            tr = t.get("transit") or (0, 0, 0)
            lines = int(t.get("lines") or 0)
            lead = 0.05 + lines * ROBOT_TIMES["load_s_per_line"][2]
            tail = ROBOT_TIMES["program_extra_s"][2]
            return lead + float(tr[2]) + STOP_PROGRAM_OVER * float(t["play_s"]) + tail + 0.3
        return float(r[2]) * (1.0 + STOP_STEP_MARGIN) + 0.15

    def tick():
        while not dead.wait(0.1):
            with lock:
                c = dict(cur)
            if c and c.get("deadline") and time.monotonic() > c["deadline"]:
                st = c["step"]
                with lock:
                    cur.clear()
                say_stop(f"{side.upper()} row {st.get('_row')} ({_step_words(st)}) ran past "
                         + (f"{STOP_PROGRAM_OVER:.0%} of its lines' time (a controller's realtime push left on slows its programs "
                            "to some 113 %)" if st["type"] == "program"
                            else f"the most it takes ({float(((st.get('_t') or {}).get('move_times') if st['type'] == 'global' else (st.get('_t') or {}).get('times') or (0, 0, 0))[2]):.1f} s) by {STOP_STEP_MARGIN:.0%}")
                         + ": stopped, as the two arms' timings are no longer the ones checked")

    threading.Thread(target=tick, name=f"lite-s2-watch-{side}", daemon=True).start()

    def on_line(text: str) -> None:
        if text.startswith("WORKFLOW STEP START"):
            try:
                k = int(text.split()[3].split("/")[0])
                st = steps[k - 1]
            except (IndexError, ValueError):
                return
            lim = limit(st)
            with lock:
                cur.clear()
                cur.update(step=st, t0=time.monotonic(), deadline=(time.monotonic() + lim) if lim else None)
        elif text.startswith("WORKFLOW STEP COMPLETE"):
            with lock:
                c = dict(cur)
                cur.clear()
            if not c:
                return
            st, dur = c["step"], time.monotonic() - c["t0"]
            t = st.get("_t") or {}
            r = t.get("move_times") if st["type"] == "global" else t.get("times")
            if st["type"] == "wait" and not t:
                sec = float(st.get("seconds") or 0)
                r = tuple(sec + x for x in ROBOT_TIMES["wait_extra_s"])
            if not r:
                return
            lo, hi = float(r[0]), float(r[2])
            words = f"{side.upper()} row {st.get('_row')} ({_step_words(st)}) took {dur:.2f} s"
            if st["type"] != "program" and dur < lo * (1.0 - STOP_STEP_MARGIN) - 0.15:
                say_stop(f"{words}, less than the least it takes ({lo:.2f} s) by more than {STOP_STEP_MARGIN:.0%}: stopped, as the "
                         "two arms' timings are no longer the ones checked")
            elif not lo - 0.05 <= dur <= hi + 0.05:
                warn(f"{words}, outside its range on the robot ({lo:.2f}-{hi:.2f} s).")

    def close():
        dead.set()

    return on_line, close


def feed_wanted(app, body: dict) -> bool:
    """Whether a Real Machine run records the controllers' feed: on by default (the owner, 23 Oct: "for now we can put it
    on by default at 20ms, but we should have a way to disable it"), off when Lite was started with ``--feed off``
    (``app.feed_ms`` None) or when the page sends ``record_feed`` false (its checkbox unticked)."""
    return bool(body.get("record_feed", False)) and bool(getattr(app, "feed_ms", None))   # off unless ticked (26 Oct)


def run_real(app, units, mode: str, pick: dict | None, body: dict):
    """Real Machine Mode (RUN BOTH / RUN LEFT / RUN RIGHT, a single task, a dual-arm task): only in Real robot mode, on
    linked controllers; a run not completed in Simulation as it is now asks first (``unsimulated_ok``; the owner, 17 Oct:
    "a user can run on the hardware without having run in simulation first", with a confirmation that says so, before
    the real-robot one); every Program slot and global point it uses checked on its controller before anything moves;
    what the simulation found confirmed once (``warnings_seen``) with the real-robot confirmation, whose card asks only
    the two safety checks (the owner, 23 Oct; the findings are in the page's Simulation card). Then the plan's stages: a stage
    between meeting points starts both arms' parts at once (each arm's part through runner v14, a call per stretch
    between its Wait-for rows; at a Wait-for the arm holds until the other arm's runner has reported the row it names
    complete) and ends when both have (the Meet); a lid or seat task runs on both arms; every step is timed against its
    range (``_watch``); a failed part ends the run there and stops the other arm. STOP (any) interrupts the runner as
    V53's STOP does and latches the arms as Lite's STOP always does."""
    import contextlib
    import threading

    robot = app.robot
    steps = _numbered(load(units))
    run = plan_for(steps, mode, pick)
    mode = run["mode"]                                      # the plan's arms (a task's, a dual task's), never the request's
    sides = list(run["steps"])
    if app.ui["mode"] != "real":                    # the owner, 17 Oct: a run on the real hardware says the mode it found
        now = {"visual": "Visual sim", "hwsim": "Hardware sim"}.get(app.ui["mode"], app.ui["mode"])
        raise UserError(f"The robot is in {now}, not Real: Real Machine Mode runs the sequence on the physical arms.",
                        "Choose Real robot in the status pill at the top (it asks to confirm), or run it in Simulation Mode.", code="mode")
    for s in sides:
        if robot._real(s) is None or not robot.arm(s).connected:
            raise UserError(f"The {s.upper()} arm's controller is not connected: Real Machine Mode runs the sequence with the "
                            "control program's runner on the controllers.", "Connect the arm (Lite started with --robot realman).",
                            code="connect")
    robot.require_motion("both" if mode == "both" else mode)
    for s in sides:                     # the times checked are the robot's with its realtime push off (ROBOT_TIMES)
        on = getattr(robot.links.get(s), "push_on", None)
        if on:
            from .udp_push import held_text

            raise UserError(f"The {s.upper()} controller's realtime push is on (toward this computer's port {on.get('port')}"
                            + (f", which {held_text(on['holders'])} holds" if on.get("holders") else "") + "): with it on, its "
                            f"programs run some 13 % slower than their lines, past the {STOP_PROGRAM_OVER:.0%} at which an S2 run "
                            "stops. Nothing moved.", "Turn it off (stop the program that holds the port, or on the pendant: realtime "
                            "push), then connect the arm again.", code="push")
    overall = int(robot.speed_pct)
    programs = {s: programs_known(app, units, s) for s in ARMS}
    points = {s: points_known(app, s) for s in ARMS}
    run = plan_for(steps, mode, pick, own=own_for(app, units, programs, run["steps"]))   # each Program's own pole and hand
    ident = identity_of(run, programs, points, overall)
    sim = load_sim(units)
    same = bool(sim and sim.get("hash") == digest(ident))
    simulated = same and bool(sim.get("completed"))
    if not simulated and not body.get("unsimulated_ok"):
        why = _changed(sim, ident) if not same else "its simulation was not played to its end"
        raise UserError(f"This run has not been run in Simulation Mode as it is now: {why}.",
                        "Run it on the real robot without simulating it first, or simulate it first: Simulation Mode "
                        "plays it in 3D and checks the two arms' collisions.", code="unsimulated")
    missing = []
    for s in sides:                                        # every slot and point, before anything moves
        held = {int(x["id"]) for x in robot.programs(s) if x.get("name")}
        want = sorted({int(st["id"]) for st in run["steps"][s] if st["type"] == "program"})
        missing += [f"{s.upper()} program {p} (its slot is empty)" for p in want if p not in held]
        names = {st["name"] for st in run["steps"][s] if st["type"] == "global"}
        if names:
            have = {p["name"] for p in (robot.global_points(s) or [])}
            missing += [f"{s.upper()} global point {n}" for n in sorted(names - have)]
    if missing:
        raise UserError("Not on the controllers: " + "; ".join(missing) + ". Nothing moved.",
                        "Upload the programs or save the points, simulate again, then run it.", code="missing")
    if any(st["kind"] == "mechanism" for st in run["plan"]):     # its checks need the wrist camera (the owner, 23 Oct)
        from . import mechanism

        mechanism.camera_required(robot)
        mechanism.joints_cool(robot)                          # and no joint hot (the owner, 26 Oct)
    # a Program at another pole or hand than its own is never run (the owner, 21 Oct): not a question, a stop. Lite sets
    # each Program's own before it (own_stages, 24 Oct), so only a simulation made before that can still hold one
    stops = [f["text"] for f in sim["findings"] if f["level"] == "stop"] if simulated else []
    if stops:
        raise UserError("Not run: " + " ".join(stops), "Each Program runs with the pole and hand it was taught with: "
                        "simulate again, then run it.", code="stage")
    warns = [f["text"] for f in sim["findings"] if f["level"] == "warn"] if simulated else []
    col = sim["collisions"] if simulated else {"ok": True}
    if not col["ok"]:
        first = col["first"]
        warns.insert(0, f"The simulation found {len(col['events']) + col['more']} collision event(s), the first at "
                     f"{first['first_t']:.2f} s: " + "; ".join(f"{p['link']} with {p['object']}" for p in first["pairs"][:3]) + ".")
    if warns and body.get("warnings_seen") != sim["hash"]:          # seen for this simulation, not another
        raise UserError("Before it runs on the robot: " + " ".join(warns), "Run it anyway, or correct the sequence.",
                        code="warnings")
    if not body.get("confirmed"):
        raise UserError("Real Machine Mode on the real robot needs your confirmation.", code="confirm")
    restore = {s: robot.found_speed(s) for s in sides}
    # the controllers' feed recorded (feed_wanted): a file each arm's part of a stage, the day's canfd folder
    want_feed = feed_wanted(app, body)
    feeds = None
    if want_feed:
        root = getattr(getattr(robot, "log", None), "root", None)
        feeds = Path(root) / time.strftime("%Y-%m-%d") / "canfd" if root else None
    used = {s: {int(st["id"]) for st in run["steps"][s] if st["type"] == "program"} for s in sides}
    lines_of = {s: {pid: int(programs[s][pid]["lines"]) for pid in used[s]
                    if (programs[s].get(pid) or {}).get("sent") and (programs[s][pid].get("lines") or 0) > 1} for s in sides}
    longest = max([n for s in sides for n in lines_of[s].values()] or [0])
    unknown = any(pid not in lines_of[s] for s in sides for pid in used[s])
    # the runner's own limit on a Program (it slow-stops one still running then): this run's longest, half as long again;
    # an hour when a Program's length is not known (one Lite did not send): Lite's own watchdog times the known ones
    timeout = max(300.0, (longest - 1) * CYCLE_S * 1.5 + 60.0, 3600.0 if unknown else 0.0)
    try:
        _times_now(app, units, run, programs, points, overall)
    except Exception as exc:                                        # noqa: BLE001 - untimed steps are said, not run blind
        robot_log = getattr(robot, "log", None)
        if robot_log is not None:
            robot_log.activity(f"S2: the steps' ranges could not be worked out ({exc}); each step runs untimed")

    def work(job):
        live = {s: "READY" for s in ARMS}
        rows = {s: None for s in ARMS}
        job.set(live=dict(live), status=f"PAGE 02 // {len(run['plan'])} stage(s)", stage=None, question=None,
                counts=run["counts"], rows=dict(rows), mode=run["mode"])
        if not simulated:
            job.log("Run on the robot without a simulation of it, as confirmed.", "warn")
        for t in run.get("said") or []:
            job.log(t)
        if want_feed:
            from .robot_real import S2_FEED_CYCLE

            job.log(f"Recording the controllers' feed every {S2_FEED_CYCLE * 5} ms (a diagnostic) to {feeds}: the Programs play "
                    "some 3 % slower meanwhile (the 23 Oct pace test: 3.4 %), still held to the same 105 % stop."
                    if feeds else "The controllers' feed is not recorded: Lite has no log folder.", "info" if feeds else "warn")
        reals = {s: robot._real(s) for s in sides}
        originals = {}
        feed_n = {s: 0 for s in ARMS}

        def say(status=None, **arms):
            live.update({k: v for k, v in arms.items()})
            job.set(live=dict(live), **({"status": status} if status else {}))

        stops = {}                                  # each running part's own stop (its runner, V53's way)
        stopped_why = {}                            # why Lite stopped a part, said with the failure
        cond = threading.Condition()
        progress = {s: set() for s in ARMS}         # the rows each arm has finished in the stage running
        ended = {s: False for s in ARMS}            # each arm's part of the stage running is over (done or not)

        def finished(side, row) -> None:
            with cond:
                progress[side].add(row)
                cond.notify_all()

        def stop_part(side, why) -> None:
            stopped_why.setdefault(side, why)
            fn = stops.get(side)
            if fn is not None:
                fn()

        def call(side, steps, label) -> dict:
            """One runner call: a stretch of an arm's rows (no Wait-for among them)."""
            current = {"slot": None}
            last_of_row = {k: (k + 1 >= len(steps) or steps[k + 1].get("_row") != st.get("_row")) for k, st in enumerate(steps)}

            def stop_hook(fn):
                stops[side] = fn
                job.on_cancel(fn)

            emulated = bool(getattr(robot.links.get(side), "emulated", False))      # ALIX's emulator keeps its own pace
            watch_line, watch_close = _watch(job, side, steps,
                                             (lambda why: job.log(f"{why} (an emulated arm: said, not stopped)", "warn")) if emulated
                                             else (lambda why: stop_part(side, why)),
                                             lambda text: job.log(text, "warn"))

            def on_line(line):                      # in the runner's own thread, as it prints
                text = line.strip()
                watch_line(text)
                if text.startswith("WORKFLOW STEP START"):
                    try:
                        k = int(text.split()[3].split("/")[0])
                        rows[side] = steps[k - 1].get("_row")
                        job.set(rows=dict(rows))
                        say(**{side: f"{label} // {_step_words(steps[k - 1])} // RUNNING"})
                    except (IndexError, ValueError):
                        pass
                    # The E-stop between two steps (the link reads nothing while the runner holds the arm): a
                    # press latches and stops both arms (their stop hooks); the runner stops at once.
                    robot.estop_check(side, on_press=lambda: stopped_why.__setitem__(side, f"the {side.upper()} E-stop was pressed"))
                elif text.startswith("WORKFLOW STEP COMPLETE"):
                    try:
                        k = int(text.split()[3].split("/")[0])
                        if last_of_row.get(k - 1):  # its row is finished (a row's Wait split from it)
                            finished(side, steps[k - 1].get("_row"))
                    except (IndexError, ValueError):
                        pass
                m = _PROGRAM_RE.match(text)
                if m:
                    current["slot"] = int(m.group(1))
                m = _COMPLETE_RE.match(text)
                if m and current["slot"] in lines_of[side]:
                    from .robot_standin import ended_early

                    short = ended_early(float(m.group(1)), lines_of[side][current["slot"]], robot.emulator_scale(side),
                                        share=STOP_PROGRAM_UNDER)
                    if short:                       # stopped on the controller (an E-stop gives no error code)
                        stop_part(side, f"program {current['slot']} ended {short}: it was stopped on the controller")
                if text.startswith(("WORKFLOW STEP START", "WORKFLOW COMPLETE", "**", "!!")):
                    job.log(f"{side.upper()}: {text}", "error" if text.startswith("**") else "info")

            try:
                out = reals[side].run_workflow(steps, overall, restore[side], timeout, on_line, stop_hook)
            finally:
                stops.pop(side, None)
                watch_close()
            if stopped_why.get(side):
                out = dict(out, ok=False, why=stopped_why[side])
            if out["ok"]:
                for st in steps:
                    finished(side, st.get("_row"))
            return out

        def clear(side, row) -> dict:
            """Lite's clear step (WARN_ONLY): the arm's errors read before a move that waits for its arrival; a 4105
            alone cleared; anything else latched ends the part there."""
            if stopped_why.get(side) or job.cancelled:
                return dict(ok=False, rc=1, why=stopped_why.get(side) or "stopped")
            try:
                got = reals[side].clear_warning(WARN_ONLY)
            except UserError as exc:
                return dict(ok=False, rc=1, why=str(exc))
            if got["other"]:
                return dict(ok=False, rc=1, why=f"before row {row}, the {side.upper()} controller reports {got['other']}: not a "
                            "warning, so Lite does not clear it (clear it on the pendant or in Robot & mode, then run again)")
            if got["cleared"]:
                job.log(f"{side.upper()} row {row}: cleared {', '.join(got['cleared'])} (a joint over its speed limit, a warning "
                        "a Program left) before the move: the controller refuses a move's arrival while it is latched.")
            elif not got["readable"]:
                job.log(f"{side.upper()} row {row}: the arm's errors could not be read before the move.", "warn")
            return dict(ok=True, rc=0, why=None)

        def pair(side, x, label) -> dict:
            """Lite's Pole with the Hand after it (pole_with_hand): the hand moving while the pole travels, timed against
            its range as runner v14's steps are (_watch); both its rows finished after it."""
            from .jobs import Cancelled

            if stopped_why.get(side) or job.cancelled:
                return dict(ok=False, rc=1, why=stopped_why.get(side) or "stopped")
            rows[side] = x.get("_row")
            job.set(rows=dict(rows))
            say(**{side: f"{label} // {_step_words(x)} // RUNNING"})
            job.log(f"{side.upper()}: {_rows_words(x)}: {_step_words(x)}.")
            robot.estop_check(side, on_press=lambda: stopped_why.__setitem__(side, f"the {side.upper()} E-stop was pressed"))
            if stopped_why.get(side):
                return dict(ok=False, rc=1, why=stopped_why[side])

            def stop_hook(fn):
                stops[side] = fn
                job.on_cancel(fn)

            t0 = time.monotonic()
            try:
                reals[side].pole_and_hand(float(x["mm"]), list(x["values"]), stop_hook=stop_hook)
            except Cancelled:
                return dict(ok=False, rc=1, why=stopped_why.get(side) or "stopped")
            except UserError as exc:
                return dict(ok=False, rc=1, why=str(exc))
            finally:
                stops.pop(side, None)
            dur = time.monotonic() - t0
            r = (x.get("_t") or {}).get("own_times")
            emulated = bool(getattr(robot.links.get(side), "emulated", False))      # ALIX's emulator keeps its own pace
            if r:
                lo, hi = float(r[0]), float(r[2])
                words = f"{side.upper()} {_rows_words(x)} ({_step_words(x)}) took {dur:.2f} s"
                if (dur > hi * (1.0 + STOP_STEP_MARGIN) + 0.15 or dur < lo * (1.0 - STOP_STEP_MARGIN) - 0.15) and emulated:
                    job.log(f"{words}, outside its range on the robot ({lo:.2f}-{hi:.2f} s) (an emulated arm: said, not stopped)", "warn")
                elif dur > hi * (1.0 + STOP_STEP_MARGIN) + 0.15 or dur < lo * (1.0 - STOP_STEP_MARGIN) - 0.15:
                    return dict(ok=False, rc=1, why=f"{words}, outside its range on the robot ({lo:.2f}-{hi:.2f} s) by more than "
                                f"{STOP_STEP_MARGIN:.0%}: stopped, as the two arms' timings are no longer the ones checked")
                if not lo - 0.05 <= dur <= hi + 0.05:
                    job.log(f"{words}, outside its range on the robot ({lo:.2f}-{hi:.2f} s).", "warn")
            for n in _rows_done(x):
                finished(side, n)
            return dict(ok=True, rc=0, why=None)

        def part(side, item, label):
            """An arm's part of a stage: its runner calls, and between them its Wait-for rows and Lite's own steps."""
            out = dict(ok=True, rc=0, why=None)
            try:
                for what, x in _segments(split_waits(item["steps"])):
                    if what == "steps":
                        out = call(side, x, label)
                        if not out["ok"]:
                            break
                        continue
                    if what == "clear":
                        out = clear(side, x.get("_row"))
                        if not out["ok"]:
                            break
                        continue
                    if what == "pair":
                        out = pair(side, x, label)
                        if not out["ok"]:
                            break
                        continue
                    tg = x.get("_target") or {}
                    other = tg.get("side")
                    if not tg.get("done") and other in item.get("_with", ()):
                        say(**{side: f"{label} // WAITING FOR {str(other).upper()} ROW {tg.get('row')}"})
                        rows[side] = x.get("_row")
                        job.set(rows=dict(rows))
                        job.log(f"{side.upper()} row {x.get('_row')}: waits for {str(other).upper()} row {tg.get('row')}.")
                        with cond:
                            while tg.get("row") not in progress[other]:
                                if ended[other]:
                                    out = dict(ok=False, rc=1, why=f"the {str(other).upper()} arm's part ended before its row "
                                               f"{tg.get('row')}, which row {x.get('_row')} waits for")
                                    break
                                if job.cancelled or stopped_why.get(side):
                                    out = dict(ok=False, rc=1, why=stopped_why.get(side) or "stopped")
                                    break
                                cond.wait(0.2)
                        if not out["ok"]:
                            break
                    finished(side, x.get("_row"))
            finally:
                with cond:
                    ended[side] = True
                    cond.notify_all()
            say(**{side: f"{label} // COMPLETE" if out["ok"] else f"{label} // FAILED rc={out['rc']}"})
            return out

        def together(stage, label, status):
            mine = [s for s in ARMS if stage.get(s)]
            say(status, **{s: f"{label} // RUNNING" for s in mine})
            barrier, results = threading.Barrier(len(mine)), {}
            with cond:
                for s in ARMS:
                    progress[s].clear()
                    ended[s] = s not in mine
            for s in mine:
                stage[s]["_with"] = tuple(mine)

            def go(side):
                # each arm's controller taken before the two meet, so neither waits for its own after the
                # other has started (a command still holding it: neither starts)
                link = getattr(reals[side], "link", None)
                try:
                    with link.reserved() if link is not None else contextlib.nullcontext():
                        feed_on(side, label)                # the stage's recording, on before the two arms meet
                        try:
                            try:
                                barrier.wait(timeout=10)
                            except threading.BrokenBarrierError:
                                results[side] = dict(ok=False, rc=1, why="the other arm did not start")
                                return
                            try:
                                results[side] = part(side, stage[side], label)
                            except BaseException as exc:                   # noqa: BLE001 - said below
                                results[side] = dict(ok=False, rc=1, why=str(exc) or type(exc).__name__)
                        finally:
                            feed_off(side)
                except UserError as exc:                                   # its controller is busy: neither starts
                    barrier.abort()
                    results[side] = dict(ok=False, rc=1, why=str(exc))
                    return
                if not results[side].get("ok"):     # the owner (16 Oct): one arm failing stops the other as well
                    other = flow.OTHER[side]
                    if other in mine:
                        stop_part(other, f"stopped because the {side.upper()} arm's part failed")

            threads = [threading.Thread(target=go, args=(s,), name=f"lite-s2-{s}", daemon=True) for s in mine]
            for t in threads:
                t.start()
            for t in threads:                       # both parts end before anything else: STOP interrupts both
                while t.is_alive():
                    t.join(0.1)
            job.check()
            bad = [s for s in mine if not results.get(s, {}).get("ok")]
            if bad:
                # Each arm's errors as the controller holds them now, its joints' own flags too (under-voltage, a brake),
                # which neither runner v14 nor the status rows read: the lab, 22 Oct, four times the RIGHT arm's Program
                # 82 ended on its controller with no system code as the LEFT arm started its Program 86.
                hw = getattr(robot, "log", None)
                for s in mine:
                    try:
                        es = reals[s].errors()
                        text = es["text"]
                    except Exception as exc:                               # noqa: BLE001 - said, never raised
                        text, es = f"not read ({exc})", {"clean": False}
                    job.log(f"{s.upper()} controller's errors after the stop: {text}.", "info" if es.get("clean") else "warn")
                    if hw is not None:
                        hw.hardware(s, "errors after the stop", text=text)
                raise UserError(f"{label} STOPPED // " + " // ".join(f"{s.upper()} rc={results.get(s, {}).get('rc')}" for s in mine)
                                + ": " + "; ".join(f"{s.upper()}: {results.get(s, {}).get('why') or 'see the hardware log'}" for s in bad),
                                "Check the arms and their controllers' errors before anything else runs.")

        def feed_on(side, label) -> None:
            """The stage's recording of the arm's feed (feed_wanted): on once for the arm's whole part, its runner calls and
            Lite's own steps between them (robot_real.keep_s2_feed), a file a stage."""
            if feeds is None:
                return
            feed_n[side] += 1
            slug = "".join(ch if ch.isalnum() else "_" for ch in str(label))[:40]
            cap = feeds / f"{time.strftime('%Y%m%d_%H%M%S')}_{side.upper()}_s2_{feed_n[side]:02d}_{slug}.frames.jsonl"
            try:
                kept = reals[side].keep_s2_feed(cap)
            except Exception as exc:                                       # noqa: BLE001 - a diagnostic: the run goes on
                kept = None
                job.log(f"{side.upper()}: its feed could not be turned on ({exc}).", "warn")
            if kept is None:
                job.log(f"{side.upper()}: its feed is not recorded for {label} (see the hardware log).", "warn")

        def feed_off(side) -> None:
            fed = reals[side].drop_s2_feed() if feeds is not None else None
            if fed:
                job.log(f"{side.upper()}: its feed, {fed['frames']} frame(s) every {fed['cycle_ms']} ms: {Path(fed['file']).name}",
                        "info" if fed["frames"] else "warn")

        def to_next(st, pid):
            """After a run of lid and seat tasks: each arm to its next task's start (next_after_tasks), both arms at once:
            its pole and hand together (pole_and_hand), then, to a Program's first point, its errors read (a 4105 the
            plan's streams left cleared, as before every move that waits for its arrival) and its MoveJ at the speed
            slider's speed (the global speed at 100 % for it, put back after). The next stage then starts both arms at
            their Programs' first points."""
            then = st.get("then") or {}
            say(f"LID / SEAT // {then_title(st)}", **{x: f"{pid} // TO THE NEXT START" for x in then})
            done, threads = {}, []

            def one(x):
                t, row = then[x], (st.get("rows") or {}).get(x)
                try:
                    if t.get("pole_mm") is not None or t.get("hand") is not None:
                        pole, hand = robot._measured(x)
                        mm = float(t["pole_mm"]) if t.get("pole_mm") is not None else pole
                        values = list(t["hand"]) if t.get("hand") is not None else hand
                        if mm is None or values is None:
                            raise UserError(f"its pole or hand is not read, so it is not moved to {then_title(st)}")
                        reals[x].pole_and_hand(float(mm), list(values))
                    if t.get("arm_deg"):
                        got = clear(x, row)
                        if not got["ok"]:
                            raise UserError(got["why"])
                        reals[x].move_to([round(float(v) * 1000) for v in t["arm_deg"]], speed=int(robot.speed_pct),
                                         restore_pct=robot.found_speed(x))
                    done[x] = None
                    job.log(f"{x.upper()}: at the start of " + (f"Program {t['id']}" if t["kind"] == "program" else t.get("name") or "its next row")
                            + (" (its own pole and hand" + (", its first point)" if t.get("arm_deg") else ")") if t.get("own") or t.get("arm_deg")
                               else "") + ".")
                except Exception as exc:                                   # noqa: BLE001 - said below
                    done[x] = f"{x.upper()}: {exc}"

            for x in sorted(then):
                threads.append(threading.Thread(target=one, args=(x,), name=f"lite-s2-next-{x}", daemon=True))
                threads[-1].start()
            for t in threads:
                while t.is_alive():
                    t.join(0.1)
            job.check()
            bad = [w for w in done.values() if w]
            if bad:
                raise UserError(f"{st['title']}: the arms did not reach their next task's start: " + "; ".join(bad),
                                "Check the arms, the poles and the hands, then run the sequence again.")

        t0 = time.monotonic()
        try:
            for s in sides:
                originals[s] = reals[s].set_run_mode("REAL")
            for i, st in enumerate(run["plan"]):
                job.check()
                job.set(stage=i)
                job.progress(f"Stage {i + 1} of {len(run['plan'])}", i, len(run["plan"]))
                kind, pid = st["kind"], st.get("program_id")
                if kind == "mechanism":                     # a lid or seat task: ALIX's plan on both arms, the camera's checks
                    from . import mechanism as mech

                    for x in st.get("clear") or []:         # its moves wait for their arrival (clear_before_moves)
                        got = clear(x, (st.get("rows") or {}).get(x))
                        if not got["ok"]:
                            raise UserError(f"{st['title']} STOPPED // {x.upper()}: {got['why']}",
                                            "Check the arms and their controllers' errors before anything else runs.")
                    say(f"LID / SEAT // {st['title']} // ALIX's plan, the wrist camera's checks",
                        **{x: f"{pid} // RUNNING" for x in ARMS})
                    kit = mech.kit(robot)
                    hw = getattr(robot, "log", None)
                    from . import placement
                    from .routes_robot import _payload

                    # the collision check's world for the approach's two MoveJs at once: the plans' own placement (the UGV at
                    # STANDOFF_MM, no offset), whether or not this run was simulated (the evening of 26 Oct: five runs, none
                    # simulated first, so the MoveJs went one after the other)
                    plans_at = {"standoff_mm": float(mech.STANDOFF_MM), "front_mm": 0.0, "rear_mm": 0.0, "left_mm": 0.0, "right_mm": 0.0}
                    try:
                        room = dict(home=dict(_payload(app)["home"]), scene=placement.scene(units.fixture, placement.load(units)["room"], plans_at),
                                    fid=units.fixture, cubicle_mm=int(app.ui.get("cubicle_mm", 1400)))
                    except Exception as exc:                                        # noqa: BLE001 - then one after the other, said
                        room = None
                        say(None, **{x: f"{pid} // the MoveJs' collision check has no world ({type(exc).__name__}): one after the other" for x in ARMS})
                    ex = mech.RealExec(job, robot, {x: reals.get(x) or robot._real(x) for x in ARMS}, kit,
                                       (lambda text: hw.hardware(None, "mechanism", text=text)) if hw is not None else None,
                                       live=lambda side, text, pid=pid: say(None, **{side: f"{pid} // {text}"}), room=room)
                    mech.camera_required(robot, st["title"])             # checked before the run; unplugged since?
                    mech.joints_cool(robot, st["title"])
                    with kit.camera.using():
                        out = mech.Runner(ex, ex.say, leave=set(st.get("then") or {}), hand_set=hand_set(st), start_checks=False).run(st["task"])
                    did = "; ".join(f"{p['title']} {p['outcome']}" + (f" after {p['relifts']} re-lift(s)" if p["relifts"] else "")
                                    for p in out["plans"])
                    job.log(f"{st['title']}: {did}.")
                    if st.get("then"):                      # each arm to its next task's start (next_after_tasks)
                        to_next(st, pid)
                    say(f"LID / SEAT // {st['title']} // DONE", **{x: f"{pid} // COMPLETE" for x in ARMS})
                elif kind == "parallel":
                    arms_here = [s for s in ARMS if st.get(s)]
                    together(st, str(pid or "PART"), f"PAGE 02 // {pid} // " + (" + ".join(s.upper() for s in arms_here)))
                else:                                                     # one arm's whole sequence, or a single task
                    item = st[sides[0]]
                    label = "SEQUENCE" if kind == "single" and pid is None else f"Program {pid}"
                    together({sides[0]: item}, label, f"PAGE 02 // {sides[0].upper()} {label.lower()}")
            elapsed = time.monotonic() - t0
            say(f"PAGE 02 SEQUENCE COMPLETE // {elapsed:.1f} s", **{s: "SEQUENCE COMPLETE" for s in sides})
            job.log(f"Sequence complete: {len(run['plan'])} stage(s) in {elapsed:.1f} s.")
            return dict(mode=run["mode"], stages=len(run["plan"]), seconds=round(elapsed, 1))
        finally:
            for s, orig in originals.items():
                reals[s].restore_run_mode(orig)
            for s in sides:                         # the runner gives the speed back in its finally; again here,
                reals[s].restore_speed(restore[s])  # in case a stop landed before its try (never raises)

    title = "S2 Real Machine: " + ("both arms" if mode == "both" else f"{mode.upper()} sequence")
    return robot._start("run", title, work, "both" if mode == "both" else mode, "real")
