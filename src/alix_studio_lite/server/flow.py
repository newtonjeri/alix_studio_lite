"""S2's plan of both arms (the owner, 22 Oct): each arm's rows run top to bottom, both arms starting together, and an
arm waits only where the sequence says so:

- a **Meet** row: the arm stops there until the other arm reaches its own Meet; then both go on together. The Meets pair
  up in order (each arm's first with the other's first, and so on);
- a **Wait-for** row: the arm stops there until the other arm has finished the row it names (``target``: that row's
  ``uid``, which follows the row when rows move; or, from a V53 file, ``program``: the other arm's first row of that
  Program); the other arm goes on;
- a lid or seat task: it uses both arms, so both finish what is before it (a meeting point too);
- an INSPECT (the user, 5 Nov): the right arm's MoveJ to an inspection pose and the wrist camera's look at the commode,
  the left arm still: a meeting point too, a stage of its own (workflow.run_real's inspect);
- a Program marked to **start together** (``together``; the user, 5 Nov: "tasks that are required to start together,
  forced to start together, no latency"): it pairs with the other arm's marked Program between the same two meeting
  points (each arm's first with the other's first, and so on); both arms finish what is before their Programs, then
  both Programs are started at the same moment (workflow.run_real's start gate).

V53's Dual Rule (TOGETHER / RUN FIRST / WAIT OTHER) and its SAFE TIMELINE planner are gone (the owner, 22 Oct: "do away
with it"), and with them their refusals: one Program ID per arm, the same ID for a handoff, a Program on each arm, the
steps after an arm's last Program waiting for the other arm. A V53 file is loaded with its RUN FIRST / WAIT OTHER pairs
as Meets and a Wait-for, which run them as V53 did (``from_v53``).

Refused before anything moves: a Meet without its partner, a Wait-for naming a row that is not there, a Program that
starts together with no partner between the same meeting points, and two arms each waiting for the other (it would
never end)."""
from __future__ import annotations

import secrets

ARMS = ("left", "right")
GATES = ("meet", "wait_for")
OTHER = {"left": "right", "right": "left"}


class PlanError(ValueError):
    """A sequence that cannot run as it is: said before anything moves."""


def new_uid() -> str:
    return secrets.token_hex(4)


def with_uids(steps: list) -> list:
    """Each row with its ``uid`` (a Wait-for names a row by it, so that it follows the row when rows move); a row
    without one gets a new one, and a second row with the same uid (a copy) gets its own."""
    seen, out = set(), []
    for st in steps:
        uid = str(st.get("uid") or "")
        if not uid or uid in seen:
            uid = new_uid()
        seen.add(uid)
        out.append(dict(st, uid=uid))
    return out


# ── a V53 file's Dual Rule, as rows

def from_v53(steps: list) -> tuple[list, list]:
    """One arm's V53 steps without the Dual Rule. A RUN FIRST / WAIT OTHER pair of Program P becomes, on both arms, a
    Meet before P's own setup rows (V53 started the pair once both arms had finished what came before it) and a Meet
    after P (both went on after the second arm's P); the WAIT OTHER arm also waits for the other arm's P (a Wait-for
    after the first Meet). TOGETHER needs nothing: both arms go on. Each arm's file changes on its own, the same way, so
    the Meets pair up as V53 paired the handoffs. Returns the steps and what changed, in words."""
    out, said, chunk = [], [], []                  # chunk: P's setup rows, since the Program before it (V53's chunk)
    for raw in steps:
        if not isinstance(raw, dict):
            chunk.append(raw)
            continue
        rule = str(raw.get("dual_rule") or "TOGETHER").strip().upper()
        st = {k: v for k, v in raw.items() if k != "dual_rule"}
        chunk.append(st)
        if st.get("type") != "program":
            continue
        if rule in ("RUN FIRST", "WAIT OTHER"):
            pid = int(st["id"])
            note = f"V53: Program {pid} {rule} (one arm at a time)"
            out.append({"type": "meet", "note": note})
            if rule == "WAIT OTHER":
                out.append({"type": "wait_for", "target": {"program": pid}, "note": note})
            out.extend(chunk)
            out.append({"type": "meet", "note": note})
            said.append(f"Program {pid} ({rule}): a Meet before it and after it"
                        + (f", and a Wait-for the other arm's Program {pid}" if rule == "WAIT OTHER" else ""))
        else:
            out.extend(chunk)
        chunk = []
    out.extend(chunk)
    return out, said


# ── the plan

def _sync(steps: list) -> list[tuple[int, str]]:
    """The arm's meeting points in order: (index, "meet", "inspect" or the lid or seat task)."""
    return [(i, st["type"] if st["type"] in ("meet", "inspect") else st["task"]) for i, st in enumerate(steps)
            if st["type"] in ("meet", "mechanism", "inspect")]


def _words(kind: str) -> str:
    if kind == "meet":
        return "a Meet"
    if kind == "inspect":
        return "an INSPECT"
    from . import mechanism

    return mechanism.TASKS.get(kind, {}).get("title", kind)


def _row(steps: list, i: int):
    return steps[i].get("_row", i + 1)


def _resolve(steps: dict) -> dict:
    """{(side, index of its Wait-for): (other side, index of the row it waits for)}; a target not there is refused."""
    out = {}
    for side in ARMS:
        other, theirs = OTHER[side], steps.get(OTHER[side]) or []
        for i, st in enumerate(steps.get(side) or []):
            if st["type"] != "wait_for":
                continue
            tg, row = st.get("target") or {}, _row(steps[side], i)
            if not theirs:
                raise PlanError(f"{side.upper()} row {row} waits for the {other.upper()} arm, which has no rows in this run.")
            if tg.get("uid"):
                j = next((n for n, x in enumerate(theirs) if x.get("uid") == tg["uid"]), None)
                if j is None:
                    raise PlanError(f"{side.upper()} row {row} waits for a {other.upper()} row that is no longer there "
                                    "(it was removed): choose the row it waits for again.")
            elif tg.get("program") is not None:
                j = next((n for n, x in enumerate(theirs) if x["type"] == "program" and int(x["id"]) == int(tg["program"])), None)
                if j is None:
                    raise PlanError(f"{side.upper()} row {row} waits for the {other.upper()} arm's Program {tg['program']}, "
                                    f"which is not in the {other.upper()} list.")
            else:
                raise PlanError(f"{side.upper()} row {row}: a Wait-for names the {other.upper()} row it waits for.")
            out[(side, i)] = (other, j)
    return out


def together_marks(steps: list) -> list[int]:
    """The indexes of an arm's Programs marked to start together with the other arm's (``together``)."""
    return [i for i, st in enumerate(steps) if st["type"] == "program" and st.get("together")]


def _walk(steps: dict, waits: dict, sync: dict) -> None:
    """Both arms walked with no time at all: each goes on while it can; at a meeting point (a Program that starts
    together too) it waits for the other arm to reach its own (then both go on), at a Wait-for for the other arm to
    have finished the row it names. If neither can go on before both are at their end, the sequence would never end:
    refused, saying where each arm waits."""
    pos = {s: 0 for s in ARMS}
    n = {s: len(steps[s]) for s in ARMS}
    at = {s: sorted([i for i, _ in sync[s]] + together_marks(steps[s])) for s in ARMS}
    gate = {s: set(at[s]) for s in ARMS}
    while pos["left"] < n["left"] or pos["right"] < n["right"]:
        moved = False
        for side in ARMS:
            other = OTHER[side]
            while pos[side] < n[side]:
                st = steps[side][pos[side]]
                if pos[side] in gate[side]:
                    j = at[other][at[side].index(pos[side])]           # the other arm's meeting point of the same number
                    if pos[other] != j:
                        break
                    pos[side] += 1
                    pos[other] += 1
                    moved = True
                    continue
                if st["type"] == "wait_for" and pos[waits[(side, pos[side])][0]] <= waits[(side, pos[side])][1]:
                    break                                               # the row it waits for is not finished yet
                pos[side] += 1
                moved = True
        if not moved:
            def where(side):
                if pos[side] >= n[side]:
                    return f"{side.upper()} at its end"
                st, row = steps[side][pos[side]], _row(steps[side], pos[side])
                if st["type"] == "wait_for":
                    other, j = waits[(side, pos[side])]
                    return f"{side.upper()} row {row} waits for {other.upper()} row {_row(steps[other], j)}"
                if st["type"] == "program":
                    return f"{side.upper()} row {row} waits to start Program {st['id']} together with the {OTHER[side].upper()} arm"
                return f"{side.upper()} row {row} waits at {_words(st['type'] if st['type'] in ('meet', 'inspect') else st['task'])}"
            raise PlanError("The arms would wait for each other for ever: " + "; ".join(where(s) for s in ARMS) + ".")


def _together_pairs(steps: dict, sync: dict) -> list[tuple[int, int]]:
    """The Programs that start together, paired (LEFT index, RIGHT index): each arm's k-th marked Program with the
    other's k-th, both between the same two meeting points; a mark without its partner there is refused."""
    marks = {s: together_marks(steps[s]) for s in ARMS}

    def part_of(side, i):                           # the meeting points before it: which part of the plan it is in
        return sum(1 for j, _ in sync[side] if j < i)

    def rows(side, idx):
        return ", ".join(str(_row(steps[side], i)) for i in idx)

    per = {s: {} for s in ARMS}
    for s in ARMS:
        for i in marks[s]:
            per[s].setdefault(part_of(s, i), []).append(i)
    for p in sorted(set(per["left"]) | set(per["right"])):
        mine = {s: per[s].get(p, []) for s in ARMS}
        if len(mine["left"]) != len(mine["right"]):
            one = "left" if len(mine["left"]) > len(mine["right"]) else "right"
            other = OTHER[one]
            raise PlanError(f"{one.upper()} row {rows(one, mine[one])} start{'s' if len(mine[one]) == 1 else ''} together with the "
                            f"{other.upper()} arm, but the {other.upper()} arm has "
                            + (f"only row {rows(other, mine[other])} marked" if mine[other] else "no Program marked to start with it")
                            + " between the same Meets and lid or seat tasks: mark its partner there, or unmark it.")
    pairs = list(zip(marks["left"], marks["right"]))
    for li, ri in pairs:
        if part_of("left", li) != part_of("right", ri):            # never here after the counts per part
            raise PlanError(f"LEFT row {_row(steps['left'], li)} and RIGHT row {_row(steps['right'], ri)} start together, but a "
                            "Meet or a lid or seat task lies between them.")
    return pairs


def build(left: list, right: list) -> list[dict]:
    """Both arms' plan: their rows between meeting points as one stage each (``parallel``: both arms' parts start
    together, each goes through its own rows, the stage ends when both have), and each lid or seat task as a stage of
    its own (``mechanism``); the Meet rows are the boundaries between stages. A Wait-for stays among its arm's rows,
    with ``_target``: the other arm, the row it waits for, and ``done`` when that row is finished before the stage
    starts (a row of an earlier stage, or a meeting point). A Program that starts together stays among its arm's rows,
    with ``_together``: its pair's number, the other arm, its partner's row and Program."""
    from . import mechanism

    steps = {"left": [dict(x) for x in left], "right": [dict(x) for x in right]}
    sync = {s: _sync(steps[s]) for s in ARMS}
    if [k for _, k in sync["left"]] != [k for _, k in sync["right"]]:
        nmeet = {s: sum(1 for _, k in sync[s] if k == "meet") for s in ARMS}
        if nmeet["left"] != nmeet["right"]:
            raise PlanError(f"Each Meet needs its partner on the other arm: the LEFT list has {nmeet['left']} Meet row(s), "
                            f"the RIGHT list {nmeet['right']}.")
        words = {s: ", ".join(_words(k) for _, k in sync[s]) or "none" for s in ARMS}
        raise PlanError("The Meets, the INSPECTs and the lid or seat tasks must come in the same order on both arms (at each, "
                        f"both arms meet): LEFT has {words['left']}; RIGHT has {words['right']}.")
    waits = _resolve(steps)
    pairs = _together_pairs(steps, sync)
    _walk(steps, waits, sync)
    bounds = {s: [-1] + [i for i, _ in sync[s]] + [len(steps[s])] for s in ARMS}
    stage_of = {s: {} for s in ARMS}
    plan, parts = [], []
    for k in range(len(bounds["left"]) - 1):
        span = {s: range(bounds[s][k] + 1, bounds[s][k + 1]) for s in ARMS}
        if any(len(span[s]) for s in ARMS):
            for s in ARMS:
                for i in span[s]:
                    stage_of[s][i] = len(plan)
            st = {"kind": "parallel", "program_id": f"PART {len(parts) + 1}",
                  **{s: ({"side": s, "steps": [steps[s][i] for i in span[s]]} if len(span[s]) else None) for s in ARMS}}
            if k < len(sync["left"]):                  # the row each arm meets the other at, after its part, and what it is
                st["meet_rows"] = {s: _row(steps[s], sync[s][k][0]) for s in ARMS}
                st["meet_kind"] = sync["left"][k][1]
            parts.append(st)
            plan.append(st)
        if k < len(sync["left"]) and sync["left"][k][1] == "inspect":     # the camera's look, both arms meeting there
            li, ri = sync["left"][k][0], sync["right"][k][0]
            pose = steps["right"][ri].get("pose") or steps["left"][li].get("pose") or ""
            plan.append({"kind": "inspect", "program_id": "INSPECT", "title": "INSPECT the commode", "pose": pose,
                         "rows": {"left": _row(steps["left"], li), "right": _row(steps["right"], ri)}})
        elif k < len(sync["left"]) and sync["left"][k][1] != "meet":
            task, li, ri = sync["left"][k][1], sync["left"][k][0], sync["right"][k][0]
            plan.append({"kind": "mechanism", "task": task, "program_id": mechanism.TASKS[task]["short"],
                         "title": mechanism.TASKS[task]["title"], "rows": {"left": _row(steps["left"], li), "right": _row(steps["right"], ri)}})
    for (side, i), (other, j) in waits.items():       # the plan's steps are these very dicts: marked in place
        here, there = stage_of[side][i], stage_of[other].get(j)
        steps[side][i]["_target"] = {"side": other, "row": _row(steps[other], j), "index": j,
                                     "done": there is None or there < here}
    for k, (li, ri) in enumerate(pairs):              # each Program that starts together, with its partner
        for side, i, j in (("left", li, ri), ("right", ri, li)):
            other = OTHER[side]
            steps[side][i]["_together"] = {"pair": k, "side": other, "row": _row(steps[other], j), "id": int(steps[other][j]["id"])}
    for st in parts:
        for s in ARMS:
            st[f"{s}_program_ids"] = [int(x["id"]) for x in ((st[s] or {}).get("steps") or []) if x["type"] == "program"]
    return plan


def single(side: str, steps: list) -> tuple[list, list]:
    """One arm's whole list on its own (RUN LEFT / RIGHT): the other arm does not run, so there is nothing to meet or
    wait for; those rows are passed over, said."""
    dropped = sum(1 for st in steps if st["type"] in GATES)
    keep = [st for st in steps if st["type"] not in GATES]
    marked = len(together_marks(steps))
    return keep, ([f"{side.upper()} alone: its {dropped} Meet / Wait-for row(s) are passed over (the other arm does not run)."]
                  if dropped else []) + ([f"{side.upper()} alone: its {marked} Program(s) marked to start together start on "
                                          "their own (the other arm does not run)."] if marked else [])


def counts(plan: list[dict]) -> dict:
    """What the run's confirmation says: the parts both arms run, the Meets that end them, the Wait-for rows, the pairs
    of Programs that start together, the lid and seat tasks."""
    parts = [st for st in plan if st["kind"] == "parallel"]
    waits = sum(1 for st in parts for s in ARMS for x in ((st.get(s) or {}).get("steps") or []) if x["type"] == "wait_for")
    together = sum(1 for st in parts for x in ((st.get("left") or {}).get("steps") or []) if x.get("_together"))
    return dict(parts=len(parts), meets=sum(1 for st in parts if st.get("meet_kind") == "meet"), waits_for=waits, together=together,
                mechanisms=sum(1 for st in plan if st["kind"] == "mechanism"), inspects=sum(1 for st in plan if st["kind"] == "inspect"),
                single=sum(1 for st in plan if st["kind"] == "single"), stages=len(plan))
