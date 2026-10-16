"""Merge + Full Sequence, S2: Full & Partial Sequence (feedback v5, slide 9; the owner, 16 Oct).

The control program's Page 02 (``rm_trajectory_ui_v53_safe_lid_checkpoint.py``, run by runner v14),
ported as it is: each arm's ordered steps (Program, Global Point, Pole, Hand, Wait), each Program with
its Start % (the controller's move to its first point; the stored timing is kept) and its Dual Rule
(TOGETHER, RUN FIRST, WAIT OTHER). RUN BOTH follows V53's SAFE TIMELINE plan (``build_plan``, V53's
own code): consecutive TOGETHER Programs as LEFT / RIGHT timeline blocks started together; a positive
WAIT before a TOGETHER Program on both arms as the manual lid checkpoint (both arms finish, run the
checkpoint's setup, then wait for the operator); RUN FIRST + WAIT OTHER as a one-arm-at-a-time
handoff, asking before the waiting arm when "Confirm before waiting arm" is on; the steps after the
last Program at the end. Each arm's part of a stage runs through runner v14's own
``run_workflow_sequence`` on that arm's controller, one call per arm per stage, as V53 runs it. Saved
and loaded as V53's JSON (``realman_workflow_sequence_v1``).

What Lite adds, none of it motion (the owner's decisions of 16 Oct):
- Simulation Mode: both arms in 3D through the same plan, with the collision check (walls, commode,
  the two arms against each other), and what would not be right on the robot: a Program whose pole
  or hand is not the one it was taught with, a Program Lite did not send (not drawn), a global point
  that could not be read;
- Real Machine Mode only after the same sequence (steps, range, programs, points, speed) completed in
  Simulation; Partial = a row range per arm;
- before the first move, every Program slot and global point checked on its controller (V53 finds a
  missing one mid-run); a loaded file of the other arm refused; the overall speed is the top bar's
  slider, and each controller's own speed, found when it was connected, is put back after (not
  V53's fixed 50 %); STOP latches, as everywhere in Lite.
"""
from __future__ import annotations

import copy
import hashlib
import json
import time
from pathlib import Path

from .jobs import UserError

FORMAT = "realman_workflow_sequence_v1"
RULES = ("TOGETHER", "RUN FIRST", "WAIT OTHER")
ARMS = ("left", "right")
MAX_STEPS = 500


# ── V53's planner, as it is (DualArmTrajectoryUI._page2_program_chunks and
#    _page2_build_program_rule_plan; tests/test_workflow.py runs V53's own methods against these)

def program_chunks(steps: list, side: str) -> list[dict]:
    """
    Split one arm's workflow into chunks that END at a Program step.

    This preserves setup steps that belong before that trajectory:
      Pole / Hand / Wait / Global / Program

    Example:
      Pole, Hand, Global, Program10, Program11, Pole, Program12

    becomes:
      chunk 1 = Pole + Hand + Global + Program10
      chunk 2 = Program11
      chunk 3 = Pole + Program12
    """
    chunks = []
    pending = []

    for step in steps:
        pending.append(json.loads(json.dumps(step)))

        if step.get("type") == "program":
            program = step
            chunks.append({
                "side": side,
                "steps": pending,
                "program_id": int(program.get("id")),
                "program_name": str(program.get("name") or ""),
                "dual_rule": str(
                    program.get("dual_rule") or "TOGETHER"
                ).upper(),
            })
            pending = []

    # Preserve any trailing non-program actions after the final Program.
    if pending:
        chunks.append({
            "side": side,
            "steps": pending,
            "program_id": None,
            "program_name": "FINAL AUX STEPS",
            "dual_rule": "TOGETHER",
        })

    return chunks


def build_plan(left_steps: list, right_steps: list) -> list[dict]:
    """
    Build the Page-02 dual-arm plan.

    v53 SAFE TIMELINE RULES

    TOGETHER sections may contain different Program IDs and different
    numbers of Program rows on LEFT and RIGHT.

    A positive WAIT located before a TOGETHER Program on BOTH arms is
    treated as a HARD CROSS-ARM CHECKPOINT:

      1. finish every preceding timeline Program on BOTH arms,
      2. run the checkpoint setup steps (for example Pole 150 + Wait 2 s)
         on BOTH arms,
      3. stop and require operator confirmation,
      4. only then start the Program after the WAIT on BOTH arms.

    This prevents one arm from entering the post-lid-cleaning Program
    while the other arm is still finishing the pre-lid section.

    RUN FIRST + WAIT OTHER remains an explicit serial handoff and still
    requires the same Program ID on both arms.

    (V53's errors are ValueError, raised before anything runs; Lite says them as they are.)
    """
    left_chunks = program_chunks(left_steps, "left")
    right_chunks = program_chunks(right_steps, "right")

    left_programs = [
        x for x in left_chunks if x["program_id"] is not None
    ]
    right_programs = [
        x for x in right_chunks if x["program_id"] is not None
    ]

    def validate_unique(items, side):
        seen = set()
        for item in items:
            pid = item["program_id"]
            if pid in seen:
                raise ValueError(
                    f"{side.upper()} Program {pid} appears more than once. "
                    "SAFE TIMELINE requires unique Program IDs per arm."
                )
            seen.add(pid)

    validate_unique(left_programs, "left")
    validate_unique(right_programs, "right")

    def rule(item):
        return str(item.get("dual_rule") or "TOGETHER").upper()

    def positive_wait_index(item):
        found = None
        for idx, step in enumerate(item.get("steps") or []):
            if str(step.get("type") or "").lower() != "wait":
                continue
            try:
                seconds = float(step.get("seconds", step.get("wait_s", 0)) or 0)
            except Exception:
                seconds = 0.0
            if seconds > 0:
                found = idx
        return found

    def combine(side, chunks, label="TOGETHER"):
        steps = []
        ids = []
        names = []

        for chunk in chunks:
            steps.extend(json.loads(json.dumps(chunk["steps"])))
            if chunk.get("program_id") is not None:
                ids.append(int(chunk["program_id"]))
            if chunk.get("program_name"):
                names.append(str(chunk["program_name"]))

        return {
            "side": side,
            "steps": steps,
            "program_id": ids[-1] if ids else None,
            "program_ids": ids,
            "program_name": " + ".join(names) or label,
            "dual_rule": "TOGETHER",
        }

    def split_wait_chunk(item):
        wi = positive_wait_index(item)
        if wi is None:
            raise ValueError("internal SAFE TIMELINE error: checkpoint WAIT not found")

        before = {
            "side": item["side"],
            "steps": json.loads(json.dumps(item["steps"][: wi + 1])),
            "program_id": None,
            "program_ids": [],
            "program_name": "MANUAL LID PREP",
            "dual_rule": "TOGETHER",
        }

        after_steps = json.loads(json.dumps(item["steps"][wi + 1:]))
        if not after_steps:
            raise ValueError(
                f"{item['side'].upper()} checkpoint WAIT has no Program after it."
            )

        programs_after = [
            s for s in after_steps if str(s.get("type") or "").lower() == "program"
        ]
        if len(programs_after) != 1:
            raise ValueError(
                f"{item['side'].upper()} checkpoint must lead to exactly one "
                "Program in that chunk."
            )

        program = programs_after[0]
        after = {
            "side": item["side"],
            "steps": after_steps,
            "program_id": int(program["id"]),
            "program_ids": [int(program["id"])],
            "program_name": str(program.get("name") or ""),
            "dual_rule": str(program.get("dual_rule") or "TOGETHER").upper(),
        }
        return before, after

    plan = []
    li = 0
    ri = 0
    timeline_number = 0
    checkpoint_number = 0

    while li < len(left_programs) or ri < len(right_programs):
        lcur = left_programs[li] if li < len(left_programs) else None
        rcur = right_programs[ri] if ri < len(right_programs) else None

        # ----------------------------------------------------------
        # TOGETHER region
        # ----------------------------------------------------------
        if (
            lcur is not None
            and rcur is not None
            and rule(lcur) == "TOGETHER"
            and rule(rcur) == "TOGETHER"
        ):
            # Locate the next WAIT checkpoint before the next serial rule.
            lbar = None
            j = li
            while j < len(left_programs) and rule(left_programs[j]) == "TOGETHER":
                if positive_wait_index(left_programs[j]) is not None:
                    lbar = j
                    break
                j += 1

            rbar = None
            j = ri
            while j < len(right_programs) and rule(right_programs[j]) == "TOGETHER":
                if positive_wait_index(right_programs[j]) is not None:
                    rbar = j
                    break
                j += 1

            if (lbar is None) != (rbar is None):
                raise ValueError(
                    "SAFE TIMELINE checkpoint mismatch.\n\n"
                    "A positive WAIT exists before the next serial section "
                    "on only one arm. Add the manual checkpoint WAIT to BOTH "
                    "arms at the same process boundary."
                )

            if lbar is not None and rbar is not None:
                # Everything BEFORE the checkpoint must complete on BOTH arms.
                left_pre = left_programs[li:lbar]
                right_pre = right_programs[ri:rbar]

                if bool(left_pre) != bool(right_pre):
                    raise ValueError(
                        "SAFE TIMELINE checkpoint alignment is ambiguous.\n\n"
                        f"LEFT Programs before checkpoint: "
                        f"{[x['program_id'] for x in left_pre] or 'none'}\n"
                        f"RIGHT Programs before checkpoint: "
                        f"{[x['program_id'] for x in right_pre] or 'none'}"
                    )

                if left_pre and right_pre:
                    timeline_number += 1
                    left_item = combine("left", left_pre)
                    right_item = combine("right", right_pre)
                    plan.append({
                        "kind": "parallel",
                        "program_id": f"TIMELINE-{timeline_number}",
                        "timeline_number": timeline_number,
                        "left_program_ids": left_item["program_ids"],
                        "right_program_ids": right_item["program_ids"],
                        "left": left_item,
                        "right": right_item,
                    })

                # Split the checkpoint chunk:
                # setup/pole/wait -> MANUAL CONFIRM -> Program.
                left_prep, left_after = split_wait_chunk(left_programs[lbar])
                right_prep, right_after = split_wait_chunk(right_programs[rbar])

                if rule(left_after) != "TOGETHER" or rule(right_after) != "TOGETHER":
                    raise ValueError(
                        "Program after the manual checkpoint must be TOGETHER "
                        "on both arms."
                    )

                checkpoint_number += 1

                plan.append({
                    "kind": "parallel",
                    "program_id": f"LID-PREP-{checkpoint_number}",
                    "timeline_number": None,
                    "left_program_ids": [],
                    "right_program_ids": [],
                    "left": left_prep,
                    "right": right_prep,
                })

                plan.append({
                    "kind": "checkpoint",
                    "program_id": f"LID-CHECK-{checkpoint_number}",
                    "checkpoint_number": checkpoint_number,
                    "title": "MANUAL LID COVER CHECKPOINT",
                    "message": (
                        "Both pre-lid cleaning sections are complete and both "
                        "checkpoint setup sequences have finished.\n\n"
                        "Open the toilet lid cover manually.\n\n"
                        "Before continuing, confirm:\n"
                        "• Lid cover is fully open\n"
                        "• Both hands are completely outside the robot workspace\n"
                        "• Both robot paths are clear\n\n"
                        "Press OK only when it is safe to start the next robot Programs."
                    ),
                })

                timeline_number += 1
                plan.append({
                    "kind": "parallel",
                    "program_id": f"TIMELINE-{timeline_number}",
                    "timeline_number": timeline_number,
                    "left_program_ids": left_after["program_ids"],
                    "right_program_ids": right_after["program_ids"],
                    "left": left_after,
                    "right": right_after,
                })

                li = lbar + 1
                ri = rbar + 1
                continue

            # No checkpoint before the next serial section:
            # take all consecutive TOGETHER chunks on each arm.
            left_block = []
            while li < len(left_programs) and rule(left_programs[li]) == "TOGETHER":
                left_block.append(left_programs[li])
                li += 1

            right_block = []
            while ri < len(right_programs) and rule(right_programs[ri]) == "TOGETHER":
                right_block.append(right_programs[ri])
                ri += 1

            if not left_block or not right_block:
                raise ValueError(
                    "SAFE TIMELINE found a TOGETHER section on only one arm."
                )

            timeline_number += 1
            left_item = combine("left", left_block)
            right_item = combine("right", right_block)
            plan.append({
                "kind": "parallel",
                "program_id": f"TIMELINE-{timeline_number}",
                "timeline_number": timeline_number,
                "left_program_ids": left_item["program_ids"],
                "right_program_ids": right_item["program_ids"],
                "left": left_item,
                "right": right_item,
            })
            continue

        # ----------------------------------------------------------
        # Serial RUN FIRST / WAIT OTHER
        # ----------------------------------------------------------
        if lcur is None or rcur is None:
            raise ValueError(
                "SAFE TIMELINE reached an unmatched Program section.\n\n"
                f"Next LEFT Program: "
                f"{None if lcur is None else lcur['program_id']}\n"
                f"Next RIGHT Program: "
                f"{None if rcur is None else rcur['program_id']}"
            )

        lpid = int(lcur["program_id"])
        rpid = int(rcur["program_id"])
        lr = rule(lcur)
        rr = rule(rcur)

        if lpid != rpid:
            raise ValueError(
                "RUN FIRST / WAIT OTHER serial handoffs must use the same "
                "Program ID on both arms.\n\n"
                f"LEFT  = Program {lpid} ({lr})\n"
                f"RIGHT = Program {rpid} ({rr})"
            )

        if lr == "RUN FIRST" and rr == "WAIT OTHER":
            plan.append({
                "kind": "serial",
                "program_id": lpid,
                "first": lcur,
                "second": rcur,
            })
        elif lr == "WAIT OTHER" and rr == "RUN FIRST":
            plan.append({
                "kind": "serial",
                "program_id": lpid,
                "first": rcur,
                "second": lcur,
            })
        else:
            raise ValueError(
                f"Program {lpid} has an invalid Dual Rule pair:\n\n"
                f"LEFT  = {lr}\n"
                f"RIGHT = {rr}\n\n"
                "Use TOGETHER on both arms, or RUN FIRST + WAIT OTHER."
            )

        li += 1
        ri += 1

    # Preserve trailing non-program steps after the final Program.
    left_tail = next(
        (x for x in left_chunks if x["program_id"] is None),
        None,
    )
    right_tail = next(
        (x for x in right_chunks if x["program_id"] is None),
        None,
    )

    if left_tail or right_tail:
        plan.append({
            "kind": "tail",
            "program_id": None,
            "left": left_tail,
            "right": right_tail,
        })

    return plan


def counts(plan: list[dict]) -> dict:
    """What V53's RUN BOTH dialog says before it starts."""
    n = {k: sum(1 for s in plan if s.get("kind") == k) for k in ("parallel", "serial", "checkpoint", "tail")}
    return dict(coordinated=n["parallel"] + n["serial"], together=n["parallel"], one_arm=n["serial"],
                checkpoints=n["checkpoint"], tail=n["tail"], stages=len(plan))


# ── the steps: checked as runner v14 checks them, kept in the project, saved and loaded as V53's JSON

def _runner():
    from .robot_real import _runner as runner

    return runner()


def check_steps(steps, side: str) -> list[dict]:
    """An arm's steps, each as the runner will take it (its ``normalize_workflow_step``) and with V53's
    own fields (a Program's Dual Rule, a Wait's note) kept: refused with the row that is wrong."""
    if not isinstance(steps, list):
        raise UserError(f"The {side.upper()} sequence must be a list of steps.")
    if len(steps) > MAX_STEPS:
        raise UserError(f"The {side.upper()} sequence has {len(steps)} steps: at most {MAX_STEPS}.")
    rt = _runner()
    out = []
    for i, raw in enumerate(steps, 1):
        try:
            norm = rt.normalize_workflow_step(raw)
        except (TypeError, ValueError, KeyError) as exc:
            raise UserError(f"{side.upper()} row {i}: {exc}.", "Correct or remove that row.", code="step") from None
        step = dict(norm)
        if norm["type"] == "program":
            rule = str(raw.get("dual_rule") or "TOGETHER").strip().upper()
            if rule not in RULES:
                raise UserError(f"{side.upper()} row {i}: the Dual Rule {rule!r} is not TOGETHER, RUN FIRST or WAIT OTHER.",
                                code="step")
            step["dual_rule"] = rule
        if raw.get("note"):
            step["note"] = str(raw["note"])[:200]
        out.append(step)
    return out


def _file(units) -> Path:
    return Path(units.root) / "sequence" / "s2.json"


def load(units) -> dict:
    try:
        d = json.loads(_file(units).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        d = {}
    return {side: list(d.get(side) or []) for side in ARMS} | {"updated": d.get("updated")}


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
    """V53's SAVE (WorkflowSequencePanel.sequence_document)."""
    return {"format": FORMAT, "side": side, "steps": steps}


def from_document(side: str, data) -> list[dict]:
    """V53's LOAD: its format and a list of steps, and its v51 compatibility (a Program row without a
    Start % gets 30 %). Lite refuses a file saved for the other arm: Program 71 is another trajectory
    on each controller."""
    if not isinstance(data, dict) or data.get("format") != FORMAT:
        raise UserError("Could not load the sequence: wrong sequence format.", f"A V53 sequence JSON has \"format\": \"{FORMAT}\".")
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


def rows(steps: list, rng) -> tuple[list, int]:
    """An arm's steps for a run: all of them, or the rows ``rng`` = [first, last] (1-based, both
    included: Partial). Returns the steps and the first row's number."""
    if rng in (None, "", []):
        return list(steps), 1
    try:
        a, b = int(rng[0]), int(rng[1])
    except (TypeError, ValueError, IndexError):
        raise UserError("A partial run is a first and a last row.") from None
    if not 1 <= a <= b <= len(steps):
        raise UserError(f"Rows {a} to {b} are not in a sequence of {len(steps)} step(s).", "Choose rows inside it.")
    return list(steps[a - 1:b]), a


def plan_for(steps: dict, mode: str, ranges: dict | None) -> dict:
    """What a run does: ``mode`` both (V53's RUN BOTH, its plan), left or right (V53's RUN LEFT / RIGHT
    SEQUENCE: that arm's steps in one runner call), over all the rows or a range per arm."""
    if mode not in ("both", *ARMS):
        raise UserError("Run both arms, or the left or the right arm.")
    ranges = ranges or {}
    picked, first = {}, {}
    for side in ARMS:
        if mode in ("both", side):
            picked[side], first[side] = rows(steps[side], ranges.get(side))
    if mode == "both":
        if not picked["left"] or not picked["right"]:
            raise UserError("Build or load BOTH LEFT and RIGHT sequences first.")
        try:
            plan = build_plan(picked["left"], picked["right"])
        except ValueError as exc:
            raise UserError(f"Cannot start dual sequence: {exc}", code="plan") from None
    else:
        if not picked[mode]:
            raise UserError(f"{mode.upper()} sequence is empty.")
        plan = [{"kind": "single", "program_id": None, mode: {"side": mode, "steps": picked[mode]}}]
    return dict(mode=mode, ranges={s: (ranges.get(s) or None) for s in picked}, first_row=first, steps=picked, plan=plan,
                counts=counts(plan))


def digest(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


# ── what Lite knows of each arm's controller: the programs it sent and the global points it read

_POINTS: dict = {}                     # side -> {"read": when, "points": {name: joints deg}}: the last read


def _project_files(units, side: str) -> dict:
    """The project's files of an arm by sha256: every region's recordings and kept versions, its
    transitions, the merges and the merges' tuned versions; each with its name, stage, phase (the
    commode's lid/seat state its region needs) and the UGV placement it was taught at."""
    from . import merges
    from .lite_units import display_name

    out = {}

    def add(sha, path, name, stage, phase, place, region):
        if sha and sha not in out:
            out[sha] = dict(path=str(path), name=name, stage=stage, phase=phase, placement=place, region=region)

    for m in merges.listing(units, side):                 # first: a program sent from S1 is named as the merge is now
        add(m.get("sha256"), merges.folder(units, side) / m["file"], m["name"], m.get("stage"), m.get("phase"), m.get("placement"),
            "Merges")
    keys = [k for k, u in units.units.items() if u["arm"] == side]    # the regions (a merge's copy in Tune is its merge)
    for key in keys:
        u = units.units[key]
        recs = units.recordings(key)
        by_file = {r["file"]: r for r in recs}
        for r in recs:
            add(r.get("sha256"), r["path"], r["name"], r.get("stage"), u.get("phase"), r.get("placement"), display_name(u))
        for v in units.versions(key):
            try:
                extra = json.loads(Path(v["path"]).with_name(Path(v["path"]).stem + ".extra.json").read_text(encoding="utf-8"))
            except (OSError, ValueError):
                extra = {}
            add(v.get("sha256"), v["path"], v.get("name") or f"{display_name(u)} {v['version']}", v.get("stage"), u.get("phase"),
                (by_file.get(extra.get("raw_file")) or {}).get("placement"), display_name(u))
        tv = units.transition(key)
        if tv:
            add(tv.get("sha256"), tv["path"], tv["name"], tv.get("stage"), u.get("phase"), None, display_name(u))
    return out


def programs_known(app, units, side: str) -> dict:
    """{program id: what Lite knows}: the programs it sent to this arm's controller (its record: name,
    lines, sha256, stage) with the project's file of those bytes, when it is still there. On a linked
    controller, the slots it holds now too: a program Lite did not send, or one that replaced Lite's,
    is known by its name only (``sent`` False)."""
    files = _project_files(units, side)
    out = {}
    for pid, r in app.robot.arm(side).programs.items():
        f = files.get(r.get("sha256")) or {}
        out[int(pid)] = dict(id=int(pid), name=r.get("name"), lines=r.get("lines"), sha256=r.get("sha256"), stage=r.get("stage"),
                             sent=True, file=f.get("path"), file_name=f.get("name"), phase=f.get("phase"),
                             placement=f.get("placement"), region=f.get("region"))
    held = None
    if app.robot._real(side) is not None and app.robot.arm(side).connected and not app.jobs.running(arm=side):
        try:
            held = {int(x["id"]): x for x in app.robot.programs(side) if x.get("name")}
        except UserError:
            held = None
    if held is not None:
        for pid, x in held.items():
            if not x.get("sha256"):                       # the controller's own, not what Lite sent
                out[pid] = dict(id=pid, name=x.get("name"), size=x.get("bytes"), lines=None, sha256=None, stage=None,
                                sent=False, file=None)
        for pid in [p for p in out if p not in held]:
            out[pid] = dict(out[pid], gone=True)          # Lite sent it, the controller no longer holds it
    return out


def points_known(app, side: str) -> dict | None:
    """The global points of a linked arm's controller, read now when the arm is idle, else as last read."""
    if app.robot._real(side) is not None and app.robot.arm(side).connected and not app.jobs.running(arm=side):
        try:
            pts = app.robot.global_points(side) or []
            _POINTS[side] = dict(read=time.strftime("%Y-%m-%d %H:%M:%S"), points={p["name"]: p["joints"] for p in pts})
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
    steps, mode and rows, the programs it uses as Lite knows them, the global points it moves to as
    read, and the overall speed."""
    used = {}
    for side, steps in run["steps"].items():
        pids = sorted({int(s["id"]) for s in steps if s["type"] == "program"})
        names = sorted({s["name"] for s in steps if s["type"] == "global"})
        pts = (points.get(side) or {}).get("points") or {}
        used[side] = dict(programs={str(p): _fingerprint(programs[side].get(p)) for p in pids},
                          points={n: [round(float(v), 2) for v in pts[n]] if n in pts else None for n in names})
    return dict(mode=run["mode"], ranges=run["ranges"], steps={s: [{k: v for k, v in st.items() if k != "_row"} for st in steps]
                                                               for s, steps in run["steps"].items()},
                used=used, overall=int(overall))


# ── Simulation Mode: each arm's actions on one clock, through V53's plan

CYCLE_S = 0.005
SAMPLE_S = 0.02                  # the collision check's clock: every 4th 5 ms line
MAX_SAMPLES = 90000
FRAME_S = 0.04                   # the 3D's frames
MAX_FRAMES = 6000


def _ease(f):
    import numpy as np

    return 0.5 - 0.5 * np.cos(np.pi * np.clip(f, 0.0, 1.0))


def _move_s(q0, q1, pct: float, rating) -> float:
    """A joint move's time at ``pct`` % of each joint's rating, as Lite's 3D times the move to a start
    (staging.execution_times): at least a second, and half a second to settle."""
    import numpy as np

    frac = max(1.0, min(100.0, float(pct))) / 100.0
    d = np.abs(np.asarray(q1, dtype=float) - np.asarray(q0, dtype=float))
    return max(1.0, float(np.max(d / (frac * np.asarray(rating, dtype=float))))) + 0.5


class _ArmSim:
    """One arm through its steps: what each step does and when (its actions), from a start state."""

    def __init__(self, side, start, rating, overall, programs, points, md_of, findings):
        self.side, self.rating, self.overall = side, rating, overall
        self.pole, self.hand, self.q = float(start["pole_mm"]), [int(c) for c in start["hand"]], [float(v) for v in start["q_deg"]]
        self.programs, self.points, self.md_of, self.findings = programs, points, md_of, findings
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
        was, and the collision check leaves it out until a drawn Program puts it on a known line again."""
        self._say("warn", step, f"{self.side.upper()} row {step.get('_row')}: {why}. From here the arm's pose is not known: "
                  "it is shown where it was, and the collision check leaves this arm out until a drawn Program.")
        self.known = False

    def run(self, steps, t, stage) -> float:
        from . import staging

        S = self.side.upper()
        for step in steps:
            kind = step["type"]
            if kind == "pole":
                mm = float(step["mm"])
                dur = staging.pole_travel_s(self.pole, mm)
                a = self._act(stage, step, "pole", t, dur, f"Pole {mm:g} mm")
                self.pole = mm
                self.set_pole = True
            elif kind == "hand":
                vals = [int(v) for v in step["values"]]
                dur = staging.hand_move_s(self.hand, vals)
                a = self._act(stage, step, "hand", t, dur, "Hand " + ",".join(str(v) for v in vals))
                self.hand = vals
                self.set_hand = True
            elif kind == "wait":
                dur = float(step["seconds"])
                a = self._act(stage, step, "wait", t, dur, f"Wait {dur:g} s")
            elif kind == "global":
                pts = (self.points or {}).get("points") or {}
                name, pct = step["name"], float(step["speed"]) * self.overall / 100.0
                if name in pts:
                    q1 = [float(v) for v in pts[name]]
                    mv = _move_s(self.q, q1, pct, self.rating)
                    dur = mv + float(step["wait_s"])
                    a = self._act(stage, step, "global", t, dur, f"{step['move'].upper()} {name} ({step['speed']} %)", move_s=round(mv, 4),
                                  drawn=True)
                    if step["move"] == "movel":
                        self._say("note", step, f"{S} row {step.get('_row')}: MoveL to {name} is drawn as a joint move "
                                  "(the controller moves the tool in a straight line).")
                    self.q = q1
                else:
                    dur = float(step["wait_s"])
                    a = self._act(stage, step, "global", t, dur, f"{step['move'].upper()} {name}: not drawn", drawn=False)
                    self._unknown(step, f"global point {name} is not drawn: "
                                  + ("the controller's global points were not read (connect the arm)" if not self.points
                                     else "the controller has no point of that name"))
            else:
                a = self._program(step, t, stage)
                dur = a["t1"] - a["t0"]
            a.update(pole1=self.pole, hand1=list(self.hand), q1=list(self.q))
            t += dur
        return t

    def _program(self, step, t, stage) -> dict:
        from . import staging

        S, pid = self.side.upper(), int(step["id"])
        info = self.programs.get(pid) or {}
        label = f"Program {pid}" + (f" // {info.get('name') or step.get('name')}" if (info.get("name") or step.get("name")) else "")
        st = staging.stage_from(info.get("stage"))
        if st is not None:                           # V53 stages nothing itself: its own Pole and Hand steps must match
            off = []
            if abs(self.pole - st.pole_mm) > staging.POLE_TOL_MM:
                off.append(f"the pole is at {self.pole:g} mm here" + ("" if self.set_pole else " (as the arm stands: no Pole step before it in this run)"))
            if not staging.same_hand(self.hand, st.hand):
                off.append(f"the hand is {staging.hand_words(self.hand)} here" + ("" if self.set_hand else " (as the arm stands: no Hand step before it)"))
            if off:
                self._say("warn", step, f"{S} row {step.get('_row')}, program {pid}: taught at {staging.describe(st)}, but "
                          + " and ".join(off) + ".")
        md = self.md_of(info.get("file")) if info.get("file") and not info.get("gone") else None
        if md is None:
            why = ("the controller no longer holds the program Lite sent" if info.get("gone")
                   else "the controller holds a program Lite did not send" if info and not info.get("sent")
                   else "Lite has not sent this program" if not info
                   else "its file is no longer in the project")
            lines = int(info.get("lines") or 0)
            dur = max(lines - 1, 0) * CYCLE_S
            a = self._act(stage, step, "program", t, dur, label + ": not drawn", program=pid, drawn=False, why=why, phase=None)
            self._unknown(step, f"program {pid} is not drawn ({why}" + (f"; {dur:.1f} s allowed for it)" if dur else ")"))
            return a
        q_first = (md[0] / 1000.0).tolist()
        transit = _move_s(self.q, q_first, float(step["transit_speed"]), self.rating)
        traj = (md.shape[0] - 1) * CYCLE_S
        a = self._act(stage, step, "program", t, transit + traj, label, program=pid, drawn=True, file=info["file"],
                      file_name=info.get("file_name"), lines=int(md.shape[0]), traj_t0=round(t + transit, 4), phase=info.get("phase"),
                      placement=info.get("placement"))
        self.q = (md[-1] / 1000.0).tolist()
        self.known = True                            # on the program's own lines: known again (its move to them is not)
        return a


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


def schedule(run: dict, sims: dict) -> tuple[list[dict], float]:
    """V53's plan on one clock: a TOGETHER (parallel) stage starts both arms' parts at once and ends
    when both have; a serial stage runs the waiting arm when the first has finished; a checkpoint is a
    moment (in Real the robots stop there for the operator); a single arm's run is one part."""
    T, out = 0.0, []
    for i, st in enumerate(run["plan"]):
        kind = st["kind"]
        row = dict(index=i, kind=kind, label=str(st.get("program_id") or ("FINAL AUX STEPS" if kind == "tail" else "")), t0=round(T, 4))
        if kind == "checkpoint":
            row.update(t1=round(T, 4), title=st["title"], message=st["message"])
        elif kind == "serial":
            first, second = st["first"], st["second"]
            mid = sims[first["side"]].run(first["steps"], T, i)
            T = sims[second["side"]].run(second["steps"], mid, i)
            row.update(t1=round(T, 4), first=first["side"], second=second["side"], handoff_t=round(mid, 4))
        else:
            ends = [sims[s].run(st[s]["steps"], T, i) for s in ARMS if st.get(s)]
            T = max(ends or [T])
            row.update(t1=round(T, 4))
            if kind == "single":
                row["label"] = f"{[s for s in ARMS if st.get(s)][0].upper()} SEQUENCE"
        for s in ARMS:
            item = st.get(s) if kind != "serial" else (st["first"] if st["first"]["side"] == s else st["second"])
            if item:
                row[f"{s}_programs"] = [int(x["id"]) for x in item["steps"] if x["type"] == "program"]
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
    if a["kind"] == "program":
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
    if a["kind"] == "global" and a.get("drawn"):
        g = (t - a["t0"]) / max(1e-9, a["move_s"])
        return p0, h0, q0 + float(_ease(g)) * (q1 - q0)
    if a["kind"] == "program" and a.get("drawn"):
        md = md_of(a["file"])
        if t < a["traj_t0"]:
            g = (t - a["t0"]) / max(1e-9, a["traj_t0"] - a["t0"])
            return p0, h0, q0 + float(_ease(g)) * (md[0] / 1000.0 - q0)
        i = int(round((t - a["traj_t0"]) / CYCLE_S))
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


def _md_reader():
    import numpy as np

    from ..motion import track

    cache: dict = {}

    def md_of(path):
        if not path:
            return None
        if path not in cache:
            try:
                cache[path] = np.asarray(track.read_md(Path(path)), dtype=float)
            except (OSError, ValueError):
                cache[path] = None
        return cache[path]

    return md_of


def _phase_changes(acts: dict) -> list[tuple[float, str]]:
    """The commode's lid/seat state through the run: each drawn Program's region's, from its start."""
    out = sorted((a["t0"], a["phase"]) for side in acts.values() for a in side
                 if a["kind"] == "program" and a.get("drawn") and a.get("phase"))
    return out or [(0.0, "lid_closed")]


def _phase_at(changes, t: float) -> str:
    ph = changes[0][1]
    for t0, p in changes:
        if t0 <= t + 1e-9:
            ph = p
        else:
            break
    return ph


def simulate(app, units, mode: str, ranges: dict | None):
    """Simulation Mode: the run (``plan_for``) through V53's plan on one clock, both arms, in the
    room and at the UGV placement its Programs were taught at; the collision check over all of it; and
    what would not be right on the robot (``findings``). Kept as the project's last simulation, with
    what makes it the same run (``identity_of``)."""
    from . import collision_check, placement, staging, workers
    from ..motion.limits import load_limits
    from .routes_robot import _payload

    run = plan_for(_numbered(load(units)), mode, ranges)
    overall = int(app.robot.speed_pct)
    home = dict(_payload(app)["home"])
    cub = int(app.ui.get("cubicle_mm", 1400))         # the cubicle chosen, when the room is not measured

    def work(job):
        import numpy as np

        job.progress("Reading what the controllers hold", 0, 3)
        sides = list(run["steps"])
        programs = {s: programs_known(app, units, s) for s in ARMS}
        points = {s: points_known(app, s) for s in ARMS}
        ident = identity_of(run, programs, points, overall)
        findings, md_of = [], _md_reader()
        starts, start_how, sims = {}, {}, {}
        for s in ARMS:
            starts[s], start_how[s] = _start_state(app, s, home)
            sims[s] = _ArmSim(s, starts[s], load_limits(s).rating_deg_s, overall, programs[s], points[s], md_of, findings)
        job.progress("Timing the stages", 1, 3)
        stages, total = schedule(run, sims)
        acts = {s: sims[s].acts for s in ARMS}
        places = [a.get("placement") for s in sides for a in acts[s] if a["kind"] == "program" and a.get("drawn")]
        place = next((p for p in places if p), None)
        if any(p and placement.differences(place, p) for p in places):
            findings.append(dict(level="warn", side=None, row=None, text="The Programs were taught at different UGV placements: "
                                 f"the 3D and the check use the first one's ({placement.words(place)})."))
        scene = placement.scene(units.fixture, placement.load(units)["room"], place)
        base = dict(home)
        for s in ARMS:
            base = staging.start_values(s, base, starts[s])
        changes = _phase_changes(acts)
        dt = max(SAMPLE_S, total / MAX_SAMPLES)
        times = np.arange(0.0, total + 1e-9, dt).tolist() if total > 0 else [0.0]
        arms = {}
        for s in sides:
            pole, hand, q, known = sample(acts[s], times, starts[s], md_of, with_known=True)
            arms[s] = dict(pole=pole.tolist(), hand=hand.tolist(), q=q.tolist(), known=known.tolist())
        job.progress(f"Checking collisions: {len(times):,} moments, both arms", 2, 3)
        phases = [_phase_at(changes, t) for t in times]
        fut = workers.pool().submit(collision_check.check_states, times, arms, base, scene, units.fixture, phases, cub)
        while not fut.done():                         # STOP is heard while it checks
            job.sleep(0.2)
        check = fut.result()
        job.check()
        for e in check["events"]:                         # where in the sequence: each arm's row and line then
            e["at"] = [_where(acts[s], e["first_t"]) for s in (ARMS if "both" in e["sides"] else e["sides"]) if s in acts]
        out = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), hash=digest(ident), identity=ident, completed=False,
                   mode=run["mode"], ranges=run["ranges"], overall=overall, counts=run["counts"], stages=stages,
                   total_s=round(total, 3), acts=acts, start=starts, start_from=start_how,
                   findings=findings, collisions=check, scene=scene, phases=changes,
                   programs={s: {str(k): {kk: v.get(kk) for kk in ("name", "sent", "file_name", "region", "gone", "lines")}
                                 for k, v in programs[s].items()} for s in ARMS},
                   points={s: (points[s] or {}).get("read") for s in ARMS})
        from .project import _atomic_write

        _sim_file(units).parent.mkdir(parents=True, exist_ok=True)
        _atomic_write(_sim_file(units), json.dumps(out))
        job.log(f"Simulated: {len(stages)} stage(s), {total:.1f} s; " + ("no collision" if check["ok"] else
                f"{len(check['events']) + check['more']} collision event(s), the first at {check['first']['first_t']:.2f} s")
                + f"; {sum(1 for f in findings if f['level'] == 'warn')} warning(s).")
        return out

    return app.jobs.start("simulate", f"S2 simulation ({'both arms' if mode == 'both' else mode.upper() + ' arm'})", work)


def _where(acts: list[dict], t: float) -> dict | None:
    a = next((x for x in acts if x["t0"] <= t < x["t1"]), None) or next((x for x in reversed(acts) if x["t0"] <= t), None)
    if a is None:
        return None
    out = dict(side=a["side"], row=a["row"], kind=a["kind"], text=a["text"], stage=a["stage"])
    if a["kind"] == "program" and a.get("drawn") and t >= a["traj_t0"]:
        out["line"] = 1 + int(round((t - a["traj_t0"]) / CYCLE_S))
    return out


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
    acts = sim["acts"][side]
    mine = [a for a in acts if a["stage"] == int(stage)]
    if int(stage) == -1:
        times = [0.0]
    elif not mine:
        raise UserError(f"The {side.upper()} arm has nothing in that stage.")
    else:
        t0, t1 = mine[0]["t0"], mine[-1]["t1"]
        dt = max(FRAME_S, (t1 - t0) / MAX_FRAMES)
        times = np.arange(t0, t1, dt).tolist() + [t1]
    pole, hand, q = sample(acts, times, sim["start"][side], _md_reader())
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


def played(units, h: str) -> dict:
    """The page played the simulation ``h`` to its end: Real Machine Mode may run that same sequence."""
    from .project import _atomic_write

    with units.lock:
        sim = load_sim(units)
        if not sim or sim.get("hash") != h:
            raise UserError("That simulation is no longer the last one.", "Run it again in Simulation Mode.")
        if not (sim.get("total_s") or 0) > 0 or not any(a.get("drawn") is not False and a["t1"] > a["t0"]
                                                        for side in sim["acts"].values() for a in side):
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
            "acts": {s: [{k: a.get(k) for k in ("row", "kind", "t0", "t1", "text", "stage", "drawn", "program", "traj_t0", "lines")}
                         for a in sim["acts"][s]] for s in ARMS},
            "steps": sim["identity"]["steps"]}
    return dict(load(units), sim=sim)


# ── Real Machine Mode: V53's run, stage by stage, each arm's part through runner v14 on its controller

def _changed(sim: dict | None, ident: dict) -> str:
    """What is not as it was simulated, in words."""
    if not sim:
        return "it has not been simulated"
    old = sim.get("identity") or {}
    what = []
    if old.get("mode") != ident["mode"] or old.get("ranges") != ident["ranges"]:
        what.append("the arms or the rows chosen")
    if old.get("steps") != ident["steps"]:
        what.append("its steps")
    for side, u in ident["used"].items():
        o = (old.get("used") or {}).get(side) or {}
        if o.get("programs") != u["programs"]:
            what.append(f"the {side.upper()} programs Lite sent")
        if o.get("points") != u["points"]:
            what.append(f"the {side.upper()} global points (as read from the controller)")
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
    if k == "program":
        return f"Program {step['id']}" + (f" // {step['name']}" if step.get("name") else "")
    if k == "pole":
        return f"Pole {float(step['mm']):g} mm"
    if k == "hand":
        return "Hand " + ",".join(str(v) for v in step["values"])
    if k == "wait":
        return f"Wait {float(step['seconds']):g} s"
    return f"{step['move'].upper()} {step['name']}"


def run_real(app, units, mode: str, ranges: dict | None, confirm_each: bool, body: dict):
    """Real Machine Mode (V53's RUN BOTH / RUN LEFT / RUN RIGHT): only in Real robot mode, on linked
    controllers, after the same run completed in Simulation; every Program slot and global point it
    uses checked on its controller before anything moves; what the simulation found asked about once,
    with the real-robot confirmation. Then V53's stages: a TOGETHER stage starts both arms' parts at
    once and waits for both; a serial stage runs the first arm, asks before the waiting arm when
    ``confirm_each`` (V53's CONFIRM BEFORE WAITING ARM), then runs it; a checkpoint waits for the
    operator's OK; a failed part ends the run there. STOP (any) interrupts the runner as V53's STOP does
    and latches the arms as Lite's STOP always does."""
    import threading

    robot = app.robot
    run = plan_for(_numbered(load(units)), mode, ranges)
    sides = list(run["steps"])
    if app.ui["mode"] != "real":
        raise UserError("Real Machine Mode runs the sequence on the robot, so Lite must be in Real robot mode.",
                        "Choose Real robot in the status pill at the top, or run it in Simulation Mode.", code="mode")
    for s in sides:
        if robot._real(s) is None or not robot.arm(s).connected:
            raise UserError(f"The {s.upper()} arm's controller is not connected: Real Machine Mode runs the sequence with the "
                            "control program's runner on the controllers.", "Connect the arm (Lite started with --robot realman).",
                            code="connect")
    robot.require_motion("both" if mode == "both" else mode)
    overall = int(robot.speed_pct)
    programs = {s: programs_known(app, units, s) for s in ARMS}
    points = {s: points_known(app, s) for s in ARMS}
    ident = identity_of(run, programs, points, overall)
    sim = load_sim(units)
    if not sim or sim.get("hash") != digest(ident):
        raise UserError(f"This run is not the one simulated: {_changed(sim, ident)}.",
                        "Run it in Simulation Mode first: Real Machine Mode runs only the sequence that completed there.",
                        code="simulate")
    if not sim.get("completed"):
        raise UserError("The simulation of this run has not been played to its end.",
                        "Play it to the end in Simulation Mode, then run it on the robot.", code="simulate")
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
    warns = [f["text"] for f in sim["findings"] if f["level"] == "warn"]
    col = sim["collisions"]
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
    used = {s: {int(st["id"]) for st in run["steps"][s] if st["type"] == "program"} for s in sides}
    lines_of = {s: {pid: int(programs[s][pid]["lines"]) for pid in used[s]
                    if (programs[s].get(pid) or {}).get("sent") and (programs[s][pid].get("lines") or 0) > 1} for s in sides}
    longest = max([n for s in sides for n in lines_of[s].values()] or [0])
    timeout = max(300.0, (longest - 1) * CYCLE_S * 1.5 + 60.0)        # the runner's 300 s, longer for this run's longer programs

    def work(job):
        live = {s: "READY" for s in ARMS}
        rows = {s: None for s in ARMS}
        job.set(live=dict(live), status=f"PAGE 02 PROGRAM RULES // {len(run['plan'])} coordinated stage(s)", stage=None,
                question=None, counts=run["counts"], rows=dict(rows), mode=run["mode"])
        reals = {s: robot._real(s) for s in sides}
        originals = {}

        def say(status=None, **arms):
            live.update({k: v for k, v in arms.items()})
            job.set(live=dict(live), **({"status": status} if status else {}))

        stops = {}                                  # each running part's own stop (its runner, V53's way)
        stopped_why = {}                            # why Lite stopped a part, said with the failure

        def part(side, item, label):
            steps = split_waits(item["steps"])
            current = {"slot": None}

            def stop_hook(fn):
                stops[side] = fn
                job.on_cancel(fn)

            def on_line(line):                      # in the runner's own thread, as it prints
                text = line.strip()
                if text.startswith("WORKFLOW STEP START"):
                    try:
                        k = int(text.split()[3].split("/")[0])
                        rows[side] = steps[k - 1].get("_row")
                        job.set(rows=dict(rows))
                        say(**{side: f"{label} // {k}/{len(steps)} {_step_words(steps[k - 1])} // RUNNING"})
                    except (IndexError, ValueError):
                        pass
                    # The E-stop between two steps (the link reads nothing while the runner holds the arm): a
                    # press latches and stops both arms (their stop hooks); the runner stops at once.
                    if robot.estop_check(side):
                        stopped_why[side] = f"the {side.upper()} E-stop was pressed"
                m = _PROGRAM_RE.match(text)
                if m:
                    current["slot"] = int(m.group(1))
                m = _COMPLETE_RE.match(text)
                if m and current["slot"] in lines_of[side]:
                    from .robot_standin import ended_early

                    short = ended_early(float(m.group(1)), lines_of[side][current["slot"]], robot.emulator_scale(side))
                    if short:                       # stopped on the controller (an E-stop gives no error code)
                        stopped_why[side] = f"program {current['slot']} ended {short}: it was stopped on the controller"
                        stops[side]()
                if text.startswith(("WORKFLOW STEP START", "WORKFLOW COMPLETE", "**", "!!")):
                    job.log(f"{side.upper()}: {text}", "error" if text.startswith("**") else "info")

            try:
                out = reals[side].run_workflow(steps, overall, restore[side], timeout, on_line, stop_hook)
            finally:
                stops.pop(side, None)
            if stopped_why.get(side):
                out = dict(out, ok=False, why=stopped_why[side])
            say(**{side: f"{label} // COMPLETE" if out["ok"] else f"{label} // FAILED rc={out['rc']}"})
            return out

        def together(stage, label, status):
            say(status, **{s: f"{label} // RUNNING" for s in ARMS})
            barrier, results = threading.Barrier(2), {}

            def go(side):
                try:
                    barrier.wait(timeout=10)
                except threading.BrokenBarrierError:
                    results[side] = dict(ok=False, rc=1, why="the other arm did not start")
                    return
                try:
                    results[side] = part(side, stage[side], label)
                except BaseException as exc:                               # noqa: BLE001 - said below
                    results[side] = dict(ok=False, rc=1, why=str(exc) or type(exc).__name__)
                if not results[side].get("ok"):     # the owner (16 Oct): one arm failing stops the other as well
                    other = "right" if side == "left" else "left"
                    fn = stops.get(other)
                    if fn is not None:
                        stopped_why.setdefault(other, f"stopped because the {side.upper()} arm's part failed")
                        fn()

            threads = [threading.Thread(target=go, args=(s,), name=f"lite-s2-{s}", daemon=True) for s in ARMS]
            for t in threads:
                t.start()
            for t in threads:                       # both parts end before anything else: STOP interrupts both
                while t.is_alive():
                    t.join(0.1)
            job.check()
            bad = [s for s in ARMS if not results.get(s, {}).get("ok")]
            if bad:
                raise UserError("PROGRAM RULES STOPPED // parallel Program stage failed // "
                                + " // ".join(f"{s.upper()} rc={results.get(s, {}).get('rc')}" for s in ARMS)
                                + ": " + "; ".join(f"{s.upper()}: {results.get(s, {}).get('why') or 'see the hardware log'}" for s in bad),
                                "Check the arms and their controllers' errors before anything else runs.")

        def alone(item, label, status):
            side = item["side"]
            say(status, **{side: f"{label} // RUNNING"})
            out = part(side, item, label)
            job.check()
            if not out["ok"]:
                raise UserError(f"{side.upper()} {label} failed (rc={out['rc']}): {out['why'] or 'see the hardware log'}.",
                                "Check the arm and its controller's errors before anything else runs.")
            return out

        t0 = time.monotonic()
        try:
            for s in sides:
                originals[s] = reals[s].set_run_mode("REAL")
            for i, st in enumerate(run["plan"]):
                job.check()
                job.set(stage=i)
                job.progress(f"Stage {i + 1} of {len(run['plan'])}", i, len(run["plan"]))
                kind, pid = st["kind"], st.get("program_id")
                if kind == "checkpoint":
                    say("SAFE TIMELINE // MANUAL LID COVER CHECKPOINT // ROBOTS PAUSED", left="PAUSED // MANUAL LID CHECK",
                        right="PAUSED // MANUAL LID CHECK")
                    if not job.ask(st["title"], st["message"], ok="OK", cancel="Cancel"):
                        say("SAFE TIMELINE CANCELLED // MANUAL CHECKPOINT NOT CONFIRMED", left="IDLE // CHECKPOINT CANCELLED",
                            right="IDLE // CHECKPOINT CANCELLED")
                        raise UserError("The manual checkpoint was not confirmed: the sequence ended there.",
                                        "Nothing ran after it.")
                    say("SAFE TIMELINE // MANUAL LID CONFIRMED // RESUMING", left="READY // LID CONFIRMED",
                        right="READY // LID CONFIRMED")
                elif kind == "parallel":
                    together(st, f"Program {pid}", f"PROGRAM RULES // Program {pid} // LEFT + RIGHT TOGETHER")
                elif kind == "tail" and st.get("left") and st.get("right"):
                    together(st, "FINAL AUX", "PROGRAM RULES // FINAL AUX STEPS // LEFT + RIGHT")
                elif kind == "tail":
                    item = st.get("left") or st.get("right")
                    alone(item, "FINAL AUX", f"PROGRAM RULES // {item['side'].upper()} FINAL AUX STEPS")
                elif kind == "serial":
                    first, second = st["first"], st["second"]
                    F, S = first["side"].upper(), second["side"].upper()
                    say(None, **{second["side"]: f"Program {pid} // WAITING FOR {F}"})
                    alone(first, f"Program {pid}", f"PROGRAM RULES // Program {pid} // {F} RUN FIRST // {S} WAITING")
                    if confirm_each:
                        say(f"PROGRAM RULES // {F} Program {pid} COMPLETE // {S} Program {pid} WAITING",
                            **{second["side"]: f"Program {pid} // WAITING // CONFIRMATION REQUIRED"})
                        if not job.ask("Program Handoff", f"{F} Program {pid} completed.\n\n{S} Program {pid} is waiting.\n\n"
                                       f"Start {S} Program {pid} now?", ok="Yes", cancel="No"):
                            say(f"PROGRAM RULES PAUSED // Program {pid} handoff cancelled",
                                **{second["side"]: f"Program {pid} // HANDOFF CANCELLED"})
                            raise UserError(f"The {S} arm's Program {pid} was not started: the sequence ended there.",
                                            "Nothing ran after it.")
                    else:
                        say(None, **{second["side"]: f"Program {pid} // WAITING // AUTO HANDOFF"})
                    alone(second, f"Program {pid}", f"PROGRAM RULES // Program {pid} // {S} STARTING AFTER OTHER ARM")
                else:                                                     # one arm's whole sequence (V53's RUN LEFT / RIGHT)
                    item = st[sides[0]]
                    alone(item, "SEQUENCE", f"PAGE 02 // {sides[0].upper()} sequence")
            elapsed = time.monotonic() - t0
            say(f"PAGE 02 PROGRAM RULES COMPLETE // {elapsed:.1f} s", **{s: "SEQUENCE COMPLETE" for s in sides})
            job.log(f"Sequence complete: {len(run['plan'])} stage(s) in {elapsed:.1f} s.")
            return dict(mode=run["mode"], stages=len(run["plan"]), seconds=round(elapsed, 1))
        finally:
            for s, orig in originals.items():
                reals[s].restore_run_mode(orig)
            for s in sides:                         # the runner gives the speed back in its finally; again here,
                reals[s].restore_speed(restore[s])  # in case a stop landed before its try (never raises)

    title = "S2 Real Machine: " + ("both arms, SAFE TIMELINE" if mode == "both" else f"{mode.upper()} sequence")
    return robot._start("run", title, work, "both" if mode == "both" else mode, "real")
