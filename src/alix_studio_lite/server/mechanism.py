"""The lid and the seat: ALIX's mechanism tasks, pre-recorded, run by Lite with no ROS.

The owner, 17-18 Oct: "lid and seat open and close using the camera ... instead of uploading a file, you
stream"; "the mechanism tasks should just be like other tasks, but these are pre-trained or recorded so they
are not subjected to teaching procedure ... tasks that are available to be injected at a point in the
sequence"; "Lite should be ROS 2 independent".

What runs is ALIX's own: commode C's four hardware plans (vendor/alix/Resource/plans/commode_c/hardware,
planned at a 250 mm standoff with no UGV offset; run on the robot and in Isaac Sim), with their camera
checks, re-lifts and prerequisites. What runs them is ALIX's arm_trajectory_sender.py
(ros/orchestration/alix_tasks/scripts, commit 842c0e59), ported here with ROS replaced:

- each arm move is streamed over the controller's CanFD pass-through, one line every 5 ms, by ALIX's
  ROS-free lane (alix_dispatch ``track.run_verified(channel="canfd")``, as cleaning_tracks.py runs a track:
  the 5 ms capture and its watch, the per-joint rate gate). The lifts and pushes come from the plans' own
  5 ms tracks (.g0/.g1, sha256 checked), the other moves are resampled by the same law
  (cleaning_tracks.emit_fastpath). On the robot ROS's controllers streamed the same moves every 10 ms;
- the pole and the hand as Lite moves them everywhere (ALIX's peripherals, read back);
- each check asks the wrist camera through gap_check.py (ALIX's gap check, ported) and is judged by
  ALIX's lift_check.py, vendored as it is;
- where ALIX planned with MoveIt, Lite moves as it does for every run: to a plan's start (and, after a
  skip, to where the plan would have ended) the pole and the hand first, then the controller's MoveJ at
  the speed slider's speed; after a re-lift the next move of the lifting arm starts where the ORIGINAL
  lift ended (7.7 to 24.3 degrees away; ALIX's driver jumped it), so it is bridged by ALIX's own explicit
  MoveJ to the stream's first line; a re-lift with no pre-planned rung (seat_open) first goes back along
  the lift it just made, in reverse, where ALIX planned a restore with MoveIt.

The order is ALIX's: the entries one after the other, each one done before the next starts (the two arms
never move at the same time inside a plan); a plan's checks before the entry that carries them; on a NO,
the check's own policy: skip (the plan's outcome already holds), run_prerequisite (the plan that makes the
state runs first, then this one again from its start), retry_lift (the lifting arm's block again, from the
next pre-planned rung), abort (a person must look). A PASS with also_observe asks the second question:
seat_close_right follows when the seat came up with the lid.

After a plan has run, its end state is checked (FINAL_CHECKS; the owner, 21 Oct: a lid opening whose right side
did not open the lid ran to its end all the same): the right arm goes to its inspection pose the way ALIX planned it
in that state, the camera is asked, and a NOT stops the run. ALIX's plans check only on the way in.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import re
import threading
import time
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

import numpy as np

from .. import vendor_paths
from .jobs import Cancelled, UserError

PLANS = vendor_paths.ALIX / "Resource" / "plans" / "commode_c" / "hardware"
LIFT_CHECK = vendor_paths.ALIX / "ros" / "orchestration" / "alix_tasks" / "scripts" / "lift_check.py"
CLOCK_S = 0.005                      # Lite's clock (D26) = the plans' track clock = the CanFD period
MODE = "toppra"
STANDOFF_MM = 250                    # the UGV placement the plans were made at, with no offset
RATE_GATE = "per_joint"              # ALIX's lane since 2026-09-21 (cleaning_tracks, run_verified's default)

#: what each task does, in the operator's words (from each plan's config, ALIX's mechanism_tasks/commode_c)
TASKS = {
    "lid_open": dict(
        title="Open the lid", short="LID OPEN", start="lid closed", end="lid open, seat down",
        does="The right arm looks with the wrist camera; if the lid is closed, the left arm lifts it, the camera "
             "checks it is up (up to three re-lifts, each a little further in), then the right arm takes it and "
             "opens it fully. If the seat came up with the lid, it is closed again after (Close the seat)."),
    "seat_close_right": dict(
        title="Close the seat", short="SEAT CLOSE", start="lid and seat open", end="lid open, seat down",
        does="The camera checks the lid is open (else it stops) and the seat is up (else there is nothing to do); "
             "the right arm closes the seat."),
    "seat_open": dict(
        title="Open the seat", short="SEAT OPEN", start="lid open, seat down", end="lid and seat open",
        does="The camera checks the lid is open (else Open the lid runs first) and the seat is down; the left arm "
             "lifts the seat, the camera checks it is up (one re-lift), then the right arm opens it fully."),
    "lid_close_right": dict(
        title="Close the lid", short="LID CLOSE", start="lid open", end="lid and seat closed",
        does="The camera checks the lid is open (else there is nothing to do) and the seat is down (else Close the "
             "seat runs first); the right arm closes the lid."),
}
#: After each plan its end state, checked by the wrist camera (the owner, 21 Oct, after a lid opening whose right side did
#: not open the lid and ran to its end all the same: "add a status check after completion to confirm the final state was
#: reached"). The camera sees the lid and the seat from the right arm's inspection pose alone, so the right arm goes there
#: as ALIX planned it in the state the plan leaves: the way in of the plan that starts in that state (its pole, its hand,
#: its move_to_inspect); the camera is asked, judged as the plans' own checks are (lift_check); the right arm goes back to
#: where the plan ended. A NOT stops the run: a person must look. {plan: (what the camera must see, whose way in looks)}
FINAL_CHECKS = {
    "lid_open": ("lid_open", "seat_open"),                    # lid open, seat down: Open the seat starts there
    "seat_open": ("seat_open", "seat_close_right"),           # lid and seat open: Close the seat starts there
    "seat_close_right": ("seat_closed", "seat_open"),         # lid open, seat down
    "lid_close_right": ("lid_closed", "lid_open"),            # lid and seat closed: Open the lid starts there
}

#: Lite's catalogue rows (docs/region_catalogue_commode_c.csv) and the plan each one runs
ROWS = {"L-T01": "lid_open", "R-T01": "lid_open", "L-T02": "seat_close_right", "R-T02": "seat_close_right",
        "L-T03": "seat_open", "R-T03": "seat_open", "L-T05": "lid_close_right", "R-T09": "lid_close_right"}
#: the commode's state after each, for the 3D and the collision check (Lite's phases)
PHASE_AFTER = {"lid_open": "lid_open", "seat_close_right": "lid_open", "seat_open": "seat_open",
               "lid_close_right": "lid_closed"}

SIDE_OF = {"R": "right", "L": "left"}


@lru_cache(maxsize=1)
def lift_check():
    """ALIX's check policy (what proves an expectation, the failure policies, the re-lift ladder): its own file."""
    spec = importlib.util.spec_from_file_location("alix_lift_check", LIFT_CHECK)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def emit_rows(t, q_rad) -> list:
    """A move's 5 ms lines, ALIX's law (cleaning_tracks.emit_fastpath; the planner's track_emit): the plan's own
    q(t), linearly interpolated onto n = ceil(T / 5 ms) + 1 instants, duration-preserving, in millidegrees."""
    t = np.asarray(t, dtype=float)
    q = np.degrees(np.asarray(q_rad, dtype=float))
    T = float(t[-1])
    n = int(np.ceil(T / CLOCK_S)) + 1
    tt = np.minimum(np.arange(n) * CLOCK_S, T)
    qq = np.column_stack([np.interp(tt, t, q[:, j]) for j in range(q.shape[1])])
    return [[int(v) for v in row] for row in np.round(qq * 1000.0).astype(np.int64)]


def kind_of(joints: list) -> tuple[str, str] | None:
    """('arm' | 'pole' | 'hand', side) of one plan entry (arm_trajectory_sender.joint_controller_key)."""
    first = joints[0]
    if "sliding_plate" in first:
        return "pole", SIDE_OF[first[0]]
    if first.startswith("IRH_"):
        return "hand", "right"
    if first.startswith("ILH_"):
        return "hand", "left"
    if first[:2] in ("R_", "L_"):
        return "arm", SIDE_OF[first[0]]
    return None


@dataclass
class Entry:
    i: int                           # its place in the plan
    index: int                       # the planner's stage index
    stage: str
    kind: str                        # arm | pole | hand
    side: str
    joints: list
    t: np.ndarray                    # seconds from the entry's start
    q: np.ndarray                    # waypoints x joints (radians; the pole in metres)
    pre_actions: list
    rows: list | None = None         # an arm move's 5 ms lines (millidegrees)
    group: int | None = None         # the plan's track group it came from
    note: str = ""                   # "rung 2", "reverse of execute_path" ...

    @property
    def seconds(self) -> float:
        return float(self.t[-1]) if len(self.t) else 0.0

    def first(self) -> list:
        return [float(v) for v in self.q[0]]

    def last(self) -> list:
        return [float(v) for v in self.q[-1]]

    def hand_counts(self, at: str = "last") -> list:
        from alix_dispatch import peripherals

        q = self.last() if at == "last" else self.first()
        return peripherals.counts_from_rad(dict(zip(self.joints, q)), self.side)

    def label(self) -> str:
        return f"{self.side.upper()} {self.kind} {self.stage}" + (f" ({self.note})" if self.note else "")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _entries(data: dict, folder: Path, what: str) -> list:
    """A plan's runnable entries, its strokes' 5 ms tracks checked against their sha256 and their own moves."""
    from alix_dispatch import track

    groups = {}
    for g in data.get("track_groups") or []:
        path = folder / g["track_path"]
        if not path.is_file():
            raise UserError(f"{what}: its track {g['track_path']} is missing.", "Run tools/vendor_from_alix.py again.")
        if _sha256(path) != g["sha256"]:
            raise UserError(f"{what}: {g['track_path']} is not the file the plan was made with (sha256 differs).",
                            "Run tools/vendor_from_alix.py again: nothing was moved.")
        if abs(float(g.get("clock_s") or 0) - CLOCK_S) > 1e-9:
            raise UserError(f"{what}: {g['track_path']} is on a {g.get('clock_s')} s clock, not {CLOCK_S}.")
        rows = [list(map(int, r)) for r in track.parse_track(path)]
        for k in g["stage_indices"]:
            groups[int(k)] = (int(g["group"]), rows)
    out = []
    for pos, st in enumerate(data.get("sub_trajectories") or []):
        joints, wps = st.get("joint_names") or [], st.get("waypoints") or []
        if not joints or not wps:
            continue
        kind = kind_of(joints)
        if kind is None:
            raise UserError(f"{what}: entry {st.get('stage_name')} moves joints Lite cannot route ({joints[:2]}).")
        t = np.array([float(w["time_from_start"]) for w in wps])
        q = np.array([w["positions"] for w in wps], dtype=float)
        e = Entry(len(out), int(st.get("index", pos)), str(st.get("stage_name") or f"entry {pos}"), kind[0], kind[1],
                  list(joints), t, q, list(st.get("pre_actions") or []))
        if e.kind == "arm":
            if pos in groups:
                e.group, e.rows = groups[pos][0], groups[pos][1]
                mine = emit_rows(t, q)
                if mine[0] != e.rows[0] or mine[-1] != e.rows[-1] or len(mine) != len(e.rows):
                    raise UserError(f"{what}: track group {e.group} does not match its own move {e.stage}.")
            else:
                e.rows = emit_rows(t, q)
        out.append(e)
    return out


class Plan:
    """One of ALIX's saved plans, read and checked (never changed)."""

    def __init__(self, name: str, path: Path | None = None):
        if path is None and name not in TASKS:
            raise UserError(f"There is no lid or seat task {name!r}.", f"One of: {', '.join(TASKS)}.")
        self.name = name
        self.path = Path(path) if path is not None else PLANS / f"{name}_{MODE}.json"
        if not self.path.is_file():
            raise UserError(f"ALIX's plan {self.path.name} is not in Lite.", "Run tools/vendor_from_alix.py.")
        self.data = json.loads(self.path.read_text(encoding="utf-8"))
        self.title = TASKS.get(name, {}).get("title", name)
        self.entries = _entries(self.data, self.path.parent, self.path.name)
        self.final_actions = list(self.data.get("final_actions") or [])
        self.made = self.data.get("timestamp_human")

    def sides(self) -> list:
        """The arms it uses, the one that moves first first (ALIX's approach: primary arm first)."""
        out = []
        for e in self.entries:
            if e.side not in out:
                out.append(e.side)
        return out

    def targets(self, at: str = "start") -> dict:
        """{side: {"arm_deg", "pole_mm", "hand"}}: each device's first commanded waypoint (approach.plan_start_targets)
        or its last (approach.plan_end_targets)."""
        out: dict = {}
        seq = self.entries if at == "start" else list(reversed(self.entries))
        for e in seq:
            b = out.setdefault(e.side, {})
            key = {"arm": "arm_deg", "pole": "pole_mm", "hand": "hand"}[e.kind]
            if key in b:
                continue
            if e.kind == "arm":
                b[key] = [float(np.degrees(v)) for v in (e.first() if at == "start" else e.last())]
            elif e.kind == "pole":
                b[key] = round((e.first() if at == "start" else e.last())[0] * 1000.0, 1)
            else:
                b[key] = e.hand_counts("first" if at == "start" else "last")
        return out

    def seconds(self) -> float:
        return round(sum(e.seconds for e in self.entries), 2)

    def checks(self) -> list:
        return [dict(entry=e.stage, **{k: a.get(k) for k in ("stage", "check", "expect", "on_fail", "max_retries",
                                                              "also_observe", "if_observed", "prerequisite")})
                for e in self.entries for a in e.pre_actions if a.get("type") == "check_fixture_state"]

    def rung_block(self, k: int, lifter: str) -> list | None:
        """rungs/<plan>.rung<k>.json: the lifting arm's block for re-lift k (relift.rung_file_block), or None."""
        path = self.path.parent / "rungs" / f"{self.path.stem}.rung{k}.json"
        if not path.is_file():
            return None
        data = json.loads(path.read_text(encoding="utf-8"))
        if not (data.get("task_flags") or {}).get("replan_block"):
            raise UserError(f"{path.name} is not a block-only plan: nothing was moved.")
        block = _entries(data, path.parent, path.name)
        foreign = [e.stage for e in block if e.side != lifter]
        if foreign or not block:
            raise UserError(f"{path.name} carries {foreign or 'no entries'}: not the {lifter} arm's block.")
        for e in block:
            e.note = f"re-lift {k}, pre-planned"
        return block

    def view(self) -> dict:
        t = TASKS.get(self.name, {})
        return dict(name=self.name, title=self.title, short=t.get("short"), does=t.get("does"), start=t.get("start"),
                    end=t.get("end"), file=self.path.name, made=self.made, seconds=self.seconds(), sides=self.sides(),
                    entries=[dict(stage=e.stage, kind=e.kind, side=e.side, seconds=round(e.seconds, 2),
                                  lines=len(e.rows) if e.rows else None, track=e.group is not None) for e in self.entries],
                    checks=self.checks(), standoff_mm=STANDOFF_MM,
                    rungs=sorted(int(m.group(1)) for p in (self.path.parent / "rungs").glob(f"{self.path.stem}.rung*.json")
                                 if (m := re.fullmatch(re.escape(self.path.stem) + r"\.rung(\d+)\.json", p.name))))


@lru_cache(maxsize=8)
def cached(name: str) -> "Plan":
    """A plan read once (ALIX's files do not change while Lite runs: tools/vendor_from_alix.py replaces them)."""
    return Plan(name)


def final_check(name: str) -> dict:
    """Plan ``name``'s end-state check (FINAL_CHECKS): {expect, action (written as the plans write a check), by (the plan
    whose way in looks), look (its entries up to its move_to_inspect: the right arm's pole, hand and move)}."""
    expect, by_name = FINAL_CHECKS[name]
    by = cached(by_name)
    k = next(i for i, e in enumerate(by.entries) if e.kind == "arm")
    look = by.entries[: k + 1]
    if look[-1].stage != "move_to_inspect" or any(e.side != "right" for e in look):
        raise UserError(f"{by.title}'s way in is not the right arm's move to its inspection pose, so {TASKS[name]['title']}'s "
                        "end state cannot be checked: nothing was moved.")
    action = dict(type="check_fixture_state", stage=f"verify_{expect}_after", check="fixture_gap", expect=expect, arm="right",
                  on_fail="abort", max_retries=0, also_observe="", if_observed="", prerequisite="")
    return dict(expect=expect, action=action, by=by, look=look)


def digest(name: str) -> str:
    """The plan file's sha256: what the simulation's identity holds, so another plan is another run."""
    return _sha256(cached(name).path)


def reverse_block(block: list) -> list:
    """The way back along a lift that did not take (seat_open has no pre-planned rung): the lifting arm's moves of
    the block in reverse order, each played backwards on its own timing (ALIX planned a restore with MoveIt)."""
    out = []
    for e in reversed([x for x in block if x.kind == "arm"]):
        T = e.seconds
        t = (T - e.t)[::-1]
        q = e.q[::-1]
        out.append(Entry(-1, e.index, f"back along {e.stage}", "arm", e.side, list(e.joints), t, q, [],
                         rows=emit_rows(t, q), note="re-lift restore, reversed"))
    return out


def assumed(expect: str):
    """The answer a check gets when no camera is asked (Simulation, Hardware sim, the stand-in): as expected."""
    from .gap_check import Answer

    return Answer(success=True, fixture_state=expect, state_confidence=1.0, recommendation="pass",
                  message=f"{expect}: assumed, no camera asked")


# ── the order and the policies (arm_trajectory_sender.py: _send_next, run_plan_actions, _check_client,
#    _observe_also, _after_check_failure, _try_relift, _run_prerequisite, _queue_follow_up)
class Runner:
    def __init__(self, ex, say=None):
        self.ex = ex
        self.say = say or (lambda *_: None)
        self.queue: list = []
        self.k = 0
        self.prereq: dict = {}
        self.done: list = []                         # each plan as it ended: done | skipped
        self.checks: list = []                       # every question asked, and its answer

    def run(self, name: str) -> dict:
        self.queue, self.k = [name], 0
        while self.k < len(self.queue):
            outcome = self._plan(self.queue[self.k])
            if outcome == "prerequisite":
                continue                             # it was put at k: it runs now, this plan again after it
            self.k += 1
        return dict(plans=self.done, checks=self.checks)

    def _plan(self, name: str) -> str:
        plan = Plan(name)
        entries = list(plan.entries)
        state = dict(relifts=0, follow_up="", skipped=False,
                     bridge=set(id(first) for first in (next((e for e in entries if e.kind == "arm" and e.side == s), None)
                                                        for s in plan.sides()) if first is not None))   # after the approach
        self.say(f"PLAN {self.k + 1}/{len(self.queue)}: {plan.title} ({plan.path.name}, {len(entries)} entries, "
                 f"{plan.seconds():.1f} s of motion)")
        self.ex.approach(plan, plan.targets("start"), "start")
        i = 0
        while i < len(entries):
            e = entries[i]
            failed = self._actions(e, state)
            if failed is not None:
                verdict = self._after_failure(plan, entries, i, failed, state)
                if verdict[0] == "retry":
                    start, block = verdict[1], verdict[2]
                    entries[start:i] = block
                    # the lifting arm's next move after the check starts where the ORIGINAL lift ended: bridged
                    after = next((x for x in entries[start + len(block):] if x.kind == "arm" and x.side == block[0].side), None)
                    if after is not None:
                        state["bridge"].add(id(after))
                    i = start
                    continue
                if verdict[0] == "prerequisite":
                    return "prerequisite"
                if verdict[0] == "skip":
                    state["skipped"] = True
                    break
                raise UserError(verdict[1], "A person must look at the commode: nothing else was moved after this.",
                                code="escalate")
            self.ex.check_stop()
            self.ex.move(e, bridge=id(e) in state["bridge"])
            i += 1
        for a in self.final_actions(plan):
            self.ex.hold(float(a.get("seconds") or 0.0), a.get("stage") or "hold_pose")
        if not state["skipped"]:
            if state["follow_up"]:                   # it looks first thing, with its own checks (a NOT there stops it)
                self.say(f"FINAL CHECK {plan.title}: {TASKS[state['follow_up']]['title']} runs next and looks first, with its "
                         "own checks")
            else:
                self._final_check(plan)
        if state["follow_up"]:
            nxt = state["follow_up"]
            if nxt not in TASKS:
                raise UserError(f"{plan.title}: its follow-up {nxt} is not one of Lite's plans: the commode is not left as "
                                "the plan declares.", "A person must look at the commode.", code="escalate")
            self.queue.insert(self.k + 1, nxt)
            self.say(f"FOLLOW-UP: {TASKS[nxt]['title']} runs next, so the plan leaves the state it declares")
        if state["skipped"] and self.k + 1 >= len(self.queue):
            self.ex.approach(plan, plan.targets("end"), "end")
        self.done.append(dict(plan=name, title=plan.title, outcome="skipped" if state["skipped"] else "done",
                              relifts=state["relifts"]))
        return "done"

    def _final_check(self, plan) -> None:
        """The plan's end state, seen by the camera (FINAL_CHECKS): the right arm to its inspection pose the way ALIX
        planned it in that state, the camera asked, the right arm back to where the plan ended. A NOT stops the run there."""
        fc = final_check(plan.name)
        a, expect, by = fc["action"], fc["expect"], fc["by"]
        self.say(f"FINAL CHECK {plan.title}: the right arm to its inspection pose ({by.title}'s way in), the camera checks "
                 f"{expect}")
        self.ex.approach(by, {"right": by.targets("start")["right"]}, "start", words=f"the final check of {plan.title}")
        for e in fc["look"]:
            self.ex.check_stop()
            self.ex.move(e)
        answer = self.ex.ask(a, expect)
        passed = lift_check().evaluate_fixture_check(a, answer)
        self.checks.append(dict(check=a["stage"], expect=expect, state=answer.fixture_state,
                                confidence=round(float(answer.state_confidence), 2), verdict="PASS" if passed else "NO",
                                message=answer.message, record=getattr(answer, "record_path", None), final=True))
        self.say(f"FINAL CHECK {a['stage']}: expect {expect}, the camera says {answer.fixture_state} at "
                 f"{float(answer.state_confidence):.2f} -> {'PASS' if passed else 'NO'}", "info" if passed else "error")
        if not passed:
            raise UserError(f"{plan.title} ran to its end, but the camera does not see {expect.replace('_', ' ')} after it "
                            f"(it reads {answer.fixture_state} at {float(answer.state_confidence):.2f}).",
                            "A person must look at the commode: nothing else was moved after this.", code="escalate")
        self.ex.approach(plan, {"right": plan.targets("end")["right"]}, "end", words=f"where {plan.title} ended")

    @staticmethod
    def final_actions(plan) -> list:
        acts = [a for a in plan.final_actions]
        bad = [a.get("type") for a in acts if a.get("type") != "hold_pose"]
        if bad:
            raise UserError(f"{plan.title} ends with an action Lite does not implement ({bad[0]}): nothing more was run.")
        return acts

    def _actions(self, e, state) -> dict | None:
        """The entry's pre_actions in order; the first check that is not proven, or None (run_plan_actions)."""
        lc = lift_check()
        for a in e.pre_actions:
            kind = a.get("type")
            if kind == "hold_pose":
                self.ex.hold(float(a.get("seconds") or 0.0), a.get("stage") or "hold_pose")
            elif kind == "check_fixture_state":
                label = a.get("stage") or "check_fixture_state"
                mode, why = lc.mode_for(a)
                if mode is None:
                    self.checks.append(dict(check=label, expect=a.get("expect"), verdict="refused", why=why))
                    self.say(f"{label}: {why} — NOT {a.get('expect')} (fail-closed)", "error")
                    return a
                answer = self.ex.ask(a, str(a.get("expect") or ""))
                passed = lc.evaluate_fixture_check(a, answer)
                models = (getattr(answer, "record", None) or {}).get("models") or {}
                row = dict(check=label, expect=a.get("expect"), state=answer.fixture_state,
                           confidence=round(float(answer.state_confidence), 2), verdict="PASS" if passed else "NO",
                           message=answer.message, record=getattr(answer, "record_path", None), models=models)
                self.checks.append(row)
                self.say(f"CHECK {label}: expect {a.get('expect')}, the camera says {answer.fixture_state} at "
                         f"{float(answer.state_confidence):.2f}" + (f" (models: {models.get('fused_state') or 'no state'} from "
                                                                    f"{models.get('state_source') or '-'} at {models.get('confidence', 0):.2f}"
                                                                    + (", CONTRADICTS: uncertain" if models.get("contradiction") else "") + ")"
                                                                    if models else "")
                         + f" -> {'PASS' if passed else 'NO'}", "info" if passed else "warn")
                if passed and a.get("also_observe"):
                    self._observe_also(a, state)
                if not passed:
                    return a
            else:
                self.say(f"{e.stage}: the plan carries an action Lite does not implement ({kind!r}) — refusing (fail-closed)", "error")
                return dict(a, on_fail="abort", stage=a.get("stage") or kind)
        return None

    def _observe_also(self, a, state) -> None:
        what = str(a.get("also_observe") or "")
        follow = str(a.get("if_observed") or "")
        answer = self.ex.ask(dict(a, expect=what), what, observe=True)
        if not answer.success:
            self.say(f"OBSERVED {a.get('stage')}: {what} = unknown ({answer.message[:140]})", "warn")
            if follow:                       # its own check decides: it skips when the part is not up
                state["follow_up"] = follow
            return
        seen = answer.fixture_state == what
        self.checks.append(dict(check=f"{a.get('stage')} (also)", expect=what, state=answer.fixture_state,
                                confidence=round(float(answer.state_confidence), 2), verdict="YES" if seen else "no",
                                message=answer.message, record=getattr(answer, "record_path", None)))
        self.say(f"OBSERVED {a.get('stage')}: {what} = {'YES' if seen else 'no'}")
        if seen and follow:
            state["follow_up"] = follow

    def _after_failure(self, plan, entries, k, failed, state) -> tuple:
        lc = lift_check()
        policy = str(failed.get("on_fail") or "abort")
        label = failed.get("stage") or "check_fixture_state"
        expect = failed.get("expect")
        if policy == "skip":
            self.say(f"{label}: NOT {expect} — this plan's outcome already holds; it ends here as done (on_fail skip)")
            return ("skip",)
        if policy == "run_prerequisite":
            task = str(failed.get("prerequisite") or "")
            budget = max(1, int(failed.get("max_retries", 1) or 1))
            key = (plan.name, task)
            done = self.prereq.get(key, 0)
            if not task or task not in TASKS:
                return ("abort", f"{label}: NOT {expect}, and its prerequisite {task or '(none named)'} is not one of Lite's plans.")
            if done >= budget:
                return ("abort", f"{label}: still NOT {expect} after running {TASKS[task]['title']} {done} time(s).")
            self.prereq[key] = done + 1
            self.queue.insert(self.k, task)
            self.say(f"{label}: NOT {expect} — PREREQUISITE {TASKS[task]['title']} runs now, then {plan.title} again from "
                     f"its start ({done + 1}/{budget})", "warn")
            return ("prerequisite",)
        if policy == "retry_lift":
            budget = int(failed.get("max_retries", 0) or 0)
            if state["relifts"] >= budget:
                return ("abort", f"{label}: still NOT {expect} after {budget} re-lift(s).")
            lifter = "left" if failed.get("arm", "right") == "right" else "right"
            i = k
            while i > 0 and entries[i - 1].side == lifter:
                i -= 1
            if i == k:
                return ("abort", f"{label}: no {lifter}-arm lift block comes before the check.")
            state["relifts"] += 1
            n = state["relifts"]
            offset = lc.retry_offset(failed, n)
            if offset is not None:
                block = plan.rung_block(n, lifter)
                if block is None:
                    return ("abort", f"{label}: re-lift {n} ({lc.describe_offset(offset)}) has no pre-planned rung in Lite, "
                                     "and Lite cannot re-plan it live (ALIX does that with ROS).")
                self.say(f"{label}: NO — re-lift {n}/{budget}: {lc.describe_offset(offset)} on the {lifter} start pose, "
                         f"the pre-planned rung ({len(block)} entries) replaces the lift; the check runs again", "warn")
            else:
                block = reverse_block(entries[i:k]) + [Entry(-1, e.index, e.stage, e.kind, e.side, e.joints, e.t, e.q, [],
                                                              rows=e.rows, group=e.group, note=f"re-lift {n}")
                                                        for e in entries[i:k]]
                self.say(f"{label}: NO — re-lift {n}/{budget}: the {lifter} arm goes back along the lift it made, then "
                         "lifts again (the recorded block); the check runs again", "warn")
            return ("retry", i, block)
        return ("abort", f"{label}: NOT {expect} (on_fail {policy}).")


# ── how each kind of run executes a plan's entries
class _Exec:
    """The common part: the job (progress, STOP), the log."""

    def __init__(self, job, log=None):
        self.job, self._log = job, log
        self.t0 = time.monotonic()

    def say(self, text: str, level: str = "info") -> None:
        self.job.log(text, level)
        if self._log is not None:
            self._log(text)

    def check_stop(self) -> None:
        self.job.check()

    def hold(self, seconds: float, label: str) -> None:
        if seconds > 0:
            self.job.progress(f"{label}: holding {seconds:g} s")
            self.job.sleep(seconds)


class StandInExec(_Exec):
    """The stand-in robot and Simulation: each entry takes its time, checks answered as expected."""

    def __init__(self, job, log=None, scale: float = 1.0, words: str = "the stand-in robot"):
        super().__init__(job, log)
        self.scale, self.words = scale, words

    def approach(self, plan, targets, at, words=None):
        self.say(f"{words or f'{plan.title}: to its {at}'} ({', '.join(f'{s.upper()} arm' for s in targets)}), on {self.words}")

    def move(self, e, bridge=False):
        self.job.progress(f"{e.label()}: {e.seconds:.2f} s")
        self.job.sleep(max(0.0, e.seconds) / self.scale)

    def ask(self, action, expect, observe=False):
        a = assumed(expect) if not observe else assumed("no_" + expect)
        self.say(f"{action.get('stage')}: {expect} assumed (no camera on {self.words})")
        return a


class DryExec(_Exec):
    """Hardware sim: every move checked as the stream would check it, nothing sent. The controllers' SIM mode is
    not known to hold a CanFD pass-through stream (ALIX never ran one in SIM), so nothing is streamed until it is."""

    def __init__(self, job, robot, log=None):
        super().__init__(job, log)
        self.robot = robot

    def approach(self, plan, targets, at, words=None):
        from alix_dispatch import track

        where = words or f"{plan.title}'s {at}"
        for side, tg in targets.items():
            link = self.robot.links.get(side)
            fb = link.read_fresh() if link is not None else {}
            q = fb.get("joints_deg")
            if "arm_deg" in tg and q:
                gap = max(abs(a - b) for a, b in zip(q, tg["arm_deg"]))
                self.say(f"{side.upper()}: Real would MoveJ {gap:.1f} degrees to {where}"
                         + (f", pole {tg['pole_mm']:.0f} mm" if "pole_mm" in tg else "") + (f", hand {tg['hand']}" if "hand" in tg else ""))
            elif "arm_deg" in tg:
                self.say(f"{side.upper()}: not read, so the move to {where} is not measured", "warn")
        _ = track

    def move(self, e, bridge=False):
        from alix_dispatch import track

        self.check_stop()
        if e.kind != "arm":
            self.say(f"{e.label()}: {e.kind} to {e.hand_counts() if e.kind == 'hand' else round(e.last()[0] * 1000)} (not sent)")
            return
        pf = track.canfd_preflight(e.rows, CLOCK_S, RATE_GATE)
        if not pf["ok"]:
            raise UserError(f"{e.label()}: the controller would not take this stream: {'; '.join(pf['reasons'])}.")
        j = max(range(7), key=lambda k: pf["peak_rate_util"][k])
        self.say(f"{e.label()}: {pf['lines']} lines, {e.seconds:.2f} s, peak {pf['peak_rate_deg_s'][j]:.0f} deg/s on J{j + 1} "
                 f"({pf['peak_rate_util'][j] * 100:.0f} % of its rating) — checked, not sent")

    def ask(self, action, expect, observe=False):
        self.say(f"{action.get('stage')}: {expect} assumed (Hardware sim: the arms do not move, the camera is not asked)")
        return assumed(expect) if not observe else assumed("no_" + expect)


class RealExec(_Exec):
    """Real: the arms move. Streams, the pole and the hand, the camera."""

    def __init__(self, job, robot, arms: dict, kit, log=None, live=None):
        super().__init__(job, log)
        self.robot, self.arms, self.kit = robot, arms, kit
        self.live = live or (lambda side, text: None)   # S2's live line of each arm

    def _arm(self, side):
        a = self.arms.get(side)
        if a is None:
            raise UserError(f"The {side.upper()} arm is not connected.", "Connect it, then run the task again.")
        return a

    def approach(self, plan, targets, at, words=None):
        from . import staging

        pct = int(self.robot.speed_pct)
        where = words or f"{plan.title}'s {at}"
        for side, tg in targets.items():
            self.check_stop()
            arm = self._arm(side)
            pole, hand = self.robot._measured(side)
            stage = staging.make_stage(tg.get("pole_mm", pole if pole is not None else 0.0),
                                       tg.get("hand") or [None] * 6, where)
            self.job.progress(f"{side.upper()}: pole and hand to {where}")
            arm.stage(stage)
            if "arm_deg" in tg:
                self.check_stop()
                self.job.progress(f"{side.upper()}: MoveJ to {where} at {pct} %")
                arm.move_to([round(v * 1000) for v in tg["arm_deg"]], speed=pct, restore_pct=self.robot.found_speed(side))
            self.say(f"{side.upper()}: at {where}")

    def move(self, e, bridge=False):
        arm = self._arm(e.side)
        self.job.progress(f"{e.label()}: {e.seconds:.2f} s")
        self.live(e.side, f"{e.stage}{f' ({e.note})' if e.note else ''} // RUNNING")
        if e.kind == "pole":
            arm.pole_to(e.last()[0] * 1000.0)
        elif e.kind == "hand":
            arm.hand_to(e.hand_counts())
        else:
            out = arm.stream(e.rows, e.label(), speed=50, restore_pct=self.robot.found_speed(e.side),
                             bridge_speed=int(self.robot.speed_pct) if bridge else None,
                             stop_hook=self.job.on_cancel, records=self.kit.records_dir("canfd"))
            self.say(f"{e.label()}: {out['lines_sent']}/{out['lines']} lines in {out['seconds']:.2f} s"
                     + (f", bridged {out['bridged_deg']:.1f} degrees first" if out.get("bridged_deg") else ""))

    def ask(self, action, expect, observe=False):
        from . import robot_fk
        from .gap_check import DepthFrame

        self.check_stop()
        self.job.progress(f"{action.get('stage')}: asking the camera ({expect})")
        self.live("right", f"CAMERA // {action.get('stage')}: {expect}?")
        scripted = self.kit.scripted(expect)
        if scripted is not None:                    # a test harness only (LITE_CHECK_SCRIPT): never on the robot
            return scripted
        readings = {}
        for side, link in self.robot.links.items():
            fb = link.read_fresh()
            readings[side] = dict(q_deg=fb.get("joints_deg"), pole_mm=fb.get("pole_mm"), hand=fb.get("hand"))
        try:
            shot = self.kit.camera.shot(wait_s=float(self.kit.gap.p.get("frame_wait_s", 2.0)))
        except UserError as exc:                     # no frame: the check refuses and reads NOT, as ALIX's sender reads it
            frame = None
            self.say(f"camera: {exc}", "error")
        else:
            frames = robot_fk.camera_frames(readings) or {}
            frame = DepthFrame(depth=shot.depth, intrinsics=shot.intrinsics, cam_to_base=frames.get("depth"),
                               hands=robot_fk.hand_positions(readings, self.kit.gap.p["gap_hand_frames"]),
                               colour=shot.colour, aligned=shot.aligned, colour_intrinsics=shot.colour_intrinsics,
                               colour_to_base=frames.get("colour"), ref_to_base=frames.get("ref"),
                               stamp_s=None if shot.camera_ms is None else shot.camera_ms / 1000.0, age_s=shot.age_s)
        answer = self.kit.gap.ask(expect, frame, q_deg=(readings.get("right") or {}).get("q_deg"))
        self.job.set(check=dict(stage=action.get("stage"), expect=expect, state=answer.fixture_state,
                                confidence=round(float(answer.state_confidence), 2), message=answer.message,
                                record=answer.record_path, observe=observe))
        return answer


def camera_check(robot) -> dict:
    """What the wrist camera sees now, as the perception node's status_check answers it: the colour model's state of
    the commode, decided by the depth state when its margin is clear (gap_check.GapCheck.status). Nothing moves; a
    linked right arm gives the camera's pose (else the colour model alone, as the node without TF). With a small
    picture of the colour frame, for the page."""
    from . import robot_fk
    from .gap_check import DepthFrame, png_bytes
    import base64

    k = kit(robot)
    readings = {}
    for side, link in robot.links.items():
        fb = link.read_fresh()
        readings[side] = dict(q_deg=fb.get("joints_deg"), pole_mm=fb.get("pole_mm"), hand=fb.get("hand"))
    with k.camera.using():
        shot = k.camera.shot(wait_s=float(k.gap.p.get("frame_wait_s", 2.0)))
    frames = robot_fk.camera_frames(readings) or {}
    frame = DepthFrame(depth=shot.depth, intrinsics=shot.intrinsics, cam_to_base=frames.get("depth"), colour=shot.colour,
                       aligned=shot.aligned, colour_intrinsics=shot.colour_intrinsics, colour_to_base=frames.get("colour"),
                       ref_to_base=frames.get("ref"))
    q_right = (readings.get("right") or {}).get("q_deg")
    a = k.gap.status(frame, "camera_check", q_deg=q_right)
    picture = None
    if shot.colour is not None:
        small = shot.colour[::2, ::2, ::-1]                       # BGR -> RGB, half size
        picture = "data:image/png;base64," + base64.b64encode(png_bytes(small)).decode("ascii")
    return dict(ok=bool(a.success), state=a.fixture_state, confidence=round(float(a.state_confidence), 3), message=a.message,
                source=(a.record or {}).get("state_source"), colour_state=(a.record or {}).get("colour_state"),
                depth_state=(a.record or {}).get("depth_state"), gate=(a.record or {}).get("gate"), posed=bool(frames),
                pose=k.gap.named_pose(q_right) if q_right is not None else None, record=a.record_path, picture=picture)


# ── the camera and the check, one of each for the server's session (the registration is the session's)
class Kit:
    def __init__(self, robot):
        from .camera_d435 import D435
        from .gap_check import GapCheck

        self.robot = robot
        log = (lambda text: robot.log.hardware("right", "camera", text=text)) if getattr(robot, "log", None) else None
        self.camera = D435(log=log)
        self.gap = GapCheck(records=lambda: self.records_dir("checks"), log=log)
        self._script = None

    def scripted(self, expect: str):
        """The camera's answers from a script, for a TEST HARNESS ONLY (as ALIX's ALIX_CHECK_SERVICE points its
        executives at a scripted answerer): ``LITE_CHECK_SCRIPT`` names a JSON list of states, one per question, in
        order. Never set on the robot: the real checks are the camera's."""
        path = os.environ.get("LITE_CHECK_SCRIPT")
        if not path:
            return None
        from .gap_check import Answer

        with _KIT_LOCK:
            stamp = Path(path).stat().st_mtime_ns
            if self._script is None or self._script[0] != stamp:      # a new script: its answers from the first
                self._script = (stamp, list(json.loads(Path(path).read_text(encoding="utf-8"))))
            state = self._script[1].pop(0) if self._script[1] else None
        return Answer(success=state is not None, fixture_state=state or "not_lifted", state_confidence=1.0 if state else 0.0,
                      message=f"scripted ({path}): {state}", recommendation="pass" if state == expect else "re-clean")

    def records_dir(self, what: str) -> Path | None:
        log = getattr(self.robot, "log", None)
        if log is None:
            return None
        return Path(log.root) / time.strftime("%Y-%m-%d") / what


_KIT_LOCK = threading.Lock()


def kit(robot) -> Kit:
    with _KIT_LOCK:
        k = getattr(robot, "_mechanism_kit", None)
        if k is None:
            k = Kit(robot)
            robot._mechanism_kit = k
        return k


def camera_warning(robot, say) -> dict:
    """The wrist camera before a Real run with a lid or seat task. ALIX's sender starts all the same and reads an
    unanswered check as NOT (fail-closed), and Lite does as it does (the owner, 18 Oct: "use the approach used in
    the alix ws code base"); what it means is said at the start. The test harness's scripted answers stand in for
    it (LITE_CHECK_SCRIPT)."""
    if os.environ.get("LITE_CHECK_SCRIPT"):
        return dict(ok=True, scripted=True)
    have = kit(robot).camera.available()
    if not have["ok"]:
        say(f"The wrist camera cannot be read ({have['why']}): every check will read NOT (fail-closed), as ALIX's sender "
            "reads an unanswered check, and each plan follows its own policy for a NOT.", "warn")
    return have


def run(robot, name: str, mode: str | None = None):
    """A lid or seat task as a job on both arms, in ``mode`` (the robot's own by default): Real moves the arms and
    asks the camera, Hardware sim checks it all and sends nothing, Visual sim and the stand-in take its time."""
    plan = Plan(name)                            # read and checked before anything starts
    mode = mode or robot.mode
    title = f"{plan.title} (ALIX's {plan.path.name})"

    def work(job):
        robot._same_mode(mode, "nothing was moved")
        log = getattr(robot, "log", None)
        say = (lambda text: log.hardware(None, "mechanism", text=text)) if log is not None else None
        linked = {s: robot._real(s) for s in ("left", "right")}
        if mode in ("real", "hwsim") and any(linked.values()) and not all(linked.values()):
            gone = [s.upper() for s, a in linked.items() if a is None]
            raise UserError(f"{plan.title} uses both arms: the {' and '.join(gone)} arm is not linked to its controller.",
                            "Connect both arms, then run it again.")
        if mode == "real" and all(linked.values()):
            k = kit(robot)
            ex = RealExec(job, robot, linked, k, say)
            camera_warning(robot, ex.say)
            with k.camera.using():
                return _finish(Runner(ex, ex.say).run(name), plan, mode)
        if mode == "hwsim" and all(linked.values()):
            ex = DryExec(job, robot, say)
        else:
            from .robot_standin import TIME_SCALE
            ex = StandInExec(job, say, TIME_SCALE, "the stand-in robot" if not any(linked.values()) else "Visual sim")
        return _finish(Runner(ex, ex.say).run(name), plan, mode)

    return robot._start("run", title, work, "both", mode)


def _finish(out: dict, plan, mode) -> dict:
    return dict(out, task=plan.name, title=plan.title, mode=mode, phase=PHASE_AFTER.get(plan.name))


def catalogue() -> list:
    return [Plan(n).view() for n in TASKS]


#: ALIX's inspection poses of the right arm (its MoveIt group states; the owner, 18 Oct: "the inspection poses 1 and 4").
#: The perception core's named_poses.py (Newton, 2026-10-08): "Only pose 4 is used for depth based state inspection ...
#: the other poses are only used for coloured inspection."
INSPECTION_POSES = {
    "inspection_pose_1": dict(title="Inspection pose 1", short="colour viewpoint",
                              what="the colour viewpoint: the camera on its side (rolled -87 degrees), "
                              "for the colour model's inspection of the commode"),
    "inspection_pose_4": dict(title="Inspection pose 4", short="depth viewpoint (the lid and seat checks)",
                              what="the depth viewpoint: the camera looking down at the lid and the "
                              "seat, where ALIX's plans check them (lid_open's move_to_inspect ends here)"),
}


def poses(gap=None) -> list:
    """The inspection poses as the page lists them: name, title, what they are for, the joints (degrees)."""
    from .gap_check import GapCheck

    named = (gap or GapCheck()).named_poses()
    out = []
    for name, info in INSPECTION_POSES.items():
        joints = named.get(name)
        if joints:
            out.append(dict(name=name, arm="right", joints_deg=[round(float(np.degrees(joints[f"R_joint{i}"])), 2) for i in range(1, 8)], **info))
    return out
