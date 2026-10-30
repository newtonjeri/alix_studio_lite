"""The lid and the seat: ALIX's mechanism tasks, pre-recorded, run by Lite with no ROS.

The owner, 17-18 Oct: "lid and seat open and close using the camera ... instead of uploading a file, you
stream"; "the mechanism tasks should just be like other tasks, but these are pre-trained or recorded so they
are not subjected to teaching procedure ... tasks that are available to be injected at a point in the
sequence"; "Lite should be ROS 2 independent".

What runs is ALIX's own: commode C's hardware plans (vendor/alix/Resource/plans/commode_c/hardware, planned at a
250 mm standoff with no UGV offset; the openings run on the robot and in Isaac Sim, the closings of 26-28 Oct planned on
the owner's Isaac bringup), with their camera checks, re-lifts and prerequisites. What runs them is ALIX's arm_trajectory_sender.py
(ros/orchestration/alix_tasks/scripts, commit 842c0e59), ported here with ROS replaced:

- each arm move is streamed over the controller's CanFD pass-through, one line every 5 ms, by ALIX's
  ROS-free lane (alix_dispatch ``track.run_verified(channel="canfd")``, as cleaning_tracks.py runs a track:
  the 5 ms capture and its watch, the per-joint rate gate). The lifts and pushes come from the plans' own
  5 ms tracks (.g0/.g1, sha256 checked), the other moves are resampled by the same law
  (cleaning_tracks.emit_fastpath). On the robot ROS's controllers streamed the same moves every 10 ms. A move
  of one point (the revised lid and seat open of 21 Oct: the right arm's start is its pre-start) is given
  SINGLE_POINT_S to get there, from where that arm's previous move ended, as the sender gives it;
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
next pre-planned rung), abort (a person must look). A check's second question (also_observe: the seat under the
lifted lid; if_observed: Close the seat after) is not asked since 26 Oct (the owner: the lid and the hand hide the seat
there, the look costs time, and the seat is judged once, at the end). After a hand lets go, the run goes on only when
the hand did let go (``released``: every channel open, the wrist unloaded before a retract), else it stops for a person.

Each plan's end state is checked by the plan itself (the owner, 26 Oct: the end look belongs in ALIX's task configs, not
in Lite): after the right arm's carry, its hand grasps, it moves to inspection_pose_1 and the camera confirms the end
state before it rests (a check stage, on_fail abort: a NOT stops the run for a person). Until 26 Oct Lite added that
look itself (FINAL_CHECKS, 21 Oct: a lid opening whose right side did not open the lid ran to its end all the same).
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
SINGLE_POINT_S = 0.5                 # a one-point move at t = 0 gets this long (arm_trajectory_sender.SINGLE_POINT_MIN_S)
MODE = "toppra"
STANDOFF_MM = 250                    # the UGV placement the plans were made at, with no offset
RATE_GATE = "per_joint"              # ALIX's lane since 2026-09-21 (cleaning_tracks, run_verified's default)
HAND_RELEASE_SHORT = 150             # a released channel this many counts short of its target did not let go (26 Oct: 449)
HAND_FREE_N = 25.0                   # a hand that let go carries less than this on its wrist (the lifts: 72-80 N; idle 3 N)
HAND_CHANNELS = ("little", "ring", "middle", "index", "thumb bend", "thumb rotation")   # ALIX's hand_units order

#: what each task does, in the operator's words (from each plan's config, ALIX's mechanism_tasks/commode_c)
TASKS = {
    "lid_open": dict(
        title="Open the lid", short="LID OPEN", start="lid closed", end="lid open, seat down",
        does="The right arm looks with the wrist camera; if the lid is closed, the left arm lifts it, the camera "
             "checks it is up (up to three re-lifts, each a little further in), then the right arm takes it and "
             "opens it fully. At the end the camera checks the lid is open and reads the seat: one that came up with "
             "the lid is said (Open the seat then finds it up and skips itself)."),
    "seat_open": dict(
        title="Open the seat", short="SEAT OPEN", start="lid open, seat down", end="lid and seat open",
        does="The camera checks the lid is open (else Open the lid runs first) and the seat is down; the left arm "
             "lifts the seat, the camera checks it is up (one re-lift), then the right arm opens it fully; at the end "
             "the camera checks the lid and the seat are open."),
    # the closings (the owner's compound plans: seat_close 26 Oct, lid_close 28 Oct; trimmed and vendored 28 Oct): the right
    # arm looks from inspection pose 1, the left hooks the part's top edge and pulls it over onto the right hand, which
    # waits under the part's line and then slides out; the part drops; the camera checks it is down from pose 1
    "seat_close": dict(
        title="Close the seat", short="SEAT CLOSE", start="lid and seat open", end="lid open, seat down",
        does="The right arm looks with the wrist camera; if the seat is up, the left arm hooks its top edge and pulls it "
             "over while the right hand waits under its line and catches it; the right hand slides out and the seat drops "
             "onto the body. At the end the camera checks the seat is down (else the run stops for a person)."),
    "lid_close": dict(
        title="Close the lid", short="LID CLOSE", start="lid open, seat down", end="lid and seat down",
        does="The right arm looks with the wrist camera; if the lid is up, the left arm hooks its top edge and pulls it "
             "over while the right hand waits under its line and catches it; the right hand slides out and the lid drops "
             "onto the seat. At the end the camera checks the lid is down (else the run stops for a person)."),
    # no sequence plans of their own (the closings 28 Oct, afternoon; the openings' lid_open_seq and seat_open_seq of 27 Oct
    # retired 29 Oct, when every task took seat_close's order and its plan was made again): a sequence runs the task's plan
    # with its checks before the first lift not asked (Runner start_checks, without_start_checks), the same moves to the
    # line; the right arm's move to its inspection pose stays (the lifted check is taken from there: the lab, 27 Oct 12:56).
    # Separately planned sequence plans came out on another arm configuration about the same hand path (the seat's left
    # start, joint 5 14 deg apart, 28 Oct; seat_open_seq's right push, joint 5 17.6 deg, 27 Oct).
}


def sequence_plan(task: str) -> str:
    """The plan a sequence runs for ``task``: its ``<task>_seq`` plan when Lite holds one (no look before the lift), else
    the task's own (its start checks then not asked: Runner start_checks)."""
    seq = f"{task}_seq"
    return seq if seq in TASKS and (PLANS / f"{seq}_{MODE}.json").is_file() else task


#: Lite's catalogue rows (docs/region_catalogue_commode_c.csv) and the plan each one runs; a row that closes both parts
#: (TASK 15 CLOSE SEAT + LID) runs two plans in this order
ROWS = {"L-T01": "lid_open", "R-T01": "lid_open", "L-T03": "seat_open", "R-T03": "seat_open",
        "L-T02": "seat_close", "R-T02": "seat_close", "L-T05": ("seat_close", "lid_close"), "R-T09": ("seat_close", "lid_close")}
#: the commode's state after each, for the 3D and the collision check (Lite's phases)
PHASE_AFTER = {"lid_open": "lid_open", "seat_open": "seat_open", "seat_close": "lid_open", "lid_close": "lid_closed"}
#: the commode's state as each starts (the approach's two MoveJs are checked in it)
PHASE_BEFORE = {"lid_open": "lid_closed", "seat_open": "lid_open", "seat_close": "seat_open", "lid_close": "lid_open"}
#: each state in the operator's words
PHASE_WORDS = {"lid_closed": "the lid and the seat are down", "lid_open": "the lid is open and the seat down",
               "seat_open": "the lid and the seat are open"}
#: each closing's outcome check (the owner, 30 Oct): the part within the closing allowance of closed counts as closed
#: (perception.yaml gap_closing_max_deg; the camera may look while a soft-close seat is still settling). The openings'
#: own "is it closed?" checks keep the plain rule.
CLOSING_OUTCOME = {"seat_close": "seat_closed", "lid_close": "lid_closed"}


def row_tasks(code: str) -> list:
    """The plans a catalogue row runs, in order (ROWS): [] for a row that runs none."""
    t = ROWS.get(code)
    return [] if t is None else list(t) if isinstance(t, (tuple, list)) else [t]
APPROACH_GRID = 7         #: each arm's MoveJ path sampled at this many points; every pair of the two arms' samples is checked
                          #: (49 states), as their timing against each other is not known

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
    blended_s: float = 0.0           # a joined stream: the stops at its junctions taken out, this much (27 Oct)
    members: list | None = None      # a joined stream: [(stage, first line)] of the moves it is made of

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


def start_tol_deg() -> float:
    """How far from a stream's first line the arm may be: what one 5 ms line may close (ALIX's run_verified: 80 % of
    the first frame's limit at the per-joint gate, 0.72 degrees)."""
    from alix_dispatch import track

    return 0.8 * track.canfd_frame0_limit_deg(CLOCK_S, RATE_GATE)


BRIDGE_MAX_DEG = 3.0      #: a move whose first line the arm has crept from by up to this is bridged first (26 Oct: the left wrist,
                          #: at its 9 A plateau holding the seat up for the 9 s the right arm needs, crept 0.5-1.1 degrees once warm;
                          #: 1.0 was refused at the retract's first line, 17:14); farther, something else is wrong: refused as before
BRIDGE_SPEED_PCT = 40     #: the bridge's MoveJ speed (the owner, 26 Oct: "40 % for that task, because it is a short movement";
                          #: the re-lifts' long bridge keeps the speed slider's)


def _gap_deg(a, b) -> float:
    return float(np.degrees(np.max(np.abs(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))))


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
        note = ""
        if kind[0] == "arm" and len(wps) == 1:
            # ALIX's sender (arm_trajectory_sender, SINGLE_POINT_MIN_S): "A single point at t = 0 asks the controller to
            # BE there now ... give the controller a short interval to close it rather than a step." A stream needs two
            # lines (track.canfd_preflight): the move goes there from where the same arm's previous move ended.
            before = next((x for x in reversed(out) if x.kind == "arm" and x.side == kind[1]), None)
            t = np.array([0.0, float(t[0]) if t[0] > 0 else SINGLE_POINT_S])
            q = np.vstack([before.q[-1] if before is not None else q[0], q[0]])
            note = f"one point: {t[1]:g} s to it"
        e = Entry(len(out), int(st.get("index", pos)), str(st.get("stage_name") or f"entry {pos}"), kind[0], kind[1],
                  list(joints), t, q, list(st.get("pre_actions") or []), note=note)
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
        self.rungs_stem = f"{TASKS.get(name, {}).get('variant_of', name)}_{MODE}"   # a sequence plan's re-lifts: its task's own
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

    def first_move_targets(self, entries=None) -> dict:
        """``targets("start")`` with each arm's pole and hand as they must be when that arm FIRST moves: the last pole
        and hand entries before its first arm entry (the plans open the hand, set the pole and grasp before the arm
        goes). Set together by one step at the approach (the owner, 26 Oct: the plans' hand open, pole, hand re-read,
        MoveJ, pole no-op and grasp took 5-7 s an arm before any arm moved), after which those entries are already met.
        An arm with no pole or hand entry before its first move keeps its start state. Only the entries before the run's
        first check (first_check_at): nothing but the look moves before a check, so a NO wastes no movement (the owner,
        29 Oct); ``entries`` the run's own, a sequence's without the start checks it does not ask (without_start_checks),
        else the plan's."""
        entries = list(self.entries if entries is None else entries)
        entries = entries[:first_check_at(entries)]
        out = {side: dict(tg) for side, tg in self.targets("start").items()}
        for side in out:
            hand_seen = False
            for e in entries:
                if e.side != side:
                    continue
                if e.kind == "arm":
                    break
                if e.kind == "pole":
                    out[side]["pole_mm"] = round(e.last()[0] * 1000.0, 1)
                elif e.kind == "hand" and not hand_seen:                     # the first hand entry (27 Oct)
                    out[side]["hand"], hand_seen = e.hand_counts(), True
        return out

    def seconds(self) -> float:
        return round(sum(e.seconds for e in self.entries), 2)

    def streams(self, start_checks: bool = True) -> list:
        """The arm moves as a run streams them: [(the joined entry, how many entries it takes)] (joined_moves over the
        entries as a task run on its own plays them, or, ``start_checks`` False, as a sequence does: its start checks
        dropped first, so the moves either side of a dropped check join too)."""
        entries = list(self.entries) if start_checks else without_start_checks(list(self.entries))[0]
        out, i = [], 0
        while i < len(entries):
            if entries[i].kind != "arm":
                i += 1
                continue
            joined, used = joined_moves(entries, i)
            out.append((joined, used))
            i += used
        return out

    def streamed(self, start_checks: bool = True) -> dict:
        """What the run's streams make of the plan's arm moves: ``seconds`` the moves take streamed (their stops at the
        junctions taken out), ``blended_s`` how much those stops were, ``streams`` how many streams, ``moves`` how many
        plan moves they carry."""
        st = self.streams(start_checks)
        return dict(seconds=round(sum(j.seconds for j, _ in st), 2), blended_s=round(sum(j.blended_s for j, _ in st), 2),
                    streams=len(st), moves=sum(n for _, n in st))

    def checks(self) -> list:
        return [dict(entry=e.stage, **{k: a.get(k) for k in ("stage", "check", "expect", "on_fail", "max_retries",
                                                              "prerequisite")})
                for e in self.entries for a in e.pre_actions if a.get("type") == "check_fixture_state"]

    def rung_block(self, k: int, lifter: str) -> list | None:
        """rungs/<plan>.rung<k>.json: the lifting arm's block for re-lift k (relift.rung_file_block), or None."""
        path = self.path.parent / "rungs" / f"{self.rungs_stem}.rung{k}.json"
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

    def rung_made(self, k: int) -> str | None:
        """When re-lift ``k``'s rung was planned (its file's timestamp), or None."""
        try:
            return json.loads((self.path.parent / "rungs" / f"{self.rungs_stem}.rung{k}.json").read_text(encoding="utf-8")).get("timestamp_human")
        except (OSError, ValueError):
            return None

    def rungs_note(self) -> str | None:
        """Why this plan's pre-planned re-lifts do not fit it, or None. They chain from where its lift ends (the first
        starts there, each next where the one before ended); re-lifts planned for another version of the plan start
        elsewhere, and a missed lift then stops the run at the first one (21 Oct: Open the lid revised at 17:42, its
        re-lifts of 10 Oct start 3.2 degrees from where its lift ends)."""
        for idx, e in enumerate(self.entries):
            for a in e.pre_actions:
                if a.get("type") != "check_fixture_state" or a.get("on_fail") != "retry_lift":
                    continue
                lifter = "left" if a.get("arm", "right") == "right" else "right"
                ended = next((x for x in reversed(self.entries[:idx]) if x.kind == "arm" and x.side == lifter), None)
                try:
                    first = next((x for x in self.rung_block(1, lifter) or [] if x.kind == "arm"), None)
                except UserError as exc:
                    return str(exc)
                if ended is None or first is None:
                    continue
                gap = _gap_deg(first.first(), ended.last())
                if gap > start_tol_deg():
                    return (f"{self.title}'s pre-planned re-lifts (planned {self.rung_made(1) or 'when, not known'}) start {gap:.1f} "
                            f"degrees from where its lift ends (the plan of {self.made}): they were planned for another version of it, "
                            "so a missed lift stops the run at the first re-lift. Plan the re-lifts again in ALIX with this plan (its "
                            "retry ladder), then vendor them into Lite.")
        return None

    def view(self) -> dict:
        t = TASKS.get(self.name, {})
        return dict(name=self.name, title=self.title, short=t.get("short"), does=t.get("does"), start=t.get("start"),
                    end=t.get("end"), file=self.path.name, made=self.made, seconds=self.seconds(), sides=self.sides(),
                    entries=[dict(stage=e.stage, kind=e.kind, side=e.side, seconds=round(e.seconds, 2),
                                  lines=len(e.rows) if e.rows else None, track=e.group is not None) for e in self.entries],
                    checks=self.checks(), standoff_mm=STANDOFF_MM, rungs_note=self.rungs_note(), streamed=self.streamed(),
                    rungs=sorted(int(m.group(1)) for p in (self.path.parent / "rungs").glob(f"{self.rungs_stem}.rung*.json")
                                 if (m := re.fullmatch(re.escape(self.rungs_stem) + r"\.rung(\d+)\.json", p.name))))


@lru_cache(maxsize=8)
def cached(name: str) -> "Plan":
    """A plan read once (ALIX's files do not change while Lite runs: tools/vendor_from_alix.py replaces them)."""
    return Plan(name)


def digest(name: str) -> str:
    """The plan file's sha256: what the simulation's identity holds, so another plan is another run."""
    return _sha256(cached(name).path)


def reverse_block(block: list, note: str = "re-lift restore, reversed") -> list:
    """The way back along a lift that did not take (seat_open has no pre-planned rung): the lifting arm's moves of
    the block in reverse order, each played backwards on its own timing (ALIX planned a restore with MoveIt). Also
    the final check's way back from its inspection pose (``note``): the way in's own path, the hand as it went."""
    out = []
    for e in reversed([x for x in block if x.kind == "arm"]):
        T = e.seconds
        t = (T - e.t)[::-1]
        q = e.q[::-1]
        out.append(Entry(-1, e.index, f"back along {e.stage}", "arm", e.side, list(e.joints), t, q, [],
                         rows=emit_rows(t, q), note=note))
    return out


JUNCTION_WINDOW_S = 0.3   #: the planner stops at every junction of two moves: slowing this long before it, speeding up this long
                          #: after (the plans of 26-27 Oct: ramps of 0.2-0.4 s); the window Lite re-times (the owner, 27 Oct)
JUNCTION_MIN_DEG_S = 5.0  #: a junction the moves already pass slower than this on their fastest joint is left alone
NO_BLEND_STAGES = ("execute_path",)   #: a carry (the lift, the push) keeps its own start and end: never blended into or out of


def blend_junction(rows: list, j: int, w: int | None = None) -> list:
    """``rows`` (5 ms lines, millidegrees) with the stop at line ``j`` (the first line of the move after) taken out: the
    window of JUNCTION_WINDOW_S each side is re-timed on the very same path (its lines interpolated along it) so the arm
    passes through at the speed it enters the window with, easing to the speed it leaves at (the fastest joint's speed,
    linear across the window); nothing before or after the window changes. Rows untouched when a side of the window is
    missing or the moves already pass under JUNCTION_MIN_DEG_S."""
    import bisect

    w = int(round(JUNCTION_WINDOW_S / CLOCK_S)) if w is None else int(w)
    a, b = max(1, j - w), min(len(rows) - 2, j + w)
    if a >= j or b <= j:
        return rows
    v_in = max(abs(x - y) for x, y in zip(rows[a], rows[a - 1])) / 1000.0 / CLOCK_S
    v_out = max(abs(x - y) for x, y in zip(rows[b + 1], rows[b])) / 1000.0 / CLOCK_S
    if min(v_in, v_out) < JUNCTION_MIN_DEG_S:
        return rows
    win = rows[a:b + 1]
    d = [max(abs(x - y) for x, y in zip(win[k], win[k - 1])) / 1000.0 for k in range(1, len(win))]
    L = sum(d)
    if L <= 0:
        return rows
    cum = [0.0]
    for dk in d:
        cum.append(cum[-1] + dk)
    T = 2.0 * L / (v_in + v_out)
    n = max(2, int(round(T / CLOCK_S)) + 1)
    out = []
    for k in range(n):
        t = T * k / (n - 1)
        s_ = min(max(v_in * t + (v_out - v_in) * t * t / (2.0 * T), 0.0), L)
        i = min(max(bisect.bisect_right(cum, s_) - 1, 0), len(d) - 1)
        f = 0.0 if d[i] <= 0 else min(1.0, (s_ - cum[i]) / d[i])
        out.append([int(round(x + (y - x) * f)) for x, y in zip(win[i], win[i + 1])])
    return rows[:a] + out + rows[b + 1:]


def joined_rows(name: str, i: int, n: int) -> list:
    """The lines of the stream a run (Real or the simulation) made of plan ``name``'s entries ``i`` to ``i + n - 1``
    (joined_moves, its junctions blended): the 3D view's ``mechanism:<task>:<i>:joined:<n>``. The entries after the
    first are taken without their actions (a join never crosses one; a sequence's start checks were dropped before)."""
    from dataclasses import replace

    part = list(cached(name).entries[i:i + n])
    part = part[:1] + [replace(e, pre_actions=[]) for e in part[1:]]
    joined, used = joined_moves(part, 0)
    if used != n:
        raise UserError(f"{name}: entries {i}..{i + n - 1} do not join into one stream (they did when the simulation was made).")
    return joined.rows


def joined_moves(entries: list, i: int, bridged=()) -> tuple:
    """(``entries[i]`` with the same arm's moves straight after it as ONE stream, how many entries it takes): each
    stream has its own start and end (24 Oct: 1.6 s a stream; 26 Oct: about 0.15 s, its end read from the feed). Only a plan's own moves are joined (no re-lift
    or rung move; a "one point" move is the plan's), none asking the camera or holding before it, none bridged, each
    starting on the line the one before ends on (every pair in the four plans: 0 millidegrees apart)."""
    e = entries[i]

    def plain(x) -> bool:
        return x.kind == "arm" and (not x.note or x.note.startswith("one point")) and x.rows is not None

    if not plain(e):
        return e, 1
    group, still = [e], []                      # ``still``: one-point moves to the line the stream already ends on
    for x in entries[i + 1:]:
        if (not plain(x) or x.side != e.side or x.pre_actions or id(x) in bridged
                or max(abs(a - b) for a, b in zip(group[-1].rows[-1], x.rows[0])) > 0):
            break
        if all(r == group[-1].rows[-1] for r in x.rows):   # the planner's one-point move to where the move before ends (26 Oct:
            still.append(x)                                  # seat_open's right move_to_start): its half second of the same line not sent
            continue
        group.append(x)
    if len(group) == 1 and not still:
        return e, 1
    rows = [r for x in group for r in x.rows]
    cuts = []                                   # the junctions, each the first line of the move after; blended last to first
    at = 0
    for x in group[:-1]:
        at += len(x.rows)
        cuts.append(at)
    blended, removed = 0, [0] * len(cuts)        # the lines each junction's blend took out
    for k in range(len(cuts) - 1, -1, -1):      # the stop between two joined moves taken out (the owner, 27 Oct), a carry's never
        if group[k].stage in NO_BLEND_STAGES or group[k + 1].stage in NO_BLEND_STAGES or (group[k].note or group[k + 1].note):
            continue
        before = len(rows)
        rows = blend_junction(rows, cuts[k])
        removed[k] = before - len(rows)
        blended += removed[k]
    members = [(group[0].stage, 0)]             # each move's first line in the stream (a blend straddles its junction: half each side)
    for k, cut in enumerate(cuts):
        members.append((group[k + 1].stage, cut - sum(removed[:k]) - removed[k] // 2))
    t = np.arange(len(rows), dtype=float) * CLOCK_S
    q = np.radians(np.asarray(rows, dtype=float) / 1000.0)
    names = [x.stage for x in group] + [x.stage for x in still]
    return Entry(e.i, e.index, " + ".join(names), "arm", e.side, list(e.joints), t, q, list(e.pre_actions), rows=rows,
                 group=e.group, note=" + ".join(x.note for x in group + still if x.note and not x.note.startswith("one point")),
                 blended_s=round(blended * CLOCK_S, 3), members=members), len(group) + len(still)


#: A joint this close to the reversed lift's first line (millidegrees) is where the lift ended.
AT_END_MDEG = 50

# ── two arms at once inside a plan (the user, 27 Oct night: "the left arm and right arm can move together for some
# canfd tasks, for example moving to the start poses"; ALIX's executive runs a plan's entries one after the other)
LEG_MARGIN_S = 0.3        #: the other arm's moves end this long before a carry's lines begin in the stream they run alongside
LEG_SHIFTS_S = (-0.3, 0.0, 0.3, 0.6)   #: the two arms' clearance checked with one arm this much early or late too (a hand move's real
                          #: time, the two streams' start apart)
LEG_GRID_S = 0.02         #: the clearance check's step
LEG_WAIT_S = 20.0         #: a leg waits this long for its check (started as the plan loads), then runs one arm after the other
LEG_CLEARANCE_MM = 50.0   #: the two arms this far apart throughout (S2's own rule for the two arms, the owner, 22 Oct)


def carries_fixture(e) -> bool:
    """Whether the (joined) move carries the lid or the seat (NO_BLEND_STAGES: the lift, the push)."""
    return e.kind == "arm" and any(st.startswith(NO_BLEND_STAGES) for st in str(e.stage).split(" + "))


def fixture_offset_s(e) -> float:
    """When, from the (joined) move's start, its carry begins: the first line of its member that carries (0 for a carry
    alone, or one not joined)."""
    for stage, line in (e.members or []):
        if stage.startswith(NO_BLEND_STAGES):
            return line * CLOCK_S
    return 0.0


def fixture_phase_at(entries: list, i: int, plan_name: str) -> str | None:
    """The commode's state as entry ``i`` starts: PHASE_BEFORE until the first carry, PHASE_AFTER after the last; None in
    between (the lid or the seat lifted and held: a state the room check has no meshes for)."""
    carries = [k for k, e in enumerate(entries) if carries_fixture(e)]
    done = sum(1 for k in carries if k < i)
    if done == 0:
        return PHASE_BEFORE.get(plan_name)
    if done == len(carries):
        return PHASE_AFTER.get(plan_name)
    return None


@dataclass
class Leg:
    """A stretch of a plan the two arms run at once: ``order`` the (joined) entries in plan order, each with its bridge flag
    and, for a hand release, the arm's next move's stage (the release gate); ``used`` the plan entries it covers (the
    trailing releases not sent among them, ``skipped``); ``phase`` the commode's state meanwhile; ``key`` names it."""
    order: list
    used: int
    phase: str
    skipped: list = field(default_factory=list)
    zero: set = field(default_factory=set)        # ids of the entries that take no time here: each arm's first hand move,
                                                  # set by the approach already (Plan.first_move_targets)

    @property
    def sides(self) -> list:
        return sorted({e.side for e, _, _ in self.order})

    def part(self, side) -> list:
        return [(e, b, n) for e, b, n in self.order if e.side == side]

    def took(self, e) -> float:
        return 0.0 if id(e) in self.zero else float(e.seconds)

    def seconds(self, side) -> float:
        return sum(self.took(e) for e, _, _ in self.part(side))

    @property
    def key(self) -> tuple:
        return tuple((e.side, e.stage, len(e.rows or [])) for e, _, _ in self.order)

    def label(self) -> str:
        return " || ".join(f"{s.upper()} " + " + ".join(e.stage.split(" + ")[-1] if e.kind == "arm" else e.stage for e, _, _ in self.part(s))
                           for s in self.sides)


def two_arm_leg(entries: list, i: int, unsent=(), bridged=(), plan_name: str = "") -> Leg | None:
    """The stretch from entry ``i`` the two arms can run at once, or None (the plan's order stands). The rules, every one
    of them a physical one:
    - only while the commode stands in a known state (fixture_phase_at): never while the lid or the seat is lifted and
      held (the room check has no meshes for it);
    - an arm's entries keep their order; a check or a hold ends the stretch before it (the camera reads both arms
      still); a re-lift's moves run alone;
    - a carry (the lift, the push) stays alone against the other arm: the other arm's moves in the stretch come before
      it in the plan and end LEG_MARGIN_S before its lines begin (the planner moved one arm at a time while the
      mechanism moved; the hand-offs — the left supporting the lid until the right has pushed it over, the left out of
      the seat's ring before the right pushes — are these orderings);
    - a trailing release not sent (trailing_releases) is passed over.
    The stretch is then one of the plan's own moves run alongside another: the two arms' clearance over it is what is
    new, and RealExec checks it (LEG_CLEARANCE_MM, with the arms shifted against each other by LEG_SHIFTS_S) before
    the two go together; not clear, the plan's order stands."""
    phase = fixture_phase_at(entries, i, plan_name)
    if phase is None:
        return None
    zero = set()                                 # each arm's first hand move before its first arm move and the run's first
    for side in {e.side for e in entries}:       # check: the approach set it (Plan.first_move_targets)
        for e in entries[:first_check_at(entries)]:
            if e.side != side:
                continue
            if e.kind == "arm":
                break
            if e.kind == "hand":
                zero.add(id(e))
                break
    order, skipped, clock, carry_side, j = [], [], {}, None, i
    while j < len(entries):
        e = entries[j]
        if (e.pre_actions and j > i) or e.note or (e.kind == "arm" and e.rows is None):
            break                                # (an unsent move carrying a check ends the stretch too: its check is asked)
        if id(e) in unsent:
            skipped.append(e)
            j += 1
            continue
        joined, used = joined_moves(entries, j, bridged) if e.kind == "arm" else (e, 1)
        side = e.side
        took = 0.0 if id(e) in zero else float(joined.seconds)
        if carries_fixture(joined):
            if carry_side is not None:
                break
            t_f = clock.get(side, 0.0) + fixture_offset_s(joined)
            if any(clock[s] + LEG_MARGIN_S > t_f for s in clock if s != side):
                break
            carry_side = side
        elif carry_side is not None and side != carry_side:
            break
        nxt = next((x.stage for x in entries[j + used:] if x.kind == "arm" and x.side == side), None) if e.kind == "hand" else None
        order.append((joined, id(e) in bridged, nxt))
        clock[side] = clock.get(side, 0.0) + took
        j += used
    if len({e.side for e, _, _ in order}) < 2:
        return None
    return Leg(order=order, used=j - i, phase=phase, skipped=skipped, zero=zero)


def leg_states(leg: Leg, side: str, q0: list, hand0: list, pole_mm: float, times, shift_s: float = 0.0, hands: str = "after") -> dict:
    """The arm's pole, hand counts and joints at ``times`` through its part of the leg, started ``shift_s`` late (early when
    negative), the hand as ``hands`` says: 'after' switches to each hand move's counts as it starts, 'before' keeps the
    counts from before it until it ends (the clearance is checked both ways)."""
    q, hand, out_q, out_h = list(q0), list(hand0), [], []
    moves = []                                               # (t0, t1, entry)
    t = shift_s
    for e, _, _ in leg.part(side):
        moves.append((t, t + leg.took(e), e))
        t += leg.took(e)
    for tt in times:
        qq, hh = list(q0), list(hand0)
        for t0, t1, e in moves:
            if tt < t0:
                break
            if e.kind == "arm":
                k = max(0, min(len(e.rows) - 1, int(round((min(tt, t1) - t0) / CLOCK_S))))
                qq = [v / 1000.0 for v in e.rows[k]]
            elif e.kind == "hand":
                target = [int(c) if c is not None else h for c, h in zip(e.hand_counts(), hh)]
                hh = target if (hands == "after" or tt >= t1) else hh
        out_q.append(qq)
        out_h.append(hh)
    return dict(pole=[float(pole_mm)] * len(times), hand=out_h, q=out_q, known=[True] * len(times))


def leg_start_state(plan, entries: list, i: int, side: str) -> tuple:
    """Where the arm stands and what its hand holds as the leg at entry ``i`` begins: the last of its moves before it in
    the plan (the plan's start pose, and the pole and hand its first move needs, before any). (q_deg, hand counts, pole mm)."""
    first = plan.first_move_targets(entries).get(side) or {}
    start = plan.targets("start").get(side) or {}
    q = list(start.get("arm_deg") or [0.0] * 7)
    hand = list(first.get("hand") or start.get("hand") or [1000] * 6)
    pole = float(first.get("pole_mm") if first.get("pole_mm") is not None else start.get("pole_mm", 150.0))
    for e in entries[:i]:
        if e.side != side:
            continue
        if e.kind == "arm" and e.rows:
            q = [v / 1000.0 for v in e.rows[-1]]
        elif e.kind == "hand":
            hand = [int(c) if c is not None else h for c, h in zip(e.hand_counts(), hand)]
        elif e.kind == "pole":
            pole = e.last()[0] * 1000.0
    return q, hand, pole


def plan_legs(plan, entries: list, unsent=(), bridged=()) -> list:
    """Every stretch of the plan the two arms run at once (two_arm_leg), with the entry index each starts at: [(i, Leg)]."""
    out, i = [], 0
    while i < len(entries):
        leg = two_arm_leg(entries, i, unsent, bridged, plan.name)
        if leg is None:
            i += 1
            continue
        out.append((i, leg))
        i += leg.used
    return out


_LEG_CLEAR: dict = {}            # (plan name, leg key) -> {ok, why}: the two arms' clearance over a leg, a pure function of the plan
_LEG_CLEAR_LOCK = threading.Lock()


def _leg_home() -> dict:
    from alix_workspace.collision import RobotCollisionModel

    from . import robot_model

    return robot_model._home(robot_model.fk_model(), RobotCollisionModel.from_file(str(vendor_paths.URDF), tag="visual").index)


def leg_clearance(plan, entries: list, i: int, leg: Leg) -> dict:
    """The two arms against each other over the leg (collision_check.clearance_states, LEG_CLEARANCE_MM): their poses
    from the plan's own lines at LEG_GRID_S, the hands as they are before and as each hand move sets them, one arm
    shifted against the other by each of LEG_SHIFTS_S, over the window up to the carry's lines (leg_window_s). The
    commode and the walls are not checked again here: every move is the planner's own, checked by it; what is new is
    the two arms at the same moment. {'ok', 'why', 'closest_mm'}; kept per plan and leg (_LEG_CLEAR)."""
    from . import collision_check, staging

    key = (plan.name, leg.key)
    with _LEG_CLEAR_LOCK:
        kept = _LEG_CLEAR.get(key)
    if kept is not None:
        return kept
    t0 = time.monotonic()
    try:
        sides = leg.sides
        starts = {s: leg_start_state(plan, entries, i, s) for s in sides}
        home = dict(_leg_home())
        for s in sides:
            q, hand, pole = starts[s]
            home = staging.start_values(s, home, dict(pole_mm=pole, hand=hand, q_deg=q))
        window = leg_window_s(leg)
        later = max(sides, key=lambda s: next((k for k, (e, _, _) in enumerate(leg.order) if e.side == s), 0))   # the side whose part starts later
        scenarios = []
        for shift in LEG_SHIFTS_S:
            times = [k * LEG_GRID_S for k in range(int(round((window + max(shift, 0.0)) / LEG_GRID_S)) + 1)]
            for hands in ("before", "after"):
                arms = {s: leg_states(leg, s, starts[s][0], starts[s][1], starts[s][2], times, shift if s == later else 0.0, hands)
                        for s in sides}
                scenarios.append(dict(times=times, arms=arms, shift=shift, hands=hands))
        outs = collision_check.clearance_states([dict(times=sc["times"], arms=sc["arms"]) for sc in scenarios], home, LEG_CLEARANCE_MM)
        closest, worst = None, None
        for sc, out in zip(scenarios, outs):
            for ev in out.get("events") or []:
                mm = ev.get("exact_mm", ev.get("min_mm"))
                if mm is not None and (closest is None or mm < closest):
                    closest, worst = mm, dict(ev, shift=sc["shift"], hands=sc["hands"])
        if worst is None:
            result = dict(ok=True, closest_mm=None, seconds=round(time.monotonic() - t0, 2),
                          why=f"never within {LEG_CLEARANCE_MM:g} mm over {window:.1f} s, one arm up to {max(LEG_SHIFTS_S):g} s late or "
                              f"{-min(LEG_SHIFTS_S):g} s early")
        else:
            result = dict(ok=False, closest_mm=round(float(closest), 1), seconds=round(time.monotonic() - t0, 2),
                          why=f"the arms would come {closest:.0f} mm apart ({', '.join(worst.get('links') or [])}) at {worst.get('first_t', 0):.2f} s"
                              + (f" with the {later.upper()} arm {abs(worst['shift']):g} s {'late' if worst['shift'] > 0 else 'early'}" if worst.get("shift") else ""))
    except Exception as exc:                                                    # noqa: BLE001 - not clear when it cannot tell
        result = dict(ok=False, closest_mm=None, seconds=round(time.monotonic() - t0, 2), why=f"their clearance could not be checked ({type(exc).__name__}: {exc})")
    with _LEG_CLEAR_LOCK:
        _LEG_CLEAR[key] = result
    return result


def leg_window_s(leg: Leg) -> float:
    """How long the two arms are both to be checked: until the later of the two parts ends, but never past a carry's
    lines (the other arm's moves ended LEG_MARGIN_S before them by construction)."""
    ends = {}
    for side in leg.sides:
        t, cut = 0.0, None
        for e, _, _ in leg.part(side):
            if carries_fixture(e):
                cut = t + fixture_offset_s(e)
                break
            t += leg.took(e)
        ends[side] = cut if cut is not None else t
    return max(ends.values()) + LEG_MARGIN_S


def hold_short(rows: list, now_deg) -> tuple[list, list]:
    """A re-lift restore's lines (``reverse_block``: the way back along a lift that did not take) from where the arm
    stands, on the robot: a joint the contact stopped short of the lift's end (the restore's first line) is held where
    it stands until the way back comes past it, then follows it; the other joints go back as planned. The first line
    is then where the arm is, and no line takes a joint further than it got. The lab, 22 Oct 11:58, 11:59 and 12:00:
    the left hand pressed on the seat and J6 ended 2.4 to 4.8 degrees short of the lift's end, the other joints at it,
    so the restore's first line was that far from the arm and the stream refused it (one 5 ms line may close 0.72
    degrees: ``start_tol_deg``; no line of the lift was within it either). Returns (the lines, the joints held:
    [(joint, degrees short)])."""
    now = [round(float(v) * 1000.0) for v in list(now_deg)[:7]]
    out = [list(r) for r in rows]
    held = []
    for j in range(7):
        d = out[0][j] - now[j]
        if abs(d) <= AT_END_MDEG:
            continue
        held.append((j + 1, round(abs(d) / 1000.0, 2)))
        for r in out:
            r[j] = min(r[j], now[j]) if d > 0 else max(r[j], now[j])
    return out, held


def assumed(expect: str):
    """The answer a check gets when no camera is asked (Simulation, Hardware sim, the stand-in): as expected."""
    from .gap_check import Answer

    return Answer(success=True, fixture_state=expect, state_confidence=1.0, recommendation="pass",
                  message=f"{expect}: assumed, no camera asked")


#: Every camera check on 3 frames, 2 more when they do not decide (the owner, 23 Oct: "for every check we should be
#: using 3 frames ... for a failed classification in 3, we add 2 more frames, a total of 5"), each voting yes / no /
#: nothing on the expectation and ruled by ALIX's own lift_check.decide (24 Oct: the same vote as ALIX's senders,
#: the same result for the same frames). A frame that cannot judge (refused: its window covered, no depth) does not
#: vote; a check left undecided stops the run for a person and never re-lifts (the seat checks of 22-23 Oct read a
#: lifted seat as NOT through a glove and re-lifted).
CANNOT_JUDGE = "cannot_judge"


FRAMES_REUSE_S = 3.0      #: the frames a check took are judged again for the check right after at the same pose, within this
                          #: (the seat's start: lid open, then seat closed, from the same frames; the owner, 26 Oct)
FRESH_READING_S = 0.5     #: an arm's latest reading, on its link's own connection, serves a frame when it is this fresh (else one read now)


def without_start_checks(entries: list) -> tuple:
    """(``entries`` with the camera checks before the plan's first lift taken out, those checks): in a sequence the
    task's start state is the sequence's own doing (the owner, 27 Oct: the lid is closed because the top of the lid was
    just cleaned; the seat is closed because its top was), so Open the lid's "lid closed" and Open the seat's "lid open"
    and "seat closed" are not asked; the lifted checks and the end looks stay. A task run on its own keeps them all."""
    from dataclasses import replace

    first_lift = next((i for i, e in enumerate(entries) if e.kind == "arm" and e.stage == "execute_path"), len(entries))
    out, dropped = [], []
    for i, e in enumerate(entries):
        checks = [a for a in (e.pre_actions or []) if a.get("type") == "check_fixture_state"]
        if i < first_lift and checks:
            dropped.extend(checks)
            e = replace(e, pre_actions=[a for a in e.pre_actions if a.get("type") != "check_fixture_state"])
        out.append(e)
    return out, dropped


def first_check_at(entries: list) -> int:
    """The index of the first of ``entries`` a camera check is asked before (its check_fixture_state pre_action), else
    their count: nothing after it is set early (Plan.first_move_targets)."""
    return next((i for i, e in enumerate(entries) if any(a.get("type") == "check_fixture_state" for a in e.pre_actions or [])),
                len(entries))


def trailing_releases(entries: list, sides) -> set:
    """ids of each of ``sides``' last entry when it is a hand release with no move of that arm after it: S2 sets the
    hand for the arm's next task's start right after (its ``then``), so the plan's final release is not sent (the owner,
    26 Oct: 0.8 s a task). A side whose last entry is a move keeps everything."""
    out = set()
    for side in sides:
        last = next((e for e in reversed(entries) if e.side == side), None)
        if last is not None and last.kind == "hand" and str(last.stage).startswith("hand_release"):
            out.add(id(last))
    return out


def final_rests(entries: list, leave) -> set:
    """ids of the plan's last arm move when it is a ``move_to_rest`` of an arm in ``leave`` (its next task's start follows
    the run: S2's ``then``) with no other arm move after it: not streamed, the arm's move to its next start goes from where
    it stands (28 Oct: every plan ends with the camera arm's rest from inspection pose 1, 1.3 s, and the next start's MoveJ
    went from rest; one MoveJ from the inspection pose does both). The check that move carries (the plan's end look) is
    still asked before it. An arm whose last move is not the plan's last (the left, resting while the right looks) keeps it."""
    last = next((e for e in reversed(entries) if e.kind == "arm"), None)
    if last is not None and last.side in leave and str(last.stage).startswith("move_to_rest") and not last.note:
        return {id(last)}
    return set()


def frames_rule():
    """(the first frames, the most frames, the spacing in seconds): lift_check's CHECK_FRAMES_FIRST, CHECK_FRAMES_MAX,
    CHECK_FRAME_SPACING_S."""
    lc = lift_check()
    return int(lc.CHECK_FRAMES_FIRST), int(lc.CHECK_FRAMES_MAX), float(lc.CHECK_FRAME_SPACING_S)


def frames_said(answers: list) -> str:
    return ", ".join(a.fixture_state if a.success else "refused" for a in answers)


# ── the order and the policies (arm_trajectory_sender.py: _send_next, run_plan_actions, _check_client,
#    _after_check_failure, _try_relift, _run_prerequisite; _observe_also and _queue_follow_up not ported since 26 Oct)
class Runner:
    def __init__(self, ex, say=None, leave=(), hand_set=(), start_checks=True, together=True):
        self.ex = ex
        self.say = say or (lambda *_: None)
        self.together = bool(together)               # the two arms at once where the plan allows it (two_arm_leg)
        # the arms whose next task's start follows the run (S2: workflow.next_after_tasks; the owner, 24 Oct): the run's
        # last plan takes them to no end of its own (a skipped plan's end, the final check's way to where the plan ended)
        self.leave = set(leave)
        self.hand_set = set(hand_set)                # the arms whose next task's start sets the hand: no final release
        self.start_checks = bool(start_checks)       # False in a sequence: the checks before the first lift are not asked
        self.queue: list = []
        self.k = 0
        self.prereq: dict = {}
        self.done: list = []                         # each plan as it ended: done | skipped
        self.checks: list = []                       # every question asked, and its answer

    def run(self, name: str) -> dict:
        self.queue, self.k = [name], 0
        getattr(self.ex, "begin", lambda _name: None)(name)
        try:
            while self.k < len(self.queue):
                outcome = self._plan(self.queue[self.k])
                if outcome == "prerequisite":
                    continue                         # it was put at k: it runs now, this plan again after it
                self.k += 1
        finally:
            getattr(self.ex, "finish", lambda: None)()
        return dict(plans=self.done, checks=self.checks)

    def _plan(self, name: str) -> str:
        plan = Plan(name)
        entries = list(plan.entries)
        if not self.start_checks:
            entries, dropped = without_start_checks(entries)
            for a in dropped:
                self.say(f"{a.get('stage')}: not asked, the sequence's own order says {a.get('expect')} (the owner, 27 Oct)")
        state = dict(relifts=0, skipped=False, closing=CLOSING_OUTCOME.get(name),
                     bridge=set(id(first) for first in (next((e for e in entries if e.kind == "arm" and e.side == s), None)
                                                        for s in plan.sides()) if first is not None))   # after the approach
        self.say(f"PLAN {self.k + 1}/{len(self.queue)}: {plan.title} ({plan.path.name}, {len(entries)} entries, "
                 f"{plan.seconds():.1f} s of motion)")
        rests = final_rests(entries, self.leave)
        unsent = trailing_releases(entries, self.hand_set) | rests
        if self.together and hasattr(self.ex, "prepare_legs"):
            self.ex.prepare_legs(plan, entries, unsent, state["bridge"])
        self.ex.approach(plan, plan.targets("start"), "start", entries=entries)
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
            if id(e) in unsent:
                self.say(f"{e.label()}: not sent, the {e.side.upper()} arm goes to its next task's start from here (28 Oct)" if id(e) in rests
                         else f"{e.label()}: not sent, the {e.side.upper()} hand is set for its next task's start right after")
                i += 1
                continue
            mover = getattr(self.ex, "move_together", None) if self.together else None
            leg = two_arm_leg(entries, i, unsent, state["bridge"], plan.name) if mover is not None else None
            if leg is not None:                          # the two arms at once where the plan allows it (the user, 27 Oct night)
                for x in leg.skipped:
                    self.say(f"{x.label()}: not sent, the {x.side.upper()} arm goes to its next task's start from here (28 Oct)" if id(x) in rests
                             else f"{x.label()}: not sent, the {x.side.upper()} hand is set for its next task's start right after")
                mover(leg)
                i += leg.used
                continue
            joined, used = joined_moves(entries, i, state["bridge"])
            self.ex.move(joined, bridge=id(e) in state["bridge"])
            if joined.kind == "hand" and joined.stage.startswith("hand_release"):      # it let go, or the run stops here
                nxt = next((x for x in entries[i + used:] if x.kind == "arm" and x.side == joined.side), None)
                getattr(self.ex, "released", lambda *_: None)(joined, nxt.stage if nxt is not None else None)
            i += used
        for a in self.final_actions(plan):
            self.ex.hold(float(a.get("seconds") or 0.0), a.get("stage") or "hold_pose")
        if state["skipped"] and self.k + 1 >= len(self.queue):
            ends = {s: t for s, t in plan.targets("end").items() if s not in self.leave}
            if ends:
                self.ex.approach(plan, ends, "end")
        self.done.append(dict(plan=name, title=plan.title, outcome="skipped" if state["skipped"] else "done",
                              relifts=state["relifts"]))
        return "done"

    def _cannot_judge(self, label, expect, answer, lead="", final=False):
        """A check its frames left undecided (FRAMES): a person must look; no plan policy runs, no re-lift (the owner,
        23 Oct: the seat checks read a lifted seat as NOT through the glove, and the re-lift dragged the hand back)."""
        self.checks.append(dict(check=label, expect=expect, state=answer.fixture_state, confidence=0.0, verdict="cannot judge",
                                message=answer.message, record=getattr(answer, "record_path", None), final=final))
        self.say(f"{'FINAL CHECK' if final else 'CHECK'} {label}: expect {expect}, the camera cannot judge it "
                 f"({answer.message}) -> STOP", "error")
        raise UserError(f"{lead}{label}: the camera cannot judge whether it is {str(expect).replace('_', ' ')} "
                        f"({answer.message}).", "A person must look at the commode: nothing else was moved after this.",
                        code="escalate")

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
                closing = state.get("closing") is not None and a.get("expect") == state.get("closing")
                answer = self.ex.ask(dict(a, closing_outcome=True) if closing else a, str(a.get("expect") or ""))
                if not answer.success:
                    self._cannot_judge(label, a.get("expect"), answer)
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
                if not passed:
                    return a
            else:
                self.say(f"{e.stage}: the plan carries an action Lite does not implement ({kind!r}) — refusing (fail-closed)", "error")
                return dict(a, on_fail="abort", stage=a.get("stage") or kind)
        return None

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
                # it starts where the lift before it ended (the rungs chain): one planned for another version of the plan
                # starts elsewhere, and its stream would be refused at its first line; said here, before anything moves
                ended = next((x for x in reversed(entries[i:k]) if x.kind == "arm" and x.side == lifter), None)
                first = next((x for x in block if x.kind == "arm"), None)
                gap = _gap_deg(first.first(), ended.last()) if ended is not None and first is not None else 0.0
                if gap > start_tol_deg():
                    return ("abort", f"{label}: NOT {expect}, and re-lift {n}'s pre-planned rung starts {gap:.1f} degrees from where "
                                     f"the lift before it ended: it was planned for another version of {plan.title} (the re-lifts "
                                     f"{plan.rung_made(n) or 'of another date'}, the plan {plan.made}). Plan the re-lifts again in "
                                     "ALIX with this plan, then vendor them into Lite.")
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

    def begin(self, name: str) -> None:
        """A run of plan ``name`` (and what follows it) starts."""

    def finish(self) -> None:
        """The run ends, however it ends."""

    def hold(self, seconds: float, label: str) -> None:
        if seconds > 0:
            self.job.progress(f"{label}: holding {seconds:g} s")
            self.job.sleep(seconds)

    def move_together(self, leg: "Leg") -> None:
        """A leg the two arms could run at once: here one move after the other, in the plan's order (the stand-in and
        Hardware sim; the simulation models the two at once itself)."""
        for e, bridge, nxt in leg.order:
            self.move(e, bridge=bridge)
            if e.kind == "hand" and str(e.stage).startswith("hand_release"):
                getattr(self, "released", lambda *_: None)(e, nxt)


class StandInExec(_Exec):
    """The stand-in robot and Simulation: each entry takes its time, checks answered as expected."""

    def __init__(self, job, log=None, scale: float = 1.0, words: str = "the stand-in robot"):
        super().__init__(job, log)
        self.scale, self.words = scale, words

    def approach(self, plan, targets, at, words=None, entries=None):
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

    def approach(self, plan, targets, at, words=None, entries=None):
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

    def __init__(self, job, robot, arms: dict, kit, log=None, live=None, room=None, feeds_after=False):
        super().__init__(job, log)
        self.robot, self.arms, self.kit = robot, arms, kit
        self.live = live or (lambda side, text: None)   # S2's live line of each arm
        self.feeds_after = bool(feeds_after)            # the caller turns the arms' feeds off after the run (S2: in its next pole
        self._name, self._feeds, self._hands = "lid_seat", set(), set()   # and hand step's travel), not finish()
        self._settled_s = 0.0                           # the longest settle of the streams since the last one started (hold)
        self._unsettled: dict = {}                      # side -> (last line, label) of a stream whose end is not yet waited for
        self._legs: dict = {}                           # leg key -> its clearance (leg_clearance), filled by prepare_legs' thread
        self._frames, self._taken = None, []            # (when, [(frame, q)]) of the last check, for the next at the same pose
        self.room = room                                # home, scene, fid, cubicle_mm: the collision check's world (S2 gives it)

    def _arm(self, side):
        a = self.arms.get(side)
        if a is None:
            raise UserError(f"The {side.upper()} arm is not connected.", "Connect it, then run the task again.")
        return a

    def begin(self, name: str) -> None:
        """The wrist camera opened at once, beside the arms' first moves, so its exposure settles while they move (24
        Oct: opened at the first check, with its 1 s settle, 1.2 s of every run). One that cannot open says so at the
        first check, as before."""
        self._name = name
        camera = self.kit.camera

        def open_it():
            try:
                camera.open()
            except Exception:                                                       # noqa: BLE001 - the check says it
                pass

        threading.Thread(target=open_it, name="lite-camera-open", daemon=True).start()

    def finish(self) -> None:
        """However the run ends: each arm's kept feed turned off (the controller's push put back), both at once (26 Oct:
        one after the other, 0.6 s each), its hand let go, and the check records written. With ``feeds_after`` the
        feeds stay on for the caller to turn off in its next step's pole travel (27 Oct: 0.6 to 0.9 s a task with both
        arms standing); robot_real.run_workflow turns one off itself before any Program. An arm whose last stream's end
        was not yet waited for is waited for first: nothing moves on from a plan while an arm still settles."""
        try:
            self._settle_all()
        except Exception as exc:                                                    # noqa: BLE001 - the run is ending anyway
            self.say(f"a stream's end could not be read as the run ended ({exc})", "warn")

        def off(side):
            try:
                self._arm(side).drop_feed()
            except Exception as exc:                                                # noqa: BLE001
                self.say(f"{side.upper()}: its 5 ms feed was not turned off ({exc})", "warn")

        threads = [threading.Thread(target=off, args=(side,), name=f"lite-feed-off-{side}", daemon=True)
                   for side in ([] if self.feeds_after else sorted(self._feeds))]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        for side in sorted(self._hands):
            try:
                self._arm(side).drop_hand()
            except Exception:                                                       # noqa: BLE001 - nothing held on it
                pass
        self._feeds.clear()
        self._hands.clear()
        self.kit.gap.flush()

    def _keep_feed(self, side) -> None:
        """The arm's 5 ms feed on for the rest of the run, from its first move (robot_real.keep_feed)."""
        if side not in self._feeds:
            self._arm(side).keep_feed(f"{side.upper()}_{self._name}", records=self.kit.records_dir("canfd"))
            self._feeds.add(side)

    def _keep_hand(self, side) -> None:
        """The arm's hand opened once for the run (robot_real.keep_hand); one that cannot be is opened by each move."""
        if side not in self._hands:
            self._hands.add(side)
            if not self._arm(side).keep_hand():
                self.say(f"{side.upper()}: its hand could not be opened for the run: each hand move opens it", "warn")

    def approach(self, plan, targets, at, words=None, entries=None):
        """Each arm to ``targets``: both arms' poles and hands at once (the owner, 24 Oct: one after the other took 2 s
        more a run), each arm's pole and hand together in Lite's own step (the hand moving while the pole travels, as
        a Program's own are set; the owner, 26 Oct) — at a plan's start to the state the arm needs when it first moves
        (``Plan.first_move_targets`` of the run's ``entries``: those before its first check), so the plan's own pole and
        hand entries before that move are already met — then each arm's MoveJ at the speed slider's speed, one arm after
        the other, not sent when the arm is already there."""
        pct = int(self.robot.speed_pct)
        where = words or f"{plan.title}'s {at}"
        self._frames = None
        self._settle_all()                                    # nothing is staged or moved while an arm still settles
        if at == "start":
            first = plan.first_move_targets(entries)
            targets = {side: dict(tg, **{k: v for k, v in first.get(side, {}).items() if k in tg}) for side, tg in targets.items()}
        steps = {}
        for side, tg in targets.items():
            if "hand" not in tg and "pole_mm" not in tg:    # the plan names neither for this arm: kept as they are
                continue
            self._arm(side)
            pole, hand = self.robot._measured(side)
            mm = float(tg["pole_mm"]) if "pole_mm" in tg else (pole if pole is not None else 0.0)
            steps[side] = (mm, [None if v is None else int(v) for v in tg["hand"]] if "hand" in tg else [None] * 6)
        self.check_stop()
        moves = {side: list(tg["arm_deg"]) for side, tg in targets.items() if "arm_deg" in tg}
        clear = {}                                            # both MoveJs at once when their paths are clear: checked meanwhile
        room = getattr(self, "room", None)
        if len(moves) == 2 and room is not None:
            threading.Thread(target=lambda: clear.update(self._paths_clear(plan, at, moves, steps)), name="lite-approach-check",
                             daemon=True).start()
        if steps:
            self.job.progress(f"{' and '.join(s.upper() for s in sorted(steps))}: pole and hand to {where}")
        failed = {}

        def one(side):
            try:
                arm = self._arm(side)
                arm.drop_hand()                                   # the step's own hand session; the plan's takes it over
                mm, hand = steps[side]
                feed = None                                       # its 5 ms feed on while its pole travels (27 Oct; before: after the
                if at == "start" and hasattr(self, "_feeds") and side not in self._feeds:    # pair, 0.5 s on the slower arm)
                    records = self.kit.records_dir("canfd")
                    feed = dict(on=f"{side.upper()}_{self._name}", records=None if records is None else str(records))
                out = arm.pole_and_hand(mm, hand, stop_hook=self.job.on_cancel, **({"feed": feed} if feed else {}))
                if feed is not None and out.get("feed"):
                    self._feeds.add(side)
                if not out.get("pole_ok") or not out.get("hand_ok"):
                    raise UserError(f"The {side.upper()} arm was not staged for {where}: pole {out.get('pole_mm')} mm of {mm:.0f}, "
                                    f"hand {out.get('hand')}. Nothing else ran.", "Check the pole and the hand, then try again.")
            except BaseException as exc:                                            # noqa: BLE001 - raised below
                failed[side] = exc

        threads = [threading.Thread(target=one, args=(side,), name=f"lite-stage-{side}", daemon=True)
                   for side in sorted(steps)]
        for t in threads:
            t.start()
        for t in threads:
            while t.is_alive():
                t.join(0.1)
        for side in sorted(steps):
            if side in failed:
                raise failed[side]

        def movej(side):
            self._arm(side).move_to([round(v * 1000) for v in moves[side]], speed=pct, restore_pct=self.robot.found_speed(side))

        together = False
        if len(moves) == 2 and room is not None:
            t_end = time.monotonic() + 20.0
            while not clear and time.monotonic() < t_end:  # the check ran while the poles and hands moved
                time.sleep(0.02)
            together = bool(clear.get("ok"))
            if not together:
                self.say(f"MoveJs to {where} one after the other: {clear.get('why') or 'the paths were not checked in time'}", "warn")
        if together:                                          # the owner, 26 Oct: 1.6-3.2 s a task
            self.check_stop()
            self.job.progress(f"LEFT and RIGHT: MoveJ to {where} at {pct} %, both at once (their paths checked clear)")
            failed.clear()

            def one_move(side):
                try:
                    movej(side)
                except BaseException as exc:                                        # noqa: BLE001 - raised below
                    failed[side] = exc

            threads = [threading.Thread(target=one_move, args=(side,), name=f"lite-movej-{side}", daemon=True) for side in sorted(moves)]
            for t in threads:
                t.start()
            for t in threads:
                while t.is_alive():
                    t.join(0.1)
            for side in sorted(moves):
                if side in failed:
                    raise failed[side]
            for side in targets:
                self.say(f"{side.upper()}: at {where}")
            return
        for side, tg in targets.items():
            if "arm_deg" in tg:
                self.check_stop()
                self.job.progress(f"{side.upper()}: MoveJ to {where} at {pct} %")
                movej(side)
            self.say(f"{side.upper()}: at {where}")

    def _paths_clear(self, plan, at: str, moves: dict, steps: dict) -> dict:
        """Whether the two arms' MoveJs to ``moves`` can go at once: each arm's straight joint-space path from where it
        stands, sampled APPROACH_GRID times, every pair of the two arms' samples checked against each other, the commode
        in the task's state then (PHASE_BEFORE / PHASE_AFTER), the walls and the bin, with the poles and hands the step
        sets (``collision_check.check_states``, S2's own check). {ok, why}; never raises (not clear when it cannot tell)."""
        try:
            from . import collision_check

            q0 = {}
            for side in moves:
                link = self.robot.links.get(side)
                fb = self._reading(link) if link is not None else {}
                if not fb.get("joints_deg"):
                    return dict(ok=False, why=f"the {side.upper()} arm's joints were not read")
                q0[side] = [float(v) for v in fb["joints_deg"]]
            n = APPROACH_GRID
            sides = sorted(moves)
            grid = [(i, j) for i in range(n) for j in range(n)]
            arms = {}
            for k, side in enumerate(sides):
                mm, hand = steps.get(side, (None, None))
                if mm is None or hand is None or any(v is None for v in hand):
                    pole, now = self.robot._measured(side)
                    mm = pole if mm is None else mm
                    hand = [h if h is not None else (int(now[i]) if now else 1000) for i, h in enumerate(hand or [None] * 6)]
                path = [[a + (b - a) * (t / (n - 1)) for a, b in zip(q0[side], moves[side])] for t in range(n)]
                arms[side] = dict(pole=[float(mm)] * len(grid), hand=[list(hand)] * len(grid), q=[path[g[k]] for g in grid],
                                  known=[True] * len(grid))
            phase = (PHASE_BEFORE if at == "start" else PHASE_AFTER).get(plan.name, "lid_closed")
            times = [0.02 * k for k in range(len(grid))]
            out = collision_check.check_states(times, arms, self.room["home"], self.room["scene"], self.room["fid"], [phase] * len(grid),
                                               int(self.room.get("cubicle_mm") or 1400))
            if out.get("ok"):
                return dict(ok=True, why=f"{len(grid)} states clear in {out.get('seconds')} s")
            first = out.get("first") or {}
            return dict(ok=False, why=f"the paths would meet: {', '.join(first.get('what') or [])} ({len(out.get('events') or [])} event(s))")
        except Exception as exc:                                                    # noqa: BLE001
            return dict(ok=False, why=f"the paths could not be checked ({type(exc).__name__}: {exc})")

    def prepare_legs(self, plan, entries: list, unsent=(), bridged=()) -> None:
        """The two arms' clearance over each leg of the plan (leg_clearance), worked out in a thread of its own while the
        arms approach the plan's start; move_together waits for it (LEG_WAIT_S)."""
        legs = plan_legs(plan, entries, unsent, bridged)
        if not legs:
            return
        self.say(f"{plan.title}: {len(legs)} stretch(es) the two arms can run at once: " + "; ".join(leg.label() for _, leg in legs)
                 + " (their clearance is checked first)")

        def work():
            for i, leg in legs:
                self._legs[leg.key] = leg_clearance(plan, entries, i, leg)

        threading.Thread(target=work, name="lite-legs-check", daemon=True).start()

    def move_together(self, leg: "Leg") -> None:
        """A leg of the plan on both arms at once (two_arm_leg), when their clearance over it is checked clear
        (prepare_legs, LEG_CLEARANCE_MM): each arm's moves in order on a thread of its own, the two started together.
        Not checked in time, or not clear: one move after the other in the plan's order, the reason said. A move that
        fails stops the run once the other arm's move has ended (the arms stand at their lines)."""
        from .jobs import Cancelled

        t_end = time.monotonic() + LEG_WAIT_S
        while leg.key not in self._legs and time.monotonic() < t_end:
            self.check_stop()
            time.sleep(0.05)
        verdict = self._legs.get(leg.key) or dict(ok=False, why=f"their clearance was not checked within {LEG_WAIT_S:g} s")
        if not verdict.get("ok"):
            self.say(f"{leg.label()}: one arm after the other: {verdict.get('why')}", "warn")
            return super().move_together(leg)
        self.job.progress(f"both arms at once: {leg.label()}")
        self.say(f"both arms at once: {leg.label()} ({verdict.get('why')})")
        failed, barrier = {}, threading.Barrier(len(leg.sides))

        def run(side):
            try:
                barrier.wait(timeout=10)
                for e, bridge, nxt in leg.part(side):
                    self.move(e, bridge=bridge)
                    if e.kind == "hand" and str(e.stage).startswith("hand_release"):
                        self.released(e, nxt)
            except BaseException as exc:                                            # noqa: BLE001 - raised below
                failed[side] = exc

        threads = [threading.Thread(target=run, args=(side,), name=f"lite-leg-{side}", daemon=True) for side in leg.sides]
        for t in threads:
            t.start()
        for t in threads:
            while t.is_alive():
                t.join(0.1)
        if failed:
            first = next((exc for exc in failed.values() if isinstance(exc, Cancelled)), None) or failed[sorted(failed)[0]]
            raise first

    def _settle(self, side) -> None:
        """A stream's end waited for now (robot_real.settle_end), when this arm's last stream left it: before this arm's
        next stream or pole move, a check, a hold, the plan's end. Not before the other arm's move or this arm's hand
        move (27 Oct night: 1.6 s a run waited for by moves that did not need the arm settled). The time it took is
        what a settle hold then counts (``_settled_s``): the longest of the arms' settles since the last stream started (28
        Oct runs: the hold took the last settle made, the right's inspect move's 0.01 s, after the left's lift had settled
        0.52 s a moment before, and held 0.49 s more at every lifted check)."""
        pending = self._unsettled.pop(side, None)
        if pending is None:
            return
        last, label = pending
        settle = self._arm(side).settle_end(list(last), label) or {}
        waited = float(settle.get("waited_s") or 0.0) if settle.get("by") == "feed" and settle.get("at_rest") else 0.0
        self._settled_s = max(self._settled_s, waited)
        self.say(f"{label}: end {settle.get('waited_s', 0) * 1000:.0f} ms" + ("" if settle.get("at_rest") else " (NOT at rest)"),
                 "info" if settle.get("at_rest") else "warn")

    def _settle_all(self) -> None:
        for side in sorted(self._unsettled):
            self._settle(side)

    def hold(self, seconds: float, label: str) -> None:
        """A plan's hold. One that lets the mechanism settle before a check (``settle_before_check``, 0.5 s in the lid
        and seat plans) counts the time the arm's own settle already took: the stream before it ends only when the
        feed shows the arm within END_TOL_DEG of its last line and still (robot_real._stream_end, 0.37 to 0.65 s after
        a lift), which is the rest the hold was there to give (27 Oct night: 0.5 s twice a run on top of it)."""
        self._frames = None                           # what the camera saw before the hold is not what it sees after
        self._settle_all()                            # the hold counts from the arms standing still
        settled, self._settled_s = self._settled_s, 0.0   # counted once: the next hold counts the settles after it
        if str(label).startswith("settle") and settled > 0:
            rest = max(0.0, float(seconds) - settled)
            self.say(f"{label} {seconds:g} s: the arm's own settle took {settled:.2f} s of it"
                     + (f", holding {rest:.2f} s more" if rest > 0.005 else ", nothing more to wait"))
            seconds = rest
        super().hold(seconds, label)

    def move(self, e, bridge=False):
        self._frames = None                           # the frames of the check before: taken before this move
        arm = self._arm(e.side)
        self.job.progress(f"{e.label()}: {e.seconds:.2f} s")
        self.live(e.side, f"{e.stage}{f' ({e.note})' if e.note else ''} // RUNNING")
        if e.kind == "pole":                          # at the speed ALIX's executive sets from its planned time
            self._settle(e.side)                      # the pole never moves while its arm does (the manufacturer, 27 Oct)
            arm.pole_to(e.last()[0] * 1000.0, duration_s=e.seconds)
        elif e.kind == "hand":
            self._keep_hand(e.side)
            self.__dict__.setdefault("_hand_results", {})[e.side] = arm.hand_to(e.hand_counts(), duration_s=e.seconds)
        else:
            rows = e.rows
            bridge_speed = int(self.robot.speed_pct) if bridge else None     # a re-lift's long bridge: the slider's speed
            self._settle(e.side)                           # its last stream's end, if not yet waited for: its first line is judged from there
            self._keep_feed(e.side)                        # its frames give the arm's joints (joints_now): no reading before the stream
            if e.note == "re-lift restore, reversed":      # from where the arm stands (hold_short)
                q = self._joints(e.side)
                if q:
                    rows, held = hold_short(e.rows, q)
                    if held:
                        self.say(f"{e.label()}: from where the arm stands: " + ", ".join(
                            f"J{j} held where it stopped, {d:.1f} degrees short of the lift's end" for j, d in held)
                            + ", until the way back comes past it")
            elif not bridge:                               # crept from its first line while it held (the seat): bridged at 40 %
                q = self._joints(e.side)
                gap = max(abs(float(a) - b / 1000.0) for a, b in zip(q, rows[0])) if q and len(q) == len(rows[0]) else None
                if gap is not None and gap > start_tol_deg():
                    if gap <= BRIDGE_MAX_DEG:
                        bridge_speed = BRIDGE_SPEED_PCT
                        self.say(f"{e.label()}: the arm is {gap:.2f} degrees from its first line (it crept while it held): "
                                 f"a MoveJ at {BRIDGE_SPEED_PCT} % closes that first")
                    else:
                        self.say(f"{e.label()}: the arm is {gap:.1f} degrees from its first line, more than the {BRIDGE_MAX_DEG:g} "
                                 "Lite bridges: the stream is refused at its first line", "warn")
            if carries_fixture(e):                         # the settle hold after a carry counts the settles from this carry on
                self._settled_s = 0.0
            out = arm.stream(rows, e.label(), speed=50, restore_pct=self.robot.found_speed(e.side), bridge_speed=bridge_speed,
                             stop_hook=self.job.on_cancel, records=self.kit.records_dir("canfd"), settle=False)
            self._unsettled[e.side] = (list(rows[-1]), e.label())     # its end waited for when something needs it (_settle)
            self.say(f"{e.label()}: {out['lines_sent']}/{out['lines']} lines in {out['seconds']:.2f} s"
                     + (f" ({e.blended_s:.2f} s of stops at its junctions taken out)" if getattr(e, "blended_s", 0) else "")
                     + (f", bridged {out['bridged_deg']:.1f} degrees first" if out.get("bridged_deg") else ""))

    def _joints(self, side) -> list | None:
        """The arm's joints (deg) now: from its kept feed's latest frame (robot_real.joints_now, no controller call), else
        one reading of its link."""
        arm = self._arm(side)
        now = getattr(arm, "joints_now", None)
        if now is not None:
            try:
                return (now() or {}).get("joints_deg")
            except (AttributeError, TypeError):            # a stand-in arm without it
                pass
        link = self.robot.links.get(side)
        return (link.read_fresh() if link is not None else {}).get("joints_deg")

    def released(self, e, next_stage):
        """After a hand let go (a plan's hand_release): the run goes on only when it did, else it stops here for a
        person (the owner, 26 Oct: a gate that prevents the next move, not a note after it). Every commanded channel
        within HAND_RELEASE_SHORT of its target; and when that arm's next move is a retract or its rest, the wrist's
        load back under HAND_FREE_N (the 26 Oct seat run: the ring finger stayed at 544 of 993 under the seat, 56 N
        still on the hand, and only the stream's start gate kept the retract from dragging it, as the 23 Oct way back
        did at 160 N)."""
        out = getattr(self, "_hand_results", {}).get(e.side) or {}
        target = [int(v) for v in e.hand_counts()]
        after = [int(v) for v in (out.get("after") or [])]
        short = [(k + 1, t - a) for k, (t, a) in enumerate(zip(target, after)) if t - a > HAND_RELEASE_SHORT]
        if short:
            raise UserError(f"The {e.side.upper()} hand did not let go: " + ", ".join(
                f"channel {k} ({HAND_CHANNELS[k - 1]}) {d} counts short of {target[k - 1]}" for k, d in short)
                + f" (it reads {after}).", "A person must look at the commode and the hand: nothing else was moved after this.",
                code="escalate")
        if next_stage and (next_stage.startswith("move_to_retract") or next_stage.startswith("move_to_rest")):
            load = self._arm(e.side).hand_load()
            if load is not None and load["load_n"] > HAND_FREE_N:
                raise UserError(f"The {e.side.upper()} hand is still loaded after it let go: {load['load_n']:.0f} N on its "
                                f"wrist (free is under {HAND_FREE_N:.0f} N), so {next_stage} was not started.",
                                "A person must look at the commode and the hand: nothing else was moved after this.", code="escalate")
            self.say(f"{e.side.upper()} hand free: " + ("no wrist reading" if load is None else f"{load['load_n']:.0f} N on the wrist"))

    def ask(self, action, expect):
        """The camera's answer on its frames, ruled by ALIX's lift_check.decide: after 3 frames, and after 5 when the
        3 do not settle it. PASS -> the first frame that voted yes; FAIL -> the first that voted no; UNDECIDED ->
        CANNOT_JUDGE (success False), which stops the run for a person. Each frame keeps its own check record."""
        from dataclasses import replace

        from .gap_check import Answer

        self.check_stop()
        self._settle_all()                          # the camera reads the scene with both arms still
        self.job.progress(f"{action.get('stage')}: asking the camera ({expect})")
        self.live("right", f"CAMERA // {action.get('stage')}: {expect}?")
        scripted = self.kit.scripted(expect)
        if scripted is not None:                    # a test harness only (LITE_CHECK_SCRIPT): never on the robot
            return scripted
        lc = lift_check()
        first, most, spacing = frames_rule()
        reuse = self._reusable_frames()
        self._taken = []
        answers, verdict = [], lc.UNDECIDED
        check_id = f"{time.strftime('%Y%m%d_%H%M%S')}_{int(time.time() * 1000) % 1000:03d}_{action.get('stage') or 'check'}"
        for k in range(most):
            check = dict(check_id=check_id, stage=action.get("stage"), expect=expect, frame=k + 1,
                         closing=bool(action.get("closing_outcome")))
            if k < len(reuse):                      # the check before, at this pose: its frames judged for this expectation
                frame, q = reuse[k]
                got = self._judge(expect, frame, q, dict(check, reused=True))
            else:
                if k > len(reuse):                  # a pause between the camera's frames (none before the first new one)
                    self.check_stop()
                    self.job.sleep(spacing)
                got = self._frame(expect, check)
            answers.append(got)
            if len(answers) >= first:               # decided as soon as the frames allow (26 Oct: two that agree), one more at a time
                verdict = lc.decide(action, answers)
                if verdict != lc.UNDECIDED:
                    break
        kept = list(reuse) + list(self._taken)
        self._frames = (time.monotonic(), kept) if kept else None
        votes = [lc.frame_vote(action, x) for x in answers]
        seen = frames_said(answers)
        tally = (f"{votes.count('yes')} yes, {votes.count('no')} no, "
                 f"{len(votes) - votes.count('yes') - votes.count('no')} no vote")
        if verdict in (lc.PASS, lc.FAIL):
            decided = answers[votes.index("yes" if verdict == lc.PASS else "no")]
            answer = replace(decided, message=f"{len(answers)} frames ({seen}; {tally}): {decided.message}")
        else:
            last = answers[-1]
            answer = Answer(success=False, fixture_state=CANNOT_JUDGE, record=last.record, record_path=last.record_path,
                            message=f"no decision in {len(answers)} frames ({seen}; {tally}); the last: {last.message}")
        self.job.set(check=dict(stage=action.get("stage"), expect=expect, state=answer.fixture_state,
                                confidence=round(float(answer.state_confidence), 2), message=answer.message,
                                record=answer.record_path))
        self._summary(check_id, action, expect, answers, answer)
        return answer

    def _summary(self, check_id, action, expect, answers, answer) -> None:
        """``<check_id>_frames.json`` beside the frames' records: each frame's record and answer, and what they decided
        (a dataset tells a check's frames apart from its decision). Each record is named relative to this file (the
        owner, 24 Oct: the lab computer's absolute paths did not open once the logs were pulled). Never fails a check."""
        root = self.kit.records_dir("checks")
        if root is None:
            return

        def beside(path):
            if not path:
                return None
            try:
                return Path(path).resolve().relative_to(root.resolve()).as_posix()
            except ValueError:
                return str(path)

        try:
            root.mkdir(parents=True, exist_ok=True)
            frames = [dict(record=beside(a.record_path), success=bool(a.success), state=a.fixture_state,
                           confidence=round(float(a.state_confidence), 3)) for a in answers]
            (root / f"{check_id}_frames.json").write_text(json.dumps(dict(
                check_id=check_id, stage=action.get("stage"), expect=expect, decided=bool(answer.success),
                state=answer.fixture_state, confidence=round(float(answer.state_confidence), 3), message=answer.message,
                frames=frames), indent=2), encoding="utf-8")
        except OSError:
            pass

    def _shot(self, check=None):
        """One shot: both arms read, the camera's frame, posed by FK (None when the camera gave no frame), and the right
        arm's joints (the named pose the check is at)."""
        from . import robot_fk
        from .gap_check import DepthFrame

        readings = {}
        for side, link in self.robot.links.items():
            fb = self._reading(link)
            readings[side] = dict(q_deg=fb.get("joints_deg"), pole_mm=fb.get("pole_mm"), hand=fb.get("hand"))
        context = dict(arms=readings, check=check, camera=getattr(self.kit.camera, "device", None))
        try:
            shot = self.kit.camera.shot(wait_s=float(self.kit.gap.p.get("frame_wait_s", 2.0)))
        except UserError as exc:                     # no frame: this one cannot judge, and does not vote
            frame = None
            self.say(f"camera: {exc}", "error")
        else:
            frames = robot_fk.camera_frames(readings) or {}
            frame = DepthFrame(depth=shot.depth, intrinsics=shot.intrinsics, cam_to_base=frames.get("depth"),
                               hands=robot_fk.hand_positions(readings, self.kit.gap.p["gap_hand_frames"]),
                               colour=shot.colour, aligned=shot.aligned, colour_intrinsics=shot.colour_intrinsics,
                               colour_to_base=frames.get("colour"), ref_to_base=frames.get("ref"),
                               stamp_s=None if shot.camera_ms is None else shot.camera_ms / 1000.0, age_s=shot.age_s,
                               context=context)
        return frame, (readings.get("right") or {}).get("q_deg")

    def _frame(self, expect, check=None):
        """One frame: the gap check's answer (its record written, a training row: both arms' readings, ``check``
        (which check, which frame) and the camera in its context). The frame is kept for a check right after."""
        frame, q = self._shot(check)
        if frame is not None:
            self._taken.append((frame, q))
        return self._judge(expect, frame, q, check)

    def _judge(self, expect, frame, q, check=None):
        """The gap check's answer on ``frame`` for ``expect``; a frame judged again (the check before's) carries this
        check in its record. A closing's outcome check (``check["closing"]``) asks with the closing allowance."""
        if frame is not None and check is not None and getattr(frame, "context", None) is not None:
            from dataclasses import replace

            frame = replace(frame, context=dict(frame.context, check=check))
        return self.kit.gap.ask(expect, frame, q_deg=q, check_id=(check or {}).get("check_id"),
                                closing=bool((check or {}).get("closing")))

    def _reusable_frames(self) -> list:
        """The frames the check just before took, when nothing has moved since (move, approach and hold forget them)
        and they are under FRAMES_REUSE_S old: judged again for a new expectation before any new frame is taken."""
        kept = self._frames
        if not kept or time.monotonic() - kept[0] > FRAMES_REUSE_S:
            return []
        return list(kept[1])

    def _reading(self, link) -> dict:
        """The arm's latest reading when its link reads on a connection of its own and it is within FRESH_READING_S,
        else one read now (a round of the SDK's reads, 0.1-0.2 s an arm: 26 Oct, three frames a check, two arms)."""
        v = link.view() if hasattr(link, "view") else {}
        fb = v.get("feedback") or {}
        age = (9.0 if fb.get("age_s") is None else float(fb["age_s"])) + float(v.get("view_age_s") or 0.0)
        if v.get("reader") and fb.get("joints_deg") and age <= FRESH_READING_S:
            return fb
        return link.read_fresh()

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
                       ref_to_base=frames.get("ref"),
                       context=dict(arms=readings, check=dict(stage="camera_check"), camera=getattr(k.camera, "device", None)))
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


def warm_up(robot) -> str:
    """The wrist camera's perception made ready as Lite starts with the real arms (its core, the colour model loaded and
    run once): on 24 Oct the first check after a start took 17.5 s, 12 s of it loading them. Never raises."""
    try:
        return kit(robot).gap.warm()
    except Exception as exc:                                                        # noqa: BLE001
        return f"not warmed ({exc})"


def camera_required(robot, stage: str | None = None) -> dict:
    """The wrist camera before a Real run with a lid or seat task: without it nothing runs (the owner, 23 Oct). Until
    then Lite did as ALIX's sender does (the owner, 18 Oct): it started all the same and read every check as NOT, so a
    lid's "is it closed?" read NOT and Open the lid ended as done, the lid closed (the lab, 23 Oct 14:38). ``stage``:
    the S2 stage about to start (a run already under way stops there). The test harness's scripted answers stand in
    for it (LITE_CHECK_SCRIPT, while its file is there)."""
    script = os.environ.get("LITE_CHECK_SCRIPT")
    if script and Path(script).is_file():
        return dict(ok=True, scripted=True)
    have = kit(robot).camera.available()
    if not have["ok"]:
        why = f"the wrist camera cannot be read ({have['why']}), and the lid and seat tasks need it to check each state"
        raise UserError(f"{stage} STOPPED: {why}." if stage else f"Not run: {why}. Nothing moved.",
                        "Plug the D435 into this computer, then run it again.", code="camera")
    return have


JOINT_HOT_C = 65.0        #: no run starts with a joint at or above this (the owner, 26 Oct)


def joints_cool(robot, stage: str | None = None) -> dict:
    """Every linked arm's joints under JOINT_HOT_C before a Real run with a lid or seat task (the owner, 26 Oct: the
    right arm's joint 3 ran hot, 80 °C at a high current, and stopped following its commands; a gate, not a note). The
    temperatures are the arms' status readings (robot_link: rm_get_current_joint_temperature with every reading); an
    arm whose controller does not answer them passes (an emulated arm). ``stage``: the S2 stage about to start."""
    hot, temps = [], {}
    for side, link in (getattr(robot, "links", None) or {}).items():
        fb = ((link.view() if link is not None else None) or {}).get("feedback") or {}
        temps[side] = fb.get("temps_c")
        hot += [f"the {side.upper()} arm's joint {j + 1} at {v:.0f} °C" for j, v in enumerate(temps[side] or [])
                if v is not None and v >= JOINT_HOT_C]
    if hot:
        why = ", ".join(hot) + f" (a run needs every joint under {JOINT_HOT_C:.0f} °C)"
        raise UserError(f"{stage} STOPPED: {why}." if stage else f"Not run: {why}. Nothing moved.",
                        "Let the arm cool and check that joint on its pendant, then run it again.", code="hot")
    return temps


def run(robot, name: str, mode: str | None = None):
    """A lid or seat task as a job on both arms, in ``mode`` (the robot's own by default): Real moves the arms and
    asks the camera, Hardware sim checks it all and sends nothing, Visual sim and the stand-in take its time."""
    plan = Plan(name)                            # read and checked before anything starts
    mode = mode or robot.mode
    title = f"{plan.title} (ALIX's {plan.path.name})"
    if mode == "real" and all(robot._real(s) for s in ("left", "right")):
        camera_required(robot)                   # refused at once, before the job starts
        joints_cool(robot)

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
            from . import contact_sim

            ok = contact_sim.gate(name, job)         # the hands' contact forces (the owner, 29 Oct): refused from 40 N
            if ok and ok.get("accepted") and say:
                say(f"{plan.title}: contact check over the limit ({ok['peak_n']:g} N)," + contact_sim.accepted_words(ok)[len(" Over the limit,"):])
            k = kit(robot)
            camera_required(robot)                   # again: it may have been unplugged since
            joints_cool(robot)
            ex = RealExec(job, robot, linked, k, say)
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
    return [Plan(n).view() for n in TASKS if "variant_of" not in TASKS[n]]    # the tasks; their sequence plans are theirs


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
