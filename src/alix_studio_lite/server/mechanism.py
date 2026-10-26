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
}
#: The closings (Close the seat, Close the lid) are not offered (the owner, 26 Oct: the plans of 25 Oct are not the
#: final ones, removed and never run; the correct ones come later). The rows that ran them are plain arm steps until then.


#: Lite's catalogue rows (docs/region_catalogue_commode_c.csv) and the plan each one runs
ROWS = {"L-T01": "lid_open", "R-T01": "lid_open", "L-T03": "seat_open", "R-T03": "seat_open"}
#: the commode's state after each, for the 3D and the collision check (Lite's phases)
PHASE_AFTER = {"lid_open": "lid_open", "seat_open": "seat_open"}

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


def start_tol_deg() -> float:
    """How far from a stream's first line the arm may be: what one 5 ms line may close (ALIX's run_verified: 80 % of
    the first frame's limit at the per-joint gate, 0.72 degrees)."""
    from alix_dispatch import track

    return 0.8 * track.canfd_frame0_limit_deg(CLOCK_S, RATE_GATE)


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
                                                              "prerequisite")})
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

    def rung_made(self, k: int) -> str | None:
        """When re-lift ``k``'s rung was planned (its file's timestamp), or None."""
        try:
            return json.loads((self.path.parent / "rungs" / f"{self.path.stem}.rung{k}.json").read_text(encoding="utf-8")).get("timestamp_human")
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
                    checks=self.checks(), standoff_mm=STANDOFF_MM, rungs_note=self.rungs_note(),
                    rungs=sorted(int(m.group(1)) for p in (self.path.parent / "rungs").glob(f"{self.path.stem}.rung*.json")
                                 if (m := re.fullmatch(re.escape(self.path.stem) + r"\.rung(\d+)\.json", p.name))))


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


def joined_moves(entries: list, i: int, bridged=()) -> tuple:
    """(``entries[i]`` with the same arm's moves straight after it as ONE stream, how many entries it takes): each
    stream has its own start, stop and settle (24 Oct: 1.6 s a stream). Only a plan's own moves are joined (no re-lift
    or rung move; a "one point" move is the plan's), none asking the camera or holding before it, none bridged, each
    starting on the line the one before ends on (every pair in the four plans: 0 millidegrees apart)."""
    e = entries[i]

    def plain(x) -> bool:
        return x.kind == "arm" and (not x.note or x.note.startswith("one point")) and x.rows is not None

    if not plain(e):
        return e, 1
    group = [e]
    for x in entries[i + 1:]:
        if (not plain(x) or x.side != e.side or x.pre_actions or id(x) in bridged
                or max(abs(a - b) for a, b in zip(group[-1].rows[-1], x.rows[0])) > 0):
            break
        group.append(x)
    if len(group) == 1:
        return e, 1
    t, q, offset = [], [], 0.0
    for x in group:
        t.append(np.asarray(x.t, dtype=float) + offset)
        q.append(np.asarray(x.q, dtype=float))
        offset += x.seconds
    return Entry(e.i, e.index, " + ".join(x.stage for x in group), "arm", e.side, list(e.joints), np.concatenate(t),
                 np.concatenate(q), list(e.pre_actions), rows=[r for x in group for r in x.rows], group=e.group,
                 note=" + ".join(x.note for x in group if x.note)), len(group)


#: A joint this close to the reversed lift's first line (millidegrees) is where the lift ended.
AT_END_MDEG = 50


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
    def __init__(self, ex, say=None, leave=()):
        self.ex = ex
        self.say = say or (lambda *_: None)
        # the arms whose next task's start follows the run (S2: workflow.next_after_tasks; the owner, 24 Oct): the run's
        # last plan takes them to no end of its own (a skipped plan's end, the final check's way to where the plan ended)
        self.leave = set(leave)
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
        state = dict(relifts=0, skipped=False,
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
                answer = self.ex.ask(a, str(a.get("expect") or ""))
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
        self._name, self._feeds, self._hands = "lid_seat", set(), set()

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
        """However the run ends: each arm's kept feed turned off (the controller's push put back), its hand let go, and
        the check records written."""
        for side in sorted(self._feeds):
            try:
                self._arm(side).drop_feed()
            except Exception as exc:                                                # noqa: BLE001
                self.say(f"{side.upper()}: its 5 ms feed was not turned off ({exc})", "warn")
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

    def approach(self, plan, targets, at, words=None):
        """Each arm to ``targets``: both arms' poles and hands at once (the owner, 24 Oct: one after the other took 2 s
        more a run), then each arm's MoveJ at the speed slider's speed, one arm after the other, as before."""
        from . import staging

        pct = int(self.robot.speed_pct)
        where = words or f"{plan.title}'s {at}"
        stages, poles = {}, {}
        for side, tg in targets.items():
            if "hand" not in tg and "pole_mm" not in tg:    # the plan names neither for this arm (the closings): kept as they are
                continue
            self._arm(side)
            pole, hand = self.robot._measured(side)
            if "hand" in tg:
                stages[side] = staging.make_stage(tg.get("pole_mm", pole if pole is not None else 0.0), tg["hand"], where)
            else:
                poles[side] = float(tg["pole_mm"])              # the pole alone; the hand as it is
        self.check_stop()
        if stages or poles:
            self.job.progress(f"{' and '.join(s.upper() for s in sorted(set(stages) | set(poles)))}: pole and hand to {where}")
        failed = {}

        def one(side):
            try:
                if side in stages:
                    self._keep_hand(side)
                    self._arm(side).stage(stages[side])
                else:
                    self._arm(side).pole_to(poles[side])
            except BaseException as exc:                                            # noqa: BLE001 - raised below
                failed[side] = exc

        threads = [threading.Thread(target=one, args=(side,), name=f"lite-stage-{side}", daemon=True)
                   for side in sorted(set(stages) | set(poles))]
        for t in threads:
            t.start()
        for t in threads:
            while t.is_alive():
                t.join(0.1)
        for side in sorted(set(stages) | set(poles)):
            if side in failed:
                raise failed[side]
        for side, tg in targets.items():
            if "arm_deg" in tg:
                self.check_stop()
                self.job.progress(f"{side.upper()}: MoveJ to {where} at {pct} %")
                self._arm(side).move_to([round(v * 1000) for v in tg["arm_deg"]], speed=pct,
                                        restore_pct=self.robot.found_speed(side))
            self.say(f"{side.upper()}: at {where}")

    def move(self, e, bridge=False):
        arm = self._arm(e.side)
        self.job.progress(f"{e.label()}: {e.seconds:.2f} s")
        self.live(e.side, f"{e.stage}{f' ({e.note})' if e.note else ''} // RUNNING")
        if e.kind == "pole":                          # at the speed ALIX's executive sets from its planned time
            arm.pole_to(e.last()[0] * 1000.0, duration_s=e.seconds)
        elif e.kind == "hand":
            self._keep_hand(e.side)
            self.__dict__.setdefault("_hand_results", {})[e.side] = arm.hand_to(e.hand_counts(), duration_s=e.seconds)
        else:
            rows = e.rows
            if e.note == "re-lift restore, reversed":      # from where the arm stands (hold_short)
                link = self.robot.links.get(e.side)
                q = (link.read_fresh() if link is not None else {}).get("joints_deg")
                if q:
                    rows, held = hold_short(e.rows, q)
                    if held:
                        self.say(f"{e.label()}: from where the arm stands: " + ", ".join(
                            f"J{j} held where it stopped, {d:.1f} degrees short of the lift's end" for j, d in held)
                            + ", until the way back comes past it")
            self._keep_feed(e.side)
            out = arm.stream(rows, e.label(), speed=50, restore_pct=self.robot.found_speed(e.side),
                             bridge_speed=int(self.robot.speed_pct) if bridge else None,
                             stop_hook=self.job.on_cancel, records=self.kit.records_dir("canfd"))
            self.say(f"{e.label()}: {out['lines_sent']}/{out['lines']} lines in {out['seconds']:.2f} s"
                     + (f", bridged {out['bridged_deg']:.1f} degrees first" if out.get("bridged_deg") else ""))

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
        self.job.progress(f"{action.get('stage')}: asking the camera ({expect})")
        self.live("right", f"CAMERA // {action.get('stage')}: {expect}?")
        scripted = self.kit.scripted(expect)
        if scripted is not None:                    # a test harness only (LITE_CHECK_SCRIPT): never on the robot
            return scripted
        lc = lift_check()
        first, most, spacing = frames_rule()
        answers, verdict = [], lc.UNDECIDED
        check_id = f"{time.strftime('%Y%m%d_%H%M%S')}_{int(time.time() * 1000) % 1000:03d}_{action.get('stage') or 'check'}"
        for k in range(most):
            if k:
                self.check_stop()
                self.job.sleep(spacing)
            check = dict(check_id=check_id, stage=action.get("stage"), expect=expect, frame=k + 1)
            got = self._frame(expect, check)
            answers.append(got)
            if len(answers) in (first, most):
                verdict = lc.decide(action, answers)
                if verdict != lc.UNDECIDED:
                    break
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
            fb = link.read_fresh()
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
        (which check, which frame) and the camera in its context)."""
        frame, q = self._shot(check)
        return self.kit.gap.ask(expect, frame, q_deg=q)

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
