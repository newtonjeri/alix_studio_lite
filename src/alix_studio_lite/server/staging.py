"""Pole and hand staging: the state a trajectory must be executed in.

A trajectory's joint values only mean something with the arm's pole at the height it was taught
at and the hand in the pose it was taught with. The pole carries the whole arm (``L_base_link``
is 160 mm lower at a 140 mm pole than at the 300 mm home pole), and the fingers change what
touches the toilet. ALIX found the same ("Seeding is MANDATORY", ``tools/dual_arm_verdicts.py``).
So every execution in Lite stages first, in this order, and moves the arm only when both arrived:

    pole  ->  hand  ->  move to the trajectory's start  ->  the trajectory

That holds in the 3D simulation (Visual sim), in Hardware sim and on the real robot. A
trajectory without a known stage, or taught on the other arm, never runs.

Units and laws are ALIX's own, imported, not copied:

- pole: the controller's millimetres are the sliding plate's travel
  (``{L,R}_sliding_plate_joint``, metres), 0-315 mm, arrival within 5 mm, travel time
  ``alix_dispatch.peripherals.lift_travel_s`` (1.5 s creep + distance / 0.059 m/s up, 0.097 down);
- hand: six counts in SDK order (little, ring, middle, index, thumb bend, thumb rotate),
  1000 = open, 0 = closed, ``None`` = hold; radians by ALIX's ``hand_units.hw_to_rad``; the time
  a move takes is span / 373 counts/s + 0.115 s (``alix_dispatch.backends.SdkBackend``);
- "hand <pct> % open" (the flow sheets' own spec): ``taught_tracks.fingers_pct_counts``.
"""
from __future__ import annotations

import base64
import math
import re
from dataclasses import dataclass

import numpy as np

from .jobs import UserError

POLE_MAX_MM = 315.0
POLE_TOL_MM = 5.0                     # the pole controller's goal tolerance (LIFT_TOL_M)
HAND_TOL = 30                         # counts: a channel within this of its target has arrived
SIDE_PREFIX = {"left": "L", "right": "R"}
CHANNELS = ("little", "ring", "middle", "index", "thumb bend", "thumb rotate")
PRESETS = {
    "OPEN": [1000, 1000, 1000, 1000, 1000, 1000],
    "GRIP": [0, 0, 0, 0, 150, 1000],
}
TRANSIT_FRACTION = 0.30               # the move to the start, as a fraction of each joint's rating
CYCLE_S = 0.005


def _alix():
    from .. import vendor_paths

    vendor_paths.activate()
    from alix_dispatch import backends, peripherals

    return peripherals, backends


def hand_units():
    return _alix()[1]._load_hand_units()


@dataclass(frozen=True)
class Stage:
    pole_mm: float
    hand: tuple                       # six counts 0-1000, or None to hold that channel
    source: str = ""                  # where the values came from, said in plain words

    def as_dict(self) -> dict:
        return {"pole_mm": self.pole_mm, "hand": list(self.hand), "source": self.source,
                "text": describe(self), "hand_text": hand_words(self.hand)}


def parse_hand(value) -> tuple:
    """Six counts from a list, a preset name (OPEN, GRIP) or the sheets' "100% open" text."""
    if isinstance(value, str):
        v = value.strip().upper()
        if v in PRESETS:
            return tuple(PRESETS[v])
        m = re.fullmatch(r"(\d{1,3})\s*%\s*OPEN(\s*\(FIRST 4 FINGERS\))?", v)
        if m:
            if int(m.group(1)) > 100:                                     # ALIX's helper would exit the server thread
                raise UserError(f"The hand cannot be {m.group(1)} % open.", "Use 0 to 100 % open, Open or Grip.")
            _alix()                                                       # the vendored path first
            import taught_tracks as tt                                    # noqa: PLC0415

            return tuple(tt.fingers_pct_counts(int(m.group(1))))
        raise UserError(f"Unknown hand setting {value!r}.", "Use six values from 0 to 1000, Open or Grip.")
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise UserError("The hand needs six values: little, ring, middle, index, thumb bend, thumb rotate.")
    out = []
    for c in value:
        if isinstance(c, float) and not math.isfinite(c):
            raise UserError("Hand values are whole numbers from 0 to 1000.")
        if c is None or (isinstance(c, (int, float)) and c < 0):
            out.append(None)                                           # hold, as the SDK's -1
            continue
        try:
            c = int(round(float(c)))
        except (TypeError, ValueError, OverflowError):
            raise UserError("Hand values are whole numbers from 0 to 1000.") from None
        if not 0 <= c <= 1000:
            raise UserError("Hand values are 0 to 1000 (1000 = open, 0 = closed).")
        out.append(c)
    if all(c is None for c in out):
        raise UserError("The hand needs at least one channel set.")
    return tuple(out)


def make_stage(pole_mm, hand, source: str = "") -> Stage:
    try:
        pole = float(pole_mm)
    except (TypeError, ValueError, OverflowError):
        raise UserError("The pole height is a number of millimetres (0 to 315).") from None
    if not math.isfinite(pole) or not 0.0 <= pole <= POLE_MAX_MM:
        raise UserError("The pole height must be 0 to 315 mm.")
    return Stage(round(pole, 1), parse_hand(hand), source)


def stage_from(d) -> Stage | None:
    """A stage from stored metadata ({pole_mm, hand, source}), or None when unknown."""
    if not isinstance(d, dict) or d.get("pole_mm") is None or d.get("hand") is None:
        return None
    try:
        return make_stage(d["pole_mm"], d["hand"], d.get("source", ""))
    except UserError:
        return None


def same_stage(a, b) -> bool:
    """Whether two stages (a Stage or its dict) are the same pole height and hand pose; an
    unknown stage is never the same as anything."""
    sa, sb = (s if isinstance(s, Stage) else stage_from(s) for s in (a, b))
    return sa is not None and sb is not None and (sa.pole_mm, sa.hand) == (sb.pole_mm, sb.hand)


def hand_words(hand) -> str:
    """A hand pose in words: "grip", "open", "fingers 50 % open, thumb held", or its six counts."""
    h = list(hand)
    if h == PRESETS["OPEN"]:
        return "open"
    if h == PRESETS["GRIP"]:
        return "grip"
    if h[4] is None and h[5] is None and len(set(h[:4])) == 1:
        return f"fingers {h[0] / 10:g} % open, thumb held"
    return ", ".join("hold" if c is None else str(c) for c in h)


def describe(stage: Stage) -> str:
    words = hand_words(stage.hand)
    return f"pole {stage.pole_mm:g} mm, {words if words.startswith('fingers') else 'hand ' + words}"


def same_hand(a, b, tol: int = HAND_TOL) -> bool:
    """Every channel the target sets is within ``tol`` counts (held channels are not judged)."""
    return all(t is None or (r is not None and abs(int(r) - int(t)) <= tol) for r, t in zip(a, b))


GRIP_CLOSE_MIN = 100                  # counts: a channel this far closed from open (1000) is holding


def hand_holds(measured, target, tol: int = HAND_TOL) -> bool:
    """Whether the hand is in ``target``'s pose as the robot sees it: every set channel within
    ``tol``, or, closing onto something (a grip round the brush), stopped short of its target
    after closing at least GRIP_CLOSE_MIN from open. The fingers of a grip stop on the tool, not
    at 0 (the hardware, 14 Oct: Grip read back 842, 762, 754, 765, 714, 992). A held channel
    (None) is anything; an opening channel that stopped short is not in the pose."""
    if measured is None:
        return False
    for m, t in zip(measured, target):
        if t is None:
            continue
        if m is None:
            return False
        if abs(m - t) <= tol or t < m <= 1000 - GRIP_CLOSE_MIN:
            continue
        return False
    return True


def pole_travel_s(from_mm: float, to_mm: float) -> float:
    d = abs(float(to_mm) - float(from_mm)) / 1000.0
    if d < POLE_TOL_MM / 1000.0:
        return 0.0
    return float(_alix()[0].lift_travel_s(d, up=to_mm > from_mm))


def hand_move_s(from_counts, to_counts) -> float:
    b = _alix()[1].SdkBackend
    span = max((abs(int(t) - int(f)) for f, t in zip(from_counts, to_counts) if t is not None and f is not None),
               default=0)
    return 0.0 if span == 0 else span / b.HAND_COUNTS_PER_S + b.HAND_LATENCY_S


def joint_values(side: str, pole_mm: float | None = None, hand=None) -> dict:
    """URDF joint values (metres, radians) for a pole height and a hand pose; mimics follow in FK."""
    out = {}
    if pole_mm is not None:
        out[f"{SIDE_PREFIX[side]}_sliding_plate_joint"] = float(pole_mm) / 1000.0
    if hand is not None:
        hu = hand_units()
        for ch, c in enumerate(hand):
            if c is not None:
                out[hu.joint_name(side, ch)] = hu.hw_to_rad(float(c), ch)
    return out


def home_stage(side: str, home: dict) -> tuple[float, list]:
    """The pole (mm) and hand counts of a pose (the standing pose, unless an earlier execution
    moved them: see start_values)."""
    hu = hand_units()
    pole = 1000.0 * float(home.get(f"{SIDE_PREFIX[side]}_sliding_plate_joint", 0.3))
    hand = [int(round(hu.rad_to_hw(float(home.get(hu.joint_name(side, ch), 0.0)), ch))) for ch in range(6)]
    return pole, hand


def start_values(side: str, home: dict, start: dict | None = None) -> dict:
    """The joint values an execution starts from: the standing pose, with this arm's pole, hand
    and joints where an earlier execution left them (``start``: {pole_mm, hand, q_deg}, as an
    execution's ``end``). A sequence chains its steps this way."""
    out = dict(home)
    if start:
        out.update(joint_values(side, start.get("pole_mm"), start.get("hand")))
        if start.get("q_deg") is not None:
            names = [f"{SIDE_PREFIX[side]}_joint{i}" for i in range(1, 8)]
            out.update(zip(names, np.radians(np.asarray(start["q_deg"], dtype=float)).tolist()))
    return out


def execution_times(side: str, q_deg: np.ndarray, stage: Stage, home: dict, rating_deg_s) -> dict:
    """The clock of an execution from ``home`` (its start values): the pole to the stage height,
    the hand to the stage pose, the joint move to the first line, then (N - 1) x 5 ms."""
    s = SIDE_PREFIX[side]
    pole0, hand0 = home_stage(side, home)
    hand1 = [c0 if c is None else int(c) for c0, c in zip(hand0, stage.hand)]
    arm0 = np.degrees([float(home.get(f"{s}_joint{i}", 0.0)) for i in range(1, 8)])
    t_pole = pole_travel_s(pole0, stage.pole_mm)
    t_hand = hand_move_s(hand0, hand1)
    rating = np.asarray(rating_deg_s, dtype=float)
    t_move = max(1.0, float(np.max(np.abs(q_deg[0] - arm0) / (TRANSIT_FRACTION * rating))) + 0.5)
    traj_t0 = t_pole + t_hand + t_move
    return dict(pole0=pole0, hand0=hand0, hand1=hand1, arm0=arm0, t_pole=t_pole, t_hand=t_hand, t_move=t_move,
                traj_t0=traj_t0, traj_s=(q_deg.shape[0] - 1) * CYCLE_S,
                end=dict(pole_mm=float(stage.pole_mm), hand=hand1, q_deg=[round(float(v), 4) for v in q_deg[-1]]))


def staged_links(side: str, links) -> list[str]:
    """Every link the pole moves: the sliding plate and everything below it (base, arm, hand)."""
    from . import robot_model

    model = robot_model.fk_model()
    s = SIDE_PREFIX[side]
    start = next(j.child for j in model.joints_by_child.values() if j.name == f"{s}_sliding_plate_joint")
    below, stack = {start}, [start]
    while stack:
        parent = stack.pop()
        for j in model.joints_by_child.values():
            if j.parent == parent:
                below.add(j.child)
                stack.append(j.child)
    return [ln for ln in links if ln in below]


def execution_timeline(side: str, md: np.ndarray, stage: Stage, *, home: dict, links,
                       rating_deg_s, every: int = 4, step_s: float = 0.05,
                       max_frames: int = 5000) -> dict:
    """What an execution does, as 3D frames on one clock: from ``home`` (the standing pose, or
    where an earlier execution left the arm: start_values), the pole moves to the stage height,
    then the hand to the stage pose, then the arm moves joint-linearly to the trajectory's first
    line (at TRANSIT_FRACTION of each joint's rating), then the trajectory plays at 5 ms per
    line. Frames hold the transform of every link the pole moves; ``end`` is where it leaves the
    arm (pole, hand counts, last line)."""
    from . import robot_model

    model = robot_model.fk_model()
    s = SIDE_PREFIX[side]
    arm_names = [f"{s}_joint{i}" for i in range(1, 8)]
    q = np.asarray(md, dtype=float) / 1000.0                              # degrees
    n = q.shape[0]
    clock = execution_times(side, q, stage, home, rating_deg_s)
    pole0, hand0, hand1, arm0 = clock["pole0"], clock["hand0"], clock["hand1"], clock["arm0"]
    t_pole, t_hand, t_move = clock["t_pole"], clock["t_hand"], clock["t_move"]
    every = max(int(every), math.ceil(n / max(1, max_frames - int((t_pole + t_hand + t_move) / step_s) - 4)))

    samples = []                                                           # (t, pole_mm, hand, arm_deg)
    t = 0.0
    for k in range(max(1, int(math.ceil(t_pole / step_s))) + 1 if t_pole else 1):
        f = min(1.0, k * step_s / t_pole) if t_pole else 1.0
        e = 0.5 - 0.5 * math.cos(math.pi * f)                              # eased, as a profiled move looks
        samples.append((t + f * t_pole, pole0 + e * (stage.pole_mm - pole0), hand0, arm0))
    t += t_pole
    for k in range(1, int(math.ceil(t_hand / step_s)) + 1 if t_hand else 0):
        f = min(1.0, k * step_s / t_hand)
        samples.append((t + f * t_hand, stage.pole_mm, [h0 + f * (h1 - h0) for h0, h1 in zip(hand0, hand1)], arm0))
    t += t_hand
    for k in range(1, int(math.ceil(t_move / step_s)) + 1):
        f = min(1.0, k * step_s / t_move)
        e = 0.5 - 0.5 * math.cos(math.pi * f)
        samples.append((t + f * t_move, stage.pole_mm, hand1, arm0 + e * (q[0] - arm0)))
    t += t_move
    traj_t0 = t
    lines = list(range(0, n, every))
    if lines[-1] != n - 1:
        lines.append(n - 1)
    for i in lines:
        samples.append((traj_t0 + i * CYCLE_S, stage.pole_mm, hand1, q[i]))

    frames = np.empty((len(samples), len(links), 16), dtype="<f4")
    hu = hand_units()
    base = dict(home)
    for f, (_, pole, hand, arm) in enumerate(samples):
        values = dict(base)
        values[f"{s}_sliding_plate_joint"] = pole / 1000.0
        for ch, c in enumerate(hand):
            values[hu.joint_name(side, ch)] = hu.hw_to_rad(float(c), ch)
        values.update(zip(arm_names, np.radians(arm).tolist()))
        tf = model.link_world_transforms(values)
        frames[f] = [tf[ln].reshape(-1) for ln in links]
    return dict(
        side=side, links=list(links), times=[round(x[0], 4) for x in samples],
        frames=base64.b64encode(frames.tobytes()).decode("ascii"),
        traj_t0=round(traj_t0, 4), total_s=round(traj_t0 + (n - 1) * CYCLE_S, 4), lines=n,
        stage=stage.as_dict(), end=clock["end"],
        phases=[
            dict(name="pole", t0=0.0, t1=round(t_pole, 3),
                 text=f"Pole {pole0:.0f} → {stage.pole_mm:g} mm" if t_pole else f"Pole already at {stage.pole_mm:g} mm"),
            dict(name="hand", t0=round(t_pole, 3), t1=round(t_pole + t_hand, 3),
                 text=f"Hand → {hand_words(stage.hand)}" if t_hand else "Hand already in pose"),
            dict(name="move", t0=round(t_pole + t_hand, 3), t1=round(traj_t0, 3), text="Arm to the trajectory's start"),
            dict(name="trajectory", t0=round(traj_t0, 3), t1=round(traj_t0 + (n - 1) * CYCLE_S, 3),
                 text=f"Trajectory: {n:,} lines at 5 ms"),
        ])
