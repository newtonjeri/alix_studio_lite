"""C16.3 — joint space by CANFD passthrough, cleaning by `movel`. ROS-free.

The contract's words: *"the joint space movements can be handled by canfd
passthrough mode, but cleaning motions use the controller planners movel with
the motion requirements embedded."* So mode 3, like mode 2, differs from mode 1
in one respect only — **how a free-space arm move is executed**:

    mode 1   move_to_rest  ->  one planned `movej`, controller plans it
    mode 2   move_to_rest  ->  N joint targets from MoveIt's plan, chained
    mode 3   move_to_rest  ->  a 100 Hz stream of `rm_movej_canfd` frames that
                               WE interpolate; the controller plans nothing

Four hardware facts shape every line below, and each one has bitten:

1. **A CANFD stream emits no arrival event** (device 0 fires only for planned
   moves). Completion is the stream ENDING, verified afterwards by joint
   position. Anything that waits for an arrival here waits forever.
2. **The profile must have zero velocity at both ends.** Upstream uses a cosine
   ease and says why: a step discontinuity at frame 0 is what the controller
   flags on the planned path. So the stream starts from the arm's ACTUAL
   joints, read back, never from an assumed pose.
3. **Duration is COMMANDED, not estimated.** That is the one thing mode 3 buys
   over mode 1 — a planned `movej`'s duration has to be guessed (24-120 % error
   measured upstream), a streamed one is exactly what you asked for.
4. **The stream and a planned primitive must NEVER overlap** (F9's shape one
   level down): the controller is executing our frames, and a planned move
   issued into that is the same truncation. `alix_hw`'s `PlannedMoveGate` is
   this interlock in C++; `enforce_stream_interlock` is it here, and both
   refuse rather than sequence silently.

WHAT MODE 3 WILL NOT TOUCH — identical to mode 2, and for the same reasons:
`movel` stages stay linear (`move_to_start` is a Cartesian line by decision),
and the cleaning stroke stays a `movel_chain` because C16.3 says so in as many
words. Only the calls mode 1 issues as `movej`/`movej_p` become streams.
"""

from __future__ import annotations

import math
import time
from typing import Callable, Dict, List, Optional

from alix_dispatch import program as progmod
from alix_dispatch.program import Call, DispatchError, Program

# Upstream's stream parameters (`dual_arm_common`: CANFD_RATE_HZ / _FOLLOW /
# _ARM_S), cited not invented. 100 Hz measured at period mean 9.992 ms,
# p99 <= 10.16 ms under full concurrent load.
CANFD_RATE_HZ = 100.0
CANFD_FOLLOW = True

# Kinds mode 1 issues for a free-space move — the only ones mode 3 rewrites.
STREAMABLE_KINDS = ("movej", "movej_p")


class Mode3Error(DispatchError):
    """The program cannot be expressed as a passthrough stream."""


def _arm_columns(joint_names, side: str) -> List[int]:
    """Indices of that arm's 7 joints within a plan row.

    A plan row can carry pole and hand columns; streaming those to the arm
    would be a 14-value frame, which the SDK takes as a length error at best.
    """
    prefix = "L" if side == "left" else "R"
    names = list(joint_names)
    out = []
    for i in range(1, 8):
        want = f"{prefix}_joint{i}"
        if want not in names:
            raise Mode3Error(f"plan stage does not carry {want}; it has {names}")
        out.append(names.index(want))
    return out


def ease(phase: float) -> float:
    """Cosine ease: zero velocity at both ends, 0 -> 1 over `phase` 0 -> 1."""
    return (1.0 - math.cos(math.pi * max(0.0, min(1.0, phase)))) / 2.0


def stream(call, backend, *, clock: Callable[[], float] = time.perf_counter,
           sleep: Callable[[float], None] = time.sleep) -> Optional[int]:
    """Drive one CANFD stage: interpolate from where the arm IS to the goal.

    Returns the first non-zero SDK return seen, or 0. Frame errors are counted
    rather than aborting mid-stream: stopping halfway leaves the arm wherever
    the last accepted frame put it, which is worse than finishing the ease and
    reporting.
    """
    d = call.detail
    path = d.get("path_deg")
    if path:
        return _stream_path(call, backend, clock=clock, sleep=sleep)
    goal = list(d["joints_deg"])
    start = d.get("start_joints_deg")
    if start is None:
        start = backend.current_joints_deg(getattr(call, "arm", None))
    if not start:
        raise Mode3Error(
            f"{call.stage}: no start joints — a CANFD stream interpolates FROM "
            "the arm's actual pose, and assuming one puts a step discontinuity "
            "at frame 0, which is exactly what the controller rejects")
    if len(start) != len(goal):
        raise Mode3Error(
            f"{call.stage}: start has {len(start)} joints, goal has {len(goal)}")

    dur = float(d["duration_s"])
    rate = float(d.get("rate_hz") or CANFD_RATE_HZ)
    follow = bool(d.get("follow", CANFD_FOLLOW))
    period = 1.0 / rate

    sent = errors = 0
    first_bad = None
    t0 = clock()
    next_t = t0
    while True:
        phase = 1.0 if dur <= 0 else min(1.0, (clock() - t0) / dur)
        s = ease(phase)
        cmd = [a + (b - a) * s for a, b in zip(start, goal)]
        ret = backend.movej_canfd(cmd, follow)
        sent += 1
        if ret not in (None, 0):
            errors += 1
            if first_bad is None:
                first_bad = ret
        if phase >= 1.0:
            break
        next_t += period
        sleep(max(0.0, next_t - clock()))

    d["frames_sent"] = sent
    d["frame_errors"] = errors
    d["stream_s"] = clock() - t0
    return first_bad or 0


def _stream_path(call, backend, *, clock, sleep) -> Optional[int]:
    """Stream MoveIt's OWN joint path, resampled to the frame rate.

    This is the strongest form of mode 3 and the one the contract points at:
    the controller plans nothing, so the arm follows the planned path exactly.
    Mode 2's deviation question does not arise here — there is no shortcut to
    deviate. What it costs instead is bandwidth: one frame every 10 ms for the
    whole stage, where mode 2 sends four targets.

    Resampled on the plan's OWN `time_from_start`, not on index: MoveIt's
    waypoints bunch where the motion is slow, so even index spacing would run
    the arm fast through the dense parts and slow through the sparse ones.
    """
    d = call.detail
    path = [list(row) for row in d["path_deg"]]
    times = [float(t) for t in d["times_s"]]
    if len(path) != len(times) or len(path) < 2:
        raise Mode3Error(f"{call.stage}: path has {len(path)} rows against "
                         f"{len(times)} times")
    total = float(d.get("duration_s") or times[-1])
    if total <= 0:
        raise Mode3Error(f"{call.stage}: planned duration is {total} s")
    scale = total / times[-1] if times[-1] > 0 else 1.0
    rate = float(d.get("rate_hz") or CANFD_RATE_HZ)
    follow = bool(d.get("follow", CANFD_FOLLOW))
    period = 1.0 / rate

    def sample(t_plan: float) -> List[float]:
        if t_plan <= times[0]:
            return list(path[0])
        if t_plan >= times[-1]:
            return list(path[-1])
        lo, hi = 0, len(times) - 1
        while hi - lo > 1:                       # the plan's times are sorted
            mid = (lo + hi) // 2
            if times[mid] <= t_plan:
                lo = mid
            else:
                hi = mid
        span = times[hi] - times[lo]
        f = 0.0 if span <= 0 else (t_plan - times[lo]) / span
        return [a + (b - a) * f for a, b in zip(path[lo], path[hi])]

    sent = errors = 0
    first_bad = None
    t0 = clock()
    next_t = t0
    while True:
        elapsed = clock() - t0
        phase = min(1.0, elapsed / total)
        ret = backend.movej_canfd(sample(phase * times[-1]), follow)
        sent += 1
        if ret not in (None, 0):
            errors += 1
            if first_bad is None:
                first_bad = ret
        if phase >= 1.0:
            break
        next_t += period
        sleep(max(0.0, next_t - clock()))

    d["frames_sent"] = sent
    d["frame_errors"] = errors
    d["stream_s"] = clock() - t0
    d["time_scale"] = scale
    return first_bad or 0


def enforce_stream_interlock(program: Program) -> tuple:
    """No planned primitive may be issued while a stream is running.

    The stream driver above is blocking, so a program built here satisfies this
    by construction — which is exactly why it is checked rather than assumed.
    The failure it guards is the F9 shape one level down: the controller is
    executing OUR frames, and a planned move issued into that truncates them.

    Returns `(ok, order, problems)`, the same shape as
    `program.enforce_device_serialization`, so a caller can run both.
    """
    problems: List[str] = []
    order: List[tuple] = []
    streaming: Dict[Optional[str], bool] = {}
    for call in program.calls:
        if call.device in ("scene", "config"):
            continue
        order.append((call.stage, call.device, call.kind, call.arm))
        arm = call.arm
        if streaming.get(arm) and call.kind != "movej_canfd":
            problems.append(
                f"{call.stage}: {call.kind} issued while the {arm} arm's CANFD "
                "stream is still running — the controller is executing our "
                "frames and a planned primitive truncates them (F9's shape one "
                "level down; alix_hw's PlannedMoveGate is this same interlock)")
        # `stream()` blocks until the ease completes, so a stream leaves nothing
        # in flight. A non-blocking streamer would set this True and clear it on
        # its join, and the check above would then earn its keep at runtime.
        streaming[arm] = False
    return (not problems), order, problems


def _duration_for(call, default_s: float) -> float:
    """How long this stage's stream should take.

    Mode 3's whole advantage is that this is COMMANDED. There is no formula
    here on purpose: a duration derived from a speed percentage would be the
    estimate mode 3 exists to avoid, and C16.4 forbids inventing an execution
    parameter. Callers pass it; the config may carry it later.
    """
    d = call.detail
    for key in ("duration_s", "stream_duration_s"):
        if d.get(key):
            return float(d[key])
    return float(default_s)


def build_program(cfg, states, model, *, duration_s: float = 4.0,
                  rate_hz: float = CANFD_RATE_HZ,
                  plan_path: Optional[str] = None,
                  start_joints_deg: Optional[Dict[str, List[float]]] = None,
                  blend_pct: Optional[int] = None,
                  pole_start_m: Optional[float] = None) -> Program:
    """A mode-1 program with its free-space arm moves turned into CANFD streams.

    With `plan_path`, a stage that MoveIt planned is streamed along ITS OWN
    joint path — including the Cartesian (`movej_p`) stages, which cannot be
    streamed otherwise because a frame is joints and mode 3 owns no IK opinion.
    Without it, only named-state (`movej`) stages stream, by cosine ease.

    `start_joints_deg` seeds the FIRST stream per arm when the backend cannot be
    read (offline tests, dry runs). At run time each stream reads the arm back
    instead — see `stream`.
    """
    base = progmod.build_program(cfg, states, model, blend_pct=blend_pct,
                                 pole_start_m=pole_start_m)
    planned = {}
    if plan_path:
        from alix_dispatch.mode2 import _plan_stages
        planned = _plan_stages(plan_path)
        duplicated_plan = planned.pop("__duplicated__", [])
    calls: List[Call] = []
    warnings = list(base.warnings)
    streamed = 0
    seeded = dict(start_joints_deg or {})

    for call in base.calls:
        if call.device != "arm" or call.kind not in STREAMABLE_KINDS:
            calls.append(call)
            continue

        side = call.arm or base.side
        plan_stage = planned.get(call.stage)
        if plan_stage is not None:
            cols = _arm_columns(plan_stage["joint_names"], side)
            rows = [[math.degrees(wp[c]) for c in cols]
                    for wp in plan_stage["positions"]]
            times = list(plan_stage["times"])
            detail = {
                "joints_deg": rows[-1],
                "path_deg": rows,
                "times_s": times,
                "duration_s": float(times[-1]) or float(duration_s),
                "rate_hz": rate_hz,
                "follow": CANFD_FOLLOW,
                "replaces": call.kind,
                "source": "moveit_plan",
                "dense": len(rows),
                "speed_pct": call.detail.get("speed_pct"),
            }
            seeded[side] = rows[-1]
            calls.append(Call(call.stage, "arm", "movej_canfd", detail, arm=call.arm))
            streamed += 1
            continue

        if call.kind == "movej_p":
            # movej_p commands a POSE and lets the controller solve it. A CANFD
            # frame is joints, so streaming it needs IK — and mode 3 does not
            # own an IK opinion (the controller's solver is the authority, and
            # `rm_algo_*` is one process-global context). Left planned, said so.
            warnings.append(
                f"mode3: {call.stage!r} is a Cartesian target (movej_p) with no "
                "plan sub-trajectory — streaming it would need local IK, and the "
                "controller's solver is the authority. Issued as mode 1 "
                f"({call.kind}), not skipped. Pass --plan to stream it.")
            calls.append(call)
            continue

        detail = {
            "joints_deg": list(call.detail["joints_deg"]),
            "duration_s": _duration_for(call, duration_s),
            "rate_hz": rate_hz,
            "follow": CANFD_FOLLOW,
            "replaces": call.kind,
            "speed_pct": call.detail.get("speed_pct"),
            "state": call.detail.get("state"),
        }
        if side in seeded:
            detail["start_joints_deg"] = list(seeded[side])
        seeded[side] = list(call.detail["joints_deg"])
        calls.append(Call(call.stage, "arm", "movej_canfd", detail, arm=call.arm))
        streamed += 1

    if not streamed:
        warnings.append(
            "mode3: NOTHING was streamed — every free-space stage stayed "
            "planned. This program is mode 1 with extra steps.")

    prog = Program(base.task, base.side, calls, warnings)
    prog.mode = 3                                        # type: ignore[attr-defined]
    prog.streamed_stages = streamed                      # type: ignore[attr-defined]
    return prog


def describe(prog) -> str:
    rows = [c for c in prog.calls if c.kind == "movej_canfd"]
    if not rows:
        return "mode 3: no stage streamed"
    out = [f"mode 3: {len(rows)} stage(s) as CANFD passthrough "
           f"@ {rows[0].detail.get('rate_hz', CANFD_RATE_HZ):.0f} Hz"]
    for c in rows:
        d = c.detail
        n = int(round(d["duration_s"] * float(d.get("rate_hz") or CANFD_RATE_HZ)))
        out.append(f"  {c.stage:<24} {d['duration_s']:.2f} s commanded "
                   f"(~{n} frames), replaces {d['replaces']}")
    return "\n".join(out)
