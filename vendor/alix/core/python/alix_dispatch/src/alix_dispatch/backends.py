"""Where a mode-1 program actually goes. ROS-free.

`RecordingBackend` is the dry run — it resolves nothing further, it just keeps
the call sequence, which is what the offline half of C11 step 4b needs.
`SdkBackend` issues the vendored RealMan SDK calls.

UNITS ARE NOT INVENTED HERE (C16.4). Two conversions are project constants
with measured provenance and are IMPORTED or cited, never guessed:

  * the pole: HW units are TRUE millimetres — 1000 per metre, harmonized 1:1
    with MoveIt (`alix_hw/conversions.hpp`, `kLiftCmdUnitsPerMetre`). The old
    `1000 * 2/3` gearing is DEPRECATED; keeping it would command the pole
    ~33 % low.
  * the hand: 0 rad = open = `scale_max`, per channel against that channel's
    max travel — reused from `alix_hw/scripts/hand_units.py`, which is itself
    the python mirror of `conversions.hpp`, rather than copied here.
"""

from __future__ import annotations

import importlib.util
import pathlib
import time
from typing import Callable, Dict, List, Optional

# alix_hw/conversions.hpp kLiftCmdUnitsPerMetre — m -> pole HW units.
# 1:1 with MoveIt: HW units ARE mm (was 1000 * 2/3 before harmonization).
LIFT_UNITS_PER_METRE = 1000.0


def _load_hand_units():
    """Import `alix_hw/scripts/hand_units.py` by path.

    It is a plain ROS-free module (no rclpy) that mirrors `conversions.hpp`.
    Loading it keeps ONE definition of the hand scale in the tree; copying the
    formula here would be a second one, which is how two `junction_limits`
    ended up disagreeing.
    """
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "ros" / "hw" / "alix_hw" / "scripts" / "hand_units.py"
        if cand.is_file():
            spec = importlib.util.spec_from_file_location("alix_hand_units", cand)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod
    return None


class RecordingBackend:
    """Records the program without touching hardware."""

    def __init__(self):
        self.calls: List[Dict] = []

    def set_tool_frame(self, tool_frame, **kw):
        self.calls.append({"call": "rm_change_tool_frame", "tool_frame": tool_frame})

    def lift(self, height_m, speed_pct, **kw):
        self.calls.append({"call": "rm_set_lift_height", "height_m": height_m,
                           "hw": round(height_m * LIFT_UNITS_PER_METRE),
                           "speed_pct": speed_pct})

    def hand(self, joints_rad, speed_pct, state=None, **kw):
        self.calls.append({"call": "rm_set_hand_angle", "state": state,
                           "joints": len(joints_rad), "speed_pct": speed_pct})

    def movej(self, joints_deg, speed_pct, r, connect, **kw):
        self.calls.append({"call": "rm_movej", "joints_deg": list(joints_deg),
                           "v": speed_pct, "r": r, "connect": connect})
        self._last_joints_deg = list(joints_deg)

    def movej_p(self, pose, speed_pct, r, connect, **kw):
        self.calls.append({"call": "rm_movej_p", "pose": dict(pose),
                           "v": speed_pct, "r": r, "connect": connect})

    def movel(self, pose, speed_pct, r, connect, **kw):
        self.calls.append({"call": "rm_movel", "pose": dict(pose),
                           "v": speed_pct, "r": r, "connect": connect})

    def movej_canfd(self, joints_deg, follow=True, **kw):
        """One CANFD passthrough frame (C16.3). NOT a planned move.

        The controller does not plan this and emits NO device-0 arrival event,
        so completion is the STREAM ending, verified afterwards by joint
        position. Anything that waits for an arrival here waits forever.
        """
        self.calls.append({"call": "rm_movej_canfd", "joints_deg": list(joints_deg),
                           "follow": bool(follow)})
        self._last_joints_deg = list(joints_deg)

    def current_joints_deg(self, side=None):
        """Where the arm is now, in degrees, or None if this backend cannot say.

        A CANFD stream has to interpolate FROM somewhere; guessing a start pose
        would put a step discontinuity at frame 0, which is precisely what the
        controller flags.
        """
        return list(getattr(self, "_last_joints_deg", []) or []) or None

    def movec(self, pose_via, pose_to, speed_pct, r, connect, loop=0, **kw):
        self.calls.append({"call": "rm_movec", "via": dict(pose_via),
                           "pose": dict(pose_to), "v": speed_pct, "r": r,
                           "loop": loop, "connect": connect})


class SdkBackend(RecordingBackend):
    """Issues the real SDK calls through a vendored `RoboticArm` handle.

    The handle is injected rather than constructed here so the executor is
    testable against a fake, and so nothing in this package opens a connection
    as a side effect of being imported.
    """

    # Hand stroke model — OURS, not the SDK's. The SDK has no duration model;
    # this is the production law from `dual_arm_common.start_hand_acked`,
    # measured 2026-08-06: 373 counts per second plus 115 ms of command latency.
    HAND_COUNTS_PER_S = 373.0
    HAND_LATENCY_S = 0.115
    HAND_FULL_SPAN = 1000

    def __init__(self, robot, side: str, block: int = 1, thread_mode: int = 3,
                 hand_rad_to_hw: Optional[Callable[[float, int], int]] = None,
                 sleep=None):
        super().__init__()
        self.robot = robot
        self.side = side
        # Set by a session that owns a run mode; None for every directly-built
        # backend, so the guard below is inert unless someone opts in.
        self._sim_guard = None
        # BLOCK SEMANTICS DEPEND ON THE SDK THREAD MODE (rm_movej docstring,
        # rm_robot_interface.py:1471-1487): in MULTI/TRIPLE mode block 0/1 is
        # non-blocking / block-until-arrival, but in SINGLE-thread mode any
        # nonzero block is a TIMEOUT IN SECONDS returning -5 on expiry — so
        # block=1 there is a one-second deadline that fails every real move.
        # Every production path inits RM_TRIPLE_MODE_E; refuse the combination
        # rather than discover it as mysterious -5s on hardware.
        if thread_mode == 0 and block != 0:
            raise ValueError(
                "single-thread SDK mode makes `block` a TIMEOUT IN SECONDS, not a "
                f"flag: block={block} would give every move a {block}s deadline. "
                "Init the arm in multi/triple thread mode, or pass block=0.")
        self.block = block
        self.thread_mode = thread_mode
        self._sleep = sleep or time.sleep
        self._last_hand: Optional[List[int]] = None
        if hand_rad_to_hw is None:
            mod = _load_hand_units()
            if mod is None:
                raise RuntimeError(
                    "hand_units.py not found and no hand_rad_to_hw supplied — "
                    "refusing to invent a hand scale (C16.4)")
            self._hand_mod = mod
            hand_rad_to_hw = mod.rad_to_hw
        else:
            self._hand_mod = None
        self._hand_rad_to_hw = hand_rad_to_hw

    def _pose_list(self, pose) -> List[float]:
        return [pose["x"], pose["y"], pose["z"], pose["rx"], pose["ry"], pose["rz"]]

    def set_tool_frame(self, tool_frame, **kw):
        super().set_tool_frame(tool_frame, **kw)
        return self.robot.rm_change_tool_frame(tool_frame)

    def _refuse_sim_bypass(self, device: str) -> None:
        """Refuse a pole or hand command that reached the SDK in SIM.

        The controller models NEITHER device (C9: "The controller does not
        simulate the pole or the hands"), so such a command does not fail
        cleanly — it sits in its arrival wait and times out. Worse, the pole
        and the hands are separate devices from the arm controller: a SIM run
        that forwarded them would leave the arm simulating while real metal
        moved.

        This cannot be caught by an offline test. The emulator executes both
        devices in BOTH run modes — `rm_set_lift_height` and
        `rm_set_hand_angle` never consult `run_mode` — so an emulator run
        passes either way. Hence a structural, fail-closed guard at the device
        rather than a test.
        """
        g = self._sim_guard
        if g is not None and g.sim_mode and not g.issuing:
            raise ExecutionError(
                f"{self.side}: {device} commanded in SIM outside the session "
                f"issuer — the controller models no {device}; this would hang "
                "rather than fail, and may move real metal")

    def lift(self, height_m, speed_pct, **kw):
        self._refuse_sim_bypass("pole")
        super().lift(height_m, speed_pct, **kw)
        return self.robot.rm_set_lift_height(
            int(speed_pct), int(round(height_m * LIFT_UNITS_PER_METRE)), self.block)

    def hand(self, joints_rad, speed_pct, state=None, **kw):
        """Fire NON-blocking, then dwell the computed stroke.

        TWO HARDWARE FACTS make the obvious blocking call wrong, and neither is
        visible in the emulator (both live in the controller's arrival stream —
        `dual_arm_common.start_hand_acked`, measured 2026-08-06):

        * device-2 (hand) arrival events are NEVER delivered to the user
          callback on fw V1.7.1/V1.7.4 with SDK 1.1.6, so nothing will ever
          wake a blocking hand call on its own account; and
        * a BLOCKING `rm_set_hand_angle` works only in isolation — with any
          other arrival in flight it consumes the ARM's arrival push and fails
          instantly with -4 (arrival-device mismatch).

        So: non-blocking fire, then sleep the stroke. Feedback is echo.
        """
        self._refuse_sim_bypass("hand")
        super().hand(joints_rad, speed_pct, state=state, **kw)
        order = self.hand_counts(joints_rad)
        ret = self.robot.rm_set_hand_angle(order, False, 2)
        if ret in (None, 0):
            self._sleep(self.hand_dwell_s(order))
            self._last_hand = list(order)
        return ret

    def hand_counts(self, joints_rad) -> List[int]:
        """Per-channel HW counts for a hand state. Pure — commands nothing.

        Separated so a SIM run can still RESOLVE the target and still raise the
        missing-channel `KeyError`, skipping only the command and the dwell.
        Upstream resolves the target before its SIM branch for the same reason:
        a state the plan cannot express is a config fault in every mode.
        """
        if self._hand_mod is None:
            raise RuntimeError("no hand channel map available")
        order = []
        for ch in range(6):
            name = self._hand_mod.joint_name(self.side, ch)
            if name not in joints_rad and ch in (4, 5):
                # exclude_thumb plan: no thumb joints. -1 is the SDK's own
                # "this DOF does nothing, keeps its state" (rm_set_hand_angle).
                order.append(-1)
                continue
            if name not in joints_rad:
                raise KeyError(f"hand state is missing {name!r}; cannot command channel {ch}")
            order.append(self._hand_rad_to_hw(joints_rad[name], ch))
        return order

    def hand_dwell_s(self, target_counts) -> float:
        """Stroke time for a hand move, in seconds.

        Span is measured against the last commanded state; with none known yet
        the FULL span is assumed, because guessing short here means commanding
        the next stage while the fingers are still closing.
        """
        if self._last_hand is None:
            span = float(self.HAND_FULL_SPAN)
        else:
            span = float(max((abs(a - b) for a, b in zip(target_counts, self._last_hand)
                              if a >= 0 and b >= 0), default=self.HAND_FULL_SPAN))
        return span / self.HAND_COUNTS_PER_S + self.HAND_LATENCY_S

    def movej_canfd(self, joints_deg, follow=True, **kw):
        """One CANFD passthrough frame. NO override here means NO frame SENT.

        `SdkBackend` inherits `RecordingBackend`, so an un-overridden method
        records the call and returns None — which `execute` reads as success.
        Mode 3's first emulator run sent 162 frames, reported 0 errors, and left
        the arm exactly where it started (2026-08-20). The frames had been
        appended to a list. That is the shape of every "green run over a code
        path that never ran" in this project (F79, F81), one layer lower.

        `rm_movej_canfd(joint, follow, expand, trajectory_mode, radio)` — the
        trailing three are 0 at every upstream call site.
        """
        super().movej_canfd(joints_deg, follow, **kw)
        return self.robot.rm_movej_canfd(list(joints_deg), bool(follow), 0, 0, 0)

    def current_joints_deg(self, side=None):
        """Where the arm actually IS, read back from the controller.

        A CANFD stream interpolates FROM here. Falling back to the last
        COMMANDED joints (the recording behaviour) would start the ease from a
        pose the arm may never have reached, putting a step at frame 0 — which
        is exactly what the controller rejects.
        """
        ret, st = self.robot.rm_get_current_arm_state()
        if ret != 0 or not st or "joint" not in st:
            return None
        return list(st["joint"])

    def movej(self, joints_deg, speed_pct, r, connect, **kw):
        super().movej(joints_deg, speed_pct, r, connect, **kw)
        return self.robot.rm_movej(list(joints_deg), int(speed_pct), int(r or 0),
                                   int(connect), self.block)

    def movej_p(self, pose, speed_pct, r, connect, **kw):
        super().movej_p(pose, speed_pct, r, connect, **kw)
        return self.robot.rm_movej_p(self._pose_list(pose), int(speed_pct),
                                     int(r or 0), int(connect), self.block)

    def movel(self, pose, speed_pct, r, connect, **kw):
        super().movel(pose, speed_pct, r, connect, **kw)
        return self.robot.rm_movel(self._pose_list(pose), int(speed_pct),
                                   int(r or 0), int(connect), self.block)

    def movec(self, pose_via, pose_to, speed_pct, r, connect, loop=0, **kw):
        """One turnaround as `rm_movec` — three points, one traced circle.

        `loop=0`: the vendor's repeat count. A turnaround traces the arc ONCE;
        the emulator accepts loop but does not model it, and no measured run
        has ever used it mid-chain.

        `block` is the SAME as every other move here, so the arc queues on
        connect=1 exactly like a movel and the chain still closes on a single
        connect=0. Blocking per-arc would give back the N-1 full stops the
        chain exists to avoid (C4 / C16.2).
        """
        super().movec(pose_via, pose_to, speed_pct, r, connect, loop=loop, **kw)
        return self.robot.rm_movec(self._pose_list(pose_via),
                                   self._pose_list(pose_to), int(speed_pct),
                                   int(r or 0), int(loop), int(connect),
                                   self.block)

    #: Per-arm MEASURED payload consensus (payload_audit.py, Newton's
    #: calibration). NOT mirrored: CX is negative on both arms and the right is
    #: 25 % heavier because a D435 rides distal of J7 (F32). The mirrored
    #: prediction is recorded in payload_audit AS a prediction and is refuted.
    PAYLOAD_CONSENSUS = {
        "left":  {"kg": 0.567, "com_m": (-0.0252, 0.0412, 0.2240)},
        "right": {"kg": 0.711, "com_m": (-0.0236, 0.0254, 0.1648)},
    }

    def verify_tool_frame(self, want_name, want_xyz_m=None, tol_mm=0.5,
                          check_payload=True, payload_tol_kg=1e-3):
        """Select the tool frame AND prove it took. Returns (ok, detail).

        MEASURED 2026-08-20, and it is why this is not a bare setter:
        `rm_change_tool_frame("R_glove_2")` returned success on the emulator and
        read back as `name 'Arm_Tip', pose [0,0,0,0,0,0]` — the flange, a 180 mm
        error on every waypoint, silently. The controller's STORED frame table
        is not the same source of truth as the computed offsets, and a write
        session that assumes it is will command the wrong point.
        """
        ret = self.robot.rm_change_tool_frame(want_name)
        got = self.robot.rm_get_current_tool_frame()
        frame = got[1] if isinstance(got, tuple) else got
        name = (frame or {}).get("name") if isinstance(frame, dict) else None
        if name != want_name:
            return False, (f"tool frame {want_name!r} did not take: controller "
                           f"reports {name!r} (ret={ret})")
        if want_xyz_m is not None:
            pose = (frame or {}).get("pose") or [0.0] * 6
            off = max(abs(float(pose[i]) - float(want_xyz_m[i])) for i in range(3))
            if off * 1000.0 > tol_mm:
                return False, (f"tool frame {want_name!r} selected but its pose "
                               f"is {off * 1000:.1f} mm from the expected offset")
        if check_payload:
            # The expectation is THIS ARM's measured consensus — not the Hand
            # frame's record, and not the other arm mirrored. Copying payload
            # from whatever frame happens to be active stamps Hand's 0.706 kg
            # over the left's calibrated 0.567 (demonstrated on the emulator,
            # 2026-08-20); mirroring gets the centroid sign wrong on both arms.
            want = self.PAYLOAD_CONSENSUS.get(getattr(self, "side", None))
            if want is not None:
                got_kg = float((frame or {}).get("payload", 0.0) or 0.0)
                if abs(got_kg - want["kg"]) > payload_tol_kg:
                    return False, (f"tool frame {want_name!r}: payload "
                                   f"{got_kg:.3f} kg, expected this arm's "
                                   f"measured consensus {want['kg']:.3f} kg")
                # KEY TRAP, measured: the read-back exposes the centroid as
                # FLAT x/y/z fields, not a "com" list — the same shape trap as
                # the name arriving under "name" rather than "frame_name",
                # which on 2026-08-08 silently skipped a restore and left both
                # arms on the last frame written. Accept either.
                com = (frame or {}).get("com")
                if com is None:
                    com = [float((frame or {}).get(k, 0.0) or 0.0)
                           for k in ("x", "y", "z")]
                d = max(abs(float(com[i]) - want["com_m"][i]) for i in range(3))
                # An exact 1000x is the mm/metre unit slip that wrote a
                # 128-METRE centroid and made every movel an arm collision.
                ratio = (abs(float(com[2])) / abs(want["com_m"][2])
                         if want["com_m"][2] else 0.0)
                if 900 < ratio < 1100:
                    return False, (f"tool frame {want_name!r}: centroid reads "
                                   f"{ratio:.0f}x the expected value — mm "
                                   f"written where the API takes METRES")
                if d * 1000.0 > tol_mm:
                    return False, (f"tool frame {want_name!r}: centroid is "
                                   f"{d * 1000:.1f} mm from this arm's consensus")
        return True, f"tool frame {want_name!r} verified (pose, payload, centroid)"



class MultiArmBackend:
    """Routes each call to the backend for the arm that call names.

    CONSTRAINTS FROM THE DUAL-ARM MECHANISM OF RECORD (`dual_arm_common.py`,
    confirmed by the RM_API2 session 2026-08-20). They are recorded here
    because each one is invisible until it bites:

    * **F9 is PER CONTROLLER, not cross-arm.** Concurrent lift + PLANNED arm
      move is a vendor-confirmed Gen-3 defect in both directions — the command
      issued second TRUNCATES the first, and a truncated pole stops SILENTLY
      ~25 mm in with no error. So a lift and a planned move must never overlap
      ON ONE ARM. Cross-arm concurrency is routine and hardware-proven, and is
      NOT restricted. This dispatcher is strictly sequential, so it satisfies
      the per-arm rule by construction.
    * **Two device domains emit NO arrival event**: hands (device 2 — completion
      is a duration/echo model) and CANFD-streamed arm moves. A dispatcher that
      waits for an event on those hangs forever. This one uses the SDK's own
      blocking form rather than an event join, so it inherits the SDK's
      completion model for each device instead of assuming one.
    * **The arrival-event callback is ONE PER PROCESS.** Registering per-arm
      clobbers the previous registration and the first arm's arrivals are lost
      silently. Anything that moves to event-based joins must register ONCE and
      demux by handle_id (that is what `ArrivalMonitor` is for), and must keep
      the ctypes callback referenced for the process lifetime — garbage
      collecting it crashes the SDK receive thread.
    * **The whole `rm_algo_*` layer is one C-library-global context** (model,
      work frame, tool frame — last writer wins). Mode 1 never calls algo: it
      sends poses to the controller and lets the controller solve, so two arms
      in one process are safe here. Anything that adds local IK must re-set the
      full context per use or confine algo to one side.
    

    Compound tasks alternate arms within ONE ordered sequence, so this holds a
    backend per side and dispatches by `call.arm`. It deliberately does NOT
    run the two arms concurrently: the config already states the order, and an
    ordering invented here would be a second opinion about a task that has
    already been sequenced (and the arms hand a lid between them, so the order
    is the safety property).
    """

    def __init__(self, by_side: Dict[str, object]):
        self.by_side = dict(by_side)
        self.calls: List[Dict] = []

    def for_call(self, call):
        side = call.arm
        if side is None:
            raise ExecutionError(
                f"{call.stage}: a device call with no arm — refusing to guess")
        if side not in self.by_side:
            raise ExecutionError(
                f"{call.stage}: no backend for arm {side!r} (have {sorted(self.by_side)})")
        return self.by_side[side]

    def record(self, side: str, entry: Dict) -> None:
        self.calls.append(dict(entry, arm=side))


class ExecutionError(RuntimeError):
    pass


def execute(program, backend, *, issue: Optional[Callable] = None) -> List[Dict]:
    """Walk a program and issue every call, in order.

    A non-zero SDK return stops the run: C10 records that a mid-chain invalid
    target returns ret=1 for that segment while the chain still completes, so
    every return code must be checked rather than assumed.
    """
    issue = issue or _issue
    multi = isinstance(backend, MultiArmBackend)
    for call in program.calls:
        if call.device == "scene":
            continue
        if multi:
            target = backend.for_call(call)
            before = len(target.calls)
            ret = issue(call, target)
            for entry in target.calls[before:]:
                backend.record(call.arm, entry)
            # CHECK THE RETURN ON THIS BRANCH TOO. It used to be discarded, so
            # a non-zero ret from a lift/hand/movej/movej_p/movel in a COMPOUND
            # program was swallowed — only `movel_chain` self-raises (below).
            # C16.4: "ANY entry that matches none of these is REPORTED, never
            # dropped quietly", and `schedule.execute_dual` already checks it.
            if ret not in (None, 0):
                raise ExecutionError(f"{call.stage}: {call.kind} returned {ret}")
            continue
        ret = issue(call, backend)
        if ret not in (None, 0):
            raise ExecutionError(f"{call.stage}: {call.kind} returned {ret}")
    return backend.calls


def _issue(call, backend):
    """Issue ONE call on ONE backend. Single-arm and multi-arm share this."""
    d = call.detail
    if call.kind == "set_tool_frame":
        ret = backend.set_tool_frame(d["tool_frame"])
    elif call.kind == "lift":
        ret = backend.lift(d["height_m"], d["speed_pct"])
    elif call.kind == "hand":
        ret = backend.hand(d["joints_rad"], d["speed_pct"], state=d.get("state"))
    elif call.kind == "movej":
        ret = backend.movej(d["joints_deg"], d["speed_pct"], d.get("r"), d.get("connect", 0))
    elif call.kind == "movej_p":
        ret = backend.movej_p(d["pose"], d["speed_pct"], d.get("r"), d.get("connect", 0))
    elif call.kind == "movel":
        ret = backend.movel(d["pose"], d["speed_pct"], d.get("r"), d.get("connect", 0))
    elif call.kind == "movel_chain":
        ret = None
        for m in d["moves"]:
            # A spanned turnaround arrives as rm_movec through its via; every
            # other move is a movel. Both queue on connect=1, so the chain
            # still closes on ONE connect=0 and yields one arrival event.
            if m.get("kind") == "movec" and m.get("via") is not None:
                ret = backend.movec(m["via"], m["pose"], m["v_pct"],
                                    m.get("r"), m["connect"])
                kind = "movec"
            else:
                ret = backend.movel(m["pose"], m["v_pct"], m.get("r"),
                                    m["connect"])
                kind = "movel"
            if ret not in (None, 0):
                # NAME THE CHAIN, NOT THE WAYPOINT. Queued moves (connect=1)
                # return immediately; only the terminal connect=0 blocks and
                # reports. So a mid-chain rejection surfaces on the LAST move,
                # and "movec to 'point5' returned 1" sent us hunting the wrong
                # arc on 2026-08-20 — the arm had actually aborted eleven moves
                # earlier, on point3->point5. The controller offers no per-move
                # attribution, so neither does this.
                if m.get("connect") == 0:
                    raise ExecutionError(
                        f"{call.stage}: the chain of {len(d['moves'])} moves "
                        f"was rejected (ret={ret}) on its terminal blocking "
                        f"call. The failing move is NOT necessarily "
                        f"{m['point']!r} — queued moves return before they "
                        f"execute. Read the run's stream.csv to find where it "
                        f"actually stopped.")
                raise ExecutionError(
                    f"{call.stage}: queued {kind} to {m['point']!r} returned "
                    f"{ret} — rejected at enqueue, so this move IS the one")
    elif call.kind == "movej_chain":
        # MODE 2. Same chaining contract as `movel_chain`: connect=1 queues,
        # the closing connect=0 executes the lot as ONE motion with a single
        # arrival event, and only that terminal call blocks and reports.
        ret = None
        for m in d["moves"]:
            ret = backend.movej(m["joints_deg"], m["v_pct"], m.get("r"),
                                m["connect"])
            if ret not in (None, 0):
                if m.get("connect") == 0:
                    raise ExecutionError(
                        f"{call.stage}: the chain of {len(d['moves'])} joint "
                        f"targets was rejected (ret={ret}) on its terminal "
                        "blocking call. Queued targets return before they "
                        "execute, so the failing one is not necessarily the "
                        "last; read the run's stream.csv.")
                raise ExecutionError(
                    f"{call.stage}: queued joint target #{m.get('index')} "
                    f"returned {ret} — rejected at enqueue, so this one IS it")
    elif call.kind == "movej_canfd":
        # MODE 3. Imported late: mode3 imports this module, and the stream
        # driver is its business, not the backend's.
        from alix_dispatch import mode3
        ret = mode3.stream(call, backend)
    else:
        raise ExecutionError(f"no backend mapping for kind {call.kind!r}")
    return ret
