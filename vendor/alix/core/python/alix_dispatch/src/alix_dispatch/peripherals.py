"""Pole (lift) and hand (Inspire RH56, ALL-MODBUS) from the SDK handle.

The devices a continuous stroke flow must set BEFORE each program's movej
(Newton, 2026-09-02: "they should come outside before the first movej, and
both pole and hand should move at maximum velocities"). ROS-free; runs on
hardware and on the emulator through the same seam (`connect.open_arm`).

Facts encoded here are cited, never invented (C16.4):

* POLE — ``rm_set_lift_height(speed %, height mm, block)``. HW units ARE mm,
  1:1 with MoveIt (alix_hw/conversions.hpp ``kLiftCmdUnitsPerMetre`` = 1000;
  ``backends.LIFT_UNITS_PER_METRE``). The driver fires it NON-blocking and
  closes the loop on feedback (rm_connection.cpp:451, LiftChannel) — so does
  this: send with block 0, then poll ``rm_get_lift_state`` (``pos`` mm,
  ``mode`` 0 = idle) to a tolerance. The timeout comes from the MEASURED
  two-phase model (``lift_travel_s``, L49) x3, not from the vendor's rated
  0.185 m/s cruise, which this pole has never achieved; and where a 5 ms push
  frame source is supplied the ARRIVAL is judged on that channel rather than on
  ``rm_get_lift_state``, whose height leads the physical pole by up to 73 mm
  (B6). Every rm_set_lift_height is a FRESH move on the
  pole's own profiler (lift_channel.hpp:3-5), and a lift command landing while
  a PLANNED arm move is in flight ABORTS that move and latches joint errors
  (hardware 2026-08-07, modelled by the emulator) — so the pole is only ever
  commanded with the arm at rest, between programs.

* HAND — the robot runs ALL-MODBUS (``hand_command_api`` modbus_rtu,
  alix.urdf.xacro:37; device 1, baud 115200, rm_side.ros2_control.xacro:37).
  The hand PROTOCOL calls (rm_set_hand_angle / speed / force) are EMPIRICALLY
  mutually exclusive with modbus mode on fw 1.7.x (conversions.hpp:302-325,
  hand_channel.hpp:8-15): blocking calls return -5, non-blocking ones are
  swallowed with ret 0 and no motion (the 2026-08-06 C6 failure; the emulator
  models both). Hence ``rm_set_modbus_mode(port 1, 115200, 2)`` once, then
  register writes exactly as hand_channel.cpp: ANGLE_SET 1486, SPEED_SET 1522,
  FORCE_SET 1498, ANGLE_ACT 1546 — byte-addressed, two bytes per DOF, hi/lo
  encoding, ONE ``rm_write_registers`` for the six (~63 ms measured) with a
  per-register fallback, so a hand command lands whole or fails whole.
  Units 0-1000 per channel, 1000 = open; channel order little ring middle
  index thumb_bend thumb_rot (``hand_units.py`` is the python mirror of
  conversions.hpp and is IMPORTED, never copied).
  Stroke law: stroke_ms = 115 + 373 x span / SPEED_SET (``kHandStrokeK``,
  both hands 2026-07-27). "Arrived" is the WRONG wait for a grasp — fingers
  closing on the tool stop early on the force threshold — so the wait is
  SETTLED: ANGLE_ACT stops changing (or reaches the target), bounded by the
  law x3.

* PRE-STROKE STATE comes from the PLAN: the last pole stage and the last hand
  stage before ``execute_path`` are what the stroke was planned with; the
  RETREAT pose is the first arm stage after ``execute_path`` — the plan's own
  way out of the fixture — so the devices are never moved with the pad on the
  surface.
"""
from __future__ import annotations

import math
import time
from typing import Callable, Dict, List, Optional

from alix_dispatch.backends import LIFT_UNITS_PER_METRE, _load_hand_units

# --- hand (conversions.hpp kHandModbus*, rm_side.ros2_control.xacro) --------
HAND_MODBUS_PORT = 1            # end interface board RS485
HAND_MODBUS_DEVICE = 1          # hand_modbus_device
HAND_MODBUS_BAUD = 115200       # hand_modbus_baud
HAND_REG_ANGLE_SET = 1486
HAND_REG_FORCE_SET = 1498
HAND_REG_SPEED_SET = 1522
HAND_REG_ANGLE_ACT = 1546
HAND_DOF = 6
HAND_SPEED_SET_MAX = 1000       # kHandSpeedSetMax — "maximum velocity"
HAND_STROKE_K = 373.0           # kHandStrokeK  (ms per count-span at SPEED_SET 1)
HAND_LATENCY_S = 0.115          # measured command latency
HAND_OPEN: List[int] = [1000] * HAND_DOF
HAND_SETTLE_COUNTS = 3          # two consecutive reads within this = settled
HAND_ARRIVE_COUNTS = 15         # within this of the target = arrived
# Smallest request the hand actually resolves. Measured 2026-09-03: the thumb
# ROTATION channel travelled 42-53 counts on the day's big moves but 0 on a
# 17-count request (target 944 from 927), so a sub-deadband ask is stiction,
# not a dead channel — the `stuck` flag must not cry wolf on it.
HAND_DEADBAND_COUNTS = 25
#: The two thumb channels in hand_units order (4 thumb_2 flex, 5 thumb_1
#: rotate). A plan built with exclude_thumb (alix.urdf.xacro) has no thumb
#: joints, so these come out of `counts_from_rad` as None: HOLD, the same
#: instruction `HandModbus.command` already takes. The fingers stay required.
HAND_THUMB_CHANNELS = (4, 5)

# --- pole (conversions.hpp kLift*, lift_channel.hpp) -------------------------
#: kLiftMaxVelocity at 100 % — the VENDOR'S RATED figure, and NOT one this
#: robot has ever achieved (L49). It is kept because the driver, the URDF
#: (0.16 m/s) and the emulator (0.104 m/s) all quote a cruise, and because
#: callers import it; it is no longer what derives a timeout. Anything that
#: needs "how long will this pole move take" must call `lift_travel_s`.
LIFT_CRUISE_MPS = 0.185
LIFT_START_LATENCY_S = 0.38     # kLiftStartLatencyS (measured max, up)
LIFT_STROKE_M = 0.315           # kLiftStrokeM
LIFT_TOL_M = 0.005              # the pole controller's goal tolerance (5 mm)

# --- L49: the TWO-PHASE travel model, fitted to the run ledger ---------------
# The single-slope model (LIFT_START_LATENCY_S + d / LIFT_CRUISE_MPS, x3) put a
# full 0.315 m stroke at 8.25 s against the ~7 s of the earlier whole-mm up fit
# (below): a 1.15x margin on a branch that has never been reached, because the
# wait exits on the 0.68 s fallback instead (B6). On the fine channel the model
# now gives the stroke 4.75 s, and its timeout 14.2 s.
# Measured here (2026-10-22) on the pole's FINE channel, the 5 ms
# push's `lift_state.pos` (counts; mm per count from the move's whole-mm height
# change), from the first move over 0.2 mm to STAYING within 0.5 mm of where the
# pole settles (the median of the capture's last second), over every
# `_frames.ndjson` in Resource/runs/tracks/2026-09-*/ that carries a
# `pre_run.pole` with a target (113 files, 101 real moves, 12 zero-distance):
#
#     UP    0.075 m  n=43   1.917 / 1.954 / 1.968 s  (min/median/max)
#     DOWN  0.075 m  n=45   1.928 / 1.948 / 1.999 s
#     UP    0.044 / 0.060 / 0.076 m  1.586 / 1.776 / 1.969 s  (max)
#     DOWN  0.036 / 0.060 m          1.427 / 1.835 s
#     DOWN  0.132 / 0.149 / 0.165 / 0.234 m   2.545 / 2.687 / 2.818 / 3.491 s
#
# The same time up and down. A 75 mm move is within 5 mm of its end at 1.23 s,
# 2 mm at 1.48 s, 1 mm at 1.67 s, 0.5 mm at 1.95 s and 0.2 mm at 2.29 s (medians,
# both directions alike): a cruise of ~0.10 m/s, then a settle of ~1 s that one
# slope cannot fit. So the model is a slew plus a fixed creep:
#
#     t(d) = LIFT_CREEP_S + d / slew
#
# The earlier fit (UP 0.059, DOWN 0.097 m/s) timed first-change to last-change
# of the WHOLE-MM `lift_state.height` channel — UP 0.075 m 2.555-2.662 s, DOWN
# 1.713-1.730 s — and the asymmetry was that channel's rounding, not the pole:
# going up, the height reaches its final integer only at the very end of the
# settle (2026-09-03/run_slot70_212854: the pole 0.1 mm from settled), going
# down at the first crossing (run_slot71_212922: still 0.9 mm above).
#
# With 0.097 m/s both ways the prediction covers the slowest within-0.5 mm time
# in every observed cell with at least 0.27 s to spare — 2.273 >= 1.999 at
# 0.075 m, 3.201 >= 2.818 at 0.165 m, 3.912 >= 3.491 at 0.234 m — and
# `test_staging.py` asserts that against the 2026-09-0* run_slot cells. The
# spare stands for command-to-first-motion, which the frames do not timestamp;
# the 2026-07-15 bench put it at 194-266 ms down and 305-378 ms up
# (conversions.hpp kLiftStartLatencyS), so the slowest 0.075 m UP move can reach
# 0.5 mm up to ~0.07 s after the prediction — still inside 1 mm, against a 5 mm
# goal tolerance (LIFT_TOL_M). Only 100 % speed has ever been commanded, so the
# speed_pct scaling below is an assumption applied to the slew alone.
LIFT_SLEW_UP_MPS = 0.097        # measured on lift_state.pos at 100 %, = DOWN
LIFT_SLEW_DOWN_MPS = 0.097      # measured on lift_state.pos at 100 %
LIFT_CREEP_S = 1.5              # the final-approach phase, direction-independent
LIFT_TIMEOUT_MARGIN = 3.0       # kept from the old model: 3x on the sum
#: A pushed height must hold inside the tolerance for this long, moving no more
#: than LIFT_FRAME_STEP_M between frames, before the pole counts as ARRIVED.
LIFT_FRAME_STABLE_S = 0.20
LIFT_FRAME_STEP_M = 0.001


def lift_travel_s(dist_m: float, *, up: Optional[bool] = None,
                  speed_pct: int = 100) -> float:
    """Expected pole travel time for *dist_m* metres (L49, two-phase).

    The pole travels the same up and down (L49, the fine channel), so the two
    slews are equal and *up* no longer changes the result. It is kept for
    the callers that pass it; None (the height getter was unreadable) takes
    the up slew, as it always has.
    """
    frac = max(1, min(100, int(speed_pct))) / 100.0
    slew = (LIFT_SLEW_DOWN_MPS if up is False else LIFT_SLEW_UP_MPS) * frac
    return LIFT_CREEP_S + abs(float(dist_m)) / max(slew, 1e-6)


def lift_timeout_s(dist_m: float, *, up: Optional[bool] = None,
                   speed_pct: int = 100) -> float:
    """The two-phase travel time with the 3x margin kept from the old model."""
    return LIFT_TIMEOUT_MARGIN * lift_travel_s(dist_m, up=up, speed_pct=speed_pct)


def _params(address: int, num: int, device: int = HAND_MODBUS_DEVICE):
    from Robotic_Arm.rm_ctypes_wrap import rm_peripheral_read_write_params_t
    # keywords: the real struct takes either form, the emulator's shim keywords only
    return rm_peripheral_read_write_params_t(port=HAND_MODBUS_PORT, address=int(address),
                                             device=int(device), num=int(num))


def encode_hi_lo(values: List[int]) -> List[int]:
    out: List[int] = []
    for v in values:
        v = int(v)
        out += [(v >> 8) & 0xFF, v & 0xFF]
    return out


def decode_hi_lo(data: List[int]) -> List[int]:
    return [((int(data[2 * i]) & 0xFF) << 8) | (int(data[2 * i + 1]) & 0xFF)
            for i in range(len(data) // 2)]


class Pole:
    """The lift pole through the SDK: command-on-change, feedback-verified."""

    def __init__(self, robot, log: Callable[[str], None] = print,
                 sleep: Callable[[float], None] = time.sleep, poll_s: float = 0.05):
        self.robot = robot
        self.log = log
        self._sleep = sleep
        self.poll_s = poll_s

    def state(self) -> Optional[dict]:
        try:
            ret, st = self.robot.rm_get_lift_state()
        except Exception:                                              # noqa: BLE001
            return None
        return st if ret == 0 and st and "pos" in st else None

    def height_m(self) -> Optional[float]:
        st = self.state()
        return None if st is None else float(st["pos"]) / LIFT_UNITS_PER_METRE

    def stop(self) -> dict:
        """Stop the pole where it is — ``rm_set_lift_speed(0)`` (M41).

        The Python stack had no pole stop at all: an operator abort ran
        ``rm_set_arm_slow_stop`` and the base column kept travelling for up to
        2.6 s (measured, L49), ending in an unrecorded mixed-pole state. The
        C++ side has had the verb since ``rm_connection.cpp:318-327``
        (``lift_stop``), called from ``rm_side_system.cpp:766-780`` together
        with the arm stop and the hand freeze; this is that call, in Python.
        The vendor documents speed 0 as 升降机构停止运动 (open-loop control).
        Never raises: a stop path that can throw is not a stop path.
        """
        out = dict(ok=False, ret=None, verb="rm_set_lift_speed(0)", height_m=None,
                   error=None)
        fn = getattr(self.robot, "rm_set_lift_speed", None)
        if fn is None:
            out["error"] = ("this SDK handle has no rm_set_lift_speed — the pole "
                            "cannot be stopped from here")
            self.log(f"   ** pole: {out['error']}")
            return out
        try:
            out["ret"] = int(fn(0))
            out["ok"] = out["ret"] == 0
        except Exception as exc:                                       # noqa: BLE001
            out["error"] = repr(exc)
            self.log(f"   ** pole: rm_set_lift_speed(0) raised {exc!r} — the pole may "
                     f"still be moving")
            return out
        try:
            out["height_m"] = self.height_m()
        except Exception:                                              # noqa: BLE001
            pass
        self.log(f"   pole: STOP issued (rm_set_lift_speed(0) ret={out['ret']})"
                 + ("" if out["height_m"] is None
                    else f", height reads {out['height_m'] * 1000:.0f} mm"))
        return out

    def move(self, height_m: float, speed_pct: int = 100, tol_m: float = LIFT_TOL_M,
             timeout_s: Optional[float] = None, block: bool = False,
             frame_source: Optional[Callable[[], Optional[float]]] = None) -> dict:
        """Command *height_m* and wait for the pole to ARRIVE within *tol_m*.

        Returns {'ok','ret','start_m','final_m','t_s','verified_by',...}.

        WHAT COUNTS AS ARRIVAL (B6). ``rm_get_lift_state`` is not a witness: in
        ``2026-09-03/run_slot70_212854`` its ``pos`` read 148 mm **6 ms** after a
        75 -> 150 mm command while the 5 ms push frame's ``lift_state.height``
        sat at 75 mm until t = 1.120 s and only reached 150 mm at t = 3.763 s.
        Across the whole ledger 70 of 70 pole moves with a series exited this
        wait on the ``elapsed > LIFT_START_LATENCY_S + 0.3`` clause — a 0.68 s
        TIMER, not a settle (six zero-distance "moves" took the same 0.69-0.75 s,
        which is the proof) — while true travel is 1.92-2.82 s, the same up
        and down (first motion to within 0.5 mm on ``lift_state.pos``, L49). So:

        * with *frame_source* — a callable returning the latest pushed pole
          height in METRES, or None — arrival is judged on that channel:
          inside *tol_m*, moving no more than ``LIFT_FRAME_STEP_M`` per frame,
          held for ``LIFT_FRAME_STABLE_S``. ``verified_by`` is ``"frames"``.
        * an OFF-TARGET WITNESS is required first, but ONLY when the move is a
          real one. Six zero-distance commands exist in the ledger and they are
          legitimate: when the pole is already inside *tol_m* of the target the
          result is ``verified_by="no_op"`` and no witness is demanded. The
          no-op decision prefers the FRAME source over the getter when frames
          are live, because the getter is the channel that lies.
        * with no frame source, or with a frame source that never yields, the
          old getter wait runs unchanged and the result says
          ``verified_by="getter"`` so a reader can see which channel answered.
          When a source WAS supplied and never yielded, ``frames_missing`` is
          True and the getter is additionally held to the modelled travel time
          rather than the 0.68 s timer, because that timer is the defect.
        * if the FIRST frame is already inside the band while the pre-command
          getter reading says a real move is due, the two channels disagree —
          one of them is stale — and the move is REFUSED with
          ``channels_disagree`` True rather than passed off as a no-op. `start`
          is read before the command, so it is not subject to the in-travel
          lead; believing a stale frame there is how a needed 75 mm move
          becomes a silent "already there".

        *block* decides who consumes the lift's ARRIVAL EVENT, which is a
        separate question from whether we wait for the motion — we always wait.
        Non-blocking (the default, and what the driver's LiftChannel does)
        leaves the arrival unconsumed. That is harmless while nothing else
        blocks, and every run to 2026-09-03 started its program non-blocking. It
        is NOT harmless next to a blocking call: the RM_API2 sessions recorded
        that a stale arrival fails the next blocking one, and named the pole as
        the classic source — "a lift arrival landing inside a later blocking
        movej" (F89). So `flow --blocking-probe`, which makes
        `rm_set_program_id_run` blocking, commands the pole with block=1 so the
        lift consumes its own arrival at source. Unprovable offline: the
        emulator models the robot's MOTION, not its arrival-event stream.
        """
        mm = int(round(float(height_m) * LIFT_UNITS_PER_METRE))
        start = self.height_m()
        dist = abs(float(height_m) - start) if start is not None else LIFT_STROKE_M
        up = None if start is None else (float(height_m) > start)
        expected = lift_travel_s(dist, up=up, speed_pct=speed_pct)
        if timeout_s is None:
            timeout_s = lift_timeout_s(dist, up=up, speed_pct=speed_pct)
        no_op = start is not None and dist <= tol_m
        out = dict(ok=False, ret=None, target_m=float(height_m), start_m=start, final_m=None,
                   t_s=0.0, timeout_s=timeout_s, speed_pct=int(speed_pct), error=0,
                   series=[], moved_mm=0.0, distinct=0,
                   verified_by=None, witness=False, no_op=no_op,
                   no_op_source="getter" if no_op else None,
                   expected_s=round(expected, 3), frames=[], frame_source=False,
                   frames_final_m=None, frames_requested=frame_source is not None,
                   frames_missing=False, channels_disagree=False)
        self.log(f"   pole: {'?' if start is None else f'{start * 1000:.0f} mm'} -> {mm} mm "
                 f"at {int(speed_pct)} % (rm_set_lift_height "
                 f"{'BLOCKING (consumes its own arrival)' if block else 'non-blocking'}"
                 f" + {'push-frame' if frame_source is not None else 'getter'} wait, "
                 f"expect {expected:.2f} s, timeout {timeout_s:.1f} s)")
        out["block"] = bool(block)
        ret = self.robot.rm_set_lift_height(int(speed_pct), mm, 1 if block else 0)
        out["ret"] = ret
        if block and ret == -4:
            # the vendor's own wording for this call: "the current arrival
            # device check failed, i.e. the arrival device is not the lift"
            self.log("   ** pole: rm_set_lift_height(block=1) returned -4 — the arrival "
                     "waiting on this handle belongs to another device. Something before "
                     "this left an unconsumed arrival; a blocking program start would "
                     "have eaten it instead.")
        if ret != 0:
            self.log(f"   pole: rm_set_lift_height REFUSED ret={ret}")
            return out
        t0 = time.monotonic()
        seen_moving = False
        pos = start
        f_prev = None            # previous pushed height [m]
        f_ok_since = None        # elapsed at which the pushed height entered the band
        while True:
            elapsed = time.monotonic() - t0
            # --- the 5 ms push, when the caller supplied it (B6) --------------
            fh = None
            if frame_source is not None:
                try:
                    fh = frame_source()
                except Exception as exc:                               # noqa: BLE001
                    self.log(f"   ** pole: frame source raised {exc!r} — falling back "
                             f"to the height getter for this move")
                    frame_source = None
            if fh is not None:
                fh = float(fh)
                if not out["frames"]:
                    out["frame_source"] = True
                    # The frames are the truth channel, so they, not the getter,
                    # decide whether this command was a real move at all — with
                    # ONE exception. `start` is read BEFORE the command is sent,
                    # so it is the previous SETPOINT and is NOT subject to the
                    # in-travel lead that B6 is about (the ledger's `start_m` is
                    # the old height on every one of the 67 real moves; the
                    # 73 mm lead appears only in the samples AFTER the command).
                    # If that pre-command reading says a real move is due while
                    # the very first push frame is already inside the band, one
                    # of the two channels is stale, and no off-target witness
                    # can ever be taken. Believing the frame there would turn a
                    # needed 75 mm move into a silent "already there" — which is
                    # the B5 mis-stage, arrived at through the B6 fix. Refuse.
                    if abs(fh - float(height_m)) <= tol_m:
                        if start is not None and not no_op:
                            out["channels_disagree"] = True
                            self.log(
                                f"   ** pole: the first push frame reads "
                                f"{fh * 1000:.0f} mm — inside the "
                                f"{tol_m * 1000:.0f} mm band of the {mm} mm target — "
                                f"while the height getter read {start * 1000:.0f} mm "
                                f"BEFORE the command. The two channels disagree, so no "
                                f"off-target witness is possible and this arrival is "
                                f"REFUSED rather than believed.")
                            break
                        out["no_op"], out["no_op_source"] = True, "frames"
                    elif out["no_op"]:
                        out["no_op"], out["no_op_source"] = False, "frames"
                out["frames"].append([round(elapsed, 3), round(fh * 1000, 1)])
                if abs(fh - float(height_m)) > tol_m:
                    out["witness"] = True
                    f_ok_since = None
                elif f_ok_since is None or (f_prev is not None
                                            and abs(fh - f_prev) > LIFT_FRAME_STEP_M):
                    f_ok_since = elapsed
                f_prev = fh
            # --- the getter, unchanged ---------------------------------------
            st = self.state()
            getter_arrived = False
            if st is not None:
                pos = float(st["pos"]) / LIFT_UNITS_PER_METRE
                # Newton, 2026-09-03: "the pole did nothing". before/after alone
                # cannot answer that — a setpoint echo and a moving pole have the
                # same endpoints. The SERIES can: a real lift ramps through the
                # intermediate millimetres, an echo jumps in one sample.
                out["series"].append([round(elapsed, 3), round(pos * 1000, 1)])
                mode = int(st.get("mode", 0) or 0)
                err = int(st.get("err_flag", 0) or 0)
                if err:
                    out["error"] = err
                    self.log(f"   pole: err_flag {err} — stopping the wait")
                    break
                if mode != 0:
                    seen_moving = True
                # When the caller PROMISED push frames and none have landed,
                # the 0.68 s timer must not sign a real move off on the getter
                # alone: that timer IS the B6 defect (70 of 70 hardware moves
                # exited on it, while true travel is 1.9-2.8 s). Hold the getter
                # to the modelled travel instead (L49) — unless the pole is
                # genuinely moving, which the mode word answers directly. A
                # no-op needs no such wait, and a caller that never asked for
                # frames keeps the proven behaviour unchanged.
                fallback_after = LIFT_START_LATENCY_S + 0.3
                if (out["frames_requested"] and not out["frames"]
                        and not out["no_op"]):
                    fallback_after = max(fallback_after, expected)
                getter_arrived = (mode == 0 and abs(pos - float(height_m)) <= tol_m
                                  and (seen_moving or elapsed > fallback_after))
            if out["frames"]:
                held = (f_ok_since is not None
                        and elapsed - f_ok_since >= LIFT_FRAME_STABLE_S)
                if held and (out["no_op"] or out["witness"]):
                    out["ok"] = True
                    out["verified_by"] = "no_op" if out["no_op"] else "frames"
                    break
            elif getter_arrived:
                out["ok"] = True
                out["verified_by"] = "no_op" if out["no_op"] else "getter"
                break
            if elapsed > timeout_s:
                self.log(f"   pole: TIMEOUT after {elapsed:.1f} s at "
                         f"{'?' if pos is None else f'{pos * 1000:.0f} mm'}"
                         + (f" (push frames say "
                            f"{'?' if f_prev is None else f'{f_prev * 1000:.0f} mm'})"
                            if out["frames"] else ""))
                if out["frames"] and out["witness"]:
                    self.log("   ** pole: the push frames witnessed the pole OFF target "
                             "and it never held the band — this is a pole that did not "
                             "arrive, not a slow read-back, and it is refused.")
                break
            self._sleep(self.poll_s)
        out["final_m"] = pos
        out["frames_final_m"] = f_prev
        out["frames_missing"] = bool(out["frames_requested"] and not out["frames"])
        if out["frames_missing"]:
            self.log("   ** pole: a push-frame source was supplied but never yielded a "
                     "height — this arrival was judged on the getter, the channel that "
                     "leads the pole by up to 73 mm (B6)")
        out["t_s"] = time.monotonic() - t0
        vals = [v for _, v in out["series"]]
        if vals:
            out["moved_mm"] = round(max(vals) - min(vals), 1)
            out["distinct"] = len(set(vals))
        if out["frames"]:
            fvals = [v for _, v in out["frames"]]
            out["frames_moved_mm"] = round(max(fvals) - min(fvals), 1)
            out["frames_distinct"] = len(set(fvals))
        if out["ok"]:
            where = f_prev if out["frames"] else pos
            self.log(f"   pole: at {where * 1000:.0f} mm after {out['t_s']:.2f} s "
                     f"(verified_by={out['verified_by']}"
                     + (f", {out['frames_distinct']} distinct pushed heights"
                        if out["frames"] else
                        f", moved {out['moved_mm']:.0f} mm through {out['distinct']} "
                        f"distinct readings") + ")")
        # Only meaningful on the non-blocking GETTER path: with block=1 the call
        # returns after the motion, so our own polling starts at the far end
        # and one distinct reading is CORRECT, not a symptom. With push frames
        # the question is answered properly and this heuristic is not needed.
        if (not block and not out["frames"] and out["distinct"] <= 2
                and abs((out["target_m"] or 0) - (start or 0)) > tol_m):
            self.log("   ** pole: the height readback showed no intermediate values for a "
                     "commanded move — either it did not move, or the readback is a "
                     "setpoint echo. Check the pole physically.")
        return out


class HandModbus:
    """The Inspire hand over the end-port Modbus, exactly the driver's path."""

    def __init__(self, robot, side: str, log: Callable[[str], None] = print,
                 sleep: Callable[[float], None] = time.sleep,
                 device: int = HAND_MODBUS_DEVICE, baud: int = HAND_MODBUS_BAUD,
                 poll_s: float = 0.05):
        self.robot = robot
        self.side = side
        self.log = log
        self._sleep = sleep
        self.device = int(device)
        self.baud = int(baud)
        self.poll_s = poll_s
        self.opened = False
        self.speed_set: Optional[int] = None
        #: The report of the most recent register write (M42) — which channels
        #: landed. `command()` copies it into its own result as `write`.
        self.last_write: Optional[dict] = None
        self._units = _load_hand_units()

    # -- session ----------------------------------------------------------
    def open(self, speed_set: int = HAND_SPEED_SET_MAX) -> int:
        ret = self.robot.rm_set_modbus_mode(HAND_MODBUS_PORT, self.baud, 2)
        if ret != 0:
            self.log(f"   hand: rm_set_modbus_mode(port {HAND_MODBUS_PORT}, {self.baud}) "
                     f"FAILED ret={ret} — no hand control")
            return ret
        self.opened = True
        self.log(f"   hand: end-port modbus mode on (baud {self.baud}, device {self.device})")
        return self.set_speed(speed_set)

    def close(self) -> int:
        if not self.opened:
            return 0
        self.opened = False
        return self.robot.rm_close_modbus_mode(HAND_MODBUS_PORT)

    # -- registers --------------------------------------------------------
    def write_report(self, address: int, values: List[int]) -> dict:
        """Write *values* and say exactly WHICH channels landed (M42).

        The old fallback loop returned on the FIRST failing register, so a
        write that got three of six channels down the wire came back as a bare
        non-zero and the caller had no way to know the hand was left
        half-commanded — an ambiguity the ledger could never resolve either,
        because 248 of 248 device commands returned 0 and no refusal branch has
        ever run. This attempts EVERY channel, keeps every return code, and
        reports the landed set. The bulk write is still tried first (~63 ms for
        the six, hand_channel.cpp), and only its failure opens the per-register
        path (hand_channel.cpp:166-215: byte-addressed, two bytes per DOF).

        Returns {'ok', 'ret', 'mode', 'bulk_ret', 'landed', 'rets', 'address',
                 'values', 'partial'} where ``landed[i]`` is True only for a
        channel some call CONFIRMED with a 0. After a failed bulk write the
        bulk call may itself have put some channels down the wire; that is
        unknowable, so ``landed`` reflects the per-register retry only and
        ``bulk_ret`` is carried so a reader can see the ambiguity.
        """
        vals = [int(v) for v in values]
        n = len(vals)
        out = dict(ok=False, ret=None, mode="bulk", bulk_ret=None,
                   landed=[False] * n, rets=[None] * n, address=int(address),
                   values=list(vals), partial=False, error=None)
        try:
            ret = int(self.robot.rm_write_registers(_params(address, n, self.device),
                                                    encode_hi_lo(vals)))
        except Exception as exc:                                       # noqa: BLE001
            out["error"] = repr(exc)
            ret = -1
        out["bulk_ret"] = out["ret"] = ret
        if ret == 0:
            out.update(ok=True, landed=[True] * n, rets=[0] * n)
            return out
        out["mode"] = "per_register"
        worst = ret
        for ch, v in enumerate(vals):
            try:
                r = int(self.robot.rm_write_single_register(
                    _params(address + 2 * ch, 1, self.device), v))
            except Exception as exc:                                   # noqa: BLE001
                out["error"] = out["error"] or repr(exc)
                r = -1
            out["rets"][ch] = r
            out["landed"][ch] = (r == 0)
            if r != 0:
                worst = r
        out["ret"] = 0 if all(out["landed"]) else worst
        out["ok"] = all(out["landed"])
        out["partial"] = any(out["landed"]) and not out["ok"]
        if not out["ok"]:
            got = [i + 1 for i, ok in enumerate(out["landed"]) if ok]
            self.log(f"   ** hand: register write to {address} landed on channel(s) "
                     f"{got or 'none'} of {n} (bulk ret {out['bulk_ret']}, per-register "
                     f"{out['rets']}) — the hand is "
                     f"{'HALF-WRITTEN' if out['partial'] else 'unchanged'}")
        return out

    def _write(self, address: int, values: List[int]) -> int:
        """Backwards-compatible int return; see :meth:`write_report`."""
        self.last_write = self.write_report(address, values)
        return self.last_write["ret"]

    def set_speed(self, speed_set: int) -> int:
        s = max(1, min(HAND_SPEED_SET_MAX, int(speed_set)))
        ret = self._write(HAND_REG_SPEED_SET, [s] * HAND_DOF)
        if ret == 0:
            self.speed_set = s
            self.log(f"   hand: SPEED_SET {s} on all six channels")
        else:
            self.log(f"   hand: SPEED_SET write FAILED ret={ret}")
        return ret

    def read(self) -> Optional[List[int]]:
        return self._read_reg(HAND_REG_ANGLE_ACT)

    def read_set(self) -> Optional[List[int]]:
        """The six ANGLE_SET values the hand is currently holding — what it was
        last TOLD, beside `read()`'s where it actually IS. Recorded whenever a
        command holds a channel, so a disagreement between the two is visible
        (the right hand's thumb channels read 506/89 actual against a 981/992
        setpoint on 2026-09-15)."""
        return self._read_reg(HAND_REG_ANGLE_SET)

    def _read_reg(self, address: int) -> Optional[List[int]]:
        try:
            ret, data = self.robot.rm_read_multiple_holding_registers(
                _params(address, HAND_DOF, self.device))
        except Exception:                                              # noqa: BLE001
            return None
        if ret != 0 or not data or len(data) < 2 * HAND_DOF:
            return None
        return decode_hi_lo(list(data)[:2 * HAND_DOF])

    def stroke_s(self, target: List[int], current: Optional[List[int]]) -> float:
        span = (max(abs(int(a) - int(b)) for a, b in zip(target, current))
                if current else 1000)
        return HAND_LATENCY_S + HAND_STROKE_K * span / max(self.speed_set or 1, 1) / 1000.0

    def command(self, counts: List[int], wait: bool = True,
                timeout_s: Optional[float] = None) -> dict:
        """ANGLE_SET *counts* (six, 0-1000) and wait until SETTLED."""
        if not self.opened:
            raise RuntimeError("hand: open() first (modbus mode)")
        if len(counts) != HAND_DOF:
            raise ValueError(f"hand: need {HAND_DOF} channel values, got {len(counts)}")
        # A None IS AN INSTRUCTION: hold this channel where it is. The sheets
        # command "the first 4 fingers" and say nothing about the thumb (Newton,
        # 2026-09-16), and the protocol writes all six registers at once — so a
        # held channel is written with its OWN current position, which commands
        # no motion and cannot be read later as a channel that failed to move.
        # Refuses rather than guess when the hand cannot be read: there is no
        # value that means "unchanged" in this register.
        held = [ch for ch, v in enumerate(counts) if v is None]
        before = self.read()
        set_before = self.read_set() if held else None
        if held and before is None:
            self.log(f"   ** hand: channel(s) {[c + 1 for c in held]} are to be HELD, but the "
                     f"hand's position does not read back — refusing to write a guess")
            return dict(ok=False, ret=None, target=None, before=None, after=None,
                        settled=False, arrived=False, t_s=0.0, law_s=0.0, timeout_s=0.0,
                        series=[], moved=[0] * HAND_DOF, stuck=[], miss=None, write=None,
                        partial=False, held=[c + 1 for c in held], set_before=set_before,
                        error="hand unreadable, channels to hold")
        target = [max(0, min(1000, int(before[ch] if v is None else v)))
                  for ch, v in enumerate(counts)]
        law = self.stroke_s(target, before)
        if timeout_s is None:
            timeout_s = 3.0 * law + 1.0
        out = dict(ok=False, ret=None, target=target, before=before, after=None,
                   settled=False, arrived=False, t_s=0.0, law_s=law, timeout_s=timeout_s,
                   series=[], moved=[0] * HAND_DOF, stuck=[], miss=None,
                   write=None, partial=False, held=[c + 1 for c in held],
                   set_before=set_before, asked=list(counts))
        hold_note = (f"; channel(s) {[c + 1 for c in held]} HELD where they are"
                     + (f" (their setpoint reads {[set_before[c] for c in held]})"
                        if set_before else "")) if held else ""
        self.log(f"   hand: ANGLE_SET {target} (from {before}; law {law:.2f} s){hold_note}")
        wr = self.write_report(HAND_REG_ANGLE_SET, target)
        out["write"], out["ret"], out["partial"] = wr, wr["ret"], wr["partial"]
        if not wr["ok"]:
            # M42: say WHICH channels landed. A half-written hand is a distinct
            # state from an unchanged one and the caller has to be able to tell
            # them apart to decide whether it may still move the pole.
            out["after"] = self.read()
            landed = [i + 1 for i, k in enumerate(wr["landed"]) if k]
            what = (f"HALF-WRITTEN — channels {landed} landed" if wr["partial"]
                    else "nothing landed")
            self.log(f"   hand: ANGLE_SET write FAILED ret={wr['ret']} ({what})")
            return out
        if not wait:
            out["ok"] = True
            return out
        t0 = time.monotonic()
        prev = None
        cur = before
        while True:
            self._sleep(self.poll_s)
            cur = self.read()
            elapsed = time.monotonic() - t0
            if cur is not None:
                # Newton, 2026-09-03: "the hand did nothing". Keep the whole
                # readback series, not just the endpoints — a channel that is
                # echoing arrives on the FIRST poll, a channel that is moving
                # walks there, and a dead channel never leaves its value.
                out["series"].append([round(elapsed, 3), list(cur)])
                arrived = all(abs(a - b) <= HAND_ARRIVE_COUNTS for a, b in zip(cur, target))
                settled = (prev is not None and elapsed >= law
                           and all(abs(a - b) <= HAND_SETTLE_COUNTS for a, b in zip(cur, prev)))
                if arrived or settled:
                    out.update(ok=True, arrived=arrived, settled=settled)
                    break
                prev = cur
            elif elapsed >= law and prev is None and before is None:
                # no ANGLE_ACT available at all (no feedback path): trust the law
                out.update(ok=True, settled=True)
                break
            if elapsed > timeout_s:
                self.log(f"   hand: TIMEOUT after {elapsed:.1f} s at {cur}")
                break
        out["after"] = cur
        out["t_s"] = time.monotonic() - t0
        # Per-channel accounting, so "the hand did nothing" is answerable and a
        # dead channel cannot hide behind a settled verdict on the other five.
        if out["series"]:
            vals = [v for _, v in out["series"]]
            for ch in range(HAND_DOF):
                col = [v[ch] for v in vals] + ([before[ch]] if before else [])
                out["moved"][ch] = max(col) - min(col)
            want_move = [ch for ch in range(HAND_DOF)
                         if before and ch not in held
                         and abs(target[ch] - before[ch]) > HAND_DEADBAND_COUNTS]
            out["stuck"] = [ch + 1 for ch in want_move
                            if out["moved"][ch] <= HAND_SETTLE_COUNTS]
        if cur:
            out["miss"] = [abs(a - b) for a, b in zip(cur, target)]
        if out["ok"]:
            self.log(f"   hand: {'arrived' if out['arrived'] else 'settled'} at {cur} "
                     f"after {out['t_s']:.2f} s (per-channel travel {out['moved']})")
        if out["stuck"]:
            self.log(f"   ** hand: channel(s) {out['stuck']} were commanded to move and "
                     f"did NOT — target {target}, readback {cur}. Either that channel is "
                     f"dead or the readback is not the device.")
        elif out["ok"] and not out["arrived"] and out["miss"]:
            worst = max(range(HAND_DOF), key=lambda i: out["miss"][i])
            self.log(f"   ** hand: settled without arriving — worst channel "
                     f"{worst + 1} is {out['miss'][worst]} counts short "
                     f"(target {target[worst]}, readback {cur[worst]})")
        return out

    def freeze(self) -> dict:
        """Stop the hand where it is: ANGLE_SET := the MEASURED ANGLE_ACT (M41).

        The SDK has no hand stop — ``rm_connection.cpp:330-...`` says so and
        does exactly this, and ``rm_side_system.cpp:766-780`` issues it
        alongside the arm stop and the pole stop. Commanding the hand to the
        angle it is CURRENTLY at is the closest thing to a freeze the device
        offers. Never raises, and never switches controller modes: if this
        handle has no open modbus session the freeze is REPORTED as not issued
        rather than silently changing the port mode inside a stop path.
        """
        out = dict(ok=False, issued=False, ret=None, counts=None, write=None,
                   why=None)
        if not self.opened:
            out["why"] = ("no end-port modbus session on this handle — a stop must "
                          "not switch controller modes, so the hand is NOT frozen")
            self.log(f"   ** hand: {out['why']}")
            return out
        counts = self.read()
        out["counts"] = counts
        if counts is None:
            out["why"] = ("ANGLE_ACT unreadable — there is no measured angle to "
                          "freeze the hand at, so nothing was commanded")
            self.log(f"   ** hand: {out['why']}")
            return out
        wr = self.write_report(HAND_REG_ANGLE_SET, counts)
        out.update(issued=True, write=wr, ret=wr["ret"], ok=wr["ok"])
        self.log(f"   hand: FREEZE at the measured ANGLE_ACT {counts} "
                 f"(ret={wr['ret']})")
        if not wr["ok"]:
            out["why"] = f"ANGLE_SET freeze write failed (ret={wr['ret']})"
        return out

    # -- units ------------------------------------------------------------
    def counts_from_rad(self, joints_rad: Dict[str, float]) -> List[int]:
        return counts_from_rad(joints_rad, self.side, self._units)


# ------------------------------------------------------------------ the plan
def counts_from_rad(joints_rad: Dict[str, float], side: str, units=None) -> List[Optional[int]]:
    """Six ANGLE_SET counts from a {joint_name: rad} map (hand_units order).

    A THUMB channel the state does not name is None (hold): an exclude_thumb
    plan has no thumb joints, and a count invented for it would move a thumb
    the plan never modelled. A missing FINGER is still a KeyError.
    """
    units = units or _load_hand_units()
    if units is None:
        raise RuntimeError("hand_units.py not found — refusing to invent a hand scale (C16.4)")
    out: List[Optional[int]] = []
    for ch in range(HAND_DOF):
        name = units.joint_name(side, ch)
        if name not in joints_rad and ch in HAND_THUMB_CHANNELS:
            out.append(None)
            continue
        if name not in joints_rad:
            raise KeyError(f"hand state is missing {name!r}; cannot command channel {ch}")
        out.append(int(units.rad_to_hw(float(joints_rad[name]), ch)))
    return out


def _stages(plan: dict) -> list:
    return plan.get("sub_trajectories") or plan.get("stages") or []


def _last_positions(stage: dict) -> Dict[str, float]:
    wp = stage.get("waypoints") or []
    if not wp:
        return {}
    return dict(zip(stage.get("joint_names", []), wp[-1]["positions"]))


def _is_pole(stage: dict) -> bool:
    return any("sliding_plate" in n for n in stage.get("joint_names", []))


def _is_hand(stage: dict) -> bool:
    names = stage.get("joint_names", [])
    return bool(names) and all(("IRH_" in n or "ILH_" in n) for n in names)


def _is_arm(stage: dict) -> bool:
    names = stage.get("joint_names", [])
    return len(names) == 7 and all(n[1:] == f"_joint{i + 1}" for i, n in enumerate(names))


def pre_stroke_state(plan: dict, side: str, units=None) -> dict:
    """What the plan had the devices at when execute_path started.

    Returns {'pole_m', 'pole_stage', 'hand_state', 'hand_counts',
             'hand_release_counts', 'execute_index'}; device keys are None
    when the plan has no such stage before execute_path."""
    stages = _stages(plan)
    ex = next((i for i, s in enumerate(stages)
               if (s.get("stage_name") or s.get("name")) == "execute_path"), None)
    out = dict(pole_m=None, pole_stage=None, hand_state=None, hand_counts=None,
               hand_release_counts=None, execute_index=ex)
    if ex is None:
        return out
    for s in stages[:ex]:
        nm = s.get("stage_name") or s.get("name")
        if _is_pole(s):
            pos = _last_positions(s)
            val = next((v for k, v in pos.items() if "sliding_plate" in k), None)
            if val is not None:
                out["pole_m"] = round(float(val), 4)
                out["pole_stage"] = nm
        elif _is_hand(s):
            pos = _last_positions(s)
            if pos:
                out["hand_state"] = nm
                out["hand_counts"] = counts_from_rad(pos, side, units)
    # BEFORE the stroke only. This used to scan every stage, so a plan whose
    # only `hand_release` is its post-stroke tear-down (slots 72 and 80) had its
    # release pose read from AFTER the wipe — contradicting this function's own
    # docstring. The pole_m and hand_counts loops above already iterate
    # stages[:ex]; this now matches them.
    rel = next((s for s in stages[:ex] if (s.get("stage_name") or s.get("name")) == "hand_release"
                and _is_hand(s) and _last_positions(s)), None)
    out["hand_release_counts"] = (counts_from_rad(_last_positions(rel), side, units)
                                  if rel is not None else None)
    out["hand_release_source"] = ("plan" if rel is not None else None)
    return out


def rest_pose(plan: dict) -> Optional[dict]:
    """The plan's own `move_to_rest` target: {'stage', 'deg': [7]} or None.

    Newton, 2026-09-04: every task replay should end by parking the arm at rest
    rather than leaving it wherever the stroke finished. The plans already
    carry that pose — the stage order is
    ... execute_path, move_to_pre_start, move_to_rest, ... — so it is read from
    the plan like the retreat, never invented here.
    """
    stages = _stages(plan)
    ex = next((i for i, s in enumerate(stages)
               if (s.get("stage_name") or s.get("name")) == "execute_path"), None)
    search = stages[ex + 1:] if ex is not None else stages
    for s in search:
        if (s.get("stage_name") or s.get("name")) == "move_to_rest" and s.get("waypoints"):
            q = s["waypoints"][-1]["positions"]
            return dict(stage="move_to_rest",
                        deg=[round(math.degrees(float(v)), 4) for v in q])
    return None


def retreat_pose(plan: dict) -> Optional[dict]:
    """The plan's own first arm move AFTER execute_path (its way out of the
    fixture): {'stage', 'deg': [7]} or None if the stroke is the last arm stage."""
    stages = _stages(plan)
    ex = next((i for i, s in enumerate(stages)
               if (s.get("stage_name") or s.get("name")) == "execute_path"), None)
    if ex is None:
        return None
    for s in stages[ex + 1:]:
        if _is_arm(s) and s.get("waypoints"):
            q = s["waypoints"][-1]["positions"]
            return dict(stage=s.get("stage_name") or s.get("name"),
                        deg=[round(math.degrees(float(v)), 4) for v in q])
    return None
